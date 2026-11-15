/*
 XIAO nRF52840 Sense + MAX4466 A0 (P0.02) + onboard LSM6DS3TR-C
 Seeed nRF52 Boards (non-mbed, Adafruit Bluefruit SoftDevice core)
 Seeed_Arduino_LSM6DS3 and core-provided Adafruit_TinyUSB.h

 Compatible with previous C3 HTML protocol:
  M,1,count,flags,seq(u32),timestampEpochUs(u64),uint16 ADC samples
  I,1,count,flags,seq(u32),timestampEpochUs(u64),count*(6*int16)
  S,status(1 start/2 finish/3 error),MTU(u16),dropsMic(u32),dropsIMU(u32)
 Commands 'T'+little-endian uint64 epoch_ms, 'B','E','R'

 Microphone: SAADC internal timer at 8000 Hz, EasyDMA double buffered.
 No nrfx_saadc library linkage required on Seeeduino nRF52 core 1.1.13.
 IMU: polled at nominal 200 Hz from onboard 208 Hz ODR, so some
 repeated/skipped values possible. Sequence gaps are counted.
 BLE notifications are scheduled in ~50 ms bursts.
 IMPORTANT: Sequence checks cannot count any conversions dropped by SAADC
 driver/hardware outside the completed buffers. No retransmission.
*/
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <Wire.h>
#include <LSM6DS3.h>
#include <nrf.h>
#include <stdint.h>

static constexpr char DEV_NAME[]="XIAO-IMU-MIC";
static constexpr char SERVICE_UUID[]="ee8e0001-7d9d-4f38-85bd-a703a196e710";
static constexpr char DATA_UUID[]="ee8e0002-7d9d-4f38-85bd-a703a196e710";
static constexpr char CMD_UUID[]="ee8e0003-7d9d-4f38-85bd-a703a196e710";
static constexpr uint32_t MIC_HZ=8000, IMU_US=5000, BURST_MS=50;
static constexpr uint16_t MIC_CAP=8192, IMU_CAP=256;
static constexpr uint8_t MAX_MIC=108, MAX_IMU=10;
static constexpr uint16_t SAADC_BUFFER=64;

BLEService service(SERVICE_UUID);
BLECharacteristic dataChr(DATA_UUID), cmdChr(CMD_UUID);
LSM6DS3 imu(I2C_MODE,0x6A);
static int16_t adcBufferA[SAADC_BUFFER];
static int16_t adcBufferB[SAADC_BUFFER];

struct Mic { uint32_t seq; uint16_t raw; };
struct Imu { uint32_t seq; uint32_t tick; int16_t v[6]; };
static Mic micQ[MIC_CAP];
static Imu imuQ[IMU_CAP];
static volatile uint16_t mHead=0,mTail=0,mCount=0,iHead=0,iTail=0,iCount=0;
static volatile uint32_t mSeq=0,iSeq=0,mDrop=0,iDrop=0;
static volatile bool recording=false, audioEnabled=false, connected=false;
static volatile bool adcFault=false;
static volatile char command=0;
static uint32_t audioStartUs=0, imuNextUs=0, nextBurstMs=0;
static uint64_t clockEpochUs=0;
static uint32_t clockMicros=0;
static volatile bool clockSet=false;
static bool draining=false, sentFinish=false, burst=false;
static uint8_t burstPackets=0;
static uint32_t stopAtMs=0;

static void p16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void p32(uint8_t*p,uint32_t v){for(int i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static void p64(uint8_t*p,uint64_t v){for(int i=0;i<8;i++)p[i]=(uint8_t)(v>>(8*i));}
static uint64_t g64(const uint8_t*p){uint64_t a=0;for(int i=0;i<8;i++)a|=((uint64_t)p[i])<<(8*i);return a;}
static uint64_t epochAt(uint32_t bootUs){return clockEpochUs+(int32_t)(bootUs-clockMicros);}
static int16_t s16(float x){if(x>32767)x=32767;if(x< -32768)x=-32768;return (int16_t)lroundf(x);}

// Direct nRF52840 peripheral implementation: no disabled nrfx driver to link.
// SAADC TIMER mode: 16 MHz / 2000 = 8 kHz. DMA blocks contain 64 samples.
// At a completed block, the ISR copies samples and rearms the next DMA block.
// A short block-boundary gap may occur; verify the actual sample rate with
// a known test tone before using the recording for quantitative measurements.
static volatile uint8_t activeAdcBuffer=0;

extern "C" void SAADC_IRQHandler(void) {
  if (!NRF_SAADC->EVENTS_END) return;
  NRF_SAADC->EVENTS_END=0;
  (void)NRF_SAADC->EVENTS_END;

  const uint8_t completed = activeAdcBuffer;
  int16_t* samples = completed ? adcBufferB : adcBufferA;
  activeAdcBuffer ^= 1;

  // Rearm immediately, before doing queue processing.
  NRF_SAADC->RESULT.PTR = (uint32_t)(activeAdcBuffer ? adcBufferB : adcBufferA);
  NRF_SAADC->RESULT.MAXCNT = SAADC_BUFFER;
  NRF_SAADC->EVENTS_STARTED = 0;
  NRF_SAADC->TASKS_START = 1;

  if (!recording) return;
  for (uint16_t j=0; j<SAADC_BUFFER; ++j) {
    int32_t x=samples[j];
    if(x<0)x=0;
    if(x>4095)x=4095;
    uint32_t seq=mSeq++;
    if(mCount<MIC_CAP) {
      micQ[mHead]={seq,(uint16_t)x};
      mHead=(mHead+1)%MIC_CAP;
      ++mCount;
    } else ++mDrop;
  }
}

static bool initAudio() {
  // Nothing else should use analogRead()/SAADC while this recorder runs.
  NVIC_DisableIRQ(SAADC_IRQn);
  NRF_SAADC->INTENCLR=0xFFFFFFFFUL;
  NRF_SAADC->TASKS_STOP=1;
  NRF_SAADC->ENABLE=0;

  for(int i=0;i<8;i++) {
    NRF_SAADC->CH[i].PSELP=SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[i].PSELN=SAADC_CH_PSELN_PSELN_NC;
  }
  NRF_SAADC->CH[0].PSELP=SAADC_CH_PSELP_PSELP_AnalogInput0; // XIAO A0 -> P0.02
  NRF_SAADC->CH[0].PSELN=SAADC_CH_PSELN_PSELN_NC;
  NRF_SAADC->CH[0].CONFIG =
      (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
      (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
      (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
      (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
      (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) |
      (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
      (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);

  NRF_SAADC->RESOLUTION=SAADC_RESOLUTION_VAL_12bit;
  NRF_SAADC->OVERSAMPLE=SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
  NRF_SAADC->SAMPLERATE =
      (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos) |
      ((uint32_t)(16000000UL/MIC_HZ) << SAADC_SAMPLERATE_CC_Pos);
  NRF_SAADC->RESULT.PTR=(uint32_t)adcBufferA;
  NRF_SAADC->RESULT.MAXCNT=SAADC_BUFFER;
  activeAdcBuffer=0;
  NRF_SAADC->EVENTS_END=0;
  NRF_SAADC->EVENTS_STARTED=0;
  NRF_SAADC->ENABLE=SAADC_ENABLE_ENABLE_Enabled;
  NRF_SAADC->INTENSET=SAADC_INTENSET_END_Msk;
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  NVIC_SetPriority(SAADC_IRQn,6); // SoftDevice-safe application interrupt priority
  NVIC_EnableIRQ(SAADC_IRQn);
  NRF_SAADC->TASKS_START=1;
  return true;
}
static void startAudio(){
 // Hardware ADC sampling is always running. Recording only gates RAM writes.
 audioStartUs=micros();
 recording=true;audioEnabled=true;
}
static void stopAudio(){
 audioEnabled=false;recording=false;
 // Keep the ADC driver running so the next recording does not require a restart.
 // The final partial 64-sample buffer is not recorded; up to 63 samples
 // can be excluded at start/stop boundaries.
}
static uint16_t mtu(){
 BLEConnection* c=Bluefruit.Connection(Bluefruit.connHandle());
 return c?c->getMtu():23;
}
static bool statusPacket(uint8_t code){
 if(!connected || !dataChr.notifyEnabled())return false;
 uint8_t p[12]={'S',code,0};p16(p+2,mtu());
 noInterrupts();uint32_t a=mDrop,b=iDrop;interrupts();
 p32(p+4,a);p32(p+8,b);return dataChr.notify(p,sizeof(p));
}
static void cmdWrite(uint16_t, BLECharacteristic*,uint8_t* p,uint16_t len){
 if(!len)return;
 if(p[0]=='T' && len==9){
   noInterrupts(); clockEpochUs=g64(p+1)*1000ULL;clockMicros=micros();clockSet=true;interrupts();
 }else if(p[0]=='B'||p[0]=='E'||p[0]=='R')command=(char)p[0];
}
static void connectCB(uint16_t){connected=true;Serial.println("BLE connected");}
static void disconnectCB(uint16_t,uint8_t){
 connected=false;
 if(recording)stopAudio();
 draining=false;burst=false;command=0;
 Bluefruit.Advertising.start(0);
 Serial.println("BLE disconnected");
}
static bool sendMic(){
 int space=(int)mtu()-3-16;
 if(space<2)return false;
 uint8_t limit=(uint8_t)min((int)MAX_MIC,space/2);
 Mic batch[MAX_MIC];uint8_t n=0;
 noInterrupts();
 uint16_t idx=mTail,available=mCount;
 while(n<limit && n<available){Mic s=micQ[idx];if(n && s.seq!=batch[n-1].seq+1)break;batch[n++]=s;idx=(idx+1)%MIC_CAP;}
 interrupts();
 if(!n)return false;
 uint8_t p[16+MAX_MIC*2];p[0]='M';p[1]=1;p[2]=n;p[3]=0;
 p32(p+4,batch[0].seq);
 uint32_t sampleUs=audioStartUs+(uint32_t)(((uint64_t)batch[0].seq*1000000ULL)/MIC_HZ);
 p64(p+8,epochAt(sampleUs));
 for(uint8_t j=0;j<n;j++)p16(p+16+2*j,batch[j].raw);
 if(!dataChr.notify(p,16+2*n))return false;
 noInterrupts();mTail=(mTail+n)%MIC_CAP;mCount-=n;interrupts();return true;
}
static bool sendImu(){
 int space=(int)mtu()-3-16;
 if(space<12)return false;
 uint8_t limit=(uint8_t)min((int)MAX_IMU,space/12);
 Imu batch[MAX_IMU];uint8_t n=0;
 noInterrupts();
 uint16_t idx=iTail,available=iCount;
 while(n<limit && n<available){Imu s=imuQ[idx];if(n && s.seq!=batch[n-1].seq+1)break;batch[n++]=s;idx=(idx+1)%IMU_CAP;}
 interrupts();
 if(!n)return false;
 uint8_t p[16+MAX_IMU*12];p[0]='I';p[1]=1;p[2]=n;p[3]=0;
 p32(p+4,batch[0].seq);p64(p+8,epochAt(batch[0].tick));
 for(uint8_t j=0;j<n;j++)for(uint8_t k=0;k<6;k++)p16(p+16+j*12+k*2,(uint16_t)batch[j].v[k]);
 if(!dataChr.notify(p,16+12*n))return false;
 noInterrupts();iTail=(iTail+n)%IMU_CAP;iCount-=n;interrupts();return true;
}
static void readImu(){
 uint32_t now=micros();
 if((int32_t)(now-imuNextUs)<0)return;
 uint32_t missed=(uint32_t)(now-imuNextUs)/IMU_US;
 imuNextUs+=(missed+1)*IMU_US;
 iSeq+=missed;
 Imu s={};s.seq=iSeq++;s.tick=micros();
 s.v[0]=s16(imu.readFloatAccelX()*16384.0f);
 s.v[1]=s16(imu.readFloatAccelY()*16384.0f);
 s.v[2]=s16(imu.readFloatAccelZ()*16384.0f);
 s.v[3]=s16(imu.readFloatGyroX()*131.0f);
 s.v[4]=s16(imu.readFloatGyroY()*131.0f);
 s.v[5]=s16(imu.readFloatGyroZ()*131.0f);
 noInterrupts();
 if(iCount<IMU_CAP){imuQ[iHead]=s;iHead=(iHead+1)%IMU_CAP;++iCount;}else ++iDrop;
 interrupts();
}
static void beginRecord(){
 if(!connected || !clockSet || mtu()<100 || !dataChr.notifyEnabled()){
   Serial.println("Start rejected: need BLE, time sync, notifications, and MTU >= 100");statusPacket(3);return;
 }
 stopAudio();
 noInterrupts();
 mHead=mTail=mCount=iHead=iTail=iCount=0;
 mSeq=iSeq=mDrop=iDrop=0;
 interrupts();
 adcFault=false;draining=false;sentFinish=false;burst=false;
 imuNextUs=micros();nextBurstMs=millis()+BURST_MS;
 startAudio();statusPacket(1);
 Serial.println("Recording started: 8k SAADC peripheral timer + 200Hz IMU");
}
void setup(){
 Serial.begin(115200);
 delay(600);
 Serial.println("XIAO nRF52840 Sense 8k/50ms boot");
 // Seeed IMU library routes internal Wire1 where required.
 imu.settings.accelRange=2;
 imu.settings.gyroRange=245;
 imu.settings.accelSampleRate=208;
 imu.settings.gyroSampleRate=208;
 if(imu.begin()!=0){Serial.println("ERROR: onboard LSM6DS3TR-C not found");while(true)delay(1000);}
 Serial.println("IMU OK 0x6A");
 if(!initAudio()){Serial.println("ERROR: SAADC setup failed");while(true)delay(1000);}
 Serial.println("SAADC direct-register timer ready 8000 Hz");
 Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
 Bluefruit.begin();Bluefruit.setTxPower(4);Bluefruit.setName(DEV_NAME);
 Bluefruit.Periph.setConnectCallback(connectCB);
 Bluefruit.Periph.setDisconnectCallback(disconnectCB);
 Bluefruit.Periph.setConnInterval(12,24); // 15-30ms (host chooses actual)
 service.begin();
 dataChr.setProperties(CHR_PROPS_NOTIFY);dataChr.setPermission(SECMODE_OPEN,SECMODE_NO_ACCESS);
 dataChr.setMaxLen(244);dataChr.begin();
 cmdChr.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
 cmdChr.setPermission(SECMODE_NO_ACCESS,SECMODE_OPEN);cmdChr.setMaxLen(20);
 cmdChr.setWriteCallback(cmdWrite);cmdChr.begin();
 Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
 Bluefruit.Advertising.addTxPower();Bluefruit.Advertising.addService(service);
 Bluefruit.ScanResponse.addName();
 Bluefruit.Advertising.restartOnDisconnect(true);
 Bluefruit.Advertising.setInterval(32,244);
 Bluefruit.Advertising.setFastTimeout(30);
 Bluefruit.Advertising.start(0);
 Serial.println("BLE ready: XIAO-IMU-MIC | A0 MAX4466 | onboard IMU | 8kHz/50ms");
}
void loop(){
 char c=command;if(c)command=0;
 if(c=='B')beginRecord();
 else if(c=='E'){
   if(recording){stopAudio();draining=true;stopAtMs=millis();burst=true;burstPackets=0;}
 }else if(c=='R')statusPacket(recording?1:2);
 if(recording)readImu();
 if(!connected){delay(2);return;}
 uint32_t now=millis();
 if(recording && (int32_t)(now-nextBurstMs)>=0){
   burst=true;burstPackets=0;
   nextBurstMs+=BURST_MS;
   if((int32_t)(now-nextBurstMs)>=0)nextBurstMs=now+BURST_MS;
 }
 bool did=false;
 if(burst && burstPackets<12){
   if(burstPackets%4==3){did=sendImu();if(!did)did=sendMic();}
   else {did=sendMic();if(!did)did=sendImu();}
   if(did)++burstPackets;
   else burst=false;
   if(burstPackets>=12)burst=false;
 }
 if(draining){
   burst=true;
   if(!did && burstPackets>=12)burstPackets=0;
   uint16_t a,b;noInterrupts();a=mCount;b=iCount;interrupts();
   if(!a && !b){
     if(statusPacket(adcFault?3:2)){draining=false;sentFinish=true;burst=false;Serial.println("Recording transferred");}
   }else if(millis()-stopAtMs>10000){
     statusPacket(3);draining=false;burst=false;
     Serial.println("BLE drain timeout: insufficient throughput");
   }
 }
 delay(did?1:0);
}
