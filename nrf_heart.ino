/*
  nrf_heart.ino - Seeed Studio XIAO nRF52840 Sense (or compatible nRF52840 Sense)
  Streams timestamped IMU (LSM6DS3, 416 Hz) and PDM microphone (8000 Hz, ADPCM)
  over BLE to a Web Bluetooth client (Android/PC).

  Board/package (Arduino IDE):
    - "Seeed nRF52 mbed-enabled Boards" -> Seeed XIAO nRF52840 Sense
      (or adapt includes for "Seeed nRF52 Boards"/non-mbed core as noted below)

  Libraries:
    - ArduinoBLE
    - Seeed Arduino LSM6DS3 (or "LSM6DS3" by SparkFun/Arduino_LSM6DS3 depending on core)
    - PDM (built-in for mbed nRF52 core)

  Notes:
    - 416 Hz is not a native LSM6DS3 ODR; we configure 416 Hz if supported by the
      library/core, otherwise fall back to the closest available rate (e.g. 208/833 Hz).
    - Web Bluetooth requires user gesture to connect and HTTPS/localhost for the page.
    - BLE throughput is limited; packets are batched to balance loss vs battery.
*/

#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <PDM.h>
#include <Wire.h>

// -------------------- Configuration --------------------
#define DEVICE_NAME           "NRF_HEART_SENSE"

// Sampling
#define IMU_HZ                416
#define MIC_HZ                8000
#define MIC_CHANNELS          1

// Packet sizing (keep under typical ATT MTU payloads after overhead; Web BT usually copes
// with ~180-244 byte notifications, but smaller is safer across stacks)
#define IMU_SAMPLES_PER_PKT   8    // 8 samples * 14 bytes = 112 bytes payload
#define ADPCM_BYTES_PER_PKT   100  // 200 audio samples per packet @ 4-bit ADPCM
#define AUDIO_SAMPLES_PER_PKT (ADPCM_BYTES_PER_PKT * 2)

// BLE service/characteristics (custom 128-bit UUIDs)
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e" // NUS-like base
#define CMD_CHAR_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // write
#define IMU_CHAR_UUID       "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // notify
#define AUDIO_CHAR_UUID     "6e400004-b5a3-f393-e0a9-e50e24dcca9e" // notify
#define STATUS_CHAR_UUID    "6e400005-b5a3-f393-e0a9-e50e24dcca9e" // notify/read

// Commands (written to CMD characteristic)
enum Cmd : uint8_t {
  CMD_SET_TIME   = 0x01, // payload: uint64_t unix epoch milliseconds (little-endian)
  CMD_START      = 0x02,
  CMD_STOP       = 0x03,
  CMD_RESET      = 0x04,
};

// Packet types
enum PktType : uint8_t {
  PKT_IMU   = 0x01,
  PKT_AUDIO = 0x02,
  PKT_STATUS= 0x10,
};

// -------------------- BLE objects --------------------
BLEService dataService(SERVICE_UUID);
BLECharacteristic cmdChar(CMD_CHAR_UUID, BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic imuChar(IMU_CHAR_UUID, BLENotify, 20 + IMU_SAMPLES_PER_PKT * 14);
BLECharacteristic audioChar(AUDIO_CHAR_UUID, BLENotify, 20 + ADPCM_BYTES_PER_PKT);
BLECharacteristic statusChar(STATUS_CHAR_UUID, BLERead | BLENotify, 20);

// -------------------- Sensors --------------------
// For Seeed XIAO nRF52840 Sense with "LSM6DS3" library the constructor is often LSM6DS3(I2C_MODE, 0x6A)
// Adjust if your board/library differs.
LSM6DS3 imu(I2C_MODE, 0x6A);

// -------------------- Time/state --------------------
volatile bool recording = false;
volatile bool timeSynced = false;
uint64_t baseEpochMs = 0;      // phone-provided epoch at sync
uint32_t baseMillis = 0;       // millis() at sync

uint32_t imuSeq = 0;
uint32_t audioSeq = 0;

// IMU batching buffer
struct __attribute__((packed)) ImuSample {
  uint32_t t_ms;   // ms since sync (or since boot if unsynced)
  int16_t ax, ay, az; // accel raw (library units) - scaled on client
  int16_t gx, gy, gz; // gyro raw
};
ImuSample imuBuf[IMU_SAMPLES_PER_PKT];
uint8_t imuCount = 0;
uint32_t lastImuSampleUs = 0;
const uint32_t imuPeriodUs = 1000000UL / IMU_HZ;

// -------------------- Audio (PDM + ADPCM) --------------------
volatile int16_t pcmRing[512];
volatile uint16_t pcmHead = 0, pcmTail = 0;

uint8_t adpcmPayload[ADPCM_BYTES_PER_PKT];
int adpcmPayloadLen = 0;

// IMA ADPCM encoder state
int16_t adpcmPredictor = 0;
uint8_t adpcmStepIndex = 0;
bool adpcmNibbleHigh = false;
uint8_t adpcmCurrentByte = 0;

static const int imaIndexTable[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
static const int imaStepTable[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,
  143,157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,
  1282,1411,1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,
  7132,7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,
  32767
};

// -------------------- Helpers --------------------
uint32_t nowMsSinceSync() {
  if (timeSynced) {
    uint32_t delta = millis() - baseMillis;
    return (uint32_t)(delta); // client can reconstruct absolute: baseEpochMs + delta
  }
  return millis();
}

uint64_t absoluteNowMs() {
  if (timeSynced) return baseEpochMs + (uint64_t)(millis() - baseMillis);
  return (uint64_t)millis();
}

void sendStatus(const char* msg) {
  if (!BLE.connected()) return;
  statusChar.writeValue((const void*)msg, strlen(msg));
}

void resetStreams() {
  imuSeq = 0;
  audioSeq = 0;
  imuCount = 0;
  pcmHead = pcmTail = 0;
  adpcmPredictor = 0;
  adpcmStepIndex = 0;
  adpcmNibbleHigh = false;
  adpcmCurrentByte = 0;
  adpcmPayloadLen = 0;
}

uint8_t adpcmEncodeSample(int16_t sample) {
  int diff = sample - adpcmPredictor;
  uint8_t code = 0;
  int step = imaStepTable[adpcmStepIndex];
  if (diff < 0) { code = 8; diff = -diff; }
  if (diff >= step) { code |= 4; diff -= step; }
  step >>= 1;
  if (diff >= step) { code |= 2; diff -= step; }
  step >>= 1;
  if (diff >= step) { code |= 1; diff -= step; }

  int step2 = imaStepTable[adpcmStepIndex];
  int delta = (step2 >> 3);
  if (code & 4) delta += step2;
  if (code & 2) delta += step2 >> 1;
  if (code & 1) delta += step2 >> 2;
  if (code & 8) adpcmPredictor -= delta; else adpcmPredictor += delta;
  if (adpcmPredictor > 32767) adpcmPredictor = 32767;
  if (adpcmPredictor < -32768) adpcmPredictor = -32768;

  adpcmStepIndex += imaIndexTable[code & 7];
  if (adpcmStepIndex < 0) adpcmStepIndex = 0;
  if (adpcmStepIndex > 88) adpcmStepIndex = 88;
  return code & 0x0F;
}

void pushAdpcmSample(int16_t sample) {
  uint8_t nib = adpcmEncodeSample(sample);
  if (!adpcmNibbleHigh) {
    adpcmCurrentByte = nib; // low nibble first
    adpcmNibbleHigh = true;
  } else {
    adpcmCurrentByte |= (nib << 4);
    adpcmNibbleHigh = false;
    if (adpcmPayloadLen < ADPCM_BYTES_PER_PKT) {
      adpcmPayload[adpcmPayloadLen++] = adpcmCurrentByte;
    }
  }
}

void flushAudioPacket() {
  if (adpcmPayloadLen == 0 || !BLE.connected()) return;
  // Packet: [type(1), seq(4 LE), t_ms(4 LE), sampleRateHz(2 LE), sampleCount(2 LE), pcmPredictor(2 LE), stepIndex(1), reserved(1), data...]
  // Note: including predictor/stepIndex per packet allows stateless decode of each packet
  // if we reset encoder per packet. We do reset per packet for robustness.
  uint8_t pkt[20 + ADPCM_BYTES_PER_PKT];
  uint16_t sampleCount = (uint16_t)(adpcmPayloadLen * 2);
  uint32_t t = nowMsSinceSync();
  pkt[0] = PKT_AUDIO;
  memcpy(pkt + 1, &audioSeq, 4);
  memcpy(pkt + 5, &t, 4);
  uint16_t sr = MIC_HZ; memcpy(pkt + 9, &sr, 2);
  memcpy(pkt + 11, &sampleCount, 2);
  // predictor/stepIndex stored as the state at START of this packet for decoding
  // We saved them before encoding in encodeAudioBatch via local reset strategy below; easier: store 0,0 and we reset decoder expectations.
  int16_t pred = 0; uint8_t step = 0; uint8_t rsv = 0;
  memcpy(pkt + 13, &pred, 2);
  pkt[15] = step; pkt[16] = rsv;
  // 3 bytes padding to align data at 17? keep simple: header 17 bytes
  memcpy(pkt + 17, adpcmPayload, adpcmPayloadLen);
  audioChar.writeValue(pkt, 17 + adpcmPayloadLen);
  audioSeq++;
  adpcmPayloadLen = 0;
  adpcmNibbleHigh = false;
}

void encodeAudioBatch() {
  // Reset encoder per packet so each packet is independently decodable (robust to loss)
  adpcmPredictor = 0;
  adpcmStepIndex = 0;
  adpcmNibbleHigh = false;
  adpcmPayloadLen = 0;

  int produced = 0;
  while (produced < AUDIO_SAMPLES_PER_PKT) {
    uint16_t tail = pcmTail;
    if (tail == pcmHead) break; // empty
    int16_t s = pcmRing[tail];
    pcmTail = (tail + 1) % 512;
    pushAdpcmSample(s);
    produced++;
  }
  // pad with silence if underfilled but we have some data and want timely flush
  if (produced > 0 && produced < AUDIO_SAMPLES_PER_PKT) {
    while (produced < AUDIO_SAMPLES_PER_PKT) { pushAdpcmSample(0); produced++; }
  }
  if (adpcmPayloadLen == ADPCM_BYTES_PER_PKT) {
    flushAudioPacket();
  } else {
    adpcmPayloadLen = 0; // drop incomplete to keep alignment simple
  }
}

void onPDMdata() {
  int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) return;
  static int16_t sampleBuffer[256];
  int toRead = bytesAvailable;
  if (toRead > (int)sizeof(sampleBuffer)) toRead = sizeof(sampleBuffer);
  PDM.read(sampleBuffer, toRead);
  int samplesRead = toRead / 2;
  for (int i = 0; i < samplesRead; i++) {
    uint16_t next = (pcmHead + 1) % 512;
    if (next != pcmTail) { // drop if full
      pcmRing[pcmHead] = sampleBuffer[i];
      pcmHead = next;
    }
  }
}

void flushImuPacket() {
  if (imuCount == 0 || !BLE.connected()) return;
  // Packet: [type(1), seq(4), count(1), reserved(2), samples...]
  // each sample: t_ms(4), ax,ay,az,gx,gy,gz (6*2)=16 bytes -> 16? 4+12=16
  const uint8_t hdr = 8;
  const uint8_t sampleSize = sizeof(ImuSample); // 16
  uint8_t pkt[hdr + IMU_SAMPLES_PER_PKT * sampleSize];
  pkt[0] = PKT_IMU;
  memcpy(pkt + 1, &imuSeq, 4);
  pkt[5] = imuCount;
  pkt[6] = 0; pkt[7] = 0;
  memcpy(pkt + hdr, imuBuf, imuCount * sampleSize);
  imuChar.writeValue(pkt, hdr + imuCount * sampleSize);
  imuSeq++;
  imuCount = 0;
}

void sampleImu() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastImuSampleUs) < imuPeriodUs) return;
  lastImuSampleUs += imuPeriodUs;
  // catch up if we fell far behind
  if ((uint32_t)(nowUs - lastImuSampleUs) > imuPeriodUs * 4) lastImuSampleUs = nowUs;

  ImuSample &s = imuBuf[imuCount];
  s.t_ms = nowMsSinceSync();
  // Library APIs vary; common: readFloatAccelX or readRawAccelX. Using floats scaled to int16.
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();
  // accel in g -> mg*16 approx, gyro in dps -> mdps/10 approx (tune on client as needed)
  s.ax = (int16_t)constrain((int)(ax * 1000.0f), -32768, 32767);
  s.ay = (int16_t)constrain((int)(ay * 1000.0f), -32768, 32767);
  s.az = (int16_t)constrain((int)(az * 1000.0f), -32768, 32767);
  s.gx = (int16_t)constrain((int)(gx * 10.0f), -32768, 32767);
  s.gy = (int16_t)constrain((int)(gy * 10.0f), -32768, 32767);
  s.gz = (int16_t)constrain((int)(gz * 10.0f), -32768, 32767);
  imuCount++;
  if (imuCount >= IMU_SAMPLES_PER_PKT) flushImuPacket();
}

void handleCommand() {
  if (!cmdChar.written()) return;
  int len = cmdChar.valueLength();
  if (len <= 0) return;
  uint8_t buf[20];
  cmdChar.readValue(buf, len);
  uint8_t cmd = buf[0];
  switch (cmd) {
    case CMD_SET_TIME:
      if (len >= 9) {
        uint64_t epoch = 0;
        memcpy(&epoch, buf + 1, 8); // little-endian epoch ms
        baseEpochMs = epoch;
        baseMillis = millis();
        timeSynced = true;
        sendStatus("TIME_SYNCED");
      }
      break;
    case CMD_START:
      resetStreams();
      recording = true;
      lastImuSampleUs = micros();
      sendStatus("STARTED");
      break;
    case CMD_STOP:
      recording = false;
      flushImuPacket();
      sendStatus("STOPPED");
      break;
    case CMD_RESET:
      recording = false;
      timeSynced = false;
      baseEpochMs = 0;
      resetStreams();
      sendStatus("RESET");
      break;
  }
}

// -------------------- Setup/Loop --------------------
void setup() {
  Serial.begin(115200);
  // don't block forever if no USB serial
  unsigned long s = millis();
  while (!Serial && (millis() - s < 2000)) {}

  if (!imu.begin()) {
    Serial.println("Failed to init LSM6DS3");
  } else {
    // Configure rates/ranges if supported by library; defaults are used otherwise.
    // imu.settings.gyroSampleRate = 416; imu.settings.accelSampleRate = 416;
    Serial.println("IMU ready");
  }

  PDM.onReceive(onPDMdata);
  // 1 channel, 16-bit
  if (!PDM.begin(MIC_CHANNELS, MIC_HZ)) {
    Serial.println("Failed to start PDM");
  }

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1) { delay(1000); }
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(dataService);

  dataService.addCharacteristic(cmdChar);
  dataService.addCharacteristic(imuChar);
  dataService.addCharacteristic(audioChar);
  dataService.addCharacteristic(statusChar);
  BLE.addService(dataService);

  statusChar.writeValue("IDLE");
  BLE.advertise();
  Serial.println("Advertising...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected: "); Serial.println(central.address());
    sendStatus("CONNECTED");
    while (central.connected()) {
      handleCommand();
      if (recording) {
        sampleImu();
        // Try to form audio packets from ring buffer
        static uint32_t lastAudioTry = 0;
        uint32_t ms = millis();
        if (ms - lastAudioTry >= 20) { // check ~50Hz
          lastAudioTry = ms;
          // count available samples
          uint16_t avail = (pcmHead >= pcmTail) ? (pcmHead - pcmTail) : (512 - pcmTail + pcmHead);
          if (avail >= AUDIO_SAMPLES_PER_PKT) {
            encodeAudioBatch();
          }
        }
      }
      // yield to BLE stack
      delay(1);
    }
    recording = false;
    Serial.println("Disconnected");
    BLE.advertise();
  }
}
