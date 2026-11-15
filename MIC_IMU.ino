/*
  MIC_IMU.ino

  ESP32-C3 SuperMini + MPU6050 + MAX4466
  --------------------------------------
  - MPU6050 on I2C: SDA=GPIO8, SCL=GPIO9, address 0x68
  - MAX4466 on ADC: GPIO0 (ADC1 channel 0)
  - IMU sampled at 200 Hz, microphone at 8000 Hz
  - BLE GATT server for Web Bluetooth (Chrome/Edge on Android or PC)
  - Host writes UTC time on connection; every data block is epoch timestamped
  - Audio is IMA ADPCM compressed (4 bits/sample)
  - IMU axes are delta + varint compressed
  - Data is sent in 1-second blocks, split into as few BLE notifications as
    the negotiated ATT MTU allows.

  Tested target: ESP32 Arduino core 3.x (ESP32-C3, NimBLE BLE backend).

  Packet layout (all little-endian)
  --------------------------------
  Notification fragment header, 10 bytes:
    0   u8  magic = 0x4D
    1   u8  version = 1
    2   u16 fragmentIndex
    4   u16 fragmentCount
    6   u16 blockId
    8   u16 payloadLength

  Block body, 40-byte header + payloads:
    0   u32 magic = 0x3142494D ("MIB1" in memory)
    4   u8  version = 1
    5   u8  codec = 1 (IMA ADPCM)
    6   u8  flags (bit0 = final block)
    7   u8  reserved
    8   u16 blockId
    10  u16 audioSamples
    12  u16 imuSamples
    14  u16 audioBytes
    16  u16 imuBytes
    18  u16 audioSampleRate (8000)
    20  u16 imuRate (200)
    22  i16 adpcmInitialPredictor
    24  u8  adpcmInitialIndex
    25  u8  reserved
    26  u16 sessionId
    28  u64 audioBlockStartEpochMicroseconds
    36  u32 CRC32 over bytes [0..35] followed by the payloads
    40  audio ADPCM bytes (low nibble = even sample)
    ... IMU compressed bytes

  IMU payload:
    first sample:  u16 tick (125 us units from block start)
                  6 x i16 raw axes (ax, ay, az, gx, gy, gz)
    next samples: u16 tickDelta
                  6 x unsigned LEB128 zig-zag(axisDelta)

  Control characteristic commands:
    0x01 + u64 unixMillis  set time
    0x02 + u16 sessionId   start
    0x03                   stop
    0x04                   reset counters / stop
    0x05                   request status
*/

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <esp_adc/adc_continuous.h>
#include <esp_timer.h>
#include <soc/soc_caps.h>
#include <sys/time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
static constexpr int PIN_I2C_SDA = 8;
static constexpr int PIN_I2C_SCL = 9;
static constexpr int PIN_MIC     = 0;
static constexpr uint8_t MPU_ADDR = 0x68;

// ---------------------------------------------------------------------------
// BLE UUIDs - must match index.html
// ---------------------------------------------------------------------------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CONTROL_CHAR_UUID   "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define STATUS_CHAR_UUID    "1c95d5e3-d8f7-413a-bf3d-24e8d3c8d3e9"

static const char *DEVICE_NAME = "MicIMU-C3";
static const char *FW_VERSION  = "1.0";

// ---------------------------------------------------------------------------
// Stream / packet constants
// ---------------------------------------------------------------------------
static constexpr uint32_t AUDIO_RATE      = 8000;
static constexpr uint32_t IMU_RATE        = 200;
static constexpr uint32_t TICKS_PER_SEC   = 8000;    // 8000 * 125 us
static constexpr uint32_t BLOCK_SAMPLES   = 8000;    // 1 second
static constexpr uint16_t BLOCK_HEADER    = 40;
static constexpr uint16_t AUDIO_BYTES_MAX = 4000;    // 8000 / 2
static constexpr uint16_t IMU_BYTES_MAX   = 4096;
static constexpr uint16_t BLOCK_MAX       = BLOCK_HEADER + AUDIO_BYTES_MAX + IMU_BYTES_MAX;
static constexpr int      BLOCK_POOL      = 3;
static constexpr uint16_t FRAG_HEADER     = 10;
static constexpr uint32_t BLOCK_MAGIC     = 0x3142494DUL; // "MIB1"
static constexpr uint8_t  FRAG_MAGIC      = 0x4D;
static constexpr uint8_t  PROTO_VERSION   = 1;
static constexpr uint8_t  CODEC_IMA_ADPCM = 1;

// ---------------------------------------------------------------------------
// Status / error codes
// ---------------------------------------------------------------------------
enum : uint8_t {
  ERR_NONE = 0,
  ERR_NEED_TIME = 1,
  ERR_ADC = 2,
  ERR_BAD_CMD = 3,
  ERR_BUSY = 4,
  ERR_TIME_RANGE = 5,
  ERR_NO_MEM = 6
};

// ---------------------------------------------------------------------------
// Globals shared with BLE callbacks
// ---------------------------------------------------------------------------
static BLEServer         *gBleServer = nullptr;
static BLECharacteristic *gDataChar = nullptr;
static BLECharacteristic *gControlChar = nullptr;
static BLECharacteristic *gStatusChar = nullptr;

static volatile bool     gConnected = false;
static volatile bool     gDataSubscribed = false;
static volatile bool     gNeedsAdvertising = false;
static volatile uint32_t gAdvertisingAt = 0;
static volatile uint16_t gConnId = 0;
static volatile uint16_t gBleMtu = 23;
static volatile uint16_t gNegotiatedMtu = 23;

static volatile bool     gTimeValid = false;
static volatile bool     gRecActive = false;
static volatile bool     gStartRequest = false;
static volatile bool     gStopRequest = false;
static volatile bool     gResetRequest = false;
static volatile uint16_t gRequestedSessionId = 0;
static volatile uint8_t  gLastError = ERR_NONE;
static volatile bool     gStatusDirty = true;

static volatile bool     gAdcOk = false;
static volatile bool     gImuOk = false;

static volatile uint32_t gSessionSamples = 0;
static volatile uint32_t gAudioDropped = 0;
static volatile uint32_t gImuDropped = 0;
static volatile uint32_t gImuSamplesTotal = 0;
static volatile uint32_t gTxBlocks = 0;
static volatile uint32_t gTxPackets = 0;
static volatile uint32_t gNotifyErrors = 0;
static volatile uint16_t gLastSessionId = 0;

struct DataBlock;

// Audio state (owned by audioTask)
static adc_continuous_handle_t gAdcHandle = nullptr;
static int16_t  gAdpcmPred = 0;
static uint8_t  gAdpcmIndex = 0;
static uint16_t gSessionId = 0;
static uint64_t gSessionEpochUs = 0;
static int64_t  gSessionMonoUs = 0;
static uint32_t gNextBlockId = 0;
static uint16_t gBlockSamples = 0;
static DataBlock *gCurrentBlock = nullptr;

// ---------------------------------------------------------------------------
// IMU queue
// ---------------------------------------------------------------------------
struct ImuSample {
  uint32_t tick;      // 125 us ticks since recording started
  int16_t ax, ay, az, gx, gy, gz;
};

static QueueHandle_t gImuQueue = nullptr;
static constexpr int IMU_QUEUE_DEPTH = 512;

// ---------------------------------------------------------------------------
// Data block and queues
// ---------------------------------------------------------------------------
struct DataBlock {
  uint8_t  data[BLOCK_MAX];
  uint16_t length;
  uint16_t blockId;
  uint16_t sessionId;
  uint16_t audioSamples;
  uint16_t imuSamples;
  uint16_t audioBytes;
  uint16_t imuBytes;
  uint32_t startSample;
  uint64_t startUs;
  int16_t  pred0;
  uint8_t  index0;
  uint8_t  finalBlock;
};

static DataBlock gBlockPool[BLOCK_POOL];
static QueueHandle_t gFreeBlocks = nullptr;
static QueueHandle_t gReadyBlocks = nullptr;

// ---------------------------------------------------------------------------
// Small endian / varint / CRC helpers
// ---------------------------------------------------------------------------
static inline void putU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static inline void putI16(uint8_t *p, int16_t v) {
  putU16(p, (uint16_t)v);
}

static inline void putU32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static inline void putU64(uint8_t *p, uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    p[i] = (uint8_t)(v & 0xFF);
    v >>= 8;
  }
}

static inline uint16_t getU16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint64_t getU64(const uint8_t *p) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) {
    v = (v << 8) | p[i];
  }
  return v;
}

static inline uint32_t crc32Update(uint32_t crc, const uint8_t *p, size_t n) {
  while (n--) {
    crc ^= *p++;
    for (int i = 0; i < 8; ++i) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1)));
    }
  }
  return crc;
}

static inline uint32_t crc32Parts(const uint8_t *a, size_t na, const uint8_t *b, size_t nb) {
  uint32_t crc = 0xFFFFFFFFUL;
  crc = crc32Update(crc, a, na);
  crc = crc32Update(crc, b, nb);
  return ~crc;
}

// Returns number of bytes written (1..5).
static inline int putVarint(uint8_t *p, uint32_t v) {
  int n = 0;
  do {
    uint8_t b = (uint8_t)(v & 0x7F);
    v >>= 7;
    if (v) b |= 0x80;
    p[n++] = b;
  } while (v);
  return n;
}

static inline uint32_t zigZag(int32_t v) {
  return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31);
}

// ---------------------------------------------------------------------------
// IMA ADPCM
// ---------------------------------------------------------------------------
static const int16_t kIndexTable[16] = {
  -1, -1, -1, -1, 2, 4, 6, 8,
  -1, -1, -1, -1, 2, 4, 6, 8
};

static const int16_t kStepTable[89] = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
  34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130,
  143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449,
  494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411,
  1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,
  4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
  11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
  27086, 29794, 32767
};

static inline int16_t adpcmDecode(int16_t pred, uint8_t index, uint8_t code) {
  int32_t step = kStepTable[index];
  int32_t diff = step >> 3;
  if (code & 4) diff += step;
  if (code & 2) diff += step >> 1;
  if (code & 1) diff += step >> 2;
  int32_t v = (int32_t)pred + ((code & 8) ? -diff : diff);
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

static inline uint8_t adpcmEncodeSample(int16_t sample) {
  const uint8_t oldIndex = gAdpcmIndex;
  int32_t step = kStepTable[oldIndex];
  int32_t diff = (int32_t)sample - (int32_t)gAdpcmPred;
  uint8_t code = 0;

  if (diff < 0) {
    code |= 8;
    diff = -diff;
  }

  int32_t t = step;
  if (diff >= t) { code |= 4; diff -= t; }
  t >>= 1;
  if (diff >= t) { code |= 2; diff -= t; }
  t >>= 1;
  if (diff >= t) { code |= 1; }

  gAdpcmPred = adpcmDecode(gAdpcmPred, oldIndex, code);
  int idx = (int)oldIndex + kIndexTable[code];
  if (idx < 0) idx = 0;
  if (idx > 88) idx = 88;
  gAdpcmIndex = (uint8_t)idx;
  return code;
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
static int64_t nowEpochUs() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (int64_t)tv.tv_sec * 1000000LL + (int64_t)tv.tv_usec;
}

static bool setTimeFromUnixMs(uint64_t ms) {
  // Reject clearly invalid host clocks (before 2017 / after 2100).
  if (ms < 1500000000000ULL || ms > 4102444800000ULL) {
    return false;
  }
  struct timeval tv;
  tv.tv_sec = (time_t)(ms / 1000ULL);
  tv.tv_usec = (suseconds_t)((ms % 1000ULL) * 1000ULL);
  if (settimeofday(&tv, nullptr) != 0) {
    return false;
  }
  gTimeValid = true;
  return true;
}

// ---------------------------------------------------------------------------
// MPU6050
// ---------------------------------------------------------------------------
static bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)len) != len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    if (!Wire.available()) return false;
    buf[i] = (uint8_t)Wire.read();
  }
  return true;
}

static bool initMpu() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  delay(50);

  uint8_t who = 0;
  if (!mpuRead(0x75, &who, 1) || who != 0x68) {
    Serial.printf("[MPU] WHO_AM_I failed (got 0x%02X)\n", who);
    return false;
  }

  mpuWrite(0x6B, 0x80);  // device reset
  delay(100);
  mpuWrite(0x6B, 0x01);  // wake, PLL with gyro X reference
  delay(10);
  mpuWrite(0x19, 0x04);  // 1 kHz / (1 + 4) = 200 Hz
  mpuWrite(0x1A, 0x03);  // DLPF ~44 Hz accel / 42 Hz gyro
  mpuWrite(0x1B, 0x00);  // gyro +/-250 dps
  mpuWrite(0x1C, 0x00);  // accel +/-2 g
  delay(20);
  Serial.println("[MPU] MPU6050 ready at 200 Hz");
  return true;
}

// ---------------------------------------------------------------------------
// ADC continuous capture (hardware-timed 8 kHz)
// ---------------------------------------------------------------------------
static bool initAdc() {
  adc_continuous_handle_cfg_t handleCfg = {};
  handleCfg.max_store_buf_size = 32768;  // ~1 s of 4-byte results
  handleCfg.conv_frame_size = 256;

  esp_err_t err = adc_continuous_new_handle(&handleCfg, &gAdcHandle);
  if (err != ESP_OK) {
    Serial.printf("[ADC] new_handle failed: %d\n", (int)err);
    return false;
  }

  adc_digi_pattern_config_t pattern = {};
  pattern.atten = ADC_ATTEN_DB_12;
  pattern.channel = ADC_CHANNEL_0;  // GPIO0 on ESP32-C3
  pattern.unit = ADC_UNIT_1;
  pattern.bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t adcCfg = {};
  adcCfg.pattern_num = 1;
  adcCfg.adc_pattern = &pattern;
  adcCfg.sample_freq_hz = AUDIO_RATE;
  adcCfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  adcCfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;

  err = adc_continuous_config(gAdcHandle, &adcCfg);
  if (err != ESP_OK) {
    Serial.printf("[ADC] config failed: %d\n", (int)err);
    return false;
  }

  err = adc_continuous_start(gAdcHandle);
  if (err != ESP_OK) {
    Serial.printf("[ADC] start failed: %d\n", (int)err);
    return false;
  }

  Serial.println("[ADC] continuous capture started at 8000 Hz");
  return true;
}

// ---------------------------------------------------------------------------
// IMU sampling task
// ---------------------------------------------------------------------------
static void imuTask(void *arg) {
  (void)arg;
  const uint8_t bufLen = 14;
  uint8_t buf[bufLen];
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t failCount = 0;

  for (;;) {
    int64_t sampleMono = esp_timer_get_time();

    if (gRecActive) {
      if (mpuRead(0x3B, buf, bufLen)) {
        ImuSample s;
        s.ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
        s.ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
        s.az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
        // buf[6..7] is temperature
        s.gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
        s.gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
        s.gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

        int64_t rel = sampleMono - gSessionMonoUs;
        if (rel >= 0) {
          s.tick = (uint32_t)(rel / 125LL);
          if (xQueueSend(gImuQueue, &s, 0) != pdTRUE) {
            gImuDropped = gImuDropped + 1;
          }
        }
        gImuOk = true;
        failCount = 0;
      } else {
        failCount++;
        if (failCount == 10) {
          gImuOk = false;
          Serial.println("[MPU] read errors while recording");
        }
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / IMU_RATE));
  }
}

// ---------------------------------------------------------------------------
// Block assembly helpers
// ---------------------------------------------------------------------------
static void appendImuSamples(DataBlock *b, uint32_t endTick) {
  ImuSample s;
  bool first = true;
  uint32_t prevTick = 0;
  int16_t prevAxes[6] = {0, 0, 0, 0, 0, 0};
  uint16_t imuOff = (uint16_t)(BLOCK_HEADER + b->audioBytes);
  uint16_t pos = imuOff;
  uint16_t count = 0;

  while (xQueuePeek(gImuQueue, &s, 0) == pdTRUE && s.tick < endTick) {
    xQueueReceive(gImuQueue, &s, 0);

    if (s.tick < b->startSample) {
      gImuDropped = gImuDropped + 1;
      continue;
    }

    uint32_t rel = s.tick - b->startSample;
    if (rel > 65535U) {
      gImuDropped = gImuDropped + 1;
      continue;
    }

    if ((uint32_t)pos + 20U > sizeof(b->data)) {
      gImuDropped = gImuDropped + 1;
      continue;
    }

    const int16_t axes[6] = {s.ax, s.ay, s.az, s.gx, s.gy, s.gz};

    if (first) {
      putU16(b->data + pos, (uint16_t)rel);
      pos += 2;
      for (int i = 0; i < 6; ++i) {
        putI16(b->data + pos, axes[i]);
        pos += 2;
        prevAxes[i] = axes[i];
      }
      first = false;
    } else {
      putU16(b->data + pos, (uint16_t)(rel - prevTick));
      pos += 2;
      for (int i = 0; i < 6; ++i) {
        int32_t delta = (int32_t)axes[i] - (int32_t)prevAxes[i];
        pos += (uint16_t)putVarint(b->data + pos, zigZag(delta));
        prevAxes[i] = axes[i];
      }
    }

    prevTick = rel;
    count++;
  }

  b->imuSamples = count;
  b->imuBytes = (uint16_t)(pos - imuOff);
  gImuSamplesTotal += count;
}

static void discardImuUntil(uint32_t endTick) {
  ImuSample s;
  while (xQueuePeek(gImuQueue, &s, 0) == pdTRUE && s.tick < endTick) {
    xQueueReceive(gImuQueue, &s, 0);
    gImuDropped = gImuDropped + 1;
  }
}

static DataBlock *takeFreeBlock() {
  DataBlock *b = nullptr;
  if (xQueueReceive(gFreeBlocks, &b, 0) != pdTRUE) {
    return nullptr;
  }
  return b;
}

static void beginAudioBlock() {
  gBlockSamples = 0;
  uint16_t id = (uint16_t)gNextBlockId++;

  DataBlock *b = takeFreeBlock();
  if (!b) {
    gCurrentBlock = nullptr;
    gAudioDropped = gAudioDropped + 1;
    return;
  }

  b->length = BLOCK_HEADER;
  b->blockId = id;
  b->sessionId = gSessionId;
  b->audioSamples = 0;
  b->imuSamples = 0;
  b->audioBytes = 0;
  b->imuBytes = 0;
  b->startSample = gSessionSamples;
  b->startUs = gSessionEpochUs + (uint64_t)gSessionSamples * 125ULL;
  b->pred0 = gAdpcmPred;
  b->index0 = gAdpcmIndex;
  b->finalBlock = 0;
  gCurrentBlock = b;
}

static void writeBlockHeaderAndQueue(DataBlock *b, bool finalBlock) {
  uint8_t *d = b->data;
  uint16_t payload = (uint16_t)(b->audioBytes + b->imuBytes);
  b->length = (uint16_t)(BLOCK_HEADER + payload);

  putU32(d, BLOCK_MAGIC);
  d[4] = PROTO_VERSION;
  d[5] = CODEC_IMA_ADPCM;
  d[6] = finalBlock ? 0x01 : 0x00;
  d[7] = 0;
  putU16(d + 8, b->blockId);
  putU16(d + 10, b->audioSamples);
  putU16(d + 12, b->imuSamples);
  putU16(d + 14, b->audioBytes);
  putU16(d + 16, b->imuBytes);
  putU16(d + 18, (uint16_t)AUDIO_RATE);
  putU16(d + 20, (uint16_t)IMU_RATE);
  putI16(d + 22, b->pred0);
  d[24] = b->index0;
  d[25] = 0;
  putU16(d + 26, b->sessionId);
  putU64(d + 28, b->startUs);

  uint32_t crc = crc32Parts(d, 36, d + BLOCK_HEADER, payload);
  putU32(d + 36, crc);

  if (xQueueSend(gReadyBlocks, &b, 0) != pdTRUE) {
    gAudioDropped = gAudioDropped + 1;
    xQueueSend(gFreeBlocks, &b, 0);
  }
}

static void finishAudioBlock(bool finalBlock) {
  uint32_t endTick = gSessionSamples;
  DataBlock *b = gCurrentBlock;

  if (!b && finalBlock) {
    // Emit an empty final block so the browser can leave "stopping" state even
    // if the user stopped before the first audio sample arrived.
    b = takeFreeBlock();
    if (b) {
      b->blockId = (uint16_t)gNextBlockId++;
      b->sessionId = gSessionId;
      b->audioSamples = 0;
      b->imuSamples = 0;
      b->audioBytes = 0;
      b->imuBytes = 0;
      b->startSample = gSessionSamples;
      b->startUs = gSessionEpochUs + (uint64_t)gSessionSamples * 125ULL;
      b->pred0 = gAdpcmPred;
      b->index0 = gAdpcmIndex;
    }
  }

  if (b) {
    b->finalBlock = finalBlock ? 1 : 0;
    appendImuSamples(b, endTick);
    writeBlockHeaderAndQueue(b, finalBlock);
  } else {
    discardImuUntil(endTick);
  }

  gCurrentBlock = nullptr;
  gBlockSamples = 0;
}

static void addAudioSample(uint16_t raw) {
  if (gBlockSamples == 0) {
    beginAudioBlock();
  }

  int16_t sample = (int16_t)raw - 2048;
  uint8_t code = adpcmEncodeSample(sample);

  if (gCurrentBlock) {
    uint16_t byteIndex = (uint16_t)(gCurrentBlock->audioBytes - 1);
    if ((gBlockSamples & 1) == 0) {
      gCurrentBlock->data[BLOCK_HEADER + gCurrentBlock->audioBytes] = code & 0x0F;
      gCurrentBlock->audioBytes++;
    } else if (byteIndex < AUDIO_BYTES_MAX) {
      gCurrentBlock->data[BLOCK_HEADER + byteIndex] |= (uint8_t)((code & 0x0F) << 4);
    }
    gCurrentBlock->audioSamples++;
  }

  gBlockSamples++;
  gSessionSamples = gSessionSamples + 1;

  if (gBlockSamples >= BLOCK_SAMPLES) {
    finishAudioBlock(false);
  }
}

// ---------------------------------------------------------------------------
// Audio capture task
// ---------------------------------------------------------------------------
static void flushAdc() {
  if (!gAdcOk) return;
  uint8_t tmp[256];
  uint32_t got = 0;
  for (int i = 0; i < 64; ++i) {
    if (adc_continuous_read(gAdcHandle, tmp, sizeof(tmp), &got, 0) != ESP_OK) {
      break;
    }
  }
}

static void startRecording(uint16_t sessionId) {
  const bool wasActive = gRecActive;
  gRecActive = false;
  if (wasActive) {
    finishAudioBlock(true);
  }

  flushAdc();
  xQueueReset(gImuQueue);

  gSessionId = sessionId ? sessionId : 1;
  gLastSessionId = gSessionId;
  gSessionMonoUs = esp_timer_get_time();
  gSessionEpochUs = (uint64_t)nowEpochUs();
  gSessionSamples = 0;
  gNextBlockId = 0;
  gBlockSamples = 0;
  gCurrentBlock = nullptr;
  gAdpcmPred = 0;
  gAdpcmIndex = 0;
  gAudioDropped = 0;
  gImuDropped = 0;
  gImuSamplesTotal = 0;
  gTxBlocks = 0;
  gTxPackets = 0;
  gNotifyErrors = 0;
  gRecActive = true;
  gStatusDirty = true;

  Serial.printf("[REC] start session %u, epoch %llu us\n",
                (unsigned)gSessionId, (unsigned long long)gSessionEpochUs);
}

static void stopRecording() {
  if (gRecActive) {
    gRecActive = false;
    finishAudioBlock(true);
    xQueueReset(gImuQueue);
    Serial.println("[REC] stop");
  }
  gStatusDirty = true;
}

static void audioTask(void *arg) {
  (void)arg;
  uint8_t buf[256];

  for (;;) {
    if (gStartRequest) {
      gStartRequest = false;
      uint16_t sid = gRequestedSessionId;
      startRecording(sid);
    }

    if (gStopRequest) {
      gStopRequest = false;
      stopRecording();
    }

    if (gResetRequest) {
      gResetRequest = false;
      if (gRecActive) {
        gRecActive = false;
        finishAudioBlock(true);
      }
      xQueueReset(gImuQueue);
      gSessionSamples = 0;
      gNextBlockId = 0;
      gAudioDropped = 0;
      gImuDropped = 0;
      gImuSamplesTotal = 0;
      gTxBlocks = 0;
      gTxPackets = 0;
      gNotifyErrors = 0;
      gLastError = ERR_NONE;
      gStatusDirty = true;
    }

    if (!gAdcOk) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    uint32_t got = 0;
    esp_err_t err = adc_continuous_read(gAdcHandle, buf, sizeof(buf), &got, 20);

    if (err == ESP_OK && got > 0 && gRecActive) {
      for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= got; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)(buf + i);
        uint32_t channel = p->type2.channel;
        uint32_t value = p->type2.data;
        if (channel == ADC_CHANNEL_0 && value < 4096) {
          addAudioSample((uint16_t)value);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// BLE sender task
// ---------------------------------------------------------------------------
static void sendBlock(DataBlock *b) {
  uint16_t mtu = gBleMtu;
  if (mtu < 23) mtu = 23;
  if (mtu > 517) mtu = 517;

  uint16_t attPayload = (uint16_t)(mtu - 3);
  if (attPayload > 509) attPayload = 509;
  if (attPayload <= FRAG_HEADER) {
    gNotifyErrors = gNotifyErrors + 1;
    return;
  }

  uint16_t chunk = (uint16_t)(attPayload - FRAG_HEADER);
  uint16_t count = (uint16_t)((b->length + chunk - 1) / chunk);
  if (count == 0) count = 1;

  uint8_t packet[FRAG_HEADER + 509];

  for (uint16_t i = 0; i < count; ++i) {
    if (!gConnected || !gDataSubscribed) {
      return;
    }

    uint16_t off = (uint16_t)(i * chunk);
    uint16_t len = b->length - off;
    if (len > chunk) len = chunk;

    packet[0] = FRAG_MAGIC;
    packet[1] = PROTO_VERSION;
    putU16(packet + 2, i);
    putU16(packet + 4, count);
    putU16(packet + 6, b->blockId);
    putU16(packet + 8, len);
    memcpy(packet + FRAG_HEADER, b->data + off, len);

    gDataChar->setValue(packet, (size_t)FRAG_HEADER + len);
    gDataChar->notify();
    gTxPackets = gTxPackets + 1;

    // Pace notifications. They still travel in BLE connection events, but this
    // prevents overflowing the small NimBLE mbuf pool. 12 ms gives the host
    // time to drain while finishing a 1-second block well before the next one.
    vTaskDelay(pdMS_TO_TICKS(12));
  }

  gTxBlocks = gTxBlocks + 1;
}

static void senderTask(void *arg) {
  (void)arg;
  DataBlock *b = nullptr;
  for (;;) {
    if (xQueueReceive(gReadyBlocks, &b, portMAX_DELAY) == pdTRUE) {
      if (gConnected && gDataSubscribed) {
        sendBlock(b);
      }
      xQueueSend(gFreeBlocks, &b, portMAX_DELAY);
    }
  }
}

// ---------------------------------------------------------------------------
// BLE callbacks
// ---------------------------------------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
public:
  void onConnect(BLEServer *pServer) override {
    gConnected = true;
    gConnId = pServer->getConnId();
    gBleMtu = pServer->getPeerMTU(gConnId);
    if (gBleMtu < 23) gBleMtu = 23;
    gNegotiatedMtu = gBleMtu;
    gDataSubscribed = false;
    gNeedsAdvertising = false;

    // 30-50 ms connection interval. Fewer radio wake-ups than the default
    // 7.5 ms, which helps with the audio noise problem.
    pServer->requestConnParams(gConnId, 24, 40, 0, 400);

    gStatusDirty = true;
    Serial.printf("[BLE] connected, conn=%u mtu=%u\n", gConnId, gBleMtu);
  }

  void onDisconnect(BLEServer *pServer) override {
    (void)pServer;
    gConnected = false;
    gDataSubscribed = false;
    gStopRequest = true;
    gNeedsAdvertising = true;
    gAdvertisingAt = millis() + 400;
    gStatusDirty = true;
    Serial.println("[BLE] disconnected");
  }
};

class DataCallbacks : public BLECharacteristicCallbacks {
public:
#if defined(CONFIG_NIMBLE_ENABLED)
  void onSubscribe(BLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue) override {
    (void)pCharacteristic;
    (void)desc;
    gDataSubscribed = (subValue & 0x0001) != 0;
    gStatusDirty = true;
    Serial.printf("[BLE] data notifications %s\n", gDataSubscribed ? "on" : "off");
  }
#endif

  void onStatus(BLECharacteristic *pCharacteristic, Status status, uint32_t code) override {
    (void)code;
    if (pCharacteristic == gDataChar &&
        status != SUCCESS_NOTIFY && status != SUCCESS_INDICATE) {
      gNotifyErrors = gNotifyErrors + 1;
      if (status == ERROR_NO_SUBSCRIBER) {
        gDataSubscribed = false;
      }
    }
  }
};

class StatusCallbacks : public BLECharacteristicCallbacks {
public:
  void onRead(BLECharacteristic *pCharacteristic) override {
    (void)pCharacteristic;
    gStatusDirty = true;
  }
};

class ControlCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *pCharacteristic) override {
    const uint8_t *d = pCharacteristic->getData();
    size_t n = pCharacteristic->getLength();
    if (!d || n == 0) return;

    uint8_t cmd = d[0];

    switch (cmd) {
      case 0x01: {  // SET_TIME
        if (gRecActive) {
          gLastError = ERR_BUSY;
          break;
        }
        if (n < 9) {
          gLastError = ERR_BAD_CMD;
          break;
        }
        uint64_t ms = getU64(d + 1);
        if (!setTimeFromUnixMs(ms)) {
          gLastError = ERR_TIME_RANGE;
        } else {
          gTimeValid = true;
          gLastError = ERR_NONE;
        }
        Serial.printf("[TIME] host epoch %llu ms\n", (unsigned long long)ms);
        break;
      }

      case 0x02: {  // START + optional u16 session id
        if (gRecActive || gStartRequest) {
          gLastError = ERR_BUSY;
        } else if (!gTimeValid) {
          gLastError = ERR_NEED_TIME;
        } else if (!gAdcOk) {
          gLastError = ERR_ADC;
        } else {
          uint16_t sid = 1;
          if (n >= 3) sid = getU16(d + 1);
          if (sid == 0) sid = 1;
          gRequestedSessionId = sid;
          gStartRequest = true;
          gStopRequest = false;
          gResetRequest = false;
          gLastError = ERR_NONE;
          Serial.printf("[CMD] start session %u\n", sid);
        }
        break;
      }

      case 0x03:  // STOP
        gStopRequest = true;
        gStartRequest = false;
        Serial.println("[CMD] stop");
        break;

      case 0x04:  // RESET / stop
        gStopRequest = true;
        gResetRequest = true;
        gStartRequest = false;
        gLastError = ERR_NONE;
        Serial.println("[CMD] reset");
        break;

      case 0x05:  // status request
        break;

      default:
        gLastError = ERR_BAD_CMD;
        break;
    }

    gStatusDirty = true;
  }
};

static ServerCallbacks  serverCallbacks;
static DataCallbacks    dataCallbacks;
static StatusCallbacks  statusCallbacks;
static ControlCallbacks controlCallbacks;

// ---------------------------------------------------------------------------
// Status JSON
// ---------------------------------------------------------------------------
static void publishStatus(bool notify) {
  if (!gStatusChar) return;

  char json[384];
  snprintf(json, sizeof(json),
           "{\"fw\":\"%s\",\"rec\":%d,\"time\":%d,\"imu\":%d,\"adc\":%d,"
           "\"conn\":%d,\"mtu\":%u,\"sub\":%d,\"sess\":%u,\"blk\":%lu,"
           "\"drop\":%lu,\"idrop\":%lu,\"txb\":%lu,\"txp\":%lu,"
           "\"nerr\":%lu,\"a\":%lu,\"i\":%lu,\"err\":%u,\"heap\":%lu}",
           FW_VERSION,
           gRecActive ? 1 : 0,
           gTimeValid ? 1 : 0,
           gImuOk ? 1 : 0,
           gAdcOk ? 1 : 0,
           gConnected ? 1 : 0,
           (unsigned)gBleMtu,
           gDataSubscribed ? 1 : 0,
           (unsigned)gLastSessionId,
           (unsigned long)gTxBlocks,
           (unsigned long)gAudioDropped,
           (unsigned long)gImuDropped,
           (unsigned long)gTxBlocks,
           (unsigned long)gTxPackets,
           (unsigned long)gNotifyErrors,
           (unsigned long)gSessionSamples,
           (unsigned long)gImuSamplesTotal,
           (unsigned)gLastError,
           (unsigned long)ESP.getFreeHeap());

  gStatusChar->setValue((uint8_t *)json, strlen(json));
  if (notify && gConnected) {
    gStatusChar->notify();
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
static void initBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(517);

  gBleServer = BLEDevice::createServer();
  gBleServer->setCallbacks(&serverCallbacks);

  BLEService *service = gBleServer->createService(SERVICE_UUID);

  gDataChar = service->createCharacteristic(
    DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  gDataChar->setCallbacks(&dataCallbacks);

  gControlChar = service->createCharacteristic(
    CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  gControlChar->setCallbacks(&controlCallbacks);

  gStatusChar = service->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  gStatusChar->setCallbacks(&statusCallbacks);

  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("[BLE] advertising as MicIMU-C3");
}

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("=== MIC_IMU ESP32-C3 ===");

  gImuQueue = xQueueCreate(IMU_QUEUE_DEPTH, sizeof(ImuSample));
  gFreeBlocks = xQueueCreate(BLOCK_POOL, sizeof(DataBlock *));
  gReadyBlocks = xQueueCreate(BLOCK_POOL, sizeof(DataBlock *));
  if (!gImuQueue || !gFreeBlocks || !gReadyBlocks) {
    Serial.println("[FATAL] queue allocation failed");
    while (true) delay(1000);
  }

  for (int i = 0; i < BLOCK_POOL; ++i) {
    DataBlock *p = &gBlockPool[i];
    xQueueSend(gFreeBlocks, &p, 0);
  }

  gAdcOk = initAdc();
  gImuOk = initMpu();

  initBle();

  xTaskCreate(audioTask, "audio", 4096, nullptr, 6, nullptr);
  xTaskCreate(imuTask, "imu", 3072, nullptr, 4, nullptr);
  xTaskCreate(senderTask, "ble_tx", 4096, nullptr, 3, nullptr);

  gStatusDirty = true;
}

// ---------------------------------------------------------------------------
// Loop: low-rate housekeeping only (no audio work here)
// ---------------------------------------------------------------------------
void loop() {
  if (gNeedsAdvertising && (int32_t)(millis() - gAdvertisingAt) >= 0) {
    gNeedsAdvertising = false;
    gBleServer->startAdvertising();
    Serial.println("[BLE] advertising restarted");
  }

  static uint32_t lastMtuCheck = 0;
  if (gConnected && millis() - lastMtuCheck > 250) {
    lastMtuCheck = millis();
    uint16_t m = gBleServer->getPeerMTU(gConnId);
    if (m >= 23 && m != gBleMtu) {
      gBleMtu = m;
      gNegotiatedMtu = m;
      gStatusDirty = true;
    }
  }

  if (gStatusDirty) {
    gStatusDirty = false;
    publishStatus(true);
  }

  delay(20);
}
