// AIR STATION: INMP441 mic -> ESP32 -> CC1101 915 MHz. Mic audio only.
// 16 kHz capture -> 8 kHz -> IMA-ADPCM (4 bit) -> 62-byte packets, ~71/s, CC1101 at 100 kbps.
//
// Library: RadioLib (>= 6.0). Board: ESP32 Dev Module.
//
// PINS (ESP32 GPIO)
//   CC1101 (3.3 V ONLY): CSN 5, SCK 18, MOSI 23, MISO 19, GDO0 4, GDO2 not connected
//   INMP441: VDD 3.3 V, GND, SD 33, WS 25, SCK 26, L/R -> GND

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "driver/i2s.h"

#define PIN_CC_CS     5
#define PIN_CC_SCK    18
#define PIN_CC_MOSI   23
#define PIN_CC_MISO   19
#define PIN_CC_GDO0   4
#define PIN_I2S_SD    33
#define PIN_I2S_WS    25
#define PIN_I2S_SCK   26

// radio settings: must match the ground station
#define RF_FREQ_MHZ      915.0
#define RF_BITRATE_KBPS  100.0
#define RF_FREQ_DEV_KHZ  47.6
#define RF_RXBW_KHZ      270.0
#define RF_POWER_DBM     10

#define I2S_RATE       16000
#define RAW_BLOCK      224     // 16 kHz samples per block (14 ms)
#define AUDIO_SAMPLES  112     // 8 kHz samples per packet
#define AUDIO_BYTES    56      // 112 samples x 4 bits
#define MIC_SHIFT      14      // 16 = unity gain, 14 = 4x, 13 = 8x
#define PKT_AUDIO      0xA0

struct __attribute__((packed)) AudioPkt {
  uint8_t  type;
  uint16_t seq;
  int16_t  pred;   // ADPCM state at start of packet, so each packet decodes on its own
  int8_t   idx;
  uint8_t  data[AUDIO_BYTES];
};
static_assert(sizeof(AudioPkt) == 62, "AudioPkt must be 62 bytes");

CC1101 radio = new Module(PIN_CC_CS, PIN_CC_GDO0, RADIOLIB_NC, RADIOLIB_NC);
static QueueHandle_t audioQ;
static volatile uint16_t qDrops = 0;
static uint16_t audioSeq = 0;
static uint32_t audioSent = 0, txErrors = 0;

static const int8_t indexTable[16] = {-1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8};
static const int16_t stepTable[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,73,80,88,97,107,118,130,143,
  157,173,190,209,230,253,279,307,337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,
  1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,7845,8630,9493,
  10442,11487,12635,13899,15289,16818,18500,20350,22385,24623,27086,29794,32767};

static int encPred = 0, encIdx = 0;

static uint8_t adpcmEncode(int16_t sample) {
  int step = stepTable[encIdx];
  int diff = sample - encPred;
  uint8_t code = 0;
  if (diff < 0) { code = 8; diff = -diff; }
  int vpdiff = step >> 3;
  if (diff >= step) { code |= 4; diff -= step; vpdiff += step; }
  step >>= 1;
  if (diff >= step) { code |= 2; diff -= step; vpdiff += step; }
  step >>= 1;
  if (diff >= step) { code |= 1; vpdiff += step; }
  encPred += (code & 8) ? -vpdiff : vpdiff;
  if (encPred > 32767) encPred = 32767;
  if (encPred < -32768) encPred = -32768;
  encIdx += indexTable[code];
  if (encIdx < 0) encIdx = 0;
  if (encIdx > 88) encIdx = 88;
  return code;
}

// I2S capture on core 0
static void captureTask(void*) {
  static int32_t raw[RAW_BLOCK];
  static int16_t blk[RAW_BLOCK];
  float prevIn = 0, prevOut = 0;   // DC-blocking high-pass
  for (;;) {
    size_t got = 0;
    i2s_read(I2S_NUM_0, raw, sizeof(raw), &got, portMAX_DELAY);
    if (got < sizeof(raw)) continue;
    for (int i = 0; i < RAW_BLOCK; i++) {
      float x = (float)(raw[i] >> MIC_SHIFT);
      float y = x - prevIn + 0.995f * prevOut;
      prevIn = x; prevOut = y;
      if (y > 32767.f) y = 32767.f;
      if (y < -32768.f) y = -32768.f;
      blk[i] = (int16_t)y;
    }
    if (xQueueSend(audioQ, blk, 0) != pdTRUE) qDrops++;
  }
}

static bool startI2S() {
  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = I2S_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;      // L/R tied to GND
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return false;

  i2s_pin_config_t pins;
  memset(&pins, 0, sizeof(pins));
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_SCK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_I2S_SD;
  if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;
  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

static void sendAudio(const int16_t* blk16k) {
  AudioPkt p;
  p.type = PKT_AUDIO;
  p.seq = audioSeq++;
  p.pred = (int16_t)encPred;
  p.idx = (int8_t)encIdx;
  for (int i = 0; i < AUDIO_SAMPLES; i += 2) {
    int16_t a = (int16_t)(((int)blk16k[2 * i]     + blk16k[2 * i + 1]) / 2);   // 16k -> 8k
    int16_t b = (int16_t)(((int)blk16k[2 * i + 2] + blk16k[2 * i + 3]) / 2);
    uint8_t lo = adpcmEncode(a);
    uint8_t hi = adpcmEncode(b);
    p.data[i / 2] = lo | (hi << 4);
  }
  int st = radio.transmit((uint8_t*)&p, sizeof(p));
  if (st == RADIOLIB_ERR_NONE) audioSent++; else txErrors++;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nAir station: mic transmitter");

  pinMode(PIN_CC_CS, OUTPUT);
  digitalWrite(PIN_CC_CS, HIGH);
  SPI.begin(PIN_CC_SCK, PIN_CC_MISO, PIN_CC_MOSI, -1);

  int st = radio.begin(RF_FREQ_MHZ, RF_BITRATE_KBPS, RF_FREQ_DEV_KHZ, RF_RXBW_KHZ, RF_POWER_DBM, 16);
  Serial.printf("CC1101 %s (code %d)\n", st == RADIOLIB_ERR_NONE ? "ok" : "FAIL", st);
  if (st != RADIOLIB_ERR_NONE) while (true) delay(1000);   // check wiring, 3.3 V, 915 MHz module

  audioQ = xQueueCreate(40, sizeof(int16_t) * RAW_BLOCK);
  if (!startI2S()) {
    Serial.println("I2S FAIL");
    while (true) delay(1000);
  }
  xTaskCreatePinnedToCore(captureTask, "capture", 4096, NULL, 3, NULL, 0);
  Serial.println("Transmitting");
}

void loop() {
  static int16_t blk[RAW_BLOCK];
  static uint32_t lastStatus = 0;

  if (xQueueReceive(audioQ, blk, pdMS_TO_TICKS(20)) == pdTRUE) sendAudio(blk);

  uint32_t now = millis();
  if (now - lastStatus >= 5000) {
    lastStatus = now;
    Serial.printf("t=%lus sent=%lu txErr=%lu qDrops=%u\n",
                  now / 1000, (unsigned long)audioSent, (unsigned long)txErrors, (unsigned)qDrops);
  }
}
