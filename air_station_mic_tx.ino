// AIR STATION: UART from logger -> ADPCM -> CC1101 915 MHz
// Library: RadioLib (>= 6.0). Board: ESP32 Dev Module.
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define PIN_CC_CS     27
#define PIN_CC_SCK    18
#define PIN_CC_MOSI   23
#define PIN_CC_MISO   19
#define PIN_CC_GDO0   4
#define PIN_LINK_RX   16
#define PIN_LINK_TX   17
#define LINK_BAUD     921600

#define RF_FREQ_MHZ      915.0
#define RF_BITRATE_KBPS  100.0
#define RF_FREQ_DEV_KHZ  47.6
#define RF_RXBW_KHZ      270.0
#define RF_POWER_DBM     10

#define RAW_BLOCK      224     // 16 kHz samples per block
#define AUDIO_SAMPLES  112     // 8 kHz samples per packet
#define AUDIO_BYTES    56
#define PKT_AUDIO      0xA0
#define PKT_TELEM      0xB0

#define FRAME_AUDIO 0x01
#define FRAME_TELEM 0x02

struct __attribute__((packed)) AudioPkt {
  uint8_t  type;
  uint16_t seq;
  int16_t  pred;
  int8_t   idx;
  uint8_t  data[AUDIO_BYTES];
};
static_assert(sizeof(AudioPkt) == 62, "AudioPkt must be 62 bytes");

// Must be identical to the logger's Telem struct
struct __attribute__((packed)) Telem {
  uint8_t  type;
  uint16_t count;
  uint32_t uptimeMs;
  uint8_t  flags;
  float bmeT, bmeP, bmeH, bmeAlt;
  float bmpT, bmpP, bmpAlt;
};
static_assert(sizeof(Telem) == 36, "Telem must be 36 bytes");

CC1101 radio = new Module(PIN_CC_CS, PIN_CC_GDO0, RADIOLIB_NC, RADIOLIB_NC);

static uint16_t audioSeq = 0;
static uint32_t audioSent = 0, telemSent = 0, txErrors = 0, badFrames = 0;

// ---------- IMA-ADPCM encoder ----------
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

// ---------- Radio transmit ----------
static void sendAudio(const int16_t *blk16k) {
  AudioPkt p;
  p.type = PKT_AUDIO;
  p.seq  = audioSeq++;
  p.pred = (int16_t)encPred;
  p.idx  = (int8_t)encIdx;
  for (int i = 0; i < AUDIO_SAMPLES; i += 2) {
    int16_t a = (int16_t)(((int)blk16k[2 * i]     + blk16k[2 * i + 1]) / 2);
    int16_t b = (int16_t)(((int)blk16k[2 * i + 2] + blk16k[2 * i + 3]) / 2);
    uint8_t lo = adpcmEncode(a);
    uint8_t hi = adpcmEncode(b);
    p.data[i / 2] = lo | (hi << 4);
  }
  int st = radio.transmit((uint8_t *)&p, sizeof(p));
  if (st == RADIOLIB_ERR_NONE) audioSent++; else txErrors++;
}

static void sendTelem(Telem &t) {
  t.type = PKT_TELEM;
  // No ACK on this link, so send twice; the ground station drops duplicates by count
  for (int i = 0; i < 2; i++) {
    if (radio.transmit((uint8_t *)&t, sizeof(t)) == RADIOLIB_ERR_NONE) telemSent++;
    else txErrors++;
  }
}

// ---------- UART frame parser ----------
// Frame: 0xAA 0x55 | type | len lo | len hi | payload | XOR checksum
static uint8_t  payload[512];
static uint8_t  state = 0, ftype = 0, fsum = 0;
static uint16_t flen = 0, fidx = 0;
static float    prevIn = 0, prevOut = 0;   // DC-blocking high-pass

static void handleFrame() {
  if (ftype == FRAME_AUDIO && flen == sizeof(int16_t) * RAW_BLOCK) {
    int16_t *blk = (int16_t *)payload;
    for (int i = 0; i < RAW_BLOCK; i++) {
      float x = blk[i];
      float y = x - prevIn + 0.995f * prevOut;
      prevIn = x; prevOut = y;
      if (y > 32767.f)  y = 32767.f;
      if (y < -32768.f) y = -32768.f;
      blk[i] = (int16_t)y;
    }
    sendAudio(blk);
  } else if (ftype == FRAME_TELEM && flen == sizeof(Telem)) {
    sendTelem(*(Telem *)payload);
  } else {
    badFrames++;
  }
}

static void pollLink() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    switch (state) {
      case 0: if (b == 0xAA) state = 1; break;
      case 1: state = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
      case 2: ftype = b; fsum = b; state = 3; break;
      case 3: flen = b; fsum ^= b; state = 4; break;
      case 4:
        flen |= (uint16_t)b << 8; fsum ^= b;
        if (flen == 0 || flen > sizeof(payload)) { badFrames++; state = 0; }
        else { fidx = 0; state = 5; }
        break;
      case 5:
        payload[fidx++] = b; fsum ^= b;
        if (fidx == flen) state = 6;
        break;
      case 6:
        if (b == fsum) handleFrame(); else badFrames++;
        state = 0;
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nAir station: logger link -> CC1101");

  Serial2.setRxBufferSize(8192);   // radio.transmit() blocks ~6 ms; this absorbs the backlog
  Serial2.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);

  pinMode(PIN_CC_CS, OUTPUT);
  digitalWrite(PIN_CC_CS, HIGH);
  SPI.begin(PIN_CC_SCK, PIN_CC_MISO, PIN_CC_MOSI, -1);

  int st = radio.begin(RF_FREQ_MHZ, RF_BITRATE_KBPS, RF_FREQ_DEV_KHZ, RF_RXBW_KHZ, RF_POWER_DBM, 16);
  Serial.printf("CC1101 %s (code %d)\n", st == RADIOLIB_ERR_NONE ? "ok" : "FAIL", st);
  if (st != RADIOLIB_ERR_NONE) while (true) delay(1000);

  Serial.println("Waiting for logger data");
}

void loop() {
  static uint32_t lastStatus = 0;
  pollLink();

  uint32_t now = millis();
  if (now - lastStatus >= 5000) {
    lastStatus = now;
    Serial.printf("t=%lus audio=%lu telem=%lu txErr=%lu badFrames=%lu\n", now / 1000,
                  (unsigned long)audioSent, (unsigned long)telemSent,
                  (unsigned long)txErrors, (unsigned long)badFrames);
  }
}
