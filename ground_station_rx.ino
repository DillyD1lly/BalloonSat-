// GROUND STATION: ESP32 + CC1101 (915 MHz) -> USB serial to the PC.
// Forwards every received packet as:  0xAA 0x55 | len | rssi (int8) | lqi | payload[len]
// Run ground_decode.py on the PC to turn that into a WAV (and live audio with --play).
// No debug text on Serial: it would corrupt the binary stream.
//
// Library: RadioLib (>= 6.0). Board: ESP32 Dev Module.
//
// PINS (ESP32 GPIO), same as the air station
//   CC1101 (3.3 V ONLY): CSN 5, SCK 18, MOSI 23, MISO 19, GDO0 4, GDO2 not connected

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define PIN_CC_CS     5
#define PIN_CC_SCK    18
#define PIN_CC_MOSI   23
#define PIN_CC_MISO   19
#define PIN_CC_GDO0   4

// radio settings: must match the air station
#define RF_FREQ_MHZ      915.0
#define RF_BITRATE_KBPS  100.0
#define RF_FREQ_DEV_KHZ  47.6
#define RF_RXBW_KHZ      270.0
#define RF_POWER_DBM     10

CC1101 radio = new Module(PIN_CC_CS, PIN_CC_GDO0, RADIOLIB_NC, RADIOLIB_NC);
volatile bool rxFlag = false;

void IRAM_ATTR onPacket() { rxFlag = true; }

void setup() {
  Serial.begin(460800);
  pinMode(PIN_CC_CS, OUTPUT);
  digitalWrite(PIN_CC_CS, HIGH);
  SPI.begin(PIN_CC_SCK, PIN_CC_MISO, PIN_CC_MOSI, -1);

  int st = radio.begin(RF_FREQ_MHZ, RF_BITRATE_KBPS, RF_FREQ_DEV_KHZ, RF_RXBW_KHZ, RF_POWER_DBM, 16);
  if (st != RADIOLIB_ERR_NONE) {
    pinMode(2, OUTPUT);                       // blink the onboard LED if the radio isn't responding
    while (true) { digitalWrite(2, !digitalRead(2)); delay(150); }
  }
  radio.setPacketReceivedAction(onPacket);
  radio.startReceive();
}

void loop() {
  if (!rxFlag) return;
  rxFlag = false;

  size_t len = radio.getPacketLength();
  if (len > 0 && len <= 63) {
    uint8_t buf[64];
    if (radio.readData(buf, len) == RADIOLIB_ERR_NONE) {
      uint8_t hdr[5] = {0xAA, 0x55, (uint8_t)len, (uint8_t)(int8_t)radio.getRSSI(), (uint8_t)radio.getLQI()};
      Serial.write(hdr, sizeof(hdr));
      Serial.write(buf, len);
    }
  }
  radio.startReceive();
}
