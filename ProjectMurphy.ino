#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BMP280.h>
#include <ESP_I2S.h>
#include <freertos/stream_buffer.h>

// ---------- Settings ----------
const int      I2S_BCLK        = 26;      // INMP441 SCK
const int      I2S_WS          = 25;      // INMP441 WS
const int      I2S_DIN         = 33;      // INMP441 SD
const uint32_t AUDIO_RATE      = 16000;   // Hz
const int      GAIN_SHIFT      = 14;      // 16 = no gain, 14 = 4x, 13 = 8x (clamped)
const uint32_t FILE_SECONDS    = 600;     // start a new WAV file every 10 minutes
const uint32_t HEADER_UPDATE_MS = 5000;   // refresh WAV header so power loss keeps the audio

// ---------- Internals ----------
static I2SClass            i2s;
static SemaphoreHandle_t   sdMutex     = NULL;
static StreamBufferHandle_t audioBuf   = NULL;
static volatile uint32_t   audioDropped = 0;   // samples lost if the buffer ever overflowed

Adafruit_BME680 bme1(&Wire); // Internal sensor (0x77)
Adafruit_BMP280 bmp1(&Wire); // External Sensor (0x76)

#define SEALEVELPRESSURE_HPA (1013.25) // standard sea level pressure, change for accuracy

File data;

int meteoCount = 1;

const int chipSelect = 5;


void setup() {
  Serial.begin(115200);
  delay(1500);
  //init SD card
  SPI.begin(18, 19, 23, 5);
  if (!SD.begin(chipSelect, SPI, 4000000)) {   // use 1000000 if 4 MHz won't mount
    Serial.println("SD card failed");
    while (1) delay(1000);
  }
  //test for microphone
  if (!audioBegin()) {
    Serial.println("Audio (INMP441) failed to start");
  }
  //Test for sd card and log initial header in MetData
  sdLock();
  if (!SD.exists("/MetData.txt")) {
    data = SD.open("/MetData.txt", FILE_WRITE);
    if (data) {
      data.println("--------------------Sensor Log--------------------");
      data.close();
    }
  }
  sdUnlock();
  //Test for sensors BME and BMP at address
  if (!bme1.begin(0x77)) {
    Serial.println("Could not find Internal Sensor at 0x77");
    while (1) delay(1000);
  }
  if (!bmp1.begin(0x76)) {
    Serial.println("Could not find External Sensor at 0x76");
    while (1) delay(1000);
  }
 
  Config_BME(bme1);
  Config_BMP280(bmp1);
}

void loop() {
  
//test for bmp
  if (!bme1.performReading()) {
    Serial.println("Sensor read failed");
    delay(5000);
    return;
  }
//test for bmp
  if (!bmp1.takeForcedMeasurement()) {
    Serial.println("Sensor read failed");
    delay(5000);
    return;
  }
  sdLock();
  data = SD.open("/MetData.txt", FILE_APPEND);

  if (data) {
    prntData(meteoCount, bme1, bmp1);  
    logData(data, meteoCount, bme1, bmp1); 
    data.close();
    meteoCount++;
  } else {
    Serial.println("Error opening MetData.txt");
  }
  sdUnlock();

  delay(30000);
}