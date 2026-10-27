//Initial setup for BME
void Config_BME(Adafruit_BME680 &bme){
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(0, 0); // gas heater off, not logging gas resistance

}
//Initial setup for BMP 
void Config_BMP280(Adafruit_BMP280 &bmp) {
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,      
                  Adafruit_BMP280::SAMPLING_X8,      
                  Adafruit_BMP280::SAMPLING_X4,      
                  Adafruit_BMP280::FILTER_X4,        
                  Adafruit_BMP280::STANDBY_MS_500);  
}
//Time since boot with 1 call and formatting
void prntTime(Print &out) {
  unsigned long ms = millis();
  unsigned long h = ms / 3600000UL;
  unsigned long m = (ms / 60000UL) % 60;
  float s = (ms % 60000UL) / 1000.0;

  if (h > 0) {
    out.printf("%luh %02lum %06.3fs\n", h, m, s);
  } else if (m > 0) {
    out.printf("%lum %06.3fs\n", m, s);
  } else {
    out.printf("%.3fs\n", s);
  }
}

//prints  data to serial monitor
void prntData(int count, Adafruit_BME680 &bme, Adafruit_BMP280 &bmp){
    Serial.print("Log count: " + String(count) + " | Up Time: "); prntTime(Serial);
    Serial.println("Internal temp: " + String(bme.temperature) + "°C | " + "External Temp: " + String(bmp.readTemperature()) + "°C");
    Serial.println("Internal Pressure: " + String(bme.pressure / 100.0) + "hPa | " + "External Pressure: " + String(bmp.readPressure() / 100.0) + "hPa");  
    Serial.println("Humidity: " + String(bme.humidity) + "%");   
    Serial.println("Altitude: " + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + "m" );
    Serial.println("");  
}
//Logs data to SD card
void logData(File &f, int count, Adafruit_BME680 &bme, Adafruit_BMP280 &bmp){
    f.print("Log count: " + String(count) + " | Up Time: "); prntTime(f);
    f.println("Internal temp: " + String(bme.temperature) + "°C | " + "External Temp: " + String(bmp.readTemperature()) + "°C");
    f.println("Internal Pressure: " + String(bme.pressure / 100.0) + "hPa | " + "External Pressure: " + String(bmp.readPressure() / 100.0) + "hPa");  
    f.println("Humidity: " + String(bme.humidity) + "%");   
    f.println("Altitude: " + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + "m" );
    f.println("");  
}

// Use these around ANY SD access in the main sketch
void sdLock()  { if (sdMutex) xSemaphoreTake(sdMutex, portMAX_DELAY); }
void sdUnlock() { if (sdMutex) xSemaphoreGive(sdMutex); }

static void writeWavHeader(File &f, uint32_t dataBytes) {
  uint32_t sr = AUDIO_RATE;
  uint16_t ch = 1, bits = 16, pcm = 1, blockAlign = ch * bits / 8;
  uint32_t byteRate = sr * blockAlign;
  uint32_t chunkSize = 36 + dataBytes, fmtSize = 16;

  f.seek(0);
  f.write((const uint8_t *)"RIFF", 4);
  f.write((uint8_t *)&chunkSize, 4);
  f.write((const uint8_t *)"WAVE", 4);
  f.write((const uint8_t *)"fmt ", 4);
  f.write((uint8_t *)&fmtSize, 4);
  f.write((uint8_t *)&pcm, 2);
  f.write((uint8_t *)&ch, 2);
  f.write((uint8_t *)&sr, 4);
  f.write((uint8_t *)&byteRate, 4);
  f.write((uint8_t *)&blockAlign, 2);
  f.write((uint8_t *)&bits, 2);
  f.write((const uint8_t *)"data", 4);
  f.write((uint8_t *)&dataBytes, 4);
}
// Rewrite the header with the current size, then return to the end of the file
static void updateHeader(File &f, uint32_t dataBytes) {
  size_t pos = f.position();
  writeWavHeader(f, dataBytes);
  f.seek(pos);
  f.flush();
}

// Task 1: I2S -> 16-bit samples -> ring buffer
static void audioReadTask(void *) {
  int32_t in[256];
  int16_t out[256];
  for (;;) {
    size_t bytes = i2s.readBytes((char *)in, sizeof(in));
    int n = bytes / sizeof(int32_t);
    if (n <= 0) { vTaskDelay(1); continue; }

    for (int i = 0; i < n; i++) {
      int32_t s = in[i] >> GAIN_SHIFT;          // 24-bit sample sits in the top of a 32-bit word
      if (s > 32767)  s = 32767;
      if (s < -32768) s = -32768;
      out[i] = (int16_t)s;
    }

    size_t need = n * sizeof(int16_t);
    if (xStreamBufferSpacesAvailable(audioBuf) >= need) {
      xStreamBufferSend(audioBuf, out, need, 0);   // all or nothing, keeps samples aligned
    } else {
      audioDropped += n;
    }
  }
}

// Task 2: ring buffer -> WAV files on the SD card
static void audioWriteTask(void *) {
  static uint8_t chunk[4096];
  int fileIndex = 1;

  for (;;) {
    char path[24];

    sdLock();
    do { snprintf(path, sizeof(path), "/rec%04d.wav", fileIndex++); } while (SD.exists(path));
    File f = SD.open(path, FILE_WRITE);
    if (f) writeWavHeader(f, 0);
    sdUnlock();

    if (!f) {
      Serial.println("Audio: could not open WAV file, retrying");
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }
    Serial.print("Audio: recording to ");
    Serial.println(path);

    uint32_t dataBytes = 0;
    uint32_t lastHeader = millis();
    const uint32_t maxBytes = AUDIO_RATE * 2UL * FILE_SECONDS;
    bool writeError = false;

    while (dataBytes < maxBytes && !writeError) {
      size_t got = xStreamBufferReceive(audioBuf, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
      if (got) {
        sdLock();
        size_t w = f.write(chunk, got);
        sdUnlock();
        dataBytes += w;
        if (w != got) writeError = true;
      }
      if (millis() - lastHeader >= HEADER_UPDATE_MS) {
        sdLock();
        updateHeader(f, dataBytes);
        sdUnlock();
        lastHeader = millis();
      }
    }

    sdLock();
    updateHeader(f, dataBytes);
    f.close();
    sdUnlock();

    Serial.print("Audio: closed ");
    Serial.print(path);
    Serial.print(", dropped samples: ");
    Serial.println(audioDropped);
    if (writeError) vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
// Call once from setup(), AFTER the SD card is mounted
bool audioBegin() {
  sdMutex  = xSemaphoreCreateMutex();
  audioBuf = xStreamBufferCreate(32768, 1);
  if (!sdMutex || !audioBuf) return false;

  i2s.setPins(I2S_BCLK, I2S_WS, -1, I2S_DIN);
  if (!i2s.begin(I2S_MODE_STD, AUDIO_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                 I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    return false;
  }

  xTaskCreatePinnedToCore(audioReadTask,  "i2sRead", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(audioWriteTask, "sdWrite", 8192, NULL, 3, NULL, 0);
  return true;
}
