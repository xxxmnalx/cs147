#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "cs147_pins.h"

inline void cs147StartSerial(const char* activityName) {
  Serial.begin(115200);
  delay(600);
  Serial.println();
  Serial.println("========================================");
  Serial.print("CS 147 Activity: ");
  Serial.println(activityName);
  Serial.println("========================================");
}

inline void cs147InitI2C() {
  Wire.begin(CS147Pins::I2C_SDA, CS147Pins::I2C_SCL);
}

inline void cs147InitSPI() {
  SPI.begin(CS147Pins::SPI_SCK, CS147Pins::SPI_MISO, CS147Pins::SPI_MOSI);
}

inline void cs147PrintDeviceId() {
  uint64_t chipid = ESP.getEfuseMac();
  Serial.print("ESP32 chip id: 0x");
  Serial.println((uint32_t)(chipid & 0xFFFFFFFF), HEX);
}

inline bool cs147Every(uint32_t intervalMs, uint32_t& lastMs) {
  const uint32_t now = millis();
  if (now - lastMs >= intervalMs) {
    lastMs = now;
    return true;
  }
  return false;
}

inline void cs147I2CScan() {
  Serial.println("Scanning I2C bus...");
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  Serial.print("I2C devices found: ");
  Serial.println(count);
}
