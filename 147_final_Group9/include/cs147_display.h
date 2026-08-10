#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "cs147_pins.h"

constexpr int CS147_SCREEN_WIDTH = 128;
constexpr int CS147_SCREEN_HEIGHT = 64;

inline Adafruit_SSD1306& cs147Display() {
  static Adafruit_SSD1306 display(
      CS147_SCREEN_WIDTH,
      CS147_SCREEN_HEIGHT,
      &SPI,
      CS147Pins::OLED_DC,
      CS147Pins::OLED_RST,
      CS147Pins::OLED_CS);
  return display;
}

inline bool cs147InitDisplay() {
  auto& display = cs147Display();
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("OLED init failed. Check SPI wiring and pin map.");
    return false;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("CS147 IoT Kit");
  display.display();
  return true;
}

inline void cs147DisplayLines(const String& line1,
                              const String& line2 = "",
                              const String& line3 = "",
                              const String& line4 = "") {
  auto& display = cs147Display();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(line1);
  if (line2.length()) display.println(line2);
  if (line3.length()) display.println(line3);
  if (line4.length()) display.println(line4);
  display.display();
}
