#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <TFT_eSPI.h>
#include "SparkFunLSM6DSO.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

//pins
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define BUZZER_PIN    25
#define BTN_START_PIN 26
#define BTN_STOP_PIN  27

//BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;

//IMU
LSM6DSO myIMU;
static boolean IMUReady = false;


//button
#define BTN_DEBOUNCE_MS 150
volatile uint32_t lastBTN1InterruptMs = 0;
volatile uint32_t lastBTN2InterruptMs = 0;
volatile bool startPressed = false;
volatile bool stopPressed  = false;

//tof
VL53L1X tof;
#define TOF_DISTANCE_MODE VL53L1X::Short
static const uint32_t TIMING_BUDGET_US = 15000;
static const uint32_t INTER_MEASUREMENT_MS = 20;
static boolean tofReady = false;


//display
TFT_eSPI tft = TFT_eSPI();
static int16_t displayW, displayH;

// rep State
enum RepState { Idle, Up, Down };
RepState repState = Idle;

// set State
enum SetState { NotStarted, InProgress, Completed };
SetState setState = NotStarted;


// function declarations:
float readMagnitude();
void IRAM_ATTR onButtonStartPressed();
void IRAM_ATTR onButtonStopPressed();
void drawStaticLayout();
bool initSensor();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting StackSense");

  tft.init();
  tft.setRotation(1);
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
  displayW = tft.width();
  displayH = tft.height();
  drawStaticLayout();

  pinMode(BTN_START_PIN, INPUT_PULLUP);
  pinMode(BTN_STOP_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  tofReady = initSensor();
  if (!tofReady) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("ToF not found", 6, 44, 4);
  }

  IMUReady = myIMU.begin();
  if (!IMUReady) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("IMU not found", 6, 64, 4);
  } else{
    myIMU.initialize(BASIC_SETTINGS);
  }
  BLEDevice::init("StackSense");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("0");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  attachInterrupt(digitalPinToInterrupt(BTN_START_PIN), onButtonStartPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_STOP_PIN), onButtonStopPressed, FALLING);

}

void loop() {
  if (!tofReady || !IMUReady) {
    delay(500);
    return;
  }

  unsigned long now = millis();

  // button event control
  if (startPressed) {
    startPressed = false;
    Serial.println("Start pressed");
  }
  if (stopPressed) {
    stopPressed = false;
    Serial.println("Stop pressed");
  }

  // read sensor data

  // BLE

  // real-time respond
}






// function definitions:
float readMagnitude() {
  // Magnitude of z axes (up and down) acceleration
  float az = myIMU.readFloatAccelZ();
  return fabs(az);
}

void IRAM_ATTR onButtonStartPressed() {
  uint32_t now = millis();
  if ((now - lastBTN1InterruptMs) > BTN_DEBOUNCE_MS ){
    //TODO
    lastBTN1InterruptMs = now;
  }
}

void IRAM_ATTR onButtonStopPressed() {
  uint32_t now = millis();
  if ((now - lastBTN2InterruptMs) > BTN_DEBOUNCE_MS ){
    //TODO
    lastBTN2InterruptMs = now;
  }
}

void drawStaticLayout() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("REPS", 6, 2, 2);
  tft.drawRect(6, 92, displayW - 12, 12, TFT_DARKGREY);
}

bool initSensor() {
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("VL53L1X init failed");
    return false;
  }
  tof.setDistanceMode(TOF_DISTANCE_MODE);
  tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
  tof.startContinuous(INTER_MEASUREMENT_MS);
  return true;
}