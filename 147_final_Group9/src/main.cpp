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

//buzzer
#define BEEP_REP    4000 // rep counted, brightest
#define BEEP_START  3400 // set started
#define BEEP_END    3400 // set ended
#define BEEP_ERROR  2400 // cancel

#define BEEP_ON_MS   150
#define BEEP_OFF_MS  100
static int  beepsLeft = 0;
static int  beepFreq = 0;
static unsigned long beepPhaseUntil = 0;
static bool beepOn = false;

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

// rep data
int repCount = 0;
int setNumber = 1;
int baselineMM = -1;

int liftMM = 0;
int velocity = 0;
int rawDistanceMM = -1;
int lastDistanceMM = -1;

int startHeight = 0;
int peakHeight = 0;
int valleyHeight = 0;

unsigned long timeStart = 0;
unsigned long timeValley = 0;
unsigned long restSince = 0;

//signal
static unsigned long lastValidMs = 0;
static const unsigned long SENSOR_TIMEOUT_MS = 500;


// function declarations:
float readMagnitude();
void IRAM_ATTR onButtonStartPressed();
void IRAM_ATTR onButtonStopPressed();
void drawStaticLayout();
bool initSensor();
void startSet();
void endSet();
void cancelSet();
void queueBeeps(int n, int freq);
void serviceBuzzer(unsigned long now);
int readDistanceMM();
void finishRep();

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
  if (!tofReady) {
    delay(500);
    return;
  }

  unsigned long now = millis();

  // button event control
  if (startPressed) {
    startPressed = false;
    Serial.println("Start pressed");
    if (setState == InProgress) {
      // IN PROGRESS -> COMPLETED
      endSet(); // COMPLETED -> NOT STARTED
    } else {
      // NOT STARTED -> IN PROGRESS
      startSet();
    }
  }

  if (stopPressed) {
    stopPressed = false;
    Serial.println("Stop pressed");
    if (setState == InProgress) {
      cancelSet(); // IN PROGRESS -> NOT STARTED
    } else {
      queueBeeps(1, BEEP_ERROR);
    }
  }

  // read sensor data
  int distanceMM = readDistanceMM();
  if (distanceMM > 0) {
    rawDistanceMM = distanceMM;
    lastValidMs = now;
    if (setState == InProgress && baselineMM > 0){
      //TODO collect data(lifrFromRaw(rawDistance))
      //TODO updateStateMachine
      Serial.printf("RAW,%lu,%d\n", now, distanceMM);
    }
  } else if (setState == InProgress && repState != Idle && (now - lastValidMs) > SENSOR_TIMEOUT_MS) {
    // sensor timeout, reset rep state
    Serial.println("Sensor timeout, resetting rep state");
    repState = Idle;
  }

  // BLE
  if (setState == Completed){ //actually transmission state
    // payload = format
    setNumber++;
    setState = NotStarted;
  }

  // real-time respond
  serviceBuzzer(now);

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
    startPressed = true;
    lastBTN1InterruptMs = now;
  }
}

void IRAM_ATTR onButtonStopPressed() {
  uint32_t now = millis();
  if ((now - lastBTN2InterruptMs) > BTN_DEBOUNCE_MS ){
    stopPressed = true;
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

void startSet() {
  // refresh sensor
  tof.stopContinuous();
  delay(10);
  tof.startContinuous(INTER_MEASUREMENT_MS);
  delay(80);

  int maxDistance = -1;
  int validCount = 0;
  unsigned long deadline = millis() + 800;


  while (millis() < deadline){
    int distance = readDistanceMM();
    if (distance > 0) {
      validCount++;
      if (distance > maxDistance) {
        maxDistance = distance;
      }
    }
  }

  if (validCount < 8) {
    Serial.println("Not enough valid distance initial readings, canceling set start");
    queueBeeps(3, BEEP_ERROR);
    return; // keep not started
  }

  baselineMM = maxDistance;
  setState = InProgress;
  repState = Idle;
  repCount = 0;
  resetSet();
  Serial.printf("Set %d started\n", setNumber);
  queueBeeps(1, BEEP_START);
}

void endSet() {
  Serial.printf("Set %d completed with %d reps\n", setNumber, repCount);
  setState = Completed;
  repState = Idle;
  queueBeeps(2, BEEP_END);
  // set number increment when BLE upload completed

}

void cancelSet() {
  Serial.printf("Set %d canceled with %d reps\n", setNumber, repCount);
  setState = NotStarted;
  repState = Idle;
  repCount = 0;
  queueBeeps(3, BEEP_ERROR);

}

void finishRep() {
  repCount++;

  if (repCount % 5 == 0) {
    queueBeeps(2, BEEP_REP + 500);
  } else {
    queueBeeps(1, BEEP_REP);
  }

  Serial.print("rep ");
  Serial.print(repCount);
}

void queueBeeps(int n, int freq) {
  beepsLeft = n;
  beepFreq = freq;
  beepOn = false;
  beepPhaseUntil = millis();
}

void serviceBuzzer(unsigned long now) {
  if (beepsLeft <= 0 && !beepOn) {
    return;
  }
  if (now < beepPhaseUntil) {
    return;
  }
  if (beepOn) {
    noTone(BUZZER_PIN);
    beepOn = false;
    beepPhaseUntil = now + BEEP_OFF_MS;
    beepsLeft--;
  } else if (beepsLeft > 0) {
    tone(BUZZER_PIN, beepFreq);
    beepOn = true;
    beepPhaseUntil = now + BEEP_ON_MS;
  }
}

int readDistanceMM() {
  if (!tof.dataReady()) {
    return -1;
  }
  // stop and read the data
  tof.read(false);

  if (tof.ranging_data.range_status != VL53L1X::RangeValid) {
    //invalid range, exceeding the maximum range
    return -1;
  }
  return (int)tof.ranging_data.range_mm;
}

void resetSet(){
  // reset the set data to 0
}


