#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <TFT_eSPI.h>
#include "SparkFunLSM6DSO.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "cs147_common.h"

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
BLE2902 *pNotifyCCCD = nullptr;
BLEServer *pServer = nullptr;
static volatile bool bleConnected = false;
static uint16_t bleMTU = 23;
static uint32_t txOffset = 0;
static uint16_t txSeq = 0;

constexpr int MAX_CHUNK_BYTES = 128;

//IMU
LSM6DSO myIMU;
static boolean IMUReady           = false;
static unsigned long lastImuMs    = 0;
static float accelPeak            = 0;
static uint32_t bufferUsed        = 0;
static unsigned long lastRecordMs = 0;
static bool bufferFull            = false;
static float restingG             = 1.0f;

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
enum SetState { NotStarted, InProgress, Completed, Transmitting };
SetState setState = NotStarted;

// rep data
int repCount = 0;
int setNumber = 1;
int baselineMM = -1;

int liftMM = 0;
int velocity = 0;

int startHeight = 0;
int peakHeight = 0;
int valleyHeight = 0;

unsigned long timeStart = 0;
unsigned long timeValley = 0;
unsigned long restSince = 0;

static const int SMOOTH_N = 3;
static const int VELOCITY_WINDOW  = 6;

static int rawBuf[SMOOTH_N];
static int smoothHeightBuf[VELOCITY_WINDOW];
static unsigned long sampleTimeBuf[VELOCITY_WINDOW];
static int rawCount  = 0;
static int histCount = 0;

//signal
static unsigned long lastValidMs = 0;
static const unsigned long SENSOR_TIMEOUT_MS = 500;

//detection thresholds
static const int HYSTERESIS_MM  = 10;
static const int START_MM       = 20;
static const int RETURN_BAND_MM = 25;
static const int MIN_ROM_MM     = 50;

static const unsigned long MIN_REP_MS      = 400;
static const unsigned long REST_CONFIRM_MS = 400;

static const int VEL_MOVING = 40;   // mm/s, above 40 is moving
static const int VEL_STILL  = 25;   // mm/s, below 25 is at rest

static const unsigned long IMU_INTERVAL_MS = 5;

// One recorded sample
struct __attribute__((packed)) Sample {
  uint8_t dt;
  int16_t height;
  int16_t accel;
};

enum PktType : uint8_t {
  PKT_SAMPLES     = 0x01,
  PKT_SET_SUMMARY = 0x02,
  PKT_TX_END      = 0x03,
  PKT_REP         = 0x05,
};

struct __attribute__((packed)) PktHeader {
  uint8_t  type;
  uint16_t seq;
  uint8_t  len;
};

struct __attribute__((packed)) SetSummary {
  PktHeader hdr;
  uint16_t setNumber;
  uint16_t repCount;
  int16_t  baselineMM;
  uint16_t sampleCount;
  uint32_t checksum;     // XOR-fold over sampleBuf
};

#define MAX_SAMPLES 8000
static uint8_t sampleBuf[MAX_SAMPLES * sizeof(Sample)];   // 40 KB

uint32_t bufferChecksum() {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < bufferUsed; i++) {
    sum = (sum << 1) ^ (sum >> 31) ^ sampleBuf[i];   // cheap rotate-xor
  }
  return sum;
}


constexpr int BLE_ATT_OVERHEAD_BYTES = 3;
constexpr int PACKET_HEADER_BYTES    = 4;
constexpr int SAMPLE_BYTES           = sizeof(Sample);

constexpr uint32_t TX_INTERVAL_MS     = 25;
constexpr uint32_t DISPLAY_REFRESH_MS = 100;

uint32_t lastTxMs = 0;
uint32_t lastDisplayMs = 0;


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
void resetSignal();
int liftFromRaw(int rawDistance);
void smoothSample(int rawHeight, unsigned long now);
void finishRep(unsigned long endTime);
void beginLift(int height, unsigned long t);
void updateStateMachine(unsigned long now);
void recordSample(unsigned long now, int16_t height, int16_t accelMg);
int usablePayloadBytes(int mtuBytes);
int calculateSamplesPerPacket(int payloadBytes);
int calculatePacketBytes(int samplesPerPacket);
int calculatePacketsRequired(uint32_t totalBytes, int packetBytes);
void printTransmissionModel(int mtuBytes, int payloadBytes, int samplesPerPacket, int packetBytes, int packetsRequired);
bool bleNotifyReady();
void sendNextChunk();
void updateDisplay();

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    bleConnected = true;
  }
  void onDisconnect(BLEServer *pServer) override {
    bleConnected = false;
    // Restart advertising, or the device is invisible after first disconnect.
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting StackSense");

  //DEBUG
  Serial.printf("Sample size: %u\n", sizeof(Sample));

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
  BLEDevice::setMTU(517);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);


  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  pNotifyCCCD = new BLE2902();
  pCharacteristic->addDescriptor(pNotifyCCCD);
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

  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

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

  // IMU peak-hold: keep the largest magnitude since the last recorded sample
  if (IMUReady && setState == InProgress && (now - lastImuMs >= IMU_INTERVAL_MS)) {
    lastImuMs = now;
    float dev = readMagnitude() - restingG;
    if (fabsf(dev) > fabsf(accelPeak)) {
      accelPeak = dev;
    }
  }

  // read sensor data
  int distanceMM = readDistanceMM();
  if (distanceMM > 0) {
    lastValidMs = now;
    if (setState == InProgress && baselineMM > 0){
      int rawLift = liftFromRaw(distanceMM);
      smoothSample(rawLift, now);
      //DEBUG
      Serial.printf("RAW,%lu,%d,%d,%d,%d\n", now, distanceMM, liftMM, velocity,(int)(accelPeak * 1000.0f));
      recordSample(now, (int16_t)rawLift, (int16_t)(accelPeak * 1000.0f));
      accelPeak = 0;
      updateStateMachine(now);
    }
  } else if (setState == InProgress && repState != Idle && (now - lastValidMs) > SENSOR_TIMEOUT_MS) {
    // sensor timeout, reset rep state
    Serial.println("Sensor timeout, resetting rep state");
    repState = Idle;
  }

  // BLE transmission
  if (setState == Completed && bleNotifyReady()) {
    bleMTU = pServer->getPeerMTU(pServer->getConnId());

    const int payloadBytes     = usablePayloadBytes(bleMTU);
    const int samplesPerPacket = calculateSamplesPerPacket(payloadBytes);
    const int packetBytes      = calculatePacketBytes(samplesPerPacket);
    const int packetsRequired  = calculatePacketsRequired(bufferUsed, packetBytes);

    printTransmissionModel(bleMTU, payloadBytes, samplesPerPacket,
                           packetBytes, packetsRequired);

    txOffset = 0;
    txSeq = 0;
    setState = Transmitting;
  }

  if (setState == Transmitting) {
    if (!bleNotifyReady()) {
      setState = Completed;      // link dropped, restart from scratch
    } else if (cs147Every(TX_INTERVAL_MS, lastTxMs)) {
      sendNextChunk();
    }
  }

  if (cs147Every(DISPLAY_REFRESH_MS, lastDisplayMs)) {
    updateDisplay();
  }

  // real-time respond
  serviceBuzzer(now);

}




// function definitions:
float readMagnitude() {
  // Magnitude of  acceleration
  float ax = myIMU.readFloatAccelX();
  float ay = myIMU.readFloatAccelY();
  float az = myIMU.readFloatAccelZ();
  return sqrtf(ax*ax + ay*ay + az*az);
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
  float accelSum = 0;
  int accelCount = 0;
  unsigned long deadline = millis() + 800;


  while (millis() < deadline){
    int distance = readDistanceMM();
    if (distance > 0) {
      validCount++;
      if (distance > maxDistance) {
        maxDistance = distance;
      }
    }

    if (IMUReady) {
      float m = readMagnitude();
      accelSum += m;
      accelCount++;
    }

  }

  if (validCount < 8) {
    Serial.println("Not enough valid distance initial readings, canceling set start");
    queueBeeps(3, BEEP_ERROR);
    return; // keep not started
  }

  if (accelCount > 0) {
    restingG = accelSum / accelCount;
  }

  baselineMM = maxDistance;
  setState = InProgress;
  repState = Idle;
  repCount = 0;
  resetSignal();
  Serial.printf("Set %d started, baseline=%dmm, restingG=%.3f\n", setNumber, baselineMM, restingG);
  queueBeeps(1, BEEP_START);
}

void endSet() {
  Serial.printf("Set %d completed with %d reps\n", setNumber, repCount);
  Serial.printf("SET,%d,%d,%lu,%d\n", setNumber, repCount, (unsigned long)(bufferUsed / SAMPLE_BYTES), bufferFull ? 1 : 0);
  setState = Completed;
  repState = Idle;
  queueBeeps(2, BEEP_END);
}

void cancelSet() {
  Serial.printf("Set %d canceled with %d reps\n", setNumber, repCount);
  setState = NotStarted;
  repState = Idle;
  repCount = 0;
  resetSignal();
  queueBeeps(3, BEEP_ERROR);

}

void finishRep(unsigned long endTime) {
  int rom   = peakHeight - startHeight;
  unsigned long total = endTime - timeStart;

  if (rom < MIN_ROM_MM || total < MIN_REP_MS) {
    Serial.printf("REJECT,rom=%d,total=%lu\n", rom, total);
    return;
  }

  repCount++;

  if (repCount % 5 == 0) {
    queueBeeps(2, BEEP_REP);
  } else {
    queueBeeps(1, BEEP_REP);
  }

  Serial.printf("REP,%d,%d,%lu\n", repCount, rom, total);

  // Live notification for the demo. If it fails or the phone is away, the
  // waveform still ships in the batch upload after the set.
  if (bleNotifyReady()) {
    struct __attribute__((packed)) {
      PktHeader hdr;
      uint16_t repIndex;
      uint16_t romMM;
      uint16_t durationMS;
    } pkt = { { PKT_REP, 0, 6 }, (uint16_t)repCount, (uint16_t)rom, (uint16_t)total };

    pCharacteristic->setValue((uint8_t*)&pkt, sizeof(pkt));
    pCharacteristic->notify();
  }
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

void resetSignal(){
  // reset the set data to 0
  rawCount  = 0;
  histCount = 0;
  liftMM    = 0;
  velocity  = 0;

  accelPeak    = 0;
  bufferUsed   = 0;
  lastRecordMs = 0;
  bufferFull   = false;
}

int liftFromRaw(int rawDistance){
  //return relative height from baseline
  return baselineMM - rawDistance;
}


void smoothSample(int unsmoothHeight, unsigned long now) {
  // Smooth the height with a 3-sample moving average, then estimate velocity
  // from the endpoints of a 6-sample window. The long time base keeps
  // differentiation from amplifying sensor noise.
  for (int i = SMOOTH_N - 1; i > 0; i--) {
    rawBuf[i] = rawBuf[i - 1];
  }
  rawBuf[0] = unsmoothHeight;
  if (rawCount < SMOOTH_N) {
    rawCount++;
  }

    long sum = 0;
  for (int i = 0; i < rawCount; i++) {
    sum += rawBuf[i];
  }
  liftMM = (int)(sum / rawCount);

  for (int i = VELOCITY_WINDOW - 1; i > 0; i--) {
    smoothHeightBuf[i] = smoothHeightBuf[i - 1];
    sampleTimeBuf[i] = sampleTimeBuf[i - 1];
  }
  smoothHeightBuf[0] = liftMM;
  sampleTimeBuf[0] = now;
  if (histCount < VELOCITY_WINDOW) {
    histCount++;
  }

  if (histCount >= 2) {
    int  oldest = histCount - 1;
    long dt = (long)(sampleTimeBuf[0] - sampleTimeBuf[oldest]);
    if (dt > 0) {
      velocity = (int)(1000L * (long)(smoothHeightBuf[0] - smoothHeightBuf[oldest]) / dt);
    }
  } else {
    velocity = 0;
  }


}

void beginLift(int h, unsigned long t) {
  repState    = Up;
  startHeight = h;
  timeStart   = t;
  peakHeight  = h;
}

void updateStateMachine(unsigned long now){
  if (histCount < VELOCITY_WINDOW) return;   // velocity not trustworthy yet

  int h = liftMM;
  int v = velocity;

  switch (repState) {
    case Idle:
    if (h > START_MM && v > VEL_MOVING) {
      beginLift(0,sampleTimeBuf[histCount - 1]);
    }
    break;

    case Up:
      if (h > peakHeight) {
        peakHeight = h;
      }

      if (h < (peakHeight - HYSTERESIS_MM)) {
        // stack is coming down, transition to down state
        repState = Down;
        timeValley = now;
        valleyHeight = h;
        restSince = 0;
      }
      break;

    case Down:
      if (h < valleyHeight) {
        valleyHeight = h;
        timeValley = now;
      }

      if((h > valleyHeight + HYSTERESIS_MM) && (v > VEL_MOVING)){
        // stack is going up again, one rep completed
        finishRep(timeValley);
        beginLift(valleyHeight, timeValley);
        break;
      }

      if ((h < START_MM + RETURN_BAND_MM) && abs(v) < VEL_STILL){
        // stack is at rest, transition to idle state
        if (restSince == 0) {
          restSince = now;
        } else if ((now - restSince) > REST_CONFIRM_MS) {
          finishRep(timeValley);
          repState = Idle;
          restSince = 0;
        }
      } else {
        restSince = 0;
      }
      break;
  }

}

void recordSample(unsigned long now, int16_t height, int16_t accelMg) {
  if (bufferUsed + sizeof(Sample) > sizeof(sampleBuf)) {
    bufferFull = true;   // stop recording waveform, but keep counting reps
    return;
  }

  unsigned long delta = (lastRecordMs == 0) ? 0 : (now - lastRecordMs);
  if (delta > 255) delta = 255;
  lastRecordMs = now;

  Sample s = { (uint8_t)delta, height, accelMg };
  memcpy(&sampleBuf[bufferUsed], &s, sizeof(s));
  bufferUsed += sizeof(s);
}


int usablePayloadBytes(int mtuBytes) {
  // Guard against getPeerMTU() returning 0 before the connection settles.
  if (mtuBytes < 23) {
    mtuBytes = 23;   // BLE default minimum
  }

  int payloadBytes = mtuBytes - BLE_ATT_OVERHEAD_BYTES - PACKET_HEADER_BYTES;

  if (payloadBytes > MAX_CHUNK_BYTES - PACKET_HEADER_BYTES) {
    payloadBytes = MAX_CHUNK_BYTES - PACKET_HEADER_BYTES;
  }
  return payloadBytes;
}

int calculateSamplesPerPacket(int payloadBytes) {
  //from C12
  int samplesPerPacket = payloadBytes / SAMPLE_BYTES;
  return samplesPerPacket;
}

int calculatePacketBytes(int samplesPerPacket) {
  //from C12
  int packetBytes = samplesPerPacket * SAMPLE_BYTES;
  return packetBytes;
}

int calculatePacketsRequired(uint32_t totalBytes, int packetBytes) {
  //from C12
  int packetsRequired = (totalBytes + packetBytes - 1) / packetBytes;
  return packetsRequired;
}

void printTransmissionModel(int mtuBytes, int payloadBytes, int samplesPerPacket, int packetBytes, int packetsRequired) {
  Serial.println();
  Serial.println("---------- Transmission Model ----------");
  Serial.print("Negotiated MTU: ");
  Serial.print(mtuBytes);
  Serial.println(" bytes");
  Serial.print("Protocol overhead: ");
  Serial.print(BLE_ATT_OVERHEAD_BYTES + PACKET_HEADER_BYTES);
  Serial.println(" bytes");
  Serial.print("Usable payload: ");
  Serial.print(payloadBytes);
  Serial.println(" bytes");
  Serial.print("Samples per packet: ");
  Serial.println(samplesPerPacket);
  Serial.print("Bytes per packet: ");
  Serial.print(packetBytes);
  Serial.println(" bytes");
  Serial.print("Buffered data: ");
  Serial.print(bufferUsed);
  Serial.println(" bytes");
  Serial.print("Packets required: ");
  Serial.println(packetsRequired);
  Serial.print("Result: ");
  Serial.println(packetsRequired == 1 ? "FITS IN ONE PACKET" : "FRAGMENTATION REQUIRED");
}

bool bleNotifyReady() {
  return bleConnected &&
         pNotifyCCCD != nullptr &&
         pNotifyCCCD->getNotifications();
}

void sendNextChunk() {
  if (!bleNotifyReady()) {
    setState = Completed;
    return;
  }

  const int packetBytes =
      calculatePacketBytes(calculateSamplesPerPacket(usablePayloadBytes(bleMTU)));

  if (txOffset >= bufferUsed) {
    SetSummary s;
    s.hdr = { PKT_SET_SUMMARY, txSeq, sizeof(SetSummary) - sizeof(PktHeader) };
    s.setNumber   = setNumber;
    s.repCount    = repCount;
    s.baselineMM  = baselineMM;
    s.sampleCount = bufferUsed / SAMPLE_BYTES;
    s.checksum    = bufferChecksum();
    pCharacteristic->setValue((uint8_t*)&s, sizeof(s));
    pCharacteristic->notify();

    delay(30);   // let the stack drain before the end marker

    PktHeader endPkt = { PKT_TX_END, 0, 0 };
    pCharacteristic->setValue((uint8_t*)&endPkt, sizeof(endPkt));
    pCharacteristic->notify();

    Serial.print("Transmission complete, fragments sent: ");
    Serial.println(txSeq);

    setNumber++;
    setState = NotStarted;
    resetSignal();
    return;
  }

  int len = packetBytes;
  if (txOffset + len > bufferUsed) {
    len = bufferUsed - txOffset;
  }

  uint8_t pkt[MAX_CHUNK_BYTES];
  PktHeader h = { PKT_SAMPLES, txSeq, (uint8_t)len };
  memcpy(pkt, &h, sizeof(h));
  memcpy(pkt + sizeof(h), &sampleBuf[txOffset], len);

  pCharacteristic->setValue(pkt, sizeof(h) + len);
  pCharacteristic->notify();

  txOffset += len;
  txSeq++;
}

void updateDisplay() {
  static int lastDrawnCount = -1;
  static SetState lastDrawnState = NotStarted;
  static int lastDrawnPct = -1;

  int pct = 0;
  if (setState == Transmitting && bufferUsed > 0) {
    pct = (int)((uint64_t)txOffset * 100 / bufferUsed);
  }

  if (repCount == lastDrawnCount && setState == lastDrawnState && pct == lastDrawnPct) {
    return;
  }
  lastDrawnCount = repCount;
  lastDrawnState = setState;
  lastDrawnPct = pct;

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(120);
  tft.drawNumber(repCount, 6, 24, 7);

  const char* status = "IDLE";
  if (setState == InProgress)        status = "RECORDING";
  else if (setState == Completed)    status = "WAITING BLE";
  else if (setState == Transmitting) status = "SENDING";

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextPadding(displayW - 12);
  tft.drawString(status, 6, 110, 2);

  // Progress bar inside the frame drawn by drawStaticLayout()
  const int barW = displayW - 16;
  tft.fillRect(8, 94, barW, 8, TFT_BLACK);
  if (pct > 0) {
    tft.fillRect(8, 94, barW * pct / 100, 8, TFT_GREEN);
  }
}