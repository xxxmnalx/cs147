#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SparkFunLSM6DSO.h"

#include "cs147_common.h"
#include "secrets.h"

//pins
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22
#define BUZZER_PIN    25
#define BTN_START_PIN 26
#define BTN_STOP_PIN  27

//upload

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

// The server decodes this stream on a fixed 5 byte stride. If alignment
// padding ever creeps back in, every field after the first sample shifts and
// the checksum fails with no obvious cause. Catch it at compile time.
static_assert(sizeof(Sample) == 5, "Sample must stay packed at 5 bytes");

#define MAX_SAMPLES 8000
static uint8_t sampleBuf[MAX_SAMPLES * sizeof(Sample)];   // 40 KB

uint32_t bufferChecksum() {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < bufferUsed; i++) {
    sum = (sum << 1) ^ (sum >> 31) ^ sampleBuf[i];   // cheap rotate-xor
  }
  return sum;
}


constexpr int SAMPLE_BYTES = sizeof(Sample);

constexpr uint32_t DISPLAY_REFRESH_MS = 100;

// Which of the three full-screen layouts is on the panel right now.
enum Screen { ScreenNormal, ScreenSending, ScreenDone };
static Screen lastScreen = ScreenNormal;

// How long "COMPLETED" stays up before the panel returns to idle.
constexpr uint32_t DONE_HOLD_MS = 2000;
static uint32_t doneUntilMs = 0;

// One dot every SEND_ANIM_MS while the upload blocks the main loop.
constexpr uint32_t SEND_ANIM_MS = 300;
static TaskHandle_t   sendAnimHandle = nullptr;
static volatile bool  sendAnimRun    = false;
static volatile bool  sendAnimIdle   = true;

constexpr uint32_t WIFI_TIMEOUT_MS   = 15000;
constexpr uint32_t UPLOAD_TIMEOUT_MS = 10000;
constexpr uint32_t UPLOAD_RETRY_MS   = 5000;

uint32_t lastDisplayMs = 0;
uint32_t lastUploadMs = 0;


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
void resetSmoothing();
int liftFromRaw(int rawDistance);
void smoothSample(int rawHeight, unsigned long now);
void finishRep(unsigned long endTime);
void beginLift(int height, unsigned long t);
void updateStateMachine(unsigned long now);
void recordSample(unsigned long now, int16_t height, int16_t accelMg);
void connectWiFi();
bool uploadSet();
void updateDisplay();
void drawSendingScreen(int dots);
void drawDoneScreen();
void sendAnimTask(void *arg);
void startSendAnimation();
void stopSendAnimation();

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

  connectWiFi();

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
    if (setState == NotStarted) {
      // NOT STARTED -> IN PROGRESS
      startSet();
    } else if (setState == InProgress) {
      // IN PROGRESS -> COMPLETED
      endSet();
    } else {
      // COMPLETED or TRANSMITTING: the set is still unsent, refuse to start
      // a new one rather than discard it.
      queueBeeps(3, BEEP_ERROR);
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
    // Discard pre-dropout samples: the moving average would otherwise blend
    // across the gap and fabricate motion that never happened.
    resetSmoothing();
  }

  // HTTP upload. Without a link the set stays buffered and nothing is lost.
  // Retries are spaced out so an unreachable server cannot pin the loop.
  if (setState == Completed && WiFi.status() == WL_CONNECTED &&
      (now - lastUploadMs) >= UPLOAD_RETRY_MS) {
    setState = Transmitting;
    updateDisplay();       // hand the panel over to the sending screen
    startSendAnimation();  // dots keep moving while the POST blocks this task

    bool sent = uploadSet();
    stopSendAnimation();   // reclaim the panel before painting the result

    if (sent) {
      setNumber++;
      // resetSignal() clears the waveform but not the tally, and startSet()
      // is too late: the idle screen would still show the shipped set's count.
      repCount = 0;
      setState = NotStarted;
      resetSignal();
      doneUntilMs = millis() + DONE_HOLD_MS;
      queueBeeps(2, BEEP_END);
    } else {
      setState = Completed;   // keep the buffer, the set can be retried
      queueBeeps(3, BEEP_ERROR);
    }
    // Stamped after the POST, not before: a timeout burns the whole budget
    // and would otherwise leave no gap at all before the next try.
    lastUploadMs = millis();
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
  int secondMax = -1;
  int validCount = 0;
  float accelSum = 0;
  int accelCount = 0;
  unsigned long deadline = millis() + 800;


  while (millis() < deadline){
    int distance = readDistanceMM();
    if (distance > 0) {
      validCount++;
      if (distance > maxDistance) {
        secondMax = maxDistance;
        maxDistance = distance;
      } else if (distance > secondMax) {
        secondMax = distance;
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

  // Second largest, so a single ToF spike cannot bias the whole set.
  baselineMM = secondMax;
  setState = InProgress;
  repState = Idle;
  repCount = 0;
  resetSignal();
  Serial.printf("Set %d started, baseline=%dmm, restingG=%.3f\n", setNumber, baselineMM, restingG);
  queueBeeps(1, BEEP_START);
}

void endSet() {
  // The set can end mid-descent, before REST_CONFIRM_MS confirms the rest.
  // Close that rep out here or it is lost. repState is reset just below,
  // so this cannot double-count.
  if (repState == Down) {
    finishRep(timeValley);
  }

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
  resetSmoothing();
  liftMM    = 0;
  velocity  = 0;

  accelPeak    = 0;
  bufferUsed   = 0;
  lastRecordMs = 0;
  bufferFull   = false;
}

void resetSmoothing(){
  // Drop the moving-average and velocity history without touching set data.
  rawCount  = 0;
  histCount = 0;
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


void connectWiFi() {
  // Bounded wait: rep counting has to work with no hotspot around, so a
  // failure here must not hold up the rest of setup().
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");

  unsigned long deadline = millis() + WIFI_TIMEOUT_MS;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi unavailable, running offline");
  }
}

bool uploadSet() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Upload skipped, WiFi down");
    return false;
  }

  // Metadata rides in the query string, the waveform is the raw body.
  char url[192];
  snprintf(url, sizeof(url),
           "http://%s:%d/api/upload?set=%d&reps=%d&base=%d&restg=%.3f&n=%lu&crc=%lu",
           SERVER_HOST, SERVER_PORT, setNumber, repCount, baselineMM, restingG,
           (unsigned long)(bufferUsed / SAMPLE_BYTES),
           (unsigned long)bufferChecksum());

  HTTPClient http;
  http.setTimeout(UPLOAD_TIMEOUT_MS);
  if (!http.begin(url)) {
    Serial.println("HTTP begin failed");
    return false;
  }
  http.addHeader("Content-Type", "application/octet-stream");

  // POST straight out of sampleBuf, no intermediate copy.
  int code = http.POST(sampleBuf, bufferUsed);
  http.end();

  Serial.printf("UPLOAD,%d,%lu\n", code, (unsigned long)bufferUsed);
  return code >= 200 && code < 300;
}

void drawSendingScreen(int dots) {
  // Nothing else on the panel: the device is busy and this is the only thing
  // worth saying. Padding to the full width erases the previous dot count.
  char label[16];
  snprintf(label, sizeof(label), "SENDING%.*s", dots, "...");
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(displayW);
  tft.drawString(label, displayW / 2, displayH / 2, 4);
}

void drawDoneScreen() {
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(displayW);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("COMPLETED", displayW / 2, displayH / 2 - 16, 4);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Ready 4 Next Sets", displayW / 2, displayH / 2 + 16, 2);
}

void sendAnimTask(void *arg) {
  int dots = 1;
  while (sendAnimRun) {
    drawSendingScreen(dots);
    dots = (dots % 3) + 1;
    // Wakes early when stopSendAnimation() signals, so the result screen is
    // not held back by a frame that is still counting down.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SEND_ANIM_MS));
  }
  sendAnimHandle = nullptr;
  sendAnimIdle = true;
  vTaskDelete(nullptr);
}

void startSendAnimation() {
  if (!sendAnimIdle) {
    return;
  }
  sendAnimRun = true;
  sendAnimIdle = false;
  // Pinned to the core the main loop runs on. That task is parked inside the
  // POST for the whole window, so the panel has exactly one writer either way.
  xTaskCreatePinnedToCore(sendAnimTask, "sendAnim", 4096, nullptr, 1, &sendAnimHandle, 1);
}

void stopSendAnimation() {
  sendAnimRun = false;
  TaskHandle_t handle = sendAnimHandle;
  if (handle != nullptr) {
    xTaskNotifyGive(handle);
  }
  // Wait for the task to let go of the panel, but never hang on it.
  uint32_t guard = millis() + 1000;
  while (!sendAnimIdle && millis() < guard) {
    delay(5);
  }
}

void updateDisplay() {
  static int lastDrawnCount = -1;
  static int lastDrawnSet = -1;
  static SetState lastDrawnState = NotStarted;

  // Starting a new set cuts the completion screen short.
  if (doneUntilMs != 0 && setState != NotStarted) {
    doneUntilMs = 0;
  }

  Screen want = ScreenNormal;
  if (setState == Transmitting) {
    want = ScreenSending;
  } else if (doneUntilMs != 0) {
    if (millis() < doneUntilMs) {
      want = ScreenDone;
    } else {
      doneUntilMs = 0;
    }
  }

  if (want != lastScreen) {
    tft.fillScreen(TFT_BLACK);
    lastScreen = want;
    // Whatever comes next has to repaint from scratch over the cleared panel.
    lastDrawnCount = -1;
    lastDrawnSet   = -1;
    if (want == ScreenNormal) {
      drawStaticLayout();
    } else if (want == ScreenDone) {
      drawDoneScreen();
    }
  }

  if (want == ScreenSending) {
    return;   // the animation task owns the panel until the POST returns
  }
  if (want == ScreenDone) {
    return;   // static until it expires
  }

  if (repCount == lastDrawnCount && setNumber == lastDrawnSet
      && setState == lastDrawnState) {
    return;
  }
  lastDrawnCount = repCount;
  lastDrawnSet   = setNumber;
  lastDrawnState = setState;

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(120);
  tft.drawNumber(repCount, 6, 24, 7);

  // Set counter, top right, balancing the REPS label on the left.
  char setLabel[12];
  snprintf(setLabel, sizeof(setLabel), "SET %d", setNumber);
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(70);
  tft.drawString(setLabel, displayW - 6, 2, 2);
  tft.setTextDatum(TL_DATUM);

  const char* status = "IDLE";
  if (setState == InProgress)        status = "RECORDING";
  else if (setState == Completed)    status = "WAITING";
  else if (setState == Transmitting) status = "SENDING";

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextPadding(displayW - 12);
  tft.drawString(status, 6, 110, 2);
}