#include <SPI.h>
#include <Servo.h>

// ---------------- USER SETTINGS ----------------
int thres = 1000;

// PID gains for RPM matching (followers track master)
double K_P = 0.01;
double K_I = 0.001;
double K_D = 0.00;

// "must be close for this long before declaring stable"
constexpr uint32_t RPM_STABLE_TIME_MS = 2000;   // 2 s
constexpr float    RPM_STABLE_BAND    = 30.0f;

// Control loop rate (SLOW loop)
// NOTE: 160000 us = 6.25 Hz. 50 Hz would be 20000 us.
constexpr uint32_t CONTROL_PERIOD_US = 160000;
constexpr float    DT = CONTROL_PERIOD_US * 1e-6f;

volatile float throttle_us = 1070.5f;
// Fast RPM measurement gate
constexpr uint32_t RPM_PERIOD_US = 7000; // adjust as you like

// ESC pulse limits
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 1200;

// Base throttle applied to MASTER
volatile int baseThrottleUs = 1100;

// SPI settings for AS5147P
constexpr uint32_t SPI_HZ   = 1000000;
constexpr uint8_t  SPI_MODE = SPI_MODE1;

// ---------------- PIN DEFINITIONS ----------------
// Master encoder CS + ESC pin
constexpr uint8_t CS_MASTER      = 4;   // <-- set
constexpr uint8_t ESC_MASTER_PIN = 5;   // <-- set (pick your pin)

// 3 followers
constexpr uint8_t N_FOLLOWERS = 3;
constexpr uint8_t CS_F[N_FOLLOWERS]  = {17, 20, 24 };  // <-- set
constexpr uint8_t ESC_F[N_FOLLOWERS] = {18, 21, 25};  // <-- set

// ---------------- INTERNAL STRUCTS ----------------
enum Mode { MODE_SPEED_MATCH, MODE_HOLD }; // HOLD = stable/locked (optional)

struct RpmState {
  uint16_t prevAngle = 0;
  uint32_t prevTimeUs = 0;
};

struct EmaState {
  float y = 0.0f;
  bool init = false;
};

struct FollowerCtrl {
  Servo esc;

  int throttleUs = 1000;

  float iTermUs = 0.0f;
  float prevErr = 0.0f;

  uint32_t stableStartMs = 0;
  bool stable = false;
  Mode mode = MODE_SPEED_MATCH;
};

static inline float wrapDeg180(float x) {
  while (x <= -180.0f) x += 360.0f;
  while (x >   180.0f) x -= 360.0f;
  return x;
}

// ---------------- SENSOR READ ----------------
uint16_t readAngle(uint8_t csPin) {
  uint16_t angle;

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE));

  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  SPI.transfer16(0xFFFF);
  digitalWrite(csPin, HIGH);

  delayMicroseconds(2);

  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  angle = SPI.transfer16(0x0000);
  digitalWrite(csPin, HIGH);

  SPI.endTransaction();

  return angle & 0x3FFF;
}

// Generic RPM from AS5147P raw angle (14-bit)
float computeRPM(uint16_t currentAngle, RpmState &st) {
  uint32_t nowUs = micros();

  if (st.prevTimeUs == 0) {
    st.prevAngle = currentAngle;
    st.prevTimeUs = nowUs;
    return 0.0f;
  }

  float dt = (nowUs - st.prevTimeUs) * 1e-6f;
  if (dt <= 0.0f) return 0.0f;

  int32_t dAngle = (int32_t)currentAngle - (int32_t)st.prevAngle;

  // wrap
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  float rev = (float)dAngle / 16384.0f;
  float rpm = (rev / dt) * 60.0f;

  st.prevAngle = currentAngle;
  st.prevTimeUs = nowUs;

  return fabsf(rpm);
}

float ema(float x, EmaState &st, float alpha) {
  if (!st.init) { st.y = x; st.init = true; }
  else         { st.y += alpha * (x - st.y); }
  return st.y;
}

// ---------------- GLOBAL OBJECTS ----------------
Servo escMaster;

RpmState  rpmMasterState;
RpmState  rpmFollowerState[N_FOLLOWERS];

EmaState  emaMasterState;
EmaState  emaFollowerState[N_FOLLOWERS];

FollowerCtrl follower[N_FOLLOWERS];

// EMA smoothing
const float ALPHA_MASTER   = 0.04f;
const float ALPHA_FOLLOWER = 0.04f;

// Latest measurements published from fast gate to slow gate
static uint16_t rawMaster_latest = 0;
static float    rpmMaster_f_latest = 0.0f;

static uint16_t rawF_latest[N_FOLLOWERS] = {0,0,0};
static float    rpmF_f_latest[N_FOLLOWERS] = {0,0,0};

// ---------------- SETUP ----------------
void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line == "?") {
    Serial.print("throttle_us = ");
    Serial.println(throttle_us, 2);
    return;
  }
  /*
  if (line.equalsIgnoreCase("arm")) {
    Serial.println("Arming: 1000us for 3s...");
    writeServoPulseUS(1000.0f);
    delay(3000);
    Serial.print("Back to throttle_us = ");
    Serial.println(throttle_us, 2);
    return;
  }
  */

  // If it starts with 't' or 'T', skip it and parse the rest
  if (line[0] == 't' || line[0] == 'T') {
    line.remove(0, 1);
    line.trim();
  }

  float v = line.toFloat(); // NOTE: returns 0.0 if it can't parse
  // Basic guard: if user truly wants 0, they can still type "t 1000" etc.
  if (v < 500.0f) {
    Serial.println("Parse failed or too small. Send like: 1100.5  or  t 1100.5");
    return;
  }

  // Clamp to ESC range you want
  if (v < ESC_MIN_US) v = ESC_MIN_US;
  if (v > ESC_MAX_US) v = ESC_MAX_US;

  throttle_us = v;

  Serial.print("New throttle_us = ");
  Serial.println(throttle_us, 2);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;  // wait for USB
  Serial.println("Ready");
  // CS pins
  pinMode(CS_MASTER, OUTPUT);
  digitalWrite(CS_MASTER, HIGH);

  for (uint8_t i = 0; i < N_FOLLOWERS; i++) {
    pinMode(CS_F[i], OUTPUT);
    digitalWrite(CS_F[i], HIGH);
  }

  SPI.begin();

  // ESCs
  escMaster.attach(ESC_MASTER_PIN, 1000, 2000);
  escMaster.writeMicroseconds(1000);
  delay(3000); //delay before arming during testing.
  for (uint8_t i = 0; i < N_FOLLOWERS; i++) {
    follower[i].esc.attach(ESC_F[i], 1000, 2000);
    follower[i].esc.writeMicroseconds(1000);
    follower[i].throttleUs = baseThrottleUs; // start near master
  }

  delay(3000);

  // Start master at base throttle
  escMaster.writeMicroseconds(baseThrottleUs);
}

// ---------------- LOOP ----------------
void loop() {

  static uint32_t lastRpmUs  = micros();
  static uint32_t lastCtrlUs = lastRpmUs;

  uint32_t nowUs = micros();

  // ---------- FAST RPM UPDATE ----------
  if ((uint32_t)(nowUs - lastRpmUs) >= RPM_PERIOD_US) {
    lastRpmUs += RPM_PERIOD_US;

    // Master
    uint16_t rawM = readAngle(CS_MASTER);
    float rpmM    = computeRPM(rawM, rpmMasterState);
    float rpmM_f  = ema(rpmM, emaMasterState, ALPHA_MASTER);

    rawMaster_latest = rawM;
    rpmMaster_f_latest = rpmM_f;

    // Followers
    for (uint8_t i = 0; i < N_FOLLOWERS; i++) {
      uint16_t rawFi = readAngle(CS_F[i]);
      float rpmFi    = computeRPM(rawFi, rpmFollowerState[i]);
      float rpmFi_f  = ema(rpmFi, emaFollowerState[i], ALPHA_FOLLOWER);

      rawF_latest[i] = rawFi;
      rpmF_f_latest[i] = rpmFi_f;
    }
  }

  // ---------- SLOW CONTROL LOOP ----------
  if ((uint32_t)(nowUs - lastCtrlUs) < CONTROL_PERIOD_US) return;
  lastCtrlUs += CONTROL_PERIOD_US;

  uint32_t nowMs = millis();

  float rpmM_f = rpmMaster_f_latest;
  handleSerial();
  // Keep master at base throttle (you can change baseThrottleUs externally)
  escMaster.writeMicroseconds(throttle_us);

  // If master too slow, reset followers
  if (rpmM_f < thres) {
    for (uint8_t i = 0; i < N_FOLLOWERS; i++) {
      follower[i].iTermUs = 0.0f;
      follower[i].prevErr = 0.0f;
      follower[i].stableStartMs = 0;
      follower[i].stable = false;
      follower[i].mode = MODE_SPEED_MATCH;

      follower[i].throttleUs = baseThrottleUs;
      follower[i].esc.writeMicroseconds(follower[i].throttleUs);
    }

    Serial.print("MASTER below threshold, rpmM=");
    Serial.println(rpmM_f);
    return;
  }

  // Debug print (control rate)
  Serial.print("rpmM="); Serial.println(rpmM_f);

  for (uint8_t i = 0; i < N_FOLLOWERS; i++) {
    float rpmFi_f = rpmF_f_latest[i];
    float e = (rpmM_f - rpmFi_f);

    // stability logic per follower
    if (fabsf(e) <= RPM_STABLE_BAND) {
      if (follower[i].stableStartMs == 0) follower[i].stableStartMs = nowMs;
      if ((nowMs - follower[i].stableStartMs) >= RPM_STABLE_TIME_MS) {
        follower[i].stable = true;
        follower[i].mode = MODE_HOLD; // optional
      }
    } else {
      follower[i].stableStartMs = 0;
      follower[i].stable = false;
      follower[i].mode = MODE_SPEED_MATCH;
    }

    // Control (you can choose to still “micro-correct” in HOLD if you want)
    if (follower[i].mode == MODE_SPEED_MATCH) {
      follower[i].iTermUs += (K_I * e) * DT;
      follower[i].iTermUs = constrain(follower[i].iTermUs, -200.0f, 200.0f);

      float dErr = (e - follower[i].prevErr) / DT;
      follower[i].prevErr = e;

      float pTerm = (float)(K_P * e);
      pTerm = constrain(pTerm, -1.0f, 1.0f);

      float dTerm = (float)(K_D * dErr);

      float u = pTerm + follower[i].iTermUs + dTerm;

      follower[i].throttleUs = constrain((int)(follower[i].throttleUs + u), ESC_MIN_US, ESC_MAX_US);
      follower[i].esc.writeMicroseconds(follower[i].throttleUs);
    } else {
      // MODE_HOLD: do nothing OR keep writing last throttle
      follower[i].esc.writeMicroseconds(follower[i].throttleUs);
    }

    Serial.print("F"); Serial.print(i+1);
    Serial.print(" rpm="); Serial.print(rpmFi_f);
    Serial.print(" e="); Serial.print(e);
    Serial.print(" thr="); Serial.print(follower[i].throttleUs);
    Serial.print(" stable="); Serial.println(follower[i].stable ? 1 : 0);
  }
}