#include <SPI.h>
#include <Servo.h>

// -------------------- USER CONFIG --------------------
// Encoder CS pins (shared SPI bus)
constexpr uint8_t CS_M1 = 10;
constexpr uint8_t CS_M2 = 9;

// ESC signal pins
constexpr uint8_t ESC1_PIN = 2;   // Motor 1 = reference (master)
constexpr uint8_t ESC2_PIN = 3;   // Motor 2 = follower

// ESC pulse limits
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;

// Base throttle applied to Motor 1 (and feedforward for Motor 2)
volatile int baseThrottleUs = 1200;

// Control period
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz

// SPI settings for AS5147P
constexpr uint32_t SPI_HZ = 1000000;   // 1 MHz to start
constexpr uint8_t  SPI_MODE = SPI_MODE1;

// -------------------- SPEED PI (inner loop) --------------------
// Units: microseconds of ESC correction per RPM error
volatile float Kp_us_per_rpm   = 0.05f;
volatile float Ki_us_per_rpm_s = 0.20f;

constexpr float ITERM_CLAMP_US = 300.0f;  // integral clamp (us)
constexpr float OUT_CLAMP_US   = 250.0f;  // total correction clamp (us)

// RPM low-pass filter
constexpr float RPM_ALPHA = 0.80f;

// -------------------- PHASE (outer loop) --------------------
// Target: motor2 angle = motor1 angle + 90 deg (i.e., motor2 leads by +90°)
volatile float TARGET_PHASE_DEG = 90.0f;

// Phase loop creates a small RPM offset command for motor2.
// Units: RPM offset per degree of phase error
volatile float Kph_rpm_per_deg   = 0.50f;   // start small
volatile float Kih_rpm_per_deg_s = 0.00f;   // often P-only is enough; enable if needed

constexpr float PHASE_I_CLAMP_RPM = 200.0f;  // clamp phase integrator (rpm)
constexpr float PHASE_OFFS_CLAMP_RPM = 300.0f; // clamp max +/- rpm offset commanded by phase loop

// Only enter phase lock when speed is stable
constexpr float MIN_RPM_FOR_PHASE = 500.0f;     // don’t phase-lock near standstill
constexpr float RPM_STABLE_BAND   = 30.0f;      // abs(rpm1-rpm2) threshold
constexpr uint32_t STABLE_HOLD_MS = 1200;       // must remain stable this long

// -------------------------------------------------------------

Servo esc1, esc2;

struct RpmEstimator {
  bool init = false;
  uint16_t prevAngle = 0;
  uint32_t prevTimeUs = 0;
  float rpmFilt = 0.0f;
};

RpmEstimator rpm1State, rpm2State;

float speedITerm_us = 0.0f;   // speed PI integrator (us)
float phaseITerm_rpm = 0.0f;  // phase PI integrator (rpm)

enum SyncMode { MODE_RPM_MATCH, MODE_PHASE_LOCK };
SyncMode mode = MODE_RPM_MATCH;
uint32_t stableStartMs = 0;

// -------------------- AS5147P helpers --------------------
static inline uint8_t evenParityBit15(uint16_t lower15) {
  lower15 &= 0x7FFF;
  uint16_t x = lower15;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

static inline bool checkEvenParity(uint16_t frame) {
  uint16_t lower15 = frame & 0x7FFF;
  uint8_t p = evenParityBit15(lower15);
  uint8_t pFrame = (frame >> 15) & 1;
  return (p == pFrame);
}

// Pipelined read of 14-bit register
uint16_t as5147pRead14(uint8_t csPin, uint16_t addr14) {
  uint16_t cmd = (addr14 & 0x3FFF) | 0x4000;     // read bit
  cmd |= (uint16_t(evenParityBit15(cmd)) << 15); // parity

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE));

  digitalWrite(csPin, LOW);
  SPI.transfer16(cmd);
  digitalWrite(csPin, HIGH);

  delayMicroseconds(1);

  digitalWrite(csPin, LOW);
  uint16_t resp = SPI.transfer16(0x0000);
  digitalWrite(csPin, HIGH);

  SPI.endTransaction();

  bool parityOK = checkEvenParity(resp);
  bool errorFlag = (resp & 0x4000) != 0;

  if (!parityOK || errorFlag) return 0xFFFF;
  return resp & 0x3FFF;
}

uint16_t readAngleMotor1() { return as5147pRead14(CS_M1, 0x3FFF); }
uint16_t readAngleMotor2() { return as5147pRead14(CS_M2, 0x3FFF); }

static inline float angle14_to_deg(uint16_t a14) {
  return (float)a14 * (360.0f / 16384.0f);
}

// Wrap degrees to (-180, 180]
static inline float wrapDeg180(float x) {
  while (x <= -180.0f) x += 360.0f;
  while (x >   180.0f) x -= 360.0f;
  return x;
}

// RPM estimator
float updateRPM(RpmEstimator &st, uint16_t angle14, uint32_t nowUs) {
  if (!st.init) {
    st.init = true;
    st.prevAngle = angle14;
    st.prevTimeUs = nowUs;
    st.rpmFilt = 0.0f;
    return 0.0f;
  }

  uint32_t dtUs = nowUs - st.prevTimeUs;
  if (dtUs == 0) return st.rpmFilt;

  float dt = dtUs * 1e-6f;

  int32_t dAngle = int32_t(angle14) - int32_t(st.prevAngle);
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  float rev = float(dAngle) / 16384.0f;
  float rpmRaw = (rev / dt) * 60.0f;

  st.rpmFilt = RPM_ALPHA * st.rpmFilt + (1.0f - RPM_ALPHA) * rpmRaw;

  st.prevAngle = angle14;
  st.prevTimeUs = nowUs;

  return st.rpmFilt;
}

// -------------------- ESC / safety --------------------
void armESCs() {
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.writeMicroseconds(ESC_MIN_US);
  delay(3000);
}

// -------------------- Serial tuning --------------------
void handleSerialTuning() {
  // Commands (examples):
  //  t1200   -> base throttle us
  //  p0.05   -> speed Kp
  //  i0.20   -> speed Ki
  //  a90     -> target phase deg (motor2 leads motor1)
  //  h0.50   -> phase Kp (rpm/deg)
  //  j0.00   -> phase Ki (rpm/(deg*s))
  //  r       -> reset back to RPM match mode
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 't') {
    int v = Serial.parseInt();
    baseThrottleUs = constrain(v, ESC_MIN_US, ESC_MAX_US);
  } else if (c == 'p') {
    Kp_us_per_rpm = Serial.parseFloat();
  } else if (c == 'i') {
    Ki_us_per_rpm_s = Serial.parseFloat();
  } else if (c == 'a') {
    TARGET_PHASE_DEG = Serial.parseFloat();
  } else if (c == 'h') {
    Kph_rpm_per_deg = Serial.parseFloat();
  } else if (c == 'j') {
    Kih_rpm_per_deg_s = Serial.parseFloat();
  } else if (c == 'r') {
    mode = MODE_RPM_MATCH;
    stableStartMs = 0;
    speedITerm_us = 0;
    phaseITerm_rpm = 0;
  } else {
    while (Serial.available() && Serial.read() != '\n') {}
  }
}

// -------------------- Setup / loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(CS_M1, OUTPUT); digitalWrite(CS_M1, HIGH);
  pinMode(CS_M2, OUTPUT); digitalWrite(CS_M2, HIGH);

  SPI.begin();

  esc1.attach(ESC1_PIN, ESC_MIN_US, ESC_MAX_US);
  esc2.attach(ESC2_PIN, ESC_MIN_US, ESC_MAX_US);

  armESCs();

  Serial.println("Ready.");
  Serial.println("Cmds: t<us> p<float> i<float> a<deg> h<float> j<float> r");
  Serial.println("Phase target means: angle2 = angle1 + target (motor2 leads).");
}

void loop() {
  handleSerialTuning();

  static uint32_t lastControlUs = micros();
  static uint32_t lastPrintUs = micros();

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    lastControlUs += CONTROL_PERIOD_US;
    float dt = CONTROL_PERIOD_US * 1e-6f;

    uint16_t a1 = readAngleMotor1();
    uint16_t a2 = readAngleMotor2();
    if (a1 == 0xFFFF || a2 == 0xFFFF) return;

    float rpm1 = updateRPM(rpm1State, a1, nowUs);
    float rpm2 = updateRPM(rpm2State, a2, nowUs);

    // Always drive motor1 at base throttle (reference)
    int out1 = constrain(baseThrottleUs, ESC_MIN_US, ESC_MAX_US);

    // -------- State machine: RPM match -> Phase lock --------
    if (mode == MODE_RPM_MATCH) {
      float err_rpm = rpm1 - rpm2;

      // Stability gate for switching into phase lock
      bool stable =
        (fabsf(err_rpm) < RPM_STABLE_BAND) &&
        (fabsf(rpm1) > MIN_RPM_FOR_PHASE) &&
        (fabsf(rpm2) > MIN_RPM_FOR_PHASE);

      uint32_t nowMs = millis();
      if (stable) {
        if (stableStartMs == 0) stableStartMs = nowMs;
        if ((nowMs - stableStartMs) >= STABLE_HOLD_MS) {
          mode = MODE_PHASE_LOCK;
          // reset integrators so we don't jump
          speedITerm_us = 0.0f;
          phaseITerm_rpm = 0.0f;
        }
      } else {
        stableStartMs = 0;
      }

      // Speed PI to match rpm2 -> rpm1
      float speedErr = rpm1 - rpm2;
      speedITerm_us += (Ki_us_per_rpm_s * speedErr) * dt;
      speedITerm_us = constrain(speedITerm_us, -ITERM_CLAMP_US, ITERM_CLAMP_US);

      float pTerm_us = Kp_us_per_rpm * speedErr;
      float corr_us = constrain(pTerm_us + speedITerm_us, -OUT_CLAMP_US, OUT_CLAMP_US);

      int out2 = constrain(int(baseThrottleUs + corr_us), ESC_MIN_US, ESC_MAX_US);

      esc1.writeMicroseconds(out1);
      esc2.writeMicroseconds(out2);

      // Print
      if ((uint32_t)(nowUs - lastPrintUs) >= 50000) {
        lastPrintUs = nowUs;
        Serial.print("[RPM] rpm1=");
        Serial.print(rpm1, 1);
        Serial.print(" rpm2=");
        Serial.print(rpm2, 1);
        Serial.print(" err=");
        Serial.print(speedErr, 1);
        Serial.print(" out2=");
        Serial.println(out2);
      }

    } else { // MODE_PHASE_LOCK
      // If speed collapses, fall back
      if (fabsf(rpm1) < MIN_RPM_FOR_PHASE || fabsf(rpm2) < MIN_RPM_FOR_PHASE) {
        mode = MODE_RPM_MATCH;
        stableStartMs = 0;
        speedITerm_us = 0;
        phaseITerm_rpm = 0;
        return;
      }

      float deg1 = angle14_to_deg(a1);
      float deg2 = angle14_to_deg(a2);

      // phaseErr = (actual phase diff) - target, wrapped to (-180,180]
      // actual phase diff defined as (deg2 - deg1)
      float phaseErrDeg = wrapDeg180((deg2 - deg1) - TARGET_PHASE_DEG);

      // Phase PI produces a small RPM offset command.
      // IMPORTANT SIGN: if phaseErrDeg > 0 (motor2 too far ahead), we want motor2 slower => negative rpm offset.
      phaseITerm_rpm += (-Kih_rpm_per_deg_s * phaseErrDeg) * dt;
      phaseITerm_rpm = constrain(phaseITerm_rpm, -PHASE_I_CLAMP_RPM, PHASE_I_CLAMP_RPM);

      float phaseOffs_rpm = (-Kph_rpm_per_deg * phaseErrDeg) + phaseITerm_rpm;
      phaseOffs_rpm = constrain(phaseOffs_rpm, -PHASE_OFFS_CLAMP_RPM, PHASE_OFFS_CLAMP_RPM);

      // New speed target for motor2:
      float rpm2_ref = rpm1 + phaseOffs_rpm;

      // Speed PI tracks rpm2_ref
      float speedErr = rpm2_ref - rpm2;

      speedITerm_us += (Ki_us_per_rpm_s * speedErr) * dt;
      speedITerm_us = constrain(speedITerm_us, -ITERM_CLAMP_US, ITERM_CLAMP_US);

      float pTerm_us = Kp_us_per_rpm * speedErr;
      float corr_us = constrain(pTerm_us + speedITerm_us, -OUT_CLAMP_US, OUT_CLAMP_US);

      int out2 = constrain(int(baseThrottleUs + corr_us), ESC_MIN_US, ESC_MAX_US);

      esc1.writeMicroseconds(out1);
      esc2.writeMicroseconds(out2);

      // Print
      if ((uint32_t)(nowUs - lastPrintUs) >= 50000) {
        lastPrintUs = nowUs;
        Serial.print("[PHASE] rpm1=");
        Serial.print(rpm1, 1);
        Serial.print(" rpm2=");
        Serial.print(rpm2, 1);
        Serial.print(" phErrDeg=");
        Serial.print(phaseErrDeg, 1);
        Serial.print(" phOffRpm=");
        Serial.print(phaseOffs_rpm, 1);
        Serial.print(" out2=");
        Serial.println(out2);
      }
    }
  }
}
