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

// Base throttle applied to Motor 1 (open-loop)
volatile int baseThrottleUs = 1200;

// Loop period
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz

// SPI settings for AS5147P
constexpr uint32_t SPI_HZ   = 1000000;  // start 1 MHz
constexpr uint8_t  SPI_MODE = SPI_MODE1;

// -------------------- SPEED (Stage 1) --------------------
// Speed PI for Motor 2 to match Motor 1 RPM
volatile float Kp_speed_us_per_rpm   = 0.05f;
volatile float Ki_speed_us_per_rpm_s = 0.20f;

constexpr float SPEED_I_CLAMP_US = 300.0f;
constexpr float SPEED_OUT_CLAMP_US = 250.0f;

// RPM low-pass filter
constexpr float RPM_ALPHA = 0.80f;

// Stability gate (when to switch to phase stage)
constexpr float MIN_RPM_FOR_PHASE = 500.0f;
constexpr float RPM_STABLE_BAND   = 30.0f;     // abs(rpm1-rpm2) < band
constexpr uint32_t STABLE_HOLD_MS = 1200;      // must hold stable this long

// Optional fallback to speed mode if things drift badly in phase mode
constexpr float RPM_DRIFT_BAND = 250.0f;       // if abs(rpm1-rpm2) exceeds this...
constexpr uint32_t DRIFT_HOLD_MS = 300;        // ...for this long, fall back

// -------------------- PHASE (Stage 2) --------------------
// Target separation: motor2 leads motor1 by +90 degrees
volatile float TARGET_PHASE_DEG = 90.0f;

// Threshold like paper: |e_phi| > pi/5 => robust, else aggressive
constexpr float PHASE_THRESH_DEG = 36.0f;

// Robust phase PI gains (gentler)
volatile float Kp_phi_r_us_per_deg   = 1.0f;
volatile float Ki_phi_r_us_per_deg_s = 0.10f;

// Aggressive phase PI gains (stronger, near target)
volatile float Kp_phi_a_us_per_deg   = 3.0f;
volatile float Ki_phi_a_us_per_deg_s = 0.30f;

// Integral & output clamps for phase controller
constexpr float PHASE_I_CLAMP_US   = 400.0f;
constexpr float PHASE_OUT_CLAMP_US = 350.0f;

// -------------------------------------------------------------

Servo esc1, esc2;

struct RpmEstimator {
  bool init = false;
  uint16_t prevAngle = 0;
  uint32_t prevTimeUs = 0;
  float rpmFilt = 0.0f;
};

RpmEstimator rpm1State, rpm2State;

enum Mode { MODE_SPEED_MATCH, MODE_PHASE_ONLY };
Mode mode = MODE_SPEED_MATCH;

float speedITerm_us = 0.0f;   // stage-1 integrator
float phaseITerm_us = 0.0f;   // stage-2 integrator

int u2_latched_us = 1200;     // latched motor2 command at phase enable time

uint32_t stableStartMs = 0;
uint32_t driftStartMs  = 0;

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

// Pipelined read of 14-bit register 0x3FFF (ANGLE)
// response: [15]=parity, [14]=error flag, [13:0]=data
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

uint16_t readAngleM1() { return as5147pRead14(CS_M1, 0x3FFF); }
uint16_t readAngleM2() { return as5147pRead14(CS_M2, 0x3FFF); }

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
void handleSerial() {
  // Commands:
  //  t1200  -> base throttle us (motor1)
  //  ps0.05 -> speed Kp
  //  is0.20 -> speed Ki
  //  a90    -> target phase deg
  //  pr1.0  -> phase robust Kp
  //  ir0.1  -> phase robust Ki
  //  pa3.0  -> phase aggressive Kp
  //  ia0.3  -> phase aggressive Ki
  //  r      -> reset to speed match
  if (!Serial.available()) return;

  char c1 = Serial.read();
  if (c1 == 't') {
    int v = Serial.parseInt();
    baseThrottleUs = constrain(v, ESC_MIN_US, ESC_MAX_US);
  } else if (c1 == 'p') {
    char c2 = Serial.read();
    if (c2 == 's') Kp_speed_us_per_rpm = Serial.parseFloat();
    else if (c2 == 'r') Kp_phi_r_us_per_deg = Serial.parseFloat();
    else if (c2 == 'a') Kp_phi_a_us_per_deg = Serial.parseFloat();
  } else if (c1 == 'i') {
    char c2 = Serial.read();
    if (c2 == 's') Ki_speed_us_per_rpm_s = Serial.parseFloat();
    else if (c2 == 'r') Ki_phi_r_us_per_deg_s = Serial.parseFloat();
    else if (c2 == 'a') Ki_phi_a_us_per_deg_s = Serial.parseFloat();
  } else if (c1 == 'a') {
    TARGET_PHASE_DEG = Serial.parseFloat();
  } else if (c1 == 'r') {
    mode = MODE_SPEED_MATCH;
    stableStartMs = 0;
    driftStartMs = 0;
    speedITerm_us = 0;
    phaseITerm_us = 0;
  } else {
    while (Serial.available() && Serial.read() != '\n') {}
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(CS_M1, OUTPUT); digitalWrite(CS_M1, HIGH);
  pinMode(CS_M2, OUTPUT); digitalWrite(CS_M2, HIGH);

  SPI.begin();

  esc1.attach(ESC1_PIN, ESC_MIN_US, ESC_MAX_US);
  esc2.attach(ESC2_PIN, ESC_MIN_US, ESC_MAX_US);

  armESCs();

  Serial.println("Ready.");
  Serial.println("Stage1: speed match (PI). Stage2: phase only (latched speed + phase PI).");
}

void loop() {
  handleSerial();

  static uint32_t lastControlUs = micros();
  static uint32_t lastPrintUs = micros();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
  lastControlUs += CONTROL_PERIOD_US;

  float dt = CONTROL_PERIOD_US * 1e-6f;

  // Read angles
  uint16_t a1 = readAngleM1();
  uint16_t a2 = readAngleM2();
  if (a1 == 0xFFFF || a2 == 0xFFFF) return;

  // Update RPM
  float rpm1 = updateRPM(rpm1State, a1, nowUs);
  float rpm2 = updateRPM(rpm2State, a2, nowUs);

  // Motor 1 command (open-loop)
  int out1 = constrain(baseThrottleUs, ESC_MIN_US, ESC_MAX_US);
  esc1.writeMicroseconds(out1);

  // -------- STAGE 1: SPEED MATCH ONLY --------
  if (mode == MODE_SPEED_MATCH) {
    float eSpeed = rpm1 - rpm2;

    // Speed PI -> correction in microseconds
    speedITerm_us += (Ki_speed_us_per_rpm_s * eSpeed) * dt;
    speedITerm_us = constrain(speedITerm_us, -SPEED_I_CLAMP_US, SPEED_I_CLAMP_US);

    float uSpeed_us = Kp_speed_us_per_rpm * eSpeed + speedITerm_us;
    uSpeed_us = constrain(uSpeed_us, -SPEED_OUT_CLAMP_US, SPEED_OUT_CLAMP_US);

    int out2 = constrain(int(baseThrottleUs + uSpeed_us), ESC_MIN_US, ESC_MAX_US);
    esc2.writeMicroseconds(out2);

    // Stability gate to switch to phase stage (paper: reach steady-state before phase) :contentReference[oaicite:2]{index=2}
    bool stable =
      (fabsf(eSpeed) < RPM_STABLE_BAND) &&
      (fabsf(rpm1) > MIN_RPM_FOR_PHASE) &&
      (fabsf(rpm2) > MIN_RPM_FOR_PHASE);

    uint32_t nowMs = millis();
    if (stable) {
      if (stableStartMs == 0) stableStartMs = nowMs;
      if ((nowMs - stableStartMs) >= STABLE_HOLD_MS) {
        // Latch the speed command at activation time (paper: lock u_omega(Tphi)) :contentReference[oaicite:3]{index=3}
        u2_latched_us = out2;

        // Reset phase integrator
        phaseITerm_us = 0.0f;

        // Stop updating speed integrator once we leave this mode
        mode = MODE_PHASE_ONLY;

        driftStartMs = 0;
      }
    } else {
      stableStartMs = 0;
    }

    // Debug
    if ((uint32_t)(nowUs - lastPrintUs) >= 50000) {
      lastPrintUs = nowUs;
      Serial.print("[SPEED] rpm1="); Serial.print(rpm1, 1);
      Serial.print(" rpm2=");        Serial.print(rpm2, 1);
      Serial.print(" e=");           Serial.print(eSpeed, 1);
      Serial.print(" out2=");        Serial.print(out2);
      Serial.print(" stableMs=");    Serial.println(stableStartMs ? (millis()-stableStartMs) : 0);
    }

    return;
  }

  // -------- STAGE 2: PHASE ONLY (speed output latched) --------
  // This follows the paper’s idea: u2 = u2_omega(Tphi) + u_phi(t) (no simultaneous speed loop) :contentReference[oaicite:4]{index=4}

  float deg1 = angle14_to_deg(a1);
  float deg2 = angle14_to_deg(a2);

  // Actual separation defined as (deg2 - deg1)
  float sepDeg = wrapDeg180(deg2 - deg1);

  // Phase error: target - actual, wrapped
  float ePhiDeg = wrapDeg180(TARGET_PHASE_DEG - sepDeg);

  // Optional drift fallback: if RPM separation gets large for a while, go back to speed match
  float eSpeedNow = rpm1 - rpm2;
  uint32_t nowMs = millis();
  if (fabsf(eSpeedNow) > RPM_DRIFT_BAND) {
    if (driftStartMs == 0) driftStartMs = nowMs;
    if ((nowMs - driftStartMs) > DRIFT_HOLD_MS) {
      mode = MODE_SPEED_MATCH;
      stableStartMs = 0;
      driftStartMs = 0;
      // re-enable speed integrator gently
      // (you can also keep the previous value if you prefer)
      speedITerm_us = 0.0f;
      return;
    }
  } else {
    driftStartMs = 0;
  }

  // Choose robust vs aggressive based on |e_phi| threshold like in the paper :contentReference[oaicite:5]{index=5}
  float Kp_phi = (fabsf(ePhiDeg) > PHASE_THRESH_DEG) ? Kp_phi_r_us_per_deg : Kp_phi_a_us_per_deg;
  float Ki_phi = (fabsf(ePhiDeg) > PHASE_THRESH_DEG) ? Ki_phi_r_us_per_deg_s : Ki_phi_a_us_per_deg_s;

  // Phase PI controller -> microseconds correction around the latched command
  phaseITerm_us += (Ki_phi * ePhiDeg) * dt;
  phaseITerm_us = constrain(phaseITerm_us, -PHASE_I_CLAMP_US, PHASE_I_CLAMP_US);

  float uPhi_us = Kp_phi * ePhiDeg + phaseITerm_us;
  uPhi_us = constrain(uPhi_us, -PHASE_OUT_CLAMP_US, PHASE_OUT_CLAMP_US);

  int out2 = constrain(int(u2_latched_us + uPhi_us), ESC_MIN_US, ESC_MAX_US);
  esc2.writeMicroseconds(out2);

  // Debug
  if ((uint32_t)(nowUs - lastPrintUs) >= 50000) {
    lastPrintUs = nowUs;
    Serial.print("[PHASE] rpm1=");   Serial.print(rpm1, 1);
    Serial.print(" rpm2=");          Serial.print(rpm2, 1);
    Serial.print(" sepDeg=");        Serial.print(sepDeg, 1);
    Serial.print(" ePhiDeg=");       Serial.print(ePhiDeg, 1);
    Serial.print(" latched=");       Serial.print(u2_latched_us);
    Serial.print(" out2=");          Serial.println(out2);
  }
}
