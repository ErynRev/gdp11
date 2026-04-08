// Working version adapted for 2 masters + 2 slaves
// Same logic as before, just duplicated cleanly via arrays/state structs

#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h>

constexpr uint8_t NUM_PAIRS = 2;

// ---------------------- ZERO OFFSETS ----------------------
float zeroOffsetDeg_L[NUM_PAIRS] = {0.0f, 0.0f};
float zeroOffsetDeg_F[NUM_PAIRS] = {0.0f, 0.0f};

const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_L[NUM_PAIRS] = {4, 12};
const int EEPROM_ADDR_F[NUM_PAIRS] = {8, 16};
const uint32_t EEPROM_MAGIC = 0x1234ABCD;

// ---------------------- MISC ----------------------
bool synchro = false;
int thres = 1000;

// gains for RPM matching
double K_I;
double K_P;
double K_D;

volatile float throttle_us = 1070.5f;

// phase gains
double K_I_phi[NUM_PAIRS] = {0.0f, 0.0f};
double K_P_phi[NUM_PAIRS] = {0.0f, 0.0f};
double K_D_phi[NUM_PAIRS] = {0.0f, 0.0f};

constexpr uint32_t RPM_PERIOD_US      = 4000;   // 500 Hz
constexpr uint32_t RPM_STABLE_TIME_MS = 3000;
constexpr uint32_t CONTROL_PERIOD_US  = 20000;  // 50 Hz
constexpr float    DT                 = CONTROL_PERIOD_US * 1e-6f;

volatile float TARGET_PHASE_DEG = 90.0f;

// ---------------------- PIN MAP ----------------------
// Pair 1 = original pins
// Pair 2 = CHANGE THESE to match your actual wiring
constexpr uint8_t CS_L[NUM_PAIRS] = {15, 16};   // leaders / masters encoders
constexpr uint8_t CS_F[NUM_PAIRS] = {38, 10};   // followers / slaves encoders

constexpr uint8_t ESC_MASTER_PIN[NUM_PAIRS] = {19, 24};
constexpr uint8_t ESC_PIN[NUM_PAIRS]        = {25, 18};

// ESC pulse limits
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;

// SPI settings for AS5147P
constexpr uint32_t SPI_HZ = 1000000;
constexpr uint8_t  SPI_MODE_CFG = SPI_MODE1;

enum Mode { MODE_SPEED_MATCH, MODE_PHASE_ONLY };

// ---------------------- PER-PAIR STATE ----------------------
Servo escMaster[NUM_PAIRS];
Servo esc[NUM_PAIRS];

Mode mode[NUM_PAIRS] = {MODE_SPEED_MATCH, MODE_SPEED_MATCH};

float prev_ePhiDeg[NUM_PAIRS] = {0.0f, 0.0f};
float PhiITerm_us[NUM_PAIRS]  = {0.0f, 0.0f};

// This was global before; for 2 pairs it must be per-pair
float RPM_STABLE_BAND[NUM_PAIRS] = {50.0f, 50.0f};

uint32_t stableStartMs[NUM_PAIRS] = {0, 0};
bool speedStable[NUM_PAIRS] = {false, false};

uint16_t raw_L_latest[NUM_PAIRS] = {0, 0};
uint16_t raw_F_latest[NUM_PAIRS] = {0, 0};
float rpm_L_f_latest[NUM_PAIRS] = {0.0f, 0.0f};
float rpm_F_f_latest[NUM_PAIRS] = {0.0f, 0.0f};

struct RpmState {
  uint16_t prevAngle;
  unsigned long prevTime;
  float ema;
  bool emaInit;
};

RpmState rpmLState[NUM_PAIRS] = {};
RpmState rpmFState[NUM_PAIRS] = {};

const float ALPHA_L = 0.04f;
const float ALPHA_F = 0.04f;

// ---------------------- EEPROM ----------------------
void saveZeroOffsets() {
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);

  for (uint8_t i = 0; i < NUM_PAIRS; i++) {
    EEPROM.put(EEPROM_ADDR_L[i], zeroOffsetDeg_L[i]);
    EEPROM.put(EEPROM_ADDR_F[i], zeroOffsetDeg_F[i]);
  }
}

void loadZeroOffsets() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);

  if (magic == EEPROM_MAGIC) {
    for (uint8_t i = 0; i < NUM_PAIRS; i++) {
      EEPROM.get(EEPROM_ADDR_L[i], zeroOffsetDeg_L[i]);
      EEPROM.get(EEPROM_ADDR_F[i], zeroOffsetDeg_F[i]);
    }
    Serial.println("Loaded saved zero offsets.");
  } else {
    for (uint8_t i = 0; i < NUM_PAIRS; i++) {
      zeroOffsetDeg_L[i] = 0.0f;
      zeroOffsetDeg_F[i] = 0.0f;
    }
    Serial.println("No saved zero offsets found. Using 0.");
  }
}

// ---------------------- ANGLE UTILS ----------------------
static inline float raw_to_deg(uint16_t a) {
  return (float)a * (360.0f / 16384.0f);
}

static inline float wrapDeg180(float x) {
  while (x <= -180.0f) x += 360.0f;
  while (x >   180.0f) x -= 360.0f;
  return x;
}

float wrap360(float a) {
  if (a < 0.0f) a += 360.0f;
  return a;
}

uint16_t readAngle(int motor_pin) {
  uint16_t angle;

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE_CFG));

  digitalWrite(motor_pin, LOW);
  delayMicroseconds(2);
  SPI.transfer16(0xFFFF);
  digitalWrite(motor_pin, HIGH);

  delayMicroseconds(2);

  digitalWrite(motor_pin, LOW);
  delayMicroseconds(2);
  angle = SPI.transfer16(0x0000);
  digitalWrite(motor_pin, HIGH);

  SPI.endTransaction();

  return angle & 0x3FFF;
}

float getZeroedAngleDeg(uint8_t csPin, float zeroOffsetDeg) {
  uint16_t raw = readAngle(csPin);
  float deg = raw_to_deg(raw);
  return wrap360(deg - zeroOffsetDeg);
}

// ---------------------- RPM ----------------------
float computeRPM(uint16_t currentAngle, RpmState &s) {
  unsigned long currentTime = micros();

  if (s.prevTime == 0) {
    s.prevAngle = currentAngle;
    s.prevTime = currentTime;
    return 0.0f;
  }

  float dt = (currentTime - s.prevTime) * 1e-6f;
  if (dt <= 0.0f) return 0.0f;

  int32_t dAngle = (int32_t)currentAngle - (int32_t)s.prevAngle;

  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  float revolutions = (float)dAngle / 16384.0f;
  float rpm = (revolutions / dt) * 60.0f;

  s.prevAngle = currentAngle;
  s.prevTime  = currentTime;

  return fabsf(rpm);
}

inline float emaUpdate(RpmState &s, float x, float alpha) {
  if (!s.emaInit) {
    s.ema = x;
    s.emaInit = true;
  } else {
    s.ema += alpha * (x - s.ema);
  }
  return s.ema;
}

// ---------------------- SERIAL ----------------------
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

  if (line.startsWith("kp_phi0")) {
    String valStr = line.substring(7);
    valStr.trim();

    float v = valStr.toFloat();
    K_P_phi[0] = v;

    Serial.print("New K_P_phi = ");
    Serial.println(K_P_phi[0], 6);
    return;
  }

  if (line.startsWith("ki_phi0")) {
    String valStr = line.substring(7);
    valStr.trim();

    float v = valStr.toFloat();
    K_I_phi[0] = v;

    Serial.print("New K_I_phi = ");
    Serial.println(K_I_phi[0], 6);
    return;
  }

  if (line.startsWith("kd_phi0")) {
    String valStr = line.substring(7);
    valStr.trim();

    float v = valStr.toFloat();
    K_D_phi[0] = v;

    Serial.print("New K_D_phi = ");
    Serial.println(K_D_phi[0], 6);
    return;
  }
  if (line.startsWith("kp_phi1")) {
    String valStr = line.substring(7);
    valStr.trim();

    float v = valStr.toFloat();
    K_P_phi[1] = v;

    Serial.print("New K_P_phi = ");
    Serial.println(K_P_phi[0], 6);
    return;
  }

  if (line.startsWith("ki_phi1")) {
    String valStr = line.substring(7);
    valStr.trim();

    float v = valStr.toFloat();
    K_I_phi[1] = v;

    Serial.print("New K_I_phi = ");
    Serial.println(K_I_phi[0], 6);
    return;
  }

  if (line.startsWith("kd_phi1")) {
    String valStr = line.substring(7);
    valStr.trim();

    float v = valStr.toFloat();
    K_D_phi[1] = v;

    Serial.print("New K_D_phi = ");
    Serial.println(K_D_phi[0], 6);
    return;
  }


  if (line.equalsIgnoreCase("zero")) {
    for (uint8_t i = 0; i < NUM_PAIRS; i++) {
      zeroOffsetDeg_L[i] = raw_to_deg(readAngle(CS_L[i]));
      zeroOffsetDeg_F[i] = raw_to_deg(readAngle(CS_F[i]));
    }
    saveZeroOffsets();

    Serial.println("Zero positions captured and saved.");
    for (uint8_t i = 0; i < NUM_PAIRS; i++) {
      Serial.print("Pair ");
      Serial.print(i + 1);
      Serial.print(" Leader zero = ");
      Serial.println(zeroOffsetDeg_L[i], 3);

      Serial.print("Pair ");
      Serial.print(i + 1);
      Serial.print(" Follower zero = ");
      Serial.println(zeroOffsetDeg_F[i], 3);
    }
    return;
  }

  if (line.equalsIgnoreCase("showzero")) {
    for (uint8_t i = 0; i < NUM_PAIRS; i++) {
      Serial.print("Pair ");
      Serial.print(i + 1);
      Serial.print(" Leader zero = ");
      Serial.println(zeroOffsetDeg_L[i], 3);

      Serial.print("Pair ");
      Serial.print(i + 1);
      Serial.print(" Follower zero = ");
      Serial.println(zeroOffsetDeg_F[i], 3);
    }
    return;
  }

  if (line[0] == 't' || line[0] == 'T') {
    line.remove(0, 1);
    line.trim();
  }

  float v = line.toFloat();
  if (v < 500.0f) {
    Serial.println("Parse failed or too small. Send like: 1100.5  or  t 1100.5");
    return;
  }

  if (v < ESC_MIN_US) v = ESC_MIN_US;
  if (v > ESC_MAX_US) v = ESC_MAX_US;

  throttle_us = v;

  Serial.print("New throttle_us = ");
  Serial.println(throttle_us, 2);
}

// ---------------------- SETUP ----------------------
void setup() {
  K_P = 0.3;
  K_I = 0.1;
  K_D = 0.00;
  
  K_P_phi[0] = 0.4;
  K_I_phi[0] = 0.1;   // preserved from your original code
  K_D_phi[0] = 0.001;

  
  K_P_phi[1] = 0.4;
  K_I_phi[1] = 0.1;   // preserved from your original code
  K_D_phi[1] = 0.001;

  for (uint8_t i = 0; i < NUM_PAIRS; i++) {
    PhiITerm_us[i] = 0.0f;
  }

  Serial.begin(115200);
  loadZeroOffsets();
  while (!Serial) ;
  Serial.println("Ready");

  for (uint8_t i = 0; i < NUM_PAIRS; i++) {
    pinMode(CS_L[i], OUTPUT);
    digitalWrite(CS_L[i], HIGH);

    pinMode(CS_F[i], OUTPUT);
    digitalWrite(CS_F[i], HIGH);
  }

  SPI.begin();

  for (uint8_t i = 0; i < NUM_PAIRS; i++) {
    escMaster[i].attach(ESC_MASTER_PIN[i], 1000, 2000);
    escMaster[i].writeMicroseconds(1000);

    esc[i].attach(ESC_PIN[i], 1000, 2000);
    esc[i].writeMicroseconds(1000);
  }

  delay(3000);
  throttle_us = 1000;
}

// ---------------------- LOOP ----------------------
void loop() {
  static uint32_t lastRpmUs  = micros();
  static uint32_t lastCtrlUs = lastRpmUs;

  uint32_t nowUs = micros();

  // ---------- FAST RPM UPDATE ----------
  if ((uint32_t)(nowUs - lastRpmUs) >= RPM_PERIOD_US) {
    lastRpmUs += RPM_PERIOD_US;

    for (uint8_t i = 0; i < NUM_PAIRS; i++) {
      uint16_t raw_L = readAngle(CS_L[i]);
      float rpm_L = computeRPM(raw_L, rpmLState[i]);
      float rpm_L_f = emaUpdate(rpmLState[i], rpm_L, ALPHA_L);

      uint16_t raw_F = readAngle(CS_F[i]);
      float rpm_F = computeRPM(raw_F, rpmFState[i]);
      float rpm_F_f = emaUpdate(rpmFState[i], rpm_F, ALPHA_F);

      raw_L_latest[i] = raw_L;
      raw_F_latest[i] = raw_F;
      rpm_L_f_latest[i] = rpm_L_f;
      rpm_F_f_latest[i] = rpm_F_f;
    }
  }

  // ---------- SLOW CONTROL LOOP ----------
  if ((uint32_t)(nowUs - lastCtrlUs) < CONTROL_PERIOD_US) return;
  lastCtrlUs += CONTROL_PERIOD_US;

  uint32_t nowMs = millis();

  handleSerial();

  // both masters get the same base throttle
  for (uint8_t i = 0; i < NUM_PAIRS; i++) {
    escMaster[i].writeMicroseconds(throttle_us);
  }

  // For each pair:
  // print: rpm_L, rpm_F, (stable_ms OR ePhi), ITerm, command
  for (uint8_t i = 0; i < NUM_PAIRS; i++) {
    float rpm_L_f = rpm_L_f_latest[i];
    float rpm_F_f = rpm_F_f_latest[i];

    float eSpeed = (rpm_L_f - rpm_F_f);

    // stability timer logic
    if (fabsf(eSpeed) <= RPM_STABLE_BAND[i]) {
      if (stableStartMs[i] == 0) stableStartMs[i] = nowMs;
      if ((nowMs - stableStartMs[i]) >= RPM_STABLE_TIME_MS) {
        speedStable[i] = true;
        mode[i] = MODE_PHASE_ONLY;
      }
    } else {
      stableStartMs[i] = 0;
      speedStable[i] = false;
      mode[i] = MODE_SPEED_MATCH;
      esc[i].writeMicroseconds(throttle_us);
    }

    Serial.print(rpm_L_f);
    Serial.print(",");
    Serial.print(rpm_F_f);
    Serial.print(",");

    // -------- SPEED MATCH MODE --------
    if (mode[i] == MODE_SPEED_MATCH) {
      RPM_STABLE_BAND[i] = 30.0f;
      PhiITerm_us[i] = 0.0f;

      uint32_t stableMs = stableStartMs[i] ? (nowMs - stableStartMs[i]) : 0;

      Serial.print(stableMs);
      Serial.print(",");
      Serial.print(0.0f);
      Serial.print(",");
      Serial.print(throttle_us, 2);
    }
    // -------- PHASE MODE --------
    else {
      RPM_STABLE_BAND[i] = 100.0f;

      float deg_L = getZeroedAngleDeg(CS_L[i], zeroOffsetDeg_L[i]);
      float deg_F = getZeroedAngleDeg(CS_F[i], zeroOffsetDeg_F[i]);

      float sepDeg  = wrapDeg180(deg_L - deg_F);
      float ePhiDeg = wrapDeg180(TARGET_PHASE_DEG - sepDeg);

      PhiITerm_us[i] += (K_I_phi[i] * ePhiDeg) * DT;
      PhiITerm_us[i] = constrain(PhiITerm_us[i], -5.0f, 5.0f);

      float dPhiDeg = (ePhiDeg - prev_ePhiDeg[i]) / DT;
      prev_ePhiDeg[i] = ePhiDeg;
      float PhidTerm_us = K_D_phi[i] * dPhiDeg;
      float throttle_adj = (K_P_phi[i] * ePhiDeg) + PhiITerm_us[i] + (PhidTerm_us);
      float throttle_phase;
      if (i == 0) {
        throttle_phase = throttle_us + throttle_adj;
      } else {
        throttle_phase = throttle_us - throttle_adj;
      }
      throttle_phase = constrain(throttle_phase, throttle_us - 10, throttle_us + 10);

      esc[i].writeMicroseconds((int)throttle_phase);

      Serial.print(ePhiDeg);
      Serial.print(",");
      Serial.print(PhiITerm_us[i]);
      Serial.print(",");
      Serial.print(PhidTerm_us);
      Serial.print(",");
      Serial.print(throttle_phase);
    }

    if (i < NUM_PAIRS - 1) Serial.print(",");
  }

  Serial.println();
}