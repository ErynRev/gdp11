#include <SPI.h>
#include <Servo.h>

bool synchro = false;
int thres = 1000;

//gains for RPM matching
double K_I;
double K_P;
double K_D;                 // derivative gain
float prev_eSpeed = 0.0f;  

volatile float throttle_us = 1070.5f;

double K_I_phi;
double K_P_phi;

constexpr uint32_t RPM_PERIOD_US = 7000; // 2 ms = 500 Hz (adjust as you like)

constexpr uint32_t RPM_STABLE_TIME_MS = 3000;   // 2 second

// Control loop rate
constexpr uint32_t CONTROL_PERIOD_US = 20000; // 50 Hz
constexpr float    DT = CONTROL_PERIOD_US * 1e-6f;

float speedITerm_us = 0.0f; 
//Threshold for initiating synchrophasing
volatile float RPM_STABLE_BAND   = 25.0f;

volatile float TARGET_PHASE_DEG = 90.0f;


// Encoder CS pins (shared SPI bus)
constexpr uint8_t CS_L = 10; //leader
constexpr uint8_t CS_F = 17;

// ESC signal pins
constexpr uint8_t ESC_MASTER_PIN = 25;
constexpr uint8_t ESC_PIN = 18;   // Motor 2 = follower


// ESC pulse limits
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;

// SPI settings for AS5147P
constexpr uint32_t SPI_HZ = 1000000;   // 1 MHz to start
constexpr uint8_t  SPI_MODE = SPI_MODE1;

enum Mode { MODE_SPEED_MATCH, MODE_PHASE_ONLY };
Mode mode = MODE_SPEED_MATCH;

static inline float raw_to_deg(uint16_t a) {
  return (float)a * (360.0f / 16384.0f);
}

static inline float wrapDeg180(float x) {
  while (x <= -180.0f) x += 360.0f;
  while (x >   180.0f) x -= 360.0f;
  return x;
}

Servo esc;
Servo escMaster;

uint16_t readAngle(int motor_pin) {
  uint16_t angle;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // start slow

  digitalWrite(motor_pin, LOW);
  delayMicroseconds(2);
  SPI.transfer16(0xFFFF);         // command frame for angle (0x3FFF read)
  digitalWrite(motor_pin, HIGH);

  delayMicroseconds(2);

  digitalWrite(motor_pin, LOW);
  delayMicroseconds(2);
  angle = SPI.transfer16(0x0000); // read response
  digitalWrite(motor_pin, HIGH);

  SPI.endTransaction();

  return angle & 0x3FFF; // 14-bit angle
}
float computeRPM_F(uint16_t currentAngle_F) {
  static uint16_t prevAngle_F = 0;
  static unsigned long prevTime_F = 0;

  unsigned long currentTime_F = micros();

  // First call guard
  if (prevTime_F == 0) {
    prevAngle_F= currentAngle_F;
    prevTime_F = currentTime_F;
    return 0.0;
  }

  // Time difference in seconds
  float dt = (currentTime_F - prevTime_F) * 1e-6;
  if (dt <= 0) return 0.0;

  // Angle difference (signed)
  int32_t dAngle = (int32_t)currentAngle_F - (int32_t)prevAngle_F;

  // Handle wrap-around
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  // Convert to revolutions
  float revolutions = (float)dAngle / 16384.0;

  // RPM
  float rpm = (revolutions / dt) * 60.0;

  // Update state
  prevAngle_F = currentAngle_F;
  prevTime_F = currentTime_F;

  return fabsf(rpm);
}
float computeRPM_L(uint16_t currentAngle_L) {
  static uint16_t prevAngle_L = 0;
  static unsigned long prevTime_L = 0;

  unsigned long currentTime_L = micros();

  // First call guard
  if (prevTime_L == 0) {
    prevAngle_L= currentAngle_L;
    prevTime_L = currentTime_L;
    return 0.0;
  }

  // Time difference in seconds
  float dt = (currentTime_L - prevTime_L) * 1e-6;
  if (dt <= 0) return 0.0;

  // Angle difference (signed)
  int32_t dAngle = (int32_t)currentAngle_L - (int32_t)prevAngle_L;

  // Handle wrap-around
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  // Convert to revolutions
  float revolutions = (float)dAngle / 16384.0;

  // RPM
  float rpm = (revolutions / dt) * 60.0;

  // Update state
  prevAngle_L = currentAngle_L;
  prevTime_L = currentTime_L;

  return fabsf(rpm);
}

// --- fast EMA for RPM ---
static float rpmEma_F = 0.0f;
static bool  rpmInit_F = false;
const float ALPHA_F = 0.04f;   // 0.05..0.30 (smaller = smoother)

inline float ema_F(float x) {
  if (!rpmInit_F) { rpmEma_F = x; rpmInit_F = true; }
  else          { rpmEma_F += ALPHA_F * (x - rpmEma_F); }
  return rpmEma_F;
}
static float rpmEma_L = 0.0f;
static bool  rpmInit_L = false;
const float ALPHA_L = 0.04f;   // 0.05..0.30 (smaller = smoother)

inline float ema_L(float x) {
  if (!rpmInit_L) { rpmEma_L = x; rpmInit_L = true; }
  else          { rpmEma_L += ALPHA_L * (x - rpmEma_L); }
  return rpmEma_L;
}

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
  // put your setup code here, to run once:
  K_P = 0.01;
  K_I = 0.001;
  K_D = 0.00;
  //K_I_phi = 0
  K_P_phi = 0.3;
  
  
  Serial.begin(115200);
  while (!Serial) ;  // wait for USB
  Serial.println("Ready");

  pinMode(CS_F, OUTPUT);
  digitalWrite(CS_F, HIGH);
  pinMode(CS_L, OUTPUT);
  digitalWrite(CS_L, HIGH);

  SPI.begin();

  escMaster.attach(ESC_MASTER_PIN, 1000, 2000);
  escMaster.writeMicroseconds(1000);

  esc.attach(ESC_PIN, 1000, 2000);                 // uses Servo library timing (~50 Hz)
  esc.writeMicroseconds(1000);
  delay(3000);
  throttle_us = 1000;
}

void loop() {
  // ---------- FAST RPM UPDATE (separate gate) ----------
  static uint32_t lastRpmUs = micros();
  static uint32_t lastCtrlUs = lastRpmUs;

  // stability timer state (CONTROL-rate logic uses these)
  static uint32_t stableStartMs = 0;
  static bool speedStable = false;

  // Latest measured values (updated fast, consumed slow)
  static uint16_t raw_L_latest = 0;
  static uint16_t raw_F_latest = 0;
  static float rpm_L_f_latest = 0.0f;
  static float rpm_F_f_latest = 0.0f;

  uint32_t nowUs = micros();

  // Choose your fast RPM measurement period (example: 500 Hz)

  if ((uint32_t)(nowUs - lastRpmUs) >= RPM_PERIOD_US) {
    lastRpmUs += RPM_PERIOD_US;

    // Read + update RPM as fast as you want
    uint16_t raw_L = (uint16_t)readAngle(CS_L);
    float rpm_L = computeRPM_L(raw_L);
    float rpm_L_f = ema_L(rpm_L);

    uint16_t raw_F = (uint16_t)readAngle(CS_F);
    float rpm_F = computeRPM_F(raw_F);
    float rpm_F_f = ema_F(rpm_F);

    // publish latest values for the control loop
    raw_L_latest = raw_L;
    raw_F_latest = raw_F;
    rpm_L_f_latest = rpm_L_f;
    rpm_F_f_latest = rpm_F_f;

    // (optional) fast-rate prints are noisy; keep if you want, but it can slow you down
    // Serial.print("rpm_L = "); Serial.println(rpm_L_f_latest);
    // Serial.print("rpm_F = "); Serial.println(rpm_F_f_latest);
  }

  // ---------- SLOW CONTROL LOOP (50 Hz) ----------
  if ((uint32_t)(nowUs - lastCtrlUs) < CONTROL_PERIOD_US) return;

  lastCtrlUs += CONTROL_PERIOD_US;

  uint32_t nowMs = millis();

  // Use latest filtered RPM values computed in the fast gate
  float rpm_L_f = rpm_L_f_latest;
  float rpm_F_f = rpm_F_f_latest;
  uint16_t raw_L = raw_L_latest;
  uint16_t raw_F = raw_F_latest;

  handleSerial();
  
  escMaster.writeMicroseconds(throttle_us);
  // (optional) print at control rate instead (much cleaner)
  Serial.print("rpm_L = "); Serial.println(rpm_L_f);
  Serial.print("rpm_F = "); Serial.println(rpm_F_f);

  float eSpeed = (rpm_L_f - rpm_F_f);

  // -------- update stability timer based on eSpeed band (50 Hz) --------
  if (fabsf(eSpeed) <= RPM_STABLE_BAND) {
    if (stableStartMs == 0) stableStartMs = nowMs;
    if ((nowMs - stableStartMs) >= RPM_STABLE_TIME_MS) {
      speedStable = true;
      mode = MODE_PHASE_ONLY;
    }
  } else {
    stableStartMs = 0;
    speedStable = false;
    mode = MODE_SPEED_MATCH;
    esc.writeMicroseconds(throttle_us);
  }

  // -------- SPEED MATCH CONTROLLER (50 Hz) --------
  if (mode == MODE_SPEED_MATCH) {
    RPM_STABLE_BAND  = 10.0f;
    Serial.print("stable_ms = ");
    Serial.println(stableStartMs ? (nowMs - stableStartMs) : 0);
    return;
  }
  // -------- PHASE CONTROLLER (50 Hz) --------
  RPM_STABLE_BAND  = 100.0f;

  float deg_L = raw_to_deg(raw_L);
  float deg_F = raw_to_deg(raw_F);

  float sepDeg  = wrapDeg180(deg_L - deg_F);
  float ePhiDeg = wrapDeg180(TARGET_PHASE_DEG - sepDeg);
  ////Serial.print("ePhiDeg = ");
  ////Serial.println(ePhiDeg);

  float throttle_phase = throttle_us - (K_P_phi * ePhiDeg);
  throttle_phase = constrain(throttle_phase, throttle_us -2, throttle_us +2);
  esc.writeMicroseconds((int)(throttle_phase));
  Serial.print("ePhiDeg = ");
  Serial.println(ePhiDeg);
  Serial.print("throttle_phase = ");
  Serial.println(throttle_phase);

}