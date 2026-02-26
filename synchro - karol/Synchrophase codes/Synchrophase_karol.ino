#include <SPI.h>
#include <Servo.h>

bool synchro = false;
int thres = 1000;

//gains for RPM matching
double K_I;
double K_P;

double K_I_phi;
double K_P_phi;


float speedITerm_us = 0.0f; 
//Threshold for initiating synchrophasing
constexpr float RPM_STABLE_BAND   = 30.0f;

volatile float TARGET_PHASE_DEG = 90.0f;

double dt, last_time; //parameters for synchrophasing 


// Encoder CS pins (shared SPI bus)
constexpr uint8_t CS_L = 10; //leader
constexpr uint8_t CS_F = 5;

// ESC signal pins
constexpr uint8_t ESC_PIN = 4;   // Motor 2 = follower
// ESC pulse limits
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 1300;

// Base throttle applied to Motor 1 (and feedforward for Motor 2)
volatile int baseThrottleUs = 1070;
int throttle = baseThrottleUs;
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

  return rpm;
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

  return rpm;
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
void setup() {
  // put your setup code here, to run once:
  K_P = 0.0001;
  K_I = 0.0;
  last_time = 0;
  //K_I_phi = 0
  K_P_phi = 0;
  
  
  Serial.begin(115200);
  pinMode(CS_F, OUTPUT);
  digitalWrite(CS_F, HIGH);
  pinMode(CS_L, OUTPUT);
  digitalWrite(CS_L, HIGH);

  SPI.begin();

  esc.attach(ESC_PIN, 1000, 2000);                 // uses Servo library timing (~50 Hz)
  esc.writeMicroseconds(1000);
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:
  double now = micros();
  
  if (last_time == 0) {
    last_time = now;
    return;   // skip first loop
  }

  dt = (now - last_time)/1000000;
  last_time = now;

  uint raw_L = readAngle(CS_L);
  float rpm_L = computeRPM_L(raw_L);
  float rpm_L_f = ema_L(rpm_L);
  Serial.print("rpm_F = ");
  Serial.println(rpm_F_f);
  if (rpm_L_f < thres) {
    speedITerm_us = 0;
    throttle = baseThrottleUs;
    esc.writeMicroseconds(throttle);
    return;
  }
  uint raw_F = readAngle(CS_F);
  float rpm_F = computeRPM_F(raw_F);
  float rpm_F_f = ema_F(rpm_F);
  Serial.print("rpm_F = ");
  Serial.println(rpm_F_f);
  float eSpeed = rpm_L_f - rpm_F_f;

  if (fabsf(eSpeed) > RPM_STABLE_BAND) {
    speedITerm_us += (K_I * eSpeed) * dt;
    speedITerm_us = constrain(speedITerm_us, -200.0f, 200.0f);
    float uSpeed_us = K_P * eSpeed + speedITerm_us;
    throttle = constrain(int(throttle + uSpeed_us), ESC_MIN_US, ESC_MAX_US);
    esc.writeMicroseconds(throttle);
    return;
  }
  speedITerm_us = 0.0f;
  float deg_L = raw_to_deg(raw_L);
  float deg_F = raw_to_deg(raw_F);
  
  float sepDeg = wrapDeg180(deg_L - deg_F);

  float ePhiDeg = wrapDeg180(TARGET_PHASE_DEG - sepDeg);

  throttle = throttle + (K_P_phi * ePhiDeg) ;
     esc.writeMicroseconds(throttle);

}

