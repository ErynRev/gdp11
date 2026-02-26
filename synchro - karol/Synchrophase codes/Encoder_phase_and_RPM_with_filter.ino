#include <SPI.h>
#include <Servo.h>
const int THROTTLE_US = 1090;
//
const int esc_PIN = 4;
const int CS_PIN = 5;


// Read angle (0x3FFF) using the “pipelined” read style:
// first transfer clocks in the response, second reads it out.
Servo esc;

uint16_t readAngle() {
  uint16_t angle;

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // start slow

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(2);
  SPI.transfer16(0xFFFF);         // command frame for angle (0x3FFF read)
  digitalWrite(CS_PIN, HIGH);

  delayMicroseconds(2);

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(2);
  angle = SPI.transfer16(0x0000); // read response
  digitalWrite(CS_PIN, HIGH);

  SPI.endTransaction();

  return angle & 0x3FFF; // 14-bit angle
}
float computeRPM(uint16_t currentAngle) {
  static uint16_t prevAngle = 0;
  static unsigned long prevTime = 0;

  unsigned long currentTime = micros();

  // First call guard
  if (prevTime == 0) {
    prevAngle = currentAngle;
    prevTime = currentTime;
    return 0.0;
  }

  // Time difference in seconds
  float dt = (currentTime - prevTime) * 1e-6;
  if (dt <= 0) return 0.0;

  // Angle difference (signed)
  int32_t dAngle = (int32_t)currentAngle - (int32_t)prevAngle;

  // Handle wrap-around
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  // Convert to revolutions
  float revolutions = (float)dAngle / 16384.0;

  // RPM
  float rpm = (revolutions / dt) * 60.0;

  // Update state
  prevAngle = currentAngle;
  prevTime = currentTime;

  return rpm;
}
float computeRPM_karol() {
  uint16_t raw1 = readAngle();
  unsigned long currentTime1 = micros();
  delayMicroseconds(10);
  uint16_t raw2 = readAngle();
  unsigned long currentTime2 = micros();
  
  float dt = (currentTime2 - currentTime1) * 1e-6;

  int32_t dAngle = (int32_t)raw2 - (int32_t)raw1;
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;
  
  float revolutions = (float)dAngle / 16384.0;

  // RPM
  float rpm = (revolutions / dt) * 60.0;

  return rpm;
  
}

// --- 10-sample moving average for RPM ---
const int MA_N = 10;
float rpmBuf[MA_N] = {0};
float rpmSum = 0.0;
int rpmIdx = 0;
int rpmCount = 0;  // ramps up from 0..MA_N

float movingAverageRPM(float newRpm) {
  // remove the value we're about to overwrite
  rpmSum -= rpmBuf[rpmIdx];

  // store new value
  rpmBuf[rpmIdx] = newRpm;
  rpmSum += newRpm;

  // advance ring index
  rpmIdx = (rpmIdx + 1) % MA_N;

  // handle startup (first 9 samples)
  if (rpmCount < MA_N) rpmCount++;

  return rpmSum / rpmCount;
}

// --- fast EMA for RPM ---
static float rpmEma = 0.0f;
static bool  rpmInit = false;
const float ALPHA = 0.04f;   // 0.05..0.30 (smaller = smoother)

inline float ema(float x) {
  if (!rpmInit) { rpmEma = x; rpmInit = true; }
  else          { rpmEma += ALPHA * (x - rpmEma); }
  return rpmEma;
}

void setup() {
  Serial.begin(115200);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  esc.attach(esc_PIN, 1000, 2000);                 // uses Servo library timing (~50 Hz)
  esc.writeMicroseconds(1000);
  delay(3000);
  
}

void loop() {
  /*
  esc.writeMicroseconds(THROTTLE_US);
  
   uint16_t raw = readAngle();
  uint16_t rawQ = quantizeTo1Deg(raw);
  float rpm = computeRPM(rawQ);
  float deg = raw * 360.0 / 16384.0;

  //float rpmSmooth = movingAverageRPM(rpm);

  //Serial.println(rpmSmooth, 2);
  Serial.println(deg, 2);
  delay(10);
  */
  esc.writeMicroseconds(THROTTLE_US);
  uint16_t raw  = readAngle();
  // If you want max speed, don't quantize:
  // float rpm = computeRPM(raw);
  float rpm = computeRPM(raw);
  float rpmSmooth = ema(rpm);

  Serial.println(rpmSmooth, 2);
  delay(10);
}
