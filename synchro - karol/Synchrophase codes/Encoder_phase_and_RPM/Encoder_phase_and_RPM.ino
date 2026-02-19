#include <SPI.h>

const int CS_PIN = 10;

// Read angle (0x3FFF) using the “pipelined” read style:
// first transfer clocks in the response, second reads it out.
uint16_t readAngle() {
  uint16_t angle;

  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1)); // start slow

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


void setup() {
  Serial.begin(115200);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
}

void loop() {
  uint16_t raw = readAngle();
  float deg = raw * 360.0 / 16384.0;
  float rpm = computeRPM(raw);

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print("  deg=");
  Serial.print(deg, 2);
  Serial.print("  rpm=");
  Serial.println(rpm, 2);

  delay(1);
}
