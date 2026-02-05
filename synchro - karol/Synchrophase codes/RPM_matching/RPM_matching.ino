#include <SPI.h>
#include <Servo.h>

// -------------------- USER CONFIG --------------------
// SPI chip selects for the 2 encoders (same SPI bus)
constexpr uint8_t CS_M1 = 10;
constexpr uint8_t CS_M2 = 9;

// ESC signal pins
constexpr uint8_t ESC1_PIN = 2;   // Motor 1 (reference)
constexpr uint8_t ESC2_PIN = 3;   // Motor 2 (follower)

// ESC pulse limits (typical "servo" range)
constexpr int ESC_MIN_US = 1000;
constexpr int ESC_MAX_US = 2000;

// Base throttle applied to Motor 1 (and feedforward for Motor 2)
volatile int baseThrottleUs = 1200;  // start low; change via Serial

// Control loop period (RPM estimation + PI update)
constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz control loop

// SPI speed for AS5147P (datasheet allows up to 10 MHz; start modest)
constexpr uint32_t SPI_HZ = 1000000; // 1 MHz
constexpr uint8_t  SPI_MODE = SPI_MODE1;

// PI gains (YOU WILL TUNE THESE)
volatile float Kp_us_per_rpm = 0.05f;     // proportional gain [microseconds / RPM]
volatile float Ki_us_per_rpm_s = 0.20f;   // integral gain [microseconds / (RPM*s)]

// Anti-windup / clamps
constexpr float ITERM_CLAMP_US = 300.0f;  // clamp integral contribution in microseconds
constexpr float OUT_CLAMP_US   = 250.0f;  // clamp total correction in microseconds

// RPM low-pass filter
constexpr float RPM_ALPHA = 0.80f;        // closer to 1 = smoother, more lag

// -----------------------------------------------------

Servo esc1, esc2;

struct RpmEstimator {
  bool init = false;
  uint16_t prevAngle = 0;
  uint32_t prevTimeUs = 0;
  float rpmFilt = 0.0f;
};

RpmEstimator rpm1State, rpm2State;
float iTerm_us = 0.0f;   // integral stored in "microseconds of correction"

// Compute parity bit such that total parity over 16 bits is even.
// Datasheet: parity bit is "even" calculated on lower 15 bits. :contentReference[oaicite:5]{index=5}
static inline uint8_t evenParityBit15(uint16_t lower15) {
  lower15 &= 0x7FFF;
  // returns 1 if number of ones is odd (so adding parity bit makes total even)
  uint16_t x = lower15;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

static inline bool checkEvenParity(uint16_t frame) {
  // parity is computed over bits 0..14, stored in bit 15
  uint16_t lower15 = frame & 0x7FFF;
  uint8_t p = evenParityBit15(lower15);
  uint8_t pFrame = (frame >> 15) & 1;
  return (p == pFrame);
}

// Read a 14-bit register using the pipelined SPI read described in the datasheet:
// Data from a read command is returned on the *next* SPI frame. :contentReference[oaicite:6]{index=6}
uint16_t as5147pRead14(uint8_t csPin, uint16_t addr14) {
  // Build command: [15]=parity, [14]=R/W (1=read), [13:0]=addr
  uint16_t cmd = (addr14 & 0x3FFF) | 0x4000;     // set R/W=1
  cmd |= (uint16_t(evenParityBit15(cmd)) << 15); // parity over lower 15 bits

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE));

  // Frame 1: send read command (response is previous data; ignore)
  digitalWrite(csPin, LOW);
  SPI.transfer16(cmd);
  digitalWrite(csPin, HIGH);

  // Small CS high time (datasheet timing is ns-scale; this is safely longer)
  delayMicroseconds(1);

  // Frame 2: clock out the response
  digitalWrite(csPin, LOW);
  uint16_t resp = SPI.transfer16(0x0000);
  digitalWrite(csPin, HIGH);

  SPI.endTransaction();

  // Response format: [15]=parity, [14]=EF (error flag), [13:0]=DATA :contentReference[oaicite:7]{index=7}
  bool parityOK = checkEvenParity(resp);
  bool errorFlag = (resp & 0x4000) != 0;

  if (!parityOK || errorFlag) {
    // If you want, you can count errors here.
    // Return 0xFFFF as "invalid" sentinel.
    return 0xFFFF;
  }

  return resp & 0x3FFF;
}

uint16_t readAngleMotor1() { return as5147pRead14(CS_M1, 0x3FFF); }
uint16_t readAngleMotor2() { return as5147pRead14(CS_M2, 0x3FFF); }

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

  // Wrap handling for 14-bit angle (0..16383)
  if (dAngle > 8192)  dAngle -= 16384;
  if (dAngle < -8192) dAngle += 16384;

  float rev = float(dAngle) / 16384.0f;
  float rpmRaw = (rev / dt) * 60.0f;

  // Low-pass filter
  st.rpmFilt = RPM_ALPHA * st.rpmFilt + (1.0f - RPM_ALPHA) * rpmRaw;

  st.prevAngle = angle14;
  st.prevTimeUs = nowUs;

  return st.rpmFilt;
}

void armESCs() {
  // Safety: remove props before running this.
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.writeMicroseconds(ESC_MIN_US);
  delay(3000); // typical arming dwell
}

void handleSerialTuning() {
  // Simple serial protocol:
  //  t<us>\n     -> set base throttle (e.g. t1200)
  //  p<float>\n  -> set Kp (e.g. p0.05)
  //  i<float>\n  -> set Ki (e.g. i0.20)
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == 't') {
    int v = Serial.parseInt();
    baseThrottleUs = constrain(v, ESC_MIN_US, ESC_MAX_US);
  } else if (c == 'p') {
    float v = Serial.parseFloat();
    Kp_us_per_rpm = v;
  } else if (c == 'i') {
    float v = Serial.parseFloat();
    Ki_us_per_rpm_s = v;
  } else {
    // flush line
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

  Serial.println("Ready. Commands: t<us>, p<float>, i<float>  (e.g. t1200, p0.05, i0.20)");
}

void loop() {
  handleSerialTuning();

  static uint32_t lastControlUs = micros();
  static uint32_t lastPrintUs = micros();

  uint32_t nowUs = micros();

  // Run control loop at fixed-ish period
  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    lastControlUs += CONTROL_PERIOD_US;

    // Read both angles
    uint16_t a1 = readAngleMotor1();
    uint16_t a2 = readAngleMotor2();

    // If invalid read, skip this iteration (keep last outputs)
    if (a1 != 0xFFFF && a2 != 0xFFFF) {
      float rpm1 = updateRPM(rpm1State, a1, nowUs);
      float rpm2 = updateRPM(rpm2State, a2, nowUs);

      // PI to make Motor2 match Motor1
      float err = rpm1 - rpm2;
      float dt = CONTROL_PERIOD_US * 1e-6f;

      // Integral in microseconds directly (so clamp is easy)
      iTerm_us += (Ki_us_per_rpm_s * err) * dt;
      iTerm_us = constrain(iTerm_us, -ITERM_CLAMP_US, ITERM_CLAMP_US);

      float pTerm_us = Kp_us_per_rpm * err;
      float corr_us = pTerm_us + iTerm_us;
      corr_us = constrain(corr_us, -OUT_CLAMP_US, OUT_CLAMP_US);

      int out1 = baseThrottleUs;
      int out2 = int(baseThrottleUs + corr_us);

      out1 = constrain(out1, ESC_MIN_US, ESC_MAX_US);
      out2 = constrain(out2, ESC_MIN_US, ESC_MAX_US);

      esc1.writeMicroseconds(out1);
      esc2.writeMicroseconds(out2);

      // Print at ~20 Hz
      if ((uint32_t)(nowUs - lastPrintUs) >= 50000) {
        lastPrintUs = nowUs;

        Serial.print("rpm1=");
        Serial.print(rpm1, 1);
        Serial.print("  rpm2=");
        Serial.print(rpm2, 1);
        Serial.print("  err=");
        Serial.print(err, 1);
        Serial.print("  baseUs=");
        Serial.print(baseThrottleUs);
        Serial.print("  corrUs=");
        Serial.print(corr_us, 1);
        Serial.print("  out2Us=");
        Serial.println(out2);
      }
    }
  }
}
