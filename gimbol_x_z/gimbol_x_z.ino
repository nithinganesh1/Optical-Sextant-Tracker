#include <Wire.h>
#include <math.h>
#include <7Semi_ISM330DHCX.h>

ISM330DHCX_7Semi imu;

// ========================================
// MOTOR A - ROLL (TB6600)
// ========================================

#define ROLL_STEP_PIN 12
#define ROLL_DIR_PIN  13

const bool ROLL_REVERSE = true;

// ========================================
// MOTOR Z - PITCH (CNC SHIELD)
// ========================================

#define PITCH_STEP_PIN 4
#define PITCH_DIR_PIN  7

#define CNC_ENABLE_PIN 8

const bool PITCH_REVERSE = false;

// ========================================
// IMU CONFIGURATION
// ========================================

const float RAD_TO_DEG_F = 57.2957795f;

const float ALPHA = 0.98f;

const float ROLL_DEADBAND = 2.0f;
const float PITCH_DEADBAND = 2.0f;

// ========================================
// ROLL PD CONTROLLER
// ========================================

const float ROLL_KP = 12.0f;
const float ROLL_KD = 2.0f;

// ========================================
// PITCH PD CONTROLLER
// ========================================

const float PITCH_KP = 12.0f;
const float PITCH_KD = 2.0f;

// ========================================
// MOTOR SETTINGS
// ========================================

const float MAX_STEP_RATE = 500.0f;
const float MIN_STEP_RATE = 25.0f;

const unsigned long STEP_PULSE_US = 5;

// ========================================
// TIMING
// ========================================

// IMU sampling at approximately 100 Hz
const unsigned long IMU_INTERVAL_US = 10000;

// Send telemetry every 100ms (10Hz)
const unsigned long TELEMETRY_INTERVAL_MS = 100;

// ========================================
// IMU STATE
// ========================================

float gxBias = 0.0f;
float gyBias = 0.0f;
float gzBias = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

unsigned long previousMicros = 0;
unsigned long lastIMURead = 0;
unsigned long lastTelemetry = 0;

// Do not report stale values as valid
bool imuDataValid = false;
unsigned int consecutiveIMUErrors = 0;

// ========================================
// STEPPER MOTOR STRUCTURE
// ========================================

struct StepperMotor {

  byte stepPin;
  byte dirPin;

  bool reverse;

  bool pulseHigh;

  unsigned long pulseStart;
  unsigned long lastStep;

  float stepRate;
};

// ========================================
// MOTOR OBJECTS
// ========================================

StepperMotor rollMotor = {
  ROLL_STEP_PIN,
  ROLL_DIR_PIN,
  ROLL_REVERSE,
  false,
  0,
  0,
  0
};

StepperMotor pitchMotor = {
  PITCH_STEP_PIN,
  PITCH_DIR_PIN,
  PITCH_REVERSE,
  false,
  0,
  0,
  0
};

// ========================================
// MOTOR INITIALIZATION
// ========================================

void initializeMotor(StepperMotor &motor) {

  pinMode(motor.stepPin, OUTPUT);
  pinMode(motor.dirPin, OUTPUT);

  digitalWrite(motor.stepPin, LOW);
  digitalWrite(motor.dirPin, LOW);

  motor.pulseHigh = false;
  motor.pulseStart = 0;
  motor.lastStep = micros();

  motor.stepRate = 0.0f;
}

// ========================================
// MOTOR STEP GENERATION
// ========================================

void serviceMotor(StepperMotor &motor) {

  unsigned long now = micros();

  // Finish an active step pulse
  if (motor.pulseHigh) {

    if ((unsigned long)(now - motor.pulseStart)
        >= STEP_PULSE_US) {

      digitalWrite(motor.stepPin, LOW);

      motor.pulseHigh = false;
    }

    return;
  }

  float rate = fabs(motor.stepRate);

  if (rate < MIN_STEP_RATE) {
    return;
  }

  if (rate > MAX_STEP_RATE) {
    rate = MAX_STEP_RATE;
  }

  bool direction = motor.stepRate > 0.0f;

  if (motor.reverse) {
    direction = !direction;
  }

  digitalWrite(
    motor.dirPin,
    direction ? HIGH : LOW
  );

  unsigned long interval =
      (unsigned long)(1000000.0f / rate);

  if ((unsigned long)(now - motor.lastStep)
      >= interval) {

    motor.lastStep = now;

    digitalWrite(motor.stepPin, HIGH);

    motor.pulseHigh = true;
    motor.pulseStart = now;
  }
}

// ========================================
// GYROSCOPE CALIBRATION
// ========================================

bool calibrateGyro() {

  const int SAMPLES = 200;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  int count = 0;

  unsigned long startTime = millis();

  while (count < SAMPLES) {

    float x, y, z;

    if (imu.readGyro(x, y, z)) {

      sumX += x;
      sumY += y;
      sumZ += z;

      count++;
    }

    if (millis() - startTime > 10000) {
      return false;
    }

    delay(10);
  }

  gxBias = sumX / SAMPLES;
  gyBias = sumY / SAMPLES;
  gzBias = sumZ / SAMPLES;

  return true;
}

// ========================================
// IMU INITIALIZATION
// ========================================

bool initializeIMU() {

  Wire.begin();
  Wire.setClock(100000);

  // Helps prevent indefinite I2C waits
  // Supported by recent Arduino AVR Wire cores
  Wire.setWireTimeout(3000, true);

  if (!imu.begin(Wire, 0x6B, 100000)) {
    return false;
  }

  imu.setAccelConfig(
    XL_ODR_104Hz,
    XL_FS_4G,
    true
  );

  imu.setGyroConfig(
    G_ODR_104Hz,
    G_FS_500,
    true
  );

  return true;
}

// ========================================
// IMU READING + STABILIZATION
// ========================================

void updateGimbal() {

  float ax, ay, az;
  float gxRaw, gyRaw, gzRaw;

  bool accelOK = imu.readAccel(ax, ay, az);
  bool gyroOK  = imu.readGyro(gxRaw, gyRaw, gzRaw);

  if (!accelOK || !gyroOK) {

    imuDataValid = false;
    consecutiveIMUErrors++;

    rollMotor.stepRate = 0;
    pitchMotor.stepRate = 0;

    return;
  }

  consecutiveIMUErrors = 0;

  unsigned long now = micros();

  float dt =
      (unsigned long)(now - previousMicros)
      / 1000000.0f;

  previousMicros = now;

  if (dt <= 0.0f || dt > 0.2f) {

    imuDataValid = false;

    rollMotor.stepRate = 0;
    pitchMotor.stepRate = 0;

    return;
  }

  gx = gxRaw - gxBias;
  gy = gyRaw - gyBias;
  gz = gzRaw - gzBias;

  // =====================================
  // ROLL
  // =====================================

  float accRoll =
      atan2(ay, az) * RAD_TO_DEG_F;

  roll =
      ALPHA * (roll + gx * dt)
      + (1.0f - ALPHA) * accRoll;

  // =====================================
  // PITCH
  // =====================================

  float accPitch =
      atan2(
        -ax,
        sqrt(ay * ay + az * az)
      ) * RAD_TO_DEG_F;

  float pitchRate = -gy;

  pitch =
      ALPHA * (pitch + pitchRate * dt)
      + (1.0f - ALPHA) * accPitch;

  // =====================================
  // YAW - GYROSCOPE INTEGRATION
  // =====================================

  yaw += gz * dt;

  // Keep estimated yaw within -180 to +180
  if (yaw > 180.0f) {
    yaw -= 360.0f;
  }

  if (yaw < -180.0f) {
    yaw += 360.0f;
  }

  // =====================================
  // ROLL PD CONTROL
  // =====================================

  float rollRate = 0.0f;

  if (fabs(roll) > ROLL_DEADBAND) {

    rollRate =
        -(ROLL_KP * roll + ROLL_KD * gx);

    rollRate = constrain(
      rollRate,
      -MAX_STEP_RATE,
      MAX_STEP_RATE
    );
  }

  // =====================================
  // PITCH PD CONTROL
  // =====================================

  float pitchMotorRate = 0.0f;

  if (fabs(pitch) > PITCH_DEADBAND) {

    pitchMotorRate =
        -(PITCH_KP * pitch
          + PITCH_KD * pitchRate);

    pitchMotorRate = constrain(
      pitchMotorRate,
      -MAX_STEP_RATE,
      MAX_STEP_RATE
    );
  }

  rollMotor.stepRate = rollRate;
  pitchMotor.stepRate = pitchMotorRate;

  imuDataValid = true;
}

// ========================================
// SERIAL TELEMETRY
// ========================================

void sendIMUData() {

  // Avoid transmitting while the outgoing
  // serial buffer does not have enough room.
  // Approximate record length is under 48B.
  if (Serial.availableForWrite() < 48) {
    return;
  }

  if (!imuDataValid) {

    Serial.println("IMU,ERR");
    return;
  }

  // Format:
  // IMU,ROLL,PITCH,YAW

  Serial.print("IMU,");

  Serial.print(roll, 2);
  Serial.print(",");

  Serial.print(pitch, 2);
  Serial.print(",");

  Serial.println(yaw, 2);
}

// ========================================
// SETUP
// ========================================

void setup() {

  // Hardware UART TX = Arduino D1
  Serial.begin(115200);

  initializeMotor(rollMotor);
  initializeMotor(pitchMotor);

  // Disable CNC shield during startup
  pinMode(CNC_ENABLE_PIN, OUTPUT);
  digitalWrite(CNC_ENABLE_PIN, HIGH);

  if (!initializeIMU()) {

    // Stay stopped if IMU is unavailable
    while (true) {
      digitalWrite(CNC_ENABLE_PIN, HIGH);
      delay(100);
    }
  }

  delay(500);

  // Keep IMU stationary during calibration
  if (!calibrateGyro()) {

    while (true) {
      digitalWrite(CNC_ENABLE_PIN, HIGH);
      delay(100);
    }
  }

  float ax, ay, az;

  if (!imu.readAccel(ax, ay, az)) {

    while (true) {
      digitalWrite(CNC_ENABLE_PIN, HIGH);
      delay(100);
    }
  }

  roll = atan2(ay, az) * RAD_TO_DEG_F;

  pitch =
      atan2(
        -ax,
        sqrt(ay * ay + az * az)
      ) * RAD_TO_DEG_F;

  yaw = 0.0f;

  previousMicros = micros();
  lastIMURead = previousMicros;
  lastTelemetry = millis();

  imuDataValid = true;

  // Enable Z-axis driver
  digitalWrite(CNC_ENABLE_PIN, LOW);
}

// ========================================
// MAIN LOOP
// ========================================

void loop() {

  unsigned long now = micros();

  // 1. IMU and gimbal control
  if ((unsigned long)(now - lastIMURead)
      >= IMU_INTERVAL_US) {

    lastIMURead = now;

    updateGimbal();
  }

  // 2. Independent motor movement
  serviceMotor(rollMotor);
  serviceMotor(pitchMotor);

  // 3. Send telemetry at 10Hz
  if ((unsigned long)(millis() - lastTelemetry)
      >= TELEMETRY_INTERVAL_MS) {

    lastTelemetry = millis();

    sendIMUData();
  }
}