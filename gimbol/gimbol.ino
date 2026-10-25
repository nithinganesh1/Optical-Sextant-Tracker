#include <Wire.h>
#include <math.h>
#include <7Semi_ISM330DHCX.h>

ISM330DHCX_7Semi imu;

// =============================================
// ROLL MOTOR - TB6600
// =============================================

#define ROLL_STEP_PIN 12
#define ROLL_DIR_PIN  13

const bool ROLL_REVERSE = true;

// =============================================
// IMU FILTER
// =============================================

const float RAD_TO_DEG_F = 57.2957795f;
const float ALPHA = 0.98f;

const float ROLL_DEADBAND = 2.0f;

// =============================================
// ROLL PD CONTROLLER
// =============================================

const float KP = 12.0f;
const float KD = 2.0f;

const float MAX_STEP_RATE = 500.0f;
const float MIN_STEP_RATE = 25.0f;

const unsigned int STEP_PULSE_US = 5;

// =============================================
// TIMING
// =============================================

const unsigned long IMU_INTERVAL_US = 10000;
const unsigned long SERIAL_INTERVAL_MS = 100;

// =============================================
// IMU STATE
// =============================================

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
unsigned long lastSerialSend = 0;

// =============================================
// MOTOR STATE
// =============================================

bool pulseHigh = false;

unsigned long pulseStartMicros = 0;
unsigned long lastStepMicros = 0;

float motorRate = 0.0f;

bool imuValid = false;

// =============================================
// GYRO CALIBRATION
// =============================================

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

// =============================================
// ROLL MOTOR STEP GENERATION
// =============================================

void serviceMotor(float signedRate) {

  unsigned long now = micros();

  // Finish current pulse
  if (pulseHigh) {

    if ((unsigned long)(now - pulseStartMicros)
        >= STEP_PULSE_US) {

      digitalWrite(ROLL_STEP_PIN, LOW);
      pulseHigh = false;
    }

    return;
  }

  float rate = fabs(signedRate);

  if (rate < MIN_STEP_RATE) {
    return;
  }

  if (rate > MAX_STEP_RATE) {
    rate = MAX_STEP_RATE;
  }

  bool direction = signedRate > 0.0f;

  if (ROLL_REVERSE) {
    direction = !direction;
  }

  digitalWrite(
    ROLL_DIR_PIN,
    direction ? HIGH : LOW
  );

  unsigned long interval =
      (unsigned long)(1000000.0f / rate);

  if ((unsigned long)(now - lastStepMicros)
      >= interval) {

    lastStepMicros = now;

    digitalWrite(ROLL_STEP_PIN, HIGH);

    pulseHigh = true;
    pulseStartMicros = now;
  }
}

// =============================================
// IMU INITIALIZATION
// =============================================

bool initializeIMU() {

  Wire.begin();
  Wire.setClock(100000);

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

// =============================================
// UPDATE IMU DATA AND ROLL CONTROL
// =============================================

void updateIMU() {

  float ax, ay, az;
  float gxRaw, gyRaw, gzRaw;

  if (!imu.readAccel(ax, ay, az) ||
      !imu.readGyro(gxRaw, gyRaw, gzRaw)) {

    imuValid = false;
    motorRate = 0.0f;
    return;
  }

  unsigned long now = micros();

  float dt =
      (unsigned long)(now - previousMicros)
      / 1000000.0f;

  previousMicros = now;

  if (dt <= 0.0f || dt > 0.2f) {

    imuValid = false;
    motorRate = 0.0f;
    return;
  }

  gx = gxRaw - gxBias;
  gy = gyRaw - gyBias;
  gz = gzRaw - gzBias;

  // =====================================
  // ROLL ANGLE
  // =====================================

  float accRoll =
      atan2(ay, az) * RAD_TO_DEG_F;

  roll =
      ALPHA * (roll + gx * dt)
      + (1.0f - ALPHA) * accRoll;

  // =====================================
  // PITCH ANGLE - TELEMETRY ONLY
  // =====================================

  float accPitch =
      atan2(
        -ax,
        sqrt(ay * ay + az * az)
      ) * RAD_TO_DEG_F;

  pitch =
      ALPHA * (pitch - gy * dt)
      + (1.0f - ALPHA) * accPitch;

  // =====================================
  // YAW - RELATIVE ESTIMATE
  // =====================================

  yaw += gz * dt;

  if (yaw > 180.0f) {
    yaw -= 360.0f;
  }

  if (yaw < -180.0f) {
    yaw += 360.0f;
  }

  // =====================================
  // ROLL MOTOR PD CONTROL
  // =====================================

  motorRate = 0.0f;

  if (fabs(roll) > ROLL_DEADBAND) {

    motorRate = -(KP * roll + KD * gx);

    if (motorRate > MAX_STEP_RATE) {
      motorRate = MAX_STEP_RATE;
    }

    if (motorRate < -MAX_STEP_RATE) {
      motorRate = -MAX_STEP_RATE;
    }
  }

  imuValid = true;
}

// =============================================
// SEND IMU DATA TO ESP32
// =============================================

void sendIMUData() {

  if (!imuValid) {
    if (Serial.availableForWrite() >= 16) {
      Serial.println("IMU,ERR");
    }
    return;
  }

  // Format: IMU,ROLL,PITCH,YAW
  // Reserve space to avoid blocking when TX
  // buffer is full.

  if (Serial.availableForWrite() < 48) {
    return;
  }

  Serial.print("IMU,");

  Serial.print(roll, 2);
  Serial.print(",");

  Serial.print(pitch, 2);
  Serial.print(",");

  Serial.println(yaw, 2);
}

// =============================================
// SETUP
// =============================================

void setup() {

  Serial.begin(115200);

  pinMode(ROLL_STEP_PIN, OUTPUT);
  pinMode(ROLL_DIR_PIN, OUTPUT);

  digitalWrite(ROLL_STEP_PIN, LOW);
  digitalWrite(ROLL_DIR_PIN, LOW);

  if (!initializeIMU()) {
    Serial.println("IMU,ERR");

    while (true) {
      digitalWrite(ROLL_STEP_PIN, LOW);
      delay(100);
    }
  }

  delay(500);

  // Keep IMU stationary during calibration
  if (!calibrateGyro()) {
    Serial.println("IMU,ERR");

    while (true) {
      digitalWrite(ROLL_STEP_PIN, LOW);
      delay(100);
    }
  }

  // Initialize roll and pitch
  float ax, ay, az;

  if (!imu.readAccel(ax, ay, az)) {
    Serial.println("IMU,ERR");

    while (true) {
      digitalWrite(ROLL_STEP_PIN, LOW);
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
  lastStepMicros = previousMicros;
  lastSerialSend = millis();

  imuValid = true;
}

// =============================================
// MAIN LOOP
// =============================================

void loop() {

  unsigned long now = micros();

  // Read IMU approximately every 10ms
  if ((unsigned long)(now - lastIMURead)
      >= IMU_INTERVAL_US) {

    lastIMURead = now;
    updateIMU();
  }

  // Continuously service roll motor
  serviceMotor(motorRate);

  // Send roll, pitch and yaw every 100ms
  if ((unsigned long)(millis() - lastSerialSend)
      >= SERIAL_INTERVAL_MS) {

    lastSerialSend = millis();
    sendIMUData();
  }
}