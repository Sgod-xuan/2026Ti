#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

const byte BT_RX_PIN = 4; // Nano D4 <- ZS-040 TXD
const byte BT_TX_PIN = 7; // Nano D7 -> ZS-040 RXD through a voltage divider
const unsigned long DEBUG_BAUD = 9600;
const unsigned long BT_BAUD = 9600;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// ==================== 1. Hardware pins ====================
const int MAI1 = 9;   const int MAI2 = 6;
const int MBI1 = 10;  const int MBI2 = 5;
const int MAEB = 2;   const int MBEB = 3;

const int sensorPins[] = {A0, A1, A2, A3};

// HC-SR04: Echo_TH_SOA -> D11, Trig_RX_scl_I/O -> D12.
const int echoPin = 11;
const int trigPin = 12;

// GY-521 / MPU6050 uses Nano I2C: SDA=A4, SCL=A5.
const byte MPU_ADDR = 0x68;
const float GYRO_Z_SENS = 131.0;  // LSB/(deg/s), +/-250 dps full scale.

// ==================== 2. Tuning constants ====================
const int SAMPLE_TIME_MS = 25;

// Line following PID.
float Kp_line = 14.0;
float Kd_line = 18.0;
float Kp_spd = 1.05;
float Ki_spd = 0.08;
float Kd_spd = 0.04;
const float BASE_RPM = 80.0;
const float MAX_TARGET_RPM = 130.0;

// Crossline detection.
const bool LINE_BLACK_IS_LOW = true;
const unsigned long START_DELAY_MS = 3000;
const int CROSS_SENSOR_REQUIRED = 4;
const int CROSS_EXIT_SENSOR_MAX = 2;
const int CROSS_EXIT_SAMPLE_REQUIRED = 3;
const float SECOND_CROSS_ALIGN_TARGET_DEG = 180.0;

// Encoder based short moves. Tune WHEEL_DIAMETER_CM first if 10 cm / 5 cm is off.
const float ENCODER_COUNTS_PER_REV = 390.0;
const float WHEEL_DIAMETER_CM = 6.5;
const float PI_F = 3.1415926;
const int POS_PWM_MIN = 42;
const int POS_PWM_MAX = 78;
const float KP_POS_RPM = 0.95;
const float POS_RPM_MIN = 16.0;
const float POS_RPM_MAX = 42.0;
const unsigned long POS_CONTROL_INTERVAL_MS = 20;
const int POS_STOP_COUNTS = 5;
const int SEARCH_PWM = 46;
const float LINE_TRIM_KP = 12.0;
const int LINE_TRIM_MAX = 18;
const unsigned long REVERSE_SEARCH_IGNORE_MS = 300;
const float REVERSE_BOOST_RPM = 45.0;
const unsigned long REVERSE_BOOST_MS = 500;
const float REVERSE_RPM = 28.0;
const float LINE_TRIM_RPM_KP = 8.0;
const float LINE_TRIM_RPM_MAX = 18.0;
const unsigned long REVERSE_CONTROL_INTERVAL_MS = 20;
const int REVERSE_PWM_MIN = 42;
const int REVERSE_PWM_MAX = 78;
const float Kp_rev_spd = 1.0;
const float Ki_rev_spd = 0.15;
const float Kd_rev_spd = 0.06;
const float KP_REVERSE_SYNC_RPM = 0.08;
const int REVERSE_LEFT_PWM_TRIM = -5;   // Motor A / left wheel: negative slows it during reverse.
const int REVERSE_RIGHT_PWM_TRIM = -10; // Motor B / right wheel: negative slows it during reverse.

// Required post-second-cross motions.
const float AFTER_SECOND_FORWARD_CM = 5.0;
const float AFTER_TURN_FORWARD_CM = 5.0;

// Gyro position-loop right turn.
const float LEFT_TURN_DEG = 90.0;
const int LEFT_TURN_DIR = 1;      // Right turn direction for the mirrored right sketch.
const float KP_ANGLE = 1.05;
const float KD_RATE = 0.16;
const int TURN_PWM_MIN_FAR = 48;
const int TURN_PWM_MIN_NEAR = 40;
const int TURN_PWM_MAX = 74;
const int TURN_PWM_MAX_NEAR = 52;
const float TURN_NEAR_DEG = 10.0;
const float TURN_STOP_ERROR_DEG = 1.0;
const float TURN_STOP_RATE_DPS = 4.0;
const unsigned long TURN_SETTLE_MS = 220;
const unsigned long TURN_TIMEOUT_MS = 8000;
const float KP_ANGLE_TO_WHEEL_RPM = 0.75;
const float TURN_RPM_MIN = 18.0;
const float TURN_RPM_MAX = 45.0;
const unsigned long TURN_CONTROL_INTERVAL_MS = 20;
const float KP_TURN_WHEEL_SPEED = 1.0;
const float KI_TURN_WHEEL_SPEED = 0.18;
const float KD_TURN_WHEEL_SPEED = 0.08;
const int MOTOR_A_TURN_TRIM = 0;
const int MOTOR_B_TURN_TRIM = 0;

// Heading correction at the second crossline. Target is the start heading, yawDeg == 0.
const int HEADING_ALIGN_DIR_SIGN = 1;  // If correction turns farther away, change to -1.
const float KP_HEADING_RATE = 3.0;
const float HEADING_RATE_MIN_DPS = 18.0;
const float HEADING_RATE_MAX_DPS = 70.0;
const float KP_HEADING_SPEED = 1.2;
const float KI_HEADING_SPEED = 0.04;
const float KD_HEADING_SPEED = 0.15;
const int HEADING_PWM_MIN = 40;
const int HEADING_PWM_MAX = 72;
const float HEADING_STOP_ERROR_DEG = 1.2;
const float HEADING_STOP_RATE_DPS = 4.0;
const unsigned long HEADING_SETTLE_MS = 250;
const unsigned long HEADING_TIMEOUT_MS = 4500;

// HC-SR04 measurement.
const unsigned long ECHO_TIMEOUT_US = 30000UL;
const float DISTANCE_OFFSET_CM = 0.4;
const int DISTANCE_AVERAGE_SAMPLES = 10;
const float WALL1_DISTANCE_ERROR_CM = 2.0;  // First wall Bluetooth/output correction.
const float WALL2_DISTANCE_ERROR_CM = 1.5;  // Second wall Bluetooth/output correction.
const float MIN_WALL_DISTANCE_CM = 18.0;    // Project nominal range: 20-200 cm.
const float MAX_WALL_DISTANCE_CM = 220.0;
const float DISTANCE_STABLE_SPAN_CM = 1.0;  // Accept when recent readings differ <= 1 cm.
const int DISTANCE_MAX_READINGS = 15;
const int DISTANCE_STABLE_WINDOW = 3;
const unsigned long TELEMETRY_INTERVAL_MS = 500;

const int SAVED_MEASURE_ADDR = 0;
const uint32_t SAVED_MEASURE_MAGIC = 0x4C434D32UL; // LCM2

struct SavedMeasure {
  uint32_t magic;
  uint16_t runId;
  float wall1;
  float wall2;
};

// ==================== 3. Global state ====================
enum State {
  STATE_WAIT,
  STATE_FINDING_FIRST_CROSS,
  STATE_LEAVING_START_CROSS,
  STATE_RUNNING,
  STATE_POST_SECOND_CROSS,
  STATE_FINISH
};

struct LineRead {
  int s[4];
  int sum;
  bool cross;
  float error;
};

State currentState = STATE_WAIT;

int crossCount = 0;
bool crossDetected = false;
unsigned long lastCrossTime = 0;
unsigned long waitStartTime = 0;
unsigned long lastTime = 0;
unsigned long lastTelemetryMs = 0;
unsigned long programStartTime = 0;
unsigned long crossStableStartTime = 0;
int crossExitSampleCount = 0;

volatile long countA = 0;
volatile long countB = 0;

float targetRPMA = 0.0;
float targetRPMB = 0.0;
float currentRPMA = 0.0;
float currentRPMB = 0.0;
float lastLineError = 0.0;
float integralA = 0.0;
float integralB = 0.0;
float lastErrA = 0.0;
float lastErrB = 0.0;

float gyroZBiasDPS = 0.0;
float yawDeg = 0.0;
float gyroZRateDPS = 0.0;
unsigned long lastGyroMicros = 0;
bool mpuOk = false;

char linkRxBuf[48];
byte linkRxLen = 0;

// ==================== 4. Basic helpers ====================
void countEncoderA() { countA++; }
void countEncoderB() { countB++; }

float absFloat(float v) {
  return v < 0.0 ? -v : v;
}

void motorControl(int pin1, int pin2, float pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0.0) {
    analogWrite(pin1, (int)pwm);
    digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW);
    analogWrite(pin2, (int)abs(pwm));
  }
}

void stopMotors() {
  motorControl(MAI1, MAI2, 0);
  motorControl(MBI1, MBI2, 0);
}

void spinInPlace(int dir, int pwm) {
  pwm = constrain(pwm, 0, 255);
  if (dir >= 0) {
    motorControl(MAI1, MAI2, pwm);
    motorControl(MBI1, MBI2, -pwm);
  } else {
    motorControl(MAI1, MAI2, -pwm);
    motorControl(MBI1, MBI2, pwm);
  }
}

void spinInPlacePWM(int dir, int pwmA, int pwmB) {
  pwmA = constrain(pwmA + MOTOR_A_TURN_TRIM, 0, 255);
  pwmB = constrain(pwmB + MOTOR_B_TURN_TRIM, 0, 255);

  if (dir >= 0) {
    motorControl(MAI1, MAI2, pwmA);
    motorControl(MBI1, MBI2, -pwmB);
  } else {
    motorControl(MAI1, MAI2, -pwmA);
    motorControl(MBI1, MBI2, pwmB);
  }
}

void drivePWM(int dir, int basePwm, float trimPwm) {
  basePwm = constrain(basePwm, 0, 255);
  trimPwm = constrain(trimPwm, -LINE_TRIM_MAX, LINE_TRIM_MAX);

  float pwmA = dir * (basePwm + trimPwm);
  float pwmB = dir * (basePwm - trimPwm);
  motorControl(MAI1, MAI2, pwmA);
  motorControl(MBI1, MBI2, pwmB);
}

void driveReversePWM(int basePwm, float trimPwm) {
  basePwm = constrain(basePwm, 0, 255);
  trimPwm = constrain(trimPwm, -LINE_TRIM_MAX, LINE_TRIM_MAX);

  float pwmA = -(basePwm - trimPwm);
  float pwmB = -(basePwm + trimPwm);
  motorControl(MAI1, MAI2, pwmA);
  motorControl(MBI1, MBI2, pwmB);
}

void driveReversePWMClosedLoop(int pwmA, int pwmB) {
  pwmA = constrain(pwmA + REVERSE_LEFT_PWM_TRIM, 0, 255);
  pwmB = constrain(pwmB + REVERSE_RIGHT_PWM_TRIM, 0, 255);
  motorControl(MAI1, MAI2, -pwmA);
  motorControl(MBI1, MBI2, -pwmB);
}

void resetEncoders() {
  noInterrupts();
  countA = 0;
  countB = 0;
  interrupts();
}

void readEncoders(long &a, long &b) {
  noInterrupts();
  a = countA;
  b = countB;
  interrupts();
}

void readAndResetEncoders(long &a, long &b) {
  noInterrupts();
  a = countA;
  b = countB;
  countA = 0;
  countB = 0;
  interrupts();
}

void resetSpeedPid() {
  integralA = 0.0;
  integralB = 0.0;
  lastErrA = 0.0;
  lastErrB = 0.0;
}

const char *stateName(State state) {
  switch (state) {
    case STATE_WAIT:
      return "WAIT";
    case STATE_FINDING_FIRST_CROSS:
      return "FINDING_FIRST_CROSS";
    case STATE_LEAVING_START_CROSS:
      return "LEAVING_START_CROSS";
    case STATE_RUNNING:
      return "RUNNING";
    case STATE_POST_SECOND_CROSS:
      return "POST_SECOND_CROSS";
    case STATE_FINISH:
      return "FINISH";
    default:
      return "UNKNOWN";
  }
}

// ==================== 5. Line and cross helpers ====================
int readBlack(int pin) {
  int value = digitalRead(pin);
  if (LINE_BLACK_IS_LOW) {
    return value == LOW ? 1 : 0;
  }
  return value == HIGH ? 1 : 0;
}

LineRead readLineSensors() {
  LineRead line;
  line.sum = 0;

  for (int i = 0; i < 4; i++) {
    line.s[i] = readBlack(sensorPins[i]);
    line.sum += line.s[i];
  }

  line.cross = line.sum >= CROSS_SENSOR_REQUIRED;

  if (line.s[1] && line.s[2]) {
    line.error = 0.0;
  } else if (line.s[1] && !line.s[2]) {
    line.error = -1.1;
    if (line.s[0]) {
      line.error = -1.7;
    }
  } else if (!line.s[1] && line.s[2]) {
    line.error = 1.1;
    if (line.s[3]) {
      line.error = 1.7;
    }
  } else if (line.s[0] && !line.s[3]) {
    line.error = -2.8;
  } else if (!line.s[0] && line.s[3]) {
    line.error = 2.8;
  } else if (line.sum == 4) {
    line.error = 0.0;
  } else {
    if (lastLineError > 0.2) {
      line.error = 2.8;
    } else if (lastLineError < -0.2) {
      line.error = -2.8;
    } else {
      line.error = 0.0;
    }
  }

  return line;
}

float lineTrimPWM(const LineRead &line) {
  if (line.cross) {
    return 0.0;
  }
  return constrain(line.error * LINE_TRIM_KP, -LINE_TRIM_MAX, LINE_TRIM_MAX);
}

float lineTrimRPM(const LineRead &line) {
  if (line.cross) {
    return 0.0;
  }
  return constrain(line.error * LINE_TRIM_RPM_KP, -LINE_TRIM_RPM_MAX, LINE_TRIM_RPM_MAX);
}

float cmToEncoderCounts(float cm) {
  float wheelCircumference = PI_F * WHEEL_DIAMETER_CM;
  return cm / wheelCircumference * ENCODER_COUNTS_PER_REV;
}

bool driveDistanceCM(int dir, float cm, int maxPwm, bool useLineTrim, unsigned long timeoutMs) {
  resetEncoders();
  unsigned long startMs = millis();
  unsigned long lastControlMs = millis();
  float targetCounts = cmToEncoderCounts(cm);
  long prevA = 0;
  long prevB = 0;
  float posIntegralA = 0.0;
  float posIntegralB = 0.0;
  float posLastErrA = 0.0;
  float posLastErrB = 0.0;

  Serial.print(F("Drive "));
  Serial.print(dir > 0 ? F("forward ") : F("backward "));
  Serial.print(cm, 1);
  Serial.println(F(" cm"));

  while (millis() - startMs < timeoutMs) {
    unsigned long nowMs = millis();
    if (nowMs - lastControlMs < POS_CONTROL_INTERVAL_MS) {
      delay(1);
      continue;
    }

    float dt = (nowMs - lastControlMs) / 1000.0;
    lastControlMs = nowMs;

    long a, b;
    readEncoders(a, b);
    float countAAbs = abs(a);
    float countBAbs = abs(b);
    float remainingA = targetCounts - countAAbs;
    float remainingB = targetCounts - countBAbs;

    if (remainingA <= POS_STOP_COUNTS && remainingB <= POS_STOP_COUNTS) {
      break;
    }

    float deltaA = abs(a - prevA);
    float deltaB = abs(b - prevB);
    prevA = a;
    prevB = b;

    float rpmA = (deltaA / ENCODER_COUNTS_PER_REV) / dt * 60.0;
    float rpmB = (deltaB / ENCODER_COUNTS_PER_REV) / dt * 60.0;

    float targetA = remainingA > POS_STOP_COUNTS ? constrain(remainingA * KP_POS_RPM, POS_RPM_MIN, POS_RPM_MAX) : 0.0;
    float targetB = remainingB > POS_STOP_COUNTS ? constrain(remainingB * KP_POS_RPM, POS_RPM_MIN, POS_RPM_MAX) : 0.0;

    LineRead line = readLineSensors();
    float trim = useLineTrim ? (dir > 0 ? lineTrimPWM(line) : -lineTrimPWM(line)) : 0.0;
    targetA = constrain(targetA + trim, 0.0, POS_RPM_MAX);
    targetB = constrain(targetB - trim, 0.0, POS_RPM_MAX);

    float errA = targetA - rpmA;
    float errB = targetB - rpmB;
    posIntegralA = constrain(posIntegralA + errA, -120, 120);
    posIntegralB = constrain(posIntegralB + errB, -120, 120);

    float pwmA = Kp_spd * errA + Ki_spd * posIntegralA + Kd_spd * (errA - posLastErrA);
    float pwmB = Kp_spd * errB + Ki_spd * posIntegralB + Kd_spd * (errB - posLastErrB);
    posLastErrA = errA;
    posLastErrB = errB;

    if (targetA > 0.0) {
      pwmA = constrain(POS_PWM_MIN + pwmA, POS_PWM_MIN, maxPwm);
    } else {
      pwmA = 0.0;
    }

    if (targetB > 0.0) {
      pwmB = constrain(POS_PWM_MIN + pwmB, POS_PWM_MIN, maxPwm);
    } else {
      pwmB = 0.0;
    }

    motorControl(MAI1, MAI2, dir * pwmA);
    motorControl(MBI1, MBI2, dir * pwmB);
  }

  stopMotors();
  delay(160);
  return true;
}

bool driveUntilCross(int dir, int targetCrossCount, unsigned long timeoutMs, bool useLineTrim) {
  unsigned long startMs = millis();

  Serial.print(F("Search cross "));
  Serial.print(targetCrossCount);
  Serial.println(dir > 0 ? F(" forward") : F(" backward"));

  while (millis() - startMs < timeoutMs) {
    LineRead line = readLineSensors();
    bool allowStopOnCross = !(dir < 0 && millis() - startMs < REVERSE_SEARCH_IGNORE_MS);

    if (line.cross && allowStopOnCross) {
      stopMotors();
      crossCount = targetCrossCount;
      crossDetected = true;
      lastCrossTime = millis();
      Serial.print(F("Confirmed Crossline: "));
      Serial.println(crossCount);
      delay(250);
      return true;
    }

    float trim = useLineTrim ? (dir > 0 ? lineTrimPWM(line) : -lineTrimPWM(line)) : 0.0;
    drivePWM(dir, SEARCH_PWM, trim);
    delay(10);
  }

  stopMotors();
  Serial.print(F("Cross search timeout, expected "));
  Serial.println(targetCrossCount);
  delay(250);
  return false;
}

bool reverseAlongLineUntilCross(unsigned long timeoutMs) {
  unsigned long startMs = millis();
  unsigned long lastControlMs = millis();
  long prevA = 0;
  long prevB = 0;
  float revIntegralA = 0.0;
  float revIntegralB = 0.0;
  float revLastErrA = 0.0;
  float revLastErrB = 0.0;
  bool ignoreInitialCross = readLineSensors().cross;

  resetEncoders();
  Serial.println(F("Reverse straight until black cross."));

  while (millis() - startMs < timeoutMs) {
    LineRead line = readLineSensors();

    if (ignoreInitialCross) {
      if (!line.cross) {
        ignoreInitialCross = false;
      }
    } else if (line.cross) {
      stopMotors();
      Serial.println(F("Black cross detected. Stop."));
      delay(250);
      return true;
    }

    unsigned long nowMs = millis();
    if (nowMs - lastControlMs < REVERSE_CONTROL_INTERVAL_MS) {
      delay(1);
      continue;
    }

    float dt = (nowMs - lastControlMs) / 1000.0;
    lastControlMs = nowMs;

    float baseRPM = nowMs - startMs < REVERSE_BOOST_MS ? REVERSE_BOOST_RPM : REVERSE_RPM;

    long a, b;
    readEncoders(a, b);
    float deltaA = abs(a - prevA);
    float deltaB = abs(b - prevB);
    prevA = a;
    prevB = b;

    float rpmA = (deltaA / ENCODER_COUNTS_PER_REV) / dt * 60.0;
    float rpmB = (deltaB / ENCODER_COUNTS_PER_REV) / dt * 60.0;
    float syncTrim = constrain((abs(b) - abs(a)) * KP_REVERSE_SYNC_RPM, -6.0, 6.0);
    float targetRPMA = constrain(baseRPM + syncTrim, 0.0, REVERSE_BOOST_RPM + 6.0);
    float targetRPMB = constrain(baseRPM - syncTrim, 0.0, REVERSE_BOOST_RPM + 6.0);

    float errA = targetRPMA - rpmA;
    float errB = targetRPMB - rpmB;
    revIntegralA = constrain(revIntegralA + errA * dt, -120.0, 120.0);
    revIntegralB = constrain(revIntegralB + errB * dt, -120.0, 120.0);

    float rawPwmA = REVERSE_PWM_MIN
      + Kp_rev_spd * errA
      + Ki_rev_spd * revIntegralA
      + Kd_rev_spd * ((errA - revLastErrA) / dt);
    float rawPwmB = REVERSE_PWM_MIN
      + Kp_rev_spd * errB
      + Ki_rev_spd * revIntegralB
      + Kd_rev_spd * ((errB - revLastErrB) / dt);
    revLastErrA = errA;
    revLastErrB = errB;

    int pwmA = targetRPMA > 0.0 ? constrain((int)rawPwmA, REVERSE_PWM_MIN, REVERSE_PWM_MAX) : 0;
    int pwmB = targetRPMB > 0.0 ? constrain((int)rawPwmB, REVERSE_PWM_MIN, REVERSE_PWM_MAX) : 0;
    driveReversePWMClosedLoop(pwmA, pwmB);

    Serial.print(F("S="));
    Serial.print(line.s[0]);
    Serial.print(line.s[1]);
    Serial.print(line.s[2]);
    Serial.print(line.s[3]);
    Serial.print(F(" targetA="));
    Serial.print(targetRPMA, 1);
    Serial.print(F(" targetB="));
    Serial.print(targetRPMB, 1);
    Serial.print(F(" rpmA="));
    Serial.print(rpmA, 1);
    Serial.print(F(" rpmB="));
    Serial.print(rpmB, 1);
    Serial.print(F(" pwmA="));
    Serial.print(pwmA);
    Serial.print(F(" pwmB="));
    Serial.println(pwmB);
  }

  stopMotors();
  Serial.println(F("Timeout. Stop."));
  return false;
}

// ==================== 6. HC-SR04 helpers ====================
float readDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0 + DISTANCE_OFFSET_CM;
}

float readAverageDistanceCM(int samples) {
  float sum = 0.0;
  int validCount = 0;

  for (int i = 0; i < samples; i++) {
    float distance = readDistanceCM();
    if (distance > 0.0) {
      sum += distance;
      validCount++;
    }
    delay(50);
  }

  if (validCount == 0) {
    return -1.0;
  }

  return sum / validCount;
}

void printDistanceToBoth(float distance) {
  if (distance < 0.0) {
    Serial.println(F("Distance: invalid"));
    btSerial.println(F("Distance: invalid"));
  } else {
    Serial.print(F("Distance: "));
    Serial.print(distance, 1);
    Serial.println(F(" cm"));

    btSerial.print(F("Distance: "));
    btSerial.print(distance, 1);
    btSerial.println(F(" cm"));
  }
}

float measureStableWallCM(int wallIndex) {
  Serial.print(F("Measure wall "));
  Serial.println(wallIndex);
  btSerial.print(F("Measure wall "));
  btSerial.println(wallIndex);

  float distance = readAverageDistanceCM(DISTANCE_AVERAGE_SAMPLES);
  return distance;
}

float measureStableWallCMWithError(int wallIndex, float errorCm) {
  float measured = measureStableWallCM(wallIndex);
  if (measured < 0.0) {
    return measured;
  }

  float adjusted = measured + errorCm;
  printDistanceToBoth(adjusted);

  Serial.print(F("Wall "));
  Serial.print(wallIndex);
  Serial.print(F(" adjusted: raw="));
  Serial.print(measured, 1);
  Serial.print(F(" cm, error="));
  Serial.print(errorCm, 1);
  Serial.print(F(" cm, final="));
  Serial.print(adjusted, 1);
  Serial.println(F(" cm"));
  return adjusted;
}

void saveWallMeasurements(float wall1, float wall2) {
  SavedMeasure saved;
  EEPROM.get(SAVED_MEASURE_ADDR, saved);

  uint16_t nextRunId = 1;
  if (saved.magic == SAVED_MEASURE_MAGIC) {
    nextRunId = saved.runId + 1;
  }

  saved.magic = SAVED_MEASURE_MAGIC;
  saved.runId = nextRunId;
  saved.wall1 = wall1;
  saved.wall2 = wall2;
  EEPROM.put(SAVED_MEASURE_ADDR, saved);

  Serial.print(F("Saved measurement run="));
  Serial.print(saved.runId);
  Serial.print(F(", wall1="));
  Serial.print(saved.wall1, 1);
  Serial.print(F(" cm, wall2="));
  Serial.print(saved.wall2, 1);
  Serial.println(F(" cm"));
}

void printSavedWallMeasurements() {
  SavedMeasure saved;
  EEPROM.get(SAVED_MEASURE_ADDR, saved);

  if (saved.magic != SAVED_MEASURE_MAGIC) {
    Serial.println(F("LAST,no saved ultrasonic measurement"));
    return;
  }

  Serial.print(F("LAST,run="));
  Serial.print(saved.runId);
  Serial.print(F(",wall1="));
  Serial.print(saved.wall1, 1);
  Serial.print(F(",wall2="));
  Serial.println(saved.wall2, 1);
}

void sendSavedWallMeasurementsToLink() {
  SavedMeasure saved;
  EEPROM.get(SAVED_MEASURE_ADDR, saved);

  if (saved.magic != SAVED_MEASURE_MAGIC) {
    btSerial.println(F("LAST,none"));
    return;
  }

  btSerial.print(F("LAST,run="));
  btSerial.print(saved.runId);
  btSerial.print(F(",wall1="));
  btSerial.print(saved.wall1, 1);
  btSerial.print(F(",wall2="));
  btSerial.println(saved.wall2, 1);
}

void sendLinkStatus() {
  btSerial.print(F("STATUS,state="));
  btSerial.print(stateName(currentState));
  btSerial.print(F(",cross="));
  btSerial.print(crossCount);
  btSerial.print(F(",yaw="));
  btSerial.print(yawDeg, 1);
  btSerial.print(F(",rpmA="));
  btSerial.print(currentRPMA, 1);
  btSerial.print(F(",rpmB="));
  btSerial.println(currentRPMB, 1);
}

void handleLinkCommand(char *cmd) {
  if (strcmp(cmd, "PING") == 0) {
    btSerial.println(F("PONG"));
  } else if (strcmp(cmd, "STATUS") == 0) {
    sendLinkStatus();
  } else if (strcmp(cmd, "LAST") == 0) {
    sendSavedWallMeasurementsToLink();
  } else if (strcmp(cmd, "STOP") == 0) {
    currentState = STATE_FINISH;
    stopMotors();
    btSerial.println(F("OK,STOP"));
  } else if (strncmp(cmd, "ECHO,", 5) == 0) {
    btSerial.print(F("ECHO,"));
    btSerial.println(cmd + 5);
  } else if (cmd[0] != '\0') {
    btSerial.print(F("ERR,unknown_cmd="));
    btSerial.println(cmd);
  }
}

void pollLinkSerial() {
  while (btSerial.available()) {
    char c = (char)btSerial.read();
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      linkRxBuf[linkRxLen] = '\0';
      handleLinkCommand(linkRxBuf);
      linkRxLen = 0;
      continue;
    }

    if (linkRxLen < sizeof(linkRxBuf) - 1) {
      linkRxBuf[linkRxLen++] = c;
    } else {
      linkRxLen = 0;
      btSerial.println(F("ERR,line_too_long"));
    }
  }
}

// ==================== 7. GY-521 / MPU6050 helpers ====================
bool writeMPU(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

int16_t readMPUWord(byte reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  Wire.requestFrom(MPU_ADDR, (byte)2, (byte)true);
  if (Wire.available() < 2) {
    return 0;
  }

  int16_t high = Wire.read();
  int16_t low = Wire.read();
  return (high << 8) | low;
}

byte readWhoAmI() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  Wire.requestFrom(MPU_ADDR, (byte)1, (byte)true);
  if (Wire.available() < 1) {
    return 0xFF;
  }
  return Wire.read();
}

bool initMPU() {
  Wire.begin();
  Wire.setClock(400000UL);
  delay(100);

  byte who = readWhoAmI();
  Serial.print(F("MPU WHO_AM_I: 0x"));
  Serial.println(who, HEX);

  bool ok = writeMPU(0x6B, 0x00);  // Wake up.
  ok = writeMPU(0x1B, 0x00) && ok; // Gyro +/-250 dps.
  ok = writeMPU(0x1C, 0x00) && ok; // Accel +/-2g.
  ok = writeMPU(0x1A, 0x03) && ok; // DLPF about 44 Hz.
  return ok;
}

float readGyroZDPS() {
  return readMPUWord(0x47) / GYRO_Z_SENS;
}

void calibrateGyroZ(unsigned int samples) {
  stopMotors();
  delay(300);

  float sum = 0.0;
  for (unsigned int i = 0; i < samples; i++) {
    sum += readGyroZDPS();
    delay(3);
  }

  gyroZBiasDPS = sum / samples;
  Serial.print(F("Gyro Z bias: "));
  Serial.print(gyroZBiasDPS, 3);
  Serial.println(F(" dps"));
}

void resetYaw() {
  yawDeg = 0.0;
  gyroZRateDPS = 0.0;
  lastGyroMicros = micros();
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastGyroMicros) / 1000000.0;
  lastGyroMicros = now;

  gyroZRateDPS = readGyroZDPS() - gyroZBiasDPS;
  if (gyroZRateDPS > -0.15 && gyroZRateDPS < 0.15) {
    gyroZRateDPS = 0.0;
  }
  yawDeg += gyroZRateDPS * dt;
}

float turnByGyroPositionLoop(float targetDeg, int turnDir) {
  if (!mpuOk) {
    Serial.println(F("MPU not ready. Timed left-turn fallback."));
    spinInPlace(turnDir, 70);
    delay(850);
    stopMotors();
    delay(250);
    return targetDeg;
  }

  calibrateGyroZ(220);
  resetYaw();

  unsigned long startMs = millis();
  unsigned long settleStartMs = 0;

  Serial.println(F("Gyro position-loop turn start"));

  while (millis() - startMs < TURN_TIMEOUT_MS) {
    updateYaw();

    float angleAbs = absFloat(yawDeg);
    float error = targetDeg - angleAbs;
    float absError = absFloat(error);
    float rateAbs = absFloat(gyroZRateDPS);

    if (absError <= TURN_STOP_ERROR_DEG && rateAbs <= TURN_STOP_RATE_DPS) {
      if (settleStartMs == 0) {
        settleStartMs = millis();
      }
      if (millis() - settleStartMs >= TURN_SETTLE_MS) {
        break;
      }
    } else {
      settleStartMs = 0;
    }

    int driveDir = (error >= 0.0) ? turnDir : -turnDir;
    int minPwm = (absError <= TURN_NEAR_DEG) ? TURN_PWM_MIN_NEAR : TURN_PWM_MIN_FAR;
    int maxPwm = (absError <= TURN_NEAR_DEG) ? TURN_PWM_MAX_NEAR : TURN_PWM_MAX;

    float rawPwm = KP_ANGLE * absError - KD_RATE * rateAbs + minPwm;
    int pwm = constrain((int)rawPwm, minPwm, maxPwm);
    if (absError < 0.4) {
      pwm = 0;
    }

    if (pwm > 0) {
      spinInPlace(driveDir, pwm);
    } else {
      stopMotors();
    }

    delay(5);
  }

  stopMotors();
  delay(300);
  updateYaw();

  Serial.print(F("Turn final yaw abs: "));
  Serial.print(absFloat(yawDeg), 1);
  Serial.println(F(" deg"));
  return absFloat(yawDeg);
}

float alignToHeadingByGyro(float targetHeadingDeg) {
  if (!mpuOk) {
    Serial.println(F("MPU not ready. Skip heading alignment."));
    return yawDeg;
  }

  resetEncoders();
  unsigned long startMs = millis();
  unsigned long settleStartMs = 0;
  unsigned long lastControlMs = millis();
  long prevA = 0;
  long prevB = 0;
  float wheelIntegralA = 0.0;
  float wheelIntegralB = 0.0;
  float wheelLastErrA = 0.0;
  float wheelLastErrB = 0.0;

  Serial.print(F("Heading align start yaw="));
  Serial.print(yawDeg, 1);
  Serial.print(F(" target="));
  Serial.print(targetHeadingDeg, 1);
  Serial.println(F(" deg"));

  while (millis() - startMs < HEADING_TIMEOUT_MS) {
    updateYaw();

    unsigned long nowMs = millis();
    if (nowMs - lastControlMs < TURN_CONTROL_INTERVAL_MS) {
      delay(1);
      continue;
    }

    float dt = (nowMs - lastControlMs) / 1000.0;
    lastControlMs = nowMs;

    float angleError = targetHeadingDeg - yawDeg;
    float absError = absFloat(angleError);
    float rateAbs = absFloat(gyroZRateDPS);

    if (absError <= HEADING_STOP_ERROR_DEG && rateAbs <= HEADING_STOP_RATE_DPS) {
      if (settleStartMs == 0) {
        settleStartMs = millis();
      }
      if (millis() - settleStartMs >= HEADING_SETTLE_MS) {
        break;
      }
    } else {
      settleStartMs = 0;
    }

    int driveDir = (angleError >= 0.0) ? HEADING_ALIGN_DIR_SIGN : -HEADING_ALIGN_DIR_SIGN;
    float targetRPM = constrain(absError * KP_ANGLE_TO_WHEEL_RPM, TURN_RPM_MIN, TURN_RPM_MAX);
    if (absError < 0.4) {
      targetRPM = 0.0;
    }

    long a, b;
    readEncoders(a, b);
    float deltaA = abs(a - prevA);
    float deltaB = abs(b - prevB);
    prevA = a;
    prevB = b;

    float rpmA = (deltaA / ENCODER_COUNTS_PER_REV) / dt * 60.0;
    float rpmB = (deltaB / ENCODER_COUNTS_PER_REV) / dt * 60.0;

    float errA = targetRPM - rpmA;
    float errB = targetRPM - rpmB;
    wheelIntegralA = constrain(wheelIntegralA + errA * dt, -120.0, 120.0);
    wheelIntegralB = constrain(wheelIntegralB + errB * dt, -120.0, 120.0);

    float rawPwmA = HEADING_PWM_MIN
      + KP_TURN_WHEEL_SPEED * errA
      + KI_TURN_WHEEL_SPEED * wheelIntegralA
      + KD_TURN_WHEEL_SPEED * ((errA - wheelLastErrA) / dt);
    float rawPwmB = HEADING_PWM_MIN
      + KP_TURN_WHEEL_SPEED * errB
      + KI_TURN_WHEEL_SPEED * wheelIntegralB
      + KD_TURN_WHEEL_SPEED * ((errB - wheelLastErrB) / dt);
    wheelLastErrA = errA;
    wheelLastErrB = errB;

    int pwmA = targetRPM > 0.0 ? constrain((int)rawPwmA, HEADING_PWM_MIN, HEADING_PWM_MAX) : 0;
    int pwmB = targetRPM > 0.0 ? constrain((int)rawPwmB, HEADING_PWM_MIN, HEADING_PWM_MAX) : 0;

    if (targetRPM > 0.0) {
      spinInPlacePWM(driveDir, pwmA, pwmB);
    } else {
      stopMotors();
    }

    delay(2);
  }

  stopMotors();
  delay(250);
  updateYaw();

  Serial.print(F("Heading align final yaw="));
  Serial.print(yawDeg, 1);
  Serial.println(F(" deg"));
  return yawDeg;
}

float turnRelativeByGyroCascade(float targetDeg, int turnDir) {
  if (!mpuOk) {
    Serial.println(F("MPU not ready. Timed turn fallback."));
    spinInPlace(turnDir, 70);
    delay(850);
    stopMotors();
    delay(250);
    return targetDeg;
  }

  calibrateGyroZ(180);
  resetYaw();
  resetEncoders();

  unsigned long startMs = millis();
  unsigned long settleStartMs = 0;
  unsigned long lastControlMs = millis();
  long prevA = 0;
  long prevB = 0;
  float wheelIntegralA = 0.0;
  float wheelIntegralB = 0.0;
  float wheelLastErrA = 0.0;
  float wheelLastErrB = 0.0;

  Serial.print(F("Gyro cascade turn target="));
  Serial.print(targetDeg, 1);
  Serial.println(F(" deg"));

  while (millis() - startMs < TURN_TIMEOUT_MS) {
    updateYaw();

    unsigned long nowMs = millis();
    if (nowMs - lastControlMs < TURN_CONTROL_INTERVAL_MS) {
      delay(1);
      continue;
    }

    float dt = (nowMs - lastControlMs) / 1000.0;
    lastControlMs = nowMs;

    float angleAbs = absFloat(yawDeg);
    float angleError = targetDeg - angleAbs;
    float absError = absFloat(angleError);
    float rateAbs = absFloat(gyroZRateDPS);

    if (absError <= TURN_STOP_ERROR_DEG && rateAbs <= TURN_STOP_RATE_DPS) {
      if (settleStartMs == 0) {
        settleStartMs = millis();
      }
      if (millis() - settleStartMs >= TURN_SETTLE_MS) {
        break;
      }
    } else {
      settleStartMs = 0;
    }

    int driveDir = (angleError >= 0.0) ? turnDir : -turnDir;
    float targetRPM = constrain(absError * KP_ANGLE_TO_WHEEL_RPM, TURN_RPM_MIN, TURN_RPM_MAX);
    if (absError < 0.4) {
      targetRPM = 0.0;
    }

    long a, b;
    readEncoders(a, b);
    float deltaA = abs(a - prevA);
    float deltaB = abs(b - prevB);
    prevA = a;
    prevB = b;

    float rpmA = (deltaA / ENCODER_COUNTS_PER_REV) / dt * 60.0;
    float rpmB = (deltaB / ENCODER_COUNTS_PER_REV) / dt * 60.0;

    float errA = targetRPM - rpmA;
    float errB = targetRPM - rpmB;
    wheelIntegralA = constrain(wheelIntegralA + errA * dt, -120.0, 120.0);
    wheelIntegralB = constrain(wheelIntegralB + errB * dt, -120.0, 120.0);

    float rawPwmA = HEADING_PWM_MIN
      + KP_TURN_WHEEL_SPEED * errA
      + KI_TURN_WHEEL_SPEED * wheelIntegralA
      + KD_TURN_WHEEL_SPEED * ((errA - wheelLastErrA) / dt);
    float rawPwmB = HEADING_PWM_MIN
      + KP_TURN_WHEEL_SPEED * errB
      + KI_TURN_WHEEL_SPEED * wheelIntegralB
      + KD_TURN_WHEEL_SPEED * ((errB - wheelLastErrB) / dt);
    wheelLastErrA = errA;
    wheelLastErrB = errB;

    int pwmA = targetRPM > 0.0 ? constrain((int)rawPwmA, HEADING_PWM_MIN, HEADING_PWM_MAX) : 0;
    int pwmB = targetRPM > 0.0 ? constrain((int)rawPwmB, HEADING_PWM_MIN, HEADING_PWM_MAX) : 0;

    if (targetRPM > 0.0) {
      spinInPlacePWM(driveDir, pwmA, pwmB);
    } else {
      stopMotors();
    }

    delay(2);
  }

  stopMotors();
  delay(300);
  updateYaw();

  Serial.print(F("Gyro cascade turn final yaw abs="));
  Serial.print(absFloat(yawDeg), 1);
  Serial.println(F(" deg"));
  return absFloat(yawDeg);
}

// ==================== 8. Required second-cross sequence ====================
void runPostSecondCrossSequence() {
  currentState = STATE_POST_SECOND_CROSS;
  stopMotors();
  resetSpeedPid();

  Serial.println(F("Cross 2 reached. Stop 2s, then measure wall 1."));
  delay(2000);
  resetSpeedPid();

  stopMotors();
  delay(500);
  float wall1 = measureStableWallCMWithError(1, WALL1_DISTANCE_ERROR_CM);

  Serial.println(F("Turn right 90 deg with gyro position/speed loop."));
  turnRelativeByGyroCascade(LEFT_TURN_DEG, LEFT_TURN_DIR);

  reverseAlongLineUntilCross(5000);

  stopMotors();
  delay(500);
  float wall2 = measureStableWallCMWithError(2, WALL2_DISTANCE_ERROR_CM);

  Serial.print(F("Result wall1="));
  Serial.print(wall1, 1);
  Serial.print(F(" cm, wall2="));
  Serial.print(wall2, 1);
  Serial.println(F(" cm"));
  saveWallMeasurements(wall1, wall2);

  currentState = STATE_FINISH;
  Serial.println(F("Task finished."));
}

// ==================== 9. Main program ====================
void setup() {
  Serial.begin(DEBUG_BAUD);
  btSerial.begin(BT_BAUD);
  Serial.println(F("HC-SR04 + ZS-040 Bluetooth test logic integrated"));
  Serial.println(F("Serial Monitor: 9600 baud"));
  Serial.println(F("Bluetooth PC port: COM4, 9600 baud"));
  btSerial.println(F("HC-SR04 + ZS-040 Bluetooth test logic integrated"));
  btSerial.println(F("Bluetooth connected at 9600 baud"));
  printSavedWallMeasurements();

  pinMode(MAI1, OUTPUT); pinMode(MAI2, OUTPUT);
  pinMode(MBI1, OUTPUT); pinMode(MBI2, OUTPUT);
  pinMode(MAEB, INPUT);  pinMode(MBEB, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(MAEB), countEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(MBEB), countEncoderB, RISING);

  mpuOk = initMPU();
  if (mpuOk) {
    Serial.println(F("MPU6050 ready."));
    calibrateGyroZ(180);
  } else {
    Serial.println(F("MPU6050 init failed. Check SDA=A4, SCL=A5."));
  }

  programStartTime = millis();
  lastTime = millis();

  Serial.println(F("Line car ready. Wait 3s, start from cross 1, then trigger post sequence at cross 2."));
  Serial.println(F("TEL,rpmA,rpmB,targetA,targetB,Kp,Ki,Kd,pA,iA,dA,pB,iB,dB"));
}

void loop() {
  pollLinkSerial();

  if (currentState == STATE_FINISH) {
    stopMotors();
    return;
  }

  if (currentState == STATE_POST_SECOND_CROSS) {
    return;
  }

  LineRead line = readLineSensors();

  if (currentState == STATE_WAIT) {
    stopMotors();
    if (millis() - programStartTime >= START_DELAY_MS) {
      currentState = STATE_FINDING_FIRST_CROSS;
      crossCount = 0;
      crossDetected = false;
      crossStableStartTime = 0;
      crossExitSampleCount = 0;
      resetSpeedPid();
      resetEncoders();
      lastTime = millis();
      Serial.println(F("Start running. Searching cross 1."));
    }
    return;
  }

  if (currentState == STATE_FINDING_FIRST_CROSS && line.cross) {
    currentState = STATE_LEAVING_START_CROSS;
    crossCount = 1;
    crossStableStartTime = 0;
    crossExitSampleCount = 0;
    Serial.println(F("Cross 1 detected. Do not stop; leave it first."));
  }

  if (currentState == STATE_LEAVING_START_CROSS) {
    if (line.sum <= CROSS_EXIT_SENSOR_MAX) {
      crossExitSampleCount++;
      if (crossExitSampleCount >= CROSS_EXIT_SAMPLE_REQUIRED) {
        currentState = STATE_RUNNING;
        crossStableStartTime = 0;
        crossExitSampleCount = 0;
        Serial.println(F("Left cross 1. Searching cross 2."));
        lastTime = millis();
        return;
      }
    } else {
      crossExitSampleCount = 0;
    }
  }

  if (mpuOk) {
    updateYaw();
  }

  if (currentState == STATE_RUNNING) {
    if (line.cross) {
      stopMotors();
      crossCount = 2;
      Serial.println(F("Cross 2 detected. Start post-second-cross sequence."));
      runPostSecondCrossSequence();
      lastTime = millis();
      return;
    } else {
      crossStableStartTime = 0;
    }
  }

  if (millis() - lastTime >= SAMPLE_TIME_MS) {
    if (currentState == STATE_FINDING_FIRST_CROSS || currentState == STATE_RUNNING || currentState == STATE_LEAVING_START_CROSS) {
      float turnComp = Kp_line * line.error + Kd_line * (line.error - lastLineError);
      lastLineError = line.error;
      targetRPMA = constrain(BASE_RPM + turnComp, 0.0, MAX_TARGET_RPM);
      targetRPMB = constrain(BASE_RPM - turnComp, 0.0, MAX_TARGET_RPM);
    }

    long deltaA, deltaB;
    readAndResetEncoders(deltaA, deltaB);

    currentRPMA = (deltaA / ENCODER_COUNTS_PER_REV) / (SAMPLE_TIME_MS / 1000.0) * 60.0;
    currentRPMB = (deltaB / ENCODER_COUNTS_PER_REV) / (SAMPLE_TIME_MS / 1000.0) * 60.0;

    if (currentState == STATE_FINDING_FIRST_CROSS || currentState == STATE_RUNNING || currentState == STATE_LEAVING_START_CROSS) {
      float errA = targetRPMA - currentRPMA;
      integralA = constrain(integralA + errA, -180.0, 180.0);
      float pA = Kp_spd * errA;
      float iA = Ki_spd * integralA;
      float dA = Kd_spd * (errA - lastErrA);
      float outA = pA + iA + dA;
      lastErrA = errA;

      float errB = targetRPMB - currentRPMB;
      integralB = constrain(integralB + errB, -180.0, 180.0);
      float pB = Kp_spd * errB;
      float iB = Ki_spd * integralB;
      float dB = Kd_spd * (errB - lastErrB);
      float outB = pB + iB + dB;
      lastErrB = errB;

      if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        Serial.print(F("TEL,"));
        Serial.print(currentRPMA, 1); Serial.print(F(","));
        Serial.print(currentRPMB, 1); Serial.print(F(","));
        Serial.print(targetRPMA, 1); Serial.print(F(","));
        Serial.print(targetRPMB, 1); Serial.print(F(","));
        Serial.print(Kp_spd, 3); Serial.print(F(","));
        Serial.print(Ki_spd, 3); Serial.print(F(","));
        Serial.print(Kd_spd, 3); Serial.print(F(","));
        Serial.print(pA, 1); Serial.print(F(","));
        Serial.print(iA, 1); Serial.print(F(","));
        Serial.print(dA, 1); Serial.print(F(","));
        Serial.print(pB, 1); Serial.print(F(","));
        Serial.print(iB, 1); Serial.print(F(","));
        Serial.println(dB, 1);
        lastTelemetryMs = millis();
      }

      motorControl(MAI1, MAI2, outA);
      motorControl(MBI1, MBI2, outB);
    } else {
      stopMotors();
    }

    lastTime = millis();
  }
}
