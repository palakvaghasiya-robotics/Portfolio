#include "ps.h"
#include "drive.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define M1_PWM 24
#define M1_DIR 30
#define M2_PWM 25
#define M2_DIR 31
#define M3_PWM 28
#define M3_DIR 32
#define M4_PWM 29
#define M4_DIR 33
#define MOTOR1_PWM 10
#define MOTOR1_DIR 36
#define MOTOR2_PWM 22
#define MOTOR2_DIR 35
#define Conveyer_pwm 22
#define Conveyer_dir 35
#define flywheelPwmPin 10
#define flywheelDirPin 36
#define elevator_dir 23
#define elevator_pwm 34
#define ledPin 2
enum ControlGroup {
  GROUP_MANUAL,
  GROUP_AUTO
};
volatile bool limitSwitchTriggered = false;
ControlGroup activeGroup = GROUP_MANUAL;
bool groupSwitchRequested = false;
ControlGroup requestedGroup = GROUP_MANUAL;

PS ps;
Drive drive(24, 30, 25, 31, 28, 32, 29, 33);
const int encoderXPinA = 14;
const int encoderXPinB = 15;
const int encoderYPinA = 2;
const int encoderYPinB = 3;

const int motor1EncoderPinA = 8;  
const int motor1EncoderPinB = 9;  

volatile long pulseCountX = 0;
volatile long pulseCountY = 0;
volatile long motor1PulseCount = 0;  

float distanceX = 0.0;
float distanceY = 0.0;

const float wheelCircumference = 18.2212;
const float distancePerPulse = wheelCircumference / 555.0;

const int MOTOR1_PPR = 600;  
float motor1TargetRPM = 0.0;
float motor1CurrentRPM = 0.0;
int motor1PWMOutput = 0;
unsigned long lastRPMCalculationTime = 0;
long lastMotor1PulseCount = 0;
const unsigned long RPM_CALCULATION_INTERVAL = 100; 
float flywheelCurrentRPM = 0.0;
unsigned long lastFlywheelRPMCalculationTime = 0;
long lastFlywheelPulseCount = 0;
bool flywheelRunning = false;
const unsigned long FLYWHEEL_RPM_CALCULATION_INTERVAL = 100; // Same as motor1
const int FLYWHEEL_PPR = 600;

float motor1_Kp = 1.0;   
float motor1_Ki = 0.5;   
float motor1_Kd = 0.1;  

float motor1_errorSum = 0.0;
float motor1_lastError = 0.0;
unsigned long motor1_lastTime = 0;

const int MAX_MOTOR_SPEED = 120;
const int MIN_MOTOR_SPEED = 60;

const float ACCEL_RATE = 2.0;
const float DECEL_RATE = 1.5;
const float DECEL_DISTANCE = 150.0;
const float RAMP_UP_DISTANCE = 100.0;

float Kp_x = 0.1; 
float Ki_x = 0.003; 
float Kd_x = 0.02;

float Kp_y = 0.4; 
float Ki_y = 0.0; 
float Kd_y = 0.01;

const float ANGLE_TOLERANCE = 4.0;
const float ANGLE_SLOW_ZONE = 10.0;
const int TURN_BASE_SPEED = 50;
const int TURN_MAX_SPEED = 60;

int currentButtonLeftState;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

enum RobotState {
  IDLE,
  MOTOR1_RUNNING,
  MOVING_Y_AXIS,
  POST_Y_DELAY, 
  MOVING_X_AXIS,
  POST_MOVEMENT_DELAY,
  TURNING_TO_ANGLE,
  RELAY_TOGGLING,
  STOPPING
};
struct Target {
  float x;
  float y;
  float rpm1;     
  float angle;
  int pwm2;
};

Target target1 = {-413, 149, 0, 0, 0};
Target target2 = {300, 0, 0, 0, 0};  
Target target3 = {0, 300, 0, 0, 0}; 
Target target4 = {0, 0, 0, 0, 0};
Target target5 = {0, 0, 0, 0, 0};
Target target6 = {0, 0, 0, 0, 0};  
Target target7 = {0, 0, 0, 0, 0}; 
Target target8 = {0, 0, 0, 0, 0};

RobotState currentState = IDLE;
Target currentTarget;
unsigned long stateStartTime = 0;
unsigned long lastEncoderTime = 0;
float currentX = 0, currentY = 0;
float targetX = 0, targetY = 0;
float intermediateX = 0; 
bool motor1Running = false;
float targetAngle = 0;

enum MovementState { IDLE_MOVE, MOVING_X, MOVING_Y };
MovementState movementState = IDLE_MOVE;
float baselineY = 0.0;
float baselineX = 0.0;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 10;
bool movementStarted = false;

float errorSumX = 0.0;
float lastErrorX = 0.0;
unsigned long lastTimeX = 0;

float errorSumY = 0.0;
float lastErrorY = 0.0;
unsigned long lastTimeY = 0;

float currentSpeedX = 0.0;
float currentSpeedY = 0.0;
float targetSpeedX = 0.0;
float targetSpeedY = 0.0;
float initialYaw = 0;
float currentYaw = 0;
float adjustedYaw = 0;
bool yawCalibrated = false;

long startEncoder1 = 0, startEncoder2 = 0, startEncoder3 = 0, startEncoder4 = 0;
float targetDistance = 0;
// bool movementStarted = false;

volatile long prevEncoder1 = 0, prevEncoder2 = 0, prevEncoder3 = 0, prevEncoder4 = 0, motor1EncoderCount = 0;
//flywheel
bool lastR1State = false;
bool lastButtonRightState = false;
//conveyer
const int relay1Pin = 4;
bool relay1State = false;
bool prevRelay1Pulse = false;

const int relay2Pin = 1;
bool relay2State = false;
bool prevRelay2Pulse = false;

const int relay3Pin = 5;
bool relay3State = false;
bool prevRelay3Pulse = false;
const int R1 = 9;
const int R2 = 17;
int pwm = 0;


void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  delay(1000);
  
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(flywheelDirPin, OUTPUT);
  pinMode(flywheelPwmPin, OUTPUT);
  pinMode(encoderXPinA, INPUT_PULLUP);
  pinMode(encoderXPinB, INPUT_PULLUP);
  pinMode(encoderYPinA, INPUT_PULLUP);
  pinMode(encoderYPinB, INPUT_PULLUP);
  pinMode(motor1EncoderPinA, INPUT_PULLUP);
  pinMode(motor1EncoderPinB, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
digitalWrite(ledPin, HIGH);  
  pinMode(relay1Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  pinMode(relay2Pin, OUTPUT);
  digitalWrite(relay2Pin, HIGH);
  pinMode(relay3Pin, OUTPUT);
  digitalWrite(relay3Pin, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderXPinA), countPulseX, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderYPinA), countPulseY, RISING);
  attachInterrupt(digitalPinToInterrupt(motor1EncoderPinA), countMotor1Pulse, RISING);

  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  Wire1.begin();
  Wire1.setClock(100000);
  
  bool imuFound = false;
  
  if (bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1)) {
    imuFound = true;
  } else {
    if (bno08x.begin_I2C(0x4B, &Wire1)) {
      imuFound = true;
    }
  }
  
  if (imuFound) {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      yawCalibrated = false;
    } else {
      stopAllMotors();
      delay(1000);
      
      calibrateInitialYaw();
      
      if (!yawCalibrated) {
        delay(2000);
        calibrateInitialYaw();
      }
    }
  } else {
    yawCalibrated = false;
  }
}
void loop() {
  ps.update();
  if (motor1Running) {
    updateMotor1RPM(); 
  }  
  if (flywheelRunning) {
    updateFlywheelRPM();
  }
  checkGroupSwitchRequests();  
  if (groupSwitchRequested) {
    switchControlGroup();
  }
  if (activeGroup == GROUP_MANUAL) {
    executeManualGroup();
  } else if (activeGroup == GROUP_AUTO) {
    executeAutoGroup();
  }  
  updatePosition();
}
void checkGroupSwitchRequests() {
  if (ps.dpadUpCount >= 1 || ps.dpadRightCount >= 1 || ps.dpadLeftCount >= 1 || ps.dpadDownCount >= 1 || ps.buttonLeft >= 1) {
    if (activeGroup != GROUP_AUTO) {
      requestedGroup = GROUP_AUTO;
      groupSwitchRequested = true;
      return;
    }
  }
  bool manualControlActive = (abs(ps.axisY) > 50 || abs(ps.axisRX) > 50 || 
                             ps.brake || ps.throttle || 
                             ps.buttonUp || ps.buttonDown || 
                             ps.r1);
  if (manualControlActive && activeGroup != GROUP_MANUAL) {
    requestedGroup = GROUP_MANUAL;
    groupSwitchRequested = true;
  }
}
void switchControlGroup() {
  if (requestedGroup != activeGroup) {
    if (activeGroup == GROUP_AUTO) {
      stopAutoGroup();
    } else {
      stopManualGroup();
    }
    
    activeGroup = requestedGroup;
    groupSwitchRequested = false;
    
    delay(100);
  }
}
void stopAutoGroup() {
  currentState = STOPPING;
  stopAllMotors();
  if (motor1Running) {
    stopMotor1();
  }
  currentState = IDLE;
}
void stopManualGroup() {
  drive.move(0, 0, false, false);
  stopDriveMotors();
  
  analogWrite(flywheelPwmPin, 0);
  pwm = 0;
}
void executeManualGroup() {
  drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);
  flywheel();
  conveyer();
  dribbler2();
  dribbler();
}
void executeAutoGroup() {
  checkSerialCommands();
  stateMachine();
}
void checkSerialCommands() {
  if (ps.dpadUpCount == 1) {
    executeTarget(target1);
  } else if (ps.dpadUpCount == 2) {
    executeTarget(target2);
  }
  if (ps.dpadRightCount == 1) {
    executeTarget(target3);
  } else if (ps.dpadRightCount == 2) {
    executeTarget(target4);
  }
  if (ps.dpadLeftCount == 1) {
    executeTarget(target5);
  } else if (ps.dpadLeftCount == 2) {
    executeTarget(target6);
  }
  if (ps.dpadDownCount == 1) {
    executeTarget(target7);
  } else if (ps.dpadDownCount == 2) {
    executeTarget(target8);
  }
}

void executeTarget(Target target) {
  if (motor1Running) stopMotor1();
  stopAllMotors();
  delay(500);
  
  currentTarget = target;
  targetX = target.x;
  targetY = target.y;
  targetAngle = target.angle;
  
  movementStarted = false;
  currentX = 0;
  currentY = 0;
   movementState = MOVING_Y; 
  
  currentState = MOVING_Y_AXIS;
}

void stateMachine() {
  static unsigned long relayToggleTime = 0;
  static int toggleCount = 0;
  switch (currentState) {
    case IDLE:
      break;
      
    case MOTOR1_RUNNING:
      movementStarted = false;
      currentState = MOVING_Y_AXIS;
      stateStartTime = millis();
      startMovementToTarget();
      break;
      
    case MOVING_Y_AXIS:
      updateMovement();
      if (movementState == IDLE_MOVE) {
        currentState = POST_Y_DELAY;
        stateStartTime = millis();
      }
      break;

    case POST_Y_DELAY:
      if (millis() - stateStartTime >= 500) {
        startMovementToTarget();  // Reset for X movement
        currentState = MOVING_X_AXIS;
      }
      break;
      
    case MOVING_X_AXIS:
      updateMovement();
      if (movementState == IDLE_MOVE) {
        currentState = IDLE;  // Go directly to IDLE after X movement
        stopAllMotors();
      }
      break;
      
    case POST_MOVEMENT_DELAY:
      if (millis() - stateStartTime >= 500) {
        if (yawCalibrated) {
          currentState = TURNING_TO_ANGLE;
          stateStartTime = millis();
        } else {
          currentState = RELAY_TOGGLING ;
          stateStartTime = millis();
        }
      }
      break;
      
    case TURNING_TO_ANGLE:
      if (millis() - stateStartTime > 15000) {
        stopDriveMotors();
        currentState = IDLE;
        stateStartTime = millis();
      } else if (turnToAngle(targetAngle)) {
        currentState = IDLE;
        stateStartTime = millis();
      }
      break;
      
    case STOPPING:
      stopAllMotors();
      currentState = IDLE;
      break;
  }
}
void countMotor1Pulse() {
  motor1PulseCount += (digitalRead(motor1EncoderPinB) == HIGH) ? 1 : -1;
}

void startMotor1RPM(float targetRPM) {
  motor1TargetRPM = targetRPM;
  motor1Running = (targetRPM > 0);
  
  motor1_errorSum = 0.0;
  motor1_lastError = 0.0;
  motor1_lastTime = millis();
  
  noInterrupts();
  motor1PulseCount = 0;
  interrupts();
  lastMotor1PulseCount = 0;
  lastRPMCalculationTime = millis();
  
  if (motor1Running) {
    digitalWrite(MOTOR1_DIR, HIGH);
    motor1PWMOutput = 100; 
    analogWrite(MOTOR1_PWM, motor1PWMOutput);
  }
}
void updateMotor1RPM() {
  if (!motor1Running || motor1TargetRPM <= 0) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastRPMCalculationTime >= RPM_CALCULATION_INTERVAL) {
    noInterrupts();
    long currentPulseCount = motor1PulseCount;
    interrupts();
    
    long pulseDiff = currentPulseCount - lastMotor1PulseCount;
    float timeInterval = (currentTime - lastRPMCalculationTime) / 1000.0; 
    
    motor1CurrentRPM = (pulseDiff / timeInterval) * 60.0 / MOTOR1_PPR;
    motor1CurrentRPM = abs(motor1CurrentRPM); 
    
    lastMotor1PulseCount = currentPulseCount;
    lastRPMCalculationTime = currentTime;
    
    float rpmError = motor1TargetRPM - motor1CurrentRPM;
    float pidOutput = calculateMotor1PID(rpmError);
    
    motor1PWMOutput += pidOutput;
    motor1PWMOutput = constrain(motor1PWMOutput, 0, 255);
    
    analogWrite(MOTOR1_PWM, motor1PWMOutput);
    
    Serial.print("Target RPM: ");
    Serial.print(motor1TargetRPM);
    Serial.print(", Current RPM: ");
    Serial.print(motor1CurrentRPM);
    Serial.print(", PWM: ");
    Serial.println(motor1PWMOutput);
  }
}

float calculateMotor1PID(float error) {
  unsigned long now = millis();
  float dt = (now - motor1_lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;
  
  motor1_errorSum += error * dt;
  
  motor1_errorSum = constrain(motor1_errorSum, -100, 100);
  
  float dError = (error - motor1_lastError) / dt;
  
  float output = motor1_Kp * error + motor1_Ki * motor1_errorSum + motor1_Kd * dError;
  
  motor1_lastError = error;
  motor1_lastTime = now;
  
  return output;
}

void startMovementToTarget() {
  errorSumX = 0.0;
  lastErrorX = 0.0;
  lastTimeX = millis();
  
  errorSumY = 0.0;
  lastErrorY = 0.0;
  lastTimeY = millis();

  noInterrupts();
  pulseCountX = 0;
  pulseCountY = 0;
  interrupts();
  distanceX = 0.0;
  distanceY = 0.0;

  currentSpeedX = 0.0;  
  currentSpeedY = 0.0;

   // Start with Y-axis movement first
  movementState = (currentState == MOVING_Y_AXIS) ? MOVING_Y : MOVING_X;

  noInterrupts();
  baselineX = pulseCountX * distancePerPulse;  // Reset baseline for X correction
  baselineY = pulseCountY * distancePerPulse;  // Reset baseline for Y correction
  interrupts();
  lastUpdateTime = millis();
}

void updateMovement() {
  if (movementState == IDLE_MOVE) return;  
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastUpdateTime;
  if (elapsedTime < updateInterval)
    return;  
  lastUpdateTime = currentTime;

  if (movementState == MOVING_X) {
    noInterrupts();
    long pulsesX = pulseCountX;
    long pulsesY = pulseCountY;
    interrupts();
    
    distanceX = pulsesX * distancePerPulse;
    float currentY = pulsesY * distancePerPulse;
    float errorY = currentY - baselineY;
    
    float remainingDistance = abs(targetX) - abs(distanceX);
    float traveledDistance = abs(distanceX);

    bool isDecelPhase = remainingDistance <= DECEL_DISTANCE;

    targetSpeedX = calculateTargetSpeed(remainingDistance, traveledDistance, MAX_MOTOR_SPEED, isDecelPhase);

    updateSpeed(currentSpeedX, targetSpeedX, elapsedTime);

    float desired_vx = (targetX >= 0) ? currentSpeedX : -currentSpeedX;

    float correction_vy = -calculatePID_Y(errorY);

    float wheel1 = desired_vx + correction_vy;  
    float wheel2 = -desired_vx + correction_vy;  
    float wheel3 = desired_vx + correction_vy;  
    float wheel4 = -desired_vx + correction_vy;  

    wheel1 = constrain(wheel1, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wheel2 = constrain(wheel2, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wheel3 = constrain(wheel3, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wheel4 = constrain(wheel4, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    setMotor(M1_PWM, M1_DIR, abs(wheel1), wheel1 >= 0);
    setMotor(M2_PWM, M2_DIR, abs(wheel2), wheel2 >= 0);
    setMotor(M3_PWM, M3_DIR, abs(wheel3), wheel3 >= 0);
    setMotor(M4_PWM, M4_DIR, abs(wheel4), wheel4 >= 0);

    if (abs(distanceX) >= abs(targetX)) {
      stopDriveMotors();
      delay(100); 
      movementState = IDLE_MOVE;

      currentSpeedX = 0.0;
      currentSpeedY = 0.0;

      noInterrupts();
      baselineX = pulseCountX * distancePerPulse;
      interrupts();

      errorSumX = 0.0;
      lastErrorX = 0.0;
      lastTimeX = millis();
    }
  }
  else if (movementState == MOVING_Y) {
    noInterrupts();
    long pulsesY = pulseCountY;
    long pulsesX = pulseCountX;
    interrupts();
    
    distanceY = pulsesY * distancePerPulse;
    float currentX = pulsesX * distancePerPulse;
    float errorX = currentX - baselineX;

    float remainingDistance = abs(targetY) - abs(distanceY);
    float traveledDistance = abs(distanceY);

    bool isDecelPhase = remainingDistance <= DECEL_DISTANCE;

    targetSpeedY = calculateTargetSpeed(remainingDistance, traveledDistance, MAX_MOTOR_SPEED, isDecelPhase);

    updateSpeed(currentSpeedY, targetSpeedY, elapsedTime);

    float desired_vy = (targetY >= 0) ? currentSpeedY : -currentSpeedY;

    float correction_vx = -calculatePID_X(errorX);
    
    float wheel1 = desired_vy + correction_vx;
    float wheel2 = desired_vy - correction_vx;
    float wheel3 = desired_vy + correction_vx;
    float wheel4 = desired_vy - correction_vx;

    wheel1 = constrain(wheel1, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wheel2 = constrain(wheel2, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wheel3 = constrain(wheel3, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    wheel4 = constrain(wheel4, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    setMotor(M1_PWM, M1_DIR, abs(wheel1), wheel1 >= 0);
    setMotor(M2_PWM, M2_DIR, abs(wheel2), wheel2 >= 0);
    setMotor(M3_PWM, M3_DIR, abs(wheel3), wheel3 >= 0);
    setMotor(M4_PWM, M4_DIR, abs(wheel4), wheel4 >= 0);

    if (abs(distanceY) >= abs(targetY)) {
      stopDriveMotors();
      movementState = IDLE_MOVE;
      delay(100); 
    }
  }
}

void updateSpeed(float &currentSpeed, float targetSpeed, unsigned long elapsedTime) {
  float speedChange = elapsedTime * (targetSpeed > currentSpeed ? ACCEL_RATE : DECEL_RATE);
  
  if (targetSpeed > currentSpeed) {
    currentSpeed = min(targetSpeed, currentSpeed + speedChange);
  } else {
    currentSpeed = max(targetSpeed, currentSpeed - speedChange);
  }
  
  if (currentSpeed > 0 && currentSpeed < MIN_MOTOR_SPEED) {
    currentSpeed = MIN_MOTOR_SPEED;
  }
}

float calculateTargetSpeed(float remainingDistance, float traveledDistance, float maxSpeed, bool isDecelPhase) {
  if (isDecelPhase) {
    float decelRatio = remainingDistance / DECEL_DISTANCE;
    return max(MIN_MOTOR_SPEED, decelRatio * maxSpeed);
  } 
  else if (traveledDistance < RAMP_UP_DISTANCE) {
    float accelRatio = traveledDistance / RAMP_UP_DISTANCE;
    float rampedSpeed = MIN_MOTOR_SPEED + (accelRatio * (maxSpeed - MIN_MOTOR_SPEED));
    return min(maxSpeed, rampedSpeed);
  }
  else {
    return maxSpeed;
  }
}

float calculatePID_X(float error) {
  unsigned long now = millis();
  float dt = (now - lastTimeX) / 1000.0;
  if (dt <= 0) dt = 0.001;  
  
  errorSumX += error * dt;
  float dError = (error - lastErrorX) / dt;
  
  float output = Kp_x * error + Ki_x * errorSumX + Kd_x * dError;
  lastErrorX = error;
  lastTimeX = now;
  return output;
}

float calculatePID_Y(float error) {
  unsigned long now = millis();
  float dt = (now - lastTimeY) / 1000.0;
  if (dt <= 0) dt = 0.001;  
  
  errorSumY += error * dt;
  float dError = (error - lastErrorY) / dt;
  
  float output = Kp_y * error + Ki_y * errorSumY + Kd_y * dError;
  lastErrorY = error;
  lastTimeY = now;
  return output;
}

void countPulseX() {
  pulseCountX += (digitalRead(encoderXPinB) == HIGH) ? 1 : -1;
}

void countPulseY() {
  pulseCountY += (digitalRead(encoderYPinB) == HIGH) ? 1 : -1;
}

bool turnToAngle(float targetAngle) {
  if (!yawCalibrated) {
    return true;
  }
  
  if (readIMU()) {
    float error = targetAngle - adjustedYaw;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    float absError = abs(error);
    
    if (absError <= ANGLE_TOLERANCE) {
      stopDriveMotors();
      return true;
    }
    
    int speed = TURN_BASE_SPEED;
    if (absError < ANGLE_SLOW_ZONE) {
      speed = map(absError, ANGLE_TOLERANCE, ANGLE_SLOW_ZONE, TURN_BASE_SPEED/2, TURN_BASE_SPEED);
    } else {
      speed = TURN_MAX_SPEED;
    }
    
    if (error > 0) {
      rotateCounterClockwise(speed);
    } else {
      rotateClockwise(speed);
    }
    
    return false;
  }
  
  return false; 
}

void maintainAngle(float targetAngle) {
  if (!yawCalibrated) return;
  
  if (readIMU()) {
    float error = targetAngle - adjustedYaw;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    float absError = abs(error);
    
    if (absError <= ANGLE_TOLERANCE) {
      stopDriveMotors();
      return;
    }
    
    int speed = TURN_BASE_SPEED / 2; 
    if (absError < ANGLE_SLOW_ZONE) {
      speed = map(absError, ANGLE_TOLERANCE, ANGLE_SLOW_ZONE, TURN_BASE_SPEED/4, TURN_BASE_SPEED/2);
    }
    
    if (error > 0) {
      rotateCounterClockwise(speed);
    } else {
      rotateClockwise(speed);
    }
  }
}

void calibrateInitialYaw() {
  yawCalibrated = false;
  
  float sumYaw = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 50; i++) {
    if (bno08x.wasReset()) {
      if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        delay(100);
        continue;
      }
      delay(100);
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float qr = sensorValue.un.rotationVector.real;
        float qi = sensorValue.un.rotationVector.i;
        float qj = sensorValue.un.rotationVector.j;
        float qk = sensorValue.un.rotationVector.k;
        
        float yaw = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk));
        yaw = yaw * 180.0 / PI;
        
        sumYaw += yaw;
        validReadings++;
      }
    }
    delay(20);
  }
  
  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    yawCalibrated = true;
  } else {
    yawCalibrated = false;
  }
}

bool readIMU() {
  if (!yawCalibrated) return false;
  
  if (bno08x.wasReset()) {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      return false;
    }
    delay(50);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qr = sensorValue.un.rotationVector.real;
      float qi = sensorValue.un.rotationVector.i;
      float qj = sensorValue.un.rotationVector.j;
      float qk = sensorValue.un.rotationVector.k;
      
      currentYaw = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk));
      currentYaw = currentYaw * 180.0 / PI;
      
      adjustedYaw = currentYaw - initialYaw;
      
      if (adjustedYaw > 180) adjustedYaw -= 360;
      if (adjustedYaw < -180) adjustedYaw += 360;
      
      return true;
    }
  }
  
  return false;
}

void rotateClockwise(int speed) {
  digitalWrite(M1_DIR, HIGH);  
  analogWrite(M1_PWM, speed);
  
  digitalWrite(M2_DIR, LOW);  
  analogWrite(M2_PWM, speed);
  
  digitalWrite(M3_DIR, LOW); 
  analogWrite(M3_PWM, speed);
  
  digitalWrite(M4_DIR, HIGH);  
  analogWrite(M4_PWM, speed);
}

void rotateCounterClockwise(int speed) {
  digitalWrite(M1_DIR, LOW);   
  analogWrite(M1_PWM, speed);
  
  digitalWrite(M2_DIR, HIGH);  
  analogWrite(M2_PWM, speed);
  
  digitalWrite(M3_DIR, HIGH);  
  analogWrite(M3_PWM, speed);
  
  digitalWrite(M4_DIR, LOW);  
  analogWrite(M4_PWM, speed);
}

void setMotor(int pwmPin, int dirPin, int speed, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, speed);
}

void stopDriveMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}

void stopAllMotors() {
  stopDriveMotors();
  stopMotor1();
}

void stopMotor1() {
  analogWrite(MOTOR1_PWM, 0);
  motor1Running = false;
  motor1TargetRPM = 0.0;
  motor1CurrentRPM = 0.0;
  motor1PWMOutput = 0;
}

void updatePosition() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) { 
    lastPrint = millis();
  }
}
void flywheel() {
  if (ps.r1 && !lastR1State) {
    digitalWrite(flywheelDirPin, HIGH);
    if (pwm == 0) {
      pwm = 100;
    } else {
      pwm += 10;
    }
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheelPwmPin, pwm);
    
    flywheelRunning = (pwm > 0);
    
    if (pwm == 100) { 
      noInterrupts();
      motor1PulseCount = 0; // Fixed: Use motor1PulseCount instead of motor1EncoderCount
      interrupts();
      lastFlywheelPulseCount = 0;
      lastFlywheelRPMCalculationTime = millis();
    }
  }

  if (ps.buttonRight && !lastButtonRightState) {
    digitalWrite(flywheelDirPin, HIGH);
    pwm -= 20;
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheelPwmPin, pwm);
    
    flywheelRunning = (pwm > 0);
    
    if (pwm == 0) {
      flywheelRunning = false;
      flywheelCurrentRPM = 0.0;
    }
  }

  if (pwm == 240) {
    digitalWrite(ledPin, LOW);
    Serial.println("LOW");
  } else {
    digitalWrite(ledPin, HIGH);
  }

  // Removed the duplicate RPM printing from here - it's handled in updateFlywheelRPM()

  lastR1State = ps.r1;
  lastButtonRightState = ps.buttonRight;
}

void updateFlywheelRPM() {
  if (!flywheelRunning || pwm <= 0) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastFlywheelRPMCalculationTime >= FLYWHEEL_RPM_CALCULATION_INTERVAL) {
    noInterrupts();
    long currentPulseCount = motor1PulseCount; 
    interrupts();
    
    long pulseDiff = currentPulseCount - lastFlywheelPulseCount;
    float timeInterval = (currentTime - lastFlywheelRPMCalculationTime) / 1000.0; 
    
    flywheelCurrentRPM = (pulseDiff / timeInterval) * 60.0 / FLYWHEEL_PPR;
    flywheelCurrentRPM = abs(flywheelCurrentRPM); 
    
    lastFlywheelPulseCount = currentPulseCount;
    lastFlywheelRPMCalculationTime = currentTime;
    
    // This should now print correctly
    Serial.print("Flywheel - PWM: ");
    Serial.print(pwm);
    Serial.print(", RPM: ");
    Serial.println(flywheelCurrentRPM);
  }
}
void conveyer() {
  bool relay1Pressed = (ps.buttonUp);

  if (relay1Pressed && !prevRelay1Pulse) {
    relay1State = !relay1State;
    digitalWrite(relay1Pin, relay1State ? HIGH : LOW);
    Serial.println(relay1State ? "Relay 1 ON" : "Relay 1 OFF");
  }

  prevRelay1Pulse = relay1Pressed;
}

void dribbler() {
  bool relay2Pressed = (ps.buttonLeft == 1);

  if (relay2Pressed && !prevRelay2Pulse) {
    relay2State = !relay2State;
    digitalWrite(relay2Pin, relay2State ? HIGH : LOW);
    Serial.println(relay2State ? "Relay 1 ON" : "Relay 1 OFF");
  }

  prevRelay2Pulse = relay2Pressed;
}


void dribbler2() {
  bool relay3Pressed = (ps.buttonLeft == 2);

  if (relay3Pressed && !prevRelay3Pulse) {
    relay3State = !relay3State;
    digitalWrite(relay3Pin, relay3State ? HIGH : LOW);
    Serial.println(relay3State ? "Relay 1 ON" : "Relay 1 OFF");
  }

  prevRelay3Pulse = relay3Pressed;
}