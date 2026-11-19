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
#define elevator_pwm 23
#define elevator_dir 34
#define ledPin 2
#define RPM_TOLERANCE 50

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
const int encoder1PinA = 0;
const int encoder1PinB = 1;
const int encoder2PinA = 18;
const int encoder2PinB = 19;
const int encoder3PinA = 4;
const int encoder3PinB = 5; 
const int encoder4PinA = 6;
const int encoder4PinB = 7;

const int motor1EncoderPinA = 8;
const int motor1EncoderPinB = 9;
const int PPR = 9.3*19;
const float WHEEL_DIAMETER = 100.0; 
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE / (PPR * 10.0); 
const float BASE_SPEED = 80; 
const float KPX = 1.0; 
const float KPY = 1.1; 

const float MAX_CORRECTION = 30; 

const int MOTOR1_PPR = 600;
const float MOTOR1_KP1 = 0.15;
const float MOTOR1_KI = 0.0;
const float MOTOR1_KD = 0.005;
const int MOTOR1_MAX_PWM = 255;
const int MOTOR1_MIN_PWM = 50; 

const float ANGLE_TOLERANCE = 2.0; 
const float ANGLE_SLOW_ZONE = 8.0;  
const int TURN_BASE_SPEED = 50;
const int TURN_MAX_SPEED = 60;
int pwm;
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;
volatile long encoder4Count = 0;
float flywheelCurrentRPM = 0.0;
unsigned long lastFlywheelRPMCalculationTime = 0;
long lastFlywheelPulseCount = 0;
bool flywheelRunning = false;
const unsigned long FLYWHEEL_RPM_CALCULATION_INTERVAL = 1000; // Same as motor1
const int FLYWHEEL_PPR = 600;
const int R1 = 9;  
const int R2 = 17;

int currentButtonLeftState;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

struct RPMTarget {
  int rpm;
};

RPMTarget targets[] = {
  {4600},{4700},{4800},{4900},{5000},
  {0}
};

const int NUM_RPM_TARGETS = sizeof(targets) / sizeof(targets[0]);
int currentRPMIndex = 0;
bool lastButtonDownState = false;

enum RobotState {
  IDLE,
  MOVING_X_AXIS,
  MOVING_Y_AXIS,
  TURNING_TO_ANGLE1,
  WAITING_FOR_PHASE2_TRIGGER,
  TURNING_TO_ANGLE2_PHASE2,
  STOPPING
};
struct Target {
  float x;
  float y;
  float angle1;
  float angle2;
};
Target target1 = {450, 40, 0, 0};
Target target2 = {420, 130, 0, 0};
Target target3 = {410, 135, 0, 0};
Target target4 = {410, 140, 0, 0};
Target target5 = {420, 140, 0, 0};
Target target6 = {0, 300, 0, 0};
Target target7 = {0, 0, 0, 0};
Target target8 = {0, 0, 50, 90};

RobotState currentState = IDLE;
Target currentTarget;
unsigned long stateStartTime = 0;
unsigned long lastEncoderTime = 0;
float currentX = 0, currentY = 0;
float targetX = 0, targetY = 0;
float intermediateX = 0; 
bool motor1Running = false;
float targetAngle = 0;

float motor1TargetRPM = 0.0;
float motor1CurrentRPM = 0.0;
int motor1PWMOutput = 0;
unsigned long lastRPMCalculationTime = 0;
long lastMotor1PulseCount = 0;
const unsigned long RPM_CALCULATION_INTERVAL = 1000; 

float motor1_errorSum = 0.0;
float motor1_lastError = 0.0;
unsigned long motor1_lastTime = 0;

float initialYaw = 0;
float currentYaw = 0;
float adjustedYaw = 0;
bool yawCalibrated = false;

long startEncoder1 = 0, startEncoder2 = 0, startEncoder3 = 0, startEncoder4 = 0;
float targetDistance = 0;
bool movementStarted = false;

volatile long prevEncoder1 = 0, prevEncoder2 = 0, prevEncoder3 = 0, prevEncoder4 = 0, motor1EncoderCount = 0;
//flywheel
bool lastR1State = false;
bool lastButtonRightState = false;
//conveyer
const int relay1Pin = 3;
bool relay1State = true;
bool prevRelay1Pulse = true;
//elevator
bool motorOn = false;   // Forward toggle state
bool lastL1Processed = false;
void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  delay(1000);
  pinMode(elevator_pwm, OUTPUT);
  pinMode(elevator_dir, OUTPUT);
    pinMode(flywheelPwmPin, OUTPUT);
  pinMode(flywheelDirPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
digitalWrite(ledPin, HIGH);  

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
  
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  pinMode(encoder3PinA, INPUT_PULLUP);
  pinMode(encoder3PinB, INPUT_PULLUP);
  pinMode(encoder4PinA, INPUT_PULLUP);
  pinMode(encoder4PinB, INPUT_PULLUP);
  pinMode(motor1EncoderPinA, INPUT_PULLUP);
  pinMode(motor1EncoderPinB, INPUT_PULLUP);
  
  pinMode(relay1Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4PinA), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor1EncoderPinA), motor1EncoderISR, CHANGE);
  
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

void motor1EncoderISR() {
  motor1EncoderCount += (digitalRead(motor1EncoderPinB) == HIGH) ? 1 : -1;
}

void setMotor1RPM(int rpm) {
  motor1TargetRPM = rpm;
  motor1Running = (rpm > 0);
  
  motor1_errorSum = 0.0;
  motor1_lastError = 0.0;
  motor1_lastTime = millis();
  
  noInterrupts();
  motor1EncoderCount = 0;
  interrupts();
  lastMotor1PulseCount = 0;
  lastRPMCalculationTime = millis();
  
  if (motor1Running) {
    digitalWrite(MOTOR1_DIR, HIGH);
    motor1PWMOutput = 100; // Start with a moderate PWM value
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
    long currentPulseCount = motor1EncoderCount;
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
  
  float output = MOTOR1_KP1 * error + MOTOR1_KI * motor1_errorSum + MOTOR1_KD * dError;
  
  motor1_lastError = error;
  motor1_lastTime = now;
  
  return output;
}

void stopMotor1() {
  analogWrite(MOTOR1_PWM, 0);
  motor1Running = false;
  motor1TargetRPM = 0.0;
  motor1CurrentRPM = 0.0;
  motor1PWMOutput = 0;
}

// RPM cycling function
void handleRPMCycling() {
  bool currentButtonDownState = ps.buttonDown;

  // On rising edge of button press
  if (currentButtonDownState && !lastButtonDownState) {
    int targetRPM = targets[currentRPMIndex].rpm;
    setMotor1RPM(targetRPM);
    
    Serial.print("Motor1 RPM set to: ");
    Serial.print(targetRPM);
    Serial.print(" (Index: ");
    Serial.print(currentRPMIndex);
    Serial.println(")");

    currentRPMIndex = (currentRPMIndex + 1) % NUM_RPM_TARGETS;
  }
  int currentRPM = motor1CurrentRPM;
  int target8RPM = targets[1].rpm;

  if (abs(currentRPM - target8RPM) <= RPM_TOLERANCE) {
    digitalWrite(ledPin, LOW); 
  } else {
    digitalWrite(ledPin, HIGH);   
  }

  lastButtonDownState = currentButtonDownState;
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
                             ps.r1 || ps.l1);
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
  
  // Stop motor1 when exiting manual mode
  if (motor1Running) {
    stopMotor1();
  }
}

void executeManualGroup() {
  drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);
  flywheel();
  conveyer();
  elevator();
  handleRPMCycling(); // Add RPM cycling functionality
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
  stopAllMotors();  
  delay(100);
  currentTarget = target;

  targetX = target.x;
  targetY = target.y;
  targetAngle = target.angle1;

  currentX = 0;
  currentY = 0;
  movementStarted = false;
  currentState = MOVING_X_AXIS;
  stateStartTime = millis();
}
void stateMachine() {
  switch (currentState) {
    case IDLE:
      break;

    case MOVING_X_AXIS:
      if (moveInXAxis(targetX)) {
        movementStarted = false;
        delay(500);
        currentState = MOVING_Y_AXIS;
        stateStartTime = millis();
      }
      break;

    case MOVING_Y_AXIS:
      if (moveInYAxis(targetY)) {
        currentState = TURNING_TO_ANGLE1;
        stateStartTime = millis();
      }
      break;

    case TURNING_TO_ANGLE1:
      if (millis() - stateStartTime > 10000 || turnToAngle(currentTarget.angle1)) {
        stopDriveMotors();
        currentState = WAITING_FOR_PHASE2_TRIGGER;
        stateStartTime = millis();
      }
      break;

    case WAITING_FOR_PHASE2_TRIGGER:
      if (ps.buttonLeft) {
        delay(500);
        currentState = TURNING_TO_ANGLE2_PHASE2;
        stateStartTime = millis();
      }
      break;

    case TURNING_TO_ANGLE2_PHASE2:
      if (millis() - stateStartTime > 10000 || turnToAngle(currentTarget.angle2)) {
        stopDriveMotors();
        currentState = STOPPING;
        stateStartTime = millis();
      }
      break;

    case STOPPING:
      stopAllMotors();
      currentState = IDLE;
      break;
  }
}
bool moveInXAxis(float targetX) {
  if (!movementStarted) {
    startEncoder1 = encoder1Count;
    startEncoder2 = encoder2Count;
    startEncoder3 = encoder3Count;
    startEncoder4 = encoder4Count;
    targetDistance = abs(targetX - currentX);
    movementStarted = true;
  }
  
  long deltaEncoder1 = encoder1Count - startEncoder1;
  long deltaEncoder2 = encoder2Count - startEncoder2;
  long deltaEncoder3 = encoder3Count - startEncoder3;
  long deltaEncoder4 = encoder4Count - startEncoder4;
  
  float distanceMoved = abs((abs(deltaEncoder1) + abs(deltaEncoder3) + abs(deltaEncoder2) + abs(deltaEncoder4)) / 4.0) * CM_PER_PULSE;
  
  if (distanceMoved >= targetDistance - 0.5) {
    stopDriveMotors();
    currentX = targetX; 
    return true;
  }
  float deltaX = targetX - currentX;
  float remainingDistance = targetDistance - distanceMoved;
  float rampDistance = max(min(targetDistance * 0.4, 30.0), 5.0);
  int baseSpeed = BASE_SPEED;
  if (distanceMoved < rampDistance) {
    float accelerationRatio = distanceMoved / rampDistance;
    baseSpeed = (int)(BASE_SPEED * 0.4 + (BASE_SPEED * 0.6 * accelerationRatio));
  } 
  else if (remainingDistance < rampDistance) {
    float decelerationRatio = remainingDistance / rampDistance;
    baseSpeed = (int)(BASE_SPEED * 0.4 + (BASE_SPEED * 0.6 * decelerationRatio));
  }
  float avgDistance13 = (abs(deltaEncoder1) + abs(deltaEncoder3)) / 2.0 * CM_PER_PULSE;
  float avgDistance24 = (abs(deltaEncoder2) + abs(deltaEncoder4)) / 2.0 * CM_PER_PULSE;
  float overallAvg = (avgDistance13 + avgDistance24) / 2.0;
  float correction13 = constrain((overallAvg - avgDistance13) * KPX * 0.5, -MAX_CORRECTION/2, MAX_CORRECTION/2);
  float correction24 = constrain((overallAvg - avgDistance24) * KPX * 0.5, -MAX_CORRECTION/2, MAX_CORRECTION/2);
  float fine1 = constrain((avgDistance13 - abs(deltaEncoder1) * CM_PER_PULSE) * KPX * 0.3, -MAX_CORRECTION/4, MAX_CORRECTION/4);
  float fine3 = constrain((avgDistance13 - abs(deltaEncoder3) * CM_PER_PULSE) * KPX * 0.3, -MAX_CORRECTION/4, MAX_CORRECTION/4);
  float fine2 = constrain((avgDistance24 - abs(deltaEncoder2) * CM_PER_PULSE) * KPX * 0.3, -MAX_CORRECTION/4, MAX_CORRECTION/4);
  float fine4 = constrain((avgDistance24 - abs(deltaEncoder4) * CM_PER_PULSE) * KPX * 0.3, -MAX_CORRECTION/4, MAX_CORRECTION/4);
  int direction = (deltaX < 0) ? -1 : 1;
  int speed1 = baseSpeed + correction13 + fine1;
  int speed2 = baseSpeed + correction24 + fine2;
  int speed3 = baseSpeed + correction13 + fine3;
  int speed4 = baseSpeed + correction24 + fine4;
  speed1 = constrain(speed1 * direction, -255, 255);
  speed2 = constrain(-speed2 * direction, -255, 255);  
  speed3 = constrain(speed3 * direction, -255, 255);
  speed4 = constrain(-speed4 * direction, -255, 255);  
  setMotorSpeed(1, speed1);
  setMotorSpeed(2, speed2);
  setMotorSpeed(3, speed3);
  setMotorSpeed(4, speed4);
  return false;
}
bool moveInYAxis(float targetY) {
  if (!movementStarted) {
    startEncoder1 = encoder1Count;
    startEncoder2 = encoder2Count;
    startEncoder3 = encoder3Count;
    startEncoder4 = encoder4Count;
    targetDistance = abs(targetY - currentY);
    movementStarted = true;  }
  
  long deltaEncoder1 = encoder1Count - startEncoder1;
  long deltaEncoder2 = encoder2Count - startEncoder2;
  long deltaEncoder3 = encoder3Count - startEncoder3;
  long deltaEncoder4 = encoder4Count - startEncoder4;
  
  float distanceMoved = abs((abs(deltaEncoder1) + abs(deltaEncoder3) + abs(deltaEncoder2) + abs(deltaEncoder4)) / 4.0) * CM_PER_PULSE;
  
  if (distanceMoved >= targetDistance - 0.5) {
    stopDriveMotors();
    currentY = targetY; 
    return true;
  }  
  
  float deltaY = targetY - currentY;
  float remainingDistance = targetDistance - distanceMoved;
  float rampDistance = max(min(targetDistance * 0.4, 30.0), 5.0);
  
    int baseSpeed = BASE_SPEED;
  if (distanceMoved < rampDistance) {
    float accelerationRatio = distanceMoved / rampDistance;
    baseSpeed = (int)(BASE_SPEED * 0.4 + (BASE_SPEED * 0.6 * accelerationRatio));
  } 
  else if (remainingDistance < rampDistance) {
    float decelerationRatio = remainingDistance / rampDistance;
    baseSpeed = (int)(BASE_SPEED * 0.4 + (BASE_SPEED * 0.6 * decelerationRatio));
  }
  
  float avgDistance = (abs(deltaEncoder1) + abs(deltaEncoder2) + abs(deltaEncoder3) + abs(deltaEncoder4)) / 4.0 * CM_PER_PULSE;
  
  float error1 = (abs(deltaEncoder1) * CM_PER_PULSE) - avgDistance;
  float error2 = (abs(deltaEncoder2) * CM_PER_PULSE) - avgDistance;
  float error3 = (abs(deltaEncoder3) * CM_PER_PULSE) - avgDistance;
  float error4 = (abs(deltaEncoder4) * CM_PER_PULSE) - avgDistance;
  
  float correction1 = constrain(-KPY * error1, -MAX_CORRECTION, MAX_CORRECTION);
  float correction2 = constrain(-KPY * error2, -MAX_CORRECTION, MAX_CORRECTION);
  float correction3 = constrain(-KPY * error3, -MAX_CORRECTION, MAX_CORRECTION);
  float correction4 = constrain(-KPY * error4, -MAX_CORRECTION, MAX_CORRECTION);
  
  int direction = (deltaY < 0) ? -1 : 1;
  
  int speed1 = constrain((baseSpeed + correction1) * direction, -255, 255);
  int speed2 = constrain((baseSpeed + correction2) * direction, -255, 255);
  int speed3 = constrain((baseSpeed + correction3) * direction, -255, 255);
  int speed4 = constrain((baseSpeed + correction4) * direction, -255, 255);
  
  setMotorSpeed(1, speed1);
  setMotorSpeed(2, speed2);
  setMotorSpeed(3, speed3);
  setMotorSpeed(4, speed4);

  return false;
}

bool turnToAngle(float targetAngle) {
  if (!yawCalibrated) {
    return true;
  }
  if (readIMU()) {
float error = normalizeAngle(targetAngle - adjustedYaw);
float absError = abs(error);
    Serial.print("Target: "); Serial.print(targetAngle);
    Serial.print(" | AdjustedYaw: "); Serial.print(adjustedYaw);
    Serial.print(" | Error: "); Serial.println(error);
    if (absError <= ANGLE_TOLERANCE) {
      stopDriveMotors();
      return true;
    }
    int speed = TURN_BASE_SPEED;
    if (absError < ANGLE_SLOW_ZONE) {
      speed = map(absError, ANGLE_TOLERANCE, ANGLE_SLOW_ZONE, TURN_BASE_SPEED/3, TURN_BASE_SPEED);
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
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
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
      if (adjustedYaw <= -180) adjustedYaw += 360;
      return true;
    }
  }
  return false;
}
void rotateClockwise(int speed) {
  digitalWrite(M1_DIR, HIGH); analogWrite(M1_PWM, speed);
  digitalWrite(M2_DIR, LOW); analogWrite(M2_PWM, speed);
  digitalWrite(M3_DIR, LOW); analogWrite(M3_PWM, speed);
  digitalWrite(M4_DIR, HIGH); analogWrite(M4_PWM, speed);
}
void rotateCounterClockwise(int speed) {
  digitalWrite(M1_DIR, LOW); analogWrite(M1_PWM, speed);
  digitalWrite(M2_DIR, HIGH); analogWrite(M2_PWM, speed);
  digitalWrite(M3_DIR, HIGH); analogWrite(M3_PWM, speed);
  digitalWrite(M4_DIR, LOW); analogWrite(M4_PWM, speed);
}
void setMotorSpeed(int motor, int speed) {
  int pwmPin, dirPin;
  switch (motor) {
    case 1: pwmPin = M1_PWM; dirPin = M1_DIR; break;
    case 2: pwmPin = M2_PWM; dirPin = M2_DIR; break;
    case 3: pwmPin = M3_PWM; dirPin = M3_DIR; break;
    case 4: pwmPin = M4_PWM; dirPin = M4_DIR; break;
    default: return;
  }
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}
void stopDriveMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}
void stopAllMotors() {
  stopDriveMotors();
  stopMotor2();
}

void startMotor2(int pwm) {
  if (pwm > 0) {  
    digitalWrite(MOTOR2_DIR, HIGH);
    analogWrite(MOTOR2_PWM, pwm);
  }
}
void stopMotor2() {
  analogWrite(MOTOR2_PWM, 0);
}
void updatePosition() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) { 
    lastPrint = millis();
  }
}
void encoder1ISR() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}
void encoder2ISR() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}
void encoder3ISR() {
  if (digitalRead(encoder3PinA) == digitalRead(encoder3PinB)) {
    encoder3Count++;
  } else {
    encoder3Count--;
  }
}
void encoder4ISR() {
  if (digitalRead(encoder4PinA) == digitalRead(encoder4PinB)) {
    encoder4Count--;
  } else {
    encoder4Count++;
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
      motor1EncoderCount = 0; 
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

  if (flywheelRunning && pwm > 0) {
    updateFlywheelRPM();
  }

  if (pwm == 150) {
    digitalWrite(ledPin, LOW);
    Serial.println("LOW");
  } else {
    digitalWrite(ledPin, HIGH);
  }

  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) { 
    Serial.print("Flywheel PWM: ");
    Serial.print(pwm);
    Serial.print(", RPM: ");
    Serial.println(flywheelCurrentRPM);
    lastPrintTime = millis();
  }

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
    long currentPulseCount = motor1EncoderCount; 
    interrupts();
    
    long pulseDiff = currentPulseCount - lastFlywheelPulseCount;
    float timeInterval = (currentTime - lastFlywheelRPMCalculationTime) / 1000.0; 
    
    flywheelCurrentRPM = (pulseDiff / timeInterval) * 60.0 / FLYWHEEL_PPR;
    flywheelCurrentRPM = abs(flywheelCurrentRPM); 
    
    lastFlywheelPulseCount = currentPulseCount;
    lastFlywheelRPMCalculationTime = currentTime;
    
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
    digitalWrite(relay1Pin, relay1State ? LOW : HIGH);
    Serial.println(relay1State ? "Relay 1 ON" : "Relay 1 OFF");
  }
  prevRelay1Pulse = relay1Pressed;
}
void elevator() {
  if (ps.l1 == 1 && !lastL1Processed) {
    motorOn = !motorOn;
    if (motorOn) {
      digitalWrite(elevator_dir, HIGH);
      analogWrite(elevator_pwm, 100);   
      Serial.println("Motor FORWARD ON");
    } else {
      analogWrite(elevator_pwm, 0);      
      Serial.println("Motor STOPPED");
    }
    lastL1Processed = true;
  }
  if (ps.l1 == 2 && !lastL1Processed) {
    digitalWrite(elevator_dir, LOW);   
    analogWrite(elevator_pwm, 100);    
    motorOn = true;               
    Serial.println("Motor REVERSE ON");
    lastL1Processed = true;
  }
  if (ps.l1 == 0) {
    lastL1Processed = false;
  }
}