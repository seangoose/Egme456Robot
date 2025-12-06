/* ==========================================
   AUTOWIPER MESSY ROOM COMPETITION CODE
   EGME 456 - Cal State Fullerton
   ==========================================
   Hardware: Parallax BOE Shield-Bot + Arduino Uno
   Strategy: Coordinate-based systematic sweeping
   Match Duration: 60 seconds

   CRITICAL: Color sensor mounted at REAR of robot
            Robot extends 8" FORWARD from sensor position
   ========================================== */

// ==================== LIBRARIES ====================
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// ==================== CONFIGURATION ====================
#define DEBUG_MODE false          // Set true for Serial debugging
#define TEST_SENSORS_ONLY false   // Set true to only test sensors
#define CALIBRATION_MODE false    // Set true to run calibration sequence

// ==================== PIN DEFINITIONS ====================
// Drive Servos (Parallax continuous rotation)
#define PIN_SERVO_LEFT 13
#define PIN_SERVO_RIGHT 12

// Arm Servos (Standard hobby servos)
#define PIN_ARM_LEFT 11
#define PIN_ARM_RIGHT 10

// TCS3200 Color Sensor
#define PIN_COLOR_OUT 4
#define PIN_COLOR_S2 5
#define PIN_COLOR_S3 A0

// Ultrasonic Sensor (3-pin: shared trigger/echo on D8)
#define PIN_ULTRASONIC 8

// I2C addresses for LSM6DSOX gyroscope (A4=SDA, A5=SCL)
#define LSM6DSOX_ADDR_PRIMARY 0x6A
#define LSM6DSOX_ADDR_SECONDARY 0x6B

// Debug outputs
#define PIN_BUZZER 3
#define PIN_LED_1 6
#define PIN_LED_2 7

// ==================== SERVO POSITIONS ====================
// Three arm positions for strategic manipulation
#define ARM_PERPENDICULAR 90       // Sideways sweeping (12" width)
#define ARM_LEFT_RETRACTED 152     // 90+62 CCW toward BACK (tech inspection)
#define ARM_RIGHT_RETRACTED 28     // 90-62 CW toward BACK
#define ARM_LEFT_FORWARD 30        // 90-60 forward for cube manipulation
#define ARM_RIGHT_FORWARD 150      // 90+60 forward for cube manipulation

// Drive servo speeds (microseconds offset from 1500)
#define SERVO_STOP 1500
#define SERVO_FULL_SPEED 200

// ==================== PHYSICAL CONSTANTS ====================
#define ROBOT_LENGTH 8.0
#define ROBOT_WIDTH_RETRACTED 4.75
#define ROBOT_WIDTH_DEPLOYED 12.0
#define SWEEP_WIDTH 12.0

#define FIELD_WIDTH 48.0
#define FIELD_LENGTH 48.0
#define BOUNDARY_WIDTH 2.0

#define MIN_SENSOR_Y 4.0
#define MAX_SENSOR_Y 40.0

// Movement calibration
#define SPEED_INCHES_PER_SEC 9.0
#define TURN_90_DEGREES_MS 590
#define ODOMETRY_CORRECTION 0.95

// Sensor thresholds
#define BLACK_THRESHOLD 600
#define COLOR_STABILIZE_MS 20
#define PULSEIN_TIMEOUT 30000
#define OBSTACLE_DISTANCE_THRESHOLD 12.0  // inches
#define GYRO_DEADBAND 0.5

// Timing
#define COLOR_CACHE_VALID_MS 50
#define WATCHDOG_TIMEOUT_MS 5000
#define WATCHDOG_MIN_PROGRESS 1.0

// ==================== GLOBAL OBJECTS ====================
Servo servoLeft, servoRight;
Servo armLeft, armRight;
Adafruit_LSM6DSOX lsm6ds;

// ==================== ARM STATE TRACKING ====================
enum ArmPosition { ARMS_RETRACTED, ARMS_PERPENDICULAR, ARMS_FORWARD };
ArmPosition currentArmPosition = ARMS_RETRACTED;

// ==================== GLOBAL STATE VARIABLES ====================
float colorSensor_X = 24.0;
float colorSensor_Y = 20.0;
float heading = 0.0;

float getRobotFrontY() { return colorSensor_Y + ROBOT_LENGTH; }
float getRobotCenterY() { return colorSensor_Y + ROBOT_LENGTH/2.0; }

bool startedOnRedSide = false;
bool currentlyOnOwnSide = true;

enum StrategyPhase { INIT, OFFENSIVE, DEFENSIVE };
StrategyPhase currentPhase = INIT;

int currentLane = 1;
bool sweepingForward = true;
int completedOffensivePasses = 0;

unsigned long startTime = 0;
unsigned long lastGyroUpdate = 0;
float gyroHeading = 0.0;

bool positionUncertain = false;
bool gyroAvailable = false;
bool phase1Complete = false;

unsigned long lastRedReading = 0;
unsigned long lastBlueReading = 0;
unsigned long lastColorReadTime = 0;

unsigned long lastSweepProgress = 0;
float lastWatchdogY = 0;

// ==================== FUNCTION PROTOTYPES ====================
// Movement
void maneuver(int speedLeft, int speedRight, int msTime);
void stopMotors();
float estimateDistance(int duration_ms, int speed);
void updatePositionFromMovement(int speedLeft, int speedRight, int duration_ms);
void moveForward(float distance_inches);
void moveBackward(float distance_inches);
void rotateLeft(float degrees);
void rotateRight(float degrees);
void rotateToHeading(float target_heading);
void navigateToCoordinate(float target_X, float target_Y);

// Sensors
unsigned long measureColorDuration(bool readRed);
void readColorSensorCached(unsigned long &red, unsigned long &blue);
bool detectBlackBoundary();
bool detectBlackBoundaryFast();
int checkFieldSide();
void updateGyroscope();
float getUltrasonicDistance();

// Arm control
void deployArmsPerpendicular();
void deployArmsForward();
void retractArms();
void ensureArmsRetracted();

// Obstacle avoidance
void avoidObstacle();

// Strategy
void executePhase1_OffensiveSweep();
void executePhase2_DefensiveClearing();

// Error handling
void handleBoundaryEmergency();
void recoverPosition();

// Calibration
void runCalibrationMode();
void calibrateWheelDistance();
void calibrateTurnLeft();
void calibrateTurnRight();
void calibrateObstacleAvoidance();
void calibrateArmPositions();
void statusPattern(int testNumber);

// Utilities
void statusBeep(int frequency, int duration);
float normalizeAngle(float angle);
bool checkWatchdog();
void resetWatchdog();

// ==================== SETUP ====================
void setup() {
  #if DEBUG_MODE || TEST_SENSORS_ONLY || CALIBRATION_MODE
    Serial.begin(9600);
    Serial.println(F("AUTOWIPER v2.0"));
  #endif

  Wire.begin();
  delay(100);

  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);
  armLeft.attach(PIN_ARM_LEFT);
  armRight.attach(PIN_ARM_RIGHT);
  stopMotors();

  pinMode(PIN_COLOR_OUT, INPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);

  // Initialize gyroscope
  #if DEBUG_MODE || TEST_SENSORS_ONLY || CALIBRATION_MODE
    Serial.println(F("I2C scan:"));
    int deviceCount = 0;
    for(byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if(Wire.endTransmission() == 0) {
        Serial.print(F(" 0x")); Serial.println(addr, HEX);
        deviceCount++;
      }
    }
    Serial.print(F("Found: ")); Serial.println(deviceCount);
  #endif

  gyroAvailable = lsm6ds.begin_I2C() ||
                  lsm6ds.begin_I2C(LSM6DSOX_ADDR_PRIMARY, &Wire) ||
                  lsm6ds.begin_I2C(LSM6DSOX_ADDR_SECONDARY, &Wire);

  if(gyroAvailable) {
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    #if DEBUG_MODE || TEST_SENSORS_ONLY || CALIBRATION_MODE
      Serial.println(F("Gyro: OK"));
    #endif
  } else {
    #if DEBUG_MODE || TEST_SENSORS_ONLY || CALIBRATION_MODE
      Serial.println(F("Gyro: FAIL"));
    #endif
  }

  // Test ultrasonic
  #if DEBUG_MODE || TEST_SENSORS_ONLY || CALIBRATION_MODE
    float testDist = getUltrasonicDistance();
    Serial.print(F("Ultra: "));
    if(testDist < 999) { Serial.print(testDist, 1); Serial.println(F("in")); }
    else { Serial.println(F("N/A")); }
  #endif

  statusBeep(3000, 500);
  digitalWrite(PIN_LED_1, HIGH);
  delay(2000);
  digitalWrite(PIN_LED_1, LOW);

  #if CALIBRATION_MODE
    runCalibrationMode();
  #endif

  #if TEST_SENSORS_ONLY
    Serial.println(F("TEST MODE"));
    Serial.println(F("R<B=red Both>600=black"));
    float testHeading = 0.0;
    unsigned long lastTestUpdate = millis();

    while(true) {
      unsigned long red = measureColorDuration(true);
      unsigned long blue = measureColorDuration(false);
      float dist = getUltrasonicDistance();

      Serial.print(F("R:")); Serial.print(red);
      Serial.print(F(" B:")); Serial.print(blue);
      Serial.print((red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD) ? F(" BLK") : (red < blue ? F(" RED") : F(" BLU")));

      Serial.print(F(" D:"));
      if(dist < 999) Serial.print(dist, 1); else Serial.print(F("--"));

      if(gyroAvailable) {
        sensors_event_t accel, gyro, temp;
        if(lsm6ds.getEvent(&accel, &gyro, &temp)) {
          unsigned long now = millis();
          float dt = (now - lastTestUpdate) / 1000.0;
          lastTestUpdate = now;
          float gyroZ_degPerSec = gyro.gyro.z * (180.0 / PI);
          testHeading += gyroZ_degPerSec * dt;
          while(testHeading > 180.0) testHeading -= 360.0;
          while(testHeading < -180.0) testHeading += 360.0;
          Serial.print(F(" H:")); Serial.print(testHeading, 1);
        }
      }
      Serial.println();
      delay(100);
    }
  #endif

  // Determine starting position
  delay(100);
  unsigned long red = measureColorDuration(true);
  unsigned long blue = measureColorDuration(false);

  if(red < 600 && blue < 600) {
    startedOnRedSide = (red < blue);
    colorSensor_Y = startedOnRedSide ? 18.0 : 26.0;
  } else {
    colorSensor_Y = 22.0;
    startedOnRedSide = true;
  }

  #if DEBUG_MODE
    Serial.print(F("Side:")); Serial.println(startedOnRedSide ? F("RED") : F("BLU"));
    Serial.print(F("Y:")); Serial.println(colorSensor_Y);
  #endif

  gyroHeading = 0.0;
  heading = 0.0;
  lastGyroUpdate = millis();

  retractArms();
  startTime = millis();
  statusBeep(2000, 200);
  digitalWrite(PIN_LED_1, HIGH);

  #if DEBUG_MODE
    Serial.println(F("READY"));
  #endif
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsed = (currentTime - startTime) / 1000;

  updateGyroscope();
  bool atBoundary = detectBlackBoundary();

  // Obstacle detection
  float obstacleDistance = getUltrasonicDistance();
  if(obstacleDistance < OBSTACLE_DISTANCE_THRESHOLD && currentPhase != INIT && elapsed > 5) {
    #if DEBUG_MODE
      Serial.print(F("!OBS ")); Serial.println(obstacleDistance, 1);
    #endif
    avoidObstacle();
    return;
  }

  // Phase management
  if(elapsed < 5) {
    currentPhase = INIT;
  } else if(elapsed < 35 && !phase1Complete) {
    currentPhase = OFFENSIVE;
  } else {
    currentPhase = DEFENSIVE;
  }

  // Time limit
  if(elapsed >= 60) {
    stopMotors();
    retractArms();
    digitalWrite(PIN_LED_2, HIGH);
    statusBeep(4000, 1000);
    #if DEBUG_MODE
      Serial.println(F("END"));
    #endif
    while(true) { delay(1000); }
  }

  // Boundary handling
  if(atBoundary) {
    bool expectedBoundary = (colorSensor_Y < 6.0 && currentPhase == INIT) ||
                            (getRobotFrontY() > 45.0 && currentPhase == OFFENSIVE);
    if(!expectedBoundary) {
      #if DEBUG_MODE
        Serial.println(F("!BND"));
      #endif
      handleBoundaryEmergency();
      return;
    }
  }

  // Execute phase
  switch(currentPhase) {
    case INIT:
      if(elapsed > 4) {
        currentLane = 1;
        completedOffensivePasses = 0;
        #if DEBUG_MODE
          Serial.println(F("->OFF"));
        #endif
      }
      break;
    case OFFENSIVE:
      executePhase1_OffensiveSweep();
      break;
    case DEFENSIVE:
      executePhase2_DefensiveClearing();
      break;
  }

  // Position verification (every 5 seconds to reduce output)
  static unsigned long lastPositionCheck = 0;
  if(currentTime - lastPositionCheck > 5000 && elapsed > 5) {
    lastPositionCheck = currentTime;
    int fieldSide = checkFieldSide();
    if(fieldSide != 0) {
      bool expectedOnOwnSide = (colorSensor_Y < 24.0);
      bool actuallyOnOwnSide = (fieldSide == 1);
      if(expectedOnOwnSide != actuallyOnOwnSide) {
        positionUncertain = true;
        #if DEBUG_MODE
          Serial.println(F("?POS"));
        #endif
      }
    }
    if(positionUncertain) recoverPosition();

    #if DEBUG_MODE
      Serial.print(elapsed); Serial.print(F("s P"));
      Serial.print(currentPhase); Serial.print(F(" Y"));
      Serial.print(colorSensor_Y, 0); Serial.print(F(" H"));
      Serial.print(heading, 0);
      if(obstacleDistance < 999) { Serial.print(F(" D")); Serial.print(obstacleDistance, 0); }
      Serial.println();
    #endif
  }

  delay(20);
}

// ==================== ULTRASONIC SENSOR ====================
float getUltrasonicDistance() {
  // 3-pin ultrasonic: same pin for trigger and echo
  pinMode(PIN_ULTRASONIC, OUTPUT);
  digitalWrite(PIN_ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC, LOW);

  pinMode(PIN_ULTRASONIC, INPUT);
  unsigned long duration = pulseIn(PIN_ULTRASONIC, HIGH, 30000); // 30ms timeout

  if(duration == 0) return 999.0; // No echo

  float distance = duration / 148.0; // Convert to inches
  return distance;
}

// ==================== OBSTACLE AVOIDANCE ====================
void avoidObstacle() {
  stopMotors();
  statusBeep(4000, 100);
  ensureArmsRetracted();

  // Determine turn direction based on available space
  float spaceLeft = colorSensor_X - 6.0;
  float spaceRight = 42.0 - colorSensor_X;
  bool turnRight = (spaceRight > spaceLeft);

  // Check if turning would hit boundary
  if(turnRight && colorSensor_X > 38.0) turnRight = false;
  if(!turnRight && colorSensor_X < 10.0) turnRight = true;

  #if DEBUG_MODE
    Serial.print(F("AVD ")); Serial.println(turnRight ? F("R") : F("L"));
  #endif

  // Back up first
  moveBackward(4.0);

  // Turn away from obstacle
  if(turnRight) {
    rotateRight(60.0);
  } else {
    rotateLeft(60.0);
  }

  // Check if still blocked
  float newDist = getUltrasonicDistance();
  if(newDist < OBSTACLE_DISTANCE_THRESHOLD) {
    // Try opposite direction
    if(turnRight) {
      rotateLeft(120.0); // Undo and go other way
    } else {
      rotateRight(120.0);
    }
  }

  // Move forward to clear
  moveForward(6.0);

  // Resume original heading (approximately)
  rotateToHeading(0.0);
  delay(100);
}

// ==================== MOVEMENT PRIMITIVES ====================
void maneuver(int speedLeft, int speedRight, int msTime) {
  servoLeft.writeMicroseconds(SERVO_STOP + speedLeft);
  servoRight.writeMicroseconds(SERVO_STOP - speedRight);
  if(msTime > 0) delay(msTime);
  else if(msTime == -1) { servoLeft.detach(); servoRight.detach(); }
}

void stopMotors() {
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  delay(50);
}

float estimateDistance(int duration_ms, int speed) {
  float speedFraction = abs(speed) / 200.0;
  return (SPEED_INCHES_PER_SEC * speedFraction * duration_ms / 1000.0) * ODOMETRY_CORRECTION;
}

void updatePositionFromMovement(int speedLeft, int speedRight, int duration_ms) {
  float distance = estimateDistance(duration_ms, (speedLeft + speedRight) / 2);
  float radians = heading * PI / 180.0;
  colorSensor_X += distance * sin(radians);
  colorSensor_Y += distance * cos(radians);
  colorSensor_X = constrain(colorSensor_X, 0, FIELD_WIDTH);
  colorSensor_Y = constrain(colorSensor_Y, 0, FIELD_LENGTH);
}

// ==================== NAVIGATION ====================
void moveForward(float distance_inches) {
  int duration = (int)(distance_inches / SPEED_INCHES_PER_SEC * 1000.0);
  unsigned long startMove = millis();
  float targetHeading = heading;

  while(millis() - startMove < duration) {
    updateGyroscope();

    // Obstacle check during movement
    if(getUltrasonicDistance() < OBSTACLE_DISTANCE_THRESHOLD) {
      #if DEBUG_MODE
        Serial.println(F("!OBS"));
      #endif
      break;
    }

    float headingError = normalizeAngle(heading - targetHeading);
    int correction = constrain((int)(headingError * 2.5), -50, 50);
    maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 50);
    updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 50);

    if(detectBlackBoundary() && getRobotFrontY() > 45.0) break;
  }
  stopMotors();
}

void moveBackward(float distance_inches) {
  int duration = (int)(distance_inches / SPEED_INCHES_PER_SEC * 1000.0);
  maneuver(-SERVO_FULL_SPEED, -SERVO_FULL_SPEED, duration);
  updatePositionFromMovement(-SERVO_FULL_SPEED, -SERVO_FULL_SPEED, duration);
  stopMotors();
}

void rotateLeft(float degrees) {
  int duration = (int)(degrees / 90.0 * TURN_90_DEGREES_MS);
  maneuver(-SERVO_FULL_SPEED, SERVO_FULL_SPEED, duration);
  heading = normalizeAngle(heading - degrees);
  stopMotors();
}

void rotateRight(float degrees) {
  int duration = (int)(degrees / 90.0 * TURN_90_DEGREES_MS);
  maneuver(SERVO_FULL_SPEED, -SERVO_FULL_SPEED, duration);
  heading = normalizeAngle(heading + degrees);
  stopMotors();
}

void rotateToHeading(float target_heading) {
  target_heading = normalizeAngle(target_heading);
  float error = normalizeAngle(target_heading - heading);
  if(abs(error) < 3.0) return;
  if(error > 0) rotateRight(abs(error));
  else rotateLeft(abs(error));
}

void navigateToCoordinate(float target_X, float target_Y) {
  ensureArmsRetracted();
  float deltaX = target_X - colorSensor_X;
  float deltaY = target_Y - colorSensor_Y;
  float targetHeading = atan2(deltaX, deltaY) * 180.0 / PI;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if(distance < 2.0) return;

  #if DEBUG_MODE
    Serial.print(F("NAV>")); Serial.print(target_X, 0);
    Serial.print(F(",")); Serial.println(target_Y, 0);
  #endif

  rotateToHeading(targetHeading);
  delay(100);

  int duration = (int)(distance / SPEED_INCHES_PER_SEC * 1000.0);
  unsigned long startMove = millis();

  while(millis() - startMove < duration) {
    // Obstacle check
    if(getUltrasonicDistance() < OBSTACLE_DISTANCE_THRESHOLD) {
      stopMotors();
      avoidObstacle();
      return;
    }

    updateGyroscope();
    float headingError = normalizeAngle(heading - targetHeading);
    int correction = constrain((int)(headingError * 2.5), -50, 50);
    maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 50);
    updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 50);

    if(detectBlackBoundaryFast()) break;
  }
  stopMotors();
  delay(100);
}

// ==================== SENSOR READING ====================
unsigned long measureColorDuration(bool readRed) {
  digitalWrite(PIN_COLOR_S2, LOW);
  digitalWrite(PIN_COLOR_S3, readRed ? LOW : HIGH);
  delay(COLOR_STABILIZE_MS);
  unsigned long duration = pulseIn(PIN_COLOR_OUT, LOW, PULSEIN_TIMEOUT);
  return (duration == 0) ? BLACK_THRESHOLD + 200 : duration;
}

void readColorSensorCached(unsigned long &red, unsigned long &blue) {
  unsigned long now = millis();
  if(now - lastColorReadTime < COLOR_CACHE_VALID_MS && lastColorReadTime > 0) {
    red = lastRedReading;
    blue = lastBlueReading;
    return;
  }
  red = measureColorDuration(true);
  blue = measureColorDuration(false);
  lastRedReading = red;
  lastBlueReading = blue;
  lastColorReadTime = now;
}

bool detectBlackBoundary() {
  unsigned long red, blue;
  readColorSensorCached(red, blue);
  return (red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD);
}

bool detectBlackBoundaryFast() {
  if(millis() - lastColorReadTime < COLOR_CACHE_VALID_MS && lastColorReadTime > 0) {
    return (lastRedReading > BLACK_THRESHOLD && lastBlueReading > BLACK_THRESHOLD);
  }
  return detectBlackBoundary();
}

int checkFieldSide() {
  unsigned long red, blue;
  readColorSensorCached(red, blue);
  if(red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD) return 0;
  bool readingRed = (red < blue);
  return (readingRed == startedOnRedSide) ? 1 : -1;
}

void updateGyroscope() {
  if(!gyroAvailable) return;
  sensors_event_t accel, gyro, temp;
  if(!lsm6ds.getEvent(&accel, &gyro, &temp)) return;

  unsigned long currentTime = millis();
  float dt = (currentTime - lastGyroUpdate) / 1000.0;
  if(dt > 0.001 && dt < 1.0) {
    float gyroZ_degPerSec = gyro.gyro.z * (180.0 / PI);
    if(abs(gyroZ_degPerSec) > GYRO_DEADBAND) {
      heading = normalizeAngle(heading + gyroZ_degPerSec * dt);
    }
  }
  lastGyroUpdate = currentTime;
}

// ==================== ARM CONTROL ====================
void deployArmsPerpendicular() {
  if(currentArmPosition == ARMS_PERPENDICULAR) return;
  armLeft.write(ARM_PERPENDICULAR);
  armRight.write(ARM_PERPENDICULAR);
  delay(500);
  currentArmPosition = ARMS_PERPENDICULAR;
  #if DEBUG_MODE
    Serial.println(F("ARM:PERP"));
  #endif
}

void deployArmsForward() {
  if(currentArmPosition == ARMS_FORWARD) return;
  armLeft.write(ARM_LEFT_FORWARD);
  armRight.write(ARM_RIGHT_FORWARD);
  delay(500);
  currentArmPosition = ARMS_FORWARD;
  #if DEBUG_MODE
    Serial.println(F("ARM:FWD"));
  #endif
}

void retractArms() {
  if(currentArmPosition == ARMS_RETRACTED) return;
  armLeft.write(ARM_LEFT_RETRACTED);
  armRight.write(ARM_RIGHT_RETRACTED);
  delay(500);
  currentArmPosition = ARMS_RETRACTED;
  #if DEBUG_MODE
    Serial.println(F("ARM:RET"));
  #endif
}

void ensureArmsRetracted() {
  if(currentArmPosition != ARMS_RETRACTED) retractArms();
}

// ==================== STRATEGY EXECUTION ====================
void executePhase1_OffensiveSweep() {
  if(completedOffensivePasses >= 4) {
    phase1Complete = true;
    #if DEBUG_MODE
      Serial.println(F("P1 DONE"));
    #endif
    return;
  }

  static bool inPosition = false;
  static bool sweepComplete = false;
  static bool armsTransitioned = false;

  if(!inPosition) {
    float laneX = 6.0 + (completedOffensivePasses * 12.0);
    #if DEBUG_MODE
      Serial.print(F("L")); Serial.println(completedOffensivePasses + 1);
    #endif
    navigateToCoordinate(laneX, 16.0);
    rotateToHeading(0.0);
    deployArmsPerpendicular();  // Start with perpendicular arms
    inPosition = true;
    sweepComplete = false;
    armsTransitioned = false;
    delay(300);
  }

  if(inPosition && !sweepComplete) {
    resetWatchdog();

    while(colorSensor_Y < 40.0) {
      // Transition arms at 3/4 distance (Y=36)
      if(!armsTransitioned && colorSensor_Y >= 36.0) {
        stopMotors();
        deployArmsForward();  // Switch to forward position
        armsTransitioned = true;
        #if DEBUG_MODE
          Serial.println(F("ARM->FWD"));
        #endif
        delay(200);
      }

      // Obstacle check
      if(getUltrasonicDistance() < OBSTACLE_DISTANCE_THRESHOLD) {
        stopMotors();
        avoidObstacle();
        break;
      }

      if(detectBlackBoundaryFast()) {
        #if DEBUG_MODE
          Serial.println(F("!BND"));
        #endif
        break;
      }

      if(checkWatchdog()) {
        statusBeep(1000, 200);
        moveBackward(4.0);
        break;
      }

      updateGyroscope();
      float headingError = normalizeAngle(heading);
      int correction = constrain((int)(headingError * 2.5), -50, 50);
      maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 100);
      updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 100);
    }

    sweepComplete = true;
    stopMotors();
    retractArms();
    moveBackward(15.0);
    completedOffensivePasses++;
    inPosition = false;
    #if DEBUG_MODE
      Serial.print(F("PASS ")); Serial.println(completedOffensivePasses);
    #endif
    delay(200);
  }
}

void executePhase2_DefensiveClearing() {
  static bool isInitialized = false;
  const float DEFENSE_START_Y = 8.0;
  const float DEFENSE_END_Y = 28.0;

  if(!isInitialized) {
    currentLane = 1;
    #if DEBUG_MODE
      Serial.println(F("P2 DEF"));
    #endif
    navigateToCoordinate(6.0, DEFENSE_START_Y);
    rotateToHeading(0.0);
    deployArmsPerpendicular();
    isInitialized = true;
  }

  resetWatchdog();
  while(colorSensor_Y < DEFENSE_END_Y) {
    if(getUltrasonicDistance() < OBSTACLE_DISTANCE_THRESHOLD) {
      stopMotors();
      avoidObstacle();
      return;
    }

    if(detectBlackBoundaryFast()) break;
    if(checkWatchdog()) {
      statusBeep(1000, 200);
      moveBackward(4.0);
      break;
    }

    updateGyroscope();
    float headingError = normalizeAngle(heading);
    int correction = constrain((int)(headingError * 2.5), -50, 50);
    maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 100);
    updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 100);
  }

  stopMotors();
  retractArms();
  currentLane = (currentLane % 4) + 1;
  float nextLaneX = 6.0 + ((currentLane - 1) * 12.0);
  navigateToCoordinate(nextLaneX, DEFENSE_START_Y + 2.0);
  rotateToHeading(0.0);
  deployArmsPerpendicular();
}

// ==================== ERROR HANDLING ====================
void handleBoundaryEmergency() {
  stopMotors();
  statusBeep(500, 300);
  ensureArmsRetracted();

  if(getRobotFrontY() > 42.0) {
    maneuver(-SERVO_FULL_SPEED, -SERVO_FULL_SPEED, 800);
    colorSensor_Y = 36.0;
  } else if(colorSensor_Y < 8.0) {
    maneuver(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 600);
    colorSensor_Y = 10.0;
  } else if(colorSensor_X < 8.0 || colorSensor_X > 40.0) {
    if(colorSensor_X < 24.0) {
      rotateRight(45.0); moveForward(6.0); rotateLeft(45.0);
      colorSensor_X += 4.0;
    } else {
      rotateLeft(45.0); moveForward(6.0); rotateRight(45.0);
      colorSensor_X -= 4.0;
    }
  } else {
    moveBackward(6.0);
    positionUncertain = true;
  }
  stopMotors();
  delay(200);
  lastColorReadTime = 0;
}

void recoverPosition() {
  #if DEBUG_MODE
    Serial.println(F("RCV"));
  #endif
  stopMotors();
  ensureArmsRetracted();

  bool atBoundary = detectBlackBoundary();
  int fieldSide = checkFieldSide();

  if(atBoundary) {
    if(colorSensor_Y > 30.0) { colorSensor_Y = 40.0; moveBackward(6.0); }
    else { colorSensor_Y = 4.0; moveForward(6.0); }
  } else if(fieldSide != 0) {
    bool expectedOnOwnSide = (colorSensor_Y < 24.0);
    bool actuallyOnOwnSide = (fieldSide == 1);
    if(expectedOnOwnSide != actuallyOnOwnSide) {
      colorSensor_Y = actuallyOnOwnSide ? 18.0 : 30.0;
      if(!actuallyOnOwnSide) moveBackward(8.0);
    }
  }

  if(abs(heading) > 10.0) rotateToHeading(0.0);
  positionUncertain = false;
}

// ==================== CALIBRATION MODE ====================
void statusPattern(int testNumber) {
  for(int i = 0; i < testNumber; i++) {
    digitalWrite(PIN_LED_1, HIGH);
    statusBeep(2000, 200);
    delay(200);
    digitalWrite(PIN_LED_1, LOW);
    delay(300);
  }
}

void calibrateWheelDistance() {
  statusPattern(1);
  delay(2000);
  Serial.println(F("=== WHEEL DISTANCE ==="));
  Serial.println(F("Measure distance traveled:"));
  delay(1000);
  maneuver(200, 200, 1000);
  stopMotors();
  Serial.println(F("Done. Measure inches traveled."));
  Serial.println(F("SPEED = distance / 1.0"));
  delay(5000);
}

void calibrateTurnLeft() {
  statusPattern(2);
  delay(2000);
  Serial.println(F("=== LEFT TURN ==="));
  Serial.println(F("Watch rotation angle..."));
  delay(1000);
  maneuver(-200, 200, TURN_90_DEGREES_MS);
  stopMotors();
  Serial.println(F("Should be 90 degrees."));
  Serial.print(F("Current TURN_90_DEGREES_MS: ")); Serial.println(TURN_90_DEGREES_MS);
  delay(5000);
}

void calibrateTurnRight() {
  statusPattern(3);
  delay(2000);
  Serial.println(F("=== RIGHT TURN ==="));
  delay(1000);
  maneuver(200, -200, TURN_90_DEGREES_MS);
  stopMotors();
  Serial.println(F("Should be 90 degrees."));
  delay(5000);
}

void calibrateObstacleAvoidance() {
  statusPattern(4);
  delay(2000);
  Serial.println(F("=== OBSTACLE TEST ==="));
  Serial.println(F("Place obstacle 6in ahead"));
  delay(5000);

  while(getUltrasonicDistance() > OBSTACLE_DISTANCE_THRESHOLD) {
    maneuver(150, 150, 100);
    float d = getUltrasonicDistance();
    Serial.print(F("D:")); Serial.println(d, 1);
    delay(50);
  }

  Serial.println(F("Detected! Avoiding..."));
  avoidObstacle();
  Serial.println(F("Complete"));
  delay(3000);
}

void calibrateArmPositions() {
  statusPattern(5);
  delay(2000);
  Serial.println(F("=== ARM POSITIONS ==="));

  Serial.println(F("1. RETRACTED (152/28)"));
  retractArms();
  delay(3000);

  Serial.println(F("2. PERPENDICULAR (90/90)"));
  deployArmsPerpendicular();
  delay(3000);

  Serial.println(F("3. FORWARD (30/150)"));
  deployArmsForward();
  delay(3000);

  Serial.println(F("4. Back to RETRACTED"));
  retractArms();
  delay(3000);

  Serial.println(F("Verify positions correct"));
  delay(3000);
}

void runCalibrationMode() {
  Serial.println(F("===================="));
  Serial.println(F(" CALIBRATION MODE"));
  Serial.println(F("===================="));
  delay(2000);

  calibrateWheelDistance();
  calibrateTurnLeft();
  calibrateTurnRight();
  calibrateObstacleAvoidance();
  calibrateArmPositions();

  Serial.println(F("===================="));
  Serial.println(F(" CALIBRATION DONE"));
  Serial.println(F("===================="));

  for(int i = 0; i < 5; i++) {
    statusBeep(2000, 100);
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);
    delay(100);
    digitalWrite(PIN_LED_1, LOW);
    digitalWrite(PIN_LED_2, LOW);
    delay(100);
  }

  while(true) { delay(1000); }
}

// ==================== UTILITIES ====================
void statusBeep(int frequency, int duration) {
  tone(PIN_BUZZER, frequency, duration);
  delay(duration);
}

float normalizeAngle(float angle) {
  while(angle > 180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}

bool checkWatchdog() {
  unsigned long now = millis();
  if(abs(colorSensor_Y - lastWatchdogY) >= WATCHDOG_MIN_PROGRESS) {
    lastSweepProgress = now;
    lastWatchdogY = colorSensor_Y;
    return false;
  }
  if(now - lastSweepProgress > WATCHDOG_TIMEOUT_MS) {
    #if DEBUG_MODE
      Serial.println(F("!STUCK"));
    #endif
    return true;
  }
  return false;
}

void resetWatchdog() {
  lastSweepProgress = millis();
  lastWatchdogY = colorSensor_Y;
}
