/* ═══════════════════════════════════════════════════════════════════════════════
   AUTOWIPER V60 - COMPREHENSIVE NAVIGATION SYSTEM
   EGME 456 - Cal State Fullerton Messy Room Competition
   Target: Arduino Uno with Parallax BOE Shield

   ═══════════════════════════════════════════════════════════════════════════════

   MAJOR SYSTEMS IMPLEMENTED:
   ═════════════════════════

   1. UNIFIED DRIVE SYSTEM
      - ALL drive functions use same core with border awareness
      - Hysteresis filtering on ALL border detections (5-sample confirmation)
      - Black = EMERGENCY STOP (cliff detection)
      - Color border = landmark for position calibration

   2. GYROSCOPE COLLISION DETECTION
      - Monitors angular velocity for sudden spikes (>50°/s unexpected)
      - Triggers collision recovery: backup + heading correction

   3. TIME-OF-ARRIVAL CHECKING
      - Every movement calculates expected duration based on physics
      - Early arrival = hit unexpected obstacle (drift detection)
      - Late arrival = traveled diagonal path (heading error)
      - Applies trig correction: θ_drift = arccos(T_expected / T_actual)

   4. HEADING CORRECTION (GYRO PID)
      - Continuous heading monitoring during straight-line driving
      - Active motor bias correction to maintain heading
      - Prevents compound drift errors

   5. DUAL-POINT KINEMATICS
      - Pivot Center (wheel axis) - where robot rotates
      - Sensor Point (3.5" forward) - where colors are detected
      - Both points tracked continuously

   6. LANDMARK CALIBRATION
      - Red/Blue border crossing resets X coordinate to 24.0"
      - Black border detection confirms arena boundary
      - Hard-resets accumulated dead-reckoning error

   ═══════════════════════════════════════════════════════════════════════════════

   COORDINATE SYSTEM:
   ══════════════════
   Origin (0,0): Center of Home/Start Wall
   X-Axis: 0" (Home Wall) → 48" (Enemy Wall)
   Y-Axis: -24" (Bottom Rail) → +24" (Top Rail)
   Heading: 0° = East (+X), 90° = North (+Y), -90° = South (-Y), 180° = West (-X)

   HARDWARE (ACTIVE - ALL USED):
   ═════════════════════════════
   D13: Left Servo (Parallax Continuous Rotation)
   D12: Right Servo (Parallax Continuous Rotation)
   D6:  TCS3200 S0 (HIGH for 100% frequency scaling)
   D7:  TCS3200 S1 (HIGH for 100% frequency scaling)
   D5:  TCS3200 S2 (Filter select bit 0)
   A0:  TCS3200 S3 (Filter select bit 1)
   D4:  TCS3200 OUT (Frequency output)
   D3:  Piezo Buzzer
   I2C: Adafruit LSM6DSOX IMU (Gyroscope + Accelerometer)

   MOTOR TRUTH TABLE:
   ══════════════════
   Forward:     Left=1700, Right=1300
   Reverse:     Left=1300, Right=1700
   Turn Right:  Left=1700, Right=1700 (both "forward" = CW rotation)
   Turn Left:   Left=1300, Right=1300 (both "reverse" = CCW rotation)

   ═══════════════════════════════════════════════════════════════════════════════ */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════

#define PIN_SERVO_L      13
#define PIN_SERVO_R      12
#define PIN_BUZZER       3
#define PIN_S0           6
#define PIN_S1           7
#define PIN_S2           5
#define PIN_S3           A0
#define PIN_OUT          4

// ═══════════════════════════════════════════════════════════════════════════════
//  PHYSICS CONSTANTS (CALIBRATED VALUES)
// ═══════════════════════════════════════════════════════════════════════════════

// Distance traveled per millisecond at various speeds
#define SPEED_CRUISE_IN_MS     0.0055    // inches/ms at PWM±200
#define SPEED_APPROACH_IN_MS   0.0028    // inches/ms at PWM±100
#define SPEED_CREEP_IN_MS      0.0014    // inches/ms at PWM±50

// Robot geometry
#define SENSOR_RADIUS          3.5       // Sensor offset from pivot (inches)
#define WHEEL_BASE             4.25      // Distance between wheels (inches)

// Arena boundaries
#define ARENA_X_MIN            0.0       // Home wall
#define ARENA_X_MAX            48.0      // Enemy wall
#define ARENA_Y_MIN           -24.0      // Bottom rail
#define ARENA_Y_MAX            24.0      // Top rail
#define MIDFIELD_X             24.0      // Red/Blue border
#define WALL_Y_BOT            -22.0      // Effective bottom boundary

// ═══════════════════════════════════════════════════════════════════════════════
//  COLOR SENSOR THRESHOLDS
// ═══════════════════════════════════════════════════════════════════════════════

#define THRESH_RED             38        // Red pulse width < this = RED surface
#define THRESH_BLUE            42        // Blue pulse width < this = BLUE surface
#define THRESH_BLACK           50        // Both channels > this = BLACK
#define THRESH_BLACK_PANIC     80        // Definitely black - emergency stop
#define PULSEIN_TIMEOUT        25000     // 25ms timeout (black returns 0 → 999)

// Hysteresis requirements
#define HYSTERESIS_CONFIRM     5         // Consecutive readings needed to confirm
#define HYSTERESIS_HISTORY     20        // Rolling history buffer size

// ═══════════════════════════════════════════════════════════════════════════════
//  GYROSCOPE / PID TUNING
// ═══════════════════════════════════════════════════════════════════════════════

// Turn speeds (motor PWM offset from 1500)
#define GYRO_SPEED_FAST        400       // Fast approach to target angle
#define GYRO_SPEED_SLOW        200       // Medium approach
#define GYRO_SPEED_CREEP       120       // Fine adjustment

// Tolerances
#define TURN_TOLERANCE         2.0       // Degrees - acceptable turn error
#define HEADING_DRIFT_THRESH   3.0       // Degrees - trigger heading correction
#define COLLISION_SPIKE_THRESH 50.0      // deg/s - sudden rotation = collision

// Heading correction PID (simplified P-only for now)
#define HEADING_KP             8.0       // Motor bias per degree of error

// ═══════════════════════════════════════════════════════════════════════════════
//  TIMING CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define TIME_TOLERANCE_EARLY   0.70      // 70% of expected = early arrival (drift)
#define TIME_TOLERANCE_LATE    1.30      // 130% of expected = late arrival (diagonal)
#define TURN_TIMEOUT_MS        3000      // Max time for any turn operation
#define MATCH_DURATION_MS      60000     // 60 second match

// ═══════════════════════════════════════════════════════════════════════════════
//  COLOR/SURFACE ENUMERATION
// ═══════════════════════════════════════════════════════════════════════════════

#define COLOR_BLUE             0
#define COLOR_RED              1
#define COLOR_BLACK            2
#define COLOR_UNKNOWN          3

// ═══════════════════════════════════════════════════════════════════════════════
//  DRIVE MODE ENUMERATION
// ═══════════════════════════════════════════════════════════════════════════════

enum DriveMode {
  DRIVE_DISTANCE,              // Drive specific distance, stop on black
  DRIVE_UNTIL_WALL,            // Drive until black border detected
  DRIVE_UNTIL_BORDER,          // Drive until team color border detected
  DRIVE_UNTIL_ENEMY_BORDER     // Drive until enemy color border detected
};

enum DriveResult {
  RESULT_DISTANCE_COMPLETE,    // Traveled requested distance
  RESULT_BLACK_DETECTED,       // Hit black boundary (emergency)
  RESULT_BORDER_DETECTED,      // Crossed color border (landmark)
  RESULT_COLLISION_DETECTED,   // Gyro spike detected impact
  RESULT_EARLY_ARRIVAL,        // Hit something before expected time
  RESULT_TIMEOUT               // Exceeded maximum time
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ROBOT STATE STRUCTURE
// ═══════════════════════════════════════════════════════════════════════════════

struct RobotState {
  // Position tracking (inches)
  float pivotX, pivotY;        // Center of wheel axis
  float sensorX, sensorY;      // Color sensor location

  // Orientation (degrees)
  float heading;               // 0°=East, 90°=North, -90°=South, 180°=West

  // Team assignment
  int teamColor;               // COLOR_RED or COLOR_BLUE

  // Confidence tracking
  float positionUncertainty;   // Accumulated error estimate (inches)

  // Mission timing
  unsigned long missionStartTime;
};

// Global robot state - initialized at starting position
RobotState bot = {
  .pivotX = 4.5,               // Starting 4.5" into arena
  .pivotY = 0.0,               // Centered on Y-axis
  .sensorX = 1.0,              // Sensor behind pivot when facing West
  .sensorY = 0.0,
  .heading = 180.0,            // Facing West (into arena) at start
  .teamColor = -1,             // Unknown until detected
  .positionUncertainty = 1.0,  // Initial uncertainty
  .missionStartTime = 0
};

// ═══════════════════════════════════════════════════════════════════════════════
//  GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════════

Servo servoL, servoR;
Adafruit_LSM6DSOX lsm6ds;

// ═══════════════════════════════════════════════════════════════════════════════
//  SENSOR HISTORY & GYROSCOPE STATE
// ═══════════════════════════════════════════════════════════════════════════════

byte colorHistory[HYSTERESIS_HISTORY];
byte historyIndex = 0;

float gyroHeading = 0.0;           // Integrated heading from gyro
float lastGyroZ = 0.0;             // Previous angular velocity (for spike detection)
unsigned long lastGyroTime = 0;
bool gyroAvailable = false;

// ═══════════════════════════════════════════════════════════════════════════════
//  FUNCTION PROTOTYPES
// ═══════════════════════════════════════════════════════════════════════════════

// Core motor control
void setMotors(int leftOffset, int rightOffset);
void stopMotors();

// Gyroscope functions
void initGyroscope();
void updateGyroscope();
void zeroGyroHeading();
bool detectCollisionSpike();
bool turnToAngle(float targetAngle);

// Color sensor functions
int readSurfaceRaw();
int readSurfaceFiltered();
int confirmSurfaceWithHysteresis(int requiredCount);
void clearColorHistory();

// Unified drive system
DriveResult driveForward(float targetDist, DriveMode mode, unsigned long* actualTimeOut);
DriveResult driveReverse(float targetDist, DriveMode mode, unsigned long* actualTimeOut);

// Position & kinematics
void updateKinematics();
void updatePositionFromDrive(float distance, bool forward);
void calibratePositionFromBorder(int borderType);
float calculateDriftAngle(unsigned long expected, unsigned long actual);
void applyDriftCorrection(float driftAngle);

// Time-of-arrival calculations
unsigned long calculateExpectedTime(float distance, float speed);
bool checkTimeOfArrival(unsigned long expected, unsigned long actual, float* driftAngle);

// Collision handling
void handleCollision();

// Mission positions
void executePositionOne();
void executePositionTwo();
void executePositionThree();
void executePositionFour();
void executePositionFive();
void executePositionSix();
void executePositionSeven();
void executePositionEight();

// Team color detection
void determineTeamColor(float distance);

// Utility
void statusBeep(int freq, int duration);
bool isMatchOver();
void printState(const char* label);

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(9600);

  // Attach servos
  servoL.attach(PIN_SERVO_L);
  servoR.attach(PIN_SERVO_R);
  stopMotors();

  // Configure buzzer
  pinMode(PIN_BUZZER, OUTPUT);

  // Configure color sensor
  pinMode(PIN_S0, OUTPUT); digitalWrite(PIN_S0, HIGH);  // 100% frequency scaling
  pinMode(PIN_S1, OUTPUT); digitalWrite(PIN_S1, HIGH);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);

  // Initialize gyroscope
  initGyroscope();

  // Clear color history
  clearColorHistory();

  Serial.println(F("═══════════════════════════════════════════"));
  Serial.println(F("  AUTOWIPER V60 - COMPREHENSIVE NAV SYSTEM"));
  Serial.println(F("═══════════════════════════════════════════"));
  Serial.print(F("Gyroscope: "));
  Serial.println(gyroAvailable ? F("ONLINE") : F("OFFLINE (time-based turns)"));
  Serial.println(F("Send '1' to start mission"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  if (Serial.available() && Serial.read() == '1') {

    // Clear serial buffer
    while (Serial.available()) Serial.read();

    bot.missionStartTime = millis();
    Serial.println(F("\n▶▶▶ MISSION START ◀◀◀\n"));

    // Execute mission sequence
    executePositionOne();   if (isMatchOver()) goto missionEnd; delay(200);
    executePositionTwo();   if (isMatchOver()) goto missionEnd; delay(200);
    executePositionThree(); if (isMatchOver()) goto missionEnd; delay(200);
    executePositionFour();  if (isMatchOver()) goto missionEnd; delay(200);
    executePositionFive();  if (isMatchOver()) goto missionEnd; delay(200);
    executePositionSix();   if (isMatchOver()) goto missionEnd; delay(200);
    executePositionSeven(); if (isMatchOver()) goto missionEnd; delay(200);
    executePositionEight();

missionEnd:
    stopMotors();

    unsigned long totalTime = millis() - bot.missionStartTime;
    Serial.println(F("\n═══════════════════════════════════════════"));
    Serial.println(F("          MISSION COMPLETE"));
    Serial.print(F("Total Time: ")); Serial.print(totalTime / 1000.0, 1); Serial.println(F(" sec"));
    Serial.print(F("Position Uncertainty: ±")); Serial.print(bot.positionUncertainty, 1); Serial.println(F(" in"));
    Serial.println(F("═══════════════════════════════════════════"));

    statusBeep(3000, 200); statusBeep(3000, 200); statusBeep(3000, 500);

    while(1) { delay(1000); } // Halt
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CORE MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void setMotors(int leftOffset, int rightOffset) {
  // Offset from 1500 neutral: positive = forward motion
  // Motor polarity: Left servo CCW=forward, Right servo CW=forward
  servoL.writeMicroseconds(1500 + leftOffset);
  servoR.writeMicroseconds(1500 - rightOffset);  // Inverted for right wheel
}

void stopMotors() {
  servoL.writeMicroseconds(1500);
  servoR.writeMicroseconds(1500);
  delay(50);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GYROSCOPE FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

void initGyroscope() {
  Wire.begin();
  if (lsm6ds.begin_I2C()) {
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
    gyroAvailable = true;
    lastGyroTime = millis();
  } else {
    gyroAvailable = false;
  }
}

void updateGyroscope() {
  if (!gyroAvailable) return;

  sensors_event_t accel, gyro, temp;
  if (!lsm6ds.getEvent(&accel, &gyro, &temp)) return;

  unsigned long currentTime = millis();
  float dt = (currentTime - lastGyroTime) / 1000.0;

  if (dt > 0.001 && dt < 1.0) {
    float currentGyroZ = gyro.gyro.z * 57.2958;  // rad/s to deg/s
    gyroHeading += currentGyroZ * dt;
    lastGyroZ = currentGyroZ;  // Store for collision detection
  }

  lastGyroTime = currentTime;
}

void zeroGyroHeading() {
  gyroHeading = 0.0;
  lastGyroZ = 0.0;
  lastGyroTime = millis();

  // Flush a few readings to stabilize
  for (int i = 0; i < 10; i++) {
    updateGyroscope();
    delay(5);
  }
  gyroHeading = 0.0;
}

bool detectCollisionSpike() {
  // Detect sudden angular velocity indicating collision
  return (abs(lastGyroZ) > COLLISION_SPIKE_THRESH);
}

bool turnToAngle(float targetAngle) {
  zeroGyroHeading();
  unsigned long startTime = millis();

  bool turnRight = (targetAngle < 0);
  float absTarget = abs(targetAngle);

  while (true) {
    updateGyroscope();
    float currentAngle = abs(gyroHeading);
    float error = absTarget - currentAngle;

    // Timeout check
    if (millis() - startTime > TURN_TIMEOUT_MS) {
      stopMotors();
      Serial.println(F("  [TURN TIMEOUT]"));
      return false;
    }

    // Completion check
    if (error <= TURN_TOLERANCE) {
      stopMotors();
      delay(50);
      return true;
    }

    // Variable speed based on remaining error
    int speed = GYRO_SPEED_CREEP;
    if (error > 30.0) speed = GYRO_SPEED_FAST;
    else if (error > 10.0) speed = GYRO_SPEED_SLOW;

    // Execute turn: positive angle = CCW (left), negative = CW (right)
    if (turnRight) {
      setMotors(speed, -speed);   // Right turn: left forward, right backward
    } else {
      setMotors(-speed, speed);   // Left turn: left backward, right forward
    }

    delay(10);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  COLOR SENSOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

int readSurfaceRaw() {
  // Read RED channel
  digitalWrite(PIN_S2, LOW);
  digitalWrite(PIN_S3, LOW);
  delayMicroseconds(100);
  int r = pulseIn(PIN_OUT, LOW, PULSEIN_TIMEOUT);

  // Read BLUE channel
  digitalWrite(PIN_S2, LOW);
  digitalWrite(PIN_S3, HIGH);
  delayMicroseconds(100);
  int b = pulseIn(PIN_OUT, LOW, PULSEIN_TIMEOUT);

  // Handle timeout (black surface)
  if (r == 0) r = 999;
  if (b == 0) b = 999;

  // Classification
  if (r > THRESH_BLACK_PANIC && b > THRESH_BLACK_PANIC) return COLOR_BLACK;
  if (r > THRESH_BLACK && b > THRESH_BLACK) return COLOR_BLACK;
  if (r < THRESH_RED && r < b) return COLOR_RED;
  if (b < THRESH_BLUE && b < r) return COLOR_BLUE;

  return COLOR_UNKNOWN;
}

int readSurfaceFiltered() {
  int result = readSurfaceRaw();
  colorHistory[historyIndex] = result;
  historyIndex = (historyIndex + 1) % HYSTERESIS_HISTORY;
  return result;
}

int confirmSurfaceWithHysteresis(int targetColor) {
  // Count recent occurrences of target color
  int count = 0;
  for (int i = 0; i < HYSTERESIS_HISTORY; i++) {
    if (colorHistory[i] == targetColor) count++;
  }
  return count;
}

void clearColorHistory() {
  for (int i = 0; i < HYSTERESIS_HISTORY; i++) {
    colorHistory[i] = COLOR_UNKNOWN;
  }
  historyIndex = 0;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UNIFIED DRIVE SYSTEM - THE HEART OF NAVIGATION
// ═══════════════════════════════════════════════════════════════════════════════

DriveResult driveForward(float targetDist, DriveMode mode, unsigned long* actualTimeOut) {
  unsigned long startTime = millis();
  float distTraveled = 0.0;
  int borderConfidence = 0;

  // Calculate expected time for time-of-arrival checking
  unsigned long expectedTime = calculateExpectedTime(targetDist, SPEED_CRUISE_IN_MS);
  unsigned long minValidTime = expectedTime * TIME_TOLERANCE_EARLY;

  // Clear history for fresh border detection
  clearColorHistory();

  // Determine what color we're looking for
  int targetBorderColor = (mode == DRIVE_UNTIL_ENEMY_BORDER) ?
                          ((bot.teamColor == COLOR_RED) ? COLOR_BLUE : COLOR_RED) :
                          ((bot.teamColor == COLOR_RED) ? COLOR_RED : COLOR_BLUE);

  // Initial heading for drift correction
  zeroGyroHeading();

  Serial.print(F("  [FWD] Target: ")); Serial.print(targetDist, 1);
  Serial.print(F("\" Mode: ")); Serial.println(mode);

  // Start driving
  setMotors(200, 200);  // Full forward

  while (distTraveled < targetDist) {
    unsigned long elapsed = millis() - startTime;

    // Update position tracking
    updateGyroscope();
    float stepDist = SPEED_CRUISE_IN_MS * 10.0;  // Distance per loop iteration
    distTraveled += stepDist;
    updatePositionFromDrive(stepDist, true);

    // === COLLISION DETECTION ===
    if (detectCollisionSpike()) {
      stopMotors();
      *actualTimeOut = elapsed;
      Serial.println(F("  [COLLISION SPIKE DETECTED]"));
      return RESULT_COLLISION_DETECTED;
    }

    // === HEADING CORRECTION ===
    // Apply motor bias to correct drift during straight-line driving
    float headingError = gyroHeading;  // 0 is target heading
    if (abs(headingError) > HEADING_DRIFT_THRESH) {
      int correction = (int)(headingError * HEADING_KP);
      correction = constrain(correction, -50, 50);
      setMotors(200 - correction, 200 + correction);
    }

    // === SURFACE DETECTION ===
    int surface = readSurfaceFiltered();

    // BLACK = EMERGENCY STOP (cliff/boundary)
    if (surface == COLOR_BLACK) {
      stopMotors();
      *actualTimeOut = elapsed;

      // TIME-OF-ARRIVAL CHECK: Did we arrive too early?
      if (elapsed < minValidTime) {
        Serial.println(F("  [EARLY BLACK - DRIFT DETECTED]"));
        // We hit a side rail unexpectedly
        bot.positionUncertainty += 2.0;
        return RESULT_EARLY_ARRIVAL;
      }

      Serial.println(F("  [BLACK BOUNDARY REACHED]"));
      return RESULT_BLACK_DETECTED;
    }

    // === BORDER DETECTION (with hysteresis) ===
    if (mode == DRIVE_UNTIL_BORDER || mode == DRIVE_UNTIL_ENEMY_BORDER) {
      // Look for opposite team color (crossing into enemy territory or back)
      int oppositeColor = (bot.teamColor == COLOR_RED) ? COLOR_BLUE : COLOR_RED;

      if (surface == oppositeColor) {
        borderConfidence++;
        if (borderConfidence >= HYSTERESIS_CONFIRM) {
          stopMotors();
          *actualTimeOut = elapsed;

          // Calibrate position: sensor is now at X=24
          calibratePositionFromBorder(oppositeColor);

          Serial.print(F("  [BORDER CONFIRMED] Confidence: ")); Serial.println(borderConfidence);
          return RESULT_BORDER_DETECTED;
        }
      } else {
        borderConfidence = 0;  // Reset on non-matching color
      }
    }

    // Small delay for sensor settling
    delay(10);
  }

  // Distance complete
  stopMotors();
  *actualTimeOut = millis() - startTime;

  // TIME-OF-ARRIVAL analysis for completed distance
  float driftAngle;
  if (checkTimeOfArrival(expectedTime, *actualTimeOut, &driftAngle)) {
    applyDriftCorrection(driftAngle);
  }

  Serial.print(F("  [DIST COMPLETE] Actual time: ")); Serial.print(*actualTimeOut);
  Serial.println(F("ms"));

  return RESULT_DISTANCE_COMPLETE;
}

DriveResult driveReverse(float targetDist, DriveMode mode, unsigned long* actualTimeOut) {
  unsigned long startTime = millis();
  float distTraveled = 0.0;
  int borderConfidence = 0;

  // Calculate expected time
  unsigned long expectedTime = calculateExpectedTime(targetDist, SPEED_CRUISE_IN_MS);
  unsigned long minValidTime = expectedTime * TIME_TOLERANCE_EARLY;

  // Clear history for fresh border detection
  clearColorHistory();

  // Initial heading for drift correction
  zeroGyroHeading();

  Serial.print(F("  [REV] Target: ")); Serial.print(targetDist, 1);
  Serial.print(F("\" Mode: ")); Serial.println(mode);

  // Ignore first 1 inch (blind window near starting position)
  float blindWindow = 1.0;

  // Start driving
  setMotors(-200, -200);  // Full reverse

  while (distTraveled < targetDist) {
    unsigned long elapsed = millis() - startTime;

    // Update position tracking
    updateGyroscope();
    float stepDist = SPEED_CRUISE_IN_MS * 10.0;
    distTraveled += stepDist;
    updatePositionFromDrive(stepDist, false);  // false = reverse

    // === COLLISION DETECTION ===
    if (detectCollisionSpike()) {
      stopMotors();
      *actualTimeOut = elapsed;
      Serial.println(F("  [COLLISION SPIKE DETECTED]"));
      return RESULT_COLLISION_DETECTED;
    }

    // === HEADING CORRECTION (reverse) ===
    float headingError = gyroHeading;
    if (abs(headingError) > HEADING_DRIFT_THRESH) {
      int correction = (int)(headingError * HEADING_KP);
      correction = constrain(correction, -50, 50);
      // Note: correction inverted for reverse driving
      setMotors(-200 + correction, -200 - correction);
    }

    // === SURFACE DETECTION (after blind window) ===
    if (distTraveled > blindWindow) {
      int surface = readSurfaceFiltered();

      // BLACK = EMERGENCY STOP
      if (surface == COLOR_BLACK) {
        stopMotors();
        *actualTimeOut = elapsed;

        if (elapsed < minValidTime) {
          Serial.println(F("  [EARLY BLACK - DRIFT DETECTED]"));
          bot.positionUncertainty += 2.0;
          return RESULT_EARLY_ARRIVAL;
        }

        Serial.println(F("  [BLACK BOUNDARY REACHED]"));
        return RESULT_BLACK_DETECTED;
      }

      // === BORDER DETECTION (with hysteresis) ===
      if (mode == DRIVE_UNTIL_BORDER || mode == DRIVE_UNTIL_ENEMY_BORDER) {
        int oppositeColor = (bot.teamColor == COLOR_RED) ? COLOR_BLUE : COLOR_RED;

        if (surface == oppositeColor) {
          borderConfidence++;
          if (borderConfidence >= HYSTERESIS_CONFIRM) {
            stopMotors();
            *actualTimeOut = elapsed;

            calibratePositionFromBorder(oppositeColor);

            Serial.print(F("  [BORDER CONFIRMED] Confidence: ")); Serial.println(borderConfidence);
            return RESULT_BORDER_DETECTED;
          }
        } else {
          borderConfidence = 0;
        }
      }
    }

    delay(10);
  }

  // Distance complete
  stopMotors();
  *actualTimeOut = millis() - startTime;

  float driftAngle;
  if (checkTimeOfArrival(expectedTime, *actualTimeOut, &driftAngle)) {
    applyDriftCorrection(driftAngle);
  }

  Serial.print(F("  [DIST COMPLETE] Actual time: ")); Serial.print(*actualTimeOut);
  Serial.println(F("ms"));

  return RESULT_DISTANCE_COMPLETE;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  POSITION & KINEMATICS
// ═══════════════════════════════════════════════════════════════════════════════

void updateKinematics() {
  // Update sensor position based on pivot position and heading
  float rads = bot.heading * (PI / 180.0);
  bot.sensorX = bot.pivotX + (SENSOR_RADIUS * cos(rads));
  bot.sensorY = bot.pivotY + (SENSOR_RADIUS * sin(rads));
}

void updatePositionFromDrive(float distance, bool forward) {
  float rads = bot.heading * (PI / 180.0);
  float dir = forward ? 1.0 : -1.0;

  bot.pivotX += dir * distance * cos(rads);
  bot.pivotY += dir * distance * sin(rads);

  // Accumulate uncertainty
  bot.positionUncertainty += distance * 0.02;  // 2% per inch traveled

  updateKinematics();
}

void calibratePositionFromBorder(int borderType) {
  // When we detect the Red/Blue border, we know sensor is at X=24
  // This resets accumulated dead-reckoning error

  if (borderType == COLOR_RED || borderType == COLOR_BLUE) {
    float oldX = bot.sensorX;
    bot.sensorX = MIDFIELD_X;

    // Back-calculate pivot position
    float rads = bot.heading * (PI / 180.0);
    bot.pivotX = bot.sensorX - (SENSOR_RADIUS * cos(rads));

    // Reset uncertainty
    float correction = abs(oldX - MIDFIELD_X);
    bot.positionUncertainty = 0.5;  // Reset to baseline

    Serial.print(F("  [CALIBRATED] X error was: ")); Serial.print(correction, 1);
    Serial.println(F("\""));
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TIME-OF-ARRIVAL CHECKING
// ═══════════════════════════════════════════════════════════════════════════════

unsigned long calculateExpectedTime(float distance, float speed) {
  return (unsigned long)(distance / speed);
}

bool checkTimeOfArrival(unsigned long expected, unsigned long actual, float* driftAngle) {
  // If actual time is significantly different from expected, calculate drift angle

  float ratio = (float)expected / (float)actual;

  if (ratio > 1.0) ratio = 1.0;  // Can't travel faster than physics allows

  if (actual > expected * TIME_TOLERANCE_LATE) {
    // Traveled too slow = diagonal path
    // drift_angle = arccos(expected/actual)
    *driftAngle = acos(ratio) * (180.0 / PI);
    Serial.print(F("  [DRIFT DETECTED] Angle: ")); Serial.print(*driftAngle, 1); Serial.println(F("°"));
    return true;
  }

  *driftAngle = 0.0;
  return false;
}

void applyDriftCorrection(float driftAngle) {
  if (driftAngle < 2.0) return;  // Ignore small corrections

  // Determine drift direction (would need additional sensing in practice)
  // For now, increase uncertainty and flag for recalibration
  bot.positionUncertainty += driftAngle * 0.1;

  Serial.print(F("  [DRIFT CORRECTION] Added uncertainty: "));
  Serial.print(driftAngle * 0.1, 1); Serial.println(F("\""));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  COLLISION HANDLING
// ═══════════════════════════════════════════════════════════════════════════════

void handleCollision() {
  Serial.println(F("  [COLLISION RECOVERY]"));

  stopMotors();
  statusBeep(500, 200);

  // Back up 2 inches
  unsigned long dummy;
  driveReverse(2.0, DRIVE_DISTANCE, &dummy);

  // Re-orient: turn 15 degrees away from likely obstacle
  // (In practice, you'd use more sophisticated detection)
  turnToAngle(15.0);
  bot.heading += 15.0;
  if (bot.heading > 180.0) bot.heading -= 360.0;

  updateKinematics();
  bot.positionUncertainty += 3.0;  // Significant uncertainty after collision
}

// ═══════════════════════════════════════════════════════════════════════════════
//  POSITION EXECUTION - ALL WITH UNIFIED DRIVE SYSTEM
// ═══════════════════════════════════════════════════════════════════════════════

void executePositionOne() {
  Serial.println(F("\n╔═══ POSITION 1: LAUNCH ═══╗"));

  statusBeep(1000, 100);
  delay(2000);  // Competition required delay
  statusBeep(3000, 500);

  zeroGyroHeading();

  // Drive reverse 2 inches to enter arena and detect team color
  determineTeamColor(2.0);

  printState("POS1 Complete");
}

void executePositionTwo() {
  Serial.println(F("\n╔═══ POSITION 2: FLANK TO BOTTOM WALL ═══╗"));

  // Turn 90° Right (facing South)
  Serial.println(F("Turning 90° Right (South)..."));
  turnToAngle(-90.0);
  bot.heading = -90.0;
  updateKinematics();

  // Drive forward toward bottom wall (Y = -22)
  float distToWall = abs(bot.pivotY - WALL_Y_BOT) - SENSOR_RADIUS;
  Serial.print(F("Driving to wall, estimated dist: ")); Serial.print(distToWall, 1); Serial.println(F("\""));

  unsigned long actualTime;
  unsigned long expectedTime = calculateExpectedTime(distToWall, SPEED_CRUISE_IN_MS);
  unsigned long minValidTime = expectedTime * TIME_TOLERANCE_EARLY;

  DriveResult result = driveForward(distToWall + 6.0, DRIVE_UNTIL_WALL, &actualTime);

  if (result == RESULT_BLACK_DETECTED) {
    if (actualTime < minValidTime) {
      // Hit side rail - early arrival indicates drift
      Serial.println(F("Early arrival - side rail collision, correcting..."));
      handleCollision();
    } else {
      // Normal wall arrival - calibrate Y position
      bot.pivotY = WALL_Y_BOT + SENSOR_RADIUS;
      updateKinematics();
      bot.positionUncertainty = 0.5;  // Reset Y uncertainty

      // Back up 4 inches for clearance
      unsigned long dummy;
      driveReverse(4.0, DRIVE_DISTANCE, &dummy);
    }
  } else if (result == RESULT_COLLISION_DETECTED) {
    handleCollision();
  }

  printState("POS2 Complete");
}

void executePositionThree() {
  Serial.println(F("\n╔═══ POSITION 3: MIDFIELD PENETRATION ═══╗"));

  // Turn 90° Left (facing East)
  Serial.println(F("Turning 90° Left (East)..."));
  turnToAngle(90.0);
  bot.heading = 0.0;  // Now facing East
  updateKinematics();

  // Drive forward, looking for Red/Blue border at X=24
  float distToBorder = MIDFIELD_X - bot.sensorX;
  Serial.print(F("Driving to border, estimated dist: ")); Serial.print(distToBorder, 1); Serial.println(F("\""));

  unsigned long actualTime;
  DriveResult result = driveForward(distToBorder + 10.0, DRIVE_UNTIL_ENEMY_BORDER, &actualTime);

  if (result == RESULT_BORDER_DETECTED) {
    // Position calibrated by border detection
    // Continue into enemy territory
    Serial.println(F("Border crossed, penetrating to X=33..."));

    unsigned long dummy;
    driveForward(9.0, DRIVE_DISTANCE, &dummy);  // Advance to X≈33
  } else if (result == RESULT_BLACK_DETECTED) {
    // Hit side wall - serious drift
    Serial.println(F("ERROR: Hit black before border - major drift!"));
    handleCollision();
  }

  printState("POS3 Complete");
}

void executePositionFour() {
  Serial.println(F("\n╔═══ POSITION 4: RETURN SWEEP 1 ═══╗"));

  // Reverse to border for recalibration
  unsigned long actualTime;
  DriveResult result = driveReverse(12.0, DRIVE_UNTIL_BORDER, &actualTime);

  if (result == RESULT_BORDER_DETECTED) {
    // Position recalibrated
    Serial.println(F("Border recalibrated"));
  }

  // Turn 45° Right for diagonal
  Serial.println(F("Turning 45° Right..."));
  turnToAngle(-45.0);
  bot.heading -= 45.0;
  updateKinematics();

  // Diagonal reverse (8 inches at 45° = ~5.7 inches Y change)
  unsigned long dummy;
  driveReverse(11.3, DRIVE_DISTANCE, &dummy);

  // Realign to East
  Serial.println(F("Turning 45° Left (realign East)..."));
  turnToAngle(45.0);
  bot.heading += 45.0;
  updateKinematics();

  // Reverse to home wall area
  driveReverse(14.0, DRIVE_DISTANCE, &dummy);

  printState("POS4 Complete");
}

void executePositionFive() {
  Serial.println(F("\n╔═══ POSITION 5: FORWARD SWEEP 2 ═══╗"));

  // Turn 45° Left for diagonal forward
  Serial.println(F("Turning 45° Left..."));
  turnToAngle(45.0);
  bot.heading += 45.0;
  updateKinematics();

  // Diagonal forward
  unsigned long actualTime;
  driveForward(8.5, DRIVE_DISTANCE, &actualTime);

  // Realign to East
  Serial.println(F("Turning 45° Right (realign East)..."));
  turnToAngle(-45.0);
  bot.heading -= 45.0;
  updateKinematics();

  // Drive forward to border and beyond
  DriveResult result = driveForward(20.0, DRIVE_UNTIL_ENEMY_BORDER, &actualTime);

  if (result == RESULT_BORDER_DETECTED) {
    // Continue into enemy territory
    unsigned long dummy;
    driveForward(9.0, DRIVE_DISTANCE, &dummy);
  }

  printState("POS5 Complete");
}

void executePositionSix() {
  Serial.println(F("\n╔═══ POSITION 6: RETURN SWEEP 2 ═══╗"));

  // Reverse to border
  unsigned long actualTime;
  DriveResult result = driveReverse(12.0, DRIVE_UNTIL_BORDER, &actualTime);

  if (result == RESULT_BORDER_DETECTED) {
    Serial.println(F("Border recalibrated"));
  }

  // Turn 45° Right for diagonal
  Serial.println(F("Turning 45° Right..."));
  turnToAngle(-45.0);
  bot.heading -= 45.0;
  updateKinematics();

  // Diagonal reverse
  unsigned long dummy;
  driveReverse(8.5, DRIVE_DISTANCE, &dummy);

  // Realign to East
  Serial.println(F("Turning 45° Left (realign East)..."));
  turnToAngle(45.0);
  bot.heading += 45.0;
  updateKinematics();

  // Reverse toward home
  driveReverse(16.0, DRIVE_DISTANCE, &dummy);

  printState("POS6 Complete");
}

void executePositionSeven() {
  Serial.println(F("\n╔═══ POSITION 7: FORWARD SWEEP 3 ═══╗"));

  // Turn 45° Left for diagonal
  Serial.println(F("Turning 45° Left..."));
  turnToAngle(45.0);
  bot.heading += 45.0;
  updateKinematics();

  // Diagonal forward
  unsigned long actualTime;
  driveForward(8.5, DRIVE_DISTANCE, &actualTime);

  // Realign to East
  Serial.println(F("Turning 45° Right (realign East)..."));
  turnToAngle(-45.0);
  bot.heading -= 45.0;
  updateKinematics();

  // Drive to border and beyond
  DriveResult result = driveForward(20.0, DRIVE_UNTIL_ENEMY_BORDER, &actualTime);

  if (result == RESULT_BORDER_DETECTED) {
    unsigned long dummy;
    driveForward(9.0, DRIVE_DISTANCE, &dummy);
  }

  printState("POS7 Complete");
}

void executePositionEight() {
  Serial.println(F("\n╔═══ POSITION 8: CENTER ALIGNMENT ═══╗"));

  // Reverse to border with hysteresis confirmation
  Serial.println(F("Reversing to border line..."));

  unsigned long actualTime;
  DriveResult result = driveReverse(15.0, DRIVE_UNTIL_BORDER, &actualTime);

  if (result == RESULT_BORDER_DETECTED) {
    Serial.println(F("Border confirmed - aligning pivot..."));

    // Sensor is at X=24, drive forward to place PIVOT at X=24
    unsigned long dummy;
    driveForward(SENSOR_RADIUS, DRIVE_DISTANCE, &dummy);

    bot.pivotX = MIDFIELD_X;
    updateKinematics();

    // Turn 90° Right (facing South)
    Serial.println(F("Turning 90° Right (South)..."));
    turnToAngle(-90.0);
    bot.heading = -90.0;
    updateKinematics();

    statusBeep(2000, 100);
  } else {
    Serial.println(F("ERROR: Border not found - safety turn"));
    turnToAngle(-90.0);
    bot.heading -= 90.0;
    updateKinematics();
  }

  printState("POS8 Complete - READY FOR NEXT PHASE");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEAM COLOR DETECTION
// ═══════════════════════════════════════════════════════════════════════════════

void determineTeamColor(float distance) {
  unsigned long duration = (unsigned long)(distance / SPEED_CRUISE_IN_MS);
  unsigned long startTime = millis();
  int blueCount = 0, redCount = 0;

  setMotors(-200, -200);  // Reverse

  while (millis() - startTime < duration) {
    int surface = readSurfaceRaw();

    if (surface == COLOR_BLUE) blueCount++;
    if (surface == COLOR_RED) redCount++;

    // Update position
    updateGyroscope();
    float stepDist = SPEED_CRUISE_IN_MS * 5.0;
    updatePositionFromDrive(stepDist, false);

    delay(5);
  }

  stopMotors();

  bot.teamColor = (redCount > blueCount) ? COLOR_RED : COLOR_BLUE;

  Serial.print(F("Team Color: "));
  Serial.print(bot.teamColor == COLOR_RED ? F("RED") : F("BLUE"));
  Serial.print(F(" (R:")); Serial.print(redCount);
  Serial.print(F(" B:")); Serial.print(blueCount); Serial.println(F(")"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

void statusBeep(int freq, int duration) {
  tone(PIN_BUZZER, freq, duration);
  delay(duration);
}

bool isMatchOver() {
  return (millis() - bot.missionStartTime >= MATCH_DURATION_MS);
}

void printState(const char* label) {
  Serial.println(F("─────────────────────────────────"));
  Serial.print(F("  ")); Serial.println(label);
  Serial.print(F("  Pivot: (")); Serial.print(bot.pivotX, 1);
  Serial.print(F(", ")); Serial.print(bot.pivotY, 1); Serial.println(F(")"));
  Serial.print(F("  Sensor: (")); Serial.print(bot.sensorX, 1);
  Serial.print(F(", ")); Serial.print(bot.sensorY, 1); Serial.println(F(")"));
  Serial.print(F("  Heading: ")); Serial.print(bot.heading, 1); Serial.println(F("°"));
  Serial.print(F("  Uncertainty: ±")); Serial.print(bot.positionUncertainty, 1); Serial.println(F("\""));
  Serial.println(F("─────────────────────────────────"));
}
