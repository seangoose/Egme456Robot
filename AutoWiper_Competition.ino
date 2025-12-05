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
#define PIN_COLOR_S3 A0  // Analog pin used as digital output

// IR Sensors (Opponent detection ONLY, not boundaries)
#define PIN_IR_LED_1 9
#define PIN_IR_RX_1 8
#define PIN_IR_LED_2 2
#define PIN_IR_RX_2 1

// Debug outputs
#define PIN_BUZZER 3
#define PIN_LED_1 6
#define PIN_LED_2 7

// ==================== SERVO POSITIONS ====================
// Arm positions
#define ARM_DEPLOYED 90      // Perpendicular to robot (12" width)
#define ARM_LEFT_RETRACT 153 // 90 + 63 = CCW toward BACK (4.75" width)
#define ARM_RIGHT_RETRACT 27 // 90 - 63 = CW toward BACK (4.75" width)

// Drive servo speeds (microseconds)
#define SERVO_STOP 1500
#define SERVO_FULL_SPEED 200  // Offset from neutral for full speed

// ==================== PHYSICAL CONSTANTS ====================
#define ROBOT_LENGTH 8.0          // inches (sensor to front)
#define ROBOT_WIDTH_RETRACTED 4.75 // inches (arms retracted)
#define ROBOT_WIDTH_DEPLOYED 12.0  // inches (arms deployed)
#define SWEEP_WIDTH 12.0           // inches per pass

#define FIELD_WIDTH 48.0           // inches
#define FIELD_LENGTH 48.0          // inches
#define BOUNDARY_WIDTH 2.0         // inches (black border)

// Safe operating boundaries for COLOR SENSOR position
#define MIN_SENSOR_Y 4.0   // Sensor 2" into field from back boundary
#define MAX_SENSOR_Y 40.0  // Robot front will be at 48" (40+8)

// Movement calibration
#define SPEED_INCHES_PER_SEC 9.0  // Typical BOE Shield-Bot speed
#define TURN_90_DEGREES_MS 590    // Time for 90° rotation at full speed
#define ODOMETRY_CORRECTION 0.95  // Correction factor for wheel slip

// Color sensor thresholds
#define BLACK_THRESHOLD 600       // Duration above this = black surface

// ==================== GLOBAL OBJECTS ====================
Servo servoLeft, servoRight;
Servo armLeft, armRight;
Adafruit_LSM6DSOX lsm6ds;

// ==================== GLOBAL STATE VARIABLES ====================
// Position tracking - COLOR SENSOR POSITION (at REAR of robot)
float colorSensor_X = 24.0;  // Start at center of field
float colorSensor_Y = 2.0;   // Start at back boundary
float heading = 0.0;         // Degrees (0 = forward into field)

// Derived position (robot extends 8" forward from sensor)
float getRobotFrontY() { return colorSensor_Y + ROBOT_LENGTH; }
float getRobotCenterY() { return colorSensor_Y + ROBOT_LENGTH/2.0; }

// Field identification
bool startedOnRedSide = false;
bool currentlyOnOwnSide = true;

// Strategy state
enum StrategyPhase { INIT, OFFENSIVE, DEFENSIVE };
StrategyPhase currentPhase = INIT;

// Sweep tracking
int currentLane = 1;         // Lanes 1-4
bool sweepingForward = true;
int completedOffensivePasses = 0;

// Arm state
bool armsDeployed = false;

// Timing
unsigned long startTime = 0;
unsigned long lastGyroUpdate = 0;

// Gyroscope integration
float gyroHeading = 0.0;

// Safety flags
bool positionUncertain = false;

// ==================== FUNCTION PROTOTYPES ====================
// Movement primitives
void maneuver(int speedLeft, int speedRight, int msTime);
void stopMotors();
float estimateDistance(int duration_ms, int speed);
void updatePositionFromMovement(int speedLeft, int speedRight, int duration_ms);

// Navigation
void moveForward(float distance_inches);
void moveBackward(float distance_inches);
void rotateLeft(float degrees);
void rotateRight(float degrees);
void rotateToHeading(float target_heading);
void navigateToCoordinate(float target_X, float target_Y);

// Sensor reading
unsigned long measureColorDuration(bool readRed);
bool detectBlackBoundary();
int checkFieldSide();
bool scanForOpponent();
void updateGyroscope();

// Arm control
void deployArms();
void retractArms();
void ensureArmsRetracted();

// Strategy execution
void calibrateCoordinateSystem();
void executePhase1_OffensiveSweep();
void executePhase2_DefensiveClearing();

// Error handling
void handleBoundaryEmergency();
void evadeOpponent();
void recoverPosition();

// Utilities
void statusBeep(int frequency, int duration);
float normalizeAngle(float angle);
int irDetect(int irLedPin, int irReceiverPin, long frequency);

// ==================== SETUP ====================
void setup() {
  // 1. Serial initialization
  #if DEBUG_MODE
    Serial.begin(9600);
    Serial.println("=== AUTOWIPER INITIALIZATION ===");
  #endif

  // 2. I2C for gyroscope
  Wire.begin();

  // 3. Attach servos
  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);
  armLeft.attach(PIN_ARM_LEFT);
  armRight.attach(PIN_ARM_RIGHT);

  // Stop drive motors initially
  stopMotors();

  // 4. Color sensor pins
  pinMode(PIN_COLOR_OUT, INPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);

  // 5. IR sensor pins (opponent detection only)
  pinMode(PIN_IR_LED_1, OUTPUT);
  pinMode(PIN_IR_RX_1, INPUT);
  pinMode(PIN_IR_LED_2, OUTPUT);
  pinMode(PIN_IR_RX_2, INPUT);

  // 6. Debug outputs
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);

  // 7. Initialize gyroscope
  if (!lsm6ds.begin_I2C()) {
    #if DEBUG_MODE
      Serial.println("ERROR: Gyroscope init failed");
    #endif
    // Continue anyway - can fall back to dead reckoning
  } else {
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    #if DEBUG_MODE
      Serial.println("Gyroscope initialized");
    #endif
  }

  // 8. MANDATORY 2-SECOND DELAY (competition requirement)
  statusBeep(3000, 500);
  digitalWrite(PIN_LED_1, HIGH);
  delay(2000);
  digitalWrite(PIN_LED_1, LOW);

  #if TEST_SENSORS_ONLY
    // Test mode - just display sensor readings
    Serial.println("=== SENSOR TEST MODE ===");
    Serial.println("Color Sensor: Lower value = more reflection");
    Serial.println("IR: 0=detected, 1=clear");
    Serial.println();

    while(true) {
      unsigned long red = measureColorDuration(true);
      unsigned long blue = measureColorDuration(false);
      bool black = detectBlackBoundary();
      bool opponent = scanForOpponent();

      // Determine surface color (INVERTED LOGIC for this sensor)
      String surfaceColor = "UNKNOWN";
      if(black) {
        surfaceColor = "BLACK";
      } else if(red > blue) {
        surfaceColor = "RED (red=" + String(red) + " > blue=" + String(blue) + ")";
      } else {
        surfaceColor = "BLUE (blue=" + String(blue) + " > red=" + String(red) + ")";
      }

      Serial.print("Red: "); Serial.print(red);
      Serial.print(" | Blue: "); Serial.print(blue);
      Serial.print(" | Surface: "); Serial.print(surfaceColor);
      Serial.print(" | IR: "); Serial.print(opponent ? "CLEAR" : "DETECTED");

      // Try to read gyroscope
      sensors_event_t accel, gyro, temp;
      if(lsm6ds.getEvent(&accel, &gyro, &temp)) {
        Serial.print(" | GyroZ: "); Serial.print(gyro.gyro.z, 3);
      } else {
        Serial.print(" | Gyro: FAIL");
      }

      Serial.println();
      delay(500);
    }
  #endif

  // 9. Determine starting field color (RED or BLUE side)
  delay(100);
  unsigned long red = measureColorDuration(true);
  unsigned long blue = measureColorDuration(false);

  if(red < 600 && blue < 600) {
    // On colored surface (not black boundary)
    // INVERTED: Higher duration = that color surface (counterintuitive but matches hardware)
    startedOnRedSide = (red > blue); // Red side if red shows higher duration
  } else {
    // Starting on black boundary - assume positioned correctly
    startedOnRedSide = false; // Default assumption
  }

  #if DEBUG_MODE
    Serial.print("Starting on ");
    Serial.println(startedOnRedSide ? "RED side" : "BLUE side");
  #endif

  // 10. Initial boundary calibration
  calibrateCoordinateSystem();

  // 11. Zero gyroscope heading
  gyroHeading = 0.0;
  heading = 0.0;
  lastGyroUpdate = millis();

  // 12. Retract arms to starting width (4.75")
  retractArms();

  // 13. Record start time
  startTime = millis();

  // 14. Ready indication
  statusBeep(2000, 200);
  digitalWrite(PIN_LED_1, HIGH);

  #if DEBUG_MODE
    Serial.println("=== INITIALIZATION COMPLETE ===");
    Serial.print("Position: X="); Serial.print(colorSensor_X);
    Serial.print(" SensorY="); Serial.print(colorSensor_Y);
    Serial.print(" FrontY="); Serial.println(getRobotFrontY());
  #endif
}

// ==================== MAIN LOOP ====================
void loop() {
  // 1. Update elapsed time
  unsigned long currentTime = millis();
  unsigned long elapsed = (currentTime - startTime) / 1000;

  // 2. Update sensors
  updateGyroscope();
  bool opponentDetected = scanForOpponent();
  bool atBoundary = detectBlackBoundary();

  // 3. Phase management
  if(elapsed < 5) {
    currentPhase = INIT;
  } else if(elapsed < 35) {
    currentPhase = OFFENSIVE;
  } else {
    currentPhase = DEFENSIVE; // Continues until 60 seconds
  }

  // 4. Time limit enforcement (60 seconds)
  if(elapsed >= 60) {
    stopMotors();
    digitalWrite(PIN_LED_2, HIGH);
    statusBeep(4000, 1000);

    #if DEBUG_MODE
      Serial.println("=== MATCH COMPLETE ===");
    #endif

    while(true) {
      delay(1000); // Match over - infinite loop
    }
  }

  // 5. Emergency boundary handling (HIGHEST PRIORITY)
  if(atBoundary) {
    bool expectedBoundary = false;

    // Check if boundary detection is expected
    if(colorSensor_Y < 6.0 && currentPhase == INIT) {
      expectedBoundary = true; // Calibration phase
    } else if(getRobotFrontY() > 45.0 && currentPhase == OFFENSIVE) {
      expectedBoundary = true; // Deep offensive push
    }

    if(!expectedBoundary) {
      #if DEBUG_MODE
        Serial.println("EMERGENCY: Unexpected boundary!");
      #endif
      handleBoundaryEmergency();
      return;
    }
  }

  // 6. Opponent avoidance (HIGH PRIORITY)
  if(opponentDetected && currentPhase != INIT) {
    #if DEBUG_MODE
      Serial.println("Opponent detected - evading");
    #endif
    evadeOpponent();
    return;
  }

  // 7. Execute strategy phase
  switch(currentPhase) {
    case INIT:
      // Initialization should be complete, prepare for offensive
      if(elapsed > 4) {
        currentLane = 1;
        completedOffensivePasses = 0;

        #if DEBUG_MODE
          Serial.println("Moving to OFFENSIVE phase");
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

  // 8. Position verification (every 3 seconds)
  if(elapsed % 3 == 0 && elapsed > 0) {
    int fieldSide = checkFieldSide();
    if(fieldSide != 0) {
      bool expectedOnOwnSide = (colorSensor_Y < 24.0);
      bool actuallyOnOwnSide = (fieldSide == 1);

      if(expectedOnOwnSide != actuallyOnOwnSide) {
        positionUncertain = true;
        #if DEBUG_MODE
          Serial.println("WARNING: Position uncertain");
        #endif
      }
    }
  }

  // 9. Debug output
  #if DEBUG_MODE
    static unsigned long lastDebug = 0;
    if(currentTime - lastDebug > 1000) {
      Serial.print("T:"); Serial.print(elapsed);
      Serial.print(" | Phase:"); Serial.print(currentPhase);
      Serial.print(" | SensorY:"); Serial.print(colorSensor_Y, 1);
      Serial.print(" | FrontY:"); Serial.print(getRobotFrontY(), 1);
      Serial.print(" | X:"); Serial.print(colorSensor_X, 1);
      Serial.print(" | H:"); Serial.print(heading, 1);

      // Add gyroscope reading
      sensors_event_t accel, gyro, temp;
      if(lsm6ds.getEvent(&accel, &gyro, &temp)) {
        Serial.print(" | GyroZ:"); Serial.print(gyro.gyro.z, 2);
      }

      // Add color sensor reading
      unsigned long red = measureColorDuration(true);
      unsigned long blue = measureColorDuration(false);
      Serial.print(" | R:"); Serial.print(red);
      Serial.print(" | B:"); Serial.print(blue);

      Serial.println();
      lastDebug = currentTime;
    }
  #endif

  delay(20); // 20ms loop cycle
}

// ==================== MOVEMENT PRIMITIVES ====================

void maneuver(int speedLeft, int speedRight, int msTime) {
  // Speed range: -200 to +200
  // Positive = forward, Negative = backward, 0 = stop

  servoLeft.writeMicroseconds(SERVO_STOP + speedLeft);
  servoRight.writeMicroseconds(SERVO_STOP - speedRight);

  if(msTime > 0) {
    delay(msTime);
  } else if(msTime == -1) {
    // Disable servos
    servoLeft.detach();
    servoRight.detach();
  }
}

void stopMotors() {
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  delay(50);
}

float estimateDistance(int duration_ms, int speed) {
  // Estimate distance traveled based on duration and speed
  // speed is in range -200 to +200
  float speedFraction = abs(speed) / 200.0;
  float distance = (SPEED_INCHES_PER_SEC * speedFraction * duration_ms / 1000.0);
  return distance * ODOMETRY_CORRECTION; // Apply correction for wheel slip
}

void updatePositionFromMovement(int speedLeft, int speedRight, int duration_ms) {
  // Update position based on movement
  float distance = estimateDistance(duration_ms, (speedLeft + speedRight) / 2);

  // Convert heading to radians
  float radians = heading * PI / 180.0;

  // Update sensor position (at rear of robot)
  colorSensor_X += distance * sin(radians);
  colorSensor_Y += distance * cos(radians);

  // Clamp to field boundaries
  if(colorSensor_X < 0) colorSensor_X = 0;
  if(colorSensor_X > FIELD_WIDTH) colorSensor_X = FIELD_WIDTH;
  if(colorSensor_Y < 0) colorSensor_Y = 0;
  if(colorSensor_Y > FIELD_LENGTH) colorSensor_Y = FIELD_LENGTH;
}

// ==================== NAVIGATION ====================

void moveForward(float distance_inches) {
  int duration = (int)(distance_inches / SPEED_INCHES_PER_SEC * 1000.0);

  // Move with gyroscope correction
  unsigned long startMove = millis();
  float targetHeading = heading;

  while(millis() - startMove < duration) {
    updateGyroscope();

    // Calculate heading error
    float headingError = normalizeAngle(heading - targetHeading);

    // Apply correction
    int correction = (int)(headingError * 2.5);
    correction = constrain(correction, -50, 50);

    maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 50);
    updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 50);

    // Safety check for boundary
    if(detectBlackBoundary() && getRobotFrontY() > 45.0) {
      #if DEBUG_MODE
        Serial.println("Boundary detected during forward movement");
      #endif
      break;
    }
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

  heading -= degrees;
  heading = normalizeAngle(heading);

  stopMotors();
}

void rotateRight(float degrees) {
  int duration = (int)(degrees / 90.0 * TURN_90_DEGREES_MS);

  maneuver(SERVO_FULL_SPEED, -SERVO_FULL_SPEED, duration);

  heading += degrees;
  heading = normalizeAngle(heading);

  stopMotors();
}

void rotateToHeading(float target_heading) {
  target_heading = normalizeAngle(target_heading);
  float error = normalizeAngle(target_heading - heading);

  if(abs(error) < 3.0) return; // Within tolerance

  if(error > 0) {
    rotateRight(abs(error));
  } else {
    rotateLeft(abs(error));
  }
}

void navigateToCoordinate(float target_X, float target_Y) {
  // Ensure arms are retracted for navigation
  ensureArmsRetracted();

  // Calculate required movement
  float deltaX = target_X - colorSensor_X;
  float deltaY = target_Y - colorSensor_Y;

  // Calculate required heading and distance
  float targetHeading = atan2(deltaX, deltaY) * 180.0 / PI;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  #if DEBUG_MODE
    Serial.print("Navigate: Current("); Serial.print(colorSensor_X, 1);
    Serial.print(","); Serial.print(colorSensor_Y, 1);
    Serial.print(") -> Target("); Serial.print(target_X, 1);
    Serial.print(","); Serial.print(target_Y, 1);
    Serial.print(") Dist:"); Serial.print(distance, 1);
    Serial.print(" Head:"); Serial.println(targetHeading, 1);
  #endif

  // Rotate to target heading
  rotateToHeading(targetHeading);
  delay(200);

  // Move forward
  moveForward(distance);
  delay(200);
}

// ==================== SENSOR READING ====================

unsigned long measureColorDuration(bool readRed) {
  if(readRed) {
    // Select RED photodiodes
    digitalWrite(PIN_COLOR_S2, LOW);
    digitalWrite(PIN_COLOR_S3, LOW);
  } else {
    // Select BLUE photodiodes
    digitalWrite(PIN_COLOR_S2, LOW);
    digitalWrite(PIN_COLOR_S3, HIGH);
  }

  delay(50); // Stabilization delay

  // Read frequency as pulse duration
  unsigned long duration = pulseIn(PIN_COLOR_OUT, LOW, 50000);

  return duration;
}

bool detectBlackBoundary() {
  unsigned long red = measureColorDuration(true);
  unsigned long blue = measureColorDuration(false);

  // Black border: BOTH colors show very low intensity (high duration)
  bool isBlack = (red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD);

  return isBlack;
}

int checkFieldSide() {
  // Returns: 1 = own side, -1 = opponent side, 0 = boundary/uncertain

  if(detectBlackBoundary()) {
    return 0; // At boundary, can't determine side
  }

  unsigned long red = measureColorDuration(true);
  unsigned long blue = measureColorDuration(false);

  // INVERTED: Higher duration = that color surface (matches hardware behavior)
  bool readingRed = (red > blue);

  // Compare with starting field color
  if(readingRed == startedOnRedSide) {
    return 1; // On own side
  } else {
    return -1; // On opponent's side
  }
}

bool scanForOpponent() {
  // IR sensors detect opponent robot ONLY (not boundaries)

  int frontLeft = irDetect(PIN_IR_LED_1, PIN_IR_RX_1, 38000);
  int frontRight = irDetect(PIN_IR_LED_2, PIN_IR_RX_2, 38000);

  // IR returns 0 when object detected
  return (frontLeft == 0 || frontRight == 0);
}

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency, 8); // Emit IR for 8ms
  delay(1);
  int ir = digitalRead(irReceiverPin);
  delay(1);
  return ir; // 0 = detection, 1 = no detection
}

void updateGyroscope() {
  sensors_event_t accel, gyro, temp;

  if(!lsm6ds.getEvent(&accel, &gyro, &temp)) {
    return; // Failed to read
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - lastGyroUpdate) / 1000.0; // Convert to seconds

  if(dt > 0.001 && dt < 1.0) { // Sanity check
    float gyroZ = gyro.gyro.z; // Z-axis rotation in rad/s

    // Integrate to get heading
    heading += gyroZ * dt * (180.0 / PI); // Convert to degrees
    heading = normalizeAngle(heading);
  }

  lastGyroUpdate = currentTime;
}

// ==================== ARM CONTROL ====================

void deployArms() {
  if(armsDeployed) return; // Already deployed

  armLeft.write(ARM_DEPLOYED);
  armRight.write(ARM_DEPLOYED);
  delay(500); // Allow deployment

  armsDeployed = true;

  #if DEBUG_MODE
    Serial.println("Arms DEPLOYED - 12\" width");
  #endif
}

void retractArms() {
  if(!armsDeployed && armLeft.read() == ARM_LEFT_RETRACT) {
    return; // Already retracted
  }

  armLeft.write(ARM_LEFT_RETRACT);   // 153° (CCW toward back)
  armRight.write(ARM_RIGHT_RETRACT); // 27° (CW toward back)
  delay(500); // Allow retraction

  armsDeployed = false;

  #if DEBUG_MODE
    Serial.println("Arms RETRACTED - 4.75\" width");
  #endif
}

void ensureArmsRetracted() {
  if(armsDeployed) {
    retractArms();
  }
}

// ==================== STRATEGY EXECUTION ====================

void calibrateCoordinateSystem() {
  #if DEBUG_MODE
    Serial.println("Calibrating coordinate system...");
  #endif

  // Robot should be at back boundary
  // Back up slowly until black is definitively detected

  bool blackDetected = false;
  int attempts = 0;

  while(!blackDetected && attempts < 10) {
    if(detectBlackBoundary()) {
      blackDetected = true;
      break;
    }

    // Back up slightly
    maneuver(-100, -100, 100); // Slow 100ms reverse
    colorSensor_Y -= estimateDistance(100, 100);
    attempts++;
    delay(50);
  }

  if(blackDetected) {
    // Lock coordinate system
    colorSensor_Y = 2.0; // Sensor at boundary edge

    #if DEBUG_MODE
      Serial.print("Calibration complete. SensorY=");
      Serial.print(colorSensor_Y);
      Serial.print(" FrontY=");
      Serial.println(getRobotFrontY());
    #endif
  } else {
    // Calibration failed - use starting assumption
    colorSensor_Y = 2.0;

    #if DEBUG_MODE
      Serial.println("WARNING: Calibration failed, using default");
    #endif
  }

  // Move slightly forward to clear boundary
  maneuver(150, 150, 300);
  colorSensor_Y += estimateDistance(300, 150);
  stopMotors();
}

void executePhase1_OffensiveSweep() {
  // Execute 4 sweeping passes across opponent's territory
  // Lanes at X = 6", 18", 30", 42"

  if(completedOffensivePasses >= 4) {
    return; // All offensive passes complete
  }

  static bool inPosition = false;
  static bool sweepComplete = false;

  if(!inPosition) {
    // Navigate to lane start position
    float laneX = 6.0 + (completedOffensivePasses * 12.0); // 6, 18, 30, 42
    float startY = 16.0; // Start position (clear of back boundary)

    #if DEBUG_MODE
      Serial.print("Phase 1: Moving to lane ");
      Serial.print(completedOffensivePasses + 1);
      Serial.print(" at X="); Serial.println(laneX);
    #endif

    navigateToCoordinate(laneX, startY);
    rotateToHeading(0.0); // Face forward

    deployArms(); // Deploy for sweeping

    inPosition = true;
    sweepComplete = false;
    delay(300);
  }

  if(inPosition && !sweepComplete) {
    // Execute forward sweep to opponent's territory
    // Target: sensor at Y=38" (front at Y=46", safe from boundary)

    #if DEBUG_MODE
      Serial.println("Executing offensive sweep...");
    #endif

    while(colorSensor_Y < 38.0) {
      // Check for opponent
      if(scanForOpponent()) {
        statusBeep(4000, 100);
        evadeOpponent();
        break;
      }

      // Check for boundary (safety)
      if(detectBlackBoundary()) {
        #if DEBUG_MODE
          Serial.println("Boundary detected - stopping sweep");
        #endif
        break;
      }

      // Move forward with correction
      updateGyroscope();
      float headingError = normalizeAngle(heading - 0.0);
      int correction = (int)(headingError * 2.5);
      correction = constrain(correction, -50, 50);

      maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 100);
      updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 100);
    }

    sweepComplete = true;
    stopMotors();

    #if DEBUG_MODE
      Serial.println("Sweep complete - returning to midfield");
    #endif

    // Retract arms and return to midfield
    retractArms();
    moveBackward(15.0); // Back to around Y=24" (midfield)

    // Complete this pass
    completedOffensivePasses++;
    inPosition = false;

    #if DEBUG_MODE
      Serial.print("Completed offensive pass ");
      Serial.print(completedOffensivePasses);
      Serial.println(" of 4");
    #endif

    delay(200);
  }
}

void executePhase2_DefensiveClearing() {
  // Continuous serpentine sweeping on own side until match end
  // Sweep forward → shift → backward → shift → repeat

  static bool isInitialized = false;

  if(!isInitialized) {
    // Initialize defensive phase
    currentLane = 1;
    sweepingForward = true;

    #if DEBUG_MODE
      Serial.println("Phase 2: Defensive clearing initiated");
    #endif

    // Navigate to starting position (lane 1, own side)
    navigateToCoordinate(6.0, 12.0);
    rotateToHeading(0.0);
    deployArms();

    isInitialized = true;
  }

  // Execute continuous sweeping
  if(sweepingForward) {
    // Forward sweep from Y~12" to Y~22" (stay on own side)

    while(colorSensor_Y < 22.0 && !detectBlackBoundary()) {
      if(scanForOpponent()) {
        evadeOpponent();
        return;
      }

      updateGyroscope();
      float headingError = normalizeAngle(heading - 0.0);
      int correction = (int)(headingError * 2.5);
      correction = constrain(correction, -50, 50);

      maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 100);
      updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 100);

      if(detectBlackBoundary()) break;
    }

    stopMotors();

    // Shift to next lane
    currentLane++;
    if(currentLane > 4) currentLane = 1;

    float nextLaneX = 6.0 + ((currentLane - 1) * 12.0);

    retractArms();
    navigateToCoordinate(nextLaneX, colorSensor_Y);
    rotateToHeading(180.0); // Face backward for reverse sweep
    deployArms();

    sweepingForward = false;

  } else {
    // Backward sweep from Y~22" to Y~8"

    while(colorSensor_Y > 8.0 && !detectBlackBoundary()) {
      if(scanForOpponent()) {
        evadeOpponent();
        return;
      }

      updateGyroscope();
      float headingError = normalizeAngle(heading - 180.0);
      int correction = (int)(headingError * 2.5);
      correction = constrain(correction, -50, 50);

      maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 100);
      updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 100);

      if(detectBlackBoundary()) break;
    }

    stopMotors();

    // Shift to next lane
    currentLane++;
    if(currentLane > 4) currentLane = 1;

    float nextLaneX = 6.0 + ((currentLane - 1) * 12.0);

    retractArms();
    navigateToCoordinate(nextLaneX, colorSensor_Y);
    rotateToHeading(0.0); // Face forward for forward sweep
    deployArms();

    sweepingForward = true;
  }
}

// ==================== ERROR HANDLING ====================

void handleBoundaryEmergency() {
  stopMotors();
  statusBeep(500, 300); // Low warning tone

  #if DEBUG_MODE
    Serial.print("BOUNDARY EMERGENCY at SensorY=");
    Serial.println(colorSensor_Y);
  #endif

  if(getRobotFrontY() > 45.0) {
    // Near front boundary - reverse
    moveBackward(6.0);
  } else if(colorSensor_Y < 6.0) {
    // Near back boundary - move forward
    moveForward(4.0);
  } else {
    // Unexpected boundary in middle - back away
    moveBackward(4.0);
  }

  // Re-verify position
  delay(200);
}

void evadeOpponent() {
  stopMotors();
  statusBeep(4000, 100); // High alert tone

  #if DEBUG_MODE
    Serial.println("Evading opponent...");
  #endif

  // Retract arms for maneuvering
  retractArms();

  // Determine evasion direction
  float distToLeftEdge = colorSensor_X - 6.0;
  float distToRightEdge = 42.0 - colorSensor_X;

  if(distToLeftEdge > distToRightEdge) {
    // Evade LEFT
    moveBackward(4.0);
    rotateLeft(90.0);
    moveForward(10.0);
    rotateRight(90.0);
  } else {
    // Evade RIGHT
    moveBackward(4.0);
    rotateRight(90.0);
    moveForward(10.0);
    rotateLeft(90.0);
  }

  delay(300);
}

// ==================== UTILITY FUNCTIONS ====================

void statusBeep(int frequency, int duration) {
  tone(PIN_BUZZER, frequency, duration);
  delay(duration);
}

float normalizeAngle(float angle) {
  // Normalize to -180 to +180 range
  while(angle > 180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}
