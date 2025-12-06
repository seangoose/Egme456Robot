/* ═══════════════════════════════════════════════════════════════════════════════
   AUTOWIPER COMPETITION CODE
   EGME 456 - Cal State Fullerton Messy Room Competition
   ═══════════════════════════════════════════════════════════════════════════════

   OBJECTIVE: Clear foam cubes from our half of 48"×48" field in 60 seconds.
   VICTORY: Fewer cubes on our side than opponent at time expiration.

   STRATEGY:
   - Phase 1: Detect which color (RED/BLUE) is our starting side
   - Phase 2: Navigate to right side, turn to face opponent
   - Phase 3: Execute multi-lane offensive sweeps pushing cubes into opponent territory

   CRITICAL DESIGN DECISIONS:
   - Position tracked in SERVO TURN UNITS (time-based), not theoretical inches
   - NO reliance on gyroscope (use calibrated turn timing instead)
   - Continuous boundary monitoring in ALL movement functions
   - Separate calibration constants for left and right turns
   - Color sensor is PRIMARY navigation tool

   HARDWARE CONFIGURATION (VERIFIED - DO NOT CHANGE):
   - D13: Left drive servo (Parallax continuous rotation)
   - D12: Right drive servo (Parallax continuous rotation)
   - D11: Left arm servo (Standard hobby)
   - D10: Right arm servo (Standard hobby)
   - D4:  TCS3200 OUT (frequency output)
   - D5:  TCS3200 S2 (filter select bit 0)
   - A0:  TCS3200 S3 (filter select bit 1, as digital output)
   - D8:  HC-SR04 Ultrasonic (3-pin version, trigger/echo on same wire)
   - D3:  Piezo buzzer
   - D6:  LED 1
   - D7:  LED 2

   CRITICAL: Color sensor mounted at REAR of robot.
             Robot extends 8" FORWARD from sensor position.
   ═══════════════════════════════════════════════════════════════════════════════ */

#include <Servo.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  DEBUG CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════

#define DEBUG_MODE false   // Set true for Serial output during testing

#if DEBUG_MODE
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
  #define DEBUG_PRINTF(x,y) Serial.print(x); Serial.println(y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x,y)
#endif

// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATED CONSTANTS - FROM CALIBRATION TOOL
//  Run AutoWiper_Calibration.ino and copy measured values here
// ═══════════════════════════════════════════════════════════════════════════════

// Distance calibration (inches traveled in 1000ms)
#define DISTANCE_PER_1000MS_FULL_SPEED  20.0   // Measure with calibration tool
#define DISTANCE_PER_1000MS_HALF_SPEED  10.0   // Measure with calibration tool

// Turn calibration (on-dime turns) - SEPARATE LEFT AND RIGHT
#define TURN_LEFT_90_MS   650    // Time for 90° left turn at full speed
#define TURN_RIGHT_90_MS  650    // Time for 90° right turn at full speed

// Traveling turn calibration
#define TRAVELING_TURN_LEFT_SPEED   200   // Outer wheel (faster)
#define TRAVELING_TURN_RIGHT_SPEED  80    // Inner wheel (slower)
#define TRAVELING_TURN_90_DURATION  1200  // Duration for 90° traveling turn

// Color sensor thresholds
#define BLACK_THRESHOLD  600   // Both red AND blue > this = black boundary

// Ultrasonic sensor
#define OBSTACLE_DISTANCE_THRESHOLD  12.0  // inches - trigger avoidance

// ═══════════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS (VERIFIED - DO NOT CHANGE)
// ═══════════════════════════════════════════════════════════════════════════════

// Drive Servos (Parallax continuous rotation)
#define PIN_SERVO_LEFT    13
#define PIN_SERVO_RIGHT   12

// Arm Servos (Standard hobby servos)
#define PIN_ARM_LEFT      11
#define PIN_ARM_RIGHT     10

// TCS3200 Color Sensor
#define PIN_COLOR_OUT     4
#define PIN_COLOR_S2      5
#define PIN_COLOR_S3      A0   // Analog pin used as digital output

// HC-SR04 Ultrasonic (3-pin version)
#define PIN_ULTRASONIC    8

// Debug outputs
#define PIN_BUZZER        3
#define PIN_LED_1         6
#define PIN_LED_2         7

// ═══════════════════════════════════════════════════════════════════════════════
//  SERVO CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define SERVO_STOP        1500   // Neutral position (stopped)
#define SPEED_OFFSET      200    // Offset from neutral for full speed

// Arm positions (degrees)
#define ARM_CENTER        90     // Perpendicular to robot (sweeping)
#define ARM_LEFT_RETRACT  152    // 90 + 62 = CCW toward BACK
#define ARM_RIGHT_RETRACT 28     // 90 - 62 = CW toward BACK
#define ARM_LEFT_FORWARD  30     // 90 - 60 = toward FRONT (cube manipulation)
#define ARM_RIGHT_FORWARD 150    // 90 + 60 = toward FRONT

// ═══════════════════════════════════════════════════════════════════════════════
//  FIELD CONSTANTS (in UNITS = 1000ms at full speed)
// ═══════════════════════════════════════════════════════════════════════════════

// Field dimensions converted to time units
// 1 UNIT = distance traveled in 1000ms at full speed
// Example: if DISTANCE_PER_1000MS_FULL_SPEED = 20", then 48" = 2.4 units = 2400ms

#define FIELD_WIDTH_INCHES   48.0
#define FIELD_LENGTH_INCHES  48.0
#define ROBOT_LENGTH_INCHES  8.0

// Calculate field dimensions in milliseconds (at full speed)
#define MS_PER_INCH          (1000.0 / DISTANCE_PER_1000MS_FULL_SPEED)
#define FIELD_WIDTH_MS       ((unsigned int)(FIELD_WIDTH_INCHES * MS_PER_INCH))
#define FIELD_LENGTH_MS      ((unsigned int)(FIELD_LENGTH_INCHES * MS_PER_INCH))
#define ROBOT_LENGTH_MS      ((unsigned int)(ROBOT_LENGTH_INCHES * MS_PER_INCH))

// Midfield position (in ms from back boundary)
#define MIDFIELD_MS          (FIELD_LENGTH_MS / 2)

// Color sensor reading constants
#define COLOR_STABILIZE_MS   15      // Stabilization delay after filter change
#define PULSEIN_TIMEOUT      25000   // Timeout for pulseIn (25ms)

// Safety margins (in ms)
#define BOUNDARY_SAFETY_MS   200     // Stay this far from black boundary

// ═══════════════════════════════════════════════════════════════════════════════
//  GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════════

Servo servoLeft, servoRight;
Servo armLeft, armRight;

// ═══════════════════════════════════════════════════════════════════════════════
//  POSITION TRACKING (in milliseconds of travel at full speed)
// ═══════════════════════════════════════════════════════════════════════════════

// Color sensor position (at REAR of robot) in ms units
int colorSensor_X_ms = 0;    // 0 = left edge, FIELD_WIDTH_MS = right edge
int colorSensor_Y_ms = 0;    // 0 = back boundary, FIELD_LENGTH_MS = front

// Heading: 0° = facing forward (into opponent territory)
//          90° = facing right, -90° = facing left, 180° = facing back
int heading_deg = 0;

// Robot front position = colorSensor_Y_ms + ROBOT_LENGTH_MS (when facing forward)

// ═══════════════════════════════════════════════════════════════════════════════
//  STATE TRACKING
// ═══════════════════════════════════════════════════════════════════════════════

// Field identification
enum FieldColor { COLOR_UNKNOWN, COLOR_RED, COLOR_BLUE };
FieldColor ownSideColor = COLOR_UNKNOWN;

// Strategy phases
enum Phase {
  PHASE_INIT,           // 2-second delay, side detection
  PHASE_NAVIGATE,       // Navigate to starting sweep position
  PHASE_SWEEP,          // Execute offensive sweeps
  PHASE_COMPLETE        // Match complete
};
Phase currentPhase = PHASE_INIT;

// Sweep tracking
int currentLane = 1;           // Lanes 1-4 from right to left
int completedSweeps = 0;
bool sweepingForward = true;

// Arm state
bool armsDeployed = false;

// Timing
unsigned long matchStartTime = 0;
#define MATCH_DURATION_MS  60000   // 60 seconds

// Emergency flags
bool emergencyStop = false;

// ═══════════════════════════════════════════════════════════════════════════════
//  FUNCTION PROTOTYPES
// ═══════════════════════════════════════════════════════════════════════════════

// Initialization
void initializeSensors();
void detectOwnSideColor();

// Movement primitives (with continuous boundary checking)
void maneuver(int speedLeft, int speedRight, unsigned int duration_ms);
void stopMotors();
bool moveForwardMs(unsigned int distance_ms);
bool moveBackwardMs(unsigned int distance_ms);
void turnOnDimeLeft();
void turnOnDimeRight();
void travelingTurnRight();
void travelingTurnLeft();

// Sensor reading
unsigned long measureColorDuration(bool readRed);
bool detectBlackBoundary();
bool detectOwnSideColorNow();
float measureUltrasonicDistance();
bool detectObstacle();

// Arm control
void deployArms();
void deployArmsForward();
void retractArms();

// Strategy execution
void executePhase_Navigate();
void executePhase_Sweep();
void executeSingleSweepLane();
void returnToOwnSide();

// Emergency handling
void handleBoundaryEmergency();
void handleObstacleAvoidance();

// Utilities
void statusBeep(int frequency, int duration);
void blinkLED(int pin, int count);
unsigned long getMatchElapsed();
bool isMatchOver();

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  #if DEBUG_MODE
    Serial.begin(9600);
    Serial.println(F("=== AUTOWIPER COMPETITION ==="));
  #endif

  // ─────────────────────────────────────────────────────────────────────────────
  // Attach servos
  // ─────────────────────────────────────────────────────────────────────────────
  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);
  armLeft.attach(PIN_ARM_LEFT);
  armRight.attach(PIN_ARM_RIGHT);

  // Stop drive motors
  stopMotors();

  // ─────────────────────────────────────────────────────────────────────────────
  // Initialize pins
  // ─────────────────────────────────────────────────────────────────────────────
  pinMode(PIN_COLOR_OUT, INPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);
  pinMode(PIN_ULTRASONIC, OUTPUT);
  digitalWrite(PIN_ULTRASONIC, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);

  // ─────────────────────────────────────────────────────────────────────────────
  // MANDATORY 2-SECOND START DELAY (Competition Requirement)
  // ─────────────────────────────────────────────────────────────────────────────
  digitalWrite(PIN_LED_1, HIGH);
  statusBeep(2000, 500);

  DEBUG_PRINTLN(F("Waiting 2 seconds..."));

  delay(2000);  // REQUIRED BY COMPETITION RULES

  digitalWrite(PIN_LED_1, LOW);
  statusBeep(3000, 200);

  // ─────────────────────────────────────────────────────────────────────────────
  // Record match start time
  // ─────────────────────────────────────────────────────────────────────────────
  matchStartTime = millis();

  // ─────────────────────────────────────────────────────────────────────────────
  // Retract arms for initial maneuvering
  // ─────────────────────────────────────────────────────────────────────────────
  retractArms();

  // ─────────────────────────────────────────────────────────────────────────────
  // Detect which side we're on (Phase 1)
  // ─────────────────────────────────────────────────────────────────────────────
  detectOwnSideColor();

  // ─────────────────────────────────────────────────────────────────────────────
  // Initialize position estimate
  // Robot starts centered on back boundary, facing forward
  // ─────────────────────────────────────────────────────────────────────────────
  colorSensor_X_ms = FIELD_WIDTH_MS / 2;  // Center of field width
  colorSensor_Y_ms = 0;                    // At back boundary
  heading_deg = 0;                         // Facing forward

  DEBUG_PRINTLN(F("Initialization complete"));
  DEBUG_PRINTF(F("Own side: "), ownSideColor == COLOR_RED ? "RED" : "BLUE");
  DEBUG_PRINTF(F("Position X (ms): "), colorSensor_X_ms);
  DEBUG_PRINTF(F("Position Y (ms): "), colorSensor_Y_ms);

  // ─────────────────────────────────────────────────────────────────────────────
  // Ready indication
  // ─────────────────────────────────────────────────────────────────────────────
  digitalWrite(PIN_LED_2, HIGH);
  statusBeep(2500, 200);

  // Move to navigation phase
  currentPhase = PHASE_NAVIGATE;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  // ─────────────────────────────────────────────────────────────────────────────
  // Check match time limit
  // ─────────────────────────────────────────────────────────────────────────────
  if (isMatchOver()) {
    currentPhase = PHASE_COMPLETE;
    stopMotors();
    retractArms();
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);
    statusBeep(4000, 1000);

    DEBUG_PRINTLN(F("=== MATCH COMPLETE ==="));

    while (true) {
      delay(1000);  // Match over - halt forever
    }
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Check for emergency conditions
  // ─────────────────────────────────────────────────────────────────────────────
  if (emergencyStop) {
    stopMotors();
    handleBoundaryEmergency();
    emergencyStop = false;
    return;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // Execute current phase
  // ─────────────────────────────────────────────────────────────────────────────
  switch (currentPhase) {
    case PHASE_INIT:
      // Should not reach here - handled in setup()
      currentPhase = PHASE_NAVIGATE;
      break;

    case PHASE_NAVIGATE:
      executePhase_Navigate();
      break;

    case PHASE_SWEEP:
      executePhase_Sweep();
      break;

    case PHASE_COMPLETE:
      // Match over
      break;
  }

  // Small delay to prevent tight loop
  delay(10);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PHASE 1: SIDE DETECTION
// ═══════════════════════════════════════════════════════════════════════════════

void detectOwnSideColor() {
  DEBUG_PRINTLN(F("Detecting field side color..."));

  // Robot starts with sensor potentially over black boundary or colored field
  // Move forward slightly until we're clearly on colored surface

  unsigned long red, blue;

  // Take initial reading
  red = measureColorDuration(true);
  blue = measureColorDuration(false);

  DEBUG_PRINTF(F("Initial red: "), red);
  DEBUG_PRINTF(F("Initial blue: "), blue);

  // If on black boundary, move forward to colored surface
  if (red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD) {
    DEBUG_PRINTLN(F("On black boundary, moving forward..."));

    // Move forward slowly until color detected
    for (int i = 0; i < 10; i++) {
      maneuver(100, 100, 100);  // Slow forward pulse
      delay(50);

      red = measureColorDuration(true);
      blue = measureColorDuration(false);

      if (red < BLACK_THRESHOLD || blue < BLACK_THRESHOLD) {
        DEBUG_PRINTLN(F("Color detected!"));
        break;
      }
    }
    stopMotors();
  }

  // Read color to determine our side
  // Lower duration = more reflection = that color
  red = measureColorDuration(true);
  blue = measureColorDuration(false);

  DEBUG_PRINTF(F("Final red: "), red);
  DEBUG_PRINTF(F("Final blue: "), blue);

  if (red < blue) {
    ownSideColor = COLOR_RED;
    DEBUG_PRINTLN(F("We are on RED side"));
    blinkLED(PIN_LED_1, 3);  // LED pattern for red
  } else {
    ownSideColor = COLOR_BLUE;
    DEBUG_PRINTLN(F("We are on BLUE side"));
    blinkLED(PIN_LED_2, 3);  // LED pattern for blue
  }

  // Update position - sensor is now slightly into field
  colorSensor_Y_ms = 200;  // Approximately 2" into field

  statusBeep(2000, 200);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PHASE 2: NAVIGATE TO STARTING POSITION
// ═══════════════════════════════════════════════════════════════════════════════

void executePhase_Navigate() {
  static int navStep = 0;

  DEBUG_PRINTF(F("Navigate step: "), navStep);

  switch (navStep) {
    case 0:
      // ─────────────────────────────────────────────────────────────────────────
      // Step 2A: Turn right to face right boundary
      // ─────────────────────────────────────────────────────────────────────────
      DEBUG_PRINTLN(F("Step 2A: Turn right"));
      turnOnDimeRight();
      heading_deg = 90;  // Now facing right
      navStep = 1;
      delay(200);
      break;

    case 1:
      // ─────────────────────────────────────────────────────────────────────────
      // Step 2B: Drive toward right boundary
      // ─────────────────────────────────────────────────────────────────────────
      {
        DEBUG_PRINTLN(F("Step 2B: Drive right"));

        // Calculate distance to drive (from center to near right edge)
        // Current X: ~FIELD_WIDTH_MS/2, Target X: ~0.9 * FIELD_WIDTH_MS
        unsigned int driveDistance = (FIELD_WIDTH_MS * 4) / 10;  // 40% of field width

        if (moveForwardMs(driveDistance)) {
          // Update position (moving right = increasing X when facing right)
          colorSensor_X_ms += driveDistance;
          navStep = 2;
        } else {
          // Boundary hit - adjust
          DEBUG_PRINTLN(F("Boundary during right drive"));
          handleBoundaryEmergency();
        }
        delay(200);
      }
      break;

    case 2:
      // ─────────────────────────────────────────────────────────────────────────
      // Step 2C: Traveling turn left to face forward
      // ─────────────────────────────────────────────────────────────────────────
      DEBUG_PRINTLN(F("Step 2C: Traveling turn left"));
      travelingTurnLeft();
      heading_deg = 0;  // Now facing forward

      // Traveling turn also moves us forward and slightly left
      colorSensor_Y_ms += 400;   // Moved forward during turn
      colorSensor_X_ms -= 200;   // Curved left slightly

      navStep = 3;
      delay(200);
      break;

    case 3:
      // ─────────────────────────────────────────────────────────────────────────
      // Navigation complete - move to sweep phase
      // ─────────────────────────────────────────────────────────────────────────
      DEBUG_PRINTLN(F("Navigation complete - starting sweeps"));
      currentPhase = PHASE_SWEEP;
      currentLane = 1;
      completedSweeps = 0;

      // Deploy arms for sweeping
      deployArms();
      delay(500);
      break;

    default:
      navStep = 0;
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  PHASE 3: OFFENSIVE SWEEP EXECUTION
// ═══════════════════════════════════════════════════════════════════════════════

void executePhase_Sweep() {
  DEBUG_PRINTF(F("Executing sweep lane: "), currentLane);

  // Execute one sweep lane
  executeSingleSweepLane();

  // After sweep, check if we have time for more
  if (!isMatchOver() && completedSweeps < 8) {  // Max 8 sweeps (4 lanes × 2)
    // Return to own side and shift to next lane
    returnToOwnSide();

    // Shift to next lane
    currentLane++;
    if (currentLane > 4) {
      currentLane = 1;  // Restart from right side
    }

    // Shift position for next lane (12" = about 600ms at 20"/s)
    unsigned int laneShift_ms = (unsigned int)(12.0 * MS_PER_INCH);

    // Turn left
    turnOnDimeLeft();
    heading_deg = -90;  // Facing left

    // Move to next lane
    if (moveForwardMs(laneShift_ms)) {
      colorSensor_X_ms -= laneShift_ms;  // Moved left
    }

    // Turn back to face forward
    turnOnDimeRight();
    heading_deg = 0;

    delay(200);
  }
}

void executeSingleSweepLane() {
  DEBUG_PRINTLN(F("Starting offensive sweep"));

  // Deploy arms for maximum sweeping width
  deployArms();
  delay(300);

  // ─────────────────────────────────────────────────────────────────────────────
  // Forward sweep: Push cubes deep into opponent territory
  // Target: 3/4 into opponent territory = 75% of field length
  // ─────────────────────────────────────────────────────────────────────────────

  unsigned int targetY_ms = (FIELD_LENGTH_MS * 3) / 4;  // 75% depth
  unsigned int moveChunk_ms = 200;  // Move in small chunks for safety

  DEBUG_PRINTF(F("Sweeping to Y: "), targetY_ms);

  while (colorSensor_Y_ms < targetY_ms) {
    // Check match time
    if (isMatchOver()) return;

    // Check for obstacles
    if (detectObstacle()) {
      DEBUG_PRINTLN(F("Obstacle detected during sweep"));
      handleObstacleAvoidance();
      continue;
    }

    // Move forward in chunks
    if (moveForwardMs(moveChunk_ms)) {
      colorSensor_Y_ms += moveChunk_ms;
    } else {
      // Boundary or obstacle - stop sweep
      DEBUG_PRINTLN(F("Stopping sweep - boundary detected"));
      break;
    }
  }

  stopMotors();
  completedSweeps++;

  DEBUG_PRINTF(F("Sweep complete, total sweeps: "), completedSweeps);

  // At 3/4 depth, transition arms forward for cube manipulation
  deployArmsForward();
  delay(300);

  // Push a bit more with arms forward
  moveForwardMs(300);
  colorSensor_Y_ms += 300;

  stopMotors();
}

void returnToOwnSide() {
  DEBUG_PRINTLN(F("Returning to own side"));

  // Retract arms for navigation
  retractArms();
  delay(300);

  // ─────────────────────────────────────────────────────────────────────────────
  // Reverse until we detect our own side color
  // This ensures we're definitively back on our territory
  // ─────────────────────────────────────────────────────────────────────────────

  unsigned int maxReverse_ms = MIDFIELD_MS + 400;  // Don't go past midfield + margin
  unsigned int reversed_ms = 0;
  unsigned int reverseChunk_ms = 200;

  while (reversed_ms < maxReverse_ms) {
    // Check match time
    if (isMatchOver()) return;

    // Check if we're back on our side
    if (detectOwnSideColorNow()) {
      DEBUG_PRINTLN(F("Own side detected!"));
      break;
    }

    // Check for back boundary
    if (detectBlackBoundary()) {
      DEBUG_PRINTLN(F("Back boundary detected"));
      moveForwardMs(200);  // Move away from boundary
      colorSensor_Y_ms += 200;
      break;
    }

    // Reverse
    if (moveBackwardMs(reverseChunk_ms)) {
      colorSensor_Y_ms -= reverseChunk_ms;
      reversed_ms += reverseChunk_ms;
    } else {
      // Boundary hit
      break;
    }
  }

  stopMotors();

  // Reset Y position estimate based on color detection
  if (detectOwnSideColorNow()) {
    colorSensor_Y_ms = MIDFIELD_MS - 400;  // Just before midfield on our side
  }

  DEBUG_PRINTF(F("Return complete, Y position: "), colorSensor_Y_ms);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOVEMENT PRIMITIVES (with continuous boundary checking)
// ═══════════════════════════════════════════════════════════════════════════════

void maneuver(int speedLeft, int speedRight, unsigned int duration_ms) {
  // Speed range: -200 to +200
  // Positive = forward, Negative = backward

  servoLeft.writeMicroseconds(SERVO_STOP + speedLeft);
  servoRight.writeMicroseconds(SERVO_STOP - speedRight);

  if (duration_ms > 0) {
    delay(duration_ms);
  }
}

void stopMotors() {
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  delay(50);
}

// Move forward with CONTINUOUS boundary monitoring
// Returns true if completed successfully, false if boundary detected
bool moveForwardMs(unsigned int distance_ms) {
  unsigned int chunk_ms = 50;  // Check boundary every 50ms
  unsigned int moved_ms = 0;

  while (moved_ms < distance_ms) {
    // CRITICAL: Check for black boundary BEFORE moving
    if (detectBlackBoundary()) {
      stopMotors();
      DEBUG_PRINTLN(F("BOUNDARY DETECTED - stopping"));
      statusBeep(500, 200);
      return false;
    }

    // Calculate this chunk size
    unsigned int thisChunk = min(chunk_ms, distance_ms - moved_ms);

    // Execute movement
    maneuver(SPEED_OFFSET, SPEED_OFFSET, thisChunk);

    moved_ms += thisChunk;
  }

  stopMotors();
  return true;
}

// Move backward with CONTINUOUS boundary monitoring
bool moveBackwardMs(unsigned int distance_ms) {
  unsigned int chunk_ms = 50;
  unsigned int moved_ms = 0;

  while (moved_ms < distance_ms) {
    // Check for black boundary
    if (detectBlackBoundary()) {
      stopMotors();
      DEBUG_PRINTLN(F("BOUNDARY DETECTED - stopping"));
      statusBeep(500, 200);
      return false;
    }

    unsigned int thisChunk = min(chunk_ms, distance_ms - moved_ms);

    // Execute backward movement
    maneuver(-SPEED_OFFSET, -SPEED_OFFSET, thisChunk);

    moved_ms += thisChunk;
  }

  stopMotors();
  return true;
}

void turnOnDimeLeft() {
  DEBUG_PRINTLN(F("Turn left 90 degrees"));

  // Left turn: left wheel backward, right wheel forward
  maneuver(-SPEED_OFFSET, SPEED_OFFSET, TURN_LEFT_90_MS);

  stopMotors();
  delay(100);
}

void turnOnDimeRight() {
  DEBUG_PRINTLN(F("Turn right 90 degrees"));

  // Right turn: left wheel forward, right wheel backward
  maneuver(SPEED_OFFSET, -SPEED_OFFSET, TURN_RIGHT_90_MS);

  stopMotors();
  delay(100);
}

void travelingTurnRight() {
  DEBUG_PRINTLN(F("Traveling turn right"));

  // Curved right turn: left wheel faster than right
  maneuver(TRAVELING_TURN_LEFT_SPEED, TRAVELING_TURN_RIGHT_SPEED,
           TRAVELING_TURN_90_DURATION);

  stopMotors();
  delay(100);
}

void travelingTurnLeft() {
  DEBUG_PRINTLN(F("Traveling turn left"));

  // Curved left turn: right wheel faster than left
  maneuver(TRAVELING_TURN_RIGHT_SPEED, TRAVELING_TURN_LEFT_SPEED,
           TRAVELING_TURN_90_DURATION);

  stopMotors();
  delay(100);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SENSOR READING FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

unsigned long measureColorDuration(bool readRed) {
  // Set filter selection
  if (readRed) {
    // Select RED photodiodes (S2=LOW, S3=LOW)
    digitalWrite(PIN_COLOR_S2, LOW);
    digitalWrite(PIN_COLOR_S3, LOW);
  } else {
    // Select BLUE photodiodes (S2=LOW, S3=HIGH)
    digitalWrite(PIN_COLOR_S2, LOW);
    digitalWrite(PIN_COLOR_S3, HIGH);
  }

  delayMicroseconds(COLOR_STABILIZE_MS * 1000);

  // Read frequency as pulse duration
  unsigned long duration = pulseIn(PIN_COLOR_OUT, LOW, PULSEIN_TIMEOUT);

  // pulseIn returns 0 on timeout - treat as very dark (black)
  if (duration == 0) {
    duration = BLACK_THRESHOLD + 200;  // Indicate timeout = black
  }

  return duration;
}

bool detectBlackBoundary() {
  unsigned long red = measureColorDuration(true);
  unsigned long blue = measureColorDuration(false);

  // Black boundary: BOTH colors show high duration (low reflection)
  return (red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD);
}

bool detectOwnSideColorNow() {
  // Check if currently on our own side color

  unsigned long red = measureColorDuration(true);
  unsigned long blue = measureColorDuration(false);

  // Check for black first
  if (red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD) {
    return false;  // On boundary, not on colored surface
  }

  // Determine current surface color
  FieldColor currentColor;
  if (red < blue) {
    currentColor = COLOR_RED;
  } else {
    currentColor = COLOR_BLUE;
  }

  return (currentColor == ownSideColor);
}

float measureUltrasonicDistance() {
  // HC-SR04 3-pin configuration: trigger and echo on same pin

  // Send trigger pulse
  pinMode(PIN_ULTRASONIC, OUTPUT);
  digitalWrite(PIN_ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC, LOW);

  // Read echo
  pinMode(PIN_ULTRASONIC, INPUT);
  unsigned long duration = pulseIn(PIN_ULTRASONIC, HIGH, 30000);  // 30ms timeout

  // Calculate distance in inches
  // Speed of sound = 343 m/s = 0.0135 in/µs
  // Distance = (duration * 0.0135) / 2 (round trip)
  float distance = duration * 0.00675;

  return distance;
}

bool detectObstacle() {
  float distance = measureUltrasonicDistance();

  // Check if obstacle within threshold
  if (distance > 0 && distance < OBSTACLE_DISTANCE_THRESHOLD) {
    DEBUG_PRINTF(F("Obstacle at: "), distance);
    return true;
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ARM CONTROL
// ═══════════════════════════════════════════════════════════════════════════════

void deployArms() {
  if (armsDeployed) return;

  DEBUG_PRINTLN(F("Deploying arms (perpendicular)"));

  armLeft.write(ARM_CENTER);   // 90° perpendicular
  armRight.write(ARM_CENTER);
  delay(400);

  armsDeployed = true;
}

void deployArmsForward() {
  DEBUG_PRINTLN(F("Deploying arms (forward)"));

  armLeft.write(ARM_LEFT_FORWARD);    // 30° (forward)
  armRight.write(ARM_RIGHT_FORWARD);  // 150° (forward)
  delay(400);

  armsDeployed = true;
}

void retractArms() {
  DEBUG_PRINTLN(F("Retracting arms"));

  armLeft.write(ARM_LEFT_RETRACT);    // 152° (back)
  armRight.write(ARM_RIGHT_RETRACT);  // 28° (back)
  delay(400);

  armsDeployed = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  EMERGENCY HANDLING
// ═══════════════════════════════════════════════════════════════════════════════

void handleBoundaryEmergency() {
  DEBUG_PRINTLN(F("BOUNDARY EMERGENCY"));

  stopMotors();
  statusBeep(500, 300);

  // Retract arms for safety
  retractArms();

  // Determine which boundary based on position estimate
  if (colorSensor_Y_ms > MIDFIELD_MS) {
    // Likely at front boundary (opponent's back) - reverse
    DEBUG_PRINTLN(F("Front boundary - reversing"));

    maneuver(-SPEED_OFFSET, -SPEED_OFFSET, 500);
    stopMotors();
    colorSensor_Y_ms -= 500;

  } else if (colorSensor_Y_ms < 300) {
    // Likely at back boundary (our back) - advance
    DEBUG_PRINTLN(F("Back boundary - advancing"));

    maneuver(SPEED_OFFSET, SPEED_OFFSET, 400);
    stopMotors();
    colorSensor_Y_ms += 400;

  } else if (colorSensor_X_ms > (FIELD_WIDTH_MS * 3 / 4)) {
    // Near right boundary - turn left and move
    DEBUG_PRINTLN(F("Right boundary - turning left"));

    turnOnDimeLeft();
    maneuver(SPEED_OFFSET, SPEED_OFFSET, 400);
    stopMotors();
    colorSensor_X_ms -= 400;
    turnOnDimeRight();  // Face forward again

  } else if (colorSensor_X_ms < (FIELD_WIDTH_MS / 4)) {
    // Near left boundary - turn right and move
    DEBUG_PRINTLN(F("Left boundary - turning right"));

    turnOnDimeRight();
    maneuver(SPEED_OFFSET, SPEED_OFFSET, 400);
    stopMotors();
    colorSensor_X_ms += 400;
    turnOnDimeLeft();  // Face forward again

  } else {
    // Unexpected boundary in middle - conservative reverse
    DEBUG_PRINTLN(F("Unexpected boundary - conservative reverse"));

    maneuver(-SPEED_OFFSET, -SPEED_OFFSET, 400);
    stopMotors();
    colorSensor_Y_ms -= 400;
  }

  delay(200);
}

void handleObstacleAvoidance() {
  DEBUG_PRINTLN(F("Avoiding obstacle"));

  stopMotors();
  statusBeep(4000, 100);

  // Simple avoidance: back up and shift lane
  maneuver(-SPEED_OFFSET, -SPEED_OFFSET, 300);
  stopMotors();
  colorSensor_Y_ms -= 300;

  // Shift right slightly
  turnOnDimeRight();
  maneuver(SPEED_OFFSET, SPEED_OFFSET, 200);
  stopMotors();
  turnOnDimeLeft();

  colorSensor_X_ms += 150;  // Approximate shift

  delay(200);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

void statusBeep(int frequency, int duration) {
  tone(PIN_BUZZER, frequency, duration);
  delay(duration);
}

void blinkLED(int pin, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(150);
    digitalWrite(pin, LOW);
    delay(150);
  }
}

unsigned long getMatchElapsed() {
  return millis() - matchStartTime;
}

bool isMatchOver() {
  return (getMatchElapsed() >= MATCH_DURATION_MS);
}
