/* ═══════════════════════════════════════════════════════════════════════════════
   AUTOWIPER CALIBRATION & TESTING TOOL
   EGME 456 - Cal State Fullerton Messy Room Competition
   ═══════════════════════════════════════════════════════════════════════════════

   PURPOSE: Standalone testing and measurement tool for calibrating the robot.
            NEVER used in competition - only for pre-competition setup.

   This file provides:
   - Test 1: Color Sensor Verification (RED, BLUE, BLACK detection)
   - Test 2: Forward Movement Distance Calibration
   - Test 3: Turn Calibration (Left and Right separately)
   - Test 4: Traveling Turn Calibration
   - Test 5: Full Field Navigation Test
   - Test 6: Ultrasonic Sensor Verification

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

   SERVO CONSTANTS:
   - Continuous rotation: 1500µs = stop, 1700µs = forward, 1300µs = backward
   - Arm positions: 90° ± 62° (retracted toward back), 90° ± 60° (forward)

   CRITICAL: Color sensor mounted at REAR of robot.
             Robot extends 8" FORWARD from sensor position.
   ═══════════════════════════════════════════════════════════════════════════════ */

#include <Servo.h>

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
#define SERVO_FULL_FWD    1700   // Full speed forward
#define SERVO_FULL_REV    1300   // Full speed reverse
#define SPEED_OFFSET      200    // Offset from neutral for full speed

// Arm positions
#define ARM_CENTER        90     // Perpendicular to robot
#define ARM_LEFT_RETRACT  152    // 90 + 62 = CCW toward BACK
#define ARM_RIGHT_RETRACT 28     // 90 - 62 = CW toward BACK
#define ARM_LEFT_FORWARD  30     // 90 - 60 = toward FRONT
#define ARM_RIGHT_FORWARD 150    // 90 + 60 = toward FRONT

// ═══════════════════════════════════════════════════════════════════════════════
//  COLOR SENSOR CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define COLOR_STABILIZE_MS   20      // Stabilization delay after filter change
#define PULSEIN_TIMEOUT      30000   // Timeout for pulseIn (30ms)

// ═══════════════════════════════════════════════════════════════════════════════
//  GLOBAL OBJECTS
// ═══════════════════════════════════════════════════════════════════════════════

Servo servoLeft, servoRight;
Servo armLeft, armRight;

// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATION DATA STORAGE
// ═══════════════════════════════════════════════════════════════════════════════

// Color sensor readings
unsigned long redOnRed = 0, blueOnRed = 0;
unsigned long redOnBlue = 0, blueOnBlue = 0;
unsigned long redOnBlack = 0, blueOnBlack = 0;
unsigned long blackThreshold = 0;

// Distance calibration
float distancePer1000ms_FullSpeed = 0.0;
float distancePer1000ms_HalfSpeed = 0.0;

// Turn calibration (separate for left and right)
unsigned int turnLeft90ms = 0;
unsigned int turnRight90ms = 0;

// Traveling turn calibration
int travelTurnLeftSpeed = 0;
int travelTurnRightSpeed = 0;
unsigned int travelTurn90Duration = 0;

// Ultrasonic calibration
float ultrasonicMaxRange = 0.0;

// ═══════════════════════════════════════════════════════════════════════════════
//  FUNCTION PROTOTYPES
// ═══════════════════════════════════════════════════════════════════════════════

void displayMenu();
void runTest1_ColorSensor();
void runTest2_ForwardDistance();
void runTest3_TurnCalibration();
void runTest4_TravelingTurn();
void runTest5_FieldNavigation();
void runTest6_Ultrasonic();
void displayCalibrationSummary();

void maneuver(int speedLeft, int speedRight, int msTime);
void stopMotors();
unsigned long measureColorDuration(bool readRed);
float measureUltrasonicDistance();
void statusBeep(int frequency, int duration);
void blinkLED(int pin, int count, int delayMs);
void waitForSerialInput();
float readSerialFloat();
int readSerialInt();
void clearSerialBuffer();

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial port to connect (for Leonardo/Micro)
  }

  delay(500);

  // Attach servos
  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);
  armLeft.attach(PIN_ARM_LEFT);
  armRight.attach(PIN_ARM_RIGHT);

  // Stop drive motors initially
  stopMotors();

  // Retract arms for safety
  armLeft.write(ARM_LEFT_RETRACT);
  armRight.write(ARM_RIGHT_RETRACT);

  // Color sensor pins
  pinMode(PIN_COLOR_OUT, INPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);

  // Ultrasonic pin (will toggle between OUTPUT and INPUT)
  pinMode(PIN_ULTRASONIC, OUTPUT);
  digitalWrite(PIN_ULTRASONIC, LOW);

  // Debug outputs
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);

  // Welcome indication
  digitalWrite(PIN_LED_1, HIGH);
  statusBeep(2000, 200);
  delay(100);
  statusBeep(2500, 200);
  delay(100);
  statusBeep(3000, 200);
  digitalWrite(PIN_LED_1, LOW);

  // Display welcome banner
  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║          AUTOWIPER CALIBRATION & TESTING TOOL                  ║"));
  Serial.println(F("║          EGME 456 - Cal State Fullerton                        ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("This tool helps you measure and verify robot parameters."));
  Serial.println(F("Run each test and record the values for competition code."));
  Serial.println();

  displayMenu();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    clearSerialBuffer();

    Serial.println();

    switch (input) {
      case '1':
        runTest1_ColorSensor();
        break;
      case '2':
        runTest2_ForwardDistance();
        break;
      case '3':
        runTest3_TurnCalibration();
        break;
      case '4':
        runTest4_TravelingTurn();
        break;
      case '5':
        runTest5_FieldNavigation();
        break;
      case '6':
        runTest6_Ultrasonic();
        break;
      case '7':
        displayCalibrationSummary();
        break;
      case 'm':
      case 'M':
        displayMenu();
        break;
      default:
        Serial.println(F("Invalid selection. Press 'm' to see menu."));
        break;
    }

    Serial.println();
    Serial.println(F("Press 'm' for menu or enter test number (1-6)"));
  }

  delay(50);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MENU DISPLAY
// ═══════════════════════════════════════════════════════════════════════════════

void displayMenu() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║                       TEST MENU                                ║"));
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  Serial.println(F("║  1 - Color Sensor Verification (RED, BLUE, BLACK)              ║"));
  Serial.println(F("║  2 - Forward Movement Distance Calibration                     ║"));
  Serial.println(F("║  3 - Turn Calibration (Left & Right separately)                ║"));
  Serial.println(F("║  4 - Traveling Turn Calibration                                ║"));
  Serial.println(F("║  5 - Full Field Navigation Test                                ║"));
  Serial.println(F("║  6 - Ultrasonic Sensor Verification                            ║"));
  Serial.println(F("║  7 - Display Calibration Summary (copy to competition code)    ║"));
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  Serial.println(F("║  m - Show this menu                                            ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Enter test number (1-7):"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEST 1: COLOR SENSOR VERIFICATION
// ═══════════════════════════════════════════════════════════════════════════════

void runTest1_ColorSensor() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║            TEST 1: COLOR SENSOR VERIFICATION                   ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("This test verifies the TCS3200 can distinguish RED, BLUE, and BLACK."));
  Serial.println(F("Lower duration values = more light reflected (brighter surface)"));
  Serial.println(F("Higher duration values = less light reflected (darker surface)"));
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // RED SURFACE TEST
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ STEP 1: RED SURFACE ═══"));
  Serial.println(F("Place the robot with color sensor over RED field surface."));
  Serial.println(F("Press ENTER when ready..."));
  waitForSerialInput();

  blinkLED(PIN_LED_1, 3, 200);
  statusBeep(1500, 100);

  // Take 10 readings
  unsigned long redSum = 0, blueSum = 0;
  Serial.println(F("Taking 10 readings..."));

  for (int i = 0; i < 10; i++) {
    unsigned long r = measureColorDuration(true);
    unsigned long b = measureColorDuration(false);
    redSum += r;
    blueSum += b;
    Serial.print(F("  Reading ")); Serial.print(i+1);
    Serial.print(F(": Red=")); Serial.print(r);
    Serial.print(F("us, Blue=")); Serial.print(b);
    Serial.println(F("us"));
    delay(100);
  }

  redOnRed = redSum / 10;
  blueOnRed = blueSum / 10;

  Serial.println();
  Serial.print(F("RED SURFACE AVERAGE: Red=")); Serial.print(redOnRed);
  Serial.print(F("us, Blue=")); Serial.print(blueOnRed);
  Serial.println(F("us"));

  // Verify: on red surface, red duration should be LOWER than blue duration
  // (red filter lets red light through, more reflection = lower pulse width)
  if (redOnRed < blueOnRed) {
    Serial.println(F("✓ PASS: Red < Blue (correct for RED surface)"));
    digitalWrite(PIN_LED_1, HIGH);
    statusBeep(2000, 300);
  } else {
    Serial.println(F("✗ WARNING: Expected Red < Blue on RED surface"));
    Serial.println(F("  Check sensor orientation or surface color"));
    blinkLED(PIN_LED_2, 5, 100);
    statusBeep(500, 500);
  }
  digitalWrite(PIN_LED_1, LOW);

  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // BLUE SURFACE TEST
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ STEP 2: BLUE SURFACE ═══"));
  Serial.println(F("Place the robot with color sensor over BLUE field surface."));
  Serial.println(F("Press ENTER when ready..."));
  waitForSerialInput();

  blinkLED(PIN_LED_1, 3, 200);
  statusBeep(1500, 100);

  redSum = 0; blueSum = 0;
  Serial.println(F("Taking 10 readings..."));

  for (int i = 0; i < 10; i++) {
    unsigned long r = measureColorDuration(true);
    unsigned long b = measureColorDuration(false);
    redSum += r;
    blueSum += b;
    Serial.print(F("  Reading ")); Serial.print(i+1);
    Serial.print(F(": Red=")); Serial.print(r);
    Serial.print(F("us, Blue=")); Serial.print(b);
    Serial.println(F("us"));
    delay(100);
  }

  redOnBlue = redSum / 10;
  blueOnBlue = blueSum / 10;

  Serial.println();
  Serial.print(F("BLUE SURFACE AVERAGE: Red=")); Serial.print(redOnBlue);
  Serial.print(F("us, Blue=")); Serial.print(blueOnBlue);
  Serial.println(F("us"));

  // Verify: on blue surface, blue duration should be LOWER than red duration
  if (blueOnBlue < redOnBlue) {
    Serial.println(F("✓ PASS: Blue < Red (correct for BLUE surface)"));
    digitalWrite(PIN_LED_1, HIGH);
    statusBeep(2000, 300);
  } else {
    Serial.println(F("✗ WARNING: Expected Blue < Red on BLUE surface"));
    Serial.println(F("  Check sensor orientation or surface color"));
    blinkLED(PIN_LED_2, 5, 100);
    statusBeep(500, 500);
  }
  digitalWrite(PIN_LED_1, LOW);

  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // BLACK SURFACE TEST
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ STEP 3: BLACK BOUNDARY ═══"));
  Serial.println(F("Place the robot with color sensor over BLACK boundary."));
  Serial.println(F("Press ENTER when ready..."));
  waitForSerialInput();

  blinkLED(PIN_LED_1, 3, 200);
  statusBeep(1500, 100);

  redSum = 0; blueSum = 0;
  Serial.println(F("Taking 10 readings..."));

  for (int i = 0; i < 10; i++) {
    unsigned long r = measureColorDuration(true);
    unsigned long b = measureColorDuration(false);
    redSum += r;
    blueSum += b;
    Serial.print(F("  Reading ")); Serial.print(i+1);
    Serial.print(F(": Red=")); Serial.print(r);
    Serial.print(F("us, Blue=")); Serial.print(b);
    Serial.println(F("us"));
    delay(100);
  }

  redOnBlack = redSum / 10;
  blueOnBlack = blueSum / 10;

  Serial.println();
  Serial.print(F("BLACK SURFACE AVERAGE: Red=")); Serial.print(redOnBlack);
  Serial.print(F("us, Blue=")); Serial.print(blueOnBlack);
  Serial.println(F("us"));

  // Calculate recommended black threshold
  // Black should have BOTH red and blue values higher than colored surfaces
  unsigned long minBlackReading = min(redOnBlack, blueOnBlack);
  unsigned long maxColorReading = max(max(redOnRed, blueOnRed), max(redOnBlue, blueOnBlue));

  // Set threshold between max color reading and min black reading
  blackThreshold = (minBlackReading + maxColorReading) / 2;

  // Verify: black should have high values for both channels
  if (redOnBlack > maxColorReading && blueOnBlack > maxColorReading) {
    Serial.println(F("✓ PASS: Both channels high (correct for BLACK surface)"));
    digitalWrite(PIN_LED_1, HIGH);
    statusBeep(2000, 300);
  } else {
    Serial.println(F("✗ WARNING: Black values not clearly higher than colors"));
    Serial.println(F("  May need manual threshold adjustment"));
    blinkLED(PIN_LED_2, 5, 100);
    statusBeep(500, 500);
  }
  digitalWrite(PIN_LED_1, LOW);

  // ─────────────────────────────────────────────────────────────────────────────
  // SUMMARY
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║              COLOR SENSOR TEST SUMMARY                         ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Surface    | Red Duration | Blue Duration | Classification"));
  Serial.println(F("-----------|--------------|---------------|---------------"));

  Serial.print(F("RED        | "));
  Serial.print(redOnRed); Serial.print(F("us"));
  for (int i = String(redOnRed).length(); i < 10; i++) Serial.print(' ');
  Serial.print(F("| ")); Serial.print(blueOnRed); Serial.print(F("us"));
  for (int i = String(blueOnRed).length(); i < 11; i++) Serial.print(' ');
  Serial.println(redOnRed < blueOnRed ? F("| PASS") : F("| CHECK"));

  Serial.print(F("BLUE       | "));
  Serial.print(redOnBlue); Serial.print(F("us"));
  for (int i = String(redOnBlue).length(); i < 10; i++) Serial.print(' ');
  Serial.print(F("| ")); Serial.print(blueOnBlue); Serial.print(F("us"));
  for (int i = String(blueOnBlue).length(); i < 11; i++) Serial.print(' ');
  Serial.println(blueOnBlue < redOnBlue ? F("| PASS") : F("| CHECK"));

  Serial.print(F("BLACK      | "));
  Serial.print(redOnBlack); Serial.print(F("us"));
  for (int i = String(redOnBlack).length(); i < 10; i++) Serial.print(' ');
  Serial.print(F("| ")); Serial.print(blueOnBlack); Serial.print(F("us"));
  for (int i = String(blueOnBlack).length(); i < 11; i++) Serial.print(' ');
  Serial.println((redOnBlack > blackThreshold && blueOnBlack > blackThreshold) ? F("| PASS") : F("| CHECK"));

  Serial.println();
  Serial.print(F("RECOMMENDED BLACK_THRESHOLD = ")); Serial.println(blackThreshold);
  Serial.println(F("(Both red AND blue must be > this value to detect black)"));

  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEST 2: FORWARD MOVEMENT DISTANCE CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════

void runTest2_ForwardDistance() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║        TEST 2: FORWARD MOVEMENT DISTANCE CALIBRATION           ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("This test measures actual distance traveled for a timed movement."));
  Serial.println(F("You will measure the distance traveled and enter it."));
  Serial.println();

  float readings[3] = {0, 0, 0};

  // ─────────────────────────────────────────────────────────────────────────────
  // FULL SPEED TEST (3 trials)
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ FULL SPEED TEST (speed offset = 200) ═══"));
  Serial.println();

  for (int trial = 1; trial <= 3; trial++) {
    Serial.print(F("TRIAL ")); Serial.print(trial); Serial.println(F(" of 3"));
    Serial.println(F("1. Mark robot's starting position on the floor"));
    Serial.println(F("2. Ensure clear path ahead (at least 30 inches)"));
    Serial.println(F("Press ENTER when ready..."));
    waitForSerialInput();

    // Countdown
    Serial.println(F("Starting in 3..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("2..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("1..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("GO!"));
    statusBeep(2000, 500);

    digitalWrite(PIN_LED_1, HIGH);

    // Execute: move forward at full speed for 1000ms
    maneuver(SPEED_OFFSET, SPEED_OFFSET, 1000);
    stopMotors();

    digitalWrite(PIN_LED_1, LOW);
    statusBeep(1500, 200);

    Serial.println();
    Serial.println(F("Robot stopped. Measure the distance from start to current position."));
    Serial.println(F("Enter distance in INCHES (e.g., 21.5):"));

    float measured = readSerialFloat();
    readings[trial - 1] = measured;

    Serial.print(F("Trial ")); Serial.print(trial);
    Serial.print(F(": ")); Serial.print(measured);
    Serial.println(F(" inches in 1000ms"));
    Serial.println();

    if (trial < 3) {
      Serial.println(F("Return robot to starting position for next trial."));
      Serial.println(F("Press ENTER when ready..."));
      waitForSerialInput();
    }
  }

  // Calculate average
  distancePer1000ms_FullSpeed = (readings[0] + readings[1] + readings[2]) / 3.0;

  Serial.println(F("═══ FULL SPEED RESULTS ═══"));
  Serial.print(F("Trial 1: ")); Serial.print(readings[0]); Serial.println(F(" inches"));
  Serial.print(F("Trial 2: ")); Serial.print(readings[1]); Serial.println(F(" inches"));
  Serial.print(F("Trial 3: ")); Serial.print(readings[2]); Serial.println(F(" inches"));
  Serial.print(F("AVERAGE: ")); Serial.print(distancePer1000ms_FullSpeed);
  Serial.println(F(" inches per 1000ms at speed 200"));
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // HALF SPEED TEST (optional, single trial)
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ HALF SPEED TEST (speed offset = 100) ═══"));
  Serial.println(F("This test is optional but recommended for slow movements."));
  Serial.println(F("Press ENTER to run test, or 's' to skip..."));

  while (!Serial.available()) { delay(50); }
  char c = Serial.read();
  clearSerialBuffer();

  if (c != 's' && c != 'S') {
    Serial.println(F("Mark robot's starting position on the floor."));
    Serial.println(F("Press ENTER when ready..."));
    waitForSerialInput();

    // Countdown
    Serial.println(F("Starting in 3..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("2..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("1..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("GO!"));
    statusBeep(2000, 500);

    digitalWrite(PIN_LED_1, HIGH);

    // Execute: move forward at half speed for 1000ms
    maneuver(100, 100, 1000);
    stopMotors();

    digitalWrite(PIN_LED_1, LOW);
    statusBeep(1500, 200);

    Serial.println();
    Serial.println(F("Enter measured distance in INCHES:"));
    distancePer1000ms_HalfSpeed = readSerialFloat();

    Serial.print(F("Half speed: ")); Serial.print(distancePer1000ms_HalfSpeed);
    Serial.println(F(" inches per 1000ms at speed 100"));
  } else {
    // Estimate half speed from full speed
    distancePer1000ms_HalfSpeed = distancePer1000ms_FullSpeed * 0.5;
    Serial.print(F("Skipped - estimating: ")); Serial.print(distancePer1000ms_HalfSpeed);
    Serial.println(F(" inches"));
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // SUMMARY
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║           FORWARD DISTANCE CALIBRATION SUMMARY                 ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.print(F("DISTANCE_PER_1000MS_FULL_SPEED = ")); Serial.println(distancePer1000ms_FullSpeed);
  Serial.print(F("DISTANCE_PER_1000MS_HALF_SPEED = ")); Serial.println(distancePer1000ms_HalfSpeed);
  Serial.println();
  Serial.println(F("Copy these values to AutoWiper_Competition.ino"));

  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEST 3: TURN CALIBRATION (LEFT AND RIGHT)
// ═══════════════════════════════════════════════════════════════════════════════

void runTest3_TurnCalibration() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║          TEST 3: TURN CALIBRATION (LEFT & RIGHT)               ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("This test calibrates on-dime turns for both directions."));
  Serial.println(F("IMPORTANT: Left and right turns may require different timing!"));
  Serial.println();
  Serial.println(F("Setup:"));
  Serial.println(F("1. Create a reference line on floor (tape/chalk)"));
  Serial.println(F("2. Align robot precisely with reference line"));
  Serial.println(F("3. Mark robot's front orientation"));
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // LEFT TURN CALIBRATION
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ LEFT TURN CALIBRATION ═══"));
  Serial.println(F("(Robot will spin counter-clockwise)"));
  Serial.println();

  unsigned int leftTurnTime = 600;  // Initial guess
  bool leftCalibrated = false;
  int leftAttempts = 0;

  while (!leftCalibrated && leftAttempts < 5) {
    leftAttempts++;
    Serial.print(F("LEFT TURN - Attempt ")); Serial.print(leftAttempts);
    Serial.print(F(" (duration = ")); Serial.print(leftTurnTime);
    Serial.println(F("ms)"));
    Serial.println(F("Align robot with reference line, press ENTER when ready..."));
    waitForSerialInput();

    // Countdown
    Serial.print(F("Turning LEFT for ")); Serial.print(leftTurnTime); Serial.println(F("ms..."));
    statusBeep(1500, 200);
    delay(500);

    digitalWrite(PIN_LED_1, HIGH);

    // Execute left turn: left wheel backward, right wheel forward
    maneuver(-SPEED_OFFSET, SPEED_OFFSET, leftTurnTime);
    stopMotors();

    digitalWrite(PIN_LED_1, LOW);
    statusBeep(1500, 200);

    Serial.println(F("Measure actual angle turned (degrees CCW from start)."));
    Serial.println(F("Enter angle (target is 90):"));

    float actualAngle = readSerialFloat();

    if (abs(actualAngle - 90.0) < 3.0) {
      // Within 3 degrees - good enough
      Serial.println(F("✓ Calibrated! Within acceptable tolerance."));
      turnLeft90ms = leftTurnTime;
      leftCalibrated = true;
    } else {
      // Calculate correction
      unsigned int newTime = (unsigned int)(leftTurnTime * 90.0 / actualAngle);
      Serial.print(F("Actual angle: ")); Serial.print(actualAngle);
      Serial.print(F("° → Adjusting from ")); Serial.print(leftTurnTime);
      Serial.print(F("ms to ")); Serial.print(newTime); Serial.println(F("ms"));
      leftTurnTime = newTime;

      if (leftAttempts >= 5) {
        Serial.println(F("Max attempts reached. Using current value."));
        turnLeft90ms = leftTurnTime;
        leftCalibrated = true;
      } else {
        Serial.println();
      }
    }
  }

  Serial.println();
  Serial.print(F("LEFT TURN RESULT: TURN_LEFT_90_MS = ")); Serial.println(turnLeft90ms);
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // RIGHT TURN CALIBRATION
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ RIGHT TURN CALIBRATION ═══"));
  Serial.println(F("(Robot will spin clockwise)"));
  Serial.println();

  unsigned int rightTurnTime = 600;  // Initial guess
  bool rightCalibrated = false;
  int rightAttempts = 0;

  while (!rightCalibrated && rightAttempts < 5) {
    rightAttempts++;
    Serial.print(F("RIGHT TURN - Attempt ")); Serial.print(rightAttempts);
    Serial.print(F(" (duration = ")); Serial.print(rightTurnTime);
    Serial.println(F("ms)"));
    Serial.println(F("Align robot with reference line, press ENTER when ready..."));
    waitForSerialInput();

    // Countdown
    Serial.print(F("Turning RIGHT for ")); Serial.print(rightTurnTime); Serial.println(F("ms..."));
    statusBeep(1500, 200);
    delay(500);

    digitalWrite(PIN_LED_1, HIGH);

    // Execute right turn: left wheel forward, right wheel backward
    maneuver(SPEED_OFFSET, -SPEED_OFFSET, rightTurnTime);
    stopMotors();

    digitalWrite(PIN_LED_1, LOW);
    statusBeep(1500, 200);

    Serial.println(F("Measure actual angle turned (degrees CW from start)."));
    Serial.println(F("Enter angle (target is 90):"));

    float actualAngle = readSerialFloat();

    if (abs(actualAngle - 90.0) < 3.0) {
      // Within 3 degrees - good enough
      Serial.println(F("✓ Calibrated! Within acceptable tolerance."));
      turnRight90ms = rightTurnTime;
      rightCalibrated = true;
    } else {
      // Calculate correction
      unsigned int newTime = (unsigned int)(rightTurnTime * 90.0 / actualAngle);
      Serial.print(F("Actual angle: ")); Serial.print(actualAngle);
      Serial.print(F("° → Adjusting from ")); Serial.print(rightTurnTime);
      Serial.print(F("ms to ")); Serial.print(newTime); Serial.println(F("ms"));
      rightTurnTime = newTime;

      if (rightAttempts >= 5) {
        Serial.println(F("Max attempts reached. Using current value."));
        turnRight90ms = rightTurnTime;
        rightCalibrated = true;
      } else {
        Serial.println();
      }
    }
  }

  Serial.println();
  Serial.print(F("RIGHT TURN RESULT: TURN_RIGHT_90_MS = ")); Serial.println(turnRight90ms);
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // SUMMARY
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║              TURN CALIBRATION SUMMARY                          ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.print(F("TURN_LEFT_90_MS  = ")); Serial.println(turnLeft90ms);
  Serial.print(F("TURN_RIGHT_90_MS = ")); Serial.println(turnRight90ms);
  Serial.println();

  int diff = abs((int)turnLeft90ms - (int)turnRight90ms);
  Serial.print(F("Difference between left and right: ")); Serial.print(diff);
  Serial.println(F("ms"));

  if (diff > 50) {
    Serial.println(F("NOTE: Significant asymmetry detected. This is normal and"));
    Serial.println(F("      is why separate left/right constants are essential!"));
  } else {
    Serial.println(F("NOTE: Turns are fairly symmetric."));
  }

  Serial.println();
  Serial.println(F("Copy these values to AutoWiper_Competition.ino"));

  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEST 4: TRAVELING TURN CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════════

void runTest4_TravelingTurn() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║            TEST 4: TRAVELING TURN CALIBRATION                  ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("A traveling turn moves forward while turning (curved path)."));
  Serial.println(F("This is used for lane changes while maintaining momentum."));
  Serial.println();
  Serial.println(F("Target: 90° turn while traveling forward ~12 inches"));
  Serial.println();

  // Default starting values
  int leftSpeed = SPEED_OFFSET;      // 200 (outer wheel, faster)
  int rightSpeed = 80;               // Inner wheel (slower)
  unsigned int duration = 1200;      // ms

  bool calibrated = false;
  int attempts = 0;

  while (!calibrated && attempts < 5) {
    attempts++;

    Serial.println(F("═══════════════════════════════════════════"));
    Serial.print(F("TRAVELING TURN - Attempt ")); Serial.println(attempts);
    Serial.print(F("Left wheel speed: ")); Serial.println(leftSpeed);
    Serial.print(F("Right wheel speed: ")); Serial.println(rightSpeed);
    Serial.print(F("Duration: ")); Serial.print(duration); Serial.println(F("ms"));
    Serial.println(F("═══════════════════════════════════════════"));
    Serial.println();
    Serial.println(F("Setup:"));
    Serial.println(F("1. Mark starting position AND orientation on floor"));
    Serial.println(F("2. Ensure ~18 inches of clear space ahead and to the right"));
    Serial.println(F("Press ENTER when ready..."));
    waitForSerialInput();

    // Countdown
    Serial.println(F("Executing traveling RIGHT turn in 3..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("2..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("1..."));
    statusBeep(1000, 200);
    delay(1000);
    Serial.println(F("GO!"));
    statusBeep(2000, 300);

    digitalWrite(PIN_LED_1, HIGH);

    // Execute traveling turn (curving right = left faster than right)
    maneuver(leftSpeed, rightSpeed, duration);
    stopMotors();

    digitalWrite(PIN_LED_1, LOW);
    statusBeep(1500, 200);

    Serial.println();
    Serial.println(F("Measure results:"));
    Serial.println(F("1. How many degrees did the robot turn? (target: 90)"));
    float actualAngle = readSerialFloat();

    Serial.println(F("2. How far forward did the robot travel? (target: ~12 inches)"));
    float actualDistance = readSerialFloat();

    Serial.print(F("Result: ")); Serial.print(actualAngle); Serial.print(F("° turn, "));
    Serial.print(actualDistance); Serial.println(F(" inches forward"));

    // Check if calibrated
    bool angleOK = (abs(actualAngle - 90.0) < 10.0);
    bool distanceOK = (actualDistance > 8.0 && actualDistance < 16.0);

    if (angleOK && distanceOK) {
      Serial.println(F("✓ Calibrated! Parameters are acceptable."));
      travelTurnLeftSpeed = leftSpeed;
      travelTurnRightSpeed = rightSpeed;
      travelTurn90Duration = duration;
      calibrated = true;
    } else {
      Serial.println(F("Adjusting parameters..."));

      // Adjust duration for angle
      if (abs(actualAngle - 90.0) > 5.0) {
        duration = (unsigned int)(duration * 90.0 / actualAngle);
        Serial.print(F("  Duration adjusted to: ")); Serial.print(duration);
        Serial.println(F("ms"));
      }

      // Adjust speed differential for turn radius
      if (actualDistance < 8.0) {
        // Too tight - increase inner wheel speed
        rightSpeed = min(rightSpeed + 20, leftSpeed - 20);
        Serial.print(F("  Right speed increased to: ")); Serial.println(rightSpeed);
      } else if (actualDistance > 16.0) {
        // Too wide - decrease inner wheel speed
        rightSpeed = max(rightSpeed - 20, 40);
        Serial.print(F("  Right speed decreased to: ")); Serial.println(rightSpeed);
      }

      if (attempts >= 5) {
        Serial.println(F("Max attempts reached. Using current values."));
        travelTurnLeftSpeed = leftSpeed;
        travelTurnRightSpeed = rightSpeed;
        travelTurn90Duration = duration;
        calibrated = true;
      }
    }

    Serial.println();
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // SUMMARY
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║           TRAVELING TURN CALIBRATION SUMMARY                   ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.print(F("TRAVELING_TURN_LEFT_SPEED  = ")); Serial.println(travelTurnLeftSpeed);
  Serial.print(F("TRAVELING_TURN_RIGHT_SPEED = ")); Serial.println(travelTurnRightSpeed);
  Serial.print(F("TRAVELING_TURN_90_DURATION = ")); Serial.println(travelTurn90Duration);
  Serial.println();
  Serial.println(F("Copy these values to AutoWiper_Competition.ino"));

  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEST 5: FULL FIELD NAVIGATION TEST
// ═══════════════════════════════════════════════════════════════════════════════

void runTest5_FieldNavigation() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║           TEST 5: FULL FIELD NAVIGATION TEST                   ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("This test verifies position tracking accuracy."));
  Serial.println(F("Robot will navigate to commanded positions using dead reckoning."));
  Serial.println();

  // Check if calibration data exists
  if (distancePer1000ms_FullSpeed == 0 || turnLeft90ms == 0 || turnRight90ms == 0) {
    Serial.println(F("WARNING: Run Tests 2 and 3 first to calibrate movement!"));
    Serial.println(F("Using default values..."));
    if (distancePer1000ms_FullSpeed == 0) distancePer1000ms_FullSpeed = 20.0;
    if (turnLeft90ms == 0) turnLeft90ms = 600;
    if (turnRight90ms == 0) turnRight90ms = 600;
  }

  Serial.println(F("Current calibration:"));
  Serial.print(F("  Distance per 1000ms: ")); Serial.print(distancePer1000ms_FullSpeed);
  Serial.println(F(" inches"));
  Serial.print(F("  Turn left 90°: ")); Serial.print(turnLeft90ms); Serial.println(F("ms"));
  Serial.print(F("  Turn right 90°: ")); Serial.print(turnRight90ms); Serial.println(F("ms"));
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // NAVIGATION TEST SEQUENCE
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ NAVIGATION TEST SEQUENCE ═══"));
  Serial.println(F("Robot will execute a square pattern:"));
  Serial.println(F("  1. Forward 12 inches"));
  Serial.println(F("  2. Turn right 90°"));
  Serial.println(F("  3. Forward 12 inches"));
  Serial.println(F("  4. Turn right 90°"));
  Serial.println(F("  5. Forward 12 inches"));
  Serial.println(F("  6. Turn right 90°"));
  Serial.println(F("  7. Forward 12 inches"));
  Serial.println(F("  8. Turn right 90° (should return to start)"));
  Serial.println();
  Serial.println(F("Mark starting position and orientation."));
  Serial.println(F("Press ENTER when ready..."));
  waitForSerialInput();

  statusBeep(2000, 500);
  delay(500);

  // Calculate movement duration for 12 inches
  unsigned int forwardTime = (unsigned int)(12.0 / distancePer1000ms_FullSpeed * 1000.0);

  Serial.print(F("Forward time for 12 inches: ")); Serial.print(forwardTime);
  Serial.println(F("ms"));
  Serial.println();

  // Execute square pattern
  for (int side = 1; side <= 4; side++) {
    Serial.print(F("Side ")); Serial.print(side); Serial.println(F(" of 4"));

    // Forward
    Serial.print(F("  Moving forward ")); Serial.print(forwardTime); Serial.println(F("ms..."));
    digitalWrite(PIN_LED_1, HIGH);
    maneuver(SPEED_OFFSET, SPEED_OFFSET, forwardTime);
    stopMotors();
    digitalWrite(PIN_LED_1, LOW);
    delay(300);

    // Turn right
    Serial.print(F("  Turning right ")); Serial.print(turnRight90ms); Serial.println(F("ms..."));
    digitalWrite(PIN_LED_2, HIGH);
    maneuver(SPEED_OFFSET, -SPEED_OFFSET, turnRight90ms);
    stopMotors();
    digitalWrite(PIN_LED_2, LOW);
    delay(300);

    statusBeep(1500, 100);
  }

  statusBeep(3000, 500);

  Serial.println();
  Serial.println(F("Navigation complete!"));
  Serial.println();
  Serial.println(F("Measure final position error from starting point:"));
  Serial.println(F("  X error (left/right of start, inches):"));
  float errorX = readSerialFloat();

  Serial.println(F("  Y error (forward/back of start, inches):"));
  float errorY = readSerialFloat();

  Serial.println(F("  Heading error (degrees CW from start, can be negative):"));
  float errorH = readSerialFloat();

  float totalError = sqrt(errorX * errorX + errorY * errorY);

  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║           NAVIGATION TEST RESULTS                              ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.print(F("Position Error:  X = ")); Serial.print(errorX);
  Serial.print(F("\", Y = ")); Serial.print(errorY);
  Serial.print(F("\" → Total = ")); Serial.print(totalError); Serial.println(F("\""));
  Serial.print(F("Heading Error:   ")); Serial.print(errorH); Serial.println(F("°"));
  Serial.println();

  // Evaluate accuracy
  if (totalError < 2.0 && abs(errorH) < 10.0) {
    Serial.println(F("✓ EXCELLENT: Odometry is very accurate!"));
  } else if (totalError < 4.0 && abs(errorH) < 20.0) {
    Serial.println(F("✓ GOOD: Odometry is acceptable for competition."));
  } else if (totalError < 6.0 && abs(errorH) < 30.0) {
    Serial.println(F("⚠ FAIR: Consider recalibrating turns."));
  } else {
    Serial.println(F("✗ POOR: Recalibration strongly recommended!"));
    Serial.println(F("  Check wheel alignment and surface traction."));
  }

  Serial.println();
  Serial.println(F("Use this error estimate for position uncertainty in competition."));

  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TEST 6: ULTRASONIC SENSOR VERIFICATION
// ═══════════════════════════════════════════════════════════════════════════════

void runTest6_Ultrasonic() {
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║          TEST 6: ULTRASONIC SENSOR VERIFICATION                ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("This test verifies HC-SR04 ultrasonic sensor accuracy."));
  Serial.println(F("Using 3-pin configuration (trigger and echo on same pin D8)."));
  Serial.println();

  // ─────────────────────────────────────────────────────────────────────────────
  // DISTANCE ACCURACY TEST
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ DISTANCE ACCURACY TEST ═══"));
  Serial.println(F("Place obstacles at known distances and verify readings."));
  Serial.println();

  float testDistances[] = {6.0, 12.0, 18.0, 24.0};
  float measuredDistances[4];

  for (int i = 0; i < 4; i++) {
    Serial.print(F("Place obstacle at EXACTLY ")); Serial.print(testDistances[i]);
    Serial.println(F(" inches from sensor."));
    Serial.println(F("Press ENTER when ready..."));
    waitForSerialInput();

    // Take 10 readings
    float sum = 0;
    int validReadings = 0;

    Serial.print(F("Taking 10 readings: "));
    for (int j = 0; j < 10; j++) {
      float dist = measureUltrasonicDistance();
      if (dist > 0 && dist < 200) {
        sum += dist;
        validReadings++;
        Serial.print(dist, 1); Serial.print(F(" "));
      } else {
        Serial.print(F("X "));
      }
      delay(100);
    }
    Serial.println();

    if (validReadings > 0) {
      measuredDistances[i] = sum / validReadings;
    } else {
      measuredDistances[i] = -1;
    }

    Serial.print(F("Average: ")); Serial.print(measuredDistances[i], 1);
    Serial.print(F("\" (expected ")); Serial.print(testDistances[i]);
    Serial.print(F("\", error: "));
    if (measuredDistances[i] > 0) {
      Serial.print(measuredDistances[i] - testDistances[i], 1);
    } else {
      Serial.print(F("N/A"));
    }
    Serial.println(F("\")"));
    Serial.println();
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // CONTINUOUS READING MODE
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println(F("═══ CONTINUOUS READING MODE ═══"));
  Serial.println(F("Move obstacle around to test sensor response."));
  Serial.println(F("Press any key to stop..."));
  Serial.println();

  while (!Serial.available()) {
    float dist = measureUltrasonicDistance();

    Serial.print(F("Distance: "));
    if (dist > 0 && dist < 200) {
      Serial.print(dist, 1);
      Serial.print(F(" inches"));

      // Visual bar graph
      Serial.print(F("  ["));
      int bars = (int)(dist / 2);
      for (int b = 0; b < 25; b++) {
        if (b < bars) Serial.print(F("█"));
        else Serial.print(F("░"));
      }
      Serial.print(F("]"));

      // LED indication
      if (dist < 12) {
        digitalWrite(PIN_LED_1, HIGH);
        digitalWrite(PIN_LED_2, LOW);
      } else if (dist < 24) {
        digitalWrite(PIN_LED_1, LOW);
        digitalWrite(PIN_LED_2, HIGH);
      } else {
        digitalWrite(PIN_LED_1, LOW);
        digitalWrite(PIN_LED_2, LOW);
      }
    } else {
      Serial.print(F("OUT OF RANGE"));
      digitalWrite(PIN_LED_1, LOW);
      digitalWrite(PIN_LED_2, LOW);
    }
    Serial.println();

    delay(200);
  }
  clearSerialBuffer();

  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);

  // ─────────────────────────────────────────────────────────────────────────────
  // SUMMARY
  // ─────────────────────────────────────────────────────────────────────────────
  Serial.println();
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║           ULTRASONIC SENSOR TEST SUMMARY                       ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Actual Distance | Measured (avg) | Error"));
  Serial.println(F("----------------|----------------|-------"));

  float maxReliableRange = 0;
  for (int i = 0; i < 4; i++) {
    Serial.print(testDistances[i], 0); Serial.print(F("\""));
    for (int s = String(testDistances[i]).length(); s < 15; s++) Serial.print(' ');
    Serial.print(F("| "));
    if (measuredDistances[i] > 0) {
      Serial.print(measuredDistances[i], 1); Serial.print(F("\""));
      for (int s = String(measuredDistances[i]).length(); s < 14; s++) Serial.print(' ');
      Serial.print(F("| "));
      float err = measuredDistances[i] - testDistances[i];
      Serial.print(err >= 0 ? "+" : ""); Serial.print(err, 1); Serial.println(F("\""));

      if (abs(err) < 2.0) {
        maxReliableRange = testDistances[i];
      }
    } else {
      Serial.println(F("FAILED         | N/A"));
    }
  }

  ultrasonicMaxRange = maxReliableRange;

  Serial.println();
  Serial.print(F("Reliable detection range: up to ")); Serial.print(maxReliableRange);
  Serial.println(F(" inches"));
  Serial.println();
  Serial.print(F("RECOMMENDED: OBSTACLE_DISTANCE_THRESHOLD = "));
  Serial.print(maxReliableRange > 0 ? maxReliableRange - 2 : 10.0);
  Serial.println(F(" inches"));
  Serial.println(F("(Setting threshold 2\" below max reliable range for safety)"));

  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CALIBRATION SUMMARY
// ═══════════════════════════════════════════════════════════════════════════════

void displayCalibrationSummary() {
  Serial.println();
  Serial.println(F("╔═══════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║              CALIBRATION COMPLETE - COPY VALUES                       ║"));
  Serial.println(F("╚═══════════════════════════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Copy and paste the following into AutoWiper_Competition.ino:"));
  Serial.println();
  Serial.println(F("// ═══════════════════════════════════════════════════════════════"));
  Serial.println(F("//  CALIBRATED CONSTANTS - FROM CALIBRATION TOOL"));
  Serial.println(F("// ═══════════════════════════════════════════════════════════════"));
  Serial.println();

  Serial.println(F("// Distance calibration"));
  Serial.print(F("#define DISTANCE_PER_1000MS_FULL_SPEED  "));
  Serial.println(distancePer1000ms_FullSpeed > 0 ? distancePer1000ms_FullSpeed : 20.0);
  Serial.print(F("#define DISTANCE_PER_1000MS_HALF_SPEED  "));
  Serial.println(distancePer1000ms_HalfSpeed > 0 ? distancePer1000ms_HalfSpeed : 10.0);
  Serial.println();

  Serial.println(F("// Turn calibration (on-dime turns)"));
  Serial.print(F("#define TURN_LEFT_90_MS   "));
  Serial.println(turnLeft90ms > 0 ? turnLeft90ms : 600);
  Serial.print(F("#define TURN_RIGHT_90_MS  "));
  Serial.println(turnRight90ms > 0 ? turnRight90ms : 600);
  Serial.println();

  Serial.println(F("// Traveling turn calibration"));
  Serial.print(F("#define TRAVELING_TURN_LEFT_SPEED   "));
  Serial.println(travelTurnLeftSpeed > 0 ? travelTurnLeftSpeed : 200);
  Serial.print(F("#define TRAVELING_TURN_RIGHT_SPEED  "));
  Serial.println(travelTurnRightSpeed > 0 ? travelTurnRightSpeed : 80);
  Serial.print(F("#define TRAVELING_TURN_90_DURATION  "));
  Serial.println(travelTurn90Duration > 0 ? travelTurn90Duration : 1200);
  Serial.println();

  Serial.println(F("// Color sensor thresholds"));
  Serial.print(F("#define BLACK_THRESHOLD  "));
  Serial.println(blackThreshold > 0 ? blackThreshold : 600);
  Serial.println();

  Serial.println(F("// Ultrasonic sensor"));
  Serial.print(F("#define OBSTACLE_DISTANCE_THRESHOLD  "));
  Serial.println(ultrasonicMaxRange > 0 ? ultrasonicMaxRange - 2.0 : 12.0);
  Serial.println();

  Serial.println(F("// ═══════════════════════════════════════════════════════════════"));
  Serial.println();

  // Additional notes
  Serial.println(F("NOTES:"));
  if (turnLeft90ms > 0 && turnRight90ms > 0) {
    int diff = abs((int)turnLeft90ms - (int)turnRight90ms);
    Serial.print(F("- Turn asymmetry: ")); Serial.print(diff);
    Serial.println(F("ms (separate L/R constants are essential)"));
  }
  if (distancePer1000ms_FullSpeed > 0) {
    Serial.print(F("- Field size in units: "));
    Serial.print(48.0 / distancePer1000ms_FullSpeed * 1000.0, 0);
    Serial.println(F("ms at full speed"));
  }
  Serial.println();

  statusBeep(2000, 200);
  delay(100);
  statusBeep(2500, 200);
  delay(100);
  statusBeep(3000, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MOVEMENT PRIMITIVES
// ═══════════════════════════════════════════════════════════════════════════════

void maneuver(int speedLeft, int speedRight, int msTime) {
  // Speed range: -200 to +200
  // Positive = forward, Negative = backward

  // Left servo: positive speed = forward
  // Right servo: positive speed = forward (inverted in hardware)
  servoLeft.writeMicroseconds(SERVO_STOP + speedLeft);
  servoRight.writeMicroseconds(SERVO_STOP - speedRight);

  if (msTime > 0) {
    delay(msTime);
  }
}

void stopMotors() {
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  delay(50);
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

  delay(COLOR_STABILIZE_MS);

  // Read frequency as pulse duration
  unsigned long duration = pulseIn(PIN_COLOR_OUT, LOW, PULSEIN_TIMEOUT);

  // pulseIn returns 0 on timeout - treat as very dark (high duration)
  if (duration == 0) {
    duration = 1000;  // Indicate timeout/very dark
  }

  return duration;
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
  unsigned long duration = pulseIn(PIN_ULTRASONIC, HIGH, 30000); // 30ms timeout

  // Calculate distance in inches
  // Speed of sound = 343 m/s = 0.0343 cm/µs = 0.0135 in/µs
  // Distance = (duration * 0.0135) / 2 (divide by 2 for round trip)
  float distance = duration * 0.00675;

  return distance;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

void statusBeep(int frequency, int duration) {
  tone(PIN_BUZZER, frequency, duration);
  delay(duration);
}

void blinkLED(int pin, int count, int delayMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}

void waitForSerialInput() {
  clearSerialBuffer();
  while (!Serial.available()) {
    delay(50);
  }
  clearSerialBuffer();
}

float readSerialFloat() {
  while (!Serial.available()) {
    delay(50);
  }
  float value = Serial.parseFloat();
  clearSerialBuffer();
  Serial.print(F("Entered: ")); Serial.println(value);
  return value;
}

int readSerialInt() {
  while (!Serial.available()) {
    delay(50);
  }
  int value = Serial.parseInt();
  clearSerialBuffer();
  Serial.print(F("Entered: ")); Serial.println(value);
  return value;
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
