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
// WARNING: Pin 1 is Serial TX - IR2 won't work when DEBUG_MODE or TEST_SENSORS_ONLY is enabled!
// For testing, use only IR pair 1 (pins 8,9) or disable Serial debugging
#define PIN_IR_LED_1 9
#define PIN_IR_RX_1 8
#define PIN_IR_LED_2 2
#define PIN_IR_RX_2 1    // CAUTION: Conflicts with Serial TX when debugging enabled!

// I2C addresses for LSM6DSOX gyroscope (A4=SDA, A5=SCL on Arduino Uno)
#define LSM6DSOX_ADDR_PRIMARY 0x6A   // Default address (SA0 pin LOW or floating)
#define LSM6DSOX_ADDR_SECONDARY 0x6B // Alternate address (SA0 pin HIGH)

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
#define COLOR_STABILIZE_MS 20     // Reduced stabilization delay (was 50ms)
#define PULSEIN_TIMEOUT 30000     // Timeout for pulseIn (30ms)

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
bool gyroAvailable = false;  // Track if gyroscope initialized successfully

// Phase completion tracking (for adaptive transitions)
bool phase1Complete = false;

// Cached color sensor readings (for efficiency)
unsigned long lastRedReading = 0;
unsigned long lastBlueReading = 0;
unsigned long lastColorReadTime = 0;
#define COLOR_CACHE_VALID_MS 50  // Cache valid for 50ms

// Watchdog for sweep loops (prevent getting stuck)
unsigned long lastSweepProgress = 0;
float lastWatchdogY = 0;
#define WATCHDOG_TIMEOUT_MS 5000  // 5 seconds without progress = stuck
#define WATCHDOG_MIN_PROGRESS 1.0 // Must move at least 1" to count as progress

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
void readColorSensorCached(unsigned long &red, unsigned long &blue);
bool detectBlackBoundary();
bool detectBlackBoundaryFast();
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
bool checkWatchdog();
void resetWatchdog();

// ==================== SETUP ====================
void setup() {
  // 1. Serial initialization
  #if DEBUG_MODE || TEST_SENSORS_ONLY
    Serial.begin(9600);
    Serial.println(F("AUTOWIPER INIT"));
  #endif

  // 2. I2C for gyroscope - IMPORTANT: delay after Wire.begin() for bus stabilization
  Wire.begin();
  delay(100);  // Allow I2C bus to stabilize

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

  // 7. Initialize gyroscope (LSM6DSOX on I2C: A4=SDA, A5=SCL)
  #if DEBUG_MODE || TEST_SENSORS_ONLY
    Serial.println(F("I2C scan:"));
    int deviceCount = 0;
    for(byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if(Wire.endTransmission() == 0) {
        Serial.print(F(" 0x")); Serial.println(addr, HEX);
        deviceCount++;
      }
    }
    if(deviceCount == 0) Serial.println(F(" NONE! Check SDA=A4 SCL=A5"));
    Serial.print(F("Found: ")); Serial.println(deviceCount);
  #endif

  // Try to initialize gyroscope
  gyroAvailable = false;
  if(lsm6ds.begin_I2C()) {
    gyroAvailable = true;
  } else if(lsm6ds.begin_I2C(LSM6DSOX_ADDR_PRIMARY, &Wire)) {
    gyroAvailable = true;
  } else if(lsm6ds.begin_I2C(LSM6DSOX_ADDR_SECONDARY, &Wire)) {
    gyroAvailable = true;
  }

  if(gyroAvailable) {
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    #if DEBUG_MODE || TEST_SENSORS_ONLY
      Serial.println(F("Gyro: OK"));
    #endif
  } else {
    #if DEBUG_MODE || TEST_SENSORS_ONLY
      Serial.println(F("Gyro: FAIL"));
    #endif
  }

  #if DEBUG_MODE || TEST_SENSORS_ONLY
    Serial.println(F("IR2 off (pin1=TX)"));
  #endif

  // 8. MANDATORY 2-SECOND DELAY (competition requirement)
  statusBeep(3000, 500);
  digitalWrite(PIN_LED_1, HIGH);
  delay(2000);
  digitalWrite(PIN_LED_1, LOW);

  #if TEST_SENSORS_ONLY
    // Compact test mode - minimal strings to save flash
    // Color: low=more light. R<B=red, both>600=black
    // IR: 0=detected, 1=clear
    Serial.println(F("TEST MODE"));
    Serial.println(F("R<B=red Both>600=black"));
    Serial.print(F("IR1 idle:")); Serial.println(digitalRead(PIN_IR_RX_1));

    while(true) {
      unsigned long red = measureColorDuration(true);
      unsigned long blue = measureColorDuration(false);
      int ir1 = irDetect(PIN_IR_LED_1, PIN_IR_RX_1, 38000);

      // Compact output: R:xxx B:xxx [surface] IR:x G:x.xx
      Serial.print(F("R:")); Serial.print(red);
      Serial.print(F(" B:")); Serial.print(blue);
      Serial.print((red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD) ? F(" BLK") : (red < blue ? F(" RED") : F(" BLU")));
      Serial.print(F(" IR:")); Serial.print(ir1 == 0 ? F("DET") : F("clr"));

      if(gyroAvailable) {
        sensors_event_t accel, gyro, temp;
        if(lsm6ds.getEvent(&accel, &gyro, &temp)) {
          Serial.print(F(" G:")); Serial.print(gyro.gyro.z, 2);
        }
      } else {
        Serial.print(F(" G:N/A"));
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
    startedOnRedSide = (red < blue); // Red side if red reflects more
  } else {
    // Starting on black boundary - assume positioned correctly
    startedOnRedSide = false; // Default assumption
  }

  #if DEBUG_MODE
    Serial.print(F("Side:")); Serial.println(startedOnRedSide ? F("RED") : F("BLU"));
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
    Serial.print(F("INIT OK X:")); Serial.print(colorSensor_X);
    Serial.print(F(" Y:")); Serial.println(colorSensor_Y);
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

  // 3. Phase management (adaptive based on completion and time)
  if(elapsed < 5) {
    currentPhase = INIT;
  } else if(elapsed < 35 && !phase1Complete) {
    // Offensive phase: continue until all 4 passes done OR time limit
    currentPhase = OFFENSIVE;
  } else {
    // Defensive phase: either Phase 1 complete or time expired
    currentPhase = DEFENSIVE;
  }

  // 4. Time limit enforcement (60 seconds)
  if(elapsed >= 60) {
    stopMotors();
    digitalWrite(PIN_LED_2, HIGH);
    statusBeep(4000, 1000);

    #if DEBUG_MODE
      Serial.println(F("MATCH END"));
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
        Serial.println(F("!BOUNDARY"));
      #endif
      handleBoundaryEmergency();
      return;
    }
  }

  // 6. Opponent avoidance (HIGH PRIORITY)
  if(opponentDetected && currentPhase != INIT) {
    #if DEBUG_MODE
      Serial.println(F("!OPP"));
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

  // 8. Position verification (every 3 seconds)
  static unsigned long lastPositionCheck = 0;
  if(currentTime - lastPositionCheck > 3000 && elapsed > 5) {
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

    // Execute recovery if position is uncertain
    if(positionUncertain) {
      recoverPosition();
    }
  }

  // 9. Debug output (compact)
  #if DEBUG_MODE
    static unsigned long lastDebug = 0;
    if(currentTime - lastDebug > 1000) {
      Serial.print(elapsed); Serial.print(F("s P"));
      Serial.print(currentPhase); Serial.print(F(" Y"));
      Serial.print(colorSensor_Y, 0); Serial.print(F(" X"));
      Serial.print(colorSensor_X, 0); Serial.print(F(" H"));
      Serial.println(heading, 0);
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
        Serial.println(F("!BND"));
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

  // Skip navigation if already at target (within tolerance)
  if(distance < 2.0) {
    return;
  }

  #if DEBUG_MODE
    Serial.print(F("NAV>")); Serial.print(target_X, 0);
    Serial.print(F(",")); Serial.println(target_Y, 0);
  #endif

  // Rotate to target heading
  rotateToHeading(targetHeading);
  delay(100);

  // Move forward with opponent detection
  int duration = (int)(distance / SPEED_INCHES_PER_SEC * 1000.0);
  unsigned long startMove = millis();

  while(millis() - startMove < duration) {
    // Check for opponent during navigation
    if(scanForOpponent()) {
      #if DEBUG_MODE
        Serial.println(F("!OPP"));
      #endif
      stopMotors();
      evadeOpponent();
      return;
    }

    updateGyroscope();

    // Calculate heading error
    float headingError = normalizeAngle(heading - targetHeading);

    // Apply correction
    int correction = (int)(headingError * 2.5);
    correction = constrain(correction, -50, 50);

    maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 50);
    updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 50);

    // Safety check for boundary
    if(detectBlackBoundaryFast()) {
      #if DEBUG_MODE
        Serial.println(F("!BND"));
      #endif
      break;
    }
  }

  stopMotors();
  delay(100);
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

  delay(COLOR_STABILIZE_MS); // Reduced stabilization delay

  // Read frequency as pulse duration
  unsigned long duration = pulseIn(PIN_COLOR_OUT, LOW, PULSEIN_TIMEOUT);

  // CRITICAL FIX: pulseIn returns 0 on timeout, NOT a high value
  // Timeout indicates no pulse received = very dark surface (black)
  // Return a high value to indicate black detection
  if(duration == 0) {
    duration = BLACK_THRESHOLD + 200; // Treat timeout as black
  }

  return duration;
}

// Optimized color reading with caching
void readColorSensorCached(unsigned long &red, unsigned long &blue) {
  unsigned long now = millis();

  // Return cached values if still valid
  if(now - lastColorReadTime < COLOR_CACHE_VALID_MS && lastColorReadTime > 0) {
    red = lastRedReading;
    blue = lastBlueReading;
    return;
  }

  // Read new values
  red = measureColorDuration(true);
  blue = measureColorDuration(false);

  // Cache the readings
  lastRedReading = red;
  lastBlueReading = blue;
  lastColorReadTime = now;
}

bool detectBlackBoundary() {
  unsigned long red, blue;
  readColorSensorCached(red, blue);

  // Black border: BOTH colors show very low intensity (high duration)
  bool isBlack = (red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD);

  return isBlack;
}

// Fast boundary check without reading sensor (uses cached values)
bool detectBlackBoundaryFast() {
  // Only use cached values if recent
  if(millis() - lastColorReadTime < COLOR_CACHE_VALID_MS && lastColorReadTime > 0) {
    return (lastRedReading > BLACK_THRESHOLD && lastBlueReading > BLACK_THRESHOLD);
  }
  // Fall back to full read
  return detectBlackBoundary();
}

int checkFieldSide() {
  // Returns: 1 = own side, -1 = opponent side, 0 = boundary/uncertain

  unsigned long red, blue;
  readColorSensorCached(red, blue);

  // Check for black boundary first (uses same cached values)
  if(red > BLACK_THRESHOLD && blue > BLACK_THRESHOLD) {
    return 0; // At boundary, can't determine side
  }

  bool readingRed = (red < blue);

  // Compare with starting field color
  if(readingRed == startedOnRedSide) {
    return 1; // On own side
  } else {
    return -1; // On opponent's side
  }
}

bool scanForOpponent() {
  // IR sensors detect opponent robot ONLY (not boundaries)
  // NOTE: IR pair 2 (pins 1,2) conflicts with Serial TX when debugging!

  int frontLeft = irDetect(PIN_IR_LED_1, PIN_IR_RX_1, 38000);

  // Only use IR pair 2 when Serial is NOT active (pins 0,1 free)
  #if !DEBUG_MODE && !TEST_SENSORS_ONLY
    int frontRight = irDetect(PIN_IR_LED_2, PIN_IR_RX_2, 38000);
    return (frontLeft == 0 || frontRight == 0);
  #else
    // When debugging, only IR pair 1 works (pair 2 conflicts with Serial TX)
    return (frontLeft == 0);
  #endif
}

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  // Standard BOE-Bot IR detection method:
  // 1. Generate 38kHz square wave on IR LED pin for 8ms
  // 2. IR light bounces off obstacle and returns to receiver
  // 3. IR receiver (TSOP-style) outputs LOW when detecting 38kHz IR
  //
  // IMPORTANT: This requires proper hardware setup:
  // - IR LED with current-limiting resistor (220-330 ohm)
  // - 38kHz IR receiver (like TSOP38238, TSOP4838, etc.)
  // - IR LED and receiver pointing in SAME direction (for reflection)

  tone(irLedPin, frequency, 8); // Emit 38kHz IR for 8ms
  delay(1);                      // Wait for detection
  int ir = digitalRead(irReceiverPin);  // Read receiver
  delay(1);                      // Cooldown
  return ir;  // 0 = object detected (IR reflected back), 1 = no detection
}

void updateGyroscope() {
  // Skip if gyroscope not available - rely on dead reckoning
  if(!gyroAvailable) {
    return;
  }

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
  if(armsDeployed) return;
  armLeft.write(ARM_DEPLOYED);
  armRight.write(ARM_DEPLOYED);
  delay(500);
  armsDeployed = true;
  #if DEBUG_MODE
    Serial.println(F("ARM+"));
  #endif
}

void retractArms() {
  if(!armsDeployed && armLeft.read() == ARM_LEFT_RETRACT) return;
  armLeft.write(ARM_LEFT_RETRACT);
  armRight.write(ARM_RIGHT_RETRACT);
  delay(500);
  armsDeployed = false;
  #if DEBUG_MODE
    Serial.println(F("ARM-"));
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
    Serial.println(F("CAL"));
  #endif

  bool blackDetected = false;
  int attempts = 0;

  while(!blackDetected && attempts < 10) {
    if(detectBlackBoundary()) {
      blackDetected = true;
      break;
    }
    maneuver(-100, -100, 100);
    colorSensor_Y -= estimateDistance(100, 100);
    attempts++;
    delay(50);
  }

  colorSensor_Y = 2.0;
  #if DEBUG_MODE
    Serial.println(blackDetected ? F("CAL OK") : F("CAL DEF"));
  #endif

  maneuver(150, 150, 300);
  colorSensor_Y += estimateDistance(300, 150);
  stopMotors();
}

void executePhase1_OffensiveSweep() {
  // Execute 4 sweeping passes across opponent's territory
  // Lanes at X = 6", 18", 30", 42"

  if(completedOffensivePasses >= 4) {
    phase1Complete = true;
    #if DEBUG_MODE
      Serial.println(F("P1 DONE"));
    #endif
    return;
  }

  static bool inPosition = false;
  static bool sweepComplete = false;

  if(!inPosition) {
    float laneX = 6.0 + (completedOffensivePasses * 12.0);
    float startY = 16.0;
    #if DEBUG_MODE
      Serial.print(F("L")); Serial.println(completedOffensivePasses + 1);
    #endif
    navigateToCoordinate(laneX, startY);
    rotateToHeading(0.0);
    deployArms();
    inPosition = true;
    sweepComplete = false;
    delay(300);
  }

  if(inPosition && !sweepComplete) {
    #if DEBUG_MODE
      Serial.println(F("SWP"));
    #endif
    resetWatchdog();

    while(colorSensor_Y < 38.0) {
      if(scanForOpponent()) {
        statusBeep(4000, 100);
        evadeOpponent();
        break;
      }
      if(detectBlackBoundaryFast()) {
        #if DEBUG_MODE
          Serial.println(F("!BND"));
        #endif
        break;
      }
      if(checkWatchdog()) {
        #if DEBUG_MODE
          Serial.println(F("!WDG"));
        #endif
        statusBeep(1000, 200);
        moveBackward(4.0);
        break;
      }

      updateGyroscope();
      float headingError = normalizeAngle(heading - 0.0);
      int correction = (int)(headingError * 2.5);
      correction = constrain(correction, -50, 50);
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
  static bool returningSweep = false;
  const float DEFENSE_START_Y = 8.0;
  const float DEFENSE_END_Y = 28.0;

  if(!isInitialized) {
    currentLane = 1;
    returningSweep = false;
    #if DEBUG_MODE
      Serial.println(F("P2 DEF"));
    #endif
    navigateToCoordinate(6.0, DEFENSE_START_Y);
    rotateToHeading(0.0);
    deployArms();
    isInitialized = true;
  }

  if(!returningSweep) {
    resetWatchdog();
    while(colorSensor_Y < DEFENSE_END_Y) {
      if(scanForOpponent()) {
        statusBeep(4000, 100);
        evadeOpponent();
        return;
      }
      if(detectBlackBoundaryFast()) {
        #if DEBUG_MODE
          Serial.println(F("!BND"));
        #endif
        break;
      }
      if(checkWatchdog()) {
        #if DEBUG_MODE
          Serial.println(F("!WDG"));
        #endif
        statusBeep(1000, 200);
        moveBackward(4.0);
        break;
      }

      updateGyroscope();
      float headingError = normalizeAngle(heading - 0.0);
      int correction = (int)(headingError * 2.5);
      correction = constrain(correction, -50, 50);
      maneuver(SERVO_FULL_SPEED - correction, SERVO_FULL_SPEED + correction, 100);
      updatePositionFromMovement(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 100);
    }

    stopMotors();
    #if DEBUG_MODE
      Serial.print(F("DEF Y")); Serial.println(colorSensor_Y, 0);
    #endif
    retractArms();
    currentLane++;
    if(currentLane > 4) currentLane = 1;
    float nextLaneX = 6.0 + ((currentLane - 1) * 12.0);
    navigateToCoordinate(nextLaneX, DEFENSE_START_Y + 2.0);
    rotateToHeading(0.0);
    deployArms();
  }
  returningSweep = false;
}

// ==================== ERROR HANDLING ====================

void handleBoundaryEmergency() {
  stopMotors();
  statusBeep(500, 300);
  ensureArmsRetracted();
  #if DEBUG_MODE
    Serial.print(F("!EMG Y")); Serial.println(colorSensor_Y, 0);
  #endif

  if(getRobotFrontY() > 42.0) {
    #if DEBUG_MODE
      Serial.println(F("FRT"));
    #endif
    maneuver(-SERVO_FULL_SPEED, -SERVO_FULL_SPEED, 800);
    colorSensor_Y -= estimateDistance(800, SERVO_FULL_SPEED);
    colorSensor_Y = 36.0;
  } else if(colorSensor_Y < 8.0) {
    #if DEBUG_MODE
      Serial.println(F("BCK"));
    #endif
    maneuver(SERVO_FULL_SPEED, SERVO_FULL_SPEED, 600);
    colorSensor_Y += estimateDistance(600, SERVO_FULL_SPEED);
    colorSensor_Y = 10.0;
  } else if(colorSensor_X < 8.0 || colorSensor_X > 40.0) {
    #if DEBUG_MODE
      Serial.println(F("SIDE"));
    #endif
    if(colorSensor_X < 24.0) {
      rotateRight(45.0);
      moveForward(6.0);
      rotateLeft(45.0);
      colorSensor_X += 4.0;
    } else {
      rotateLeft(45.0);
      moveForward(6.0);
      rotateRight(45.0);
      colorSensor_X -= 4.0;
    }
  } else {
    #if DEBUG_MODE
      Serial.println(F("?EMG"));
    #endif
    moveBackward(6.0);
    positionUncertain = true;
  }

  stopMotors();
  delay(200);
  lastColorReadTime = 0;
}

void evadeOpponent() {
  stopMotors();
  statusBeep(4000, 100);
  #if DEBUG_MODE
    Serial.println(F("EVD"));
  #endif
  retractArms();
  float distToLeftEdge = colorSensor_X - 6.0;
  float distToRightEdge = 42.0 - colorSensor_X;
  if(distToLeftEdge > distToRightEdge) {
    moveBackward(4.0);
    rotateLeft(90.0);
    moveForward(10.0);
    rotateRight(90.0);
  } else {
    moveBackward(4.0);
    rotateRight(90.0);
    moveForward(10.0);
    rotateLeft(90.0);
  }
  delay(300);
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
    #if DEBUG_MODE
      Serial.println(F("@BND"));
    #endif
    if(colorSensor_Y > 30.0) {
      colorSensor_Y = 40.0;
      moveBackward(6.0);
    } else {
      colorSensor_Y = 4.0;
      moveForward(6.0);
    }
  } else if(fieldSide != 0) {
    bool expectedOnOwnSide = (colorSensor_Y < 24.0);
    bool actuallyOnOwnSide = (fieldSide == 1);
    if(expectedOnOwnSide != actuallyOnOwnSide) {
      #if DEBUG_MODE
        Serial.println(F("XSIDE"));
      #endif
      if(actuallyOnOwnSide && !expectedOnOwnSide) {
        colorSensor_Y = 18.0;
      } else {
        colorSensor_Y = 30.0;
        moveBackward(8.0);
      }
    }
  }

  if(abs(heading) > 10.0) {
    rotateToHeading(0.0);
  }
  positionUncertain = false;
  #if DEBUG_MODE
    Serial.print(F("RCV>")); Serial.print(colorSensor_X, 0);
    Serial.print(F(",")); Serial.println(colorSensor_Y, 0);
  #endif
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

// Watchdog check: returns true if robot appears stuck (no Y progress)
bool checkWatchdog() {
  unsigned long now = millis();

  // Check if we've made progress
  if(abs(colorSensor_Y - lastWatchdogY) >= WATCHDOG_MIN_PROGRESS) {
    // Made progress - reset watchdog
    lastSweepProgress = now;
    lastWatchdogY = colorSensor_Y;
    return false;
  }

  // Check if watchdog timeout exceeded
  if(now - lastSweepProgress > WATCHDOG_TIMEOUT_MS) {
    #if DEBUG_MODE
      Serial.println(F("!STUCK"));
    #endif
    return true;
  }

  return false;
}

// Reset watchdog (call at start of new sweep)
void resetWatchdog() {
  lastSweepProgress = millis();
  lastWatchdogY = colorSensor_Y;
}
