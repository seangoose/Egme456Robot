/* ═══════════════════════════════════════════════════════════════════════════════
   AUTOWIPER V22 - PERIMETER PILOT (PREDICTIVE)
   Target: Arduino Uno | Sensor: TCS3200 + LSM6DSOX
   Logic: 
     - Odometry tracks (X,Y).
     - Turns are triggered by Coordinate Targets (Predictive).
     - Sensors correct position errors (Reality Check).
   
   Pattern: Start -> Left Turn -> Right Turn loops (Box Pattern)
   ═══════════════════════════════════════════════════════════════════════════════ */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  CONSTANTS & PINS
// ═══════════════════════════════════════════════════════════════════════════════
#define PIN_SERVO_LEFT    13
#define PIN_SERVO_RIGHT   12
#define PIN_BUZZER        3
#define PIN_COLOR_OUT     4
#define PIN_COLOR_S2      5   
#define PIN_COLOR_S3      A0  
#define PIN_COLOR_S0      6   
#define PIN_COLOR_S1      7   
#define SERVO_STOP        1500

// --- PHYSICS CONSTANTS ---
#define ROBOT_SPEED       0.0055 // inches per ms
#define WALL_LIMIT        22.0   // Actual Black Line
#define TARGET_LIMIT      18.0   // Virtual Corner (Turn point)
#define START_Y           -22.0
#define START_X           0.0

// --- SENSOR THRESHOLDS ---
#define BLACK_MIN_VAL     50  
#define GREEN_BLACK_LIMIT 65  
#define RED_STRONG_LIMIT  38
#define BLUE_STRONG_LIMIT 42

Servo servoLeft, servoRight;
Adafruit_LSM6DSOX lsm6ds;

// --- STATE MACHINE ---
enum State {
  WAIT_FOR_START,
  LAUNCH_NORTH,
  FIRST_TURN_LEFT,
  DRIVE_WEST,
  TURN_NORTH,
  DRIVE_NORTH,
  TURN_EAST,
  DRIVE_EAST,
  TURN_SOUTH,
  DRIVE_SOUTH,
  TURN_WEST_LOOP // Loops back to DRIVE_WEST
};

State currentState = WAIT_FOR_START;

// --- ODOMETRY GLOBALS ---
float posX = START_X;
float posY = START_Y;
float gyroHeading = 0.0; // 0=North, 90=East, 180=South, -90=West
unsigned long lastUpdate = 0;
int currentZoneColor = -1; // 0=Blue, 1=Red

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);
  stopMotors();

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_COLOR_OUT, INPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);
  pinMode(PIN_COLOR_S0, OUTPUT);
  pinMode(PIN_COLOR_S1, OUTPUT);

  // 100% Scaling
  digitalWrite(PIN_COLOR_S0, HIGH);
  digitalWrite(PIN_COLOR_S1, HIGH);

  Wire.begin(); 
  if(!lsm6ds.begin_I2C()) while(1);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); 
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println(F("=== V22 PERIMETER PILOT ==="));
  Serial.println(F("Start: (0, -22). Heading: North."));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  
  // 1. UPDATE PHYSICS (Dead Reckoning)
  updateOdometry();
  
  // 2. CHECK SENSORS (Reality Correction)
  checkSensorsAndCorrect();

  // 3. EXECUTE MISSION
  switch(currentState) {
    
    case WAIT_FOR_START:
      if (Serial.available() && Serial.read() == '1') {
        Serial.println(F("LAUNCH!"));
        statusBeep(1000, 200);
        currentState = LAUNCH_NORTH;
      }
      break;

    case LAUNCH_NORTH:
      // Drive from Y=-22 to Y=-16 (Safe Zone)
      driveStraight(200);
      if (posY > -16.0) {
        stopMotors();
        currentState = FIRST_TURN_LEFT;
      }
      break;

    case FIRST_TURN_LEFT:
      // User Logic: "First turn Left"
      Serial.println(F("Turn 1: LEFT (Face West)"));
      turnToHeading(-90); 
      currentState = DRIVE_WEST;
      break;

    // --- THE LOOP ---
    
    case DRIVE_WEST:
      // Target: X = -18
      driveStraight(200);
      // Correction: If drifted too far Y (North/South), steer back? 
      // Handled in driveStraight via P-Control on Heading.
      
      if (posX < -TARGET_LIMIT) { // Reached Virtual Corner
        Serial.println(F("Corner Reached. Turning Right."));
        currentState = TURN_NORTH;
      }
      break;

    case TURN_NORTH:
      // User Logic: "Next turn has to be Right"
      turnToHeading(0); // 0 is North
      currentState = DRIVE_NORTH;
      break;

    case DRIVE_NORTH:
      // Target: Y = 18
      driveStraight(200);
      if (posY > TARGET_LIMIT) {
        currentState = TURN_EAST;
      }
      break;

    case TURN_EAST:
      turnToHeading(90); // 90 is East
      currentState = DRIVE_EAST;
      break;

    case DRIVE_EAST:
      // Target: X = 18
      driveStraight(200);
      if (posX > TARGET_LIMIT) {
        currentState = TURN_SOUTH;
      }
      break;

    case TURN_SOUTH:
      turnToHeading(180); // 180 is South
      currentState = DRIVE_SOUTH;
      break;

    case DRIVE_SOUTH:
      // Target: Y = -18
      driveStraight(200);
      if (posY < -TARGET_LIMIT) {
        currentState = TURN_WEST_LOOP;
      }
      break;

    case TURN_WEST_LOOP:
      turnToHeading(270); // 270 (-90) is West
      // We completed a lap! 
      // Future: Increment Lap Counter, reduce TARGET_LIMIT to spiral in.
      Serial.println(F("Lap Complete!"));
      statusBeep(2000, 100);
      currentState = DRIVE_WEST;
      break;
  }
  
  delay(10);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ODOMETRY & PHYSICS
// ═══════════════════════════════════════════════════════════════════════════════
void updateOdometry() {
  unsigned long now = millis();
  float dt = (now - lastUpdate); // ms
  lastUpdate = now;

  // 1. Gyro
  sensors_event_t a, g, t;
  if(lsm6ds.getEvent(&a, &g, &t)) {
    float gyroZ = g.gyro.z * 57.2958; 
    if(abs(gyroZ) > 0.5) { // Noise filter
      gyroHeading += gyroZ * (dt / 1000.0);
    }
  }

  // 2. Position (Only if Motors are Active)
  // Simplified: If current command was "Go", we update.
  // We assume driveStraight calls update this.
  // Actually, let's put the Pos update INSIDE driveStraight to be accurate.
}

void driveStraight(int speed) {
  // 1. Maintain Heading (P-Control)
  // We want to drive straight on the CURRENT Desired Heading
  // Logic: Snap to nearest 90 degree cardinal direction
  float desired = 0;
  if (gyroHeading >= -45 && gyroHeading < 45) desired = 0; // North
  else if (gyroHeading >= 45 && gyroHeading < 135) desired = 90; // East
  else if (gyroHeading >= 135 || gyroHeading < -135) desired = 180; // South
  else desired = -90; // West

  float error = desired - gyroHeading;
  // Handle wrap around (e.g. 180 vs -180) - simplified for now
  
  int adj = error * 2; // Gain
  maneuver(speed + adj, speed - adj, 0);

  // 2. Update Coordinate
  unsigned long now = millis();
  float dt = 10.0; // approx loop time
  
  // Convert Heading to Radians. 0 deg = North (+Y). 90 deg = East (+X).
  // Math: X += sin(angle), Y += cos(angle)
  float rads = gyroHeading * 0.0174533;
  
  posX += (ROBOT_SPEED * dt) * sin(rads);
  posY += (ROBOT_SPEED * dt) * cos(rads);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  REALITY CHECKS (The Magic)
// ═══════════════════════════════════════════════════════════════════════════════
void checkSensorsAndCorrect() {
  int surface = checkSurfaceColor(); // 0=Blue, 1=Red, 2=Black

  // 1. BLACK DETECTED (Wall Hit)
  if (surface == 2) {
    Serial.println(F("!!! BLACK DETECTED (EARLY) !!!"));
    stopMotors();
    
    // We hit a wall. Which one?
    // Look at Heading.
    if (gyroHeading > -45 && gyroHeading < 45) { // Driving North
      posY = WALL_LIMIT; // We hit Top Wall
      Serial.println(F("Hit Top Wall. Reset Y=22."));
    }
    else if (gyroHeading > 45 && gyroHeading < 135) { // Driving East
      posX = WALL_LIMIT; // Hit Right Wall
      Serial.println(F("Hit Right Wall. Reset X=22."));
    }
    else if (abs(gyroHeading) > 135) { // Driving South
      posY = -WALL_LIMIT; // Hit Bottom Wall
      Serial.println(F("Hit Bottom Wall. Reset Y=-22."));
    }
    else { // Driving West
      posX = -WALL_LIMIT; // Hit Left Wall
      Serial.println(F("Hit Left Wall. Reset X=-22."));
    }
    
    // ACTION: Back up and Force Turn
    maneuver(-200, -200, 600); 
    stopMotors();
    forceNextTurn();
  }

  // 2. COLOR TRANSITION (Center Line Crossing)
  // If we cross from Blue->Red or Red->Blue, we know we are at X=0 or Y=0?
  // User map: "Inner area is red rectangle and blue rectangle". 
  // Assuming split down the middle?
  // If we detect a switch, we can correct the non-travel axis to 0. 
  // (Left for V23 Refinement)
}

void forceNextTurn() {
  // If we hit a wall early, skip the drive state and jump to turn
  if(currentState == DRIVE_WEST) currentState = TURN_NORTH;
  else if(currentState == DRIVE_NORTH) currentState = TURN_EAST;
  else if(currentState == DRIVE_EAST) currentState = TURN_SOUTH;
  else if(currentState == DRIVE_SOUTH) currentState = TURN_WEST_LOOP;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════
void turnToHeading(float target) {
  stopMotors();
  Serial.print("Turning to "); Serial.println(target);
  
  unsigned long s = millis();
  while(millis() - s < 3000) {
    updateOdometry(); // Keep tracking gyro
    
    float error = target - gyroHeading;
    // Handle -180/180 wrap logic if needed, but simple for now
    
    if(abs(error) < 2.0) break;
    
    int speed = 200;
    if(abs(error) < 15.0) speed = 150;
    
    if(error > 0) maneuver(-speed, speed, 0); // Left
    else maneuver(speed, -speed, 0); // Right
    
    delay(10);
  }
  stopMotors();
}

unsigned long readSensor(int s2, int s3) {
  digitalWrite(PIN_COLOR_S2, s2);
  digitalWrite(PIN_COLOR_S3, s3);
  delay(4); 
  unsigned long d = pulseIn(PIN_COLOR_OUT, LOW, 5000); 
  if(d == 0) d = 1000; 
  return d;
}

int checkSurfaceColor() {
  unsigned long r = readSensor(LOW, LOW);   
  unsigned long b = readSensor(LOW, HIGH);  
  unsigned long g = readSensor(HIGH, HIGH); 
  
  if (r < RED_STRONG_LIMIT) return 1;
  if (b < BLUE_STRONG_LIMIT) return 0;
  if (r > BLACK_MIN_VAL && b > BLACK_MIN_VAL && g > GREEN_BLACK_LIMIT) return 2;
  return 0; // Default
}

void maneuver(int left, int right, int ms) {
  servoLeft.writeMicroseconds(SERVO_STOP + left);
  servoRight.writeMicroseconds(SERVO_STOP - right);
  if(ms > 0) delay(ms);
}

void stopMotors() {
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  delay(50);
}

void statusBeep(int f, int d) {
  tone(PIN_BUZZER, f, d); delay(d);
}
