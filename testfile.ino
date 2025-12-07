/* ═══════════════════════════════════════════════════════════════════════════════
   AUTOWIPER GYRO V10 - TAPE SEAM FIX
   Target: Arduino Uno | Sensor: LSM6DSOX + TCS3200
   Logic Update: 
     - Reduced COLOR_DIFF_MIN from 70 to 30.
     - Prevents tape seams (diff ~69) from triggering Black.
   ═══════════════════════════════════════════════════════════════════════════════ */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  PINS & OBJECTS
// ═══════════════════════════════════════════════════════════════════════════════
#define PIN_SERVO_LEFT    13
#define PIN_SERVO_RIGHT   12
#define PIN_ARM_LEFT      11
#define PIN_ARM_RIGHT     10
#define PIN_BUZZER        3

// TCS3200 Color Sensor
#define PIN_COLOR_OUT     4
#define PIN_COLOR_S2      5
#define PIN_COLOR_S3      A0

#define SERVO_STOP        1500

// --- CRITICAL THRESHOLDS ---
#define COMP_BLACK_LIMIT  400  // Competition: Both > 400 is Black
#define TEST_BRIGHT_LIMIT 300  // Test: Both < 300 is "Bright Mode"

// UPDATED: Tightened from 70 to 30 to ignore tape seams
#define COLOR_DIFF_MIN    30   
// Explanation:
// Tape Seam (R:107 B:176) Diff is 69.  69 > 30 -> SAFE (Color)
// Real Grey (R:150 B:160) Diff is 10.  10 < 30 -> BLACK (Line)

Servo servoLeft, servoRight, armLeft, armRight;
Adafruit_LSM6DSOX lsm6ds;

// Gyro State
float gyroHeading = 0.0;
unsigned long lastGyroUpdate = 0;
bool gyroReady = false;

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  while (!Serial);

  servoLeft.attach(PIN_SERVO_LEFT);
  servoRight.attach(PIN_SERVO_RIGHT);
  armLeft.attach(PIN_ARM_LEFT);
  armRight.attach(PIN_ARM_RIGHT);
  stopMotors();

  armLeft.write(152);
  armRight.write(28);

  pinMode(PIN_BUZZER, OUTPUT);
  
  // Color sensor pins
  pinMode(PIN_COLOR_OUT, INPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);

  Serial.println(F("\n=== GYRO + V10 SEAM FIX ==="));
  
  Wire.begin(); 
  
  if (lsm6ds.begin_I2C()) {
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS); 
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
    gyroReady = true;
    Serial.println(F("Gyro OK."));
  } else {
    Serial.println(F("GYRO FAILED. Check wiring."));
    while (1);
  }

  displayMenu();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    delay(10); 
    while(Serial.available()) Serial.read(); // Clear buffer
    
    switch (input) {
      case '1': test_Forward(); break;
      case '2': turnToAngleGyro(90.0); break;
      case '3': turnToAngleGyro(-90.0); break;
      case '4': turnToAngleGyro(180.0); break;
      case '5': test_ColorSensor(); break;
      case 'm': displayMenu(); break;
      default: break; 
    }
    
    if (input >= '1' && input <= '5' || input == 'm') {
      Serial.println(F("\nReady. Enter command:"));
    }
  }
}

void displayMenu() {
  Serial.println(F("\n-- MENU --"));
  Serial.println(F("1. Fwd 12in (Stop on True Black)"));
  Serial.println(F("2. LEFT 90 (Active Fix)"));
  Serial.println(F("3. RIGHT 90 (Active Fix)"));
  Serial.println(F("4. 180 Turn (Active Fix)"));
  Serial.println(F("5. Continuous Color Monitor (Any Key to Stop)"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  COLOR SENSOR LOGIC
// ═══════════════════════════════════════════════════════════════════════════════

unsigned long measureColor(bool readRed) {
  if(readRed) {
    digitalWrite(PIN_COLOR_S2, LOW);
    digitalWrite(PIN_COLOR_S3, LOW);
  } else {
    digitalWrite(PIN_COLOR_S2, LOW);
    digitalWrite(PIN_COLOR_S3, HIGH);
  }
  delay(15); 
  unsigned long duration = pulseIn(PIN_COLOR_OUT, LOW, 20000); 
  if(duration == 0) duration = 1000; 
  return duration;
}

// HELPER: Returns true ONLY if surface is black
bool isSurfaceBlack(unsigned long r, unsigned long b) {
  
  // 1. Competition Logic (Dark)
  if (r > COMP_BLACK_LIMIT && b > COMP_BLACK_LIMIT) {
    return true; 
  }

  // 2. Test Paper Logic (Bright but Grey)
  if (r < TEST_BRIGHT_LIMIT && b < TEST_BRIGHT_LIMIT) {
    int diff = abs((int)r - (int)b);
    
    // STRICT CHECK: Red and Blue must be VERY close (diff < 30)
    if (diff < COLOR_DIFF_MIN) { 
      return true; 
    }
  }

  return false; 
}

void test_ColorSensor() {
  Serial.println(F("\n-- COLOR MONITOR (V10 LOGIC) --"));
  Serial.println(F("Send ANY KEY to return to menu..."));
  
  delay(100); 
  while(Serial.available()) Serial.read();

  while (!Serial.available()) {
    unsigned long red = measureColor(true);
    unsigned long blue = measureColor(false);
    
    Serial.print("R: "); Serial.print(red);
    Serial.print("\tB: "); Serial.print(blue);
    
    if (isSurfaceBlack(red, blue)) {
       Serial.println(F("\t[BLACK LINE]"));
    } else {
       if (red < blue) Serial.println(F("\t[RED Surface]"));
       else Serial.println(F("\t[BLUE Surface]"));
    }
    
    delay(100); 
  }
  while(Serial.available()) Serial.read();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FORWARD MOVEMENT (TEST 1)
// ═══════════════════════════════════════════════════════════════════════════════

void test_Forward() {
  Serial.println(F("Fwd 12in... (Scanning V10 LOGIC)"));
  Serial.print(F("3..")); statusBeep(1000,100); delay(900);
  Serial.print(F("2..")); statusBeep(1000,100); delay(900);
  Serial.println(F("GO"));
  
  unsigned long startTime = millis();
  unsigned long targetDuration = 2200; 
  
  while(millis() - startTime < targetDuration) {
    
    unsigned long r = measureColor(true);
    unsigned long b = measureColor(false);

    Serial.print("R:"); Serial.print(r);
    Serial.print(" B:"); Serial.print(b);

    if (isSurfaceBlack(r, b)) {
      stopMotors();
      Serial.println(F(" -> BLACK DETECTED! Reversing..."));
      statusBeep(2000, 500); 
      maneuver(-250, -250, 1000); 
      stopMotors();
      return; 
    } else {
      Serial.println(F(" -> OK"));
      maneuver(250, 250, 0); 
      delay(50);             
    }
  }
  
  stopMotors();
  Serial.println(F("Forward complete."));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  GYRO TURN LOGIC
// ═══════════════════════════════════════════════════════════════════════════════

void updateGyroscope() {
  sensors_event_t accel, gyro, temp;
  if (!lsm6ds.getEvent(&accel, &gyro, &temp)) return;
  unsigned long now = millis();
  float dt = (now - lastGyroUpdate) / 1000.0;
  if (dt > 0.001 && dt < 0.5) {
    gyroHeading += gyro.gyro.z * dt * 57.2958;
    while (gyroHeading > 180.0) gyroHeading -= 360.0;
    while (gyroHeading < -180.0) gyroHeading += 360.0;
  }
  lastGyroUpdate = now;
}

void zeroGyroHeading() {
  gyroHeading = 0.0;
  lastGyroUpdate = millis();
  for (int i = 0; i < 10; i++) { 
    updateGyroscope();
    delay(10);
  }
  gyroHeading = 0.0;
}

bool turnToAngleGyro(float targetAngle) {
  if (!gyroReady) return false;
  Serial.print(F("Turning to ")); Serial.println(targetAngle);
  zeroGyroHeading();
  unsigned long startTime = millis();
  bool turnRight = (targetAngle < 0); 
  float absTarget = abs(targetAngle);
  
  while (abs(abs(gyroHeading) - absTarget) > 15.0) {
    updateGyroscope();
    if (millis() - startTime > 3000) break;
    maneuver(turnRight ? 400 : -400, turnRight ? -400 : 400, 0);
    delay(20);
  }
  while (abs(abs(gyroHeading) - absTarget) > 5.0) {
    updateGyroscope();
    if (millis() - startTime > 4000) break;
    maneuver(turnRight ? 200 : -200, turnRight ? -200 : 200, 0);
    delay(20);
  }
  while (abs(abs(gyroHeading) - absTarget) > 2.0) {
    updateGyroscope();
    if (millis() - startTime > 5000) break;
    maneuver(turnRight ? 120 : -120, turnRight ? -120 : 120, 0);
    delay(30);
  }
  
  stopMotors();
  delay(250); 
  
  for(int attempt = 1; attempt <= 5; attempt++) {
    updateGyroscope();
    float rawError = targetAngle - gyroHeading;
    if (abs(rawError) <= 1.0) {
      statusBeep(2000, 100);
      Serial.println(F(">> LOCKED <<"));
      return true;
    }
    int fixSpeed = 150;
    if (rawError > 0) maneuver(-fixSpeed, fixSpeed, 50); 
    else maneuver(fixSpeed, -fixSpeed, 50);              
    stopMotors();
    delay(200);
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════
void maneuver(int leftSpeed, int rightSpeed, int msTime) {
  servoLeft.writeMicroseconds(SERVO_STOP + leftSpeed);
  servoRight.writeMicroseconds(SERVO_STOP - rightSpeed);
  if (msTime > 0) delay(msTime);
}

void stopMotors() {
  servoLeft.writeMicroseconds(SERVO_STOP);
  servoRight.writeMicroseconds(SERVO_STOP);
  delay(50);
}

void statusBeep(int f, int d) {
  tone(PIN_BUZZER, f, d); delay(d);
}
