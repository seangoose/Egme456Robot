/* AutoWiper Calibration Tool - EGME 456
   Run tests to measure robot parameters, then copy values to competition code.

   Hardware: D13/D12=drive, D11/D10=arms, D4/D5/A0=color, D8=ultrasonic, D3=buzzer, D6/D7=LED */

#include <Servo.h>

// Pins
#define P_SL 13
#define P_SR 12
#define P_AL 11
#define P_AR 10
#define P_CO 4
#define P_S2 5
#define P_S3 A0
#define P_US 8
#define P_BZ 3
#define P_L1 6
#define P_L2 7

#define STOP 1500
#define SPD 200

Servo sL, sR, aL, aR;

// Calibration results
unsigned int turnL90 = 0, turnR90 = 0;
unsigned int travelL = 0, travelR = 0, travelDur = 0;
unsigned long blackThr = 0;
float distPer1000 = 0;

void setup() {
  Serial.begin(9600);
  sL.attach(P_SL); sR.attach(P_SR);
  aL.attach(P_AL); aR.attach(P_AR);
  stop();
  aL.write(152); aR.write(28);

  pinMode(P_CO, INPUT);
  pinMode(P_S2, OUTPUT);
  pinMode(P_S3, OUTPUT);
  pinMode(P_US, OUTPUT);
  pinMode(P_BZ, OUTPUT);
  pinMode(P_L1, OUTPUT);
  pinMode(P_L2, OUTPUT);

  beep(2000, 200);

  Serial.println(F("=== CALIBRATION ==="));
  Serial.println(F("1:Color 2:Distance 3:Turns 4:TravelTurn 5:Ultrasonic 6:Summary"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while(Serial.available()) Serial.read();

    switch(c) {
      case '1': testColor(); break;
      case '2': testDistance(); break;
      case '3': testTurns(); break;
      case '4': testTravelTurn(); break;
      case '5': testUltrasonic(); break;
      case '6': printSummary(); break;
    }
    Serial.println(F("\n1:Color 2:Dist 3:Turn 4:Travel 5:Ultra 6:Summary"));
  }
}

void stop() { sL.writeMicroseconds(STOP); sR.writeMicroseconds(STOP); delay(50); }
void move(int l, int r, int t) { sL.writeMicroseconds(STOP+l); sR.writeMicroseconds(STOP-r); if(t>0) delay(t); }
void beep(int f, int d) { tone(P_BZ, f, d); delay(d); }

unsigned long readColor(bool red) {
  digitalWrite(P_S2, LOW);
  digitalWrite(P_S3, red ? LOW : HIGH);
  delay(15);
  unsigned long d = pulseIn(P_CO, LOW, 25000);
  return d == 0 ? 800 : d;
}

float readUltrasonic() {
  pinMode(P_US, OUTPUT);
  digitalWrite(P_US, LOW); delayMicroseconds(2);
  digitalWrite(P_US, HIGH); delayMicroseconds(10);
  digitalWrite(P_US, LOW);
  pinMode(P_US, INPUT);
  return pulseIn(P_US, HIGH, 30000) * 0.00675;
}

void waitEnter() {
  Serial.println(F("Press ENTER..."));
  while(!Serial.available()) delay(50);
  while(Serial.available()) Serial.read();
}

float readFloat() {
  while(!Serial.available()) delay(50);
  float v = Serial.parseFloat();
  while(Serial.available()) Serial.read();
  return v;
}

// Test 1: Color Sensor
void testColor() {
  Serial.println(F("\n=== COLOR TEST ==="));
  unsigned long rR, bR, rB, bB, rK, bK;

  Serial.println(F("Place on RED"));
  waitEnter();
  rR = bR = 0;
  for(int i=0; i<5; i++) { rR += readColor(true); bR += readColor(false); delay(50); }
  rR /= 5; bR /= 5;
  Serial.print(F("RED: r=")); Serial.print(rR); Serial.print(F(" b=")); Serial.println(bR);

  Serial.println(F("Place on BLUE"));
  waitEnter();
  rB = bB = 0;
  for(int i=0; i<5; i++) { rB += readColor(true); bB += readColor(false); delay(50); }
  rB /= 5; bB /= 5;
  Serial.print(F("BLUE: r=")); Serial.print(rB); Serial.print(F(" b=")); Serial.println(bB);

  Serial.println(F("Place on BLACK"));
  waitEnter();
  rK = bK = 0;
  for(int i=0; i<5; i++) { rK += readColor(true); bK += readColor(false); delay(50); }
  rK /= 5; bK /= 5;
  Serial.print(F("BLACK: r=")); Serial.print(rK); Serial.print(F(" b=")); Serial.println(bK);

  unsigned long maxColor = max(max(rR, bR), max(rB, bB));
  unsigned long minBlack = min(rK, bK);
  blackThr = (maxColor + minBlack) / 2;

  Serial.print(F("BLACK_THRESHOLD = ")); Serial.println(blackThr);
  beep(2000, 300);
}

// Test 2: Distance
void testDistance() {
  Serial.println(F("\n=== DISTANCE TEST ==="));
  Serial.println(F("Mark start position"));
  waitEnter();

  Serial.println(F("3..2..1..GO!"));
  beep(1000,200); delay(800);
  beep(1000,200); delay(800);
  beep(2000,500);

  digitalWrite(P_L1, HIGH);
  move(SPD, SPD, 1000);
  stop();
  digitalWrite(P_L1, LOW);

  Serial.println(F("Enter distance (inches):"));
  distPer1000 = readFloat();
  Serial.print(F("DISTANCE_PER_1000MS = ")); Serial.println(distPer1000);
  beep(2000, 300);
}

// Test 3: Turn Calibration
void testTurns() {
  Serial.println(F("\n=== TURN TEST ==="));
  unsigned int t = 600;

  // Left turn
  Serial.println(F("LEFT TURN - align with line"));
  waitEnter();
  for(int i=0; i<3; i++) {
    Serial.print(F("Left ")); Serial.print(t); Serial.println(F("ms"));
    beep(1500,200); delay(300);
    move(-SPD, SPD, t);
    stop();
    Serial.println(F("Enter actual angle:"));
    float a = readFloat();
    if(abs(a-90) < 3) { turnL90 = t; break; }
    t = (unsigned int)(t * 90.0 / a);
    Serial.println(F("Re-align, ENTER to retry"));
    waitEnter();
  }
  if(turnL90 == 0) turnL90 = t;
  Serial.print(F("TURN_LEFT_90_MS = ")); Serial.println(turnL90);

  // Right turn
  t = 600;
  Serial.println(F("RIGHT TURN - align with line"));
  waitEnter();
  for(int i=0; i<3; i++) {
    Serial.print(F("Right ")); Serial.print(t); Serial.println(F("ms"));
    beep(1500,200); delay(300);
    move(SPD, -SPD, t);
    stop();
    Serial.println(F("Enter actual angle:"));
    float a = readFloat();
    if(abs(a-90) < 3) { turnR90 = t; break; }
    t = (unsigned int)(t * 90.0 / a);
    Serial.println(F("Re-align, ENTER to retry"));
    waitEnter();
  }
  if(turnR90 == 0) turnR90 = t;
  Serial.print(F("TURN_RIGHT_90_MS = ")); Serial.println(turnR90);
  beep(2000, 300);
}

// Test 4: Traveling Turn
void testTravelTurn() {
  Serial.println(F("\n=== TRAVELING TURN ==="));
  travelL = 200; travelR = 80; travelDur = 1200;

  Serial.println(F("Mark start pos/angle"));
  waitEnter();

  Serial.print(F("L=")); Serial.print(travelL);
  Serial.print(F(" R=")); Serial.print(travelR);
  Serial.print(F(" T=")); Serial.println(travelDur);

  beep(1500,200); delay(300);
  move(travelL, travelR, travelDur);
  stop();

  Serial.println(F("Enter angle turned:"));
  float a = readFloat();
  Serial.println(F("Enter distance forward:"));
  float d = readFloat();

  // Adjust
  travelDur = (unsigned int)(travelDur * 90.0 / a);

  Serial.print(F("TRAVEL_L=")); Serial.print(travelL);
  Serial.print(F(" TRAVEL_R=")); Serial.print(travelR);
  Serial.print(F(" TRAVEL_DUR=")); Serial.println(travelDur);
  beep(2000, 300);
}

// Test 5: Ultrasonic
void testUltrasonic() {
  Serial.println(F("\n=== ULTRASONIC ==="));
  Serial.println(F("Move obstacle, any key to stop"));

  while(!Serial.available()) {
    float d = readUltrasonic();
    Serial.print(F("Dist: ")); Serial.print(d,1); Serial.println(F("\""));
    delay(300);
  }
  while(Serial.available()) Serial.read();
  beep(2000, 300);
}

// Summary
void printSummary() {
  Serial.println(F("\n=== COPY TO COMPETITION CODE ==="));
  Serial.print(F("#define DISTANCE_PER_1000MS_FULL_SPEED "));
  Serial.println(distPer1000 > 0 ? distPer1000 : 20.0);
  Serial.print(F("#define TURN_LEFT_90_MS "));
  Serial.println(turnL90 > 0 ? turnL90 : 650);
  Serial.print(F("#define TURN_RIGHT_90_MS "));
  Serial.println(turnR90 > 0 ? turnR90 : 650);
  Serial.print(F("#define TRAVELING_TURN_LEFT_SPEED "));
  Serial.println(travelL > 0 ? travelL : 200);
  Serial.print(F("#define TRAVELING_TURN_RIGHT_SPEED "));
  Serial.println(travelR > 0 ? travelR : 80);
  Serial.print(F("#define TRAVELING_TURN_90_DURATION "));
  Serial.println(travelDur > 0 ? travelDur : 1200);
  Serial.print(F("#define BLACK_THRESHOLD "));
  Serial.println(blackThr > 0 ? blackThr : 600);
  beep(3000, 500);
}
