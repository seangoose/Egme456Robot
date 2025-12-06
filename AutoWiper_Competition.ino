/* AutoWiper Competition Code - EGME 456 Messy Room
   Strategy: Detect side color, navigate to right, sweep lanes pushing cubes to opponent
   Hardware: D13/D12=drive, D11/D10=arms, D4/D5/A0=color, D8=ultrasonic, D3=buzzer, D6/D7=LED
   Color sensor at REAR - robot extends 8" forward from sensor */

#include <Servo.h>

// === CALIBRATED CONSTANTS - UPDATE FROM CALIBRATION TOOL ===
#define DIST_PER_1000MS  20.0   // inches at full speed
#define TURN_L_90_MS     650    // left 90° turn time
#define TURN_R_90_MS     650    // right 90° turn time
#define TRAV_L_SPD       200    // traveling turn outer wheel
#define TRAV_R_SPD       80     // traveling turn inner wheel
#define TRAV_90_DUR      1200   // traveling turn duration
#define BLACK_THR        600    // black boundary threshold
#define OBSTACLE_THR     12.0   // obstacle distance (inches)

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

// Constants
#define STOP 1500
#define SPD 200
#define ARM_RET_L 152
#define ARM_RET_R 28
#define ARM_CTR 90
#define ARM_FWD_L 30
#define ARM_FWD_R 150

// Field calculations (in ms at full speed)
#define MS_PER_INCH (1000.0/DIST_PER_1000MS)
#define FIELD_MS ((unsigned int)(48.0*MS_PER_INCH))
#define MID_MS (FIELD_MS/2)

Servo sL, sR, aL, aR;

// State
int posX, posY;           // position in ms units
int8_t heading;           // 0=fwd, 1=right, -1=left
uint8_t phase;            // 0=nav, 1=sweep
uint8_t lane;
uint8_t sweeps;
bool isRed;               // our side color
bool armsOut;
unsigned long startTime;

// === CORE FUNCTIONS ===
void stp() { sL.writeMicroseconds(STOP); sR.writeMicroseconds(STOP); delay(30); }
void mv(int l, int r, unsigned int t) { sL.writeMicroseconds(STOP+l); sR.writeMicroseconds(STOP-r); if(t>0) delay(t); }
void beep(int f, int d) { tone(P_BZ,f,d); delay(d); }

unsigned long rdClr(bool red) {
  digitalWrite(P_S2, LOW);
  digitalWrite(P_S3, red ? LOW : HIGH);
  delayMicroseconds(12000);
  unsigned long d = pulseIn(P_CO, LOW, 20000);
  return d == 0 ? BLACK_THR + 100 : d;
}

bool isBlack() {
  return (rdClr(true) > BLACK_THR && rdClr(false) > BLACK_THR);
}

bool isOwnSide() {
  unsigned long r = rdClr(true), b = rdClr(false);
  if(r > BLACK_THR && b > BLACK_THR) return false;
  return (r < b) == isRed;
}

float rdUltra() {
  pinMode(P_US, OUTPUT);
  digitalWrite(P_US, LOW); delayMicroseconds(2);
  digitalWrite(P_US, HIGH); delayMicroseconds(10);
  digitalWrite(P_US, LOW);
  pinMode(P_US, INPUT);
  return pulseIn(P_US, HIGH, 25000) * 0.00675;
}

// Move forward with boundary checking (returns false if boundary hit)
bool fwd(unsigned int ms) {
  unsigned int done = 0;
  while(done < ms) {
    if(isBlack()) { stp(); beep(500,150); return false; }
    unsigned int chunk = min((unsigned int)40, ms - done);
    mv(SPD, SPD, chunk);
    done += chunk;
  }
  stp();
  return true;
}

bool bwd(unsigned int ms) {
  unsigned int done = 0;
  while(done < ms) {
    if(isBlack()) { stp(); beep(500,150); return false; }
    unsigned int chunk = min((unsigned int)40, ms - done);
    mv(-SPD, -SPD, chunk);
    done += chunk;
  }
  stp();
  return true;
}

void turnL() { mv(-SPD, SPD, TURN_L_90_MS); stp(); }
void turnR() { mv(SPD, -SPD, TURN_R_90_MS); stp(); }
void travelL() { mv(TRAV_R_SPD, TRAV_L_SPD, TRAV_90_DUR); stp(); }

void armsRet() { if(armsOut) { aL.write(ARM_RET_L); aR.write(ARM_RET_R); delay(300); armsOut=false; } }
void armsDep() { if(!armsOut) { aL.write(ARM_CTR); aR.write(ARM_CTR); delay(300); armsOut=true; } }
void armsFwd() { aL.write(ARM_FWD_L); aR.write(ARM_FWD_R); delay(250); armsOut=true; }

void boundaryRecover() {
  stp(); beep(500,200); armsRet();
  if(posY > MID_MS) { mv(-SPD,-SPD,400); stp(); posY -= 400; }
  else if(posY < 300) { mv(SPD,SPD,350); stp(); posY += 350; }
  else if(posX > (FIELD_MS*3/4)) { turnL(); mv(SPD,SPD,350); stp(); turnR(); posX -= 350; }
  else if(posX < (FIELD_MS/4)) { turnR(); mv(SPD,SPD,350); stp(); turnL(); posX += 350; }
  else { mv(-SPD,-SPD,350); stp(); posY -= 350; }
}

void obstacleAvoid() {
  stp(); beep(4000,80);
  mv(-SPD,-SPD,250); stp();
  posY -= 250;
  turnR(); mv(SPD,SPD,180); stp(); turnL();
  posX += 130;
}

// === SETUP ===
void setup() {
  sL.attach(P_SL); sR.attach(P_SR);
  aL.attach(P_AL); aR.attach(P_AR);
  stp();

  pinMode(P_CO, INPUT);
  pinMode(P_S2, OUTPUT);
  pinMode(P_S3, OUTPUT);
  pinMode(P_US, OUTPUT);
  pinMode(P_BZ, OUTPUT);
  pinMode(P_L1, OUTPUT);
  pinMode(P_L2, OUTPUT);

  // Retract arms
  aL.write(ARM_RET_L); aR.write(ARM_RET_R);
  armsOut = false;

  // 2-second delay (competition requirement)
  digitalWrite(P_L1, HIGH);
  beep(2000, 400);
  delay(2000);
  digitalWrite(P_L1, LOW);
  beep(3000, 150);

  startTime = millis();

  // Detect side color
  unsigned long r = rdClr(true), b = rdClr(false);
  if(r > BLACK_THR && b > BLACK_THR) {
    // On black, move forward
    for(int i=0; i<8; i++) {
      mv(80, 80, 80);
      r = rdClr(true); b = rdClr(false);
      if(r < BLACK_THR || b < BLACK_THR) break;
    }
    stp();
    r = rdClr(true); b = rdClr(false);
  }
  isRed = (r < b);
  digitalWrite(isRed ? P_L1 : P_L2, HIGH);
  beep(2000, 150);

  // Init position
  posX = FIELD_MS / 2;
  posY = 150;
  heading = 0;
  phase = 0;
  lane = 1;
  sweeps = 0;
}

// === MAIN LOOP ===
void loop() {
  // Check time limit (60s)
  if(millis() - startTime >= 60000) {
    stp(); armsRet();
    digitalWrite(P_L1, HIGH); digitalWrite(P_L2, HIGH);
    beep(4000, 800);
    while(1) delay(1000);
  }

  if(phase == 0) doNavigate();
  else doSweep();

  delay(5);
}

// Phase 0: Navigate to sweep position
void doNavigate() {
  static uint8_t step = 0;

  switch(step) {
    case 0: // Turn right
      turnR();
      heading = 1;
      step = 1;
      break;

    case 1: { // Drive right
      unsigned int dist = (FIELD_MS * 4) / 10;
      if(fwd(dist)) posX += dist;
      else boundaryRecover();
      step = 2;
      break;
    }

    case 2: // Traveling turn left
      travelL();
      heading = 0;
      posY += 350;
      posX -= 180;
      step = 3;
      break;

    case 3: // Start sweeping
      phase = 1;
      armsDep();
      delay(200);
      break;
  }
}

// Phase 1: Sweep lanes
void doSweep() {
  // Execute sweep
  unsigned int target = (FIELD_MS * 3) / 4;

  while(posY < target) {
    if(millis() - startTime >= 60000) return;

    float d = rdUltra();
    if(d > 0 && d < OBSTACLE_THR) { obstacleAvoid(); continue; }

    if(fwd(150)) posY += 150;
    else { boundaryRecover(); break; }
  }

  stp();
  sweeps++;

  // Push extra with arms forward
  armsFwd();
  fwd(250);
  posY += 250;

  // Return to own side
  armsRet();
  unsigned int maxRev = MID_MS + 300;
  unsigned int rev = 0;

  while(rev < maxRev) {
    if(millis() - startTime >= 60000) return;
    if(isOwnSide()) break;
    if(isBlack()) { fwd(150); posY += 150; break; }
    if(bwd(150)) { posY -= 150; rev += 150; }
    else break;
  }
  stp();

  if(isOwnSide()) posY = MID_MS - 350;

  // Move to next lane if time permits
  if(sweeps < 6 && (millis() - startTime < 50000)) {
    lane++;
    if(lane > 4) lane = 1;

    unsigned int shift = (unsigned int)(12.0 * MS_PER_INCH);
    turnL();
    if(fwd(shift)) posX -= shift;
    turnR();

    armsDep();
    delay(150);
  }
}
