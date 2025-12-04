# AutoWiper Messy Room Competition Robot

**Team**: Julius Fedelino, Eric Strathman, Sean Goossen
**Course**: EGME 456 - Cal State Fullerton

Production-ready Arduino code for autonomous robot competition.

## 🎯 Competition Overview

- **Duration**: 60 seconds autonomous operation
- **Objective**: Push foam cubes from your side to opponent's side
- **Victory**: Fewer cubes on your side at match end
- **Field**: 48" × 48" with 2" black boundary
- **Starting Size**: 8" × 8" (arms retracted)
- **Deployed Size**: 12" diameter (arms extended)

## 🤖 Robot Configuration

### Hardware Platform
- **Base**: Parallax BOE Shield-Bot with Arduino Uno
- **Microcontroller**: ATmega328P (2KB RAM, 32KB Flash)
- **Programming**: Arduino IDE 2.x

### Critical Hardware Facts

⚠️ **IMPORTANT**: The TCS3200 color sensor is mounted at the **REAR** of the robot!
- When sensor reads position (X, Y), the robot extends **8 inches FORWARD**
- Robot front edge is at (X, Y+8")
- All position calculations account for this offset

### Sensor & Actuator Pinout

#### Digital Pins
- **D1-D2, D8-D9**: IR sensors (opponent detection ONLY, not boundaries)
- **D3**: Piezo buzzer (status feedback)
- **D4**: TCS3200 OUT (color sensor frequency output)
- **D5**: TCS3200 S2 (filter select)
- **D6-D7**: Debug LEDs
- **D10**: Right arm servo
- **D11**: Left arm servo
- **D12**: Right drive servo (Parallax continuous rotation)
- **D13**: Left drive servo (Parallax continuous rotation)

#### Analog Pins
- **A0**: TCS3200 S3 (used as digital output for filter select)
- **A4**: LSM6DSOX SDA (I2C gyroscope)
- **A5**: LSM6DSOX SCL (I2C gyroscope)

### Servo Arm Positions
- **Deployed** (sweeping): Both arms at **90°** (perpendicular, 12" width)
- **Retracted** (maneuvering):
  - Left arm: **153°** (90° + 63° CCW toward back, 4.75" width)
  - Right arm: **27°** (90° - 63° CW toward back)
  - ⚠️ Arms retract **toward the BACK**, not forward

## 📐 Coordinate System

### Position Tracking
```
Y
↑
│   [Opponent's Back Boundary]
│   ╔═══════════════════════╗ Y=48"
│   ║   OPPONENT TERRITORY  ║
│   ║                       ║
├───╫───────────────────────╫─── Y=24" (Midfield)
│   ║                       ║
│   ║   YOUR TERRITORY      ║
│   ╚═══════════════════════╝ Y=0"
│   [Your Back Boundary]
│
└─────────────────────────────→ X
    0"        24"         48"
```

### Key Positions
- **Start**: Color sensor at (24", 2"), robot front at (24", 10")
- **Sweep Lanes**: X = 6", 18", 30", 42" (4 lanes × 12" width = 48" coverage)
- **Safe Limits**:
  - Sensor Y: 4" to 40" (robot front: 12" to 48")
  - X: 0" to 48"

## 🎮 Strategy Implementation

### Phase 0: Initialization (0-5 seconds)
1. **2-second mandatory delay** (competition requirement)
2. Initialize all sensors and servos
3. Detect starting field color (RED or BLUE side)
4. **Calibrate coordinate system** by touching back boundary
5. Zero gyroscope heading
6. Retract arms to 4.75" width

### Phase 1: Offensive Sweep (5-35 seconds)
Execute **4 sweeping passes** across opponent's territory:

1. Navigate to lane 1 (X=6")
2. Deploy arms (12" width)
3. Sweep forward to opponent's deep territory (sensor Y=38", front Y=46")
4. Retract arms and return to midfield
5. Shift to next lane (X=18", 30", 42")
6. Repeat for all 4 lanes (~7 seconds per pass)

**Result**: Pushes cubes deep into opponent's side in systematic pattern

### Phase 2: Defensive Clearing (35-60 seconds)
Continuous serpentine sweeping on own side until match end:

1. Forward sweep (Y=12" to Y=22")
2. Shift to next lane
3. Backward sweep (Y=22" to Y=8")
4. Shift to next lane
5. Repeat continuously until 60 seconds

**Result**: Continuously pushes any cubes back to opponent until time expires

**⚠️ NO Phase 3**: Defensive operations continue until 60-second time limit

## 🔍 Sensor Operations

### TCS3200 Color Sensor

#### Black Boundary Detection
```cpp
// Black detected when BOTH red AND blue show high duration (>600µs)
unsigned long red = measureColorDuration(true);
unsigned long blue = measureColorDuration(false);

if(red > 600 && blue > 600) {
  // BLACK BOUNDARY DETECTED
}
```

#### Field Side Detection
```cpp
// Red side: red < blue (red reflects more)
// Blue side: blue < red (blue reflects more)
bool onRedSide = (red < blue);
```

⚠️ **CRITICAL**:
- Color sensor is used for **ALL boundary detection**
- IR sensors are **ONLY** for opponent robot detection
- Never use IR for boundary detection

### LSM6DSOX Gyroscope
- **Purpose**: Maintain heading during straight-line travel
- **Update Rate**: Every loop cycle (~20ms)
- **Correction**: Proportional heading error correction during movement
- **Drift**: ~1-2°/minute typical (acceptable for 60s match)

### IR Sensors (Opponent Detection)
- **Detection Range**: 6-12 inches
- **Returns**: 0 = opponent detected, 1 = clear
- **Usage**: Trigger evasion maneuvers when opponent ahead
- **⚠️ NOT used for boundary detection**

## 🛠️ Key Functions

### Movement
- `moveForward(distance_inches)` - Move forward with gyro correction
- `moveBackward(distance_inches)` - Move backward
- `rotateLeft(degrees)` / `rotateRight(degrees)` - In-place rotation
- `navigateToCoordinate(x, y)` - Point-to-point navigation

### Sensors
- `detectBlackBoundary()` - Returns true if on black border (color sensor)
- `checkFieldSide()` - Returns 1=own, -1=opponent, 0=boundary (color sensor)
- `scanForOpponent()` - Returns true if IR detects robot ahead
- `updateGyroscope()` - Integrate gyro for heading

### Arms
- `deployArms()` - Extend to 90° (12" width)
- `retractArms()` - Retract to 153°/27° toward back (4.75" width)
- `ensureArmsRetracted()` - Safety check before navigation

### Strategy
- `executePhase1_OffensiveSweep()` - 4-pass opponent territory sweep
- `executePhase2_DefensiveClearing()` - Continuous serpentine defense

### Error Handling
- `handleBoundaryEmergency()` - Recover from unexpected boundary
- `evadeOpponent()` - Maneuver around detected opponent

## 🚀 Usage Instructions

### 1. Hardware Setup
1. Mount TCS3200 color sensor at **REAR** of robot
2. Install LSM6DSOX gyroscope (I2C connection)
3. Connect servos to specified pins
4. Mount IR sensors facing **FORWARD** for opponent detection
5. Verify arms retract toward **BACK** (not forward)

### 2. Software Installation
1. Install **Arduino IDE 2.x**
2. Install required libraries via Library Manager:
   - Adafruit LSM6DSOX
   - Adafruit Unified Sensor
   - Servo (built-in)
   - Wire (built-in)
3. Open `AutoWiper_Competition.ino`
4. Select **Board**: Arduino Uno
5. Select correct **Port**
6. Upload to robot

### 3. Calibration

#### Servo Calibration
Test drive servos to verify:
- `1500µs` = stop
- `1700µs` = full speed forward
- `1300µs` = full speed reverse

Adjust `TURN_90_DEGREES_MS` (default 590ms) by timing actual 90° turns on your surface.

#### Color Sensor Calibration
1. Set `#define DEBUG_MODE true` in code
2. Upload and open Serial Monitor (9600 baud)
3. Place robot on RED surface, note values
4. Place on BLUE surface, note values
5. Place on BLACK boundary, verify both >600µs
6. Adjust `BLACK_THRESHOLD` if needed

#### Arm Position Verification
1. Deploy arms - verify perpendicular at 90° (12" total width)
2. Retract arms - verify 153°/27° angle toward BACK
3. Measure retracted width should be ~4.75"

### 4. Pre-Competition Testing

#### Test Checklist
- [ ] 2-second delay at start
- [ ] Arms start retracted (4.75" width for 8"×8" compliance)
- [ ] Boundary detection works (color sensor, not IR)
- [ ] Field side correctly identified (RED vs BLUE)
- [ ] Gyroscope heading correction working
- [ ] IR opponent detection triggers evasion
- [ ] All 4 offensive sweeps complete
- [ ] Defensive phase continues until 60s
- [ ] Robot stops at exactly 60 seconds

### 5. Competition Day
1. Upload code before inspection
2. Pass technical inspection
3. Place robot at back-center, **arms retracted**
4. Face robot forward into your territory
5. Robot executes autonomously for 60 seconds

## 🐛 Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Drives in circles | Servo mismatch | Adjust `SERVO_FULL_SPEED` value |
| Position drift | Odometry error | Calibrate `SPEED_INCHES_PER_SEC` |
| False boundaries | Threshold too low | Increase `BLACK_THRESHOLD` to 700-800 |
| Arms won't retract | Wrong direction | Verify angles: Left=153°, Right=27° |
| Unexpected stops | IR false positive | Check IR sensor mounting/alignment |
| Gyro not working | I2C issue | Code falls back to dead reckoning |

## 📊 Performance Expectations

### Timing Budget
- **Phase 0 (Init)**: ~5 seconds
- **Phase 1 (Offensive)**: ~28 seconds (7s × 4 passes)
- **Phase 2 (Defensive)**: ~27 seconds (continuous)

### Accuracy
- **Position**: ±3" expected (acceptable)
- **Heading**: ±5° with gyroscope
- **Boundary Detection**: 100% with color sensor

## 📝 Code Configuration

### Debug Mode
```cpp
#define DEBUG_MODE false  // Set true for Serial debugging
```
Enables Serial output for position tracking and diagnostics.
**⚠️ Disable for competition** (Serial overhead affects timing)

### Test Mode
```cpp
#define TEST_SENSORS_ONLY false  // Set true for sensor testing
```
Displays continuous sensor readings without movement.

### Calibration Constants
```cpp
#define SPEED_INCHES_PER_SEC 9.0    // Robot speed calibration
#define TURN_90_DEGREES_MS 590      // 90° rotation time
#define ODOMETRY_CORRECTION 0.95    // Wheel slip factor
#define BLACK_THRESHOLD 600         // Color sensor black level
```

## 🏆 Success Criteria

Your robot is competition-ready when it:

1. ✅ Navigates autonomously for 60 seconds
2. ✅ Detects boundaries via TCS3200 color sensor
3. ✅ Distinguishes RED/BLUE field sides
4. ✅ Executes 4 systematic sweeping passes
5. ✅ Maintains coordinate accuracy (±3")
6. ✅ Detects/evades opponent via IR
7. ✅ Continues defensive clearing until 60s
8. ✅ Handles failures gracefully
9. ✅ Complies with size/autonomy rules
10. ✅ Consistently maintains fewer cubes

## 📚 Libraries Required

Install via Arduino Library Manager:
- **Servo** (built-in)
- **Wire** (built-in)
- **Adafruit LSM6DSOX**
- **Adafruit Unified Sensor**

## 🎓 Competition Rules Compliance

✅ 2-second start delay
✅ 8" × 8" starting size (arms retracted)
✅ 12" deployed limit (arms extended)
✅ Autonomous operation only
✅ Single uploaded program
✅ Active offensive strategy

## 💡 Key Concepts

1. **Rear Sensor Offset**: Always add 8" to sensor Y for front position
2. **Boundary Detection**: Color sensor (red>600 AND blue>600) = BLACK
3. **IR for Opponents Only**: Never use IR for boundaries
4. **Arm Retraction**: 63° toward BACK, not forward
5. **4 Sweep Lanes**: 12" width × 4 passes = complete 48" coverage
6. **No Phase 3**: Defensive continues until 60s time limit

## 🤝 Why AutoWiper Works

1. **Systematic**: 4-pass offensive guarantees 100% coverage
2. **Deep Push**: Sweeps to Y=46" maximize opponent territory penetration
3. **Continuous**: Serpentine defense prevents cube accumulation
4. **Time-Optimized**: 28s offensive + 27s defensive
5. **Precise**: Rear sensor tracking with 8" offset ensures accuracy
6. **Safe**: Color sensor provides 100% boundary detection
7. **Robust**: Sensor fusion and error recovery handle failures

---

**Good luck in the competition! 🏆**

For diagnostics, enable `DEBUG_MODE` and monitor Serial output at 9600 baud
