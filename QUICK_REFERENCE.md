# AutoWiper Quick Reference Guide

**⚡ Fast reference for testing and competition day**

## 🚨 CRITICAL HARDWARE FACTS

### Color Sensor Position
- **Location**: REAR of robot (back edge)
- **Math**: Robot front = Sensor Y + 8 inches
- **Example**: Sensor at Y=40" → Front at Y=48" (field edge)

### Boundary Detection
- **Method**: TCS3200 color sensor ONLY
- **Detection**: red>600µs AND blue>600µs = BLACK
- **⚠️ NEVER use IR for boundaries** (IR = opponent detection only)

### Servo Arm Angles
- **Deployed**: 90° both arms (perpendicular, 12" width)
- **Retracted**: Left=153°, Right=27° (toward BACK, 4.75" width)
- **⚠️ Arms retract toward BACK**, not forward!

## 📍 Key Coordinates

```
Starting Position:
- Sensor: (24", 2")
- Robot Front: (24", 10")
- Heading: 0° (forward)

Sweep Lanes:
- Lane 1: X = 6"
- Lane 2: X = 18"
- Lane 3: X = 30"
- Lane 4: X = 42"

Safe Limits:
- Sensor Y: 4" to 40"
- Robot Front Y: 12" to 48"
```

## 🎯 Strategy Timeline

| Time | Phase | Action |
|------|-------|--------|
| 0-2s | Init | Mandatory 2-second delay |
| 2-5s | Init | Calibrate, retract arms |
| 5-35s | Offensive | 4 sweeps @ X=6,18,30,42" |
| 35-60s | Defensive | Continuous serpentine on own side |

**⚠️ NO Phase 3** - Defensive continues until 60s

## 🔌 Pin Assignments (Quick Reference)

| Pin | Device | Purpose |
|-----|--------|---------|
| D13 | Left Drive | Parallax servo |
| D12 | Right Drive | Parallax servo |
| D11 | Left Arm | Hobby servo |
| D10 | Right Arm | Hobby servo |
| D4 | Color OUT | TCS3200 frequency |
| D5 | Color S2 | TCS3200 filter |
| A0 | Color S3 | TCS3200 filter |
| A4-A5 | Gyro I2C | LSM6DSOX |
| D1-D2, D8-D9 | IR | Opponent detection |

## 🛠️ Pre-Competition Checklist

### Hardware
- [ ] Color sensor at REAR (back edge)
- [ ] Arms retract to 153°/27° toward BACK
- [ ] All servos connected to correct pins
- [ ] Batteries fresh (5×AA + 1×9V)
- [ ] Robot measures ≤8"×8" with arms retracted

### Software
- [ ] `DEBUG_MODE false` (competition mode)
- [ ] Code uploaded successfully
- [ ] 2-second delay verified
- [ ] Boundary detection tested (color sensor)
- [ ] Field color detection tested (RED/BLUE)

### Testing
- [ ] Forward/backward movement straight
- [ ] 90° turns accurate (adjust `TURN_90_DEGREES_MS`)
- [ ] Arms deploy/retract properly
- [ ] Black boundary stops robot
- [ ] IR detects opponent (optional test)
- [ ] Full 60-second autonomous run

## ⚙️ Calibration Constants

```cpp
// Speed calibration
#define SPEED_INCHES_PER_SEC 9.0      // Default: 9.0
#define TURN_90_DEGREES_MS 590        // Default: 590

// Sensor thresholds
#define BLACK_THRESHOLD 600           // Default: 600
#define ODOMETRY_CORRECTION 0.95      // Default: 0.95

// Arm positions
#define ARM_DEPLOYED 90               // Don't change
#define ARM_LEFT_RETRACT 153          // Don't change
#define ARM_RIGHT_RETRACT 27          // Don't change
```

### How to Calibrate

#### Robot Speed
1. Mark 36" distance on floor
2. Time robot: `moveForward(36.0)`
3. Calculate: `speed = 36 / time_in_seconds`
4. Update `SPEED_INCHES_PER_SEC`

#### Turn Angle
1. Mark starting orientation
2. Execute: `rotateRight(90.0)`
3. Measure actual angle turned
4. Adjust: `TURN_90_DEGREES_MS = 590 × (90 / actual_angle)`

#### Color Sensor
1. Enable `DEBUG_MODE true`
2. Place on RED surface → note values
3. Place on BLUE surface → note values
4. Place on BLACK boundary → both should be >600
5. If black reads <600, decrease `BLACK_THRESHOLD`
6. If colored surfaces read >600, increase threshold

## 🚫 Common Mistakes to Avoid

1. ❌ Using IR for boundary detection
   - ✅ Use TCS3200 color sensor (red>600 AND blue>600)

2. ❌ Forgetting 8" sensor offset
   - ✅ Robot front = Sensor Y + 8"

3. ❌ Arms retracting forward
   - ✅ Arms retract toward BACK (153°/27°)

4. ❌ Enabling DEBUG_MODE in competition
   - ✅ Set `DEBUG_MODE false` before upload

5. ❌ Starting with arms deployed
   - ✅ Arms MUST start retracted (4.75" width)

6. ❌ Not calibrating turn time
   - ✅ Test and adjust `TURN_90_DEGREES_MS`

## 🐛 Quick Troubleshooting

| Symptom | Fix |
|---------|-----|
| Drives in circles | Swap left/right servo values or adjust speeds |
| Turns wrong amount | Adjust `TURN_90_DEGREES_MS` |
| False boundaries | Increase `BLACK_THRESHOLD` to 700-800 |
| Position drifts | Recalibrate `SPEED_INCHES_PER_SEC` |
| Arms stuck | Check servo connections, verify angles |
| Won't detect boundary | Verify color sensor wiring, check S2/S3 |

## 📊 Expected Behavior

### Phase 1 (Offensive)
- Navigate to X=6", deploy arms
- Sweep forward to Y=38" (front at Y=46")
- Retract, return to Y=24"
- Repeat for X=18", 30", 42"
- Duration: ~28 seconds

### Phase 2 (Defensive)
- Navigate to own side
- Deploy arms
- Forward sweep Y=12"→22"
- Shift lane, backward sweep Y=22"→8"
- Repeat until 60 seconds

### Boundary Detection
- When red>600 AND blue>600: BLACK detected
- If at Y>40: reverse away
- If at Y<6: move forward away

## 🎮 Competition Day Protocol

1. **Upload**: Final code before inspection
2. **Inspection**: Pass size/sensor/code checks
3. **Placement**: Back-center, arms RETRACTED
4. **Orientation**: Face forward into your territory
5. **Start**: Robot waits 2s, then autonomous
6. **Duration**: 60 seconds exactly
7. **Hands Off**: No touching during match

## 🔧 Emergency Fixes

### If robot won't stop at boundary:
```cpp
// Increase threshold in code
#define BLACK_THRESHOLD 700  // Was 600
```

### If turns are inaccurate:
```cpp
// Adjust turn timing
#define TURN_90_DEGREES_MS 550  // Was 590 (decrease = less turn)
#define TURN_90_DEGREES_MS 630  // Was 590 (increase = more turn)
```

### If position drifts significantly:
```cpp
// Adjust speed or correction factor
#define SPEED_INCHES_PER_SEC 8.5  // Was 9.0 (robot slower)
#define ODOMETRY_CORRECTION 0.90  // Was 0.95 (more slip)
```

## 📱 Debug Output Format

When `DEBUG_MODE true`:
```
T:15 | Phase:1 | SensorY:25.3 | FrontY:33.3 | X:18.2 | H:2.1
```
- **T**: Elapsed time (seconds)
- **Phase**: 0=Init, 1=Offensive, 2=Defensive
- **SensorY**: Color sensor Y position
- **FrontY**: Robot front Y position (SensorY + 8)
- **X**: X position
- **H**: Heading (degrees, 0=forward)

## 🎯 Victory Conditions

**Win**: Fewer cubes on YOUR side than opponent at 60s

**Strategy Ensures Victory**:
1. ✅ Push opponent's cubes deep (Y=46")
2. ✅ Continuous defense prevents returns
3. ✅ 100% coverage (4×12" lanes = 48" width)
4. ✅ No wasted time (28s offense + 27s defense)

---

**🏆 You're ready! Good luck in the competition!**

*Last check: Arms retracted? DEBUG_MODE false? Batteries fresh?*
