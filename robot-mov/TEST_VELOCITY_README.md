# Single Wheel Velocity Control Test

## Overview
This test validates the velocity control system for a single wheel using:
- **AS5600 Magnetic Encoder** - Position feedback via ADC
- **Kalman Filter** - Noise reduction on velocity measurements
- **PID Controller** - Velocity tracking
- **BLDC Motor** - PWM control

## Test Configuration

### Default Parameters
- **Target Velocity**: 30 cm/s
- **Test Duration**: 10 seconds
- **Sample Time**: 5 ms (200 Hz)
- **Wheel Radius**: 3.0 cm
- **Wheel to Test**: RIGHT wheel

### Expected Results
With the default configuration:
- **Expected Distance**: 300 cm (30 cm/s × 10 s)
- **Expected Rotations**: ~15.92 turns (300 cm ÷ 18.85 cm circumference)

## Building and Running

### Step 1: Set the Build Flag
```bash
cd /home/administrador/Documentos/striker1/SEA_Striker_01/robot-mov
idf.py -DTEST_VEL_WHEEL=1 build
```

### Step 2: Flash to ESP32-S3
```bash
idf.py -p /dev/ttyUSB0 flash
```

### Step 3: Monitor Output
```bash
idf.py -p /dev/ttyUSB0 monitor
```

Or combine all steps:
```bash
idf.py -DTEST_VEL_WHEEL=1 build flash monitor
```

## What You'll See

### Initialization Phase
```
═══════════════════════════════════════
  Single Wheel Velocity Test
  Testing RIGHT wheel only
═══════════════════════════════════════
✓ Motors initialized
✓ Encoders initialized
✓ Kalman filter initialized (Q=0.005, R=1.0)
✓ PID controller initialized (Kp=0.020, Ki=0.010, Kd=0.000)
✓ Tasks created
```

### Test Parameters Display
```
═══════════════════════════════════════
  TEST PARAMETERS
─────────────────────────────────────
  Target velocity: 30.00 cm/s
  Test duration: 10.00 seconds
  Wheel radius: 3.00 cm
  Wheel circumference: 18.85 cm
─────────────────────────────────────
  Expected distance: 300.00 cm
  Expected rotations: 15.92 turns
═══════════════════════════════════════
```

### Real-time Monitoring (every 200ms)
```
EncoderTask: Angle: 45.23° | Velocity: 29.87 cm/s | Distance: 125.43 cm | Rotations: 6.65
PIDControl: Setpoint: 30.00 | Input: 29.87 | Output: 45.23
```

### Test Completion Summary
```
PIDControl: Test completed!
═══════════════════════════════════════
  Target velocity: 30.00 cm/s
  Test duration: 10.00 seconds
  Total distance: 298.76 cm
  Total rotations: 15.85 turns
  Average velocity: 29.88 cm/s
═══════════════════════════════════════
```

## Customizing the Test

Edit these variables in `test_velocity_wheel.c` (inside `app_main`):

```c
desired_velocity_cm_s = 30.0f;  // Change target velocity (cm/s)
test_duration_s = 10.0f;        // Change test duration (seconds)
```

### PID Tuning
Modify PID parameters in `app_main`:
```c
pid_parameter_t pid_param = {
    .kp = 0.02f,    // Proportional gain
    .ki = 0.01f,    // Integral gain
    .kd = 0.0f,     // Derivative gain
    .max_output = 70.0f,
    .min_output = -70.0f,
    // ...
};
```

### Kalman Filter Tuning
Adjust filter parameters for different noise characteristics:
```c
kalman_init(&km_right_wheel, 0.005f, 1.0f);
//                           ^^^^^^  ^^^^
//                           Q       R
// Q (process noise): Lower = smoother, slower response
// R (measurement noise): Lower = trust measurements more
```

## Troubleshooting

### Motor doesn't move
- Check PWM GPIO connections (GPIO 20, 21)
- Verify BLDC ESC calibration
- Check power supply

### Encoder readings are erratic
- Verify AS5600 is properly aligned with magnet
- Check ADC GPIO connection (GPIO 5)
- Increase Kalman R parameter for more filtering

### Velocity doesn't track setpoint
- Tune PID gains (start with Kp only)
- Check for mechanical issues
- Verify encoder is reading correctly

### Build errors
Make sure you set the flag:
```bash
idf.py -DTEST_VEL_WHEEL=1 build
```

## Physical Verification

To verify the rotations in real life:
1. Mark the wheel with a visible indicator (tape, marker)
2. Count full rotations manually during the 10-second test
3. Compare with displayed rotation count
4. Expected: ~16 complete rotations at 30 cm/s

## Code Structure

### Tasks Created:
1. **vTaskReadEncoder** (Priority 5)
   - Reads AS5600 ADC every 5ms
   - Calculates velocity
   - Applies Kalman filter
   - Tracks distance and rotations

2. **vTaskPIDControl** (Priority 4)
   - Updates PID setpoint every 5ms
   - Computes control output
   - Sets motor PWM duty cycle
   - Monitors test completion

### Key Functions:
- `estimate_velocity_encoder()` - Velocity calculation from angle changes
- `kalman_update()` - Noise filtering
- `pid_compute()` - Control algorithm
- `bldc_set_duty()` - Motor PWM output

## Notes
- Test automatically stops after configured duration
- Motor stops (duty = 0) when test completes
- Distance and rotations are cumulative
- Angle wrapping is handled (-180° to +180°)
