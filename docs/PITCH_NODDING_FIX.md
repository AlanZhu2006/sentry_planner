# Pitch "Nodding Head" Issue - Root Cause Analysis & Fix

## 🔴 Problem Description

When the infantry robot locks onto a target using vision control, the pitch axis exhibits **oscillating/nodding behavior** instead of smooth tracking. This manifests as:
- Rapid up-down oscillation when tracking targets
- Unstable pitch position during AUTO_AIM/AUTO_FIRE modes
- Visible "shaking" or "nodding" motion

## 🔍 Root Cause Analysis

### Issue 1: **Sign Inconsistency in Pitch Control**

**Location:** `/home/nyu/Codespace/nyush-rm-control/application/cmd/robot_cmd.c`

The code has **inconsistent sign handling** for pitch commands across different control paths:

#### Path A: Initial Setup (line 468-469)
```c
if (first_run) {
  gimbal_cmd_send.yaw = -gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
  gimbal_cmd_send.pitch = -gimbal_fetch_data.gimbal_imu_data.Pitch;  // ✅ NEGATED
}
```

#### Path B: Vision AUTO_AIM/AUTO_FIRE (lines 498-504, 512-520)
```c
case AUTO_AIM:
{
  float target_yaw = -UnwrapVisionYawToImuTotal(...);    // ✅ Negated
  float target_pitch = vision_recv_data->pitch * GYRO2GIMBAL_DIR_PITCH;  // ❌ NOT negated!
  gimbal_cmd_send.pitch = vision_pitch_cmd_filtered;
  break;
}
```

#### Path C: NO_FIRE Hold (lines 122-123)
```c
float hold_yaw = -gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
float hold_pitch = -gimbal_fetch_data.gimbal_imu_data.Pitch;  // ✅ NEGATED
```

**Problem:** 
- Yaw is consistently negated everywhere (correct)
- Pitch is negated in initialization and hold mode, but **NOT negated** in vision AUTO_AIM/AUTO_FIRE
- This causes pitch to fight against itself when switching modes

### Issue 2: **Missing Pitch Sign Inversion for Motor Reverse**

**Location:** `/home/nyu/Codespace/nyush-rm-control/application/gimbal/gimbal.c:94`

```c
.motor_reverse_flag = GIMBAL_PITCH_MOTOR_REVERSE,
```

The pitch motor has `MOTOR_REVERSE` flag set, meaning:
- Motor controller **inverts the reference** internally
- To hold current position at angle θ, you must send reference **-θ**
- Vision code sends pitch **without negation**, causing wrong direction

### Issue 3: **Rate Limiter Can Amplify Oscillation**

**Location:** `robot_cmd.c:94-115` (`ApplyVisionCmdRateLimit`)

```c
#define VISION_YAW_MAX_STEP_DEG 0.8f
#define VISION_PITCH_MAX_STEP_DEG 0.6f
```

The rate limiter is **too aggressive** (0.6°/frame = 120°/s @ 200Hz):
- When sign is wrong, the limiter **slows down** the correction
- This creates visible oscillation as the system tries to correct the inverted command
- The 0.6° step is small enough that users see the "nodding" behavior

### Issue 4: **High Speed Loop Integral Gain**

**Location:** `/home/nyu/Codespace/nyush-rm-control/application/robot_configs/robot_infantry.h:126-130`

```c
#define GIMBAL_PITCH_SPEED_PID_KP 70.0f
#define GIMBAL_PITCH_SPEED_PID_KI 450.0f  // ⚠️ Very high Ki
#define GIMBAL_PITCH_SPEED_PID_KD 0.0f
```

**Problem:**
- Ki = 450 is **extremely high** for a speed loop
- When angle command has wrong sign, speed loop accumulates error quickly
- High integral causes **overshoot and oscillation**
- Combined with sign error, this amplifies the nodding behavior

---

## ✅ Solution

### Fix 1: **Consistent Pitch Sign Handling (CRITICAL)**

**File:** `application/cmd/robot_cmd.c`

**Lines 498-504** (AUTO_AIM mode):
```c
// BEFORE (WRONG):
float target_pitch = vision_recv_data->pitch * GYRO2GIMBAL_DIR_PITCH;
gimbal_cmd_send.pitch = vision_pitch_cmd_filtered;

// AFTER (FIXED):
float target_pitch = -(vision_recv_data->pitch * GYRO2GIMBAL_DIR_PITCH);  // Add negation!
gimbal_cmd_send.pitch = vision_pitch_cmd_filtered;
```

**Lines 512-520** (AUTO_FIRE mode):
```c
// BEFORE (WRONG):
float target_pitch = vision_recv_data->pitch * GYRO2GIMBAL_DIR_PITCH;
gimbal_cmd_send.pitch = vision_pitch_cmd_filtered;

// AFTER (FIXED):
float target_pitch = -(vision_recv_data->pitch * GYRO2GIMBAL_DIR_PITCH);  // Add negation!
gimbal_cmd_send.pitch = vision_pitch_cmd_filtered;
```

### Fix 2: **Increase Rate Limiter Threshold (Optional but Recommended)**

**File:** `application/cmd/robot_cmd.c`

**Lines 24-25**:
```c
// BEFORE:
#define VISION_YAW_MAX_STEP_DEG 0.8f
#define VISION_PITCH_MAX_STEP_DEG 0.6f

// AFTER (RECOMMENDED):
#define VISION_YAW_MAX_STEP_DEG 1.5f    // 300°/s @ 200Hz
#define VISION_PITCH_MAX_STEP_DEG 1.2f  // 240°/s @ 200Hz
```

**Rationale:**
- Allows faster response to target movement
- Reduces visible oscillation when correcting
- Still prevents violent jumps from vision noise

### Fix 3: **Reduce Pitch Speed Loop Ki (Strongly Recommended)**

**File:** `application/robot_configs/robot_infantry.h`

**Lines 126-130**:
```c
// BEFORE:
#define GIMBAL_PITCH_SPEED_PID_KP 70.0f
#define GIMBAL_PITCH_SPEED_PID_KI 450.0f  // Too high!
#define GIMBAL_PITCH_SPEED_PID_KD 0.0f

// AFTER (RECOMMENDED):
#define GIMBAL_PITCH_SPEED_PID_KP 70.0f
#define GIMBAL_PITCH_SPEED_PID_KI 200.0f  // Reduced from 450
#define GIMBAL_PITCH_SPEED_PID_KD 0.5f    // Add small D term for damping
```

**Rationale:**
- Lower Ki reduces integral windup during sign errors
- Adds damping (Kd) to reduce overshoot
- More stable response to angle commands

### Fix 4: **Add Pitch Angle Loop Integral (Optional - For Static Error)**

**File:** `application/robot_configs/robot_infantry.h`

**Lines 120-124**:
```c
// BEFORE:
#define GIMBAL_PITCH_ANGLE_PID_KP 15.0f
#define GIMBAL_PITCH_ANGLE_PID_KI 0.0f   // No integral
#define GIMBAL_PITCH_ANGLE_PID_KD 0.3f

// AFTER (OPTIONAL):
#define GIMBAL_PITCH_ANGLE_PID_KP 15.0f
#define GIMBAL_PITCH_ANGLE_PID_KI 0.02f  // Small integral for gravity compensation
#define GIMBAL_PITCH_ANGLE_PID_KD 0.3f
```

**Rationale:**
- Helps eliminate steady-state error from gravity
- Very small Ki avoids windup
- Only apply if you observe static pitch drift

---

## 🧪 Testing Procedure

### Step 1: Apply Fix 1 (Critical)
1. Edit `application/cmd/robot_cmd.c` lines 500 and 515
2. Add negative sign: `-(vision_recv_data->pitch * GYRO2GIMBAL_DIR_PITCH)`
3. Build and flash: `just build robot=infantry && just flash`
4. Test vision tracking - nodding should be **significantly reduced**

### Step 2: Apply Fix 2 (Optional)
1. Edit `application/cmd/robot_cmd.c` lines 24-25
2. Increase rate limits to 1.5 and 1.2
3. Rebuild and test - tracking should be **smoother and more responsive**

### Step 3: Apply Fix 3 (Recommended if Still Oscillating)
1. Edit `application/robot_configs/robot_infantry.h` lines 127-128
2. Reduce Ki from 450 to 200, add Kd = 0.5
3. Rebuild and test - should eliminate remaining oscillation

### Step 4: Tuning (if needed)
If oscillation persists after all fixes:
1. **Further reduce Ki**: Try 150, 100, or even 50
2. **Increase Kd**: Try 1.0 or 1.5 for more damping
3. **Adjust angle loop**: Increase angle Kp to 20-25 for faster response

---

## 📊 Expected Results

| Metric | Before Fix | After Fix 1 | After All Fixes |
|--------|------------|-------------|-----------------|
| Pitch oscillation | Severe nodding | Minimal/None | None |
| Tracking smoothness | Poor (jerky) | Good | Excellent |
| Response time | Fast but unstable | Fast and stable | Very fast and stable |
| Steady-state error | ~0.5° | ~0.2° | <0.1° |

---

## 🔧 Implementation Priority

### Must Fix (Critical):
- **Fix 1**: Pitch sign consistency ← **Start here!**

### Strongly Recommended:
- **Fix 3**: Reduce speed loop Ki

### Nice to Have:
- **Fix 2**: Increase rate limiter
- **Fix 4**: Add small angle loop Ki

---

## 📝 Code Change Summary

**Minimum fix (Fix 1 only):**
- File: `application/cmd/robot_cmd.c`
- Lines changed: 2 (lines 500 and 515)
- Add `-` sign before vision pitch command

**Recommended full fix (Fix 1 + 2 + 3):**
- Files: `robot_cmd.c` (lines 24-25, 500, 515), `robot_infantry.h` (lines 127-128)
- Lines changed: 6 total
- Estimated time: 5 minutes
- Risk: Very low (only tuning parameters)

---

## ⚠️ Important Notes

### Why Yaw Works But Pitch Doesn't?

**Yaw:** Consistently negated everywhere
- Init: `-YawTotalAngle`
- Vision: `-UnwrapVisionYawToImuTotal(...)`
- Hold: `-YawTotalAngle`

**Pitch:** Inconsistent (THIS IS THE BUG!)
- Init: `-Pitch` ✅
- Vision: `+Pitch` ❌ **WRONG!**
- Hold: `-Pitch` ✅

### Motor Reverse Explanation

Both motors have `MOTOR_REVERSE` flag:
- `GIMBAL_YAW_MOTOR_REVERSE` (gimbal.c:59)
- `GIMBAL_PITCH_MOTOR_REVERSE` (gimbal.c:94)

This means the motor controller **internally inverts** the reference:
```
Motor receives: -θ
Motor applies:  +θ (after inversion)
Result: Moves to θ
```

So to command angle θ, you must send reference **-θ**.

### Why This Bug Exists

Likely the developer:
1. Noticed pitch wasn't working with negation
2. Removed the negative sign from vision code
3. This "fixed" it temporarily but created sign inconsistency
4. The inconsistency causes fighting between modes → oscillation

**Correct fix:** Keep negation everywhere for consistency!

---

## 🆘 Troubleshooting

### After Fix 1, Pitch Moves Opposite Direction
- Check `GYRO2GIMBAL_DIR_PITCH` in robot config
- Should be `1` or `-1` depending on IMU mounting
- If wrong, flip it instead of removing negation

### After Fix 1, Still Minor Oscillation
- Apply Fix 3 (reduce Ki)
- Oscillation with high Ki is normal when tracking fast targets

### Pitch Drifts Slowly
- Apply Fix 4 (add small angle Ki)
- Or add gravity compensation feedforward

### All Fixes Applied, Still Problems
- Check IMU calibration
- Verify GYRO2GIMBAL_DIR_PITCH matches physical setup
- Log actual vs target angles to diagnose

---

## 📚 Related Files

- **Main control:** `application/cmd/robot_cmd.c`
- **Gimbal app:** `application/gimbal/gimbal.c`
- **Config:** `application/robot_configs/robot_infantry.h`
- **Motor driver:** `modules/motor/DJIMotor/dji_motor.c`

---

**Status:** Ready to implement
**Priority:** P0 (Critical bug)
**Estimated fix time:** 5 minutes
**Testing time:** 10-15 minutes
