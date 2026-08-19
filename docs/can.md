# CAN Communication Guide

## What is CAN

CAN bus (Controller Area Network) connects all our electrical components together. CAN was created so that any two electrical components can send and receive commands without a central processing unit.

A CAN wire is composed of two wires: **CAN_H (high)** and **CAN_L (low)**. For RoboMaster, we use 1.25-mm pitch 2-pin GHR wire.

**Important:** Both GM6020 and C620 use **1 Mbps** bitrate. Wire colors: **CAN_H = Red, CAN_L = Black**.

**CAN Bus Topology:**
- Use **daisy-chain** topology (linear bus, not star topology)
- Maximum bus length: ~40m at 1 Mbps
- All devices connected in series along a single bus line
- **Termination resistors (120Ω) required at both physical ends only**

**Message Priority (Arbitration):**
- Lower CAN ID = Higher priority
- During bus arbitration, lower ID messages win
- Control commands (0x1FF, 0x200, 0x2FF) have higher priority than feedback (0x201-0x20B)
- If multiple boards send simultaneously, lower ID transmits first

## CAN Frame Structure

A CAN frame consists of the following components:

```
┌─────┬────────┬─────┬─────────┬──────────────┬─────┬─────┬─────┐
│ SOF │ CAN ID │ RTR │ CONTROL │ DATA (0-8 B) │ CRC │ ACK │ EOF │
└─────┴────────┴─────┴─────────┴──────────────┴─────┴─────┴─────┘
```

Key components:
- **SOF (Start of Frame)**: Signals a CAN message is coming (1 bit)
- **CAN ID (Identifier)**: Message identifier - allows filtering by recipient (11 bits for standard frame)
- **RTR (Remote Transmission Request)**: Request/data flag (1 bit)
- **CONTROL**: Data length code (4 bits)
- **DATA**: Actual data payload (0-8 bytes)
- **CRC (Cyclic Redundancy Check)**: Error checking (15 bits + 1 delimiter)
- **ACK (Acknowledgment)**: Acknowledgment slot (2 bits)
- **EOF (End of Frame)**: End of frame marker (7 bits)

For RoboMaster motor control, you mainly need to understand **CAN ID** and **DATA** fields.

### Quick Reference Table

| Device | Control CAN ID | Control Range | Feedback CAN ID | Motor IDs |
|--------|----------------|---------------|-----------------|-----------|
| GM6020 | 0x1FF | -25000 to 25000 | 0x205-0x208 | 1-4 |
| GM6020 | 0x2FF | -25000 to 25000 | 0x209-0x20B | 5-7 |
| C620 | 0x200 | -16384 to 16384 | 0x201-0x204 | 1-4 |
| C620 | 0x1FF | -16384 to 16384 | 0x205-0x208 | 5-8 |

**Note:** C620's 0x1FF is shared with GM6020 motors 1-4 on some setups. Plan your CAN IDs carefully to avoid conflicts.

---

## GM6020 Motor (Gimbal)

**Motor Specifications:**
- Type: Brushless DC motor with built-in ESC
- Rated voltage: 24V DC
- Rated current: 3A (peak ~5A)
- Output torque: 0.3 N·m (rated)
- No-load speed: ~300 RPM
- Built-in encoder resolution: 8192 counts/revolution (13-bit)
- Common use: Gimbal yaw/pitch control

**Control Frequency Recommendation:**
- Minimum: 100 Hz (10ms interval)
- Recommended: 200-500 Hz (2-5ms interval)
- Maximum: 1000 Hz (1ms interval)
- Higher frequency = smoother control, but increases CAN bus load

### Send Control (Board → Motor)

**Control Identifiers:**
- `0x1FF` - Controls motor IDs 1-4
- `0x2FF` - Controls motor IDs 5-7

**Voltage control range:** -25000 to 25000

Each CAN frame controls up to 4 motors. Since the control value doesn't fit in 1 byte, it's split across 2 bytes (high/low).

**DATA Field Structure (8 bytes):**

For CAN ID `0x1FF` (Motors 1-4):
```
Byte 0-1: Motor 1 voltage (high byte, low byte)
Byte 2-3: Motor 2 voltage (high byte, low byte)
Byte 4-5: Motor 3 voltage (high byte, low byte)
Byte 6-7: Motor 4 voltage (high byte, low byte)
```

For CAN ID `0x2FF` (Motors 5-7):
```
Byte 0-1: Motor 5 voltage (high byte, low byte)
Byte 2-3: Motor 6 voltage (high byte, low byte)
Byte 4-5: Motor 7 voltage (high byte, low byte)
Byte 6-7: Reserved (0x00, 0x00)
```

Each 16-bit voltage value is split as: `[High 8 bits][Low 8 bits]`

**Code example:**
```c
HAL_StatusTypeDef CAN_Manager_SendGM6020Current(CAN_HandleTypeDef *hcan, uint8_t motor_id, int16_t current)
{
    if (hcan == NULL) return HAL_ERROR;
    if (motor_id < 1 || motor_id > 7) return HAL_ERROR;

    // Clamp current
    if (current >  25000) current =  25000;
    if (current < -25000) current = -25000;

    // Determine CAN ID based on motor ID
    uint16_t stdId = (motor_id <= 4) ? 0x1FF : 0x2FF;
    uint8_t  slot  = (motor_id <= 4) ? (motor_id - 1) : (motor_id - 5);

    CAN_TxHeaderTypeDef tx = {0};
    uint8_t d[8] = {0};
    uint32_t mb;

    tx.StdId = stdId;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    // Split 16-bit value into 2 bytes
    d[slot*2 + 0] = (uint8_t)((current >> 8) & 0xFF);  // High byte
    d[slot*2 + 1] = (uint8_t)( current       & 0xFF);  // Low byte

    return HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
}
```

### Receive Feedback (Motor → Board)

**Feedback Identifiers:** `0x204 + motor_id` (e.g., motor 1 → `0x205`, motor 2 → `0x206`, ..., motor 7 → `0x20B`)

**DATA Field Structure (8 bytes):**
```
Byte 0-1: Rotor mechanical angle (uint16_t, 0-8191, maps to 0-360°)
Byte 2-3: Rotational speed in RPM (int16_t, signed value)
Byte 4-5: Actual torque current (int16_t, signed value)
Byte 6:   Motor temperature (uint8_t, °C)
Byte 7:   Reserved (unused)
```

**Angle Calculation:**
- Raw value range: 0-8191 (13-bit effective resolution)
- Actual angle (degrees) = (raw_value / 8192) × 360°
- Actual angle (radians) = (raw_value / 8192) × 2π

Feedback data includes: rotor angle, rotational speed, torque current, motor temperature.

**Code example:**
```c
// GM6020 feedback handler
if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x205 && rx.StdId<=0x20B) {
    uint8_t gid = (uint8_t)(rx.StdId - 0x204);
    if (gid >= 1 && gid <= 7) {
        uint16_t angle_raw = (uint16_t)((d[0]<<8) | d[1]);
        int16_t  speed_rpm = (int16_t)((d[2]<<8) | d[3]);
        // Process feedback...
    }
}
```

### Hardware Setup

**Motor ID Setting (DIP switch Bit0-Bit2):**

| DIP[2:0] | Motor ID | Feedback CAN ID |
|----------|----------|-----------------|
| 001      | 1        | 0x205           |
| 010      | 2        | 0x206           |
| 011      | 3        | 0x207           |
| 100      | 4        | 0x208           |
| 101      | 5        | 0x209           |
| 110      | 6        | 0x20A           |
| 111      | 7        | 0x20B           |

**Termination Resistance:** DIP **4th bit** enables/disables 120Ω termination. Only enable on the **two physical ends** of the CAN bus.

**LED Diagnostics:**
- Green blinks **N times/sec** → Motor ID = N
- Orange blinks **twice/sec** → **Duplicate ID detected** (fix immediately!)

---

## C620 + M3508 Motor (Chassis/Shooter)

M3508 motors require an external C620 speed controller (ESC).

**M3508 Motor Specifications:**
- Type: Brushless DC motor (requires external ESC)
- Rated voltage: 24V DC
- Rated current: 10A (peak 20A via C620)
- Rated power: 144W
- No-load speed: 469 RPM (without reduction)
- Stall torque: 3.0 N·m (at stall current 20A)
- Built-in reduction ratio: 19:1 (3591:187 internally)
- Encoder resolution: 8192 counts/revolution (13-bit, after reduction)
- Common use: Chassis wheels, friction wheels, feed motor

**C620 ESC Specifications:**
- Input voltage: 12-24V DC (typically 24V)
- Continuous current: 20A
- Peak current: 23A (short duration)
- Current control range: -20A to +20A (mapped to -16384 to +16384)
- Communication: CAN bus only (1 Mbps)
- Protection: Over-current, over-temperature, short-circuit

**Control Frequency Recommendation:**
- Minimum: 50 Hz (20ms interval)
- Recommended: 100-200 Hz (5-10ms interval)
- Maximum: 1000 Hz (1ms interval)

**Current to Torque Relationship:**
- Control value: -16384 to +16384 → -20A to +20A
- 1 unit ≈ 1.22 mA
- Actual output torque depends on motor load and speed
- At stall: 20A ≈ 3.0 N·m output torque

### Send Control (Board → ESC)

**Control Identifiers:**
- `0x200` - Controls IDs 1-4
- `0x1FF` - Controls IDs 5-8

**Current control range:** -16384 to 16384 (corresponds to -20A to 20A)

**DATA Field Structure (8 bytes):**

For CAN ID `0x200` (ESC IDs 1-4):
```
Byte 0-1: ESC 1 current (high byte, low byte)
Byte 2-3: ESC 2 current (high byte, low byte)
Byte 4-5: ESC 3 current (high byte, low byte)
Byte 6-7: ESC 4 current (high byte, low byte)
```

For CAN ID `0x1FF` (ESC IDs 5-8):
```
Byte 0-1: ESC 5 current (high byte, low byte)
Byte 2-3: ESC 6 current (high byte, low byte)
Byte 4-5: ESC 7 current (high byte, low byte)
Byte 6-7: ESC 8 current (high byte, low byte)
```

Each 16-bit current value is split as: `[High 8 bits][Low 8 bits]`

**Code example:**
```c
HAL_StatusTypeDef CAN_Manager_SendMotorCurrents4(CAN_HandleTypeDef *hcan, uint16_t std_id,
                                                int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    if (hcan == NULL) return HAL_ERROR;

    CAN_TxHeaderTypeDef tx = {0};
    uint8_t d[8];
    uint32_t mb;

    tx.StdId = std_id;  // 0x200 or 0x1FF
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    // Pack 4 motor currents
    d[0] = (uint8_t)(i1 >> 8); d[1] = (uint8_t)i1;
    d[2] = (uint8_t)(i2 >> 8); d[3] = (uint8_t)i2;
    d[4] = (uint8_t)(i3 >> 8); d[5] = (uint8_t)i3;
    d[6] = (uint8_t)(i4 >> 8); d[7] = (uint8_t)i4;

    return HAL_CAN_AddTxMessage(hcan, &tx, d, &mb);
}
```

### Receive Feedback (ESC → Board)

**Feedback Identifiers:** `0x200 + ESC_ID` (e.g., ID 1 → `0x201`, ID 2 → `0x202`, ..., ID 8 → `0x208`)

**DATA Field Structure (8 bytes):**
```
Byte 0-1: Rotor mechanical angle (uint16_t, 0-8191, maps to 0-360°)
Byte 2-3: Rotational speed in RPM (int16_t, signed value)
Byte 4-5: Actual output torque current (int16_t, signed value)
Byte 6:   Motor temperature (uint8_t, °C)
Byte 7:   Reserved (unused)
```

**Angle Calculation:**
- Raw value range: 0-8191 (13-bit effective resolution)
- Actual angle (degrees) = (raw_value / 8192) × 360°
- Actual angle (radians) = (raw_value / 8192) × 2π

Feedback includes: rotor angle, speed, torque current, temperature.

**Code example:**
```c
// C620 feedback handler
if (rx.IDE==CAN_ID_STD && rx.DLC==8 && rx.StdId>=0x201 && rx.StdId<=0x208) {
    uint8_t mid = rx.StdId - 0x201;  // Motor ID 0-7

    uint16_t angle   = (d[0]<<8) | d[1];
    int16_t  speed   = (int16_t)((d[2]<<8) | d[3]);
    int16_t  current = (int16_t)((d[4]<<8) | d[5]);
    uint8_t  temp    = d[6];

    // Process feedback...
}
```

### Hardware Setup

**ESC ID Setting (SET button):**

**Method A - One-by-one:**
1. Press SET once → LED off
2. Press SET **N times** (≤8) → ID = N (LED blinks orange each press)
3. Wait 3 seconds → auto-save
4. Power cycle to apply

**Method B - Quick (rotate motors):**
1. Press and hold SET until all LEDs are solid green
2. Rotate each M3508 rotor ≥180° in order → assigns IDs 1-8 sequentially
3. Power cycle to apply

**Termination Resistance:** Hardware switch on ESC. Only enable on the **two physical ends** of the CAN bus.

**LED Diagnostics:**
- Green blinks **N times/sec** → ESC ID = N
- Orange blinks **twice/sec** → **Duplicate ID detected** (fix immediately!)

---

## Troubleshooting & Best Practices

### Common Issues and Solutions

**Problem: Motor not responding**
1. Check CAN wiring (Red=CAN_H, Black=CAN_L)
2. Verify motor ID matches code configuration
3. Confirm termination resistors enabled at both bus ends only
4. Check power supply (24V connected and stable)
5. Use LED diagnostics to verify motor ID

**Problem: Duplicate ID detected (orange LED blinks 2×/sec)**
- This is CRITICAL - motors will conflict on the bus
- Reconfigure motor IDs immediately
- Verify each motor has unique ID (1-7 for GM6020, 1-8 for C620)
- Power cycle after ID change

**Problem: Intermittent communication / lost messages**
1. Check termination resistors (120Ω at both ends ONLY)
2. Verify wire connections are secure
3. Reduce CAN bus load (lower control frequency if possible)
4. Check for electromagnetic interference (EMI) from nearby motors
5. Ensure total bus length <40m at 1 Mbps

**Problem: Motor overheating**
- Verify current limits in code (-25000 to 25000 for GM6020, -16384 to 16384 for C620)
- Check mechanical resistance (gimbal may be blocked)
- Monitor temperature via feedback (Byte 6)
- Reduce duty cycle or add cooling

### Best Practices

**CAN Bus Design:**
- ✅ Use daisy-chain topology (linear bus)
- ✅ Enable 120Ω termination at both physical ends
- ✅ Keep wiring as short as possible
- ✅ Use twisted-pair or shielded cable for noise immunity
- ❌ Do NOT use star topology (multiple branches)
- ❌ Do NOT enable termination on middle nodes
- ❌ Do NOT exceed 40m total bus length at 1 Mbps

**Motor ID Assignment:**
- Assign sequential IDs (1, 2, 3, 4...) for easier management
- Document your ID mapping (which motor = which ID)
- Use lower IDs for critical motors (higher bus priority)
- Avoid gaps in ID sequence when possible

**Control Loop Timing:**
- Send control commands at consistent intervals
- GM6020: 200-500 Hz recommended for smooth gimbal control
- C620: 100-200 Hz sufficient for chassis/shooter
- Monitor CAN bus utilization (don't saturate the bus)
- Formula: Bus load ≈ (# of devices) × (control frequency) × (frame size)

**Code Safety:**
- Always clamp control values to valid ranges
- Implement timeout detection (if no feedback, stop motor)
- Validate received data (check CAN ID, DLC=8)
- Use proper byte order (big-endian: high byte first)

**Debugging Tips:**
- Use LED blink patterns to verify motor IDs
- Monitor feedback data to confirm communication
- Log CAN errors (check HAL_CAN error counters)
- Test one motor at a time when troubleshooting
- Use oscilloscope on CAN_H/CAN_L if persistent issues

### CAN Bus Load Calculation

Example: 4 GM6020 motors + 4 C620 ESCs at 200 Hz control frequency

**Frame breakdown:**
- Control frame: ~108 bits (SOF + ID + control + 8 data bytes + CRC + ACK + EOF + IFS)
- Each GM6020 feedback: ~108 bits
- Each C620 feedback: ~108 bits

**Calculation:**
- Control messages: 2 frames/cycle (0x1FF for GM6020s, 0x200 for C620s)
- Feedback messages: 8 frames/cycle (4 GM6020s + 4 C620s)
- Total: 10 frames × 108 bits = 1080 bits/cycle
- At 200 Hz: 1080 × 200 = 216,000 bits/sec
- Bus utilization: 216,000 / 1,000,000 = **21.6%** (safe)

**Safe thresholds:**
- <30%: Safe, reliable operation
- 30-50%: Acceptable, monitor for errors
- 50-70%: High load, reduce frequency or devices
- >70%: Saturated, expect message loss

### Important Warnings

⚠️ **Do not mix CAN and PWM modes** - Power off before switching control modes. Some motors support both CAN and PWM, but NEVER connect both simultaneously.

⚠️ **Termination resistance is critical** - Only enable 120Ω termination at the two physical ends of the bus. Incorrect termination causes signal reflection and communication errors.

⚠️ **Follow wire color coding** - Red=CAN_H, Black=CAN_L (ignore A/B labels on some connectors). Reversed wiring may work intermittently but causes unreliable communication.

⚠️ **Avoid bus conflicts** - Duplicate motor IDs cause unpredictable behavior. Always verify IDs using LED diagnostics before operation.

⚠️ **Monitor bus voltage** - CAN transceivers require stable 3.3V or 5V logic level. Low voltage causes communication failures.

---

## Official Documentation

- [GM6020 User Guide](user-guides/GM6020_User_Guide.pdf)
- [C620 User Guide](user-guides/C620_User_Guide.pdf)
- [C610 User Guide](user-guides/C610_User_Guide.pdf)
- [C Board User Guide](user-guides/CBoard_User_Guide.pdf)
