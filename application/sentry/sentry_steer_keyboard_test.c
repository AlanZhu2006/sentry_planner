#include "sentry_steer_keyboard_test.h"

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "dji_motor.h"
#include "general_def.h"
#include "led.h"
#include "robot_def.h"

#if defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_KEYBOARD_TEST

typedef enum
{
    STEER_KEYBOARD_LF = 0,
    STEER_KEYBOARD_RF,
    STEER_KEYBOARD_LR,
    STEER_KEYBOARD_RR,
} SteerKeyboardIndex_e;

#define DJI_ECD_FULL_RANGE 8192
#define DJI_ECD_HALF_RANGE 4096
#define DJI_ECD_QUARTER_RANGE 2048
#define DJI_ECD_EIGHTH_RANGE 1024
#define STEER_ALL_ONLINE_BITMAP 0x0fu
#define STEER_COMMAND_DISABLED '!'

static DJIMotorInstance *steer_motors[SENTRY_STEER_KEYBOARD_MOTOR_COUNT];
static DJIMotorInstance *drive_motors[SENTRY_STEER_KEYBOARD_MOTOR_COUNT];
static float target_total_angle_deg[SENTRY_STEER_KEYBOARD_MOTOR_COUNT];
static int8_t drive_target_sign[SENTRY_STEER_KEYBOARD_MOTOR_COUNT] = {1, 1, 1, 1};
static uint8_t *usb_rx_buffer;
static volatile uint32_t usb_rx_sequence;
static volatile uint8_t usb_rx_command = STEER_COMMAND_DISABLED;
static uint32_t handled_usb_rx_sequence;
static uint32_t last_command_ms;
static uint32_t online_stable_start_ms;
static uint32_t last_log_ms;
static uint8_t online_timer_started;
static uint8_t targets_initialized;
static uint8_t host_command_seen;
static uint8_t active_command = 'w';

volatile SentrySteerKeyboardState_s sentry_steer_keyboard_state;

/* Logical order: LF, RF, LR (ID 4), RR (ID 3). */
static const uint8_t steer_motor_ids[SENTRY_STEER_KEYBOARD_MOTOR_COUNT] = {
    [STEER_KEYBOARD_LF] = CHASSIS_STEER_MOTOR_LF_ID,
    [STEER_KEYBOARD_RF] = CHASSIS_STEER_MOTOR_RF_ID,
    [STEER_KEYBOARD_LR] = CHASSIS_STEER_MOTOR_LB_ID,
    [STEER_KEYBOARD_RR] = CHASSIS_STEER_MOTOR_RB_ID,
};

static const uint8_t drive_motor_ids[SENTRY_STEER_KEYBOARD_MOTOR_COUNT] = {
    [STEER_KEYBOARD_LF] = CHASSIS_DRIVE_MOTOR_LF_ID,
    [STEER_KEYBOARD_RF] = CHASSIS_DRIVE_MOTOR_RF_ID,
    [STEER_KEYBOARD_LR] = CHASSIS_DRIVE_MOTOR_LB_ID,
    [STEER_KEYBOARD_RR] = CHASSIS_DRIVE_MOTOR_RB_ID,
};

static const Motor_Reverse_Flag_e drive_motor_directions[SENTRY_STEER_KEYBOARD_MOTOR_COUNT] = {
    [STEER_KEYBOARD_LF] = CHASSIS_MOTOR_LF_REVERSE,
    [STEER_KEYBOARD_RF] = CHASSIS_MOTOR_RF_REVERSE,
    [STEER_KEYBOARD_LR] = CHASSIS_MOTOR_LB_REVERSE,
    [STEER_KEYBOARD_RR] = CHASSIS_MOTOR_RB_REVERSE,
};

static const uint16_t steer_zero_ecd[SENTRY_STEER_KEYBOARD_MOTOR_COUNT] = {
    [STEER_KEYBOARD_LF] = CHASSIS_STEER_MOTOR_LF_ZERO_ECD,
    [STEER_KEYBOARD_RF] = CHASSIS_STEER_MOTOR_RF_ZERO_ECD,
    [STEER_KEYBOARD_LR] = CHASSIS_STEER_MOTOR_LB_ZERO_ECD,
    [STEER_KEYBOARD_RR] = CHASSIS_STEER_MOTOR_RB_ZERO_ECD,
};

static const int8_t steer_direction_sign[SENTRY_STEER_KEYBOARD_MOTOR_COUNT] = {
    [STEER_KEYBOARD_LF] = CHASSIS_STEER_KEYBOARD_LF_DIRECTION_SIGN,
    [STEER_KEYBOARD_RF] = CHASSIS_STEER_KEYBOARD_RF_DIRECTION_SIGN,
    [STEER_KEYBOARD_LR] = CHASSIS_STEER_KEYBOARD_LR_DIRECTION_SIGN,
    [STEER_KEYBOARD_RR] = CHASSIS_STEER_KEYBOARD_RR_DIRECTION_SIGN,
};

static int16_t WrapEcdDelta(int32_t delta_ecd)
{
    if (delta_ecd > DJI_ECD_HALF_RANGE)
        delta_ecd -= DJI_ECD_FULL_RANGE;
    else if (delta_ecd < -DJI_ECD_HALF_RANGE)
        delta_ecd += DJI_ECD_FULL_RANGE;

    return (int16_t)delta_ecd;
}

static uint16_t WrapEcdTarget(int32_t target_ecd)
{
    target_ecd %= DJI_ECD_FULL_RANGE;
    if (target_ecd < 0)
        target_ecd += DJI_ECD_FULL_RANGE;
    return (uint16_t)target_ecd;
}

static int16_t CommandDirectionOffsetEcd(uint8_t command)
{
    switch (command)
    {
    case 'q':
        return -DJI_ECD_EIGHTH_RANGE;
    case 'e':
        return DJI_ECD_EIGHTH_RANGE;
    case 'a':
        return -DJI_ECD_QUARTER_RANGE;
    case 'z':
        return -(DJI_ECD_QUARTER_RANGE + DJI_ECD_EIGHTH_RANGE);
    case 'c':
        return DJI_ECD_QUARTER_RANGE + DJI_ECD_EIGHTH_RANGE;
    case 's':
        return DJI_ECD_HALF_RANGE;
    case 'd':
        return DJI_ECD_QUARTER_RANGE;
    case 'w':
    default:
        return 0;
    }
}

static uint8_t IsDirectionCommand(uint8_t command)
{
    return command == 'w' || command == 'a' || command == 's' || command == 'd' ||
           command == 'q' || command == 'e' || command == 'z' || command == 'c';
}

static void KeyboardUsbRxCallback(uint16_t length)
{
    for (uint16_t i = 0u; i < length; ++i)
    {
        uint8_t command = usb_rx_buffer[i];
        if (command >= 'A' && command <= 'Z')
            command = (uint8_t)(command + ('a' - 'A'));
        if (!IsDirectionCommand(command) && command != STEER_COMMAND_DISABLED)
            continue;

        usb_rx_command = command;
        ++usb_rx_sequence;
    }
}

static uint8_t MotorOnlineBitmap(DJIMotorInstance *const motors[SENTRY_STEER_KEYBOARD_MOTOR_COUNT])
{
    uint8_t bitmap = 0u;

    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        if (DaemonIsOnline(motors[i]->daemon))
            bitmap |= (uint8_t)(1u << i);
    }

    return bitmap;
}

static int MotorRpm(const DJIMotorInstance *motor)
{
    return (int)(motor->measure.speed_aps / RPM_2_ANGLE_PER_SEC);
}

static void StopAllSteerMotors(void)
{
    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        DJIMotorSetRef(steer_motors[i], steer_motors[i]->measure.total_angle);
        DJIMotorStop(steer_motors[i]);
    }
}

static void StopAllDriveMotors(void)
{
    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        DJIMotorSetRef(drive_motors[i], 0.0f);
        DJIMotorStop(drive_motors[i]);
    }

    sentry_steer_keyboard_state.drive_enabled = 0u;
}

static void RunAllDriveMotors(void)
{
    const float drive_target = CHASSIS_STEER_KEYBOARD_DRIVE_TARGET_RPM * RPM_2_ANGLE_PER_SEC;

    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        DJIMotorSetRef(drive_motors[i], drive_target * (float)drive_target_sign[i]);
        DJIMotorEnable(drive_motors[i]);
    }

    sentry_steer_keyboard_state.drive_enabled = 1u;
}

static void RefreshPositionSnapshot(void)
{
    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        sentry_steer_keyboard_state.current_ecd[i] = steer_motors[i]->measure.ecd;
        sentry_steer_keyboard_state.error_ecd[i] = WrapEcdDelta(
            (int32_t)sentry_steer_keyboard_state.target_ecd[i] -
            (int32_t)steer_motors[i]->measure.ecd);
    }
}

static void SetDirectionTargets(uint8_t command)
{
    const int16_t direction_offset_ecd = CommandDirectionOffsetEcd(command);

    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        uint16_t target_ecd = WrapEcdTarget(
            (int32_t)steer_zero_ecd[i] +
            (int32_t)steer_direction_sign[i] * direction_offset_ecd);
        int16_t delta_ecd = WrapEcdDelta(
            (int32_t)target_ecd - (int32_t)steer_motors[i]->measure.ecd);

        drive_target_sign[i] = 1;
        if (delta_ecd > DJI_ECD_QUARTER_RANGE)
        {
            delta_ecd -= DJI_ECD_HALF_RANGE;
            drive_target_sign[i] = -1;
        }
        else if (delta_ecd < -DJI_ECD_QUARTER_RANGE)
        {
            delta_ecd += DJI_ECD_HALF_RANGE;
            drive_target_sign[i] = -1;
        }

        target_ecd = WrapEcdTarget(
            (int32_t)steer_motors[i]->measure.ecd + (int32_t)delta_ecd);
        sentry_steer_keyboard_state.target_ecd[i] = target_ecd;
        target_total_angle_deg[i] = steer_motors[i]->measure.total_angle +
                                    (float)delta_ecd * ECD_ANGLE_COEF_DJI;
        DJIMotorSetRef(steer_motors[i], target_total_angle_deg[i]);
    }

    targets_initialized = 1u;
}

static void LogKeyboardState(uint32_t now_ms)
{
    if ((now_ms - last_log_ms) < SENTRY_STEER_KEYBOARD_LOG_PERIOD_MS)
        return;

    last_log_ms = now_ms;
    LOGINFO("[steer-keyboard] cmd=%c steer=0x%x drive=0x%x enabled=%u/%u aligned=0x%x",
            sentry_steer_keyboard_state.command,
            sentry_steer_keyboard_state.online_bitmap,
            sentry_steer_keyboard_state.drive_online_bitmap,
            sentry_steer_keyboard_state.control_enabled,
            sentry_steer_keyboard_state.drive_enabled,
            sentry_steer_keyboard_state.aligned_bitmap);
    LOGINFO("[steer-keyboard] ecd LF/RF/LR/RR=%u/%u/%u/%u",
            sentry_steer_keyboard_state.current_ecd[STEER_KEYBOARD_LF],
            sentry_steer_keyboard_state.current_ecd[STEER_KEYBOARD_RF],
            sentry_steer_keyboard_state.current_ecd[STEER_KEYBOARD_LR],
            sentry_steer_keyboard_state.current_ecd[STEER_KEYBOARD_RR]);
    LOGINFO("[steer-keyboard] error LF/RF/LR/RR=%d/%d/%d/%d",
            sentry_steer_keyboard_state.error_ecd[STEER_KEYBOARD_LF],
            sentry_steer_keyboard_state.error_ecd[STEER_KEYBOARD_RF],
            sentry_steer_keyboard_state.error_ecd[STEER_KEYBOARD_LR],
            sentry_steer_keyboard_state.error_ecd[STEER_KEYBOARD_RR]);
    LOGINFO("[steer-keyboard] drive rpm LF/RF/LR/RR=%d/%d/%d/%d",
            MotorRpm(drive_motors[STEER_KEYBOARD_LF]),
            MotorRpm(drive_motors[STEER_KEYBOARD_RF]),
            MotorRpm(drive_motors[STEER_KEYBOARD_LR]),
            MotorRpm(drive_motors[STEER_KEYBOARD_RR]));
}

void SentrySteerKeyboardTestInit(void)
{
    Motor_Init_Config_s motor_config = {
        .can_init_config.can_handle = NULL,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = CHASSIS_STEER_KEYBOARD_ANGLE_PID_KP,
                .Ki = CHASSIS_STEER_KEYBOARD_ANGLE_PID_KI,
                .Kd = CHASSIS_STEER_KEYBOARD_ANGLE_PID_KD,
                .DeadBand = CHASSIS_STEER_KEYBOARD_ANGLE_PID_DEADBAND,
                .IntegralLimit = CHASSIS_STEER_KEYBOARD_ANGLE_PID_INT_LIMIT,
                .Improve = PID_Trapezoid_Intergral |
                           PID_Integral_Limit |
                           PID_OutputFilter,
                .Output_LPF_RC = CHASSIS_STEER_KEYBOARD_ANGLE_PID_OUTPUT_LPF_RC,
                .MaxOut = CHASSIS_STEER_KEYBOARD_MAX_SPEED_APS,
            },
            .speed_PID = {
                .Kp = CHASSIS_STEER_KEYBOARD_SPEED_PID_KP,
                .Ki = CHASSIS_STEER_KEYBOARD_SPEED_PID_KI,
                .Kd = CHASSIS_STEER_KEYBOARD_SPEED_PID_KD,
                .DeadBand = CHASSIS_STEER_KEYBOARD_SPEED_PID_DEADBAND,
                .IntegralLimit = CHASSIS_STEER_KEYBOARD_SPEED_PID_INT_LIMIT,
                .Improve = PID_Trapezoid_Intergral |
                           PID_Integral_Limit |
                           PID_OutputFilter,
                .Output_LPF_RC = CHASSIS_STEER_KEYBOARD_SPEED_PID_OUTPUT_LPF_RC,
                .MaxOut = CHASSIS_STEER_KEYBOARD_MAX_CURRENT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    Motor_Init_Config_s drive_config = {
        .can_init_config.can_handle = NULL,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = CHASSIS_SPEED_PID_KP,
                .Ki = CHASSIS_SPEED_PID_KI,
                .Kd = CHASSIS_SPEED_PID_KD,
                .IntegralLimit = CHASSIS_SPEED_PID_INT_LIMIT,
                .MaxOut = CHASSIS_SPEED_PID_MAX_OUT,
            },
            .current_PID = {
                .Kp = CHASSIS_CURRENT_PID_KP,
                .Ki = CHASSIS_CURRENT_PID_KI,
                .Kd = CHASSIS_CURRENT_PID_KD,
                .IntegralLimit = CHASSIS_CURRENT_PID_INT_LIMIT,
                .MaxOut = CHASSIS_CURRENT_PID_MAX_OUT,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    USB_Init_Config_s usb_config = {.rx_cbk = KeyboardUsbRxCallback};

    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        motor_config.can_init_config.can_handle = CHASSIS_MOTOR_CAN_HANDLE(steer_motor_ids[i]);
        motor_config.can_init_config.tx_id = steer_motor_ids[i];
        steer_motors[i] = DJIMotorInit(&motor_config);
        DJIMotorStop(steer_motors[i]);
        sentry_steer_keyboard_state.target_ecd[i] = steer_zero_ecd[i];

        drive_config.can_init_config.can_handle = CHASSIS_MOTOR_CAN_HANDLE(drive_motor_ids[i]);
        drive_config.can_init_config.tx_id = drive_motor_ids[i];
        drive_config.controller_setting_init_config.motor_reverse_flag = drive_motor_directions[i];
        drive_motors[i] = DJIMotorInit(&drive_config);
        DJIMotorSetRef(drive_motors[i], 0.0f);
        DJIMotorStop(drive_motors[i]);
    }

    usb_rx_buffer = USBInit(usb_config);
    sentry_steer_keyboard_state.command = 'w';
    LEDSetStatus(LED_STATUS_RED_ON);
    LOGINFO("[steer-keyboard] waiting for 4x GM6020 and 4x M3508 feedback before automatic forward homing");
}

void SentrySteerKeyboardTestTask(void)
{
    const uint32_t now_ms = (uint32_t)DWT_GetTimeline_ms();
    const uint8_t online_bitmap = MotorOnlineBitmap(steer_motors);
    const uint8_t drive_online_bitmap = MotorOnlineBitmap(drive_motors);
    uint8_t aligned_bitmap = 0u;

    if (handled_usb_rx_sequence != usb_rx_sequence)
    {
        const uint8_t command = usb_rx_command;
        handled_usb_rx_sequence = usb_rx_sequence;
        last_command_ms = now_ms;
        host_command_seen = 1u;
        if (command != active_command)
        {
            active_command = command;
            targets_initialized = 0u;
        }
    }

    if (host_command_seen &&
        (now_ms - last_command_ms) > CHASSIS_STEER_KEYBOARD_COMMAND_TIMEOUT_MS)
    {
        active_command = STEER_COMMAND_DISABLED;
        targets_initialized = 0u;
    }

    sentry_steer_keyboard_state.command = active_command;
    sentry_steer_keyboard_state.online_bitmap = online_bitmap;
    sentry_steer_keyboard_state.drive_online_bitmap = drive_online_bitmap;
    RefreshPositionSnapshot();

    if (online_bitmap != STEER_ALL_ONLINE_BITMAP ||
        drive_online_bitmap != STEER_ALL_ONLINE_BITMAP)
    {
        StopAllSteerMotors();
        StopAllDriveMotors();
        online_timer_started = 0u;
        targets_initialized = 0u;
        sentry_steer_keyboard_state.control_enabled = 0u;
        sentry_steer_keyboard_state.aligned_bitmap = 0u;
        LEDSetStatus(LED_STATUS_RED_ON);
        LogKeyboardState(now_ms);
        return;
    }

    if (!online_timer_started)
    {
        online_stable_start_ms = now_ms;
        online_timer_started = 1u;
    }

    if (!IsDirectionCommand(active_command) ||
        (now_ms - online_stable_start_ms) < CHASSIS_STEER_KEYBOARD_ONLINE_STABLE_MS)
    {
        StopAllSteerMotors();
        StopAllDriveMotors();
        sentry_steer_keyboard_state.control_enabled = 0u;
        sentry_steer_keyboard_state.aligned_bitmap = 0u;
        LEDSetStatus(LED_STATUS_BLUE_ON);
        LogKeyboardState(now_ms);
        return;
    }

    if (!targets_initialized)
        SetDirectionTargets(active_command);

    RefreshPositionSnapshot();
    for (uint8_t i = 0u; i < SENTRY_STEER_KEYBOARD_MOTOR_COUNT; ++i)
    {
        DJIMotorSetRef(steer_motors[i], target_total_angle_deg[i]);
        DJIMotorEnable(steer_motors[i]);
        if (abs(sentry_steer_keyboard_state.error_ecd[i]) <= CHASSIS_STEER_KEYBOARD_TOLERANCE_ECD)
            aligned_bitmap |= (uint8_t)(1u << i);
    }

    sentry_steer_keyboard_state.control_enabled = 1u;
    sentry_steer_keyboard_state.aligned_bitmap = aligned_bitmap;
    if (host_command_seen && IsDirectionCommand(active_command))
        RunAllDriveMotors();
    else
        StopAllDriveMotors();
    LEDSetStatus(aligned_bitmap == STEER_ALL_ONLINE_BITMAP ? LED_STATUS_GREEN_ON : LED_STATUS_YELLOW_ON);
    LogKeyboardState(now_ms);
}

#else

volatile SentrySteerKeyboardState_s sentry_steer_keyboard_state;

void SentrySteerKeyboardTestInit(void) {}
void SentrySteerKeyboardTestTask(void) {}

#endif
