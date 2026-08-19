#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

#define DISABLE_GIMBAL_MOTORS 0
#define GIMBAL_GYRO_SPEED_LPF_ALPHA 0.35f
#define GIMBAL_CTRL_DT_S 0.005f
#define DJI_YAW_ECD_FULL_RANGE 8192
#define DJI_YAW_ECD_HALF_RANGE (DJI_YAW_ECD_FULL_RANGE / 2)

#ifndef GIMBAL_YAW_SPEED_FEEDFORWARD_ENABLE
#define GIMBAL_YAW_SPEED_FEEDFORWARD_ENABLE 0
#endif

#ifndef GIMBAL_YAW_SPEED_FEEDFORWARD_GAIN
#define GIMBAL_YAW_SPEED_FEEDFORWARD_GAIN 0.6f
#endif

#ifndef GIMBAL_YAW_SPEED_FEEDFORWARD_LPF_ALPHA
#define GIMBAL_YAW_SPEED_FEEDFORWARD_LPF_ALPHA 0.25f
#endif

#ifndef GIMBAL_YAW_SPEED_FEEDFORWARD_MAX_DEG_S
#define GIMBAL_YAW_SPEED_FEEDFORWARD_MAX_DEG_S 400.0f
#endif

#ifndef GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK
#define GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK 0
#endif

#ifndef GIMBAL_PITCH_ANGLE_PID_DEADBAND
#define GIMBAL_PITCH_ANGLE_PID_DEADBAND 0.0f
#endif

#ifndef GIMBAL_PITCH_SPEED_PID_DEADBAND
#define GIMBAL_PITCH_SPEED_PID_DEADBAND 0.0f
#endif

#ifndef GIMBAL_PITCH_ANGLE_DERIVATIVE_LPF_RC
#define GIMBAL_PITCH_ANGLE_DERIVATIVE_LPF_RC 0.0f
#endif

#ifndef GIMBAL_PITCH_SPEED_DERIVATIVE_LPF_RC
#define GIMBAL_PITCH_SPEED_DERIVATIVE_LPF_RC 0.0f
#endif

#ifndef GIMBAL_PITCH_PID_USE_DERIVATIVE_FILTER
#define GIMBAL_PITCH_PID_USE_DERIVATIVE_FILTER 0
#endif

#ifndef GIMBAL_PITCH_MOTOR_BACKEND_DJI
#define GIMBAL_PITCH_MOTOR_BACKEND_DJI 0
#endif

#ifndef GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
#define GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO 1
#endif

#ifndef GIMBAL_PITCH_MOTOR_BACKEND
#define GIMBAL_PITCH_MOTOR_BACKEND GIMBAL_PITCH_MOTOR_BACKEND_DJI
#endif

#ifndef GIMBAL_PITCH_MOTOR_MASTER_ID
#define GIMBAL_PITCH_MOTOR_MASTER_ID 0u
#endif

#ifndef GIMBAL_PITCH_DAMIAO_SPEED_MAX_DEG_S
#define GIMBAL_PITCH_DAMIAO_SPEED_MAX_DEG_S GIMBAL_PITCH_ANGLE_PID_MAX_OUT
#endif

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor;
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
static DMMotorInstance *pitch_motor;
static PIDInstance pitch_angle_pid;
static PID_Init_Config_s pitch_angle_pid_config;
static float pitch_target_speed_deg_s;
static gimbal_mode_e last_gimbal_mode;
static uint8_t last_gimbal_mode_valid;
#else
static DJIMotorInstance *pitch_motor;
#endif
static float yaw_ctrl_feedback_deg;
static float yaw_ctrl_feedback_deg_s;
static float yaw_gyro_deg_s_lpf;
static float pitch_gyro_deg_s_lpf;
static uint8_t gyro_speed_filter_inited;
static float yaw_speed_feedforward_deg_s;
static float yaw_cmd_last_deg;
static uint8_t yaw_speed_feedforward_inited;
static gimbal_mode_e yaw_speed_feedforward_last_mode;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static float ReverseFlagToSign(Motor_Reverse_Flag_e reverse_flag)
{
    return (reverse_flag == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
}

static uint8_t YawMotorFeedbackReady(void)
{
    return (uint8_t)(yaw_motor->feed_cnt != 0u);
}

static int16_t WrapYawEncoderDeltaEcd(int32_t delta_ecd)
{
    if (delta_ecd > DJI_YAW_ECD_HALF_RANGE)
        delta_ecd -= DJI_YAW_ECD_FULL_RANGE;
    else if (delta_ecd < -DJI_YAW_ECD_HALF_RANGE)
        delta_ecd += DJI_YAW_ECD_FULL_RANGE;

    return (int16_t)delta_ecd;
}

static float YawCtrlToMotorRelativeRawDeg(float ctrl_deg)
{
    return ctrl_deg * (float)GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN;
}

static float YawMotorRelativeRawDegToCtrlDeg(float motor_relative_raw_deg)
{
    float ref_sign = (float)GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN;

    if (ref_sign == 0.0f)
    {
        return 0.0f;
    }

    return motor_relative_raw_deg / ref_sign;
}

static float YawCtrlToMotorRefDeg(float ctrl_deg)
{
    return ctrl_deg * (float)GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN *
           ReverseFlagToSign(GIMBAL_YAW_MOTOR_REVERSE);
}

static float YawMotorTotalRefToPidRefDeg(float motor_total_ref_deg)
{
    return motor_total_ref_deg *
           ReverseFlagToSign(GIMBAL_YAW_MOTOR_REVERSE);
}

static float CurrentYawMotorRelativeRawDeg(void)
{
    int32_t delta_ecd = (int32_t)WrapYawEncoderDeltaEcd(
        (int32_t)yaw_motor->measure.ecd - (int32_t)YAW_CHASSIS_ALIGN_ECD);
    return (float)delta_ecd * ECD_ANGLE_COEF_DJI;
}

static float YawCtrlToMotorTotalRefDeg(float ctrl_deg)
{
    float target_relative_raw_deg = YawCtrlToMotorRelativeRawDeg(ctrl_deg);
    float current_relative_raw_deg = CurrentYawMotorRelativeRawDeg();

    return yaw_motor->measure.total_angle +
           (target_relative_raw_deg - current_relative_raw_deg);
}

static int32_t ClampYawTargetDeltaEcd(int32_t target_delta_ecd)
{
#if defined(YAW_RIGHT_LIMIT_DELTA_ECD) && defined(YAW_LEFT_LIMIT_DELTA_ECD)
    int32_t right_limit_delta_ecd;
    int32_t left_limit_delta_ecd;
    int32_t min_limit_delta_ecd;
    int32_t max_limit_delta_ecd;

    if (YawCtrlToMotorRelativeRawDeg(1.0f) >= 0.0f)
    {
        right_limit_delta_ecd = (int32_t)YAW_RIGHT_LIMIT_DELTA_ECD;
        left_limit_delta_ecd = -(int32_t)YAW_LEFT_LIMIT_DELTA_ECD;
    }
    else
    {
        right_limit_delta_ecd = -(int32_t)YAW_RIGHT_LIMIT_DELTA_ECD;
        left_limit_delta_ecd = (int32_t)YAW_LEFT_LIMIT_DELTA_ECD;
    }

    min_limit_delta_ecd = (right_limit_delta_ecd < left_limit_delta_ecd)
                              ? right_limit_delta_ecd
                              : left_limit_delta_ecd;
    max_limit_delta_ecd = (right_limit_delta_ecd > left_limit_delta_ecd)
                              ? right_limit_delta_ecd
                              : left_limit_delta_ecd;

    if (target_delta_ecd > max_limit_delta_ecd)
        target_delta_ecd = max_limit_delta_ecd;
    if (target_delta_ecd < min_limit_delta_ecd)
        target_delta_ecd = min_limit_delta_ecd;
#endif

    return target_delta_ecd;
}

static float ClampYawRelativeCtrlCmdToEncoderWindow(float yaw_cmd)
{
    int32_t target_delta_ecd =
        (int32_t)lroundf(YawCtrlToMotorRelativeRawDeg(yaw_cmd) /
                         ECD_ANGLE_COEF_DJI);
    target_delta_ecd = ClampYawTargetDeltaEcd(target_delta_ecd);
    return YawMotorRelativeRawDegToCtrlDeg((float)target_delta_ecd *
                                           ECD_ANGLE_COEF_DJI);
}

static float YawPidSpeedRefToCtrlDegS(float pid_speed_ref_deg_s)
{
#if GIMBAL_YAW_PID_USE_CTRL_FEEDBACK
    return pid_speed_ref_deg_s;
#else
    return pid_speed_ref_deg_s * (float)GIMBAL_YAW_CTRL_FROM_IMU_SIGN;
#endif
}

static float YawCtrlSpeedToPidRefDegS(float ctrl_deg_s)
{
#if GIMBAL_YAW_PID_USE_CTRL_FEEDBACK
    return ctrl_deg_s;
#else
    return ctrl_deg_s * (float)GIMBAL_YAW_CTRL_FROM_IMU_SIGN;
#endif
}

#ifndef GIMBAL_PITCH_FEEDBACK_FROM_IMU_SIGN
#define GIMBAL_PITCH_FEEDBACK_FROM_IMU_SIGN GYRO2GIMBAL_DIR_PITCH
#endif

static float PitchFeedbackFromImuDeg(float imu_deg)
{
    return imu_deg * (float)GIMBAL_PITCH_FEEDBACK_FROM_IMU_SIGN;
}

static float PitchFeedbackFromImuDegS(float imu_deg_s)
{
    return imu_deg_s * (float)GIMBAL_PITCH_FEEDBACK_FROM_IMU_SIGN;
}

static float PitchPidSpeedRefToCtrlDegS(float pid_speed_ref_deg_s)
{
    return pid_speed_ref_deg_s * (float)GIMBAL_PITCH_FEEDBACK_FROM_IMU_SIGN;
}

static uint16_t PitchMotorEncoderRaw(void)
{
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
    float normalized_position =
        (pitch_motor->measure.position - DM_P_MIN) / (DM_P_MAX - DM_P_MIN);

    LIMIT_MIN_MAX(normalized_position, 0.0f, 1.0f);
    return (uint16_t)(normalized_position * 65535.0f + 0.5f);
#else
    return pitch_motor->measure.ecd;
#endif
}

static void ResetYawSpeedFeedforward(float current_yaw_cmd, gimbal_mode_e mode)
{
    yaw_speed_feedforward_deg_s = 0.0f;
    yaw_cmd_last_deg = current_yaw_cmd;
    yaw_speed_feedforward_inited = 1u;
    yaw_speed_feedforward_last_mode = mode;
}

static void UpdateYawSpeedFeedforward(float current_yaw_cmd, gimbal_mode_e mode)
{
#if !GIMBAL_YAW_SPEED_FEEDFORWARD_ENABLE
    UNUSED(current_yaw_cmd);
    UNUSED(mode);
    yaw_speed_feedforward_deg_s = 0.0f;
    yaw_speed_feedforward_inited = 0u;
    return;
#else
    float ctrl_speed_ff_deg_s;
    float pid_speed_ff_deg_s;

    if ((mode != GIMBAL_GYRO_MODE && mode != GIMBAL_FREE_MODE) ||
        !yaw_speed_feedforward_inited || yaw_speed_feedforward_last_mode != mode)
    {
        ResetYawSpeedFeedforward(current_yaw_cmd, mode);
        return;
    }

    ctrl_speed_ff_deg_s = (current_yaw_cmd - yaw_cmd_last_deg) / GIMBAL_CTRL_DT_S;
    yaw_cmd_last_deg = current_yaw_cmd;

    ctrl_speed_ff_deg_s *= GIMBAL_YAW_SPEED_FEEDFORWARD_GAIN;
    LIMIT_MIN_MAX(ctrl_speed_ff_deg_s,
                  -GIMBAL_YAW_SPEED_FEEDFORWARD_MAX_DEG_S,
                  GIMBAL_YAW_SPEED_FEEDFORWARD_MAX_DEG_S);

    pid_speed_ff_deg_s = YawCtrlSpeedToPidRefDegS(ctrl_speed_ff_deg_s);
    yaw_speed_feedforward_deg_s +=
        (pid_speed_ff_deg_s - yaw_speed_feedforward_deg_s) *
        GIMBAL_YAW_SPEED_FEEDFORWARD_LPF_ALPHA;
#endif
}

static float PitchCtrlToMotorRefDeg(float ctrl_deg)
{
    return ctrl_deg * (float)GYRO2GIMBAL_DIR_PITCH *
           ReverseFlagToSign(GIMBAL_PITCH_MOTOR_REVERSE);
}

#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
static float PitchCtrlToMotorSpeedRefRad(float ctrl_deg_s)
{
    return ctrl_deg_s * DEGREE_2_RAD;
}

static float PitchClampDamiaoSpeedDeg(float speed_deg_s)
{
    if (GIMBAL_PITCH_DAMIAO_SPEED_MAX_DEG_S > 0.0f)
    {
        LIMIT_MIN_MAX(speed_deg_s,
                      -GIMBAL_PITCH_DAMIAO_SPEED_MAX_DEG_S,
                      GIMBAL_PITCH_DAMIAO_SPEED_MAX_DEG_S);
    }
    return speed_deg_s;
}

static void PitchDmResetAnglePid(void)
{
    PIDInit(&pitch_angle_pid, &pitch_angle_pid_config);
    pitch_target_speed_deg_s = 0.0f;
}
#endif

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{
#if DISABLE_GIMBAL_MOTORS
    return;
#endif
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &GIMBAL_YAW_CAN_BUS,
            .tx_id = GIMBAL_YAW_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = GIMBAL_YAW_ANGLE_PID_KP,
                .Ki = GIMBAL_YAW_ANGLE_PID_KI,
                .Kd = GIMBAL_YAW_ANGLE_PID_KD,
                .DeadBand = GIMBAL_YAW_ANGLE_PID_DEADBAND,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = GIMBAL_YAW_ANGLE_PID_INT_LIMIT,

                .MaxOut = GIMBAL_YAW_ANGLE_PID_MAX_OUT,
            },
            .speed_PID = {
                .Kp = GIMBAL_YAW_SPEED_PID_KP,
                .Ki = GIMBAL_YAW_SPEED_PID_KI,
                .Kd = GIMBAL_YAW_SPEED_PID_KD,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = GIMBAL_YAW_SPEED_PID_INT_LIMIT,
                .MaxOut = GIMBAL_YAW_SPEED_PID_MAX_OUT,
            },
#if GIMBAL_YAW_PID_USE_CTRL_FEEDBACK
            .other_angle_feedback_ptr = &yaw_ctrl_feedback_deg,
            .other_speed_feedback_ptr = &yaw_ctrl_feedback_deg_s,
#else
            .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
            .other_speed_feedback_ptr = &yaw_gyro_deg_s_lpf,
#endif
            .speed_feedforward_ptr = &yaw_speed_feedforward_deg_s,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = GIMBAL_YAW_MOTOR_REVERSE,
            .feedforward_flag = GIMBAL_YAW_SPEED_FEEDFORWARD_ENABLE ? SPEED_FEEDFORWARD : FEEDFORWARD_NONE,
        },
        .motor_type = GM6020};
    // PITCH
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &GIMBAL_PITCH_CAN_BUS,
            .tx_id = GIMBAL_PITCH_MOTOR_ID,
            .rx_id = GIMBAL_PITCH_MOTOR_MASTER_ID,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = GIMBAL_PITCH_MOTOR_REVERSE,
        },
        .motor_type = MOTOR_TYPE_NONE,
        .dm_auto_zero_on_boot = 0u,
    };

    pitch_angle_pid_config = (PID_Init_Config_s){
        .Kp = GIMBAL_PITCH_ANGLE_PID_KP,
        .Ki = GIMBAL_PITCH_ANGLE_PID_KI,
        .Kd = GIMBAL_PITCH_ANGLE_PID_KD,
        .DeadBand = GIMBAL_PITCH_ANGLE_PID_DEADBAND,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement |
                   (GIMBAL_PITCH_PID_USE_DERIVATIVE_FILTER ? PID_DerivativeFilter : PID_IMPROVE_NONE),
        .IntegralLimit = GIMBAL_PITCH_ANGLE_PID_INT_LIMIT,
        .Derivative_LPF_RC = GIMBAL_PITCH_ANGLE_DERIVATIVE_LPF_RC,
        .MaxOut = GIMBAL_PITCH_ANGLE_PID_MAX_OUT,
    };
#else
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &GIMBAL_PITCH_CAN_BUS,
            .tx_id = GIMBAL_PITCH_MOTOR_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = GIMBAL_PITCH_ANGLE_PID_KP,
                .Ki = GIMBAL_PITCH_ANGLE_PID_KI,
                .Kd = GIMBAL_PITCH_ANGLE_PID_KD,
                .DeadBand = GIMBAL_PITCH_ANGLE_PID_DEADBAND,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement |
                           (GIMBAL_PITCH_PID_USE_DERIVATIVE_FILTER ? PID_DerivativeFilter : PID_IMPROVE_NONE),
                .IntegralLimit = GIMBAL_PITCH_ANGLE_PID_INT_LIMIT,
                .Derivative_LPF_RC = GIMBAL_PITCH_ANGLE_DERIVATIVE_LPF_RC,
                .MaxOut = GIMBAL_PITCH_ANGLE_PID_MAX_OUT,
            },
            .speed_PID = {
                .Kp = GIMBAL_PITCH_SPEED_PID_KP,
                .Ki = GIMBAL_PITCH_SPEED_PID_KI,
                .Kd = GIMBAL_PITCH_SPEED_PID_KD,
                .DeadBand = GIMBAL_PITCH_SPEED_PID_DEADBAND,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement |
                           (GIMBAL_PITCH_PID_USE_DERIVATIVE_FILTER ? PID_DerivativeFilter : PID_IMPROVE_NONE),
                .IntegralLimit = GIMBAL_PITCH_SPEED_PID_INT_LIMIT,
                .Derivative_LPF_RC = GIMBAL_PITCH_SPEED_DERIVATIVE_LPF_RC,
                .MaxOut = GIMBAL_PITCH_SPEED_PID_MAX_OUT,
            },
            .other_angle_feedback_ptr = &gimba_IMU_data->Pitch,
            .other_speed_feedback_ptr = &pitch_gyro_deg_s_lpf,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = GIMBAL_PITCH_MOTOR_REVERSE,
        },
        .motor_type = GM6020,
    };
#endif
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
    PitchDmResetAnglePid();
    pitch_motor = DMMotorInit(&pitch_config);
    DMMotorSetControlMode(pitch_motor, DM_MOTOR_CONTROL_SPEED);
    DMMotorSetRef(pitch_motor, 0.0f);
#else
    pitch_motor = DJIMotorInit(&pitch_config);
#endif

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
#if DISABLE_GIMBAL_MOTORS
    return;
#endif
    float yaw_feedback_angle_deg;
    float yaw_feedback_speed_deg_s;
    float yaw_target_speed_deg_s;
    float yaw_target_angle_deg;
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    {
        float yaw_gyro_deg_s = gimba_IMU_data->Gyro[2] * RAD_2_DEGREE;
        float pitch_gyro_deg_s = gimba_IMU_data->Gyro[0] * RAD_2_DEGREE;
        if (!gyro_speed_filter_inited)
        {
            gyro_speed_filter_inited = 1u;
            yaw_gyro_deg_s_lpf = yaw_gyro_deg_s;
            pitch_gyro_deg_s_lpf = pitch_gyro_deg_s;
        }
        else
        {
            yaw_gyro_deg_s_lpf += (yaw_gyro_deg_s - yaw_gyro_deg_s_lpf) * GIMBAL_GYRO_SPEED_LPF_ALPHA;
            pitch_gyro_deg_s_lpf += (pitch_gyro_deg_s - pitch_gyro_deg_s_lpf) * GIMBAL_GYRO_SPEED_LPF_ALPHA;
        }
    }
    yaw_ctrl_feedback_deg = gimba_IMU_data->YawTotalAngle * (float)GIMBAL_YAW_CTRL_FROM_IMU_SIGN;
    yaw_ctrl_feedback_deg_s = yaw_gyro_deg_s_lpf * (float)GIMBAL_YAW_CTRL_FROM_IMU_SIGN;
    float pitch_feedback_deg = PitchFeedbackFromImuDeg(gimba_IMU_data->Pitch);
    float pitch_feedback_deg_s = PitchFeedbackFromImuDegS(pitch_gyro_deg_s_lpf);
    yaw_feedback_angle_deg = yaw_ctrl_feedback_deg;
    yaw_feedback_speed_deg_s = yaw_ctrl_feedback_deg_s;
    yaw_target_angle_deg = gimbal_cmd_recv.yaw;
    yaw_target_speed_deg_s =
        YawPidSpeedRefToCtrlDegS(yaw_motor->motor_controller.speed_PID.Ref);

#if GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK
    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_FREE_MODE && YawMotorFeedbackReady())
    {
        yaw_target_angle_deg =
            ClampYawRelativeCtrlCmdToEncoderWindow(gimbal_cmd_recv.yaw);
    }
#endif

#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
    if (!last_gimbal_mode_valid || last_gimbal_mode != gimbal_cmd_recv.gimbal_mode)
    {
        if (gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)
            PitchDmResetAnglePid();

        last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;
        last_gimbal_mode_valid = 1u;
    }
#endif

    UpdateYawSpeedFeedforward(yaw_target_angle_deg, gimbal_cmd_recv.gimbal_mode);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
        pitch_target_speed_deg_s = 0.0f;
        DMMotorSetRef(pitch_motor, 0.0f);
        DMMotorStop(pitch_motor);
#else
        DJIMotorStop(pitch_motor);
#endif
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, YawCtrlToMotorRefDeg(gimbal_cmd_recv.yaw));
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
        DMMotorEnable(pitch_motor);
        pitch_target_speed_deg_s = PIDCalculate(&pitch_angle_pid, pitch_feedback_deg, gimbal_cmd_recv.pitch);
        pitch_target_speed_deg_s = PitchClampDamiaoSpeedDeg(pitch_target_speed_deg_s);
        DMMotorSetRef(pitch_motor, PitchCtrlToMotorSpeedRefRad(pitch_target_speed_deg_s));
#else
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(pitch_motor, PitchCtrlToMotorRefDeg(gimbal_cmd_recv.pitch));
#endif
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
#if GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK
        if (!YawMotorFeedbackReady())
        {
            DJIMotorStop(yaw_motor);
        }
        else
        {
            DJIMotorEnable(yaw_motor);
            DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
            DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
            DJIMotorSetRef(
                yaw_motor,
                YawMotorTotalRefToPidRefDeg(YawCtrlToMotorTotalRefDeg(yaw_target_angle_deg)));
        }
#else
        DJIMotorEnable(yaw_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, YawCtrlToMotorRefDeg(gimbal_cmd_recv.yaw));
#endif
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
        DMMotorEnable(pitch_motor);
        pitch_target_speed_deg_s = PIDCalculate(&pitch_angle_pid, pitch_feedback_deg, gimbal_cmd_recv.pitch);
        pitch_target_speed_deg_s = PitchClampDamiaoSpeedDeg(pitch_target_speed_deg_s);
        DMMotorSetRef(pitch_motor, PitchCtrlToMotorSpeedRefRad(pitch_target_speed_deg_s));
#else
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(pitch_motor, PitchCtrlToMotorRefDeg(gimbal_cmd_recv.pitch));
#endif
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.yaw_motor_encoder_raw = yaw_motor->measure.ecd;
    gimbal_feedback_data.pitch_motor_encoder_raw = PitchMotorEncoderRaw();
    gimbal_feedback_data.motor_online_bitmap = 0u;
    gimbal_feedback_data.motor_online_bitmap |=
        (uint8_t)((DaemonIsOnline(yaw_motor->daemon) && YawMotorFeedbackReady()) ? 0x01u : 0u);
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
    gimbal_feedback_data.motor_online_bitmap |= (uint8_t)(DMMotorIsOnline(pitch_motor) ? 0x02u : 0u);
#else
    gimbal_feedback_data.motor_online_bitmap |= (uint8_t)(DaemonIsOnline(pitch_motor->daemon) ? 0x02u : 0u);
#endif
#if GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK
    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_FREE_MODE && YawMotorFeedbackReady())
    {
        yaw_feedback_angle_deg =
            YawMotorRelativeRawDegToCtrlDeg(CurrentYawMotorRelativeRawDeg());
        yaw_feedback_speed_deg_s =
            YawMotorRelativeRawDegToCtrlDeg(yaw_motor->measure.speed_aps);
        yaw_target_speed_deg_s =
            YawMotorRelativeRawDegToCtrlDeg(yaw_motor->motor_controller.speed_PID.Ref);
    }
#endif
    gimbal_feedback_data.yaw_target_angle = yaw_target_angle_deg;
    gimbal_feedback_data.yaw_actual_angle = yaw_feedback_angle_deg;
    gimbal_feedback_data.yaw_target_speed = yaw_target_speed_deg_s;
    gimbal_feedback_data.yaw_actual_speed = yaw_feedback_speed_deg_s;
    gimbal_feedback_data.pitch_target_angle = gimbal_cmd_recv.pitch;
    gimbal_feedback_data.pitch_actual_angle = pitch_feedback_deg;
#if GIMBAL_PITCH_MOTOR_BACKEND == GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
    gimbal_feedback_data.pitch_target_speed = pitch_target_speed_deg_s;
#else
    gimbal_feedback_data.pitch_target_speed =
        PitchPidSpeedRefToCtrlDegS(pitch_motor->motor_controller.speed_PID.Ref);
#endif
    gimbal_feedback_data.pitch_actual_speed = pitch_feedback_deg_s;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
