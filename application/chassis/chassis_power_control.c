#include "chassis_power_control.h"

#include "general_def.h"

#include <math.h>
#include <string.h>

#if defined(CHASSIS_POWER_CONTROL_ENABLED)

#define CHASSIS_POWER_FLAG_ACTIVE 0x01u
#define CHASSIS_POWER_FLAG_BURST 0x02u
#define CHASSIS_POWER_FLAG_REFEREE_ONLINE 0x04u
#define CHASSIS_POWER_FLAG_LIMITED 0x08u
#define CHASSIS_POWER_FLAG_FALLBACK 0x10u

static DJIMotorInstance *power_motors[4];
static Chassis_Ctrl_Cmd_s *power_cmd;
static referee_info_t *power_referee;
static uint8_t power_control_inited;
static PIDInstance buffer_pid;
static PID_Init_Config_s buffer_pid_init_config = {
    .Kp = CHASSIS_POWER_BUFFER_PID_KP,
    .Ki = CHASSIS_POWER_BUFFER_PID_KI,
    .Kd = CHASSIS_POWER_BUFFER_PID_KD,
    .MaxOut = CHASSIS_POWER_BUFFER_PID_MAX_OUT,
    .IntegralLimit = CHASSIS_POWER_BUFFER_PID_INT_LIMIT,
    .Improve = PID_IMPROVE_NONE,
};
static ChassisPowerControlTelemetry_s power_telemetry = {
    .power_scale = 1.0f,
};

static float PowerClamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static float PowerRotorRpm(const DJIMotorInstance *motor)
{
    return motor->measure.speed_aps / RPM_2_ANGLE_PER_SEC;
}

static float PowerEstimate(float current_cmd, float motor_rpm)
{
    return CHASSIS_POWER_MODEL_CT * current_cmd * motor_rpm +
           CHASSIS_POWER_MODEL_K_RPM2 * motor_rpm * motor_rpm +
           CHASSIS_POWER_MODEL_K_CMD2 * current_cmd * current_cmd +
           CHASSIS_POWER_MODEL_CONST;
}

static float PowerFallbackScale(float current_cmd, float scale)
{
    return current_cmd * scale;
}

static float PowerSolveCurrent(float current_cmd, float motor_rpm, float target_power)
{
    float coeff_a;
    float coeff_b;
    float coeff_c;
    float discriminant;
    float sqrt_discriminant;
    float root_a;
    float root_b;

    if (target_power <= 0.0f)
        return 0.0f;

    coeff_a = CHASSIS_POWER_MODEL_K_CMD2;
    coeff_b = CHASSIS_POWER_MODEL_CT * motor_rpm;
    coeff_c = CHASSIS_POWER_MODEL_K_RPM2 * motor_rpm * motor_rpm +
              CHASSIS_POWER_MODEL_CONST - target_power;
    discriminant = coeff_b * coeff_b - 4.0f * coeff_a * coeff_c;

    if (discriminant < 0.0f || coeff_a <= 0.0f)
        return NAN;

    sqrt_discriminant = sqrtf(discriminant);
    root_a = (-coeff_b + sqrt_discriminant) / (2.0f * coeff_a);
    root_b = (-coeff_b - sqrt_discriminant) / (2.0f * coeff_a);

    if (current_cmd > 0.0f)
    {
        if (root_a >= 0.0f && root_b >= 0.0f)
            return fabsf(root_a - current_cmd) < fabsf(root_b - current_cmd) ? root_a : root_b;
        if (root_a >= 0.0f)
            return root_a;
        if (root_b >= 0.0f)
            return root_b;
        return 0.0f;
    }

    if (current_cmd < 0.0f)
    {
        if (root_a <= 0.0f && root_b <= 0.0f)
            return fabsf(root_a - current_cmd) < fabsf(root_b - current_cmd) ? root_a : root_b;
        if (root_a <= 0.0f)
            return root_a;
        if (root_b <= 0.0f)
            return root_b;
        return 0.0f;
    }

    return 0.0f;
}

static void PowerTelemetryReset(void)
{
    memset(&power_telemetry, 0, sizeof(power_telemetry));
    power_telemetry.power_scale = 1.0f;
}

static void PowerControllerReset(void)
{
    PIDInit(&buffer_pid, &buffer_pid_init_config);
    PowerTelemetryReset();
}

static void PowerBufferPidReset(void)
{
    PIDInit(&buffer_pid, &buffer_pid_init_config);
}

void ChassisPowerControlInit(const ChassisPowerControl_Init_Config_s *config)
{
    if (config == NULL || config->motor_lf == NULL || config->motor_rf == NULL ||
        config->motor_lb == NULL || config->motor_rb == NULL || config->chassis_cmd == NULL)
        return;

    power_motors[0] = config->motor_lf;
    power_motors[1] = config->motor_rf;
    power_motors[2] = config->motor_lb;
    power_motors[3] = config->motor_rb;
    power_cmd = config->chassis_cmd;
    power_referee = config->referee_data;
    power_control_inited = 1u;
    PowerControllerReset();
}

void ChassisPowerControlReset(void)
{
    if (!power_control_inited)
        return;

    PowerControllerReset();
}

ChassisPowerControlTelemetry_s ChassisPowerControlGetTelemetry(void)
{
    return power_telemetry;
}

void ChassisPowerControlCallback(void)
{
    float current_cmd[4];
    float estimated_power[4];
    float motor_rpm[4];
    float total_positive_power = 0.0f;
    float target_buffer = CHASSIS_POWER_BUFFER_TARGET_NORMAL;
    float power_limit = CHASSIS_POWER_REFEREE_OFFLINE_BUDGET_W;
    float power_budget = CHASSIS_POWER_REFEREE_OFFLINE_BUDGET_W;
    float power_scale = 1.0f;
    float buffer_pid_output = 0.0f;
    uint16_t buffer_energy = 0u;
    uint8_t referee_online = 0u;
    uint8_t burst_enabled = 0u;

    if (!power_control_inited || power_cmd == NULL)
        return;

    if (power_cmd->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        PowerControllerReset();
        return;
    }

    PowerTelemetryReset();
    power_telemetry.power_mode_flags |= CHASSIS_POWER_FLAG_ACTIVE;

    if (power_referee != NULL &&
        RefereeIsOnline() &&
        power_referee->GameRobotState.robot_id != 0u &&
        power_referee->GameRobotState.chassis_power_limit != 0u)
    {
        referee_online = 1u;
        power_limit = (float)power_referee->GameRobotState.chassis_power_limit;
        buffer_energy = power_referee->PowerHeatData.buffer_energy;
        burst_enabled = power_cmd->power_burst_enabled;
        target_buffer = burst_enabled ? CHASSIS_POWER_BUFFER_TARGET_BURST :
                                        CHASSIS_POWER_BUFFER_TARGET_NORMAL;
        buffer_pid_output = PIDCalculate(&buffer_pid, (float)buffer_energy, target_buffer);
        power_budget = PowerClamp(power_limit - buffer_pid_output, 0.0f, power_limit);
    }
    else
    {
        PowerBufferPidReset();
        power_budget = CHASSIS_POWER_REFEREE_OFFLINE_BUDGET_W;
        power_telemetry.power_mode_flags |= CHASSIS_POWER_FLAG_FALLBACK;
    }

    power_telemetry.buffer_energy = buffer_energy;
    power_telemetry.power_budget_w = power_budget;
    if (referee_online)
        power_telemetry.power_mode_flags |= CHASSIS_POWER_FLAG_REFEREE_ONLINE;
    if (burst_enabled)
        power_telemetry.power_mode_flags |= CHASSIS_POWER_FLAG_BURST;

    for (size_t i = 0; i < 4; ++i)
    {
        current_cmd[i] = power_motors[i]->motor_controller.pre_current_ref;
        motor_rpm[i] = PowerRotorRpm(power_motors[i]);
        estimated_power[i] = PowerEstimate(current_cmd[i], motor_rpm[i]);
        if (estimated_power[i] > 0.0f)
            total_positive_power += estimated_power[i];
    }

    power_telemetry.estimated_power_w = total_positive_power;

    if (total_positive_power <= 0.0f)
        return;

    if (total_positive_power > power_budget)
    {
        power_scale = PowerClamp(power_budget / total_positive_power, 0.0f, 1.0f);
        power_telemetry.power_mode_flags |= CHASSIS_POWER_FLAG_LIMITED;
    }

    power_telemetry.power_scale = power_scale;

    if (power_scale >= 1.0f)
        return;

    for (size_t i = 0; i < 4; ++i)
    {
        float scaled_power;
        float solved_current;

        if (estimated_power[i] <= 0.0f)
            continue;

        scaled_power = estimated_power[i] * power_scale;
        solved_current = PowerSolveCurrent(current_cmd[i], motor_rpm[i], scaled_power);

        if (!isfinite(solved_current))
            solved_current = PowerFallbackScale(current_cmd[i], power_scale);

        solved_current = PowerClamp(solved_current,
                                    -CHASSIS_POWER_CURRENT_LIMIT,
                                    CHASSIS_POWER_CURRENT_LIMIT);
        power_motors[i]->motor_controller.pre_current_ref = solved_current;
    }
}

#else

void ChassisPowerControlInit(const ChassisPowerControl_Init_Config_s *config)
{
    (void)config;
}

void ChassisPowerControlCallback(void)
{
}

void ChassisPowerControlReset(void)
{
}

ChassisPowerControlTelemetry_s ChassisPowerControlGetTelemetry(void)
{
    ChassisPowerControlTelemetry_s telemetry = {0};
    telemetry.power_scale = 1.0f;
    return telemetry;
}

#endif
