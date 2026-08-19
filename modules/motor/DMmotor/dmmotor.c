#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

typedef enum
{
    DM_BOOTSTRAP_IDLE = 0,
    DM_BOOTSTRAP_WRITE_MODE,
    DM_BOOTSTRAP_CLEAR_ERROR,
    DM_BOOTSTRAP_ENABLE,
    DM_BOOTSTRAP_WAIT_ONLINE,
    DM_BOOTSTRAP_ZERO_POSITION,
} DMMotorBootstrapState_e;

#define DM_BOOTSTRAP_STEP_INTERVAL_MS 5u
#define DM_BOOTSTRAP_RETRY_INTERVAL_MS 50u

static const char *DMMotorCanLecString(uint32_t lec)
{
    switch (lec & 0x7u)
    {
    case 0u:
        return "none";
    case 1u:
        return "stuff";
    case 2u:
        return "form";
    case 3u:
        return "ack";
    case 4u:
        return "bit_recessive";
    case 5u:
        return "bit_dominant";
    case 6u:
        return "crc";
    case 7u:
    default:
        return "software";
    }
}

static const char *DMMotorCanStateString(uint32_t state)
{
    switch (state)
    {
    case HAL_FDCAN_STATE_RESET:
        return "reset";
    case HAL_FDCAN_STATE_READY:
        return "ready";
    case HAL_FDCAN_STATE_BUSY:
        return "busy";
    case HAL_FDCAN_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

static const char *DMMotorFeedbackStateString(uint8_t state)
{
    switch (state)
    {
    case 0x0u:
        return "disabled";
    case 0x1u:
        return "enabled";
    case 0x8u:
        return "over_voltage";
    case 0x9u:
        return "under_voltage";
    case 0xAu:
        return "over_current";
    case 0xBu:
        return "mos_over_temp";
    case 0xCu:
        return "coil_over_temp";
    case 0xDu:
        return "communication_lost";
    case 0xEu:
        return "overload";
    default:
        return "unknown";
    }
}

static uint32_t DMMotorModeOffset(DMMotor_Control_Mode_e mode)
{
    switch (mode)
    {
    case DM_MOTOR_CONTROL_SPEED:
        return 0x200u;
    case DM_MOTOR_CONTROL_MIT:
    default:
        return 0x000u;
    }
}

static void DMMotorSendRawFrame(DMMotorInstance *motor, uint32_t std_id, uint8_t dlc)
{
    uint32_t old_std_id = motor->motor_can_instace->txconf.Identifier;
    uint32_t old_dlc = motor->motor_can_instace->txconf.DataLength;
    uint8_t ok;

    motor->motor_can_instace->txconf.Identifier = std_id;
    motor->motor_can_instace->txconf.DataLength = dlc;
    ok = CANTransmit(motor->motor_can_instace, 1);
    motor->motor_can_instace->txconf.Identifier = old_std_id;
    motor->motor_can_instace->txconf.DataLength = old_dlc;

    if (ok)
        motor->tx_ok_count++;
    else
        motor->tx_fail_count++;
}

static void DMMotorWriteRegister(DMMotorInstance *motor, uint8_t reg, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    uint16_t id = (uint16_t)motor->motor_can_instace->tx_id;
    motor->motor_can_instace->tx_buff[0] = (uint8_t)(id & 0xFFu);
    motor->motor_can_instace->tx_buff[1] = (uint8_t)((id >> 8) & 0x07u);
    motor->motor_can_instace->tx_buff[2] = 0x55u;
    motor->motor_can_instace->tx_buff[3] = reg;
    motor->motor_can_instace->tx_buff[4] = d0;
    motor->motor_can_instace->tx_buff[5] = d1;
    motor->motor_can_instace->tx_buff[6] = d2;
    motor->motor_can_instace->tx_buff[7] = d3;
    DMMotorSendRawFrame(motor, 0x7FFu, 8u);
}

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    // Official Damiao control commands (enable/disable/zero/clear error) are sent to plain CAN ID,
    // while continuous control frames use mode-specific offsets such as 0x200 + id for speed mode.
    DMMotorSendRawFrame(motor, motor->motor_can_instace->tx_id, 8u);
}

static void DMMotorStartBootstrap(DMMotorInstance *motor)
{
    uint32_t now_ms;

    if (motor == NULL)
        return;

    now_ms = (uint32_t)DWT_GetTimeline_ms();
    motor->bootstrap_state = DM_BOOTSTRAP_WRITE_MODE;
    motor->bootstrap_last_action_ms = 0u;
    motor->bootstrap_not_before_ms = now_ms;
    motor->bootstrap_rx_count = motor->rx_count;
}

static void DMMotorBootstrapStep(DMMotorInstance *motor)
{
    uint32_t now_ms;

    if (motor == NULL || motor->motor_can_instace == NULL)
        return;

    if (motor->bootstrap_state == DM_BOOTSTRAP_IDLE)
        return;

    now_ms = (uint32_t)DWT_GetTimeline_ms();

    if (now_ms < motor->bootstrap_not_before_ms)
        return;

    if (motor->bootstrap_state == DM_BOOTSTRAP_WAIT_ONLINE)
    {
        if (motor->rx_count > motor->bootstrap_rx_count && DMMotorIsEnabled(motor))
        {
            if (motor->dm_auto_zero_on_boot && !motor->dm_auto_zero_done)
                motor->bootstrap_state = DM_BOOTSTRAP_ZERO_POSITION;
            else
                motor->bootstrap_state = DM_BOOTSTRAP_IDLE;
            motor->bootstrap_last_action_ms = 0u;
            motor->bootstrap_not_before_ms = 0u;
            return;
        }

        if (motor->bootstrap_last_action_ms != 0u &&
            (now_ms - motor->bootstrap_last_action_ms) < DM_BOOTSTRAP_RETRY_INTERVAL_MS)
            return;

        motor->bootstrap_state = DM_BOOTSTRAP_WRITE_MODE;
        motor->bootstrap_last_action_ms = 0u;
        motor->bootstrap_not_before_ms = now_ms;
        motor->bootstrap_rx_count = motor->rx_count;
    }

    if (motor->bootstrap_last_action_ms != 0u &&
        (now_ms - motor->bootstrap_last_action_ms) < DM_BOOTSTRAP_STEP_INTERVAL_MS)
        return;

    switch ((DMMotorBootstrapState_e)motor->bootstrap_state)
    {
    case DM_BOOTSTRAP_WRITE_MODE:
        DMMotorWriteRegister(motor, 10u, (uint8_t)motor->control_mode, 0u, 0u, 0u);
        motor->bootstrap_state = DM_BOOTSTRAP_CLEAR_ERROR;
        break;
    case DM_BOOTSTRAP_CLEAR_ERROR:
        DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
        motor->bootstrap_state = DM_BOOTSTRAP_ENABLE;
        break;
    case DM_BOOTSTRAP_ENABLE:
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
        motor->bootstrap_state = DM_BOOTSTRAP_WAIT_ONLINE;
        break;
    case DM_BOOTSTRAP_ZERO_POSITION:
        DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
        motor->dm_auto_zero_done = 1u;
        motor->bootstrap_state = DM_BOOTSTRAP_IDLE;
        motor->bootstrap_not_before_ms = 0u;
        break;
    case DM_BOOTSTRAP_WAIT_ONLINE:
    case DM_BOOTSTRAP_IDLE:
    default:
        break;
    }

    motor->bootstrap_last_action_ms = now_ms;
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    measure->state = (uint8_t)(rxbuff[0] >> 4);
    measure->id = (uint8_t)(rxbuff[0] & 0x0Fu);
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    motor->rx_count++;
    motor->last_rx_ms = (uint32_t)DWT_GetTimeline_ms();
    motor->ever_online = 1u;
    motor->offline_reported = 0u;
}

uint8_t DMMotorIsOnline(const DMMotorInstance *motor)
{
    if (motor == NULL || motor->motor_daemon == NULL)
        return 0u;

    return (uint8_t)(motor->rx_count > 0u && DaemonIsOnline(motor->motor_daemon));
}

uint8_t DMMotorIsEnabled(const DMMotorInstance *motor)
{
    return (uint8_t)(DMMotorIsOnline(motor) &&
                     motor->measure.state == DM_MOTOR_STATE_ENABLED);
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instace->can_handle == &hfdcan1 ? 1u : 2u;

    if (motor->stop_flag == MOTOR_STOP)
    {
        motor->offline_reported = 1u;
        return;
    }

    if (motor->ever_online && !motor->offline_reported)
    {
        LOGWARNING("[dmmotor] motor lost, can bus [%d], tx id [0x%x]",
                   can_bus,
                   motor->motor_can_instace->tx_id);
        DMMotorLogDebug(motor);
    }

    motor->offline_reported = 1u;
    if (motor->bootstrap_state == DM_BOOTSTRAP_IDLE)
        DMMotorStartBootstrap(motor);
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    if (motor == NULL)
        return;

    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    motor->dm_auto_zero_done = 1u;
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    if (idx >= DM_MOTOR_CNT)
    {
        LOGERROR("[dmmotor] instance overflow, max motor count = %d", DM_MOTOR_CNT);
        return NULL;
    }

    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    motor->control_mode = DM_MOTOR_CONTROL_MIT;
    motor->dm_auto_zero_on_boot = config->dm_auto_zero_on_boot ? 1u : 0u;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);
    CANSetAutoBusOff(config->can_init_config.can_handle, 1u);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorStartBootstrap(motor);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    if (motor == NULL)
        return;

    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    if (motor == NULL)
        return;

    if (motor->stop_flag == MOTOR_STOP && motor->bootstrap_state == DM_BOOTSTRAP_IDLE)
        DMMotorStartBootstrap(motor);

    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)
{
    if (motor == NULL)
        return;

    if (motor->stop_flag == MOTOR_STOP)
        return;

    motor->pid_ref = 0.0f;
    motor->stop_flag = MOTOR_STOP;
    motor->bootstrap_state = DM_BOOTSTRAP_IDLE;
    motor->bootstrap_not_before_ms = 0u;
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorSetControlMode(DMMotorInstance *motor, DMMotor_Control_Mode_e mode)
{
    if (motor == NULL)
        return;

    motor->control_mode = mode;
    if (motor->stop_flag != MOTOR_STOP)
        DMMotorStartBootstrap(motor);
}

void DMMotorLogDebug(DMMotorInstance *motor)
{
    CAN_Debug_Info_s can_debug = {0};
    uint16_t can_bus;
    uint32_t ecr;
    uint32_t psr;
    uint32_t lec;
    uint32_t tec;
    uint32_t rec;

    if (motor == NULL || motor->motor_can_instace == NULL)
        return;

    CANGetDebugInfo(motor->motor_can_instace->can_handle, &can_debug);
    can_bus = motor->motor_can_instace->can_handle == &hfdcan1 ? 1u : 2u;
    ecr = can_debug.esr;
    psr = can_debug.msr;
    lec = psr & 0x7u;
    tec = ecr & 0xFFu;
    rec = (ecr >> 8) & 0x7Fu;

    LOGINFO("[dmmotor] dbg bus=%d tx=0x%x rx=0x%x mode=%d online=%d enabled=%d state=%s(%d) fb_id=0x%x boot=%d txok=%d txfail=%d rx=%d last_rx_ms=%d free=%d",
            can_bus,
            motor->motor_can_instace->tx_id,
            motor->motor_can_instace->rx_id,
            (int)motor->control_mode,
            (int)DMMotorIsOnline(motor),
            (int)DMMotorIsEnabled(motor),
            DMMotorFeedbackStateString(motor->measure.state),
            (int)motor->measure.state,
            (unsigned int)motor->measure.id,
            (int)motor->bootstrap_state,
            (int)motor->tx_ok_count,
            (int)motor->tx_fail_count,
            (int)motor->rx_count,
            (int)motor->last_rx_ms,
            (int)can_debug.tx_free_level);
    LOGINFO("[dmmotor] meas pos=%d vel=%d tq=%d temp=%d/%d",
            (int)(motor->measure.position * 1000.0f),
            (int)(motor->measure.velocity * 1000.0f),
            (int)(motor->measure.torque * 1000.0f),
            (int)motor->measure.T_Mos,
            (int)motor->measure.T_Rotor);
    LOGINFO("[dmmotor] can esr=0x%x tsr=0x%x msr=0x%x herr=0x%x state=0x%x",
            (unsigned int)can_debug.esr,
            (unsigned int)can_debug.tsr,
            (unsigned int)can_debug.msr,
            (unsigned int)can_debug.hal_error,
            (unsigned int)can_debug.state);
    LOGINFO("[dmmotor] err ewgf=%d epvf=%d boff=%d lec=%s tec=%d rec=%d state=%s",
            (int)((psr >> 6) & 0x1u),
            (int)((psr >> 5) & 0x1u),
            (int)((psr >> 7) & 0x1u),
            DMMotorCanLecString(lec),
            (int)tec,
            (int)rec,
            DMMotorCanStateString(can_debug.state));
}


//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument)
{
    float  pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    while (1)
    {
        pid_ref = motor->pid_ref;
        DMMotorBootstrapStep(motor);
        
        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;

        if (motor->stop_flag == MOTOR_STOP)
        {
            osDelay(2);
            continue;
        }

        if (motor->bootstrap_state != DM_BOOTSTRAP_IDLE)
        {
            osDelay(2);
            continue;
        }

        if (!DMMotorIsEnabled(motor))
        {
            DMMotorStartBootstrap(motor);
            osDelay(2);
            continue;
        }

        if (motor->control_mode == DM_MOTOR_CONTROL_SPEED)
        {
            uint8_t *speed_bytes = (uint8_t *)&set;
            motor->motor_can_instace->tx_buff[0] = speed_bytes[0];
            motor->motor_can_instace->tx_buff[1] = speed_bytes[1];
            motor->motor_can_instace->tx_buff[2] = speed_bytes[2];
            motor->motor_can_instace->tx_buff[3] = speed_bytes[3];
            motor->motor_can_instace->tx_buff[4] = 0u;
            motor->motor_can_instace->tx_buff[5] = 0u;
            motor->motor_can_instace->tx_buff[6] = 0u;
            motor->motor_can_instace->tx_buff[7] = 0u;
            DMMotorSendRawFrame(motor,
                                motor->motor_can_instace->tx_id + DMMotorModeOffset(motor->control_mode),
                                8u);
        }
        else
        {
            LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(set, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;

            motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
            motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
            motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
            motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
            motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
            motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
            motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
            motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
            DMMotorSendRawFrame(motor,
                                motor->motor_can_instace->tx_id + DMMotorModeOffset(motor->control_mode),
                                8u);
        }

        osDelay(2);
    }
}
void DMMotorControlInit()
{
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        osThreadDef(dmtask, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dmtask), dm_motor_instance[i]);
    }
}
