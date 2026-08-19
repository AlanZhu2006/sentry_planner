#include "damiao.h"

#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "dmmotor.h"
#include "led.h"
#include "robot_def.h"

#if defined(ROBOT_TYPE_damiao)

static DMMotorInstance *damiao_motor;
static uint32_t damiao_last_log_ms;
static uint8_t damiao_last_online = 0xFFu;
static uint8_t damiao_last_enabled = 0xFFu;

static uint32_t DamiaoNowMs(void)
{
    return (uint32_t)DWT_GetTimeline_ms();
}

static uint8_t DamiaoBusNumber(FDCAN_HandleTypeDef *hcan)
{
    return hcan == &hfdcan1 ? 1u : 2u;
}

static void DamiaoLogState(uint8_t online, uint8_t enabled)
{
    if (damiao_motor == NULL)
        return;

    LOGINFO("[damiao] online=%d enabled=%d bus=%d tx=0x%x rx=0x%x ref=%d pos=%d vel=%d tq=%d temp=%d/%d",
            (int)online,
            (int)enabled,
            (int)DamiaoBusNumber(damiao_motor->motor_can_instace->can_handle),
            (unsigned int)damiao_motor->motor_can_instace->tx_id,
            (unsigned int)damiao_motor->motor_can_instace->rx_id,
            (int)(DAMIAO_TARGET_SPEED_RAD_S * 1000.0f),
            (int)(damiao_motor->measure.position * 1000.0f),
            (int)(damiao_motor->measure.velocity * 1000.0f),
            (int)(damiao_motor->measure.torque * 1000.0f),
            (int)damiao_motor->measure.T_Mos,
            (int)damiao_motor->measure.T_Rotor);
}

void DamiaoInit(void)
{
    Motor_Init_Config_s motor_config = {
        .can_init_config = {
            .can_handle = &DAMIAO_MOTOR_CAN_BUS,
            .tx_id = DAMIAO_MOTOR_ID,
            .rx_id = DAMIAO_MOTOR_MASTER_ID,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = DAMIAO_MOTOR_REVERSE,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = MOTOR_TYPE_NONE,
        .dm_auto_zero_on_boot = 0u,
    };

    CANSetAutoRetransmission(&DAMIAO_MOTOR_CAN_BUS, 1u);
    CANSetMode(&DAMIAO_MOTOR_CAN_BUS, FDCAN_MODE_NORMAL);

    damiao_motor = DMMotorInit(&motor_config);
    if (damiao_motor == NULL)
    {
        LOGERROR("[damiao] init failed");
        return;
    }

    DMMotorSetControlMode(damiao_motor, DM_MOTOR_CONTROL_SPEED);
    DMMotorSetRef(damiao_motor, DAMIAO_TARGET_SPEED_RAD_S);

    LOGINFO("[damiao] init bus=%d tx=0x%x rx=0x%x mode=speed ref=%d",
            (int)DamiaoBusNumber(damiao_motor->motor_can_instace->can_handle),
            (unsigned int)damiao_motor->motor_can_instace->tx_id,
            (unsigned int)damiao_motor->motor_can_instace->rx_id,
            (int)(DAMIAO_TARGET_SPEED_RAD_S * 1000.0f));
}

void DamiaoTask(void)
{
    uint32_t now_ms;
    uint8_t online;
    uint8_t enabled;

    if (damiao_motor == NULL)
    {
        LEDSetStatus(LED_STATUS_RED_ON);
        return;
    }

    now_ms = DamiaoNowMs();
#if DAMIAO_ENABLE_ONLY_TEST
    DMMotorSetRef(damiao_motor, 0.0f);
#else
    DMMotorSetRef(damiao_motor, DAMIAO_TARGET_SPEED_RAD_S);
#endif
    online = DMMotorIsOnline(damiao_motor);
    enabled = DMMotorIsEnabled(damiao_motor);

    if (enabled)
        LEDSetStatus(LED_STATUS_GREEN_ON);
    else
        LEDSetStatus(LED_STATUS_RED_ON);

    if (online != damiao_last_online)
    {
        damiao_last_online = online;
        if (online)
            LOGINFO("[damiao] motor online");
        else
            LOGWARNING("[damiao] motor offline");
    }

    if (enabled != damiao_last_enabled)
    {
        damiao_last_enabled = enabled;
        if (enabled)
            LOGINFO("[damiao] motor enabled");
        else
            LOGINFO("[damiao] motor disabled");
    }

    if ((now_ms - damiao_last_log_ms) < DAMIAO_LOG_PERIOD_MS)
        return;

    damiao_last_log_ms = now_ms;
    DamiaoLogState(online, enabled);
    DMMotorLogDebug(damiao_motor);
}

#else

void DamiaoInit(void) {}

void DamiaoTask(void) {}

#endif
