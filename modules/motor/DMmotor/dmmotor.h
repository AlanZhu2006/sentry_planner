#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

typedef enum
{
    DM_MOTOR_CONTROL_MIT = 1,
    DM_MOTOR_CONTROL_SPEED = 3,
} DMMotor_Control_Mode_e;

typedef enum
{
    DM_MOTOR_STATE_DISABLED = 0x0,
    DM_MOTOR_STATE_ENABLED = 0x1,
} DMMotor_Feedback_State_e;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    DMMotor_Control_Mode_e control_mode;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    uint32_t tx_ok_count;
    uint32_t tx_fail_count;
    uint32_t rx_count;
    uint32_t last_rx_ms;
    uint32_t bootstrap_last_action_ms;
    uint32_t bootstrap_not_before_ms;
    uint32_t bootstrap_rx_count;
    uint8_t dm_auto_zero_on_boot;
    uint8_t dm_auto_zero_done;
    uint8_t bootstrap_state;
    uint8_t ever_online;
    uint8_t offline_reported;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

uint8_t DMMotorIsOnline(const DMMotorInstance *motor);

uint8_t DMMotorIsEnabled(const DMMotorInstance *motor);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorSetControlMode(DMMotorInstance *motor, DMMotor_Control_Mode_e mode);
void DMMotorLogDebug(DMMotorInstance *motor);
void DMMotorControlInit();
#endif // !DMMOTOR
