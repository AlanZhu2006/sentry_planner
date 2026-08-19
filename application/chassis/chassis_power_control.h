#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "dji_motor.h"
#include "referee_task.h"
#include "robot_def.h"

typedef struct
{
    DJIMotorInstance *motor_lf;
    DJIMotorInstance *motor_rf;
    DJIMotorInstance *motor_lb;
    DJIMotorInstance *motor_rb;
    Chassis_Ctrl_Cmd_s *chassis_cmd;
    referee_info_t *referee_data;
} ChassisPowerControl_Init_Config_s;

typedef struct
{
    uint16_t buffer_energy;
    float power_budget_w;
    float estimated_power_w;
    float power_scale;
    uint8_t power_mode_flags;
} ChassisPowerControlTelemetry_s;

/**
 * @brief 初始化步兵底盘功率控制
 *
 * @param config 初始化配置
 */
void ChassisPowerControlInit(const ChassisPowerControl_Init_Config_s *config);

/**
 * @brief 供DJI电机控制任务调用的速度环后钩子
 */
void ChassisPowerControlCallback(void);

/**
 * @brief 重置功率控制内部状态
 */
void ChassisPowerControlReset(void);

/**
 * @brief 获取最新功率控制遥测
 *
 * @return ChassisPowerControlTelemetry_s 最新遥测
 */
ChassisPowerControlTelemetry_s ChassisPowerControlGetTelemetry(void);

#endif // CHASSIS_POWER_CONTROL_H
