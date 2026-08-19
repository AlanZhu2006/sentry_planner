/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 机器人配置选择: 基于编译宏ROBOT_TYPE_* */
#include "robot_config_select.h"

// IMU->云台坐标方向映射必须在具体机器人配置中定义,且只能取±1
#if !defined(GYRO2GIMBAL_DIR_YAW) || !defined(GYRO2GIMBAL_DIR_PITCH) || !defined(GYRO2GIMBAL_DIR_ROLL)
#error Missing GYRO2GIMBAL_DIR_* definitions in robot config.
#endif

#if ((GYRO2GIMBAL_DIR_YAW != 1) && (GYRO2GIMBAL_DIR_YAW != -1)) || \
    ((GYRO2GIMBAL_DIR_PITCH != 1) && (GYRO2GIMBAL_DIR_PITCH != -1)) || \
    ((GYRO2GIMBAL_DIR_ROLL != 1) && (GYRO2GIMBAL_DIR_ROLL != -1))
#error GYRO2GIMBAL_DIR_* must be either 1 or -1.
#endif

// yaw 控制坐标固定为 +yaw = 向右。
// 默认值保持 infantry / sentry 现有行为，hero 可按需覆盖。
#ifndef GIMBAL_YAW_CTRL_FROM_IMU_SIGN
#define GIMBAL_YAW_CTRL_FROM_IMU_SIGN (-(GYRO2GIMBAL_DIR_YAW))
#endif

#ifndef GIMBAL_YAW_PID_USE_CTRL_FEEDBACK
#define GIMBAL_YAW_PID_USE_CTRL_FEEDBACK 0
#endif

#ifndef GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN
#define GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN (-(GYRO2GIMBAL_DIR_YAW))
#endif

#if ((GIMBAL_YAW_CTRL_FROM_IMU_SIGN != 1) && (GIMBAL_YAW_CTRL_FROM_IMU_SIGN != -1))
#error GIMBAL_YAW_CTRL_FROM_IMU_SIGN must be either 1 or -1.
#endif

#if ((GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN != 1) && (GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN != -1))
#error GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN must be either 1 or -1.
#endif

#if ((GIMBAL_YAW_PID_USE_CTRL_FEEDBACK != 0) && (GIMBAL_YAW_PID_USE_CTRL_FEEDBACK != 1))
#error GIMBAL_YAW_PID_USE_CTRL_FEEDBACK must be either 0 or 1.
#endif

#ifndef CHASSIS_OFFSET_ANGLE_FORCE_ZERO
#define CHASSIS_OFFSET_ANGLE_FORCE_ZERO 0
#endif

#if ((CHASSIS_OFFSET_ANGLE_FORCE_ZERO != 0) && (CHASSIS_OFFSET_ANGLE_FORCE_ZERO != 1))
#error CHASSIS_OFFSET_ANGLE_FORCE_ZERO must be either 0 or 1.
#endif

#ifndef CHASSIS_MANUAL_INPUT_FORWARD_ON_VX
#define CHASSIS_MANUAL_INPUT_FORWARD_ON_VX 0
#endif

#if ((CHASSIS_MANUAL_INPUT_FORWARD_ON_VX != 0) && (CHASSIS_MANUAL_INPUT_FORWARD_ON_VX != 1))
#error CHASSIS_MANUAL_INPUT_FORWARD_ON_VX must be either 0 or 1.
#endif

#ifndef CHASSIS_RC_VX_INPUT_SIGN
#define CHASSIS_RC_VX_INPUT_SIGN 1
#endif

#ifndef CHASSIS_RC_VY_INPUT_SIGN
#define CHASSIS_RC_VY_INPUT_SIGN 1
#endif

#ifndef CHASSIS_KB_VX_INPUT_SIGN
#define CHASSIS_KB_VX_INPUT_SIGN 1
#endif

#ifndef CHASSIS_KB_VY_INPUT_SIGN
#define CHASSIS_KB_VY_INPUT_SIGN 1
#endif

#if ((CHASSIS_RC_VX_INPUT_SIGN != 1) && (CHASSIS_RC_VX_INPUT_SIGN != -1))
#error CHASSIS_RC_VX_INPUT_SIGN must be either 1 or -1.
#endif

#if ((CHASSIS_RC_VY_INPUT_SIGN != 1) && (CHASSIS_RC_VY_INPUT_SIGN != -1))
#error CHASSIS_RC_VY_INPUT_SIGN must be either 1 or -1.
#endif

#if ((CHASSIS_KB_VX_INPUT_SIGN != 1) && (CHASSIS_KB_VX_INPUT_SIGN != -1))
#error CHASSIS_KB_VX_INPUT_SIGN must be either 1 or -1.
#endif

#if ((CHASSIS_KB_VY_INPUT_SIGN != 1) && (CHASSIS_KB_VY_INPUT_SIGN != -1))
#error CHASSIS_KB_VY_INPUT_SIGN must be either 1 or -1.
#endif

// 人机输入方向映射只定义“操作器输入 -> 云台逻辑坐标”，不参与 IMU/电机/视觉方向
#if !defined(GIMBAL_RC_YAW_INPUT_SIGN) || !defined(GIMBAL_RC_PITCH_INPUT_SIGN) || \
    !defined(GIMBAL_MOUSE_YAW_INPUT_SIGN) || !defined(GIMBAL_MOUSE_PITCH_INPUT_SIGN)
#error Missing GIMBAL_*_INPUT_SIGN definitions in robot config.
#endif

#if ((GIMBAL_RC_YAW_INPUT_SIGN != 1) && (GIMBAL_RC_YAW_INPUT_SIGN != -1)) || \
    ((GIMBAL_RC_PITCH_INPUT_SIGN != 1) && (GIMBAL_RC_PITCH_INPUT_SIGN != -1)) || \
    ((GIMBAL_MOUSE_YAW_INPUT_SIGN != 1) && (GIMBAL_MOUSE_YAW_INPUT_SIGN != -1)) || \
    ((GIMBAL_MOUSE_PITCH_INPUT_SIGN != 1) && (GIMBAL_MOUSE_PITCH_INPUT_SIGN != -1))
#error GIMBAL_*_INPUT_SIGN must be either 1 or -1.
#endif

#ifndef REMOTE_CONTROL_UART_HANDLE
#error Missing REMOTE_CONTROL_UART_HANDLE in robot config.
#endif

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#if (defined(CHASSIS_TYPE_MECANUM) && defined(CHASSIS_TYPE_SWERVE)) || \
    (!defined(CHASSIS_TYPE_MECANUM) && !defined(CHASSIS_TYPE_SWERVE))
#error Conflict chassis type definition! Define only one chassis type.
#endif

#ifndef CHASSIS_FOLLOW_HEAD_DIRECTION
#define CHASSIS_FOLLOW_HEAD_DIRECTION 1
#endif

#if ((CHASSIS_FOLLOW_HEAD_DIRECTION != 1) && (CHASSIS_FOLLOW_HEAD_DIRECTION != -1))
#error CHASSIS_FOLLOW_HEAD_DIRECTION must be either 1 or -1.
#endif

// 视觉通信角度转换常量
#define VISION_RAD_TO_DEG (57.295779513f)
#define VISION_DEG_TO_RAD (0.01745329251f)

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

typedef enum
{
    FIRE_SELECT_SINGLE = 0,
    FIRE_SELECT_TRIPLE,
    FIRE_SELECT_AUTO,
} shoot_fire_select_e;

#ifndef VTM_DEFAULT_FIRE_SELECT
#define VTM_DEFAULT_FIRE_SELECT FIRE_SELECT_SINGLE
#endif

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    uint8_t power_burst_enabled;
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;   // 统一逻辑坐标: +yaw=向右
    float pitch; // 统一逻辑坐标: +pitch=向上
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red
    float motor_target_rpm[4];   // 底盘四轮目标转速,LF RF LB RB
    float motor_rpm[4];          // 底盘四轮转速,LF RF LB RB
    uint8_t motor_online_bitmap; // bit0..bit3: LF RF LB RB
    uint16_t power_buffer_energy;
    float power_budget_w;
    float estimated_power_w;
    float power_scale;
    uint8_t power_mode_flags; // bit0: active, bit1: burst, bit2: referee, bit3: limited, bit4: fallback

} Chassis_Upload_Data_s;


typedef struct
{
    attitude_t gimbal_imu_data; // 原始 IMU 姿态快照(raw IMU frame)
    uint16_t yaw_motor_single_round_angle;
    uint16_t yaw_motor_encoder_raw;   // yaw 原始编码器值(DJI ECD)
    uint16_t pitch_motor_encoder_raw; // pitch 原始编码器值/位置码
    uint8_t motor_online_bitmap; // bit0: yaw motor, bit1: pitch motor
    float yaw_target_angle;   // 统一逻辑坐标: +yaw=向右
    float yaw_actual_angle;   // 统一逻辑坐标: +yaw=向右
    float yaw_target_speed;   // 统一逻辑坐标: +yaw=向右, unit: deg/s
    float yaw_actual_speed;   // 统一逻辑坐标: +yaw=向右, unit: deg/s
    float pitch_target_angle; // 统一逻辑坐标: +pitch=向上
    float pitch_actual_angle; // 统一逻辑坐标: +pitch=向上
    float pitch_target_speed; // 统一逻辑坐标: +pitch=向上, unit: deg/s
    float pitch_actual_speed; // 统一逻辑坐标: +pitch=向上, unit: deg/s
} Gimbal_Upload_Data_s;

typedef struct
{
    uint8_t motor_online_bitmap; // bit0: loader, bit1: friction_l, bit2: friction_r
    float loader_speed_aps;
    float friction_l_speed_aps;
    float friction_r_speed_aps;
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H
