#pragma once
#ifndef ROBOT_HERO_CONFIG_H
#define ROBOT_HERO_CONFIG_H

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义!
 */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#define REMOTE_CONTROL_UART_HANDLE huart1 // VT03/VT13 图传遥控
// #define REMOTE_CONTROL_UART_HANDLE huart3 // DBUS 遥控

#define CHASSIS_TYPE_MECANUM
#define CHASSIS_POWER_CONTROL_ENABLED

#define CHASSIS_FOLLOW_HEAD_DIRECTION -1

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾
 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD                                                  \
  800 // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
       // (Calibrated 2025-12-25)
#define YAW_ECD_GREATER_THAN_4096                                              \
  0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD                                                      \
  3370 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改 (Calibrated
       // 2025-12-25)
#define PITCH_MAX_ANGLE                                                        \
  15.0f // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度) -
        // 向上抬头
#define PITCH_MIN_ANGLE                                                        \
  -20.0f // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度) -
         // 向下低头
#define YAW_RIGHT_LIMIT_DELTA_ECD                                              \
  2000 // 以 YAW_CHASSIS_ALIGN_ECD 为中心, +yaw=向右允许的最大编码器偏移
#define YAW_LEFT_LIMIT_DELTA_ECD                                               \
  2000 // 以 YAW_CHASSIS_ALIGN_ECD 为中心, -yaw=向左允许的最大编码器偏移
#define GYRO2GIMBAL_DIR_YAW                                                    \
  -1 // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH                                                  \
  -1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GIMBAL_PITCH_FEEDBACK_FROM_IMU_SIGN -1
#define GYRO2GIMBAL_DIR_ROLL                                                   \
  1 // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
// hero 保持 4df1489 的 yaw 语义:
// 对外 ctrl 坐标 +yaw=向右, 但 yaw PID 继续直接工作在 ctrl 反馈域。
#define GIMBAL_YAW_CTRL_FROM_IMU_SIGN -1
#define GIMBAL_YAW_PID_USE_CTRL_FEEDBACK 1
#define GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN 1
// hero 手动控制改为 body-lock: 底盘转动时云台跟随底盘，yaw 相对底盘保持静止。
#define MANUAL_GIMBAL_BODY_LOCK_ENABLE 1
#define GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK 1
// hero 底盘平移方向跟随 yaw 轴/头部朝向，但保持原有 hero 输入坐标基与底盘投影矩阵。
#define CHASSIS_OFFSET_ANGLE_FORCE_ZERO 0
#define CHASSIS_MANUAL_INPUT_FORWARD_ON_VX 0
// 人机输入方向配置,只影响操作器输入映射,不影响IMU/电机/视觉方向
#define GIMBAL_RC_YAW_INPUT_SIGN 1
#define GIMBAL_RC_PITCH_INPUT_SIGN -1
#define GIMBAL_MOUSE_YAW_INPUT_SIGN 1
#define GIMBAL_MOUSE_PITCH_INPUT_SIGN -1
// 发射参数
#define VTM_DEFAULT_FIRE_SELECT FIRE_SELECT_AUTO
#define ONE_BULLET_DELTA_ANGLE                                                 \
  36 // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER                                                 \
  19.0f // hero 拨盘使用 M3508, 减速比为 19.0f
#define SHOOT_LOADER_MOTOR_TYPE M3508
#define NUM_PER_CIRCLE 10 // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 362  // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 255 // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X                                                 \
  0 // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y                                                 \
  0 // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60 // 轮子半径
#define REDUCTION_RATIO_WHEEL                                                  \
  19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

// 电机正反转配置 (MOTOR_DIRECTION_NORMAL=0, MOTOR_DIRECTION_REVERSE=1)
#define GIMBAL_YAW_MOTOR_REVERSE MOTOR_DIRECTION_REVERSE
#define GIMBAL_PITCH_MOTOR_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_MOTOR_LF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_MOTOR_RF_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_MOTOR_LB_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_MOTOR_RB_REVERSE MOTOR_DIRECTION_REVERSE
#define SHOOT_FRICTION_L_REVERSE MOTOR_DIRECTION_REVERSE
#define SHOOT_FRICTION_R_REVERSE MOTOR_DIRECTION_REVERSE
#define SHOOT_LOADER_REVERSE MOTOR_DIRECTION_NORMAL

// 底盘电机CAN配置
#define CHASSIS_CAN_BUS hfdcan1
#define CHASSIS_MOTOR_LF_ID 2 // 左前
#define CHASSIS_MOTOR_RF_ID 1 // 右前
#define CHASSIS_MOTOR_LB_ID 4 // 左后
#define CHASSIS_MOTOR_RB_ID 3 // 右后

// 云台电机CAN配置
#define GIMBAL_YAW_CAN_BUS hfdcan1
#define GIMBAL_YAW_MOTOR_ID 7
#define GIMBAL_PITCH_MOTOR_BACKEND GIMBAL_PITCH_MOTOR_BACKEND_DAMIAO
#define GIMBAL_PITCH_CAN_BUS hfdcan2
#define GIMBAL_PITCH_MOTOR_ID 6
#define GIMBAL_PITCH_MOTOR_MASTER_ID 0u

// 发射机构电机CAN配置
#define SHOOT_CAN_BUS hfdcan2
#define SHOOT_FRICTION_CAN_BUS hfdcan2
#define SHOOT_LOADER_CAN_BUS hfdcan2
#define SHOOT_LOADER_ID 5     // 拨盘/供弹电机 (CAN RX 0x205)
#define SHOOT_FRICTION_L_ID 7 // 左摩擦轮 (CAN RX 0x207)
#define SHOOT_FRICTION_R_ID 8 // 右摩擦轮 (CAN RX 0x208)

// 底盘运动参数
#define CHASSIS_ROTATE_SPEED 2000.0f // 小陀螺模式旋转速度
#define CHASSIS_RC_MOVE_RATIO_X 30.0f // 遥控器模式底盘前后移动速度系数
#define CHASSIS_RC_MOVE_RATIO_Y 30.0f // 遥控器模式底盘左右移动速度系数
#define CHASSIS_RC_VX_INPUT_SIGN -1   // hero 遥控器前后输入与底盘前后逻辑坐标反向
#define CHASSIS_RC_VY_INPUT_SIGN -1   // hero 遥控器左右输入与底盘左右逻辑坐标反向
#define CHASSIS_KB_VX_INPUT_SIGN -1   // hero 键盘 W/S 输入与底盘前后逻辑坐标反向
#define CHASSIS_KB_VY_INPUT_SIGN -1   // hero 键盘 A/D 输入与底盘左右逻辑坐标反向
#define CHASSIS_KB_MOVE_SPEED_X 300.0f // 键鼠模式底盘前后移动速度
#define CHASSIS_KB_MOVE_SPEED_Y 300.0f // 键鼠模式底盘左右移动速度
#define CHASSIS_MECANUM_LATERAL_SIGN -1.0f

// Initial hero chassis power-control parameters.
#define CHASSIS_POWER_MODEL_CT 1.99688994e-6f
#define CHASSIS_POWER_MODEL_K_RPM2 1.23e-07f
#define CHASSIS_POWER_MODEL_K_CMD2 1.453e-07f
#define CHASSIS_POWER_MODEL_CONST 4.081f
#define CHASSIS_POWER_BUFFER_TARGET_NORMAL 30.0f
#define CHASSIS_POWER_BUFFER_TARGET_BURST 5.0f
#define CHASSIS_POWER_REFEREE_OFFLINE_BUDGET_W 40.0f
#define CHASSIS_POWER_CURRENT_LIMIT 16384.0f
#define CHASSIS_POWER_BUFFER_PID_KP 0.8f
#define CHASSIS_POWER_BUFFER_PID_KI 0.0f
#define CHASSIS_POWER_BUFFER_PID_KD 0.0f
#define CHASSIS_POWER_BUFFER_PID_INT_LIMIT 30.0f
#define CHASSIS_POWER_BUFFER_PID_MAX_OUT 30.0f

// PID参数 - 底盘
#define CHASSIS_SPEED_PID_KP 7.5f
#define CHASSIS_SPEED_PID_KI 0.25f
#define CHASSIS_SPEED_PID_KD 0.0f
#define CHASSIS_SPEED_PID_INT_LIMIT 2000.0f
#define CHASSIS_SPEED_PID_MAX_OUT 12000.0f

#define CHASSIS_CURRENT_PID_KP 0.35f
#define CHASSIS_CURRENT_PID_KI 0.0f
#define CHASSIS_CURRENT_PID_KD 0.0f
#define CHASSIS_CURRENT_PID_INT_LIMIT 3000.0f
#define CHASSIS_CURRENT_PID_MAX_OUT 15000.0f

// PID参数 - 云台
#define GIMBAL_YAW_ANGLE_PID_KP 16.0f
#define GIMBAL_YAW_ANGLE_PID_KI 1.0f
#define GIMBAL_YAW_ANGLE_PID_KD 0.0f
#define GIMBAL_YAW_ANGLE_PID_DEADBAND 0.008f
#define GIMBAL_YAW_ANGLE_PID_INT_LIMIT 100.0f
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT 330.0f
#define GIMBAL_YAW_SPEED_PID_KP 80.0f
#define GIMBAL_YAW_SPEED_PID_KI 1.0f
#define GIMBAL_YAW_SPEED_PID_KD 0.0f
#define GIMBAL_YAW_SPEED_PID_INT_LIMIT 3000.0f
#define GIMBAL_YAW_SPEED_PID_MAX_OUT 25000.0f

#define GIMBAL_PITCH_ANGLE_PID_KP 4.0f
#define GIMBAL_PITCH_ANGLE_PID_KI 0.1f
#define GIMBAL_PITCH_ANGLE_PID_KD 0.02f
#define GIMBAL_PITCH_ANGLE_PID_DEADBAND 0.03f
#define GIMBAL_PITCH_ANGLE_PID_INT_LIMIT 100.0f
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT 300.0f
#define GIMBAL_PITCH_DAMIAO_SPEED_MAX_DEG_S 0.0f

#define GIMBAL_PITCH_SPEED_PID_KP 200.0f
#define GIMBAL_PITCH_SPEED_PID_KI 20.0f
#define GIMBAL_PITCH_SPEED_PID_KD 2.0f
#define GIMBAL_PITCH_SPEED_PID_INT_LIMIT 1200.0f
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT 20000.0f

// PID参数 - 发射
#define SHOOT_FRICTION_SPEED_PID_KP 7.0f
#define SHOOT_FRICTION_SPEED_PID_KI 0.03f
#define SHOOT_FRICTION_SPEED_PID_KD 0.0f
#define SHOOT_FRICTION_SPEED_PID_INT_LIMIT 20000.0f
#define SHOOT_FRICTION_SPEED_PID_MAX_OUT 12000.0f

#define SHOOT_FRICTION_CURRENT_PID_KP 1.0f
#define SHOOT_FRICTION_CURRENT_PID_KI 0.0f
#define SHOOT_FRICTION_CURRENT_PID_KD 0.0f
#define SHOOT_FRICTION_CURRENT_PID_INT_LIMIT 12000.0f
#define SHOOT_FRICTION_CURRENT_PID_MAX_OUT 15500.0f

#define SHOOT_LOADER_ANGLE_PID_KP 10.0f
#define SHOOT_LOADER_ANGLE_PID_KI 0.8f
#define SHOOT_LOADER_ANGLE_PID_KD 0.0f
#define SHOOT_LOADER_ANGLE_PID_MAX_OUT 300.0f

#define SHOOT_LOADER_SPEED_PID_KP 12.0f
#define SHOOT_LOADER_SPEED_PID_KI 1.2f
#define SHOOT_LOADER_SPEED_PID_KD 0.0f
#define SHOOT_LOADER_SPEED_PID_INT_LIMIT 7000.0f
#define SHOOT_LOADER_SPEED_PID_MAX_OUT 7000.0f

#define SHOOT_LOADER_CURRENT_PID_KP 0.7f
#define SHOOT_LOADER_CURRENT_PID_KI 0.15f
#define SHOOT_LOADER_CURRENT_PID_KD 0.0f
#define SHOOT_LOADER_CURRENT_PID_INT_LIMIT 7000.0f
#define SHOOT_LOADER_CURRENT_PID_MAX_OUT 7000.0f

#endif // ROBOT_INFANTRY_CONFIG_H
