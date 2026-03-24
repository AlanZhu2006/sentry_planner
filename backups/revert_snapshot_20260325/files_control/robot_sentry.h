#pragma once
#ifndef ROBOT_SENTRY_CONFIG_H
#define ROBOT_SENTRY_CONFIG_H

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

#define REMOTE_CONTROL_UART_HANDLE huart3 // DBUS 遥控
// #define REMOTE_CONTROL_UART_HANDLE huart1 // VT03/VT13 图传遥控

#define CHASSIS_TYPE_SWERVE

/* 机器人重要参数定义 (已从 robomaster-control sentry_swerve 迁移) */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 5736  // 云台和底盘对齐时的编码器值 (robomaster: initial_angle aligned with chassis vx)
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 15.0f       // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度) - 向上抬头
#define PITCH_MIN_ANGLE -36.0f      // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度) - 向下低头
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 2006拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH -1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反
// 人机输入方向配置,只影响操作器输入映射,不影响IMU/电机/视觉方向
#define GIMBAL_RC_YAW_INPUT_SIGN 1
#define GIMBAL_RC_PITCH_INPUT_SIGN 1
#define GIMBAL_MOUSE_YAW_INPUT_SIGN 1
#define GIMBAL_MOUSE_PITCH_INPUT_SIGN -1

// 电机正反转配置 (参考步兵; 云台单向转不动时尝试改为REVERSE)
#define GIMBAL_YAW_MOTOR_REVERSE MOTOR_DIRECTION_REVERSE
#define GIMBAL_PITCH_MOTOR_REVERSE MOTOR_DIRECTION_REVERSE
#define CHASSIS_MOTOR_LF_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_RF_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_LB_REVERSE MOTOR_DIRECTION_NORMAL
#define CHASSIS_MOTOR_RB_REVERSE MOTOR_DIRECTION_NORMAL
#define STEER_MOTOR_A_REVERSE MOTOR_DIRECTION_NORMAL  // Swerve转向电机A
#define STEER_MOTOR_B_REVERSE MOTOR_DIRECTION_NORMAL  // Swerve转向电机B
#define SHOOT_FRICTION_L_REVERSE MOTOR_DIRECTION_REVERSE
#define SHOOT_FRICTION_R_REVERSE MOTOR_DIRECTION_NORMAL  // 与左摩擦轮同向
#define SHOOT_LOADER_REVERSE MOTOR_DIRECTION_NORMAL

// 底盘驱动电机CAN配置 (M3508)
#define CHASSIS_CAN_BUS hcan1
#define CHASSIS_MOTOR_LF_ID 1  // 左前驱动轮
#define CHASSIS_MOTOR_RF_ID 2  // 右前驱动轮
#define CHASSIS_MOTOR_LB_ID 4  // 左后驱动轮
#define CHASSIS_MOTOR_RB_ID 3  // 右后驱动轮

// Swerve转向电机CAN配置 (GM6020)
// 与 robomaster-control sentry_swerve.h 一致: Steer A=5, Steer B=6, Gimbal Yaw=7
#define STEER_MOTOR_A_ID 5            // 转向电机A (CAN1, 0x209)
#define STEER_MOTOR_B_ID 6            // 转向电机B (CAN1, 0x20A), 必须为6(GM6020无ID8)
#define STEER_ECD_PER_REV 8192.0f     // GM6020编码器分辨率 (0-8191)
// 两个舵轮应同时指向前方时对应的编码器刻度
// 若 RC 控制走不了、两轮反向，对其中一轮 init 加 4096(半圈) 修正
#define STEER_MOTOR_A_INIT_ANGLE 1084.0f   // A朝前 (robomaster)
#define STEER_MOTOR_B_INIT_ANGLE 6530.0f   // B朝前: 2434+4096 修正B轮180°反向

// 云台电机CAN配置 (GM6020)
// CAN1: Steer 5,6 + Yaw 7; CAN2: Pitch 5 (与robomaster-control sentry_swerve一致)
#define GIMBAL_YAW_CAN_BUS hcan1
#define GIMBAL_YAW_MOTOR_ID 7         // 云台Yaw (CAN1, 0x20B), 必须为7避免与Steer B(ID6)冲突
#define GIMBAL_PITCH_CAN_BUS hcan2
#define GIMBAL_PITCH_MOTOR_ID 5       // 云台Pitch (CAN2, 0x209), 与robomaster一致

// 发射机构电机CAN配置 (与robomaster-control sentry_swerve.h一致)
#define SHOOT_CAN_BUS hcan2
#define SHOOT_LOADER_ID 1      // 拨盘/供弹 (M3508 ID 1, 0x201)
#define SHOOT_FRICTION_L_ID 6  // 左摩擦轮 (M3508 ID 6, 0x206)
#define SHOOT_FRICTION_R_ID 8  // 右摩擦轮 (M3508 ID 8, 0x208)

// 底盘运动参数
#define CHASSIS_ROTATE_SPEED 15000.0f  // 小陀螺模式旋转速度
#define CHASSIS_DRIVE_SPEED_SCALE 18000.0f  // 驱动轮满速缩放(mag=1 时的速度)，调大=最大速度更快
#define CHASSIS_RC_MOVE_RATIO_X 3.0f // 遥控器模式底盘前后移动速度系数(已按摇杆归一化，此保留备用)
#define CHASSIS_RC_MOVE_RATIO_Y 3.0f
#define SENTRY_OPERATOR_INPUT_ROTATE_CCW_90 1 // 操作器前后左右映射整体逆时针旋转 90 度
#define CHASSIS_KB_MOVE_SPEED_X 300.0f // 键鼠模式底盘前后移动速度
#define CHASSIS_KB_MOVE_SPEED_Y 300.0f
#define SENTRY_EXT_CMD_FIXED_FRAME 1 // 外部 BT/ROS 底盘命令按固定底盘坐标系解释，云台扫描时不再旋转 vx/vy
#define SENTRY_EXT_CMD_VX_SIGN -1.0f // 外部 BT/ROS vx 轴修正: 当前机构前进方向与雷达前方相反
#define SENTRY_EXT_CMD_VY_SIGN 1.0f  // 外部 BT/ROS vy 轴修正: +vy 保持为雷达左方
#define SENTRY_GIMBAL_AUTO_ROTATE_PERIOD_S 4.0f // 右拨杆中位时云台自转一圈周期，建议保持在 3~5s
#define SENTRY_VISION_SEARCH_YAW_RATE_DEG_S 120.0f // 左拨杆中位且未识别目标时，yaw 360°搜索角速度
#define SENTRY_VISION_SEARCH_PITCH_CENTER_DEG -6.0f // 左拨杆中位搜索时 pitch 摆动中心角
#define SENTRY_VISION_SEARCH_PITCH_SWEEP_DEG 24.0f // 左拨杆中位搜索时 pitch 半摆幅，实际范围 [center-sweep, center+sweep]
#define SENTRY_VISION_SEARCH_PITCH_PERIOD_S 1.6f // 左拨杆中位搜索时 pitch 完成一轮上下摆的周期

// PID参数 - 底盘驱动轮 (M3508, 来自 robomaster sentry_swerve)
#define CHASSIS_SPEED_PID_KP 10.0f
#define CHASSIS_SPEED_PID_KI 0.0f
#define CHASSIS_SPEED_PID_KD 0.0f
#define CHASSIS_SPEED_PID_INT_LIMIT 3000.0f
#define CHASSIS_SPEED_PID_MAX_OUT 12000.0f

#define CHASSIS_CURRENT_PID_KP 0.5f
#define CHASSIS_CURRENT_PID_KI 0.0f
#define CHASSIS_CURRENT_PID_KD 0.0f
#define CHASSIS_CURRENT_PID_INT_LIMIT 3000.0f
#define CHASSIS_CURRENT_PID_MAX_OUT 15000.0f

// PID参数 - Swerve转向电机 (GM6020)
// 回到「基本上是好的」那版；死区过大(如 3.5°)易在边界极限环→颤更大
// 角度环 (outer: angle->speed)
#define STEER_ANGLE_PID_KP 15.0f
#define STEER_ANGLE_PID_KI 0.22f
#define STEER_ANGLE_PID_KD 0.0f
#define STEER_ANGLE_PID_DEADBAND 0.5f   // 度
#define STEER_ANGLE_PID_INT_LIMIT 520.0f
#define STEER_ANGLE_PID_MAX_OUT 650.0f
#define STEER_ANGLE_OUTPUT_LPF_RC 0.048f
#define STEER_ANGLE_OUTPUT_MAX_DELTA 38.0f

// 速度环 (inner)
#define STEER_SPEED_PID_KP 90.0f
#define STEER_SPEED_PID_KI 52.0f
#define STEER_SPEED_PID_KD 0.0f
#define STEER_SPEED_PID_INT_LIMIT 3000.0f
#define STEER_SPEED_PID_MAX_OUT 10000.0f

// 电流环 (GM6020一般不使用)
#define STEER_CURRENT_PID_KP 0.0f
#define STEER_CURRENT_PID_KI 0.0f
#define STEER_CURRENT_PID_KD 0.0f
#define STEER_CURRENT_PID_INT_LIMIT 3000.0f
#define STEER_CURRENT_PID_MAX_OUT 15000.0f

// PID参数 - 云台 Yaw
// 第二轮调整：减轻拖尾，外环略增强、微分再放轻，速度环提高P并下调I，减少积分拖尾。
#define GIMBAL_YAW_ANGLE_PID_KP 9.0f
#define GIMBAL_YAW_ANGLE_PID_KI 0.0f
#define GIMBAL_YAW_ANGLE_PID_KD 0.4f
#define GIMBAL_YAW_ANGLE_PID_DEADBAND 0.008f
#define GIMBAL_YAW_ANGLE_PID_INT_LIMIT 100.0f
#define GIMBAL_YAW_ANGLE_PID_MAX_OUT 800.0f

#define GIMBAL_YAW_SPEED_PID_KP 130.0f
#define GIMBAL_YAW_SPEED_PID_KI 70.0f
#define GIMBAL_YAW_SPEED_PID_KD 0.0f
#define GIMBAL_YAW_SPEED_PID_INT_LIMIT 2000.0f
#define GIMBAL_YAW_SPEED_PID_MAX_OUT 23000.0f

// Pitch: 角度环为主 (robomaster single-loop, outer: 20,0,2,30000,25000)
#define GIMBAL_PITCH_ANGLE_PID_KP 20.0f
#define GIMBAL_PITCH_ANGLE_PID_KI 0.0f
#define GIMBAL_PITCH_ANGLE_PID_KD 2.0f
#define GIMBAL_PITCH_ANGLE_PID_INT_LIMIT 30000.0f
#define GIMBAL_PITCH_ANGLE_PID_MAX_OUT 25000.0f

#define GIMBAL_PITCH_SPEED_PID_KP 50.0f
#define GIMBAL_PITCH_SPEED_PID_KI 0.0f
#define GIMBAL_PITCH_SPEED_PID_KD 0.0f
#define GIMBAL_PITCH_SPEED_PID_INT_LIMIT 2500.0f
#define GIMBAL_PITCH_SPEED_PID_MAX_OUT 20000.0f

// PID参数 - 发射（直接对齐 infantry）
#define SHOOT_FRICTION_SPEED_PID_KP 7.0f
#define SHOOT_FRICTION_SPEED_PID_KI 0.8f
#define SHOOT_FRICTION_SPEED_PID_KD 0.08f
#define SHOOT_FRICTION_SPEED_PID_INT_LIMIT 20000.0f
#define SHOOT_FRICTION_SPEED_PID_MAX_OUT 12000.0f

#define SHOOT_FRICTION_CURRENT_PID_KP 0.8f
#define SHOOT_FRICTION_CURRENT_PID_KI 0.12f
#define SHOOT_FRICTION_CURRENT_PID_KD 0.0f
#define SHOOT_FRICTION_CURRENT_PID_INT_LIMIT 12000.0f
#define SHOOT_FRICTION_CURRENT_PID_MAX_OUT 15500.0f

#define SHOOT_LOADER_ANGLE_PID_KP 8.0f
#define SHOOT_LOADER_ANGLE_PID_KI 0.0f
#define SHOOT_LOADER_ANGLE_PID_KD 0.0f
#define SHOOT_LOADER_ANGLE_PID_MAX_OUT 200.0f

#define SHOOT_LOADER_SPEED_PID_KP 10.0f
#define SHOOT_LOADER_SPEED_PID_KI 1.0f
#define SHOOT_LOADER_SPEED_PID_KD 0.0f
#define SHOOT_LOADER_SPEED_PID_INT_LIMIT 5000.0f
#define SHOOT_LOADER_SPEED_PID_MAX_OUT 5000.0f

#define SHOOT_LOADER_CURRENT_PID_KP 0.5f
#define SHOOT_LOADER_CURRENT_PID_KI 0.1f
#define SHOOT_LOADER_CURRENT_PID_KD 0.0f
#define SHOOT_LOADER_CURRENT_PID_INT_LIMIT 5000.0f
#define SHOOT_LOADER_CURRENT_PID_MAX_OUT 5000.0f

#endif // ROBOT_SENTRY_CONFIG_H
