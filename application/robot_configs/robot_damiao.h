#pragma once
#ifndef ROBOT_DAMIAO_CONFIG_H
#define ROBOT_DAMIAO_CONFIG_H

#include "robot_infantry.h"

/*
 * damiao 车型复用 infantry 的整机基础配置，只额外打开一个最小达妙单电机示例。
 * 当前 bench 配置用于单 C 板 + 单达妙 + CAN1 / CAN ID 6 / master ID 0 的使能测试。
 */
#define DAMIAO_MOTOR_CAN_BUS hfdcan1
#define DAMIAO_MOTOR_ID 0x06u
#define DAMIAO_MOTOR_MASTER_ID 0x00u
#define DAMIAO_MOTOR_REVERSE MOTOR_DIRECTION_NORMAL

#define DAMIAO_ENABLE_ONLY_TEST 1u
#define DAMIAO_TARGET_SPEED_RAD_S 0.00f
#define DAMIAO_LOG_PERIOD_MS 1000u

#endif // ROBOT_DAMIAO_CONFIG_H
