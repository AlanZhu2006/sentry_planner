#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <stdint.h>
#include "robot_def.h"

typedef struct
{
    uint8_t vision_enabled;
    uint8_t keyboard_mouse_enabled;
    uint8_t emergency_latched;
    uint8_t shooter_ready;
    uint8_t fire_request_active;
    uint8_t reverse_request_active;
    shoot_fire_select_e fire_select;
} RobotCMDDebugState_s;

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

/**
 * @brief 获取 CMD 层调试状态快照(供 dashboard 使用)
 */
void RobotCMDGetDebugState(RobotCMDDebugState_s *out);

#endif // !ROBOT_CMD_H
