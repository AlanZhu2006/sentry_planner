/**
 * @file led.h
 * @brief RGB LED status indicator module
 * @version 1.0
 * @date 2026-01-29
 */

#ifndef LED_H
#define LED_H

#include "stdint.h"

/**
 * @brief LED状态枚举
 */
typedef enum {
    LED_STATUS_OFF = 0,        // LED关闭
    LED_STATUS_RED_BLINK,      // 红灯闪烁（IMU校准中）
    LED_STATUS_GREEN_ON,       // 绿灯常亮（初始化成功）
    LED_STATUS_BLUE_BLINK,     // 蓝灯闪烁（初始化失败）
    LED_STATUS_RED_ON,         // 红灯常亮
    LED_STATUS_BLUE_ON,        // 蓝灯常亮
    LED_STATUS_YELLOW_ON,      // 黄灯常亮（红+绿）
    LED_STATUS_PURPLE_ON,      // 紫灯常亮（红+蓝）
    LED_STATUS_CYAN_ON,        // 青灯常亮（绿+蓝）
    LED_STATUS_WHITE_ON,       // 白灯常亮（红+绿+蓝）
} LED_Status_e;

/**
 * @brief LED实例结构体
 */
typedef struct {
    LED_Status_e status;       // 当前LED状态
    uint32_t blink_count;      // 闪烁计数器
    float brightness;          // 亮度 (0.0~1.0)
} LEDInstance;

/**
 * @brief 初始化LED模块
 * @note 必须在PWM初始化之后调用
 */
void LEDInit(void);

/**
 * @brief 设置LED状态
 * @param status LED状态
 */
void LEDSetStatus(LED_Status_e status);

/**
 * @brief 设置LED亮度
 * @param brightness 亮度 (0.0~1.0)
 */
void LEDSetBrightness(float brightness);

/**
 * @brief LED任务，需要周期性调用以实现闪烁效果
 * @note 建议以50-100Hz频率调用
 */
void LEDTask(void);

#endif // LED_H
