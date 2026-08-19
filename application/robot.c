#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "led.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_NAV2_CONTROL
#include "sentry_steer_nav2.h"
#elif defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_KEYBOARD_TEST
#include "sentry_steer_keyboard_test.h"
#elif defined(ROBOT_TYPE_damiao)
#include "damiao.h"
#endif

#if !defined(ROBOT_TYPE_damiao) && \
    !(defined(ROBOT_TYPE_sentry_swerve) && \
      (SENTRY_STEER_KEYBOARD_TEST || SENTRY_STEER_NAV2_CONTROL)) && \
    (defined(ONE_BOARD) || defined(CHASSIS_BOARD))
#include "chassis.h"
#endif

#if !defined(ROBOT_TYPE_damiao) && \
    !(defined(ROBOT_TYPE_sentry_swerve) && \
      (SENTRY_STEER_KEYBOARD_TEST || SENTRY_STEER_NAV2_CONTROL)) && \
    (defined(ONE_BOARD) || defined(GIMBAL_BOARD))
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

    LEDInit();
    LEDSetStatus(LED_STATUS_RED_ON);

#if defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_NAV2_CONTROL
    SentrySteerNav2Init();
#elif defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_KEYBOARD_TEST
    SentrySteerKeyboardTestInit();
#elif defined(ROBOT_TYPE_damiao)
    DamiaoInit();
#elif defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#endif

#if !defined(ROBOT_TYPE_damiao) && \
    !(defined(ROBOT_TYPE_sentry_swerve) && \
      (SENTRY_STEER_KEYBOARD_TEST || SENTRY_STEER_NAV2_CONTROL)) && \
    (defined(ONE_BOARD) || defined(CHASSIS_BOARD))
    ChassisInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_NAV2_CONTROL
    SentrySteerNav2Task();
#elif defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_KEYBOARD_TEST
    SentrySteerKeyboardTestTask();
#elif defined(ROBOT_TYPE_damiao)
    DamiaoTask();
#elif defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
#endif

#if !defined(ROBOT_TYPE_damiao) && \
    !(defined(ROBOT_TYPE_sentry_swerve) && \
      (SENTRY_STEER_KEYBOARD_TEST || SENTRY_STEER_NAV2_CONTROL)) && \
    (defined(ONE_BOARD) || defined(CHASSIS_BOARD))
    ChassisTask();
#endif

}
