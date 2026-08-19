// app
#include "robot_cmd.h"
#include "robot_def.h"
// module
#include "bmi088.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "referee_task.h"
#include "remote_control.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "led.h"
// std
#include <math.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE                                                        \
  (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PITCH_HORIZON_ANGLE 0.0f // pitch水平时的IMU角度(IMU域: -180~180)
#define VISION_SEND_DIV 2u // 200Hz / 2 = 100Hz
#define VISION_CTRL_DT_S 0.005f
#ifndef VISION_YAW_MAX_SLEW_DEG_S
#define VISION_YAW_MAX_SLEW_DEG_S 700.0f
#endif
#ifndef VISION_PITCH_MAX_SLEW_DEG_S
#define VISION_PITCH_MAX_SLEW_DEG_S 360.0f
#endif
#ifndef VISION_TARGET_FILTER_ALPHA
#define VISION_TARGET_FILTER_ALPHA 0.9f
#endif
#ifndef VISION_TARGET_VEL_FILTER_ALPHA
#define VISION_TARGET_VEL_FILTER_ALPHA 0.9f
#endif
#ifndef VISION_TARGET_ACC_FILTER_ALPHA
#define VISION_TARGET_ACC_FILTER_ALPHA 0.35f
#endif
#ifndef VISION_VEL_LEAD_TIME_S
#define VISION_VEL_LEAD_TIME_S 0.030f
#endif
#ifndef VISION_USE_ACCEL_LEAD
#define VISION_USE_ACCEL_LEAD 1
#endif
#ifndef MANUAL_GIMBAL_BODY_LOCK_ENABLE
#define MANUAL_GIMBAL_BODY_LOCK_ENABLE 0
#endif
#define VISION_RC_DEADBAND 20
#define VISION_YAW_DEBUG_LOG 0u
#define VISION_PITCH_DEBUG_LOG 0u
#define ROBOT_CMD_DEBUG_FLAG_VISION_ENABLED 0x01u
#define ROBOT_CMD_DEBUG_FLAG_KEYBOARD_MODE 0x02u
#define ROBOT_CMD_DEBUG_FLAG_EMERGENCY_STOP 0x04u
#define DJI_YAW_ECD_FULL_RANGE 8192
#define DJI_YAW_ECD_HALF_RANGE (DJI_YAW_ECD_FULL_RANGE / 2)

#if defined(YAW_RIGHT_LIMIT_DELTA_ECD) &&                                   \
    ((YAW_RIGHT_LIMIT_DELTA_ECD < 0) ||                                      \
     (YAW_RIGHT_LIMIT_DELTA_ECD > DJI_YAW_ECD_HALF_RANGE))
#error YAW_RIGHT_LIMIT_DELTA_ECD must be in [0, 4096].
#endif

#if defined(YAW_LEFT_LIMIT_DELTA_ECD) &&                                    \
    ((YAW_LEFT_LIMIT_DELTA_ECD < 0) ||                                       \
     (YAW_LEFT_LIMIT_DELTA_ECD > DJI_YAW_ECD_HALF_RANGE))
#error YAW_LEFT_LIMIT_DELTA_ECD must be in [0, 4096].
#endif

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s
    chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s
    chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
static uint8_t vision_cmd_filter_inited;
static float vision_yaw_cmd_filtered;
static float vision_pitch_cmd_filtered;
static float vision_yaw_target_filtered;
static float vision_pitch_target_filtered;
static float vision_yaw_vel_filtered_deg_s;
static float vision_pitch_vel_filtered_deg_s;
static float vision_yaw_acc_filtered_deg_s2;
static float vision_pitch_acc_filtered_deg_s2;
static RobotCMDDebugState_s robot_cmd_debug_state;
static uint8_t vtm_vision_control_enabled;
static uint8_t vtm_keyboard_mouse_enabled;
static uint8_t vtm_shooter_ready;
static chassis_mode_e vtm_keyboard_chassis_mode;
static chassis_mode_e vtm_last_non_rotate_chassis_mode;
static shoot_fire_select_e vtm_fire_select;
static lid_mode_e vtm_lid_mode;
static Bullet_Speed_e vtm_bullet_speed;
static uint8_t vtm_fire_request_active;
static uint8_t vtm_reverse_request_active;
static uint8_t vtm_last_pause_button;
static uint8_t vtm_last_custom_right_button;
static uint8_t vtm_last_fire_request;
static uint8_t vtm_last_key_v_count;
static uint8_t vtm_last_key_f_count;
static uint8_t vtm_last_key_e_count;
static uint8_t vtm_last_key_q_count;
static uint8_t vtm_last_key_r_count;
static uint8_t vtm_last_key_z_count;
static uint8_t vtm_last_key_c_count;
static uint8_t last_referee_game_status = 0xFFu;

static uint8_t IsVideoTransmitterRemote(void) {
  return (uint8_t)(RemoteControlGetProtocol() ==
                   REMOTE_CONTROL_PROTOCOL_VIDEO_TRANSMITTER);
}

static uint8_t IsVisionControlEnabled(void) {
  if (IsVideoTransmitterRemote()) {
    return (uint8_t)(vtm_vision_control_enabled ||
                     (vtm_keyboard_mouse_enabled && rc_data[TEMP].mouse.press_r));
  }
  return switch_is_mid(rc_data[TEMP].rc.switch_left);
}

static uint8_t IsKeyboardMouseControlEnabled(void) {
  if (IsVideoTransmitterRemote()) {
    return vtm_keyboard_mouse_enabled;
  }
  return switch_is_up(rc_data[TEMP].rc.switch_left);
}

static uint8_t ConsumeKeyEdgeCount(uint8_t *last_count, uint8_t current_count) {
  uint8_t delta = (uint8_t)(current_count - *last_count);
  *last_count = current_count;
  return delta;
}

static shoot_fire_select_e NextFireSelect(shoot_fire_select_e select) {
  switch (select) {
  case FIRE_SELECT_SINGLE:
    return FIRE_SELECT_TRIPLE;
  case FIRE_SELECT_TRIPLE:
    return FIRE_SELECT_AUTO;
  case FIRE_SELECT_AUTO:
  default:
    return FIRE_SELECT_SINGLE;
  }
}

static Bullet_Speed_e NextBulletSpeed(Bullet_Speed_e speed) {
  switch (speed) {
  case SMALL_AMU_15:
    return SMALL_AMU_18;
  case SMALL_AMU_18:
    return SMALL_AMU_30;
  case SMALL_AMU_30:
  default:
    return SMALL_AMU_15;
  }
}

static int NextSpeedBuff(int speed_buff) {
  switch (speed_buff) {
  case 40:
    return 60;
  case 60:
    return 80;
  case 80:
    return 100;
  case 100:
  default:
    return 40;
  }
}

static shoot_fire_select_e FireSelectFromLoadMode(loader_mode_e load_mode) {
  switch (load_mode) {
  case LOAD_1_BULLET:
    return FIRE_SELECT_SINGLE;
  case LOAD_3_BULLET:
    return FIRE_SELECT_TRIPLE;
  case LOAD_BURSTFIRE:
  default:
    return FIRE_SELECT_AUTO;
  }
}

static gimbal_mode_e GimbalModeFromChassisMode(chassis_mode_e mode) {
  switch (mode) {
  case CHASSIS_NO_FOLLOW:
    return GIMBAL_FREE_MODE;
  case CHASSIS_ROTATE:
#if MANUAL_GIMBAL_BODY_LOCK_ENABLE
    return GIMBAL_FREE_MODE;
#endif
  case CHASSIS_FOLLOW_GIMBAL_YAW:
  default:
    return GIMBAL_GYRO_MODE;
  }
}

static inline chassis_mode_e DefaultManualChassisMode(void) {
#if MANUAL_GIMBAL_BODY_LOCK_ENABLE
  return CHASSIS_NO_FOLLOW;
#else
  return CHASSIS_FOLLOW_GIMBAL_YAW;
#endif
}

static void UpdateVideoTransmitterControlState(void) {
  uint8_t pause_pressed = rc_data[TEMP].vtm.pause;
  uint8_t custom_right_pressed = rc_data[TEMP].vtm.custom_right;
  uint8_t key_edge_count = 0u;

  if (pause_pressed && !vtm_last_pause_button) {
    vtm_vision_control_enabled ^= 1u;
  }

  if (custom_right_pressed && !vtm_last_custom_right_button) {
    vtm_keyboard_mouse_enabled ^= 1u;
    if (vtm_keyboard_mouse_enabled) {
      if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE) {
        vtm_keyboard_chassis_mode = DefaultManualChassisMode();
      } else {
        vtm_keyboard_chassis_mode = chassis_cmd_send.chassis_mode;
      }

      if (vtm_keyboard_chassis_mode != CHASSIS_ROTATE) {
        vtm_last_non_rotate_chassis_mode = vtm_keyboard_chassis_mode;
      }
    }
  }

  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_v_count, rc_data[TEMP].key_count[KEY_PRESS][Key_V]);
  while (key_edge_count--) {
    vtm_vision_control_enabled ^= 1u;
  }

  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_f_count, rc_data[TEMP].key_count[KEY_PRESS][Key_F]);
  while (key_edge_count--) {
    vtm_shooter_ready ^= 1u;
  }

#ifdef KB_ZE_DISABLE
  ConsumeKeyEdgeCount(&vtm_last_key_e_count,
                      rc_data[TEMP].key_count[KEY_PRESS][Key_E]);
#else
  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_e_count, rc_data[TEMP].key_count[KEY_PRESS][Key_E]);
  while (key_edge_count--) {
    vtm_fire_select = NextFireSelect(vtm_fire_select);
  }
#endif

  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_q_count, rc_data[TEMP].key_count[KEY_PRESS][Key_Q]);
  while (key_edge_count--) {
    if (vtm_keyboard_chassis_mode != CHASSIS_ROTATE) {
      if (vtm_keyboard_chassis_mode != CHASSIS_ZERO_FORCE) {
        vtm_last_non_rotate_chassis_mode = vtm_keyboard_chassis_mode;
      }
      vtm_keyboard_chassis_mode = CHASSIS_ROTATE;
    } else {
      vtm_keyboard_chassis_mode = vtm_last_non_rotate_chassis_mode;
    }
  }

  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_r_count, rc_data[TEMP].key_count[KEY_PRESS][Key_R]);
  while (key_edge_count--) {
    vtm_lid_mode = (vtm_lid_mode == LID_OPEN) ? LID_CLOSE : LID_OPEN;
  }

#ifdef KB_ZE_DISABLE
  ConsumeKeyEdgeCount(&vtm_last_key_z_count,
                      rc_data[TEMP].key_count[KEY_PRESS][Key_Z]);
#else
  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_z_count, rc_data[TEMP].key_count[KEY_PRESS][Key_Z]);
  while (key_edge_count--) {
    vtm_bullet_speed = NextBulletSpeed(vtm_bullet_speed);
  }
#endif

  key_edge_count =
      ConsumeKeyEdgeCount(&vtm_last_key_c_count, rc_data[TEMP].key_count[KEY_PRESS][Key_C]);
  while (key_edge_count--) {
    chassis_cmd_send.chassis_speed_buff = NextSpeedBuff(chassis_cmd_send.chassis_speed_buff);
  }

  vtm_last_pause_button = pause_pressed;
  vtm_last_custom_right_button = custom_right_pressed;
}

static float GetVideoTransmitterDialWz(void) {
  return ((float)rc_data[TEMP].rc.dial / (float)RC_CH_VALUE_OFFSET) *
         CHASSIS_ROTATE_SPEED;
}

static float GetKeyboardMoveSpeedScale(void) {
  float speed_scale;
  if (chassis_cmd_send.chassis_speed_buff <= 0) {
    speed_scale = 1.0f;
  } else {
    speed_scale = (float)chassis_cmd_send.chassis_speed_buff * 0.01f;
  }

  if (rc_data[TEMP].key[KEY_PRESS].shift) {
    return 1.0f;
  }
  if (rc_data[TEMP].key[KEY_PRESS].ctrl) {
    return speed_scale * 0.35f;
  }
  return speed_scale;
}

static float GetKeyboardAimScale(void) {
  if (rc_data[TEMP].key[KEY_PRESS].ctrl) {
    return 0.35f;
  }
  return 1.0f;
}

static void ApplyVideoTransmitterShootControl(void) {
  uint8_t keyboard_mouse_enabled = IsKeyboardMouseControlEnabled();
  uint8_t fire_request = keyboard_mouse_enabled
                             ? (uint8_t)(rc_data[TEMP].vtm.trigger ||
                                         rc_data[TEMP].mouse.press_l)
                             : rc_data[TEMP].vtm.trigger;

  vtm_fire_request_active = fire_request;
  vtm_reverse_request_active =
      keyboard_mouse_enabled ? rc_data[TEMP].key[KEY_PRESS].g : 0u;

  shoot_cmd_send.shoot_rate = 8.0f;
  shoot_cmd_send.bullet_speed = vtm_bullet_speed;
  shoot_cmd_send.lid_mode = vtm_lid_mode;

  if (robot_state != ROBOT_READY) {
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    vtm_last_fire_request = fire_request;
    return;
  }

  if (!keyboard_mouse_enabled) {
    shoot_cmd_send.friction_mode = fire_request ? FRICTION_ON : FRICTION_OFF;
    shoot_cmd_send.load_mode = fire_request ? LOAD_BURSTFIRE : LOAD_STOP;
    vtm_last_fire_request = fire_request;
    return;
  }

  if (vtm_reverse_request_active) {
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_REVERSE;
    vtm_last_fire_request = fire_request;
    return;
  }

  shoot_cmd_send.friction_mode =
      vtm_shooter_ready ? FRICTION_ON : FRICTION_OFF;

  if (!vtm_shooter_ready) {
    shoot_cmd_send.load_mode = LOAD_STOP;
    vtm_last_fire_request = fire_request;
    return;
  }

  switch (vtm_fire_select) {
  case FIRE_SELECT_SINGLE:
    shoot_cmd_send.load_mode =
        (fire_request && !vtm_last_fire_request) ? LOAD_1_BULLET : LOAD_STOP;
    break;
  case FIRE_SELECT_TRIPLE:
    shoot_cmd_send.load_mode =
        (fire_request && !vtm_last_fire_request) ? LOAD_3_BULLET : LOAD_STOP;
    break;
  case FIRE_SELECT_AUTO:
  default:
    shoot_cmd_send.load_mode = fire_request ? LOAD_BURSTFIRE : LOAD_STOP;
    break;
  }

  vtm_last_fire_request = fire_request;
}

static void UpdateRobotCMDDebugState(uint8_t vision_enabled,
                                     uint8_t keyboard_mouse_enabled) {
  robot_cmd_debug_state.vision_enabled = vision_enabled;
  robot_cmd_debug_state.keyboard_mouse_enabled = keyboard_mouse_enabled;
  robot_cmd_debug_state.emergency_latched =
      (uint8_t)(robot_state == ROBOT_STOP);
  robot_cmd_debug_state.shooter_ready =
      (uint8_t)(shoot_cmd_send.friction_mode == FRICTION_ON);
  robot_cmd_debug_state.fire_request_active =
      IsVideoTransmitterRemote()
          ? vtm_fire_request_active
          : (uint8_t)(shoot_cmd_send.load_mode != LOAD_STOP);
  robot_cmd_debug_state.reverse_request_active =
      IsVideoTransmitterRemote()
          ? vtm_reverse_request_active
          : (uint8_t)(shoot_cmd_send.load_mode == LOAD_REVERSE);
  robot_cmd_debug_state.fire_select =
      IsVideoTransmitterRemote() ? vtm_fire_select
                                 : FireSelectFromLoadMode(shoot_cmd_send.load_mode);
}

static inline void LimitPitchAngle(void) {
  if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
  else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
}

static inline float YawMotorRelativeRawDegToCtrlDeg(float motor_relative_raw_deg) {
  float ref_sign = (float)GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN;

  if (ref_sign == 0.0f) {
    return 0.0f;
  }

  return motor_relative_raw_deg / ref_sign;
}

static inline float YawCtrlDegToMotorRelativeRawDeg(float ctrl_deg) {
  float ref_sign = (float)GIMBAL_YAW_PID_REF_FROM_CTRL_SIGN;

  return ctrl_deg * ref_sign;
}

static inline int16_t WrapYawEncoderDeltaEcd(int32_t delta_ecd) {
  if (delta_ecd > DJI_YAW_ECD_HALF_RANGE)
    delta_ecd -= DJI_YAW_ECD_FULL_RANGE;
  else if (delta_ecd < -DJI_YAW_ECD_HALF_RANGE)
    delta_ecd += DJI_YAW_ECD_FULL_RANGE;

  return (int16_t)delta_ecd;
}

static inline float CurrentGimbalYawBodyLockCtrlDeg(void);

static inline int32_t ClampYawTargetDeltaEcd(int32_t target_delta_ecd) {
#if defined(YAW_RIGHT_LIMIT_DELTA_ECD) && defined(YAW_LEFT_LIMIT_DELTA_ECD)
  int32_t right_limit_delta_ecd;
  int32_t left_limit_delta_ecd;
  int32_t min_limit_delta_ecd;
  int32_t max_limit_delta_ecd;

  if (YawCtrlDegToMotorRelativeRawDeg(1.0f) >= 0.0f) {
    right_limit_delta_ecd = (int32_t)YAW_RIGHT_LIMIT_DELTA_ECD;
    left_limit_delta_ecd = -(int32_t)YAW_LEFT_LIMIT_DELTA_ECD;
  } else {
    right_limit_delta_ecd = -(int32_t)YAW_RIGHT_LIMIT_DELTA_ECD;
    left_limit_delta_ecd = (int32_t)YAW_LEFT_LIMIT_DELTA_ECD;
  }

  min_limit_delta_ecd = (right_limit_delta_ecd < left_limit_delta_ecd)
                            ? right_limit_delta_ecd
                            : left_limit_delta_ecd;
  max_limit_delta_ecd = (right_limit_delta_ecd > left_limit_delta_ecd)
                            ? right_limit_delta_ecd
                            : left_limit_delta_ecd;

  if (target_delta_ecd > max_limit_delta_ecd)
    target_delta_ecd = max_limit_delta_ecd;
  if (target_delta_ecd < min_limit_delta_ecd)
    target_delta_ecd = min_limit_delta_ecd;
#endif
  return target_delta_ecd;
}

static inline float ClampYawRelativeCtrlCmdToEncoderWindow(float yaw_cmd) {
  int32_t target_delta_ecd =
      (int32_t)lroundf(YawCtrlDegToMotorRelativeRawDeg(yaw_cmd) /
                       ECD_ANGLE_COEF_DJI);
  target_delta_ecd = ClampYawTargetDeltaEcd(target_delta_ecd);
  return YawMotorRelativeRawDegToCtrlDeg((float)target_delta_ecd *
                                         ECD_ANGLE_COEF_DJI);
}

static inline float ClampYawCmdToEncoderWindow(float yaw_cmd,
                                               float current_cmd_yaw) {
#if defined(YAW_RIGHT_LIMIT_DELTA_ECD) && defined(YAW_LEFT_LIMIT_DELTA_ECD)
  int32_t current_delta_ecd;
  int32_t target_delta_ecd;

  if ((gimbal_fetch_data.motor_online_bitmap & 0x01u) == 0u) {
    return yaw_cmd;
  }

#if GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK
  if (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE) {
    return ClampYawRelativeCtrlCmdToEncoderWindow(yaw_cmd);
  }
#endif

  current_delta_ecd = (int32_t)WrapYawEncoderDeltaEcd(
      (int32_t)gimbal_fetch_data.yaw_motor_encoder_raw -
      (int32_t)YAW_CHASSIS_ALIGN_ECD);
  target_delta_ecd =
      current_delta_ecd +
      (int32_t)lroundf(YawCtrlDegToMotorRelativeRawDeg(yaw_cmd - current_cmd_yaw) /
                       ECD_ANGLE_COEF_DJI);
  target_delta_ecd = ClampYawTargetDeltaEcd(target_delta_ecd);

  return current_cmd_yaw + YawMotorRelativeRawDegToCtrlDeg(
                               (float)(target_delta_ecd - current_delta_ecd) *
                               ECD_ANGLE_COEF_DJI);
#else
  return yaw_cmd;
#endif
}

static inline void LimitYawAngle(void) {
  float current_cmd_yaw = gimbal_fetch_data.yaw_actual_angle;

#if GIMBAL_FREE_YAW_USE_MOTOR_FEEDBACK
  if (gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE &&
      (gimbal_fetch_data.motor_online_bitmap & 0x01u) != 0u) {
    current_cmd_yaw = CurrentGimbalYawBodyLockCtrlDeg();
  }
#endif

  gimbal_cmd_send.yaw =
      ClampYawCmdToEncoderWindow(gimbal_cmd_send.yaw, current_cmd_yaw);
}

static inline float WrapDeg180(float deg) {
  deg = fmodf(deg + 180.0f, 360.0f);
  if (deg < 0.0f)
    deg += 360.0f;
  return deg - 180.0f;
}

static inline float ImuAxisToGimbalDeg(float imu_deg, int dir) {
  return imu_deg * (float)dir;
}

static inline float RcYawDeltaDeg(float raw_input) {
  return 0.003f * raw_input * (float)GIMBAL_RC_YAW_INPUT_SIGN;
}

static inline float RcPitchDeltaDeg(float raw_input) {
  return 0.001f * raw_input * (float)GIMBAL_RC_PITCH_INPUT_SIGN;
}

static inline float MouseYawDeltaDeg(float raw_input) {
  return (raw_input / 660.0f) * 10.0f * (float)GIMBAL_MOUSE_YAW_INPUT_SIGN;
}

static inline float MousePitchDeltaDeg(float raw_input) {
  return (raw_input / 660.0f) * 10.0f * (float)GIMBAL_MOUSE_PITCH_INPUT_SIGN;
}

static inline float RcChassisForwardInput(void) {
#if CHASSIS_MANUAL_INPUT_FORWARD_ON_VX
  return (float)rc_data[TEMP].rc.rocker_r1;
#else
  return (float)rc_data[TEMP].rc.rocker_r_;
#endif
}

static inline float RcChassisLateralInput(void) {
#if CHASSIS_MANUAL_INPUT_FORWARD_ON_VX
  return (float)rc_data[TEMP].rc.rocker_r_;
#else
  return (float)rc_data[TEMP].rc.rocker_r1;
#endif
}

static inline float KeyboardChassisForwardInput(void) {
#if CHASSIS_MANUAL_INPUT_FORWARD_ON_VX
  return (float)(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s);
#else
  return (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d);
#endif
}

static inline float KeyboardChassisLateralInput(void) {
#if CHASSIS_MANUAL_INPUT_FORWARD_ON_VX
  return (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d);
#else
  return (float)(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s);
#endif
}

static inline float UnwrapVisionYawToGimbalTotal(float vision_yaw_deg,
                                                 float gimbal_yaw_total_deg) {
  float gimbal_yaw_mod_deg = WrapDeg180(gimbal_yaw_total_deg);
  float delta_deg = WrapDeg180(vision_yaw_deg - gimbal_yaw_mod_deg);
  return gimbal_yaw_total_deg + delta_deg;
}

static inline float LimitDelta(float delta, float max_abs) {
  if (delta > max_abs)
    return max_abs;
  if (delta < -max_abs)
    return -max_abs;
  return delta;
}

static inline float FiniteOrZero(float value) {
  return (value == value) ? value : 0.0f;
}

static inline void ResetVisionCmdFilter(float hold_yaw, float hold_pitch) {
  vision_cmd_filter_inited = 1u;
  vision_yaw_cmd_filtered = hold_yaw;
  vision_pitch_cmd_filtered = hold_pitch;
  vision_yaw_target_filtered = hold_yaw;
  vision_pitch_target_filtered = hold_pitch;
  vision_yaw_vel_filtered_deg_s = 0.0f;
  vision_pitch_vel_filtered_deg_s = 0.0f;
  vision_yaw_acc_filtered_deg_s2 = 0.0f;
  vision_pitch_acc_filtered_deg_s2 = 0.0f;
}

static inline void SyncGimbalCmdToCurrentPose(void) {
  gimbal_cmd_send.yaw = gimbal_fetch_data.yaw_actual_angle;
  gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_actual_angle;
  LimitPitchAngle();
  ResetVisionCmdFilter(gimbal_cmd_send.yaw, gimbal_cmd_send.pitch);
}

static inline float CurrentGimbalYawMotorRelativeRawDeg(void) {
  int32_t delta_ecd = (int32_t)WrapYawEncoderDeltaEcd(
      (int32_t)gimbal_fetch_data.yaw_motor_encoder_raw -
      (int32_t)YAW_CHASSIS_ALIGN_ECD);
  return (float)delta_ecd * ECD_ANGLE_COEF_DJI;
}

static inline float CurrentGimbalYawBodyLockCtrlDeg(void) {
  return YawMotorRelativeRawDegToCtrlDeg(CurrentGimbalYawMotorRelativeRawDeg());
}

static inline void SyncGimbalCmdToCurrentBodyLockPose(void) {
  if ((gimbal_fetch_data.motor_online_bitmap & 0x01u) == 0u) {
    SyncGimbalCmdToCurrentPose();
    return;
  }

  gimbal_cmd_send.yaw = CurrentGimbalYawBodyLockCtrlDeg();
  gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_actual_angle;
  LimitPitchAngle();
  LimitYawAngle();
  ResetVisionCmdFilter(gimbal_cmd_send.yaw, gimbal_cmd_send.pitch);
}

static inline void HandleManualBodyLockModeTransition(void) {
#if MANUAL_GIMBAL_BODY_LOCK_ENABLE
  static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE;
  static uint8_t last_gimbal_mode_valid = 0u;
  static uint8_t last_yaw_feedback_online = 0u;
  uint8_t yaw_feedback_online =
      (uint8_t)((gimbal_fetch_data.motor_online_bitmap & 0x01u) != 0u);

  if ((!last_gimbal_mode_valid || last_gimbal_mode != GIMBAL_FREE_MODE) &&
      gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE) {
    SyncGimbalCmdToCurrentBodyLockPose();
  } else if (!last_yaw_feedback_online && yaw_feedback_online &&
             gimbal_cmd_send.gimbal_mode == GIMBAL_FREE_MODE) {
    SyncGimbalCmdToCurrentBodyLockPose();
  }

  last_gimbal_mode = gimbal_cmd_send.gimbal_mode;
  last_gimbal_mode_valid = 1u;
  last_yaw_feedback_online = yaw_feedback_online;
#endif
}

static inline uint8_t CanUpdateGimbalTargets(void) {
  return (uint8_t)(gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE);
}

static inline void UpdateVisionTargetFilter(float target_yaw,
                                            float target_pitch,
                                            float target_yaw_vel_deg_s,
                                            float target_pitch_vel_deg_s,
                                            float target_yaw_acc_deg_s2,
                                            float target_pitch_acc_deg_s2) {
  target_yaw = FiniteOrZero(target_yaw);
  target_pitch = FiniteOrZero(target_pitch);
  target_yaw_vel_deg_s = FiniteOrZero(target_yaw_vel_deg_s);
  target_pitch_vel_deg_s = FiniteOrZero(target_pitch_vel_deg_s);
  target_yaw_acc_deg_s2 = FiniteOrZero(target_yaw_acc_deg_s2);
  target_pitch_acc_deg_s2 = FiniteOrZero(target_pitch_acc_deg_s2);
  target_yaw =
      ClampYawCmdToEncoderWindow(target_yaw, gimbal_fetch_data.yaw_actual_angle);
  if (!vision_cmd_filter_inited) {
    vision_cmd_filter_inited = 1u;
    vision_yaw_cmd_filtered = target_yaw;
    vision_pitch_cmd_filtered = target_pitch;
    vision_yaw_target_filtered = target_yaw;
    vision_pitch_target_filtered = target_pitch;
    vision_yaw_vel_filtered_deg_s = target_yaw_vel_deg_s;
    vision_pitch_vel_filtered_deg_s = target_pitch_vel_deg_s;
    vision_yaw_acc_filtered_deg_s2 = target_yaw_acc_deg_s2;
    vision_pitch_acc_filtered_deg_s2 = target_pitch_acc_deg_s2;
    return;
  }

  vision_yaw_target_filtered +=
      (target_yaw - vision_yaw_target_filtered) * VISION_TARGET_FILTER_ALPHA;
  vision_yaw_target_filtered =
      ClampYawCmdToEncoderWindow(vision_yaw_target_filtered,
                                 gimbal_fetch_data.yaw_actual_angle);
  vision_pitch_target_filtered +=
      (target_pitch - vision_pitch_target_filtered) * VISION_TARGET_FILTER_ALPHA;
  vision_yaw_vel_filtered_deg_s +=
      (target_yaw_vel_deg_s - vision_yaw_vel_filtered_deg_s) *
      VISION_TARGET_VEL_FILTER_ALPHA;
  vision_pitch_vel_filtered_deg_s +=
      (target_pitch_vel_deg_s - vision_pitch_vel_filtered_deg_s) *
      VISION_TARGET_VEL_FILTER_ALPHA;
  vision_yaw_acc_filtered_deg_s2 +=
      (target_yaw_acc_deg_s2 - vision_yaw_acc_filtered_deg_s2) *
      VISION_TARGET_ACC_FILTER_ALPHA;
  vision_pitch_acc_filtered_deg_s2 +=
      (target_pitch_acc_deg_s2 - vision_pitch_acc_filtered_deg_s2) *
      VISION_TARGET_ACC_FILTER_ALPHA;
}

static inline void ApplyVisionCmdSlew(float dt_s) {
  float lead_time_s = VISION_VEL_LEAD_TIME_S;
  float yaw_target = vision_yaw_target_filtered +
                     vision_yaw_vel_filtered_deg_s * lead_time_s;
  float pitch_target = vision_pitch_target_filtered +
                       vision_pitch_vel_filtered_deg_s * lead_time_s;
#if VISION_USE_ACCEL_LEAD
  float lead_time_sq = lead_time_s * lead_time_s;
  yaw_target += 0.5f * vision_yaw_acc_filtered_deg_s2 * lead_time_sq;
  pitch_target += 0.5f * vision_pitch_acc_filtered_deg_s2 * lead_time_sq;
#endif
  float yaw_step_max = VISION_YAW_MAX_SLEW_DEG_S * dt_s;
  float pitch_step_max = VISION_PITCH_MAX_SLEW_DEG_S * dt_s;

  float yaw_delta = yaw_target - vision_yaw_cmd_filtered;
  yaw_delta = LimitDelta(yaw_delta, yaw_step_max);
  vision_yaw_cmd_filtered += yaw_delta;
  vision_yaw_cmd_filtered =
      ClampYawCmdToEncoderWindow(vision_yaw_cmd_filtered,
                                 gimbal_fetch_data.yaw_actual_angle);

  float pitch_delta = pitch_target - vision_pitch_cmd_filtered;
  pitch_delta = LimitDelta(pitch_delta, pitch_step_max);
  vision_pitch_cmd_filtered += pitch_delta;
}

static inline void ApplyVisionNoFireHold(void) {
  gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

  float hold_yaw = gimbal_fetch_data.yaw_actual_angle;
  float hold_pitch = gimbal_fetch_data.pitch_actual_angle;

  // 丢失目标时：键鼠模式优先允许鼠标接管，否则回退到摇杆微调。
  if (IsVideoTransmitterRemote() && IsKeyboardMouseControlEnabled()) {
    float aim_scale = GetKeyboardAimScale();
    gimbal_cmd_send.yaw =
        hold_yaw + MouseYawDeltaDeg((float)rc_data[TEMP].mouse.x) * aim_scale;
    gimbal_cmd_send.pitch =
        hold_pitch + MousePitchDeltaDeg((float)rc_data[TEMP].mouse.y) * aim_scale;
  } else if (rc_data[TEMP].rc.rocker_l_ > VISION_RC_DEADBAND ||
             rc_data[TEMP].rc.rocker_l_ < -VISION_RC_DEADBAND ||
             rc_data[TEMP].rc.rocker_l1 > VISION_RC_DEADBAND ||
             rc_data[TEMP].rc.rocker_l1 < -VISION_RC_DEADBAND) {
    gimbal_cmd_send.yaw = hold_yaw + RcYawDeltaDeg((float)rc_data[TEMP].rc.rocker_l_);
    gimbal_cmd_send.pitch = hold_pitch + RcPitchDeltaDeg((float)rc_data[TEMP].rc.rocker_l1);
  } else {
    gimbal_cmd_send.yaw = hold_yaw;
    gimbal_cmd_send.pitch = hold_pitch;
  }
  LimitPitchAngle();
  LimitYawAngle();

  // 同步视觉指令滤波器，防止重新进入视觉控制时首帧跳变
  ResetVisionCmdFilter(gimbal_cmd_send.yaw, gimbal_cmd_send.pitch);

  // 丢目标时不持续拨弹
  shoot_cmd_send.load_mode = LOAD_STOP;
}

void RobotCMDInit() {
  // The alternative interrupt-driven BMI088Register example below belongs to
  // the old C-board pinout and must not be enabled on OmniCtrl Pro 2. The new
  // Cube configuration exposes no BMI088 DRDY pins; the active polling INS
  // path in ins_task.c uses SPI4, PE4/PA0 chip selects, and TIM3_CH4 safely.
  // BMI088_Init_Config_s bmi088_config = {
  //     .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
  //     .work_mode = BMI088_BLOCK_TRIGGER_MODE,
  //     .spi_acc_config = {
  //         .spi_handle = &hspi1,
  //         .GPIOx = GPIOA,
  //         .cs_pin = GPIO_PIN_4,
  //         .spi_work_mode = SPI_DMA_MODE,
  //     },
  //     .acc_int_config = {
  //         .GPIOx = GPIOC,
  //         .GPIO_Pin = GPIO_PIN_4,
  //         .exti_mode = GPIO_EXTI_MODE_RISING,
  //     },
  //     .spi_gyro_config = {
  //         .spi_handle = &hspi1,
  //         .GPIOx = GPIOB,
  //         .cs_pin = GPIO_PIN_0,
  //         .spi_work_mode = SPI_DMA_MODE,
  //     },
  //     .gyro_int_config = {
  //         .GPIO_Pin = GPIO_PIN_5,
  //         .GPIOx = GPIOC,
  //         .exti_mode = GPIO_EXTI_MODE_RISING,
  //     },
  //     .heat_pwm_config = {
  //         .htim = &htim10,
  //         .channel = TIM_CHANNEL_1,
  //         .period = 1,
  //     },
  //     .heat_pid_config = {
  //         .Kp = 0.5,
  //         .Ki = 0,
  //         .Kd = 0,
  //         .DeadBand = 0.1,
  //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
  //         PID_Derivative_On_Measurement, .IntegralLimit = 100, .MaxOut = 100,
  //     },
  // };
  // bmi088_test = BMI088Register(&bmi088_config);
  rc_data = RemoteControlInit(&REMOTE_CONTROL_UART_HANDLE);
  if (rc_data == NULL) {
    LOGERROR("[CMD] remote control init failed");
    while (1)
      ;
  }
  vtm_vision_control_enabled = 0u;
  vtm_keyboard_mouse_enabled = 1u;
  vtm_shooter_ready = 0u;
  vtm_keyboard_chassis_mode = DefaultManualChassisMode();
  vtm_last_non_rotate_chassis_mode = DefaultManualChassisMode();
  vtm_fire_select = VTM_DEFAULT_FIRE_SELECT;
  vtm_lid_mode = LID_CLOSE;
  vtm_bullet_speed = SMALL_AMU_15;
  vtm_fire_request_active = 0u;
  vtm_reverse_request_active = 0u;
  vtm_last_pause_button = 0u;
  vtm_last_custom_right_button = 0u;
  vtm_last_fire_request = 0u;
  vtm_last_key_v_count = 0u;
  vtm_last_key_f_count = 0u;
  vtm_last_key_e_count = 0u;
  vtm_last_key_q_count = 0u;
  vtm_last_key_r_count = 0u;
  vtm_last_key_z_count = 0u;
  vtm_last_key_c_count = 0u;
  chassis_cmd_send.chassis_speed_buff = 100;
  chassis_cmd_send.power_burst_enabled = 0u;
  // vision_recv_data = VisionInit(&huart1); // 视觉 UART 暂未启用
  vision_recv_data = VisionInit(NULL);

  gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
  gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
  shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
  shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
  chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
  chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  CANComm_Init_Config_s comm_conf = {
      .can_config =
          {
              .can_handle = &hfdcan1,
              .tx_id = 0x312,
              .rx_id = 0x311,
          },
      .recv_data_len = sizeof(Chassis_Upload_Data_s),
      .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
  };
  cmd_can_comm = CANCommInit(&comm_conf);
#endif                                         // GIMBAL_BOARD
  gimbal_cmd_send.pitch = PITCH_HORIZON_ANGLE; // 初始化为水平位置(0度)
  robot_state =
      ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle() {
  // 保持 hero 之前已验证的“编码器单圈角 - 对齐角”逻辑；
  // YAW_CHASSIS_ALIGN_ECD 描述的是云台与底盘对齐时的编码器值，
  // 不是 yaw 电机机械零位，因此这里不能直接拿新的命令坐标零点替代。
  static float angle;
#if CHASSIS_OFFSET_ANGLE_FORCE_ZERO
  chassis_cmd_send.offset_angle = 0.0f;
  return;
#endif
  if ((gimbal_fetch_data.motor_online_bitmap & 0x01u) == 0u) {
    // Yaw 电机离线时，退化为底盘坐标系控制，避免使用失效的云台偏角。
    chassis_cmd_send.offset_angle = 0.0f;
    return;
  }
  angle = gimbal_fetch_data
              .yaw_motor_single_round_angle;
#if YAW_ECD_GREATER_THAN_4096
  if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle > 180.0f + YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
  else
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else
  if (angle > YAW_ALIGN_ANGLE)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
  else
    chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void DBUSRemoteControlSet() {
  // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
  // 注意：左侧开关为[中]时（视觉模式），不设置gimbal_mode，避免覆盖视觉控制逻辑
  uint8_t is_vision_mode = switch_is_mid(rc_data[TEMP].rc.switch_left);
  
  if (switch_is_down(
          rc_data[TEMP].rc.switch_right)) // 右侧开关状态[下],底盘跟随云台
  {
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    if (!is_vision_mode) {
      gimbal_cmd_send.gimbal_mode =
          GimbalModeFromChassisMode(chassis_cmd_send.chassis_mode);
    }
  } else if (
      switch_is_mid(
          rc_data[TEMP]
              .rc
              .switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
  {
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    if (!is_vision_mode) {
      gimbal_cmd_send.gimbal_mode =
          GimbalModeFromChassisMode(chassis_cmd_send.chassis_mode);
    }
  }
  // 右侧开关[上]：不设置gimbal_mode和chassis_mode（与main分支行为一致）

  // 云台参数,确定云台控制数据
  // 左侧开关状态为[中]时启用视觉控制(在RobotCMDTask主函数中处理,此处不设置云台参数)
  // 左侧开关状态为[下]时,纯遥控器拨杆控制
  if (switch_is_down(
          rc_data[TEMP]
              .rc
              .switch_left) &&
      CanUpdateGimbalTargets()) { // 按照摇杆的输出大小进行角度增量,增益系数需调整
    gimbal_cmd_send.yaw += RcYawDeltaDeg((float)rc_data[TEMP].rc.rocker_l_);
    gimbal_cmd_send.pitch += RcPitchDeltaDeg((float)rc_data[TEMP].rc.rocker_l1);
    // 云台软件限位
    LimitPitchAngle();
    LimitYawAngle();
  }
  // 左侧[中]时不在此设置云台参数，完全由视觉控制逻辑处理

  // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
  chassis_cmd_send.vx =
      CHASSIS_RC_MOVE_RATIO_X * (float)CHASSIS_RC_VX_INPUT_SIGN *
      RcChassisForwardInput();
  chassis_cmd_send.vy =
      CHASSIS_RC_MOVE_RATIO_Y * (float)CHASSIS_RC_VY_INPUT_SIGN *
      RcChassisLateralInput();

  // 发射参数
  if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
    ; // 弹舱舵机控制,待添加servo_motor模块,开启
  else
    ; // 弹舱舵机控制,待添加servo_motor模块,关闭

  // 摩擦轮控制,拨轮向上打为负,向下为正
  if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
    shoot_cmd_send.friction_mode = FRICTION_ON;
  else
    shoot_cmd_send.friction_mode = FRICTION_OFF;
  // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
  if (rc_data[TEMP].rc.dial < -500)
    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
  else
    shoot_cmd_send.load_mode = LOAD_STOP;
  // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
  shoot_cmd_send.shoot_rate = 8;
}

static void VideoTransmitterRemoteControlSet() {
  uint8_t is_vision_mode = IsVisionControlEnabled();

  if (switch_is_down(rc_data[TEMP].rc.switch_right)) {
    chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    if (!is_vision_mode) {
      gimbal_cmd_send.gimbal_mode =
          GimbalModeFromChassisMode(chassis_cmd_send.chassis_mode);
    }
  } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) {
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    if (!is_vision_mode) {
      gimbal_cmd_send.gimbal_mode =
          GimbalModeFromChassisMode(chassis_cmd_send.chassis_mode);
    }
  } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    chassis_cmd_send.chassis_mode = DefaultManualChassisMode();
    if (!is_vision_mode) {
      gimbal_cmd_send.gimbal_mode =
          GimbalModeFromChassisMode(chassis_cmd_send.chassis_mode);
    }
  }

  if (!is_vision_mode && CanUpdateGimbalTargets()) {
    gimbal_cmd_send.yaw += RcYawDeltaDeg((float)rc_data[TEMP].rc.rocker_l_);
    gimbal_cmd_send.pitch += RcPitchDeltaDeg((float)rc_data[TEMP].rc.rocker_l1);
    LimitPitchAngle();
    LimitYawAngle();
  }

  chassis_cmd_send.vx =
      CHASSIS_RC_MOVE_RATIO_X *
      (float)CHASSIS_RC_VX_INPUT_SIGN * RcChassisForwardInput();
  chassis_cmd_send.vy =
      CHASSIS_RC_MOVE_RATIO_Y *
      (float)CHASSIS_RC_VY_INPUT_SIGN * RcChassisLateralInput();
  chassis_cmd_send.wz = GetVideoTransmitterDialWz();
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet() {
  float keyboard_move_speed_scale = GetKeyboardMoveSpeedScale();
  float keyboard_aim_scale = GetKeyboardAimScale();
  float keyboard_base_vx =
      CHASSIS_RC_MOVE_RATIO_X * (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
  float keyboard_base_vy =
      CHASSIS_RC_MOVE_RATIO_Y * (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);
  chassis_cmd_send.vx =
      (float)CHASSIS_KB_VX_INPUT_SIGN *
      KeyboardChassisForwardInput() *
      keyboard_base_vx * keyboard_move_speed_scale;
  chassis_cmd_send.vy =
      (float)CHASSIS_KB_VY_INPUT_SIGN *
      KeyboardChassisLateralInput() *
      keyboard_base_vy * keyboard_move_speed_scale;

  if (IsVideoTransmitterRemote()) {
    chassis_cmd_send.chassis_mode = vtm_keyboard_chassis_mode;
    if (!IsVisionControlEnabled()) {
      gimbal_cmd_send.gimbal_mode =
          GimbalModeFromChassisMode(vtm_keyboard_chassis_mode);
    }
  }

  if (CanUpdateGimbalTargets() && !IsVisionControlEnabled()) {
    gimbal_cmd_send.yaw +=
        MouseYawDeltaDeg((float)rc_data[TEMP].mouse.x) * keyboard_aim_scale;
    gimbal_cmd_send.pitch +=
        MousePitchDeltaDeg((float)rc_data[TEMP].mouse.y) * keyboard_aim_scale;
    LimitPitchAngle();
    LimitYawAngle();
  }

  if (IsVideoTransmitterRemote()) {
    return;
  }
  #ifdef KB_ZE_DISABLE
      // Z/E 逻辑禁用，兜底由 ShootTask/默认值提供
  #else
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
      shoot_cmd_send.bullet_speed = 15;
      break;
    case 1:
      shoot_cmd_send.bullet_speed = 18;
      break;
    default:
      shoot_cmd_send.bullet_speed = 30;
      break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
      shoot_cmd_send.load_mode = LOAD_STOP;
      break;
    case 1:
      shoot_cmd_send.load_mode = LOAD_1_BULLET;
      break;
    case 2:
      shoot_cmd_send.load_mode = LOAD_3_BULLET;
      break;
    default:
      shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
      break;
    }
  #endif
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
  {
  case 0:
    shoot_cmd_send.lid_mode = LID_OPEN;
    break;
  default:
    shoot_cmd_send.lid_mode = LID_CLOSE;
    break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
  {
  case 0:
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    break;
  default:
    shoot_cmd_send.friction_mode = FRICTION_ON;
    break;
  }
  switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
  {
  case 0:
    chassis_cmd_send.chassis_speed_buff = 40;
    break;
  case 1:
    chassis_cmd_send.chassis_speed_buff = 60;
    break;
  case 2:
    chassis_cmd_send.chassis_speed_buff = 80;
    break;
  default:
    chassis_cmd_send.chassis_speed_buff = 100;
    break;
  }
  switch (rc_data[TEMP]
              .key[KEY_PRESS]
              .shift) // 待添加 按shift允许超功率 消耗缓冲能量
  {
  case 1:
#if defined(CHASSIS_POWER_CONTROL_ENABLED)
    chassis_cmd_send.power_burst_enabled = 1u;
#endif
    break;

  default:
    chassis_cmd_send.power_burst_enabled = 0u;
    break;
  }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo
 * 后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler() {
  uint8_t emergency_request = 0u;

  if (IsVideoTransmitterRemote()) {
    emergency_request = rc_data[TEMP].vtm.custom_left;
  } else {
    // 拨轮的向下拨超过一半进入急停模式.注意向下打拨轮是正
    emergency_request = (uint8_t)(rc_data[TEMP].rc.dial > 300);
  }

  if (emergency_request || robot_state == ROBOT_STOP) {
    if (robot_state != ROBOT_STOP) {
      LOGERROR("[CMD] emergency stop!");
      LEDSetStatus(LED_STATUS_RED_ON);
    }
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    // 急停期间始终把角度目标同步到当前姿态，避免恢复时追旧目标而疯转。
    SyncGimbalCmdToCurrentPose();
  }

  if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
    if (robot_state != ROBOT_READY) {
      LOGINFO("[CMD] reinstate, robot ready");
      LEDSetStatus(LED_STATUS_GREEN_ON);
    }
    robot_state = ROBOT_READY;
  }

  if (robot_state == ROBOT_READY) {
    shoot_cmd_send.shoot_mode = SHOOT_ON;
  }
}

static uint8_t IsRefereeResetGameStatus(uint8_t status) {
  return (uint8_t)(status >= 3u && status <= 5u);
}

static uint8_t ShouldClearCommandsOnRefereeStatusSwitch(void) {
  const referee_info_t *referee_data = RefereeGetData();
  uint8_t current_status;
  uint8_t switched;

  if (referee_data == NULL || !referee_data->init_flag || !RefereeIsOnline()) {
    return 0u;
  }

  current_status = (uint8_t)(referee_data->GameState.game_progress & 0x0Fu);

  if (last_referee_game_status == 0xFFu) {
    last_referee_game_status = current_status;
    return 0u;
  }

  switched = (uint8_t)(current_status != last_referee_game_status);
  if (switched &&
      (IsRefereeResetGameStatus(current_status) ||
       IsRefereeResetGameStatus(last_referee_game_status))) {
    last_referee_game_status = current_status;
    return 1u;
  }

  last_referee_game_status = current_status;
  return 0u;
}

static void ClearAllCommandsOnce(void) {
  chassis_cmd_send.vx = 0.0f;
  chassis_cmd_send.vy = 0.0f;
  chassis_cmd_send.wz = 0.0f;
  chassis_cmd_send.offset_angle = 0.0f;
  chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
  chassis_cmd_send.power_burst_enabled = 0u;

  gimbal_cmd_send.chassis_rotate_wz = 0.0f;
  gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
  SyncGimbalCmdToCurrentPose();

  shoot_cmd_send.shoot_mode = SHOOT_OFF;
  shoot_cmd_send.load_mode = LOAD_STOP;
  shoot_cmd_send.friction_mode = FRICTION_OFF;
  shoot_cmd_send.shoot_rate = 0.0f;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask() {
  static uint32_t vision_send_tick = 0;
  static Fire_Mode_e vision_last_fire_mode = NO_FIRE;
  static uint8_t first_run = 1;
  uint8_t vision_enabled;
  uint8_t keyboard_mouse_enabled;
  uint8_t vision_tracking_mode = 0u;
  uint8_t gimbal_msg_updated;
  uint8_t allow_vision_auto_fire =
      (uint8_t)(!IsVideoTransmitterRemote());
  // BMI088Acquire(bmi088_test,&bmi088_data) ;
  // 从其他应用获取回传数据
#ifdef ONE_BOARD
  SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
  SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
  gimbal_msg_updated = SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

  // 第一次收到云台反馈时，用当前姿态初始化 gimbal_cmd_send，避免启动时追 0 度。
  if (first_run && gimbal_msg_updated) {
    first_run = 0;
    SyncGimbalCmdToCurrentPose();
  }

  // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
  CalcOffsetAngle();
  chassis_cmd_send.wz = 0.0f;
  chassis_cmd_send.power_burst_enabled = 0u;
  if (IsVideoTransmitterRemote()) {
    UpdateVideoTransmitterControlState();
  }
  vision_enabled = IsVisionControlEnabled();
  keyboard_mouse_enabled = IsKeyboardMouseControlEnabled();

  if (IsVideoTransmitterRemote()) {
    if (keyboard_mouse_enabled) {
      MouseKeySet();
    } else {
      VideoTransmitterRemoteControlSet();
    }
  } else if (switch_is_down(rc_data[TEMP].rc.switch_left) ||
             switch_is_mid(rc_data[TEMP].rc.switch_left)) {
    DBUSRemoteControlSet();
  } else if (switch_is_up(
                 rc_data[TEMP]
                     .rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
    MouseKeySet();

  EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

  // 视觉控制逻辑 - 处理SP协议的控制指令
  if (robot_state != ROBOT_STOP && vision_enabled && CanUpdateGimbalTargets()) {
    if (vision_recv_data->new_data) {
      vision_recv_data->new_data = 0; // 清除新数据标志
      vision_last_fire_mode = vision_recv_data->fire_mode;

      switch (vision_recv_data->fire_mode) {
      case AUTO_AIM: // mode=1 仅控制云台
      {
        float vision_yaw_deg = FiniteOrZero(vision_recv_data->yaw);
        float vision_yaw_vel_deg_s = vision_recv_data->yaw_vel_rad_s * RAD_2_DEGREE;
        float vision_yaw_acc_deg_s2 = vision_recv_data->yaw_acc_rad_s2 * RAD_2_DEGREE;
        float vision_pitch_vel_deg_s = vision_recv_data->pitch_vel_rad_s * RAD_2_DEGREE;
        float vision_pitch_acc_deg_s2 = vision_recv_data->pitch_acc_rad_s2 * RAD_2_DEGREE;
        float target_yaw = UnwrapVisionYawToGimbalTotal(
            vision_yaw_deg, gimbal_fetch_data.yaw_actual_angle);
#if VISION_YAW_DEBUG_LOG
        LOGINFO("[VISION] mode=AIM raw_yaw=%.2f gimbal_yaw=%.2f target_yaw=%.2f",
                vision_recv_data->yaw, vision_yaw_deg, target_yaw);
#endif
        float target_pitch = FiniteOrZero(vision_recv_data->pitch);
#if VISION_PITCH_DEBUG_LOG
        LOGINFO("[VISION] mode=AIM raw_pitch=%.2f gimbal_pitch=%.2f target_pitch=%.2f",
                vision_recv_data->pitch,
                vision_recv_data->pitch, target_pitch);
#endif
        UpdateVisionTargetFilter(target_yaw, target_pitch, vision_yaw_vel_deg_s,
                                 vision_pitch_vel_deg_s, vision_yaw_acc_deg_s2,
                                 vision_pitch_acc_deg_s2);
        vision_tracking_mode = 1u;
        break;
      }

      case AUTO_FIRE: // mode=2 控制云台+开火
      {
        float vision_yaw_deg = FiniteOrZero(vision_recv_data->yaw);
        float vision_yaw_vel_deg_s = vision_recv_data->yaw_vel_rad_s * RAD_2_DEGREE;
        float vision_yaw_acc_deg_s2 = vision_recv_data->yaw_acc_rad_s2 * RAD_2_DEGREE;
        float vision_pitch_vel_deg_s = vision_recv_data->pitch_vel_rad_s * RAD_2_DEGREE;
        float vision_pitch_acc_deg_s2 = vision_recv_data->pitch_acc_rad_s2 * RAD_2_DEGREE;
        float target_yaw = UnwrapVisionYawToGimbalTotal(
            vision_yaw_deg, gimbal_fetch_data.yaw_actual_angle);
#if VISION_YAW_DEBUG_LOG
        LOGINFO("[VISION] mode=FIRE raw_yaw=%.2f gimbal_yaw=%.2f target_yaw=%.2f",
                vision_recv_data->yaw, vision_yaw_deg, target_yaw);
#endif
        float target_pitch = FiniteOrZero(vision_recv_data->pitch);
#if VISION_PITCH_DEBUG_LOG
        LOGINFO("[VISION] mode=FIRE raw_pitch=%.2f gimbal_pitch=%.2f target_pitch=%.2f",
                vision_recv_data->pitch,
                vision_recv_data->pitch, target_pitch);
#endif
        UpdateVisionTargetFilter(target_yaw, target_pitch, vision_yaw_vel_deg_s,
                                 vision_pitch_vel_deg_s, vision_yaw_acc_deg_s2,
                                 vision_pitch_acc_deg_s2);
        vision_tracking_mode = 1u;
        break;
      }

      case NO_FIRE:
      default:
        ApplyVisionNoFireHold();
        break;
      }
    } else {
      if (vision_last_fire_mode == AUTO_AIM || vision_last_fire_mode == AUTO_FIRE) {
        vision_tracking_mode = 1u;
  } else {
    // 上一次是 NO_FIRE 或从未收到数据：保持当前姿态
    ApplyVisionNoFireHold();
  }
}

    if (vision_tracking_mode) {
      gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
      ApplyVisionCmdSlew(VISION_CTRL_DT_S);
      gimbal_cmd_send.yaw = vision_yaw_cmd_filtered;
      gimbal_cmd_send.pitch = vision_pitch_cmd_filtered;
      LimitPitchAngle();
      LimitYawAngle();

      if (vision_last_fire_mode == AUTO_FIRE) {
        if (allow_vision_auto_fire) {
          shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
          shoot_cmd_send.friction_mode = FRICTION_ON;
        } else {
          shoot_cmd_send.load_mode = LOAD_STOP;
        }
      } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
      }
    }
  } else {
    vision_cmd_filter_inited = 0;
    vision_yaw_vel_filtered_deg_s = 0.0f;
    vision_pitch_vel_filtered_deg_s = 0.0f;
    vision_yaw_acc_filtered_deg_s2 = 0.0f;
    vision_pitch_acc_filtered_deg_s2 = 0.0f;
    vision_last_fire_mode = NO_FIRE;
  }

  if (IsVideoTransmitterRemote()) {
    ApplyVideoTransmitterShootControl();
  }
  HandleManualBodyLockModeTransition();

  if (ShouldClearCommandsOnRefereeStatusSwitch()) {
    ClearAllCommandsOnce();
    LOGINFO("[CMD] clear commands once on referee GAME_STATUS switch to %u",
            (unsigned int)last_referee_game_status);
  }

  UpdateRobotCMDDebugState(vision_enabled, keyboard_mouse_enabled);

  // 设置视觉发送数据,还需增加加速度和角速度数据
  VisionSetFlag(chassis_fetch_data.enemy_color, VISION_MODE_AIM,
                chassis_fetch_data.bullet_speed);
  VisionSetAltitude(
      gimbal_fetch_data.yaw_actual_angle,
      gimbal_fetch_data.pitch_actual_angle,
      ImuAxisToGimbalDeg(gimbal_fetch_data.gimbal_imu_data.Roll, GYRO2GIMBAL_DIR_ROLL));
  vision_send_tick++;
  if (vision_send_tick >= VISION_SEND_DIV) {
    vision_send_tick = 0;
    VisionSend();
  }

  // 推送消息,双板通信,视觉通信等
  // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
  PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
  CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
  PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
  PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}

void RobotCMDGetDebugState(RobotCMDDebugState_s *out) {
  if (out != NULL) {
    *out = robot_cmd_debug_state;
  }
}
