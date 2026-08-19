#include "dashboard.h"

#include "main.h"
#include "fdcan.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "message_center.h"
#include "referee_task.h"
#include "remote_control.h"
#include "master_process.h"
#include "general_def.h"
#include "crc16.h"
#include "SEGGER_RTT.h"
#include "FreeRTOS.h"

#include <string.h>

_Static_assert(sizeof(Dashboard_Payload_t) == 415u, "Dashboard payload size mismatch");
_Static_assert(sizeof(Dashboard_Frame_t) == 425u, "Dashboard frame size mismatch");
_Static_assert(DASHBOARD_RTT_BUFFER_SIZE > (sizeof(Dashboard_Frame_t) * 4u),
               "Dashboard RTT buffer should hold multiple whole frames");

#if defined(ROBOT_TYPE_sentry_swerve) && SENTRY_STEER_NAV2_CONTROL
#include "sentry_steer_nav2.h"
#define DASHBOARD_SWERVE_NAV2 1
static Subscriber_t *dashboard_swerve_sub;
static SentrySteerNav2Telemetry_s dashboard_swerve_data;
static uint8_t dashboard_swerve_seen;
#else
#define DASHBOARD_SWERVE_NAV2 0
#endif

#if defined(ONE_BOARD) && !DASHBOARD_SWERVE_NAV2
static Subscriber_t *dashboard_chassis_sub;
static Subscriber_t *dashboard_gimbal_sub;
static Subscriber_t *dashboard_chassis_cmd_sub;
static Subscriber_t *dashboard_gimbal_cmd_sub;
static Subscriber_t *dashboard_shoot_cmd_sub;
static Subscriber_t *dashboard_shoot_sub;
static Chassis_Upload_Data_s dashboard_chassis_data;
static Gimbal_Upload_Data_s dashboard_gimbal_data;
static Chassis_Ctrl_Cmd_s dashboard_chassis_cmd;
static Gimbal_Ctrl_Cmd_s dashboard_gimbal_cmd;
static Shoot_Ctrl_Cmd_s dashboard_shoot_cmd;
static Shoot_Upload_Data_s dashboard_shoot_data;
#endif

static uint8_t dashboard_inited;
static uint32_t dashboard_seq;
static uint32_t dashboard_drop_count;

#if !DASHBOARD_SWERVE_NAV2
#ifndef MOTOR_DIRECTION_NORMAL
#define MOTOR_DIRECTION_NORMAL 0
#endif
#ifndef MOTOR_DIRECTION_REVERSE
#define MOTOR_DIRECTION_REVERSE 1
#endif
#ifndef SHOOT_FRICTION_CAN_BUS
#define SHOOT_FRICTION_CAN_BUS SHOOT_CAN_BUS
#endif
#ifndef SHOOT_LOADER_CAN_BUS
#define SHOOT_LOADER_CAN_BUS SHOOT_CAN_BUS
#endif

static float DashboardFiniteOrZero(float v)
{
    return (v == v) ? v : 0.0f;
}

static uint8_t DashboardPackRemoteControlFlags(const RobotCMDDebugState_s *cmd_debug)
{
    uint8_t flags = 0u;

    if (cmd_debug == NULL)
    {
        return 0u;
    }

    flags |= (uint8_t)(cmd_debug->vision_enabled ? 0x01u : 0u);
    flags |= (uint8_t)(cmd_debug->keyboard_mouse_enabled ? 0x02u : 0u);
    flags |= (uint8_t)(cmd_debug->emergency_latched ? 0x04u : 0u);
    return flags;
}

static void DashboardMarkCanOnline(FDCAN_HandleTypeDef *can_handle, uint8_t *bitmap)
{
    if (can_handle == &hfdcan1)
    {
        *bitmap |= 0x01u;
    }
    else if (can_handle == &hfdcan2)
    {
        *bitmap |= 0x02u;
    }
}

static uint8_t DashboardEncodeRemoteModeSwitch(uint8_t raw_mode)
{
    switch (raw_mode)
    {
    case 0u:
    case 1u:
    case 2u:
        return raw_mode;
    default:
        return 3u;
    }
}

static uint8_t DashboardEncodeBulletSpeed(Bullet_Speed_e speed)
{
    switch (speed)
    {
    case BIG_AMU_10:
        return 1u;
    case SMALL_AMU_15:
        return 2u;
    case BIG_AMU_16:
        return 3u;
    case SMALL_AMU_18:
        return 4u;
    case SMALL_AMU_30:
        return 5u;
    case BULLET_SPEED_NONE:
    default:
        return 0u;
    }
}
#endif

static void DashboardCollectPayload(Dashboard_Payload_t *payload)
{
#if DASHBOARD_SWERVE_NAV2
    const uint8_t updated = SubGetMessage(dashboard_swerve_sub, &dashboard_swerve_data);
    uint8_t can_link_bitmap = 0u;

    if (updated)
        dashboard_swerve_seen = 1u;

    memset(payload, 0, sizeof(*payload));
    payload->timestamp_ms = HAL_GetTick();
    payload->chassis_power_scale = 1.0f;
    payload->telemetry_drop_count = dashboard_drop_count;
    payload->free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();

    if (!dashboard_swerve_seen)
        return;

    payload->chassis_cmd_vx = (float)dashboard_swerve_data.command_vx_mm_s;
    payload->chassis_cmd_vy = (float)dashboard_swerve_data.command_vy_mm_s;
    payload->chassis_cmd_wz = (float)dashboard_swerve_data.command_wz_mrad_s;
    memcpy(payload->motor_target_rpm,
           dashboard_swerve_data.drive_target_rpm,
           sizeof(payload->motor_target_rpm));
    memcpy(payload->motor_rpm,
           dashboard_swerve_data.drive_actual_rpm,
           sizeof(payload->motor_rpm));
    payload->motor_online_bitmap = dashboard_swerve_data.drive_online_bitmap;
    payload->status_flags = updated ? 0x10u : 0u;

    if ((dashboard_swerve_data.steer_online_bitmap |
         dashboard_swerve_data.drive_online_bitmap) & 0x05u)
        can_link_bitmap |= 0x01u;
    if ((dashboard_swerve_data.steer_online_bitmap |
         dashboard_swerve_data.drive_online_bitmap) & 0x0au)
        can_link_bitmap |= 0x02u;
    payload->link_bitmap_packed = (uint8_t)((can_link_bitmap & 0x03u) << 5);

    payload->swerve_flags = DASHBOARD_SWERVE_VALID;
    if (dashboard_swerve_data.control_enabled)
        payload->swerve_flags |= DASHBOARD_SWERVE_CONTROL_ENABLED;
    if (dashboard_swerve_data.command_fresh)
        payload->swerve_flags |= DASHBOARD_SWERVE_COMMAND_FRESH;
    payload->swerve_steer_online_bitmap = dashboard_swerve_data.steer_online_bitmap;
    payload->swerve_drive_online_bitmap = dashboard_swerve_data.drive_online_bitmap;
    payload->swerve_aligned_bitmap = dashboard_swerve_data.aligned_bitmap;
    payload->swerve_flip_bitmap = dashboard_swerve_data.flip_bitmap;
    payload->swerve_drive_scale_permille = dashboard_swerve_data.drive_scale_permille;
    memcpy(payload->swerve_steer_target_ecd,
           dashboard_swerve_data.steer_target_ecd,
           sizeof(payload->swerve_steer_target_ecd));
    memcpy(payload->swerve_steer_actual_ecd,
           dashboard_swerve_data.steer_actual_ecd,
           sizeof(payload->swerve_steer_actual_ecd));
    memcpy(payload->swerve_steer_error_ecd,
           dashboard_swerve_data.steer_error_ecd,
           sizeof(payload->swerve_steer_error_ecd));
    memcpy(payload->swerve_steer_target_angle_deg,
           dashboard_swerve_data.steer_target_angle_deg,
           sizeof(payload->swerve_steer_target_angle_deg));
    memcpy(payload->swerve_steer_actual_angle_deg,
           dashboard_swerve_data.steer_actual_angle_deg,
           sizeof(payload->swerve_steer_actual_angle_deg));
    memcpy(payload->swerve_steer_target_speed_deg_s,
           dashboard_swerve_data.steer_target_speed_deg_s,
           sizeof(payload->swerve_steer_target_speed_deg_s));
    memcpy(payload->swerve_steer_actual_speed_deg_s,
           dashboard_swerve_data.steer_actual_speed_deg_s,
           sizeof(payload->swerve_steer_actual_speed_deg_s));
    memcpy(payload->swerve_steer_output,
           dashboard_swerve_data.steer_output,
           sizeof(payload->swerve_steer_output));
    memcpy(payload->swerve_steer_current,
           dashboard_swerve_data.steer_current,
           sizeof(payload->swerve_steer_current));
    memcpy(payload->swerve_drive_output,
           dashboard_swerve_data.drive_output,
           sizeof(payload->swerve_drive_output));
    memcpy(payload->swerve_drive_current,
           dashboard_swerve_data.drive_current,
           sizeof(payload->swerve_drive_current));
#else
    const referee_info_t *referee_data = RefereeGetData();
    const RC_ctrl_t *rc_data = RemoteControlGetData();
    Vision_Recv_s vision_recv = {0};
    Vision_Send_s vision_send = {0};
    RobotCMDDebugState_s cmd_debug = {0};
    const uint8_t remote_protocol = (uint8_t)RemoteControlGetProtocol();
    uint8_t chassis_msg_updated = 0;
    uint8_t gimbal_msg_updated = 0;
    uint8_t chassis_cmd_updated = 0;
    uint8_t gimbal_cmd_updated = 0;
    uint8_t shoot_cmd_updated = 0;
    uint8_t shoot_msg_updated = 0;
    uint8_t status_flags = 0;
    uint8_t rc_switches = 0u;
    uint8_t remote_mode_code = 3u;
    uint8_t remote_button_bits = 0u;
    uint8_t gimbal_motor_online_bitmap = 0u;
    uint8_t shoot_motor_online_bitmap = 0u;
    uint8_t can_link_bitmap = 0u;
    uint8_t power_management_flags = 0u;
    uint8_t chassis_mode = 0u;
    uint8_t gimbal_mode = 0u;
    uint8_t shoot_mode = 0u;
    uint8_t shoot_load_mode = 0u;
    uint8_t shoot_lid_mode = 0u;
    uint8_t shoot_friction_mode = 0u;
    Bullet_Speed_e shoot_bullet_speed = BULLET_SPEED_NONE;

    memset(payload, 0, sizeof(*payload));
    payload->timestamp_ms = HAL_GetTick();
    RobotCMDGetDebugState(&cmd_debug);
    payload->remote_state_flags =
        (uint8_t)((DashboardPackRemoteControlFlags(&cmd_debug) & 0x07u) << 5);

#if defined(ONE_BOARD)
    chassis_msg_updated = SubGetMessage(dashboard_chassis_sub, &dashboard_chassis_data);
    gimbal_msg_updated = SubGetMessage(dashboard_gimbal_sub, &dashboard_gimbal_data);
    chassis_cmd_updated = SubGetMessage(dashboard_chassis_cmd_sub, &dashboard_chassis_cmd);
    gimbal_cmd_updated = SubGetMessage(dashboard_gimbal_cmd_sub, &dashboard_gimbal_cmd);
    shoot_cmd_updated = SubGetMessage(dashboard_shoot_cmd_sub, &dashboard_shoot_cmd);
    shoot_msg_updated = SubGetMessage(dashboard_shoot_sub, &dashboard_shoot_data);
    (void)shoot_cmd_updated;

    payload->chassis_cmd_vx = dashboard_chassis_cmd.vx;
    payload->chassis_cmd_vy = dashboard_chassis_cmd.vy;
    payload->chassis_cmd_wz = dashboard_chassis_cmd.wz;
    chassis_mode = (uint8_t)dashboard_chassis_cmd.chassis_mode;
    gimbal_mode = (uint8_t)dashboard_gimbal_cmd.gimbal_mode;
    payload->gimbal_cmd_yaw_deg = DashboardFiniteOrZero(dashboard_gimbal_cmd.yaw);
    payload->gimbal_cmd_pitch_deg = DashboardFiniteOrZero(dashboard_gimbal_cmd.pitch);
    payload->gimbal_cmd_chassis_rotate_wz = dashboard_gimbal_cmd.chassis_rotate_wz;
    shoot_mode = (uint8_t)dashboard_shoot_cmd.shoot_mode;
    shoot_load_mode = (uint8_t)dashboard_shoot_cmd.load_mode;
    shoot_lid_mode = (uint8_t)dashboard_shoot_cmd.lid_mode;
    shoot_friction_mode = (uint8_t)dashboard_shoot_cmd.friction_mode;
    shoot_bullet_speed = dashboard_shoot_cmd.bullet_speed;
    payload->shoot_rest_heat = dashboard_shoot_cmd.rest_heat;
    payload->shoot_rate = dashboard_shoot_cmd.shoot_rate;

    payload->chassis_power_buffer_energy = dashboard_chassis_data.power_buffer_energy;
    payload->chassis_power_budget_w = dashboard_chassis_data.power_budget_w;
    payload->chassis_estimated_power_w = dashboard_chassis_data.estimated_power_w;
    payload->chassis_power_scale = dashboard_chassis_data.power_scale;
    payload->chassis_power_mode_flags = dashboard_chassis_data.power_mode_flags;
    payload->motor_target_rpm[0] = dashboard_chassis_data.motor_target_rpm[0];
    payload->motor_target_rpm[1] = dashboard_chassis_data.motor_target_rpm[1];
    payload->motor_target_rpm[2] = dashboard_chassis_data.motor_target_rpm[2];
    payload->motor_target_rpm[3] = dashboard_chassis_data.motor_target_rpm[3];
    payload->motor_rpm[0] = dashboard_chassis_data.motor_rpm[0];
    payload->motor_rpm[1] = dashboard_chassis_data.motor_rpm[1];
    payload->motor_rpm[2] = dashboard_chassis_data.motor_rpm[2];
    payload->motor_rpm[3] = dashboard_chassis_data.motor_rpm[3];
    payload->motor_online_bitmap = dashboard_chassis_data.motor_online_bitmap;
    gimbal_motor_online_bitmap = dashboard_gimbal_data.motor_online_bitmap;
    shoot_motor_online_bitmap = dashboard_shoot_data.motor_online_bitmap;
    payload->shoot_loader_speed_aps = dashboard_shoot_data.loader_speed_aps;
    payload->shoot_friction_l_speed_aps = dashboard_shoot_data.friction_l_speed_aps;
    payload->shoot_friction_r_speed_aps = dashboard_shoot_data.friction_r_speed_aps;

    payload->imu_angle_deg[0] = dashboard_gimbal_data.gimbal_imu_data.Pitch;
    payload->imu_angle_deg[1] = dashboard_gimbal_data.gimbal_imu_data.Yaw;
    payload->imu_angle_deg[2] = dashboard_gimbal_data.gimbal_imu_data.Roll;
    payload->imu_gyro_deg_s[0] = dashboard_gimbal_data.gimbal_imu_data.Gyro[0] * RAD_2_DEGREE;
    payload->imu_gyro_deg_s[1] = dashboard_gimbal_data.gimbal_imu_data.Gyro[1] * RAD_2_DEGREE;
    payload->imu_gyro_deg_s[2] = dashboard_gimbal_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE;

    payload->gimbal_yaw_target_deg = DashboardFiniteOrZero(dashboard_gimbal_data.yaw_target_angle);
    payload->gimbal_yaw_actual_deg = DashboardFiniteOrZero(dashboard_gimbal_data.yaw_actual_angle);
    // gimbal feed 已经统一为 deg/s，dashboard 直接使用即可。
    payload->gimbal_yaw_target_deg_s =
        DashboardFiniteOrZero(dashboard_gimbal_data.yaw_target_speed);
    payload->gimbal_yaw_actual_deg_s =
        DashboardFiniteOrZero(dashboard_gimbal_data.yaw_actual_speed);
    payload->gimbal_pitch_target_deg = DashboardFiniteOrZero(dashboard_gimbal_data.pitch_target_angle);
    payload->gimbal_pitch_actual_deg = DashboardFiniteOrZero(dashboard_gimbal_data.pitch_actual_angle);
    payload->gimbal_pitch_target_deg_s =
        DashboardFiniteOrZero(dashboard_gimbal_data.pitch_target_speed);
    payload->gimbal_pitch_actual_deg_s =
        DashboardFiniteOrZero(dashboard_gimbal_data.pitch_actual_speed);
    payload->gimbal_yaw_encoder_raw = dashboard_gimbal_data.yaw_motor_encoder_raw;
    payload->gimbal_pitch_encoder_raw = dashboard_gimbal_data.pitch_motor_encoder_raw;

    if (!(payload->imu_angle_deg[0] == payload->imu_angle_deg[0]) ||
        !(payload->imu_angle_deg[1] == payload->imu_angle_deg[1]) ||
        !(payload->imu_angle_deg[2] == payload->imu_angle_deg[2]))
    {
        payload->imu_angle_deg[0] = 0.0f;
        payload->imu_angle_deg[1] = 0.0f;
        payload->imu_angle_deg[2] = 0.0f;
        payload->imu_gyro_deg_s[0] = 0.0f;
        payload->imu_gyro_deg_s[1] = 0.0f;
        payload->imu_gyro_deg_s[2] = 0.0f;
    }
    if (gimbal_msg_updated)
    {
        status_flags |= 0x04u; // imu online
    }
    status_flags |= (uint8_t)(gimbal_msg_updated ? 0x08u : 0u);
    status_flags |= (uint8_t)(chassis_msg_updated ? 0x10u : 0u);
    status_flags |= (uint8_t)(gimbal_cmd_updated ? 0x20u : 0u);
    status_flags |= (uint8_t)(chassis_cmd_updated ? 0x40u : 0u);
    status_flags |= (uint8_t)(shoot_msg_updated ? 0x80u : 0u);
#endif

    if (referee_data != NULL)
    {
        payload->chassis_power = referee_data->PowerHeatData.chassis_power;
        payload->chassis_volt = 0.001f * (float)referee_data->PowerHeatData.chassis_voltage;
        payload->referee_game_state = (uint8_t)(((referee_data->GameState.game_progress & 0x0Fu) << 4) |
                                                (referee_data->GameState.game_type & 0x0Fu));
        payload->referee_stage_remain_time = referee_data->GameState.stage_remain_time;
        payload->referee_robot_id = referee_data->GameRobotState.robot_id;
        payload->referee_robot_level = referee_data->GameRobotState.robot_level;
        payload->referee_current_hp = referee_data->GameRobotState.current_HP;
        payload->referee_maximum_hp = referee_data->GameRobotState.maximum_HP;
        payload->referee_shooter_barrel_cooling_value =
            referee_data->GameRobotState.shooter_barrel_cooling_value;
        payload->referee_shooter_barrel_heat_limit =
            referee_data->GameRobotState.shooter_barrel_heat_limit;
        payload->referee_chassis_power_limit = referee_data->GameRobotState.chassis_power_limit;
        power_management_flags = (uint8_t)(
            (referee_data->GameRobotState.power_management_gimbal_output ? 0x01u : 0u) |
            (referee_data->GameRobotState.power_management_chassis_output ? 0x02u : 0u) |
            (referee_data->GameRobotState.power_management_shooter_output ? 0x04u : 0u));
        payload->referee_power_management_flags = power_management_flags;
        payload->referee_chassis_current = referee_data->PowerHeatData.chassis_current;
        payload->referee_buffer_energy = referee_data->PowerHeatData.buffer_energy;
        payload->referee_shooter_17mm_1_barrel_heat =
            referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
        payload->referee_shooter_17mm_2_barrel_heat =
            referee_data->PowerHeatData.shooter_17mm_2_barrel_heat;
        payload->referee_shooter_42mm_barrel_heat =
            referee_data->PowerHeatData.shooter_42mm_barrel_heat;
        payload->referee_shoot_bullet_speed_mps =
            DashboardFiniteOrZero(referee_data->ShootData.bullet_speed);
        status_flags |= (uint8_t)(RefereeIsOnline() ? 0x01u : 0u);
    }

    if (rc_data != NULL)
    {
        rc_switches = (uint8_t)((rc_data->rc.switch_left & 0x03u) |
                                ((rc_data->rc.switch_right & 0x03u) << 2));
        payload->rc_rocker_l_x = rc_data->rc.rocker_l_;
        payload->rc_rocker_l_y = rc_data->rc.rocker_l1;
        payload->rc_rocker_r_x = rc_data->rc.rocker_r_;
        payload->rc_rocker_r_y = rc_data->rc.rocker_r1;
        payload->rc_dial = rc_data->rc.dial;
        payload->mouse_x = rc_data->mouse.x;
        payload->mouse_y = rc_data->mouse.y;
        payload->mouse_z = rc_data->mouse.z;
        payload->mouse_buttons = (uint8_t)((rc_data->mouse.press_l ? 0x01u : 0u) |
                                           (rc_data->mouse.press_r ? 0x02u : 0u));
        payload->key_pressed_bits = rc_data->key[KEY_PRESS].keys;
        if (remote_protocol == 2u)
        {
            remote_mode_code = DashboardEncodeRemoteModeSwitch(rc_data->vtm.mode_switch);
        }
        remote_button_bits |= (uint8_t)(rc_data->vtm.pause ? 0x01u : 0u);
        remote_button_bits |= (uint8_t)(rc_data->vtm.custom_left ? 0x02u : 0u);
        remote_button_bits |= (uint8_t)(rc_data->vtm.custom_right ? 0x04u : 0u);
        remote_button_bits |= (uint8_t)(rc_data->vtm.trigger ? 0x08u : 0u);
        remote_button_bits |= (uint8_t)(rc_data->mouse.press_m ? 0x10u : 0u);
    }

    status_flags |= (uint8_t)(RemoteControlIsOnline() ? 0x02u : 0u);
    payload->status_flags = status_flags;
    payload->remote_state_flags |= (uint8_t)(remote_button_bits & 0x1Fu);
    payload->remote_packed = (uint8_t)((rc_switches & 0x0Fu) |
                                       ((remote_protocol & 0x03u) << 4) |
                                       ((remote_mode_code & 0x03u) << 6));

    can_link_bitmap = 0u;
    if (payload->motor_online_bitmap != 0u)
    {
        DashboardMarkCanOnline(&CHASSIS_CAN_BUS, &can_link_bitmap);
    }
    if (gimbal_motor_online_bitmap & 0x01u)
    {
        DashboardMarkCanOnline(&GIMBAL_YAW_CAN_BUS, &can_link_bitmap);
    }
    if (gimbal_motor_online_bitmap & 0x02u)
    {
        DashboardMarkCanOnline(&GIMBAL_PITCH_CAN_BUS, &can_link_bitmap);
    }
    if (shoot_motor_online_bitmap != 0u)
    {
        DashboardMarkCanOnline(&SHOOT_FRICTION_CAN_BUS, &can_link_bitmap);
        DashboardMarkCanOnline(&SHOOT_LOADER_CAN_BUS, &can_link_bitmap);
    }
    payload->link_bitmap_packed = (uint8_t)((gimbal_motor_online_bitmap & 0x03u) |
                                            ((shoot_motor_online_bitmap & 0x07u) << 2) |
                                            ((can_link_bitmap & 0x03u) << 5));
    payload->mode_packed_low = (uint8_t)((chassis_mode & 0x03u) |
                                         ((gimbal_mode & 0x03u) << 2) |
                                         ((shoot_mode & 0x01u) << 4) |
                                         ((shoot_load_mode & 0x07u) << 5));
    payload->mode_packed_high = (uint8_t)((shoot_lid_mode & 0x01u) |
                                          ((shoot_friction_mode & 0x01u) << 1) |
                                          ((DashboardEncodeBulletSpeed(shoot_bullet_speed) & 0x07u)
                                           << 2));

    payload->telemetry_drop_count = dashboard_drop_count;
    payload->free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();

    VisionGetRecvSnapshot(&vision_recv);
    VisionGetSendSnapshot(&vision_send);
    payload->vision_meta_flags = 0u;
    payload->vision_meta_flags |= (uint8_t)(vision_recv.new_data ? 0x01u : 0u);
    payload->vision_meta_flags |= 0x0002u; // recv raw snapshot fields are populated
    payload->vision_meta_flags |= 0x0004u; // send raw snapshot fields are populated
    payload->vision_sp_mode_packed = (uint8_t)((vision_recv.sp_mode & 0x0Fu) |
                                               ((vision_send.sp_mode & 0x0Fu) << 4));
    payload->vision_recv_yaw_raw_rad = vision_recv.yaw_rad;
    payload->vision_recv_yaw_vel_raw_rad_s = vision_recv.yaw_vel_rad_s;
    payload->vision_recv_yaw_acc_raw_rad_s2 = vision_recv.yaw_acc_rad_s2;
    payload->vision_recv_pitch_raw_rad = vision_recv.pitch_rad;
    payload->vision_recv_pitch_vel_raw_rad_s = vision_recv.pitch_vel_rad_s;
    payload->vision_recv_pitch_acc_raw_rad_s2 = vision_recv.pitch_acc_rad_s2;
    memcpy(payload->vision_send_q, vision_send.q, sizeof(payload->vision_send_q));
    payload->vision_send_yaw_raw_rad = vision_send.yaw_rad;
    payload->vision_send_yaw_vel_raw_rad_s = vision_send.yaw_vel_rad_s;
    payload->vision_send_pitch_raw_rad = vision_send.pitch_rad;
    payload->vision_send_pitch_vel_raw_rad_s = vision_send.pitch_vel_rad_s;
    payload->vision_send_bullet_speed_mps = vision_send.bullet_speed_mps;
    payload->vision_send_bullet_count = vision_send.bullet_count;
#endif
}

void DashboardInit(void)
{
    if (dashboard_inited)
    {
        return;
    }

#if DASHBOARD_SWERVE_NAV2
    dashboard_swerve_sub =
        SubRegister("sentry_steer_feed", sizeof(SentrySteerNav2Telemetry_s));
#elif defined(ONE_BOARD)
    dashboard_chassis_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    dashboard_gimbal_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    dashboard_chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    dashboard_gimbal_cmd_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    dashboard_shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    dashboard_shoot_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
#endif

    dashboard_inited = 1;
}

void DashboardTask(void)
{
    Dashboard_Frame_t frame;
    unsigned written;

    if (!dashboard_inited)
    {
        DashboardInit();
    }

    frame.magic = DASHBOARD_FRAME_MAGIC;
    frame.version = DASHBOARD_FRAME_VERSION;
    // payload_len stores the low 8 bits of payload size; host side selects full size by frame version.
    frame.payload_len = (uint8_t)sizeof(Dashboard_Payload_t);
    frame.seq = dashboard_seq++;
    DashboardCollectPayload(&frame.payload);
    frame.crc16 = crc_16((const uint8_t *)&frame, (uint16_t)(sizeof(frame) - sizeof(frame.crc16)));

    written = SEGGER_RTT_Write(DASHBOARD_RTT_CHANNEL, &frame, sizeof(frame));
    if (written != sizeof(frame))
    {
        dashboard_drop_count++;
    }
}
