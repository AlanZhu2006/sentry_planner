#pragma once

#include <stdint.h>

#define DASHBOARD_RTT_CHANNEL 1u
#define DASHBOARD_RTT_BUFFER_SIZE 2048u

#define DASHBOARD_FRAME_MAGIC 0x4452u
#define DASHBOARD_FRAME_VERSION 8u

#define DASHBOARD_SWERVE_VALID 0x01u
#define DASHBOARD_SWERVE_CONTROL_ENABLED 0x02u
#define DASHBOARD_SWERVE_COMMAND_FRESH 0x04u

#pragma pack(1)
typedef struct
{
    uint32_t timestamp_ms;
    float chassis_power;
    float chassis_volt;
    uint16_t chassis_power_buffer_energy;
    float chassis_power_budget_w;
    float chassis_estimated_power_w;
    float chassis_power_scale;
    uint8_t chassis_power_mode_flags;
    float motor_target_rpm[4];
    float motor_rpm[4];
    float imu_angle_deg[3];
    float imu_gyro_deg_s[3];
    float gimbal_yaw_target_deg;
    float gimbal_yaw_actual_deg;
    float gimbal_yaw_target_deg_s;
    float gimbal_yaw_actual_deg_s;
    float gimbal_pitch_target_deg;
    float gimbal_pitch_actual_deg;
    float gimbal_pitch_target_deg_s;
    float gimbal_pitch_actual_deg_s;
    float gimbal_cmd_yaw_deg;
    float gimbal_cmd_pitch_deg;
    float gimbal_cmd_chassis_rotate_wz;
    uint16_t gimbal_yaw_encoder_raw;   // yaw 原始编码器值(DJI ECD)
    uint16_t gimbal_pitch_encoder_raw; // pitch 原始编码器值/位置码
    uint8_t referee_game_state; // bit[3:0]=game_type, bit[7:4]=game_progress
    uint16_t referee_stage_remain_time;
    uint8_t referee_robot_id;
    uint8_t referee_robot_level;
    uint16_t referee_current_hp;
    uint16_t referee_maximum_hp;
    uint16_t referee_shooter_barrel_cooling_value;
    uint16_t referee_shooter_barrel_heat_limit;
    uint16_t referee_chassis_power_limit;
    uint8_t referee_power_management_flags; // bit0: gimbal, bit1: chassis, bit2: shooter
    uint8_t remote_packed; // bit[3:0]=rc_switches, bit[5:4]=remote_protocol, bit[7:6]=mode_switch(3=N/A)
    uint8_t motor_online_bitmap;
    uint8_t link_bitmap_packed; // bit[1:0]=gimbal, bit[4:2]=shoot, bit[6:5]=can
    uint8_t status_flags;
    uint8_t remote_state_flags; // bit[4:0]=buttons, bit[7:5]=control flags(vision/km/estop)
    int16_t rc_rocker_l_x;
    int16_t rc_rocker_l_y;
    int16_t rc_rocker_r_x;
    int16_t rc_rocker_r_y;
    int16_t rc_dial;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_buttons; // bit0: left, bit1: right
    uint16_t key_pressed_bits;
    uint8_t mode_packed_low; // bit[1:0]=chassis, bit[3:2]=gimbal, bit4=shoot, bit[7:5]=load
    uint8_t mode_packed_high; // bit0=lid, bit1=friction, bit[4:2]=bullet_speed_code
    uint8_t shoot_rest_heat;
    uint16_t referee_chassis_current;
    uint16_t referee_buffer_energy;
    uint16_t referee_shooter_17mm_1_barrel_heat;
    uint16_t referee_shooter_17mm_2_barrel_heat;
    uint16_t referee_shooter_42mm_barrel_heat;
    float referee_shoot_bullet_speed_mps;
    uint32_t telemetry_drop_count;
    uint32_t free_heap_bytes;
    float chassis_cmd_vx;
    float chassis_cmd_vy;
    float chassis_cmd_wz;
    float shoot_rate;
    float shoot_loader_speed_aps;
    float shoot_friction_l_speed_aps;
    float shoot_friction_r_speed_aps;
    uint8_t vision_meta_flags; // bit0: recv new_data latch, bit1: recv raw snapshot valid, bit2: send raw snapshot valid
    uint8_t vision_sp_mode_packed; // bit[3:0]=recv mode, bit[7:4]=send mode
    float vision_recv_yaw_raw_rad;
    float vision_recv_yaw_vel_raw_rad_s;
    float vision_recv_yaw_acc_raw_rad_s2;
    float vision_recv_pitch_raw_rad;
    float vision_recv_pitch_vel_raw_rad_s;
    float vision_recv_pitch_acc_raw_rad_s2;
    float vision_send_q[4];
    float vision_send_yaw_raw_rad;
    float vision_send_yaw_vel_raw_rad_s;
    float vision_send_pitch_raw_rad;
    float vision_send_pitch_vel_raw_rad_s;
    float vision_send_bullet_speed_mps;
    uint16_t vision_send_bullet_count;
    uint8_t swerve_flags;
    uint8_t swerve_steer_online_bitmap;
    uint8_t swerve_drive_online_bitmap;
    uint8_t swerve_aligned_bitmap;
    uint8_t swerve_flip_bitmap;
    uint16_t swerve_drive_scale_permille;
    uint16_t swerve_steer_target_ecd[4];
    uint16_t swerve_steer_actual_ecd[4];
    int16_t swerve_steer_error_ecd[4];
    float swerve_steer_target_angle_deg[4];
    float swerve_steer_actual_angle_deg[4];
    float swerve_steer_target_speed_deg_s[4];
    float swerve_steer_actual_speed_deg_s[4];
    int16_t swerve_steer_output[4];
    int16_t swerve_steer_current[4];
    int16_t swerve_drive_output[4];
    int16_t swerve_drive_current[4];
} Dashboard_Payload_t;

typedef struct
{
    uint16_t magic;
    uint8_t version;
    uint8_t payload_len;
    uint32_t seq;
    Dashboard_Payload_t payload;
    uint16_t crc16;
} Dashboard_Frame_t;
#pragma pack()

void DashboardInit(void);
void DashboardTask(void);
