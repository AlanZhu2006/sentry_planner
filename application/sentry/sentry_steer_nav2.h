#ifndef SENTRY_STEER_NAV2_H
#define SENTRY_STEER_NAV2_H

#include <stdint.h>

#define SENTRY_STEER_NAV2_MOTOR_COUNT 4u

/**
 * @brief Runtime state of the USB Nav2 swerve controller.
 *
 * Command units follow the wire protocol: mm/s for translation and mrad/s
 * for yaw rate. Motor order is LF, RF, LR, RR.
 */
typedef struct
{
    int16_t command_vx_mm_s;
    int16_t command_vy_mm_s;
    int16_t command_wz_mrad_s;
    int16_t drive_target_rpm[SENTRY_STEER_NAV2_MOTOR_COUNT];
    uint16_t current_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT];
    uint16_t target_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT];
    int16_t error_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT];
    uint8_t steer_online_bitmap;
    uint8_t drive_online_bitmap;
    uint8_t aligned_bitmap;
    uint8_t control_enabled;
} SentrySteerNav2State_s;

/**
 * @brief Snapshot published for the RTT dashboard.
 *
 * Motor order is LF, RF, LR, RR. Angles and angular speeds use degrees,
 * drive speeds use logical motor RPM, and current values use DJI raw units.
 */
typedef struct
{
    int16_t command_vx_mm_s;
    int16_t command_vy_mm_s;
    int16_t command_wz_mrad_s;
    float drive_target_rpm[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float drive_actual_rpm[SENTRY_STEER_NAV2_MOTOR_COUNT];
    uint16_t steer_target_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT];
    uint16_t steer_actual_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT];
    int16_t steer_error_ecd[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float steer_target_angle_deg[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float steer_actual_angle_deg[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float steer_target_speed_deg_s[SENTRY_STEER_NAV2_MOTOR_COUNT];
    float steer_actual_speed_deg_s[SENTRY_STEER_NAV2_MOTOR_COUNT];
    int16_t steer_output[SENTRY_STEER_NAV2_MOTOR_COUNT];
    int16_t steer_current[SENTRY_STEER_NAV2_MOTOR_COUNT];
    int16_t drive_output[SENTRY_STEER_NAV2_MOTOR_COUNT];
    int16_t drive_current[SENTRY_STEER_NAV2_MOTOR_COUNT];
    uint16_t drive_scale_permille;
    uint8_t steer_online_bitmap;
    uint8_t drive_online_bitmap;
    uint8_t aligned_bitmap;
    uint8_t flip_bitmap;
    uint8_t control_enabled;
    uint8_t command_fresh;
} SentrySteerNav2Telemetry_s;

extern volatile SentrySteerNav2State_s sentry_steer_nav2_state;

/** Register the four steering motors, four drive motors, and USB command link. */
void SentrySteerNav2Init(void);

/** Decode current motion command, apply swerve kinematics, and enforce safety. */
void SentrySteerNav2Task(void);

#endif // SENTRY_STEER_NAV2_H
