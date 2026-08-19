#ifndef SENTRY_STEER_KEYBOARD_TEST_H
#define SENTRY_STEER_KEYBOARD_TEST_H

#include <stdint.h>

#define SENTRY_STEER_KEYBOARD_MOTOR_COUNT 4u

/** Runtime state of the USB WASD swerve-drive test. */
typedef struct
{
    uint16_t current_ecd[SENTRY_STEER_KEYBOARD_MOTOR_COUNT];
    uint16_t target_ecd[SENTRY_STEER_KEYBOARD_MOTOR_COUNT];
    int16_t error_ecd[SENTRY_STEER_KEYBOARD_MOTOR_COUNT];
    uint8_t online_bitmap;
    uint8_t drive_online_bitmap;
    uint8_t aligned_bitmap;
    uint8_t control_enabled;
    uint8_t drive_enabled;
    uint8_t command;
} SentrySteerKeyboardState_s;

extern volatile SentrySteerKeyboardState_s sentry_steer_keyboard_state;

/** Register four GM6020 and four stopped M3508 motors plus the USB keyboard callback. */
void SentrySteerKeyboardTestInit(void);

/** Apply WASD steering targets and drive only after all steering motors are aligned. */
void SentrySteerKeyboardTestTask(void);

#endif // SENTRY_STEER_KEYBOARD_TEST_H
