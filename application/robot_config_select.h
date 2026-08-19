#pragma once
#ifndef ROBOT_CONFIG_SELECT_H
#define ROBOT_CONFIG_SELECT_H

#if defined(ROBOT_TYPE_infantry)
#include "robot_configs/robot_infantry.h"
#elif defined(ROBOT_TYPE_damiao)
#include "robot_configs/robot_damiao.h"
#elif defined(ROBOT_TYPE_hero)
#include "robot_configs/robot_hero.h"
#elif defined(ROBOT_TYPE_sentry)
#include "robot_configs/robot_sentry.h"
#elif defined(ROBOT_TYPE_sentry_swerve)
#include "robot_configs/robot_sentry_swerve.h"
#else
#error "Unknown or missing ROBOT_TYPE_*. Use ROBOT_TYPE=infantry|damiao|hero|sentry|sentry_swerve"
#endif

#endif // ROBOT_CONFIG_SELECT_H
