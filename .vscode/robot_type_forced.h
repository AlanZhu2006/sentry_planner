#pragma once

/*
 * Keep IntelliSense aligned with the local `.robot_type` selection.
 * `just sync-vscode-robot` or any `just build/flash` command refreshes
 * `.vscode/robot_type.local.h`. When no local override exists, fall back
 * to the repository default robot.
 */
#if defined(__has_include) && __has_include("robot_type.local.h")
#include "robot_type.local.h"
#else
#define ROBOT_TYPE_infantry
#endif
