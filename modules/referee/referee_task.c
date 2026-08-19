/**
 * @file referee_task.c
 * @brief Referee UI task for operator-facing status display.
 */
#include "referee_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "message_center.h"
#include "referee_UI.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "rm_referee.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef REFEREE_UI_SELFTEST
#define REFEREE_UI_SELFTEST 0
#endif

#define UI_FIELD_COUNT 10u
#define UI_VALUE_TEXT_WIDTH 12u
#define UI_TEXT_SIZE 15u
#define UI_TEXT_STROKE 2u
#define UI_VALUE_LAYER 8u
#define UI_LABEL_LAYER 7u

#define UI_POWER_BAR_START_X 900u
#define UI_POWER_BAR_END_X 1340u
#define UI_POWER_BAR_Y 530u
#define UI_POWER_BAR_WIDTH 18u
#define UI_POWER_BOX_TOP_Y 510u
#define UI_POWER_BOX_BOTTOM_Y 550u
#define UI_POWER_UPDATE_INTERVAL_MS 200u
#define UI_BOOTSTRAP_REDRAW_COUNT 3u
#define UI_BOOTSTRAP_REDRAW_INTERVAL_MS 1500u
#define UI_PERIODIC_REDRAW_INTERVAL_MS 10000u

typedef enum
{
    UI_FIELD_CTRL = 0,
    UI_FIELD_VISION,
    UI_FIELD_CHASSIS,
    UI_FIELD_SHOOTER,
    UI_FIELD_FIRE,
    UI_FIELD_LID,
    UI_FIELD_BSPD,
    UI_FIELD_HEAT,
    UI_FIELD_POWER,
    UI_FIELD_ESTOP,
} UI_Field_e;

typedef struct
{
    uint16_t label_x;
    uint16_t value_x;
    uint16_t y;
} UI_Field_Layout_s;

typedef struct
{
    Chassis_Ctrl_Cmd_s chassis_cmd;
    Gimbal_Ctrl_Cmd_s gimbal_cmd;
    Shoot_Ctrl_Cmd_s shoot_cmd;
    Shoot_Upload_Data_s shoot_feed;
    RobotCMDDebugState_s cmd_debug;
} Referee_UI_Runtime_s;

static const char ui_label_names[UI_FIELD_COUNT][4] = {
    "l00", "l01", "l02", "l03", "l04",
    "l05", "l06", "l07", "l08", "l09",
};
static const char ui_value_names[UI_FIELD_COUNT][4] = {
    "d00", "d01", "d02", "d03", "d04",
    "d05", "d06", "d07", "d08", "d09",
};
static const char *const ui_label_text[UI_FIELD_COUNT] = {
    "CTRL:", "VISION:", "CHASSIS:", "SHOOTER:", "FIRE:",
    "LID:", "BSPD:", "HEAT:", "POWER:", "ESTOP:",
};
static const UI_Field_Layout_s ui_field_layout[UI_FIELD_COUNT] = {
    {120u, 300u, 760u},
    {120u, 300u, 715u},
    {120u, 300u, 670u},
    {120u, 300u, 625u},
    {120u, 300u, 580u},
    {900u, 1080u, 760u},
    {900u, 1080u, 715u},
    {900u, 1080u, 670u},
    {900u, 1080u, 625u},
    {900u, 1080u, 580u},
};
static const uint32_t ui_label_color[UI_FIELD_COUNT] = {
    UI_Color_White, UI_Color_White, UI_Color_White, UI_Color_White, UI_Color_White,
    UI_Color_White, UI_Color_White, UI_Color_White, UI_Color_White, UI_Color_White,
};

static referee_info_t *referee_recv_info;
static Subscriber_t *ui_chassis_cmd_sub;
static Subscriber_t *ui_gimbal_cmd_sub;
static Subscriber_t *ui_shoot_cmd_sub;
static Subscriber_t *ui_shoot_feed_sub;
static Referee_UI_Runtime_s ui_runtime_state;
static uint8_t ui_runtime_initialized;

uint8_t UI_Seq;

static Graph_Data_t ui_shoot_line[5];
static Graph_Data_t ui_power_graph[2];
static String_Data_t ui_state_label[UI_FIELD_COUNT];
static String_Data_t ui_state_value[UI_FIELD_COUNT];
static char ui_value_cache[UI_FIELD_COUNT][30];
static uint32_t ui_color_cache[UI_FIELD_COUNT];
static uint16_t ui_power_bar_end_x;
static uint32_t ui_power_bar_color;
static uint32_t ui_power_last_refresh_ms;
static uint32_t ui_last_full_redraw_ms;
static uint8_t ui_bootstrap_redraw_count;
static uint16_t ui_last_robot_id;

static void DeterminRobotID(void)
{
    referee_recv_info->referee_id.Robot_Color =
        referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID =
        0x0100 + referee_recv_info->referee_id.Robot_ID;
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void PadUIValue(char *out, size_t out_size, const char *value)
{
    snprintf(out, out_size, "%-12s", value);
}

static const char *ChassisModeText(chassis_mode_e mode)
{
    switch (mode)
    {
    case CHASSIS_ROTATE:
        return "ROTATE";
    case CHASSIS_NO_FOLLOW:
        return "NOFOLLOW";
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        return "FOLLOW";
    case CHASSIS_ZERO_FORCE:
    default:
        return "ZERO";
    }
}

static const char *ShooterStateText(const Referee_UI_Runtime_s *state)
{
    float loader_speed_abs = fabsf(state->shoot_feed.loader_speed_aps);

    if (state->cmd_debug.reverse_request_active ||
        state->shoot_cmd.load_mode == LOAD_REVERSE)
    {
        return "REVERSE";
    }

    if (state->shoot_cmd.load_mode != LOAD_STOP &&
        (state->cmd_debug.fire_request_active || loader_speed_abs > 1.0f))
    {
        return "FIRING";
    }

    if (state->cmd_debug.shooter_ready ||
        state->shoot_cmd.friction_mode == FRICTION_ON)
    {
        return "READY";
    }

    return "SAFE";
}

static const char *FireSelectText(shoot_fire_select_e select)
{
    switch (select)
    {
    case FIRE_SELECT_SINGLE:
        return "1";
    case FIRE_SELECT_TRIPLE:
        return "3";
    case FIRE_SELECT_AUTO:
    default:
        return "AUTO";
    }
}

static const char *LidModeText(lid_mode_e mode)
{
    return (mode == LID_OPEN) ? "OPEN" : "CLOSE";
}

static const char *BulletSpeedText(Bullet_Speed_e speed)
{
    switch (speed)
    {
    case SMALL_AMU_15:
        return "15";
    case SMALL_AMU_18:
        return "18";
    case SMALL_AMU_30:
        return "30";
    default:
        return "--";
    }
}

static uint32_t VisionFieldColor(const Referee_UI_Runtime_s *state)
{
    if (!state->cmd_debug.vision_enabled)
    {
        return UI_Color_White;
    }

    return state->gimbal_cmd.gimbal_mode == GIMBAL_GYRO_MODE ? UI_Color_Cyan
                                                              : UI_Color_Yellow;
}

static uint32_t ShooterFieldColor(const char *shooter_state)
{
    if (strcmp(shooter_state, "REVERSE") == 0)
    {
        return UI_Color_Yellow;
    }
    if (strcmp(shooter_state, "FIRING") == 0)
    {
        return UI_Color_Pink;
    }
    if (strcmp(shooter_state, "READY") == 0)
    {
        return UI_Color_Green;
    }
    return UI_Color_Orange;
}

static uint32_t HeatFieldColor(uint16_t current, uint16_t limit)
{
    if (limit == 0u)
    {
        return UI_Color_White;
    }

    if ((uint32_t)current * 10u >= (uint32_t)limit * 9u)
    {
        return UI_Color_Pink;
    }
    if ((uint32_t)current * 10u >= (uint32_t)limit * 7u)
    {
        return UI_Color_Orange;
    }
    return UI_Color_Green;
}

static uint32_t PowerFieldColor(float power, float limit)
{
    if (limit <= 0.0f)
    {
        return UI_Color_White;
    }

    if (power >= limit * 0.9f)
    {
        return UI_Color_Pink;
    }
    if (power >= limit * 0.7f)
    {
        return UI_Color_Orange;
    }
    return UI_Color_Green;
}

static void CollectRuntimeState(void)
{
    if (ui_chassis_cmd_sub != NULL)
    {
        SubGetMessage(ui_chassis_cmd_sub, &ui_runtime_state.chassis_cmd);
    }
    if (ui_gimbal_cmd_sub != NULL)
    {
        SubGetMessage(ui_gimbal_cmd_sub, &ui_runtime_state.gimbal_cmd);
    }
    if (ui_shoot_cmd_sub != NULL)
    {
        SubGetMessage(ui_shoot_cmd_sub, &ui_runtime_state.shoot_cmd);
    }
    if (ui_shoot_feed_sub != NULL)
    {
        SubGetMessage(ui_shoot_feed_sub, &ui_runtime_state.shoot_feed);
    }

    RobotCMDGetDebugState(&ui_runtime_state.cmd_debug);
    ui_runtime_initialized = 1u;
}

static void DrawValueField(UI_Field_e field,
                           uint32_t graph_operate,
                           uint32_t color,
                           const char *value)
{
    char padded_value[30] = {0};

    PadUIValue(padded_value, sizeof(padded_value), value);
    UICharDraw(&ui_state_value[field], (char *)ui_value_names[field], graph_operate,
               UI_VALUE_LAYER, color, UI_TEXT_SIZE, UI_TEXT_STROKE,
               ui_field_layout[field].value_x, ui_field_layout[field].y, "%s",
               padded_value);
    UICharRefresh(&referee_recv_info->referee_id, ui_state_value[field]);

    strncpy(ui_value_cache[field], padded_value, sizeof(ui_value_cache[field]) - 1u);
    ui_value_cache[field][sizeof(ui_value_cache[field]) - 1u] = '\0';
    ui_color_cache[field] = color;
}

static void UpdateValueField(UI_Field_e field, uint32_t color, const char *value)
{
    char padded_value[30] = {0};

    PadUIValue(padded_value, sizeof(padded_value), value);
    if (strcmp(ui_value_cache[field], padded_value) == 0 &&
        ui_color_cache[field] == color)
    {
        return;
    }

    UICharDraw(&ui_state_value[field], (char *)ui_value_names[field], UI_Graph_Change,
               UI_VALUE_LAYER, color, UI_TEXT_SIZE, UI_TEXT_STROKE,
               ui_field_layout[field].value_x, ui_field_layout[field].y, "%s",
               padded_value);
    UICharRefresh(&referee_recv_info->referee_id, ui_state_value[field]);

    strncpy(ui_value_cache[field], padded_value, sizeof(ui_value_cache[field]) - 1u);
    ui_value_cache[field][sizeof(ui_value_cache[field]) - 1u] = '\0';
    ui_color_cache[field] = color;
}

static void DrawFieldValues(uint32_t graph_operate)
{
    char heat_text[20] = {0};
    char power_text[20] = {0};
    const char *shooter_state;
    uint16_t current_heat;
    uint16_t heat_limit;
    float chassis_power;

    shooter_state = ShooterStateText(&ui_runtime_state);
    current_heat = referee_recv_info->PowerHeatData.shooter_17mm_1_barrel_heat;
    heat_limit = referee_recv_info->GameRobotState.shooter_barrel_heat_limit;
    chassis_power = referee_recv_info->PowerHeatData.chassis_power;

    snprintf(heat_text, sizeof(heat_text), "%u/%u",
             (unsigned int)current_heat, (unsigned int)heat_limit);
    snprintf(power_text, sizeof(power_text), "%.0fW", chassis_power);

    if (graph_operate == UI_Graph_ADD)
    {
        DrawValueField(UI_FIELD_CTRL,
                       graph_operate,
                       ui_runtime_state.cmd_debug.keyboard_mouse_enabled
                           ? UI_Color_Cyan
                           : UI_Color_White,
                       ui_runtime_state.cmd_debug.keyboard_mouse_enabled ? "KM" : "RC");
        DrawValueField(UI_FIELD_VISION,
                       graph_operate,
                       VisionFieldColor(&ui_runtime_state),
                       ui_runtime_state.cmd_debug.vision_enabled ? "ON" : "OFF");
        DrawValueField(UI_FIELD_CHASSIS, graph_operate, UI_Color_Main,
                       ChassisModeText(ui_runtime_state.chassis_cmd.chassis_mode));
        DrawValueField(UI_FIELD_SHOOTER, graph_operate,
                       ShooterFieldColor(shooter_state), shooter_state);
        DrawValueField(UI_FIELD_FIRE, graph_operate, UI_Color_White,
                       FireSelectText(ui_runtime_state.cmd_debug.fire_select));
        DrawValueField(UI_FIELD_LID, graph_operate, UI_Color_White,
                       LidModeText(ui_runtime_state.shoot_cmd.lid_mode));
        DrawValueField(UI_FIELD_BSPD, graph_operate, UI_Color_White,
                       BulletSpeedText(ui_runtime_state.shoot_cmd.bullet_speed));
        DrawValueField(UI_FIELD_HEAT, graph_operate,
                       HeatFieldColor(current_heat, heat_limit), heat_text);
        DrawValueField(UI_FIELD_POWER, graph_operate,
                       PowerFieldColor(
                           chassis_power,
                           (float)referee_recv_info->GameRobotState.chassis_power_limit),
                       power_text);
        DrawValueField(UI_FIELD_ESTOP, graph_operate,
                       ui_runtime_state.cmd_debug.emergency_latched
                           ? UI_Color_Pink
                           : UI_Color_Green,
                       ui_runtime_state.cmd_debug.emergency_latched ? "ON" : "OFF");
        return;
    }

    UpdateValueField(UI_FIELD_CTRL,
                     ui_runtime_state.cmd_debug.keyboard_mouse_enabled
                         ? UI_Color_Cyan
                         : UI_Color_White,
                     ui_runtime_state.cmd_debug.keyboard_mouse_enabled ? "KM" : "RC");
    UpdateValueField(UI_FIELD_VISION, VisionFieldColor(&ui_runtime_state),
                     ui_runtime_state.cmd_debug.vision_enabled ? "ON" : "OFF");
    UpdateValueField(UI_FIELD_CHASSIS, UI_Color_Main,
                     ChassisModeText(ui_runtime_state.chassis_cmd.chassis_mode));
    UpdateValueField(UI_FIELD_SHOOTER, ShooterFieldColor(shooter_state),
                     shooter_state);
    UpdateValueField(UI_FIELD_FIRE, UI_Color_White,
                     FireSelectText(ui_runtime_state.cmd_debug.fire_select));
    UpdateValueField(UI_FIELD_LID, UI_Color_White,
                     LidModeText(ui_runtime_state.shoot_cmd.lid_mode));
    UpdateValueField(UI_FIELD_BSPD, UI_Color_White,
                     BulletSpeedText(ui_runtime_state.shoot_cmd.bullet_speed));
    UpdateValueField(UI_FIELD_HEAT,
                     HeatFieldColor(current_heat, heat_limit), heat_text);
    UpdateValueField(UI_FIELD_POWER,
                     PowerFieldColor(
                         chassis_power,
                         (float)referee_recv_info->GameRobotState.chassis_power_limit),
                     power_text);
    UpdateValueField(UI_FIELD_ESTOP,
                     ui_runtime_state.cmd_debug.emergency_latched
                         ? UI_Color_Pink
                         : UI_Color_Green,
                     ui_runtime_state.cmd_debug.emergency_latched ? "ON" : "OFF");
}

static void RefreshPowerBar(uint32_t graph_operate)
{
    float power_limit = (float)referee_recv_info->GameRobotState.chassis_power_limit;
    float chassis_power = referee_recv_info->PowerHeatData.chassis_power;
    float ratio;
    uint16_t bar_end_x;
    uint32_t bar_color;
    uint32_t now_ms = HAL_GetTick();

    if (graph_operate == UI_Graph_Change &&
        (now_ms - ui_power_last_refresh_ms) < UI_POWER_UPDATE_INTERVAL_MS)
    {
        return;
    }

    if (power_limit <= 0.0f)
    {
        power_limit = 100.0f;
    }

    if (chassis_power < 0.0f)
    {
        chassis_power = 0.0f;
    }

    ratio = chassis_power / power_limit;
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    bar_end_x = (uint16_t)(UI_POWER_BAR_START_X +
                           ratio * (float)(UI_POWER_BAR_END_X - UI_POWER_BAR_START_X));
    bar_color = PowerFieldColor(chassis_power, power_limit);

    if (graph_operate == UI_Graph_Change &&
        bar_end_x == ui_power_bar_end_x &&
        bar_color == ui_power_bar_color)
    {
        return;
    }

    UILineDraw(&ui_power_graph[1], "p02", graph_operate, UI_LABEL_LAYER, bar_color,
               UI_POWER_BAR_WIDTH, UI_POWER_BAR_START_X, UI_POWER_BAR_Y, bar_end_x,
               UI_POWER_BAR_Y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, ui_power_graph[1]);

    ui_power_bar_end_x = bar_end_x;
    ui_power_bar_color = bar_color;
    ui_power_last_refresh_ms = now_ms;
}

static void MyUIRefresh(void)
{
    DrawFieldValues(UI_Graph_Change);
    RefreshPowerBar(UI_Graph_Change);
}

static void RedrawFullUI(uint8_t delete_first)
{
    uint32_t i;

    if (delete_first)
    {
        UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);
    }

    UILineDraw(&ui_shoot_line[0], "c00", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White,
               3, 710, 540, 1210, 540);
    UILineDraw(&ui_shoot_line[1], "c01", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_White,
               3, 960, 340, 960, 740);
    UILineDraw(&ui_shoot_line[2], "c02", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_Yellow,
               2, 810, 490, 1110, 490);
    UILineDraw(&ui_shoot_line[3], "c03", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_Yellow,
               2, 810, 515, 1110, 515);
    UILineDraw(&ui_shoot_line[4], "c04", UI_Graph_ADD, UI_LABEL_LAYER, UI_Color_Yellow,
               2, 810, 565, 1110, 565);
    UIGraphRefresh(&referee_recv_info->referee_id, 5,
                   ui_shoot_line[0], ui_shoot_line[1], ui_shoot_line[2],
                   ui_shoot_line[3], ui_shoot_line[4]);

    for (i = 0; i < UI_FIELD_COUNT; ++i)
    {
        UICharDraw(&ui_state_label[i], (char *)ui_label_names[i], UI_Graph_ADD,
                   UI_LABEL_LAYER, ui_label_color[i], UI_TEXT_SIZE, UI_TEXT_STROKE,
                   ui_field_layout[i].label_x, ui_field_layout[i].y,
                   (char *)ui_label_text[i]);
        UICharRefresh(&referee_recv_info->referee_id, ui_state_label[i]);
    }

    UIRectangleDraw(&ui_power_graph[0], "p01", UI_Graph_ADD, UI_LABEL_LAYER,
                    UI_Color_Green, 2, UI_POWER_BAR_START_X, UI_POWER_BOX_TOP_Y,
                    UI_POWER_BAR_END_X, UI_POWER_BOX_BOTTOM_Y);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, ui_power_graph[0]);

    DrawFieldValues(UI_Graph_ADD);
    RefreshPowerBar(UI_Graph_ADD);
    ui_last_full_redraw_ms = HAL_GetTick();
    ui_bootstrap_redraw_count++;
}

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle)
{
    uint32_t i;

    referee_recv_info = RefereeInit(referee_usart_handle);
    ui_chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    ui_gimbal_cmd_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    ui_shoot_cmd_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    ui_shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    memset(&ui_runtime_state, 0, sizeof(ui_runtime_state));
    memset(ui_value_cache, 0, sizeof(ui_value_cache));
    for (i = 0; i < UI_FIELD_COUNT; ++i)
    {
        ui_color_cache[i] = 0xFFFFFFFFu;
    }
    ui_power_bar_end_x = 0u;
    ui_power_bar_color = 0xFFFFFFFFu;
    ui_power_last_refresh_ms = 0u;
    ui_last_full_redraw_ms = 0u;
    ui_bootstrap_redraw_count = 0u;
    ui_last_robot_id = 0u;
    ui_runtime_initialized = 0u;

    referee_recv_info->init_flag = 1u;
    return referee_recv_info;
}

referee_info_t *RefereeGetData(void)
{
    return referee_recv_info;
}

void UITask(void)
{
    if (referee_recv_info == NULL || !referee_recv_info->init_flag)
    {
        return;
    }

    CollectRuntimeState();
#if REFEREE_UI_SELFTEST
#error "REFEREE_UI_SELFTEST is not implemented for the production HUD."
#endif
    if (referee_recv_info->GameRobotState.robot_id != ui_last_robot_id)
    {
        DeterminRobotID();
        ui_last_robot_id = referee_recv_info->GameRobotState.robot_id;
        ui_bootstrap_redraw_count = 0u;
    }

    if (ui_bootstrap_redraw_count < UI_BOOTSTRAP_REDRAW_COUNT &&
        (HAL_GetTick() - ui_last_full_redraw_ms) >= UI_BOOTSTRAP_REDRAW_INTERVAL_MS)
    {
        RedrawFullUI(1u);
        return;
    }

    if ((HAL_GetTick() - ui_last_full_redraw_ms) >= UI_PERIODIC_REDRAW_INTERVAL_MS)
    {
        RedrawFullUI(1u);
        return;
    }

    MyUIRefresh();
}

void MyUIInit(void)
{
    if (referee_recv_info == NULL || !referee_recv_info->init_flag)
    {
        vTaskDelete(NULL);
    }

    while (referee_recv_info->GameRobotState.robot_id == 0u)
    {
        osDelay(100);
    }

    DeterminRobotID();
    ui_last_robot_id = referee_recv_info->GameRobotState.robot_id;

    CollectRuntimeState();
    if (!ui_runtime_initialized)
    {
        memset(&ui_runtime_state, 0, sizeof(ui_runtime_state));
    }

    RedrawFullUI(1u);
}
