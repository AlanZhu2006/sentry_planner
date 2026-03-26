/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "robot_def.h"
#include "crc16.h"
#include "ins_task.h"
#include "referee_task.h"
#include <string.h>
#include <math.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

#ifdef ROBOT_TYPE_sentry
_Static_assert(sizeof(SentryTelemetryFrame_s) ==
                   VISION_SENTRY_TELEMETRY_SEND_SIZE,
               "Sentry telemetry frame size mismatch");

#pragma pack(1)
typedef struct
{
    uint8_t head[2];
    float vx;
    float vy;
    float wz;
    float gimbal_yaw_delta;
    float gimbal_pitch_delta;
    uint8_t control_flags;
    float scan_yaw_rate_deg_s;
    float search_pitch_deg;
    uint16_t crc16;
} VisionToSentryExtFrame_s;
#pragma pack()

_Static_assert(sizeof(VisionToSentryExtFrame_s) ==
                   VISION_SENTRY_EXT_RECV_SIZE,
               "Sentry ext frame size mismatch");

static VisionToSentryExt_s sentry_ext_data;
static SentryTelemetry_s sentry_telemetry_data;

static void LogSentryExtPacket(const VisionToSentryExtFrame_s *packet)
{
    static uint32_t last_log_ms = 0u;
    static uint8_t last_flags = 0xFFu;
    static float last_scan_yaw_rate_deg_s = 0.0f;
    static float last_search_pitch_deg = 0.0f;
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    static float last_wz = 0.0f;
    uint32_t now_ms;
    uint8_t search_pitch_changed;
    uint8_t should_log = 0u;

    if (!packet)
    {
        return;
    }

    now_ms = HAL_GetTick();
    search_pitch_changed = (uint8_t)(
        (isnan(packet->search_pitch_deg) && !isnan(last_search_pitch_deg)) ||
        (!isnan(packet->search_pitch_deg) && isnan(last_search_pitch_deg)) ||
        (!isnan(packet->search_pitch_deg) && !isnan(last_search_pitch_deg) &&
         fabsf(packet->search_pitch_deg - last_search_pitch_deg) > 1e-3f));

    if (packet->control_flags != last_flags ||
        fabsf(packet->scan_yaw_rate_deg_s - last_scan_yaw_rate_deg_s) > 1e-3f ||
        search_pitch_changed ||
        fabsf(packet->vx - last_vx) > 1e-3f ||
        fabsf(packet->vy - last_vy) > 1e-3f ||
        fabsf(packet->wz - last_wz) > 1e-3f)
    {
        should_log = 1u;
    }
    else if ((now_ms - last_log_ms) >= 1000u)
    {
        should_log = 1u;
    }

    if (!should_log)
    {
        return;
    }

    LOGINFO("[vision][sx][rx] flags=0x%02X vx=%.3f vy=%.3f wz=%.3f scan_yaw=%.3f "
            "search_pitch=%.3f",
            packet->control_flags, packet->vx, packet->vy, packet->wz,
            packet->scan_yaw_rate_deg_s, packet->search_pitch_deg);

    last_log_ms = now_ms;
    last_flags = packet->control_flags;
    last_scan_yaw_rate_deg_s = packet->scan_yaw_rate_deg_s;
    last_search_pitch_deg = packet->search_pitch_deg;
    last_vx = packet->vx;
    last_vy = packet->vy;
    last_wz = packet->wz;
}

static uint8_t EncodeSentryRobotStatus(const referee_info_t *referee_data)
{
    uint8_t robot_status = 0u;

    if (referee_data == NULL)
    {
        return robot_status;
    }

    robot_status |= (uint8_t)(referee_data->GameRobotState.power_management_gimbal_output
                                  ? SENTRY_ROBOT_STATUS_GIMBAL_POWER_BIT
                                  : 0u);
    robot_status |= (uint8_t)(referee_data->GameRobotState.power_management_chassis_output
                                  ? SENTRY_ROBOT_STATUS_CHASSIS_POWER_BIT
                                  : 0u);
    robot_status |= (uint8_t)(referee_data->GameRobotState.power_management_shooter_output
                                  ? SENTRY_ROBOT_STATUS_SHOOTER_POWER_BIT
                                  : 0u);
    return robot_status;
}

static uint8_t EncodeSentryTeamColor(const referee_info_t *referee_data)
{
    if (referee_data == NULL || referee_data->GameRobotState.robot_id == 0u)
    {
        return 0u;
    }

    return (uint8_t)(referee_data->GameRobotState.robot_id > 7u ? 1u : 0u);
}

static uint8_t EncodeSentryAttackedFlag(const referee_info_t *referee_data)
{
    static uint8_t last_robot_id = 0u;
    static uint16_t last_hp = 0u;
    static float attacked_tick_ms = -1000.0f;
    float now_ms = DWT_GetTimeline_ms();

    if (referee_data == NULL || referee_data->GameRobotState.robot_id == 0u)
    {
        return 0u;
    }

    if (referee_data->GameRobotState.robot_id != last_robot_id)
    {
        last_robot_id = referee_data->GameRobotState.robot_id;
        last_hp = referee_data->GameRobotState.current_HP;
        attacked_tick_ms = -1000.0f;
        return 0u;
    }

    if (last_hp != 0u && referee_data->GameRobotState.current_HP < last_hp)
    {
        attacked_tick_ms = now_ms;
    }
    last_hp = referee_data->GameRobotState.current_HP;

    return (uint8_t)((now_ms - attacked_tick_ms) <= 500.0f);
}
#endif

// ==================== SP协议辅助函数 ====================

/**
 * @brief 在缓冲区中查找SP帧头
 * @param buffer 数据缓冲区
 * @param len 缓冲区长度
 * @return 帧头位置索引,未找到返回0xFF
 */
static uint8_t FindSPHeader(uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i < len - 1; i++)
    {
        if (buffer[i] == VISION_SP_HEADER_1 && buffer[i + 1] == VISION_SP_HEADER_2)
            return i;
    }
    return 0xFF;
}

#ifdef ROBOT_TYPE_sentry
static uint8_t FindSentryExtHeader(uint8_t *buffer, size_t len)
{
    for (size_t i = 0; i + 1u < len; i++)
    {
        if (buffer[i] == VISION_SENTRY_EXT_HEADER_1 &&
            buffer[i + 1u] == VISION_SENTRY_EXT_HEADER_2)
            return (uint8_t)i;
    }
    return 0xFFu;
}
#endif

/**
 * @brief 填充云台到视觉的数据包姿态字段（统一坐标语义）
 * @note q 和 yaw/pitch 使用同一套 send_data 欧拉角定义，避免字段语义不一致
 */
static void FillGimbalToVisionAttitude(GimbalToVision_s *packet)
{
    if (!packet)
        return;

    float q_send[4];
    float *gyro = INS_GetGyro();

    // send_data 的 yaw/pitch/roll 已在 RobotCMDTask 中映射到统一逻辑坐标
    // （+yaw=向右, +pitch=向上, 单位: deg）。
    // 这里统一由同一套欧拉角生成四元数，确保 q 与 yaw/pitch 完全同源。
    EularAngleToQuaternion(send_data.yaw, send_data.pitch, send_data.roll, q_send);
    memcpy(packet->q, q_send, sizeof(float) * 4);
    memcpy(send_data.q, q_send, sizeof(float) * 4);

    // 角度转换: 度 -> 弧度
    packet->yaw = send_data.yaw * 0.01745329251f;
    packet->yaw_vel = gyro[2] * (float)(-GYRO2GIMBAL_DIR_YAW); // 映射到统一逻辑坐标rad/s
    packet->pitch = send_data.pitch * 0.01745329251f;
    packet->pitch_vel = gyro[0] * (float)GYRO2GIMBAL_DIR_PITCH; // 映射到统一逻辑坐标rad/s
    send_data.yaw_rad = packet->yaw;
    send_data.yaw_vel_rad_s = packet->yaw_vel;
    send_data.pitch_rad = packet->pitch;
    send_data.pitch_vel_rad_s = packet->pitch_vel;
}

#ifdef ROBOT_TYPE_sentry
static void FillSentryTelemetryPacket(SentryTelemetryFrame_s *packet)
{
    const referee_info_t *referee_data = RefereeGetData();

    if (!packet)
        return;

    packet->head[0] = VISION_SENTRY_TELEMETRY_HEADER_1;
    packet->head[1] = VISION_SENTRY_TELEMETRY_HEADER_2;
    packet->cmd_vx = sentry_telemetry_data.cmd_vx;
    packet->cmd_vy = sentry_telemetry_data.cmd_vy;
    packet->cmd_wz = sentry_telemetry_data.cmd_wz;
    packet->real_vx = sentry_telemetry_data.real_vx;
    packet->real_vy = sentry_telemetry_data.real_vy;
    packet->real_wz = sentry_telemetry_data.real_wz;
    packet->robot_status = EncodeSentryRobotStatus(referee_data);
    packet->game_status = 0u;
    packet->stage_remain_time = 0u;
    packet->robot_id = 0u;
    packet->current_hp = 0u;
    packet->shooter_heat = 0u;
    packet->team_color = 0u;
    packet->is_attacked = 0u;

    if (referee_data != NULL)
    {
        packet->game_status = referee_data->GameState.game_progress;
        packet->stage_remain_time = referee_data->GameState.stage_remain_time;
        packet->robot_id = referee_data->GameRobotState.robot_id;
        packet->current_hp = referee_data->GameRobotState.current_HP;
        packet->shooter_heat = referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
        packet->team_color = EncodeSentryTeamColor(referee_data);
        packet->is_attacked = EncodeSentryAttackedFlag(referee_data);
    }

    packet->crc16 =
        crc_16((uint8_t *)packet, sizeof(SentryTelemetryFrame_s) - 2u);
}
#endif

/**
 * @brief 验证四元数有效性
 * @param q 四元数数组 [w,x,y,z]
 * @return 1=有效, 0=无效
 */
static uint8_t ValidateQuaternion(float q[4])
{
    float norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return (fabsf(norm_sq - 1.0f) < 0.01f); // 模在1.0的1%误差范围内
}

/**
 * @brief 验证视觉数据有效性
 * @param data VisionToGimbal数据包指针
 * @return 1=有效, 0=无效
 */
static uint8_t ValidateVisionData(VisionToGimbal_s *data)
{
    // 检查CRC
    uint16_t calc_crc = crc_16((uint8_t *)data, sizeof(VisionToGimbal_s) - 2);
    if (calc_crc != data->crc16)
    {
        LOGERROR("[vision] CRC error: calc=0x%04X, recv=0x%04X", calc_crc, data->crc16);
        return 0;
    }

    // 检查模式范围
    if (data->mode > 2)
    {
        LOGERROR("[vision] Invalid mode: %d", data->mode);
        return 0;
    }

    // 检查角度范围 (±π)
    if (fabsf(data->yaw) > 3.15f || fabsf(data->pitch) > 3.15f)
    {
        LOGERROR("[vision] Angle out of range: yaw=%.2f, pitch=%.2f", data->yaw, data->pitch);
        return 0;
    }

    return 1;
}

/**
 * @brief 快速校验视觉包（静默模式，用于流式重同步）
 */
static uint8_t ValidateVisionDataFast(const VisionToGimbal_s *data)
{
    if (!data)
        return 0;

    uint16_t calc_crc = crc_16((uint8_t *)data, sizeof(VisionToGimbal_s) - 2);
    if (calc_crc != data->crc16)
        return 0;

    if (data->mode > 2)
        return 0;

    if (fabsf(data->yaw) > 3.15f || fabsf(data->pitch) > 3.15f)
        return 0;

    return 1;
}

/**
 * @brief 将协议包内容映射到接收结构体
 */
static void ApplyVisionPacket(const VisionToGimbal_s *packet)
{
    if (!packet)
        return;

    recv_data.sp_mode = packet->mode;
    recv_data.yaw_rad = packet->yaw;
    recv_data.yaw_vel_rad_s = packet->yaw_vel;
    recv_data.yaw_acc_rad_s2 = packet->yaw_acc;
    recv_data.pitch_rad = packet->pitch;
    recv_data.pitch_vel_rad_s = packet->pitch_vel;
    recv_data.pitch_acc_rad_s2 = packet->pitch_acc;

    switch (packet->mode)
    {
    case 0: // 不控制 - 手动模式
        recv_data.fire_mode = NO_FIRE;
        recv_data.target_state = NO_TARGET;
        break;
    case 1: // 仅控制云台
        recv_data.fire_mode = AUTO_AIM;
        recv_data.target_state = READY_TO_FIRE;
        break;
    case 2: // 控制云台+开火
        recv_data.fire_mode = AUTO_FIRE;
        recv_data.target_state = READY_TO_FIRE;
        break;
    default:
        recv_data.fire_mode = NO_FIRE;
        recv_data.target_state = NO_TARGET;
        break;
    }

    recv_data.yaw = packet->yaw * 57.295779513f;
    recv_data.pitch = packet->pitch * 57.295779513f;
    recv_data.target_type = NO_TARGET_NUM; // 视觉端未提供此数据
    recv_data.new_data = 1;
}

#ifdef ROBOT_TYPE_sentry
static uint8_t ValidateSentryExtFrame(const VisionToSentryExtFrame_s *packet)
{
    if (!packet)
        return 0;

    return crc_16((uint8_t *)packet, sizeof(VisionToSentryExtFrame_s) - 2u) ==
           packet->crc16;
}

static void ApplySentryExtPacket(const VisionToSentryExtFrame_s *packet)
{
    if (!packet)
        return;

    LogSentryExtPacket(packet);
    sentry_ext_data.vx = packet->vx;
    sentry_ext_data.vy = packet->vy;
    sentry_ext_data.wz = packet->wz;
    sentry_ext_data.gimbal_yaw_delta = packet->gimbal_yaw_delta;
    sentry_ext_data.gimbal_pitch_delta = packet->gimbal_pitch_delta;
    sentry_ext_data.control_flags = packet->control_flags;
    sentry_ext_data.scan_yaw_rate_deg_s = packet->scan_yaw_rate_deg_s;
    sentry_ext_data.search_pitch_deg = packet->search_pitch_deg;
    sentry_ext_data.ts_ms = HAL_GetTick();
    sentry_ext_data.new_data = 1u;
}
#endif

static uint8_t BulletSpeedToCode(Bullet_Speed_e speed)
{
    switch (speed)
    {
    case BIG_AMU_10:
        return 1;
    case SMALL_AMU_15:
        return 2;
    case BIG_AMU_16:
        return 3;
    case SMALL_AMU_18:
        return 4;
    case SMALL_AMU_30:
        return 5;
    case BULLET_SPEED_NONE:
    default:
        return 0;
    }
}

static void VisionDecodeFlags(uint16_t flags, Vision_Recv_s *out)
{
    if (!out)
        return;
    out->fire_mode = (Fire_Mode_e)((flags & VISION_FLAG_FIRE_MODE_MASK) >> VISION_FLAG_FIRE_MODE_SHIFT);
    out->target_state = (Target_State_e)((flags & VISION_FLAG_TARGET_STATE_MASK) >> VISION_FLAG_TARGET_STATE_SHIFT);
    out->target_type = (Target_Type_e)((flags & VISION_FLAG_TARGET_TYPE_MASK) >> VISION_FLAG_TARGET_TYPE_SHIFT);
}

static uint16_t VisionEncodeFlags(const Vision_Send_s *in)
{
    uint16_t flags = 0;
    if (!in)
        return flags;
    flags |= (((uint16_t)in->enemy_color << VISION_FLAG_ENEMY_COLOR_SHIFT) & VISION_FLAG_ENEMY_COLOR_MASK);
    flags |= (((uint16_t)in->work_mode << VISION_FLAG_WORK_MODE_SHIFT) & VISION_FLAG_WORK_MODE_MASK);
    flags |= (((uint16_t)BulletSpeedToCode(in->bullet_speed) << VISION_FLAG_BULLET_SPEED_SHIFT) & VISION_FLAG_BULLET_SPEED_MASK);
    return flags;
}

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    if (!(yaw == yaw))
        yaw = 0.0f;
    if (!(pitch == pitch))
        pitch = 0.0f;
    if (!(roll == roll))
        roll = 0.0f;
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

void VisionGetRecvSnapshot(Vision_Recv_s *out)
{
    if (!out)
        return;
    out->fire_mode = recv_data.fire_mode;
    out->target_state = recv_data.target_state;
    out->target_type = recv_data.target_type;
    out->pitch = recv_data.pitch;
    out->yaw = recv_data.yaw;
    out->new_data = recv_data.new_data;
    out->sp_mode = recv_data.sp_mode;
    out->yaw_rad = recv_data.yaw_rad;
    out->yaw_vel_rad_s = recv_data.yaw_vel_rad_s;
    out->yaw_acc_rad_s2 = recv_data.yaw_acc_rad_s2;
    out->pitch_rad = recv_data.pitch_rad;
    out->pitch_vel_rad_s = recv_data.pitch_vel_rad_s;
    out->pitch_acc_rad_s2 = recv_data.pitch_acc_rad_s2;
}

void VisionGetSendSnapshot(Vision_Send_s *out)
{
    if (!out)
        return;
    out->enemy_color = send_data.enemy_color;
    out->work_mode = send_data.work_mode;
    out->bullet_speed = send_data.bullet_speed;
    out->yaw = send_data.yaw;
    out->pitch = send_data.pitch;
    out->roll = send_data.roll;
    out->sp_mode = send_data.sp_mode;
    memcpy(out->q, send_data.q, sizeof(send_data.q));
    out->yaw_rad = send_data.yaw_rad;
    out->yaw_vel_rad_s = send_data.yaw_vel_rad_s;
    out->pitch_rad = send_data.pitch_rad;
    out->pitch_vel_rad_s = send_data.pitch_vel_rad_s;
    out->bullet_speed_mps = send_data.bullet_speed_mps;
    out->bullet_count = send_data.bullet_count;
}

#ifdef ROBOT_TYPE_sentry
void VisionGetSentryExtSnapshot(VisionToSentryExt_s *out)
{
    if (!out)
        return;

    *out = sentry_ext_data;
    sentry_ext_data.new_data = 0u;
}

void VisionSetSentryTelemetry(float cmd_vx, float cmd_vy, float cmd_wz,
                              float real_vx, float real_vy, float real_wz)
{
    sentry_telemetry_data.cmd_vx = cmd_vx;
    sentry_telemetry_data.cmd_vy = cmd_vy;
    sentry_telemetry_data.cmd_wz = cmd_wz;
    sentry_telemetry_data.real_vx = real_vx;
    sentry_telemetry_data.real_vy = real_vy;
    sentry_telemetry_data.real_wz = real_wz;
}
#endif

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART

    recv_data.fire_mode = NO_FIRE;
    recv_data.target_state = NO_TARGET;
    recv_data.new_data = 1;

#ifdef ROBOT_TYPE_sentry
    memset(&sentry_ext_data, 0, sizeof(sentry_ext_data));
#endif

    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @note  使用SP协议: 帧头'S','P' + mode(1B) + yaw/vel/acc(12B) + pitch/vel/acc(12B) + CRC16(2B) = 29字节
 */
static void DecodeVision()
{
    uint8_t header_pos = FindSPHeader(vision_usart_instance->recv_buff, VISION_RECV_SIZE);
#ifdef ROBOT_TYPE_sentry
    uint8_t sx_header_pos =
        FindSentryExtHeader(vision_usart_instance->recv_buff, VISION_RECV_SIZE);
#endif
    if (header_pos == 0xFF)
    {
#ifdef ROBOT_TYPE_sentry
        if (sx_header_pos != 0xFFu &&
            (VISION_RECV_SIZE - sx_header_pos) >= sizeof(VisionToSentryExtFrame_s))
        {
            VisionToSentryExtFrame_s *sentry_packet =
                (VisionToSentryExtFrame_s *)(vision_usart_instance->recv_buff +
                                             sx_header_pos);
            if (ValidateSentryExtFrame(sentry_packet))
            {
                DaemonReload(vision_daemon_instance);
                ApplySentryExtPacket(sentry_packet);
                return;
            }
        }
#endif
        LOGERROR("[vision] SP header not found");
        return;
    }

    if (header_pos != 0)
    {
#ifdef ROBOT_TYPE_sentry
        if (sx_header_pos != 0xFFu &&
            (VISION_RECV_SIZE - sx_header_pos) >= sizeof(VisionToSentryExtFrame_s))
        {
            VisionToSentryExtFrame_s *sentry_packet =
                (VisionToSentryExtFrame_s *)(vision_usart_instance->recv_buff +
                                             sx_header_pos);
            if (ValidateSentryExtFrame(sentry_packet))
            {
                DaemonReload(vision_daemon_instance);
                ApplySentryExtPacket(sentry_packet);
                return;
            }
        }
#endif
        LOGERROR("[vision] Misaligned SP frame: header_pos=%u", header_pos);
        return;
    }

    VisionToGimbal_s *packet = (VisionToGimbal_s *)(vision_usart_instance->recv_buff + header_pos);

    // 验证数据有效性
    if (!ValidateVisionData(packet))
    {
        return;
    }

    DaemonReload(vision_daemon_instance); // 仅在有效包时喂狗
    ApplyVisionPacket(packet);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 * @note  使用SP协议: 帧头'S','P' + mode(1B) + q[4](16B) + yaw/vel/pitch/vel(16B) + bullet_speed(4B) + bullet_count(2B) + CRC16(2B) = 43字节(packed)
 */
void VisionSend()
{
    static GimbalToVision_s packet;

    // 设置帧头
    packet.head[0] = VISION_SP_HEADER_1;
    packet.head[1] = VISION_SP_HEADER_2;

    // 工作模式映射: VISION_MODE_AIM(0)→AUTO_AIM(1), SMALL_BUFF(1)→SMALL_BUFF(2), BIG_BUFF(2)→BIG_BUFF(3)
    // Note: mode=0 (IDLE) is not sent; vision enters IDLE when no valid data received (daemon timeout)
    packet.mode = send_data.work_mode + 1;

    // 统一姿态语义：q 与 yaw/pitch 同源
    FillGimbalToVisionAttitude(&packet);

    // 弹速和弹丸数
    packet.bullet_speed = (float)send_data.bullet_speed;
    packet.bullet_count = 0; // TODO: 需要从裁判系统获取实际弹丸计数
    send_data.sp_mode = packet.mode;
    send_data.bullet_speed_mps = packet.bullet_speed;
    send_data.bullet_count = packet.bullet_count;

    // 计算CRC16
    packet.crc16 = crc_16((uint8_t *)&packet, sizeof(GimbalToVision_s) - 2);

    USARTSend(vision_usart_instance, (uint8_t *)&packet, sizeof(GimbalToVision_s), USART_TRANSFER_DMA);
}

#ifdef ROBOT_TYPE_sentry
void VisionSendSentryTelemetry(void)
{
    static SentryTelemetryFrame_s packet;

    FillSentryTelemetryPacket(&packet);
    USARTSend(vision_usart_instance, (uint8_t *)&packet,
              sizeof(SentryTelemetryFrame_s), USART_TRANSFER_DMA);
}

void VisionServiceTx(void)
{
}
#endif

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

#ifdef ROBOT_TYPE_sentry
#define VISION_VCP_TX_QUEUE_DEPTH 8u
#define VISION_VCP_TX_MAX_FRAME_SIZE VISION_SEND_SIZE

typedef struct
{
    uint16_t len;
    uint8_t data[VISION_VCP_TX_MAX_FRAME_SIZE];
} VisionVcpTxSlot_s;

static VisionVcpTxSlot_s vision_vcp_tx_queue[VISION_VCP_TX_QUEUE_DEPTH];
static volatile uint8_t vision_vcp_tx_busy;
static volatile uint8_t vision_vcp_tx_head;
static volatile uint8_t vision_vcp_tx_tail;
static volatile uint8_t vision_vcp_tx_count;

static uint32_t VisionEnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void VisionExitCritical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

static void VisionVcpTryKickTx(void);

static uint8_t VisionVcpQueueFrame(const uint8_t *data, uint16_t len)
{
    uint8_t slot_idx;
    uint32_t primask;

    if (!data || len == 0u || len > VISION_VCP_TX_MAX_FRAME_SIZE)
    {
        return 0u;
    }

    primask = VisionEnterCritical();
    if (vision_vcp_tx_count >= VISION_VCP_TX_QUEUE_DEPTH)
    {
        VisionExitCritical(primask);
        return 0u;
    }

    slot_idx = vision_vcp_tx_tail;
    vision_vcp_tx_queue[slot_idx].len = len;
    memcpy(vision_vcp_tx_queue[slot_idx].data, data, len);
    vision_vcp_tx_tail =
        (uint8_t)((vision_vcp_tx_tail + 1u) % VISION_VCP_TX_QUEUE_DEPTH);
    vision_vcp_tx_count++;
    VisionExitCritical(primask);
    VisionVcpTryKickTx();
    return 1u;
}

static void VisionVcpTryKickTx(void)
{
    uint8_t slot_idx;
    uint16_t len;
    uint8_t status;
    uint32_t primask;

    primask = VisionEnterCritical();
    if (vision_vcp_tx_busy || vision_vcp_tx_count == 0u)
    {
        VisionExitCritical(primask);
        return;
    }

    slot_idx = vision_vcp_tx_head;
    len = vision_vcp_tx_queue[slot_idx].len;
    vision_vcp_tx_busy = 1u;
    VisionExitCritical(primask);

    status = USBTransmit(vision_vcp_tx_queue[slot_idx].data, len);
    if (status == USBD_OK)
    {
        return;
    }

    primask = VisionEnterCritical();
    vision_vcp_tx_busy = 0u;
    VisionExitCritical(primask);
}

static void VisionVcpTxCallback(uint16_t len)
{
    uint32_t primask;

    UNUSED(len);
    primask = VisionEnterCritical();
    if (vision_vcp_tx_count > 0u)
    {
        vision_vcp_tx_head =
            (uint8_t)((vision_vcp_tx_head + 1u) % VISION_VCP_TX_QUEUE_DEPTH);
        vision_vcp_tx_count--;
    }
    vision_vcp_tx_busy = 0u;
    VisionExitCritical(primask);
}

void VisionServiceTx(void)
{
    VisionVcpTryKickTx();
}
#endif

/**
 * @brief VCP接收解包回调函数
 * @note  使用SP协议: 帧头'S','P' + mode(1B) + yaw/vel/acc(12B) + pitch/vel/acc(12B) + CRC16(2B) = 29字节
 */
static void DecodeVision(uint16_t recv_len)
{
    static uint8_t stream_buf[VISION_RECV_SIZE * 4u];
    static uint16_t stream_len = 0u;
    uint8_t got_valid_packet = 0u;
#ifdef ROBOT_TYPE_sentry
    const uint16_t min_packet_size = VISION_SENTRY_EXT_RECV_SIZE;
#else
    const uint16_t min_packet_size = VISION_RECV_SIZE;
#endif

    if (recv_len == 0u)
    {
        return;
    }

    // 追加新数据到流缓冲
    uint16_t append_len = recv_len;
    if (append_len > sizeof(stream_buf))
    {
        append_len = (uint16_t)sizeof(stream_buf);
    }

    if (stream_len + append_len > sizeof(stream_buf))
    {
        uint16_t drop = (uint16_t)(stream_len + append_len - sizeof(stream_buf));
        if (drop >= stream_len)
        {
            stream_len = 0u;
        }
        else
        {
            memmove(stream_buf, stream_buf + drop, stream_len - drop);
            stream_len -= drop;
        }
    }

    memcpy(stream_buf + stream_len, vis_recv_buff, append_len);
    stream_len += append_len;

    // 从流中提取完整帧（支持粘包/分包）
    while (stream_len >= min_packet_size)
    {
        uint16_t header_pos = 0xFFFFu;
        uint8_t sp_header_pos = FindSPHeader(stream_buf, stream_len);

        if (sp_header_pos != 0xFFu)
        {
            header_pos = sp_header_pos;
        }

#ifdef ROBOT_TYPE_sentry
        uint8_t sx_header_pos = FindSentryExtHeader(stream_buf, stream_len);
        if (sx_header_pos != 0xFFu &&
            (header_pos == 0xFFFFu || sx_header_pos < header_pos))
        {
            header_pos = sx_header_pos;
        }
#endif

        if (header_pos == 0xFFFFu)
        {
            // 保留可能的半个帧头
            if (stream_buf[stream_len - 1u] == VISION_SP_HEADER_1)
            {
                stream_buf[0] = VISION_SP_HEADER_1;
                stream_len = 1u;
            }
            else
            {
                stream_len = 0u;
            }
            break;
        }

        if (header_pos > 0u)
        {
            memmove(stream_buf, stream_buf + header_pos, stream_len - header_pos);
            stream_len -= header_pos;
            if (stream_len < min_packet_size)
            {
                break;
            }
        }

#ifdef ROBOT_TYPE_sentry
        if (stream_len >= sizeof(VisionToSentryExtFrame_s) &&
            stream_buf[0] == VISION_SENTRY_EXT_HEADER_1 &&
            stream_buf[1] == VISION_SENTRY_EXT_HEADER_2)
        {
            VisionToSentryExtFrame_s *sentry_packet =
                (VisionToSentryExtFrame_s *)stream_buf;
            if (!ValidateSentryExtFrame(sentry_packet))
            {
                memmove(stream_buf, stream_buf + 1u, stream_len - 1u);
                stream_len -= 1u;
                continue;
            }

            ApplySentryExtPacket(sentry_packet);
            got_valid_packet = 1u;

            if (stream_len > sizeof(VisionToSentryExtFrame_s))
            {
                memmove(stream_buf, stream_buf + sizeof(VisionToSentryExtFrame_s),
                        stream_len - sizeof(VisionToSentryExtFrame_s));
                stream_len -= sizeof(VisionToSentryExtFrame_s);
            }
            else
            {
                stream_len = 0u;
            }
            continue;
        }
#endif

        if (stream_len < VISION_RECV_SIZE)
        {
            break;
        }

        VisionToGimbal_s *packet = (VisionToGimbal_s *)stream_buf;
        if (!ValidateVisionDataFast(packet))
        {
            // 帧头后数据非法，右移一字节继续重同步
            memmove(stream_buf, stream_buf + 1u, stream_len - 1u);
            stream_len -= 1u;
            continue;
        }

        ApplyVisionPacket(packet);
        got_valid_packet = 1u;

        if (stream_len > VISION_RECV_SIZE)
        {
            memmove(stream_buf, stream_buf + VISION_RECV_SIZE, stream_len - VISION_RECV_SIZE);
            stream_len -= VISION_RECV_SIZE;
        }
        else
        {
            stream_len = 0u;
        }
    }

    if (got_valid_packet)
    {
        DaemonReload(vision_daemon_instance); // 仅在有效包时喂狗
    }
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {
#ifdef ROBOT_TYPE_sentry
        .tx_cbk = VisionVcpTxCallback,
#else
        .tx_cbk = NULL,
#endif
        .rx_cbk = DecodeVision};

#ifdef ROBOT_TYPE_sentry
    vision_vcp_tx_busy = 0u;
    vision_vcp_tx_head = 0u;
    vision_vcp_tx_tail = 0u;
    vision_vcp_tx_count = 0u;
    memset(vision_vcp_tx_queue, 0, sizeof(vision_vcp_tx_queue));
#endif
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief VCP发送函数
 * @note  使用SP协议: 帧头'S','P' + mode(1B) + q[4](16B) + yaw/vel/pitch/vel(16B) + bullet_speed(4B) + bullet_count(2B) + CRC16(2B) = 43字节(packed)
 */
void VisionSend()
{
    static GimbalToVision_s packet;

    // 设置帧头
    packet.head[0] = VISION_SP_HEADER_1;
    packet.head[1] = VISION_SP_HEADER_2;

    // 工作模式映射: VISION_MODE_AIM(0)→AUTO_AIM(1), SMALL_BUFF(1)→SMALL_BUFF(2), BIG_BUFF(2)→BIG_BUFF(3)
    // Note: mode=0 (IDLE) is not sent; vision enters IDLE when no valid data received (daemon timeout)
    packet.mode = send_data.work_mode + 1;

    // 统一姿态语义：q 与 yaw/pitch 同源
    FillGimbalToVisionAttitude(&packet);

    // 弹速和弹丸数
    packet.bullet_speed = (float)send_data.bullet_speed;
    packet.bullet_count = 0; // TODO: 需要从裁判系统获取实际弹丸计数
    send_data.sp_mode = packet.mode;
    send_data.bullet_speed_mps = packet.bullet_speed;
    send_data.bullet_count = packet.bullet_count;

    // 计算CRC16
    packet.crc16 = crc_16((uint8_t *)&packet, sizeof(GimbalToVision_s) - 2);

#ifdef ROBOT_TYPE_sentry
    VisionVcpQueueFrame((const uint8_t *)&packet, sizeof(GimbalToVision_s));
#else
    USBTransmit((uint8_t *)&packet, sizeof(GimbalToVision_s));
#endif
}

#ifdef ROBOT_TYPE_sentry
void VisionSendSentryTelemetry(void)
{
    static SentryTelemetryFrame_s packet;

    FillSentryTelemetryPacket(&packet);
    VisionVcpQueueFrame((const uint8_t *)&packet,
                        sizeof(SentryTelemetryFrame_s));
}
#endif

#endif // VISION_USE_VCP
