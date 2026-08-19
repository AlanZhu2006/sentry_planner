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
#include "robot_def.h"
#include "crc16.h"
#include "ins_task.h"
#include <string.h>
#include <math.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;

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
    float yaw_ctrl_rad_s = gyro[2] * (float)GIMBAL_YAW_CTRL_FROM_IMU_SIGN;
    float pitch_ctrl_rad_s = gyro[0] * (float)GYRO2GIMBAL_DIR_PITCH;

    // send_data 的 yaw/pitch/roll 始终保存 ctrl 坐标（+yaw=向右, +pitch=向上, 单位: deg）。
    // 这里统一由同一套欧拉角生成四元数，确保 q 与 yaw/pitch 完全同源。
    EularAngleToQuaternion(send_data.yaw, send_data.pitch, send_data.roll, q_send);
    memcpy(packet->q, q_send, sizeof(float) * 4);
    memcpy(send_data.q, q_send, sizeof(float) * 4);

    // 协议发送坐标由 ctrl 坐标在边界处转换。
    packet->yaw = send_data.yaw * VISION_DEG_TO_RAD;
    packet->yaw_vel = yaw_ctrl_rad_s;
    packet->pitch = send_data.pitch * VISION_DEG_TO_RAD;
    packet->pitch_vel = pitch_ctrl_rad_s;
    send_data.yaw_rad = packet->yaw;
    send_data.yaw_vel_rad_s = packet->yaw_vel;
    send_data.pitch_rad = packet->pitch;
    send_data.pitch_vel_rad_s = packet->pitch_vel;
}

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

    recv_data.yaw = packet->yaw * VISION_RAD_TO_DEG;
    recv_data.pitch = packet->pitch * VISION_RAD_TO_DEG;
    recv_data.target_type = NO_TARGET_NUM; // 视觉端未提供此数据
    recv_data.new_data = 1;
}

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
    if (header_pos == 0xFF)
    {
        LOGERROR("[vision] SP header not found");
        return;
    }

    if (header_pos != 0)
    {
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

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

/**
 * @brief VCP接收解包回调函数
 * @note  使用SP协议: 帧头'S','P' + mode(1B) + yaw/vel/acc(12B) + pitch/vel/acc(12B) + CRC16(2B) = 29字节
 */
static void DecodeVision(uint16_t recv_len)
{
    static uint8_t stream_buf[VISION_RECV_SIZE * 4u];
    static uint16_t stream_len = 0u;
    uint8_t got_valid_packet = 0u;

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
    while (stream_len >= VISION_RECV_SIZE)
    {
        uint8_t header_pos = FindSPHeader(stream_buf, stream_len);
        if (header_pos == 0xFF)
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
            if (stream_len < VISION_RECV_SIZE)
            {
                break;
            }
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
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
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

    USBTransmit((uint8_t *)&packet, sizeof(GimbalToVision_s));
}

#endif // VISION_USE_VCP
