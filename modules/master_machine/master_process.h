#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 29u // SP协议: VisionToGimbal 29字节
#define VISION_SEND_SIZE 43u // SP协议: GimbalToVision 43字节(packed)

// flags_register bit layout (LSB -> MSB)
// [1:0]  fire_mode
// [3:2]  target_state
// [7:4]  target_type
// [9:8]  enemy_color
// [11:10] work_mode
// [15:12] bullet_speed_code
#define VISION_FLAG_FIRE_MODE_SHIFT 0
#define VISION_FLAG_FIRE_MODE_MASK  0x0003
#define VISION_FLAG_TARGET_STATE_SHIFT 2
#define VISION_FLAG_TARGET_STATE_MASK  0x000C
#define VISION_FLAG_TARGET_TYPE_SHIFT 4
#define VISION_FLAG_TARGET_TYPE_MASK  0x00F0
#define VISION_FLAG_ENEMY_COLOR_SHIFT 8
#define VISION_FLAG_ENEMY_COLOR_MASK  0x0300
#define VISION_FLAG_WORK_MODE_SHIFT 10
#define VISION_FLAG_WORK_MODE_MASK  0x0C00
#define VISION_FLAG_BULLET_SPEED_SHIFT 12
#define VISION_FLAG_BULLET_SPEED_MASK  0xF000

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch; // deg, ctrl坐标(+pitch=向上)
	float yaw;   // deg, ctrl坐标(+yaw=向右)
	float yaw_rad;          // raw protocol
	float yaw_vel_rad_s;    // raw protocol
	float yaw_acc_rad_s2;   // raw protocol
	float pitch_rad;        // raw protocol
	float pitch_vel_rad_s;  // raw protocol
	float pitch_acc_rad_s2; // raw protocol
	volatile uint8_t new_data; // set by decode callback, cleared after cmd processes it
	uint8_t sp_mode;           // VisionToGimbal_s.mode (raw SP protocol field)
	uint16_t reserved0;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;   // deg, ctrl坐标(+yaw=向右)
	float pitch; // deg, ctrl坐标(+pitch=向上)
	float roll;  // deg, ctrl坐标
	float q[4];
	float yaw_rad;         // raw protocol
	float yaw_vel_rad_s;   // raw protocol
	float pitch_rad;       // raw protocol
	float pitch_vel_rad_s; // raw protocol
	float bullet_speed_mps;
	uint16_t bullet_count;
	uint8_t sp_mode; // GimbalToVision_s.mode (raw SP protocol field)
	uint8_t reserved0;
} Vision_Send_s;
#pragma pack()

// ==================== SP协议定义 (用于视觉通信) ====================
#define VISION_SP_HEADER_1 'S'
#define VISION_SP_HEADER_2 'P'

#pragma pack(1)
/**
 * @brief 云台发送给视觉的数据包 (43字节, packed)
 */
typedef struct
{
    uint8_t head[2];        // 'S', 'P' 帧头
    uint8_t mode;           // 工作模式: 0=空闲, 1=自瞄, 2=小符, 3=大符
    float q[4];             // 四元数 [w,x,y,z]
    float yaw;              // 偏航角 (弧度)
    float yaw_vel;          // 偏航角速度 (rad/s)
    float pitch;            // 俯仰角 (弧度)
    float pitch_vel;        // 俯仰角速度 (rad/s)
    float bullet_speed;     // 弹速 (m/s)
    uint16_t bullet_count;  // 累计弹丸数
    uint16_t crc16;         // CRC16校验
} GimbalToVision_s;

/**
 * @brief 视觉发送给云台的数据包 (29字节)
 */
typedef struct
{
    uint8_t head[2];        // 'S', 'P' 帧头
    uint8_t mode;           // 控制模式: 0=不控制, 1=控制云台, 2=控制云台+开火
    float yaw;              // 目标偏航角 (弧度)
    float yaw_vel;          // 目标偏航角速度 (rad/s)
    float yaw_acc;          // 目标偏航角加速度 (rad/s²)
    float pitch;            // 目标俯仰角 (弧度)
    float pitch_vel;        // 目标俯仰角速度 (rad/s)
    float pitch_acc;        // 目标俯仰角加速度 (rad/s²)
    uint16_t crc16;         // CRC16校验
} VisionToGimbal_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

/**
 * @brief 获取视觉接收数据快照(用于调试/遥测)
 */
void VisionGetRecvSnapshot(Vision_Recv_s *out);

/**
 * @brief 获取视觉发送数据快照(用于调试/遥测)
 */
void VisionGetSendSnapshot(Vision_Send_s *out);

#endif // !MASTER_PROCESS_H
