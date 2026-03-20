# 机器人控制命令（云台，底盘）
# legacy compatibility fields
bool stop_gimbal_scan # 旧语义: 0->继续扫描 1->停止扫描并允许自瞄接管
float32 chassis_spin_vel # 底盘小陀螺旋转速度 (rad/s)

# sentry functional control fields
bool scan_enabled # 是否启用云台扫描
bool allow_vision_control # 是否允许视觉自瞄接管云台
bool search_when_target_lost # 自瞄无目标时是否回到扫描搜索
float32 scan_yaw_rate_deg_s # 扫描时云台yaw角速度 (deg/s)
float32 search_pitch_deg # 扫描/搜索时云台俯仰目标角 (deg)
