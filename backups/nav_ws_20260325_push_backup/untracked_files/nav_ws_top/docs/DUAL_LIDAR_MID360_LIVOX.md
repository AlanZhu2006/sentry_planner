# 双雷达哨兵部署（MID360 + Livox + FAST-LIO，对应 start_robot.sh）

你当前链路：**Livox 驱动 → /livox/lidar、/livox/imu → FAST-LIO → /cloud_registered → pointcloud_to_laserscan → /scan → Nav2**。  
下面说明在这一套里如何部署**双 MID360**（雷达驱动、里程计、LIO 各层怎么接）。

---

## 1. 当前 start_robot.sh 里的分工

| 步骤 | 作用 | 话题/配置 |
|------|------|-----------|
| livox_ros_driver2 (msg_MID360_launch.py) | 雷达驱动 | 发布 `/livox/lidar`、`/livox/imu` |
| FAST-LIO (mid360.yaml) | 里程计 + LIO | 订阅 `/livox/lidar`、`/livox/imu`，发布 `/cloud_registered` 等 |
| pointcloud_to_laserscan | 转 2D 扫描 | 订阅 `/cloud_registered`，发布 `/scan` |
| Nav2 | 导航 | 用 `/map`、`/scan`、odom 等 |

双雷达时：**驱动**可以一个进程接两个 MID360，**里程计/LIO** 仍只用一个 FAST-LIO（只接一路点云 + 一路 IMU）。

---

## 2. 雷达驱动层：双 MID360 怎么接

Livox 驱动支持**一个进程多雷达**，用配置文件里的 `lidar_configs` 列出多台设备（多 IP）。

### 2.1 配置文件（双 MID360）

当前单雷达配置在：`src/livox_ros_driver2/config/MID360_config.json`，里面只有一条 `lidar_configs`。  
双雷达时复制一份（例如 `MID360_dual_config.json`），在 `lidar_configs` 里写**两个** MID360 的 IP（每台 MID360 在局域网里的 IP 不同），例如：

```json
{
  "lidar_summary_info": { "lidar_type": 8 },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100, "push_msg_port": 56200,
      "point_data_port": 56300, "imu_data_port": 56400, "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.2", "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.2", "push_msg_port": 56201,
      "point_data_ip": "192.168.1.2", "point_data_port": 56301,
      "imu_data_ip": "192.168.1.2", "imu_data_port": 56401,
      "log_data_ip": "", "log_data_port": 56501
    }
  },
  "lidar_configs": [
    { "ip": "192.168.1.182", "pcl_data_type": 0, "pattern_mode": 0, "extrinsic_parameter": { "roll": 0, "pitch": 0, "yaw": 0, "x": 0, "y": 0, "z": 0 } },
    { "ip": "192.168.1.183", "pcl_data_type": 0, "pattern_mode": 0, "extrinsic_parameter": { "roll": 0, "pitch": 0, "yaw": 0, "x": 0, "y": 0, "z": 0 } }
  ]
}
```

请把两个 `ip` 改成你两台 MID360 的实际 IP；`host_net_info` 里的 IP 改成你工控机网口 IP（和单雷达时一致即可）。

### 2.2 多雷达时的 topic：multi_topic

- **multi_topic = 0**（默认）：所有雷达**共用一个话题**，驱动会把多台 MID360 的点云合成一条，发布到 `/livox/lidar`，IMU 一般来自其中一台（如第一台）发布到 `/livox/imu`。  
  → 你**不用改** FAST-LIO 和 start_robot.sh 里任何话题，只要换用上面的双雷达 config 并起一个驱动即可。

- **multi_topic = 1**：每台雷达**单独话题**，例如 `/livox/lidar_0`、`/livox/lidar_1` 和 `/livox/imu_0`、`/livox/imu_1`。  
  → 需要让 FAST-LIO 只订主雷达：在 `mid360.yaml` 里把 `lid_topic` / `imu_topic` 改成例如 `/livox/lidar_0`、`/livox/imu_0`；副雷达 `/livox/lidar_1` 可单独做一层感知（例如再转一层 `/scan2` 或和主雷达点云合并后再转 `/scan`）。

**和 start_robot.sh 的对应关系**：  
- 若用 **multi_topic=0**：仍起 **msg_MID360_launch.py**，只把其中的 `user_config_path` 指到上面的双雷达 JSON（例如新建一个 `dual_MID360_launch.py` 里改路径）。start_robot.sh 里把“启动雷达驱动”那一步改成起这个新 launch 即可，后面 FAST-LIO、pointcloud_to_laserscan、Nav2 都不改。  
- 若用 **multi_topic=1**：同样用新 launch 指向双雷达 JSON，并在 FAST-LIO 的 mid360.yaml 里改成主雷达的 topic（如 `/livox/lidar_0`、`/livox/imu_0`）；start_robot.sh 其余不变。

---

## 3. 里程计 / LIO 层：仍只用一个 FAST-LIO

FAST-LIO 只接受**一个** `lid_topic` 和一个 `imu_topic`，所以：

- **主雷达**：用其中一路点云 + 对应 IMU 跑 FAST-LIO（要么是合并后的 `/livox/lidar` + `/livox/imu`，要么是 `/livox/lidar_0` + `/livox/imu_0`）。  
- **副雷达**：不参与里程计，只用于感知（或和主雷达点云一起生成更丰富的 `/scan`）。

你现有的 **start_robot.sh 里第 5 步**仍然是：

- 只起**一个** FAST-LIO；
- 配置文件继续用 **mid360.yaml**（若用 multi_topic=1 双 topic，只需把 mid360.yaml 里的 `lid_topic` / `imu_topic` 改成主雷达的 topic）。

不需要起两个 FAST-LIO，也不要把两路点云“拼在一起”再喂给 FAST-LIO（时间戳和运动畸变模型会乱）。

---

## 4. 在 start_robot.sh 里的具体改法（推荐 multi_topic=0）

1. **复制并改 Livox 配置**  
   - 在 `src/livox_ros_driver2/config/` 下新建 `MID360_dual_config.json`，内容如上面 2.1，两个 `ip` 改成你两台 MID360 的 IP。

2. **复制并改 launch，用双雷达配置**  
   - 复制 `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 为 `msg_MID360_dual_launch.py`；  
   - 在 launch 里把 `user_config_path` 改成指向 `MID360_dual_config.json`；  
   - 保持 `multi_topic = 0`，这样仍发布 `/livox/lidar`、`/livox/imu`，后面都不用改。

3. **改 start_robot.sh 里“启动雷达”那一行**  
   - 把：
     - `ros2 launch livox_ros_driver2 msg_MID360_launch.py`
   - 改成：
     - `ros2 launch livox_ros_driver2 msg_MID360_dual_launch.py`  
   - 已提供：`src/livox_ros_driver2/config/MID360_dual_config.json`、`src/livox_ros_driver2/launch_ROS2/msg_MID360_dual_launch.py`。修改完双雷达 IP 后需执行 `colcon build --packages-select livox_ros_driver2` 再启动。

4. **其余不动**  
   - FAST-LIO 仍用 `config_file:=mid360.yaml`；  
   - pointcloud_to_laserscan 仍订 `/cloud_registered`；  
   - Nav2 仍用现有 map 和 params。

这样就是：**双 MID360 在驱动里合并 → 同一套 /livox/lidar、/livox/imu → 一个 FAST-LIO → 一个 /scan → Nav2**，和现在的 start_robot.sh 流程一致，只是雷达从单台变成两台。

---

## 5. 小结（针对你当前 start_robot.sh + MID360 + Livox）

| 层级 | 单雷达（当前） | 双 MID360 哨兵 |
|------|----------------|----------------|
| **雷达驱动** | 一个 `msg_MID360_launch.py`，一个 MID360_config.json | 一个 launch + 一个 JSON，JSON 里 `lidar_configs` 写两个 MID360 的 IP；可选 multi_topic=0（合并）或 1（分 topic） |
| **里程计 / LIO** | 一个 FAST-LIO，mid360.yaml 订 /livox/lidar、/livox/imu | 仍只一个 FAST-LIO；multi_topic=0 时配置不用改；multi_topic=1 时改为订主雷达的 /livox/lidar_0、/livox/imu_0 |
| **start_robot.sh** | 起 msg_MID360_launch.py → FAST-LIO → pointcloud_to_laserscan → Nav2 | 只把“起雷达”改成双雷达的 launch；其余步骤不变 |

双雷达时：**驱动**用一份配置接两个 MID360；**里程计和 LIO** 仍只跑一个 FAST-LIO，接主雷达（或合并后）的那一路；**Nav2** 仍用现有 `/map`、`/scan` 即可。若你希望副雷达单独参与一层感知（例如多一个 `/scan2`），再在 multi_topic=1 的基础上加一层对 `/livox/lidar_1` 的 pointcloud_to_laserscan 即可。
