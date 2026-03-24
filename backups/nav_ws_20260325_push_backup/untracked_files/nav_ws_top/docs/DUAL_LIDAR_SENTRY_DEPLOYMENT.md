# 双雷达哨兵部署说明

本文说明在**雷达驱动、里程计、LIO SLAM** 层面如何部署双雷达（双 Unilidar）哨兵方案。

---

## 1. 整体架构概览

| 层级 | 单雷达（当前） | 双雷达哨兵 |
|------|----------------|------------|
| **雷达驱动** | 1 个 `unitree_lidar_ros2_node`，话题 `/unilidar/cloud`、`/unilidar/imu` | 2 个驱动节点，不同话题与 frame（见下） |
| **里程计 / LIO** | 1 个 Point-LIO，订阅 1 个 cloud + 1 个 IMU | **主雷达**跑 Point-LIO 提供 odom；**副雷达**仅作感知（或合并后仍用单 LIO） |
| **Nav2** | `/map` + `/scan`（来自主雷达点云） | `/map` 来自主雷达建图；`/scan` 可来自合并点云或主雷达 |

要点：

- **Point-LIO 只支持单路点云 + 单路 IMU**，不能直接“双雷达进一个 LIO”。
- 双雷达时要么：**主雷达 LIO + 副雷达只做感知**，要么：**两路点云先合并到同一 frame，再给一个 Point-LIO（仍只用主雷达 IMU）**。

---

## 2. 雷达驱动层：双雷达怎么起

每个 Unilidar 一个驱动进程，用**不同参数**区分：不同串口或不同网络（IP/端口），以及不同的 **topic 和 frame**，避免冲突。

### 2.1 两台雷达的两种连接方式

- **方式 A：双串口**  
  雷达 1 → `/dev/ttyACM0`，雷达 2 → `/dev/ttyUSB0`（或另一 USB 转串口）。

- **方式 B：双网口**  
  雷达 1：`lidar_ip: 192.168.1.62`，`lidar_port/local_port` 一组；雷达 2：另一 IP 或同一网段不同 IP，另一组 `lidar_port/local_port`。

### 2.2 建议的 topic 与 frame 命名

| 雷达 | cloud_topic | imu_topic | cloud_frame | imu_frame |
|------|-------------|-----------|-------------|-----------|
| 主雷达（做 LIO） | `unilidar/cloud` | `unilidar/imu` | `unilidar_lidar` | `unilidar_imu` |
| 副雷达 | `unilidar2/cloud` | `unilidar2/imu` | `unilidar2_lidar` | `unilidar2_imu` |

这样你现有的 Point-LIO 配置（订阅 `/unilidar/cloud`、`/unilidar/imu`）不用改，只加“第二个驱动 + 副雷达的 TF + 可选合并”。

### 2.3 双驱动启动示例（launch）

- 主雷达：保持现有 `unitree_lidar_ros2` 的 launch（或现有脚本里那套参数），保证发布 `unilidar/cloud`、`unilidar/imu`。
- 副雷达：再起一个 `unitree_lidar_ros2_node`，例如：
  - 串口方式：`serial_port: '/dev/ttyUSB0'`，并设置 `cloud_topic: 'unilidar2/cloud'`、`imu_topic: 'unilidar2/imu'`，以及 `cloud_frame: 'unilidar2_lidar'`、`imu_frame: 'unilidar2_imu'`。
  - 网络方式：另一组 `lidar_ip`/`lidar_port`/`local_port`，同样改 topic/frame 为 `unilidar2/*`。

仓库中已提供示例 launch：`configs/unitree_l2_complete/dual_unilidar_drivers.launch.py`。运行方式（在 `nav_ws` 下）：

```bash
cd ~/nav_ws && source install/setup.bash
# 仅主雷达时仍用：ros2 launch unitree_lidar_ros2 launch.py

# 双雷达时：先按 2.1 在 launch 里改副雷达 serial_port 或网络参数，再执行
python3 configs/unitree_l2_complete/dual_unilidar_drivers.launch.py
```

可按你实际串口/网络在 launch 文件里改副雷达的 `serial_port` 或 `lidar_ip`/`local_port`。

---

## 3. 里程计与 LIO SLAM 层：怎么接

Point-LIO 只接受**一个** `lid_topic` 和一个 `imu_topic`，因此：

- **主雷达**：`/unilidar/cloud` + `/unilidar/imu` → 当前 `unilidar_l2.yaml` 不变，继续作为**唯一 LIO 里程计**，输出 `aft_mapped_to_init`、点云等。
- **副雷达**：不参与 LIO，只用于感知。

两种用法：

### 方案 A：主雷达 LIO + 副雷达仅作感知（推荐，最简单）

- 驱动：主雷达 → `unilidar/cloud`、`unilidar/imu`；副雷达 → `unilidar2/cloud`、`unilidar2/imu`。
- LIO：只跑一个 Point-LIO，配置不变（`lid_topic: "/unilidar/cloud"`, `imu_topic: "/unilidar/imu"`）。
- 建图 / 定位：全部用主雷达 + Point-LIO（和现在一样）。
- 副雷达用途：
  - 用 `pointcloud_to_laserscan` 把 `unilidar2/cloud` 转成例如 `/scan2`，给 Nav2 做第二路障碍；或
  - 与主雷达点云合并后再转 `/scan`（见方案 B）。

TF 要求：  
- 主雷达：`base_link` → `unilidar_lidar` / `unilidar_imu_initial`（你现有 TF 保持）。  
- 副雷达：发布 `base_link` → `unilidar2_lidar`（以及可选 `unilidar2_imu`），用 static_transform_publisher 或 URDF 均可。

### 方案 B：双雷达点云合并 + 单 LIO（仍用一个 IMU）

- 两路点云先转到同一坐标系（如 `base_link`），再合并成一条 PointCloud2。
- **只把合并后的点云**作为 LIO 的输入会破坏“单雷达 + 单 IMU”的假设（时间戳、运动畸变模型不一致），一般不推荐把合并点云直接喂给 Point-LIO。
- 更稳妥仍是：**LIO 只用主雷达**（`unilidar/cloud` + `unilidar/imu`），合并点云只用于：
  - 生成更强的 `/scan`（例如 `pointcloud_to_laserscan` 订阅合并后的 cloud），或
  - 显示、录制、其他感知算法。

因此**里程计/LIO 层**建议始终是：**一个 Point-LIO，只订阅主雷达的 cloud + IMU**；副雷达仅参与感知或合并后做 `/scan`。

---

## 4. 在你当前工程里的部署顺序

1. **雷达驱动**  
   - 保持主雷达现有 launch/脚本（`unilidar/cloud`、`unilidar/imu`）。  
   - 增加副雷达：第二个 `unitree_lidar_ros2_node`（不同串口或网络 + `unilidar2/cloud`、`unilidar2/imu`）。  
   - 用 `dual_unilidar_drivers.launch.py` 或等价脚本一次起两个驱动。

2. **TF**  
   - 主雷达：沿用现有（如 `base_link` → `unilidar_imu_initial` 等）。  
   - 副雷达：增加 `base_link` → `unilidar2_lidar`（安装位置、朝向按实车测量）。

3. **里程计 / LIO**  
   - 不改 `unilidar_l2.yaml`，不增加第二个 Point-LIO。  
   - 继续用主雷达做唯一 LIO；副雷达不接 LIO。

4. **Nav2 / 哨兵逻辑**  
   - `/map`、`/odom`、`/tf` 仍来自主雷达 + Point-LIO。  
   - `/scan`：可继续用主雷达点云转；若希望更大视野，可用合并点云再转 `pointcloud_to_laserscan` 得到 `/scan`。

5. **可选：点云合并**  
   - 用 `robot_state_publisher` + 上面 TF 把 `unilidar2/cloud` 转到 `base_link`，再用 `topic_tools/relay` 或自写小节点把两路 cloud 合并为一条，发布到例如 `/merged/cloud`，最后用 `pointcloud_to_laserscan` 转成 `/scan`。

---

## 5. 小结

| 问题 | 答案 |
|------|------|
| 双雷达在**驱动**怎么部署？ | 两个 `unitree_lidar_ros2_node`，不同串口或不同 IP/端口，不同 topic 与 frame（主：`unilidar/*`，副：`unilidar2/*`）。 |
| 双雷达在**里程计**怎么部署？ | 只用一个里程计：Point-LIO 只接主雷达的 `unilidar/cloud` + `unilidar/imu`。 |
| 双雷达在 **LIO SLAM** 怎么部署？ | 只跑一个 Point-LIO，配置不变；副雷达不参与 LIO，只做感知或合并后做 `/scan`。 |
| 副雷达有什么用？ | 扩大 FOV、冗余、或合并后给 Nav2 的 `/scan` 用，提高避障/哨兵效果。 |

按上述方式，你的**雷达驱动、里程计、LIO SLAM** 在双雷达哨兵场景下的部署关系是：**双驱动 + 单 LIO（主雷达） + 副雷达仅感知**；可选再加点云合并与统一 `/scan`。
