# RViz 地图不显示 - 排查指南

## 1. 确认 /map 是否发布

```bash
# 查看是否有 /map 话题
ros2 topic list | grep map

# 查看 /map 是否有数据（等待几秒）
ros2 topic echo /map --once

# 查看 /map 发布频率
ros2 topic hz /map
```

若没有 `/map` 或一直无数据，说明 map_server 或 slam_toolbox 未正常工作。

---

## 2. 方案 A（假传感器 + 预存地图）

- 地图由 **map_server** 从 yaml 加载
- 若 map_server 未启动或报错，`/map` 不会发布
- 检查：`ros2 topic list` 中应有 `/map`

**可能原因**：Nav2 controller 之前失败导致 lifecycle 未完全激活，map_server 可能未正常启动。  
**处理**：重启整个测试，或单独起 map_server 做验证。

---

## 3. 方案 B（Gazebo 仿真 mapping 模式）

- 地图由 **slam_toolbox** 实时构建
- slam_toolbox 需要 **/scan**
- 数据链：Gazebo Livox → Fast-LIO → /cloud_registered → 点云分割 → pointcloud_to_laserscan → **/scan**

**可能原因**：`segmentation` 的 `input_topic` 为 `/livox/lidar/pointcloud`，仿真中该话题可能不存在（仿真多发布 CustomMsg 到 `/livox/lidar`），导致无 /scan，slam 不建图，/map 为空或很晚才出现。

---

## 4. RViz 设置检查

1. **Fixed Frame**：应为 `map`
   - 左侧 Global Options → Fixed Frame → 选 `map`
   - 若选 `base_link` 等，地图可能不显示或错位

2. **Map 显示**
   - 左侧 Displays 里确认有 "Map"
   - 展开 Map，Topic 应为 `/map`
   - 确认 Map 已勾选启用 (Enable)

3. **QoS 不匹配**
   - 若 Map 显示为 "Status: Ok" 但无画面，尝试：
   - 添加 Map 时，QoS 选 `transient_local` + `reliable`

---

## 5. TF 检查

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查 map 是否在 TF 中
ros2 run tf2_ros tf2_echo map base_link
```

Map 显示不依赖 TF，但若 Fixed Frame 为 `map` 且 `map` 不在 TF 中，其他显示可能报错。
