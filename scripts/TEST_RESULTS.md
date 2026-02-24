# 方案 A 测试结果

## 测试环境
- 脚本: `run_test_a.sh` / `run_test_a_headless.sh`
- 地图: 默认使用项目自带的 `rm_navigation_ws/src/rm_nav_bringup/map/RMUL.yaml`（RMUL 高校联盟赛场地）
- Nav2 参数: `/home/nyu/nav_ws/my_nav2_params.yaml` ✓

## 测试发现

### 1. ~/.ros 权限问题
`~/.ros` 目录当前归属 root，导致 ROS 节点无法写入日志：

```
PermissionError: /home/nyu/.ros/log/...
```

**修复：**
```bash
sudo chown -R $(whoami):$(whoami) ~/.ros
```

### 2. RViz 需要显示环境
在无图形界面终端中运行会报错：
```
qt.qpa.xcb: could not connect to display
```
需在有桌面/显示器的终端中运行 `run_test_a.sh`。

### 3. Nav2 map_server lifecycle 失败
使用 `run_test_a_headless.sh` 时，map_server 在 configure 阶段失败：
```
[ERROR] lifecycle_manager_localization: Failed to change state for node: map_server
[ERROR] Failed to bring up all requested nodes. Aborting bringup.
```
导致 `navigate_to_pose` action server 不可用，决策树报错：
```
Action server with name 'navigate_to_pose' is not reachable
```

### 4. 正常运行的组件
- ✓ 假传感器 (fake_sensors_for_test.py) 正常发布 /scan, /odom, TF
- ✓ 决策行为树正常加载，订阅 game_status
- ✓ game_status 正常发布 (game_progress=4)

## 建议操作顺序

### 完整测试（带 RViz）
1. 修复权限：`sudo chown -R $(whoami):$(whoami) ~/.ros`
2. 在**有显示器的终端**中执行：
   ```bash
   cd /home/nyu/sentry_planner && ./scripts/run_test_a.sh
   ```
3. 在 RViz 中用 "Nav2 Goal" 发送目标

### 无界面快速验证
使用 headless 脚本（会绕过 ~/.ros 权限，使用 /tmp）：
```bash
cd /home/nyu/sentry_planner && ./scripts/run_test_a_headless.sh
```
注：当前 map_server 可能仍会失败，需进一步排查 Nav2 参数传递。

## 相关文件
- `run_test_a.sh` - 完整版（含 RViz）
- `run_test_a_headless.sh` - 无界面版
- `fake_sensors_for_test.py` - 假传感器
