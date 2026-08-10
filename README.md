# ROS 2 Humble 纯双目室内小车 SLAM 导航仿真

这是一个独立的 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 工作区。定位、建图和避障只使用左右相机；Gazebo 差速里程计仅桥接为 `/ground_truth/odom`，供验收程序比较，不进入 RTAB-Map、Nav2 或 TF 树。

## 组件

- `stereo_nav_description`：差速小车 URDF 与 REP-103 光学坐标系。
- `stereo_nav_gazebo`：Gazebo 模型、双目相机、带局部视觉特征的室内世界和显式 bridge。
- `stereo_nav_bringup`：RTAB-Map 双目视觉里程计、建图/定位、点云和 Nav2。
- `stereo_nav_tests`：离线配置契约、运行时接口、轨迹精度和五目标导航验收。

## 安装与构建

目标系统必须是原生 Ubuntu 22.04。安装脚本发现系统版本、ROS overlay、NVIDIA、OpenGL 或 Gazebo Fortress 不匹配时会停止，不会升级操作系统，也不会修改 `.bashrc`。

如果当前副本在 Windows 共享盘，先复制到 Ubuntu 文件系统：

```bash
cp -a /mnt/e/Code/ROV\ DataRos2/stereo_nav_sim_ws ~/stereo_nav_sim_ws
cd ~/stereo_nav_sim_ws
chmod +x setup_project.sh env.sh scripts/*.sh
./setup_project.sh
source env.sh
```

无显示器的主机使用：

```bash
STEREO_NAV_HEADLESS=1 ./setup_project.sh
```

项目环境固定为 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`，只在 `source env.sh` 的终端生效。

## 三个入口

仅启动仿真、bridge 和机器人：

```bash
ros2 launch stereo_nav_bringup sim.launch.py gui:=true rviz:=true
```

另开终端遥控：

```bash
source ~/stereo_nav_sim_ws/env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

纯双目建图。`new_map:=false` 是安全默认值，会续写已有数据库；仅在明确重建时显式删除：

```bash
ros2 launch stereo_nav_bringup mapping.launch.py new_map:=true
```

数据库实时写入 `~/.ros/stereo_nav/rtabmap.db`。闭环完成后导出二维快照：

```bash
./scripts/save_map.sh
```

加载同一数据库，以 `Mem/IncrementalMemory=false` 定位并运行 Nav2：

```bash
ros2 launch stereo_nav_bringup navigation.launch.py moving_obstacle:=false
```

动态障碍验收时改成 `moving_obstacle:=true`。随后可在 RViz 使用 `2D Goal Pose` 或调用 `/navigate_to_pose`。

## 关键接口

| Topic / TF | 发布者 | 说明 |
|---|---|---|
| `/stereo/left/image_raw`、`camera_info` | Gazebo bridge | 左目，640×480 mono8，30 Hz |
| `/stereo/right/image_raw`、`camera_info` | Gazebo bridge | 右目，`P[3] = -38.4` |
| `/odom`、`odom → base_link` | RTAB-Map stereo odometry | 唯一运行里程计 |
| `/map`、`map → odom` | RTAB-Map SLAM | 二维栅格和全局定位 |
| `/stereo/points2` | RTAB-Map utility | Nav2 局部 Voxel Layer 唯一实时障碍输入 |
| `/ground_truth/odom` | Gazebo | 只允许验收工具订阅，不发布 TF |
| `/cmd_vel_nav → /cmd_vel` | Nav2 controller / velocity smoother | 0.8 秒命令超时，最大 0.30 m/s、1.0 rad/s |

Gazebo 没有激光或 IMU，差速插件的 TF 话题也不桥接。RTAB-Map 二进制内部固定创建的可选 IMU/GPS/标志订阅被隔离到 `/_stereo_nav_disabled/*`；这些话题没有发布者，真实 `/imu`、`/scan`、轮速和真值数据不会进入算法。

## 验证

离线配置契约会随构建执行：

```bash
colcon test
colcon test-result --verbose
```

完整无 GUI 冒烟测试会创建临时数据库，不触碰正式地图：

```bash
./scripts/headless_smoke_test.sh
```

仿真运行时检查 30 Hz 图像、3 ms 同步、标定、10 Hz 点云、topic/TF 发布者和禁用输入：

```bash
ros2 run stereo_nav_tests runtime_audit --duration 20
```

遥控不少于 20 m 闭环时评估视觉轨迹：

```bash
ros2 run stereo_nav_tests trajectory_evaluator \
  --duration 180 --minimum-distance 20 --rmse-limit 0.30 --final-limit 0.25
```

在已完整建图的数据库上启动定位和 Nav2，再执行五个跨房间目标：

```bash
ros2 run stereo_nav_tests navigation_acceptance \
  --goal-timeout 120 --position-limit 0.25
```

最后用 `moving_obstacle:=true` 重复导航，并执行 15 分钟无 GUI 稳定性检查：

```bash
./scripts/headless_smoke_test.sh 900
```

确认局部点云能触发停车/绕行，且没有 TF authority 冲突、持续同步警告或节点退出。

## 设计边界

- 工程针对 Gazebo Fortress（Gazebo Sim 6）和 ROS 2 Humble API，不安装或使用 Gazebo Classic。
- 仿真图像零畸变、共面，直接视为已校正；真实相机不能复用这里的 `CameraInfo`。
- 纯视觉在低纹理、单侧遮挡和快速旋转时可能失跟。Nav2 的速度限制与 1 秒命令超时用于让失跟后底盘停止，不会回退到真值、轮速、IMU 或激光定位。
- 五目标验收坐标对应本项目固定出生点与完整室内地图；修改出生点或世界后需要同步更新验收目标。
