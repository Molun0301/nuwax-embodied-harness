# Go2 SLAM / Nav2

本仓库用于在 GO2 外部计算机上运行 ROS 2 Jazzy 导航与建图。当前实现假设外部计算机通过网线直连 GO2，在外部计算机上完成驱动接入、SLAM、Nav2、点云建图、地图保存和定位导航。

## 运行环境

- 系统：Ubuntu 24.04 + ROS 2 Jazzy。
- 运行位置：GO2 外部计算机，不在机器狗本体上运行。
- 连接方式：外部计算机通过以太网线连接 GO2，并配置好 Unitree ROS 2 DDS 网络。
- 图形界面：`start_go2_slam_nav2.sh` 和 `start_go2_nav2_localization.sh` 会启动 RViz，需要本机桌面会话或可用的 X11/Wayland 显示环境。

网络侧需要确认外部计算机能看到 GO2 的 DDS 话题，并且已经设置正确的网卡。典型检查方式：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

如果看不到 Unitree 相关话题，先检查网线、网卡 IP、`RMW_IMPLEMENTATION`、CycloneDDS/Unitree 环境脚本和防火墙。

## ROS 依赖

先安装 ROS 2 Jazzy 和基础构建工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-jazzy-desktop
```

再安装本仓库会直接用到的 ROS 包：

```bash
sudo apt install -y \
  ros-jazzy-ament-cmake \
  ros-jazzy-ament-cmake-gtest \
  ros-jazzy-ament-index-python \
  ros-jazzy-ament-lint-auto \
  ros-jazzy-ament-lint-common \
  ros-jazzy-launch \
  ros-jazzy-launch-ros \
  ros-jazzy-rclpy \
  ros-jazzy-rclcpp \
  ros-jazzy-rclcpp-components \
  ros-jazzy-rclcpp-lifecycle \
  ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-map-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-laser-geometry \
  ros-jazzy-message-filters \
  ros-jazzy-image-geometry \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-amcl \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-core \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-smoother \
  ros-jazzy-nav2-util \
  ros-jazzy-nav2-waypoint-follower \
  ros-jazzy-pluginlib \
  ros-jazzy-ros2launch \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-default-runtime
```

点云建图编译还会用到系统库：

```bash
sudo apt install -y libpcl-dev libopencv-dev libboost-thread-dev
```

`unitree_go` 和 `unitree_api` 不是本仓库内的源码包，也通常不是 Ubuntu 官方 ROS apt 包。构建本仓库前，需要先按 Unitree 官方 ROS 2 接口说明安装或编译这两个消息/API 包，并确保下面命令能找到它们：

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg prefix unitree_go
ros2 pkg prefix unitree_api
```

也可以用 `rosdep` 对当前工作区做一次依赖校验：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

如果 `rosdep` 报 `unitree_go` 或 `unitree_api` 无法解析，先完成 Unitree ROS 2 接口包安装，再重新构建。

## 构建

```bash
cd ~/go2slam
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果改动了源码或刚安装了新的 ROS 依赖，重新执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 顶层脚本

| 脚本 | 功能 |
| --- | --- |
| `start_go2_slam_nav2.sh` | 启动 GO2 驱动、EKF、点云转 LaserScan、slam_toolbox、Nav2 和 RViz，用于边建 2D 地图边导航。 |
| `start_go2_slam_nav2.sh --mapping` | 在 SLAM/Nav2 基础上启动 `go2_mapping_only` 点云建图，发布 `/go2_built_map`，用于后续保存 PCD 点云地图。 |
| `start_go2_slam_nav2.sh --no-rviz` | 启动 SLAM/Nav2 但不启动 RViz，适用于无显示环境或远程终端。 |
| `start_go2_nav2_localization.sh` | 使用 `maps/` 下最新 2D 地图启动 map_server、AMCL、Nav2 和 RViz；如果 `output/` 下有 PCD，会尝试自动初始定位。 |
| `start_go2_nav2_localization.sh <map.yaml>` | 使用指定 2D 地图启动 AMCL + Nav2，PCD 仍自动选择 `output/` 下最新文件。 |
| `start_go2_nav2_localization.sh <map.yaml> <map.pcd>` | 使用指定 2D 地图和指定 PCD 点云图启动定位导航。 |
| `save_2dmap.sh` | 调用 `nav2_map_server map_saver_cli` 保存当前 `/map` 到 `maps/map.yaml` 和 `maps/map.pgm`。 |
| `save_pointcloud.sh` | 等待一条 `/go2_built_map` 点云消息，并保存为 `output/go2_built_map_<timestamp>.pcd`。 |
| `get_go2_map_pose.sh` | 读取 `map -> base_footprint` TF，打印 GO2 在地图中的位置和朝向。 |
| `get_go2_map_pose.sh --watch` | 持续打印 GO2 当前地图坐标，可配合 `--rate` 指定刷新频率。 |
| `get_go2_map_pose.sh forward <m>` | 基于当前朝向计算前方目标点，只打印不发送。 |
| `get_go2_map_pose.sh forward <m> --send` | 基于当前朝向向 Nav2 的 `/navigate_to_pose` action 发送前进目标。 |
| `get_go2_map_pose.sh map-x <m> --send` | 沿 `map` 坐标系 x 方向发送相对导航目标。 |
| `get_go2_map_pose.sh map-y <m> --send` | 沿 `map` 坐标系 y 方向发送相对导航目标。 |
| `kill_all_ros2.sh` | 优雅终止匹配到的 ROS 2 相关进程，残留进程会在等待后强制结束。 |
| `kill_all_ros2.sh --force` | 直接强制结束匹配到的 ROS 2 相关进程。 |

每个脚本的更详细说明在 `docs/` 目录：

- `docs/start_go2_slam_nav2.md`
- `docs/start_go2_nav2_localization.md`
- `docs/save_2dmap.md`
- `docs/save_pointcloud.md`
- `docs/get_go2_map_pose.md`

## 常用流程

第一次建图：

```bash
./start_go2_slam_nav2.sh --mapping
```

建图完成后另开终端保存 2D 地图和 PCD：

```bash
./save_2dmap.sh
./save_pointcloud.sh
```

使用已有地图导航：

```bash
./start_go2_nav2_localization.sh
```

查看当前地图坐标：

```bash
./get_go2_map_pose.sh
```

让机器狗按当前朝向前进 3 米：

```bash
./get_go2_map_pose.sh forward 3 --send
```

## 主要话题和坐标系

- `/utlidar/cloud_deskewed`：GO2 雷达点云输入，点云转 LaserScan 和 PCD 初始定位会使用。
- `/scan`：由点云转换得到的 LaserScan，供 SLAM/Nav2 使用。
- `/map`：slam_toolbox 或 map_server 发布的 2D 栅格地图。
- `/go2_built_map`：点云建图输出，`save_pointcloud.sh` 会保存它。
- `/cmd_vel`：导航速度命令入口，最终由 GO2 桥接到 Unitree 控制接口。
- `map -> odom -> base_footprint`：导航定位需要的主要 TF 链路。

## 注意事项

- 运行脚本前必须先完成 `colcon build`，否则脚本会因为找不到 `install/setup.bash` 退出。
- 使用 `--send` 发送导航目标前，确认 Nav2 已 active，周围环境安全。
- 如果 CycloneDDS participant index 报错，脚本会尝试自动补充 `MaxAutoParticipantIndex=100`；自定义 `CYCLONEDDS_URI` 时仍可能需要手动配置。
- 如果 AMCL 没有初始位姿且没有可用 PCD，需在 RViz 中手动使用 `2D Pose Estimate`。
