# Unitree GO2 — SLAM · Nav2 · 自主探索

基于 **ROS 2 Jazzy** 的 Unitree GO2 真机方案：RTAB-Map 激光 SLAM、Nav2 导航、m-explore 自主探索，以及 **路线 B**（2D/3D 障碍对齐）。

---

## 功能概览

| 功能 | 说明 |
|------|------|
| 激光建图 | RTAB-Map + GO2 机载 `/utlidar/cloud_deskewed` |
| DB 定位 | `go2.sh --localization` 加载已有地图，不扩图 |
| 全局重定位 | 定位模式自动启用 `ProximityGlobalScanMap`（激光 ICP 对齐 DB） |
| 路线 B | `/map` + `/cloud_obstacles` → `/map_obstacles`，2D 与 3D 一致 |
| 导航 | Nav2 全局代价地图订阅 `/map_obstacles` |
| 探索 | m-explore（`explore_lite`）基于 `/map_obstacles` |
| 运动约束 | 禁止倒车/侧移，最低速度 0.3 m/s（`go2_cmd_processor`） |

---

## 系统架构

```
GO2 机载
  /utlidar/cloud_deskewed  +  /utlidar/robot_odom
           │
           ▼
  sensor_stamp_relay  →  /go2/cloud , /go2/odom , /go2/tf
           │
           ▼
  RTAB-Map (rtabmap)  →  /map , /cloud_obstacles , /cloud_map ...
           │
           ▼
  obstacle_grid_projector (路线 B)  →  /map_obstacles
           │
           ├─► RViz Map2D_RouteB
           ├─► Nav2 global_costmap (static_layer)
           └─► m-explore
```

**原始 RTAB 栅格** `/map` 仍在 RViz 的 `Map2D_Raw` 显示，仅作参考；规划与探索统一使用 `/map_obstacles`。

---

## 环境要求

- Ubuntu 24.04 + **ROS 2 Jazzy**
- Unitree ROS2：[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)（CycloneDDS）
- RTAB-Map：`sudo apt install ros-jazzy-rtabmap-ros`
- Nav2：`sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup`
- GO2 通过 USB 以太网连接（`192.168.123.x`，接口名通常为 `enx*`）

### 机载话题（需正常发布）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/utlidar/cloud_deskewed` | PointCloud2 | 运动补偿点云（odom 系） |
| `/utlidar/robot_odom` | Odometry | 机载里程计 |
| `/utlidar/imu` | Imu | ICP 模式备用 |

---

## 工作区搭建

推荐目录（可按需修改）：

```
~/huang_grok/
├── unitree-go2-slam-nav2/    # 本仓库（源码）
├── ws/                       # colcon 工作区
│   ├── src/ → 软链到本仓库各 package
│   └── source_go2.bash       # 环境加载脚本
└── go2.sh                    # 一键启动（也可使用 scripts/go2.sh）
```

### 1. 创建工作区并链接源码

```bash
mkdir -p ~/huang_grok/ws/src
cd ~/huang_grok/ws/src

ln -sfn ~/huang_grok/unitree-go2-slam-nav2/go2_slam_nav .
ln -sfn ~/huang_grok/unitree-go2-slam-nav2/go2_cmd_processor .
# 探索（m-explore）
ln -sfn ~/huang_grok/m-explore-ros2/explore explore_lite
ln -sfn ~/huang_grok/m-explore-ros2/explore_lite_msgs .
```

### 2. 编译

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash

cd ~/huang_grok/ws
colcon build --symlink-install
```

> **注意**：编译前不要 source 旧的 `~/go2_slam_nav_ws`，否则会链入过期 underlay，导致找不到 `sensor_stamp_relay` 或 `rtabmap_slam`。

### 3. 加载环境（每次新终端）

```bash
source ~/huang_grok/ws/source_go2.bash
```

脚本会自动绑定 CycloneDDS 到 `192.168.123.x` 网卡（或 `GO2_ROBOT_IFACE`）。**不要**只 `source /opt/ros/jazzy/setup.bash` 就查机载话题。

验证：

```bash
ros2 pkg prefix go2_slam_nav      # → ~/huang_grok/ws/install/go2_slam_nav
ros2 pkg prefix rtabmap_slam      # → /opt/ros/jazzy
ls $(ros2 pkg prefix go2_slam_nav)/lib/go2_slam_nav/
# 应包含: sensor_stamp_relay  obstacle_grid_projector  go2_tf_relay
```

---

## 快速启动

### 方式一：`go2.sh`（推荐）

```bash
# 建图 + Nav2 + 2D 目标点（路线 B 默认开启）
~/huang_grok/go2.sh --fresh --mapping

# 仅建图（无 Nav2，负载更低）
~/huang_grok/go2.sh --mapping-only

# 加载已有地图定位 + 导航
~/huang_grok/go2.sh --localization ~/maps/room1.db

# 建图 + 导航 + 自主探索
~/huang_grok/go2.sh --explore
```

| 参数 | 说明 |
|------|------|
| `--fresh` | 强制 `restart_map:=true`，清空 DB 重新建图 |
| `--mapping [db]` | SLAM + Nav2，默认 `~/.ros/rtabmap.db` |
| `--mapping-only [db]` | 纯 SLAM，关闭 `obstacle_grid` |
| `--localization <db>` | 只定位不扩图，DB 只读拷贝；**自动全局激光重定位** |
| `--explore [db]` | SLAM + Nav2 + m-explore |

环境变量：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `GO2_WS` | `~/huang_grok/ws` | colcon 工作区 |
| `GO2_ROBOT_IFACE` | 自动检测 | Unitree 网卡名 |
| `GO2_CLOUD_MODE` | `deskewed` | 改为 `base` 使用 `/utlidar/cloud_base` |
| `GO2_SKIP_KILL` | `0` | 设为 `1` 跳过启动前杀旧进程 |

### 方式二：`ros2 launch`

```bash
source ~/huang_grok/ws/source_go2.bash

# 建图（路线 B + RViz）
ros2 launch go2_slam_nav mapping_go2.launch.py restart_map:=true

# 建图 + Nav2
ros2 launch go2_slam_nav nav_go2.launch.py restart_map:=true

# 自主探索
ros2 launch go2_slam_nav explore_go2.launch.py restart_map:=true

# 加载 DB 定位 + Nav2（等同 go2.sh --localization）
ros2 launch go2_slam_nav nav_go2.launch.py localize_only:=true restart_map:=false
```

---

## DB 定位与全局重定位

`go2.sh --localization <map.db>` 会：

1. 将源 DB **只读拷贝**到 `~/.ros/rtabmap_localization_session.db`（不修改原始地图）
2. 设置 `localize_only:=true`（`Mem/IncrementalMemory=false`，不增图）
3. **仅在此模式下**开启全局激光重定位参数（建图模式保持关闭，减少告警与负载）

| 参数 | 建图 / `--explore` | `--localization` |
|------|-------------------|-------------------|
| `RGBD/ProximityBySpace` | `true` | `true` |
| `RGBD/ProximityPathMaxNeighbors` | `10` | `10` |
| `RGBD/ProximityMaxGraphDepth` | `50` | `50` |
| `RGBD/ProximityGlobalScanMap` | `false` | `true`（全局重定位） |

建图/探索：走过已扫区域时靠 **局部 ICP 邻近回环** 拉正位姿。  
定位：额外启用 **全局激光地图** 匹配，支持任意起点重定位。

### 启动后会发生什么

- 第一帧会先用 DB 里**上次关机保存的位姿**初始化 `map→go2_odom`（TF 会动，但绝对位置可能不对）
- RTAB-Map 在后台构建全局激光地图，随后在已建图区域做 ICP 匹配
- 匹配成功后 `map→go2_odom` 会被拉回真实位置

### 推荐操作

```bash
source ~/huang_grok/ws/source_go2.bash
./go2.sh --localization ~/maps/room1.db

# 慢速开过有墙/障碍的已建图区域，观察重定位
ros2 topic echo /rtabmap/info --field proximity_detection_id
ros2 topic echo /rtabmap/info --field loop_closure_id
```

`proximity_detection_id` 或 `loop_closure_id` **大于 0**，且 RViz 里机器人与真实环境对齐，即重定位成功。

### 手动初始位姿（兜底）

若环境特征少、匹配较慢，可在 RViz 使用 **2D Pose Estimate**（话题 `/initialpose`）点选已知位置。

### 常见误解

| 现象 | 含义 |
|------|------|
| 建图时 `proximity_detection_id` 一直为 0 | 尚未重访已扫区域；走过熟悉路段后可能变非 0 |
| 定位启动后 TF 在动但位置不对 | 尚未完成全局对齐；慢速行驶或手给 `/initialpose` |
| TF 在动 | 说明 odom + `mapCorrection` 链路正常，不等于绝对位置已对准 |

---

## Launch 文件说明

| 文件 | 用途 |
|------|------|
| `mapping_go2.launch.py` | **真机建图入口**（不要用 `mapping.launch.py`，它需要 `rslidar_sdk`） |
| `nav_go2.launch.py` | 建图/定位 + Nav2 |
| `explore_go2.launch.py` | 建图 + Nav2 + m-explore |
| `rtab_lidar_go2.launch.py` | 底层：relay、RTAB-Map、路线 B、RViz；`localize_only` 时切换重定位参数 |

---

## 路线 B（`/map_obstacles`）

RTAB-Map 的 `/map`（2D 栅格）与 `/cloud_obstacles`（3D 障碍点云）走不同流水线，直接叠加可能对不齐。

**路线 B** 由 `obstacle_grid_projector` 实现：

1. 以 RTAB-Map `/map` 的自由空间为底图
2. 将 `/cloud_obstacles` 投影到 2D 栅格
3. 输出 `/map_obstacles` 与 `/map_obstacles_updates`

配置：`config/obstacle_grid_go2.yaml`

```yaml
publish_rate: 5.0        # Hz
max_cloud_points: 25000  # 投影点数上限
```

关闭路线 B（最轻建图）：

```bash
ros2 launch go2_slam_nav mapping_go2.launch.py use_obstacle_grid:=false
```

---

## Nav2 与地图

`config/nav2_params_go2.yaml` 中 **global_costmap** 的 `static_layer` 订阅 `/map_obstacles`（路线 B），不是原始 `/map`。

| 层级 | 数据来源 | 作用 |
|------|----------|------|
| global static_layer | `/map_obstacles` | 全局路径规划 |
| global/local obstacle | `/go2/cloud` | 实时 3D 障碍 |
| local costmap | `/go2/cloud`（voxel） | 近距离动态避障 |

Nav2 行为树：`config/behavior_trees/navigate_to_pose_go2.xml`（含 `GoalUpdatedController`，避免同目标空转）。

---

## RViz 图层

`config/mapping_go2.rviz` 默认显示：

| 图层 | 话题 | 说明 |
|------|------|------|
| Map2D_Raw | `/map` | RTAB 原始 2D 图（半透明背景） |
| Map2D_RouteB | `/map_obstacles` | 路线 B 融合图（前景） |
| CloudMap | `/cloud_map` | 累积 3D 点云地图 |
| LiveScan | `/go2/cloud` | 当前帧（默认关闭） |

---

## 性能参考

| 模式 | CPU 负载 | 说明 |
|------|----------|------|
| `mapping_go2` + 路线 B | 中（约 40–70%） | 日常建图推荐 |
| `--mapping-only` 无路线 B | 低～中 | 最快扫图 |
| `--mapping`（含 Nav2） | 中～高（约 70–90%） | 建图同时试导航 |

主要算力在 RTAB-Map；路线 B 额外约 10–20%。邻近 ICP 回环再增加约 10–15%。已关闭视觉特征管道（纯激光）。

---

## 常见问题

### 1. `package 'rtabmap_slam' not found`

未正确 source Jazzy。使用 `source_go2.bash`，不要手动 `unset AMENT_PREFIX_PATH`。

### 2. 节点是 `/rtabmap_reset`，没有 `/sensor_stamp_relay`

加载了旧工作区 `~/go2_slam_nav_ws`。执行：

```bash
source ~/huang_grok/ws/source_go2.bash
ros2 pkg prefix go2_slam_nav   # 必须指向 huang_grok/ws
```

### 3. RViz `Map2D_RouteB` 显示 "No map received"

```bash
ros2 node list | grep obstacle_grid   # 应有 /obstacle_grid_projector
ros2 topic hz /map_obstacles
```

若节点不存在，确认 `use_obstacle_grid:=true` 且已重新编译。

### 4. `rtabmap: Did not receive data /go2/cloud`

```bash
ros2 topic hz /utlidar/cloud_deskewed
ros2 topic hz /go2/cloud
```

检查 `sensor_stamp_relay` 日志；可尝试 `GO2_CLOUD_MODE=base ./go2.sh --mapping`。

### 5. CycloneDDS 网卡错误

USB 以太网未连接或接口名变化。设置 `GO2_ROBOT_IFACE=enxXXXXXXXX` 或检查 `~/unitree_ros2/setup.sh` 中的 `CYCLONEDDS_URI`。

### 6. 启动前清理旧进程

```bash
pgrep -f 'ros2 launch go2_slam_nav|lib/rtabmap_slam/rtabmap|sensor_stamp_relay|obstacle_grid' | xargs -r kill -9
# 不要用 pgrep -f 'rtabmap'，会误杀带 rtabmap.db 路径的 go2.sh
```

### 7. `--localization` 后机器人在地图上位置不对

定位启动时默认锚在 DB **关机位姿**，若当前物理位置不同，地图上会偏移。

**处理：**

1. 慢速开过已建图区域，等待 `proximity_detection_id` 或 `loop_closure_id` > 0
2. 或在 RViz 用 **2D Pose Estimate** 发布 `/initialpose`
3. 确认参数：`localize_only=true` 时 rtabmap 进程应带 `RGBD/ProximityGlobalScanMap true`

```bash
pgrep -af 'lib/rtabmap_slam/rtabmap' | grep -o 'ProximityGlobalScanMap [^ ]*'
```

### 8. `proximity_detection_id` 一直为 0

- **建图/探索**：需**重访已建图区域**才会触发；新区域一直为 0 正常
- **定位模式**：启动后需几秒构建全局激光图；在特征少的区域可能更久。多走几圈或手给初始位姿

### 9. `ros2 topic list` 看不到 GO2 机载话题

只出现 `/parameter_events`、`/rosout` 或个别 `/utlidar/*`，通常是 **DDS 网卡未绑定** 或 **机器人侧节点未全起**。

```bash
# 1. 正确 source（会打印 DDS interface）
source ~/huang_grok/ws/source_go2.bash

# 2. 确认网线 / 网卡
ip -br addr | grep 192.168.123
ping -c 2 192.168.123.161

# 3. 刷新 ROS 发现
ros2 daemon stop && sleep 1
ros2 topic list | grep utlidar

# 4. 应有（正常开机、站立后）
# /utlidar/cloud_deskewed
# /utlidar/robot_odom
# /utlidar/imu
```

若只有 `/utlidar/cloud_synced`：机器人激光/里程计栈可能未完全启动——确认 GO2 已开机、站立（Sport 模式），等待 10–30 秒再试。

网卡名变化时（换 USB 口）：

```bash
export GO2_ROBOT_IFACE=enxXXXXXXXX
source ~/huang_grok/ws/source_go2.bash
```

也可用 Unitree 官方环境（效果相同）：

```bash
source ~/unitree_ros2/setup.sh
```

---

## 项目结构

```
unitree-go2-slam-nav2/
├── go2_slam_nav/              # 核心：SLAM、Nav2 launch、路线 B
│   ├── go2_slam_nav/
│   │   ├── sensor_stamp_relay.py
│   │   ├── obstacle_grid_projector.py   # 路线 B
│   │   └── go2_tf_relay.py
│   ├── config/
│   │   ├── nav2_params_go2.yaml
│   │   ├── explore_go2.yaml
│   │   ├── obstacle_grid_go2.yaml
│   │   └── mapping_go2.rviz
│   └── launch/
│       ├── mapping_go2.launch.py      # 真机建图入口
│       ├── nav_go2.launch.py
│       └── explore_go2.launch.py
├── go2_cmd_processor/         # GO2 运动控制（速度/方向约束）
├── frontier/                  # 旧探索模块（已由 explore_lite 替代）
└── scripts/
    ├── go2.sh                 # 一键启动
    └── source_go2.bash        # 环境加载 + 校验
```

---

## 诊断命令

```bash
# 节点
ros2 node list | grep -E 'sensor_stamp|rtabmap|obstacle_grid|nav2'

# 话题频率
ros2 topic hz /go2/cloud
ros2 topic hz /map
ros2 topic hz /map_obstacles

# TF
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map go2_base_link

# 定位 / 重定位状态（仅 --localization 有意义）
ros2 topic echo /rtabmap/info --field proximity_detection_id
ros2 topic echo /rtabmap/info --field loop_closure_id
pgrep -af 'lib/rtabmap_slam/rtabmap' | grep -E 'IncrementalMemory|ProximityGlobalScanMap'
```

---

## 致谢

本项目基于 [unitree-go2-slam-nav2](https://github.com/h-naderi/unitree-go2-slam-nav2)（Hossein Naderi）扩展，适配 Unitree GO2 真机、ROS 2 Jazzy、路线 B 融合地图与 m-explore 探索。