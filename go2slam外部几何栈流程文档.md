# external_slam_nav2 外部 SLAM/Nav2 几何栈流程文档

> **分支说明**
>
> - **main 分支**：默认使用 `services/geometry/start_rtabmap_sidecar.sh` 作为外部 SLAM/Nav2 脚本（RTAB-Map + Nav2）。
> - **go2_inner_slam 分支**：`services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh`（slam_toolbox + Nav2）仅在此分支可用。
>
> 本文档中出现的 `start_go2_slam_nav2.sh` 引用保留作为架构参考，**实际部署请使用 `services/geometry/start_rtabmap_sidecar.sh`**。

本文档说明如何让网关托管外部 SLAM/Nav2 脚本，并使用该脚本提供的 `/map`、TF（坐标变换树）和 Nav2（导航栈）。当前默认脚本仍位于 `services/geometry/ros2_ws/src/go2slam/`，平台配置栈名统一为 `external_slam_nav2`。

## 适用场景

当 `services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh` 已经负责启动外部驱动、robot_localization、点云转 `/scan`、slam_toolbox 和 Nav2 时，平台必须使用 `external_slam_nav2` 外部几何栈模式。

这个模式下：

- 网关只托管启动外部 SLAM/Nav2 脚本。
- 网关不再启动平台内置 RTAB-Map。
- 网关不再启动平台内置 Nav2。
- `start_go2_slam_nav2.sh` 会启动外部脚本自己的 Go2 driver，由它提供 `/utlidar/cloud_deskewed`、`/utlidar/robot_pose`、`/odom`、点云转 `/scan`、slam_toolbox 和 Nav2。
- 默认不要开启网关平台输入桥，也不要开启网关点云桥。避免网关点云桥和外部脚本的 `go2_driver` 同时发布同名 `/utlidar/cloud_deskewed`。
- 只有当外部脚本没有独立 Unitree ROS2 driver、确实缺少外部栈输入话题时，才开启平台输入桥。
- 网关仍会观察 `/map`、`/tf`、`/tf_static`，并通过 `/navigate_to_pose` 对接外部 Nav2。
- 不使用 `ROS_DOMAIN_ID=1`。

## 环境配置

`.env` 中使用下面这组配置：

```dotenv
NUWAX_GEOMETRY_ENABLED=true
NUWAX_GEOMETRY_STACK=external_slam_nav2
NUWAX_GEOMETRY_REQUIRE_ROS2=true
NUWAX_GEOMETRY_SETUP_SCRIPT=/home/huang/nuwax-robot-bridge/services/geometry/ros2_ws/install/setup.bash
NUWAX_GEOMETRY_ODOM_TOPIC=/odom
NUWAX_GEOMETRY_POINT_CLOUD_TOPIC=/utlidar/cloud_deskewed
NUWAX_GEOMETRY_EXTERNAL_SCRIPT=/home/huang/nuwax-robot-bridge/services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh
NUWAX_GEOMETRY_EXTERNAL_ARGS=--no-rviz
NUWAX_GEOMETRY_EXTERNAL_PROCESS_NAME=external_slam_nav2
NUWAX_GEOMETRY_EXTERNAL_LOG_NAME=external_slam_nav2.log
NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED=false
NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED=false
NUWAX_NAV2_MODE=external
NUWAX_NAV2_ACTION_NAME=/navigate_to_pose
NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map
NUWAX_GEOMETRY_COST_MAP_TOPIC=/global_costmap/costmap
NUWAX_GEOMETRY_MAP_FRAME=map
NUWAX_GEOMETRY_ODOM_FRAME=odom
NUWAX_GEOMETRY_BASE_FRAME=base_footprint
```

`base_frame` 必须用 `base_footprint`，因为当前默认外部脚本的主 TF 链是：

```text
map -> odom -> base_footprint
```

不要在根 `.env` 全局设置 `RMW_IMPLEMENTATION`。`RMW_IMPLEMENTATION` 应由外部 SLAM/Nav2 启动脚本在 ROS2 子进程内设置，否则会影响网关主进程里的 Unitree SDK 初始化。

## DDS / RMW 边界

`start_go2_slam_nav2.sh` 内部默认设置：

```bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
```

这样只有外部 go2slam 脚本及其启动的 ROS2 子进程使用 CycloneDDS。不要把下面这行放进根 `.env`：

```dotenv
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

原因：

- 外部 SLAM/Nav2 的 ROS2 侧需要 `rmw_cyclonedds_cpp` 才能稳定接通 Unitree/Go2 的 `/utlidar/robot_pose`、`/utlidar/cloud_deskewed` 等链路。
- 网关主进程不应继承这个 RMW 设置；全局设置后可能导致 Unitree SDK `ChannelFactoryInitialize` 或官方服务访问异常。
- `CYCLONEDDS_URI` 和 `NUWAX_GEOMETRY_DDS_IFACE` 可以继续用于指定网卡，但 RMW 实现边界必须留在外部脚本内。

## 启动流程

1. 启动网关。
2. `PlatformGeometryRuntime` 识别 `NUWAX_GEOMETRY_STACK=external_slam_nav2`。
3. 网关启动 `/map` 和 TF 观察器。
4. 默认不启动平台 ROS2 odom 发布桥、点云发布桥和 `external_odom_tf_bridge`。
5. 网关托管启动：

```bash
/home/huang/nuwax-robot-bridge/services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh --no-rviz
```

6. 外部 SLAM/Nav2 脚本在子进程内使用 `rmw_cyclonedds_cpp` 启动 Go2 driver、cloud_accumulation、pointcloud_to_laserscan、slam_toolbox、Nav2。
7. 点云链路应为：

```text
/utlidar/cloud_deskewed -> cloud_accumulation -> /trans_cloud -> pointcloud_to_laserscan -> /scan -> slam_toolbox -> /map
```

8. slam_toolbox 发布 `/map`。
9. Nav2 提供 `/navigate_to_pose` action。
10. 网关 runtime 状态在 `/map` 连续更新、TF 可解析、Nav2 ready 后进入 ready。

## 重新编译 go2_perception

如果修改了 `cloud_accumulation.cpp` 或点云转换节点，需要重编外部 SLAM/Nav2 工作空间里的 `go2_perception`：

```bash
source /opt/ros/jazzy/setup.bash
cd services/geometry/ros2_ws/src/go2slam
colcon build --symlink-install --packages-select go2_perception --cmake-clean-cache
```

如果不加 `--cmake-clean-cache`，旧 CMake cache 可能仍指向历史路径，导致二进制没有可靠更新。

## 验收命令

查看托管脚本日志：

```bash
tail -f runtime_data/geometry/logs/external_slam_nav2.log
```

确认 `/map` 有数据：

```bash
source /opt/ros/jazzy/setup.bash
source services/geometry/ros2_ws/src/go2slam/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /map --once
```

确认外部 SLAM/Nav2 输入链路有数据：

```bash
ros2 topic hz /utlidar/cloud_deskewed
ros2 topic hz /trans_cloud
ros2 topic hz /scan
ros2 topic hz /map
```

当前验证过的正常参考值：

```text
/utlidar/cloud_deskewed 约 15 Hz
/trans_cloud            约 40 Hz
/scan                   约 40 Hz
/map                    约 2.5 Hz
```

确认 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
```

确认 Nav2 action：

```bash
ros2 action list | grep navigate_to_pose
```

确认网关没有启动内置 RTAB-Map/Nav2：

```bash
grep -E "rtabmap|nav2|external_slam_nav2" runtime_data/geometry/logs/*.log
```

正常情况下，外部模式只应看到 `external_slam_nav2.log` 和可选的 `external_odom_tf_bridge.log` 由网关托管启动；不应出现新的平台 `rtabmap_live.log` 或平台 `nav2.log` 启动记录。

确认网关没有启动平台点云桥：

```bash
grep "ROS2 点云发布桥已启动" runtime_data/geometry/logs/*.log nuwax_robot_bridge.log
ps -ef | grep nuwax_pointcloud_publisher
```

使用 `start_go2_slam_nav2.sh` 的默认模式时，新启动日志里不应再出现：

```text
ROS2 点云发布桥已启动 point_cloud_topic=/utlidar/cloud_deskewed
```

## 常见问题

### 启动时报外部脚本不存在

检查：

```bash
ls -l /home/huang/nuwax-robot-bridge/services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh
```

如果不可执行：

```bash
chmod +x /home/huang/nuwax-robot-bridge/services/geometry/ros2_ws/src/go2slam/start_go2_slam_nav2.sh
```

### 网关一直等待 TF

确认 `.env` 中是：

```dotenv
NUWAX_GEOMETRY_BASE_FRAME=base_footprint
```

再检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

如果 `odom -> base_footprint` 和 `/utlidar/cloud_deskewed` 已由外部 SLAM/Nav2 原脚本提供，保持：

```dotenv
NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED=false
NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED=false
NUWAX_GEOMETRY_POINT_CLOUD_TOPIC=/utlidar/cloud_deskewed
```

不要只因为 `/map` 没动就打开 `NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED=true`。先确认外部脚本的 `go2_driver` 是否还在发布 `/utlidar/cloud_deskewed`：

```bash
ros2 topic info /utlidar/cloud_deskewed -v
```

如果同时出现外部脚本的 `driver` publisher 和网关的 `nuwax_pointcloud_publisher` publisher，说明同名点云话题被双发布，需要关闭网关点云桥。

如果 `odom -> base_footprint` 不存在，同时机器上没有独立 Unitree ROS2 驱动发布 `/utlidar/robot_pose` 和 `/utlidar/cloud_deskewed`，再开启完整平台输入桥：

```dotenv
NUWAX_GEOMETRY_EXTERNAL_INPUT_BRIDGE_ENABLED=true
NUWAX_GEOMETRY_EXTERNAL_POINT_CLOUD_BRIDGE_ENABLED=true
NUWAX_GEOMETRY_ODOM_TOPIC=/odom
NUWAX_GEOMETRY_POINT_CLOUD_TOPIC=/utlidar/cloud_deskewed
NUWAX_GEOMETRY_BASE_FRAME=base_footprint
```

### 网关一直等待地图

确认 `external_slam_nav2.log` 中 slam_toolbox 已启动，再检查：

```bash
ros2 topic list | grep '^/map$'
ros2 topic echo /map --once
```

如果 `/map` 有 publisher 但 `echo --once` 没有任何输出，继续检查外部 SLAM/Nav2 输入源：

```bash
ros2 topic info /utlidar/robot_pose -v
ros2 topic info /utlidar/cloud_deskewed -v
ros2 topic echo /odom --once
ros2 topic echo /scan --once
```

`/utlidar/robot_pose`、`/utlidar/cloud_deskewed`、`/odom` 或 `/scan` 没有数据时，slam_toolbox 不会稳定产出 `/map`。

如果 ROS2 和网关已经有地图，但 `http://127.0.0.1:8766/debug` 仍显示“等待地图数据”，先确认网关数据流是否能直接拿到地图帧：

```bash
timeout 8 curl -sS 'http://127.0.0.1:8766/streams/sse?topics=map.frame&after_cursor=999999999&max_frames=1&access_token=agent-robot-bridge-token'
```

期望返回 `event: stream.frame`，且 JSON 中 `topic` 为 `map.frame`、`status` 为 `available`、`occupancy_grid.width/height` 大于 0。

这个场景通常是浏览器 WebSocket 重连时带了旧网关进程的 `after_cursor`。当前网关会在服务端检测到 `after_cursor` 大于当前 `latest_cursor` 时自动归零并重放 retained frame；如果仍是旧代码，刷新页面或重启网关后页面可能继续等待地图，需要同步包含该修复的 `/streams/ws`、`/streams/sse` 游标归一化逻辑。

### Nav2 不 ready

确认外部模式没有写成 sidecar：

```dotenv
NUWAX_NAV2_MODE=external
```

再检查：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
```

### `/map` 有 revision 但画面不变

网关日志中的 `mapping.snapshot_ready` revision 递增，只代表网关收到了 `/map` 消息，不代表 SLAM 真的吃到了新的 `/scan`。如果画面不变，按链路从上游查：

```bash
ros2 topic hz /utlidar/cloud_deskewed
ros2 topic hz /trans_cloud
ros2 topic hz /scan
ros2 topic hz /map
```

如果 `/utlidar/cloud_deskewed` 有数据，但 `/trans_cloud` 或 `/scan` 没有数据，重点检查 `cloud_accumulation` 是否退出：

```bash
ps -ef | grep cloud_accumulation
grep -n "cloud_accumulation.*process has died" runtime_data/geometry/logs/external_slam_nav2.log | tail
```

已确认过的故障形态：

```text
[ERROR] [cloud_accumulation-7]: process has died [..., exit code -11, cmd '.../cloud_accumulation ...']
```

含义：

- `exit code -11` 是段错误。
- 常见原因是网关点云桥和外部脚本 `go2_driver` 同时发布 `/utlidar/cloud_deskewed`，`cloud_accumulation` 收到混合 PointCloud2 格式后按错误字段布局解析。
- 修复方式是关闭网关点云桥，并使用带字段名和边界检查的 `cloud_accumulation.cpp`，然后重编 `go2_perception`。

修复后应看到：

```text
cloud_accumulation 进程常驻
/trans_cloud 约 40 Hz
/scan 约 40 Hz
/map 约 2.5 Hz
```

### `cloud_accumulation` 修改点

当前 `cloud_accumulation.cpp` 不再硬编码 `cloud.fields[2]` 作为 z 字段，而是：

- 按字段名查找 `FLOAT32 z`。
- 使用 `point_step` 和 `cloud.data.size()` 计算有效点数。
- 合并多帧点云前检查字段布局是否一致。
- 遇到不兼容 PointCloud2 帧时跳过并节流告警，不让节点段错误退出。

这能防住同名点云话题混入不同 PointCloud2 布局时的崩溃，但正确配置仍然是避免双发布。
