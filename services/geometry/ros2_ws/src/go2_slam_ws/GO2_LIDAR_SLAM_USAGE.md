# Go2 LiDAR SLAM 启动脚本使用说明

本文档说明 `start_go2_lidar_slam.sh` 的常用启动方式、RTAB-Map 定位/建图模式、RViz 观察方法和排障命令。

脚本路径：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh
```

## 前置条件

先确认 Go2 LiDAR 和里程计 topic 有数据：

```bash
ros2 topic echo /utlidar/cloud_base --once
ros2 topic echo /utlidar/robot_odom --once
```

如果 workspace 还没编译：

```bash
cd services/geometry/ros2_ws/src/go2_slam_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select go2_slam
```

## 关键概念

`/utlidar/robot_odom` 是 Unitree 原始局部里程计，不会被 RTAB-Map 改写。RTAB-Map 的重定位结果体现在 TF：

```text
map -> odom       RTAB-Map 发布，全局校正
odom -> base_link odom_to_tf.py 从 /utlidar/robot_odom 转换
map -> base_link  最终 map 坐标系下机器人位姿
```

为了方便 RViz 观察，脚本会额外发布校正后的 Odometry topic：

```text
/rtabmap/localization_odom
```

这个 topic 表示 `map -> base_link`，才是应该在 RViz 中观察的全局定位结果。

## 常用命令

### 新建地图

指定一个新的 db 路径并清空同名旧库：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh \
  --new-map \
  --dataset-path runtime_data/geometry/rtabmap/my_new_map.db
```

`--dataset-path` 和 `--database-path` 等价。

### 旧图定位

加载已有 db，只定位，不扩图：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh \
  --localization \
  --dataset-path runtime_data/geometry/rtabmap/my_new_map.db
```

如果机器人就在建图起点附近启动，可以加：

```bash
--start-at-origin
```

如果知道大概初始位姿，也可以给：

```bash
--initial-pose "x y yaw"
```

例如：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh \
  --localization \
  --dataset-path runtime_data/geometry/rtabmap/my_new_map.db \
  --initial-pose "1.2 0.5 1.57"
```

### 扩增旧地图

推荐使用 `--mapping`：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh \
  --mapping \
  --dataset-path runtime_data/geometry/rtabmap/my_new_map.db
```

当 db 已存在时，脚本不会直接盲目扩图，而是：

1. 先以 localization 模式启动；
2. 等 `/info` 中出现非零 `proximity_detection_id` 或 `loop_closure_id`；
3. 自动调用 RTAB-Map 的 mapping 服务；
4. 切换成功后继续扩图。

日志中看到下面两行才表示已经从定位切到扩图：

```text
RTAB-Map relocalization match detected ...
RTAB-Map switched to mapping mode.
```

如果想跳过“先重定位再扩图”，直接进入 RTAB-Map mapping 模式：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh \
  --direct-mapping \
  --dataset-path runtime_data/geometry/rtabmap/my_new_map.db
```

通常不推荐 `--direct-mapping` 扩旧图，除非你确认当前 odom 起点和旧地图坐标基本一致。

## RViz 正确观察方式

不要用 `/utlidar/robot_odom` 判断全局定位，它只是原始局部 odom。

RViz 推荐配置：

```text
Fixed Frame: map
Odometry topic: /rtabmap/localization_odom
```

也可以打开 TF，观察：

```text
map -> odom -> base_link
```

命令行查看校正后位姿：

```bash
ros2 topic echo /rtabmap/localization_odom
```

查看 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 判断是否重定位成功

RTAB-Map 这个 launch 没有 namespace，所以 info topic 是：

```bash
/info
```

查看 LiDAR proximity 重定位：

```bash
ros2 topic echo /info | grep proximity_detection_id
```

出现非零值表示当前点云匹配到了旧图节点，例如：

```text
proximity_detection_id: 75
```

查看视觉/词袋回环：

```bash
ros2 topic echo /info | grep loop_closure_id
```

LiDAR-only 配置下 `loop_closure_id` 常常为 `0`，更应关注 `proximity_detection_id`。

真正生效时，`map -> odom` 会有校正：

```bash
ros2 run tf2_ros tf2_echo map odom
```

## 参数速查

| 参数 | 作用 |
| --- | --- |
| `--dataset-path PATH` | 指定 RTAB-Map db 路径，等价于 `--database-path` |
| `--new-map` | 删除/覆盖指定 db，重新建图 |
| `--localization` | 加载旧图定位，不继续保存新节点 |
| `--mapping` | 已有 db 时先重定位，成功后自动切 mapping 扩图 |
| `--direct-mapping` | 直接进入 mapping，不等待重定位 |
| `--start-at-origin` | localization 时假设机器人从地图原点启动 |
| `--resume-last-localization` | localization 时使用 db 中保存的上次定位位姿 |
| `--initial-pose "x y yaw"` | 手动指定初始位姿猜测 |
| `--planar-odom` | 默认，只保留 odom 的 x/y/yaw，适合地面机器人 |
| `--full-6dof-odom` | 保留 Unitree odom 的完整 6DoF |
| `--wait-inputs` | 启动前等待点云和 odom 有消息 |
| `--input-timeout SEC` | `--wait-inputs` 的等待秒数 |
| `--optimize-max-error N` | RTAB-Map 定位图优化误差阈值，默认 `20.0` |
| `--max-odom-cache-size N` | RTAB-Map localization odom cache，默认 `0` |

## 常用环境变量

| 环境变量 | 默认值 |
| --- | --- |
| `NUWAX_RTABMAP_DATABASE_ROOT` | `runtime_data/geometry/rtabmap` |
| `NUWAX_RTABMAP_DATABASE_PATH` | `${DB_ROOT}/go2_lidar_slam.db` |
| `NUWAX_GEOMETRY_POINT_CLOUD_TOPIC` | `/utlidar/cloud_base` |
| `NUWAX_GEOMETRY_ODOM_TOPIC` | `/utlidar/robot_odom` |
| `NUWAX_GEOMETRY_MAP_FRAME` | `map` |
| `NUWAX_GEOMETRY_ODOM_FRAME` | `odom` |
| `NUWAX_GEOMETRY_BASE_FRAME` | `base_link` |
| `NUWAX_GO2_SLAM_LOCALIZATION_ODOM_TOPIC` | `/rtabmap/localization_odom` |
| `NUWAX_GO2_SLAM_MODE` | `auto` |
| `NUWAX_GO2_SLAM_PLANAR_ODOM` | `true` |
| `NUWAX_GO2_SLAM_START_AT_ORIGIN` | `false` |
| `NUWAX_GO2_SLAM_OPTIMIZE_MAX_ERROR` | `20.0` |
| `NUWAX_GO2_SLAM_MAX_ODOM_CACHE_SIZE` | `0` |

## 常见问题

### 为什么 `/utlidar/robot_odom` 看起来还是错的？

这是正常的。`/utlidar/robot_odom` 是原始局部里程计，RTAB-Map 不会改写它。请看：

```text
/rtabmap/localization_odom
```

或 RViz 中 `map` fixed frame 下的 `base_link`。

### 为什么 `--mapping` 一开始显示 `Mem/IncrementalMemory=false`？

这是设计行为。已有 db 时，脚本会先进入 localization，等重定位成功后自动调用 mapping 服务。看到：

```text
RTAB-Map switched to mapping mode.
```

之后才开始扩图。

### 为什么 `loop_closure_id` 一直是 0？

当前是 LiDAR-only，没有图像输入，bag-of-words 回环被禁用。请看：

```bash
ros2 topic echo /info | grep proximity_detection_id
```

### 如何保存完整日志？

建议同时保存 stdout 和 stderr：

```bash
services/geometry/ros2_ws/src/go2_slam_ws/start_go2_lidar_slam.sh \
  --mapping \
  --dataset-path runtime_data/geometry/rtabmap/my_new_map.db \
  > huang_log.md 2>&1
```

### 路径不要断行

下面这种路径断开会导致 shell 解析错误：

```text
my_new_map.
db
```

应该写成：

```text
my_new_map.db
```
