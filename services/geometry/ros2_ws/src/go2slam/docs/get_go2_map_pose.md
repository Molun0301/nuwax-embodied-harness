# get_go2_map_pose.sh 使用文档

`get_go2_map_pose.sh` 用来读取机器狗当前在地图中的坐标，也可以基于当前位置计算并发送一个简单的 Nav2 目标点。

脚本默认读取：

```text
map -> base_footprint
```

也就是 `base_footprint` 在 `map` 坐标系下的位置。导航、记录当前位置、基于当前位置前进几米，通常都应该使用这个坐标。

## 前置条件

运行脚本前，需要先启动 localization 或 SLAM：

```bash
./start_go2_nav2_localization.sh
```

或者：

```bash
./start_go2_slam_nav2.sh
```

并确认 TF 链路正常：

```text
map -> odom -> base_footprint
```

如果只启动了底层驱动但没有 AMCL/SLAM，通常只能得到 `odom -> base_footprint`，无法得到 `map -> base_footprint`。

## 查看当前位置

查看一次当前地图坐标：

```bash
./get_go2_map_pose.sh
```

输出示例：

```text
base_footprint in map: x=0.253 m, y=-0.052 m, z=0.003 m, yaw=134.7 deg
```

含义：

```text
x/y/z: 机器狗 base_footprint 在 map 中的位置，单位米
yaw: 机器狗当前朝向，单位度
```

持续查看当前位置：

```bash
./get_go2_map_pose.sh --watch
```

指定刷新频率，例如 5Hz：

```bash
./get_go2_map_pose.sh --watch --rate 5
```

## 计算前方目标点

按机器狗当前朝向，向前 3 米，只计算目标点，不发送：

```bash
./get_go2_map_pose.sh forward 3
```

输出示例：

```text
base_footprint in map: x=0.253 m, y=-0.052 m, z=0.003 m, yaw=134.7 deg
目标点: frame=map x=-1.857 m, y=2.081 m, yaw=134.7 deg
仅计算目标点；加 --send 才会发送给 Nav2。
```

`forward 3` 的意思是沿机器狗当前朝向前进 3 米，不一定等于 `map` 坐标系的 `x + 3`。

## 发送前方目标点

按机器狗当前朝向前进 3 米，并发送给 Nav2：

```bash
./get_go2_map_pose.sh forward 3 --send
```

这会发送 `NavigateToPose` action 到 Nav2：

```text
/navigate_to_pose
```

机器狗会真的开始导航。使用 `--send` 前请确认周围环境安全。

## 沿 map 坐标轴移动

如果你想严格沿 `map` 坐标系的 x 轴移动，而不是沿机器狗朝向移动，使用 `map-x`。

沿 `map` 的 x 轴正方向 3 米：

```bash
./get_go2_map_pose.sh map-x 3 --send
```

沿 `map` 的 x 轴负方向 3 米：

```bash
./get_go2_map_pose.sh map-x -3 --send
```

沿 `map` 的 y 轴正方向 3 米：

```bash
./get_go2_map_pose.sh map-y 3 --send
```

沿 `map` 的 y 轴负方向 3 米：

```bash
./get_go2_map_pose.sh map-y -3 --send
```

区别总结：

```text
forward 3   按机器狗当前朝向前进 3 米
map-x 3     按 map 坐标系 x 轴正方向移动 3 米
map-y 3     按 map 坐标系 y 轴正方向移动 3 米
```

## 常用参数

查看帮助：

```bash
./get_go2_map_pose.sh --help
```

指定固定坐标系，默认是 `map`：

```bash
./get_go2_map_pose.sh --fixed-frame map
```

指定机器人坐标系，默认是 `base_footprint`：

```bash
./get_go2_map_pose.sh --base-frame base_footprint
```

增加等待 TF/action 的超时时间，默认 5 秒：

```bash
./get_go2_map_pose.sh --timeout 10
```

显示 Unitree/CycloneDDS 的类型哈希警告：

```bash
GO2_SHOW_DDS_WARNINGS=1 ./get_go2_map_pose.sh
```

默认情况下，脚本会过滤类似下面的噪声警告：

```text
Failed to parse type hash
```

## 常见问题

### 无法获取 TF: map -> base_footprint

常见原因：

```text
1. 没有启动 start_go2_nav2_localization.sh 或 start_go2_slam_nav2.sh
2. AMCL 还没有收到初始位姿
3. SLAM 还没有建立 map -> odom
4. TF 链路缺失 odom -> base_footprint
```

可以先检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

如果这里也失败，说明不是脚本问题，而是 TF 链路还没有准备好。

### 找不到 /navigate_to_pose action server

说明 Nav2 的 `bt_navigator` 还没有 active，或者 Nav2 没有正常启动。

检查 Nav2 节点状态：

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

正常应该是：

```text
active [3]
```

### 使用 --send 后没有动

可能原因：

```text
1. 目标点不可达，被 costmap 或 planner 拒绝
2. Nav2 lifecycle 节点没有 active
3. /cmd_vel 没有被 go2_twist_bridge 正常转给机器狗
4. 机器狗当前处于不接受运动指令的模式
```

可以检查：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /cmd_vel --once
```

## 推荐工作流

先确认当前坐标：

```bash
./get_go2_map_pose.sh
```

再计算目标点，确认方向没问题：

```bash
./get_go2_map_pose.sh forward 3
```

最后再发送：

```bash
./get_go2_map_pose.sh forward 3 --send
```

这样可以避免把 `forward` 和 `map-x` 混淆，导致机器狗朝非预期方向移动。
