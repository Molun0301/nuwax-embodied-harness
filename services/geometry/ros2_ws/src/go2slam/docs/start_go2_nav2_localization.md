# start_go2_nav2_localization.sh

使用已有地图启动 Go2 的 AMCL 定位 + Nav2 导航。

## 用法

自动使用 `maps/` 下最新 2D 地图，并自动使用 `output/` 下最新 PCD 点云图：

```bash
./start_go2_nav2_localization.sh
```

指定 2D 地图：

```bash
./start_go2_nav2_localization.sh maps/my_map.yaml
```

同时指定 2D 地图和 PCD 点云图：

```bash
./start_go2_nav2_localization.sh maps/my_map.yaml output/my_map.pcd
```

## 会启动什么

```text
Go2 driver
robot_localization
点云转 LaserScan
map_server
AMCL
Nav2
RViz
```

如果找到 PCD 点云图，还会启动自动初始定位：

```text
pcd_initial_pose_publisher
```

PCD 配准成功后会自动发布 `/initialpose`，AMCL 收到初始位姿后，机器狗会原地旋转 3 圈帮助重定位。

## 适用场景

```text
已有 maps/map.yaml 和 maps/map.pgm
想直接在旧地图中导航
想减少手动 RViz 发布 2D Pose Estimate
```

## 注意

如果没有 PCD 点云图，脚本仍会启动 AMCL，但可能需要在 RViz 手动发布初始位姿。

需要图形界面，因为脚本会启动 RViz。
