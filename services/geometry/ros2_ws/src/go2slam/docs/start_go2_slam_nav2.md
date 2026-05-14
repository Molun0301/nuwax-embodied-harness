# start_go2_slam_nav2.sh

启动 Go2 的 SLAM + Nav2，用于边建 2D 地图边导航。SLAM 后端为 slam_toolbox。

## 用法

只启动 SLAM/Nav2：

```bash
./start_go2_slam_nav2.sh
```

同时启动点云建图：

```bash
./start_go2_slam_nav2.sh --mapping
```

无图形界面或不需要 RViz：

```bash
./start_go2_slam_nav2.sh --no-rviz
```

查看帮助：

```bash
./start_go2_slam_nav2.sh --help
```

## 会启动什么

```text
Go2 driver
robot_localization
点云转 LaserScan
slam_toolbox
Nav2
RViz
```

加 `--no-rviz` 时不会启动 RViz，也不会要求 `DISPLAY` 或 `WAYLAND_DISPLAY`，脚本会保持前台运行直到按 `Ctrl-C` 或底层 launch 退出。

加 `--mapping` 时，还会启动：

```text
go2_mapping_only/mapping_only.launch.py
```

用于生成 `/go2_built_map` 点云地图，之后可用 `save_pointcloud.sh` 保存。

> frontier 探索由平台侧 `services/navigation/PlatformExplorationRuntime` + `Nav2ActionNavigationProvider` 负责（基于 occupancy 计算 frontier 并通过 Nav2 action 派发），go2slam 不再内置 explore_lite/m-explore-ros2。

## 适用场景

```text
第一次建图
边走边生成 2D map
需要同时保存点云图时加 --mapping
```

## 注意

默认需要图形界面，因为脚本会启动 RViz。远程终端或无显示环境使用 `--no-rviz`。

如果只是用已有地图导航，不要用这个脚本，使用：

```bash
./start_go2_nav2_localization.sh
```
