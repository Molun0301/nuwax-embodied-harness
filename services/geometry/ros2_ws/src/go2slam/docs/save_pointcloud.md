# save_pointcloud.sh

保存当前点云建图结果为 PCD 文件。

## 用法

```bash
./save_pointcloud.sh
```

## 保存位置

脚本会等待一条 `/go2_built_map` 点云消息，然后保存到：

```text
output/go2_built_map_<timestamp>.pcd
```

## 运行前提

需要先启动点云建图：

```bash
./start_go2_slam_nav2.sh --mapping
```

然后另开一个终端运行：

```bash
./save_pointcloud.sh
```

## 适用场景

```text
保存 PCD 点云图
后续给 start_go2_nav2_localization.sh 做自动初始定位
```

## 注意

脚本只保存收到的第一条 `/go2_built_map` 消息。

如果一直卡在等待，说明 `/go2_built_map` 没有发布。可以检查：

```bash
ros2 topic echo /go2_built_map --once
```
