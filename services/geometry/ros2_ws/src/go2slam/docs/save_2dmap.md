# save_2dmap.sh

保存当前 SLAM 生成的 2D 栅格地图。

## 用法

```bash
./save_2dmap.sh
```

## 保存位置

脚本会保存：

```text
maps/map.yaml
maps/map.pgm
```

## 运行前提

需要先启动 SLAM，并且系统里已经有 `/map`：

```bash
./start_go2_slam_nav2.sh
```

然后另开一个终端运行：

```bash
./save_2dmap.sh
```

## 适用场景

```text
建完 2D 地图后保存
后续给 start_go2_nav2_localization.sh 使用
```

## 注意

如果 `/map` 没有发布，保存会失败。可以先检查：

```bash
ros2 topic echo /map --once
```
