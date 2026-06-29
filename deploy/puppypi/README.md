# PuppyPi 方案 A 部署说明

## 架构

```text
女娲平台
   ↓
NuwaxClaw / MQTT
   ↓
nuwax_robot_bridge 网关（宿主机，无 unitree_sdk2py）
   ↓
nuwax_bridge_py + nuwax_puppypi_adapter（puppypi_ros2 容器）
   ↓
Nav2 / puppy_control
   ↓
幻尔机器狗
```

## 一键部署

```bash
cd ~/nuwax-robot-bridge
bash scripts/deploy_puppypi.sh
```

## 启动

```bash
bash scripts/start_puppypi_stack.sh map:=map_01
```

## 手动速度测试（容器内）

```bash
docker exec -it puppypi_ros2 bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  ros2 topic pub /puppy_control/velocity puppy_control_msgs/msg/Velocity "{x: 5.0, y: 0.0, yaw_rate: 0.0}" --once
'
```

## 平台动作 JSON 示例

遥控前进（经 action_bridge）：

```json
{"action": "move", "linear_x": 0.05, "yaw_rate": 0.0}
```

停止：

```json
{"action": "stop"}
```

导航目标：

```json
{"action": "navigate", "x": 1.0, "y": 0.5, "yaw": 0.0}
```

## 注意

- 不安装 `unitree_sdk2py`、不启用 RTAB-Map。
- 默认不启动 `cmd_vel` 适配器，避免与官方 `navigation.launch` 重复控制。
- 若 Nav2 未直接驱动 `puppy_control`，设置 `ENABLE_CMD_VEL_ADAPTER=true` 后重启栈。
