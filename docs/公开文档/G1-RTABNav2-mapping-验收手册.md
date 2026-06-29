# G1 RTAB/Nav2 Mapping 验收手册

> 本文件为 OpenSpec change `g1-rtabnav2-adapter` 实机 SSH 验收手册。
> 详细规格见 `openspec/changes/g1-rtabnav2-adapter/specs/g1-rtabnav2-adapter/spec.md`；设计见 `design.md`；RALPLAN 共识原文见 `docs/开发文档/Ralplan/g1-rtabnav2-适配/方案.md`（v4 + v5 修订段）。
>
> **实机 IP**：`192.168.123.164`（2026-06-29 用户提供；旧值 `192.168.31.33` 已废弃）
> **ROS distro**：`foxy`（`/opt/ros/foxy/setup.bash` 实证）
> **G1 头部 lidar**：Livox Mid-360（livox_ros_driver2 命名：`/utlidar/cloud_livox_mid360`）
> **G1 base frame**：`pelvis`（双足 URDF）
> **G1 启动入口**：`./robot_slam.sh --mapping`（替代已删除的 `go2.sh`）

## 0. 前置

- 已完成 OpenSpec change `g1-rtabnav2-adapter` 的本地实施（`scripts/start_gateway.sh` + `.env.G1_example` + `mapping_g1.launch.py` + `robot_slam.sh` + `drivers/g1/defaults.py`）
- 已 push 并在实机 G1 pull
- 已 `colcon build --packages-select go2_slam_nav` 编译成功
- 已 source `.bashrc`（含 ROS foxy + unitree_ros2 等）

## 1. 静态预检（开发端跑）

```bash
cd ~/nuwax_robot_bridge
python3 scripts/check_launch_args.py   # 7 条断言（A1-A7）全过 → 退出 0
openspec validate g1-rtabnav2-adapter --strict   # 无错误
```

## 2. 实机 M0 话题探测

```bash
G1_SSH="sshpass -p '123' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null unitree@192.168.123.164"

# M0: 5 关键话题存在
$G1_SSH "source /opt/ros/foxy/setup.bash && ros2 topic list | grep -E '/utlidar|/dog_odom|/unitree/slam|/slam_info|/planner_map'"
# 期望：/utlidar/cloud_livox_mid360 /utlidar/imu_livox_mid360 /dog_odom
#      /unitree/slam_mapping/odom /unitree/slam_relocation/odom
#      /slam_info /planner_map 等至少 5+ 行

# M0.1: 探测 lidar frame_id
$G1_SSH "source /opt/ros/foxy/setup.bash && ros2 topic info /utlidar/cloud_livox_mid360 --verbose | grep -E 'frame_id|Type'"
# 期望：Type: sensor_msgs/msg/PointCloud2；frame_id ∈ {livox_mid360_link, livox_frame, mid360_link}
# 落 M0.1 真实值到 .env.G1_example 的 NUWAX_GEOMETRY_LIDAR_FRAME（如非 livox_mid360_link）

# M0.2: 探测 lidar 频率
$G1_SSH "source /opt/ros/foxy/setup.bash && ros2 topic hz /utlidar/cloud_livox_mid360"
# 期望：≥ 5Hz（Mid-360 默认 10Hz；livox_ros_driver2 配置可调）

# M0.3: 探测 odom 频率
$G1_SSH "source /opt/ros/foxy/setup.bash && ros2 topic hz /dog_odom /unitree/slam_mapping/odom /unitree/slam_relocation/odom 2>/dev/null || echo 'topic not present'"
# 期望：3 路中至少 1 路 ≥ 5Hz（slamer_server 启后 /unitree/slam_mapping/odom 必有；/dog_odom 与 slamer 相关）
```

## 3. M1 物理：videohub → UDS → ROS topic 三段式

```bash
# videohub 进程在
$G1_SSH "pgrep -a videohub_pc4"
# 期望：PID + 路径命中

# UDS 字节流量 ≥ 15MB/5s
$G1_SSH "timeout 5 socat - UNIX-CONNECT:/tmp/nuwax_video.sock 2>/dev/null | wc -c"
# 期望：≥ 15000000（约 3 MB/s 下限 × 5s）

# ROS topic 镜像
$G1_SSH "source /opt/ros/foxy/setup.bash && ros2 topic hz /utlidar/cloud_livox_mid360"
# 期望：≥ 5Hz
```

## 4. M2 mapping 模式

```bash
# 准备：cp .env.G1_example .env（实机一次性操作；只读模板覆盖到 .env）
# 或：直接 source .env.G1_example（不复制 .env，但 ROS env 在脚本结束后失效）
# 推荐：cp .env.G1_example .env，然后 .env 已生效

$G1_SSH "cd ~/nuwax_robot_bridge/services/geometry/ros2_ws/src/GO2_RTABNAV2 && ./robot_slam.sh --mapping"
# 期望：rtabmap_progress_monitor 报首帧 /map ≤ 30s；/cloud_obstacles 非空
# 失败排查：见 §6
```

## 5. M3 localization 模式

```bash
$G1_SSH "cd ~/nuwax_robot_bridge/services/geometry/ros2_ws/src/GO2_RTABNAV2 && ./robot_slam.sh --localization ~/maps/g1_home.db"
# 期望：rtabmap 报 proximity_detection_id > 0（G1 用 /unitree/slam_relocation/odom 走 ProximityGlobalScanMap）
# 注意：本期 robot_slam.sh 对 G1 localization 模式会直接报错退出（FOllow-up F1 范围）
```

## 6. 故障排查 7 条

| 现象 | 可能根因 | 检查命令 |
|------|------|------|
| M0 探测失败：5 关键话题缺 | G1 slamer_server 未启 | `ssh ... "pgrep -a slamer_server"` |
| M0.1 frame 不匹配 | livox_mid360_link 与 launch 注入的 lidar_frame 不一致 | `ros2 topic info /utlidar/cloud_livox_mid360 --verbose` 拿真实 frame_id；写回 .env.G1_example |
| M0.2 频率 < 5Hz | UDS 带宽不足 | `Rtabmap/DetectionRate=1.0` 降采样；带宽下限 ≥ 3 MB/s |
| RTAB-Map 报 `Did not receive data` | QoS 不匹配（livox 默认 BEST_EFFORT vs RTAB 默认 RELIABLE） | launch 设 `qos:=2`（已含） |
| /unitree/slam_mapping/odom 不存在 | slamer_server 未启 | `pgrep -a slamer_server` |
| /dog_odom 频率 0 | G1 站立无运动 | 正常；本期 mapping 模式走 /unitree/slam_mapping/odom 兜底 |
| pelvis frame 找不到 | 静态 TF 未发布 | `ros2 run tf2_tools view_frames` 看 base→pelvis 是否在 |

## 7. GO2 实机回归（M4，本期范围不在 G1 端但必跑）

```bash
# GO2 实机（原 IP，本文档不记录具体 IP）跑 mapping + localization
GO2_SSH="sshpass -p '...' ssh unitree@<GO2_IP>"
$GO2_SSH "cd ~/GO2_RTABNAV2 && ./robot_slam.sh --mapping"
$GO2_SSH "cd ~/GO2_RTABNAV2 && ./robot_slam.sh --localization ~/maps/go2_home.db"
# 期望：与改动前 mapping_go2.launch.py 直启行为完全等价
# 注意：GO2 端不应消费 .env.G1_example（NUWAX_ACTIVE_ROBOT=go2）
```

## 8. 退出 / 清理

```bash
# G1 端停止 mapping
$G1_SSH "pkill -f 'ros2 launch go2_slam_nav mapping_g1'"
# 或：Ctrl+C 当前 SSH session
```

## 9. 不在范围（follow-up）

- F1 Nav2 闭环（`navigate_to_pose` Action）
- F2 m-explore-ros2 frontier 探索
- F3 头部多相机 3D 投影
- F9 G1 Livox Mid-360 运动畸变 deskewing 优化
- F4 / F5 / F6 / F7：平台通用化拆硬编码（详见 RALPLAN v4）