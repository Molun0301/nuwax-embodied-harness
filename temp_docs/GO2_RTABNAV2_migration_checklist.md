# GO2_RTABNAV2 全量迁移实施清单

- **版本**：v2.8
- **日期**：2026-06-26
- **入口约定**：唯一正式 `go2.sh` = `services/geometry/ros2_ws/src/GO2_RTABNAV2/go2.sh`
- **用法**：完成一项将 `[ ]` 改为 `[x]`；§8 由实机测试打勾
- **同步规则**：每次代码改动后更新本文件 + `GO2_RTABNAV2_实机测试清单.md`

---

## 0. 审批项（实施前确认）

- [ ] **A** — `NUWAX_GEOMETRY_*` Nav2 参数物化到 launch yaml：`是` / `否`（当前：**仅 .env 注入**，yaml 仍用包内默认）
- [x] **B** — Rust bridge `env.sh` 回退：**`GO2_RTABNAV2/scripts/env.sh` 已新增**
- [x] **C** — `source_go2.bash` 默认工作区：**仓库相对 `GO2_RTABNAV2/ws`**
- [x] **D** — `GO2_NAV_*` 环境变量：**废弃**（GO2_RTABNAV2 用 `localize_only`/`restart_map`/`slam_mode`）
- [ ] **E** — `GO2_RTABNAV2/ws/install`：`仅 bootstrap 构建` / `入库`（当前：**不入库**，install 已存在）
- [ ] **F** — 旧目录 `GO2_RTABMAP`：`验收后删除` / `归档保留`（**待 §8 验收后**）
- [x] **G** — bootstrap CLI：**`--build-go2-rtabnav2` 新名 + 保留 `--build-go2slam` 别名**
- [x] **H** — 探索元数据命名：**`explore_lite` / `go2_rtabnav2_script`**
- [x] **I** — Debug 2D 主图：**仅 `/map_obstacles`**（不做双图层）
- [x] **J** — Rust bridge `NUWAX_MAP_TOPIC`：**随 occupancy 改 `/map_obstacles`**

---

## Phase 0 — GO2_RTABNAV2 工作区自洽

- [x] `GO2_RTABNAV2/ws/source_go2.bash` 默认 `GO2_WS` 改为 `${SCRIPT_DIR}`（仓库内 `ws/`）
- [x] `GO2_RTABNAV2/ws/rebuild_after_rename.sh` 路径改为仓库相对
- [x] `ws/src` 符号链接指向 `unitree-go2-slam-nav2`、`m-explore-ros2` 等依赖正确
- [x] `colcon build --symlink-install`（`GO2_RTABNAV2/ws`）成功（install 已存在）
- [x] `ros2 pkg prefix go2_slam_nav` 解析到 `GO2_RTABNAV2/ws/install/go2_slam_nav`
- [x] 手工 `./go2.sh --mapping` 能拉起 SLAM + Nav2（**实机** · 2026-06-26 `#13a`）
- [ ] 手工 `./go2.sh --localization <db>` 能定位导航（**实机**）
- [x] 手工 `./go2.sh --explore <db>` 能自主探索（**实机** · v2.5 · **#8 CLI 通过**：预滚动 + 连续追前沿，**无** `nav_loop_guard`）

---

## Phase 1 — 配置与部署入口（P0）

- [x] `.env`：`NUWAX_GEOMETRY_EXTERNAL_SCRIPT` → `.../GO2_RTABNAV2/go2.sh`（顶层）
- [x] `.env`：`NUWAX_GEOMETRY_OCCUPANCY_TOPIC` → `/map_obstacles`
- [x] `.env.example`：同步上述路径与 topic 注释
- [x] `scripts/bootstrap.py`：`GO2SLAM_WS_DIR` → `GO2_RTABNAV2/ws`
- [x] `scripts/bootstrap.py`：运行时包检查改为 `go2_slam_nav`、`explore_lite` 等
- [x] `scripts/bootstrap.py`：默认生成 `NUWAX_GEOMETRY_EXTERNAL_SCRIPT` 指向顶层 `go2.sh`
- [x] `scripts/bootstrap.py`：默认 `NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map_obstacles`
- [x] `scripts/start_gateway.sh`：`geometry_root` → `GO2_RTABNAV2`
- [x] `scripts/start_gateway.sh`：进程清理模式对齐 `go2_slam_nav` / `explore_node`
- [x] `python3 scripts/bootstrap.py check --robot go2 --geometry` 通过（Go2SLAM 项全 OK）
- [ ] `python3 scripts/bootstrap.py apply` 后配置一致（**可选复验**）

---

## Phase 2 — 平台运行时适配（P1）

### 2.1 go2.sh 日志

- [x] `external_slam_nav2` 托管进程 stdout 同步写入 `nuwax_robot_bridge.log`
- [x] `_append_main_log_line` + `_publish_main_log_stream` → StreamBus `logs.entry`（避免 file handler 重复写盘）
- [x] `GET /api/debug/logs/tail` 增量读主日志（含 go2.sh 建图/导航/探索行）
- [x] `debug_panel.html`：`pollLogFileTail()` 轮询合并主日志；`source+时间+message` 去重
- [x] 侧车日志仍保留 `runtime_data/geometry/logs/external_slam_nav2.log`
- [ ] 实机验证：`grep 'geometry.go2.sh' nuwax_robot_bridge.log` 有 go2.sh 行
- [ ] 实机验证：Debug 日志区搜「建图」「导航」「探索」有 `geometry.go2.sh` / `rtabmap_progress_monitor` 匹配

### 2.2 探索终态与文案

- [x] `runtime.py`：`GO2_RTABMAP` 文案 → `GO2_RTABNAV2`
- [x] `runtime.py`：探索 PID/日志检测适配 `explore_node` / `explore_lite`
- [x] `runtime.py`：`exploration_mode` / `exploration_runtime` 元数据更新
- [x] `runtime.py`：就绪提示 `/map_obstacles`
- [x] `external_debug_observer.py`：命名去 `rtabmap`

### 2.3 Debug 地图与前沿

- [x] `external_debug_observer.py`：订阅 `/map_obstacles`；前沿 `explore/frontiers`；`explore/status`
- [x] `debug_panel.py`：诊断文案动态 topic
- [x] `debug_panel.html`：`#mapMeta` 来源话题 + 图层构成；explore_lite 前沿文案
- [x] 不实现 Raw `/map` 双图层 UI

### 2.4 配置契约与 Rust bridge

- [x] `core/settings.py`：occupancy 默认 `/map_obstacles`
- [x] `contracts/geometry_runtime.py`：`frontier_topic`
- [x] `GO2_RTABNAV2/scripts/env.sh` + `load_platform_geometry_env.sh`
- [x] `go2.sh`：定位直读传入 `.db`；DB 根目录对齐 `NUWAX_RTABMAP_DATABASE_ROOT`
- [x] `ros_bridge_client.py`：`env.sh` 回退路径可用
- [x] `nuwax_ros_bridge` + `start_gateway.sh`：默认 `/map_obstacles`

### 2.5 其他 P1/P2

- [x] `services/telemetry/collector.py` + `.env`：关键字增加 `go2_slam_nav` / `explore_lite` 等
- [x] `drivers/go2/data_plane.env.example`：注释路径更新
- [x] `gateways/runtime.py`：探索能力文案改为 explore_lite

### 2.7 MCP 地图版本响应精简（Agent 遥控建图结束）

- [x] `gateways/runtime.py`：`save_active_map_version` / `list_map_versions` MCP 响应裁剪 `occupancy_grid.data`
- [x] `gateways/runtime.py`：返回 `data_cell_count` + `data_omitted` + `geometry_asset` 路径摘要 + `next_step_hint`
- [x] `gateways/runtime.py`：异步任务结果（`load_map_version` / 探索保存）同步裁剪 `map_version`
- [ ] 实机验证：遥控建图 `save_active_map_version` 响应 < 10KB，Agent 无需解析落盘大 JSON

### 2.6 `/map_obstacles` 图层优化（不含 live 扫描烙印）

- [x] `obstacle_grid_go2.yaml`：`live_cloud_enabled: false`；`publish_on_cloud: false`
- [x] `rtab_lidar_go2.launch.py`：支持 `GO2_OBSTACLE_GRID_*` 环境变量覆盖
- [x] `colcon build go2_slam_nav`：install 内 launch/yaml 与源码同步（旧 install 曾仍 `live_cloud=true`）
- [x] Debug 定位：`external_debug_observer` 订阅 `/go2/pose_map`；`NUWAX_GEOMETRY_BASE_FRAME=go2_base_link`
- [x] Nav2 `force` 模式读 `get_external_live_pose`（修复 2D Goal「无实时位姿」）
- [x] go2.sh 侧车日志走 Python logging → Debug `logs.entry` + 文件 tail 双通道可搜「建图」
- [ ] 实机验证：Debug 地图上有机器狗三角位姿；2D Goal 驱动 Nav2；日志搜「建图」有匹配
- [x] `load_platform_geometry_env.sh` + `.env`：默认关闭 live cloud 烙印
- [x] Debug 元数据：`map_composition` / `map_excludes_live_scan`（observer + debug_panel）
- [x] 确认：Debug 前端不叠加点云层，异常来自 projector 融合 live `/go2/cloud`
- [ ] 实机验证：`obstacle_grid_projector` 日志「实时 N 点」为 0；地图无扇形扫描刷新

### 2.8 Debug 2D Goal 先发原点再纠正（严重）

**现象**：下发 2D Goal 后机器狗先朝 map 原点 `(0,0)` 走约 3s，`/plan` 先指向原点，再切到用户目标。

**根因（代码审计）**：
- 日志证明 API 下发的 `target=(x,y)` 正确，非前端坐标错误
- 新 goal 发出前 Nav2 在途目标取消不完整（只 `cancel_goal_async` 不等待）
- `force_nav2_goal` 额外发布 `/goal_pose`，与 action 双通道干扰 Foxy 抢占
- goal `header.stamp=now()` 在 TF 未稳时可能错误变换
- Debug 未清空旧 `navigationPath`，绿线误导
- 探索循环可能与 Debug 目标争用同一 Nav2 backend

- [x] `nav2_action_provider.py`：`_cancel_active_nav2_goals_before_send()` 等待 cancel 完成
- [x] `nav2_action_provider.py`：移除 Debug 路径 `/goal_pose` 同步发布（仅 action）
- [x] `nav2_action_provider.py`：`_wait_action_dispatch()` 等新 goal 真正发出
- [x] `nav2_action_provider.py`：NavigateToPose goal `header.stamp=0`（Nav2 用最新 TF）
- [x] `gateways/runtime.py`：`dispatch_debug_nav2_goal` 前取消平台探索在途导航
- [x] `debug_panel.html`：下发时清空 `navigationPath`、立即更新 `goalPose`
- [ ] 实机验证：2D Goal 后**不**先朝 map 原点走；`/plan` 直指用户点击点；约 3s 内无「折返纠正」

### 2.9 Debug 2D Pose 无反应（严重）

**现象**：Debug「2D Pose」拖拽后无重定位效果；三角位姿不跳转；日志可能显示已发布但 RTAB-Map 未收到。

**根因（代码审计）**：
- `publish_initial_pose` 强依赖 Nav2 action provider `_ensure_started()`（Nav2 lifecycle 未就绪则 503）
- 单次 publish + 默认 QoS depth=10，无等待订阅者与重发
- 仅发 `/initialpose`，未发 RTAB-Map 同时订阅的 `/global_pose`
- Debug 前端只画 `initialPoses` 标记，不乐观更新 `state.pose` 三角
- `get_external_live_pose` 缓存未在 initialpose 后失效，位姿流滞后

- [x] `runtime.py`：`publish_debug_initial_pose()` 独立 ROS 节点，不依赖 Nav2 provider
- [x] `runtime.py`：RELIABLE QoS + 等待订阅者 + 连发 2 次；仅发 `/initialpose`（不发 `/global_pose`）
- [x] `runtime.py`：`invalidate_external_live_pose_cache()` 发布后清 `/go2/pose_map` 缓存
- [x] `gateways/runtime.py`：`dispatch_debug_initial_pose` 改走 geometry runtime
- [x] `nav2_action_provider.py`：`publish_initial_pose` 委托 geometry runtime
- [x] `debug_panel.html`：下发时乐观更新 `state.pose`；无订阅者时 UI 警告
- [ ] 实机验证：定位模式下 2D Pose 后三角位姿跳转；`ros2 topic info /initialpose` 有 rtabmap 订阅者
- [ ] 实机验证：建图模式下 UI 有 hint；定位模式重定位后 Nav2 2D Goal 可用

### 2.10 load_map_version 后 Debug 不显示地图（localization）

**现象**：`load_map_version(h1)` 成功后 Debug 空白；`active_workspace.map_loaded=false`；日志显示 go2.sh 已 `--localization` 但等待 `/map_obstacles` 首帧。

**根因**：
- `_load_map_version_into_runtime` 在 `rtabmap_localization` + 非 static nav2 yaml 时调用 `clear_latest_snapshot()` 并 return
- `clear_latest_snapshot` 向 stream 发 `map.frame status=cleared`，前端立即隐藏画布
- 定位模式 `/map_obstacles` 首帧可能延迟数十秒，期间无任何 checkpoint 可显示
- `map_loaded` 仅认 live `map_ready`，不认已 ingest 的 checkpoint

- [x] `map_workspace.py`：定位加载改 `_ingest_localization_checkpoint_snapshot()`，不再 clear
- [x] `map_workspace.py`：`map_loaded` / workspace 聚合认 `offline_checkpoint` / `platform_map_version_id`
- [ ] 实机验证：`load_map_version` 后 Debug **立即**显示 h1 地图；`map_loaded=true`；`/map_obstacles` 首帧后平滑切 live（可选）

### 2.11 Agent 异步任务高频刷 get_task_status（MCP 体验）

**现象**：大模型 `load_map_version` 后立即连续秒级调用 `get_task_status` 十余次，刷出大量 `platform` 行；任务实际需 45–90s，只需等待后少量轮询。

**根因**：
- 能力描述仅写「用 get_task_status 轮询」，未给初始等待与间隔
- 异步接受响应无 `polling_guidance` / `next_step_hint`
- `get_task_status` running 态无 `retry_after_sec`，Agent 无法判断何时再 poll

- [x] `gateways/runtime.py`：`_ASYNC_GEOMETRY_TASK_POLLING_GUIDANCE`（load 45s/12s；activate 15s/8s；relocalize 20s/10s；**explore 30s/15s** · v2.4）
- [x] `gateways/runtime.py`：异步接受响应 + task metadata 含 `polling_guidance` 与 `next_step_hint`
- [x] `gateways/runtime.py`：`get_task_snapshot` → `polling_advisory`（`retry_after_sec` / `polled_too_soon` / events 截断）
- [x] `core/task_manager.py`：`get_task_spec()` 供轮询指引读取 capability
- [x] `localization_skills.py`：`handle_load_map_version` 完成态 `next_step_hint`
- [x] 能力/Skill 描述：`load_map_version` / `get_task_status` 等写明「先等再 poll，勿高频连刷」
- [ ] 实机验证：Agent `load_map_version` 后**不**秒级连刷 `get_task_status`；按 `retry_after_sec` 等待；总 poll 次数 ≤ 8

### 2.12 2D Pose 跳回 + 地图/位姿冻结（严重 · **实机未通过**）

**状态**：🔴 **阻塞** — 2026-06-26 用户复测 v1.8 补丁后**仍未解决**

**现象**：`load_map_version` 后用 Debug 2D Pose 设初始点，三角瞬间跳到目标又拉回旧位；地图界面像死掉；遥控机器狗时地图上定位不更新。

**根因（已确认 + 待查）**：
- 前端乐观位姿被旧 `/go2/pose_map` 流覆盖；单次接近目标即清除 pending，随后 stale 位姿跳回
- `map.heartbeat` / `clearMapDisplay` 在定位任务结束后误清 checkpoint 地图，连带隐藏位姿
- `publish_debug_initial_pose` 仅发 `/initialpose`；RTAB-Map 可能短暂接受后 TF 再次冻结（ROS 侧待查）
- `external_debug_observer`：`/go2/pose_map` 10Hz 刷新 `_last_pose_map_at`，旧位姿阻塞 TF fallback

**v1.8 已合入（未验收）**：
- [x] `runtime.py`：hint 字段 + 3 帧稳定收敛
- [x] `runtime.py`：`publish_debug_initial_pose` 仅 `/initialpose` + optimistic 流
- [x] `external_debug_observer.py`：pose_map 冻结时 TF fallback
- [x] `debug_panel.html`：pending/checkpoint 时保留三角

**v1.9 补丁（待复测）**：
- [x] `runtime.py` / `debug_panel.html`：收敛判定须**远离 anchor**（防「闪对后跳回原点」）
- [x] `debug_panel.html`：`map.heartbeat` / `clearMapDisplay` 不误清 checkpoint 地图
- [ ] ROS 侧：2D Pose 后 `tf2_echo map go2_base_link` 是否冻结；`rtabmap` 是否丢定位

**实机验收（未通过）**：
- [ ] ❌ 2D Pose 后三角**保持**在目标附近 ≥10s，不跳回旧位
- [ ] ❌ 遥控后地图三角持续跟随（或 TF 收敛后跟随）
- [ ] `ros2 topic info /initialpose` 有 rtabmap 订阅者；`external_slam_nav2.log` 无定位崩溃

### 2.13 原生建图 2D Goal 无反应 + bt_navigator SIGSEGV（`go2.sh --mapping`）

**状态**：🟡 **v2.8 恢复 `rviz_goal_bridge` + Nav2 双检门控** · 🟡 **#13a/#13e 待复测** · **v2.5 已移除 `nav_loop_guard`**

**现象（v2.0 回归）**：`./go2.sh --mapping --fresh`；下发 2D Goal **完全无反应**或首目标即 `bt_navigator` exit -11。

**根因（v2.1 确认）**：
- RViz 改 `GoalTool` 后未稳定接 action；`SetGoal` 发 `/goal_pose` 需桥接
- Nav2 子节点 lifecycle 未 active 时发 Goal → `send_goal failed` + SIGSEGV
- 回环时 `map→odom` 跳变 + `FollowPath` 执行中 → bt_navigator 崩溃

**v2.5 结论（实机 2026-06-26）**：
- **`nav_loop_guard` 已从 `nav_go2.launch.py` 全模式移除**（mapping / explore / localization **均不启动**）
- Foxy 上其 lifecycle SHUTDOWN/STARTUP **反而搞崩** `lifecycle_manager`（SIGSEGV），探索/建图均有害
- 恢复策略改为：Nav2 节点 `respawn=True` + explore 侧「Nav2 未就绪不拉黑、暂停追击」
- `nav_loop_guard.py` / `setup.py` entry 保留源码，**launch 永不拉起**

**v2.1 补丁（部分仍有效）**：
- [x] `nav2_params_go2_foxy.yaml`：`recoveries_server` 去掉 `Spin`（GO2 SIGSEGV）
- [x] `navigate_to_pose_go2_foxy.xml`：扁平 Recovery（Wait + clear costmap）
- [x] `nav_go2.launch.py`：**无** `nav_loop_guard` / `go2_stack_health`；**v2.8 恢复** `rviz_goal_bridge`（Foxy SetGoal 必需）
- [x] ~~`nav_loop_guard` lifecycle 恢复~~ · ~~`rviz_goal_bridge`~~ · ~~`go2_stack_health`~~（v2.5 均不启）

**实机验收**：
- [x] ✅ `./go2.sh --mapping --fresh`；RViz 2D Goal → 机器狗移动（**#13a**）
- [ ] 日志**无** `[nav_loop_guard]`（mapping / explore 均适用）
- [ ] 回环后第二个 2D Goal 可正常导航（**#13e**）；**无** bt_navigator / lifecycle_manager exit -11

### 2.14 原生 go2.sh 启动即 `Killed`（OOM / 双栈）

**现象**：`./go2.sh --mapping --fresh` 刚拉起 rtabmap/rviz 即终端输出 `Killed`，无 bt_navigator 日志。

**根因**：Jetson 内存不足（MemAvailable < 600MB 或 rviz2 峰值触发 OOM）；nuwax gateway 定位栈与原生 go2.sh **双栈**并存。

- [x] `go2.sh`：`preflight_memory` + `preflight_conflicting_stack`；`GO2_USE_RVIZ=0`；`kill_other_go2_shells`（安全版 pkill go2.sh，排除 $$）
- [ ] 实机验证：停 gateway 后 `GO2_USE_RVIZ=0 ./go2.sh --mapping --fresh` 可持续运行 ≥60s

### 2.15 原生探索不追前沿（`go2.sh --explore` · 对齐干净包）

**状态**：🟢 **v2.5 实机通过（#8 CLI）**

**现象（v2.2–v2.4 回归）**：预滚动正常但 Nav2 未就绪即发前沿 → `send_goal failed` / bt_navigator SIGSEGV；`nav_loop_guard` lifecycle 恢复搞崩全栈。

**根因**：
- `ros2 lifecycle get` 子进程在 Jetson 负载下误超时，**不能**用于 preroll 门控
- `navigate_to_pose` action 就绪 ≠ `follow_path`（controller）可接受子目标
- explore 过快换前沿（`min_goal_interval` 过短）在 Nav2 recovery 期间抢占 → SIGSEGV
- **`nav_loop_guard` SHUTDOWN/STARTUP 在 Foxy 混合 lifecycle 状态下导致 `lifecycle_manager` SIGSEGV**

**v2.5 补丁（已合入 + 实机通过）**：
- [x] `explore_preroll.py`：等 `navigate_to_pose` + **`follow_path`** action server；稳定 **8s**；**不用** lifecycle CLI
- [x] `explore_go2.launch.py`：Nav2 **t=20s**；preroll **t=30s**；预滚动 **0.30 m/s + 360°**（对齐干净包）
- [x] `explore_go2.yaml`：`min_goal_interval` **10s**；`cmd_vel_stale_timeout` **12s**
- [x] `explore.cpp`：Nav2 未就绪时**暂停追击、不拉黑**；等 respawn 恢复
- [x] `nav_go2.launch.py`：**全模式不启** `nav_loop_guard` / `go2_stack_health` / `rviz_goal_bridge`
- [x] `sport_ctrl.py`：探索预滚动仍对齐干净包 **0.30 m/s + 360°**（v2.5）；**导航**最小速度 v2.6 另调（见 §2.17）

**实机验收**：
- [x] ✅ 预滚动 → `Nav2 action 就绪且已稳定，启动 explore_lite`
- [x] ✅ `explore_node` 首发前沿后机器狗移动；可连续追多个前沿（**#8**）
- [x] ✅ 日志**无** `[nav_loop_guard]`
- [ ] 直线轨迹无明显 S 形（**#13f** · v2.7 再降速后复测）

### 2.16 GO2 导航/预滚动速度调参

**状态**：探索预滚动 **v2.5 保持干净包**；**导航 DWB / sport_ctrl v2.7 再降速**（见 §2.17）

| 参数 | 文件 | v2.6 | v2.7 |
|------|------|------|------|
| 探索预滚动前进 | `explore_go2.launch.py` | 0.30 m/s | **不变** |
| 探索预滚动旋转 | 同上 | 360° | **不变** |
| Nav2 线速度上限 | `nav2_params_go2*.yaml` | 0.22 | **0.15** |
| Nav2 角速度上限 | 同上 | 0.28 | **0.18** |
| Nav2 线/角加速度 | 同上 | 1.0 / 1.0 | **0.6 / 0.5** |
| velocity_smoother 上限 | `nav2_params_go2.yaml` | [0.22,0,0.28] | **[0.15,0,0.18]** |

- [x] v2.7 导航 yaml / `sport_ctrl` / `nav_go2.launch.py` 已修改
- [x] `colcon build --packages-select go2_slam_nav go2_cmd_processor` 成功（v2.7）
- [ ] 实机体感：直线 2D Goal 更贴路径、少 S 形（**#13f**）

### 2.17 导航直线却走 S 形 · 最小速度底（v2.7）

**状态**：🟡 **v2.7 已合入** · 待 **#13f** 实机复测

**根因**：`sport_ctrl` 将低于下限的 cmd_vel **抬到最小值**；DWB 小幅路径修正被放大 → 过冲纠偏 → S 形。

| 参数 | 文件 | v2.6 | v2.7 |
|------|------|------|------|
| `min_linear_speed` | `sport_ctrl.py` / `nav_go2.launch.py` | 0.08 | **0.05** |
| `min_angular_speed` | 同上 | 0.08 | **0.05** |
| `min_speed_xy` / `min_speed_theta` | `nav2_params_go2*.yaml` DWB | 0.08 | **0.05** |
| `trans_stopped_velocity` | 同上 | 0.06 | **0.04** |
| recovery `min/max_rotational_vel` | `nav2_params_go2_foxy.yaml` | 0.10 / 0.28 | **0.06 / 0.18** |

- [x] v2.7：`sport_ctrl` + DWB `min_*` 对齐 **0.05**
- [ ] 实机体感：直线轨迹是否可接受（**#13f**）；过慢则 `max_vel_x` 可回调 **0.18**

### 2.20 `nav_loop_guard` 移除 + Nav2 探索就绪门控（v2.5 · ROS 包）

**状态**：🟢 **已合入 + 探索实机验证**

| 项 | 决策 |
|----|------|
| `nav_loop_guard` launch | **全模式移除**（mapping / explore / localization） |
| 原因 | Foxy `cancel_all_goals_async` 不存在；lifecycle SHUTDOWN 搞崩 `lifecycle_manager` |
| 替代 | Nav2 `respawn=True`；`explore_preroll` action 双检 + 8s 稳定；explore 未就绪不拉黑 |
| 源码 | `nav_loop_guard.py` 保留未删；`setup.py` entry 保留；**永不 launch** |

- [x] `nav_go2.launch.py` / `explore_go2.launch.py`：无 `nav_loop_guard` TimerAction
- [x] `go2.sh` kill 列表仍含 `nav_loop_guard`（清理残留进程）
- [x] 探索 CLI 连续追前沿成功（用户确认 2026-06-26）
- [ ] mapping CLI：日志无 `[nav_loop_guard]`；RViz 2D Goal 正常（**#6/#13a 复验**）

### 2.18 MCP `explore_area` 误报 `external_tf`（平台侧 · v2.4）

**状态**：🟡 **v2.4 已合入** · 待 **#10** 复测（gateway 重启后）

**现象（t1 · 2026-06-26）**：
- `explore_area(map_name=t1)` 已提交；狗已动；`frontier_count=32`；`/map_obstacles` streaming；`/go2/pose_map` 有位姿
- RTAB-Map 日志：`map→odom TF 已由 RTAB-Map 维护`
- 平台/MCP 仍报 `waiting_for: external_tf`、`localization_ready=false`；`get_task_status` progress≈0.25
- Agent 反复查 TF、触发 `relocalize_active_map`（探索中不应做）

**根因（代码审计）**：
- `_build_tf_chain_status` 要求 Rust `GetStatus` 的 `pose_ready` + `map_to_base_ready`，但 gRPC `GetStatus` **不返回**这些字段 → 永远 `external_tf`
- `_get_external_script_exploration_state`：无 `status_text` 时回落 diagnostics，忽略 `frontier_count`
- `_augment_exploration_state_with_mapping_diagnostics`：`explorer_actually_running` 仅认 `status_text`
- `_advance_runtime_state_from_inputs`：探索状态检查在 `localization_ready` 门控**之后**

**v2.4 补丁（仅平台 Python，未改 ROS 包）**：
- [x] `runtime.py`：`_resolve_external_tf_localization_status` 合并 `/go2/pose_map` + Rust `GetFrameTree` 链
- [x] `runtime.py`：探索运行中 `frontier_count` / `plan_fresh` 优先，清除 `external_tf` headline
- [x] `runtime.py`：运行时状态「探索中」优先于 `localization_ready` 门控
- [x] `gateways/runtime.py`：`explore_area` `polling_guidance`（30s 初等 / 15s 间隔）
- [x] `gateways/runtime.py`：`_estimate_exploration_progress` 有 `frontier_count>0` 时不因 `external_tf` 卡 0.25
- [x] `gateways/runtime.py` / `runtime.py`：`agent_guidance` 注明勿 `relocalize`、看 `frontier_count`

**实机验收**：
- [ ] 重启 gateway 后 `explore_area`：`get_task_status` / `get_active_map` **不报** `external_tf`（`frontier_count>0`）
- [ ] `waiting_for=null`；`message` 含「正在探索：N 个前沿点」
- [ ] Agent **不**因 `localization_ready=false` 触发 `relocalize_active_map`
- [ ] 任务终态 `task.succeeded`（非 300s 超时；探索完成或达 `max_duration_sec`）

### 2.19 Debug 建图/探索前沿分流（平台展示层 · v2.4）

**现象**：遥控建图（`start_mapping`）时 Debug 误显探索前沿；建图地图刷新过频。

**v2.4 补丁（仅平台，未改 ROS 包）**：
- [x] `external_debug_observer.py`：仅 `session_mode=explore` 投递前沿；mapping 地图 1.2s 节流元数据
- [x] `runtime.py`：非 explore 忽略前沿回调；`start_mapping` 切 session 清前沿缓存
- [x] `navigation/service.py`：仅 `geometry_session_mode==explore` 发 `exploration.frontier` 流
- [x] `debug_panel.html`：建图/定位不画前沿；`shouldThrottleMappingMapPaint` 节流

**实机验收**：
- [ ] `start_mapping` 后 Debug **不**显示探索前沿圆点
- [ ] `explore_area` 后 Debug **显示**前沿 + `frontier_count` 与 MCP 一致
- [ ] 遥控建图时地图刷新平稳（无高频闪烁）

---

## Phase 3 — 文档与清理（P2/P4）

- [x] `docs/公开文档/部署文档.md`：`go2slam` → `GO2_RTABNAV2/ws`
- [ ] `README.md`：补充 geometry 工作区与构建命令
- [ ] `2026-05-25_RTAB-MAP+Nav2平台接入评测与交接文档.md`：标注 GO2_RTABMAP deprecated
- [ ] `temp_docs/GO2_RTABMAP_to_GO2_RTABNAV2_migration_audit.md`：与清单状态同步
- [ ] （审批项 F）删除或归档 `GO2_RTABMAP/`
- [x] `unitree-go2-slam-nav2/scripts/go2.sh`：加注释指向顶层正式入口

---

## §8 整体验收（实机测试 — 由你打勾）

### 部署与启动

- [ ] `bash scripts/start_gateway.sh` 启动无报错
- [ ] 网关健康：`curl /api/health` 正常
- [ ] `bootstrap check` 识别 `GO2_RTABNAV2/ws/install/setup.bash`

### 几何栈能力

- [ ] `activate_map` + 定位：`go2.sh --localization <db>`，`/map_obstacles` 有数据
- [ ] `load_map_version`：Debug 立即显示 checkpoint；`map_loaded=true`；Agent 按 `polling_advisory` 轮询（非秒级连刷）
- [ ] `explore_area`：探索任务 `task.succeeded`（非超时卡死）；MCP **不报** `external_tf`（§2.18 · v2.4）
- [ ] `stop_mapping`：几何栈进程全部退出
- [ ] MCP/HTTP `navigate_to_pose`：Nav2 到点

### Debug 面板（D1–D10）

- [ ] **D1** 2D 地图与 Nav2 一致；**无 live 扫描扇形烙印**（仅 RTAB+/cloud_obstacles）
- [ ] **D2** 探索时前沿或 `frontiers_available` 有反馈
- [ ] **D3** 地图摘要 `source_topic=/map_obstacles` + 图层构成说明
- [ ] **D4** 2D Goal 后 `/plan` 有规划线
- [ ] **D5** 未启几何时不显示误导性旧 `/map` 缓存
- [ ] **D6** 地图上有机器狗三角位姿；`位姿定位` 卡片非「缺失」
- [ ] **D7** 2D Goal 后 Nav2 接受（非「无实时位姿」）；机器狗开始移动
- [ ] **D8** 日志区搜「建图」「导航」「探索」有 `geometry.go2.sh` 匹配（含文件 tail 轮询）
- [ ] **D9** 2D Goal **不**先导航到 map 原点再纠正；规划线直指点击目标
- [ ] **D10** 2D Pose 后三角位姿跳转；`ros2 topic info /initialpose` 有 rtabmap 订阅者
- [ ] **D10b** 2D Pose 后**不**瞬间跳回旧位；遥控时三角持续更新（§2.12 · 🔴 未通过）

### 日志

- [ ] `nuwax_robot_bridge.log` 含 `[nuwax_robot_bridge.geometry.go2.sh]` 行
- [ ] `external_slam_nav2.log` 可用于探索终态扫描
- [ ] `external_slam_nav2.log` 中 `/map_obstacles` 发布「实时 0 点」

### 清理

- [ ] 无残留 `go2_nav` 进程误杀/漏杀
- [ ] `git status` 无意外提交 `ws/install`、`log/` 构建产物

---

## 进度统计

| 阶段 | 已完成 | 总数 | 说明 |
|------|--------|------|------|
| §0 审批 | 7 | 10 | A/E/F 待决策或验收后 |
| Phase 0 | 7 | 8 | localization 待测；explore **#8 v2.5 已通过** |
| Phase 1 | 10 | 11 | apply 复验可选 |
| Phase 2 | 81 | 89 | §2.12 阻塞；#8 ✅；#10/#13e/#13f 待测；§2.17 v2.7 导航再降速 |
| Phase 3 | 2 | 6 | README/audit/删旧栈待做 |
| §8 验收 | 0 | 23 | **全部由你测试** |
| **代码侧合计** | **102** | **118** | **§8 另计 23 项** |

---

## 变更记录

| 版本 | 日期 | 摘要 |
|------|------|------|
| v2.8 | 2026-06-26 | §2.13 mapping 2D Goal：恢复 `rviz_goal_bridge`；action 双检+8s 稳定；`nav2_startup_delay` 20s |
| v2.7 | 2026-06-26 | §2.17 导航再降速：min **0.05**；max **0.15/0.18**；加速度 **0.6/0.5** |
| v2.6 | 2026-06-26 | §2.17 导航 S 形修复：sport_ctrl/DWB 最小速度 **0.08**；线/角上限 **0.22/0.28**；colcon 已构建 |
| v2.5 | 2026-06-26 | §2.20 **`nav_loop_guard` 全模式移除**；§2.15 探索 #8 CLI 通过；`explore_preroll` action 双检+8s 稳定；§2.13 更新 |
| v2.4 | 2026-06-26 | §2.18 MCP `explore_area` `external_tf` 误报修复（t1 复现）；§2.19 Debug 建图/探索前沿分流；explore polling 30s/15s |
| v2.3 | 2026-06-26 | §2.15 对齐干净包 explore 启动（fail-open）；§2.16 预滚动 0.20m/s；§2.17 最小速度 0.10/0.08 |
| v2.2 | 2026-06-26 | §2.15 探索 strict lifecycle 门控（**已回滚**，v2.3 取代） |
| v2.1 | 2026-06-26 | §2.13 v2.1：`rviz_goal_bridge` + lifecycle 门控；**#13a 实机通过** |
| v2.0 | 2026-06-26 | §2.13 初版：nav_loop_guard（后回滚 GoalTool/GoalUpdatedController） |
| v1.9 | 2026-06-26 | §2.12 标为实机未通过；增 v1.9 待办（anchor 防跳回、heartbeat 不误清图） |
| v1.8 | 2026-06-26 | §2.12 首版补丁：hint + pose_map/TF fallback（用户复测未通过） |
| v1.7 | 2026-06-26 | §2.11 异步任务 polling_guidance + get_task_status retry_after_sec；§8 增 load_map_version 验收项 |
| v1.6 | 2026-06-26 | §2.10 load_map_version 定位模式先 ingest checkpoint，修复 Debug 空白 |
| v1.5 | 2026-06-26 | §2.9 2D Pose 重定位：独立 publisher + `/global_pose` + 乐观 UI；§8 增 D10 |
| v1.4 | 2026-06-26 | §2.1 日志 tail 轮询；§2.8 2D Goal 原点折返修复；§8 扩 D6–D9 |
| v1.3 | 2026-06-26 | §2.7：`save_active_map_version` MCP 响应精简（去掉栅格 data 数组，避免 Agent token 超限） |
| v1.2 | 2026-06-26 | §2.6：`/map_obstacles` 关闭 live `/go2/cloud` 烙印；Debug 图层构成元数据；§8 D1/D3/日志项更新 |
| v1.1 | 2026-06-26 | env.sh 注入、定位直读 db、GO2_NAV_* 废弃、实机测试清单拆分 |
| v1.0 | 2026-06-26 | 初版迁移清单 |

> 实机测试细项见：`temp_docs/GO2_RTABNAV2_实机测试清单.md`