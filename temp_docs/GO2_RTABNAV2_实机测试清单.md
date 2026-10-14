# GO2_RTABNAV2 实机测试清单

- **版本**：v2.8
- **日期**：2026-06-26
- **正式入口**：`services/geometry/ros2_ws/src/GO2_RTABNAV2/go2.sh`
- **地图话题**：`/map_obstacles`（单图层；**不含 live `/go2/cloud` 扫描烙印**）
- **用法**：测完一项将 `☐` 改为 `☑` 或 `[x]`
- **同步规则**：与 `GO2_RTABNAV2_migration_checklist.md` 同步更新

> 代码侧迁移进度见：`temp_docs/GO2_RTABNAV2_migration_checklist.md`

---

## 一、环境与启动（先做）

| # | 检查项 | 命令 / 预期 | 状态 |
|---|--------|-------------|------|
| 1 | Bootstrap 检查 | `python3 scripts/bootstrap.py check --robot go2 --geometry` → Go2SLAM 全 OK | ☐ |
| 2 | 工作区包前缀 | `cd services/geometry/ros2_ws/src/GO2_RTABNAV2/ws && source source_go2.bash && ros2 pkg prefix go2_slam_nav` | ☐ |
| 3 | Rust bridge 已编译 | `services/geometry/nuwax_ros_bridge/target/release/nuwax-ros-bridge` 存在 | ☐ |
| 4 | 启动 gateway | `bash scripts/start_gateway.sh` 无报错 | ☐ |
| 5 | 健康检查 | `curl http://127.0.0.1:8766/api/health` 正常 | ☐ |

---

## 二、手工 go2.sh（CLI 直测）

工作目录：`services/geometry/ros2_ws/src/GO2_RTABNAV2/`

| # | 模式 | 命令 | 预期 | 状态 |
|---|------|------|------|------|
| 6 | 建图 | `./go2.sh --mapping [db]` | SLAM + Nav2 拉起，`/map_obstacles` 有数据 | ☑ |
| 7 | 定位 | `./go2.sh --localization <已有.db>` | **直读传入 db**（不复制 session）；`localize_only:=true` | ☐ |
| 8 | 探索 | `GO2_USE_RVIZ=0 ./go2.sh --explore --fresh` | 预滚动 1m+360°@0.30m/s → `Nav2 action 就绪且已稳定` → `explore_node` 追前沿、狗移动；**无** `[nav_loop_guard]`；可连续换前沿 | ☑ v2.5 CLI 通过 |

---

## 三、平台能力（经 gateway）

| # | 能力 | 操作 | 预期 | 状态 |
|---|------|------|------|------|
| 9 | 激活地图 | `activate_map`（有数据的版本 db） | 拉起 `go2.sh --localization <db>`，`/map_obstacles` 有数据 | ☐ |
| 9b | load_map_version 立显 | `load_map_version(map_name=h1)` 成功后开 Debug | **立即**显示 checkpoint 地图（不必等 `/map_obstacles` 首帧）；`map_loaded=true` | ☐ |
| 9c | 异步轮询指引 | Agent 调 `load_map_version` → 按响应 `next_step_hint` 等待 ~45s 再 `get_task_status` | 接受响应含 `polling_guidance`；running 含 `retry_after_sec`；**不**秒级连刷（总 poll ≤ 8 次） | ☐ |
| 9d | explore 轮询指引 | Agent 调 `explore_area` → 等 ~30s 再 `get_task_status`，之后每 15s | 接受响应含 `polling_guidance`；**不**查 TF / **不** `relocalize`；看 `frontier_count` | ☐ |
| 10 | 自主探索 | `explore_area`（如 t1） | 狗移动；`frontier_count>0`；`waiting_for` **非** `external_tf`；终态 `task.succeeded` | 🟡 t1 狗已动但 MCP 误报 external_tf（**v2.4 已修**，重启 gateway 后复测） |
| 10b | 探索 MCP 状态 | `explore_area` 运行中 `get_active_map` / `get_task_status` | `message` 含「正在探索」；`localization_ready` 有 `/go2/pose_map` 时为 true；Agent 不触发 relocalize | ☐ |
| 11 | 遥控建图保存 | `start_mapping` → 遥控走动 → `save_active_map_version` | 响应精简（无 `occupancy_grid.data`）；含 `version_id`/`data_cell_count`/`next_step_hint` | ☐ |
| 11b | 建图不误显前沿 | `start_mapping` 后开 Debug | **不**显示探索前沿圆点；地图刷新平稳（§2.19） | ☐ |
| 12 | 停止建图 | `stop_mapping` | `go2_slam_nav` / `explore_node` / `rtabmap` 等进程退出 | ☐ |
| 13 | 导航到点 | Debug 2D Goal 或 `navigate_to_pose` | Nav2 执行，`/plan` 有路径 | ☐ |
| 13b | 2D Goal 无原点折返 | Debug 2D Goal（目标远离 map 原点） | **不**先朝 `(0,0)` 走约 3s；`/plan` 直指点击点 | ☐ |
| 13a | 原生建图首个 2D Goal | `./go2.sh --mapping --fresh`；等日志 `[rviz_goal_bridge] Nav2 已就绪`；RViz **2D Goal** | 机器狗移动；RViz 见红色 `/plan`；**无** send_goal failed / bt_navigator exit -11 | ☐ v2.8 待复测 |
| 13e | 建图连续 2D Goal | 原生 `go2.sh --mapping --fresh` RViz 连发两个 Goal | **无** `nav_loop_guard` lifecycle 恢复；回环后第二个 Goal 可正常导航 | ☐ |
| 13f | 导航直线少 S 形 | mapping/localization 下发**直线** 2D Goal（路径近似直线） | 机器狗**贴绿线**前进，无明显左右 S 形摆动；速度更慢更稳（v2.7） | ☐ |
| 13c | 2D Pose 重定位 | Debug 2D Pose（定位模式） | 三角位姿跳转；`ros2 topic info /initialpose` 有 rtabmap 订阅者 | ☐ |
| 13d | 2D Pose 不跳回 | `load_map_version` 后 2D Pose | 三角**保持**在目标附近 ≥10s，不瞬间拉回旧位；遥控后三角跟随 | ❌ 未通过 |

---

## 四、Debug 面板（D1–D10）

| # | 项 | 预期 | 状态 |
|---|-----|------|------|
| D1 | 2D 地图 | 与 RViz Route B 一致；**无随扫描扇形刷新的 live 点云烙印** | ☐ |
| D2 | 探索前沿 | `explore_area` 时 `explore/frontiers` 有反馈；`start_mapping` 时**不**显示前沿 | ☐ |
| D3 | 地图摘要 | `来源话题=/map_obstacles`；`图层构成=RTAB /map + /cloud_obstacles（不含 live 扫描）` | ☐ |
| D4 | 规划线 | 2D Goal 后画布有 `/plan` 路径 | ☐ |
| D5 | 无误导缓存 | 未启几何时不显示旧 `/map` 残留帧 | ☐ |
| D6 | 机器狗位姿 | 地图上有三角定位标；`位姿定位` 卡片非「缺失」 | ☐ |
| D7 | 2D Goal | 点目标后 Nav2 接受（非「无实时位姿」）；机器狗开始移动 | ☐ |
| D8 | 运行日志搜索 | 日志区搜「建图」「导航」「探索」有 `geometry.go2.sh` / 进度监控匹配（含 `/api/debug/logs/tail` 轮询） | ☐ |
| D9 | 2D Goal 无原点折返 | 点远离原点的目标后 | 机器狗**直接**朝目标转向；绿线不先指向 map 原点 `(0,0)` | ☐ |
| D10 | 2D Pose 重定位 | 定位模式拖拽 2D Pose | 三角位姿立即跳转；数秒内 TF 收敛；无「无 RTAB-Map 订阅者」警告 | ☐ |
| D10b | 2D Pose 不跳回/不冻结 | `load_map_version` 后 2D Pose + 遥控 | 不跳回旧位；地图界面不死；遥控时三角持续更新 | ❌ 未通过 |

---

## 五、日志

| # | 检查项 | 命令 / 预期 | 状态 |
|---|--------|-------------|------|
| 14 | 主日志含 go2.sh | `grep 'geometry.go2.sh' nuwax_robot_bridge.log` 有输出 | ☐ |
| 14b | Debug 日志 tail | 打开 Debug 面板后日志区能搜到「建图」（无需只靠 WebSocket 实时流） | ☐ |
| 15 | 侧车日志 | `runtime_data/geometry/logs/external_slam_nav2.log` 有探索/定位日志 | ☐ |
| 16 | 探索终态 | 日志含 `exploration_complete` 或 explore_lite 完成标记 | ☐ |
| 17 | obstacle 图层 | `grep '已发布 /map_obstacles' external_slam_nav2.log` → **「实时 0 点」** | ☐ |

---

## 六、清理与回归

| # | 检查项 | 预期 | 状态 |
|---|--------|------|------|
| 18 | 进程清理 | 重启 gateway 后无残留 `go2_nav`（旧栈）误杀/漏杀 | ☐ |
| 19 | `.env` 注入 | go2 启动日志可见 `occupancy=/map_obstacles`（`[env.sh]` 行） | ☐ |
| 20 | DB 根目录 | 无 db 参数时默认 `runtime_data/geometry/rtabmap/live_rtabmap.db` | ☐ |
| 21 | Nav2 实时避障仍可用 | 近距离障碍仍由 local costmap `voxel_layer(/go2/cloud)` 处理 | ☐ |

---

## 七、代码完成度（已实现，无需再测代码）

| 模块 | 状态 |
|------|------|
| 单图层 `/map_obstacles`（Debug + Rust bridge） | ✅ 已完成 |
| `scripts/env.sh` + `.env` 平台注入 | ✅ 已完成 |
| 定位直读传入 `.db` | ✅ 已完成 |
| `GO2_NAV_*` 废弃 | ✅ 已完成 |
| bootstrap / start_gateway 对齐 GO2_RTABNAV2 | ✅ 已完成 |
| explore_lite 元数据 / 前沿 topic | ✅ 已完成 |
| `/map_obstacles` 关闭 live `/go2/cloud` 烙印 | ✅ 已完成 |
| Debug `#mapMeta` 图层构成说明 | ✅ 已完成 |
| MCP `save_active_map_version` 响应精简（无栅格 data） | ✅ 已完成 |
| `colcon build go2_slam_nav` 同步 install（修复 live_cloud 仍开启） | ✅ 已完成 |
| Debug 定位：`/go2/pose_map` + `go2_base_link` TF 对齐 | ✅ 已完成 |
| 2D Goal 读 `get_external_live_pose`；go2 日志进 `logs.entry` | ✅ 已完成 |
| Debug 主日志 tail：`/api/debug/logs/tail` + `pollLogFileTail()` | ✅ 已完成 |
| 2D Goal 原点折返修复：cancel 等待 + 去 `/goal_pose` 双发 + stamp=0 | ✅ 已完成 |
| 2D Pose 重定位：独立 publisher + 仅 `/initialpose` + 乐观 UI | ✅ 已完成 |
| 2D Pose 跳回修复 v1.8：hint + pose_map/TF fallback | ⚠️ 已合入，**实机未通过**（§2.12 阻塞） |
| 原生 2D Goal 链路 v2.1：`SetGoal` + `rviz_goal_bridge` + lifecycle 就绪门控 | ✅ 已完成；**#13a 实机通过** |
| **`rviz_goal_bridge` v2.8**（Foxy SetGoal→action；双检+8s 稳定；`nav2_startup_delay` 20s） | ✅ 已合入；**#13a 待复测** |
| **`nav_loop_guard` 全模式移除 v2.5**（mapping/explore/localization 均不 launch） | ✅ 已合入；**#8 ☑**；mapping 日志应无 `[nav_loop_guard]` |
| 探索就绪门控 v2.5：`explore_preroll` action 双检 + 8s 稳定；explore 未就绪不拉黑 | ✅ 已合入；**#8 ☑** |
| 探索换点节流 v2.5：`min_goal_interval` 10s；`cmd_vel_stale` 12s | ✅ 已合入 |
| MCP explore `external_tf` 误报修复 v2.4：`/go2/pose_map` + frontier 优先 | ✅ 已合入；**#10/#10b 待复测**（需重启 gateway） |
| Debug 建图/探索前沿分流 v2.4（observer + debug_panel） | ✅ 已合入；**#11b/D2 待复测** |
| explore_area polling_guidance 30s/15s + 勿 relocalize 指引 | ✅ 已合入 |
| GO2 探索速度 v2.5：预滚动 0.30m/s、360°；`min_goal_interval` 10s | ✅ 已合入；**#8 ☑** |
| 导航速度 v2.7：sport_ctrl/DWB 最小 **0.05**；线/角上限 **0.15/0.18** | ✅ 已合入；**#13f 待复测** |
| load_map_version 定位模式先 ingest checkpoint（Debug 不空白） | ✅ 已完成 |
| 异步任务 polling_guidance + get_task_status `retry_after_sec`（防 Agent 高频连刷） | ✅ 已完成（含 explore 30s/15s · v2.4） |

---

## 八、验收后待定（暂不测）

| 项 | 说明 |
|----|------|
| 删除 `GO2_RTABMAP/` | §8 全通过后执行（审批项 F） |
| Nav2 参数从 `.env` 物化到 yaml | 审批项 A，当前未做 |
| README / 旧交接文档更新 | 文档项，非阻塞实机 |

---

## 建议测试顺序

```text
1 → 5（环境与启动）
  → 6 → **13a** / **13f** / 7 / 8（选手工 CLI；建图导航先 #6+#13a+#13f）
  → 9 → 9c → 9d → 10b → 13d（平台能力 + **#9b/#9c/#9d 轮询 / #10/#10b 探索 MCP / #13b #13c #13d**）
  → D1 → D10b（Debug 面板，重点 D1/D3/D7/D8/D9/D10/D10b）
  → 14 → 17（日志，含 #14b tail 轮询）
  → 18 → 21（清理回归）
```

---

## 关键配置速查（`.env`）

```bash
NUWAX_GEOMETRY_EXTERNAL_SCRIPT=.../GO2_RTABNAV2/go2.sh
NUWAX_GEOMETRY_OCCUPANCY_TOPIC=/map_obstacles
NUWAX_GEOMETRY_EXTERNAL_FRONTIER_TOPIC=explore/frontiers
NUWAX_BACKEND_DATABASE_ROOT=.../runtime_data/geometry/rtabmap
GO2_OBSTACLE_GRID_LIVE_CLOUD_ENABLED=false
GO2_OBSTACLE_GRID_PUBLISH_ON_CLOUD=false
```

**原生 go2.sh 速度默认（v2.7）**：

| 项 | 值 |
|----|-----|
| 探索预滚动前进 | **0.30 m/s**（v2.5 干净包，不变） |
| 探索预滚动旋转 | 360° (6.28 rad) |
| 探索换点间隔 `min_goal_interval` | **10 s** |
| 探索 cmd_vel 卡住拉黑 | **12 s** |
| **导航** sport_ctrl 最小线/角速度 | **0.05 / 0.05** |
| **导航** DWB `max_vel_x` / `max_vel_theta` | **0.15 / 0.18** |
| **导航** DWB `min_speed_xy` / `min_speed_theta` | **0.05 / 0.05** |

**探索启动时序（v2.5）**：

```text
t=0s     SLAM + sport_ctrl + tf_relay（无 nav_loop_guard / stack_health / goal_bridge）
t=20s    Nav2 autostart
t=30s    explore_preroll（前进 1m @ 0.30m/s + 旋转 360°）
t≈preroll 结束  等 navigate_to_pose + follow_path action，稳定 8s
         → explore_lite 启动 → 追前沿
```

**mapping 模式**：与 explore 共用 `nav_go2.launch.py`，同样 **不启 `nav_loop_guard`**。

---

## 进度统计

| 区块 | 待测项 | 已完成（代码侧） |
|------|--------|------------------|
| 一、环境与启动 | 5 | — |
| 二、手工 go2.sh | 2 | 2（#6 ☑ · #8 ☑ v2.5） |
| 三、平台能力 | 13 | — |
| 四、Debug D1–D10b | 11 | — |
| 五、日志 | 5 | — |
| 六、清理回归 | 4 | — |
| **实机合计** | **40** | **27 项代码已完成** |

---

## 变更记录

| 版本 | 日期 | 摘要 |
|------|------|------|
| v2.7 | 2026-06-26 | 导航再降速：min **0.05**；max **0.15/0.18**；加速度放缓 |
| v2.6 | 2026-06-26 | 导航降速防 S 形：sport_ctrl/DWB min **0.08**；max **0.22/0.28**；增 **#13f** |
| v2.5 | 2026-06-26 | **`nav_loop_guard` 全模式移除**；**#8 ☑** 探索 CLI 通过；更新时序/速度表；#13a 去 stack_health 依赖 |
| v2.4 | 2026-06-26 | §2.18 MCP `external_tf` 误报（t1）；§2.19 Debug 前沿分流；增 **#9d/#10b/#11b**；**#10 🟡 部分通过** |
| v2.3 | 2026-06-26 | 对齐干净包 explore 启动；预滚动 0.20m/s；最小速度 0.10/0.08；**#8 待复测** |
| v2.2 | 2026-06-26 | strict lifecycle 门控（已回滚，v2.3 取代） |
| v2.0 | 2026-06-26 | **#6/#13a 通过**（原生 mapping + 2D Goal 机器狗移动）；§2.13 v2.1 补丁 |
| v1.9 | 2026-06-26 | #13e 建图连续 2D Goal / bt_navigator SIGSEGV；§2.13 |
| v1.8 | 2026-06-26 | #13d/D10b 标 ❌ 未通过；§2.12 阻塞；v1.8 补丁验收失败 |
| v1.7 | 2026-06-26 | #13d/D10b 2D Pose 跳回/位姿冻结修复；§七 +1；实机合计 37 项 |
| v1.6 | 2026-06-26 | #9c 异步轮询指引；§七 +1；实机合计 35 项 |
| v1.5 | 2026-06-26 | #9b load_map_version Debug 立显 checkpoint；§七 +1 |
| v1.4 | 2026-06-26 | D10/#13c 2D Pose 重定位；§七 +1；实机合计 33 项 |
| v1.3 | 2026-06-26 | D9/#13b 2D Goal 原点折返；D8/#14b 日志 tail；§七 代码完成度 +2 |
| v1.2 | 2026-06-26 | 增加遥控建图 #11；记录 install 未重建导致 live_cloud 仍开的根因与 colcon 修复 |
| v1.1 | 2026-06-26 | 增加 obstacle 图层验证（#16/#20）；D3 图层构成；`.env` GO2_OBSTACLE_GRID_* |
| v1.0 | 2026-06-26 | 初版实机测试清单 |