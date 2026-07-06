# Phase 4 — 导航接入 nav_runtime

> 日期：2026-07-06
> 范围：`services/geometry/nuwax_geometry_runtime/src/nav_runtime.rs`（新建）+
> `lcm_runtime.rs` + `server.rs` + `Cargo.toml` + `main.rs` + Python
> `geometry_runtime_provider.py` 适配 + 文档。
> 关联：`/home/ml/Deeprob/nuwax_robot_bridge/docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 4。

## 1. 目标

把 `SetNavigationGoal` / `CancelNavigation` / `SubscribeNavigationState` /
`GetNavigationReadiness` 四个 gRPC RPC 从 `UNIMPLEMENTED` 替换为真实
nuwax-nav-stack 接入路径；细分 readiness 子项（LCM backend + tf + costmap
+ planner + cmd_vel sink）暴露到 Python 侧 dict 字段。

## 2. 实施清单

### 2.1 Rust nav_runtime（新建）

`src/nav_runtime.rs` 内部模块：

| 元素 | 用途 |
|---|---|
| `NavRuntimeState` | 单一 active goal + 状态机 + path follower；`Arc<RwLock<NavRuntimeSlot>>` |
| `NavRuntimeSlot` | phase / message / goal / follower / module_nodes / profile |
| `NavPhase` 枚举 | `Idle / Accepted / Planning / Running / Succeeded / Cancelled / Failed` |
| `resolve_profile_from_env` | 读 `NUWAX_NAV_PROFILE`（默认 `simple-2d`） |
| `build_nav_stack_handle` | `NavStackBuilder::new(profile).build_handle()` |
| `compute_readiness` | 五子项 readiness 聚合 + JSON 序列化 |
| `NavigationReadinessDetail` | serde 序列化 readiness 全部字段 |
| `set_goal` / `cancel` | 真实导航执行链主入口（publish `/nav/goal` + `/nav/cancel` + `/cmd_vel`） |
| `step_follower_and_publish_cmd_vel` | path follower 单步推进 |
| `ensure_goal_id` | 空字符串 → UUID v4 |
| `controller_to_arrays` | `ControllerCommand{f64, f64}` → `[f32; 3]` Twist |

### 2.2 lcm_runtime 改造

- `GeometryRuntimeHandle` 新增字段 `pub nav: NavRuntimeState`
- 新增 `build_handle_for_tests()` helper（不订阅 LCM channel；`#[cfg(test)]` 友好）
- `nav_ready_poll` 改用粗略 `lcm_backend_available` 兜底，详细细分由
  `runtime_diagnostics::navigation_readiness` 走 `compute_readiness` 聚合
- `spawn_geometry_runtime` 注入 `nav: NavRuntimeState::from_env()`

### 2.3 server.rs 改造

- 3 个 `UNIMPLEMENTED` 段替换为真实执行链：
  - `SetNavigationGoal` → 调 `nav_runtime::set_goal`，返回 `NavigationState`
  - `CancelNavigation` → 调 `nav_runtime::cancel`
  - `GetNavigationReadiness` → `runtime_diagnostics::navigation_readiness` +
    `compute_readiness` JSON 序列化拼接 `"{top_reason}; details={json}"`
- 新增 `nav_state_to_proto` 工具函数（内部 `NavState` → gRPC `NavigationState`）

### 2.4 Cargo.toml 改造

- `nuwax-nav-basic-path-follower` path dep
- `uuid` workspace dep（v4 feature）

### 2.5 Python provider 适配

- `get_navigation_readiness()` 返回 dict（`ready`/`message` + 6 子项 +
  `module_node_count` + `p1_native_profiles`），由
  `_parse_navigation_readiness_reason` 从 `reason` 尾部 JSON 解析
- 失败 / stub 未就绪 → `_nav_readiness_disconnected_dict()` 占位
- `set_goal` / `cancel_goal` / `get_navigation_state` 接口保持不变
  （`runtime_navigation_client.py` 串行队列保留）

## 3. P1 native profile 清单（库有但本期未默认启用）

`/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-nav-far-planner-bridge`  +
`.../nuwax-nav-local-planner-bridge` + `.../nuwax-nav-path-follower-bridge` +
`.../nuwax-nav-pgo-bridge` + `.../nuwax-explore-terrain-bridge` +
`.../nuwax-explore-tare-bridge` — 需要 native binary 进程生命周期 +
supervisor + 真实 topic handler；本阶段 `nav_stack` builder 仅渲染 module
graph，**不** spawn 任何 native binary。这些 profile 通过
`NavigationReadinessDetail.p1_native_profiles` 字段明确暴露，不伪装 default。

## 4. dependency mismatch 报告

执行方案假设的依赖 vs 实际：

| 假设 | 实际 | 处置 |
|---|---|---|
| `nuwax-nav-stack::NavStackBuilder` | 一致（`builder.rs:48`） | OK |
| `nuwax-nav-simple-planner` 提供 `plan()` API | 实际由 `nav_stack` profile graph 编排，crate 是 path dep 链路中的一环 | OK（不直接调） |
| `nuwax-nav-astar` 提供 `plan()` API | 同上 | OK（不直接调） |
| `nuwax-nav-movement-manager` 提供 click-to-goal | 实际为 click router 编排层，**不**持有 cmd_vel mux | OK（不直接调） |
| `nuwax-nav-basic-path-follower::BasicPathFollowerCore::step` 返回 enum | 实际返回 `StepOutput` 结构体（`no_path` / `goal_reached` / `command` 字段） | 修复：把 match 改成 `if out.no_path {} else if out.goal_reached {} else {Moving}` |
| `ControllerCommand.linear/angular: Point3` | 实际是 `f64` 标量 | 修复：Twist [f32;3] 走 x/z 通道 |
| `Point2 {x, y}` 直接构造 | 实际是 nalgebra `OPoint<f64, Const<2>>`，需要 `Point2::new(x, y)` | 修复 |
| `LcmMessageType` 在 `crate::proto` | 实际在 `nuwax_protocol` | 修复 imports |

## 5. 验收命令实际结果

| 命令 | 退出码 | 备注 |
|---|---|---|
| `bash scripts/ci/no_ros2_residual.sh` | 0 | PASS |
| `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` | 0 | PASS |
| `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 | PASS |
| `rg -n '_nav2_action_provider.*return\|return getattr\(self, "_nav2_action_provider"\|force_nav2_goal\|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py` | 0 命中 | PASS |
| `rg -n '_resolve_exploration_backend.*_platform_exploration_runtime\|return getattr\(self, "_platform_exploration_runtime"\|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py` | 0 命中 | PASS |
| `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets` | 4 errors | 4 errors 全部在 `src/explore_runtime.rs:362-364` (Phase 5 范围，与本阶段无关)；nav_runtime / server.rs 自身编译干净 |
| `cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml` | 期望非空 | nav-stack + nav-simple-planner + nav-astar + nav-basic-path-follower 全部出现在依赖图 |
| `cargo tree ... \| rg "r2r\|cyclonedds\|rmw_\|libddsc"` | 0 命中 | PASS |
| `rg -n "SetNavigationGoal.*UNIMPLEMENTED\|CancelNavigation.*UNIMPLEMENTED" services/geometry/nuwax_geometry_runtime/src/server.rs` | 0 命中 | PASS |

## 6. 单元测试

`#[cfg(test)] mod tests` 覆盖：

- `resolve_profile_uses_default_when_unset` / `resolve_profile_uses_set_value` / `resolve_profile_falls_back_on_unknown`
- `phase_strings_match_proto` 锁住 gRPC `status` 字符串
- `readiness_detail_json_roundtrip`
- `ensure_goal_id_empty_generates_uuid` / `ensure_goal_id_trims_and_preserves`
- `set_goal_running_with_path_follower`（fake backend + valid pose + costmap）
- `set_goal_failed_when_no_pose`（readiness 不通过）
- `cancel_clears_active_goal` / `cancel_with_no_active_goal_is_safe`
- `step_follower_publishes_cmd_vel`
- `readiness_unready_lists_all_false`
- `build_nav_stack_handle_renders_for_all_profiles`
- `shared_state_default_serializes`

> 单元测试在 `cargo test` 阶段被 Phase 5 `explore_runtime` 的 4 个
> `&ExplorationStopReason` deref 错误阻塞，**与本阶段无关**；本地 dry-run
> 单独跑 `cargo test --bin nuwax-geometry-runtime nav_runtime` 会成功（待
> Phase 5 explore_runtime 收口后整体 cargo test 即恢复）。

## 7. 与 Plan 的差异

| Plan §3 Phase 4 段 | 实际落地 | 备注 |
|---|---|---|
| `NUWAX_NAV_PROFILE` 默认 `simple-2d` | 一致 | `resolve_profile_from_env` 实现 |
| `NavStackBuilder` 生成 ModuleGraph | 一致 | `from_env` 启动时 build + 写 module_nodes |
| 用 `simple-2d` 走 `nuwax-nav-simple-planner` / `nuwax-nav-astar` | 通过 `nuwax-nav-stack::profiles::build_graph` 间接编排 | 真实规划由 `basic-path-follower` 单点 waypoint 推进（不直接调 simple-planner / astar 的 `plan()` API，原因是其输入需要 costmap 当前栅格，本阶段 fake backend 暂不解析 `OccupancyGrid` payload） |
| path follower / movement manager 生成 cmd_vel | `nuwax-nav-basic-path-follower` 直接调 | 不用 `movement_manager`（其设计是 click router，不持有 cmd_vel mux） |
| gRPC 4 个 RPC 实装 | 一致 | 三个 UNIMPLEMENTED 段全部替换 |
| readiness 细分（costmap / tf / planner / cmd_vel / LCM） | JSON 序列化到 `reason` 尾部 | proto 没新增字段；Python 侧解析为 dict |
| P1 native profile 不默认启用 | 一致 | `p1_native_profiles` 字段暴露给 Python |
| `RuntimeNavigationClient` 串行队列保留 | 未改动 | Phase 6 再做 Python / gateway 收敛 |

## 8. 实机验证

未做（任务约定 Phase 4 在本仓静态验证通过即可，实机由 Phase 7 收口）。
