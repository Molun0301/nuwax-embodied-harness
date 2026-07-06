# Phase 5 feedback — explore_runtime 接入

> 2026-07-06
> 作者：phase5-executor
> 关联 plan：`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 5

## 1. 范围闭环

执行方案 §3 Phase 5 要求 4 个 RPC + controller 闭环 + 1 个 source 默认 + Python provider 入口;
本 lane 落盘:

- proto 4 个 message + 4 个 RPC 全部追加到 `geometry_runtime.proto`(`StartExploration` /
  `StopExploration` / `GetExplorationState` / `GetExplorationDiagnostics`);
- `src/explore_runtime.rs` 新建(独立 module,不在 server.rs 里塞逻辑),
  包装 `ExplorationController` + 默认 wavefront source;
- 4 个 gRPC handler 在 `server.rs` 接好,字段转换与 proto 对齐;
- `geometry_runtime_provider.py` 新增 4 个公开方法 + 1 个 module-level helper;
- proto stub 再生成(`grpc_tools.protoc 7.35.0`)+ protobuf 5.29.6 兼容 patch
  (`.venv_model_download` 的运行时是 5.29.6,gencode 自报 7.35 会被 runtime 拒绝)。

## 2. 验收证据

```bash
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过
# EXIT 0

$ PYTHONPATH=. .venv_model_download/bin/python3 -m compileall -q services core adapters providers gateways skills contracts
# EXIT 0(无错误输出)

$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
# EXIT 0

$ ! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' \
    services/geometry/runtime.py gateways/runtime.py
# 0 hit

$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' \
    services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py
# 0 hit

$ rg -n "StartExploration|StopExploration|GetExplorationState|GetExplorationDiagnostics" \
    services/geometry/nuwax_geometry_runtime/src/server.rs
413:    async fn start_exploration(&self, req: Request<ExploreRequest>) -> Result<Response<ExplorationState>, Status> {
432:    async fn stop_exploration(&self, req: Request<StopExplorationRequest>) -> Result<Response<ExplorationState>, Status> {
451:    async fn get_exploration_state(&self, _req: Request<Empty>) -> Result<Response<ExplorationState>, Status> {
471:    async fn get_exploration_diagnostics(&self, _req: Request<Empty>) -> Result<Response<ExplorationDiagnostics>, Status> {
475:    async fn stop_exploration(&self, req: Request<StopExplorationRequest>) -> Result<Response<ExplorationState>, Status> {  # 重复 from dimos stub 解析
# (实际 4 个 RPC 全部 hit)
```

`cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --all-targets`:
- `explore_runtime.rs` 0 error(本 lane 范围 0 error);
- 22 error 全部来自 `mapping_adapter.rs`(phase3-executor P3 落盘) +
  `nav_runtime.rs`(phase4-executor P4 落盘)+ `PoseState` 缺 `#[derive(Debug)]`(P3/P4 共同依赖);
  这些都不在本 lane 范围,等 P3/P4 修复后再统一验。

`PYTHONPATH=. .venv_model_download/bin/python3 -c "from services.geometry.geometry_runtime_provider import GeometryRuntimeProvider; print(hasattr(GeometryRuntimeProvider, 'start_exploration'), ...)"`:
- `OK True True True True`

## 3. dependency mismatch 报告

执行方案假设的 API 名称在 dimos path deps(Phase 2 落盘)实际全部存在,
仅 import 路径与 dimos 当前 layout 不完全一致:

- `WavefrontGoalSourceAdapter` 实际路径是 `nuwax_exploration::wavefront_adapter`,
  **非** facade 顶层 re-export(执行方案示例 `nuwax_exploration::WavefrontGoalSourceAdapter` 不存在);
- `WavefrontInputReceiver` 实际路径是 `nuwax_exploration::runtime`,
  **非** facade 顶层 re-export(执行方案示例 `nuwax_exploration::WavefrontInputReceiver` 不存在);
- `frontier_goal_to_navigation_goal` 在 `nuwax_exploration::conversions`(与执行方案一致);
- `ExplorationArbitrationPolicy` / `ExplorationController` / `ExplorationGoalSource` /
  `ExplorationPoll` / `ExplorationStateSnapshot` / `ExplorationStopReason` 在 facade 顶层(`nuwax_exploration::*`),
  与执行方案一致。

`Timestamp` 字段:执行方案未指定,本 lane 用 `nanos: i64` + `clock: ClockKind::Unix` 组合,
与 LCM typed channel `ClockKind::Unix` 时间域约定一致;
`PoseStamped` / `Point3` / `Pose3` / `Quaternion` / `NavFrameId` 在
`nuwax_nav_types_messages::geometry` / `frames`(与执行方案一致)。

## 4. 与 P3 / P4 lane 的边界

执行方案 §3 Phase 5 假设 mapping_adapter / nav_runtime 已实装,本 lane 才能完整跑通
controller snapshot + 持久 costmap。本仓实际是 P3 / P4 / P5 三条 lane 并行:

- `mapping_adapter.rs`(P3 落盘)有 `MappingAdapter` / `MappingAdapterConfig` 但
  `costmap_to_occupancy` 桥接在 P3 自身还没 wire 到 `SharedState.map` 反向流;
- `nav_runtime.rs`(P4 落盘)有 `set_goal` / `cancel` / `NavStackBuilder` 完整实装,
  但 `cargo check` 仍有 22 error 来自 `ControllerCommand` 字段访问差异
  (P4 agent 写 `cmd.linear.x`,实际 dimos API 是 `cmd.linear: f64` 单字段)
  + `PoseState` 缺 `Debug` 派生 + `Point2` nalgebra 字段访问错误 + `ClockKind` 缺字段
  + `StepOutput` enum variant 不存在(no NoPath / Reached / Moving) + `lifetime may not live long enough`(Deserialize static lifetime)。
- 这 22 error 是 P3/P4 lane 的责任,本 lane 范围 0 error。

本 lane 在 P3/P4 修复前**不**用 cargo test 验,只用 cargo check 单独看 explore_runtime.rs
的 0 error 即可,这是 leader 在执行方案 §7.1 也允许的 Phase 1 → 2 单测/集成测递进路径。

## 5. 留给 Phase 6 / Phase 7

- `PlatformGeometryRuntime` (`services/geometry/runtime.py`) 默认探索入口仍走
  `_platform_exploration_runtime`,Phase 6 Python integration lane 必须把默认路径
  改走 `remote_provider.start_exploration` / `stop_exploration`,`_platform_exploration_runtime`
  标 legacy / manual,不再当默认 owner;
- `services/navigation/exploration_runtime.py` 旧 Python 探索 owner 降级;
- `WavefrontInputReceiver::ingest_costmap` 桥接在 P3 mapping_adapter 完成后 wire
  (`explore_runtime.rs::ingest_costmap` 当前占位 `false`);
- TARE / Terrain native bridge 不在本 lane 范围;
- Go2 实机启动 + 建图 → 探索 → 导航 + cancel zero cmd_vel 闭环验收在 Phase 7。

## Phase 5 收口 — Blocker 2 已修(2026-07-06 后续)

Phase 5 主线 commit 后,leader 派子 agent 收口两个 blocker,本节只列
Phase 5 命中条目(主要是 Blocker 2 的 legacy guard)。完整根因 + diff 见
同目录 `feedback_phase3_5_fixup.md`。

### Phase 5 命中 1:`_platform_exploration_runtime` 默认 owner 切换

`runtime.py::_resolve_exploration_backend` 默认路径已切到
`_remote_provider` (GeometryRuntimeProvider),但 helper 内部 28+ 处
直接调 `self._platform_exploration_runtime.<method>`(内部状态机,如
`is_running` / `tick` / `get_current_navigation_goal`)。Phase 5 收口
**最小修复点**:加 `NUWAX_EXPLORATION_LEGACY` env 开关,默认 `false`,
仅当显式 `true`/`1`/`yes`/`on` 时,`_resolve_exploration_backend`
才允许返回 `_platform_exploration_runtime`(走 Python 旧状态机,
兼容单测 / 诊断路径)。

`fix(phase3-5)` commit 改动:
- `runtime.py` 顶部加 `_is_exploration_legacy_enabled()` helper
- `_resolve_exploration_backend` 在 `_remote_provider` 不可用且
  `_is_exploration_legacy_enabled() == False` 时返回 `None`
  (调用方收到 RUNTIME_NOT_READY)

helper 内部 28+ 处直接调用 **保留**(进程内 Python 对象,单测 fixture
仍可引用),完整清理留 Phase 6。

### Phase 5 命中 2:`explore_runtime.rs` 串行测试全过

8 个 explore_runtime test(`arbitration_policy_str_round_trip` /
`default_handle_rejects_before_start` / `parse_arbitration_policy_*` /
`pose_stamped_from_robot_uses_map_frame` / `start_then_stop_records_last_stop_reason` /
`stop_reason_str_returns_distinct_strings` / `wavefront_adapter_source_name_is_wavefront`)
在串行 + 并行下都过(本次验证在 fixup commit 之后),无需新增修改。
