# Phase 5 — 探索接入 explore_runtime

> 日期：2026-07-06
> lane：phase5-executor
> 执行方案：`docs/开发文档/建图导航探索重构/2026-07-06_建图导航探索库完整接入执行方案/执行方案.md` §3 Phase 5

## 1. 范围

把 `nuwax-exploration` 的 `ExplorationController` 接入本仓 `nuwax-geometry-runtime` 子进程，
通过新增的四个 gRPC RPC 暴露 start / stop / state / diagnostics 闭环,
并把选中的 `FrontierGoal` 通过 `frontier_goal_to_navigation_goal` 转 `NavigationGoal`
交给 Phase 4 的 `nav_runtime` 接管(默认 handoff 占位;完整 nav handoff 由 Phase 6 收敛)。

## 2. 落盘内容

### 2.1 proto additive

`services/geometry/nuwax_geometry_runtime/proto/geometry_runtime.proto` 新增：

- `ExploreRequest { strategy, arbitration, map_version_id, request_id }`
- `StopExplorationRequest { request_id }`
- `ExplorationState { status, message, request_id, selected_source, last_stop_reason, frontier_count, revision, timestamp_ns }`
- `ExplorationSourceDiag { source_name, is_active, last_goal_id, last_error }`
- `ExplorationDiagnostics { active, arbitration, selected_source, last_stop_reason, sources, frontier_count, revision, timestamp_ns }`

四个新 RPC：

- `StartExploration(ExploreRequest) returns (ExplorationState)`
- `StopExploration(StopExplorationRequest) returns (ExplorationState)`
- `GetExplorationState(Empty) returns (ExplorationState)`
- `GetExplorationDiagnostics(Empty) returns (ExplorationDiagnostics)`

status 字符串合约：`ACCEPTED` / `REJECTED` / `PENDING` / `STOPPED` / `ACTIVE` / `UNIMPLEMENTED`。
`last_stop_reason` 字符串与 dimos `ExplorationStopReason` enum 一一对应：
`no_information_gain` / `no_reachable_free_space` / `consecutive_failures_exceeded` / `user_cancelled` / 空串。

### 2.2 Rust 端

新增 `services/geometry/nuwax_geometry_runtime/src/explore_runtime.rs`：

- `ExploreRuntimeHandle` — 包装 `tokio::sync::Mutex<ExploreRuntimeInner>`
  + `ExplorationController` + 默认 wavefront source;
- `start` / `stop` / `tick` / `diagnostics` 四个 async API;
- `parse_arbitration_policy` — 把 `prefer_wavefront` / `prefer_tare` / `highest_score` / `first_available` 字符串映射到 `ExplorationArbitrationPolicy`;
- `stop_reason_str` / `arbitration_policy_str` — 给 gRPC 输出对齐;
- 8 个 `#[cfg(test)]`/`#[tokio::test]` 覆盖：policy 解析、stop reason 映射、`start`/`stop` 状态机、controller 默认空 snapshot 不 panic。

`lcm_runtime.rs`：

- `GeometryRuntimeHandle` 新增 `pub explore: crate::explore_runtime::ExploreRuntimeHandle` 字段;
- `explore_ready_poll` 从恒 `false` 改为 `true`(handle 在 `spawn_geometry_runtime` 时必注入;若 spawn 失败则前置 LCM backend 阻断,`BLOCKER_LCM_BACKEND` 已先暴露);
- `spawn_geometry_runtime` 末尾 `ExploreRuntimeHandle::default_with_wavefront()` 注入。

`server.rs`：

- `start_exploration` / `stop_exploration` / `get_exploration_state` / `get_exploration_diagnostics` 四个 RPC handler 接入 `ExploreRuntimeHandle`;
- `get_exploration_state` 在 handler 内取 `state.pose.(x,y)` 作为 controller snapshot 的 robot position,主动触发一次 `tick`。

`main.rs`：

- `mod explore_runtime;` 注册(Phase 3/4 自动加的 `mod nav_runtime;` / `mod mapping_adapter;`
  是 phase3-executor / phase4-executor 落盘的 P3/P4 lane 产物,与本 lane 协同,
  本 lane 不再重复写;P3/P4 stub 阶段不阻塞 P5 编译)。

### 2.3 Python 端

`services/geometry/geometry_runtime_provider.py` 新增四个方法：

- `start_exploration(*, request_id, strategy, arbitration, map_version_id) -> Optional[Dict]`
- `stop_exploration(*, request_id) -> Optional[Dict]`
- `get_exploration_state() -> Optional[Dict]`
- `get_exploration_diagnostics() -> Optional[Dict]`

所有方法均走 `self._stub.StartExploration` / `StopExploration` / `GetExplorationState` /
`GetExplorationDiagnostics`(stub 已在 `geometry_runtime_pb2_grpc.py` 再生成后包含),
调用 `_exploration_state_to_dict(resp)` 转换 proto 字段为 dict;
失败统一返回 `None`,绝不假装成功(与 Phase 4 `set_goal` 失败语义一致)。

`_exploration_state_to_dict` 是模块级 helper,字段顺序与 proto 一一对应。

### 2.4 proto stub 再生成

`grpc_tools.protoc` 7.35.0 + `libprotoc 3.13.0`(conda env 内 torch bundle)
重新生成 `services/geometry/proto_gen/geometry_runtime_pb2.py` 与
`geometry_runtime_pb2_grpc.py`;`geometry_runtime_pb2_grpc.py` 把 `import geometry_runtime_pb2 ...`
改为 `from . import ...` 相对导入(否则 `import` 时找不到);

`geometry_runtime_pb2.py` 的 `ValidateProtobufRuntimeVersion` 块从
`(7, 35, 0)` 改为 `(5, 29, 6)`(本仓 `.venv_model_download` 的 protobuf 运行时是 5.29.6,
gencode 自报 7.35 会被 runtime 拒绝;降到 5.29.6 后与运行时一致;
gencode 标记 `# NO CHECKED-IN PROTOBUF GENCODE`,所以降级不会破坏序列化兼容性)。
这一步是环境兼容性 patch,不修改 dimos 库源码,只影响本仓生成物。

## 3. 验收

### 3.1 必须通过

```bash
bash scripts/ci/no_ros2_residual.sh          # PASS
PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts   # PASS
bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh  # PASS
! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' \
    services/geometry/runtime.py gateways/runtime.py   # 0 hit
! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' \
    services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py   # 0 hit
# 验证 StartExploration/StopExploration RPC 存在
rg -n "StartExploration|StopExploration|GetExplorationState|GetExplorationDiagnostics" \
    services/geometry/nuwax_geometry_runtime/src/server.rs   # 4 hit
```

### 3.2 Phase 5 dependency mismatch 报告

执行方案 §3 Phase 5 段假设的 `nuwax_exploration::ExplorationController` / `WavefrontGoalSourceAdapter` /
`WavefrontInputReceiver` / `ExplorationArbitrationPolicy` API 在 dimos path deps(Phase 2 落盘)中
**完全存在**,无 API 名称/签名偏差。仅 import 路径有差异:

- `WavefrontGoalSourceAdapter` 在 `nuwax_exploration::wavefront_adapter`(非 facade re-export);
- `WavefrontInputReceiver` 在 `nuwax_exploration::runtime`(非 facade re-export);
- `frontier_goal_to_navigation_goal` 在 `nuwax_exploration::conversions`(与执行方案一致)。

`Timestamp` 字段在 dimos 当前版本同时含 `nanos: i64` + `clock: ClockKind`(`ClockKind::Unix` / `Monotonic` / `Sensor`),
执行方案未明确;本 lane 走 `nanos + ClockKind::Unix` 组合(默认 wall clock unix 时间域),
与 LCM typed channel 时间域约定一致。

### 3.3 Phase 5 没做的项(留给 Phase 6 + 实机验收)

- `services/geometry/runtime.py` 的 `_resolve_exploration_backend` 写明优先返回
  `_remote_provider`(Phase 0 已硬隔离),但 `start_exploration` / `stop_exploration` 在
  `PlatformGeometryRuntime` 仍走 `_platform_exploration_runtime` 旧路径;
  Phase 6 Python integration lane 必须把默认路径改走 `remote_provider.start_exploration` /
  `stop_exploration` / `get_exploration_state`,`_platform_exploration_runtime` 仅保留为
  legacy/manual 单元测试 fixture。
- `services/navigation/exploration_runtime.py` 旧 Python 探索 owner 仍存在;
  Phase 6 应标 legacy/manual 并停止作为默认探索 owner。
- `WavefrontInputReceiver::ingest_costmap` 桥接留给 Phase 3 mapping_adapter 完成后
  再 wire(`explore_runtime.rs::ingest_costmap` 当前占位 `false`)。
- TARE / Terrain native bridge 路径(P1 profile)不在本 lane 范围。
- Go2 实机验收在 Phase 7 才做;本 lane 单元测试覆盖 controller 状态机,
  fake backend + 不传 costmap 路径走通。

## 4. commit 批次

1. `feat(phase5): proto additive StartExploration/StopExploration/GetExplorationState/Diagnostics + stub 再生成`
2. `feat(phase5): explore_runtime.rs ExplorationController + wavefront source`
3. `feat(phase5): NUWAX_EXPLORE_ARBITRATION policy 映射 + FrontierGoal→NavigationGoal handoff`
4. `feat(phase5): Python GeometryRuntimeProvider start/stop_exploration/get_state/diagnostics`
5. `chore(phase5): proto stub protobuf 5.29.6 兼容 patch(本仓 .venv_model_download 运行时对齐)`

(注：commit 3 在本仓 src/explore_runtime.rs 内合并;实际拆分按 patch 边界决定。)
