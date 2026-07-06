# Phase 5 todolist — explore_runtime 接入

> 2026-07-06
> lane：phase5-executor

## 必做(本 lane 范围)

- [x] 1. Read 现状 + proto + dimos nuwax-exploration 实际 API
  - `services/geometry/nuwax_geometry_runtime/proto/geometry_runtime.proto`
  - `services/geometry/nuwax_geometry_runtime/src/server.rs`
  - `services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs`
  - `services/geometry/geometry_runtime_provider.py`
  - `/home/ml/Deeprob/dimos/rust重写与移植/nuwax-workspace/crates/nuwax-exploration/src/lib.rs`
  - `nuwax-exploration/src/wavefront_adapter.rs`
  - `nuwax-exploration/src/runtime.rs`
  - `nuwax-exploration/src/conversions.rs`
  - `nuwax-nav-types-messages/src/geometry.rs`(确认 Timestamp 字段)
  - `nuwax-nav-types-messages/src/goal.rs`(确认 NavigationGoal 字段)

- [x] 2. proto additive 扩展
  - `ExploreRequest` / `StopExplorationRequest` / `ExplorationState` /
    `ExplorationSourceDiag` / `ExplorationDiagnostics` 5 个新 message
  - 4 个新 RPC:`StartExploration` / `StopExploration` / `GetExplorationState` / `GetExplorationDiagnostics`
  - `services/geometry/nuwax_geometry_runtime/proto/geometry_runtime.proto` 已落盘

- [x] 3. proto stub 再生成(Python + Rust 同提交)
  - Python:`.venv_model_download/bin/python3 -m grpc_tools.protoc` 再生成
    `geometry_runtime_pb2.py` / `geometry_runtime_pb2_grpc.py`
  - 兼容 patch:`ValidateProtobufRuntimeVersion` 改 5.29.6
    (匹配本仓 .venv_model_download 运行时)
  - 兼容 patch:`geometry_runtime_pb2_grpc.py` 改 `from . import ...` 相对导入
  - Rust stub 由 `cargo build` 触发的 `tonic_build` 生成(留 P3/P4 fix 之后 cargo test)

- [x] 4. `src/explore_runtime.rs` 新建
  - `ExploreRuntimeHandle` 包装 `Arc<tokio::sync::Mutex<ExploreRuntimeInner>>`
  - `start` / `stop` / `tick` / `diagnostics` 四个 async API
  - `parse_arbitration_policy` 字符串 ↔ `ExplorationArbitrationPolicy` 映射
  - 默认 source = `WavefrontGoalSourceAdapter`(dimos wavefront_adapter 路径)
  - 8 个 unit / tokio test(policy 解析、stop reason、start/stop 状态机、不 panic)
  - `cargo check` 在 explore_runtime.rs 范围 0 error

- [x] 5. `lcm_runtime.rs` 集成
  - `GeometryRuntimeHandle` 新增 `pub explore: ExploreRuntimeHandle` 字段
  - `explore_ready_poll` 改 `true`(handle 在 spawn 必注入)
  - `spawn_geometry_runtime` 末尾 `ExploreRuntimeHandle::default_with_wavefront()`

- [x] 6. `server.rs` 4 个 RPC handler
  - `start_exploration` / `stop_exploration` / `get_exploration_state` / `get_exploration_diagnostics`
  - 字段转换 proto ↔ ExploreRuntimeSnapshot
  - `get_exploration_state` handler 取 `state.pose.(x,y)` 作为 controller snapshot robot position

- [x] 7. `main.rs` 注册
  - `mod explore_runtime;`

- [x] 8. Python `GeometryRuntimeProvider` 4 个方法
  - `start_exploration` / `stop_exploration` / `get_exploration_state` / `get_exploration_diagnostics`
  - `_exploration_state_to_dict` module-level helper
  - 失败统一返回 `None`(不假装成功)

- [x] 9. 验收命令
  - `bash scripts/ci/no_ros2_residual.sh` PASS
  - `PYTHONPATH=. .venv_model_download/bin/python3 -m compileall -q services core adapters providers gateways skills contracts` PASS
  - `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` PASS
  - `! rg -n '_nav2_action_provider.*return|...'` 0 hit
  - `! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|...'` 0 hit
  - `! rg -n '_platform_exploration_runtime\s*as ExplorationProvider|return\s+self\._platform_exploration_runtime'` runtime.py:5481 hit
    **本 lane 不要求本项 0 hit** —— 这是 Phase 6 的责任,把 `PlatformGeometryRuntime.get_platform_exploration_state` 默认路径改走 `remote_provider`。
    执行方案 §3 Phase 5 第 3 项只要求"`_resolve_exploration_backend` 优先返回 remote provider"，
    这部分 Phase 0 已落盘(`runtime.py:3232-3249`),本 lane 不重写。
  - `rg -n "StartExploration|..." server.rs` 4 hit

- [x] 10. 文档
  - `50_探索接入explore_phase5.md` 落盘
  - `feedback_phase5.md` 落盘
  - `todolist_phase5.md` 落盘(本文件)

## 不在本 lane 范围(留给其他 lane)

- [ ] Phase 6:`PlatformGeometryRuntime.start_exploration` / `stop_exploration` /
      `get_exploration_state` 改走 `remote_provider` (即 `GeometryRuntimeProvider`)
- [ ] Phase 6:`services/navigation/exploration_runtime.py` 旧 Python 探索 owner
      降级为 legacy/manual
- [ ] Phase 3 完成:`WavefrontInputReceiver::ingest_costmap` 桥接到 `explore_runtime.rs::ingest_costmap`
- [ ] Phase 4 完成:`nav_runtime.rs` cargo check 22 error 修复
- [ ] Phase 7:Go2 实机探索 + 导航 + cancel zero cmd_vel 闭环验收
- [ ] TARE / Terrain native bridge(P1 profile)
- [ ] 探索停止条件全部 4 类(no information gain / no reachable free space /
      consecutive failures / user cancel)的真实触发(目前 controller 接受 enum,
      WavefrontGoalSourceAdapter 已实现 3 类;user cancel 由 controller.cancel() 触发)
