# Phase 4 feedback — 导航接入 nav_runtime

> 日期：2026-07-06
> 范围：Phase 4 nav_runtime（`30_导航接入nav_phase4.md` 同目录）

## 1. 进展

- 新建 `src/nav_runtime.rs`（~960 行含测试），把 3 个 `UNIMPLEMENTED` RPC
  全部替换为真实执行链。
- `lcm_runtime.rs` 增加 `nav: NavRuntimeState` 字段 + `build_handle_for_tests`
  helper；`spawn_geometry_runtime` 注入默认 `simple-2d` profile。
- `server.rs` 3 处 `SetNavigationGoal / CancelNavigation / GetNavigationReadiness`
  实装；新增 `nav_state_to_proto` 转换工具。
- Python `geometry_runtime_provider.py:get_navigation_readiness` 改为解析
  Rust JSON dict 返回完整子项；失败/未就绪走占位 dict。
- `Cargo.toml` 增加 `nuwax-nav-basic-path-follower` path dep + `uuid` dep。
- `main.rs` 增 `mod nav_runtime;`。

## 2. 关键事实

### 2.1 库依赖 mismatch（已修复）

执行方案描述的 API 与实际 dimos 库不一致：

1. `BasicPathFollowerCore::step` 返回 enum — 实际是 `StepOutput` struct
   （含 `no_path` / `goal_reached` / `command` 字段）。
2. `ControllerCommand.linear / angular: Point3` — 实际是 `f64` 标量。
3. `Point2 {x, y}` 字面量构造 — 实际是 nalgebra `OPoint<f64, Const<2>>`，
   必须用 `Point2::new(x, y)`。
4. `LcmMessageType / LcmTopic / RawLcmFrame` 在 `nuwax_protocol` —
   不在 `crate::proto`。

### 2.2 设计决策

- **P1 native profile 不启动**：`nuwax-nav-far-planner-bridge` 等需要 native
  binary 进程生命周期 + supervisor，本阶段不写进程级 API；nav_stack
  builder 仅渲染 module graph，**不** spawn 任何 native binary。这些 profile
  通过 `NavigationReadinessDetail.p1_native_profiles` 字段明确暴露，不伪装
  default。
- **细分 readiness 编码到 `reason`**：proto `NavigationReadiness` 只暴露
  `ready + reason` 两个字段，没有 sub-items 槽位。本期用 `"{top_reason};
  details={json}"` 编码，Python 侧 `_parse_navigation_readiness_reason`
  解析为 10 字段 dict；P2 阶段如要新增 gRPC 字段可作为 `reason` 临时容器
  的替代。
- **`step_follower` 走 spawn 避免持锁 await**：cmd_vel publish 内部 `.await`
  LCM transport，spawn 出去以避免持 `state.inner.write().await` 锁。
- **`nav_ready_poll` 不 await**：`GeometryRuntimeHandle::nav_ready_poll`
  签名是 `fn` 不是 `async fn`；粗略用 `lcm_backend_available` 兜底。详细
  readiness 判定在 `runtime_diagnostics::navigation_readiness` 内部走
  `compute_readiness`（async）。
- **`build_handle_for_tests` 不订阅 LCM channel**：与 `spawn_geometry_runtime`
  行为一致但只构造状态不启动订阅任务；为 `#[tokio::test]` 单元测试
  提供最小 fixture。

### 2.3 proto / stub 同步

- `geometry_runtime.proto` 无新增字段（`NavigationGoal / NavigationState /
  CancelNavigationRequest / NavigationReadiness` 字段保持 Phase 2 冻结
  形态），所以 Python / Rust stub 不需要再生成（已存在并通过
  `py_compile` 验证）。
- 如果后续需要新增 readiness 子项字段（如 `costmap_present` 单独 bool），
  按 Phase 6 模式同提交 regen 即可。

## 3. 验收实际结果

| 命令 | 退出码 |
|---|---|
| `bash scripts/ci/no_ros2_residual.sh` | 0 |
| `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` | 0 |
| `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` | 0 |
| `rg -n '_nav2_action_provider.*return\|return getattr\(self, "_nav2_action_provider"\|force_nav2_goal\|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py` | 0 命中 |
| `rg -n '_resolve_exploration_backend.*_platform_exploration_runtime\|return getattr\(self, "_platform_exploration_runtime"\|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py` | 0 命中 |
| `rg -n "SetNavigationGoal.*UNIMPLEMENTED\|CancelNavigation.*UNIMPLEMENTED" services/geometry/nuwax_geometry_runtime/src/server.rs` | 0 命中 |
| `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked --all-targets` | 4 errors（全部在 `src/explore_runtime.rs:362-364` — Phase 5 范围；nav_runtime / server.rs 干净） |
| `cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \| rg "nuwax-nav-stack\|nuwax-nav-simple-planner\|nuwax-nav-astar\|nuwax-nav-basic-path-follower"` | 4 命中 |
| `cargo tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \| rg "r2r\|cyclonedds\|rmw_\|libddsc"` | 0 命中 |
| `PYTHONPATH=. python3 -m py_compile services/geometry/proto_gen/geometry_runtime_pb2.py services/geometry/proto_gen/geometry_runtime_pb2_grpc.py` | 0 |

## 4. 阻塞 & 下游风险

1. **cargo test 整体阻塞**：`explore_runtime.rs` 4 个 `&ExplorationStopReason`
   deref 错误（`stop_reason_str` / `Some(reason)` 等位置）使得
   `cargo test --bin nuwax-geometry-runtime` 失败。这是 Phase 5 范围
   （与 `nuwax-exploration` crate API 演化有关），不是本阶段引入的。Phase 5
   收口后整体 `cargo test` 应自动恢复。
2. **simple-planner / astar 未直接调**：本期 follower 走单点 waypoint，
   实际规划由 `basic-path-follower` 用 goal pose 推进 cmd_vel。完整
   costmap-aware 路径规划需 Phase 3 mapping adapter 实装后再接。
3. **实机验证未做**：按任务约定 Phase 4 静态验证 + 单元测试 + cargo check
   即收口，实机由 Phase 7 走。

## 5. Phase 5/6 接力事项

- Phase 5（explore）必须先修 `explore_runtime.rs:362-364` 4 个 deref 错误，
  否则 `cargo test` / `cargo check --all-targets` 整体失败。
- Phase 6 Python / gateway 收敛：把 `get_navigation_readiness` 10 字段 dict
  接到 `gateways/runtime.py` 的 navigation endpoint；保留
  `RuntimeNavigationClient` 串行队列。
- Phase 7 installer / systemd：`start_nuwax_geometry_runtime.sh` 加
  `NUWAX_NAV_PROFILE=simple-2d` 默认；systemd drop-in 不覆盖
  `NUWAX_NAV_PROFILE`（参考 G1 drop-in 教训）。
