# Phase 2 — Rust 运行时 deps + typed channel 收敛

> 日期：2026-07-06
> lane：rust-runtime（Phase 2）
> 关联：`执行方案.md` §3 Phase 2 段 / `00_接触面盘点_phase0.md` / `06_LCM接入与wire-format验证_phase1.md`
> 产物路径：
> - 代码：`services/geometry/nuwax_geometry_runtime/{Cargo.toml, src/{channel_registry,typed_codec,runtime_diagnostics,server,lcm_runtime,main}.rs}`
> - 验证：cargo check / test / tree / no_ros2_residual / rg 守门

---

## 1. Cargo.toml 增 path deps（dimos 实际路径锁定）

`services/geometry/nuwax_geometry_runtime/Cargo.toml` 新增 8 条 path 依赖，feature 名以 dimos 实际 `Cargo.toml` 为准（**先 Read dimos 那边再写本仓**，不直接抄执行方案文字）：

| crate | 实际 dimos 路径 | feature |
|---|---|---|
| `nuwax-mapping` | `crates/nuwax-mapping` | 默认 |
| `nuwax-nav-stack` | `crates/nuwax-nav-stack` | `default-features = false`, `features = ["foundation", "simple-2d", "replanning-astar"]`（P1 再开 `native-*` / `explore`） |
| `nuwax-nav-types-messages` | `crates/nuwax-nav-types-messages` | 默认 |
| `nuwax-nav-simple-planner` | `crates/nuwax-nav-simple-planner` | 默认 |
| `nuwax-nav-astar` | `crates/nuwax-nav-astar` | 默认 |
| `nuwax-nav-terrain-map-ext` | `crates/nuwax-nav-terrain-map-ext` | 默认 |
| `nuwax-exploration` | `crates/nuwax-exploration` | `default-features = false`, `features = ["wavefront", "runtime"]`（P1 再开 `tare,terrain-map-ext,spec-summary`） |
| `nuwax-explore-wavefront` | `crates/nuwax-explore-wavefront` | 默认 |

dimos `nuwax-nav-stack/Cargo.toml` line 34-137 验证 feature 名；`nuwax-exploration/Cargo.toml` line 15-27 验证 feature 名；两者与执行方案 §3 Phase 2 段描述一致。

---

## 2. 新增 / 重构 Rust 模块

### 2.1 `src/channel_registry.rs`（新建，141 行）

统一读取 `NUWAX_LCM_CHANNEL_*`（平台级 Python contract）与 `NUWAX_GEOMETRY_RUNTIME_*_CHANNEL`（本进程 alias）。

- `TYPED_CHANNELS` 常量列出 9 类 typed channel:occupancy / tf / cloud_map / nav_state / frontiers / pose / odometry / cmd_vel / nav_cancel
- `resolve_channel(runtime_alias, platform_alias, default) -> String`:env 探测顺序 = runtime alias → platform alias → default
- `resolve_all() -> Vec<(&'static str, String)>`:批量解析,日志/registry 用
- 3 条 unit test 覆盖 default 兜底 / runtime alias 优先 / platform alias 兜底

### 2.2 `src/typed_codec.rs`（新建，252 行）

封装 LCM typed encode / decode,禁止 `server.rs` / `lcm_runtime.rs` 继续写 ad-hoc bytes。

- `encode_odometry_payload(pose: &PoseState) -> Vec<u8>`：7×f64 + u64 stamp + u16 frame_len + frame
- `encode_pointcloud_payload(points: &[u8], point_step: u32) -> Vec<u8>`：u32 step + raw bytes
- `encode_cmd_vel_payload(linear, angular, stamp_ns) -> Vec<u8>`：6×f32 + u64 stamp
- `encode_nav_cancel_payload(goal_id, stamp_ns) -> Vec<u8>`：u16 goal_id_len + goal_id + u64 stamp
- `encode_pose_stamped_payload`：复用 odom 布局
- `build_typed_frame(topic, msg_type, payload, seqno) -> RawLcmFrame`：fingerprint 取 `LcmMessageType::fingerprint()`（FNV-1a 64-bit,P1 切 MurmurHash64A 时本函数是唯一改点）
- `decode_odometry_payload` / `decode_tf_payload` / `decode_frontiers_payload` / `decode_nav_state_payload`：与 `lcm_runtime::parse_*` 镜像对账
- 4 条 unit test:roundtrip / layout / wire size / fingerprint 非零

### 2.3 `src/runtime_diagnostics.rs`（新建，190 行）

统一 health / readiness / blocker 输出,Phase 4/5 nav_runtime / explore_runtime 直接挂接 mapping / nav / explore readiness 字段。

- `RuntimeDiagnostics`:healthy / phase / uptime_ms / rss_kb / lcm_backend / schema_hash_version / mapping_ready / nav_ready / explore_ready / blockers
- `ComponentReadiness` 板块级 readiness 报告
- `from_handle(handle, phase, started_at_ns)`:从 `GeometryRuntimeHandle` 聚合
- `navigation_readiness` / `mapping_readiness` / `exploration_readiness`:三个板块的 readiness 函数
- `BLOCKER_LCM_BACKEND` / `BLOCKER_MAPPING_NOT_READY` / `BLOCKER_NAV_STACK_NOT_READY` / `BLOCKER_EXPLORATION_NOT_READY` 关键字(grep gate 用)
- 3 条 unit test:rss 读取 / readiness not_ready 字段 / format_blockers no_blockers

### 2.4 `src/lcm_runtime.rs` 修改

`GeometryRuntimeHandle` 新增 3 个方法（Phase 2 baseline 恒 `false`,Phase 3/4/5 实装后改 `true`）：

```rust
impl GeometryRuntimeHandle {
    pub fn mapping_ready_poll(&self) -> bool { false }   // Phase 3 改 true
    pub fn nav_ready_poll(&self) -> bool { false }       // Phase 4 改 true
    pub fn explore_ready_poll(&self) -> bool { false }   // Phase 5 改 true
}
```

### 2.5 `src/server.rs` 重构

- 删除内联 `ServiceState`(迁移到 `runtime_diagnostics` 模块)
- `get_status` / `health_check`:从 `RuntimeDiagnostics::from_handle` 读 healthy / phase / uptime_ms / rss_kb / lcm_backend
- `get_navigation_readiness`:从 `runtime_diagnostics::navigation_readiness` 读 readiness,不再直接读 `nav_state.status`
- `publish_odom` / `publish_point_cloud`:typed payload 编码走 `typed_codec::encode_odometry_payload` / `encode_pointcloud_payload` + `build_typed_frame`,禁止内联 `payload.extend_from_slice(...)`

### 2.6 `src/main.rs` 重构

- 删除内联 `env_first` 函数(迁移到 `channel_registry::resolve_channel`)
- 9 类 typed channel env 探测统一走 `channel_registry::resolve_channel`
- `lcm_runtime::spawn_geometry_runtime` 调用从 `Option<&str>` 改成 `Some(&String)`(直接传 owned String)

---

## 3. Proto / stub 再生成

- Python stub：使用 `.venv_model_download/bin/python -m grpc_tools.protoc` 再生成 `services/geometry/proto_gen/geometry_runtime_pb2.py / _grpc.py`
- Rust stub：cargo build 触发 `build.rs` → `tonic_build::compile_protos("proto/geometry_runtime.proto")`,生成 `target/debug/build/.../nuwax.geometry.rs`
- 验证：`py_compile` 双 stub 通过

> 注：本次 Phase 2 未改 proto schema(原 `geometry_runtime.proto` 已是 Phase 0/1 目标态),stub 实际是 byte-stable 再生成。

---

## 4. .env.example + contracts/geometry_runtime.py 调整

`.env.example` line 255-258 增 backend 警告注释,标明 real-lcm 当前启动期 fail-fast。

`contracts/geometry_runtime.py` `PlatformGeometryConfig` docstring 写明 backend 默认 = real-lcm,Phase 1 fail-fast 行为约束,CI / contract test 显式覆盖 `NUWAX_LCM_BACKEND=fake-lcm`。

---

## 5. 验收命令实际输出

```text
$ bash scripts/ci/no_ros2_residual.sh
[no_ros2_residual] PASS: ROS2 残留守门通过

$ PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts
COMPILEALL_OK

$ bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh
BASH_N_OK

$ ! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py
RG1_RC=1  (无匹配,守门通过)

$ ! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py
RG2_RC=1  (无匹配,守门通过)

$ cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --all-targets
Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.65s
(warning 仅为既有 dead-code,不影响编译)

$ cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked
running 10 tests
test channel_registry::tests::resolve_channel_runtime_wins_over_platform ... ok
test channel_registry::tests::resolve_channel_platform_used_when_runtime_empty ... ok
test channel_registry::tests::resolve_channel_uses_default_when_no_env ... ok
test runtime_diagnostics::tests::component_readiness_not_ready_has_blockers ... ok
test runtime_diagnostics::tests::format_blockers_empty_returns_no_blockers ... ok
test typed_codec::tests::build_typed_frame_wire_size ... ok
test runtime_diagnostics::tests::read_rss_kb_returns_u32 ... ok
test typed_codec::tests::encode_cmd_vel_layout ... ok
test typed_codec::tests::encode_decode_odometry_roundtrip ... ok
test typed_codec::tests::encode_pointcloud_layout ... ok

test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

$ cargo +1.88.0 tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "nuwax-mapping|nuwax-nav-stack|nuwax-exploration"
├── nuwax-exploration v0.1.0 (.../crates/nuwax-exploration)
├── nuwax-mapping v0.1.0 (.../crates/nuwax-mapping)
├── nuwax-nav-stack v0.1.0 (.../crates/nuwax-nav-stack)
(均成功 link)

$ ! cargo +1.88.0 tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"
RC=1  (无匹配,ROS2 依赖零)
```

---

## 6. 后续 Phase 3/4/5 接入锚点

- Phase 3 `mapping_adapter.rs`:在 `GeometryRuntimeHandle::mapping_ready_poll` 中加真实判定(occupancy revision 递增 / cloud_map revision 递增等)
- Phase 4 `nav_runtime.rs`:在 `nav_ready_poll` 中加 readiness 判定(nuwax-nav-stack module graph 已构建 + costmap/cmd_vel sink 就绪)
- Phase 5 `explore_runtime.rs`:在 `explore_ready_poll` 中加 readiness 判定(ExplorationController 已初始化 + frontier source 可用)
- P1 切 MurmurHash64A:仅改 `typed_codec::build_typed_frame` 的 fingerprint 来源,从 `msg_type.fingerprint()` 切到 `nuwax_codec::compute_hash(msg_type_fqn)`
- P1 启 real-lcm:`lcm_runtime::LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)` 调用,移除 `RealLcmBackendUnavailable` 返回
