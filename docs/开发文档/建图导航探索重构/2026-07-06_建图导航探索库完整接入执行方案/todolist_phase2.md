# Phase 2 — todolist（已完成状态）

> 日期：2026-07-06
> lane:rust-runtime Phase 2
> 关联:`执行方案.md` §3 Phase 2 段 / `feedback_phase2.md` / `20_Rust运行时deps与typed_channel_phase2.md`

## 1. Cargo.toml 增 path deps(commit 1)

- [x] Read `nuwax-nav-stack/Cargo.toml` 确认 feature 名
- [x] Read `nuwax-exploration/Cargo.toml` 确认 feature 名
- [x] 增 8 条 path dep:nuwax-mapping / nuwax-nav-stack(foundation,simple-2d,replanning-astar) / nuwax-nav-types-messages / nuwax-nav-simple-planner / nuwax-nav-astar / nuwax-nav-terrain-map-ext / nuwax-exploration(wavefront,runtime) / nuwax-explore-wavefront
- [x] 验证 `cargo check` 通过

## 2. channel_registry.rs(commit 2)

- [x] TYPED_CHANNELS 常量(9 类 typed channel)
- [x] resolve_channel(runtime_alias, platform_alias, default) -> String
- [x] resolve_all() -> Vec<(&'static str, String)>
- [x] 3 条 unit test(default / runtime_wins / platform_used)
- [x] main.rs 9 处 env 探测迁移

## 3. typed_codec.rs(commit 3)

- [x] encode_odometry_payload / encode_pointcloud_payload / encode_cmd_vel_payload / encode_nav_cancel_payload / encode_pose_stamped_payload
- [x] build_typed_frame(topic, msg_type, payload, seqno) -> RawLcmFrame
- [x] decode_odometry_payload / decode_tf_payload / decode_frontiers_payload / decode_nav_state_payload
- [x] 4 条 unit test(roundtrip / layout / wire_size / fingerprint)
- [x] server.rs::publish_odom 迁移
- [x] server.rs::publish_point_cloud 迁移
- [x] 删除 server.rs 内联 ad-hoc bytes

## 4. runtime_diagnostics.rs(commit 4)

- [x] RuntimeDiagnostics struct(healthy / phase / uptime_ms / rss_kb / lcm_backend / schema_hash_version / mapping_ready / nav_ready / explore_ready / blockers)
- [x] ComponentReadiness 板块级 readiness 报告
- [x] from_handle / navigation_readiness / mapping_readiness / exploration_readiness
- [x] 4 个 BLOCKER 常量(LCM_BACKEND_UNAVAILABLE / MAPPING_NOT_READY / NAV_STACK_NOT_READY / EXPLORATION_NOT_READY)
- [x] 3 条 unit test(rss / not_ready / format_blockers)
- [x] server.rs::get_status 迁移
- [x] server.rs::health_check 迁移
- [x] server.rs::get_navigation_readiness 迁移

## 5. lcm_runtime.rs 扩展

- [x] GeometryRuntimeHandle::mapping_ready_poll() / nav_ready_poll() / explore_ready_poll()
- [x] Phase 2 baseline 恒 `false`(Phase 3/4/5 实装后改 `true`)

## 6. server.rs 重构

- [x] 删除内联 ServiceState(迁移到 runtime_diagnostics 模块)
- [x] get_status / health_check / get_navigation_readiness 走 diagnostics
- [x] publish_odom / publish_point_cloud 走 typed_codec
- [x] 编译通过(0 errors)

## 7. main.rs 重构

- [x] 删除内联 env_first 函数
- [x] 9 类 typed channel env 探测走 channel_registry::resolve_channel
- [x] spawn_geometry_runtime 调用从 Option<&str> 改 Some(&String)

## 8. proto / stub 再生成(commit 5)

- [x] Python stub 用 .venv_model_download/bin/python -m grpc_tools.protoc 重生成
- [x] py_compile 双 stub 通过
- [x] Rust stub 由 cargo build 触发 build.rs / tonic_build
- [x] 本次未改 proto schema(stub 是 byte-stable 重生成)

## 9. .env.example + contracts 调整(commit 5)

- [x] .env.example line 255-258 增 backend fail-fast 警告注释
- [x] contracts/geometry_runtime.py PlatformGeometryConfig docstring 写明 backend 默认 = real-lcm + CI 显式覆盖

## 10. 验收命令(实际执行)

- [x] `bash scripts/ci/no_ros2_residual.sh` → PASS
- [x] `PYTHONPATH=. python3 -m compileall -q services core adapters providers gateways skills contracts` → OK
- [x] `bash -n services/geometry/scripts/start_nuwax_geometry_runtime.sh` → OK
- [x] `! rg -n '_nav2_action_provider.*return|return getattr\(self, "_nav2_action_provider"|force_nav2_goal|debug_nav2_goal' services/geometry/runtime.py gateways/runtime.py` → RC=1
- [x] `! rg -n '_resolve_exploration_backend.*_platform_exploration_runtime|return getattr\(self, "_platform_exploration_runtime"|NUWAX_GEOMETRY_RUNTIME_ARGS' services/geometry/runtime.py core/settings.py core/config_registry.py gateways/runtime.py` → RC=1
- [x] `cargo +1.88.0 check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --all-targets` → 0 errors
- [x] `cargo +1.88.0 test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --locked` → 10/10 pass
- [x] `cargo +1.88.0 tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "nuwax-mapping|nuwax-nav-stack|nuwax-exploration"` → 3 条都 link
- [x] `! cargo +1.88.0 tree --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml | rg "r2r|cyclonedds|rmw_|libddsc"` → RC=1

## 11. 文档落盘

- [x] `20_Rust运行时deps与typed_channel_phase2.md`(详细改写记录)
- [x] `feedback_phase2.md`(执行结果汇总)
- [x] `todolist_phase2.md`(本文档)

## 12. 后续 Phase 3/4/5 未完成项(移交)

- [ ] Phase 3 mapping_adapter.rs:`mapping_ready_poll` 加真实判定
- [ ] Phase 4 nav_runtime.rs:`nav_ready_poll` 加 readiness 判定 + SetNavigationGoal / CancelNavigation 实装
- [ ] Phase 5 explore_runtime.rs:`explore_ready_poll` 加 readiness 判定 + StartExploration / StopExploration 实装
- [ ] P1 切 MurmurHash64A:`typed_codec::build_typed_frame` fingerprint 来源切换
- [ ] P1 启 real-lcm:`lcm_runtime::LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)`
- [ ] P1 `GetOccupancyGridShm` 实装:`server.rs::get_occupancy_grid_shm` 解锁
