# Phase 2 Feedback — runtime deps + typed channel registry

> 日期：2026-07-06
> lane：rust-runtime Phase 2
> 关联：`执行方案.md` §3 Phase 2 段 / `20_Rust运行时deps与typed_channel_phase2.md`

## 1. 完成清单

1. ✅ `services/geometry/nuwax_geometry_runtime/Cargo.toml` 增 8 条 path deps（mapping / nav-stack / nav-types-messages / nav-simple-planner / nav-astar / nav-terrain-map-ext / exploration / explore-wavefront）。feature 名以 dimos 实际 Cargo.toml 为准（先 Read 后写）。
2. ✅ 新建 `src/channel_registry.rs`（141 行 + 3 unit test）：统一 `NUWAX_LCM_CHANNEL_*` + `NUWAX_GEOMETRY_RUNTIME_*_CHANNEL` alias,env 探测顺序 = runtime → platform → default
3. ✅ 新建 `src/typed_codec.rs`（252 行 + 4 unit test）：LCM typed encode/decode,镜像对账 `lcm_runtime::parse_*` 字段布局
4. ✅ 新建 `src/runtime_diagnostics.rs`（190 行 + 3 unit test）：统一 health / readiness / blockers 输出,4 类 blocker 关键字(LCM_BACKEND_UNAVAILABLE / MAPPING_NOT_READY / NAV_STACK_NOT_READY / EXPLORATION_NOT_READY)做 CI gate
5. ✅ `src/lcm_runtime.rs` `GeometryRuntimeHandle` 新增 3 个 `*_ready_poll()` 方法(Phase 2 baseline 恒 `false`)
6. ✅ `src/server.rs` 重构:health_check / get_status / get_navigation_readiness / publish_odom / publish_point_cloud 全部走 typed_codec + runtime_diagnostics,禁止内联 bytes
7. ✅ `src/main.rs` 重构:9 类 typed channel env 探测统一走 channel_registry,删除内联 `env_first` 函数
8. ✅ proto / stub 再生成:Python stub 用 `.venv_model_download/bin/python -m grpc_tools.protoc` 重生成,`py_compile` 双 stub 通过;Rust stub 走 `cargo build` 触发 `tonic_build::compile_protos`
9. ✅ `.env.example` + `contracts/geometry_runtime.py` backend 默认调整:`real-lcm` 仍是默认,加 fail-fast 行为说明 + CI 显式覆盖 `fake-lcm` 的注释
10. ✅ 全部 9 项验收命令通过(详见 `20_Rust运行时deps与typed_channel_phase2.md` §5)

## 2. dependency mismatch 报告(无 blocker)

dimos 实际 Cargo.toml 验证全部 8 个 crate 存在 + feature 名匹配执行方案 §3 Phase 2 段描述,**没有 dependency mismatch**。详细：

- `nuwax-mapping/Cargo.toml`:默认 deps 满足,无 native feature 需要
- `nuwax-nav-stack/Cargo.toml:34-137`:`foundation` / `simple-2d` / `replanning-astar` 三个 feature 全部存在(line 38-51);native-* / explore feature 暂不开
- `nuwax-exploration/Cargo.toml:15-27`:`wavefront` / `runtime` feature 全部存在(line 17 / 20)
- `nuwax-nav-simple-planner` / `nuwax-nav-astar` / `nuwax-nav-terrain-map-ext` / `nuwax-nav-types-messages` / `nuwax-explore-wavefront`:默认 deps 满足,无 feature 切换

## 3. 实测关键事实

- cargo check `--all-targets`:0 errors,16 warnings(全部是既有 dead-code 警告,与 Phase 2 改动无关)
- cargo test:10/10 pass(3 channel_registry + 4 typed_codec + 3 runtime_diagnostics)
- cargo tree:`nuwax-mapping` / `nuwax-nav-stack` / `nuwax-exploration` 三条线全部 link 成功
- cargo tree 负向 gate `r2r|cyclonedds|rmw_|libddsc`:无匹配(返回 1 = grep 无 match)
- no_ros2_residual: PASS
- compileall Python: PASS
- bash -n start script: PASS
- rg 两个守门命令:无匹配(返回 1 = grep 无 match)

## 4. proto / stub 状态

本次 Phase 2 未改 proto schema(`geometry_runtime.proto` Phase 0/1 已收敛到目标态)。stub 再生成是 byte-stable 重生成,无功能变化。

Python stub 用 `.venv_model_download/lib/python3.12/site-packages/grpc_tools` 重生成,`py_compile` 双 stub 通过。Rust stub 由 `cargo build` 触发 `build.rs` 内 `tonic_build::compile_protos("proto/geometry_runtime.proto")` 生成。

> 注:本机系统 python3 没有 grpc_tools,但 `.venv_model_download` 有;CI / contract test 必须在该 venv 或等价环境跑。

## 5. 后续 Phase 3/4/5 接入锚点

- Phase 3 `mapping_adapter.rs`:在 `GeometryRuntimeHandle::mapping_ready_poll` 中加真实判定
- Phase 4 `nav_runtime.rs`:在 `nav_ready_poll` 中加 readiness 判定
- Phase 5 `explore_runtime.rs`:在 `explore_ready_poll` 中加 readiness 判定
- P1 切 MurmurHash64A:仅改 `typed_codec::build_typed_frame` 一处
- P1 启 real-lcm:`lcm_runtime::LcmBackendKind::Real` 分支补 `LcmTransport::new_real(url)` 调用,移除 `RealLcmBackendUnavailable` 返回

## 6. 未完成 / 已知 issue

- `server.rs::set_navigation_goal` / `cancel_navigation` 仍返回 `UNIMPLEMENTED`(B7 blocker,Phase 4 解决)
- `server.rs::get_occupancy_grid_shm` 仍 `unimplemented`(`OccupancyGrid SHM sidecar 待 P1 接入`)
- `video_relay.rs` 有 5 条 dead-code warning(Phase 0 遗留,与 Phase 2 无关)
- `lcm_runtime.rs` 内联 `parse_*_payload` 函数仍存在(typed_codec 提供镜像,Phase 3/4/5 实装后再合并)
