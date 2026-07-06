# Phase 3 todolist — mapping_adapter (建图)

> 日期：2026-07-06
> lane：rust-runtime Phase 3
> 关联：`执行方案.md` §3 Phase 3 段 / `30_建图接入mapping_phase3.md` / `feedback_phase3.md`

## 任务清单

- [x] P3.1.1 — `src/mapping_adapter.rs` 新建 + 9 unit test
  - [x] `MappingAdapter` struct
  - [x] `FeedOnlyAccumulator` (本地实现 `nuwax_mapping::PointCloudAccumulator` trait)
  - [x] `MappingAdapterConfig` + `from_env`
  - [x] `feed_pointcloud` / `feed_pose` / `build_occupancy` / `build_and_publish` / `reset_accumulator`
  - [x] `map_cost_to_i8` u8 → i8 conversion
- [x] P3.1.2 — `lcm_runtime.rs` 集成
  - [x] `GeometryRuntimeHandle` 加 `mapping_adapter: Option<...>` 字段
  - [x] `GeometryRuntimeHandle` 加 `occupancy_shm: Option<...>` 字段
  - [x] `mapping_ready_poll` 真实判定
  - [x] `spawn_mapping_feed_tasks` 订阅 /odom + /go2/lidar/points
  - [x] `build_handle_for_tests` 配套构造
- [x] P3.1.3 — `Cargo.toml` 加 `nalgebra = { version = "0.33", features = ["serde-serialize"] }`
- [x] P3.1.4 — `main.rs` 加 `mod mapping_adapter;` / `mod occupancy_shm;`
- [x] P3.2.1 — `src/occupancy_shm.rs` 新建 + 4 unit test
  - [x] 字节布局(magic + version + meta + frame_id + revision + data)
  - [x] 扩容机制(默认 16MB,自动 grow)
  - [x] `last_revision` tracking
- [x] P3.2.2 — `server::get_occupancy_grid_shm` 实装
  - [x] 走 SHM 写入 + 返回 `ShmRef`
  - [x] 错误路径(unavailable / failed_precondition / internal)
- [x] P3.3.1 — `contracts/map_workspace.py` 扩展 `MapGeometryAssetType` 加 3 个新值
  - [x] `NUWAX_MAPPING_OCCUPANCY_GRID`
  - [x] `NUWAX_MAPPING_COSTMAP`
  - [x] `NUWAX_MAPPING_CLOUD_MAP`
- [x] P3.3.2 — `services/geometry/runtime.py` 接入
  - [x] `_is_nuwax_mapping_active()` helper
  - [x] `_mapping_lcm_backend_name()` helper
  - [x] `prepare_map_version_assets` 追加 nuwax_mapping_* asset refs
- [x] P3.3.3 — `services/geometry/geometry_runtime_provider.py` 接入
  - [x] `is_mapping_ready()` 走 gRPC HealthCheck
  - [x] `get_lcm_backend_name()` 返回 canonical backend
- [x] P3.4.1 — `relocalization` API 评估
  - [x] `nuwax_mapping::relocalization::IcpSolver` / `IcpConfig` / `RelocalizationModule` 存在
  - [x] 本期不接 gRPC(避免与 Phase 4/5 冲突)
  - [x] 写入 feedback §4 接口扩展清单
- [x] P3.5.1 — 文档落盘
  - [x] `30_建图接入mapping_phase3.md`
  - [x] `feedback_phase3.md`
  - [x] `todolist_phase3.md` (本文件)
- [x] P3.5.2 — 验收命令实际跑通
  - [x] no_ros2_residual
  - [x] Python compileall
  - [x] cargo check (本人代码 0 errors;剩余 6 errors 是 nav_runtime / explore_runtime 其它 agent)
  - [x] rg 两个守门
- [x] P3.5.3 — 3-5 commit 批次落盘
  - [x] commit 1: mapping_adapter.rs + lcm_runtime 集成 + Cargo.toml nalgebra
  - [x] commit 2: occupancy_shm.rs + server.rs get_occupancy_grid_shm 实装
  - [x] commit 3: contracts/map_workspace.py + runtime.py + provider.py nuwax_mapping 资产
  - [x] commit 4: 文档 + 验收命令输出

## 未完成 blocker

- `GetRelocalize` / `GetRelocalizationState` gRPC:本期**未接**,等 Phase 6 决定
- `GetCloudMapShm` 独立 RPC:proto 未声明,Phase 6 再补
- 实机 E2E:Go2 / Jetson 实机 SSH 在 dimos 测试环境,本期 NOT RUN(由 Phase 7 收口)
- `cargo tree` 输出:仍由 Phase 4/5 修复后才有完整输出(本人 deps 路径 OK)
