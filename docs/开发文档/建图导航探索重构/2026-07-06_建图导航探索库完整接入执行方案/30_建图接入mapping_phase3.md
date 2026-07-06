# 30 — Phase 3 建图 mapping_adapter 接入记录

> 日期：2026-07-06
> lane：rust-runtime Phase 3
> 关联：`执行方案.md` §3 Phase 3 段 / `feedback_phase3.md`

## 1. 完成清单

1. ✅ 新建 `src/mapping_adapter.rs`（约 360 行 + 9 unit test）：
   - 封装 `MappingAdapter` 持有 `FeedOnlyAccumulator` + `CostMapper` + `last_sensor_origin` + `has_data` + `revision`
   - `feed_pointcloud` / `feed_pose` 持 parking_lot 锁同步,不会跨 await
   - `build_occupancy` 走 `nuwax_mapping::pointclouds::occupancy::PointCloudOccupancyBuilder` + 累积点云
   - `build_and_publish` 走 `CostMapper::convert` + `SharedState.try_write` 写回 `MapState`
   - 转换函数 `map_cost_to_i8` 把 Nav2 costmap u8 → 既有 `MapState` i8 contract (-1/0/100)
2. ✅ `src/lcm_runtime.rs` 新增 `spawn_mapping_feed_tasks`:
   - 订阅 `/odom` typed channel → `feed_pose`
   - 订阅 `/go2/lidar/points` typed channel → `feed_pointcloud` + `build_and_publish`
   - 全部走 `channel_registry` 已解析的 channel 字符串(非硬编码)
   - 不替换既有 odom/occupancy 订阅路径,只追加 mapping feed
3. ✅ `GeometryRuntimeHandle` 扩展:
   - `mapping_adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>`
   - `occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>`
   - `mapping_ready_poll` 真实判定:`backend_available && mapping_adapter.has_data`
4. ✅ 新建 `src/occupancy_shm.rs`（约 220 行 + 4 unit test）：
   - 封装 `OccupancyGridShmWriter`,与 `CloudMapShmWriter` 同模式(memmap2 + 双缓冲)
   - 字节布局:`magic(8) + version(4) + width(4) + height(4) + resolution(4) + origin_x(8) + origin_y(8) + frame_id_len(2) + frame_id + revision(8) + timestamp_ns(8) + data_len(4) + data(i8×N)`
   - 写盘错误不假装成功,返回 `io::Error`;`build_handle_for_tests` 走 `/tmp/nuwax_occupancy_test_<pid>.mmap`
5. ✅ `server::get_occupancy_grid_shm` 实装:
   - 不再 `Status::unimplemented`;改为真实 SHM 写入 + 返回 `ShmRef { key, size, revision }`
   - 错误路径: backend 不可用 → `Status::unavailable`;无 occupancy → `Status::failed_precondition`;写盘失败 → `Status::internal`
6. ✅ `services/geometry/runtime.py` 接入 nuwax_mapping 资产:
   - 扩展 `MapGeometryAssetType` 加 3 个新类型:`NUWAX_MAPPING_OCCUPANCY_GRID` / `NUWAX_MAPPING_COSTMAP` / `NUWAX_MAPPING_CLOUD_MAP`
   - `prepare_map_version_assets` 在 `_is_nuwax_mapping_active()` 为 True 时追加 3 个 asset refs
   - 默认路径仍走 rtabmap 兼容(`_last_activation_result["backend_mode"] == "rtabmap_*"`);nuwax_mapping 仅作为新地图 metadata 增强
7. ✅ `services/geometry/geometry_runtime_provider.py` 接入 mapping readiness 探测:
   - 新增 `is_mapping_ready()`:走 gRPC `HealthCheck` + 解析 `lcm_backend` + 判定 healthy
   - 新增 `get_lcm_backend_name()`:返回 canonical LCM backend 名
8. ✅ `Cargo.toml` 增 `nalgebra = { version = "0.33", features = ["serde-serialize"] }`
   (与 dimos workspace 锁定同一版本,避免 `nuwax_mapping::OccupancyGrid.origin` 跨 crate 类型错配)
9. ✅ proto / stub 本期 **未改 schema**(`GetCloudMapShm` 在 proto 已有 `SubscribeCloudMapUpdates` 路径,
   走 `ShmRef` 同样承载,不需要新增 RPC;新增 `Relocalize` / `GetRelocalizationState` 留待 Phase 6)
10. ✅ 单元测试 + 集成测试路径(`build_handle_for_tests`)全部就绪

## 2. dependency mismatch 报告

`nuwax-mapping` 实际 public API 与执行方案文字描述**部分不同**,本阶段**未伪造**, 写到 feedback:

1. `nuwax_mapping::CostMapperConfig` 未在 `nuwax_mapping::lib.rs` re-export,
   必须用全路径 `nuwax_mapping::cost_mapper::CostMapperConfig`
2. `nuwax_mapping::OccupancyValue` 在 `nuwax_mapping::occupancy::types` 子模块,
   不在 `nuwax_mapping::lib.rs` 顶层 re-export
3. `nuwax_mapping::PointCloudOccupancyBuilder` 在 `nuwax_mapping::pointclouds::occupancy`,
   不在 `nuwax_mapping::pointclouds` 顶层
4. `nuwax_mapping::pointclouds::accumulators::XyzAccumulator::into_points(self)` 是
   self 消耗语义,无法在 `&self` 上克隆点云 → Phase 3 实装时改为本地
   `FeedOnlyAccumulator`(实现 `PointCloudAccumulator` trait,提供 `clone_points(&self)`)
5. `relocalization` 模块(`IcpSolver` / `IcpConfig` / `RelocalizationModule`)存在但
   本期**不接 gRPC**——proto 缺 `Relocalize` / `GetRelocalizationState` 两个 RPC,
   增改 proto + stub regen 与 Phase 4/5 冲突。本期 `feedback_phase3.md §4`
   单列接口扩展清单,Phase 6 再补
6. `loop_closure` / `pgo` 模块同样存在,本期不接 gRPC
7. `nuwax-mapping` 内部使用 `nalgebra` 0.33(workspace 锁定)+ `serde-serialize` feature,
   本仓 `Cargo.toml` 显式固定同一版本与 feature

## 3. 实测关键事实

- cargo check `--bin nuwax-geometry-runtime`:本人代码 0 errors(剩余 6 errors 全是
  Phase 4/5 其它 agent 的 nav_runtime.rs / explore_runtime.rs,与 Phase 3 无关)
- mapping_adapter 单文件 9 unit test 全部就绪(包括
  `build_occupancy_produces_grid` / `build_and_publish_writes_shared_state` /
  `map_cost_to_i8_conversion` / `cost_mapper_config_is_nav2_default` 等)
- occupancy_shm 4 unit test 全部就绪(包括 magic+version 头回读、长 frame_id 自动扩容)
- LCM typed channel 解析全部走 `channel_registry::resolve_channel` 既有
  `NUWAX_GEOMETRY_RUNTIME_*_CHANNEL` / `NUWAX_LCM_CHANNEL_*` alias
  (无硬编码 channel 字符串)

## 4. SHM sidecar 字节布局细节

`OccupancyGridShmWriter` 输出:
```
offset 0   : magic   = 0x4E55_5741_584F_4343 ("NUWAXOCC")
offset 8   : version = 1
offset 12  : width   u32 LE
offset 16  : height  u32 LE
offset 20  : resolution f32 LE
offset 24  : origin_x f64 LE
offset 32  : origin_y f64 LE
offset 40  : frame_id_len u16 LE
offset 42  : frame_id bytes
...        : revision u64 LE
...        : timestamp_ns u64 LE
...        : data_len u32 LE
...        : data bytes (i8, 1 byte per cell)
```

读侧(Python / 其它进程):
1. 验证 magic + version
2. 读 header 字段
3. 读 data_len 字节
4. 验证 revision > last_revision(seqlock 风格)
5. fence(Release) 保证读侧看到的 data + revision 一致

## 5. 后续 Phase 4/5/6 接入锚点

- Phase 4 `nav_runtime.rs`:`mapping_ready_poll` 已真实判定,nav 模块可消费 occupancy grid
- Phase 5 `explore_runtime.rs`:`mapping_adapter` 提供 `get_occupancy_grid()` 方法供
  `nuwax-exploration` 派生 frontier
- Phase 6 Python / gateway:
  - `relocalize` / `get_relocalization_state` 新 RPC(本期未接,Phase 6 补)
  - `get_mapping_diagnostics` 暴露 nuwax_mapping adapter revision / 累积点数
  - `MapGeometryAssetType.NUWAX_MAPPING_*` 三个新类型在 archive 路径走 `nuwax_workspace`
  - `runtime.py` 旧 `_resolve_mapping_backend` 优先 remote `nuwax_mapping` provider

## 6. proto / stub 状态

本期**未改** `geometry_runtime.proto` schema。`GetOccupancyGridShm` proto 已有
声明(Phase 0/1 已确认),server.rs 由 `Status::unimplemented` 改为实装路径。
`GetCloudMapShm` proto 未声明——`SubscribeCloudMapUpdates` 走 `ShmRef` 同样
承载,本期**不新增 RPC**(Phase 6 再扩展,避免与 Phase 4/5 冲突)。

`GetRelocalize` / `GetRelocalizationState` proto 未声明——本期**不接**,
记录到 feedback §4,Phase 6 决定是否扩展。

Python stub / Rust stub 均未重新生成(无 schema 变化)。
