# nuwax-nav-terrain-map-ext

纯 Rust 扩展地形图处理器（Phase 3 迁移自 Python `terrain_map_ext.py`）。

## 模块

| 模块 | 职责 |
|---|---|
| `voxel_grid` | `_voxel_index` 纯函数（复刻 C++/Python）+ `RollingTerrainGrid` 滚动 |
| `crop` | 高度裁剪 + `distance_ratio_z` 缩放 + `max_range` 限制 |
| `decay` | voxel 下采样 + 时间衰减 + clearing 过滤 |
| `ground` | planar voxel ground estimation + BFS connectivity |
| `connectivity` | 连通性 helper |
| `merge` | 输出构造 + local terrain 半径合并 |
| `config` | 20 字段配置 |
| `module` | `feature = "module"` 启用的 stream/RPC stub |

## 公开 API（核心）

```rust
pub const TERRAIN_VOXEL_WIDTH: i64 = 41;
pub const PLANAR_VOXEL_WIDTH: i64 = 101;

pub fn voxel_index(x: f64, y_ref: f64, voxel_size: f64, half_width: i64) -> i64;

pub struct TerrainMapExtProcessor { /* ... */ }
impl TerrainMapExtProcessor {
    pub fn new(config: TerrainMapExtConfig) -> Self;
    pub fn update_odometry(&mut self, pose: Pose3);
    pub fn update_local_terrain(&mut self, cloud: &[Point3]);
    pub fn request_clearing(&mut self);
    pub fn ingest_registered_scan(&mut self, cloud: TimedPointCloud<'_>) -> Result<(), NavError>;
    pub fn process_tick(&mut self, now: Timestamp) -> Result<Option<PointCloud>, NavError>;
}

pub struct TimedPointCloud<'a> {
    pub points: &'a [Point3],
    pub times: &'a [f64],
}
```

## 测试

```bash
cargo test -p nuwax-nav-terrain-map-ext
```

测试覆盖：
- `tests/voxel_index_tests.rs`：origin、负 offset 修正、边界、y_ref offset、不同 voxel_size、负大半宽
- `tests/crop_tests.rs`：in-bounds 保留、超 scaled upper 拒绝、超 scaled lower 拒绝、distance_ratio 扩张窗口、`crop_scan` 返回 per-point mask
- `tests/decay_tests.rs`：过期删除、`no_decay_distance` 保留近点、`clearing_distance` 在 clearing=true 时清除
- `tests/voxel_grid_tests.rs`：roll 触发 shift、count 不迁移、新边界 voxel 清空
- `tests/ground_tests.rs`：min 默认、quantile=0 取 min、空 cloud 时中心仍被 BFS 标记
- `tests/connectivity_tests.rs`：is_connected helper、邻域连通、ceiling 排斥
- `tests/merge_tests.rs`：radius=0 远场输出、`merge=false` 忽略 local、`merge=true` 包含 local、local 在 radius 外不输出
- `tests/processor_tests.rs`：端到端 ingest → process_tick、无 odom 拒绝、长度不一致拒绝、`system_init_time` 记录

## 关键语义（与 Python 一致）

1. **`_voxel_index` 复刻**：负坐标 offset 触发再减 1；offset 边界为 `-voxel_size/2` 正好不修正。
2. **rolling grid**：窗口内按方向平移复用；新边界 voxel 清空；`update_num / update_time` 不随点云迁移。
3. **decay**：`no_decay_distance` 内点不删；`clearing_distance` 仅在 `clearing=true` 时清。
4. **connectivity BFS**：中心无采样点时 elevation 设为 `vehicle_z + terrain_under_vehicle`；BFS 范围 21x21 邻域。
5. **merge**：`merge_local_terrain=false` 仅发布 `local_terrain_map_radius` 外点。

## Foundation 依赖

- `nuwax-nav-types-messages`：替换本 crate 自带的 `Point3/Pose3/Timestamp/PointCloud/Odometry`。
- `nuwax-nav-base`：将提供 `NavError` 分层与共享 trait。

## 平台约束

- 无 Open3D / NumPy / SciPy 依赖。
- `edition = "2021"`，MSRV 1.88。
- 默认 features 为空；启用 `module` 才编译 `module.rs`。
- rolling grid 内存固定 `TERRAIN_VOXEL_NUM = 41*41 = 1681` cell；每个 cell 是 `Vec<VoxelPoint>`，长时间运行不无限增长（rolling 复用 + 边界清空 + 下采样收敛）。