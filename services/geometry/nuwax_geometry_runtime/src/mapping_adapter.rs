//! Phase 3: nuwax-mapping 接入 adapter。
//!
//! 目标:把 `nuwax-mapping::PointCloudAccumulator` / `OccupancyGrid` / `CostMapper`
//! 接成本仓 `MapProvider`/occupancy/costmap/cloud_map 的真实数据源。
//!
//! ## 数据流
//! 1. 订阅 `/odom` (odometry) + `/tf` (transform) + `/go2/lidar/points` (point cloud)
//! 2. 用本地 `FeedOnlyAccumulator`(实现 `nuwax_mapping::PointCloudAccumulator`
//!    trait)累积点云
//! 3. 用 `nuwax_mapping::PointCloudOccupancyBuilder` 把累积点云 + 当前 odom pose
//!    生成 `OccupancyGrid`
//! 4. 用 `nuwax_mapping::CostMapper` (nav2_default) 把 occupancy → costmap
//! 5. 把 occupancy → costmap 推入 `SharedState.map` (`MapState` 字段复用)
//!
//! ## readiness 语义
//! - `mapping_ready_poll()` 真实判定:`accumulator_has_data && lcm_backend_available`
//!   之前 Phase 2 baseline 恒 `false`,本次实装后改 true (有数据时)
//!
//! ## 为什么不用 XyzAccumulator
//! - `nuwax_mapping::pointclouds::accumulators::XyzAccumulator::into_points(self)`
//!   是 self 消耗语义,无法在 `&self` 上克隆点云
//! - occupancy builder 需要 `&[Point3<f64>]`,因此需要 read-only view
//! - 本地 `FeedOnlyAccumulator` 同时实现 trait + 提供 `clone_points(&self)`
//!
//! ## LCM typed channel 解析
//! - 优先用 `channel_registry::resolve_channel` 解析 `/odom` `/tf`
//!   `/go2/lidar/points`;不重读 `std::env::var`
//! - 该 adapter 是 `lcm_runtime.rs` 启动时调用的 `spawn_*` 之一,
//!   接收的 channel 字符串由调用方决定,这里只在 missing 时回退到 default

use std::sync::Arc;

use nuwax_mapping::pointclouds::occupancy::PointCloudOccupancyBuilder;
use nuwax_mapping::{
    cost_mapper::CostMapperConfig, CostMapper, OccupancyGrid, PointCloudAccumulator,
};
use nalgebra::Point3;
use tokio::sync::RwLock;
use tracing::warn;

use crate::lcm_runtime::{MapState, SharedState};

// ============================================================================
// 配置
// ============================================================================

/// nuwax-mapping adapter 配置(env 探测 + 合理 default)。
///
/// 关键参数:
/// - `resolution` —— occupancy 栅格分辨率 (m/cell);默认 0.05
/// - `width_cells` / `height_cells` —— 栅格尺寸;默认 200×200 (10m × 10m)
/// - `voxel_size` —— 体素降采样边长;默认 0.05,与 `pointcloud.rs::VOXEL_SIZE` 对齐
/// - `origin_xy` —— 栅格原点 (世界坐标);默认 (-5, -5) 配 10m 边长
#[derive(Debug, Clone, Copy)]
pub struct MappingAdapterConfig {
    pub resolution: f64,
    pub width_cells: u32,
    pub height_cells: u32,
    pub origin_xy: (f64, f64),
    pub voxel_size: f64,
}

impl MappingAdapterConfig {
    /// 从 env 探测,带 default。
    ///
    /// - `NUWAX_MAPPING_RESOLUTION` (默认 0.05)
    /// - `NUWAX_MAPPING_WIDTH_CELLS` (默认 200)
    /// - `NUWAX_MAPPING_HEIGHT_CELLS` (默认 200)
    /// - `NUWAX_MAPPING_VOXEL_SIZE` (默认 0.05)
    /// - `NUWAX_MAPPING_ORIGIN_X` / `NUWAX_MAPPING_ORIGIN_Y` (默认 -5 / -5)
    pub fn from_env() -> Self {
        Self {
            resolution: parse_env_f64("NUWAX_MAPPING_RESOLUTION", 0.05),
            width_cells: parse_env_u32("NUWAX_MAPPING_WIDTH_CELLS", 200),
            height_cells: parse_env_u32("NUWAX_MAPPING_HEIGHT_CELLS", 200),
            origin_xy: (
                parse_env_f64("NUWAX_MAPPING_ORIGIN_X", -5.0),
                parse_env_f64("NUWAX_MAPPING_ORIGIN_Y", -5.0),
            ),
            voxel_size: parse_env_f64("NUWAX_MAPPING_VOXEL_SIZE", 0.05),
        }
    }
}

fn parse_env_f64(key: &str, default: f64) -> f64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse::<f64>().ok())
        .unwrap_or(default)
}

fn parse_env_u32(key: &str, default: u32) -> u32 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.trim().parse::<u32>().ok())
        .unwrap_or(default)
}

// ============================================================================
// FeedOnlyAccumulator —— 本地实现 nuwax_mapping::PointCloudAccumulator trait
// ============================================================================

/// Phase 3 真实使用的 accumulator。
///
/// 上游 `XyzAccumulator` 暴露的是 `into_points(self)`,无法在 `&self` 上克隆,
/// 因此在 adapter 内重新实现一个等价 `FeedOnlyAccumulator`
/// (只允许 feed + clone_points)。
pub struct FeedOnlyAccumulator {
    points: Vec<Point3<f64>>,
}

impl FeedOnlyAccumulator {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }
    /// 返回当前累积的点的克隆(用于 occupancy builder 输入)
    pub fn clone_points(&self) -> Vec<Point3<f64>> {
        self.points.clone()
    }
}

impl Default for FeedOnlyAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl PointCloudAccumulator for FeedOnlyAccumulator {
    fn feed(&mut self, points: &[Point3<f64>]) {
        self.points.extend_from_slice(points);
    }
    fn count(&self) -> usize {
        self.points.len()
    }
}

// ============================================================================
// MappingAdapter
// ============================================================================

/// nuwax-mapping adapter 本体。
///
/// 持有:
/// - `accumulator: FeedOnlyAccumulator` —— 累积点云
/// - `cost_mapper: CostMapper` —— occupancy → costmap
/// - `last_pose: (x, y, z)` —— 上一次 odom pose,作为 occupancy builder sensor_origin
///
/// 公开方法:
/// - `feed_pointcloud(&[Point3<f64>])` —— 喂入点云 (从 LCM /go2/lidar/points 解析)
/// - `feed_pose(x, y, z)` —— 更新 sensor_origin (从 LCM /odom 解析)
/// - `build_and_publish(&SharedState)` —— 生成 occupancy + costmap,写回 SharedState.map
pub struct MappingAdapter {
    config: MappingAdapterConfig,
    /// accumulator 字段需要 `pub(crate)` 让 `lcm_runtime::spawn_mapping_feed_tasks`
    /// 能在同 crate 直接调 `PointCloudAccumulator::feed` 路径;
    /// 不暴露给 `server.rs` / gRPC 层。
    pub(crate) accumulator: FeedOnlyAccumulator,
    cost_mapper: CostMapper,
    pub(crate) last_sensor_origin: [f64; 3],
    /// `has_data` 字段需要 `pub(crate)` 让 `lcm_runtime::spawn_mapping_feed_tasks`
    /// 在 feed 路径同步置 true;`mapping_ready_poll` 通过 `has_data()` 方法访问。
    pub(crate) has_data: bool,
    /// occupancy grid 累计 revision(每次 build_and_publish 自增)
    revision: u64,
}

impl MappingAdapter {
    /// 构造 adapter;配置从 env 探测,`CostMapper` 用 Nav2 默认值。
    pub fn new(config: MappingAdapterConfig) -> Self {
        let cost_mapper = CostMapper::new(CostMapperConfig::nav2_default());
        Self {
            config,
            accumulator: FeedOnlyAccumulator::new(),
            cost_mapper,
            last_sensor_origin: [0.0, 0.0, 0.0],
            has_data: false,
            revision: 0,
        }
    }

    /// 喂入一帧点云;`mapping_adapter` 不持锁跨 await,调用方负责同步。
    pub fn feed_pointcloud(&mut self, points: &[Point3<f64>]) {
        if points.is_empty() {
            return;
        }
        <FeedOnlyAccumulator as PointCloudAccumulator>::feed(&mut self.accumulator, points);
        self.has_data = true;
    }

    /// 更新 sensor_origin (odom pose 平移部分)。
    pub fn feed_pose(&mut self, x: f64, y: f64, z: f64) {
        self.last_sensor_origin = [x, y, z];
    }

    /// 当前是否有数据 (用于 `mapping_ready_poll` 判定)。
    pub fn has_data(&self) -> bool {
        self.has_data
    }

    /// 当前累计点数 (debug / diagnostics 用)。
    pub fn accumulated_count(&self) -> usize {
        <FeedOnlyAccumulator as PointCloudAccumulator>::count(&self.accumulator)
    }

    /// 当前 adapter 持有的 costmap 配置。
    pub fn cost_mapper_config(&self) -> &CostMapperConfig {
        self.cost_mapper.config()
    }

    /// 把当前累积的点云 + sensor_origin 转换为 occupancy grid。
    ///
    /// **不消费 accumulator** —— 内部走 `clone_points` 路径,
    /// 后续 feed 仍能继续累积;若要清空,调用方调用 `reset_accumulator`。
    pub fn build_occupancy(&self) -> OccupancyGrid {
        let points = self.accumulator.clone_points();
        let origin = self.last_sensor_origin;
        let builder = PointCloudOccupancyBuilder::new(
            nalgebra::point![origin[0], origin[1], origin[2]],
            self.config.resolution,
            self.config.width_cells,
            self.config.height_cells,
            self.config.origin_xy,
        );
        builder.build(&points)
    }

    /// 把当前 accumulated 点云 + sensor_origin 走完整链路:
    /// 1. 重建 occupancy
    /// 2. costmap 转换
    /// 3. 写回 `SharedState.map` + revision 自增
    ///
    /// 返回 true 表示数据有效且已写入, false 表示无数据可建图。
    ///
    /// 用 `try_write` 避免在 tokio context 中跨 await;
    /// 若取不到锁就放弃本轮写入(下一轮 feed 会重新触发)。
    pub fn build_and_publish(&mut self, state: &Arc<RwLock<SharedState>>) -> bool {
        if !self.has_data {
            return false;
        }
        let grid = self.build_occupancy();
        let costmap = self.cost_mapper.convert(&grid);
        // Costmap data 长度 = width * height,转换回 i8 vector 喂回 MapState.data
        // (沿用 Phase 0 contract: -1 = unknown / 0 = free / 100 = occupied)
        let data: Vec<i8> = costmap.data.iter().map(|&v| map_cost_to_i8(v)).collect();
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        self.revision += 1;
        if let Ok(mut s) = state.try_write() {
            s.map = MapState {
                width: costmap.width(),
                height: costmap.height(),
                resolution: costmap.metadata.resolution as f32,
                origin_x: costmap.metadata.origin.translation.x,
                origin_y: costmap.metadata.origin.translation.y,
                frame_id: "map".to_string(),
                revision: self.revision,
                timestamp_ns: now_ns,
                data,
            };
            true
        } else {
            warn!("mapping_adapter: SharedState.try_write 失败,本轮 build_and_publish 跳过");
            false
        }
    }

    /// 清空 accumulator(用于重新建图 / 地图 reset)。
    pub fn reset_accumulator(&mut self) {
        self.accumulator = FeedOnlyAccumulator::new();
        self.has_data = false;
    }

    /// adapter 当前 revision(给 diagnostics / 调试用)。
    pub fn current_revision(&self) -> u64 {
        self.revision
    }
}

/// nuwax-mapping CostMapper 输出的 u8 → MapState i8 转换。
///
/// CostMapper 约定:
/// - 0 = free
/// - 254 = occupied (Nav2 lethal)
/// - 253 = inscribed obstacle
/// - 252 = circumscribed inflation
/// - 128 = inflation outer ring
/// - 255 = no information
///
/// MapState.data 沿用 Phase 0 contract(模仿 ROS OccupancyGrid int8):
/// -1 = unknown / 0 = free / 100 = occupied
fn map_cost_to_i8(v: u8) -> i8 {
    if v >= 65 {
        100
    } else if v == 0 {
        0
    } else {
        -1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_mapping::occupancy::types::OccupancyValue;

    fn sample_config() -> MappingAdapterConfig {
        MappingAdapterConfig {
            resolution: 0.1,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-1.0, -1.0),
            voxel_size: 0.1,
        }
    }

    #[test]
    fn adapter_default_state_is_empty() {
        let a = MappingAdapter::new(sample_config());
        assert!(!a.has_data());
        assert_eq!(a.accumulated_count(), 0);
        assert_eq!(a.current_revision(), 0);
    }

    #[test]
    fn feed_pointcloud_marks_has_data() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.5, 0.5, 0.0)]);
        assert!(a.has_data());
        assert_eq!(a.accumulated_count(), 1);
    }

    #[test]
    fn feed_pointcloud_empty_no_mark() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[]);
        assert!(!a.has_data());
    }

    #[test]
    fn feed_pose_updates_origin() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pose(1.0, 2.0, 0.0);
        assert_eq!(a.last_sensor_origin, [1.0, 2.0, 0.0]);
    }

    #[test]
    fn build_occupancy_produces_grid() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.0, 0.0),
        ]);
        let grid = a.build_occupancy();
        assert_eq!(grid.width, 20);
        assert_eq!(grid.height, 20);
        // 至少有一个 occupied cell
        let mut has_occupied = false;
        for r in 0..grid.height {
            for c in 0..grid.width {
                if matches!(grid.get(r, c), Some(OccupancyValue::Occupied)) {
                    has_occupied = true;
                    break;
                }
            }
        }
        assert!(has_occupied, "occupancy grid 至少应有 1 个 occupied cell");
    }

    #[test]
    fn build_and_publish_writes_shared_state() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(a.build_and_publish(&state));
        let s = state.try_read().expect("read");
        assert!(s.map.revision >= 1);
        assert_eq!(s.map.width, 20);
        assert_eq!(s.map.height, 20);
    }

    #[test]
    fn build_and_publish_no_data_returns_false() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut a = MappingAdapter::new(sample_config());
        assert!(!a.build_and_publish(&state));
    }

    #[test]
    fn reset_clears_accumulator() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(a.has_data());
        a.reset_accumulator();
        assert!(!a.has_data());
        assert_eq!(a.accumulated_count(), 0);
    }

    #[test]
    fn map_cost_to_i8_conversion() {
        // 0 → 0 (free)
        assert_eq!(map_cost_to_i8(0), 0);
        // 65+ → 100 (occupied)
        assert_eq!(map_cost_to_i8(65), 100);
        assert_eq!(map_cost_to_i8(100), 100);
        assert_eq!(map_cost_to_i8(254), 100);
        // 1..64 → -1 (no info / inflation rings)
        assert_eq!(map_cost_to_i8(1), -1);
        assert_eq!(map_cost_to_i8(50), -1);
    }

    #[test]
    fn config_from_env_uses_defaults() {
        // 不污染环境;直接调 from_env,应走 default
        let cfg = MappingAdapterConfig::from_env();
        assert!(cfg.resolution > 0.0);
        assert!(cfg.width_cells > 0);
        assert!(cfg.height_cells > 0);
    }

    #[test]
    fn cost_mapper_config_is_nav2_default() {
        let a = MappingAdapter::new(sample_config());
        let c = a.cost_mapper_config();
        // Nav2 默认:0 = free, 254 = lethal, 255 = no info
        assert_eq!(c.free_space_cost, 0);
        assert_eq!(c.lethal_cost, 254);
        assert_eq!(c.no_information_cost, 255);
    }
}
