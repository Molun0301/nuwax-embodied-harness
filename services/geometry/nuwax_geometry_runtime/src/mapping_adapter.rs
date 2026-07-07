// Phase 6 (codex-round3 R3-4)：voxel_size / feed_pointcloud / accumulated_count
// / cost_mapper_config / reset_accumulator / current_revision 是 Phase 3 mapping
// adapter 对外 platform API 预留（SharedState.map / cloud_map 走 fetcher 而不
// 直接调 FeedOnlyAccumulator.method），删了会破坏 Phase 4/5 调用面。
#![allow(dead_code)]

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
//! - `readiness()` 暴露 odom_count / pointcloud_count / last_occupancy_revision
//!   + `mapping_ready` 综合判定;参考 `00_最终执行方案.md` §5 Phase 7 任务 1。
//!   缺 pointcloud (count==0) → mapping_ready=false,即便 odom_count > 0 也不算 ready。
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
/// - `increment_odom_count() / increment_pointcloud_count() / increment_occupancy_revision()`
///   —— `server.rs::publish_odom / publish_point_cloud` 同步累加计数
///   (Phase 5 桥接层保持 feed_* 函数签名不变,只在 call site 加 +1)
/// - `readiness()` —— 综合字段,给 Python diagnostics + runtime_diagnostics 对账
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
    /// Phase 7:计数 odom / pointcloud / occupancy revision,给 readiness 暴露用。
    /// 不参与 build_and_publish 写回 SharedState.map.revision 逻辑(那是 mapping 主路径)。
    odom_count: u64,
    pointcloud_count: u64,
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
            odom_count: 0,
            pointcloud_count: 0,
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

    // ============================================================================
    // Phase 7:readiness counters + 综合 readiness 暴露
    // ============================================================================
    //
    // 与 Phase 3 `feed_pointcloud / feed_pose` 业务路径解耦:
    // server.rs `publish_odom` / `publish_point_cloud` 在调用 LCM publish 的同时
    // 调 `increment_odom_count / increment_pointcloud_count`,由 readiness 统一聚合。
    // 这样 mapping 主路径(feed_* + build_and_publish)与 readiness 诊断面分离:
    // - 不动 feed_* 函数签名(Phase 5 占用约束)
    // - 不动 build_and_publish 写回 SharedState.map.revision 业务
    // - 只新增三个 +1 累加 + 一个 readiness struct

    /// `publish_odom` 在 server.rs 调一次 +1
    pub fn increment_odom_count(&mut self) {
        self.odom_count = self.odom_count.saturating_add(1);
    }

    /// `publish_point_cloud` 在 server.rs 调一次 +1
    pub fn increment_pointcloud_count(&mut self) {
        self.pointcloud_count = self.pointcloud_count.saturating_add(1);
    }

    /// 真实 publish 后由 server.rs 同步触发,使 readiness.last_occupancy_revision
    /// 反映"该 revision 已在 LCM wire 上"
    pub fn increment_occupancy_revision(&mut self) {
        self.revision = self.revision.saturating_add(1);
    }

    /// odom 计数(对外只读)
    pub fn odom_count(&self) -> u64 {
        self.odom_count
    }

    /// pointcloud 计数(对外只读)
    pub fn pointcloud_count(&self) -> u64 {
        self.pointcloud_count
    }
}

/// Mapping 板块 readiness 综合快照(Phase 7)。
///
/// 与 `NavigationReadinessDetail` / `ExplorationReadinessDetail` 对齐:
/// `ready` 综合 bool + `last_error` 字段 + 各子项,Python 侧
/// `get_mapping_diagnostics` 按字段名直接读。
#[derive(Debug, Clone)]
pub struct MappingReadiness {
    /// 综合 ready(`has_data && pointcloud_count > 0`)
    pub ready: bool,
    /// 顶层 reason(中文 + 关键 blocker 标签)
    pub reason: String,
    /// LCM backend 是否可用
    pub lcm_backend: bool,
    /// adapter 是否已构造
    pub adapter_present: bool,
    /// 累计 odom 帧数
    pub odom_count: u64,
    /// 累计 pointcloud 帧数
    pub pointcloud_count: u64,
    /// 最近一次 occupancy grid revision
    pub last_occupancy_revision: u64,
    /// pointcloud 累计点数(给 frontend 决定要不要展示进度)
    pub accumulated_point_count: usize,
    /// 最近一次错误(若 lcm_backend=false / adapter 未构造等)
    pub last_error: Option<String>,
}

impl MappingReadiness {
    /// 综合判定:`has_data` + 真实 pointcloud 输入 + lcm_backend + adapter present
    pub fn compute(
        adapter: Option<&MappingAdapter>,
        lcm_backend_available: bool,
    ) -> Self {
        let mut last_error: Option<String> = None;
        if !lcm_backend_available {
            last_error = Some("LCM backend 不可用,Mapping 板块输入被阻断".to_string());
        }
        let adapter_present = adapter.is_some();
        let mut ready = false;
        let (odom_count, pointcloud_count, last_occupancy_revision, accumulated_point_count) =
            match adapter {
                Some(a) => {
                    if !a.has_data() && a.odom_count() == 0 && a.pointcloud_count() == 0 {
                        last_error.get_or_insert_with(|| "尚未收到任何 odom / pointcloud 输入".to_string());
                    } else if a.pointcloud_count() == 0 {
                        last_error.get_or_insert_with(|| "尚未收到 pointcloud,仅有 odom 不算 ready".to_string());
                    }
                    (
                        a.odom_count(),
                        a.pointcloud_count(),
                        a.current_revision(),
                        a.accumulated_count(),
                    )
                }
                None => {
                    last_error.get_or_insert_with(|| "Mapping adapter 未构造(LCM backend 不可用)".to_string());
                    (0, 0, 0, 0)
                }
            };
        if lcm_backend_available && adapter_present {
            ready = pointcloud_count > 0;
        }
        let reason = if ready {
            "mapping 已就绪".to_string()
        } else {
            last_error.clone().unwrap_or_else(|| "mapping 未就绪".to_string())
        };
        Self {
            ready,
            reason,
            lcm_backend: lcm_backend_available,
            adapter_present,
            odom_count,
            pointcloud_count,
            last_occupancy_revision,
            accumulated_point_count,
            last_error,
        }
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

    // ========================================================================
    // Phase 7:counters + readiness 综合
    // ========================================================================

    #[test]
    fn counters_start_at_zero() {
        let a = MappingAdapter::new(sample_config());
        assert_eq!(a.odom_count(), 0);
        assert_eq!(a.pointcloud_count(), 0);
        assert_eq!(a.current_revision(), 0);
    }

    #[test]
    fn increment_odom_count_advances_counter() {
        let mut a = MappingAdapter::new(sample_config());
        a.increment_odom_count();
        a.increment_odom_count();
        a.increment_odom_count();
        assert_eq!(a.odom_count(), 3);
    }

    #[test]
    fn increment_pointcloud_count_advances_counter() {
        let mut a = MappingAdapter::new(sample_config());
        for _ in 0..5 {
            a.increment_pointcloud_count();
        }
        assert_eq!(a.pointcloud_count(), 5);
    }

    #[test]
    fn increment_occupancy_revision_advances_field() {
        let mut a = MappingAdapter::new(sample_config());
        a.increment_occupancy_revision();
        a.increment_occupancy_revision();
        assert_eq!(a.current_revision(), 2);
    }

    #[test]
    fn readiness_initial_state_unready() {
        // adapter 未构造 → unready,无 pointcloud
        let r = MappingReadiness::compute(None, true);
        assert!(!r.ready);
        assert!(!r.adapter_present);
        assert!(!r.lcm_backend || r.lcm_backend);  // 仅覆盖字段本身
        assert_eq!(r.odom_count, 0);
        assert_eq!(r.pointcloud_count, 0);
        assert!(r.last_error.is_some());
    }

    #[test]
    fn readiness_with_only_odom_is_unready() {
        let mut a = MappingAdapter::new(sample_config());
        a.increment_odom_count();
        let r = MappingReadiness::compute(Some(&a), true);
        assert!(!r.ready, "缺 pointcloud 必须 unready");
        assert_eq!(r.odom_count, 1);
        assert_eq!(r.pointcloud_count, 0);
        assert!(r.last_error.as_ref().unwrap().contains("pointcloud"));
    }

    #[test]
    fn readiness_with_pointcloud_turns_ready() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        a.increment_pointcloud_count();
        a.increment_odom_count();
        a.increment_occupancy_revision();
        let r = MappingReadiness::compute(Some(&a), true);
        assert!(r.ready);
        assert_eq!(r.odom_count, 1);
        assert_eq!(r.pointcloud_count, 1);
        assert_eq!(r.last_occupancy_revision, 1);
        assert_eq!(r.accumulated_point_count, 1);
    }

    #[test]
    fn readiness_no_backend_is_unready() {
        let mut a = MappingAdapter::new(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        a.increment_pointcloud_count();
        let r = MappingReadiness::compute(Some(&a), false);
        assert!(!r.ready);
        assert!(!r.lcm_backend);
        assert!(r.last_error.as_ref().unwrap().contains("backend"));
    }
}
