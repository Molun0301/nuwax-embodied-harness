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
//! 2. 用 `MappingSession` 的有界滚动窗口累积点云
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
//!     缺 pointcloud (count==0) → mapping_ready=false,即便 odom_count > 0 也不算 ready。
//!
//! ## LCM typed channel 解析
//! - 优先用 `channel_registry::resolve_channel` 解析 `/odom` `/tf`
//!   `/go2/lidar/points`;不重读 `std::env::var`
//! - 该 adapter 是 `lcm_runtime.rs` 启动时调用的 `spawn_*` 之一,
//!   接收的 channel 字符串由调用方决定,这里只在 missing 时回退到 default

use std::sync::Arc;

use nalgebra::Point3;
use nuwax_mapping::cost_mapper::{
    CostAlgoSelector, CostMapperConfig, OccupancyConfig, PointCloud2Like, SimplePointCloud2,
};
use nuwax_mapping::pointclouds::occupancy::{MappingError, PointCloudOccConfig};
use nuwax_mapping::{CostMapper, MappingSession, OccupancyGrid, PointCloudAccumulator};
use thiserror::Error;
use tokio::sync::{broadcast, RwLock};
use tracing::warn;

#[allow(unused_imports)]
use nuwax_mapping::SessionStats as MappingSessionStats;

use crate::lcm_runtime::{MapRevisionEvent, MapState, SharedState};
use crate::sequence_state::SequenceStateStore;

const MIN_REBUILD_INTERVAL_NS: u64 = 250_000_000;
const MAX_REBUILD_INTERVAL_NS: u64 = 1_000_000_000;
const REBUILD_GROWTH_DIVISOR: usize = 20;

/// 栅格输出坐标系契约。未执行真实 transform 时，输出必须保留源 frame。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MapFrameContract {
    pub source_frame_id: String,
    pub output_frame_id: String,
    pub expected_map_frame_id: String,
    pub transform_required: bool,
    pub transform_applied: bool,
}

impl MapFrameContract {
    pub fn ready_for_global_consumers(&self) -> bool {
        (!self.transform_required || self.transform_applied)
            && self.output_frame_id == self.expected_map_frame_id
    }
}

/// 只读平面位姿快照。位置使用目标 frame，yaw 绕 +Z，单位为弧度。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanarPoseSnapshot {
    pub position: [f64; 3],
    pub yaw_radians: f64,
}

/// mapping session 固定的 `map <- odom` 刚体变换与最新 map 位姿。
///
/// 首个有效 odom pose `T_odom_base(0)` 建立
/// `T_map_odom = inverse(T_odom_base(0))`，因此首帧 base 位于 map 原点。
/// 该变换在 reset 前保持不变；`generation` 在每次重新锚定时单调递增。
#[derive(Debug, Clone, PartialEq)]
pub struct MapOdomTransformSnapshot {
    pub map_frame_id: String,
    pub odom_frame_id: String,
    pub translation: [f64; 3],
    pub yaw_radians: f64,
    pub anchor_odom_pose: PlanarPoseSnapshot,
    pub latest_map_pose: PlanarPoseSnapshot,
    pub generation: u64,
}

impl MapOdomTransformSnapshot {
    fn from_anchor(
        map_frame_id: &str,
        odom_frame_id: &str,
        anchor_odom_pose: PlanarPoseSnapshot,
        generation: u64,
    ) -> Self {
        let yaw_radians = normalize_yaw(-anchor_odom_pose.yaw_radians);
        let (sin_yaw, cos_yaw) = yaw_radians.sin_cos();
        let [x, y, z] = anchor_odom_pose.position;
        let translation = [
            -(cos_yaw * x - sin_yaw * y),
            -(sin_yaw * x + cos_yaw * y),
            -z,
        ];
        Self {
            map_frame_id: map_frame_id.to_string(),
            odom_frame_id: odom_frame_id.to_string(),
            translation,
            yaw_radians,
            anchor_odom_pose,
            latest_map_pose: PlanarPoseSnapshot {
                position: [0.0, 0.0, 0.0],
                yaw_radians: 0.0,
            },
            generation,
        }
    }

    fn transform_position(&self, position: [f64; 3]) -> [f64; 3] {
        let (sin_yaw, cos_yaw) = self.yaw_radians.sin_cos();
        [
            cos_yaw * position[0] - sin_yaw * position[1] + self.translation[0],
            sin_yaw * position[0] + cos_yaw * position[1] + self.translation[1],
            position[2] + self.translation[2],
        ]
    }

    fn transform_pose(&self, pose: PlanarPoseSnapshot) -> PlanarPoseSnapshot {
        PlanarPoseSnapshot {
            position: self.transform_position(pose.position),
            yaw_radians: normalize_yaw(self.yaw_radians + pose.yaw_radians),
        }
    }
}

fn normalize_frame_id(frame_id: &str) -> String {
    frame_id.trim().trim_start_matches('/').to_string()
}

fn normalize_yaw(yaw_radians: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    (yaw_radians + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI
}

struct BorrowedPointCloud<'a> {
    points: &'a [Point3<f64>],
    sensor_origin: Point3<f64>,
    frame_id: &'a str,
    timestamp: f64,
}

impl PointCloud2Like for BorrowedPointCloud<'_> {
    fn points(&self) -> &[Point3<f64>] {
        self.points
    }

    fn sensor_origin(&self) -> Point3<f64> {
        self.sensor_origin
    }

    fn frame_id(&self) -> &str {
        self.frame_id
    }

    fn timestamp(&self) -> f64 {
        self.timestamp
    }
}

// ============================================================================
// 配置
// ============================================================================

/// nuwax-mapping adapter 配置(env 探测 + 合理 default)。
///
/// 关键参数:
/// - `resolution` —— occupancy 栅格分辨率 (m/cell);默认 0.05
/// - `width_cells` / `height_cells` —— 栅格尺寸;默认 200×200 (10m × 10m)
/// - `voxel_size` —— 体素降采样边长;默认 0.05,与 `pointcloud.rs::VOXEL_SIZE` 对齐
/// - `origin_xy` —— 旧 fixed-bounds 兼容配置;production 使用点云四周 1 m auto bounds
/// - `occupancy_algo` —— 原版 HeightCost / General / Simple kernel selector
#[derive(Debug, Clone, Copy)]
pub struct MappingAdapterConfig {
    pub resolution: f64,
    pub width_cells: u32,
    pub height_cells: u32,
    pub origin_xy: (f64, f64),
    pub voxel_size: f64,
    pub occupancy_algo: CostAlgoSelector,
    pub initial_safe_radius_meters: f32,
}

impl MappingAdapterConfig {
    /// 从 env 探测,带 default。
    ///
    /// - `NUWAX_MAPPING_RESOLUTION` (默认 0.05)
    /// - `NUWAX_MAPPING_WIDTH_CELLS` (默认 200)
    /// - `NUWAX_MAPPING_HEIGHT_CELLS` (默认 200)
    /// - `NUWAX_MAPPING_VOXEL_SIZE` (默认 0.05)
    /// - `NUWAX_MAPPING_ORIGIN_X` / `NUWAX_MAPPING_ORIGIN_Y` (默认 -5 / -5)
    /// - `NUWAX_MAPPING_OCCUPANCY_ALGO` (默认 height_cost)
    /// - `NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS` (默认 0)
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
            occupancy_algo: parse_env_occupancy_algo(),
            initial_safe_radius_meters: parse_env_nonnegative_f32(
                "NUWAX_MAPPING_INITIAL_SAFE_RADIUS_METERS",
                0.0,
            ),
        }
    }
}

fn parse_env_occupancy_algo() -> CostAlgoSelector {
    let Some(value) = std::env::var("NUWAX_MAPPING_OCCUPANCY_ALGO")
        .ok()
        .map(|value| value.trim().to_ascii_lowercase())
        .filter(|value| !value.is_empty())
    else {
        return CostAlgoSelector::HeightCost;
    };
    match value.as_str() {
        "height_cost" | "height-cost" => CostAlgoSelector::HeightCost,
        "general" | "probability" => CostAlgoSelector::General,
        "simple" | "probabilistic" => CostAlgoSelector::Simple,
        other => {
            warn!("NUWAX_MAPPING_OCCUPANCY_ALGO={other:?} 无效;回退到原版默认 height_cost");
            CostAlgoSelector::HeightCost
        }
    }
}

fn parse_env_nonnegative_f32(key: &str, default: f32) -> f32 {
    std::env::var(key)
        .ok()
        .and_then(|value| value.trim().parse::<f32>().ok())
        .filter(|value| value.is_finite() && *value >= 0.0)
        .unwrap_or(default)
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

/// 仅为旧的 crate-private 测试访问保留的 accumulator 兼容壳。
///
/// 上游 `XyzAccumulator` 暴露的是 `into_points(self)`,无法在 `&self` 上克隆,
/// 因此在 adapter 内重新实现一个等价 `FeedOnlyAccumulator`
/// (只允许 feed + clone_points)。
///
/// **2026-07-13 W37 (RALPLAN §Phase M #1)**:此类型已 **deprecated**,
/// 生产写入已经全部迁移到 `MappingSession`。`MappingAdapter::accumulator` 只因
/// `lcm_runtime.rs` 的 crate-private 测试仍直接读取该字段而保留。
#[deprecated(
    since = "0.1.0-w37",
    note = "由 nuwax_mapping::MappingSession 替代;保留为 compat alias 直到 lcm_runtime.rs 切换"
)]
pub struct FeedOnlyAccumulator {
    points: Vec<Point3<f64>>,
}

#[allow(deprecated)]
impl FeedOnlyAccumulator {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }
    /// 返回当前累积的点的克隆(用于 occupancy builder 输入)
    pub fn clone_points(&self) -> Vec<Point3<f64>> {
        self.points.clone()
    }
}

#[allow(deprecated)]
impl Default for FeedOnlyAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[allow(deprecated)]
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MappingSessionPhase {
    Idle,
    Active,
    Stopped,
}

impl MappingSessionPhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Idle => "IDLE",
            Self::Active => "ACTIVE",
            Self::Stopped => "STOPPED",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MappingSessionSnapshot {
    pub phase: MappingSessionPhase,
    pub session_id: String,
    pub map_name: String,
    pub map_version_id: String,
    pub session_generation: u64,
    pub revision: u64,
    pub active: bool,
    pub idempotent_replay: bool,
}

impl MappingSessionSnapshot {
    fn idle() -> Self {
        Self {
            phase: MappingSessionPhase::Idle,
            session_id: String::new(),
            map_name: String::new(),
            map_version_id: String::new(),
            session_generation: 0,
            revision: 0,
            active: false,
            idempotent_replay: false,
        }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum MappingLifecycleError {
    #[error("{0}")]
    InvalidArgument(String),
    #[error("active mapping session conflicts with request: {0}")]
    Conflict(String),
    #[error("mapping session id does not match current session: {0}")]
    StaleSession(String),
    #[error("mapping session id was already stopped and cannot be reused: {0}")]
    TerminalSession(String),
    #[error("mapping session generation exhausted")]
    GenerationExhausted,
}

/// nuwax-mapping adapter 本体。
///
/// 持有:
/// - `accumulator: FeedOnlyAccumulator` —— 仅测试兼容，不接收生产数据。
/// - `session: MappingSession` —— **RALPLAN §Phase M #1 有界增量 session**
///   bounded_capacity 默认 8M 滚动点预算 + 消息时间 freshness + 有界重放 dedup。
/// - `cost_mapper: CostMapper` —— occupancy → costmap
/// - `last_pose: (x, y, z)` —— 上一次 odom pose,作为 occupancy builder sensor_origin
///
/// 公开方法:
/// - `feed_pointcloud(&[Point3<f64>])` —— 无源时间戳的兼容入口
/// - `feed_pointcloud_session(...)` —— 喂入到 session(带 freshness / dedup / typed error)
/// - `feed_pose(x, y, z)` —— 更新 sensor_origin (从 LCM /odom 解析)
/// - `build_and_publish(&SharedState, Option<&Sender<MapRevisionEvent>>)` ——
///   生成 occupancy + costmap,写回 SharedState.map;M #3 同步广播 MapRevisionEvent
/// - `session_stats()` —— session 快照(Phase M readiness 暴露)
/// - `increment_odom_count() / increment_pointcloud_count() / increment_occupancy_revision()`
///   —— `server.rs::publish_odom / publish_point_cloud` 同步累加计数
///   (Phase 5 桥接层保持 feed_* 函数签名不变,只在 call site 加 +1)
/// - `readiness()` —— 综合字段,给 Python diagnostics + runtime_diagnostics 对账
pub struct MappingAdapter {
    config: MappingAdapterConfig,
    /// 旧的 crate-private 测试仍直接读取该字段；生产入口不再写入。
    #[allow(deprecated)]
    pub(crate) accumulator: FeedOnlyAccumulator,
    /// **W37 RALPLAN §Phase M #1**:有界增量 `MappingSession`。
    ///
    /// 默认 `bounded_capacity = DEFAULT_BOUNDED_CAPACITY = 8_000_000`
    /// 是跨帧滚动窗口，与单帧点数上限无关；freshness 允许消息时间最多逆序 200ms；
    /// dedup 只过滤同时间戳、同内容、同位姿的重放。
    ///
    /// **W37 直持 `MappingSession`**:与上游 `nuwax_mapping::MappingSession`
    /// 完全对齐;`feed_*` 走 `&mut self`,`session_stats` 走 `&self`。
    pub(crate) session: MappingSession,
    cost_mapper: CostMapper,
    pub(crate) last_sensor_origin: [f64; 3],
    /// `has_data` 字段需要 `pub(crate)` 让 `lcm_runtime::spawn_mapping_feed_tasks`
    /// 在 feed 路径同步置 true;`mapping_ready_poll` 通过 `has_data()` 方法访问。
    pub(crate) has_data: bool,
    /// 当前 mapping session 已提交的 occupancy revision。新 session 在首图前归零。
    revision: u64,
    /// 进程级 occupancy revision 水位。跨 mapping session 保持单调，避免无 session
    /// identity 的 `nav_msgs.OccupancyGrid` consumer 把新会话首图当作旧 revision。
    global_revision: u64,
    /// Production uses the crash-durable allocator. Legacy unit constructors
    /// retain an in-memory checked allocator so tests never touch host state.
    sequence_store: Option<Arc<SequenceStateStore>>,
    /// Phase 7:计数 odom / pointcloud / occupancy revision,给 readiness 暴露用。
    /// 不参与 build_and_publish 写回 SharedState.map.revision 逻辑(那是 mapping 主路径)。
    odom_count: u64,
    pointcloud_count: u64,
    map_frame_id: String,
    odom_frame_id: String,
    latest_pointcloud_source_frame_id: String,
    latest_pointcloud_frame_id: String,
    latest_pointcloud_transform_applied: bool,
    latest_pointcloud_timestamp_ns: u64,
    latest_pointcloud_received_at_ns: u64,
    map_odom_transform: Option<MapOdomTransformSnapshot>,
    frame_generation: u64,
    global_frame_established: bool,
    transformed_points: Vec<Point3<f64>>,
    latest_accepted_map_points: Vec<Point3<f64>>,
    checkpoint_grid: Option<OccupancyGrid>,
    last_built_generation: u64,
    last_built_point_count: usize,
    last_built_received_at_ns: u64,
    lifecycle: MappingSessionSnapshot,
    next_session_generation: u64,
}

impl MappingAdapter {
    /// 构造 adapter;配置从 env 探测,`CostMapper` 使用原版 occupancy selector。
    pub fn new(config: MappingAdapterConfig) -> Self {
        Self::new_with_frames(config, "map", "odom")
    }

    /// 构造带显式 frame 契约的 adapter。
    ///
    /// LCM owner 应把运行时 canonical map/odom frame 传入此入口，避免各层独立
    /// 读取 env。空 frame 会回退到 `map` / `odom`。
    pub fn new_with_frames(
        config: MappingAdapterConfig,
        map_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
    ) -> Self {
        Self::new_with_frames_and_optional_sequence_store(config, map_frame_id, odom_frame_id, None)
    }

    /// Construct a production adapter backed by the process-exclusive durable
    /// map-revision allocator shared with the GoalEnvelope producer.
    pub(crate) fn new_with_frames_and_sequence_store(
        config: MappingAdapterConfig,
        map_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
        sequence_store: Arc<SequenceStateStore>,
    ) -> Self {
        Self::new_with_frames_and_optional_sequence_store(
            config,
            map_frame_id,
            odom_frame_id,
            Some(sequence_store),
        )
    }

    fn new_with_frames_and_optional_sequence_store(
        config: MappingAdapterConfig,
        map_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
        sequence_store: Option<Arc<SequenceStateStore>>,
    ) -> Self {
        let cost_mapper = CostMapper::new(CostMapperConfig {
            config: OccupancyConfig {
                algo: config.occupancy_algo,
                pointcloud: PointCloudOccConfig::default(),
                initial_safe_radius_meters: config.initial_safe_radius_meters,
            },
        });
        let map_frame_id = normalize_frame_id(&map_frame_id.into());
        let odom_frame_id = normalize_frame_id(&odom_frame_id.into());
        let map_frame_id = if map_frame_id.is_empty() {
            "map".to_string()
        } else {
            map_frame_id
        };
        let odom_frame_id = if odom_frame_id.is_empty() {
            "odom".to_string()
        } else {
            odom_frame_id
        };
        let global_revision = sequence_store
            .as_ref()
            .map_or(0, |store| store.current_map_revision());
        Self {
            config,
            #[allow(deprecated)]
            accumulator: FeedOnlyAccumulator::new(),
            session: MappingSession::new(),
            cost_mapper,
            last_sensor_origin: [0.0, 0.0, 0.0],
            has_data: false,
            revision: 0,
            global_revision,
            sequence_store,
            odom_count: 0,
            pointcloud_count: 0,
            map_frame_id: map_frame_id.clone(),
            odom_frame_id,
            latest_pointcloud_source_frame_id: String::new(),
            latest_pointcloud_frame_id: String::new(),
            latest_pointcloud_transform_applied: false,
            latest_pointcloud_timestamp_ns: 0,
            latest_pointcloud_received_at_ns: 0,
            map_odom_transform: None,
            frame_generation: 0,
            global_frame_established: false,
            transformed_points: Vec::new(),
            latest_accepted_map_points: Vec::new(),
            checkpoint_grid: None,
            last_built_generation: 0,
            last_built_point_count: 0,
            last_built_received_at_ns: 0,
            lifecycle: MappingSessionSnapshot::idle(),
            next_session_generation: 0,
        }
    }

    fn reserve_next_revision(&self, floor: u64) -> Result<u64, String> {
        if let Some(store) = self.sequence_store.as_ref() {
            return store
                .reserve_map_revision(floor)
                .map_err(|error| error.to_string());
        }
        let next = floor
            .checked_add(1)
            .ok_or_else(|| format!("map revision exhausted above floor {floor}"))?;
        let wire_revision = i32::try_from(next)
            .map_err(|_| format!("map revision {next} exceeds positive LCM i32 domain"))?;
        if wire_revision <= 0 {
            return Err(format!(
                "map revision {next} is outside positive LCM i32 domain"
            ));
        }
        Ok(next)
    }

    pub fn start_session(
        &mut self,
        session_id: &str,
        map_name: &str,
        map_version_id: &str,
    ) -> Result<MappingSessionSnapshot, MappingLifecycleError> {
        let session_id = session_id.trim();
        let map_name = map_name.trim();
        let map_version_id = map_version_id.trim();
        for (field, value) in [
            ("session_id", session_id),
            ("map_name", map_name),
            ("map_version_id", map_version_id),
        ] {
            if value.is_empty() {
                return Err(MappingLifecycleError::InvalidArgument(format!(
                    "{field} must not be empty"
                )));
            }
        }

        if self.lifecycle.active {
            if self.lifecycle.session_id == session_id
                && self.lifecycle.map_name == map_name
                && self.lifecycle.map_version_id == map_version_id
            {
                let mut replay = self.snapshot();
                replay.idempotent_replay = true;
                return Ok(replay);
            }
            return Err(MappingLifecycleError::Conflict(format!(
                "active_session_id={} active_map_name={} active_map_version_id={}",
                self.lifecycle.session_id, self.lifecycle.map_name, self.lifecycle.map_version_id
            )));
        }
        if self.lifecycle.phase == MappingSessionPhase::Stopped
            && self.lifecycle.session_id == session_id
        {
            return Err(MappingLifecycleError::TerminalSession(
                session_id.to_string(),
            ));
        }

        let generation = self
            .next_session_generation
            .checked_add(1)
            .ok_or(MappingLifecycleError::GenerationExhausted)?;
        self.reset_for_new_session();
        self.next_session_generation = generation;
        self.lifecycle = MappingSessionSnapshot {
            phase: MappingSessionPhase::Active,
            session_id: session_id.to_string(),
            map_name: map_name.to_string(),
            map_version_id: map_version_id.to_string(),
            session_generation: generation,
            revision: 0,
            active: true,
            idempotent_replay: false,
        };
        Ok(self.snapshot())
    }

    pub fn stop_session(
        &mut self,
        session_id: &str,
    ) -> Result<MappingSessionSnapshot, MappingLifecycleError> {
        let session_id = session_id.trim();
        if session_id.is_empty() {
            return Err(MappingLifecycleError::InvalidArgument(
                "session_id must not be empty".to_string(),
            ));
        }
        if self.lifecycle.session_id != session_id {
            return Err(MappingLifecycleError::StaleSession(format!(
                "requested={session_id} current={}",
                self.lifecycle.session_id
            )));
        }
        if self.lifecycle.phase == MappingSessionPhase::Stopped {
            let mut replay = self.snapshot();
            replay.idempotent_replay = true;
            return Ok(replay);
        }
        if !self.lifecycle.active {
            return Err(MappingLifecycleError::StaleSession(format!(
                "requested={session_id} current_phase={}",
                self.lifecycle.phase.as_str()
            )));
        }
        self.lifecycle.phase = MappingSessionPhase::Stopped;
        self.lifecycle.active = false;
        Ok(self.snapshot())
    }

    pub fn install_checkpoint(
        &mut self,
        checkpoint: OccupancyGrid,
        revision_floor: u64,
    ) -> Result<(MapState, MapRevisionEvent), String> {
        if !self.is_active() {
            return Err("mapping session must be active before checkpoint install".to_string());
        }
        validate_checkpoint_grid(&checkpoint, &self.map_frame_id)?;
        let next_revision = self
            .reserve_next_revision(revision_floor.max(self.revision).max(self.global_revision))?;
        let timestamp_ns = if checkpoint.ts.is_finite() && checkpoint.ts > 0.0 {
            (checkpoint.ts * 1_000_000_000.0).round() as u64
        } else {
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64
        };
        let map = MapState {
            width: checkpoint.width,
            height: checkpoint.height,
            resolution: checkpoint.resolution as f32,
            origin_x: checkpoint.origin[0],
            origin_y: checkpoint.origin[1],
            frame_id: checkpoint.frame_id.clone(),
            revision: next_revision,
            timestamp_ns,
            data: checkpoint.data.clone(),
        };
        self.checkpoint_grid = Some(checkpoint);
        self.global_revision = next_revision;
        self.revision = next_revision;
        self.lifecycle.revision = next_revision;
        let event = MapRevisionEvent::from_map(&map, &self.lifecycle);
        Ok((map, event))
    }

    pub fn has_checkpoint(&self) -> bool {
        self.checkpoint_grid.is_some() && self.is_active() && self.current_revision() > 0
    }

    pub fn snapshot(&self) -> MappingSessionSnapshot {
        let mut snapshot = self.lifecycle.clone();
        snapshot.revision = self.revision;
        snapshot.idempotent_replay = false;
        snapshot
    }

    pub fn is_active(&self) -> bool {
        self.lifecycle.active
    }

    /// 喂入一帧点云;`mapping_adapter` 不持锁跨 await,调用方负责同步。
    ///
    /// 无源时间戳时不做重放去重，但仍受滚动点预算约束。
    pub fn feed_pointcloud(&mut self, points: &[Point3<f64>]) {
        if !self.is_active() || points.is_empty() {
            return;
        }
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let stats = self.session.feed_unstamped(points);
        if stats.accepted > 0 {
            self.pointcloud_count = self.pointcloud_count.saturating_add(1);
            self.has_data = true;
            self.latest_pointcloud_source_frame_id = self.map_frame_id.clone();
            self.latest_pointcloud_frame_id = self.map_frame_id.clone();
            self.latest_pointcloud_transform_applied = false;
            self.latest_pointcloud_timestamp_ns = now_ns;
            self.latest_pointcloud_received_at_ns = now_ns;
            if !self.global_frame_established {
                self.frame_generation = self.frame_generation.saturating_add(1).max(1);
                self.global_frame_established = true;
            }
        }
    }

    /// 喂入一帧点云到 `MappingSession`(RALPLAN §Phase M #1 主入口)。
    ///
    /// 与 `feed_pointcloud` 的差异:
    /// - freshness 使用 `source_timestamp_ns`；仅在源时间缺失时回退 receive time
    /// - 容量压力滚动淘汰旧点，不会令 session 永久停更
    /// - dedup 只过滤完整消息重放，静止位置的新采样仍被接收
    ///
    /// **调用方契约**:由 `lcm_runtime::spawn_mapping_feed_tasks` 或 `server.rs`
    /// 在解析 `/odom` + `/go2/lidar/points` LCM 帧后调用。
    pub fn feed_pointcloud_session(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        received_at_ns: u64,
        frame_id: &str,
        source_timestamp_ns: u64,
    ) -> Result<nuwax_mapping::FeedStats, nuwax_mapping::SessionError> {
        self.latest_accepted_map_points.clear();
        let source_frame_id = normalize_frame_id(frame_id);
        if !self.is_active() {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping session is inactive".to_string(),
            });
        }
        let is_map_frame = source_frame_id == self.map_frame_id;
        let is_odom_frame = source_frame_id == self.odom_frame_id;
        if !is_map_frame && !is_odom_frame {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "source frame is neither configured map nor odom".to_string(),
            });
        }
        if is_odom_frame && self.map_odom_transform.is_none() {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "first valid odom pose has not anchored T_map_odom".to_string(),
            });
        }
        if points.is_empty() {
            return Ok(nuwax_mapping::FeedStats::accepted(
                0,
                self.session.current_size(),
                self.session.capacity(),
                self.session.last_seen_at_ns(),
            ));
        }
        let message_timestamp_ns = if source_timestamp_ns > 0 {
            source_timestamp_ns
        } else {
            received_at_ns
        };
        let (map_points, dedup_xy, transform_applied): (&[Point3<f64>], (f64, f64), bool) =
            if is_map_frame {
                (points, odom_xy, false)
            } else {
                let transform = self
                    .map_odom_transform
                    .as_ref()
                    .expect("odom frame was guarded by an established transform")
                    .clone();
                self.transformed_points.clear();
                self.transformed_points.reserve(points.len());
                self.transformed_points.extend(points.iter().map(|point| {
                    let transformed = transform.transform_position([point.x, point.y, point.z]);
                    Point3::new(transformed[0], transformed[1], transformed[2])
                }));
                (
                    self.transformed_points.as_slice(),
                    (self.last_sensor_origin[0], self.last_sensor_origin[1]),
                    true,
                )
            };
        let stats =
            self.session
                .feed_pointcloud(map_points, dedup_xy, point_step, message_timestamp_ns)?;
        if stats.accepted > 0 {
            let accepted_start = map_points.len().saturating_sub(stats.accepted);
            self.latest_accepted_map_points
                .extend_from_slice(&map_points[accepted_start..]);
            self.has_data = true;
            self.pointcloud_count = self.pointcloud_count.saturating_add(1);
            self.latest_pointcloud_source_frame_id = source_frame_id;
            self.latest_pointcloud_frame_id = self.map_frame_id.clone();
            self.latest_pointcloud_transform_applied = transform_applied;
            self.latest_pointcloud_timestamp_ns = message_timestamp_ns;
            self.latest_pointcloud_received_at_ns = received_at_ns;
            if !self.global_frame_established {
                self.frame_generation = self.frame_generation.saturating_add(1).max(1);
                self.global_frame_established = true;
            }
        }
        Ok(stats)
    }

    /// 更新 sensor_origin (odom pose 平移部分)。
    ///
    /// **W37** 同步更新 `session.last_pose`,让 session 的 `last_pose` 与
    /// `MappingAdapter.last_sensor_origin` 保持一致。
    pub fn feed_pose(&mut self, x: f64, y: f64, z: f64) {
        let odom_frame_id = self.odom_frame_id.clone();
        let _ = self.feed_pose_in_frame(x, y, z, 0.0, &odom_frame_id);
    }

    /// 接收带 frame/yaw 的 pose，并把 session sensor origin 统一转换到 map frame。
    ///
    /// - 首个 odom pose 固定 `T_map_odom = inverse(T_odom_base)`；后续不漂移锚点。
    /// - map pose 直接写入。
    /// - 其它 frame 与非有限数值 fail-closed，不增加计数。
    pub fn feed_pose_in_frame(
        &mut self,
        x: f64,
        y: f64,
        z: f64,
        yaw_radians: f64,
        frame_id: &str,
    ) -> Result<PlanarPoseSnapshot, nuwax_mapping::SessionError> {
        let source_frame_id = normalize_frame_id(frame_id);
        if !self.is_active() {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping session is inactive".to_string(),
            });
        }
        if !x.is_finite() || !y.is_finite() || !z.is_finite() || !yaw_radians.is_finite() {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "pose contains a non-finite position or yaw".to_string(),
            });
        }
        let source_pose = PlanarPoseSnapshot {
            position: [x, y, z],
            yaw_radians: normalize_yaw(yaw_radians),
        };
        let map_pose = if source_frame_id == self.map_frame_id {
            if !self.global_frame_established {
                self.frame_generation = self.frame_generation.saturating_add(1).max(1);
                self.global_frame_established = true;
            }
            source_pose
        } else if source_frame_id == self.odom_frame_id {
            if self.map_odom_transform.is_none() {
                if self.global_frame_established {
                    return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                        source_frame_id,
                        target_frame_id: self.map_frame_id.clone(),
                        reason: "session already uses direct map input; reset before establishing an odom anchor"
                            .to_string(),
                    });
                }
                self.frame_generation = self.frame_generation.saturating_add(1).max(1);
                self.global_frame_established = true;
                self.map_odom_transform = Some(MapOdomTransformSnapshot::from_anchor(
                    &self.map_frame_id,
                    &self.odom_frame_id,
                    source_pose,
                    self.frame_generation,
                ));
            }
            let transform = self
                .map_odom_transform
                .as_ref()
                .expect("odom anchor was initialized above");
            transform.transform_pose(source_pose)
        } else {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "pose frame is neither configured map nor odom".to_string(),
            });
        };
        if let Some(transform) = self.map_odom_transform.as_mut() {
            transform.latest_map_pose = map_pose;
        }
        self.last_sensor_origin = map_pose.position;
        self.session.feed_pose(
            map_pose.position[0],
            map_pose.position[1],
            map_pose.position[2],
        );
        self.odom_count = self.odom_count.saturating_add(1);
        Ok(map_pose)
    }

    /// session 快照(Phase M readiness 暴露)。
    pub fn session_stats(&self) -> MappingSessionStats {
        self.session.snapshot_stats()
    }

    /// 当前是否有数据 (用于 `mapping_ready_poll` 判定)。
    pub fn has_data(&self) -> bool {
        self.is_active()
            && self.has_data
            && self
                .frame_contract(&self.map_frame_id)
                .ready_for_global_consumers()
    }

    /// 当前累计点数 (debug / diagnostics 用)。
    ///
    /// `session` 是唯一生产 SSOT；返回滚动点窗口的当前大小。
    pub fn accumulated_count(&self) -> usize {
        self.session.current_size()
    }

    /// Move out the most recent accepted frame after it has been transformed
    /// into the canonical map frame. Production consumes this once to update
    /// the independent cloud-map voxel accumulator without cloning the full
    /// MappingSession rolling window.
    pub(crate) fn take_latest_accepted_map_points(&mut self) -> Vec<Point3<f64>> {
        std::mem::take(&mut self.latest_accepted_map_points)
    }

    /// 描述最近一次已接收点云的源/output frame 与真实 transform 状态。
    pub fn frame_contract(&self, expected_map_frame_id: &str) -> MapFrameContract {
        let expected_map_frame_id = normalize_frame_id(expected_map_frame_id);
        let source_frame_id = self.latest_pointcloud_source_frame_id.clone();
        let output_frame_id = self.latest_pointcloud_frame_id.clone();
        let transform_required = !source_frame_id.is_empty() && source_frame_id != output_frame_id;
        MapFrameContract {
            output_frame_id,
            source_frame_id,
            expected_map_frame_id,
            transform_required,
            transform_applied: self.latest_pointcloud_transform_applied,
        }
    }

    /// 返回当前固定 `map <- odom` 变换和最新 map pose 的只读快照。
    pub fn map_odom_transform_snapshot(&self) -> Option<MapOdomTransformSnapshot> {
        self.map_odom_transform.clone()
    }

    /// 当前全局 frame/anchor 代次。reset 后下一次 anchor 会使用更大的代次。
    pub fn global_frame_generation(&self) -> u64 {
        self.frame_generation
    }

    /// 当前 adapter 持有的 costmap 配置。
    pub fn cost_mapper_config(&self) -> &CostMapperConfig {
        self.cost_mapper.config()
    }

    /// 把当前累积的点云 + sensor_origin 转换为 occupancy grid。
    ///
    /// **不消费 session** —— 内部走 `session.clone_points()` 路径,
    /// 后续 feed 仍能继续累积;若要清空,调用方调用 `reset_accumulator`。
    ///
    /// 该只读诊断入口保留 clone；生产 `build_and_publish` 走零拷贝连续切片。
    pub fn build_occupancy(&self) -> Result<OccupancyGrid, MappingError> {
        self.build_maps().map(|(_, occupancy)| occupancy)
    }

    fn build_maps(&self) -> Result<(CostMapper, OccupancyGrid), MappingError> {
        let points = self.session.clone_points();
        let origin = self.last_sensor_origin;
        let cloud = SimplePointCloud2::with_metadata(
            points,
            nalgebra::point![origin[0], origin[1], origin[2]],
            self.latest_pointcloud_frame_id.clone(),
            self.latest_pointcloud_timestamp_ns as f64 / 1_000_000_000.0,
        );
        let (mapper, occupancy, _costmap) = CostMapper::from_pointcloud_auto(
            &cloud,
            &self.cost_mapper.config().config,
            self.config.resolution,
        )?;
        Ok((mapper, occupancy))
    }

    fn build_maps_zero_copy(&mut self) -> Result<(CostMapper, OccupancyGrid), MappingError> {
        let origin = self.last_sensor_origin;
        let frame_id = self.latest_pointcloud_frame_id.clone();
        let timestamp = self.latest_pointcloud_timestamp_ns as f64 / 1_000_000_000.0;
        let occupancy_config = self.cost_mapper.config().config.clone();
        let resolution = self.config.resolution;
        let cloud = BorrowedPointCloud {
            points: self.session.make_contiguous_points(),
            sensor_origin: nalgebra::point![origin[0], origin[1], origin[2]],
            frame_id: &frame_id,
            timestamp,
        };
        let (mapper, occupancy, _costmap) =
            CostMapper::from_pointcloud_auto(&cloud, &occupancy_config, resolution)?;
        Ok((mapper, occupancy))
    }

    fn should_rebuild(&self) -> bool {
        let generation = self.session.generation();
        if generation == self.last_built_generation {
            return false;
        }
        if self.revision == 0 {
            return true;
        }

        let elapsed_ns = self
            .latest_pointcloud_received_at_ns
            .saturating_sub(self.last_built_received_at_ns);
        if elapsed_ns >= MAX_REBUILD_INTERVAL_NS {
            return true;
        }
        if elapsed_ns < MIN_REBUILD_INTERVAL_NS {
            return false;
        }

        let current_count = self.session.current_size();
        if current_count < self.last_built_point_count {
            return true;
        }
        let new_points = current_count - self.last_built_point_count;
        let growth_threshold = (self.last_built_point_count / REBUILD_GROWTH_DIVISOR).max(1);
        new_points >= growth_threshold
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
    ///
    /// M #3 (RALPLAN §Phase 2): `map_revision_tx` 可选 —
    /// - 生产 `spawn_mapping_feed_tasks` 传 `Some(&handle.map_revision_tx)`,
    ///   commit 时通过 broadcast::Sender 推送 `MapRevisionEvent`
    ///   (data/size/schema_version/checksum),`server::subscribe_map_updates`
    ///   不再轮询 500ms。
    /// - 单测路径默认 `None` 保持兼容,无需广播通道也能验证业务。
    pub fn build_and_publish(
        &mut self,
        state: &Arc<RwLock<SharedState>>,
        map_revision_tx: Option<&broadcast::Sender<MapRevisionEvent>>,
    ) -> bool {
        if !self.is_active()
            || !self.has_data()
            || !self
                .frame_contract(&self.map_frame_id)
                .ready_for_global_consumers()
        {
            return false;
        }
        if !self.should_rebuild() {
            return false;
        }
        let built_generation = self.session.generation();
        let built_point_count = self.session.current_size();
        let built_received_at_ns = self.latest_pointcloud_received_at_ns;
        let (mapper, live_grid) = match self.build_maps_zero_copy() {
            Ok(result) => result,
            Err(error) => {
                warn!("mapping_adapter: auto-bounds occupancy 构建失败: {error}");
                return false;
            }
        };
        self.cost_mapper = mapper;
        let grid = match self.checkpoint_grid.as_ref() {
            Some(checkpoint) => match merge_checkpoint_grid(checkpoint, &live_grid) {
                Ok(merged) => merged,
                Err(error) => {
                    warn!("mapping_adapter: checkpoint/live occupancy 合并失败: {error}");
                    return false;
                }
            },
            None => live_grid,
        };
        if let Ok(mut s) = state.try_write() {
            let revision_floor = s.map.revision.max(self.revision).max(self.global_revision);
            let next_revision = match self.reserve_next_revision(revision_floor) {
                Ok(revision) => revision,
                Err(error) => {
                    warn!(
                        "mapping_adapter: revision reservation failed closed before commit: {error}"
                    );
                    return false;
                }
            };
            self.global_revision = next_revision;
            self.revision = next_revision;
            s.map = MapState {
                width: grid.width,
                height: grid.height,
                resolution: grid.resolution as f32,
                origin_x: grid.origin[0],
                origin_y: grid.origin[1],
                frame_id: grid.frame_id,
                revision: next_revision,
                timestamp_ns: self.latest_pointcloud_timestamp_ns,
                data: grid.data,
            };
            self.last_built_generation = built_generation;
            self.last_built_point_count = built_point_count;
            self.last_built_received_at_ns = built_received_at_ns;
            // 从刚 commit 的 MapState 构造自包含事件，确保 metadata/data/checksum
            // 属于同一 revision，server 不需要再读取可能已前进的 SharedState。
            self.lifecycle.revision = next_revision;
            let event = MapRevisionEvent::from_map(&s.map, &self.lifecycle);
            drop(s); // 释放 write lock,后续 send 不持锁
            if let Some(tx) = map_revision_tx {
                // broadcast::Sender::send 是 non-blocking,无 consumer 时返 Err,
                // 这里忽略(no subscriber — spawn 早期或被 unsubscribe)
                let _ = tx.send(event);
            }
            true
        } else {
            warn!("mapping_adapter: SharedState.try_write 失败,本轮 build_and_publish 跳过");
            false
        }
    }

    /// 清空 accumulator(用于重新建图 / 地图 reset)。
    ///
    /// 同时清理 session 和遗留测试兼容壳。
    pub fn reset_accumulator(&mut self) {
        #[allow(deprecated)]
        {
            self.accumulator = FeedOnlyAccumulator::new();
        }
        self.session.reset_state();
        self.has_data = false;
        self.last_sensor_origin = [0.0, 0.0, 0.0];
        self.latest_pointcloud_source_frame_id.clear();
        self.latest_pointcloud_frame_id.clear();
        self.latest_pointcloud_transform_applied = false;
        self.latest_pointcloud_timestamp_ns = 0;
        self.latest_pointcloud_received_at_ns = 0;
        self.map_odom_transform = None;
        self.global_frame_established = false;
        self.transformed_points.clear();
        self.latest_accepted_map_points.clear();
        self.checkpoint_grid = None;
        self.last_built_generation = 0;
        self.last_built_point_count = 0;
        self.last_built_received_at_ns = 0;
    }

    /// Reset per-session mapping data while preserving process-wide generations and revision.
    pub fn reset_for_new_session(&mut self) {
        self.reset_accumulator();
        // RPC/session projection must read zero until this session commits its first map.
        // `global_revision` deliberately survives so unversioned occupancy consumers never
        // observe a revision rollback when a later mapping session starts.
        self.revision = 0;
        self.odom_count = 0;
        self.pointcloud_count = 0;
    }

    /// adapter 当前 revision(给 diagnostics / 调试用)。
    pub fn current_revision(&self) -> u64 {
        self.revision
    }

    /// 进程级最后提交 revision。仅用于跨会话 fencing/诊断，不投影为新会话当前地图。
    pub fn global_revision(&self) -> u64 {
        self.global_revision
    }

    // ============================================================================
    // Phase 7:readiness counters + 综合 readiness 暴露
    // ============================================================================
    //
    // Counters advance only at accepted ingestion/commit boundaries. A producer-side
    // LCM publish acknowledgement is not evidence that mapping consumed the frame.

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
    /// 综合 ready(`global_consumer_ready && pointcloud_count > 0`)
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
    /// 最近点云是否具备真实 map-frame 输出契约。
    pub global_consumer_ready: bool,
    /// 当前 map-frame/odom-anchor 代次。
    pub frame_generation: u64,
    /// 最近一次错误(若 lcm_backend=false / adapter 未构造等)
    pub last_error: Option<String>,
}

impl MappingReadiness {
    /// 综合判定:`has_data` + 真实 pointcloud 输入 + lcm_backend + adapter present
    pub fn compute(adapter: Option<&MappingAdapter>, lcm_backend_available: bool) -> Self {
        let mut last_error: Option<String> = None;
        if !lcm_backend_available {
            last_error = Some("LCM backend 不可用,Mapping 板块输入被阻断".to_string());
        }
        let adapter_present = adapter.is_some();
        let mut ready = false;
        let (
            odom_count,
            pointcloud_count,
            last_occupancy_revision,
            accumulated_point_count,
            global_consumer_ready,
            frame_generation,
        ) = match adapter {
            Some(a) => {
                let frame_ready = a
                    .frame_contract(&a.map_frame_id)
                    .ready_for_global_consumers();
                let checkpoint_ready = a.has_checkpoint();
                if !a.is_active() {
                    last_error.get_or_insert_with(|| {
                        "mapping session 未启动;调用 StartMapping 后才接收输入".to_string()
                    });
                } else if checkpoint_ready {
                    // A validated checkpoint is already a global map consumer input.
                } else if !a.has_data && a.odom_count() == 0 && a.pointcloud_count() == 0 {
                    last_error
                        .get_or_insert_with(|| "尚未收到任何 odom / pointcloud 输入".to_string());
                } else if a.pointcloud_count() == 0 {
                    last_error.get_or_insert_with(|| {
                        "尚未收到 pointcloud,仅有 odom 不算 ready".to_string()
                    });
                } else if a.current_revision() == 0 {
                    last_error.get_or_insert_with(|| {
                        "pointcloud 已接收但 occupancy revision 尚未提交".to_string()
                    });
                } else if !frame_ready {
                    last_error.get_or_insert_with(|| {
                        "mapping frame anchor/transform 尚未建立，禁止全局消费".to_string()
                    });
                }
                (
                    a.odom_count(),
                    a.pointcloud_count(),
                    a.current_revision(),
                    a.accumulated_count(),
                    frame_ready || checkpoint_ready,
                    a.global_frame_generation(),
                )
            }
            None => {
                last_error.get_or_insert_with(|| {
                    "Mapping adapter 未构造(LCM backend 不可用)".to_string()
                });
                (0, 0, 0, 0, false, 0)
            }
        };
        if let Some(adapter) = adapter {
            let checkpoint_ready = adapter.has_checkpoint();
            ready = lcm_backend_available
                && (adapter.has_data() || checkpoint_ready)
                && global_consumer_ready
                && (pointcloud_count > 0 || checkpoint_ready)
                && last_occupancy_revision > 0;
        }
        let reason = if ready {
            "mapping 已就绪".to_string()
        } else {
            last_error
                .clone()
                .unwrap_or_else(|| "mapping 未就绪".to_string())
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
            global_consumer_ready,
            frame_generation,
            last_error,
        }
    }
}

fn validate_checkpoint_grid(grid: &OccupancyGrid, expected_frame_id: &str) -> Result<(), String> {
    if grid.width == 0 || grid.height == 0 {
        return Err("checkpoint width/height must be positive".to_string());
    }
    let expected_len = (grid.width as usize)
        .checked_mul(grid.height as usize)
        .ok_or_else(|| "checkpoint dimensions overflow".to_string())?;
    if grid.data.len() != expected_len {
        return Err(format!(
            "checkpoint data size mismatch: expected={expected_len} actual={}",
            grid.data.len()
        ));
    }
    if !grid.resolution.is_finite() || grid.resolution <= 0.0 {
        return Err("checkpoint resolution must be finite and positive".to_string());
    }
    if grid.frame_id.trim().trim_start_matches('/') != expected_frame_id {
        return Err(format!(
            "checkpoint frame must be {expected_frame_id}, received {}",
            grid.frame_id
        ));
    }
    if grid
        .origin
        .iter()
        .chain(grid.origin_orientation.iter())
        .any(|value| !value.is_finite())
    {
        return Err("checkpoint origin contains non-finite values".to_string());
    }
    if grid.data.iter().any(|value| !(-1..=100).contains(value)) {
        return Err("checkpoint occupancy values must be within [-1, 100]".to_string());
    }
    Ok(())
}

fn merge_checkpoint_grid(
    checkpoint: &OccupancyGrid,
    live: &OccupancyGrid,
) -> Result<OccupancyGrid, String> {
    validate_checkpoint_grid(
        checkpoint,
        checkpoint.frame_id.trim().trim_start_matches('/'),
    )?;
    validate_checkpoint_grid(live, live.frame_id.trim().trim_start_matches('/'))?;
    if checkpoint.frame_id.trim().trim_start_matches('/')
        != live.frame_id.trim().trim_start_matches('/')
    {
        return Err("checkpoint/live frames differ".to_string());
    }
    let resolution = checkpoint.resolution;
    let tolerance = resolution.abs().max(1.0) * 1e-6;
    if (live.resolution - resolution).abs() > tolerance {
        return Err("checkpoint/live resolutions differ".to_string());
    }
    if checkpoint.origin_orientation != live.origin_orientation {
        return Err("checkpoint/live origin orientations differ".to_string());
    }

    let snap_near_integer = |value: f64| {
        let rounded = value.round();
        if (value - rounded).abs() <= 1e-6 {
            rounded
        } else {
            value
        }
    };
    let live_min_col = (live.origin[0] - checkpoint.origin[0]) / resolution;
    let live_min_row = (live.origin[1] - checkpoint.origin[1]) / resolution;
    let live_max_col =
        (live.origin[0] + live.width as f64 * live.resolution - checkpoint.origin[0]) / resolution;
    let live_max_row =
        (live.origin[1] + live.height as f64 * live.resolution - checkpoint.origin[1]) / resolution;
    let min_col = snap_near_integer(live_min_col.min(0.0)).floor() as i64;
    let min_row = snap_near_integer(live_min_row.min(0.0)).floor() as i64;
    let max_col = snap_near_integer(live_max_col.max(checkpoint.width as f64)).ceil() as i64;
    let max_row = snap_near_integer(live_max_row.max(checkpoint.height as f64)).ceil() as i64;
    let width = max_col.saturating_sub(min_col);
    let height = max_row.saturating_sub(min_row);
    if width <= 0 || height <= 0 || width > u32::MAX as i64 || height > u32::MAX as i64 {
        return Err("merged checkpoint dimensions are invalid".to_string());
    }
    let mut merged = OccupancyGrid::with_frame(
        width as u32,
        height as u32,
        resolution,
        &checkpoint.frame_id,
    );
    merged.origin = [
        checkpoint.origin[0] + min_col as f64 * resolution,
        checkpoint.origin[1] + min_row as f64 * resolution,
        checkpoint.origin[2],
    ];
    merged.origin_orientation = checkpoint.origin_orientation;
    merged.ts = checkpoint.ts.max(live.ts);

    for source in [checkpoint, live] {
        for row in 0..source.height {
            for col in 0..source.width {
                let incoming = source.data[source.index(row, col)];
                if incoming < 0 {
                    continue;
                }
                let world_x = source.origin[0] + (col as f64 + 0.5) * source.resolution;
                let world_y = source.origin[1] + (row as f64 + 0.5) * source.resolution;
                let target_col = ((world_x - merged.origin[0]) / resolution).floor() as i64;
                let target_row = ((world_y - merged.origin[1]) / resolution).floor() as i64;
                if target_col < 0 || target_row < 0 || target_col >= width || target_row >= height {
                    return Err("checkpoint/live cell projects outside merged grid".to_string());
                }
                let target_index = merged.index(target_row as u32, target_col as u32);
                let existing = merged.data[target_index];
                merged.data[target_index] = if existing < 0 {
                    incoming
                } else {
                    existing.max(incoming)
                };
            }
        }
    }
    Ok(merged)
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
    use nuwax_mapping::cost_mapper::CostAlgoSelector;
    use nuwax_mapping::occupancy::types::OccupancyValue;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU64, Ordering};

    static NEXT_SEQUENCE_TEST_PATH: AtomicU64 = AtomicU64::new(0);

    const GENERAL_GOLDEN: &[i8] = &[
        -1, -1, -1, -1, -1, -1, -1, // row 0
        -1, -1, -1, 0, -1, -1, -1, // row 1
        -1, -1, 100, 0, 100, -1, -1, // row 2
        -1, -1, -1, 0, -1, -1, -1, // row 3
    ];

    fn sample_config() -> MappingAdapterConfig {
        MappingAdapterConfig {
            resolution: 0.1,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-1.0, -1.0),
            voxel_size: 0.1,
            occupancy_algo: CostAlgoSelector::HeightCost,
            initial_safe_radius_meters: 0.0,
        }
    }

    fn active_adapter(config: MappingAdapterConfig) -> MappingAdapter {
        let mut adapter = MappingAdapter::new(config);
        adapter
            .start_session("test-session", "test-map", "mapver_000001")
            .expect("activate test mapping session");
        adapter
    }

    fn sequence_test_path(label: &str) -> PathBuf {
        std::env::temp_dir()
            .join(format!(
                "nuwax-mapping-sequence-{}-{}-{label}",
                std::process::id(),
                NEXT_SEQUENCE_TEST_PATH.fetch_add(1, Ordering::Relaxed),
            ))
            .join("sequence_state.json")
    }

    fn active_persistent_adapter(sequence_store: Arc<SequenceStateStore>) -> MappingAdapter {
        let mut adapter = MappingAdapter::new_with_frames_and_sequence_store(
            sample_config(),
            "map",
            "odom",
            sequence_store,
        );
        adapter
            .start_session("persistent-session", "test-map", "mapver_000001")
            .expect("activate persistent mapping session");
        adapter
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-9,
            "actual={actual} expected={expected}"
        );
    }

    #[test]
    fn adapter_default_state_is_empty() {
        let a = MappingAdapter::new(sample_config());
        assert!(!a.has_data());
        assert_eq!(a.accumulated_count(), 0);
        assert_eq!(a.current_revision(), 0);
        assert_eq!(a.snapshot().phase, MappingSessionPhase::Idle);
        assert!(!a.is_active());
    }

    #[test]
    fn inactive_adapter_rejects_ingestion_and_publish() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut adapter = MappingAdapter::new(sample_config());
        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "map")
            .is_err());
        assert_eq!(adapter.accumulated_count(), 0);
        assert_eq!(adapter.pointcloud_count(), 0);
        assert!(!adapter.build_and_publish(&state, None));
    }

    #[test]
    fn feed_pointcloud_marks_has_data() {
        let mut a = active_adapter(sample_config());
        a.feed_pointcloud(&[Point3::new(0.5, 0.5, 0.0)]);
        assert!(a.has_data());
        assert_eq!(a.accumulated_count(), 1);
        assert_eq!(a.pointcloud_count(), 1);
    }

    #[test]
    fn feed_pointcloud_empty_no_mark() {
        let mut a = active_adapter(sample_config());
        a.feed_pointcloud(&[]);
        assert!(!a.has_data());
    }

    #[test]
    fn first_odom_pose_anchors_map_origin() {
        let mut a = active_adapter(sample_config());
        a.feed_pose(1.0, 2.0, 0.0);
        assert_eq!(a.last_sensor_origin, [0.0, 0.0, 0.0]);
        assert_eq!(a.odom_count(), 1);
        let transform = a
            .map_odom_transform_snapshot()
            .expect("first odom pose establishes an anchor");
        assert_eq!(transform.translation, [-1.0, -2.0, 0.0]);
        assert_eq!(transform.latest_map_pose.position, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn build_occupancy_produces_grid() {
        let mut a = active_adapter(sample_config());
        // 默认 HeightCost 语义下,z > 1.5m 才是 Occupied；z=0 属于 Unknown。
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0), Point3::new(0.1, 0.0, 2.0)]);
        let grid = a.build_occupancy().expect("valid auto-bounds occupancy");
        assert_eq!(grid.width, 21);
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
    fn production_mapping_uses_auto_bounds_selected_kernel_and_source_metadata() {
        let mut config = sample_config();
        config.resolution = 0.5;
        config.occupancy_algo = CostAlgoSelector::General;
        let mut adapter = active_adapter(config);
        let points = vec![
            Point3::new(0.00, 0.00, 0.00),
            Point3::new(0.00, 0.00, 0.50),
            Point3::new(0.74, 0.00, 0.00),
            Point3::new(1.26, 0.00, 0.50),
        ];
        let source_timestamp_ns = 123_250_000_000_u64;

        adapter
            .feed_pointcloud_session(
                &points,
                (0.0, 0.0),
                12,
                1_750_000_000_000_000_000,
                "map",
                source_timestamp_ns,
            )
            .expect("accepted mapping frame");

        let occupancy = adapter
            .build_occupancy()
            .expect("selected production occupancy");
        assert_eq!((occupancy.width, occupancy.height), (7, 4));
        assert_eq!(occupancy.origin, [-1.0, -1.0, 0.0]);
        assert_eq!(occupancy.frame_id, "map");
        assert_eq!(occupancy.ts, 123.25);
        assert_eq!(occupancy.data, GENERAL_GOLDEN);

        let state = Arc::new(RwLock::new(SharedState::default()));
        assert!(adapter.build_and_publish(&state, None));
        let state = state.try_read().expect("committed map state");
        assert_eq!((state.map.width, state.map.height), (7, 4));
        assert_eq!((state.map.origin_x, state.map.origin_y), (-1.0, -1.0));
        assert_eq!(state.map.frame_id, "map");
        assert_eq!(state.map.timestamp_ns, source_timestamp_ns);
        assert_eq!(state.map.data, GENERAL_GOLDEN);
    }

    #[test]
    fn build_and_publish_writes_shared_state() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut a = active_adapter(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        // 测试路径不接 broadcast Sender,传 None — 保持 Phase 5 后向兼容。
        assert!(a.build_and_publish(&state, None));
        let s = state.try_read().expect("read");
        assert!(s.map.revision >= 1);
        assert_eq!(s.map.width, 20);
        assert_eq!(s.map.height, 20);
    }

    #[test]
    fn build_and_publish_no_data_returns_false() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        let mut a = active_adapter(sample_config());
        assert!(!a.build_and_publish(&state, None));
    }

    #[test]
    fn persistence_failure_cannot_advance_mapping_or_fanout() {
        let path = sequence_test_path("persist-failure");
        let parent = path.parent().unwrap().to_path_buf();
        let lock_path = parent.join("sequence_state.json.lock");
        let store = Arc::new(SequenceStateStore::open(&path).expect("sequence store"));
        let mut adapter = active_persistent_adapter(store);
        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0)]);
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(1);

        std::fs::remove_file(&path).expect("remove state before failure injection");
        std::fs::remove_file(lock_path).expect("unlink held lock file");
        std::fs::remove_dir(&parent).expect("remove sequence directory");
        std::fs::write(&parent, b"not a directory").expect("install unwritable path component");

        assert!(!adapter.build_and_publish(&state, Some(&map_revision_tx)));
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(adapter.global_revision(), 0);
        assert_eq!(state.try_read().unwrap().map.revision, 0);
        assert!(matches!(
            map_revision_rx.try_recv(),
            Err(broadcast::error::TryRecvError::Empty)
        ));
        assert!(
            !adapter.build_and_publish(&state, Some(&map_revision_tx)),
            "poisoned allocator must remain failed closed"
        );
    }

    #[test]
    fn i32_max_revision_exhaustion_cannot_advance_mapping_or_fanout() {
        let path = sequence_test_path("i32-max");
        let store = Arc::new(SequenceStateStore::open(&path).expect("sequence store"));
        assert_eq!(
            store
                .reserve_map_revision(i32::MAX as u64 - 1)
                .expect("reserve final wire revision"),
            i32::MAX as u64
        );
        let mut adapter = active_persistent_adapter(store);
        assert_eq!(adapter.global_revision(), i32::MAX as u64);
        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0)]);
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(1);

        assert!(!adapter.build_and_publish(&state, Some(&map_revision_tx)));
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(adapter.global_revision(), i32::MAX as u64);
        assert_eq!(state.try_read().unwrap().map.revision, 0);
        assert!(matches!(
            map_revision_rx.try_recv(),
            Err(broadcast::error::TryRecvError::Empty)
        ));
    }

    #[test]
    fn u64_max_shared_floor_cannot_advance_local_revision_or_fanout() {
        let mut adapter = active_adapter(sample_config());
        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0)]);
        let mut shared = SharedState::default();
        shared.map.revision = u64::MAX;
        let state = Arc::new(RwLock::new(shared));
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(1);

        assert!(!adapter.build_and_publish(&state, Some(&map_revision_tx)));
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(adapter.global_revision(), 0);
        assert_eq!(state.try_read().unwrap().map.revision, u64::MAX);
        assert!(matches!(
            map_revision_rx.try_recv(),
            Err(broadcast::error::TryRecvError::Empty)
        ));
    }

    #[test]
    fn reset_clears_accumulator() {
        let mut a = active_adapter(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(a.has_data());
        a.reset_accumulator();
        assert!(!a.has_data());
        assert_eq!(a.accumulated_count(), 0);
    }

    #[test]
    fn mapping_lifecycle_keeps_global_revision_monotonic_across_sessions() {
        let mut adapter = MappingAdapter::new(sample_config());
        let first = adapter
            .start_session("session-1", "warehouse", "mapver_000001")
            .expect("first start");
        assert_eq!(first.session_generation, 1);
        assert!(first.active);

        let replay = adapter
            .start_session("session-1", "warehouse", "mapver_000001")
            .expect("idempotent start");
        assert!(replay.idempotent_replay);
        assert!(matches!(
            adapter.start_session("session-2", "warehouse", "mapver_000002"),
            Err(MappingLifecycleError::Conflict(_))
        ));

        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0)]);
        let first_state = Arc::new(RwLock::new(SharedState::default()));
        assert!(adapter.build_and_publish(&first_state, None));
        assert_eq!(adapter.current_revision(), 1);
        assert_eq!(adapter.global_revision(), 1);

        let stopped = adapter.stop_session("session-1").expect("exact stop");
        assert_eq!(stopped.phase, MappingSessionPhase::Stopped);
        let count_at_stop = adapter.accumulated_count();
        adapter.feed_pointcloud(&[Point3::new(1.0, 0.0, 2.0)]);
        assert_eq!(adapter.accumulated_count(), count_at_stop);
        assert!(!adapter.build_and_publish(&first_state, None));
        assert!(
            adapter
                .stop_session("session-1")
                .expect("idempotent stop")
                .idempotent_replay
        );

        let second = adapter
            .start_session("session-2", "warehouse", "mapver_000002")
            .expect("second start");
        assert_eq!(second.session_generation, 2);
        assert_eq!(second.revision, 0);
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(adapter.global_revision(), 1);
        assert_eq!(adapter.accumulated_count(), 0);
        assert_eq!(adapter.odom_count(), 0);
        assert!(matches!(
            adapter.stop_session("session-1"),
            Err(MappingLifecycleError::StaleSession(_))
        ));
        assert!(
            adapter.is_active(),
            "stale stop must not kill the new session"
        );

        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0)]);
        let second_state = Arc::new(RwLock::new(SharedState::default()));
        assert!(adapter.build_and_publish(&second_state, None));
        assert_eq!(adapter.current_revision(), 2);
        assert_eq!(adapter.global_revision(), 2);
        assert_eq!(second_state.try_read().expect("second map").map.revision, 2);
    }

    #[test]
    fn mapping_revision_is_durably_newer_after_geometry_process_restart() {
        let path = sequence_test_path("process-restart");
        let first_store = Arc::new(SequenceStateStore::open(&path).expect("first store"));
        let mut first_adapter = active_persistent_adapter(first_store.clone());
        first_adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 2.0)]);
        let first_state = Arc::new(RwLock::new(SharedState::default()));
        assert!(first_adapter.build_and_publish(&first_state, None));
        let first_revision = first_adapter.current_revision();
        assert_eq!(first_revision, 1);
        drop(first_adapter);
        drop(first_store);

        let restarted_store =
            Arc::new(SequenceStateStore::open(&path).expect("restarted geometry sequence store"));
        let mut restarted_adapter = active_persistent_adapter(restarted_store);
        assert_eq!(restarted_adapter.current_revision(), 0);
        assert_eq!(restarted_adapter.global_revision(), first_revision);
        restarted_adapter.feed_pointcloud(&[Point3::new(1.0, 0.0, 2.0)]);
        let restarted_state = Arc::new(RwLock::new(SharedState::default()));
        assert!(restarted_adapter.build_and_publish(&restarted_state, None));
        assert!(restarted_adapter.current_revision() > first_revision);
        assert_eq!(
            restarted_state.try_read().unwrap().map.revision,
            first_revision + 1
        );
    }

    #[test]
    fn legacy_feed_accepts_more_than_one_unstamped_frame() {
        let mut adapter = active_adapter(sample_config());
        adapter.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        adapter.feed_pointcloud(&[Point3::new(1.0, 0.0, 0.0)]);

        assert_eq!(adapter.accumulated_count(), 2);
        assert_eq!(adapter.pointcloud_count(), 2);
    }

    #[test]
    fn session_freshness_uses_source_message_time_not_receive_time() {
        let mut adapter = active_adapter(sample_config());
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("odom anchor");
        adapter
            .feed_pointcloud_session(
                &[Point3::new(0.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                10_000_000_000,
                "odom",
                1_000_000_000,
            )
            .expect("first source frame accepted");

        let error = adapter
            .feed_pointcloud_session(
                &[Point3::new(1.0, 0.0, 0.0)],
                (1.0, 0.0),
                16,
                10_100_000_000,
                "odom",
                700_000_000,
            )
            .expect_err("source frame 300ms behind the watermark must be stale");

        assert!(matches!(
            error,
            nuwax_mapping::SessionError::StaleRevision { .. }
        ));
    }

    #[test]
    fn odom_pointcloud_before_pose_is_rejected_without_mutation() {
        let mut adapter = active_adapter(sample_config());
        let error = adapter
            .feed_pointcloud_session(
                &[Point3::new(0.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                1_000_000_000,
                "odom",
                1_000_000_000,
            )
            .expect_err("odom pointcloud must wait for the session anchor");

        assert!(matches!(
            error,
            nuwax_mapping::SessionError::FrameTransformUnavailable { .. }
        ));
        assert!(!adapter.has_data());
        assert_eq!(adapter.accumulated_count(), 0);
        assert_eq!(adapter.pointcloud_count(), 0);
    }

    #[test]
    fn nonzero_map_odom_anchor_transforms_points_and_pose_before_publish() {
        let mut adapter = active_adapter(sample_config());
        let initial = adapter
            .feed_pose_in_frame(10.0, 5.0, 2.0, std::f64::consts::FRAC_PI_2, "odom")
            .expect("nonzero odom anchor");
        assert_eq!(initial.position, [0.0, 0.0, 0.0]);
        assert_close(initial.yaw_radians, 0.0);

        adapter
            .feed_pointcloud_session(
                &[Point3::new(10.0, 6.0, 3.0)],
                (10.0, 5.0),
                16,
                1_000_000_000,
                "odom",
                1_000_000_000,
            )
            .expect("anchored odom pointcloud");
        let points = adapter.session.clone_points();
        assert_eq!(points.len(), 1);
        assert_close(points[0].x, 1.0);
        assert_close(points[0].y, 0.0);
        assert_close(points[0].z, 1.0);

        let moved = adapter
            .feed_pose_in_frame(11.0, 5.0, 2.0, std::f64::consts::FRAC_PI_2 + 0.25, "odom")
            .expect("subsequent odom pose");
        assert_close(moved.position[0], 0.0);
        assert_close(moved.position[1], -1.0);
        assert_close(moved.position[2], 0.0);
        assert_close(moved.yaw_radians, 0.25);

        let contract = adapter.frame_contract("map");
        assert_eq!(contract.source_frame_id, "odom");
        assert_eq!(contract.output_frame_id, "map");
        assert_eq!(contract.expected_map_frame_id, "map");
        assert!(contract.transform_required);
        assert!(contract.transform_applied);
        assert!(contract.ready_for_global_consumers());

        let transform = adapter
            .map_odom_transform_snapshot()
            .expect("read-only transform snapshot");
        assert_close(transform.translation[0], -5.0);
        assert_close(transform.translation[1], 10.0);
        assert_close(transform.translation[2], -2.0);
        assert_close(transform.yaw_radians, -std::f64::consts::FRAC_PI_2);
        assert_eq!(transform.generation, 1);
        assert_close(transform.latest_map_pose.position[1], -1.0);

        let state = Arc::new(RwLock::new(SharedState::default()));
        assert!(adapter.build_and_publish(&state, None));
        assert_eq!(state.try_read().expect("map state").map.frame_id, "map");
    }

    #[test]
    fn map_frame_pointcloud_is_passed_through_without_transform() {
        let mut adapter = active_adapter(sample_config());
        let source = Point3::new(3.5, -2.0, 0.75);
        adapter
            .feed_pointcloud_session(
                &[source],
                (3.5, -2.0),
                16,
                1_000_000_000,
                "/map",
                1_000_000_000,
            )
            .expect("map pointcloud is already global");

        assert_eq!(adapter.session.clone_points(), vec![source]);
        assert!(adapter.map_odom_transform_snapshot().is_none());
        let contract = adapter.frame_contract("map");
        assert_eq!(contract.source_frame_id, "map");
        assert_eq!(contract.output_frame_id, "map");
        assert!(!contract.transform_required);
        assert!(!contract.transform_applied);
        assert!(contract.ready_for_global_consumers());
    }

    #[test]
    fn unknown_sensor_frame_is_rejected() {
        let mut adapter = active_adapter(sample_config());
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("odom anchor");
        let error = adapter
            .feed_pointcloud_session(
                &[Point3::new(1.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                1_000_000_000,
                "lidar_link",
                1_000_000_000,
            )
            .expect_err("sensor frame needs an explicit transform");
        assert!(matches!(
            error,
            nuwax_mapping::SessionError::FrameTransformUnavailable { .. }
        ));
        assert_eq!(adapter.accumulated_count(), 0);
    }

    #[test]
    fn reset_clears_anchor_and_reanchors_with_a_new_generation() {
        let mut adapter = active_adapter(sample_config());
        adapter
            .feed_pose_in_frame(4.0, 2.0, 0.0, 0.0, "odom")
            .expect("first anchor");
        let first_generation = adapter.global_frame_generation();
        adapter
            .feed_pointcloud_session(
                &[Point3::new(5.0, 2.0, 0.0)],
                (4.0, 2.0),
                16,
                1_000_000_000,
                "odom",
                1_000_000_000,
            )
            .expect("first generation points");
        assert_close(adapter.session.clone_points()[0].x, 1.0);

        adapter.reset_accumulator();
        assert!(adapter.map_odom_transform_snapshot().is_none());
        assert_eq!(adapter.accumulated_count(), 0);
        let error = adapter
            .feed_pointcloud_session(
                &[Point3::new(-3.0, 5.0, 0.0)],
                (-3.0, 4.0),
                16,
                2_000_000_000,
                "odom",
                2_000_000_000,
            )
            .expect_err("reset requires a new pose anchor");
        assert!(matches!(
            error,
            nuwax_mapping::SessionError::FrameTransformUnavailable { .. }
        ));

        adapter
            .feed_pose_in_frame(-3.0, 4.0, 0.0, std::f64::consts::FRAC_PI_2, "odom")
            .expect("second anchor");
        let second_generation = adapter.global_frame_generation();
        assert!(second_generation > first_generation);
        adapter
            .feed_pointcloud_session(
                &[Point3::new(-3.0, 5.0, 0.0)],
                (-3.0, 4.0),
                16,
                2_000_000_000,
                "odom",
                2_000_000_000,
            )
            .expect("second generation points");
        let transformed = adapter.session.clone_points();
        assert_close(transformed[0].x, 1.0);
        assert_close(transformed[0].y, 0.0);
    }

    #[test]
    fn build_and_publish_does_not_rebuild_every_high_rate_frame() {
        let mut adapter = active_adapter(sample_config());
        let state = Arc::new(RwLock::new(SharedState::default()));
        adapter
            .feed_pointcloud_session(
                &[Point3::new(0.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                1_000_000_000,
                "map",
                1_000_000_000,
            )
            .expect("first frame accepted");
        assert!(adapter.build_and_publish(&state, None));

        adapter
            .feed_pointcloud_session(
                &[Point3::new(1.0, 0.0, 0.0)],
                (1.0, 0.0),
                16,
                1_100_000_000,
                "map",
                1_100_000_000,
            )
            .expect("second frame accepted");
        assert!(
            !adapter.build_and_publish(&state, None),
            "100ms high-rate updates should be coalesced into the next bounded rebuild"
        );
        assert_eq!(adapter.current_revision(), 1);
    }

    #[test]
    fn rebuild_cadence_uses_receive_time_when_source_timestamp_is_frozen() {
        let mut adapter = active_adapter(sample_config());
        let state = Arc::new(RwLock::new(SharedState::default()));
        adapter
            .feed_pointcloud_session(
                &[Point3::new(0.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                10_000_000_000,
                "map",
                1_000_000_000,
            )
            .expect("first frame accepted");
        assert!(adapter.build_and_publish(&state, None));

        adapter
            .feed_pointcloud_session(
                &[Point3::new(1.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                11_000_000_000,
                "map",
                1_000_000_000,
            )
            .expect("different content with the same source time is not a replay");
        assert!(adapter.build_and_publish(&state, None));
        assert_eq!(adapter.current_revision(), 2);
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

    // ~~cost_mapper_config_is_nav2_default~~ —— W41 删除:
    // `CostMapperConfig` 已在 nuwax-mapping::cost_mapper 重构为 selector
    // 风格(`OccupancyConfig { algo, initial_safe_radius_meters }`),原
    // `free_space_cost` / `lethal_cost` / `no_information_cost` 字段全部
    // 移至 `Nav2CostLayer`(独立类型,属 Nav2 适配层不在 mapping 接口)。
    // 该单测基于废弃字段硬编码,继续保留会锁死旧结构;由 RALPLAN §Phase 7
    // Nav2 适配层 `Nav2CostLayer` 测试覆盖。

    // ========================================================================
    // Phase 7:counters + readiness 综合
    // ========================================================================

    #[test]
    fn counters_start_at_zero() {
        let a = active_adapter(sample_config());
        assert_eq!(a.odom_count(), 0);
        assert_eq!(a.pointcloud_count(), 0);
        assert_eq!(a.current_revision(), 0);
    }

    #[test]
    fn readiness_initial_state_unready() {
        // adapter 未构造 → unready,无 pointcloud
        let r = MappingReadiness::compute(None, true);
        assert!(!r.ready);
        assert!(!r.adapter_present);
        assert!(r.lcm_backend); // 仅覆盖字段本身,期望 backend true
        assert_eq!(r.odom_count, 0);
        assert_eq!(r.pointcloud_count, 0);
        assert!(r.last_error.is_some());
    }

    #[test]
    fn readiness_with_only_odom_is_unready() {
        let mut a = active_adapter(sample_config());
        a.feed_pose(0.0, 0.0, 0.0);
        let r = MappingReadiness::compute(Some(&a), true);
        assert!(!r.ready, "缺 pointcloud 必须 unready");
        assert_eq!(r.odom_count, 1);
        assert_eq!(r.pointcloud_count, 0);
        assert!(r.last_error.as_ref().unwrap().contains("pointcloud"));
    }

    #[test]
    fn readiness_with_pointcloud_turns_ready() {
        let mut a = active_adapter(sample_config());
        let state = Arc::new(RwLock::new(SharedState::default()));
        a.feed_pose(0.0, 0.0, 0.0);
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        assert!(a.build_and_publish(&state, None));
        let r = MappingReadiness::compute(Some(&a), true);
        assert!(r.ready);
        assert_eq!(r.odom_count, 1);
        assert_eq!(r.pointcloud_count, 1);
        assert_eq!(r.last_occupancy_revision, 1);
        assert_eq!(r.accumulated_point_count, 1);
    }

    #[test]
    fn readiness_no_backend_is_unready() {
        let mut a = active_adapter(sample_config());
        a.feed_pointcloud(&[Point3::new(0.0, 0.0, 0.0)]);
        let r = MappingReadiness::compute(Some(&a), false);
        assert!(!r.ready);
        assert!(!r.lcm_backend);
        assert!(r.last_error.as_ref().unwrap().contains("backend"));
    }

    #[test]
    fn installed_checkpoint_is_ready_without_fake_pointcloud_input() {
        let mut adapter = MappingAdapter::new(sample_config());
        adapter
            .start_session("mapload:one", "warehouse", "mapver_000001")
            .expect("start checkpoint session");
        let mut checkpoint = OccupancyGrid::with_frame(2, 2, 0.05, "map");
        checkpoint.origin = [-1.0, 2.0, 0.0];
        checkpoint.data = vec![-1, 0, 100, 50];

        let (map, event) = adapter
            .install_checkpoint(checkpoint, 0)
            .expect("install checkpoint");

        assert_eq!(map.data, vec![-1, 0, 100, 50]);
        assert_eq!(event.map_version_id, "mapver_000001");
        assert_eq!(event.session_id, "mapload:one");
        assert_eq!(event.revision, 1);
        assert_eq!(adapter.pointcloud_count(), 0);
        let readiness = MappingReadiness::compute(Some(&adapter), true);
        assert!(readiness.ready);
        assert!(readiness.global_consumer_ready);
    }

    #[test]
    fn live_grid_merges_with_loaded_checkpoint_in_world_coordinates() {
        let mut checkpoint = OccupancyGrid::with_frame(2, 2, 0.5, "map");
        checkpoint.origin = [-0.5, -0.5, 0.0];
        checkpoint.data = vec![100, 0, -1, 0];
        let mut live = OccupancyGrid::with_frame(2, 2, 0.5, "map");
        live.origin = [0.0, -0.5, 0.0];
        live.data = vec![0, 100, 0, -1];

        let merged = merge_checkpoint_grid(&checkpoint, &live).expect("merge aligned grids");

        assert_eq!(merged.width, 3);
        assert_eq!(merged.height, 2);
        assert_eq!(merged.origin, [-0.5, -0.5, 0.0]);
        assert_eq!(merged.data, vec![100, 0, 100, -1, 0, -1]);
    }

    #[test]
    fn live_grid_with_non_aligned_origin_projects_onto_checkpoint_lattice() {
        let checkpoint_resolution = 0.05000000074505806;
        let mut checkpoint = OccupancyGrid::with_frame(2, 1, checkpoint_resolution, "map");
        checkpoint.origin = [-0.0977304454380725, 0.0, 0.0];
        checkpoint.data = vec![100, 0];
        let mut live = OccupancyGrid::with_frame(1, 1, 0.05, "map");
        live.origin = [0.0, 0.0, 0.0];
        live.data = vec![100];

        let merged = merge_checkpoint_grid(&checkpoint, &live)
            .expect("fractional world origins must be projected onto one grid");

        assert_eq!(merged.width, 3);
        assert_eq!(merged.height, 1);
        assert_eq!(merged.origin, checkpoint.origin);
        assert_eq!(merged.data, vec![100, 0, 100]);
    }

    #[test]
    fn loaded_checkpoint_commits_live_pointcloud_revision() {
        let mut adapter = active_adapter(sample_config());
        let mut checkpoint = OccupancyGrid::with_frame(2, 1, 0.1, "map");
        checkpoint.origin = [-0.0977304454380725, 0.0, 0.0];
        checkpoint.data = vec![100, 0];
        let (checkpoint_map, _) = adapter
            .install_checkpoint(checkpoint, 0)
            .expect("install checkpoint");
        let state = Arc::new(RwLock::new(SharedState::default()));
        state.blocking_write().map = checkpoint_map;
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(1);

        adapter
            .feed_pointcloud_session(
                &[Point3::new(0.0, 0.0, 2.0)],
                (0.0, 0.0),
                16,
                1_000_000_000,
                "map",
                1_000_000_000,
            )
            .expect("live pointcloud accepted");

        assert!(adapter.build_and_publish(&state, Some(&map_revision_tx)));
        let event = map_revision_rx
            .try_recv()
            .expect("live map revision broadcast");
        assert!(event.revision > 1);
        assert_eq!(state.blocking_read().map.revision, event.revision);
    }
}
