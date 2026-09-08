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

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use nalgebra::Point3;
#[cfg(test)]
use nuwax_mapping::cost_mapper::SimplePointCloud2;
use nuwax_mapping::cost_mapper::{
    CostAlgoSelector, CostMapperConfig, OccupancyConfig, PointCloud2Like,
};
#[cfg(test)]
use nuwax_mapping::pointclouds::occupancy::MappingError;
use nuwax_mapping::pointclouds::occupancy::PointCloudOccConfig;
use nuwax_mapping::{CostMapper, MappingSession, OccupancyGrid, PointCloudAccumulator};
use thiserror::Error;
use tokio::sync::{broadcast, RwLock};
use tracing::warn;

#[allow(unused_imports)]
use nuwax_mapping::SessionStats as MappingSessionStats;

use crate::lcm_runtime::{MapRevisionEvent, MapState, SharedState};
#[cfg(test)]
use crate::mapping_backend::{
    join_retained_mapping_backend_thread_for_tests,
    mapping_backend_process_exit_retains_thread_for_tests, MappingBackendRetirementTestGate,
    MappingBackendTestGate, MappingKernel,
};
use crate::mapping_backend::{
    MappingBackendCandidate, MappingBackendClient, MappingBackendCommitHandle,
    MappingBackendCompletion, MappingBackendComputeConfig, MappingBackendInput,
    MappingBackendJoinHandle, MappingBackendKind, MappingBackendSpec, MappingBackendWorker,
    MappingComputeToken,
};
use crate::mapping_commit::{MappingCommitClock, OccupancyCommitReceipt};
#[cfg(test)]
use crate::persistent_occupancy::{snap_grid_index, OCCUPANCY_CHANGE_CONFIRMATIONS};
use crate::persistent_occupancy::{validate_checkpoint_grid, PersistentOccupancyGrid};
use crate::sequence_state::SequenceStateStore;

const MIN_REBUILD_INTERVAL_NS: u64 = 250_000_000;
const MAX_REBUILD_INTERVAL_NS: u64 = 1_000_000_000;
const REBUILD_GROWTH_DIVISOR: usize = 20;
#[cfg(not(test))]
const MAPPING_FINAL_ATTEMPT_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const MAPPING_FINAL_ATTEMPT_TIMEOUT: Duration = Duration::from_millis(50);
const MAPPING_FINAL_JOIN_POLL_INTERVAL: Duration = Duration::from_millis(1);

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
    pub session_generation: u64,
    pub revision: u64,
    pub active: bool,
    pub idempotent_replay: bool,
    pub legacy_checkpoint_reconstructed: bool,
    pub mapping_backend_kind: String,
}

impl MappingSessionSnapshot {
    fn idle(mapping_backend_kind: MappingBackendKind) -> Self {
        Self {
            phase: MappingSessionPhase::Idle,
            session_id: String::new(),
            map_name: String::new(),
            session_generation: 0,
            revision: 0,
            active: false,
            idempotent_replay: false,
            legacy_checkpoint_reconstructed: false,
            mapping_backend_kind: mapping_backend_kind.stable_name().to_owned(),
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
    #[error("mapping backend lifecycle failed: {0}")]
    Backend(String),
}

/// Exact forbidden-zone snapshot currently composed into the effective map.
///
/// `applied_revision` is the first canonical occupancy revision that contains
/// `generation`/`signature`; later physical-map rebuilds may advance the map
/// revision without changing this acknowledgement.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(crate) struct ForbiddenZoneAck {
    pub map_name: String,
    pub generation: u64,
    pub signature: String,
    pub applied_revision: u64,
}

/// Read-only result of registering a raw odom-frame cloud into the canonical map frame.
///
/// This seam performs only frame registration. The caller remains responsible for
/// handing the returned points to the canonical mapping ingestion path exactly once.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RegisteredMapCloud {
    pub(crate) points: Vec<Point3<f64>>,
    pub(crate) session_generation: u64,
    pub(crate) frame_generation: u64,
}

#[derive(Debug)]
pub(crate) struct PreparedMappingCompute {
    pub(crate) worker: MappingBackendClient,
    pub(crate) token: MappingComputeToken,
    pub(crate) input: MappingBackendInput,
}

#[derive(Debug)]
pub(crate) struct PreparedMappingFlush {
    pub(crate) worker: MappingBackendClient,
    pub(crate) token: MappingComputeToken,
}

#[derive(Debug)]
pub(crate) struct CommittedMappingCandidate {
    pub(crate) token: MappingComputeToken,
    pub(crate) resolution: MappingBackendCommitHandle,
    pub(crate) accepted_map_points: Vec<Point3<f64>>,
    pub(crate) event: Option<MapRevisionEvent>,
    pub(crate) session_generation: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) backend_kind: MappingBackendKind,
    pub(crate) backend_generation: u64,
    pub(crate) occupancy_receipt: Option<OccupancyCommitReceipt>,
}

#[derive(Debug)]
pub(crate) struct MappingBackendRetirement {
    generation: u64,
    acknowledge_pending: bool,
    worker: MappingBackendWorker,
}

impl MappingBackendRetirement {
    pub(crate) const fn generation(&self) -> u64 {
        self.generation
    }

    const fn acknowledge_pending(&self) -> bool {
        self.acknowledge_pending
    }

    pub(crate) fn stop_and_join(self) -> Result<(), String> {
        self.worker.into_stop_join_handle()?.join()
    }

    fn into_stop_join_handle(
        self,
    ) -> Result<(u64, bool, MappingBackendJoinHandle), MappingTerminalError> {
        let Self {
            generation,
            acknowledge_pending,
            worker,
        } = self;
        let join_handle = worker
            .into_stop_join_handle()
            .map_err(MappingTerminalError::new)?;
        Ok((generation, acknowledge_pending, join_handle))
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct ShutdownFailures {
    failures: Vec<String>,
}

impl ShutdownFailures {
    pub(crate) fn record(&mut self, failure: impl Into<String>) {
        let failure = failure.into();
        if !self.failures.iter().any(|existing| existing == &failure) {
            self.failures.push(failure);
        }
    }

    pub(crate) fn detail(&self) -> Option<String> {
        (!self.failures.is_empty()).then(|| self.failures.join("; "))
    }

    fn into_terminal(
        mut self,
        joined_terminal: Result<(), MappingTerminalError>,
    ) -> Result<(), MappingTerminalError> {
        if let Err(error) = joined_terminal {
            self.record(error.to_string());
        }
        match self.detail() {
            Some(detail) => Err(MappingTerminalError::new(detail)),
            None => Ok(()),
        }
    }
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
#[error("{detail}")]
pub(crate) struct MappingTerminalError {
    detail: String,
}

impl MappingTerminalError {
    fn new(detail: impl Into<String>) -> Self {
        Self {
            detail: detail.into(),
        }
    }
}

#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub(crate) enum MappingPendingCause {
    #[error("timed out acquiring the Mapping lifecycle lock")]
    LifecycleLockTimeout,
    #[error("timed out joining Mapping backend generation {generation}")]
    BackendJoinTimeout { generation: u64 },
    #[error("timed out acquiring the Mapping retirement ACK lock for generation {generation}")]
    RetirementAckLockTimeout { generation: u64 },
    #[error(
        "Mapping retirement acknowledgement generation mismatch: expected={expected} pending={pending:?}"
    )]
    GenerationMismatch { expected: u64, pending: Option<u64> },
    #[error("Mapping retirement could not be proven: {0}")]
    RetirementUnproven(String),
}

#[derive(Debug)]
enum MappingRetirementProof {
    NoBackend,
    QuiescentGeneration(u64),
    ExactGeneration(u64),
}

#[derive(Debug)]
pub(crate) struct MappingRetiredCapability {
    proof: MappingRetirementProof,
}

impl MappingRetiredCapability {
    fn no_backend() -> Self {
        Self {
            proof: MappingRetirementProof::NoBackend,
        }
    }

    fn quiescent(generation: u64) -> Self {
        Self {
            proof: MappingRetirementProof::QuiescentGeneration(generation),
        }
    }

    fn exact(generation: u64) -> Self {
        Self {
            proof: MappingRetirementProof::ExactGeneration(generation),
        }
    }

    #[cfg(test)]
    fn generation(&self) -> Option<u64> {
        match self.proof {
            MappingRetirementProof::NoBackend => None,
            MappingRetirementProof::QuiescentGeneration(generation)
            | MappingRetirementProof::ExactGeneration(generation) => Some(generation),
        }
    }
}

#[allow(clippy::enum_variant_names)]
pub(crate) enum MappingFinalOwner {
    WaitingLifecycleLock {
        adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>,
        lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    },
    WaitingBackendJoin {
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
        generation: u64,
        acknowledge_pending: bool,
        join_handle: MappingBackendJoinHandle,
    },
    WaitingRetirementAck {
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
        generation: u64,
        acknowledge_pending: bool,
        joined_terminal: Result<(), MappingTerminalError>,
    },
}

impl std::fmt::Debug for MappingFinalOwner {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WaitingLifecycleLock { .. } => formatter
                .debug_struct("WaitingLifecycleLock")
                .finish_non_exhaustive(),
            Self::WaitingBackendJoin { generation, .. } => formatter
                .debug_struct("WaitingBackendJoin")
                .field("generation", generation)
                .finish_non_exhaustive(),
            Self::WaitingRetirementAck {
                generation,
                joined_terminal,
                ..
            } => formatter
                .debug_struct("WaitingRetirementAck")
                .field("generation", generation)
                .field("joined_terminal", joined_terminal)
                .finish_non_exhaustive(),
        }
    }
}

pub(crate) enum MappingFinalAttempt {
    Pending {
        owner: MappingFinalOwner,
        cause: MappingPendingCause,
        sticky: ShutdownFailures,
    },
    Settled {
        capability: MappingRetiredCapability,
        terminal: Result<(), MappingTerminalError>,
    },
}

enum MappingFinalPreparation {
    Pending(MappingPendingCause),
    Settled(MappingRetiredCapability),
    Retirement {
        retirement: MappingBackendRetirement,
        sticky_error: Option<MappingTerminalError>,
    },
}

impl MappingFinalOwner {
    pub(crate) fn waiting_for_lifecycle_lock(
        adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>,
        lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    ) -> Self {
        Self::WaitingLifecycleLock {
            adapter,
            lifecycle_lock,
        }
    }

    fn from_retirement(
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
        retirement: MappingBackendRetirement,
    ) -> Result<Self, MappingTerminalError> {
        let (generation, acknowledge_pending, join_handle) = retirement.into_stop_join_handle()?;
        Ok(Self::WaitingBackendJoin {
            adapter,
            lifecycle_lock,
            generation,
            acknowledge_pending,
            join_handle,
        })
    }

    pub(crate) async fn attempt(self, mut sticky: ShutdownFailures) -> MappingFinalAttempt {
        let mut owner = self;
        loop {
            owner = match owner {
                Self::WaitingLifecycleLock {
                    adapter,
                    lifecycle_lock,
                } => {
                    let lifecycle_guard = match tokio::time::timeout(
                        MAPPING_FINAL_ATTEMPT_TIMEOUT,
                        Arc::clone(&lifecycle_lock).lock_owned(),
                    )
                    .await
                    {
                        Ok(guard) => guard,
                        Err(_) => {
                            return MappingFinalAttempt::Pending {
                                owner: Self::WaitingLifecycleLock {
                                    adapter,
                                    lifecycle_lock,
                                },
                                cause: MappingPendingCause::LifecycleLockTimeout,
                                sticky,
                            };
                        }
                    };
                    let Some(adapter) = adapter else {
                        drop(lifecycle_guard);
                        return MappingFinalAttempt::Settled {
                            capability: MappingRetiredCapability::no_backend(),
                            terminal: sticky.into_terminal(Ok(())),
                        };
                    };
                    let preparation = adapter.lock().prepare_final_backend_retirement();
                    drop(lifecycle_guard);
                    match preparation {
                        MappingFinalPreparation::Pending(cause) => {
                            return MappingFinalAttempt::Pending {
                                owner: Self::WaitingLifecycleLock {
                                    adapter: Some(adapter),
                                    lifecycle_lock,
                                },
                                cause,
                                sticky,
                            };
                        }
                        MappingFinalPreparation::Settled(capability) => {
                            return MappingFinalAttempt::Settled {
                                capability,
                                terminal: sticky.into_terminal(Ok(())),
                            };
                        }
                        MappingFinalPreparation::Retirement {
                            retirement,
                            sticky_error,
                        } => {
                            if let Some(error) = sticky_error {
                                sticky.record(error.to_string());
                            }
                            match Self::from_retirement(
                                Arc::clone(&adapter),
                                Arc::clone(&lifecycle_lock),
                                retirement,
                            ) {
                                Ok(owner) => owner,
                                Err(error) => {
                                    sticky.record(error.to_string());
                                    return MappingFinalAttempt::Pending {
                                        owner: Self::WaitingLifecycleLock {
                                            adapter: Some(adapter),
                                            lifecycle_lock,
                                        },
                                        cause: MappingPendingCause::RetirementUnproven(
                                            error.to_string(),
                                        ),
                                        sticky,
                                    };
                                }
                            }
                        }
                    }
                }
                Self::WaitingBackendJoin {
                    adapter,
                    lifecycle_lock,
                    generation,
                    acknowledge_pending,
                    join_handle,
                } => {
                    let deadline = Instant::now() + MAPPING_FINAL_ATTEMPT_TIMEOUT;
                    while !join_handle.is_finished() {
                        let remaining = deadline.saturating_duration_since(Instant::now());
                        if remaining.is_zero() {
                            return MappingFinalAttempt::Pending {
                                owner: Self::WaitingBackendJoin {
                                    adapter,
                                    lifecycle_lock,
                                    generation,
                                    acknowledge_pending,
                                    join_handle,
                                },
                                cause: MappingPendingCause::BackendJoinTimeout { generation },
                                sticky,
                            };
                        }
                        tokio::time::sleep(MAPPING_FINAL_JOIN_POLL_INTERVAL.min(remaining)).await;
                    }
                    let joined_terminal = join_handle.join().map_err(MappingTerminalError::new);
                    Self::WaitingRetirementAck {
                        adapter,
                        lifecycle_lock,
                        generation,
                        acknowledge_pending,
                        joined_terminal,
                    }
                }
                Self::WaitingRetirementAck {
                    adapter,
                    lifecycle_lock,
                    generation,
                    acknowledge_pending,
                    joined_terminal,
                } => {
                    if !acknowledge_pending {
                        return MappingFinalAttempt::Pending {
                            owner: Self::WaitingRetirementAck {
                                adapter,
                                lifecycle_lock,
                                generation,
                                acknowledge_pending,
                                joined_terminal,
                            },
                            cause: MappingPendingCause::RetirementUnproven(format!(
                                "generation {generation} is owned by another retirement token"
                            )),
                            sticky,
                        };
                    }
                    let lifecycle_guard = match tokio::time::timeout(
                        MAPPING_FINAL_ATTEMPT_TIMEOUT,
                        Arc::clone(&lifecycle_lock).lock_owned(),
                    )
                    .await
                    {
                        Ok(guard) => guard,
                        Err(_) => {
                            return MappingFinalAttempt::Pending {
                                owner: Self::WaitingRetirementAck {
                                    adapter,
                                    lifecycle_lock,
                                    generation,
                                    acknowledge_pending,
                                    joined_terminal,
                                },
                                cause: MappingPendingCause::RetirementAckLockTimeout { generation },
                                sticky,
                            };
                        }
                    };
                    let (acknowledgement, pending) = {
                        let mut adapter_guard = adapter.lock();
                        let acknowledgement =
                            adapter_guard.acknowledge_backend_retirement(generation);
                        (acknowledgement, adapter_guard.pending_backend_retirement)
                    };
                    drop(lifecycle_guard);
                    match acknowledgement {
                        Ok(capability) => {
                            return MappingFinalAttempt::Settled {
                                capability,
                                terminal: sticky.into_terminal(joined_terminal),
                            };
                        }
                        Err(error) => {
                            sticky.record(error.to_string());
                            return MappingFinalAttempt::Pending {
                                owner: Self::WaitingRetirementAck {
                                    adapter,
                                    lifecycle_lock,
                                    generation,
                                    acknowledge_pending,
                                    joined_terminal,
                                },
                                cause: MappingPendingCause::GenerationMismatch {
                                    expected: generation,
                                    pending,
                                },
                                sticky,
                            };
                        }
                    }
                }
            };
        }
    }

    #[cfg(test)]
    fn state_name(&self) -> &'static str {
        match self {
            Self::WaitingLifecycleLock { .. } => "waiting_lifecycle_lock",
            Self::WaitingBackendJoin { .. } => "waiting_backend_join",
            Self::WaitingRetirementAck { .. } => "waiting_retirement_ack",
        }
    }

    #[cfg(test)]
    fn join_thread_id(&self) -> Option<std::thread::ThreadId> {
        match self {
            Self::WaitingBackendJoin { join_handle, .. } => Some(join_handle.thread_id()),
            _ => None,
        }
    }

    #[cfg(test)]
    fn joined_terminal(&self) -> Option<&Result<(), MappingTerminalError>> {
        match self {
            Self::WaitingRetirementAck {
                joined_terminal, ..
            } => Some(joined_terminal),
            _ => None,
        }
    }
}

pub(crate) async fn finish_backend_retirement(
    adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    retirement: Option<MappingBackendRetirement>,
) -> Result<(), MappingLifecycleError> {
    let Some(retirement) = retirement else {
        return Ok(());
    };
    let owner =
        spawn_backend_retirement_owner(Arc::clone(adapter), Arc::clone(lifecycle_lock), retirement);
    owner.await.map_err(|error| {
        MappingLifecycleError::Backend(format!(
            "mapping backend retirement owner task failed: {error}"
        ))
    })?
}

fn spawn_backend_retirement_owner(
    adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    retirement: MappingBackendRetirement,
) -> tokio::task::JoinHandle<Result<(), MappingLifecycleError>> {
    tokio::spawn(async move {
        let generation = retirement.generation();
        let acknowledge_pending = retirement.acknowledge_pending();
        let result = tokio::task::spawn_blocking(move || retirement.stop_and_join())
            .await
            .map_err(|error| {
                MappingLifecycleError::Backend(format!(
                    "mapping backend retirement task failed: {error}"
                ))
            })?;
        let _lifecycle_guard = lifecycle_lock.lock().await;
        if acknowledge_pending {
            adapter
                .lock()
                .complete_backend_retirement(generation, result)
        } else {
            result.map_err(MappingLifecycleError::Backend)
        }
    })
}

#[derive(Debug)]
pub(crate) struct MappingStopTransition {
    pub(crate) snapshot: MappingSessionSnapshot,
    pub(crate) retirement: Option<MappingBackendRetirement>,
}

enum MappingSessionStartAdmission {
    Replay(MappingSessionSnapshot),
    Ready {
        session_id: String,
        map_name: String,
        session_generation: u64,
        backend_generation: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct MappingCheckpointExportLease {
    lease_id: u128,
    session_id: String,
    session_generation: u64,
    backend_generation: u64,
}

#[derive(Debug)]
struct ActiveMappingCheckpointExport {
    lease: MappingCheckpointExportLease,
    flush_token_id: Option<u128>,
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
    mapping_backend: MappingBackendSpec,
    #[cfg(test)]
    mapping_kernel: MappingKernel,
    mapping_worker: Option<MappingBackendWorker>,
    pending_backend_retirement: Option<u64>,
    backend_generation: u64,
    in_flight_compute: Option<MappingComputeToken>,
    checkpoint_export: Option<ActiveMappingCheckpointExport>,
    token_namespace: u128,
    next_token_sequence: u64,
    next_source_frame_sequence: u64,
    coalesced_frame_count: u64,
    stale_completion_count: u64,
    commit_clock: MappingCommitClock,
    last_occupancy_commit_receipt: Option<OccupancyCommitReceipt>,
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
    map_odom_transform_cache: Arc<parking_lot::RwLock<Option<MapOdomTransformSnapshot>>>,
    frame_generation: u64,
    global_frame_established: bool,
    transformed_points: Vec<Point3<f64>>,
    latest_accepted_map_points: Vec<Point3<f64>>,
    /// Number of newest points in `session` belonging to the last accepted
    /// frame.  It lets the occupancy builder obtain a latest-frame view from
    /// the rolling window without cloning the high-rate cloud payload.
    latest_accepted_point_count: usize,
    /// Stable 2D map state.  Unlike `session`, this grid never rolls old cells
    /// out merely because the bounded point budget is full.
    persistent_grid: Option<PersistentOccupancyGrid>,
    checkpoint_installed: bool,
    checkpoint_activation_digest: Option<[u8; 32]>,
    /// Optional production-only shared forbidden-zone document.
    forbidden_zones_path: Option<PathBuf>,
    /// Last physical occupancy grid before virtual forbidden zones are applied.
    raw_map: Option<MapState>,
    /// Signature of the zone set used to compose the current effective map.
    forbidden_zone_signature: String,
    /// Monotonic store generation represented by `forbidden_zone_signature`.
    forbidden_zone_generation: u64,
    /// First canonical occupancy revision composed with the current signature.
    forbidden_zone_applied_revision: u64,
    /// 与当前有效占据图同次应用的禁区快照；发布时不得重读未来文件。
    applied_forbidden_zones: Option<crate::forbidden_zones::LoadedForbiddenZones>,
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
        Self::new_with_frames_and_optional_sequence_store(
            config,
            MappingBackendSpec::Voxel,
            map_frame_id,
            odom_frame_id,
            None,
        )
    }

    #[cfg(test)]
    pub(crate) fn new_with_backend_for_tests(
        config: MappingAdapterConfig,
        mapping_backend: MappingBackendSpec,
        map_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
    ) -> Self {
        Self::new_with_frames_and_optional_sequence_store(
            config,
            mapping_backend,
            map_frame_id,
            odom_frame_id,
            None,
        )
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
            MappingBackendSpec::Voxel,
            map_frame_id,
            odom_frame_id,
            Some(sequence_store),
        )
    }

    /// Construct the production adapter with a shared map-scoped
    /// forbidden-zone document.  The existing constructor remains unchanged so
    /// legacy/unit callers do not acquire filesystem behavior implicitly.
    pub(crate) fn new_with_frames_sequence_store_and_forbidden_zones(
        config: MappingAdapterConfig,
        mapping_backend: MappingBackendSpec,
        map_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
        sequence_store: Arc<SequenceStateStore>,
        forbidden_zones_path: PathBuf,
    ) -> Self {
        let mut adapter = Self::new_with_frames_and_optional_sequence_store(
            config,
            mapping_backend,
            map_frame_id,
            odom_frame_id,
            Some(sequence_store),
        );
        adapter.forbidden_zones_path = Some(forbidden_zones_path);
        adapter
    }

    fn new_with_frames_and_optional_sequence_store(
        config: MappingAdapterConfig,
        mapping_backend: MappingBackendSpec,
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
        let session = MappingSession::new();
        #[cfg(test)]
        let session_capacity = session.capacity();
        Self {
            config,
            mapping_backend,
            #[cfg(test)]
            mapping_kernel: mapping_backend.into_kernel(session_capacity),
            mapping_worker: None,
            pending_backend_retirement: None,
            backend_generation: 0,
            in_flight_compute: None,
            checkpoint_export: None,
            token_namespace: uuid::Uuid::new_v4().as_u128() & (u128::MAX << 64),
            next_token_sequence: 0,
            next_source_frame_sequence: 0,
            coalesced_frame_count: 0,
            stale_completion_count: 0,
            commit_clock: MappingCommitClock::new(),
            last_occupancy_commit_receipt: None,
            #[allow(deprecated)]
            accumulator: FeedOnlyAccumulator::new(),
            session,
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
            map_odom_transform_cache: Arc::new(parking_lot::RwLock::new(None)),
            frame_generation: 0,
            global_frame_established: false,
            transformed_points: Vec::new(),
            latest_accepted_map_points: Vec::new(),
            latest_accepted_point_count: 0,
            persistent_grid: None,
            checkpoint_installed: false,
            checkpoint_activation_digest: None,
            forbidden_zones_path: None,
            raw_map: None,
            forbidden_zone_signature: String::new(),
            forbidden_zone_generation: 0,
            forbidden_zone_applied_revision: 0,
            applied_forbidden_zones: None,
            last_built_generation: 0,
            last_built_point_count: 0,
            last_built_received_at_ns: 0,
            lifecycle: MappingSessionSnapshot::idle(mapping_backend.kind()),
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
    ) -> Result<MappingSessionSnapshot, MappingLifecycleError> {
        self.start_session_inner(session_id, map_name, None)
    }

    pub(crate) fn stage_backend_worker_for_next_session(
        &self,
    ) -> Result<MappingBackendWorker, MappingLifecycleError> {
        if self.lifecycle.active {
            return Err(MappingLifecycleError::Conflict(format!(
                "active_session_id={} active_map_name={}",
                self.lifecycle.session_id, self.lifecycle.map_name
            )));
        }
        if let Some(generation) = self.pending_backend_retirement {
            return Err(MappingLifecycleError::Backend(format!(
                "backend generation {generation} retirement is still pending"
            )));
        }
        let backend_generation = self
            .backend_generation
            .checked_add(1)
            .ok_or_else(|| MappingLifecycleError::Backend("generation exhausted".to_owned()))?;
        MappingBackendWorker::start(
            self.mapping_backend,
            backend_generation,
            self.backend_compute_config(),
        )
        .map_err(MappingLifecycleError::Backend)
    }

    pub(crate) fn start_session_with_staged_backend(
        &mut self,
        session_id: &str,
        map_name: &str,
        worker: MappingBackendWorker,
    ) -> Result<MappingSessionSnapshot, MappingLifecycleError> {
        self.start_session_inner(session_id, map_name, Some(worker))
    }

    pub(crate) fn abort_started_session_detached(
        &mut self,
        session_id: &str,
        session_generation: u64,
    ) -> Result<Option<MappingBackendRetirement>, MappingLifecycleError> {
        if !self.lifecycle.active
            || self.lifecycle.session_id != session_id
            || self.lifecycle.session_generation != session_generation
        {
            return Err(MappingLifecycleError::StaleSession(format!(
                "requested={session_id}/{session_generation} current={}/{} phase={}",
                self.lifecycle.session_id,
                self.lifecycle.session_generation,
                self.lifecycle.phase.as_str()
            )));
        }
        let next_backend_generation = self
            .backend_generation
            .checked_add(1)
            .ok_or(MappingLifecycleError::GenerationExhausted)?;
        let retirement = self.detach_backend_worker()?;
        self.in_flight_compute = None;
        self.checkpoint_export = None;
        self.backend_generation = next_backend_generation;
        self.lifecycle = MappingSessionSnapshot::idle(self.mapping_backend.kind());
        self.revision = 0;
        self.checkpoint_installed = false;
        self.checkpoint_activation_digest = None;
        Ok(retirement)
    }

    /// Terminalize the exact session after a post-start process-fatal error.
    ///
    /// Unlike the ordinary stop transition, this path cannot reject generation
    /// exhaustion. The process is already poisoned, so keeping the terminal
    /// session identity and returning every worker owner is more important than
    /// reserving a generation for a future session that will never be admitted.
    pub(crate) fn force_stop_started_session_detached(
        &mut self,
        session_id: &str,
        session_generation: u64,
    ) -> Result<MappingStopTransition, MappingLifecycleError> {
        if !self.lifecycle.active
            || self.lifecycle.session_id != session_id
            || self.lifecycle.session_generation != session_generation
        {
            return Err(MappingLifecycleError::StaleSession(format!(
                "requested={session_id}/{session_generation} current={}/{} phase={}",
                self.lifecycle.session_id,
                self.lifecycle.session_generation,
                self.lifecycle.phase.as_str()
            )));
        }
        let retirement = self.detach_backend_worker_for_terminal_failure();
        self.lifecycle.phase = MappingSessionPhase::Stopped;
        self.lifecycle.active = false;
        self.in_flight_compute = None;
        self.checkpoint_export = None;
        self.backend_generation = self.backend_generation.saturating_add(1);
        Ok(MappingStopTransition {
            snapshot: self.snapshot(),
            retirement,
        })
    }

    fn start_session_inner(
        &mut self,
        session_id: &str,
        map_name: &str,
        staged_worker: Option<MappingBackendWorker>,
    ) -> Result<MappingSessionSnapshot, MappingLifecycleError> {
        let admission = self.prepare_session_start_admission(session_id, map_name);
        let (session_id, map_name, generation, backend_generation) = match admission {
            Ok(MappingSessionStartAdmission::Replay(snapshot)) => {
                Self::settle_rejected_staged_worker(staged_worker)?;
                return Ok(snapshot);
            }
            Ok(MappingSessionStartAdmission::Ready {
                session_id,
                map_name,
                session_generation,
                backend_generation,
            }) => (session_id, map_name, session_generation, backend_generation),
            Err(error) => return Err(Self::reject_staged_worker(staged_worker, error)),
        };
        let worker = match staged_worker {
            Some(worker) => {
                let client = worker.client();
                if client.backend_generation() != backend_generation
                    || client.backend_kind() != self.mapping_backend.kind()
                {
                    return Err(Self::reject_staged_worker(
                        Some(worker),
                        MappingLifecycleError::Backend(
                            "staged mapping backend worker identity mismatch".to_owned(),
                        ),
                    ));
                }
                worker
            }
            None => MappingBackendWorker::start(
                self.mapping_backend,
                backend_generation,
                self.backend_compute_config(),
            )
            .map_err(MappingLifecycleError::Backend)?,
        };
        self.reset_for_new_session();
        self.backend_generation = backend_generation;
        self.mapping_worker = Some(worker);
        self.next_session_generation = generation;
        self.lifecycle = MappingSessionSnapshot {
            phase: MappingSessionPhase::Active,
            session_id,
            map_name,
            session_generation: generation,
            revision: 0,
            active: true,
            idempotent_replay: false,
            legacy_checkpoint_reconstructed: false,
            mapping_backend_kind: self.mapping_backend.kind().stable_name().to_owned(),
        };
        Ok(self.snapshot())
    }

    fn prepare_session_start_admission(
        &self,
        session_id: &str,
        map_name: &str,
    ) -> Result<MappingSessionStartAdmission, MappingLifecycleError> {
        let session_id = session_id.trim();
        let map_name = map_name.trim();
        for (field, value) in [("session_id", session_id), ("map_name", map_name)] {
            if value.is_empty() {
                return Err(MappingLifecycleError::InvalidArgument(format!(
                    "{field} must not be empty"
                )));
            }
        }

        if self.lifecycle.active {
            if self.lifecycle.session_id == session_id && self.lifecycle.map_name == map_name {
                let mut replay = self.snapshot();
                replay.idempotent_replay = true;
                return Ok(MappingSessionStartAdmission::Replay(replay));
            }
            return Err(MappingLifecycleError::Conflict(format!(
                "active_session_id={} active_map_name={}",
                self.lifecycle.session_id, self.lifecycle.map_name
            )));
        }
        if self.lifecycle.phase == MappingSessionPhase::Stopped
            && self.lifecycle.session_id == session_id
        {
            return Err(MappingLifecycleError::TerminalSession(
                session_id.to_string(),
            ));
        }

        if let Some(generation) = self.pending_backend_retirement {
            return Err(MappingLifecycleError::Backend(format!(
                "backend generation {generation} retirement is still pending"
            )));
        }
        let generation = self
            .next_session_generation
            .checked_add(1)
            .ok_or(MappingLifecycleError::GenerationExhausted)?;
        let backend_generation = self
            .backend_generation
            .checked_add(1)
            .ok_or_else(|| MappingLifecycleError::Backend("generation exhausted".to_owned()))?;
        Ok(MappingSessionStartAdmission::Ready {
            session_id: session_id.to_owned(),
            map_name: map_name.to_owned(),
            session_generation: generation,
            backend_generation,
        })
    }

    fn settle_rejected_staged_worker(
        staged_worker: Option<MappingBackendWorker>,
    ) -> Result<(), MappingLifecycleError> {
        match staged_worker {
            Some(worker) => worker.stop_and_join().map_err(|error| {
                MappingLifecycleError::Backend(format!("rejected worker join failed: {error}"))
            }),
            None => Ok(()),
        }
    }

    fn reject_staged_worker(
        staged_worker: Option<MappingBackendWorker>,
        rejection: MappingLifecycleError,
    ) -> MappingLifecycleError {
        match Self::settle_rejected_staged_worker(staged_worker) {
            Ok(()) => rejection,
            Err(join_error) => MappingLifecycleError::Backend(format!("{rejection}; {join_error}")),
        }
    }

    pub fn stop_session(
        &mut self,
        session_id: &str,
    ) -> Result<MappingSessionSnapshot, MappingLifecycleError> {
        let transition = self.stop_session_detached(session_id)?;
        if let Some(retirement) = transition.retirement {
            let generation = retirement.generation();
            let result = retirement.stop_and_join();
            self.complete_backend_retirement(generation, result)?;
        }
        Ok(transition.snapshot)
    }

    pub(crate) fn stop_session_detached(
        &mut self,
        session_id: &str,
    ) -> Result<MappingStopTransition, MappingLifecycleError> {
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
            return Ok(MappingStopTransition {
                snapshot: replay,
                retirement: None,
            });
        }
        if !self.lifecycle.active {
            return Err(MappingLifecycleError::StaleSession(format!(
                "requested={session_id} current_phase={}",
                self.lifecycle.phase.as_str()
            )));
        }
        let next_backend_generation = self
            .backend_generation
            .checked_add(1)
            .ok_or(MappingLifecycleError::GenerationExhausted)?;
        let retirement = self.detach_backend_worker()?;
        self.lifecycle.phase = MappingSessionPhase::Stopped;
        self.lifecycle.active = false;
        self.in_flight_compute = None;
        self.checkpoint_export = None;
        self.backend_generation = next_backend_generation;
        Ok(MappingStopTransition {
            snapshot: self.snapshot(),
            retirement,
        })
    }

    fn detach_backend_worker(
        &mut self,
    ) -> Result<Option<MappingBackendRetirement>, MappingLifecycleError> {
        if let Some(generation) = self.pending_backend_retirement {
            return Err(MappingLifecycleError::Backend(format!(
                "backend generation {generation} retirement is already pending"
            )));
        }
        let Some(worker) = self.mapping_worker.take() else {
            return Ok(None);
        };
        let generation = worker.backend_generation();
        self.pending_backend_retirement = Some(generation);
        Ok(Some(MappingBackendRetirement {
            generation,
            acknowledge_pending: true,
            worker,
        }))
    }

    fn detach_backend_worker_for_terminal_failure(&mut self) -> Option<MappingBackendRetirement> {
        let worker = self.mapping_worker.take()?;
        let generation = worker.backend_generation();
        let acknowledge_pending = if self.pending_backend_retirement.is_none() {
            self.pending_backend_retirement = Some(generation);
            true
        } else {
            false
        };
        Some(MappingBackendRetirement {
            generation,
            acknowledge_pending,
            worker,
        })
    }

    fn prepare_final_backend_retirement(&mut self) -> MappingFinalPreparation {
        if let Some(generation) = self.pending_backend_retirement {
            return MappingFinalPreparation::Pending(MappingPendingCause::RetirementUnproven(
                format!("backend generation {generation} already has a detached retirement owner"),
            ));
        }

        if self.lifecycle.active {
            let session_id = self.lifecycle.session_id.clone();
            let session_generation = self.lifecycle.session_generation;
            return match self.stop_session_detached(&session_id) {
                Ok(transition) => match transition.retirement {
                    Some(retirement) => MappingFinalPreparation::Retirement {
                        retirement,
                        sticky_error: None,
                    },
                    None => MappingFinalPreparation::Settled(MappingRetiredCapability::quiescent(
                        self.backend_generation,
                    )),
                },
                Err(MappingLifecycleError::GenerationExhausted) => {
                    let sticky_error = MappingTerminalError::new(
                        "mapping backend generation exhausted during final retirement",
                    );
                    match self.force_stop_started_session_detached(&session_id, session_generation)
                    {
                        Ok(transition) => match transition.retirement {
                            Some(retirement) => MappingFinalPreparation::Retirement {
                                retirement,
                                sticky_error: Some(sticky_error),
                            },
                            None => MappingFinalPreparation::Pending(
                                MappingPendingCause::RetirementUnproven(
                                    "active Mapping session had no final backend join handle"
                                        .to_owned(),
                                ),
                            ),
                        },
                        Err(error) => MappingFinalPreparation::Pending(
                            MappingPendingCause::RetirementUnproven(error.to_string()),
                        ),
                    }
                }
                Err(error) => MappingFinalPreparation::Pending(
                    MappingPendingCause::RetirementUnproven(error.to_string()),
                ),
            };
        }

        match self.detach_backend_worker() {
            Ok(Some(retirement)) => MappingFinalPreparation::Retirement {
                retirement,
                sticky_error: None,
            },
            Ok(None) => MappingFinalPreparation::Settled(MappingRetiredCapability::quiescent(
                self.backend_generation,
            )),
            Err(error) => MappingFinalPreparation::Pending(
                MappingPendingCause::RetirementUnproven(error.to_string()),
            ),
        }
    }

    pub(crate) fn acknowledge_backend_retirement(
        &mut self,
        generation: u64,
    ) -> Result<MappingRetiredCapability, MappingLifecycleError> {
        if self.pending_backend_retirement != Some(generation) {
            return Err(MappingLifecycleError::Backend(format!(
                "backend retirement acknowledgement mismatch: pending={:?} completed={generation}",
                self.pending_backend_retirement
            )));
        }
        self.pending_backend_retirement = None;
        Ok(MappingRetiredCapability::exact(generation))
    }

    pub(crate) fn complete_backend_retirement(
        &mut self,
        generation: u64,
        result: Result<(), String>,
    ) -> Result<(), MappingLifecycleError> {
        let terminal = result.map_err(MappingLifecycleError::Backend);
        self.acknowledge_backend_retirement(generation)?;
        terminal
    }

    fn backend_compute_config(&self) -> MappingBackendComputeConfig {
        MappingBackendComputeConfig {
            resolution: self.config.resolution,
            occupancy_algo: self.config.occupancy_algo,
            initial_safe_radius_meters: self.config.initial_safe_radius_meters,
            window_capacity: self.session.capacity(),
        }
    }

    pub(crate) const fn backend_kind(&self) -> MappingBackendKind {
        self.mapping_backend.kind()
    }

    pub(crate) fn backend_checkpoint_client(&self) -> Result<MappingBackendClient, String> {
        if !self.is_active() {
            return Err("mapping backend checkpoint requires an active session".to_owned());
        }
        if self.in_flight_compute.is_some() {
            return Err("mapping backend checkpoint requires compute to drain".to_owned());
        }
        if self.pending_backend_retirement.is_some() {
            return Err("mapping backend checkpoint is fenced by worker retirement".to_owned());
        }
        let worker = self
            .mapping_worker
            .as_ref()
            .ok_or_else(|| "mapping backend worker is unavailable".to_owned())?;
        let client = worker.client();
        if client.backend_generation() != self.backend_generation
            || client.backend_kind() != self.mapping_backend.kind()
        {
            return Err("mapping backend worker identity mismatch".to_owned());
        }
        Ok(client)
    }

    pub(crate) fn begin_backend_checkpoint_export(
        &mut self,
    ) -> Result<MappingCheckpointExportLease, String> {
        if !self.is_active() {
            return Err("mapping backend checkpoint requires an active session".to_owned());
        }
        if self.checkpoint_export.is_some() {
            return Err("mapping backend checkpoint export is already pending".to_owned());
        }
        let lease = MappingCheckpointExportLease {
            lease_id: uuid::Uuid::new_v4().as_u128(),
            session_id: self.lifecycle.session_id.clone(),
            session_generation: self.lifecycle.session_generation,
            backend_generation: self.backend_generation,
        };
        self.checkpoint_export = Some(ActiveMappingCheckpointExport {
            lease: lease.clone(),
            flush_token_id: None,
        });
        Ok(lease)
    }

    pub(crate) fn backend_checkpoint_client_if_drained(
        &self,
        lease: &MappingCheckpointExportLease,
    ) -> Result<Option<MappingBackendClient>, String> {
        self.validate_backend_checkpoint_export_lease(lease)?;
        if self.in_flight_compute.is_some() {
            return Ok(None);
        }
        self.backend_checkpoint_client().map(Some)
    }

    pub(crate) fn finish_backend_checkpoint_export(
        &mut self,
        lease: &MappingCheckpointExportLease,
    ) {
        let Some(active) = self.checkpoint_export.as_ref() else {
            return;
        };
        if &active.lease != lease {
            return;
        }
        if let Some(flush_token_id) = active.flush_token_id {
            if self
                .in_flight_compute
                .as_ref()
                .is_some_and(|token| token.token_id == flush_token_id)
            {
                self.in_flight_compute = None;
            }
        }
        self.checkpoint_export = None;
    }

    #[cfg(test)]
    pub(crate) fn checkpoint_export_pending_for_tests(&self) -> bool {
        self.checkpoint_export.is_some()
    }

    #[cfg(test)]
    pub(crate) fn exhaust_backend_generation_for_tests(&mut self) {
        self.backend_generation = u64::MAX;
    }

    #[cfg(test)]
    pub(crate) fn backend_owner_state_for_tests(&self) -> (bool, bool) {
        (
            self.mapping_worker.is_some(),
            self.pending_backend_retirement.is_some(),
        )
    }

    #[cfg(test)]
    fn set_pending_backend_retirement_for_tests(&mut self, generation: u64) {
        self.pending_backend_retirement = Some(generation);
    }

    pub(crate) fn prepare_backend_checkpoint_flush(
        &mut self,
        lease: &MappingCheckpointExportLease,
        prepared_at_ns: u64,
    ) -> Result<Option<PreparedMappingFlush>, String> {
        self.validate_backend_checkpoint_export_lease(lease)?;
        if self.in_flight_compute.is_some() {
            return Ok(None);
        }
        if self.next_source_frame_sequence == 0 {
            return Ok(None);
        }
        let worker = self.backend_checkpoint_client()?;
        self.next_token_sequence = self
            .next_token_sequence
            .checked_add(1)
            .ok_or_else(|| "mapping checkpoint flush token sequence exhausted".to_owned())?;
        let token = MappingComputeToken {
            token_id: self.token_namespace | u128::from(self.next_token_sequence),
            session_id: self.lifecycle.session_id.clone(),
            session_generation: self.lifecycle.session_generation,
            backend_generation: self.backend_generation,
            source_frame_sequence: self.next_source_frame_sequence,
            source_timestamp_ns: self.latest_pointcloud_timestamp_ns,
            prepared_at_ns,
            base_occupancy_revision: self.revision.max(self.global_revision),
        };
        self.in_flight_compute = Some(token.clone());
        self.checkpoint_export
            .as_mut()
            .expect("validated mapping checkpoint export lease")
            .flush_token_id = Some(token.token_id);
        Ok(Some(PreparedMappingFlush { worker, token }))
    }

    fn validate_backend_checkpoint_export_lease(
        &self,
        lease: &MappingCheckpointExportLease,
    ) -> Result<(), String> {
        let active = self
            .checkpoint_export
            .as_ref()
            .ok_or_else(|| "mapping backend checkpoint export barrier is not active".to_owned())?;
        if &active.lease != lease
            || lease.session_id != self.lifecycle.session_id
            || lease.session_generation != self.lifecycle.session_generation
            || lease.backend_generation != self.backend_generation
            || !self.is_active()
        {
            return Err("mapping backend checkpoint export lease is stale".to_owned());
        }
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn prepare_backend_compute(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        received_at_ns: u64,
        frame_id: &str,
        source_timestamp_ns: u64,
        source_fenced: bool,
    ) -> Result<Option<PreparedMappingCompute>, nuwax_mapping::SessionError> {
        if self.checkpoint_export.is_some() || self.in_flight_compute.is_some() {
            self.coalesced_frame_count = self.coalesced_frame_count.saturating_add(1);
            return Ok(None);
        }
        let worker = self
            .mapping_worker
            .as_ref()
            .map(MappingBackendWorker::client)
            .ok_or_else(|| nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: normalize_frame_id(frame_id),
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping backend worker is unavailable".to_owned(),
            })?;
        if worker.backend_generation() != self.backend_generation
            || worker.backend_kind() != self.mapping_backend.kind()
        {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: normalize_frame_id(frame_id),
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping backend worker identity mismatch".to_owned(),
            });
        }

        let stats = if source_fenced {
            self.feed_pointcloud_session_after_source_fence_impl(
                points,
                odom_xy,
                point_step,
                received_at_ns,
                frame_id,
                source_timestamp_ns,
                false,
            )?
        } else {
            self.feed_pointcloud_session_impl(
                points,
                odom_xy,
                point_step,
                received_at_ns,
                frame_id,
                source_timestamp_ns,
                false,
            )?
        };
        if stats.accepted == 0 {
            return Ok(None);
        }

        self.next_source_frame_sequence = self
            .next_source_frame_sequence
            .checked_add(1)
            .ok_or_else(|| nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: normalize_frame_id(frame_id),
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping source frame sequence exhausted".to_owned(),
            })?;
        self.next_token_sequence = self.next_token_sequence.checked_add(1).ok_or_else(|| {
            nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: normalize_frame_id(frame_id),
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping compute token sequence exhausted".to_owned(),
            }
        })?;
        let source_frame_sequence = self.next_source_frame_sequence;
        let effective_timestamp_ns = self.latest_pointcloud_timestamp_ns;
        let token = MappingComputeToken {
            token_id: self.token_namespace | u128::from(self.next_token_sequence),
            session_id: self.lifecycle.session_id.clone(),
            session_generation: self.lifecycle.session_generation,
            backend_generation: self.backend_generation,
            source_frame_sequence,
            source_timestamp_ns: effective_timestamp_ns,
            prepared_at_ns: received_at_ns,
            base_occupancy_revision: self.revision.max(self.global_revision),
        };
        let input = MappingBackendInput {
            session_id: token.session_id.clone(),
            session_generation: token.session_generation,
            backend_generation: token.backend_generation,
            frame_sequence: source_frame_sequence,
            source_timestamp_ns: effective_timestamp_ns,
            received_at_ns,
            registered_frame_id: self.map_frame_id.clone(),
            sensor_origin_map: self.last_sensor_origin,
            registered_points_map: self.take_latest_accepted_map_points(),
        };
        self.in_flight_compute = Some(token.clone());
        Ok(Some(PreparedMappingCompute {
            worker,
            token,
            input,
        }))
    }

    pub(crate) fn fail_backend_compute(&mut self, token: &MappingComputeToken) -> bool {
        if self.in_flight_compute.as_ref() != Some(token) {
            self.stale_completion_count = self.stale_completion_count.saturating_add(1);
            return false;
        }
        self.in_flight_compute = None;
        true
    }

    pub(crate) fn commit_backend_completion(
        &mut self,
        completion: MappingBackendCompletion,
        shared: &mut SharedState,
    ) -> Result<Option<CommittedMappingCandidate>, String> {
        let MappingBackendCompletion {
            token,
            accepted_points_map,
            result,
            resolution,
        } = completion;
        if self.in_flight_compute.as_ref() != Some(&token) {
            self.stale_completion_count = self.stale_completion_count.saturating_add(1);
            return Ok(None);
        }
        if !self.is_active()
            || self.lifecycle.session_id != token.session_id
            || self.lifecycle.session_generation != token.session_generation
            || self.backend_generation != token.backend_generation
            || self.revision.max(self.global_revision) != token.base_occupancy_revision
        {
            self.stale_completion_count = self.stale_completion_count.saturating_add(1);
            self.in_flight_compute = None;
            return Ok(None);
        }
        let result = match result {
            Ok(result) => result,
            Err(error) => {
                self.in_flight_compute = None;
                return Err(error);
            }
        };
        let Some(candidate) = result else {
            return Ok(Some(CommittedMappingCandidate {
                token: token.clone(),
                resolution,
                accepted_map_points: accepted_points_map,
                event: None,
                session_generation: token.session_generation,
                source_timestamp_ns: token.source_timestamp_ns,
                backend_kind: self.mapping_backend.kind(),
                backend_generation: token.backend_generation,
                occupancy_receipt: None,
            }));
        };
        if let Err(error) = self.validate_backend_candidate(&token, &candidate) {
            self.in_flight_compute = None;
            return Err(error);
        }
        match self.commit_backend_candidate(token, candidate, resolution, shared) {
            Ok(committed) => Ok(Some(committed)),
            Err(error) => {
                self.in_flight_compute = None;
                Err(error)
            }
        }
    }

    pub(crate) fn resolve_backend_completion(
        &mut self,
        token: &MappingComputeToken,
        resolution: MappingBackendCommitHandle,
        commit: bool,
    ) -> Result<(), String> {
        if self.in_flight_compute.as_ref() != Some(token)
            || resolution.transaction_id() != token.token_id
        {
            self.stale_completion_count = self.stale_completion_count.saturating_add(1);
            let _ = resolution.abort();
            return Err("mapping backend completion resolution identity is stale".to_owned());
        }
        let result = if commit {
            resolution.commit()
        } else {
            resolution.abort()
        };
        if result.is_ok() {
            self.in_flight_compute = None;
        }
        result
    }

    fn validate_backend_candidate(
        &self,
        token: &MappingComputeToken,
        candidate: &MappingBackendCandidate,
    ) -> Result<(), String> {
        if candidate.session_id != token.session_id
            || candidate.session_generation != token.session_generation
            || candidate.backend_generation != token.backend_generation
            || candidate.source_frame_sequence != token.source_frame_sequence
            || candidate.source_timestamp_ns != token.source_timestamp_ns
            || candidate.backend_kind != self.mapping_backend.kind()
            || candidate.live_window_occupancy.frame_id != self.map_frame_id
            || candidate.latest_frame_occupancy.frame_id != self.map_frame_id
        {
            return Err("mapping backend candidate identity mismatch".to_owned());
        }
        Ok(())
    }

    fn commit_backend_candidate(
        &mut self,
        token: MappingComputeToken,
        candidate: MappingBackendCandidate,
        resolution: MappingBackendCommitHandle,
        shared: &mut SharedState,
    ) -> Result<CommittedMappingCandidate, String> {
        let mut next_persistent = if candidate.backend_kind == MappingBackendKind::Raytracing {
            PersistentOccupancyGrid::from_grid(candidate.live_window_occupancy.clone())?
        } else {
            match self.persistent_grid.clone() {
                Some(persistent) => persistent,
                None => {
                    PersistentOccupancyGrid::from_grid(candidate.live_window_occupancy.clone())?
                }
            }
        };
        if candidate.backend_kind != MappingBackendKind::Raytracing {
            next_persistent.merge(
                &candidate.live_window_occupancy,
                &candidate.latest_frame_occupancy,
            )?;
        }
        let grid = next_persistent.grid.clone();
        let revision_floor = shared
            .map
            .revision
            .max(self.revision)
            .max(self.global_revision);
        if revision_floor != token.base_occupancy_revision {
            self.stale_completion_count = self.stale_completion_count.saturating_add(1);
            return Err("mapping candidate base occupancy revision changed".to_owned());
        }
        let next_revision = self.reserve_next_revision(revision_floor)?;
        let raw_map = MapState {
            width: grid.width,
            height: grid.height,
            resolution: grid.resolution as f32,
            origin_x: grid.origin[0],
            origin_y: grid.origin[1],
            frame_id: grid.frame_id,
            revision: next_revision,
            timestamp_ns: candidate.source_timestamp_ns,
            data: grid.data,
        };
        let loaded = self.load_forbidden_zones();
        let effective_map = self.compose_forbidden_zones(&raw_map, &loaded)?;
        let occupancy_receipt = self.reserve_occupancy_commit_receipt(
            next_revision,
            candidate.source_timestamp_ns,
            candidate.backend_kind,
            candidate.backend_generation,
        )?;
        self.persistent_grid = Some(next_persistent);
        self.record_forbidden_zone_application(&loaded, next_revision);
        self.global_revision = next_revision;
        self.revision = next_revision;
        self.lifecycle.revision = next_revision;
        self.last_built_generation = self.session.generation();
        self.last_built_point_count = self.session.current_size();
        self.last_built_received_at_ns = candidate.received_at_ns;
        shared.map = effective_map;
        let event =
            MapRevisionEvent::from_effective_and_raw(&shared.map, &raw_map, &self.lifecycle);
        self.last_occupancy_commit_receipt = Some(occupancy_receipt.clone());
        self.raw_map = Some(raw_map);
        Ok(CommittedMappingCandidate {
            token: token.clone(),
            resolution,
            accepted_map_points: candidate.accepted_points_map,
            event: Some(event),
            session_generation: token.session_generation,
            source_timestamp_ns: token.source_timestamp_ns,
            backend_kind: candidate.backend_kind,
            backend_generation: candidate.backend_generation,
            occupancy_receipt: Some(occupancy_receipt),
        })
    }

    fn reserve_occupancy_commit_receipt(
        &mut self,
        occupancy_revision: u64,
        source_timestamp_ns: u64,
        backend_kind: MappingBackendKind,
        backend_generation: u64,
    ) -> Result<OccupancyCommitReceipt, String> {
        if occupancy_revision == 0
            || self.lifecycle.session_id.is_empty()
            || self.lifecycle.session_generation == 0
            || backend_generation == 0
        {
            return Err("mapping occupancy commit identity is incomplete".to_owned());
        }
        Ok(OccupancyCommitReceipt {
            commit_id: self.commit_clock.reserve()?,
            session_id: self.lifecycle.session_id.clone(),
            session_generation: self.lifecycle.session_generation,
            occupancy_revision,
            source_timestamp_ns,
            backend_kind,
            backend_generation,
        })
    }

    pub(crate) fn latest_occupancy_commit_receipt(&self) -> Option<OccupancyCommitReceipt> {
        self.last_occupancy_commit_receipt.clone()
    }

    /// 读取当前规范地图提交时钟标识，供同拥有方的清空发布使用。
    pub(crate) fn mapping_commit_boot_nonce(&self) -> u128 {
        self.commit_clock.boot_nonce()
    }

    pub(crate) fn set_legacy_checkpoint_reconstructed(&mut self, reconstructed: bool) {
        self.lifecycle.legacy_checkpoint_reconstructed = reconstructed;
    }

    pub fn install_checkpoint(
        &mut self,
        checkpoint: OccupancyGrid,
        source_timestamp_ns: u64,
        revision_floor: u64,
    ) -> Result<(MapState, MapRevisionEvent), String> {
        if !self.is_active() {
            return Err("mapping session must be active before checkpoint install".to_string());
        }
        validate_checkpoint_grid(&checkpoint, &self.map_frame_id)?;
        let persistent_grid = PersistentOccupancyGrid::from_grid(checkpoint.clone())?;
        let next_revision = self
            .reserve_next_revision(revision_floor.max(self.revision).max(self.global_revision))?;
        let raw_map = MapState {
            width: checkpoint.width,
            height: checkpoint.height,
            resolution: checkpoint.resolution as f32,
            origin_x: checkpoint.origin[0],
            origin_y: checkpoint.origin[1],
            frame_id: checkpoint.frame_id.clone(),
            revision: next_revision,
            timestamp_ns: source_timestamp_ns,
            data: checkpoint.data.clone(),
        };
        let loaded = self.load_forbidden_zones();
        let map = self.compose_forbidden_zones(&raw_map, &loaded)?;
        let receipt = self.reserve_occupancy_commit_receipt(
            next_revision,
            source_timestamp_ns,
            self.mapping_backend.kind(),
            self.backend_generation,
        )?;
        self.checkpoint_installed = true;
        self.persistent_grid = Some(persistent_grid);
        self.record_forbidden_zone_application(&loaded, next_revision);
        self.global_revision = next_revision;
        self.revision = next_revision;
        self.lifecycle.revision = next_revision;
        let event = MapRevisionEvent::from_effective_and_raw(&map, &raw_map, &self.lifecycle);
        self.last_occupancy_commit_receipt = Some(receipt);
        self.raw_map = Some(raw_map);
        Ok((map, event))
    }

    pub fn has_checkpoint(&self) -> bool {
        self.checkpoint_installed && self.is_active() && self.current_revision() > 0
    }

    pub(crate) fn checkpoint_activation_matches(&self, digest: &[u8; 32]) -> bool {
        self.has_checkpoint() && self.checkpoint_activation_digest.as_ref() == Some(digest)
    }

    pub(crate) fn record_checkpoint_activation(&mut self, digest: [u8; 32]) -> Result<(), String> {
        if !self.has_checkpoint() {
            return Err("mapping checkpoint must be installed before activation commit".to_owned());
        }
        self.checkpoint_activation_digest = Some(digest);
        Ok(())
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

    /// Return the latest physical occupancy grid without virtual forbidden zones.
    pub fn raw_map_snapshot(&self) -> Option<MapState> {
        self.raw_map.clone()
    }

    /// 只生成已应用禁区的虚拟遮罩，物理占据和未知单元均不进入该层。
    pub(crate) fn virtual_obstacles_snapshot(&self) -> Result<Option<MapState>, String> {
        let Some(loaded) = self.applied_forbidden_zones.as_ref() else {
            return Ok(None);
        };
        if !matches!(
            loaded.status,
            crate::forbidden_zones::ZoneLoadStatus::FailClosed(_)
        ) && !loaded.zones.iter().any(|zone| zone.enabled)
        {
            return Ok(None);
        }
        let mut free = self
            .raw_map
            .clone()
            .ok_or_else(|| "applied forbidden zones have no physical map geometry".to_string())?;
        free.data.fill(0);
        crate::forbidden_zones::compose_effective_map(&free, loaded).map(Some)
    }

    fn load_forbidden_zones(&self) -> crate::forbidden_zones::LoadedForbiddenZones {
        match self.forbidden_zones_path.as_deref() {
            Some(path) => crate::forbidden_zones::load_for_map(path, &self.lifecycle.map_name),
            None => crate::forbidden_zones::LoadedForbiddenZones {
                signature: format!("disabled:{}", self.lifecycle.map_name),
                generation: 0,
                zones: Vec::new(),
                status: crate::forbidden_zones::ZoneLoadStatus::Empty,
            },
        }
    }

    fn record_forbidden_zone_application(
        &mut self,
        loaded: &crate::forbidden_zones::LoadedForbiddenZones,
        applied_revision: u64,
    ) {
        let signature_changed = loaded.signature != self.forbidden_zone_signature;
        self.forbidden_zone_signature = loaded.signature.clone();
        self.forbidden_zone_generation = loaded.generation;
        self.applied_forbidden_zones = Some(loaded.clone());
        if signature_changed || self.forbidden_zone_applied_revision == 0 {
            self.forbidden_zone_applied_revision = applied_revision;
        }
    }

    fn record_forbidden_zone_observation_without_map(
        &mut self,
        loaded: &crate::forbidden_zones::LoadedForbiddenZones,
    ) {
        self.forbidden_zone_signature = loaded.signature.clone();
        self.forbidden_zone_generation = loaded.generation;
        self.forbidden_zone_applied_revision = 0;
        self.applied_forbidden_zones = None;
    }

    pub(crate) fn forbidden_zone_ack(&self) -> ForbiddenZoneAck {
        ForbiddenZoneAck {
            map_name: self.lifecycle.map_name.clone(),
            generation: self.forbidden_zone_generation,
            signature: self.forbidden_zone_signature.clone(),
            applied_revision: self.forbidden_zone_applied_revision,
        }
    }

    fn compose_forbidden_zones(
        &self,
        raw_map: &MapState,
        loaded: &crate::forbidden_zones::LoadedForbiddenZones,
    ) -> Result<MapState, String> {
        if let crate::forbidden_zones::ZoneLoadStatus::FailClosed(reason) = &loaded.status {
            warn!(
                map_name = %self.lifecycle.map_name,
                generation = loaded.generation,
                %reason,
                "forbidden-zone snapshot rejected; effective occupancy fails closed"
            );
        }
        crate::forbidden_zones::compose_effective_map(raw_map, loaded)
    }

    /// Recompose the most recent physical grid after an atomic forbidden-zone
    /// file replacement.  A successful update advances the canonical map
    /// revision and therefore reaches SHM, LCM, planners, guard and exploration
    /// through the existing fanout path.
    pub fn refresh_forbidden_zones(
        &mut self,
        state: &Arc<RwLock<SharedState>>,
        map_revision_tx: &broadcast::Sender<MapRevisionEvent>,
    ) -> Result<bool, String> {
        if !self.is_active() || self.forbidden_zones_path.is_none() {
            return Ok(false);
        }
        let loaded = self.load_forbidden_zones();
        if loaded.signature == self.forbidden_zone_signature {
            return Ok(false);
        }
        let Some(mut raw_map) = self.raw_map.clone() else {
            self.record_forbidden_zone_observation_without_map(&loaded);
            return Ok(false);
        };
        let mut shared = state.try_write().map_err(|_| {
            "SharedState.try_write failed during forbidden-zone refresh".to_string()
        })?;
        let revision_floor = shared
            .map
            .revision
            .max(self.revision)
            .max(self.global_revision);
        let next_revision = self.reserve_next_revision(revision_floor)?;
        let timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        raw_map.revision = next_revision;
        raw_map.timestamp_ns = timestamp_ns;
        let effective_map = self.compose_forbidden_zones(&raw_map, &loaded)?;

        self.record_forbidden_zone_application(&loaded, next_revision);
        self.global_revision = next_revision;
        self.revision = next_revision;
        self.lifecycle.revision = next_revision;
        shared.map = effective_map;
        let event =
            MapRevisionEvent::from_effective_and_raw(&shared.map, &raw_map, &self.lifecycle);
        self.raw_map = Some(raw_map);
        drop(shared);
        let _ = map_revision_tx.send(event);
        Ok(true)
    }

    /// 喂入一帧点云;`mapping_adapter` 不持锁跨 await,调用方负责同步。
    ///
    /// 无源时间戳时不做重放去重，但仍受滚动点预算约束。
    #[cfg(test)]
    pub fn feed_pointcloud(&mut self, points: &[Point3<f64>]) {
        if !self.is_active() || points.is_empty() {
            return;
        }
        let accepted = points.len().min(self.session.capacity());
        let accepted_points = &points[points.len() - accepted..];
        if let Err(error) = self
            .mapping_kernel
            .prepare_map_frame(self.last_sensor_origin, accepted_points)
        {
            warn!(
                backend = self.mapping_kernel.stable_name(),
                %error,
                "mapping frame rejected"
            );
            return;
        }
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let stats = self.session.feed_unstamped(points);
        if stats.accepted > 0 {
            if let Err(error) = self.mapping_kernel.commit_prepared_map_frame() {
                warn!(
                    backend = self.mapping_kernel.stable_name(),
                    %error,
                    "prepared mapping frame commit failed"
                );
                return;
            }
            self.latest_accepted_point_count = stats.accepted;
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

    /// Register a raw odom-frame cloud into the canonical map frame without mutating mapping state.
    ///
    /// The narrow contract rejects already-registered map input and all other source frames so a
    /// Device ingress owner cannot accidentally apply `map <- odom` zero or multiple times.
    pub(crate) fn register_odom_cloud_to_map(
        &self,
        points: &[Point3<f64>],
        source_frame_id: &str,
    ) -> Result<RegisteredMapCloud, nuwax_mapping::SessionError> {
        let source_frame_id = normalize_frame_id(source_frame_id);
        if !self.is_active() {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "mapping session is inactive".to_string(),
            });
        }
        if source_frame_id != self.odom_frame_id {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "raw cloud source frame must equal the configured odom frame".to_string(),
            });
        }
        let transform = self.map_odom_transform.as_ref().ok_or_else(|| {
            nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: source_frame_id.clone(),
                target_frame_id: self.map_frame_id.clone(),
                reason: "first valid odom pose has not anchored T_map_odom".to_string(),
            }
        })?;
        if points
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite())
        {
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id,
                target_frame_id: self.map_frame_id.clone(),
                reason: "point cloud contains a non-finite point".to_string(),
            });
        }

        let points = points
            .iter()
            .map(|point| {
                let transformed = transform.transform_position([point.x, point.y, point.z]);
                Point3::new(transformed[0], transformed[1], transformed[2])
            })
            .collect();
        Ok(RegisteredMapCloud {
            points,
            session_generation: self.lifecycle.session_generation,
            frame_generation: self.frame_generation,
        })
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
        self.feed_pointcloud_session_impl(
            points,
            odom_xy,
            point_step,
            received_at_ns,
            frame_id,
            source_timestamp_ns,
            true,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn feed_pointcloud_session_impl(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        received_at_ns: u64,
        frame_id: &str,
        source_timestamp_ns: u64,
        compatibility_kernel: bool,
    ) -> Result<nuwax_mapping::FeedStats, nuwax_mapping::SessionError> {
        self.latest_accepted_map_points.clear();
        self.latest_accepted_point_count = 0;
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
        let candidate_count = map_points.len().min(self.session.capacity());
        let candidate_points = &map_points[map_points.len() - candidate_count..];
        self.mapping_backend
            .validate_frame_input(self.last_sensor_origin, candidate_points)
            .map_err(
                |reason| nuwax_mapping::SessionError::FrameTransformUnavailable {
                    source_frame_id: source_frame_id.clone(),
                    target_frame_id: self.map_frame_id.clone(),
                    reason,
                },
            )?;
        #[cfg(test)]
        if compatibility_kernel {
            self.mapping_kernel
                .prepare_map_frame(self.last_sensor_origin, candidate_points)
                .map_err(
                    |reason| nuwax_mapping::SessionError::FrameTransformUnavailable {
                        source_frame_id: source_frame_id.clone(),
                        target_frame_id: self.map_frame_id.clone(),
                        reason,
                    },
                )?;
        }
        #[cfg(not(test))]
        let _ = compatibility_kernel;
        let stats =
            self.session
                .feed_pointcloud(map_points, dedup_xy, point_step, message_timestamp_ns)?;
        if stats.accepted > 0 {
            #[cfg(test)]
            if compatibility_kernel {
                self.mapping_kernel
                    .commit_prepared_map_frame()
                    .map_err(
                        |reason| nuwax_mapping::SessionError::FrameTransformUnavailable {
                            source_frame_id: source_frame_id.clone(),
                            target_frame_id: self.map_frame_id.clone(),
                            reason,
                        },
                    )?;
            }
            self.latest_accepted_point_count = stats.accepted;
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

    /// Feed a point cloud whose source identity has already been fenced by the
    /// generation owner. The adapter receipt clock drives session freshness,
    /// while canonical output metadata retains the exact source timestamp,
    /// including the valid value zero.
    pub(crate) fn feed_pointcloud_session_after_source_fence(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        received_at_ns: u64,
        frame_id: &str,
        source_timestamp_ns: u64,
    ) -> Result<nuwax_mapping::FeedStats, nuwax_mapping::SessionError> {
        self.feed_pointcloud_session_after_source_fence_impl(
            points,
            odom_xy,
            point_step,
            received_at_ns,
            frame_id,
            source_timestamp_ns,
            true,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn feed_pointcloud_session_after_source_fence_impl(
        &mut self,
        points: &[Point3<f64>],
        odom_xy: (f64, f64),
        point_step: u32,
        received_at_ns: u64,
        frame_id: &str,
        source_timestamp_ns: u64,
        compatibility_kernel: bool,
    ) -> Result<nuwax_mapping::FeedStats, nuwax_mapping::SessionError> {
        let stats = self.feed_pointcloud_session_impl(
            points,
            odom_xy,
            point_step,
            received_at_ns,
            frame_id,
            received_at_ns,
            compatibility_kernel,
        )?;
        if stats.accepted > 0 {
            self.latest_pointcloud_timestamp_ns = source_timestamp_ns;
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
        *self.map_odom_transform_cache.write() = self.map_odom_transform.clone();
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

    /// Canonical map frame fixed by the runtime's validated startup bindings.
    pub(crate) fn configured_map_frame_id(&self) -> &str {
        &self.map_frame_id
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

    /// Shared read-only transform snapshot for high-rate pose publication.
    /// The cache is updated with the adapter and cleared on every map reset.
    pub(crate) fn map_odom_transform_cache(
        &self,
    ) -> Arc<parking_lot::RwLock<Option<MapOdomTransformSnapshot>>> {
        Arc::clone(&self.map_odom_transform_cache)
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
    #[cfg(test)]
    pub fn build_occupancy(&self) -> Result<OccupancyGrid, MappingError> {
        self.build_maps().map(|(_, occupancy)| occupancy)
    }

    #[cfg(test)]
    fn build_maps(&self) -> Result<(CostMapper, OccupancyGrid), MappingError> {
        let voxel_points = self.session.clone_points();
        let points = self.mapping_kernel.accumulated_points(&voxel_points);
        let origin = self.last_sensor_origin;
        let cloud = SimplePointCloud2::with_metadata(
            points.to_vec(),
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

    #[cfg(test)]
    fn build_maps_zero_copy(
        &mut self,
    ) -> Result<(CostMapper, OccupancyGrid, OccupancyGrid), MappingError> {
        let origin = self.last_sensor_origin;
        let frame_id = self.latest_pointcloud_frame_id.clone();
        let timestamp = self.latest_pointcloud_timestamp_ns as f64 / 1_000_000_000.0;
        let occupancy_config = self.cost_mapper.config().config.clone();
        let resolution = self.config.resolution;
        let latest_count = self.latest_accepted_point_count;
        let voxel_points = self.session.make_contiguous_points();
        let points = self.mapping_kernel.accumulated_points(voxel_points);
        let cloud = BorrowedPointCloud {
            points,
            sensor_origin: nalgebra::point![origin[0], origin[1], origin[2]],
            frame_id: &frame_id,
            timestamp,
        };
        let (mapper, occupancy, _costmap) =
            CostMapper::from_pointcloud_auto(&cloud, &occupancy_config, resolution)?;
        let latest_count = latest_count.min(points.len());
        let latest_grid = if self.mapping_kernel.is_raytracing() {
            occupancy.clone()
        } else if latest_count == 0 {
            let mut unknown = occupancy.clone();
            unknown.data.fill(-1);
            unknown
        } else {
            let latest_start = points.len() - latest_count;
            let latest_cloud = BorrowedPointCloud {
                points: &points[latest_start..],
                sensor_origin: nalgebra::point![origin[0], origin[1], origin[2]],
                frame_id: &frame_id,
                timestamp,
            };
            let (_latest_mapper, latest_grid, _latest_costmap) =
                CostMapper::from_pointcloud_auto(&latest_cloud, &occupancy_config, resolution)?;
            latest_grid
        };
        Ok((mapper, occupancy, latest_grid))
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
    #[cfg(test)]
    pub fn build_and_publish(
        &mut self,
        state: &Arc<RwLock<SharedState>>,
        map_revision_tx: Option<&broadcast::Sender<MapRevisionEvent>>,
    ) -> bool {
        if !self.is_active()
            || !self.has_data()
            || !self.mapping_kernel.ready_to_build()
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
        let (mapper, live_grid, latest_grid) = match self.build_maps_zero_copy() {
            Ok(result) => result,
            Err(error) => {
                warn!("mapping_adapter: auto-bounds occupancy 构建失败: {error}");
                return false;
            }
        };
        self.cost_mapper = mapper;
        let raytracing_backend = self.mapping_kernel.is_raytracing();
        let mut next_persistent = if raytracing_backend {
            match PersistentOccupancyGrid::from_grid(live_grid.clone()) {
                Ok(persistent) => persistent,
                Err(error) => {
                    warn!("mapping_adapter: persistent occupancy initialization failed: {error}");
                    return false;
                }
            }
        } else {
            match self.persistent_grid.clone() {
                Some(persistent) => persistent,
                None => match PersistentOccupancyGrid::from_grid(live_grid.clone()) {
                    Ok(persistent) => persistent,
                    Err(error) => {
                        warn!(
                            "mapping_adapter: persistent occupancy initialization failed: {error}"
                        );
                        return false;
                    }
                },
            }
        };
        if !raytracing_backend {
            if let Err(error) = next_persistent.merge(&live_grid, &latest_grid) {
                warn!("mapping_adapter: persistent/live occupancy merge failed: {error}");
                return false;
            }
        }
        let grid = next_persistent.grid.clone();
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
            let raw_map = MapState {
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
            let loaded = self.load_forbidden_zones();
            let effective_map = match self.compose_forbidden_zones(&raw_map, &loaded) {
                Ok(map) => map,
                Err(error) => {
                    warn!(%error, "mapping_adapter: forbidden-zone composition failed closed before commit");
                    return false;
                }
            };
            let receipt = match self.reserve_occupancy_commit_receipt(
                next_revision,
                self.latest_pointcloud_timestamp_ns,
                self.mapping_backend.kind(),
                self.backend_generation,
            ) {
                Ok(receipt) => receipt,
                Err(error) => {
                    warn!(%error, "mapping_adapter: commit receipt reservation failed closed");
                    return false;
                }
            };
            self.persistent_grid = Some(next_persistent);
            self.record_forbidden_zone_application(&loaded, next_revision);
            self.global_revision = next_revision;
            self.revision = next_revision;
            s.map = effective_map;
            self.last_built_generation = built_generation;
            self.last_built_point_count = built_point_count;
            self.last_built_received_at_ns = built_received_at_ns;
            // 从刚 commit 的 MapState 构造自包含事件，确保 metadata/data/checksum
            // 属于同一 revision，server 不需要再读取可能已前进的 SharedState。
            self.lifecycle.revision = next_revision;
            let event = MapRevisionEvent::from_effective_and_raw(&s.map, &raw_map, &self.lifecycle);
            self.last_occupancy_commit_receipt = Some(receipt);
            self.raw_map = Some(raw_map);
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
        let retirement = self.reset_accumulator_detached();
        if let Some(retirement) = retirement {
            let generation = retirement.generation();
            let result = retirement.stop_and_join();
            if let Err(error) = self.complete_backend_retirement(generation, result) {
                warn!(%error, "mapping backend worker retirement failed during reset");
                return;
            }
        }
        self.restart_backend_worker_after_reset();
    }

    fn reset_accumulator_detached(&mut self) -> Option<MappingBackendRetirement> {
        self.in_flight_compute = None;
        self.checkpoint_export = None;
        let retirement = match self.detach_backend_worker() {
            Ok(retirement) => retirement,
            Err(error) => {
                warn!(%error, "mapping backend worker detach failed during reset");
                None
            }
        };
        self.backend_generation = self.backend_generation.saturating_add(1);
        #[allow(deprecated)]
        {
            self.accumulator = FeedOnlyAccumulator::new();
        }
        self.session.reset_state();
        #[cfg(test)]
        self.mapping_kernel.reset();
        self.has_data = false;
        self.last_sensor_origin = [0.0, 0.0, 0.0];
        self.latest_pointcloud_source_frame_id.clear();
        self.latest_pointcloud_frame_id.clear();
        self.latest_pointcloud_transform_applied = false;
        self.latest_pointcloud_timestamp_ns = 0;
        self.latest_pointcloud_received_at_ns = 0;
        self.map_odom_transform = None;
        *self.map_odom_transform_cache.write() = None;
        self.global_frame_established = false;
        self.transformed_points.clear();
        self.latest_accepted_map_points.clear();
        self.latest_accepted_point_count = 0;
        self.persistent_grid = None;
        self.checkpoint_installed = false;
        self.checkpoint_activation_digest = None;
        self.raw_map = None;
        self.last_occupancy_commit_receipt = None;
        self.forbidden_zone_signature.clear();
        self.forbidden_zone_generation = 0;
        self.forbidden_zone_applied_revision = 0;
        self.applied_forbidden_zones = None;
        self.last_built_generation = 0;
        self.last_built_point_count = 0;
        self.last_built_received_at_ns = 0;
        retirement
    }

    fn restart_backend_worker_after_reset(&mut self) {
        if self.is_active() && self.pending_backend_retirement.is_none() {
            match MappingBackendWorker::start(
                self.mapping_backend,
                self.backend_generation,
                self.backend_compute_config(),
            ) {
                Ok(worker) => self.mapping_worker = Some(worker),
                Err(error) => warn!(%error, "mapping backend worker restart failed after reset"),
            }
        }
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

    #[cfg(test)]
    pub(crate) fn replace_worker_with_test_gate(
        &mut self,
        gate: MappingBackendTestGate,
    ) -> Result<(), String> {
        if let Some(worker) = self.mapping_worker.take() {
            worker.stop_and_join()?;
        }
        self.mapping_worker = Some(MappingBackendWorker::start_with_test_gate(
            self.mapping_backend,
            self.backend_generation,
            self.backend_compute_config(),
            gate,
        )?);
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn replace_worker_with_retirement_test_gate(
        &mut self,
        gate: MappingBackendRetirementTestGate,
    ) -> Result<(), String> {
        if let Some(worker) = self.mapping_worker.take() {
            worker.stop_and_join()?;
        }
        self.mapping_worker = Some(MappingBackendWorker::start_with_retirement_test_gate(
            self.mapping_backend,
            self.backend_generation,
            gate,
        )?);
        Ok(())
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
            .start_session("test-session", "test-map")
            .expect("activate test mapping session");
        adapter
    }

    fn active_raytracing_adapter() -> MappingAdapter {
        let mut adapter = MappingAdapter::new_with_frames_and_optional_sequence_store(
            sample_config(),
            MappingBackendSpec::raytracing_defaults(),
            "map",
            "odom",
            None,
        );
        adapter
            .start_session("ray-session", "ray-map")
            .expect("activate raytracing mapping session");
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
            .start_session("persistent-session", "test-map")
            .expect("activate persistent mapping session");
        adapter
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1.0e-9,
            "actual={actual} expected={expected}"
        );
    }

    #[derive(Debug, PartialEq)]
    struct RegistrationState {
        lifecycle: MappingSessionSnapshot,
        session_stats: MappingSessionStats,
        session_points: Vec<Point3<f64>>,
        transform: Option<MapOdomTransformSnapshot>,
        cached_transform: Option<MapOdomTransformSnapshot>,
        frame_generation: u64,
        global_frame_established: bool,
        has_data: bool,
        odom_count: u64,
        pointcloud_count: u64,
        latest_accepted_map_points: Vec<Point3<f64>>,
        transformed_points: Vec<Point3<f64>>,
    }

    fn registration_state(adapter: &MappingAdapter) -> RegistrationState {
        RegistrationState {
            lifecycle: adapter.snapshot(),
            session_stats: adapter.session_stats(),
            session_points: adapter.session.clone_points(),
            transform: adapter.map_odom_transform_snapshot(),
            cached_transform: adapter.map_odom_transform_cache.read().clone(),
            frame_generation: adapter.frame_generation,
            global_frame_established: adapter.global_frame_established,
            has_data: adapter.has_data,
            odom_count: adapter.odom_count,
            pointcloud_count: adapter.pointcloud_count,
            latest_accepted_map_points: adapter.latest_accepted_map_points.clone(),
            transformed_points: adapter.transformed_points.clone(),
        }
    }

    fn assert_registration_rejected_without_mutation(
        adapter: &MappingAdapter,
        points: &[Point3<f64>],
        source_frame_id: &str,
    ) -> String {
        let before = registration_state(adapter);
        let error = adapter
            .register_odom_cloud_to_map(points, source_frame_id)
            .expect_err("registration must fail closed");
        assert_eq!(registration_state(adapter), before);
        match error {
            nuwax_mapping::SessionError::FrameTransformUnavailable { reason, .. } => reason,
            other => panic!("unexpected registration error: {other}"),
        }
    }

    fn grid_cell_at_world(grid: &OccupancyGrid, x: f64, y: f64) -> i8 {
        let col = snap_grid_index((x - grid.origin[0]) / grid.resolution).floor() as i64;
        let row = snap_grid_index((y - grid.origin[1]) / grid.resolution).floor() as i64;
        assert!(col >= 0 && col < i64::from(grid.width));
        assert!(row >= 0 && row < i64::from(grid.height));
        grid.data[grid.index(row as u32, col as u32)]
    }

    fn map_cell_at_world(map: &MapState, x: f64, y: f64) -> i8 {
        let resolution = f64::from(map.resolution);
        let col = snap_grid_index((x - map.origin_x) / resolution).floor() as i64;
        let row = snap_grid_index((y - map.origin_y) / resolution).floor() as i64;
        assert!(col >= 0 && col < i64::from(map.width));
        assert!(row >= 0 && row < i64::from(map.height));
        map.data[(row as u32 * map.width + col as u32) as usize]
    }

    fn observed_grid(origin: (f64, f64), width: u32, height: u32, value: i8) -> OccupancyGrid {
        let mut grid = OccupancyGrid::with_frame(width, height, 1.0, "map");
        grid.origin = [origin.0, origin.1, 0.0];
        grid.data.fill(value);
        grid
    }

    #[test]
    fn persistent_grid_keeps_old_world_cells_and_expands_for_new_cells() {
        let mut seed = observed_grid((0.0, 0.0), 2, 1, -1);
        seed.data = vec![100, 0];
        let mut persistent =
            PersistentOccupancyGrid::from_grid(seed).expect("valid persistent seed");
        let accumulated = observed_grid((1.0, 0.0), 2, 1, 0);
        let latest = accumulated.clone();

        assert!(persistent
            .merge(&accumulated, &latest)
            .expect("merge extending live grid"));

        assert_eq!(persistent.grid.origin, [0.0, 0.0, 0.0]);
        assert_eq!((persistent.grid.width, persistent.grid.height), (3, 1));
        assert_eq!(grid_cell_at_world(&persistent.grid, 0.5, 0.5), 100);
        assert_eq!(grid_cell_at_world(&persistent.grid, 2.5, 0.5), 0);
    }

    #[test]
    fn persistent_grid_ignores_unknown_and_single_frame_contradictions() {
        let seed = observed_grid((0.0, 0.0), 1, 1, 100);
        let mut persistent =
            PersistentOccupancyGrid::from_grid(seed).expect("valid persistent seed");
        let unknown = observed_grid((0.0, 0.0), 1, 1, -1);
        let free = observed_grid((0.0, 0.0), 1, 1, 0);

        assert!(!persistent
            .merge(&unknown, &unknown)
            .expect("unknown does not erase known cell"));
        assert_eq!(persistent.grid.data, vec![100]);
        assert!(!persistent
            .merge(&free, &free)
            .expect("first contradiction is pending"));
        assert_eq!(persistent.grid.data, vec![100]);
        assert!(!persistent
            .merge(&free, &free)
            .expect("second contradiction is pending"));
        assert_eq!(persistent.grid.data, vec![100]);
    }

    #[test]
    fn persistent_grid_clears_moving_obstacle_after_confirmed_free_observations() {
        let seed = observed_grid((0.0, 0.0), 1, 1, 100);
        let mut persistent =
            PersistentOccupancyGrid::from_grid(seed).expect("valid persistent seed");
        let free = observed_grid((0.0, 0.0), 1, 1, 0);

        for _ in 1..OCCUPANCY_CHANGE_CONFIRMATIONS {
            assert!(!persistent
                .merge(&free, &free)
                .expect("contradictory observation stays pending"));
            assert_eq!(persistent.grid.data, vec![100]);
        }
        assert!(persistent
            .merge(&free, &free)
            .expect("confirmed free observation clears obstacle"));
        assert_eq!(persistent.grid.data, vec![0]);
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
    fn published_grid_survives_window_eviction_expands_and_clears_after_confirmation() {
        let mut config = sample_config();
        config.resolution = 1.0;
        config.occupancy_algo = CostAlgoSelector::General;
        let mut adapter = active_adapter(config);
        adapter.session = MappingSession::with_capacity(1);
        let state = Arc::new(RwLock::new(SharedState::default()));
        let feed = |adapter: &mut MappingAdapter, point: Point3<f64>, timestamp_ns: u64| {
            adapter
                .feed_pointcloud_session(
                    &[point],
                    (0.0, 0.0),
                    12,
                    timestamp_ns,
                    "map",
                    timestamp_ns,
                )
                .expect("accepted mapping frame");
        };

        feed(&mut adapter, Point3::new(0.0, 0.0, 0.5), 1_000_000_000);
        assert!(adapter.build_and_publish(&state, None));
        assert_eq!(
            map_cell_at_world(&state.try_read().expect("first map").map, 0.5, 0.5),
            100
        );

        // Capacity one evicts the obstacle point.  Its published cell must
        // remain while a newly explored world region is appended.
        feed(&mut adapter, Point3::new(3.0, 0.0, 0.0), 2_100_000_000);
        assert!(adapter.build_and_publish(&state, None));
        {
            let shared = state.try_read().expect("expanded map");
            assert_eq!(map_cell_at_world(&shared.map, 0.5, 0.5), 100);
            assert_eq!(map_cell_at_world(&shared.map, 3.5, 0.5), 0);
        }

        for observation in 0..OCCUPANCY_CHANGE_CONFIRMATIONS {
            let timestamp_ns = 3_200_000_000 + u64::from(observation) * 1_100_000_000;
            feed(&mut adapter, Point3::new(0.0, 0.0, 0.0), timestamp_ns);
            assert!(adapter.build_and_publish(&state, None));
            let value = map_cell_at_world(&state.try_read().expect("confirmed map").map, 0.5, 0.5);
            if observation + 1 < OCCUPANCY_CHANGE_CONFIRMATIONS {
                assert_eq!(value, 100, "single-frame clears must be suppressed");
            } else {
                assert_eq!(value, 0, "confirmed free observations clear the obstacle");
            }
        }
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
    fn virtual_obstacles_use_applied_snapshot_without_physical_projection_or_future_file() {
        let zone_path = std::env::temp_dir().join(format!(
            "nuwax-applied-virtual-zones-{}.json",
            uuid::Uuid::new_v4()
        ));
        struct Cleanup(PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) {
                let _ = std::fs::remove_file(&self.0);
            }
        }
        let _cleanup = Cleanup(zone_path.clone());
        std::fs::write(&zone_path, br#"{"schema_version":1,"maps":{"test-map":{"map_name":"test-map","generation":1,"zones":[{"id":"circle","shape":"circle","mode":"inside","frame_id":"map","center":{"x":2.5,"y":2.5},"radius_m":0.3}]}}}"#).unwrap();
        let mut adapter = active_adapter(sample_config());
        adapter.forbidden_zones_path = Some(zone_path.clone());
        let mut checkpoint = OccupancyGrid::with_frame(6, 6, 1.0, "map");
        checkpoint.origin = [0.0, 0.0, 0.0];
        checkpoint.data.fill(-1);
        checkpoint.data[0] = 100;
        let (effective, _) = adapter
            .install_checkpoint(checkpoint, 10_000_000_000, 0)
            .unwrap();
        let mask = adapter.virtual_obstacles_snapshot().unwrap().unwrap();
        assert_eq!(
            (mask.revision, mask.timestamp_ns, mask.frame_id.as_str()),
            (effective.revision, 10_000_000_000, "map")
        );
        assert_eq!(mask.data[0], 0, "物理障碍不能投影为跨层虚拟禁区");
        assert_eq!(mask.data[2 + 2 * 6], 100);
        assert!(mask.data.iter().all(|value| matches!(value, 0 | 100)));
        std::fs::write(&zone_path, b"{broken-json").unwrap();
        assert_eq!(
            adapter.virtual_obstacles_snapshot().unwrap().unwrap().data,
            mask.data,
            "未应用的未来文件不能改变当前提交的遮罩"
        );
        let state = Arc::new(RwLock::new(SharedState {
            map: effective,
            ..SharedState::default()
        }));
        let (map_tx, _) = broadcast::channel(4);
        assert!(adapter.refresh_forbidden_zones(&state, &map_tx).unwrap());
        let blocked = adapter.virtual_obstacles_snapshot().unwrap().unwrap();
        assert!(blocked.revision > mask.revision);
        assert!(blocked.data.iter().all(|value| *value == 100));
        assert_eq!(blocked.revision, state.try_read().unwrap().map.revision);
        std::fs::write(&zone_path, br#"{"schema_version":1,"maps":{"test-map":{"map_name":"test-map","generation":2,"zones":[]}}}"#).unwrap();
        assert!(adapter
            .virtual_obstacles_snapshot()
            .unwrap()
            .unwrap()
            .data
            .iter()
            .all(|value| *value == 100));
        assert!(adapter.refresh_forbidden_zones(&state, &map_tx).unwrap());
        assert!(adapter.virtual_obstacles_snapshot().unwrap().is_none());
        assert_eq!(adapter.raw_map_snapshot().unwrap().data[0], 100);
    }

    #[test]
    fn forbidden_zone_refresh_changes_effective_map_but_preserves_raw_checkpoint() {
        let zone_path = std::env::temp_dir().join(format!(
            "nuwax-forbidden-zone-adapter-{}-{}.json",
            std::process::id(),
            NEXT_SEQUENCE_TEST_PATH.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::write(
            &zone_path,
            br#"{"schema_version":1,"maps":{"test-map":{"map_name":"test-map","generation":1,"signature":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","updated_at":"2026-07-19T00:00:00Z","zones":[{"id":"box","shape":"rectangle","mode":"inside","frame_id":"map","vertices":[{"x":2.0,"y":2.0},{"x":4.0,"y":4.0}],"center":null,"radius_m":null,"stroke_width_m":null,"enabled":true,"metadata":{}}]}}}"#,
        )
        .expect("write initial forbidden zones");

        let mut adapter = active_adapter(sample_config());
        adapter.forbidden_zones_path = Some(zone_path.clone());
        let mut checkpoint = OccupancyGrid::with_frame(6, 6, 1.0, "map");
        checkpoint.data.fill(0);
        checkpoint.origin = [0.0, 0.0, 0.0];
        checkpoint.ts = 10.0;
        let (effective, _) = adapter
            .install_checkpoint(checkpoint, 10_000_000_000, 0)
            .expect("install physical checkpoint with virtual zone");
        let initial_ack = adapter.forbidden_zone_ack();
        assert_eq!(initial_ack.map_name, "test-map");
        assert_eq!(initial_ack.generation, 1);
        assert_eq!(initial_ack.signature, "a".repeat(64));
        assert_eq!(initial_ack.applied_revision, effective.revision);
        assert!(effective.data.contains(&100));
        assert!(adapter
            .raw_map_snapshot()
            .expect("raw map")
            .data
            .iter()
            .all(|value| *value == 0));

        let state = Arc::new(RwLock::new(SharedState {
            map: effective,
            ..SharedState::default()
        }));
        let previous_revision = adapter.current_revision();
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(2);
        std::fs::write(
            &zone_path,
            br#"{"schema_version":1,"maps":{"test-map":{"map_name":"test-map","generation":2,"signature":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","updated_at":"2026-07-19T00:01:00Z","zones":[]}}}"#,
        )
        .expect("replace forbidden zones with explicit empty set");
        assert!(adapter
            .refresh_forbidden_zones(&state, &map_revision_tx)
            .expect("refresh forbidden zones"));
        assert!(adapter.current_revision() > previous_revision);
        assert!(state
            .try_read()
            .expect("effective state")
            .map
            .data
            .iter()
            .all(|value| *value == 0));
        let event = map_revision_rx.try_recv().expect("canonical refresh event");
        assert_eq!(event.revision, adapter.current_revision());
        assert!(event.data.iter().all(|value| *value == 0));
        let refreshed_ack = adapter.forbidden_zone_ack();
        assert_eq!(refreshed_ack.map_name, "test-map");
        assert_eq!(refreshed_ack.generation, 2);
        assert_eq!(refreshed_ack.signature, "b".repeat(64));
        assert_eq!(refreshed_ack.applied_revision, event.revision);
        assert!(adapter
            .raw_map_snapshot()
            .expect("raw map after refresh")
            .data
            .iter()
            .all(|value| *value == 0));

        let mut replacement_checkpoint = OccupancyGrid::with_frame(6, 6, 1.0, "map");
        replacement_checkpoint.data.fill(0);
        replacement_checkpoint.origin = [0.0, 0.0, 0.0];
        replacement_checkpoint.ts = 11.0;
        adapter
            .install_checkpoint(
                replacement_checkpoint,
                11_000_000_000,
                adapter.current_revision(),
            )
            .expect("install later physical checkpoint with unchanged forbidden zones");
        assert!(adapter.current_revision() > refreshed_ack.applied_revision);
        let rebuilt_ack = adapter.forbidden_zone_ack();
        assert_eq!(rebuilt_ack.generation, refreshed_ack.generation);
        assert_eq!(
            rebuilt_ack.applied_revision, refreshed_ack.applied_revision,
            "unrelated physical-map revisions must not rewrite the causal acknowledgement"
        );
        let _ = std::fs::remove_file(zone_path);
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
            .start_session("session-1", "warehouse")
            .expect("first start");
        assert_eq!(first.session_generation, 1);
        assert!(first.active);

        let replay = adapter
            .start_session("session-1", "warehouse")
            .expect("idempotent start");
        assert!(replay.idempotent_replay);
        assert!(matches!(
            adapter.start_session("session-1", "office"),
            Err(MappingLifecycleError::Conflict(_))
        ));
        assert!(matches!(
            adapter.start_session("session-2", "warehouse"),
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
            .start_session("session-2", "warehouse")
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
    fn source_fenced_zero_timestamp_uses_receipt_freshness_and_preserves_metadata() {
        const RECEIVED_AT_NS: u64 = 10_000_000_000;

        let mut adapter = active_adapter(sample_config());
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("odom anchor");
        let stats = adapter
            .feed_pointcloud_session_after_source_fence(
                &[Point3::new(0.0, 0.0, 0.0)],
                (0.0, 0.0),
                16,
                RECEIVED_AT_NS,
                "odom",
                0,
            )
            .expect("source-fenced frame accepted");

        assert_eq!(stats.accepted, 1);
        assert_eq!(adapter.session.last_seen_at_ns(), RECEIVED_AT_NS);
        assert_eq!(adapter.latest_pointcloud_received_at_ns, RECEIVED_AT_NS);
        assert_eq!(adapter.latest_pointcloud_timestamp_ns, 0);

        let state = Arc::new(RwLock::new(SharedState::default()));
        let (revision_tx, mut revision_rx) = broadcast::channel(1);
        assert!(adapter.build_and_publish(&state, Some(&revision_tx)));
        assert_eq!(state.try_read().expect("canonical map").map.timestamp_ns, 0);
        assert_eq!(
            revision_rx
                .try_recv()
                .expect("canonical revision")
                .timestamp_ns,
            0
        );
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
    fn odom_cloud_registration_transforms_once_and_returns_authoritative_generations() {
        let mut adapter = active_adapter(sample_config());
        adapter
            .feed_pose_in_frame(10.0, 5.0, 2.0, std::f64::consts::FRAC_PI_2, "odom")
            .expect("establish odom anchor");
        let before = registration_state(&adapter);

        let registered = adapter
            .register_odom_cloud_to_map(
                &[Point3::new(10.0, 6.0, 3.0), Point3::new(8.0, 5.0, 1.0)],
                "/odom",
            )
            .expect("raw odom cloud registration");

        assert_eq!(
            registered.session_generation,
            adapter.snapshot().session_generation
        );
        assert_eq!(
            registered.frame_generation,
            adapter.global_frame_generation()
        );
        assert_eq!(registered.points.len(), 2);
        assert_close(registered.points[0].x, 1.0);
        assert_close(registered.points[0].y, 0.0);
        assert_close(registered.points[0].z, 1.0);
        assert_close(registered.points[1].x, 0.0);
        assert_close(registered.points[1].y, 2.0);
        assert_close(registered.points[1].z, -1.0);
        assert_eq!(registration_state(&adapter), before);
    }

    #[test]
    fn odom_cloud_registration_rejects_non_raw_or_unregistered_frames_without_mutation() {
        let mut adapter = active_adapter(sample_config());
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("establish odom anchor");
        let points = [Point3::new(1.0, 2.0, 3.0)];

        let map_reason = assert_registration_rejected_without_mutation(&adapter, &points, "/map");
        assert!(map_reason.contains("configured odom frame"));
        let sensor_reason =
            assert_registration_rejected_without_mutation(&adapter, &points, "lidar_link");
        assert!(sensor_reason.contains("configured odom frame"));
    }

    #[test]
    fn odom_cloud_registration_rejects_inactive_missing_anchor_and_nonfinite_without_mutation() {
        let inactive = MappingAdapter::new(sample_config());
        let inactive_reason = assert_registration_rejected_without_mutation(
            &inactive,
            &[Point3::new(0.0, 0.0, 0.0)],
            "odom",
        );
        assert!(inactive_reason.contains("inactive"));

        let missing_anchor = active_adapter(sample_config());
        let missing_anchor_reason = assert_registration_rejected_without_mutation(
            &missing_anchor,
            &[Point3::new(0.0, 0.0, 0.0)],
            "odom",
        );
        assert!(missing_anchor_reason.contains("has not anchored"));

        let mut anchored = active_adapter(sample_config());
        anchored
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("establish odom anchor");
        for point in [
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(0.0, f64::INFINITY, 0.0),
            Point3::new(0.0, 0.0, f64::NEG_INFINITY),
        ] {
            let reason = assert_registration_rejected_without_mutation(&anchored, &[point], "odom");
            assert!(reason.contains("non-finite"));
        }
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
        let transform_cache = adapter.map_odom_transform_cache();
        assert!(transform_cache.read().is_some());
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
        assert!(transform_cache.read().is_none());
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
        assert_eq!(
            transform_cache
                .read()
                .as_ref()
                .map(|transform| transform.generation),
            Some(second_generation)
        );
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
            .start_session("mapload:one", "warehouse")
            .expect("start checkpoint session");
        let mut checkpoint = OccupancyGrid::with_frame(2, 2, 0.05, "map");
        checkpoint.origin = [-1.0, 2.0, 0.0];
        checkpoint.data = vec![-1, 0, 100, 50];

        let (map, event) = adapter
            .install_checkpoint(checkpoint, 0, 0)
            .expect("install checkpoint");

        assert_eq!(map.data, vec![-1, 0, 100, 50]);
        assert_eq!(event.map_name, "warehouse");
        assert_eq!(event.session_id, "mapload:one");
        assert_eq!(event.revision, 1);
        assert_eq!(adapter.pointcloud_count(), 0);
        let readiness = MappingReadiness::compute(Some(&adapter), true);
        assert!(readiness.ready);
        assert!(readiness.global_consumer_ready);
    }

    #[test]
    fn rejected_checkpoint_receipt_does_not_mutate_adapter_state() {
        let mut adapter = active_adapter(sample_config());
        adapter.backend_generation = 0;
        let before = adapter.snapshot();
        let mut checkpoint = OccupancyGrid::with_frame(2, 2, 0.05, "map");
        checkpoint.data = vec![-1, 0, 100, 50];

        let Err(error) = adapter.install_checkpoint(checkpoint, 17, 0) else {
            panic!("invalid backend identity must reject checkpoint");
        };

        assert!(error.contains("occupancy commit identity is incomplete"));
        assert_eq!(adapter.snapshot(), before);
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(adapter.global_revision(), 0);
        assert!(!adapter.checkpoint_installed);
        assert!(adapter.persistent_grid.is_none());
        assert!(adapter.raw_map.is_none());
        assert!(adapter.last_occupancy_commit_receipt.is_none());
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
            .install_checkpoint(checkpoint, 0, 0)
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

    #[test]
    fn raytracing_health_gate_commits_through_the_canonical_revision_path() {
        let mut adapter = active_raytracing_adapter();
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (revision_tx, mut revision_rx) = broadcast::channel(4);
        let points = [
            Point3::new(1.0, -0.1, 2.0),
            Point3::new(1.0, 0.0, 2.0),
            Point3::new(1.0, 0.1, 2.0),
        ];

        for timestamp_ns in [1_000_000_000, 2_000_000_000] {
            let stats = adapter
                .feed_pointcloud_session(&points, (0.0, 0.0), 24, timestamp_ns, "map", timestamp_ns)
                .expect("raytracing frame accepted");
            assert_eq!(stats.accepted, points.len());
            assert!(!adapter.build_and_publish(&state, Some(&revision_tx)));
            assert_eq!(adapter.current_revision(), 0);
            assert!(revision_rx.try_recv().is_err());
        }

        adapter
            .feed_pointcloud_session(&points, (0.0, 0.0), 24, 3_000_000_000, "map", 3_000_000_000)
            .expect("third hit makes ray voxels healthy");
        assert!(adapter.build_and_publish(&state, Some(&revision_tx)));
        let event = revision_rx
            .try_recv()
            .expect("canonical raytracing revision");
        assert_eq!(event.revision, 1);
        assert_eq!(event.frame_id, "map");
        assert_eq!(event.session_id, "ray-session");
        assert_eq!(state.blocking_read().map.revision, event.revision);
        assert!(!event.data.is_empty());
    }

    #[test]
    fn raytracing_rejection_stop_and_generation_reset_do_not_advance_revision() {
        let mut adapter = active_raytracing_adapter();
        let state = Arc::new(RwLock::new(SharedState::default()));
        let points = [Point3::new(1.0, 0.0, 2.0)];
        for timestamp_ns in [1_000_000_000, 2_000_000_000, 3_000_000_000] {
            adapter
                .feed_pointcloud_session(&points, (0.0, 0.0), 24, timestamp_ns, "map", timestamp_ns)
                .expect("valid raytracing frame");
        }
        assert!(adapter.build_and_publish(&state, None));
        let committed_revision = adapter.current_revision();
        let session_before_rejection = adapter.session_stats();

        let error = adapter
            .feed_pointcloud_session(
                &[Point3::new(f64::MAX, 0.0, 0.0)],
                (0.0, 0.0),
                24,
                4_000_000_000,
                "map",
                4_000_000_000,
            )
            .expect_err("unrepresentable ray point must fail closed");
        assert!(error.to_string().contains("RAYTRACING_UPDATE_REJECTED"));
        assert_eq!(adapter.session_stats(), session_before_rejection);
        assert_eq!(adapter.current_revision(), committed_revision);
        assert!(!adapter.build_and_publish(&state, None));

        adapter
            .stop_session("ray-session")
            .expect("stop exact session");
        let stopped_error = adapter
            .feed_pointcloud_session(&points, (0.0, 0.0), 24, 5_000_000_000, "map", 5_000_000_000)
            .expect_err("stopped session must reject input");
        assert!(stopped_error.to_string().contains("inactive"));
        assert!(!adapter.build_and_publish(&state, None));
        assert_eq!(adapter.current_revision(), committed_revision);

        adapter
            .start_session("ray-session-2", "ray-map")
            .expect("start next generation");
        adapter
            .feed_pointcloud_session(&points, (0.0, 0.0), 24, 6_000_000_000, "map", 6_000_000_000)
            .expect("new generation accepts input");
        assert!(
            !adapter.build_and_publish(&state, None),
            "one hit in a reset ray kernel must remain below the health gate"
        );
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(state.blocking_read().map.revision, committed_revision);
    }

    #[test]
    fn backend_protocol_coalesces_one_in_flight_and_rejects_duplicate_completion() {
        let mut adapter = active_adapter(sample_config());
        let points = [Point3::new(1.0, 0.0, 1.0)];
        let prepared = adapter
            .prepare_backend_compute(
                &points,
                (0.0, 0.0),
                12,
                1_000_000_000,
                "map",
                1_000_000_000,
                false,
            )
            .expect("prepare first frame")
            .expect("first frame obtains token");
        assert!(adapter
            .prepare_backend_compute(
                &points,
                (0.0, 0.0),
                12,
                2_000_000_000,
                "map",
                2_000_000_000,
                false,
            )
            .expect("coalesced frame is not an error")
            .is_none());
        assert_eq!(adapter.coalesced_frame_count, 1);

        let duplicate = MappingBackendCompletion {
            token: prepared.token.clone(),
            accepted_points_map: prepared.input.registered_points_map.clone(),
            result: Ok(None),
            resolution: MappingBackendCommitHandle::detached_for_tests(prepared.token.token_id),
        };
        let completion = prepared
            .worker
            .submit(prepared.token, prepared.input)
            .expect("submit first frame")
            .blocking_recv()
            .expect("worker completion");
        let mut state = SharedState::default();
        assert!(adapter
            .commit_backend_completion(completion, &mut state)
            .expect("first completion")
            .is_some());
        assert!(adapter
            .commit_backend_completion(duplicate, &mut state)
            .expect("duplicate is dropped, not fatal")
            .is_none());
        assert_eq!(adapter.stale_completion_count, 1);
    }

    #[test]
    fn backend_protocol_rejects_base_revision_change_without_reservation() {
        let mut adapter = active_adapter(sample_config());
        let prepared = adapter
            .prepare_backend_compute(
                &[Point3::new(1.0, 0.0, 1.0)],
                (0.0, 0.0),
                12,
                1_000_000_000,
                "map",
                1_000_000_000,
                false,
            )
            .expect("prepare frame")
            .expect("token");
        let completion = prepared
            .worker
            .submit(prepared.token, prepared.input)
            .expect("submit frame")
            .blocking_recv()
            .expect("completion");
        let mut state = SharedState::default();
        state.map.revision = 7;
        let error = adapter
            .commit_backend_completion(completion, &mut state)
            .expect_err("changed base revision must reject candidate");
        assert!(error.contains("base occupancy revision changed"));
        assert_eq!(adapter.current_revision(), 0);
        assert_eq!(adapter.global_revision(), 0);
        assert_eq!(state.map.revision, 7);
    }

    #[test]
    fn backend_retirement_fences_late_completion_and_next_generation() {
        let mut adapter = active_adapter(sample_config());
        let prepared = adapter
            .prepare_backend_compute(
                &[Point3::new(1.0, 0.0, 1.0)],
                (0.0, 0.0),
                12,
                1_000_000_000,
                "map",
                1_000_000_000,
                false,
            )
            .expect("prepare frame")
            .expect("token");
        let late = MappingBackendCompletion {
            token: prepared.token.clone(),
            accepted_points_map: prepared.input.registered_points_map,
            result: Ok(None),
            resolution: MappingBackendCommitHandle::detached_for_tests(prepared.token.token_id),
        };
        let transition = adapter
            .stop_session_detached("test-session")
            .expect("detach active worker");
        let retirement = transition.retirement.expect("owned retirement");
        let generation = retirement.generation();
        let blocked = adapter
            .start_session("test-session-2", "test-map")
            .expect_err("new worker cannot start before old worker is destroyed");
        assert!(blocked.to_string().contains("retirement is still pending"));
        let result = retirement.stop_and_join();
        adapter
            .complete_backend_retirement(generation, result)
            .expect("acknowledge destroyed worker");
        let mut state = SharedState::default();
        assert!(adapter
            .commit_backend_completion(late, &mut state)
            .expect("late completion is dropped")
            .is_none());
        let started = adapter
            .start_session("test-session-2", "test-map")
            .expect("next generation starts after retirement");
        assert_eq!(started.session_generation, 2);
        assert!(adapter.backend_generation > generation);
    }

    #[tokio::test]
    async fn mapping_final_lifecycle_lock_timeout_retains_every_owner() {
        let adapter = Arc::new(parking_lot::Mutex::new(active_adapter(sample_config())));
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let lifecycle_guard = Arc::clone(&lifecycle_lock).lock_owned().await;
        let attempt = MappingFinalOwner::waiting_for_lifecycle_lock(
            Some(Arc::clone(&adapter)),
            Arc::clone(&lifecycle_lock),
        )
        .attempt(ShutdownFailures::default())
        .await;
        let (owner, sticky) = match attempt {
            MappingFinalAttempt::Pending {
                owner,
                cause: MappingPendingCause::LifecycleLockTimeout,
                sticky,
            } => (owner, sticky),
            _ => panic!("lifecycle-lock contention must return the retained final owner"),
        };
        assert_eq!(owner.state_name(), "waiting_lifecycle_lock");
        assert!(adapter.lock().snapshot().active);
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (true, false)
        );

        drop(lifecycle_guard);
        match owner.attempt(sticky).await {
            MappingFinalAttempt::Settled {
                capability,
                terminal,
            } => {
                assert!(capability.generation().is_some());
                assert_eq!(terminal, Ok(()));
            }
            MappingFinalAttempt::Pending { cause, .. } => {
                panic!("released lifecycle lock remained pending: {cause}")
            }
        }
    }

    #[tokio::test]
    async fn mapping_final_backend_join_timeout_retains_the_same_os_join_handle() {
        let adapter = Arc::new(parking_lot::Mutex::new(active_adapter(sample_config())));
        let retirement_gate = MappingBackendRetirementTestGate::blocked();
        adapter
            .lock()
            .replace_worker_with_retirement_test_gate(retirement_gate.clone())
            .expect("install retained retirement worker");
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let attempt = MappingFinalOwner::waiting_for_lifecycle_lock(
            Some(Arc::clone(&adapter)),
            Arc::clone(&lifecycle_lock),
        )
        .attempt(ShutdownFailures::default())
        .await;
        assert!(retirement_gate.entered());
        let (owner, sticky) = match attempt {
            MappingFinalAttempt::Pending {
                owner,
                cause: MappingPendingCause::BackendJoinTimeout { .. },
                sticky,
            } => (owner, sticky),
            _ => panic!("blocked backend join must return Pending"),
        };
        let join_thread_id = owner
            .join_thread_id()
            .expect("pending owner retains the backend OS join handle");

        let attempt = owner.attempt(sticky).await;
        let (owner, sticky) = match attempt {
            MappingFinalAttempt::Pending {
                owner,
                cause: MappingPendingCause::BackendJoinTimeout { .. },
                sticky,
            } => (owner, sticky),
            _ => panic!("second bounded attempt must retain the same join owner"),
        };
        assert_eq!(owner.join_thread_id(), Some(join_thread_id));
        retirement_gate.release();

        match owner.attempt(sticky).await {
            MappingFinalAttempt::Settled {
                capability,
                terminal,
            } => {
                assert!(capability.generation().is_some());
                assert_eq!(terminal, Ok(()));
            }
            MappingFinalAttempt::Pending { cause, .. } => {
                panic!("released backend join remained pending: {cause}")
            }
        }
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (false, false)
        );
    }

    #[tokio::test]
    async fn mapping_final_ack_lock_timeout_retains_the_joined_terminal() {
        let adapter = Arc::new(parking_lot::Mutex::new(active_adapter(sample_config())));
        let retirement = adapter
            .lock()
            .stop_session_detached("test-session")
            .expect("detach exact backend generation")
            .retirement
            .expect("active session owns backend retirement");
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let owner = MappingFinalOwner::from_retirement(
            Arc::clone(&adapter),
            Arc::clone(&lifecycle_lock),
            retirement,
        )
        .expect("retirement owns final OS join handle");
        let lifecycle_guard = Arc::clone(&lifecycle_lock).lock_owned().await;

        let (owner, sticky) = match owner.attempt(ShutdownFailures::default()).await {
            MappingFinalAttempt::Pending {
                owner,
                cause: MappingPendingCause::RetirementAckLockTimeout { .. },
                sticky,
            } => (owner, sticky),
            _ => panic!("ACK-lock contention after join must return Pending"),
        };
        assert_eq!(owner.state_name(), "waiting_retirement_ack");
        assert_eq!(owner.joined_terminal(), Some(&Ok(())));
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (false, true)
        );

        drop(lifecycle_guard);
        match owner.attempt(sticky).await {
            MappingFinalAttempt::Settled { terminal, .. } => assert_eq!(terminal, Ok(())),
            MappingFinalAttempt::Pending { cause, .. } => {
                panic!("released ACK lock remained pending: {cause}")
            }
        }
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (false, false)
        );
    }

    #[tokio::test]
    async fn mapping_final_generation_mismatch_never_creates_a_capability() {
        let adapter = Arc::new(parking_lot::Mutex::new(active_adapter(sample_config())));
        let retirement = adapter
            .lock()
            .stop_session_detached("test-session")
            .expect("detach exact backend generation")
            .retirement
            .expect("active session owns backend retirement");
        let exact_generation = retirement.generation();
        let mismatched_generation = exact_generation
            .checked_add(1)
            .expect("test generation increment");
        adapter
            .lock()
            .set_pending_backend_retirement_for_tests(mismatched_generation);
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let owner = MappingFinalOwner::from_retirement(
            Arc::clone(&adapter),
            Arc::clone(&lifecycle_lock),
            retirement,
        )
        .expect("retirement owns final OS join handle");

        let (owner, sticky) = match owner.attempt(ShutdownFailures::default()).await {
            MappingFinalAttempt::Pending {
                owner,
                cause: MappingPendingCause::GenerationMismatch { expected, pending },
                sticky,
            } => {
                assert_eq!(expected, exact_generation);
                assert_eq!(pending, Some(mismatched_generation));
                (owner, sticky)
            }
            MappingFinalAttempt::Settled { .. } => {
                panic!("generation mismatch must not mint a retirement capability")
            }
            MappingFinalAttempt::Pending { cause, .. } => {
                panic!("unexpected pending cause: {cause}")
            }
        };
        assert_eq!(owner.state_name(), "waiting_retirement_ack");
        assert_eq!(owner.joined_terminal(), Some(&Ok(())));
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (false, true)
        );

        adapter
            .lock()
            .set_pending_backend_retirement_for_tests(exact_generation);
        match owner.attempt(sticky).await {
            MappingFinalAttempt::Settled {
                capability,
                terminal,
            } => {
                assert_eq!(capability.generation(), Some(exact_generation));
                assert!(terminal.is_err(), "the observed mismatch remains sticky");
            }
            MappingFinalAttempt::Pending { cause, .. } => {
                panic!("restored exact generation remained pending: {cause}")
            }
        }
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (false, false)
        );
    }

    #[tokio::test]
    async fn joined_backend_error_still_acknowledges_the_exact_generation() {
        let adapter = Arc::new(parking_lot::Mutex::new(active_adapter(sample_config())));
        adapter
            .lock()
            .replace_worker_with_retirement_test_gate(MappingBackendRetirementTestGate::failing())
            .expect("install failing retirement worker");
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let owner = MappingFinalOwner::waiting_for_lifecycle_lock(
            Some(Arc::clone(&adapter)),
            lifecycle_lock,
        );

        match owner.attempt(ShutdownFailures::default()).await {
            MappingFinalAttempt::Settled {
                capability,
                terminal,
            } => {
                assert!(capability.generation().is_some());
                assert!(terminal.is_err(), "joined panic remains a sticky terminal");
            }
            MappingFinalAttempt::Pending { cause, .. } => {
                panic!("joined backend error remained pending: {cause}")
            }
        }
        assert_eq!(
            adapter.lock().backend_owner_state_for_tests(),
            (false, false)
        );
    }

    #[test]
    fn legacy_completion_clears_exact_pending_generation_before_returning_join_error() {
        let mut adapter = active_adapter(sample_config());
        adapter
            .replace_worker_with_retirement_test_gate(MappingBackendRetirementTestGate::failing())
            .expect("install failing retirement worker");
        let retirement = adapter
            .stop_session_detached("test-session")
            .expect("detach exact backend generation")
            .retirement
            .expect("active session owns backend retirement");
        let generation = retirement.generation();
        let joined_terminal = retirement.stop_and_join();
        assert!(joined_terminal.is_err());
        assert!(adapter
            .complete_backend_retirement(generation, joined_terminal)
            .is_err());
        assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));
    }

    #[test]
    fn rejected_staged_worker_is_joined_before_identity_error_returns() {
        // Every rejected staged worker gets an explicit stop-and-join.  Releasing the
        // retirement gate before admission makes a successful join observable without
        // adding a timing dependency to the admission assertions.
        let settled_staged_worker = |generation| {
            let gate = MappingBackendRetirementTestGate::blocked();
            gate.release();
            let worker = MappingBackendWorker::start_with_retirement_test_gate(
                MappingBackendSpec::Voxel,
                generation,
                gate.clone(),
            )
            .expect("start staged worker");
            (worker, gate)
        };

        let mut invalid = MappingAdapter::new(sample_config());
        let (staged, gate) = settled_staged_worker(1);
        let error = invalid
            .start_session_with_staged_backend("", "test-map", staged)
            .expect_err("empty session id must be rejected");
        assert_eq!(
            error,
            MappingLifecycleError::InvalidArgument("session_id must not be empty".to_owned())
        );
        assert!(gate.entered(), "invalid session id worker was not joined");

        let (staged, gate) = settled_staged_worker(2);
        let error = invalid
            .start_session_with_staged_backend("test-session", "   ", staged)
            .expect_err("empty map name must be rejected");
        assert_eq!(
            error,
            MappingLifecycleError::InvalidArgument("map_name must not be empty".to_owned())
        );
        assert!(gate.entered(), "invalid map name worker was not joined");
        assert_eq!(invalid.backend_owner_state_for_tests(), (false, false));

        let mut active = MappingAdapter::new(sample_config());
        let active_snapshot = active
            .start_session("active-session", "active-map")
            .expect("start active session");
        let (staged, gate) = settled_staged_worker(99);
        let replay = active
            .start_session_with_staged_backend("active-session", "active-map", staged)
            .expect("same active session is an idempotent replay");
        assert!(replay.idempotent_replay);
        assert_eq!(
            replay.session_generation,
            active_snapshot.session_generation
        );
        assert!(gate.entered(), "replay worker was not joined");

        let (staged, gate) = settled_staged_worker(100);
        let error = active
            .start_session_with_staged_backend("other-session", "active-map", staged)
            .expect_err("different active session must conflict");
        assert!(matches!(error, MappingLifecycleError::Conflict(_)));
        assert!(gate.entered(), "conflict worker was not joined");
        active
            .stop_session("active-session")
            .expect("settle the active adapter worker");

        let mut terminal = MappingAdapter::new(sample_config());
        terminal
            .start_session("terminal-session", "terminal-map")
            .expect("start terminal-session fixture");
        terminal
            .stop_session("terminal-session")
            .expect("terminalize session fixture");
        let (staged, gate) = settled_staged_worker(3);
        let error = terminal
            .start_session_with_staged_backend("terminal-session", "new-map", staged)
            .expect_err("stopped session id must remain terminal");
        assert_eq!(
            error,
            MappingLifecycleError::TerminalSession("terminal-session".to_owned())
        );
        assert!(gate.entered(), "terminal-session worker was not joined");
        assert_eq!(terminal.backend_owner_state_for_tests(), (false, false));

        let mut pending_retirement = MappingAdapter::new(sample_config());
        pending_retirement.set_pending_backend_retirement_for_tests(7);
        let (staged, gate) = settled_staged_worker(4);
        let error = pending_retirement
            .start_session_with_staged_backend("pending-session", "pending-map", staged)
            .expect_err("pending backend retirement must reject admission");
        assert!(
            matches!(error, MappingLifecycleError::Backend(message) if message.contains("retirement is still pending"))
        );
        assert!(gate.entered(), "pending-retirement worker was not joined");

        let mut session_exhausted = MappingAdapter::new(sample_config());
        session_exhausted.next_session_generation = u64::MAX;
        let (staged, gate) = settled_staged_worker(5);
        let error = session_exhausted
            .start_session_with_staged_backend("exhausted-session", "exhausted-map", staged)
            .expect_err("session generation exhaustion must reject admission");
        assert_eq!(error, MappingLifecycleError::GenerationExhausted);
        assert!(gate.entered(), "session-exhaustion worker was not joined");

        let mut backend_exhausted = MappingAdapter::new(sample_config());
        backend_exhausted.exhaust_backend_generation_for_tests();
        let (staged, gate) = settled_staged_worker(6);
        let error = backend_exhausted
            .start_session_with_staged_backend("backend-exhausted", "backend-map", staged)
            .expect_err("backend generation exhaustion must reject admission");
        assert!(
            matches!(error, MappingLifecycleError::Backend(message) if message.contains("generation exhausted"))
        );
        assert!(gate.entered(), "backend-exhaustion worker was not joined");

        let mut identity = MappingAdapter::new(sample_config());
        let (staged, gate) = settled_staged_worker(99);
        let error = identity
            .start_session_with_staged_backend("identity-session", "identity-map", staged)
            .expect_err("wrong-generation staged worker must be rejected");
        assert_eq!(
            error,
            MappingLifecycleError::Backend(
                "staged mapping backend worker identity mismatch".to_owned()
            )
        );
        assert!(gate.entered(), "identity-mismatch worker was not joined");
        assert_eq!(identity.backend_owner_state_for_tests(), (false, false));

        let mut failed_join = MappingAdapter::new(sample_config());
        let failing_gate = MappingBackendRetirementTestGate::failing();
        let staged = MappingBackendWorker::start_with_retirement_test_gate(
            MappingBackendSpec::Voxel,
            99,
            failing_gate.clone(),
        )
        .expect("start failing staged worker");
        let error = failed_join
            .start_session_with_staged_backend("identity-session", "identity-map", staged)
            .expect_err("identity rejection with a failed join must remain an error");
        assert!(failing_gate.entered(), "failed-join worker was not joined");
        let detail = error.to_string();
        assert!(detail.contains("staged mapping backend worker identity mismatch"));
        assert!(detail.contains("rejected worker join failed"));
        assert_eq!(failed_join.backend_owner_state_for_tests(), (false, false));
    }

    #[test]
    fn blocked_rejected_staged_worker_returns_with_the_exact_retained_join_owner() {
        let mut adapter = MappingAdapter::new(sample_config());
        let gate = MappingBackendRetirementTestGate::blocked();
        let staged = MappingBackendWorker::start_with_retirement_test_gate(
            MappingBackendSpec::Voxel,
            99,
            gate.clone(),
        )
        .expect("start blocked staged worker");
        let thread_id = staged.thread_id_for_tests();

        let started = Instant::now();
        let error = adapter
            .start_session_with_staged_backend("identity-session", "identity-map", staged)
            .expect_err("wrong-generation staged worker must be rejected");
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "blocked staged-worker rejection must return within its bounded join deadline"
        );
        let detail = error.to_string();
        assert!(detail.contains("staged mapping backend worker identity mismatch"));
        assert!(detail.contains("pre-install stop remained pending"));
        assert!(detail.contains("exact join owner retained until process exit"));
        assert!(
            mapping_backend_process_exit_retains_thread_for_tests(thread_id),
            "bounded rejection must retain the exact unfinished OS join handle"
        );
        assert_eq!(adapter.backend_owner_state_for_tests(), (false, false));

        gate.release();
        join_retained_mapping_backend_thread_for_tests(thread_id)
            .expect("released retained staged worker joins exactly once");
        assert!(!mapping_backend_process_exit_retains_thread_for_tests(
            thread_id
        ));
    }

    #[test]
    fn failed_stop_keeps_worker_owned_until_terminal_fallback_retires_it() {
        let mut adapter = active_adapter(sample_config());
        let session = adapter.snapshot();
        adapter.exhaust_backend_generation_for_tests();

        assert!(matches!(
            adapter.stop_session_detached(&session.session_id),
            Err(MappingLifecycleError::GenerationExhausted)
        ));
        assert!(adapter.snapshot().active);
        assert!(adapter.mapping_worker.is_some());
        assert!(adapter.pending_backend_retirement.is_none());

        let transition = adapter
            .force_stop_started_session_detached(&session.session_id, session.session_generation)
            .expect("process-fatal fallback terminalizes the exact session");
        assert!(!transition.snapshot.active);
        assert_eq!(transition.snapshot.phase, MappingSessionPhase::Stopped);
        let retirement = transition
            .retirement
            .expect("fallback owns worker retirement");
        let generation = retirement.generation();
        let result = retirement.stop_and_join();
        adapter
            .complete_backend_retirement(generation, result)
            .expect("fallback retirement joins and acknowledges worker");
        assert!(adapter.mapping_worker.is_none());
        assert!(adapter.pending_backend_retirement.is_none());
    }

    #[tokio::test]
    async fn detached_retirement_owner_still_acknowledges_completion() {
        let adapter = Arc::new(parking_lot::Mutex::new(active_adapter(sample_config())));
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let retirement = adapter
            .lock()
            .stop_session_detached("test-session")
            .expect("detach active worker")
            .retirement
            .expect("owned retirement");
        let lifecycle_guard = lifecycle_lock.lock().await;
        let observer = spawn_backend_retirement_owner(
            Arc::clone(&adapter),
            Arc::clone(&lifecycle_lock),
            retirement,
        );

        drop(observer);
        drop(lifecycle_guard);

        tokio::time::timeout(std::time::Duration::from_secs(1), async {
            loop {
                let start_result = adapter.lock().start_session("test-session-2", "test-map");
                match start_result {
                    Ok(started) => {
                        assert_eq!(started.session_generation, 2);
                        break;
                    }
                    Err(MappingLifecycleError::Backend(message))
                        if message.contains("retirement is still pending") =>
                    {
                        tokio::task::yield_now().await;
                    }
                    Err(error) => panic!("unexpected next-session failure: {error}"),
                }
            }
        })
        .await
        .expect("owned retirement must acknowledge after observer cancellation");
    }

    #[test]
    fn checkpoint_export_lease_rejects_stop_start_aba() {
        let mut adapter = active_adapter(sample_config());
        let lease_a = adapter
            .begin_backend_checkpoint_export()
            .expect("begin session-one export");
        adapter
            .stop_session("test-session")
            .expect("stop session one");
        adapter
            .start_session("test-session-2", "test-map")
            .expect("start session two");
        let lease_b = adapter
            .begin_backend_checkpoint_export()
            .expect("begin session-two export");

        let error = adapter
            .backend_checkpoint_client_if_drained(&lease_a)
            .expect_err("stale lease must not observe the new export");
        assert!(error.contains("stale"));
        adapter.finish_backend_checkpoint_export(&lease_a);
        assert!(adapter.checkpoint_export_pending_for_tests());
        assert!(adapter
            .backend_checkpoint_client_if_drained(&lease_b)
            .expect("live lease")
            .is_some());

        adapter.finish_backend_checkpoint_export(&lease_b);
        assert!(!adapter.checkpoint_export_pending_for_tests());
        adapter
            .stop_session("test-session-2")
            .expect("stop session two");
    }

    #[test]
    fn checkpoint_export_finish_clears_only_lease_owned_flush_token() {
        let mut adapter = active_adapter(sample_config());
        adapter.next_source_frame_sequence = 1;
        adapter.latest_pointcloud_timestamp_ns = 1_000_000_000;
        let lease = adapter
            .begin_backend_checkpoint_export()
            .expect("begin checkpoint export");
        let prepared = adapter
            .prepare_backend_checkpoint_flush(&lease, 1_000_000_001)
            .expect("prepare checkpoint flush")
            .expect("flush token");
        assert_eq!(adapter.in_flight_compute.as_ref(), Some(&prepared.token));
        adapter.finish_backend_checkpoint_export(&lease);
        assert!(adapter.in_flight_compute.is_none());
        assert!(!adapter.checkpoint_export_pending_for_tests());

        let lease = adapter
            .begin_backend_checkpoint_export()
            .expect("begin second checkpoint export");
        let prepared = adapter
            .prepare_backend_checkpoint_flush(&lease, 1_000_000_002)
            .expect("prepare second checkpoint flush")
            .expect("second flush token");
        let mut unrelated = prepared.token;
        unrelated.token_id = unrelated.token_id.saturating_add(1);
        adapter.in_flight_compute = Some(unrelated.clone());
        adapter.finish_backend_checkpoint_export(&lease);
        assert_eq!(adapter.in_flight_compute.as_ref(), Some(&unrelated));
        assert!(!adapter.checkpoint_export_pending_for_tests());
        adapter.in_flight_compute = None;
        adapter
            .stop_session("test-session")
            .expect("stop checkpoint test session");
    }
}
