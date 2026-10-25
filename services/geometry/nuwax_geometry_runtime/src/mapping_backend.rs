//! Geometry-owned mapping kernels selected by the validated owner profile.


#[path = "mapping_backend_sidecar.rs"]
mod sidecar;

use std::cell::RefCell;
use std::collections::VecDeque;
use std::env;
use std::fmt::Display;
use std::mem::size_of;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex, OnceLock};
use std::thread;
use std::time::Duration;
use std::time::Instant;

use nalgebra::Point3;
use nuwax_mapping::cost_mapper::{CostMapper, OccupancyConfig, PointCloud2Like};
use nuwax_mapping::OccupancyGrid;
use nuwax_mapping_raytracing::voxel_ray_tracer::{
    append_voxel_map_checkpoint, decode_voxel_map_checkpoint_for_config,
    try_update_map, voxel_map_checkpoint_encoded_len, Config, PreparedRayUpdate, VoxelMap,
    VOXEL_MAP_CHECKPOINT_ENTRY_BYTES, VOXEL_MAP_CHECKPOINT_HEADER_BYTES,
};
#[cfg(test)]
use nuwax_mapping_raytracing::voxel_ray_tracer::iter_global_points;

const WORKER_QUEUE_CAPACITY: usize = 1;
const WORKER_RESOLUTION_TIMEOUT: Duration = Duration::from_secs(15);
const WORKER_RESOLUTION_STOP_POLL_INTERVAL: Duration = Duration::from_millis(1);
#[cfg(not(test))]
const PREINSTALL_WORKER_JOIN_TIMEOUT: Duration = Duration::from_secs(3);
#[cfg(test)]
const PREINSTALL_WORKER_JOIN_TIMEOUT: Duration = Duration::from_millis(50);
const PREINSTALL_WORKER_JOIN_POLL_INTERVAL: Duration = Duration::from_millis(1);
// 在线最多30次/秒，仍只消费真实新输入；历史回放逐帧累积水平观测。
pub(crate) const MAPPING_CANDIDATE_INTERVAL_NS: u64 = 33_333_333;
pub(crate) const MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION: u32 = 1;
pub(crate) const MAX_MAPPING_BACKEND_CHECKPOINT_BYTES: usize = 64 * 1024 * 1024;
const MAPPING_BACKEND_CHECKPOINT_MAGIC: [u8; 8] = *b"NWMBCKP\0";
const MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES: usize = 16;
const MAPPING_BACKEND_VOXEL_METADATA_BYTES: usize = 4 * size_of::<u64>();
const MAPPING_BACKEND_VOXEL_HEADER_BYTES: usize =
    MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + MAPPING_BACKEND_VOXEL_METADATA_BYTES;
const MAPPING_BACKEND_VOXEL_POINT_BYTES: usize = 3 * size_of::<f64>();
const MAX_RETAINED_VOXEL_POINTS: usize = (MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
    - MAPPING_BACKEND_VOXEL_HEADER_BYTES)
    / MAPPING_BACKEND_VOXEL_POINT_BYTES;
const MAPPING_BACKEND_RAY_CONFIG_BYTES: usize = 12 * size_of::<u32>();
const MAPPING_BACKEND_RAY_METADATA_BYTES: usize = 4 * size_of::<u64>();
const MAPPING_BACKEND_RAY_HEADER_BYTES: usize = MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES
    + MAPPING_BACKEND_RAY_CONFIG_BYTES
    + MAPPING_BACKEND_RAY_METADATA_BYTES;
const MAPPING_BACKEND_OCCUPANCY_FIXED_BYTES: usize = 2 * size_of::<u32>()
    + size_of::<f64>()
    + 3 * size_of::<f64>()
    + 4 * size_of::<f64>()
    + size_of::<f64>()
    + size_of::<u32>()
    + size_of::<u64>();
const MAPPING_BACKEND_KIND_VOXEL: u8 = 0;
const MAPPING_BACKEND_KIND_RAYTRACING: u8 = 1;

thread_local! {
    static MAPPING_BACKEND_WORKER_STOPPED: RefCell<Option<Arc<AtomicBool>>> =
        const { RefCell::new(None) };
}

pub(crate) fn run_requested_sidecar() -> Option<Result<(), String>> {
    sidecar::run_requested_sidecar()
}

#[derive(Clone, Copy, PartialEq)]
struct RayConfigCheckpoint {
    voxel_size: f32,
    max_range: f32,
    ray_subsample: u32,
    shadow_depth: f32,
    grace_depth: f32,
    min_health: i32,
    max_health: i32,
    graze_cos: f32,
    recency_window: u32,
    emit_every: u32,
    global_emit_every: u32,
    region_percentile: f32,
}

pub(crate) const RAY_VOXEL_SIZE_ENV: &str = "NUWAX_MAPPING_RAYTRACING_VOXEL_SIZE";
pub(crate) const RAY_MAX_RANGE_ENV: &str = "NUWAX_MAPPING_RAYTRACING_MAX_RANGE";
pub(crate) const RAY_SUBSAMPLE_ENV: &str = "NUWAX_MAPPING_RAYTRACING_RAY_SUBSAMPLE";
pub(crate) const RAY_SHADOW_DEPTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_SHADOW_DEPTH";
pub(crate) const RAY_GRACE_DEPTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_GRACE_DEPTH";
pub(crate) const RAY_MIN_HEALTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_MIN_HEALTH";
pub(crate) const RAY_MAX_HEALTH_ENV: &str = "NUWAX_MAPPING_RAYTRACING_MAX_HEALTH";
pub(crate) const RAY_GRAZE_COS_ENV: &str = "NUWAX_MAPPING_RAYTRACING_GRAZE_COS";
pub(crate) const RAY_RECENCY_WINDOW_ENV: &str = "NUWAX_MAPPING_RAYTRACING_RECENCY_WINDOW";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RaytracingBackendSpec {
    voxel_size: f32,
    max_range: f32,
    ray_subsample: u32,
    shadow_depth: f32,
    grace_depth: f32,
    min_health: i32,
    max_health: i32,
    graze_cos: f32,
    recency_window: u32,
}

impl Default for RaytracingBackendSpec {
    fn default() -> Self {
        Self {
            voxel_size: 0.1,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: -2,
            max_health: 1,
            graze_cos: 0.7,
            recency_window: 15,
        }
    }
}

impl RaytracingBackendSpec {
    fn from_env() -> Result<Self, String> {
        Self::from_fallible_lookup(|key| match env::var(key) {
            Ok(value) => Ok(Some(value)),
            Err(env::VarError::NotPresent) => Ok(None),
            Err(env::VarError::NotUnicode(_)) => Err(format!(
                "RAYTRACING_CONFIG_INVALID: {key} must contain valid Unicode"
            )),
        })
    }

    #[cfg(test)]
    fn from_lookup(mut lookup: impl FnMut(&str) -> Option<String>) -> Result<Self, String> {
        Self::from_fallible_lookup(|key| Ok(lookup(key)))
    }

    fn from_fallible_lookup(
        mut lookup: impl FnMut(&str) -> Result<Option<String>, String>,
    ) -> Result<Self, String> {
        let defaults = Self::default();
        let spec = Self {
            voxel_size: parse_value(&mut lookup, RAY_VOXEL_SIZE_ENV, defaults.voxel_size)?,
            max_range: parse_value(&mut lookup, RAY_MAX_RANGE_ENV, defaults.max_range)?,
            ray_subsample: parse_value(&mut lookup, RAY_SUBSAMPLE_ENV, defaults.ray_subsample)?,
            shadow_depth: parse_value(&mut lookup, RAY_SHADOW_DEPTH_ENV, defaults.shadow_depth)?,
            grace_depth: parse_value(&mut lookup, RAY_GRACE_DEPTH_ENV, defaults.grace_depth)?,
            min_health: parse_value(&mut lookup, RAY_MIN_HEALTH_ENV, defaults.min_health)?,
            max_health: parse_value(&mut lookup, RAY_MAX_HEALTH_ENV, defaults.max_health)?,
            graze_cos: parse_value(&mut lookup, RAY_GRAZE_COS_ENV, defaults.graze_cos)?,
            recency_window: parse_value(
                &mut lookup,
                RAY_RECENCY_WINDOW_ENV,
                defaults.recency_window,
            )?,
        };
        spec.validate()?;
        Ok(spec)
    }

    fn validate(self) -> Result<(), String> {
        let mut empty = VoxelMap::default();
        try_update_map(&mut empty, (0.0, 0.0, 0.0), &[], &self.config())
            .map(|_| ())
            .map_err(|error| format!("RAYTRACING_CONFIG_INVALID: {error}"))
    }

    fn config(self) -> Config {
        Config {
            voxel_size: self.voxel_size,
            max_range: self.max_range,
            ray_subsample: self.ray_subsample,
            shadow_depth: self.shadow_depth,
            grace_depth: self.grace_depth,
            min_health: self.min_health,
            max_health: self.max_health,
            graze_cos: self.graze_cos,
            recency_window: self.recency_window,
            emit_every: 0,
            global_emit_every: 0,
            region_percentile: 95.0,
        }
    }
}

fn parse_value<T>(
    lookup: &mut impl FnMut(&str) -> Result<Option<String>, String>,
    key: &str,
    default: T,
) -> Result<T, String>
where
    T: Copy + Display + FromStr,
    T::Err: Display,
{
    let Some(raw) = lookup(key)? else {
        return Ok(default);
    };
    let value = raw.trim();
    if value.is_empty() {
        return Err(format!(
            "RAYTRACING_CONFIG_INVALID: {key} must not be empty"
        ));
    }
    value.parse::<T>().map_err(|error| {
        format!("RAYTRACING_CONFIG_INVALID: {key}={raw:?} cannot be parsed: {error}")
    })
}

impl From<&Config> for RayConfigCheckpoint {
    fn from(config: &Config) -> Self {
        Self {
            voxel_size: config.voxel_size,
            max_range: config.max_range,
            ray_subsample: config.ray_subsample,
            shadow_depth: config.shadow_depth,
            grace_depth: config.grace_depth,
            min_health: config.min_health,
            max_health: config.max_health,
            graze_cos: config.graze_cos,
            recency_window: config.recency_window,
            emit_every: config.emit_every,
            global_emit_every: config.global_emit_every,
            region_percentile: config.region_percentile,
        }
    }
}

fn ray_config_checkpoint_matches(config: &Config, checkpoint: RayConfigCheckpoint) -> bool {
    config.voxel_size.to_bits() == checkpoint.voxel_size.to_bits()
        && config.max_range.to_bits() == checkpoint.max_range.to_bits()
        && config.ray_subsample == checkpoint.ray_subsample
        && config.shadow_depth.to_bits() == checkpoint.shadow_depth.to_bits()
        && config.grace_depth.to_bits() == checkpoint.grace_depth.to_bits()
        && config.min_health == checkpoint.min_health
        && config.max_health == checkpoint.max_health
        && config.graze_cos.to_bits() == checkpoint.graze_cos.to_bits()
        && config.recency_window == checkpoint.recency_window
        && config.emit_every == checkpoint.emit_every
        && config.global_emit_every == checkpoint.global_emit_every
        && config.region_percentile.to_bits() == checkpoint.region_percentile.to_bits()
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[doc(hidden)]
pub enum MappingBackendSpec {
    SlamToolbox,
    Voxel,
    Raytracing(RaytracingBackendSpec),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[doc(hidden)]
pub enum MappingBackendKind {
    SlamToolbox,
    Voxel,
    Raytracing,
}

impl MappingBackendKind {
    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::SlamToolbox => "slam_toolbox",
            Self::Voxel => "voxel",
            Self::Raytracing => "raytracing",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct MappingComputeToken {
    pub(crate) token_id: u128,
    pub(crate) session_id: String,
    pub(crate) session_generation: u64,
    pub(crate) backend_generation: u64,
    pub(crate) source_frame_sequence: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) prepared_at_ns: u64,
    pub(crate) base_occupancy_revision: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MappingBackendInput {
    pub(crate) session_id: String,
    pub(crate) session_generation: u64,
    pub(crate) backend_generation: u64,
    pub(crate) frame_sequence: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) received_at_ns: u64,
    pub(crate) registered_frame_id: String,
    pub(crate) sensor_origin_map: [f64; 3],
    pub(crate) dynamic_frame: Option<crate::dynamic_scene::MappedFrameDecision>,
    pub(crate) registered_points_map: Vec<Point3<f64>>,
}

#[derive(Debug)]
pub(crate) struct MappingBackendCandidate {
    pub(crate) session_id: String,
    pub(crate) session_generation: u64,
    pub(crate) backend_generation: u64,
    pub(crate) source_frame_sequence: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) received_at_ns: u64,
    pub(crate) backend_kind: MappingBackendKind,
    pub(crate) live_window_occupancy: OccupancyGrid,
    pub(crate) latest_frame_occupancy: OccupancyGrid,
    pub(crate) accepted_points_map: Vec<Point3<f64>>,
    pub(crate) compute_elapsed_us: u64,
}

#[derive(Clone, Copy, Debug)]
#[doc(hidden)]
pub struct MappingBackendComputeConfig {
    pub resolution: f64,
    pub initial_safe_radius_meters: f32,
    pub window_capacity: usize,
}

#[derive(Debug)]
pub(crate) struct MappingBackendCompletion {
    pub(crate) token: MappingComputeToken,
    pub(crate) accepted_points_map: Vec<Point3<f64>>,
    pub(crate) result: Result<Option<MappingBackendCandidate>, String>,
    pub(crate) resolution: MappingBackendCommitHandle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MappingBackendDecisionKind {
    Commit,
    Abort,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MappingBackendDecision {
    transaction_id: u128,
    kind: MappingBackendDecisionKind,
}

pub(crate) struct MappingBackendCommitHandle {
    transaction_id: u128,
    sender: Option<mpsc::SyncSender<MappingBackendDecision>>,
}

impl std::fmt::Debug for MappingBackendCommitHandle {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("MappingBackendCommitHandle")
            .field("transaction_id", &self.transaction_id)
            .field("resolved", &self.sender.is_none())
            .finish()
    }
}

impl MappingBackendCommitHandle {
    fn new(transaction_id: u128, sender: mpsc::SyncSender<MappingBackendDecision>) -> Self {
        Self {
            transaction_id,
            sender: Some(sender),
        }
    }

    pub(crate) const fn transaction_id(&self) -> u128 {
        self.transaction_id
    }

    pub(crate) fn commit(mut self) -> Result<(), String> {
        self.resolve(MappingBackendDecisionKind::Commit)
    }

    pub(crate) fn abort(mut self) -> Result<(), String> {
        self.resolve(MappingBackendDecisionKind::Abort)
    }

    fn resolve(&mut self, kind: MappingBackendDecisionKind) -> Result<(), String> {
        let sender = self
            .sender
            .as_ref()
            .ok_or_else(|| "mapping backend completion was already resolved".to_owned())?;
        let decision = MappingBackendDecision {
            transaction_id: self.transaction_id,
            kind,
        };
        match sender.send(decision) {
            Ok(()) => {
                self.sender.take();
                Ok(())
            }
            Err(_) => {
                // Keep the sender pending.  Returning the error consumes the
                // commit handle, whose Drop implementation performs the only
                // available best-effort Abort delivery.  If the receiver is
                // already closed, that abort is unavailable and the caller
                // receives a terminal decision-unavailable error; no commit or
                // abort is claimed to have been delivered.
                Err("mapping backend worker closed before completion resolution; abort decision unavailable".to_owned())
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn detached_for_tests(transaction_id: u128) -> Self {
        let (sender, receiver) = mpsc::sync_channel(1);
        drop(receiver);
        Self::new(transaction_id, sender)
    }
}

impl Drop for MappingBackendCommitHandle {
    fn drop(&mut self) {
        let Some(sender) = self.sender.take() else {
            return;
        };
        let _ = sender.try_send(MappingBackendDecision {
            transaction_id: self.transaction_id,
            kind: MappingBackendDecisionKind::Abort,
        });
    }
}

fn mapping_backend_resolution_channel(
    transaction_id: u128,
) -> (
    MappingBackendCommitHandle,
    mpsc::Receiver<MappingBackendDecision>,
) {
    let (sender, receiver) = mpsc::sync_channel(1);
    (
        MappingBackendCommitHandle::new(transaction_id, sender),
        receiver,
    )
}

fn mapping_backend_resolution_commits(
    transaction_id: u128,
    receiver: &mpsc::Receiver<MappingBackendDecision>,
) -> bool {
    let deadline = Instant::now() + WORKER_RESOLUTION_TIMEOUT;
    loop {
        let stopped = MAPPING_BACKEND_WORKER_STOPPED.with(|worker_stopped| {
            worker_stopped
                .borrow()
                .as_ref()
                .is_some_and(|stopped| stopped.load(Ordering::Acquire))
        });
        if stopped {
            return false;
        }
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return false;
        }
        match receiver.recv_timeout(WORKER_RESOLUTION_STOP_POLL_INTERVAL.min(remaining)) {
            Ok(MappingBackendDecision {
                transaction_id: resolved_id,
                kind: MappingBackendDecisionKind::Commit,
            }) => return resolved_id == transaction_id,
            Ok(_) | Err(mpsc::RecvTimeoutError::Disconnected) => return false,
            Err(mpsc::RecvTimeoutError::Timeout) => {}
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum MappingBackendSubmitError {
    QueueFull,
    WorkerStopped,
}

struct MappingBackendJob {
    token: MappingComputeToken,
    input: MappingBackendInput,
    completion: tokio::sync::oneshot::Sender<MappingBackendCompletion>,
}

struct MappingBackendExportJob {
    completion: tokio::sync::oneshot::Sender<Result<Vec<u8>, String>>,
}

struct MappingBackendRestoreJob {
    transaction_id: u128,
    input: MappingBackendRestoreInput,
    completion: tokio::sync::oneshot::Sender<Result<MappingBackendCommitHandle, String>>,
}

/// Immutable encoded checkpoint storage owned by a queued restore operation.
/// Kept independent of the RPC binary so the reusable offline library uses the same worker.
pub(crate) trait MappingBackendEncodedOwner: Send + Sync {
    fn checkpoint_bytes(&self) -> &[u8];
}

enum MappingBackendRestoreInput {
    Encoded(Vec<u8>),
    OwnedEncoded(Arc<dyn MappingBackendEncodedOwner>),
    Prepared(PreparedMappingBackendRestore),
}

/// 已在后台完整解码验证的内核；跨入提交区时只转移拥有权。
pub(crate) struct PreparedMappingBackendRestore {
    kernel: MappingKernel,
    encoded: Vec<u8>,
    spec: MappingBackendSpec,
    config: MappingBackendComputeConfig,
}

/// 私有重放已经逐帧验证并完成物化，同进程提交直接转移内核。
/// 射线隔离进程仍需要原协议字节，但无需在发送端再次解码同一内核。
pub(crate) fn prepare_computed_mapping_backend_restore(
    spec: MappingBackendSpec,
    config: MappingBackendComputeConfig,
    kernel: MappingKernel,
) -> Result<PreparedMappingBackendRestore, String> {
    let encoded = match (spec, &kernel) {
        (MappingBackendSpec::Voxel, MappingKernel::Voxel(voxel)) => {
            // 省去编码不代表绕过原检查点的容量界限；仍须能被正式保存和恢复。
            let length = mapping_backend_voxel_checkpoint_encoded_len(voxel)?;
            if length > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
                return Err("私有体素内核超过可保存的检查点容量".into());
            }
            Vec::new()
        }
        (MappingBackendSpec::Raytracing(_), MappingKernel::Raytracing(_)) => {
            export_mapping_kernel_checkpoint(&kernel)?
        }
        _ => return Err("私有重放内核与地图后端类型不匹配".into()),
    };
    Ok(PreparedMappingBackendRestore {
        kernel,
        encoded,
        spec,
        config,
    })
}

struct MappingBackendFlushJob {
    token: MappingComputeToken,
    completion: tokio::sync::oneshot::Sender<MappingBackendCompletion>,
}

enum MappingBackendCommand {
    Compute(MappingBackendJob),
    Flush(MappingBackendFlushJob),
    ExportCheckpoint(MappingBackendExportJob),
    RestoreCheckpoint(MappingBackendRestoreJob),
    Stop,
}

#[derive(Clone)]
pub(crate) struct MappingBackendClient {
    backend_generation: u64,
    backend_kind: MappingBackendKind,
    sender: mpsc::SyncSender<MappingBackendCommand>,
    stopped: Arc<AtomicBool>,
}

impl std::fmt::Debug for MappingBackendClient {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("MappingBackendClient")
            .field("backend_generation", &self.backend_generation)
            .field("backend_kind", &self.backend_kind)
            .field("stopped", &self.stopped.load(Ordering::Acquire))
            .finish_non_exhaustive()
    }
}

#[derive(Debug)]
pub(crate) struct MappingBackendWorker {
    client: MappingBackendClient,
    join: Option<thread::JoinHandle<()>>,
}

#[derive(Debug)]
pub(crate) struct MappingBackendJoinHandle {
    join: Option<thread::JoinHandle<()>>,
}

fn mapping_backend_process_exit_retention() -> &'static Mutex<Vec<thread::JoinHandle<()>>> {
    static RETAINED: OnceLock<Mutex<Vec<thread::JoinHandle<()>>>> = OnceLock::new();
    RETAINED.get_or_init(|| Mutex::new(Vec::new()))
}

fn retain_mapping_backend_join_until_process_exit(join: thread::JoinHandle<()>) {
    mapping_backend_process_exit_retention()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .push(join);
}

#[cfg(test)]
pub(crate) fn mapping_backend_process_exit_retains_thread_for_tests(
    thread_id: thread::ThreadId,
) -> bool {
    mapping_backend_process_exit_retention()
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
        .iter()
        .any(|join| join.thread().id() == thread_id)
}

#[cfg(test)]
pub(crate) fn join_retained_mapping_backend_thread_for_tests(
    thread_id: thread::ThreadId,
) -> Result<(), String> {
    let join = {
        let mut retained = mapping_backend_process_exit_retention()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let index = retained
            .iter()
            .position(|join| join.thread().id() == thread_id)
            .ok_or_else(|| format!("retained Mapping backend thread {thread_id:?} is missing"))?;
        retained.swap_remove(index)
    };
    join.join()
        .map_err(|_| "retained mapping backend worker panicked during shutdown".to_owned())
}

impl MappingBackendJoinHandle {
    pub(crate) fn is_finished(&self) -> bool {
        self.join.as_ref().is_none_or(|join| join.is_finished())
    }

    pub(crate) fn join(mut self) -> Result<(), String> {
        self.join
            .take()
            .expect("Mapping backend final join handle is present")
            .join()
            .map_err(|_| "mapping backend worker panicked during shutdown".to_owned())
    }

    #[cfg(test)]
    pub(crate) fn thread_id(&self) -> thread::ThreadId {
        self.join
            .as_ref()
            .expect("Mapping backend final join handle is present")
            .thread()
            .id()
    }
}

impl Drop for MappingBackendJoinHandle {
    fn drop(&mut self) {
        if let Some(join) = self.join.take() {
            retain_mapping_backend_join_until_process_exit(join);
        }
    }
}

#[cfg(test)]
#[derive(Clone, Debug, Default)]
pub(crate) struct MappingBackendTestGate {
    entered: Arc<AtomicBool>,
    released: Arc<AtomicBool>,
}

#[cfg(test)]
#[derive(Clone, Debug)]
pub(crate) struct MappingBackendRetirementTestGate {
    entered: Arc<AtomicBool>,
    released: Arc<AtomicBool>,
    panic_after_release: bool,
}

#[cfg(test)]
impl MappingBackendRetirementTestGate {
    pub(crate) fn blocked() -> Self {
        Self {
            entered: Arc::new(AtomicBool::new(false)),
            released: Arc::new(AtomicBool::new(false)),
            panic_after_release: false,
        }
    }

    pub(crate) fn failing() -> Self {
        Self {
            entered: Arc::new(AtomicBool::new(false)),
            released: Arc::new(AtomicBool::new(true)),
            panic_after_release: true,
        }
    }

    pub(crate) fn entered(&self) -> bool {
        self.entered.load(Ordering::Acquire)
    }

    pub(crate) fn release(&self) {
        self.released.store(true, Ordering::Release);
    }
}

#[cfg(test)]
impl MappingBackendTestGate {
    pub(crate) fn entered(&self) -> bool {
        self.entered.load(Ordering::Acquire)
    }

    pub(crate) fn release(&self) {
        self.released.store(true, Ordering::Release);
    }
}

impl MappingBackendWorker {
    pub(crate) fn start(
        spec: MappingBackendSpec,
        backend_generation: u64,
        compute_config: MappingBackendComputeConfig,
    ) -> Result<Self, String> {
        if matches!(spec, MappingBackendSpec::SlamToolbox) {
            return Err("二维算法由Geometry独立进程拥有方管理".into());
        }
        Self::start_inner(spec, backend_generation, compute_config, None)
    }

    #[cfg(test)]
    pub(crate) fn start_with_test_gate(
        spec: MappingBackendSpec,
        backend_generation: u64,
        compute_config: MappingBackendComputeConfig,
        gate: MappingBackendTestGate,
    ) -> Result<Self, String> {
        Self::start_inner(spec, backend_generation, compute_config, Some(gate))
    }

    fn start_inner(
        spec: MappingBackendSpec,
        backend_generation: u64,
        compute_config: MappingBackendComputeConfig,
        #[cfg(test)] gate: Option<MappingBackendTestGate>,
        #[cfg(not(test))] _gate: Option<()>,
    ) -> Result<Self, String> {
        if backend_generation == 0 {
            return Err("mapping backend generation must be positive".to_owned());
        }
        let backend_kind = spec.kind();
        let supervised_sidecar = if sidecar::requires_supervision(spec) {
            let MappingBackendSpec::Raytracing(ray_spec) = spec else {
                return Err("only raytracing may use the mapping sidecar".to_owned());
            };
            Some(sidecar::SupervisedRaySidecar::start(
                ray_spec,
                compute_config,
            )?)
        } else {
            None
        };
        let (sender, receiver) = mpsc::sync_channel(WORKER_QUEUE_CAPACITY);
        let stopped = Arc::new(AtomicBool::new(false));
        let worker_stopped = stopped.clone();
        let join = thread::Builder::new()
            .name(format!(
                "mapping-{}-{backend_generation}",
                backend_kind.stable_name()
            ))
            .spawn(move || {
                MAPPING_BACKEND_WORKER_STOPPED.with(|worker_stopped_slot| {
                    *worker_stopped_slot.borrow_mut() = Some(Arc::clone(&worker_stopped));
                });
                if let Some(sidecar) = supervised_sidecar {
                    sidecar::run_worker_loop(sidecar, receiver, worker_stopped);
                    return;
                }
                let mut kernel = spec.into_kernel(compute_config.window_capacity);
                let mut last_input = None;
                while let Ok(command) = receiver.recv() {
                    if worker_stopped.load(Ordering::Acquire) {
                        break;
                    }
                    match command {
                        MappingBackendCommand::Compute(job) => {
                            #[cfg(test)]
                            if let Some(gate) = gate.as_ref() {
                                gate.entered.store(true, Ordering::Release);
                                while !gate.released.load(Ordering::Acquire)
                                    && !worker_stopped.load(Ordering::Acquire)
                                {
                                    thread::park_timeout(Duration::from_millis(1));
                                }
                                if worker_stopped.load(Ordering::Acquire) {
                                    break;
                                }
                            }
                            let result =
                                validate_backend_input(&kernel, &job.input).and_then(|()| {
                                    compute_candidate_in_place(
                                        &mut kernel,
                                        &job.input,
                                        compute_config,
                                    )
                                });
                            if result.is_ok() && !job.input.registered_points_map.is_empty() {
                                last_input = Some(job.input.clone());
                            }
                            let can_commit = result.is_ok();
                            let transaction_id = job.token.token_id;
                            let (resolution, resolution_rx) =
                                mapping_backend_resolution_channel(transaction_id);
                            let delivered = job
                                .completion
                                .send(MappingBackendCompletion {
                                    token: job.token,
                                    accepted_points_map: job.input.registered_points_map.clone(),
                                    result,
                                    resolution,
                                })
                                .is_ok();
                            let committed = delivered
                                && can_commit
                                && mapping_backend_resolution_commits(
                                    transaction_id,
                                    &resolution_rx,
                                );
                            if !committed || worker_stopped.load(Ordering::Acquire) {
                                break;
                            }
                        }
                        MappingBackendCommand::Flush(job) => {
                            let result = match last_input.as_ref() {
                                Some(input) => {
                                    flush_candidate(&mut kernel, input, &job.token, compute_config)
                                }
                                None => Ok(None),
                            };
                            let can_commit = result.is_ok();
                            let transaction_id = job.token.token_id;
                            let (resolution, resolution_rx) =
                                mapping_backend_resolution_channel(transaction_id);
                            let delivered = job
                                .completion
                                .send(MappingBackendCompletion {
                                    token: job.token,
                                    accepted_points_map: Vec::new(),
                                    result,
                                    resolution,
                                })
                                .is_ok();
                            let committed = delivered
                                && can_commit
                                && mapping_backend_resolution_commits(
                                    transaction_id,
                                    &resolution_rx,
                                );
                            if !committed || worker_stopped.load(Ordering::Acquire) {
                                break;
                            }
                        }
                        MappingBackendCommand::ExportCheckpoint(job) => {
                            let _ = job
                                .completion
                                .send(export_mapping_kernel_checkpoint(&kernel));
                        }
                        MappingBackendCommand::RestoreCheckpoint(job) => {
                            let result = match job.input {
                                MappingBackendRestoreInput::Encoded(payload) => {
                                    restore_mapping_kernel_checkpoint(
                                        spec,
                                        compute_config,
                                        &payload,
                                    )
                                }
                                MappingBackendRestoreInput::OwnedEncoded(owner) => {
                                    restore_mapping_kernel_checkpoint(
                                        spec,
                                        compute_config,
                                        owner.checkpoint_bytes(),
                                    )
                                }
                                MappingBackendRestoreInput::Prepared(prepared) => {
                                    if prepared.spec == spec
                                        && prepared.config.window_capacity
                                            == compute_config.window_capacity
                                        && prepared.config.resolution.to_bits()
                                            == compute_config.resolution.to_bits()
                                        && prepared.config.initial_safe_radius_meters.to_bits()
                                            == compute_config.initial_safe_radius_meters.to_bits()
                                    {
                                        Ok(prepared.kernel)
                                    } else {
                                        Err("预解码地图后端与当前配置不匹配".into())
                                    }
                                }
                            };
                            match result {
                                Ok(restored) => {
                                    let (resolution, resolution_rx) =
                                        mapping_backend_resolution_channel(job.transaction_id);
                                    let delivered = job.completion.send(Ok(resolution)).is_ok();
                                    if delivered
                                        && mapping_backend_resolution_commits(
                                            job.transaction_id,
                                            &resolution_rx,
                                        )
                                    {
                                        kernel = restored;
                                        last_input = None;
                                    }
                                }
                                Err(error) => {
                                    let _ = job.completion.send(Err(error));
                                }
                            }
                        }
                        MappingBackendCommand::Stop => break,
                    }
                }
                worker_stopped.store(true, Ordering::Release);
            })
            .map_err(|error| format!("mapping backend worker spawn failed: {error}"))?;
        let client = MappingBackendClient {
            backend_generation,
            backend_kind,
            sender,
            stopped,
        };
        Ok(Self {
            client,
            join: Some(join),
        })
    }

    pub(crate) fn into_stop_join_handle(mut self) -> Result<MappingBackendJoinHandle, String> {
        self.client.request_stop();
        self.join
            .take()
            .map(|join| MappingBackendJoinHandle { join: Some(join) })
            .ok_or_else(|| "mapping backend worker final join handle is missing".to_owned())
    }

    #[cfg(test)]
    pub(crate) fn start_with_retirement_test_gate(
        spec: MappingBackendSpec,
        backend_generation: u64,
        gate: MappingBackendRetirementTestGate,
    ) -> Result<Self, String> {
        if backend_generation == 0 {
            return Err("mapping backend generation must be positive".to_owned());
        }
        let backend_kind = spec.kind();
        let (sender, receiver) = mpsc::sync_channel(WORKER_QUEUE_CAPACITY);
        let stopped = Arc::new(AtomicBool::new(false));
        let worker_stopped = Arc::clone(&stopped);
        let join = thread::Builder::new()
            .name(format!(
                "mapping-{}-{backend_generation}-retirement-test",
                backend_kind.stable_name()
            ))
            .spawn(move || {
                let _receiver = receiver;
                while !worker_stopped.load(Ordering::Acquire) {
                    thread::park_timeout(Duration::from_millis(1));
                }
                gate.entered.store(true, Ordering::Release);
                while !gate.released.load(Ordering::Acquire) {
                    thread::park_timeout(Duration::from_millis(1));
                }
                if gate.panic_after_release {
                    panic!("injected mapping backend retirement failure");
                }
            })
            .map_err(|error| format!("mapping backend worker spawn failed: {error}"))?;
        Ok(Self {
            client: MappingBackendClient {
                backend_generation,
                backend_kind,
                sender,
                stopped,
            },
            join: Some(join),
        })
    }

    pub(crate) fn client(&self) -> MappingBackendClient {
        self.client.clone()
    }

    pub(crate) fn backend_generation(&self) -> u64 {
        self.client.backend_generation()
    }

    #[cfg(test)]
    pub(crate) fn thread_id_for_tests(&self) -> thread::ThreadId {
        self.join
            .as_ref()
            .expect("Mapping backend worker join handle is present")
            .thread()
            .id()
    }

    pub(crate) fn stop_and_join(mut self) -> Result<(), String> {
        self.client.request_stop();
        let Some(join) = self.join.take() else {
            return Ok(());
        };
        let generation = self.client.backend_generation();
        let join_handle = MappingBackendJoinHandle { join: Some(join) };
        let deadline = Instant::now() + PREINSTALL_WORKER_JOIN_TIMEOUT;
        while !join_handle.is_finished() {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                drop(join_handle);
                return Err(format!(
                    "mapping backend generation {generation} pre-install stop remained pending after {} ms; exact join owner retained until process exit",
                    PREINSTALL_WORKER_JOIN_TIMEOUT.as_millis()
                ));
            }
            thread::sleep(PREINSTALL_WORKER_JOIN_POLL_INTERVAL.min(remaining));
        }
        join_handle.join()
    }
}

impl MappingBackendClient {
    pub(crate) const fn backend_generation(&self) -> u64 {
        self.backend_generation
    }

    pub(crate) const fn backend_kind(&self) -> MappingBackendKind {
        self.backend_kind
    }

    pub(crate) fn submit(
        &self,
        token: MappingComputeToken,
        input: MappingBackendInput,
    ) -> Result<tokio::sync::oneshot::Receiver<MappingBackendCompletion>, MappingBackendSubmitError>
    {
        if self.stopped.load(Ordering::Acquire) {
            return Err(MappingBackendSubmitError::WorkerStopped);
        }
        let (completion, receiver) = tokio::sync::oneshot::channel();
        match self
            .sender
            .try_send(MappingBackendCommand::Compute(MappingBackendJob {
                token,
                input,
                completion,
            })) {
            Ok(()) => Ok(receiver),
            Err(mpsc::TrySendError::Full(_)) => Err(MappingBackendSubmitError::QueueFull),
            Err(mpsc::TrySendError::Disconnected(_)) => {
                self.stopped.store(true, Ordering::Release);
                Err(MappingBackendSubmitError::WorkerStopped)
            }
        }
    }

    pub(crate) fn export_checkpoint(
        &self,
    ) -> Result<tokio::sync::oneshot::Receiver<Result<Vec<u8>, String>>, MappingBackendSubmitError>
    {
        if self.stopped.load(Ordering::Acquire) {
            return Err(MappingBackendSubmitError::WorkerStopped);
        }
        let (completion, receiver) = tokio::sync::oneshot::channel();
        match self
            .sender
            .try_send(MappingBackendCommand::ExportCheckpoint(
                MappingBackendExportJob { completion },
            )) {
            Ok(()) => Ok(receiver),
            Err(mpsc::TrySendError::Full(_)) => Err(MappingBackendSubmitError::QueueFull),
            Err(mpsc::TrySendError::Disconnected(_)) => {
                self.stopped.store(true, Ordering::Release);
                Err(MappingBackendSubmitError::WorkerStopped)
            }
        }
    }

    pub(crate) fn flush(
        &self,
        token: MappingComputeToken,
    ) -> Result<tokio::sync::oneshot::Receiver<MappingBackendCompletion>, MappingBackendSubmitError>
    {
        if self.stopped.load(Ordering::Acquire) {
            return Err(MappingBackendSubmitError::WorkerStopped);
        }
        let (completion, receiver) = tokio::sync::oneshot::channel();
        match self
            .sender
            .try_send(MappingBackendCommand::Flush(MappingBackendFlushJob {
                token,
                completion,
            })) {
            Ok(()) => Ok(receiver),
            Err(mpsc::TrySendError::Full(_)) => Err(MappingBackendSubmitError::QueueFull),
            Err(mpsc::TrySendError::Disconnected(_)) => {
                self.stopped.store(true, Ordering::Release);
                Err(MappingBackendSubmitError::WorkerStopped)
            }
        }
    }

    pub(crate) fn restore_checkpoint(
        &self,
        payload: Vec<u8>,
    ) -> Result<
        tokio::sync::oneshot::Receiver<Result<MappingBackendCommitHandle, String>>,
        MappingBackendSubmitError,
    > {
        self.restore_input(MappingBackendRestoreInput::Encoded(payload))
    }

    /// Share admitted immutable bytes with the queued job; no encoded checkpoint clone.
    pub(crate) fn restore_owned_checkpoint<T: MappingBackendEncodedOwner + 'static>(
        &self,
        owner: T,
    ) -> Result<
        tokio::sync::oneshot::Receiver<Result<MappingBackendCommitHandle, String>>,
        MappingBackendSubmitError,
    > {
        self.restore_input(MappingBackendRestoreInput::OwnedEncoded(Arc::new(owner)))
    }

    pub(crate) fn restore_prepared_checkpoint(
        &self,
        prepared: PreparedMappingBackendRestore,
    ) -> Result<
        tokio::sync::oneshot::Receiver<Result<MappingBackendCommitHandle, String>>,
        MappingBackendSubmitError,
    > {
        self.restore_input(MappingBackendRestoreInput::Prepared(prepared))
    }

    fn restore_input(
        &self,
        input: MappingBackendRestoreInput,
    ) -> Result<
        tokio::sync::oneshot::Receiver<Result<MappingBackendCommitHandle, String>>,
        MappingBackendSubmitError,
    > {
        if self.stopped.load(Ordering::Acquire) {
            return Err(MappingBackendSubmitError::WorkerStopped);
        }
        let (completion, receiver) = tokio::sync::oneshot::channel();
        match self
            .sender
            .try_send(MappingBackendCommand::RestoreCheckpoint(
                MappingBackendRestoreJob {
                    transaction_id: uuid::Uuid::new_v4().as_u128(),
                    input,
                    completion,
                },
            )) {
            Ok(()) => Ok(receiver),
            Err(mpsc::TrySendError::Full(_)) => Err(MappingBackendSubmitError::QueueFull),
            Err(mpsc::TrySendError::Disconnected(_)) => {
                self.stopped.store(true, Ordering::Release);
                Err(MappingBackendSubmitError::WorkerStopped)
            }
        }
    }

    fn request_stop(&self) {
        if !self.stopped.swap(true, Ordering::AcqRel) {
            let _ = self.sender.try_send(MappingBackendCommand::Stop);
        }
    }
}

impl MappingBackendSpec {
    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub fn raytracing(
        voxel_size: f32,
        max_range: f32,
        ray_subsample: u32,
        shadow_depth: f32,
        grace_depth: f32,
        min_health: i32,
        max_health: i32,
        graze_cos: f32,
        recency_window: u32,
    ) -> Self {
        Self::Raytracing(RaytracingBackendSpec {
            voxel_size,
            max_range,
            ray_subsample,
            shadow_depth,
            grace_depth,
            min_health,
            max_health,
            graze_cos,
            recency_window,
        })
    }

    #[doc(hidden)]
    pub fn from_env(selected: MappingBackendKind) -> Result<Self, String> {
        match selected {
            MappingBackendKind::SlamToolbox => Ok(Self::SlamToolbox),
            MappingBackendKind::Voxel => Ok(Self::Voxel),
            MappingBackendKind::Raytracing => {
                Ok(Self::Raytracing(RaytracingBackendSpec::from_env()?))
            }
        }
    }

    #[cfg(test)]
    fn from_lookup(
        selected: MappingBackendKind,
        lookup: impl FnMut(&str) -> Option<String>,
    ) -> Result<Self, String> {
        match selected {
            MappingBackendKind::SlamToolbox => Ok(Self::SlamToolbox),
            MappingBackendKind::Voxel => Ok(Self::Voxel),
            MappingBackendKind::Raytracing => Ok(Self::Raytracing(
                RaytracingBackendSpec::from_lookup(lookup)?,
            )),
        }
    }

    pub(crate) const fn stable_name(self) -> &'static str {
        match self {
            Self::SlamToolbox => "slam_toolbox",
            Self::Voxel => "voxel",
            Self::Raytracing(_) => "raytracing",
        }
    }

    pub(crate) const fn kind(self) -> MappingBackendKind {
        match self {
            Self::SlamToolbox => MappingBackendKind::SlamToolbox,
            Self::Voxel => MappingBackendKind::Voxel,
            Self::Raytracing(_) => MappingBackendKind::Raytracing,
        }
    }

    pub(crate) fn into_kernel(self, window_capacity: usize) -> MappingKernel {
        match self {
            Self::SlamToolbox => unreachable!("二维进程不能进入点云内核"),
            Self::Voxel => MappingKernel::Voxel(VoxelKernel {
                current_window: VecDeque::new(),
                window_cells: Default::default(),
                capacity: window_capacity,
                last_candidate_point_count: 0,
                last_candidate_received_at_ns: 0,
                dirty_since_candidate: false,
                dynamic_scene: Default::default(),
            }),
            Self::Raytracing(spec) => MappingKernel::Raytracing(RaytracingKernel {
                config: spec.config(),
                map: VoxelMap::default(),
                prepared_update: None,
                input: Vec::new(),
                last_occupancy: None,
                last_candidate_point_count: 0,
                last_candidate_received_at_ns: 0,
                dirty_since_candidate: false,
                dynamic_scene: Default::default(),
            }),
        }
    }

    #[cfg(test)]
    pub(crate) fn raytracing_defaults() -> Self {
        Self::Raytracing(RaytracingBackendSpec::default())
    }
}

#[allow(clippy::large_enum_variant)]
#[derive(Clone)]
pub(crate) enum MappingKernel {
    Voxel(VoxelKernel),
    Raytracing(RaytracingKernel),
}

#[cfg(test)]
impl MappingKernel {
    pub(crate) const fn stable_name(&self) -> &'static str {
        match self {
            Self::Voxel(_) => "voxel",
            Self::Raytracing(_) => "raytracing",
        }
    }

    pub(crate) fn ready_to_build(&self) -> bool {
        match self {
            Self::Voxel(_) => true,
            Self::Raytracing(kernel) => kernel.map.healthy_count() != 0,
        }
    }

    pub(crate) fn prepare_map_frame(
        &mut self,
        sensor_origin: [f64; 3],
        points: &[Point3<f64>],
    ) -> Result<(), String> {
        let Self::Raytracing(kernel) = self else {
            return Ok(());
        };
        kernel.prepare_map_frame(sensor_origin, points)
    }

    pub(crate) fn commit_prepared_map_frame(&mut self) -> Result<(), String> {
        let Self::Raytracing(kernel) = self else {
            return Ok(());
        };
        kernel.commit_prepared_map_frame()
    }

    // 旧诊断测试显式读取时才物化；生产构图只读取健康体素计数。
    pub(crate) fn accumulated_points(&self, voxel_points: &[Point3<f64>]) -> Vec<Point3<f64>> {
        match self {
            Self::Voxel(_) => voxel_points.to_vec(),
            Self::Raytracing(kernel) => {
                let mut points = iter_global_points(&kernel.map, kernel.config.voxel_size)
                    .map(|(x, y, z)| Point3::new(f64::from(x), f64::from(y), f64::from(z)))
                    .collect::<Vec<_>>();
                points.sort_unstable_by(|left, right| {
                    left.x.total_cmp(&right.x)
                        .then_with(|| left.y.total_cmp(&right.y))
                        .then_with(|| left.z.total_cmp(&right.z))
                });
                points
            }
        }
    }

    pub(crate) fn reset(&mut self) {
        if let Self::Raytracing(kernel) = self {
            kernel.map = VoxelMap::default();
            kernel.prepared_update = None;
            kernel.input.clear();
        }
    }
}

#[derive(Clone)]
pub(crate) struct VoxelKernel {
    current_window: VecDeque<Point3<f64>>,
    // 窗口内各动态清除格的点数，与 current_window 同步维护；落不进格的点不会被清除，不计入。
    window_cells: ahash::AHashMap<crate::dynamic_scene::Cell, u32>,
    capacity: usize,
    last_candidate_point_count: usize,
    last_candidate_received_at_ns: u64,
    dirty_since_candidate: bool,
    dynamic_scene: crate::dynamic_scene::DynamicScene,
}

impl VoxelKernel {
    fn append_frame(&mut self, points: &[Point3<f64>]) {
        let retention_capacity = self.capacity.min(MAX_RETAINED_VOXEL_POINTS);
        let accepted = points.len().min(retention_capacity);
        let retained_start = points.len().saturating_sub(accepted);
        let existing_budget = retention_capacity.saturating_sub(accepted);
        let evicted = self.current_window.len().saturating_sub(existing_budget);
        for point in self.current_window.drain(..evicted) {
            uncount_window_cell(&mut self.window_cells, &point);
        }
        for point in &points[retained_start..] {
            count_window_cell(&mut self.window_cells, point);
        }
        self.current_window
            .extend(points[retained_start..].iter().copied());
    }
    /// 逐点撤销本帧判定为空的历史点。使用本进程动态场景时，`clears` 只对落在清除格内的点
    /// 返回真，清除格里没有窗口点就不会删除任何点，直接跳过遍历，结果与逐点筛除相同。
    /// 预计算帧决策先把点变换到里程计坐标系再查清除格，按地图坐标维护的窗口格计数
    /// 不能用来预判，照原样逐点筛除。
    fn retain_uncleared(&mut self, frame: Option<&crate::dynamic_scene::MappedFrameDecision>) {
        if frame.is_none()
            && (!self.dynamic_scene.has_clears()
                || !self.dynamic_scene.clear_cells().any(|cell| self.window_cells.contains_key(cell)))
        {
            return;
        }
        let (scene, cells) = (&self.dynamic_scene, &mut self.window_cells);
        self.current_window.retain(|point| {
            let p = [point.x, point.y, point.z];
            let cleared = frame.map_or_else(|| scene.clears(p), |frame| frame.clears(p));
            if cleared {
                uncount_window_cell(cells, point);
            }
            !cleared
        });
    }
}
fn count_window_cell(cells: &mut ahash::AHashMap<crate::dynamic_scene::Cell, u32>, point: &Point3<f64>) {
    if let Some(cell) = crate::dynamic_scene::cell([point.x, point.y, point.z]) {
        *cells.entry(cell).or_default() += 1;
    }
}
fn uncount_window_cell(cells: &mut ahash::AHashMap<crate::dynamic_scene::Cell, u32>, point: &Point3<f64>) {
    if let Some(cell) = crate::dynamic_scene::cell([point.x, point.y, point.z]) {
        if let std::collections::hash_map::Entry::Occupied(mut entry) = cells.entry(cell) {
            *entry.get_mut() -= 1;
            if *entry.get() == 0 {
                entry.remove();
            }
        }
    }
}
fn window_cells(window: &VecDeque<Point3<f64>>) -> ahash::AHashMap<crate::dynamic_scene::Cell, u32> {
    let mut cells = ahash::AHashMap::new();
    for point in window {
        count_window_cell(&mut cells, point);
    }
    cells
}

#[derive(Clone)]
pub(crate) struct RaytracingKernel {
    config: Config,
    map: VoxelMap,
    prepared_update: Option<PreparedRayUpdate>,
    input: Vec<(f32, f32, f32)>,
    last_occupancy: Option<OccupancyGrid>,
    last_candidate_point_count: usize,
    last_candidate_received_at_ns: u64,
    dirty_since_candidate: bool,
    dynamic_scene: crate::dynamic_scene::DynamicScene,
}

impl RaytracingKernel {
    fn prepare_map_frame(
        &mut self,
        sensor_origin: [f64; 3],
        points: &[Point3<f64>],
    ) -> Result<(), String> {
        self.prepared_update = None;
        let origin = (
            sensor_origin[0] as f32,
            sensor_origin[1] as f32,
            sensor_origin[2] as f32,
        );
        self.input.clear();
        self.input.reserve(points.len());
        self.input.extend(
            points
                .iter()
                .map(|point| (point.x as f32, point.y as f32, point.z as f32)),
        );
        let input = std::mem::take(&mut self.input);
        self.prepared_update = Some(
            PreparedRayUpdate::new(origin, input, self.config.clone())
                .map_err(|error| format!("RAYTRACING_UPDATE_REJECTED: {error}"))?,
        );
        Ok(())
    }

    #[cfg(test)]
    fn commit_prepared_map_frame(&mut self) -> Result<(), String> {
        self.commit_prepared_map_frame_with_parallelism(true)
    }

    fn commit_prepared_map_frame_with_parallelism(&mut self, parallel: bool) -> Result<(), String> {
        let prepared = self
            .prepared_update
            .take()
            .ok_or_else(|| "RAYTRACING_UPDATE_REJECTED: no prepared frame".to_owned())?;
        self.input = if parallel {
            prepared.apply(&mut self.map)
        } else {
            prepared.apply_serial(&mut self.map)
        };
        Ok(())
    }

    fn validate_checkpoint_budget(&self) -> Result<(), String> {
        // 已拥有的栅格由构图或检查点解码验证；预算只需精确长度，不再逐格检查旧内容。
        let occupancy_len = self
            .last_occupancy
            .as_ref()
            .map(|grid| {
                MAPPING_BACKEND_OCCUPANCY_FIXED_BYTES + grid.frame_id.len() + grid.data.len()
            })
            .unwrap_or(0);
        let encoded_len = MAPPING_BACKEND_RAY_HEADER_BYTES
            .checked_add(voxel_map_checkpoint_encoded_len(&self.map)?)
            .and_then(|length| length.checked_add(occupancy_len))
            .ok_or_else(|| "RAYTRACING_UPDATE_REJECTED: checkpoint length overflow".to_owned())?;
        if encoded_len > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
            return Err(format!(
                "RAYTRACING_UPDATE_REJECTED: retained ray state requires {encoded_len} checkpoint bytes, limit is {MAX_MAPPING_BACKEND_CHECKPOINT_BYTES}"
            ));
        }
        Ok(())
    }
}

fn mapping_backend_voxel_checkpoint_encoded_len(voxel: &VoxelKernel) -> Result<usize, String> {
    if voxel.current_window.len() > MAX_RETAINED_VOXEL_POINTS {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window contains {} points, limit is {MAX_RETAINED_VOXEL_POINTS}",
            voxel.current_window.len()
        ));
    }
    MAPPING_BACKEND_VOXEL_POINT_BYTES
        .checked_mul(voxel.current_window.len())
        .and_then(|points| MAPPING_BACKEND_VOXEL_HEADER_BYTES.checked_add(points))
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel checkpoint length overflow".to_owned()
        })
}

fn mapping_backend_occupancy_checkpoint_encoded_len(
    occupancy: &OccupancyGrid,
) -> Result<usize, String> {
    validate_kernel_checkpoint_occupancy(occupancy)?;
    u32::try_from(occupancy.frame_id.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id is too long".to_owned()
    })?;
    u64::try_from(occupancy.data.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy data length overflow".to_owned()
    })?;
    MAPPING_BACKEND_OCCUPANCY_FIXED_BYTES
        .checked_add(occupancy.frame_id.len())
        .and_then(|length| length.checked_add(occupancy.data.len()))
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy checkpoint length overflow".to_owned()
        })
}

fn mapping_backend_ray_checkpoint_encoded_len(ray: &RaytracingKernel) -> Result<usize, String> {
    let voxel_map_len = voxel_map_checkpoint_encoded_len(&ray.map).map_err(|error| {
        format!("MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length failed: {error}")
    })?;
    let occupancy_len = ray
        .last_occupancy
        .as_ref()
        .map(mapping_backend_occupancy_checkpoint_encoded_len)
        .transpose()?
        .unwrap_or(0);
    MAPPING_BACKEND_RAY_HEADER_BYTES
        .checked_add(voxel_map_len)
        .and_then(|length| length.checked_add(occupancy_len))
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: ray checkpoint length overflow".to_owned()
        })
}

fn allocate_mapping_checkpoint(encoded_len: usize) -> Result<Vec<u8>, String> {
    if encoded_len == 0 || encoded_len > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: payload size {encoded_len} is outside 1..={MAX_MAPPING_BACKEND_CHECKPOINT_BYTES} bytes"
        ));
    }
    let mut payload = Vec::new();
    payload.try_reserve_exact(encoded_len).map_err(|error| {
        format!("MAPPING_BACKEND_CHECKPOINT_INVALID: allocation failed: {error}")
    })?;
    Ok(payload)
}

fn encode_mapping_checkpoint_header(payload: &mut Vec<u8>, backend_kind: u8) {
    payload.extend_from_slice(&MAPPING_BACKEND_CHECKPOINT_MAGIC);
    payload.extend_from_slice(&MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION.to_le_bytes());
    payload.push(backend_kind);
    payload.push(0);
    payload.extend_from_slice(&0_u16.to_le_bytes());
}

fn encode_ray_config(payload: &mut Vec<u8>, config: &Config) {
    payload.extend_from_slice(&config.voxel_size.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.max_range.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.ray_subsample.to_le_bytes());
    payload.extend_from_slice(&config.shadow_depth.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.grace_depth.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.min_health.to_le_bytes());
    payload.extend_from_slice(&config.max_health.to_le_bytes());
    payload.extend_from_slice(&config.graze_cos.to_bits().to_le_bytes());
    payload.extend_from_slice(&config.recency_window.to_le_bytes());
    payload.extend_from_slice(&config.emit_every.to_le_bytes());
    payload.extend_from_slice(&config.global_emit_every.to_le_bytes());
    payload.extend_from_slice(&config.region_percentile.to_bits().to_le_bytes());
}

fn encode_checkpoint_occupancy(
    payload: &mut Vec<u8>,
    occupancy: &OccupancyGrid,
) -> Result<(), String> {
    let frame_id_len = u32::try_from(occupancy.frame_id.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id is too long".to_owned()
    })?;
    let data_len = u64::try_from(occupancy.data.len()).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy data length overflow".to_owned()
    })?;
    payload.extend_from_slice(&occupancy.width.to_le_bytes());
    payload.extend_from_slice(&occupancy.height.to_le_bytes());
    payload.extend_from_slice(&occupancy.resolution.to_bits().to_le_bytes());
    for value in occupancy.origin {
        payload.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    for value in occupancy.origin_orientation {
        payload.extend_from_slice(&value.to_bits().to_le_bytes());
    }
    payload.extend_from_slice(&occupancy.ts.to_bits().to_le_bytes());
    payload.extend_from_slice(&frame_id_len.to_le_bytes());
    payload.extend_from_slice(&data_len.to_le_bytes());
    payload.extend_from_slice(occupancy.frame_id.as_bytes());
    payload.extend(occupancy.data.iter().map(|value| *value as u8));
    Ok(())
}

pub(crate) fn export_mapping_kernel_checkpoint(kernel: &MappingKernel) -> Result<Vec<u8>, String> {
    let dirty = match kernel {
        MappingKernel::Voxel(voxel) => voxel.dirty_since_candidate,
        MappingKernel::Raytracing(ray) => ray.dirty_since_candidate,
    };
    if dirty {
        return Err(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: backend state must be flushed before export"
                .to_owned(),
        );
    }
    let encoded_len = match kernel {
        MappingKernel::Voxel(voxel) => mapping_backend_voxel_checkpoint_encoded_len(voxel)?,
        MappingKernel::Raytracing(ray) => mapping_backend_ray_checkpoint_encoded_len(ray)?,
    };
    let mut payload = match kernel {
        MappingKernel::Voxel(voxel) => {
            let mut payload = allocate_mapping_checkpoint(encoded_len)?;
            encode_mapping_checkpoint_header(&mut payload, MAPPING_BACKEND_KIND_VOXEL);
            payload.extend_from_slice(
                &u64::try_from(voxel.capacity)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel capacity overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(
                &u64::try_from(voxel.current_window.len())
                    .map_err(|_| {
                        "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window length overflow"
                    })?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(
                &u64::try_from(voxel.last_candidate_point_count)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel point count overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(&voxel.last_candidate_received_at_ns.to_le_bytes());
            for point in &voxel.current_window {
                payload.extend_from_slice(&point.x.to_bits().to_le_bytes());
                payload.extend_from_slice(&point.y.to_bits().to_le_bytes());
                payload.extend_from_slice(&point.z.to_bits().to_le_bytes());
            }
            payload
        }
        MappingKernel::Raytracing(ray) => {
            let mut payload = allocate_mapping_checkpoint(encoded_len)?;
            let voxel_map_len = voxel_map_checkpoint_encoded_len(&ray.map).map_err(|error| {
                format!("MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length failed: {error}")
            })?;
            let occupancy_len = ray
                .last_occupancy
                .as_ref()
                .map(mapping_backend_occupancy_checkpoint_encoded_len)
                .transpose()?
                .unwrap_or(0);
            encode_mapping_checkpoint_header(&mut payload, MAPPING_BACKEND_KIND_RAYTRACING);
            encode_ray_config(&mut payload, &ray.config);
            payload.extend_from_slice(
                &u64::try_from(ray.last_candidate_point_count)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: ray point count overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(&ray.last_candidate_received_at_ns.to_le_bytes());
            payload.extend_from_slice(
                &u64::try_from(voxel_map_len)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length overflow")?
                    .to_le_bytes(),
            );
            payload.extend_from_slice(
                &u64::try_from(occupancy_len)
                    .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy length overflow")?
                    .to_le_bytes(),
            );
            append_voxel_map_checkpoint(&ray.map, &mut payload).map_err(|error| {
                format!("MAPPING_BACKEND_CHECKPOINT_INVALID: ray map encode failed: {error}")
            })?;
            if let Some(occupancy) = ray.last_occupancy.as_ref() {
                encode_checkpoint_occupancy(&mut payload, occupancy)?;
            }
            payload
        }
    };
    if payload.len() != encoded_len {
        let actual_len = payload.len();
        payload.clear();
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: encoded length mismatch: expected={encoded_len} actual={actual_len}"
        ));
    }
    Ok(payload)
}

struct MappingCheckpointReader<'a> {
    payload: &'a [u8],
    offset: usize,
}

impl<'a> MappingCheckpointReader<'a> {
    fn new(payload: &'a [u8]) -> Self {
        Self { payload, offset: 0 }
    }

    fn remaining(&self) -> usize {
        self.payload.len() - self.offset
    }

    fn read_bytes(&mut self, length: usize) -> Result<&'a [u8], String> {
        let end = self.offset.checked_add(length).ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: decode offset overflow".to_owned()
        })?;
        let bytes = self.payload.get(self.offset..end).ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: checkpoint is truncated".to_owned()
        })?;
        self.offset = end;
        Ok(bytes)
    }

    fn read_array<const N: usize>(&mut self) -> Result<[u8; N], String> {
        self.read_bytes(N)?
            .try_into()
            .map_err(|_| "MAPPING_BACKEND_CHECKPOINT_INVALID: checkpoint is truncated".to_owned())
    }

    fn read_u8(&mut self) -> Result<u8, String> {
        Ok(self.read_array::<1>()?[0])
    }

    fn read_u16(&mut self) -> Result<u16, String> {
        Ok(u16::from_le_bytes(self.read_array()?))
    }

    fn read_u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.read_array()?))
    }

    fn read_i32(&mut self) -> Result<i32, String> {
        Ok(i32::from_le_bytes(self.read_array()?))
    }

    fn read_u64(&mut self) -> Result<u64, String> {
        Ok(u64::from_le_bytes(self.read_array()?))
    }

    fn read_f32(&mut self) -> Result<f32, String> {
        Ok(f32::from_bits(self.read_u32()?))
    }

    fn read_f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_bits(self.read_u64()?))
    }

    fn finish(self) -> Result<(), String> {
        if self.remaining() == 0 {
            Ok(())
        } else {
            Err(format!(
                "MAPPING_BACKEND_CHECKPOINT_INVALID: {} trailing bytes",
                self.remaining()
            ))
        }
    }
}

fn decode_mapping_checkpoint_header(
    reader: &mut MappingCheckpointReader<'_>,
    spec: MappingBackendSpec,
) -> Result<(), String> {
    if reader.read_array::<8>()? != MAPPING_BACKEND_CHECKPOINT_MAGIC {
        return Err("MAPPING_BACKEND_CHECKPOINT_INVALID: invalid checkpoint magic".to_owned());
    }
    let schema_version = reader.read_u32()?;
    if schema_version != MAPPING_BACKEND_CHECKPOINT_SCHEMA_VERSION {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: unsupported schema version {schema_version}"
        ));
    }
    let backend_kind = reader.read_u8()?;
    let expected_kind = match spec {
        MappingBackendSpec::SlamToolbox => 2,
        MappingBackendSpec::Voxel => MAPPING_BACKEND_KIND_VOXEL,
        MappingBackendSpec::Raytracing(_) => MAPPING_BACKEND_KIND_RAYTRACING,
    };
    if backend_kind != expected_kind {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: backend kind mismatch: checkpoint={backend_kind} selected={}",
            spec.stable_name()
        ));
    }
    let flags = reader.read_u8()?;
    let reserved = reader.read_u16()?;
    if flags != 0 || reserved != 0 {
        return Err(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: checkpoint flags/reserved bytes must be zero"
                .to_owned(),
        );
    }
    Ok(())
}

fn decode_ray_config(
    reader: &mut MappingCheckpointReader<'_>,
) -> Result<RayConfigCheckpoint, String> {
    Ok(RayConfigCheckpoint {
        voxel_size: reader.read_f32()?,
        max_range: reader.read_f32()?,
        ray_subsample: reader.read_u32()?,
        shadow_depth: reader.read_f32()?,
        grace_depth: reader.read_f32()?,
        min_health: reader.read_i32()?,
        max_health: reader.read_i32()?,
        graze_cos: reader.read_f32()?,
        recency_window: reader.read_u32()?,
        emit_every: reader.read_u32()?,
        global_emit_every: reader.read_u32()?,
        region_percentile: reader.read_f32()?,
    })
}

fn decode_checkpoint_occupancy(payload: &[u8]) -> Result<OccupancyGrid, String> {
    let mut reader = MappingCheckpointReader::new(payload);
    let width = reader.read_u32()?;
    let height = reader.read_u32()?;
    let resolution = reader.read_f64()?;
    let mut origin = [0.0; 3];
    for value in &mut origin {
        *value = reader.read_f64()?;
    }
    let mut origin_orientation = [0.0; 4];
    for value in &mut origin_orientation {
        *value = reader.read_f64()?;
    }
    let ts = reader.read_f64()?;
    let frame_id_len = usize::try_from(reader.read_u32()?).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id length exceeds usize".to_owned()
    })?;
    let data_len = usize::try_from(reader.read_u64()?).map_err(|_| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy data length exceeds usize".to_owned()
    })?;
    let expected_remaining = frame_id_len.checked_add(data_len).ok_or_else(|| {
        "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy variable length overflow".to_owned()
    })?;
    if reader.remaining() != expected_remaining {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy payload length mismatch: declared={expected_remaining} actual={}",
            reader.remaining()
        ));
    }
    let frame_id =
        String::from_utf8(reader.read_bytes(frame_id_len)?.to_vec()).map_err(|error| {
            format!("MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy frame id is not UTF-8: {error}")
        })?;
    let data = reader
        .read_bytes(data_len)?
        .iter()
        .map(|value| *value as i8)
        .collect();
    reader.finish()?;
    let occupancy = OccupancyGrid {
        data,
        width,
        height,
        resolution,
        origin,
        origin_orientation,
        frame_id,
        ts,
    };
    validate_kernel_checkpoint_occupancy(&occupancy)?;
    Ok(occupancy)
}

fn restore_mapping_kernel_checkpoint(
    spec: MappingBackendSpec,
    compute_config: MappingBackendComputeConfig,
    payload: &[u8],
) -> Result<MappingKernel, String> {
    if payload.is_empty() || payload.len() > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES {
        return Err(format!(
            "MAPPING_BACKEND_CHECKPOINT_INVALID: payload size {} is outside 1..={} bytes",
            payload.len(),
            MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
        ));
    }
    let mut reader = MappingCheckpointReader::new(payload);
    decode_mapping_checkpoint_header(&mut reader, spec)?;

    match spec {
        MappingBackendSpec::SlamToolbox => Err("二维图检查点须由二维工作进程恢复".into()),
        MappingBackendSpec::Voxel => {
            let capacity = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel capacity exceeds usize".to_owned()
            })?;
            let point_count = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel point count exceeds usize".to_owned()
            })?;
            let last_candidate_point_count = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel candidate count exceeds usize".to_owned()
            })?;
            let last_candidate_received_at_ns = reader.read_u64()?;
            if capacity != compute_config.window_capacity {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel capacity mismatch: checkpoint={capacity} selected={}",
                    compute_config.window_capacity
                ));
            }
            let retained_capacity = capacity.min(MAX_RETAINED_VOXEL_POINTS);
            if point_count > retained_capacity {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window contains {point_count} points for retained capacity {retained_capacity}"
                ));
            }
            if last_candidate_point_count > retained_capacity {
                return Err(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel candidate count exceeds retained capacity"
                        .to_owned(),
                );
            }
            let encoded_points = point_count
                .checked_mul(MAPPING_BACKEND_VOXEL_POINT_BYTES)
                .ok_or_else(|| {
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel point bytes overflow".to_owned()
                })?;
            if reader.remaining() != encoded_points {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel payload length mismatch: declared={encoded_points} actual={}",
                    reader.remaining()
                ));
            }
            let mut current_window = VecDeque::new();
            current_window
                .try_reserve_exact(point_count)
                .map_err(|error| {
                    format!("MAPPING_BACKEND_CHECKPOINT_INVALID: voxel allocation failed: {error}")
                })?;
            for _ in 0..point_count {
                let point = Point3::new(reader.read_f64()?, reader.read_f64()?, reader.read_f64()?);
                if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                    return Err(
                        "MAPPING_BACKEND_CHECKPOINT_INVALID: voxel window contains non-finite coordinates"
                            .to_owned(),
                    );
                }
                current_window.push_back(point);
            }
            reader.finish()?;
            Ok(MappingKernel::Voxel(VoxelKernel {
                window_cells: window_cells(&current_window),
                current_window,
                capacity,
                last_candidate_point_count,
                last_candidate_received_at_ns,
                dirty_since_candidate: false,
                dynamic_scene: Default::default(),
            }))
        }
        MappingBackendSpec::Raytracing(ray_spec) => {
            let checkpoint_config = decode_ray_config(&mut reader)?;
            let last_candidate_point_count = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: ray candidate count exceeds usize".to_owned()
            })?;
            let last_candidate_received_at_ns = reader.read_u64()?;
            let voxel_map_len = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: ray map length exceeds usize".to_owned()
            })?;
            let occupancy_len = usize::try_from(reader.read_u64()?).map_err(|_| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy length exceeds usize".to_owned()
            })?;
            let encoded_body_len = voxel_map_len.checked_add(occupancy_len).ok_or_else(|| {
                "MAPPING_BACKEND_CHECKPOINT_INVALID: ray body length overflow".to_owned()
            })?;
            if reader.remaining() != encoded_body_len {
                return Err(format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: ray payload length mismatch: declared={encoded_body_len} actual={}",
                    reader.remaining()
                ));
            }
            let config = ray_spec.config();
            if !ray_config_checkpoint_matches(&config, checkpoint_config) {
                return Err(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: ray configuration mismatch".to_owned(),
                );
            }
            let max_voxels = (MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
                - MAPPING_BACKEND_RAY_HEADER_BYTES
                - VOXEL_MAP_CHECKPOINT_HEADER_BYTES)
                / VOXEL_MAP_CHECKPOINT_ENTRY_BYTES;
            let map = decode_voxel_map_checkpoint_for_config(
                reader.read_bytes(voxel_map_len)?,
                max_voxels,
                &config,
            )?;
            let last_occupancy = if occupancy_len == 0 {
                None
            } else {
                Some(decode_checkpoint_occupancy(
                    reader.read_bytes(occupancy_len)?,
                )?)
            };
            reader.finish()?;
            if last_candidate_point_count > max_voxels {
                return Err(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: ray candidate count exceeds state limit"
                        .to_owned(),
                );
            }
            let ray = RaytracingKernel {
                config,
                map,
                prepared_update: None,
                input: Vec::new(),
                last_occupancy,
                last_candidate_point_count,
                last_candidate_received_at_ns,
                dirty_since_candidate: false,
                dynamic_scene: Default::default(),
            };
            ray.validate_checkpoint_budget().map_err(|error| {
                format!(
                    "MAPPING_BACKEND_CHECKPOINT_INVALID: restored ray state is oversized: {error}"
                )
            })?;
            Ok(MappingKernel::Raytracing(ray))
        }
    }
}

fn validate_kernel_checkpoint_occupancy(occupancy: &OccupancyGrid) -> Result<(), String> {
    let expected = usize::try_from(occupancy.width)
        .ok()
        .and_then(|width| {
            usize::try_from(occupancy.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| {
            "MAPPING_BACKEND_CHECKPOINT_INVALID: occupancy dimensions overflow".to_owned()
        })?;
    if occupancy.width == 0
        || occupancy.height == 0
        || occupancy.data.len() != expected
        || !occupancy.resolution.is_finite()
        || occupancy.resolution <= 0.0
        || occupancy.origin.iter().any(|value| !value.is_finite())
        || occupancy
            .origin_orientation
            .iter()
            .any(|value| !value.is_finite())
        || occupancy.frame_id.trim().is_empty()
        || !occupancy.ts.is_finite()
        || occupancy
            .data
            .iter()
            .any(|value| !(-1..=100).contains(value))
    {
        return Err("MAPPING_BACKEND_CHECKPOINT_INVALID: retained occupancy is invalid".to_owned());
    }
    Ok(())
}

fn occupancy_config(config: MappingBackendComputeConfig) -> OccupancyConfig {
    OccupancyConfig {
        initial_safe_radius_meters: config.initial_safe_radius_meters,
    }
}

struct BorrowedBackendCloud<'a> {
    points: &'a [Point3<f64>],
    sensor_origin: Point3<f64>,
    frame_id: &'a str,
    timestamp: f64,
}

impl PointCloud2Like for BorrowedBackendCloud<'_> {
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

fn build_occupancy(
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
    fallback: &crate::dynamic_scene::DynamicScene,
) -> Result<OccupancyGrid, String> {
    // 水平扫描必须与本帧雷达原点配对，不能从当前原点向历史点云发射射线。
    let retained:Vec<_>=input.registered_points_map.iter().filter(|p|
        !input.dynamic_frame.as_ref().map_or_else(||fallback.is_dynamic([p.x,p.y,p.z]),|f|f.is_dynamic([p.x,p.y,p.z]))).copied().collect();
    let cloud = BorrowedBackendCloud {
        points: &retained,
        sensor_origin: nalgebra::point![
            input.sensor_origin_map[0],
            input.sensor_origin_map[1],
            input.sensor_origin_map[2]
        ],
        frame_id: &input.registered_frame_id,
        timestamp: input.source_timestamp_ns as f64 / 1_000_000_000.0,
    };
    CostMapper::occupancy_from_pointcloud_auto(&cloud, &occupancy_config(config), config.resolution)
        .map_err(|error| format!("MAPPING_BACKEND_COMPUTE_REJECTED: {error}"))
}

fn candidate_due(last_received_at_ns: u64, received_at_ns: u64, interval_ns: u64) -> bool {
    last_received_at_ns == 0 || received_at_ns.saturating_sub(last_received_at_ns) >= interval_ns
}

pub(crate) fn flush_candidate(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    token: &MappingComputeToken,
    config: MappingBackendComputeConfig,
) -> Result<Option<MappingBackendCandidate>, String> {
    if token.session_id != input.session_id
        || token.session_generation != input.session_generation
        || token.backend_generation != input.backend_generation
        || token.source_frame_sequence != input.frame_sequence
        || token.source_timestamp_ns != input.source_timestamp_ns
    {
        return Err("MAPPING_BACKEND_FLUSH_REJECTED: token does not match latest input".to_owned());
    }
    let started = Instant::now();
    let (backend_kind, live_window_occupancy, latest_frame_occupancy) = match kernel {
        MappingKernel::Voxel(voxel) => {
            if !voxel.dirty_since_candidate {
                return Ok(None);
            }
            if voxel.current_window.is_empty() {
                voxel.dirty_since_candidate = false;
                return Ok(None);
            }
            let latest = build_occupancy(input, config, &voxel.dynamic_scene)?;
            voxel.last_candidate_received_at_ns = input.received_at_ns;
            voxel.last_candidate_point_count = voxel.current_window.len();
            voxel.dirty_since_candidate = false;
            (MappingBackendKind::Voxel, latest.clone(), latest)
        }
        MappingKernel::Raytracing(ray) => {
            if !ray.dirty_since_candidate {
                return Ok(None);
            }
            let live = build_occupancy(input, config, &ray.dynamic_scene)?;
            ray.last_occupancy = Some(live.clone());
            ray.last_candidate_received_at_ns = input.received_at_ns;
            ray.last_candidate_point_count = ray.map.healthy_count();
            ray.dirty_since_candidate = false;
            (MappingBackendKind::Raytracing, live.clone(), live)
        }
    };
    Ok(Some(MappingBackendCandidate {
        session_id: input.session_id.clone(),
        session_generation: input.session_generation,
        backend_generation: input.backend_generation,
        source_frame_sequence: input.frame_sequence,
        source_timestamp_ns: input.source_timestamp_ns,
        received_at_ns: input.received_at_ns,
        backend_kind,
        live_window_occupancy,
        latest_frame_occupancy,
        accepted_points_map: Vec::new(),
        compute_elapsed_us: u64::try_from(started.elapsed().as_micros()).unwrap_or(u64::MAX),
    }))
}

#[cfg(test)]
fn compute_candidate(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
) -> Result<Option<MappingBackendCandidate>, String> {
    validate_backend_input(kernel, input)?;
    let mut staged = kernel.clone();
    let result = compute_candidate_in_place(&mut staged, input, config)?;
    *kernel = staged;
    Ok(result)
}

pub(crate) fn validate_backend_input(
    _kernel: &MappingKernel,
    input: &MappingBackendInput,
) -> Result<(), String> {
    if input.registered_points_map.is_empty() {
        return Ok(());
    }
    if input.session_id.trim().is_empty()
        || input.session_generation == 0
        || input.backend_generation == 0
        || input.frame_sequence == 0
        || input.registered_frame_id.trim().is_empty()
    {
        return Err("MAPPING_BACKEND_COMPUTE_REJECTED: invalid input identity".to_owned());
    }
    Ok(())
}

pub(crate) fn compute_candidate_in_place(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
) -> Result<Option<MappingBackendCandidate>, String> {
    compute_candidate_with_parallelism(kernel, input, config, MAPPING_CANDIDATE_INTERVAL_NS, true)
}

/// 历史回放自行选择构图节拍；点云接纳与累积仍逐帧执行。
pub(crate) fn compute_candidate_in_place_with_interval(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
    candidate_interval_ns: u64,
) -> Result<Option<MappingBackendCandidate>, String> {
    compute_candidate_with_parallelism(kernel, input, config, candidate_interval_ns, false)
}

fn compute_candidate_with_parallelism(
    kernel: &mut MappingKernel,
    input: &MappingBackendInput,
    config: MappingBackendComputeConfig,
    candidate_interval_ns: u64,
    parallel: bool,
) -> Result<Option<MappingBackendCandidate>, String> {
    if input.registered_points_map.is_empty() {
        return Ok(None);
    }

    let started = Instant::now();
    #[cfg(test)]
    let diagnostic = std::env::var_os("NEH_PGO_PHASE_DIAGNOSTIC").is_some();
    let backend_kind = match kernel {
        MappingKernel::Voxel(voxel) => {
            if input.dynamic_frame.is_none(){voxel.dynamic_scene.observe(&input.registered_points_map,input.sensor_origin_map,input.source_timestamp_ns);}
            voxel.retain_uncleared(input.dynamic_frame.as_ref());
            let retained=input.registered_points_map.iter().filter(|p|!input.dynamic_frame.as_ref().map_or_else(||voxel.dynamic_scene.is_dynamic([p.x,p.y,p.z]),|f|f.is_dynamic([p.x,p.y,p.z]))).copied().collect::<Vec<_>>();
            voxel.append_frame(&retained);
            voxel.dirty_since_candidate = true;
            MappingBackendKind::Voxel
        }
        MappingKernel::Raytracing(ray) => {
            #[cfg(test)]
            let phase = Instant::now();
            if input.dynamic_frame.is_none(){ray.dynamic_scene.observe(&input.registered_points_map,input.sensor_origin_map,input.source_timestamp_ns);}
            let retained=input.registered_points_map.iter().filter(|p|!input.dynamic_frame.as_ref().map_or_else(||ray.dynamic_scene.is_dynamic([p.x,p.y,p.z]),|f|f.is_dynamic([p.x,p.y,p.z]))).copied().collect::<Vec<_>>();
            // 本地 9e719718 已删除 validate_retained_state_bound（重复的保留体素上限校验），这里不再调用。
            ray.prepare_map_frame(input.sensor_origin_map, &retained)?;
            #[cfg(test)]
            let prepare_us = phase.elapsed().as_micros();
            #[cfg(test)]
            let phase = Instant::now();
            ray.commit_prepared_map_frame_with_parallelism(parallel)?;
            #[cfg(test)]
            let update_us = phase.elapsed().as_micros();
            #[cfg(test)]
            let phase = Instant::now();
            if input.dynamic_frame.is_some()||ray.dynamic_scene.has_clears() {
                // 当前回波仍作为导航障碍，清理只撤销本帧未占据的历史体素。
                let size = f64::from(ray.config.voxel_size);
                // 命中键必须与射线库的单精度乘逆量化完全相同，边界点不可漏保护。
                let inv = 1.0_f32 / ray.config.voxel_size;
                let hits: ahash::AHashSet<_> = ray
                    .input
                    .iter()
                    .map(|point| {
                        (
                            (point.0 * inv).floor() as i32,
                            (point.1 * inv).floor() as i32,
                            (point.2 * inv).floor() as i32,
                        )
                    })
                    .collect();
                ray.map.voxels.retain(|key, _| {
                    hits.contains(key)
                        || !{let p=[(key.0 as f64+0.5)*size,(key.1 as f64+0.5)*size,(key.2 as f64+0.5)*size];
                            input.dynamic_frame.as_ref().map_or_else(||ray.dynamic_scene.clears(p),|f|f.clears(p))}
                });
            }
            #[cfg(test)]
            if diagnostic {
                eprintln!("PGO_RAY_PHASE frame={} points={} voxels={} prepare_us={prepare_us} update_us={update_us} dynamic_us={}", input.frame_sequence, input.registered_points_map.len(), ray.map.voxels.len(), phase.elapsed().as_micros());
            }
            ray.dirty_since_candidate = true;
            // 每帧更新体素；构图只读本帧扫描，历史体素无需物化成点云。
            if !candidate_due(
                ray.last_candidate_received_at_ns,
                input.received_at_ns,
                candidate_interval_ns,
            ) {
                return Ok(None);
            }
            MappingBackendKind::Raytracing
        }
    };

    let (live_window_occupancy, latest_frame_occupancy) = match kernel {
        MappingKernel::Voxel(voxel) => {
            let point_count = voxel.current_window.len();
            if !candidate_due(
                voxel.last_candidate_received_at_ns,
                input.received_at_ns,
                candidate_interval_ns,
            ) {
                return Ok(None);
            }
            let latest = build_occupancy(input, config, &voxel.dynamic_scene)?;
            let result = (latest.clone(), latest);
            voxel.last_candidate_received_at_ns = input.received_at_ns;
            voxel.last_candidate_point_count = point_count;
            voxel.dirty_since_candidate = false;
            result
        }
        MappingKernel::Raytracing(ray) => {
            let point_count = ray.map.healthy_count();
            if !candidate_due(
                ray.last_candidate_received_at_ns,
                input.received_at_ns,
                candidate_interval_ns,
            ) {
                return Ok(None);
            }
            let live = build_occupancy(input, config, &ray.dynamic_scene)?;
            ray.last_occupancy = Some(live.clone());
            ray.last_candidate_received_at_ns = input.received_at_ns;
            ray.last_candidate_point_count = point_count;
            ray.dirty_since_candidate = false;
            (live.clone(), live)
        }
    };

    Ok(Some(MappingBackendCandidate {
        session_id: input.session_id.clone(),
        session_generation: input.session_generation,
        backend_generation: input.backend_generation,
        source_frame_sequence: input.frame_sequence,
        source_timestamp_ns: input.source_timestamp_ns,
        received_at_ns: input.received_at_ns,
        backend_kind,
        live_window_occupancy,
        latest_frame_occupancy,
        accepted_points_map: input.registered_points_map.clone(),
        compute_elapsed_us: u64::try_from(started.elapsed().as_micros()).unwrap_or(u64::MAX),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_mapping_kernels_clear_passing_person_history_and_keep_current_obstacles() {
        use crate::dynamic_scene::tests::{corridor, ORIGIN};
        for spec in [
            MappingBackendSpec::Voxel,
            MappingBackendSpec::raytracing_defaults(),
        ] {
            let config = MappingBackendComputeConfig {
                window_capacity: 1_000_000,
                ..compute_config()
            };
            let mut kernel = spec.into_kernel(config.window_capacity);
            let mut people_count = 0;
            let mut persistent: Option<crate::persistent_occupancy::PersistentOccupancyGrid> = None;
            let mut peak_obstacles = 0;
            let mut final_obstacles = 0;
            for frame in 1..=22 {
                let person = (7..=16)
                    .contains(&frame)
                    .then(|| -0.5 + (frame - 7) as f64 * 0.12);
                let (points, labels) = corridor(person, (frame % 3) as f64);
                people_count += labels.iter().filter(|label| **label).count();
                let input = MappingBackendInput {
                    session_id: "动态人体双后端".into(),
                    session_generation: 1,
                    backend_generation: 1,
                    frame_sequence: frame,
                    source_timestamp_ns: frame * 100_000_000,
                    received_at_ns: frame * 100_000_000,
                    registered_frame_id: "map".into(),
                    sensor_origin_map: ORIGIN,
                    dynamic_frame: None,
                    registered_points_map: points,
                };
                if let Some(candidate) =
                    compute_candidate_in_place(&mut kernel, &input, config).unwrap()
                {
                    assert_eq!(candidate.accepted_points_map,input.registered_points_map,"realtime retains every raw return, including the person");
                    persistent = Some(
                        crate::persistent_occupancy::PersistentOccupancyGrid::stage_observation(
                            persistent.as_ref(),
                            &candidate.latest_frame_occupancy,
                        )
                        .unwrap(),
                    );
                }
                if let Some(persistent) = &persistent {
                    let grid = &persistent.grid;
                    final_obstacles = (0..grid.height)
                        .flat_map(|row| (0..grid.width).map(move |col| (row, col)))
                        .filter(|(row, col)| {
                            let x = grid.origin[0] + (*col as f64 + 0.5) * grid.resolution;
                            let y = grid.origin[1] + (*row as f64 + 0.5) * grid.resolution;
                            (2.6..3.4).contains(&x)
                                && (-1.0..1.3).contains(&y)
                                && grid.data[grid.index(*row, *col)] >= 50
                        })
                        .count();
                    peak_obstacles = peak_obstacles.max(final_obstacles);
                }
            }
            let retained = match &kernel {
                MappingKernel::Voxel(voxel) => {
                    voxel.current_window.iter().copied().collect::<Vec<_>>()
                }
                MappingKernel::Raytracing(_) => kernel.accumulated_points(&[]),
            };
            let ghosts = retained
                .iter()
                .filter(|p| p.x > 2.7 && p.x < 3.3 && p.z > 0.1 && p.z < 1.9)
                .count();
            eprintln!("二维后端={} 人体输入={people_count} 剩余点={ghosts} 占据峰值={peak_obstacles} 末态占据={final_obstacles}", spec.kind().stable_name());
            assert!(ghosts * 5 < people_count);
            assert!(
                final_obstacles == 0,
                "动态人体不再写入持久二维图；实时原始回波另行保留"
            );
            let checkpoint = export_mapping_kernel_checkpoint(&kernel).unwrap();
            let restored = restore_mapping_kernel_checkpoint(spec, config, &checkpoint).unwrap();
            assert_eq!(
                export_mapping_kernel_checkpoint(&restored).unwrap(),
                checkpoint
            );
        }
    }

    #[test]
    fn final_join_handle_drop_is_nonblocking_and_retains_blocked_thread() {
        let gate = MappingBackendRetirementTestGate::blocked();
        let worker = MappingBackendWorker::start_with_retirement_test_gate(
            MappingBackendSpec::Voxel,
            1,
            gate.clone(),
        )
        .expect("start blocked retirement worker");
        let join_handle = worker
            .into_stop_join_handle()
            .expect("extract final backend join handle");
        for _ in 0..1_000 {
            if gate.entered() {
                break;
            }
            std::thread::yield_now();
        }
        assert!(
            gate.entered(),
            "retirement worker reached the blocking gate"
        );
        let retained_before = mapping_backend_process_exit_retention()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .len();
        let (dropped_tx, dropped_rx) = mpsc::channel();
        let dropper = thread::spawn(move || {
            drop(join_handle);
            dropped_tx
                .send(())
                .expect("report nonblocking join-handle destruction");
        });
        dropped_rx
            .recv_timeout(Duration::from_millis(100))
            .expect("Drop must not join the blocked backend thread");
        dropper.join().expect("join Drop observer");
        let retained_after = mapping_backend_process_exit_retention()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .len();
        assert_eq!(retained_after, retained_before + 1);
        gate.release();
    }

    #[test]
    fn stop_interrupts_pending_resolution_before_worker_join() {
        let worker = MappingBackendWorker::start(MappingBackendSpec::Voxel, 1, compute_config())
            .expect("start backend worker");
        let completion = worker
            .client()
            .submit(token(1), input(1))
            .expect("submit unresolved compute")
            .blocking_recv()
            .expect("receive unresolved completion");
        let started = Instant::now();
        worker
            .stop_and_join()
            .expect("stop flag interrupts resolution wait and joins worker");
        assert!(
            started.elapsed() < Duration::from_secs(1),
            "worker join must not wait for the 15-second resolution deadline"
        );
        drop(completion);
    }

    fn compute_config() -> MappingBackendComputeConfig {
        MappingBackendComputeConfig {
            resolution: 0.1,
            initial_safe_radius_meters: 0.0,
            window_capacity: 1024,
        }
    }

    fn token(sequence: u64) -> MappingComputeToken {
        MappingComputeToken {
            token_id: u128::from(sequence),
            session_id: "worker-test".to_owned(),
            session_generation: 1,
            backend_generation: 1,
            source_frame_sequence: sequence,
            source_timestamp_ns: sequence,
            prepared_at_ns: sequence,
            base_occupancy_revision: 0,
        }
    }

    fn input(sequence: u64) -> MappingBackendInput {
        let timestamp_ns = sequence.saturating_mul(1_000_000_000);
        MappingBackendInput {
            session_id: "worker-test".to_owned(),
            session_generation: 1,
            backend_generation: 1,
            frame_sequence: sequence,
            source_timestamp_ns: timestamp_ns,
            received_at_ns: timestamp_ns,
            registered_frame_id: "map".to_owned(),
            sensor_origin_map: [0.0; 3],
            dynamic_frame: None,
            registered_points_map: vec![Point3::new(1.0, 0.0, 1.0)],
        }
    }

    #[test]
    fn planar_scan_backends_use_current_frame_origin_and_survive_restore() {
        let config = MappingBackendComputeConfig { ..compute_config() };
        for spec in [
            MappingBackendSpec::Voxel,
            MappingBackendSpec::raytracing_defaults(),
        ] {
            let mut kernel = spec.into_kernel(config.window_capacity);
            let mut first = input(1);
            first.registered_points_map = vec![Point3::new(2.05, 0.05, 0.0)];
            compute_candidate(&mut kernel, &first, config)
                .unwrap()
                .unwrap();
            let mut next = input(2);
            next.sensor_origin_map = [10.05, 0.05, 0.0];
            next.registered_points_map = vec![Point3::new(12.05, 0.05, 0.0)];
            let candidate = compute_candidate(&mut kernel, &next, config)
                .unwrap()
                .unwrap();
            assert!(candidate.live_window_occupancy.origin[0] >= 9.0);
            assert_eq!(
                candidate.live_window_occupancy.data,
                candidate.latest_frame_occupancy.data
            );
            assert!(candidate.latest_frame_occupancy.data.contains(&100));
            let checkpoint = export_mapping_kernel_checkpoint(&kernel).unwrap();
            let mut restored =
                restore_mapping_kernel_checkpoint(spec, config, &checkpoint).unwrap();
            next.frame_sequence = 3;
            next.source_timestamp_ns += 1_000_000_000;
            next.received_at_ns += 1_000_000_000;
            let expected = compute_candidate(&mut kernel, &next, config)
                .unwrap()
                .unwrap();
            let actual = compute_candidate(&mut restored, &next, config)
                .unwrap()
                .unwrap();
            assert_eq!(
                actual.live_window_occupancy.data,
                expected.live_window_occupancy.data
            );
            next.frame_sequence = 4;
            next.source_timestamp_ns += 1_000_000_000;
            next.received_at_ns += 1_000_000_000;
            next.registered_points_map = vec![Point3::new(12.05, 0.05, -0.4)];
            let slanted = compute_candidate(&mut restored, &next, config)
                .unwrap()
                .unwrap();
            assert!(slanted.latest_frame_occupancy.data.contains(&0));
            assert!(!slanted.latest_frame_occupancy.data.contains(&100));
            // 斜射线离开高度带前不足最小观测距离时，仍没有可用二维证据。
            next.frame_sequence = 5;
            next.source_timestamp_ns += 1_000_000_000;
            next.received_at_ns += 1_000_000_000;
            next.registered_points_map = vec![Point3::new(10.15, 0.05, -0.4)];
            let missing = compute_candidate(&mut restored, &next, config)
                .unwrap()
                .unwrap();
            assert!(missing.latest_frame_occupancy.data.iter().all(|&v| v < 0));
        }
    }

    fn commit_completion(
        completion: MappingBackendCompletion,
    ) -> Result<Option<MappingBackendCandidate>, String> {
        let MappingBackendCompletion {
            result, resolution, ..
        } = completion;
        resolution.commit().expect("commit worker completion");
        result
    }

    #[test]
    fn voxel_selection_does_not_parse_raytracing_environment() {
        let spec = MappingBackendSpec::from_lookup(MappingBackendKind::Voxel, |_| {
            Some("invalid".to_owned())
        })
        .expect("inactive raytracing config must not affect voxel startup");
        assert_eq!(spec, MappingBackendSpec::Voxel);
    }

    #[test]
    fn raytracing_defaults_are_valid_and_deterministic() {
        let spec = MappingBackendSpec::from_lookup(MappingBackendKind::Raytracing, |_| None)
            .expect("defaults must pass standalone validation");
        assert_eq!(spec.stable_name(), "raytracing");
        let mut kernel = spec.into_kernel(1024);
        let points = [
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
        ];
        for _ in 0..3 {
            kernel
                .prepare_map_frame([0.0; 3], &points)
                .expect("valid map-frame points");
            kernel
                .commit_prepared_map_frame()
                .expect("prepared frame must commit");
        }
        let output = kernel.accumulated_points(&[]);
        assert!(!output.is_empty());
        assert!(output.windows(2).all(|pair| {
            (pair[0].x, pair[0].y, pair[0].z) <= (pair[1].x, pair[1].y, pair[1].z)
        }));
    }

    #[test]
    fn invalid_selected_config_fails_closed_with_key() {
        let error = MappingBackendSpec::from_lookup(MappingBackendKind::Raytracing, |key| {
            (key == RAY_SUBSAMPLE_ENV).then(|| "0".to_owned())
        })
        .expect_err("zero ray subsample must fail startup");
        assert!(error.contains("RAYTRACING_CONFIG_INVALID"));
        assert!(error.contains("ray_subsample"));
    }

    #[test]
    fn rejected_frame_does_not_mutate_ray_snapshot() {
        let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
        let valid = [Point3::new(1.0, 0.0, 0.0)];
        for _ in 0..3 {
            kernel
                .prepare_map_frame([0.0; 3], &valid)
                .expect("valid frame");
            kernel
                .commit_prepared_map_frame()
                .expect("prepared frame must commit");
        }
        let before = kernel.accumulated_points(&[]).to_vec();
        let error = kernel
            .prepare_map_frame([0.0; 3], &[Point3::new(f64::MAX, 0.0, 0.0)])
            .expect_err("out-of-range point must be rejected");
        assert!(error.contains("RAYTRACING_UPDATE_REJECTED"));
        assert_eq!(kernel.accumulated_points(&[]), before);
    }

    #[test]
    fn aggregate_frame_work_is_bounded_before_map_mutation() {
        let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
        let points = vec![Point3::new(49.0, 0.0, 0.0); 60_000];
        let error = kernel
            .prepare_map_frame([0.0; 3], &points)
            .expect_err("aggregate ray work must be bounded");
        assert!(error.contains("more than 50000000 ray steps"));
        assert!(!kernel.ready_to_build());
    }

    #[test]
    fn ray_candidate_and_flush_preserve_healthy_count_without_history_points() {
        let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
        for sequence in 1..=3 {
            compute_candidate_in_place(&mut kernel, &input(sequence), compute_config()).unwrap();
        }
        let mut skipped = input(4);
        skipped.received_at_ns = 3_000_000_000 + MAPPING_CANDIDATE_INTERVAL_NS / 2;
        skipped.source_timestamp_ns = skipped.received_at_ns;
        assert!(
            compute_candidate_in_place(&mut kernel, &skipped, compute_config())
                .unwrap()
                .is_none()
        );
        let MappingKernel::Raytracing(ray) = &kernel else {
            panic!("射线后端");
        };
        assert!(ray.dirty_since_candidate);
        let mut flush_token = token(4);
        flush_token.source_timestamp_ns = skipped.source_timestamp_ns;
        assert!(
            flush_candidate(&mut kernel, &skipped, &flush_token, compute_config())
                .unwrap()
                .is_some()
        );
        let MappingKernel::Raytracing(ray) = &kernel else {
            panic!("射线后端");
        };
        assert!(!ray.dirty_since_candidate);
        assert_eq!(ray.last_candidate_point_count, ray.map.healthy_count());
    }

    #[test]
    fn worker_queue_capacity_is_exactly_one() {
        let gate = MappingBackendTestGate::default();
        let worker = MappingBackendWorker::start_with_test_gate(
            MappingBackendSpec::Voxel,
            1,
            compute_config(),
            gate.clone(),
        )
        .expect("start gated worker");
        let client = worker.client();
        let first = client.submit(token(1), input(1)).expect("first job");
        for _ in 0..1_000 {
            if gate.entered() {
                break;
            }
            std::thread::yield_now();
        }
        assert!(gate.entered(), "worker must dequeue and enter first job");
        let second = client.submit(token(2), input(2)).expect("one queued job");
        assert!(matches!(
            client.submit(token(3), input(3)),
            Err(MappingBackendSubmitError::QueueFull)
        ));
        gate.release();
        commit_completion(first.blocking_recv().expect("first completion")).expect("first compute");
        commit_completion(second.blocking_recv().expect("second completion"))
            .expect("second compute");
        worker.stop_and_join().expect("join worker");
    }

    #[test]
    fn aborted_compute_retires_mutated_worker_generation() {
        let worker = MappingBackendWorker::start(MappingBackendSpec::Voxel, 1, compute_config())
            .expect("start worker");
        let client = worker.client();
        let completion = client
            .submit(token(1), input(1))
            .expect("submit compute")
            .blocking_recv()
            .expect("compute completion");
        completion
            .resolution
            .abort()
            .expect("abort staged generation");
        // abort 只投递决策；等待后台实际退出，不用调度次数代替完成条件。
        let deadline = Instant::now() + Duration::from_secs(2);
        while !worker.join.as_ref().expect("worker join owner").is_finished() {
            assert!(Instant::now() < deadline,
                "aborted generation must stop before accepting another compute");
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(matches!(client.submit(token(2), input(2)),
            Err(MappingBackendSubmitError::WorkerStopped)));
        worker.stop_and_join().expect("join retired worker");
    }

    #[test]
    fn worker_checkpoint_round_trip_preserves_voxel_and_ray_continuation() {
        for spec in [
            MappingBackendSpec::Voxel,
            MappingBackendSpec::raytracing_defaults(),
        ] {
            let original = MappingBackendWorker::start(spec, 1, compute_config())
                .expect("start original worker");
            let original_client = original.client();
            for sequence in 1..=3 {
                let completion = original_client
                    .submit(token(sequence), input(sequence))
                    .expect("submit seed frame")
                    .blocking_recv()
                    .expect("seed completion");
                commit_completion(completion).expect("seed compute");
            }
            let payload = original_client
                .export_checkpoint()
                .expect("submit export")
                .blocking_recv()
                .expect("export response")
                .expect("export checkpoint");

            let restored = MappingBackendWorker::start(spec, 2, compute_config())
                .expect("start restored worker");
            let restored_client = restored.client();
            let restore = restored_client
                .restore_checkpoint(payload.clone())
                .expect("submit restore")
                .blocking_recv()
                .expect("restore response")
                .expect("restore checkpoint");
            restore.commit().expect("commit restored checkpoint");
            let restored_payload = restored_client
                .export_checkpoint()
                .expect("submit restored export")
                .blocking_recv()
                .expect("restored export response")
                .expect("restored export checkpoint");
            assert_eq!(restored_payload, payload);

            let original_completion = original_client
                .submit(token(4), input(4))
                .expect("submit original continuation")
                .blocking_recv()
                .expect("original continuation response");
            commit_completion(original_completion).expect("original continuation compute");
            let restored_completion = restored_client
                .submit(token(4), input(4))
                .expect("submit restored continuation")
                .blocking_recv()
                .expect("restored continuation response");
            commit_completion(restored_completion).expect("restored continuation compute");
            let original_continued = original_client
                .export_checkpoint()
                .expect("submit original continued export")
                .blocking_recv()
                .expect("original continued export response")
                .expect("original continued checkpoint");
            let restored_continued = restored_client
                .export_checkpoint()
                .expect("submit restored continued export")
                .blocking_recv()
                .expect("restored continued export response")
                .expect("restored continued checkpoint");
            assert_eq!(restored_continued, original_continued);

            original.stop_and_join().expect("join original worker");
            restored.stop_and_join().expect("join restored worker");
        }
    }

    #[test]
    fn steady_small_maps_publish_real_candidates_before_navigation_receipt_expires() {
        for spec in [
            MappingBackendSpec::Voxel,
            MappingBackendSpec::raytracing_defaults(),
        ] {
            let mut config = compute_config();
            config.window_capacity = 1;
            let worker = MappingBackendWorker::start(spec, 1, config).expect("真实后端线程");
            let client = worker.client();
            let mut last_candidate_received = None;
            let mut candidate_count = 0;
            for sequence in 1_u64..=16 {
                let mut observation = input(sequence);
                observation.received_at_ns = 1_000_000_000 + (sequence - 1) * 100_000_000;
                observation.source_timestamp_ns = 77_000_000_000 + (sequence - 1) * 100_000_000;
                let received_at_ns = observation.received_at_ns;
                let source_timestamp_ns = observation.source_timestamp_ns;
                let mut request = token(sequence);
                request.prepared_at_ns = received_at_ns;
                request.source_timestamp_ns = source_timestamp_ns;
                let completion = client
                    .submit(request, observation)
                    .expect("持续真实点输入")
                    .blocking_recv()
                    .expect("后端回执");
                if let Some(candidate) = commit_completion(completion).expect("提交真实候选")
                {
                    assert_eq!(candidate.source_timestamp_ns, source_timestamp_ns);
                    assert_eq!(candidate.received_at_ns, received_at_ns);
                    last_candidate_received = Some(received_at_ns);
                    candidate_count += 1;
                }
                if let Some(last) = last_candidate_received {
                    assert!(
                        received_at_ns - last <= 500_000_000,
                        "低增长真实输入不应跨过导航500毫秒地图接收期限：{}",
                        spec.stable_name()
                    );
                }
            }
            assert!(
                candidate_count >= 14,
                "100毫秒真实输入在完成后端初始化后应逐次交付，不再被250毫秒门槛丢弃"
            );
            let mut empty = input(17);
            empty.registered_points_map.clear();
            empty.source_timestamp_ns = 79_000_000_000;
            empty.received_at_ns = 3_000_000_000;
            let mut request = token(17);
            request.source_timestamp_ns = empty.source_timestamp_ns;
            assert!(
                commit_completion(
                    client
                        .submit(request, empty)
                        .unwrap()
                        .blocking_recv()
                        .unwrap()
                )
                .unwrap()
                .is_none(),
                "空输入不得从缓存生成保活候选"
            );
            worker.stop_and_join().expect("停止并等待真实后端线程");
            assert!(
                client.submit(token(18), input(18)).is_err(),
                "已停止后端不能续发地图"
            );
        }
    }

    #[test]
    fn worker_flush_forces_one_cadence_candidate_before_export() {
        let worker = MappingBackendWorker::start(MappingBackendSpec::Voxel, 1, compute_config())
            .expect("start worker");
        let client = worker.client();
        let first_input = input(1);
        let mut first_token = token(1);
        first_token.source_timestamp_ns = first_input.source_timestamp_ns;
        let first = client
            .submit(first_token, first_input)
            .expect("submit first frame")
            .blocking_recv()
            .expect("first response");
        assert!(commit_completion(first).expect("first compute").is_some());

        let mut skipped_input = input(2);
        skipped_input.source_timestamp_ns = 1_000_000_000 + MAPPING_CANDIDATE_INTERVAL_NS / 2;
        skipped_input.received_at_ns = skipped_input.source_timestamp_ns;
        let mut skipped_token = token(2);
        skipped_token.source_timestamp_ns = skipped_input.source_timestamp_ns;
        let skipped = client
            .submit(skipped_token.clone(), skipped_input)
            .expect("submit cadence frame")
            .blocking_recv()
            .expect("cadence response");
        assert!(commit_completion(skipped)
            .expect("cadence compute")
            .is_none());
        assert!(client
            .export_checkpoint()
            .expect("submit dirty export")
            .blocking_recv()
            .expect("dirty export response")
            .expect_err("dirty state must require flush")
            .contains("must be flushed"));

        let flushed = client
            .flush(skipped_token.clone())
            .expect("submit flush")
            .blocking_recv()
            .expect("flush response");
        let candidate = commit_completion(flushed)
            .expect("flush compute")
            .expect("forced candidate");
        assert!(candidate.accepted_points_map.is_empty());
        let repeated = client
            .flush(skipped_token)
            .expect("submit repeated flush")
            .blocking_recv()
            .expect("repeated flush response");
        assert!(commit_completion(repeated)
            .expect("repeated flush compute")
            .is_none());
        client
            .export_checkpoint()
            .expect("submit clean export")
            .blocking_recv()
            .expect("clean export response")
            .expect("clean checkpoint");
        worker.stop_and_join().expect("join worker");
    }

    #[test]
    fn voxel_checkpoint_point_budget_is_the_exact_64_mib_boundary() {
        assert_eq!(MAX_RETAINED_VOXEL_POINTS, 2_796_200);
        let maximum = MAPPING_BACKEND_VOXEL_HEADER_BYTES
            + MAX_RETAINED_VOXEL_POINTS * MAPPING_BACKEND_VOXEL_POINT_BYTES;
        let first_rejected = maximum + MAPPING_BACKEND_VOXEL_POINT_BYTES;
        assert!(maximum <= MAX_MAPPING_BACKEND_CHECKPOINT_BYTES);
        assert!(first_rejected > MAX_MAPPING_BACKEND_CHECKPOINT_BYTES);
    }

    #[test]
    fn binary_voxel_checkpoint_rejects_noncanonical_and_unbounded_inputs() {
        let config = compute_config();
        let empty = MappingBackendSpec::Voxel.into_kernel(config.window_capacity);
        let payload = export_mapping_kernel_checkpoint(&empty).expect("encode empty checkpoint");
        assert_eq!(payload.len(), MAPPING_BACKEND_VOXEL_HEADER_BYTES);

        let mut trailing = payload.clone();
        trailing.push(0);
        assert!(
            restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &trailing)
                .err()
                .expect("trailing byte must fail")
                .contains("length mismatch")
        );

        let mut unknown_flags = payload.clone();
        unknown_flags[13] = 1;
        assert!(restore_mapping_kernel_checkpoint(
            MappingBackendSpec::Voxel,
            config,
            &unknown_flags
        )
        .err()
        .expect("unknown flag must fail")
        .contains("flags/reserved"));

        let mut oversized_count = payload;
        oversized_count[MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + size_of::<u64>()
            ..MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + 2 * size_of::<u64>()]
            .copy_from_slice(&u64::MAX.to_le_bytes());
        assert!(restore_mapping_kernel_checkpoint(
            MappingBackendSpec::Voxel,
            config,
            &oversized_count,
        )
        .err()
        .expect("oversized point count must fail before allocation")
        .contains("voxel window contains"));

        let one_point = MappingKernel::Voxel(VoxelKernel {
            current_window: [Point3::new(1.0, 2.0, 3.0)].into_iter().collect(),
            window_cells: window_cells(&[Point3::new(1.0, 2.0, 3.0)].into_iter().collect()),
            capacity: config.window_capacity,
            last_candidate_point_count: 1,
            last_candidate_received_at_ns: 7,
            dirty_since_candidate: false,
            dynamic_scene: Default::default(),
        });
        let mut non_finite =
            export_mapping_kernel_checkpoint(&one_point).expect("encode one point");
        non_finite[MAPPING_BACKEND_VOXEL_HEADER_BYTES
            ..MAPPING_BACKEND_VOXEL_HEADER_BYTES + size_of::<f64>()]
            .copy_from_slice(&f64::NAN.to_bits().to_le_bytes());
        assert!(
            restore_mapping_kernel_checkpoint(MappingBackendSpec::Voxel, config, &non_finite)
                .err()
                .expect("non-finite point must fail")
                .contains("non-finite")
        );
    }

    #[test]
    fn ray_cadence_skipped_state_remains_within_checkpoint_budget_before_ack() {
        let worker = MappingBackendWorker::start(
            MappingBackendSpec::raytracing_defaults(),
            1,
            compute_config(),
        )
        .expect("start ray worker");
        let client = worker.client();
        for sequence in 1..=3 {
            let seeded = client
                .submit(token(sequence), input(sequence))
                .expect("submit ray seed frame")
                .blocking_recv()
                .expect("ray seed response");
            commit_completion(seeded).expect("ray seed compute");
        }

        let mut skipped_input = input(4);
        skipped_input.source_timestamp_ns = 3_000_000_000 + MAPPING_CANDIDATE_INTERVAL_NS / 2;
        skipped_input.received_at_ns = skipped_input.source_timestamp_ns;
        let mut skipped_token = token(4);
        skipped_token.source_timestamp_ns = skipped_input.source_timestamp_ns;
        let skipped = client
            .submit(skipped_token.clone(), skipped_input)
            .expect("submit cadence-skipped ray frame")
            .blocking_recv()
            .expect("cadence-skipped ray response");
        assert!(commit_completion(skipped)
            .expect("cadence-skipped ray compute")
            .is_none());

        let flushed = client
            .flush(skipped_token)
            .expect("submit ray flush")
            .blocking_recv()
            .expect("ray flush response");
        assert!(commit_completion(flushed).expect("ray flush").is_some());
        let payload = client
            .export_checkpoint()
            .expect("submit ray export")
            .blocking_recv()
            .expect("ray export response")
            .expect("ray checkpoint");
        assert!(payload.len() <= MAX_MAPPING_BACKEND_CHECKPOINT_BYTES);
        worker.stop_and_join().expect("join ray worker");
    }

    #[test]
    fn ray_worker_exceeds_real_checkpoint_limit_and_keeps_mapping_after_export_failure() {
        let worker = MappingBackendWorker::start(
            MappingBackendSpec::raytracing_defaults(),
            1,
            compute_config(),
        )
        .expect("启动默认射线工作线程");
        let client = worker.client();
        // 92 个互不相交的水平面，每面 92×92 个有限实点；每束射线仍在默认量程内。
        // 所有状态均由正常输入接纳及射线更新产生，不改预算、计数或检查点数据。
        for sequence in 1..=92 {
            let mut frame = input(sequence);
            let height = sequence as f64 + 0.05;
            frame.sensor_origin_map = [0.0, 0.0, height];
            frame.registered_points_map = (0..92)
                .flat_map(|x| (0..92).map(move |y| {
                    Point3::new(x as f64 * 0.1 - 4.55, y as f64 * 0.1 - 4.55, height)
                }))
                .collect();
            let mut frame_token = token(sequence);
            frame_token.source_timestamp_ns = frame.source_timestamp_ns;
            let result = client.submit(frame_token, frame).unwrap().blocking_recv().unwrap();
            assert!(commit_completion(result).expect("合法在线帧不能被未来导出大小拒绝").is_some());
        }

        let mut skipped = input(93);
        skipped.sensor_origin_map = [0.0, 0.0, 92.05];
        skipped.registered_points_map = vec![Point3::new(6.05, 6.05, 92.05)];
        skipped.source_timestamp_ns = 92_000_000_001;
        skipped.received_at_ns = skipped.source_timestamp_ns;
        let mut skipped_token = token(93);
        skipped_token.source_timestamp_ns = skipped.source_timestamp_ns;
        let completion = client.submit(skipped_token.clone(), skipped).unwrap().blocking_recv().unwrap();
        assert!(commit_completion(completion).expect("超限后仍按节拍接纳新帧").is_none());
        let flush = client.flush(skipped_token.clone()).unwrap().blocking_recv().unwrap();
        assert!(commit_completion(flush).expect("超限后的脏状态仍可刷新").is_some());
        let flush = client.flush(skipped_token).unwrap().blocking_recv().unwrap();
        assert!(commit_completion(flush).expect("超限后的已刷新状态不被再次拒绝").is_none());

        let export_error = client.export_checkpoint().unwrap().blocking_recv().unwrap()
            .expect_err("实际检查点必须超过原 64MiB 上限");
        assert!(export_error.contains("payload size"), "{export_error}");
        assert!(export_error.contains(&format!("1..={MAX_MAPPING_BACKEND_CHECKPOINT_BYTES}")), "{export_error}");
        eprintln!("真实射线超限导出: {export_error}");

        let mut next = input(94);
        next.sensor_origin_map = [0.0, 0.0, 93.05];
        next.registered_points_map = vec![Point3::new(1.05, 1.05, 93.05)];
        let mut next_token = token(94);
        next_token.source_timestamp_ns = next.source_timestamp_ns;
        let completion = client.submit(next_token, next).unwrap().blocking_recv().unwrap();
        let candidate = commit_completion(completion).expect("导出失败后工作线程仍可推进")
            .expect("新帧达到构图节拍");
        assert_eq!(candidate.source_frame_sequence, 94);
        assert_eq!(candidate.source_timestamp_ns, 94_000_000_000);
        assert_eq!(candidate.accepted_points_map, vec![Point3::new(1.05, 1.05, 93.05)]);
        assert!(client.export_checkpoint().unwrap().blocking_recv().unwrap().unwrap_err()
            .contains("payload size"));
        worker.into_stop_join_handle().expect("请求超限工作线程停止")
            .join().expect("超限工作线程实际回收");
    }

    #[test]
    fn failed_worker_checkpoint_restore_is_transactional() {
        let worker = MappingBackendWorker::start(MappingBackendSpec::Voxel, 1, compute_config())
            .expect("start worker");
        let client = worker.client();
        let seed = client
            .submit(token(1), input(1))
            .expect("submit seed frame")
            .blocking_recv()
            .expect("seed response");
        commit_completion(seed).expect("seed compute");
        let before = client
            .export_checkpoint()
            .expect("submit export")
            .blocking_recv()
            .expect("export response")
            .expect("export checkpoint");
        let mut invalid = before.clone();
        invalid[MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES
            ..MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES + size_of::<u64>()]
            .copy_from_slice(
                &u64::try_from(compute_config().window_capacity + 1)
                    .expect("test capacity fits u64")
                    .to_le_bytes(),
            );
        let error = client
            .restore_checkpoint(invalid)
            .expect("submit invalid restore")
            .blocking_recv()
            .expect("invalid restore response")
            .expect_err("capacity mismatch must fail");
        assert!(error.contains("capacity mismatch"));
        let after = client
            .export_checkpoint()
            .expect("submit post-failure export")
            .blocking_recv()
            .expect("post-failure export response")
            .expect("post-failure checkpoint");
        assert_eq!(after, before);
        worker.stop_and_join().expect("join worker");
    }

    #[test]
    fn failed_candidate_construction_does_not_mutate_backend_state() {
        let mut kernel = MappingBackendSpec::raytracing_defaults().into_kernel(1024);
        for sequence in 1..=3 {
            compute_candidate(&mut kernel, &input(sequence), compute_config())
                .expect("seed ray state");
        }
        let MappingKernel::Raytracing(before) = &kernel else {
            panic!("ray kernel expected");
        };
        let voxel_count = before.map.voxels.len();
        let stable_points = kernel.accumulated_points(&[]);
        let last_candidate_point_count = before.last_candidate_point_count;
        let last_candidate_received_at_ns = before.last_candidate_received_at_ns;
        let mut invalid_config = compute_config();
        invalid_config.resolution = 0.0;
        let result = compute_candidate(&mut kernel, &input(4), invalid_config);
        assert!(result.is_err());
        let MappingKernel::Raytracing(ray) = &kernel else {
            panic!("ray kernel expected");
        };
        assert_eq!(ray.map.voxels.len(), voxel_count);
        assert_eq!(kernel.accumulated_points(&[]), stable_points);
        assert_eq!(ray.last_candidate_point_count, last_candidate_point_count);
        assert_eq!(
            ray.last_candidate_received_at_ns,
            last_candidate_received_at_ns
        );
    }


}

#[cfg(test)]
#[path = "mapping_backend_ray_count_equivalence_tests.rs"]
pub(crate) mod ray_count_equivalence_tests;

/// Offline copy only: retain the original backend configuration and update its
/// persisted members from the same exact fine-voxel removal set as the preview.
pub(crate) fn cleanup_completed_checkpoint(payload:&[u8], q:f32,
    removed:&ahash::AHashSet<[i32;3]>, retained:&[[i32;3]], occupancy:OccupancyGrid)->Result<Vec<u8>,String>{
    use nuwax_cloud_cleanup::offline_residual_cleanup::asset_key;
    if payload.len()<MAPPING_BACKEND_CHECKPOINT_HEADER_BYTES{return Err("建图后端存档不完整".into());}
    let mut reader=MappingCheckpointReader::new(payload);
    let spec=match payload[12]{0=>MappingBackendSpec::Voxel,1=>MappingBackendSpec::Raytracing(Default::default()),_=>return Err("该地图没有点云建图后端".into())};
    decode_mapping_checkpoint_header(&mut reader,spec)?;
    let kernel=match spec {
        MappingBackendSpec::Voxel=>{
            let capacity=reader.read_u64()? as usize;let count=reader.read_u64()? as usize;
            let _old_count=reader.read_u64()?;let stamp=reader.read_u64()?;
            let mut points=VecDeque::new();for _ in 0..count{
                let p=Point3::new(reader.read_f64()?,reader.read_f64()?,reader.read_f64()?);
                if !removed.contains(&asset_key([p.x,p.y,p.z],q)){points.push_back(p);}
            }reader.finish()?;
            // 窗口格计数与恢复检查点时相同，按清理后保留的窗口点重建。
            let window_cells=window_cells(&points);
            MappingKernel::Voxel(VoxelKernel{last_candidate_point_count:points.len(),window_cells,current_window:points,capacity,last_candidate_received_at_ns:stamp,dirty_since_candidate:false,dynamic_scene:Default::default()})
        },
        MappingBackendSpec::Raytracing(_)=>{
            let c=decode_ray_config(&mut reader)?;
            let config=Config{voxel_size:c.voxel_size,max_range:c.max_range,ray_subsample:c.ray_subsample,
                shadow_depth:c.shadow_depth,grace_depth:c.grace_depth,min_health:c.min_health,max_health:c.max_health,
                graze_cos:c.graze_cos,recency_window:c.recency_window,emit_every:c.emit_every,global_emit_every:c.global_emit_every,region_percentile:c.region_percentile};
            let _old_count=reader.read_u64()?;let stamp=reader.read_u64()?;
            let map_len=reader.read_u64()? as usize;let occ_len=reader.read_u64()? as usize;
            let mut map=decode_voxel_map_checkpoint_for_config(reader.read_bytes(map_len)?,MAX_MAPPING_BACKEND_CHECKPOINT_BYTES/VOXEL_MAP_CHECKPOINT_ENTRY_BYTES,&config)?;
            reader.read_bytes(occ_len)?;reader.finish()?;
            let inv=1.0f32/config.voxel_size;
            let ray_key=|k:&[i32;3]|{let p=k.map(|v|((v as f64+0.5)*f64::from(q)) as f32);((p[0]*inv).floor() as i32,(p[1]*inv).floor() as i32,(p[2]*inv).floor() as i32)};
            let keep:ahash::AHashSet<_>=retained.iter().map(ray_key).collect();
            let cut:ahash::AHashSet<_>=removed.iter().map(ray_key).filter(|k|!keep.contains(k)).collect();
            map.voxels.retain(|k,_|!cut.contains(k));
            MappingKernel::Raytracing(RaytracingKernel{last_candidate_point_count:map.healthy_count(),map,config,prepared_update:None,input:Vec::new(),last_occupancy:Some(occupancy),last_candidate_received_at_ns:stamp,dirty_since_candidate:false,dynamic_scene:Default::default()})
        },_=>unreachable!()
    };
    export_mapping_kernel_checkpoint(&kernel)
}
