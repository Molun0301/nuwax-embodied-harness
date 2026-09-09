// Phase 6 (codex-round3 R3-4)：build_handle_for_tests 是 #[cfg(test)] 内部
// 用的 helper,dead_code 是预期行为;needless_range_loop 与 too_many_arguments
// 是 typed_codec 长 payload 拼接的合理选择,不强行收敛。
#![allow(dead_code, clippy::needless_range_loop, clippy::too_many_arguments)]

//! LCM-first geometry runtime state loop。
//!
//! 该模块通过 `nuwax_transport::LcmTransport` 订阅 LCM typed channel，将 typed frame
//! 累积到 `SharedState`，供 gRPC facade（`server.rs`）查询/订阅。
//!
//! ## backend 策略
//! - 唯一 backend:`LcmTransport::new_real(&lcm_url)`,基于 `nuwax-transport/real-lcm` feature。
//! - `NUWAX_LCM_BACKEND` 仅接受 `real-lcm` 或空值（默认 real-lcm）；
//!   其它值在启动期以 `LCM_BACKEND_UNSUPPORTED` 退出。
//! - 不允许降级到其它传输后端。
//!
//! ## channel 命名约定
//! 与 `00_执行方案.md` §2.3 一致：
//! - `/odom#nav_msgs.Odometry`
//! - `/tf#tf2_msgs.TFMessage`
//! - `/map/occupancy#nav_msgs.OccupancyGrid`
//! - `/go2/lidar/points#sensor_msgs.PointCloud2`
//! - `/nav/state#nuwax_msgs.NavigationState`
//! - `/explore/frontiers#nuwax_msgs.FrontierList`
//!
//! 这里的 `nav_msgs/Odometry` 等名称仅是 LCM typed message 命名空间字符串。

use std::collections::{HashMap, HashSet, VecDeque};
use std::future::Future;
use std::pin::Pin;
use std::sync::{Arc, LazyLock, Weak};
use std::time::Duration;

use async_trait::async_trait;
use futures::FutureExt;
use nuwax_codec::lcm::schema;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame};
use nuwax_tf::{make_transform, TfTree, TransformStamped};
use nuwax_transport::LcmTransport;
use thiserror::Error;
use tokio::sync::{broadcast, mpsc, watch, RwLock};
use tokio::task::{JoinHandle, JoinSet};
use tracing::{debug, error, info, warn};

use crate::mapping_adapter::{
    finish_backend_retirement, MappingAdapter, MappingAdapterConfig, MappingBackendRetirement,
};
use crate::mapping_backend::MappingBackendSpec;
use crate::occupancy_shm::OccupancyGridShmWriter;
use crate::pointcloud::CloudMapShmWriter;
use crate::sequence_state::{sequence_state_path_from_env, SequenceStateStore};

const MAP_FANOUT_SINK_TIMEOUT: Duration = Duration::from_secs(2);
const EXPLORATION_CANCEL_RETRY_INTERVAL: Duration = Duration::from_millis(500);
const GEOMETRY_TASK_START_ACK_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(not(test))]
const GEOMETRY_TASK_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(15);
#[cfg(test)]
const GEOMETRY_TASK_SHUTDOWN_TIMEOUT: Duration = Duration::from_secs(2);
const GLOBAL_MAP_CHANNEL: &str = "/global_map";
const POINT_CLOUD2_MESSAGE_TYPE: &str = "sensor_msgs.PointCloud2";

// Phase 4 (channel registry) + Phase 7 (readiness cache): 统一在这里导入,
// 避免每个函数 body 内部重复 `use crate::channel_registry::...`。
//
// 同时 `use crate::channel_registry;` 让 module 名进 scope,内部多处以
// `channel_registry::channel_for(...)` 限定调用,无需每次都写完整 `crate::` 路径。
use crate::channel_registry;
use crate::channel_registry::ChannelKey;

#[cfg(test)]
#[path = "vertical_v2_canary.rs"]
mod vertical_v2_canary;

// ============================================================================
// 状态定义
// ============================================================================

/// Geometry runtime 启动失败。保留稳定错误码供启动脚本识别，同时携带
/// transport、持久化或配置阶段的真实失败原因，避免把所有错误误报为缺 liblcm。
#[derive(Debug, Error, PartialEq, Eq)]
#[error("LCM_BACKEND_UNAVAILABLE: category={category}: {detail}")]
pub struct RealLcmBackendUnavailable {
    category: &'static str,
    detail: String,
}

impl RealLcmBackendUnavailable {
    fn transport(detail: impl Into<String>) -> Self {
        Self {
            category: "transport",
            detail: detail.into(),
        }
    }

    fn persistence(detail: impl Into<String>) -> Self {
        Self {
            category: "persistence",
            detail: detail.into(),
        }
    }

    fn configuration(detail: impl Into<String>) -> Self {
        Self {
            category: "configuration",
            detail: detail.into(),
        }
    }
}

#[derive(Clone, Default)]
pub struct PoseState {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub frame_id: String,
    pub timestamp_ns: u64,
}

#[derive(Clone, Default)]
pub struct MapState {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f64,
    pub origin_y: f64,
    pub frame_id: String,
    pub revision: u64,
    pub timestamp_ns: u64,
    pub data: Vec<i8>,
}

#[derive(Clone, Default)]
pub struct TransformEntry {
    pub parent: String,
    pub child: String,
    pub tx: f64,
    pub ty: f64,
    pub tz: f64,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
    pub rw: f64,
    /// 该 TF 边的 stamp（纳秒，LCM 时间域）。0 表示无戳（latched/静态边），永不过期。
    pub stamp_ns: u64,
}

#[derive(Clone, Default)]
pub struct NavState {
    pub status: String,
    pub message: String,
    pub goal_id: String,
    pub distance_remaining: f32,
    pub timestamp_ns: u64,
}

#[derive(Clone, Debug, Default)]
pub struct NavigationFeedback {
    pub cancel_phase: String,
    pub cancel_generation: u64,
    pub actuator_zeroed: bool,
    pub reason_code: String,
    pub runtime_generation: u64,
    pub guard_generation: u64,
    pub sink_ready: bool,
    pub planner_map_frame: String,
    pub planner_pose_frame: String,
    pub zero_ack_id: String,
    pub zero_applied_monotonic_ns: u64,
    pub owner_kind: i32,
    pub parent_request_id: String,
    pub goal_generation: u64,
    pub path_revision: u64,
    pub goal_payload_sha256: [u8; 32],
    pub path_payload_sha256: [u8; 32],
}

#[derive(Clone)]
pub struct NavigationUpdate {
    pub state: NavState,
    pub feedback: NavigationFeedback,
    pub producer_runtime_generation: u64,
}

impl NavigationUpdate {
    pub async fn from_local(
        state: NavState,
        nav: &crate::nav_runtime::NavRuntimeState,
        lease: Option<crate::nav_runtime::GoalLease>,
    ) -> Self {
        let cancel = nav.cancel_snapshot();
        let cancel_phase = match cancel.phase {
            crate::nav_runtime::CancelPhase::Idle => "IDLE",
            crate::nav_runtime::CancelPhase::Pending => "PENDING",
            crate::nav_runtime::CancelPhase::Zeroed => "ZEROED",
            crate::nav_runtime::CancelPhase::Failed => "FAILED",
            crate::nav_runtime::CancelPhase::TimedOut => "TIMED_OUT",
        };
        let actuator_zeroed = cancel.phase == crate::nav_runtime::CancelPhase::Zeroed;
        let reason_code = navigation_reason_code(&state, cancel_phase);
        Self {
            state,
            producer_runtime_generation: lease
                .as_ref()
                .map_or(0, |lease| lease.producer_runtime_generation),
            feedback: NavigationFeedback {
                cancel_phase: cancel_phase.to_string(),
                cancel_generation: cancel.generation,
                actuator_zeroed,
                reason_code,
                owner_kind: lease.as_ref().map_or(0, |lease| lease.owner_kind),
                parent_request_id: lease
                    .as_ref()
                    .map_or_else(String::new, |lease| lease.parent_request_id.clone()),
                goal_generation: lease.as_ref().map_or(0, |lease| lease.goal_generation),
                path_revision: lease.as_ref().map_or(0, |lease| lease.path_revision),
                goal_payload_sha256: lease
                    .as_ref()
                    .map_or([0; 32], |lease| lease.goal_payload_sha256),
                path_payload_sha256: lease
                    .as_ref()
                    .map_or([0; 32], |lease| lease.path_payload_sha256),
                ..NavigationFeedback::default()
            },
        }
    }
}

fn navigation_reason_code(state: &NavState, cancel_phase: &str) -> String {
    if cancel_phase == "ZEROED" {
        return "CANCEL_ZEROED".to_string();
    }
    if cancel_phase == "TIMED_OUT" {
        return "CANCEL_ZERO_ACK_TIMEOUT".to_string();
    }
    let message = state.message.as_str();
    if let Some(start) = message.find('[') {
        if let Some(end) = message[start + 1..].find(']') {
            return message[start + 1..start + 1 + end].to_string();
        }
    }
    state.status.clone()
}

#[derive(Clone, Default)]
pub struct FrontierEntry {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub frame_id: String,
    pub size_cells: u32,
    pub heading: f32,
}

pub struct SharedState {
    pub pose: PoseState,
    /// 位姿与坐标树提交完成后唤醒订阅；通知只保留最新变化，数据仍以本状态为准。
    pub(crate) pose_changed: watch::Sender<()>,
    pub map: MapState,
    /// Canonical 10-second TF history used by pose synthesis and gRPC frame inventory.
    pub tf_tree: Arc<TfTree>,
    /// Latest-edge compatibility mirror for older internal tests and diagnostics.
    pub transforms: Vec<TransformEntry>,
    pub nav_state: NavState,
    pub cloud_map_revision: u64,
    pub frontiers: Vec<FrontierEntry>,
    pub frontiers_revision: u64,
}

impl Default for SharedState {
    fn default() -> Self {
        Self {
            pose: PoseState::default(),
            pose_changed: watch::channel(()).0,
            map: MapState::default(),
            tf_tree: Arc::new(TfTree::new()),
            transforms: Vec::new(),
            nav_state: NavState {
                status: "IDLE".into(),
                message: "空闲".into(),
                ..Default::default()
            },
            cloud_map_revision: 0,
            frontiers: Vec::new(),
            frontiers_revision: 0,
        }
    }
}

impl SharedState {
    /// Return the latest transform for every child retained by the canonical TF tree.
    pub fn latest_transform_entries(&self) -> Vec<TransformEntry> {
        self.tf_tree
            .frames()
            .into_iter()
            .filter_map(|frame| self.tf_tree.lookup(&frame).ok())
            .filter_map(transform_to_entry)
            .collect()
    }
}

#[derive(Clone, Debug)]
pub struct CloudMapNotification {
    pub revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
    pub session_generation: u64,
}

#[derive(Clone, Debug)]
pub(crate) enum CloudMapSessionAdmission {
    Registered(CloudMapNotification),
    WriterUnavailable,
    ClearRejected(String),
    AdmissionClosed(String),
}

pub(crate) struct CloudMapSessionReservation {
    activation_tx: Option<tokio::sync::oneshot::Sender<u64>>,
    update_rx: tokio::sync::oneshot::Receiver<CloudMapSessionAdmission>,
}

pub(crate) enum CloudMapSessionRegistration {
    Admitted(CloudMapSessionReservation),
    Rejected(CloudMapSessionAdmission),
}

impl CloudMapSessionReservation {
    pub(crate) async fn activate(mut self, session_generation: u64) -> CloudMapSessionAdmission {
        let Some(activation_tx) = self.activation_tx.take() else {
            return CloudMapSessionAdmission::AdmissionClosed(
                "cloud-map clear reservation was already activated".to_owned(),
            );
        };
        if activation_tx.send(session_generation).is_err() {
            return CloudMapSessionAdmission::AdmissionClosed(
                "cloud-map clear reservation task was unavailable".to_owned(),
            );
        }
        self.update_rx.await.unwrap_or_else(|error| {
            CloudMapSessionAdmission::ClearRejected(format!(
                "cloud-map clear task did not report its admission result: {error}"
            ))
        })
    }
}

/// RALPLAN §Phase M #3: occupancy grid revision 事件。
///
/// `mapping_adapter::MappingAdapter::build_and_publish` 在 commit 新 revision
/// 后构造本结构并通过 `broadcast::Sender<MapRevisionEvent>` 写入兼容入口；
/// coalescer 将其收敛为 latest-only `watch`，SHM/LCM/explore/gRPC 分别消费，
/// 避免慢下游阻塞其它链路。`data` / `raw_data` 使用 `Arc<[u8]>`，跨下游
/// clone 不复制地图；二者属于同一 revision，避免 Web 再做一次有竞态的 unary 读取。
///
/// 字段对齐 W29 `MapStreamEventBroker.StreamSnapshot.parsed` schema:
/// - `revision` —— geometry 进程级 occupancy revision(u64,跨 mapping session 单调递增)
/// - `data`     —— costmap 字节(`i8 as u8`,长度 = `width * height`)
/// - `size`     —— `data.len()`(u64,等价于 wire 上 rbuf.data_size)
/// - `schema_version` —— `NativeMapManifest` schema 版本(当前 v1)
/// - `checksum` —— SHA-256(`data`)的 hex(由 `map_checksum::sha256_hex` 算)
/// - `last_seen_ms` —— commit 时 `now_ns / 1_000_000`(i64,毫秒,给前端排序)
#[derive(Clone, Debug)]
pub struct MapRevisionEvent {
    pub session_id: String,
    pub map_name: String,
    pub session_generation: u64,
    pub revision: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f64,
    pub origin_y: f64,
    pub frame_id: String,
    pub timestamp_ns: u64,
    pub data: Arc<[u8]>,
    pub raw_data: Arc<[u8]>,
    pub size: u64,
    pub schema_version: u32,
    pub checksum: String,
    pub raw_checksum: String,
    pub last_seen_ms: i64,
}

impl MapRevisionEvent {
    pub fn from_map(
        map: &MapState,
        session: &crate::mapping_adapter::MappingSessionSnapshot,
    ) -> Self {
        let data: Arc<[u8]> = map
            .data
            .iter()
            .map(|&value| value as u8)
            .collect::<Vec<_>>()
            .into();
        let checksum = crate::map_checksum::sha256_hex(data.as_ref());
        Self {
            session_id: session.session_id.clone(),
            map_name: session.map_name.clone(),
            session_generation: session.session_generation,
            revision: map.revision,
            width: map.width,
            height: map.height,
            resolution: map.resolution,
            origin_x: map.origin_x,
            origin_y: map.origin_y,
            frame_id: map.frame_id.clone(),
            timestamp_ns: map.timestamp_ns,
            size: data.len() as u64,
            checksum: checksum.clone(),
            raw_checksum: checksum,
            last_seen_ms: (map.timestamp_ns / 1_000_000) as i64,
            schema_version: 1,
            raw_data: Vec::<u8>::new().into(),
            data,
        }
    }

    pub fn from_effective_and_raw(
        effective: &MapState,
        raw: &MapState,
        session: &crate::mapping_adapter::MappingSessionSnapshot,
    ) -> Self {
        let mut event = Self::from_map(effective, session);
        let same_layout = raw.revision == effective.revision
            && raw.width == effective.width
            && raw.height == effective.height
            && raw.resolution == effective.resolution
            && raw.origin_x == effective.origin_x
            && raw.origin_y == effective.origin_y
            && raw.frame_id == effective.frame_id
            && raw.timestamp_ns == effective.timestamp_ns;
        if !same_layout {
            warn!(
                effective_revision = effective.revision,
                raw_revision = raw.revision,
                "raw occupancy layout differs from effective revision; stream falls back to effective data"
            );
            return event;
        }

        // 未叠加禁区的同内容地图共用已编码字节与摘要，不再分配并散列第二份整图。
        if raw.data == effective.data {
            return event;
        }

        let raw_data: Arc<[u8]> = raw
            .data
            .iter()
            .map(|&value| value as u8)
            .collect::<Vec<_>>()
            .into();
        event.raw_checksum = crate::map_checksum::sha256_hex(raw_data.as_ref());
        if event.raw_checksum != event.checksum {
            event.raw_data = raw_data;
        }
        event
    }

    pub fn to_map_state(&self) -> MapState {
        MapState {
            width: self.width,
            height: self.height,
            resolution: self.resolution,
            origin_x: self.origin_x,
            origin_y: self.origin_y,
            frame_id: self.frame_id.clone(),
            revision: self.revision,
            timestamp_ns: self.timestamp_ns,
            data: self.data.iter().map(|&value| value as i8).collect(),
        }
    }
}

/// Latest successfully written occupancy SHM reference.
///
/// Only the map-revision SHM sink may advance this snapshot. gRPC readers use
/// it as an immutable reference and never replay an older map into the mmap.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OccupancyShmSnapshot {
    pub key: String,
    pub file_size: u64,
    pub payload_size: u64,
    pub revision: u64,
    pub layout_version: u32,
    pub payload_schema_version: u32,
    pub generation: u64,
    pub active_slot: usize,
}

fn encode_occupancy_revision(event: &MapRevisionEvent) -> Result<Vec<u8>, String> {
    encode_occupancy_revision_in_frame(event, "map")
}

fn encode_occupancy_revision_in_frame(
    event: &MapRevisionEvent,
    expected_map_frame: &str,
) -> Result<Vec<u8>, String> {
    if event.frame_id != expected_map_frame {
        return Err(format!(
            "global occupancy fanout requires frame={expected_map_frame}, received {}",
            event.frame_id,
        ));
    }
    let expected_len = usize::try_from(event.width)
        .ok()
        .and_then(|width| {
            usize::try_from(event.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| "occupancy dimensions overflow usize".to_string())?;
    if expected_len != event.data.len() || event.size != event.data.len() as u64 {
        return Err(format!(
            "occupancy revision {} dimensions/size mismatch: {}x{}, size={}, data={}",
            event.revision,
            event.width,
            event.height,
            event.size,
            event.data.len()
        ));
    }
    if crate::map_checksum::sha256_hex(event.data.as_ref()) != event.checksum {
        return Err(format!(
            "occupancy revision {} checksum mismatch",
            event.revision
        ));
    }
    let wire_revision = i32::try_from(event.revision).map_err(|_| {
        format!(
            "occupancy revision {} exceeds positive LCM i32 domain",
            event.revision
        )
    })?;
    if wire_revision <= 0 {
        return Err(format!(
            "occupancy revision {} must be positive",
            event.revision
        ));
    }
    let stamp_sec = i32::try_from(event.timestamp_ns / 1_000_000_000)
        .map_err(|_| "occupancy timestamp seconds exceed int32".to_string())?;
    let width = i32::try_from(event.width).map_err(|_| "occupancy width exceeds int32")?;
    let height = i32::try_from(event.height).map_err(|_| "occupancy height exceeds int32")?;
    schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
        header: schema::Header {
            seq: wire_revision,
            stamp_sec,
            stamp_nsec: (event.timestamp_ns % 1_000_000_000) as i32,
            frame_id: expected_map_frame.to_string(),
        },
        info: schema::MapMetaData {
            map_load_time_sec: stamp_sec,
            map_load_time_nsec: (event.timestamp_ns % 1_000_000_000) as i32,
            resolution: event.resolution,
            width,
            height,
            origin: schema::Pose {
                position: [event.origin_x, event.origin_y, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        },
        data: event.data.iter().map(|value| *value as i8).collect(),
    })
    .map_err(|error| format!("encode occupancy revision failed: {error}"))
}

pub(crate) fn committed_map_identity_from_revision(
    event: &MapRevisionEvent,
) -> Result<schema::CommittedMapIdentity, String> {
    let session_generation = i64::try_from(event.session_generation)
        .map_err(|_| "committed map session_generation exceeds i64".to_string())?;
    let revision = i64::try_from(event.revision)
        .map_err(|_| "committed map revision exceeds i64".to_string())?;
    if event.session_id.trim().is_empty()
        || session_generation <= 0
        || revision <= 0
        || event.frame_id.trim().is_empty()
    {
        return Err("committed map identity is incomplete".to_string());
    }
    let payload_sha256 = crate::map_checksum::sha256_bytes(event.data.as_ref());
    if crate::map_checksum::sha256_hex(event.data.as_ref()) != event.checksum {
        return Err("committed map payload checksum mismatch".to_string());
    }
    Ok(schema::CommittedMapIdentity {
        session_id: event.session_id.clone(),
        session_generation,
        revision,
        payload_sha256,
        frame_id: event.frame_id.clone(),
    })
}

fn exploration_costmap_from_revision(
    event: &MapRevisionEvent,
) -> Result<crate::explore_runtime::ExploreCostmapSnapshot, String> {
    exploration_costmap_from_revision_in_frame(event, "map")
}

fn exploration_costmap_from_revision_in_frame(
    event: &MapRevisionEvent,
    expected_map_frame: &str,
) -> Result<crate::explore_runtime::ExploreCostmapSnapshot, String> {
    if event.frame_id != expected_map_frame {
        return Err(format!(
            "global exploration requires frame={expected_map_frame}, received {}",
            event.frame_id,
        ));
    }
    let expected_len = usize::try_from(event.width)
        .ok()
        .and_then(|width| {
            usize::try_from(event.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| "exploration costmap dimensions overflow usize".to_string())?;
    if expected_len != event.data.len() {
        return Err("exploration costmap cells length does not match dimensions".to_string());
    }
    let cells = event
        .data
        .iter()
        .map(|value| match *value as i8 {
            value if value < 0 => OccupancyCell::Unknown,
            0 => OccupancyCell::Free,
            value if value >= 100 => OccupancyCell::Occupied,
            value => OccupancyCell::Cost(value as u8),
        })
        .collect();
    Ok(crate::explore_runtime::ExploreCostmapSnapshot {
        grid: OccupancyGrid2D::new(
            Point2 {
                x: event.origin_x,
                y: event.origin_y,
            },
            f64::from(event.resolution),
            event.width,
            event.height,
            cells,
        ),
        frame_id: expected_map_frame.to_string(),
    })
}

fn map_event_matches_current_session(
    event: &MapRevisionEvent,
    mapping_adapter: &Arc<parking_lot::Mutex<MappingAdapter>>,
) -> bool {
    let snapshot = mapping_adapter.lock().snapshot();
    snapshot.active
        && event.session_generation > 0
        && event.session_generation == snapshot.session_generation
        && event.session_id == snapshot.session_id
        && event.map_name == snapshot.map_name
}

type GeometryTaskFuture = Pin<Box<dyn Future<Output = Result<(), String>> + Send + 'static>>;

#[derive(Default)]
pub(crate) struct ExploreProjectionGate {
    pending_latest: Option<MapRevisionEvent>,
}

pub(crate) type ExploreProjectionOwnedGuard = tokio::sync::OwnedMutexGuard<ExploreProjectionGate>;

fn newer_projection_event(current: &MapRevisionEvent, candidate: &MapRevisionEvent) -> bool {
    candidate.session_generation > current.session_generation
        || (candidate.session_generation == current.session_generation
            && candidate.revision > current.revision)
}

fn same_projection_event(current: &MapRevisionEvent, candidate: &MapRevisionEvent) -> bool {
    current.session_id == candidate.session_id
        && current.map_name == candidate.map_name
        && current.session_generation == candidate.session_generation
        && current.revision == candidate.revision
        && current.checksum == candidate.checksum
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GeometryTaskClass {
    Persistent,
    Retained,
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GeometryTopologyInjectedTerminal {
    Eof,
    TransportError,
    Panic,
    JoinFailure,
}

#[cfg(test)]
#[derive(Default)]
pub(crate) struct GeometryTopologyTestControl {
    events: parking_lot::Mutex<Vec<String>>,
    task_control: parking_lot::Mutex<Option<GeometryRuntimeTaskControl>>,
    startup_shutdown_emitter:
        parking_lot::Mutex<Option<crate::lio_owner::GeometryShutdownEventEmitter>>,
    startup_shutdown_recorder:
        parking_lot::Mutex<Option<crate::lio_owner::GeometryShutdownEventRecorder>>,
    startup_shutdown_emitted: parking_lot::Mutex<Vec<crate::lio_owner::GeometryShutdownStage>>,
    rejected_registration: parking_lot::Mutex<Option<&'static str>>,
    injected_terminal: parking_lot::Mutex<Option<(&'static str, GeometryTopologyInjectedTerminal)>>,
    terminal_released: std::sync::atomic::AtomicBool,
    terminal_release: tokio::sync::Notify,
    revision_prepare_enabled: std::sync::atomic::AtomicBool,
    revision_prepare_entered: std::sync::atomic::AtomicBool,
    revision_prepare_released: std::sync::atomic::AtomicBool,
    scheduler_block_enabled: std::sync::atomic::AtomicBool,
    scheduler_block_entered: std::sync::atomic::AtomicBool,
    scheduler_block_released: std::sync::atomic::AtomicBool,
    cancel_retry_enabled: std::sync::atomic::AtomicBool,
    cancel_retry_entered: std::sync::atomic::AtomicBool,
    cancel_retry_released: std::sync::atomic::AtomicBool,
    cancel_retry_release: tokio::sync::Notify,
}

#[cfg(test)]
impl GeometryTopologyTestControl {
    pub(crate) fn new() -> Arc<Self> {
        let control = Arc::new(Self::default());
        let (emitter, recorder) = crate::lio_owner::GeometryShutdownEventEmitter::recording(0);
        *control.startup_shutdown_emitter.lock() = Some(emitter);
        *control.startup_shutdown_recorder.lock() = Some(recorder);
        control
    }

    pub(crate) fn install(self: &Arc<Self>) -> GeometryTopologyTestInstallation {
        GeometryTopologyTestInstallation {
            control: self.clone(),
        }
    }

    pub(crate) fn reject_registration(&self, name: &'static str) {
        *self.rejected_registration.lock() = Some(name);
    }

    pub(crate) fn inject_terminal(
        &self,
        name: &'static str,
        terminal: GeometryTopologyInjectedTerminal,
    ) {
        *self.injected_terminal.lock() = Some((name, terminal));
    }

    pub(crate) fn release_terminal(&self) {
        self.terminal_released
            .store(true, std::sync::atomic::Ordering::Release);
        self.terminal_release.notify_waiters();
    }

    pub(crate) fn events(&self) -> Vec<String> {
        self.events.lock().clone()
    }

    pub(crate) fn task_control(&self) -> GeometryRuntimeTaskControl {
        self.task_control
            .lock()
            .as_ref()
            .cloned()
            .expect("production topology task admission control")
    }

    pub(crate) fn startup_shutdown_stages(&self) -> Vec<crate::lio_owner::GeometryShutdownStage> {
        self.startup_shutdown_recorder
            .lock()
            .as_ref()
            .expect("production startup shutdown recorder")
            .stages()
    }

    pub(crate) fn enable_revision_prepare_probe(&self) {
        self.revision_prepare_enabled
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn revision_prepare_entered(&self) -> bool {
        self.revision_prepare_entered
            .load(std::sync::atomic::Ordering::Acquire)
    }

    pub(crate) fn release_revision_prepare(&self) {
        self.revision_prepare_released
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn enable_scheduler_block(&self) {
        self.scheduler_block_enabled
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn scheduler_block_entered(&self) -> bool {
        self.scheduler_block_entered
            .load(std::sync::atomic::Ordering::Acquire)
    }

    pub(crate) fn release_scheduler_block(&self) {
        self.scheduler_block_released
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn enable_cancel_retry_gate(&self) {
        self.cancel_retry_enabled
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn cancel_retry_entered(&self) -> bool {
        self.cancel_retry_entered
            .load(std::sync::atomic::Ordering::Acquire)
    }

    pub(crate) fn release_cancel_retry(&self) {
        self.cancel_retry_released
            .store(true, std::sync::atomic::Ordering::Release);
        self.cancel_retry_release.notify_waiters();
    }

    fn release_all(&self) {
        self.release_terminal();
        self.release_revision_prepare();
        self.release_scheduler_block();
        self.release_cancel_retry();
    }

    fn attach_task_control(&self, control: GeometryRuntimeTaskControl) {
        *self.task_control.lock() = Some(control);
    }

    fn clear_task_control(&self) {
        self.task_control.lock().take();
    }

    fn emit_startup_shutdown_stage(&self, stage: crate::lio_owner::GeometryShutdownStage) {
        let should_emit = {
            let mut emitted = self.startup_shutdown_emitted.lock();
            if emitted.contains(&stage) {
                false
            } else {
                emitted.push(stage);
                true
            }
        };
        if should_emit {
            self.startup_shutdown_emitter
                .lock()
                .as_ref()
                .expect("production startup shutdown emitter")
                .emit_completed(stage);
        }
    }

    fn record(&self, event: impl Into<String>) {
        self.events.lock().push(event.into());
    }

    fn should_reject_registration(&self, name: &'static str) -> bool {
        self.rejected_registration.lock().as_ref() == Some(&name)
    }

    fn injected_terminal_for(
        &self,
        name: &'static str,
    ) -> Option<GeometryTopologyInjectedTerminal> {
        self.injected_terminal
            .lock()
            .as_ref()
            .and_then(|(registered, terminal)| (*registered == name).then_some(*terminal))
    }
}

#[cfg(test)]
tokio::task_local! {
    static TEST_GEOMETRY_TOPOLOGY_CONTROL: Arc<GeometryTopologyTestControl>;
}

#[cfg(test)]
pub(crate) struct GeometryTopologyTestInstallation {
    control: Arc<GeometryTopologyTestControl>,
}

#[cfg(test)]
impl Drop for GeometryTopologyTestInstallation {
    fn drop(&mut self) {
        self.control.release_all();
        self.control.clear_task_control();
    }
}

#[cfg(test)]
pub(crate) fn topology_test_control() -> Option<Arc<GeometryTopologyTestControl>> {
    TEST_GEOMETRY_TOPOLOGY_CONTROL.try_with(Arc::clone).ok()
}

#[cfg(test)]
fn record_topology_event(event: impl Into<String>) {
    if let Some(control) = topology_test_control() {
        control.record(event);
    }
}

#[cfg(test)]
async fn wait_for_topology_cancel_retry_release(control: Option<Arc<GeometryTopologyTestControl>>) {
    let Some(control) = control else {
        return;
    };
    if !control
        .cancel_retry_enabled
        .load(std::sync::atomic::Ordering::Acquire)
    {
        return;
    }
    control
        .cancel_retry_entered
        .store(true, std::sync::atomic::Ordering::Release);
    control.record("cancel-retry-entered");
    loop {
        let released = control.cancel_retry_release.notified();
        if control
            .cancel_retry_released
            .load(std::sync::atomic::Ordering::Acquire)
        {
            return;
        }
        released.await;
    }
}

#[cfg(test)]
async fn wait_for_topology_terminal_release() {
    let Some(control) = topology_test_control() else {
        return;
    };
    loop {
        let released = control.terminal_release.notified();
        if control
            .terminal_released
            .load(std::sync::atomic::Ordering::Acquire)
        {
            return;
        }
        released.await;
    }
}

#[cfg(test)]
fn wait_for_topology_revision_prepare_release() {
    let Some(control) = topology_test_control() else {
        return;
    };
    if !control
        .revision_prepare_enabled
        .load(std::sync::atomic::Ordering::Acquire)
    {
        return;
    }
    control
        .revision_prepare_entered
        .store(true, std::sync::atomic::Ordering::Release);
    control.record("revision-prepare-entered");
    while !control
        .revision_prepare_released
        .load(std::sync::atomic::Ordering::Acquire)
    {
        std::thread::yield_now();
    }
}

#[cfg(test)]
pub(crate) fn wait_for_topology_scheduler_release(
    control: Option<Arc<GeometryTopologyTestControl>>,
) {
    let Some(control) = control else {
        return;
    };
    if !control
        .scheduler_block_enabled
        .load(std::sync::atomic::Ordering::Acquire)
    {
        return;
    }
    control
        .scheduler_block_entered
        .store(true, std::sync::atomic::Ordering::Release);
    control.record("scheduler-block-entered");
    while !control
        .scheduler_block_released
        .load(std::sync::atomic::Ordering::Acquire)
    {
        std::thread::yield_now();
    }
}

struct GeometryTaskRegistration {
    name: &'static str,
    class: GeometryTaskClass,
    future: GeometryTaskFuture,
    started_tx: Option<tokio::sync::oneshot::Sender<()>>,
    #[cfg(test)]
    injected_terminal: Option<GeometryTopologyInjectedTerminal>,
    #[cfg(test)]
    topology_control: Option<Arc<GeometryTopologyTestControl>>,
}

struct GeometryTaskAdmission {
    open: bool,
    registration_tx: mpsc::UnboundedSender<GeometryTaskRegistration>,
}

struct GeometryRuntimeTaskControlInner {
    admission: parking_lot::Mutex<GeometryTaskAdmission>,
    shutdown_request_tx: watch::Sender<bool>,
    persistent_stop_tx: watch::Sender<bool>,
    #[cfg(test)]
    topology_control: Option<Arc<GeometryTopologyTestControl>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum GeometryTaskRegistrationFailureKind {
    AdmissionClosed,
    SupervisorUnavailable,
}

struct RejectedGeometryTaskRegistration {
    kind: GeometryTaskRegistrationFailureKind,
    registration: GeometryTaskRegistration,
}

impl RejectedGeometryTaskRegistration {
    fn detail(&self) -> String {
        match self.kind {
            GeometryTaskRegistrationFailureKind::AdmissionClosed => format!(
                "Geometry task admission is closed: {}",
                self.registration.name
            ),
            GeometryTaskRegistrationFailureKind::SupervisorUnavailable => format!(
                "Geometry task supervisor is unavailable: {}",
                self.registration.name
            ),
        }
    }
}

/// Cloneable control-plane token used by Phase A and dynamic Geometry work.
/// The registration mutex is the linearization point between an accepted task
/// and closing admission; accepted work is therefore always visible to the
/// non-clone root owner before shutdown can settle producers.
#[derive(Clone)]
pub(crate) struct GeometryRuntimeTaskControl {
    inner: Arc<GeometryRuntimeTaskControlInner>,
}

impl GeometryRuntimeTaskControl {
    fn channel() -> (Self, mpsc::UnboundedReceiver<GeometryTaskRegistration>) {
        #[cfg(test)]
        return Self::channel_with_topology_control(None);

        #[cfg(not(test))]
        {
            let (registration_tx, registration_rx) = mpsc::unbounded_channel();
            let (shutdown_request_tx, _) = watch::channel(false);
            let (persistent_stop_tx, _) = watch::channel(false);
            (
                Self {
                    inner: Arc::new(GeometryRuntimeTaskControlInner {
                        admission: parking_lot::Mutex::new(GeometryTaskAdmission {
                            open: true,
                            registration_tx,
                        }),
                        shutdown_request_tx,
                        persistent_stop_tx,
                    }),
                },
                registration_rx,
            )
        }
    }

    #[cfg(test)]
    fn channel_with_topology_control(
        topology_control: Option<Arc<GeometryTopologyTestControl>>,
    ) -> (Self, mpsc::UnboundedReceiver<GeometryTaskRegistration>) {
        let (registration_tx, registration_rx) = mpsc::unbounded_channel();
        let (shutdown_request_tx, _) = watch::channel(false);
        let (persistent_stop_tx, _) = watch::channel(false);
        let attached_topology_control = topology_control.clone();
        let control = Self {
            inner: Arc::new(GeometryRuntimeTaskControlInner {
                admission: parking_lot::Mutex::new(GeometryTaskAdmission {
                    open: true,
                    registration_tx,
                }),
                shutdown_request_tx,
                persistent_stop_tx,
                topology_control,
            }),
        };
        if let Some(topology_control) = attached_topology_control {
            topology_control.attach_task_control(control.clone());
        }
        (control, registration_rx)
    }

    pub(crate) fn close_admission(&self) {
        self.inner.admission.lock().open = false;
    }

    pub(crate) fn admission_is_open(&self) -> bool {
        self.inner.admission.lock().open
    }

    fn request_stop(&self) {
        self.close_admission();
        self.inner.shutdown_request_tx.send_replace(true);
    }

    fn request_persistent_stop(&self) {
        self.inner.persistent_stop_tx.send_replace(true);
    }

    fn shutdown_request_receiver(&self) -> watch::Receiver<bool> {
        self.inner.shutdown_request_tx.subscribe()
    }

    fn shutdown_receiver(&self) -> watch::Receiver<bool> {
        self.inner.persistent_stop_tx.subscribe()
    }

    fn registration(
        name: &'static str,
        class: GeometryTaskClass,
        future: impl Future<Output = Result<(), String>> + Send + 'static,
    ) -> GeometryTaskRegistration {
        GeometryTaskRegistration {
            name,
            class,
            future: Box::pin(future),
            started_tx: None,
            #[cfg(test)]
            injected_terminal: None,
            #[cfg(test)]
            topology_control: None,
        }
    }

    fn registration_with_start_ack(
        name: &'static str,
        class: GeometryTaskClass,
        future: impl Future<Output = Result<(), String>> + Send + 'static,
    ) -> (GeometryTaskRegistration, tokio::sync::oneshot::Receiver<()>) {
        let (started_tx, started_rx) = tokio::sync::oneshot::channel();
        (
            GeometryTaskRegistration {
                name,
                class,
                future: Box::pin(future),
                started_tx: Some(started_tx),
                #[cfg(test)]
                injected_terminal: None,
                #[cfg(test)]
                topology_control: None,
            },
            started_rx,
        )
    }

    fn try_register(
        &self,
        registration: GeometryTaskRegistration,
    ) -> Result<(), RejectedGeometryTaskRegistration> {
        #[cfg(test)]
        let mut registration = registration;
        #[cfg(test)]
        let topology_control = self.inner.topology_control.as_ref();
        #[cfg(test)]
        if let Some(control) = topology_control {
            if control.should_reject_registration(registration.name) {
                control.record(format!("rejected:{}", registration.name));
                return Err(RejectedGeometryTaskRegistration {
                    kind: GeometryTaskRegistrationFailureKind::AdmissionClosed,
                    registration,
                });
            }
            if registration.injected_terminal.is_none() {
                registration.injected_terminal = control.injected_terminal_for(registration.name);
            }
            registration.topology_control = Some(Arc::clone(control));
        }
        let admission = self.inner.admission.lock();
        if !admission.open {
            return Err(RejectedGeometryTaskRegistration {
                kind: GeometryTaskRegistrationFailureKind::AdmissionClosed,
                registration,
            });
        }
        #[cfg(test)]
        if let Some(control) = topology_control {
            control.record(format!("registered:{}", registration.name));
        }
        admission
            .registration_tx
            .send(registration)
            .map_err(|error| RejectedGeometryTaskRegistration {
                kind: GeometryTaskRegistrationFailureKind::SupervisorUnavailable,
                registration: error.0,
            })
    }

    fn register(
        &self,
        name: &'static str,
        class: GeometryTaskClass,
        future: impl Future<Output = Result<(), String>> + Send + 'static,
    ) -> Result<(), String> {
        self.try_register(Self::registration(name, class, future))
            .map_err(|error| error.detail())
    }

    fn register_persistent(
        &self,
        name: &'static str,
        future: impl Future<Output = Result<(), String>> + Send + 'static,
    ) -> Result<(), String> {
        self.register(name, GeometryTaskClass::Persistent, future)
    }

    fn register_retained(
        &self,
        name: &'static str,
        future: impl Future<Output = Result<(), String>> + Send + 'static,
    ) -> Result<(), String> {
        self.register(name, GeometryTaskClass::Retained, future)
    }

    pub(crate) fn register_mapping_backend_retirement(
        &self,
        adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
        lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
        retirement: MappingBackendRetirement,
    ) -> Result<(), (String, MappingBackendRetirement)> {
        let retained = Arc::new(parking_lot::Mutex::new(Some(retirement)));
        let task_retained = Arc::clone(&retained);
        let task = async move {
            let retirement = task_retained
                .lock()
                .take()
                .ok_or_else(|| "mapping backend rejection retirement owner was empty".to_owned())?;
            finish_backend_retirement(&adapter, &lifecycle_lock, Some(retirement))
                .await
                .map_err(|error| error.to_string())
        };
        let registration = Self::registration(
            "mapping-backend-rejection-retirement",
            GeometryTaskClass::Retained,
            task,
        );
        match self.try_register(registration) {
            Ok(()) => Ok(()),
            Err(rejected) => {
                let retirement = retained
                    .lock()
                    .take()
                    .expect("rejected retirement registration must retain its capability");
                Err((rejected.detail(), retirement))
            }
        }
    }

    #[cfg(test)]
    fn register_injected_terminal(
        &self,
        name: &'static str,
        terminal: GeometryTopologyInjectedTerminal,
    ) -> Result<(), String> {
        let mut registration =
            Self::registration(name, GeometryTaskClass::Persistent, std::future::pending());
        registration.injected_terminal = Some(terminal);
        self.try_register(registration)
            .map_err(|error| error.detail())
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct GeometryTaskSettlement {
    settled: bool,
}

impl GeometryTaskSettlement {
    pub(crate) fn is_settled(self) -> bool {
        self.settled
    }
}

#[derive(Clone, Debug)]
pub(crate) struct GeometryTaskShutdownOutcome {
    pub(crate) result: Result<(), String>,
    settlement: GeometryTaskSettlement,
}

impl GeometryTaskShutdownOutcome {
    fn new(result: Result<(), String>, settled: bool) -> Self {
        Self {
            result,
            settlement: GeometryTaskSettlement { settled },
        }
    }

    pub(crate) fn unsettled(error: impl Into<String>) -> Self {
        Self::new(Err(error.into()), false)
    }

    pub(crate) fn producers_settled(&self) -> bool {
        self.settlement.is_settled()
    }

    pub(crate) fn into_parts(self) -> (Result<(), String>, GeometryTaskSettlement) {
        (self.result, self.settlement)
    }
}

/// Non-clone root owner for every Geometry observer, tool, and finite child.
pub(crate) struct GeometryRuntimeTaskOwner {
    control: GeometryRuntimeTaskControl,
    supervisor: Option<JoinHandle<GeometryTaskShutdownOutcome>>,
    terminal_rx: watch::Receiver<Option<String>>,
    terminal: Option<GeometryTaskShutdownOutcome>,
}

impl GeometryRuntimeTaskOwner {
    fn start(
        control: GeometryRuntimeTaskControl,
        registration_rx: mpsc::UnboundedReceiver<GeometryTaskRegistration>,
    ) -> Self {
        let (terminal_tx, terminal_rx) = watch::channel(None);
        let supervisor_control = control.clone();
        let supervisor = tokio::spawn(run_geometry_task_supervisor(
            supervisor_control,
            registration_rx,
            terminal_tx,
        ));
        Self {
            control,
            supervisor: Some(supervisor),
            terminal_rx,
            terminal: None,
        }
    }

    pub(crate) fn control(&self) -> GeometryRuntimeTaskControl {
        self.control.clone()
    }

    pub(crate) fn close_admission(&self) {
        self.control.close_admission();
    }

    pub(crate) fn completed_failure(&self) -> Option<String> {
        self.terminal_rx.borrow().clone()
    }

    pub(crate) fn is_finished(&self) -> bool {
        self.terminal.is_some()
            || self.terminal_rx.borrow().is_some()
            || self.supervisor.as_ref().is_none_or(JoinHandle::is_finished)
    }

    pub(crate) async fn wait_terminal(&mut self) -> Result<(), String> {
        if let Some(outcome) = self.terminal.clone() {
            return outcome.result;
        }
        if let Some(error) = self.terminal_rx.borrow().clone() {
            return Err(error);
        }

        let supervisor = self
            .supervisor
            .as_mut()
            .ok_or_else(|| "Geometry task supervisor join already consumed".to_owned())?;
        tokio::select! {
            biased;
            changed = self.terminal_rx.changed() => {
                if changed.is_err() {
                    return Err("Geometry task terminal channel closed unexpectedly".to_owned());
                }
                let error = self.terminal_rx.borrow().clone().unwrap_or_else(|| {
                    "Geometry task terminal channel changed without a failure".to_owned()
                });
                Err(error)
            }
            joined = supervisor => {
                let outcome = joined.unwrap_or_else(|error| {
                    GeometryTaskShutdownOutcome::unsettled(format!(
                        "Geometry task supervisor join failed: {error}"
                    ))
                });
                self.supervisor.take();
                self.terminal = Some(outcome.clone());
                outcome.result
            }
        }
    }

    pub(crate) async fn shutdown(&mut self) -> GeometryTaskShutdownOutcome {
        self.control.request_stop();
        if let Some(outcome) = self.terminal.clone() {
            return outcome;
        }
        let Some(supervisor) = self.supervisor.as_mut() else {
            return self.terminal.clone().unwrap_or_else(|| {
                GeometryTaskShutdownOutcome::unsettled(
                    "Geometry task supervisor join was already consumed",
                )
            });
        };
        match tokio::time::timeout(GEOMETRY_TASK_SHUTDOWN_TIMEOUT, supervisor).await {
            Ok(joined) => {
                let outcome = joined.unwrap_or_else(|error| {
                    GeometryTaskShutdownOutcome::unsettled(format!(
                        "Geometry task supervisor join failed: {error}"
                    ))
                });
                self.supervisor.take();
                self.terminal = Some(outcome.clone());
                outcome
            }
            Err(_) => GeometryTaskShutdownOutcome::unsettled(
                "Geometry task supervisor shutdown timed out; final join handle retained",
            ),
        }
    }
}

impl Drop for GeometryRuntimeTaskOwner {
    fn drop(&mut self) {
        self.control.request_stop();
    }
}

async fn run_geometry_task_supervisor(
    control: GeometryRuntimeTaskControl,
    mut registration_rx: mpsc::UnboundedReceiver<GeometryTaskRegistration>,
    terminal_tx: watch::Sender<Option<String>>,
) -> GeometryTaskShutdownOutcome {
    let mut shutdown_request_rx = control.shutdown_request_receiver();
    let mut tasks = JoinSet::new();
    let mut stopping = *shutdown_request_rx.borrow();
    let mut retained_tasks = 0_usize;
    let mut persistent_stop_sent = false;
    let mut first_failure: Option<String> = None;

    loop {
        if stopping {
            while let Ok(registration) = registration_rx.try_recv() {
                spawn_registered_geometry_task(&mut tasks, registration, &mut retained_tasks);
            }
            if retained_tasks == 0 && !persistent_stop_sent {
                control.request_persistent_stop();
                persistent_stop_sent = true;
            }
            if tasks.is_empty() {
                break;
            }
        }

        tokio::select! {
            biased;
            changed = shutdown_request_rx.changed(), if !stopping => {
                if changed.is_err() || *shutdown_request_rx.borrow() {
                    stopping = true;
                }
            }
            registration = registration_rx.recv(), if !stopping => {
                match registration {
                    Some(registration) => {
                        spawn_registered_geometry_task(
                            &mut tasks,
                            registration,
                            &mut retained_tasks,
                        );
                    }
                    None => {
                        let failure = "Geometry task registration channel closed unexpectedly".to_owned();
                        terminal_tx.send_replace(Some(failure.clone()));
                        first_failure.get_or_insert(failure);
                        control.request_stop();
                        stopping = true;
                    }
                }
            }
            joined = tasks.join_next(), if !tasks.is_empty() => {
                let failure = match joined {
                    Some(Ok((name, class, result))) => {
                        if class == GeometryTaskClass::Retained {
                            retained_tasks = retained_tasks.saturating_sub(1);
                        }
                        match (class, result) {
                            (GeometryTaskClass::Persistent, Ok(())) if !stopping => {
                                Some(format!("persistent Geometry task exited unexpectedly: {name}"))
                            }
                            (_, Ok(())) => None,
                            (_, Err(error)) => {
                                Some(format!("Geometry task {name} failed: {error}"))
                            }
                        }
                    }
                    Some(Err(error)) => {
                        retained_tasks = 0;
                        Some(format!("Geometry task join failed: {error}"))
                    }
                    None => None,
                };
                if let Some(failure) = failure {
                    if first_failure.is_none() {
                        terminal_tx.send_replace(Some(failure.clone()));
                        first_failure = Some(failure);
                    }
                    control.request_stop();
                    stopping = true;
                }
            }
        }
    }

    GeometryTaskShutdownOutcome::new(first_failure.map_or(Ok(()), Err), true)
}

fn spawn_registered_geometry_task(
    tasks: &mut JoinSet<(&'static str, GeometryTaskClass, Result<(), String>)>,
    registration: GeometryTaskRegistration,
    retained_tasks: &mut usize,
) {
    if registration.class == GeometryTaskClass::Retained {
        *retained_tasks = (*retained_tasks).saturating_add(1);
    }
    tasks.spawn(async move {
        #[cfg(test)]
        let topology_control = registration.topology_control.clone();
        let run = async move {
            let GeometryTaskRegistration {
                name,
                class,
                future,
                started_tx,
                #[cfg(test)]
                injected_terminal,
                #[cfg(test)]
                    topology_control: _,
            } = registration;
            #[cfg(test)]
            record_topology_event(format!("started:{name}"));
            if let Some(started_tx) = started_tx {
                let _ = started_tx.send(());
            }
            let result = std::panic::AssertUnwindSafe(future)
                .catch_unwind()
                .await
                .unwrap_or_else(|_| Err(format!("Geometry task {name} panicked")));
            #[cfg(test)]
            if injected_terminal == Some(GeometryTopologyInjectedTerminal::JoinFailure) {
                panic!("injected join failure for {name}");
            }
            (name, class, result)
        };
        #[cfg(test)]
        {
            match topology_control {
                Some(control) => TEST_GEOMETRY_TOPOLOGY_CONTROL.scope(control, run).await,
                None => run.await,
            }
        }
        #[cfg(not(test))]
        {
            run.await
        }
    });
}

pub(crate) struct SpawnedGeometryRuntime {
    pub(crate) handle: GeometryRuntimeHandle,
    pub(crate) task_owner: GeometryRuntimeTaskOwner,
}

pub struct GeometryRuntimeStartupFailure {
    error: RealLcmBackendUnavailable,
    rollback_owner: Option<Box<GeometryRuntimeStartupRollbackOwner>>,
}

impl std::fmt::Debug for GeometryRuntimeStartupFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("GeometryRuntimeStartupFailure")
            .field("error", &self.error)
            .field("has_rollback_owner", &self.rollback_owner.is_some())
            .finish()
    }
}

impl std::fmt::Display for GeometryRuntimeStartupFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.error.fmt(formatter)
    }
}

impl std::error::Error for GeometryRuntimeStartupFailure {}

impl From<RealLcmBackendUnavailable> for GeometryRuntimeStartupFailure {
    fn from(error: RealLcmBackendUnavailable) -> Self {
        Self {
            error,
            rollback_owner: None,
        }
    }
}

impl GeometryRuntimeStartupFailure {
    fn with_rollback_owner(
        error: RealLcmBackendUnavailable,
        handle: GeometryRuntimeHandle,
        task_owner: GeometryRuntimeTaskOwner,
    ) -> Self {
        Self {
            error,
            rollback_owner: Some(Box::new(GeometryRuntimeStartupRollbackOwner {
                handle,
                task_owner,
                rejected_work: None,
                sticky_error: None,
            })),
        }
    }

    fn with_rejected_work_rollback_owner(
        error: RealLcmBackendUnavailable,
        handle: GeometryRuntimeHandle,
        task_owner: GeometryRuntimeTaskOwner,
        rejected_work: Option<GeometryRuntimeStartupRejectedWorkOwner>,
    ) -> Self {
        let sticky_error = error.to_string();
        Self {
            error,
            rollback_owner: Some(Box::new(GeometryRuntimeStartupRollbackOwner {
                handle,
                task_owner,
                rejected_work,
                sticky_error: Some(sticky_error),
            })),
        }
    }

    pub(crate) fn into_parts(
        self,
    ) -> (
        RealLcmBackendUnavailable,
        Option<Box<GeometryRuntimeStartupRollbackOwner>>,
    ) {
        (self.error, self.rollback_owner)
    }

    #[cfg(test)]
    pub(crate) fn has_rollback_owner_for_tests(&self) -> bool {
        self.rollback_owner.is_some()
    }
}

pub(crate) struct GeometryRuntimeStartupRejectedWorkOwner {
    name: &'static str,
    handle: Option<JoinHandle<GeometryTaskShutdownOutcome>>,
    terminal: Option<GeometryTaskShutdownOutcome>,
}

impl GeometryRuntimeStartupRejectedWorkOwner {
    fn spawn(
        name: &'static str,
        rejected_future: Option<GeometryTaskFuture>,
        pending_subscriptions: Vec<(&'static str, nuwax_transport::OwnedLcmSubscription)>,
        pending_children: Vec<(&'static str, JoinHandle<()>)>,
        #[cfg(test)] topology_control: Option<Arc<GeometryTopologyTestControl>>,
    ) -> Self {
        let rollback_inner = async move {
            let rejected_settlement = async {
                match rejected_future {
                    Some(future) => future.await,
                    None => Ok(()),
                }
            };
            let (rejected_result, subscription_result, child_result) = tokio::join!(
                rejected_settlement,
                rollback_owned_geometry_subscriptions(pending_subscriptions),
                rollback_pending_geometry_children(pending_children),
            );
            let subscriptions_settled = subscription_result.is_ok();
            let mut errors = Vec::new();
            if let Err(error) = rejected_result {
                errors.push(format!("rejected task settlement failed: {error}"));
            }
            if let Err(error) = subscription_result {
                errors.push(format!("pending subscription rollback failed: {error}"));
            }
            if let Err(error) = child_result {
                errors.push(format!("pending child rollback failed: {error}"));
            }
            let result = if errors.is_empty() {
                Ok(())
            } else {
                Err(errors.join("; "))
            };
            GeometryTaskShutdownOutcome::new(result, subscriptions_settled)
        };
        #[cfg(test)]
        let rollback = async move {
            match topology_control {
                Some(control) => {
                    TEST_GEOMETRY_TOPOLOGY_CONTROL
                        .scope(control, rollback_inner)
                        .await
                }
                None => rollback_inner.await,
            }
        };
        #[cfg(not(test))]
        let rollback = rollback_inner;
        Self {
            name,
            handle: Some(tokio::spawn(rollback)),
            terminal: None,
        }
    }

    async fn shutdown(&mut self) -> GeometryTaskShutdownOutcome {
        if let Some(outcome) = self.terminal.clone() {
            return outcome;
        }
        let Some(handle) = self.handle.as_mut() else {
            return GeometryTaskShutdownOutcome::unsettled(format!(
                "rejected Geometry task {} final join handle was lost",
                self.name
            ));
        };
        match tokio::time::timeout(GEOMETRY_TASK_SHUTDOWN_TIMEOUT, handle).await {
            Ok(joined) => {
                self.handle.take();
                let outcome = joined.unwrap_or_else(|error| {
                    GeometryTaskShutdownOutcome::unsettled(format!(
                        "rejected Geometry task {} final join failed: {error}",
                        self.name
                    ))
                });
                self.terminal = Some(outcome.clone());
                outcome
            }
            Err(_) => GeometryTaskShutdownOutcome::unsettled(format!(
                "rejected Geometry task {} shutdown timed out; final join handle retained",
                self.name
            )),
        }
    }
}

pub(crate) struct GeometryRuntimeStartupRollbackOwner {
    handle: GeometryRuntimeHandle,
    task_owner: GeometryRuntimeTaskOwner,
    rejected_work: Option<GeometryRuntimeStartupRejectedWorkOwner>,
    sticky_error: Option<String>,
}

pub(crate) struct GeometryRuntimeStartupRollbackFailure {
    detail: String,
    pending_owner: Option<Box<GeometryRuntimeStartupRollbackOwner>>,
}

impl std::fmt::Debug for GeometryRuntimeStartupRollbackFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("GeometryRuntimeStartupRollbackFailure")
            .field("detail", &self.detail)
            .field("has_pending_owner", &self.pending_owner.is_some())
            .finish()
    }
}

impl std::fmt::Display for GeometryRuntimeStartupRollbackFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.detail)
    }
}

impl GeometryRuntimeStartupRollbackFailure {
    pub(crate) fn into_pending_owner(self) -> Option<Box<GeometryRuntimeStartupRollbackOwner>> {
        self.pending_owner
    }
}

impl GeometryRuntimeStartupRollbackOwner {
    pub(crate) async fn shutdown(mut self) -> Result<(), GeometryRuntimeStartupRollbackFailure> {
        #[cfg(test)]
        let topology_control = self.task_owner.control.inner.topology_control.clone();
        let task_shutdown = self.task_owner.shutdown();
        let rejected_shutdown = async {
            match self.rejected_work.as_mut() {
                Some(owner) => owner.shutdown().await,
                None => GeometryTaskShutdownOutcome::new(Ok(()), true),
            }
        };
        let (task_outcome, rejected_outcome) = tokio::join!(task_shutdown, rejected_shutdown);
        if rejected_outcome.producers_settled() {
            self.rejected_work.take();
        }
        let mut rollback_errors = Vec::new();
        if let Err(error) = task_outcome.result.clone() {
            rollback_errors.push(format!("task-owner rollback failed: {error}"));
        }
        if let Err(error) = rejected_outcome.result.clone() {
            rollback_errors.push(format!("rejected task rollback failed: {error}"));
        }
        if !task_outcome.producers_settled() || !rejected_outcome.producers_settled() {
            let mut details = self.sticky_error.iter().cloned().collect::<Vec<_>>();
            details.extend(rollback_errors);
            if details.is_empty() {
                details.push("Geometry startup rollback producers remained unsettled".to_owned());
            }
            return Err(GeometryRuntimeStartupRollbackFailure {
                detail: details.join("; "),
                pending_owner: Some(Box::new(self)),
            });
        }
        #[cfg(test)]
        if let Some(control) = &topology_control {
            control.emit_startup_shutdown_stage(
                crate::lio_owner::GeometryShutdownStage::GeometryTasksSettled,
            );
        }
        let cleanup = cleanup_geometry_startup_files(&self.handle);
        if let Err(error) = cleanup {
            let mut details = self.sticky_error.iter().cloned().collect::<Vec<_>>();
            details.extend(rollback_errors);
            details.push(error);
            return Err(GeometryRuntimeStartupRollbackFailure {
                detail: details.join("; "),
                pending_owner: Some(Box::new(self)),
            });
        }
        #[cfg(test)]
        if let Some(control) = &topology_control {
            control.emit_startup_shutdown_stage(
                crate::lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
            );
        }
        let mut details = self.sticky_error.into_iter().collect::<Vec<_>>();
        details.extend(rollback_errors);
        if details.is_empty() {
            Ok(())
        } else {
            Err(GeometryRuntimeStartupRollbackFailure {
                detail: details.join("; "),
                pending_owner: None,
            })
        }
    }
}

#[cfg(test)]
pub(crate) fn attach_root_task_owner_for_tests(
    mut handle: GeometryRuntimeHandle,
) -> SpawnedGeometryRuntime {
    let (control, registration_rx) = GeometryRuntimeTaskControl::channel();
    handle.task_control = Some(control.clone());
    let task_owner = GeometryRuntimeTaskOwner::start(control.clone(), registration_rx);
    let mut shutdown_rx = control.shutdown_receiver();
    control
        .register_persistent("test-root-task-owner", async move {
            wait_for_geometry_task_stop(&mut shutdown_rx).await;
            Ok(())
        })
        .expect("test root task owner registration must succeed");
    SpawnedGeometryRuntime { handle, task_owner }
}

#[cfg(test)]
static PRODUCTION_TOPOLOGY_TEST_ENV_LOCK: LazyLock<Arc<tokio::sync::Mutex<()>>> =
    LazyLock::new(|| Arc::new(tokio::sync::Mutex::new(())));

#[cfg(test)]
struct ProductionTopologyTestEnv {
    _lock: tokio::sync::OwnedMutexGuard<()>,
    saved: Vec<(&'static str, Option<std::ffi::OsString>)>,
    root: std::path::PathBuf,
}

#[cfg(test)]
impl ProductionTopologyTestEnv {
    async fn install() -> Self {
        let lock = PRODUCTION_TOPOLOGY_TEST_ENV_LOCK.clone().lock_owned().await;
        let root = std::env::temp_dir().join(format!(
            "nuwax-geometry-production-topology-{}-{}",
            std::process::id(),
            uuid::Uuid::new_v4(),
        ));
        std::fs::create_dir_all(&root).expect("production topology test directory");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            std::fs::set_permissions(&root, std::fs::Permissions::from_mode(0o700))
                .expect("production topology test directory permissions");
        }
        let values = [
            (
                "NUWAX_CLOUDMAP_SHM_PATH",
                root.join("cloud-map.mmap").into_os_string(),
            ),
            (
                "NUWAX_OCCUPANCY_SHM_PATH",
                root.join("occupancy.mmap").into_os_string(),
            ),
            (
                "NUWAX_GEOMETRY_EXPLORATION_RECONCILIATION_JOURNAL_PATH",
                root.join("exploration-journal.json").into_os_string(),
            ),
            (
                "NUWAX_FORBIDDEN_ZONES_PATH",
                root.join("forbidden-zones.json").into_os_string(),
            ),
        ];
        let mut saved = Vec::with_capacity(values.len());
        for (key, value) in values {
            saved.push((key, std::env::var_os(key)));
            std::env::set_var(key, value);
        }
        Self {
            _lock: lock,
            saved,
            root,
        }
    }
}

#[cfg(test)]
impl Drop for ProductionTopologyTestEnv {
    fn drop(&mut self) {
        for (key, value) in self.saved.drain(..).rev() {
            match value {
                Some(value) => std::env::set_var(key, value),
                None => std::env::remove_var(key),
            }
        }
        let _ = std::fs::remove_dir_all(&self.root);
    }
}

#[cfg(test)]
pub(crate) struct ProductionTopologyTestSetup {
    env: ProductionTopologyTestEnv,
    pub(crate) control: Arc<GeometryTopologyTestControl>,
    _installation: GeometryTopologyTestInstallation,
    prepared: Option<PreparedGeometryRuntime>,
}

#[cfg(test)]
pub(crate) struct ProductionTopologyTestFixture {
    env: ProductionTopologyTestEnv,
    pub(crate) control: Arc<GeometryTopologyTestControl>,
    _installation: GeometryTopologyTestInstallation,
    spawned: Option<SpawnedGeometryRuntime>,
}

#[cfg(test)]
pub(crate) struct ProductionTopologyStartupFailureFixture {
    env: ProductionTopologyTestEnv,
    pub(crate) control: Arc<GeometryTopologyTestControl>,
    _installation: GeometryTopologyTestInstallation,
    failure: Option<GeometryRuntimeStartupFailure>,
}

#[cfg(test)]
impl ProductionTopologyTestSetup {
    pub(crate) async fn prepare(configure: impl FnOnce(&Arc<GeometryTopologyTestControl>)) -> Self {
        let env = ProductionTopologyTestEnv::install().await;
        let control = GeometryTopologyTestControl::new();
        configure(&control);
        let installation = control.install();
        let sequence_store = Arc::new(
            SequenceStateStore::open(env.root.join("sequence-state.json"))
                .expect("production topology sequence state"),
        );
        let nav_runtime =
            crate::nav_runtime::NavRuntimeState::try_from_sequence_store(sequence_store.as_ref())
                .expect("production topology navigation runtime");
        let prepared = PreparedGeometryRuntime {
            backend_name: "real-lcm",
            sequence_store,
            nav_runtime,
            transport: LcmTransport::new_real(&test_lcm_url())
                .expect("production topology real LCM transport"),
            backend_available: true,
            topology_control: Some(control.clone()),
        };
        Self {
            env,
            control,
            _installation: installation,
            prepared: Some(prepared),
        }
    }

    pub(crate) fn transport(&self) -> LcmTransport {
        self.prepared
            .as_ref()
            .expect("production topology prepared runtime remains available")
            .transport()
    }

    pub(crate) fn product_paths(&self) -> (std::path::PathBuf, std::path::PathBuf) {
        (
            self.env.root.join("cloud-map.mmap"),
            self.env.root.join("occupancy.mmap"),
        )
    }

    pub(crate) async fn spawn(
        mut self,
    ) -> Result<ProductionTopologyTestFixture, ProductionTopologyStartupFailureFixture> {
        let prepared = self
            .prepared
            .take()
            .expect("production topology prepared runtime remains available");
        let result = spawn_geometry_runtime(
            prepared,
            MappingBackendSpec::Voxel,
            channel_registry::ValidatedChannelBindings::from_env()
                .expect("production topology channel bindings"),
            crate::nav_runtime::path_envelope_binding()
                .expect("production topology PathEnvelope binding"),
            ValidatedFrameBindings::from_values("map", "odom", "base_link")
                .expect("production topology frame bindings"),
        )
        .await;
        match result {
            Ok(spawned) => Ok(ProductionTopologyTestFixture {
                env: self.env,
                control: self.control,
                _installation: self._installation,
                spawned: Some(spawned),
            }),
            Err(failure) => Err(ProductionTopologyStartupFailureFixture {
                env: self.env,
                control: self.control,
                _installation: self._installation,
                failure: Some(failure),
            }),
        }
    }
}

#[cfg(test)]
impl ProductionTopologyTestFixture {
    async fn try_start(
        configure: impl FnOnce(&Arc<GeometryTopologyTestControl>),
    ) -> Result<Self, ProductionTopologyStartupFailureFixture> {
        ProductionTopologyTestSetup::prepare(configure)
            .await
            .spawn()
            .await
    }

    pub(crate) async fn start(configure: impl FnOnce(&Arc<GeometryTopologyTestControl>)) -> Self {
        match Self::try_start(configure).await {
            Ok(fixture) => fixture,
            Err(failure) => {
                let detail = failure
                    .failure
                    .as_ref()
                    .map(ToString::to_string)
                    .unwrap_or_else(|| "startup failure was unavailable".to_owned());
                panic!("production topology spawn failed: {detail}");
            }
        }
    }

    pub(crate) fn take_spawned(&mut self) -> SpawnedGeometryRuntime {
        self.spawned
            .take()
            .expect("production topology owner remains available")
    }

    pub(crate) fn product_paths(&self) -> (std::path::PathBuf, std::path::PathBuf) {
        (
            self.env.root.join("cloud-map.mmap"),
            self.env.root.join("occupancy.mmap"),
        )
    }
}

#[cfg(test)]
impl ProductionTopologyStartupFailureFixture {
    pub(crate) fn take_failure(&mut self) -> GeometryRuntimeStartupFailure {
        self.failure
            .take()
            .expect("production topology startup failure remains available")
    }

    pub(crate) fn product_paths(&self) -> (std::path::PathBuf, std::path::PathBuf) {
        (
            self.env.root.join("cloud-map.mmap"),
            self.env.root.join("occupancy.mmap"),
        )
    }
}

#[cfg(test)]
const PRODUCTION_TOPOLOGY_PERSISTENT_SLOTS: [&str; 10] = [
    "navigation-state-consumer",
    "path-envelope-consumer",
    "public-tf-observer",
    "cancel-timeout-maintenance",
    "frontier-fanout",
    "exploration-scheduler",
    "map-revision-coalescer",
    "occupancy-lcm-fanout",
    "exploration-costmap-fanout",
    "forbidden-zone-watcher",
];

#[cfg(test)]
const PRODUCTION_TOPOLOGY_OWNED_CONSUMERS: [&str; 3] = [
    "navigation-state-consumer",
    "path-envelope-consumer",
    "public-tf-observer",
];

#[cfg(test)]
fn assert_production_topology_started(events: &[String]) {
    let expected = PRODUCTION_TOPOLOGY_PERSISTENT_SLOTS
        .into_iter()
        .flat_map(|name| [format!("registered:{name}"), format!("started:{name}")])
        .collect::<Vec<_>>();
    let actual = events
        .iter()
        .filter(|event| event.starts_with("registered:") || event.starts_with("started:"))
        .cloned()
        .collect::<Vec<_>>();
    assert_eq!(actual, expected, "production topology slot order drifted");
    for name in PRODUCTION_TOPOLOGY_PERSISTENT_SLOTS {
        let registered = format!("registered:{name}");
        let started = format!("started:{name}");
        let registered_at = events
            .iter()
            .position(|event| event == &registered)
            .unwrap_or_else(|| panic!("missing production registration event {registered}"));
        let started_at = events
            .iter()
            .position(|event| event == &started)
            .unwrap_or_else(|| panic!("missing production start ACK event {started}"));
        assert!(
            registered_at < started_at,
            "{name} started before registration"
        );
    }
}

#[cfg(test)]
async fn wait_for_topology_condition(label: &str, condition: impl Fn() -> bool) {
    tokio::time::timeout(Duration::from_secs(2), async {
        while !condition() {
            tokio::task::yield_now().await;
        }
    })
    .await
    .unwrap_or_else(|_| panic!("timed out waiting for {label}"));
}

#[cfg(test)]
fn assert_product_paths(paths: &(std::path::PathBuf, std::path::PathBuf), exist: bool) {
    assert_eq!(paths.0.exists(), exist, "cloud-map product path mismatch");
    assert_eq!(paths.1.exists(), exist, "occupancy product path mismatch");
}

#[cfg(test)]
fn cleanup_settled_topology_spawn(
    spawned: &SpawnedGeometryRuntime,
    outcome: &GeometryTaskShutdownOutcome,
) {
    assert!(outcome.producers_settled());
    cleanup_geometry_startup_files(&spawned.handle)
        .expect("settled production topology product cleanup");
}

#[cfg(test)]
fn assert_owned_consumers_unsubscribed_once(events: &[String]) {
    for name in PRODUCTION_TOPOLOGY_OWNED_CONSUMERS {
        assert_eq!(
            events
                .iter()
                .filter(|event| event.as_str() == format!("unsubscribed:{name}"))
                .count(),
            1,
            "{name} must acknowledge exactly one owned unsubscribe"
        );
    }
}

#[cfg(test)]
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn geometry_runtime_tasks_subscribe_before_ready_and_join_on_shutdown() {
    let mut fixture = lifecycle_fixture_with_topology_control(79, "subscription", |_| {}).await;
    let topology_control = fixture.topology_control();
    let events_at_constructor_return = topology_control.events();
    assert_production_topology_started(&events_at_constructor_return);
    assert_eq!(
        events_at_constructor_return
            .iter()
            .filter(|event| event.starts_with("registered:"))
            .count(),
        PRODUCTION_TOPOLOGY_PERSISTENT_SLOTS.len()
    );
    assert_eq!(
        events_at_constructor_return
            .iter()
            .filter(|event| event.starts_with("started:"))
            .count(),
        PRODUCTION_TOPOLOGY_PERSISTENT_SLOTS.len()
    );

    let paths = fixture.product_paths();
    assert_product_paths(&paths, true);
    let control = fixture
        .handle()
        .task_control()
        .expect("production root task control");
    let owner = fixture.take_owner();
    owner.close_ingress();
    assert!(!control.admission_is_open());
    let phase_a_rejection = control
        .register_retained("post-phase-a-probe", async { Ok(()) })
        .expect_err("Phase A must reject new retained work immediately");
    assert!(phase_a_rejection.contains("admission is closed"));
    assert!(!topology_control
        .events()
        .iter()
        .any(|event| event == "registered:post-phase-a-probe"));
    fixture.assert_downstream_held_before_task_settlement();
    finish_clean_production_shutdown(Box::new(owner), &fixture, "subscription").await;
    let events = topology_control.events();
    assert_owned_consumers_unsubscribed_once(&events);
    assert_product_paths(&paths, false);
}

#[cfg(test)]
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn geometry_runtime_task_failure_is_sticky_and_reaches_root_terminal() {
    for (index, (terminal_kind, expected)) in [
        (
            GeometryTopologyInjectedTerminal::Eof,
            "persistent Geometry task exited unexpectedly: public-tf-observer",
        ),
        (
            GeometryTopologyInjectedTerminal::TransportError,
            "Geometry task public-tf-observer failed: public TF receive failed: injected transport error",
        ),
        (
            GeometryTopologyInjectedTerminal::Panic,
            "Geometry task public-tf-observer panicked",
        ),
        (
            GeometryTopologyInjectedTerminal::JoinFailure,
            "Geometry task join failed:",
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let mut fixture = lifecycle_fixture_with_topology_control(
            80 + index as u64,
            "terminal",
            |control| {
                control.inject_terminal("public-tf-observer", terminal_kind);
            },
        )
        .await;
        let control = fixture.topology_control();
        assert_production_topology_started(&control.events());
        control.release_terminal();
        fixture
            .assert_geometry_terminal_reaches_main_arbitration(expected)
            .await;
        assert_owned_consumers_unsubscribed_once(&control.events());
        let owner = Box::new(fixture.take_owner());
        finish_fatal_production_shutdown(owner, &fixture, "terminal", expected).await;
    }
}

#[cfg(test)]
fn start_topology_mapping(
    handle: &GeometryRuntimeHandle,
    tag: &str,
) -> crate::mapping_adapter::MappingSessionSnapshot {
    let snapshot = handle
        .mapping_adapter
        .as_ref()
        .expect("production topology mapping adapter")
        .lock()
        .start_session(
            &format!("production-topology-session-{tag}"),
            &format!("production-topology-map-{tag}"),
        )
        .expect("production topology Mapping start");
    assert!(snapshot.active);
    snapshot
}

#[cfg(test)]
fn test_committed_map_identity(revision: u64) -> schema::CommittedMapIdentity {
    schema::CommittedMapIdentity {
        session_id: "exploration-test-map".to_string(),
        session_generation: 1,
        revision: i64::try_from(revision).expect("test committed map revision"),
        payload_sha256: crate::map_checksum::sha256_bytes(&revision.to_le_bytes()),
        frame_id: "map".to_string(),
    }
}

#[cfg(test)]
fn test_map_revision_authority(
    session_id: &str,
    map_name: &str,
    frame_id: &str,
    revision: u64,
) -> MapRevisionEvent {
    let data: Arc<[u8]> = vec![0].into();
    let checksum = crate::map_checksum::sha256_hex(data.as_ref());
    MapRevisionEvent {
        session_id: session_id.to_string(),
        map_name: map_name.to_string(),
        session_generation: 1,
        revision,
        width: 1,
        height: 1,
        resolution: 0.1,
        origin_x: 0.0,
        origin_y: 0.0,
        frame_id: frame_id.to_string(),
        timestamp_ns: revision,
        data,
        raw_data: Vec::<u8>::new().into(),
        size: 1,
        schema_version: 1,
        checksum: checksum.clone(),
        raw_checksum: checksum,
        last_seen_ms: 0,
    }
}

#[cfg(test)]
fn test_wavefront_goal_intent(
    goal_id: &str,
    parent_request_id: &str,
    producer_epoch: u64,
    cancel_generation: u64,
    committed_map: schema::CommittedMapIdentity,
) -> schema::GoalIntent {
    let timestamp_ns = crate::explore_runtime::now_unix_nanos();
    let goal = schema::PoseStamped {
        header: schema::Header {
            seq: 0,
            stamp_sec: i32::try_from(timestamp_ns / 1_000_000_000)
                .expect("test goal timestamp seconds"),
            stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
            frame_id: "map".to_string(),
        },
        pose: schema::Pose {
            position: [1.0, 2.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
    };
    let goal_sha256 = schema::goal_intent_goal_sha256(&goal).expect("test goal hash");
    let motion_admission = crate::nav_runtime::test_motion_admission();
    schema::GoalIntent {
        producer: "wavefront".to_string(),
        producer_epoch: i64::try_from(producer_epoch).expect("test producer epoch"),
        priority: 100,
        session_id: goal_id.to_string(),
        owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
        parent_request_id: parent_request_id.to_string(),
        goal_generation: 0,
        cancel_generation: i64::try_from(cancel_generation).expect("test cancel generation"),
        committed_map,
        boundary_id: String::new(),
        boundary_sha256: [0; 32],
        boundary_points: Vec::new(),
        motion_guard_generation: motion_admission.guard_generation as i64,
        motion_guard_boot_nonce: motion_admission.guard_boot_nonce,
        motion_stop_generation: motion_admission.motion_stop_generation as i64,
        goal_sha256,
        goal,
    }
}

#[cfg(test)]
async fn start_topology_exploration_goal(
    handle: &GeometryRuntimeHandle,
    goal_id: &str,
) -> crate::nav_runtime::GoalLease {
    handle.set_cmd_vel_sink_ready(true);
    handle
        .nav
        .observe_stack_state(NavState {
            status: "IDLE".to_owned(),
            message: "production topology heartbeat".to_owned(),
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            ..NavState::default()
        })
        .await;
    handle
        .nav
        .observe_tf_chain(crate::nav_runtime::TfChainObservation {
            generation: 1,
            map_frame: "map".to_owned(),
            odom_frame: "odom".to_owned(),
            base_frame: "base_link".to_owned(),
            map_to_odom: true,
            odom_to_base: true,
        })
        .expect("production topology TF observation");
    handle
        .nav
        .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
            generation: 1,
            map_revision: 1,
            map_frame: "map".to_owned(),
            pose_frame: "map".to_owned(),
            ready: true,
        })
        .expect("production topology planner observation");
    handle
        .nav
        .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
            generation: 1,
            ready: true,
        })
        .expect("production topology motion-sink observation");
    let committed_map = test_committed_map_identity(1);
    *handle
        .committed_map_authority
        .lock()
        .expect("production topology committed map authority") = Some(committed_map.clone());
    handle
        .map_revision_latest_tx
        .send_replace(Some(test_map_revision_authority(
            &committed_map.session_id,
            "exploration-test-map",
            &committed_map.frame_id,
            1,
        )));
    let mut grid = OccupancyGrid2D {
        resolution: 0.5,
        origin: Point2 { x: 0.0, y: 0.0 },
        width: 12,
        height: 12,
        cells: vec![OccupancyCell::Unknown; 144],
    };
    for y in 2..=7 {
        for x in 2..=7 {
            grid.set(x, y, OccupancyCell::Free);
        }
    }
    handle
        .explore
        .ingest_committed_costmap_snapshot(
            crate::explore_runtime::ExploreCostmapSnapshot {
                grid,
                frame_id: "map".to_string(),
            },
            committed_map,
        )
        .await
        .expect("production topology committed costmap");
    assert_eq!(
        handle
            .explore
            .start("production-topology-exploration", "prefer_wavefront")
            .await
            .status,
        crate::explore_runtime::STATUS_ACCEPTED
    );
    let producer_epoch_projection = handle.explore.producer_epoch_projection();
    let producer_epoch = producer_epoch_projection.load(std::sync::atomic::Ordering::Acquire);
    assert!(producer_epoch > 0);
    let admission_context = crate::nav_runtime::NavAdmissionContext::new(
        handle.mapping_lifecycle_lock.clone(),
        handle.committed_map_authority.clone(),
        handle.map_revision_latest_tx.subscribe(),
    );
    let admission_io = crate::nav_runtime::NavAdmissionIo::new(
        handle.state.clone(),
        handle.transport.clone(),
        handle.transport_backend_name,
        handle.lcm_backend_available,
    );
    let outcome = crate::nav_runtime::admit_goal_intent(
        &admission_context,
        &admission_io,
        &handle.nav,
        test_wavefront_goal_intent(
            goal_id,
            "production-topology-exploration",
            producer_epoch,
            handle.nav.cancel_snapshot().generation,
            test_committed_map_identity(1),
        ),
        crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
            current_session_generation: producer_epoch_projection,
        },
    )
    .await;
    assert_eq!(
        outcome.state.status, "ACCEPTED",
        "{}",
        outcome.state.message
    );
    outcome
        .lease
        .expect("accepted production topology goal lease")
}

#[cfg(test)]
async fn lifecycle_fixture_with_topology_control(
    generation: u64,
    tag: &str,
    configure: impl FnOnce(&Arc<GeometryTopologyTestControl>),
) -> crate::lifecycle_tests::LifecycleProductionFixture {
    let setup = ProductionTopologyTestSetup::prepare(configure).await;
    crate::lifecycle_tests::lifecycle_production_fixture_with_setup(generation, tag, false, setup)
        .await
}

#[cfg(test)]
async fn first_blocked_production_shutdown(
    fixture: &mut crate::lifecycle_tests::LifecycleProductionFixture,
    label: &str,
) -> Box<crate::GeometryProductionOwner> {
    let owner = fixture.take_owner();
    let shutdown = tokio::spawn(async move { owner.shutdown().await });
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(
        !shutdown.is_finished(),
        "{label} shutdown settled before release"
    );
    fixture.assert_downstream_held_before_task_settlement();

    let result = tokio::time::timeout(
        GEOMETRY_TASK_SHUTDOWN_TIMEOUT + Duration::from_secs(1),
        shutdown,
    )
    .await
    .unwrap_or_else(|_| panic!("{label} first shutdown attempt deadline"))
    .unwrap_or_else(|error| panic!("{label} shutdown task join failed: {error}"));
    let failure = match result {
        Ok(()) => panic!("{label} first shutdown unexpectedly settled"),
        Err(failure) => failure,
    };
    fixture.assert_downstream_held_before_task_settlement();
    failure
        .into_pending_owner()
        .unwrap_or_else(|| panic!("{label} first shutdown lost the production owner"))
}

#[cfg(test)]
async fn finish_clean_production_shutdown(
    owner: Box<crate::GeometryProductionOwner>,
    fixture: &crate::lifecycle_tests::LifecycleProductionFixture,
    label: &str,
) {
    tokio::time::timeout(Duration::from_secs(5), owner.shutdown())
        .await
        .unwrap_or_else(|_| panic!("{label} final shutdown deadline"))
        .unwrap_or_else(|error| panic!("{label} final shutdown failed: {error}"));
    fixture.assert_shutdown_complete();
}

#[cfg(test)]
async fn finish_fatal_production_shutdown(
    owner: Box<crate::GeometryProductionOwner>,
    fixture: &crate::lifecycle_tests::LifecycleProductionFixture,
    label: &str,
    expected: &str,
) {
    let failure = tokio::time::timeout(Duration::from_secs(5), owner.shutdown())
        .await
        .unwrap_or_else(|_| panic!("{label} final shutdown deadline"))
        .expect_err("fatal Geometry task must survive downstream settlement");
    let detail = failure.to_string();
    assert!(detail.contains(expected), "shutdown failure={detail:?}");
    assert!(
        failure.into_pending_owner().is_none(),
        "{label} final shutdown retained an unexpected owner"
    );
    fixture.assert_shutdown_complete();
}

#[cfg(test)]
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn geometry_runtime_retained_tasks_settle_before_mapping_and_shm_cleanup() {
    // Real retained cloud-map clear, blocked at its canonical publication fence.
    {
        let mut fixture = lifecycle_fixture_with_topology_control(61, "cloud-clear", |_| {}).await;
        let handle = fixture.handle();
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("production cloud-clear Mapping")
            .lock()
            .snapshot();
        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let admission = handle
            .begin_cloud_map_session(mapping.session_generation)
            .await;
        assert!(matches!(admission, CloudMapSessionAdmission::Registered(_)));
        wait_for_topology_condition("production cloud-map clear start", || {
            fixture
                .topology_control()
                .events()
                .iter()
                .any(|event| event == "started:cloud-map-clear")
        })
        .await;

        let pending = first_blocked_production_shutdown(&mut fixture, "cloud-clear").await;
        drop(lifecycle_guard);
        finish_clean_production_shutdown(pending, &fixture, "cloud-clear").await;
    }

    // Real correlated cancellation retry retained by the production root owner.
    {
        let mut fixture = lifecycle_fixture_with_topology_control(62, "cancel-retry", |control| {
            control.enable_cancel_retry_gate();
        })
        .await;
        let handle = fixture.handle();
        let lease = start_topology_exploration_goal(&handle, "production-topology-cancel").await;
        let sink = handle.explore.navigation_sink_for_tests();
        let ticket = sink
            .cancel(&exploration_child_lease(&lease))
            .await
            .expect("production correlated cancellation ticket");
        wait_for_topology_condition("production correlated retry gate", || {
            fixture.topology_control().cancel_retry_entered()
        })
        .await;
        let pending = first_blocked_production_shutdown(&mut fixture, "cancel-retry").await;
        let zeroed = handle
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: ticket.cancel_generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "production topology cancellation settled".to_owned(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        fixture.topology_control().release_cancel_retry();
        finish_clean_production_shutdown(pending, &fixture, "cancel-retry").await;
    }

    // A cancelled caller drops only its waiter; the real mapping-ingest task stays owned.
    {
        let mut fixture = lifecycle_fixture_with_topology_control(63, "ingest", |_| {}).await;
        let handle = fixture.handle();
        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("production mapping adapter")
            .clone();
        let mapping_lifecycle_lock = handle.mapping_lifecycle_lock.clone();
        let explore_projection_gate = handle.explore_projection_gate.clone();
        let committed_map_authority = handle.committed_map_authority.clone();
        let map_revision_latest_tx = handle.map_revision_latest_tx.clone();
        let cloud_map_writer = handle.cloud_map_writer.clone();
        let state = handle.state.clone();
        let task_control = handle
            .task_control()
            .expect("production mapping-ingest task control");
        let caller = tokio::spawn(async move {
            ingest_mapping_points_with_backend(
                &mapping,
                &mapping_lifecycle_lock,
                &explore_projection_gate,
                &committed_map_authority,
                &map_revision_latest_tx,
                &cloud_map_writer,
                &[],
                16,
                1,
                "map",
                1,
                &state,
                false,
                None,
                None,
                None,
                Some(&task_control),
                None,
            )
            .await
        });
        wait_for_topology_condition("production mapping-ingest start", || {
            let events = fixture.topology_control().events();
            events
                .iter()
                .any(|event| event == "registered:mapping-ingest")
                && events.iter().any(|event| event == "started:mapping-ingest")
        })
        .await;
        caller.abort();
        let caller_error = match caller.await {
            Err(error) => error,
            Ok(_) => panic!("mapping-ingest caller unexpectedly completed"),
        };
        assert!(caller_error.is_cancelled());

        let pending =
            first_blocked_production_shutdown(&mut fixture, "caller-cancelled-ingest").await;
        drop(lifecycle_guard);
        finish_clean_production_shutdown(pending, &fixture, "caller-cancelled-ingest").await;
    }
}

#[cfg(test)]
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn geometry_runtime_startup_registration_failure_never_detaches_pre_spawned_child() {
    for (index, rejected_name) in PRODUCTION_TOPOLOGY_PERSISTENT_SLOTS.into_iter().enumerate() {
        let blocks_pre_spawned_scheduler = rejected_name == "map-revision-coalescer";
        let setup = ProductionTopologyTestSetup::prepare(|control| {
            control.reject_registration(rejected_name);
            if blocks_pre_spawned_scheduler {
                control.enable_scheduler_block();
            }
        })
        .await;
        let paths = setup.product_paths();
        let control = setup.control.clone();
        let (lio, lio_shutdown) = crate::lifecycle_tests::start_lio_before_geometry_for_tests(
            setup.transport(),
            70 + index as u64,
            &format!("r{index}"),
        )
        .await;
        assert!(!lio_shutdown.is_shutdown_requested());
        let attempt = setup.spawn().await;
        let mut failed = match attempt {
            Ok(mut fixture) => {
                let spawned = fixture.take_spawned();
                let _ = crate::lio_owner::GeometryRuntimeOwner::new(spawned, lio)
                    .shutdown()
                    .await;
                panic!("production registration {rejected_name} unexpectedly succeeded");
            }
            Err(failed) => failed,
        };
        assert_eq!(failed.product_paths(), paths);
        let events = control.events();
        assert!(events
            .iter()
            .any(|event| event == &format!("rejected:{rejected_name}")));
        let failure = failed.take_failure();
        let registration_error = failure.to_string();
        assert!(
            registration_error.contains(rejected_name),
            "registration_error={registration_error}"
        );
        let task_control = control.task_control();
        assert!(
            !task_control.admission_is_open(),
            "{rejected_name} left public task admission open after Phase A"
        );
        let phase_a_rejection = task_control
            .register_retained("post-startup-rejection-probe", async { Ok(()) })
            .expect_err("startup Phase A must reject retained registration");
        assert!(
            phase_a_rejection.contains("admission is closed"),
            "{rejected_name} probe rejection={phase_a_rejection:?}"
        );
        assert!(!control
            .events()
            .iter()
            .any(|event| event == "registered:post-startup-rejection-probe"));

        if blocks_pre_spawned_scheduler {
            wait_for_topology_condition("pre-spawned scheduler worker", || {
                control.scheduler_block_entered()
            })
            .await;
            assert!(failure.has_rollback_owner_for_tests());
            assert_product_paths(&paths, true);
            assert!(!lio_shutdown.is_shutdown_requested());
            let pending = tokio::time::timeout(
                GEOMETRY_TASK_SHUTDOWN_TIMEOUT + Duration::from_secs(1),
                crate::lifecycle_tests::first_geometry_startup_and_lio_rollback_attempt_for_tests(
                    failure, lio,
                ),
            )
            .await
            .expect("combined Geometry/LIO rollback attempt deadline")
            .expect("blocked startup rollback must retain the combined owner pair");
            assert_product_paths(&paths, true);
            assert!(!lio_shutdown.is_shutdown_requested());
            assert!(control.startup_shutdown_stages().is_empty());
            control.release_scheduler_block();
            tokio::time::timeout(Duration::from_secs(5), pending.final_join())
                .await
                .expect("same combined owner pair final-join deadline");
            assert!(lio_shutdown.is_shutdown_requested());
            assert_product_paths(&paths, false);
            assert_eq!(
                control.startup_shutdown_stages(),
                vec![
                    crate::lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                    crate::lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
                ]
            );
        } else {
            assert_eq!(
                control.startup_shutdown_stages(),
                vec![
                    crate::lio_owner::GeometryShutdownStage::GeometryTasksSettled,
                    crate::lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
                ],
                "{rejected_name} startup cleanup recorder was not connected"
            );
            let pending =
                crate::lifecycle_tests::first_geometry_startup_and_lio_rollback_attempt_for_tests(
                    failure, lio,
                )
                .await;
            assert!(
                pending.is_none(),
                "settled rejection retained an owner pair"
            );
            assert!(lio_shutdown.is_shutdown_requested());
            assert_product_paths(&paths, false);
        }
        assert_owned_consumers_unsubscribed_once(&control.events());
    }
}

#[cfg(test)]
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn geometry_runtime_blocking_revision_prepare_settles_before_resource_cleanup() {
    let mut fixture = lifecycle_fixture_with_topology_control(64, "revision-prepare", |control| {
        control.enable_revision_prepare_probe();
    })
    .await;
    let handle = fixture.handle();
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .expect("production revision Mapping")
        .lock()
        .snapshot();
    let event = MapRevisionEvent::from_map(
        &MapState {
            width: 1,
            height: 1,
            resolution: 0.25,
            frame_id: "map".to_owned(),
            revision: 1,
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            data: vec![0],
            ..MapState::default()
        },
        &mapping,
    );
    handle
        .map_revision_tx
        .send(event)
        .expect("production map-revision coalescer receiver");
    wait_for_topology_condition("real occupancy prepare boundary", || {
        fixture.topology_control().revision_prepare_entered()
    })
    .await;
    assert!(fixture
        .topology_control()
        .events()
        .iter()
        .any(|event| event == "started:map-revision-coalescer"));

    let pending = first_blocked_production_shutdown(&mut fixture, "revision-prepare").await;
    fixture.topology_control().release_revision_prepare();
    wait_for_topology_condition("occupancy preparation actually returned", || {
        fixture.topology_control().events().iter().any(|event| event == "revision-prepare-finished")
    }).await;
    fixture.assert_downstream_held_before_task_settlement();
    finish_clean_production_shutdown(pending, &fixture, "revision-prepare").await;
}

#[cfg(test)]
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn geometry_runtime_scheduler_abort_join_timeout_blocks_resource_cleanup() {
    let mut fixture = lifecycle_fixture_with_topology_control(65, "scheduler-timeout", |control| {
        control.enable_scheduler_block();
    })
    .await;
    wait_for_topology_condition("real exploration scheduler worker", || {
        fixture.topology_control().scheduler_block_entered()
    })
    .await;
    assert!(fixture
        .topology_control()
        .events()
        .iter()
        .any(|event| event == "started:exploration-scheduler"));

    let pending = first_blocked_production_shutdown(&mut fixture, "scheduler-timeout").await;
    fixture.topology_control().release_scheduler_block();
    finish_fatal_production_shutdown(
        pending,
        &fixture,
        "scheduler-timeout",
        "abort join timed out",
    )
    .await;
}

/// 几何 runtime 句柄。`server.rs` 通过此句柄读 SharedState、订阅 cloud_map_tx
/// 以及通过 transport.publish 推送 odom / point cloud(real backend 直透 liblcm)。
#[derive(Clone)]
pub struct GeometryRuntimeHandle {
    pub state: Arc<RwLock<SharedState>>,
    pub cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    /// Sole 3D mapping SHM writer shared by lidar ingestion and mapping
    /// lifecycle fencing. `None` is an allowed fail-soft runtime state.
    pub(crate) cloud_map_writer: Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    pub nav_state_tx: broadcast::Sender<NavigationUpdate>,
    pub frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    pub map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    /// Latest-only revision stream. The broadcast sender above remains the
    /// adapter ingress ABI; one coalescer owns the broadcast -> watch bridge.
    pub map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    /// Explore committed-map projection gate P. All projection pending/revoke
    /// transitions take P before the mapping lifecycle fence.
    pub(crate) explore_projection_gate: Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    /// Composition-owned authoritative committed-map identity. The mapping
    /// lifecycle fence is its sole mutation/admission gate.
    pub(crate) committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    pub transport: LcmTransport,
    /// canonical LCM backend 名（HealthCheck / 日志 / diagnostics 用），
    /// 字段值固定为 `real-lcm`；
    /// 用户请求 real-lcm 但 liblcm 不可用时由上层 `set transport_backend_name` 改写为
    /// `real-lcm-unavailable`。
    pub transport_backend_name: &'static str,
    /// backend 是否真正可用。当 `false` 时 server.rs 的 HealthCheck 必须
    /// 返回 `healthy=false` + `LCM_BACKEND_UNAVAILABLE` message；
    /// 该 flag 当前唯一触发源是 `LcmBackendKind::Real`（fail-fast 设计）。
    pub lcm_backend_available: bool,
    /// Deprecated compatibility bit exposed by the old gRPC facade. Navigation
    /// readiness no longer consumes it: only typed feedback from the private
    /// motion-guard UDS owner may mark the actuator sink ready. `/cmd_vel` is
    /// telemetry and is never an actuator ingress.
    pub cmd_vel_sink_ready: Arc<std::sync::atomic::AtomicBool>,
    /// Phase 5: 探索 runtime 句柄(由 `spawn_geometry_runtime` 注入)。
    /// 负责 start / stop / state / diagnostics 四类 RPC。
    /// 持有 `Arc<tokio::sync::Mutex<...>>` 内部,允许 `Clone` 句柄
    /// 在 gRPC 任务间安全共享。
    pub explore: crate::explore_runtime::ExploreRuntimeHandle,
    /// Phase 4: 导航 runtime 状态(由 `spawn_geometry_runtime` / 单元测试
    /// 构造时初始化)。`server.rs` 通过它调 `set_goal` / `cancel` /
    /// `compute_readiness` 真实执行链。内部仅读 `state.pose.timestamp_ns`
    /// 与 `state.map.revision`,不持有 LCM 订阅线程。
    pub nav: crate::nav_runtime::NavRuntimeState,
    /// Phase 3: mapping adapter 句柄。
    /// - `None` —— backend 不可用 / LCM fail-fast 路径不构造 adapter
    /// - `Some` —— adapter 已注入,`mapping_ready_poll` 通过该字段判定
    ///   类型为 `Arc<parking_lot::Mutex<MappingAdapter>>`:
    ///   - `Arc` 允许多订阅任务共享
    ///   - `parking_lot::Mutex` 走同步路径,避免 async 持锁跨 await
    pub mapping_adapter: Option<Arc<parking_lot::Mutex<MappingAdapter>>>,
    /// Serializes StartMapping/StopMapping/GetMappingState control-plane
    /// transitions and downstream reset boundaries.
    pub mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    /// Phase 3: occupancy grid SHM sidecar 写入器。
    ///
    /// - `None` —— backend 不可用 / LCM fail-fast 路径不构造
    /// - `Some` —— `get_occupancy_grid_shm` 走 SHM 直读
    pub occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
    /// 与 `occupancy_shm` 同时创建的权威路径；gRPC 返回的 `ShmRef.key`
    /// 必须使用该值，不能在请求时重新读取可能已变化的环境变量。
    pub occupancy_shm_path: Option<String>,
    /// Reference to the latest revision successfully committed by the sole
    /// occupancy SHM writer task.
    pub occupancy_shm_current: Arc<RwLock<Option<OccupancyShmSnapshot>>>,
    /// Exactly one background exploration scheduler belongs to the process.
    pub explore_scheduler:
        Arc<tokio::sync::Mutex<Option<crate::explore_runtime::ExploreSchedulerHandle>>>,
    /// Monotonic generation assigned to typed runtime observations received by
    /// the gRPC facade (for example guard-backed motion-sink health).
    pub runtime_observation_generation: Arc<std::sync::atomic::AtomicU64>,
    /// Production-only cloneable admission token. Test facades intentionally
    /// leave this unset so they do not require a root supervisor.
    pub(crate) task_control: Option<GeometryRuntimeTaskControl>,
}

impl GeometryRuntimeHandle {
    pub(crate) fn task_control(&self) -> Option<GeometryRuntimeTaskControl> {
        self.task_control.clone()
    }

    fn revoke_committed_map_projection_parts_locked(
        committed_map_authority: &Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
        map_revision_latest_tx: &watch::Sender<Option<MapRevisionEvent>>,
    ) -> Result<(), String> {
        let mut authority = committed_map_authority
            .lock()
            .map_err(|_| "committed-map authority state is unavailable".to_string())?;
        *authority = None;
        map_revision_latest_tx.send_replace(None);
        Ok(())
    }

    /// 调用方必须按 P -> mapping_lifecycle_lock 持有两个 owner fence。
    /// 同步撤销组合层 pending、authority 与 latest watch，不接管 backend retirement。
    pub(crate) async fn lock_explore_projection_owned(&self) -> ExploreProjectionOwnedGuard {
        self.explore_projection_gate.clone().lock_owned().await
    }

    pub(crate) fn revoke_committed_map_projection_locked(
        &self,
        projection: &mut ExploreProjectionOwnedGuard,
    ) -> Result<(), String> {
        projection.pending_latest = None;
        Self::revoke_committed_map_projection_parts_locked(
            &self.committed_map_authority,
            &self.map_revision_latest_tx,
        )
    }

    pub fn subscribe_map_revisions(&self) -> watch::Receiver<Option<MapRevisionEvent>> {
        self.map_revision_latest_tx.subscribe()
    }

    /// 复用导航运行时已捕获的模式与生产者代际，不重新读取环境配置。
    pub(crate) fn navigation_map_3d_publish_context(
        &self,
    ) -> Option<NavigationMap3DPublishContext> {
        Some(NavigationMap3DPublishContext {
            enabled: self.nav.configured_profile() == crate::nav_runtime::PROFILE_MLS3D,
            producer_runtime_generation: self.nav.producer_runtime_generation(),
            mapping: self.mapping_adapter.as_ref()?.clone(),
            committed_map_authority: self.committed_map_authority.clone(),
        })
    }

    /// Reserve cloud-map clear before any worker can touch SHM or lifecycle
    /// state. Rejected reservations own and immediately drop their unstarted
    /// future, so Phase A closure cannot fall through to detached work.
    pub(crate) fn reserve_cloud_map_session(
        &self,
        clear_canonical_mapping_view: bool,
    ) -> CloudMapSessionRegistration {
        let transport = self.transport.clone();
        let writer = self.cloud_map_writer.clone();
        let state = self.state.clone();
        let occupancy_shm_current = self.occupancy_shm_current.clone();
        let cloud_map_tx = self.cloud_map_tx.clone();
        let mapping_adapter = self.mapping_adapter.clone();
        let lifecycle_lock = self.mapping_lifecycle_lock.clone();
        let navigation_runtime = self.nav.clone();
        let committed_map_authority = self.committed_map_authority.clone();
        let (activation_tx, activation_rx) = tokio::sync::oneshot::channel();
        let (update_tx, update_rx) = tokio::sync::oneshot::channel();
        let task = async move {
            let session_generation = match activation_rx.await {
                Ok(session_generation) => session_generation,
                Err(_) => return Ok(()),
            };
            let update = {
                let mut guard = writer.lock();
                let Some(writer) = guard.as_mut() else {
                    let _ = update_tx.send(CloudMapSessionAdmission::WriterUnavailable);
                    return Ok(());
                };
                let Some((revision, point_count, voxel_size)) =
                    writer.begin_session(session_generation)
                else {
                    let _ = update_tx.send(CloudMapSessionAdmission::ClearRejected(format!(
                        "cloud-map writer rejected Mapping generation {session_generation}"
                    )));
                    return Ok(());
                };
                CloudMapNotification {
                    revision,
                    point_count,
                    voxel_size,
                    session_generation,
                }
            };
            {
                let mut shared = state.write().await;
                if shared.cloud_map_revision < update.revision {
                    shared.cloud_map_revision = update.revision;
                }
            }
            let _ = cloud_map_tx.send(update.clone());

            if clear_canonical_mapping_view {
                // StartMapping retires the prior canonical occupancy/map view only
                // after begin_session has exposed the new cloud generation.
                {
                    let mut shared = state.write().await;
                    shared.map = MapState::default();
                }
                *occupancy_shm_current.write().await = None;
            }
            let _ = update_tx.send(CloudMapSessionAdmission::Registered(update));

            // Direct checkpoint callers already hold this fence. The deferred
            // clear path acquires the same fence before canonical publication.
            let navigation_enabled =
                navigation_runtime.configured_profile() == crate::nav_runtime::PROFILE_MLS3D;
            let navigation_context =
                mapping_adapter
                    .as_ref()
                    .map(|mapping| NavigationMap3DPublishContext {
                        enabled: navigation_enabled,
                        producer_runtime_generation: navigation_runtime
                            .producer_runtime_generation(),
                        mapping: mapping.clone(),
                        committed_map_authority,
                    });
            let _lifecycle_guard = lifecycle_lock.lock().await;
            let frame_id = mapping_adapter
                .as_ref()
                .map(|adapter| adapter.lock().configured_map_frame_id().to_owned())
                .unwrap_or_else(|| "map".to_owned());
            publish_latest_canonical_cloud_map_with_navigation(
                &transport,
                &writer,
                &state,
                &cloud_map_tx,
                &frame_id,
                navigation_context.as_ref(),
            )
            .await
            .map(|_| ())
            .map_err(|error| error.to_string())
        };

        if let Some(control) = &self.task_control {
            let registration = GeometryRuntimeTaskControl::registration(
                "cloud-map-clear",
                GeometryTaskClass::Retained,
                task,
            );
            if let Err(rejected) = control.try_register(registration) {
                let detail = rejected.detail();
                drop(rejected);
                return CloudMapSessionRegistration::Rejected(
                    CloudMapSessionAdmission::AdmissionClosed(detail),
                );
            }
            CloudMapSessionRegistration::Admitted(CloudMapSessionReservation {
                activation_tx: Some(activation_tx),
                update_rx,
            })
        } else {
            tokio::spawn(task);
            CloudMapSessionRegistration::Admitted(CloudMapSessionReservation {
                activation_tx: Some(activation_tx),
                update_rx,
            })
        }
    }

    /// Reserve, activate, and await one cloud-map clear session.
    pub(crate) async fn begin_cloud_map_session(
        &self,
        session_generation: u64,
    ) -> CloudMapSessionAdmission {
        match self.reserve_cloud_map_session(false) {
            CloudMapSessionRegistration::Admitted(reservation) => {
                reservation.activate(session_generation).await
            }
            CloudMapSessionRegistration::Rejected(admission) => admission,
        }
    }

    /// Read the deprecated compatibility bit. This value is diagnostic only.
    pub fn cmd_vel_sink_ready(&self) -> bool {
        self.cmd_vel_sink_ready
            .load(std::sync::atomic::Ordering::Acquire)
    }

    /// Preserve the legacy setter ABI without promoting it to typed readiness.
    pub fn set_cmd_vel_sink_ready(&self, ready: bool) {
        self.cmd_vel_sink_ready
            .store(ready, std::sync::atomic::Ordering::Release);
    }

    pub fn next_runtime_observation_generation(&self) -> u64 {
        self.runtime_observation_generation
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel)
            .saturating_add(1)
    }

    pub async fn shutdown_explore_scheduler(&self) -> Result<(), String> {
        if let Some(scheduler) = self.explore_scheduler.lock().await.take() {
            scheduler.shutdown().await
        } else {
            Ok(())
        }
    }

    /// mapping 板块 readiness。
    /// - backend 不可用 → false
    /// - adapter 未构造 → false
    /// - adapter 已构造 + occupancy revision + global frame/anchor ready → true
    /// - 缺点云、revision 或 frame contract 任一项 → false
    pub fn mapping_ready_poll(&self) -> bool {
        match &self.mapping_adapter {
            Some(adapter) => {
                crate::mapping_adapter::MappingReadiness::compute(
                    Some(&*adapter.lock()),
                    self.lcm_backend_available,
                )
                .ready
            }
            None => {
                crate::mapping_adapter::MappingReadiness::compute(None, self.lcm_backend_available)
                    .ready
            }
        }
    }

    /// navigation 板块 readiness：必须有可用 backend，且最近收到真实
    /// stack 的 `/nav/state` heartbeat/state。
    pub fn nav_ready_poll(&self) -> bool {
        self.lcm_backend_available && self.nav.stack_is_live()
    }

    /// exploration 板块 readiness。
    /// Phase 7:从 `ExploreRuntimeHandle::readiness_cache` 取真实聚合
    /// (`backend_available && controller.is_active() &&
    /// frontier_source_ready && costmap_input_ready`),不再依赖
    /// "handle 存在即 true" 的假 ready。
    ///
    /// 该方法在 sync context(diagnostics 聚合路径)被调;走缓存快照避免
    /// await 内层 `tokio::Mutex`,cache 由 `ingest_costmap / start / stop
    /// / tick` 在变更状态后刷新,以及 `refresh_readiness_cache_with_backend`
    /// 在 backend 切换时刷新。
    pub fn explore_ready_poll(&self) -> bool {
        if !self.lcm_backend_available {
            return false;
        }
        // backend_available = lcm_backend_available,先用 sync 版本尝试刷新 cache;
        // 抢不到锁则退到 cached_readiness(降级,不假装 ready)。
        self.explore
            .refresh_readiness_cache_with_backend(self.lcm_backend_available);
        self.explore.cached_readiness().ready
    }
}

// ============================================================================
// 内部：tf2 坐标链合成（map→odom→base_link）。算法与原 ros2_bridge.rs 一致，
// 保证 server.rs 语义不因 backend 切换而漂移。
// ============================================================================

type Mat4 = [[f64; 4]; 4];

fn identity4() -> Mat4 {
    let mut m = [[0.0f64; 4]; 4];
    m[0][0] = 1.0;
    m[1][1] = 1.0;
    m[2][2] = 1.0;
    m[3][3] = 1.0;
    m
}

fn quat_to_rot(x: f64, y: f64, z: f64, w: f64) -> [[f64; 3]; 3] {
    let norm = (x * x + y * y + z * z + w * w).sqrt();
    if norm <= 1e-12 {
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    }
    let (x, y, z, w) = (x / norm, y / norm, z / norm, w / norm);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn mat_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut o = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[i][k] * b[k][j];
            }
            o[i][j] = s;
        }
    }
    o
}

fn entry_to_transform(entry: &TransformEntry) -> Option<TransformStamped> {
    let parent = normalize_frame(&entry.parent);
    let child = normalize_frame(&entry.child);
    let values = [
        entry.tx, entry.ty, entry.tz, entry.rx, entry.ry, entry.rz, entry.rw,
    ];
    let quaternion_norm_sq =
        entry.rx * entry.rx + entry.ry * entry.ry + entry.rz * entry.rz + entry.rw * entry.rw;
    if parent.is_empty()
        || child.is_empty()
        || !values.into_iter().all(f64::is_finite)
        || quaternion_norm_sq <= f64::EPSILON
    {
        return None;
    }
    let mut transform = make_transform(
        parent,
        child,
        [entry.tx, entry.ty, entry.tz],
        [entry.rx, entry.ry, entry.rz, entry.rw],
    );
    transform.stamp_secs = i64::try_from(entry.stamp_ns / 1_000_000_000).ok()?;
    transform.stamp_nanos = u32::try_from(entry.stamp_ns % 1_000_000_000).ok()?;
    Some(transform)
}

fn transform_to_entry(transform: TransformStamped) -> Option<TransformEntry> {
    let stamp_secs = u64::try_from(transform.stamp_secs).ok()?;
    let stamp_ns = stamp_secs
        .checked_mul(1_000_000_000)?
        .checked_add(u64::from(transform.stamp_nanos))?;
    let translation = transform.transform.translation.vector;
    let quaternion = transform.transform.rotation.quaternion();
    Some(TransformEntry {
        parent: transform.parent,
        child: transform.child,
        tx: translation.x,
        ty: translation.y,
        tz: translation.z,
        rx: quaternion.i,
        ry: quaternion.j,
        rz: quaternion.k,
        rw: quaternion.w,
        stamp_ns,
    })
}

fn entry_to_matrix(e: &TransformEntry) -> Mat4 {
    let r = quat_to_rot(e.rx, e.ry, e.rz, e.rw);
    let mut m = identity4();
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] = r[i][j];
        }
    }
    m[0][3] = e.tx;
    m[1][3] = e.ty;
    m[2][3] = e.tz;
    m
}

fn rigid_inverse(m: &Mat4) -> Mat4 {
    let mut o = identity4();
    for (i, row) in o.iter_mut().enumerate().take(3) {
        for (j, col) in row.iter_mut().enumerate().take(3) {
            *col = m[j][i];
        }
    }
    for i in 0..3 {
        o[i][3] = -(o[i][0] * m[0][3] + o[i][1] * m[1][3] + o[i][2] * m[2][3]);
    }
    o
}

fn normalize_frame(f: &str) -> String {
    f.trim().trim_start_matches('/').to_string()
}

const BASE_FRAME_ALIASES: &[&str] = &[
    "base",
    "base_link",
    "base_footprint",
    "body",
    "go2_base_link",
];

const MAP_FRAME_ENV: &str = "NUWAX_GEOMETRY_MAP_FRAME";
const MAP_FRAME_LEGACY_ENV: &str = "NUWAX_MAP_FRAME";
const ODOM_FRAME_ENV: &str = "NUWAX_GEOMETRY_ODOM_FRAME";
const ODOM_FRAME_LEGACY_ENV: &str = "NUWAX_ODOM_FRAME";
const BASE_FRAME_ENV: &str = "NUWAX_BASE_FRAME";
const BASE_FRAME_LEGACY_ENV: &str = "NUWAX_GEOMETRY_BASE_FRAME";

#[derive(Clone, Debug, Error, PartialEq, Eq)]
#[error("FRAME_BINDING_INVALID role={role} env={variable} frame={value:?}")]
pub(crate) struct FrameBindingError {
    pub(crate) role: &'static str,
    pub(crate) variable: &'static str,
    pub(crate) value: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ValidatedFrameBindings {
    map_frame: String,
    odom_frame: String,
    base_frame: String,
}

impl ValidatedFrameBindings {
    pub(crate) fn from_env() -> Result<Self, FrameBindingError> {
        Ok(Self {
            map_frame: read_frame_binding("map", MAP_FRAME_ENV, MAP_FRAME_LEGACY_ENV, "map")?,
            odom_frame: read_frame_binding("odom", ODOM_FRAME_ENV, ODOM_FRAME_LEGACY_ENV, "odom")?,
            base_frame: read_frame_binding(
                "base",
                BASE_FRAME_ENV,
                BASE_FRAME_LEGACY_ENV,
                "base_link",
            )?,
        })
    }

    fn from_values(
        map_frame: impl Into<String>,
        odom_frame: impl Into<String>,
        base_frame: impl Into<String>,
    ) -> Result<Self, FrameBindingError> {
        Ok(Self {
            map_frame: validate_frame_binding("map", "<value>", map_frame.into())?,
            odom_frame: validate_frame_binding("odom", "<value>", odom_frame.into())?,
            base_frame: validate_frame_binding("base", "<value>", base_frame.into())?,
        })
    }

    fn into_parts(self) -> (String, String, String) {
        (self.map_frame, self.odom_frame, self.base_frame)
    }

    pub(crate) fn as_parts(&self) -> (&str, &str, &str) {
        (&self.map_frame, &self.odom_frame, &self.base_frame)
    }
}

fn read_frame_binding(
    role: &'static str,
    preferred: &'static str,
    legacy: &'static str,
    default: &'static str,
) -> Result<String, FrameBindingError> {
    for variable in [preferred, legacy] {
        match std::env::var(variable) {
            Ok(value) => return validate_frame_binding(role, variable, value),
            Err(std::env::VarError::NotPresent) => {}
            Err(std::env::VarError::NotUnicode(_)) => {
                return Err(FrameBindingError {
                    role,
                    variable,
                    value: "<non-unicode>".to_string(),
                });
            }
        }
    }
    validate_frame_binding(role, "<default>", default.to_string())
}

fn validate_frame_binding(
    role: &'static str,
    variable: &'static str,
    value: String,
) -> Result<String, FrameBindingError> {
    let value = value.trim().to_string();
    if validated_frame_tokens(&value).is_some() {
        Ok(value)
    } else {
        Err(FrameBindingError {
            role,
            variable,
            value,
        })
    }
}

fn validated_frame_tokens(frame_id: &str) -> Option<Vec<&str>> {
    let frame_id = frame_id.trim();
    if frame_id.is_empty() {
        return None;
    }
    let tokens: Vec<_> = frame_id.split('/').collect();
    tokens
        .iter()
        .all(|token| {
            let mut chars = token.chars();
            chars
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && chars.all(|character| character.is_ascii_alphanumeric() || character == '_')
        })
        .then_some(tokens)
}

fn is_base_frame_leaf(leaf: &str) -> bool {
    BASE_FRAME_ALIASES.contains(&leaf)
        || leaf.ends_with("_base_link")
        || leaf.ends_with("_base_footprint")
}

fn base_frame_ids_semantically_equal(left: &str, right: &str) -> bool {
    let (Some(left_tokens), Some(right_tokens)) =
        (validated_frame_tokens(left), validated_frame_tokens(right))
    else {
        return false;
    };
    if left_tokens == right_tokens {
        return true;
    }
    let (Some(left_leaf), Some(right_leaf)) = (left_tokens.last(), right_tokens.last()) else {
        return false;
    };
    if !is_base_frame_leaf(left_leaf) || !is_base_frame_leaf(right_leaf) {
        return false;
    }
    if left_tokens.len() > 1
        && right_tokens.len() > 1
        && left_tokens[..left_tokens.len() - 1] != right_tokens[..right_tokens.len() - 1]
    {
        return false;
    }
    true
}

fn resolve_base_frame_alias(preferred: Option<&str>, legacy: Option<&str>) -> String {
    preferred
        .and_then(|value| (!value.trim().is_empty()).then(|| value.trim().to_string()))
        .or_else(|| {
            legacy.and_then(|value| (!value.trim().is_empty()).then(|| value.trim().to_string()))
        })
        .unwrap_or_else(|| "base_link".to_string())
}

fn semantic_base_candidates(transforms: &[TransformEntry], configured: &str) -> Vec<String> {
    let mut candidates = vec![configured.to_string()];
    for entry in transforms {
        for frame in [&entry.parent, &entry.child] {
            if base_frame_ids_semantically_equal(frame, configured) {
                let frame = frame.trim().to_string();
                if !candidates.contains(&frame) {
                    candidates.push(frame);
                }
            }
        }
    }
    candidates
}

fn resolve_transform(transforms: &[TransformEntry], source: &str, target: &str) -> Option<Mat4> {
    let source = normalize_frame(source);
    let target = normalize_frame(target);
    if source.is_empty() || target.is_empty() {
        return None;
    }
    if source == target {
        return Some(identity4());
    }

    let mut adjacency: HashMap<String, Vec<(String, Mat4)>> = HashMap::new();
    for e in transforms {
        let parent = normalize_frame(&e.parent);
        let child = normalize_frame(&e.child);
        let m = entry_to_matrix(e); // child→parent
        adjacency
            .entry(child.clone())
            .or_default()
            .push((parent.clone(), m));
        adjacency
            .entry(parent)
            .or_default()
            .push((child, rigid_inverse(&m)));
    }

    let mut visited: HashSet<String> = HashSet::new();
    visited.insert(source.clone());
    let mut queue: VecDeque<(String, Mat4)> = VecDeque::new();
    queue.push_back((source, identity4()));
    while let Some((frame, mat)) = queue.pop_front() {
        if frame == target {
            return Some(mat);
        }
        if let Some(edges) = adjacency.get(&frame) {
            for (next, next_mat) in edges {
                if visited.contains(next) {
                    continue;
                }
                visited.insert(next.clone());
                queue.push_back((next.clone(), mat_mul(next_mat, &mat)));
            }
        }
    }
    None
}

fn mat_to_quat(m: &Mat4) -> (f64, f64, f64, f64) {
    let trace = m[0][0] + m[1][1] + m[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        )
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        (
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        )
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        (
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        )
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        (
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[0][1] - m[1][0]) / s,
        )
    }
}

/// (x, y, z, qx, qy, qz, qw, normalized_base_frame)
type PoseWithFrame = (f64, f64, f64, f64, f64, f64, f64, String);

fn resolve_map_pose(
    transforms: &[TransformEntry],
    map_frame: &str,
    base_frames: &[&str],
) -> Option<PoseWithFrame> {
    for base in base_frames {
        if let Some(m) = resolve_transform(transforms, base, map_frame) {
            let (qx, qy, qz, qw) = mat_to_quat(&m);
            return Some((
                m[0][3],
                m[1][3],
                m[2][3],
                qx,
                qy,
                qz,
                qw,
                normalize_frame(base),
            ));
        }
    }
    None
}

fn resolve_map_pose_tree(
    tree: &TfTree,
    map_frame: &str,
    base_frames: &[&str],
) -> Option<PoseWithFrame> {
    let map_frame = normalize_frame(map_frame);
    for base in base_frames {
        let base = normalize_frame(base);
        let Ok(transform) = tree.chain(&base, &map_frame) else {
            continue;
        };
        let translation = transform.translation.vector;
        let quaternion = transform.rotation.quaternion();
        return Some((
            translation.x,
            translation.y,
            translation.z,
            quaternion.i,
            quaternion.j,
            quaternion.k,
            quaternion.w,
            base,
        ));
    }
    None
}

// ============================================================================
// LCM payload 解析辅助（generated-schema 解码）
// ============================================================================
//
// `nuwax-transport` 已验证并剥离 generated fingerprint，所以这些函数接收
// `_encode_one()` body。为复用 `nuwax-codec` 的 top-level generated decoder，
// 仅在进程内重新拼回对应 fingerprint；不再维护第二套 little-endian layout。

fn decode_generated<T>(
    fingerprint: [u8; 8],
    body: &[u8],
    decode: impl FnOnce(&[u8]) -> Result<T, nuwax_codec::CodecError>,
) -> Option<T> {
    let mut encoded = Vec::with_capacity(fingerprint.len() + body.len());
    encoded.extend_from_slice(&fingerprint);
    encoded.extend_from_slice(body);
    decode(&encoded).ok()
}

fn header_stamp_ns(header: &schema::Header) -> Option<u64> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return None;
    }
    (header.stamp_sec as u64)
        .checked_mul(1_000_000_000)?
        .checked_add(header.stamp_nsec as u64)
}

fn parse_odom_payload(payload: &[u8]) -> Option<PoseState> {
    crate::typed_codec::decode_odometry_payload(payload)
}

fn parse_pose_stamped_payload(payload: &[u8]) -> Option<PoseState> {
    let message = decode_generated(
        schema::POSE_STAMPED_FINGERPRINT,
        payload,
        schema::decode_pose_stamped,
    )?;
    let timestamp_ns = header_stamp_ns(&message.header)?;
    let frame_id = normalize_frame(&message.header.frame_id);
    if frame_id.is_empty() {
        return None;
    }
    Some(PoseState {
        x: message.pose.position[0],
        y: message.pose.position[1],
        z: message.pose.position[2],
        qx: message.pose.orientation_xyzw[0],
        qy: message.pose.orientation_xyzw[1],
        qz: message.pose.orientation_xyzw[2],
        qw: message.pose.orientation_xyzw[3],
        frame_id,
        timestamp_ns,
    })
}

struct CanonicalMappingObservation {
    map_pose: PoseState,
    transforms: [TransformEntry; 2],
    tf_payload: Vec<u8>,
}

fn normalized_quaternion_and_yaw(pose: &PoseState) -> Result<([f64; 4], f64), String> {
    let values = [pose.qx, pose.qy, pose.qz, pose.qw];
    if !values.into_iter().all(f64::is_finite) {
        return Err("odometry quaternion contains a non-finite component".to_string());
    }
    let norm_sq = pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw;
    if norm_sq <= f64::EPSILON {
        return Err("odometry quaternion has zero norm".to_string());
    }
    let inverse_norm = norm_sq.sqrt().recip();
    let quaternion = [
        pose.qx * inverse_norm,
        pose.qy * inverse_norm,
        pose.qz * inverse_norm,
        pose.qw * inverse_norm,
    ];
    let [x, y, z, w] = quaternion;
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    if !yaw.is_finite() {
        return Err("odometry quaternion produced a non-finite yaw".to_string());
    }
    Ok((quaternion, yaw))
}

fn yaw_quaternion(yaw_radians: f64) -> [f64; 4] {
    let half_yaw = yaw_radians * 0.5;
    [0.0, 0.0, half_yaw.sin(), half_yaw.cos()]
}

fn canonical_mapping_observation(
    transform: &crate::mapping_adapter::MapOdomTransformSnapshot,
    odom_pose: &PoseState,
    base_frame: &str,
) -> Result<CanonicalMappingObservation, String> {
    let map_from_odom = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(
            transform.translation[0],
            transform.translation[1],
            transform.translation[2],
        ),
        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, transform.yaw_radians),
    );
    canonical_mapping_observation_from_isometry(
        &transform.map_frame_id,
        &transform.odom_frame_id,
        transform.generation,
        &map_from_odom,
        odom_pose,
        base_frame,
    )
}

fn canonical_mapping_observation_from_isometry(
    map_frame_id: &str,
    odom_frame_id: &str,
    transform_generation: u64,
    map_from_odom: &nalgebra::Isometry3<f64>,
    odom_pose: &PoseState,
    base_frame: &str,
) -> Result<CanonicalMappingObservation, String> {
    let map_frame = normalize_frame(map_frame_id);
    let odom_frame = normalize_frame(odom_frame_id);
    let source_frame = normalize_frame(&odom_pose.frame_id);
    let base_frame = normalize_frame(base_frame);
    if map_frame.is_empty() || odom_frame.is_empty() || base_frame.is_empty() {
        return Err("canonical mapping frames must be non-empty".to_string());
    }
    if source_frame != odom_frame {
        return Err(format!(
            "odometry frame {} does not match configured frame {}",
            source_frame, odom_frame
        ));
    }
    let (odom_quaternion, _) = normalized_quaternion_and_yaw(odom_pose)?;
    let map_translation = map_from_odom.translation.vector;
    let map_rotation = map_from_odom.rotation.quaternion();
    if map_translation.iter().any(|value| !value.is_finite())
        || [
            map_rotation.i,
            map_rotation.j,
            map_rotation.k,
            map_rotation.w,
        ]
        .into_iter()
        .any(|value| !value.is_finite())
    {
        return Err("canonical map<-odom transform contains a non-finite value".to_owned());
    }
    let odom_from_body = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(odom_pose.x, odom_pose.y, odom_pose.z),
        nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(
            odom_quaternion[3],
            odom_quaternion[0],
            odom_quaternion[1],
            odom_quaternion[2],
        )),
    );
    let map_from_body = map_from_odom * odom_from_body;
    let map_body_rotation = map_from_body.rotation.quaternion();
    let map_odom_quaternion = [
        map_rotation.i,
        map_rotation.j,
        map_rotation.k,
        map_rotation.w,
    ];
    let map_pose_quaternion = [
        map_body_rotation.i,
        map_body_rotation.j,
        map_body_rotation.k,
        map_body_rotation.w,
    ];
    let stamp_sec = i32::try_from(odom_pose.timestamp_ns / 1_000_000_000)
        .map_err(|_| "odometry timestamp seconds exceed int32".to_string())?;
    let stamp_nsec = (odom_pose.timestamp_ns % 1_000_000_000) as i32;
    let map_pose_position = map_from_body.translation.vector;
    let transforms = [
        TransformEntry {
            parent: map_frame.clone(),
            child: odom_frame.clone(),
            tx: map_translation[0],
            ty: map_translation[1],
            tz: map_translation[2],
            rx: map_odom_quaternion[0],
            ry: map_odom_quaternion[1],
            rz: map_odom_quaternion[2],
            rw: map_odom_quaternion[3],
            stamp_ns: odom_pose.timestamp_ns,
        },
        TransformEntry {
            parent: odom_frame.clone(),
            child: base_frame.clone(),
            tx: odom_pose.x,
            ty: odom_pose.y,
            tz: odom_pose.z,
            rx: odom_quaternion[0],
            ry: odom_quaternion[1],
            rz: odom_quaternion[2],
            rw: odom_quaternion[3],
            stamp_ns: odom_pose.timestamp_ns,
        },
    ];
    let header = |frame_id: String| schema::Header {
        seq: i32::try_from(transform_generation).unwrap_or(i32::MAX),
        stamp_sec,
        stamp_nsec,
        frame_id,
    };
    let tf_payload = schema::encode_tf_message_body(&schema::TfMessage {
        transforms: vec![
            schema::TransformStamped {
                header: header(map_frame.clone()),
                child_frame_id: odom_frame,
                transform: schema::Transform {
                    translation: [map_translation[0], map_translation[1], map_translation[2]],
                    rotation_xyzw: map_odom_quaternion,
                },
            },
            schema::TransformStamped {
                header: header(source_frame),
                child_frame_id: base_frame,
                transform: schema::Transform {
                    translation: [odom_pose.x, odom_pose.y, odom_pose.z],
                    rotation_xyzw: odom_quaternion,
                },
            },
        ],
    })
    .map_err(|error| format!("canonical TF encode failed: {error}"))?;
    Ok(CanonicalMappingObservation {
        map_pose: PoseState {
            x: map_pose_position.x,
            y: map_pose_position.y,
            z: map_pose_position.z,
            qx: map_pose_quaternion[0],
            qy: map_pose_quaternion[1],
            qz: map_pose_quaternion[2],
            qw: map_pose_quaternion[3],
            frame_id: map_frame,
            timestamp_ns: odom_pose.timestamp_ns,
        },
        transforms,
        tf_payload,
    })
}

fn canonical_mapping_observation_for_odom(
    mapping: &parking_lot::Mutex<MappingAdapter>,
    transform_cache: &parking_lot::RwLock<Option<crate::mapping_adapter::MapOdomTransformSnapshot>>,
    odom_pose: &PoseState,
    base_frame: &str,
) -> Result<CanonicalMappingObservation, String> {
    let (_, yaw_radians) = normalized_quaternion_and_yaw(odom_pose)?;
    let transform = if let Some(mut adapter) = mapping.try_lock() {
        adapter
            .feed_pose_in_frame(
                odom_pose.x,
                odom_pose.y,
                odom_pose.z,
                yaw_radians,
                &odom_pose.frame_id,
            )
            .map_err(|error| error.to_string())?;
        adapter
            .map_odom_transform_snapshot()
            .ok_or_else(|| "odom pose did not establish T_map_odom".to_string())?
    } else {
        transform_cache
            .read()
            .clone()
            .ok_or_else(|| "mapping adapter busy before T_map_odom was established".to_string())?
    };
    canonical_mapping_observation(&transform, odom_pose, base_frame)
}

async fn apply_canonical_mapping_observation(
    state: &Arc<RwLock<SharedState>>,
    observation: &CanonicalMappingObservation,
) {
    let mut state = state.write().await;
    apply_canonical_mapping_observation_locked(&mut state, observation);
}

fn apply_canonical_mapping_observation_locked(
    state: &mut SharedState,
    observation: &CanonicalMappingObservation,
) {
    state.pose = observation.map_pose.clone();
    for entry in &observation.transforms {
        if let Some(transform) = entry_to_transform(entry) {
            state.tf_tree.set(transform);
        }
        if let Some(existing) = state.transforms.iter_mut().find(|existing| {
            normalize_frame(&existing.parent) == entry.parent
                && normalize_frame(&existing.child) == entry.child
        }) {
            *existing = entry.clone();
        } else {
            state.transforms.push(entry.clone());
        }
    }
    state.pose_changed.send_replace(());
}

#[derive(Clone, Debug)]
pub(crate) struct CanonicalCorrectionSnapshot {
    pub(crate) runtime_generation: u64,
    pub(crate) backend_generation: u64,
    pub(crate) backend: crate::correction_coordinator::CorrectionCandidateBackend,
    pub(crate) mapping_session_id: String,
    pub(crate) mapping_session_generation: u64,
    pub(crate) map_frame_generation: u64,
    pub(crate) mapping_commit_id: crate::mapping_commit::MappingCommitId,
    pub(crate) occupancy_revision: u64,
    pub(crate) cloud_revision: u64,
    pub(crate) correction_revision: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) committed_map_source_timestamp_ns: u64,
    pub(crate) map_frame_id: String,
    pub(crate) odom_frame_id: String,
    pub(crate) body_frame_id: String,
    pub(crate) correction_lease: crate::correction_coordinator::CorrectionLeaseId,
    pub(crate) effective_map_from_odom: nalgebra::Isometry3<f64>,
}

pub(crate) struct GeometryCanonicalCorrectionApplier {
    expected_identity: crate::correction_coordinator::CorrectionOwnerRuntimeIdentity,
    map_frame_id: String,
    odom_frame_id: String,
    body_frame_id: String,
    state: Arc<RwLock<SharedState>>,
    transport: LcmTransport,
    tf_binding: (LcmTopic, LcmMessageType),
    current: Arc<parking_lot::RwLock<Option<CanonicalCorrectionSnapshot>>>,
}

impl GeometryCanonicalCorrectionApplier {
    pub(crate) fn new(
        expected_identity: crate::correction_coordinator::CorrectionOwnerRuntimeIdentity,
        map_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
        body_frame_id: impl Into<String>,
        state: Arc<RwLock<SharedState>>,
        transport: LcmTransport,
        tf_binding: (LcmTopic, LcmMessageType),
    ) -> Result<Self, String> {
        let map_frame_id = normalize_frame(&map_frame_id.into());
        let odom_frame_id = normalize_frame(&odom_frame_id.into());
        let body_frame_id = normalize_frame(&body_frame_id.into());
        if map_frame_id.is_empty()
            || odom_frame_id.is_empty()
            || body_frame_id.is_empty()
            || map_frame_id == odom_frame_id
            || map_frame_id == body_frame_id
            || odom_frame_id == body_frame_id
            || expected_identity.runtime_generation == 0
            || expected_identity.backend_generation == 0
            || expected_identity.correction_lease.boot_nonce == 0
            || expected_identity.correction_lease.generation == 0
        {
            return Err("canonical correction applier identity is incomplete".to_owned());
        }
        Ok(Self {
            expected_identity,
            map_frame_id,
            odom_frame_id,
            body_frame_id,
            state,
            transport,
            tf_binding,
            current: Arc::new(parking_lot::RwLock::new(None)),
        })
    }

    pub(crate) fn snapshot(&self) -> Option<CanonicalCorrectionSnapshot> {
        self.current.read().clone()
    }

    fn validate_transaction(
        &self,
        transaction: &crate::correction_coordinator::CanonicalCorrectionTransaction,
    ) -> Result<(), String> {
        if transaction.runtime_generation != self.expected_identity.runtime_generation
            || transaction.backend_generation != self.expected_identity.backend_generation
            || transaction.correction_lease != self.expected_identity.correction_lease
            || normalize_frame(&transaction.map_frame_id) != self.map_frame_id
            || normalize_frame(&transaction.odom_frame_id) != self.odom_frame_id
            || normalize_frame(&transaction.body_frame_id) != self.body_frame_id
        {
            return Err("canonical correction transaction identity mismatch".to_owned());
        }
        if transaction.candidate_sequence == 0
            || transaction.payload_sha256.iter().all(|byte| *byte == 0)
            || transaction.correction_revision == 0
            || transaction.mapping_commit_id.boot_nonce == 0
            || transaction.mapping_commit_id.sequence == 0
            || transaction.mapping_session_id.is_empty()
            || transaction.mapping_session_id.trim() != transaction.mapping_session_id
            || transaction.mapping_session_generation == 0
            || transaction.map_frame_generation == 0
            || transaction.occupancy_revision == 0
            || transaction.cloud_revision == 0
            || transaction.frozen_odom_source_timestamp_ns > transaction.source_timestamp_ns
            || match transaction.backend {
                crate::correction_coordinator::CorrectionCandidateBackend::Relocalization => {
                    transaction.committed_map_source_timestamp_ns > transaction.source_timestamp_ns
                }
                crate::correction_coordinator::CorrectionCandidateBackend::LibraryPgo
                | crate::correction_coordinator::CorrectionCandidateBackend::NativePgo => {
                    transaction.committed_map_source_timestamp_ns != transaction.source_timestamp_ns
                }
            }
            || !canonical_isometry_is_finite(&transaction.base_anchor_map_from_odom)
            || !canonical_isometry_is_finite(&transaction.absolute_left_correction)
            || !canonical_isometry_is_finite(&transaction.effective_map_from_odom)
            || !canonical_isometry_is_finite(&transaction.frozen_odom_from_body)
        {
            return Err("canonical correction transaction fields are invalid".to_owned());
        }
        let recomposed =
            transaction.base_anchor_map_from_odom * transaction.absolute_left_correction;
        if !canonical_isometry_near(&recomposed, &transaction.effective_map_from_odom) {
            return Err("effective correction is not base_anchor * absolute C".to_owned());
        }
        let current = self.current.read();
        match current.as_ref() {
            None if transaction.correction_revision != 1 => {
                return Err("first correction revision must be one".to_owned());
            }
            Some(current)
                if transaction.mapping_session_generation < current.mapping_session_generation =>
            {
                return Err("canonical correction transaction uses a stale session".to_owned());
            }
            Some(current)
                if transaction.mapping_session_generation == current.mapping_session_generation
                    && (transaction.mapping_session_id != current.mapping_session_id
                        || transaction.map_frame_generation != current.map_frame_generation) =>
            {
                return Err("canonical correction transaction session identity mismatch".to_owned());
            }
            Some(current)
                if transaction.mapping_session_generation > current.mapping_session_generation
                    && (transaction.correction_revision != 1
                        || transaction.occupancy_revision <= current.occupancy_revision
                        || transaction.cloud_revision <= current.cloud_revision) =>
            {
                return Err("new correction session did not reset revision cleanly".to_owned());
            }
            Some(current)
                if transaction.mapping_session_generation == current.mapping_session_generation
                    && ({
                        let immutable_map_reuse = transaction.backend
                            == crate::correction_coordinator::CorrectionCandidateBackend::Relocalization
                            && transaction.mapping_commit_id == current.mapping_commit_id
                            && transaction.occupancy_revision == current.occupancy_revision
                            && transaction.cloud_revision == current.cloud_revision;
                        transaction.correction_revision
                            != current
                                .correction_revision
                                .checked_add(1)
                                .ok_or_else(|| "correction revision exhausted".to_owned())?
                            || (!immutable_map_reuse
                                && (transaction.mapping_commit_id.boot_nonce
                                    != current.mapping_commit_id.boot_nonce
                                    || transaction.mapping_commit_id.sequence
                                        <= current.mapping_commit_id.sequence
                                    || transaction.occupancy_revision
                                        <= current.occupancy_revision
                                    || transaction.cloud_revision <= current.cloud_revision))
                            || transaction.source_timestamp_ns <= current.source_timestamp_ns
                    }) =>
            {
                return Err("canonical correction transaction regressed".to_owned());
            }
            _ => {}
        }
        Ok(())
    }
}

#[async_trait]
impl crate::correction_coordinator::CanonicalCorrectionApplier
    for GeometryCanonicalCorrectionApplier
{
    async fn apply_transaction(
        &self,
        transaction: crate::correction_coordinator::CanonicalCorrectionTransaction,
        cancellation: crate::correction_coordinator::CorrectionRequestCancellation,
    ) -> Result<(), crate::correction_coordinator::CanonicalCorrectionApplyError> {
        use crate::correction_coordinator::CanonicalCorrectionApplyError as ApplyError;

        let operation = cancellation.operation();
        if cancellation.is_cancelled() {
            return Err(ApplyError::Cancelled);
        }
        let mut shared = tokio::select! {
            biased;
            _ = cancellation.cancelled() => return Err(ApplyError::Cancelled),
            shared = self.state.write() => shared,
        };
        if cancellation.is_cancelled() {
            return Err(ApplyError::Cancelled);
        }
        self.validate_transaction(&transaction)
            .map_err(ApplyError::Failed)?;
        if shared.map.revision != transaction.occupancy_revision
            || shared.map.timestamp_ns != transaction.committed_map_source_timestamp_ns
            || normalize_frame(&shared.map.frame_id) != self.map_frame_id
            || shared.cloud_map_revision != transaction.cloud_revision
        {
            return Err(ApplyError::Failed(
                "canonical map/cloud state no longer matches correction receipt".to_owned(),
            ));
        }
        let odom = if shared.pose.timestamp_ns > transaction.frozen_odom_source_timestamp_ns {
            // 校正的证明属于冻结扫描，但公共输出不能覆盖已经提交的较新运动。
            // 原共享状态锁同时保护位姿与规范坐标树，不从外部设备或异步缓存补位姿。
            let latest = shared
                .tf_tree
                .lookup(&self.body_frame_id)
                .ok()
                .and_then(transform_to_entry)
                .filter(|entry| {
                    entry.parent == self.odom_frame_id
                        && entry.child == self.body_frame_id
                        && entry.stamp_ns == shared.pose.timestamp_ns
                        && normalize_frame(&shared.pose.frame_id) == self.map_frame_id
                })
                .ok_or_else(|| {
                    ApplyError::Failed(
                        "newer canonical pose has no matching odometry transform".to_owned(),
                    )
                })?;
            PoseState {
                x: latest.tx,
                y: latest.ty,
                z: latest.tz,
                qx: latest.rx,
                qy: latest.ry,
                qz: latest.rz,
                qw: latest.rw,
                frame_id: self.odom_frame_id.clone(),
                timestamp_ns: latest.stamp_ns,
            }
        } else {
            pose_state_from_isometry(
                &transaction.frozen_odom_from_body,
                &self.odom_frame_id,
                transaction.frozen_odom_source_timestamp_ns,
            )
        };
        let observation = canonical_mapping_observation_from_isometry(
            &self.map_frame_id,
            &self.odom_frame_id,
            transaction.correction_revision,
            &transaction.effective_map_from_odom,
            &odom,
            &self.body_frame_id,
        )
        .map_err(ApplyError::Failed)?;
        let tf_frame = crate::typed_codec::build_typed_frame(
            self.tf_binding.0.clone(),
            self.tf_binding.1.clone(),
            observation.tf_payload.clone(),
            transaction.correction_revision,
        );
        if !operation.try_claim(&cancellation) {
            return Err(ApplyError::Cancelled);
        }
        if let Err(error) = self.transport.publish(tf_frame).await {
            if let Err(phase_error) = operation.mark_acked_err() {
                operation.poison(phase_error.clone());
                return Err(ApplyError::Failed(phase_error));
            }
            return Err(ApplyError::Failed(format!(
                "canonical correction TF publish failed: {error}"
            )));
        }
        if let Err(error) = operation.mark_acked_ok() {
            operation.poison(error.clone());
            return Err(ApplyError::Failed(error));
        }
        apply_canonical_mapping_observation_locked(&mut shared, &observation);
        *self.current.write() = Some(CanonicalCorrectionSnapshot {
            runtime_generation: transaction.runtime_generation,
            backend_generation: transaction.backend_generation,
            backend: transaction.backend,
            mapping_session_id: transaction.mapping_session_id,
            mapping_session_generation: transaction.mapping_session_generation,
            map_frame_generation: transaction.map_frame_generation,
            mapping_commit_id: transaction.mapping_commit_id,
            occupancy_revision: transaction.occupancy_revision,
            cloud_revision: transaction.cloud_revision,
            correction_revision: transaction.correction_revision,
            source_timestamp_ns: transaction.source_timestamp_ns,
            committed_map_source_timestamp_ns: transaction.committed_map_source_timestamp_ns,
            map_frame_id: transaction.map_frame_id,
            odom_frame_id: transaction.odom_frame_id,
            body_frame_id: transaction.body_frame_id,
            correction_lease: transaction.correction_lease,
            effective_map_from_odom: transaction.effective_map_from_odom,
        });
        if let Err(error) = operation.mark_state_committed() {
            operation.poison(error.clone());
            return Err(ApplyError::Failed(error));
        }
        tracing::info!(
            event = "geometry.correction.applied",
            backend = ?transaction.backend,
            candidate_sequence = transaction.candidate_sequence,
            correction_revision = transaction.correction_revision,
            source_timestamp_ns = transaction.source_timestamp_ns,
            pose_timestamp_ns = observation.map_pose.timestamp_ns,
            correction_translation_m = ?transaction.absolute_left_correction.translation.vector,
            correction_rotation_deg = transaction.absolute_left_correction.rotation.angle().to_degrees(),
            "规范位姿校正已提交"
        );
        Ok(())
    }


}

fn pose_state_from_isometry(
    transform: &nalgebra::Isometry3<f64>,
    frame_id: &str,
    timestamp_ns: u64,
) -> PoseState {
    let rotation = transform.rotation.quaternion();
    PoseState {
        x: transform.translation.vector.x,
        y: transform.translation.vector.y,
        z: transform.translation.vector.z,
        qx: rotation.i,
        qy: rotation.j,
        qz: rotation.k,
        qw: rotation.w,
        frame_id: frame_id.to_owned(),
        timestamp_ns,
    }
}

fn canonical_isometry_is_finite(transform: &nalgebra::Isometry3<f64>) -> bool {
    let rotation = transform.rotation.quaternion();
    transform
        .translation
        .vector
        .iter()
        .all(|value| value.is_finite())
        && [rotation.i, rotation.j, rotation.k, rotation.w]
            .into_iter()
            .all(f64::is_finite)
}

fn canonical_isometry_near(
    left: &nalgebra::Isometry3<f64>,
    right: &nalgebra::Isometry3<f64>,
) -> bool {
    (left.translation.vector - right.translation.vector).norm() <= 1.0e-9
        && left.rotation.angle_to(&right.rotation) <= 1.0e-9
}

/// 单次 /map/occupancy frame 允许的 width/height 上限(防御性)。
///
/// 2000×2000 = 4M cells = 4 MiB(0.05m/cell 覆盖 100×100 m),已是物理合理
/// 上限。超过则上游 occupancy 帧字段异常,直接拒绝。width * height
/// 决定后续 `data` 字节数,这一项炸掉会直接 175 GB。
const OCCUPANCY_MAX_DIMENSION: u32 = 8192;

/// 单次 /explore/frontiers frame 允许的最大 frontier 条目数(防御性上限)。
///
/// 与 `TF_MAX_ENTRIES_PER_FRAME` 同源:防止异常帧的 count 字段触发
/// Vec::with_capacity 申请上百 GB。
const FRONTIERS_MAX_ENTRIES_PER_FRAME: usize = 16384;

/// 单次 /tf frame 允许的最大 transform 条目数(防御性上限)。
///
/// 理论合法值取决于上游 SLAM 输出,但物理合理范围是 < 1e4;一旦上游帧
/// 因 uninit memory / 协议异常把 count 字段解析成大数,`Vec::with_capacity`
/// 会触发数百 GB 的 mmap/alloc,直接炸掉 7 GiB 内存的 Jetson/Go2 实机。
/// 这里把上限钉死在 65536,既覆盖任何现实 SLAM 节点,又让异常帧
/// 仅触发 warn + 跳过(不阻塞其它 channel)。
const TF_MAX_ENTRIES_PER_FRAME: usize = 65536;

fn parse_tf_payload(payload: &[u8]) -> Option<Vec<TransformEntry>> {
    let transforms = crate::typed_codec::decode_tf_payload(payload)?;
    if transforms.len() > TF_MAX_ENTRIES_PER_FRAME {
        warn!(
            "/tf payload count={} 超过 sanity cap={};按异常帧跳过",
            transforms.len(),
            TF_MAX_ENTRIES_PER_FRAME
        );
        return None;
    }
    Some(transforms)
}

fn parse_occupancy_payload(payload: &[u8]) -> Option<MapState> {
    let message = decode_generated(
        schema::OCCUPANCY_GRID_FINGERPRINT,
        payload,
        schema::decode_occupancy_grid,
    )?;
    let width = u32::try_from(message.info.width).ok()?;
    let height = u32::try_from(message.info.height).ok()?;
    // 防御性 sanity cap:见 OCCUPANCY_MAX_DIMENSION 注释。
    if width == 0
        || height == 0
        || width > OCCUPANCY_MAX_DIMENSION
        || height > OCCUPANCY_MAX_DIMENSION
    {
        warn!(
            "/map/occupancy payload width={} height={} 超过 sanity cap={};按异常帧跳过",
            width, height, OCCUPANCY_MAX_DIMENSION
        );
        return None;
    }
    // data 必须等于 width*height;否则说明上游帧截断/异常,拒绝
    // 防止后续 Vec<i8> 申请非预期容量。
    let expected_len = (width as usize).saturating_mul(height as usize);
    if message.data.len() != expected_len {
        warn!(
            "/map/occupancy payload data_len={} 与 width*height={} 不一致;按异常帧跳过",
            message.data.len(),
            expected_len
        );
        return None;
    }
    Some(MapState {
        width,
        height,
        resolution: message.info.resolution,
        origin_x: message.info.origin.position[0],
        origin_y: message.info.origin.position[1],
        frame_id: message.header.frame_id.clone(),
        revision: 0,
        timestamp_ns: header_stamp_ns(&message.header)?,
        data: message.data,
    })
}

struct ParsedPointCloudPayload {
    data: Vec<u8>,
    point_step: u32,
    frame_id: String,
    timestamp_ns: u64,
}

fn parse_pointcloud_payload(payload: &[u8]) -> Option<ParsedPointCloudPayload> {
    let message = decode_generated(
        schema::POINT_CLOUD2_FINGERPRINT,
        payload,
        schema::decode_point_cloud2,
    )?;
    let timestamp_ns = header_stamp_ns(&message.header)?;
    let frame_id = message.header.frame_id.clone();
    let point_step = u32::try_from(message.point_step).ok()?;
    // 防御性 sanity cap:见 POINTCLOUD_MAX_POINT_STEP_BYTES 注释。
    // point_step 来自 untrusted LCM frame,sender 可伪造极端值(0/1/u32::MAX)
    // 让后续 `bytes.len() / point_step` 推算 raw_count 时产生巨大值,触发
    // `Vec::with_capacity(raw_count)` 申请数百 GB alloc(2026-07-08 实机
    // 187GB abort 模式)。
    if !(12..=crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES).contains(&point_step) {
        warn!(
            "/go2/lidar/points payload point_step={} 超过 sanity cap [12, {}];按异常帧跳过",
            point_step,
            crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES
        );
        return None;
    }
    // 防御性 sanity cap:payload 总长 4 MB 上限,远超物理合理点云单帧
    // (Go2 单帧 ~30k 点 ≈ 360 KB);超过则 upstream LCM 帧异常,拒绝。
    if message.data.len() > 4 * 1024 * 1024 {
        warn!(
            "/go2/lidar/points payload len={} 超过 4 MiB sanity cap;按异常帧跳过",
            message.data.len()
        );
        return None;
    }
    if message.is_bigendian {
        warn!("/go2/lidar/points big-endian PointCloud2 暂不支持;按异常帧跳过");
        return None;
    }
    let (height, width, row_step) = (
        usize::try_from(message.height).ok()?,
        usize::try_from(message.width).ok()?,
        usize::try_from(message.row_step).ok()?,
    );
    let point_count = height.checked_mul(width)?;
    if point_count > crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS {
        warn!(
            "/go2/lidar/points point_count={} 超过单帧上限={};按异常帧跳过",
            point_count,
            crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS
        );
        return None;
    }
    let packed_row = width.checked_mul(point_step as usize)?;
    if height == 0
        || width == 0
        || row_step != packed_row
        || message.data.len() != height.checked_mul(row_step)?
    {
        warn!("/go2/lidar/points PointCloud2 dimensions/data length inconsistent;按异常帧跳过");
        return None;
    }
    let xyz_layout = [("x", 0_i32), ("y", 4_i32), ("z", 8_i32)];
    if xyz_layout.iter().any(|(name, offset)| {
        !message.fields.iter().any(|field| {
            field.name == *name
                && field.offset == *offset
                && field.datatype == 7
                && field.count == 1
        })
    }) {
        warn!("/go2/lidar/points PointCloud2 缺少 packed float32 x/y/z 字段;按异常帧跳过");
        return None;
    }
    Some(ParsedPointCloudPayload {
        data: message.data,
        point_step,
        frame_id,
        timestamp_ns,
    })
}

// ============================================================================
// 启动：构造 LcmTransport (real-lcm) + SharedState + 各订阅 task
// ============================================================================

/// LCM backend 选择，只提供 `Real`。
/// `NUWAX_LCM_BACKEND` 仅接受 `real-lcm` 或空值；其它值返回
/// [`LcmBackendUnsupported`]，由二进制入口在产生启动副作用前退出。
///
/// Phase 7 readiness 字段仍由 `mapping_adapter::MappingReadiness`
/// + `nav_runtime::NavigationReadinessDetail`
/// + `explore_runtime::ExplorationReadinessDetail` 三个 layer 独立暴露
///   counters + active backend(backend 字段值固定为 "real-lcm")。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LcmBackendKind {
    /// 真实 liblcm backend(2026-07-08 唯一 backend)
    Real,
}

impl LcmBackendKind {
    pub(crate) fn from_env() -> Result<Self, LcmBackendUnsupported> {
        let raw = std::env::var("NUWAX_LCM_BACKEND").unwrap_or_default();
        let normalized = raw.trim().to_ascii_lowercase();
        match normalized.as_str() {
            "" => {
                tracing::info!("NUWAX_LCM_BACKEND 未设置,默认 real-lcm");
                Ok(Self::Real)
            }
            "real-lcm" => Ok(Self::Real),
            other => Err(LcmBackendUnsupported {
                value: other.to_owned(),
            }),
        }
    }

    fn canonical_name(self) -> &'static str {
        match self {
            Self::Real => "real-lcm",
        }
    }
}

/// 非生产 LCM backend 被拒绝时的稳定启动错误。
#[derive(Debug, Error, PartialEq, Eq)]
#[error("LCM_BACKEND_UNSUPPORTED: NUWAX_LCM_BACKEND={value:?} 不受支持。只允许 real-lcm 或空值。")]
pub(crate) struct LcmBackendUnsupported {
    value: String,
}

/// 读 `NUWAX_REAL_LCM_REQUIRED`：true/false/1/0/yes/no/on/off (大小写不敏感)。
///
/// - `true` + liblcm 不可用 → 返回带原始原因的 `LCM_BACKEND_UNAVAILABLE`，
///   由二进制入口统一决定退出码。
/// - `true` + liblcm OK → 启动 real-lcm backend 正常。
/// - 默认 false；与 `core/settings.py::REAL_LCM_REQUIRED` 语义保持对齐。
pub fn real_lcm_required_from_env() -> bool {
    let raw = std::env::var("NUWAX_REAL_LCM_REQUIRED").unwrap_or_default();
    matches!(
        raw.trim().to_ascii_lowercase().as_str(),
        "1" | "true" | "yes" | "on"
    )
}

/// `LcmTransport::new_real` 的启动结果分类。
///
/// `NUWAX_REAL_LCM_REQUIRED` 只分类创建失败，不能跳过真实 backend
/// 的创建尝试。将这一步保留为纯逻辑，既避免 required=true 被误写成前置退出，
/// 也便于在不终止测试进程的情况下锁定生产启动语义。
#[derive(Debug, PartialEq, Eq)]
enum RealLcmStartupError<E> {
    Required(E),
    Optional(E),
}

fn attempt_real_lcm_startup<T, E>(
    real_lcm_required: bool,
    create: impl FnOnce() -> Result<T, E>,
) -> Result<T, RealLcmStartupError<E>> {
    match create() {
        Ok(transport) => Ok(transport),
        Err(error) if real_lcm_required => Err(RealLcmStartupError::Required(error)),
        Err(error) => Err(RealLcmStartupError::Optional(error)),
    }
}

fn parse_expected_typed_channel(
    channel: &str,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, suffix) = crate::nav_runtime::parse_channel_with_suffix(channel)?;
    let suffix = suffix.ok_or_else(|| format!("typed channel {channel} 缺少 #message suffix"))?;
    let actual = LcmMessageType::new(&suffix)
        .map_err(|error| format!("typed channel {channel} message type 无效: {error}"))?;
    let expected = LcmMessageType::new(expected_type)
        .map_err(|error| format!("expected message type {expected_type} 无效: {error}"))?;
    if actual != expected {
        return Err(format!(
            "typed channel {channel} 的 message type={}，期望 {}",
            actual.fqn(),
            expected.fqn()
        ));
    }
    Ok((topic, actual))
}

fn binding_for_key(
    key: ChannelKey,
    expected_type: &str,
) -> Result<(LcmTopic, LcmMessageType), String> {
    let (topic, suffix) = channel_registry::channel_for_with_suffix(key);
    let suffix = suffix.ok_or_else(|| format!("channel_for_with_suffix({key:?}) 缺少 suffix"))?;
    parse_expected_typed_channel(&format!("{}#{suffix}", topic.channel()), expected_type)
}

#[derive(Clone)]
struct SharedStatePoseProvider {
    state: Arc<RwLock<SharedState>>,
    expected_map_frame: Arc<str>,
    freshness: Arc<parking_lot::Mutex<PoseFreshnessTracker>>,
    max_receipt_age: Duration,
}

#[derive(Default)]
struct PoseFreshnessTracker {
    source_timestamp_ns: u64,
    received_at: Option<std::time::Instant>,
    received_monotonic_ns: u64,
}

#[async_trait]
impl crate::explore_runtime::ExplorePoseProvider for SharedStatePoseProvider {
    async fn current_pose(&self) -> Result<crate::explore_runtime::ExploreRobotPose, String> {
        let pose = self.state.read().await.pose.clone();
        if pose.frame_id != self.expected_map_frame.as_ref() || pose.timestamp_ns == 0 {
            return Err(format!(
                "map-frame pose unavailable (expected_frame={}, frame={}, timestamp_ns={})",
                self.expected_map_frame, pose.frame_id, pose.timestamp_ns,
            ));
        }
        let source_age_ns =
            crate::explore_runtime::now_unix_nanos().saturating_sub(pose.timestamp_ns);
        if source_age_ns > self.max_receipt_age.as_nanos() as u64 {
            return Err(format!(
                "map-frame pose source timestamp is stale (source_timestamp_ns={}, source_age_ms={})",
                pose.timestamp_ns,
                source_age_ns / 1_000_000
            ));
        }
        let (received_monotonic_ns, receipt_age) = {
            let mut freshness = self.freshness.lock();
            if freshness.source_timestamp_ns != pose.timestamp_ns {
                freshness.source_timestamp_ns = pose.timestamp_ns;
                freshness.received_at = Some(std::time::Instant::now());
                freshness.received_monotonic_ns = crate::explore_runtime::now_monotonic_nanos();
            }
            let received_at = freshness
                .received_at
                .ok_or_else(|| "map-frame pose freshness observation is unavailable".to_string())?;
            (freshness.received_monotonic_ns, received_at.elapsed())
        };
        if receipt_age > self.max_receipt_age {
            return Err(format!(
                "map-frame pose is stale (source_timestamp_ns={}, receipt_age_ms={})",
                pose.timestamp_ns,
                receipt_age.as_millis()
            ));
        }
        Ok(crate::explore_runtime::ExploreRobotPose {
            x: pose.x,
            y: pose.y,
            z: pose.z,
            orientation: nuwax_nav_types_messages::geometry::Quaternion { x: pose.qx, y: pose.qy, z: pose.qz, w: pose.qw },
            frame_id: self.expected_map_frame.to_string(),
            source_timestamp_ns: pose.timestamp_ns,
            received_monotonic_ns,
        })
    }
}

fn exploration_child_lease(
    lease: &crate::nav_runtime::GoalLease,
) -> crate::explore_runtime::ChildNavigationLease {
    crate::explore_runtime::ChildNavigationLease {
        session_id: lease.session_id.clone(),
        owner_kind: lease.owner_kind,
        parent_request_id: lease.parent_request_id.clone(),
        goal_generation: lease.goal_generation,
        producer_runtime_generation: lease.producer_runtime_generation,
        goal_payload_sha256: lease.goal_payload_sha256,
    }
}

#[derive(Clone)]
struct ExploreNavigationBridge {
    state: Arc<RwLock<SharedState>>,
    expected_map_frame: Arc<str>,
    cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    nav_state_tx: broadcast::Sender<NavigationUpdate>,
    frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    explore_projection_gate: Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    transport: LcmTransport,
    transport_backend_name: &'static str,
    lcm_backend_available: bool,
    cmd_vel_sink_ready: Arc<std::sync::atomic::AtomicBool>,
    nav: crate::nav_runtime::NavRuntimeState,
    exploration_producer_epoch: Arc<std::sync::atomic::AtomicU64>,
    runtime_observation_generation: Arc<std::sync::atomic::AtomicU64>,
    orphan_cancellations: Arc<crate::explore_runtime::OrphanCancellationRegistry>,
    #[cfg(not(test))]
    task_control: Option<GeometryRuntimeTaskControl>,
}

#[cfg(test)]
type TestExploreBridgeTaskControls = parking_lot::Mutex<
    HashMap<
        usize,
        (
            Weak<std::sync::atomic::AtomicU64>,
            GeometryRuntimeTaskControl,
        ),
    >,
>;

#[cfg(test)]
static TEST_EXPLORE_BRIDGE_TASK_CONTROLS: LazyLock<TestExploreBridgeTaskControls> =
    LazyLock::new(|| parking_lot::Mutex::new(HashMap::new()));

impl ExploreNavigationBridge {
    #[cfg(not(test))]
    fn geometry_task_control(&self) -> Option<GeometryRuntimeTaskControl> {
        self.task_control.clone()
    }

    #[cfg(test)]
    fn geometry_task_control(&self) -> Option<GeometryRuntimeTaskControl> {
        let key = Arc::as_ptr(&self.runtime_observation_generation) as usize;
        let mut controls = TEST_EXPLORE_BRIDGE_TASK_CONTROLS.lock();
        controls.retain(|_, (generation, _)| generation.upgrade().is_some());
        let (generation, control) = controls.get(&key)?;
        if generation.upgrade().is_some_and(|generation| {
            Arc::ptr_eq(&generation, &self.runtime_observation_generation)
        }) {
            return Some(control.clone());
        }
        controls.remove(&key);
        None
    }

    #[cfg(test)]
    fn set_geometry_task_control_for_tests(&self, control: Option<GeometryRuntimeTaskControl>) {
        let key = Arc::as_ptr(&self.runtime_observation_generation) as usize;
        let mut controls = TEST_EXPLORE_BRIDGE_TASK_CONTROLS.lock();
        if let Some(control) = control {
            controls.insert(
                key,
                (
                    Arc::downgrade(&self.runtime_observation_generation),
                    control,
                ),
            );
        } else {
            controls.remove(&key);
        }
    }

    fn facade_handle(&self) -> GeometryRuntimeHandle {
        GeometryRuntimeHandle {
            state: self.state.clone(),
            cloud_map_tx: self.cloud_map_tx.clone(),
            cloud_map_writer: Arc::new(parking_lot::Mutex::new(None)),
            nav_state_tx: self.nav_state_tx.clone(),
            frontiers_tx: self.frontiers_tx.clone(),
            map_revision_tx: self.map_revision_tx.clone(),
            map_revision_latest_tx: self.map_revision_latest_tx.clone(),
            explore_projection_gate: self.explore_projection_gate.clone(),
            committed_map_authority: self.committed_map_authority.clone(),
            transport: self.transport.clone(),
            transport_backend_name: self.transport_backend_name,
            lcm_backend_available: self.lcm_backend_available,
            cmd_vel_sink_ready: self.cmd_vel_sink_ready.clone(),
            explore: crate::explore_runtime::ExploreRuntimeHandle::with_wavefront_map_frame(
                self.expected_map_frame.to_string(),
            )
            .expect("startup-validated map frame must remain valid"),
            nav: self.nav.clone(),
            mapping_adapter: None,
            mapping_lifecycle_lock: self.mapping_lifecycle_lock.clone(),
            occupancy_shm: None,
            occupancy_shm_path: None,
            occupancy_shm_current: Arc::new(RwLock::new(None)),
            explore_scheduler: Arc::new(tokio::sync::Mutex::new(None)),
            runtime_observation_generation: self.runtime_observation_generation.clone(),
            task_control: self.geometry_task_control(),
        }
    }

    async fn publish_state(&self, state: NavState, lease: Option<crate::nav_runtime::GoalLease>) {
        self.state.write().await.nav_state = state.clone();
        let update = NavigationUpdate::from_local(state, &self.nav, lease).await;
        let _ = self.nav_state_tx.send(update);
    }

    fn spawn_correlated_cancel_retry(
        &self,
        lease: crate::nav_runtime::GoalLease,
        cancel_generation: u64,
    ) -> Result<(), String> {
        let bridge = self.clone();
        #[cfg(test)]
        let topology_control = self
            .geometry_task_control()
            .and_then(|control| control.inner.topology_control.clone());
        let task = async move {
            #[cfg(test)]
            wait_for_topology_cancel_retry_release(topology_control).await;
            loop {
                tokio::time::sleep(EXPLORATION_CANCEL_RETRY_INTERVAL).await;
                let pending = bridge.nav.cancel_snapshot();
                if pending.phase != crate::nav_runtime::CancelPhase::Pending
                    || pending.generation != cancel_generation
                    || pending.lease.as_ref() != Some(&lease)
                {
                    return Ok(());
                }

                let facade = bridge.facade_handle();
                let outcome = crate::nav_runtime::cancel(
                    &facade,
                    &bridge.nav,
                    crate::nav_runtime::CancelGoalInput {
                        goal_id: lease.session_id.clone(),
                        owner_kind: lease.owner_kind,
                        parent_request_id: lease.parent_request_id.clone(),
                        goal_generation: lease.goal_generation,
                        producer_runtime_generation: lease.producer_runtime_generation,
                    },
                )
                .await;
                let current = bridge.nav.cancel_snapshot();
                if current.generation != cancel_generation
                    || current.lease.as_ref() != Some(&lease)
                    || current.phase != crate::nav_runtime::CancelPhase::Pending
                {
                    return Ok(());
                }
                if outcome.state.message == "CANCEL_PENDING_RETRY_PUBLISHED" {
                    info!(
                        session_id = %lease.session_id,
                        cancel_generation,
                        "exploration child cancellation exact retry published"
                    );
                } else {
                    warn!(
                        session_id = %lease.session_id,
                        cancel_generation,
                        status = %outcome.state.status,
                        message = %outcome.state.message,
                        "exploration child cancellation exact retry did not publish"
                    );
                }
            }
        };
        if let Some(control) = self.geometry_task_control() {
            control
                .register_retained("correlated-cancel-retry", task)
                .map_err(|error| {
                    format!(
                        "correlated cancellation retry admission failed for generation {cancel_generation}: {error}"
                    )
                })?;
        } else {
            tokio::spawn(async move {
                let _ = task.await;
            });
        }
        Ok(())
    }
}

fn classify_exploration_submit_rejection(
    retry_reason: Option<crate::nav_runtime::SetGoalRetryReason>,
    reject_reason: Option<crate::nav_runtime::SetGoalRejectReason>,
    message: String,
) -> crate::explore_runtime::ChildNavigationSubmitError {
    use crate::nav_runtime::{NavigationReadinessCode, SetGoalRejectReason, SetGoalRetryReason};

    if matches!(
        reject_reason,
        Some(SetGoalRejectReason::ActiveMapNameChanged)
    ) {
        return crate::explore_runtime::ChildNavigationSubmitError::SessionInvalidated(message);
    }
    let retryable = match retry_reason {
        Some(SetGoalRetryReason::NavigationNotReady(code)) => matches!(
            code,
            NavigationReadinessCode::MotionSinkUnobserved
                | NavigationReadinessCode::MotionSinkStale
                | NavigationReadinessCode::MotionSinkNotReady
                | NavigationReadinessCode::StackStateUnobserved
                | NavigationReadinessCode::StackStateStale
                | NavigationReadinessCode::TfUnobserved
                | NavigationReadinessCode::TfStale
                | NavigationReadinessCode::TfIncomplete
                | NavigationReadinessCode::PlannerInputUnobserved
                | NavigationReadinessCode::PlannerInputStale
                | NavigationReadinessCode::PlannerFrameMismatch
                | NavigationReadinessCode::CostmapUnavailable
                | NavigationReadinessCode::PlannerNotReady
        ),
        _ => false,
    };
    if retryable {
        crate::explore_runtime::ChildNavigationSubmitError::RetryableReadiness(message)
    } else {
        crate::explore_runtime::ChildNavigationSubmitError::Rejected(message)
    }
}

#[async_trait]
impl crate::explore_runtime::ExploreNavigationSink for ExploreNavigationBridge {
    fn current_cancel_generation(&self) -> Result<u64, String> {
        let snapshot = self.nav.cancel_snapshot();
        if snapshot.phase == crate::nav_runtime::CancelPhase::Failed
            && snapshot.message == "CANCEL_STATE_UNAVAILABLE"
        {
            return Err(snapshot.message);
        }
        Ok(snapshot.generation)
    }

    fn current_map_name(&self) -> Result<String, String> {
        self.map_revision_latest_tx
            .borrow()
            .as_ref()
            .map(|revision| revision.map_name.clone())
            .filter(|map_name| !map_name.trim().is_empty())
            .ok_or_else(|| "active map name unavailable".to_string())
    }

    async fn submit(
        &self,
        request: crate::explore_runtime::ChildNavigationRequest,
    ) -> Result<
        crate::explore_runtime::ChildNavigationTicket,
        crate::explore_runtime::ChildNavigationSubmitError,
    > {
        let child_request_id = request.request_id.clone();
        if request.intent.session_id != child_request_id
            || request.intent.parent_request_id != request.parent_request_id
            || request.intent.producer != "wavefront"
            || request.intent.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        {
            return Err(
                crate::explore_runtime::ChildNavigationSubmitError::Rejected(
                    "wavefront child request does not match its frozen GoalIntent".to_string(),
                ),
            );
        }
        let admission_context = crate::nav_runtime::NavAdmissionContext::with_expected_map_name(
            self.mapping_lifecycle_lock.clone(),
            self.committed_map_authority.clone(),
            self.map_revision_latest_tx.subscribe(),
            Some(request.expected_map_name.clone()),
        );
        let admission_io = crate::nav_runtime::NavAdmissionIo::new(
            self.state.clone(),
            self.transport.clone(),
            self.transport_backend_name,
            self.lcm_backend_available,
        );
        let outcome = crate::nav_runtime::admit_goal_intent(
            &admission_context,
            &admission_io,
            &self.nav,
            request.intent,
            crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
                current_session_generation: self.exploration_producer_epoch.clone(),
            },
        )
        .await;
        let accepted = matches!(
            outcome.state.status.to_ascii_uppercase().as_str(),
            "ACCEPTED" | "PLANNING" | "RUNNING"
        );
        let message = outcome.state.message.clone();
        let lease = outcome.lease.clone();
        let retry_reason = outcome.retry_reason;
        let reject_reason = outcome.reject_reason;
        self.publish_state(outcome.state, outcome.lease).await;
        if !accepted {
            return Err(classify_exploration_submit_rejection(
                retry_reason,
                reject_reason,
                message,
            ));
        }
        let lease = lease.as_ref().map(exploration_child_lease).ok_or_else(|| {
            crate::explore_runtime::ChildNavigationSubmitError::Rejected(
                "accepted exploration child is missing its goal lease".to_string(),
            )
        })?;
        Ok(crate::explore_runtime::ChildNavigationTicket { lease })
    }

    async fn cancel(
        &self,
        expected: &crate::explore_runtime::ChildNavigationLease,
    ) -> Result<crate::explore_runtime::ChildNavigationCancelTicket, String> {
        let facade = self.facade_handle();
        let lease = self
            .nav
            .active_goal_lease()
            .await
            .filter(|lease| {
                let projected = exploration_child_lease(lease);
                projected.matches(expected)
                    && lease.owner_kind == schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
            })
            .ok_or_else(|| "exploration child does not own the active goal lease".to_string())?;
        let outcome = crate::nav_runtime::cancel(
            &facade,
            &self.nav,
            crate::nav_runtime::CancelGoalInput {
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
            },
        )
        .await;
        let failed = outcome.state.status.eq_ignore_ascii_case("FAILED");
        let message = outcome.state.message.clone();
        let cancel = self.nav.cancel_snapshot();
        self.publish_state(outcome.state, outcome.lease).await;
        if failed {
            Err(message)
        } else {
            let cancel_lease = cancel
                .lease
                .as_ref()
                .map(exploration_child_lease)
                .ok_or_else(|| "navigation cancel tracker lost the child lease".to_string())?;
            if cancel.generation == 0 || !cancel_lease.matches(expected) {
                return Err("navigation cancel tracker returned a mismatched ticket".to_string());
            }
            self.spawn_correlated_cancel_retry(lease, cancel.generation)?;
            Ok(crate::explore_runtime::ChildNavigationCancelTicket {
                lease: cancel_lease,
                cancel_generation: cancel.generation,
            })
        }
    }

    fn recover_cancel_receipt(
        &self,
        expected: &crate::explore_runtime::ChildNavigationLease,
    ) -> Option<crate::explore_runtime::ChildNavigationCancelRecovery> {
        let cancel = self.nav.cancel_snapshot();
        if cancel.generation == 0 || cancel.phase == crate::nav_runtime::CancelPhase::Idle {
            return None;
        }
        let lease = cancel.lease.as_ref()?;
        if lease.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD {
            return None;
        }
        let projected = exploration_child_lease(lease);
        if !projected.matches(expected) {
            return None;
        }
        let retry_failure = if cancel.phase == crate::nav_runtime::CancelPhase::Pending {
            self.spawn_correlated_cancel_retry(lease.clone(), cancel.generation)
                .err()
        } else {
            None
        };
        let terminal_failure = matches!(
            cancel.phase,
            crate::nav_runtime::CancelPhase::Failed | crate::nav_runtime::CancelPhase::TimedOut
        )
        .then(|| cancel.message.clone());
        Some(crate::explore_runtime::ChildNavigationCancelRecovery {
            ticket: crate::explore_runtime::ChildNavigationCancelTicket {
                lease: projected,
                cancel_generation: cancel.generation,
            },
            failure: retry_failure.or(terminal_failure),
        })
    }

    fn retain_orphan_cancellation(
        &self,
        lease: crate::explore_runtime::ChildNavigationLease,
    ) -> Result<(), String> {
        self.orphan_cancellations.register(lease.clone());
        if !self.orphan_cancellations.claim_owner(&lease) {
            return Ok(());
        }
        let bridge = self.clone();
        let registry = self.orphan_cancellations.clone();
        let retained_lease = lease.clone();
        let task = async move {
            loop {
                if crate::explore_runtime::ExploreNavigationSink::orphan_lease_is_terminal(
                    &bridge,
                    &retained_lease,
                )
                .await
                {
                    registry.remove_terminal(&retained_lease);
                    return Ok(());
                }
                let should_cancel =
                    match crate::explore_runtime::ExploreNavigationSink::recover_cancel_receipt(
                        &bridge,
                        &retained_lease,
                    ) {
                        Some(recovery) if recovery.failure.is_none() => {
                            registry.mark_cancel_pending(&retained_lease);
                            false
                        }
                        Some(_) | None => true,
                    };
                if should_cancel {
                    match crate::explore_runtime::ExploreNavigationSink::cancel(
                        &bridge,
                        &retained_lease,
                    )
                    .await
                    {
                        Ok(_) => registry.mark_cancel_pending(&retained_lease),
                        Err(error) => warn!(
                            session_id = %retained_lease.session_id,
                            %error,
                            "orphan exploration lease cancellation retry failed"
                        ),
                    }
                }
                tokio::time::sleep(EXPLORATION_CANCEL_RETRY_INTERVAL).await;
            }
        };
        let admission = if let Some(control) = self.geometry_task_control() {
            control.register_retained("orphan-child-cancel", task)
        } else {
            tokio::spawn(async move {
                let _ = task.await;
            });
            Ok(())
        };
        if let Err(error) = admission {
            self.orphan_cancellations.release_owner(&lease);
            return Err(error);
        }
        Ok(())
    }

    async fn orphan_lease_is_terminal(
        &self,
        expected: &crate::explore_runtime::ChildNavigationLease,
    ) -> bool {
        if self
            .nav
            .active_goal_lease()
            .await
            .as_ref()
            .is_some_and(|lease| exploration_child_lease(lease).matches(expected))
        {
            return false;
        }
        let cancel = self.nav.cancel_snapshot();
        let matching_cancel = cancel.lease.as_ref().filter(|lease| {
            lease.owner_kind == schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
                && exploration_child_lease(lease).matches(expected)
        });
        match matching_cancel {
            Some(_) => cancel.phase == crate::nav_runtime::CancelPhase::Zeroed,
            None => true,
        }
    }
}

fn cancel_phase_name(phase: crate::nav_runtime::CancelPhase) -> &'static str {
    match phase {
        crate::nav_runtime::CancelPhase::Idle => "IDLE",
        crate::nav_runtime::CancelPhase::Pending => "PENDING",
        crate::nav_runtime::CancelPhase::Zeroed => "ZEROED",
        crate::nav_runtime::CancelPhase::Failed => "FAILED",
        crate::nav_runtime::CancelPhase::TimedOut => "TIMED_OUT",
    }
}

fn normalize_pending_cancel_zero_ack_path_identity(
    nav: &crate::nav_runtime::NavRuntimeState,
    update: &mut NavigationUpdate,
) {
    let feedback = &update.feedback;
    let complete_zero_proof = feedback.cancel_generation > 0
        && !feedback.zero_ack_id.trim().is_empty()
        && feedback.zero_applied_monotonic_ns > 0;
    let pre_path_sentinel = feedback.path_revision == 0 && feedback.path_payload_sha256 == [0; 32];
    if !complete_zero_proof || !pre_path_sentinel {
        return;
    }

    let cancel = nav.cancel_snapshot();
    let Some(lease) = cancel.lease.as_ref() else {
        return;
    };
    let authoritative_path_is_sealed =
        lease.path_revision > 0 && lease.path_payload_sha256 != [0; 32];
    let cancellation_identity_matches = matches!(
        cancel.phase,
        crate::nav_runtime::CancelPhase::Pending
            | crate::nav_runtime::CancelPhase::Failed
            | crate::nav_runtime::CancelPhase::TimedOut
    ) && cancel.generation == feedback.cancel_generation
        && cancel.goal_id == update.state.goal_id
        && lease.session_id == update.state.goal_id
        && lease.owner_kind == feedback.owner_kind
        && lease.parent_request_id == feedback.parent_request_id
        && lease.goal_generation == feedback.goal_generation
        && lease.goal_payload_sha256 == feedback.goal_payload_sha256;
    if !authoritative_path_is_sealed || !cancellation_identity_matches {
        return;
    }

    update.feedback.path_revision = lease.path_revision;
    update.feedback.path_payload_sha256 = lease.path_payload_sha256;
}

async fn apply_typed_navigation_feedback(
    nav: &crate::nav_runtime::NavRuntimeState,
    mut update: NavigationUpdate,
) -> Option<NavigationUpdate> {
    normalize_pending_cancel_zero_ack_path_identity(nav, &mut update);
    let planner_observation = if update.feedback.runtime_generation > 0 {
        let map_frame = normalize_frame(&update.feedback.planner_map_frame);
        let pose_frame = normalize_frame(&update.feedback.planner_pose_frame);
        let ready = !map_frame.is_empty() && map_frame == pose_frame;
        Some(crate::nav_runtime::PlannerInputObservation {
            generation: update.feedback.runtime_generation,
            map_revision: 0,
            map_frame,
            pose_frame,
            ready,
        })
    } else if !update.feedback.planner_map_frame.is_empty()
        || !update.feedback.planner_pose_frame.is_empty()
    {
        warn!("planner feedback missing positive runtime_generation; fail closed");
        None
    } else {
        None
    };

    let idle_motion_sink_ready =
        update.state.goal_id.is_empty() && update.feedback.guard_generation == 0;
    let motion_sink_observation = if update.feedback.guard_generation > 0 {
        Some(crate::nav_runtime::MotionSinkObservation {
            generation: update.feedback.guard_generation,
            ready: update.feedback.sink_ready,
        })
    } else if update.feedback.sink_ready && !idle_motion_sink_ready {
        warn!("sink_ready=true missing positive guard_generation; fail closed");
        None
    } else {
        None
    };

    if update.state.goal_id.is_empty() {
        if let Some(observation) = planner_observation {
            if let Err(error) = nav.observe_planner_input(observation) {
                warn!(?error, "typed planner observation rejected");
            }
        }
        if let Some(observation) = motion_sink_observation {
            if let Err(error) = nav.observe_motion_sink(observation) {
                warn!(?error, "typed motion-sink observation rejected");
            }
        } else if idle_motion_sink_ready {
            if let Err(error) = nav.observe_idle_motion_sink(update.feedback.sink_ready) {
                warn!(?error, "idle motion-sink observation rejected");
            }
        }
        update.state = nav.observe_unowned_stack_heartbeat(update.state);
    } else {
        match nav
            .apply_correlated_navigation_state(
                update.state.clone(),
                &update.state.goal_id,
                update.feedback.owner_kind,
                &update.feedback.parent_request_id,
                update.feedback.goal_generation,
                update.feedback.path_revision,
                update.feedback.goal_payload_sha256,
                update.feedback.path_payload_sha256,
                planner_observation,
                motion_sink_observation,
            )
            .await
        {
            Ok((state, lease)) => {
                update.state = state;
                update.producer_runtime_generation = lease.producer_runtime_generation;
            }
            Err(error) => {
                let cancel = nav.cancel_snapshot();
                let active = nav.active_goal_lease().await;
                let (expected, expected_lease_source) =
                    match (active.as_ref(), cancel.lease.as_ref()) {
                        (Some(active), _) => (Some(active), "active"),
                        (None, Some(cancel)) => (Some(cancel), "cancel"),
                        (None, None) => (None, "none"),
                    };
                warn!(
                    %error,
                    session_id = %update.state.goal_id,
                    received_path_revision = update.feedback.path_revision,
                    expected_lease_source,
                    expected_path_revision = expected.map_or(0, |lease| lease.path_revision),
                    received_path_hash_zero = update.feedback.path_payload_sha256 == [0; 32],
                    expected_path_hash_zero = match expected {
                        Some(lease) => lease.path_payload_sha256 == [0; 32],
                        None => true,
                    },
                    path_hash_matches = expected.is_some_and(|lease| {
                        lease.path_payload_sha256 == update.feedback.path_payload_sha256
                    }),
                    "typed navigation state rejected"
                );
                return None;
            }
        }
    }
    let has_zero_proof = update.feedback.cancel_generation > 0
        || !update.feedback.zero_ack_id.is_empty()
        || update.feedback.zero_applied_monotonic_ns > 0;
    let complete_zero_proof = update.feedback.cancel_generation > 0
        && !update.feedback.zero_ack_id.trim().is_empty()
        && update.feedback.zero_applied_monotonic_ns > 0;
    let cancel_snapshot = nav.cancel_snapshot();
    let correlated = complete_zero_proof
        && matches!(
            cancel_snapshot.phase,
            crate::nav_runtime::CancelPhase::Pending
                | crate::nav_runtime::CancelPhase::Failed
                | crate::nav_runtime::CancelPhase::TimedOut
        )
        && cancel_snapshot.generation == update.feedback.cancel_generation
        && cancel_snapshot.goal_id == update.state.goal_id
        && cancel_snapshot.lease.as_ref().is_some_and(|lease| {
            lease.owner_kind == update.feedback.owner_kind
                && lease.parent_request_id == update.feedback.parent_request_id
                && lease.goal_generation == update.feedback.goal_generation
        });

    if correlated {
        let outcome = nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: update.feedback.cancel_generation,
                goal_id: update.state.goal_id.clone(),
                owner_kind: update.feedback.owner_kind,
                parent_request_id: update.feedback.parent_request_id.clone(),
                goal_generation: update.feedback.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: format!(
                    "motion guard ZEROED ack={} applied_monotonic_ns={}",
                    update.feedback.zero_ack_id, update.feedback.zero_applied_monotonic_ns
                ),
            })
            .await;
        update.state = outcome.state;
        update.feedback.cancel_phase = "ZEROED".to_string();
        update.feedback.actuator_zeroed = true;
        update.feedback.reason_code = "CANCEL_ZEROED".to_string();
    } else {
        if has_zero_proof {
            warn!(
                received_generation = update.feedback.cancel_generation,
                expected_generation = cancel_snapshot.generation,
                received_goal = %update.state.goal_id,
                expected_goal = %cancel_snapshot.goal_id,
                "incomplete or uncorrelated zero acknowledgement rejected"
            );
            update.feedback.zero_ack_id.clear();
            update.feedback.zero_applied_monotonic_ns = 0;
        }
        let current = nav.cancel_snapshot();
        update.feedback.cancel_phase = cancel_phase_name(current.phase).to_string();
        update.feedback.actuator_zeroed = current.phase == crate::nav_runtime::CancelPhase::Zeroed;
        update.feedback.reason_code =
            navigation_reason_code(&update.state, update.feedback.cancel_phase.as_str());
    }
    Some(update)
}

async fn report_exploration_navigation_terminal(
    explore: &crate::explore_runtime::ExploreRuntimeHandle,
    update: &NavigationUpdate,
) {
    let snapshot = explore.snapshot();
    let child_task_id = snapshot.child_task_id;
    if child_task_id.is_empty()
        || child_task_id != update.state.goal_id
        || update.feedback.owner_kind != schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        || update.feedback.parent_request_id != snapshot.request_id
    {
        return;
    }
    if let Some(result) = exploration_navigation_terminal_result(update) {
        if let Err(error) = explore
            .report_navigation_result(&child_task_id, result)
            .await
        {
            warn!(child_task_id, %error, "exploration child feedback rejected");
        }
    }
}

async fn fanout_navigation_update(
    state: &Arc<RwLock<SharedState>>,
    nav_state_tx: &broadcast::Sender<NavigationUpdate>,
    explore: &crate::explore_runtime::ExploreRuntimeHandle,
    update: NavigationUpdate,
) {
    if exploration_navigation_terminal_result(&update).is_some() {
        match state.try_write() {
            Ok(mut state) => state.nav_state = update.state.clone(),
            Err(_) => warn!(
                goal_id = %update.state.goal_id,
                status = %update.state.status,
                "navigation telemetry cache busy; preserving ordered broadcast and ownership terminal"
            ),
        }
    } else {
        state.write().await.nav_state = update.state.clone();
    }
    let _ = nav_state_tx.send(update.clone());
    report_exploration_navigation_terminal(explore, &update).await;
}

fn exploration_navigation_terminal_result(
    update: &NavigationUpdate,
) -> Option<crate::explore_runtime::ChildNavigationResult> {
    let proof = crate::explore_runtime::ChildNavigationTerminalProof {
        lease: crate::explore_runtime::ChildNavigationLease {
            session_id: update.state.goal_id.clone(),
            owner_kind: update.feedback.owner_kind,
            parent_request_id: update.feedback.parent_request_id.clone(),
            goal_generation: update.feedback.goal_generation,
            producer_runtime_generation: update.producer_runtime_generation,
            goal_payload_sha256: update.feedback.goal_payload_sha256,
        },
    };
    match update.state.status.as_str() {
        "SUCCEEDED" => Some(crate::explore_runtime::ChildNavigationResult::Reached(
            proof,
        )),
        "FAILED"
            if matches!(
                update.feedback.cancel_phase.as_str(),
                "FAILED" | "TIMED_OUT"
            ) =>
        {
            Some(
                crate::explore_runtime::ChildNavigationResult::CancelFailed {
                    proof,
                    cancel_generation: update.feedback.cancel_generation,
                    reason: update.state.message.clone(),
                },
            )
        }
        "FAILED" => Some(crate::explore_runtime::ChildNavigationResult::Failed {
            proof,
            reason: update.state.message.clone(),
        }),
        "CANCELLED"
            if update.feedback.actuator_zeroed
                && update.feedback.cancel_generation > 0
                && !update.feedback.zero_ack_id.trim().is_empty()
                && update.feedback.zero_applied_monotonic_ns > 0 =>
        {
            Some(crate::explore_runtime::ChildNavigationResult::Cancelled(
                crate::explore_runtime::ChildNavigationZeroProof {
                    lease: proof.lease,
                    cancel_generation: update.feedback.cancel_generation,
                    zero_ack_id: update.feedback.zero_ack_id.clone(),
                    zero_applied_monotonic_ns: update.feedback.zero_applied_monotonic_ns,
                },
            ))
        }
        _ => None,
    }
}

fn spawn_cancel_timeout_task(
    nav: crate::nav_runtime::NavRuntimeState,
    state: Arc<RwLock<SharedState>>,
    nav_state_tx: broadcast::Sender<NavigationUpdate>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
) -> JoinHandle<()> {
    let future = cancel_timeout_future(nav, state, nav_state_tx, explore);
    tokio::spawn(async move {
        let _ = future.await;
    })
}

fn cancel_timeout_future(
    nav: crate::nav_runtime::NavRuntimeState,
    state: Arc<RwLock<SharedState>>,
    nav_state_tx: broadcast::Sender<NavigationUpdate>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
) -> GeometryTaskFuture {
    Box::pin(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(100));
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            interval.tick().await;
            let Some(outcome) = nav.poll_cancel_timeout().await else {
                continue;
            };
            let update = NavigationUpdate::from_local(outcome.state, &nav, outcome.lease).await;
            fanout_navigation_update(&state, &nav_state_tx, &explore, update).await;
        }
    })
}

fn spawn_path_envelope_consumer(
    transport: LcmTransport,
    nav: crate::nav_runtime::NavRuntimeState,
    path_binding: (LcmTopic, LcmMessageType),
) -> JoinHandle<()> {
    let (path_topic, path_msg_type) = path_binding;
    tokio::spawn(async move {
        let mut stream = transport.subscribe(path_topic, Some(path_msg_type)).await;
        use futures::StreamExt;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(frame) => frame,
                Err(error) => {
                    error!(%error, "PathEnvelope LCM subscription failed");
                    continue;
                }
            };
            let path = match crate::typed_codec::decode_path_envelope_payload(&frame.payload) {
                Ok(path) => path,
                Err(error) => {
                    warn!(%error, "PathEnvelope payload rejected");
                    continue;
                }
            };
            if let Err(error) = nav.observe_path_envelope(&path).await {
                warn!(%error, session_id = %path.session_id, "PathEnvelope lease rejected");
            }
        }
    })
}

fn frontier_entry(
    frontier: &nuwax_nav_types_messages::visual::FrontierGoal,
    expected_map_frame: &str,
) -> FrontierEntry {
    let pose = frontier.pose.pose;
    FrontierEntry {
        x: pose.position.x,
        y: pose.position.y,
        z: pose.position.z,
        qx: pose.orientation.x,
        qy: pose.orientation.y,
        qz: pose.orientation.z,
        qw: pose.orientation.w,
        frame_id: expected_map_frame.to_string(),
        size_cells: frontier
            .metadata
            .get("cell_count")
            .and_then(|value| value.parse().ok())
            .unwrap_or(0),
        heading: pose.orientation.yaw() as f32,
    }
}

fn stable_frontier_id(value: &str) -> i64 {
    let hash = value
        .as_bytes()
        .iter()
        .fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
            (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
        });
    hash as i64
}

fn encode_frontier_snapshot(
    snapshot: &crate::explore_runtime::FrontierSnapshot,
) -> Result<Vec<u8>, String> {
    encode_frontier_snapshot_in_frame(snapshot, "map")
}

fn encode_frontier_snapshot_in_frame(
    snapshot: &crate::explore_runtime::FrontierSnapshot,
    expected_map_frame: &str,
) -> Result<Vec<u8>, String> {
    if snapshot.frontiers.len() > schema::FRONTIER_LIST_CAPACITY {
        return Err(format!(
            "frontier count {} exceeds LCM capacity {}",
            snapshot.frontiers.len(),
            schema::FRONTIER_LIST_CAPACITY
        ));
    }
    if snapshot.frontiers.iter().any(|frontier| {
        !matches!(
            &frontier.pose.frame_id,
            nuwax_nav_types_messages::frames::NavFrameId::Map
        )
    }) {
        return Err(format!(
            "frontier snapshot contains a non-map semantic goal for configured frame {expected_map_frame}"
        ));
    }
    let now_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64;
    let mut frontiers = vec![schema::Frontier::default(); schema::FRONTIER_LIST_CAPACITY];
    for (slot, frontier) in frontiers.iter_mut().zip(snapshot.frontiers.iter()) {
        *slot = schema::Frontier {
            id: stable_frontier_id(&frontier.id),
            center: [frontier.pose.pose.position.x, frontier.pose.pose.position.y],
            radius: frontier
                .frontier_area_m2
                .map(|area| (f64::from(area) / std::f64::consts::PI).sqrt())
                .unwrap_or(0.0),
            score: f64::from(frontier.score),
            cell_count: frontier
                .metadata
                .get("cell_count")
                .and_then(|value| value.parse().ok())
                .unwrap_or(0),
            note: frontier.producer.clone(),
        };
    }
    schema::encode_frontier_list_body(&schema::FrontierList {
        stamp_sec: (now_ns / 1_000_000_000) as i64,
        stamp_nsec: (now_ns % 1_000_000_000) as i32,
        frame_id: expected_map_frame.to_string(),
        frontiers_length: snapshot.frontiers.len(),
        frontiers,
    })
    .map_err(|error| format!("encode frontier snapshot failed: {error}"))
}

fn spawn_map_revision_coalescer(
    map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) -> JoinHandle<()> {
    spawn_map_revision_coalescer_in_frame(
        map_revision_rx,
        map_revision_latest_tx,
        committed_map_authority,
        mapping_adapter,
        mapping_lifecycle_lock,
        Arc::from("map"),
        OccupancyShmCommitMode::Disabled,
    )
}

enum OccupancyShmCommitMode {
    Disabled,
    Misconfigured,
    Ready {
        writer: Arc<parking_lot::Mutex<OccupancyGridShmWriter>>,
        path: String,
        current: Arc<RwLock<Option<OccupancyShmSnapshot>>>,
    },
}

fn spawn_map_revision_coalescer_in_frame(
    map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    expected_map_frame: Arc<str>,
    occupancy_shm: OccupancyShmCommitMode,
) -> JoinHandle<()> {
    let future = map_revision_coalescer_future_in_frame(
        map_revision_rx,
        map_revision_latest_tx,
        committed_map_authority,
        mapping_adapter,
        mapping_lifecycle_lock,
        expected_map_frame,
        occupancy_shm,
    );
    tokio::spawn(async move {
        let _ = future.await;
    })
}

fn map_revision_coalescer_future_in_frame(
    mut map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    expected_map_frame: Arc<str>,
    occupancy_shm: OccupancyShmCommitMode,
) -> GeometryTaskFuture {
    Box::pin(async move {
        loop {
            match map_revision_rx.recv().await {
                Ok(event) => {
                    if event.frame_id != expected_map_frame.as_ref() {
                        warn!(
                            revision = event.revision,
                            frame_id = %event.frame_id,
                            expected_map_frame = %expected_map_frame,
                            "map revision rejected before latest-state retention"
                        );
                        continue;
                    }
                    let claimed = {
                        let _lifecycle_guard = mapping_lifecycle_lock.lock().await;
                        map_event_matches_current_session(&event, &mapping_adapter)
                    };
                    if !claimed {
                        warn!(
                            session_id = %event.session_id,
                            session_generation = event.session_generation,
                            revision = event.revision,
                            "stale map revision rejected by lifecycle fence"
                        );
                        continue;
                    }

                    let prepared = match &occupancy_shm {
                        OccupancyShmCommitMode::Disabled => None,
                        OccupancyShmCommitMode::Misconfigured => {
                            warn!(
                                revision = event.revision,
                                "occupancy SHM path is unavailable; map revision withheld"
                            );
                            continue;
                        }
                        OccupancyShmCommitMode::Ready { writer, .. } => {
                            let map = event.to_map_state();
                            let session_generation = event.session_generation;
                            // Keep mmap preparation inside the supervised task. A Tokio
                            // abort cannot report this task joined until the synchronous
                            // writer section has actually returned, so root settlement
                            // cannot race a detached `spawn_blocking` closure.
                            let prepare_result = {
                                let mut writer = writer.lock();
                                writer.begin_session(session_generation).and_then(|()| {
                                    #[cfg(test)]
                                    wait_for_topology_revision_prepare_release();
                                    writer.prepare_occupancy(&map)
                                })
                            };
                            #[cfg(test)]
                            if let Some(control) = topology_test_control() {
                                if control.revision_prepare_entered() {
                                    control.record("revision-prepare-finished");
                                }
                            }
                            match prepare_result {
                                Ok(prepared) => Some(prepared),
                                Err(error) => {
                                    warn!(
                                        revision = event.revision,
                                        %error,
                                        "occupancy SHM prepare failed; map revision withheld"
                                    );
                                    continue;
                                }
                            }
                        }
                    };

                    let lifecycle_guard = mapping_lifecycle_lock.lock().await;
                    if !map_event_matches_current_session(&event, &mapping_adapter) {
                        if let (OccupancyShmCommitMode::Ready { writer, .. }, Some(prepared)) =
                            (&occupancy_shm, prepared)
                        {
                            let _ = writer.lock().abort_prepared(prepared);
                        }
                        warn!(
                            session_id = %event.session_id,
                            session_generation = event.session_generation,
                            revision = event.revision,
                            "map revision invalidated before occupancy SHM commit"
                        );
                        continue;
                    }

                    match (&occupancy_shm, prepared) {
                        (OccupancyShmCommitMode::Disabled, None) => {}
                        (
                            OccupancyShmCommitMode::Ready {
                                writer,
                                path,
                                current,
                            },
                            Some(prepared),
                        ) => {
                            let mut current = current.write().await;
                            if current
                                .as_ref()
                                .is_some_and(|snapshot| snapshot.revision > event.revision)
                            {
                                let _ = writer.lock().abort_prepared(prepared);
                                warn!(
                                    revision = event.revision,
                                    current_revision = current
                                        .as_ref()
                                        .map(|snapshot| snapshot.revision)
                                        .unwrap_or_default(),
                                    "older occupancy SHM prepare rejected before commit"
                                );
                                continue;
                            }
                            let commit = match writer.lock().commit_prepared(prepared) {
                                Ok(commit) => commit,
                                Err(error) => {
                                    warn!(
                                        revision = event.revision,
                                        %error,
                                        "occupancy SHM commit failed; map revision withheld"
                                    );
                                    continue;
                                }
                            };
                            *current = Some(OccupancyShmSnapshot {
                                key: path.clone(),
                                file_size: commit.file_size,
                                payload_size: event.size,
                                revision: commit.revision,
                                layout_version: commit.layout_version,
                                payload_schema_version: event.schema_version,
                                generation: commit.generation,
                                active_slot: commit.active_slot,
                            });
                        }
                        _ => {
                            warn!(
                                revision = event.revision,
                                "occupancy SHM commit state is inconsistent; map revision withheld"
                            );
                            continue;
                        }
                    }
                    let committed_map = match committed_map_identity_from_revision(&event) {
                        Ok(identity) => identity,
                        Err(error) => {
                            warn!(revision = event.revision, %error, "committed map identity rejected before authority publication");
                            continue;
                        }
                    };
                    let mut authority = match committed_map_authority.lock() {
                        Ok(authority) => authority,
                        Err(_) => {
                            warn!(
                                revision = event.revision,
                                "committed map authority state is unavailable"
                            );
                            continue;
                        }
                    };
                    if authority.as_ref().is_some_and(|current| {
                        let same_domain = current.session_id == committed_map.session_id
                            && current.session_generation == committed_map.session_generation;
                        !same_domain
                            || committed_map.revision < current.revision
                            || (committed_map.revision == current.revision
                                && committed_map != *current)
                    }) {
                        warn!(
                            revision = event.revision,
                            "stale, conflicting, or event-driven map domain switch rejected"
                        );
                        continue;
                    }
                    *authority = Some(committed_map);
                    map_revision_latest_tx.send_replace(Some(event));
                    drop(authority);
                    drop(lifecycle_guard);
                }
                Err(broadcast::error::RecvError::Lagged(skipped)) => {
                    warn!(
                        skipped,
                        "map revision ingress lagged; coalescer will publish the latest retained revision"
                    );
                }
                Err(broadcast::error::RecvError::Closed) => break,
            }
        }
        Ok(())
    })
}

fn spawn_map_revision_fanout(
    map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    explore_projection_gate: Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    transport: LcmTransport,
    occupancy_topic: LcmTopic,
    occupancy_msg_type: LcmMessageType,
    occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
    occupancy_shm_path: Option<String>,
    occupancy_shm_current: Arc<RwLock<Option<OccupancyShmSnapshot>>>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) -> Vec<JoinHandle<()>> {
    spawn_map_revision_fanout_in_frame(
        map_revision_rx,
        map_revision_latest_tx,
        committed_map_authority,
        explore_projection_gate,
        transport,
        occupancy_topic,
        occupancy_msg_type,
        occupancy_shm,
        occupancy_shm_path,
        occupancy_shm_current,
        explore,
        mapping_adapter,
        mapping_lifecycle_lock,
        Arc::from("map"),
    )
}

fn spawn_map_revision_fanout_in_frame(
    map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    explore_projection_gate: Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    transport: LcmTransport,
    occupancy_topic: LcmTopic,
    occupancy_msg_type: LcmMessageType,
    occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
    occupancy_shm_path: Option<String>,
    occupancy_shm_current: Arc<RwLock<Option<OccupancyShmSnapshot>>>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    expected_map_frame: Arc<str>,
) -> Vec<JoinHandle<()>> {
    map_revision_fanout_futures_in_frame(
        map_revision_rx,
        map_revision_latest_tx,
        committed_map_authority,
        explore_projection_gate,
        transport,
        occupancy_topic,
        occupancy_msg_type,
        occupancy_shm,
        occupancy_shm_path,
        occupancy_shm_current,
        explore,
        mapping_adapter,
        mapping_lifecycle_lock,
        expected_map_frame,
    )
    .into_iter()
    .map(|future| {
        tokio::spawn(async move {
            let _ = future.await;
        })
    })
    .collect()
}

fn map_revision_fanout_futures_in_frame(
    map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    explore_projection_gate: Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    transport: LcmTransport,
    occupancy_topic: LcmTopic,
    occupancy_msg_type: LcmMessageType,
    occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
    occupancy_shm_path: Option<String>,
    occupancy_shm_current: Arc<RwLock<Option<OccupancyShmSnapshot>>>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
    expected_map_frame: Arc<str>,
) -> Vec<GeometryTaskFuture> {
    let mut lcm_rx = map_revision_latest_tx.subscribe();
    let mut explore_rx = map_revision_latest_tx.subscribe();
    let occupancy_shm = match (occupancy_shm, occupancy_shm_path) {
        (Some(writer), Some(path)) => OccupancyShmCommitMode::Ready {
            writer,
            path,
            current: occupancy_shm_current,
        },
        (Some(_), None) => {
            warn!("occupancy SHM writer exists without its authoritative path");
            OccupancyShmCommitMode::Misconfigured
        }
        (None, _) => OccupancyShmCommitMode::Disabled,
    };

    let lcm_mapping_adapter = mapping_adapter.clone();
    let lcm_lifecycle_lock = mapping_lifecycle_lock.clone();
    let lcm_expected_map_frame = expected_map_frame.clone();
    let lcm_task: GeometryTaskFuture = Box::pin(async move {
        while lcm_rx.changed().await.is_ok() {
            let Some(event) = lcm_rx.borrow_and_update().clone() else {
                continue;
            };
            let claimed = {
                let _lifecycle_guard = lcm_lifecycle_lock.lock().await;
                map_event_matches_current_session(&event, &lcm_mapping_adapter)
            };
            if !claimed {
                continue;
            }
            let payload = match encode_occupancy_revision_in_frame(
                &event,
                lcm_expected_map_frame.as_ref(),
            ) {
                Ok(payload) => payload,
                Err(error) => {
                    warn!(revision = event.revision, %error, "map revision rejected at global fanout boundary");
                    continue;
                }
            };
            let frame = crate::typed_codec::build_typed_frame(
                occupancy_topic.clone(),
                occupancy_msg_type.clone(),
                payload,
                event.revision,
            );
            match tokio::time::timeout(MAP_FANOUT_SINK_TIMEOUT, transport.publish(frame)).await {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    warn!(revision = event.revision, %error, "occupancy LCM fanout failed");
                }
                Err(_) => {
                    warn!(revision = event.revision, "occupancy LCM fanout timed out");
                }
            }
        }
        Ok(())
    });

    let explore_mapping_adapter = mapping_adapter.clone();
    let explore_lifecycle_lock = mapping_lifecycle_lock.clone();
    let explore_committed_map_authority = committed_map_authority.clone();
    let explore_expected_map_frame = expected_map_frame.clone();
    let explore_task: GeometryTaskFuture = Box::pin(async move {
        loop {
            let event = {
                let mut projection = explore_projection_gate.lock().await;
                if projection.pending_latest.is_none() {
                    drop(projection);
                    if explore_rx.changed().await.is_err() {
                        break;
                    }
                    let observed = {
                        let observed = explore_rx.borrow_and_update();
                        observed.clone()
                    };
                    let Some(observed) = observed else {
                        continue;
                    };
                    projection = explore_projection_gate.lock().await;
                    let pending_is_older = match projection.pending_latest.as_ref() {
                        Some(current) => newer_projection_event(current, &observed),
                        None => true,
                    };
                    if pending_is_older {
                        projection.pending_latest = Some(observed);
                    }
                }
                projection
                    .pending_latest
                    .clone()
                    .expect("projection pending event initialized above")
            };

            let snapshot = match exploration_costmap_from_revision_in_frame(
                &event,
                explore_expected_map_frame.as_ref(),
            ) {
                Ok(snapshot) => snapshot,
                Err(error) => {
                    warn!(revision = event.revision, %error, "exploration costmap conversion failed");
                    let mut projection = explore_projection_gate.lock().await;
                    if projection
                        .pending_latest
                        .as_ref()
                        .is_some_and(|current| same_projection_event(current, &event))
                    {
                        projection.pending_latest = None;
                    }
                    continue;
                }
            };
            let committed_map = match committed_map_identity_from_revision(&event) {
                Ok(identity) => identity,
                Err(error) => {
                    warn!(revision = event.revision, %error, "committed map identity rejected before exploration fanout");
                    let mut projection = explore_projection_gate.lock().await;
                    if projection
                        .pending_latest
                        .as_ref()
                        .is_some_and(|current| same_projection_event(current, &event))
                    {
                        projection.pending_latest = None;
                    }
                    continue;
                }
            };

            // P serializes projection apply with cutover. L is held only for
            // exact-domain validation and never crosses Explore ingest await.
            let mut projection = explore_projection_gate.lock().await;
            if !projection
                .pending_latest
                .as_ref()
                .is_some_and(|current| same_projection_event(current, &event))
            {
                continue;
            }
            let current = {
                let _lifecycle_guard = explore_lifecycle_lock.lock().await;
                let active_domain_matches =
                    map_event_matches_current_session(&event, &explore_mapping_adapter);
                let authority_matches = explore_committed_map_authority
                    .lock()
                    .ok()
                    .and_then(|authority| authority.clone())
                    .as_ref()
                    .is_some_and(|authority| authority == &committed_map);
                active_domain_matches && authority_matches
            };
            if !current {
                projection.pending_latest = None;
                continue;
            }

            let ingest_result = tokio::time::timeout(
                MAP_FANOUT_SINK_TIMEOUT,
                explore.ingest_committed_costmap_snapshot(snapshot, committed_map.clone()),
            )
            .await;
            let still_current = {
                let _lifecycle_guard = explore_lifecycle_lock.lock().await;
                map_event_matches_current_session(&event, &explore_mapping_adapter)
                    && explore_committed_map_authority
                        .lock()
                        .ok()
                        .and_then(|authority| authority.clone())
                        .as_ref()
                        .is_some_and(|authority| authority == &committed_map)
            };
            let retry_pending = !matches!(&ingest_result, Ok(Ok(_)));
            match ingest_result {
                Ok(Ok(_)) if still_current => {
                    projection.pending_latest = None;
                }
                Ok(Ok(_)) => {
                    projection.pending_latest = None;
                    warn!(
                        revision = event.revision,
                        "exploration costmap projection lost authority during apply"
                    );
                }
                Ok(Err(error)) => {
                    warn!(revision = event.revision, %error, "exploration costmap fanout failed; latest revision retained for retry");
                }
                Err(_) => {
                    warn!(
                        revision = event.revision,
                        "exploration costmap fanout timed out; latest revision retained for retry"
                    );
                }
            }
            drop(projection);

            if retry_pending {
                tokio::select! {
                    _ = tokio::time::sleep(MAP_FANOUT_SINK_TIMEOUT) => {}
                    changed = explore_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        let observed = {
                            let observed = explore_rx.borrow_and_update();
                            observed.clone()
                        };
                        if let Some(observed) = observed {
                            let mut projection = explore_projection_gate.lock().await;
                            let pending_is_older = match projection.pending_latest.as_ref() {
                                Some(current) => newer_projection_event(current, &observed),
                                None => true,
                            };
                            if pending_is_older {
                                projection.pending_latest = Some(observed);
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    });

    let coalescer_task = map_revision_coalescer_future_in_frame(
        map_revision_rx,
        map_revision_latest_tx,
        committed_map_authority,
        mapping_adapter,
        mapping_lifecycle_lock,
        expected_map_frame,
        occupancy_shm,
    );
    vec![coalescer_task, lcm_task, explore_task]
}

fn spawn_forbidden_zone_watcher(
    state: Arc<RwLock<SharedState>>,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) -> JoinHandle<()> {
    let future = forbidden_zone_watcher_future(
        state,
        mapping_adapter,
        map_revision_tx,
        mapping_lifecycle_lock,
    );
    tokio::spawn(async move {
        let _ = future.await;
    })
}

fn forbidden_zone_watcher_future(
    state: Arc<RwLock<SharedState>>,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) -> GeometryTaskFuture {
    let poll_interval_ms = std::env::var("NUWAX_FORBIDDEN_ZONES_POLL_INTERVAL_MS")
        .ok()
        .and_then(|value| value.trim().parse::<u64>().ok())
        .filter(|value| *value >= 50)
        .unwrap_or(250);
    Box::pin(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(poll_interval_ms));
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            interval.tick().await;
            let request = {
                let _lifecycle_guard = mapping_lifecycle_lock.lock().await;
                mapping_adapter.lock().begin_forbidden_zone_refresh()
            };
            let Some(mut request) = request else {
                continue;
            };
            // 文件读取和解析不占用建图锁；没有新增脱离根拥有方的后台工作。
            let loaded = request.load();
            let _lifecycle_guard = mapping_lifecycle_lock.lock().await;
            let result = mapping_adapter.lock().finish_forbidden_zone_refresh(
                request,
                loaded,
                &state,
                &map_revision_tx,
            );
            match result {
                Ok(true) => info!("forbidden-zone update committed as canonical map revision"),
                Ok(false) => {}
                Err(error) => warn!(%error, "forbidden-zone watcher update failed; will retry"),
            }
        }
        #[allow(unreachable_code)]
        Ok(())
    })
}

fn spawn_frontier_fanout(
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    state: Arc<RwLock<SharedState>>,
    frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    transport: LcmTransport,
    frontier_binding: Option<(LcmTopic, LcmMessageType)>,
) -> JoinHandle<()> {
    spawn_frontier_fanout_in_frame(
        explore,
        state,
        frontiers_tx,
        transport,
        frontier_binding,
        Arc::from("map"),
    )
}

fn spawn_frontier_fanout_in_frame(
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    state: Arc<RwLock<SharedState>>,
    frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    transport: LcmTransport,
    frontier_binding: Option<(LcmTopic, LcmMessageType)>,
    expected_map_frame: Arc<str>,
) -> JoinHandle<()> {
    let future = frontier_fanout_future_in_frame(
        explore,
        state,
        frontiers_tx,
        transport,
        frontier_binding,
        expected_map_frame,
    );
    tokio::spawn(async move {
        let _ = future.await;
    })
}

fn frontier_fanout_future_in_frame(
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    state: Arc<RwLock<SharedState>>,
    frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    transport: LcmTransport,
    frontier_binding: Option<(LcmTopic, LcmMessageType)>,
    expected_map_frame: Arc<str>,
) -> GeometryTaskFuture {
    let mut frontier_rx = explore.subscribe_frontiers();
    Box::pin(async move {
        loop {
            if frontier_rx.changed().await.is_err() {
                break;
            }
            let snapshot = frontier_rx.borrow_and_update().clone();
            if snapshot.frontiers.iter().any(|frontier| {
                !matches!(
                    &frontier.pose.frame_id,
                    nuwax_nav_types_messages::frames::NavFrameId::Map
                )
            }) {
                warn!(
                    revision = snapshot.revision,
                    expected_map_frame = %expected_map_frame,
                    "frontier snapshot rejected outside configured map frame"
                );
                continue;
            }
            let entries: Vec<_> = snapshot
                .frontiers
                .iter()
                .map(|frontier| frontier_entry(frontier, expected_map_frame.as_ref()))
                .collect();
            {
                let mut shared = state.write().await;
                shared.frontiers = entries.clone();
                shared.frontiers_revision = snapshot.revision;
            }
            let _ = frontiers_tx.send(entries);
            let Some((topic, msg_type)) = frontier_binding.as_ref() else {
                continue;
            };
            let payload =
                match encode_frontier_snapshot_in_frame(&snapshot, expected_map_frame.as_ref()) {
                    Ok(payload) => payload,
                    Err(error) => {
                        warn!(revision = snapshot.revision, %error, "frontier LCM fanout rejected");
                        continue;
                    }
                };
            let frame = crate::typed_codec::build_typed_frame(
                topic.clone(),
                msg_type.clone(),
                payload,
                snapshot.revision,
            );
            if let Err(error) = transport.publish(frame).await {
                warn!(revision = snapshot.revision, %error, "frontier LCM fanout failed");
            }
        }
        #[allow(unreachable_code)]
        Ok(())
    })
}

const PUBLIC_TF_HISTORY_TTL_NS: u64 = 10_000_000_000;

#[derive(Default)]
struct PublicTfHistory {
    entries: HashMap<(String, String), TransformEntry>,
    latest_stamp_ns: u64,
}

impl PublicTfHistory {
    fn merge(&mut self, entries: Vec<TransformEntry>) -> bool {
        let mut accepted = false;
        for mut entry in entries {
            if entry_to_transform(&entry).is_none() {
                warn!(
                    parent = %entry.parent,
                    child = %entry.child,
                    "忽略 frame/quaternion 非法的 /tf entry"
                );
                continue;
            }
            entry.parent = normalize_frame(&entry.parent);
            entry.child = normalize_frame(&entry.child);
            self.latest_stamp_ns = self.latest_stamp_ns.max(entry.stamp_ns);
            let key = (entry.parent.clone(), entry.child.clone());
            let replace = self
                .entries
                .get(&key)
                .is_none_or(|current| entry.stamp_ns == 0 || entry.stamp_ns >= current.stamp_ns);
            if replace {
                self.entries.insert(key, entry);
            }
            accepted = true;
        }
        if accepted {
            let latest_stamp_ns = self.latest_stamp_ns;
            self.entries.retain(|_, entry| {
                entry.stamp_ns == 0
                    || latest_stamp_ns.saturating_sub(entry.stamp_ns) <= PUBLIC_TF_HISTORY_TTL_NS
            });
        }
        accepted
    }

    fn entries(&self) -> Vec<TransformEntry> {
        self.entries.values().cloned().collect()
    }
}

/// Observe public TF frames for navigation telemetry without granting the
/// public channel write authority over canonical geometry state.
async fn observe_public_tf_stream(
    mut stream: nuwax_transport::LcmFrameStream,
    map_frame: String,
    odom_frame: String,
    base_frame: String,
    nav_runtime: crate::nav_runtime::NavRuntimeState,
) {
    use futures::StreamExt;

    let mut tf_generation = 0_u64;
    let mut history = PublicTfHistory::default();
    while let Some(frame_result) = stream.next().await {
        let frame = match frame_result {
            Ok(frame) => frame,
            Err(error) => {
                error!("/tf LCM 订阅错误: {}", error);
                continue;
            }
        };
        let entries = match parse_tf_payload(&frame.payload) {
            Some(entries) => entries,
            None => {
                warn!("/tf payload 解析失败 ({} 字节)", frame.payload.len());
                continue;
            }
        };

        // Public TF is telemetry-only. This short-lived history restores
        // cross-message chain compatibility without touching canonical state.
        if !history.merge(entries) {
            continue;
        }
        let observed_entries = history.entries();

        let base_candidates = semantic_base_candidates(&observed_entries, &base_frame);
        let map_to_odom = resolve_transform(&observed_entries, &odom_frame, &map_frame).is_some();
        let odom_to_base = base_candidates
            .iter()
            .any(|base| resolve_transform(&observed_entries, base, &odom_frame).is_some());
        tf_generation = tf_generation.saturating_add(1);
        if let Err(error) = nav_runtime.observe_tf_chain(crate::nav_runtime::TfChainObservation {
            generation: tf_generation,
            map_frame: normalize_frame(&map_frame),
            odom_frame: normalize_frame(&odom_frame),
            base_frame: normalize_frame(&base_frame),
            map_to_odom,
            odom_to_base,
        }) {
            warn!(?error, "nav TF observation rejected");
        }
    }
}

async fn wait_for_geometry_task_stop(shutdown_rx: &mut watch::Receiver<bool>) {
    if *shutdown_rx.borrow() {
        return;
    }
    let _ = shutdown_rx.wait_for(|stopping| *stopping).await;
}

async fn run_persistent_future_until_stop(
    mut shutdown_rx: watch::Receiver<bool>,
    mut future: GeometryTaskFuture,
) -> Result<(), String> {
    tokio::select! {
        biased;
        _ = wait_for_geometry_task_stop(&mut shutdown_rx) => Ok(()),
        result = future.as_mut() => result,
    }
}

async fn settle_explore_scheduler(
    scheduler: &crate::explore_runtime::ExploreSchedulerHandle,
) -> Result<(), String> {
    let mut sticky_error = None;
    loop {
        let outcome = scheduler.shutdown_with_settlement().await;
        let settled = outcome.producers_settled();
        if let Err(error) = outcome.into_result() {
            sticky_error.get_or_insert(error);
        }
        if settled {
            return sticky_error.map_or(Ok(()), Err);
        }
        tokio::task::yield_now().await;
    }
}

async fn run_explore_scheduler_persistent(
    scheduler: crate::explore_runtime::ExploreSchedulerHandle,
    mut shutdown_rx: watch::Receiver<bool>,
) -> Result<(), String> {
    tokio::select! {
        biased;
        _ = wait_for_geometry_task_stop(&mut shutdown_rx) => {
            settle_explore_scheduler(&scheduler).await
        }
        terminal = scheduler.wait_terminal() => {
            let terminal = terminal.and_then(|()| {
                Err("exploration scheduler exited without a shutdown request".to_owned())
            });
            let settlement = settle_explore_scheduler(&scheduler).await;
            match (terminal, settlement) {
                (Err(terminal), Ok(())) | (Ok(()), Err(terminal)) => Err(terminal),
                (Err(terminal), Err(settlement)) => Err(format!("{terminal}; {settlement}")),
                (Ok(()), Ok(())) => Ok(()),
            }
        }
    }
}

async fn register_geometry_startup_persistent(
    control: &GeometryRuntimeTaskControl,
    task_owner: &mut Option<GeometryRuntimeTaskOwner>,
    handle: &GeometryRuntimeHandle,
    pending_subscriptions: &mut Vec<(&'static str, nuwax_transport::OwnedLcmSubscription)>,
    pending_children: &mut Vec<(&'static str, JoinHandle<()>)>,
    name: &'static str,
    future: impl Future<Output = Result<(), String>> + Send + 'static,
) -> Result<(), GeometryRuntimeStartupFailure> {
    let (registration, started_rx) = GeometryRuntimeTaskControl::registration_with_start_ack(
        name,
        GeometryTaskClass::Persistent,
        future,
    );
    let (registration_error, rejected_future) = match control.try_register(registration) {
        Ok(()) => match tokio::time::timeout(GEOMETRY_TASK_START_ACK_TIMEOUT, started_rx).await {
            Ok(Ok(())) => return Ok(()),
            Ok(Err(_)) => (
                format!("Geometry task supervisor dropped before starting {name}"),
                None,
            ),
            Err(_) => (
                format!("Geometry task start acknowledgement timed out: {name}"),
                None,
            ),
        },
        Err(rejected) => {
            let detail = rejected.detail();
            (detail, Some(rejected.registration.future))
        }
    };

    control.request_stop();
    control.request_persistent_stop();
    #[cfg(test)]
    let mut rejected_work = Some(GeometryRuntimeStartupRejectedWorkOwner::spawn(
        name,
        rejected_future,
        std::mem::take(pending_subscriptions),
        std::mem::take(pending_children),
        control.inner.topology_control.clone(),
    ));
    #[cfg(not(test))]
    let mut rejected_work = Some(GeometryRuntimeStartupRejectedWorkOwner::spawn(
        name,
        rejected_future,
        std::mem::take(pending_subscriptions),
        std::mem::take(pending_children),
    ));
    let owner_shutdown = async {
        match task_owner.as_mut() {
            Some(owner) => owner.shutdown().await,
            None => GeometryTaskShutdownOutcome::unsettled(
                "Geometry startup rollback lost the root task owner",
            ),
        }
    };
    let rejected_settlement = async {
        match rejected_work.as_mut() {
            Some(owner) => owner.shutdown().await,
            None => GeometryTaskShutdownOutcome::new(Ok(()), true),
        }
    };
    let (owner_outcome, rejected_outcome) = tokio::join!(owner_shutdown, rejected_settlement);
    if rejected_outcome.producers_settled() {
        rejected_work.take();
    }
    let owner_settled = owner_outcome.producers_settled();
    let rejected_settled = rejected_outcome.producers_settled();
    let cleanup_result = if owner_settled && rejected_settled {
        #[cfg(test)]
        if let Some(topology_control) = &control.inner.topology_control {
            topology_control.emit_startup_shutdown_stage(
                crate::lio_owner::GeometryShutdownStage::GeometryTasksSettled,
            );
        }
        cleanup_geometry_startup_files(handle)
    } else {
        Err(
            "Geometry startup producers/subscriptions did not settle; product mmap cleanup withheld"
                .to_owned(),
        )
    };

    let mut rollback_errors = Vec::new();
    if let Err(error) = rejected_outcome.result {
        rollback_errors.push(error);
    }
    if let Err(error) = owner_outcome.result {
        rollback_errors.push(format!("task-owner rollback failed: {error}"));
    }
    let cleanup_settled = cleanup_result.is_ok();
    #[cfg(test)]
    if cleanup_settled {
        if let Some(topology_control) = &control.inner.topology_control {
            topology_control.emit_startup_shutdown_stage(
                crate::lio_owner::GeometryShutdownStage::ResourceCleanupComplete,
            );
        }
    }
    if let Err(error) = cleanup_result {
        rollback_errors.push(error);
    }
    let rollback = if rollback_errors.is_empty() {
        "rollback completed".to_owned()
    } else {
        rollback_errors.join("; ")
    };
    let error = RealLcmBackendUnavailable::transport(format!("{registration_error}; {rollback}"));
    if !owner_settled || !rejected_settled || !cleanup_settled {
        if let Some(task_owner) = task_owner.take() {
            return Err(
                GeometryRuntimeStartupFailure::with_rejected_work_rollback_owner(
                    error,
                    handle.clone(),
                    task_owner,
                    rejected_work,
                ),
            );
        }
    }
    task_owner.take();
    Err(error.into())
}

#[cfg(test)]
async fn register_spawned_persistent(
    control: &GeometryRuntimeTaskControl,
    task_owner: &mut Option<GeometryRuntimeTaskOwner>,
    handle: &GeometryRuntimeHandle,
    pending_subscriptions: &mut Vec<(&'static str, nuwax_transport::OwnedLcmSubscription)>,
    pending_children: &mut Vec<(&'static str, JoinHandle<()>)>,
    name: &'static str,
    mut child: JoinHandle<()>,
) -> Result<(), GeometryRuntimeStartupFailure> {
    let mut shutdown_rx = control.shutdown_receiver();
    register_geometry_startup_persistent(
        control,
        task_owner,
        handle,
        pending_subscriptions,
        pending_children,
        name,
        async move {
            tokio::select! {
                biased;
                _ = wait_for_geometry_task_stop(&mut shutdown_rx) => {
                    child.abort();
                    match child.await {
                        Ok(()) => Ok(()),
                        Err(error) if error.is_cancelled() => Ok(()),
                        Err(error) => Err(format!("{name} abort join failed: {error}")),
                    }
                }
                joined = &mut child => joined
                    .map_err(|error| format!("{name} join failed: {error}")),
            }
        },
    )
    .await
}

async fn rollback_pending_geometry_children(
    mut children: Vec<(&'static str, JoinHandle<()>)>,
) -> Result<(), String> {
    let mut errors = Vec::new();
    while let Some((name, child)) = children.pop() {
        child.abort();
        match child.await {
            Ok(()) => {}
            Err(error) if error.is_cancelled() => {}
            Err(error) => errors.push(format!("{name} startup abort join failed: {error}")),
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

fn cleanup_geometry_startup_files(handle: &GeometryRuntimeHandle) -> Result<(), String> {
    let mut errors = Vec::new();
    if let Some(writer) = handle.cloud_map_writer.lock().as_mut() {
        if let Err(error) = writer.cleanup_product_owned_file() {
            errors.push(format!("cloud-map startup mmap cleanup failed: {error}"));
        }
    }
    if let Some(writer) = handle.occupancy_shm.as_ref() {
        if let Err(error) = writer.lock().cleanup_product_owned_file() {
            errors.push(format!("occupancy startup mmap cleanup failed: {error}"));
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

async fn unsubscribe_owned_geometry_subscription(
    name: &'static str,
    subscription: nuwax_transport::OwnedLcmSubscription,
    terminal: Result<(), String>,
) -> Result<(), String> {
    let unsubscribe = subscription
        .unsubscribe()
        .await
        .map_err(|error| format!("{name} unsubscribe failed: {error}"));
    #[cfg(test)]
    if unsubscribe.is_ok() {
        record_topology_event(format!("unsubscribed:{name}"));
    }
    match (terminal, unsubscribe) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
        (Err(terminal), Err(unsubscribe)) => Err(format!("{terminal}; {unsubscribe}")),
    }
}

async fn run_owned_public_tf_observer(
    mut subscription: nuwax_transport::OwnedLcmSubscription,
    map_frame: String,
    odom_frame: String,
    base_frame: String,
    nav_runtime: crate::nav_runtime::NavRuntimeState,
    mut shutdown_rx: watch::Receiver<bool>,
) -> Result<(), String> {
    let mut tf_generation = 0_u64;
    let mut history = PublicTfHistory::default();
    #[cfg(test)]
    let injected_terminal = topology_test_control()
        .and_then(|control| control.injected_terminal_for("public-tf-observer"));
    #[cfg(test)]
    if let Some(injected_terminal) = injected_terminal {
        wait_for_topology_terminal_release().await;
        let terminal = match injected_terminal {
            GeometryTopologyInjectedTerminal::Eof
            | GeometryTopologyInjectedTerminal::Panic
            | GeometryTopologyInjectedTerminal::JoinFailure => Ok(()),
            GeometryTopologyInjectedTerminal::TransportError => {
                Err("public TF receive failed: injected transport error".to_owned())
            }
        };
        let result =
            unsubscribe_owned_geometry_subscription("public-tf-observer", subscription, terminal)
                .await;
        if injected_terminal == GeometryTopologyInjectedTerminal::Panic {
            panic!("injected panic in public-tf-observer after unsubscribe acknowledgement");
        }
        return result;
    }
    let terminal = loop {
        tokio::select! {
            biased;
            _ = wait_for_geometry_task_stop(&mut shutdown_rx) => break Ok(()),
            frame_result = subscription.recv() => {
                let frame = match frame_result {
                    Ok(frame) => frame,
                    Err(error) => break Err(format!("public TF receive failed: {error}")),
                };
                let Some(entries) = parse_tf_payload(&frame.payload) else {
                    warn!("/tf payload 解析失败 ({} 字节)", frame.payload.len());
                    continue;
                };
                if !history.merge(entries) {
                    continue;
                }
                let observed_entries = history.entries();
                let base_candidates = semantic_base_candidates(&observed_entries, &base_frame);
                let map_to_odom =
                    resolve_transform(&observed_entries, &odom_frame, &map_frame).is_some();
                let odom_to_base = base_candidates.iter().any(|base| {
                    resolve_transform(&observed_entries, base, &odom_frame).is_some()
                });
                tf_generation = tf_generation.saturating_add(1);
                if let Err(error) =
                    nav_runtime.observe_tf_chain(crate::nav_runtime::TfChainObservation {
                        generation: tf_generation,
                        map_frame: normalize_frame(&map_frame),
                        odom_frame: normalize_frame(&odom_frame),
                        base_frame: normalize_frame(&base_frame),
                        map_to_odom,
                        odom_to_base,
                    })
                {
                    warn!(?error, "nav TF observation rejected");
                }
            }
        }
    };
    unsubscribe_owned_geometry_subscription("public-tf-observer", subscription, terminal).await
}

async fn run_owned_path_envelope_consumer(
    mut subscription: nuwax_transport::OwnedLcmSubscription,
    nav: crate::nav_runtime::NavRuntimeState,
    mut shutdown_rx: watch::Receiver<bool>,
) -> Result<(), String> {
    let terminal = loop {
        tokio::select! {
            biased;
            _ = wait_for_geometry_task_stop(&mut shutdown_rx) => break Ok(()),
            frame_result = subscription.recv() => {
                let frame = match frame_result {
                    Ok(frame) => frame,
                    Err(error) => break Err(format!("PathEnvelope receive failed: {error}")),
                };
                let path = match crate::typed_codec::decode_path_envelope_payload(&frame.payload) {
                    Ok(path) => path,
                    Err(error) => {
                        warn!(%error, "PathEnvelope payload rejected");
                        continue;
                    }
                };
                if let Err(error) = nav.observe_path_envelope(&path).await {
                    warn!(%error, session_id = %path.session_id, "PathEnvelope lease rejected");
                }
            }
        }
    };
    unsubscribe_owned_geometry_subscription("path-envelope-consumer", subscription, terminal).await
}

async fn run_owned_nav_state_consumer(
    mut subscription: nuwax_transport::OwnedLcmSubscription,
    state: Arc<RwLock<SharedState>>,
    nav_state_tx: broadcast::Sender<NavigationUpdate>,
    nav_runtime: crate::nav_runtime::NavRuntimeState,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    mut shutdown_rx: watch::Receiver<bool>,
) -> Result<(), String> {
    let terminal = loop {
        tokio::select! {
            biased;
            _ = wait_for_geometry_task_stop(&mut shutdown_rx) => break Ok(()),
            frame_result = subscription.recv() => {
                let frame = match frame_result {
                    Ok(frame) => frame,
                    Err(error) => break Err(format!("navigation-state receive failed: {error}")),
                };
                let Some(mut update) = crate::typed_codec::decode_nav_state_payload(&frame.payload)
                else {
                    warn!("nav_state generated payload 解析失败 ({} 字节)", frame.payload.len());
                    continue;
                };
                if let Some(target) = nav_runtime.spatial_feedback_target(
                    &update.state.goal_id, update.feedback.owner_kind,
                    &update.feedback.parent_request_id, update.feedback.goal_generation,
                    update.feedback.goal_payload_sha256,
                ).await {
                    let current = state.read().await.pose.clone();
                    let distance = (current.x - target.x).hypot(current.y - target.y)
                        .hypot(current.z - target.z);
                    if distance.is_finite() {
                        update.state.distance_remaining = distance as f32;
                    }
                }
                let Some(update) = apply_typed_navigation_feedback(&nav_runtime, update).await else {
                    continue;
                };
                fanout_navigation_update(&state, &nav_state_tx, &explore, update).await;
            }
        }
    };
    unsubscribe_owned_geometry_subscription("navigation-state-consumer", subscription, terminal)
        .await
}

type SurfaceMapComputation = (
    crate::explore_runtime::SurfaceMapTicket,
    Result<Arc<nuwax_nav_mls_planner::SurfaceSnapshot>, String>,
);

fn build_exploration_surface_map(
    message: &schema::NavigationMap3D,
    geometry: nuwax_nav_mls_planner::mls_planner::Config,
) -> Result<Arc<nuwax_nav_mls_planner::SurfaceSnapshot>, String> {
    let points = schema::navigation_map_3d_points(message).map_err(|error| error.to_string())?;
    let mut snapshot = nuwax_nav_mls_planner::SurfaceSnapshot::from_points(
        &points, geometry, message.source_voxel_size,
    ).map_err(|error| error.to_string())?;
    if let Some(mask) = message.virtual_obstacles.as_ref() {
        let blocked = mask.data.iter().map(|value| *value == 100).collect::<Vec<_>>();
        snapshot = snapshot.with_virtual_obstacles(
            [mask.info.origin.position[0], mask.info.origin.position[1]],
            f64::from(mask.info.resolution), mask.info.width as u32, mask.info.height as u32, &blocked,
        ).map_err(|error| error.to_string())?;
    }
    Ok(Arc::new(snapshot))
}

fn register_surface_map_build(
    control: &GeometryRuntimeTaskControl,
    ticket: crate::explore_runtime::SurfaceMapTicket,
    completed: tokio::sync::mpsc::Sender<SurfaceMapComputation>,
    compute: impl FnOnce() -> Result<Arc<nuwax_nav_mls_planner::SurfaceSnapshot>, String> + Send + 'static,
) -> Result<(), String> {
    control.register_retained("surface-map-build", async move {
        // 注册接纳后才启动实际计算；保留任务始终等待阻塞工作句柄结束。
        let result = tokio::task::spawn_blocking(compute).await
            .map_err(|error| format!("surface map worker join failed: {error}"))
            .and_then(|result| result);
        let _ = completed.send((ticket, result)).await;
        Ok(())
    })
}

/// 订阅拥有方保持接收清空消息；至多一个保留任务构建表面，排队只保留最新输入。
async fn run_owned_surface_map_consumer(
    mut subscription: nuwax_transport::OwnedLcmSubscription,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    config: nuwax_nav_stack::surface_config::SurfaceNavigationConfig,
    control: GeometryRuntimeTaskControl,
    mut shutdown_rx: watch::Receiver<bool>,
) -> Result<(), String> {
    let (computed_tx, mut computed_rx) = tokio::sync::mpsc::channel::<SurfaceMapComputation>(1);
    let mut computing = false;
    let mut latest_pending: Option<(schema::NavigationMap3D, crate::explore_runtime::SurfaceMapTicket)> = None;
    let terminal = loop {
        if !computing {
            if let Some((message, ticket)) = latest_pending.take() {
                let geometry = config.geometry.clone();
                let completed = computed_tx.clone();
                let rejected_ticket = ticket.clone();
                let compute = move || build_exploration_surface_map(&message, geometry);
                match register_surface_map_build(&control, ticket, completed, compute) {
                    Ok(()) => computing = true,
                    Err(error) => {
                        explore.finish_surface_map_update(rejected_ticket, Err(error)).await;
                    }
                }
            }
        }
        tokio::select! {
            biased;
            _ = wait_for_geometry_task_stop(&mut shutdown_rx) => break Ok(()),
            completed = computed_rx.recv(), if computing => {
                let Some((ticket, result)) = completed else {
                    break Err("surface map completion channel closed".into());
                };
                computing = false;
                explore.finish_surface_map_update(ticket, result).await;
            }
            frame = subscription.recv() => {
                // 接收时刻先于解码和MLS构建，计算耗时不能使旧图重新变新鲜。
                let received = crate::explore_runtime::now_monotonic_nanos();
                let frame = match frame {
                    Ok(frame) => frame,
                    Err(error) => break Err(format!("surface map receive failed: {error}")),
                };
                let message = match schema::decode_navigation_map_3d_body(&frame.payload) {
                    Ok(message) => message,
                    Err(error) => { warn!(%error, "invalid NavigationMap3D rejected by exploration"); continue; }
                };
                match explore.begin_surface_map_update(&message, received).await {
                    Ok(Some(ticket)) => latest_pending = Some((message, ticket)),
                    Ok(None) => latest_pending = None,
                    Err(error) => warn!(%error, "stale or foreign surface map rejected by exploration"),
                }
            }
        }
    };
    explore.stop_surface_input_owner().await;
    unsubscribe_owned_geometry_subscription("surface-map-consumer", subscription, terminal).await
}

async fn rollback_owned_geometry_subscriptions(
    mut subscriptions: Vec<(&'static str, nuwax_transport::OwnedLcmSubscription)>,
) -> Result<(), String> {
    let mut errors = Vec::new();
    while let Some((name, subscription)) = subscriptions.pop() {
        match subscription.unsubscribe().await {
            Ok(()) => {
                #[cfg(test)]
                record_topology_event(format!("unsubscribed:{name}"));
            }
            Err(error) => {
                errors.push(format!("{name} startup unsubscribe failed: {error}"));
            }
        }
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

/// 启动 geometry runtime 异步循环（LCM-first）。
///
/// backend 固定为 `LcmBackendKind::Real`。启动期调用
/// `LcmTransport::new_real(&lcm_url)`；若 liblcm 不可用或启动失败，
/// → fail-fast 返回 [`RealLcmBackendUnavailable`]；二进制入口统一输出错误并退出。
///
/// All channel, path, and frame inputs are immutable startup-preflight values;
/// this function never reparses their environment variables after side effects.
pub(crate) struct PreparedGeometryRuntime {
    backend_name: &'static str,
    sequence_store: Arc<SequenceStateStore>,
    nav_runtime: crate::nav_runtime::NavRuntimeState,
    transport: LcmTransport,
    backend_available: bool,
    #[cfg(test)]
    topology_control: Option<Arc<GeometryTopologyTestControl>>,
}

impl PreparedGeometryRuntime {
    pub(crate) fn runtime_generation(&self) -> u64 {
        self.nav_runtime.producer_runtime_generation()
    }

    pub(crate) fn transport(&self) -> LcmTransport {
        self.transport.clone()
    }
}

pub(crate) fn prepare_geometry_runtime(
    backend: LcmBackendKind,
) -> Result<PreparedGeometryRuntime, RealLcmBackendUnavailable> {
    let backend_name = backend.canonical_name();
    let sequence_state_path = sequence_state_path_from_env();
    let sequence_store = match SequenceStateStore::open(&sequence_state_path) {
        Ok(store) => Arc::new(store),
        Err(error) => {
            error!(path = %sequence_state_path.display(), %error, "geometry sequence state startup failed closed");
            return Err(RealLcmBackendUnavailable::persistence(format!(
                "geometry sequence state startup failed closed: path={} error={error}",
                sequence_state_path.display()
            )));
        }
    };
    let nav_runtime = match crate::nav_runtime::NavRuntimeState::try_from_sequence_store(
        sequence_store.as_ref(),
    ) {
        Ok(runtime) => runtime,
        Err(error) => {
            error!(path = %sequence_state_path.display(), %error, "GoalEnvelope producer epoch reservation failed closed");
            return Err(RealLcmBackendUnavailable::persistence(format!(
                "GoalEnvelope producer epoch reservation failed closed: path={} error={error}",
                sequence_state_path.display()
            )));
        }
    };
    let real_lcm_required = real_lcm_required_from_env();
    let (transport, backend_available): (LcmTransport, bool) = {
        let lcm_url = std::env::var("NUWAX_LCM_URL")
            .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=1".to_owned());
        #[cfg(feature = "real-lcm-bridge")]
        {
            match attempt_real_lcm_startup(real_lcm_required, || LcmTransport::new_real(&lcm_url)) {
                Ok(transport) => (transport, true),
                Err(RealLcmStartupError::Required(error)) => {
                    return Err(RealLcmBackendUnavailable::transport(format!(
                        "LcmTransport::new_real({lcm_url}) failed while \
                         NUWAX_REAL_LCM_REQUIRED=true: {error}"
                    )));
                }
                Err(RealLcmStartupError::Optional(error)) => {
                    return Err(RealLcmBackendUnavailable::transport(format!(
                        "LcmTransport::new_real({lcm_url}) failed: {error}"
                    )));
                }
            }
        }
        #[cfg(not(feature = "real-lcm-bridge"))]
        {
            let _ = lcm_url;
            return Err(RealLcmBackendUnavailable::transport(
                "binary built without real-lcm-bridge feature",
            ));
        }
    };
    Ok(PreparedGeometryRuntime {
        backend_name,
        sequence_store,
        nav_runtime,
        transport,
        backend_available,
        #[cfg(test)]
        topology_control: None,
    })
}

pub async fn spawn_geometry_runtime(
    prepared: PreparedGeometryRuntime,
    mapping_backend: MappingBackendSpec,
    channel_bindings: channel_registry::ValidatedChannelBindings,
    path_binding: (LcmTopic, LcmMessageType),
    frame_bindings: ValidatedFrameBindings,
) -> Result<SpawnedGeometryRuntime, GeometryRuntimeStartupFailure> {
    let PreparedGeometryRuntime {
        backend_name,
        sequence_store,
        nav_runtime,
        transport,
        backend_available,
        #[cfg(test)]
        topology_control,
    } = prepared;
    let (occ_topic, occ_msg_type) = channel_bindings.binding(ChannelKey::Occupancy);
    let (tf_topic, tf_msg_type) = channel_bindings.binding(ChannelKey::Tf);
    let legacy_cm_binding = Some(channel_bindings.binding(ChannelKey::CloudMap));
    let nav_binding = Some(channel_bindings.binding(ChannelKey::NavState));
    let fr_binding = Some(channel_bindings.binding(ChannelKey::Frontiers));
    let odom_binding = Some(channel_bindings.binding(ChannelKey::Odom));
    let (map_frame, odom_frame, base_frame) = frame_bindings.into_parts();
    let expected_map_frame: Arc<str> = Arc::from(map_frame.clone());
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavigationUpdate>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);
    let (map_revision_tx, _) = broadcast::channel::<MapRevisionEvent>(16);
    let (map_revision_latest_tx, _) = watch::channel::<Option<MapRevisionEvent>>(None);
    let committed_map_authority = Arc::new(std::sync::Mutex::new(None));
    let explore_projection_gate =
        Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default()));
    #[cfg(test)]
    let (task_control, task_registration_rx) =
        GeometryRuntimeTaskControl::channel_with_topology_control(topology_control);
    #[cfg(not(test))]
    let (task_control, task_registration_rx) = GeometryRuntimeTaskControl::channel();
    // Register the production fanout receiver before mapping can commit. A
    // tokio broadcast sender drops events when it has no receivers.
    let map_revision_fanout_rx = map_revision_tx.subscribe();

    let state = Arc::new(RwLock::new(SharedState::default()));
    let explore_runtime =
        crate::explore_runtime::ExploreRuntimeHandle::with_wavefront_map_frame(map_frame.clone())
            .map_err(|error| {
            RealLcmBackendUnavailable::configuration(format!(
                "exploration map frame rejected after startup validation: {error}"
            ))
        })?;
    if let Err(error) = explore_runtime.restore_latest_durable_reconciliation_snapshot() {
        error!(
            reason = error.message(),
            "exploration ownership journal startup recovery failed closed"
        );
        return Err(RealLcmBackendUnavailable::persistence(format!(
            "exploration ownership journal startup recovery failed closed: {}",
            error.message()
        ))
        .into());
    }
    let runtime_observation_generation = Arc::new(std::sync::atomic::AtomicU64::new(0));

    info!(
        "nuwax-geometry-runtime 启动：LCM backend = {} (available={})",
        backend_name, backend_available
    );

    // Phase 4 startup health wire-up:在启动日志中暴露 backend 选择，
    // 供 start 脚本 / smoke test / debug_panel 观测。
    info!(
        "LCM backend status (Phase 4 startup wire-up): lcm_backend={} available={}",
        backend_name, backend_available
    );

    // Every binding above was parsed and type-checked before Tokio, tracing,
    // persistence, transport, files, or tasks were created.
    // Phase 1:打印 channel registry 让运维和 healthcheck 都能看到本进程
    // 注册的全部 typed channel;Phase 2+ 将在此基础上挂接订阅任务。
    info!(
        "LCM channel registry: occupancy={} tf={} cloud_map={} nav_state={} frontiers={} odom={}",
        occ_topic.channel(),
        tf_topic.channel(),
        legacy_cm_binding
            .as_ref()
            .map(|(topic, _)| topic.channel())
            .unwrap_or("<none>"),
        nav_binding
            .as_ref()
            .map(|(topic, _)| topic.channel())
            .unwrap_or("<none>"),
        fr_binding
            .as_ref()
            .map(|(topic, _)| topic.channel())
            .unwrap_or("<none>"),
        odom_binding
            .as_ref()
            .map(|(topic, _)| topic.channel())
            .unwrap_or("<none>"),
    );
    info!(
        "LCM msg-type registry: occupancy={} tf={} cloud_map={} nav_state={} frontiers={} odom={}",
        occ_msg_type.fqn(),
        tf_msg_type.fqn(),
        legacy_cm_binding
            .as_ref()
            .map(|(_, kind)| kind.fqn())
            .unwrap_or("<none>"),
        nav_binding
            .as_ref()
            .map(|(_, kind)| kind.fqn())
            .unwrap_or("<none>"),
        fr_binding
            .as_ref()
            .map(|(_, kind)| kind.fqn())
            .unwrap_or("<none>"),
        odom_binding
            .as_ref()
            .map(|(_, kind)| kind.fqn())
            .unwrap_or("<none>"),
    );

    // ---- 订阅 /tf：只观测本帧 TF chain，不写 canonical SharedState ----

    // Mapping exists for the process lifetime but starts inactive. The
    // StartMapping RPC is the only production transition that opens ingestion.
    let mapping_adapter = if backend_available {
        let cfg = MappingAdapterConfig::from_env();
        let forbidden_zones_path = crate::forbidden_zones::path_from_env();
        info!(
            "mapping_adapter 已构造且等待 StartMapping: backend={} resolution={}m auto_bounds_padding=1m occupancy_algo={} forbidden_zones_path={}",
            mapping_backend.stable_name(),
            cfg.resolution,
            cfg.occupancy_algo.as_dimos_str(),
            forbidden_zones_path.display()
        );
        Some(Arc::new(parking_lot::Mutex::new(
            MappingAdapter::new_with_frames_sequence_store_and_forbidden_zones(
                cfg,
                mapping_backend,
                map_frame.clone(),
                odom_frame.clone(),
                sequence_store.clone(),
                forbidden_zones_path,
            ),
        )))
    } else {
        None
    };

    let mut owned_subscriptions = Vec::with_capacity(3);
    let surface_config = if nav_runtime.configured_profile() == crate::nav_runtime::PROFILE_MLS3D {
        Some(nuwax_nav_stack::surface_config::SurfaceNavigationConfig::from_env()
            .map_err(|error| error.to_string()).and_then(|config| {
                if config.geometry.world_frame == map_frame { Ok(config) }
                else { Err("surface navigation config does not match Geometry map frame".into()) }
            }))
    } else { None };
    let tf_subscription = match transport
        .subscribe_owned(tf_topic.clone(), Some(tf_msg_type.clone()))
        .await
    {
        Ok(subscription) => subscription,
        Err(error) => {
            return Err(RealLcmBackendUnavailable::transport(format!(
                "public TF subscribe acknowledgement failed: {error}"
            ))
            .into());
        }
    };
    owned_subscriptions.push(("public-tf-observer", tf_subscription));

    let (path_topic, path_message_type) = path_binding;
    let path_subscription = match transport
        .subscribe_owned(path_topic, Some(path_message_type))
        .await
    {
        Ok(subscription) => subscription,
        Err(error) => {
            let rollback = rollback_owned_geometry_subscriptions(owned_subscriptions).await;
            return Err(RealLcmBackendUnavailable::transport(format!(
                "PathEnvelope subscribe acknowledgement failed: {error}; rollback={rollback:?}"
            ))
            .into());
        }
    };
    owned_subscriptions.push(("path-envelope-consumer", path_subscription));

    let (nav_topic, nav_message_type) = nav_binding
        .clone()
        .expect("validated navigation-state binding must exist");
    let nav_subscription = match transport
        .subscribe_owned(nav_topic, Some(nav_message_type))
        .await
    {
        Ok(subscription) => subscription,
        Err(error) => {
            let rollback = rollback_owned_geometry_subscriptions(owned_subscriptions).await;
            return Err(RealLcmBackendUnavailable::transport(format!(
                "navigation-state subscribe acknowledgement failed: {error}; rollback={rollback:?}"
            ))
            .into());
        }
    };
    owned_subscriptions.push(("navigation-state-consumer", nav_subscription));

    if surface_config.as_ref().is_some_and(Result::is_ok) {
        let surface_subscription = transport.subscribe_owned(
            LcmTopic::new("/nav/map3d").expect("固定三维地图通道"),
            Some(LcmMessageType::new("nuwax_msgs.NavigationMap3D").expect("三维地图消息类型")),
        ).await;
        match surface_subscription {
            Ok(subscription) => owned_subscriptions.insert(0, ("surface-map-consumer", subscription)),
            Err(error) => {
                let rollback = rollback_owned_geometry_subscriptions(owned_subscriptions).await;
                return Err(RealLcmBackendUnavailable::transport(format!(
                    "surface-map subscribe acknowledgement failed: {error}; rollback={rollback:?}"
                )).into());
            }
        }
    }

    if legacy_cm_binding.is_some() {
        info!("legacy /cloud_map input is ignored; MappingSession owns cloud-map SHM revisions");
    }

    // Product mmap creation is deliberately after all owned subscribe ACKs.
    // Subscription rollback therefore cannot leave an unowned mmap behind.
    let cm_shm_path = crate::pointcloud::cloud_map_shm_path();
    let cm_writer = match CloudMapShmWriter::new(&cm_shm_path) {
        Ok(writer) => Some(writer),
        Err(error) => {
            warn!("累积地图 mmap 初始化失败（降级为无 cloud_map 模式）: {error}");
            None
        }
    };
    let cm_writer = Arc::new(parking_lot::Mutex::new(cm_writer));

    // Phase 3 增强:订阅 /odom 和 /go2/lidar/points 时同步把数据喂给 mapping
    // adapter。`feed_pose` / `feed_pointcloud` 内部持 parking_lot 锁同步,
    // 不会跨 await。
    // Phase 3: 构造 occupancy grid SHM sidecar(默认 /tmp/nuwax_occupancy.mmap)。
    // 与 cloud_map SHM 区分:occupancy 走 grid layout；cloud_map 由
    // MappingSession 接受的 map-frame 新帧增量更新独立 voxel accumulator。
    let occ_shm_path = std::path::PathBuf::from(
        std::env::var("NUWAX_OCCUPANCY_SHM_PATH")
            .unwrap_or_else(|_| "/tmp/nuwax_occupancy.mmap".into()),
    );
    let occupancy_shm = if backend_available {
        match OccupancyGridShmWriter::new(&occ_shm_path) {
            Ok(w) => {
                info!("occupancy grid SHM 已就绪: {:?}", occ_shm_path);
                Some(Arc::new(parking_lot::Mutex::new(w)))
            }
            Err(e) => {
                warn!("occupancy grid SHM 初始化失败 (降级为无 SHM): {}", e);
                None
            }
        }
    } else {
        None
    };
    let occupancy_shm_path = occupancy_shm
        .as_ref()
        .map(|_| occ_shm_path.to_string_lossy().into_owned());
    let occupancy_shm_current = Arc::new(RwLock::new(None));

    let cmd_vel_sink_ready = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let explore_scheduler = Arc::new(tokio::sync::Mutex::new(None));
    let mapping_lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
    let handle = GeometryRuntimeHandle {
        state: state.clone(),
        cloud_map_tx: cloud_map_tx.clone(),
        cloud_map_writer: cm_writer.clone(),
        nav_state_tx: nav_state_tx.clone(),
        frontiers_tx: frontiers_tx.clone(),
        map_revision_tx: map_revision_tx.clone(),
        map_revision_latest_tx: map_revision_latest_tx.clone(),
        explore_projection_gate: explore_projection_gate.clone(),
        committed_map_authority: committed_map_authority.clone(),
        transport: transport.clone(),
        transport_backend_name: backend_name,
        lcm_backend_available: backend_available,
        // Phase 6 / Rework-9: 默认 `false` —— 这是**生产 spawn 路径**,
        // 该路径必须由 Python `LcmMotionCommandBridge.start()` 成功完成后,
        // 通过 gateway `_start_motion_command_bridge` 链路真实写回 true;
        // 若 Python 端未启动 / start 失败 / providers 不实现 MotionControl,
        // 该 flag 保持 false,`SetNavigationGoal` 在 `compute_readiness` 因
        // `cmd_vel_sink_ready=false` 被拒绝(参见 `REJECT_REASON_CMD_VEL_SINK_DOWN`)。
        //
        // 注意:`build_handle_for_tests`(第 1354 行 test helper profile)
        // 仍保留默认 `true` —— 单测直接调 `set_cmd_vel_sink_ready(false)`
        // 验证 readiness 拒绝分支(参照 `phase6_handle_cmd_vel_sink_ready_setter_propagates_to_readiness`)。
        cmd_vel_sink_ready: cmd_vel_sink_ready.clone(),
        nav: nav_runtime.clone(),
        explore: explore_runtime.clone(),
        mapping_adapter: mapping_adapter.clone(),
        mapping_lifecycle_lock: mapping_lifecycle_lock.clone(),
        occupancy_shm: occupancy_shm.clone(),
        occupancy_shm_path: occupancy_shm_path.clone(),
        occupancy_shm_current: occupancy_shm_current.clone(),
        explore_scheduler: explore_scheduler.clone(),
        runtime_observation_generation: runtime_observation_generation.clone(),
        task_control: Some(task_control.clone()),
    };

    if let Some(config) = surface_config.as_ref() {
        if let Err(error) = config { warn!(%error, "surface exploration configuration unavailable; mapping remains enabled"); }
        let boot_nonce = mapping_adapter.as_ref().map(|mapping| mapping.lock().mapping_commit_boot_nonce().to_be_bytes())
            .unwrap_or([0; 16]);
        explore_runtime.configure_surface_source(config.clone(), nav_runtime.producer_runtime_generation(), boot_nonce)
            .await.expect("新建探索拥有方尚未接收用户会话");
    }

    let mut task_owner = Some(GeometryRuntimeTaskOwner::start(
        task_control.clone(),
        task_registration_rx,
    ));
    let mut pending_children = Vec::new();
    let (nav_subscription_name, nav_subscription) = owned_subscriptions
        .pop()
        .expect("navigation-state subscription was acknowledged");
    debug_assert_eq!(nav_subscription_name, "navigation-state-consumer");
    let nav_shutdown_rx = task_control.shutdown_receiver();
    register_geometry_startup_persistent(
        &task_control,
        &mut task_owner,
        &handle,
        &mut owned_subscriptions,
        &mut pending_children,
        "navigation-state-consumer",
        run_owned_nav_state_consumer(
            nav_subscription,
            state.clone(),
            nav_state_tx.clone(),
            nav_runtime.clone(),
            explore_runtime.clone(),
            nav_shutdown_rx,
        ),
    )
    .await?;
    let (path_subscription_name, path_subscription) = owned_subscriptions
        .pop()
        .expect("PathEnvelope subscription was acknowledged");
    debug_assert_eq!(path_subscription_name, "path-envelope-consumer");
    let path_shutdown_rx = task_control.shutdown_receiver();
    register_geometry_startup_persistent(
        &task_control,
        &mut task_owner,
        &handle,
        &mut owned_subscriptions,
        &mut pending_children,
        "path-envelope-consumer",
        run_owned_path_envelope_consumer(path_subscription, nav_runtime.clone(), path_shutdown_rx),
    )
    .await?;
    let (tf_subscription_name, tf_subscription) = owned_subscriptions
        .pop()
        .expect("public TF subscription was acknowledged");
    debug_assert_eq!(tf_subscription_name, "public-tf-observer");
    let tf_shutdown_rx = task_control.shutdown_receiver();
    register_geometry_startup_persistent(
        &task_control,
        &mut task_owner,
        &handle,
        &mut owned_subscriptions,
        &mut pending_children,
        "public-tf-observer",
        run_owned_public_tf_observer(
            tf_subscription,
            map_frame.clone(),
            odom_frame.clone(),
            base_frame.clone(),
            nav_runtime.clone(),
            tf_shutdown_rx,
        ),
    )
    .await?;

    if let Some(Ok(config)) = surface_config {
        let (name, subscription) = owned_subscriptions.pop().expect("三维地图订阅已经确认");
        debug_assert_eq!(name, "surface-map-consumer");
        register_geometry_startup_persistent(
            &task_control, &mut task_owner, &handle, &mut owned_subscriptions, &mut pending_children,
            "surface-map-consumer",
            run_owned_surface_map_consumer(subscription, explore_runtime.clone(), config, task_control.clone(),
                task_control.shutdown_request_receiver()),
        ).await?;
    }

    let cancel_shutdown_rx = task_control.shutdown_receiver();
    register_geometry_startup_persistent(
        &task_control,
        &mut task_owner,
        &handle,
        &mut owned_subscriptions,
        &mut pending_children,
        "cancel-timeout-maintenance",
        run_persistent_future_until_stop(
            cancel_shutdown_rx,
            cancel_timeout_future(
                nav_runtime.clone(),
                state.clone(),
                nav_state_tx.clone(),
                explore_runtime.clone(),
            ),
        ),
    )
    .await?;
    let frontier_shutdown_rx = task_control.shutdown_receiver();
    register_geometry_startup_persistent(
        &task_control,
        &mut task_owner,
        &handle,
        &mut owned_subscriptions,
        &mut pending_children,
        "frontier-fanout",
        run_persistent_future_until_stop(
            frontier_shutdown_rx,
            frontier_fanout_future_in_frame(
                explore_runtime.clone(),
                state.clone(),
                frontiers_tx.clone(),
                transport.clone(),
                fr_binding,
                expected_map_frame.clone(),
            ),
        ),
    )
    .await?;

    explore_runtime.set_navigation_sink(Arc::new({
        let bridge = ExploreNavigationBridge {
            state: state.clone(),
            expected_map_frame: expected_map_frame.clone(),
            cloud_map_tx,
            nav_state_tx,
            frontiers_tx,
            map_revision_tx: map_revision_tx.clone(),
            map_revision_latest_tx: map_revision_latest_tx.clone(),
            explore_projection_gate: explore_projection_gate.clone(),
            committed_map_authority: committed_map_authority.clone(),
            mapping_lifecycle_lock: mapping_lifecycle_lock.clone(),
            transport: transport.clone(),
            transport_backend_name: backend_name,
            lcm_backend_available: backend_available,
            cmd_vel_sink_ready,
            nav: handle.nav.clone(),
            exploration_producer_epoch: explore_runtime.producer_epoch_projection(),
            runtime_observation_generation,
            orphan_cancellations: explore_runtime.orphan_cancellation_registry(),
            #[cfg(not(test))]
            task_control: Some(task_control.clone()),
        };
        #[cfg(test)]
        bridge.set_geometry_task_control_for_tests(Some(task_control.clone()));
        bridge
    }));
    let scheduler_period_ms = std::env::var("NUWAX_EXPLORE_SCHEDULER_PERIOD_MS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .unwrap_or(200);
    let pose_ttl_ms = std::env::var("NUWAX_EXPLORE_POSE_TTL_MS")
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .unwrap_or(1_000);
    let scheduler_runtime = explore_runtime.clone();
    let scheduler_slot = explore_scheduler.clone();
    let scheduler_pose_provider = Arc::new(SharedStatePoseProvider {
        state: state.clone(),
        expected_map_frame: expected_map_frame.clone(),
        freshness: Arc::new(parking_lot::Mutex::new(PoseFreshnessTracker::default())),
        max_receipt_age: Duration::from_millis(pose_ttl_ms),
    });
    let scheduler_period = Duration::from_millis(scheduler_period_ms);
    let scheduler_shutdown_rx = task_control.shutdown_receiver();
    register_geometry_startup_persistent(
        &task_control,
        &mut task_owner,
        &handle,
        &mut owned_subscriptions,
        &mut pending_children,
        "exploration-scheduler",
        async move {
            let scheduler =
                scheduler_runtime.spawn_scheduler(scheduler_pose_provider, scheduler_period);
            *scheduler_slot.lock().await = Some(scheduler.clone());
            run_explore_scheduler_persistent(scheduler, scheduler_shutdown_rx).await
        },
    )
    .await?;

    let fanout_mapping_adapter = mapping_adapter
        .as_ref()
        .expect("available LCM backend must construct mapping adapter")
        .clone();
    let map_revision_tasks = map_revision_fanout_futures_in_frame(
        map_revision_fanout_rx,
        map_revision_latest_tx,
        committed_map_authority,
        explore_projection_gate,
        transport.clone(),
        occ_topic,
        occ_msg_type,
        occupancy_shm,
        occupancy_shm_path,
        occupancy_shm_current,
        explore_runtime,
        fanout_mapping_adapter,
        mapping_lifecycle_lock,
        expected_map_frame,
    );
    for (name, future) in [
        "map-revision-coalescer",
        "occupancy-lcm-fanout",
        "exploration-costmap-fanout",
    ]
    .into_iter()
    .zip(map_revision_tasks)
    {
        let shutdown_rx = task_control.shutdown_receiver();
        register_geometry_startup_persistent(
            &task_control,
            &mut task_owner,
            &handle,
            &mut owned_subscriptions,
            &mut pending_children,
            name,
            run_persistent_future_until_stop(shutdown_rx, future),
        )
        .await?;
    }
    if let Some(mapping_arc) = handle.mapping_adapter.as_ref() {
        let shutdown_rx = task_control.shutdown_receiver();
        register_geometry_startup_persistent(
            &task_control,
            &mut task_owner,
            &handle,
            &mut owned_subscriptions,
            &mut pending_children,
            "forbidden-zone-watcher",
            run_persistent_future_until_stop(
                shutdown_rx,
                forbidden_zone_watcher_future(
                    handle.state.clone(),
                    mapping_arc.clone(),
                    handle.map_revision_tx.clone(),
                    handle.mapping_lifecycle_lock.clone(),
                ),
            ),
        )
        .await?;
    }
    debug_assert!(owned_subscriptions.is_empty());
    debug_assert!(pending_children.is_empty());
    tokio::task::yield_now().await;
    if let Some(failure) = task_owner
        .as_ref()
        .and_then(GeometryRuntimeTaskOwner::completed_failure)
    {
        let outcome = task_owner
            .as_mut()
            .expect("startup failure retains task owner")
            .shutdown()
            .await;
        if !outcome.producers_settled() {
            let mut detail = format!("Geometry startup task failed before ready: {failure}");
            if let Err(error) = outcome.result {
                detail.push_str(&format!("; task rollback failed: {error}"));
            }
            detail
                .push_str("; Geometry startup producers remained unsettled; mmap cleanup withheld");
            let task_owner = task_owner
                .take()
                .expect("unsettled startup failure retains task owner");
            return Err(GeometryRuntimeStartupFailure::with_rollback_owner(
                RealLcmBackendUnavailable::transport(detail),
                handle,
                task_owner,
            ));
        }
        task_owner.take();
        let cleanup = if outcome.producers_settled() {
            cleanup_geometry_startup_files(&handle)
        } else {
            Err(
                "Geometry startup task failure remained unsettled; mmap cleanup withheld"
                    .to_owned(),
            )
        };
        let mut detail = format!("Geometry startup task failed before ready: {failure}");
        if let Err(error) = outcome.result {
            detail.push_str(&format!("; task rollback failed: {error}"));
        }
        if let Err(error) = cleanup {
            detail.push_str(&format!("; {error}"));
        }
        return Err(RealLcmBackendUnavailable::transport(detail).into());
    }
    let task_owner = task_owner.expect("successful startup retains the root task owner");
    Ok(SpawnedGeometryRuntime { handle, task_owner })
}

#[derive(Debug)]
struct MappingIngestOutcome {
    sensor_origin_map: Option<[f64; 3]>,
    accepted_map_points: Vec<nalgebra::Point3<f64>>,
    occupancy_event: Option<MapRevisionEvent>,
    occupancy_receipt: Option<crate::mapping_commit::OccupancyCommitReceipt>,
    #[cfg(test)]
    occupancy_committed: bool,
    session_generation: u64,
    source_timestamp_ns: u64,
}

struct MappingIngestCommit {
    outcome: MappingIngestOutcome,
    cloud_commit: Option<CommittedMappingCloudMap>,
}

#[cfg(test)]
fn ingest_mapping_points(
    adapter: &mut MappingAdapter,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    now_ns: u64,
    frame_id: &str,
    source_timestamp_ns: u64,
    state: &Arc<RwLock<SharedState>>,
    map_revision_tx: &broadcast::Sender<MapRevisionEvent>,
) -> Result<MappingIngestOutcome, nuwax_mapping::SessionError> {
    let odom_xy = (adapter.last_sensor_origin[0], adapter.last_sensor_origin[1]);
    let stats = adapter.feed_pointcloud_session(
        points,
        odom_xy,
        point_step,
        now_ns,
        frame_id,
        source_timestamp_ns,
    )?;
    if stats.accepted == 0 {
        return Ok(MappingIngestOutcome {
            sensor_origin_map: None,
            accepted_map_points: Vec::new(),
            occupancy_event: None,
            occupancy_receipt: None,
            occupancy_committed: false,
            session_generation: adapter.snapshot().session_generation,
            source_timestamp_ns,
        });
    }
    let accepted_map_points = adapter.take_latest_accepted_map_points();
    let session_generation = adapter.snapshot().session_generation;
    let mut revision_rx = map_revision_tx.subscribe();
    let occupancy_committed = adapter.build_and_publish(state, Some(map_revision_tx));
    let occupancy_event = occupancy_committed
        .then(|| revision_rx.try_recv().ok())
        .flatten();
    let occupancy_receipt = occupancy_event
        .as_ref()
        .and_then(|_| adapter.latest_occupancy_commit_receipt());
    Ok(MappingIngestOutcome {
        sensor_origin_map: None,
        accepted_map_points,
        occupancy_committed: occupancy_event.is_some(),
        occupancy_event,
        occupancy_receipt,
        session_generation,
        source_timestamp_ns,
    })
}

pub(crate) struct CommittedMappingCloudMap {
    pub(crate) notification: CloudMapNotification,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) xyz_f32_le: Vec<u8>,
    pub(crate) receipt: crate::mapping_commit::CloudCommitReceipt,
}

async fn ingest_mapping_points_with_backend(
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    explore_projection_gate: &Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    committed_map_authority: &Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    map_revision_latest_tx: &watch::Sender<Option<MapRevisionEvent>>,
    cloud_map_writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    received_at_ns: u64,
    frame_id: &str,
    source_timestamp_ns: u64,
    state: &Arc<RwLock<SharedState>>,
    source_fenced: bool,
    expected_generations: Option<(u64, u64)>,
    expected_relocalization_epoch: Option<u64>,
    relocalization_mode: Option<Arc<crate::correction_coordinator::RelocalizationModeFence>>,
    task_control: Option<&GeometryRuntimeTaskControl>,
    sensor_origin_map: Option<[f64; 3]>,
) -> Result<MappingIngestCommit, nuwax_mapping::SessionError> {
    let owned_mapping = mapping.clone();
    let owned_lifecycle_lock = lifecycle_lock.clone();
    let owned_explore_projection_gate = explore_projection_gate.clone();
    let owned_committed_map_authority = committed_map_authority.clone();
    let owned_map_revision_latest_tx = map_revision_latest_tx.clone();
    let owned_cloud_map_writer = cloud_map_writer.clone();
    let owned_points = points.to_vec();
    let owned_frame_id = frame_id.to_owned();
    let owned_state = state.clone();
    let owned_relocalization_mode = relocalization_mode.clone();
    let target_frame_id = mapping.lock().configured_map_frame_id().to_owned();
    let (result_tx, result_rx) = tokio::sync::oneshot::channel();
    let task = async move {
        let result = ingest_mapping_points_with_backend_owned(
            &owned_mapping,
            &owned_lifecycle_lock,
            &owned_explore_projection_gate,
            &owned_committed_map_authority,
            &owned_map_revision_latest_tx,
            &owned_cloud_map_writer,
            &owned_points,
            point_step,
            received_at_ns,
            &owned_frame_id,
            source_timestamp_ns,
            &owned_state,
            source_fenced,
            expected_generations,
            expected_relocalization_epoch,
            owned_relocalization_mode,
            sensor_origin_map,
        )
        .await;
        let terminal = result
            .as_ref()
            .map(|_| ())
            .map_err(|error| error.to_string());
        let _ = result_tx.send(result);
        terminal
    };
    if let Some(control) = task_control {
        control
            .register_retained("mapping-ingest", task)
            .map_err(
                |error| nuwax_mapping::SessionError::FrameTransformUnavailable {
                    source_frame_id: frame_id.to_owned(),
                    target_frame_id: target_frame_id.clone(),
                    reason: error,
                },
            )?;
    } else {
        tokio::spawn(task);
    }
    result_rx.await.map_err(
        |error| nuwax_mapping::SessionError::FrameTransformUnavailable {
            source_frame_id: frame_id.to_owned(),
            target_frame_id,
            reason: format!("owned mapping backend task result was unavailable: {error}"),
        },
    )?
}

async fn ingest_mapping_points_with_backend_owned(
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    explore_projection_gate: &Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    committed_map_authority: &Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    map_revision_latest_tx: &watch::Sender<Option<MapRevisionEvent>>,
    cloud_map_writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    received_at_ns: u64,
    frame_id: &str,
    source_timestamp_ns: u64,
    state: &Arc<RwLock<SharedState>>,
    source_fenced: bool,
    expected_generations: Option<(u64, u64)>,
    expected_relocalization_epoch: Option<u64>,
    relocalization_mode: Option<Arc<crate::correction_coordinator::RelocalizationModeFence>>,
    sensor_origin_map: Option<[f64; 3]>,
) -> Result<MappingIngestCommit, nuwax_mapping::SessionError> {
    let prepared = {
        let _lifecycle_guard = lifecycle_lock.lock().await;
        let mut adapter = mapping.lock();
        if !adapter.is_active() {
            return Ok(MappingIngestCommit {
                outcome: MappingIngestOutcome {
                    sensor_origin_map: None,
                    accepted_map_points: Vec::new(),
                    occupancy_event: None,
                    occupancy_receipt: None,
                    #[cfg(test)]
                    occupancy_committed: false,
                    session_generation: adapter.snapshot().session_generation,
                    source_timestamp_ns,
                },
                cloud_commit: None,
            });
        }
        if let Some((expected_session_generation, expected_frame_generation)) = expected_generations
        {
            if adapter.snapshot().session_generation != expected_session_generation
                || adapter.global_frame_generation() != expected_frame_generation
            {
                return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                    source_frame_id: frame_id.to_owned(),
                    target_frame_id: adapter.configured_map_frame_id().to_owned(),
                    reason: "registration permit mapping generation is stale".to_owned(),
                });
            }
        }
        let odom_xy = (adapter.last_sensor_origin[0], adapter.last_sensor_origin[1]);
        adapter.prepare_backend_compute(
            points,
            odom_xy,
            point_step,
            received_at_ns,
            frame_id,
            source_timestamp_ns,
            source_fenced,
        )?
    };
    let Some(prepared) = prepared else {
        let adapter = mapping.lock();
        return Ok(MappingIngestCommit {
            outcome: MappingIngestOutcome {
                sensor_origin_map: None,
                accepted_map_points: Vec::new(),
                occupancy_event: None,
                occupancy_receipt: None,
                #[cfg(test)]
                occupancy_committed: false,
                session_generation: adapter.snapshot().session_generation,
                source_timestamp_ns,
            },
            cloud_commit: None,
        });
    };
    let token = prepared.token.clone();
    let completion_rx = match prepared.worker.submit(prepared.token, prepared.input) {
        Ok(receiver) => receiver,
        Err(error) => {
            let target_frame_id = mapping.lock().configured_map_frame_id().to_owned();
            let retirement_error = fence_failed_mapping_backend(
                mapping,
                lifecycle_lock,
                explore_projection_gate,
                committed_map_authority,
                map_revision_latest_tx,
                &token.session_id,
            )
            .await
            .err();
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: frame_id.to_owned(),
                target_frame_id,
                reason: match retirement_error {
                    Some(retirement_error) => format!(
                        "mapping backend submission failed: {error:?}; retirement failed: {retirement_error}"
                    ),
                    None => format!("mapping backend submission failed: {error:?}"),
                },
            });
        }
    };
    let completion = match tokio::time::timeout(Duration::from_secs(5), completion_rx).await {
        Ok(Ok(completion)) => completion,
        Ok(Err(_)) | Err(_) => {
            let target_frame_id = mapping.lock().configured_map_frame_id().to_owned();
            let retirement_error = fence_failed_mapping_backend(
                mapping,
                lifecycle_lock,
                explore_projection_gate,
                committed_map_authority,
                map_revision_latest_tx,
                &token.session_id,
            )
            .await
            .err();
            return Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
                source_frame_id: frame_id.to_owned(),
                target_frame_id,
                reason: match retirement_error {
                    Some(error) => format!(
                        "mapping backend completion timed out or closed; retirement failed: {error}"
                    ),
                    None => "mapping backend completion timed out or closed".to_owned(),
                },
            });
        }
    };
    let commit_result: Result<
        MappingIngestCommit,
        (
            String,
            String,
            Option<crate::mapping_adapter::MappingBackendRetirement>,
        ),
    > = {
        let mut projection_guard = explore_projection_gate.clone().lock_owned().await;
        let _lifecycle_guard = lifecycle_lock.lock().await;
        let mut shared_guard = Some(state.write().await);
        let mut adapter = mapping.lock();
        let target_frame_id = adapter.configured_map_frame_id().to_owned();
        let mut completion = Some(completion);
        let fence_violation = match (expected_relocalization_epoch, relocalization_mode.as_ref()) {
            (Some(expected_epoch), Some(mode)) => {
                let snapshot = mode.snapshot();
                snapshot.active || snapshot.epoch != expected_epoch
            }
            _ => false,
        };
        let fence_commit = if fence_violation {
            drop(shared_guard.take());
            let completion = completion
                .take()
                .expect("mapping completion is present before relocalization fence");
            let token = completion.token.clone();
            let resolution = completion.resolution;
            match resolution.abort() {
                Err(error) => Some(Err((
                    target_frame_id.clone(),
                    format!("mapping completion abort failed across relocalization fence: {error}"),
                    None,
                ))),
                Ok(()) if !adapter.fail_backend_compute(&token) => Some(Err((
                    target_frame_id.clone(),
                    "mapping completion became stale while aborting across relocalization fence"
                        .to_owned(),
                    None,
                ))),
                Ok(()) => {
                    let session_generation = adapter.snapshot().session_generation;
                    Some(Ok(MappingIngestCommit {
                        outcome: MappingIngestOutcome {
                            sensor_origin_map: None,
                            accepted_map_points: Vec::new(),
                            occupancy_event: None,
                            occupancy_receipt: None,
                            #[cfg(test)]
                            occupancy_committed: false,
                            session_generation,
                            source_timestamp_ns,
                        },
                        cloud_commit: None,
                    }))
                }
            }
        } else {
            None
        };
        let mut commit_result = if let Some(fence_commit) = fence_commit {
            fence_commit
        } else {
            let completion_result = adapter.commit_backend_completion(
                completion.expect("mapping completion is present without a relocalization fence"),
                &mut *shared_guard
                    .as_mut()
                    .expect("shared state guard is present before mapping commit"),
            );
            drop(shared_guard.take());
            match completion_result {
                Ok(Some(committed)) => {
                    let crate::mapping_adapter::CommittedMappingCandidate {
                        token: committed_token,
                        resolution,
                        accepted_map_points,
                        event,
                        session_generation,
                        source_timestamp_ns: committed_timestamp_ns,
                        occupancy_receipt,
                        ..
                    } = committed;
                    let outcome = MappingIngestOutcome {
                        sensor_origin_map,
                        accepted_map_points,
                        #[cfg(test)]
                        occupancy_committed: event.is_some(),
                        occupancy_event: event,
                        occupancy_receipt,
                        session_generation,
                        source_timestamp_ns: committed_timestamp_ns,
                    };
                    let current_session = adapter.snapshot();
                    drop(adapter);
                    let cloud_acceptance = update_mapping_cloud_map_with_acceptance(
                        &outcome,
                        &current_session,
                        cloud_map_writer,
                    );
                    let accepted = cloud_acceptance.accepted;
                    let mut adapter = mapping.lock();
                    let resolution_result =
                        adapter.resolve_backend_completion(&committed_token, resolution, accepted);
                    if !accepted || resolution_result.is_err() {
                        let reason = if !accepted {
                            "canonical cloud-map acceptance failed".to_owned()
                        } else {
                            format!(
                                "mapping backend completion resolution failed: {}",
                                resolution_result.expect_err("resolution failure checked above")
                            )
                        };
                        let retirement = adapter
                            .stop_session_detached(&token.session_id)
                            .ok()
                            .and_then(|transition| transition.retirement);
                        Err((target_frame_id, reason, retirement))
                    } else {
                        Ok(MappingIngestCommit {
                            outcome,
                            cloud_commit: cloud_acceptance.cloud_commit,
                        })
                    }
                }
                Ok(None) => {
                    let session_generation = adapter.snapshot().session_generation;
                    Ok(MappingIngestCommit {
                        outcome: MappingIngestOutcome {
                            sensor_origin_map: None,
                            accepted_map_points: Vec::new(),
                            occupancy_event: None,
                            occupancy_receipt: None,
                            #[cfg(test)]
                            occupancy_committed: false,
                            session_generation,
                            source_timestamp_ns,
                        },
                        cloud_commit: None,
                    })
                }
                Err(reason) => {
                    let retirement = adapter
                        .stop_session_detached(&token.session_id)
                        .ok()
                        .and_then(|transition| transition.retirement);
                    Err((target_frame_id, reason, retirement))
                }
            }
        };
        if commit_result
            .as_ref()
            .is_err_and(|(_, _, retirement)| retirement.is_some())
        {
            projection_guard.pending_latest = None;
            if let Err(error) = GeometryRuntimeHandle::revoke_committed_map_projection_parts_locked(
                committed_map_authority,
                map_revision_latest_tx,
            ) {
                if let Err((_, reason, _)) = &mut commit_result {
                    *reason = format!("{reason}; authority revoke failed: {error}");
                }
            }
        }
        commit_result
    };
    match commit_result {
        Ok(commit) => Ok(commit),
        Err((target_frame_id, reason, retirement)) => {
            finish_mapping_ingest_failure(
                mapping,
                lifecycle_lock,
                frame_id,
                target_frame_id,
                reason,
                retirement,
            )
            .await
        }
    }
}

async fn finish_mapping_ingest_failure(
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    frame_id: &str,
    target_frame_id: String,
    mut reason: String,
    retirement: Option<crate::mapping_adapter::MappingBackendRetirement>,
) -> Result<MappingIngestCommit, nuwax_mapping::SessionError> {
    if let Err(error) = finish_backend_retirement(mapping, lifecycle_lock, retirement).await {
        reason = format!("{reason}; retirement failed: {error}");
    }
    Err(nuwax_mapping::SessionError::FrameTransformUnavailable {
        source_frame_id: frame_id.to_owned(),
        target_frame_id,
        reason,
    })
}

async fn fence_failed_mapping_backend(
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    explore_projection_gate: &Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    committed_map_authority: &Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    map_revision_latest_tx: &watch::Sender<Option<MapRevisionEvent>>,
    session_id: &str,
) -> Result<(), String> {
    let retirement = {
        let mut projection_guard = explore_projection_gate.clone().lock_owned().await;
        let _lifecycle_guard = lifecycle_lock.lock().await;
        let mut adapter = mapping.lock();
        let snapshot = adapter.snapshot();
        if !snapshot.active || snapshot.session_id != session_id {
            return Ok(());
        }
        let retirement = adapter
            .stop_session_detached(session_id)
            .map_err(|error| error.to_string())?
            .retirement;
        drop(adapter);
        projection_guard.pending_latest = None;
        GeometryRuntimeHandle::revoke_committed_map_projection_parts_locked(
            committed_map_authority,
            map_revision_latest_tx,
        )?;
        retirement
    };
    finish_backend_retirement(mapping, lifecycle_lock, retirement)
        .await
        .map_err(|error| error.to_string())
}

fn update_mapping_cloud_map_if_current(
    outcome: &MappingIngestOutcome,
    current_session: &crate::mapping_adapter::MappingSessionSnapshot,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
) -> Option<CommittedMappingCloudMap> {
    update_mapping_cloud_map_with_acceptance(outcome, current_session, writer).cloud_commit
}

pub(crate) fn accept_mapping_cloud_map_for_backend_candidate(
    accepted_map_points: &[nalgebra::Point3<f64>],
    event: &Option<MapRevisionEvent>,
    occupancy_receipt: &Option<crate::mapping_commit::OccupancyCommitReceipt>,
    session_generation: u64,
    source_timestamp_ns: u64,
    current_session: &crate::mapping_adapter::MappingSessionSnapshot,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
) -> (bool, Option<CommittedMappingCloudMap>) {
    let outcome = MappingIngestOutcome {
        sensor_origin_map: None,
        accepted_map_points: accepted_map_points.to_vec(),
        occupancy_event: event.clone(),
        occupancy_receipt: occupancy_receipt.clone(),
        #[cfg(test)]
        occupancy_committed: event.is_some(),
        session_generation,
        source_timestamp_ns,
    };
    let acceptance = update_mapping_cloud_map_with_acceptance(&outcome, current_session, writer);
    (acceptance.accepted, acceptance.cloud_commit)
}

struct MappingCloudMapAcceptance {
    cloud_commit: Option<CommittedMappingCloudMap>,
    accepted: bool,
}

fn update_mapping_cloud_map_with_acceptance(
    outcome: &MappingIngestOutcome,
    current_session: &crate::mapping_adapter::MappingSessionSnapshot,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
) -> MappingCloudMapAcceptance {
    if !current_session.active || current_session.session_generation != outcome.session_generation {
        return MappingCloudMapAcceptance {
            cloud_commit: None,
            accepted: false,
        };
    }
    let mut writer = writer.lock();
    let Some(writer) = writer.as_mut() else {
        return MappingCloudMapAcceptance {
            cloud_commit: None,
            accepted: true,
        };
    };
    let (event, receipt) = match (
        outcome.occupancy_event.as_ref(),
        outcome.occupancy_receipt.as_ref(),
    ) {
        (None, None) => {
            let accepted = outcome.accepted_map_points.is_empty()
                || writer.accumulate_mapping_scan(
                    outcome.session_generation,
                    outcome.accepted_map_points.as_slice(),
                    outcome.sensor_origin_map,
                    outcome.source_timestamp_ns,
                );
            return MappingCloudMapAcceptance {
                cloud_commit: None,
                accepted,
            };
        }
        (Some(event), Some(receipt)) => (event, receipt),
        _ => {
            return MappingCloudMapAcceptance {
                cloud_commit: None,
                accepted: false,
            };
        }
    };
    if receipt.session_generation != outcome.session_generation
        || receipt.session_generation != current_session.session_generation
        || event.session_generation != receipt.session_generation
        || receipt.occupancy_revision != event.revision
        || receipt.source_timestamp_ns != outcome.source_timestamp_ns
        || event.timestamp_ns != receipt.source_timestamp_ns
        || receipt.session_id != current_session.session_id
        || event.session_id != receipt.session_id
    {
        return MappingCloudMapAcceptance {
            cloud_commit: None,
            accepted: false,
        };
    }
    if !outcome.accepted_map_points.is_empty()
        && !writer.accumulate_mapping_scan(
            outcome.session_generation,
            outcome.accepted_map_points.as_slice(),
            outcome.sensor_origin_map,
            outcome.source_timestamp_ns,
        )
    {
        return MappingCloudMapAcceptance {
            cloud_commit: None,
            accepted: false,
        };
    }
    let Some(committed) = writer.commit_accumulated_snapshot(receipt).ok().flatten() else {
        return MappingCloudMapAcceptance {
            cloud_commit: None,
            accepted: false,
        };
    };
    let snapshot = committed.snapshot;
    MappingCloudMapAcceptance {
        cloud_commit: Some(CommittedMappingCloudMap {
            notification: CloudMapNotification {
                revision: snapshot.revision,
                point_count: snapshot.point_count,
                voxel_size: snapshot.voxel_size,
                session_generation: outcome.session_generation,
            },
            source_timestamp_ns: receipt.source_timestamp_ns,
            xyz_f32_le: snapshot.xyz_f32_le,
            receipt: committed.receipt,
        }),
        accepted: true,
    }
}

pub(crate) async fn publish_committed_global_map_in_frame(
    handle: &GeometryRuntimeHandle,
    committed: &CommittedMappingCloudMap,
    frame_id: &str,
) -> Result<(), String> {
    let navigation_context = handle.navigation_map_3d_publish_context();
    // The server callers retain mapping_lifecycle_lock. Reconstructing this
    // identity from current receipts makes their checkpoint path share the
    // canonical SHM/typed-publication sequencer instead of publishing bytes
    // directly to transport.
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "canonical cloud publication mapping adapter is unavailable".to_owned())?;
    let occupancy_receipt = mapping
        .lock()
        .latest_occupancy_commit_receipt()
        .ok_or_else(|| "canonical cloud publication occupancy receipt is unavailable".to_owned())?;
    let direct_commit = MappingIngestCommit {
        outcome: MappingIngestOutcome {
            sensor_origin_map: None,
            accepted_map_points: Vec::new(),
            occupancy_event: None,
            occupancy_receipt: Some(occupancy_receipt),
            #[cfg(test)]
            occupancy_committed: false,
            session_generation: committed.notification.session_generation,
            source_timestamp_ns: committed.source_timestamp_ns,
        },
        cloud_commit: Some(CommittedMappingCloudMap {
            notification: committed.notification.clone(),
            source_timestamp_ns: committed.source_timestamp_ns,
            xyz_f32_le: committed.xyz_f32_le.clone(),
            receipt: committed.receipt.clone(),
        }),
    };
    let expected = canonical_mapping_cloud_identity_from_commit(
        mapping,
        frame_id,
        &direct_commit,
        None,
        None,
    )?
    .ok_or_else(|| "canonical cloud publication direct commit is unavailable".to_owned())?;
    publish_canonical_mapping_cloud_while_lifecycle_fenced(
        &handle.transport,
        mapping,
        &handle.cloud_map_writer,
        &handle.state,
        &handle.cloud_map_tx,
        &expected,
        navigation_context.as_ref(),
    )
    .await?;
    Ok(())
}

pub(crate) fn build_committed_global_map_frame_in_frame(
    committed: &CommittedMappingCloudMap,
    frame_id: &str,
) -> Result<RawLcmFrame, String> {
    build_global_map_snapshot_frame_in_frame(
        &committed.xyz_f32_le,
        committed.notification.revision,
        committed.source_timestamp_ns,
        frame_id,
    )
}

fn build_global_map_snapshot_frame_in_frame(
    xyz_f32_le: &[u8],
    revision: u64,
    source_timestamp_ns: u64,
    frame_id: &str,
) -> Result<RawLcmFrame, String> {
    let payload = crate::typed_codec::encode_global_map_pointcloud_payload_in_frame(
        xyz_f32_le,
        revision,
        source_timestamp_ns,
        frame_id,
    )
    .map_err(|error| format!("global-map PointCloud2 encode failed: {error}"))?;
    let topic = LcmTopic::new(GLOBAL_MAP_CHANNEL)
        .map_err(|error| format!("canonical global-map topic invalid: {error}"))?;
    let message_type = LcmMessageType::new(POINT_CLOUD2_MESSAGE_TYPE)
        .map_err(|error| format!("canonical PointCloud2 type invalid: {error}"))?;
    Ok(crate::typed_codec::build_typed_frame(
        topic,
        message_type,
        payload,
        revision,
    ))
}

async fn publish_raw_lcm_frame(transport: &LcmTransport, frame: RawLcmFrame) -> Result<(), String> {
    match tokio::time::timeout(MAP_FANOUT_SINK_TIMEOUT, transport.publish(frame)).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(format!("global-map typed publish failed: {error}")),
        Err(_) => Err("global-map typed publish timed out".to_string()),
    }
}

pub(crate) async fn publish_retained_global_map_frame(
    handle: &GeometryRuntimeHandle,
    frame: &RawLcmFrame,
) -> Result<(), String> {
    publish_raw_lcm_frame(&handle.transport, frame.clone()).await
}

async fn publish_global_map_snapshot_in_frame(
    transport: &LcmTransport,
    xyz_f32_le: &[u8],
    revision: u64,
    source_timestamp_ns: u64,
    frame_id: &str,
) -> Result<(), String> {
    let frame = build_global_map_snapshot_frame_in_frame(
        xyz_f32_le,
        revision,
        source_timestamp_ns,
        frame_id,
    )?;
    publish_raw_lcm_frame(transport, frame).await
}

#[derive(Clone)]
struct CanonicalCloudPublication {
    notification: CloudMapNotification,
    source_timestamp_ns: u64,
    xyz_f32_le: Vec<u8>,
    /// `None` is reserved for the zero-point session-clear publication. Every
    /// non-empty canonical snapshot carries the complete commit receipt.
    receipt: Option<crate::mapping_commit::CloudCommitReceipt>,
}

#[derive(Clone)]
pub(crate) struct NavigationMap3DPublishContext {
    enabled: bool,
    producer_runtime_generation: u64,
    mapping: Arc<parking_lot::Mutex<MappingAdapter>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
}

#[derive(Clone, Debug, PartialEq)]
struct NavigationMap3DPublicationMetadata {
    committed_map: schema::CommittedMapIdentity,
    producer_runtime_generation: i64,
    map_frame_generation: i64,
    commit_boot_nonce: [u8; 16],
    commit_sequence: i64,
    virtual_obstacles: Option<schema::OccupancyGrid>,
}

async fn navigation_map_3d_publication_metadata(
    context: &NavigationMap3DPublishContext,
    publication: &CanonicalCloudPublication,
    state: &Arc<RwLock<SharedState>>,
    frame_id: &str,
) -> Result<NavigationMap3DPublicationMetadata, String> {
    let (session, map_frame_generation, occupancy_receipt, boot_nonce, configured_frame, virtual_obstacles) = {
        let mapping = context.mapping.lock();
        (
            mapping.snapshot(),
            mapping.global_frame_generation(),
            mapping.latest_occupancy_commit_receipt(),
            mapping.mapping_commit_boot_nonce(),
            mapping.configured_map_frame_id().to_owned(),
            if publication.receipt.is_some() { mapping.virtual_obstacles_snapshot()? } else { None },
        )
    };
    if session.session_generation != publication.notification.session_generation
        || configured_frame != frame_id
        || boot_nonce == 0
    {
        return Err("NavigationMap3D publication crossed mapping session/frame identity".into());
    }
    let mut metadata = NavigationMap3DPublicationMetadata {
        committed_map: schema::CommittedMapIdentity {
            session_id: session.session_id.clone(),
            session_generation: i64::try_from(session.session_generation)
                .map_err(|_| "NavigationMap3D mapping generation exceeds i64".to_string())?,
            frame_id: frame_id.to_owned(),
            ..schema::CommittedMapIdentity::default()
        },
        producer_runtime_generation: i64::try_from(context.producer_runtime_generation)
            .map_err(|_| "NavigationMap3D producer generation exceeds i64".to_string())?,
        map_frame_generation: i64::try_from(map_frame_generation)
            .map_err(|_| "NavigationMap3D frame generation exceeds i64".to_string())?,
        commit_boot_nonce: boot_nonce.to_be_bytes(),
        commit_sequence: 0,
        virtual_obstacles: None,
    };
    let Some(cloud_receipt) = publication.receipt.as_ref() else {
        if publication.notification.point_count != 0 || !publication.xyz_f32_le.is_empty() {
            return Err("NavigationMap3D clear must have an empty canonical cloud".into());
        }
        if metadata.committed_map.session_id.is_empty() {
            metadata.committed_map.session_generation = 0;
        }
        return Ok(metadata);
    };
    let occupancy_receipt = occupancy_receipt
        .ok_or_else(|| "NavigationMap3D requires a real occupancy receipt".to_string())?;
    if !session.active
        || session.session_id != occupancy_receipt.session_id
        || session.revision != occupancy_receipt.occupancy_revision
        || session.session_generation != occupancy_receipt.session_generation
        || cloud_receipt.commit_id != occupancy_receipt.commit_id
        || cloud_receipt.commit_id.boot_nonce != boot_nonce
        || cloud_receipt.session_generation != occupancy_receipt.session_generation
        || cloud_receipt.occupancy_revision != occupancy_receipt.occupancy_revision
        || cloud_receipt.source_timestamp_ns != occupancy_receipt.source_timestamp_ns
        || cloud_receipt.source_timestamp_ns != publication.source_timestamp_ns
        || cloud_receipt.cloud_revision != publication.notification.revision
        || cloud_receipt.point_count != publication.notification.point_count
    {
        return Err("NavigationMap3D map/cloud commit receipts do not match".into());
    }
    let shared = state.read().await;
    if shared.map.revision != occupancy_receipt.occupancy_revision
        || shared.map.timestamp_ns != occupancy_receipt.source_timestamp_ns
        || shared.map.frame_id != frame_id
    {
        return Err("NavigationMap3D canonical occupancy state does not match receipt".into());
    }
    // 摘要沿用规范占据载荷算法，不为三维地图创建另一套内容身份。
    let canonical_data = shared
        .map
        .data
        .iter()
        .map(|value| *value as u8)
        .collect::<Vec<_>>();
    metadata.committed_map.revision = i64::try_from(occupancy_receipt.occupancy_revision)
        .map_err(|_| "NavigationMap3D occupancy revision exceeds i64".to_string())?;
    metadata.committed_map.payload_sha256 = crate::map_checksum::sha256_bytes(&canonical_data);
    metadata.commit_sequence = i64::try_from(cloud_receipt.commit_id.sequence)
        .map_err(|_| "NavigationMap3D commit sequence exceeds i64".to_string())?;
    if let Some(mask) = virtual_obstacles {
        if mask.revision != occupancy_receipt.occupancy_revision
            || mask.timestamp_ns != occupancy_receipt.source_timestamp_ns
            || mask.frame_id != frame_id
            || mask.width != shared.map.width || mask.height != shared.map.height
            || mask.resolution != shared.map.resolution
            || mask.origin_x != shared.map.origin_x || mask.origin_y != shared.map.origin_y
        {
            return Err("NavigationMap3D virtual obstacles do not match the committed occupancy geometry".into());
        }
        let stamp_sec = i32::try_from(mask.timestamp_ns / 1_000_000_000)
            .map_err(|_| "virtual obstacle timestamp exceeds i32".to_string())?;
        let stamp_nsec = (mask.timestamp_ns % 1_000_000_000) as i32;
        metadata.virtual_obstacles = Some(schema::OccupancyGrid {
            header: schema::Header {
                seq: i32::try_from(mask.revision).map_err(|_| "virtual obstacle revision exceeds i32".to_string())?,
                stamp_sec, stamp_nsec, frame_id: mask.frame_id,
            },
            info: schema::MapMetaData {
                map_load_time_sec: stamp_sec, map_load_time_nsec: stamp_nsec,
                resolution: mask.resolution,
                width: i32::try_from(mask.width).map_err(|_| "virtual obstacle width exceeds i32".to_string())?,
                height: i32::try_from(mask.height).map_err(|_| "virtual obstacle height exceeds i32".to_string())?,
                origin: schema::Pose { position: [mask.origin_x, mask.origin_y, 0.0], orientation_xyzw: [0.0, 0.0, 0.0, 1.0] },
            },
            data: mask.data,
        });
    }
    let authority = context
        .committed_map_authority
        .lock()
        .map_err(|_| "NavigationMap3D committed-map authority is unavailable".to_string())?;
    if let Some(authority) = authority.as_ref() {
        if authority.session_id == metadata.committed_map.session_id
            && authority.session_generation == metadata.committed_map.session_generation
            && authority.revision == metadata.committed_map.revision
            && authority != &metadata.committed_map
        {
            return Err("NavigationMap3D canonical occupancy hash conflicts with authority".into());
        }
    }
    Ok(metadata)
}

fn build_navigation_map_3d_publication_frame(
    publication: &CanonicalCloudPublication,
    metadata: &NavigationMap3DPublicationMetadata,
) -> Result<RawLcmFrame, String> {
    let source_timestamp_ns = i64::try_from(publication.source_timestamp_ns)
        .map_err(|_| "NavigationMap3D source time exceeds i64".to_string())?;
    let revision = i64::try_from(publication.notification.revision)
        .map_err(|_| "NavigationMap3D cloud revision exceeds i64".to_string())?;
    let cloud = schema::PointCloud2 {
        header: schema::Header {
            seq: i32::try_from(revision)
                .map_err(|_| "NavigationMap3D cloud revision exceeds header".to_string())?,
            stamp_sec: i32::try_from(source_timestamp_ns / 1_000_000_000)
                .map_err(|_| "NavigationMap3D source time exceeds header".to_string())?,
            stamp_nsec: (source_timestamp_ns % 1_000_000_000) as i32,
            frame_id: metadata.committed_map.frame_id.clone(),
        },
        height: 1,
        width: i32::try_from(publication.notification.point_count)
            .map_err(|_| "NavigationMap3D point count exceeds i32".to_string())?,
        fields: ["x", "y", "z"]
            .iter()
            .enumerate()
            .map(|(index, name)| schema::PointField {
                name: (*name).to_string(),
                offset: index as i32 * 4,
                datatype: 7,
                count: 1,
            })
            .collect(),
        point_step: 12,
        row_step: i32::try_from(publication.xyz_f32_le.len())
            .map_err(|_| "NavigationMap3D point payload exceeds i32".to_string())?,
        data: publication.xyz_f32_le.clone(),
        is_dense: true,
        ..schema::PointCloud2::default()
    };
    let message = schema::NavigationMap3D {
        schema_version: 1,
        producer_runtime_generation: metadata.producer_runtime_generation,
        committed_map: metadata.committed_map.clone(),
        map_frame_generation: metadata.map_frame_generation,
        cloud_revision: revision,
        source_timestamp_ns,
        source_voxel_size: publication.notification.voxel_size,
        commit_boot_nonce: metadata.commit_boot_nonce,
        commit_sequence: metadata.commit_sequence,
        cleared: publication.receipt.is_none(),
        virtual_obstacles: metadata.virtual_obstacles.clone(),
        cloud,
    };
    let payload =
        schema::encode_navigation_map_3d_body(&message).map_err(|error| error.to_string())?;
    Ok(crate::typed_codec::build_typed_frame(
        LcmTopic::new("/nav/map3d").map_err(|error| error.to_string())?,
        LcmMessageType::new("nuwax_msgs.NavigationMap3D").map_err(|error| error.to_string())?,
        payload,
        publication.notification.revision,
    ))
}

fn canonical_cloud_payload_matches(notification: &CloudMapNotification, xyz_f32_le: &[u8]) -> bool {
    usize::try_from(notification.point_count)
        .ok()
        .and_then(|point_count| point_count.checked_mul(12))
        == Some(xyz_f32_le.len())
}

fn canonical_cloud_receipt_matches_snapshot(
    receipt: &crate::mapping_commit::CloudCommitReceipt,
    snapshot: &crate::pointcloud::CommittedCloudMapSnapshot,
) -> bool {
    receipt.session_generation == snapshot.session_generation
        && receipt.cloud_revision == snapshot.revision
        && receipt.point_count == snapshot.point_count
        && canonical_cloud_payload_matches(
            &CloudMapNotification {
                revision: snapshot.revision,
                point_count: snapshot.point_count,
                voxel_size: snapshot.voxel_size,
                session_generation: snapshot.session_generation,
            },
            &snapshot.xyz_f32_le,
        )
}

fn same_canonical_cloud_publication_identity(
    left: &CanonicalCloudPublication,
    right: &CanonicalCloudPublication,
) -> bool {
    same_cloud_map_notification(&left.notification, &right.notification)
        && left.source_timestamp_ns == right.source_timestamp_ns
        && left.xyz_f32_le == right.xyz_f32_le
        && left.receipt == right.receipt
}

fn canonical_committed_cloud_publication_matches_writer(
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    expected: &CanonicalCloudPublication,
) -> bool {
    if expected.receipt.is_none() {
        return false;
    }
    let guard = writer.lock();
    let Some(writer) = guard.as_ref() else {
        return false;
    };
    let Some(snapshot) = writer.committed_snapshot() else {
        return false;
    };
    let Some(receipt) = writer.last_cloud_receipt() else {
        return false;
    };
    if !canonical_cloud_receipt_matches_snapshot(&receipt, &snapshot) {
        return false;
    }
    let current = CanonicalCloudPublication {
        notification: CloudMapNotification {
            revision: snapshot.revision,
            point_count: snapshot.point_count,
            voxel_size: snapshot.voxel_size,
            session_generation: snapshot.session_generation,
        },
        source_timestamp_ns: receipt.source_timestamp_ns,
        xyz_f32_le: snapshot.xyz_f32_le,
        receipt: Some(receipt),
    };
    same_canonical_cloud_publication_identity(&current, expected)
}

/// Per-writer invalidation signal for the canonical `/global_map` sequencer.
///
/// `TypedCloudPublishState` owns ordering, but it only exposes a session watch.
/// A newer revision in the same session must also cancel a blocked older send so
/// the sequencer can publish the current canonical snapshot instead.
struct CanonicalCloudPublicationInvalidation {
    identity_tx: watch::Sender<(u64, u64)>,
}

impl CanonicalCloudPublicationInvalidation {
    fn new() -> Self {
        let (identity_tx, _) = watch::channel((0, 0));
        Self { identity_tx }
    }

    fn observe(&self, identity: (u64, u64)) {
        if *self.identity_tx.borrow() != identity {
            self.identity_tx.send_replace(identity);
        }
    }

    fn subscribe(&self) -> watch::Receiver<(u64, u64)> {
        self.identity_tx.subscribe()
    }
}

struct CanonicalCloudPublicationInvalidationEntry {
    writer: Weak<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    invalidation: Arc<CanonicalCloudPublicationInvalidation>,
}

static CANONICAL_CLOUD_PUBLICATION_INVALIDATIONS: LazyLock<
    parking_lot::Mutex<HashMap<usize, CanonicalCloudPublicationInvalidationEntry>>,
> = LazyLock::new(|| parking_lot::Mutex::new(HashMap::new()));

fn canonical_cloud_publication_invalidation(
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
) -> Arc<CanonicalCloudPublicationInvalidation> {
    let key = Arc::as_ptr(writer) as usize;
    let mut invalidations = CANONICAL_CLOUD_PUBLICATION_INVALIDATIONS.lock();
    invalidations.retain(|_, entry| entry.writer.upgrade().is_some());
    if let Some(entry) = invalidations.get(&key) {
        if entry
            .writer
            .upgrade()
            .is_some_and(|current| Arc::ptr_eq(&current, writer))
        {
            return Arc::clone(&entry.invalidation);
        }
    }
    let invalidation = Arc::new(CanonicalCloudPublicationInvalidation::new());
    invalidations.insert(
        key,
        CanonicalCloudPublicationInvalidationEntry {
            writer: Arc::downgrade(writer),
            invalidation: Arc::clone(&invalidation),
        },
    );
    invalidation
}

#[allow(clippy::type_complexity)]
fn latest_canonical_cloud_publication(
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
) -> Result<
    Option<(
        Arc<crate::pointcloud::TypedCloudPublishState>,
        Arc<CanonicalCloudPublicationInvalidation>,
        CanonicalCloudPublication,
    )>,
    String,
> {
    let invalidation = canonical_cloud_publication_invalidation(writer);
    let guard = writer.lock();
    let Some(writer) = guard.as_ref() else {
        return Ok(None);
    };
    let publish_state = writer.typed_publish_state();
    let Some(snapshot) = writer.committed_snapshot() else {
        return Ok(None);
    };
    let receipt = writer.last_cloud_receipt();
    if let Some(receipt) = receipt.as_ref() {
        if !canonical_cloud_receipt_matches_snapshot(receipt, &snapshot) {
            return Err(
                "canonical cloud publication receipt does not match the SHM snapshot".to_owned(),
            );
        }
    } else if snapshot.point_count != 0 || !snapshot.xyz_f32_le.is_empty() {
        return Err("non-empty canonical cloud snapshot is missing its commit receipt".to_owned());
    }
    let source_timestamp_ns = receipt
        .as_ref()
        .map_or(0, |receipt| receipt.source_timestamp_ns);
    publish_state.observe_session_generation(snapshot.session_generation);
    invalidation.observe((snapshot.session_generation, snapshot.revision));
    Ok(Some((
        publish_state,
        invalidation,
        CanonicalCloudPublication {
            notification: CloudMapNotification {
                revision: snapshot.revision,
                point_count: snapshot.point_count,
                voxel_size: snapshot.voxel_size,
                session_generation: snapshot.session_generation,
            },
            source_timestamp_ns,
            xyz_f32_le: snapshot.xyz_f32_le,
            receipt,
        },
    )))
}

async fn publish_latest_canonical_cloud_map(
    transport: &LcmTransport,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    state: &Arc<RwLock<SharedState>>,
    cloud_map_tx: &broadcast::Sender<CloudMapNotification>,
    frame_id: &str,
) -> Result<Option<CloudMapNotification>, String> {
    publish_latest_canonical_cloud_map_with_navigation(
        transport,
        writer,
        state,
        cloud_map_tx,
        frame_id,
        None,
    )
    .await
}

async fn publish_latest_canonical_cloud_map_with_navigation(
    transport: &LcmTransport,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    state: &Arc<RwLock<SharedState>>,
    cloud_map_tx: &broadcast::Sender<CloudMapNotification>,
    frame_id: &str,
    navigation_context: Option<&NavigationMap3DPublishContext>,
) -> Result<Option<CloudMapNotification>, String> {
    'acquire_sequencer: loop {
        let Some((publish_state, _, _)) = latest_canonical_cloud_publication(writer)? else {
            return Ok(None);
        };
        let _sequencer = publish_state.lock_sequencer().await;
        loop {
            let Some((current_publish_state, invalidation, latest_publication)) =
                latest_canonical_cloud_publication(writer)?
            else {
                return Ok(None);
            };
            if !Arc::ptr_eq(&publish_state, &current_publish_state) {
                continue 'acquire_sequencer;
            }
            let pending_clear = publish_state
                .pending_clear_for_session(latest_publication.notification.session_generation);
            let canonical_identity = (
                latest_publication.notification.session_generation,
                latest_publication.notification.revision,
            );
            let publication = pending_clear
                .as_ref()
                .map(|clear| CanonicalCloudPublication {
                    notification: CloudMapNotification {
                        revision: clear.revision,
                        point_count: clear.point_count,
                        voxel_size: clear.voxel_size,
                        session_generation: clear.session_generation,
                    },
                    source_timestamp_ns: 0,
                    xyz_f32_le: Vec::new(),
                    receipt: None,
                })
                .unwrap_or(latest_publication);
            let identity = (
                publication.notification.session_generation,
                publication.notification.revision,
            );
            if publish_state.published_identity() == Some(identity) {
                publish_state.mark_published(identity);
                if pending_clear.is_some() {
                    continue;
                }
                return Ok(Some(publication.notification));
            }

            if publication.receipt.is_some()
                && !canonical_committed_cloud_publication_matches_writer(writer, &publication)
            {
                continue;
            }

            let navigation_metadata = if let Some(context) =
                navigation_context.filter(|context| context.enabled)
            {
                Some(
                    navigation_map_3d_publication_metadata(context, &publication, state, frame_id)
                        .await?,
                )
            } else {
                None
            };
            let navigation_frame = navigation_metadata
                .as_ref()
                .map(|metadata| build_navigation_map_3d_publication_frame(&publication, metadata))
                .transpose()?;

            let mut canonical_identity_rx = invalidation.subscribe();
            if *canonical_identity_rx.borrow_and_update() != canonical_identity {
                continue;
            }
            let mut session_generation_rx = publish_state.session_generation_rx();
            if *session_generation_rx.borrow_and_update() != canonical_identity.0 {
                continue;
            }
            // 两路规范输出共享原有发送预算，不把生命周期等待上限翻倍。
            let publish_deadline = tokio::time::Instant::now() + MAP_FANOUT_SINK_TIMEOUT;
            let publish_result = tokio::select! {
                biased;
                changed = session_generation_rx.changed() => {
                    match changed {
                        Ok(()) => None,
                        Err(_) => return Ok(None),
                    }
                }
                changed = canonical_identity_rx.changed() => {
                    match changed {
                        Ok(()) => None,
                        Err(_) => return Ok(None),
                    }
                }
                result = async {
                    publish_global_map_snapshot_in_frame(
                        transport, &publication.xyz_f32_le, publication.notification.revision,
                        publication.source_timestamp_ns, frame_id,
                    ).await?;
                    if let Some(frame) = navigation_frame {
                        let Some((_, _, current)) = latest_canonical_cloud_publication(writer)? else {
                            return Ok::<bool, String>(false);
                        };
                        if (current.notification.session_generation, current.notification.revision) != canonical_identity
                            || (publication.receipt.is_some()
                                && !same_canonical_cloud_publication_identity(&current, &publication))
                        {
                            return Ok(false);
                        }
                        if let (Some(context), Some(expected)) = (navigation_context, navigation_metadata.as_ref()) {
                            let current = navigation_map_3d_publication_metadata(context, &publication, state, frame_id).await?;
                            if current != *expected {
                                return Ok(false);
                            }
                        }
                        tokio::time::timeout_at(publish_deadline, publish_raw_lcm_frame(transport, frame))
                            .await
                            .map_err(|_| "NavigationMap3D publish exceeded canonical cloud deadline".to_string())??;
                    }
                    Ok::<bool, String>(true)
                } => Some(result),
            };
            let Some(publish_result) = publish_result else {
                continue;
            };
            if !publish_result? {
                continue;
            }

            if let (Some(context), Some(expected)) =
                (navigation_context, navigation_metadata.as_ref())
            {
                let current =
                    navigation_map_3d_publication_metadata(context, &publication, state, frame_id)
                        .await?;
                if current != *expected {
                    return Err("NavigationMap3D publication identity changed during send".into());
                }
            }

            let mut shared = state.write().await;
            let current_snapshot_matches = {
                let current_writer = writer.lock();
                let Some(current_writer) = current_writer.as_ref() else {
                    return Ok(None);
                };
                if !Arc::ptr_eq(&publish_state, &current_writer.typed_publish_state()) {
                    drop(shared);
                    continue 'acquire_sequencer;
                }
                let Some(current_snapshot) = current_writer.committed_snapshot() else {
                    return Ok(None);
                };
                current_snapshot.session_generation == canonical_identity.0
                    && current_snapshot.revision == canonical_identity.1
            };
            let committed_publication_matches = publication.receipt.as_ref().is_none_or(|_| {
                canonical_committed_cloud_publication_matches_writer(writer, &publication)
            });
            if !current_snapshot_matches
                || !committed_publication_matches
                || *session_generation_rx.borrow_and_update() != canonical_identity.0
                || *canonical_identity_rx.borrow_and_update() != canonical_identity
            {
                drop(shared);
                continue;
            }

            publish_state.mark_published(identity);
            if shared.cloud_map_revision < publication.notification.revision {
                shared.cloud_map_revision = publication.notification.revision;
                let _ = cloud_map_tx.send(publication.notification.clone());
            }
        }
    }
}

#[derive(Clone)]
struct ExpectedRelocalizationMode {
    fence: Arc<crate::correction_coordinator::RelocalizationModeFence>,
    epoch: u64,
    active: bool,
}

impl ExpectedRelocalizationMode {
    fn matches(&self) -> bool {
        let current = self.fence.snapshot();
        current.epoch == self.epoch && current.active == self.active
    }
}

struct CanonicalMappingCloudIdentity<'a> {
    map_frame_id: String,
    mapping_session_id: String,
    mapping_session_generation: u64,
    map_frame_generation: u64,
    occupancy_receipt: &'a crate::mapping_commit::OccupancyCommitReceipt,
    cloud: &'a CommittedMappingCloudMap,
    expected_mode: Option<ExpectedRelocalizationMode>,
}

fn same_cloud_map_notification(left: &CloudMapNotification, right: &CloudMapNotification) -> bool {
    left.revision == right.revision
        && left.point_count == right.point_count
        && left.voxel_size.to_bits() == right.voxel_size.to_bits()
        && left.session_generation == right.session_generation
}

fn canonical_mapping_cloud_identity_from_commit<'a>(
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    map_frame: &str,
    commit: &'a MappingIngestCommit,
    binding: Option<&crate::lio_ingress::RegistrationBinding>,
    expected_mode: Option<ExpectedRelocalizationMode>,
) -> Result<Option<CanonicalMappingCloudIdentity<'a>>, String> {
    let Some(cloud) = commit.cloud_commit.as_ref() else {
        return Ok(None);
    };
    let occupancy_receipt = commit
        .outcome
        .occupancy_receipt
        .as_ref()
        .ok_or_else(|| "canonical cloud publication lacks an occupancy receipt".to_owned())?;
    let map_frame_id = normalize_frame(map_frame);
    if map_frame_id.is_empty() {
        return Err("canonical cloud publication map frame is empty".to_owned());
    }

    let (mapping_session_id, mapping_session_generation, map_frame_generation) =
        if let Some(binding) = binding {
            (
                binding.mapping_session_id.clone(),
                binding.mapping_session_generation,
                binding.map_frame_generation,
            )
        } else {
            let mapping = mapping.lock();
            let snapshot = mapping.snapshot();
            (
                snapshot.session_id,
                snapshot.session_generation,
                mapping.global_frame_generation(),
            )
        };
    let expected_payload_len = usize::try_from(cloud.notification.point_count)
        .ok()
        .and_then(|point_count| point_count.checked_mul(12));
    if occupancy_receipt.session_id != mapping_session_id
        || occupancy_receipt.session_generation != mapping_session_generation
        || cloud.receipt.commit_id != occupancy_receipt.commit_id
        || cloud.receipt.session_generation != mapping_session_generation
        || cloud.receipt.occupancy_revision != occupancy_receipt.occupancy_revision
        || cloud.receipt.source_timestamp_ns != occupancy_receipt.source_timestamp_ns
        || cloud.notification.revision != cloud.receipt.cloud_revision
        || cloud.notification.point_count != cloud.receipt.point_count
        || cloud.notification.session_generation != mapping_session_generation
        || cloud.source_timestamp_ns != cloud.receipt.source_timestamp_ns
        || expected_payload_len != Some(cloud.xyz_f32_le.len())
    {
        return Err("canonical cloud publication commit identity mismatch".to_owned());
    }

    Ok(Some(CanonicalMappingCloudIdentity {
        map_frame_id,
        mapping_session_id,
        mapping_session_generation,
        map_frame_generation,
        occupancy_receipt,
        cloud,
        expected_mode,
    }))
}

/// Validate the map, SHM, and shared-state proofs for one committed cloud
/// publication. The caller owns `mapping_lifecycle_lock` throughout the
/// publication attempt, so a passing pre-check remains meaningful until the
/// post-publish check below.
async fn ensure_canonical_mapping_cloud_identity(
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    state: &Arc<RwLock<SharedState>>,
    expected: &CanonicalMappingCloudIdentity<'_>,
    require_published_cloud_revision: bool,
) -> Result<(), String> {
    if expected
        .expected_mode
        .as_ref()
        .is_some_and(|mode| !mode.matches())
    {
        return Err("canonical cloud publication crossed a relocalization mode fence".to_owned());
    }
    let (mapping_snapshot, map_frame_generation, occupancy_receipt, configured_map_frame) = {
        let mapping = mapping.lock();
        (
            mapping.snapshot(),
            mapping.global_frame_generation(),
            mapping.latest_occupancy_commit_receipt(),
            normalize_frame(mapping.configured_map_frame_id()),
        )
    };
    let (cloud_snapshot, cloud_receipt) = {
        let writer = writer.lock();
        let writer = writer
            .as_ref()
            .ok_or_else(|| "canonical cloud publication writer is unavailable".to_owned())?;
        (
            writer
                .committed_snapshot()
                .ok_or_else(|| "canonical cloud publication snapshot is unavailable".to_owned())?,
            writer
                .last_cloud_receipt()
                .ok_or_else(|| "canonical cloud publication receipt is unavailable".to_owned())?,
        )
    };
    let shared = state.read().await;
    let cloud_revision_matches = if require_published_cloud_revision {
        shared.cloud_map_revision == expected.cloud.receipt.cloud_revision
    } else {
        shared.cloud_map_revision <= expected.cloud.receipt.cloud_revision
    };
    if !mapping_snapshot.active
        || mapping_snapshot.session_id != expected.mapping_session_id
        || mapping_snapshot.session_generation != expected.mapping_session_generation
        || mapping_snapshot.revision != expected.occupancy_receipt.occupancy_revision
        || map_frame_generation != expected.map_frame_generation
        || configured_map_frame != expected.map_frame_id
        || occupancy_receipt.as_ref() != Some(expected.occupancy_receipt)
        || cloud_receipt != expected.cloud.receipt
        || cloud_snapshot.revision != expected.cloud.notification.revision
        || cloud_snapshot.point_count != expected.cloud.notification.point_count
        || cloud_snapshot.voxel_size.to_bits() != expected.cloud.notification.voxel_size.to_bits()
        || cloud_snapshot.session_generation != expected.cloud.notification.session_generation
        || cloud_snapshot.xyz_f32_le != expected.cloud.xyz_f32_le
        || shared.map.revision != expected.occupancy_receipt.occupancy_revision
        || shared.map.timestamp_ns != expected.occupancy_receipt.source_timestamp_ns
        || normalize_frame(&shared.map.frame_id) != expected.map_frame_id
        || !cloud_revision_matches
        || expected
            .expected_mode
            .as_ref()
            .is_some_and(|mode| !mode.matches())
    {
        return Err("canonical map/cloud identity changed around publication".to_owned());
    }
    Ok(())
}

async fn publish_canonical_mapping_cloud_while_lifecycle_fenced(
    transport: &LcmTransport,
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    state: &Arc<RwLock<SharedState>>,
    cloud_map_tx: &broadcast::Sender<CloudMapNotification>,
    expected: &CanonicalMappingCloudIdentity<'_>,
    navigation_context: Option<&NavigationMap3DPublishContext>,
) -> Result<CloudMapNotification, String> {
    ensure_canonical_mapping_cloud_identity(mapping, writer, state, expected, false).await?;
    let published = publish_latest_canonical_cloud_map_with_navigation(
        transport,
        writer,
        state,
        cloud_map_tx,
        &expected.map_frame_id,
        navigation_context,
    )
    .await?
    .ok_or_else(|| "canonical cloud-map snapshot became unavailable".to_owned())?;
    if !same_cloud_map_notification(&published, &expected.cloud.notification) {
        return Err("canonical cloud publication identity changed".to_owned());
    }
    ensure_canonical_mapping_cloud_identity(mapping, writer, state, expected, true).await?;
    Ok(published)
}

#[allow(clippy::too_many_arguments)]
async fn publish_mapping_ingest_commit_while_lifecycle_fenced(
    transport: &LcmTransport,
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    state: &Arc<RwLock<SharedState>>,
    cloud_map_tx: &broadcast::Sender<CloudMapNotification>,
    map_revision_tx: &broadcast::Sender<MapRevisionEvent>,
    map_frame: &str,
    commit: &MappingIngestCommit,
    binding: Option<&crate::lio_ingress::RegistrationBinding>,
    expected_mode: Option<ExpectedRelocalizationMode>,
    navigation_context: Option<&NavigationMap3DPublishContext>,
) -> Result<(), String> {
    if let Some(event) = commit.outcome.occupancy_event.as_ref() {
        let _ = map_revision_tx.send(event.clone());
    }
    let Some(expected) = canonical_mapping_cloud_identity_from_commit(
        mapping,
        map_frame,
        commit,
        binding,
        expected_mode,
    )?
    else {
        return Ok(());
    };
    publish_canonical_mapping_cloud_while_lifecycle_fenced(
        transport,
        mapping,
        writer,
        state,
        cloud_map_tx,
        &expected,
        navigation_context,
    )
    .await?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
async fn publish_mapping_ingest_commit_with_lifecycle_fence(
    transport: &LcmTransport,
    mapping: &Arc<parking_lot::Mutex<MappingAdapter>>,
    lifecycle_lock: &Arc<tokio::sync::Mutex<()>>,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    state: &Arc<RwLock<SharedState>>,
    cloud_map_tx: &broadcast::Sender<CloudMapNotification>,
    map_revision_tx: &broadcast::Sender<MapRevisionEvent>,
    map_frame: &str,
    commit: &MappingIngestCommit,
    binding: Option<&crate::lio_ingress::RegistrationBinding>,
    expected_mode: Option<ExpectedRelocalizationMode>,
    navigation_context: Option<&NavigationMap3DPublishContext>,
) -> Result<(), String> {
    let _lifecycle_guard = lifecycle_lock.lock().await;
    publish_mapping_ingest_commit_while_lifecycle_fenced(
        transport,
        mapping,
        writer,
        state,
        cloud_map_tx,
        map_revision_tx,
        map_frame,
        commit,
        binding,
        expected_mode,
        navigation_context,
    )
    .await
}

async fn publish_read_only_mapping_cloud_while_lifecycle_fenced(
    handle: &GeometryRuntimeHandle,
    map_frame: &str,
    binding: &crate::lio_ingress::RegistrationBinding,
    expected_cloud: &CommittedMappingCloudMap,
    expected_mode: ExpectedRelocalizationMode,
    navigation_context: Option<&NavigationMap3DPublishContext>,
) -> Result<CloudMapNotification, String> {
    if !expected_mode.matches() {
        return Err("relocalization mode changed before read-only publication".to_owned());
    }
    ensure_read_only_mapping_identity(handle, binding, expected_cloud).await?;
    let published = publish_latest_canonical_cloud_map_with_navigation(
        &handle.transport,
        &handle.cloud_map_writer,
        &handle.state,
        &handle.cloud_map_tx,
        map_frame,
        navigation_context,
    )
    .await?
    .ok_or_else(|| "relocalization canonical cloud publication disappeared".to_owned())?;
    if !same_cloud_map_notification(&published, &expected_cloud.notification) {
        return Err("relocalization canonical cloud publication identity changed".to_owned());
    }
    if !expected_mode.matches() {
        return Err("relocalization mode changed during read-only publication".to_owned());
    }
    ensure_read_only_mapping_identity(handle, binding, expected_cloud).await?;
    Ok(published)
}

fn mapping_feed_should_wait_for_active_session(
    mapping: &parking_lot::Mutex<MappingAdapter>,
) -> bool {
    mapping
        .try_lock()
        .map(|adapter| !adapter.is_active())
        .unwrap_or(false)
}

/// Apply one source-fenced LIO odometry observation through the existing
/// canonical MappingAdapter pose/TF boundary.
///
/// The caller owns `mapping_lifecycle_lock`; this function deliberately does
/// not introduce a second lifecycle or TF authority.
pub(crate) async fn handoff_lio_odometry(
    handle: &GeometryRuntimeHandle,
    pose: &PoseState,
    base_frame: &str,
    tf_binding: &(LcmTopic, LcmMessageType),
) -> Result<bool, String> {
    handoff_lio_odometry_inner(handle, pose, base_frame, tf_binding, None).await
}

pub(crate) async fn handoff_lio_odometry_with_correction(
    handle: &GeometryRuntimeHandle,
    pose: &PoseState,
    base_frame: &str,
    tf_binding: &(LcmTopic, LcmMessageType),
    correction: &GeometryCanonicalCorrectionApplier,
) -> Result<bool, String> {
    handoff_lio_odometry_inner(handle, pose, base_frame, tf_binding, Some(correction)).await
}

async fn handoff_lio_odometry_inner(
    handle: &GeometryRuntimeHandle,
    pose: &PoseState,
    base_frame: &str,
    tf_binding: &(LcmTopic, LcmMessageType),
    correction: Option<&GeometryCanonicalCorrectionApplier>,
) -> Result<bool, String> {
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_string())?;
    if mapping_feed_should_wait_for_active_session(mapping.as_ref()) {
        return Ok(false);
    }

    let transform_cache = mapping.lock().map_odom_transform_cache();
    let base_observation = canonical_mapping_observation_for_odom(
        mapping.as_ref(),
        transform_cache.as_ref(),
        pose,
        base_frame,
    )?;
    let (mapping_snapshot, frame_generation) = {
        let mapping = mapping.lock();
        (mapping.snapshot(), mapping.global_frame_generation())
    };
    let mut shared = handle.state.write().await;
    let observation = match correction.and_then(GeometryCanonicalCorrectionApplier::snapshot) {
        Some(snapshot) => {
            if mapping_snapshot.active
                && mapping_snapshot.session_id == snapshot.mapping_session_id
                && mapping_snapshot.session_generation == snapshot.mapping_session_generation
                && frame_generation == snapshot.map_frame_generation
            {
                canonical_mapping_observation_from_isometry(
                    &snapshot.map_frame_id,
                    &snapshot.odom_frame_id,
                    snapshot.correction_revision,
                    &snapshot.effective_map_from_odom,
                    pose,
                    base_frame,
                )?
            } else {
                base_observation
            }
        }
        None => base_observation,
    };
    apply_canonical_mapping_observation_locked(&mut shared, &observation);
    // 规范状态已经提交；远程坐标变换发布不能继续挡住位姿和坐标树读取。
    // 调用方持有的建图生命周期围栏仍按原范围保护发布与会话切换。
    drop(shared);

    let tf_frame = crate::typed_codec::build_typed_frame(
        tf_binding.0.clone(),
        tf_binding.1.clone(),
        observation.tf_payload,
        pose.timestamp_ns,
    );
    match tokio::time::timeout(MAP_FANOUT_SINK_TIMEOUT, handle.transport.publish(tf_frame)).await {
        Ok(Ok(())) => Ok(true),
        Ok(Err(error)) => Err(format!("canonical mapping TF publish failed: {error}")),
        Err(_) => Err("canonical mapping TF publish timed out".to_string()),
    }
}

/// 接纳已注册到地图坐标的实时LIO点云；返回值表示点被接纳，不表示本帧一定发布新修订。
/// 发布节流期间仍保留已接纳点，等后续候选按原有地图/共享内存边界发布。
///
/// `adapter_timestamp_ns` belongs to the adapter's monotonic freshness domain;
/// `source_timestamp_ns` remains the public source identity and output metadata.
/// The caller owns `mapping_lifecycle_lock` and the one-shot registration permit.
pub(crate) async fn handoff_registered_lio_cloud(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    adapter_timestamp_ns: u64,
    source_timestamp_ns: u64,
    map_frame: &str,
) -> Result<bool, String> {
    handoff_registered_lio_cloud_inner(
        handle,
        points,
        point_step,
        adapter_timestamp_ns,
        source_timestamp_ns,
        map_frame,
        None,
        None,
    )
    .await
}

pub(crate) async fn handoff_registered_lio_cloud_fenced(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    adapter_timestamp_ns: u64,
    source_timestamp_ns: u64,
    map_frame: &str,
    expected_session_generation: u64,
    expected_frame_generation: u64,
    sensor_origin_map: Option<[f64; 3]>,
) -> Result<bool, String> {
    handoff_registered_lio_cloud_inner(
        handle,
        points,
        point_step,
        adapter_timestamp_ns,
        source_timestamp_ns,
        map_frame,
        Some((expected_session_generation, expected_frame_generation)),
        sensor_origin_map,
    )
    .await
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn handoff_registered_lio_cloud_with_correction(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    adapter_timestamp_ns: u64,
    source_timestamp_ns: u64,
    map_frame: &str,
    body_frame: &str,
    binding: &crate::lio_ingress::RegistrationBinding,
    correction: &crate::correction_coordinator::CorrectionFrameIngress,
) -> Result<bool, String> {
    let navigation_context = handle.navigation_map_3d_publish_context();
    let initial_mode = correction.relocalization_mode_snapshot();
    let Some(frame_capture_epoch) = binding.frame_capture_relocalization_epoch else {
        return Ok(false);
    };
    if frame_capture_epoch != initial_mode.epoch {
        // This cloud crossed the LIO admission boundary in a different mode
        // generation. It cannot become a source for this command or be
        // committed after a relocalization edge.
        return Ok(false);
    }
    if initial_mode.active {
        let lifecycle_guard = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        let active_mode = correction.relocalization_mode_snapshot();
        if active_mode == initial_mode {
            let frame = registered_lio_read_only_correction_frame(
                handle,
                points,
                source_timestamp_ns,
                map_frame,
                body_frame,
                binding,
                active_mode.epoch,
            )
            .await?;
            if correction.relocalization_mode_snapshot() != active_mode {
                return Ok(false);
            }
            match correction
                .submit_with_lifecycle_guard(frame.input, lifecycle_guard)
                .await
            {
                Ok(_) | Err(crate::correction_coordinator::CorrectionRuntimeError::Stopped) => {}
                Err(error) => return Err(error.to_string()),
            }
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            if correction.relocalization_mode_snapshot() != active_mode {
                return Ok(false);
            }
            publish_read_only_mapping_cloud_while_lifecycle_fenced(
                handle,
                map_frame,
                binding,
                &frame.cloud,
                ExpectedRelocalizationMode {
                    fence: correction.relocalization_mode_fence(),
                    epoch: active_mode.epoch,
                    active: active_mode.active,
                },
                navigation_context.as_ref(),
            )
            .await?;
            return Ok(true);
        }
    }

    let commit = commit_registered_lio_cloud_inner(
        handle,
        points,
        point_step,
        adapter_timestamp_ns,
        source_timestamp_ns,
        map_frame,
        Some((
            binding.mapping_session_generation,
            binding.map_frame_generation,
        )),
        Some(initial_mode.epoch),
        Some(correction.relocalization_mode_fence()),
        binding.sensor_origin_map,
    )
    .await?;
    let occupancy_committed = commit.outcome.occupancy_event.is_some();
    if !occupancy_committed {
        let accepted = !commit.outcome.accepted_map_points.is_empty();
        if accepted {
            debug!(
                event = "lio.cloud.accepted_without_revision",
                reason = "publication_cadence",
                accepted_points = commit.outcome.accepted_map_points.len(),
                source_timestamp_ns,
                map_frame,
                "实时点云已接纳，等待下一次地图发布"
            );
        }
        return Ok(accepted);
    }

    let lifecycle_guard = handle.mapping_lifecycle_lock.clone().lock_owned().await;
    if correction.relocalization_mode_snapshot() != initial_mode {
        return Ok(false);
    }
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
    let mapping_snapshot = mapping.lock().snapshot();
    if !mapping_snapshot.active
        || mapping_snapshot.session_id != binding.mapping_session_id
        || mapping_snapshot.session_generation != binding.mapping_session_generation
        || mapping.lock().global_frame_generation() != binding.map_frame_generation
    {
        return Ok(false);
    }

    publish_mapping_ingest_commit_while_lifecycle_fenced(
        &handle.transport,
        mapping,
        &handle.cloud_map_writer,
        &handle.state,
        &handle.cloud_map_tx,
        &handle.map_revision_tx,
        map_frame,
        &commit,
        Some(binding),
        Some(ExpectedRelocalizationMode {
            fence: correction.relocalization_mode_fence(),
            epoch: initial_mode.epoch,
            active: initial_mode.active,
        }),
        navigation_context.as_ref(),
    )
    .await?;
    let event = commit
        .outcome
        .occupancy_event
        .as_ref()
        .ok_or_else(|| "registered LIO correction lost its occupancy event".to_owned())?;
    let occupancy_receipt = commit
        .outcome
        .occupancy_receipt
        .as_ref()
        .ok_or_else(|| "registered LIO correction lacks an occupancy receipt".to_owned())?;
    let cloud = commit
        .cloud_commit
        .as_ref()
        .ok_or_else(|| "registered LIO correction lacks a cloud receipt".to_owned())?;
    if occupancy_receipt.commit_id != cloud.receipt.commit_id
        || occupancy_receipt.session_id != binding.mapping_session_id
        || occupancy_receipt.session_generation != binding.mapping_session_generation
        || cloud.receipt.session_generation != binding.mapping_session_generation
        || occupancy_receipt.occupancy_revision != event.revision
        || cloud.receipt.occupancy_revision != event.revision
        || cloud.receipt.source_timestamp_ns != source_timestamp_ns
        || occupancy_receipt.source_timestamp_ns != source_timestamp_ns
        || event.timestamp_ns != source_timestamp_ns
    {
        return Err("registered LIO correction receipt identity mismatch".to_owned());
    }

    let map_from_body = binding.base_anchor_map_from_odom * binding.frozen_odom_from_body;
    let body_from_map = map_from_body.inverse();
    let body_points = points
        .iter()
        .map(|point| body_from_map.transform_point(point))
        .collect::<Vec<_>>();
    let shared = handle.state.read().await;
    if shared.map.revision != event.revision
        || shared.map.timestamp_ns != source_timestamp_ns
        || normalize_frame(&shared.map.frame_id) != normalize_frame(map_frame)
        || shared.cloud_map_revision != cloud.receipt.cloud_revision
    {
        return Err("canonical map/cloud state does not match correction receipts".to_owned());
    }
    let committed_map = crate::correction_coordinator::CommittedMapProof {
        source_timestamp_ns,
        width: u64::from(shared.map.width),
        height: u64::from(shared.map.height),
        resolution_m: f64::from(shared.map.resolution),
        origin_x_m: shared.map.origin_x,
        origin_y_m: shared.map.origin_y,
        payload_sha256: crate::map_checksum::sha256_bytes(event.data.as_ref()),
        boundary_sha256: [0; 32],
    };
    drop(shared);
    let mut committed_map = committed_map;
    committed_map.boundary_sha256 =
        crate::correction_coordinator::expected_map_boundary_sha256(map_frame, &committed_map)
            .map_err(|error| error.to_string())?;
    let input = crate::correction_coordinator::CorrectionFrameInput {
        mapping_session_id: binding.mapping_session_id.clone(),
        mapping_session_generation: binding.mapping_session_generation,
        mapping_commit_id: occupancy_receipt.commit_id,
        occupancy_revision: occupancy_receipt.occupancy_revision,
        cloud_revision: cloud.receipt.cloud_revision,
        map_frame_generation: binding.map_frame_generation,
        registered_cloud: crate::correction_coordinator::RegisteredCloudProof {
            source_sequence: u64::try_from(binding.cloud_source_sequence)
                .map_err(|_| "registered cloud sequence is negative".to_owned())?,
            source_timestamp_ns,
            point_count: u64::try_from(body_points.len())
                .map_err(|_| "registered cloud point count overflow".to_owned())?,
            envelope_sha256: binding.envelope_sha256,
            body_points_sha256: crate::correction_coordinator::body_points_sha256(&body_points)
                .map_err(|error| error.to_string())?,
        },
        correlated_odom: crate::correction_coordinator::FrozenCorrelatedOdom {
            proof: crate::correction_coordinator::CorrelatedOdomProof {
                source_sequence: u64::try_from(binding.correlated_odometry.source_sequence)
                    .map_err(|_| "correlated odometry sequence is negative".to_owned())?,
                source_timestamp_ns: u64::try_from(binding.correlated_odometry.source_timestamp_ns)
                    .map_err(|_| "correlated odometry timestamp is negative".to_owned())?,
                pose_sha256: crate::correction_coordinator::odom_pose_sha256(
                    &binding.frozen_odom_from_body,
                )
                .map_err(|error| error.to_string())?,
            },
            odom_from_body: binding.frozen_odom_from_body,
        },
        committed_map,
        committed_map_occupancy: Arc::clone(&event.data),
        points_frame_id: body_frame.to_owned(),
        body_points,
        base_anchor_map_from_odom: binding.base_anchor_map_from_odom,
        frame_capture_relocalization_epoch: frame_capture_epoch,
        relocalization_mode_epoch: None,
    };
    match correction
        .submit_with_lifecycle_guard(input, lifecycle_guard)
        .await
    {
        Ok(_) | Err(crate::correction_coordinator::CorrectionRuntimeError::Stopped) => {}
        Err(error) => return Err(error.to_string()),
    }
    Ok(true)
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn handoff_registered_lio_cloud_replay_with_correction(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    adapter_timestamp_ns: u64,
    source_timestamp_ns: u64,
    map_frame: &str,
    body_frame: &str,
    binding: &crate::lio_ingress::RegistrationBinding,
    correction: &crate::correction_coordinator::CorrectionFrameIngress,
) -> Result<bool, String> {
    let navigation_context = handle.navigation_map_3d_publish_context();
    let initial_mode = correction.relocalization_mode_snapshot();
    let Some(frame_capture_epoch) = binding.frame_capture_relocalization_epoch else {
        return Ok(false);
    };
    if frame_capture_epoch != initial_mode.epoch {
        return Ok(false);
    }
    if initial_mode.active {
        let lifecycle_guard = handle.mapping_lifecycle_lock.clone().lock_owned().await;
        let active_mode = correction.relocalization_mode_snapshot();
        if active_mode == initial_mode {
            let frame = registered_lio_read_only_correction_frame(
                handle,
                points,
                source_timestamp_ns,
                map_frame,
                body_frame,
                binding,
                active_mode.epoch,
            )
            .await?;
            if correction.relocalization_mode_snapshot() != active_mode {
                return Ok(false);
            }
            correction
                .submit_with_lifecycle_guard(frame.input, lifecycle_guard)
                .await
                .map_err(|error| error.to_string())?;
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            if correction.relocalization_mode_snapshot() != active_mode {
                return Ok(false);
            }
            publish_read_only_mapping_cloud_while_lifecycle_fenced(
                handle,
                map_frame,
                binding,
                &frame.cloud,
                ExpectedRelocalizationMode {
                    fence: correction.relocalization_mode_fence(),
                    epoch: active_mode.epoch,
                    active: active_mode.active,
                },
                navigation_context.as_ref(),
            )
            .await?;
            return Ok(true);
        }
    }

    let commit = commit_registered_lio_cloud_inner(
        handle,
        points,
        point_step,
        adapter_timestamp_ns,
        source_timestamp_ns,
        map_frame,
        Some((
            binding.mapping_session_generation,
            binding.map_frame_generation,
        )),
        Some(initial_mode.epoch),
        Some(correction.relocalization_mode_fence()),
        binding.sensor_origin_map,
    )
    .await?;
    if commit.outcome.occupancy_event.is_none() {
        return Ok(false);
    }

    {
        let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        if correction.relocalization_mode_snapshot() != initial_mode {
            return Ok(false);
        }
        ensure_replay_mapping_identity(handle, binding)?;
    }
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
    publish_mapping_ingest_commit_with_lifecycle_fence(
        &handle.transport,
        mapping,
        &handle.mapping_lifecycle_lock,
        &handle.cloud_map_writer,
        &handle.state,
        &handle.cloud_map_tx,
        &handle.map_revision_tx,
        map_frame,
        &commit,
        Some(binding),
        Some(ExpectedRelocalizationMode {
            fence: correction.relocalization_mode_fence(),
            epoch: initial_mode.epoch,
            active: initial_mode.active,
        }),
        navigation_context.as_ref(),
    )
    .await?;
    let input = registered_lio_correction_input(
        handle,
        points,
        source_timestamp_ns,
        map_frame,
        body_frame,
        binding,
        &commit,
    )
    .await?;
    correction
        .submit(input)
        .await
        .map_err(|error| error.to_string())?;
    {
        let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        if correction.relocalization_mode_snapshot() != initial_mode {
            return Ok(false);
        }
        ensure_replay_mapping_identity(handle, binding)?;
    }
    Ok(true)
}

fn ensure_replay_mapping_identity(
    handle: &GeometryRuntimeHandle,
    binding: &crate::lio_ingress::RegistrationBinding,
) -> Result<(), String> {
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
    let mapping = mapping.lock();
    let snapshot = mapping.snapshot();
    if !snapshot.active
        || snapshot.session_id != binding.mapping_session_id
        || snapshot.session_generation != binding.mapping_session_generation
        || mapping.global_frame_generation() != binding.map_frame_generation
    {
        return Err("deferred correction mapping identity changed".to_owned());
    }
    Ok(())
}

/// Revalidate a frozen read-only relocalization payload while the caller owns
/// `mapping_lifecycle_lock`.  A session/generation check alone is not enough:
/// a normal mapping commit could otherwise advance the receipt and let an old
/// `/global_map` snapshot publish after the new revision.
async fn ensure_read_only_mapping_identity(
    handle: &GeometryRuntimeHandle,
    binding: &crate::lio_ingress::RegistrationBinding,
    expected_cloud: &CommittedMappingCloudMap,
) -> Result<(), String> {
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
    let (mapping_snapshot, map_frame_generation, occupancy_receipt, configured_map_frame) = {
        let mapping = mapping.lock();
        (
            mapping.snapshot(),
            mapping.global_frame_generation(),
            mapping.latest_occupancy_commit_receipt(),
            normalize_frame(mapping.configured_map_frame_id()),
        )
    };
    let occupancy_receipt = occupancy_receipt
        .ok_or_else(|| "relocalization current occupancy receipt is unavailable".to_owned())?;
    let (cloud_snapshot, cloud_receipt) = {
        let writer = handle.cloud_map_writer.lock();
        let writer = writer
            .as_ref()
            .ok_or_else(|| "relocalization current cloud-map writer is unavailable".to_owned())?;
        (
            writer.committed_snapshot().ok_or_else(|| {
                "relocalization current cloud-map snapshot is unavailable".to_owned()
            })?,
            writer.last_cloud_receipt().ok_or_else(|| {
                "relocalization current cloud-map receipt is unavailable".to_owned()
            })?,
        )
    };
    let shared = handle.state.read().await;
    if !mapping_snapshot.active
        || mapping_snapshot.session_id != binding.mapping_session_id
        || mapping_snapshot.session_generation != binding.mapping_session_generation
        || map_frame_generation != binding.map_frame_generation
        || mapping_snapshot.revision != occupancy_receipt.occupancy_revision
        || occupancy_receipt.commit_id != expected_cloud.receipt.commit_id
        || occupancy_receipt.session_id != binding.mapping_session_id
        || occupancy_receipt.session_generation != binding.mapping_session_generation
        || occupancy_receipt.occupancy_revision != expected_cloud.receipt.occupancy_revision
        || occupancy_receipt.source_timestamp_ns != expected_cloud.receipt.source_timestamp_ns
        || cloud_receipt.commit_id != expected_cloud.receipt.commit_id
        || cloud_receipt.session_generation != binding.mapping_session_generation
        || cloud_receipt.occupancy_revision != expected_cloud.receipt.occupancy_revision
        || cloud_receipt.cloud_revision != expected_cloud.receipt.cloud_revision
        || cloud_receipt.source_timestamp_ns != expected_cloud.receipt.source_timestamp_ns
        || cloud_receipt.point_count != expected_cloud.receipt.point_count
        || cloud_snapshot.revision != expected_cloud.notification.revision
        || cloud_snapshot.point_count != expected_cloud.notification.point_count
        || cloud_snapshot.voxel_size.to_bits() != expected_cloud.notification.voxel_size.to_bits()
        || cloud_snapshot.session_generation != expected_cloud.notification.session_generation
        || cloud_snapshot.xyz_f32_le != expected_cloud.xyz_f32_le
        || shared.map.revision != expected_cloud.receipt.occupancy_revision
        || shared.map.timestamp_ns != expected_cloud.receipt.source_timestamp_ns
        || normalize_frame(&shared.map.frame_id) != configured_map_frame
        || shared.cloud_map_revision != expected_cloud.receipt.cloud_revision
    {
        return Err("relocalization read-only map receipt changed before publication".to_owned());
    }
    Ok(())
}

async fn registered_lio_correction_input(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    source_timestamp_ns: u64,
    map_frame: &str,
    body_frame: &str,
    binding: &crate::lio_ingress::RegistrationBinding,
    commit: &MappingIngestCommit,
) -> Result<crate::correction_coordinator::CorrectionFrameInput, String> {
    let frame_capture_epoch = binding
        .frame_capture_relocalization_epoch
        .ok_or_else(|| "registered LIO correction lacks frame capture provenance".to_owned())?;
    let event = commit
        .outcome
        .occupancy_event
        .as_ref()
        .ok_or_else(|| "registered LIO correction lost its occupancy event".to_owned())?;
    let occupancy_receipt = commit
        .outcome
        .occupancy_receipt
        .as_ref()
        .ok_or_else(|| "registered LIO correction lacks an occupancy receipt".to_owned())?;
    let cloud = commit
        .cloud_commit
        .as_ref()
        .ok_or_else(|| "registered LIO correction lacks a cloud receipt".to_owned())?;
    if occupancy_receipt.commit_id != cloud.receipt.commit_id
        || occupancy_receipt.session_id != binding.mapping_session_id
        || occupancy_receipt.session_generation != binding.mapping_session_generation
        || cloud.receipt.session_generation != binding.mapping_session_generation
        || occupancy_receipt.occupancy_revision != event.revision
        || cloud.receipt.occupancy_revision != event.revision
        || cloud.receipt.source_timestamp_ns != source_timestamp_ns
        || occupancy_receipt.source_timestamp_ns != source_timestamp_ns
        || event.timestamp_ns != source_timestamp_ns
    {
        return Err("registered LIO correction receipt identity mismatch".to_owned());
    }

    let map_from_body = binding.base_anchor_map_from_odom * binding.frozen_odom_from_body;
    let body_from_map = map_from_body.inverse();
    let body_points = points
        .iter()
        .map(|point| body_from_map.transform_point(point))
        .collect::<Vec<_>>();
    let shared = handle.state.read().await;
    if shared.map.revision != event.revision
        || shared.map.timestamp_ns != source_timestamp_ns
        || normalize_frame(&shared.map.frame_id) != normalize_frame(map_frame)
        || shared.cloud_map_revision != cloud.receipt.cloud_revision
    {
        return Err("canonical map/cloud state does not match correction receipts".to_owned());
    }
    let committed_map = crate::correction_coordinator::CommittedMapProof {
        source_timestamp_ns,
        width: u64::from(shared.map.width),
        height: u64::from(shared.map.height),
        resolution_m: f64::from(shared.map.resolution),
        origin_x_m: shared.map.origin_x,
        origin_y_m: shared.map.origin_y,
        payload_sha256: crate::map_checksum::sha256_bytes(event.data.as_ref()),
        boundary_sha256: [0; 32],
    };
    drop(shared);
    let mut committed_map = committed_map;
    committed_map.boundary_sha256 =
        crate::correction_coordinator::expected_map_boundary_sha256(map_frame, &committed_map)
            .map_err(|error| error.to_string())?;
    Ok(crate::correction_coordinator::CorrectionFrameInput {
        mapping_session_id: binding.mapping_session_id.clone(),
        mapping_session_generation: binding.mapping_session_generation,
        mapping_commit_id: occupancy_receipt.commit_id,
        occupancy_revision: occupancy_receipt.occupancy_revision,
        cloud_revision: cloud.receipt.cloud_revision,
        map_frame_generation: binding.map_frame_generation,
        registered_cloud: crate::correction_coordinator::RegisteredCloudProof {
            source_sequence: u64::try_from(binding.cloud_source_sequence)
                .map_err(|_| "registered cloud sequence is negative".to_owned())?,
            source_timestamp_ns,
            point_count: u64::try_from(body_points.len())
                .map_err(|_| "registered cloud point count overflow".to_owned())?,
            envelope_sha256: binding.envelope_sha256,
            body_points_sha256: crate::correction_coordinator::body_points_sha256(&body_points)
                .map_err(|error| error.to_string())?,
        },
        correlated_odom: crate::correction_coordinator::FrozenCorrelatedOdom {
            proof: crate::correction_coordinator::CorrelatedOdomProof {
                source_sequence: u64::try_from(binding.correlated_odometry.source_sequence)
                    .map_err(|_| "correlated odometry sequence is negative".to_owned())?,
                source_timestamp_ns: u64::try_from(binding.correlated_odometry.source_timestamp_ns)
                    .map_err(|_| "correlated odometry timestamp is negative".to_owned())?,
                pose_sha256: crate::correction_coordinator::odom_pose_sha256(
                    &binding.frozen_odom_from_body,
                )
                .map_err(|error| error.to_string())?,
            },
            odom_from_body: binding.frozen_odom_from_body,
        },
        committed_map,
        committed_map_occupancy: Arc::clone(&event.data),
        points_frame_id: body_frame.to_owned(),
        body_points,
        base_anchor_map_from_odom: binding.base_anchor_map_from_odom,
        frame_capture_relocalization_epoch: frame_capture_epoch,
        relocalization_mode_epoch: None,
    })
}

struct ReadOnlyRegisteredLioCorrectionFrame {
    input: crate::correction_coordinator::CorrectionFrameInput,
    cloud: CommittedMappingCloudMap,
}

async fn registered_lio_read_only_correction_frame(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    source_timestamp_ns: u64,
    map_frame: &str,
    body_frame: &str,
    binding: &crate::lio_ingress::RegistrationBinding,
    mode_epoch: u64,
) -> Result<ReadOnlyRegisteredLioCorrectionFrame, String> {
    if binding.frame_capture_relocalization_epoch != Some(mode_epoch) {
        return Err("relocalization frame capture mode epoch changed".to_owned());
    }
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
    let (mapping_snapshot, map_frame_generation, occupancy_receipt) = {
        let mapping = mapping.lock();
        (
            mapping.snapshot(),
            mapping.global_frame_generation(),
            mapping.latest_occupancy_commit_receipt(),
        )
    };
    let occupancy_receipt = occupancy_receipt
        .ok_or_else(|| "relocalization requires a committed occupancy receipt".to_owned())?;
    if !mapping_snapshot.active
        || mapping_snapshot.session_id != binding.mapping_session_id
        || mapping_snapshot.session_generation != binding.mapping_session_generation
        || map_frame_generation != binding.map_frame_generation
    {
        return Err("relocalization mapping identity changed".to_owned());
    }

    let (cloud_snapshot, cloud_receipt) = {
        let writer = handle.cloud_map_writer.lock();
        let writer = writer
            .as_ref()
            .ok_or_else(|| "relocalization requires the canonical cloud-map writer".to_owned())?;
        let snapshot = writer
            .committed_snapshot()
            .ok_or_else(|| "relocalization requires a committed cloud-map snapshot".to_owned())?;
        let receipt = writer
            .last_cloud_receipt()
            .ok_or_else(|| "relocalization requires a committed cloud-map receipt".to_owned())?;
        (snapshot, receipt)
    };
    let map = {
        let shared = handle.state.read().await;
        if shared.cloud_map_revision != cloud_snapshot.revision {
            return Err("relocalization cloud-map state does not match its snapshot".to_owned());
        }
        shared.map.clone()
    };
    if mapping_snapshot.revision != map.revision
        || occupancy_receipt.commit_id != cloud_receipt.commit_id
        || occupancy_receipt.session_id != binding.mapping_session_id
        || occupancy_receipt.session_generation != binding.mapping_session_generation
        || occupancy_receipt.occupancy_revision != map.revision
        || occupancy_receipt.source_timestamp_ns != map.timestamp_ns
        || cloud_receipt.session_generation != binding.mapping_session_generation
        || cloud_receipt.occupancy_revision != occupancy_receipt.occupancy_revision
        || cloud_receipt.cloud_revision != cloud_snapshot.revision
        || cloud_receipt.source_timestamp_ns != occupancy_receipt.source_timestamp_ns
        || cloud_receipt.point_count != cloud_snapshot.point_count
        || cloud_snapshot.session_generation != binding.mapping_session_generation
        || map.timestamp_ns > source_timestamp_ns
        || normalize_frame(&map.frame_id) != normalize_frame(map_frame)
    {
        return Err("relocalization committed map/cloud receipt identity mismatch".to_owned());
    }

    let map_from_body = binding.base_anchor_map_from_odom * binding.frozen_odom_from_body;
    let body_from_map = map_from_body.inverse();
    let body_points = points
        .iter()
        .map(|point| body_from_map.transform_point(point))
        .collect::<Vec<_>>();
    let committed_map_occupancy: Arc<[u8]> = map
        .data
        .iter()
        .map(|value| *value as u8)
        .collect::<Vec<_>>()
        .into();
    let mut committed_map = crate::correction_coordinator::CommittedMapProof {
        source_timestamp_ns: map.timestamp_ns,
        width: u64::from(map.width),
        height: u64::from(map.height),
        resolution_m: f64::from(map.resolution),
        origin_x_m: map.origin_x,
        origin_y_m: map.origin_y,
        payload_sha256: crate::map_checksum::sha256_bytes(committed_map_occupancy.as_ref()),
        boundary_sha256: [0; 32],
    };
    committed_map.boundary_sha256 =
        crate::correction_coordinator::expected_map_boundary_sha256(map_frame, &committed_map)
            .map_err(|error| error.to_string())?;
    let input = crate::correction_coordinator::CorrectionFrameInput {
        mapping_session_id: binding.mapping_session_id.clone(),
        mapping_session_generation: binding.mapping_session_generation,
        mapping_commit_id: occupancy_receipt.commit_id,
        occupancy_revision: occupancy_receipt.occupancy_revision,
        cloud_revision: cloud_receipt.cloud_revision,
        map_frame_generation: binding.map_frame_generation,
        registered_cloud: crate::correction_coordinator::RegisteredCloudProof {
            source_sequence: u64::try_from(binding.cloud_source_sequence)
                .map_err(|_| "registered cloud sequence is negative".to_owned())?,
            source_timestamp_ns,
            point_count: u64::try_from(body_points.len())
                .map_err(|_| "registered cloud point count overflow".to_owned())?,
            envelope_sha256: binding.envelope_sha256,
            body_points_sha256: crate::correction_coordinator::body_points_sha256(&body_points)
                .map_err(|error| error.to_string())?,
        },
        correlated_odom: crate::correction_coordinator::FrozenCorrelatedOdom {
            proof: crate::correction_coordinator::CorrelatedOdomProof {
                source_sequence: u64::try_from(binding.correlated_odometry.source_sequence)
                    .map_err(|_| "correlated odometry sequence is negative".to_owned())?,
                source_timestamp_ns: u64::try_from(binding.correlated_odometry.source_timestamp_ns)
                    .map_err(|_| "correlated odometry timestamp is negative".to_owned())?,
                pose_sha256: crate::correction_coordinator::odom_pose_sha256(
                    &binding.frozen_odom_from_body,
                )
                .map_err(|error| error.to_string())?,
            },
            odom_from_body: binding.frozen_odom_from_body,
        },
        committed_map,
        committed_map_occupancy,
        points_frame_id: body_frame.to_owned(),
        body_points,
        base_anchor_map_from_odom: binding.base_anchor_map_from_odom,
        frame_capture_relocalization_epoch: mode_epoch,
        relocalization_mode_epoch: Some(mode_epoch),
    };
    Ok(ReadOnlyRegisteredLioCorrectionFrame {
        input,
        cloud: CommittedMappingCloudMap {
            notification: CloudMapNotification {
                revision: cloud_snapshot.revision,
                point_count: cloud_snapshot.point_count,
                voxel_size: cloud_snapshot.voxel_size,
                session_generation: cloud_snapshot.session_generation,
            },
            source_timestamp_ns: cloud_receipt.source_timestamp_ns,
            xyz_f32_le: cloud_snapshot.xyz_f32_le,
            receipt: cloud_receipt,
        },
    })
}

async fn handoff_registered_lio_cloud_inner(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    adapter_timestamp_ns: u64,
    source_timestamp_ns: u64,
    map_frame: &str,
    expected_generations: Option<(u64, u64)>,
    sensor_origin_map: Option<[f64; 3]>,
) -> Result<bool, String> {
    let commit = commit_registered_lio_cloud_inner(
        handle,
        points,
        point_step,
        adapter_timestamp_ns,
        source_timestamp_ns,
        map_frame,
        expected_generations,
        None,
        None,
        sensor_origin_map,
    )
    .await?;
    let occupancy_committed = commit.outcome.occupancy_event.is_some();
    let accepted = occupancy_committed || !commit.outcome.accepted_map_points.is_empty();
    publish_registered_lio_commit(handle, map_frame, &commit).await?;
    if accepted && !occupancy_committed {
        debug!(
            event = "lio.cloud.accepted_without_revision",
            reason = "publication_cadence",
            accepted_points = commit.outcome.accepted_map_points.len(),
            source_timestamp_ns,
            map_frame,
            "实时点云已接纳，等待下一次地图发布"
        );
    }
    Ok(accepted)
}

#[allow(clippy::too_many_arguments)]
async fn commit_registered_lio_cloud_inner(
    handle: &GeometryRuntimeHandle,
    points: &[nalgebra::Point3<f64>],
    point_step: u32,
    adapter_timestamp_ns: u64,
    source_timestamp_ns: u64,
    map_frame: &str,
    expected_generations: Option<(u64, u64)>,
    expected_relocalization_epoch: Option<u64>,
    relocalization_mode: Option<Arc<crate::correction_coordinator::RelocalizationModeFence>>,
    sensor_origin_map: Option<[f64; 3]>,
) -> Result<MappingIngestCommit, String> {
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_string())?;
    let outcome = ingest_mapping_points_with_backend(
        mapping,
        &handle.mapping_lifecycle_lock,
        &handle.explore_projection_gate,
        &handle.committed_map_authority,
        &handle.map_revision_latest_tx,
        &handle.cloud_map_writer,
        points,
        point_step,
        adapter_timestamp_ns,
        map_frame,
        source_timestamp_ns,
        &handle.state,
        true,
        expected_generations,
        expected_relocalization_epoch,
        relocalization_mode,
        handle.task_control.as_ref(),
        sensor_origin_map,
    )
    .await
    .map_err(|error| error.to_string())?;
    Ok(outcome)
}

async fn publish_registered_lio_commit(
    handle: &GeometryRuntimeHandle,
    map_frame: &str,
    commit: &MappingIngestCommit,
) -> Result<(), String> {
    let navigation_context = handle.navigation_map_3d_publish_context();
    let mapping = handle
        .mapping_adapter
        .as_ref()
        .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
    publish_mapping_ingest_commit_with_lifecycle_fence(
        &handle.transport,
        mapping,
        &handle.mapping_lifecycle_lock,
        &handle.cloud_map_writer,
        &handle.state,
        &handle.cloud_map_tx,
        &handle.map_revision_tx,
        map_frame,
        commit,
        None,
        None,
        navigation_context.as_ref(),
    )
    .await
}

/// Phase 3: 把 LCM `/odom` 与 `/go2/lidar/points` typed channel 喂给 mapping adapter。
///
/// MappingAdapter 是 occupancy revision 的唯一 writer；`/map/occupancy` 仅为
/// 输出 fanout，不作为输入回灌。mapping feed 累积点云、维护
/// `last_sensor_origin`，然后调用 `build_and_publish` 写回 `SharedState.map`。
///
/// - `/odom` payload 走 `parse_odom_payload` → 归一化 quaternion/yaw →
///   `feed_pose_in_frame`，并从固定 anchor 发布 map→odom→base canonical TF
/// - `/go2/lidar/points` payload 走 `parse_pointcloud_payload` → 转 Point3 →
///   `feed_pointcloud_session` + 增量 cloud-map voxel + `build_and_publish`
///
/// 点云 feed 仍持 parking_lot Mutex；里程计在锁繁忙时使用同会话的只读
/// map<-odom 缓存继续发布 canonical pose/TF，避免栅格重建阻塞导航位姿心跳。
/// `/pose_stamped` 不再具有全局位姿写权限。
fn spawn_mapping_feed_tasks(
    transport: LcmTransport,
    state: Arc<RwLock<SharedState>>,
    mapping_arc: Arc<parking_lot::Mutex<MappingAdapter>>,
    map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    committed_map_authority: Arc<std::sync::Mutex<Option<schema::CommittedMapIdentity>>>,
    explore_projection_gate: Arc<tokio::sync::Mutex<ExploreProjectionGate>>,
    odom_binding: Option<(LcmTopic, LcmMessageType)>,
    pointcloud_binding: (LcmTopic, LcmMessageType),
    tf_binding: (LcmTopic, LcmMessageType),
    base_frame: String,
    cloud_map_writer: Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
    cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) {
    use futures::StreamExt;
    use nalgebra::Point3;

    // ---- /odom → feed_pose ----
    if let Some((odom_topic, odom_msg_type)) = odom_binding {
        let transport_odom = transport.clone();
        let mapping_odom = mapping_arc.clone();
        let transform_cache = mapping_odom.lock().map_odom_transform_cache();
        let state_odom = state.clone();
        let mapping_lifecycle_lock_odom = mapping_lifecycle_lock.clone();
        let (tf_topic, tf_msg_type) = tf_binding;
        tokio::spawn(async move {
            let mut stream = transport_odom
                .subscribe(odom_topic, Some(odom_msg_type))
                .await;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => {
                        error!("mapping_feed /odom LCM 订阅错误: {}", e);
                        continue;
                    }
                };
                let Some(pose) = parse_odom_payload(&frame.payload) else {
                    warn!("mapping_feed /odom payload decode failed");
                    continue;
                };
                let _lifecycle_guard = mapping_lifecycle_lock_odom.lock().await;
                if mapping_feed_should_wait_for_active_session(mapping_odom.as_ref()) {
                    continue;
                }
                let observation = canonical_mapping_observation_for_odom(
                    mapping_odom.as_ref(),
                    transform_cache.as_ref(),
                    &pose,
                    &base_frame,
                );
                let observation = match observation {
                    Ok(observation) => observation,
                    Err(error) => {
                        warn!(%error, frame = %pose.frame_id, "mapping_feed /odom rejected");
                        continue;
                    }
                };
                apply_canonical_mapping_observation(&state_odom, &observation).await;
                let tf_frame = crate::typed_codec::build_typed_frame(
                    tf_topic.clone(),
                    tf_msg_type.clone(),
                    observation.tf_payload,
                    pose.timestamp_ns,
                );
                match tokio::time::timeout(
                    MAP_FANOUT_SINK_TIMEOUT,
                    transport_odom.publish(tf_frame),
                )
                .await
                {
                    Ok(Ok(())) => {}
                    Ok(Err(error)) => warn!(%error, "canonical mapping TF publish failed"),
                    Err(_) => warn!("canonical mapping TF publish timed out"),
                }
            }
        });
    } else {
        warn!("mapping_feed: odometry binding unavailable; odom anchor/TF feed disabled");
    }

    // ---- /go2/lidar/points → feed_pointcloud + build_and_publish ----
    let (lidar_topic, lidar_msg_type) = pointcloud_binding;
    let transport_lidar = transport.clone();
    let mapping_lidar = mapping_arc.clone();
    let mapping_lidar_frame = mapping_lidar.lock().configured_map_frame_id().to_string();
    let state_lidar = state.clone();
    let map_revision_lidar = map_revision_tx.clone();
    let map_revision_latest_lidar = map_revision_latest_tx;
    let committed_map_authority_lidar = committed_map_authority;
    let explore_projection_gate_lidar = explore_projection_gate;
    tokio::spawn(async move {
        let mut stream = transport_lidar
            .subscribe(lidar_topic, Some(lidar_msg_type))
            .await;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => {
                    error!("mapping_feed /go2/lidar/points LCM 订阅错误: {}", e);
                    continue;
                }
            };
            let parsed = match parse_pointcloud_payload(&frame.payload) {
                Some(parsed) => parsed,
                None => continue,
            };
            let ps = parsed.point_step as usize;
            if ps < 12 {
                continue;
            }
            let raw_count = parsed.data.len() / ps;
            // 防御性 sanity cap:见 POINTCLOUD_MAX_FRAME_POINTS 注释。
            // raw_count 由 untrusted bytes.len()/point_step 推得,point_step
            // 已在 parse_pointcloud_payload 入口校验；此处保留二次校验，
            // 防止后续解析调整绕过单帧上限。
            if raw_count > crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS {
                warn!(
                    "mapping_feed /go2/lidar/points raw_count={} 超过 sanity cap={};按异常帧跳过",
                    raw_count,
                    crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS
                );
                continue;
            }
            let mut points: Vec<Point3<f64>> = Vec::with_capacity(raw_count);
            for i in 0..raw_count {
                let off = i * ps;
                if off + 12 > parsed.data.len() {
                    break;
                }
                let x = f32::from_le_bytes(parsed.data[off..off + 4].try_into().unwrap_or([0; 4]))
                    as f64;
                let y =
                    f32::from_le_bytes(parsed.data[off + 4..off + 8].try_into().unwrap_or([0; 4]))
                        as f64;
                let z =
                    f32::from_le_bytes(parsed.data[off + 8..off + 12].try_into().unwrap_or([0; 4]))
                        as f64;
                if x.is_nan() || y.is_nan() || z.is_nan() {
                    continue;
                }
                points.push(Point3::new(x, y, z));
            }
            if points.is_empty() {
                continue;
            }
            // MappingSession 是生产建图的唯一点云数据源。只有 session 接受了
            // 新点后才构建 revision，dedup/stale/resource-exhausted 帧不会发布假更新。
            let now_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;
            let ingest_outcome = ingest_mapping_points_with_backend(
                &mapping_lidar,
                &mapping_lifecycle_lock,
                &explore_projection_gate_lidar,
                &committed_map_authority_lidar,
                &map_revision_latest_lidar,
                &cloud_map_writer,
                &points,
                parsed.point_step,
                now_ns,
                &parsed.frame_id,
                parsed.timestamp_ns,
                &state_lidar,
                false,
                None,
                None,
                None,
                None,
                None,
            )
            .await;
            match ingest_outcome {
                Ok(commit) => {
                    let expected = commit.cloud_commit.as_ref().map(|cloud| {
                        (
                            cloud.notification.revision,
                            cloud.notification.session_generation,
                        )
                    });
                    if let Err(error) = publish_mapping_ingest_commit_with_lifecycle_fence(
                        &transport_lidar,
                        &mapping_lidar,
                        &mapping_lifecycle_lock,
                        &cloud_map_writer,
                        &state_lidar,
                        &cloud_map_tx,
                        &map_revision_lidar,
                        &mapping_lidar_frame,
                        &commit,
                        None,
                        None,
                        None,
                    )
                    .await
                    {
                        warn!(
                            %error,
                            revision = expected.map(|(revision, _)| revision),
                            session_generation = expected.map(|(_, session_generation)| session_generation),
                            "committed cloud-map revision was not delivered to typed /global_map"
                        );
                    }
                }
                Err(error) => {
                    warn!("mapping_feed: MappingSession 拒绝点云帧: {error}");
                }
            }
        }
    });
}

/// 单测端口隔离:Architect R1 防止重现 07_08 实机 175 GB 崩溃。
///
/// `pid * 10000 + counter` 让 cargo test 的 N 个并行 test 进程分到不同端口,
/// 避免互相 bind 失败 / udpm 重复收包导致 Rust 进程内存爆涨。
///
/// 所有 `build_handle_for_tests` 走 real-lcm，必须用隔离 URL 防止
/// cargo test 并发撞端口。
///
/// 不加 `#[cfg(test)]`:函数本身只返回 String(无 test-only 依赖),
/// 公开供 `build_handle_for_tests` 跨 #[cfg(test)] 模块引用,且允许 binary
/// 在测试场景外显式构造测试 URL 用。
pub fn test_lcm_url() -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let pid = std::process::id() as u64;
    let counter = COUNTER.fetch_add(1, Ordering::Relaxed);
    let port = (pid * 10_000 + counter) % 60_000 + 7_667; // [7667, 67666]
    format!("udpm://239.255.76.67:{}?ttl=1", port)
}

/// 单元测试 helper:构造一个最简 `GeometryRuntimeHandle`。
///
/// 不会启动订阅任务,只提供 real-lcm backend(test_lcm_url 隔离端口) +
/// 全新 `SharedState`,供 `nav_runtime` / `mapping_adapter` / 其它模块做集成测试。
/// test helper 走 `LcmTransport::new_real`，真实验证 liblcm 行为。
///
/// 与 `spawn_geometry_runtime` 的关键差异:
/// - 不启动订阅任务
/// - 默认创建隔离的 cloud-map SHM；`build_handle_for_tests_without_cloud_writer` 覆盖 writer 缺失路径
/// - 立即返回,适合 `#[tokio::test]` async 测试
pub async fn build_handle_for_tests() -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    build_handle_for_tests_with_cloud_writer(true).await
}

/// 构造测试句柄并显式禁用 cloud-map writer，覆盖 fail-closed 路径。
pub async fn build_handle_for_tests_without_cloud_writer(
) -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    build_handle_for_tests_with_cloud_writer(false).await
}

async fn build_handle_for_tests_with_cloud_writer(
    with_cloud_writer: bool,
) -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavigationUpdate>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);
    let (map_revision_tx, _) = broadcast::channel::<MapRevisionEvent>(16);
    let map_revision_fanout_rx = map_revision_tx.subscribe();
    let (map_revision_latest_tx, _) = watch::channel::<Option<MapRevisionEvent>>(None);
    let committed_map_authority = Arc::new(std::sync::Mutex::new(None));
    let explore_projection_gate =
        Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default()));
    let state = Arc::new(RwLock::new(SharedState::default()));
    static SEQUENCE_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let sequence_state_path = std::env::temp_dir()
        .join(format!(
            "nuwax-geometry-sequence-test-{}-{}",
            std::process::id(),
            SEQUENCE_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        ))
        .join("sequence_state.json");
    let sequence_store = Arc::new(
        SequenceStateStore::open(&sequence_state_path)
            .expect("test sequence state must be writable and exclusively lockable"),
    );
    // test_lcm_url() 端口公式防止 cargo test 并发端口冲突。
    let transport = LcmTransport::new_real(&crate::lcm_runtime::test_lcm_url())
        .expect("test real LCM 启动失败,需先 apt-get install -y liblcm-dev 1.3~1.5");
    // Phase 3: 测试 helper 也构造 mapping_adapter,便于 unit test 走完整链路。
    let mapping_adapter = Some(Arc::new(parking_lot::Mutex::new(
        MappingAdapter::new_with_frames_and_sequence_store(
            MappingAdapterConfig::from_env(),
            "map",
            "odom",
            sequence_store.clone(),
        ),
    )));
    // Phase 3: 测试 helper 也构造 occupancy_shm(走 /tmp 测试路径)
    // 注意:必须包含 per-call 唯一后缀(monotonic counter),否则并行
    // cargo test 跑多次 build_handle_for_tests 时,多个
    // OccupancyGridShmWriter::new() 会同时 truncate 同一文件 + 同一
    // mmap 段,导致 SIGBUS(实测:28 tests pass 后崩溃)。
    static OCC_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let occ_test_path = std::path::PathBuf::from(format!(
        "/tmp/nuwax_occupancy_test_{}_{}.mmap",
        std::process::id(),
        OCC_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
    ));
    let occupancy_shm = match OccupancyGridShmWriter::new(&occ_test_path) {
        Ok(w) => Some(Arc::new(parking_lot::Mutex::new(w))),
        Err(_) => None,
    };
    let occupancy_shm_path = occupancy_shm
        .as_ref()
        .map(|_| occ_test_path.to_string_lossy().into_owned());
    let occupancy_shm_current = Arc::new(RwLock::new(None));
    let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let mapping_lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
    spawn_map_revision_fanout(
        map_revision_fanout_rx,
        map_revision_latest_tx.clone(),
        committed_map_authority.clone(),
        explore_projection_gate.clone(),
        transport.clone(),
        LcmTopic::new(&format!(
            "/test/geometry/occupancy/helper/{}",
            std::process::id()
        ))
        .expect("test occupancy topic"),
        LcmMessageType::new("nav_msgs.OccupancyGrid").expect("test occupancy type"),
        occupancy_shm.clone(),
        occupancy_shm_path.clone(),
        occupancy_shm_current.clone(),
        explore.clone(),
        mapping_adapter
            .as_ref()
            .expect("test mapping adapter")
            .clone(),
        mapping_lifecycle_lock.clone(),
    );
    let cloud_map_writer = if with_cloud_writer {
        static CM_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let cloud_map_path = std::path::PathBuf::from(format!(
            "/tmp/nuwax_cloudmap_test_{}_{}.mmap",
            std::process::id(),
            CM_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        ));
        let cloud_map_writer = CloudMapShmWriter::new(&cloud_map_path).map_err(|error| {
            RealLcmBackendUnavailable::persistence(format!(
                "test cloud-map SHM initialization failed: {error}"
            ))
        })?;
        Arc::new(parking_lot::Mutex::new(Some(cloud_map_writer)))
    } else {
        Arc::new(parking_lot::Mutex::new(None))
    };
    Ok(GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        cloud_map_writer,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx,
        map_revision_latest_tx,
        explore_projection_gate,
        committed_map_authority,
        transport,
        transport_backend_name: "real-lcm",
        lcm_backend_available: true,
        // Phase 6 / Rework-9:test helper 默认保留 `true`。
        // 配套测试用 `set_cmd_vel_sink_ready(false)` 验证 readiness 拒绝分支。
        // 生产 spawn 路径(`spawn_geometry_runtime`)默认 `false`。
        cmd_vel_sink_ready: Arc::new(std::sync::atomic::AtomicBool::new(true)),
        nav: crate::nav_runtime::NavRuntimeState::try_from_sequence_store(sequence_store.as_ref())
            .expect("test producer generation reservation"),
        explore,
        mapping_adapter,
        mapping_lifecycle_lock,
        occupancy_shm,
        occupancy_shm_path,
        occupancy_shm_current,
        explore_scheduler: Arc::new(tokio::sync::Mutex::new(None)),
        runtime_observation_generation: Arc::new(std::sync::atomic::AtomicU64::new(0)),
        task_control: None,
    })
}

#[cfg(test)]
mod tf_tree_tests {
    use super::*;

    fn entry(stamp_ns: u64, tx: f64) -> TransformEntry {
        TransformEntry {
            parent: "/map".into(),
            child: "/base_link".into(),
            tx,
            ty: 2.0,
            tz: 0.0,
            rx: 0.0,
            ry: 0.0,
            rz: 0.0,
            rw: 1.0,
            stamp_ns,
        }
    }

    #[test]
    fn canonical_tree_retains_history_and_drives_latest_inventory() {
        let state = SharedState::default();
        state
            .tf_tree
            .set(entry_to_transform(&entry(1_000_000_100, 1.0)).unwrap());
        state
            .tf_tree
            .set(entry_to_transform(&entry(2_000_000_200, 3.0)).unwrap());

        assert_eq!(state.tf_tree.history_len("base_link"), 2);
        let historical = state
            .tf_tree
            .lookup_at("base_link", 1.000_000_100, 0.0)
            .unwrap();
        assert_eq!(historical.transform.translation.x, 1.0);
        let latest = state.latest_transform_entries();
        assert_eq!(latest.len(), 1);
        assert_eq!(latest[0].parent, "map");
        assert_eq!(latest[0].child, "base_link");
        assert_eq!(latest[0].tx, 3.0);
        assert_eq!(latest[0].stamp_ns, 2_000_000_200);
    }

    #[test]
    fn canonical_tree_resolves_base_to_map_and_rejects_invalid_quaternion() {
        let state = SharedState::default();
        state
            .tf_tree
            .set(entry_to_transform(&entry(2_000_000_200, 3.0)).unwrap());
        let pose = resolve_map_pose_tree(&state.tf_tree, "/map", &["/base_link"]).unwrap();
        assert_eq!((pose.0, pose.1, pose.2), (3.0, 2.0, 0.0));

        let mut invalid = entry(3_000_000_000, 0.0);
        invalid.rw = 0.0;
        assert!(entry_to_transform(&invalid).is_none());
    }

    #[test]
    fn semantic_base_candidates_accept_aliases_and_reject_foreign_namespaces() {
        let mut unnamespaced = entry(1, 0.0);
        unnamespaced.parent = "odom".into();
        unnamespaced.child = "base_footprint".into();
        let mut same_namespace = entry(2, 0.0);
        same_namespace.parent = "odom".into();
        same_namespace.child = "robot/body".into();
        let mut foreign_namespace = entry(3, 0.0);
        foreign_namespace.parent = "odom".into();
        foreign_namespace.child = "other/base_link".into();
        let mut foreign_role = entry(4, 0.0);
        foreign_role.parent = "odom".into();
        foreign_role.child = "camera".into();

        let candidates = semantic_base_candidates(
            &[
                unnamespaced,
                same_namespace,
                foreign_namespace,
                foreign_role,
            ],
            "robot/base_link",
        );
        assert_eq!(
            candidates,
            vec!["robot/base_link", "base_footprint", "robot/body"]
        );
    }

    #[test]
    fn base_frame_contract_handles_suffixes_validation_and_env_precedence() {
        assert!(base_frame_ids_semantically_equal(
            "robot/chassis_base_link",
            "robot/base_footprint"
        ));
        assert!(!base_frame_ids_semantically_equal(
            "robot/chassis_base_link",
            "other/base_link"
        ));
        assert!(!base_frame_ids_semantically_equal(
            "/base_link",
            "base_link"
        ));
        assert_eq!(
            resolve_base_frame_alias(Some(" body "), Some("base_footprint")),
            "body"
        );
        assert_eq!(
            resolve_base_frame_alias(Some(" "), Some(" base_footprint ")),
            "base_footprint"
        );
    }

    #[test]
    fn validated_frame_bindings_cover_map_odom_and_base_roles() {
        let bindings = ValidatedFrameBindings::from_values("map", "robot/odom", "robot/base_link")
            .expect("valid frame bindings");
        assert_eq!(
            bindings.into_parts(),
            (
                "map".to_string(),
                "robot/odom".to_string(),
                "robot/base_link".to_string()
            )
        );

        for (map, odom, base, role) in [
            ("invalid frame", "odom", "base_link", "map"),
            ("map", "invalid frame", "base_link", "odom"),
            ("map", "odom", "invalid frame", "base"),
        ] {
            let error = ValidatedFrameBindings::from_values(map, odom, base)
                .expect_err("invalid frame must fail startup preflight");
            assert_eq!(error.role, role);
            assert_eq!(error.variable, "<value>");
        }
    }
}

#[cfg(test)]
mod public_tf_ownership_tests {
    use super::*;
    use nuwax_codec::lcm::schema;

    #[derive(Debug, Eq, Ord, PartialEq, PartialOrd)]
    struct TransformFingerprint {
        parent: String,
        child: String,
        translation: [u64; 3],
        rotation: [u64; 4],
        stamp_ns: u64,
    }

    #[derive(Debug, Eq, PartialEq)]
    struct FrontierFingerprint {
        position: [u64; 3],
        rotation: [u64; 4],
        frame_id: String,
        size_cells: u32,
        heading: u32,
    }

    #[derive(Debug, Eq, PartialEq)]
    struct SharedStateFingerprint {
        pose: [u64; 7],
        pose_frame_id: String,
        pose_timestamp_ns: u64,
        map_dimensions: [u32; 2],
        map_resolution: u32,
        map_origin: [u64; 2],
        map_frame_id: String,
        map_revision: u64,
        map_timestamp_ns: u64,
        map_data: Vec<i8>,
        tf_tree_latest: Vec<TransformFingerprint>,
        tf_tree_history_lengths: Vec<(String, usize)>,
        transforms: Vec<TransformFingerprint>,
        nav_text: [String; 3],
        nav_distance_remaining: u32,
        nav_timestamp_ns: u64,
        cloud_map_revision: u64,
        frontiers: Vec<FrontierFingerprint>,
        frontiers_revision: u64,
    }

    fn transform_fingerprint(entry: &TransformEntry) -> TransformFingerprint {
        TransformFingerprint {
            parent: entry.parent.clone(),
            child: entry.child.clone(),
            translation: [entry.tx.to_bits(), entry.ty.to_bits(), entry.tz.to_bits()],
            rotation: [
                entry.rx.to_bits(),
                entry.ry.to_bits(),
                entry.rz.to_bits(),
                entry.rw.to_bits(),
            ],
            stamp_ns: entry.stamp_ns,
        }
    }

    fn shared_state_fingerprint(state: &SharedState) -> SharedStateFingerprint {
        let mut tf_tree_latest = state
            .latest_transform_entries()
            .iter()
            .map(transform_fingerprint)
            .collect::<Vec<_>>();
        tf_tree_latest.sort();
        let tf_tree_history_lengths = state
            .tf_tree
            .frames()
            .into_iter()
            .map(|frame| {
                let history_len = state.tf_tree.history_len(&frame);
                (frame, history_len)
            })
            .collect();
        SharedStateFingerprint {
            pose: [
                state.pose.x.to_bits(),
                state.pose.y.to_bits(),
                state.pose.z.to_bits(),
                state.pose.qx.to_bits(),
                state.pose.qy.to_bits(),
                state.pose.qz.to_bits(),
                state.pose.qw.to_bits(),
            ],
            pose_frame_id: state.pose.frame_id.clone(),
            pose_timestamp_ns: state.pose.timestamp_ns,
            map_dimensions: [state.map.width, state.map.height],
            map_resolution: state.map.resolution.to_bits(),
            map_origin: [state.map.origin_x.to_bits(), state.map.origin_y.to_bits()],
            map_frame_id: state.map.frame_id.clone(),
            map_revision: state.map.revision,
            map_timestamp_ns: state.map.timestamp_ns,
            map_data: state.map.data.clone(),
            tf_tree_latest,
            tf_tree_history_lengths,
            transforms: state.transforms.iter().map(transform_fingerprint).collect(),
            nav_text: [
                state.nav_state.status.clone(),
                state.nav_state.message.clone(),
                state.nav_state.goal_id.clone(),
            ],
            nav_distance_remaining: state.nav_state.distance_remaining.to_bits(),
            nav_timestamp_ns: state.nav_state.timestamp_ns,
            cloud_map_revision: state.cloud_map_revision,
            frontiers: state
                .frontiers
                .iter()
                .map(|frontier| FrontierFingerprint {
                    position: [
                        frontier.x.to_bits(),
                        frontier.y.to_bits(),
                        frontier.z.to_bits(),
                    ],
                    rotation: [
                        frontier.qx.to_bits(),
                        frontier.qy.to_bits(),
                        frontier.qz.to_bits(),
                        frontier.qw.to_bits(),
                    ],
                    frame_id: frontier.frame_id.clone(),
                    size_cells: frontier.size_cells,
                    heading: frontier.heading.to_bits(),
                })
                .collect(),
            frontiers_revision: state.frontiers_revision,
        }
    }

    fn transform_entry(parent: &str, child: &str, tx: f64, stamp_ns: u64) -> TransformEntry {
        TransformEntry {
            parent: parent.to_owned(),
            child: child.to_owned(),
            tx,
            ty: tx + 0.25,
            tz: tx + 0.5,
            rx: 0.0,
            ry: 0.0,
            rz: 0.0,
            rw: 1.0,
            stamp_ns,
        }
    }

    #[tokio::test]
    async fn hostile_public_tf_is_observed_without_mutating_shared_state() {
        let handle = build_handle_for_tests()
            .await
            .expect("real-LCM test handle");
        let canonical_entries = vec![
            transform_entry("map", "odom", 1.0, 7_000_000_001),
            transform_entry("odom", "base_link", 2.0, 7_000_000_001),
            transform_entry("base_link", "owned_sensor", 3.0, 0),
        ];
        {
            let mut state = handle.state.write().await;
            state.pose = PoseState {
                x: 4.0,
                y: 5.0,
                z: 6.0,
                qx: 0.1,
                qy: 0.2,
                qz: 0.3,
                qw: 0.9,
                frame_id: "map".to_owned(),
                timestamp_ns: 7_000_000_001,
            };
            state.map = MapState {
                width: 3,
                height: 1,
                resolution: 0.125,
                origin_x: -2.0,
                origin_y: -3.0,
                frame_id: "map".to_owned(),
                revision: 41,
                timestamp_ns: 7_000_000_002,
                data: vec![-1, 0, 100],
            };
            for entry in &canonical_entries {
                state
                    .tf_tree
                    .set(entry_to_transform(entry).expect("seed canonical TF"));
            }
            state.transforms = canonical_entries;
            state.nav_state = NavState {
                status: "RUNNING".to_owned(),
                message: "canonical state sentinel".to_owned(),
                goal_id: "goal-owned".to_owned(),
                distance_remaining: 8.5,
                timestamp_ns: 7_000_000_003,
            };
            state.cloud_map_revision = 42;
            state.frontiers = vec![FrontierEntry {
                x: 1.0,
                y: 2.0,
                z: 3.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
                frame_id: "map".to_owned(),
                size_cells: 9,
                heading: 0.75,
            }];
            state.frontiers_revision = 43;
        }
        let before = {
            let state = handle.state.read().await;
            shared_state_fingerprint(&state)
        };

        let topic = LcmTopic::new(&format!(
            "/test/public_tf_fence/{}/{}",
            std::process::id(),
            before.pose_timestamp_ns
        ))
        .expect("public TF topic");
        let message_type = LcmMessageType::new("tf2_msgs.TFMessage").expect("TF message type");
        let stream = handle
            .transport
            .subscribe(topic.clone(), Some(message_type.clone()))
            .await;
        let observer = tokio::spawn(observe_public_tf_stream(
            stream,
            "map".to_owned(),
            "odom".to_owned(),
            "base_link".to_owned(),
            handle.nav.clone(),
        ));

        let header = |frame_id: &str, stamp_sec: i32, seq: i32| schema::Header {
            seq,
            stamp_sec,
            stamp_nsec: 7,
            frame_id: frame_id.to_owned(),
        };
        let map_payload = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: header("map", 99, 99),
                child_frame_id: "odom".to_owned(),
                transform: schema::Transform {
                    translation: [800.0, 801.0, 802.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode split map-to-odom public TF");
        handle
            .transport
            .publish(crate::typed_codec::build_typed_frame(
                topic.clone(),
                message_type.clone(),
                map_payload,
                99_000_000_007,
            ))
            .await
            .expect("publish split map-to-odom public TF");
        let first = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let detail = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
                if detail.tf_count >= 1 {
                    break detail;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("first split public TF observation");
        assert!(!first.tf, "one split edge must not establish readiness");

        let base_payload = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![
                schema::TransformStamped {
                    header: header("odom", 100, 100),
                    child_frame_id: "base_link".to_owned(),
                    transform: schema::Transform {
                        translation: [900.0, 901.0, 902.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                },
                schema::TransformStamped {
                    header: header("base_link", 100, 100),
                    child_frame_id: "hostile_camera".to_owned(),
                    transform: schema::Transform {
                        translation: [1_000.0, 1_001.0, 1_002.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                },
            ],
        })
        .expect("encode split odom-to-base public TF");
        handle
            .transport
            .publish(crate::typed_codec::build_typed_frame(
                topic.clone(),
                message_type.clone(),
                base_payload,
                100_000_000_007,
            ))
            .await
            .expect("publish split odom-to-base public TF");

        let observation = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let detail = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
                if detail.tf_count >= 2 {
                    break detail;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("public TF observer consumption");
        assert_eq!(observation.tf_count, 2);
        assert!(
            observation.tf,
            "split messages within history TTL must establish the chain"
        );

        let expiry_payload = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: header("hostile_world", 111, 111),
                child_frame_id: "hostile_clock".to_owned(),
                transform: schema::Transform {
                    translation: [1_100.0, 1_101.0, 1_102.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode TF history expiry frame");
        handle
            .transport
            .publish(crate::typed_codec::build_typed_frame(
                topic,
                message_type,
                expiry_payload,
                111_000_000_007,
            ))
            .await
            .expect("publish TF history expiry frame");
        let expired = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let detail = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
                if detail.tf_count >= 3 {
                    break detail;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("expired public TF observation");
        assert!(!expired.tf, "expired split edges must not remain ready");

        let after = {
            let state = handle.state.read().await;
            shared_state_fingerprint(&state)
        };
        assert_eq!(after, before, "public TF mutated canonical SharedState");
        observer.abort();
        let _ = observer.await;
    }
}

#[cfg(test)]
mod mapping_feed_tests {
    use super::*;
    use crate::mapping_adapter::MappingReadiness;
    use crate::mapping_backend::MappingBackendTestGate;
    use crate::occupancy_shm::OccupancyGridShmReader;
    use nalgebra::Point3;
    use nuwax_protocol::RawLcmFrame;
    use nuwax_transport::{LcmBackend, LcmFrameStream, TransportResult};

    #[derive(Default)]
    struct RecordingCorrectionFrameSink {
        staged: tokio::sync::Mutex<Vec<crate::correction_coordinator::CorrectionFrameInput>>,
    }

    #[async_trait]
    impl crate::correction_coordinator::CorrectionCandidateSink for RecordingCorrectionFrameSink {
        async fn stage_frame(
            &self,
            input: &crate::correction_coordinator::CorrectionFrameInput,
        ) -> Result<(), String> {
            self.staged.lock().await.push(input.clone());
            Ok(())
        }

        async fn apply_candidate(
            &self,
            _candidate: crate::correction_coordinator::CorrectionCandidate,
            _cancellation: crate::correction_coordinator::CorrectionRequestCancellation,
        ) -> Result<crate::correction_coordinator::CorrectionDisposition, String> {
            Err("none correction owner cannot produce a candidate".to_owned())
        }
    }

    #[derive(Clone)]
    struct MappingFeedRecordingBackend {
        odom_topic: LcmTopic,
        odom_sender: tokio::sync::mpsc::UnboundedSender<TransportResult<RawLcmFrame>>,
        odom_receiver: Arc<
            std::sync::Mutex<
                Option<tokio::sync::mpsc::UnboundedReceiver<TransportResult<RawLcmFrame>>>,
            >,
        >,
        odom_subscribed: Arc<tokio::sync::Notify>,
        frame_published: Arc<tokio::sync::Notify>,
        publish_started: Arc<tokio::sync::Notify>,
        block_publish: Arc<std::sync::atomic::AtomicBool>,
        published: Arc<std::sync::Mutex<Vec<RawLcmFrame>>>,
    }

    impl MappingFeedRecordingBackend {
        fn new(odom_topic: LcmTopic) -> Self {
            let (odom_sender, odom_receiver) = tokio::sync::mpsc::unbounded_channel();
            Self {
                odom_topic,
                odom_sender,
                odom_receiver: Arc::new(std::sync::Mutex::new(Some(odom_receiver))),
                odom_subscribed: Arc::new(tokio::sync::Notify::new()),
                frame_published: Arc::new(tokio::sync::Notify::new()),
                publish_started: Arc::new(tokio::sync::Notify::new()),
                block_publish: Arc::new(std::sync::atomic::AtomicBool::new(false)),
                published: Arc::new(std::sync::Mutex::new(Vec::new())),
            }
        }

        fn published_count(&self) -> usize {
            self.published.lock().expect("published frames").len()
        }

        fn block_publication(&self) {
            self.block_publish
                .store(true, std::sync::atomic::Ordering::Release);
        }
    }

    #[async_trait]
    impl LcmBackend for MappingFeedRecordingBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            self.publish_started.notify_one();
            if self
                .block_publish
                .load(std::sync::atomic::Ordering::Acquire)
            {
                std::future::pending::<()>().await;
            }
            self.published.lock().expect("published frames").push(frame);
            self.frame_published.notify_one();
            Ok(())
        }

        async fn subscribe(
            &self,
            topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            if topic != self.odom_topic {
                return Box::pin(futures::stream::empty());
            }
            let receiver = self
                .odom_receiver
                .lock()
                .expect("odom receiver")
                .take()
                .expect("single odom subscription");
            self.odom_subscribed.notify_one();
            Box::pin(tokio_stream::wrappers::UnboundedReceiverStream::new(
                receiver,
            ))
        }

        fn name(&self) -> &str {
            "mapping-feed-recording"
        }
    }

    #[derive(Clone, Default)]
    struct FirstGlobalMapPublishGateBackend {
        publish_attempts: Arc<std::sync::atomic::AtomicU64>,
        first_publish_started: Arc<tokio::sync::Notify>,
        frame_published: Arc<tokio::sync::Notify>,
        published: Arc<std::sync::Mutex<Vec<RawLcmFrame>>>,
    }

    #[async_trait]
    impl LcmBackend for FirstGlobalMapPublishGateBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            let attempt = self
                .publish_attempts
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            if attempt == 0 {
                self.first_publish_started.notify_one();
                std::future::pending::<()>().await;
            }
            self.published
                .lock()
                .expect("global-map published frames")
                .push(frame);
            self.frame_published.notify_one();
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "first-global-map-publish-gate"
        }
    }

    #[derive(Clone)]
    struct SameSessionGlobalMapPublishGateBackend {
        publish_attempts: Arc<std::sync::atomic::AtomicU64>,
        first_publish_started: Arc<tokio::sync::Notify>,
        first_release_tx: tokio::sync::watch::Sender<bool>,
        frame_published: Arc<tokio::sync::Notify>,
        published: Arc<std::sync::Mutex<Vec<RawLcmFrame>>>,
    }

    impl Default for SameSessionGlobalMapPublishGateBackend {
        fn default() -> Self {
            let (first_release_tx, _) = tokio::sync::watch::channel(false);
            Self {
                publish_attempts: Arc::new(std::sync::atomic::AtomicU64::new(0)),
                first_publish_started: Arc::new(tokio::sync::Notify::new()),
                first_release_tx,
                frame_published: Arc::new(tokio::sync::Notify::new()),
                published: Arc::new(std::sync::Mutex::new(Vec::new())),
            }
        }
    }

    impl SameSessionGlobalMapPublishGateBackend {
        fn release_first_publish(&self) {
            self.first_release_tx.send_replace(true);
        }
    }

    #[async_trait]
    impl LcmBackend for SameSessionGlobalMapPublishGateBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            let attempt = self
                .publish_attempts
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            if attempt == 0 {
                self.first_publish_started.notify_one();
                let mut first_release_rx = self.first_release_tx.subscribe();
                while !*first_release_rx.borrow_and_update() {
                    if first_release_rx.changed().await.is_err() {
                        return Ok(());
                    }
                }
            }
            self.published
                .lock()
                .expect("same-session global-map published frames")
                .push(frame);
            self.frame_published.notify_one();
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "same-session-global-map-publish-gate"
        }
    }

    fn mapping_config() -> MappingAdapterConfig {
        MappingAdapterConfig {
            resolution: 0.5,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-5.0, -5.0),
            voxel_size: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::General,
            initial_safe_radius_meters: 0.0,
        }
    }

    fn active_mapping_adapter() -> MappingAdapter {
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), "map", "odom");
        adapter
            .start_session("test-session", "test-map")
            .expect("active mapping test session");
        adapter
    }

    fn shared_active_mapping_adapter() -> Arc<parking_lot::Mutex<MappingAdapter>> {
        Arc::new(parking_lot::Mutex::new(active_mapping_adapter()))
    }

    struct NavigationMap3DFixture {
        context: NavigationMap3DPublishContext,
        state: Arc<RwLock<SharedState>>,
        writer: Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
        cloud_tx: broadcast::Sender<CloudMapNotification>,
        map_tx: broadcast::Sender<MapRevisionEvent>,
        path: std::path::PathBuf,
    }

    impl NavigationMap3DFixture {
        fn new() -> Self {
            Self::with_mapping(shared_active_mapping_adapter())
        }

        fn with_mapping(mapping: Arc<parking_lot::Mutex<MappingAdapter>>) -> Self {
            let generation = mapping.lock().snapshot().session_generation;
            let path = std::env::temp_dir().join(format!(
                "nuwax-navigation-map-3d-{}-{}.mmap",
                std::process::id(),
                uuid::Uuid::new_v4(),
            ));
            let mut writer = CloudMapShmWriter::new(&path).expect("三维发布测试共享内存");
            writer.begin_session(generation).expect("会话清空");
            Self {
                context: NavigationMap3DPublishContext {
                    enabled: true,
                    producer_runtime_generation: 37,
                    mapping,
                    committed_map_authority: Arc::new(std::sync::Mutex::new(None)),
                },
                state: Arc::new(RwLock::new(SharedState::default())),
                writer: Arc::new(parking_lot::Mutex::new(Some(writer))),
                cloud_tx: broadcast::channel(4).0,
                map_tx: broadcast::channel(4).0,
                path,
            }
        }

        fn commit(&self, timestamp_ns: u64) -> CommittedMappingCloudMap {
            self.commit_points(timestamp_ns, &[Point3::new(0.5, 0.0, 0.0), Point3::new(1.0, 1.0, 0.5)])
        }

        fn commit_points(&self, timestamp_ns: u64, points: &[Point3<f64>]) -> CommittedMappingCloudMap {
            let mut mapping = self.context.mapping.lock();
            let outcome = ingest_mapping_points(
                &mut mapping,
                points,
                12,
                timestamp_ns,
                "map",
                timestamp_ns,
                &self.state,
                &self.map_tx,
            )
            .expect("真实规范建图提交");
            let event = outcome.occupancy_event.as_ref().expect("真实占据修订");
            *self.context.committed_map_authority.lock().unwrap() =
                Some(committed_map_identity_from_revision(event).expect("规范占据身份"));
            update_mapping_cloud_map_if_current(&outcome, &mapping.snapshot(), &self.writer)
                .expect("真实关联点云提交")
        }
    }

    impl Drop for NavigationMap3DFixture {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.path);
        }
    }

    #[tokio::test]
    async fn navigation_map_3d_virtual_obstacles_keep_commit_identity_and_block_both_floors() {
        let root = std::env::temp_dir().join(format!("nuwax-nav3d-virtual-mask-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&root).unwrap();
        struct Cleanup(std::path::PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) { let _ = std::fs::remove_dir_all(&self.0); }
        }
        let _cleanup = Cleanup(root.clone());
        let zone_path = root.join("禁区.json");
        std::fs::write(&zone_path, br#"{"schema_version":1,"maps":{"test-map":{"map_name":"test-map","generation":1,"zones":[{"id":"line","shape":"line","mode":"inside","frame_id":"map","vertices":[{"x":0.0,"y":-5.0},{"x":0.0,"y":5.0}],"stroke_width_m":0.1}]}}}"#).unwrap();
        let store = Arc::new(crate::sequence_state::SequenceStateStore::open(&root.join("sequence.json")).unwrap());
        let mut mapping = MappingAdapter::new_with_frames_sequence_store_and_forbidden_zones(
            mapping_config(), crate::mapping_backend::MappingBackendSpec::Voxel, "map", "odom", store, zone_path.clone(),
        );
        mapping.start_session("test-session", "test-map").unwrap();
        let fixture = NavigationMap3DFixture::with_mapping(Arc::new(parking_lot::Mutex::new(mapping)));
        let mut points = Vec::new();
        for z in [0.0, 3.0] {
            for x in -30..30 {
                for y in -30..30 {
                    points.push(Point3::new((f64::from(x) + 0.5) * 0.1, (f64::from(y) + 0.5) * 0.1, z));
                }
            }
        }
        fixture.commit_points(1_700_000_000_000_000_000, &points);
        let (_, _, publication) = latest_canonical_cloud_publication(&fixture.writer).unwrap().unwrap();
        let metadata = navigation_map_3d_publication_metadata(&fixture.context, &publication, &fixture.state, "map").await.unwrap();
        let frame = build_navigation_map_3d_publication_frame(&publication, &metadata).unwrap();
        let message = schema::decode_navigation_map_3d_body(&frame.payload).unwrap();
        let mask = message.virtual_obstacles.as_ref().unwrap();
        assert_eq!(i64::from(mask.header.seq), message.committed_map.revision);
        assert_eq!((mask.header.stamp_sec, mask.header.stamp_nsec), (message.cloud.header.stamp_sec, message.cloud.header.stamp_nsec));
        assert!(mask.data.contains(&0) && mask.data.contains(&100));
        assert!(mask.data.iter().all(|value| matches!(value, 0 | 100)));
        let geometry = nuwax_nav_stack::surface_config::SurfaceNavigationConfig::new(
            "map".into(), 0.5, 0.2, 0.6, 0.1, 0.15, 18.0, 0.2,
        ).unwrap().geometry;
        let mut unmasked = message.clone();
        unmasked.virtual_obstacles = None;
        let free = build_exploration_surface_map(&unmasked, geometry.clone()).unwrap();
        let protected = build_exploration_surface_map(&message, geometry.clone()).unwrap();
        for z in [0.0, 3.0] {
            let start = free.locate_support([-2.0, 0.0, z]).unwrap().support_xyz;
            let goal = free.locate_support([2.0, 0.0, z]).unwrap().support_xyz;
            assert!(free.plan(start, goal).is_ok(), "物理三维图原本连通");
            assert!(protected.plan(start, goal).is_err(), "纯虚拟细线必须阻断两层路径");
        }
        std::fs::write(&zone_path, b"{broken-json").unwrap();
        let unchanged = navigation_map_3d_publication_metadata(&fixture.context, &publication, &fixture.state, "map").await.unwrap();
        assert_eq!(unchanged, metadata, "未应用文件不得泄漏进已提交三维消息");
        {
            let mut mapping = fixture.context.mapping.lock();
            assert!(mapping.refresh_forbidden_zones(&fixture.state, &fixture.map_tx).unwrap());
        }
        assert!(navigation_map_3d_publication_metadata(&fixture.context, &publication, &fixture.state, "map").await.is_err(),
            "只更新二维禁区后不得伪造一条新传感器点云提交");
        fixture.commit_points(1_700_000_001_000_000_000, &points);
        let (_, _, publication) = latest_canonical_cloud_publication(&fixture.writer).unwrap().unwrap();
        let metadata = navigation_map_3d_publication_metadata(&fixture.context, &publication, &fixture.state, "map").await.unwrap();
        let frame = build_navigation_map_3d_publication_frame(&publication, &metadata).unwrap();
        let blocked = schema::decode_navigation_map_3d_body(&frame.payload).unwrap();
        assert!(blocked.virtual_obstacles.as_ref().unwrap().data.iter().all(|value| *value == 100));
        let closed = build_exploration_surface_map(&blocked, geometry).unwrap();
        assert!(closed.locate_support([-2.0, 0.0, 0.0]).is_err());
        assert!(closed.locate_support([-2.0, 0.0, 3.0]).is_err());
    }

    #[tokio::test]
    async fn navigation_map_3d_publication_preserves_real_receipts_and_clear_order() {
        let fixture = NavigationMap3DFixture::new();
        let committed = fixture.commit(1_700_000_000_000_000_000);
        let backend = MappingFeedRecordingBackend::new(LcmTopic::new("/unused/nav3d").unwrap());
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        publish_latest_canonical_cloud_map_with_navigation(
            &transport,
            &fixture.writer,
            &fixture.state,
            &fixture.cloud_tx,
            "map",
            Some(&fixture.context),
        )
        .await
        .expect("三维关联发布");
        publish_latest_canonical_cloud_map_with_navigation(
            &transport,
            &fixture.writer,
            &fixture.state,
            &fixture.cloud_tx,
            "map",
            Some(&fixture.context),
        )
        .await
        .expect("重复发布不重复发送");
        let frames = backend.published.lock().unwrap().clone();
        assert_eq!(
            frames
                .iter()
                .map(|frame| frame.topic.channel())
                .collect::<Vec<_>>(),
            vec![
                GLOBAL_MAP_CHANNEL,
                "/nav/map3d",
                GLOBAL_MAP_CHANNEL,
                "/nav/map3d"
            ]
        );
        let clear = schema::decode_navigation_map_3d_body(&frames[1].payload).unwrap();
        let live = schema::decode_navigation_map_3d_body(&frames[3].payload).unwrap();
        assert!(clear.cleared);
        assert!(!live.cleared);
        assert!(clear.cloud_revision < live.cloud_revision);
        assert_eq!(clear.commit_sequence, 0);
        assert_eq!(
            clear.commit_boot_nonce,
            committed.receipt.commit_id.boot_nonce.to_be_bytes()
        );
        assert_eq!(live.commit_boot_nonce, clear.commit_boot_nonce);
        assert_eq!(
            live.commit_sequence,
            committed.receipt.commit_id.sequence as i64
        );
        assert_eq!(live.cloud_revision, committed.receipt.cloud_revision as i64);
        assert_eq!(
            live.source_timestamp_ns,
            committed.receipt.source_timestamp_ns as i64
        );
        assert_eq!(live.source_voxel_size, committed.notification.voxel_size);
        assert_eq!(
            live.map_frame_generation,
            fixture.context.mapping.lock().global_frame_generation() as i64
        );
        assert_eq!(live.producer_runtime_generation, 37);
        assert_eq!(
            clear.producer_runtime_generation,
            live.producer_runtime_generation
        );
        assert_eq!(
            Some(live.committed_map),
            *fixture.context.committed_map_authority.lock().unwrap()
        );
        assert_eq!(live.cloud.data, committed.xyz_f32_le);
        assert_eq!(
            frames[3].msg_type.fingerprint(),
            schema::NAVIGATION_MAP_3D_FINGERPRINT
        );
    }

    #[tokio::test]
    async fn navigation_map_3d_rejects_wrong_authority_and_preserves_two_dimensional_output() {
        for enabled in [true, false] {
            let mut fixture = NavigationMap3DFixture::new();
            fixture.context.enabled = enabled;
            let backend = MappingFeedRecordingBackend::new(LcmTopic::new("/unused/nav3d").unwrap());
            let transport = LcmTransport::new(Arc::new(backend.clone()));
            publish_latest_canonical_cloud_map_with_navigation(
                &transport,
                &fixture.writer,
                &fixture.state,
                &fixture.cloud_tx,
                "map",
                Some(&fixture.context),
            )
            .await
            .expect("首先发布会话清空");
            fixture.commit(1_700_000_000_000_000_000);
            fixture
                .context
                .committed_map_authority
                .lock()
                .unwrap()
                .as_mut()
                .unwrap()
                .payload_sha256[0] ^= 1;
            backend.published.lock().unwrap().clear();
            let result = publish_latest_canonical_cloud_map_with_navigation(
                &transport,
                &fixture.writer,
                &fixture.state,
                &fixture.cloud_tx,
                "map",
                Some(&fixture.context),
            )
            .await;
            let frames = backend.published.lock().unwrap().clone();
            if enabled {
                assert!(result.unwrap_err().contains("hash conflicts"));
                assert!(frames.is_empty(), "错误权威身份不得发布生效三维图");
            } else {
                assert!(result.is_ok());
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].topic.channel(), GLOBAL_MAP_CHANNEL);
            }
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn navigation_map_3d_delayed_old_session_cannot_replace_new_clear_and_live() {
        let fixture = NavigationMap3DFixture::new();
        let initial_backend =
            MappingFeedRecordingBackend::new(LcmTopic::new("/unused/nav3d").unwrap());
        let initial_transport = LcmTransport::new(Arc::new(initial_backend));
        publish_latest_canonical_cloud_map_with_navigation(
            &initial_transport,
            &fixture.writer,
            &fixture.state,
            &fixture.cloud_tx,
            "map",
            Some(&fixture.context),
        )
        .await
        .unwrap();
        let old = fixture.commit(1_700_000_000_000_000_000);
        let backend = FirstGlobalMapPublishGateBackend::default();
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let delayed = {
            let transport = transport.clone();
            let writer = fixture.writer.clone();
            let state = fixture.state.clone();
            let cloud_tx = fixture.cloud_tx.clone();
            let context = fixture.context.clone();
            tokio::spawn(async move {
                publish_latest_canonical_cloud_map_with_navigation(
                    &transport,
                    &writer,
                    &state,
                    &cloud_tx,
                    "map",
                    Some(&context),
                )
                .await
            })
        };
        tokio::time::timeout(
            Duration::from_secs(1),
            backend.first_publish_started.notified(),
        )
        .await
        .unwrap();
        let new_generation = {
            let mut mapping = fixture.context.mapping.lock();
            mapping.stop_session("test-session").unwrap();
            mapping.start_session("new-session", "new-map").unwrap();
            mapping.snapshot().session_generation
        };
        fixture
            .writer
            .lock()
            .as_mut()
            .unwrap()
            .begin_session(new_generation)
            .unwrap();
        let new = fixture.commit(1_700_000_003_000_000_000);
        publish_latest_canonical_cloud_map_with_navigation(
            &transport,
            &fixture.writer,
            &fixture.state,
            &fixture.cloud_tx,
            "map",
            Some(&fixture.context),
        )
        .await
        .unwrap();
        tokio::time::timeout(Duration::from_secs(1), delayed)
            .await
            .unwrap()
            .unwrap()
            .unwrap();
        let messages = backend
            .published
            .lock()
            .unwrap()
            .iter()
            .filter(|frame| frame.topic.channel() == "/nav/map3d")
            .map(|frame| schema::decode_navigation_map_3d_body(&frame.payload).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(messages.len(), 2);
        assert!(messages[0].cleared);
        assert!(!messages[1].cleared);
        assert!(messages
            .iter()
            .all(|message| message.committed_map.session_id == "new-session"));
        assert!(messages
            .iter()
            .all(|message| message.cloud_revision > old.receipt.cloud_revision as i64));
        assert_eq!(
            messages[1].cloud_revision,
            new.receipt.cloud_revision as i64
        );
    }

    #[tokio::test]
    async fn surface_pose_provider_preserves_real_height_and_complete_orientation() {
        let state = Arc::new(RwLock::new(SharedState::default()));
        state.write().await.pose = PoseState {
            x: 1.0, y: 2.0, z: 3.5, qx: 0.1, qy: -0.2, qz: 0.3, qw: 0.86_f64.sqrt(),
            frame_id: "map".into(), timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            ..PoseState::default()
        };
        let provider = SharedStatePoseProvider {
            state, expected_map_frame: Arc::from("map"),
            freshness: Arc::new(parking_lot::Mutex::new(PoseFreshnessTracker::default())),
            max_receipt_age: Duration::from_secs(1),
        };
        let pose = crate::explore_runtime::ExplorePoseProvider::current_pose(&provider).await.unwrap();
        assert_eq!((pose.x, pose.y, pose.z), (1.0, 2.0, 3.5));
        assert_eq!(pose.orientation, nuwax_nav_types_messages::geometry::Quaternion {
            x: 0.1, y: -0.2, z: 0.3, w: 0.86_f64.sqrt(),
        });
        assert!(pose.source_timestamp_ns > 0 && pose.received_monotonic_ns > 0);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn surface_build_is_retained_until_actual_blocking_work_finishes() {
        let fixture = NavigationMap3DFixture::new();
        fixture.commit(1_700_000_000_000_000_000);
        let (_, _, publication) = latest_canonical_cloud_publication(&fixture.writer).unwrap().unwrap();
        let metadata = navigation_map_3d_publication_metadata(&fixture.context, &publication, &fixture.state, "map").await.unwrap();
        let frame = build_navigation_map_3d_publication_frame(&publication, &metadata).unwrap();
        let message = schema::decode_navigation_map_3d_body(&frame.payload).unwrap();
        let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
        let config = nuwax_nav_stack::surface_config::SurfaceNavigationConfig::new(
            "map".into(), 0.5, 0.2, 0.6, 0.1, 0.15, 18.0, 0.2,
        ).unwrap();
        explore.configure_surface_source(Ok(config), 37, message.commit_boot_nonce).await.unwrap();
        let ticket = explore.begin_surface_map_update(&message, crate::explore_runtime::now_monotonic_nanos())
            .await.unwrap().unwrap();
        let (control, registrations) = GeometryRuntimeTaskControl::channel();
        let mut owner = GeometryRuntimeTaskOwner::start(control.clone(), registrations);
        let (completed_tx, mut completed_rx) = tokio::sync::mpsc::channel(1);
        struct Gate(Arc<(std::sync::Mutex<bool>, std::sync::Condvar)>);
        impl Drop for Gate {
            fn drop(&mut self) { *self.0.0.lock().unwrap() = true; self.0.1.notify_all(); }
        }
        let gate = Gate(Arc::new((std::sync::Mutex::new(false), std::sync::Condvar::new())));
        let worker_gate = gate.0.clone();
        let entered = Arc::new(tokio::sync::Notify::new());
        let worker_entered = entered.clone();
        register_surface_map_build(&control, ticket.clone(), completed_tx.clone(), move || {
            worker_entered.notify_one();
            let mut released = worker_gate.0.lock().unwrap();
            while !*released { released = worker_gate.1.wait(released).unwrap(); }
            Err("受控计算已真实结束".into())
        }).unwrap();
        tokio::time::timeout(Duration::from_secs(1), entered.notified()).await.unwrap();
        control.request_stop();
        let mut stopping = Box::pin(owner.shutdown());
        assert!(tokio::time::timeout(Duration::from_millis(25), stopping.as_mut()).await.is_err(),
            "阻塞计算尚未结束时不能报告拥有方关闭完成");
        let attempted = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let worker_attempted = attempted.clone();
        assert!(register_surface_map_build(&control, ticket, completed_tx, move || {
            worker_attempted.store(true, std::sync::atomic::Ordering::Release);
            Err("不应启动".into())
        }).is_err());
        assert!(!attempted.load(std::sync::atomic::Ordering::Acquire));
        drop(gate);
        let outcome = tokio::time::timeout(Duration::from_secs(1), stopping).await.unwrap();
        assert!(outcome.producers_settled());
        assert!(outcome.result.is_ok());
        assert!(completed_rx.recv().await.unwrap().1.is_err());
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn backend_compute_holds_neither_lifecycle_nor_adapter_lock() {
        let mapping = shared_active_mapping_adapter();
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let gate = MappingBackendTestGate::default();
        mapping
            .lock()
            .replace_worker_with_test_gate(gate.clone())
            .expect("install gated worker");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let cloud_map_writer = Arc::new(parking_lot::Mutex::new(None));
        let task = {
            let mapping = mapping.clone();
            let lifecycle_lock = lifecycle_lock.clone();
            let state = state.clone();
            let cloud_map_writer = cloud_map_writer.clone();
            tokio::spawn(async move {
                ingest_mapping_points_with_backend(
                    &mapping,
                    &lifecycle_lock,
                    &Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
                    &Arc::new(std::sync::Mutex::new(None)),
                    &watch::channel(None).0,
                    &cloud_map_writer,
                    &[Point3::new(1.0, 0.0, 1.0)],
                    12,
                    1_000_000_000,
                    "map",
                    1_000_000_000,
                    &state,
                    false,
                    None,
                    None,
                    None,
                    None,
                    None,
                )
                .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            while !gate.entered() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("worker entered compute");
        assert!(
            lifecycle_lock.try_lock().is_ok(),
            "lifecycle lock must be free during backend compute"
        );
        assert!(
            mapping.try_lock().is_some(),
            "adapter lock must be free during backend compute"
        );
        gate.release();
        let outcome = task.await.expect("ingest task").expect("backend ingest");
        assert_eq!(outcome.outcome.accepted_map_points.len(), 1);
        mapping
            .lock()
            .stop_session("test-session")
            .expect("join test worker");
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn relocalization_activation_aborts_prepared_mapping_completion_without_map_write() {
        let mapping = shared_active_mapping_adapter();
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let gate = MappingBackendTestGate::default();
        mapping
            .lock()
            .replace_worker_with_test_gate(gate.clone())
            .expect("install gated worker");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let cloud_map_writer = Arc::new(parking_lot::Mutex::new(None));
        let mode = Arc::new(crate::correction_coordinator::RelocalizationModeFence::default());
        let expected_epoch = mode.snapshot().epoch;
        let revision_before = mapping.lock().snapshot().revision;
        let state_before = {
            let shared = state.read().await;
            (
                shared.map.revision,
                shared.cloud_map_revision,
                shared.map.data.clone(),
            )
        };
        let task = {
            let mapping = mapping.clone();
            let lifecycle_lock = lifecycle_lock.clone();
            let state = state.clone();
            let cloud_map_writer = cloud_map_writer.clone();
            let mode = mode.clone();
            tokio::spawn(async move {
                ingest_mapping_points_with_backend(
                    &mapping,
                    &lifecycle_lock,
                    &Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
                    &Arc::new(std::sync::Mutex::new(None)),
                    &watch::channel(None).0,
                    &cloud_map_writer,
                    &[Point3::new(1.0, 0.0, 1.0)],
                    12,
                    1_000_000_000,
                    "map",
                    1_000_000_000,
                    &state,
                    false,
                    None,
                    Some(expected_epoch),
                    Some(mode),
                    None,
                    None,
                )
                .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), async {
            while !gate.entered() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("mapping backend entered compute");
        let active_mode = mode.activate_for_tests();
        assert!(active_mode.active);
        assert_ne!(active_mode.epoch, expected_epoch);
        gate.release();
        let outcome = task
            .await
            .expect("mapping task joins")
            .expect("fenced mapping completion resolves");
        assert!(outcome.outcome.occupancy_event.is_none());
        assert!(outcome.cloud_commit.is_none());
        assert_eq!(mapping.lock().snapshot().revision, revision_before);
        let state_after = state.read().await;
        assert_eq!(
            (
                state_after.map.revision,
                state_after.cloud_map_revision,
                state_after.map.data.clone(),
            ),
            state_before
        );
        assert!(
            cloud_map_writer.lock().as_ref().is_none(),
            "fenced completion must not create or write cloud mmap"
        );
        mapping
            .lock()
            .stop_session("test-session")
            .expect("join test worker");
    }

    fn pointcloud_binding() -> (LcmTopic, LcmMessageType) {
        (
            LcmTopic::new("/test/mapping/pointcloud").expect("pointcloud topic"),
            LcmMessageType::new("sensor_msgs.PointCloud2").expect("pointcloud type"),
        )
    }

    #[test]
    fn mapping_feed_waits_only_when_inactive_state_is_observable() {
        let mapping = Arc::new(parking_lot::Mutex::new(MappingAdapter::new_with_frames(
            mapping_config(),
            "map",
            "odom",
        )));
        assert!(mapping_feed_should_wait_for_active_session(
            mapping.as_ref()
        ));

        mapping
            .lock()
            .start_session("test-session", "test-map")
            .expect("active mapping test session");
        assert!(!mapping_feed_should_wait_for_active_session(
            mapping.as_ref()
        ));

        let guard = mapping.lock();
        assert!(guard.is_active());
        assert!(
            !mapping_feed_should_wait_for_active_session(mapping.as_ref()),
            "lock contention must keep the existing cached-transform path available"
        );
    }

    #[test]
    fn canonical_cloud_publication_identity_includes_full_receipt_and_payload() {
        let receipt = crate::mapping_commit::CloudCommitReceipt {
            commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 7,
                sequence: 11,
            },
            session_generation: 3,
            occupancy_revision: 13,
            cloud_revision: 17,
            source_timestamp_ns: 19,
            point_count: 1,
        };
        let publication = CanonicalCloudPublication {
            notification: CloudMapNotification {
                revision: receipt.cloud_revision,
                point_count: receipt.point_count,
                voxel_size: crate::pointcloud::VOXEL_SIZE,
                session_generation: receipt.session_generation,
            },
            source_timestamp_ns: receipt.source_timestamp_ns,
            xyz_f32_le: vec![0; 12],
            receipt: Some(receipt),
        };
        assert!(same_canonical_cloud_publication_identity(
            &publication,
            &publication
        ));

        let mut wrong_commit = publication.clone();
        wrong_commit
            .receipt
            .as_mut()
            .expect("receipt")
            .commit_id
            .sequence += 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_commit
        ));

        let mut wrong_session = publication.clone();
        wrong_session
            .receipt
            .as_mut()
            .expect("receipt")
            .session_generation += 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_session
        ));

        let mut wrong_occupancy_revision = publication.clone();
        wrong_occupancy_revision
            .receipt
            .as_mut()
            .expect("receipt")
            .occupancy_revision += 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_occupancy_revision
        ));

        let mut wrong_cloud_revision = publication.clone();
        wrong_cloud_revision
            .receipt
            .as_mut()
            .expect("receipt")
            .cloud_revision += 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_cloud_revision
        ));

        let mut wrong_timestamp = publication.clone();
        wrong_timestamp
            .receipt
            .as_mut()
            .expect("receipt")
            .source_timestamp_ns += 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_timestamp
        ));

        let mut wrong_point_count = publication.clone();
        wrong_point_count
            .receipt
            .as_mut()
            .expect("receipt")
            .point_count += 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_point_count
        ));

        let mut wrong_payload = publication.clone();
        wrong_payload.xyz_f32_le[0] = 1;
        assert!(!same_canonical_cloud_publication_identity(
            &publication,
            &wrong_payload
        ));
    }

    fn map_revision_event(map: MapState) -> MapRevisionEvent {
        let adapter = active_mapping_adapter();
        MapRevisionEvent::from_map(&map, &adapter.snapshot())
    }

    fn occupancy_test_path(label: &str) -> std::path::PathBuf {
        static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        std::env::temp_dir().join(format!(
            "nuwax-{label}-occupancy-{}-{}.mmap",
            std::process::id(),
            COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        ))
    }

    #[test]
    fn production_ingest_uses_mapping_session_and_broadcasts_revision() {
        let mut adapter = active_mapping_adapter();
        adapter
            .feed_pose_in_frame(1.0, 2.0, 0.0, std::f64::consts::FRAC_PI_2, "odom")
            .expect("establish odom anchor");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (map_revision_tx, mut map_revision_rx) = broadcast::channel(4);
        let points = vec![Point3::new(1.0, 2.0, 0.0), Point3::new(1.0, 3.0, 0.0)];

        let outcome = ingest_mapping_points(
            &mut adapter,
            &points,
            12,
            1_750_000_000_000_000_000,
            "odom",
            1_234_567_890_123_456_789,
            &state,
            &map_revision_tx,
        )
        .expect("MappingSession ingest");
        assert!(outcome.occupancy_committed);
        assert_eq!(outcome.accepted_map_points.len(), points.len());
        let mut cloud_map_points = outcome.accepted_map_points.clone();
        cloud_map_points.sort_by(|left, right| left.x.total_cmp(&right.x));
        assert!((cloud_map_points[0].x - 0.0).abs() < 1e-9);
        assert!((cloud_map_points[0].y - 0.0).abs() < 1e-9);
        assert!((cloud_map_points[1].x - 1.0).abs() < 1e-9);
        assert!((cloud_map_points[1].y - 0.0).abs() < 1e-9);

        assert_eq!(adapter.accumulated_count(), points.len());
        assert_eq!(adapter.odom_count(), 1);
        assert_eq!(adapter.pointcloud_count(), 1);
        assert_eq!(adapter.current_revision(), 1);
        let frame_contract = adapter.frame_contract("map");
        assert_eq!(frame_contract.source_frame_id, "odom");
        assert_eq!(frame_contract.output_frame_id, "map");
        assert!(frame_contract.transform_applied);
        assert!(frame_contract.ready_for_global_consumers());
        #[allow(deprecated)]
        {
            assert!(adapter.accumulator.clone_points().is_empty());
        }
        let event = map_revision_rx.try_recv().expect("map revision event");
        assert_eq!(event.revision, 1);
        assert_eq!(event.session_id, "test-session");
        assert_eq!(event.map_name, "test-map");
        assert_eq!(event.session_generation, 1);
        assert!(event.size > 0);
        assert_eq!(event.data.len() as u64, event.size);
        assert_eq!(
            event.checksum,
            crate::map_checksum::sha256_hex(event.data.as_ref())
        );
        let committed = state.try_read().expect("committed mapping state");
        assert_eq!(committed.map.frame_id, "map");
        assert_eq!(committed.map.timestamp_ns, 1_234_567_890_123_456_789);
        assert_eq!(committed.map.revision, event.revision);
        assert_eq!(committed.map.data.len(), event.data.len());
        let readiness = MappingReadiness::compute(Some(&adapter), true);
        assert!(readiness.ready);
        let path = std::env::temp_dir().join(format!(
            "nuwax-cloudmap-mapping-feed-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
        )));
        let current_session = adapter.snapshot();
        let event = outcome
            .occupancy_event
            .as_ref()
            .expect("occupancy event")
            .clone();
        let receipt = outcome
            .occupancy_receipt
            .as_ref()
            .expect("occupancy receipt")
            .clone();
        for invalid_receipt in [
            crate::mapping_commit::OccupancyCommitReceipt {
                session_id: "wrong-session".to_owned(),
                ..receipt.clone()
            },
            crate::mapping_commit::OccupancyCommitReceipt {
                session_generation: receipt.session_generation + 1,
                ..receipt.clone()
            },
            crate::mapping_commit::OccupancyCommitReceipt {
                occupancy_revision: receipt.occupancy_revision + 1,
                ..receipt.clone()
            },
            crate::mapping_commit::OccupancyCommitReceipt {
                source_timestamp_ns: receipt.source_timestamp_ns + 1,
                ..receipt.clone()
            },
        ] {
            let invalid_outcome = MappingIngestOutcome {
                sensor_origin_map: None,
                accepted_map_points: outcome.accepted_map_points.clone(),
                occupancy_event: Some(event.clone()),
                occupancy_receipt: Some(invalid_receipt),
                occupancy_committed: true,
                session_generation: outcome.session_generation,
                source_timestamp_ns: outcome.source_timestamp_ns,
            };
            assert!(update_mapping_cloud_map_if_current(
                &invalid_outcome,
                &current_session,
                &writer
            )
            .is_none());
            let writer_guard = writer.lock();
            let writer_state = writer_guard.as_ref().expect("cloud writer");
            assert!(writer_state.committed_snapshot().is_none());
            assert!(writer_state.last_cloud_receipt().is_none());
        }
        let update = update_mapping_cloud_map_if_current(&outcome, &current_session, &writer)
            .expect("cloud map update from mapping session");
        assert_eq!(update.notification.revision, 1);
        assert_eq!(update.notification.point_count, 2);
        assert_eq!(update.source_timestamp_ns, 1_234_567_890_123_456_789);
        assert_eq!(update.xyz_f32_le.len(), 24);
        assert_eq!(update.receipt.commit_id, receipt.commit_id);
        assert_eq!(update.receipt.occupancy_revision, event.revision);
        assert_eq!(update.receipt.cloud_revision, update.notification.revision);
        let mut stale_session = current_session.clone();
        stale_session.session_generation += 1;
        assert!(update_mapping_cloud_map_if_current(&outcome, &stale_session, &writer).is_none());
        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn delayed_session_one_global_map_publish_converges_to_session_two_clear() {
        const MAP_FRAME: &str = "map";
        let backend = FirstGlobalMapPublishGateBackend::default();
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let mut handle = build_handle_for_tests()
            .await
            .expect("global-map race test handle");
        handle.transport = transport.clone();
        let path = std::env::temp_dir().join(format!(
            "nuwax-global-map-session-race-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new(&path).expect("global-map race writer"));
        {
            let mut writer = handle.cloud_map_writer.lock();
            let writer = writer.as_mut().expect("global-map race writer");
            assert_eq!(
                writer.begin_session(1),
                Some((1, 0, crate::pointcloud::VOXEL_SIZE))
            );
            assert!(writer.accumulate_mapping_frame(1, &[Point3::new(1.0, 0.0, 1.0)]));
            let committed = writer
                .commit_accumulated_snapshot(&crate::mapping_commit::OccupancyCommitReceipt {
                    commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 1,
                        sequence: 1,
                    },
                    session_id: "session-one".to_owned(),
                    session_generation: 1,
                    occupancy_revision: 1,
                    source_timestamp_ns: 1_000_000_000,
                    backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                    backend_generation: 1,
                })
                .expect("session-one cloud commit")
                .expect("session-one cloud snapshot");
            assert_eq!(committed.snapshot.revision, 2);
        }
        handle.state.write().await.cloud_map_revision = 2;
        let mut cloud_updates = handle.cloud_map_tx.subscribe();

        let session_one_publish = {
            let transport = transport.clone();
            let writer = handle.cloud_map_writer.clone();
            let state = handle.state.clone();
            let cloud_map_tx = handle.cloud_map_tx.clone();
            tokio::spawn(async move {
                publish_latest_canonical_cloud_map(
                    &transport,
                    &writer,
                    &state,
                    &cloud_map_tx,
                    MAP_FRAME,
                )
                .await
            })
        };
        tokio::time::timeout(
            Duration::from_secs(1),
            backend.first_publish_started.notified(),
        )
        .await
        .expect("session-one typed publish entered gate");

        let clear = handle.begin_cloud_map_session(2).await;
        let CloudMapSessionAdmission::Registered(clear) = clear else {
            panic!("session-two clear revision was not registered");
        };
        assert_eq!(clear.revision, 3);
        assert_eq!(clear.point_count, 0);
        assert_eq!(clear.session_generation, 2);
        {
            let mut writer = handle.cloud_map_writer.lock();
            let writer = writer.as_mut().expect("global-map race writer");
            assert!(writer.accumulate_mapping_frame(2, &[Point3::new(2.0, 0.0, 2.0)]));
            let committed = writer
                .commit_accumulated_snapshot(&crate::mapping_commit::OccupancyCommitReceipt {
                    commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 1,
                        sequence: 2,
                    },
                    session_id: "session-two".to_owned(),
                    session_generation: 2,
                    occupancy_revision: 2,
                    source_timestamp_ns: 2_000_000_000,
                    backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                    backend_generation: 2,
                })
                .expect("session-two cloud commit")
                .expect("session-two cloud snapshot");
            assert_eq!(committed.snapshot.revision, 4);
            assert_eq!(committed.snapshot.point_count, 1);
        }
        let session_two_publish = {
            let transport = transport.clone();
            let writer = handle.cloud_map_writer.clone();
            let state = handle.state.clone();
            let cloud_map_tx = handle.cloud_map_tx.clone();
            tokio::spawn(async move {
                publish_latest_canonical_cloud_map(
                    &transport,
                    &writer,
                    &state,
                    &cloud_map_tx,
                    MAP_FRAME,
                )
                .await
            })
        };
        session_one_publish
            .await
            .expect("session-one publish join")
            .expect("session-one publish convergence")
            .expect("latest canonical cloud publication");
        session_two_publish
            .await
            .expect("session-two publish join")
            .expect("session-two publish convergence")
            .expect("latest canonical cloud publication");
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if backend
                    .published
                    .lock()
                    .expect("global-map published frames")
                    .len()
                    >= 2
                {
                    break;
                }
                backend.frame_published.notified().await;
            }
        })
        .await
        .expect("session-two typed clear publication");
        tokio::time::sleep(Duration::from_millis(20)).await;

        let published = backend
            .published
            .lock()
            .expect("global-map published frames")
            .clone();
        assert_eq!(published.len(), 2);
        assert_eq!(
            backend
                .publish_attempts
                .load(std::sync::atomic::Ordering::Acquire),
            3,
            "one stale S1 attempt must be cancelled before two ordered S2 frames"
        );
        let clear_frame = &published[0];
        assert_eq!(clear_frame.header.seqno, 3);
        let mut encoded = Vec::with_capacity(8 + clear_frame.payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&clear_frame.payload);
        let clear_cloud =
            schema::decode_point_cloud2(&encoded).expect("session-two empty PointCloud2");
        assert_eq!(clear_cloud.header.seq, 3);
        assert_eq!(clear_cloud.header.frame_id, MAP_FRAME);
        assert_eq!(clear_cloud.width, 0);
        assert!(clear_cloud.data.is_empty());

        let final_frame = published.last().expect("session-two typed cloud");
        assert_eq!(final_frame.header.seqno, 4);
        let mut encoded = Vec::with_capacity(8 + final_frame.payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&final_frame.payload);
        let final_cloud = schema::decode_point_cloud2(&encoded).expect("session-two PointCloud2");
        assert_eq!(final_cloud.header.seq, 4);
        assert_eq!(final_cloud.header.frame_id, MAP_FRAME);
        assert_eq!(final_cloud.width, 1);
        assert_eq!(final_cloud.data.len(), 12);
        assert_eq!(handle.state.read().await.cloud_map_revision, 4);
        let clear_update = cloud_updates
            .recv()
            .await
            .expect("session-two clear update");
        assert_eq!(clear_update.revision, 3);
        assert_eq!(clear_update.point_count, 0);
        assert_eq!(clear_update.session_generation, 2);
        let final_update = cloud_updates
            .recv()
            .await
            .expect("session-two cloud update");
        assert_eq!(final_update.revision, 4);
        assert_eq!(final_update.point_count, 1);
        assert_eq!(final_update.session_generation, 2);
        assert!(
            tokio::time::timeout(Duration::from_millis(50), cloud_updates.recv())
                .await
                .is_err()
        );
        let final_snapshot = handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .and_then(CloudMapShmWriter::committed_snapshot)
            .expect("session-two SHM clear");
        assert_eq!(final_snapshot.revision, 4);
        assert_eq!(final_snapshot.session_generation, 2);
        assert_eq!(final_snapshot.point_count, 1);
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn direct_global_map_checkpoint_publish_is_fenced_against_session_interleave() {
        const MAP_FRAME: &str = "map";
        let backend = FirstGlobalMapPublishGateBackend::default();
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let mut handle = build_handle_for_tests()
            .await
            .expect("direct global-map checkpoint test handle");
        handle.transport = transport;
        let mapping = Arc::new(parking_lot::Mutex::new(active_mapping_adapter()));
        handle.mapping_adapter = Some(mapping.clone());
        let path = std::env::temp_dir().join(format!(
            "nuwax-direct-global-map-fence-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new(&path).expect("direct global-map checkpoint writer"));

        let outcome = {
            let mut adapter = mapping.lock();
            adapter
                .feed_pose_in_frame(1.0, 2.0, 0.0, 0.0, "odom")
                .expect("mapping anchor");
            ingest_mapping_points(
                &mut adapter,
                &[Point3::new(1.0, 2.0, 0.0)],
                12,
                1_000_000_000,
                MAP_FRAME,
                1_000_000_000,
                &handle.state,
                &handle.map_revision_tx,
            )
            .expect("checkpoint source commit")
        };
        let committed = update_mapping_cloud_map_if_current(
            &outcome,
            &mapping.lock().snapshot(),
            &handle.cloud_map_writer,
        )
        .expect("checkpoint cloud commit");
        handle.state.write().await.cloud_map_revision = committed.notification.revision;
        let old_revision = committed.notification.revision;
        let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let direct_handle = handle.clone();
        let direct = tokio::spawn(async move {
            publish_committed_global_map_in_frame(&direct_handle, &committed, MAP_FRAME).await
        });
        tokio::time::timeout(
            Duration::from_secs(1),
            backend.first_publish_started.notified(),
        )
        .await
        .expect("direct checkpoint publish entered the transport gate");

        let clear = handle.begin_cloud_map_session(2).await;
        let CloudMapSessionAdmission::Registered(clear) = clear else {
            panic!("new session clear revision was not registered");
        };
        assert_eq!(clear.session_generation, 2);
        assert_eq!(clear.point_count, 0);
        drop(_lifecycle_guard);

        let direct_result = tokio::time::timeout(Duration::from_secs(1), direct)
            .await
            .expect("direct checkpoint task deadline")
            .expect("direct checkpoint task join");
        assert!(direct_result
            .expect_err("stale direct checkpoint must fail closed")
            .contains("identity"));
        tokio::time::timeout(Duration::from_secs(1), backend.frame_published.notified())
            .await
            .expect("new-session canonical clear publication");
        let published = backend
            .published
            .lock()
            .expect("direct checkpoint published frames")
            .clone();
        assert!(
            published
                .iter()
                .all(|frame| u64::from(frame.header.seqno) != old_revision),
            "a direct checkpoint from the retired session must never reach transport"
        );

        *handle.cloud_map_writer.lock() = None;
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn newer_same_session_revision_cancels_blocked_old_global_map_publish() {
        const MAP_FRAME: &str = "map";
        let path = std::env::temp_dir().join(format!(
            "nuwax-global-map-revision-race-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        let writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("global-map revision race writer"),
        )));
        {
            let mut writer_guard = writer.lock();
            let writer = writer_guard
                .as_mut()
                .expect("global-map revision race writer");
            assert_eq!(
                writer.begin_session(1),
                Some((1, 0, crate::pointcloud::VOXEL_SIZE))
            );
            assert!(writer.accumulate_mapping_frame(1, &[Point3::new(1.0, 0.0, 1.0)]));
            let committed = writer
                .commit_accumulated_snapshot(&crate::mapping_commit::OccupancyCommitReceipt {
                    commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 1,
                        sequence: 1,
                    },
                    session_id: "same-session".to_owned(),
                    session_generation: 1,
                    occupancy_revision: 1,
                    source_timestamp_ns: 1_000_000_000,
                    backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                    backend_generation: 1,
                })
                .expect("initial same-session cloud commit")
                .expect("initial same-session cloud snapshot");
            assert_eq!(committed.snapshot.revision, 2);
        }

        // Drain the session-clear marker before installing the gate, leaving
        // the blocked call focused on the first non-empty revision.
        let priming_backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/unused/global-map-primer").expect("primer topic"),
        );
        let priming_transport = LcmTransport::new(Arc::new(priming_backend));
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (cloud_map_tx, _) = broadcast::channel(8);
        publish_latest_canonical_cloud_map(
            &priming_transport,
            &writer,
            &state,
            &cloud_map_tx,
            MAP_FRAME,
        )
        .await
        .expect("prime canonical cloud publication")
        .expect("primed canonical cloud snapshot");

        let old_revision = {
            let mut writer_guard = writer.lock();
            let writer = writer_guard
                .as_mut()
                .expect("global-map revision race writer");
            assert!(writer.accumulate_mapping_frame(1, &[Point3::new(2.0, 0.0, 2.0)]));
            let committed = writer
                .commit_accumulated_snapshot(&crate::mapping_commit::OccupancyCommitReceipt {
                    commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 1,
                        sequence: 2,
                    },
                    session_id: "same-session".to_owned(),
                    session_generation: 1,
                    occupancy_revision: 2,
                    source_timestamp_ns: 2_000_000_000,
                    backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                    backend_generation: 1,
                })
                .expect("blocked same-session cloud commit")
                .expect("blocked same-session cloud snapshot");
            committed.snapshot.revision
        };

        let backend = SameSessionGlobalMapPublishGateBackend::default();
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let first_publish_started = backend.first_publish_started.notified();
        let blocked_old = {
            let transport = transport.clone();
            let writer = writer.clone();
            let state = state.clone();
            let cloud_map_tx = cloud_map_tx.clone();
            tokio::spawn(async move {
                publish_latest_canonical_cloud_map(
                    &transport,
                    &writer,
                    &state,
                    &cloud_map_tx,
                    MAP_FRAME,
                )
                .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), first_publish_started)
            .await
            .expect("old same-session publish entered gate");

        let newer_revision = {
            let mut writer_guard = writer.lock();
            let writer = writer_guard
                .as_mut()
                .expect("global-map revision race writer");
            assert!(writer.accumulate_mapping_frame(1, &[Point3::new(3.0, 0.0, 3.0)]));
            let committed = writer
                .commit_accumulated_snapshot(&crate::mapping_commit::OccupancyCommitReceipt {
                    commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 1,
                        sequence: 3,
                    },
                    session_id: "same-session".to_owned(),
                    session_generation: 1,
                    occupancy_revision: 3,
                    source_timestamp_ns: 3_000_000_000,
                    backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                    backend_generation: 1,
                })
                .expect("newer same-session cloud commit")
                .expect("newer same-session cloud snapshot");
            committed.snapshot.revision
        };

        let newer_publish_started = backend.frame_published.notified();
        let newer = {
            let transport = transport.clone();
            let writer = writer.clone();
            let state = state.clone();
            let cloud_map_tx = cloud_map_tx.clone();
            tokio::spawn(async move {
                publish_latest_canonical_cloud_map(
                    &transport,
                    &writer,
                    &state,
                    &cloud_map_tx,
                    MAP_FRAME,
                )
                .await
            })
        };
        tokio::time::timeout(Duration::from_secs(1), newer_publish_started)
            .await
            .expect("newer same-session revision published first");
        {
            let published = backend
                .published
                .lock()
                .expect("same-session global-map published frames");
            assert_eq!(published.len(), 1);
            assert_eq!(
                published[0].header.seqno,
                u32::try_from(newer_revision).expect("newer revision fits typed sequence")
            );
            assert_ne!(
                published[0].header.seqno,
                u32::try_from(old_revision).expect("old revision fits typed sequence")
            );
        }

        // The old backend call has already been canceled by the revision
        // invalidation. Releasing its gate must not emit the stale snapshot.
        backend.release_first_publish();
        tokio::time::timeout(Duration::from_secs(1), blocked_old)
            .await
            .expect("blocked old publish joins after cancellation")
            .expect("blocked old publish task")
            .expect("blocked old publish converges")
            .expect("blocked old publish returns a canonical snapshot");
        newer
            .await
            .expect("newer publish task")
            .expect("newer publish converges")
            .expect("newer publish returns a canonical snapshot");
        assert_eq!(
            backend
                .publish_attempts
                .load(std::sync::atomic::Ordering::Acquire),
            2
        );
        assert_eq!(
            backend
                .published
                .lock()
                .expect("same-session global-map published frames")
                .len(),
            1
        );
        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test]
    async fn committed_cumulative_map_reaches_person_follow_decoder_once_per_revision() {
        use nuwax_nav_visual_servoing::person_follow::PackedPointCloud;

        const MAP_FRAME: &str = "robot/map";
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), MAP_FRAME, "odom");
        adapter
            .start_session("test-session", "test-map")
            .expect("active mapping test session");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (map_revision_tx, _) = broadcast::channel(4);
        let (cloud_map_tx, _) = broadcast::channel(4);
        let path = std::env::temp_dir().join(format!(
            "nuwax-global-map-lcm-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
        )));
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/unused/odom").expect("dummy odom topic"),
        );
        let transport = LcmTransport::new(Arc::new(backend.clone()));

        for (index, point) in [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0)]
            .into_iter()
            .enumerate()
        {
            let timestamp_ns = 1_700_000_000_000_000_000 + index as u64 * 3_000_000_000;
            let outcome = ingest_mapping_points(
                &mut adapter,
                &[point],
                12,
                timestamp_ns,
                MAP_FRAME,
                timestamp_ns,
                &state,
                &map_revision_tx,
            )
            .expect("accepted canonical map frame");
            assert!(outcome.occupancy_committed);
            update_mapping_cloud_map_if_current(&outcome, &adapter.snapshot(), &writer)
                .expect("committed cumulative cloud map");
            publish_latest_canonical_cloud_map(
                &transport,
                &writer,
                &state,
                &cloud_map_tx,
                MAP_FRAME,
            )
            .await
            .expect("typed global-map publish")
            .expect("canonical cloud snapshot");
        }

        let published = backend.published.lock().expect("published frames").clone();
        assert_eq!(published.len(), 2, "one typed frame per committed revision");
        let latest = published.last().expect("latest global-map frame");
        assert_eq!(latest.topic.channel(), GLOBAL_MAP_CHANNEL);
        assert_eq!(latest.msg_type.fqn(), POINT_CLOUD2_MESSAGE_TYPE);
        assert_eq!(latest.header.seqno, 2);
        let mut encoded = Vec::with_capacity(8 + latest.payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&latest.payload);
        let cloud = schema::decode_point_cloud2(&encoded).expect("typed PointCloud2 body");
        assert_eq!(cloud.header.frame_id, MAP_FRAME);
        assert_eq!(cloud.header.seq, 2);
        assert_eq!(
            cloud.width, 2,
            "second revision is cumulative, not latest-only"
        );
        assert_eq!(cloud.point_step, 12);
        assert_eq!(cloud.row_step, 24);
        let decoded = PackedPointCloud::from_lcm(&cloud)
            .expect("person-follow decoder accepts configured map frame");
        assert_eq!(decoded.frame_id(), MAP_FRAME);
        assert_eq!(decoded.xyz().len(), 2);

        let replay_timestamp = 1_700_000_003_000_000_000;
        let replay = ingest_mapping_points(
            &mut adapter,
            &[Point3::new(1.0, 2.0, 3.0)],
            12,
            replay_timestamp + 1,
            MAP_FRAME,
            replay_timestamp,
            &state,
            &map_revision_tx,
        )
        .expect("dedup replay is a non-error rejection");
        assert!(replay.accepted_map_points.is_empty());
        assert!(
            update_mapping_cloud_map_if_current(&replay, &adapter.snapshot(), &writer).is_none()
        );

        let pending = ingest_mapping_points(
            &mut adapter,
            &[Point3::new(4.0, 5.0, 6.0)],
            12,
            replay_timestamp + 3_000_000_000,
            MAP_FRAME,
            replay_timestamp + 3_000_000_000,
            &state,
            &map_revision_tx,
        )
        .expect("accepted before stop");
        adapter
            .stop_session("test-session")
            .expect("stop active session");
        assert!(
            update_mapping_cloud_map_if_current(&pending, &adapter.snapshot(), &writer).is_none()
        );
        assert_eq!(
            backend.published_count(),
            2,
            "replayed/stopped frames publish nothing"
        );

        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn cloud_map_handoff_is_new_frame_only_and_replay_safe() {
        let mut adapter = active_mapping_adapter();
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("establish odom anchor");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (map_revision_tx, _map_revision_rx) = broadcast::channel(4);
        let first_points = (0..1_000)
            .map(|index| Point3::new(f64::from(index) * 0.01, 0.0, 0.0))
            .collect::<Vec<_>>();
        let first = ingest_mapping_points(
            &mut adapter,
            &first_points,
            12,
            1_750_000_000_000_000_000,
            "odom",
            1_234_567_890_000_000_000,
            &state,
            &map_revision_tx,
        )
        .expect("first frame");
        assert_eq!(first.accepted_map_points.len(), first_points.len());

        let second_points = vec![Point3::new(20.0, 0.0, 0.0), Point3::new(21.0, 0.0, 0.0)];
        let second = ingest_mapping_points(
            &mut adapter,
            &second_points,
            12,
            1_750_000_000_100_000_000,
            "odom",
            1_234_567_890_100_000_000,
            &state,
            &map_revision_tx,
        )
        .expect("second frame");
        assert_eq!(
            adapter.accumulated_count(),
            first_points.len() + second_points.len()
        );
        assert_eq!(
            second.accepted_map_points.len(),
            second_points.len(),
            "cloud-map handoff must stay O(new frame), not O(session window)"
        );

        let replay = ingest_mapping_points(
            &mut adapter,
            &second_points,
            12,
            1_750_000_000_100_000_100,
            "odom",
            1_234_567_890_100_000_000,
            &state,
            &map_revision_tx,
        )
        .expect("dedup replay");
        assert!(replay.accepted_map_points.is_empty());
        assert!(!replay.occupancy_committed);
    }

    #[tokio::test]
    async fn stop_between_lidar_ingest_and_cloud_commit_rejects_old_generation() {
        let mut adapter = active_mapping_adapter();
        adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
            .expect("establish odom anchor");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let (map_revision_tx, _) = broadcast::channel(4);
        let outcome = ingest_mapping_points(
            &mut adapter,
            &[Point3::new(1.0, 2.0, 0.0)],
            12,
            1_750_000_000_000_000_000,
            "odom",
            1_234_567_890_123_456_789,
            &state,
            &map_revision_tx,
        )
        .expect("LiDAR ingest before stop");
        assert!(outcome.occupancy_committed);
        adapter
            .stop_session("test-session")
            .expect("stop between ingest and cloud commit");

        let path = std::env::temp_dir().join(format!(
            "nuwax-cloudmap-stop-fence-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        let writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
        )));
        let lifecycle_lock = tokio::sync::Mutex::new(());
        let _lifecycle_guard = lifecycle_lock.lock().await;
        let stopped_session = adapter.snapshot();
        assert!(!stopped_session.active);
        assert!(
            update_mapping_cloud_map_if_current(&outcome, &stopped_session, &writer).is_none(),
            "post-stop LiDAR outcome must not commit cloud SHM or notify"
        );
        drop(writer);
        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn mapping_feed_rejects_unknown_pose_and_cloud_frames() {
        let mut adapter = active_mapping_adapter();
        assert!(adapter
            .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "base_link")
            .is_err());
        assert_eq!(adapter.odom_count(), 0);
        let points = [Point3::new(0.0, 0.0, 0.0)];
        assert!(adapter
            .feed_pointcloud_session(&points, (0.0, 0.0), 12, 10, "laser", 10)
            .is_err());
        assert_eq!(adapter.pointcloud_count(), 0);
    }

    #[test]
    fn canonical_tf_uses_anchor_math_and_odometry_source_timestamp() {
        let mut adapter = active_mapping_adapter();
        adapter
            .feed_pose_in_frame(2.0, 3.0, 0.0, std::f64::consts::FRAC_PI_2, "odom")
            .expect("anchor");
        let pose = PoseState {
            x: 2.0,
            y: 4.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 2.0 * std::f64::consts::FRAC_PI_4.sin(),
            qw: 2.0 * std::f64::consts::FRAC_PI_4.cos(),
            frame_id: "odom".to_string(),
            timestamp_ns: 12_345_678_901,
        };
        let snapshot = adapter
            .map_odom_transform_snapshot()
            .expect("anchored transform");
        let observation = canonical_mapping_observation(&snapshot, &pose, "base_link")
            .expect("canonical observation");

        assert!((observation.map_pose.x - 1.0).abs() < 1e-9);
        assert!(observation.map_pose.y.abs() < 1e-9);
        assert_eq!(observation.map_pose.frame_id, "map");
        assert_eq!(observation.map_pose.timestamp_ns, pose.timestamp_ns);
        assert!((observation.transforms[0].tx + 3.0).abs() < 1e-9);
        assert!((observation.transforms[0].ty - 2.0).abs() < 1e-9);
        assert_eq!(observation.transforms[0].stamp_ns, pose.timestamp_ns);
        assert_eq!(observation.transforms[1].stamp_ns, pose.timestamp_ns);

        let decoded = parse_tf_payload(&observation.tf_payload).expect("decode canonical TF");
        assert_eq!(decoded.len(), 2);
        assert_eq!(
            (decoded[0].parent.as_str(), decoded[0].child.as_str()),
            ("map", "odom")
        );
        assert_eq!(
            (decoded[1].parent.as_str(), decoded[1].child.as_str()),
            ("odom", "base_link")
        );
        assert!(decoded
            .iter()
            .all(|entry| entry.stamp_ns == pose.timestamp_ns));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn canonical_correction_applier_preserves_full_se3_over_real_lcm() {
        let provider = test_lcm_url();
        let publisher = LcmTransport::new_real(&provider).expect("correction TF publisher");
        let subscriber = LcmTransport::new_real(&provider).expect("correction TF subscriber");
        assert!(!Arc::ptr_eq(
            &publisher.shared_lcm_context().unwrap(),
            &subscriber.shared_lcm_context().unwrap()
        ));
        let topic = LcmTopic::new(&format!("/ctf/{}", std::process::id())).unwrap();
        let message_type = LcmMessageType::new("tf2_msgs.TFMessage").unwrap();
        let mut subscription = subscriber
            .subscribe_owned(topic.clone(), Some(message_type.clone()))
            .await
            .expect("correction TF subscription ACK");
        let source_timestamp_ns = 1_234_567_890;
        let occupancy_revision = 11;
        let cloud_revision = 21;
        let state = Arc::new(RwLock::new(SharedState {
            map: MapState {
                width: 1,
                height: 1,
                resolution: 0.1,
                origin_x: 0.0,
                origin_y: 0.0,
                frame_id: "robot_map".into(),
                revision: occupancy_revision,
                timestamp_ns: source_timestamp_ns,
                data: vec![0],
            },
            cloud_map_revision: cloud_revision,
            ..SharedState::default()
        }));
        let identity = crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
            runtime_generation: 31,
            backend_generation: 32,
            correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                boot_nonce: 0x1234,
                generation: 35,
            },
        };
        let applier = GeometryCanonicalCorrectionApplier::new(
            identity.clone(),
            "robot_map",
            "robot_odom",
            "robot_body",
            state.clone(),
            publisher,
            (topic, message_type),
        )
        .unwrap();
        let base_anchor = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(4.0, -2.0, 1.0),
            nalgebra::UnitQuaternion::from_euler_angles(0.2, -0.1, 0.4),
        );
        let correction = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(-0.5, 0.7, 0.3),
            nalgebra::UnitQuaternion::from_euler_angles(-0.3, 0.25, -0.2),
        );
        let frozen_odom = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(1.2, 0.8, -0.4),
            nalgebra::UnitQuaternion::from_euler_angles(0.15, -0.35, 0.6),
        );
        let effective = base_anchor * correction;
        let transaction = crate::correction_coordinator::CanonicalCorrectionTransaction {
            runtime_generation: identity.runtime_generation,
            backend_generation: identity.backend_generation,
            backend: crate::correction_coordinator::CorrectionCandidateBackend::Relocalization,
            candidate_sequence: 1,
            payload_sha256: [0x5a; 32],
            correction_revision: 1,
            mapping_session_id: "canonical-correction-session".into(),
            mapping_session_generation: 33,
            mapping_commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 0x5678,
                sequence: 1,
            },
            occupancy_revision,
            cloud_revision,
            map_frame_generation: 34,
            source_timestamp_ns,
            committed_map_source_timestamp_ns: source_timestamp_ns,
            map_frame_id: "robot_map".into(),
            odom_frame_id: "robot_odom".into(),
            body_frame_id: "robot_body".into(),
            correction_lease: identity.correction_lease,
            base_anchor_map_from_odom: base_anchor,
            absolute_left_correction: correction,
            effective_map_from_odom: effective,
            frozen_odom_source_timestamp_ns: source_timestamp_ns,
            frozen_odom_from_body: frozen_odom,
        };
        let cancellation = crate::correction_coordinator::CorrectionRequestCancellation::new();
        crate::correction_coordinator::CanonicalCorrectionApplier::apply_transaction(
            &applier,
            transaction.clone(),
            cancellation.clone(),
        )
        .await
        .unwrap();
        cancellation
            .operation()
            .mark_coordinator_finalized()
            .unwrap();
        cancellation.finish_canonical_commit();

        let received = tokio::time::timeout(Duration::from_secs(2), subscription.recv())
            .await
            .expect("canonical correction TF receive timeout")
            .expect("canonical correction TF frame");
        let decoded = parse_tf_payload(&received.payload).expect("full-SE3 correction TF");
        assert_eq!(decoded.len(), 2);
        let map_odom = &decoded[0];
        assert_eq!(
            (map_odom.parent.as_str(), map_odom.child.as_str()),
            ("robot_map", "robot_odom")
        );
        assert!((map_odom.tx - effective.translation.vector.x).abs() < 1.0e-9);
        assert!((map_odom.ty - effective.translation.vector.y).abs() < 1.0e-9);
        assert!((map_odom.tz - effective.translation.vector.z).abs() < 1.0e-9);
        let effective_rotation = effective.rotation.quaternion();
        assert!((map_odom.rx - effective_rotation.i).abs() < 1.0e-9);
        assert!((map_odom.ry - effective_rotation.j).abs() < 1.0e-9);
        assert!((map_odom.rz - effective_rotation.k).abs() < 1.0e-9);
        assert!((map_odom.rw - effective_rotation.w).abs() < 1.0e-9);
        assert!(map_odom.rx.abs() > 1.0e-3 || map_odom.ry.abs() > 1.0e-3);

        let expected_map_body = effective * frozen_odom;
        let shared = state.read().await;
        assert!((shared.pose.x - expected_map_body.translation.vector.x).abs() < 1.0e-9);
        assert!((shared.pose.y - expected_map_body.translation.vector.y).abs() < 1.0e-9);
        assert!((shared.pose.z - expected_map_body.translation.vector.z).abs() < 1.0e-9);
        assert_eq!(shared.map.revision, occupancy_revision);
        assert_eq!(shared.cloud_map_revision, cloud_revision);
        drop(shared);
        let snapshot = applier.snapshot().expect("applied correction snapshot");
        assert_eq!(snapshot.correction_revision, 1);
        assert!(canonical_isometry_near(
            &snapshot.effective_map_from_odom,
            &effective
        ));

        // A second relocalization uses a new cloud timestamp against the same
        // immutable map receipt.  This is legal only for relocalization; the
        // map/occupancy/cloud identities must remain exactly unchanged.
        let mut second_relocalization = transaction.clone();
        second_relocalization.candidate_sequence = 2;
        second_relocalization.correction_revision = 2;
        second_relocalization.source_timestamp_ns += 1;
        let second_cancellation =
            crate::correction_coordinator::CorrectionRequestCancellation::new();
        crate::correction_coordinator::CanonicalCorrectionApplier::apply_transaction(
            &applier,
            second_relocalization.clone(),
            second_cancellation.clone(),
        )
        .await
        .expect("same-map second relocalization is accepted");
        second_cancellation
            .operation()
            .mark_coordinator_finalized()
            .unwrap();
        second_cancellation.finish_canonical_commit();
        let _ = tokio::time::timeout(Duration::from_secs(2), subscription.recv())
            .await
            .expect("second relocalization TF receive timeout")
            .expect("second relocalization TF frame");
        let second_snapshot = applier.snapshot().expect("second correction snapshot");
        assert_eq!(second_snapshot.correction_revision, 2);
        assert_eq!(
            second_snapshot.mapping_commit_id,
            transaction.mapping_commit_id
        );
        assert_eq!(
            second_snapshot.occupancy_revision,
            transaction.occupancy_revision
        );
        assert_eq!(second_snapshot.cloud_revision, transaction.cloud_revision);
        assert_eq!(
            second_snapshot.committed_map_source_timestamp_ns,
            transaction.committed_map_source_timestamp_ns
        );
        assert_eq!(
            second_snapshot.source_timestamp_ns,
            transaction.source_timestamp_ns + 1
        );

        let mut library_timestamp_split = second_relocalization.clone();
        library_timestamp_split.backend =
            crate::correction_coordinator::CorrectionCandidateBackend::LibraryPgo;
        library_timestamp_split.candidate_sequence = 3;
        library_timestamp_split.correction_revision = 3;
        library_timestamp_split.source_timestamp_ns += 1;
        assert!(
            applier
                .validate_transaction(&library_timestamp_split)
                .is_err(),
            "library PGO must retain same map/cloud source timestamp policy"
        );
        let mut native_timestamp_split = library_timestamp_split.clone();
        native_timestamp_split.backend =
            crate::correction_coordinator::CorrectionCandidateBackend::NativePgo;
        assert!(
            applier
                .validate_transaction(&native_timestamp_split)
                .is_err(),
            "native PGO must retain same map/cloud source timestamp policy"
        );

        let mut wrong_revision = transaction;
        wrong_revision.correction_revision = 3;
        assert!(
            crate::correction_coordinator::CanonicalCorrectionApplier::apply_transaction(
                &applier,
                wrong_revision,
                crate::correction_coordinator::CorrectionRequestCancellation::new(),
            )
            .await
            .is_err()
        );
        assert!(
            tokio::time::timeout(Duration::from_millis(100), subscription.recv())
                .await
                .is_err()
        );
        subscription.unsubscribe().await.unwrap();
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn correction_of_delayed_scan_preserves_newer_odometry_pose_and_timestamp() {
        use crate::correction_coordinator::{
            CanonicalCorrectionApplier, CanonicalCorrectionTransaction, CorrectionCandidateBackend,
            CorrectionLeaseId, CorrectionOwnerRuntimeIdentity, CorrectionRequestCancellation,
        };
        for backend in [
            CorrectionCandidateBackend::LibraryPgo,
            CorrectionCandidateBackend::NativePgo,
            CorrectionCandidateBackend::Relocalization,
        ] {
            for (scan_delta, latest_delta) in [
                (0, Some(100_000_000)),
                (100_000_000, Some(50_000_000)),
                (100_000_000, None),
            ] {
                let handle = build_handle_for_tests().await.unwrap();
                let frozen_timestamp_ns = 60_000_000_000;
                let source_timestamp_ns = frozen_timestamp_ns + scan_delta;
                let latest_timestamp_ns = frozen_timestamp_ns + latest_delta.unwrap_or(0);
                let identity = CorrectionOwnerRuntimeIdentity {
                    runtime_generation: 7,
                    backend_generation: 7,
                    correction_lease: CorrectionLeaseId {
                        boot_nonce: 42,
                        generation: 7,
                    },
                };
                let tf = (
                    LcmTopic::new("/test/delayed-pgo/tf").unwrap(),
                    LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
                );
                let applier = GeometryCanonicalCorrectionApplier::new(
                    identity.clone(),
                    "map",
                    "odom",
                    "base_link",
                    handle.state.clone(),
                    handle.transport.clone(),
                    tf,
                )
                .unwrap();
                let anchor = nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(2.0, -1.0, 0.3),
                    nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.1, 0.4),
                );
                let correction = nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(-0.15, 0.02, 0.01),
                    nalgebra::UnitQuaternion::from_euler_angles(0.02, 0.01, -0.03),
                );
                let frozen = nalgebra::Isometry3::translation(0.15, 0.0, 0.0);
                let latest = nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(0.25, 0.01, 0.0),
                    nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.01, 0.1),
                );
                let latest_pose = pose_state_from_isometry(&latest, "odom", latest_timestamp_ns);
                let latest_observation = canonical_mapping_observation_from_isometry(
                    "map",
                    "odom",
                    1,
                    &anchor,
                    &latest_pose,
                    "base_link",
                )
                .unwrap();
                {
                    let mut shared = handle.state.write().await;
                    shared.map = MapState {
                        width: 1,
                        height: 1,
                        resolution: 0.1,
                        origin_x: 0.0,
                        origin_y: 0.0,
                        frame_id: "map".into(),
                        revision: 11,
                        timestamp_ns: source_timestamp_ns,
                        data: vec![0],
                    };
                    shared.cloud_map_revision = 21;
                    // 点云仍在处理时，较新的真实里程计已经发布到规范状态和坐标树。
                    if latest_delta.is_some() {
                        apply_canonical_mapping_observation_locked(
                            &mut shared,
                            &latest_observation,
                        );
                    }
                }
                let transaction = CanonicalCorrectionTransaction {
                    runtime_generation: 7,
                    backend_generation: 7,
                    backend,
                    candidate_sequence: 1,
                    payload_sha256: [3; 32],
                    correction_revision: 1,
                    mapping_session_id: "delayed-pgo".into(),
                    mapping_session_generation: 1,
                    mapping_commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 43,
                        sequence: 1,
                    },
                    occupancy_revision: 11,
                    cloud_revision: 21,
                    map_frame_generation: 1,
                    source_timestamp_ns,
                    committed_map_source_timestamp_ns: source_timestamp_ns,
                    map_frame_id: "map".into(),
                    odom_frame_id: "odom".into(),
                    body_frame_id: "base_link".into(),
                    correction_lease: identity.correction_lease,
                    base_anchor_map_from_odom: anchor,
                    absolute_left_correction: correction,
                    effective_map_from_odom: anchor * correction,
                    frozen_odom_source_timestamp_ns: frozen_timestamp_ns,
                    frozen_odom_from_body: frozen,
                };
                // 公共位姿较新而同刻里程计缺失时，拒绝候选并保留既有公共状态。
                if latest_delta.is_some() {
                    let latest_tree = {
                        let mut shared = handle.state.write().await;
                        std::mem::replace(&mut shared.tf_tree, Arc::new(TfTree::new()))
                    };
                    assert!(applier
                        .apply_transaction(
                            transaction.clone(),
                            CorrectionRequestCancellation::new()
                        )
                        .await
                        .is_err());
                    assert!(applier.snapshot().is_none());
                    {
                        let mut shared = handle.state.write().await;
                        assert_eq!(shared.pose.timestamp_ns, latest_timestamp_ns);
                        shared.tf_tree = latest_tree;
                    }
                }
                let cancellation = CorrectionRequestCancellation::new();
                applier
                    .apply_transaction(transaction, cancellation.clone())
                    .await
                    .unwrap();
                cancellation
                    .operation()
                    .mark_coordinator_finalized()
                    .unwrap();
                cancellation.finish_canonical_commit();
                let shared = handle.state.read().await;
                let expected = anchor
                    * correction
                    * if latest_delta.is_some() {
                        latest
                    } else {
                        frozen
                    };
                let actual = nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(shared.pose.x, shared.pose.y, shared.pose.z),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        shared.pose.qw,
                        shared.pose.qx,
                        shared.pose.qy,
                        shared.pose.qz,
                    )),
                );
                println!("延迟校正 backend={backend:?} source={source_timestamp_ns} latest={latest_timestamp_ns} published={} error={:?}",
                shared.pose.timestamp_ns, expected.inverse()*actual);
                assert_eq!(
                    shared.pose.timestamp_ns, latest_timestamp_ns,
                    "校正不能倒退公共位姿的源时间"
                );
                assert!(canonical_isometry_near(&actual, &expected));
                let tree_pose = shared.tf_tree.lookup("base_link").unwrap();
                assert_eq!(
                    transform_to_entry(tree_pose).unwrap().stamp_ns,
                    latest_timestamp_ns
                );
                assert_eq!(
                    applier.snapshot().unwrap().source_timestamp_ns,
                    source_timestamp_ns,
                    "候选证明仍属于原扫描，不能伪造为较新里程计来源"
                );
            }
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn correction_commit_cannot_be_overwritten_by_stale_odometry_snapshot() {
        let handle = build_handle_for_tests()
            .await
            .expect("test Geometry handle");
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .clone();
        let (mapping_session_id, mapping_session_generation, map_frame_generation) = {
            let mut adapter = mapping.lock();
            adapter
                .start_session("correction-race-session", "correction-race-map")
                .expect("active mapping session");
            adapter
                .feed_pose_in_frame(0.0, 0.0, 0.0, 0.0, "odom")
                .expect("canonical odometry anchor");
            let snapshot = adapter.snapshot();
            (
                snapshot.session_id,
                snapshot.session_generation,
                adapter.global_frame_generation(),
            )
        };
        let source_timestamp_ns = 500;
        let occupancy_revision = 11;
        let cloud_revision = 21;
        {
            let mut shared = handle.state.write().await;
            shared.map = MapState {
                width: 1,
                height: 1,
                resolution: 0.1,
                origin_x: 0.0,
                origin_y: 0.0,
                frame_id: "map".into(),
                revision: occupancy_revision,
                timestamp_ns: source_timestamp_ns,
                data: vec![0],
            };
            shared.cloud_map_revision = cloud_revision;
        }
        let identity = crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
            runtime_generation: 51,
            backend_generation: 52,
            correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                boot_nonce: 0x1234,
                generation: 53,
            },
        };
        let tf_binding = (
            LcmTopic::new(&format!("/correction-race/{}", std::process::id())).unwrap(),
            LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
        );
        let applier = Arc::new(
            GeometryCanonicalCorrectionApplier::new(
                identity.clone(),
                "map",
                "odom",
                "base_link",
                handle.state.clone(),
                handle.transport.clone(),
                tf_binding.clone(),
            )
            .expect("canonical correction applier"),
        );
        let old_effective = nalgebra::Isometry3::translation(2.0, 0.0, 0.0);
        *applier.current.write() = Some(CanonicalCorrectionSnapshot {
            runtime_generation: identity.runtime_generation,
            backend_generation: identity.backend_generation,
            backend: crate::correction_coordinator::CorrectionCandidateBackend::Relocalization,
            mapping_session_id: mapping_session_id.clone(),
            mapping_session_generation,
            map_frame_generation,
            mapping_commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 0x5678,
                sequence: 1,
            },
            occupancy_revision: 10,
            cloud_revision: 20,
            correction_revision: 1,
            source_timestamp_ns: 400,
            committed_map_source_timestamp_ns: 400,
            map_frame_id: "map".into(),
            odom_frame_id: "odom".into(),
            body_frame_id: "base_link".into(),
            correction_lease: identity.correction_lease,
            effective_map_from_odom: old_effective,
        });

        let new_effective = nalgebra::Isometry3::translation(9.0, -1.0, 0.5);
        let frozen_odom_from_body = nalgebra::Isometry3::translation(0.25, 0.5, -0.25);
        let transaction = crate::correction_coordinator::CanonicalCorrectionTransaction {
            runtime_generation: identity.runtime_generation,
            backend_generation: identity.backend_generation,
            backend: crate::correction_coordinator::CorrectionCandidateBackend::Relocalization,
            candidate_sequence: 2,
            payload_sha256: [0x6b; 32],
            correction_revision: 2,
            mapping_session_id,
            mapping_session_generation,
            mapping_commit_id: crate::mapping_commit::MappingCommitId {
                boot_nonce: 0x5678,
                sequence: 2,
            },
            occupancy_revision,
            cloud_revision,
            map_frame_generation,
            source_timestamp_ns,
            committed_map_source_timestamp_ns: source_timestamp_ns,
            map_frame_id: "map".into(),
            odom_frame_id: "odom".into(),
            body_frame_id: "base_link".into(),
            correction_lease: identity.correction_lease,
            base_anchor_map_from_odom: nalgebra::Isometry3::identity(),
            absolute_left_correction: new_effective,
            effective_map_from_odom: new_effective,
            frozen_odom_source_timestamp_ns: source_timestamp_ns,
            frozen_odom_from_body,
        };

        let blocked_current = applier.current.clone();
        let (current_locked_tx, current_locked_rx) = tokio::sync::oneshot::channel();
        let (release_current_tx, release_current_rx) = std::sync::mpsc::channel();
        let current_blocker = tokio::task::spawn_blocking(move || {
            let guard = blocked_current.write();
            let _ = current_locked_tx.send(());
            release_current_rx
                .recv_timeout(Duration::from_secs(2))
                .expect("release correction snapshot lock");
            drop(guard);
        });
        current_locked_rx
            .await
            .expect("correction snapshot lock acquired");

        let odom_handle = handle.clone();
        let odom_applier = Arc::clone(&applier);
        let odom_binding = tf_binding.clone();
        let odom = tokio::spawn(async move {
            handoff_lio_odometry_with_correction(
                &odom_handle,
                &PoseState {
                    x: 1.0,
                    y: 2.0,
                    z: 0.0,
                    qw: 1.0,
                    frame_id: "odom".into(),
                    timestamp_ns: 450,
                    ..PoseState::default()
                },
                "base_link",
                &odom_binding,
                odom_applier.as_ref(),
            )
            .await
        });
        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.state.try_write().is_ok() {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("odometry must hold canonical state while selecting its snapshot");

        let correction_applier = Arc::clone(&applier);
        let correction = tokio::spawn(async move {
            let cancellation = crate::correction_coordinator::CorrectionRequestCancellation::new();
            let result =
                crate::correction_coordinator::CanonicalCorrectionApplier::apply_transaction(
                    correction_applier.as_ref(),
                    transaction,
                    cancellation.clone(),
                )
                .await;
            if result.is_ok() {
                cancellation
                    .operation()
                    .mark_coordinator_finalized()
                    .expect("test harness coordinator finalization");
                cancellation.finish_canonical_commit();
            }
            result
        });
        tokio::time::sleep(Duration::from_millis(25)).await;
        assert!(
            !correction.is_finished(),
            "correction must wait behind the odometry state transaction"
        );
        release_current_tx
            .send(())
            .expect("release correction snapshot read");

        tokio::time::timeout(Duration::from_secs(2), async {
            assert!(odom
                .await
                .expect("odometry task")
                .expect("odometry handoff"));
            correction
                .await
                .expect("correction task")
                .expect("correction commit");
            current_blocker.await.expect("snapshot blocker");
        })
        .await
        .expect("serialized odometry/correction completion");

        let snapshot = applier.snapshot().expect("latest correction snapshot");
        assert_eq!(snapshot.correction_revision, 2);
        assert!(canonical_isometry_near(
            &snapshot.effective_map_from_odom,
            &new_effective
        ));
        let expected_map_from_body = new_effective * frozen_odom_from_body;
        let shared = handle.state.read().await;
        assert!((shared.pose.x - expected_map_from_body.translation.vector.x).abs() < 1.0e-9);
        assert!((shared.pose.y - expected_map_from_body.translation.vector.y).abs() < 1.0e-9);
        assert!((shared.pose.z - expected_map_from_body.translation.vector.z).abs() < 1.0e-9);
        assert_eq!(shared.pose.timestamp_ns, source_timestamp_ns);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn library_pgo_partial_scan_simulation_keeps_canonical_pose_with_map() {
        use crate::correction_coordinator::{
            CorrectionLeaseId, CorrectionOwnerRuntimeIdentity, CorrectionOwnerSpec,
            CorrectionTransportOwner, GeometryCorrectionSink,
        };
        use nuwax_mapping::loop_closure::LoopClosurePipelineConfig;

        // 准确里程计下改变视野：少量相似点块不能代表整帧回环。
        let patch: Vec<_> = (0..5)
            .flat_map(|x| {
                (0..4).flat_map(move |y| {
                    (0..3).map(move |z| {
                        Point3::new(
                            3.0 + x as f64 * 0.07 + (z * z) as f64 * 0.003,
                            2.0 + y as f64 * 0.09 + (x * z) as f64 * 0.002,
                            0.4 + z as f64 * 0.08 + (x * x) as f64 * 0.001,
                        )
                    })
                })
            })
            .collect();
        let mut historical = patch.clone();
        historical.extend((0..400).map(|i| {
            Point3::new(
                -4.0 + i as f64 * 0.0001,
                i as f64 * 0.02,
                0.1 + (i % 7) as f64 * 0.1 + i as f64 * 0.00003,
            )
        }));
        let mut current: Vec<_> = patch
            .iter()
            .map(|point| point + nalgebra::Vector3::new(0.6, 0.2, 0.1))
            .collect();
        current.extend((0..400).map(|i| {
            Point3::new(
                8.0 + i as f64 * 0.0001,
                -i as f64 * 0.02,
                0.1 + (i % 11) as f64 * 0.1 + i as f64 * 0.00003,
            )
        }));
        let mut runs = Vec::new();
        for enabled in [false, true] {
            let handle = build_handle_for_tests().await.unwrap();
            let mapping = handle.mapping_adapter.as_ref().unwrap();
            mapping
                .lock()
                .start_session("pgo-simulation", "pgo-simulation-map")
                .unwrap();
            let identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 7,
                backend_generation: 7,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: 42,
                    generation: 7,
                },
            };
            let tf = (
                LcmTopic::new("/test/pgo-simulation/tf").unwrap(),
                LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
            );
            let applier = Arc::new(
                GeometryCanonicalCorrectionApplier::new(
                    identity.clone(),
                    "map",
                    "odom",
                    "base_link",
                    handle.state.clone(),
                    handle.transport.clone(),
                    tf.clone(),
                )
                .unwrap(),
            );
            let spec = if enabled {
                CorrectionOwnerSpec::LibraryPgo {
                    map_frame_id: "map".into(),
                    odom_frame_id: "odom".into(),
                    body_frame_id: "base_link".into(),
                    pipeline: LoopClosurePipelineConfig::default(),
                }
            } else {
                CorrectionOwnerSpec::None
            };
            let sink = Arc::new(
                GeometryCorrectionSink::new_for_spec(
                    identity.clone(),
                    &spec,
                    "map",
                    "odom",
                    "base_link",
                    applier.clone(),
                )
                .unwrap(),
            );
            let owner = CorrectionTransportOwner::start(spec, identity, &test_lcm_url(), sink)
                .await
                .unwrap();
            let mut samples = Vec::new();
            for (sequence, seconds, x) in [
                (1, 1, 0.0),
                (2, 11, 2.0),
                (3, 61, 0.6),
                (4, 62, 4.0),
                (5, 63, 4.7),
            ] {
                let source_timestamp_ns = seconds * 1_000_000_000;
                let odom = nalgebra::Isometry3::translation(x, 0.0, 0.0);
                let pose = pose_state_from_isometry(&odom, "odom", source_timestamp_ns);
                handoff_lio_odometry_with_correction(&handle, &pose, "base_link", &tf, &applier)
                    .await
                    .unwrap();
                let (map_points, binding) = {
                    let adapter = mapping.lock();
                    let registered = adapter
                        .register_odom_cloud_to_map(
                            if sequence <= 2 { &historical } else { &current },
                            "odom",
                        )
                        .unwrap();
                    let snapshot = adapter.snapshot();
                    (
                        registered.points,
                        crate::lio_ingress::RegistrationBinding {
                            sensor_origin_map: Some([x, 0.0, 0.0]),
                            runtime_generation: 7,
                            cloud_source_sequence: sequence,
                            correlated_odometry: crate::lio_ingress::CorrelatedOdometry {
                                source_sequence: sequence,
                                source_timestamp_ns: source_timestamp_ns as i64,
                            },
                            mapping_session_id: snapshot.session_id,
                            mapping_session_generation: snapshot.session_generation,
                            map_frame_generation: adapter.global_frame_generation(),
                            envelope_sha256: crate::map_checksum::sha256_bytes(
                                &sequence.to_be_bytes(),
                            ),
                            frozen_odom_from_body: odom,
                            base_anchor_map_from_odom: nalgebra::Isometry3::identity(),
                            frame_capture_relocalization_epoch: Some(0),
                        },
                    )
                };
                assert!(handoff_registered_lio_cloud_with_correction(
                    &handle,
                    &map_points,
                    12,
                    sequence as u64 * 300_000_000,
                    source_timestamp_ns,
                    "map",
                    "base_link",
                    &binding,
                    &owner.ingress()
                )
                .await
                .unwrap());
                let shared = handle.state.read().await;
                let public = &shared.pose;
                let error = ((public.x - x).powi(2) + public.y.powi(2) + public.z.powi(2)).sqrt();
                let rotation_error = nalgebra::UnitQuaternion::from_quaternion(
                    nalgebra::Quaternion::new(public.qw, public.qx, public.qy, public.qz),
                )
                .angle();
                let cloud = handle
                    .cloud_map_writer
                    .lock()
                    .as_ref()
                    .unwrap()
                    .committed_snapshot()
                    .unwrap();
                println!("生产模拟 pgo={enabled} sequence={sequence} source={source_timestamp_ns} odom_x={x:.3} public=({:.3},{:.3},{:.3}) error_m={error:.6} error_deg={:.6} points={} correction={}",
                    public.x,public.y,public.z,rotation_error.to_degrees(),cloud.point_count,
                    applier.snapshot().map_or(0, |s| s.correction_revision));
                samples.push((error, rotation_error, cloud.xyz_f32_le.to_vec()));
            }
            owner.shutdown().await.unwrap();
            runs.push(samples);
        }
        for (index, (baseline, corrected)) in runs[0].iter().zip(&runs[1]).enumerate() {
            assert_eq!(
                baseline.2,
                corrected.2,
                "相同输入的规范点云应相同，帧{}",
                index + 1
            );
            assert!(
                corrected.0 < 1e-6 && corrected.1 < 1e-6,
                "点云正确但公共位姿被错误校正，帧{}：平移={}，旋转={}",
                index + 1,
                corrected.0,
                corrected.1
            );
        }
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn registered_lio_commit_receipts_form_exact_correction_frame_identity() {
        let handle = build_handle_for_tests()
            .await
            .expect("test Geometry handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-g004b-receipt-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() =
            Some(CloudMapShmWriter::new(&cloud_path).expect("real cloud-map receipt writer"));
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .clone();
        let (mapping_snapshot, map_frame_generation, base_anchor_map_from_odom) = {
            let mut adapter = mapping.lock();
            adapter
                .start_session("correction-receipt-session", "correction-receipt-map")
                .expect("active mapping session");
            adapter
                .feed_pose_in_frame(2.0, -1.0, 0.0, 0.35, "odom")
                .expect("immutable map-from-odom anchor");
            let anchor = adapter
                .map_odom_transform_snapshot()
                .expect("mapping anchor snapshot");
            (
                adapter.snapshot(),
                adapter.global_frame_generation(),
                nalgebra::Isometry3::from_parts(
                    nalgebra::Translation3::new(
                        anchor.translation[0],
                        anchor.translation[1],
                        anchor.translation[2],
                    ),
                    nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, anchor.yaw_radians),
                ),
            )
        };
        let frozen_odom_from_body = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(0.5, 0.25, -0.1),
            nalgebra::UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
        );
        let source_timestamp_ns = 8_765_432_100;
        let mut binding = crate::lio_ingress::RegistrationBinding {
            sensor_origin_map: None,
            runtime_generation: 41,
            cloud_source_sequence: 7,
            correlated_odometry: crate::lio_ingress::CorrelatedOdometry {
                source_sequence: 6,
                source_timestamp_ns: 8_765_432_000,
            },
            mapping_session_id: mapping_snapshot.session_id.clone(),
            mapping_session_generation: mapping_snapshot.session_generation,
            map_frame_generation,
            envelope_sha256: [0x5a; 32],
            frozen_odom_from_body,
            base_anchor_map_from_odom,
            frame_capture_relocalization_epoch: Some(0),
        };
        let sink = Arc::new(RecordingCorrectionFrameSink::default());
        let identity = crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
            runtime_generation: 41,
            backend_generation: 41,
            correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                boot_nonce: 0x1234,
                generation: 41,
            },
        };
        let owner = crate::correction_coordinator::CorrectionTransportOwner::start(
            crate::correction_coordinator::CorrectionOwnerSpec::None,
            identity,
            "unused-by-none",
            sink.clone(),
        )
        .await
        .expect("retained correction ingress");
        let correction = owner.ingress();
        let mut revision_rx = handle.map_revision_tx.subscribe();
        let map_points = vec![
            Point3::new(2.5, -0.5, 0.0),
            Point3::new(2.6, -0.4, 0.1),
            Point3::new(2.7, -0.3, 0.2),
        ];

        assert!(handoff_registered_lio_cloud_with_correction(
            &handle,
            &map_points,
            12,
            source_timestamp_ns + 1,
            source_timestamp_ns,
            "map",
            "base_link",
            &binding,
            &correction,
        )
        .await
        .expect("registered cloud commit and correction handoff"));
        let event = tokio::time::timeout(Duration::from_secs(1), revision_rx.recv())
            .await
            .expect("map revision event deadline")
            .expect("map revision event");
        let frames = sink.staged.lock().await;
        assert_eq!(frames.len(), 1);
        let frame = &frames[0];
        let occupancy_receipt = mapping
            .lock()
            .latest_occupancy_commit_receipt()
            .expect("occupancy receipt");
        let shared = handle.state.read().await;

        assert_eq!(frame.mapping_session_id, binding.mapping_session_id);
        assert_eq!(
            frame.mapping_session_generation,
            binding.mapping_session_generation
        );
        assert_eq!(frame.map_frame_generation, binding.map_frame_generation);
        assert_eq!(frame.mapping_commit_id, occupancy_receipt.commit_id);
        assert_eq!(frame.occupancy_revision, event.revision);
        assert_eq!(frame.cloud_revision, shared.cloud_map_revision);
        assert_eq!(frame.registered_cloud.source_sequence, 7);
        assert_eq!(
            frame.registered_cloud.source_timestamp_ns,
            source_timestamp_ns
        );
        assert_eq!(frame.registered_cloud.envelope_sha256, [0x5a; 32]);
        assert_eq!(frame.correlated_odom.proof.source_sequence, 6);
        assert_eq!(
            frame.correlated_odom.proof.source_timestamp_ns,
            8_765_432_000
        );
        assert_eq!(frame.correlated_odom.odom_from_body, frozen_odom_from_body);
        assert_eq!(
            frame.correlated_odom.proof.pose_sha256,
            crate::correction_coordinator::odom_pose_sha256(&frozen_odom_from_body)
                .expect("frozen odometry proof")
        );
        assert_eq!(frame.committed_map.source_timestamp_ns, source_timestamp_ns);
        assert_eq!(
            frame.committed_map.payload_sha256,
            crate::map_checksum::sha256_bytes(event.data.as_ref())
        );
        assert_eq!(shared.map.revision, frame.occupancy_revision);
        assert_eq!(shared.map.timestamp_ns, source_timestamp_ns);
        let expected_body_points = map_points
            .iter()
            .map(|point| {
                (base_anchor_map_from_odom * frozen_odom_from_body)
                    .inverse()
                    .transform_point(point)
            })
            .collect::<Vec<_>>();
        assert_eq!(frame.body_points, expected_body_points);
        assert_eq!(
            frame.registered_cloud.body_points_sha256,
            crate::correction_coordinator::body_points_sha256(&expected_body_points)
                .expect("body points proof")
        );
        drop(shared);
        drop(frames);

        // 高频样本已经进入后端，但未到栅格发布周期；不能误记拒绝或制造校正收据。
        let before_revision = handle.state.read().await.map.revision;
        let before_cloud_revision = handle.state.read().await.cloud_map_revision;
        let mut throttled_binding = binding.clone();
        throttled_binding.cloud_source_sequence += 1;
        assert!(handoff_registered_lio_cloud_with_correction(
            &handle,
            &map_points,
            12,
            source_timestamp_ns + 2,
            source_timestamp_ns + 1,
            "map",
            "base_link",
            &throttled_binding,
            &correction,
        )
        .await
        .expect("live cloud is accepted during publication cadence"));
        assert!(revision_rx.try_recv().is_err());
        assert_eq!(sink.staged.lock().await.len(), 1);
        assert_eq!(handle.state.read().await.map.revision, before_revision);
        assert_eq!(handle.state.read().await.cloud_map_revision, before_cloud_revision);

        // A read-only relocalization payload may only be republished while
        // its exact committed receipt is still current.  Advancing the
        // observable cloud revision makes the frozen publication fail closed.
        binding.frame_capture_relocalization_epoch = Some(1);
        let read_only = {
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            registered_lio_read_only_correction_frame(
                &handle,
                &map_points,
                source_timestamp_ns + 1,
                "map",
                "base_link",
                &binding,
                1,
            )
            .await
            .expect("read-only relocalization frame")
        };
        {
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            ensure_read_only_mapping_identity(&handle, &binding, &read_only.cloud)
                .await
                .expect("fresh read-only receipt identity");
        }
        let next_cloud_revision = handle
            .state
            .read()
            .await
            .cloud_map_revision
            .checked_add(1)
            .expect("test cloud revision increment");
        handle.state.write().await.cloud_map_revision = next_cloud_revision;
        {
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            assert!(
                ensure_read_only_mapping_identity(&handle, &binding, &read_only.cloud)
                    .await
                    .is_err(),
                "stale read-only global-map payload must never publish"
            );
        }
        owner.shutdown().await.expect("correction owner shutdown");
        drop(handle);
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn frame_captured_before_relocalization_admission_never_reaches_map_or_correction() {
        let handle = build_handle_for_tests()
            .await
            .expect("test Geometry handle");
        let sink = Arc::new(RecordingCorrectionFrameSink::default());
        let owner = crate::correction_coordinator::CorrectionTransportOwner::start(
            crate::correction_coordinator::CorrectionOwnerSpec::None,
            crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
                runtime_generation: 77,
                backend_generation: 77,
                correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                    boot_nonce: 0x7755,
                    generation: 77,
                },
            },
            "unused-by-none",
            sink.clone(),
        )
        .await
        .expect("correction owner starts");
        let correction = owner.ingress();
        let captured = correction.relocalization_mode_snapshot();
        assert!(!captured.active);
        let binding = crate::lio_ingress::RegistrationBinding {
            sensor_origin_map: None,
            runtime_generation: 77,
            cloud_source_sequence: 9,
            correlated_odometry: crate::lio_ingress::CorrelatedOdometry {
                source_sequence: 8,
                source_timestamp_ns: 800,
            },
            mapping_session_id: "captured-before-admission".to_owned(),
            mapping_session_generation: 1,
            map_frame_generation: 1,
            envelope_sha256: [0x77; 32],
            frozen_odom_from_body: nalgebra::Isometry3::identity(),
            base_anchor_map_from_odom: nalgebra::Isometry3::identity(),
            frame_capture_relocalization_epoch: Some(captured.epoch),
        };
        let before = {
            let state = handle.state.read().await;
            (
                state.map.revision,
                state.map.timestamp_ns,
                state.map.data.clone(),
                state.cloud_map_revision,
            )
        };
        let mut revision_rx = handle.map_revision_tx.subscribe();

        // This explicit transition is the race gate: the registration was
        // admitted under `captured`, then relocalization claims the lifecycle
        // before its handoff begins.
        let activated = correction.relocalization_mode_fence().activate_for_tests();
        assert!(activated.active);
        assert_ne!(activated.epoch, captured.epoch);

        assert!(!handoff_registered_lio_cloud_with_correction(
            &handle,
            &[Point3::new(1.0, 2.0, 3.0)],
            12,
            901,
            900,
            "map",
            "base_link",
            &binding,
            &correction,
        )
        .await
        .expect("stale frame is rejected without a handoff error"));
        assert!(sink.staged.lock().await.is_empty());
        let after = {
            let state = handle.state.read().await;
            (
                state.map.revision,
                state.map.timestamp_ns,
                state.map.data.clone(),
                state.cloud_map_revision,
            )
        };
        assert_eq!(after, before);
        assert!(matches!(
            revision_rx.try_recv(),
            Err(tokio::sync::broadcast::error::TryRecvError::Empty)
        ));
        owner.shutdown().await.expect("correction owner shutdown");
    }

    #[test]
    fn canonical_tf_uses_cached_anchor_while_mapping_lock_is_busy() {
        let mapping = shared_active_mapping_adapter();
        let transform_cache = {
            let mut adapter = mapping.lock();
            adapter
                .feed_pose_in_frame(2.0, 3.0, 0.0, std::f64::consts::FRAC_PI_2, "odom")
                .expect("anchor");
            adapter.map_odom_transform_cache()
        };
        let pose = PoseState {
            x: 2.0,
            y: 4.0,
            z: 0.0,
            qz: std::f64::consts::FRAC_PI_4.sin(),
            qw: std::f64::consts::FRAC_PI_4.cos(),
            frame_id: "odom".to_string(),
            timestamp_ns: 12_345_678_902,
            ..PoseState::default()
        };

        let adapter_guard = mapping.lock();
        let observation = canonical_mapping_observation_for_odom(
            mapping.as_ref(),
            transform_cache.as_ref(),
            &pose,
            "base_link",
        )
        .expect("cached canonical observation");

        assert_eq!(adapter_guard.odom_count(), 1);
        assert!((observation.map_pose.x - 1.0).abs() < 1e-9);
        assert!(observation.map_pose.y.abs() < 1e-9);
        assert_eq!(observation.map_pose.timestamp_ns, pose.timestamp_ns);
        assert!(observation
            .transforms
            .iter()
            .all(|entry| entry.stamp_ns == pose.timestamp_ns));
    }

    #[tokio::test]
    #[allow(clippy::await_holding_lock)]
    async fn active_lidar_adapter_contention_publishes_cached_odom_promptly() {
        let mapping = shared_active_mapping_adapter();
        mapping
            .lock()
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.25, "odom")
            .expect("anchor");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let odom_topic = LcmTopic::new("/test/mapping/cached-odom").expect("odom topic");
        let odom_type = LcmMessageType::new("nav_msgs.Odometry").expect("odom type");
        let tf_topic = LcmTopic::new("/test/mapping/cached-tf").expect("TF topic");
        let tf_type = LcmMessageType::new("tf2_msgs.TFMessage").expect("TF type");
        let backend = MappingFeedRecordingBackend::new(odom_topic.clone());
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let (map_revision_tx, _) = broadcast::channel(4);
        let (cloud_map_tx, _) = broadcast::channel(4);

        spawn_mapping_feed_tasks(
            transport,
            state.clone(),
            mapping.clone(),
            map_revision_tx,
            watch::channel(None).0,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            Some((odom_topic.clone(), odom_type.clone())),
            pointcloud_binding(),
            (tf_topic, tf_type),
            "base_link".into(),
            Arc::new(parking_lot::Mutex::new(None)),
            cloud_map_tx,
            lifecycle_lock,
        );
        tokio::time::timeout(Duration::from_secs(1), backend.odom_subscribed.notified())
            .await
            .expect("odom subscriber startup");

        // This is the same exclusive adapter guard held by an in-flight LiDAR ingest.
        let adapter_guard = mapping.lock();
        assert!(adapter_guard.is_active());
        let pose = PoseState {
            x: 2.0,
            y: 3.0,
            qz: (0.25_f64 * 0.5).sin(),
            qw: (0.25_f64 * 0.5).cos(),
            frame_id: "odom".into(),
            timestamp_ns: 12_345_678_904,
            ..PoseState::default()
        };
        backend
            .odom_sender
            .send(Ok(crate::typed_codec::build_typed_frame(
                odom_topic,
                odom_type,
                crate::typed_codec::encode_odometry_payload(&pose, "base_link")
                    .expect("encode odometry"),
                pose.timestamp_ns,
            )))
            .expect("inject odometry");

        tokio::time::timeout(
            Duration::from_millis(250),
            backend.frame_published.notified(),
        )
        .await
        .expect("cached odometry TF publication must not wait for LiDAR adapter release");
        assert_eq!(backend.published_count(), 1);
        let shared = state.read().await;
        assert_eq!(shared.pose.timestamp_ns, pose.timestamp_ns);
        assert_eq!(shared.latest_transform_entries().len(), 2);
        drop(shared);
        drop(adapter_guard);
    }

    #[tokio::test]
    async fn blocked_tf_publish_times_out_and_releases_lifecycle_fence() {
        let mapping = shared_active_mapping_adapter();
        mapping
            .lock()
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.25, "odom")
            .expect("anchor");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let odom_topic = LcmTopic::new("/test/mapping/blocked-odom").expect("odom topic");
        let odom_type = LcmMessageType::new("nav_msgs.Odometry").expect("odom type");
        let tf_topic = LcmTopic::new("/test/mapping/blocked-tf").expect("TF topic");
        let tf_type = LcmMessageType::new("tf2_msgs.TFMessage").expect("TF type");
        let backend = MappingFeedRecordingBackend::new(odom_topic.clone());
        backend.block_publication();
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let (map_revision_tx, _) = broadcast::channel(4);
        let (cloud_map_tx, _) = broadcast::channel(4);

        spawn_mapping_feed_tasks(
            transport,
            state,
            mapping,
            map_revision_tx,
            watch::channel(None).0,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            Some((odom_topic.clone(), odom_type.clone())),
            pointcloud_binding(),
            (tf_topic, tf_type),
            "base_link".into(),
            Arc::new(parking_lot::Mutex::new(None)),
            cloud_map_tx,
            lifecycle_lock.clone(),
        );
        tokio::time::timeout(Duration::from_secs(1), backend.odom_subscribed.notified())
            .await
            .expect("odom subscriber startup");
        let pose = PoseState {
            x: 2.0,
            y: 3.0,
            qz: (0.25_f64 * 0.5).sin(),
            qw: (0.25_f64 * 0.5).cos(),
            frame_id: "odom".into(),
            timestamp_ns: 12_345_678_905,
            ..PoseState::default()
        };
        backend
            .odom_sender
            .send(Ok(crate::typed_codec::build_typed_frame(
                odom_topic,
                odom_type,
                crate::typed_codec::encode_odometry_payload(&pose, "base_link")
                    .expect("encode odometry"),
                pose.timestamp_ns,
            )))
            .expect("inject odometry");
        tokio::time::timeout(Duration::from_secs(1), backend.publish_started.notified())
            .await
            .expect("blocked TF publication started");
        assert!(lifecycle_lock.try_lock().is_err());

        let lifecycle_guard = tokio::time::timeout(
            MAP_FANOUT_SINK_TIMEOUT + Duration::from_secs(1),
            lifecycle_lock.lock(),
        )
        .await
        .expect("lifecycle fence must release after TF publish timeout");
        assert_eq!(backend.published_count(), 0);
        drop(lifecycle_guard);
    }

    #[tokio::test]
    async fn canonical_pose_remains_readable_while_tf_publication_is_blocked() {
        let mut handle = build_handle_for_tests().await.unwrap();
        handle.mapping_adapter.as_ref().unwrap().lock()
            .start_session("unblocked-pose-session", "warehouse").unwrap();
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/test/mapping/pose-readable-odom").unwrap(),
        );
        backend.block_publication();
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        let state = handle.state.clone();
        let lifecycle = handle.mapping_lifecycle_lock.clone();
        let mut changes = state.read().await.pose_changed.subscribe();
        let publication = tokio::spawn(async move {
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            handoff_lio_odometry(
                &handle,
                &PoseState {
                    x: 2.0,
                    qw: 1.0,
                    frame_id: "odom".into(),
                    timestamp_ns: 123_456_789,
                    ..PoseState::default()
                },
                "base_link",
                &(
                    LcmTopic::new("/test/mapping/pose-readable-tf").unwrap(),
                    LcmMessageType::new("tf2_msgs.TFMessage").unwrap(),
                ),
            ).await
        });
        tokio::time::timeout(Duration::from_secs(1), backend.publish_started.notified())
            .await.expect("坐标变换发布进入阻塞器");
        tokio::time::timeout(Duration::from_millis(50), changes.changed())
            .await.expect("规范提交应先通知位姿消费者").unwrap();
        let shared = tokio::time::timeout(Duration::from_millis(50), state.read())
            .await.expect("远程发布不能持有共享数据锁");
        assert_eq!(shared.pose.timestamp_ns, 123_456_789);
        assert_eq!(shared.latest_transform_entries().len(), 2);
        drop(shared);
        assert!(lifecycle.try_lock().is_err(), "原建图生命周期围栏保持有效");
        let error = tokio::time::timeout(MAP_FANOUT_SINK_TIMEOUT + Duration::from_secs(1), publication)
            .await.unwrap().unwrap().expect_err("原发布超时必须仍返回失败");
        assert!(error.contains("timed out"));
        assert!(lifecycle.try_lock().is_ok());
    }

    #[tokio::test]
    #[allow(clippy::await_holding_lock)]
    async fn stopped_session_cannot_publish_cached_odom_during_lock_contention() {
        let mapping = shared_active_mapping_adapter();
        mapping
            .lock()
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.25, "odom")
            .expect("anchor");
        let state = Arc::new(RwLock::new(SharedState::default()));
        let lifecycle_lock = Arc::new(tokio::sync::Mutex::new(()));
        let odom_topic = LcmTopic::new("/test/mapping/odom").expect("odom topic");
        let odom_type = LcmMessageType::new("nav_msgs.Odometry").expect("odom type");
        let tf_topic = LcmTopic::new("/test/mapping/tf").expect("TF topic");
        let tf_type = LcmMessageType::new("tf2_msgs.TFMessage").expect("TF type");
        let backend = MappingFeedRecordingBackend::new(odom_topic.clone());
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let (map_revision_tx, _) = broadcast::channel(4);
        let (cloud_map_tx, _) = broadcast::channel(4);

        spawn_mapping_feed_tasks(
            transport,
            state.clone(),
            mapping.clone(),
            map_revision_tx,
            watch::channel(None).0,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            Some((odom_topic.clone(), odom_type.clone())),
            pointcloud_binding(),
            (tf_topic, tf_type),
            "base_link".into(),
            Arc::new(parking_lot::Mutex::new(None)),
            cloud_map_tx,
            lifecycle_lock.clone(),
        );
        tokio::time::timeout(Duration::from_secs(1), backend.odom_subscribed.notified())
            .await
            .expect("odom subscriber startup");

        let lifecycle_guard = lifecycle_lock.lock().await;
        let mut adapter_guard = mapping.lock();
        adapter_guard
            .stop_session("test-session")
            .expect("stop active mapping session");
        let pose = PoseState {
            x: 2.0,
            y: 3.0,
            qz: (0.25_f64 * 0.5).sin(),
            qw: (0.25_f64 * 0.5).cos(),
            frame_id: "odom".into(),
            timestamp_ns: 12_345_678_903,
            ..PoseState::default()
        };
        backend
            .odom_sender
            .send(Ok(crate::typed_codec::build_typed_frame(
                odom_topic,
                odom_type,
                crate::typed_codec::encode_odometry_payload(&pose, "base_link")
                    .expect("encode odometry"),
                pose.timestamp_ns,
            )))
            .expect("inject odometry");

        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(backend.published_count(), 0);
        assert_eq!(state.read().await.pose.timestamp_ns, 0);

        drop(adapter_guard);
        drop(lifecycle_guard);
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(backend.published_count(), 0);
        let shared = state.read().await;
        assert_eq!(shared.pose.timestamp_ns, 0);
        assert!(shared.latest_transform_entries().is_empty());
    }

    #[tokio::test]
    async fn canonical_tf_is_observable_over_real_lcm() {
        use futures::StreamExt;

        let mut adapter = active_mapping_adapter();
        adapter
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.25, "odom")
            .expect("anchor");
        let pose = PoseState {
            x: 1.5,
            y: 2.5,
            z: 0.0,
            qz: (0.25_f64 * 0.5).sin(),
            qw: (0.25_f64 * 0.5).cos(),
            frame_id: "odom".to_string(),
            timestamp_ns: 8_000_000_009,
            ..PoseState::default()
        };
        adapter
            .feed_pose_in_frame(pose.x, pose.y, pose.z, 0.25, "odom")
            .expect("latest pose");
        let observation = canonical_mapping_observation(
            &adapter
                .map_odom_transform_snapshot()
                .expect("anchor snapshot"),
            &pose,
            "base_link",
        )
        .expect("canonical TF");
        let state = Arc::new(RwLock::new(SharedState::default()));
        apply_canonical_mapping_observation(&state, &observation).await;
        {
            let state = state.read().await;
            assert_eq!(state.pose.frame_id, "map");
            assert_eq!(state.pose.timestamp_ns, pose.timestamp_ns);
            assert_eq!(state.latest_transform_entries().len(), 2);
        }
        let transport = LcmTransport::new_real(&test_lcm_url()).expect("real LCM transport");
        let topic =
            LcmTopic::new(&format!("/test/geometry/tf/{}", std::process::id())).expect("TF topic");
        let message_type = LcmMessageType::new("tf2_msgs.TFMessage").expect("TF message type");
        let mut stream = transport
            .subscribe(topic.clone(), Some(message_type.clone()))
            .await;
        transport
            .publish(crate::typed_codec::build_typed_frame(
                topic,
                message_type,
                observation.tf_payload,
                pose.timestamp_ns,
            ))
            .await
            .expect("publish canonical TF");
        let frame = tokio::time::timeout(Duration::from_secs(2), stream.next())
            .await
            .expect("real LCM TF timeout")
            .expect("TF stream open")
            .expect("TF frame");
        let decoded = parse_tf_payload(&frame.payload).expect("decode TF body");
        assert_eq!(decoded.len(), 2);
        assert!(decoded
            .iter()
            .all(|entry| entry.stamp_ns == pose.timestamp_ns));
    }

    #[test]
    fn map_revision_clone_shares_large_payload_allocation() {
        let event = map_revision_event(MapState {
            width: 2 * 1024 * 1024,
            height: 1,
            frame_id: "map".to_string(),
            data: vec![42; 2 * 1024 * 1024],
            ..MapState::default()
        });
        let clone = event.clone();
        assert!(Arc::ptr_eq(&event.data, &clone.data));
        assert_eq!(clone.data.len(), 2 * 1024 * 1024);
    }

    #[tokio::test]
    async fn map_revision_watch_coalesces_bursts_to_latest_revision() {
        let (ingress_tx, ingress_rx) = broadcast::channel(8);
        let (latest_tx, mut latest_rx) = watch::channel(None);
        let mapping_adapter = shared_active_mapping_adapter();
        spawn_map_revision_coalescer(
            ingress_rx,
            latest_tx,
            Arc::new(std::sync::Mutex::new(None)),
            mapping_adapter,
            Arc::new(tokio::sync::Mutex::new(())),
        );
        for revision in 1..=64 {
            ingress_tx
                .send(map_revision_event(MapState {
                    width: 1,
                    height: 1,
                    frame_id: "map".to_string(),
                    revision,
                    data: vec![revision as i8],
                    ..MapState::default()
                }))
                .expect("coalescer receiver");
        }

        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                latest_rx.changed().await.expect("coalescer open");
                let latest = latest_rx.borrow_and_update().clone();
                if latest.as_ref().map(|event| event.revision) == Some(64) {
                    break;
                }
            }
        })
        .await
        .expect("latest revision must be observable");
        assert_eq!(
            latest_rx.borrow().as_ref().map(|event| event.revision),
            Some(64)
        );
    }

    #[tokio::test]
    async fn external_occupancy_lcm_cannot_advance_native_mapping_revision() {
        let handle = build_handle_for_tests().await.expect("test handle");
        handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .start_session("external-test", "warehouse")
            .expect("active mapping session");
        let mut map_revision_rx = handle.map_revision_tx.subscribe();
        let event = map_revision_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "map".into(),
            revision: 1,
            timestamp_ns: 123_000_000,
            data: vec![0, 100],
        });
        let topic = LcmTopic::new(&format!(
            "/test/geometry/occupancy/helper/{}",
            std::process::id()
        ))
        .expect("test occupancy topic");
        let message_type =
            LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy message type");
        handle
            .transport
            .publish(crate::typed_codec::build_typed_frame(
                topic,
                message_type,
                encode_occupancy_revision(&event).expect("encode external occupancy"),
                1,
            ))
            .await
            .expect("publish external occupancy");
        tokio::time::sleep(Duration::from_millis(100)).await;

        assert_eq!(handle.state.read().await.map.revision, 0);
        assert_eq!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .current_revision(),
            0
        );
        assert!(map_revision_rx.try_recv().is_err());
    }

    #[test]
    fn map_revision_fanout_uses_canonical_occupancy_and_exploration_cells() {
        let event = map_revision_event(MapState {
            width: 4,
            height: 1,
            resolution: 0.25,
            origin_x: -1.0,
            origin_y: 2.0,
            frame_id: "map".into(),
            revision: 9,
            timestamp_ns: 5_000_000_006,
            data: vec![-1, 0, 42, 100],
        });

        let payload = encode_occupancy_revision(&event).expect("canonical occupancy payload");
        let decoded = schema::decode_occupancy_grid_body(&payload).expect("decode occupancy");
        assert_eq!(decoded.header.frame_id, "map");
        assert_eq!(decoded.info.width, 4);
        assert_eq!(decoded.info.height, 1);
        assert_eq!(decoded.data, vec![-1, 0, 42, 100]);

        let costmap =
            exploration_costmap_from_revision(&event).expect("map-frame exploration snapshot");
        assert_eq!(
            costmap.grid.cells[0],
            nuwax_nav_types_messages::grid::OccupancyCell::Unknown
        );
        assert_eq!(
            costmap.grid.cells[1],
            nuwax_nav_types_messages::grid::OccupancyCell::Free
        );
        assert_eq!(
            costmap.grid.cells[2],
            nuwax_nav_types_messages::grid::OccupancyCell::Cost(42)
        );
        assert_eq!(
            costmap.grid.cells[3],
            nuwax_nav_types_messages::grid::OccupancyCell::Occupied
        );
    }

    #[tokio::test]
    async fn configured_map_frame_is_preserved_by_occupancy_costmap_and_pose_adapters() {
        const MAP_FRAME: &str = "robot/map";
        let event = map_revision_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.25,
            origin_x: -1.0,
            origin_y: 2.0,
            frame_id: MAP_FRAME.into(),
            revision: 9,
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            data: vec![0, 100],
        });

        let payload = encode_occupancy_revision_in_frame(&event, MAP_FRAME)
            .expect("configured-frame occupancy payload");
        let decoded = schema::decode_occupancy_grid_body(&payload).expect("decode occupancy");
        assert_eq!(decoded.header.frame_id, MAP_FRAME);
        assert!(encode_occupancy_revision(&event).is_err());

        let costmap = exploration_costmap_from_revision_in_frame(&event, MAP_FRAME)
            .expect("configured-frame exploration snapshot");
        assert_eq!(costmap.frame_id, MAP_FRAME);
        assert!(exploration_costmap_from_revision(&event).is_err());

        let state = Arc::new(RwLock::new(SharedState {
            pose: PoseState {
                x: 1.0,
                y: 2.0,
                qw: 1.0,
                frame_id: MAP_FRAME.to_string(),
                timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                ..PoseState::default()
            },
            ..SharedState::default()
        }));
        let provider = SharedStatePoseProvider {
            state,
            expected_map_frame: Arc::from(MAP_FRAME),
            freshness: Arc::new(parking_lot::Mutex::new(PoseFreshnessTracker::default())),
            max_receipt_age: Duration::from_secs(1),
        };
        let pose = crate::explore_runtime::ExplorePoseProvider::current_pose(&provider)
            .await
            .expect("configured-frame pose");
        assert_eq!(pose.frame_id, MAP_FRAME);
    }

    #[test]
    fn non_map_revision_is_rejected_for_global_consumers() {
        let event = map_revision_event(MapState {
            width: 1,
            height: 1,
            resolution: 0.25,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "odom".into(),
            revision: 1,
            timestamp_ns: 1,
            data: vec![0],
        });
        assert!(encode_occupancy_revision(&event).is_err());
        assert!(exploration_costmap_from_revision(&event).is_err());
    }

    #[test]
    fn occupancy_revision_wire_domain_is_positive_i32_without_truncation() {
        let event = |revision| {
            map_revision_event(MapState {
                width: 1,
                height: 1,
                resolution: 0.25,
                frame_id: "map".into(),
                revision,
                timestamp_ns: 1,
                data: vec![0],
                ..MapState::default()
            })
        };

        assert!(encode_occupancy_revision(&event(0)).is_err());
        let max_payload =
            encode_occupancy_revision(&event(i32::MAX as u64)).expect("final positive i32");
        let decoded =
            schema::decode_occupancy_grid_body(&max_payload).expect("decode max revision");
        assert_eq!(decoded.header.seq, i32::MAX);
        assert!(encode_occupancy_revision(&event(i32::MAX as u64 + 1)).is_err());
        assert!(encode_occupancy_revision(&event(u64::MAX)).is_err());
    }

    #[tokio::test]
    async fn map_revision_fanout_is_observable_over_real_lcm() {
        use futures::StreamExt;

        let transport = LcmTransport::new_real(&test_lcm_url()).expect("real LCM transport");
        let topic_name = format!("/test/geometry/occupancy/{}", std::process::id());
        let topic = LcmTopic::new(&topic_name).expect("test topic");
        let message_type =
            LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy message type");
        let mut stream = transport
            .subscribe(topic.clone(), Some(message_type.clone()))
            .await;
        let (tx, rx) = broadcast::channel(4);
        let (latest_tx, _) = watch::channel(None);
        let mapping_adapter = shared_active_mapping_adapter();
        spawn_map_revision_fanout(
            rx,
            latest_tx,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            transport,
            topic,
            message_type,
            None,
            None,
            Arc::new(RwLock::new(None)),
            crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
            mapping_adapter,
            Arc::new(tokio::sync::Mutex::new(())),
        );
        let event = map_revision_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "map".into(),
            revision: 1,
            timestamp_ns: 9_000_000_010,
            data: vec![0, 100],
        });
        tx.send(event).expect("fanout receiver registered");

        let frame = tokio::time::timeout(Duration::from_secs(2), stream.next())
            .await
            .expect("real LCM receive timeout")
            .expect("real LCM stream closed")
            .expect("real LCM frame");
        let decoded = schema::decode_occupancy_grid_body(&frame.payload)
            .expect("decode real LCM occupancy body");
        assert_eq!(decoded.header.frame_id, "map");
        assert_eq!(decoded.data, vec![0, 100]);
    }

    #[tokio::test]
    async fn wrong_frame_revision_cannot_advance_configured_latest_or_occupancy_shm() {
        const MAP_FRAME: &str = "robot/map";
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), MAP_FRAME, "odom");
        adapter
            .start_session("configured-session", "configured-map")
            .expect("active configured mapping session");
        let session = adapter.snapshot();
        let mapping_adapter = Arc::new(parking_lot::Mutex::new(adapter));

        static SHM_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let shm_path = std::env::temp_dir().join(format!(
            "nuwax-wrong-frame-occupancy-{}-{}.mmap",
            std::process::id(),
            SHM_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
        ));
        let writer = Arc::new(parking_lot::Mutex::new(
            OccupancyGridShmWriter::new(&shm_path).expect("temporary occupancy SHM"),
        ));
        assert_eq!(writer.lock().published_generation(), 0);
        let current = Arc::new(RwLock::new(None));
        let (tx, rx) = broadcast::channel(4);
        let (latest_tx, mut latest_rx) = watch::channel(None);
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/test/configured-frame/unused-odom").expect("unused odom topic"),
        );

        spawn_map_revision_fanout_in_frame(
            rx,
            latest_tx,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            LcmTransport::new(Arc::new(backend.clone())),
            LcmTopic::new("/test/configured-frame/occupancy").expect("occupancy topic"),
            LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy type"),
            Some(writer.clone()),
            Some(shm_path.to_string_lossy().into_owned()),
            current.clone(),
            crate::explore_runtime::ExploreRuntimeHandle::with_wavefront_map_frame(MAP_FRAME)
                .expect("configured explore runtime"),
            mapping_adapter,
            Arc::new(tokio::sync::Mutex::new(())),
            Arc::from(MAP_FRAME),
        );

        tx.send(MapRevisionEvent::from_map(
            &MapState {
                width: 1,
                height: 1,
                resolution: 0.25,
                frame_id: "map".to_string(),
                revision: 1,
                timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                data: vec![0],
                ..MapState::default()
            },
            &session,
        ))
        .expect("configured fanout receiver registered");

        assert!(
            tokio::time::timeout(Duration::from_millis(150), latest_rx.changed())
                .await
                .is_err(),
            "wrong-frame revision advanced the retained latest event"
        );
        assert!(latest_rx.borrow().is_none());
        assert!(current.read().await.is_none());
        assert_eq!(writer.lock().published_generation(), 0);
        assert_eq!(backend.published_count(), 0);

        drop(tx);
        let _ = std::fs::remove_file(shm_path);
    }

    #[tokio::test]
    async fn live_watch_visibility_follows_descriptor_and_backing_commit() {
        let shm_path = occupancy_test_path("commit-before-watch");
        let writer = Arc::new(parking_lot::Mutex::new(
            OccupancyGridShmWriter::new(&shm_path).expect("temporary occupancy SHM"),
        ));
        let mut reader = OccupancyGridShmReader::open(&shm_path).expect("separate SHM reader");
        let current = Arc::new(RwLock::new(None));
        let mapping_adapter = shared_active_mapping_adapter();
        let (tx, rx) = broadcast::channel(4);
        let (latest_tx, mut latest_rx) = watch::channel(None);
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/test/commit-before-watch/unused-odom").expect("unused odom topic"),
        );
        spawn_map_revision_fanout_in_frame(
            rx,
            latest_tx,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            LcmTransport::new(Arc::new(backend)),
            LcmTopic::new("/test/commit-before-watch/occupancy").expect("occupancy topic"),
            LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy type"),
            Some(writer.clone()),
            Some(shm_path.to_string_lossy().into_owned()),
            current.clone(),
            crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
            mapping_adapter,
            Arc::new(tokio::sync::Mutex::new(())),
            Arc::from("map"),
        );
        let event = map_revision_event(MapState {
            width: 2,
            height: 2,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 1,
            data: vec![0, 100, -1, 42],
            ..MapState::default()
        });
        tx.send(event.clone()).expect("fanout receiver");

        tokio::time::timeout(Duration::from_secs(1), latest_rx.changed())
            .await
            .expect("watch visibility timeout")
            .expect("watch remains open");
        assert_eq!(
            latest_rx.borrow().as_ref().map(|event| event.revision),
            Some(event.revision)
        );
        let descriptor = current
            .read()
            .await
            .clone()
            .expect("descriptor precedes watch visibility");
        assert_eq!(descriptor.revision, event.revision);
        let backing = reader
            .read_stable()
            .expect("stable SHM read")
            .expect("backing precedes watch visibility");
        assert_eq!(backing.revision, event.revision);
        assert_eq!(backing.generation, descriptor.generation);
        assert_eq!(backing.active_slot, descriptor.active_slot);
        assert_eq!(backing.data, vec![0, 100, -1, 42]);
        let _ = std::fs::remove_file(shm_path);
    }

    #[tokio::test]
    async fn failed_shm_prepare_withholds_descriptor_watch_and_typed_fanout() {
        let shm_path = occupancy_test_path("failed-prepare");
        let writer = Arc::new(parking_lot::Mutex::new(
            OccupancyGridShmWriter::new(&shm_path).expect("temporary occupancy SHM"),
        ));
        let mut reader = OccupancyGridShmReader::open(&shm_path).expect("separate SHM reader");
        let current = Arc::new(RwLock::new(None));
        let mapping_adapter = shared_active_mapping_adapter();
        let (tx, rx) = broadcast::channel(4);
        let (latest_tx, mut latest_rx) = watch::channel(None);
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/test/failed-prepare/unused-odom").expect("unused odom topic"),
        );
        spawn_map_revision_fanout_in_frame(
            rx,
            latest_tx,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            LcmTransport::new(Arc::new(backend.clone())),
            LcmTopic::new("/test/failed-prepare/occupancy").expect("occupancy topic"),
            LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy type"),
            Some(writer.clone()),
            Some(shm_path.to_string_lossy().into_owned()),
            current.clone(),
            crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
            mapping_adapter,
            Arc::new(tokio::sync::Mutex::new(())),
            Arc::from("map"),
        );
        let malformed = map_revision_event(MapState {
            width: 2,
            height: 2,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 1,
            data: vec![0],
            ..MapState::default()
        });
        tx.send(malformed).expect("fanout receiver");

        assert!(
            tokio::time::timeout(Duration::from_millis(150), latest_rx.changed())
                .await
                .is_err(),
            "failed SHM prepare must withhold watch visibility"
        );
        assert!(current.read().await.is_none());
        assert!(reader.read_stable().expect("stable SHM read").is_none());
        assert_eq!(writer.lock().published_generation(), 0);
        assert_eq!(backend.published_count(), 0);
        let _ = std::fs::remove_file(shm_path);
    }

    #[test]
    fn stop_between_prepare_and_commit_aborts_without_backing_mutation() {
        let shm_path = occupancy_test_path("stop-before-commit");
        let mut writer = OccupancyGridShmWriter::new(&shm_path).expect("temporary occupancy SHM");
        let mut reader = OccupancyGridShmReader::open(&shm_path).expect("separate SHM reader");
        let mapping_adapter = shared_active_mapping_adapter();
        let event = map_revision_event(MapState {
            width: 1,
            height: 1,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 1,
            data: vec![100],
            ..MapState::default()
        });
        assert!(map_event_matches_current_session(&event, &mapping_adapter));
        writer
            .begin_session(event.session_generation)
            .expect("session fence");
        let prepared = writer
            .prepare_occupancy(&event.to_map_state())
            .expect("inactive-slot prepare");
        assert!(reader.read_stable().expect("staged read").is_none());

        mapping_adapter
            .lock()
            .stop_session("test-session")
            .expect("stop mapping before commit");
        assert!(!map_event_matches_current_session(&event, &mapping_adapter));
        assert!(writer.abort_prepared(prepared));
        assert!(reader.read_stable().expect("aborted read").is_none());
        assert_eq!(writer.published_generation(), 0);
        assert_eq!(writer.physical_write_count(), 0);
        let _ = std::fs::remove_file(shm_path);
    }

    #[tokio::test]
    async fn stopped_session_event_cannot_mutate_backing_cache_or_watch() {
        let shm_path = occupancy_test_path("stopped-event");
        let writer = Arc::new(parking_lot::Mutex::new(
            OccupancyGridShmWriter::new(&shm_path).expect("temporary occupancy SHM"),
        ));
        let mut reader = OccupancyGridShmReader::open(&shm_path).expect("separate SHM reader");
        let mapping_adapter = shared_active_mapping_adapter();
        let stopped_snapshot = {
            let mut adapter = mapping_adapter.lock();
            adapter
                .stop_session("test-session")
                .expect("stop mapping before event")
        };
        let event = MapRevisionEvent::from_map(
            &MapState {
                width: 1,
                height: 1,
                resolution: 0.1,
                frame_id: "map".to_string(),
                revision: 1,
                timestamp_ns: 1,
                data: vec![100],
                ..MapState::default()
            },
            &stopped_snapshot,
        );
        let current = Arc::new(RwLock::new(None));
        let (tx, rx) = broadcast::channel(4);
        let (latest_tx, mut latest_rx) = watch::channel(None);
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/test/stopped-event/unused-odom").expect("unused odom topic"),
        );
        spawn_map_revision_fanout_in_frame(
            rx,
            latest_tx,
            Arc::new(std::sync::Mutex::new(None)),
            Arc::new(tokio::sync::Mutex::new(ExploreProjectionGate::default())),
            LcmTransport::new(Arc::new(backend.clone())),
            LcmTopic::new("/test/stopped-event/occupancy").expect("occupancy topic"),
            LcmMessageType::new("nav_msgs.OccupancyGrid").expect("occupancy type"),
            Some(writer.clone()),
            Some(shm_path.to_string_lossy().into_owned()),
            current.clone(),
            crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
            mapping_adapter,
            Arc::new(tokio::sync::Mutex::new(())),
            Arc::from("map"),
        );
        tx.send(event).expect("fanout receiver");

        assert!(
            tokio::time::timeout(Duration::from_millis(150), latest_rx.changed())
                .await
                .is_err()
        );
        assert!(current.read().await.is_none());
        assert!(reader.read_stable().expect("stable SHM read").is_none());
        assert_eq!(writer.lock().physical_write_count(), 0);
        assert_eq!(backend.published_count(), 0);
        let _ = std::fs::remove_file(shm_path);
    }

    #[tokio::test]
    async fn handoff_lio_cloud_preserves_configured_map_frame_and_source_zero() {
        let mut handle = build_handle_for_tests().await.expect("test runtime handle");
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), "robot/map", "odom");
        adapter
            .start_session("lio-session", "lio-map")
            .expect("active LIO mapping session");
        adapter
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.0, "odom")
            .expect("map <- odom anchor");
        let mapping = Arc::new(parking_lot::Mutex::new(adapter));
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/unused/lio/odom").expect("unused odom topic"),
        );
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        handle.mapping_adapter = Some(mapping.clone());
        let path = std::env::temp_dir().join(format!(
            "nuwax-lio-configured-frame-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
        )));
        let mut revision_rx = handle.map_revision_tx.subscribe();

        assert!(handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(1.0, 2.0, 0.0)],
            12,
            100,
            0,
            "robot/map",
        )
        .await
        .expect("configured-frame LIO handoff"));
        assert_eq!(mapping.lock().current_revision(), 1);
        let revision = revision_rx.try_recv().expect("map revision");
        assert_eq!(revision.frame_id, "robot/map");
        assert_eq!(revision.timestamp_ns, 0);
        let state = handle.state.read().await;
        assert_eq!(state.map.frame_id, "robot/map");
        assert_eq!(state.map.timestamp_ns, 0);
        assert_eq!(state.cloud_map_revision, 1);
        drop(state);
        let published = backend.published.lock().expect("published frames");
        assert_eq!(published.len(), 1);
        let mut encoded = Vec::with_capacity(8 + published[0].payload.len());
        encoded.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        encoded.extend_from_slice(&published[0].payload);
        let cloud = schema::decode_point_cloud2(&encoded).expect("global cloud payload");
        assert_eq!(cloud.header.frame_id, "robot/map");
        assert_eq!((cloud.header.stamp_sec, cloud.header.stamp_nsec), (0, 0));
        drop(published);
        drop(handle);
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test]
    async fn handoff_lio_cloud_without_shm_writer_still_commits_canonical_revision() {
        let mut handle = build_handle_for_tests().await.expect("test runtime handle");
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), "robot/map", "odom");
        adapter
            .start_session("lio-session", "lio-map")
            .expect("active LIO mapping session");
        adapter
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.0, "odom")
            .expect("map <- odom anchor");
        let mapping = Arc::new(parking_lot::Mutex::new(adapter));
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/unused/lio/odom").expect("unused odom topic"),
        );
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        handle.mapping_adapter = Some(mapping.clone());
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(None));
        let mut revision_rx = handle.map_revision_tx.subscribe();

        assert!(handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(1.0, 2.0, 0.0)],
            12,
            100,
            0,
            "robot/map",
        )
        .await
        .expect("configured-frame LIO handoff without cloud SHM"));
        assert_eq!(mapping.lock().current_revision(), 1);
        let revision = revision_rx.try_recv().expect("map revision");
        assert_eq!(revision.frame_id, "robot/map");
        assert_eq!(revision.revision, 1);
        let state = handle.state.read().await;
        assert_eq!(state.map.frame_id, "robot/map");
        assert_eq!(state.map.revision, 1);
        assert_eq!(state.cloud_map_revision, 0);
        drop(state);
        assert_eq!(backend.published_count(), 0);
    }

    #[tokio::test]
    async fn backend_submission_failure_fences_prepared_session_state() {
        let mut handle = build_handle_for_tests().await.expect("test runtime handle");
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), "robot/map", "odom");
        let session_id = "submit-failure-session";
        adapter
            .start_session(session_id, "submit-failure-map")
            .expect("active mapping session");
        let mapping = Arc::new(parking_lot::Mutex::new(adapter));
        handle.mapping_adapter = Some(mapping.clone());
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(None));

        let client = mapping
            .lock()
            .backend_checkpoint_client()
            .expect("backend checkpoint client");
        let payload = tokio::time::timeout(
            Duration::from_secs(1),
            client
                .export_checkpoint()
                .expect("submit checkpoint export"),
        )
        .await
        .expect("checkpoint export timeout")
        .expect("checkpoint export response")
        .expect("checkpoint payload");
        let restore_resolution = tokio::time::timeout(
            Duration::from_secs(1),
            client
                .restore_checkpoint(payload)
                .expect("submit checkpoint restore"),
        )
        .await
        .expect("checkpoint restore timeout")
        .expect("checkpoint restore response")
        .expect("checkpoint restore validation");
        let queued_export = client
            .export_checkpoint()
            .expect("fill the single-slot worker queue");

        let lifecycle_lock = handle.mapping_lifecycle_lock.clone();
        let cloud_map_writer = handle.cloud_map_writer.clone();
        let state = handle.state.clone();
        let ingest_mapping = mapping.clone();
        let ingest = tokio::spawn(async move {
            ingest_mapping_points_with_backend(
                &ingest_mapping,
                &lifecycle_lock,
                &handle.explore_projection_gate,
                &handle.committed_map_authority,
                &handle.map_revision_latest_tx,
                &cloud_map_writer,
                &[Point3::new(1.0, 2.0, 0.0)],
                12,
                100,
                "robot/map",
                100,
                &state,
                false,
                None,
                None,
                None,
                None,
                None,
            )
            .await
        });
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if !mapping.lock().snapshot().active {
                    break;
                }
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("submission failure must fence the session before retirement join");
        drop(restore_resolution);
        let ingest_result = tokio::time::timeout(Duration::from_secs(1), ingest)
            .await
            .expect("submission failure retirement timeout")
            .expect("owned ingest task");
        let Err(error) = ingest_result else {
            panic!("full worker queue must reject submission");
        };
        assert!(error
            .to_string()
            .contains("mapping backend submission failed"));
        assert!(error.to_string().contains("QueueFull"));
        drop(queued_export);
        let snapshot = mapping.lock().snapshot();
        assert_eq!(
            snapshot.phase,
            crate::mapping_adapter::MappingSessionPhase::Stopped
        );
        assert!(!snapshot.active);
        assert_eq!(snapshot.revision, 0);
        let retry_error = mapping
            .lock()
            .start_session(session_id, "submit-failure-map")
            .expect_err("failed generation must remain terminal");
        assert!(matches!(
            retry_error,
            crate::mapping_adapter::MappingLifecycleError::TerminalSession(_)
        ));
    }

    #[tokio::test]
    async fn handoff_lio_inactive_adapter_returns_false_without_mutation() {
        let mut handle = build_handle_for_tests().await.expect("test runtime handle");
        let mapping = Arc::new(parking_lot::Mutex::new(MappingAdapter::new_with_frames(
            mapping_config(),
            "robot/map",
            "odom",
        )));
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/unused/lio/odom").expect("unused odom topic"),
        );
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        handle.mapping_adapter = Some(mapping.clone());
        let tf_binding = (
            LcmTopic::new("/test/lio/tf").expect("tf topic"),
            LcmMessageType::new("tf2_msgs.TFMessage").expect("tf type"),
        );
        let pose = PoseState {
            qw: 1.0,
            frame_id: "odom".into(),
            timestamp_ns: 0,
            ..PoseState::default()
        };

        assert!(
            !handoff_lio_odometry(&handle, &pose, "base_link", &tf_binding)
                .await
                .expect("inactive odometry is a zero commit")
        );
        assert!(!handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(1.0, 2.0, 0.0)],
            12,
            100,
            0,
            "robot/map",
        )
        .await
        .expect("inactive cloud is a zero commit"));
        let adapter_state = {
            let adapter = mapping.lock();
            (
                adapter.odom_count(),
                adapter.pointcloud_count(),
                adapter.current_revision(),
                adapter.accumulated_count(),
            )
        };
        assert_eq!(adapter_state, (0, 0, 0, 0));
        assert_eq!(backend.published_count(), 0);
        let state = handle.state.read().await;
        assert_eq!(state.pose.timestamp_ns, 0);
        assert!(state.transforms.is_empty());
        assert_eq!(state.cloud_map_revision, 0);
    }

    #[tokio::test]
    async fn handoff_lio_cloud_accepted_without_new_revision() {
        let mut handle = build_handle_for_tests().await.expect("test runtime handle");
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), "map", "odom");
        adapter
            .start_session("lio-session", "lio-map")
            .expect("active LIO mapping session");
        let mapping = Arc::new(parking_lot::Mutex::new(adapter));
        let backend = MappingFeedRecordingBackend::new(
            LcmTopic::new("/unused/lio/odom").expect("unused odom topic"),
        );
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        handle.mapping_adapter = Some(mapping.clone());
        let path = std::env::temp_dir().join(format!(
            "nuwax-lio-zero-commit-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
        )));

        assert!(handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(0.0, 0.0, 0.0)],
            12,
            10_000_000_000,
            0,
            "map",
        )
        .await
        .expect("first revision"));
        assert!(handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(1.0, 0.0, 0.0)],
            12,
            10_100_000_000,
            1,
            "map",
        )
        .await
        .expect("high-rate frame is accepted but coalesced"));
        let adapter_state = {
            let adapter = mapping.lock();
            (
                adapter.current_revision(),
                adapter.pointcloud_count(),
                adapter.accumulated_count(),
            )
        };
        assert_eq!(adapter_state, (1, 2, 2));
        assert_eq!(backend.published_count(), 1);
        assert_eq!(handle.state.read().await.cloud_map_revision, 1);
        {
            let writer = handle.cloud_map_writer.lock();
            let writer = writer.as_ref().expect("cloud-map writer");
            let snapshot = writer.committed_snapshot().expect("first cloud snapshot");
            assert_eq!(snapshot.revision, 1);
            assert_eq!(snapshot.point_count, 1);
        }

        assert!(handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(2.0, 0.0, 0.0)],
            12,
            11_000_000_000,
            2,
            "map",
        )
        .await
        .expect("next cadence revision"));
        assert_eq!(backend.published_count(), 2);
        assert_eq!(handle.state.read().await.cloud_map_revision, 2);
        {
            let writer = handle.cloud_map_writer.lock();
            let writer = writer.as_ref().expect("cloud-map writer");
            let snapshot = writer.committed_snapshot().expect("second cloud snapshot");
            assert_eq!(snapshot.revision, 2);
            assert_eq!(snapshot.point_count, 3);
        }
        drop(handle);
        let _ = std::fs::remove_file(path);
    }
}

#[cfg(test)]
mod exploration_adapter_tests {
    use super::*;
    use crate::explore_runtime::ExplorePoseProvider;

    fn pose_provider(
        timestamp_ns: u64,
        ttl: Duration,
    ) -> (SharedStatePoseProvider, Arc<RwLock<SharedState>>) {
        let state = SharedState {
            pose: PoseState {
                frame_id: "map".to_string(),
                timestamp_ns,
                qw: 1.0,
                ..PoseState::default()
            },
            ..SharedState::default()
        };
        let state = Arc::new(RwLock::new(state));
        (
            SharedStatePoseProvider {
                state: state.clone(),
                expected_map_frame: Arc::from("map"),
                freshness: Arc::new(parking_lot::Mutex::new(PoseFreshnessTracker::default())),
                max_receipt_age: ttl,
            },
            state,
        )
    }

    #[tokio::test]
    async fn pose_provider_requires_fresh_source_and_monotonic_receipt() {
        let (provider, state) = pose_provider(
            crate::explore_runtime::now_unix_nanos(),
            Duration::from_millis(20),
        );
        let first = provider.current_pose().await.expect("fresh pose");
        assert!(first.source_timestamp_ns > 0);
        assert!(first.received_monotonic_ns > 0);

        tokio::time::sleep(Duration::from_millis(25)).await;
        assert!(provider.current_pose().await.unwrap_err().contains("stale"));

        state.write().await.pose.timestamp_ns = crate::explore_runtime::now_unix_nanos();
        let refreshed = provider.current_pose().await.expect("refreshed pose");
        assert!(refreshed.received_monotonic_ns >= first.received_monotonic_ns);

        let (stale_source, _) = pose_provider(
            crate::explore_runtime::now_unix_nanos().saturating_sub(100_000_000),
            Duration::from_millis(20),
        );
        assert!(stale_source
            .current_pose()
            .await
            .unwrap_err()
            .contains("source timestamp is stale"));
    }

    fn exploration_update(status: &str) -> NavigationUpdate {
        NavigationUpdate {
            producer_runtime_generation: 0,
            state: NavState {
                status: status.to_string(),
                goal_id: "explore-1.nav.1".to_string(),
                message: "terminal".to_string(),
                ..NavState::default()
            },
            feedback: NavigationFeedback {
                owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                parent_request_id: "explore-1".to_string(),
                goal_generation: 7,
                goal_payload_sha256: [9; 32],
                ..NavigationFeedback::default()
            },
        }
    }

    #[test]
    fn exploration_terminal_adapter_preserves_zeroed_and_failure_proof() {
        let mut cancelled = exploration_update("CANCELLED");
        cancelled.feedback.actuator_zeroed = true;
        cancelled.feedback.cancel_generation = 11;
        assert!(exploration_navigation_terminal_result(&cancelled).is_none());
        cancelled.feedback.zero_ack_id = "zero-11".to_string();
        cancelled.feedback.zero_applied_monotonic_ns = 123;
        let result = exploration_navigation_terminal_result(&cancelled).expect("complete zero");
        let crate::explore_runtime::ChildNavigationResult::Cancelled(proof) = result else {
            panic!("expected cancelled proof");
        };
        assert_eq!(proof.cancel_generation, 11);
        assert_eq!(proof.zero_ack_id, "zero-11");
        assert_eq!(proof.lease.parent_request_id, "explore-1");

        let mut timed_out = exploration_update("FAILED");
        timed_out.feedback.cancel_phase = "TIMED_OUT".to_string();
        timed_out.feedback.cancel_generation = 11;
        assert!(matches!(
            exploration_navigation_terminal_result(&timed_out),
            Some(
                crate::explore_runtime::ChildNavigationResult::CancelFailed {
                    cancel_generation: 11,
                    ..
                }
            )
        ));
    }
}

#[cfg(test)]
mod typed_navigation_feedback_tests {
    use super::*;
    use std::io::Write;

    #[derive(Clone, Default)]
    struct CapturedLogs(Arc<std::sync::Mutex<Vec<u8>>>);

    struct CapturedLogWriter(Arc<std::sync::Mutex<Vec<u8>>>);

    impl CapturedLogs {
        fn snapshot(&self) -> String {
            String::from_utf8(self.0.lock().expect("captured log mutex").clone())
                .expect("tracing output is UTF-8")
        }
    }

    impl Write for CapturedLogWriter {
        fn write(&mut self, buffer: &[u8]) -> std::io::Result<usize> {
            self.0
                .lock()
                .expect("captured log mutex")
                .extend_from_slice(buffer);
            Ok(buffer.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl<'writer> tracing_subscriber::fmt::MakeWriter<'writer> for CapturedLogs {
        type Writer = CapturedLogWriter;

        fn make_writer(&'writer self) -> Self::Writer {
            CapturedLogWriter(self.0.clone())
        }
    }

    fn test_lease(goal_id: &str) -> crate::nav_runtime::GoalLease {
        crate::nav_runtime::GoalLease {
            session_id: goal_id.to_string(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: format!("request-{goal_id}"),
            goal_generation: 1,
            producer_runtime_generation: 1,
            goal_payload_sha256: [7; 32],
            path_revision: 0,
            path_payload_sha256: [0; 32],
        }
    }

    fn feedback_update(goal_id: &str) -> NavigationUpdate {
        let lease = (!goal_id.is_empty()).then(|| test_lease(goal_id));
        NavigationUpdate {
            producer_runtime_generation: lease
                .as_ref()
                .map_or(0, |lease| lease.producer_runtime_generation),
            state: NavState {
                status: "IDLE".into(),
                message: "guard-backed scheduler heartbeat".into(),
                goal_id: goal_id.into(),
                distance_remaining: 0.0,
                timestamp_ns: 10,
            },
            feedback: NavigationFeedback {
                runtime_generation: 7,
                guard_generation: 11,
                sink_ready: true,
                planner_map_frame: "map".into(),
                planner_pose_frame: "map".into(),
                owner_kind: lease.as_ref().map_or(0, |lease| lease.owner_kind),
                parent_request_id: lease
                    .as_ref()
                    .map_or_else(String::new, |lease| lease.parent_request_id.clone()),
                goal_generation: lease.as_ref().map_or(0, |lease| lease.goal_generation),
                goal_payload_sha256: lease
                    .as_ref()
                    .map_or([0; 32], |lease| lease.goal_payload_sha256),
                ..NavigationFeedback::default()
            },
        }
    }

    async fn handle_with_active_path_lease(
        goal_id: &str,
    ) -> (GeometryRuntimeHandle, crate::nav_runtime::GoalLease) {
        let handle = build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let mut lease = test_lease(goal_id);
        lease.path_revision = 1;
        lease.path_payload_sha256 = [8; 32];
        {
            let inner = handle.nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Running;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 1,
                lease: lease.clone(),
            });
        }
        (handle, lease)
    }

    async fn begin_pending_cancel(
        handle: &GeometryRuntimeHandle,
        lease: &crate::nav_runtime::GoalLease,
    ) -> crate::nav_runtime::CancelSnapshot {
        let pending = crate::nav_runtime::cancel(
            handle,
            &handle.nav,
            crate::nav_runtime::CancelGoalInput {
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
            },
        )
        .await;
        assert_eq!(pending.state.status, "PENDING");
        handle.nav.cancel_snapshot()
    }

    #[tokio::test]
    async fn typed_feedback_drives_planner_and_motion_observations_without_map_sequence() {
        let handle = build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 3,
                map_frame: "map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");

        let update = apply_typed_navigation_feedback(&handle.nav, feedback_update(""))
            .await
            .expect("unowned heartbeat");
        assert_eq!(update.state.status, "IDLE");
        let readiness = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
        assert!(readiness.ready, "{}", readiness.reason);
        assert_eq!(readiness.planner_generation, 7);
        assert_eq!(readiness.motion_sink_generation, 11);
        assert_eq!(readiness.costmap_revision, 0);
    }

    #[tokio::test]
    async fn idle_transport_heartbeat_opens_readiness_without_guard_generation() {
        let handle = build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 3,
                map_frame: "map".into(),
                odom_frame: "odom".into(),
                base_frame: "base_link".into(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        let mut heartbeat = feedback_update("");
        heartbeat.feedback.guard_generation = 0;

        let update = apply_typed_navigation_feedback(&handle.nav, heartbeat)
            .await
            .expect("unowned heartbeat");

        assert_eq!(update.state.status, "IDLE");
        let readiness = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
        assert!(readiness.ready, "{}", readiness.reason);
        assert!(readiness.motion_sink_observed);
        assert!(readiness.motion_sink_fresh);
        assert!(readiness.cmd_vel_sink_ready);
        assert_eq!(readiness.motion_sink_generation, 0);
    }

    #[tokio::test]
    async fn unowned_heartbeat_cannot_mutate_or_impersonate_an_active_goal() {
        for status in ["IDLE", "FAILED", "CANCELLED", "SUCCEEDED"] {
            let handle = build_handle_for_tests()
                .await
                .expect("real-lcm test handle");
            let lease_b = test_lease("unowned-active-b");
            {
                let inner = handle.nav.inner();
                let mut slot = inner.write().await;
                slot.phase = crate::nav_runtime::NavPhase::Running;
                slot.message = "B remains active".into();
                slot.goal = Some(crate::nav_runtime::ActiveGoal {
                    id: lease_b.session_id.clone(),
                    frame_id: "map".to_string(),
                    target: PoseState::default(),
                    accepted_at_ns: 1,
                    lease: lease_b.clone(),
                });
            }
            let before_cancel = handle.nav.cancel_snapshot();
            let mut heartbeat = feedback_update("");
            heartbeat.state.status = status.into();
            heartbeat.state.message = "unowned scheduler heartbeat".into();

            let applied = apply_typed_navigation_feedback(&handle.nav, heartbeat)
                .await
                .expect("unowned heartbeat remains valid for liveness");
            assert!(
                applied.state.goal_id.is_empty(),
                "an unowned heartbeat must not impersonate B"
            );
            assert_eq!(handle.nav.cancel_snapshot(), before_cancel);
            let inner = handle.nav.inner();
            let slot = inner.read().await;
            assert_eq!(slot.phase, crate::nav_runtime::NavPhase::Running);
            assert_eq!(slot.message, "B remains active");
            assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
        }
    }

    #[tokio::test]
    async fn unowned_heartbeat_cannot_change_a_pending_cancel_transaction() {
        for status in ["IDLE", "FAILED"] {
            let handle = build_handle_for_tests()
                .await
                .expect("real-lcm test handle");
            let lease_b = test_lease("unowned-pending-b");
            {
                let inner = handle.nav.inner();
                let mut slot = inner.write().await;
                slot.phase = crate::nav_runtime::NavPhase::Running;
                slot.goal = Some(crate::nav_runtime::ActiveGoal {
                    id: lease_b.session_id.clone(),
                    frame_id: "map".to_string(),
                    target: PoseState::default(),
                    accepted_at_ns: 1,
                    lease: lease_b.clone(),
                });
            }
            let pending = crate::nav_runtime::cancel(
                &handle,
                &handle.nav,
                crate::nav_runtime::CancelGoalInput {
                    goal_id: lease_b.session_id.clone(),
                    owner_kind: lease_b.owner_kind,
                    parent_request_id: lease_b.parent_request_id.clone(),
                    goal_generation: lease_b.goal_generation,
                    producer_runtime_generation: lease_b.producer_runtime_generation,
                },
            )
            .await;
            assert_eq!(pending.state.status, "PENDING");
            let before_cancel = handle.nav.cancel_snapshot();
            let inner = handle.nav.inner();
            let before_phase = inner.read().await.phase;

            let mut heartbeat = feedback_update("");
            heartbeat.state.status = status.into();
            heartbeat.state.message = "unowned scheduler heartbeat".into();
            let applied = apply_typed_navigation_feedback(&handle.nav, heartbeat)
                .await
                .expect("unowned heartbeat remains valid for liveness");

            assert!(applied.state.goal_id.is_empty());
            assert_eq!(handle.nav.cancel_snapshot(), before_cancel);
            let slot = inner.read().await;
            assert_eq!(slot.phase, before_phase);
            assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
        }
    }

    #[tokio::test]
    async fn zero_ack_requires_goal_and_cancel_generation_correlation() {
        let handle = build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let lease = test_lease("goal-zero");
        {
            let inner = handle.nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Running;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 1,
                lease: lease.clone(),
            });
        }
        let pending = crate::nav_runtime::cancel(
            &handle,
            &handle.nav,
            crate::nav_runtime::CancelGoalInput {
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
            },
        )
        .await;
        assert_eq!(pending.state.status, "PENDING");
        let cancel = handle.nav.cancel_snapshot();

        let mut stale = feedback_update("goal-zero");
        stale.state.status = "CANCELLED".into();
        stale.state.message = "planner stopped".into();
        stale.feedback.cancel_generation = cancel.generation.saturating_add(1);
        stale.feedback.zero_ack_id = "wrong-generation".into();
        stale.feedback.zero_applied_monotonic_ns = 20;
        let stale = apply_typed_navigation_feedback(&handle.nav, stale)
            .await
            .expect("correlated lease");
        assert_eq!(stale.state.status, "PENDING");
        assert!(!stale.feedback.actuator_zeroed);
        assert!(stale.feedback.zero_ack_id.is_empty());

        let mut correlated = feedback_update("goal-zero");
        correlated.state.status = "CANCELLED".into();
        correlated.state.message = "planner stopped".into();
        correlated.feedback.cancel_generation = cancel.generation;
        correlated.feedback.zero_ack_id = "zero-ok".into();
        correlated.feedback.zero_applied_monotonic_ns = 21;
        let correlated = apply_typed_navigation_feedback(&handle.nav, correlated)
            .await
            .expect("correlated zero ack");
        assert_eq!(correlated.state.status, "CANCELLED");
        assert_eq!(correlated.feedback.cancel_phase, "ZEROED");
        assert!(correlated.feedback.actuator_zeroed);
        assert_eq!(
            handle.nav.cancel_snapshot().phase,
            crate::nav_runtime::CancelPhase::Zeroed
        );
    }

    #[tokio::test]
    async fn pending_zero_ack_accepts_pre_path_sentinel_after_path_rollover() {
        let (handle, lease) = handle_with_active_path_lease("goal-path-race").await;
        let cancel = begin_pending_cancel(&handle, &lease).await;

        let mut update = feedback_update(&lease.session_id);
        update.state.status = "CANCELLED".into();
        update.state.message = "planner stopped".into();
        update.feedback.cancel_generation = cancel.generation;
        update.feedback.zero_ack_id = "zero-path-race".into();
        update.feedback.zero_applied_monotonic_ns = 21;

        let applied = apply_typed_navigation_feedback(&handle.nav, update)
            .await
            .expect("correlated pre-path zero acknowledgement");
        assert_eq!(applied.state.status, "CANCELLED");
        assert_eq!(applied.feedback.path_revision, lease.path_revision);
        assert_eq!(
            applied.feedback.path_payload_sha256,
            lease.path_payload_sha256
        );
        assert_eq!(applied.feedback.cancel_phase, "ZEROED");
        assert!(applied.feedback.actuator_zeroed);
        assert_eq!(
            handle.nav.cancel_snapshot().phase,
            crate::nav_runtime::CancelPhase::Zeroed
        );
    }

    #[tokio::test]
    async fn exact_late_zero_recovers_failed_or_timed_out_cancel_with_pre_path_sentinel() {
        for recover_from_failure in [true, false] {
            let goal_id = if recover_from_failure {
                "goal-late-zero-failed"
            } else {
                "goal-late-zero-timeout"
            };
            let (handle, lease) = handle_with_active_path_lease(goal_id).await;
            let cancel = begin_pending_cancel(&handle, &lease).await;
            if recover_from_failure {
                let failed = handle
                    .nav
                    .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                        cancel_generation: cancel.generation,
                        goal_id: lease.session_id.clone(),
                        owner_kind: lease.owner_kind,
                        parent_request_id: lease.parent_request_id.clone(),
                        goal_generation: lease.goal_generation,
                        status: crate::nav_runtime::MotionSinkCancelAckStatus::Failed,
                        message: "guard reported cancellation failure".to_string(),
                    })
                    .await;
                assert_eq!(failed.state.status, "FAILED");
                assert_eq!(
                    handle.nav.cancel_snapshot().phase,
                    crate::nav_runtime::CancelPhase::Failed
                );
            } else {
                handle.nav.expire_cancel_for_test();
                assert!(handle.nav.poll_cancel_timeout().await.is_some());
                assert_eq!(
                    handle.nav.cancel_snapshot().phase,
                    crate::nav_runtime::CancelPhase::TimedOut
                );
            }
            let retained_phase = handle.nav.cancel_snapshot().phase;

            let mut wrong_generation = feedback_update(&lease.session_id);
            wrong_generation.state.status = "CANCELLED".into();
            wrong_generation.feedback.cancel_generation = cancel.generation.saturating_add(1);
            wrong_generation.feedback.zero_ack_id = "late-zero-wrong-generation".into();
            wrong_generation.feedback.zero_applied_monotonic_ns = 31;
            assert!(
                apply_typed_navigation_feedback(&handle.nav, wrong_generation)
                    .await
                    .is_none()
            );
            assert_eq!(handle.nav.cancel_snapshot().phase, retained_phase);

            let mut wrong_lease = feedback_update(&lease.session_id);
            wrong_lease.state.status = "CANCELLED".into();
            wrong_lease.feedback.owner_kind += 1;
            wrong_lease.feedback.cancel_generation = cancel.generation;
            wrong_lease.feedback.zero_ack_id = "late-zero-wrong-lease".into();
            wrong_lease.feedback.zero_applied_monotonic_ns = 32;
            assert!(apply_typed_navigation_feedback(&handle.nav, wrong_lease)
                .await
                .is_none());
            assert_eq!(handle.nav.cancel_snapshot().phase, retained_phase);

            let mut exact = feedback_update(&lease.session_id);
            exact.state.status = "CANCELLED".into();
            exact.state.message = "late actuator ZEROED".into();
            exact.feedback.cancel_generation = cancel.generation;
            exact.feedback.zero_ack_id = "late-zero-exact".into();
            exact.feedback.zero_applied_monotonic_ns = 33;
            let exact = apply_typed_navigation_feedback(&handle.nav, exact)
                .await
                .expect("exact late ZEROED proof");

            assert_eq!(exact.state.status, "CANCELLED");
            assert_eq!(exact.feedback.cancel_phase, "ZEROED");
            assert!(exact.feedback.actuator_zeroed);
            assert_eq!(exact.feedback.path_revision, lease.path_revision);
            assert_eq!(
                exact.feedback.path_payload_sha256,
                lease.path_payload_sha256
            );
            assert_eq!(
                handle.nav.cancel_snapshot().phase,
                crate::nav_runtime::CancelPhase::Zeroed
            );
        }
    }

    #[tokio::test]
    async fn pending_zero_ack_with_wrong_nonzero_path_identity_remains_rejected() {
        let (handle, lease) = handle_with_active_path_lease("goal-wrong-path").await;
        let cancel = begin_pending_cancel(&handle, &lease).await;

        let mut update = feedback_update(&lease.session_id);
        update.state.status = "CANCELLED".into();
        update.feedback.path_revision = lease.path_revision.saturating_add(1);
        update.feedback.path_payload_sha256 = [9; 32];
        update.feedback.cancel_generation = cancel.generation;
        update.feedback.zero_ack_id = "zero-wrong-path".into();
        update.feedback.zero_applied_monotonic_ns = 22;

        assert!(
            apply_typed_navigation_feedback(&handle.nav, update)
                .await
                .is_none(),
            "non-sentinel path mismatches must fail closed"
        );
        assert_eq!(
            handle.nav.cancel_snapshot().phase,
            crate::nav_runtime::CancelPhase::Pending
        );
    }

    #[tokio::test]
    async fn pending_incomplete_zero_ack_with_path_sentinel_remains_rejected() {
        let (handle, lease) = handle_with_active_path_lease("goal-incomplete-zero").await;
        let cancel = begin_pending_cancel(&handle, &lease).await;

        let mut update = feedback_update(&lease.session_id);
        update.state.status = "CANCELLED".into();
        update.feedback.cancel_generation = cancel.generation;
        update.feedback.zero_applied_monotonic_ns = 23;

        assert!(
            apply_typed_navigation_feedback(&handle.nav, update)
                .await
                .is_none(),
            "path sentinels require a complete correlated ZERO proof"
        );
        assert_eq!(
            handle.nav.cancel_snapshot().phase,
            crate::nav_runtime::CancelPhase::Pending
        );
    }

    #[tokio::test]
    async fn navigation_state_with_wrong_lease_or_hash_is_rejected_before_observation() {
        let handle = build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let lease = test_lease("goal-correlated");
        {
            let inner = handle.nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Running;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 1,
                lease,
            });
        }
        let mut wrong_owner = feedback_update("goal-correlated");
        wrong_owner.feedback.owner_kind = schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
        assert!(apply_typed_navigation_feedback(&handle.nav, wrong_owner)
            .await
            .is_none());
        let mut wrong_hash = feedback_update("goal-correlated");
        wrong_hash.feedback.goal_payload_sha256 = [9; 32];
        assert!(apply_typed_navigation_feedback(&handle.nav, wrong_hash)
            .await
            .is_none());
    }

    #[test]
    fn rejection_diagnostics_report_the_active_lease_identity() {
        let nav = crate::nav_runtime::NavRuntimeState::from_env();
        let mut lease = test_lease("goal-diagnostic-source");
        lease.path_revision = 7;
        lease.path_payload_sha256 = [8; 32];
        futures::executor::block_on(async {
            let inner = nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Running;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 1,
                lease: lease.clone(),
            });
        });

        let captured = CapturedLogs::default();
        let subscriber = tracing_subscriber::fmt()
            .without_time()
            .with_ansi(false)
            .with_target(false)
            .with_writer(captured.clone())
            .finish();
        let rejected = tracing::subscriber::with_default(subscriber, || {
            futures::executor::block_on(apply_typed_navigation_feedback(
                &nav,
                feedback_update(&lease.session_id),
            ))
        });

        assert!(rejected.is_none());
        let output = captured.snapshot();
        assert!(output.contains("typed navigation state rejected"));
        assert!(output.contains("expected_lease_source=\"active\""));
        assert!(output.contains("expected_path_revision=7"));
        assert!(output.contains("expected_path_hash_zero=false"));
        assert!(output.contains("path_hash_matches=false"));
    }

    #[tokio::test]
    async fn correlated_a_terminal_cannot_mutate_replacement_b_incarnation() {
        let handle = build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 7,
                map_revision: 42,
                map_frame: "map".into(),
                pose_frame: "map".into(),
                ready: true,
            })
            .expect("baseline planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 11,
                ready: true,
            })
            .expect("baseline sink observation");
        let baseline = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
        let lease_a = test_lease("same-feedback-goal");
        {
            let inner = handle.nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Running;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease_a.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 1,
                lease: lease_a.clone(),
            });
        }

        let mut stale_a = feedback_update("same-feedback-goal");
        stale_a.state.status = "FAILED".into();
        stale_a.state.message = "late A terminal".into();
        stale_a.feedback.runtime_generation = 99;
        stale_a.feedback.planner_map_frame = "map".into();
        stale_a.feedback.planner_pose_frame = "odom".into();
        stale_a.feedback.guard_generation = 99;
        stale_a.feedback.sink_ready = false;
        let mut lease_b = lease_a.clone();
        lease_b.path_revision = 1;
        lease_b.path_payload_sha256 = [8; 32];

        let stale_planner = crate::nav_runtime::PlannerInputObservation {
            generation: stale_a.feedback.runtime_generation,
            map_revision: baseline.costmap_revision,
            map_frame: stale_a.feedback.planner_map_frame.clone(),
            pose_frame: stale_a.feedback.planner_pose_frame.clone(),
            ready: false,
        };
        let stale_sink = crate::nav_runtime::MotionSinkObservation {
            generation: stale_a.feedback.guard_generation,
            ready: stale_a.feedback.sink_ready,
        };
        let correlation_barrier = Arc::new(tokio::sync::Barrier::new(2));
        handle
            .nav
            .inject_transaction_after_correlation_barrier(correlation_barrier.clone());
        let nav_a = handle.nav.clone();
        let transaction_a = tokio::spawn(async move {
            nav_a
                .apply_correlated_navigation_state(
                    stale_a.state.clone(),
                    &stale_a.state.goal_id,
                    stale_a.feedback.owner_kind,
                    &stale_a.feedback.parent_request_id,
                    stale_a.feedback.goal_generation,
                    stale_a.feedback.path_revision,
                    stale_a.feedback.goal_payload_sha256,
                    stale_a.feedback.path_payload_sha256,
                    Some(stale_planner),
                    Some(stale_sink),
                )
                .await
        });
        tokio::time::timeout(Duration::from_secs(1), correlation_barrier.wait())
            .await
            .expect("A transaction must reach the correlation/promotion boundary");

        let replacement_goal = crate::nav_runtime::ActiveGoal {
            id: lease_b.session_id.clone(),
            frame_id: "map".to_string(),
            target: PoseState::default(),
            accepted_at_ns: 2,
            lease: lease_b.clone(),
        };
        let nav_b = handle.nav.clone();
        let mut replacement_b =
            tokio::spawn(async move { nav_b.replace_active_goal_for_test(replacement_goal).await });
        assert!(
            tokio::time::timeout(Duration::from_millis(50), &mut replacement_b)
                .await
                .is_err(),
            "replacement B must queue behind A's single N transaction"
        );
        tokio::time::timeout(Duration::from_secs(1), correlation_barrier.wait())
            .await
            .expect("the test must release A's final promotion");

        let (applied_state, applied_lease) =
            tokio::time::timeout(Duration::from_secs(1), transaction_a)
                .await
                .expect("A transaction must finish after barrier release")
                .expect("A transaction task must not panic")
                .expect("A terminal must commit before queued replacement B");
        assert_eq!(applied_state.status, "FAILED");
        assert_eq!(applied_lease, lease_a);
        tokio::time::timeout(Duration::from_secs(1), replacement_b)
            .await
            .expect("replacement B must finish after A releases N")
            .expect("replacement B task must not panic");

        let inner = handle.nav.inner();
        let slot = inner.read().await;
        assert_eq!(slot.phase, crate::nav_runtime::NavPhase::Running);
        assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
        drop(slot);
        let after = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
        assert_eq!(after.planner_generation, 99);
        assert_eq!(after.map_frame, "map");
        assert_eq!(after.pose_frame, "odom");
        assert_eq!(after.motion_sink_generation, 99);
        assert!(!after.cmd_vel_sink_ready);
    }

    #[tokio::test]
    async fn geometry_consumes_only_correlated_path_envelopes_over_real_lcm() {
        let transport = LcmTransport::new_real(&test_lcm_url()).expect("real LCM transport");
        let nav = crate::nav_runtime::NavRuntimeState::from_env();
        let lease = test_lease("path-lcm");
        {
            let inner = nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Planning;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 1,
                lease: lease.clone(),
            });
        }
        let topic = LcmTopic::new(&format!("/test/geometry/path/{}", std::process::id()))
            .expect("path topic");
        let envelope_type =
            LcmMessageType::new("nuwax_msgs.PathEnvelope").expect("PathEnvelope type");
        spawn_path_envelope_consumer(
            transport.clone(),
            nav.clone(),
            (topic.clone(), envelope_type.clone()),
        );
        tokio::time::sleep(Duration::from_millis(30)).await;

        let raw_path = schema::encode_path_body(&schema::Path {
            header: schema::Header {
                frame_id: "map".to_string(),
                ..schema::Header::default()
            },
            poses: vec![schema::PoseStamped {
                header: schema::Header {
                    frame_id: "map".to_string(),
                    ..schema::Header::default()
                },
                ..schema::PoseStamped::default()
            }],
        })
        .expect("raw path");
        transport
            .publish(crate::typed_codec::build_typed_frame(
                topic.clone(),
                LcmMessageType::new("nav_msgs.Path").expect("raw Path type"),
                raw_path,
                1,
            ))
            .await
            .expect("publish raw path");
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(nav.active_goal_lease().await.unwrap().path_revision, 0);

        let mut path = schema::PathEnvelope {
            schema_version: schema::NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: lease.session_id.clone(),
            owner_kind: lease.owner_kind,
            parent_request_id: lease.parent_request_id.clone(),
            goal_generation: lease.goal_generation as i64,
            producer: "public-navigation".to_string(),
            producer_epoch: lease.producer_runtime_generation as i64,
            producer_runtime_generation: lease.producer_runtime_generation as i64,
            committed_map_session_id: "path-lcm-map".to_string(),
            committed_map_session_generation: 1,
            committed_map_revision: 1,
            committed_map_payload_sha256: [0x41; 32],
            committed_map_frame_id: "map".to_string(),
            motion_guard_generation: 1,
            motion_guard_boot_nonce: [1; 16],
            motion_stop_generation: 0,
            planner_runtime_generation: 1,
            path_revision: 1,
            source_timestamp_ns: 10,
            goal_payload_sha256: lease.goal_payload_sha256,
            path: schema::Path {
                header: schema::Header {
                    stamp_sec: 1,
                    frame_id: "map".to_string(),
                    ..schema::Header::default()
                },
                poses: vec![schema::PoseStamped {
                    header: schema::Header {
                        stamp_sec: 1,
                        frame_id: "map".to_string(),
                        ..schema::Header::default()
                    },
                    pose: schema::Pose {
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                        ..schema::Pose::default()
                    },
                }],
            },
            ..schema::PathEnvelope::default()
        };
        schema::seal_path_envelope(&mut path).expect("seal path envelope");
        transport
            .publish(crate::typed_codec::build_typed_frame(
                topic,
                envelope_type,
                schema::encode_path_envelope_body(&path).expect("encode PathEnvelope"),
                2,
            ))
            .await
            .expect("publish PathEnvelope");
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(
            nav.active_goal_lease().await.as_ref(),
            Some(&lease),
            "PathEnvelope publication must remain a candidate"
        );

        let mut state = feedback_update(&lease.session_id);
        state.feedback.path_revision = 1;
        state.feedback.path_payload_sha256 = path.path_payload_sha256;
        let promoted = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if let Some(promoted) = apply_typed_navigation_feedback(&nav, state.clone()).await {
                    break promoted;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("exact NavigationState must promote the observed candidate");
        assert_eq!(promoted.feedback.path_revision, 1);
        assert_eq!(
            nav.active_goal_lease().await.unwrap().path_payload_sha256,
            path.path_payload_sha256
        );
    }
}

#[cfg(test)]
mod real_lcm_startup_tests {
    use std::cell::Cell;

    use super::{attempt_real_lcm_startup, RealLcmStartupError};

    #[test]
    fn required_backend_attempts_creation_and_accepts_success() {
        let attempted = Cell::new(false);

        let result = attempt_real_lcm_startup(true, || {
            attempted.set(true);
            Ok::<_, &'static str>("real-lcm")
        });

        assert!(attempted.get(), "required=true 也必须先尝试创建 real-lcm");
        assert_eq!(result, Ok("real-lcm"));
    }

    #[test]
    fn required_backend_classifies_actual_creation_failure_for_main() {
        let result = attempt_real_lcm_startup(true, || Err::<(), _>("liblcm unavailable"));

        assert_eq!(
            result,
            Err(RealLcmStartupError::Required("liblcm unavailable"))
        );
    }

    #[test]
    fn optional_backend_returns_actual_creation_failure() {
        let result = attempt_real_lcm_startup(false, || Err::<(), _>("invalid provider"));

        assert_eq!(
            result,
            Err(RealLcmStartupError::Optional("invalid provider"))
        );
    }
}

#[cfg(test)]
mod exploration_cancel_retry_tests {
    use super::*;
    use futures::stream;
    use nuwax_protocol::RawLcmFrame;
    use nuwax_transport::{LcmBackend, LcmFrameStream, TransportResult};
    use std::sync::Mutex;

    #[derive(Clone, Default)]
    struct RecordingBackend {
        frames: Arc<Mutex<Vec<RawLcmFrame>>>,
    }

    #[async_trait]
    impl LcmBackend for RecordingBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            self.frames.lock().expect("recording backend").push(frame);
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(stream::empty())
        }

        fn name(&self) -> &str {
            "recording"
        }
    }

    impl RecordingBackend {
        fn cancel_count(&self) -> usize {
            self.frames
                .lock()
                .expect("recording backend")
                .iter()
                .filter(|frame| frame.msg_type.fqn() == "nuwax_msgs.CancelNavigation")
                .count()
        }

        fn decoded_cancels(&self) -> Vec<schema::CancelNavigation> {
            self.frames
                .lock()
                .expect("recording backend")
                .iter()
                .filter(|frame| frame.msg_type.fqn() == "nuwax_msgs.CancelNavigation")
                .map(|frame| {
                    schema::decode_cancel_navigation_body(&frame.payload)
                        .expect("decode CancelNavigation")
                })
                .collect()
        }
    }

    fn frontier_costmap() -> crate::explore_runtime::ExploreCostmapSnapshot {
        let mut grid = OccupancyGrid2D {
            resolution: 0.5,
            origin: Point2 { x: 0.0, y: 0.0 },
            width: 12,
            height: 12,
            cells: vec![OccupancyCell::Unknown; 144],
        };
        for y in 2..=7 {
            for x in 2..=7 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        crate::explore_runtime::ExploreCostmapSnapshot {
            grid,
            frame_id: "map".to_string(),
        }
    }

    #[derive(Clone)]
    struct DelayedExploreBridge {
        inner: ExploreNavigationBridge,
        submit_entered: Arc<tokio::sync::Notify>,
        submit_release: Arc<tokio::sync::Notify>,
    }

    #[async_trait]
    impl crate::explore_runtime::ExploreNavigationSink for DelayedExploreBridge {
        fn current_cancel_generation(&self) -> Result<u64, String> {
            crate::explore_runtime::ExploreNavigationSink::current_cancel_generation(&self.inner)
        }

        fn current_map_name(&self) -> Result<String, String> {
            crate::explore_runtime::ExploreNavigationSink::current_map_name(&self.inner)
        }

        async fn submit(
            &self,
            request: crate::explore_runtime::ChildNavigationRequest,
        ) -> Result<
            crate::explore_runtime::ChildNavigationTicket,
            crate::explore_runtime::ChildNavigationSubmitError,
        > {
            self.submit_entered.notify_one();
            self.submit_release.notified().await;
            crate::explore_runtime::ExploreNavigationSink::submit(&self.inner, request).await
        }

        async fn cancel(
            &self,
            lease: &crate::explore_runtime::ChildNavigationLease,
        ) -> Result<crate::explore_runtime::ChildNavigationCancelTicket, String> {
            crate::explore_runtime::ExploreNavigationSink::cancel(&self.inner, lease).await
        }
    }

    async fn configured_bridge() -> (ExploreNavigationBridge, RecordingBackend) {
        let backend = RecordingBackend::default();
        let transport = LcmTransport::new(Arc::new(backend.clone()));
        let nav = crate::nav_runtime::NavRuntimeState::from_env();
        nav.observe_stack_state(NavState {
            status: "IDLE".to_string(),
            message: "retry test heartbeat".to_string(),
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            ..NavState::default()
        })
        .await;
        nav.observe_tf_chain(crate::nav_runtime::TfChainObservation {
            generation: 1,
            map_frame: "map".to_string(),
            odom_frame: "odom".to_string(),
            base_frame: "base_link".to_string(),
            map_to_odom: true,
            odom_to_base: true,
        })
        .expect("TF observation");
        nav.observe_planner_input(crate::nav_runtime::PlannerInputObservation {
            generation: 1,
            map_revision: 1,
            map_frame: "map".to_string(),
            pose_frame: "map".to_string(),
            ready: true,
        })
        .expect("planner observation");
        nav.observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
            generation: 1,
            ready: true,
        })
        .expect("motion sink observation");

        let (cloud_map_tx, _) = broadcast::channel(4);
        let (nav_state_tx, _) = broadcast::channel(4);
        let (frontiers_tx, _) = broadcast::channel(4);
        let (map_revision_tx, _) = broadcast::channel(4);
        let (map_revision_latest_tx, _) = watch::channel(Some(MapRevisionEvent {
            session_id: "exploration-test-map".to_string(),
            map_name: "office".to_string(),
            session_generation: 1,
            revision: 1,
            width: 12,
            height: 12,
            resolution: 0.5,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "map".to_string(),
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            data: Vec::<u8>::new().into(),
            raw_data: Vec::<u8>::new().into(),
            size: 0,
            schema_version: 1,
            checksum: crate::map_checksum::sha256_hex(&[]),
            raw_checksum: crate::map_checksum::sha256_hex(&[]),
            last_seen_ms: 0,
        }));
        let state = Arc::new(RwLock::new(SharedState::default()));
        let bridge = ExploreNavigationBridge {
            state: state.clone(),
            expected_map_frame: Arc::from("map"),
            cloud_map_tx,
            nav_state_tx,
            frontiers_tx,
            map_revision_tx,
            map_revision_latest_tx,
            explore_projection_gate: Arc::new(tokio::sync::Mutex::new(
                ExploreProjectionGate::default(),
            )),
            committed_map_authority: Arc::new(std::sync::Mutex::new(None)),
            mapping_lifecycle_lock: Arc::new(tokio::sync::Mutex::new(())),
            transport,
            transport_backend_name: "real-lcm",
            lcm_backend_available: true,
            cmd_vel_sink_ready: Arc::new(std::sync::atomic::AtomicBool::new(true)),
            nav: nav.clone(),
            exploration_producer_epoch: Arc::new(std::sync::atomic::AtomicU64::new(1)),
            runtime_observation_generation: Arc::new(std::sync::atomic::AtomicU64::new(1)),
            orphan_cancellations: Arc::new(
                crate::explore_runtime::OrphanCancellationRegistry::default(),
            ),
        };
        *bridge
            .committed_map_authority
            .lock()
            .expect("exploration fixture committed map authority") =
            Some(test_committed_map_identity(1));
        (bridge, backend)
    }

    async fn start_goal(
        goal_id: &str,
    ) -> (
        ExploreNavigationBridge,
        RecordingBackend,
        crate::nav_runtime::GoalLease,
    ) {
        let (bridge, backend) = configured_bridge().await;
        let producer_epoch = bridge
            .exploration_producer_epoch
            .load(std::sync::atomic::Ordering::Acquire);
        let committed_map = {
            bridge
                .committed_map_authority
                .lock()
                .expect("exploration fixture committed map authority")
                .clone()
                .expect("exploration fixture committed map identity")
        };
        let outcome = crate::nav_runtime::admit_goal_intent(
            &crate::nav_runtime::NavAdmissionContext::new(
                bridge.mapping_lifecycle_lock.clone(),
                bridge.committed_map_authority.clone(),
                bridge.map_revision_latest_tx.subscribe(),
            ),
            &crate::nav_runtime::NavAdmissionIo::new(
                bridge.state.clone(),
                bridge.transport.clone(),
                bridge.transport_backend_name,
                bridge.lcm_backend_available,
            ),
            &bridge.nav,
            test_wavefront_goal_intent(
                goal_id,
                "exploration-retry-parent",
                producer_epoch,
                bridge.nav.cancel_snapshot().generation,
                committed_map,
            ),
            crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
                current_session_generation: bridge.exploration_producer_epoch.clone(),
            },
        )
        .await;
        assert_eq!(
            outcome.state.status, "ACCEPTED",
            "{}",
            outcome.state.message
        );
        let lease = outcome.lease.expect("accepted goal lease");
        (bridge, backend, lease)
    }

    async fn start_cancel(
        goal_id: &str,
    ) -> (
        ExploreNavigationBridge,
        RecordingBackend,
        crate::nav_runtime::GoalLease,
        crate::explore_runtime::ChildNavigationCancelTicket,
    ) {
        let (bridge, backend, lease) = start_goal(goal_id).await;
        let expected = exploration_child_lease(&lease);
        let ticket = crate::explore_runtime::ExploreNavigationSink::cancel(&bridge, &expected)
            .await
            .expect("exploration cancellation ticket");
        (bridge, backend, lease, ticket)
    }

    async fn wait_for_cancel_count(backend: &RecordingBackend, expected_minimum: usize) {
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                if backend.cancel_count() >= expected_minimum {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        })
        .await
        .expect("exact cancellation retry was not published");
    }

    fn assert_exact_cancels(
        backend: &RecordingBackend,
        lease: &crate::nav_runtime::GoalLease,
        cancel_generation: u64,
    ) {
        for cancel in backend.decoded_cancels() {
            assert_eq!(cancel.session_id, lease.session_id);
            assert_eq!(cancel.owner_kind, lease.owner_kind);
            assert_eq!(cancel.parent_request_id, lease.parent_request_id);
            assert_eq!(cancel.goal_generation, lease.goal_generation as i64);
            assert_eq!(
                cancel.producer_runtime_generation,
                lease.producer_runtime_generation as i64
            );
            assert_eq!(cancel.cancel_generation, cancel_generation as i64);
        }
    }

    async fn assert_retry_stopped(backend: &RecordingBackend) {
        tokio::time::sleep(Duration::from_millis(50)).await;
        let settled_count = backend.cancel_count();
        tokio::time::sleep(EXPLORATION_CANCEL_RETRY_INTERVAL + Duration::from_millis(100)).await;
        assert_eq!(backend.cancel_count(), settled_count);
    }

    #[tokio::test]
    async fn phase_a_rejects_correlated_cancel_retry_registration_fail_closed() {
        let (bridge, _backend, lease) = start_goal("phase-a-cancel-retry").await;
        let (control, _registration_rx) = GeometryRuntimeTaskControl::channel();
        control.close_admission();
        bridge.set_geometry_task_control_for_tests(Some(control));

        let error = crate::explore_runtime::ExploreNavigationSink::cancel(
            &bridge,
            &exploration_child_lease(&lease),
        )
        .await
        .expect_err("Phase A must reject an unowned correlated retry");
        assert!(error.contains("retry admission failed"));
        assert_eq!(
            bridge.nav.cancel_snapshot().phase,
            crate::nav_runtime::CancelPhase::Pending
        );
    }

    #[tokio::test]
    async fn exploration_cancel_retries_exact_transaction_until_zeroed() {
        let (bridge, backend, lease, ticket) = start_cancel("retry-until-zeroed").await;
        wait_for_cancel_count(&backend, 3).await;
        assert_exact_cancels(&backend, &lease, ticket.cancel_generation);

        let zeroed = bridge
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: ticket.cancel_generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "retry test zeroed".to_string(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        assert_retry_stopped(&backend).await;
    }

    #[tokio::test]
    async fn exploration_cancel_retry_stops_after_failed_ack() {
        let (bridge, backend, lease, ticket) = start_cancel("retry-until-failed").await;
        wait_for_cancel_count(&backend, 2).await;
        let failed = bridge
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: ticket.cancel_generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Failed,
                message: "retry test failed".to_string(),
            })
            .await;
        assert_eq!(failed.state.status, "FAILED");
        assert_retry_stopped(&backend).await;
    }

    #[tokio::test]
    async fn exploration_cancel_receipt_survives_zeroed_active_lease_clear() {
        let (bridge, backend, lease) = start_goal("receipt-after-zeroed").await;
        let outcome = crate::nav_runtime::cancel(
            &bridge.facade_handle(),
            &bridge.nav,
            crate::nav_runtime::CancelGoalInput {
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                producer_runtime_generation: lease.producer_runtime_generation,
            },
        )
        .await;
        assert_eq!(outcome.state.status, "PENDING");
        let expected = exploration_child_lease(&lease);
        let recovery = crate::explore_runtime::ExploreNavigationSink::recover_cancel_receipt(
            &bridge, &expected,
        )
        .expect("exact pending cancellation receipt");
        assert!(recovery.failure.is_none());
        let ticket = recovery.ticket;
        wait_for_cancel_count(&backend, 2).await;
        assert_exact_cancels(&backend, &lease, ticket.cancel_generation);

        let zeroed = bridge
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: ticket.cancel_generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "receipt recovery test zeroed".to_string(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        assert!(bridge.nav.active_goal_lease().await.is_none());
        let zeroed_recovery =
            crate::explore_runtime::ExploreNavigationSink::recover_cancel_receipt(
                &bridge, &expected,
            )
            .expect("exact zeroed cancellation receipt");
        assert!(zeroed_recovery.failure.is_none());
        assert_eq!(zeroed_recovery.ticket, ticket);

        let mut wrong = expected;
        wrong.goal_generation = wrong.goal_generation.saturating_add(1);
        assert!(
            crate::explore_runtime::ExploreNavigationSink::recover_cancel_receipt(&bridge, &wrong)
                .is_none()
        );
    }

    #[tokio::test]
    async fn wavefront_intent_rejects_map_name_changed_before_bridge_submit() {
        let (bridge, backend) = configured_bridge().await;
        let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
        let submit_entered = Arc::new(tokio::sync::Notify::new());
        let submit_release = Arc::new(tokio::sync::Notify::new());
        explore.set_navigation_sink(Arc::new(DelayedExploreBridge {
            inner: bridge.clone(),
            submit_entered: submit_entered.clone(),
            submit_release: submit_release.clone(),
        }));
        explore
            .ingest_committed_costmap_snapshot(frontier_costmap(), test_committed_map_identity(1))
            .await
            .expect("committed frontier costmap");
        assert_eq!(
            explore.start("cross-map", "prefer_wavefront").await.status,
            crate::explore_runtime::STATUS_ACCEPTED
        );

        let tick = {
            let explore = explore.clone();
            tokio::spawn(async move { explore.tick((2.0, 2.0)).await })
        };
        tokio::time::timeout(Duration::from_secs(2), submit_entered.notified())
            .await
            .expect("wavefront intent did not reach delayed submit");
        let mut hallway = bridge
            .map_revision_latest_tx
            .borrow()
            .clone()
            .expect("office map revision");
        assert_eq!(hallway.map_name, "office");
        hallway.map_name = "hallway".to_string();
        bridge.map_revision_latest_tx.send_replace(Some(hallway));
        submit_release.notify_one();

        let snapshot = tick.await.expect("wavefront tick task");
        assert_eq!(snapshot.status, crate::explore_runtime::STATUS_FAILED);
        assert!(snapshot.message.contains("active map name changed"));
        assert!(bridge.nav.active_goal_lease().await.is_none());
        assert_eq!(backend.frames.lock().expect("recorded frames").len(), 0);
    }

    #[tokio::test]
    async fn exact_zero_releases_exploration_while_telemetry_cache_is_busy() {
        let (bridge, backend) = configured_bridge().await;
        let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
        explore.set_navigation_sink(Arc::new(bridge.clone()));
        explore
            .ingest_committed_costmap_snapshot(frontier_costmap(), test_committed_map_identity(1))
            .await
            .expect("committed frontier costmap");
        assert_eq!(
            explore
                .start("telemetry-lock", "prefer_wavefront")
                .await
                .status,
            crate::explore_runtime::STATUS_ACCEPTED
        );
        let active = explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, crate::explore_runtime::STATUS_ACTIVE);
        let child_task_id = active.child_task_id.clone();
        let child_lease = active.child_lease.expect("active exploration child lease");

        let telemetry_guard = bridge.state.write().await;
        let cached_goal_before = telemetry_guard.nav_state.goal_id.clone();
        let cached_status_before = telemetry_guard.nav_state.status.clone();
        let stop_task = {
            let explore = explore.clone();
            tokio::spawn(async move { explore.stop("telemetry-lock").await })
        };
        wait_for_cancel_count(&backend, 1).await;
        let pending = bridge.nav.cancel_snapshot();
        assert_eq!(pending.phase, crate::nav_runtime::CancelPhase::Pending);
        let nav_lease = pending.lease.as_ref().expect("pending navigation lease");
        assert_eq!(exploration_child_lease(nav_lease), child_lease);

        let applied = apply_typed_navigation_feedback(
            &bridge.nav,
            NavigationUpdate {
                producer_runtime_generation: nav_lease.producer_runtime_generation,
                state: NavState {
                    status: "CANCELLED".to_string(),
                    message: "exact guard zero while telemetry cache is busy".to_string(),
                    goal_id: nav_lease.session_id.clone(),
                    timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                    ..NavState::default()
                },
                feedback: NavigationFeedback {
                    cancel_generation: pending.generation,
                    actuator_zeroed: true,
                    zero_ack_id: "telemetry-lock-zero".to_string(),
                    zero_applied_monotonic_ns: crate::explore_runtime::now_monotonic_nanos(),
                    owner_kind: nav_lease.owner_kind,
                    parent_request_id: nav_lease.parent_request_id.clone(),
                    goal_generation: nav_lease.goal_generation,
                    goal_payload_sha256: nav_lease.goal_payload_sha256,
                    path_revision: nav_lease.path_revision,
                    path_payload_sha256: nav_lease.path_payload_sha256,
                    ..NavigationFeedback::default()
                },
            },
        )
        .await
        .expect("exact typed ZEROED feedback");
        assert_eq!(applied.state.status, "CANCELLED");
        assert!(bridge.nav.active_goal_lease().await.is_none());

        let mut nav_updates = bridge.nav_state_tx.subscribe();
        tokio::time::timeout(
            Duration::from_millis(100),
            fanout_navigation_update(&bridge.state, &bridge.nav_state_tx, &explore, applied),
        )
        .await
        .expect("ownership terminal must bypass the busy telemetry cache");
        let broadcast = nav_updates.try_recv().expect("ordered terminal broadcast");
        assert_eq!(broadcast.state.status, "CANCELLED");
        assert_eq!(broadcast.state.goal_id, child_task_id);
        assert_eq!(telemetry_guard.nav_state.goal_id, cached_goal_before);
        assert_eq!(telemetry_guard.nav_state.status, cached_status_before);

        let stopped = tokio::time::timeout(Duration::from_secs(1), stop_task)
            .await
            .expect("exploration response-loss recovery deadline")
            .expect("exploration Stop task");
        assert_eq!(stopped.status, crate::explore_runtime::STATUS_CANCELLED);
        assert!(stopped.child_task_id.is_empty());
        assert!(!explore.blocks_mapping_mutation().await);
        drop(telemetry_guard);
    }

    #[tokio::test]
    async fn exploration_cancel_retry_stops_after_existing_timeout() {
        let (bridge, backend, _lease, _ticket) = start_cancel("retry-until-timeout").await;
        wait_for_cancel_count(&backend, 2).await;
        bridge.nav.expire_cancel_for_test();

        let timed_out = bridge
            .nav
            .poll_cancel_timeout()
            .await
            .expect("pending cancellation should reach its existing timeout");
        assert_eq!(timed_out.state.status, "FAILED");
        assert_eq!(timed_out.state.message, "CANCEL_ZERO_ACK_TIMEOUT");
        assert_retry_stopped(&backend).await;
    }

    #[tokio::test]
    async fn exploration_cancel_retry_stays_stopped_when_next_goal_replaces_zeroed_lease() {
        let (bridge, backend, lease, ticket) = start_cancel("retry-before-replacement").await;
        wait_for_cancel_count(&backend, 2).await;
        let producer_epoch = bridge
            .exploration_producer_epoch
            .load(std::sync::atomic::Ordering::Acquire);
        let blocked_replacement = crate::nav_runtime::admit_goal_intent(
            &crate::nav_runtime::NavAdmissionContext::new(
                bridge.mapping_lifecycle_lock.clone(),
                bridge.committed_map_authority.clone(),
                bridge.map_revision_latest_tx.subscribe(),
            ),
            &crate::nav_runtime::NavAdmissionIo::new(
                bridge.state.clone(),
                bridge.transport.clone(),
                bridge.transport_backend_name,
                bridge.lcm_backend_available,
            ),
            &bridge.nav,
            test_wavefront_goal_intent(
                "blocked-replacement-child",
                "exploration-retry-parent",
                producer_epoch,
                bridge.nav.cancel_snapshot().generation,
                test_committed_map_identity(1),
            ),
            crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
                current_session_generation: bridge.exploration_producer_epoch.clone(),
            },
        )
        .await;
        assert_eq!(blocked_replacement.state.status, "FAILED");
        assert!(blocked_replacement
            .state
            .message
            .contains("GOAL_LEASE_BUSY"));
        wait_for_cancel_count(&backend, 3).await;

        let zeroed = bridge
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: ticket.cancel_generation,
                goal_id: lease.session_id.clone(),
                owner_kind: lease.owner_kind,
                parent_request_id: lease.parent_request_id.clone(),
                goal_generation: lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "replacement fence zeroed".to_string(),
            })
            .await;
        assert_eq!(zeroed.state.status, "CANCELLED");
        bridge
            .nav
            .observe_stack_state(NavState {
                status: "IDLE".to_string(),
                message: "replacement heartbeat".to_string(),
                timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                ..NavState::default()
            })
            .await;
        bridge
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 2,
                map_frame: "map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("replacement TF observation");
        bridge
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 2,
                map_revision: 2,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("replacement planner observation");
        bridge
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 2,
                ready: true,
            })
            .expect("replacement motion sink observation");
        let replacement_map = test_committed_map_identity(2);
        *bridge
            .committed_map_authority
            .lock()
            .expect("replacement committed map authority") = Some(replacement_map.clone());
        let replacement = crate::nav_runtime::admit_goal_intent(
            &crate::nav_runtime::NavAdmissionContext::new(
                bridge.mapping_lifecycle_lock.clone(),
                bridge.committed_map_authority.clone(),
                bridge.map_revision_latest_tx.subscribe(),
            ),
            &crate::nav_runtime::NavAdmissionIo::new(
                bridge.state.clone(),
                bridge.transport.clone(),
                bridge.transport_backend_name,
                bridge.lcm_backend_available,
            ),
            &bridge.nav,
            test_wavefront_goal_intent(
                "replacement-child",
                "exploration-retry-parent",
                producer_epoch,
                bridge.nav.cancel_snapshot().generation,
                replacement_map,
            ),
            crate::nav_runtime::GoalProducerEpochAuthority::Wavefront {
                current_session_generation: bridge.exploration_producer_epoch.clone(),
            },
        )
        .await;
        assert_eq!(
            replacement.state.status, "ACCEPTED",
            "{}",
            replacement.state.message
        );
        assert_retry_stopped(&backend).await;
    }
}

#[cfg(test)]
mod payload_sanity_tests {
    //! 防御性 sanity cap 测试(2026-07-08 rework17 修复后)。
    //!
    //! 真实机 175 GB alloc 失败的根因推测:上游 LCM 帧 `count` 字段因 uninit
    //! memory 或协议异常被解析成 `usize::MAX` 级别,触发 `Vec::with_capacity`
    //! 申请上百 GB。本模块给 `parse_tf_payload` / `parse_occupancy_payload` /
    //! frontiers 路径加了 sanity cap,本测试用直接构造异常 payload 的方式
    //! 验证 cap 生效,异常帧返回 None 而不分配。

    use super::{
        parse_occupancy_payload, parse_pointcloud_payload, parse_tf_payload,
        FRONTIERS_MAX_ENTRIES_PER_FRAME, OCCUPANCY_MAX_DIMENSION, TF_MAX_ENTRIES_PER_FRAME,
    };
    use nuwax_codec::lcm::schema;

    /// count 字段填 0xFFFF_FFFF(usize 等价 ~ 18 EB)→ 必须被 sanity cap 拒绝
    #[test]
    fn parse_tf_payload_rejects_huge_count() {
        let payload = i32::MAX.to_be_bytes();
        assert!(
            parse_tf_payload(&payload).is_none(),
            "parse_tf_payload 应该拒绝 count > TF_MAX_ENTRIES_PER_FRAME 的异常帧"
        );
    }

    /// count 字段填 0 → 应正常解析为空 vec
    #[test]
    fn parse_tf_payload_accepts_zero_count() {
        let payload = schema::encode_tf_message_body(&schema::TfMessage::default())
            .expect("encode empty TFMessage");
        let result = parse_tf_payload(&payload).expect("empty TFMessage 应可解析");
        assert!(result.is_empty());
    }

    /// width = u32::MAX → 应被 OCCUPANCY_MAX_DIMENSION 拒绝
    #[test]
    fn parse_occupancy_payload_rejects_huge_dimension() {
        let payload = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            info: schema::MapMetaData {
                width: i32::MAX,
                height: i32::MAX,
                ..schema::MapMetaData::default()
            },
            ..schema::OccupancyGrid::default()
        })
        .expect("encode hostile OccupancyGrid");
        assert!(
            parse_occupancy_payload(&payload).is_none(),
            "parse_occupancy_payload 应该拒绝 width/height > OCCUPANCY_MAX_DIMENSION 的异常帧"
        );
    }

    /// 合法 width*height=16 + data_len=16 happy path
    #[test]
    fn parse_occupancy_payload_accepts_well_formed() {
        let payload = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                stamp_sec: 12,
                stamp_nsec: 34,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.05,
                width: 4,
                height: 4,
                ..schema::MapMetaData::default()
            },
            data: vec![0; 16],
        })
        .expect("encode OccupancyGrid");
        let result = parse_occupancy_payload(&payload).expect("happy path 应解析成功");
        assert_eq!(result.width, 4);
        assert_eq!(result.height, 4);
        assert_eq!(result.data.len(), 16);
        assert_eq!(result.frame_id, "map");
        assert_eq!(result.timestamp_ns, 12_000_000_034);
    }

    /// cap 常量本身在物理合理范围内(const-eval 不会触发 lint)
    #[test]
    fn sanity_caps_are_in_physically_reasonable_range() {
        // 这些都是编译期常量,clippy 会把 assert!(const_expr) 警告为可优化;
        // 这里改用 const_assertions 风格:`const _: () = assert!(...);` 是
        // 编译期验证,运行时无开销。
        const _: () = assert!(TF_MAX_ENTRIES_PER_FRAME > 0 && TF_MAX_ENTRIES_PER_FRAME < 1_000_000);
        const _: () = assert!(
            FRONTIERS_MAX_ENTRIES_PER_FRAME > 0 && FRONTIERS_MAX_ENTRIES_PER_FRAME < 1_000_000
        );
        const _: () = assert!(OCCUPANCY_MAX_DIMENSION > 0 && OCCUPANCY_MAX_DIMENSION < 100_000);
        const _: () =
            assert!((OCCUPANCY_MAX_DIMENSION as usize).saturating_pow(2) < 100 * 1024 * 1024);
        const _: () = assert!(
            crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS > 0
                && crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS < 1_000_000
        );
        const _: () = assert!(
            crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES >= 12
                && crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES < 4_096
        );
    }

    /// point_step = 1 (untrusted 上游 frame 异常)→ 必须被 sanity cap 拒绝
    /// (point_step < 12 触发)
    #[test]
    fn parse_pointcloud_payload_rejects_tiny_point_step() {
        let payload = pointcloud_body(1, vec![0; 1]);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "parse_pointcloud_payload 应该拒绝 point_step < 12 的异常帧"
        );
    }

    /// point_step = u32::MAX → 必须被 POINTCLOUD_MAX_POINT_STEP_BYTES 拒绝
    #[test]
    fn parse_pointcloud_payload_rejects_huge_point_step() {
        let payload = pointcloud_body(4_096, vec![0; 4_096]);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "parse_pointcloud_payload 应该拒绝 point_step > POINTCLOUD_MAX_POINT_STEP_BYTES 的异常帧"
        );
    }

    /// point_step = 12 + 24 bytes raw → 应正常解析 (happy path)
    #[test]
    fn parse_pointcloud_payload_accepts_well_formed() {
        let payload = pointcloud_body(12, vec![0; 24]);
        let parsed = parse_pointcloud_payload(&payload).expect("happy path 应解析成功");
        assert_eq!(parsed.point_step, 12);
        assert_eq!(parsed.data.len(), 24);
        assert_eq!(parsed.frame_id, "map");
        assert_eq!(parsed.timestamp_ns, 12_345_678_901);
    }

    #[test]
    fn parse_pointcloud_payload_rejects_more_than_single_frame_cap() {
        let data = vec![0; (crate::pointcloud::POINTCLOUD_MAX_FRAME_POINTS + 1) * 12];
        let payload = pointcloud_body(12, data);
        assert!(
            parse_pointcloud_payload(&payload).is_none(),
            "valid PointCloud2 layout above the physical single-frame cap must be rejected"
        );
    }

    fn pointcloud_body(point_step: i32, data: Vec<u8>) -> Vec<u8> {
        let width = i32::try_from(data.len() / point_step.max(1) as usize).unwrap();
        schema::encode_point_cloud2_body(&schema::PointCloud2 {
            header: schema::Header {
                stamp_sec: 12,
                stamp_nsec: 345_678_901,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            height: 1,
            width,
            fields: vec![
                schema::PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                schema::PointField {
                    name: "y".into(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                schema::PointField {
                    name: "z".into(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
            ],
            point_step,
            row_step: point_step.saturating_mul(width),
            data,
            is_dense: true,
            ..schema::PointCloud2::default()
        })
        .expect("encode PointCloud2")
    }
}

#[cfg(test)]
mod exploration_lcm_smoke_tests {
    use super::*;
    use crate::explore_runtime::{ExploreCostmapSnapshot, ExploreStartConfig};
    use crate::nav_runtime::{MotionSinkObservation, PlannerInputObservation, TfChainObservation};
    use futures::StreamExt;
    use nuwax_exploration::ExplorationBoundary;
    use std::sync::atomic::{AtomicBool, AtomicU64};

    fn smoke_costmap(_fixture: u64, map_frame: &str) -> ExploreCostmapSnapshot {
        let mut grid = OccupancyGrid2D {
            resolution: 0.5,
            origin: Point2 { x: 0.0, y: 0.0 },
            width: 12,
            height: 12,
            cells: vec![OccupancyCell::Unknown; 144],
        };
        for y in 2..=7 {
            for x in 2..=7 {
                grid.set(x, y, OccupancyCell::Free);
            }
        }
        ExploreCostmapSnapshot {
            grid,
            frame_id: map_frame.to_string(),
        }
    }

    #[test]
    fn exploration_submit_rejection_classification_is_typed() {
        use crate::explore_runtime::ChildNavigationSubmitError;
        use crate::nav_runtime::{
            NavigationReadinessCode, SetGoalRejectReason, SetGoalRetryReason,
        };

        assert!(matches!(
            classify_exploration_submit_rejection(
                Some(SetGoalRetryReason::NavigationNotReady(
                    NavigationReadinessCode::MotionSinkNotReady,
                )),
                None,
                "motion sink unavailable".to_string(),
            ),
            ChildNavigationSubmitError::RetryableReadiness(_)
        ));
        assert!(matches!(
            classify_exploration_submit_rejection(None, None, "invalid boundary".to_string(),),
            ChildNavigationSubmitError::Rejected(_)
        ));
        assert!(matches!(
            classify_exploration_submit_rejection(
                None,
                Some(SetGoalRejectReason::ActiveMapNameChanged),
                "active map changed".to_string(),
            ),
            ChildNavigationSubmitError::SessionInvalidated(_)
        ));
        assert!(matches!(
            classify_exploration_submit_rejection(
                Some(SetGoalRetryReason::NavigationNotReady(
                    NavigationReadinessCode::ProfileUnsupported,
                )),
                None,
                "unsupported profile".to_string(),
            ),
            ChildNavigationSubmitError::Rejected(_)
        ));
    }

    async fn exploration_channels_round_trip_over_real_lcm(map_frame: &'static str) {
        const MAP_REVISION: u64 = 41;
        const PARENT_REQUEST: &str = "smoke-explore";

        let provider = test_lcm_url();
        assert!(provider.starts_with("udpm://239.255.76.67:"));
        assert!(provider.ends_with("?ttl=1"));
        let publisher = LcmTransport::new_real(&provider)
            .expect("real LCM publisher context; install liblcm-dev to run this smoke");
        let subscriber =
            LcmTransport::new_real(&provider).expect("independent real LCM subscriber context");

        let frontier_topic = LcmTopic::new("/explore/frontiers").expect("frontier topic");
        let frontier_type =
            LcmMessageType::new("nuwax_msgs.FrontierList").expect("FrontierList type");
        let (goal_topic, goal_type) =
            crate::nav_runtime::goal_envelope_binding().expect("canonical goal binding");
        assert_eq!(goal_topic.channel(), "/nav/goal");
        assert_eq!(goal_type.fqn(), "nuwax_msgs.GoalEnvelope");
        assert_eq!(
            frontier_type.fingerprint(),
            schema::FRONTIER_LIST_FINGERPRINT
        );
        assert_eq!(goal_type.fingerprint(), schema::GOAL_ENVELOPE_FINGERPRINT);

        let mut frontier_frames = subscriber
            .subscribe(frontier_topic.clone(), Some(frontier_type.clone()))
            .await;
        let mut goal_frames = subscriber
            .subscribe(goal_topic.clone(), Some(goal_type.clone()))
            .await;
        tokio::time::sleep(Duration::from_millis(50)).await;

        publisher
            .publish(crate::typed_codec::build_typed_frame(
                frontier_topic.clone(),
                LcmMessageType::new("std_msgs.Bool").expect("registered wrong type"),
                schema::encode_bool_body(&schema::Bool { data: true }),
                1,
            ))
            .await
            .expect("publish wrong-type control frame");
        assert!(
            tokio::time::timeout(Duration::from_millis(150), frontier_frames.next())
                .await
                .is_err(),
            "FrontierList typed subscriber accepted a std_msgs.Bool frame"
        );

        let state = Arc::new(RwLock::new(SharedState {
            pose: PoseState {
                x: 2.0,
                y: 2.0,
                qw: 1.0,
                frame_id: map_frame.to_string(),
                timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                ..PoseState::default()
            },
            map: MapState {
                width: 12,
                height: 12,
                resolution: 0.5,
                frame_id: map_frame.to_string(),
                revision: MAP_REVISION,
                timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                ..MapState::default()
            },
            ..SharedState::default()
        }));
        let nav = crate::nav_runtime::NavRuntimeState::from_env();
        nav.observe_stack_state(NavState {
            status: "IDLE".to_string(),
            message: "real nav-stack heartbeat".to_string(),
            timestamp_ns: crate::explore_runtime::now_unix_nanos(),
            ..NavState::default()
        })
        .await;
        nav.observe_tf_chain(TfChainObservation {
            generation: 1,
            map_frame: map_frame.to_string(),
            odom_frame: "odom".to_string(),
            base_frame: "base_link".to_string(),
            map_to_odom: true,
            odom_to_base: true,
        })
        .expect("TF observation");
        nav.observe_planner_input(PlannerInputObservation {
            generation: 1,
            map_revision: MAP_REVISION,
            map_frame: map_frame.to_string(),
            pose_frame: map_frame.to_string(),
            ready: true,
        })
        .expect("planner observation");
        nav.observe_motion_sink(MotionSinkObservation {
            generation: 1,
            ready: true,
        })
        .expect("guard-backed motion sink observation");

        let (cloud_map_tx, _) = broadcast::channel(4);
        let (nav_state_tx, _) = broadcast::channel(8);
        let (frontiers_tx, _) = broadcast::channel(4);
        let (map_revision_tx, _) = broadcast::channel(4);
        let (map_revision_latest_tx, _) = watch::channel(None);
        let committed_map = schema::CommittedMapIdentity {
            session_id: "smoke-map".to_string(),
            session_generation: 1,
            revision: MAP_REVISION as i64,
            payload_sha256: crate::map_checksum::sha256_bytes(&[]),
            frame_id: map_frame.to_string(),
        };
        map_revision_latest_tx.send_replace(Some(test_map_revision_authority(
            &committed_map.session_id,
            "smoke-map",
            map_frame,
            MAP_REVISION,
        )));
        let explore =
            crate::explore_runtime::ExploreRuntimeHandle::with_wavefront_map_frame(map_frame)
                .expect("configured exploration runtime");
        let bridge = ExploreNavigationBridge {
            state: state.clone(),
            expected_map_frame: Arc::from(map_frame),
            cloud_map_tx,
            nav_state_tx,
            frontiers_tx: frontiers_tx.clone(),
            map_revision_tx,
            map_revision_latest_tx,
            explore_projection_gate: Arc::new(tokio::sync::Mutex::new(
                ExploreProjectionGate::default(),
            )),
            committed_map_authority: Arc::new(std::sync::Mutex::new(Some(committed_map.clone()))),
            mapping_lifecycle_lock: Arc::new(tokio::sync::Mutex::new(())),
            transport: publisher.clone(),
            transport_backend_name: "real-lcm",
            lcm_backend_available: true,
            cmd_vel_sink_ready: Arc::new(AtomicBool::new(true)),
            nav: nav.clone(),
            exploration_producer_epoch: explore.producer_epoch_projection(),
            runtime_observation_generation: Arc::new(AtomicU64::new(1)),
            orphan_cancellations: explore.orphan_cancellation_registry(),
        };
        explore.set_navigation_sink(Arc::new(bridge.clone()));
        spawn_frontier_fanout_in_frame(
            explore.clone(),
            state.clone(),
            frontiers_tx,
            publisher.clone(),
            Some((frontier_topic.clone(), frontier_type.clone())),
            Arc::from(map_frame),
        );

        assert!(explore
            .ingest_committed_costmap_snapshot(
                smoke_costmap(MAP_REVISION, map_frame),
                committed_map,
            )
            .await
            .expect("ingest smoke costmap"));
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2 { x: 0.0, y: 0.0 },
            Point2 { x: 5.0, y: 0.0 },
            Point2 { x: 5.0, y: 5.0 },
            Point2 { x: 0.0, y: 5.0 },
        ])
        .expect("valid exploration boundary");
        let started = explore
            .start_with_config(
                PARENT_REQUEST,
                ExploreStartConfig {
                    motion_admission: crate::nav_runtime::test_motion_admission(),
                    boundary: Some(boundary),
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        assert_eq!(started.status, "ACCEPTED", "{}", started.message);
        let active = explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, "ACTIVE", "{}", active.message);
        let frontier_snapshot = explore.frontier_snapshot();
        assert!(!frontier_snapshot.frontiers.is_empty());

        let frontier_frame = tokio::time::timeout(Duration::from_secs(3), async {
            loop {
                let frame = frontier_frames
                    .next()
                    .await
                    .expect("frontier stream closed")
                    .expect("frontier transport error");
                let mut generated_message = frontier_type.fingerprint().to_vec();
                generated_message.extend_from_slice(&frame.payload);
                let decoded = schema::decode_frontier_list(&generated_message)
                    .expect("decode generated FrontierList with fingerprint");
                if decoded.frontiers_length > 0 {
                    break (frame, decoded);
                }
            }
        })
        .await
        .expect("timed out waiting for non-empty canonical FrontierList");
        assert_eq!(frontier_frame.0.topic, frontier_topic);
        assert_eq!(frontier_frame.0.msg_type, frontier_type);
        assert_eq!(frontier_frame.1.frame_id, map_frame);
        assert_eq!(
            frontier_frame.1.frontiers.len(),
            schema::FRONTIER_LIST_CAPACITY
        );
        assert_eq!(
            frontier_frame.1.frontiers_length,
            frontier_snapshot.frontiers.len()
        );
        assert!(frontier_frame.1.stamp_sec > 0);
        assert!((0..1_000_000_000).contains(&frontier_frame.1.stamp_nsec));
        for (wire, source) in frontier_frame
            .1
            .frontiers
            .iter()
            .take(frontier_frame.1.frontiers_length)
            .zip(frontier_snapshot.frontiers.iter())
        {
            assert_eq!(wire.id, stable_frontier_id(&source.id));
            assert_eq!(
                wire.center,
                [source.pose.pose.position.x, source.pose.pose.position.y]
            );
            assert_eq!(
                wire.cell_count,
                source
                    .metadata
                    .get("cell_count")
                    .expect("frontier cell_count metadata")
                    .parse::<i32>()
                    .expect("numeric frontier cell_count")
            );
            assert_eq!(wire.note, source.producer);
        }

        let goal_frame = tokio::time::timeout(Duration::from_secs(3), goal_frames.next())
            .await
            .expect("timed out waiting for canonical GoalEnvelope")
            .expect("goal stream closed")
            .expect("goal transport error");
        let mut generated_goal = goal_type.fingerprint().to_vec();
        generated_goal.extend_from_slice(&goal_frame.payload);
        let goal = schema::decode_goal_envelope(&generated_goal)
            .expect("decode generated GoalEnvelope with fingerprint");
        let lease = nav
            .active_goal_lease()
            .await
            .expect("active exploration lease");
        assert_eq!(goal_frame.topic, goal_topic);
        assert_eq!(goal_frame.msg_type, goal_type);
        assert_eq!(
            goal.owner_kind,
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        );
        assert_eq!(goal.parent_request_id, PARENT_REQUEST);
        assert_eq!(goal.session_id, lease.session_id);
        assert_eq!(goal.goal_generation, lease.goal_generation as i64);
        assert!(goal.goal_generation > 0);
        assert_eq!(goal.payload_sha256, lease.goal_payload_sha256);
        assert_eq!(
            goal.payload_sha256,
            schema::goal_envelope_payload_sha256(&goal).expect("recompute goal hash")
        );
        assert_eq!(goal.goal.header.frame_id, map_frame);
        assert_eq!(goal.boundary_frame_id, map_frame);
        assert_eq!(
            goal.boundary_points,
            vec![
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 5.0, 0.0],
                [0.0, 5.0, 0.0],
            ]
        );

        let wrong_goal = nuwax_exploration::conversions::frontier_goal_to_navigation_goal(
            &frontier_snapshot.frontiers[0],
        );
        let committed_map = bridge
            .committed_map_authority
            .lock()
            .expect("smoke committed map authority")
            .clone()
            .expect("smoke committed map identity");
        let mut wrong_intent = crate::explore_runtime::build_wavefront_goal_intent(
            active.exploration_generation,
            nav.cancel_snapshot().generation,
            PARENT_REQUEST,
            "wrong-semantic-frame",
            &crate::nav_runtime::test_motion_admission(),
            &wrong_goal,
            None,
            committed_map,
        )
        .expect("otherwise-valid wrong-frame intent fixture");
        wrong_intent.goal.header.frame_id = "odom".to_string();
        wrong_intent.goal_sha256 = schema::goal_intent_goal_sha256(&wrong_intent.goal)
            .expect("rehash the deliberately wrong target frame");
        let rejection = crate::explore_runtime::ExploreNavigationSink::submit(
            &bridge,
            crate::explore_runtime::ChildNavigationRequest {
                parent_request_id: PARENT_REQUEST.to_string(),
                request_id: "wrong-semantic-frame".to_string(),
                expected_map_name: "smoke-map".to_string(),
                motion_admission: crate::nav_runtime::test_motion_admission(),
                frontier_goal_id: "wrong-semantic-frontier".to_string(),
                goal: wrong_goal,
                boundary: None,
                intent: wrong_intent,
            },
        )
        .await
        .expect_err("wrong frozen intent frame must fail closed before lease CAS");
        assert!(matches!(
            rejection,
            crate::explore_runtime::ChildNavigationSubmitError::Rejected(message)
                if message == "SET_GOAL_REJECTED[GOAL_INTENT_INVALID]: goal header, frame, timestamp, or pose is invalid"
        ));

        let shared = state.read().await;
        assert_eq!(shared.frontiers_revision, frontier_snapshot.revision);
        assert_eq!(shared.frontiers.len(), frontier_snapshot.frontiers.len());
        assert!(shared
            .frontiers
            .iter()
            .all(|frontier| frontier.frame_id == map_frame));
        drop(shared);

        drop(frontier_frames);
        drop(goal_frames);
        drop(explore);
        drop(subscriber);
        drop(publisher);
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn canonical_exploration_channels_round_trip_over_real_lcm() {
        exploration_channels_round_trip_over_real_lcm("map").await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn configured_map_frame_round_trips_over_real_exploration_channels() {
        exploration_channels_round_trip_over_real_lcm("robot/map").await;
    }
}
