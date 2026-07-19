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
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nuwax_codec::lcm::schema;
use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use nuwax_protocol::{LcmMessageType, LcmTopic};
use nuwax_tf::{make_transform, TfTree, TransformStamped};
use nuwax_transport::LcmTransport;
use thiserror::Error;
use tokio::sync::{broadcast, watch, RwLock};
use tracing::{error, info, warn};

use crate::mapping_adapter::{MappingAdapter, MappingAdapterConfig};
use crate::occupancy_shm::OccupancyGridShmWriter;
use crate::pointcloud::CloudMapShmWriter;
use crate::sequence_state::{sequence_state_path_from_env, SequenceStateStore};

const MAP_FANOUT_SINK_TIMEOUT: Duration = Duration::from_secs(2);

// Phase 4 (channel registry) + Phase 7 (readiness cache): 统一在这里导入,
// 避免每个函数 body 内部重复 `use crate::channel_registry::...`。
//
// 同时 `use crate::channel_registry;` 让 module 名进 scope,内部多处以
// `channel_registry::channel_for(...)` 限定调用,无需每次都写完整 `crate::` 路径。
use crate::channel_registry;
use crate::channel_registry::ChannelKey;

// ============================================================================
// 状态定义
// ============================================================================

/// Phase 1 fail-fast 错误：上游 dimos `nuwax-lcm-sys`/`nuwax-lcm` 尚不存在
///（`nuwax-transport/real-lcm` feature 仍为空桩），所以 `real-lcm` backend
/// 启动期直接拒绝且不做降级。错误码 `LCM_BACKEND_UNAVAILABLE`
/// 必须在 stderr 中可见供 start 脚本与运维 grep 识别。
#[derive(Debug, Error)]
#[error(
    "LCM_BACKEND_UNAVAILABLE: NUWAX_LCM_BACKEND=real-lcm 不可用 —— 上游 dimos \
     nuwax-transport/real-lcm feature 仍为空桩 (Rust LCM binding 未实装 + \
     fingerprint 未切 MurmurHash64A)。详见 feedback_phase1.md。"
)]
pub struct RealLcmBackendUnavailable;

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

/// 动态 /tf 兼容镜像存活时长（纳秒）。stamp>0 且超过 TTL 的边在合成位姿前被裁剪，
/// 避免运行时重启遗留陈旧 map→odom 污染 BFS。stamp==0 不过期。
/// 主查询路径使用 [`TfTree`] 的同一 10 秒历史窗口。
const TF_ENTRY_TTL_NS: u64 = 10_000_000_000;

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
    pub planner_map_revision: u64,
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
    pub map_version_id: String,
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
                planner_map_revision: lease.as_ref().map_or(0, |lease| lease.map_revision),
                path_revision: lease.as_ref().map_or(0, |lease| lease.path_revision),
                goal_payload_sha256: lease
                    .as_ref()
                    .map_or([0; 32], |lease| lease.goal_payload_sha256),
                path_payload_sha256: lease
                    .as_ref()
                    .map_or([0; 32], |lease| lease.path_payload_sha256),
                map_version_id: lease
                    .as_ref()
                    .map_or_else(String::new, |lease| lease.map_version_id.clone()),
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

/// RALPLAN §Phase M #3: occupancy grid revision 事件。
///
/// `mapping_adapter::MappingAdapter::build_and_publish` 在 commit 新 revision
/// 后构造本结构并通过 `broadcast::Sender<MapRevisionEvent>` 写入兼容入口；
/// coalescer 将其收敛为 latest-only `watch`，SHM/LCM/explore/gRPC 分别消费，
/// 避免慢下游阻塞其它链路。`data` 使用 `Arc<[u8]>`，跨下游 clone 不复制地图。
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
    pub map_version_id: String,
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
    pub size: u64,
    pub schema_version: u32,
    pub checksum: String,
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
        Self {
            session_id: session.session_id.clone(),
            map_version_id: session.map_version_id.clone(),
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
            checksum: crate::map_checksum::sha256_hex(data.as_ref()),
            last_seen_ms: (map.timestamp_ns / 1_000_000) as i64,
            schema_version: 1,
            data,
        }
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

fn is_global_map_frame(frame_id: &str) -> bool {
    normalize_frame(frame_id) == "map"
}

fn encode_occupancy_revision(event: &MapRevisionEvent) -> Result<Vec<u8>, String> {
    if !is_global_map_frame(&event.frame_id) {
        return Err(format!(
            "global occupancy fanout requires frame=map, received {}",
            event.frame_id
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
            frame_id: "map".to_string(),
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

fn exploration_costmap_from_revision(
    event: &MapRevisionEvent,
) -> Result<crate::explore_runtime::ExploreCostmapSnapshot, String> {
    if !is_global_map_frame(&event.frame_id) {
        return Err(format!(
            "global exploration requires frame=map, received {}",
            event.frame_id
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
        frame_id: "map".to_string(),
        map_version_id: event.map_version_id.clone(),
        map_revision: event.revision,
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
        && event.map_version_id == snapshot.map_version_id
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
}

impl GeometryRuntimeHandle {
    pub fn subscribe_map_revisions(&self) -> watch::Receiver<Option<MapRevisionEvent>> {
        self.map_revision_latest_tx.subscribe()
    }

    /// Publish a zero-point cloud-map revision for a newly started mapping
    /// generation. Missing/failed cloud-map SHM is fail-soft and never blocks
    /// the authoritative 2D MappingSession lifecycle.
    pub(crate) async fn begin_cloud_map_session(
        &self,
        session_generation: u64,
    ) -> Option<CloudMapNotification> {
        let update = {
            let mut guard = self.cloud_map_writer.lock();
            let writer = guard.as_mut()?;
            let (revision, point_count, voxel_size) = writer.begin_session(session_generation)?;
            CloudMapNotification {
                revision,
                point_count,
                voxel_size,
                session_generation,
            }
        };
        {
            let mut shared = self.state.write().await;
            shared.cloud_map_revision = update.revision;
        }
        let _ = self.cloud_map_tx.send(update.clone());
        Some(update)
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

    pub async fn shutdown_explore_scheduler(&self) {
        if let Some(scheduler) = self.explore_scheduler.lock().await.take() {
            scheduler.shutdown().await;
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
    let map_frame = normalize_frame(&transform.map_frame_id);
    let odom_frame = normalize_frame(&transform.odom_frame_id);
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
    let (odom_quaternion, odom_yaw) = normalized_quaternion_and_yaw(odom_pose)?;
    let stamp_sec = i32::try_from(odom_pose.timestamp_ns / 1_000_000_000)
        .map_err(|_| "odometry timestamp seconds exceed int32".to_string())?;
    let stamp_nsec = (odom_pose.timestamp_ns % 1_000_000_000) as i32;
    let map_odom_quaternion = yaw_quaternion(transform.yaw_radians);
    let (sin_yaw, cos_yaw) = transform.yaw_radians.sin_cos();
    let map_pose_position = [
        cos_yaw * odom_pose.x - sin_yaw * odom_pose.y + transform.translation[0],
        sin_yaw * odom_pose.x + cos_yaw * odom_pose.y + transform.translation[1],
        odom_pose.z + transform.translation[2],
    ];
    let map_pose_quaternion = yaw_quaternion(transform.yaw_radians + odom_yaw);
    let transforms = [
        TransformEntry {
            parent: map_frame.clone(),
            child: odom_frame.clone(),
            tx: transform.translation[0],
            ty: transform.translation[1],
            tz: transform.translation[2],
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
        seq: i32::try_from(transform.generation).unwrap_or(i32::MAX),
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
                    translation: transform.translation,
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
            x: map_pose_position[0],
            y: map_pose_position[1],
            z: map_pose_position[2],
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
/// - `true` + liblcm 不可用 → 启动期进程直接 exit 1 + `LCM_BACKEND_UNAVAILABLE`。
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
/// `NUWAX_REAL_LCM_REQUIRED` 只决定创建失败后的退出策略，不能跳过真实 backend
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
        if !is_global_map_frame(&pose.frame_id) || pose.timestamp_ns == 0 {
            return Err(format!(
                "map-frame pose unavailable (frame={}, timestamp_ns={})",
                pose.frame_id, pose.timestamp_ns
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
            frame_id: "map".to_string(),
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
        map_version_id: lease.map_version_id.clone(),
        map_revision: lease.map_revision,
        goal_payload_sha256: lease.goal_payload_sha256,
    }
}

#[derive(Clone)]
struct ExploreNavigationBridge {
    state: Arc<RwLock<SharedState>>,
    cloud_map_tx: broadcast::Sender<CloudMapNotification>,
    nav_state_tx: broadcast::Sender<NavigationUpdate>,
    frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    map_revision_tx: broadcast::Sender<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    transport: LcmTransport,
    transport_backend_name: &'static str,
    lcm_backend_available: bool,
    cmd_vel_sink_ready: Arc<std::sync::atomic::AtomicBool>,
    nav: crate::nav_runtime::NavRuntimeState,
    runtime_observation_generation: Arc<std::sync::atomic::AtomicU64>,
}

impl ExploreNavigationBridge {
    fn facade_handle(&self) -> GeometryRuntimeHandle {
        GeometryRuntimeHandle {
            state: self.state.clone(),
            cloud_map_tx: self.cloud_map_tx.clone(),
            cloud_map_writer: Arc::new(parking_lot::Mutex::new(None)),
            nav_state_tx: self.nav_state_tx.clone(),
            frontiers_tx: self.frontiers_tx.clone(),
            map_revision_tx: self.map_revision_tx.clone(),
            map_revision_latest_tx: self.map_revision_latest_tx.clone(),
            transport: self.transport.clone(),
            transport_backend_name: self.transport_backend_name,
            lcm_backend_available: self.lcm_backend_available,
            cmd_vel_sink_ready: self.cmd_vel_sink_ready.clone(),
            explore: crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront(),
            nav: self.nav.clone(),
            mapping_adapter: None,
            mapping_lifecycle_lock: Arc::new(tokio::sync::Mutex::new(())),
            occupancy_shm: None,
            occupancy_shm_path: None,
            occupancy_shm_current: Arc::new(RwLock::new(None)),
            explore_scheduler: Arc::new(tokio::sync::Mutex::new(None)),
            runtime_observation_generation: self.runtime_observation_generation.clone(),
        }
    }

    async fn publish_state(&self, state: NavState, lease: Option<crate::nav_runtime::GoalLease>) {
        self.state.write().await.nav_state = state.clone();
        let update = NavigationUpdate::from_local(state, &self.nav, lease).await;
        let _ = self.nav_state_tx.send(update);
    }
}

fn classify_exploration_submit_rejection(
    retry_reason: Option<crate::nav_runtime::SetGoalRetryReason>,
    message: String,
) -> crate::explore_runtime::ChildNavigationSubmitError {
    use crate::nav_runtime::{NavigationReadinessCode, SetGoalRetryReason};

    let retryable = match retry_reason {
        Some(SetGoalRetryReason::MapRevisionMismatch) => true,
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
    async fn submit(
        &self,
        request: crate::explore_runtime::ChildNavigationRequest,
    ) -> Result<
        crate::explore_runtime::ChildNavigationTicket,
        crate::explore_runtime::ChildNavigationSubmitError,
    > {
        let child_request_id = request.request_id.clone();
        let parent_request_id = request.parent_request_id.clone();
        let map_version_id = request.map_version_id.clone();
        let map_revision = request.map_revision;
        let target = request.goal.target;
        let frame_id = target.frame_id.as_str().to_string();
        let pose = target.pose;
        let target = PoseState {
            x: pose.position.x,
            y: pose.position.y,
            z: pose.position.z,
            qx: pose.orientation.x,
            qy: pose.orientation.y,
            qz: pose.orientation.z,
            qw: pose.orientation.w,
            frame_id: frame_id.clone(),
            timestamp_ns: target.stamp.nanos.max(0) as u64,
        };
        let facade = self.facade_handle();
        let boundary =
            request
                .boundary
                .as_ref()
                .map(|boundary| crate::nav_runtime::GoalBoundaryConstraint {
                    frame_id: "map".to_string(),
                    points: boundary
                        .vertices()
                        .iter()
                        .map(|point| [point.x, point.y, 0.0])
                        .collect(),
                });
        let outcome = crate::nav_runtime::set_goal_with_boundary(
            &facade,
            &self.nav,
            crate::nav_runtime::SetGoalInput {
                goal_id: child_request_id.clone(),
                owner_kind: schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                parent_request_id,
                goal_generation: 0,
                map_version_id,
                map_revision,
                frame_id,
                target,
            },
            boundary,
        )
        .await;
        let accepted = matches!(
            outcome.state.status.to_ascii_uppercase().as_str(),
            "ACCEPTED" | "PLANNING" | "RUNNING"
        );
        let message = outcome.state.message.clone();
        let lease = outcome.lease.clone();
        let retry_reason = outcome.retry_reason;
        self.publish_state(outcome.state, outcome.lease).await;
        if !accepted {
            return Err(classify_exploration_submit_rejection(retry_reason, message));
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
                exploration_child_lease(lease) == *expected
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
            if cancel.generation == 0 || cancel_lease != *expected {
                return Err("navigation cancel tracker returned a mismatched ticket".to_string());
            }
            Ok(crate::explore_runtime::ChildNavigationCancelTicket {
                lease: cancel_lease,
                cancel_generation: cancel.generation,
            })
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
        && lease.map_revision == feedback.planner_map_revision
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
    let correlated_lease = match nav
        .correlate_navigation_state(
            &update.state.goal_id,
            update.feedback.owner_kind,
            &update.feedback.parent_request_id,
            update.feedback.goal_generation,
            update.feedback.planner_map_revision,
            update.feedback.path_revision,
            update.feedback.goal_payload_sha256,
            update.feedback.path_payload_sha256,
        )
        .await
    {
        Ok(Some(lease)) => {
            update.feedback.map_version_id = lease.map_version_id.clone();
            update.producer_runtime_generation = lease.producer_runtime_generation;
            Some(lease)
        }
        Ok(None) => None,
        Err(error) => {
            let cancel = nav.cancel_snapshot();
            let expected = cancel.lease.as_ref();
            let zero_ack_present = !update.feedback.zero_ack_id.trim().is_empty();
            let complete_zero_proof = update.feedback.cancel_generation > 0
                && zero_ack_present
                && update.feedback.zero_applied_monotonic_ns > 0;
            warn!(
                %error,
                session_id = %update.state.goal_id,
                received_path_revision = update.feedback.path_revision,
                expected_path_revision = expected.map_or(0, |lease| lease.path_revision),
                received_path_hash_zero = update.feedback.path_payload_sha256 == [0; 32],
                expected_path_hash_zero = expected
                    .is_none_or(|lease| lease.path_payload_sha256 == [0; 32]),
                path_hash_matches = expected.is_some_and(|lease| {
                    lease.path_payload_sha256 == update.feedback.path_payload_sha256
                }),
                received_cancel_generation = update.feedback.cancel_generation,
                expected_cancel_generation = cancel.generation,
                zero_ack_present,
                zero_applied_monotonic_ns = update.feedback.zero_applied_monotonic_ns,
                complete_zero_proof,
                cancel_phase = ?cancel.phase,
                cancel_goal_matches = cancel.goal_id == update.state.goal_id,
                owner_matches = expected
                    .is_some_and(|lease| lease.owner_kind == update.feedback.owner_kind),
                parent_matches = expected.is_some_and(|lease| {
                    lease.parent_request_id == update.feedback.parent_request_id
                }),
                goal_generation_matches = expected.is_some_and(|lease| {
                    lease.goal_generation == update.feedback.goal_generation
                }),
                map_revision_matches = expected.is_some_and(|lease| {
                    lease.map_revision == update.feedback.planner_map_revision
                }),
                goal_hash_matches = expected.is_some_and(|lease| {
                    lease.goal_payload_sha256 == update.feedback.goal_payload_sha256
                }),
                "typed navigation state rejected"
            );
            return None;
        }
    };
    let planner_observation = if update.feedback.runtime_generation > 0 {
        let map_frame = normalize_frame(&update.feedback.planner_map_frame);
        let pose_frame = normalize_frame(&update.feedback.planner_pose_frame);
        let ready = update.feedback.planner_map_revision > 0
            && !map_frame.is_empty()
            && map_frame == pose_frame;
        Some(crate::nav_runtime::PlannerInputObservation {
            generation: update.feedback.runtime_generation,
            map_revision: update.feedback.planner_map_revision,
            map_frame,
            pose_frame,
            ready,
        })
    } else if update.feedback.planner_map_revision > 0
        || !update.feedback.planner_map_frame.is_empty()
        || !update.feedback.planner_pose_frame.is_empty()
    {
        warn!("planner feedback missing positive runtime_generation; fail closed");
        None
    } else {
        None
    };

    let motion_sink_observation = if update.feedback.guard_generation > 0 {
        Some(crate::nav_runtime::MotionSinkObservation {
            generation: update.feedback.guard_generation,
            ready: update.feedback.sink_ready,
        })
    } else if update.feedback.sink_ready {
        warn!("sink_ready=true missing positive guard_generation; fail closed");
        None
    } else {
        None
    };

    update.state = if let Some(lease) = correlated_lease.as_ref() {
        match nav
            .observe_correlated_navigation_state(
                update.state.clone(),
                lease,
                planner_observation,
                motion_sink_observation,
            )
            .await
        {
            Ok(state) => state,
            Err(error) => {
                warn!(%error, session_id = %update.state.goal_id, "superseded navigation state rejected");
                return None;
            }
        }
    } else {
        if let Some(observation) = planner_observation {
            if let Err(error) = nav.observe_planner_input(observation) {
                warn!(?error, "typed planner observation rejected");
            }
        }
        if let Some(observation) = motion_sink_observation {
            if let Err(error) = nav.observe_motion_sink(observation) {
                warn!(?error, "typed motion-sink observation rejected");
            }
        }
        nav.observe_unowned_stack_heartbeat(update.state)
    };
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

fn exploration_navigation_terminal_result(
    update: &NavigationUpdate,
) -> Option<crate::explore_runtime::ChildNavigationResult> {
    let proof = crate::explore_runtime::ChildNavigationTerminalProof {
        lease: crate::explore_runtime::ChildNavigationLease {
            session_id: update.state.goal_id.clone(),
            owner_kind: update.feedback.owner_kind,
            parent_request_id: update.feedback.parent_request_id.clone(),
            goal_generation: update.feedback.goal_generation,
            map_version_id: update.feedback.map_version_id.clone(),
            map_revision: update.feedback.planner_map_revision,
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
) {
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_millis(100));
        interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            interval.tick().await;
            let Some(outcome) = nav.poll_cancel_timeout().await else {
                continue;
            };
            let update = NavigationUpdate::from_local(outcome.state, &nav, outcome.lease).await;
            state.write().await.nav_state = update.state.clone();
            let _ = nav_state_tx.send(update.clone());
            report_exploration_navigation_terminal(&explore, &update).await;
        }
    });
}

fn spawn_path_envelope_consumer(
    transport: LcmTransport,
    nav: crate::nav_runtime::NavRuntimeState,
    path_binding: (LcmTopic, LcmMessageType),
) {
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
    });
}

fn frontier_entry(frontier: &nuwax_nav_types_messages::visual::FrontierGoal) -> FrontierEntry {
    let pose = frontier.pose.pose;
    FrontierEntry {
        x: pose.position.x,
        y: pose.position.y,
        z: pose.position.z,
        qx: pose.orientation.x,
        qy: pose.orientation.y,
        qz: pose.orientation.z,
        qw: pose.orientation.w,
        frame_id: frontier.pose.frame_id.as_str().to_string(),
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
    if snapshot.frontiers.len() > schema::FRONTIER_LIST_CAPACITY {
        return Err(format!(
            "frontier count {} exceeds LCM capacity {}",
            snapshot.frontiers.len(),
            schema::FRONTIER_LIST_CAPACITY
        ));
    }
    if snapshot.frontiers.iter().any(|frontier| {
        !matches!(
            frontier.pose.frame_id,
            nuwax_nav_types_messages::frames::NavFrameId::Map
        )
    }) {
        return Err("frontier snapshot contains a non-map-frame goal".to_string());
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
        frame_id: "map".to_string(),
        frontiers_length: snapshot.frontiers.len(),
        frontiers,
    })
    .map_err(|error| format!("encode frontier snapshot failed: {error}"))
}

fn spawn_map_revision_coalescer(
    mut map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) {
    tokio::spawn(async move {
        loop {
            match map_revision_rx.recv().await {
                Ok(event) => {
                    let _lifecycle_guard = mapping_lifecycle_lock.lock().await;
                    if !map_event_matches_current_session(&event, &mapping_adapter) {
                        warn!(
                            session_id = %event.session_id,
                            session_generation = event.session_generation,
                            revision = event.revision,
                            "stale map revision rejected by lifecycle fence"
                        );
                        continue;
                    }
                    map_revision_latest_tx.send_replace(Some(event));
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
    });
}

fn spawn_map_revision_fanout(
    map_revision_rx: broadcast::Receiver<MapRevisionEvent>,
    map_revision_latest_tx: watch::Sender<Option<MapRevisionEvent>>,
    transport: LcmTransport,
    occupancy_topic: LcmTopic,
    occupancy_msg_type: LcmMessageType,
    occupancy_shm: Option<Arc<parking_lot::Mutex<OccupancyGridShmWriter>>>,
    occupancy_shm_path: Option<String>,
    occupancy_shm_current: Arc<RwLock<Option<OccupancyShmSnapshot>>>,
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    mapping_adapter: Arc<parking_lot::Mutex<MappingAdapter>>,
    mapping_lifecycle_lock: Arc<tokio::sync::Mutex<()>>,
) {
    let mut shm_rx = map_revision_latest_tx.subscribe();
    let mut lcm_rx = map_revision_latest_tx.subscribe();
    let mut explore_rx = map_revision_latest_tx.subscribe();

    match (occupancy_shm, occupancy_shm_path) {
        (Some(writer), Some(path)) => {
            let current = occupancy_shm_current;
            let mapping_adapter = mapping_adapter.clone();
            let lifecycle_lock = mapping_lifecycle_lock.clone();
            tokio::spawn(async move {
                while shm_rx.changed().await.is_ok() {
                    let Some(event) = shm_rx.borrow_and_update().clone() else {
                        continue;
                    };
                    let _lifecycle_guard = lifecycle_lock.lock().await;
                    if !map_event_matches_current_session(&event, &mapping_adapter) {
                        continue;
                    }
                    let map = event.to_map_state();
                    let write_result = {
                        let mut writer = writer.lock();
                        writer
                            .begin_session(event.session_generation)
                            .and_then(|_| writer.write_occupancy(&map))
                            .and_then(|_| {
                                let file_size = std::fs::metadata(writer.shm_path())?.len();
                                let active_slot = writer.active_slot().ok_or_else(|| {
                                    std::io::Error::other(
                                        "occupancy SHM write completed without an active slot",
                                    )
                                })?;
                                Ok(OccupancyShmSnapshot {
                                    key: path.clone(),
                                    file_size,
                                    payload_size: event.size,
                                    revision: event.revision,
                                    layout_version: writer.layout_version(),
                                    payload_schema_version: event.schema_version,
                                    generation: writer.published_generation(),
                                    active_slot,
                                })
                            })
                    };
                    match write_result {
                        Ok(snapshot) => {
                            *current.write().await = Some(snapshot);
                        }
                        Err(error) => {
                            warn!(revision = event.revision, %error, "occupancy SHM fanout failed");
                        }
                    }
                }
            });
        }
        (Some(_), None) => {
            warn!("occupancy SHM writer exists without its authoritative path; SHM sink disabled");
        }
        (None, _) => {}
    }

    let lcm_mapping_adapter = mapping_adapter.clone();
    let lcm_lifecycle_lock = mapping_lifecycle_lock.clone();
    tokio::spawn(async move {
        while lcm_rx.changed().await.is_ok() {
            let Some(event) = lcm_rx.borrow_and_update().clone() else {
                continue;
            };
            let _lifecycle_guard = lcm_lifecycle_lock.lock().await;
            if !map_event_matches_current_session(&event, &lcm_mapping_adapter) {
                continue;
            }
            let payload = match encode_occupancy_revision(&event) {
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
    });

    let explore_mapping_adapter = mapping_adapter.clone();
    let explore_lifecycle_lock = mapping_lifecycle_lock.clone();
    tokio::spawn(async move {
        while explore_rx.changed().await.is_ok() {
            let Some(event) = explore_rx.borrow_and_update().clone() else {
                continue;
            };
            let _lifecycle_guard = explore_lifecycle_lock.lock().await;
            if !map_event_matches_current_session(&event, &explore_mapping_adapter) {
                continue;
            }
            match exploration_costmap_from_revision(&event) {
                Ok(snapshot) => {
                    match tokio::time::timeout(
                        MAP_FANOUT_SINK_TIMEOUT,
                        explore.ingest_costmap_snapshot(snapshot),
                    )
                    .await
                    {
                        Ok(Ok(_)) => {}
                        Ok(Err(error)) => {
                            warn!(revision = event.revision, %error, "exploration costmap fanout failed");
                        }
                        Err(_) => {
                            warn!(
                                revision = event.revision,
                                "exploration costmap fanout timed out"
                            );
                        }
                    }
                }
                Err(error) => {
                    warn!(revision = event.revision, %error, "exploration costmap conversion failed");
                }
            }
        }
    });

    spawn_map_revision_coalescer(
        map_revision_rx,
        map_revision_latest_tx,
        mapping_adapter,
        mapping_lifecycle_lock,
    );
}

fn spawn_frontier_fanout(
    explore: crate::explore_runtime::ExploreRuntimeHandle,
    state: Arc<RwLock<SharedState>>,
    frontiers_tx: broadcast::Sender<Vec<FrontierEntry>>,
    transport: LcmTransport,
    frontier_binding: Option<(LcmTopic, LcmMessageType)>,
) {
    let mut frontier_rx = explore.subscribe_frontiers();
    tokio::spawn(async move {
        loop {
            if frontier_rx.changed().await.is_err() {
                break;
            }
            let snapshot = frontier_rx.borrow_and_update().clone();
            let entries: Vec<_> = snapshot.frontiers.iter().map(frontier_entry).collect();
            {
                let mut shared = state.write().await;
                shared.frontiers = entries.clone();
                shared.frontiers_revision = snapshot.revision;
            }
            let _ = frontiers_tx.send(entries);
            let Some((topic, msg_type)) = frontier_binding.as_ref() else {
                continue;
            };
            let payload = match encode_frontier_snapshot(&snapshot) {
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
    });
}

/// 启动 geometry runtime 异步循环（LCM-first）。
///
/// backend 固定为 `LcmBackendKind::Real`。启动期调用
/// `LcmTransport::new_real(&lcm_url)`；若 liblcm 不可用或启动失败，
/// → fail-fast 返回 [`RealLcmBackendUnavailable`] 或 `process::exit(1)`
/// （当 `NUWAX_REAL_LCM_REQUIRED=true` 时）。
///
/// # Args
///
/// * `odometry_channel` — Phase 1 新增；`nav_msgs/Odometry` typed channel
/// * 其余形参来自 Phase 0 默认 channel registry
pub fn spawn_geometry_runtime(
    backend: LcmBackendKind,
    occupancy_channel: &str,
    tf_channel: &str,
    cloud_map_channel: Option<&str>,
    nav_state_channel: Option<&str>,
    frontiers_channel: Option<&str>,
    odometry_channel: Option<&str>,
) -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    let backend_name = backend.canonical_name();
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavigationUpdate>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);
    let (map_revision_tx, _) = broadcast::channel::<MapRevisionEvent>(16);
    let (map_revision_latest_tx, _) = watch::channel::<Option<MapRevisionEvent>>(None);
    // Register the production fanout receiver before mapping can commit. A
    // tokio broadcast sender drops events when it has no receivers.
    let map_revision_fanout_rx = map_revision_tx.subscribe();

    let sequence_state_path = sequence_state_path_from_env();
    let sequence_store = match SequenceStateStore::open(&sequence_state_path) {
        Ok(store) => Arc::new(store),
        Err(error) => {
            error!(path = %sequence_state_path.display(), %error, "geometry sequence state startup failed closed");
            return Err(RealLcmBackendUnavailable);
        }
    };
    let state = Arc::new(RwLock::new(SharedState::default()));
    let nav_runtime = match crate::nav_runtime::NavRuntimeState::try_from_sequence_store(
        sequence_store.as_ref(),
    ) {
        Ok(runtime) => runtime,
        Err(error) => {
            error!(path = %sequence_state_path.display(), %error, "GoalEnvelope producer epoch reservation failed closed");
            return Err(RealLcmBackendUnavailable);
        }
    };
    let explore_runtime = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
    let runtime_observation_generation = Arc::new(std::sync::atomic::AtomicU64::new(0));

    // 按 NUWAX_LCM_BACKEND 选择真实 backend。强制模式下启动失败直接退出。
    let real_lcm_required = real_lcm_required_from_env();
    let (transport, backend_available): (LcmTransport, bool) = {
        let lcm_url = std::env::var("NUWAX_LCM_URL")
            .unwrap_or_else(|_| "udpm://239.255.76.67:7667?ttl=1".to_owned());
        // real-lcm feature gate —— 二选一:
        //   1. binary --features real-lcm-bridge → 真起 liblcm
        //   2. binary 未启用 real-lcm-bridge feature → 保留 Phase 2 行为 (return Err)
        //   2. 未启用 feature → 返回 backend 不可用
        #[cfg(feature = "real-lcm-bridge")]
        {
            match attempt_real_lcm_startup(real_lcm_required, || LcmTransport::new_real(&lcm_url)) {
                Ok(t) => {
                    info!("LCM backend 选择: real-lcm (provider={})", lcm_url);
                    (t, true)
                }
                Err(RealLcmStartupError::Required(e)) => {
                    eprintln!(
                        "LCM_BACKEND_UNAVAILABLE: LcmTransport::new_real({}) 失败: {} \
                         —— NUWAX_REAL_LCM_REQUIRED=true，按生产意图立即硬退出 \
                         (exit code = 1)。请确认 liblcm-dev 已安装 \
                         (apt-get install liblcm-dev) 且 provider URL 合法。",
                        lcm_url, e
                    );
                    std::process::exit(1);
                }
                Err(RealLcmStartupError::Optional(e)) => {
                    // 不允许降级，直接 fail-fast。
                    eprintln!(
                        "LCM_BACKEND_UNAVAILABLE: LcmTransport::new_real({}) 失败: {} \
                         —— 请确认 liblcm-dev 已安装 (apt-get install liblcm-dev) \
                         且 provider URL 合法。",
                        lcm_url, e
                    );
                    return Err(RealLcmBackendUnavailable);
                }
            }
        }
        #[cfg(not(feature = "real-lcm-bridge"))]
        {
            let _ = lcm_url;
            return Err(RealLcmBackendUnavailable);
        }
    };
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

    let (occ_topic, occ_msg_type) =
        match parse_expected_typed_channel(occupancy_channel, "nav_msgs.OccupancyGrid") {
            Ok(binding) => binding,
            Err(e) => {
                error!("lcm_runtime::spawn: occupancy channel 解析失败: {}", e);
                return Err(RealLcmBackendUnavailable);
            }
        };
    let (tf_topic, tf_msg_type) =
        match parse_expected_typed_channel(tf_channel, "tf2_msgs.TFMessage") {
            Ok(binding) => binding,
            Err(e) => {
                error!("lcm_runtime::spawn: tf channel 解析失败: {}", e);
                return Err(RealLcmBackendUnavailable);
            }
        };
    // Legacy `/cloud_map` is diagnostics-only. MappingSession is the sole
    // cloud-map writer; accepting both sources would interleave unrelated
    // revisions in the same mmap.
    let legacy_cm_binding = match cloud_map_channel {
        Some(c) => match parse_expected_typed_channel(c, "sensor_msgs.PointCloud2") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: cloud_map channel 解析失败 ({}): 降级无 cloud_map 模式",
                    e
                );
                None
            }
        },
        None => None,
    };
    let nav_binding = match nav_state_channel {
        Some(c) => match parse_expected_typed_channel(c, "nuwax_msgs.NavigationState") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: nav_state channel 解析失败 ({}): 降级无 nav 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    let fr_binding = match frontiers_channel {
        Some(c) => match parse_expected_typed_channel(c, "nuwax_msgs.FrontierList") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: frontiers channel 解析失败 ({}): 降级无 frontier 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    // Phase 1 channel registry 补齐:解析 typed odometry channel。
    // Phase 4: 改走 `parse_channel_with_suffix`,强制 typed suffix;不再 untyped 兜底。
    let odom_binding = match odometry_channel {
        Some(c) => match parse_expected_typed_channel(c, "nav_msgs.Odometry") {
            Ok(binding) => Some(binding),
            Err(e) => {
                warn!(
                    "lcm_runtime::spawn: odom channel 解析失败 ({}): 降级无 /odom 订阅",
                    e
                );
                None
            }
        },
        None => None,
    };
    if let Err(error) = crate::nav_runtime::goal_envelope_binding() {
        error!(%error, "versioned goal channel binding rejected");
        return Err(RealLcmBackendUnavailable);
    }
    let path_binding = match crate::nav_runtime::path_envelope_binding() {
        Ok(binding) => binding,
        Err(error) => {
            error!(%error, "versioned path channel binding rejected");
            return Err(RealLcmBackendUnavailable);
        }
    };
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

    // 累积地图 mmap 初始化（降级允许）
    let cm_shm_path = crate::pointcloud::cloud_map_shm_path();
    let cm_writer = match CloudMapShmWriter::new(&cm_shm_path) {
        Ok(w) => Some(w),
        Err(e) => {
            warn!("累积地图 mmap 初始化失败（降级为无 cloud_map 模式）: {}", e);
            None
        }
    };
    // 用 parking_lot::Mutex 避免 std::sync::MutexGuard 不 Send 的问题。
    let cm_writer = Arc::new(parking_lot::Mutex::new(cm_writer));

    // ---- 订阅 /tf：写入 canonical TfTree 历史并合成 base→map 位姿 ----
    let map_frame = std::env::var("NUWAX_GEOMETRY_MAP_FRAME")
        .or_else(|_| std::env::var("NUWAX_MAP_FRAME"))
        .unwrap_or_else(|_| "map".into());
    let odom_frame = std::env::var("NUWAX_GEOMETRY_ODOM_FRAME")
        .or_else(|_| std::env::var("NUWAX_ODOM_FRAME"))
        .unwrap_or_else(|_| "odom".into());
    let base_frame = std::env::var("NUWAX_GEOMETRY_BASE_FRAME")
        .or_else(|_| std::env::var("NUWAX_BASE_FRAME"))
        .unwrap_or_else(|_| "base_link".into());
    let base_candidates: Vec<String> =
        if base_frame == "base_link" || base_frame == "base_footprint" {
            vec!["base_link".into(), "base_footprint".into()]
        } else {
            vec![
                base_frame.clone(),
                "base_link".into(),
                "base_footprint".into(),
            ]
        };

    // Mapping exists for the process lifetime but starts inactive. The
    // StartMapping RPC is the only production transition that opens ingestion.
    let mapping_adapter = if backend_available {
        let cfg = MappingAdapterConfig::from_env();
        info!(
            "mapping_adapter 已构造且等待 StartMapping: resolution={}m auto_bounds_padding=1m occupancy_algo={}",
            cfg.resolution,
            cfg.occupancy_algo.as_dimos_str()
        );
        Some(Arc::new(parking_lot::Mutex::new(
            MappingAdapter::new_with_frames_and_sequence_store(
                cfg,
                map_frame.clone(),
                odom_frame.clone(),
                sequence_store.clone(),
            ),
        )))
    } else {
        None
    };

    let tf_state = state.clone();
    let tf_transport = transport.clone();
    let tf_topic_clone = tf_topic.clone();
    let tf_msg_type_clone = tf_msg_type.clone();
    let map_frame_owned = map_frame.clone();
    let odom_frame_owned = odom_frame.clone();
    let base_candidates_owned = base_candidates.clone();
    let tf_nav_runtime = nav_runtime.clone();
    tokio::spawn(async move {
        let mut tf_generation = 0_u64;
        let mut stream = tf_transport
            .subscribe(tf_topic_clone, Some(tf_msg_type_clone))
            .await;
        use futures::StreamExt;
        while let Some(frame_result) = stream.next().await {
            let frame = match frame_result {
                Ok(f) => f,
                Err(e) => {
                    error!("/tf LCM 订阅错误: {}", e);
                    continue;
                }
            };
            let new_entries = match parse_tf_payload(&frame.payload) {
                Some(v) => v,
                None => {
                    warn!("/tf payload 解析失败 ({} 字节)", frame.payload.len());
                    continue;
                }
            };
            let mut s = tf_state.write().await;
            let mut max_stamp_ns: u64 = 0;
            let mut accepted_entries = 0_u64;
            for entry in new_entries {
                let Some(transform) = entry_to_transform(&entry) else {
                    warn!(
                        parent = %entry.parent,
                        child = %entry.child,
                        "忽略 frame/quaternion 非法的 /tf entry"
                    );
                    continue;
                };
                let stamp = entry.stamp_ns;
                s.tf_tree.set(transform);
                accepted_entries = accepted_entries.saturating_add(1);
                if let Some(existing) = s
                    .transforms
                    .iter_mut()
                    .find(|t| t.parent == entry.parent && t.child == entry.child)
                {
                    *existing = entry;
                } else {
                    s.transforms.push(entry);
                }
                if stamp > max_stamp_ns {
                    max_stamp_ns = stamp;
                }
            }
            if max_stamp_ns > TF_ENTRY_TTL_NS {
                let cutoff = max_stamp_ns - TF_ENTRY_TTL_NS;
                s.transforms
                    .retain(|t| t.stamp_ns == 0 || t.stamp_ns >= cutoff);
            }
            let base_refs: Vec<&str> = base_candidates_owned.iter().map(|s| s.as_str()).collect();
            if let Some((x, y, z, qx, qy, qz, qw, _base)) =
                resolve_map_pose_tree(&s.tf_tree, &map_frame_owned, &base_refs)
            {
                s.pose.x = x;
                s.pose.y = y;
                s.pose.z = z;
                s.pose.qx = qx;
                s.pose.qy = qy;
                s.pose.qz = qz;
                s.pose.qw = qw;
                s.pose.frame_id = map_frame_owned.clone();
                if max_stamp_ns > 0 {
                    s.pose.timestamp_ns = max_stamp_ns;
                }
            }
            let map_to_odom =
                resolve_transform(&s.transforms, &odom_frame_owned, &map_frame_owned).is_some();
            let odom_to_base = base_candidates_owned
                .iter()
                .any(|base| resolve_transform(&s.transforms, base, &odom_frame_owned).is_some());
            drop(s);
            if accepted_entries > 0 {
                tf_generation = tf_generation.saturating_add(1);
                if let Err(error) =
                    tf_nav_runtime.observe_tf_chain(crate::nav_runtime::TfChainObservation {
                        generation: tf_generation,
                        map_frame: normalize_frame(&map_frame_owned),
                        odom_frame: normalize_frame(&odom_frame_owned),
                        base_frame: normalize_frame(&base_candidates_owned[0]),
                        map_to_odom,
                        odom_to_base,
                    })
                {
                    warn!(?error, "nav TF observation rejected");
                }
            }
        }
    });

    if legacy_cm_binding.is_some() {
        info!("legacy /cloud_map input is ignored; MappingSession owns cloud-map SHM revisions");
    }

    // ---- Consume only versioned PathEnvelope planner output. ----
    spawn_path_envelope_consumer(transport.clone(), nav_runtime.clone(), path_binding);

    // ---- 订阅 nav_state（navigation state 推送）----
    if let Some((nav_topic, nav_msg_type)) = nav_binding {
        let nav_state = state.clone();
        let nav_tx = nav_state_tx.clone();
        let nav_transport = transport.clone();
        let nav_msg_type_clone = nav_msg_type;
        let nav_runtime_state = nav_runtime.clone();
        let nav_explore = explore_runtime.clone();
        tokio::spawn(async move {
            let mut stream = nav_transport
                .subscribe(nav_topic, Some(nav_msg_type_clone))
                .await;
            use futures::StreamExt;
            while let Some(frame_result) = stream.next().await {
                let frame = match frame_result {
                    Ok(f) => f,
                    Err(e) => {
                        error!("nav_state LCM 订阅错误: {}", e);
                        continue;
                    }
                };
                let Some(update) = crate::typed_codec::decode_nav_state_payload(&frame.payload)
                else {
                    warn!(
                        "nav_state generated payload 解析失败 ({} 字节)",
                        frame.payload.len()
                    );
                    continue;
                };
                let Some(update) =
                    apply_typed_navigation_feedback(&nav_runtime_state, update).await
                else {
                    continue;
                };
                nav_state.write().await.nav_state = update.state.clone();
                let _ = nav_tx.send(update.clone());
                report_exploration_navigation_terminal(&nav_explore, &update).await;
            }
        });
    }

    spawn_cancel_timeout_task(
        nav_runtime.clone(),
        state.clone(),
        nav_state_tx.clone(),
        explore_runtime.clone(),
    );

    // ExploreRuntime is the sole frontier owner. Its watch stream updates the
    // process cache, gRPC broadcast, and typed LCM channel from one revision.
    spawn_frontier_fanout(
        explore_runtime.clone(),
        state.clone(),
        frontiers_tx.clone(),
        transport.clone(),
        fr_binding,
    );

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
        nav: nav_runtime,
        explore: explore_runtime.clone(),
        mapping_adapter: mapping_adapter.clone(),
        mapping_lifecycle_lock: mapping_lifecycle_lock.clone(),
        occupancy_shm: occupancy_shm.clone(),
        occupancy_shm_path: occupancy_shm_path.clone(),
        occupancy_shm_current: occupancy_shm_current.clone(),
        explore_scheduler: explore_scheduler.clone(),
        runtime_observation_generation: runtime_observation_generation.clone(),
    };

    explore_runtime.set_navigation_sink(Arc::new(ExploreNavigationBridge {
        state: state.clone(),
        cloud_map_tx,
        nav_state_tx,
        frontiers_tx,
        map_revision_tx: map_revision_tx.clone(),
        map_revision_latest_tx: map_revision_latest_tx.clone(),
        transport: transport.clone(),
        transport_backend_name: backend_name,
        lcm_backend_available: backend_available,
        cmd_vel_sink_ready,
        nav: handle.nav.clone(),
        runtime_observation_generation,
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
    let scheduler = explore_runtime.spawn_scheduler(
        Arc::new(SharedStatePoseProvider {
            state: state.clone(),
            freshness: Arc::new(parking_lot::Mutex::new(PoseFreshnessTracker::default())),
            max_receipt_age: Duration::from_millis(pose_ttl_ms),
        }),
        Duration::from_millis(scheduler_period_ms),
    );
    *explore_scheduler
        .try_lock()
        .expect("new exploration scheduler lock must be uncontended") = Some(scheduler);

    let fanout_mapping_adapter = mapping_adapter
        .as_ref()
        .expect("available LCM backend must construct mapping adapter")
        .clone();
    spawn_map_revision_fanout(
        map_revision_fanout_rx,
        map_revision_latest_tx,
        transport.clone(),
        occ_topic,
        occ_msg_type,
        occupancy_shm,
        occupancy_shm_path,
        occupancy_shm_current,
        explore_runtime,
        fanout_mapping_adapter,
        mapping_lifecycle_lock,
    );
    if let Some(mapping_arc) = mapping_adapter {
        spawn_mapping_feed_tasks(
            transport,
            state,
            mapping_arc,
            map_revision_tx,
            odom_binding,
            (tf_topic, tf_msg_type),
            base_frame,
            handle.cloud_map_writer.clone(),
            handle.cloud_map_tx.clone(),
            handle.mapping_lifecycle_lock.clone(),
        );
    }

    Ok(handle)
}

#[derive(Debug)]
struct MappingIngestOutcome {
    accepted_map_points: Vec<nalgebra::Point3<f64>>,
    occupancy_committed: bool,
    session_generation: u64,
}

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
            accepted_map_points: Vec::new(),
            occupancy_committed: false,
            session_generation: adapter.snapshot().session_generation,
        });
    }
    let accepted_map_points = adapter.take_latest_accepted_map_points();
    let session_generation = adapter.snapshot().session_generation;
    let occupancy_committed = adapter.build_and_publish(state, Some(map_revision_tx));
    Ok(MappingIngestOutcome {
        accepted_map_points,
        occupancy_committed,
        session_generation,
    })
}

fn update_mapping_cloud_map_if_current(
    outcome: &MappingIngestOutcome,
    current_session: &crate::mapping_adapter::MappingSessionSnapshot,
    writer: &Arc<parking_lot::Mutex<Option<CloudMapShmWriter>>>,
) -> Option<CloudMapNotification> {
    if !current_session.active || current_session.session_generation != outcome.session_generation {
        return None;
    }
    let mut writer = writer.lock();
    let writer = writer.as_mut()?;
    if !writer.accumulate_mapping_frame(
        outcome.session_generation,
        outcome.accepted_map_points.as_slice(),
    ) || !outcome.occupancy_committed
    {
        return None;
    }
    let (revision, point_count, voxel_size) = writer.commit_accumulated()?;
    Some(CloudMapNotification {
        revision,
        point_count,
        voxel_size,
        session_generation: outcome.session_generation,
    })
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
    odom_binding: Option<(LcmTopic, LcmMessageType)>,
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
                if let Err(error) = transport_odom.publish(tf_frame).await {
                    warn!(%error, "canonical mapping TF publish failed");
                }
            }
        });
    } else {
        warn!("mapping_feed: odometry binding unavailable; odom anchor/TF feed disabled");
    }

    // ---- /go2/lidar/points → feed_pointcloud + build_and_publish ----
    let (lidar_topic, lidar_msg_type) =
        match binding_for_key(ChannelKey::PointCloud, "sensor_msgs.PointCloud2") {
            Ok(binding) => binding,
            Err(e) => {
                error!(
                    "spawn_mapping_feed_tasks: pointcloud typed channel 无效: {}",
                    e
                );
                return;
            }
        };
    let transport_lidar = transport.clone();
    let mapping_lidar = mapping_arc.clone();
    let state_lidar = state.clone();
    let map_revision_lidar = map_revision_tx.clone();
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
            let _lifecycle_guard = mapping_lifecycle_lock.lock().await;
            let (ingest_outcome, current_session) = {
                let mut adapter = mapping_lidar.lock();
                let outcome = ingest_mapping_points(
                    &mut adapter,
                    &points,
                    parsed.point_step,
                    now_ns,
                    &parsed.frame_id,
                    parsed.timestamp_ns,
                    &state_lidar,
                    &map_revision_lidar,
                );
                let snapshot = adapter.snapshot();
                (outcome, snapshot)
            };
            match ingest_outcome {
                Ok(outcome) => {
                    let Some(update) = update_mapping_cloud_map_if_current(
                        &outcome,
                        &current_session,
                        &cloud_map_writer,
                    ) else {
                        continue;
                    };
                    {
                        let mut shared = state_lidar.write().await;
                        shared.cloud_map_revision = update.revision;
                    }
                    let _ = cloud_map_tx.send(update);
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
/// - 不订阅任何 LCM typed channel
/// - 不写 SHM 文件
/// - 立即返回,适合 `#[tokio::test]` async 测试
pub async fn build_handle_for_tests() -> Result<GeometryRuntimeHandle, RealLcmBackendUnavailable> {
    let (cloud_map_tx, _) = broadcast::channel::<CloudMapNotification>(4);
    let (nav_state_tx, _) = broadcast::channel::<NavigationUpdate>(8);
    let (frontiers_tx, _) = broadcast::channel::<Vec<FrontierEntry>>(4);
    let (map_revision_tx, _) = broadcast::channel::<MapRevisionEvent>(16);
    let map_revision_fanout_rx = map_revision_tx.subscribe();
    let (map_revision_latest_tx, _) = watch::channel::<Option<MapRevisionEvent>>(None);
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
    Ok(GeometryRuntimeHandle {
        state,
        cloud_map_tx,
        cloud_map_writer: Arc::new(parking_lot::Mutex::new(None)),
        nav_state_tx,
        frontiers_tx,
        map_revision_tx,
        map_revision_latest_tx,
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
}

#[cfg(test)]
mod mapping_feed_tests {
    use super::*;
    use crate::mapping_adapter::MappingReadiness;
    use nalgebra::Point3;

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
            .start_session("test-session", "test-map", "mapver_000001")
            .expect("active mapping test session");
        adapter
    }

    fn shared_active_mapping_adapter() -> Arc<parking_lot::Mutex<MappingAdapter>> {
        Arc::new(parking_lot::Mutex::new(active_mapping_adapter()))
    }

    fn map_revision_event(map: MapState) -> MapRevisionEvent {
        let adapter = active_mapping_adapter();
        MapRevisionEvent::from_map(&map, &adapter.snapshot())
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
        assert_eq!(event.map_version_id, "mapver_000001");
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
        let update = update_mapping_cloud_map_if_current(&outcome, &current_session, &writer)
            .expect("cloud map update from mapping session");
        assert_eq!(update.revision, 1);
        assert_eq!(update.point_count, 2);
        let mut stale_session = current_session;
        stale_session.session_generation += 1;
        assert!(update_mapping_cloud_map_if_current(&outcome, &stale_session, &writer).is_none());
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
            .start_session("external-test", "warehouse", "mapver_000001")
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
        assert_eq!(costmap.map_revision, 9);
        assert_eq!(costmap.map_version_id, "mapver_000001");
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
                planner_map_revision: 3,
                map_version_id: "map-v1".to_string(),
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
        assert_eq!(proof.lease.map_revision, 3);

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

    fn test_lease(goal_id: &str) -> crate::nav_runtime::GoalLease {
        crate::nav_runtime::GoalLease {
            session_id: goal_id.to_string(),
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: format!("request-{goal_id}"),
            goal_generation: 1,
            producer_runtime_generation: 1,
            map_version_id: "map-v1".to_string(),
            map_revision: 42,
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
                planner_map_revision: 42,
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
                map_version_id: lease
                    .as_ref()
                    .map_or_else(String::new, |lease| lease.map_version_id.clone()),
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
    async fn typed_feedback_drives_real_planner_and_motion_observations() {
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
        assert_eq!(readiness.costmap_revision, 42);
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
        let correlated_a = handle
            .nav
            .correlate_navigation_state(
                &stale_a.state.goal_id,
                stale_a.feedback.owner_kind,
                &stale_a.feedback.parent_request_id,
                stale_a.feedback.goal_generation,
                stale_a.feedback.planner_map_revision,
                stale_a.feedback.path_revision,
                stale_a.feedback.goal_payload_sha256,
                stale_a.feedback.path_payload_sha256,
            )
            .await
            .expect("A feedback must correlate before the pause")
            .expect("A lease");
        assert_eq!(correlated_a, lease_a);

        let mut lease_b = lease_a.clone();
        lease_b.path_revision = 1;
        lease_b.path_payload_sha256 = [8; 32];
        {
            let inner = handle.nav.inner();
            let mut slot = inner.write().await;
            slot.phase = crate::nav_runtime::NavPhase::Running;
            slot.goal = Some(crate::nav_runtime::ActiveGoal {
                id: lease_b.session_id.clone(),
                frame_id: "map".to_string(),
                target: PoseState::default(),
                accepted_at_ns: 2,
                lease: lease_b.clone(),
            });
        }

        let stale_planner = crate::nav_runtime::PlannerInputObservation {
            generation: stale_a.feedback.runtime_generation,
            map_revision: stale_a.feedback.planner_map_revision,
            map_frame: stale_a.feedback.planner_map_frame.clone(),
            pose_frame: stale_a.feedback.planner_pose_frame.clone(),
            ready: false,
        };
        let stale_sink = crate::nav_runtime::MotionSinkObservation {
            generation: stale_a.feedback.guard_generation,
            ready: stale_a.feedback.sink_ready,
        };
        let rejected = handle
            .nav
            .observe_correlated_navigation_state(
                stale_a.state,
                &correlated_a,
                Some(stale_planner),
                Some(stale_sink),
            )
            .await;
        assert!(
            rejected.is_err(),
            "the stale A terminal must fail its final active-incarnation CAS"
        );
        let inner = handle.nav.inner();
        let slot = inner.read().await;
        assert_eq!(slot.phase, crate::nav_runtime::NavPhase::Running);
        assert_eq!(slot.goal.as_ref().map(|goal| &goal.lease), Some(&lease_b));
        drop(slot);
        let after = crate::nav_runtime::compute_readiness(&handle, &handle.nav).await;
        assert_eq!(after.planner_generation, baseline.planner_generation);
        assert_eq!(after.map_frame, baseline.map_frame);
        assert_eq!(after.pose_frame, baseline.pose_frame);
        assert_eq!(
            after.motion_sink_generation,
            baseline.motion_sink_generation
        );
        assert_eq!(after.cmd_vel_sink_ready, baseline.cmd_vel_sink_ready);
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
            planner_runtime_generation: 1,
            map_version_id: lease.map_version_id.clone(),
            map_revision: lease.map_revision as i64,
            path_revision: 1,
            source_timestamp_ns: 10,
            goal_payload_sha256: lease.goal_payload_sha256,
            path: schema::Path {
                header: schema::Header {
                    frame_id: "map".to_string(),
                    ..schema::Header::default()
                },
                poses: vec![schema::PoseStamped {
                    header: schema::Header {
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
        tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                if nav.active_goal_lease().await.unwrap().path_revision == 1 {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("PathEnvelope consumer must update the lease");
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
    fn required_backend_classifies_actual_creation_failure_for_hard_exit() {
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

    fn smoke_costmap(map_revision: u64) -> ExploreCostmapSnapshot {
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
            frame_id: "map".to_string(),
            map_version_id: "smoke-map-v41".to_string(),
            map_revision,
        }
    }

    #[test]
    fn exploration_submit_rejection_classification_is_typed() {
        use crate::explore_runtime::ChildNavigationSubmitError;
        use crate::nav_runtime::{NavigationReadinessCode, SetGoalRetryReason};

        assert!(matches!(
            classify_exploration_submit_rejection(
                Some(SetGoalRetryReason::NavigationNotReady(
                    NavigationReadinessCode::MotionSinkNotReady,
                )),
                "motion sink unavailable".to_string(),
            ),
            ChildNavigationSubmitError::RetryableReadiness(_)
        ));
        assert!(matches!(
            classify_exploration_submit_rejection(
                Some(SetGoalRetryReason::MapRevisionMismatch),
                "planner revision is catching up".to_string(),
            ),
            ChildNavigationSubmitError::RetryableReadiness(_)
        ));
        assert!(matches!(
            classify_exploration_submit_rejection(None, "invalid boundary".to_string(),),
            ChildNavigationSubmitError::Rejected(_)
        ));
        assert!(matches!(
            classify_exploration_submit_rejection(
                Some(SetGoalRetryReason::NavigationNotReady(
                    NavigationReadinessCode::ProfileUnsupported,
                )),
                "unsupported profile".to_string(),
            ),
            ChildNavigationSubmitError::Rejected(_)
        ));
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn canonical_exploration_channels_round_trip_over_real_lcm() {
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
                frame_id: "map".to_string(),
                timestamp_ns: crate::explore_runtime::now_unix_nanos(),
                ..PoseState::default()
            },
            map: MapState {
                width: 12,
                height: 12,
                resolution: 0.5,
                frame_id: "map".to_string(),
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
            map_frame: "map".to_string(),
            odom_frame: "odom".to_string(),
            base_frame: "base_link".to_string(),
            map_to_odom: true,
            odom_to_base: true,
        })
        .expect("TF observation");
        nav.observe_planner_input(PlannerInputObservation {
            generation: 1,
            map_revision: MAP_REVISION,
            map_frame: "map".to_string(),
            pose_frame: "map".to_string(),
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
        let explore = crate::explore_runtime::ExploreRuntimeHandle::default_with_wavefront();
        explore.set_navigation_sink(Arc::new(ExploreNavigationBridge {
            state: state.clone(),
            cloud_map_tx,
            nav_state_tx,
            frontiers_tx: frontiers_tx.clone(),
            map_revision_tx,
            map_revision_latest_tx,
            transport: publisher.clone(),
            transport_backend_name: "real-lcm",
            lcm_backend_available: true,
            cmd_vel_sink_ready: Arc::new(AtomicBool::new(true)),
            nav: nav.clone(),
            runtime_observation_generation: Arc::new(AtomicU64::new(1)),
        }));
        spawn_frontier_fanout(
            explore.clone(),
            state.clone(),
            frontiers_tx,
            publisher.clone(),
            Some((frontier_topic.clone(), frontier_type.clone())),
        );

        assert!(explore
            .ingest_costmap_snapshot(smoke_costmap(MAP_REVISION))
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
                    map_version_id: "smoke-map-v41".to_string(),
                    map_revision: MAP_REVISION,
                    boundary: Some(boundary),
                    ..ExploreStartConfig::default()
                },
            )
            .await;
        assert_eq!(started.status, "ACCEPTED", "{}", started.message);
        let active = explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, "ACTIVE", "{}", active.message);
        let frontier_snapshot = explore.frontier_snapshot();
        assert_eq!(frontier_snapshot.map_revision, MAP_REVISION);
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
        assert_eq!(frontier_frame.1.frame_id, "map");
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
        assert_eq!(goal.map_version_id, "smoke-map-v41");
        assert_eq!(goal.map_revision, MAP_REVISION as i64);
        assert_eq!(goal.payload_sha256, lease.goal_payload_sha256);
        assert_eq!(
            goal.payload_sha256,
            schema::goal_envelope_payload_sha256(&goal).expect("recompute goal hash")
        );
        assert_eq!(goal.boundary_frame_id, "map");
        assert_eq!(
            goal.boundary_points,
            vec![
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [5.0, 5.0, 0.0],
                [0.0, 5.0, 0.0],
            ]
        );

        let shared = state.read().await;
        assert_eq!(shared.frontiers_revision, frontier_snapshot.revision);
        assert_eq!(shared.frontiers.len(), frontier_snapshot.frontiers.len());
        drop(shared);

        drop(frontier_frames);
        drop(goal_frames);
        drop(explore);
        drop(subscriber);
        drop(publisher);
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}
