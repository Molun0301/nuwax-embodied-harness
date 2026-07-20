// Phase 6 (codex-round3 R3-4)：server.rs 多处 format! 调用被 clippy 1.88
// `uninlined_format_args` 报告(e.g. line 239/590/605/624/639 的 `format!("...{}", e)`),
// 风格性 lint。统一 file-level 标注不强行收敛——debug log 与 Status::internal
// 字符串拼接的可读性高于命名参数形式。
#![allow(clippy::uninlined_format_args)]

//! gRPC facade — `GeometryRuntime` service.
//!
//! 所有状态来自 `GeometryRuntimeHandle`（持有 LCM runtime SharedState）；
//! 与具体 backend 实现解耦。
//!
//! Phase 2 重构:
//! - 状态聚合统一走 `runtime_diagnostics`（health / readiness / blockers）
//! - typed payload encode 走 `typed_codec` 或 `nuwax_codec::lcm::schema`
//!   structured messages（禁止写 ad-hoc bytes）
//! - env 解析入口统一走 `channel_registry`

use nuwax_codec::{lcm::schema, CodecError, CodecResult};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::{watch, RwLock};
use tonic::{Request, Response, Status};
use tracing::info;

use crate::lcm_runtime::{
    CloudMapNotification, GeometryRuntimeHandle, MapRevisionEvent, NavigationUpdate, PoseState,
};
use crate::nav_runtime::{
    cancel as nav_cancel, compute_readiness, ensure_goal_id, readiness_to_reason, set_goal,
    CancelOutcome, MotionAdmissionProof, SetGoalInput, SetGoalOutcome,
};
use crate::proto::geometry_runtime_server::GeometryRuntime;
use crate::proto::*;
use crate::runtime_diagnostics::{self, RuntimeDiagnostics, ServiceState, SharedServiceState};
use crate::typed_codec;
use nuwax_exploration::{BoundarySegment, ExplorationBoundary};
use nuwax_nav_types_messages::geometry::Point2;

#[derive(Clone)]
pub struct GeometryRuntimeService {
    state: SharedServiceState,
    bridge: GeometryRuntimeHandle,
    base_frame_id: String,
    shutdown: Option<ShutdownCoordinator>,
}

const STOP_LISTENER_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(not(test))]
const STOP_EXPLORATION_SAFETY_TIMEOUT: Duration = Duration::from_secs(6);
#[cfg(test)]
const STOP_EXPLORATION_SAFETY_TIMEOUT: Duration = Duration::from_millis(250);
const STOP_EXPLORATION_SAFETY_POLL: Duration = Duration::from_millis(20);

fn configured_base_frame() -> String {
    std::env::var("NUWAX_GEOMETRY_BASE_FRAME")
        .or_else(|_| std::env::var("NUWAX_BASE_FRAME"))
        .ok()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
        .unwrap_or_else(|| "base_link".to_string())
}

#[derive(Clone)]
pub struct ShutdownCoordinator {
    request_tx: watch::Sender<bool>,
    listener_closed_rx: watch::Receiver<bool>,
    graceful_tx: watch::Sender<bool>,
}

pub struct ShutdownListener {
    request_rx: watch::Receiver<bool>,
    listener_closed_tx: watch::Sender<bool>,
    graceful_rx: watch::Receiver<bool>,
}

pub fn shutdown_channel() -> (ShutdownCoordinator, ShutdownListener) {
    let (request_tx, request_rx) = watch::channel(false);
    let (listener_closed_tx, listener_closed_rx) = watch::channel(false);
    let (graceful_tx, graceful_rx) = watch::channel(false);
    (
        ShutdownCoordinator {
            request_tx,
            listener_closed_rx,
            graceful_tx,
        },
        ShutdownListener {
            request_rx,
            listener_closed_tx,
            graceful_rx,
        },
    )
}

impl ShutdownCoordinator {
    pub fn request_shutdown(&self) {
        self.request_tx.send_replace(true);
    }

    async fn request_and_wait_for_listener(&self) -> Result<(), Status> {
        let mut listener_closed_rx = self.listener_closed_rx.clone();
        if *listener_closed_rx.borrow() {
            self.graceful_tx.send_replace(true);
            return Ok(());
        }

        self.request_shutdown();
        let listener_result = tokio::time::timeout(
            STOP_LISTENER_TIMEOUT,
            listener_closed_rx.wait_for(|closed| *closed),
        )
        .await;

        // 显式 graceful 阶段必须在 listener-close acknowledgement 之后启动。
        // tonic 会等待当前 Stop RPC 完成回包，再关闭既有 HTTP/2 connection。
        self.graceful_tx.send_replace(true);
        match listener_result {
            Ok(Ok(_)) => Ok(()),
            Ok(Err(_)) => Err(Status::unavailable("gRPC listener 关闭确认通道已断开")),
            Err(_) => Err(Status::deadline_exceeded("关闭 gRPC listener 超时")),
        }
    }
}

impl ShutdownListener {
    #[cfg(test)]
    pub async fn wait_for_request(&mut self) {
        if *self.request_rx.borrow() {
            return;
        }
        let _ = self.request_rx.wait_for(|requested| *requested).await;
    }

    #[cfg(test)]
    pub fn mark_listener_closed(&self) {
        self.listener_closed_tx.send_replace(true);
    }

    pub fn into_parts(
        self,
    ) -> (
        watch::Receiver<bool>,
        watch::Sender<bool>,
        watch::Receiver<bool>,
    ) {
        (self.request_rx, self.listener_closed_tx, self.graceful_rx)
    }
}

impl GeometryRuntimeService {
    pub fn new(bridge: GeometryRuntimeHandle) -> Self {
        Self {
            state: Arc::new(RwLock::new(ServiceState::new_running(
                "Geometry LCM runtime 已启动",
            ))),
            bridge,
            base_frame_id: configured_base_frame(),
            shutdown: None,
        }
    }

    pub fn new_with_shutdown(bridge: GeometryRuntimeHandle, shutdown: ShutdownCoordinator) -> Self {
        let mut service = Self::new(bridge);
        service.shutdown = Some(shutdown);
        service
    }

    /// 构造本进程的 RuntimeDiagnostics(读 ServiceState 算 uptime)
    async fn diagnostics(&self) -> RuntimeDiagnostics {
        let s = self.state.read().await;
        RuntimeDiagnostics::from_handle(&self.bridge, &s.phase, s.started_at_ns)
    }

    async fn ensure_running_admission(&self, operation: &str) -> Result<(), Status> {
        let phase = self.state.read().await.phase.clone();
        if phase != "running" {
            return Err(Status::failed_precondition(format!(
                "{operation} requires runtime phase=running, current={phase}"
            )));
        }
        Ok(())
    }
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn mapping_session_state(
    snapshot: crate::mapping_adapter::MappingSessionSnapshot,
    message: impl Into<String>,
) -> MappingSessionState {
    MappingSessionState {
        success: true,
        phase: snapshot.phase.as_str().to_string(),
        message: message.into(),
        session_id: snapshot.session_id,
        map_name: snapshot.map_name,
        map_version_id: snapshot.map_version_id,
        session_generation: snapshot.session_generation,
        revision: snapshot.revision,
        active: snapshot.active,
        idempotent_replay: snapshot.idempotent_replay,
    }
}

fn mapping_lifecycle_status(error: crate::mapping_adapter::MappingLifecycleError) -> Status {
    use crate::mapping_adapter::MappingLifecycleError;
    match error {
        MappingLifecycleError::InvalidArgument(message) => Status::invalid_argument(message),
        MappingLifecycleError::Conflict(message) => Status::already_exists(message),
        MappingLifecycleError::StaleSession(message)
        | MappingLifecycleError::TerminalSession(message) => Status::failed_precondition(message),
        MappingLifecycleError::GenerationExhausted => {
            Status::resource_exhausted("mapping session generation exhausted")
        }
    }
}

fn decode_checkpoint_grid(
    checkpoint: OccupancyGridData,
) -> Result<nuwax_mapping::OccupancyGrid, String> {
    let meta = checkpoint
        .meta
        .ok_or_else(|| "checkpoint.meta is required".to_string())?;
    if meta.width == 0 || meta.height == 0 {
        return Err("checkpoint width/height must be positive".to_string());
    }
    let expected_size = (meta.width as usize)
        .checked_mul(meta.height as usize)
        .ok_or_else(|| "checkpoint dimensions overflow".to_string())?;
    if checkpoint.data.len() != expected_size {
        return Err(format!(
            "checkpoint data size mismatch: expected={} actual={}",
            expected_size,
            checkpoint.data.len()
        ));
    }
    if !meta.resolution.is_finite() || meta.resolution <= 0.0 {
        return Err("checkpoint resolution must be finite and positive".to_string());
    }
    let frame_id = meta.frame_id.trim().trim_start_matches('/');
    if frame_id != "map" {
        return Err(format!(
            "checkpoint frame must be map, received {}",
            meta.frame_id
        ));
    }
    let data = checkpoint
        .data
        .into_iter()
        .map(|value| value as i8)
        .collect::<Vec<_>>();
    if data.iter().any(|value| !(-1..=100).contains(value)) {
        return Err("checkpoint occupancy values must be within [-1, 100]".to_string());
    }
    let mut grid = nuwax_mapping::OccupancyGrid::with_frame(
        meta.width,
        meta.height,
        f64::from(meta.resolution),
        frame_id,
    );
    grid.origin = [meta.origin_x, meta.origin_y, 0.0];
    grid.ts = meta.timestamp_ns as f64 / 1_000_000_000.0;
    grid.data = data;
    Ok(grid)
}

fn validate_cloud_checkpoint_metadata(checkpoint: &CloudMapCheckpoint) -> Result<(), String> {
    let frame_id = checkpoint.frame_id.trim().trim_start_matches('/');
    if frame_id != "map" {
        return Err(format!(
            "cloud-map checkpoint frame must be map, received {}",
            checkpoint.frame_id
        ));
    }
    if checkpoint.payload_schema_version != crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION {
        return Err(format!(
            "unsupported cloud-map checkpoint schema: expected={} actual={}",
            crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
            checkpoint.payload_schema_version
        ));
    }
    let checksum = checkpoint.xyz_sha256.trim();
    if checksum.len() != 64
        || !checksum.bytes().all(|value| value.is_ascii_hexdigit())
        || crate::map_checksum::sha256_hex(&checkpoint.xyz_f32_le) != checksum.to_ascii_lowercase()
    {
        return Err("cloud-map checkpoint xyz_sha256 mismatch".to_string());
    }
    Ok(())
}

fn exploration_state_from_snapshot(
    snapshot: crate::explore_runtime::ExploreRuntimeSnapshot,
) -> ExplorationState {
    ExplorationState {
        status: snapshot.status.to_string(),
        message: snapshot.message,
        request_id: snapshot.request_id,
        selected_source: snapshot.selected_source,
        last_stop_reason: snapshot.last_stop_reason,
        frontier_count: snapshot.frontier_count,
        revision: snapshot.revision,
        timestamp_ns: now_ns(),
        child_task_id: snapshot.child_task_id,
        map_version_id: snapshot.map_version_id,
        map_revision: snapshot.map_revision,
    }
}

fn explore_config_from_request(
    request: &ExploreRequest,
) -> Result<crate::explore_runtime::ExploreStartConfig, String> {
    let boundary = if request.boundary_line_segments.is_empty() {
        if !request.boundary_frame_id.trim().is_empty() {
            return Err("empty exploration boundary must have an empty frame".to_string());
        }
        None
    } else {
        if request.boundary_frame_id.trim().trim_start_matches('/') != "map" {
            return Err("exploration boundary_frame_id must be map".to_string());
        }
        let segments = request
            .boundary_line_segments
            .iter()
            .map(|segment| BoundarySegment {
                start: Point2 {
                    x: segment.start_x,
                    y: segment.start_y,
                },
                end: Point2 {
                    x: segment.end_x,
                    y: segment.end_y,
                },
            })
            .collect();
        Some(ExplorationBoundary::from_segments(segments).map_err(|error| error.to_string())?)
    };
    let stop_conditions = request
        .stop_conditions
        .as_ref()
        .map(|conditions| crate::explore_runtime::ExploreStopConditions {
            max_coverage_pct: conditions.max_coverage_pct,
            no_frontier_timeout_ms: conditions.no_frontier_timeout_ms,
            max_duration_ms: conditions.max_duration_ms,
            max_goals: conditions.max_goals,
            stop_on_map_revision_change: conditions.stop_on_map_revision_change,
        })
        .unwrap_or_default();
    let arbitration = if request.arbitration.trim().is_empty() {
        std::env::var("NUWAX_EXPLORE_ARBITRATION")
            .unwrap_or_else(|_| "prefer_wavefront".to_string())
    } else {
        request.arbitration.clone()
    };
    Ok(crate::explore_runtime::ExploreStartConfig {
        strategy: if request.strategy.trim().is_empty() {
            "wavefront".to_string()
        } else {
            request.strategy.clone()
        },
        arbitration,
        map_version_id: request.map_version_id.clone(),
        map_revision: request.map_revision,
        motion_admission: MotionAdmissionProof::try_from_wire(
            request.motion_guard_generation,
            &request.motion_guard_boot_nonce,
            request.motion_stop_generation,
        )?,
        boundary,
        stop_conditions,
        max_navigation_retries: request.max_navigation_retries,
        blacklist_radius_m: request.blacklist_radius_m,
    })
}

async fn forward_map_revision(
    tx: &tokio::sync::mpsc::Sender<Result<MapUpdateNotification, Status>>,
    event: MapRevisionEvent,
    occupancy_shm_current: &Arc<RwLock<Option<crate::lcm_runtime::OccupancyShmSnapshot>>>,
) -> bool {
    if event.session_id.trim().is_empty()
        || event.map_version_id.trim().is_empty()
        || event.session_generation == 0
    {
        return tx
            .send(Err(Status::failed_precondition(
                "map revision is missing mapping session identity",
            )))
            .await
            .is_ok();
    }
    if event.frame_id.trim().trim_start_matches('/') != "map" {
        return tx
            .send(Err(Status::failed_precondition(format!(
                "global map subscriber requires frame=map, received {}",
                event.frame_id
            ))))
            .await
            .is_ok();
    }
    if event.size != event.data.len() as u64 {
        return tx
            .send(Err(Status::data_loss(format!(
                "map revision {} size mismatch: declared={}, actual={}",
                event.revision,
                event.size,
                event.data.len()
            ))))
            .await
            .is_ok();
    }
    if crate::map_checksum::sha256_hex(event.data.as_ref()) != event.checksum {
        return tx
            .send(Err(Status::data_loss(format!(
                "map revision {} checksum mismatch",
                event.revision
            ))))
            .await
            .is_ok();
    }

    let shm_ref = match occupancy_shm_current.read().await.clone() {
        Some(snapshot) if snapshot.revision == event.revision => {
            match occupancy_shm_ref(snapshot) {
                Ok(reference) => Some(reference),
                Err(status) => return tx.send(Err(*status)).await.is_ok(),
            }
        }
        Some(_) => None,
        None => None,
    };

    tx.send(Ok(MapUpdateNotification {
        meta: Some(OccupancyGridMeta {
            width: event.width,
            height: event.height,
            resolution: event.resolution,
            origin_x: event.origin_x,
            origin_y: event.origin_y,
            frame_id: event.frame_id,
            revision: event.revision,
            timestamp_ns: event.timestamp_ns,
        }),
        shm_ref,
        data: event.data.as_ref().to_vec(),
        size: event.size,
        schema_version: event.schema_version,
        checksum: event.checksum,
        last_seen_ms: event.last_seen_ms,
        session_id: event.session_id,
        map_version_id: event.map_version_id,
        session_generation: event.session_generation,
    }))
    .await
    .is_ok()
}

fn occupancy_shm_ref(
    snapshot: crate::lcm_runtime::OccupancyShmSnapshot,
) -> Result<ShmRef, Box<Status>> {
    if snapshot.key.trim().is_empty()
        || snapshot.file_size == 0
        || snapshot.revision == 0
        || snapshot.layout_version != 2
        || snapshot.payload_schema_version != 1
        || snapshot.generation == 0
        || snapshot.active_slot >= 2
        || snapshot.payload_size == 0
        || snapshot.payload_size >= snapshot.file_size
    {
        return Err(Box::new(Status::failed_precondition(format!(
            "invalid occupancy SHM v2 reference: layout={} schema={} generation={} slot={} file_size={} payload_size={} revision={}",
            snapshot.layout_version,
            snapshot.payload_schema_version,
            snapshot.generation,
            snapshot.active_slot,
            snapshot.file_size,
            snapshot.payload_size,
            snapshot.revision,
        ))));
    }
    let active_slot = u32::try_from(snapshot.active_slot).map_err(|_| {
        Box::new(Status::failed_precondition(
            "occupancy SHM active slot exceeds uint32",
        ))
    })?;
    Ok(ShmRef {
        key: snapshot.key,
        size: snapshot.file_size,
        revision: snapshot.revision,
        layout_version: snapshot.layout_version,
        payload_schema_version: snapshot.payload_schema_version,
        generation: snapshot.generation,
        active_slot,
        payload_size: snapshot.payload_size,
    })
}

fn cloud_map_shm_ref(notification: &CloudMapNotification) -> Result<ShmRef, Box<Status>> {
    if notification.revision == 0 || notification.session_generation == 0 {
        return Err(Box::new(Status::failed_precondition(format!(
            "invalid cloud-map SHM v2 reference: revision={} generation={}",
            notification.revision, notification.session_generation
        ))));
    }
    let file_size = u64::try_from(crate::pointcloud::CM_TOTAL_SIZE)
        .map_err(|_| Box::new(Status::internal("cloud-map SHM size exceeds uint64")))?;
    let payload_size = u64::from(notification.point_count)
        .checked_mul(crate::pointcloud::CM_POINT_SIZE_BYTES as u64)
        .ok_or_else(|| Box::new(Status::internal("cloud-map payload size overflow")))?;
    let max_payload_size = u64::try_from(crate::pointcloud::CM_SINGLE_BUFFER_SIZE)
        .map_err(|_| Box::new(Status::internal("cloud-map slot size exceeds uint64")))?;
    if payload_size > max_payload_size {
        return Err(Box::new(Status::failed_precondition(format!(
            "cloud-map payload exceeds one SHM slot: payload_size={} max={}",
            payload_size, max_payload_size
        ))));
    }
    let active_slot =
        crate::pointcloud::cloud_map_active_slot(notification.revision).ok_or_else(|| {
            Box::new(Status::failed_precondition(
                "cloud-map revision must be positive",
            ))
        })?;
    Ok(ShmRef {
        key: crate::pointcloud::cloud_map_shm_path()
            .to_string_lossy()
            .into_owned(),
        size: file_size,
        revision: notification.revision,
        layout_version: crate::pointcloud::CM_SHM_VERSION,
        payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
        generation: notification.session_generation,
        active_slot,
        payload_size,
    })
}

fn canonical_xyz_fields() -> Vec<schema::PointField> {
    vec![
        schema::PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: 7,
            count: 1,
        },
        schema::PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: 7,
            count: 1,
        },
        schema::PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: 7,
            count: 1,
        },
    ]
}

fn point_field_size(datatype: u32) -> Option<u32> {
    match datatype {
        1 | 2 => Some(1),
        3 | 4 => Some(2),
        5..=7 => Some(4),
        8 => Some(8),
        _ => None,
    }
}

fn validate_runtime_point_fields(fields: &[PointCloudField], point_step: u32) -> CodecResult<()> {
    for field in fields {
        let size = point_field_size(field.datatype).ok_or_else(|| {
            CodecError::LcmEncode(format!(
                "unsupported point field datatype {} for {:?}",
                field.datatype, field.name
            ))
        })?;
        let field_end = field
            .offset
            .checked_add(size.checked_mul(field.count).ok_or_else(|| {
                CodecError::LcmEncode(format!("point field {:?} size overflow", field.name))
            })?)
            .ok_or_else(|| {
                CodecError::LcmEncode(format!("point field {:?} offset overflow", field.name))
            })?;
        if field.name.is_empty() || field.count == 0 || field_end > point_step {
            return Err(CodecError::LcmEncode(format!(
                "invalid point field name={:?} offset={} datatype={} count={} point_step={}",
                field.name, field.offset, field.datatype, field.count, point_step
            )));
        }
    }

    for (name, offset) in [("x", 0), ("y", 4), ("z", 8)] {
        let field = fields.iter().find(|field| field.name == name);
        if !matches!(
            field,
            Some(field) if field.offset == offset && field.datatype == 7 && field.count == 1
        ) {
            return Err(CodecError::LcmEncode(format!(
                "runtime requires canonical {name} float32 field at offset {offset}"
            )));
        }
    }
    Ok(())
}

fn encode_pointcloud_request_payload(request: &PublishPointCloudRequest) -> CodecResult<Vec<u8>> {
    let point_step = usize::try_from(request.point_step)
        .map_err(|_| CodecError::LcmEncode("point_step exceeds usize".into()))?;
    if point_step < 12 || request.points.len() % point_step != 0 {
        return Err(CodecError::LcmEncode(format!(
            "point cloud data length {} is not aligned to point_step {}",
            request.points.len(),
            request.point_step
        )));
    }

    let derived_point_count = request.points.len() / point_step;
    if request.point_count != 0 && request.point_count as usize != derived_point_count {
        return Err(CodecError::LcmEncode(format!(
            "point_count {} does not match data-derived count {}",
            request.point_count, derived_point_count
        )));
    }

    let fields = if request.fields.is_empty() {
        canonical_xyz_fields()
    } else {
        validate_runtime_point_fields(&request.fields, request.point_step)?;
        request
            .fields
            .iter()
            .map(|field| {
                Ok(schema::PointField {
                    name: field.name.clone(),
                    offset: i32::try_from(field.offset).map_err(|_| {
                        CodecError::LcmEncode("point field offset exceeds int32".into())
                    })?,
                    datatype: u8::try_from(field.datatype).map_err(|_| {
                        CodecError::LcmEncode("point field datatype exceeds uint8".into())
                    })?,
                    count: i32::try_from(field.count).map_err(|_| {
                        CodecError::LcmEncode("point field count exceeds int32".into())
                    })?,
                })
            })
            .collect::<CodecResult<Vec<_>>>()?
    };

    let legacy_request = request.frame_id.is_empty()
        && request.timestamp_ns == 0
        && request.fields.is_empty()
        && request.point_count == 0;
    let stamp_sec = i32::try_from(request.timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("point cloud timestamp seconds exceed int32".into()))?;
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema::Header {
            seq: 0,
            stamp_sec,
            stamp_nsec: (request.timestamp_ns % 1_000_000_000) as i32,
            frame_id: if request.frame_id.is_empty() {
                "map".to_string()
            } else {
                request.frame_id.clone()
            },
        },
        height: 1,
        width: i32::try_from(derived_point_count)
            .map_err(|_| CodecError::LcmEncode("point cloud width exceeds int32".into()))?,
        fields,
        is_bigendian: false,
        point_step: i32::try_from(request.point_step)
            .map_err(|_| CodecError::LcmEncode("point_step exceeds int32".into()))?,
        row_step: i32::try_from(request.points.len())
            .map_err(|_| CodecError::LcmEncode("point cloud row_step exceeds int32".into()))?,
        data: request.points.clone(),
        is_dense: legacy_request || request.is_dense,
    })
}

/// 把内部 `NavState`(crate::lcm_runtime::NavState)转成 gRPC
/// `proto::NavigationState`,避免 server.rs 直接依赖 internal type。
fn nav_state_to_proto(update: NavigationUpdate) -> NavigationState {
    let NavigationUpdate {
        state: nav,
        feedback,
        producer_runtime_generation,
    } = update;
    NavigationState {
        status: nav.status,
        message: nav.message,
        goal_id: nav.goal_id,
        distance_remaining: nav.distance_remaining,
        timestamp_ns: nav.timestamp_ns,
        cancel_phase: feedback.cancel_phase,
        cancel_generation: feedback.cancel_generation,
        actuator_zeroed: feedback.actuator_zeroed,
        reason_code: feedback.reason_code,
        runtime_generation: feedback.runtime_generation,
        guard_generation: feedback.guard_generation,
        sink_ready: feedback.sink_ready,
        planner_map_revision: feedback.planner_map_revision,
        planner_map_frame: feedback.planner_map_frame,
        planner_pose_frame: feedback.planner_pose_frame,
        zero_ack_id: feedback.zero_ack_id,
        zero_applied_monotonic_ns: feedback.zero_applied_monotonic_ns,
        owner_kind: feedback.owner_kind,
        parent_request_id: feedback.parent_request_id,
        goal_generation: feedback.goal_generation,
        path_revision: feedback.path_revision,
        goal_payload_sha256: feedback.goal_payload_sha256.to_vec(),
        path_payload_sha256: feedback.path_payload_sha256.to_vec(),
        map_version_id: feedback.map_version_id,
        producer_runtime_generation,
    }
}

#[tonic::async_trait]
impl GeometryRuntime for GeometryRuntimeService {
    // ========================================================================
    // 控制面
    // ========================================================================

    async fn start(&self, _req: Request<StartRequest>) -> Result<Response<StartResult>, Status> {
        Ok(Response::new(StartResult {
            success: true,
            message: "已在运行".into(),
        }))
    }

    async fn stop(&self, _req: Request<StopRequest>) -> Result<Response<StopResult>, Status> {
        info!("收到 Stop 指令");

        let shutdown = self.shutdown.as_ref().ok_or_else(|| {
            Status::failed_precondition("runtime 未配置进程级 shutdown coordinator")
        })?;
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        {
            let mut state = self.state.write().await;
            if state.phase == "stopped" {
                return Ok(Response::new(StopResult { success: true }));
            }
            state.phase = "stopping".into();
            state.message = "正在等待探索执行器安全停车".into();
        }

        let request_id = self.bridge.explore.snapshot().request_id;
        let stop_snapshot = self.bridge.explore.stop(&request_id).await;
        let safety_wait = tokio::time::timeout(STOP_EXPLORATION_SAFETY_TIMEOUT, async {
            let mut snapshot = stop_snapshot;
            loop {
                if !self.bridge.explore.blocks_mapping_mutation().await {
                    return Ok(snapshot);
                }
                if matches!(
                    snapshot.status,
                    crate::explore_runtime::STATUS_FAILED
                        | crate::explore_runtime::STATUS_REJECTED
                ) {
                    return Err(Status::failed_precondition(format!(
                        "exploration safety ownership retained: request_id={} child_task_id={} message={}",
                        snapshot.request_id, snapshot.child_task_id, snapshot.message
                    )));
                }
                tokio::time::sleep(STOP_EXPLORATION_SAFETY_POLL).await;
                snapshot = self.bridge.explore.snapshot();
            }
        })
        .await;
        match safety_wait {
            Ok(Ok(_)) => {}
            Ok(Err(error)) => {
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }
            Err(_) => {
                if self.bridge.explore.blocks_mapping_mutation().await {
                    let snapshot = self.bridge.explore.snapshot();
                    let error = Status::deadline_exceeded(format!(
                        "timed out waiting for exploration safety ownership release: request_id={} child_task_id={} status={}",
                        snapshot.request_id, snapshot.child_task_id, snapshot.status
                    ));
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
            }
        }

        if let Some(active_lease) = self.bridge.nav.active_goal_lease().await {
            {
                let mut state = self.state.write().await;
                state.message = "正在等待导航执行器安全停车".into();
            }
            let CancelOutcome { state: nav, lease } = nav_cancel(
                &self.bridge,
                &self.bridge.nav,
                crate::nav_runtime::CancelGoalInput {
                    goal_id: active_lease.session_id.clone(),
                    owner_kind: active_lease.owner_kind,
                    parent_request_id: active_lease.parent_request_id.clone(),
                    goal_generation: active_lease.goal_generation,
                    producer_runtime_generation: active_lease.producer_runtime_generation,
                },
            )
            .await;
            {
                let mut shared = self.bridge.state.write().await;
                shared.nav_state = nav.clone();
            }
            let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
            let _ = self.bridge.nav_state_tx.send(update);

            let cancel_after_dispatch = self.bridge.nav.cancel_snapshot();
            let expected_cancel_lease = cancel_after_dispatch.lease.clone();
            if expected_cancel_lease.is_none()
                && self.bridge.nav.active_goal_lease().await.is_some()
            {
                let error = Status::failed_precondition(format!(
                    "navigation cancel did not establish an exact safety ticket: goal_id={} phase={:?} message={}",
                    active_lease.session_id,
                    cancel_after_dispatch.phase,
                    cancel_after_dispatch.message
                ));
                let mut state = self.state.write().await;
                state.phase = "failed".into();
                state.message = error.message().to_string();
                return Err(error);
            }

            let navigation_wait =
                tokio::time::timeout(STOP_EXPLORATION_SAFETY_TIMEOUT, async {
                    loop {
                        let cancel = self.bridge.nav.cancel_snapshot();
                        let active = self.bridge.nav.active_goal_lease().await;
                        if expected_cancel_lease.is_none() && active.is_none() {
                            return Ok(());
                        }
                        if active.is_none()
                            && cancel.phase == crate::nav_runtime::CancelPhase::Zeroed
                            && cancel.lease == expected_cancel_lease
                        {
                            return Ok(());
                        }
                        if matches!(
                            cancel.phase,
                            crate::nav_runtime::CancelPhase::Failed
                                | crate::nav_runtime::CancelPhase::TimedOut
                        ) {
                            return Err(Status::failed_precondition(format!(
                                "navigation safety ownership retained: goal_id={} cancel_generation={} phase={:?} message={}",
                                active_lease.session_id,
                                cancel.generation,
                                cancel.phase,
                                cancel.message
                            )));
                        }
                        tokio::time::sleep(STOP_EXPLORATION_SAFETY_POLL).await;
                    }
                })
                .await;
            match navigation_wait {
                Ok(Ok(())) => {}
                Ok(Err(error)) => {
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
                Err(_) => {
                    let cancel = self.bridge.nav.cancel_snapshot();
                    let error = Status::deadline_exceeded(format!(
                        "timed out waiting for navigation safety ownership release: goal_id={} cancel_generation={} phase={:?}",
                        active_lease.session_id, cancel.generation, cancel.phase
                    ));
                    let mut state = self.state.write().await;
                    state.phase = "failed".into();
                    state.message = error.message().to_string();
                    return Err(error);
                }
            }
        }
        self.bridge.shutdown_explore_scheduler().await;

        {
            let mut state = self.state.write().await;
            state.message = "正在关闭 gRPC listener".into();
        }

        if let Err(error) = shutdown.request_and_wait_for_listener().await {
            let mut state = self.state.write().await;
            state.phase = "failed".into();
            state.message = error.message().to_string();
            return Err(error);
        }

        {
            let mut state = self.state.write().await;
            state.phase = "stopped".into();
            state.message = "gRPC listener 已关闭".into();
        }
        Ok(Response::new(StopResult { success: true }))
    }

    async fn get_status(&self, _req: Request<Empty>) -> Result<Response<RuntimeStatus>, Status> {
        let s = self.state.read().await;
        let diag = RuntimeDiagnostics::from_handle(&self.bridge, &s.phase, s.started_at_ns);
        Ok(Response::new(RuntimeStatus {
            phase: diag.phase,
            message: s.message.clone(),
            uptime_ms: diag.uptime_ms,
            rss_kb: diag.rss_kb,
            lcm_backend: diag.lcm_backend,
        }))
    }

    async fn health_check(
        &self,
        _req: Request<HealthCheckRequest>,
    ) -> Result<Response<HealthCheckResponse>, Status> {
        let diag = self.diagnostics().await;
        // Phase 1 fail-fast + Phase 2 diagnostics 收敛:health_check 统一
        // 从 RuntimeDiagnostics 读 healthy + lcm_backend。
        // HealthCheck 调用方可从 `lcm_backend` 字段 grep
        // "LCM_BACKEND_UNAVAILABLE" 关键字做 CI gate。
        Ok(Response::new(HealthCheckResponse {
            healthy: diag.healthy,
            phase: diag.phase,
            uptime_ms: diag.uptime_ms,
            rss_kb: diag.rss_kb,
            video_frame_sequence: 0,
            video_fps_estimate: 0,
            lcm_backend: diag.lcm_backend,
        }))
    }

    // ========================================================================
    // 定位
    // ========================================================================

    async fn get_current_pose(&self, _req: Request<PoseRequest>) -> Result<Response<Pose>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(Pose {
            position: Some(Vec3 {
                x: s.pose.x,
                y: s.pose.y,
                z: s.pose.z,
            }),
            orientation: Some(Quat {
                x: s.pose.qx,
                y: s.pose.qy,
                z: s.pose.qz,
                w: s.pose.qw,
            }),
            frame_id: s.pose.frame_id.clone(),
            timestamp_ns: s.pose.timestamp_ns,
        }))
    }

    type SubscribePoseStream = tokio_stream::wrappers::ReceiverStream<Result<PoseUpdate, Status>>;
    async fn subscribe_pose(
        &self,
        _req: Request<PoseRequest>,
    ) -> Result<Response<Self::SubscribePoseStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(32);
        let state = self.bridge.state.clone();
        tokio::spawn(async move {
            let mut seq = 0u64;
            // 变化驱动：仅当位姿实际更新（timestamp_ns 变化）时才推送帧；
            // 避免静止时以 100ms 周期推送大量重复帧。last_stamp 初值 u64::MAX
            // 确保首个有效帧（含 stamp==0 的早期态）至少推送一次后再去重。
            let mut last_stamp = u64::MAX;
            loop {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                let update = {
                    let s = state.read().await;
                    if s.pose.timestamp_ns == last_stamp {
                        continue;
                    }
                    last_stamp = s.pose.timestamp_ns;
                    seq += 1;
                    PoseUpdate {
                        pose: Some(Pose {
                            position: Some(Vec3 {
                                x: s.pose.x,
                                y: s.pose.y,
                                z: s.pose.z,
                            }),
                            orientation: Some(Quat {
                                x: s.pose.qx,
                                y: s.pose.qy,
                                z: s.pose.qz,
                                w: s.pose.qw,
                            }),
                            frame_id: s.pose.frame_id.clone(),
                            timestamp_ns: s.pose.timestamp_ns,
                        }),
                        sequence: seq,
                    }
                };
                if tx.send(Ok(update)).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }

    async fn get_frame_tree(&self, _req: Request<Empty>) -> Result<Response<FrameTree>, Status> {
        let s = self.bridge.state.read().await;
        let transforms = s
            .latest_transform_entries()
            .into_iter()
            .map(|t| Transform {
                parent_frame: t.parent,
                child_frame: t.child,
                translation: Some(Vec3 {
                    x: t.tx,
                    y: t.ty,
                    z: t.tz,
                }),
                rotation: Some(Quat {
                    x: t.rx,
                    y: t.ry,
                    z: t.rz,
                    w: t.rw,
                }),
                timestamp_ns: t.stamp_ns,
            })
            .collect();
        Ok(Response::new(FrameTree { transforms }))
    }

    // ========================================================================
    // 地图
    // ========================================================================

    async fn start_mapping(
        &self,
        req: Request<StartMappingRequest>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let request = req.into_inner();
        if request.session_id.trim().is_empty()
            || request.map_name.trim().is_empty()
            || request.map_version_id.trim().is_empty()
        {
            return Err(Status::invalid_argument(
                "session_id, map_name and map_version_id must not be empty",
            ));
        }
        if !self.bridge.lcm_backend_available {
            return Err(Status::unavailable("mapping LCM backend is unavailable"));
        }
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("StartMapping").await?;

        let current = adapter.lock().snapshot();
        if current.active
            || (current.phase == crate::mapping_adapter::MappingSessionPhase::Stopped
                && current.session_id == request.session_id)
        {
            let snapshot = adapter
                .lock()
                .start_session(
                    &request.session_id,
                    &request.map_name,
                    &request.map_version_id,
                )
                .map_err(mapping_lifecycle_status)?;
            return Ok(Response::new(mapping_session_state(
                snapshot,
                "mapping session already active",
            )));
        }

        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "exploration safety ownership must be released before starting a new mapping session: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }

        {
            let mut shared = self.bridge.state.write().await;
            shared.map = crate::lcm_runtime::MapState::default();
        }
        self.bridge.map_revision_latest_tx.send_replace(None);
        *self.bridge.occupancy_shm_current.write().await = None;

        let snapshot = adapter
            .lock()
            .start_session(
                &request.session_id,
                &request.map_name,
                &request.map_version_id,
            )
            .map_err(mapping_lifecycle_status)?;
        if let Some(writer) = &self.bridge.occupancy_shm {
            if let Err(error) = writer.lock().begin_session(snapshot.session_generation) {
                let _ = adapter.lock().stop_session(&snapshot.session_id);
                return Err(Status::internal(format!(
                    "occupancy SHM session fence failed: {error}"
                )));
            }
        }
        // 3D SHM is fail-soft: a missing writer must not roll back the
        // authoritative 2D mapping session. When available, publish an empty
        // revision immediately so readers cannot retain the prior session map.
        let _ = self
            .bridge
            .begin_cloud_map_session(snapshot.session_generation)
            .await;
        Ok(Response::new(mapping_session_state(
            snapshot,
            "mapping session active",
        )))
    }

    async fn stop_mapping(
        &self,
        req: Request<StopMappingRequest>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let request = req.into_inner();
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("StopMapping").await?;
        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "exploration safety ownership must be released before StopMapping: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }
        let snapshot = adapter
            .lock()
            .stop_session(&request.session_id)
            .map_err(mapping_lifecycle_status)?;
        let reason = request.reason.trim();
        Ok(Response::new(mapping_session_state(
            snapshot,
            if reason.is_empty() {
                "mapping session stopped".to_string()
            } else {
                format!("mapping session stopped: {reason}")
            },
        )))
    }

    async fn get_mapping_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        let snapshot = adapter.lock().snapshot();
        Ok(Response::new(mapping_session_state(
            snapshot,
            "mapping session state",
        )))
    }

    async fn activate_map_version(
        &self,
        req: Request<ActivateMapVersionRequest>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let mut request = req.into_inner();
        if request.session_id.trim().is_empty()
            || request.map_name.trim().is_empty()
            || request.map_version_id.trim().is_empty()
        {
            return Err(Status::invalid_argument(
                "session_id, map_name and map_version_id must not be empty",
            ));
        }
        if !self.bridge.lcm_backend_available {
            return Err(Status::unavailable("mapping LCM backend is unavailable"));
        }
        let checkpoint = decode_checkpoint_grid(
            request
                .checkpoint
                .take()
                .ok_or_else(|| Status::invalid_argument("checkpoint is required"))?,
        )
        .map_err(Status::invalid_argument)?;
        let cloud_checkpoint = request.cloud_checkpoint.take();
        if let Some(checkpoint) = cloud_checkpoint.as_ref() {
            validate_cloud_checkpoint_metadata(checkpoint).map_err(Status::invalid_argument)?;
            let writer = self.bridge.cloud_map_writer.lock();
            let writer = writer
                .as_ref()
                .ok_or_else(|| Status::unavailable("cloud-map SHM writer is unavailable"))?;
            writer
                .validate_checkpoint(
                    checkpoint.voxel_size,
                    checkpoint.point_count,
                    &checkpoint.xyz_f32_le,
                )
                .map_err(Status::invalid_argument)?;
        }
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("ActivateMapVersion").await?;
        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "exploration safety ownership must be released before ActivateMapVersion: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }

        let current = adapter.lock().snapshot();
        if current.active
            && current.session_id == request.session_id
            && current.map_name == request.map_name
            && current.map_version_id == request.map_version_id
            && current.revision > 0
        {
            let mut replay = current;
            replay.idempotent_replay = true;
            return Ok(Response::new(mapping_session_state(
                replay,
                "map checkpoint already active",
            )));
        }

        let started = adapter
            .lock()
            .start_session(
                &request.session_id,
                &request.map_name,
                &request.map_version_id,
            )
            .map_err(mapping_lifecycle_status)?;
        if let Some(writer) = &self.bridge.occupancy_shm {
            if let Err(error) = writer.lock().begin_session(started.session_generation) {
                let _ = adapter.lock().stop_session(&started.session_id);
                return Err(Status::internal(format!(
                    "occupancy SHM checkpoint fence failed: {error}"
                )));
            }
        }
        let revision_floor = self.bridge.state.read().await.map.revision;
        let (map, event) = match adapter
            .lock()
            .install_checkpoint(checkpoint, revision_floor)
        {
            Ok(result) => result,
            Err(error) => {
                let _ = adapter.lock().stop_session(&started.session_id);
                return Err(Status::invalid_argument(error));
            }
        };
        let cloud_update = if let Some(checkpoint) = cloud_checkpoint.as_ref() {
            let installed = {
                let mut writer = self.bridge.cloud_map_writer.lock();
                let writer = writer.as_mut().ok_or_else(|| {
                    Status::unavailable("cloud-map SHM writer became unavailable")
                })?;
                writer.install_checkpoint(
                    started.session_generation,
                    checkpoint.voxel_size,
                    checkpoint.point_count,
                    &checkpoint.xyz_f32_le,
                )
            };
            let (revision, point_count, voxel_size) = match installed {
                Ok(installed) => installed,
                Err(error) => {
                    let _ = adapter.lock().stop_session(&started.session_id);
                    return Err(Status::internal(format!(
                        "cloud-map checkpoint install failed after validation: {error}"
                    )));
                }
            };
            Some(CloudMapNotification {
                revision,
                point_count,
                voxel_size,
                session_generation: started.session_generation,
            })
        } else {
            None
        };
        {
            let mut shared = self.bridge.state.write().await;
            shared.map = map;
            if let Some(update) = cloud_update.as_ref() {
                shared.cloud_map_revision = update.revision;
            }
        }
        self.bridge.map_revision_latest_tx.send_replace(None);
        *self.bridge.occupancy_shm_current.write().await = None;
        if let Some(update) = cloud_update {
            let _ = self.bridge.cloud_map_tx.send(update);
        } else {
            let _ = self
                .bridge
                .begin_cloud_map_session(started.session_generation)
                .await;
        }
        let _ = self.bridge.map_revision_tx.send(event);
        let snapshot = adapter.lock().snapshot();
        Ok(Response::new(mapping_session_state(
            snapshot,
            "map checkpoint activated",
        )))
    }

    async fn get_occupancy_grid(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<OccupancyGridMeta>, Status> {
        let s = self.bridge.state.read().await;
        Ok(Response::new(OccupancyGridMeta {
            width: s.map.width,
            height: s.map.height,
            resolution: s.map.resolution,
            origin_x: s.map.origin_x,
            origin_y: s.map.origin_y,
            frame_id: s.map.frame_id.clone(),
            revision: s.map.revision,
            timestamp_ns: s.map.timestamp_ns,
        }))
    }

    async fn get_occupancy_grid_data(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<OccupancyGridData>, Status> {
        let (meta, raw_data) = {
            let s = self.bridge.state.read().await;
            (
                OccupancyGridMeta {
                    width: s.map.width,
                    height: s.map.height,
                    resolution: s.map.resolution,
                    origin_x: s.map.origin_x,
                    origin_y: s.map.origin_y,
                    frame_id: s.map.frame_id.clone(),
                    revision: s.map.revision,
                    timestamp_ns: s.map.timestamp_ns,
                },
                s.map.data.clone(),
            )
        };
        Ok(Response::new(OccupancyGridData {
            meta: Some(meta),
            data: raw_data.iter().map(|&v| v as u8).collect(),
        }))
    }

    async fn get_raw_occupancy_grid_data(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<OccupancyGridData>, Status> {
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?;
        let raw_map = adapter
            .lock()
            .raw_map_snapshot()
            .ok_or_else(|| Status::failed_precondition("physical occupancy grid is unavailable"))?;
        Ok(Response::new(OccupancyGridData {
            meta: Some(OccupancyGridMeta {
                width: raw_map.width,
                height: raw_map.height,
                resolution: raw_map.resolution,
                origin_x: raw_map.origin_x,
                origin_y: raw_map.origin_y,
                frame_id: raw_map.frame_id,
                revision: raw_map.revision,
                timestamp_ns: raw_map.timestamp_ns,
            }),
            data: raw_map.data.iter().map(|&value| value as u8).collect(),
        }))
    }

    async fn get_occupancy_grid_shm(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ShmRef>, Status> {
        // SHM sidecar 只有 map-revision fanout task 可写。本 RPC 只返回最新一次
        // 成功发布的不可变引用，size 是 mmap 文件长度而非 occupancy payload 长度。
        if self.bridge.occupancy_shm.is_none() {
            return Err(Status::unavailable(
                "occupancy_shm 未初始化 (LCM backend 不可用,导致 SHM 写入被禁用)",
            ));
        }
        let snapshot = self
            .bridge
            .occupancy_shm_current
            .read()
            .await
            .clone()
            .ok_or_else(|| {
                Status::failed_precondition("occupancy grid 尚未由 map-revision SHM writer 提交")
            })?;
        Ok(Response::new(
            occupancy_shm_ref(snapshot).map_err(|status| *status)?,
        ))
    }

    type SubscribeMapUpdatesStream =
        tokio_stream::wrappers::ReceiverStream<Result<MapUpdateNotification, Status>>;
    async fn subscribe_map_updates(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeMapUpdatesStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let mut map_revision_rx = self.bridge.subscribe_map_revisions();
        let occupancy_shm_current = self.bridge.occupancy_shm_current.clone();
        tokio::spawn(async move {
            let mut last_generation = 0u64;
            let mut last_rev = 0u64;
            let initial = map_revision_rx.borrow_and_update().clone();
            if let Some(event) = initial {
                last_generation = event.session_generation;
                last_rev = event.revision;
                if !forward_map_revision(&tx, event, &occupancy_shm_current).await {
                    return;
                }
            }

            while map_revision_rx.changed().await.is_ok() {
                let Some(event) = map_revision_rx.borrow_and_update().clone() else {
                    continue;
                };
                if event.session_generation < last_generation
                    || (event.session_generation == last_generation && event.revision <= last_rev)
                {
                    continue;
                }
                last_generation = event.session_generation;
                last_rev = event.revision;
                if !forward_map_revision(&tx, event, &occupancy_shm_current).await {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }

    type SubscribeCloudMapUpdatesStream =
        tokio_stream::wrappers::ReceiverStream<Result<CloudMapShmRef, Status>>;
    async fn subscribe_cloud_map_updates(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeCloudMapUpdatesStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.cloud_map_tx.subscribe();
        tokio::spawn(async move {
            while let Ok(notif) = broadcast_rx.recv().await {
                let shm_ref = match cloud_map_shm_ref(&notif) {
                    Ok(reference) => reference,
                    Err(status) => {
                        let _ = tx.send(Err(*status)).await;
                        break;
                    }
                };
                let msg = CloudMapShmRef {
                    shm_ref: Some(shm_ref),
                    point_count: notif.point_count,
                    voxel_size: notif.voxel_size,
                };
                if tx.send(Ok(msg)).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }

    // ========================================================================
    // 导航兼容
    // ========================================================================
    //
    // Phase 4:`/nav/goal` 与 `/nav/cancel` publisher 已由 nav_runtime.rs 接管
    // (`publish_goal_frame` / `publish_cancel_frame`),通过 LcmTransport.publish
    // 推到 LCM typed channel。本节保留 setter RPC 入口供 Python/前端调用。
    // 删除 Phase 3 的 "UNIMPLEMENTED" placeholder 注释 —— readiness 与状态机
    // 都在 nav_runtime.rs 内真实执行(`set_goal` / `cancel`)。
    //
    // Phase 7:`SetNavigationGoal` 在 real production profile 且 cmd_vel_sink=false
    // 时被 nav_runtime::set_goal 拒绝;readiness 全量字段由
    // `mapping_adapter::MappingReadiness` / `nav_runtime::NavigationReadinessDetail`
    // / `explore_runtime::ExplorationReadinessDetail` 暴露,见
    // `docs/.../11_readiness_收敛.md`。

    async fn set_navigation_goal(
        &self,
        req: Request<NavigationGoal>,
    ) -> Result<Response<NavigationState>, Status> {
        // Phase 4:接入 nav_runtime。流程:
        // 1) 归一化 NavigationGoal -> SetGoalInput(自动生成 UUID 当 goal_id 空)
        // 2) 调 set_goal():readiness 校验 -> 仅 publish /nav/goal typed frame
        //    -> 返回 ACCEPTED；规划/跟随/cmd_vel 全部归真实 stack 所有
        // 3) 把 NavState 同步到 SharedState.nav_state + broadcast 通道
        let ng = req.get_ref();
        let pose = ng.pose.as_ref();
        let pos = pose.and_then(|p| p.position.as_ref());
        let orient = pose.and_then(|p| p.orientation.as_ref());
        let target = PoseState {
            x: pos.map(|v| v.x).unwrap_or(0.0),
            y: pos.map(|v| v.y).unwrap_or(0.0),
            z: pos.map(|v| v.z).unwrap_or(0.0),
            qx: orient.map(|v| v.x).unwrap_or(0.0),
            qy: orient.map(|v| v.y).unwrap_or(0.0),
            qz: orient.map(|v| v.z).unwrap_or(0.0),
            qw: orient.map(|v| v.w).unwrap_or(1.0),
            frame_id: pose.map(|p| p.frame_id.clone()).unwrap_or_default(),
            timestamp_ns: pose.map(|p| p.timestamp_ns).unwrap_or(now_ns()),
        };
        let goal_id = ensure_goal_id(&ng.goal_id);
        let owner_kind = match ng.owner_kind {
            0 | 1 => nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            _ => {
                return Err(Status::invalid_argument(
                    "public SetNavigationGoal cannot claim exploration ownership",
                ));
            }
        };
        let parent_request_id = if ng.parent_request_id.trim().is_empty() {
            goal_id.clone()
        } else {
            ng.parent_request_id.trim().to_string()
        };
        let map_revision = if ng.map_revision == 0 {
            self.bridge.state.read().await.map.revision
        } else {
            ng.map_revision
        };
        let frame_id = if ng.frame_id.is_empty() {
            "map".to_string()
        } else {
            ng.frame_id.clone()
        };
        let motion_admission = MotionAdmissionProof::try_from_wire(
            ng.motion_guard_generation,
            &ng.motion_guard_boot_nonce,
            ng.motion_stop_generation,
        )
        .map_err(Status::invalid_argument)?;
        let input = SetGoalInput {
            goal_id: goal_id.clone(),
            owner_kind,
            parent_request_id,
            goal_generation: ng.goal_generation,
            map_version_id: ng.map_version_id.clone(),
            map_revision,
            motion_admission,
            frame_id,
            target,
        };
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("SetNavigationGoal").await?;
        if self.bridge.explore.blocks_mapping_mutation().await {
            let exploration = self.bridge.explore.snapshot();
            return Err(Status::failed_precondition(format!(
                "public navigation cannot start while exploration owns admission: status={} request_id={} child_task_id={}",
                exploration.status, exploration.request_id, exploration.child_task_id
            )));
        }
        let SetGoalOutcome {
            state: nav, lease, ..
        } = set_goal(&self.bridge, &self.bridge.nav, input).await;
        // 同步到 SharedState.nav_state,供 SubscribeNavigationState 推送。
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
        let _ = self.bridge.nav_state_tx.send(update.clone());
        Ok(Response::new(nav_state_to_proto(update)))
    }

    async fn cancel_navigation(
        &self,
        req: Request<CancelNavigationRequest>,
    ) -> Result<Response<NavigationState>, Status> {
        // Phase 4:接入 nav_runtime.cancel。流程:
        // 1) publish /nav/cancel typed frame
        // 2) best-effort publish zero cmd_vel
        // 3) 返回 PENDING；只有观测到 stack /nav/state 才进入 terminal CANCELLED
        let request = req.get_ref();
        let CancelOutcome { state: nav, lease } = nav_cancel(
            &self.bridge,
            &self.bridge.nav,
            crate::nav_runtime::CancelGoalInput {
                goal_id: request.goal_id.trim().to_string(),
                owner_kind: request.owner_kind,
                parent_request_id: request.parent_request_id.trim().to_string(),
                goal_generation: request.goal_generation,
                producer_runtime_generation: request.producer_runtime_generation,
            },
        )
        .await;
        {
            let mut s = self.bridge.state.write().await;
            s.nav_state = nav.clone();
        }
        let update = NavigationUpdate::from_local(nav, &self.bridge.nav, lease).await;
        let _ = self.bridge.nav_state_tx.send(update.clone());
        Ok(Response::new(nav_state_to_proto(update)))
    }

    type SubscribeNavigationStateStream =
        tokio_stream::wrappers::ReceiverStream<Result<NavigationState, Status>>;
    async fn subscribe_navigation_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeNavigationStateStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(8);
        let mut broadcast_rx = self.bridge.nav_state_tx.subscribe();
        tokio::spawn(async move {
            while let Ok(update) = broadcast_rx.recv().await {
                let state = nav_state_to_proto(update);
                if tx.send(Ok(state)).await.is_err() {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }

    async fn get_navigation_readiness(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<NavigationReadiness>, Status> {
        // Phase 4:细分 readiness 子项(LCM backend + tf + costmap +
        // planner profile + cmd_vel sink)由 nav_runtime::compute_readiness
        // 计算,JSON 序列化到 `reason` 字段。Python 侧
        // `get_navigation_readiness` 解析 JSON 返回 dict(满足"costmap、
        // tf、planner、cmd_vel sink、LCM backend 子项"要求)。
        // 顶层 ready/reason 仍走 runtime_diagnostics 的粗略判定,保证
        // CI/CLI 现有 grep "nav ready" 的守门不会失效。
        let r = runtime_diagnostics::navigation_readiness(&self.bridge);
        let detail = compute_readiness(&self.bridge, &self.bridge.nav).await;
        let forbidden_zone_ack = self
            .bridge
            .mapping_adapter
            .as_ref()
            .map(|adapter| adapter.lock().forbidden_zone_ack())
            .unwrap_or_default();
        let json = readiness_to_reason(&detail);
        Ok(Response::new(NavigationReadiness {
            ready: r.ready && detail.ready,
            reason: format!("{}; details={}", r.reason, json),
            reason_code: detail.reason_code.as_str().to_string(),
            blockers: detail
                .blockers
                .iter()
                .map(|blocker| blocker.as_str().to_string())
                .collect(),
            tf_generation: detail.tf_generation,
            planner_generation: detail.planner_generation,
            motion_sink_generation: detail.motion_sink_generation,
            map_frame: detail.map_frame,
            pose_frame: detail.pose_frame,
            forbidden_zone_map_version_id: forbidden_zone_ack.map_version_id,
            forbidden_zone_generation: forbidden_zone_ack.generation,
            forbidden_zone_signature: forbidden_zone_ack.signature,
            forbidden_zone_applied_revision: forbidden_zone_ack.applied_revision,
            planner_costmap_revision: detail.costmap_revision,
        }))
    }

    /// Phase 7:mapping 板块 readiness 综合快照。
    async fn get_mapping_readiness(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<MappingReadiness>, Status> {
        use crate::mapping_adapter::MappingReadiness as MappingReadinessDetail;
        // parking_lot Mutex guard 不能直接跨 await 持有,这里把锁解开再算。
        let (snapshot, lifecycle) = {
            let lcm_backend = self.bridge.lcm_backend_available;
            match &self.bridge.mapping_adapter {
                Some(arc) => {
                    let g = arc.lock();
                    (
                        MappingReadinessDetail::compute(Some(&*g), lcm_backend),
                        Some(g.snapshot()),
                    )
                }
                None => (MappingReadinessDetail::compute(None, lcm_backend), None),
            }
        };
        let message_json = serde_json::to_string(&serde_json::json!({
            "lcm_backend": snapshot.lcm_backend,
            "adapter_present": snapshot.adapter_present,
            "odom_count": snapshot.odom_count,
            "pointcloud_count": snapshot.pointcloud_count,
            "last_occupancy_revision": snapshot.last_occupancy_revision,
            "accumulated_point_count": snapshot.accumulated_point_count,
            "global_consumer_ready": snapshot.global_consumer_ready,
            "frame_generation": snapshot.frame_generation,
            "last_error": snapshot.last_error,
            "mapping_phase": lifecycle.as_ref().map(|state| state.phase.as_str()),
            "mapping_session_id": lifecycle.as_ref().map(|state| state.session_id.as_str()),
            "map_name": lifecycle.as_ref().map(|state| state.map_name.as_str()),
            "map_version_id": lifecycle.as_ref().map(|state| state.map_version_id.as_str()),
            "session_generation": lifecycle.as_ref().map(|state| state.session_generation),
            "mapping_active": lifecycle.as_ref().map(|state| state.active).unwrap_or(false),
        }))
        .unwrap_or_else(|e| format!("{{\"serialization_error\":\"{}\"}}", e));
        Ok(Response::new(MappingReadiness {
            ready: snapshot.ready,
            message: message_json,
        }))
    }

    /// Phase 7:exploration 板块 readiness 综合快照。
    async fn get_exploration_readiness(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ExplorationReadiness>, Status> {
        let backend = self.bridge.lcm_backend_available;
        let snapshot = self.bridge.explore.readiness(backend).await;
        let message_json = serde_json::to_string(&serde_json::json!({
            "backend_available": snapshot.backend_available,
            "controller_active": snapshot.controller_active,
            "source_count": snapshot.source_count,
            "frontier_source_ready": snapshot.frontier_source_ready,
            "costmap_input_ready": snapshot.costmap_input_ready,
            "costmap_input_revision": snapshot.costmap_input_revision,
            "active_source": snapshot.active_source,
            "arbitration": snapshot.arbitration,
        }))
        .unwrap_or_else(|e| format!("{{\"serialization_error\":\"{}\"}}", e));
        Ok(Response::new(ExplorationReadiness {
            ready: snapshot.ready,
            message: message_json,
        }))
    }

    // ========================================================================
    // cmd_vel sink readiness 写回（rework17）
    // ========================================================================
    //
    // Python `LcmMotionCommandBridge` 在 start/stop 时通过本 RPC 把 sink ready
    // 状态推到 Rust handle。`compute_readiness` 的 cmd_vel_sink 子项读这个 flag;
    // 之前 spawn 默认 false + 跨进程无通道 → readiness 永远 false → 真实
    // production 路径走不通。
    async fn set_cmd_vel_sink_ready(
        &self,
        request: Request<CmdVelSinkReadyRequest>,
    ) -> Result<Response<CmdVelSinkReadyResponse>, Status> {
        let request = request.into_inner();
        if !request.ready {
            self.bridge.set_cmd_vel_sink_ready(false);
            return Ok(Response::new(CmdVelSinkReadyResponse { success: true }));
        }
        if request.generation == 0 {
            return Ok(Response::new(CmdVelSinkReadyResponse { success: false }));
        }
        let detail = compute_readiness(&self.bridge, &self.bridge.nav).await;
        let accepted = detail.motion_sink_fresh
            && detail.cmd_vel_sink_ready
            && detail.motion_sink_generation == request.generation;
        if accepted {
            self.bridge.set_cmd_vel_sink_ready(true);
        }
        Ok(Response::new(CmdVelSinkReadyResponse { success: accepted }))
    }

    // ========================================================================
    // 探索
    // ========================================================================

    async fn get_frontiers(&self, _req: Request<Empty>) -> Result<Response<FrontierList>, Status> {
        let s = self.bridge.state.read().await;
        let frontiers = s
            .frontiers
            .iter()
            .map(|f| Frontier {
                pose: Some(Pose {
                    position: Some(Vec3 {
                        x: f.x,
                        y: f.y,
                        z: f.z,
                    }),
                    orientation: Some(Quat {
                        x: f.qx,
                        y: f.qy,
                        z: f.qz,
                        w: f.qw,
                    }),
                    frame_id: f.frame_id.clone(),
                    timestamp_ns: 0,
                }),
                size_cells: f.size_cells,
                heading: f.heading,
            })
            .collect();
        Ok(Response::new(FrontierList {
            frontiers,
            revision: s.frontiers_revision,
            map_revision: self.bridge.explore.frontier_snapshot().map_revision,
            map_version_id: self.bridge.explore.snapshot().map_version_id,
        }))
    }

    type SubscribeFrontiersStream =
        tokio_stream::wrappers::ReceiverStream<Result<FrontierList, Status>>;
    async fn subscribe_frontiers(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<Self::SubscribeFrontiersStream>, Status> {
        let (tx, rx) = tokio::sync::mpsc::channel(4);
        let mut broadcast_rx = self.bridge.frontiers_tx.subscribe();
        let state_handle = self.bridge.state.clone();
        let explore = self.bridge.explore.clone();
        tokio::spawn(async move {
            while let Ok(entries) = broadcast_rx.recv().await {
                let revision = {
                    let s = state_handle.read().await;
                    s.frontiers_revision
                };
                let frontier_snapshot = explore.frontier_snapshot();
                let map_version_id = explore.snapshot().map_version_id;
                let frontiers = entries
                    .iter()
                    .map(|f| Frontier {
                        pose: Some(Pose {
                            position: Some(Vec3 {
                                x: f.x,
                                y: f.y,
                                z: f.z,
                            }),
                            orientation: Some(Quat {
                                x: f.qx,
                                y: f.qy,
                                z: f.qz,
                                w: f.qw,
                            }),
                            frame_id: f.frame_id.clone(),
                            timestamp_ns: 0,
                        }),
                        size_cells: f.size_cells,
                        heading: f.heading,
                    })
                    .collect();
                if tx
                    .send(Ok(FrontierList {
                        frontiers,
                        revision,
                        map_revision: frontier_snapshot.map_revision,
                        map_version_id,
                    }))
                    .await
                    .is_err()
                {
                    break;
                }
            }
        });
        Ok(Response::new(tokio_stream::wrappers::ReceiverStream::new(
            rx,
        )))
    }

    // ========================================================================
    // 探索 start / stop / state / diagnostics（Phase 5 additive）
    // ========================================================================
    //
    // 这些 RPC 把 `nuwax-exploration::ExplorationController` 暴露给 Python
    // 平台层；上游 `nuwax-exploration` crate 的 source adapter（wavefront）
    // 在 `explore_runtime.rs` 内部组装；本 facade 只做字段转换 + 锁定 dimos
    // 行为,不再自造隐式 fallback。
    //
    // 状态契约（与 proto 注释一致）：
    // - `ACCEPTED` / `REJECTED` / `PENDING` / `STOPPED` / `ACTIVE` / `UNIMPLEMENTED`
    // - `last_stop_reason` 字符串: `no_information_gain` / `no_reachable_free_space` /
    //   `consecutive_failures_exceeded` / `user_cancelled` / 空串

    async fn start_exploration(
        &self,
        req: Request<ExploreRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let mut config = explore_config_from_request(r).map_err(Status::invalid_argument)?;
        let _lifecycle_guard = self.bridge.mapping_lifecycle_lock.lock().await;
        self.ensure_running_admission("StartExploration").await?;
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?
            .clone();
        let mapping = adapter.lock().snapshot();
        if !mapping.active {
            return Err(Status::failed_precondition(
                "StartExploration requires an ACTIVE mapping session",
            ));
        }
        let requested_version = config.map_version_id.trim();
        if !requested_version.is_empty() && requested_version != mapping.map_version_id {
            return Err(Status::failed_precondition(format!(
                "StartExploration map_version_id mismatch: requested={} active={}",
                requested_version, mapping.map_version_id
            )));
        }
        config.map_version_id = mapping.map_version_id.clone();
        if config.map_revision > mapping.revision
            || (config.stop_conditions.stop_on_map_revision_change
                && config.map_revision > 0
                && config.map_revision != mapping.revision)
        {
            return Err(Status::failed_precondition(format!(
                "StartExploration map_revision mismatch: requested={} active={}",
                config.map_revision, mapping.revision
            )));
        }
        config.map_revision = mapping.revision;
        if let Some(active_navigation) = self.bridge.nav.active_goal_lease().await {
            return Err(Status::failed_precondition(format!(
                "StartExploration cannot replace active navigation ownership: goal_id={} parent_request_id={}",
                active_navigation.session_id, active_navigation.parent_request_id
            )));
        }
        let snapshot = self
            .bridge
            .explore
            .start_with_config(&r.request_id, config)
            .await;
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn stop_exploration(
        &self,
        req: Request<StopExplorationRequest>,
    ) -> Result<Response<ExplorationState>, Status> {
        let r = req.get_ref();
        let snapshot = self.bridge.explore.stop(&r.request_id).await;
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn get_exploration_state(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ExplorationState>, Status> {
        let snapshot = self.bridge.explore.snapshot();
        Ok(Response::new(exploration_state_from_snapshot(snapshot)))
    }

    async fn get_exploration_diagnostics(
        &self,
        _req: Request<Empty>,
    ) -> Result<Response<ExplorationDiagnostics>, Status> {
        let diag = self.bridge.explore.diagnostics().await;
        let sources = diag
            .sources
            .into_iter()
            .map(|s| ExplorationSourceDiag {
                source_name: s.source_name,
                is_active: s.is_active,
                last_goal_id: s.last_goal_id,
                last_error: s.last_error,
            })
            .collect();
        Ok(Response::new(ExplorationDiagnostics {
            active: diag.active,
            arbitration: diag.arbitration,
            selected_source: diag.selected_source,
            last_stop_reason: diag.last_stop_reason,
            sources,
            frontier_count: diag.frontier_count,
            revision: diag.revision,
            timestamp_ns: now_ns(),
        }))
    }

    // ========================================================================
    // 输入发布兼容（gRPC 端用于调试；走 LCM 后端发布）
    // ========================================================================
    //
    // Phase 4 重写：`publish_odom` / `publish_point_cloud` 不再硬编码 channel；
    // 改走 `channel_registry::channel_for(Odom / PointCloud)`，由 env 或
    // default 拼 typed suffix。删除 stale "B6 blocker" / "Phase 5 才完成"
    // 注释 —— typed payload 编码与 LcmTransport.publish 调用已是当前正确路径。

    async fn publish_odom(
        &self,
        req: Request<PublishOdomRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        // Phase 4:typed payload 编码走 typed_codec;channel 走 channel_registry;
        // 禁止本文件再写硬编码 `/odom` 或 ad-hoc bytes。
        use nuwax_protocol::LcmMessageType;
        let pose = req.get_ref().pose.as_ref();
        let (frame_id, ts_ns) = match pose {
            Some(p) => (p.frame_id.clone(), p.timestamp_ns),
            None => (String::new(), now_ns()),
        };
        let (x, y, z, qx, qy, qz, qw) = match pose {
            Some(p) => {
                let pos = p.position.as_ref();
                let q = p.orientation.as_ref();
                (
                    pos.map(|v| v.x).unwrap_or(0.0),
                    pos.map(|v| v.y).unwrap_or(0.0),
                    pos.map(|v| v.z).unwrap_or(0.0),
                    q.map(|v| v.x).unwrap_or(0.0),
                    q.map(|v| v.y).unwrap_or(0.0),
                    q.map(|v| v.z).unwrap_or(0.0),
                    q.map(|v| v.w).unwrap_or(0.0),
                )
            }
            None => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        };
        let pose_state = crate::lcm_runtime::PoseState {
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            frame_id,
            timestamp_ns: ts_ns,
        };
        let payload = match typed_codec::encode_odometry_payload(&pose_state, &self.base_frame_id) {
            Ok(payload) => payload,
            Err(error) => {
                tracing::warn!("publish_odom: payload encode failed: {}", error);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("Odometry payload encode failed: {}", error),
                }));
            }
        };

        // Phase 4:走 channel_registry::channel_for(Odom) 公共 helper。
        let topic = crate::channel_registry::channel_for(crate::channel_registry::ChannelKey::Odom);
        let msg_type = match LcmMessageType::new("nav_msgs/Odometry") {
            Ok(m) => m,
            Err(e) => {
                tracing::error!("publish_odom: LcmMessageType 构造失败: {}", e);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LcmMessageType::new failed: {}", e),
                }));
            }
        };
        let frame = typed_codec::build_typed_frame(topic, msg_type, payload, 0);
        let lcm_transport = self.bridge.transport.clone();
        match lcm_transport.publish(frame).await {
            Ok(()) => Ok(Response::new(PublishResult {
                success: true,
                message: format!(
                    "odom LCM publish 已接受（backend={}）",
                    self.bridge.transport_backend_name
                ),
            })),
            Err(e) => {
                tracing::warn!("publish_odom: LCM publish 失败: {}", e);
                Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LCM publish failed: {}", e),
                }))
            }
        }
    }

    async fn publish_point_cloud(
        &self,
        req: Request<PublishPointCloudRequest>,
    ) -> Result<Response<PublishResult>, Status> {
        // Phase 4:channel 走 channel_registry::channel_for(PointCloud)，
        // 不再硬编码 `/go2/lidar/points`。
        use nuwax_protocol::LcmMessageType;
        let pcr = req.get_ref();
        let payload = match encode_pointcloud_request_payload(pcr) {
            Ok(payload) => payload,
            Err(error) => {
                tracing::warn!("publish_point_cloud: payload encode failed: {}", error);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("PointCloud2 payload encode failed: {}", error),
                }));
            }
        };

        let topic =
            crate::channel_registry::channel_for(crate::channel_registry::ChannelKey::PointCloud);
        let msg_type = match LcmMessageType::new("sensor_msgs/PointCloud2") {
            Ok(m) => m,
            Err(e) => {
                tracing::error!("publish_point_cloud: LcmMessageType 构造失败: {}", e);
                return Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LcmMessageType::new failed: {}", e),
                }));
            }
        };
        let frame = typed_codec::build_typed_frame(topic, msg_type, payload, 0);
        let lcm_transport = self.bridge.transport.clone();
        match lcm_transport.publish(frame).await {
            Ok(()) => Ok(Response::new(PublishResult {
                success: true,
                message: format!(
                    "PointCloud2 LCM publish 已接受（backend={}）",
                    self.bridge.transport_backend_name
                ),
            })),
            Err(e) => {
                tracing::warn!("publish_point_cloud: LCM publish 失败: {}", e);
                Ok(Response::new(PublishResult {
                    success: false,
                    message: format!("LCM publish failed: {}", e),
                }))
            }
        }
    }
}

#[cfg(test)]
mod pointcloud_request_tests {
    use super::*;
    use crate::mapping_adapter::MappingReadiness;

    fn request() -> PublishPointCloudRequest {
        PublishPointCloudRequest {
            points: vec![
                0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64, 0, 0, 128, 64, 0, 0, 160, 64, 0, 0, 192,
                64,
            ],
            point_step: 12,
            frame_id: "utlidar_lidar".to_string(),
            timestamp_ns: 1_752_345_678_901_234_000,
            fields: vec![
                PointCloudField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                PointCloudField {
                    name: "y".into(),
                    offset: 4,
                    datatype: 7,
                    count: 1,
                },
                PointCloudField {
                    name: "z".into(),
                    offset: 8,
                    datatype: 7,
                    count: 1,
                },
            ],
            point_count: 2,
            is_dense: true,
        }
    }

    #[test]
    fn structured_request_preserves_pointcloud2_metadata_and_data() {
        let request = request();
        let payload = encode_pointcloud_request_payload(&request).expect("encode request");
        let mut generated = Vec::with_capacity(8 + payload.len());
        generated.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        generated.extend_from_slice(&payload);
        let decoded = schema::decode_point_cloud2(&generated).expect("decode PointCloud2");

        assert_eq!(decoded.header.frame_id, request.frame_id);
        assert_eq!(decoded.header.stamp_sec, 1_752_345_678);
        assert_eq!(decoded.header.stamp_nsec, 901_234_000);
        assert_eq!(decoded.width, 2);
        assert_eq!(decoded.point_step, 12);
        assert_eq!(decoded.data, request.points);
        assert_eq!(
            decoded
                .fields
                .iter()
                .map(|field| field.name.as_str())
                .collect::<Vec<_>>(),
            vec!["x", "y", "z"]
        );
    }

    #[test]
    fn structured_request_rejects_noncanonical_xyz_layout() {
        let mut request = request();
        request.fields[0].offset = 4;

        let error = encode_pointcloud_request_payload(&request).expect_err("x offset must fail");
        assert!(error.to_string().contains("canonical x float32"));
    }

    #[test]
    fn legacy_request_keeps_map_xyz_dense_defaults() {
        let request = PublishPointCloudRequest {
            points: request().points,
            point_step: 12,
            ..PublishPointCloudRequest::default()
        };
        let payload = encode_pointcloud_request_payload(&request).expect("encode legacy request");
        let mut generated = Vec::with_capacity(8 + payload.len());
        generated.extend_from_slice(&schema::POINT_CLOUD2_FINGERPRINT);
        generated.extend_from_slice(&payload);
        let decoded = schema::decode_point_cloud2(&generated).expect("decode legacy PointCloud2");

        assert_eq!(decoded.header.frame_id, "map");
        assert_eq!(decoded.fields, canonical_xyz_fields());
        assert!(decoded.is_dense);
    }

    #[tokio::test]
    async fn publish_pointcloud_rpc_does_not_claim_ingestion_before_subscriber_accepts() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());

        let response = GeometryRuntime::publish_point_cloud(&service, Request::new(request()))
            .await
            .expect("PublishPointCloud RPC")
            .into_inner();

        assert!(response.success, "{}", response.message);
        let adapter = handle.mapping_adapter.expect("mapping adapter");
        let adapter = adapter.lock();
        assert_eq!(adapter.pointcloud_count(), 0);
        assert_eq!(adapter.current_revision(), 0);
        assert!(!MappingReadiness::compute(Some(&adapter), true).ready);
    }

    #[tokio::test]
    async fn mapping_readiness_exposes_global_frame_contract() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle);
        let response = GeometryRuntime::get_mapping_readiness(&service, Request::new(Empty {}))
            .await
            .expect("GetMappingReadiness")
            .into_inner();
        let detail: serde_json::Value =
            serde_json::from_str(&response.message).expect("mapping readiness JSON");
        assert_eq!(detail["global_consumer_ready"], false);
        assert_eq!(detail["frame_generation"], 0);
        assert!(!response.ready);
    }
}

#[cfg(test)]
mod frame_tree_tests {
    use super::*;
    use nuwax_tf::make_transform;

    #[tokio::test]
    async fn get_frame_tree_reads_latest_edges_from_canonical_tf_history() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        {
            let state = handle.state.write().await;
            let mut older =
                make_transform("map", "base_link", [1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
            older.stamp_secs = 1;
            older.stamp_nanos = 2;
            state.tf_tree.set(older);
            let mut latest =
                make_transform("map", "base_link", [3.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
            latest.stamp_secs = 4;
            latest.stamp_nanos = 5;
            state.tf_tree.set(latest);
        }
        let service = GeometryRuntimeService::new(handle);

        let response = GeometryRuntime::get_frame_tree(&service, Request::new(Empty {}))
            .await
            .expect("GetFrameTree RPC")
            .into_inner();

        assert_eq!(response.transforms.len(), 1);
        let transform = &response.transforms[0];
        assert_eq!(transform.parent_frame, "map");
        assert_eq!(transform.child_frame, "base_link");
        assert_eq!(transform.translation.as_ref().unwrap().x, 3.0);
        assert_eq!(transform.timestamp_ns, 4_000_000_005);
    }
}

#[cfg(test)]
mod map_update_tests {
    use super::*;
    use crate::lcm_runtime::{MapRevisionEvent, MapState};
    use tokio_stream::StreamExt;

    fn test_mapping_event(map: MapState) -> MapRevisionEvent {
        let mut adapter = crate::mapping_adapter::MappingAdapter::new_with_frames(
            crate::mapping_adapter::MappingAdapterConfig::from_env(),
            "map",
            "odom",
        );
        let session = adapter
            .start_session("stream-session", "warehouse", "mapver_000001")
            .expect("test mapping session");
        MapRevisionEvent::from_map(&map, &session)
    }

    #[tokio::test]
    async fn subscribe_map_updates_is_event_driven_and_carries_real_payload() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .start_session("stream-session", "warehouse", "mapver_000001")
            .expect("activate stream session");
        let service = GeometryRuntimeService::new(handle.clone());
        let response = GeometryRuntime::subscribe_map_updates(&service, Request::new(Empty {}))
            .await
            .expect("SubscribeMapUpdates RPC");
        let mut stream = response.into_inner();

        let event = test_mapping_event(MapState {
            width: 2,
            height: 2,
            resolution: 0.05,
            origin_x: -1.0,
            origin_y: -2.0,
            frame_id: "map".into(),
            revision: 7,
            timestamp_ns: 1_750_000_123_456_000_000,
            data: vec![0, 100, -1, 0],
        });
        handle
            .map_revision_tx
            .send(event.clone())
            .expect("subscriber must be registered before RPC returns");

        let update = tokio::time::timeout(std::time::Duration::from_millis(400), stream.next())
            .await
            .expect("event-driven stream must not wait for the former 500ms poll")
            .expect("stream item")
            .expect("map update");

        let meta = update.meta.expect("metadata");
        assert_eq!(meta.revision, event.revision);
        assert_eq!(meta.width, event.width);
        assert_eq!(meta.height, event.height);
        assert_eq!(update.data.as_slice(), event.data.as_ref());
        assert_eq!(update.size, event.size);
        assert_eq!(update.schema_version, 1);
        assert_eq!(update.checksum, event.checksum);
        assert_eq!(update.last_seen_ms, event.last_seen_ms);
        assert_eq!(update.session_id, "stream-session");
        assert_eq!(update.map_version_id, "mapver_000001");
        assert_eq!(update.session_generation, 1);
        let shm_ref = tokio::time::timeout(std::time::Duration::from_secs(1), async {
            loop {
                match GeometryRuntime::get_occupancy_grid_shm(&service, Request::new(Empty {}))
                    .await
                {
                    Ok(response) => break response.into_inner(),
                    Err(status) if status.code() == tonic::Code::FailedPrecondition => {
                        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
                    }
                    Err(status) => panic!("unexpected SHM status: {status}"),
                }
            }
        })
        .await
        .expect("sole SHM writer must publish current reference");
        assert_eq!(shm_ref.revision, event.revision);
        assert!(shm_ref.size > event.size, "ShmRef.size is mmap file size");
        assert_eq!(shm_ref.layout_version, 2);
        assert_eq!(shm_ref.payload_schema_version, 1);
        assert!(shm_ref.generation > 0);
        assert!(shm_ref.active_slot < 2);
        assert_eq!(shm_ref.payload_size, event.size);
        let internal = handle
            .occupancy_shm_current
            .read()
            .await
            .clone()
            .expect("v2 SHM metadata");
        assert_eq!(internal.layout_version, 2);
        assert_eq!(internal.payload_schema_version, 1);
        assert_eq!(internal.payload_size, event.size);
        assert!(internal.generation > 0);
        assert!(internal.active_slot < 2);
        assert_eq!(
            shm_ref.key,
            handle
                .occupancy_shm_path
                .as_deref()
                .expect("authoritative SHM path")
        );
    }

    #[tokio::test]
    async fn map_update_omits_shm_when_current_revision_differs() {
        let event = test_mapping_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 41,
            timestamp_ns: 41,
            data: vec![0, 100],
            ..MapState::default()
        });
        for current_revision in [40, 42] {
            let current = Arc::new(RwLock::new(Some(
                crate::lcm_runtime::OccupancyShmSnapshot {
                    key: "/tmp/current-v2.mmap".to_string(),
                    file_size: 8192,
                    payload_size: event.size,
                    revision: current_revision,
                    layout_version: 2,
                    payload_schema_version: 1,
                    generation: 2,
                    active_slot: 1,
                },
            )));
            let (tx, mut rx) = tokio::sync::mpsc::channel(1);

            assert!(forward_map_revision(&tx, event.clone(), &current).await);
            let update = rx
                .recv()
                .await
                .expect("stream update")
                .expect("no INTERNAL");
            assert_eq!(update.meta.as_ref().unwrap().revision, event.revision);
            assert_eq!(update.data.as_slice(), event.data.as_ref());
            assert_eq!(update.size, event.size);
            assert_eq!(update.checksum, event.checksum);
            assert!(
                update.shm_ref.is_none(),
                "revision {current_revision} must not be attached to event revision {}",
                event.revision
            );
        }
    }

    #[tokio::test]
    async fn map_update_carries_shm_when_revision_matches() {
        let event = test_mapping_event(MapState {
            width: 2,
            height: 1,
            resolution: 0.1,
            frame_id: "map".to_string(),
            revision: 41,
            timestamp_ns: 41,
            data: vec![0, 100],
            ..MapState::default()
        });
        let current = Arc::new(RwLock::new(Some(
            crate::lcm_runtime::OccupancyShmSnapshot {
                key: "/tmp/current-v2.mmap".to_string(),
                file_size: 8192,
                payload_size: event.size,
                revision: event.revision,
                layout_version: 2,
                payload_schema_version: 1,
                generation: 2,
                active_slot: 1,
            },
        )));
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);

        assert!(forward_map_revision(&tx, event.clone(), &current).await);
        let update = rx
            .recv()
            .await
            .expect("stream update")
            .expect("no INTERNAL");
        assert_eq!(update.meta.as_ref().unwrap().revision, event.revision);
        assert_eq!(update.data.as_slice(), event.data.as_ref());
        assert_eq!(update.size, event.size);
        assert_eq!(update.checksum, event.checksum);
        let shm_ref = update.shm_ref.expect("matching SHM reference");
        assert_eq!(shm_ref.revision, event.revision);
        assert_eq!(shm_ref.size, 8192);
        assert_eq!(shm_ref.layout_version, 2);
        assert_eq!(shm_ref.payload_schema_version, 1);
        assert_eq!(shm_ref.generation, 2);
        assert_eq!(shm_ref.active_slot, 1);
        assert_eq!(shm_ref.payload_size, event.size);
    }

    #[test]
    fn occupancy_shm_v2_projection_rejects_old_or_inconsistent_references() {
        let valid = crate::lcm_runtime::OccupancyShmSnapshot {
            key: "/tmp/occupancy-v2.mmap".to_string(),
            file_size: 8192,
            payload_size: 256,
            revision: 7,
            layout_version: 2,
            payload_schema_version: 1,
            generation: 3,
            active_slot: 0,
        };
        assert!(occupancy_shm_ref(valid.clone()).is_ok());
        for invalid in [
            crate::lcm_runtime::OccupancyShmSnapshot {
                layout_version: 0,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                generation: 0,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                active_slot: 2,
                ..valid.clone()
            },
            crate::lcm_runtime::OccupancyShmSnapshot {
                payload_size: 8192,
                ..valid
            },
        ] {
            assert_eq!(
                occupancy_shm_ref(invalid).unwrap_err().code(),
                tonic::Code::FailedPrecondition
            );
        }
    }
}

#[cfg(test)]
mod cloud_map_update_tests {
    use super::*;
    use crate::lcm_runtime::CloudMapNotification;
    use tokio_stream::StreamExt;

    async fn next_cloud_map_update(
        service: &GeometryRuntimeService,
        handle: &GeometryRuntimeHandle,
        notification: CloudMapNotification,
    ) -> CloudMapShmRef {
        let response =
            GeometryRuntime::subscribe_cloud_map_updates(service, Request::new(Empty {}))
                .await
                .expect("SubscribeCloudMapUpdates RPC");
        let mut stream = response.into_inner();
        handle
            .cloud_map_tx
            .send(notification)
            .expect("cloud-map subscriber registered before RPC returns");
        tokio::time::timeout(Duration::from_secs(1), stream.next())
            .await
            .expect("cloud-map update timeout")
            .expect("cloud-map stream item")
            .expect("cloud-map update")
    }

    fn assert_common_v2_reference(reference: &ShmRef, revision: u64, generation: u64) {
        assert_eq!(
            reference.key,
            crate::pointcloud::cloud_map_shm_path()
                .to_string_lossy()
                .into_owned()
        );
        assert_eq!(
            reference.size,
            u64::try_from(crate::pointcloud::CM_TOTAL_SIZE).unwrap()
        );
        assert_eq!(reference.revision, revision);
        assert_eq!(reference.layout_version, crate::pointcloud::CM_SHM_VERSION);
        assert_eq!(
            reference.payload_schema_version,
            crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION
        );
        assert_eq!(reference.generation, generation);
        assert_eq!(
            reference.active_slot,
            crate::pointcloud::cloud_map_active_slot(revision).unwrap()
        );
    }

    #[tokio::test]
    async fn cloud_map_clear_update_carries_complete_v2_shm_reference() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        let update = next_cloud_map_update(
            &service,
            &handle,
            CloudMapNotification {
                revision: 1,
                point_count: 0,
                voxel_size: 0.05,
                session_generation: 7,
            },
        )
        .await;

        assert_eq!(update.point_count, 0);
        assert_eq!(update.voxel_size, 0.05);
        let reference = update.shm_ref.expect("v2 SHM reference");
        assert_common_v2_reference(&reference, 1, 7);
        assert_eq!(reference.active_slot, 0);
        assert_eq!(reference.payload_size, 0);
    }

    #[tokio::test]
    async fn cloud_map_data_update_carries_payload_size_and_active_slot() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        let update = next_cloud_map_update(
            &service,
            &handle,
            CloudMapNotification {
                revision: 2,
                point_count: 3,
                voxel_size: 0.10,
                session_generation: 8,
            },
        )
        .await;

        assert_eq!(update.point_count, 3);
        assert_eq!(update.voxel_size, 0.10);
        let reference = update.shm_ref.expect("v2 SHM reference");
        assert_common_v2_reference(&reference, 2, 8);
        assert_eq!(reference.active_slot, 1);
        assert_eq!(reference.payload_size, 36);
    }
}

#[cfg(test)]
mod lifecycle_tests {
    use super::*;
    use crate::explore_runtime::{
        now_monotonic_nanos, now_unix_nanos, ChildNavigationCancelTicket, ChildNavigationLease,
        ChildNavigationRequest, ChildNavigationResult, ChildNavigationSubmitError,
        ChildNavigationTerminalProof, ChildNavigationTicket, ChildNavigationZeroProof,
        ExploreCostmapSnapshot, ExploreNavigationSink, ExploreRobotPose, STATUS_ACCEPTED,
        STATUS_ACTIVE, STATUS_FAILED, STATUS_STOPPING,
    };
    use crate::lcm_runtime::{MapRevisionEvent, MapState};
    use crate::pointcloud::CloudMapShmWriter;
    use nalgebra::Point3;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
    use std::sync::Mutex as StdMutex;
    use tokio_stream::StreamExt;

    #[derive(Default)]
    struct LifecycleExploreSink {
        last_lease: StdMutex<Option<ChildNavigationLease>>,
        cancel_error: StdMutex<Option<String>>,
    }

    impl LifecycleExploreSink {
        fn last_lease(&self) -> ChildNavigationLease {
            self.last_lease
                .lock()
                .expect("lifecycle exploration sink lock")
                .clone()
                .expect("submitted child lease")
        }
    }

    #[async_trait::async_trait]
    impl ExploreNavigationSink for LifecycleExploreSink {
        async fn submit(
            &self,
            request: ChildNavigationRequest,
        ) -> Result<ChildNavigationTicket, ChildNavigationSubmitError> {
            let lease = ChildNavigationLease {
                session_id: request.request_id,
                owner_kind: 2,
                parent_request_id: request.parent_request_id,
                goal_generation: 1,
                map_version_id: request.map_version_id,
                map_revision: request.map_revision,
                goal_payload_sha256: [7; 32],
            };
            *self
                .last_lease
                .lock()
                .expect("lifecycle exploration sink lock") = Some(lease.clone());
            Ok(ChildNavigationTicket { lease })
        }

        async fn cancel(
            &self,
            lease: &ChildNavigationLease,
        ) -> Result<ChildNavigationCancelTicket, String> {
            if let Some(error) = self
                .cancel_error
                .lock()
                .expect("lifecycle exploration sink lock")
                .clone()
            {
                return Err(error);
            }
            Ok(ChildNavigationCancelTicket {
                lease: lease.clone(),
                cancel_generation: 1,
            })
        }
    }

    async fn start_mapping_session(
        service: &GeometryRuntimeService,
        session_id: &str,
        map_version_id: &str,
    ) -> MappingSessionState {
        GeometryRuntime::start_mapping(
            service,
            Request::new(StartMappingRequest {
                session_id: session_id.to_string(),
                map_name: "warehouse".to_string(),
                map_version_id: map_version_id.to_string(),
            }),
        )
        .await
        .expect("StartMapping")
        .into_inner()
    }

    fn commit_test_map(handle: &GeometryRuntimeHandle, x: f64) -> u64 {
        let mut adapter = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock();
        adapter.feed_pointcloud(&[Point3::new(x, 0.0, 2.0)]);
        assert!(adapter.build_and_publish(&handle.state, Some(&handle.map_revision_tx)));
        adapter.current_revision()
    }

    fn frontier_costmap(revision: u64) -> ExploreCostmapSnapshot {
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
            map_version_id: "mapver_000201".to_string(),
            map_revision: revision,
        }
    }

    fn activation_request(session_id: &str, map_version_id: &str) -> ActivateMapVersionRequest {
        ActivateMapVersionRequest {
            session_id: session_id.to_string(),
            map_name: "warehouse".to_string(),
            map_version_id: map_version_id.to_string(),
            checkpoint: Some(OccupancyGridData {
                meta: Some(OccupancyGridMeta {
                    width: 2,
                    height: 2,
                    resolution: 0.05,
                    origin_x: -1.0,
                    origin_y: 2.0,
                    frame_id: "map".to_string(),
                    revision: 1,
                    timestamp_ns: 1_700_000_000_000_000_000,
                }),
                data: vec![u8::MAX, 0, 100, 50],
            }),
            cloud_checkpoint: None,
        }
    }

    fn exploration_request(
        request_id: &str,
        map_version_id: &str,
        map_revision: u64,
    ) -> ExploreRequest {
        ExploreRequest {
            request_id: request_id.to_string(),
            strategy: "wavefront".to_string(),
            arbitration: "prefer_wavefront".to_string(),
            map_version_id: map_version_id.to_string(),
            map_revision,
            motion_guard_generation: 7,
            motion_guard_boot_nonce: vec![7; 16],
            motion_stop_generation: 1,
            ..ExploreRequest::default()
        }
    }

    async fn start_public_navigation(
        service: &GeometryRuntimeService,
        handle: &GeometryRuntimeHandle,
        goal_id: &str,
    ) -> NavigationState {
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                qw: 1.0,
                ..PoseState::default()
            };
            shared.map.frame_id = "map".to_string();
            shared.map.revision = 1;
        }
        handle
            .nav
            .observe_stack_state(crate::lcm_runtime::NavState {
                status: "IDLE".to_string(),
                timestamp_ns: now_ns(),
                ..crate::lcm_runtime::NavState::default()
            })
            .await;
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 1,
                map_frame: "map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink");
        GeometryRuntime::set_navigation_goal(
            service,
            Request::new(NavigationGoal {
                pose: Some(Pose {
                    orientation: Some(Quat {
                        w: 1.0,
                        ..Quat::default()
                    }),
                    frame_id: "map".to_string(),
                    timestamp_ns: now_ns(),
                    ..Pose::default()
                }),
                frame_id: "map".to_string(),
                goal_id: goal_id.to_string(),
                map_version_id: "map-v1".to_string(),
                motion_guard_generation: 7,
                motion_guard_boot_nonce: vec![7; 16],
                motion_stop_generation: 1,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect("public navigation goal")
        .into_inner()
    }

    async fn start_active_exploration(
        handle: &GeometryRuntimeHandle,
        sink: Arc<LifecycleExploreSink>,
        request_id: &str,
    ) -> (String, ChildNavigationLease) {
        handle.explore.set_navigation_sink(sink.clone());
        handle
            .explore
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .expect("frontier costmap");
        assert_eq!(
            handle
                .explore
                .start(request_id, "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let active = handle.explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        let lease = sink.last_lease();
        assert_eq!(active.child_task_id, lease.session_id);
        (active.child_task_id, lease)
    }

    #[tokio::test]
    async fn process_stop_waits_for_exploration_zero_before_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        let (child_task_id, lease) =
            start_active_exploration(&handle, sink, "process-stop-zero").await;
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_millis(100), async {
            while handle.explore.snapshot().status != STATUS_STOPPING {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("process Stop must dispatch exploration cancellation");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must remain gated on ZEROED proof"
        );

        handle
            .explore
            .report_navigation_result(
                &child_task_id,
                ChildNavigationResult::Cancelled(ChildNavigationZeroProof {
                    lease,
                    cancel_generation: 1,
                    zero_ack_id: "process-stop-zero-1".to_string(),
                    zero_applied_monotonic_ns: now_monotonic_nanos(),
                }),
            )
            .await
            .expect("process Stop ZEROED proof");

        let response = stop_task
            .await
            .expect("process Stop task")
            .expect("process Stop response")
            .into_inner();
        assert!(response.success);
        listener_task.await.expect("listener shutdown task");
        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "stopped");
    }

    #[tokio::test]
    async fn process_stop_failure_keeps_scheduler_and_listener_alive() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        start_active_exploration(&handle, sink.clone(), "process-stop-failed").await;
        *sink
            .cancel_error
            .lock()
            .expect("lifecycle exploration sink lock") = Some("guard unavailable".to_string());
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);

        let error = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect_err("retained ownership must reject process Stop");

        assert_eq!(error.code(), tonic::Code::FailedPrecondition);
        assert!(error.message().contains("safety ownership retained"));
        assert!(handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "failed");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "failed process Stop must not request listener shutdown"
        );
    }

    #[tokio::test]
    async fn process_stop_timeout_does_not_request_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let sink = Arc::new(LifecycleExploreSink::default());
        start_active_exploration(&handle, sink, "process-stop-timeout").await;
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);

        let error = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect_err("missing ZEROED proof must time out process Stop");

        assert_eq!(error.code(), tonic::Code::DeadlineExceeded);
        assert!(error.message().contains("safety ownership release"));
        assert!(handle.explore.blocks_mapping_mutation().await);
        assert_eq!(service.state.read().await.phase, "failed");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener.wait_for_request())
                .await
                .is_err(),
            "timed-out process Stop must not request listener shutdown"
        );
    }

    #[tokio::test]
    async fn process_stop_waits_for_public_navigation_zero_before_listener_shutdown() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        let accepted = start_public_navigation(&service, &handle, "public-stop-zero").await;
        assert_eq!(accepted.status, "ACCEPTED");
        let active_lease = handle
            .nav
            .active_goal_lease()
            .await
            .expect("active public lease");
        let listener_requested = Arc::new(tokio::sync::Notify::new());
        let listener_task = {
            let listener_requested = listener_requested.clone();
            tokio::spawn(async move {
                listener.wait_for_request().await;
                listener_requested.notify_one();
                listener.mark_listener_closed();
            })
        };
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };

        tokio::time::timeout(Duration::from_secs(1), async {
            while handle.nav.cancel_snapshot().phase != crate::nav_runtime::CancelPhase::Pending {
                tokio::task::yield_now().await;
            }
        })
        .await
        .expect("process Stop must dispatch public navigation cancellation");
        assert!(
            tokio::time::timeout(Duration::from_millis(40), listener_requested.notified())
                .await
                .is_err(),
            "listener shutdown must wait for public navigation ZEROED proof"
        );
        let pending = handle.nav.cancel_snapshot();
        assert_eq!(pending.lease.as_ref(), Some(&active_lease));
        handle
            .nav
            .observe_cancel_ack(crate::nav_runtime::MotionSinkCancelAck {
                cancel_generation: pending.generation,
                goal_id: active_lease.session_id.clone(),
                owner_kind: active_lease.owner_kind,
                parent_request_id: active_lease.parent_request_id.clone(),
                goal_generation: active_lease.goal_generation,
                status: crate::nav_runtime::MotionSinkCancelAckStatus::Zeroed,
                message: "process stop public zero".to_string(),
            })
            .await;

        let stopped = stop_task
            .await
            .expect("process Stop join")
            .expect("process Stop response")
            .into_inner();
        assert!(stopped.success);
        listener_task.await.expect("listener shutdown task");
        assert!(handle.nav.active_goal_lease().await.is_none());
        assert_eq!(service.state.read().await.phase, "stopped");
    }

    #[tokio::test]
    async fn process_stop_closes_exploration_and_public_navigation_admission_atomically() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle.clone(), shutdown);
        start_mapping_session(&service, "stop-admission", "mapver_stop_admission").await;
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        let admission_guard = handle.mapping_lifecycle_lock.lock().await;
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop(&service, Request::new(StopRequest {})).await
            })
        };
        tokio::time::sleep(Duration::from_millis(20)).await;
        let explore_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::start_exploration(
                    &service,
                    Request::new(exploration_request(
                        "queued-after-stop",
                        "mapver_stop_admission",
                        0,
                    )),
                )
                .await
            })
        };
        let navigation_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::set_navigation_goal(
                    &service,
                    Request::new(NavigationGoal {
                        goal_id: "queued-public-after-stop".to_string(),
                        motion_guard_generation: 7,
                        motion_guard_boot_nonce: vec![7; 16],
                        motion_stop_generation: 1,
                        ..NavigationGoal::default()
                    }),
                )
                .await
            })
        };
        let listener_task = tokio::spawn(async move {
            listener.wait_for_request().await;
            listener.mark_listener_closed();
        });
        drop(admission_guard);

        assert!(stop_task.await.unwrap().unwrap().into_inner().success);
        listener_task.await.unwrap();
        let explore_error = explore_task
            .await
            .unwrap()
            .expect_err("explore admission closed");
        let navigation_error = navigation_task
            .await
            .unwrap()
            .expect_err("public navigation admission closed");
        assert_eq!(explore_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(navigation_error.code(), tonic::Code::FailedPrecondition);
        assert!(explore_error.message().contains("current=stopped"));
        assert!(navigation_error.message().contains("current=stopped"));
        assert!(!handle.explore.blocks_mapping_mutation().await);
        assert!(handle.nav.active_goal_lease().await.is_none());
    }

    #[tokio::test]
    async fn start_exploration_requires_exact_active_mapping_identity() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));

        let inactive = GeometryRuntime::start_exploration(
            &service,
            Request::new(exploration_request("inactive-map", "mapver_exact", 0)),
        )
        .await
        .expect_err("inactive mapping must reject exploration");
        assert_eq!(inactive.code(), tonic::Code::FailedPrecondition);

        let activated = GeometryRuntime::activate_map_version(
            &service,
            Request::new(activation_request("exact-map", "mapver_exact")),
        )
        .await
        .expect("activate exact test map")
        .into_inner();
        let current_revision = activated.revision;
        let wrong_version = GeometryRuntime::start_exploration(
            &service,
            Request::new(exploration_request("wrong-version", "mapver_other", 0)),
        )
        .await
        .expect_err("wrong map version must reject exploration");
        assert_eq!(wrong_version.code(), tonic::Code::FailedPrecondition);

        let mut wrong_revision =
            exploration_request("wrong-revision", "mapver_exact", current_revision + 1);
        wrong_revision.stop_conditions = Some(ExplorationStopConditions {
            stop_on_map_revision_change: true,
            ..ExplorationStopConditions::default()
        });
        let revision_error =
            GeometryRuntime::start_exploration(&service, Request::new(wrong_revision))
                .await
                .expect_err("mismatched pinned revision must reject exploration");
        assert_eq!(revision_error.code(), tonic::Code::FailedPrecondition);

        let accepted = GeometryRuntime::start_exploration(
            &service,
            Request::new(exploration_request(
                "exact-mapping",
                "mapver_exact",
                current_revision,
            )),
        )
        .await
        .expect("exact active mapping identity")
        .into_inner();
        assert_eq!(accepted.status, STATUS_ACCEPTED);
        assert_eq!(accepted.request_id, "exact-mapping");
    }

    #[tokio::test]
    async fn exploration_start_and_mapping_mutations_share_one_transition_barrier() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "barrier-map", "mapver_barrier").await;
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        let lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
        let start_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::start_exploration(
                    &service,
                    Request::new(exploration_request("barrier-explore", "mapver_barrier", 0)),
                )
                .await
            })
        };
        tokio::time::sleep(Duration::from_millis(20)).await;
        let stop_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::stop_mapping(
                    &service,
                    Request::new(StopMappingRequest {
                        session_id: "barrier-map".to_string(),
                        reason: "must not overtake start".to_string(),
                    }),
                )
                .await
            })
        };
        let activate_task = {
            let service = service.clone();
            tokio::spawn(async move {
                GeometryRuntime::activate_map_version(
                    &service,
                    Request::new(activation_request(
                        "barrier-activate",
                        "mapver_after_barrier",
                    )),
                )
                .await
            })
        };
        drop(lifecycle_guard);

        let started = start_task
            .await
            .unwrap()
            .expect("queued exploration start")
            .into_inner();
        assert_eq!(started.status, STATUS_ACCEPTED);
        let stop_error = stop_task
            .await
            .unwrap()
            .expect_err("StopMapping must be fenced");
        let activate_error = activate_task
            .await
            .unwrap()
            .expect_err("ActivateMapVersion must be fenced");
        assert_eq!(stop_error.code(), tonic::Code::FailedPrecondition);
        assert_eq!(activate_error.code(), tonic::Code::FailedPrecondition);
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        assert!(mapping.active);
        assert_eq!(mapping.map_version_id, "mapver_barrier");
    }

    #[tokio::test]
    async fn mapping_mutation_allows_failed_exploration_without_safety_ownership() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "failed-session", "mapver_000200").await;
        handle
            .explore
            .set_navigation_sink(Arc::new(LifecycleExploreSink::default()));
        assert_eq!(
            handle
                .explore
                .start("failed-without-child", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );

        let failed = handle
            .explore
            .tick_with_pose(ExploreRobotPose {
                x: 0.0,
                y: 0.0,
                frame_id: "odom".to_string(),
                source_timestamp_ns: now_unix_nanos(),
                received_monotonic_ns: now_monotonic_nanos(),
            })
            .await;
        assert_eq!(failed.status, STATUS_FAILED);
        assert!(failed.child_task_id.is_empty());
        assert!(!handle.explore.blocks_mapping_mutation().await);

        let stopped = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "failed-session".to_string(),
                reason: "terminal exploration failure".to_string(),
            }),
        )
        .await
        .expect("child-free FAILED must allow StopMapping")
        .into_inner();
        assert_eq!(stopped.phase, "STOPPED");
        assert_eq!(handle.explore.snapshot().status, STATUS_FAILED);
        assert_eq!(
            handle.explore.snapshot().last_stop_reason,
            failed.last_stop_reason
        );

        let activated = GeometryRuntime::activate_map_version(
            &service,
            Request::new(activation_request("mapload:failed", "mapver_000201")),
        )
        .await
        .expect("child-free FAILED must allow ActivateMapVersion")
        .into_inner();
        assert_eq!(activated.phase, "ACTIVE");
        assert_eq!(handle.explore.snapshot().status, STATUS_FAILED);
        assert_eq!(
            handle.explore.snapshot().last_stop_reason,
            failed.last_stop_reason
        );
    }

    #[tokio::test]
    async fn mapping_mutation_waits_for_late_zeroed_after_child_cancel_failure() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "parked-session", "mapver_000201").await;
        let sink = Arc::new(LifecycleExploreSink::default());
        handle.explore.set_navigation_sink(sink.clone());
        handle
            .explore
            .ingest_costmap_snapshot(frontier_costmap(1))
            .await
            .expect("frontier costmap");
        assert_eq!(
            handle
                .explore
                .start("parked-exploration", "prefer_wavefront")
                .await
                .status,
            STATUS_ACCEPTED
        );
        let active = handle.explore.tick((2.0, 2.0)).await;
        assert_eq!(active.status, STATUS_ACTIVE);
        let lease = sink.last_lease();
        assert_eq!(active.child_task_id, lease.session_id);
        assert_eq!(
            handle.explore.stop("parked-exploration").await.status,
            STATUS_STOPPING
        );

        let failed = handle
            .explore
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::CancelFailed {
                    proof: ChildNavigationTerminalProof {
                        lease: lease.clone(),
                    },
                    cancel_generation: 1,
                    reason: "CANCEL_ZERO_ACK_TIMEOUT".to_string(),
                },
            )
            .await
            .expect("cancel failure terminal");
        assert_eq!(failed.status, STATUS_FAILED);
        assert_eq!(failed.last_stop_reason, "child_cancel_failed");
        assert_eq!(failed.child_task_id, active.child_task_id);
        assert!(handle.explore.blocks_mapping_mutation().await);

        let stop_error = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "parked-session".to_string(),
                reason: "must remain parked".to_string(),
            }),
        )
        .await
        .expect_err("retained child ownership must block StopMapping");
        assert_eq!(stop_error.code(), tonic::Code::FailedPrecondition);
        let activate_error = GeometryRuntime::activate_map_version(
            &service,
            Request::new(activation_request("mapload:blocked", "mapver_000202")),
        )
        .await
        .expect_err("retained child ownership must block ActivateMapVersion");
        assert_eq!(activate_error.code(), tonic::Code::FailedPrecondition);
        assert!(handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());

        let released = handle
            .explore
            .report_navigation_result(
                &active.child_task_id,
                ChildNavigationResult::Cancelled(ChildNavigationZeroProof {
                    lease,
                    cancel_generation: 1,
                    zero_ack_id: "late-zero-1".to_string(),
                    zero_applied_monotonic_ns: now_monotonic_nanos(),
                }),
            )
            .await
            .expect("late ZEROED proof");
        assert_eq!(released.status, STATUS_FAILED);
        assert_eq!(released.last_stop_reason, "child_cancel_failed");
        assert!(released.child_task_id.is_empty());
        assert!(!handle.explore.blocks_mapping_mutation().await);

        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "parked-session".to_string(),
                reason: "late ZEROED released ownership".to_string(),
            }),
        )
        .await
        .expect("late ZEROED must release StopMapping");
        let activated = GeometryRuntime::activate_map_version(
            &service,
            Request::new(activation_request("mapload:released", "mapver_000202")),
        )
        .await
        .expect("late ZEROED must release ActivateMapVersion")
        .into_inner();
        assert_eq!(activated.phase, "ACTIVE");
        let final_exploration = handle.explore.snapshot();
        assert_eq!(final_exploration.status, STATUS_FAILED);
        assert_eq!(final_exploration.last_stop_reason, "child_cancel_failed");
    }

    #[tokio::test]
    async fn start_mapping_publishes_one_monotonic_zero_cloud_frame_per_new_session() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let path = std::env::temp_dir().join(format!(
            "nuwax-cloudmap-start-mapping-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        handle.cloud_map_writer = Arc::new(parking_lot::Mutex::new(Some(
            CloudMapShmWriter::new(&path).expect("cloud map writer"),
        )));
        let mut cloud_updates = handle.cloud_map_tx.subscribe();
        let service = GeometryRuntimeService::new(handle.clone());

        let first = start_mapping_session(&service, "cloud-session-1", "mapver_000101").await;
        assert_eq!(first.session_generation, 1);
        let first_clear = tokio::time::timeout(Duration::from_secs(1), cloud_updates.recv())
            .await
            .expect("first clear timeout")
            .expect("first clear notification");
        assert_eq!(first_clear.point_count, 0);
        assert!(first_clear.revision > 0);
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            first_clear.revision
        );

        let replay = start_mapping_session(&service, "cloud-session-1", "mapver_000101").await;
        assert!(replay.idempotent_replay);
        assert!(
            tokio::time::timeout(Duration::from_millis(100), cloud_updates.recv())
                .await
                .is_err(),
            "idempotent StartMapping must not publish another clear revision"
        );
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            first_clear.revision
        );

        GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "cloud-session-1".to_string(),
                reason: "new session test".to_string(),
            }),
        )
        .await
        .expect("StopMapping");
        let second = start_mapping_session(&service, "cloud-session-2", "mapver_000102").await;
        assert_eq!(second.session_generation, 2);
        let second_clear = tokio::time::timeout(Duration::from_secs(1), cloud_updates.recv())
            .await
            .expect("second clear timeout")
            .expect("second clear notification");
        assert_eq!(second_clear.point_count, 0);
        assert!(second_clear.revision > first_clear.revision);
        assert_eq!(
            handle.state.read().await.cloud_map_revision,
            second_clear.revision
        );

        drop(service);
        drop(handle);
        let _ = std::fs::remove_file(path);
    }

    #[tokio::test]
    async fn mapping_rpc_fences_sessions_and_keeps_wire_revision_globally_monotonic() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());

        let first = start_mapping_session(&service, "session-1", "mapver_000001").await;
        assert_eq!(first.phase, "ACTIVE");
        assert_eq!(first.session_generation, 1);
        assert!(!first.idempotent_replay);
        let replay = start_mapping_session(&service, "session-1", "mapver_000001").await;
        assert!(replay.idempotent_replay);

        let response = GeometryRuntime::subscribe_map_updates(&service, Request::new(Empty {}))
            .await
            .expect("SubscribeMapUpdates");
        let mut stream = response.into_inner();
        let first_revision = commit_test_map(&handle, 0.0);
        assert_eq!(first_revision, 1);
        let first_update = tokio::time::timeout(Duration::from_secs(1), stream.next())
            .await
            .expect("first session update timeout")
            .expect("first session stream")
            .expect("first session update");
        assert_eq!(first_update.meta.as_ref().unwrap().revision, first_revision);
        assert_eq!(first_update.session_generation, 1);

        let stopped = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "session-1".to_string(),
                reason: "test".to_string(),
            }),
        )
        .await
        .expect("StopMapping")
        .into_inner();
        assert_eq!(stopped.phase, "STOPPED");
        assert!(!stopped.active);
        let stopped_snapshot = handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .snapshot();
        handle
            .map_revision_tx
            .send(MapRevisionEvent::from_map(
                &MapState {
                    width: 1,
                    height: 1,
                    resolution: 0.1,
                    frame_id: "map".to_string(),
                    revision: first_revision + 1,
                    timestamp_ns: first_revision + 1,
                    data: vec![100],
                    ..MapState::default()
                },
                &stopped_snapshot,
            ))
            .expect("coalescer remains registered");
        assert!(
            tokio::time::timeout(Duration::from_millis(100), stream.next())
                .await
                .is_err(),
            "stopped session must not publish queued map events"
        );

        {
            let mut shared = handle.state.write().await;
            shared.map.revision = 99;
        }
        *handle.occupancy_shm_current.write().await =
            Some(crate::lcm_runtime::OccupancyShmSnapshot {
                key: "/tmp/stale.mmap".to_string(),
                file_size: 8192,
                payload_size: 1,
                revision: 99,
                layout_version: 2,
                payload_schema_version: 1,
                generation: 1,
                active_slot: 0,
            });

        let second = start_mapping_session(&service, "session-2", "mapver_000002").await;
        assert_eq!(second.session_generation, 2);
        assert_eq!(second.revision, 0);
        assert_eq!(handle.state.read().await.map.revision, 0);
        assert_eq!(
            handle
                .mapping_adapter
                .as_ref()
                .expect("mapping adapter")
                .lock()
                .global_revision(),
            first_revision
        );
        assert!(handle.map_revision_latest_tx.borrow().is_none());
        assert!(handle.occupancy_shm_current.read().await.is_none());

        let stale_stop = GeometryRuntime::stop_mapping(
            &service,
            Request::new(StopMappingRequest {
                session_id: "session-1".to_string(),
                reason: "late retry".to_string(),
            }),
        )
        .await
        .expect_err("old session stop must fail closed");
        assert_eq!(stale_stop.code(), tonic::Code::FailedPrecondition);
        assert!(handle
            .mapping_adapter
            .as_ref()
            .expect("mapping adapter")
            .lock()
            .is_active());

        let second_revision = commit_test_map(&handle, 1.0);
        assert!(second_revision > first_revision);
        let second_update = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let update = stream
                    .next()
                    .await
                    .expect("second session stream")
                    .expect("second session update");
                if update.session_generation == 2 {
                    break update;
                }
            }
        })
        .await
        .expect("new session globally newer revision must not be deduplicated");
        assert_eq!(
            second_update.meta.as_ref().unwrap().revision,
            second_revision
        );
        assert_eq!(second_update.map_version_id, "mapver_000002");
    }

    #[tokio::test]
    async fn stop_mapping_waits_for_inflight_fanout_barrier() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let service = GeometryRuntimeService::new(handle.clone());
        start_mapping_session(&service, "barrier-session", "mapver_000010").await;

        // Fanout sinks hold this lock from lifecycle validation through their
        // bounded sink operation. Holding it here deterministically pauses the
        // same critical section and proves StopMapping cannot overtake it.
        let fanout_guard = handle.mapping_lifecycle_lock.lock().await;
        let stop_service = service.clone();
        let stop_task = tokio::spawn(async move {
            GeometryRuntime::stop_mapping(
                &stop_service,
                Request::new(StopMappingRequest {
                    session_id: "barrier-session".to_string(),
                    reason: "barrier-test".to_string(),
                }),
            )
            .await
        });
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(
            !stop_task.is_finished(),
            "StopMapping must wait for an in-flight fanout critical section"
        );

        drop(fanout_guard);
        let stopped = tokio::time::timeout(Duration::from_secs(1), stop_task)
            .await
            .expect("StopMapping must finish after fanout barrier release")
            .expect("StopMapping join")
            .expect("StopMapping RPC")
            .into_inner();
        assert_eq!(stopped.phase, "STOPPED");
        assert!(!stopped.active);
    }

    #[tokio::test]
    async fn activate_map_version_commits_checkpoint_and_start_mapping_reuses_session() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-activate-cloud-test-{}-{}.mmap",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        *handle.cloud_map_writer.lock() = Some(
            crate::pointcloud::CloudMapShmWriter::new(&cloud_path).expect("test cloud-map writer"),
        );
        let service = GeometryRuntimeService::new(handle.clone());
        let mut revision_rx = handle.map_revision_tx.subscribe();
        let mut cloud_revision_rx = handle.cloud_map_tx.subscribe();
        let cloud_xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .iter()
            .flat_map(|point| point.iter())
            .flat_map(|value| value.to_le_bytes())
            .collect::<Vec<_>>();

        let activated = GeometryRuntime::activate_map_version(
            &service,
            Request::new(ActivateMapVersionRequest {
                session_id: "mapload:one".to_string(),
                map_name: "warehouse".to_string(),
                map_version_id: "mapver_000001".to_string(),
                checkpoint: Some(OccupancyGridData {
                    meta: Some(OccupancyGridMeta {
                        width: 2,
                        height: 2,
                        resolution: 0.05,
                        origin_x: -1.0,
                        origin_y: 2.0,
                        frame_id: "map".to_string(),
                        revision: 119,
                        timestamp_ns: 1_700_000_000_000_000_000,
                    }),
                    data: vec![u8::MAX, 0, 100, 50],
                }),
                cloud_checkpoint: Some(CloudMapCheckpoint {
                    frame_id: "map".to_string(),
                    source_revision: 119,
                    point_count: 2,
                    voxel_size: crate::pointcloud::VOXEL_SIZE,
                    xyz_sha256: crate::map_checksum::sha256_hex(&cloud_xyz),
                    xyz_f32_le: cloud_xyz,
                    payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
                }),
            }),
        )
        .await
        .expect("ActivateMapVersion")
        .into_inner();

        assert!(activated.success);
        assert!(activated.active);
        assert_eq!(activated.phase, "ACTIVE");
        assert_eq!(activated.map_version_id, "mapver_000001");
        assert!(activated.revision > 0);
        let event = revision_rx.recv().await.expect("checkpoint revision event");
        assert_eq!(event.session_id, "mapload:one");
        assert_eq!(event.map_version_id, "mapver_000001");
        assert_eq!(event.data.as_ref(), &[u8::MAX, 0, 100, 50]);
        let cloud_event = cloud_revision_rx
            .recv()
            .await
            .expect("cloud checkpoint revision event");
        assert_eq!(cloud_event.point_count, 2);
        assert_eq!(cloud_event.session_generation, activated.session_generation);
        let shared = handle.state.read().await;
        assert_eq!(shared.map.data, vec![-1, 0, 100, 50]);
        assert_eq!(shared.cloud_map_revision, cloud_event.revision);
        drop(shared);

        let replay = start_mapping_session(&service, "mapload:one", "mapver_000001").await;
        assert!(replay.idempotent_replay);
        assert_eq!(replay.revision, activated.revision);
        assert_eq!(handle.state.read().await.map.data, vec![-1, 0, 100, 50]);
        let _ = std::fs::remove_file(cloud_path);
    }

    #[tokio::test]
    async fn stop_waits_until_the_listener_is_closed_before_reporting_success() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        let (shutdown, mut listener) = crate::server::shutdown_channel();
        let service = GeometryRuntimeService::new_with_shutdown(handle, shutdown);

        let listener_task = tokio::spawn(async move {
            listener.wait_for_request().await;
            listener.mark_listener_closed();
        });

        let response = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect("Stop RPC")
            .into_inner();

        assert!(response.success);
        listener_task.await.expect("listener acknowledgement task");
        assert_eq!(service.state.read().await.phase, "stopped");

        let repeated = GeometryRuntime::stop(&service, Request::new(StopRequest {}))
            .await
            .expect("repeated Stop RPC")
            .into_inner();
        assert!(repeated.success, "Stop must remain idempotent");
    }
}

#[cfg(test)]
mod exploration_contract_tests {
    use super::*;

    #[test]
    fn start_request_parses_boundary_stop_conditions_and_map_pin() {
        let request = ExploreRequest {
            strategy: "wavefront".into(),
            arbitration: "prefer_wavefront".into(),
            map_version_id: "map-v7".into(),
            request_id: "explore-1".into(),
            map_revision: 7,
            boundary_line_segments: vec![
                ExplorationBoundarySegment {
                    start_x: 0.0,
                    start_y: 0.0,
                    end_x: 2.0,
                    end_y: 0.0,
                },
                ExplorationBoundarySegment {
                    start_x: 2.0,
                    start_y: 0.0,
                    end_x: 0.0,
                    end_y: 2.0,
                },
                ExplorationBoundarySegment {
                    start_x: 0.0,
                    start_y: 2.0,
                    end_x: 0.0,
                    end_y: 0.0,
                },
            ],
            stop_conditions: Some(ExplorationStopConditions {
                max_coverage_pct: Some(85.0),
                no_frontier_timeout_ms: Some(4_000),
                max_duration_ms: Some(60_000),
                max_goals: Some(12),
                stop_on_map_revision_change: true,
            }),
            max_navigation_retries: 3,
            blacklist_radius_m: 0.75,
            boundary_frame_id: "map".into(),
            motion_guard_generation: 7,
            motion_guard_boot_nonce: vec![7; 16],
            motion_stop_generation: 1,
        };

        let config = explore_config_from_request(&request).expect("valid explore config");
        assert_eq!(config.map_version_id, "map-v7");
        assert_eq!(config.map_revision, 7);
        assert_eq!(config.max_navigation_retries, 3);
        assert_eq!(config.blacklist_radius_m, 0.75);
        assert_eq!(
            config.motion_admission,
            crate::nav_runtime::test_motion_admission()
        );
        assert!(config.boundary.is_some());
        assert_eq!(config.stop_conditions.max_goals, Some(12));
        assert_eq!(config.stop_conditions.no_frontier_timeout_ms, Some(4_000));
    }

    #[test]
    fn exploration_state_preserves_child_and_map_fields() {
        let proto =
            exploration_state_from_snapshot(crate::explore_runtime::ExploreRuntimeSnapshot {
                status: crate::explore_runtime::STATUS_ACTIVE,
                message: "moving".into(),
                request_id: "parent".into(),
                selected_source: "wavefront".into(),
                last_stop_reason: String::new(),
                frontier_count: 2,
                revision: 8,
                child_task_id: "child-3".into(),
                map_version_id: "map-v7".into(),
                map_revision: 7,
            });
        assert_eq!(proto.child_task_id, "child-3");
        assert_eq!(proto.map_version_id, "map-v7");
        assert_eq!(proto.map_revision, 7);
    }
}

#[cfg(test)]
mod navigation_feedback_contract_tests {
    use super::*;
    use crate::lcm_runtime::{NavigationFeedback, NavigationUpdate};

    #[test]
    fn navigation_proto_projection_preserves_typed_feedback() {
        let proto = nav_state_to_proto(NavigationUpdate {
            state: crate::lcm_runtime::NavState {
                status: "CANCELLED".into(),
                message: "motion guard zeroed".into(),
                goal_id: "goal-7".into(),
                distance_remaining: 0.0,
                timestamp_ns: 99,
            },
            producer_runtime_generation: 41,
            feedback: NavigationFeedback {
                cancel_phase: "ZEROED".into(),
                cancel_generation: 3,
                actuator_zeroed: true,
                reason_code: "CANCEL_ZEROED".into(),
                runtime_generation: 7,
                guard_generation: 11,
                sink_ready: true,
                planner_map_revision: 42,
                planner_map_frame: "map".into(),
                planner_pose_frame: "map".into(),
                zero_ack_id: "zero-3".into(),
                zero_applied_monotonic_ns: 123,
                owner_kind: nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                parent_request_id: "request-7".into(),
                goal_generation: 5,
                path_revision: 2,
                goal_payload_sha256: [7; 32],
                path_payload_sha256: [8; 32],
                map_version_id: "map-v1".into(),
            },
        });

        assert_eq!(proto.cancel_phase, "ZEROED");
        assert_eq!(proto.cancel_generation, 3);
        assert!(proto.actuator_zeroed);
        assert_eq!(proto.reason_code, "CANCEL_ZEROED");
        assert_eq!(proto.runtime_generation, 7);
        assert_eq!(proto.guard_generation, 11);
        assert!(proto.sink_ready);
        assert_eq!(proto.planner_map_revision, 42);
        assert_eq!(proto.planner_map_frame, "map");
        assert_eq!(proto.planner_pose_frame, "map");
        assert_eq!(proto.zero_ack_id, "zero-3");
        assert_eq!(proto.zero_applied_monotonic_ns, 123);
        assert_eq!(
            proto.owner_kind,
            nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
        );
        assert_eq!(proto.parent_request_id, "request-7");
        assert_eq!(proto.goal_generation, 5);
        assert_eq!(proto.path_revision, 2);
        assert_eq!(proto.goal_payload_sha256, vec![7; 32]);
        assert_eq!(proto.path_payload_sha256, vec![8; 32]);
        assert_eq!(proto.map_version_id, "map-v1");
        assert_eq!(proto.producer_runtime_generation, 41);
    }

    #[tokio::test]
    async fn deprecated_sink_rpc_cannot_create_ready_without_generation() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        handle.set_cmd_vel_sink_ready(false);
        let service = GeometryRuntimeService::new(handle.clone());

        let response = GeometryRuntime::set_cmd_vel_sink_ready(
            &service,
            Request::new(CmdVelSinkReadyRequest {
                ready: true,
                generation: 0,
            }),
        )
        .await
        .expect("deprecated RPC response")
        .into_inner();

        assert!(!response.success);
        assert!(!handle.cmd_vel_sink_ready());
    }

    #[tokio::test]
    async fn public_goal_normalizes_once_and_cancel_preserves_complete_lease() {
        let handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("real-lcm test handle");
        {
            let mut shared = handle.state.write().await;
            shared.pose = PoseState {
                frame_id: "map".to_string(),
                timestamp_ns: now_ns(),
                qw: 1.0,
                ..PoseState::default()
            };
            shared.map.frame_id = "map".to_string();
            shared.map.revision = 1;
        }
        handle
            .nav
            .observe_stack_state(crate::lcm_runtime::NavState {
                status: "IDLE".to_string(),
                timestamp_ns: now_ns(),
                ..crate::lcm_runtime::NavState::default()
            })
            .await;
        handle
            .nav
            .observe_tf_chain(crate::nav_runtime::TfChainObservation {
                generation: 1,
                map_frame: "map".to_string(),
                odom_frame: "odom".to_string(),
                base_frame: "base_link".to_string(),
                map_to_odom: true,
                odom_to_base: true,
            })
            .expect("TF observation");
        handle
            .nav
            .observe_planner_input(crate::nav_runtime::PlannerInputObservation {
                generation: 1,
                map_revision: 1,
                map_frame: "map".to_string(),
                pose_frame: "map".to_string(),
                ready: true,
            })
            .expect("planner observation");
        handle
            .nav
            .observe_motion_sink(crate::nav_runtime::MotionSinkObservation {
                generation: 1,
                ready: true,
            })
            .expect("motion sink");
        let service = GeometryRuntimeService::new(handle);
        let goal = GeometryRuntime::set_navigation_goal(
            &service,
            Request::new(NavigationGoal {
                pose: Some(Pose {
                    orientation: Some(Quat {
                        w: 1.0,
                        ..Quat::default()
                    }),
                    frame_id: "map".to_string(),
                    timestamp_ns: now_ns(),
                    ..Pose::default()
                }),
                frame_id: "map".to_string(),
                goal_id: "public-goal".to_string(),
                map_version_id: "map-v1".to_string(),
                motion_guard_generation: 7,
                motion_guard_boot_nonce: vec![7; 16],
                motion_stop_generation: 1,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect("public goal")
        .into_inner();
        assert_eq!(goal.status, "ACCEPTED");
        assert_eq!(
            goal.owner_kind,
            nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
        );
        assert_eq!(goal.parent_request_id, "public-goal");
        assert!(goal.goal_generation > 0);
        assert!(goal.producer_runtime_generation > 0);
        assert_eq!(goal.goal_payload_sha256.len(), 32);
        assert_eq!(goal.map_version_id, "map-v1");

        let wrong_producer_runtime_generation = if goal.producer_runtime_generation == 1 {
            2
        } else {
            goal.producer_runtime_generation - 1
        };
        let stale_cancel = GeometryRuntime::cancel_navigation(
            &service,
            Request::new(CancelNavigationRequest {
                goal_id: goal.goal_id.clone(),
                owner_kind: goal.owner_kind,
                parent_request_id: goal.parent_request_id.clone(),
                goal_generation: goal.goal_generation,
                producer_runtime_generation: wrong_producer_runtime_generation,
            }),
        )
        .await
        .expect("stale producer cancel response")
        .into_inner();
        assert_eq!(stale_cancel.status, "FAILED");
        assert!(stale_cancel.message.contains("GOAL_LEASE_MISMATCH"));
        assert_eq!(service.bridge.nav.cancel_snapshot().generation, 0);

        let cancelled = GeometryRuntime::cancel_navigation(
            &service,
            Request::new(CancelNavigationRequest {
                goal_id: goal.goal_id.clone(),
                owner_kind: goal.owner_kind,
                parent_request_id: goal.parent_request_id.clone(),
                goal_generation: goal.goal_generation,
                producer_runtime_generation: goal.producer_runtime_generation,
            }),
        )
        .await
        .expect("correlated cancel")
        .into_inner();
        assert_eq!(cancelled.status, "PENDING");
        assert_eq!(cancelled.owner_kind, goal.owner_kind);
        assert_eq!(cancelled.parent_request_id, goal.parent_request_id);
        assert_eq!(cancelled.goal_generation, goal.goal_generation);
        assert_eq!(
            cancelled.producer_runtime_generation,
            goal.producer_runtime_generation
        );
        assert_eq!(cancelled.goal_payload_sha256, goal.goal_payload_sha256);

        let rejected = GeometryRuntime::set_navigation_goal(
            &service,
            Request::new(NavigationGoal {
                owner_kind: nuwax_codec::lcm::schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
                ..NavigationGoal::default()
            }),
        )
        .await
        .expect_err("public RPC cannot forge exploration owner");
        assert_eq!(rejected.code(), tonic::Code::InvalidArgument);
    }
}
