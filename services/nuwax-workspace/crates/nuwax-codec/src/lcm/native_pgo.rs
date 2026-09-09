//! 原生PGO第二版协议：校正携完整优化轨迹，基本传感器证明保持原定义。
//!
//! The repository IDL is authoritative. `*_body` functions operate on the
//! generated body without the top-level eight-byte LCM fingerprint; `*_wire`
//! functions operate on the complete generated value. Payload digests exclude
//! both that fingerprint and the final self-digest field.

use crate::{hash::sha256_bytes, CodecError, CodecResult};

/// Locked `nuwax_msgs.NativePgoRequest` V1 fingerprint from lcm-gen 1.5.2.
pub const NATIVE_PGO_REQUEST_FINGERPRINT_V2: [u8; 8] = 0xa714e98d60d0b620u64.to_be_bytes();
/// Locked `nuwax_msgs.NativePgoResult` V1 fingerprint from lcm-gen 1.5.2.
pub const NATIVE_PGO_RESULT_FINGERPRINT_V2: [u8; 8] = 0xadd11e19aa0ec18bu64.to_be_bytes();
/// Locked `nuwax_msgs.NativePgoStatus` V1 fingerprint from lcm-gen 1.5.2.
pub const NATIVE_PGO_STATUS_FINGERPRINT_V2: [u8; 8] = 0x3091859182a57b03u64.to_be_bytes();

/// First immutable native-PGO wire schema.
pub const NATIVE_PGO_SCHEMA_VERSION_V2: i32 = 2;
/// Native-PGO backend code, aligned with `CorrectionCandidate`.
pub const NATIVE_PGO_BACKEND: i32 = 2;
/// Full SE(3) correction applied on the left of the frozen odometry pose.
pub const NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3: i32 = 1;
/// Maximum registered body points accepted by V1.
pub const MAX_NATIVE_PGO_REGISTERED_BODY_POINT_COUNT: i64 = 100_000;
/// Maximum flattened XYZ coordinates accepted by V1.
pub const MAX_NATIVE_PGO_REGISTERED_BODY_COORDINATE_COUNT: i32 = 300_000;
/// Maximum committed occupancy cells accepted by V1.
pub const MAX_NATIVE_PGO_COMMITTED_MAP_CELL_COUNT: i32 = 67_108_864;

/// No admissible loop was found; this is a successful terminal result.
pub const NATIVE_PGO_DISPOSITION_NO_CANDIDATE: i32 = 1;
/// A complete full-SE(3) correction and proof are present.
pub const NATIVE_PGO_DISPOSITION_CORRECTION: i32 = 2;
/// The admitted request was rejected with a typed reason.
pub const NATIVE_PGO_DISPOSITION_REJECTED: i32 = 3;
/// No result rejection.
pub const NATIVE_PGO_REJECTION_NONE: i32 = 0;
/// The admitted graph was too small to optimize.
pub const NATIVE_PGO_REJECTION_INSUFFICIENT_GRAPH: i32 = 1;
/// The admitted graph was structurally invalid.
pub const NATIVE_PGO_REJECTION_INVALID_GRAPH: i32 = 2;
/// Optimization encountered a numerical failure.
pub const NATIVE_PGO_REJECTION_NUMERIC_FAILURE: i32 = 3;
/// Optimization exceeded a bounded resource limit.
pub const NATIVE_PGO_REJECTION_RESOURCE_LIMIT: i32 = 4;
/// The admitted request was cancelled before optimization completed.
pub const NATIVE_PGO_REJECTION_CANCELLED: i32 = 5;
/// Maximum graph nodes emitted by the online SimplePGO owner.
pub const MAX_NATIVE_PGO_GRAPH_NODES: i64 = 2_000;
/// Maximum total graph edges for 2,000 nodes and at most one loop per admitted frame.
pub const MAX_NATIVE_PGO_GRAPH_EDGES: i64 = 3_997;
/// Maximum loop edges; the first possible loop is at node index two.
pub const MAX_NATIVE_PGO_GRAPH_LOOP_EDGES: i64 = 1_998;
/// Maximum ICP loop inliers, bounded by the deterministic source-cloud cap.
pub const MAX_NATIVE_PGO_LOOP_INLIERS: i64 = 20_000;
/// Maximum iterations configured by the online owner.
pub const MAX_NATIVE_PGO_OPTIMIZATION_ITERATIONS: i32 = 300;

/// Owner is starting.
pub const NATIVE_PGO_EVENT_STARTING: i32 = 1;
/// Owner has subscribed and is ready to accept requests.
pub const NATIVE_PGO_EVENT_READY: i32 = 2;
/// Owner health sample.
pub const NATIVE_PGO_EVENT_HEALTH: i32 = 3;
/// A request passed protocol admission.
pub const NATIVE_PGO_EVENT_REQUEST_ACCEPTED: i32 = 4;
/// An admitted request is running.
pub const NATIVE_PGO_EVENT_RUNNING: i32 = 5;
/// Per-request terminal status; it never replaces `NativePgoResult`.
pub const NATIVE_PGO_EVENT_TERMINAL: i32 = 6;
/// Geometry requests cancellation on the private cancel channel.
pub const NATIVE_PGO_EVENT_CANCEL_REQUESTED: i32 = 7;
/// Native owner acknowledges cancellation or reports it was too late.
pub const NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED: i32 = 8;
/// Typed protocol failure.
pub const NATIVE_PGO_EVENT_PROTOCOL_ERROR: i32 = 9;
/// Owner is stopping.
pub const NATIVE_PGO_EVENT_STOPPING: i32 = 10;
/// Owner stopped.
pub const NATIVE_PGO_EVENT_STOPPED: i32 = 11;

/// Health is not applicable to the event.
pub const NATIVE_PGO_HEALTH_UNKNOWN: i32 = 0;
/// Owner is healthy.
pub const NATIVE_PGO_HEALTH_HEALTHY: i32 = 1;
/// Owner is degraded but remains able to report typed state.
pub const NATIVE_PGO_HEALTH_DEGRADED: i32 = 2;
/// Owner is unhealthy.
pub const NATIVE_PGO_HEALTH_UNHEALTHY: i32 = 3;
/// Event is not terminal.
pub const NATIVE_PGO_TERMINAL_NONE: i32 = 0;
/// Request completed and emitted a result.
pub const NATIVE_PGO_TERMINAL_SUCCEEDED: i32 = 1;
/// Request failed after admission.
pub const NATIVE_PGO_TERMINAL_FAILED: i32 = 2;
/// Request completed with `DISPOSITION_REJECTED`.
pub const NATIVE_PGO_TERMINAL_REJECTED: i32 = 3;
/// Request was cancelled.
pub const NATIVE_PGO_TERMINAL_CANCELLED: i32 = 4;
/// Event has no cancellation state.
pub const NATIVE_PGO_CANCEL_NONE: i32 = 0;
/// Cancellation was requested.
pub const NATIVE_PGO_CANCEL_REQUESTED: i32 = 1;
/// Cancellation was acknowledged.
pub const NATIVE_PGO_CANCEL_ACKNOWLEDGED: i32 = 2;
/// Result was already terminal when cancellation arrived.
pub const NATIVE_PGO_CANCEL_TOO_LATE: i32 = 3;

/// No status reason.
pub const NATIVE_PGO_REASON_NONE: i32 = 0;
/// Startup readiness timed out.
pub const NATIVE_PGO_REASON_STARTUP_TIMEOUT: i32 = 1;
/// Startup preflight rejected the owner.
pub const NATIVE_PGO_REASON_PREFLIGHT_FAILED: i32 = 2;
/// Typed transport failed.
pub const NATIVE_PGO_REASON_TRANSPORT_ERROR: i32 = 3;
/// A generated value could not be decoded.
pub const NATIVE_PGO_REASON_PROTOCOL_DECODE_FAILED: i32 = 4;
/// Request or response identity did not correlate.
pub const NATIVE_PGO_REASON_PROTOCOL_IDENTITY_MISMATCH: i32 = 5;
/// A canonical payload digest did not match.
pub const NATIVE_PGO_REASON_PROTOCOL_HASH_MISMATCH: i32 = 6;
/// Request fields failed protocol admission.
pub const NATIVE_PGO_REASON_INVALID_REQUEST: i32 = 7;
/// Optimization failed after admission.
pub const NATIVE_PGO_REASON_OPTIMIZATION_FAILED: i32 = 8;
/// Geometry requested shutdown.
pub const NATIVE_PGO_REASON_SHUTDOWN_REQUESTED: i32 = 9;
/// Geometry requested cancellation.
pub const NATIVE_PGO_REASON_CANCEL_REQUESTED: i32 = 10;
/// Cancellation completed.
pub const NATIVE_PGO_REASON_CANCELLED: i32 = 11;
/// Cancellation arrived after terminal completion.
pub const NATIVE_PGO_REASON_CANCEL_TOO_LATE: i32 = 12;
/// Native child exited normally.
pub const NATIVE_PGO_REASON_CHILD_EXITED: i32 = 13;
/// Native child crashed.
pub const NATIVE_PGO_REASON_CHILD_CRASHED: i32 = 14;

/// Request self-digest domain, including its NUL terminator.
pub const NATIVE_PGO_REQUEST_HASH_DOMAIN: &[u8] = b"nuwax.native-pgo.request.v2\0";
/// Result self-digest domain, including its NUL terminator.
pub const NATIVE_PGO_RESULT_HASH_DOMAIN: &[u8] = b"nuwax.native-pgo.result.v2\0";
/// Status self-digest domain, including its NUL terminator.
pub const NATIVE_PGO_STATUS_HASH_DOMAIN: &[u8] = b"nuwax.native-pgo.status.v2\0";
/// Loop-proof digest domain, including its NUL terminator.
pub const NATIVE_PGO_LOOP_PROOF_HASH_DOMAIN: &[u8] = b"nuwax.native-pgo.loop-proof.v1\0";
/// Optimized-graph digest domain, including its NUL terminator.
pub const NATIVE_PGO_OPTIMIZED_GRAPH_HASH_DOMAIN: &[u8] = b"nuwax.native-pgo.optimized-graph.v2\0";

const BODY_POINTS_HASH_DOMAIN: &[u8] = b"nuwax.correction.body-points.v1\0";
const ODOM_POSE_HASH_DOMAIN: &[u8] = b"nuwax.correction.odom-from-body.v1\0";
const MAP_BOUNDARY_HASH_DOMAIN: &[u8] = b"nuwax.correction.committed-map-boundary.v1\0";
const MAX_IDENTITY_STRING_BYTES: usize = 255;
const MAX_REASON_BYTES: usize = 1_024;
const MAX_TRANSLATION_ABS_METERS: f64 = 1_000_000.0;
const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-6;

/// Stable mapping-commit identity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NativePgoMappingCommitId {
    /// Mapping process boot nonce.
    pub boot_nonce: [u8; 16],
    /// Monotonic commit sequence within that boot.
    pub sequence: i64,
}

/// Proof for the registered body-frame cloud.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NativePgoRegisteredCloudProof {
    /// Source cloud sequence.
    pub source_sequence: i64,
    /// Source cloud timestamp in nanoseconds; zero is valid.
    pub source_timestamp_ns: i64,
    /// Number of body-frame XYZ points.
    pub point_count: i64,
    /// Digest of the complete generated `LioCloudEnvelope` value.
    pub envelope_sha256: [u8; 32],
    /// Digest of the canonical body-frame XYZ triples.
    pub body_points_sha256: [u8; 32],
}

/// Proof for the frozen absolute odometry pose.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NativePgoCorrelatedOdomProof {
    /// Source odometry sequence.
    pub source_sequence: i64,
    /// Source odometry timestamp in nanoseconds; zero is valid.
    pub source_timestamp_ns: i64,
    /// Digest of the frozen absolute `odom <- body` pose.
    pub pose_sha256: [u8; 32],
}

/// Proof and geometry for the committed occupancy map.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NativePgoCommittedMapProof {
    /// Committed map source timestamp in nanoseconds; zero is valid.
    pub source_timestamp_ns: i64,
    /// Map width in cells.
    pub width: i64,
    /// Map height in cells.
    pub height: i64,
    /// Cell resolution in metres.
    pub resolution_m: f64,
    /// Map origin X in metres.
    pub origin_x_m: f64,
    /// Map origin Y in metres.
    pub origin_y_m: f64,
    /// Digest of the canonical signed occupancy bytes.
    pub payload_sha256: [u8; 32],
    /// Digest binding frame and map geometry.
    pub boundary_sha256: [u8; 32],
}

/// Geometry-owned correction lease identity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NativePgoCorrectionLeaseId {
    /// Geometry process boot nonce.
    pub boot_nonce: [u8; 16],
    /// Monotonic lease generation.
    pub generation: i64,
}

/// Request identity echoed by every request-scoped result and status.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NativePgoRequestIdentity {
    /// Backend code; V1 only accepts native PGO.
    pub backend: i32,
    /// Geometry runtime generation.
    pub runtime_generation: i64,
    /// Native backend generation within the runtime.
    pub backend_generation: i64,
    /// Monotonic request sequence within the backend generation.
    pub request_sequence: i64,
    /// Mapping session identifier.
    pub mapping_session_id: String,
    /// Mapping session generation.
    pub mapping_session_generation: i64,
    /// Generation of the authoritative map frame.
    pub map_frame_generation: i64,
    /// Mapping commit identity.
    pub mapping_commit_id: NativePgoMappingCommitId,
    /// Independently monotonic occupancy revision.
    pub occupancy_revision: i64,
    /// Independently monotonic cumulative cloud-map revision.
    pub cloud_revision: i64,
    /// Registered-cloud proof.
    pub registered_cloud: NativePgoRegisteredCloudProof,
    /// Frozen correlated-odometry proof.
    pub correlated_odom: NativePgoCorrelatedOdomProof,
    /// Committed-map proof.
    pub committed_map: NativePgoCommittedMapProof,
    /// Canonical request source timestamp in nanoseconds.
    pub source_timestamp_ns: i64,
    /// Authoritative map frame.
    pub map_frame_id: String,
    /// Authoritative odometry frame.
    pub odom_frame_id: String,
    /// Authoritative robot body frame.
    pub body_frame_id: String,
    /// Point coordinate frame; V1 requires the body frame.
    pub points_frame_id: String,
    /// Geometry correction lease.
    pub correction_lease: NativePgoCorrectionLeaseId,
}

/// Complete input for one native-PGO optimization attempt.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NativePgoRequest {
    /// Wire schema version.
    pub schema_version: i32,
    /// Immutable request identity and proofs.
    pub identity: NativePgoRequestIdentity,
    /// Redundant generated-wire coordinate count.
    pub registered_body_coordinate_count: i32,
    /// Flattened body-frame XYZ triples.
    pub registered_body_points_xyz: Vec<f64>,
    /// Frozen absolute odometry translation.
    pub frozen_odom_translation_m: [f64; 3],
    /// Frozen absolute odometry quaternion in XYZW order.
    pub frozen_odom_rotation_xyzw: [f64; 4],
    /// Redundant generated-wire occupancy cell count.
    pub committed_map_cell_count: i32,
    /// Signed occupancy payload.
    pub committed_map_occupancy: Vec<i8>,
    /// Self-digest over the canonical body prefix.
    pub request_payload_sha256: [u8; 32],
}

/// Exact request identity and digest echoed by the native owner.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NativePgoRequestEcho {
    /// Immutable request identity and proof values.
    pub identity: NativePgoRequestIdentity,
    /// Exact request self-digest.
    pub request_payload_sha256: [u8; 32],
}

impl NativePgoRequest {
    /// Returns the exact identity/hash tuple required in result and status messages.
    #[must_use]
    pub fn echo(&self) -> NativePgoRequestEcho {
        NativePgoRequestEcho {
            identity: self.identity.clone(),
            request_payload_sha256: self.request_payload_sha256,
        }
    }
}

/// Per-request terminal native-PGO result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NativePgoResult {
    /// Wire schema version.
    pub schema_version: i32,
    /// Exact request identity and digest.
    pub request: NativePgoRequestEcho,
    /// One of the `NATIVE_PGO_DISPOSITION_*` values.
    pub result_disposition: i32,
    /// Result timestamp in nanoseconds; zero is valid.
    pub result_timestamp_ns: i64,
    /// Full-SE(3) transform semantics for correction results, otherwise zero.
    pub transform_semantics: i32,
    /// Absolute-left correction translation.
    pub translation_m: [f64; 3],
    /// Absolute-left correction quaternion in XYZW order.
    pub rotation_xyzw: [f64; 4],
    /// Nonnegative correction fitness score; lower is better.
    pub fitness_score: f64,
    /// Nonnegative inlier RMSE in metres.
    pub inlier_rmse_m: f64,
    /// Source loop node identifier.
    pub loop_source_node_id: i64,
    /// Target loop node identifier.
    pub loop_target_node_id: i64,
    /// Number of loop inliers.
    pub loop_inlier_count: i64,
    /// Canonical loop-proof digest.
    pub loop_proof_sha256: [u8; 32],
    /// Number of optimized graph nodes.
    pub optimized_graph_node_count: i64,
    /// Number of optimized graph edges.
    pub optimized_graph_edge_count: i64,
    /// Number of loop edges in the optimized graph.
    pub optimized_graph_loop_edge_count: i64,
    /// Bounded optimizer iteration count.
    pub optimization_iterations: i32,
    /// Generated-wire boolean: zero or one.
    pub optimization_converged: i8,
    /// Nonnegative cost before optimization.
    pub optimization_initial_cost: f64,
    /// Nonnegative cost after optimization.
    pub optimization_final_cost: f64,
    /// Canonical optimized-graph metadata digest.
    pub optimized_graph_sha256: [u8; 32],
    /// 校正必须同时提供完整优化轨迹，不允许只移动公共位姿。
    pub trajectory: Vec<OptimizedKeyframe>,
    /// Typed rejection reason for `DISPOSITION_REJECTED`.
    pub rejection_reason_code: i32,
    /// Bounded rejection detail for `DISPOSITION_REJECTED`.
    pub rejection_reason: String,
    /// Self-digest over the canonical body prefix.
    pub result_payload_sha256: [u8; 32],
}

/// 精确源时间对应的优化机体位姿，供规范地图与位姿一起重建。
#[derive(Clone, Debug, Default, PartialEq)]
pub struct OptimizedKeyframe {
    pub source_timestamp_ns: i64,
    pub translation_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

/// 校验轨迹的边界、真实顺序和完整刚体变换；空轨迹表示未提供地图更新。
pub fn validate_optimized_keyframes(frames: &[OptimizedKeyframe]) -> Result<(), String> {
    if frames.len() > MAX_NATIVE_PGO_GRAPH_NODES as usize {
        return Err("trajectory exceeds node bound".into());
    }
    let mut previous = None;
    for frame in frames {
        if frame.source_timestamp_ns < 0
            || previous.is_some_and(|stamp| stamp >= frame.source_timestamp_ns)
        {
            return Err("trajectory source timestamps must increase".into());
        }
        validate_transform(&frame.translation_m, &frame.rotation_xyzw, "trajectory")?;
        previous = Some(frame.source_timestamp_ns);
    }
    Ok(())
}

/// 与生成LCM类型相同的列式轨迹编码。
pub fn encode_optimized_keyframes(frames: &[OptimizedKeyframe]) -> CodecResult<Vec<u8>> {
    validate_optimized_keyframes(frames).map_err(encode_error)?;
    let mut out = Vec::with_capacity(4 + 64 * frames.len());
    out.extend_from_slice(&(frames.len() as i32).to_be_bytes());
    for frame in frames {
        out.extend_from_slice(&frame.source_timestamp_ns.to_be_bytes());
    }
    for frame in frames {
        for value in frame.translation_m {
            out.extend_from_slice(&value.to_be_bytes());
        }
    }
    for frame in frames {
        for value in frame.rotation_xyzw {
            out.extend_from_slice(&value.to_be_bytes());
        }
    }
    Ok(out)
}

/// 解码独立轨迹载荷，先检查长度再分配，拒绝尾随数据。
pub fn decode_optimized_keyframes(body: &[u8]) -> CodecResult<Vec<OptimizedKeyframe>> {
    let mut cursor = Cursor::new(body);
    let result = decode_trajectory(&mut cursor)?;
    cursor.finish("optimized trajectory")?;
    Ok(result)
}

fn decode_trajectory(cursor: &mut Cursor<'_>) -> CodecResult<Vec<OptimizedKeyframe>> {
    let count = cursor.read_i32()?;
    if count < 0 || i64::from(count) > MAX_NATIVE_PGO_GRAPH_NODES {
        return Err(decode_error("invalid trajectory count".into()));
    }
    let count = count as usize;
    if cursor.input.len().saturating_sub(cursor.offset) < count * 64 {
        return Err(decode_error("truncated trajectory".into()));
    }
    let mut frames = vec![OptimizedKeyframe::default(); count];
    for frame in &mut frames {
        frame.source_timestamp_ns = cursor.read_i64()?;
    }
    for frame in &mut frames {
        frame.translation_m = cursor.read_f64_array()?;
    }
    for frame in &mut frames {
        frame.rotation_xyzw = cursor.read_f64_array()?;
    }
    validate_optimized_keyframes(&frames).map_err(decode_error)?;
    Ok(frames)
}

/// Typed native-PGO owner lifecycle, health, terminal, and cancellation event.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NativePgoStatus {
    /// Wire schema version.
    pub schema_version: i32,
    /// Backend code; V1 only accepts native PGO.
    pub backend: i32,
    /// Geometry runtime generation.
    pub runtime_generation: i64,
    /// Native backend generation.
    pub backend_generation: i64,
    /// Monotonic status sequence.
    pub status_sequence: i64,
    /// Status timestamp in nanoseconds; zero is valid.
    pub status_timestamp_ns: i64,
    /// Exact request identity for request-scoped events; absent for owner events.
    pub request: Option<NativePgoRequestEcho>,
    /// One of the `NATIVE_PGO_EVENT_*` values.
    pub event_kind: i32,
    /// One of the `NATIVE_PGO_HEALTH_*` values.
    pub health_state: i32,
    /// One of the `NATIVE_PGO_TERMINAL_*` values.
    pub terminal_state: i32,
    /// One of the `NATIVE_PGO_CANCEL_*` values.
    pub cancel_state: i32,
    /// One of the `NATIVE_PGO_REASON_*` values.
    pub reason_code: i32,
    /// Bounded typed-reason detail.
    pub reason: String,
    /// Self-digest over the canonical body prefix.
    pub status_payload_sha256: [u8; 32],
}

/// Computes the canonical body-point proof used by correction candidates.
pub fn compute_native_pgo_body_points_sha256(points_xyz: &[f64]) -> CodecResult<[u8; 32]> {
    if points_xyz.is_empty()
        || points_xyz.len() % 3 != 0
        || points_xyz.len()
            > usize::try_from(MAX_NATIVE_PGO_REGISTERED_BODY_COORDINATE_COUNT)
                .expect("coordinate limit fits usize")
        || points_xyz.iter().any(|value| !value.is_finite())
    {
        return Err(encode_error(
            "registered body points must be finite nonempty XYZ triples within the V1 limit".into(),
        ));
    }
    let point_count = u64::try_from(points_xyz.len() / 3)
        .map_err(|_| encode_error("registered body point count overflow".into()))?;
    let mut canonical =
        Vec::with_capacity(BODY_POINTS_HASH_DOMAIN.len() + 8 + points_xyz.len() * 8);
    canonical.extend_from_slice(BODY_POINTS_HASH_DOMAIN);
    canonical.extend_from_slice(&point_count.to_be_bytes());
    for value in points_xyz {
        canonical.extend_from_slice(&value.to_be_bytes());
    }
    Ok(sha256_bytes(&canonical))
}

/// Computes the canonical frozen absolute-odometry pose proof.
pub fn compute_native_pgo_odom_pose_sha256(
    translation_m: &[f64; 3],
    rotation_xyzw: &[f64; 4],
) -> CodecResult<[u8; 32]> {
    validate_transform(translation_m, rotation_xyzw, "frozen odometry").map_err(encode_error)?;
    let mut canonical = Vec::with_capacity(ODOM_POSE_HASH_DOMAIN.len() + 56);
    canonical.extend_from_slice(ODOM_POSE_HASH_DOMAIN);
    for value in translation_m.iter().chain(rotation_xyzw.iter()) {
        canonical.extend_from_slice(&value.to_be_bytes());
    }
    Ok(sha256_bytes(&canonical))
}

/// Computes the canonical committed signed-occupancy payload proof.
#[must_use]
pub fn compute_native_pgo_map_payload_sha256(occupancy: &[i8]) -> [u8; 32] {
    let bytes: Vec<u8> = occupancy.iter().map(|value| *value as u8).collect();
    sha256_bytes(&bytes)
}

/// Computes the canonical committed-map boundary proof.
pub fn compute_native_pgo_map_boundary_sha256(
    map_frame_id: &str,
    proof: &NativePgoCommittedMapProof,
) -> CodecResult<[u8; 32]> {
    validate_frame_id("map_frame_id", map_frame_id).map_err(encode_error)?;
    if proof.width <= 0
        || proof.height <= 0
        || !proof.resolution_m.is_finite()
        || proof.resolution_m <= 0.0
        || !proof.origin_x_m.is_finite()
        || !proof.origin_y_m.is_finite()
    {
        return Err(encode_error("committed map geometry is invalid".into()));
    }
    let mut canonical = Vec::new();
    canonical.extend_from_slice(MAP_BOUNDARY_HASH_DOMAIN);
    encode_string(map_frame_id, &mut canonical)?;
    canonical.extend_from_slice(&proof.width.to_be_bytes());
    canonical.extend_from_slice(&proof.height.to_be_bytes());
    canonical.extend_from_slice(&proof.resolution_m.to_be_bytes());
    canonical.extend_from_slice(&proof.origin_x_m.to_be_bytes());
    canonical.extend_from_slice(&proof.origin_y_m.to_be_bytes());
    Ok(sha256_bytes(&canonical))
}

/// Computes the correction-only loop proof bound to the request digest.
pub fn compute_native_pgo_loop_proof_sha256(result: &NativePgoResult) -> CodecResult<[u8; 32]> {
    validate_loop_metadata(result).map_err(encode_error)?;
    let mut canonical = Vec::new();
    canonical.extend_from_slice(NATIVE_PGO_LOOP_PROOF_HASH_DOMAIN);
    canonical.extend_from_slice(&result.request.request_payload_sha256);
    canonical.extend_from_slice(&result.loop_source_node_id.to_be_bytes());
    canonical.extend_from_slice(&result.loop_target_node_id.to_be_bytes());
    canonical.extend_from_slice(&result.loop_inlier_count.to_be_bytes());
    canonical.extend_from_slice(&result.fitness_score.to_be_bytes());
    canonical.extend_from_slice(&result.inlier_rmse_m.to_be_bytes());
    Ok(sha256_bytes(&canonical))
}

/// Computes the correction-only optimized-graph metadata proof.
pub fn compute_native_pgo_optimized_graph_sha256(
    result: &NativePgoResult,
) -> CodecResult<[u8; 32]> {
    validate_graph_metadata(result).map_err(encode_error)?;
    validate_transform(&result.translation_m, &result.rotation_xyzw, "correction")
        .map_err(encode_error)?;
    let mut canonical = Vec::new();
    canonical.extend_from_slice(NATIVE_PGO_OPTIMIZED_GRAPH_HASH_DOMAIN);
    canonical.extend_from_slice(&result.request.request_payload_sha256);
    canonical.extend_from_slice(&result.optimized_graph_node_count.to_be_bytes());
    canonical.extend_from_slice(&result.optimized_graph_edge_count.to_be_bytes());
    canonical.extend_from_slice(&result.optimized_graph_loop_edge_count.to_be_bytes());
    canonical.extend_from_slice(&result.optimization_iterations.to_be_bytes());
    canonical.push(result.optimization_converged as u8);
    canonical.extend_from_slice(&result.optimization_initial_cost.to_be_bytes());
    canonical.extend_from_slice(&result.optimization_final_cost.to_be_bytes());
    for value in result
        .translation_m
        .iter()
        .chain(result.rotation_xyzw.iter())
    {
        canonical.extend_from_slice(&value.to_be_bytes());
    }
    canonical.extend_from_slice(&encode_optimized_keyframes(&result.trajectory)?);
    Ok(sha256_bytes(&canonical))
}

/// Computes the request self-digest over the generated body before its final digest.
pub fn compute_native_pgo_request_payload_sha256(
    message: &NativePgoRequest,
) -> CodecResult<[u8; 32]> {
    Ok(hash_body_prefix(
        NATIVE_PGO_REQUEST_HASH_DOMAIN,
        &encode_request_body_prefix(message)?,
    ))
}

/// Computes the result self-digest over the generated body before its final digest.
pub fn compute_native_pgo_result_payload_sha256(
    message: &NativePgoResult,
) -> CodecResult<[u8; 32]> {
    Ok(hash_body_prefix(
        NATIVE_PGO_RESULT_HASH_DOMAIN,
        &encode_result_body_prefix(message)?,
    ))
}

/// Computes the status self-digest over the generated body before its final digest.
pub fn compute_native_pgo_status_payload_sha256(
    message: &NativePgoStatus,
) -> CodecResult<[u8; 32]> {
    Ok(hash_body_prefix(
        NATIVE_PGO_STATUS_HASH_DOMAIN,
        &encode_status_body_prefix(message)?,
    ))
}

/// Fills all request proofs derivable from its typed payload and seals its self-digest.
pub fn seal_native_pgo_request(mut message: NativePgoRequest) -> CodecResult<NativePgoRequest> {
    message.identity.registered_cloud.body_points_sha256 =
        compute_native_pgo_body_points_sha256(&message.registered_body_points_xyz)?;
    message.identity.correlated_odom.pose_sha256 = compute_native_pgo_odom_pose_sha256(
        &message.frozen_odom_translation_m,
        &message.frozen_odom_rotation_xyzw,
    )?;
    message.identity.committed_map.payload_sha256 =
        compute_native_pgo_map_payload_sha256(&message.committed_map_occupancy);
    message.identity.committed_map.boundary_sha256 = compute_native_pgo_map_boundary_sha256(
        &message.identity.map_frame_id,
        &message.identity.committed_map,
    )?;
    message.request_payload_sha256 = compute_native_pgo_request_payload_sha256(&message)?;
    Ok(message)
}

/// Fills correction-only derived proofs and seals the result self-digest.
pub fn seal_native_pgo_result(mut message: NativePgoResult) -> CodecResult<NativePgoResult> {
    if message.result_disposition == NATIVE_PGO_DISPOSITION_CORRECTION {
        message.loop_proof_sha256 = compute_native_pgo_loop_proof_sha256(&message)?;
        message.optimized_graph_sha256 = compute_native_pgo_optimized_graph_sha256(&message)?;
    }
    message.result_payload_sha256 = compute_native_pgo_result_payload_sha256(&message)?;
    Ok(message)
}

/// Seals a typed status self-digest.
pub fn seal_native_pgo_status(mut message: NativePgoStatus) -> CodecResult<NativePgoStatus> {
    message.status_payload_sha256 = compute_native_pgo_status_payload_sha256(&message)?;
    Ok(message)
}

/// Encodes a generated request body without its top-level fingerprint.
pub fn encode_native_pgo_request_body(message: &NativePgoRequest) -> CodecResult<Vec<u8>> {
    validate_request_integrity(message).map_err(encode_error)?;
    let mut body = encode_request_body_prefix(message)?;
    body.extend_from_slice(&message.request_payload_sha256);
    Ok(body)
}

/// Encodes a complete generated request value including its fingerprint.
pub fn encode_native_pgo_request_wire(message: &NativePgoRequest) -> CodecResult<Vec<u8>> {
    encode_wire(
        NATIVE_PGO_REQUEST_FINGERPRINT_V2,
        encode_native_pgo_request_body(message)?,
    )
}

/// Decodes and validates a generated request body without a fingerprint.
pub fn decode_native_pgo_request_body(body: &[u8]) -> CodecResult<NativePgoRequest> {
    let mut cursor = Cursor::new(body);
    let message = decode_request_body_from_cursor(&mut cursor)?;
    cursor.finish("nuwax_msgs.NativePgoRequest body")?;
    validate_request_integrity(&message).map_err(decode_error)?;
    Ok(message)
}

/// Decodes and validates a complete generated request value.
pub fn decode_native_pgo_request_wire(wire: &[u8]) -> CodecResult<NativePgoRequest> {
    decode_wire(
        wire,
        NATIVE_PGO_REQUEST_FINGERPRINT_V2,
        "nuwax_msgs.NativePgoRequest",
        decode_native_pgo_request_body,
    )
}

/// Encodes a generated result body without its top-level fingerprint.
pub fn encode_native_pgo_result_body(message: &NativePgoResult) -> CodecResult<Vec<u8>> {
    validate_result_integrity(message).map_err(encode_error)?;
    let mut body = encode_result_body_prefix(message)?;
    body.extend_from_slice(&message.result_payload_sha256);
    Ok(body)
}

/// Encodes a complete generated result value including its fingerprint.
pub fn encode_native_pgo_result_wire(message: &NativePgoResult) -> CodecResult<Vec<u8>> {
    encode_wire(
        NATIVE_PGO_RESULT_FINGERPRINT_V2,
        encode_native_pgo_result_body(message)?,
    )
}

/// Decodes and validates a generated result body without a fingerprint.
pub fn decode_native_pgo_result_body(body: &[u8]) -> CodecResult<NativePgoResult> {
    let mut cursor = Cursor::new(body);
    let message = decode_result_body_from_cursor(&mut cursor)?;
    cursor.finish("nuwax_msgs.NativePgoResult body")?;
    validate_result_integrity(&message).map_err(decode_error)?;
    Ok(message)
}

/// Decodes and validates a complete generated result value.
pub fn decode_native_pgo_result_wire(wire: &[u8]) -> CodecResult<NativePgoResult> {
    decode_wire(
        wire,
        NATIVE_PGO_RESULT_FINGERPRINT_V2,
        "nuwax_msgs.NativePgoResult",
        decode_native_pgo_result_body,
    )
}

/// Encodes a generated status body without its top-level fingerprint.
pub fn encode_native_pgo_status_body(message: &NativePgoStatus) -> CodecResult<Vec<u8>> {
    validate_status_integrity(message).map_err(encode_error)?;
    let mut body = encode_status_body_prefix(message)?;
    body.extend_from_slice(&message.status_payload_sha256);
    Ok(body)
}

/// Encodes a complete generated status value including its fingerprint.
pub fn encode_native_pgo_status_wire(message: &NativePgoStatus) -> CodecResult<Vec<u8>> {
    encode_wire(
        NATIVE_PGO_STATUS_FINGERPRINT_V2,
        encode_native_pgo_status_body(message)?,
    )
}

/// Decodes and validates a generated status body without a fingerprint.
pub fn decode_native_pgo_status_body(body: &[u8]) -> CodecResult<NativePgoStatus> {
    let mut cursor = Cursor::new(body);
    let message = decode_status_body_from_cursor(&mut cursor)?;
    cursor.finish("nuwax_msgs.NativePgoStatus body")?;
    validate_status_integrity(&message).map_err(decode_error)?;
    Ok(message)
}

/// Decodes and validates a complete generated status value.
pub fn decode_native_pgo_status_wire(wire: &[u8]) -> CodecResult<NativePgoStatus> {
    decode_wire(
        wire,
        NATIVE_PGO_STATUS_FINGERPRINT_V2,
        "nuwax_msgs.NativePgoStatus",
        decode_native_pgo_status_body,
    )
}

/// Verifies that a result echoes the request identity and digest exactly.
pub fn validate_native_pgo_result_echo(
    request: &NativePgoRequest,
    result: &NativePgoResult,
) -> CodecResult<()> {
    if result.request == request.echo() {
        Ok(())
    } else {
        Err(CodecError::LcmDecode(
            "native PGO result request identity/hash mismatch".into(),
        ))
    }
}

/// Verifies that a request-scoped status echoes the request exactly.
pub fn validate_native_pgo_status_echo(
    request: &NativePgoRequest,
    status: &NativePgoStatus,
) -> CodecResult<()> {
    if status.request.as_ref() == Some(&request.echo()) {
        Ok(())
    } else {
        Err(CodecError::LcmDecode(
            "native PGO status request identity/hash mismatch".into(),
        ))
    }
}

fn encode_request_body_prefix(message: &NativePgoRequest) -> CodecResult<Vec<u8>> {
    validate_request_fields(message).map_err(encode_error)?;
    let identity = &message.identity;
    let mut out = Vec::new();
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    encode_identity_until_cloud_count(identity, true, &mut out)?;
    out.extend_from_slice(&message.registered_body_coordinate_count.to_be_bytes());
    for value in &message.registered_body_points_xyz {
        out.extend_from_slice(&value.to_be_bytes());
    }
    out.extend_from_slice(&identity.registered_cloud.envelope_sha256);
    out.extend_from_slice(&identity.registered_cloud.body_points_sha256);
    out.extend_from_slice(&identity.correlated_odom.source_sequence.to_be_bytes());
    out.extend_from_slice(&identity.correlated_odom.source_timestamp_ns.to_be_bytes());
    for value in message
        .frozen_odom_translation_m
        .iter()
        .chain(message.frozen_odom_rotation_xyzw.iter())
    {
        out.extend_from_slice(&value.to_be_bytes());
    }
    out.extend_from_slice(&identity.correlated_odom.pose_sha256);
    encode_map_geometry(&identity.committed_map, &mut out);
    out.extend_from_slice(&message.committed_map_cell_count.to_be_bytes());
    out.extend(
        message
            .committed_map_occupancy
            .iter()
            .map(|value| *value as u8),
    );
    out.extend_from_slice(&identity.committed_map.payload_sha256);
    out.extend_from_slice(&identity.committed_map.boundary_sha256);
    encode_identity_tail(identity, &mut out)?;
    Ok(out)
}

fn encode_result_body_prefix(message: &NativePgoResult) -> CodecResult<Vec<u8>> {
    validate_result_fields(message).map_err(encode_error)?;
    let mut out = Vec::new();
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    encode_request_echo(&message.request, &mut out)?;
    out.extend_from_slice(&message.result_disposition.to_be_bytes());
    out.extend_from_slice(&message.result_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&message.transform_semantics.to_be_bytes());
    for value in message
        .translation_m
        .iter()
        .chain(message.rotation_xyzw.iter())
    {
        out.extend_from_slice(&value.to_be_bytes());
    }
    out.extend_from_slice(&message.fitness_score.to_be_bytes());
    out.extend_from_slice(&message.inlier_rmse_m.to_be_bytes());
    out.extend_from_slice(&message.loop_source_node_id.to_be_bytes());
    out.extend_from_slice(&message.loop_target_node_id.to_be_bytes());
    out.extend_from_slice(&message.loop_inlier_count.to_be_bytes());
    out.extend_from_slice(&message.loop_proof_sha256);
    out.extend_from_slice(&message.optimized_graph_node_count.to_be_bytes());
    out.extend_from_slice(&message.optimized_graph_edge_count.to_be_bytes());
    out.extend_from_slice(&message.optimized_graph_loop_edge_count.to_be_bytes());
    out.extend_from_slice(&message.optimization_iterations.to_be_bytes());
    out.push(message.optimization_converged as u8);
    out.extend_from_slice(&message.optimization_initial_cost.to_be_bytes());
    out.extend_from_slice(&message.optimization_final_cost.to_be_bytes());
    out.extend_from_slice(&message.optimized_graph_sha256);
    out.extend_from_slice(&encode_optimized_keyframes(&message.trajectory)?);
    out.extend_from_slice(&message.rejection_reason_code.to_be_bytes());
    encode_string(&message.rejection_reason, &mut out)?;
    Ok(out)
}

fn encode_status_body_prefix(message: &NativePgoStatus) -> CodecResult<Vec<u8>> {
    validate_status_fields(message).map_err(encode_error)?;
    let mut out = Vec::new();
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    out.extend_from_slice(&message.backend.to_be_bytes());
    out.extend_from_slice(&message.runtime_generation.to_be_bytes());
    out.extend_from_slice(&message.backend_generation.to_be_bytes());
    out.extend_from_slice(&message.status_sequence.to_be_bytes());
    out.extend_from_slice(&message.status_timestamp_ns.to_be_bytes());
    match &message.request {
        Some(request) => {
            out.push(1);
            encode_status_identity_tail(request, &mut out)?;
        }
        None => {
            out.push(0);
            encode_status_identity_tail(&NativePgoRequestEcho::default(), &mut out)?;
        }
    }
    out.extend_from_slice(&message.event_kind.to_be_bytes());
    out.extend_from_slice(&message.health_state.to_be_bytes());
    out.extend_from_slice(&message.terminal_state.to_be_bytes());
    out.extend_from_slice(&message.cancel_state.to_be_bytes());
    out.extend_from_slice(&message.reason_code.to_be_bytes());
    encode_string(&message.reason, &mut out)?;
    Ok(out)
}

fn encode_identity_until_cloud_count(
    identity: &NativePgoRequestIdentity,
    include_owner_header: bool,
    out: &mut Vec<u8>,
) -> CodecResult<()> {
    if include_owner_header {
        out.extend_from_slice(&identity.backend.to_be_bytes());
        out.extend_from_slice(&identity.runtime_generation.to_be_bytes());
        out.extend_from_slice(&identity.backend_generation.to_be_bytes());
    }
    out.extend_from_slice(&identity.request_sequence.to_be_bytes());
    encode_string(&identity.mapping_session_id, out)?;
    out.extend_from_slice(&identity.mapping_session_generation.to_be_bytes());
    out.extend_from_slice(&identity.map_frame_generation.to_be_bytes());
    out.extend_from_slice(&identity.mapping_commit_id.boot_nonce);
    out.extend_from_slice(&identity.mapping_commit_id.sequence.to_be_bytes());
    out.extend_from_slice(&identity.occupancy_revision.to_be_bytes());
    out.extend_from_slice(&identity.cloud_revision.to_be_bytes());
    out.extend_from_slice(&identity.committed_map.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&identity.registered_cloud.source_sequence.to_be_bytes());
    out.extend_from_slice(&identity.registered_cloud.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&identity.registered_cloud.point_count.to_be_bytes());
    Ok(())
}

fn encode_map_geometry(proof: &NativePgoCommittedMapProof, out: &mut Vec<u8>) {
    out.extend_from_slice(&proof.width.to_be_bytes());
    out.extend_from_slice(&proof.height.to_be_bytes());
    out.extend_from_slice(&proof.resolution_m.to_be_bytes());
    out.extend_from_slice(&proof.origin_x_m.to_be_bytes());
    out.extend_from_slice(&proof.origin_y_m.to_be_bytes());
}

fn encode_identity_tail(identity: &NativePgoRequestIdentity, out: &mut Vec<u8>) -> CodecResult<()> {
    out.extend_from_slice(&identity.source_timestamp_ns.to_be_bytes());
    encode_string(&identity.map_frame_id, out)?;
    encode_string(&identity.odom_frame_id, out)?;
    encode_string(&identity.body_frame_id, out)?;
    encode_string(&identity.points_frame_id, out)?;
    out.extend_from_slice(&identity.correction_lease.boot_nonce);
    out.extend_from_slice(&identity.correction_lease.generation.to_be_bytes());
    Ok(())
}

fn encode_request_echo(request: &NativePgoRequestEcho, out: &mut Vec<u8>) -> CodecResult<()> {
    let identity = &request.identity;
    encode_identity_until_cloud_count(identity, true, out)?;
    out.extend_from_slice(&identity.registered_cloud.envelope_sha256);
    out.extend_from_slice(&identity.registered_cloud.body_points_sha256);
    out.extend_from_slice(&identity.correlated_odom.source_sequence.to_be_bytes());
    out.extend_from_slice(&identity.correlated_odom.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&identity.correlated_odom.pose_sha256);
    encode_map_geometry(&identity.committed_map, out);
    out.extend_from_slice(&identity.committed_map.payload_sha256);
    out.extend_from_slice(&identity.committed_map.boundary_sha256);
    encode_identity_tail(identity, out)?;
    out.extend_from_slice(&request.request_payload_sha256);
    Ok(())
}

fn encode_status_identity_tail(
    request: &NativePgoRequestEcho,
    out: &mut Vec<u8>,
) -> CodecResult<()> {
    let identity = &request.identity;
    encode_identity_until_cloud_count(identity, false, out)?;
    out.extend_from_slice(&identity.registered_cloud.envelope_sha256);
    out.extend_from_slice(&identity.registered_cloud.body_points_sha256);
    out.extend_from_slice(&identity.correlated_odom.source_sequence.to_be_bytes());
    out.extend_from_slice(&identity.correlated_odom.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&identity.correlated_odom.pose_sha256);
    encode_map_geometry(&identity.committed_map, out);
    out.extend_from_slice(&identity.committed_map.payload_sha256);
    out.extend_from_slice(&identity.committed_map.boundary_sha256);
    encode_identity_tail(identity, out)?;
    out.extend_from_slice(&request.request_payload_sha256);
    Ok(())
}

fn decode_request_body_from_cursor(cursor: &mut Cursor<'_>) -> CodecResult<NativePgoRequest> {
    let schema_version = cursor.read_i32()?;
    let (mut identity, point_count) = decode_identity_until_cloud_count(cursor, None)?;
    identity.registered_cloud.point_count = point_count;
    let registered_body_coordinate_count = cursor.read_bounded_count(
        MAX_NATIVE_PGO_REGISTERED_BODY_COORDINATE_COUNT,
        "registered_body_coordinate_count",
    )?;
    let registered_body_points_xyz = cursor.read_f64_vec(registered_body_coordinate_count)?;
    identity.registered_cloud.envelope_sha256 = cursor.read_array()?;
    identity.registered_cloud.body_points_sha256 = cursor.read_array()?;
    identity.correlated_odom.source_sequence = cursor.read_i64()?;
    identity.correlated_odom.source_timestamp_ns = cursor.read_i64()?;
    let frozen_odom_translation_m = cursor.read_f64_array()?;
    let frozen_odom_rotation_xyzw = cursor.read_f64_array()?;
    identity.correlated_odom.pose_sha256 = cursor.read_array()?;
    decode_map_geometry(cursor, &mut identity.committed_map)?;
    let committed_map_cell_count = cursor.read_bounded_count(
        MAX_NATIVE_PGO_COMMITTED_MAP_CELL_COUNT,
        "committed_map_cell_count",
    )?;
    let committed_map_occupancy = cursor.read_i8_vec(committed_map_cell_count)?;
    identity.committed_map.payload_sha256 = cursor.read_array()?;
    identity.committed_map.boundary_sha256 = cursor.read_array()?;
    decode_identity_tail(cursor, &mut identity)?;
    let request_payload_sha256 = cursor.read_array()?;
    Ok(NativePgoRequest {
        schema_version,
        identity,
        registered_body_coordinate_count,
        registered_body_points_xyz,
        frozen_odom_translation_m,
        frozen_odom_rotation_xyzw,
        committed_map_cell_count,
        committed_map_occupancy,
        request_payload_sha256,
    })
}

fn decode_result_body_from_cursor(cursor: &mut Cursor<'_>) -> CodecResult<NativePgoResult> {
    let schema_version = cursor.read_i32()?;
    let request = decode_request_echo(cursor, None)?;
    Ok(NativePgoResult {
        schema_version,
        request,
        result_disposition: cursor.read_i32()?,
        result_timestamp_ns: cursor.read_i64()?,
        transform_semantics: cursor.read_i32()?,
        translation_m: cursor.read_f64_array()?,
        rotation_xyzw: cursor.read_f64_array()?,
        fitness_score: cursor.read_f64()?,
        inlier_rmse_m: cursor.read_f64()?,
        loop_source_node_id: cursor.read_i64()?,
        loop_target_node_id: cursor.read_i64()?,
        loop_inlier_count: cursor.read_i64()?,
        loop_proof_sha256: cursor.read_array()?,
        optimized_graph_node_count: cursor.read_i64()?,
        optimized_graph_edge_count: cursor.read_i64()?,
        optimized_graph_loop_edge_count: cursor.read_i64()?,
        optimization_iterations: cursor.read_i32()?,
        optimization_converged: cursor.read_i8()?,
        optimization_initial_cost: cursor.read_f64()?,
        optimization_final_cost: cursor.read_f64()?,
        optimized_graph_sha256: cursor.read_array()?,
        trajectory: decode_trajectory(cursor)?,
        rejection_reason_code: cursor.read_i32()?,
        rejection_reason: cursor.read_string(MAX_REASON_BYTES, "rejection_reason")?,
        result_payload_sha256: cursor.read_array()?,
    })
}

fn decode_status_body_from_cursor(cursor: &mut Cursor<'_>) -> CodecResult<NativePgoStatus> {
    let schema_version = cursor.read_i32()?;
    let backend = cursor.read_i32()?;
    let runtime_generation = cursor.read_i64()?;
    let backend_generation = cursor.read_i64()?;
    let status_sequence = cursor.read_i64()?;
    let status_timestamp_ns = cursor.read_i64()?;
    let identity_present = cursor.read_i8()?;
    if !matches!(identity_present, 0 | 1) {
        return Err(decode_error(format!(
            "identity_present must be zero or one, got {identity_present}"
        )));
    }
    let parsed_request = decode_request_echo(
        cursor,
        Some((backend, runtime_generation, backend_generation)),
    )?;
    let request = if identity_present == 1 {
        Some(parsed_request)
    } else if request_echo_tail_is_canonical_zero(&parsed_request) {
        None
    } else {
        return Err(decode_error(
            "owner-scoped status must encode a canonical-zero request identity".into(),
        ));
    };
    Ok(NativePgoStatus {
        schema_version,
        backend,
        runtime_generation,
        backend_generation,
        status_sequence,
        status_timestamp_ns,
        request,
        event_kind: cursor.read_i32()?,
        health_state: cursor.read_i32()?,
        terminal_state: cursor.read_i32()?,
        cancel_state: cursor.read_i32()?,
        reason_code: cursor.read_i32()?,
        reason: cursor.read_string(MAX_REASON_BYTES, "reason")?,
        status_payload_sha256: cursor.read_array()?,
    })
}

fn decode_identity_until_cloud_count(
    cursor: &mut Cursor<'_>,
    owner_header: Option<(i32, i64, i64)>,
) -> CodecResult<(NativePgoRequestIdentity, i64)> {
    let (backend, runtime_generation, backend_generation) = match owner_header {
        Some(header) => header,
        None => (cursor.read_i32()?, cursor.read_i64()?, cursor.read_i64()?),
    };
    let request_sequence = cursor.read_i64()?;
    let mapping_session_id = cursor.read_string(MAX_IDENTITY_STRING_BYTES, "mapping_session_id")?;
    let mapping_session_generation = cursor.read_i64()?;
    let map_frame_generation = cursor.read_i64()?;
    let mapping_commit_id = NativePgoMappingCommitId {
        boot_nonce: cursor.read_array()?,
        sequence: cursor.read_i64()?,
    };
    let occupancy_revision = cursor.read_i64()?;
    let cloud_revision = cursor.read_i64()?;
    let committed_map_source_timestamp_ns = cursor.read_i64()?;
    let registered_cloud_source_sequence = cursor.read_i64()?;
    let registered_cloud_source_timestamp_ns = cursor.read_i64()?;
    let point_count = cursor.read_i64()?;
    Ok((
        NativePgoRequestIdentity {
            backend,
            runtime_generation,
            backend_generation,
            request_sequence,
            mapping_session_id,
            mapping_session_generation,
            map_frame_generation,
            mapping_commit_id,
            occupancy_revision,
            cloud_revision,
            registered_cloud: NativePgoRegisteredCloudProof {
                source_sequence: registered_cloud_source_sequence,
                source_timestamp_ns: registered_cloud_source_timestamp_ns,
                point_count,
                ..NativePgoRegisteredCloudProof::default()
            },
            correlated_odom: NativePgoCorrelatedOdomProof::default(),
            committed_map: NativePgoCommittedMapProof {
                source_timestamp_ns: committed_map_source_timestamp_ns,
                ..NativePgoCommittedMapProof::default()
            },
            source_timestamp_ns: 0,
            map_frame_id: String::new(),
            odom_frame_id: String::new(),
            body_frame_id: String::new(),
            points_frame_id: String::new(),
            correction_lease: NativePgoCorrectionLeaseId::default(),
        },
        point_count,
    ))
}

fn decode_request_echo(
    cursor: &mut Cursor<'_>,
    owner_header: Option<(i32, i64, i64)>,
) -> CodecResult<NativePgoRequestEcho> {
    let (mut identity, point_count) = decode_identity_until_cloud_count(cursor, owner_header)?;
    identity.registered_cloud.point_count = point_count;
    identity.registered_cloud.envelope_sha256 = cursor.read_array()?;
    identity.registered_cloud.body_points_sha256 = cursor.read_array()?;
    identity.correlated_odom.source_sequence = cursor.read_i64()?;
    identity.correlated_odom.source_timestamp_ns = cursor.read_i64()?;
    identity.correlated_odom.pose_sha256 = cursor.read_array()?;
    decode_map_geometry(cursor, &mut identity.committed_map)?;
    identity.committed_map.payload_sha256 = cursor.read_array()?;
    identity.committed_map.boundary_sha256 = cursor.read_array()?;
    decode_identity_tail(cursor, &mut identity)?;
    Ok(NativePgoRequestEcho {
        identity,
        request_payload_sha256: cursor.read_array()?,
    })
}

fn decode_map_geometry(
    cursor: &mut Cursor<'_>,
    proof: &mut NativePgoCommittedMapProof,
) -> CodecResult<()> {
    proof.width = cursor.read_i64()?;
    proof.height = cursor.read_i64()?;
    proof.resolution_m = cursor.read_f64()?;
    proof.origin_x_m = cursor.read_f64()?;
    proof.origin_y_m = cursor.read_f64()?;
    Ok(())
}

fn decode_identity_tail(
    cursor: &mut Cursor<'_>,
    identity: &mut NativePgoRequestIdentity,
) -> CodecResult<()> {
    identity.source_timestamp_ns = cursor.read_i64()?;
    identity.map_frame_id = cursor.read_string(MAX_IDENTITY_STRING_BYTES, "map_frame_id")?;
    identity.odom_frame_id = cursor.read_string(MAX_IDENTITY_STRING_BYTES, "odom_frame_id")?;
    identity.body_frame_id = cursor.read_string(MAX_IDENTITY_STRING_BYTES, "body_frame_id")?;
    identity.points_frame_id = cursor.read_string(MAX_IDENTITY_STRING_BYTES, "points_frame_id")?;
    identity.correction_lease.boot_nonce = cursor.read_array()?;
    identity.correction_lease.generation = cursor.read_i64()?;
    Ok(())
}

fn validate_request_integrity(message: &NativePgoRequest) -> Result<(), String> {
    validate_request_fields(message)?;
    let expected = compute_native_pgo_request_payload_sha256(message).map_err(codec_message)?;
    if message.request_payload_sha256 != expected {
        return Err("native PGO request payload hash mismatch".into());
    }
    Ok(())
}

fn validate_result_integrity(message: &NativePgoResult) -> Result<(), String> {
    validate_result_fields(message)?;
    let expected = compute_native_pgo_result_payload_sha256(message).map_err(codec_message)?;
    if message.result_payload_sha256 != expected {
        return Err("native PGO result payload hash mismatch".into());
    }
    Ok(())
}

fn validate_status_integrity(message: &NativePgoStatus) -> Result<(), String> {
    validate_status_fields(message)?;
    let expected = compute_native_pgo_status_payload_sha256(message).map_err(codec_message)?;
    if message.status_payload_sha256 != expected {
        return Err("native PGO status payload hash mismatch".into());
    }
    Ok(())
}

fn validate_request_fields(message: &NativePgoRequest) -> Result<(), String> {
    validate_schema_version(message.schema_version)?;
    validate_request_identity(&message.identity)?;
    let expected_coordinate_count = message
        .identity
        .registered_cloud
        .point_count
        .checked_mul(3)
        .and_then(|value| i32::try_from(value).ok())
        .ok_or_else(|| "registered body coordinate count overflow".to_string())?;
    if message.registered_body_coordinate_count != expected_coordinate_count
        || usize::try_from(message.registered_body_coordinate_count).ok()
            != Some(message.registered_body_points_xyz.len())
        || message.registered_body_coordinate_count
            > MAX_NATIVE_PGO_REGISTERED_BODY_COORDINATE_COUNT
    {
        return Err(
            "registered_body_coordinate_count must equal point_count*3 and the XYZ payload length"
                .into(),
        );
    }
    let body_points_sha256 =
        compute_native_pgo_body_points_sha256(&message.registered_body_points_xyz)
            .map_err(codec_message)?;
    if message.identity.registered_cloud.body_points_sha256 != body_points_sha256 {
        return Err("registered body points hash mismatch".into());
    }
    let odom_pose_sha256 = compute_native_pgo_odom_pose_sha256(
        &message.frozen_odom_translation_m,
        &message.frozen_odom_rotation_xyzw,
    )
    .map_err(codec_message)?;
    if message.identity.correlated_odom.pose_sha256 != odom_pose_sha256 {
        return Err("frozen odometry pose hash mismatch".into());
    }
    let expected_cells = message
        .identity
        .committed_map
        .width
        .checked_mul(message.identity.committed_map.height)
        .and_then(|value| i32::try_from(value).ok())
        .ok_or_else(|| "committed map cell count overflow".to_string())?;
    if message.committed_map_cell_count != expected_cells
        || usize::try_from(message.committed_map_cell_count).ok()
            != Some(message.committed_map_occupancy.len())
        || message.committed_map_cell_count > MAX_NATIVE_PGO_COMMITTED_MAP_CELL_COUNT
    {
        return Err(
            "committed_map_cell_count must equal width*height and the occupancy payload length"
                .into(),
        );
    }
    if message
        .committed_map_occupancy
        .iter()
        .any(|value| !(-1..=100).contains(value))
    {
        return Err("committed occupancy values must be in [-1,100]".into());
    }
    if message.identity.committed_map.payload_sha256
        != compute_native_pgo_map_payload_sha256(&message.committed_map_occupancy)
    {
        return Err("committed map payload hash mismatch".into());
    }
    let expected_boundary = compute_native_pgo_map_boundary_sha256(
        &message.identity.map_frame_id,
        &message.identity.committed_map,
    )
    .map_err(codec_message)?;
    if message.identity.committed_map.boundary_sha256 != expected_boundary {
        return Err("committed map boundary hash mismatch".into());
    }
    Ok(())
}

fn validate_request_identity(identity: &NativePgoRequestIdentity) -> Result<(), String> {
    if identity.backend != NATIVE_PGO_BACKEND {
        return Err(format!(
            "unsupported native PGO backend={}",
            identity.backend
        ));
    }
    for (name, value) in [
        ("runtime_generation", identity.runtime_generation),
        ("backend_generation", identity.backend_generation),
        ("request_sequence", identity.request_sequence),
        (
            "mapping_session_generation",
            identity.mapping_session_generation,
        ),
        ("map_frame_generation", identity.map_frame_generation),
        (
            "mapping_commit_sequence",
            identity.mapping_commit_id.sequence,
        ),
        ("occupancy_revision", identity.occupancy_revision),
        ("cloud_revision", identity.cloud_revision),
        (
            "registered_cloud_source_sequence",
            identity.registered_cloud.source_sequence,
        ),
        (
            "registered_body_point_count",
            identity.registered_cloud.point_count,
        ),
        (
            "correlated_odom_source_sequence",
            identity.correlated_odom.source_sequence,
        ),
        (
            "correction_lease_generation",
            identity.correction_lease.generation,
        ),
    ] {
        if value <= 0 {
            return Err(format!("{name} must be positive"));
        }
    }
    validate_identity_string("mapping_session_id", &identity.mapping_session_id)?;
    if !digest_is_present(&identity.mapping_commit_id.boot_nonce) {
        return Err("mapping commit boot nonce must be present".into());
    }
    if identity.registered_cloud.point_count > MAX_NATIVE_PGO_REGISTERED_BODY_POINT_COUNT {
        return Err("registered body point count exceeds the V1 limit".into());
    }
    for (name, value) in [
        (
            "committed_map_source_timestamp_ns",
            identity.committed_map.source_timestamp_ns,
        ),
        (
            "registered_cloud_source_timestamp_ns",
            identity.registered_cloud.source_timestamp_ns,
        ),
        (
            "correlated_odom_source_timestamp_ns",
            identity.correlated_odom.source_timestamp_ns,
        ),
        ("source_timestamp_ns", identity.source_timestamp_ns),
    ] {
        if value < 0 {
            return Err(format!("{name} must be non-negative"));
        }
    }
    if identity.committed_map.source_timestamp_ns != identity.source_timestamp_ns
        || identity.registered_cloud.source_timestamp_ns != identity.source_timestamp_ns
    {
        return Err(
            "committed map, registered cloud, and request source timestamps must agree".into(),
        );
    }
    if identity.correlated_odom.source_timestamp_ns > identity.source_timestamp_ns {
        return Err("correlated odometry timestamp cannot follow the source cloud".into());
    }
    for (name, digest) in [
        (
            "registered_cloud_envelope_sha256",
            &identity.registered_cloud.envelope_sha256,
        ),
        (
            "registered_body_points_sha256",
            &identity.registered_cloud.body_points_sha256,
        ),
        (
            "frozen_odom_pose_sha256",
            &identity.correlated_odom.pose_sha256,
        ),
        (
            "committed_map_payload_sha256",
            &identity.committed_map.payload_sha256,
        ),
        (
            "committed_map_boundary_sha256",
            &identity.committed_map.boundary_sha256,
        ),
    ] {
        if !digest_is_present(digest) {
            return Err(format!("{name} must be present"));
        }
    }
    let map = &identity.committed_map;
    if map.width <= 0
        || map.height <= 0
        || map.width > i64::from(u32::MAX)
        || map.height > i64::from(u32::MAX)
        || map
            .width
            .checked_mul(map.height)
            .is_none_or(|cells| cells > i64::from(MAX_NATIVE_PGO_COMMITTED_MAP_CELL_COUNT))
        || !map.resolution_m.is_finite()
        || map.resolution_m <= 0.0
        || !map.origin_x_m.is_finite()
        || !map.origin_y_m.is_finite()
    {
        return Err("committed map geometry is invalid or exceeds the V1 limit".into());
    }
    for (name, frame) in [
        ("map_frame_id", identity.map_frame_id.as_str()),
        ("odom_frame_id", identity.odom_frame_id.as_str()),
        ("body_frame_id", identity.body_frame_id.as_str()),
        ("points_frame_id", identity.points_frame_id.as_str()),
    ] {
        validate_frame_id(name, frame)?;
    }
    if identity.map_frame_id == identity.odom_frame_id
        || identity.map_frame_id == identity.body_frame_id
        || identity.odom_frame_id == identity.body_frame_id
    {
        return Err("map, odom, and body frames must be distinct".into());
    }
    if identity.points_frame_id != identity.body_frame_id {
        return Err("points_frame_id must equal body_frame_id in V1".into());
    }
    if !digest_is_present(&identity.correction_lease.boot_nonce) {
        return Err("correction lease boot nonce must be present".into());
    }
    Ok(())
}

fn validate_request_echo(request: &NativePgoRequestEcho) -> Result<(), String> {
    validate_request_identity(&request.identity)?;
    if !digest_is_present(&request.request_payload_sha256) {
        return Err("request payload hash must be present".into());
    }
    Ok(())
}

fn validate_result_fields(message: &NativePgoResult) -> Result<(), String> {
    validate_schema_version(message.schema_version)?;
    validate_request_echo(&message.request)?;
    if message.result_timestamp_ns < 0 {
        return Err("result_timestamp_ns must be non-negative".into());
    }
    match message.result_disposition {
        NATIVE_PGO_DISPOSITION_CORRECTION => validate_correction_result(message),
        NATIVE_PGO_DISPOSITION_NO_CANDIDATE => {
            validate_zero_correction_fields(message)?;
            if message.rejection_reason_code != NATIVE_PGO_REJECTION_NONE
                || !message.rejection_reason.is_empty()
            {
                return Err("NO_CANDIDATE must not carry rejection metadata".into());
            }
            Ok(())
        }
        NATIVE_PGO_DISPOSITION_REJECTED => {
            validate_zero_correction_fields(message)?;
            if !matches!(
                message.rejection_reason_code,
                NATIVE_PGO_REJECTION_INSUFFICIENT_GRAPH
                    | NATIVE_PGO_REJECTION_INVALID_GRAPH
                    | NATIVE_PGO_REJECTION_NUMERIC_FAILURE
                    | NATIVE_PGO_REJECTION_RESOURCE_LIMIT
                    | NATIVE_PGO_REJECTION_CANCELLED
            ) || !valid_reason(&message.rejection_reason)
            {
                return Err("REJECTED requires a typed rejection code and bounded detail".into());
            }
            Ok(())
        }
        other => Err(format!("unknown native PGO result_disposition={other}")),
    }
}

fn validate_correction_result(message: &NativePgoResult) -> Result<(), String> {
    validate_optimized_keyframes(&message.trajectory)?;
    if message.trajectory.len() as i64 != message.optimized_graph_node_count
        || message
            .trajectory
            .last()
            .map(|frame| frame.source_timestamp_ns)
            != Some(message.request.identity.source_timestamp_ns)
    {
        return Err("correction requires the complete source-bound trajectory".into());
    }
    if message.transform_semantics != NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3 {
        return Err("CORRECTION requires absolute-left full-SE(3) semantics".into());
    }
    validate_transform(&message.translation_m, &message.rotation_xyzw, "correction")?;
    validate_loop_metadata(message)?;
    validate_graph_metadata(message)?;
    if message.rejection_reason_code != NATIVE_PGO_REJECTION_NONE
        || !message.rejection_reason.is_empty()
    {
        return Err("CORRECTION must not carry rejection metadata".into());
    }
    let loop_proof = compute_native_pgo_loop_proof_sha256(message).map_err(codec_message)?;
    if message.loop_proof_sha256 != loop_proof {
        return Err("loop proof hash mismatch".into());
    }
    let graph_proof = compute_native_pgo_optimized_graph_sha256(message).map_err(codec_message)?;
    if message.optimized_graph_sha256 != graph_proof {
        return Err("optimized graph hash mismatch".into());
    }
    Ok(())
}

fn validate_loop_metadata(message: &NativePgoResult) -> Result<(), String> {
    if !message.fitness_score.is_finite()
        || message.fitness_score < 0.0
        || !message.inlier_rmse_m.is_finite()
        || message.inlier_rmse_m < 0.0
        || message.loop_source_node_id < 0
        || message.loop_target_node_id < 0
        || message.loop_source_node_id == message.loop_target_node_id
        || message.loop_inlier_count <= 0
        || message.loop_inlier_count > MAX_NATIVE_PGO_LOOP_INLIERS
        || message.loop_source_node_id >= message.optimized_graph_node_count
        || message.loop_target_node_id >= message.optimized_graph_node_count
    {
        return Err("correction loop metadata is invalid".into());
    }
    Ok(())
}

fn validate_graph_metadata(message: &NativePgoResult) -> Result<(), String> {
    let expected_edges =
        message
            .optimized_graph_node_count
            .checked_sub(1)
            .and_then(|odometry_edges| {
                odometry_edges.checked_add(message.optimized_graph_loop_edge_count)
            });
    if message.optimized_graph_node_count < 3
        || message.optimized_graph_node_count > MAX_NATIVE_PGO_GRAPH_NODES
        || message.optimized_graph_edge_count < message.optimized_graph_node_count
        || message.optimized_graph_edge_count > MAX_NATIVE_PGO_GRAPH_EDGES
        || message.optimized_graph_loop_edge_count <= 0
        || message.optimized_graph_loop_edge_count > MAX_NATIVE_PGO_GRAPH_LOOP_EDGES
        || message.optimized_graph_loop_edge_count > message.optimized_graph_node_count - 2
        || message.optimized_graph_loop_edge_count > message.optimized_graph_edge_count
        || expected_edges != Some(message.optimized_graph_edge_count)
        || message.optimization_iterations <= 0
        || message.optimization_iterations > MAX_NATIVE_PGO_OPTIMIZATION_ITERATIONS
        || message.optimization_converged != 1
        || !message.optimization_initial_cost.is_finite()
        || message.optimization_initial_cost < 0.0
        || !message.optimization_final_cost.is_finite()
        || message.optimization_final_cost < 0.0
        || message.optimization_final_cost > message.optimization_initial_cost
    {
        return Err("optimized graph metadata is invalid".into());
    }
    Ok(())
}

fn validate_zero_correction_fields(message: &NativePgoResult) -> Result<(), String> {
    if !message.trajectory.is_empty() {
        return Err("non-correction must not carry trajectory".into());
    }
    let f64_zero = message
        .translation_m
        .iter()
        .chain(message.rotation_xyzw.iter())
        .chain([
            &message.fitness_score,
            &message.inlier_rmse_m,
            &message.optimization_initial_cost,
            &message.optimization_final_cost,
        ])
        .all(|value| value.to_bits() == 0);
    if message.transform_semantics != 0
        || !f64_zero
        || message.loop_source_node_id != 0
        || message.loop_target_node_id != 0
        || message.loop_inlier_count != 0
        || digest_is_present(&message.loop_proof_sha256)
        || message.optimized_graph_node_count != 0
        || message.optimized_graph_edge_count != 0
        || message.optimized_graph_loop_edge_count != 0
        || message.optimization_iterations != 0
        || message.optimization_converged != 0
        || digest_is_present(&message.optimized_graph_sha256)
    {
        return Err(
            "NO_CANDIDATE/REJECTED must canonicalize every correction-only field to zero".into(),
        );
    }
    Ok(())
}

fn validate_status_fields(message: &NativePgoStatus) -> Result<(), String> {
    validate_schema_version(message.schema_version)?;
    if message.backend != NATIVE_PGO_BACKEND {
        return Err(format!(
            "unsupported native PGO backend={}",
            message.backend
        ));
    }
    if message.runtime_generation <= 0
        || message.backend_generation <= 0
        || message.status_sequence <= 0
        || message.status_timestamp_ns < 0
    {
        return Err("status runtime/backend/status identity is invalid".into());
    }
    if let Some(request) = &message.request {
        validate_request_echo(request)?;
        if request.identity.backend != message.backend
            || request.identity.runtime_generation != message.runtime_generation
            || request.identity.backend_generation != message.backend_generation
        {
            return Err("status owner identity does not match request identity".into());
        }
    }
    if message.reason.len() > MAX_REASON_BYTES || message.reason.contains('\0') {
        return Err("status reason exceeds its bound or contains NUL".into());
    }
    let reason_present = valid_reason(&message.reason);
    let owner_scoped = message.request.is_none();
    let request_scoped = message.request.is_some();
    let valid = match message.event_kind {
        NATIVE_PGO_EVENT_STARTING => {
            owner_scoped
                && message.health_state == NATIVE_PGO_HEALTH_UNKNOWN
                && no_terminal_or_cancel(message)
                && message.reason_code == NATIVE_PGO_REASON_NONE
                && message.reason.is_empty()
        }
        NATIVE_PGO_EVENT_READY => {
            owner_scoped
                && message.health_state == NATIVE_PGO_HEALTH_HEALTHY
                && no_terminal_or_cancel(message)
                && message.reason_code == NATIVE_PGO_REASON_NONE
                && message.reason.is_empty()
        }
        NATIVE_PGO_EVENT_HEALTH => {
            owner_scoped
                && no_terminal_or_cancel(message)
                && match message.health_state {
                    NATIVE_PGO_HEALTH_HEALTHY => {
                        message.reason_code == NATIVE_PGO_REASON_NONE && message.reason.is_empty()
                    }
                    NATIVE_PGO_HEALTH_DEGRADED | NATIVE_PGO_HEALTH_UNHEALTHY => {
                        matches!(
                            message.reason_code,
                            NATIVE_PGO_REASON_TRANSPORT_ERROR
                                | NATIVE_PGO_REASON_CHILD_CRASHED
                                | NATIVE_PGO_REASON_OPTIMIZATION_FAILED
                        ) && reason_present
                    }
                    _ => false,
                }
        }
        NATIVE_PGO_EVENT_REQUEST_ACCEPTED | NATIVE_PGO_EVENT_RUNNING => {
            request_scoped
                && message.health_state == NATIVE_PGO_HEALTH_HEALTHY
                && no_terminal_or_cancel(message)
                && message.reason_code == NATIVE_PGO_REASON_NONE
                && message.reason.is_empty()
        }
        NATIVE_PGO_EVENT_TERMINAL => validate_terminal_status(message, reason_present),
        NATIVE_PGO_EVENT_CANCEL_REQUESTED => {
            request_scoped
                && message.health_state == NATIVE_PGO_HEALTH_UNKNOWN
                && message.terminal_state == NATIVE_PGO_TERMINAL_NONE
                && message.cancel_state == NATIVE_PGO_CANCEL_REQUESTED
                && message.reason_code == NATIVE_PGO_REASON_CANCEL_REQUESTED
                && reason_present
        }
        NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED => {
            request_scoped
                && message.health_state == NATIVE_PGO_HEALTH_HEALTHY
                && ((message.terminal_state == NATIVE_PGO_TERMINAL_CANCELLED
                    && message.cancel_state == NATIVE_PGO_CANCEL_ACKNOWLEDGED
                    && message.reason_code == NATIVE_PGO_REASON_CANCELLED)
                    || (message.terminal_state == NATIVE_PGO_TERMINAL_NONE
                        && message.cancel_state == NATIVE_PGO_CANCEL_TOO_LATE
                        && message.reason_code == NATIVE_PGO_REASON_CANCEL_TOO_LATE))
                && reason_present
        }
        NATIVE_PGO_EVENT_PROTOCOL_ERROR => {
            matches!(
                message.health_state,
                NATIVE_PGO_HEALTH_DEGRADED | NATIVE_PGO_HEALTH_UNHEALTHY
            ) && no_terminal_or_cancel(message)
                && matches!(
                    message.reason_code,
                    NATIVE_PGO_REASON_PROTOCOL_DECODE_FAILED
                        | NATIVE_PGO_REASON_PROTOCOL_IDENTITY_MISMATCH
                        | NATIVE_PGO_REASON_PROTOCOL_HASH_MISMATCH
                        | NATIVE_PGO_REASON_INVALID_REQUEST
                )
                && reason_present
        }
        NATIVE_PGO_EVENT_STOPPING => {
            owner_scoped
                && message.health_state == NATIVE_PGO_HEALTH_UNKNOWN
                && no_terminal_or_cancel(message)
                && message.reason_code == NATIVE_PGO_REASON_SHUTDOWN_REQUESTED
                && reason_present
        }
        NATIVE_PGO_EVENT_STOPPED => {
            owner_scoped
                && message.health_state == NATIVE_PGO_HEALTH_UNKNOWN
                && no_terminal_or_cancel(message)
                && matches!(
                    message.reason_code,
                    NATIVE_PGO_REASON_SHUTDOWN_REQUESTED | NATIVE_PGO_REASON_CHILD_EXITED
                )
                && reason_present
        }
        _ => false,
    };
    if !valid {
        return Err(format!(
            "invalid native PGO status matrix: event={} health={} terminal={} cancel={} reason={} identity_present={}",
            message.event_kind,
            message.health_state,
            message.terminal_state,
            message.cancel_state,
            message.reason_code,
            i32::from(message.request.is_some())
        ));
    }
    Ok(())
}

fn validate_terminal_status(message: &NativePgoStatus, reason_present: bool) -> bool {
    if message.request.is_none() || message.health_state != NATIVE_PGO_HEALTH_HEALTHY {
        return false;
    }
    match message.terminal_state {
        NATIVE_PGO_TERMINAL_SUCCEEDED => {
            message.cancel_state == NATIVE_PGO_CANCEL_NONE
                && message.reason_code == NATIVE_PGO_REASON_NONE
                && message.reason.is_empty()
        }
        NATIVE_PGO_TERMINAL_FAILED => {
            message.cancel_state == NATIVE_PGO_CANCEL_NONE
                && matches!(
                    message.reason_code,
                    NATIVE_PGO_REASON_OPTIMIZATION_FAILED
                        | NATIVE_PGO_REASON_TRANSPORT_ERROR
                        | NATIVE_PGO_REASON_CHILD_CRASHED
                )
                && reason_present
        }
        NATIVE_PGO_TERMINAL_REJECTED => {
            message.cancel_state == NATIVE_PGO_CANCEL_NONE
                && matches!(
                    message.reason_code,
                    NATIVE_PGO_REASON_INVALID_REQUEST | NATIVE_PGO_REASON_OPTIMIZATION_FAILED
                )
                && reason_present
        }
        _ => false,
    }
}

fn no_terminal_or_cancel(message: &NativePgoStatus) -> bool {
    message.terminal_state == NATIVE_PGO_TERMINAL_NONE
        && message.cancel_state == NATIVE_PGO_CANCEL_NONE
}

fn validate_schema_version(schema_version: i32) -> Result<(), String> {
    if schema_version == NATIVE_PGO_SCHEMA_VERSION_V2 {
        Ok(())
    } else {
        Err(format!(
            "unsupported native PGO schema_version={schema_version}"
        ))
    }
}

fn validate_transform(
    translation_m: &[f64; 3],
    rotation_xyzw: &[f64; 4],
    name: &str,
) -> Result<(), String> {
    if translation_m
        .iter()
        .chain(rotation_xyzw.iter())
        .any(|value| !value.is_finite())
    {
        return Err(format!("{name} transform must be finite"));
    }
    if translation_m
        .iter()
        .any(|value| value.abs() > MAX_TRANSLATION_ABS_METERS)
    {
        return Err(format!("{name} translation exceeds the V1 bound"));
    }
    let norm_squared = rotation_xyzw.iter().map(|value| value * value).sum::<f64>();
    if (norm_squared - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
        return Err(format!("{name} quaternion must be unit length"));
    }
    Ok(())
}

fn validate_identity_string(name: &str, value: &str) -> Result<(), String> {
    if value.is_empty()
        || value.len() > MAX_IDENTITY_STRING_BYTES
        || value.trim() != value
        || value.contains('\0')
    {
        Err(format!(
            "{name} must be nonempty, trimmed, NUL-free, and bounded"
        ))
    } else {
        Ok(())
    }
}

fn validate_frame_id(name: &str, value: &str) -> Result<(), String> {
    let valid = !value.is_empty()
        && value.len() <= MAX_IDENTITY_STRING_BYTES
        && value.trim() == value
        && value.split('/').all(|segment| {
            let mut characters = segment.chars();
            characters
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && characters.all(|character| character.is_ascii_alphanumeric() || character == '_')
        });
    if valid {
        Ok(())
    } else {
        Err(format!(
            "{name} must use bounded slash-separated ASCII frame tokens"
        ))
    }
}

fn valid_reason(reason: &str) -> bool {
    !reason.is_empty()
        && reason.len() <= MAX_REASON_BYTES
        && reason.trim() == reason
        && !reason.contains('\0')
}

fn request_echo_tail_is_canonical_zero(request: &NativePgoRequestEcho) -> bool {
    let identity = &request.identity;
    identity.request_sequence == 0
        && identity.mapping_session_id.is_empty()
        && identity.mapping_session_generation == 0
        && identity.map_frame_generation == 0
        && !digest_is_present(&identity.mapping_commit_id.boot_nonce)
        && identity.mapping_commit_id.sequence == 0
        && identity.occupancy_revision == 0
        && identity.cloud_revision == 0
        && identity.registered_cloud == NativePgoRegisteredCloudProof::default()
        && identity.correlated_odom == NativePgoCorrelatedOdomProof::default()
        && identity.committed_map.source_timestamp_ns == 0
        && identity.committed_map.width == 0
        && identity.committed_map.height == 0
        && identity.committed_map.resolution_m.to_bits() == 0
        && identity.committed_map.origin_x_m.to_bits() == 0
        && identity.committed_map.origin_y_m.to_bits() == 0
        && !digest_is_present(&identity.committed_map.payload_sha256)
        && !digest_is_present(&identity.committed_map.boundary_sha256)
        && identity.source_timestamp_ns == 0
        && identity.map_frame_id.is_empty()
        && identity.odom_frame_id.is_empty()
        && identity.body_frame_id.is_empty()
        && identity.points_frame_id.is_empty()
        && !digest_is_present(&identity.correction_lease.boot_nonce)
        && identity.correction_lease.generation == 0
        && !digest_is_present(&request.request_payload_sha256)
}

fn digest_is_present<const N: usize>(digest: &[u8; N]) -> bool {
    digest.iter().any(|byte| *byte != 0)
}

fn hash_body_prefix(domain: &[u8], body_prefix: &[u8]) -> [u8; 32] {
    let mut canonical = Vec::with_capacity(domain.len() + body_prefix.len());
    canonical.extend_from_slice(domain);
    canonical.extend_from_slice(body_prefix);
    sha256_bytes(&canonical)
}

fn encode_string(value: &str, out: &mut Vec<u8>) -> CodecResult<()> {
    let length = value
        .len()
        .checked_add(1)
        .and_then(|value| i32::try_from(value).ok())
        .ok_or_else(|| encode_error("LCM string length overflow".into()))?;
    out.extend_from_slice(&length.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    out.push(0);
    Ok(())
}

fn encode_wire(fingerprint: [u8; 8], body: Vec<u8>) -> CodecResult<Vec<u8>> {
    let capacity = 8_usize
        .checked_add(body.len())
        .ok_or_else(|| encode_error("native PGO wire length overflow".into()))?;
    let mut wire = Vec::with_capacity(capacity);
    wire.extend_from_slice(&fingerprint);
    wire.extend_from_slice(&body);
    Ok(wire)
}

fn decode_wire<T>(
    wire: &[u8],
    fingerprint: [u8; 8],
    message_type: &str,
    decode_body: fn(&[u8]) -> CodecResult<T>,
) -> CodecResult<T> {
    if wire.len() < 8 {
        return Err(CodecError::Incomplete {
            need: 8,
            got: wire.len(),
        });
    }
    if wire[..8] != fingerprint {
        return Err(decode_error(format!(
            "fingerprint mismatch for {message_type}"
        )));
    }
    decode_body(&wire[8..])
}

fn encode_error(message: String) -> CodecError {
    CodecError::LcmEncode(message)
}

fn decode_error(message: String) -> CodecError {
    CodecError::LcmDecode(message)
}

fn codec_message(error: CodecError) -> String {
    match error {
        CodecError::LcmEncode(message) | CodecError::LcmDecode(message) => message,
        other => other.to_string(),
    }
}

struct Cursor<'a> {
    input: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(input: &'a [u8]) -> Self {
        Self { input, offset: 0 }
    }

    fn read_exact(&mut self, length: usize) -> CodecResult<&'a [u8]> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(CodecError::Incomplete {
                need: usize::MAX,
                got: self.input.len(),
            })?;
        if end > self.input.len() {
            return Err(CodecError::Incomplete {
                need: end,
                got: self.input.len(),
            });
        }
        let bytes = &self.input[self.offset..end];
        self.offset = end;
        Ok(bytes)
    }

    fn read_i8(&mut self) -> CodecResult<i8> {
        Ok(self.read_exact(1)?[0] as i8)
    }

    fn read_i32(&mut self) -> CodecResult<i32> {
        Ok(i32::from_be_bytes(
            self.read_exact(4)?.try_into().expect("four bytes"),
        ))
    }

    fn read_i64(&mut self) -> CodecResult<i64> {
        Ok(i64::from_be_bytes(
            self.read_exact(8)?.try_into().expect("eight bytes"),
        ))
    }

    fn read_f64(&mut self) -> CodecResult<f64> {
        Ok(f64::from_be_bytes(
            self.read_exact(8)?.try_into().expect("eight bytes"),
        ))
    }

    fn read_array<const N: usize>(&mut self) -> CodecResult<[u8; N]> {
        Ok(self.read_exact(N)?.try_into().expect("fixed byte array"))
    }

    fn read_f64_array<const N: usize>(&mut self) -> CodecResult<[f64; N]> {
        let mut values = [0.0; N];
        for value in &mut values {
            *value = self.read_f64()?;
        }
        Ok(values)
    }

    fn read_bounded_count(&mut self, maximum: i32, name: &str) -> CodecResult<i32> {
        let count = self.read_i32()?;
        if !(0..=maximum).contains(&count) {
            return Err(decode_error(format!(
                "{name} must be in [0,{maximum}], got {count}"
            )));
        }
        Ok(count)
    }

    fn read_f64_vec(&mut self, count: i32) -> CodecResult<Vec<f64>> {
        let count = usize::try_from(count).expect("bounded count is nonnegative");
        let mut values = Vec::with_capacity(count);
        for _ in 0..count {
            values.push(self.read_f64()?);
        }
        Ok(values)
    }

    fn read_i8_vec(&mut self, count: i32) -> CodecResult<Vec<i8>> {
        let count = usize::try_from(count).expect("bounded count is nonnegative");
        Ok(self
            .read_exact(count)?
            .iter()
            .map(|byte| *byte as i8)
            .collect())
    }

    fn read_string(&mut self, maximum: usize, name: &str) -> CodecResult<String> {
        let length = self.read_i32()?;
        if length <= 0 {
            return Err(decode_error(format!(
                "{name} LCM string length must include a trailing NUL"
            )));
        }
        let length = usize::try_from(length).expect("positive i32 fits usize");
        if length > maximum.saturating_add(1) {
            return Err(decode_error(format!("{name} exceeds its wire bound")));
        }
        let bytes = self.read_exact(length)?;
        if bytes.last() != Some(&0) || bytes[..length - 1].contains(&0) {
            return Err(decode_error(format!(
                "{name} must have exactly one trailing NUL"
            )));
        }
        String::from_utf8(bytes[..length - 1].to_vec())
            .map_err(|_| decode_error(format!("{name} is not valid UTF-8")))
    }

    fn finish(&self, message_type: &str) -> CodecResult<()> {
        if self.offset == self.input.len() {
            Ok(())
        } else {
            Err(decode_error(format!(
                "trailing bytes after {message_type}: {}",
                self.input.len() - self.offset
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::process::{Command, Output};
    use std::sync::atomic::{AtomicU64, Ordering};

    fn request() -> NativePgoRequest {
        let identity = NativePgoRequestIdentity {
            backend: NATIVE_PGO_BACKEND,
            runtime_generation: 7,
            backend_generation: 3,
            request_sequence: 11,
            mapping_session_id: "session-a".into(),
            mapping_session_generation: 5,
            map_frame_generation: 9,
            mapping_commit_id: NativePgoMappingCommitId {
                boot_nonce: [0x10; 16],
                sequence: 13,
            },
            occupancy_revision: 17,
            cloud_revision: 19,
            registered_cloud: NativePgoRegisteredCloudProof {
                source_sequence: 23,
                source_timestamp_ns: 42,
                point_count: 2,
                envelope_sha256: [0x31; 32],
                ..NativePgoRegisteredCloudProof::default()
            },
            correlated_odom: NativePgoCorrelatedOdomProof {
                source_sequence: 21,
                source_timestamp_ns: 40,
                ..NativePgoCorrelatedOdomProof::default()
            },
            committed_map: NativePgoCommittedMapProof {
                source_timestamp_ns: 42,
                width: 2,
                height: 2,
                resolution_m: 0.5,
                origin_x_m: 1.0,
                origin_y_m: 2.0,
                ..NativePgoCommittedMapProof::default()
            },
            source_timestamp_ns: 42,
            map_frame_id: "map".into(),
            odom_frame_id: "odom".into(),
            body_frame_id: "base_link".into(),
            points_frame_id: "base_link".into(),
            correction_lease: NativePgoCorrectionLeaseId {
                boot_nonce: [0x20; 16],
                generation: 29,
            },
        };
        seal_native_pgo_request(NativePgoRequest {
            schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
            identity,
            registered_body_coordinate_count: 6,
            registered_body_points_xyz: vec![1.25, 2.5, 3.75, -1.0, -2.0, -3.0],
            frozen_odom_translation_m: [4.0, 5.0, 6.0],
            frozen_odom_rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            committed_map_cell_count: 4,
            committed_map_occupancy: vec![-1, 0, 50, 100],
            request_payload_sha256: [0; 32],
        })
        .unwrap()
    }

    fn no_candidate_result(request: &NativePgoRequest) -> NativePgoResult {
        seal_native_pgo_result(NativePgoResult {
            schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
            request: request.echo(),
            result_disposition: NATIVE_PGO_DISPOSITION_NO_CANDIDATE,
            result_timestamp_ns: 43,
            rejection_reason_code: NATIVE_PGO_REJECTION_NONE,
            ..NativePgoResult::default()
        })
        .unwrap()
    }

    fn correction_result(request: &NativePgoRequest) -> NativePgoResult {
        seal_native_pgo_result(NativePgoResult {
            schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
            request: request.echo(),
            result_disposition: NATIVE_PGO_DISPOSITION_CORRECTION,
            result_timestamp_ns: 44,
            transform_semantics: NATIVE_PGO_TRANSFORM_ABSOLUTE_LEFT_CORRECTION_SE3,
            translation_m: [0.1, -0.2, 0.3],
            rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            fitness_score: 0.25,
            inlier_rmse_m: 0.125,
            loop_source_node_id: 0,
            loop_target_node_id: 1,
            loop_inlier_count: 20,
            trajectory: (0..3)
                .map(|i| OptimizedKeyframe {
                    source_timestamp_ns: request.identity.source_timestamp_ns - 2 + i,
                    translation_m: [i as f64, 0.0, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                })
                .collect(),
            optimized_graph_node_count: 3,
            optimized_graph_edge_count: 3,
            optimized_graph_loop_edge_count: 1,
            optimization_iterations: 3,
            optimization_converged: 1,
            optimization_initial_cost: 10.0,
            optimization_final_cost: 2.0,
            rejection_reason_code: NATIVE_PGO_REJECTION_NONE,
            ..NativePgoResult::default()
        })
        .unwrap()
    }

    fn cancelled_result(request: &NativePgoRequest) -> NativePgoResult {
        seal_native_pgo_result(NativePgoResult {
            schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
            request: request.echo(),
            result_disposition: NATIVE_PGO_DISPOSITION_REJECTED,
            result_timestamp_ns: 45,
            rejection_reason_code: NATIVE_PGO_REJECTION_CANCELLED,
            rejection_reason: "cancelled".into(),
            ..NativePgoResult::default()
        })
        .unwrap()
    }

    fn ready_status() -> NativePgoStatus {
        seal_native_pgo_status(NativePgoStatus {
            schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
            backend: NATIVE_PGO_BACKEND,
            runtime_generation: 7,
            backend_generation: 3,
            status_sequence: 1,
            status_timestamp_ns: 0,
            event_kind: NATIVE_PGO_EVENT_READY,
            health_state: NATIVE_PGO_HEALTH_HEALTHY,
            terminal_state: NATIVE_PGO_TERMINAL_NONE,
            cancel_state: NATIVE_PGO_CANCEL_NONE,
            reason_code: NATIVE_PGO_REASON_NONE,
            ..NativePgoStatus::default()
        })
        .unwrap()
    }

    fn cancelled_status(request: &NativePgoRequest) -> NativePgoStatus {
        seal_native_pgo_status(NativePgoStatus {
            schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
            backend: NATIVE_PGO_BACKEND,
            runtime_generation: 7,
            backend_generation: 3,
            status_sequence: 5,
            status_timestamp_ns: 46,
            request: Some(request.echo()),
            event_kind: NATIVE_PGO_EVENT_CANCEL_ACKNOWLEDGED,
            health_state: NATIVE_PGO_HEALTH_HEALTHY,
            terminal_state: NATIVE_PGO_TERMINAL_CANCELLED,
            cancel_state: NATIVE_PGO_CANCEL_ACKNOWLEDGED,
            reason_code: NATIVE_PGO_REASON_CANCELLED,
            reason: "cancelled".into(),
            ..NativePgoStatus::default()
        })
        .unwrap()
    }

    #[test]
    fn fresh_lcm_gen_fingerprints_are_locked() {
        assert_eq!(
            crate::hash::fingerprint("nuwax_msgs.NativePgoRequest", 0).unwrap(),
            NATIVE_PGO_REQUEST_FINGERPRINT_V2
        );
        assert_eq!(
            crate::hash::fingerprint("nuwax_msgs.NativePgoResult", 0).unwrap(),
            NATIVE_PGO_RESULT_FINGERPRINT_V2
        );
        assert_eq!(
            crate::hash::fingerprint("nuwax_msgs.NativePgoStatus", 0).unwrap(),
            NATIVE_PGO_STATUS_FINGERPRINT_V2
        );
    }

    #[test]
    fn fresh_lcm_gen_c_and_python_match_constants_and_exact_wires() {
        let request_wire = encode_native_pgo_request_wire(&request()).unwrap();
        let request = request();
        let result_wire = encode_native_pgo_result_wire(&correction_result(&request)).unwrap();
        let status_wire = encode_native_pgo_status_wire(&cancelled_status(&request)).unwrap();
        let temp = FreshCodegenDir::new();
        let c_dir = temp.path().join("c");
        let python_dir = temp.path().join("python");
        fs::create_dir(&c_dir).unwrap();
        fs::create_dir(&python_dir).unwrap();
        let schemas = native_pgo_schema_paths();

        let mut version = Command::new("lcm-gen");
        version.arg("--version");
        let version = run_command(version, "lcm-gen --version");
        assert_eq!(
            String::from_utf8_lossy(&version.stdout).trim(),
            "lcm-gen 1.5.2"
        );

        let mut generate_c = Command::new("lcm-gen");
        generate_c
            .arg("-c")
            .arg("--c-cpath")
            .arg(&c_dir)
            .arg("--c-hpath")
            .arg(&c_dir)
            .args(&schemas);
        run_command(generate_c, "fresh native PGO C codegen");

        let mut generate_python = Command::new("lcm-gen");
        generate_python
            .arg("-p")
            .arg("--ppath")
            .arg(&python_dir)
            .args(&schemas);
        run_command(generate_python, "fresh native PGO Python codegen");

        let helper_source = temp.path().join("native_pgo_c_parity.c");
        let helper_binary = temp.path().join("native_pgo_c_parity");
        fs::write(&helper_source, C_PARITY_SOURCE).unwrap();
        let mut compile_c = Command::new("cc");
        compile_c
            .arg("-std=gnu11")
            .arg("-I")
            .arg(&c_dir)
            .arg("-o")
            .arg(&helper_binary)
            .arg(&helper_source)
            .arg(c_dir.join("nuwax_msgs_NativePgoRequest.c"))
            .arg(c_dir.join("nuwax_msgs_NativePgoResult.c"))
            .arg(c_dir.join("nuwax_msgs_NativePgoStatus.c"))
            .arg("-llcm");
        run_command(
            compile_c,
            "compile fresh generated native PGO C parity helper",
        );

        for (message_type, wire) in [
            ("request", request_wire.as_slice()),
            ("result", result_wire.as_slice()),
            ("status", status_wire.as_slice()),
        ] {
            let mut parity = Command::new(&helper_binary);
            parity.arg(message_type).arg(hex(wire));
            run_command(parity, &format!("fresh generated C {message_type} parity"));
        }

        let mut python_parity = Command::new("python3");
        python_parity
            .arg("-I")
            .arg("-S")
            .arg("-c")
            .arg(PYTHON_PARITY_SOURCE)
            .arg(&python_dir)
            .arg(hex(&request_wire))
            .arg(hex(&result_wire))
            .arg(hex(&status_wire));
        run_command(
            python_parity,
            "clean-import fresh generated native PGO Python parity",
        );
    }

    #[test]
    fn fresh_lcm_gen_cpp_matches_constants_and_exact_wires() {
        let request_wire = encode_native_pgo_request_wire(&request()).unwrap();
        let request = request();
        let result_wire = encode_native_pgo_result_wire(&correction_result(&request)).unwrap();
        let status_wire = encode_native_pgo_status_wire(&cancelled_status(&request)).unwrap();
        let temp = FreshCodegenDir::new();
        let cpp_dir = temp.path().join("cpp");
        fs::create_dir(&cpp_dir).unwrap();

        let mut generate_cpp = Command::new("lcm-gen");
        generate_cpp
            .arg("-x")
            .arg("--cpp-std")
            .arg("c++11")
            .arg("--cpp-hpath")
            .arg(&cpp_dir)
            .args(native_pgo_schema_paths());
        run_command(generate_cpp, "fresh native PGO C++ codegen");

        let helper_source = temp.path().join("native_pgo_cpp_parity.cpp");
        let helper_binary = temp.path().join("native_pgo_cpp_parity");
        fs::write(&helper_source, CPP_PARITY_SOURCE).unwrap();
        let mut compile_cpp = Command::new("c++");
        compile_cpp
            .arg("-std=c++17")
            .arg("-I")
            .arg(&cpp_dir)
            .arg("-o")
            .arg(&helper_binary)
            .arg(&helper_source)
            .arg("-llcm");
        run_command(
            compile_cpp,
            "compile fresh generated native PGO C++ parity helper",
        );

        for (message_type, wire) in [
            ("request", request_wire.as_slice()),
            ("result", result_wire.as_slice()),
            ("status", status_wire.as_slice()),
        ] {
            let mut parity = Command::new(&helper_binary);
            parity.arg(message_type).arg(hex(wire));
            run_command(
                parity,
                &format!("fresh generated C++ {message_type} parity"),
            );
        }
    }

    #[test]
    fn body_and_wire_round_trip_all_three_types() {
        let request = request();
        let request_body = encode_native_pgo_request_body(&request).unwrap();
        let request_wire = encode_native_pgo_request_wire(&request).unwrap();
        assert_eq!(&request_wire[..8], &NATIVE_PGO_REQUEST_FINGERPRINT_V2);
        assert_eq!(&request_wire[8..], request_body);
        assert_eq!(
            decode_native_pgo_request_body(&request_body).unwrap(),
            request
        );
        assert_eq!(
            decode_native_pgo_request_wire(&request_wire).unwrap(),
            request
        );

        let result = correction_result(&request);
        let result_body = encode_native_pgo_result_body(&result).unwrap();
        let result_wire = encode_native_pgo_result_wire(&result).unwrap();
        assert_eq!(&result_wire[..8], &NATIVE_PGO_RESULT_FINGERPRINT_V2);
        assert_eq!(&result_wire[8..], result_body);
        assert_eq!(decode_native_pgo_result_body(&result_body).unwrap(), result);
        assert_eq!(decode_native_pgo_result_wire(&result_wire).unwrap(), result);

        let status = cancelled_status(&request);
        let status_body = encode_native_pgo_status_body(&status).unwrap();
        let status_wire = encode_native_pgo_status_wire(&status).unwrap();
        assert_eq!(&status_wire[..8], &NATIVE_PGO_STATUS_FINGERPRINT_V2);
        assert_eq!(&status_wire[8..], status_body);
        assert_eq!(decode_native_pgo_status_body(&status_body).unwrap(), status);
        assert_eq!(decode_native_pgo_status_wire(&status_wire).unwrap(), status);
    }

    #[test]
    fn every_admitted_completion_has_a_typed_result_disposition() {
        let request = request();
        for result in [
            no_candidate_result(&request),
            correction_result(&request),
            cancelled_result(&request),
        ] {
            let wire = encode_native_pgo_result_wire(&result).unwrap();
            let decoded = decode_native_pgo_result_wire(&wire).unwrap();
            assert_eq!(decoded, result);
            validate_native_pgo_result_echo(&request, &decoded).unwrap();
        }
        assert_eq!(
            cancelled_result(&request).rejection_reason_code,
            NATIVE_PGO_REJECTION_CANCELLED
        );
    }

    #[test]
    fn correction_proof_maxima_and_graph_relations_are_fail_closed() {
        let mut request = request();
        request.identity.source_timestamp_ns = 5000;
        request.identity.registered_cloud.source_timestamp_ns = 5000;
        request.identity.committed_map.source_timestamp_ns = 5000;
        request = seal_native_pgo_request(request).unwrap();
        let mut maximum = correction_result(&request);
        maximum.loop_source_node_id = MAX_NATIVE_PGO_GRAPH_NODES - 1;
        maximum.loop_target_node_id = 0;
        maximum.loop_inlier_count = MAX_NATIVE_PGO_LOOP_INLIERS;
        maximum.optimized_graph_node_count = MAX_NATIVE_PGO_GRAPH_NODES;
        maximum.trajectory = (0..MAX_NATIVE_PGO_GRAPH_NODES)
            .map(|i| OptimizedKeyframe {
                source_timestamp_ns: 5000 - MAX_NATIVE_PGO_GRAPH_NODES + 1 + i,
                translation_m: [i as f64 * 0.01, 0.0, 0.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            })
            .collect();
        maximum.optimized_graph_edge_count = MAX_NATIVE_PGO_GRAPH_EDGES;
        maximum.optimized_graph_loop_edge_count = MAX_NATIVE_PGO_GRAPH_LOOP_EDGES;
        maximum.optimization_iterations = MAX_NATIVE_PGO_OPTIMIZATION_ITERATIONS;
        assert!(seal_native_pgo_result(maximum).is_ok());

        for invalid in [
            {
                let mut value = correction_result(&request);
                value.optimized_graph_node_count = MAX_NATIVE_PGO_GRAPH_NODES + 1;
                value
            },
            {
                let mut value = correction_result(&request);
                value.optimized_graph_edge_count = MAX_NATIVE_PGO_GRAPH_EDGES + 1;
                value
            },
            {
                let mut value = correction_result(&request);
                value.optimized_graph_loop_edge_count = MAX_NATIVE_PGO_GRAPH_LOOP_EDGES + 1;
                value
            },
            {
                let mut value = correction_result(&request);
                value.loop_inlier_count = MAX_NATIVE_PGO_LOOP_INLIERS + 1;
                value
            },
            {
                let mut value = correction_result(&request);
                value.optimization_iterations = MAX_NATIVE_PGO_OPTIMIZATION_ITERATIONS + 1;
                value
            },
            {
                let mut value = correction_result(&request);
                value.optimized_graph_edge_count -= 1;
                value
            },
            {
                let mut value = correction_result(&request);
                value.loop_source_node_id = value.optimized_graph_node_count;
                value
            },
            {
                let mut value = correction_result(&request);
                value.optimized_graph_node_count = 2;
                value.optimized_graph_edge_count = 2;
                value
            },
        ] {
            assert!(seal_native_pgo_result(invalid).is_err());
        }
    }

    #[test]
    fn status_matrix_covers_ready_health_cancel_and_terminal() {
        let request = request();
        let statuses = [
            ready_status(),
            seal_native_pgo_status(NativePgoStatus {
                schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
                backend: NATIVE_PGO_BACKEND,
                runtime_generation: 7,
                backend_generation: 3,
                status_sequence: 2,
                status_timestamp_ns: 0,
                event_kind: NATIVE_PGO_EVENT_HEALTH,
                health_state: NATIVE_PGO_HEALTH_HEALTHY,
                terminal_state: NATIVE_PGO_TERMINAL_NONE,
                cancel_state: NATIVE_PGO_CANCEL_NONE,
                reason_code: NATIVE_PGO_REASON_NONE,
                ..NativePgoStatus::default()
            })
            .unwrap(),
            seal_native_pgo_status(NativePgoStatus {
                schema_version: NATIVE_PGO_SCHEMA_VERSION_V2,
                backend: NATIVE_PGO_BACKEND,
                runtime_generation: 7,
                backend_generation: 3,
                status_sequence: 3,
                status_timestamp_ns: 45,
                request: Some(request.echo()),
                event_kind: NATIVE_PGO_EVENT_CANCEL_REQUESTED,
                health_state: NATIVE_PGO_HEALTH_UNKNOWN,
                terminal_state: NATIVE_PGO_TERMINAL_NONE,
                cancel_state: NATIVE_PGO_CANCEL_REQUESTED,
                reason_code: NATIVE_PGO_REASON_CANCEL_REQUESTED,
                reason: "cancel requested".into(),
                ..NativePgoStatus::default()
            })
            .unwrap(),
            cancelled_status(&request),
        ];
        for status in statuses {
            let wire = encode_native_pgo_status_wire(&status).unwrap();
            assert_eq!(decode_native_pgo_status_wire(&wire).unwrap(), status);
            if status.request.is_some() {
                validate_native_pgo_status_echo(&request, &status).unwrap();
            }
        }

        let mut invalid = ready_status();
        invalid.event_kind = NATIVE_PGO_EVENT_CANCEL_REQUESTED;
        assert!(seal_native_pgo_status(invalid).is_err());
        let mut invalid = cancelled_status(&request);
        invalid.cancel_state = NATIVE_PGO_CANCEL_NONE;
        assert!(seal_native_pgo_status(invalid).is_err());
        let mut duplicate_terminal = cancelled_status(&request);
        duplicate_terminal.event_kind = NATIVE_PGO_EVENT_TERMINAL;
        assert!(seal_native_pgo_status(duplicate_terminal).is_err());
    }

    #[test]
    fn wrong_fingerprint_truncation_trailing_and_body_wire_confusion_fail_closed() {
        let request = request();
        let result = correction_result(&request);
        let status = ready_status();
        let mut wires = [
            encode_native_pgo_request_wire(&request).unwrap(),
            encode_native_pgo_result_wire(&result).unwrap(),
            encode_native_pgo_status_wire(&status).unwrap(),
        ];
        wires[0][0] ^= 1;
        assert!(decode_native_pgo_request_wire(&wires[0]).is_err());
        wires[1].pop();
        assert!(decode_native_pgo_result_wire(&wires[1]).is_err());
        wires[2].push(0);
        assert!(decode_native_pgo_status_wire(&wires[2]).is_err());

        let request_body = encode_native_pgo_request_body(&request).unwrap();
        let result_body = encode_native_pgo_result_body(&result).unwrap();
        let status_body = encode_native_pgo_status_body(&status).unwrap();
        assert!(decode_native_pgo_request_wire(&request_body).is_err());
        assert!(decode_native_pgo_result_wire(&result_body).is_err());
        assert!(decode_native_pgo_status_wire(&status_body).is_err());
        assert!(
            decode_native_pgo_request_body(&encode_native_pgo_request_wire(&request).unwrap())
                .is_err()
        );
    }

    #[test]
    fn dynamic_counts_and_nonfinite_payloads_fail_before_allocation_or_use() {
        let request = request();
        let mut wire = encode_native_pgo_request_wire(&request).unwrap();
        let (coordinate_count_offset, first_coordinate_offset, map_cell_count_offset) =
            request_dynamic_offsets(&wire);
        wire[coordinate_count_offset..coordinate_count_offset + 4]
            .copy_from_slice(&(MAX_NATIVE_PGO_REGISTERED_BODY_COORDINATE_COUNT + 1).to_be_bytes());
        assert!(decode_native_pgo_request_wire(&wire).is_err());

        let mut wire = encode_native_pgo_request_wire(&request).unwrap();
        wire[map_cell_count_offset..map_cell_count_offset + 4]
            .copy_from_slice(&(MAX_NATIVE_PGO_COMMITTED_MAP_CELL_COUNT + 1).to_be_bytes());
        assert!(decode_native_pgo_request_wire(&wire).is_err());

        let mut wire = encode_native_pgo_request_wire(&request).unwrap();
        wire[first_coordinate_offset..first_coordinate_offset + 8]
            .copy_from_slice(&f64::NAN.to_be_bytes());
        reseal_wire(NATIVE_PGO_REQUEST_HASH_DOMAIN, &mut wire);
        assert!(decode_native_pgo_request_wire(&wire).is_err());

        let mut result = correction_result(&request);
        result.fitness_score = f64::NAN;
        assert!(seal_native_pgo_result(result).is_err());
        let mut wire = encode_native_pgo_result_wire(&correction_result(&request)).unwrap();
        let fitness_offset = unique_window_offset(&wire, &0.25_f64.to_be_bytes());
        wire[fitness_offset..fitness_offset + 8].copy_from_slice(&f64::NAN.to_be_bytes());
        reseal_wire(NATIVE_PGO_RESULT_HASH_DOMAIN, &mut wire);
        assert!(decode_native_pgo_result_wire(&wire).is_err());

        let status = ready_status();
        let mut wire = encode_native_pgo_status_wire(&status).unwrap();
        let event_offset = wire.len() - 32 - (5 * 4 + 4 + status.reason.len() + 1);
        wire[event_offset..event_offset + 4]
            .copy_from_slice(&NATIVE_PGO_EVENT_CANCEL_REQUESTED.to_be_bytes());
        reseal_wire(NATIVE_PGO_STATUS_HASH_DOMAIN, &mut wire);
        assert!(decode_native_pgo_status_wire(&wire).is_err());

        let mut request = request;
        request.committed_map_occupancy[0] = 101;
        assert!(seal_native_pgo_request(request).is_err());
    }

    #[test]
    fn proof_hashes_and_exact_echo_are_fail_closed() {
        let request = request();
        let mut corrupt = request.clone();
        corrupt.request_payload_sha256[0] ^= 1;
        assert!(encode_native_pgo_request_wire(&corrupt).is_err());

        let mut result = correction_result(&request);
        result.loop_proof_sha256[0] ^= 1;
        assert!(encode_native_pgo_result_wire(&result).is_err());

        let mut wrong_echo = no_candidate_result(&request);
        wrong_echo.request.identity.occupancy_revision += 1;
        wrong_echo = seal_native_pgo_result(wrong_echo).unwrap();
        assert!(validate_native_pgo_result_echo(&request, &wrong_echo).is_err());
    }

    #[test]
    fn wire_digest_goldens_are_stable() {
        let request = request();
        let result = correction_result(&request);
        let status = cancelled_status(&request);
        let request_wire = encode_native_pgo_request_wire(&request).unwrap();
        let result_wire = encode_native_pgo_result_wire(&result).unwrap();
        let status_wire = encode_native_pgo_status_wire(&status).unwrap();
        assert_eq!(request_wire.len(), 583);
        assert_eq!(
            hex(&sha256_bytes(&request_wire)),
            "a274800c8d7ba751cc520cba4a4ac706642eab179e87778f2b721ce19dd5123d"
        );
        assert_eq!(result_wire.len(), 925);
        assert_eq!(
            hex(&sha256_bytes(&result_wire)),
            "164f1d6055179e5eeb859ee5ad9d57475658659e286c956f806785d2183771cd"
        );
        assert_eq!(status_wire.len(), 550);
        assert_eq!(
            hex(&sha256_bytes(&status_wire)),
            "be249c8ef74ae26757c175340d8b76d9fb812376df468d1cadfa879d8b2fa687"
        );
    }

    fn request_dynamic_offsets(wire: &[u8]) -> (usize, usize, usize) {
        let mut cursor = Cursor::new(&wire[8..]);
        cursor.read_i32().unwrap();
        cursor.read_i32().unwrap();
        cursor.read_i64().unwrap();
        cursor.read_i64().unwrap();
        cursor.read_i64().unwrap();
        cursor
            .read_string(MAX_IDENTITY_STRING_BYTES, "mapping_session_id")
            .unwrap();
        cursor.read_i64().unwrap();
        cursor.read_i64().unwrap();
        cursor.read_array::<16>().unwrap();
        for _ in 0..7 {
            cursor.read_i64().unwrap();
        }
        let coordinate_count_offset = 8 + cursor.offset;
        let coordinate_count = cursor.read_i32().unwrap();
        let first_coordinate_offset = 8 + cursor.offset;
        for _ in 0..coordinate_count {
            cursor.read_f64().unwrap();
        }
        cursor.read_array::<32>().unwrap();
        cursor.read_array::<32>().unwrap();
        cursor.read_i64().unwrap();
        cursor.read_i64().unwrap();
        for _ in 0..7 {
            cursor.read_f64().unwrap();
        }
        cursor.read_array::<32>().unwrap();
        cursor.read_i64().unwrap();
        cursor.read_i64().unwrap();
        cursor.read_f64().unwrap();
        cursor.read_f64().unwrap();
        cursor.read_f64().unwrap();
        (
            coordinate_count_offset,
            first_coordinate_offset,
            8 + cursor.offset,
        )
    }

    fn reseal_wire(domain: &[u8], wire: &mut [u8]) {
        let body_end = wire.len() - 32;
        let digest = hash_body_prefix(domain, &wire[8..body_end]);
        wire[body_end..].copy_from_slice(&digest);
    }

    fn hex(bytes: &[u8]) -> String {
        bytes.iter().map(|byte| format!("{byte:02x}")).collect()
    }

    fn unique_window_offset(haystack: &[u8], needle: &[u8]) -> usize {
        let offsets: Vec<_> = haystack
            .windows(needle.len())
            .enumerate()
            .filter_map(|(offset, candidate)| (candidate == needle).then_some(offset))
            .collect();
        assert_eq!(offsets.len(), 1, "test value must have one wire occurrence");
        offsets[0]
    }

    fn native_pgo_schema_paths() -> [PathBuf; 3] {
        let lcmtypes = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../lcmtypes/nuwax_msgs");
        [
            lcmtypes.join("NativePgoRequest.lcm"),
            lcmtypes.join("NativePgoResult.lcm"),
            lcmtypes.join("NativePgoStatus.lcm"),
        ]
    }

    fn run_command(mut command: Command, label: &str) -> Output {
        let output = command
            .output()
            .unwrap_or_else(|error| panic!("{label} could not start: {error}"));
        assert!(
            output.status.success(),
            "{label} failed with {:?}\nstdout:\n{}\nstderr:\n{}",
            output.status.code(),
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        output
    }

    struct FreshCodegenDir {
        path: PathBuf,
    }

    impl FreshCodegenDir {
        fn new() -> Self {
            static NEXT_ID: AtomicU64 = AtomicU64::new(0);
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "nuwax-native-pgo-codegen-{}-{id}",
                std::process::id()
            ));
            fs::create_dir(&path).unwrap();
            Self { path }
        }

        fn path(&self) -> &Path {
            &self.path
        }
    }

    impl Drop for FreshCodegenDir {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.path);
        }
    }

    const C_PARITY_SOURCE: &str = r#"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nuwax_msgs_NativePgoRequest.h"
#include "nuwax_msgs_NativePgoResult.h"
#include "nuwax_msgs_NativePgoStatus.h"

_Static_assert(NUWAX_MSGS_NATIVEPGOREQUEST_SCHEMA_VERSION_V2 == 2, "request schema");
_Static_assert(NUWAX_MSGS_NATIVEPGOREQUEST_BACKEND_NATIVE_PGO == 2, "request backend");
_Static_assert(NUWAX_MSGS_NATIVEPGOREQUEST_MAX_REGISTERED_BODY_POINT_COUNT == 100000LL, "request point max");
_Static_assert(NUWAX_MSGS_NATIVEPGOREQUEST_MAX_REGISTERED_BODY_COORDINATE_COUNT == 300000, "request coordinate max");
_Static_assert(NUWAX_MSGS_NATIVEPGOREQUEST_MAX_COMMITTED_MAP_CELL_COUNT == 67108864, "request map max");

_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_SCHEMA_VERSION_V2 == 2, "result schema");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_BACKEND_NATIVE_PGO == 2, "result backend");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_REJECTION_CANCELLED == 5, "cancelled result");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_MAX_GRAPH_NODES == 2000LL, "node max");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_MAX_GRAPH_EDGES == 3997LL, "edge max");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_MAX_GRAPH_LOOP_EDGES == 1998LL, "loop edge max");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_MAX_LOOP_INLIERS == 20000LL, "inlier max");
_Static_assert(NUWAX_MSGS_NATIVEPGORESULT_MAX_OPTIMIZATION_ITERATIONS == 300, "iteration max");

_Static_assert(NUWAX_MSGS_NATIVEPGOSTATUS_SCHEMA_VERSION_V2 == 2, "status schema");
_Static_assert(NUWAX_MSGS_NATIVEPGOSTATUS_EVENT_CANCEL_ACKNOWLEDGED == 8, "cancel ack event");
_Static_assert(NUWAX_MSGS_NATIVEPGOSTATUS_TERMINAL_CANCELLED == 4, "cancel terminal");
_Static_assert(NUWAX_MSGS_NATIVEPGOSTATUS_CANCEL_ACKNOWLEDGED == 2, "cancel ack");
_Static_assert(NUWAX_MSGS_NATIVEPGOSTATUS_REASON_CANCELLED == 11, "cancel reason");

static int nibble(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

#define ROUNDTRIP(PREFIX) do { \
    PREFIX message; \
    memset(&message, 0, sizeof(message)); \
    int decoded = PREFIX##_decode(input, 0, (int) length, &message); \
    if (decoded != (int) length) return 30; \
    int encoded_size = PREFIX##_encoded_size(&message); \
    if (encoded_size != (int) length) return 31; \
    uint8_t *output = (uint8_t *) malloc(length); \
    if (output == NULL) return 32; \
    int encoded = PREFIX##_encode(output, 0, encoded_size, &message); \
    int equal = encoded == encoded_size && memcmp(input, output, length) == 0; \
    PREFIX##_decode_cleanup(&message); \
    free(output); \
    free(input); \
    return equal ? 0 : 33; \
} while (0)

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    if ((uint64_t) __nuwax_msgs_NativePgoRequest_get_hash() != UINT64_C(0xa714e98d60d0b620)) return 20;
    if ((uint64_t) __nuwax_msgs_NativePgoResult_get_hash() != UINT64_C(0xadd11e19aa0ec18b)) return 21;
    if ((uint64_t) __nuwax_msgs_NativePgoStatus_get_hash() != UINT64_C(0x3091859182a57b03)) return 22;

    size_t hex_length = strlen(argv[2]);
    if (hex_length == 0 || hex_length % 2 != 0) return 3;
    size_t length = hex_length / 2;
    uint8_t *input = (uint8_t *) malloc(length);
    if (input == NULL) return 4;
    for (size_t index = 0; index < length; ++index) {
        int high = nibble(argv[2][index * 2]);
        int low = nibble(argv[2][index * 2 + 1]);
        if (high < 0 || low < 0) return 5;
        input[index] = (uint8_t) ((high << 4) | low);
    }

    if (strcmp(argv[1], "request") == 0) ROUNDTRIP(nuwax_msgs_NativePgoRequest);
    if (strcmp(argv[1], "result") == 0) ROUNDTRIP(nuwax_msgs_NativePgoResult);
    if (strcmp(argv[1], "status") == 0) ROUNDTRIP(nuwax_msgs_NativePgoStatus);
    free(input);
    return 6;
}
"#;

    const PYTHON_PARITY_SOURCE: &str = r#"
import sys
sys.path.insert(0, sys.argv[1])
from nuwax_msgs import NativePgoRequest, NativePgoResult, NativePgoStatus

assert NativePgoRequest.SCHEMA_VERSION_V2 == 2
assert NativePgoRequest.BACKEND_NATIVE_PGO == 2
assert NativePgoRequest.MAX_REGISTERED_BODY_POINT_COUNT == 100000
assert NativePgoRequest.MAX_REGISTERED_BODY_COORDINATE_COUNT == 300000
assert NativePgoRequest.MAX_COMMITTED_MAP_CELL_COUNT == 67108864

assert NativePgoResult.SCHEMA_VERSION_V2 == 2
assert NativePgoResult.BACKEND_NATIVE_PGO == 2
assert NativePgoResult.REJECTION_CANCELLED == 5
assert NativePgoResult.MAX_GRAPH_NODES == 2000
assert NativePgoResult.MAX_GRAPH_EDGES == 3997
assert NativePgoResult.MAX_GRAPH_LOOP_EDGES == 1998
assert NativePgoResult.MAX_LOOP_INLIERS == 20000
assert NativePgoResult.MAX_OPTIMIZATION_ITERATIONS == 300

assert NativePgoStatus.SCHEMA_VERSION_V2 == 2
assert NativePgoStatus.EVENT_CANCEL_ACKNOWLEDGED == 8
assert NativePgoStatus.TERMINAL_CANCELLED == 4
assert NativePgoStatus.CANCEL_ACKNOWLEDGED == 2
assert NativePgoStatus.REASON_CANCELLED == 11

assert NativePgoRequest().get_hash() == 0xa714e98d60d0b620
assert NativePgoResult().get_hash() == 0xadd11e19aa0ec18b
assert NativePgoStatus().get_hash() == 0x3091859182a57b03

for cls, encoded_hex in zip(
    (NativePgoRequest, NativePgoResult, NativePgoStatus),
    sys.argv[2:5],
):
    encoded = bytes.fromhex(encoded_hex)
    decoded = cls.decode(encoded)
    assert decoded.encode() == encoded
"#;

    const CPP_PARITY_SOURCE: &str = r#"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include <nuwax_msgs/NativePgoRequest.hpp>
#include <nuwax_msgs/NativePgoResult.hpp>
#include <nuwax_msgs/NativePgoStatus.hpp>

using Request = nuwax_msgs::NativePgoRequest;
using Result = nuwax_msgs::NativePgoResult;
using Status = nuwax_msgs::NativePgoStatus;

static_assert(Request::SCHEMA_VERSION_V2 == 2, "request schema");
static_assert(Request::BACKEND_NATIVE_PGO == 2, "request backend");
static_assert(Request::MAX_REGISTERED_BODY_POINT_COUNT == 100000LL, "request point max");
static_assert(Request::MAX_REGISTERED_BODY_COORDINATE_COUNT == 300000, "request coordinate max");
static_assert(Request::MAX_COMMITTED_MAP_CELL_COUNT == 67108864, "request map max");

static_assert(Result::SCHEMA_VERSION_V2 == 2, "result schema");
static_assert(Result::BACKEND_NATIVE_PGO == 2, "result backend");
static_assert(Result::REJECTION_CANCELLED == 5, "cancelled result");
static_assert(Result::MAX_GRAPH_NODES == 2000LL, "node max");
static_assert(Result::MAX_GRAPH_EDGES == 3997LL, "edge max");
static_assert(Result::MAX_GRAPH_LOOP_EDGES == 1998LL, "loop edge max");
static_assert(Result::MAX_LOOP_INLIERS == 20000LL, "inlier max");
static_assert(Result::MAX_OPTIMIZATION_ITERATIONS == 300, "iteration max");
static_assert(std::is_same<decltype(Result::MAX_GRAPH_NODES), const int64_t>::value,
              "graph maxima must remain int64");

static_assert(Status::SCHEMA_VERSION_V2 == 2, "status schema");
static_assert(Status::EVENT_CANCEL_ACKNOWLEDGED == 8, "cancel ack event");
static_assert(Status::TERMINAL_CANCELLED == 4, "cancel terminal");
static_assert(Status::CANCEL_ACKNOWLEDGED == 2, "cancel ack");
static_assert(Status::REASON_CANCELLED == 11, "cancel reason");

static int nibble(char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    return -1;
}

template <typename Message>
static int roundtrip(const std::vector<uint8_t>& input) {
    Message message;
    int decoded = message.decode(input.data(), 0, static_cast<int>(input.size()));
    if (decoded != static_cast<int>(input.size())) return 30;
    std::vector<uint8_t> output(static_cast<std::size_t>(message.getEncodedSize()));
    int encoded = message.encode(output.data(), 0, static_cast<int>(output.size()));
    if (encoded != static_cast<int>(input.size())) return 31;
    return output == input ? 0 : 32;
}

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    if (static_cast<uint64_t>(Request::getHash()) != UINT64_C(0xa714e98d60d0b620)) return 20;
    if (static_cast<uint64_t>(Result::getHash()) != UINT64_C(0xadd11e19aa0ec18b)) return 21;
    if (static_cast<uint64_t>(Status::getHash()) != UINT64_C(0x3091859182a57b03)) return 22;

    std::string encoded_hex(argv[2]);
    if (encoded_hex.empty() || encoded_hex.size() % 2 != 0) return 3;
    std::vector<uint8_t> input(encoded_hex.size() / 2);
    for (std::size_t index = 0; index < input.size(); ++index) {
        int high = nibble(encoded_hex[index * 2]);
        int low = nibble(encoded_hex[index * 2 + 1]);
        if (high < 0 || low < 0) return 4;
        input[index] = static_cast<uint8_t>((high << 4) | low);
    }

    std::string type(argv[1]);
    if (type == "request") return roundtrip<Request>(input);
    if (type == "result") return roundtrip<Result>(input);
    if (type == "status") return roundtrip<Status>(input);
    return 5;
}
"#;
}
