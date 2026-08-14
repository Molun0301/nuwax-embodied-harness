//! LCM message codecs used by the Rust transport boundary.
//!
//! `services/nuwax-workspace/lcmtypes` is the IDL source of truth. The layouts
//! and fingerprints in this module must match `lcm-gen 1.5.2`; the cross-language
//! golden tests enforce that relationship. Top-level messages begin with the
//! eight-byte recursive schema fingerprint, while nested fields do not repeat it.

use crate::{CodecError, CodecResult};

/// `geometry_msgs.Pose` fingerprint from `dimos-lcm==0.1.3`.
pub const POSE_FINGERPRINT: [u8; 8] = 0x2456_34ae_2ab1_7b76_u64.to_be_bytes();
/// `geometry_msgs.PoseStamped` fingerprint from `dimos-lcm==0.1.3`.
pub const POSE_STAMPED_FINGERPRINT: [u8; 8] = 0x6a82_6964_58c2_79a0_u64.to_be_bytes();
/// `geometry_msgs.PointStamped` fingerprint from `dimos-lcm==0.1.3`.
pub const POINT_STAMPED_FINGERPRINT: [u8; 8] = 0x9cd7_6473_8ea6_29af_u64.to_be_bytes();
/// `geometry_msgs.Twist` fingerprint from `dimos-lcm==0.1.3`.
pub const TWIST_FINGERPRINT: [u8; 8] = 0x2e7c_07d7_cdf7_e027_u64.to_be_bytes();
/// `std_msgs.Bool` fingerprint from `dimos-lcm==0.1.3`.
pub const BOOL_FINGERPRINT: [u8; 8] = 0x1ebe_f06a_5095_0e3e_u64.to_be_bytes();
/// `nav_msgs.Path` fingerprint from `dimos-lcm==0.1.3`.
pub const PATH_FINGERPRINT: [u8; 8] = 0xc3ae_62ac_b357_93e2_u64.to_be_bytes();
/// `nuwax_msgs.GoalEnvelope` generated-C recursive fingerprint.
pub const GOAL_ENVELOPE_FINGERPRINT: [u8; 8] = 0xe2dc_6882_37b9_a31b_u64.to_be_bytes();
/// `nuwax_msgs.PathEnvelope` generated-C recursive fingerprint.
pub const PATH_ENVELOPE_FINGERPRINT: [u8; 8] = 0xf11d_70cc_600f_1ab1_u64.to_be_bytes();
/// `nav_msgs.OccupancyGrid` fingerprint from `dimos-lcm==0.1.3`.
pub const OCCUPANCY_GRID_FINGERPRINT: [u8; 8] = 0xe7df_d179_cdfc_3b65_u64.to_be_bytes();
/// `nav_msgs.Odometry` fingerprint from `dimos-lcm==0.1.3`.
pub const ODOMETRY_FINGERPRINT: [u8; 8] = 0x94e1_7f94_648c_f0e4_u64.to_be_bytes();
/// `sensor_msgs.PointCloud2` fingerprint from `dimos-lcm==0.1.3`.
pub const POINT_CLOUD2_FINGERPRINT: [u8; 8] = 0xf5eb_3da1_c285_3175_u64.to_be_bytes();
/// `tf2_msgs.TFMessage` fingerprint from `dimos-lcm==0.1.3`.
pub const TF_MESSAGE_FINGERPRINT: [u8; 8] = 0xc2b8_a1c3_3a89_23ec_u64.to_be_bytes();

/// `geometry_msgs.Transform` fingerprint generated from the repository IDL.
pub const GEOMETRY_TRANSFORM_FINGERPRINT: [u8; 8] = 0xee5f_f426_47bf_3f91_u64.to_be_bytes();
/// `nuwax_msgs.NavigationState` generated-C fingerprint.
pub const NAVIGATION_STATE_FINGERPRINT: [u8; 8] = 0x402e_9704_f8d2_5156_u64.to_be_bytes();
/// `nuwax_msgs.CancelNavigation` generated-C fingerprint.
pub const CANCEL_NAVIGATION_FINGERPRINT: [u8; 8] = 0x9704_4b28_e9c5_dd22_u64.to_be_bytes();
/// `nuwax_msgs.Frontier` generated-C fingerprint.
pub const FRONTIER_FINGERPRINT: [u8; 8] = 0x1abf_83ea_0ced_9c44_u64.to_be_bytes();
/// `nuwax_msgs.FrontierList` generated-C recursive fingerprint.
pub const FRONTIER_LIST_FINGERPRINT: [u8; 8] = 0x213d_b68f_b817_1f76_u64.to_be_bytes();
/// The checked-in `FrontierList.lcm` declares a fixed array of this size.
pub const FRONTIER_LIST_CAPACITY: usize = 256;
/// Current immutable navigation-envelope schema.
pub const NAVIGATION_ENVELOPE_SCHEMA_VERSION: i32 = 4;
/// A public navigation request owns the goal lease.
pub const NAVIGATION_OWNER_KIND_PUBLIC_NAV: i32 = 1;
/// An exploration parent owns the child navigation goal lease.
pub const NAVIGATION_OWNER_KIND_EXPLORATION_CHILD: i32 = 2;

/// LCM representation of `std_msgs.Header`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Header {
    /// Sequence number.
    pub seq: i32,
    /// Timestamp seconds.
    pub stamp_sec: i32,
    /// Timestamp nanoseconds.
    pub stamp_nsec: i32,
    /// Frame identifier.
    pub frame_id: String,
}

/// LCM representation of `geometry_msgs.Pose`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Pose {
    /// Position `(x, y, z)`.
    pub position: [f64; 3],
    /// Quaternion `(x, y, z, w)`, matching the generated DimOS schema.
    pub orientation_xyzw: [f64; 4],
}

/// LCM representation of `geometry_msgs.PoseStamped`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PoseStamped {
    /// Header.
    pub header: Header,
    /// Pose.
    pub pose: Pose,
}

/// LCM representation of `geometry_msgs.PointStamped`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PointStamped {
    /// Header.
    pub header: Header,
    /// Point `(x, y, z)`.
    pub point: [f64; 3],
}

/// LCM representation of `std_msgs.Bool`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Bool {
    /// Boolean payload.
    pub data: bool,
}

/// LCM representation of `nav_msgs.Path`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Path {
    /// Header shared by the complete path.
    pub header: Header,
    /// Ordered stamped poses.
    pub poses: Vec<PoseStamped>,
}

/// 目标准入前冻结的权威已提交地图身份。
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CommittedMapIdentity {
    /// 提交该快照的建图会话。
    pub session_id: String,
    /// 建图会话实例代次。
    pub session_generation: i64,
    /// 进程级已提交占据栅格修订号。
    pub revision: i64,
    /// 规范占据栅格载荷摘要。
    pub payload_sha256: [u8; 32],
    /// 已提交快照携带的规范规划坐标系。
    pub frame_id: String,
}

/// 由唯一导航租约 owner 准入的共享 producer 请求。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GoalIntent {
    /// 稳定 producer 名称，例如 `public-navigation` 或 `wavefront`。
    pub producer: String,
    /// 首次创建该 intent 的 producer 实例代次。
    pub producer_epoch: i64,
    /// producer 冻结的仲裁优先级。
    pub priority: i32,
    /// 公开目标/会话标识。
    pub session_id: String,
    /// `NAVIGATION_OWNER_KIND_*` 常量之一。
    pub owner_kind: i32,
    /// 拥有该目标的公开请求。
    pub parent_request_id: String,
    /// 请求的目标代次；零表示委托租约 owner 分配。
    pub goal_generation: i64,
    /// producer 冻结 intent 时观察到的取消代次。
    pub cancel_generation: i64,
    /// 选择该目标时使用的权威已提交地图身份。
    pub committed_map: CommittedMapIdentity,
    /// 规范边界身份；无边界时为空。
    pub boundary_id: String,
    /// 规范边界字段的 SHA-256；无边界时全零。
    pub boundary_sha256: [u8; 32],
    /// 有序边界顶点；闭合边为隐式。
    pub boundary_points: Vec<[f64; 3]>,
    /// 请求级运动检查准入的 Guard 代次。
    pub motion_guard_generation: i64,
    /// 与 `motion_guard_generation` 一起准入的 Guard 启动 nonce。
    pub motion_guard_boot_nonce: [u8; 16],
    /// 请求级运动检查观察到的全局停止代次。
    pub motion_stop_generation: i64,
    /// 规范目标位姿字段的 SHA-256。
    pub goal_sha256: [u8; 32],
    /// 已提交地图坐标系中的目标位姿。
    pub goal: PoseStamped,
}

/// 计算单个目标位姿的规范摘要。
pub fn goal_intent_goal_sha256(goal: &PoseStamped) -> CodecResult<[u8; 32]> {
    let canonical = encode_pose_stamped_body(goal)?;
    Ok(crate::hash::sha256_bytes(&canonical))
}

/// 计算单个边界身份和多边形的规范摘要。
pub fn goal_intent_boundary_sha256(
    frame_id: &str,
    boundary_id: &str,
    points: &[[f64; 3]],
) -> CodecResult<[u8; 32]> {
    let mut canonical = Vec::new();
    encode_string(frame_id, &mut canonical)?;
    encode_string(boundary_id, &mut canonical)?;
    let point_count = i32::try_from(points.len())
        .map_err(|_| CodecError::LcmEncode("GoalIntent boundary exceeds int32".into()))?;
    canonical.extend_from_slice(&point_count.to_be_bytes());
    for point in points {
        for coordinate in point {
            canonical.extend_from_slice(&coordinate.to_bits().to_be_bytes());
        }
    }
    Ok(crate::hash::sha256_bytes(&canonical))
}

/// Versioned navigation goal with public identity and lease ownership.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GoalEnvelope {
    /// Wire schema version. Only version 4 is currently accepted.
    pub schema_version: i32,
    /// Single public business identifier; API `goal_id` maps here losslessly.
    pub session_id: String,
    /// One of the `NAVIGATION_OWNER_KIND_*` constants.
    pub owner_kind: i32,
    /// Public task/request that owns this lease.
    pub parent_request_id: String,
    /// Monotonic generation for this owner's goal slot.
    pub goal_generation: i64,
    /// Stable producer name copied from the admitted goal intent.
    pub producer: String,
    /// Producer instance generation copied from the admitted goal intent.
    pub producer_epoch: i64,
    /// Geometry runtime generation that sealed the envelope and owns cancellation.
    pub producer_runtime_generation: i64,
    /// Committed mapping session that admitted the goal.
    pub committed_map_session_id: String,
    /// Committed mapping-session instance generation.
    pub committed_map_session_generation: i64,
    /// Committed map revision used for planning.
    pub committed_map_revision: i64,
    /// Canonical committed map payload digest.
    pub committed_map_payload_sha256: [u8; 32],
    /// Authoritative planning frame carried by the committed map.
    pub committed_map_frame_id: String,
    /// Source timestamp in nanoseconds.
    pub source_timestamp_ns: i64,
    /// Stable boundary identifier; empty when no boundary is active.
    pub boundary_id: String,
    /// Canonical boundary identity and polygon digest; zero when absent.
    pub boundary_sha256: [u8; 32],
    /// Authoritative frame for all boundary points.
    pub boundary_frame_id: String,
    /// Optional closed navigation boundary, represented as XYZ points.
    pub boundary_points: Vec<[f64; 3]>,
    /// Guard generation admitted by the request-level motion check.
    pub motion_guard_generation: i64,
    /// Guard boot nonce admitted with `motion_guard_generation`.
    pub motion_guard_boot_nonce: [u8; 16],
    /// Global stop generation observed by the request-level motion check.
    pub motion_stop_generation: i64,
    /// SHA-256 of the canonical fields excluding this digest.
    pub payload_sha256: [u8; 32],
    /// Goal pose.
    pub goal: PoseStamped,
}

/// Planner output correlated to exactly one versioned goal.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PathEnvelope {
    /// Wire schema version. Only version 4 is currently accepted.
    pub schema_version: i32,
    /// Public business identifier copied from the goal envelope.
    pub session_id: String,
    /// Lease owner copied from the goal envelope.
    pub owner_kind: i32,
    /// Parent request copied from the goal envelope.
    pub parent_request_id: String,
    /// Goal generation copied from the goal envelope.
    pub goal_generation: i64,
    /// Stable producer name copied from the goal envelope.
    pub producer: String,
    /// Producer instance generation copied from the goal envelope.
    pub producer_epoch: i64,
    /// Geometry runtime generation copied from the goal envelope.
    pub producer_runtime_generation: i64,
    /// Committed mapping session copied from the goal envelope.
    pub committed_map_session_id: String,
    /// Committed mapping-session instance generation copied from the goal envelope.
    pub committed_map_session_generation: i64,
    /// Committed map revision copied from the goal envelope.
    pub committed_map_revision: i64,
    /// Committed map payload digest copied from the goal envelope.
    pub committed_map_payload_sha256: [u8; 32],
    /// Committed map frame copied from the goal envelope.
    pub committed_map_frame_id: String,
    /// Boundary identifier copied from the goal envelope.
    pub boundary_id: String,
    /// Boundary digest copied from the goal envelope.
    pub boundary_sha256: [u8; 32],
    /// Guard generation copied from the goal envelope.
    pub motion_guard_generation: i64,
    /// Guard boot nonce copied from the goal envelope.
    pub motion_guard_boot_nonce: [u8; 16],
    /// Global stop generation copied from the goal envelope.
    pub motion_stop_generation: i64,
    /// Planner restart generation that produced the path.
    pub planner_runtime_generation: i64,
    /// Monotonic path revision within the goal generation.
    pub path_revision: i64,
    /// Source timestamp in nanoseconds.
    pub source_timestamp_ns: i64,
    /// Goal canonical digest copied without modification.
    pub goal_payload_sha256: [u8; 32],
    /// SHA-256 of canonical path-envelope fields excluding this digest.
    pub path_payload_sha256: [u8; 32],
    /// Planner path.
    pub path: Path,
}

/// LCM representation of `nav_msgs.MapMetaData` nested in an occupancy grid.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MapMetaData {
    /// Map load timestamp seconds.
    pub map_load_time_sec: i32,
    /// Map load timestamp nanoseconds.
    pub map_load_time_nsec: i32,
    /// Cell resolution in meters.
    pub resolution: f32,
    /// Grid width in cells.
    pub width: i32,
    /// Grid height in cells.
    pub height: i32,
    /// World pose of cell `(0, 0)`.
    pub origin: Pose,
}

/// LCM representation of `nav_msgs.OccupancyGrid`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct OccupancyGrid {
    /// Header.
    pub header: Header,
    /// Map metadata.
    pub info: MapMetaData,
    /// Signed ROS occupancy values.
    pub data: Vec<i8>,
}

/// DimOS transform body nested in `geometry_msgs.TransformStamped`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Transform {
    /// Translation `(x, y, z)`.
    pub translation: [f64; 3],
    /// Quaternion `(x, y, z, w)`.
    pub rotation_xyzw: [f64; 4],
}

/// LCM representation of `geometry_msgs.TransformStamped`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TransformStamped {
    /// Parent-frame header.
    pub header: Header,
    /// Child frame identifier.
    pub child_frame_id: String,
    /// Transform from parent to child.
    pub transform: Transform,
}

/// LCM representation of `tf2_msgs.TFMessage`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TfMessage {
    /// Ordered stamped transforms.
    pub transforms: Vec<TransformStamped>,
}

/// Repository-native navigation state bridge message.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct NavigationState {
    /// Navigation session identifier.
    pub session_id: String,
    /// Robot frame identifier.
    pub robot_frame: String,
    /// Timestamp seconds.
    pub stamp_sec: i64,
    /// Timestamp nanoseconds.
    pub stamp_nsec: i32,
    /// Native state enum value.
    pub state: i32,
    /// Goal `(x, y, yaw)`.
    pub goal: [f64; 3],
    /// Current `(x, y, yaw)`.
    pub current: [f64; 3],
    /// Normalized completion progress.
    pub progress: f64,
    /// Human-readable status.
    pub message: String,
    /// Geometry runtime process generation.
    pub runtime_generation: i64,
    /// Motion guard generation that acknowledged the command path.
    pub guard_generation: i64,
    /// Typed sink readiness flag (`0` false, nonzero true).
    pub sink_ready: i8,
    /// Planner map frame.
    pub planner_map_frame: String,
    /// Planner pose frame.
    pub planner_pose_frame: String,
    /// Monotonic cancellation generation.
    pub cancel_generation: i64,
    /// Motion guard zero acknowledgement identifier.
    pub zero_ack_id: String,
    /// Monotonic timestamp at which zero velocity was applied.
    pub zero_applied_monotonic_ns: i64,
    /// Lease owner kind associated with this state.
    pub owner_kind: i32,
    /// Parent request associated with this state.
    pub parent_request_id: String,
    /// Goal generation associated with this state.
    pub goal_generation: i64,
    /// Latest accepted path revision.
    pub path_revision: i64,
    /// Canonical goal payload digest.
    pub goal_payload_sha256: [u8; 32],
    /// Canonical path payload digest.
    pub path_payload_sha256: [u8; 32],
}

/// Repository-native navigation cancellation bridge message.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CancelNavigation {
    /// Navigation session identifier.
    pub session_id: String,
    /// Native cancellation reason enum value.
    pub reason: i32,
    /// Monotonic cancellation generation.
    pub cancel_generation: i64,
    /// Lease owner kind of the goal being cancelled.
    pub owner_kind: i32,
    /// Parent request that owns the goal being cancelled.
    pub parent_request_id: String,
    /// Goal generation being cancelled.
    pub goal_generation: i64,
    /// Geometry producer generation that sealed the cancelled goal.
    pub producer_runtime_generation: i64,
}

/// Repository-native frontier candidate bridge message.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Frontier {
    /// Stable candidate identifier.
    pub id: i64,
    /// Candidate center `(x, y)`.
    pub center: [f64; 2],
    /// Candidate radius.
    pub radius: f64,
    /// Candidate utility score.
    pub score: f64,
    /// Number of frontier cells.
    pub cell_count: i32,
    /// Optional annotation.
    pub note: String,
}

/// Repository-native fixed-capacity frontier list bridge message.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontierList {
    /// Timestamp seconds.
    pub stamp_sec: i64,
    /// Timestamp nanoseconds.
    pub stamp_nsec: i32,
    /// Coordinate frame identifier.
    pub frame_id: String,
    /// Logical number of populated entries in `frontiers`.
    pub frontiers_length: usize,
    /// All 256 slots serialized by generated C, including unused slots.
    pub frontiers: Vec<Frontier>,
}

impl Default for FrontierList {
    fn default() -> Self {
        Self {
            stamp_sec: 0,
            stamp_nsec: 0,
            frame_id: String::new(),
            frontiers_length: 0,
            frontiers: vec![Frontier::default(); FRONTIER_LIST_CAPACITY],
        }
    }
}

/// LCM representation of `geometry_msgs.Twist`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Twist {
    /// Linear velocity `(x, y, z)`.
    pub linear: [f64; 3],
    /// Angular velocity `(x, y, z)`.
    pub angular: [f64; 3],
}

/// LCM representation of `nav_msgs.Odometry`.
#[derive(Debug, Clone, PartialEq)]
pub struct Odometry {
    /// Header.
    pub header: Header,
    /// Child frame identifier.
    pub child_frame_id: String,
    /// Pose.
    pub pose: Pose,
    /// Row-major 6x6 pose covariance.
    pub pose_covariance: [f64; 36],
    /// Linear velocity `(x, y, z)`.
    pub twist_linear: [f64; 3],
    /// Angular velocity `(x, y, z)`.
    pub twist_angular: [f64; 3],
    /// Row-major 6x6 twist covariance.
    pub twist_covariance: [f64; 36],
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            header: Header::default(),
            child_frame_id: String::new(),
            pose: Pose::default(),
            pose_covariance: [0.0; 36],
            twist_linear: [0.0; 3],
            twist_angular: [0.0; 3],
            twist_covariance: [0.0; 36],
        }
    }
}

/// LCM representation of `sensor_msgs.PointField`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PointField {
    /// Field name.
    pub name: String,
    /// Byte offset within one point.
    pub offset: i32,
    /// ROS `PointField` datatype identifier.
    pub datatype: u8,
    /// Element count.
    pub count: i32,
}

/// LCM representation of `sensor_msgs.PointCloud2`.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PointCloud2 {
    /// Header.
    pub header: Header,
    /// Point-cloud height.
    pub height: i32,
    /// Point-cloud width.
    pub width: i32,
    /// Point field descriptors.
    pub fields: Vec<PointField>,
    /// Whether scalar values in `data` use big-endian byte order.
    pub is_bigendian: bool,
    /// Bytes per point.
    pub point_step: i32,
    /// Bytes per row.
    pub row_step: i32,
    /// Raw point bytes.
    pub data: Vec<u8>,
    /// Whether the cloud contains no invalid points.
    pub is_dense: bool,
}

/// Encode a top-level `geometry_msgs.Pose` exactly like generated `lcm_encode()`.
pub fn encode_pose(message: &Pose) -> Vec<u8> {
    let mut out = Vec::with_capacity(8 + 56);
    out.extend_from_slice(&POSE_FINGERPRINT);
    encode_pose_body(message, &mut out);
    out
}

/// Decode a top-level `geometry_msgs.Pose` generated message.
pub fn decode_pose(encoded: &[u8]) -> CodecResult<Pose> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(POSE_FINGERPRINT, "geometry_msgs.Pose")?;
    let pose = decode_pose_body(&mut cursor)?;
    cursor.finish("geometry_msgs.Pose")?;
    Ok(pose)
}

/// Encode a top-level `geometry_msgs.PoseStamped` generated message.
pub fn encode_pose_stamped(message: &PoseStamped) -> CodecResult<Vec<u8>> {
    let body = encode_pose_stamped_body(message)?;
    Ok(with_fingerprint(POSE_STAMPED_FINGERPRINT, body))
}

/// Encode only the generated `PoseStamped._encode_one()` body.
///
/// This is the payload convention consumed by `RawLcmFrame`; the transport
/// prepends the fingerprint exactly once before calling `lcm_publish`.
pub fn encode_pose_stamped_body(message: &PoseStamped) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_header(&message.header, &mut out)?;
    encode_pose_body(&message.pose, &mut out);
    Ok(out)
}

/// Decode a top-level `geometry_msgs.PoseStamped` generated message.
pub fn decode_pose_stamped(encoded: &[u8]) -> CodecResult<PoseStamped> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(POSE_STAMPED_FINGERPRINT, "geometry_msgs.PoseStamped")?;
    let message = PoseStamped {
        header: decode_header(&mut cursor)?,
        pose: decode_pose_body(&mut cursor)?,
    };
    cursor.finish("geometry_msgs.PoseStamped")?;
    Ok(message)
}

/// Encode a top-level `geometry_msgs.PointStamped` generated message.
pub fn encode_point_stamped(message: &PointStamped) -> CodecResult<Vec<u8>> {
    let body = encode_point_stamped_body(message)?;
    Ok(with_fingerprint(POINT_STAMPED_FINGERPRINT, body))
}

/// Encode only the generated `PointStamped._encode_one()` body.
pub fn encode_point_stamped_body(message: &PointStamped) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_header(&message.header, &mut out)?;
    encode_f64_array(&message.point, &mut out);
    Ok(out)
}

/// Decode a top-level `geometry_msgs.PointStamped` generated message.
pub fn decode_point_stamped(encoded: &[u8]) -> CodecResult<PointStamped> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(POINT_STAMPED_FINGERPRINT, "geometry_msgs.PointStamped")?;
    let message = decode_point_stamped_from(&mut cursor)?;
    cursor.finish("geometry_msgs.PointStamped")?;
    Ok(message)
}

/// Decode only a generated `PointStamped._encode_one()` body.
pub fn decode_point_stamped_body(encoded: &[u8]) -> CodecResult<PointStamped> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_point_stamped_from(&mut cursor)?;
    cursor.finish("geometry_msgs.PointStamped body")?;
    Ok(message)
}

fn decode_point_stamped_from(cursor: &mut Cursor<'_>) -> CodecResult<PointStamped> {
    Ok(PointStamped {
        header: decode_header(cursor)?,
        point: cursor.read_f64_array()?,
    })
}

/// Encode a top-level `std_msgs.Bool` generated message.
pub fn encode_bool(message: &Bool) -> Vec<u8> {
    with_fingerprint(BOOL_FINGERPRINT, encode_bool_body(message))
}

/// Encode only the generated `Bool._encode_one()` body.
pub fn encode_bool_body(message: &Bool) -> Vec<u8> {
    vec![u8::from(message.data)]
}

/// Decode a top-level `std_msgs.Bool` generated message.
pub fn decode_bool(encoded: &[u8]) -> CodecResult<Bool> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(BOOL_FINGERPRINT, "std_msgs.Bool")?;
    let message = decode_bool_from(&mut cursor)?;
    cursor.finish("std_msgs.Bool")?;
    Ok(message)
}

/// Decode only a generated `Bool._encode_one()` body.
pub fn decode_bool_body(encoded: &[u8]) -> CodecResult<Bool> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_bool_from(&mut cursor)?;
    cursor.finish("std_msgs.Bool body")?;
    Ok(message)
}

fn decode_bool_from(cursor: &mut Cursor<'_>) -> CodecResult<Bool> {
    Ok(Bool {
        data: cursor.read_u8()? != 0,
    })
}

/// Encode a top-level `nav_msgs.Path` generated message.
pub fn encode_path(message: &Path) -> CodecResult<Vec<u8>> {
    let body = encode_path_body(message)?;
    Ok(with_fingerprint(PATH_FINGERPRINT, body))
}

/// Encode only the generated `Path._encode_one()` body.
pub fn encode_path_body(message: &Path) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_path_into(message, &mut out)?;
    Ok(out)
}

/// Decode a top-level `nav_msgs.Path` generated message.
pub fn decode_path(encoded: &[u8]) -> CodecResult<Path> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(PATH_FINGERPRINT, "nav_msgs.Path")?;
    let message = decode_path_from(&mut cursor)?;
    cursor.finish("nav_msgs.Path")?;
    Ok(message)
}

fn encode_path_into(message: &Path, out: &mut Vec<u8>) -> CodecResult<()> {
    let poses_length = i32::try_from(message.poses.len())
        .map_err(|_| CodecError::LcmEncode("Path poses_length exceeds int32".into()))?;
    out.extend_from_slice(&poses_length.to_be_bytes());
    encode_header(&message.header, out)?;
    for pose in &message.poses {
        encode_header(&pose.header, out)?;
        encode_pose_body(&pose.pose, out);
    }
    Ok(())
}

fn decode_path_from(cursor: &mut Cursor<'_>) -> CodecResult<Path> {
    let poses_length = cursor.read_nonnegative_len("poses_length")?;
    if poses_length > cursor.remaining() / 72 {
        return Err(CodecError::LcmDecode(format!(
            "poses_length {poses_length} exceeds remaining Path bytes"
        )));
    }
    let header = decode_header(cursor)?;
    let mut poses = Vec::with_capacity(poses_length);
    for _ in 0..poses_length {
        poses.push(PoseStamped {
            header: decode_header(cursor)?,
            pose: decode_pose_body(cursor)?,
        });
    }
    Ok(Path { header, poses })
}

/// Fill the canonical SHA-256 field of a versioned goal envelope.
pub fn seal_goal_envelope(message: &mut GoalEnvelope) -> CodecResult<()> {
    validate_goal_envelope_semantics(message, false).map_err(CodecError::LcmEncode)?;
    message.payload_sha256 = goal_envelope_payload_sha256(message)?;
    Ok(())
}

/// Compute the canonical goal digest, excluding the digest field itself.
pub fn goal_envelope_payload_sha256(message: &GoalEnvelope) -> CodecResult<[u8; 32]> {
    let mut canonical = Vec::new();
    encode_goal_envelope_prefix(message, &mut canonical)?;
    encode_header(&message.goal.header, &mut canonical)?;
    encode_pose_body(&message.goal.pose, &mut canonical);
    Ok(crate::hash::sha256_bytes(&canonical))
}

/// Validate version, ownership, revision, boundary and canonical goal digest.
pub fn validate_goal_envelope(message: &GoalEnvelope) -> CodecResult<()> {
    validate_goal_envelope_semantics(message, true).map_err(CodecError::LcmDecode)
}

/// Encode a top-level `nuwax_msgs.GoalEnvelope` generated message.
pub fn encode_goal_envelope(message: &GoalEnvelope) -> CodecResult<Vec<u8>> {
    let body = encode_goal_envelope_body(message)?;
    Ok(with_fingerprint(GOAL_ENVELOPE_FINGERPRINT, body))
}

/// Encode only the generated-C `GoalEnvelope` body.
pub fn encode_goal_envelope_body(message: &GoalEnvelope) -> CodecResult<Vec<u8>> {
    validate_goal_envelope_semantics(message, true).map_err(CodecError::LcmEncode)?;
    let mut out = Vec::new();
    encode_goal_envelope_prefix(message, &mut out)?;
    out.extend_from_slice(&message.payload_sha256);
    encode_header(&message.goal.header, &mut out)?;
    encode_pose_body(&message.goal.pose, &mut out);
    Ok(out)
}

/// Decode and validate a top-level `nuwax_msgs.GoalEnvelope` message.
pub fn decode_goal_envelope(encoded: &[u8]) -> CodecResult<GoalEnvelope> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(GOAL_ENVELOPE_FINGERPRINT, "nuwax_msgs.GoalEnvelope")?;
    let message = decode_goal_envelope_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.GoalEnvelope")?;
    validate_goal_envelope(&message)?;
    Ok(message)
}

/// Decode and validate only a generated-C `GoalEnvelope` body.
pub fn decode_goal_envelope_body(encoded: &[u8]) -> CodecResult<GoalEnvelope> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_goal_envelope_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.GoalEnvelope body")?;
    validate_goal_envelope(&message)?;
    Ok(message)
}

fn encode_goal_envelope_prefix(message: &GoalEnvelope, out: &mut Vec<u8>) -> CodecResult<()> {
    let boundary_points_length = i32::try_from(message.boundary_points.len())
        .map_err(|_| CodecError::LcmEncode("boundary_points_length exceeds int32".into()))?;
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    encode_string(&message.session_id, out)?;
    out.extend_from_slice(&message.owner_kind.to_be_bytes());
    encode_string(&message.parent_request_id, out)?;
    out.extend_from_slice(&message.goal_generation.to_be_bytes());
    encode_string(&message.producer, out)?;
    out.extend_from_slice(&message.producer_epoch.to_be_bytes());
    out.extend_from_slice(&message.producer_runtime_generation.to_be_bytes());
    encode_string(&message.committed_map_session_id, out)?;
    out.extend_from_slice(&message.committed_map_session_generation.to_be_bytes());
    out.extend_from_slice(&message.committed_map_revision.to_be_bytes());
    out.extend_from_slice(&message.committed_map_payload_sha256);
    encode_string(&message.committed_map_frame_id, out)?;
    out.extend_from_slice(&message.source_timestamp_ns.to_be_bytes());
    encode_string(&message.boundary_id, out)?;
    out.extend_from_slice(&message.boundary_sha256);
    encode_string(&message.boundary_frame_id, out)?;
    out.extend_from_slice(&boundary_points_length.to_be_bytes());
    for point in &message.boundary_points {
        encode_f64_array(point, out);
    }
    out.extend_from_slice(&message.motion_guard_generation.to_be_bytes());
    out.extend_from_slice(&message.motion_guard_boot_nonce);
    out.extend_from_slice(&message.motion_stop_generation.to_be_bytes());
    Ok(())
}

fn decode_goal_envelope_from(cursor: &mut Cursor<'_>) -> CodecResult<GoalEnvelope> {
    let schema_version = cursor.read_i32()?;
    let session_id = cursor.read_string()?;
    let owner_kind = cursor.read_i32()?;
    let parent_request_id = cursor.read_string()?;
    let goal_generation = cursor.read_i64()?;
    let producer = cursor.read_string()?;
    let producer_epoch = cursor.read_i64()?;
    let producer_runtime_generation = cursor.read_i64()?;
    let committed_map_session_id = cursor.read_string()?;
    let committed_map_session_generation = cursor.read_i64()?;
    let committed_map_revision = cursor.read_i64()?;
    let committed_map_payload_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte committed map digest");
    let committed_map_frame_id = cursor.read_string()?;
    let source_timestamp_ns = cursor.read_i64()?;
    let boundary_id = cursor.read_string()?;
    let boundary_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte boundary digest");
    let boundary_frame_id = cursor.read_string()?;
    let boundary_points_length = cursor.read_nonnegative_len("boundary_points_length")?;
    if boundary_points_length > cursor.remaining() / 24 {
        return Err(CodecError::LcmDecode(format!(
            "boundary_points_length {boundary_points_length} exceeds remaining GoalEnvelope bytes"
        )));
    }
    let mut boundary_points = Vec::with_capacity(boundary_points_length);
    for _ in 0..boundary_points_length {
        boundary_points.push(cursor.read_f64_array()?);
    }
    let motion_guard_generation = cursor.read_i64()?;
    let motion_guard_boot_nonce = cursor
        .read_exact(16)?
        .try_into()
        .expect("16-byte motion guard boot nonce");
    let motion_stop_generation = cursor.read_i64()?;
    let payload_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte goal digest");
    let goal = PoseStamped {
        header: decode_header(cursor)?,
        pose: decode_pose_body(cursor)?,
    };
    Ok(GoalEnvelope {
        schema_version,
        session_id,
        owner_kind,
        parent_request_id,
        goal_generation,
        producer,
        producer_epoch,
        producer_runtime_generation,
        committed_map_session_id,
        committed_map_session_generation,
        committed_map_revision,
        committed_map_payload_sha256,
        committed_map_frame_id,
        source_timestamp_ns,
        boundary_id,
        boundary_sha256,
        boundary_frame_id,
        boundary_points,
        motion_guard_generation,
        motion_guard_boot_nonce,
        motion_stop_generation,
        payload_sha256,
        goal,
    })
}

fn validate_goal_envelope_semantics(
    message: &GoalEnvelope,
    check_digest: bool,
) -> Result<(), String> {
    validate_navigation_identity(
        message.schema_version,
        &message.session_id,
        message.owner_kind,
        &message.parent_request_id,
        message.goal_generation,
        message.source_timestamp_ns,
    )?;
    validate_goal_authority_identity(
        &message.producer,
        message.producer_epoch,
        message.producer_runtime_generation,
        &message.committed_map_session_id,
        message.committed_map_session_generation,
        message.committed_map_revision,
        &message.committed_map_payload_sha256,
        &message.committed_map_frame_id,
    )?;
    if message.motion_guard_generation <= 0 {
        return Err("motion_guard_generation must be positive".into());
    }
    if message
        .motion_guard_boot_nonce
        .iter()
        .all(|byte| *byte == 0)
    {
        return Err("motion_guard_boot_nonce must not be all zero".into());
    }
    if message.motion_stop_generation < 0 {
        return Err("motion_stop_generation must be non-negative".into());
    }
    validate_navigation_frame_id(&message.goal.header.frame_id, "goal pose")?;
    if message.goal.header.frame_id != message.committed_map_frame_id {
        return Err("goal pose frame_id must match committed map frame_id".into());
    }
    if !pose_is_finite(&message.goal.pose)
        || message
            .boundary_points
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
    {
        return Err("goal or boundary contains a non-finite coordinate".into());
    }
    match message.owner_kind {
        NAVIGATION_OWNER_KIND_PUBLIC_NAV => {
            if !message.boundary_id.is_empty()
                || message.boundary_sha256 != [0; 32]
                || !message.boundary_frame_id.is_empty()
                || !message.boundary_points.is_empty()
            {
                return Err("PUBLIC_NAV goal must not carry a navigation boundary".into());
            }
        }
        NAVIGATION_OWNER_KIND_EXPLORATION_CHILD => {
            if message.boundary_points.is_empty() {
                if !message.boundary_id.is_empty()
                    || message.boundary_sha256 != [0; 32]
                    || !message.boundary_frame_id.is_empty()
                {
                    return Err("empty boundary must have empty identity and frame".into());
                }
            } else {
                if message.boundary_id.trim().is_empty()
                    || message.boundary_id.trim() != message.boundary_id
                    || message.boundary_sha256 == [0; 32]
                    || message.boundary_frame_id != message.committed_map_frame_id
                {
                    return Err("exploration boundary identity or frame is invalid".into());
                }
                let expected = goal_intent_boundary_sha256(
                    &message.boundary_frame_id,
                    &message.boundary_id,
                    &message.boundary_points,
                )
                .map_err(|error| error.to_string())?;
                if message.boundary_sha256 != expected {
                    return Err("boundary_sha256 mismatch".into());
                }
            }
        }
        _ => unreachable!("owner kind validated above"),
    }
    if check_digest {
        let expected = goal_envelope_payload_sha256(message).map_err(|error| error.to_string())?;
        if message.payload_sha256 != expected {
            return Err("goal payload_sha256 mismatch".into());
        }
    }
    Ok(())
}

/// Fill the canonical SHA-256 field of a versioned path envelope.
pub fn seal_path_envelope(message: &mut PathEnvelope) -> CodecResult<()> {
    validate_path_envelope_semantics(message, false).map_err(CodecError::LcmEncode)?;
    message.path_payload_sha256 = path_envelope_payload_sha256(message)?;
    Ok(())
}

/// Compute the canonical path digest, excluding the path digest field itself.
pub fn path_envelope_payload_sha256(message: &PathEnvelope) -> CodecResult<[u8; 32]> {
    let mut canonical = Vec::new();
    encode_path_envelope_prefix(message, &mut canonical)?;
    encode_path_into(&message.path, &mut canonical)?;
    Ok(crate::hash::sha256_bytes(&canonical))
}

/// Validate version, ownership, revision and canonical path digest.
pub fn validate_path_envelope(message: &PathEnvelope) -> CodecResult<()> {
    validate_path_envelope_semantics(message, true).map_err(CodecError::LcmDecode)
}

/// Encode a top-level `nuwax_msgs.PathEnvelope` generated message.
pub fn encode_path_envelope(message: &PathEnvelope) -> CodecResult<Vec<u8>> {
    let body = encode_path_envelope_body(message)?;
    Ok(with_fingerprint(PATH_ENVELOPE_FINGERPRINT, body))
}

/// Encode only the generated-C `PathEnvelope` body.
pub fn encode_path_envelope_body(message: &PathEnvelope) -> CodecResult<Vec<u8>> {
    validate_path_envelope_semantics(message, true).map_err(CodecError::LcmEncode)?;
    let mut out = Vec::new();
    encode_path_envelope_prefix(message, &mut out)?;
    out.extend_from_slice(&message.path_payload_sha256);
    encode_path_into(&message.path, &mut out)?;
    Ok(out)
}

/// Decode and validate a top-level `nuwax_msgs.PathEnvelope` message.
pub fn decode_path_envelope(encoded: &[u8]) -> CodecResult<PathEnvelope> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(PATH_ENVELOPE_FINGERPRINT, "nuwax_msgs.PathEnvelope")?;
    let message = decode_path_envelope_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.PathEnvelope")?;
    validate_path_envelope(&message)?;
    Ok(message)
}

/// Decode and validate only a generated-C `PathEnvelope` body.
pub fn decode_path_envelope_body(encoded: &[u8]) -> CodecResult<PathEnvelope> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_path_envelope_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.PathEnvelope body")?;
    validate_path_envelope(&message)?;
    Ok(message)
}

fn encode_path_envelope_prefix(message: &PathEnvelope, out: &mut Vec<u8>) -> CodecResult<()> {
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    encode_string(&message.session_id, out)?;
    out.extend_from_slice(&message.owner_kind.to_be_bytes());
    encode_string(&message.parent_request_id, out)?;
    out.extend_from_slice(&message.goal_generation.to_be_bytes());
    encode_string(&message.producer, out)?;
    out.extend_from_slice(&message.producer_epoch.to_be_bytes());
    out.extend_from_slice(&message.producer_runtime_generation.to_be_bytes());
    encode_string(&message.committed_map_session_id, out)?;
    out.extend_from_slice(&message.committed_map_session_generation.to_be_bytes());
    out.extend_from_slice(&message.committed_map_revision.to_be_bytes());
    out.extend_from_slice(&message.committed_map_payload_sha256);
    encode_string(&message.committed_map_frame_id, out)?;
    encode_string(&message.boundary_id, out)?;
    out.extend_from_slice(&message.boundary_sha256);
    out.extend_from_slice(&message.motion_guard_generation.to_be_bytes());
    out.extend_from_slice(&message.motion_guard_boot_nonce);
    out.extend_from_slice(&message.motion_stop_generation.to_be_bytes());
    out.extend_from_slice(&message.planner_runtime_generation.to_be_bytes());
    out.extend_from_slice(&message.path_revision.to_be_bytes());
    out.extend_from_slice(&message.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&message.goal_payload_sha256);
    Ok(())
}

fn decode_path_envelope_from(cursor: &mut Cursor<'_>) -> CodecResult<PathEnvelope> {
    let schema_version = cursor.read_i32()?;
    let session_id = cursor.read_string()?;
    let owner_kind = cursor.read_i32()?;
    let parent_request_id = cursor.read_string()?;
    let goal_generation = cursor.read_i64()?;
    let producer = cursor.read_string()?;
    let producer_epoch = cursor.read_i64()?;
    let producer_runtime_generation = cursor.read_i64()?;
    let committed_map_session_id = cursor.read_string()?;
    let committed_map_session_generation = cursor.read_i64()?;
    let committed_map_revision = cursor.read_i64()?;
    let committed_map_payload_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte committed map digest");
    let committed_map_frame_id = cursor.read_string()?;
    let boundary_id = cursor.read_string()?;
    let boundary_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte boundary digest");
    let motion_guard_generation = cursor.read_i64()?;
    let motion_guard_boot_nonce = cursor
        .read_exact(16)?
        .try_into()
        .expect("16-byte motion guard boot nonce");
    let motion_stop_generation = cursor.read_i64()?;
    let planner_runtime_generation = cursor.read_i64()?;
    let path_revision = cursor.read_i64()?;
    let source_timestamp_ns = cursor.read_i64()?;
    let goal_payload_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte goal digest");
    let path_payload_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte path digest");
    let path = decode_path_from(cursor)?;
    Ok(PathEnvelope {
        schema_version,
        session_id,
        owner_kind,
        parent_request_id,
        goal_generation,
        producer,
        producer_epoch,
        producer_runtime_generation,
        committed_map_session_id,
        committed_map_session_generation,
        committed_map_revision,
        committed_map_payload_sha256,
        committed_map_frame_id,
        boundary_id,
        boundary_sha256,
        motion_guard_generation,
        motion_guard_boot_nonce,
        motion_stop_generation,
        planner_runtime_generation,
        path_revision,
        source_timestamp_ns,
        goal_payload_sha256,
        path_payload_sha256,
        path,
    })
}

fn validate_path_envelope_semantics(
    message: &PathEnvelope,
    check_digest: bool,
) -> Result<(), String> {
    validate_navigation_identity(
        message.schema_version,
        &message.session_id,
        message.owner_kind,
        &message.parent_request_id,
        message.goal_generation,
        message.source_timestamp_ns,
    )?;
    validate_goal_authority_identity(
        &message.producer,
        message.producer_epoch,
        message.producer_runtime_generation,
        &message.committed_map_session_id,
        message.committed_map_session_generation,
        message.committed_map_revision,
        &message.committed_map_payload_sha256,
        &message.committed_map_frame_id,
    )?;
    if message.boundary_id.is_empty() != (message.boundary_sha256 == [0; 32]) {
        return Err("path boundary identity and digest must be present together".into());
    }
    if message.owner_kind == NAVIGATION_OWNER_KIND_PUBLIC_NAV && !message.boundary_id.is_empty() {
        return Err("PUBLIC_NAV path must not carry a navigation boundary".into());
    }
    if message.motion_guard_generation <= 0 {
        return Err("motion_guard_generation must be positive".into());
    }
    if message
        .motion_guard_boot_nonce
        .iter()
        .all(|byte| *byte == 0)
    {
        return Err("motion_guard_boot_nonce must not be all zero".into());
    }
    if message.motion_stop_generation < 0 {
        return Err("motion_stop_generation must be non-negative".into());
    }
    if message.planner_runtime_generation <= 0 {
        return Err("planner_runtime_generation must be positive".into());
    }
    if message.path_revision <= 0 {
        return Err("path_revision must be positive".into());
    }
    if message.goal_payload_sha256 == [0; 32] {
        return Err("goal_payload_sha256 must be populated".into());
    }
    validate_navigation_frame_id(&message.path.header.frame_id, "path")?;
    if message.path.header.frame_id != message.committed_map_frame_id {
        return Err("path frame_id must match committed map frame_id".into());
    }
    if message.path.poses.is_empty() {
        return Err("path must contain at least one pose".into());
    }
    if message.path.poses.iter().any(|pose| {
        pose.header.frame_id != message.path.header.frame_id || !pose_is_finite(&pose.pose)
    }) {
        return Err("path poses must be finite and match the path frame_id".into());
    }
    if check_digest {
        let expected = path_envelope_payload_sha256(message).map_err(|error| error.to_string())?;
        if message.path_payload_sha256 != expected {
            return Err("path payload_sha256 mismatch".into());
        }
    }
    Ok(())
}

fn validate_navigation_frame_id(frame_id: &str, role: &str) -> Result<(), String> {
    let valid = frame_id == frame_id.trim()
        && !frame_id.is_empty()
        && frame_id.len() <= 255
        && frame_id.split('/').all(|token| {
            let mut chars = token.chars();
            chars
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && chars.all(|character| character.is_ascii_alphanumeric() || character == '_')
        });
    if valid {
        Ok(())
    } else {
        Err(format!("{role} frame_id is invalid: {frame_id:?}"))
    }
}

fn validate_navigation_identity(
    schema_version: i32,
    session_id: &str,
    owner_kind: i32,
    parent_request_id: &str,
    goal_generation: i64,
    source_timestamp_ns: i64,
) -> Result<(), String> {
    if schema_version != NAVIGATION_ENVELOPE_SCHEMA_VERSION {
        return Err(format!(
            "unsupported navigation envelope schema_version={schema_version}"
        ));
    }
    if session_id.trim().is_empty() {
        return Err("session_id must not be empty".into());
    }
    if !matches!(
        owner_kind,
        NAVIGATION_OWNER_KIND_PUBLIC_NAV | NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
    ) {
        return Err(format!("unsupported owner_kind={owner_kind}"));
    }
    if parent_request_id.trim().is_empty() {
        return Err("parent_request_id must not be empty".into());
    }
    if goal_generation <= 0 {
        return Err("goal_generation must be positive".into());
    }
    if source_timestamp_ns <= 0 {
        return Err("source_timestamp_ns must be positive".into());
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_goal_authority_identity(
    producer: &str,
    producer_epoch: i64,
    producer_runtime_generation: i64,
    committed_map_session_id: &str,
    committed_map_session_generation: i64,
    committed_map_revision: i64,
    committed_map_payload_sha256: &[u8; 32],
    committed_map_frame_id: &str,
) -> Result<(), String> {
    if producer.trim().is_empty() || producer.trim() != producer {
        return Err("producer must be a non-empty canonical name".into());
    }
    if producer_epoch <= 0 {
        return Err("producer_epoch must be positive".into());
    }
    if producer_runtime_generation <= 0 {
        return Err("producer_runtime_generation must be positive".into());
    }
    if committed_map_session_id.trim().is_empty()
        || committed_map_session_id.trim() != committed_map_session_id
    {
        return Err("committed_map_session_id must be non-empty and canonical".into());
    }
    if committed_map_session_generation <= 0 {
        return Err("committed_map_session_generation must be positive".into());
    }
    if committed_map_revision <= 0 {
        return Err("committed_map_revision must be positive".into());
    }
    if *committed_map_payload_sha256 == [0; 32] {
        return Err("committed_map_payload_sha256 must be populated".into());
    }
    validate_navigation_frame_id(committed_map_frame_id, "committed map")
}

fn pose_is_finite(pose: &Pose) -> bool {
    pose.position
        .iter()
        .chain(pose.orientation_xyzw.iter())
        .all(|value| value.is_finite())
}

/// Encode a top-level `nav_msgs.OccupancyGrid` generated message.
pub fn encode_occupancy_grid(message: &OccupancyGrid) -> CodecResult<Vec<u8>> {
    let body = encode_occupancy_grid_body(message)?;
    Ok(with_fingerprint(OCCUPANCY_GRID_FINGERPRINT, body))
}

/// Encode only the generated `OccupancyGrid._encode_one()` body.
pub fn encode_occupancy_grid_body(message: &OccupancyGrid) -> CodecResult<Vec<u8>> {
    let data_length = i32::try_from(message.data.len())
        .map_err(|_| CodecError::LcmEncode("OccupancyGrid data_length exceeds int32".into()))?;
    let mut out = Vec::new();
    out.extend_from_slice(&data_length.to_be_bytes());
    encode_header(&message.header, &mut out)?;
    encode_map_metadata_body(&message.info, &mut out);
    out.extend(message.data.iter().map(|value| *value as u8));
    Ok(out)
}

/// Decode a top-level `nav_msgs.OccupancyGrid` generated message.
pub fn decode_occupancy_grid(encoded: &[u8]) -> CodecResult<OccupancyGrid> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(OCCUPANCY_GRID_FINGERPRINT, "nav_msgs.OccupancyGrid")?;
    let message = decode_occupancy_grid_from(&mut cursor)?;
    cursor.finish("nav_msgs.OccupancyGrid")?;
    Ok(message)
}

/// Decode only a generated `OccupancyGrid._encode_one()` body.
pub fn decode_occupancy_grid_body(encoded: &[u8]) -> CodecResult<OccupancyGrid> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_occupancy_grid_from(&mut cursor)?;
    cursor.finish("nav_msgs.OccupancyGrid body")?;
    Ok(message)
}

fn decode_occupancy_grid_from(cursor: &mut Cursor<'_>) -> CodecResult<OccupancyGrid> {
    let data_length = cursor.read_nonnegative_len("data_length")?;
    let header = decode_header(cursor)?;
    let info = decode_map_metadata_body(cursor)?;
    let data = cursor
        .read_exact(data_length)?
        .iter()
        .map(|value| *value as i8)
        .collect();
    Ok(OccupancyGrid { header, info, data })
}

/// Encode a top-level `tf2_msgs.TFMessage` generated message.
pub fn encode_tf_message(message: &TfMessage) -> CodecResult<Vec<u8>> {
    let body = encode_tf_message_body(message)?;
    Ok(with_fingerprint(TF_MESSAGE_FINGERPRINT, body))
}

/// Encode only the generated `TFMessage._encode_one()` body.
pub fn encode_tf_message_body(message: &TfMessage) -> CodecResult<Vec<u8>> {
    let transforms_length = i32::try_from(message.transforms.len())
        .map_err(|_| CodecError::LcmEncode("TFMessage transforms_length exceeds int32".into()))?;
    let mut out = Vec::new();
    out.extend_from_slice(&transforms_length.to_be_bytes());
    for transform in &message.transforms {
        encode_transform_stamped_body(transform, &mut out)?;
    }
    Ok(out)
}

/// Decode a top-level `tf2_msgs.TFMessage` generated message.
pub fn decode_tf_message(encoded: &[u8]) -> CodecResult<TfMessage> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(TF_MESSAGE_FINGERPRINT, "tf2_msgs.TFMessage")?;
    let message = decode_tf_message_from(&mut cursor)?;
    cursor.finish("tf2_msgs.TFMessage")?;
    Ok(message)
}

/// Decode only a generated `TFMessage._encode_one()` body.
pub fn decode_tf_message_body(encoded: &[u8]) -> CodecResult<TfMessage> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_tf_message_from(&mut cursor)?;
    cursor.finish("tf2_msgs.TFMessage body")?;
    Ok(message)
}

fn decode_tf_message_from(cursor: &mut Cursor<'_>) -> CodecResult<TfMessage> {
    const MIN_TRANSFORM_STAMPED_BODY: usize = 78;

    let transforms_length = cursor.read_nonnegative_len("transforms_length")?;
    if transforms_length > cursor.remaining() / MIN_TRANSFORM_STAMPED_BODY {
        return Err(CodecError::LcmDecode(format!(
            "transforms_length {transforms_length} exceeds remaining TFMessage bytes"
        )));
    }
    let mut transforms = Vec::with_capacity(transforms_length);
    for _ in 0..transforms_length {
        transforms.push(decode_transform_stamped_body(cursor)?);
    }
    Ok(TfMessage { transforms })
}

/// Encode a top-level `geometry_msgs.Twist` generated message.
pub fn encode_twist(message: &Twist) -> Vec<u8> {
    let mut out = Vec::with_capacity(8 + 48);
    out.extend_from_slice(&TWIST_FINGERPRINT);
    out.extend_from_slice(&encode_twist_body(message));
    out
}

/// Encode only the generated `Twist._encode_one()` body.
///
/// This is the body API that runtime publishers should place in
/// `RawLcmFrame::payload` before the transport prepends the fingerprint.
pub fn encode_twist_body(message: &Twist) -> Vec<u8> {
    let mut out = Vec::with_capacity(48);
    encode_f64_array(&message.linear, &mut out);
    encode_f64_array(&message.angular, &mut out);
    out
}

/// Decode a top-level `geometry_msgs.Twist` generated message.
pub fn decode_twist(encoded: &[u8]) -> CodecResult<Twist> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(TWIST_FINGERPRINT, "geometry_msgs.Twist")?;
    let message = Twist {
        linear: cursor.read_f64_array()?,
        angular: cursor.read_f64_array()?,
    };
    cursor.finish("geometry_msgs.Twist")?;
    Ok(message)
}

/// Encode a top-level `nav_msgs.Odometry` generated message.
pub fn encode_odometry(message: &Odometry) -> CodecResult<Vec<u8>> {
    let body = encode_odometry_body(message)?;
    Ok(with_fingerprint(ODOMETRY_FINGERPRINT, body))
}

/// Encode only the generated `Odometry._encode_one()` body.
pub fn encode_odometry_body(message: &Odometry) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_header(&message.header, &mut out)?;
    encode_string(&message.child_frame_id, &mut out)?;
    encode_pose_body(&message.pose, &mut out);
    encode_f64_array(&message.pose_covariance, &mut out);
    encode_f64_array(&message.twist_linear, &mut out);
    encode_f64_array(&message.twist_angular, &mut out);
    encode_f64_array(&message.twist_covariance, &mut out);
    Ok(out)
}

/// Decode a top-level `nav_msgs.Odometry` generated message.
pub fn decode_odometry(encoded: &[u8]) -> CodecResult<Odometry> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(ODOMETRY_FINGERPRINT, "nav_msgs.Odometry")?;
    let message = Odometry {
        header: decode_header(&mut cursor)?,
        child_frame_id: cursor.read_string()?,
        pose: decode_pose_body(&mut cursor)?,
        pose_covariance: cursor.read_f64_array()?,
        twist_linear: cursor.read_f64_array()?,
        twist_angular: cursor.read_f64_array()?,
        twist_covariance: cursor.read_f64_array()?,
    };
    cursor.finish("nav_msgs.Odometry")?;
    Ok(message)
}

/// Encode a top-level `sensor_msgs.PointCloud2` generated message.
pub fn encode_point_cloud2(message: &PointCloud2) -> CodecResult<Vec<u8>> {
    let body = encode_point_cloud2_body(message)?;
    Ok(with_fingerprint(POINT_CLOUD2_FINGERPRINT, body))
}

/// Encode only the generated `PointCloud2._encode_one()` body.
pub fn encode_point_cloud2_body(message: &PointCloud2) -> CodecResult<Vec<u8>> {
    let fields_length = i32::try_from(message.fields.len())
        .map_err(|_| CodecError::LcmEncode("PointCloud2 fields_length exceeds int32".into()))?;
    let data_length = i32::try_from(message.data.len())
        .map_err(|_| CodecError::LcmEncode("PointCloud2 data_length exceeds int32".into()))?;

    let mut out = Vec::new();
    out.extend_from_slice(&fields_length.to_be_bytes());
    out.extend_from_slice(&data_length.to_be_bytes());
    encode_header(&message.header, &mut out)?;
    out.extend_from_slice(&message.height.to_be_bytes());
    out.extend_from_slice(&message.width.to_be_bytes());
    for field in &message.fields {
        encode_string(&field.name, &mut out)?;
        out.extend_from_slice(&field.offset.to_be_bytes());
        out.push(field.datatype);
        out.extend_from_slice(&field.count.to_be_bytes());
    }
    out.push(u8::from(message.is_bigendian));
    out.extend_from_slice(&message.point_step.to_be_bytes());
    out.extend_from_slice(&message.row_step.to_be_bytes());
    out.extend_from_slice(&message.data);
    out.push(u8::from(message.is_dense));
    Ok(out)
}

/// Decode a top-level `sensor_msgs.PointCloud2` generated message.
pub fn decode_point_cloud2(encoded: &[u8]) -> CodecResult<PointCloud2> {
    const MIN_POINT_FIELD_BODY: usize = 14;
    const POINT_CLOUD_TAIL_WITHOUT_DATA: usize = 10;

    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(POINT_CLOUD2_FINGERPRINT, "sensor_msgs.PointCloud2")?;
    let fields_length = cursor.read_nonnegative_len("fields_length")?;
    let data_length = cursor.read_nonnegative_len("data_length")?;
    let header = decode_header(&mut cursor)?;
    let height = cursor.read_i32()?;
    let width = cursor.read_i32()?;
    let required_tail = data_length
        .checked_add(POINT_CLOUD_TAIL_WITHOUT_DATA)
        .ok_or(CodecError::LcmDecode(
            "PointCloud2 data_length overflows usize".into(),
        ))?;
    if required_tail > cursor.remaining() {
        return Err(CodecError::LcmDecode(format!(
            "PointCloud2 data_length {data_length} exceeds remaining bytes"
        )));
    }
    let available_field_bytes = cursor.remaining() - required_tail;
    if fields_length > available_field_bytes / MIN_POINT_FIELD_BODY {
        return Err(CodecError::LcmDecode(format!(
            "fields_length {fields_length} exceeds remaining PointCloud2 field bytes"
        )));
    }
    let mut fields = Vec::with_capacity(fields_length);
    for _ in 0..fields_length {
        fields.push(PointField {
            name: cursor.read_string()?,
            offset: cursor.read_i32()?,
            datatype: cursor.read_u8()?,
            count: cursor.read_i32()?,
        });
    }
    let is_bigendian = cursor.read_u8()? != 0;
    let point_step = cursor.read_i32()?;
    let row_step = cursor.read_i32()?;
    let data = cursor.read_exact(data_length)?.to_vec();
    let is_dense = cursor.read_u8()? != 0;
    cursor.finish("sensor_msgs.PointCloud2")?;
    Ok(PointCloud2 {
        header,
        height,
        width,
        fields,
        is_bigendian,
        point_step,
        row_step,
        data,
        is_dense,
    })
}

/// Encode a top-level `geometry_msgs.Transform` message.
pub fn encode_geometry_transform(message: &Transform) -> Vec<u8> {
    with_fingerprint(
        GEOMETRY_TRANSFORM_FINGERPRINT,
        encode_geometry_transform_body(message),
    )
}

/// Encode only the generated-C `geometry_msgs.Transform` body.
pub fn encode_geometry_transform_body(message: &Transform) -> Vec<u8> {
    let mut out = Vec::with_capacity(56);
    encode_f64_array(&message.translation, &mut out);
    encode_f64_array(&message.rotation_xyzw, &mut out);
    out
}

/// Decode a top-level `geometry_msgs.Transform` message.
pub fn decode_geometry_transform(encoded: &[u8]) -> CodecResult<Transform> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(GEOMETRY_TRANSFORM_FINGERPRINT, "geometry_msgs.Transform")?;
    let message = decode_geometry_transform_from(&mut cursor)?;
    cursor.finish("geometry_msgs.Transform")?;
    Ok(message)
}

/// Decode only a generated-C `geometry_msgs.Transform` body.
pub fn decode_geometry_transform_body(encoded: &[u8]) -> CodecResult<Transform> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_geometry_transform_from(&mut cursor)?;
    cursor.finish("geometry_msgs.Transform body")?;
    Ok(message)
}

fn decode_geometry_transform_from(cursor: &mut Cursor<'_>) -> CodecResult<Transform> {
    Ok(Transform {
        translation: cursor.read_f64_array()?,
        rotation_xyzw: cursor.read_f64_array()?,
    })
}

/// Encode a top-level repository-native `nuwax_msgs.NavigationState` message.
pub fn encode_navigation_state(message: &NavigationState) -> CodecResult<Vec<u8>> {
    let body = encode_navigation_state_body(message)?;
    Ok(with_fingerprint(NAVIGATION_STATE_FINGERPRINT, body))
}

/// Encode only the generated-C `NavigationState` body.
pub fn encode_navigation_state_body(message: &NavigationState) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_string(&message.session_id, &mut out)?;
    encode_string(&message.robot_frame, &mut out)?;
    out.extend_from_slice(&message.stamp_sec.to_be_bytes());
    out.extend_from_slice(&message.stamp_nsec.to_be_bytes());
    out.extend_from_slice(&message.state.to_be_bytes());
    encode_f64_array(&message.goal, &mut out);
    encode_f64_array(&message.current, &mut out);
    out.extend_from_slice(&message.progress.to_be_bytes());
    encode_string(&message.message, &mut out)?;
    out.extend_from_slice(&message.runtime_generation.to_be_bytes());
    out.extend_from_slice(&message.guard_generation.to_be_bytes());
    out.push(message.sink_ready as u8);
    encode_string(&message.planner_map_frame, &mut out)?;
    encode_string(&message.planner_pose_frame, &mut out)?;
    out.extend_from_slice(&message.cancel_generation.to_be_bytes());
    encode_string(&message.zero_ack_id, &mut out)?;
    out.extend_from_slice(&message.zero_applied_monotonic_ns.to_be_bytes());
    out.extend_from_slice(&message.owner_kind.to_be_bytes());
    encode_string(&message.parent_request_id, &mut out)?;
    out.extend_from_slice(&message.goal_generation.to_be_bytes());
    out.extend_from_slice(&message.path_revision.to_be_bytes());
    out.extend_from_slice(&message.goal_payload_sha256);
    out.extend_from_slice(&message.path_payload_sha256);
    Ok(out)
}

/// Decode a top-level repository-native `nuwax_msgs.NavigationState` message.
pub fn decode_navigation_state(encoded: &[u8]) -> CodecResult<NavigationState> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(NAVIGATION_STATE_FINGERPRINT, "nuwax_msgs.NavigationState")?;
    let message = decode_navigation_state_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.NavigationState")?;
    Ok(message)
}

/// Decode only a generated-C `NavigationState` body.
pub fn decode_navigation_state_body(encoded: &[u8]) -> CodecResult<NavigationState> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_navigation_state_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.NavigationState body")?;
    Ok(message)
}

fn decode_navigation_state_from(cursor: &mut Cursor<'_>) -> CodecResult<NavigationState> {
    Ok(NavigationState {
        session_id: cursor.read_string()?,
        robot_frame: cursor.read_string()?,
        stamp_sec: cursor.read_i64()?,
        stamp_nsec: cursor.read_i32()?,
        state: cursor.read_i32()?,
        goal: cursor.read_f64_array()?,
        current: cursor.read_f64_array()?,
        progress: cursor.read_f64()?,
        message: cursor.read_string()?,
        runtime_generation: cursor.read_i64()?,
        guard_generation: cursor.read_i64()?,
        sink_ready: cursor.read_u8()? as i8,
        planner_map_frame: cursor.read_string()?,
        planner_pose_frame: cursor.read_string()?,
        cancel_generation: cursor.read_i64()?,
        zero_ack_id: cursor.read_string()?,
        zero_applied_monotonic_ns: cursor.read_i64()?,
        owner_kind: cursor.read_i32()?,
        parent_request_id: cursor.read_string()?,
        goal_generation: cursor.read_i64()?,
        path_revision: cursor.read_i64()?,
        goal_payload_sha256: cursor
            .read_exact(32)?
            .try_into()
            .expect("32-byte goal digest"),
        path_payload_sha256: cursor
            .read_exact(32)?
            .try_into()
            .expect("32-byte path digest"),
    })
}

/// Encode a top-level repository-native `nuwax_msgs.CancelNavigation` message.
pub fn encode_cancel_navigation(message: &CancelNavigation) -> CodecResult<Vec<u8>> {
    let body = encode_cancel_navigation_body(message)?;
    Ok(with_fingerprint(CANCEL_NAVIGATION_FINGERPRINT, body))
}

/// Encode only the generated-C `CancelNavigation` body.
pub fn encode_cancel_navigation_body(message: &CancelNavigation) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_string(&message.session_id, &mut out)?;
    out.extend_from_slice(&message.reason.to_be_bytes());
    out.extend_from_slice(&message.cancel_generation.to_be_bytes());
    out.extend_from_slice(&message.owner_kind.to_be_bytes());
    encode_string(&message.parent_request_id, &mut out)?;
    out.extend_from_slice(&message.goal_generation.to_be_bytes());
    out.extend_from_slice(&message.producer_runtime_generation.to_be_bytes());
    Ok(out)
}

/// Decode a top-level repository-native `nuwax_msgs.CancelNavigation` message.
pub fn decode_cancel_navigation(encoded: &[u8]) -> CodecResult<CancelNavigation> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(CANCEL_NAVIGATION_FINGERPRINT, "nuwax_msgs.CancelNavigation")?;
    let message = decode_cancel_navigation_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.CancelNavigation")?;
    Ok(message)
}

/// Decode only a generated-C `CancelNavigation` body.
pub fn decode_cancel_navigation_body(encoded: &[u8]) -> CodecResult<CancelNavigation> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_cancel_navigation_from(&mut cursor)?;
    cursor.finish("nuwax_msgs.CancelNavigation body")?;
    Ok(message)
}

fn decode_cancel_navigation_from(cursor: &mut Cursor<'_>) -> CodecResult<CancelNavigation> {
    Ok(CancelNavigation {
        session_id: cursor.read_string()?,
        reason: cursor.read_i32()?,
        cancel_generation: cursor.read_i64()?,
        owner_kind: cursor.read_i32()?,
        parent_request_id: cursor.read_string()?,
        goal_generation: cursor.read_i64()?,
        producer_runtime_generation: cursor.read_i64()?,
    })
}

/// Encode a top-level repository-native `nuwax_msgs__Frontier` message.
pub fn encode_frontier(message: &Frontier) -> CodecResult<Vec<u8>> {
    let body = encode_frontier_body(message)?;
    Ok(with_fingerprint(FRONTIER_FINGERPRINT, body))
}

/// Encode only the generated-C `Frontier` body.
pub fn encode_frontier_body(message: &Frontier) -> CodecResult<Vec<u8>> {
    let mut out = Vec::new();
    encode_frontier_into(message, &mut out)?;
    Ok(out)
}

/// Decode a top-level repository-native `nuwax_msgs__Frontier` message.
pub fn decode_frontier(encoded: &[u8]) -> CodecResult<Frontier> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(FRONTIER_FINGERPRINT, "nuwax_msgs__Frontier")?;
    let message = decode_frontier_from(&mut cursor)?;
    cursor.finish("nuwax_msgs__Frontier")?;
    Ok(message)
}

/// Decode only a generated-C `Frontier` body.
pub fn decode_frontier_body(encoded: &[u8]) -> CodecResult<Frontier> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_frontier_from(&mut cursor)?;
    cursor.finish("nuwax_msgs__Frontier body")?;
    Ok(message)
}

fn encode_frontier_into(message: &Frontier, out: &mut Vec<u8>) -> CodecResult<()> {
    out.extend_from_slice(&message.id.to_be_bytes());
    encode_f64_array(&message.center, out);
    out.extend_from_slice(&message.radius.to_be_bytes());
    out.extend_from_slice(&message.score.to_be_bytes());
    out.extend_from_slice(&message.cell_count.to_be_bytes());
    encode_string(&message.note, out)
}

fn decode_frontier_from(cursor: &mut Cursor<'_>) -> CodecResult<Frontier> {
    Ok(Frontier {
        id: cursor.read_i64()?,
        center: cursor.read_f64_array()?,
        radius: cursor.read_f64()?,
        score: cursor.read_f64()?,
        cell_count: cursor.read_i32()?,
        note: cursor.read_string()?,
    })
}

/// Encode a top-level repository-native `nuwax_msgs__FrontierList` message.
pub fn encode_frontier_list(message: &FrontierList) -> CodecResult<Vec<u8>> {
    let body = encode_frontier_list_body(message)?;
    Ok(with_fingerprint(FRONTIER_LIST_FINGERPRINT, body))
}

/// Encode only the generated-C fixed-array `FrontierList` body.
pub fn encode_frontier_list_body(message: &FrontierList) -> CodecResult<Vec<u8>> {
    if message.frontiers_length > FRONTIER_LIST_CAPACITY {
        return Err(CodecError::LcmEncode(format!(
            "FrontierList frontiers_length {} exceeds fixed capacity {FRONTIER_LIST_CAPACITY}",
            message.frontiers_length
        )));
    }
    if message.frontiers.len() != FRONTIER_LIST_CAPACITY {
        return Err(CodecError::LcmEncode(format!(
            "FrontierList generated-C storage must contain exactly {FRONTIER_LIST_CAPACITY} entries, got {}",
            message.frontiers.len()
        )));
    }
    let frontiers_length = i32::try_from(message.frontiers_length)
        .map_err(|_| CodecError::LcmEncode("FrontierList length exceeds int32".into()))?;
    let mut out = Vec::new();
    out.extend_from_slice(&message.stamp_sec.to_be_bytes());
    out.extend_from_slice(&message.stamp_nsec.to_be_bytes());
    encode_string(&message.frame_id, &mut out)?;
    out.extend_from_slice(&frontiers_length.to_be_bytes());
    for frontier in &message.frontiers {
        encode_frontier_into(frontier, &mut out)?;
    }
    Ok(out)
}

/// Decode a top-level repository-native `nuwax_msgs__FrontierList` message.
pub fn decode_frontier_list(encoded: &[u8]) -> CodecResult<FrontierList> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(FRONTIER_LIST_FINGERPRINT, "nuwax_msgs__FrontierList")?;
    let message = decode_frontier_list_from(&mut cursor)?;
    cursor.finish("nuwax_msgs__FrontierList")?;
    Ok(message)
}

/// Decode only a generated-C fixed-array `FrontierList` body.
pub fn decode_frontier_list_body(encoded: &[u8]) -> CodecResult<FrontierList> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_frontier_list_from(&mut cursor)?;
    cursor.finish("nuwax_msgs__FrontierList body")?;
    Ok(message)
}

fn decode_frontier_list_from(cursor: &mut Cursor<'_>) -> CodecResult<FrontierList> {
    let stamp_sec = cursor.read_i64()?;
    let stamp_nsec = cursor.read_i32()?;
    let frame_id = cursor.read_string()?;
    let frontiers_length = cursor.read_nonnegative_len("frontiers_length")?;
    if frontiers_length > FRONTIER_LIST_CAPACITY {
        return Err(CodecError::LcmDecode(format!(
            "frontiers_length {frontiers_length} exceeds fixed capacity {FRONTIER_LIST_CAPACITY}"
        )));
    }
    let mut frontiers = Vec::with_capacity(FRONTIER_LIST_CAPACITY);
    for _ in 0..FRONTIER_LIST_CAPACITY {
        frontiers.push(decode_frontier_from(cursor)?);
    }
    Ok(FrontierList {
        stamp_sec,
        stamp_nsec,
        frame_id,
        frontiers_length,
        frontiers,
    })
}

fn with_fingerprint(fingerprint: [u8; 8], body: Vec<u8>) -> Vec<u8> {
    let mut out = Vec::with_capacity(8 + body.len());
    out.extend_from_slice(&fingerprint);
    out.extend_from_slice(&body);
    out
}

fn encode_map_metadata_body(message: &MapMetaData, out: &mut Vec<u8>) {
    out.extend_from_slice(&message.map_load_time_sec.to_be_bytes());
    out.extend_from_slice(&message.map_load_time_nsec.to_be_bytes());
    out.extend_from_slice(&message.resolution.to_be_bytes());
    out.extend_from_slice(&message.width.to_be_bytes());
    out.extend_from_slice(&message.height.to_be_bytes());
    encode_pose_body(&message.origin, out);
}

fn decode_map_metadata_body(cursor: &mut Cursor<'_>) -> CodecResult<MapMetaData> {
    Ok(MapMetaData {
        map_load_time_sec: cursor.read_i32()?,
        map_load_time_nsec: cursor.read_i32()?,
        resolution: cursor.read_f32()?,
        width: cursor.read_i32()?,
        height: cursor.read_i32()?,
        origin: decode_pose_body(cursor)?,
    })
}

fn encode_transform_stamped_body(message: &TransformStamped, out: &mut Vec<u8>) -> CodecResult<()> {
    encode_header(&message.header, out)?;
    encode_string(&message.child_frame_id, out)?;
    encode_f64_array(&message.transform.translation, out);
    encode_f64_array(&message.transform.rotation_xyzw, out);
    Ok(())
}

fn decode_transform_stamped_body(cursor: &mut Cursor<'_>) -> CodecResult<TransformStamped> {
    Ok(TransformStamped {
        header: decode_header(cursor)?,
        child_frame_id: cursor.read_string()?,
        transform: Transform {
            translation: cursor.read_f64_array()?,
            rotation_xyzw: cursor.read_f64_array()?,
        },
    })
}

fn encode_header(header: &Header, out: &mut Vec<u8>) -> CodecResult<()> {
    out.extend_from_slice(&header.seq.to_be_bytes());
    out.extend_from_slice(&header.stamp_sec.to_be_bytes());
    out.extend_from_slice(&header.stamp_nsec.to_be_bytes());
    encode_string(&header.frame_id, out)
}

fn decode_header(cursor: &mut Cursor<'_>) -> CodecResult<Header> {
    Ok(Header {
        seq: cursor.read_i32()?,
        stamp_sec: cursor.read_i32()?,
        stamp_nsec: cursor.read_i32()?,
        frame_id: cursor.read_string()?,
    })
}

fn encode_pose_body(pose: &Pose, out: &mut Vec<u8>) {
    encode_f64_array(&pose.position, out);
    encode_f64_array(&pose.orientation_xyzw, out);
}

fn decode_pose_body(cursor: &mut Cursor<'_>) -> CodecResult<Pose> {
    Ok(Pose {
        position: cursor.read_f64_array()?,
        orientation_xyzw: cursor.read_f64_array()?,
    })
}

fn encode_f64_array(values: &[f64], out: &mut Vec<u8>) {
    for value in values {
        out.extend_from_slice(&value.to_be_bytes());
    }
}

fn encode_string(value: &str, out: &mut Vec<u8>) -> CodecResult<()> {
    let encoded_len = value
        .len()
        .checked_add(1)
        .and_then(|len| u32::try_from(len).ok())
        .ok_or_else(|| CodecError::LcmEncode("LCM string exceeds uint32 length".into()))?;
    out.extend_from_slice(&encoded_len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    out.push(0);
    Ok(())
}

struct Cursor<'a> {
    input: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(input: &'a [u8]) -> Self {
        Self { input, offset: 0 }
    }

    fn read_exact(&mut self, len: usize) -> CodecResult<&'a [u8]> {
        let end = self.offset.checked_add(len).ok_or(CodecError::Incomplete {
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

    fn read_u8(&mut self) -> CodecResult<u8> {
        Ok(self.read_exact(1)?[0])
    }

    fn read_i32(&mut self) -> CodecResult<i32> {
        let bytes: [u8; 4] = self.read_exact(4)?.try_into().expect("four bytes");
        Ok(i32::from_be_bytes(bytes))
    }

    fn read_i64(&mut self) -> CodecResult<i64> {
        let bytes: [u8; 8] = self.read_exact(8)?.try_into().expect("eight bytes");
        Ok(i64::from_be_bytes(bytes))
    }

    fn read_u32(&mut self) -> CodecResult<u32> {
        let bytes: [u8; 4] = self.read_exact(4)?.try_into().expect("four bytes");
        Ok(u32::from_be_bytes(bytes))
    }

    fn read_nonnegative_len(&mut self, field: &str) -> CodecResult<usize> {
        let value = self.read_i32()?;
        usize::try_from(value).map_err(|_| {
            CodecError::LcmDecode(format!("{field} must be non-negative, got {value}"))
        })
    }

    fn read_f32(&mut self) -> CodecResult<f32> {
        let bytes: [u8; 4] = self.read_exact(4)?.try_into().expect("four bytes");
        Ok(f32::from_be_bytes(bytes))
    }

    fn read_f64(&mut self) -> CodecResult<f64> {
        let bytes: [u8; 8] = self.read_exact(8)?.try_into().expect("eight bytes");
        Ok(f64::from_be_bytes(bytes))
    }

    fn read_f64_array<const N: usize>(&mut self) -> CodecResult<[f64; N]> {
        let mut values = [0.0; N];
        for value in &mut values {
            let bytes: [u8; 8] = self.read_exact(8)?.try_into().expect("eight bytes");
            *value = f64::from_be_bytes(bytes);
        }
        Ok(values)
    }

    fn read_string(&mut self) -> CodecResult<String> {
        let encoded_len = usize::try_from(self.read_u32()?).expect("u32 fits usize");
        if encoded_len == 0 {
            return Err(CodecError::LcmDecode(
                "LCM string length must include a trailing NUL".into(),
            ));
        }
        let bytes = self.read_exact(encoded_len)?;
        if bytes.last() != Some(&0) {
            return Err(CodecError::LcmDecode(
                "LCM string is missing its trailing NUL".into(),
            ));
        }
        Ok(String::from_utf8_lossy(&bytes[..encoded_len - 1]).into_owned())
    }

    fn expect_fingerprint(&mut self, expected: [u8; 8], message_type: &str) -> CodecResult<()> {
        let actual = self.read_exact(8)?;
        if actual != expected {
            return Err(CodecError::LcmDecode(format!(
                "fingerprint mismatch for {message_type}: got {}, expected {}",
                hex(actual),
                hex(&expected)
            )));
        }
        Ok(())
    }

    fn remaining(&self) -> usize {
        self.input.len() - self.offset
    }

    fn finish(&self, message_type: &str) -> CodecResult<()> {
        if self.offset != self.input.len() {
            return Err(CodecError::LcmDecode(format!(
                "trailing bytes after {message_type}: {}",
                self.input.len() - self.offset
            )));
        }
        Ok(())
    }
}

fn hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write as _;
        let _ = write!(out, "{byte:02x}");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_pose_stamped_matches_locked_package_size_and_fingerprint() {
        let encoded = encode_pose_stamped(&PoseStamped::default()).unwrap();
        assert_eq!(encoded.len(), 81);
        assert_eq!(&encoded[..8], &POSE_STAMPED_FINGERPRINT);
        assert_eq!(
            decode_pose_stamped(&encoded).unwrap(),
            PoseStamped::default()
        );
    }

    #[test]
    fn point_stamped_and_bool_cover_top_level_and_body_apis() {
        let point = PointStamped {
            header: Header {
                seq: 1,
                stamp_sec: 2,
                stamp_nsec: 3,
                frame_id: "map".into(),
            },
            point: [4.0, 5.0, 6.0],
        };
        let point_body = encode_point_stamped_body(&point).unwrap();
        let point_encoded = encode_point_stamped(&point).unwrap();
        assert_eq!(&point_encoded[..8], &POINT_STAMPED_FINGERPRINT);
        assert_eq!(&point_encoded[8..], point_body);
        assert_eq!(decode_point_stamped_body(&point_body).unwrap(), point);
        assert_eq!(decode_point_stamped(&point_encoded).unwrap(), point);

        for data in [false, true] {
            let message = Bool { data };
            let body = encode_bool_body(&message);
            let encoded = encode_bool(&message);
            assert_eq!(&encoded[..8], &BOOL_FINGERPRINT);
            assert_eq!(&encoded[8..], body);
            assert_eq!(decode_bool_body(&body).unwrap(), message);
            assert_eq!(decode_bool(&encoded).unwrap(), message);
        }
        assert!(decode_bool_body(&[]).is_err());
        assert!(decode_bool_body(&[0, 1]).is_err());
    }

    #[test]
    fn path_round_trip_preserves_nested_pose_stamped_messages() {
        let message = Path {
            header: Header {
                seq: 1,
                stamp_sec: 2,
                stamp_nsec: 3,
                frame_id: "map".into(),
            },
            poses: vec![PoseStamped {
                header: Header {
                    seq: 4,
                    stamp_sec: 5,
                    stamp_nsec: 6,
                    frame_id: "map".into(),
                },
                pose: Pose {
                    position: [1.0, 2.0, 3.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        };
        let encoded = encode_path(&message).unwrap();
        assert_eq!(&encoded[..8], &PATH_FINGERPRINT);
        assert_eq!(decode_path(&encoded).unwrap(), message);
    }

    fn configured_frame_goal(frame_id: &str) -> GoalEnvelope {
        let boundary_id = "configured-frame-exploration:boundary".to_string();
        let boundary_points = vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let boundary_sha256 =
            goal_intent_boundary_sha256(frame_id, &boundary_id, &boundary_points).unwrap();
        GoalEnvelope {
            schema_version: NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: "configured-frame-goal".into(),
            owner_kind: NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            parent_request_id: "configured-frame-exploration".into(),
            goal_generation: 2,
            producer: "wavefront".into(),
            producer_epoch: 5,
            producer_runtime_generation: 7,
            committed_map_session_id: "configured-map-session".into(),
            committed_map_session_generation: 3,
            committed_map_revision: 9,
            committed_map_payload_sha256: [0x31; 32],
            committed_map_frame_id: frame_id.into(),
            source_timestamp_ns: 123_000_000_456,
            boundary_id,
            boundary_sha256,
            boundary_frame_id: frame_id.into(),
            boundary_points,
            motion_guard_generation: 11,
            motion_guard_boot_nonce: [0x5a; 16],
            motion_stop_generation: 4,
            goal: PoseStamped {
                header: Header {
                    seq: 42,
                    stamp_sec: 123,
                    stamp_nsec: 456,
                    frame_id: frame_id.into(),
                },
                pose: Pose {
                    position: [1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
            ..GoalEnvelope::default()
        }
    }

    fn configured_frame_path(goal: &GoalEnvelope, frame_id: &str) -> PathEnvelope {
        PathEnvelope {
            schema_version: NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: goal.session_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer: goal.producer.clone(),
            producer_epoch: goal.producer_epoch,
            producer_runtime_generation: goal.producer_runtime_generation,
            committed_map_session_id: goal.committed_map_session_id.clone(),
            committed_map_session_generation: goal.committed_map_session_generation,
            committed_map_revision: goal.committed_map_revision,
            committed_map_payload_sha256: goal.committed_map_payload_sha256,
            committed_map_frame_id: goal.committed_map_frame_id.clone(),
            boundary_id: goal.boundary_id.clone(),
            boundary_sha256: goal.boundary_sha256,
            motion_guard_generation: goal.motion_guard_generation,
            motion_guard_boot_nonce: goal.motion_guard_boot_nonce,
            motion_stop_generation: goal.motion_stop_generation,
            planner_runtime_generation: 8,
            path_revision: 1,
            source_timestamp_ns: 124_000_000_000,
            goal_payload_sha256: goal.payload_sha256,
            path: Path {
                header: Header {
                    frame_id: frame_id.into(),
                    ..Header::default()
                },
                poses: vec![PoseStamped {
                    header: Header {
                        frame_id: frame_id.into(),
                        ..Header::default()
                    },
                    pose: goal.goal.pose.clone(),
                }],
            },
            ..PathEnvelope::default()
        }
    }

    #[test]
    fn configured_navigation_frames_round_trip_and_bind_payload_hashes() {
        let mut goal_digests = Vec::new();
        let mut path_digests = Vec::new();

        for frame_id in ["map", "robot/map"] {
            let mut goal = configured_frame_goal(frame_id);
            seal_goal_envelope(&mut goal).unwrap();
            assert_eq!(
                goal.payload_sha256,
                goal_envelope_payload_sha256(&goal).unwrap()
            );

            let goal_body = encode_goal_envelope_body(&goal).unwrap();
            assert_eq!(decode_goal_envelope_body(&goal_body).unwrap(), goal);
            let encoded_goal = encode_goal_envelope(&goal).unwrap();
            assert_eq!(&encoded_goal[..8], &GOAL_ENVELOPE_FINGERPRINT);
            assert_eq!(decode_goal_envelope(&encoded_goal).unwrap(), goal);
            goal_digests.push(goal.payload_sha256);

            if frame_id == "robot/map" {
                let mut stale_goal_frame = goal.clone();
                stale_goal_frame.boundary_frame_id = "robot/odom".into();
                stale_goal_frame.goal.header.frame_id = "robot/odom".into();
                assert!(encode_goal_envelope(&stale_goal_frame)
                    .unwrap_err()
                    .to_string()
                    .contains("must match committed map frame_id"));
            }

            let mut path = configured_frame_path(&goal, frame_id);
            seal_path_envelope(&mut path).unwrap();
            assert_eq!(
                path.path_payload_sha256,
                path_envelope_payload_sha256(&path).unwrap()
            );

            let path_body = encode_path_envelope_body(&path).unwrap();
            assert_eq!(decode_path_envelope_body(&path_body).unwrap(), path);
            let encoded_path = encode_path_envelope(&path).unwrap();
            assert_eq!(&encoded_path[..8], &PATH_ENVELOPE_FINGERPRINT);
            assert_eq!(decode_path_envelope(&encoded_path).unwrap(), path);
            path_digests.push(path.path_payload_sha256);

            if frame_id == "robot/map" {
                let mut stale_path_frame = path.clone();
                stale_path_frame.path.header.frame_id = "robot/odom".into();
                stale_path_frame.path.poses[0].header.frame_id = "robot/odom".into();
                assert!(encode_path_envelope(&stale_path_frame)
                    .unwrap_err()
                    .to_string()
                    .contains("must match committed map frame_id"));
            }
        }

        assert_ne!(goal_digests[0], goal_digests[1]);
        assert_ne!(path_digests[0], path_digests[1]);
    }

    #[test]
    fn configured_public_goal_without_boundary_remains_valid() {
        let mut goal = configured_frame_goal("robot/map");
        goal.owner_kind = NAVIGATION_OWNER_KIND_PUBLIC_NAV;
        goal.parent_request_id = "configured-frame-public".into();
        goal.boundary_id.clear();
        goal.boundary_sha256 = [0; 32];
        goal.boundary_frame_id.clear();
        goal.boundary_points.clear();

        seal_goal_envelope(&mut goal).unwrap();
        let encoded = encode_goal_envelope(&goal).unwrap();
        assert_eq!(decode_goal_envelope(&encoded).unwrap(), goal);
    }

    #[test]
    fn configured_navigation_frames_reject_empty_mismatched_or_non_finite_geometry() {
        for invalid_frame_id in ["", " robot/map ", "/robot/map"] {
            let mut goal = configured_frame_goal(invalid_frame_id);
            assert!(seal_goal_envelope(&mut goal)
                .unwrap_err()
                .to_string()
                .contains("frame_id is invalid"));
        }

        let mut missing_boundary_frame = configured_frame_goal("robot/map");
        missing_boundary_frame.boundary_frame_id.clear();
        assert!(seal_goal_envelope(&mut missing_boundary_frame)
            .unwrap_err()
            .to_string()
            .contains("exploration boundary identity or frame is invalid"));

        let mut mismatched_boundary_frame = configured_frame_goal("robot/map");
        mismatched_boundary_frame.boundary_frame_id = "map".into();
        assert!(seal_goal_envelope(&mut mismatched_boundary_frame)
            .unwrap_err()
            .to_string()
            .contains("exploration boundary identity or frame is invalid"));

        let mut non_finite_goal = configured_frame_goal("robot/map");
        non_finite_goal.goal.pose.position[0] = f64::NAN;
        assert!(seal_goal_envelope(&mut non_finite_goal)
            .unwrap_err()
            .to_string()
            .contains("non-finite coordinate"));

        let mut non_finite_boundary = configured_frame_goal("robot/map");
        non_finite_boundary.boundary_points[0][0] = f64::INFINITY;
        assert!(seal_goal_envelope(&mut non_finite_boundary)
            .unwrap_err()
            .to_string()
            .contains("non-finite coordinate"));

        let mut goal = configured_frame_goal("robot/map");
        seal_goal_envelope(&mut goal).unwrap();

        let mut empty_path_frame = configured_frame_path(&goal, "");
        assert!(seal_path_envelope(&mut empty_path_frame)
            .unwrap_err()
            .to_string()
            .contains("path frame_id is invalid"));

        let mut mismatched_pose_frame = configured_frame_path(&goal, "robot/map");
        mismatched_pose_frame.path.poses[0].header.frame_id = "map".into();
        assert!(seal_path_envelope(&mut mismatched_pose_frame)
            .unwrap_err()
            .to_string()
            .contains("match the path frame_id"));

        let mut non_finite_path = configured_frame_path(&goal, "robot/map");
        non_finite_path.path.poses[0].pose.position[0] = f64::INFINITY;
        assert!(seal_path_envelope(&mut non_finite_path)
            .unwrap_err()
            .to_string()
            .contains("must be finite"));
    }

    #[test]
    fn versioned_envelopes_round_trip_and_reject_identity_or_hash_mutation() {
        let pose = PoseStamped {
            header: Header {
                seq: 42,
                stamp_sec: 123,
                stamp_nsec: 456,
                frame_id: "map".into(),
            },
            pose: Pose {
                position: [1.0, 2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        };
        let mut goal = configured_frame_goal("map");
        goal.goal = pose.clone();
        seal_goal_envelope(&mut goal).unwrap();
        let encoded = encode_goal_envelope(&goal).unwrap();
        assert_eq!(&encoded[..8], &GOAL_ENVELOPE_FINGERPRINT);
        assert_eq!(decode_goal_envelope(&encoded).unwrap(), goal);

        let mut mutated_goal = goal.clone();
        mutated_goal.boundary_points[1][0] = 3.0;
        assert!(encode_goal_envelope(&mutated_goal)
            .unwrap_err()
            .to_string()
            .contains("boundary_sha256 mismatch"));

        let mut mutated_proof = goal.clone();
        mutated_proof.motion_stop_generation += 1;
        assert!(encode_goal_envelope(&mutated_proof)
            .unwrap_err()
            .to_string()
            .contains("payload_sha256 mismatch"));

        let mut missing_proof = goal.clone();
        missing_proof.motion_guard_boot_nonce = [0; 16];
        assert!(seal_goal_envelope(&mut missing_proof)
            .unwrap_err()
            .to_string()
            .contains("must not be all zero"));

        let mut public_with_boundary = goal.clone();
        public_with_boundary.owner_kind = NAVIGATION_OWNER_KIND_PUBLIC_NAV;
        assert!(seal_goal_envelope(&mut public_with_boundary)
            .unwrap_err()
            .to_string()
            .contains("must not carry a navigation boundary"));

        let mut path = PathEnvelope {
            schema_version: NAVIGATION_ENVELOPE_SCHEMA_VERSION,
            session_id: goal.session_id.clone(),
            owner_kind: goal.owner_kind,
            parent_request_id: goal.parent_request_id.clone(),
            goal_generation: goal.goal_generation,
            producer: goal.producer.clone(),
            producer_epoch: goal.producer_epoch,
            producer_runtime_generation: goal.producer_runtime_generation,
            committed_map_session_id: goal.committed_map_session_id.clone(),
            committed_map_session_generation: goal.committed_map_session_generation,
            committed_map_revision: goal.committed_map_revision,
            committed_map_payload_sha256: goal.committed_map_payload_sha256,
            committed_map_frame_id: goal.committed_map_frame_id.clone(),
            boundary_id: goal.boundary_id.clone(),
            boundary_sha256: goal.boundary_sha256,
            motion_guard_generation: goal.motion_guard_generation,
            motion_guard_boot_nonce: goal.motion_guard_boot_nonce,
            motion_stop_generation: goal.motion_stop_generation,
            planner_runtime_generation: 8,
            path_revision: 1,
            source_timestamp_ns: 124_000_000_000,
            goal_payload_sha256: goal.payload_sha256,
            path: Path {
                header: pose.header.clone(),
                poses: vec![pose],
            },
            ..PathEnvelope::default()
        };
        seal_path_envelope(&mut path).unwrap();
        let encoded = encode_path_envelope(&path).unwrap();
        assert_eq!(&encoded[..8], &PATH_ENVELOPE_FINGERPRINT);
        assert_eq!(decode_path_envelope(&encoded).unwrap(), path);

        let mut mutated_path = path.clone();
        mutated_path.source_timestamp_ns += 1;
        assert!(encode_path_envelope(&mutated_path)
            .unwrap_err()
            .to_string()
            .contains("payload_sha256 mismatch"));
    }

    #[test]
    fn odometry_round_trip_preserves_covariance_and_frames() {
        let mut pose_covariance = [0.0; 36];
        pose_covariance[35] = 4.5;
        let mut twist_covariance = [0.0; 36];
        twist_covariance[0] = 1.25;
        let message = Odometry {
            header: Header {
                seq: 7,
                stamp_sec: 12,
                stamp_nsec: 345,
                frame_id: "odom".into(),
            },
            child_frame_id: "base_link".into(),
            pose: Pose {
                position: [1.0, 2.0, 3.0],
                orientation_xyzw: [0.1, 0.2, 0.3, 0.9],
            },
            pose_covariance,
            twist_linear: [0.4, 0.5, 0.6],
            twist_angular: [0.7, 0.8, 0.9],
            twist_covariance,
        };

        let encoded = encode_odometry(&message).unwrap();
        assert_eq!(&encoded[..8], &ODOMETRY_FINGERPRINT);
        assert_eq!(decode_odometry(&encoded).unwrap(), message);
    }

    #[test]
    fn point_cloud_round_trip_preserves_dynamic_arrays() {
        let message = PointCloud2 {
            header: Header {
                seq: 1,
                stamp_sec: 2,
                stamp_nsec: 3,
                frame_id: "lidar".into(),
            },
            height: 1,
            width: 2,
            fields: vec![
                PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                },
                PointField {
                    name: "intensity".into(),
                    offset: 12,
                    datatype: 7,
                    count: 1,
                },
            ],
            is_bigendian: false,
            point_step: 16,
            row_step: 32,
            data: (0_u8..32).collect(),
            is_dense: true,
        };

        let encoded = encode_point_cloud2(&message).unwrap();
        assert_eq!(&encoded[..8], &POINT_CLOUD2_FINGERPRINT);
        assert_eq!(decode_point_cloud2(&encoded).unwrap(), message);
    }

    #[test]
    fn point_cloud_rejects_impossible_field_count_before_allocation() {
        let mut encoded = encode_point_cloud2(&PointCloud2::default()).unwrap();
        encoded[8..12].copy_from_slice(&i32::MAX.to_be_bytes());
        let error = decode_point_cloud2(&encoded).unwrap_err();
        assert!(
            error
                .to_string()
                .contains("fields_length 2147483647 exceeds remaining"),
            "unexpected error: {error}"
        );
    }

    #[test]
    fn twist_matches_locked_fingerprint_and_six_f64_body() {
        let message = Twist {
            linear: [1.0, 2.0, 3.0],
            angular: [4.0, 5.0, 6.0],
        };
        let body = encode_twist_body(&message);
        assert_eq!(body.len(), 48);
        let encoded = encode_twist(&message);
        assert_eq!(encoded.len(), 56);
        assert_eq!(&encoded[..8], &TWIST_FINGERPRINT);
        assert_eq!(decode_twist(&encoded).unwrap(), message);
    }

    #[test]
    fn occupancy_grid_round_trip_covers_body_and_rejects_negative_length() {
        let message = OccupancyGrid {
            header: Header {
                seq: 1,
                stamp_sec: 2,
                stamp_nsec: 3,
                frame_id: "map".into(),
            },
            info: MapMetaData {
                map_load_time_sec: 4,
                map_load_time_nsec: 5,
                resolution: 0.05,
                width: 2,
                height: 2,
                origin: Pose {
                    position: [-1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
            data: vec![-1, 0, 50, 100],
        };
        let body = encode_occupancy_grid_body(&message).unwrap();
        let encoded = encode_occupancy_grid(&message).unwrap();
        assert_eq!(&encoded[..8], &OCCUPANCY_GRID_FINGERPRINT);
        assert_eq!(&encoded[8..], body);
        assert_eq!(decode_occupancy_grid_body(&body).unwrap(), message);
        assert_eq!(decode_occupancy_grid(&encoded).unwrap(), message);

        let mut negative_length = body;
        negative_length[..4].copy_from_slice(&(-1_i32).to_be_bytes());
        let error = decode_occupancy_grid_body(&negative_length).unwrap_err();
        assert!(error
            .to_string()
            .contains("data_length must be non-negative"));
    }

    #[test]
    fn tf_message_round_trip_covers_nested_layout_and_length_bounds() {
        let message = TfMessage {
            transforms: vec![TransformStamped {
                header: Header {
                    seq: 8,
                    stamp_sec: 13,
                    stamp_nsec: 21,
                    frame_id: "map".into(),
                },
                child_frame_id: "base_link".into(),
                transform: Transform {
                    translation: [1.0, 2.0, 3.0],
                    rotation_xyzw: [0.1, 0.2, 0.3, 0.9],
                },
            }],
        };
        let body = encode_tf_message_body(&message).unwrap();
        let encoded = encode_tf_message(&message).unwrap();
        assert_eq!(&encoded[..8], &TF_MESSAGE_FINGERPRINT);
        assert_eq!(&encoded[8..], body);
        assert_eq!(decode_tf_message_body(&body).unwrap(), message);
        assert_eq!(decode_tf_message(&encoded).unwrap(), message);

        let mut negative_length = body.clone();
        negative_length[..4].copy_from_slice(&(-1_i32).to_be_bytes());
        assert!(decode_tf_message_body(&negative_length)
            .unwrap_err()
            .to_string()
            .contains("transforms_length must be non-negative"));

        let mut impossible_length = body;
        impossible_length[..4].copy_from_slice(&100_i32.to_be_bytes());
        assert!(decode_tf_message_body(&impossible_length)
            .unwrap_err()
            .to_string()
            .contains("exceeds remaining TFMessage bytes"));
    }

    #[test]
    fn native_bridge_single_messages_round_trip_with_generated_fingerprints() {
        let transform = Transform {
            translation: [1.0, 2.0, 3.0],
            rotation_xyzw: [0.1, 0.2, 0.3, 0.9],
        };
        let encoded = encode_geometry_transform(&transform);
        assert_eq!(&encoded[..8], &GEOMETRY_TRANSFORM_FINGERPRINT);
        assert_eq!(decode_geometry_transform(&encoded).unwrap(), transform);
        assert_eq!(
            decode_geometry_transform_body(&encode_geometry_transform_body(&transform)).unwrap(),
            transform
        );

        let navigation = NavigationState {
            session_id: "session-1".into(),
            robot_frame: "base_link".into(),
            stamp_sec: 123,
            stamp_nsec: 456,
            state: 2,
            goal: [1.0, 2.0, 0.5],
            current: [0.5, 1.0, 0.25],
            progress: 0.75,
            message: "moving".into(),
            runtime_generation: 7,
            guard_generation: 11,
            sink_ready: 1,
            planner_map_frame: "map".into(),
            planner_pose_frame: "map".into(),
            cancel_generation: 3,
            zero_ack_id: "zero-3".into(),
            zero_applied_monotonic_ns: 987_654_321,
            owner_kind: NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "request-1".into(),
            goal_generation: 5,
            path_revision: 2,
            goal_payload_sha256: [0x11; 32],
            path_payload_sha256: [0x22; 32],
        };
        let encoded = encode_navigation_state(&navigation).unwrap();
        assert_eq!(&encoded[..8], &NAVIGATION_STATE_FINGERPRINT);
        assert_eq!(decode_navigation_state(&encoded).unwrap(), navigation);
        assert_eq!(
            decode_navigation_state_body(&encode_navigation_state_body(&navigation).unwrap())
                .unwrap(),
            navigation
        );

        let cancel = CancelNavigation {
            session_id: "session-1".into(),
            reason: 2,
            cancel_generation: 3,
            owner_kind: NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "request-1".into(),
            goal_generation: 5,
            producer_runtime_generation: 9,
        };
        let encoded = encode_cancel_navigation(&cancel).unwrap();
        assert_eq!(&encoded[..8], &CANCEL_NAVIGATION_FINGERPRINT);
        assert_eq!(decode_cancel_navigation(&encoded).unwrap(), cancel);
        assert_eq!(
            decode_cancel_navigation_body(&encode_cancel_navigation_body(&cancel).unwrap())
                .unwrap(),
            cancel
        );

        let frontier = Frontier {
            id: 9,
            center: [3.0, 4.0],
            radius: 1.25,
            score: 8.5,
            cell_count: 12,
            note: "north".into(),
        };
        let encoded = encode_frontier(&frontier).unwrap();
        assert_eq!(&encoded[..8], &FRONTIER_FINGERPRINT);
        assert_eq!(decode_frontier(&encoded).unwrap(), frontier);
        assert_eq!(
            decode_frontier_body(&encode_frontier_body(&frontier).unwrap()).unwrap(),
            frontier
        );
    }

    #[test]
    fn legacy_cancel_body_is_a_deterministic_truncation_not_a_valid_additive_message() {
        let cancel = CancelNavigation {
            session_id: "session-legacy".into(),
            reason: 2,
            cancel_generation: 3,
            owner_kind: NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "request-legacy".into(),
            goal_generation: 5,
            producer_runtime_generation: 9,
        };
        let encoded = encode_cancel_navigation_body(&cancel).expect("encode current cancellation");
        assert_eq!(&encoded[encoded.len() - 8..], &9_i64.to_be_bytes());
        let legacy = &encoded[..encoded.len() - 8];
        let first = decode_cancel_navigation_body(legacy)
            .expect_err("legacy body lacks producer generation")
            .to_string();
        let second = decode_cancel_navigation_body(legacy)
            .expect_err("legacy body remains deterministically invalid")
            .to_string();
        assert_eq!(first, second);
    }

    #[test]
    fn native_frontier_list_preserves_all_fixed_slots_and_validates_logical_length() {
        let mut message = FrontierList {
            stamp_sec: 10,
            stamp_nsec: 20,
            frame_id: "map".into(),
            frontiers_length: 2,
            ..FrontierList::default()
        };
        message.frontiers[0] = Frontier {
            id: 1,
            center: [2.0, 3.0],
            radius: 0.5,
            score: 4.0,
            cell_count: 5,
            note: "a".into(),
        };
        message.frontiers[1] = Frontier {
            id: 6,
            center: [7.0, 8.0],
            radius: 1.5,
            score: 9.0,
            cell_count: 10,
            note: "b".into(),
        };
        message.frontiers[255].note = "serialized-unused-slot".into();

        let body = encode_frontier_list_body(&message).unwrap();
        let encoded = encode_frontier_list(&message).unwrap();
        assert_eq!(&encoded[..8], &FRONTIER_LIST_FINGERPRINT);
        assert_eq!(&encoded[8..], body);
        assert_eq!(decode_frontier_list_body(&body).unwrap(), message);
        assert_eq!(decode_frontier_list(&encoded).unwrap(), message);

        let mut negative_length = body.clone();
        negative_length[20..24].copy_from_slice(&(-1_i32).to_be_bytes());
        assert!(decode_frontier_list_body(&negative_length)
            .unwrap_err()
            .to_string()
            .contains("frontiers_length must be non-negative"));

        let mut oversized_length = body;
        oversized_length[20..24].copy_from_slice(&257_i32.to_be_bytes());
        assert!(decode_frontier_list_body(&oversized_length)
            .unwrap_err()
            .to_string()
            .contains("exceeds fixed capacity"));

        let mut invalid_storage = message;
        invalid_storage.frontiers.pop();
        assert!(encode_frontier_list(&invalid_storage)
            .unwrap_err()
            .to_string()
            .contains("exactly 256 entries"));
    }
}
