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
pub const GOAL_ENVELOPE_FINGERPRINT: [u8; 8] = 0xec74_63f7_558a_ac72_u64.to_be_bytes();
/// `nuwax_msgs.PathEnvelope` generated-C recursive fingerprint.
pub const PATH_ENVELOPE_FINGERPRINT: [u8; 8] = 0xeec1_8428_127b_3f5a_u64.to_be_bytes();
/// `nav_msgs.OccupancyGrid` fingerprint from `dimos-lcm==0.1.3`.
pub const OCCUPANCY_GRID_FINGERPRINT: [u8; 8] = 0xe7df_d179_cdfc_3b65_u64.to_be_bytes();
/// `nav_msgs.Odometry` fingerprint from `dimos-lcm==0.1.3`.
pub const ODOMETRY_FINGERPRINT: [u8; 8] = 0x94e1_7f94_648c_f0e4_u64.to_be_bytes();
/// `sensor_msgs.PointCloud2` fingerprint from `dimos-lcm==0.1.3`.
pub const POINT_CLOUD2_FINGERPRINT: [u8; 8] = 0xf5eb_3da1_c285_3175_u64.to_be_bytes();
/// Verified historical v2 fingerprint; never used to encode the C1 v4 layout.
pub const NAVIGATION_MAP_3D_V2_FINGERPRINT: [u8; 8] = 0xd11d6dd10d5490b8_u64.to_be_bytes();
/// C1 v4 recursive fingerprint from the checked-in `lcm-gen 1.5.2` bindings.
pub const NAVIGATION_MAP_3D_FINGERPRINT: [u8; 8] = 0x131a_5884_d2d6_8d28_u64.to_be_bytes();
/// Tagged immutable-object block fingerprint from `lcm-gen 1.5.2`.
pub const NAVIGATION_MAP_BLOCK_FINGERPRINT: [u8; 8] = 0xe544_975d_2a8d_dc13_u64.to_be_bytes();
/// C1 object/revision wire layout, distinct from the old digest-addressed v2 body.
pub const NAVIGATION_MAP_3D_SCHEMA_VERSION: i32 = 4;
const NAVIGATION_MAP_BLOCK_KIND_ASSIGNED: u8 = 1;
const NAVIGATION_MAP_BLOCK_KIND_LEGACY: u8 = 2;
const NAVIGATION_MAP_MAX_INCLUDED_BLOCKS: usize = 16_384;
const NAVIGATION_MAP_MAX_INCLUDED_BYTES: usize = 512 * 1024;
/// `sensor_msgs.Imu` fingerprint from the canonical DimOS nested Quaternion/Vector3 schema.
pub const IMU_FINGERPRINT: [u8; 8] = 0x31ab_205c_8dd5_7aa8_u64.to_be_bytes();
/// `tf2_msgs.TFMessage` fingerprint from `dimos-lcm==0.1.3`.
pub const TF_MESSAGE_FINGERPRINT: [u8; 8] = 0xc2b8_a1c3_3a89_23ec_u64.to_be_bytes();

/// `geometry_msgs.Transform` fingerprint generated from the repository IDL.
pub const GEOMETRY_TRANSFORM_FINGERPRINT: [u8; 8] = 0xee5f_f426_47bf_3f91_u64.to_be_bytes();
/// `nuwax_msgs.NavigationState` generated-C fingerprint.
pub const NAVIGATION_STATE_FINGERPRINT: [u8; 8] = 0x9ea3_f3f5_fd5a_fbda_u64.to_be_bytes();
/// `nuwax_msgs.CancelNavigation` generated-C fingerprint.
pub const CANCEL_NAVIGATION_FINGERPRINT: [u8; 8] = 0x9704_4b28_e9c5_dd22_u64.to_be_bytes();
/// `nuwax_msgs.Frontier` generated-C fingerprint.
pub const FRONTIER_FINGERPRINT: [u8; 8] = 0x1abf_83ea_0ced_9c44_u64.to_be_bytes();
/// `nuwax_msgs.FrontierList` generated-C recursive fingerprint.
pub const FRONTIER_LIST_FINGERPRINT: [u8; 8] = 0x213d_b68f_b817_1f76_u64.to_be_bytes();
/// The checked-in `FrontierList.lcm` declares a fixed array of this size.
pub const FRONTIER_LIST_CAPACITY: usize = 256;
/// Current immutable navigation-envelope schema.
pub const NAVIGATION_ENVELOPE_SCHEMA_VERSION: i32 = 7;
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
    /// 真实占据提交回执的进程启动域；不能跨域排序。
    pub commit_boot_nonce: [u8; 16],
    /// 真实占据提交序号。
    pub commit_sequence: i64,
    /// 真实占据提交来源时间，纳秒。
    pub source_timestamp_ns: i64,
    /// 已提交快照携带的规范规划坐标系。
    pub frame_id: String,
}

/// 由唯一导航租约 owner 准入的共享 producer 请求。
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GoalIntent {
    /// 目标来源的坐标代际，普通地图修订前进不能改变它。
    pub map_frame_generation: i64,
    /// 随原目标封存的倒车与到达约束。
    pub options: nuwax_types::navigation::GoalOptions,
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
    goal_intent_action_sha256(goal, &nuwax_types::navigation::GoalOptions::default())
}

/// 目标位姿与执行约束共同决定请求摘要。
pub fn goal_intent_action_sha256(goal: &PoseStamped, options: &nuwax_types::navigation::GoalOptions) -> CodecResult<[u8;32]> {
    options.validate().map_err(CodecError::LcmEncode)?;
    let mut canonical=encode_pose_stamped_body(goal)?;
    encode_goal_options(options,&mut canonical);
    Ok(crate::hash::sha256_bytes(&canonical))
}
fn encode_goal_options(options:&nuwax_types::navigation::GoalOptions,out:&mut Vec<u8>) {
    out.push(u8::from(options.allow_reverse));
    out.extend_from_slice(&options.position_tolerance_m.to_be_bytes());
    out.extend_from_slice(&options.yaw_tolerance_rad.to_be_bytes());
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
    /// 随目标摘要封存的地图坐标代际。
    pub map_frame_generation: i64,
    /// 几何与控制组件必须一致的启动参数摘要。
    pub navigation_configuration_sha256: [u8;32],
    /// 随目标载荷摘要封存的执行约束。
    pub options: nuwax_types::navigation::GoalOptions,
    /// 当前线格式版本为7。
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
    /// Occupancy commit boot domain copied from the actual receipt.
    pub committed_map_boot_nonce: [u8; 16],
    /// Actual occupancy commit sequence.
    pub committed_map_commit_sequence: i64,
    /// Actual occupancy commit source timestamp, in nanoseconds.
    pub committed_map_source_timestamp_ns: i64,
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
    /// 当前线格式版本为7。
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
    /// Occupancy commit boot domain copied from the actual receipt.
    pub committed_map_boot_nonce: [u8; 16],
    /// Actual occupancy commit sequence.
    pub committed_map_commit_sequence: i64,
    /// Actual occupancy commit source timestamp, in nanoseconds.
    pub committed_map_source_timestamp_ns: i64,
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
    /// Source timestamp of the latest scheduler-consumed pose.
    pub pose_source_timestamp_ns: i64,
    /// Source timestamp of the latest scheduler-consumed costmap.
    pub costmap_source_timestamp_ns: i64,
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
    /// 同一目标的局部规划展示点，不授予运动权限。
    pub local_path: Vec<[f64; 3]>,

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

/// 复用规范地图身份与真实点云提交回执的三维导航地图。
#[derive(Debug, Clone, PartialEq)]
pub struct NavigationMap3D {
    /// 当前消息版本，固定为4；旧摘要寻址布局不兼容。
    pub schema_version: i32,
    /// 复用导航生产者的持久运行代际，生效与清空消息均必须为正。
    pub producer_runtime_generation: i64,
    /// 当前规范占据地图身份；线格式展开各字段。
    pub committed_map: CommittedMapIdentity,
    /// 当前地图坐标代际，初始代际可为0。
    pub map_frame_generation: i64,
    /// 已提交点云修订，对应内部云头序号。
    pub cloud_revision: i64,
    /// 点云回执保留的占据修订基线；可落后于当前规范占据修订。
    pub cloud_occupancy_revision: i64,
    /// 原始观测时间，纳秒。
    pub source_timestamp_ns: i64,
    /// 来源点云的体素大小，单位米。
    pub source_voxel_size: f32,
    /// 点云提交时钟的进程随机标识。
    pub commit_boot_nonce: [u8; 16],
    /// 真实点云提交序号。
    pub commit_sequence: i64,
    /// 明确清空三维地图，清空时内部点云必须为空。
    pub cleared: bool,
    /// 同次占据提交中的纯虚拟禁区；不包含二维物理投影。
    pub virtual_obstacles: Option<OccupancyGrid>,
    /// 固定精度完整根；有该字段时不另携带全量云。
    pub fine: Option<nuwax_map_chunks::Transfer>,
    /// 位于规范地图坐标系的三维点云。
    pub cloud: PointCloud2,
}

impl Default for NavigationMap3D {
    fn default() -> Self {
        Self {
            schema_version: NAVIGATION_MAP_3D_SCHEMA_VERSION,
            producer_runtime_generation: 0,
            committed_map: CommittedMapIdentity::default(),
            map_frame_generation: 0,
            cloud_revision: 0,
            cloud_occupancy_revision: 0,
            source_timestamp_ns: 0,
            source_voxel_size: 0.0,
            commit_boot_nonce: [0; 16],
            commit_sequence: 0,
            cleared: false,
            virtual_obstacles: None,
            fine: None,
            cloud: PointCloud2::default(),
        }
    }
}

/// 校验三维导航地图身份、内部云头及有限三维点布局。
pub fn validate_navigation_map_3d(message: &NavigationMap3D) -> Result<(), String> {
    if message.schema_version != NAVIGATION_MAP_3D_SCHEMA_VERSION {
        return Err("unsupported NavigationMap3D schema_version".into());
    }
    if message.producer_runtime_generation <= 0 {
        return Err("NavigationMap3D producer_runtime_generation must be positive".into());
    }
    let map = &message.committed_map;
    validate_navigation_frame_id(&map.frame_id, "NavigationMap3D")?;
    if map.session_id.trim() != map.session_id || map.session_id.contains('\0') {
        return Err("NavigationMap3D map_session_id must be canonical and NUL-free".into());
    }
    if message.map_frame_generation < 0 || map.session_generation < 0 || map.revision < 0 {
        return Err(
            "NavigationMap3D map generations and occupancy revision must be non-negative".into(),
        );
    }
    if message.cloud_revision <= 0 || message.cloud_revision > i64::from(i32::MAX) {
        return Err(
            "NavigationMap3D cloud_revision must fit a positive cloud header sequence".into(),
        );
    }
    if !message.source_voxel_size.is_finite() || message.source_voxel_size <= 0.0 {
        return Err("NavigationMap3D source_voxel_size must be positive and finite".into());
    }
    if message.commit_boot_nonce == [0; 16]
        || message.commit_sequence < 0
        || message.source_timestamp_ns < 0
    {
        return Err("NavigationMap3D commit receipt and source timestamp are invalid".into());
    }
    let occupancy_absent = map.revision == 0
        && map.commit_boot_nonce == [0; 16]
        && map.commit_sequence == 0
        && map.source_timestamp_ns == 0;
    let occupancy_complete = !map.session_id.is_empty()
        && map.session_generation > 0
        && map.revision > 0
        && map.commit_boot_nonce != [0; 16]
        && map.commit_sequence > 0
        && map.source_timestamp_ns >= 0;
    if message.cleared {
        // A clear before the first occupancy commit is an explicit no-map
        // variant. These zero fields never constitute a committed identity.
        if (!occupancy_absent && !occupancy_complete)
            || (map.session_id.is_empty() && map.session_generation != 0)
        {
            return Err("cleared NavigationMap3D must carry either absent or complete occupancy provenance".into());
        }
    } else if !occupancy_complete
        || message.commit_sequence == 0
        || message.source_timestamp_ns == 0
        || message.cloud_occupancy_revision <= 0
    {
        return Err("live NavigationMap3D requires complete committed map and cloud receipt".into());
    }
    if message.cloud_occupancy_revision < 0 {
        return Err("NavigationMap3D cloud occupancy baseline must be non-negative".into());
    }
    if occupancy_absent {
        if message.cloud_occupancy_revision != 0 {
            return Err("absent occupancy provenance cannot carry a cloud occupancy baseline".into());
        }
    } else {
        // Both receipts share the single session/frame domain carried by this
        // envelope. Nonces are opaque domains, never ordered integers. Numeric
        // ordering is only a consistency check: the producer must prove lineage.
        if map.commit_boot_nonce != message.commit_boot_nonce
            || message.cloud_occupancy_revision > map.revision
            || message.commit_sequence > map.commit_sequence
        {
            return Err("NavigationMap3D cloud receipt must belong to the current occupancy domain and baseline".into());
        }
        if (message.commit_sequence == map.commit_sequence
            || message.cloud_occupancy_revision == map.revision)
            && (message.commit_sequence != map.commit_sequence
                || message.cloud_occupancy_revision != map.revision
                || message.source_timestamp_ns != map.source_timestamp_ns)
        {
            return Err("equal NavigationMap3D occupancy/cloud commits require the same revision, sequence and source timestamp".into());
        }
    }
    let header = &message.cloud.header;
    if header.frame_id != map.frame_id || i64::from(header.seq) != message.cloud_revision {
        return Err("NavigationMap3D cloud header frame and sequence must match envelope".into());
    }
    if header.stamp_sec < 0
        || !(0..1_000_000_000).contains(&header.stamp_nsec)
        || i64::from(header.stamp_sec) * 1_000_000_000 + i64::from(header.stamp_nsec)
            != message.source_timestamp_ns
    {
        return Err("NavigationMap3D cloud header timestamp must match source_timestamp_ns".into());
    }
    if let Some(mask) = message.virtual_obstacles.as_ref() {
        if message.cleared {
            return Err("cleared NavigationMap3D cannot contain virtual obstacles".into());
        }
        if mask.header.frame_id != map.frame_id
            || i64::from(mask.header.seq) != map.revision
            || mask.header.stamp_sec < 0
            || !(0..1_000_000_000).contains(&mask.header.stamp_nsec)
            || i64::from(mask.header.stamp_sec) * 1_000_000_000
                + i64::from(mask.header.stamp_nsec) != map.source_timestamp_ns
        {
            return Err(
                "NavigationMap3D virtual obstacle header must match occupancy commit".into(),
            );
        }
        let info = &mask.info;
        let cells = usize::try_from(info.width).ok().and_then(|width| {
            usize::try_from(info.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        });
        if info.width <= 0
            || info.height <= 0
            || !info.resolution.is_finite()
            || info.resolution <= 0.0
            || !pose_is_finite(&info.origin)
            || info.origin.position[2] != 0.0
            || info.origin.orientation_xyzw != [0.0, 0.0, 0.0, 1.0]
            || cells != Some(mask.data.len())
            || mask.data.iter().any(|value| !matches!(value, 0 | 100))
            || info.map_load_time_sec < 0
            || !(0..1_000_000_000).contains(&info.map_load_time_nsec)
        {
            return Err("NavigationMap3D virtual obstacles require finite axis-aligned XY geometry and 0/100 cells".into());
        }
    }
    if let Some(fine) = &message.fine {
        let root = nuwax_map_chunks::Manifest::decode(&fine.root)?;
        if message.cleared || root.domain != navigation_map_3d_domain(message)
            || root.revision != message.cloud_revision as u64
            || root.source_timestamp_ns != message.source_timestamp_ns as u64
            || root.voxel_size_m != message.source_voxel_size || root.blocks.is_empty() {
            return Err("精细根与三维规范提交不一致".into());
        }
        if fine.included.len() > NAVIGATION_MAP_MAX_INCLUDED_BLOCKS {
            return Err("携带精细块数量超预算".into());
        }
        let blocks = root.blocks.iter().map(|b| (b.object, b)).collect::<std::collections::BTreeMap<_, _>>();
        let mut total = 0_usize;
        for (object, bytes) in &fine.included {
            total = total.checked_add(bytes.len()).ok_or("精细块大小溢出")?;
            if total > NAVIGATION_MAP_MAX_INCLUDED_BYTES { return Err("携带精细块超预算".into()); }
            let reference = blocks.get(object).ok_or("携带精细块不属于根清单")?;
            let decoded = nuwax_map_chunks::decode_block(bytes)?;
            if decoded.len() != reference.points as usize || decoded.values().any(|e| e.observed_ns > root.source_timestamp_ns) {
                return Err("携带精细块证据与根不符".into());
            }
        }
    }
    validate_navigation_map_3d_cloud(&message.cloud, message.cleared || message.fine.is_some())
}

/// 分块根直接复用同一消息的地图域。
pub fn navigation_map_3d_domain(message: &NavigationMap3D) -> nuwax_map_chunks::Domain {
    nuwax_map_chunks::Domain { session: message.committed_map.session_id.clone(),
        generation: message.committed_map.session_generation as u64,
        frame_generation: message.map_frame_generation as u64,
        producer_generation: message.producer_runtime_generation as u64 }
}

fn validate_navigation_map_3d_cloud(cloud: &PointCloud2, cleared: bool) -> Result<(), String> {
    if cleared {
        if cloud.width != 0
            || !(0..=1).contains(&cloud.height)
            || cloud.row_step != 0
            || cloud.point_step < 0
            || !cloud.data.is_empty()
        {
            return Err("cleared NavigationMap3D must contain an explicit empty cloud".into());
        }
        if cloud.fields.is_empty() {
            return Ok(());
        }
    } else if cloud.width <= 0 || cloud.height <= 0 || cloud.point_step <= 0 {
        return Err("live NavigationMap3D cloud dimensions and point_step must be positive".into());
    }
    let packed_row = cloud
        .width
        .checked_mul(cloud.point_step)
        .ok_or_else(|| "NavigationMap3D packed row overflow".to_string())?;
    if cloud.row_step < packed_row || cloud.row_step < 0 {
        return Err("NavigationMap3D row_step is smaller than packed point data".into());
    }
    let data_len = usize::try_from(cloud.height)
        .ok()
        .and_then(|height| {
            usize::try_from(cloud.row_step)
                .ok()
                .and_then(|row| height.checked_mul(row))
        })
        .ok_or_else(|| "NavigationMap3D cloud data length overflow".to_string())?;
    if data_len != cloud.data.len() {
        return Err("NavigationMap3D cloud data length must match height*row_step".into());
    }
    let mut names = std::collections::BTreeSet::new();
    let mut axes = [None; 3];
    for field in &cloud.fields {
        let bytes = match field.datatype {
            1 | 2 => 1,
            3 | 4 => 2,
            5..=7 => 4,
            8 => 8,
            _ => return Err("NavigationMap3D cloud has an unknown field datatype".into()),
        };
        if field.name.is_empty()
            || field.name.trim() != field.name
            || field.name.contains('\0')
            || !names.insert(field.name.as_str())
            || field.offset < 0
            || field.count <= 0
        {
            return Err("NavigationMap3D cloud field name, count or offset is invalid".into());
        }
        let end = field
            .count
            .checked_mul(bytes)
            .and_then(|count| field.offset.checked_add(count))
            .ok_or_else(|| "NavigationMap3D cloud field layout overflow".to_string())?;
        if end > cloud.point_step {
            return Err("NavigationMap3D cloud field exceeds point_step".into());
        }
        if let Some(axis) = ["x", "y", "z"].iter().position(|name| *name == field.name) {
            if field.count != 1 || !matches!(field.datatype, 7 | 8) {
                return Err("NavigationMap3D XYZ fields must be scalar float32 or float64".into());
            }
            axes[axis] = Some((field.offset as usize, bytes as usize));
        }
    }
    let [Some(x), Some(y), Some(z)] = axes else {
        return Err("NavigationMap3D cloud requires unique x, y and z fields".into());
    };
    let axes = [x, y, z];
    for first in 0..3 {
        for second in first + 1..3 {
            let (a, a_len) = axes[first];
            let (b, b_len) = axes[second];
            if a < b + b_len && b < a + a_len {
                return Err("NavigationMap3D XYZ fields must not overlap".into());
            }
        }
    }
    // 即使来源声明非稠密，也逐点验证规划实际使用的三个坐标。
    for row in 0..cloud.height as usize {
        for col in 0..cloud.width as usize {
            let base = row * cloud.row_step as usize + col * cloud.point_step as usize;
            for (offset, bytes) in axes {
                let value = navigation_map_3d_coordinate(cloud, base + offset, bytes);
                if !value.is_finite() {
                    return Err("NavigationMap3D cloud XYZ must be finite".into());
                }
            }
        }
    }
    Ok(())
}

fn navigation_map_3d_coordinate(cloud: &PointCloud2, offset: usize, bytes: usize) -> f64 {
    let data = &cloud.data[offset..offset + bytes];
    if bytes == 4 {
        let data = data.try_into().expect("已验证四字节坐标范围");
        (if cloud.is_bigendian {
            f32::from_be_bytes(data)
        } else {
            f32::from_le_bytes(data)
        }) as f64
    } else {
        let data = data.try_into().expect("已验证八字节坐标范围");
        if cloud.is_bigendian {
            f64::from_be_bytes(data)
        } else {
            f64::from_le_bytes(data)
        }
    }
}

/// 校验消息并按真实布局提取单精度三维点；清空返回空列表，非法缩窄明确拒绝。
pub fn navigation_map_3d_points(message: &NavigationMap3D) -> CodecResult<Vec<(f32, f32, f32)>> {
    validate_navigation_map_3d(message).map_err(CodecError::LcmDecode)?;
    if message.fine.is_some() { return Err(CodecError::LcmDecode("精细地图必须先完成缺片补取，再按空间范围读取".into())); }
    if message.cleared {
        return Ok(Vec::new());
    }
    let cloud = &message.cloud;
    let axes = ["x", "y", "z"].map(|name| {
        let field = cloud
            .fields
            .iter()
            .find(|field| field.name == name)
            .expect("消息校验已确认唯一三维字段");
        (
            field.offset as usize,
            if field.datatype == 7 { 4 } else { 8 },
        )
    });
    let count = (cloud.height as usize)
        .checked_mul(cloud.width as usize)
        .ok_or_else(|| CodecError::LcmDecode("NavigationMap3D point count exceeds usize".into()))?;
    let mut points = Vec::with_capacity(count);
    for row in 0..cloud.height as usize {
        for col in 0..cloud.width as usize {
            let base = row * cloud.row_step as usize + col * cloud.point_step as usize;
            let xyz = axes.map(|(offset, bytes)| {
                navigation_map_3d_coordinate(cloud, base + offset, bytes) as f32
            });
            if !xyz.iter().all(|value| value.is_finite()) {
                return Err(CodecError::LcmDecode(
                    "NavigationMap3D XYZ conversion to float32 must remain finite".into(),
                ));
            }
            points.push((xyz[0], xyz[1], xyz[2]));
        }
    }
    Ok(points)
}

/// 编码包含真实LCM指纹的三维导航地图。
pub fn encode_navigation_map_3d(message: &NavigationMap3D) -> CodecResult<Vec<u8>> {
    Ok(with_fingerprint(
        NAVIGATION_MAP_3D_FINGERPRINT,
        encode_navigation_map_3d_body(message)?,
    ))
}

/// 编码三维导航地图的纯消息体，与生成绑定的嵌套编码一致。
pub fn encode_navigation_map_3d_body(message: &NavigationMap3D) -> CodecResult<Vec<u8>> {
    validate_navigation_map_3d(message).map_err(CodecError::LcmEncode)?;
    let mut out = Vec::new();
    let map = &message.committed_map;
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    out.extend_from_slice(&message.producer_runtime_generation.to_be_bytes());
    encode_string(&map.session_id, &mut out)?;
    out.extend_from_slice(&map.session_generation.to_be_bytes());
    out.extend_from_slice(&map.revision.to_be_bytes());
    out.extend_from_slice(&map.commit_boot_nonce);
    out.extend_from_slice(&map.commit_sequence.to_be_bytes());
    out.extend_from_slice(&map.source_timestamp_ns.to_be_bytes());
    encode_string(&map.frame_id, &mut out)?;
    out.extend_from_slice(&message.map_frame_generation.to_be_bytes());
    out.extend_from_slice(&message.cloud_revision.to_be_bytes());
    out.extend_from_slice(&message.cloud_occupancy_revision.to_be_bytes());
    out.extend_from_slice(&message.source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&message.source_voxel_size.to_be_bytes());
    out.extend_from_slice(&message.commit_boot_nonce);
    out.extend_from_slice(&message.commit_sequence.to_be_bytes());
    out.push(u8::from(message.cleared));
    out.push(u8::from(message.virtual_obstacles.is_some()));
    out.extend_from_slice(&encode_occupancy_grid_body(
        message
            .virtual_obstacles
            .as_ref()
            .unwrap_or(&OccupancyGrid::default()),
    )?);
    out.push(u8::from(message.fine.is_some()));
    if let Some(fine) = &message.fine {
        out.extend_from_slice(&(fine.root.len() as i32).to_be_bytes());
        out.extend_from_slice(&fine.root);
        out.extend_from_slice(&(fine.included.len() as i32).to_be_bytes());
        for (object, bytes) in &fine.included {
            encode_navigation_map_object_identity(*object, &mut out);
            out.extend_from_slice(&(bytes.len() as i32).to_be_bytes());
            out.extend_from_slice(bytes);
        }
    } else {
        out.extend_from_slice(&[0; 8]);
    }
    out.extend_from_slice(&encode_point_cloud2_body(&message.cloud)?);
    Ok(out)
}

/// 解码并验证包含指纹的三维导航地图。
pub fn decode_navigation_map_3d(encoded: &[u8]) -> CodecResult<NavigationMap3D> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(NAVIGATION_MAP_3D_FINGERPRINT, "nuwax_msgs.NavigationMap3D")?;
    decode_navigation_map_3d_body(cursor.read_exact(cursor.remaining())?)
}

/// 解码并验证三维导航地图的纯消息体。
pub fn decode_navigation_map_3d_body(encoded: &[u8]) -> CodecResult<NavigationMap3D> {
    let mut cursor = Cursor::new(encoded);
    let schema_version = cursor.read_i32()?;
    if schema_version != NAVIGATION_MAP_3D_SCHEMA_VERSION {
        return Err(CodecError::LcmDecode("unsupported NavigationMap3D schema_version".into()));
    }
    let producer_runtime_generation = cursor.read_i64()?;
    let committed_map = CommittedMapIdentity {
        session_id: cursor.read_string()?,
        session_generation: cursor.read_i64()?,
        revision: cursor.read_i64()?,
        commit_boot_nonce: cursor.read_exact(16)?.try_into().expect("16-byte occupancy boot nonce"),
        commit_sequence: cursor.read_i64()?,
        source_timestamp_ns: cursor.read_i64()?,
        frame_id: cursor.read_string()?,
    };
    let map_frame_generation = cursor.read_i64()?;
    let cloud_revision = cursor.read_i64()?;
    let cloud_occupancy_revision = cursor.read_i64()?;
    let source_timestamp_ns = cursor.read_i64()?;
    let source_voxel_size = cursor.read_f32()?;
    let commit_boot_nonce = cursor
        .read_exact(16)?
        .try_into()
        .expect("16字节点云提交进程标识");
    let commit_sequence = cursor.read_i64()?;
    let cleared = match cursor.read_u8()? {
        0 => false,
        1 => true,
        _ => {
            return Err(CodecError::LcmDecode(
                "NavigationMap3D cleared must be 0 or 1".into(),
            ))
        }
    };
    let has_virtual_obstacles = match cursor.read_u8()? {
        0 => false,
        1 => true,
        _ => {
            return Err(CodecError::LcmDecode(
                "NavigationMap3D has_virtual_obstacles must be 0 or 1".into(),
            ))
        }
    };
    let mask = decode_occupancy_grid_from(&mut cursor)?;
    let virtual_obstacles = if has_virtual_obstacles {
        Some(mask)
    } else {
        if mask != OccupancyGrid::default() {
            return Err(CodecError::LcmDecode(
                "NavigationMap3D absent virtual obstacles must use an empty placeholder".into(),
            ));
        }
        None
    };
    let has_fine = match cursor.read_u8()? { 0 => false, 1 => true, _ => return Err(CodecError::LcmDecode("has_fine_map必须为0或1".into())) };
    let length = cursor.read_i32()?;
    if length < 0 || length as usize > nuwax_map_chunks::MAX_MANIFEST_BYTES { return Err(CodecError::LcmDecode("精细根长度无效".into())); }
    let root = cursor.read_exact(length as usize)?.to_vec();
    let count = cursor.read_i32()?;
    if count < 0 || count as usize > NAVIGATION_MAP_MAX_INCLUDED_BLOCKS { return Err(CodecError::LcmDecode("携带精细块数量无效".into())); }
    let mut included = std::collections::BTreeMap::new();
    let mut total = 0;
    for _ in 0..count {
        let object = decode_navigation_map_object_identity(&mut cursor)?;
        let length = cursor.read_i32()?;
        if length < 0 || length as usize > nuwax_map_chunks::MAX_BLOCK_BYTES { return Err(CodecError::LcmDecode("精细块长度无效".into())); }
        total += length as usize;
        if total > NAVIGATION_MAP_MAX_INCLUDED_BYTES { return Err(CodecError::LcmDecode("携带精细块总长超预算".into())); }
        if included.insert(object, cursor.read_exact(length as usize)?.to_vec()).is_some() { return Err(CodecError::LcmDecode("携带精细块重复".into())); }
    }
    let fine = if has_fine { Some(nuwax_map_chunks::Transfer { root, included }) }
        else if root.is_empty() && included.is_empty() { None }
        else { return Err(CodecError::LcmDecode("空精细根不能携带隐藏数据".into())); };
    let cloud_body = cursor.read_exact(cursor.remaining())?;
    let cloud = decode_point_cloud2(&with_fingerprint(
        POINT_CLOUD2_FINGERPRINT,
        cloud_body.to_vec(),
    ))?;
    let message = NavigationMap3D {
        schema_version,
        producer_runtime_generation,
        committed_map,
        map_frame_generation,
        cloud_revision,
        cloud_occupancy_revision,
        source_timestamp_ns,
        source_voxel_size,
        commit_boot_nonce,
        commit_sequence,
        cleared,
        virtual_obstacles,
        fine,
        cloud,
    };
    validate_navigation_map_3d(&message).map_err(CodecError::LcmDecode)?;
    Ok(message)
}

/// Fixed-width tagged identity used only by NavigationMapBlock's v3 wire body.
fn encode_navigation_map_object_identity(object: nuwax_map_chunks::ObjectIdentity, out: &mut Vec<u8>) {
    use nuwax_map_chunks::ObjectIdentity;
    match object {
        ObjectIdentity::Assigned(id) => {
            out.push(NAVIGATION_MAP_BLOCK_KIND_ASSIGNED);
            out.extend_from_slice(&id.0);
            out.extend_from_slice(&[0; 32]);
        }
        ObjectIdentity::Legacy(alias) => {
            out.push(NAVIGATION_MAP_BLOCK_KIND_LEGACY);
            out.extend_from_slice(&[0; 16]);
            out.extend_from_slice(&alias.0);
        }
    }
}

fn decode_navigation_map_object_identity(cursor: &mut Cursor<'_>) -> CodecResult<nuwax_map_chunks::ObjectIdentity> {
    use nuwax_map_chunks::{LegacyAlias, ObjectId, ObjectIdentity};
    let kind = cursor.read_u8()?;
    let object_id: [u8; 16] = cursor.read_exact(16)?.try_into().unwrap();
    let legacy_alias: [u8; 32] = cursor.read_exact(32)?.try_into().unwrap();
    let object = match kind {
        NAVIGATION_MAP_BLOCK_KIND_ASSIGNED if legacy_alias == [0; 32] => ObjectIdentity::Assigned(ObjectId(object_id)),
        NAVIGATION_MAP_BLOCK_KIND_LEGACY if object_id == [0; 16] => ObjectIdentity::Legacy(LegacyAlias(legacy_alias)),
        _ => return Err(CodecError::LcmDecode("invalid or mixed NavigationMapBlock object identity".into())),
    };
    object.validate().map_err(CodecError::LcmDecode)?;
    Ok(object)
}

/// LCM representation of `sensor_msgs.Imu`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Imu {
    /// Header with source timestamp and frame.
    pub header: Header,
    /// Orientation quaternion `(x, y, z, w)`.
    pub orientation_xyzw: [f64; 4],
    /// Orientation covariance.
    pub orientation_covariance: [f64; 9],
    /// Angular velocity `(x, y, z)`.
    pub angular_velocity: [f64; 3],
    /// Angular velocity covariance.
    pub angular_velocity_covariance: [f64; 9],
    /// Linear acceleration `(x, y, z)`.
    pub linear_acceleration: [f64; 3],
    /// Linear acceleration covariance.
    pub linear_acceleration_covariance: [f64; 9],
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
    out.extend_from_slice(&message.committed_map_boot_nonce);
    out.extend_from_slice(&message.committed_map_commit_sequence.to_be_bytes());
    out.extend_from_slice(&message.committed_map_source_timestamp_ns.to_be_bytes());
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
    encode_goal_options(&message.options, out);
    out.extend_from_slice(&message.navigation_configuration_sha256);
    out.extend_from_slice(&message.map_frame_generation.to_be_bytes());
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
    let committed_map_boot_nonce = cursor.read_exact(16)?.try_into().expect("16-byte committed map boot nonce");
    let committed_map_commit_sequence = cursor.read_i64()?;
    let committed_map_source_timestamp_ns = cursor.read_i64()?;
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
    let allow_reverse=match cursor.read_exact(1)?[0] {0=>false,1=>true,_=>return Err(CodecError::LcmDecode("allow_reverse must be 0 or 1".into()))};
    let options=nuwax_types::navigation::GoalOptions {allow_reverse,position_tolerance_m:cursor.read_f64()?,yaw_tolerance_rad:cursor.read_f64()?};
    let navigation_configuration_sha256=cursor.read_exact(32)?.try_into().expect("32-byte configuration digest");
    let map_frame_generation = cursor.read_i64()?;
    let payload_sha256 = cursor
        .read_exact(32)?
        .try_into()
        .expect("32-byte goal digest");
    let goal = PoseStamped {
        header: decode_header(cursor)?,
        pose: decode_pose_body(cursor)?,
    };
    Ok(GoalEnvelope {
        map_frame_generation,
        navigation_configuration_sha256,
        options,
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
        committed_map_boot_nonce,
        committed_map_commit_sequence,
        committed_map_source_timestamp_ns,
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
    message.options.validate()?;
    if message.map_frame_generation < 0 { return Err("map_frame_generation must be non-negative".into()); }
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
        &message.committed_map_boot_nonce,
        message.committed_map_commit_sequence,
        message.committed_map_source_timestamp_ns,
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
    out.extend_from_slice(&message.committed_map_boot_nonce);
    out.extend_from_slice(&message.committed_map_commit_sequence.to_be_bytes());
    out.extend_from_slice(&message.committed_map_source_timestamp_ns.to_be_bytes());
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
    let committed_map_boot_nonce = cursor.read_exact(16)?.try_into().expect("16-byte committed map boot nonce");
    let committed_map_commit_sequence = cursor.read_i64()?;
    let committed_map_source_timestamp_ns = cursor.read_i64()?;
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
        committed_map_boot_nonce,
        committed_map_commit_sequence,
        committed_map_source_timestamp_ns,
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
        &message.committed_map_boot_nonce,
        message.committed_map_commit_sequence,
        message.committed_map_source_timestamp_ns,
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
    committed_map_boot_nonce: &[u8; 16],
    committed_map_commit_sequence: i64,
    committed_map_source_timestamp_ns: i64,
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
    if *committed_map_boot_nonce == [0; 16]
        || committed_map_commit_sequence <= 0
        || committed_map_source_timestamp_ns < 0
    {
        return Err("committed map requires an actual populated occupancy commit receipt".into());
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

/// Encode a top-level `sensor_msgs.Imu` generated message.
pub fn encode_imu(message: &Imu) -> Vec<u8> {
    with_fingerprint(IMU_FINGERPRINT, encode_imu_body(message))
}

/// Encode only the generated `Imu._encode_one()` body.
pub fn encode_imu_body(message: &Imu) -> Vec<u8> {
    let mut out = Vec::new();
    encode_header(&message.header, &mut out).expect("Header frame_id length must fit");
    encode_f64_array(&message.orientation_xyzw, &mut out);
    encode_f64_array(&message.orientation_covariance, &mut out);
    encode_f64_array(&message.angular_velocity, &mut out);
    encode_f64_array(&message.angular_velocity_covariance, &mut out);
    encode_f64_array(&message.linear_acceleration, &mut out);
    encode_f64_array(&message.linear_acceleration_covariance, &mut out);
    out
}

/// Decode a top-level `sensor_msgs.Imu` generated message.
pub fn decode_imu(encoded: &[u8]) -> CodecResult<Imu> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(IMU_FINGERPRINT, "sensor_msgs.Imu")?;
    let message = decode_imu_from(&mut cursor)?;
    cursor.finish("sensor_msgs.Imu")?;
    Ok(message)
}

/// Decode only a generated `Imu._encode_one()` body.
pub fn decode_imu_body(encoded: &[u8]) -> CodecResult<Imu> {
    let mut cursor = Cursor::new(encoded);
    let message = decode_imu_from(&mut cursor)?;
    cursor.finish("sensor_msgs.Imu body")?;
    Ok(message)
}

fn decode_imu_from(cursor: &mut Cursor<'_>) -> CodecResult<Imu> {
    Ok(Imu {
        header: decode_header(cursor)?,
        orientation_xyzw: cursor.read_f64_array()?,
        orientation_covariance: cursor.read_f64_array()?,
        angular_velocity: cursor.read_f64_array()?,
        angular_velocity_covariance: cursor.read_f64_array()?,
        linear_acceleration: cursor.read_f64_array()?,
        linear_acceleration_covariance: cursor.read_f64_array()?,
    })
}

/// 解码嵌套惯性前缀，复用同一布局并返回实际消耗字节数。
pub(crate) fn decode_imu_prefix(encoded: &[u8]) -> CodecResult<(Imu, usize)> {
    let mut cursor = Cursor::new(encoded);
    let sample = decode_imu_from(&mut cursor)?;
    Ok((sample, encoded.len() - cursor.remaining()))
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
    out.extend_from_slice(&message.pose_source_timestamp_ns.to_be_bytes());
    out.extend_from_slice(&message.costmap_source_timestamp_ns.to_be_bytes());
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
    if message.local_path.len() > 65536 || message.local_path.iter().flatten().any(|v| !v.is_finite()) {
        return Err(CodecError::LcmEncode("局部路线点数越界或坐标非法".into()));
    }
    out.extend_from_slice(&(message.local_path.len() as i32).to_be_bytes());
    for point in &message.local_path { encode_f64_array(point, &mut out); }
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
        pose_source_timestamp_ns: cursor.read_i64()?,
        costmap_source_timestamp_ns: cursor.read_i64()?,
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
        local_path: {
            let count = cursor.read_nonnegative_len("local_path_count")?;
            if count > 65536 || count > cursor.remaining() / 24 {
                return Err(CodecError::LcmDecode("局部路线点数超过载荷长度或容量".into()));
            }
            let mut points = Vec::with_capacity(count);
            for _ in 0..count {
                let point: [f64; 3] = cursor.read_f64_array()?;
                if point.iter().any(|v| !v.is_finite()) {
                    return Err(CodecError::LcmDecode("局部路线坐标非法".into()));
                }
                points.push(point);
            }
            points
        },
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
mod navigation_map_3d_tests {
    use super::*;

    fn live_map() -> NavigationMap3D {
        NavigationMap3D {
            producer_runtime_generation: 11,
            committed_map: CommittedMapIdentity {
                session_id: "mapping:test".into(),
                session_generation: 2,
                revision: 9,
                commit_boot_nonce: [0x34; 16],
                commit_sequence: 41,
                source_timestamp_ns: 2_000_000_003,
                frame_id: "robot/map".into(),
            },
            map_frame_generation: 0,
            cloud_revision: 17,
            cloud_occupancy_revision: 9,
            source_timestamp_ns: 2_000_000_003,
            source_voxel_size: 0.125,
            commit_boot_nonce: [0x34; 16],
            commit_sequence: 41,
            cloud: PointCloud2 {
                header: Header {
                    seq: 17,
                    stamp_sec: 2,
                    stamp_nsec: 3,
                    frame_id: "robot/map".into(),
                },
                height: 1,
                width: 2,
                fields: ["x", "y", "z"]
                    .iter()
                    .enumerate()
                    .map(|(index, name)| PointField {
                        name: (*name).into(),
                        offset: index as i32 * 4,
                        datatype: 7,
                        count: 1,
                    })
                    .collect(),
                point_step: 12,
                row_step: 24,
                data: [1.0_f32, -2.0, 3.0, 4.0, 5.0, 6.0]
                    .iter()
                    .flat_map(|value| value.to_le_bytes())
                    .collect(),
                is_dense: true,
                ..PointCloud2::default()
            },
            ..NavigationMap3D::default()
        }
    }

    fn cleared_map() -> NavigationMap3D {
        let mut message = live_map();
        message.cleared = true;
        message.committed_map = CommittedMapIdentity {
            frame_id: "robot/map".into(),
            ..CommittedMapIdentity::default()
        };
        message.source_timestamp_ns = 0;
        message.commit_sequence = 0;
        message.cloud_occupancy_revision = 0;
        message.cloud = PointCloud2 {
            header: Header {
                seq: 17,
                frame_id: "robot/map".into(),
                ..Header::default()
            },
            ..PointCloud2::default()
        };
        message
    }

    fn map_with_virtual_obstacles() -> NavigationMap3D {
        let mut message = live_map();
        message.virtual_obstacles = Some(OccupancyGrid {
            header: Header {
                seq: message.committed_map.revision as i32,
                ..message.cloud.header.clone()
            },
            info: MapMetaData {
                resolution: 0.25,
                width: 2,
                height: 2,
                origin: Pose {
                    position: [-1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                ..MapMetaData::default()
            },
            data: vec![0, 100, 100, 0],
        });
        message
    }

    fn fine_map(object: nuwax_map_chunks::ObjectIdentity) -> NavigationMap3D {
        use nuwax_map_chunks::{Block, BlockRef, Evidence, Manifest, Transfer, encode_block};
        let mut message = live_map();
        let block = Block::from([(1, Evidence {
            observed_ns: message.source_timestamp_ns as u64,
            hits: 2,
        })]);
        let root = Manifest {
            version: 2,
            domain: navigation_map_3d_domain(&message),
            revision: message.cloud_revision as u64,
            geometry_revision: 9,
            source_timestamp_ns: message.source_timestamp_ns as u64,
            voxel_size_m: message.source_voxel_size,
            blocks: vec![BlockRef {
                key: [0; 3], object, block_revision: 12, geometry_revision: 9, points: 1,
            }],
        };
        message.fine = Some(Transfer {
            root: root.encode().unwrap(),
            included: std::collections::BTreeMap::from([(object, encode_block(&block).unwrap())]),
        });
        message.cloud.width = 0;
        message.cloud.row_step = 0;
        message.cloud.data.clear();
        message
    }

    #[test]
    fn composite_receipt_keeps_retained_cloud_time_and_current_mask_time_distinct() {
        let mut message = fine_map(nuwax_map_chunks::ObjectIdentity::Assigned(
            nuwax_map_chunks::ObjectId([0x51; 16])));
        let retained_fine = message.fine.clone();
        let retained_cloud = message.cloud.clone();
        message.committed_map.revision += 1;
        message.committed_map.commit_sequence += 1;
        message.committed_map.source_timestamp_ns += 100;
        let mut mask = map_with_virtual_obstacles().virtual_obstacles.unwrap();
        mask.header.seq = message.committed_map.revision as i32;
        mask.header.stamp_nsec += 100;
        message.virtual_obstacles = Some(mask);
        let wire = encode_navigation_map_3d(&message).unwrap();
        assert_eq!(decode_navigation_map_3d(&wire).unwrap(), message);
        assert_eq!(message.fine, retained_fine);
        assert_eq!(message.cloud, retained_cloud);
        assert_eq!(message.cloud_occupancy_revision, 9);
        assert_eq!(message.source_timestamp_ns, 2_000_000_003);
        let mut wrong_mask = message.clone();
        wrong_mask.virtual_obstacles.as_mut().unwrap().header.stamp_nsec -= 100;
        assert!(validate_navigation_map_3d(&wrong_mask).is_err());
        for mutate in [
            (|m: &mut NavigationMap3D| m.commit_boot_nonce = [0x01; 16]) as fn(&mut NavigationMap3D),
            |m| m.commit_boot_nonce = [0xff; 16],
            |m| m.cloud_occupancy_revision = m.committed_map.revision + 1,
            |m| m.commit_sequence = m.committed_map.commit_sequence + 1,
            |m| m.cloud_occupancy_revision = m.committed_map.revision,
            |m| m.commit_sequence = m.committed_map.commit_sequence,
        ] {
            let mut bad = message.clone();
            mutate(&mut bad);
            assert!(validate_navigation_map_3d(&bad).is_err());
        }
        let mut conflicting_time = live_map();
        conflicting_time.committed_map.source_timestamp_ns += 1;
        assert!(validate_navigation_map_3d(&conflicting_time).is_err());
    }

    #[test]
    fn clear_is_explicit_absence_and_partial_occupancy_receipts_are_rejected() {
        let cleared = cleared_map();
        assert!(validate_navigation_map_3d(&cleared).is_ok());
        let mut live_absence = cleared.clone();
        live_absence.cleared = false;
        assert!(validate_navigation_map_3d(&live_absence).is_err());
        for mutate in [
            (|m: &mut NavigationMap3D| m.committed_map.commit_boot_nonce = [1; 16]) as fn(&mut NavigationMap3D),
            |m| m.committed_map.commit_sequence = 1,
            |m| m.committed_map.source_timestamp_ns = 1,
            |m| m.cloud_occupancy_revision = 1,
        ] {
            let mut partial = cleared.clone();
            mutate(&mut partial);
            assert!(validate_navigation_map_3d(&partial).is_err());
        }
    }

    #[test]
    fn virtual_obstacles_round_trip_and_reject_mismatched_or_unsafe_masks() {
        let message = map_with_virtual_obstacles();
        assert_eq!(
            decode_navigation_map_3d(&encode_navigation_map_3d(&message).unwrap()).unwrap(),
            message
        );
        let mutations: &[fn(&mut OccupancyGrid)] = &[
            |mask| mask.header.seq += 1,
            |mask| mask.header.frame_id = "wrong/map".into(),
            |mask| mask.header.stamp_nsec += 1,
            |mask| mask.info.resolution = f32::NAN,
            |mask| mask.info.resolution = 0.0,
            |mask| mask.info.origin.position[0] = f64::INFINITY,
            |mask| mask.info.origin.position[2] = 1.0,
            |mask| mask.info.origin.orientation_xyzw = [0.0, 0.0, 1.0, 0.0],
            |mask| mask.info.width = -1,
            |mask| mask.data.pop().map(|_| ()).unwrap(),
            |mask| mask.data[0] = -1,
            |mask| mask.data[0] = 50,
        ];
        for mutation in mutations {
            let mut corrupted = message.clone();
            mutation(corrupted.virtual_obstacles.as_mut().unwrap());
            assert!(validate_navigation_map_3d(&corrupted).is_err());
        }
        let mut clear = cleared_map();
        clear.virtual_obstacles = message.virtual_obstacles;
        assert!(validate_navigation_map_3d(&clear).is_err());
    }

    #[test]
    fn live_and_clear_round_trip_and_protocol_fingerprint() {
        let message_type =
            nuwax_protocol::LcmMessageType::new("nuwax_msgs.NavigationMap3D").unwrap();
        assert_eq!(message_type.fingerprint(), NAVIGATION_MAP_3D_FINGERPRINT);
        assert_eq!(
            navigation_map_3d_points(&live_map()).unwrap(),
            vec![(1.0, -2.0, 3.0), (4.0, 5.0, 6.0)]
        );
        assert!(navigation_map_3d_points(&cleared_map()).unwrap().is_empty());
        for message in [live_map(), cleared_map()] {
            let wire = encode_navigation_map_3d(&message).unwrap();
            let body = encode_navigation_map_3d_body(&message).unwrap();
            assert_eq!(&wire[..8], &NAVIGATION_MAP_3D_FINGERPRINT);
            assert_eq!(&wire[8..], body);
            assert_eq!(decode_navigation_map_3d(&wire).unwrap(), message);
            assert_eq!(decode_navigation_map_3d_body(&body).unwrap(), message);
        }
        let mut cleared = cleared_map();
        cleared.committed_map.session_id = "mapping:prepared".into();
        cleared.committed_map.session_generation = 3;
        cleared.cloud.height = 1;
        cleared.cloud.point_step = 12;
        cleared.cloud.fields = live_map().cloud.fields;
        assert!(validate_navigation_map_3d(&cleared).is_ok());
    }

    #[test]
    fn invalid_identities_and_cloud_headers_are_rejected() {
        let mutations: &[fn(&mut NavigationMap3D)] = &[
            |m| m.schema_version = 2,
            |m| m.producer_runtime_generation = 0,
            |m| m.committed_map.session_id.clear(),
            |m| m.committed_map.session_id.push('\0'),
            |m| m.committed_map.session_generation = 0,
            |m| m.committed_map.revision = 0,
            |m| m.committed_map.commit_boot_nonce = [0; 16],
            |m| m.committed_map.commit_sequence = 0,
            |m| m.committed_map.source_timestamp_ns = -1,
            |m| m.cloud_occupancy_revision = 0,
            |m| m.committed_map.frame_id = "/map".into(),
            |m| m.map_frame_generation = -1,
            |m| m.cloud_revision = 0,
            |m| m.cloud_revision = i64::from(i32::MAX) + 1,
            |m| m.source_timestamp_ns = 0,
            |m| m.source_voxel_size = f32::NAN,
            |m| m.source_voxel_size = f32::INFINITY,
            |m| m.source_voxel_size = 0.0,
            |m| m.commit_boot_nonce = [0; 16],
            |m| m.commit_sequence = 0,
            |m| m.cloud.header.seq += 1,
            |m| m.cloud.header.frame_id = "map".into(),
            |m| m.cloud.header.stamp_nsec = 1_000_000_000,
            |m| m.cloud.header.stamp_sec = -1,
        ];
        for (index, mutate) in mutations.iter().enumerate() {
            let mut message = live_map();
            mutate(&mut message);
            assert!(
                validate_navigation_map_3d(&message).is_err(),
                "非法身份用例{index}"
            );
            assert!(encode_navigation_map_3d(&message).is_err());
        }
        for mutate in [
            (|m: &mut NavigationMap3D| m.cloud.data.push(1)) as fn(&mut NavigationMap3D),
            |m| m.committed_map.session_generation = 1,
            |m| m.committed_map.frame_id.clear(),
            |m| m.commit_sequence = -1,
        ] {
            let mut message = cleared_map();
            mutate(&mut message);
            assert!(validate_navigation_map_3d(&message).is_err());
        }
    }

    #[test]
    fn invalid_layout_and_nonfinite_coordinates_are_rejected() {
        let mutations: &[fn(&mut NavigationMap3D)] = &[
            |m| m.cloud.width = 0,
            |m| m.cloud.height = -1,
            |m| m.cloud.row_step = 23,
            |m| m.cloud.point_step = 8,
            |m| {
                m.cloud.data.pop();
            },
            |m| m.cloud.fields.pop().map(|_| ()).unwrap(),
            |m| m.cloud.fields[1].name = "x".into(),
            |m| m.cloud.fields[1].offset = 0,
            |m| m.cloud.fields[2].count = 2,
            |m| m.cloud.fields[0].datatype = 5,
            |m| m.cloud.fields[0].offset = -1,
            |m| m.cloud.fields[0].count = i32::MAX,
            |m| m.cloud.data[..4].copy_from_slice(&f32::NAN.to_le_bytes()),
            |m| {
                m.cloud.is_dense = false;
                m.cloud.data[20..24].copy_from_slice(&f32::INFINITY.to_le_bytes());
            },
        ];
        for (index, mutate) in mutations.iter().enumerate() {
            let mut message = live_map();
            mutate(&mut message);
            assert!(
                validate_navigation_map_3d(&message).is_err(),
                "非法布局用例{index}"
            );
        }
    }

    #[test]
    fn float64_big_endian_and_padded_rows_are_supported() {
        let mut message = live_map();
        message.cloud.width = 1;
        message.cloud.height = 2;
        message.cloud.point_step = 24;
        message.cloud.row_step = 32;
        message.cloud.is_bigendian = true;
        for (index, field) in message.cloud.fields.iter_mut().enumerate() {
            field.offset = [16, 0, 8][index];
            field.datatype = 8;
        }
        message.cloud.fields.reverse();
        message.cloud.data = [1.0_f64, 2.0, 3.0, 0.0, 4.0, 5.0, 6.0, 0.0]
            .iter()
            .flat_map(|value| value.to_be_bytes())
            .collect();
        let wire = encode_navigation_map_3d(&message).unwrap();
        assert_eq!(decode_navigation_map_3d(&wire).unwrap(), message);
        assert_eq!(
            navigation_map_3d_points(&message).unwrap(),
            vec![(3.0, 1.0, 2.0), (6.0, 4.0, 5.0)]
        );
        for finite_but_too_large in [f64::MAX, -f64::MAX] {
            message.cloud.data[32..40].copy_from_slice(&finite_but_too_large.to_be_bytes());
            assert!(validate_navigation_map_3d(&message).is_ok());
            assert!(navigation_map_3d_points(&message)
                .unwrap_err()
                .to_string()
                .contains("float32"));
        }
        message.cloud.data[32..40].copy_from_slice(&f64::INFINITY.to_be_bytes());
        assert!(validate_navigation_map_3d(&message).is_err());
        assert!(navigation_map_3d_points(&message).is_err());
    }

    #[test]
    fn truncated_trailing_and_invalid_clear_flag_wires_are_rejected() {
        let message = live_map();
        let wire = encode_navigation_map_3d(&message).unwrap();
        for length in 0..wire.len() {
            assert!(
                decode_navigation_map_3d(&wire[..length]).is_err(),
                "截断长度{length}"
            );
        }
        let mut corrupted = wire.clone();
        corrupted.push(0);
        assert!(decode_navigation_map_3d(&corrupted).is_err());
        let clear_offset = wire.len()
            - 9 // 空精细根的标志和两个长度字段。
            - encode_point_cloud2_body(&message.cloud).unwrap().len()
            - encode_occupancy_grid_body(&OccupancyGrid::default())
                .unwrap()
                .len()
            - 2;
        corrupted = wire.clone();
        corrupted[clear_offset] = 2;
        assert!(decode_navigation_map_3d(&corrupted).is_err());
        corrupted = wire.clone();
        corrupted[clear_offset + 1] = 2;
        assert!(decode_navigation_map_3d(&corrupted).is_err());
        let mut hidden_mask = encode_navigation_map_3d(&map_with_virtual_obstacles()).unwrap();
        hidden_mask[clear_offset + 1] = 0;
        assert!(decode_navigation_map_3d(&hidden_mask).is_err());
        corrupted = wire;
        corrupted[0] ^= 1;
        assert!(decode_navigation_map_3d(&corrupted).is_err());
    }

    #[test]
    fn fine_root_round_trip_and_commit_binding_rejects_mixed_source() {
        use nuwax_map_chunks::{LegacyAlias, ObjectId, ObjectIdentity};
        for object in [
            ObjectIdentity::Assigned(ObjectId([0x51; 16])),
            ObjectIdentity::Legacy(LegacyAlias([0x72; 32])),
        ] {
            let message = fine_map(object);
            let wire = encode_navigation_map_3d(&message).unwrap();
            assert_eq!(decode_navigation_map_3d(&wire).unwrap(), message);
            assert!(navigation_map_3d_points(&message).is_err());
            let mut corrupted = message.clone();
            corrupted.map_frame_generation += 1;
            assert!(validate_navigation_map_3d(&corrupted).is_err());
            corrupted = message.clone();
            // Structural rejection replaces the removed content-digest comparison.
            corrupted.fine.as_mut().unwrap().included.values_mut().next().unwrap()[16..18]
                .copy_from_slice(&u16::MAX.to_le_bytes());
            assert!(validate_navigation_map_3d(&corrupted).is_err());
            corrupted = message.clone();
            corrupted.fine.as_mut().unwrap().included.values_mut().next().unwrap()[18..26]
                .copy_from_slice(&(message.source_timestamp_ns as u64 + 1).to_le_bytes());
            assert!(validate_navigation_map_3d(&corrupted).is_err());
            corrupted = message.clone();
            let fine = corrupted.fine.as_mut().unwrap();
            let bytes = fine.included.remove(&object).unwrap();
            fine.included.insert(ObjectIdentity::Assigned(ObjectId([0x99; 16])), bytes);
            assert!(validate_navigation_map_3d(&corrupted).is_err());
            let mut missing = message;
            missing.fine.as_mut().unwrap().included.clear();
            assert_eq!(decode_navigation_map_3d(&encode_navigation_map_3d(&missing).unwrap()).unwrap(), missing);
        }
    }

    #[test]
    fn c1_object_tags_reject_mixed_zero_unknown_and_truncated_identities() {
        use nuwax_map_chunks::{LegacyAlias, ObjectId, ObjectIdentity};
        for object in [
            ObjectIdentity::Assigned(ObjectId([0x51; 16])),
            ObjectIdentity::Legacy(LegacyAlias([0x72; 32])),
        ] {
            let mut encoded = Vec::new();
            encode_navigation_map_object_identity(object, &mut encoded);
            assert_eq!(encoded.len(), 49);
            assert_eq!(decode_navigation_map_object_identity(&mut Cursor::new(&encoded)).unwrap(), object);
            for length in 0..encoded.len() {
                assert!(decode_navigation_map_object_identity(&mut Cursor::new(&encoded[..length])).is_err());
            }
            let mut mixed = encoded.clone();
            mixed[1] = 1;
            mixed[17] = 1;
            assert!(decode_navigation_map_object_identity(&mut Cursor::new(&mixed)).is_err());
            encoded[1..].fill(0);
            assert!(decode_navigation_map_object_identity(&mut Cursor::new(&encoded)).is_err());
        }
        for kind in [0, 3, u8::MAX] {
            let mut encoded = [0; 49];
            encoded[0] = kind;
            assert!(decode_navigation_map_object_identity(&mut Cursor::new(&encoded)).is_err());
        }
    }

    #[test]
    fn c1_wire_rejects_old_version_duplicate_inclusions_and_hidden_fine_data() {
        use nuwax_map_chunks::{ObjectId, ObjectIdentity};
        let message = fine_map(ObjectIdentity::Assigned(ObjectId([0x51; 16])));
        let wire = encode_navigation_map_3d(&message).unwrap();
        let fine = message.fine.as_ref().unwrap();
        let cloud_offset = wire.len() - encode_point_cloud2_body(&message.cloud).unwrap().len();
        let included_offset = cloud_offset - 49 - 4 - fine.included.values().next().unwrap().len();
        let fine_flag_offset = included_offset - 4 - fine.root.len() - 4 - 1;
        let mut old_fingerprint = wire.clone();
        old_fingerprint[..8].copy_from_slice(&NAVIGATION_MAP_3D_V2_FINGERPRINT);
        assert!(decode_navigation_map_3d(&old_fingerprint).is_err());
        for version in [2_i32, 3] {
            let mut old_version = wire.clone();
            old_version[8..12].copy_from_slice(&version.to_be_bytes());
            assert!(decode_navigation_map_3d(&old_version).is_err());
        }
        let mut prior_fingerprint = wire.clone();
        prior_fingerprint[..8].copy_from_slice(&0x06bd_4e44_dd7a_2e13_u64.to_be_bytes());
        assert!(decode_navigation_map_3d(&prior_fingerprint).is_err());
        let mut hidden = wire.clone();
        hidden[fine_flag_offset] = 0;
        assert!(decode_navigation_map_3d(&hidden).is_err());
        let mut duplicate = wire.clone();
        duplicate[included_offset - 4..included_offset].copy_from_slice(&2_i32.to_be_bytes());
        duplicate.splice(cloud_offset..cloud_offset, wire[included_offset..cloud_offset].iter().copied());
        assert!(decode_navigation_map_3d(&duplicate).is_err());
    }

    #[test]
    fn fresh_generated_c_and_python_interoperate_with_rust_and_committed_bindings() {
        use std::{fs, process::Command};
        struct TempDir(std::path::PathBuf);
        impl Drop for TempDir {
            fn drop(&mut self) {
                let _ = fs::remove_dir_all(&self.0);
            }
        }
        fn run(mut command: Command) {
            let output = command.output().expect("执行协议生成/互操作工具");
            assert!(
                output.status.success(),
                "工具失败：{}",
                String::from_utf8_lossy(&output.stderr)
            );
        }
        let nonce = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let temp = TempDir(std::env::temp_dir().join(format!(
            "nuwax-navigation-map-3d-{}-{nonce}",
            std::process::id()
        )));
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../lcmtypes");
        let c_dir = temp.0.join("c");
        let py_dir = temp.0.join("python");
        fs::create_dir_all(&c_dir).unwrap();
        fs::create_dir_all(&py_dir).unwrap();
        let types = [
            "std_msgs/Time",
            "std_msgs/Header",
            "geometry_msgs/Point",
            "geometry_msgs/Quaternion",
            "geometry_msgs/Pose",
            "geometry_msgs/PoseStamped",
            "nav_msgs/Path",
            "nav_msgs/MapMetaData",
            "nav_msgs/OccupancyGrid",
            "sensor_msgs/PointField",
            "sensor_msgs/PointCloud2",
            "nuwax_msgs/NavigationMapBlock",
            "nuwax_msgs/NavigationMap3D",
            "nuwax_msgs/GoalEnvelope",
            "nuwax_msgs/PathEnvelope",
            "nuwax_msgs/PlanarNavigationMap",
        ];
        for python in [false, true] {
            let mut command = Command::new("lcm-gen");
            if python {
                command.args(["--python", "--ppath"]).arg(&py_dir);
            } else {
                command
                    .args(["--c", "--use-quotes-for-includes", "--c-cpath"])
                    .arg(&c_dir)
                    .arg("--c-hpath")
                    .arg(&c_dir);
            }
            command.args(types.iter().map(|name| root.join(format!("{name}.lcm"))));
            run(command);
        }
        for file in [
            "nuwax_msgs_NavigationMapBlock.c",
            "nuwax_msgs_NavigationMapBlock.h",
            "nuwax_msgs_NavigationMap3D.c",
            "nuwax_msgs_NavigationMap3D.h",
            "nuwax_msgs_GoalEnvelope.c",
            "nuwax_msgs_GoalEnvelope.h",
            "nuwax_msgs_PathEnvelope.c",
            "nuwax_msgs_PathEnvelope.h",
            "nuwax_msgs_PlanarNavigationMap.c",
            "nuwax_msgs_PlanarNavigationMap.h",
        ] {
            assert_eq!(
                fs::read(c_dir.join(file)).unwrap(),
                fs::read(root.join("generated/c").join(file)).unwrap()
            );
        }
        for file in ["NavigationMap3D.py", "NavigationMapBlock.py", "GoalEnvelope.py", "PathEnvelope.py", "PlanarNavigationMap.py"] {
            assert_eq!(
                fs::read(py_dir.join("nuwax_msgs").join(file)).unwrap(),
                fs::read(root.join("generated/python/nuwax_msgs").join(file)).unwrap()
            );
        }
        let source = temp.0.join("parity.c");
        let binary = temp.0.join("parity");
        fs::write(&source, r#"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nuwax_msgs_NavigationMap3D.h"
int main(int argc, char **argv) {
    if (argc != 3) return 1;
    FILE *input = fopen(argv[1], "rb");
    if (!input) return 2;
    fseek(input, 0, SEEK_END); long size = ftell(input); rewind(input);
    unsigned char *data = malloc((size_t) size);
    if (!data || fread(data, 1, (size_t)size, input) != (size_t)size) return 3;
    fclose(input);
    nuwax_msgs_NavigationMap3D message;
    if (nuwax_msgs_NavigationMap3D_decode(data, 0, (int)size, &message) != size) return 4;
    if (message.schema_version != 4 || message.cloud_revision != 17 || message.commit_boot_nonce[0] != 0x34 ||
        message.cloud.header.seq != 17 || strcmp(message.frame_id, "robot/map")) return 5;
    if (!message.cleared && (message.occupancy_commit_boot_nonce[0] != 0x34 ||
        message.occupancy_commit_sequence != 41 || message.occupancy_source_timestamp_ns != 2000000003LL ||
        message.cloud_occupancy_revision != 9)) return 14;
    if (message.has_virtual_obstacles && (message.virtual_obstacles.data_length != 4 ||
        message.virtual_obstacles.data[1] != 100 || message.virtual_obstacles.header.seq != message.occupancy_revision)) return 8;
    if (message.has_fine_map) {
        if (message.fine_blocks_length != 1 || message.fine_manifest_length <= 0) return 9;
        const nuwax_msgs_NavigationMapBlock *block = &message.fine_blocks[0];
        if (block->object_kind == 1) {
            if (block->object_id[0] != 0x51 || block->legacy_alias[0] != 0) return 10;
        } else if (block->object_kind == 2) {
            if (block->object_id[0] != 0 || block->legacy_alias[0] != 0x72) return 11;
        } else return 12;
        if (block->data_length != 28 || memcmp(block->data, "NUWAXBL1", 8)) return 13;
    }
    message.producer_runtime_generation += 1;
    int encoded_size = nuwax_msgs_NavigationMap3D_encoded_size(&message);
    unsigned char *encoded = malloc((size_t)encoded_size);
    if (!encoded || nuwax_msgs_NavigationMap3D_encode(encoded, 0, encoded_size, &message) != encoded_size) return 6;
    FILE *output = fopen(argv[2], "wb");
    if (!output || fwrite(encoded, 1, (size_t)encoded_size, output) != (size_t)encoded_size) return 7;
    fclose(output);
    nuwax_msgs_NavigationMap3D_decode_cleanup(&message);
    free(data); free(encoded); return 0;
}
"#).unwrap();
        let mut compile = Command::new("cc");
        compile
            .args(["-std=gnu11", "-I"])
            .arg(&c_dir)
            .arg("-o")
            .arg(&binary)
            .arg(&source);
        compile
            .args(
                types
                    .iter()
                    .map(|name| c_dir.join(format!("{}.c", name.replace('/', "_")))),
            )
            .arg("-llcm");
        run(compile);
        for (name, message) in [
            ("live", live_map()),
            ("clear", cleared_map()),
            ("virtual", map_with_virtual_obstacles()),
            ("assigned", fine_map(nuwax_map_chunks::ObjectIdentity::Assigned(nuwax_map_chunks::ObjectId([0x51; 16])))),
            ("legacy_alias", fine_map(nuwax_map_chunks::ObjectIdentity::Legacy(nuwax_map_chunks::LegacyAlias([0x72; 32])))),
        ] {
            let input = temp.0.join(format!("{name}.bin"));
            let c_output = temp.0.join(format!("{name}-c.bin"));
            let py_output = temp.0.join(format!("{name}-python.bin"));
            fs::write(&input, encode_navigation_map_3d(&message).unwrap()).unwrap();
            let mut parity = Command::new(&binary);
            parity.arg(&input).arg(&c_output);
            run(parity);
            let mut expected = message.clone();
            expected.producer_runtime_generation += 1;
            assert_eq!(
                decode_navigation_map_3d(&fs::read(&c_output).unwrap()).unwrap(),
                expected
            );
            let mut python = Command::new("python3");
            python
                .args([
                    "-I",
                    "-S",
                    "-c",
                    r#"
import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from nuwax_msgs import NavigationMap3D
raw = Path(sys.argv[2]).read_bytes()
message = NavigationMap3D.decode(raw)
assert message.encode() == raw
assert message.schema_version == 4
assert message.cloud_revision == message.cloud.header.seq == 17
assert message.frame_id == message.cloud.header.frame_id == "robot/map"
if not message.cleared:
    assert bytes(message.occupancy_commit_boot_nonce) == bytes([0x34] * 16)
    assert message.occupancy_commit_sequence == 41
    assert message.occupancy_source_timestamp_ns == 2_000_000_003
    assert message.cloud_occupancy_revision == 9
if message.has_virtual_obstacles:
    assert message.virtual_obstacles.header.seq == message.occupancy_revision
    assert list(message.virtual_obstacles.data) == [0, 100, 100, 0]
if message.has_fine_map:
    assert message.fine_blocks_length == len(message.fine_blocks) == 1
    block = message.fine_blocks[0]
    if block.object_kind == 1:
        assert bytes(block.object_id) == bytes([0x51] * 16)
        assert bytes(block.legacy_alias) == bytes(32)
    else:
        assert block.object_kind == 2
        assert bytes(block.object_id) == bytes(16)
        assert bytes(block.legacy_alias) == bytes([0x72] * 32)
    assert bytes(block.data).startswith(b"NUWAXBL1")
message.producer_runtime_generation += 1
Path(sys.argv[3]).write_bytes(message.encode())
"#,
                ])
                .arg(&py_dir)
                .arg(&input)
                .arg(&py_output);
            run(python);
            let mut expected = message;
            expected.producer_runtime_generation += 1;
            assert_eq!(
                decode_navigation_map_3d(&fs::read(py_output).unwrap()).unwrap(),
                expected
            );
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn imu_round_trip_preserves_typed_sensor_sample() {
        let mut message = Imu::default();
        message.header.frame_id = "imu_link".into();
        message.header.stamp_sec = 12;
        message.header.stamp_nsec = 34;
        message.orientation_xyzw = [0.0, 0.0, 0.1, 0.99];
        message.angular_velocity = [1.0, 2.0, 3.0];
        message.linear_acceleration = [4.0, 5.0, 6.0];
        let encoded = encode_imu(&message);
        assert_eq!(decode_imu(&encoded).unwrap(), message);
        assert_eq!(&encoded[..8], &IMU_FINGERPRINT);
    }

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
            committed_map_boot_nonce: [0x31; 16],
            committed_map_commit_sequence: 29,
            committed_map_source_timestamp_ns: 122_000_000_456,
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
            committed_map_boot_nonce: goal.committed_map_boot_nonce,
            committed_map_commit_sequence: goal.committed_map_commit_sequence,
            committed_map_source_timestamp_ns: goal.committed_map_source_timestamp_ns,
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
    fn envelope_receipts_are_required_bound_and_versioned_without_digest_compatibility() {
        let mut goal = configured_frame_goal("robot/map");
        seal_goal_envelope(&mut goal).unwrap();
        let mut path = configured_frame_path(&goal, "robot/map");
        seal_path_envelope(&mut path).unwrap();
        for mutate in [
            (|m: &mut GoalEnvelope| m.committed_map_boot_nonce[0] ^= 1) as fn(&mut GoalEnvelope),
            |m| m.committed_map_commit_sequence += 1,
            |m| m.committed_map_source_timestamp_ns += 1,
        ] {
            let mut altered = goal.clone();
            mutate(&mut altered);
            assert!(validate_goal_envelope(&altered).is_err());
            seal_goal_envelope(&mut altered).unwrap();
            assert_ne!(altered.payload_sha256, goal.payload_sha256);
        }
        for mutate in [
            (|m: &mut PathEnvelope| m.committed_map_boot_nonce[0] ^= 1) as fn(&mut PathEnvelope),
            |m| m.committed_map_commit_sequence += 1,
            |m| m.committed_map_source_timestamp_ns += 1,
        ] {
            let mut altered = path.clone();
            mutate(&mut altered);
            assert!(validate_path_envelope(&altered).is_err());
            seal_path_envelope(&mut altered).unwrap();
            assert_ne!(altered.path_payload_sha256, path.path_payload_sha256);
        }
        for mutate in [
            (|m: &mut GoalEnvelope| m.committed_map_boot_nonce = [0; 16]) as fn(&mut GoalEnvelope),
            |m| m.committed_map_commit_sequence = 0,
            |m| m.committed_map_source_timestamp_ns = -1,
        ] {
            let mut absent = goal.clone();
            mutate(&mut absent);
            assert!(seal_goal_envelope(&mut absent).is_err());
        }
        let mut old_goal = encode_goal_envelope(&goal).unwrap();
        old_goal[8..12].copy_from_slice(&6_i32.to_be_bytes());
        assert!(decode_goal_envelope(&old_goal).is_err());
        let mut old_path = encode_path_envelope(&path).unwrap();
        old_path[8..12].copy_from_slice(&6_i32.to_be_bytes());
        assert!(decode_path_envelope(&old_path).is_err());
        let mut old_goal = encode_goal_envelope(&goal).unwrap();
        old_goal[..8].copy_from_slice(&0xdd19_bb81_9d6c_3e44_u64.to_be_bytes());
        assert!(decode_goal_envelope(&old_goal).is_err());
        let mut old_path = encode_path_envelope(&path).unwrap();
        old_path[..8].copy_from_slice(&0xf11d_70cc_600f_1ab1_u64.to_be_bytes());
        assert!(decode_path_envelope(&old_path).is_err());
    }

    #[test]
    fn generated_python_preserves_goal_path_and_planar_occupancy_receipts() {
        let mut goal = configured_frame_goal("robot/map");
        // An epoch-zero source timestamp is legitimate receipt data.
        goal.committed_map_source_timestamp_ns = 0;
        seal_goal_envelope(&mut goal).unwrap();
        let mut path = configured_frame_path(&goal, "robot/map");
        seal_path_envelope(&mut path).unwrap();
        let planar = crate::lcm::planar_navigation::PlanarNavigationMap {
            map_frame_generation: 1, runtime_generation: 7,
            session_id: goal.committed_map_session_id.clone(), session_generation: 3,
            map_revision: 9, map_boot_nonce: goal.committed_map_boot_nonce,
            map_commit_sequence: 29, map_source_timestamp_ns: 0,
            observation_revision: 2, source_sequence: 3, source_timestamp_ns: 1_000_000_001,
            cleared: false, grid: OccupancyGrid {
                header: Header { stamp_sec: 1, stamp_nsec: 1, frame_id: "robot/map".into(), ..Header::default() },
                info: MapMetaData { width: 1, height: 1, resolution: 0.1,
                    origin: Pose { orientation_xyzw: [0.0, 0.0, 0.0, 1.0], ..Pose::default() },
                    ..MapMetaData::default() }, data: vec![0],
            },
        };
        let planar_body = crate::lcm::planar_navigation::encode_body(&planar).unwrap();
        let cases = [
            ("GoalEnvelope", encode_goal_envelope(&goal).unwrap()),
            ("PathEnvelope", encode_path_envelope(&path).unwrap()),
            ("PlanarNavigationMap", with_fingerprint(crate::lcm::planar_navigation::FINGERPRINT, planar_body)),
        ];
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../lcmtypes/generated/python");
        for (name, wire) in cases {
            let result = std::process::Command::new("python3")
                .args(["-I", "-S", "-c", r#"
import sys
sys.path.insert(0, sys.argv[1])
import nuwax_msgs
name = sys.argv[2]
raw = bytes.fromhex(sys.argv[3])
message = getattr(nuwax_msgs, name).decode(raw)
if name == 'PlanarNavigationMap':
    assert message.schema_version == 3
    assert bytes(message.map_boot_nonce) == bytes([0x31] * 16)
    assert message.map_commit_sequence == 29 and message.map_source_timestamp_ns == 0
    assert message.source_timestamp_ns == 1_000_000_001
else:
    assert message.schema_version == 7
    assert bytes(message.committed_map_boot_nonce) == bytes([0x31] * 16)
    assert message.committed_map_commit_sequence == 29
    assert message.committed_map_source_timestamp_ns == 0
    assert message.committed_map_frame_id == 'robot/map'
assert message.encode() == raw
sys.stdout.buffer.write(message.encode())
"#])
                .arg(&root).arg(name).arg(hex(&wire)).output().unwrap();
            assert!(result.status.success(), "{}", String::from_utf8_lossy(&result.stderr));
            assert_eq!(result.stdout, wire);
        }
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
            committed_map_boot_nonce: goal.committed_map_boot_nonce,
            committed_map_commit_sequence: goal.committed_map_commit_sequence,
            committed_map_source_timestamp_ns: goal.committed_map_source_timestamp_ns,
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
    fn navigation_local_display_rejects_corrupt_sizes_and_coordinates() {
        let mut message = NavigationState { local_path: vec![[0.0, 1.0, 0.4]], ..Default::default() };
        let encoded = encode_navigation_state_body(&message).unwrap();
        let count_at = encoded.len() - 28;
        let mut bad_count = encoded.clone();
        bad_count[count_at..count_at + 4].copy_from_slice(&65537_i32.to_be_bytes());
        assert!(decode_navigation_state_body(&bad_count).is_err());
        assert!(decode_navigation_state_body(&encoded[..encoded.len() - 1]).is_err());
        let mut bad_coordinate = encoded;
        bad_coordinate[count_at + 4..count_at + 12].copy_from_slice(&f64::NAN.to_be_bytes());
        assert!(decode_navigation_state_body(&bad_coordinate).is_err());
        message.local_path[0][0] = f64::INFINITY;
        assert!(encode_navigation_state_body(&message).is_err());
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
            pose_source_timestamp_ns: 123_000_000_456,
            costmap_source_timestamp_ns: 122_000_000_000,
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
            local_path: vec![[0.5, 1.0, 0.4], [1.0, 2.0, 0.6]],
        };
        let encoded = encode_navigation_state(&navigation).unwrap();
        assert_eq!(&encoded[..8], &NAVIGATION_STATE_FINGERPRINT);
        assert_eq!(decode_navigation_state(&encoded).unwrap(), navigation);
        assert_eq!(
            decode_navigation_state_body(&encode_navigation_state_body(&navigation).unwrap())
                .unwrap(),
            navigation
        );

        let mut legacy_fingerprint = encoded.clone();
        legacy_fingerprint[..8].copy_from_slice(&0x402e_9704_f8d2_5156_u64.to_be_bytes());
        assert!(decode_navigation_state(&legacy_fingerprint).is_err());

        let mut legacy_body = encode_navigation_state_body(&navigation).unwrap();
        let receipt_bytes = [
            navigation.pose_source_timestamp_ns.to_be_bytes(),
            navigation.costmap_source_timestamp_ns.to_be_bytes(),
        ]
        .concat();
        let receipt_offset = legacy_body
            .windows(receipt_bytes.len())
            .position(|window| window == receipt_bytes)
            .expect("receipt fields in NavigationState body");
        legacy_body.drain(receipt_offset..receipt_offset + receipt_bytes.len());
        assert!(decode_navigation_state_body(&legacy_body).is_err());

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
