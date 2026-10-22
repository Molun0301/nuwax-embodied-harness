// Phase 6 (codex-round3 R3-4):POSE_STAMPED_PAYLOAD_HEADER_BYTES /
// decode_odometry_payload / decode_nav_state_payload / decode_tf_payload /
// decode_frontiers_payload 是 typed_codec payload 解码入口,目前 typed_codec
// 走 Encode-only contract(写入侧),解码侧留给 Phase 6.1 WebSocket 接管。
#![allow(dead_code)]

//! LCM typed encode/decode —— 替换 `server.rs` / `lcm_runtime.rs` 中的
//! ad-hoc bytes 段。
//!
//! ## 设计口径
//! - 默认协议层 = LCM(`nuwax-protocol::RawLcmFrame`)
//! - typed payload 编码与 `lcm_runtime.rs` 中各 `parse_*_payload` 镜像
//!   对账;布局保持完全一致,只把字节构造从内联 `extend_from_slice` 收敛
//!   到此处
//! - fingerprint 仍走 FNV-1a 64-bit(等价于
//!   `nuwax_protocol::LcmMessageType::fingerprint`);P1 切 MurmurHash64A
//!   时本模块作为唯一迁移入口
//! - 禁止 `server.rs` / `lcm_runtime.rs` 继续写裸
//!   `payload.extend_from_slice(...)`;新增 typed message 必须在本文件
//!   加 `encode_*` / `decode_*` 函数并被对应位置调用
//!
//! ## 与 `nuwax-codec::lcm::golden_*` 的关系
//! `golden_pose_stamped` / `golden_odometry` / `golden_point_cloud2` 是
//! **测试 fixture**(确定性 bytes,用于跨模块 byte-for-byte 对账测试);
//! 运行时 typed payload 不复用 fixture 路径(那只是 12 字节 marker),而是
//! 用本文件的 typed encode 真实序列化。
//!
//! ## P1 切换入口
//! 当上游 `nuwax-codec` 切换到 MurmurHash64A 时,把本模块 fingerprint
//! 计算从 `LcmMessageType::fingerprint` 切到 `nuwax_codec::compute_hash`;
//! 调用方接口不变。

use nuwax_codec::lcm::schema;
use nuwax_codec::{CodecError, CodecResult};
use nuwax_protocol::{LcmFrameHeader, LcmMessageType, LcmTopic, RawLcmFrame, LCM_MAGIC};

use crate::lcm_runtime::{
    FrontierEntry, NavState, NavigationFeedback, NavigationUpdate, PoseState, TransformEntry,
};

// ============================================================================
// typed payload layout 常量(必须与 lcm_runtime.rs parse_*_payload 一致)
// ============================================================================

/// `/cmd_vel` typed payload 布局:
/// `6×f64` (linear xyz + angular xyz)，与生成的 `geometry_msgs.Twist` 一致。
pub const CMD_VEL_PAYLOAD_BYTES: usize = 48;

// ============================================================================
// encode 端
// ============================================================================

// Phase 4:`topic_or_default(channel, default)` 已废弃 —— 它把 typed suffix
// 缺失的 channel 静默 fallback 到 default，是 untyped channel 与 typed
// channel 共存的过渡设计。Phase 4 起 channel 构造统一走
// `channel_registry::channel_for(ChannelKey)` 公共 helper（保证 100% typed
// suffix），不再提供 untyped fallback helper；旧调用方已迁移到 channel_for。

/// 把 PoseState 编码成 odometry typed payload bytes。
///
/// 字节布局与 `lcm_runtime::parse_odom_payload` 完全镜像:
/// `7×f64 + u64 stamp + u16 frame_len + frame bytes`。
pub fn encode_odometry_payload(pose: &PoseState, child_frame_id: &str) -> CodecResult<Vec<u8>> {
    let child_frame_id = child_frame_id.trim();
    if child_frame_id.is_empty() {
        return Err(CodecError::LcmEncode(
            "odometry child_frame_id must not be empty".into(),
        ));
    }
    schema::encode_odometry_body(&schema::Odometry {
        header: header_from_pose(pose)?,
        child_frame_id: child_frame_id.to_string(),
        pose: schema_pose(pose),
        ..schema::Odometry::default()
    })
}

/// 把 PointCloud2 raw bytes + point_step 编码成 typed payload。
///
/// 字节布局:`u32 point_step + raw_point_cloud_bytes`。
pub fn encode_pointcloud_payload(points: &[u8], point_step: u32) -> CodecResult<Vec<u8>> {
    encode_pointcloud_payload_with_header(points, point_step, 0, 0, "map")
}

/// Encode a committed cumulative map snapshot for the canonical typed
/// `/global_map#sensor_msgs.PointCloud2` transport boundary.
pub fn encode_global_map_pointcloud_payload(
    xyz_f32_le: &[u8],
    revision: u64,
    timestamp_ns: u64,
) -> CodecResult<Vec<u8>> {
    encode_global_map_pointcloud_payload_in_frame(xyz_f32_le, revision, timestamp_ns, "map")
}

pub fn encode_global_map_pointcloud_payload_in_frame(
    xyz_f32_le: &[u8],
    revision: u64,
    timestamp_ns: u64,
    frame_id: &str,
) -> CodecResult<Vec<u8>> {
    let sequence = i32::try_from(revision)
        .map_err(|_| CodecError::LcmEncode("cloud-map revision exceeds int32".into()))?;
    if sequence <= 0 {
        return Err(CodecError::LcmEncode(
            "cloud-map revision must be positive".into(),
        ));
    }
    if frame_id.is_empty() || frame_id.trim() != frame_id {
        return Err(CodecError::LcmEncode(
            "cloud-map frame_id must be non-empty without surrounding whitespace".into(),
        ));
    }
    encode_pointcloud_payload_with_header(xyz_f32_le, 12, sequence, timestamp_ns, frame_id)
}

fn encode_pointcloud_payload_with_header(
    points: &[u8],
    point_step: u32,
    sequence: i32,
    timestamp_ns: u64,
    frame_id: &str,
) -> CodecResult<Vec<u8>> {
    if point_step < 12 || points.len() % point_step as usize != 0 {
        return Err(CodecError::LcmEncode(format!(
            "point cloud data length {} is not aligned to point_step {}",
            points.len(),
            point_step
        )));
    }
    let width = i32::try_from(points.len() / point_step as usize)
        .map_err(|_| CodecError::LcmEncode("point cloud width exceeds int32".into()))?;
    let stamp_sec = i32::try_from(timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("point cloud timestamp seconds exceed int32".into()))?;
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema::Header {
            seq: sequence,
            stamp_sec,
            stamp_nsec: (timestamp_ns % 1_000_000_000) as i32,
            frame_id: frame_id.to_string(),
        },
        height: 1,
        width,
        fields: vec![
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
        ],
        is_bigendian: false,
        point_step: point_step as i32,
        row_step: i32::try_from(points.len())
            .map_err(|_| CodecError::LcmEncode("point cloud row_step exceeds int32".into()))?,
        data: points.to_vec(),
        is_dense: true,
    })
}

/// 把 `geometry_msgs/Twist` 编码成 typed payload bytes。
pub fn encode_cmd_vel_payload(linear: [f32; 3], angular: [f32; 3], _stamp_ns: u64) -> Vec<u8> {
    schema::encode_twist_body(&schema::Twist {
        linear: linear.map(f64::from),
        angular: angular.map(f64::from),
    })
}

/// 把用户触发的 `nuwax_msgs/CancelNavigation` 编码成 generated-C body。
pub fn encode_nav_cancel_payload(
    session_id: &str,
    reason: i32,
    cancel_generation: u64,
    owner_kind: i32,
    parent_request_id: &str,
    goal_generation: u64,
    producer_runtime_generation: u64,
) -> CodecResult<Vec<u8>> {
    let cancel_generation = i64::try_from(cancel_generation).map_err(|_| {
        CodecError::LcmEncode("cancel_generation exceeds positive int64 range".into())
    })?;
    if cancel_generation == 0 {
        return Err(CodecError::LcmEncode(
            "cancel_generation must be positive".into(),
        ));
    }
    let goal_generation = i64::try_from(goal_generation).map_err(|_| {
        CodecError::LcmEncode("goal_generation exceeds positive int64 range".into())
    })?;
    let producer_runtime_generation = i64::try_from(producer_runtime_generation).map_err(|_| {
        CodecError::LcmEncode("producer_runtime_generation exceeds positive int64 range".into())
    })?;
    if producer_runtime_generation == 0 {
        return Err(CodecError::LcmEncode(
            "producer_runtime_generation must be positive".into(),
        ));
    }
    if session_id.trim().is_empty()
        || parent_request_id.trim().is_empty()
        || goal_generation == 0
        || !matches!(
            owner_kind,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
                | schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
        )
    {
        return Err(CodecError::LcmEncode(
            "CancelNavigation requires a complete goal lease identity".into(),
        ));
    }
    schema::encode_cancel_navigation_body(&schema::CancelNavigation {
        session_id: session_id.to_string(),
        reason,
        cancel_generation,
        owner_kind,
        parent_request_id: parent_request_id.to_string(),
        goal_generation,
        producer_runtime_generation,
    })
}

/// Encode one validated, sealed `nuwax_msgs.GoalEnvelope` generated-C body.
pub fn encode_goal_envelope_payload(message: &schema::GoalEnvelope) -> CodecResult<Vec<u8>> {
    schema::encode_goal_envelope_body(message)
}

/// Decode and validate a `nuwax_msgs.GoalEnvelope` generated-C body.
pub fn decode_goal_envelope_payload(payload: &[u8]) -> CodecResult<schema::GoalEnvelope> {
    schema::decode_goal_envelope_body(payload)
}

/// Decode and validate a `nuwax_msgs.PathEnvelope` generated-C body.
pub fn decode_path_envelope_payload(payload: &[u8]) -> CodecResult<schema::PathEnvelope> {
    schema::decode_path_envelope_body(payload)
}

/// 把 `geometry_msgs/PoseStamped` 编码成 typed payload bytes(复用 odom 布局)。
pub fn encode_pose_stamped_payload(pose: &PoseState) -> CodecResult<Vec<u8>> {
    schema::encode_pose_stamped_body(&schema::PoseStamped {
        header: header_from_pose(pose)?,
        pose: schema_pose(pose),
    })
}

fn header_from_pose(pose: &PoseState) -> CodecResult<schema::Header> {
    let stamp_sec = i32::try_from(pose.timestamp_ns / 1_000_000_000)
        .map_err(|_| CodecError::LcmEncode("pose timestamp seconds exceed int32".into()))?;
    Ok(schema::Header {
        seq: 0,
        stamp_sec,
        stamp_nsec: (pose.timestamp_ns % 1_000_000_000) as i32,
        frame_id: pose.frame_id.clone(),
    })
}

fn schema_pose(pose: &PoseState) -> schema::Pose {
    schema::Pose {
        position: [pose.x, pose.y, pose.z],
        orientation_xyzw: [pose.qx, pose.qy, pose.qz, pose.qw],
    }
}

/// 用 typed payload + 强类型 topic / msg_type 构造 `RawLcmFrame`。
///
/// fingerprint 取 `LcmMessageType::fingerprint`(基于 FNV-1a 64-bit,与
/// `nuwax-codec::compute_hash` 等价;P1 切 MurmurHash64A 时本函数是
/// 唯一改点)。
pub fn build_typed_frame(
    topic: LcmTopic,
    msg_type: LcmMessageType,
    payload: Vec<u8>,
    seqno: u64,
) -> RawLcmFrame {
    // W41 (Bug #1): `LcmFrameHeader` 当前定义仅含 `magic + seqno`(8 字节
    // preamble,严格对齐 liblcm 1.x UDP multicast wire format,见
    // `nuwax-protocol/src/lcm/frame.rs:44-54`);旧实现里多余的
    // `fingerprint` / `payload_len` / `protocol_major` / `protocol_minor`
    // 字段全部删除。
    // - `fingerprint`:不进 wire,改由 `LcmTopic` typed suffix 表达
    //   (见 `LcmTopic::as_subscription_pattern`);进程内仍可在
    //   `RawLcmFrame.msg_type` 上访问,无需经 header
    // - `payload_len`:不写入 wire,接收端用 `rbuf.data_size` 推出
    // - `protocol_*`:real LCM 1.x wire 无此字段,本期不引入
    let header = LcmFrameHeader {
        magic: LCM_MAGIC,
        seqno: seqno as u32,
    };
    RawLcmFrame {
        topic,
        msg_type,
        header,
        payload,
    }
}

// ============================================================================
// decode 端
// ============================================================================

/// 把 raw bytes 解析成 `PoseState`。失败返回 `None`,调用方按需 warn。
///
/// 镜像 `lcm_runtime::parse_odom_payload`;`server.rs` 旧逻辑完全迁移到
/// 本函数后,`lcm_runtime::parse_odom_payload` 可在 P3 标注 `pub(super)`
/// 收敛。
pub fn decode_odometry_payload(payload: &[u8]) -> Option<PoseState> {
    let mut encoded = Vec::with_capacity(8 + payload.len());
    encoded.extend_from_slice(&schema::ODOMETRY_FINGERPRINT);
    encoded.extend_from_slice(payload);
    let message = schema::decode_odometry(&encoded).ok()?;
    if message.header.stamp_sec < 0 || message.header.stamp_nsec < 0 {
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
        frame_id: message.header.frame_id,
        timestamp_ns: message.header.stamp_sec as u64 * 1_000_000_000
            + message.header.stamp_nsec as u64,
    })
}

/// 把 raw bytes 解析成 `NavState`(P4 接入后给 `nav_state` 解析路径用)。
pub fn decode_nav_state_payload(payload: &[u8]) -> Option<NavigationUpdate> {
    let message = schema::decode_navigation_state_body(payload).ok()?;
    let timestamp_ns = native_stamp_ns(message.stamp_sec, message.stamp_nsec)?;
    let dx = message.goal[0] - message.current[0];
    let dy = message.goal[1] - message.current[1];
    let distance_remaining = dx.hypot(dy);
    if !distance_remaining.is_finite() || !message.progress.is_finite() {
        return None;
    }
    let runtime_generation = u64::try_from(message.runtime_generation).ok()?;
    let guard_generation = u64::try_from(message.guard_generation).ok()?;
    let cancel_generation = u64::try_from(message.cancel_generation).ok()?;
    let zero_applied_monotonic_ns = u64::try_from(message.zero_applied_monotonic_ns).ok()?;
    let goal_generation = u64::try_from(message.goal_generation).ok()?;
    let path_revision = u64::try_from(message.path_revision).ok()?;
    let producer_runtime_generation = u64::try_from(message.producer_runtime_generation).ok()?;
    let planner_runtime_generation = u64::try_from(message.planner_runtime_generation).ok()?;
    let sink_ready = match message.sink_ready {
        0 => false,
        1 => true,
        _ => return None,
    };
    let has_session = !message.session_id.trim().is_empty();
    if has_session {
        if (sink_ready && guard_generation == 0)
            || !matches!(
                message.owner_kind,
                schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
                    | schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
            )
            || message.parent_request_id.trim().is_empty()
            || goal_generation == 0
            || message.producer_runtime_generation == 0
            || (path_revision == 0 && message.planner_runtime_generation != 0)
            || (path_revision > 0 && message.planner_runtime_generation == 0)
        {
            return None;
        }
    } else if message.owner_kind != 0
        || !message.parent_request_id.is_empty()
        || goal_generation != 0
        || path_revision != 0
        || message.producer_runtime_generation != 0
        || message.planner_runtime_generation != 0
    {
        return None;
    }
    // 只有调度器明确的未授予租约终态可以省略停车证明；后半段保留守卫原始原因。
    let lease_not_granted = message.state == 4 && message.message.starts_with("LEASE_NOT_GRANTED: ");
    if lease_not_granted && (!has_session || guard_generation == 0 || path_revision != 0
        || planner_runtime_generation != 0 || !message.zero_ack_id.is_empty()
        || zero_applied_monotonic_ns != 0) {
        return None;
    }
    let state = NavState {
        status: navigation_state_name(message.state, &message.message),
        message: message.message,
        goal_id: message.session_id,
        distance_remaining: distance_remaining as f32,
        timestamp_ns,
    };
    Some(NavigationUpdate {
        producer_runtime_generation,
        feedback: NavigationFeedback {
            reason_code: if lease_not_granted { "LEASE_NOT_GRANTED".into() }
                else { navigation_state_name(message.state, &state.message) },
            runtime_generation,
            guard_generation,
            sink_ready,
            planner_map_frame: message.planner_map_frame,
            planner_pose_frame: message.planner_pose_frame,
            cancel_generation,
            zero_ack_id: message.zero_ack_id,
            zero_applied_monotonic_ns,
            owner_kind: message.owner_kind,
            parent_request_id: message.parent_request_id,
            goal_generation,
            path_revision,
            producer_runtime_generation,
            planner_runtime_generation,
            local_path: if message.state == 2 { message.local_path } else { Vec::new() },
            ..NavigationFeedback::default()
        },
        state,
    })
}

/// 把 raw bytes 解析成 `Vec<TransformEntry>`(给 /tf typed channel 用)。
///
/// 镜像 `lcm_runtime::parse_tf_payload` 完整字段;P3 接入后
/// `lcm_runtime::parse_tf_payload` 改 `pub(super)`。
pub fn decode_tf_payload(payload: &[u8]) -> Option<Vec<TransformEntry>> {
    schema::decode_tf_message_body(payload)
        .ok()?
        .transforms
        .into_iter()
        .map(|transform| {
            Some(TransformEntry {
                parent: transform.header.frame_id,
                child: transform.child_frame_id,
                tx: transform.transform.translation[0],
                ty: transform.transform.translation[1],
                tz: transform.transform.translation[2],
                rx: transform.transform.rotation_xyzw[0],
                ry: transform.transform.rotation_xyzw[1],
                rz: transform.transform.rotation_xyzw[2],
                rw: transform.transform.rotation_xyzw[3],
                stamp_ns: native_stamp_ns(
                    i64::from(transform.header.stamp_sec),
                    transform.header.stamp_nsec,
                )?,
            })
        })
        .collect()
}

/// 把 raw bytes 解析成 `Vec<FrontierEntry>`(给 /explore/frontiers typed channel 用)。
pub fn decode_frontiers_payload(payload: &[u8]) -> Option<Vec<FrontierEntry>> {
    let message = schema::decode_frontier_list_body(payload).ok()?;
    message
        .frontiers
        .into_iter()
        .take(message.frontiers_length)
        .map(|frontier| {
            Some(FrontierEntry {
                x: frontier.center[0],
                y: frontier.center[1],
                z: 0.0,
                qx: 0.0,
                qy: 0.0,
                qz: 0.0,
                qw: 1.0,
                frame_id: message.frame_id.clone(),
                size_cells: u32::try_from(frontier.cell_count).ok()?,
                heading: 0.0,
            })
        })
        .collect()
}

fn native_stamp_ns(stamp_sec: i64, stamp_nsec: i32) -> Option<u64> {
    if stamp_sec < 0 || !(0..1_000_000_000).contains(&stamp_nsec) {
        return None;
    }
    (stamp_sec as u64)
        .checked_mul(1_000_000_000)?
        .checked_add(stamp_nsec as u64)
}

fn navigation_state_name(state: i32, message: &str) -> String {
    match state {
        0 if message.trim_start().starts_with("CANCELLED:") => "CANCELLED".into(),
        0 => "IDLE".into(),
        1 => "PLANNING".into(),
        2 => "RUNNING".into(),
        3 => "SUCCEEDED".into(),
        4 => "FAILED".into(),
        value => format!("UNKNOWN({value})"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::Command;

    const UV: &str = "/home/ml/.local/bin/uv";

    fn sample_pose() -> PoseState {
        PoseState {
            x: 1.0,
            y: 2.0,
            z: 0.0,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
            frame_id: "map".to_string(),
            timestamp_ns: 12345,
        }
    }

    #[test]
    fn encode_decode_odometry_roundtrip() {
        let pose = sample_pose();
        let bytes = encode_odometry_payload(&pose, "go2_base_link").expect("encode");
        let decoded = decode_odometry_payload(&bytes).expect("decode");
        assert_eq!(decoded.x, pose.x);
        assert_eq!(decoded.y, pose.y);
        assert_eq!(decoded.z, pose.z);
        assert_eq!(decoded.qw, pose.qw);
        assert_eq!(decoded.frame_id, pose.frame_id);
        assert_eq!(decoded.timestamp_ns, pose.timestamp_ns);

        let mut encoded = schema::ODOMETRY_FINGERPRINT.to_vec();
        encoded.extend_from_slice(&bytes);
        let typed = schema::decode_odometry(&encoded).expect("typed odometry");
        assert_eq!(typed.child_frame_id, "go2_base_link");
    }

    #[test]
    fn encode_pointcloud_layout() {
        let bytes = encode_pointcloud_payload(&[0xab; 16], 16).expect("encode");
        let mut encoded = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
        encoded.extend_from_slice(&bytes);
        let decoded = schema::decode_point_cloud2(&encoded).expect("decode");
        assert_eq!(decoded.width, 1);
        assert_eq!(decoded.height, 1);
        assert_eq!(decoded.point_step, 16);
        assert_eq!(decoded.data, vec![0xab; 16]);
        assert_eq!(decoded.fields.len(), 3);
    }

    #[test]
    fn encode_cmd_vel_layout() {
        let bytes = encode_cmd_vel_payload([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 999);
        assert_eq!(bytes.len(), CMD_VEL_PAYLOAD_BYTES);
        let mut encoded = schema::TWIST_FINGERPRINT.to_vec();
        encoded.extend_from_slice(&bytes);
        let decoded = schema::decode_twist(&encoded).expect("decode");
        assert_eq!(decoded.linear, [1.0, 2.0, 3.0]);
        assert!((decoded.angular[0] - 0.1_f32 as f64).abs() < f64::EPSILON);
    }

    #[test]
    fn native_bridge_payloads_use_generated_layouts() {
        let cancel = encode_nav_cancel_payload(
            "goal-7",
            2,
            17,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            "request-7",
            5,
            9,
        )
        .expect("encode cancellation");
        assert_eq!(
            schema::decode_cancel_navigation_body(&cancel).unwrap(),
            schema::CancelNavigation {
                session_id: "goal-7".into(),
                reason: 2,
                cancel_generation: 17,
                owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
                parent_request_id: "request-7".into(),
                goal_generation: 5,
                producer_runtime_generation: 9,
            }
        );

        let nav = schema::encode_navigation_state_body(&schema::NavigationState {
            session_id: "goal-7".into(),
            robot_frame: "base_link".into(),
            stamp_sec: 4,
            stamp_nsec: 5,
            state: 2,
            goal: [3.0, 4.0, 0.0],
            current: [0.0, 0.0, 0.0],
            progress: 0.5,
            message: "moving".into(),
            runtime_generation: 7,
            guard_generation: 11,
            pose_source_timestamp_ns: 4_000_000_005,
            costmap_source_timestamp_ns: 3_000_000_004,
            sink_ready: 1,
            planner_map_frame: "map".into(),
            planner_pose_frame: "map".into(),
            cancel_generation: 0,
            zero_ack_id: String::new(),
            zero_applied_monotonic_ns: 0,
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "request-7".into(),
            goal_generation: 5,
            path_revision: 0,
            producer_runtime_generation: 7,
            planner_runtime_generation: 0,
            local_path: vec![[0.0,0.0,0.4],[0.5,0.0,0.4]],
        })
        .unwrap();
        let nav = decode_nav_state_payload(&nav).expect("decode navigation state");
        assert_eq!(nav.state.status, "RUNNING");
        assert_eq!(nav.feedback.local_path, vec![[0.0,0.0,0.4],[0.5,0.0,0.4]]);
        assert_eq!(nav.state.goal_id, "goal-7");
        assert_eq!(nav.state.distance_remaining, 5.0);
        assert_eq!(nav.state.timestamp_ns, 4_000_000_005);
        assert_eq!(nav.feedback.runtime_generation, 7);
        assert_eq!(nav.feedback.guard_generation, 11);
        assert!(nav.feedback.sink_ready);
        assert_eq!(nav.feedback.planner_map_frame, "map");
        assert_eq!(nav.feedback.planner_pose_frame, "map");
        assert_eq!(
            nav.feedback.owner_kind,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV
        );
        assert_eq!(nav.feedback.parent_request_id, "request-7");
        assert_eq!(nav.feedback.goal_generation, 5);
        assert_eq!(nav.feedback.producer_runtime_generation, 7);

        let tf = schema::encode_tf_message_body(&schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 6,
                    stamp_nsec: 7,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "base_link".into(),
                transform: schema::Transform {
                    translation: [1.0, 2.0, 3.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .unwrap();
        let transforms = decode_tf_payload(&tf).expect("decode TFMessage");
        assert_eq!(transforms.len(), 1);
        assert_eq!(transforms[0].parent, "map");
        assert_eq!(transforms[0].child, "base_link");
        assert_eq!(transforms[0].stamp_ns, 6_000_000_007);

        let mut frontier_list = schema::FrontierList {
            frame_id: "map".into(),
            frontiers_length: 1,
            ..schema::FrontierList::default()
        };
        frontier_list.frontiers[0] = schema::Frontier {
            id: 9,
            center: [2.0, 3.0],
            radius: 1.5,
            score: 4.0,
            cell_count: 12,
            note: "candidate".into(),
        };
        let frontier_body = schema::encode_frontier_list_body(&frontier_list).unwrap();
        let frontiers = decode_frontiers_payload(&frontier_body).expect("decode frontier list");
        assert_eq!(frontiers.len(), 1);
        assert_eq!(frontiers[0].x, 2.0);
        assert_eq!(frontiers[0].y, 3.0);
        assert_eq!(frontiers[0].frame_id, "map");
        assert_eq!(frontiers[0].size_cells, 12);
        assert_eq!(frontiers[0].qw, 1.0);
    }

    #[test]
    fn stack_cancel_ack_decodes_as_terminal_cancelled_state() {
        let payload = schema::encode_navigation_state_body(&schema::NavigationState {
            session_id: "goal-cancelled".into(),
            robot_frame: "base_link".into(),
            stamp_sec: 8,
            stamp_nsec: 9,
            state: 0,
            goal: [1.0, 2.0, 0.0],
            current: [1.0, 2.0, 0.0],
            progress: 0.0,
            message: "CANCELLED: stop and zero command published".into(),
            runtime_generation: 9,
            guard_generation: 12,
            pose_source_timestamp_ns: 8_000_000_009,
            costmap_source_timestamp_ns: 7_000_000_008,
            sink_ready: 1,
            planner_map_frame: "map".into(),
            planner_pose_frame: "map".into(),
            cancel_generation: 3,
            zero_ack_id: "zero-3".into(),
            zero_applied_monotonic_ns: 55,
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "request-cancelled".into(),
            goal_generation: 3,
            path_revision: 1,
            producer_runtime_generation: 3,
            planner_runtime_generation: 4,
            local_path: vec![[0.0,0.0,0.4],[0.5,0.0,0.4]],
        })
        .expect("encode stack cancel ack");

        let nav = decode_nav_state_payload(&payload).expect("decode stack cancel ack");
        assert_eq!(nav.state.status, "CANCELLED");
        assert!(nav.feedback.local_path.is_empty());
        assert_eq!(nav.state.goal_id, "goal-cancelled");
        assert_eq!(nav.feedback.cancel_generation, 3);
        assert_eq!(nav.feedback.zero_ack_id, "zero-3");
        assert_eq!(nav.feedback.zero_applied_monotonic_ns, 55);
    }

    #[test]
    fn idle_sink_ready_without_guard_generation_decodes() {
        let payload = schema::encode_navigation_state_body(&schema::NavigationState {
            robot_frame: "base_link".into(),
            stamp_sec: 8,
            state: 0,
            message: "idle".into(),
            runtime_generation: 9,
            sink_ready: 1,
            ..schema::NavigationState::default()
        })
        .expect("encode idle navigation state");

        let nav = decode_nav_state_payload(&payload).expect("decode idle navigation state");
        assert_eq!(nav.state.status, "IDLE");
        assert!(nav.state.goal_id.is_empty());
        assert!(nav.feedback.sink_ready);
        assert_eq!(nav.feedback.guard_generation, 0);
    }

    #[test]
    fn active_sink_ready_without_guard_generation_is_rejected() {
        let payload = schema::encode_navigation_state_body(&schema::NavigationState {
            session_id: "goal-without-guard".into(),
            robot_frame: "base_link".into(),
            stamp_sec: 8,
            state: 2,
            message: "moving".into(),
            runtime_generation: 9,
            sink_ready: 1,
            owner_kind: schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            parent_request_id: "request-without-guard".into(),
            goal_generation: 1,
            producer_runtime_generation: 7,
            ..schema::NavigationState::default()
        })
        .expect("encode active navigation state");

        assert!(decode_nav_state_payload(&payload).is_none());
    }

    #[test]
    fn cancel_encoder_rejects_missing_generation() {
        let error = encode_nav_cancel_payload(
            "goal-7",
            0,
            0,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            "request-7",
            1,
            1,
        )
        .expect_err("zero must fail");
        assert!(error.to_string().contains("cancel_generation"));

        let error = encode_nav_cancel_payload(
            "goal-7",
            0,
            1,
            schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV,
            "request-7",
            1,
            0,
        )
        .expect_err("zero producer generation must fail");
        assert!(error.to_string().contains("producer_runtime_generation"));
    }

    #[test]
    fn versioned_envelope_decoders_reject_legacy_raw_goal_and_path_bodies() {
        let raw_goal = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            ..schema::PoseStamped::default()
        })
        .expect("raw PoseStamped");
        assert!(decode_goal_envelope_payload(&raw_goal).is_err());

        let raw_path = schema::encode_path_body(&schema::Path {
            header: schema::Header {
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            poses: vec![schema::PoseStamped {
                header: schema::Header {
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                ..schema::PoseStamped::default()
            }],
        })
        .expect("raw Path");
        assert!(decode_path_envelope_payload(&raw_path).is_err());
    }

    #[test]
    fn build_typed_frame_wire_size() {
        // Phase 4:直接用 `LcmTopic::new` typed 形式，不再走已废弃 `topic_or_default`。
        let topic = LcmTopic::new("/odom").expect("typed /odom");
        let msg_type = LcmMessageType::new("nav_msgs/Odometry").expect("typed");
        let pose = sample_pose();
        let payload = encode_odometry_payload(&pose, "base_link").expect("encode");
        let frame = build_typed_frame(topic, msg_type, payload.clone(), 1);
        // header + payload 一致
        assert_eq!(frame.payload, payload);
        // W41 (Bug #1 fix): `LcmFrameHeader` 已收紧为 `magic + seqno`,
        // `payload_len` 不写入 wire(由 `rbuf.data_size` 推出),
        // `fingerprint` 移至 `LcmTopic` typed suffix,不进 header。
        // 这里只校验 magic 正确 + seqno 截断正确(1 → 1)。
        assert_eq!(frame.header.magic, LCM_MAGIC);
        assert_eq!(frame.header.seqno, 1);
    }

    #[test]
    #[ignore = "requires uv and dimos-lcm==0.1.3"]
    fn geometry_publish_payloads_decode_with_locked_python_schemas() {
        assert_eq!(
            std::env::var("NUWAX_RUN_DIMOS_LCM_CROSS_LANG").as_deref(),
            Ok("1"),
            "set NUWAX_RUN_DIMOS_LCM_CROSS_LANG=1 to run the differential test"
        );

        fn wire_bytes(message_type: &str, body: Vec<u8>) -> String {
            let message = nuwax_protocol::build_lcm_message_bytes(
                "/geometry-differential",
                message_type,
                &body,
                0,
            )
            .expect("build generated LCM bytes");
            message
                .as_slice()
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect()
        }

        let pose = sample_pose();
        let pose_bytes = wire_bytes(
            "geometry_msgs.PoseStamped",
            encode_pose_stamped_payload(&pose).expect("encode PoseStamped"),
        );
        let twist_bytes = wire_bytes(
            "geometry_msgs.Twist",
            encode_cmd_vel_payload([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], 0),
        );
        let odom_bytes = wire_bytes(
            "nav_msgs.Odometry",
            encode_odometry_payload(&pose, "base_link").expect("encode Odometry"),
        );
        let pointcloud_bytes = wire_bytes(
            "sensor_msgs.PointCloud2",
            encode_pointcloud_payload(&[0; 16], 16).expect("encode PointCloud2"),
        );

        let output = Command::new(UV)
            .args([
                "run",
                "--isolated",
                "--no-project",
                "--no-config",
                "--with",
                "dimos-lcm==0.1.3",
                "python",
                "-c",
                "import sys\nfrom dimos_lcm.geometry_msgs.PoseStamped import PoseStamped\nfrom dimos_lcm.geometry_msgs.Twist import Twist\nfrom dimos_lcm.nav_msgs.Odometry import Odometry\nfrom dimos_lcm.sensor_msgs.PointCloud2 import PointCloud2\np=PoseStamped.lcm_decode(bytes.fromhex(sys.argv[1]))\nt=Twist.lcm_decode(bytes.fromhex(sys.argv[2]))\no=Odometry.lcm_decode(bytes.fromhex(sys.argv[3]))\nc=PointCloud2.lcm_decode(bytes.fromhex(sys.argv[4]))\nprint(p.header.frame_id,p.pose.position.x,p.pose.orientation.w,t.linear.y,t.angular.z,o.child_frame_id,o.pose.pose.position.y,c.width,c.point_step,c.fields[0].name,len(c.data),sep='|')",
                &pose_bytes,
                &twist_bytes,
                &odom_bytes,
                &pointcloud_bytes,
            ])
            .output()
            .expect("launch Python differential decoder");
        assert!(
            output.status.success(),
            "Python differential failed: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert_eq!(
            String::from_utf8(output.stdout)
                .expect("Python output is UTF-8")
                .trim(),
            "map|1.0|1.0|2.0|6.0|base_link|2.0|1|16|x|16"
        );
    }
}
