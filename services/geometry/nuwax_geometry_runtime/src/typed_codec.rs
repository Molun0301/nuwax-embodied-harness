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

use crate::lcm_runtime::{FrontierEntry, NavState, PoseState, TransformEntry};

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
pub fn encode_odometry_payload(pose: &PoseState) -> CodecResult<Vec<u8>> {
    schema::encode_odometry_body(&schema::Odometry {
        header: header_from_pose(pose)?,
        child_frame_id: "base_link".to_string(),
        pose: schema_pose(pose),
        ..schema::Odometry::default()
    })
}

/// 把 PointCloud2 raw bytes + point_step 编码成 typed payload。
///
/// 字节布局:`u32 point_step + raw_point_cloud_bytes`。
pub fn encode_pointcloud_payload(points: &[u8], point_step: u32) -> CodecResult<Vec<u8>> {
    if point_step < 12 || points.len() % point_step as usize != 0 {
        return Err(CodecError::LcmEncode(format!(
            "point cloud data length {} is not aligned to point_step {}",
            points.len(),
            point_step
        )));
    }
    let width = i32::try_from(points.len() / point_step as usize)
        .map_err(|_| CodecError::LcmEncode("point cloud width exceeds int32".into()))?;
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema::Header {
            frame_id: "map".to_string(),
            ..schema::Header::default()
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
pub fn encode_nav_cancel_payload(session_id: &str, reason: i32) -> CodecResult<Vec<u8>> {
    schema::encode_cancel_navigation_body(&schema::CancelNavigation {
        session_id: session_id.to_string(),
        reason,
    })
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
pub fn decode_nav_state_payload(payload: &[u8]) -> Option<NavState> {
    let message = schema::decode_navigation_state_body(payload).ok()?;
    let timestamp_ns = native_stamp_ns(message.stamp_sec, message.stamp_nsec)?;
    let dx = message.goal[0] - message.current[0];
    let dy = message.goal[1] - message.current[1];
    let distance_remaining = dx.hypot(dy);
    if !distance_remaining.is_finite() || !message.progress.is_finite() {
        return None;
    }
    Some(NavState {
        status: navigation_state_name(message.state, &message.message),
        message: message.message,
        goal_id: message.session_id,
        distance_remaining: distance_remaining as f32,
        timestamp_ns,
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
        let bytes = encode_odometry_payload(&pose).expect("encode");
        let decoded = decode_odometry_payload(&bytes).expect("decode");
        assert_eq!(decoded.x, pose.x);
        assert_eq!(decoded.y, pose.y);
        assert_eq!(decoded.z, pose.z);
        assert_eq!(decoded.qw, pose.qw);
        assert_eq!(decoded.frame_id, pose.frame_id);
        assert_eq!(decoded.timestamp_ns, pose.timestamp_ns);
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
        let cancel = encode_nav_cancel_payload("goal-7", 2).expect("encode cancellation");
        assert_eq!(
            schema::decode_cancel_navigation_body(&cancel).unwrap(),
            schema::CancelNavigation {
                session_id: "goal-7".into(),
                reason: 2,
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
        })
        .unwrap();
        let nav = decode_nav_state_payload(&nav).expect("decode navigation state");
        assert_eq!(nav.status, "RUNNING");
        assert_eq!(nav.goal_id, "goal-7");
        assert_eq!(nav.distance_remaining, 5.0);
        assert_eq!(nav.timestamp_ns, 4_000_000_005);

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
        })
        .expect("encode stack cancel ack");

        let nav = decode_nav_state_payload(&payload).expect("decode stack cancel ack");
        assert_eq!(nav.status, "CANCELLED");
        assert_eq!(nav.goal_id, "goal-cancelled");
    }

    #[test]
    fn build_typed_frame_wire_size() {
        // Phase 4:直接用 `LcmTopic::new` typed 形式，不再走已废弃 `topic_or_default`。
        let topic = LcmTopic::new("/odom").expect("typed /odom");
        let msg_type = LcmMessageType::new("nav_msgs/Odometry").expect("typed");
        let pose = sample_pose();
        let payload = encode_odometry_payload(&pose).expect("encode");
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
            encode_odometry_payload(&pose).expect("encode Odometry"),
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
