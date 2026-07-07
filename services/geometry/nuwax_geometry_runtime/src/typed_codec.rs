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

use nuwax_protocol::{LcmFrameHeader, LcmMessageType, LcmTopic, RawLcmFrame, LCM_MAGIC};

use crate::lcm_runtime::{
    FrontierEntry, NavState, PoseState, TransformEntry,
};

// ============================================================================
// typed payload layout 常量(必须与 lcm_runtime.rs parse_*_payload 一致)
// ============================================================================

/// Pose/Odometry typed payload 布局(共享):
/// `7×f64` (pos+quat) + `u64` stamp + `u16` frame_len + `frame` bytes
/// 总固定 66 字节 + frame。
pub const ODOMETRY_PAYLOAD_HEADER_BYTES: usize = 66;

/// `/cmd_vel` typed payload 布局:
/// `6×f32` (linear xyz + angular xyz) + `u64` stamp
pub const CMD_VEL_PAYLOAD_BYTES: usize = 32;

/// `/pose_stamped` typed payload 布局:复用 odom 布局
pub const POSE_STAMPED_PAYLOAD_HEADER_BYTES: usize = ODOMETRY_PAYLOAD_HEADER_BYTES;

/// `/nav/cancel` typed payload 布局:
/// `u16` goal_id_len + `goal_id` bytes + `u64` stamp
pub const NAV_CANCEL_HEADER_BYTES: usize = 2 + 8;

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
pub fn encode_odometry_payload(pose: &PoseState) -> Vec<u8> {
    let mut buf = Vec::with_capacity(ODOMETRY_PAYLOAD_HEADER_BYTES + pose.frame_id.len());
    buf.extend_from_slice(&pose.x.to_le_bytes());
    buf.extend_from_slice(&pose.y.to_le_bytes());
    buf.extend_from_slice(&pose.z.to_le_bytes());
    buf.extend_from_slice(&pose.qx.to_le_bytes());
    buf.extend_from_slice(&pose.qy.to_le_bytes());
    buf.extend_from_slice(&pose.qz.to_le_bytes());
    buf.extend_from_slice(&pose.qw.to_le_bytes());
    buf.extend_from_slice(&pose.timestamp_ns.to_le_bytes());
    let frame_len = u16::try_from(pose.frame_id.len()).unwrap_or(u16::MAX);
    buf.extend_from_slice(&frame_len.to_le_bytes());
    buf.extend_from_slice(pose.frame_id.as_bytes());
    buf
}

/// 把 PointCloud2 raw bytes + point_step 编码成 typed payload。
///
/// 字节布局:`u32 point_step + raw_point_cloud_bytes`。
pub fn encode_pointcloud_payload(points: &[u8], point_step: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(4 + points.len());
    buf.extend_from_slice(&point_step.to_le_bytes());
    buf.extend_from_slice(points);
    buf
}

/// 把 `geometry_msgs/Twist` 编码成 typed payload bytes。
pub fn encode_cmd_vel_payload(linear: [f32; 3], angular: [f32; 3], stamp_ns: u64) -> Vec<u8> {
    let mut buf = Vec::with_capacity(CMD_VEL_PAYLOAD_BYTES);
    for v in linear.iter().chain(angular.iter()) {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    buf.extend_from_slice(&stamp_ns.to_le_bytes());
    buf
}

/// 把 `nuwax_msgs/CancelNavigation` 编码成 typed payload bytes。
pub fn encode_nav_cancel_payload(goal_id: &str, stamp_ns: u64) -> Vec<u8> {
    let mut buf = Vec::with_capacity(NAV_CANCEL_HEADER_BYTES + goal_id.len());
    let goal_id_len = u16::try_from(goal_id.len()).unwrap_or(u16::MAX);
    buf.extend_from_slice(&goal_id_len.to_le_bytes());
    buf.extend_from_slice(goal_id.as_bytes());
    buf.extend_from_slice(&stamp_ns.to_le_bytes());
    buf
}

/// 把 `geometry_msgs/PoseStamped` 编码成 typed payload bytes(复用 odom 布局)。
pub fn encode_pose_stamped_payload(pose: &PoseState) -> Vec<u8> {
    encode_odometry_payload(pose)
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
    let header = LcmFrameHeader {
        magic: LCM_MAGIC,
        fingerprint: msg_type.fingerprint(),
        seqno,
        payload_len: payload.len() as u32,
        protocol_major: 0,
        protocol_minor: 0,
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
    if payload.len() < ODOMETRY_PAYLOAD_HEADER_BYTES {
        return None;
    }
    let x = f64::from_le_bytes(payload[0..8].try_into().ok()?);
    let y = f64::from_le_bytes(payload[8..16].try_into().ok()?);
    let z = f64::from_le_bytes(payload[16..24].try_into().ok()?);
    let qx = f64::from_le_bytes(payload[24..32].try_into().ok()?);
    let qy = f64::from_le_bytes(payload[32..40].try_into().ok()?);
    let qz = f64::from_le_bytes(payload[40..48].try_into().ok()?);
    let qw = f64::from_le_bytes(payload[48..56].try_into().ok()?);
    let stamp = u64::from_le_bytes(payload[56..64].try_into().ok()?);
    let frame_len = u16::from_le_bytes(payload[64..66].try_into().ok()?) as usize;
    if payload.len() < ODOMETRY_PAYLOAD_HEADER_BYTES + frame_len {
        return None;
    }
    let frame_id = String::from_utf8_lossy(
        &payload[ODOMETRY_PAYLOAD_HEADER_BYTES..ODOMETRY_PAYLOAD_HEADER_BYTES + frame_len],
    )
    .to_string();
    Some(PoseState {
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        frame_id,
        timestamp_ns: stamp,
    })
}

/// 把 raw bytes 解析成 `NavState`(P4 接入后给 `nav_state` 解析路径用)。
pub fn decode_nav_state_payload(payload: &[u8]) -> Option<NavState> {
    // 至少 66 字节 + 2 字节 status_len + ...(见 lcm_runtime::parse_nav_state 注释)
    if payload.len() < 66 {
        return None;
    }
    // 当前 lcm_runtime::subscribe nav_state 的复杂 layout 与 odom 同源
    // 字段复用;这里只导出 baseline stub,P4 接入后做完整镜像。
    Some(NavState::default())
}

/// 把 raw bytes 解析成 `Vec<TransformEntry>`(给 /tf typed channel 用)。
///
/// 镜像 `lcm_runtime::parse_tf_payload` 完整字段;P3 接入后
/// `lcm_runtime::parse_tf_payload` 改 `pub(super)`。
pub fn decode_tf_payload(payload: &[u8]) -> Option<Vec<TransformEntry>> {
    if payload.len() < 4 {
        return None;
    }
    let count = u32::from_le_bytes(payload[0..4].try_into().ok()?) as usize;
    let mut cursor = 4usize;
    let mut out = Vec::with_capacity(count);
    for _ in 0..count {
        if cursor + 2 > payload.len() {
            return None;
        }
        let p_len = u16::from_le_bytes(payload[cursor..cursor + 2].try_into().ok()?) as usize;
        cursor += 2;
        if cursor + p_len > payload.len() {
            return None;
        }
        let parent = String::from_utf8_lossy(&payload[cursor..cursor + p_len]).to_string();
        cursor += p_len;
        if cursor + 2 > payload.len() {
            return None;
        }
        let c_len = u16::from_le_bytes(payload[cursor..cursor + 2].try_into().ok()?) as usize;
        cursor += 2;
        if cursor + c_len > payload.len() {
            return None;
        }
        let child = String::from_utf8_lossy(&payload[cursor..cursor + c_len]).to_string();
        cursor += c_len;
        if cursor + 56 > payload.len() {
            return None;
        }
        let tx = f64::from_le_bytes(payload[cursor..cursor + 8].try_into().ok()?);
        let ty = f64::from_le_bytes(payload[cursor + 8..cursor + 16].try_into().ok()?);
        let tz = f64::from_le_bytes(payload[cursor + 16..cursor + 24].try_into().ok()?);
        let rx = f64::from_le_bytes(payload[cursor + 24..cursor + 32].try_into().ok()?);
        let ry = f64::from_le_bytes(payload[cursor + 32..cursor + 40].try_into().ok()?);
        let rz = f64::from_le_bytes(payload[cursor + 40..cursor + 48].try_into().ok()?);
        let rw = f64::from_le_bytes(payload[cursor + 48..cursor + 56].try_into().ok()?);
        cursor += 56;
        if cursor + 8 > payload.len() {
            return None;
        }
        let stamp = u64::from_le_bytes(payload[cursor..cursor + 8].try_into().ok()?);
        cursor += 8;
        out.push(TransformEntry {
            parent,
            child,
            tx,
            ty,
            tz,
            rx,
            ry,
            rz,
            rw,
            stamp_ns: stamp,
        });
    }
    Some(out)
}

/// 把 raw bytes 解析成 `Vec<FrontierEntry>`(给 /explore/frontiers typed channel 用)。
pub fn decode_frontiers_payload(payload: &[u8]) -> Option<Vec<FrontierEntry>> {
    if payload.len() < 4 {
        return None;
    }
    let count = u32::from_le_bytes(payload[0..4].try_into().ok()?) as usize;
    let mut cursor = 4usize;
    let mut entries = Vec::with_capacity(count);
    for _ in 0..count {
        if cursor + 56 + 2 > payload.len() {
            return None;
        }
        let x = f64::from_le_bytes(payload[cursor..cursor + 8].try_into().ok()?);
        let y = f64::from_le_bytes(payload[cursor + 8..cursor + 16].try_into().ok()?);
        let z = f64::from_le_bytes(payload[cursor + 16..cursor + 24].try_into().ok()?);
        let qx = f64::from_le_bytes(payload[cursor + 24..cursor + 32].try_into().ok()?);
        let qy = f64::from_le_bytes(payload[cursor + 32..cursor + 40].try_into().ok()?);
        let qz = f64::from_le_bytes(payload[cursor + 40..cursor + 48].try_into().ok()?);
        let qw = f64::from_le_bytes(payload[cursor + 48..cursor + 56].try_into().ok()?);
        cursor += 56;
        let f_len = u16::from_le_bytes(payload[cursor..cursor + 2].try_into().ok()?) as usize;
        cursor += 2;
        if cursor + f_len + 4 + 4 > payload.len() {
            return None;
        }
        let frame_id =
            String::from_utf8_lossy(&payload[cursor..cursor + f_len]).to_string();
        cursor += f_len;
        let size_cells =
            u32::from_le_bytes(payload[cursor..cursor + 4].try_into().ok()?);
        cursor += 4;
        let heading = f32::from_le_bytes(payload[cursor..cursor + 4].try_into().ok()?);
        cursor += 4;
        entries.push(FrontierEntry {
            x,
            y,
            z,
            qx,
            qy,
            qz,
            qw,
            frame_id,
            size_cells,
            heading,
        });
    }
    Some(entries)
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let bytes = encode_odometry_payload(&pose);
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
        let bytes = encode_pointcloud_payload(&[0xab; 12], 16);
        assert_eq!(bytes.len(), 4 + 12);
        let step = u32::from_le_bytes(bytes[0..4].try_into().unwrap());
        assert_eq!(step, 16);
        assert_eq!(&bytes[4..], &[0xab; 12]);
    }

    #[test]
    fn encode_cmd_vel_layout() {
        let bytes = encode_cmd_vel_payload([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 999);
        assert_eq!(bytes.len(), CMD_VEL_PAYLOAD_BYTES);
        // 前 24 字节 = 6×f32 linear + angular
        let linear_x = f32::from_le_bytes(bytes[0..4].try_into().unwrap());
        assert!((linear_x - 1.0).abs() < f32::EPSILON);
    }

    #[test]
    fn build_typed_frame_wire_size() {
        // Phase 4:直接用 `LcmTopic::new` typed 形式，不再走已废弃 `topic_or_default`。
        let topic = LcmTopic::new("/odom").expect("typed /odom");
        let msg_type = LcmMessageType::new("nav_msgs/Odometry").expect("typed");
        let pose = sample_pose();
        let payload = encode_odometry_payload(&pose);
        let frame = build_typed_frame(topic, msg_type, payload.clone(), 1);
        // header + payload 一致
        assert_eq!(frame.payload, payload);
        assert_eq!(frame.header.payload_len, payload.len() as u32);
        // fingerprint 非 0(FNV-1a hash)
        assert_ne!(frame.header.fingerprint, [0u8; 8]);
    }
}
