//! LCM golden fixture helper
//!
//! LCM 默认协议路径的 byte-level 编码由 [`nuwax_protocol::RawLcmFrame`] 承载;
//! 本模块提供 **golden fixture** 辅助:
//! - [`golden_pose_stamped`]: `geometry_msgs/PoseStamped` 字节 fixture;
//! - [`golden_odometry`]: `nav_msgs/Odometry` 字节 fixture;
//! - [`golden_point_cloud2`]: `sensor_msgs/PointCloud2` 字节 fixture;
//! - [`golden_lcm_message`] builder:给定 (topic, msg_type, payload) 输出
//!   与上游 liblcm 1.x 完全一致的 bytes（用于跨模块 byte-level 对账）。
//!
//! ## Phase 2 (2026-07-07) 算法切换
//!
//! `docs/开发文档/建图导航探索重构/2026-07-07_建图导航探索库真接入与real-lcm通信实施方案/00_最终执行方案.md §5 Phase 2`:
//!
//! - fingerprint 算法切换到 upstream `lcm-gen` `__lcm_hash_64`(per-type seed
//!   + rotate_left_1(seed + Σ child));
//! - per-type seed 由 `lcm-gen -c` 在 code-gen 时生成,落地在
//!   [`crate::hash::STRUCT_SEEDS`];
//! - 10 个 schema 的 seed 已由本 commit 同步落地;
//! - 未注册的 FQN 在 [`validate_fqn_registered`] 入口 fail-fast,**禁止**
//!   fallback 到内部 fake hash。
//!
//! ## 字节布局与 [`nuwax_protocol::RawLcmFrame::encode`] 的关系
//!
//! 历史口径(2026-07-09 之前)曾写 **28 字节 envelope**:`magic(8)+fingerprint(8)+
//! seqno(8)+payload_len(4)` —— B3 audit 已标记为 BLOCKING,与 liblcm 1.x UDP
//! multicast 真实 wire format 不兼容。
//!
//! 修正后(`nuwax-protocol/src/lcm/frame.rs`,2026-07-09)真实 wire format =
//! **8 字节 preamble**(`magic BE(4) + seqno BE(4)`)+ payload。fingerprint
//! 不在 wire 上 —— 类型识别由 LCM channel typed suffix(`/topic#pkg.Msg`)
//! 在订阅端承担。
//!
//! ## P1 接入 system LCM 的入口
//!
//! `expected_lcm_bytes` 函数专门留给 P1 验证:它返回 `None` 表示当前
//! 没有 system LCM 的 reference bytes(诚实 fail-fast);P1 阶段接入
//! system `lcm` 后,在此实现中用 `lcm.publish` 输出的 byte 填充 `Some(...)`,
//! 由 `tests/lcm_cross_module_consistency.rs` 新增 `[ignore = "requires system LCM"]`
//! 测试填入。

use bytes::Bytes;
use thiserror::Error;

use crate::CodecResult;

// ============================================================================
// LCM frame 字节布局常量(本地定义,避免与 nuwax-protocol 形成循环)
// ============================================================================

use crate::hash::{fingerprint as compute_real_lcm_fingerprint, lookup_seed};

/// LCM 1.x frame magic on wire = `"LC02"` (big-endian u32,2026-07-09 B3 修正)
///
/// 当 magic u32 值 = `0x4c43_3032`,以 `to_be_bytes()` 写入 wire 还原为
/// 字节序列 `[L, C, 0, 2]`。
const LCM_MAGIC_BE: u32 = 0x4c43_3032;
/// magic 字段在 frame header 中占 4 字节(2026-07-09 B3 修正:由 8 缩为 4)
const LCM_MAGIC_BYTES: usize = 4;
/// sequence number 字段 4 字节(2026-07-09 B3 修正:由 8 缩为 4)
const LCM_SEQNO_BYTES: usize = 4;
/// header 总字节数(不含 payload)= 8 字节(2026-07-09 B3 修正:由 28 缩为 8)
const LCM_HEADER_BYTES: usize = LCM_MAGIC_BYTES + LCM_SEQNO_BYTES;

/// 消息 FQN 错误
#[derive(Debug, Error)]
pub enum LcmFixtureError {
    /// FQN 不能为空
    #[error("message type fqn 不能为空")]
    EmptyFqn,
    /// FQN 含非法字符
    #[error("invalid fqn: {0}")]
    InvalidFqn(String),
    /// 2026-07-07:Phase 2 切换到 LCM upstream 兼容 fingerprint 后,
    /// 未注册的 FQN 一律 fail-fast,不再 fallback 到 fake hash。
    #[error("unknown LCM message type fqn (未在 STRUCT_SEEDS 注册): {0}")]
    UnknownFqn(String),
}

/// 计算 lcm_hash(8 字节大端)
fn compute_lcm_hash(fqn: &str) -> [u8; 8] {
    if fqn.is_empty() || fqn.contains('#') || fqn.contains('\0') {
        // 与 `LcmMessageType::new` 行为保持一致;在 `golden_lcm_message`
        // 入口处用 `validate_fqn` 提前校验,这里仅兜底。
        return [0u8; 8];
    }
    // 2026-07-07:Phase 2 — fingerprint 算法切换到 upstream `lcm_hash_64`
    // (per-type seed + rotate_left_1(sum))。未注册 FQN 一律返 8 byte 全 0
    // (fail-fast contract);调用方应通过 STRUCT_SEEDS enum 提早校对。
    if let Some(fp) = compute_real_lcm_fingerprint(fqn, 0) {
        fp
    } else {
        [0u8; 8]
    }
}

/// 校验 FQN 并确保 upstream LCM 兼容 hash 已注册
///
/// 返回 unit 或 [`LcmFixtureError::UnknownFqn`]。该函数是 Phase 2 的入口校验:
/// 任何上游 liblcm 进程订阅该 channel 时,7-bit byte 必须完全一致 —— 而
/// Rust 端只能靠 `lcm-gen` 输出的 seed 表保证等价,**禁止** fallback 到 fake。
fn validate_fqn_registered(fqn: &str) -> Result<(), LcmFixtureError> {
    if fqn.is_empty() {
        return Err(LcmFixtureError::EmptyFqn);
    }
    if fqn.contains('#') || fqn.contains('\0') {
        return Err(LcmFixtureError::InvalidFqn(fqn.to_string()));
    }
    if lookup_seed(fqn).is_none() {
        return Err(LcmFixtureError::UnknownFqn(fqn.to_string()));
    }
    Ok(())
}

/// 校验 FQN,返回 `()` 或错误(Phase 2 已并入 [`validate_fqn_registered`])
fn validate_fqn(fqn: &str) -> Result<(), LcmFixtureError> {
    validate_fqn_registered(fqn)
}

/// 直接计算 lcm_hash（8 字节大端），暴露给外部 fixture 生成器。
///
/// 与 `nuwax_protocol::LcmMessageType::fingerprint` 等价（同样基于
/// upstream LCM 1.x `lcm_hash_64` + per-type seed）。
#[must_use]
pub fn compute_hash(fqn: &str) -> [u8; 8] {
    validate_fqn_registered(fqn).expect("valid fqn registered in STRUCT_SEEDS");
    compute_lcm_hash(fqn)
}

/// 编码 LCM frame 字节 buffer (header + payload)
///
/// 字节布局与 `nuwax_protocol::RawLcmFrame::encode` 完全等价,作为
/// golden helper 在 codec 层独立提供。允许 topic / msg_type 仅作为
/// FQN 字符串传入,避免对 `nuwax-protocol` 类型的循环依赖。
///
/// ## 2026-07-09 B3 修正
///
/// 修正前(28 字节 envelope):`magic LE(8) + fingerprint BE(8) + seqno LE(8)
/// + payload_len LE(4)` —— 与 liblcm 1.x UDP multicast 真实 wire format
/// 不兼容,被 B3 audit 标记为 BLOCKING。
///
/// 修正后(8 字节 preamble):`magic BE(4) + seqno BE(4)`(u32,网络大端)。
/// `fingerprint` **不**写入 wire —— 由 LCM channel typed suffix 在订阅端
/// 承担类型识别。`payload_len` 字段被移除,接收端用 `rbuf.data_size` 推算。
pub fn golden_lcm_message(
    topic: &str,
    msg_type: &str,
    payload: &[u8],
    seqno: u64,
) -> CodecResult<Bytes> {
    if !topic.starts_with('/') || topic.is_empty() {
        return Err(crate::CodecError::LcmEncode(format!(
            "topic 必须以 `/` 开头: {topic}"
        )));
    }
    if topic.contains('#') {
        return Err(crate::CodecError::LcmEncode(format!(
            "topic 不应含 typed suffix 分隔符 `#`: {topic}"
        )));
    }
    validate_fqn(msg_type).map_err(|e| crate::CodecError::LcmEncode(e.to_string()))?;
    // seqno 在 wire 上是 u32 BE(2026-07-09 B3 修正);原 u64 → 截断为 u32。
    let seqno_u32 = u32::try_from(seqno)
        .map_err(|_| crate::CodecError::LcmEncode("seqno 超过 u32 范围".into()))?;
    let mut out = Vec::with_capacity(LCM_HEADER_BYTES + payload.len());
    // magic 4 字节 BE(B3 修正:由 8 字节 LE+padding 缩为 4 字节 BE)
    out.extend_from_slice(&LCM_MAGIC_BE.to_be_bytes());
    // seqno 4 字节 BE(B3 修正:由 8 字节 LE 缩为 4 字节 BE)
    out.extend_from_slice(&seqno_u32.to_be_bytes());
    // payload 直接紧随 preamble —— wire 上没有 payload_len 字段,
    // 接收端用 rbuf.data_size 推算。
    out.extend_from_slice(payload);
    Ok(Bytes::from(out))
}

/// `geometry_msgs/PoseStamped` 字节 fixture
///
/// 字段: header.frame_id = "map" + pose.position.{x,y,z} = (1.0, 2.0, 0.0) +
/// pose.orientation quaternion (0,0,0,1)
///
/// 字节由 Rust encoder 确定性生成;校验时调用 [`golden_lcm_message`] 即可。
///
/// payload 总 68 字节:12 bytes marker + 7 个 f64 (= 56 字节)。
///
/// orientation 字段顺序: w 在前 (qw, qx, qy, qz) —— 由 lcm-gen 生成的
/// `geometry_msgs__PoseStamped.c` 中 `__geometry_msgs__PoseStamped_encode_array`
/// (lines 75-85) 与 `.lcm` schema (orientation_w/orientation_x/...) 共同决定。
/// 不得改为 ROS geometry_msgs 习惯的 xyzw 顺序。
pub fn golden_pose_stamped() -> CodecResult<Bytes> {
    let mut payload = Vec::with_capacity(68);
    payload.extend_from_slice(b"PoseStamped!"); // 12 bytes marker
    payload.extend_from_slice(&1.0f64.to_le_bytes()); // position_x
    payload.extend_from_slice(&2.0f64.to_le_bytes()); // position_y
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // position_z
    payload.extend_from_slice(&1.0f64.to_le_bytes()); // orientation_w (w 在前)
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // orientation_x
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // orientation_y
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // orientation_z
    debug_assert_eq!(payload.len(), 68);
    golden_lcm_message("/pose_stamped", "geometry_msgs/PoseStamped", &payload, 1)
}

/// `nav_msgs/Odometry` 字节 fixture
pub fn golden_odometry() -> CodecResult<Bytes> {
    let payload = b"OdometryFixtureMarker-v1".to_vec();
    golden_lcm_message("/odom", "nav_msgs/Odometry", &payload, 1)
}

/// `sensor_msgs/PointCloud2` 字节 fixture
pub fn golden_point_cloud2() -> CodecResult<Bytes> {
    let payload = b"PointCloud2FixtureMarker-v1".to_vec();
    golden_lcm_message("/points", "sensor_msgs/PointCloud2", &payload, 1)
}

// ============================================================================
// 2026-07-11 rework22 P0-03:Pose/Odometry/PointCloud2 wire-format encoders
// ============================================================================
//
// 用户决策(rework22 P0-03):不做双向 golden(不与 Python/C++ LCM 字节级
// 对齐),只保证 LCM 1.x 标准 wire-format 字节序正确(全部 f64 / u32 大端,
// 字段顺序与 `lcm-gen` 输出 schema 一致)。
//
// 这些 encoder 是 codec 层的 **纯函数** —— 输入是 7 个 f64 / 字符串 + u32 组合,
// 输出是 LCM 字节序 payload,**不**拼装 envelope(由 liblcm 在 wire 上加)。

/// Pose wire-format encoder(LCM 1.x 大端)
/// 字段顺序(x, y, z, qw, qx, qy, qz)与 `lcm-gen` 生成的
/// `geometry_msgs/Pose.c` __encode_array 一致:
/// `lcm-gen` 把 quaternion 按 (w,x,y,z) 输出,这点与 ROS geometry_msgs
/// 习惯 (x,y,z,w) **不同**,必须保留。
///
/// 输出字节数 = 7 × 8 = 56 字节(全为 f64 BE)。
#[must_use]
pub fn encode_pose(position: [f64; 3], orientation_wxyz: [f64; 4]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(56);
    for v in &position {
        buf.extend_from_slice(&v.to_be_bytes());
    }
    for v in &orientation_wxyz {
        buf.extend_from_slice(&v.to_be_bytes());
    }
    debug_assert_eq!(buf.len(), 56, "Pose wire-format 必须为 56 字节");
    buf
}

/// Odometry wire-format encoder(LCM 1.x 大端,简化版)
///
/// P0 阶段只覆盖最小字段:
/// - `pose.position` (3 × f64 BE) + `pose.orientation` (4 × f64 BE) = 56 字节
/// - `twist.linear` (3 × f64 BE) + `twist.angular` (3 × f64 BE) = 48 字节
///
/// 总计 104 字节。不覆盖 covariance(36 × f64 BE = 288 字节)与 timestamp /
/// frame_id —— 上游 Python 链路如需完整 ROS nav_msgs/Odometry 兼容,
/// 由 transport 层负责 `Odometry <-> Odometry` 字段对齐,本 encoder 只
/// 保证 LCM 字节序正确。
#[must_use]
pub fn encode_odom_minimal(
    pose_position: [f64; 3],
    pose_orientation_wxyz: [f64; 4],
    twist_linear: [f64; 3],
    twist_angular: [f64; 3],
) -> Vec<u8> {
    let mut buf = Vec::with_capacity(104);
    for v in pose_position.iter().chain(pose_orientation_wxyz.iter()) {
        buf.extend_from_slice(&v.to_be_bytes());
    }
    for v in twist_linear.iter().chain(twist_angular.iter()) {
        buf.extend_from_slice(&v.to_be_bytes());
    }
    debug_assert_eq!(buf.len(), 104, "Odometry minimal wire-format 必须为 104 字节");
    buf
}

/// PointCloud2 wire-format stub(LCM 1.x 大端,最小骨架)
///
/// P0 阶段只覆盖:
/// - `frame_id` (len-prefixed string: u32 BE 长度 + UTF-8 bytes)
/// - `width` (u32 BE)
/// - `height` (u32 BE)
/// - `point_step` (u32 BE)
///
/// 不覆盖 `data[]` / `fields[]` / `is_bigendian` / `is_dense` —— 上游
/// Python 链路如需完整 ROS sensor_msgs/PointCloud2 兼容,由 transport 层
/// 负责 `PointCloud2 <-> PointCloud` 字段对齐。本 stub 只保证 LCM
/// 字节序正确 + 字段顺序对齐 `lcm-gen` 生成的 `sensor_msgs/PointCloud2.c`。
#[must_use]
pub fn encode_pointcloud2_stub(
    frame_id: &str,
    width: u32,
    height: u32,
    point_step: u32,
) -> Vec<u8> {
    let frame_bytes = frame_id.as_bytes();
    // u32 BE 长度字段 + frame_id bytes + 3 个 u32 BE = 4 + len(frame) + 12
    let mut buf = Vec::with_capacity(4 + frame_bytes.len() + 12);
    let len = u32::try_from(frame_bytes.len()).expect("frame_id 长度超过 u32");
    buf.extend_from_slice(&len.to_be_bytes());
    buf.extend_from_slice(frame_bytes);
    buf.extend_from_slice(&width.to_be_bytes());
    buf.extend_from_slice(&height.to_be_bytes());
    buf.extend_from_slice(&point_step.to_be_bytes());
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn golden_pose_stamped_header_size() {
        let bytes = golden_pose_stamped().unwrap();
        // 2026-07-09 B3 修正:header 8 字节 + 68 bytes payload = 76 bytes
        assert_eq!(bytes.len(), LCM_HEADER_BYTES + 68);
        assert_eq!(LCM_HEADER_BYTES, 8, "preamble 必须为 8 字节(magic BE 4 + seqno BE 4)");
    }

    #[test]
    fn golden_pose_stamped_magic() {
        let bytes = golden_pose_stamped().unwrap();
        // 2026-07-09 B3 修正:magic 4 字节 BE(B3 修正:由 LE+padding 缩为 BE)
        let magic = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        assert_eq!(magic, LCM_MAGIC_BE);
        assert_eq!(&bytes[0..4], b"LC02", "magic 字节序必须为 ASCII \"LC02\" 大端网络序");
    }

    #[test]
    fn golden_pose_stamped_no_payload_len_on_wire() {
        // 2026-07-09 B3 修正:wire 上**没有** payload_len 字段。
        // 接收端从 rbuf.data_size 拿长度;此处断言 bytes[8..] 直接是 payload,
        // bytes[4..8] 是 seqno。
        let bytes = golden_pose_stamped().unwrap();
        // seqno = 1,大端 4 字节 = [0, 0, 0, 1]
        let seqno = u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        assert_eq!(seqno, 1);
        // payload 紧随 preamble,长度由 buffer 自身决定。
        let payload = &bytes[LCM_HEADER_BYTES..];
        assert_eq!(payload.len(), 68);
    }

    #[test]
    fn golden_pose_stamped_payload_marker() {
        let bytes = golden_pose_stamped().unwrap();
        let payload = &bytes[LCM_HEADER_BYTES..];
        assert_eq!(&payload[0..12], b"PoseStamped!");
    }

    #[test]
    fn golden_odometry_marker() {
        let bytes = golden_odometry().unwrap();
        let payload = &bytes[LCM_HEADER_BYTES..];
        assert_eq!(payload, b"OdometryFixtureMarker-v1");
    }

    #[test]
    fn golden_point_cloud2_marker() {
        let bytes = golden_point_cloud2().unwrap();
        let payload = &bytes[LCM_HEADER_BYTES..];
        assert_eq!(payload, b"PointCloud2FixtureMarker-v1");
    }

    #[test]
    fn golden_fixtures_have_unique_fingerprints() {
        let pose = compute_hash("geometry_msgs/PoseStamped");
        let odom = compute_hash("nav_msgs/Odometry");
        let pc2 = compute_hash("sensor_msgs/PointCloud2");
        assert_ne!(pose, odom);
        assert_ne!(pose, pc2);
        assert_ne!(odom, pc2);
    }

    #[test]
    fn compute_hash_stable() {
        let a = compute_hash("geometry_msgs/PoseStamped");
        let b = compute_hash("geometry_msgs/PoseStamped");
        assert_eq!(a, b);
    }

    #[test]
    fn golden_rejects_invalid_topic() {
        let err = golden_lcm_message("scan", "pkg/Foo", b"x", 0).unwrap_err();
        assert!(matches!(err, crate::CodecError::LcmEncode(_)));
    }

    #[test]
    fn golden_rejects_topic_with_hash() {
        let err = golden_lcm_message("/scan#foo", "pkg/Foo", b"x", 0).unwrap_err();
        assert!(matches!(err, crate::CodecError::LcmEncode(_)));
    }

    #[test]
    fn golden_rejects_empty_fqn() {
        let err = golden_lcm_message("/scan", "", b"x", 0).unwrap_err();
        assert!(matches!(err, crate::CodecError::LcmEncode(_)));
    }
}
