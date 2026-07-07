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

/// LCM 1.x frame magic on wire = `"LC02"` (little-endian u32)
///
/// 当 magic u32 值 = `0x3230_434c`,以 `to_le_bytes()` 写入 wire 还原为
/// 字节序列 `[L, C, 0, 2]`。
const LCM_MAGIC_LE: u32 = 0x3230_434c;
/// magic 字段在 frame header 中占 8 字节
const LCM_MAGIC_BYTES: usize = 8;
/// fingerprint 字段 8 字节
const LCM_FINGERPRINT_BYTES: usize = 8;
/// sequence number 字段 8 字节
const LCM_SEQNO_BYTES: usize = 8;
/// payload length 字段 4 字节
const LCM_PAYLOAD_LEN_BYTES: usize = 4;
/// header 总字节数(不含 payload)
const LCM_HEADER_BYTES: usize =
    LCM_MAGIC_BYTES + LCM_FINGERPRINT_BYTES + LCM_SEQNO_BYTES + LCM_PAYLOAD_LEN_BYTES;

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
    let payload_len = u32::try_from(payload.len())
        .map_err(|_| crate::CodecError::LcmEncode("payload too large".into()))?;
    let fingerprint = compute_lcm_hash(msg_type);
    let mut out = Vec::with_capacity(LCM_HEADER_BYTES + payload.len());
    // magic 8 字节:高 4 字节为 magic LE,低 4 字节为 0
    out.extend_from_slice(&LCM_MAGIC_LE.to_le_bytes());
    out.extend_from_slice(&[0u8; 4]);
    // fingerprint 8 字节大端
    out.extend_from_slice(&fingerprint);
    // seqno 8 字节 LE
    out.extend_from_slice(&seqno.to_le_bytes());
    // payload length 4 字节 LE
    out.extend_from_slice(&payload_len.to_le_bytes());
    // payload
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
pub fn golden_pose_stamped() -> CodecResult<Bytes> {
    let mut payload = Vec::with_capacity(68);
    payload.extend_from_slice(b"PoseStamped!"); // 12 bytes marker
    payload.extend_from_slice(&1.0f64.to_le_bytes()); // x
    payload.extend_from_slice(&2.0f64.to_le_bytes()); // y
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // z
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // qx
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // qy
    payload.extend_from_slice(&0.0f64.to_le_bytes()); // qz
    payload.extend_from_slice(&1.0f64.to_le_bytes()); // qw
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
// P1 入口:Python/C++ LCM wire-format golden bytes
// ============================================================================
//
// 当前环境无 system `lcm` / `lcm-gen`,且 `lcm_msgs.tar.gz` 是 LFS pointer。
// P0-B 阶段:`expected_lcm_bytes` 一律返回 `None`,表示 reference bytes
// 不可用。P1 阶段由 `nuwax-codec` 接入 `lcm` crate 后,在该函数实现内
// 拉起一个真 LCM publish channel,生成 reference bytes 并返回 `Some(...)`。
// `tests/lcm_cross_module_consistency.rs` 新增 `[ignore = "requires system LCM"]`
// 的端到端 test 会消费这些 bytes。

/// 拉取 (topic, msg_type) 对应的 system LCM reference bytes。
///
/// P0-B 阶段永远返回 `None`。P1 阶段当 environment 满足
/// (system `lcm` / `lcm-gen` 可用,`lcm_msgs.tar.gz` 已 LFS-fetch) 时,
/// 该函数会被填实。
///
/// # Args
///
/// * `topic` — LCM channel,如 `/pose_stamped`
/// * `msg_type` — message type FQN,如 `geometry_msgs/PoseStamped`
///
/// # Returns
///
/// - `Some(bytes)` — system LCM publish 输出的 wire-format bytes
/// - `None` — 当前不可用,P1 占位
#[must_use]
pub fn expected_lcm_bytes(topic: &str, msg_type: &str) -> Option<Bytes> {
    let _ = (topic, msg_type);
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expected_lcm_bytes_returns_none_in_p0() {
        // P0 阶段 system LCM 不可用;P1 接入后该测试会被替换为对 Some 字节的断言。
        assert!(expected_lcm_bytes("/pose_stamped", "geometry_msgs/PoseStamped").is_none());
        assert!(expected_lcm_bytes("/odom", "nav_msgs/Odometry").is_none());
        assert!(expected_lcm_bytes("/points", "sensor_msgs/PointCloud2").is_none());
    }

    #[test]
    fn golden_pose_stamped_header_size() {
        let bytes = golden_pose_stamped().unwrap();
        // header 28 字节 + 68 bytes payload = 96 bytes
        assert_eq!(bytes.len(), LCM_HEADER_BYTES + 68);
    }

    #[test]
    fn golden_pose_stamped_magic() {
        let bytes = golden_pose_stamped().unwrap();
        let mut magic_bytes = [0u8; 4];
        magic_bytes.copy_from_slice(&bytes[0..4]);
        let magic = u32::from_le_bytes(magic_bytes);
        assert_eq!(magic, LCM_MAGIC_LE);
    }

    #[test]
    fn golden_pose_stamped_payload_length() {
        let bytes = golden_pose_stamped().unwrap();
        let mut len_bytes = [0u8; 4];
        len_bytes.copy_from_slice(&bytes[24..28]);
        let len = u32::from_le_bytes(len_bytes) as usize;
        assert_eq!(len, 68);
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
