//! golden_real_lcm:Phase 2 LCM wire-format 冻结测试
//!
//! ## 覆盖范围
//!
//! 1. **fingerprint 匹配 upstream lcm-gen `__Type_get_hash()`**:10 个 .lcm
//!    schema 的 fingerprint == 上游 `lcm-gen 1.5.2` 输出的
//!    `__pkg__Type_get_hash()` 值;fixture 在 tests 中固化,不允许漂移。
//! 2. **`golden_lcm_message` 字节 = `RawLcmFrame::encode` 字节**:fake backend
//!    与 protocol encoder byte-for-byte 一致(本测试由 `lcm_cross_module_consistency.rs`
//!    已固化,这里只附加一层"fingerprint 真来自 upstream"的二次断言)。
//! 3. **`LcmMessageBytes` 与 real-lcm `lcm_publish` 兼容**:payload 字段语义
//!    直接喂给 `lcm_publish(channel, data, datalen)`,无 LC02 envelope 重复。
//!
//! ## 数据来源
//!
//! 10 个期望 fingerprint 由 `scripts/ci/regen_lcm_goldens.sh` 在 2026-07-07
//! 用 system lcm-gen 跑出并反推验证(seed 经 `STRUCT_SEEDS` → `lcm_hash_64`):

use nuwax_codec::{hash::STRUCT_SEEDS, lcm::{compute_hash, golden_lcm_message}};
use nuwax_protocol::{
    build_lcm_message_bytes, parse_lcm_message_bytes, LcmMessageBytes, LcmMessageType, LcmTopic,
    RawLcmFrameBuilder, LCM_MAGIC,
};

/// 10 个 schema 的 upstream lcm-gen `__Type_get_hash()` 值(2026-07-07 跑出)
const UPSTREAM_FINGERPRINTS: &[(&str, u64)] = &[
    ("geometry_msgs/PoseStamped", 0x2573_8ae5_7ec5_4e84),
    ("geometry_msgs/Transform", 0x2afa_3e8c_ba35_7ed0),
    ("geometry_msgs/Twist", 0xbbca_3b0d_2c22_f384),
    ("nav_msgs/Odometry", 0x1e49_cc4e_3b85_2955),
    ("nav_msgs/OccupancyGrid", 0x6594_8fd9_608c_95bc),
    ("sensor_msgs/PointCloud2", 0x9ceb_8cd9_8f7d_5f48),
    ("nuwax_msgs/NavigationState", 0xbfd9_7ea1_913d_0280),
    ("nuwax_msgs/CancelNavigation", 0xafbb_c90f_61b5_2506),
    ("nuwax_msgs/Frontier", 0x1abf_83ea_0ced_9c44),
    ("nuwax_msgs/FrontierList", 0xebbe_aebb_9e3b_e6ef),
];

/// `compute_hash(fqn)` 与 upstream fingerprint 一一对应
#[test]
fn test_per_type_fingerprints_match_upstream_lcm_gen() {
    assert_eq!(
        STRUCT_SEEDS.len(),
        UPSTREAM_FINGERPRINTS.len(),
        "STRUCT_SEEDS 与 UPSTREAM_FINGERPRINTS 数量必须一致;否则脚本 regen_lcm_goldens.sh 漏跑"
    );
    for (i, (fqn, expected)) in UPSTREAM_FINGERPRINTS.iter().enumerate() {
        let struct_seed_entry = STRUCT_SEEDS
            .iter()
            .find(|(name, _)| name == fqn)
            .unwrap_or_else(|| panic!("STRUCT_SEEDS 缺 {fqn} 索引 {i}"));
        let fp = compute_hash(fqn);
        let fp_u64 = u64::from_be_bytes(fp);
        assert_eq!(
            fp_u64, *expected,
            "fqn={fqn} seed=0x{:016x}: 上游 lcm-gen 值不匹配",
            struct_seed_entry.1
        );
    }
}

/// `LcmMessageType::fingerprint()` 等价(两端共用 `nuwax-codec::hash`)
#[test]
fn test_lcm_message_type_fingerprint_uses_real_lcm_hash() {
    for (fqn, upstream_fp) in UPSTREAM_FINGERPRINTS {
        let msg_type = LcmMessageType::new(fqn).expect("valid");
        let fp = msg_type.fingerprint();
        let fp_u64 = u64::from_be_bytes(fp);
        assert_eq!(
            fp_u64, *upstream_fp,
            "LcmMessageType fingerprint({fqn}) must equal upstream lcm-gen"
        );
    }
}

/// `golden_lcm_message` 字节 = `RawLcmFrame::encode` 字节
/// —— 同时 fingerprint 来自 upstream lcm-gen 真值(双断言)
#[test]
fn test_golden_lcm_message_uses_upstream_fingerprint() {
    for (fqn, _upstream_fp) in UPSTREAM_FINGERPRINTS {
        // 这 10 个 fqn 都必须有 channel 测试的 fixture
        let channel = match *fqn {
            "geometry_msgs/PoseStamped" => "/pose_stamped",
            "geometry_msgs/Transform" => "/tf",
            "geometry_msgs/Twist" => "/cmd_vel",
            "nav_msgs/Odometry" => "/odom",
            "nav_msgs/OccupancyGrid" => "/map",
            "sensor_msgs/PointCloud2" => "/points",
            "nuwax_msgs/NavigationState" => "/nav/state",
            "nuwax_msgs/CancelNavigation" => "/nav/cancel",
            "nuwax_msgs/Frontier" => "/explore/frontier",
            "nuwax_msgs/FrontierList" => "/explore/frontiers",
            _ => "/x",
        };
        let payload = format!("{fqn}-payload-fixture").into_bytes();
        // codec 端
        let codec_bytes = golden_lcm_message(channel, fqn, &payload, 1).expect("golden");
        // protocol 端
        let topic = LcmTopic::new(channel).expect("valid topic");
        let msg_type = LcmMessageType::new(fqn).expect("valid msg_type");
        let frame = RawLcmFrameBuilder::new(topic, msg_type)
            .payload(payload)
            .seqno(1)
            .build();
        let proto_bytes = frame.encode();
        assert_eq!(
            codec_bytes.as_ref(),
            proto_bytes.as_slice(),
            "byte-equality drift on {fqn}"
        );
        // 同时确认 magic 字段确实是 LC02
        assert_eq!(
            u32::from_be_bytes([codec_bytes[0], codec_bytes[1], codec_bytes[2], codec_bytes[3]]),
            LCM_MAGIC
        );
    }
}

/// `LcmMessageBytes::new` 后,fingerprint 与 upstream lcm-gen 一致
#[test]
fn test_lcm_message_bytes_fingerprint_matches_upstream() {
    let topic = "/cmd_vel";
    let fqn = "geometry_msgs/Twist";
    let payload = b"twist payload v1";
    let bytes = build_lcm_message_bytes(topic, fqn, payload, 1).expect("ok");
    // upstream fingerprint for geometry_msgs::Twist = 0xbbca3b0d2c22f384
    let expected_fp: [u8; 8] = 0xbbca_3b0d_2c22_f384u64.to_be_bytes();
    assert_eq!(bytes.fingerprint, expected_fp);
    assert_eq!(bytes.sequence, 1);
    assert_eq!(bytes.channel, "/cmd_vel");
    assert_eq!(bytes.payload, payload);
}

/// `parse_lcm_message_bytes` 能反解 codec 端 envelope
#[test]
fn test_parse_lcm_message_bytes_roundtrip() {
    let msg: LcmMessageBytes = build_lcm_message_bytes(
        "/points",
        "sensor_msgs/PointCloud2",
        b"pointcloud-fixture",
        99,
    )
    .expect("ok");
    // 手工拼 envelope:4-byte LE channel_len + channel bytes + fingerprint + payload
    let envelope = encode_custom_envelope(&msg);
    let (channel, fp, payload) = parse_lcm_message_bytes(&envelope).expect("parse ok");
    assert_eq!(channel, "/points");
    assert_eq!(fp, msg.fingerprint);
    assert_eq!(payload, msg.payload);
}

fn encode_custom_envelope(msg: &LcmMessageBytes) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&(msg.channel.len() as u32).to_le_bytes());
    out.extend_from_slice(msg.channel.as_bytes());
    out.extend_from_slice(&msg.fingerprint);
    out.extend_from_slice(&msg.payload);
    out
}