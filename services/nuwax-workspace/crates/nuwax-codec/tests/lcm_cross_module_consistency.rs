//! 跨模块 byte-level 一致性测试
//!
//! ## P0-B 验收范围(诚实口径)
//!
//! - **当前实现**:`nuwax-codec::golden_lcm_message` 与
//!   `nuwax-protocol::RawLcmFrame::encode` 输出 byte-for-byte 一致。
//!   这是 P0-B "LCM adapter foundation + Rust 内部 stable contract" 的
//!   最低可验收条件。
//! - **不在 P0 范围**:与 system LCM(Python / C++)的真实 wire-format 对账
//!   — 依赖 `lcm_msgs.tar.gz` 已 LFS-fetch + system `lcm` / `lcm-gen`
//!   可用。P1 阶段新增 `[ignore = "requires system LCM"]` 的端到端 test,
//!   由 `nuwax_codec::lcm::expected_lcm_bytes` 提供 reference bytes。
//!
//! 该测试作为 integration test 存在,以强制 codec + protocol 同时被构建,
//! 而不是仅靠各自 crate 的 unit test。

use nuwax_codec::lcm::{
    compute_hash, golden_lcm_message, golden_odometry, golden_point_cloud2, golden_pose_stamped,
};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};

/// 构造 payload 与 `golden_pose_stamped` 内部保持一致
///
/// payload 总 68 字节:12 bytes marker + 7 个 f64 (= 56 字节)。
///
/// orientation 顺序 w 在前 (qw, qx, qy, qz),与 lcm-gen 生成的
/// `geometry_msgs__PoseStamped.c` encode_array 一致。
fn pose_payload() -> Vec<u8> {
    let mut p = Vec::with_capacity(68);
    p.extend_from_slice(b"PoseStamped!");
    p.extend_from_slice(&1.0f64.to_le_bytes()); // position_x
    p.extend_from_slice(&2.0f64.to_le_bytes()); // position_y
    p.extend_from_slice(&0.0f64.to_le_bytes()); // position_z
    p.extend_from_slice(&1.0f64.to_le_bytes()); // orientation_w (w 在前)
    p.extend_from_slice(&0.0f64.to_le_bytes()); // orientation_x
    p.extend_from_slice(&0.0f64.to_le_bytes()); // orientation_y
    p.extend_from_slice(&0.0f64.to_le_bytes()); // orientation_z
    p
}

#[test]
fn pose_stamped_bytes_match_protocol_encode() {
    let topic_str = "/pose_stamped";
    let msg_type_str = "geometry_msgs/PoseStamped";
    let payload = pose_payload();

    // codec 端 golden 字节
    let codec_bytes = golden_lcm_message(topic_str, msg_type_str, &payload, 1).expect("golden ok");

    // protocol 端 RawLcmFrame::encode
    let topic = LcmTopic::new(topic_str).unwrap();
    let msg_type = LcmMessageType::new(msg_type_str).unwrap();
    let frame = RawLcmFrameBuilder::new(topic, msg_type)
        .payload(payload)
        .seqno(1)
        .build();
    let protocol_bytes = frame.encode();

    assert_eq!(codec_bytes.as_ref(), protocol_bytes.as_slice());
}

#[test]
fn odometry_bytes_match_protocol_encode() {
    let topic_str = "/odom";
    let msg_type_str = "nav_msgs/Odometry";
    let payload = b"OdometryFixtureMarker-v1".to_vec();

    let codec_bytes = golden_lcm_message(topic_str, msg_type_str, &payload, 1).unwrap();

    let topic = LcmTopic::new(topic_str).unwrap();
    let msg_type = LcmMessageType::new(msg_type_str).unwrap();
    let frame = RawLcmFrameBuilder::new(topic, msg_type)
        .payload(payload.clone())
        .seqno(1)
        .build();

    assert_eq!(codec_bytes.as_ref(), frame.encode().as_slice());
}

#[test]
fn point_cloud2_bytes_match_protocol_encode() {
    let topic_str = "/points";
    let msg_type_str = "sensor_msgs/PointCloud2";
    let payload = b"PointCloud2FixtureMarker-v1".to_vec();

    let codec_bytes = golden_lcm_message(topic_str, msg_type_str, &payload, 1).unwrap();

    let topic = LcmTopic::new(topic_str).unwrap();
    let msg_type = LcmMessageType::new(msg_type_str).unwrap();
    let frame = RawLcmFrameBuilder::new(topic, msg_type)
        .payload(payload.clone())
        .seqno(1)
        .build();

    assert_eq!(codec_bytes.as_ref(), frame.encode().as_slice());
}

#[test]
fn codec_compute_hash_matches_protocol_fingerprint() {
    // 验证 codec 端的 lcm_hash 与 protocol 端 `LcmMessageType::fingerprint()`
    // 完全一致 —— 这是字节一致性的前提。
    for fqn in [
        "geometry_msgs/PoseStamped",
        "nav_msgs/Odometry",
        "sensor_msgs/PointCloud2",
    ] {
        let a = compute_hash(fqn);
        let b = LcmMessageType::new(fqn).unwrap().fingerprint();
        assert_eq!(a, b, "hash mismatch for fqn={fqn}");
    }
}

#[test]
fn golden_fixtures_roundtrip_through_protocol_decode() {
    // 验证 codec golden bytes 可以被 protocol 解码为相同 payload
    let topic = LcmTopic::new("/pose_stamped").unwrap();
    let msg_type = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
    let bytes = golden_pose_stamped().unwrap();
    let frame = nuwax_protocol::RawLcmFrame::decode(&bytes, topic, msg_type).unwrap();
    assert_eq!(frame.payload, pose_payload());

    let topic = LcmTopic::new("/odom").unwrap();
    let msg_type = LcmMessageType::new("nav_msgs/Odometry").unwrap();
    let bytes = golden_odometry().unwrap();
    let frame = nuwax_protocol::RawLcmFrame::decode(&bytes, topic, msg_type).unwrap();
    assert_eq!(frame.payload, b"OdometryFixtureMarker-v1");

    let topic = LcmTopic::new("/points").unwrap();
    let msg_type = LcmMessageType::new("sensor_msgs/PointCloud2").unwrap();
    let bytes = golden_point_cloud2().unwrap();
    let frame = nuwax_protocol::RawLcmFrame::decode(&bytes, topic, msg_type).unwrap();
    assert_eq!(frame.payload, b"PointCloud2FixtureMarker-v1");
}
