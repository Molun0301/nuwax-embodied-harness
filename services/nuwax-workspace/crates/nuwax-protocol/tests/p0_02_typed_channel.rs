//! P0-02 回归测试:typed channel 合同。
//!
//! ## 背景 (2026-07-11 rework22)
//!
//! Python `lcmpubsub.py:36-72` 在 publish/subscribe 时使用
//! `/topic#pkg.Msg` 形式作为 typed channel;`(#.*)?` 后缀可省略(只取
//! channel 部分)。
//!
//! Rust 侧 `LcmTopic` 仅保存 channel 字符串;typed suffix 由
//! `as_subscription_pattern(msg_type)` 拼装。rework22 P0-02 目标:
//!
//! 1. `LcmTopic::fqn(msg_type)` / `as_subscription_pattern(msg_type)`
//!    输出 `/topic#msg_type` 形式;
//! 2. `LcmTopic::matches(actual)` 支持 typed-suffix 可选 ——
//!    `/topic#Msg` 与 `/topic` 都算匹配;
//! 3. 与 `parse_subscription_pattern` roundtrip。
//!
//! ## 不在范围内
//!
//! - real_backend 的 typed-suffix 过滤(`crates/nuwax-transport`)走
//!   `LcmMessageType::fqn()` 等价校验,与 `LcmTopic::matches()` 解耦,
//!   由 `nuwax-transport` 自家测试覆盖。
//! - `expected_lcm_bytes` / Pose/Odom/PointCloud2 wire-format 字节
//!   一致性由 P0-03 测试承担(`crates/nuwax-codec/tests/p0_03_codec.rs`)。

use nuwax_protocol::LcmTopic;

/// `LcmTopic::fqn(msg_type)` 输出 `/<channel>#<msg_type>` 形式,
/// 对齐 Python lcmpubsub 的 publish channel 形式。
#[test]
fn p0_02_topic_fqn_format() {
    let t = LcmTopic::new("/cmd_vel").expect("valid topic");
    assert_eq!(t.fqn("Twist"), "/cmd_vel#Twist");
    assert_eq!(t.fqn("geometry_msgs/Twist"), "/cmd_vel#geometry_msgs/Twist");
}

/// `as_subscription_pattern` 是 `fqn` 的同义 API,保持兼容旧调用方。
#[test]
fn p0_02_topic_as_subscription_pattern_alias() {
    let t = LcmTopic::new("/odom").expect("valid topic");
    assert_eq!(
        t.as_subscription_pattern("nav_msgs/Odometry"),
        "/odom#nav_msgs/Odometry"
    );
    // 同一字符串应与 fqn 等价
    assert_eq!(
        t.as_subscription_pattern("nav_msgs/Odometry"),
        t.fqn("nav_msgs/Odometry")
    );
}

/// 完全匹配(typed suffix 形式):`actual == "/channel#msg_type"`。
#[test]
fn p0_02_topic_matches_typed_exact() {
    let t = LcmTopic::new("/cmd_vel").expect("valid topic");
    assert!(t.matches("/cmd_vel#Twist"));
    assert!(t.matches("/cmd_vel#geometry_msgs/Twist"));
}

/// pattern 匹配(`(#.*)?` 后缀省略):仅 channel 部分相等。
#[test]
fn p0_02_topic_matches_pattern_suffix_optional() {
    let t = LcmTopic::new("/cmd_vel").expect("valid topic");
    assert!(t.matches("/cmd_vel"), "pattern suffix 可省略");
    // typed suffix 存在时也应匹配(只看 channel 部分)
    assert!(t.matches("/cmd_vel#anything"));
}

/// 不同 channel 应被拒绝。
#[test]
fn p0_02_topic_rejects_other_channel() {
    let t = LcmTopic::new("/cmd_vel").expect("valid topic");
    assert!(!t.matches("/odom#Odometry"));
    assert!(!t.matches("/odom"));
    assert!(!t.matches("cmd_vel")); // 缺前导 `/`
    assert!(!t.matches("/cmd_velocity")); // 长前缀 ≠
}

/// 与 `parse_subscription_pattern` roundtrip:输入 `/topic#Msg` 解析出来的
/// topic + msg_type 应当能再次生成等价的 fqn。
#[test]
fn p0_02_parse_then_fqn_roundtrip() {
    let pattern = "/scan#sensor_msgs/PointCloud2";
    let (topic, msg_type) =
        LcmTopic::parse_subscription_pattern(pattern).expect("valid pattern");
    assert_eq!(topic.channel(), "/scan");
    let m = msg_type.as_deref().expect("has suffix");
    assert_eq!(topic.fqn(m), pattern);
}

/// 不带 typed suffix 的 pattern:roundtrip 后应得 None suffix + 同 channel。
#[test]
fn p0_02_parse_no_suffix_roundtrip() {
    let (topic, msg_type) = LcmTopic::parse_subscription_pattern("/scan").expect("valid");
    assert_eq!(topic.channel(), "/scan");
    assert!(msg_type.is_none());
    // matches 仍命中
    assert!(topic.matches("/scan"));
    assert!(topic.matches("/scan#anything"));
}