//! `waypoint_schema` 集成测试:验证 `TareWaypoint` typed schema 合同。
//!
//! 完成边界(方案 §5.4):serde roundtrip、clone/debug、`validate()` 成功/失败、
//! 手工构造 waypoint 的字段语义。
//!
//! **不**订阅、**不**解码 native `way_point` payload;`TareWaypoint -> FrontierGoal`
//! 转换唯一归属 `nuwax-exploration::TareWaypointGoalSourceAdapter`,本 crate **不得**
//! 实现该转换,**不得**新增 native payload decoder。

use nuwax_explore_tare_bridge::waypoint::{TareWaypoint, TareWaypointError};
use nuwax_nav_types_messages::frames::NavFrameId;
use nuwax_nav_types_messages::geometry::Timestamp;

fn sample() -> TareWaypoint {
    TareWaypoint::new(
        NavFrameId::Map,
        Timestamp::from_unix(1_700_000_000, 42),
        1.25,
        -3.5,
        0.0,
    )
}

#[test]
fn manual_construction_preserves_all_fields() {
    let wp = sample();
    assert_eq!(wp.frame_id, NavFrameId::Map);
    assert_eq!(
        wp.stamp.nanos,
        Timestamp::from_unix(1_700_000_000, 42).nanos
    );
    assert_eq!(wp.x, 1.25);
    assert_eq!(wp.y, -3.5);
    assert_eq!(wp.z, 0.0);
}

#[test]
fn clone_and_debug() {
    let wp = sample();
    let cp = wp.clone();
    assert_eq!(cp, wp);
    let dbg = format!("{wp:?}");
    assert!(dbg.contains("TareWaypoint"));
}

#[test]
fn serde_roundtrip_json() {
    let wp = sample();
    let s = serde_json::to_string(&wp).expect("serialize");
    let back: TareWaypoint = serde_json::from_str(&s).expect("deserialize");
    assert_eq!(back, wp);
}

#[test]
fn serde_roundtrip_custom_frame() {
    let wp = TareWaypoint::new(
        NavFrameId::Custom("tare_local".into()),
        Timestamp::from_unix(0, 0),
        0.0,
        0.0,
        0.0,
    );
    let s = serde_json::to_string(&wp).expect("serialize");
    assert!(s.contains("tare_local"), "frame payload missing: {s}");
    let back: TareWaypoint = serde_json::from_str(&s).expect("deserialize");
    assert_eq!(back.frame_id, NavFrameId::Custom("tare_local".into()));
}

#[test]
fn validate_accepts_normal_waypoint() {
    assert!(sample().validate().is_ok());
}

#[test]
fn validate_rejects_nan_x() {
    let mut wp = sample();
    wp.x = f64::NAN;
    let err = wp.validate().unwrap_err();
    assert!(matches!(
        err,
        TareWaypointError::NonFiniteCoordinate { field: "x", .. }
    ));
}

#[test]
fn validate_rejects_inf_y() {
    let mut wp = sample();
    wp.y = f64::INFINITY;
    let err = wp.validate().unwrap_err();
    assert!(matches!(
        err,
        TareWaypointError::NonFiniteCoordinate { field: "y", .. }
    ));
}

#[test]
fn validate_rejects_neg_inf_z() {
    let mut wp = sample();
    wp.z = f64::NEG_INFINITY;
    let err = wp.validate().unwrap_err();
    assert!(matches!(
        err,
        TareWaypointError::NonFiniteCoordinate { field: "z", .. }
    ));
}

#[test]
fn validate_rejects_out_of_range() {
    let mut wp = sample();
    wp.x = 1.0e9;
    let err = wp.validate().unwrap_err();
    assert!(matches!(
        err,
        TareWaypointError::CoordinateOutOfRange { field: "x", .. }
    ));
}

#[test]
fn frame_id_as_str_for_mapping_documentation() {
    // 文档化 mapper 输入:TareWaypoint -> FrontierGoal id field 用 `frame_id.as_str()`。
    let wp = sample();
    assert_eq!(wp.frame_id.as_str(), "map");
    let custom = TareWaypoint::new(
        NavFrameId::Custom("tare_world".into()),
        Timestamp::from_unix(0, 0),
        0.0,
        0.0,
        0.0,
    );
    assert_eq!(custom.frame_id.as_str(), "tare_world");
}
