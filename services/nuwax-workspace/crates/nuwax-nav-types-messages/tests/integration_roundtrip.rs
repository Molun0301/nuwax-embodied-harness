//! 跨 crate 协议面 roundtrip 测试
//!
//! 验证 Foundation 共享类型的核心协议语义：
//! - Pose2 <-> Pose3 转换（xy + yaw ↔ xyz + quaternion）
//! - BoundingBox2D xyxy 不变量（构造路径必须产出合法 xyxy）
//! - Timestamp 纳秒语义（i64 nanos + ClockKind 序列化稳定）
//! - GoalSource 枚举变体 serde 稳定（snake_case）

use nuwax_nav_types_messages::{
    geometry::{ClockKind, Point2, Point3, Pose2, Pose3, Quaternion, Timestamp},
    goal::GoalSource,
    visual::BoundingBox2D,
};

#[test]
fn pose2_pose3_consistency() {
    let p2 = Pose2::new(1.5, -2.5, 0.3);
    let p3 = Pose3::from_pose2(p2);
    let back = p3.to_pose2();

    assert!((back.position.x - p2.position.x).abs() < 1e-9);
    assert!((back.position.y - p2.position.y).abs() < 1e-9);
    assert!((back.yaw - p2.yaw).abs() < 1e-9);

    // 3D 路径走的是 quaternion.from_yaw，z 必须为 0
    assert_eq!(p3.position.z, 0.0);

    // identity quaternion 与 yaw=0 一致
    assert!(Quaternion::IDENTITY.yaw().abs() < 1e-9);
}

#[test]
fn bbox_xyxy_invariant() {
    // 合法 xyxy
    let a = BoundingBox2D::from_xyxy(10.0, 20.0, 30.0, 40.0).expect("valid xyxy");
    assert_eq!(a.x1, 10.0);
    assert_eq!(a.y1, 20.0);
    assert_eq!(a.x2, 30.0);
    assert_eq!(a.y2, 40.0);

    // from_xywh 必须保证 x1 <= x2 / y1 <= y2
    let b = BoundingBox2D::from_xywh(50.0, 60.0, 100.0, 200.0).expect("valid xywh");
    assert_eq!(b.x1, 50.0);
    assert_eq!(b.y1, 60.0);
    assert_eq!(b.x2, 150.0);
    assert_eq!(b.y2, 260.0);

    // 倒序 xyxy 必须归一化（auto-swap），不能保留非法顺序
    let swapped = BoundingBox2D::from_xyxy(100.0, 200.0, 10.0, 20.0).expect("swap ok");
    assert_eq!(swapped.x1, 10.0);
    assert_eq!(swapped.y1, 20.0);
    assert_eq!(swapped.x2, 100.0);
    assert_eq!(swapped.y2, 200.0);

    // NaN / 负宽高必须返回 None
    assert!(BoundingBox2D::from_xyxy(f32::NAN, 0.0, 1.0, 1.0).is_none());
    assert!(BoundingBox2D::from_xywh(0.0, 0.0, -1.0, 1.0).is_none());
}

#[test]
fn timestamp_nanos() {
    let t = Timestamp::from_unix(1, 500_000_000);
    assert_eq!(t.unix_seconds(), 1);
    assert_eq!(t.unix_subsec_nanos(), 500_000_000);
    assert_eq!(t.clock, ClockKind::Unix);

    let t_mono = Timestamp::from_monotonic(42);
    assert_eq!(t_mono.clock, ClockKind::Monotonic);

    // diff_nanos 必须是有符号 i64（可负）
    assert_eq!(
        Timestamp::from_unix(2, 0).diff_nanos(Timestamp::from_unix(1, 0)),
        1_000_000_000
    );
    assert_eq!(
        Timestamp::from_unix(1, 0).diff_nanos(Timestamp::from_unix(2, 0)),
        -1_000_000_000
    );

    // serde 必须保留 snake_case clock 字段
    let s = serde_json::to_string(&t_mono).unwrap();
    assert!(s.contains("\"monotonic\""));
    let back: Timestamp = serde_json::from_str(&s).unwrap();
    assert_eq!(back, t_mono);
}

#[test]
fn goal_source_roundtrip() {
    let variants = [
        GoalSource::Click,
        GoalSource::Agent,
        GoalSource::Patrol,
        GoalSource::Frontier,
        GoalSource::Visual,
        GoalSource::Replay,
        GoalSource::Manual,
        GoalSource::Other("unit-test".into()),
    ];

    for v in &variants {
        let s = serde_json::to_string(v).unwrap();
        let back: GoalSource = serde_json::from_str(&s).unwrap_or_else(|e| {
            panic!("GoalSource serde roundtrip failed for {s}: {e}");
        });
        assert_eq!(&back, v, "GoalSource roundtrip mismatch for {s}");
    }

    // 校验 serde 输出 snake_case 风格（与既有 `tag` 模式兼容）
    let click_json = serde_json::to_string(&GoalSource::Click).unwrap();
    assert!(click_json.contains("click"));
}

#[test]
fn point3_to_2d_drops_z() {
    let p3 = Point3::new(1.0, 2.0, 3.0);
    let p2 = p3.to_2d();
    assert_eq!(p2, Point2::new(1.0, 2.0));
}
