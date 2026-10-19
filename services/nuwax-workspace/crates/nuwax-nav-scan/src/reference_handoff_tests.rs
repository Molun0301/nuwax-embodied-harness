use super::*;
use crate::tests::coarse_open_floor;

// 固定旧公式，仅用于证明交接误判；生产仍使用完整标定转换。
fn nominal_handoff(p: BodyPose, reference: &[[f64; 3]], length: f64) -> bool {
    let target = reference.last().unwrap();
    let support = [p.0[0], p.0[1], p.0[2] - 0.4];
    let (deviation, progress, height) = reference_progress(support, reference);
    (p.0[0] - target[0]).hypot(p.0[1] - target[1]) <= 0.81
        && deviation <= 0.81
        && progress >= length - 1e-6
        && (support[2] - height).abs() <= 0.275
        && angle(yaw(p.1)).cos() > 0.0
}

#[test]
fn tilted_body_centre_cannot_handoff_before_actual_support_crosses_window() {
    let map = coarse_open_floor();
    let pitch = 10.0_f64.to_radians();
    let q = [0.0, (pitch * 0.5).sin(), 0.0, (pitch * 0.5).cos()];
    let reference = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
    let before = ([1.05, 0.0, 0.4 * pitch.cos()], q);
    let support = map.support_position_from_body(before.0, before.1, 0.4).unwrap();
    assert!((support[0] - 0.980540728933228).abs() < 1e-12);
    assert!(support[2].abs() < 1e-12);
    assert!(nominal_handoff(before, &reference, 1.0), "旧公式提前交接");
    assert!(!reference_handoff(&map, before, &reference, 0.4, 0.81, 0.275, 1.0, 0.0));

    let crossed = ([1.10, before.0[1], before.0[2]], q);
    assert!(reference_handoff(&map, crossed, &reference, 0.4, 0.81, 0.275, 1.0, 0.0),
        "实际支撑点越过末横截面后仍允许交接");
}

#[test]
fn tilted_support_cannot_handoff_to_lower_overlapping_reference() {
    let map = coarse_open_floor();
    let pitch = 60.0_f64.to_radians();
    let p = ([1.4, 0.0, 0.5], [0.0, (pitch * 0.5).sin(), 0.0, (pitch * 0.5).cos()]);
    let reference = [[0.0, 0.0, 0.3], [1.0, 0.0, 0.3], [1.0, 1.0, 0.3],
        [0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
    let support = map.support_position_from_body(p.0, p.1, 0.4).unwrap();
    let (_, progress, height) = reference_progress(support, &reference);
    assert!((support[2] - 0.3).abs() < 1e-12);
    assert_eq!(progress, 1.0);
    assert_eq!(height, 0.3);
    assert!(nominal_handoff(p, &reference, 5.0), "旧公式把上层支撑误配到下层末段");
    assert!(!reference_handoff(&map, p, &reference, 0.4, 0.81, 0.275, 5.0, 0.0));
}
