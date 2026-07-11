//! P1 #1 回归测试：`OccupancyCell` 保留原始 int8 + 可选概率

use nuwax_mapping::occupancy::types::{OccupancyCell, OccupancyValue};

#[test]
fn from_i8_preserves_intermediate_probability() {
    // 1..99 中间概率不应被吞掉
    let cell = OccupancyCell::from_i8(50);
    assert_eq!(cell.value, 50);
    assert_eq!(cell.prob, None);
    assert_eq!(cell.to_value(), OccupancyValue::Unknown); // 三态 enum 中无对应
    assert!(cell.has_probability()); // 但 cell 知道这是概率值
}

#[test]
fn from_i8_with_prob_clamps_to_unit_interval() {
    let cell = OccupancyCell::from_i8_with_prob(50, 1.5);
    assert_eq!(cell.prob, Some(1.0), "1.5 should clamp to 1.0");
    let cell2 = OccupancyCell::from_i8_with_prob(50, -0.5);
    assert_eq!(cell2.prob, Some(0.0), "-0.5 should clamp to 0.0");
}

#[test]
fn to_value_three_state_semantics() {
    let occupied = OccupancyCell::from_i8(100);
    assert_eq!(occupied.to_value(), OccupancyValue::Occupied);

    let free = OccupancyCell::from_i8(0);
    assert_eq!(free.to_value(), OccupancyValue::Free);

    let unknown = OccupancyCell::from_i8(-1);
    assert_eq!(unknown.to_value(), OccupancyValue::Unknown);

    // 中间概率 → Unknown
    let prob = OccupancyCell::from_i8(50);
    assert_eq!(prob.to_value(), OccupancyValue::Unknown);
}

#[test]
fn from_value_roundtrip() {
    for v in [
        OccupancyValue::Unknown,
        OccupancyValue::Free,
        OccupancyValue::Occupied,
    ] {
        let cell = OccupancyCell::from_value(v);
        assert_eq!(cell.to_value(), v);
        assert_eq!(cell.prob, None);
    }
}

#[test]
fn probability_from_value_inference() {
    // 无显式 prob 时，按 value 推断
    let cell_free = OccupancyCell::from_i8(0);
    assert_eq!(cell_free.probability(), 0.0);

    let cell_occ = OccupancyCell::from_i8(100);
    assert_eq!(cell_occ.probability(), 1.0);

    let cell_50 = OccupancyCell::from_i8(50);
    assert!((cell_50.probability() - 0.5).abs() < 1e-6);

    // 显式 prob 覆盖 value 推断
    let cell_explicit = OccupancyCell::from_i8_with_prob(100, 0.7);
    assert_eq!(cell_explicit.probability(), 0.7);
}

#[test]
fn is_lethal_and_has_probability() {
    assert!(OccupancyCell::from_i8(100).is_lethal());
    assert!(OccupancyCell::from_i8(127).is_lethal()); // ≥ 100 都算 lethal
    assert!(!OccupancyCell::from_i8(99).is_lethal());
    assert!(!OccupancyCell::from_i8(0).is_lethal());

    assert!(OccupancyCell::from_i8(50).has_probability()); // 1..99
    assert!(OccupancyCell::from_i8(99).has_probability()); // 1..99 含 99
    assert!(OccupancyCell::from_i8_with_prob(100, 1.0).has_probability()); // 显式 prob 即使 100 也算
    assert!(OccupancyCell::from_i8_with_prob(0, 0.0).has_probability()); // 显式 prob
    assert!(!OccupancyCell::from_i8(0).has_probability()); // 无 prob 无 1..99
    assert!(!OccupancyCell::from_i8(100).has_probability()); // 100 不是 1..99 但 is_lethal
    assert!(!OccupancyCell::from_i8(-1).has_probability()); // Unknown
}

#[test]
fn occupancy_value_backward_compat() {
    // 旧 OccupancyValue API 不变
    assert_eq!(OccupancyValue::from_i8(0), OccupancyValue::Free);
    assert_eq!(OccupancyValue::from_i8(100), OccupancyValue::Occupied);
    assert_eq!(OccupancyValue::from_i8(-1), OccupancyValue::Unknown);
    assert_eq!(OccupancyValue::from_i8(50), OccupancyValue::Unknown); // 中间值
}