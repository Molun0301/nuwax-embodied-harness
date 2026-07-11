//! P0-16 回归测试：stamp_pair + NaN cancel gate
//!
//! 直接调用 `stamp_pair` 模块公共 API 验证 gate 行为。
//! 这些函数在 `mls_planner_native` 内部作为 `plan_gate` 使用,
//! 这里作为独立测试覆盖 gate 边界。

use nuwax_nav_mls_planner::stamp_pair::{
    NaNCancelError, nan_cancel_goal, plan_input_gate, stamp_pair_latest_only,
};

#[test]
fn stamp_pair_drops_zero() {
    assert_eq!(stamp_pair_latest_only(0, 1), None);
    assert_eq!(stamp_pair_latest_only(1, 0), None);
}

#[test]
fn stamp_pair_drops_mismatch() {
    assert_eq!(stamp_pair_latest_only(1, 2), None);
    assert_eq!(stamp_pair_latest_only(7, 3), None);
}

#[test]
fn stamp_pair_accepts_equal() {
    assert_eq!(stamp_pair_latest_only(42, 42), Some(()));
}

#[test]
fn nan_cancel_blocks_nan() {
    let err = nan_cancel_goal((f32::NAN, 0.0, 0.0)).unwrap_err();
    assert_eq!(err, NaNCancelError::NaN);
}

#[test]
fn nan_cancel_blocks_inf() {
    let err = nan_cancel_goal((f32::INFINITY, 0.0, 0.0)).unwrap_err();
    assert_eq!(err, NaNCancelError::NaN);
}

#[test]
fn nan_cancel_passes_finite() {
    assert!(nan_cancel_goal((1.0, 2.0, 3.0)).is_ok());
}

#[test]
fn gate_combined_stamp_then_nan() {
    // stamp ok, nan -> NaN
    assert_eq!(
        plan_input_gate(1, 1, (f32::NAN, 0.0, 0.0)),
        Err(NaNCancelError::NaN)
    );
    // stamp mismatch -> StampMismatch
    assert_eq!(
        plan_input_gate(1, 2, (1.0, 0.0, 0.0)),
        Err(NaNCancelError::StampMismatch)
    );
    // 全部 ok
    assert_eq!(plan_input_gate(5, 5, (1.0, 2.0, 3.0)), Ok(()));
}