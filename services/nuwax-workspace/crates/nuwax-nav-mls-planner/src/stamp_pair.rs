//! Stamp pairing(latest-only) + NaN cancel gate
//!
//! 给 `mls_planner_native` binary 提供"最新帧"语义：
//! - 仅当 `odom` 与 `map` 两个 LCM frame 都到达(同时 stamp >= last_seen)才放行 plan
//! - 输入含 NaN 即拒(不静默错乱)
//!
//! **当前 dev 简化版本**:仅做 strict equality 检查;实机走 LCM channel 时由
//! `nuwax-lcm` 的 stamp sequence 提供 monotonic counter。

use thiserror::Error;

/// Stamp pairing 错误
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum NaNCancelError {
    /// stamp mismatch (odom / map 未同时到最新)
    #[error("stamp mismatch: odom or map not at latest frame")]
    StampMismatch,
    /// 输入含 NaN
    #[error("input contains NaN")]
    NaN,
}

/// Latest-only stamp pairing gate
///
/// dev 简化版：仅当 `odom_stamp == map_stamp` 时返回 `Some(())`。
/// 真实实装应当检查 monotonic counter + 各自上次 seen stamp；
/// 当前 `mls_planner_native` 接受固定 `(1, 1)`，dev 自测覆盖。
#[must_use]
pub fn stamp_pair_latest_only(odom_stamp: u64, map_stamp: u64) -> Option<()> {
    if odom_stamp == 0 || map_stamp == 0 {
        return None;
    }
    if odom_stamp != map_stamp {
        return None;
    }
    Some(())
}

/// NaN cancel gate
///
/// 任意一个值为 NaN / Infinity 则返 `Err(NaN)`；有限值返 `Ok(())`。
pub fn nan_cancel_goal(goal: (f32, f32, f32)) -> Result<(), NaNCancelError> {
    if goal.0.is_nan() || goal.1.is_nan() || goal.2.is_nan()
        || !goal.0.is_finite() || !goal.1.is_finite() || !goal.2.is_finite()
    {
        return Err(NaNCancelError::NaN);
    }
    Ok(())
}

/// 联合 gate：stamp + NaN
pub fn plan_input_gate(
    odom_stamp: u64,
    map_stamp: u64,
    goal: (f32, f32, f32),
) -> Result<(), NaNCancelError> {
    stamp_pair_latest_only(odom_stamp, map_stamp).ok_or(NaNCancelError::StampMismatch)?;
    nan_cancel_goal(goal)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamp_pair_equal_passes() {
        assert_eq!(stamp_pair_latest_only(1, 1), Some(()));
    }

    #[test]
    fn stamp_pair_zero_rejects() {
        assert_eq!(stamp_pair_latest_only(0, 1), None);
        assert_eq!(stamp_pair_latest_only(1, 0), None);
        assert_eq!(stamp_pair_latest_only(0, 0), None);
    }

    #[test]
    fn stamp_pair_mismatch_rejects() {
        assert_eq!(stamp_pair_latest_only(1, 2), None);
        assert_eq!(stamp_pair_latest_only(2, 1), None);
    }

    #[test]
    fn nan_cancel_detects_nan() {
        assert_eq!(nan_cancel_goal((1.0, 2.0, 3.0)), Ok(()));
        assert_eq!(
            nan_cancel_goal((f32::NAN, 0.0, 0.0)),
            Err(NaNCancelError::NaN)
        );
    }

    #[test]
    fn nan_cancel_detects_infinity() {
        assert_eq!(
            nan_cancel_goal((f32::INFINITY, 0.0, 0.0)),
            Err(NaNCancelError::NaN)
        );
    }

    #[test]
    fn plan_input_gate_combined() {
        assert_eq!(plan_input_gate(1, 1, (1.0, 2.0, 3.0)), Ok(()));
        assert_eq!(
            plan_input_gate(1, 2, (1.0, 2.0, 3.0)),
            Err(NaNCancelError::StampMismatch)
        );
        assert_eq!(
            plan_input_gate(1, 1, (f32::NAN, 2.0, 3.0)),
            Err(NaNCancelError::NaN)
        );
    }
}