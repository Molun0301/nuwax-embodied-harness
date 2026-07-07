//! Stuck 升级逻辑：进度未达标时按系数缩小 inflation_radius。

/// Stuck 检测状态。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StuckState {
    /// 上次记录的 goal_dist
    pub ref_goal_dist: f64,
    /// 上次进度时间戳（monotonic 秒）
    pub last_progress_time: f64,
    /// 当前 effective inflation
    pub effective_inflation: f64,
}

/// 单次 stuck 检测 tick。
///
/// 返回 `(new_state, escalated)`：
/// - `escalated = true` 表示 `effective_inflation` 被缩小；
/// - `escalated = false` 表示保持不变（或仅刷新 ref_goal_dist）。
#[must_use]
pub fn progress_tick(
    state: StuckState,
    goal_dist: f64,
    mono_now: f64,
    progress_epsilon: f64,
    stuck_seconds: f64,
    stuck_shrink_factor: f64,
    stuck_min_inflation: f64,
) -> (StuckState, bool) {
    // 1. 进度达标
    if goal_dist < state.ref_goal_dist - progress_epsilon {
        return (
            StuckState {
                ref_goal_dist: goal_dist,
                last_progress_time: mono_now,
                effective_inflation: state.effective_inflation,
            },
            false,
        );
    }
    // 2. 升级判定
    if mono_now - state.last_progress_time >= stuck_seconds
        && state.effective_inflation > stuck_min_inflation
    {
        let prev = state.effective_inflation;
        let candidate = prev * stuck_shrink_factor;
        let new_inflation = candidate.max(stuck_min_inflation);
        if new_inflation < prev {
            return (
                StuckState {
                    ref_goal_dist: goal_dist,
                    last_progress_time: mono_now,
                    effective_inflation: new_inflation,
                },
                true,
            );
        }
    }
    (state, false)
}
