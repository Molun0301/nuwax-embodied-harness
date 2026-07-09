//! Replan limiter — 与 Python `ReplanLimiter` 兼容
//!
//! 对应 Python 源 `dimos/navigation/replanning_a_star/replan_limiter.py`。
//!
//! 严格语义：
//! - `_max_attempts = 6`
//! - `_reset_distance = 2.0 m`
//! - `can_retry(position)` 首次记录 `attempt_pos`；
//!   已有 attempt 且当前位置距 `attempt_pos >= reset_distance` 时清零 attempt 并刷新 `attempt_pos`。
//! - `can_retry` 返回 `attempt + 1 <= max_attempts`。
//! - `will_retry()` 把 `attempt` 加一。
//! - `reset()` 只清 attempt，保留 `attempt_pos`（与 Python `replan_limiter.py:62-64` 一致；
//!   修复 rework20 H-19 — 历史 Rust 实现误多清 `attempt_pos`，导致跨 goal 状态被错误隔断）。
//!
//! 重要：**本类型不是 D* Lite**。D* Lite 是后续算法增强，不属本轮验收。

use crate::helpers::Point2;

/// 与 Python `ReplanLimiter` 兼容的限流器
#[derive(Debug, Clone)]
pub struct ReplanLimiter {
    max_attempts: u32,
    reset_distance_m: f64,
    attempt_pos: Option<Point2>,
    attempt: u32,
}

impl Default for ReplanLimiter {
    fn default() -> Self {
        Self::new(6, 2.0)
    }
}

impl ReplanLimiter {
    /// 构造
    ///
    /// # 参数
    /// - `max_attempts`：最大允许重试次数（默认 6）
    /// - `reset_distance_m`：移动超过该距离后清零 attempts（默认 2.0m）
    #[must_use]
    pub fn new(max_attempts: u32, reset_distance_m: f64) -> Self {
        Self {
            max_attempts,
            reset_distance_m,
            attempt_pos: None,
            attempt: 0,
        }
    }

    /// 询问当前位置是否能再 retry 一次。
    ///
    /// 副作用：
    /// - 若 `attempt == 0`，记录 `attempt_pos = position`。
    /// - 若已有 `attempt_pos`，且距离 >= reset_distance，则 attempt 清零并刷新 position。
    ///
    /// 返回 `attempt + 1 <= max_attempts`。
    #[must_use]
    pub fn can_retry(&mut self, position: Point2) -> bool {
        if self.attempt == 0 {
            self.attempt_pos = Some(position);
        }

        if self.attempt >= 1 {
            if let Some(prev) = self.attempt_pos {
                let dx = prev.x - position.x;
                let dy = prev.y - position.y;
                let dist = (dx * dx + dy * dy).sqrt();
                if dist >= self.reset_distance_m {
                    tracing::info!(
                        "Traveled enough to reset attempts (dist={:.3}m, attempts={})",
                        dist,
                        self.attempt
                    );
                    self.attempt = 0;
                    self.attempt_pos = Some(position);
                }
            }
        }

        self.attempt < self.max_attempts
    }

    /// 真正记录一次 retry（attempt 加一）
    pub fn will_retry(&mut self) {
        self.attempt += 1;
    }

    /// 清零 attempt（保留 `attempt_pos`，与 Python `replan_limiter.py:62-64` 一致；
    /// 修复 rework20 H-19 — 历史实现误多清 `attempt_pos`）
    pub fn reset(&mut self) {
        self.attempt = 0;
    }

    /// 当前 attempt 计数
    #[must_use]
    pub fn attempt(&self) -> u32 {
        self.attempt
    }

    /// 当前 attempt position
    #[must_use]
    pub fn attempt_pos(&self) -> Option<Point2> {
        self.attempt_pos
    }

    /// 最大 attempt
    #[must_use]
    pub fn max_attempts(&self) -> u32 {
        self.max_attempts
    }

    /// reset distance（米）
    #[must_use]
    pub fn reset_distance(&self) -> f64 {
        self.reset_distance_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64) -> Point2 {
        Point2::new(x, y)
    }

    #[test]
    fn first_can_retry_records_position() {
        let mut l = ReplanLimiter::default();
        let ok = l.can_retry(p(0.0, 0.0));
        assert!(ok);
        assert_eq!(l.attempt(), 0);
        assert_eq!(l.attempt_pos(), Some(p(0.0, 0.0)));
    }

    #[test]
    fn six_attempts_allowed_then_blocked() {
        let mut l = ReplanLimiter::default();
        for i in 0..6 {
            assert!(l.can_retry(p(0.0, 0.0)), "attempt {i} should be allowed");
            l.will_retry();
        }
        // 现在 attempt = 6, 再次询问应返回 false
        assert!(!l.can_retry(p(0.0, 0.0)));
    }

    #[test]
    fn reset_distance_clears_attempts() {
        let mut l = ReplanLimiter::default();
        // 3 次 retry
        for _ in 0..3 {
            assert!(l.can_retry(p(0.0, 0.0)));
            l.will_retry();
        }
        assert_eq!(l.attempt(), 3);

        // 移动 2.5m 后，attempt 应清零
        let ok = l.can_retry(p(2.5, 0.0));
        assert!(ok);
        assert_eq!(l.attempt(), 0, "distance reset should clear attempts");
        assert_eq!(l.attempt_pos(), Some(p(2.5, 0.0)));
    }

    #[test]
    fn reset_distance_below_threshold_does_not_clear() {
        let mut l = ReplanLimiter::default();
        for _ in 0..2 {
            let _ = l.can_retry(p(0.0, 0.0));
            l.will_retry();
        }
        // 移动 1.5m < 2.0m, 不清
        let _ = l.can_retry(p(1.5, 0.0));
        assert_eq!(l.attempt(), 2);
    }

    #[test]
    fn reset_clears_attempt_keeps_position() {
        // 修复 rework20 H-19:reset() 只清 attempt，保留 attempt_pos（对齐 Python 语义）
        let mut l = ReplanLimiter::default();
        let _ = l.can_retry(p(0.0, 0.0));
        l.will_retry();
        l.will_retry();
        l.reset();
        assert_eq!(l.attempt(), 0);
        assert_eq!(l.attempt_pos(), Some(p(0.0, 0.0)), "reset must keep attempt_pos (Python 语义对齐)");
    }

    #[test]
    fn can_retry_returns_attempt_plus_one_le_max() {
        let mut l = ReplanLimiter::new(3, 2.0);
        // attempt 0 → can_retry: 0+1 <= 3 → true
        assert!(l.can_retry(p(0.0, 0.0)));
        l.will_retry(); // attempt = 1
        assert!(l.can_retry(p(0.0, 0.0))); // 1+1 <= 3
        l.will_retry(); // attempt = 2
        assert!(l.can_retry(p(0.0, 0.0))); // 2+1 <= 3
        l.will_retry(); // attempt = 3
        assert!(!l.can_retry(p(0.0, 0.0))); // 3+1 > 3
    }

    #[test]
    fn config_methods() {
        let l = ReplanLimiter::new(10, 5.0);
        assert_eq!(l.max_attempts(), 10);
        assert!((l.reset_distance() - 5.0).abs() < 1e-9);
    }
}
