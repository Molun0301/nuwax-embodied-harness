//! `ExplorationSession` + `Direction2`:探索会话状态(no-gain counter / direction / explored goals),
//! **不**包含 select_goal 逻辑(归属 `WavefrontGoalSelector`)。

use nuwax_nav_types_messages::geometry::Point2;

use crate::config::WavefrontConfig;

/// 2D 单位方向(可为零;`from_delta` 拒绝零长度 / NaN)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Direction2 {
    /// 单位向量 x 分量
    pub x: f64,
    /// 单位向量 y 分量
    pub y: f64,
}

impl Direction2 {
    /// 由 delta 构造单位方向;零长度或 NaN 返回 `None`。
    #[must_use]
    pub fn from_delta(dx: f64, dy: f64) -> Option<Self> {
        if !dx.is_finite() || !dy.is_finite() {
            return None;
        }
        let len_sq = dx * dx + dy * dy;
        if len_sq <= 0.0 {
            return None;
        }
        let inv = 1.0 / len_sq.sqrt();
        Some(Self {
            x: dx * inv,
            y: dy * inv,
        })
    }
}

/// 探索会话状态:已发布 goal / 方向 / no-gain 计数 / 连续失败计数。
#[derive(Debug, Clone, Default)]
pub struct ExplorationSession {
    /// 已发布的历史 goal(用于 explored_goals_score)
    pub(crate) explored_goals: Vec<Point2>,
    /// 上一 goal 方向(用于 momentum)
    pub(crate) exploration_direction: Option<Direction2>,
    /// 上一次 information count(用于 no-gain 判定)
    pub(crate) last_information_count: Option<u64>,
    /// 连续 no-gain 计数
    pub(crate) no_gain_counter: u32,
    /// 已发布 goal 数
    pub(crate) goals_published: u32,
    /// 连续失败数
    pub(crate) consecutive_failures: u32,
}

impl ExplorationSession {
    /// 构造空 session
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// 标记已发布的 goal
    pub fn mark_explored_goal(&mut self, goal: Point2) {
        self.explored_goals.push(goal);
    }

    /// 由最新 goal 更新探索方向(若 delta 非法则保持不变)
    pub fn update_direction_from_goal(&mut self, previous: Point2, latest: Point2) {
        let dx = latest.x - previous.x;
        let dy = latest.y - previous.y;
        if let Some(d) = Direction2::from_delta(dx, dy) {
            self.exploration_direction = Some(d);
        }
    }

    /// 记录 goal 发布成功
    pub fn record_goal_published(&mut self) {
        self.goals_published = self.goals_published.saturating_add(1);
        // 成功发布后清零连续失败
        self.consecutive_failures = 0;
    }

    /// 记录失败
    pub fn record_failure(&mut self) {
        self.consecutive_failures = self.consecutive_failures.saturating_add(1);
    }

    /// 已发布 goal 数
    #[must_use]
    pub fn goals_published(&self) -> u32 {
        self.goals_published
    }

    /// 连续失败数
    #[must_use]
    pub fn consecutive_failures(&self) -> u32 {
        self.consecutive_failures
    }

    /// 探索方向(若有)
    #[must_use]
    pub fn exploration_direction(&self) -> Option<Direction2> {
        self.exploration_direction
    }

    /// 累计栅格信息量(free + occupied,occupied 按 `cfg.occupancy_threshold` 判定)
    #[must_use]
    pub fn information_count(
        &self,
        grid: &nuwax_nav_types_messages::grid::OccupancyGrid2D,
        cfg: &WavefrontConfig,
    ) -> u64 {
        use nuwax_nav_types_messages::grid::OccupancyCell;
        let mut count: u64 = 0;
        for cell in &grid.cells {
            match cell {
                OccupancyCell::Free => count += 1,
                OccupancyCell::Occupied | OccupancyCell::Inflated => count += 1,
                OccupancyCell::Cost(v) if (*v as i16) > cfg.occupancy_threshold => count += 1,
                OccupancyCell::Cost(_) | OccupancyCell::Unknown => {}
            }
        }
        count
    }

    /// 判定是否触发 no-gain stop。
    ///
    /// 顺序固定(方案 §4.10):
    /// 1. 计算 `current_info`
    /// 2. 若 `last_information_count` 为 `None`,只 seed 为 `Some(current_info)` 并返回 None
    /// 3. 否则当 `explored_goals.len() >= cfg.min_goals_before_no_gain_check` 时启用 no-gain 判定;
    ///    增长比例低于 `cfg.info_gain_threshold_ratio` 时递增 counter;
    ///    `no_gain_counter >= cfg.num_no_gain_attempts` 返回 `Some(StopReason::NoInformationGain)` 并清零
    /// 4. 函数结束前更新 `last_information_count = Some(current_info)`
    #[must_use]
    pub fn check_information_gain(
        &mut self,
        grid: &nuwax_nav_types_messages::grid::OccupancyGrid2D,
        cfg: &WavefrontConfig,
    ) -> Option<crate::frontier_goal::StopReason> {
        let current_info = self.information_count(grid, cfg);
        let last = self.last_information_count;
        if last.is_none() {
            self.last_information_count = Some(current_info);
            return None;
        }
        let last = last.expect("checked is_some above");
        let stop_reason = if self.explored_goals.len() as u32 >= cfg.min_goals_before_no_gain_check
        {
            let gain_ratio = if last == 0 {
                0.0
            } else {
                (current_info as f64 - last as f64) / (last as f64)
            };
            if gain_ratio < cfg.info_gain_threshold_ratio {
                self.no_gain_counter = self.no_gain_counter.saturating_add(1);
                if self.no_gain_counter >= cfg.num_no_gain_attempts {
                    self.no_gain_counter = 0;
                    Some(crate::frontier_goal::StopReason::NoInformationGain)
                } else {
                    None
                }
            } else {
                self.no_gain_counter = 0;
                None
            }
        } else {
            None
        };
        self.last_information_count = Some(current_info);
        stop_reason
    }

    /// 重置全部状态(explored goals、direction、no-gain、failure)
    pub fn reset(&mut self) {
        self.explored_goals.clear();
        self.exploration_direction = None;
        self.last_information_count = None;
        self.no_gain_counter = 0;
        self.goals_published = 0;
        self.consecutive_failures = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::WavefrontConfig;
    use nuwax_nav_types_messages::geometry::Point2;
    use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};

    fn empty_grid(w: u32, h: u32) -> OccupancyGrid2D {
        OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.1,
            width: w,
            height: h,
            cells: vec![OccupancyCell::Unknown; (w * h) as usize],
        }
    }

    #[test]
    fn direction_from_delta_normalizes() {
        let d = Direction2::from_delta(3.0, 4.0).unwrap();
        assert!((d.x - 0.6).abs() < 1e-9);
        assert!((d.y - 0.8).abs() < 1e-9);
    }

    #[test]
    fn direction_from_zero_returns_none() {
        assert!(Direction2::from_delta(0.0, 0.0).is_none());
        assert!(Direction2::from_delta(f64::NAN, 1.0).is_none());
    }

    #[test]
    fn first_call_seeds_last_info() {
        let mut s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let g = empty_grid(4, 4);
        assert!(s.check_information_gain(&g, &cfg).is_none());
        assert_eq!(s.last_information_count, Some(0));
    }

    #[test]
    fn below_min_goals_does_not_trigger_no_gain() {
        let mut s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let g = empty_grid(4, 4);
        // seed
        let _ = s.check_information_gain(&g, &cfg);
        // explored_goals 仍为 0,即使连续调用也不应触发
        for _ in 0..10 {
            assert!(s.check_information_gain(&g, &cfg).is_none());
        }
        assert_eq!(s.no_gain_counter, 0);
    }

    #[test]
    fn no_gain_triggers_stop_after_threshold() {
        let mut s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let g = empty_grid(4, 4);
        // seed
        let _ = s.check_information_gain(&g, &cfg);
        // 推到 min_goals_before_no_gain_check
        for _ in 0..cfg.min_goals_before_no_gain_check {
            s.mark_explored_goal(Point2 { x: 0.0, y: 0.0 });
        }
        // 连续 num_no_gain_attempts 次相同 map -> 触发 NoInformationGain
        let mut triggered = false;
        for _ in 0..cfg.num_no_gain_attempts + 1 {
            if s.check_information_gain(&g, &cfg)
                .is_some_and(|r| matches!(r, crate::frontier_goal::StopReason::NoInformationGain))
            {
                triggered = true;
                break;
            }
        }
        assert!(triggered, "NoInformationGain should be triggered");
        // 触发后 counter 清零
        assert_eq!(s.no_gain_counter, 0);
    }

    #[test]
    fn reset_clears_all_state() {
        let mut s = ExplorationSession::new();
        s.mark_explored_goal(Point2 { x: 1.0, y: 1.0 });
        s.record_goal_published();
        s.record_failure();
        s.reset();
        assert!(s.explored_goals.is_empty());
        assert!(s.exploration_direction.is_none());
        assert!(s.last_information_count.is_none());
        assert_eq!(s.no_gain_counter, 0);
        assert_eq!(s.goals_published, 0);
        assert_eq!(s.consecutive_failures, 0);
    }
}
