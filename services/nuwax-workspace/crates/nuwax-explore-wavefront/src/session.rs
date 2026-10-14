//! `ExplorationSession` + `Direction2`:探索会话状态(no-gain counter / direction / explored goals),
//! **不**包含 select_goal 逻辑(归属 `WavefrontGoalSelector`)。

use nuwax_nav_types_messages::geometry::Point2;

use crate::config::WavefrontConfig;
use crate::grid::occupancy_value;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct GoalFailureRecord {
    point: Point2,
    failures: u32,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct BlacklistedGoal {
    point: Point2,
    radius_m: f64,
}

/// 2D 单位方向(可为零;`from_delta` 拒绝零长度 / NaN)。
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
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
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExplorationSession {
    /// 随同一探索会话推进的区域等待状态。
    pub(crate) regional_schedule: nuwax_explore_routing::RegionalSchedule,
    /// 已发布的历史 goal(用于 explored_goals_score)
    pub(crate) explored_goals: Vec<Point2>,
    /// 上一 goal 方向(用于 momentum)
    pub(crate) exploration_direction: Option<Direction2>,
    /// 上一次 information count(用于 no-gain 判定)
    pub(crate) last_information_count: Option<u64>,
    /// 上次信息评估对应的已完成目标数，防止重复调度被计为探索尝试。
    last_information_completed_goals: usize,
    /// 连续 no-gain 计数
    pub(crate) no_gain_counter: u32,
    /// 已发布 goal 数
    pub(crate) goals_published: u32,
    /// 连续失败数
    pub(crate) consecutive_failures: u32,
    /// 按空间位置累计的导航失败次数。
    failed_goals: Vec<GoalFailureRecord>,
    /// 已耗尽重试预算的目标区域。
    blacklisted_goals: Vec<BlacklistedGoal>,
}

/// The three scalar changes produced by an information-gain check.
/// It contains no goal history; dropping it leaves the session unchanged.
#[derive(Debug)]
#[must_use = "commit this delta only when accepting its prepared selection"]
pub struct InformationGainDelta {
    last_information_count: Option<u64>,
    last_information_completed_goals: usize,
    no_gain_counter: u32,
}

impl ExplorationSession {
    /// 恢复只接纳有界、有限且自洽的确认历史；动态地图缓存不进入此载荷。
    pub fn validate_checkpoint(&self)->Result<(),String> {
        let valid_point=|p:Point2|p.x.is_finite() && p.y.is_finite() && p.x.abs()<=1e9 && p.y.abs()<=1e9;
        if self.explored_goals.len()>65536 || self.failed_goals.len()>65536 || self.blacklisted_goals.len()>65536
            || self.explored_goals.iter().any(|p|!valid_point(*p))
            || self.failed_goals.iter().any(|p|!valid_point(p.point))
            || self.blacklisted_goals.iter().any(|p|!valid_point(p.point) || !p.radius_m.is_finite() || !(0.0..=1e6).contains(&p.radius_m))
            || self.last_information_completed_goals>self.explored_goals.len()
            || self.exploration_direction.is_some_and(|d|!d.x.is_finite() || !d.y.is_finite() || (d.x.hypot(d.y)-1.0).abs()>1e-6)
        {return Err("二维探索进度无效或超过有界历史容量".into());}
        Ok(())
    }

    /// 已确认导航终态推动的区域决策轮数。
    pub fn regional_completed_rounds(&self) -> u64 {
        self.regional_schedule.completed_rounds()
    }
    /// 构造空 session
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// 标记已发布的 goal
    pub fn mark_explored_goal(&mut self, goal: Point2) {
        self.explored_goals.push(goal);
    }

    /// 由最新 goal 更新探索方向(距离不超过 0.1 m 或 delta 非法时保持不变)
    pub fn update_direction_from_goal(&mut self, previous: Point2, latest: Point2) {
        let dx = latest.x - previous.x;
        let dy = latest.y - previous.y;
        if dx.hypot(dy) <= 0.1 {
            return;
        }
        if let Some(d) = Direction2::from_delta(dx, dy) {
            self.exploration_direction = Some(d);
        }
    }

    /// 记录 goal 发布成功
    pub fn record_goal_published(&mut self) {
        self.goals_published = self.goals_published.saturating_add(1);
    }

    /// 记录失败
    pub fn record_failure(&mut self) {
        self.consecutive_failures = self.consecutive_failures.saturating_add(1);
    }

    /// 记录导航到达。只有收到真实到达反馈后，目标才进入已探索集合。
    pub fn record_goal_reached(&mut self, goal: Point2) {
        self.regional_schedule.record_outcome([goal.x, goal.y, 0.0]);
        self.mark_explored_goal(goal);
        self.consecutive_failures = 0;
        self.failed_goals
            .retain(|record| point_distance(record.point, goal) > 1.0e-6);
        self.blacklisted_goals
            .retain(|record| point_distance(record.point, goal) > record.radius_m);
    }

    /// 记录导航失败，并在失败次数超过重试预算后将目标区域加入 blacklist。
    ///
    /// 返回 `true` 表示本次反馈新触发了 blacklist。
    pub fn record_goal_failed(
        &mut self,
        goal: Point2,
        max_retries: u32,
        blacklist_radius_m: f64,
    ) -> bool {
        self.regional_schedule.record_outcome([goal.x, goal.y, 0.0]);
        self.record_failure();
        let radius_m = blacklist_radius_m.max(0.0);
        let match_radius = radius_m.max(1.0e-6);
        let failures = if let Some(record) = self
            .failed_goals
            .iter_mut()
            .find(|record| point_distance(record.point, goal) <= match_radius)
        {
            record.failures = record.failures.saturating_add(1);
            record.failures
        } else {
            self.failed_goals.push(GoalFailureRecord {
                point: goal,
                failures: 1,
            });
            1
        };

        if failures <= max_retries
            || self
                .blacklisted_goals
                .iter()
                .any(|record| point_distance(record.point, goal) <= record.radius_m)
        {
            return false;
        }
        self.blacklisted_goals.push(BlacklistedGoal {
            point: goal,
            radius_m,
        });
        true
    }

    /// 主动把一个目标区域加入 blacklist，例如 boundary 拒绝的候选点。
    pub fn blacklist_goal(&mut self, goal: Point2, radius_m: f64) {
        let radius_m = radius_m.max(0.0);
        if self
            .blacklisted_goals
            .iter()
            .any(|record| point_distance(record.point, goal) <= record.radius_m.max(radius_m))
        {
            return;
        }
        self.blacklisted_goals.push(BlacklistedGoal {
            point: goal,
            radius_m,
        });
    }

    /// 点是否落在任一 blacklist 区域内。
    #[must_use]
    pub fn is_blacklisted(&self, point: Point2) -> bool {
        self.blacklisted_goals
            .iter()
            .any(|record| point_distance(record.point, point) <= record.radius_m)
    }

    /// 当前 blacklist 区域数量。
    #[must_use]
    pub fn blacklist_count(&self) -> usize {
        self.blacklisted_goals.len()
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
                cell if occupancy_value(*cell) >= cfg.occupancy_threshold => count += 1,
                OccupancyCell::Cost(_)
                | OccupancyCell::Occupied
                | OccupancyCell::Inflated
                | OccupancyCell::Unknown => {}
            }
        }
        count
    }

    /// 判定是否触发 no-gain stop。
    ///
    /// 顺序固定(方案 §4.10):
    /// 1. 计算 `current_info`
    /// 2. 若 `last_information_count` 为 `None`,只 seed 为 `Some(current_info)` 并返回 None
    /// 3. 只有新增已完成目标、历史目标数超过阈值且上次信息量大于0时，才累计低增益次数。
    ///    增长比例低于 `cfg.info_gain_threshold_ratio` 时递增 counter;
    ///    `no_gain_counter >= cfg.num_no_gain_attempts` 返回 `Some(StopReason::NoInformationGain)` 并清零
    /// 4. 函数结束前更新 `last_information_count = Some(current_info)`
    #[must_use]
    pub fn check_information_gain(
        &mut self,
        grid: &nuwax_nav_types_messages::grid::OccupancyGrid2D,
        cfg: &WavefrontConfig,
    ) -> Option<crate::frontier_goal::StopReason> {
        let (reason, delta) = self.prepare_information_gain(grid, cfg);
        self.commit_information_gain(delta);
        reason
    }

    /// Evaluate information gain using borrowed history and return a narrow scalar delta.
    /// Resolve it before changing completed-goal history or preparing another selection.
    #[must_use]
    pub fn prepare_information_gain(
        &self,
        grid: &nuwax_nav_types_messages::grid::OccupancyGrid2D,
        cfg: &WavefrontConfig,
    ) -> (
        Option<crate::frontier_goal::StopReason>,
        InformationGainDelta,
    ) {
        let current_info = self.information_count(grid, cfg);
        let last = self.last_information_count;
        let completed_goals = self.explored_goals.len();
        let has_new_completed_goal = completed_goals > self.last_information_completed_goals;
        let mut delta = InformationGainDelta {
            last_information_count: Some(current_info),
            last_information_completed_goals: completed_goals,
            no_gain_counter: self.no_gain_counter,
        };
        if last.is_none() {
            return (None, delta);
        }
        let last = last.expect("checked is_some above");
        let stop_reason = if has_new_completed_goal
            && completed_goals as u32 > cfg.min_goals_before_no_gain_check
            && last > 0
        {
            let gain_ratio = (current_info as f64 - last as f64) / (last as f64);
            if gain_ratio < cfg.info_gain_threshold_ratio {
                delta.no_gain_counter = delta.no_gain_counter.saturating_add(1);
                if delta.no_gain_counter >= cfg.num_no_gain_attempts {
                    delta.no_gain_counter = 0;
                    Some(crate::frontier_goal::StopReason::NoInformationGain)
                } else {
                    None
                }
            } else {
                delta.no_gain_counter = 0;
                None
            }
        } else {
            None
        };
        (stop_reason, delta)
    }

    /// Accept the three information-gain scalar changes from a prepared selection.
    pub fn commit_information_gain(&mut self, delta: InformationGainDelta) {
        self.last_information_count = delta.last_information_count;
        self.last_information_completed_goals = delta.last_information_completed_goals;
        self.no_gain_counter = delta.no_gain_counter;
    }

    /// Keep confirmed world positions and publication counts, not old planning/retry decisions.
    pub fn prepare_for_replan(&mut self) {
        self.regional_schedule.reset();
        self.exploration_direction = None;
        self.last_information_count = None;
        self.last_information_completed_goals = self.explored_goals.len();
        self.no_gain_counter = 0;
        self.consecutive_failures = 0;
        self.failed_goals.clear();
        self.blacklisted_goals.clear();
    }

    /// 清理 no-frontier 选择历史，但保留导航失败、retry 与 blacklist 状态。
    pub fn reset_information_history(&mut self) {
        self.regional_schedule.reset();
        self.explored_goals.clear();
        self.exploration_direction = None;
        self.last_information_count = None;
        self.last_information_completed_goals = 0;
        self.no_gain_counter = 0;
    }

    /// 重置全部状态(explored goals、direction、no-gain、failure)
    pub fn reset(&mut self) {
        self.explored_goals.clear();
        self.exploration_direction = None;
        self.last_information_count = None;
        self.last_information_completed_goals = 0;
        self.no_gain_counter = 0;
        self.goals_published = 0;
        self.consecutive_failures = 0;
        self.failed_goals.clear();
        self.blacklisted_goals.clear();
    }
}

fn point_distance(a: Point2, b: Point2) -> f64 {
    (a.x - b.x).hypot(a.y - b.y)
}

#[cfg(test)]
mod tests {
    fn frozen_information_gain(
        session: &mut ExplorationSession,
        grid: &OccupancyGrid2D,
        cfg: &WavefrontConfig,
    ) -> Option<crate::frontier_goal::StopReason> {
        let current_info = session.information_count(grid, cfg);
        let last = session.last_information_count;
        let completed_goals = session.explored_goals.len();
        let has_new_completed_goal = completed_goals > session.last_information_completed_goals;
        session.last_information_completed_goals = completed_goals;
        if last.is_none() {
            session.last_information_count = Some(current_info);
            return None;
        }
        let last = last.expect("checked is_some above");
        let stop_reason = if has_new_completed_goal
            && completed_goals as u32 > cfg.min_goals_before_no_gain_check
            && last > 0
        {
            let gain_ratio = (current_info as f64 - last as f64) / (last as f64);
            if gain_ratio < cfg.info_gain_threshold_ratio {
                session.no_gain_counter = session.no_gain_counter.saturating_add(1);
                if session.no_gain_counter >= cfg.num_no_gain_attempts {
                    session.no_gain_counter = 0;
                    Some(crate::frontier_goal::StopReason::NoInformationGain)
                } else {
                    None
                }
            } else {
                session.no_gain_counter = 0;
                None
            }
        } else {
            None
        };
        session.last_information_count = Some(current_info);
        stop_reason
    }

    #[test]
    fn prepared_information_matches_frozen_scalars_and_discard_preserves_history() {
        let cfg = WavefrontConfig {
            min_goals_before_no_gain_check: 0,
            num_no_gain_attempts: 2,
            ..WavefrontConfig::default()
        };
        let mut direct = ExplorationSession::new();
        let mut prepared = ExplorationSession::new();
        let mut grid = empty_grid(10, 10);
        for step in 0..12 {
            grid.cells[..(20 + step * 3)].fill(OccupancyCell::Free);
            if step % 3 != 0 {
                let goal = Point2::new(step as f64, 0.0);
                direct.record_goal_reached(goal);
                prepared.record_goal_reached(goal);
            }
            let before = serde_json::to_value(&prepared).unwrap();
            let (_, discarded) = prepared.prepare_information_gain(&grid, &cfg);
            drop(discarded);
            assert_eq!(before, serde_json::to_value(&prepared).unwrap());
            let expected = frozen_information_gain(&mut direct, &grid, &cfg);
            let (actual, delta) = prepared.prepare_information_gain(&grid, &cfg);
            assert_eq!(actual, expected);
            prepared.commit_information_gain(delta);
            assert_eq!(
                serde_json::to_value(&direct).unwrap(),
                serde_json::to_value(&prepared).unwrap()
            );
        }
    }

    #[test]
    fn discarded_no_gain_stop_does_not_consume_completed_goal_or_reset_counter() {
        let cfg = WavefrontConfig {
            min_goals_before_no_gain_check: 0,
            num_no_gain_attempts: 1,
            ..WavefrontConfig::default()
        };
        let mut grid = empty_grid(4, 4);
        grid.cells.fill(OccupancyCell::Free);
        let mut session = ExplorationSession::new();
        session.check_information_gain(&grid, &cfg);
        session.record_goal_reached(Point2::new(1.0, 1.0));
        let before = serde_json::to_value(&session).unwrap();
        let (reason, discarded) = session.prepare_information_gain(&grid, &cfg);
        assert_eq!(
            reason,
            Some(crate::frontier_goal::StopReason::NoInformationGain)
        );
        drop(discarded);
        assert_eq!(before, serde_json::to_value(&session).unwrap());
        let (again, accepted) = session.prepare_information_gain(&grid, &cfg);
        assert_eq!(again, reason);
        session.commit_information_gain(accepted);
        assert_eq!(session.last_information_completed_goals, 1);
        assert_eq!(session.no_gain_counter, 0);
    }

    #[test]
    fn legacy_replan_keeps_confirmed_world_history_and_resets_derived_state() {
        let mut session = super::ExplorationSession::new();
        session.record_goal_reached(super::Point2::new(12.5, -3.25));
        session.record_goal_published();
        session.record_goal_failed(super::Point2::new(7.0, 9.0), 0, 2.0);
        session.update_direction_from_goal(super::Point2::new(0.0, 0.0), super::Point2::new(1.0, 0.0));
        session.last_information_count = Some(40);
        session.no_gain_counter = 7;
        session.prepare_for_replan();
        session.validate_checkpoint().unwrap();
        assert_eq!(session.explored_goals, vec![super::Point2::new(12.5, -3.25)]);
        assert_eq!(session.goals_published(), 1);
        assert_eq!(session.consecutive_failures(), 0);
        assert_eq!(session.blacklist_count(), 0);
        assert!(session.failed_goals.is_empty());
        assert!(session.exploration_direction().is_none());
        assert_eq!(session.no_gain_counter, 0);
        assert_eq!(session.last_information_count, None);
        assert_eq!(session.regional_completed_rounds(), 0);
    }

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
    fn direction_update_keeps_previous_value_for_python_close_goal_boundary() {
        let mut session = ExplorationSession::new();
        session.update_direction_from_goal(Point2 { x: 0.0, y: 0.0 }, Point2 { x: 1.0, y: 0.0 });
        let previous = session.exploration_direction();
        session.update_direction_from_goal(Point2 { x: 0.0, y: 0.0 }, Point2 { x: 0.0, y: 0.1 });
        assert_eq!(session.exploration_direction(), previous);
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
    fn information_count_uses_python_inclusive_occupied_threshold() {
        let s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let g = OccupancyGrid2D {
            origin: Point2 { x: 0.0, y: 0.0 },
            resolution: 0.1,
            width: 7,
            height: 1,
            cells: vec![
                OccupancyCell::Unknown,
                OccupancyCell::Free,
                OccupancyCell::Cost(98),
                OccupancyCell::Cost(99),
                OccupancyCell::Cost(100),
                OccupancyCell::Occupied,
                OccupancyCell::Inflated,
            ],
        };
        assert_eq!(s.information_count(&g, &cfg), 5);
    }

    #[test]
    fn exactly_five_goals_do_not_enable_python_no_gain_check() {
        let mut s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let g = empty_grid(4, 4);
        assert!(s.check_information_gain(&g, &cfg).is_none());
        for _ in 0..5 {
            s.mark_explored_goal(Point2 { x: 0.0, y: 0.0 });
        }
        for _ in 0..cfg.num_no_gain_attempts + 1 {
            assert!(s.check_information_gain(&g, &cfg).is_none());
        }
        assert_eq!(s.no_gain_counter, 0);
    }

    #[test]
    fn zero_previous_information_skips_python_no_gain_check() {
        let mut s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let g = empty_grid(4, 4);
        assert!(s.check_information_gain(&g, &cfg).is_none());
        for _ in 0..=cfg.min_goals_before_no_gain_check {
            s.mark_explored_goal(Point2 { x: 0.0, y: 0.0 });
        }
        for _ in 0..cfg.num_no_gain_attempts + 1 {
            assert!(s.check_information_gain(&g, &cfg).is_none());
        }
        assert_eq!(s.no_gain_counter, 0);
    }

    #[test]
    fn no_gain_triggers_stop_after_threshold() {
        let mut s = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let mut g = empty_grid(4, 4);
        g.cells.fill(OccupancyCell::Free);
        // seed
        let _ = s.check_information_gain(&g, &cfg);
        // Python enables the check only when len(explored_goals) > 5.
        for _ in 0..=cfg.min_goals_before_no_gain_check {
            s.mark_explored_goal(Point2 { x: 0.0, y: 0.0 });
        }
        // 不同已完成目标连续没有带来新地图信息，才触发低增益停止。
        let mut triggered = false;
        for _ in 0..cfg.num_no_gain_attempts + 1 {
            s.record_goal_reached(Point2 { x: 1.0, y: 1.0 });
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
    fn repeated_polls_without_a_new_reached_goal_do_not_count_as_no_gain_attempts() {
        let mut session = ExplorationSession::new();
        let cfg = WavefrontConfig::default();
        let mut grid = empty_grid(4, 5);
        grid.cells[..16].fill(OccupancyCell::Free);
        let _ = session.check_information_gain(&grid, &cfg);
        for index in 0..=cfg.min_goals_before_no_gain_check {
            session.record_goal_reached(Point2 {
                x: f64::from(index),
                y: 0.0,
            });
        }
        assert!(session.check_information_gain(&grid, &cfg).is_none());
        for _ in 0..10 {
            assert!(
                session.check_information_gain(&grid, &cfg).is_none(),
                "重复调度不是新完成的探索尝试"
            );
        }
        assert_eq!(session.no_gain_counter, 1);
        grid.cells.fill(OccupancyCell::Free);
        assert!(session.check_information_gain(&grid, &cfg).is_none());
        assert_eq!(session.last_information_count, Some(20));
        assert_eq!(
            session.no_gain_counter, 1,
            "新地图更新基准，但不是新的完成目标"
        );
        session.record_goal_reached(Point2 { x: 9.0, y: 0.0 });
        assert_eq!(
            session.check_information_gain(&grid, &cfg),
            Some(crate::frontier_goal::StopReason::NoInformationGain)
        );
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
        assert_eq!(s.last_information_completed_goals, 0);
        assert_eq!(s.no_gain_counter, 0);
        assert_eq!(s.goals_published, 0);
        assert_eq!(s.consecutive_failures, 0);
    }

    #[test]
    fn publishing_is_not_treated_as_reaching_the_goal() {
        let mut s = ExplorationSession::new();
        s.record_failure();
        s.record_goal_published();

        assert!(s.explored_goals.is_empty());
        assert_eq!(s.goals_published(), 1);
        assert_eq!(s.consecutive_failures(), 1);
    }

    #[test]
    fn reached_feedback_marks_goal_and_resets_consecutive_failures() {
        let mut s = ExplorationSession::new();
        let goal = Point2 { x: 1.5, y: -2.0 };
        s.record_failure();
        s.record_goal_reached(goal);

        assert_eq!(s.explored_goals, vec![goal]);
        assert_eq!(s.consecutive_failures(), 0);
    }

    #[test]
    fn failed_goal_is_blacklisted_after_retry_budget_is_exhausted() {
        let mut s = ExplorationSession::new();
        let goal = Point2 { x: 2.0, y: 3.0 };

        assert!(!s.record_goal_failed(goal, 1, 0.5));
        assert!(s.record_goal_failed(goal, 1, 0.5));
        assert!(s.is_blacklisted(Point2 { x: 2.2, y: 3.1 }));
        assert!(!s.is_blacklisted(Point2 { x: 3.0, y: 3.0 }));
        assert_eq!(s.consecutive_failures(), 2);
    }

    #[test]
    fn no_frontier_history_reset_preserves_retry_blacklist() {
        let mut s = ExplorationSession::new();
        let goal = Point2 { x: 2.0, y: 3.0 };
        s.record_goal_failed(goal, 0, 0.5);
        s.mark_explored_goal(Point2 { x: 1.0, y: 1.0 });

        s.reset_information_history();

        assert!(s.explored_goals.is_empty());
        assert!(s.is_blacklisted(goal));
        assert_eq!(s.consecutive_failures(), 1);
    }
}
