//! FrontierPatrolRouter：**默认只消费外部 FrontierGoal**，不实现 wavefront 算法。
//!
//! 完整 wavefront frontier 检测、frontier mask、cluster 生成和信息增益计算属于
//! `nuwax-explore-wavefront`，**不得回流**到本 crate。
//!
//! 本 router 接收集群（来自探索板块，按 `FrontierGoalInput` 提供），按 score / distance 排序，
//! 选择下一个巡逻目标。同时支持 `legacy_simple_frontier` feature，启用轻量 grid cluster 实现
//! 以兼容旧 Python `frontier_patrol_router.py`（默认关闭）。

use crate::routers::{FrontierGoalInput, GridSnapshot, PatrolRouter};
use crate::visitation_history::VisitationHistory;

/// Frontier router（默认 FrontierGoal consumer）
pub struct FrontierPatrolRouter {
    history: VisitationHistory,
    frontier_goals: Vec<FrontierGoalInput>,
}

impl FrontierPatrolRouter {
    /// 构造
    #[must_use]
    pub fn new(clearance_radius_m: f64) -> Self {
        Self {
            history: VisitationHistory::new(clearance_radius_m),
            frontier_goals: Vec::new(),
        }
    }

    /// 注入 frontier 候选列表（来自探索板块）
    pub fn set_frontier_goals(&mut self, goals: Vec<FrontierGoalInput>) {
        self.frontier_goals = goals;
    }

    /// 当前 frontier 候选数量
    #[must_use]
    pub fn frontier_goal_count(&self) -> usize {
        self.frontier_goals.len()
    }
}

impl PatrolRouter for FrontierPatrolRouter {
    fn update_grid(&mut self, grid: GridSnapshot) {
        self.history.update_grid(
            grid.width as usize,
            grid.height as usize,
            grid.resolution,
            grid.origin_x,
            grid.origin_y,
        );
    }

    fn update_pose(&mut self, pose: (f64, f64, f64)) {
        self.history.handle_odom(pose.0, pose.1);
    }

    fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
        if self.frontier_goals.is_empty() {
            return None;
        }
        // 评分 = score * visited_penalty(distance)；最简实现：score 最大者
        let mut best: Option<&FrontierGoalInput> = None;
        for g in &self.frontier_goals {
            match best {
                None => best = Some(g),
                Some(prev) => {
                    if g.score > prev.score {
                        best = Some(g);
                    }
                }
            }
        }
        let g = best?;
        Some(g.pose)
    }

    fn saturation(&self) -> f32 {
        self.history.saturation()
    }

    fn reset(&mut self) {
        self.history.reset();
        self.frontier_goals.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picks_highest_score() {
        let mut r = FrontierPatrolRouter::new(0.5);
        let grid = GridSnapshot {
            width: 5,
            height: 5,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; 25],
        };
        r.update_grid(grid);
        r.set_frontier_goals(vec![
            FrontierGoalInput {
                pose: (1.0, 0.0, 0.0),
                score: 0.5,
                information_gain: None,
                distance_cost: None,
                frontier_area_m2: None,
                producer: "wavefront".into(),
            },
            FrontierGoalInput {
                pose: (2.0, 0.0, 0.0),
                score: 0.9,
                information_gain: None,
                distance_cost: None,
                frontier_area_m2: None,
                producer: "wavefront".into(),
            },
        ]);
        let g = r.next_goal();
        assert_eq!(g, Some((2.0, 0.0, 0.0)));
    }

    #[test]
    fn no_goals_no_target() {
        let mut r = FrontierPatrolRouter::new(0.5);
        let grid = GridSnapshot {
            width: 5,
            height: 5,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; 25],
        };
        r.update_grid(grid);
        assert!(r.next_goal().is_none());
    }
}
