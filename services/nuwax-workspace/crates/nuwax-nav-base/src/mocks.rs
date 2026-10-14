//! Trait mock 实现
//!
//! 为后续 crate 单元测试提供"不需要真实 transport / 真实模型"的占位实现。
//! 这些 mock **不是**仿真实现,只用于测试 wiring;真实行为由各算法 crate 提供。

use std::collections::VecDeque;

use nuwax_nav_types_messages::{
    frames::NavFrameId,
    geometry::{Point2, Pose3, PoseStamped, Timestamp},
    goal::{NavigationGoal, PlannerRequest, PlannerResult},
    grid::OccupancyGrid2D,
    path::{ControllerCommand, Path2D},
    visual::{BoundingBox2D, ImageEncoding},
    Pose2,
};

use crate::{
    error::NavResult,
    native::ImageView,
    traits::{
        AsyncGlobalPlanner, GlobalPlanner, GoalManager, GoalValidator, PathController,
        PathPlannerExt, VisualQueryEngine,
    },
    types::{
        ControllerInput, GoalId, GoalManagerOutput, GoalTicket, GoalValidation, NavStateSnapshot,
    },
};

/// Mock global planner:返回 straight-line 路径(start→goal),或 None 表示 no path
pub struct MockStraightLinePlanner;

impl GlobalPlanner for MockStraightLinePlanner {
    fn plan(&self, request: PlannerRequest) -> NavResult<PlannerResult> {
        let start = request.start.pose.position.to_2d();
        let goal = request.goal.pose.position.to_2d();
        // 简单 mock:总是 5 个点的直线
        let mut points = Vec::with_capacity(5);
        for i in 0..5 {
            let t = i as f64 / 4.0;
            points.push(start.lerp(goal, t));
        }
        let path = Path2D::from_points(points);
        let cost = start.distance(goal);
        Ok(PlannerResult::success(path, cost, 5))
    }
}

impl AsyncGlobalPlanner for MockStraightLinePlanner {
    fn plan_async(
        &self,
        request: PlannerRequest,
    ) -> impl std::future::Future<Output = NavResult<PlannerResult>> + Send {
        let result = GlobalPlanner::plan(self, request);
        async move { result }
    }
}

impl PathPlannerExt for MockStraightLinePlanner {
    fn plan_path(
        &self,
        start: Point2,
        goal: Point2,
        _map: &OccupancyGrid2D,
    ) -> NavResult<Option<Path2D>> {
        let points = vec![start, goal];
        Ok(Some(Path2D::from_points(points)))
    }
}

/// Mock goal validator:仅检查越界,不查 occupancy
pub struct MockBoundsGoalValidator {
    /// 地图外边距(米,作为 buffer)
    pub buffer_m: f64,
}

impl Default for MockBoundsGoalValidator {
    fn default() -> Self {
        Self { buffer_m: 0.0 }
    }
}

impl GoalValidator for MockBoundsGoalValidator {
    fn validate_goal(&self, goal: &NavigationGoal, map: &OccupancyGrid2D) -> GoalValidation {
        let target = &goal.target.pose.position;
        let min_x = map.origin.x - self.buffer_m;
        let min_y = map.origin.y - self.buffer_m;
        let max_x = map.origin.x + (map.width as f64) * map.resolution + self.buffer_m;
        let max_y = map.origin.y + (map.height as f64) * map.resolution + self.buffer_m;
        if target.x < min_x || target.x > max_x || target.y < min_y || target.y > max_y {
            GoalValidation::Rejected {
                reason: crate::types::GoalRejectionReason::OutOfBounds,
            }
        } else {
            GoalValidation::Valid
        }
    }
}

/// Mock path controller:若 stop_requested 返回 stop,否则输出零 twist
pub struct MockZeroController;

impl PathController for MockZeroController {
    fn compute_command(&mut self, input: ControllerInput) -> NavResult<ControllerCommand> {
        if input.stop_requested {
            Ok(ControllerCommand::stop(input.current_pose.frame_id.clone()))
        } else {
            Ok(ControllerCommand::twist_2d(
                nuwax_nav_types_messages::geometry::Twist2::ZERO,
                input.current_pose.frame_id.clone(),
            ))
        }
    }
}

/// Mock goal manager:FIFO 队列
pub struct MockFifoGoalManager {
    queue: VecDeque<NavigationGoal>,
    /// 下一目标分配位置(预留给多消费者场景,当前未读)。
    #[allow(dead_code)]
    next_position: usize,
    /// 下一目标 ID 序号(预留给多消费者场景,当前未读)。
    #[allow(dead_code)]
    next_id_seq: u64,
}

impl MockFifoGoalManager {
    /// 构造空 manager
    #[must_use]
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
            next_position: 0,
            next_id_seq: 0,
        }
    }

    /// 队列长度
    #[must_use]
    pub fn len(&self) -> usize {
        self.queue.len()
    }

    /// 是否为空
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }
}

impl Default for MockFifoGoalManager {
    fn default() -> Self {
        Self::new()
    }
}

impl GoalManager for MockFifoGoalManager {
    fn enqueue(&mut self, goal: NavigationGoal) -> NavResult<GoalTicket> {
        let ticket = GoalTicket {
            id: GoalId::new(goal.id.clone()),
            accepted_at: Timestamp::default(),
            queue_position: self.queue.len(),
        };
        self.queue.push_back(goal);
        Ok(ticket)
    }

    fn cancel(&mut self, id: GoalId) -> NavResult<()> {
        self.queue.retain(|g| g.id != id.0);
        Ok(())
    }

    fn tick(&mut self, _state: &NavStateSnapshot) -> NavResult<GoalManagerOutput> {
        if let Some(goal) = self.queue.pop_front() {
            Ok(GoalManagerOutput::Dispatch { goal })
        } else {
            Ok(GoalManagerOutput::Noop)
        }
    }
}

/// Mock visual query engine:仅按 phrase 长度构造伪 bbox(便于测试)
pub struct MockFixedPhraseVisualQuery;

impl VisualQueryEngine for MockFixedPhraseVisualQuery {
    fn query_bbox(&self, _image: ImageView<'_>, phrase: &str) -> NavResult<Option<BoundingBox2D>> {
        if phrase.is_empty() {
            return Ok(None);
        }
        let len = phrase.len() as f32;
        Ok(BoundingBox2D::from_xyxy(0.0, 0.0, len * 10.0, len * 10.0))
    }
}

/// Mock ImageEncoding 默认值 helper
#[must_use]
pub fn default_image_encoding() -> ImageEncoding {
    ImageEncoding::Rgb8
}

/// Mock PoseStamped helper
#[must_use]
pub fn mock_pose_stamped() -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::default(),
        pose: Pose3::from_pose2(Pose2::default()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::GoalRejectionReason;

    #[test]
    fn mock_planner_returns_success() {
        let grid = OccupancyGrid2D::filled(
            Point2::new(0.0, 0.0),
            1.0,
            10,
            10,
            nuwax_nav_types_messages::grid::OccupancyCell::Free,
        );
        let req = PlannerRequest::new((0.0, 0.0), (4.0, 4.0), grid);
        let r = MockStraightLinePlanner.plan(req).unwrap();
        assert!(r.path.is_some());
        assert_eq!(r.path.unwrap().points.len(), 5);
    }

    #[test]
    fn mock_validator_rejects_out_of_bounds() {
        let grid = OccupancyGrid2D::filled(
            Point2::new(0.0, 0.0),
            1.0,
            2,
            2,
            nuwax_nav_types_messages::grid::OccupancyCell::Free,
        );
        let v = MockBoundsGoalValidator::default();
        let target = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            pose: Pose3::from_pose2(Pose2::new(100.0, 0.0, 0.0)),
        };
        let g = NavigationGoal::new("g", target);
        assert!(matches!(
            v.validate_goal(&g, &grid),
            GoalValidation::Rejected {
                reason: GoalRejectionReason::OutOfBounds
            }
        ));
    }

    #[test]
    fn mock_controller_stop_returns_stop() {
        let mut c = MockZeroController;
        let input = ControllerInput {
            now: Timestamp::default(),
            current_pose: mock_pose_stamped(),
            path: None,
            active_goal: None,
            dt_seconds: 0.05,
            speed_limit: None,
            stop_requested: true,
        };
        let cmd = c.compute_command(input).unwrap();
        assert!(cmd.stop);
    }

    #[test]
    fn mock_manager_dispatch_in_fifo_order() {
        let mut m = MockFifoGoalManager::new();
        let g1 = NavigationGoal::new("a", mock_pose_stamped());
        let g2 = NavigationGoal::new("b", mock_pose_stamped());
        m.enqueue(g1).unwrap();
        m.enqueue(g2).unwrap();
        assert_eq!(m.len(), 2);
        let snap = NavStateSnapshot {
            now: Timestamp::default(),
            pose: Some(mock_pose_stamped()),
            active_goal: None,
            current_path: None,
            status: nuwax_nav_types_messages::goal::NavigationStatus::Idle,
            last_goal_result: None,
        };
        let o1 = m.tick(&snap).unwrap();
        assert!(matches!(o1, GoalManagerOutput::Dispatch { ref goal } if goal.id == "a"));
        let o2 = m.tick(&snap).unwrap();
        assert!(matches!(o2, GoalManagerOutput::Dispatch { ref goal } if goal.id == "b"));
        let o3 = m.tick(&snap).unwrap();
        assert!(matches!(o3, GoalManagerOutput::Noop));
    }

    #[test]
    fn mock_visual_query_empty_phrase_returns_none() {
        let m = MockFixedPhraseVisualQuery;
        let data = vec![0u8; 3];
        let v = ImageView::from_rgb(1, 1, &data);
        assert!(m.query_bbox(v, "").unwrap().is_none());
    }

    #[test]
    fn mock_visual_query_nonzero_phrase_returns_bbox() {
        let m = MockFixedPhraseVisualQuery;
        let data = vec![0u8; 6];
        let v = ImageView::from_rgb(2, 1, &data);
        let bb = m.query_bbox(v, "cup").unwrap().unwrap();
        assert!(bb.area() > 0.0);
    }
}
