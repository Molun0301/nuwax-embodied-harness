//! 共享测试夹具构造器
//!
//! 每个 fixture 都提供 `*_json()` 返回 `serde_json::Value`,便于
//! - 直接做 serde roundtrip 断言
//! - 写入 fixture 文件参与 golden 对比

use nuwax_nav_base::diagnostics::NavDiagnostics;
use nuwax_nav_types_messages::diagnostics::DiagnosticLevel;
use nuwax_nav_types_messages::{
    frames::NavFrameId,
    geometry::{Point2, Point3, Pose3, PoseStamped, Quaternion, Timestamp, Twist2},
    goal::{
        GoalSource, NavigationGoal, NavigationStatus, PlannerFailure, PlannerOptions,
        PlannerRequest, PlannerResult,
    },
    grid::{CostMap2D, CostMapPolicy, OccupancyCell, OccupancyGrid2D},
    path::{ControllerCommand, Path2D},
    visual::{BoundingBox2D, FrontierGoal, VisualTarget},
};

fn map_frame() -> NavFrameId {
    NavFrameId::Map
}

/// 构造一个 `PoseStamped` fixture(世界 (x, y, yaw))
#[must_use]
pub fn pose(x: f64, y: f64, yaw: f64) -> PoseStamped {
    PoseStamped {
        frame_id: map_frame(),
        stamp: Timestamp::from_unix(0, 0),
        pose: Pose3::from_pose2(nuwax_nav_types_messages::geometry::Pose2::new(x, y, yaw)),
    }
}

/// 构造 4x4 free occupancy grid fixture
#[must_use]
pub fn grid_4x4_free() -> OccupancyGrid2D {
    OccupancyGrid2D::filled(Point2::new(0.0, 0.0), 1.0, 4, 4, OccupancyCell::Free)
}

/// 构造 4x4 grid 中放置障碍(中心 2x2 区域为 Occupied)
#[must_use]
pub fn grid_4x4_with_obstacle() -> OccupancyGrid2D {
    let mut g = grid_4x4_free();
    g.set(1, 1, OccupancyCell::Occupied);
    g.set(2, 1, OccupancyCell::Occupied);
    g.set(1, 2, OccupancyCell::Occupied);
    g.set(2, 2, OccupancyCell::Occupied);
    g
}

/// 构造简单 2D path(直线 3 点)
#[must_use]
pub fn straight_path(a: Point2, b: Point2) -> Path2D {
    Path2D::from_points(vec![
        a,
        Point2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0),
        b,
    ])
}

/// 构造 NavigationGoal fixture
#[must_use]
pub fn goal(id: &str, x: f64, y: f64) -> NavigationGoal {
    NavigationGoal::new(id, pose(x, y, 0.0))
        .with_source(GoalSource::Click)
        .with_priority(0)
}

/// 构造 BoundingBox2D fixture(xyxy 像素坐标)
#[must_use]
pub fn bbox(x1: f32, y1: f32, x2: f32, y2: f32) -> BoundingBox2D {
    BoundingBox2D::from_xyxy(x1, y1, x2, y2).expect("bbox fixture must be valid")
}

/// 构造 VisualTarget fixture
#[must_use]
pub fn visual_target(phrase: &str) -> VisualTarget {
    VisualTarget::new(bbox(10.0, 20.0, 110.0, 220.0), phrase, NavFrameId::Camera)
}

/// 构造 ControllerCommand fixture
#[must_use]
pub fn command(linear: f64, angular: f64) -> ControllerCommand {
    ControllerCommand::twist_2d(
        Twist2 {
            linear: Point2::new(linear, 0.0),
            angular_z: angular,
        },
        NavFrameId::Base,
    )
}

/// 构造 PlannerRequest fixture
#[must_use]
pub fn planner_request(start: (f64, f64), goal: (f64, f64)) -> PlannerRequest {
    PlannerRequest {
        start: pose(start.0, start.1, 0.0),
        goal: pose(goal.0, goal.1, 0.0),
        map: grid_4x4_with_obstacle(),
        options: PlannerOptions::default(),
    }
}

/// 构造 PlannerResult 失败 fixture
#[must_use]
pub fn planner_result_no_path() -> PlannerResult {
    PlannerResult::failure(PlannerFailure::NoPath, 12)
}

/// 构造 CostMap2D fixture(全 free + zero cost)
#[must_use]
pub fn costmap_zero_4x4() -> CostMap2D {
    CostMap2D::from_grid_with_costs(&grid_4x4_free(), &[0.0; 16])
}

/// 构造 CostMapPolicy fixture(默认)
#[must_use]
pub fn default_policy() -> CostMapPolicy {
    CostMapPolicy::default()
}

/// 构造 NavDiagnostics fixture
#[must_use]
pub fn nav_diagnostics() -> NavDiagnostics {
    NavDiagnostics {
        component: "nuwax-nav-stack".to_string(),
        state: "Moving".to_string(),
        input_age_ns: Some(50_000_000),
        last_output_age_ns: Some(20_000_000),
        last_tick_latency_ns: Some(2_000_000),
        last_error: None,
        counters: nuwax_nav_base::diagnostics::Counters::default(),
        last_stamp: Some(Timestamp::from_unix(0, 0)),
    }
}

/// 构造 NavigationStatus 列表
#[must_use]
pub fn all_statuses() -> Vec<NavigationStatus> {
    vec![
        NavigationStatus::Idle,
        NavigationStatus::Planning,
        NavigationStatus::Moving,
        NavigationStatus::Reached,
        NavigationStatus::Blocked,
        NavigationStatus::Cancelled,
        NavigationStatus::Error,
    ]
}

/// 构造 FrontierGoal fixture
#[must_use]
pub fn frontier_goal(id: &str, x: f64, y: f64, score: f32) -> FrontierGoal {
    let mut g = FrontierGoal::new(id, pose(x, y, 0.0), score);
    g.information_gain = Some(2.5);
    g.distance_cost = Some(3.1);
    g.frontier_area_m2 = Some(1.0);
    g.producer = "wavefront_v1".to_string();
    g.stamp = Timestamp::from_unix(0, 0);
    g.metadata.insert("cell_count".into(), "5".into());
    g
}

/// DiagnosticLevel 全列表
#[must_use]
pub fn all_levels() -> Vec<DiagnosticLevel> {
    vec![
        DiagnosticLevel::Trace,
        DiagnosticLevel::Debug,
        DiagnosticLevel::Info,
        DiagnosticLevel::Warn,
        DiagnosticLevel::Error,
    ]
}

/// 单测 helper:把任意 Serialize 对象转 JSON Value
pub fn to_json<T: serde::Serialize>(v: &T) -> serde_json::Value {
    serde_json::to_value(v).expect("serialize fixture")
}

/// Quaternion identity helper
#[must_use]
pub fn identity_quaternion() -> Quaternion {
    Quaternion::IDENTITY
}

/// Point3 helper
#[must_use]
pub fn point3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_helper_basic() {
        let p = pose(1.0, 2.0, 0.5);
        assert_eq!(p.frame_id, NavFrameId::Map);
        assert!((p.pose.position.x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn obstacle_grid_centers_marked() {
        let g = grid_4x4_with_obstacle();
        assert_eq!(g.get(1, 1), Some(OccupancyCell::Occupied));
        assert_eq!(g.get(0, 0), Some(OccupancyCell::Free));
    }

    #[test]
    fn straight_path_has_three_points() {
        let p = straight_path(Point2::new(0.0, 0.0), Point2::new(2.0, 0.0));
        assert_eq!(p.points.len(), 3);
    }

    #[test]
    fn bbox_fixture_is_valid() {
        let b = bbox(1.0, 2.0, 3.0, 4.0);
        assert!(b.is_valid());
    }

    #[test]
    fn planner_request_fixture_has_options() {
        let r = planner_request((0.0, 0.0), (3.0, 3.0));
        assert_eq!(r.options.cost_threshold, 100);
    }

    #[test]
    fn to_json_roundtrip_path() {
        let p = straight_path(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0));
        let j = to_json(&p);
        assert!(j["points"].is_array());
        let back: Path2D = serde_json::from_value(j).unwrap();
        assert_eq!(back.points.len(), 3);
    }

    #[test]
    fn frontier_goal_fixture_validates() {
        let g = frontier_goal("f-1", 1.0, 1.0, 0.7);
        assert!(g.validate().is_ok());
    }
}
