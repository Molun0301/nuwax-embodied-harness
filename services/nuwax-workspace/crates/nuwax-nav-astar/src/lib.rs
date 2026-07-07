//! nuwax-nav-astar
//!
//! 纯 Rust 实现 replanning A* 导航核心。对应方案文档 §5.1，参考源码为
//! `dimos/navigation/replanning_a_star/`(12 个 Python 文件,仅作 golden 行为对照)。
//!
//! 模块组织:
//! - [`astar`]               最小代价 8 邻接 A* 搜索。
//! - [`goal_validator`]      safe goal BFS / contiguous BFS。
//! - [`navigation_map`]      二元 / 梯度 costmap 缓存。
//! - [`replan_limiter`]      Python `ReplanLimiter` 兼容语义的限流器(非 D* Lite)。
//! - [`path_distancer`]      路径 lookahead / closest point / 距离计算。
//! - [`path_clearance`]      路径前向 obstacle 探测与 mask。
//! - [`controllers`]         P / PD 角速度控制器。
//! - [`local_planner`]       局部路径跟随状态机。
//! - [`global_planner`]      全局规划编排(含 replanning 状态机)。
//! - [`position_tracker`]    odom / current pose 时间窗缓存与 stuck 检测。
//! - [`helpers`]             Foundation 字段形态便捷构造器(无新类型定义)。
//!
//! ## 与 Foundation 协议面对齐
//!
//! 所有公开 API 的输入/输出类型直接使用 Foundation(`nuwax-nav-types-messages` +
//! `nuwax-nav-base`),无本地重复定义。Foundation 是单一真实来源。

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod astar;
pub mod controllers;
pub mod global_planner;
pub mod goal_validator;
pub mod helpers;
pub mod local_planner;
pub mod navigation_map;
pub mod path_clearance;
pub mod path_distancer;
pub mod position_tracker;
pub mod replan_limiter;

#[cfg(feature = "module")]
pub mod module;

// ============================================================================
// 公开 API:Foundation re-export(单一真实来源)
// ============================================================================

pub use nuwax_nav_base::{ControllerDiagnostics, NavError, NavResult, PlannerDiagnostics};
pub use nuwax_nav_types_messages::{
    BoundingBox2D,
    // sensors
    CameraInfo,
    ClockKind,
    ControllerCommand,
    CostCellTreatment,
    CostMap2D,
    CostMapPolicy,
    Detection2D,
    Detection2DArray,
    FrontierGoal,
    FrontierGoalInput,
    GoalSource,
    GridSnapshot,
    HeightCell,
    HeightMap,
    ImageEncoding,
    NavFrameId,
    NavigationGoal,
    NavigationStatus,
    OccupancyCell,
    OccupancyGrid2D,
    Odometry,
    Path2D,
    Path3D,
    PlannerFailure,
    PlannerOptions,
    PlannerRequest,
    PlannerResult,
    Point2,
    Point3,
    PointCloud,
    PointStamped,
    Pose2,
    Pose3,
    PoseStamped,
    Quaternion,
    TerrainMap,
    Timestamp,
    Twist2,
    Twist3,
    VisualTarget,
};

// 便捷构造器 re-export(从 `helpers` 模块转出)
pub use helpers::{
    make_path2d, make_path2d_empty, make_pose_stamped, make_pose_stamped_xy, make_twist2,
    make_twist2_zero, pose_distance, pose_frame, pose_to_point2, pose_to_point3, pose_xy,
    pose_xy_yaw, pose_xyz,
};

pub use astar::{min_cost_astar, AStarOptions, AStarResult};
pub use controllers::{Controller, PController, PdController};
pub use global_planner::{GlobalPlanner, GlobalPlannerConfig, GlobalPlannerOutput};
pub use goal_validator::{
    find_safe_goal, find_safe_goal_bfs, find_safe_goal_bfs_contiguous, GoalValidatorOptions,
};
pub use local_planner::{LocalPlanner, LocalPlannerState, StopMessage};
pub use navigation_map::{GradientStrategy, NavigationMap};
pub use path_clearance::PathClearance;
pub use path_distancer::PathDistancer;
pub use position_tracker::PositionTracker;
pub use replan_limiter::ReplanLimiter;
