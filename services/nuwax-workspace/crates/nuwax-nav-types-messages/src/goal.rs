//! 导航目标、状态机与 planner 请求/结果
//!
//! `NavigationStatus` 状态机(`栈/编排 crate` 必须按以下转换):
//! - `Idle -> Planning -> Moving -> Reached | Blocked | Cancelled | Error`
//! - `Blocked -> Planning (replan) | Cancelled (exhausted retries)`
//! - 任意状态可被 `Cancelled` 中断(由用户 / movement-manager / patrolling 触发)

use serde::{Deserialize, Serialize};

use crate::geometry::{Point3, Pose3, PoseStamped, Quaternion, Timestamp};
use crate::grid::OccupancyGrid2D;
use crate::path::Path2D;

/// 目标来源
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum GoalSource {
    /// 来自 UI / RViz 点击
    Click,
    /// 来自 agent / LLM 调用
    Agent,
    /// 来自巡逻策略
    Patrol,
    /// 来自 frontier exploration(探索板块)
    Frontier,
    /// 来自视觉 query
    Visual,
    /// 来自回放 / rosbag
    Replay,
    /// 人工(CLI / RPC)
    #[default]
    Manual,
    /// 其它来源(字符串透传)
    Other(String),
}

/// 导航目标
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavigationGoal {
    /// 唯一 id(由调用方提供;helper 可生成 UUID/ULID)
    pub id: String,
    /// 目标位姿(以 stamp 时刻,坐标系 frame_id)
    pub target: PoseStamped,
    /// 到达容差(米)
    pub position_tolerance_m: f64,
    /// 朝向容差(弧度)
    pub yaw_tolerance_rad: f64,
    /// 优先级;数值越大越优先
    pub priority: i32,
    /// 来源
    pub source: GoalSource,
}

impl NavigationGoal {
    /// 构造一个简单 goal(默认容差 0.5m / 0.2 rad)
    #[must_use]
    pub fn new(id: impl Into<String>, target: PoseStamped) -> Self {
        Self {
            id: id.into(),
            target,
            position_tolerance_m: 0.5,
            yaw_tolerance_rad: 0.2,
            priority: 0,
            source: GoalSource::default(),
        }
    }

    /// 设置来源并返回 self
    #[must_use]
    pub fn with_source(mut self, source: GoalSource) -> Self {
        self.source = source;
        self
    }

    /// 设置优先级并返回 self
    #[must_use]
    pub fn with_priority(mut self, priority: i32) -> Self {
        self.priority = priority;
        self
    }
}

/// 导航状态机
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavigationStatus {
    /// 空闲,等待新目标
    #[default]
    Idle,
    /// 正在规划全局路径
    Planning,
    /// 正在跟随路径
    Moving,
    /// 已到达
    Reached,
    /// 被障碍 / 无解阻塞
    Blocked,
    /// 被取消
    Cancelled,
    /// 内部错误(由 `NavError` 描述)
    Error,
}

impl NavigationStatus {
    /// 是否处于"活跃"状态(Planning / Moving)
    #[must_use]
    pub fn is_active(self) -> bool {
        matches!(self, Self::Planning | Self::Moving)
    }

    /// 是否为终态
    #[must_use]
    pub fn is_terminal(self) -> bool {
        matches!(
            self,
            Self::Reached | Self::Blocked | Self::Cancelled | Self::Error
        )
    }
}

/// planner 选项
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannerOptions {
    /// cost 阈值(>= 视为不可通行)
    pub cost_threshold: u8,
    /// unknown cell 惩罚因子;乘到 cost_threshold 之上判断不可走
    pub unknown_penalty: f32,
    /// 对角代价(默认 1.42 = sqrt(2))
    pub diagonal_cost: f32,
    /// A* 最大扩展节点数(None = 不限)
    pub max_expansions: Option<usize>,
}

impl Default for PlannerOptions {
    fn default() -> Self {
        Self {
            cost_threshold: 100,
            unknown_penalty: 1.0,
            diagonal_cost: 1.42,
            max_expansions: None,
        }
    }
}

/// planner 请求
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannerRequest {
    /// 起点
    pub start: PoseStamped,
    /// 终点(世界坐标 + frame)
    pub goal: PoseStamped,
    /// 占据栅格
    pub map: OccupancyGrid2D,
    /// planner 选项
    pub options: PlannerOptions,
}

impl PlannerRequest {
    /// 构造时设置起点和终点(world 2D + 默认 frame)
    #[must_use]
    pub fn new(start_xy: (f64, f64), goal_xy: (f64, f64), map: OccupancyGrid2D) -> Self {
        let start_pose = PoseStamped {
            frame_id: map_frame(),
            stamp: Timestamp::default(),
            pose: Pose3 {
                position: Point3 {
                    x: start_xy.0,
                    y: start_xy.1,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        let goal_pose = PoseStamped {
            frame_id: map_frame(),
            stamp: Timestamp::default(),
            pose: Pose3 {
                position: Point3 {
                    x: goal_xy.0,
                    y: goal_xy.1,
                    z: 0.0,
                },
                orientation: Quaternion::IDENTITY,
            },
        };
        Self {
            start: start_pose,
            goal: goal_pose,
            map,
            options: PlannerOptions::default(),
        }
    }
}

fn map_frame() -> crate::frames::NavFrameId {
    crate::frames::NavFrameId::Map
}

/// planner 结果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannerResult {
    /// 规划路径(成功)
    pub path: Option<Path2D>,
    /// 状态(成功 / 失败 / 被取消)
    pub status: NavigationStatus,
    /// 累计 cost(成功时)
    pub cost: Option<f64>,
    /// 探索节点数(诊断)
    pub explored_nodes: usize,
    /// 失败原因(失败时,不依赖字符串解析)
    pub failure: Option<PlannerFailure>,
}

/// planner 失败原因(枚举而非字符串,避免调用方解析字符串)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlannerFailure {
    /// 起点越界
    StartOutOfBounds,
    /// 终点越界
    GoalOutOfBounds,
    /// 起点不可通行
    StartBlocked,
    /// 终点不可通行
    GoalBlocked,
    /// 无可达路径
    NoPath,
    /// 超过最大扩展节点数
    ExpansionBudgetExhausted,
    /// 地图为空
    EmptyMap,
    /// 内部错误
    Internal,
}

impl PlannerResult {
    /// 成功的便捷构造
    #[must_use]
    pub fn success(path: Path2D, cost: f64, explored_nodes: usize) -> Self {
        Self {
            path: Some(path),
            status: NavigationStatus::Planning,
            cost: Some(cost),
            explored_nodes,
            failure: None,
        }
    }

    /// 失败的便捷构造
    #[must_use]
    pub fn failure(failure: PlannerFailure, explored_nodes: usize) -> Self {
        let status = match failure {
            PlannerFailure::StartOutOfBounds
            | PlannerFailure::GoalOutOfBounds
            | PlannerFailure::StartBlocked
            | PlannerFailure::GoalBlocked
            | PlannerFailure::EmptyMap => NavigationStatus::Error,
            PlannerFailure::NoPath | PlannerFailure::ExpansionBudgetExhausted => {
                NavigationStatus::Blocked
            }
            PlannerFailure::Internal => NavigationStatus::Error,
        };
        Self {
            path: None,
            status,
            cost: None,
            explored_nodes,
            failure: Some(failure),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::NavFrameId;

    #[test]
    fn navigation_goal_with_helpers() {
        let target = PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::default(),
            pose: Pose3::default(),
        };
        let g = NavigationGoal::new("goal-1", target)
            .with_source(GoalSource::Agent)
            .with_priority(10);
        assert_eq!(g.id, "goal-1");
        assert_eq!(g.source, GoalSource::Agent);
        assert_eq!(g.priority, 10);
    }

    #[test]
    fn status_terminal_and_active_partition() {
        assert!(!NavigationStatus::Idle.is_terminal());
        assert!(NavigationStatus::Planning.is_active());
        assert!(NavigationStatus::Moving.is_active());
        assert!(NavigationStatus::Reached.is_terminal());
        assert!(NavigationStatus::Blocked.is_terminal());
        assert!(NavigationStatus::Cancelled.is_terminal());
        assert!(NavigationStatus::Error.is_terminal());
    }

    #[test]
    fn goal_source_other_passthrough() {
        let s = GoalSource::Other("voice".to_string());
        let json = serde_json::to_string(&s).unwrap();
        assert!(json.contains("voice"));
        let back: GoalSource = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn planner_result_failure_maps_status() {
        let r = PlannerResult::failure(PlannerFailure::NoPath, 100);
        assert_eq!(r.status, NavigationStatus::Blocked);
        assert_eq!(r.failure, Some(PlannerFailure::NoPath));
        assert!(r.path.is_none());
    }

    #[test]
    fn planner_result_success_has_path_and_cost() {
        let p = Path2D::from_points(vec![]);
        let r = PlannerResult::success(p, 3.5, 10);
        assert_eq!(r.cost, Some(3.5));
        assert_eq!(r.explored_nodes, 10);
        assert!(r.path.is_some());
    }

    #[test]
    fn planner_request_serde_roundtrip() {
        let grid = OccupancyGrid2D::filled(
            crate::geometry::Point2::new(0.0, 0.0),
            1.0,
            2,
            2,
            crate::grid::OccupancyCell::Free,
        );
        let req = PlannerRequest::new((0.0, 0.0), (1.0, 1.0), grid);
        let s = serde_json::to_string(&req).unwrap();
        let back: PlannerRequest = serde_json::from_str(&s).unwrap();
        assert_eq!(back.options.cost_threshold, 100);
        assert_eq!(back.start.pose.position.x, 0.0);
        assert_eq!(back.goal.pose.position.x, 1.0);
    }
}
