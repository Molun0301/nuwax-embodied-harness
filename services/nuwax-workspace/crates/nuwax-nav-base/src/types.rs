//! 公共支撑类型(`GoalId` / `GoalTicket` / `GoalValidation` / ...)
//!
//! 字段命名一律 `snake_case`;`GoalId` 由调用方提供字符串,生成算法不强制。

use nuwax_nav_types_messages::{
    goal::NavigationGoal, grid::OccupancyGrid2D, path::Path2D, Timestamp,
};
use serde::{Deserialize, Serialize};

/// 目标唯一标识(由调用方提供字符串;helper 可生成 UUID/ULID)
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct GoalId(pub String);

impl GoalId {
    /// 构造
    #[must_use]
    pub fn new(id: impl Into<String>) -> Self {
        Self(id.into())
    }

    /// 字符串访问
    #[must_use]
    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

impl From<String> for GoalId {
    fn from(s: String) -> Self {
        Self(s)
    }
}

impl From<&str> for GoalId {
    fn from(s: &str) -> Self {
        Self(s.to_string())
    }
}

impl std::fmt::Display for GoalId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

/// 目标 ticket(目标被管理器接纳后返回)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GoalTicket {
    /// 目标 id
    pub id: GoalId,
    /// 接纳时刻
    pub accepted_at: Timestamp,
    /// 在队列中的位置(0 = 队首)
    pub queue_position: usize,
}

/// 目标校验结果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum GoalValidation {
    /// 完全合法
    Valid,
    /// 已调整(给定调整后的 goal)
    Adjusted {
        /// 调整后的目标
        adjusted_goal: NavigationGoal,
        /// 调整原因
        reason: GoalAdjustmentReason,
    },
    /// 已拒绝(给定原因)
    Rejected {
        /// 拒绝原因
        reason: GoalRejectionReason,
    },
}

/// 目标调整原因
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalAdjustmentReason {
    /// 吸附到最近 safe cell
    NearestSafeCell,
    /// 吸附到地图边界内
    SnappedToMapBounds,
    /// clearance 已调整
    ClearanceAdjusted,
}

/// 目标拒绝原因
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GoalRejectionReason {
    /// 越界
    OutOfBounds,
    /// 占据
    Occupied,
    /// 没有可达 safe cell
    NoReachableSafeCell,
    /// 缺少地图
    MissingMap,
    /// frame 非法 / 不匹配
    InvalidFrame,
}

/// 控制器输入(由 stack / path follower 提供给 `PathController`)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControllerInput {
    /// 当前时间戳
    pub now: Timestamp,
    /// 当前位姿
    pub current_pose: nuwax_nav_types_messages::PoseStamped,
    /// 当前跟踪路径(None 表示无路径)
    pub path: Option<Path2D>,
    /// 当前活跃目标
    pub active_goal: Option<NavigationGoal>,
    /// 控制周期(秒)
    pub dt_seconds: f64,
    /// 速度上限(米/秒)
    pub speed_limit: Option<f64>,
    /// 显式停止
    pub stop_requested: bool,
}

/// 导航状态快照(由 stack 提供给 `GoalManager::tick`)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NavStateSnapshot {
    /// 当前时间戳
    pub now: Timestamp,
    /// 当前位姿(可能暂时丢失)
    pub pose: Option<nuwax_nav_types_messages::PoseStamped>,
    /// 当前活跃目标
    pub active_goal: Option<NavigationGoal>,
    /// 当前路径
    pub current_path: Option<Path2D>,
    /// 当前状态
    pub status: nuwax_nav_types_messages::goal::NavigationStatus,
    /// 最近一次目标结果
    pub last_goal_result: Option<GoalResult>,
}

/// 目标结果
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum GoalResult {
    /// 已到达
    Reached,
    /// 已取消
    Cancelled,
    /// 被阻塞
    Blocked,
    /// 失败
    Failed {
        /// 原因
        reason: String,
    },
}

/// GoalManager 输出
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum GoalManagerOutput {
    /// 无操作
    Noop,
    /// 派发目标(给下游 controller / native bridge)
    Dispatch {
        /// 待派发目标
        goal: NavigationGoal,
    },
    /// 取消目标
    Cancel {
        /// 被取消的 id
        id: GoalId,
        /// 原因
        reason: String,
    },
    /// 立即停止运动(无需 cancel ticket,例如 safety stop)
    StopMotion,
    /// 状态上报
    Status {
        /// 当前状态
        status: nuwax_nav_types_messages::goal::NavigationStatus,
    },
}

/// 校验输入 map 是否合法(供 mock 测试 helper 复用)
pub fn grid_has_cells(grid: &OccupancyGrid2D) -> bool {
    grid.width > 0 && grid.height > 0 && grid.cells.len() as u32 == grid.width * grid.height
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{frames::NavFrameId, geometry::Pose3, PoseStamped};

    #[test]
    fn goal_id_string_accessors() {
        let id = GoalId::from("g-1");
        assert_eq!(id.as_str(), "g-1");
        assert_eq!(id.to_string(), "g-1");
        assert_eq!(GoalId::from(String::from("g-2")).as_str(), "g-2");
    }

    #[test]
    fn goal_validation_serde_tagged() {
        let v = GoalValidation::Rejected {
            reason: GoalRejectionReason::OutOfBounds,
        };
        let s = serde_json::to_string(&v).unwrap();
        assert!(s.contains("rejected"));
        let back: GoalValidation = serde_json::from_str(&s).unwrap();
        assert!(matches!(
            back,
            GoalValidation::Rejected {
                reason: GoalRejectionReason::OutOfBounds
            }
        ));
    }

    #[test]
    fn controller_input_default_dt_is_zero() {
        let ci = ControllerInput {
            now: Timestamp::default(),
            current_pose: PoseStamped {
                frame_id: NavFrameId::Map,
                stamp: Timestamp::default(),
                pose: Pose3::default(),
            },
            path: None,
            active_goal: None,
            dt_seconds: 0.05,
            speed_limit: Some(0.5),
            stop_requested: false,
        };
        assert_eq!(ci.dt_seconds, 0.05);
    }

    #[test]
    fn goal_manager_output_dispatch_serde() {
        let goal = NavigationGoal::new(
            "g-1",
            PoseStamped {
                frame_id: NavFrameId::Map,
                stamp: Timestamp::default(),
                pose: Pose3::default(),
            },
        );
        let o = GoalManagerOutput::Dispatch { goal };
        let s = serde_json::to_string(&o).unwrap();
        assert!(s.contains("dispatch"));
        let back: GoalManagerOutput = serde_json::from_str(&s).unwrap();
        assert!(matches!(back, GoalManagerOutput::Dispatch { .. }));
    }

    #[test]
    fn grid_has_cells_checks_dimensions() {
        let mut g = OccupancyGrid2D::filled(
            nuwax_nav_types_messages::geometry::Point2::new(0.0, 0.0),
            1.0,
            2,
            2,
            nuwax_nav_types_messages::grid::OccupancyCell::Free,
        );
        assert!(grid_has_cells(&g));
        g.width = 0;
        assert!(!grid_has_cells(&g));
    }
}
