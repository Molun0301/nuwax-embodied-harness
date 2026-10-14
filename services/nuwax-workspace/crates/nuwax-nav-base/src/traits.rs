//! 核心 trait
//!
//! 所有公开 trait 的输入/输出只使用 `nuwax-nav-types-messages` 或 shared crate 类型,
//! 不依赖具体算法 crate。供 `A*` / `simple-planner` / `path-follower` / `patrolling` /
//! `visual-query` / `movement-manager` 等实现。

use nuwax_nav_types_messages::{
    goal::{NavigationGoal, PlannerRequest, PlannerResult},
    grid::OccupancyGrid2D,
    path::Path2D,
    visual::BoundingBox2D,
};

use crate::{
    error::NavResult,
    native::ImageView,
    types::{
        ControllerInput, GoalId, GoalManagerOutput, GoalTicket, GoalValidation, NavStateSnapshot,
    },
};

/// 全局规划器(同步)
pub trait GlobalPlanner: Send + Sync {
    /// 同步规划
    fn plan(&self, request: PlannerRequest) -> NavResult<PlannerResult>;
}

/// 全局规划器(异步)
pub trait AsyncGlobalPlanner: Send + Sync {
    /// 异步规划
    fn plan_async(
        &self,
        request: PlannerRequest,
    ) -> impl std::future::Future<Output = NavResult<PlannerResult>> + Send;
}

/// 目标校验器(基于 occupancy grid)
pub trait GoalValidator: Send + Sync {
    /// 校验给定 goal 是否合法 / 调整 / 拒绝
    fn validate_goal(&self, goal: &NavigationGoal, map: &OccupancyGrid2D) -> GoalValidation;
}

/// 路径控制器(path follower / visual servoing 实现)
pub trait PathController: Send + Sync {
    /// 计算下一步命令
    fn compute_command(
        &mut self,
        input: ControllerInput,
    ) -> NavResult<nuwax_nav_types_messages::path::ControllerCommand>;
}

/// 目标管理器(队列、取消、tick)
pub trait GoalManager: Send + Sync {
    /// 入队新目标
    fn enqueue(&mut self, goal: NavigationGoal) -> NavResult<GoalTicket>;
    /// 取消目标
    fn cancel(&mut self, id: GoalId) -> NavResult<()>;
    /// tick(消费最新状态,返回本次输出)
    fn tick(&mut self, state: &NavStateSnapshot) -> NavResult<GoalManagerOutput>;
}

/// 视觉 query 引擎(返回 bbox 或 None)
pub trait VisualQueryEngine: Send + Sync {
    /// 由图像 + 短语查询 bbox
    fn query_bbox(&self, image: ImageView<'_>, phrase: &str) -> NavResult<Option<BoundingBox2D>>;
}

/// 可选辅助 trait:规划器可消费额外上下文(地图/路径),用于 `GlobalPlanner` 实现中扩展
pub trait PathPlannerExt: Send + Sync {
    /// 由 start / goal / map 直接生成 2D 路径(供 path follower 直接使用)
    fn plan_path(
        &self,
        start: nuwax_nav_types_messages::geometry::Point2,
        goal: nuwax_nav_types_messages::geometry::Point2,
        map: &OccupancyGrid2D,
    ) -> NavResult<Option<Path2D>>;
}
