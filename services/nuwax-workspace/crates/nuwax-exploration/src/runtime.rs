//! 探索的波前输入与目标、停车输出接口。

use nuwax_nav_types_messages::geometry::PoseStamped;
use nuwax_nav_types_messages::goal::NavigationGoal;
use nuwax_nav_types_messages::grid::OccupancyGrid2D;
use nuwax_nav_types_messages::visual::FrontierGoal;

use crate::{ExplorationController, ExplorationStateSnapshot, ExploreResult};

/// 接收 wavefront 输入(costmap / odom)
pub trait WavefrontInputReceiver {
    /// 注入 costmap(用于 source adapter 缓存)
    fn ingest_costmap(&mut self, costmap: OccupancyGrid2D) -> ExploreResult<()>;
    /// 注入 odom pose
    fn ingest_odometry(&mut self, pose: PoseStamped) -> ExploreResult<()>;
}

/// 发布 exploration goal
pub trait ExplorationGoalSink {
    /// 发布 frontier goal(原始)
    fn publish_frontier_goal(&mut self, goal: FrontierGoal) -> ExploreResult<()>;
    /// 发布 navigation goal(`crate::conversions` 转换后)
    fn publish_navigation_goal(&mut self, goal: NavigationGoal) -> ExploreResult<()>;
}

/// 通知运动层在当前位姿停下
pub trait StopMotionSink {
    /// 在当前位姿停下
    fn stop_at_current_pose(&mut self, pose: PoseStamped) -> ExploreResult<()>;
}

/// Runtime adapter:包装 controller + goal sink + stop sink
///
/// 不直接持有 source adapter 句柄;`begin_exploration` / `poll_next_goal` /
/// `stop` 全部委托到内部 `ExplorationController`。Goal / Stop publish 走 sink
/// 回调。`runtime` feature 提供同步 controller;若上层需要 tokio 调度,在
/// 各自的 stack wiring 处做。
pub struct ExplorationRuntimeAdapter<C, G, S>
where
    C: AsRef<ExplorationController> + AsMut<ExplorationController>,
    G: ExplorationGoalSink,
    S: StopMotionSink,
{
    controller: C,
    goal_sink: G,
    stop_sink: S,
}

impl<C, G, S> ExplorationRuntimeAdapter<C, G, S>
where
    C: AsRef<ExplorationController> + AsMut<ExplorationController>,
    G: ExplorationGoalSink,
    S: StopMotionSink,
{
    /// 构造 adapter
    #[must_use]
    pub fn new(controller: C, goal_sink: G, stop_sink: S) -> Self {
        Self {
            controller,
            goal_sink,
            stop_sink,
        }
    }

    /// 获取 controller 引用
    #[must_use]
    pub fn controller(&self) -> &ExplorationController {
        self.controller.as_ref()
    }

    /// 获取 controller 可变引用
    pub fn controller_mut(&mut self) -> &mut ExplorationController {
        self.controller.as_mut()
    }

    /// 单次 tick:把 controller 输出转发到 goal sink / stop sink。
    ///
    /// - `Goal(goal)`:同时 publish `FrontierGoal` 与 `NavigationGoal`
    ///   (后者由 `crate::conversions::frontier_goal_to_navigation_goal` 转换)。
    /// - `Stop(reason)`:在 `ExplorationStateSnapshot::robot_pose` 位置调用 stop sink,
    ///   并调用 `controller.cancel()` 释放 active。
    /// - `Pending` / `SourceUnavailable`:不调用 sink,直接返回原始 `ExplorationPoll`。
    ///
    /// # Errors
    /// 内部 controller error 或 sink error 透传。
    pub fn tick(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<crate::ExplorationPoll> {
        let poll = self.controller.as_mut().poll_next_goal(state)?;
        match &poll {
            crate::ExplorationPoll::Goal(goal) => {
                self.goal_sink.publish_frontier_goal(goal.clone())?;
                let nav = crate::conversions::frontier_goal_to_navigation_goal(goal);
                self.goal_sink.publish_navigation_goal(nav)?;
            }
            crate::ExplorationPoll::Stop(_reason) => {
                self.stop_sink
                    .stop_at_current_pose(state.robot_pose.clone())?;
                self.controller.as_mut().cancel()?;
            }
            _ => {}
        }
        Ok(poll)
    }
}
