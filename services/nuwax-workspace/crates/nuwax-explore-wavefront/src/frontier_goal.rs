//! `FrontierGoal` 公共协议:仅暴露 `FrontierGoalIdSeed` / `FrontierGoalIdSource` / `GoalSelection` / `StopReason`,
//! 导航转换不在本 crate。

use std::borrow::Cow;

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::geometry::Timestamp;
use nuwax_nav_types_messages::visual::FrontierGoal;

/// `WavefrontGoalSelector::select_goal_at` 输出。
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // 频繁 clone,box 会增加堆分配;语义上仅 ≤ 3 变体
pub enum GoalSelection {
    /// 已选择 frontier goal
    Goal(FrontierGoal),
    /// 未找到 frontier(可由 facade arbitration 决定 fallback)
    NoFrontier,
    /// 探索需要停止
    Stop(StopReason),
}

/// 停止原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StopReason {
    /// 连续 N 次无信息增益(no-gain counter 触发)
    NoInformationGain,
    /// BFS 找不到可达 free cell(机器人起点被封死)
    NoReachableFreeSpace,
    /// 连续失败次数超限
    ConsecutiveFailuresExceeded,
    /// 用户主动取消(`request_stop_movement` 触发,`MEDIUM-2` 修正)
    UserCancelled,
}

/// 由 caller 提供的 frontier goal id 种子(只暴露摘要,不暴露 crate-private `FrontierCandidate`)。
#[derive(Debug, Clone)]
pub struct FrontierGoalIdSeed {
    /// 候选 cluster 中心(世界坐标)
    pub centroid: Point2,
    /// 候选总分
    pub score: f32,
    /// 候选覆盖面积(平方米,可选)
    pub frontier_area_m2: Option<f32>,
    /// frontier 产生时间戳
    pub stamp: Timestamp,
    /// 产生方(实例/source name,使用 `Cow<'static, str>` 支持静态与动态命名)
    pub source_name: Cow<'static, str>,
}

/// frontier goal id 生成器 trait(测试可注入 deterministic generator)。
pub trait FrontierGoalIdSource {
    /// 由 seed 派生唯一 id
    fn next_frontier_goal_id(&mut self, seed: &FrontierGoalIdSeed) -> String;
}
