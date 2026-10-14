//! `FrontierGoal` 公共协议:仅暴露 `FrontierGoalIdSeed` / `FrontierGoalIdSource` / `GoalSelection` / `StopReason`,
//! 导航转换不在本 crate。

use std::borrow::Cow;
use std::collections::BTreeMap;

use nuwax_nav_types_messages::geometry::Point2;
use nuwax_nav_types_messages::geometry::{PoseStamped, Timestamp};
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
#[derive(Debug, Clone, PartialEq)]
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

/// A prepared selection cannot be dispatched until its semantic delta is accepted.
#[derive(Debug)]
#[allow(clippy::large_enum_variant)]
pub enum PreparedGoalSelection {
    /// An idless candidate and its exact deferred ID-generator seed.
    Goal(FrontierGoalDraft),
    /// No candidate was found.
    NoFrontier,
    /// Exploration should stop if this selection is accepted.
    Stop(StopReason),
}

impl PreparedGoalSelection {
    /// Finalize an accepted outcome, invoking the opaque ID generator only for a goal.
    pub fn finalize(self, id_source: &mut dyn FrontierGoalIdSource) -> GoalSelection {
        match self {
            Self::Goal(draft) => GoalSelection::Goal(draft.finalize(id_source)),
            Self::NoFrontier => GoalSelection::NoFrontier,
            Self::Stop(reason) => GoalSelection::Stop(reason),
        }
    }
}

/// An idless frontier payload with the original ID-generator seed.
/// The payload is provisional advice, never a dispatchable goal.
#[derive(Debug)]
pub struct FrontierGoalDraft {
    seed: FrontierGoalIdSeed,
    /// Goal pose, frame and timestamp captured by preparation.
    pub pose: PoseStamped,
    /// Candidate score.
    pub score: f32,
    /// Estimated information gain in square meters.
    pub information_gain: Option<f32>,
    /// Path distance cost in meters.
    pub distance_cost: Option<f32>,
    /// Frontier area in square meters.
    pub frontier_area_m2: Option<f32>,
    /// Producer identity.
    pub producer: String,
    /// Original selection timestamp.
    pub stamp: Timestamp,
    /// Candidate and routing diagnostics.
    pub metadata: BTreeMap<String, String>,
}

impl FrontierGoalDraft {
    /// Original seed passed unchanged to the ID generator on accepted finalization.
    #[must_use]
    pub fn seed(&self) -> &FrontierGoalIdSeed {
        &self.seed
    }

    /// Return provisional payload for boundary/arbitration checks, without generating an ID.
    /// Its empty ID intentionally fails `FrontierGoal::validate`; never dispatch this preview.
    #[must_use]
    pub fn preview_goal(&self) -> FrontierGoal {
        FrontierGoal {
            id: String::new(),
            pose: self.pose.clone(),
            score: self.score,
            information_gain: self.information_gain,
            distance_cost: self.distance_cost,
            frontier_area_m2: self.frontier_area_m2,
            producer: self.producer.clone(),
            stamp: self.stamp,
            metadata: self.metadata.clone(),
        }
    }

    /// Convert accepted advice into a dispatchable goal using exactly one opaque ID call.
    pub fn finalize(self, id_source: &mut dyn FrontierGoalIdSource) -> FrontierGoal {
        let id = id_source.next_frontier_goal_id(&self.seed);
        self.with_id(id)
    }

    pub(crate) fn new(seed: FrontierGoalIdSeed, pose: PoseStamped) -> Self {
        Self {
            score: seed.score,
            information_gain: seed.frontier_area_m2,
            distance_cost: None,
            frontier_area_m2: seed.frontier_area_m2,
            producer: format!("nuwax-explore-wavefront/{}", env!("CARGO_PKG_VERSION")),
            stamp: seed.stamp,
            seed,
            pose,
            metadata: BTreeMap::new(),
        }
    }

    pub(crate) fn with_id(self, id: String) -> FrontierGoal {
        FrontierGoal {
            id,
            pose: self.pose,
            score: self.score,
            information_gain: self.information_gain,
            distance_cost: self.distance_cost,
            frontier_area_m2: self.frontier_area_m2,
            producer: self.producer,
            stamp: self.stamp,
            metadata: self.metadata,
        }
    }
}
