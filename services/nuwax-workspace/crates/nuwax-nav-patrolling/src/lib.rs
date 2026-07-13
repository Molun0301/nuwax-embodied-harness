//! 巡逻路由：根据 occupancy grid / 当前位置 / 访问历史生成巡逻目标。
//!
//! 行为对账源：`dimos/navigation/patrolling/`。
//!
//! 硬边界：
//! - 完整 wavefront frontier 算法归 `nuwax-explore-wavefront`，本 crate 不持有。
//! - `FrontierPatrolRouter` 默认只消费外部传入的 `FrontierGoal` 列表并按 score / 距离 / visited 选目标。
//! - 提供 `legacy_simple_frontier` feature 默认关闭，可启用 grid cluster 的轻量实现以兼容旧 `frontier_patrol_router.py`。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use serde::{Deserialize, Serialize};

// Foundation 协议面 re-export — 跨 crate FrontierGoal / OccupancyGrid2D 协议语义对齐
pub use nuwax_nav_base::NavError as FoundationNavError;
pub use nuwax_nav_types_messages::{
    FrontierGoal as FoundationFrontierGoal, NavigationGoal as FoundationNavigationGoal,
    OccupancyGrid2D as FoundationOccupancyGrid2D, Point2 as FoundationPoint2,
    PoseStamped as FoundationPoseStamped,
};

pub mod module;
pub mod routers;
pub mod visitation_history;

pub use module::{Module, PatrollingModule, PatrollingModuleConfig};
pub use routers::{
    CoveragePatrolRouter, FrontierGoalInput, FrontierPatrolRouter, PatrolRouter,
    PatrollingContext, RandomPatrolRouter,
};

pub use visitation_history::VisitationHistory;

/// PatrolRouterName
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PatrolRouterName {
    /// random router
    Random,
    /// coverage router
    Coverage,
    /// frontier router (consumes external FrontierGoal list)
    Frontier,
}

impl PatrolRouterName {
    /// 从字符串解析
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "random" => Some(Self::Random),
            "coverage" => Some(Self::Coverage),
            "frontier" => Some(Self::Frontier),
            _ => None,
        }
    }

    /// 字符串名
    #[must_use]
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Random => "random",
            Self::Coverage => "coverage",
            Self::Frontier => "frontier",
        }
    }
}

/// 工厂错误
#[derive(Debug, thiserror::Error)]
pub enum PatrolRouterFactoryError {
    /// 未知 router 名
    #[error("unknown patrol router name: {0}")]
    UnknownName(String),
}

/// 工厂创建 router
pub fn create_patrol_router(
    name: PatrolRouterName,
    clearance_radius_m: f64,
    candidates_to_consider: usize,
    rng_seed: Option<u64>,
) -> Box<dyn PatrolRouter> {
    match name {
        PatrolRouterName::Random => Box::new(RandomPatrolRouter::new(clearance_radius_m, rng_seed)),
        PatrolRouterName::Coverage => Box::new(CoveragePatrolRouter::new(
            clearance_radius_m,
            candidates_to_consider,
            rng_seed,
        )),
        PatrolRouterName::Frontier => Box::new(FrontierPatrolRouter::new(clearance_radius_m)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn router_name_parse() {
        assert_eq!(
            PatrolRouterName::parse("random"),
            Some(PatrolRouterName::Random)
        );
        assert_eq!(
            PatrolRouterName::parse("coverage"),
            Some(PatrolRouterName::Coverage)
        );
        assert_eq!(
            PatrolRouterName::parse("frontier"),
            Some(PatrolRouterName::Frontier)
        );
        assert_eq!(PatrolRouterName::parse("nope"), None);
        assert_eq!(PatrolRouterName::Random.as_str(), "random");
    }
}
