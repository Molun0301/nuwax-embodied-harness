//! `WavefrontDiagnostics` / `FrontierDebugSummary` / `FrontierScoreBreakdown`(摘要级,
//! 不暴露 cell list / mask / BFS queue)。

use nuwax_nav_types_messages::geometry::Point2;
use serde::{Deserialize, Serialize};

/// 5 项 frontier score 分解(总权重 0.3*info + 0.3*explored + 0.2*distance + 0.15*obstacle + 0.05*momentum)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrontierScoreBreakdown {
    /// 总分
    pub total: f32,
    /// 信息增益得分
    pub information_gain: f32,
    /// 历史目标接近度得分
    pub explored_goals: f32,
    /// 距离得分
    pub distance: f32,
    /// 障碍物清空得分
    pub obstacle_clearance: f32,
    /// 方向动量得分
    pub direction_momentum: f32,
}

/// 单个 frontier 候选的摘要(不暴露 cell list)。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrontierDebugSummary {
    /// 候选 cluster 中心(世界坐标)
    pub centroid: Point2,
    /// 候选 cell 数量
    pub cell_count: usize,
    /// 候选周长(米)
    pub perimeter_m: f64,
    /// 总分
    pub score: f32,
    /// 分项明细
    pub score_breakdown: FrontierScoreBreakdown,
}

/// Wavefront detector 摘要诊断。
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct WavefrontDiagnostics {
    /// 最近一次检测候选摘要(只读摘要,不暴露 cell list / mask)
    pub last_candidates: Vec<FrontierDebugSummary>,
    /// 最近一次 select 路径:`NoFrontier` / `Stop(...)` / `Goal(...)`
    pub last_selection_kind: Option<String>,
    /// 最近一次 select goal id(若产生)
    pub last_goal_id: Option<String>,
    /// no-gain counter(由 session 维护)
    pub no_gain_counter: u32,
    /// 已发布 goal 数
    pub goals_published: u32,
    /// 连续失败数
    pub consecutive_failures: u32,
}
