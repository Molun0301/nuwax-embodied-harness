//! 导航原编号与执行选项；与通信和具体规划器解耦。
use serde::{Deserialize, Serialize};

/// Geometry owner 已分配的目标编号；非空编号各分量必须在正 i64 范围。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GoalKey {
    /// 分配目标的 Geometry runtime 代次。
    pub producer_runtime_generation: u64,
    /// 该 runtime 中的目标代次。
    pub goal_generation: u64,
}

/// 规划 owner 已分配的路径编号，不由路径内容派生。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PathKey {
    /// 路径所属的原目标。
    pub goal: GoalKey,
    /// 生成路径的 planner runtime 代次。
    pub planner_runtime_generation: u64,
    /// 该目标的路径修订号。
    pub path_revision: u64,
}

/// 必须随目标传递的执行约束。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GoalOptions {
    /// 当前目标是否允许倒车，仍须匹配硬件能力。
    pub allow_reverse: bool,
    /// 实际位置到达容差，米。
    pub position_tolerance_m: f64,
    /// 实际朝向到达容差，弧度。
    pub yaw_tolerance_rad: f64,
}
impl Default for GoalOptions {
    fn default()->Self {Self {allow_reverse:false,position_tolerance_m:0.35,yaw_tolerance_rad:0.35}}
}
impl GoalOptions {
    /// 容差覆盖整个圆周时，不约束最终朝向；仍保留沿途机体方向。
    pub fn requires_orientation(&self) -> bool { self.yaw_tolerance_rad < std::f64::consts::PI }

    /// 校验数值；零容差仅保留数值精度边界，不替换为默认容差。
    pub fn validate(&self)->Result<(),String> {
        if !self.position_tolerance_m.is_finite() || self.position_tolerance_m < 0.0
            || !self.yaw_tolerance_rad.is_finite() || self.yaw_tolerance_rad < 0.0 {return Err("导航目标容差无效".into());}
        Ok(())
    }
}

/// 目标数值的参考语义；未指定时不能默认解释为地面目标。
///
/// 当前仅供纯到达区域判断使用，尚未接入目标、运行流程或 wire。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalReference {
    /// 机体三维位置。
    Body,
    /// 地面 XY 与独立的层关系。
    Ground,
}

/// 冻结的原高度意图与后续解析参考；解析不得改写原 presence。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundHeightIntent {
    /// 原请求显式给出的地面高度；`None` 不等于显式零。
    pub requested_ground_z: Option<f64>,
    /// 后续解析出的参考，可仍未知；不作为层匹配证明。
    pub resolved_ground_z: Option<f64>,
}

/// 运行 owner 提供的层关系结论；枚举值本身不是实际执行证明。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroundLayerResult {
    /// owner 已判定层条件匹配；不要求此刻有新的观测。
    Matched,
    /// 层条件尚未解析或确认。
    Unresolved,
    /// 已知层条件冲突。
    KnownConflict,
}

/// 纯区域判断，不产生到达事件、运动许可或成功终态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GoalRegionAssessment {
    /// 数值区域与所需层条件满足；原执行、来源与停车资格仍由 owner 核对。
    WithinRegion,
    /// XY/XYZ 或 yaw 尚不在数值区域内。
    OutsideRegion,
    /// 已在数值区域内，但原显式高度约束尚未确认。
    TargetLayerUnresolved,
    /// 已在数值区域内，但层条件有已知冲突。
    KnownLayerConflict,
}

/// 保留不同阶段原有的正容差开闭边界，不是新的运行模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrivalBoundary {
    /// 误差严格小于正容差。
    Strict,
    /// 误差小于或等于正容差。
    Closed,
}
