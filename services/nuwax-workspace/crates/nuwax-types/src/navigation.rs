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
