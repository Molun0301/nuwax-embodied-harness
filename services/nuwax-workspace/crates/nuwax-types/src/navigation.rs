//! 封存导航目标的执行选项；与通信和具体规划器解耦。
use serde::{Deserialize, Serialize};

/// 必须随目标摘要传递的执行约束。
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
    fn default()->Self {Self {allow_reverse:false,position_tolerance_m:0.2,yaw_tolerance_rad:0.35}}
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
