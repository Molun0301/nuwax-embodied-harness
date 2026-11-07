//! 单层探索的实际行程和返航预算；路径距离由当前二维/三维安全查询提供。
use serde::{Deserialize, Serialize};

/// 单个源的持久历史，不携带瞬时目标、地图读者或运动租约。
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceCheckpoint {
    /// 生产源名称，恢复必须精确匹配。
    pub name: String,
    /// 源自己的版本化JSON载荷，最多一兆字节。
    pub payload: String,
}

/// 可选行程上限与明确保留量，不把距离换算为未标定的电量。
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReturnPolicy {
    /// 本次任务可使用的总路程；没有值时不限制累计路程。
    pub maximum_distance_m: Option<f64>,
    /// 在规划回程之外保留的行程。
    #[serde(default)]
    pub reserve_distance_m: f64,
    /// 自然结束后先到起点并停车。
    #[serde(default)]
    pub return_on_completion: bool,
}
impl ReturnPolicy {
    /// 只接纳有限、可解释的米制预算。
    pub fn validate(&self) -> Result<(), String> {
        if !self.reserve_distance_m.is_finite() || self.reserve_distance_m < 0.0 {
            return Err("返航保留路程必须有限且非负".into());
        }
        if let Some(maximum) = self.maximum_distance_m {
            if !maximum.is_finite() || maximum <= self.reserve_distance_m || maximum > 1_000_000.0 {
                return Err("总路程须大于保留路程且不超过一百万米".into());
            }
        } else if self.reserve_distance_m != 0.0 {
            return Err("返航保留路程需要明确总路程预算".into());
        }
        Ok(())
    }
    /// 是否需要冻结起点及检查返回路线。
    pub fn enabled(&self) -> bool {
        self.maximum_distance_m.is_some() || self.return_on_completion
    }
    /// 当前几何的实际路线必须包含绕障和方向限制，调用者不能传直线替代值。
    pub fn can_visit(&self, travelled_m: f64, outward_m: f64, homeward_m: f64) -> bool {
        if self.validate().is_err()
            || ![travelled_m, outward_m, homeward_m]
                .iter()
                .all(|v| v.is_finite() && *v >= 0.0)
        {
            return false;
        }
        self.maximum_distance_m.is_none_or(|maximum| {
            travelled_m + outward_m + homeward_m + self.reserve_distance_m <= maximum
        })
    }
    /// 接近回程所需余额时停止派发新探索目标。
    pub fn should_return(&self, travelled_m: f64, homeward_m: f64) -> Result<bool, String> {
        self.validate()?;
        if ![travelled_m, homeward_m]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.0)
        {
            return Err("返航需要当前地图中的有限可通行路线".into());
        }
        Ok(self
            .maximum_distance_m
            .is_some_and(|maximum| travelled_m + homeward_m + self.reserve_distance_m >= maximum))
    }
}

/// 可随准确地图资产保存的完整机体起点。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MissionPose {
    /// 地图中的机体基准，三维保留真实高度。
    pub position: [f64; 3],
    /// 机体朝向，顺序为x、y、z、w。
    pub orientation_xyzw: [f64; 4],
}
impl MissionPose {
    /// 校验有限坐标和单位四元数，不悄悄修正坏持久数据。
    pub fn validate(&self) -> Result<(), String> {
        let norm: f64 = self.orientation_xyzw.iter().map(|v| v * v).sum();
        if self
            .position
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 1e9)
            || self.orientation_xyzw.iter().any(|v| !v.is_finite())
            || (norm - 1.0).abs() > 1e-3
        {
            return Err("探索进度位姿无效".into());
        }
        Ok(())
    }
}

/// 只有同坐标代际内的真实采样推进路程；新进程恢复由资产拥有方重新绑定。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MissionProgress {
    /// 版本化载荷，当前只支持1。
    pub schema_version: u32,
    /// 原开始位置，停止或重启不自动改成当前位置。
    pub home: MissionPose,
    /// 已实际走过的累计三维路程。
    pub travelled_m: f64,
    /// 已决定返航时恢复继续返航，不再挑新前沿。
    pub returning: bool,
    /// 运行期采样不进入持久资产，避免重启间隙被算为行走。
    #[serde(skip)]
    last_sample: Option<(u64, [f64; 3])>,
}
impl MissionProgress {
    /// 第一张有效规范位姿冻结任务起点。
    pub fn new(home: MissionPose) -> Result<Self, String> {
        home.validate()?;
        Ok(Self {
            schema_version: 1,
            home,
            travelled_m: 0.0,
            returning: false,
            last_sample: None,
        })
    }
    /// 恢复前由上层确认地图资产与定位，此处验证载荷自身。
    pub fn validate(&self) -> Result<(), String> {
        self.home.validate()?;
        if self.schema_version != 1
            || !self.travelled_m.is_finite()
            || !(0.0..=1_000_000.0).contains(&self.travelled_m)
        {
            return Err("探索进度版本或累计路程无效".into());
        }
        Ok(())
    }
    /// 按源时间累加相邻位姿的距离，重复和倒序样本不增加行程。
    pub fn observe(
        &mut self,
        pose: &MissionPose,
        source_ns: u64,
    ) {
        if let Some((previous_ns, previous)) = self.last_sample {
            if source_ns <= previous_ns {
                return;
            }
            let distance = previous
                .iter()
                .zip(pose.position)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            self.travelled_m += distance;
        }
        self.last_sample = Some((source_ns, pose.position));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn pose(x: f64, z: f64) -> MissionPose {
        MissionPose {
            position: [x, 0.0, z],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
    #[test]
    fn return_budget_uses_both_directed_routes_and_reserve() {
        let policy = ReturnPolicy {
            maximum_distance_m: Some(10.0),
            reserve_distance_m: 1.0,
            return_on_completion: true,
        };
        assert!(policy.can_visit(2.0, 2.0, 4.0));
        assert!(!policy.can_visit(2.0, 2.0, 6.0));
        assert!(!policy.can_visit(2.0, 2.0, f64::INFINITY));
        assert!(policy.should_return(5.0, 4.0).unwrap());
        assert!(policy.should_return(5.0, f64::INFINITY).is_err());
    }
    #[test]
    fn adjacent_positions_count_height_and_jumps_but_not_replayed_samples() {
        let mut state = MissionProgress::new(pose(0.0, 0.0)).unwrap();
        state.observe(&pose(0.0, 0.0), 1_000_000_000);
        state.observe(&pose(0.0, 1.0), 2_000_000_000);
        state.observe(&pose(0.0, 5.0), 1_000_000_000);
        state.observe(&pose(0.0, 5.0), 2_000_000_000);
        assert_eq!(state.travelled_m, 1.0);
        state.observe(&pose(10.0, 1.0), 3_000_000_000);
        assert_eq!(state.travelled_m, 11.0);
        state.observe(&pose(11.0, 1.0), 4_000_000_000);
        assert_eq!(state.travelled_m, 12.0);
        assert_eq!(state.home, pose(0.0, 0.0));
        let mut restored: MissionProgress = serde_json::from_str(&serde_json::to_string(&state).unwrap()).unwrap();
        restored.observe(&pose(100.0, 0.0), 5_000_000_000);
        assert_eq!(restored.travelled_m, 12.0, "恢复后的首帧不累计离线间隔");
    }
}
