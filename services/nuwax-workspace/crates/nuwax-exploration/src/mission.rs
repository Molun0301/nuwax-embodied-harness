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
    /// 运行期采样不进入持久资产，避免重启间隙或新坐标代际被算为行走。
    #[serde(skip)]
    last_sample: Option<(u64, u64, [f64; 3])>,
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
    /// 实际采样按源序推进；过期重复不增加行程，坐标跳变需要上层先重新对账。
    pub fn observe(
        &mut self,
        pose: &MissionPose,
        source_ns: u64,
        frame_generation: u64,
        max_speed_mps: f64,
    ) -> Result<(), String> {
        self.validate()?;
        pose.validate()?;
        if source_ns == 0
            || frame_generation == 0
            || !max_speed_mps.is_finite()
            || max_speed_mps <= 0.0
        {
            return Err("探索行程缺少有效来源、代际或速度上界".into());
        }
        if let Some((previous_ns, previous_frame, previous)) = self.last_sample {
            if previous_frame != frame_generation {
                return Err("探索坐标代际变化，须核对起点和进度后恢复".into());
            }
            if source_ns <= previous_ns {
                return Ok(());
            }
            let distance = previous
                .iter()
                .zip(pose.position)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            // 一厘米只吸收位置量化；实际速度上界仍来自明确标定。
            if distance > max_speed_mps * (source_ns - previous_ns) as f64 * 1e-9 + 0.01 {
                return Err("规范位姿变化超过标定运动上界，不能记为实际行程".into());
            }
            if self.travelled_m + distance > 1_000_000.0 {
                return Err("累计探索行程超出预算".into());
            }
            self.travelled_m += distance;
        }
        self.last_sample = Some((source_ns, frame_generation, pose.position));
        Ok(())
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
    fn actual_height_counts_but_replayed_pose_and_frame_jump_do_not() {
        let mut state = MissionProgress::new(pose(0.0, 0.0)).unwrap();
        state
            .observe(&pose(0.0, 0.0), 1_000_000_000, 1, 2.0)
            .unwrap();
        state
            .observe(&pose(0.0, 1.0), 2_000_000_000, 1, 2.0)
            .unwrap();
        state
            .observe(&pose(0.0, 5.0), 1_000_000_000, 1, 2.0)
            .unwrap();
        assert_eq!(state.travelled_m, 1.0);
        assert!(state
            .observe(&pose(10.0, 1.0), 3_000_000_000, 2, 2.0)
            .is_err());
        assert!(state
            .observe(&pose(10.0, 1.0), 3_000_000_000, 1, 2.0)
            .is_err());
        assert_eq!(state.travelled_m, 1.0);
        state
            .observe(&pose(1.0, 1.0), 3_000_000_000, 1, 2.0)
            .unwrap();
        assert_eq!(state.travelled_m, 2.0);
        assert_eq!(state.home, pose(0.0, 0.0));
    }
}
