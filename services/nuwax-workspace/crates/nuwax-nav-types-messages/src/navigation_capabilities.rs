//! 导航与探索共用的显式硬件标定，不以缺省值推断高级能力。
use serde::{Deserialize, Serialize};

/// 运动学类型。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionModel {
    /// 差速或等效机体速度控制。
    Differential,
    /// 可受控横移的全向底盘。
    Omnidirectional,
    /// 有最小曲率半径的阿克曼底盘。
    Ackermann,
}
/// 执行器已声明的平面速度通道；不包含未经测量的响应或制动保证。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionKinematics {
    /// 可执行的运动形式。
    pub model: MotionModel,
    /// 前进速度上限，米/秒。
    pub max_forward_mps: f64,
    /// 倒退速度上限，零表示不支持。
    pub max_reverse_mps: f64,
    /// 横移速度上限，零表示不支持。
    pub max_lateral_mps: f64,
    /// 角速度上限，弧度/秒。
    pub max_angular_rps: f64,
    /// 最小转弯半径，米。
    pub minimum_turning_radius_m: f64,
    /// 执行器是否支持原地转向。
    pub can_rotate_in_place: bool,
}

impl MotionKinematics {
    /// 仅验证通道与速度范围，不能据此启用保证制动控制。
    pub fn validate(&self) -> Result<(), String> {
        if !range(self.max_forward_mps, 0.0, 5.0, true)
            || !range(self.max_reverse_mps, 0.0, 5.0, false)
            || !range(self.max_lateral_mps, 0.0, 5.0, false)
            || !range(self.max_angular_rps, 0.0, 4.0 * std::f64::consts::PI, true)
            || !range(self.minimum_turning_radius_m, 0.0, 100.0, false)
            || (self.model != MotionModel::Omnidirectional && self.max_lateral_mps != 0.0)
            || (self.model == MotionModel::Ackermann
                && (self.can_rotate_in_place || self.minimum_turning_radius_m <= 0.0))
            || (self.can_rotate_in_place && self.minimum_turning_radius_m != 0.0)
        {
            return Err("执行器平面运动通道或速度上限无效".into());
        }
        Ok(())
    }
}

/// 同一速度执行器的速度、响应和保证制动能力，均采用国际单位。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionLimits {
    /// 运动学模型。
    pub model: MotionModel,
    /// 最大前进速度。
    pub max_forward_mps: f64,
    /// 最大倒车速度，零表示不支持。
    pub max_reverse_mps: f64,
    /// 最大横移速度，零表示不支持。
    pub max_lateral_mps: f64,
    /// 最大角速度。
    pub max_angular_rps: f64,
    /// 最大平移加速度。
    pub linear_acceleration_mps2: f64,
    /// 实测可保证的平移减速度。
    pub guaranteed_braking_mps2: f64,
    /// 最大角加速度。
    pub angular_acceleration_rps2: f64,
    /// 实测可保证的角减速度。
    pub guaranteed_angular_braking_rps2: f64,
    /// 从观察至制动生效的保守延迟。
    pub reaction_time_sec: f64,
    /// 最小转弯半径，原地转向平台为零。
    pub minimum_turning_radius_m: f64,
    /// 是否经过原地转向能力确认。
    pub can_rotate_in_place: bool,
}
/// 激光观察模型，传感器坐标正X为观察中心，正Z向上。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationModel {
    /// `planar`为二维扫描，`volume`为三维视域。
    pub kind: ObservationKind,
    /// 最小有效观测距离。
    pub min_range_m: f64,
    /// 最大有效观测距离。
    pub max_range_m: f64,
    /// 水平视场角，弧度。
    pub horizontal_fov_rad: f64,
    /// 垂直最低仰角，弧度。
    pub vertical_min_rad: f64,
    /// 垂直最高仰角，弧度。
    pub vertical_max_rad: f64,
    /// 传感器原点在机体坐标中的位置。
    pub sensor_translation_body_m: [f64; 3],
    /// 传感器坐标到机体坐标的单位旋转。
    pub sensor_rotation_body_xyzw: [f64; 4],
    /// 单次收益评估射线预算。
    pub max_rays: usize,
}
/// 观察模型维度。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationKind {
    /// 二维平面扫描。
    Planar,
    /// 三维视域。
    Volume,
}
/// 与步态执行能力匹配的地形约束。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerrainLimits {
    /// 最大上阶高度。
    pub max_step_up_m: f64,
    /// 最大下阶高度。
    pub max_step_down_m: f64,
    /// 最小踏面深度。
    pub min_tread_depth_m: f64,
    /// 最大纵坡。
    pub max_slope_deg: f64,
    /// 最大横坡。
    pub max_cross_slope_deg: f64,
    /// 支撑高度的保守误差界。
    pub support_height_uncertainty_m: f64,
    /// 执行近场观测的最长年龄。
    pub max_observation_age_sec: f64,
    /// 是否有已确认的上下阶执行能力。
    pub stairs_supported: bool,
    /// 可选的实际步态模式；不配置时不推断厂商模式。
    pub required_locomotion_mode: Option<String>,
    /// 守卫启动时模式切换的反馈等待上限。
    #[serde(default = "default_mode_transition_timeout")]
    pub mode_transition_timeout_sec: f64,
}
fn default_mode_transition_timeout()->f64 {5.0}
/// 同一标定版本，供两条生产链共同捕获。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NavigationCapabilities {
    /// 当前固定为1。
    pub schema_version: u32,
    /// 实际标定或明确仿真标定的标识。
    pub calibration_id: String,
    /// 运动限制。
    pub motion: MotionLimits,
    /// 缺省表示没有可用的传感器视域标定。
    pub observation: Option<ObservationModel>,
    /// 缺省表示没有可用的高级地形标定。
    pub terrain: Option<TerrainLimits>,
}
fn range(value: f64, min: f64, max: f64, positive: bool) -> bool {
    value.is_finite() && value >= min && value <= max && (!positive || value > 0.0)
}
impl MotionLimits {
    /// 从完整标定提取同一执行器的速度通道。
    pub fn kinematics(&self) -> MotionKinematics {
        MotionKinematics {
            model: self.model,
            max_forward_mps: self.max_forward_mps,
            max_reverse_mps: self.max_reverse_mps,
            max_lateral_mps: self.max_lateral_mps,
            max_angular_rps: self.max_angular_rps,
            minimum_turning_radius_m: self.minimum_turning_radius_m,
            can_rotate_in_place: self.can_rotate_in_place,
        }
    }
    /// 拒绝非有限、不成立或超出支持范围的标定。
    pub fn validate(&self) -> Result<(), String> {
        if self.maximum_stopping_time_sec() > 30.0 || !range(self.max_forward_mps, 0.0, 5.0, true)
            || !range(self.max_reverse_mps, 0.0, 5.0, false)
            || !range(self.max_lateral_mps, 0.0, 5.0, false)
            || !range(self.max_angular_rps, 0.0, 4.0 * std::f64::consts::PI, true)
            || !range(self.linear_acceleration_mps2, 0.0, 50.0, true)
            || !range(self.guaranteed_braking_mps2, 0.0, 50.0, true)
            || !range(self.angular_acceleration_rps2, 0.0, 100.0, true)
            || !range(self.guaranteed_angular_braking_rps2, 0.0, 100.0, true)
            || !range(self.reaction_time_sec, 0.0, 2.0, false)
            || !range(self.minimum_turning_radius_m, 0.0, 100.0, false)
            || (self.model != MotionModel::Omnidirectional && self.max_lateral_mps != 0.0)
            || (self.model == MotionModel::Ackermann
                && (self.can_rotate_in_place || self.minimum_turning_radius_m <= 0.0))
            || (self.can_rotate_in_place && self.minimum_turning_radius_m != 0.0)
        {
            return Err("导航运动能力标定无效".into());
        }
        Ok(())
    }
    /// 全部平移/转动速度上界对应的最迟停稳时间，包含反应延迟。
    pub fn maximum_stopping_time_sec(&self)->f64 {
        self.reaction_time_sec + (self.max_forward_mps.max(self.max_reverse_mps).hypot(self.max_lateral_mps)/self.guaranteed_braking_mps2)
            .max(self.max_angular_rps/self.guaranteed_angular_braking_rps2)
    }
    /// 包含响应延迟的保守平移停车距离。
    pub fn stopping_distance(&self, speed: f64) -> f64 {
        speed.abs() * self.reaction_time_sec + speed.powi(2) / (2.0 * self.guaranteed_braking_mps2)
    }
}
impl ObservationModel {
    /// 校验视域与传感器外参。
    pub fn validate(&self) -> Result<(), String> {
        let pi = std::f64::consts::PI;
        if !range(self.min_range_m, 0.0, 200.0, false)
            || !range(self.max_range_m, 0.0, 200.0, true)
            || self.min_range_m >= self.max_range_m
            || !range(self.horizontal_fov_rad, 0.0, 2.0 * pi, true)
            || !range(self.vertical_min_rad, -pi / 2.0, pi / 2.0, false)
            || !range(self.vertical_max_rad, -pi / 2.0, pi / 2.0, false)
            || self.vertical_min_rad > self.vertical_max_rad
            || (self.kind == ObservationKind::Planar
                && (self.vertical_min_rad != 0.0 || self.vertical_max_rad != 0.0))
            || self
                .sensor_translation_body_m
                .iter()
                .any(|&v| !range(v, -100.0, 100.0, false))
            || self
                .sensor_rotation_body_xyzw
                .iter()
                .any(|v| !v.is_finite())
            || (self
                .sensor_rotation_body_xyzw
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                - 1.0)
                .abs()
                > 1e-6
            || !(16..=8192).contains(&self.max_rays)
        {
            return Err("传感器视域标定无效".into());
        }
        Ok(())
    }
}
impl TerrainLimits {
    /// 校验地形限制，不自动推断上下阶支持。
    pub fn validate(&self) -> Result<(), String> {
        if self.required_locomotion_mode.as_ref().is_some_and(|mode|mode.is_empty() || mode.chars().count()>128 || mode.trim()!=mode || mode.chars().any(char::is_control))
            || !range(self.mode_transition_timeout_sec,0.0,30.0,true)
            || !range(self.max_step_up_m, 0.0, 1.0, false)
            || !range(self.max_step_down_m, 0.0, 1.0, false)
            || !range(self.min_tread_depth_m, 0.0, 5.0, true)
            || !range(self.max_slope_deg, 0.0, 90.0, false)
            || self.max_slope_deg >= 90.0
            || !range(self.max_cross_slope_deg, 0.0, 90.0, false)
            || self.max_cross_slope_deg >= 90.0
            || !range(self.support_height_uncertainty_m, 0.0, 1.0, false)
            || !range(self.max_observation_age_sec, 0.0, 30.0, true)
            || (!self.stairs_supported
                && (self.max_step_up_m != 0.0 || self.max_step_down_m != 0.0))
        {
            return Err("地形通行能力标定无效".into());
        }
        Ok(())
    }
}
impl NavigationCapabilities {
    /// 从启动环境捕获；空值明确表示无标定。
    pub fn from_env() -> Result<Option<Self>, String> {
        let value = std::env::var("NUWAX_NAV_CAPABILITIES_JSON").unwrap_or_default();
        if value.trim().is_empty() {
            return Ok(None);
        }
        Self::from_json(&value).map(Some)
    }
    /// 读取跨语言统一结构并完整校验。
    pub fn from_json(value: &str) -> Result<Self, String> {
        let result: Self =
            serde_json::from_str(value).map_err(|e| format!("导航能力标定格式错误：{e}"))?;
        result.validate()?;
        Ok(result)
    }
    /// 校验版本、标定身份和所有已声明能力。
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1
            || self.calibration_id.trim() != self.calibration_id
            || self.calibration_id.is_empty()
            || self.calibration_id.chars().count() > 128
            || self.calibration_id.chars().any(char::is_control)
        {
            return Err("导航能力标定版本或标识无效".into());
        }
        self.motion.validate()?;
        if let Some(model) = &self.observation {
            model.validate()?;
        }
        if let Some(limits) = &self.terrain {
            limits.validate()?;
        }
        Ok(())
    }
}

