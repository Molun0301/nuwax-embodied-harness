//! 楼梯、斜坡规划与跟随共用的物理参数；机体到支撑面的标定不得猜测。

use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope};
use nuwax_nav_runtime::SurfaceFollowerConfig;
use nuwax_nav_types_messages::{NavigationCapabilities, RobotGeometry};

use crate::{NavStackError, NavStackResult};

/// 一次启动捕获并同时传给规划器和控制器的三维导航参数。
#[derive(Clone)]
pub struct SurfaceNavigationConfig {
    /// 支撑面、净空、台阶和坡道约束。
    pub geometry: Config,
    /// 完整机体包络，缺失表示尚无外形标定。
    pub body_envelope: Option<SurfaceBodyEnvelope>,
    /// 机体参考位姿与受控速度参数。
    pub follower: SurfaceFollowerConfig,
    /// 实际机器人宽度，单位米。
    pub robot_width_m: f64,
    /// 当前后台搜索的取消信号；不进入机器人标定或地图身份。
    pub(crate) planning_cancelled: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
}

impl SurfaceNavigationConfig {
    /// 从现有设置加载器提供的环境读取，缺少支撑面标定时明确拒绝。
    pub fn from_env() -> NavStackResult<Self> {
        let offset = std::env::var("NUWAX_NAV_BODY_TO_SUPPORT_Z_M")
            .ok()
            .filter(|value| !value.trim().is_empty())
            .ok_or_else(|| {
                NavStackError::InvalidConfig(
                    "mls-3d requires calibrated NUWAX_NAV_BODY_TO_SUPPORT_Z_M".into(),
                )
            })?;
        let offset = parse_number("NUWAX_NAV_BODY_TO_SUPPORT_Z_M", &offset)?;
        let width = configured_number("NUWAX_NAV_ROBOT_WIDTH_M", 0.3)?;
        let height = configured_number("NUWAX_NAV_ROBOT_HEIGHT_M", 0.6)?;
        let voxel = configured_number("NUWAX_NAV_SURFACE_VOXEL_SIZE_M", 0.1)?;
        let max_step = configured_number("NUWAX_GEOMETRY_TRAVERSABILITY_MAX_STEP_HEIGHT_M", 0.15)?;
        let max_slope = configured_number("NUWAX_GEOMETRY_TRAVERSABILITY_MAX_SLOPE_DEG", 18.0)?;
        let tread = configured_number("NUWAX_NAV_MIN_STAIR_TREAD_M", 0.2)?;
        let frame = std::env::var("NUWAX_GEOMETRY_MAP_FRAME").unwrap_or_else(|_| "map".into());
        let mut config = Self::new(
            frame, offset, width, height, voxel, max_step, max_slope, tread,
        )?;
        let capabilities =
            NavigationCapabilities::from_env().map_err(NavStackError::InvalidConfig)?;
        let body = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON")
            .ok()
            .filter(|v| !v.trim().is_empty())
            .map(|v| RobotGeometry::from_runtime_json(&v))
            .transpose()
            .map_err(NavStackError::InvalidConfig)?;
        config.apply_calibration(body, capabilities)?;
        Ok(config)
    }

    /// 显式构造便于部署校验和无硬件回归，不读取或修改进程环境。
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        frame: String,
        body_to_support_z_m: f64,
        robot_width_m: f64,
        robot_height_m: f64,
        voxel_size_m: f64,
        max_step_height_m: f64,
        max_slope_deg: f64,
        min_stair_tread_m: f64,
    ) -> NavStackResult<Self> {
        if frame != "map"
            || [
                body_to_support_z_m,
                robot_width_m,
                robot_height_m,
                voxel_size_m,
                min_stair_tread_m,
            ]
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
            || !max_step_height_m.is_finite()
            || max_step_height_m < 0.0
            || !max_slope_deg.is_finite()
            || !(0.0..90.0).contains(&max_slope_deg)
        {
            return Err(NavStackError::InvalidConfig(
                "invalid calibrated surface navigation geometry".into(),
            ));
        }
        let geometry = Config {
            world_frame: frame,
            voxel_size: voxel_size_m as f32,
            robot_height: robot_height_m as f32,
            // 生产支撑面不能靠形态学补出原本没有观测证据的地面。
            surface_closing_radius: 0.0,
            node_spacing_m: (voxel_size_m * 3.0) as f32,
            wall_clearance_m: (robot_width_m * 0.5) as f32,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            step_threshold_m: max_step_height_m as f32,
            step_penalty_weight: 2.0,
            max_slope_deg: max_slope_deg as f32,
            min_step_tread_depth_m: min_stair_tread_m as f32,
            goal_tolerance: 0.2,
            viz_publish_hz: 0.0,
        };
        if !(body_to_support_z_m as f32).is_normal()
            || !geometry.voxel_size.is_normal()
            || !geometry.robot_height.is_normal()
            || !geometry.wall_clearance_m.is_normal()
            || !geometry.min_step_tread_depth_m.is_normal()
            || !geometry.step_threshold_m.is_finite()
            || !geometry.node_spacing_m.is_normal()
            || geometry.min_step_tread_depth_m < geometry.voxel_size * 2.0
        {
            return Err(NavStackError::InvalidConfig(
                "surface geometry exceeds float32 domain or tread is below two voxels".into(),
            ));
        }
        Ok(Self {
            geometry,
            body_envelope: None,
            follower: SurfaceFollowerConfig::new(body_to_support_z_m),
            robot_width_m,
            planning_cancelled: None,
        })
    }
    /// 两端用同一已验证标定物化外形和运动约束。
    pub fn apply_calibration(
        &mut self,
        body: Option<RobotGeometry>,
        capabilities: Option<NavigationCapabilities>,
    ) -> NavStackResult<()> {
        if capabilities.is_some() && body.is_none() {return Err(NavStackError::InvalidConfig("三维运动标定需要完整机器人外形".into()));}
        if let Some(caps) = &capabilities {
            caps.validate().map_err(NavStackError::InvalidConfig)?;
            self.follower.motion_limits = Some(caps.motion.clone());
            self.follower.controller.speed = self
                .follower
                .controller
                .speed
                .min(caps.motion.max_forward_mps);
            if let Some(terrain) = &caps.terrain {
                self.geometry.step_threshold_m = if terrain.stairs_supported {
                    terrain.max_step_up_m.max(terrain.max_step_down_m) as f32
                } else {
                    0.0
                };
                self.geometry.max_slope_deg = terrain.max_slope_deg as f32;
                self.geometry.min_step_tread_depth_m = terrain
                    .min_tread_depth_m
                    .max(f64::from(self.geometry.voxel_size) * 2.0)
                    as f32;
                self.follower.max_observation_age_sec = Some(terrain.max_observation_age_sec);
            } else {
                // 未声明地形执行能力时，不接纳离散阶跃。
                self.geometry.step_threshold_m = 0.0;
            }
        }
        if let Some(body) = body {
            body.validate().map_err(NavStackError::InvalidConfig)?;
            let terrain = capabilities.as_ref().and_then(|c| c.terrain.as_ref());
            let envelope = SurfaceBodyEnvelope {
                footprint_xy_m: body.footprint_xy_m,
                z_bounds_m: [body.z_min_m, body.z_max_m],
                body_to_support_z_m: self.follower.body_to_support_z_m,
                max_step_up_m: terrain.map_or(f64::from(self.geometry.step_threshold_m), |t| {
                    if t.stairs_supported {
                        t.max_step_up_m
                    } else {
                        0.0
                    }
                }),
                max_step_down_m: terrain.map_or(f64::from(self.geometry.step_threshold_m), |t| {
                    if t.stairs_supported {
                        t.max_step_down_m
                    } else {
                        0.0
                    }
                }),
                max_cross_slope_deg: terrain.map_or(f64::from(self.geometry.max_slope_deg), |t| {
                    t.max_cross_slope_deg
                }),
                support_uncertainty_m: terrain.map_or(0.0, |t| t.support_height_uncertainty_m),
            };
            envelope
                .validate()
                .map_err(|e| NavStackError::InvalidConfig(e.to_string()))?;
            // 完整机体在每个位姿和扫掠段执行碰撞检查。支撑图不再先按
            // 通用宽度膨胀一次，否则逐列机体检查会重复扣除通道净空。
            self.geometry.wall_clearance_m = 0.0;
            self.geometry.robot_height = (envelope.body_to_support_z_m + body.z_max_m) as f32;
            self.body_envelope = Some(envelope);
        }
        Ok(())
    }
}

fn configured_number(key: &str, default: f64) -> NavStackResult<f64> {
    match std::env::var(key) {
        Ok(value) if !value.trim().is_empty() => parse_number(key, &value),
        _ => Ok(default),
    }
}

fn parse_number(key: &str, value: &str) -> NavStackResult<f64> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| NavStackError::InvalidConfig(format!("{key} must be a finite number")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_and_calibration_are_validated_before_starting_either_owner() {
        for (frame, offset, voxel, tread, step) in [
            ("map", 0.0, 0.1, 0.2, 0.2),
            ("odom", 0.4, 0.1, 0.2, 0.2),
            ("map", 0.4, 0.1, 0.1, 0.2),
            ("map", 1e300, 0.1, 0.2, 0.2),
            ("map", 0.4, 0.1, 0.2, 1e300),
        ] {
            assert!(SurfaceNavigationConfig::new(
                frame.into(),
                offset,
                0.3,
                0.6,
                voxel,
                step,
                35.0,
                tread
            )
            .is_err());
        }
        let valid =
            SurfaceNavigationConfig::new("map".into(), 0.4, 0.3, 0.6, 0.1, 0.2, 35.0, 0.2).unwrap();
        assert_eq!(valid.geometry.wall_clearance_m, 0.15);
        assert_eq!(valid.follower.body_to_support_z_m, 0.4);
        assert_eq!(valid.geometry.surface_closing_radius, 0.0);
    }
}
