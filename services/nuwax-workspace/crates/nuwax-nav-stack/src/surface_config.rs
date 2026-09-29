//! 楼梯、斜坡规划与跟随共用的物理参数；机体到支撑面的标定不得猜测。

use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope};
use nuwax_nav_runtime::SurfaceFollowerConfig;
use nuwax_nav_types_messages::{MotionKinematics, MotionModel, NavigationCapabilities, RobotGeometry};

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
                    "pct-scan-3d requires calibrated NUWAX_NAV_BODY_TO_SUPPORT_Z_M".into(),
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
        config.follower.scan.horizon_m = configured_number(
            "NUWAX_NAV_LOCAL_HORIZON_M", config.follower.scan.horizon_m,
        )?;
        config.follower.scan.refresh_distance_m = configured_number("NUWAX_NAV_LOCAL_REFRESH_DISTANCE_M", 1.0)?;
        let iterations = configured_number("NUWAX_NAV_SPLINE_ITERATIONS", 12.0)?;
        if iterations.fract() != 0.0 || !(0.0..=100.0).contains(&iterations) {
            return Err(NavStackError::InvalidConfig("近场样条迭代次数需要0—100的整数".into()));
        }
        config.follower.scan.optimization_iterations = iterations as usize;
        config.follower.scan.validate().map_err(NavStackError::InvalidConfig)?;
        if let Ok(value) = std::env::var("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS") {
            if !value.trim().is_empty() {
                let limit = parse_number("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS", &value)?;
                if limit <= 0.0 {return Err(NavStackError::InvalidConfig("执行器线速度上限必须为正".into()));}
                // 设备声明给出巡航上限；旧控制器默认速度不能再次隐式截断。
                config.follower.controller.speed = config.follower.motion_limits.as_ref()
                    .map_or(limit,|motion|limit.min(motion.max_forward_mps));
            }
        }
        if let Ok(value) = std::env::var("_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS") {
            if !value.trim().is_empty() {
                let limit = parse_number("_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS", &value)?;
                if limit <= 0.0 {return Err(NavStackError::InvalidConfig("执行器角速度上限必须为正".into()));}
                config.follower.max_angular_speed_rps = limit;
                if let Some(limits) = config.follower.motion_limits.as_mut() {
                    limits.max_angular_rps = limits.max_angular_rps.min(limit);
                }
            }
        }
        if let Ok(model)=std::env::var("_NUWAX_NAV_ACTUATOR_MOTION_MODEL") {
            if !model.trim().is_empty() {
                if config.body_envelope.is_none() {
                    return Err(NavStackError::InvalidConfig("三维基本运动通道需要完整机器人外形".into()));
                }
                if model!="omnidirectional" {
                    return Err(NavStackError::InvalidConfig("不支持的执行器基本运动形式".into()));
                }
                let speed=std::env::var("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS")
                    .map_err(|_|NavStackError::InvalidConfig("基本运动通道缺少执行器线速度上限".into()))?;
                let angular=std::env::var("_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS")
                    .map_err(|_|NavStackError::InvalidConfig("基本运动通道缺少执行器角速度上限".into()))?;
                let speed=parse_number("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS",&speed)?;
                let motion=MotionKinematics {model:MotionModel::Omnidirectional,
                    max_forward_mps:speed,max_reverse_mps:speed,max_lateral_mps:speed,
                    max_angular_rps:parse_number("_NUWAX_NAV_ACTUATOR_MAX_YAW_RPS",&angular)?,
                    minimum_turning_radius_m:0.0,can_rotate_in_place:true};
                motion.validate().map_err(NavStackError::InvalidConfig)?;
                config.follower.basic_kinematics=Some(motion);
            }
        }
        tracing::info!(target:"navigation.config",event="navigation.config.loaded",profile="pct-scan-3d",
            voxel_m=config.geometry.voxel_size,local_horizon_m=config.follower.scan.horizon_m,
            local_refresh_m=config.follower.scan.refresh_distance_m,spline_iterations=config.follower.scan.optimization_iterations,
            "三维导航已读取统一配置");
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
        let mut follower=SurfaceFollowerConfig::new(body_to_support_z_m);
        // 三维生产调度为20赫兹，速度窗口的加减速时间必须采用同一控制周期。
        follower.controller.control_frequency=1.0/nuwax_nav_runtime::DEFAULT_TICK_INTERVAL.as_secs_f64();
        Ok(Self {
            geometry,
            body_envelope: None,
            follower,
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
            self.follower.controller.speed = caps.motion.max_forward_mps;
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
            // 支撑图提取独立地面层，只要求一个体素的空隙。完整宽高由
            // 每个位姿和扫掠的机体检查承担，避免侧面/高处回波抹掉整列地面。
            self.geometry.wall_clearance_m = 0.0;
            // 软余量从机体最远水平轮廓外开始生效；重复装配先取回原余量，避免逐次累加。
            if self.geometry.wall_buffer_m>0.0 && self.geometry.wall_buffer_weight>0.0 {
                let previous_radius=self.body_envelope.as_ref().map_or(0.0,SurfaceBodyEnvelope::radius);
                let padding=(f64::from(self.geometry.wall_buffer_m)-previous_radius).max(0.0);
                self.geometry.wall_buffer_m=(envelope.radius()+padding) as f32;
            }
            self.geometry.robot_height = self.geometry.voxel_size;
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
    fn actuator_speed_is_not_capped_by_the_legacy_controller_default() {
        const CHILD: &str = "NUWAX_TEST_SURFACE_SPEED_CHILD";
        if let Ok(expected)=std::env::var(CHILD) {
            let config=SurfaceNavigationConfig::from_env().unwrap();
            assert_eq!(config.follower.controller.speed,expected.parse::<f64>().unwrap());
            return;
        }
        for speed in ["1.0","0.35"] {
            let result=std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact","surface_config::tests::actuator_speed_is_not_capped_by_the_legacy_controller_default","--nocapture"])
                .env_clear().env(CHILD,speed).env("NUWAX_NAV_BODY_TO_SUPPORT_Z_M","0.4")
                .env("_NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS",speed).output().unwrap();
            assert!(result.status.success(),"生产环境解析未消费设备速度：{}",String::from_utf8_lossy(&result.stderr));
        }
    }

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
