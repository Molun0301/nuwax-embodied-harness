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
    /// Owner-local work budget for this attempt; never part of calibration/map identity.
    pub(crate) planning_search_budget: Option<usize>,
    /// This attempt consumes this exact PCT cost snapshot; no second mutex read.
    pub(crate) planning_penalties: Option<std::sync::Arc<Vec<nuwax_nav_pct::RoutePenalty>>>,
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
        config.apply_calibration(body, capabilities.clone())?;
        config.apply_step_overrides(
            optional_step("NUWAX_NAV_MAX_STEP_UP_M")?,
            optional_step("NUWAX_NAV_MAX_STEP_DOWN_M")?,
            capabilities.as_ref(),
        )?;
        config.follower.local_planning_enabled = configured_bool("NUWAX_NAV_LOCAL_PLANNING_ENABLED", true)?;
        config.follower.scan.horizon_m = configured_number(
            "NUWAX_NAV_LOCAL_HORIZON_M", config.follower.scan.horizon_m,
        )?;
        config.follower.scan.refresh_distance_m = configured_number("NUWAX_NAV_LOCAL_REFRESH_DISTANCE_M", 1.0)?;
        let iterations = configured_number("NUWAX_NAV_SPLINE_ITERATIONS", 12.0)?;
        if iterations.fract() != 0.0 || !(0.0..=100.0).contains(&iterations) {
            return Err(NavStackError::InvalidConfig("近场样条迭代次数需要0—100的整数".into()));
        }
        config.follower.scan.optimization_iterations = iterations as usize;
        let expansion_limit = configured_number(
            "NUWAX_NAV_LOCAL_SEARCH_EXPANSIONS",
            config.follower.scan.search_expansion_limit as f64,
        )?;
        if expansion_limit.fract() != 0.0 || expansion_limit < 1.0 || expansion_limit > 1_000_000.0 {
            return Err(NavStackError::InvalidConfig(
                "近场搜索展开上限需要1—1000000的整数".into(),
            ));
        }
        config.follower.scan.search_expansion_limit = expansion_limit as usize;
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
            local_planning_enabled=config.follower.local_planning_enabled,
            voxel_m=config.geometry.voxel_size,local_horizon_m=config.follower.scan.horizon_m,
            max_step_up_m=config.body_envelope.as_ref().map(|body|body.max_step_up_m),
            max_step_down_m=config.body_envelope.as_ref().map(|body|body.max_step_down_m),
            local_refresh_m=config.follower.scan.refresh_distance_m,spline_iterations=config.follower.scan.optimization_iterations,
            "三维导航已读取统一配置");
        Ok(config)
    }

    /// 简单设置与完整能力标定共用一份机体参数，不能在两端静默取不同限制。
    fn apply_step_overrides(
        &mut self,
        up: Option<f64>,
        down: Option<f64>,
        capabilities: Option<&NavigationCapabilities>,
    ) -> NavStackResult<()> {
        if up.is_none() && down.is_none() {
            return Ok(());
        }
        let fallback = f64::from(self.geometry.step_threshold_m);
        let up = up.unwrap_or_else(|| self.body_envelope.as_ref().map_or(fallback, |body| body.max_step_up_m));
        let down = down.unwrap_or_else(|| self.body_envelope.as_ref().map_or(fallback, |body| body.max_step_down_m));
        if ![up, down].into_iter().all(|v| v.is_finite() && (0.0..=1.0).contains(&v)) {
            return Err(NavStackError::InvalidConfig("上、下台阶高度必须是0—1米的有限数值".into()));
        }
        if let Some(caps) = capabilities {
            let limits = caps.terrain.as_ref().filter(|terrain| terrain.stairs_supported)
                .map_or([0.0, 0.0], |terrain| [terrain.max_step_up_m, terrain.max_step_down_m]);
            if up > limits[0] + 1e-9 || down > limits[1] + 1e-9 {
                return Err(NavStackError::InvalidConfig(
                    "台阶设置超过NUWAX_NAV_CAPABILITIES_JSON中声明的地形能力，请同步机器人能力标定".into(),
                ));
            }
        }
        if let Some(body) = self.body_envelope.as_mut() {
            body.max_step_up_m = up;
            body.max_step_down_m = down;
            body.validate().map_err(|e| NavStackError::InvalidConfig(e.to_string()))?;
        } else if (up - down).abs() > 1e-9 {
            return Err(NavStackError::InvalidConfig("分别设置上、下台阶能力需要完整机器人外形".into()));
        }
        self.geometry.step_threshold_m = up.max(down) as f32;
        Ok(())
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
            planning_search_budget: None,
            planning_penalties: None,
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
                fixed_collision: body.self_collision,
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

fn optional_step(key: &str) -> NavStackResult<Option<f64>> {
    match std::env::var(key) {
        Ok(value) if !value.trim().is_empty() => {
            let value = parse_number(key, &value)?;
            if !(0.0..=1.0).contains(&value) {
                return Err(NavStackError::InvalidConfig(format!("{key}需要0—1米的有限数值")));
            }
            Ok(Some(value))
        }
        _ => Ok(None),
    }
}

fn configured_bool(key: &str, default: bool) -> NavStackResult<bool> {
    match std::env::var(key) {
        Ok(value) => match value.trim().to_ascii_lowercase().as_str() {
            "" => Ok(default),
            "true" | "1" | "yes" | "on" => Ok(true),
            "false" | "0" | "no" | "off" => Ok(false),
            _ => Err(NavStackError::InvalidConfig(format!("{key} must be a boolean"))),
        },
        Err(std::env::VarError::NotPresent) => Ok(default),
        Err(_) => Err(NavStackError::InvalidConfig(format!("{key} must be valid Unicode"))),
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
    fn runtime_fixed_collision_json_reaches_surface_envelope_without_copying() {
        let body = RobotGeometry::from_runtime_json(r#"{
            "footprint_xy_m":[[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
            "z_min_m":-0.448,"z_max_m":0.057,"source":"robot_default_urdf_collision",
            "self_collision":{"model_base_link":"trunk","excluded_moving_collision_count":12,
                "unsupported_collision_count":2,"primitives":[
                {"link_name":"trunk","shape":"box","translation_m":[0.012,0,-0.004],
                    "rotation_row_major":[0,-1,0,1,0,0,0,0,1],"size_m":[0.4,0.15,0.1]},
                {"link_name":"sensor","shape":"sphere","translation_m":[0.2,0,0.05],
                    "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.03},
                {"link_name":"fixed_mount","shape":"cylinder","translation_m":[-0.1,0,0.02],
                    "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.02,"length_m":0.08}]}}
        "#).unwrap();
        let shared = body.self_collision.as_ref().unwrap().clone();
        let mut config = SurfaceNavigationConfig::new("map".into(),0.4,0.3,0.6,0.1,0.15,18.0,0.2).unwrap();
        config.apply_calibration(Some(body),None).unwrap();
        let envelope = config.body_envelope.as_ref().unwrap();
        let fixed = envelope.fixed_collision.as_ref().unwrap();
        assert!(std::sync::Arc::ptr_eq(&shared,fixed));
        assert!(std::sync::Arc::ptr_eq(fixed,config.clone().body_envelope.as_ref().unwrap().fixed_collision.as_ref().unwrap()));
        assert_eq!(fixed.model_base_link,"trunk");
        assert_eq!(fixed.excluded_moving_collision_count,12);
        assert_eq!(fixed.unsupported_collision_count,2);
        assert_eq!(fixed.primitives.len(),3);
        assert_eq!(fixed.primitives[0].rotation_row_major,[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]);
        assert_eq!(envelope.z_bounds_m,[-0.448,0.057],"导航外包仍保留完整标定");
        envelope.validate().unwrap();
        let mut invalid = envelope.clone();
        std::sync::Arc::make_mut(invalid.fixed_collision.as_mut().unwrap())
            .primitives[0].translation_m[0]=f64::INFINITY;
        assert!(invalid.validate().is_err(),"直接设置envelope也须校验几何有限性");
        assert!(shared.primitives[0].translation_m[0].is_finite());
    }

    #[test]
    fn missing_fixed_collision_json_keeps_surface_compatibility_absent() {
        let body = RobotGeometry::from_runtime_json(r#"{"footprint_xy_m":[[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
            "z_min_m":-0.448,"z_max_m":0.057,"source":"manual"}"#).unwrap();
        let mut config = SurfaceNavigationConfig::new("map".into(),0.4,0.3,0.6,0.1,0.15,18.0,0.2).unwrap();
        config.apply_calibration(Some(body),None).unwrap();
        assert!(config.body_envelope.unwrap().fixed_collision.is_none());
    }

    fn isolated_loader_process() -> std::process::Command {
        let mut command = std::process::Command::new(std::env::current_exe().unwrap());
        command.env_clear();
        // Keep only the dynamic-loader search path required by relocatable SDKs;
        // navigation settings remain isolated from the parent test environment.
        for key in ["LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH"] {
            if let Some(value) = std::env::var_os(key) { command.env(key,value); }
        }
        command
    }

    #[test]
    fn local_planning_boolean_uses_the_actual_environment_loader() {
        const CHILD: &str = "NUWAX_TEST_LOCAL_PLANNING_BOOL";
        if let Ok(expected) = std::env::var(CHILD) {
            let result = SurfaceNavigationConfig::from_env();
            if expected == "invalid" { assert!(result.is_err()); }
            else { assert_eq!(result.unwrap().follower.local_planning_enabled, expected == "true"); }
            return;
        }
        for (value, expected) in [(None, "true"), (Some(""), "true"), (Some("  "), "true"),
            (Some("true"), "true"), (Some("1"), "true"), (Some(" YES "), "true"), (Some("on"), "true"),
            (Some("false"), "false"), (Some("0"), "false"), (Some(" No "), "false"), (Some("OFF"), "false"),
            (Some("enabled"), "invalid"), (Some("2"), "invalid")] {
            let mut command = isolated_loader_process();
            command.args(["--exact", "surface_config::tests::local_planning_boolean_uses_the_actual_environment_loader", "--nocapture"])
                .env(CHILD, expected).env("NUWAX_NAV_BODY_TO_SUPPORT_Z_M", "0.4");
            if let Some(value) = value { command.env("NUWAX_NAV_LOCAL_PLANNING_ENABLED", value); }
            let result = command.output().unwrap();
            assert!(result.status.success(), "{value:?}: {} {}", String::from_utf8_lossy(&result.stdout), String::from_utf8_lossy(&result.stderr));
        }
    }

    #[test]
    fn directional_step_settings_reach_the_body_and_geometry_from_the_real_loader() {
        const CHILD: &str = "NUWAX_TEST_DIRECTIONAL_STEPS";
        if let Ok(case) = std::env::var(CHILD) {
            let loaded = SurfaceNavigationConfig::from_env();
            if case == "invalid" {
                assert!(loaded.is_err());
                return;
            }
            let config = loaded.unwrap();
            let body = config.body_envelope.unwrap();
            let expected = if case == "inherit" { [0.15, 0.15] } else { [0.2, 0.1] };
            assert!((body.max_step_up_m - expected[0]).abs() < 1e-6);
            assert!((body.max_step_down_m - expected[1]).abs() < 1e-6);
            assert!((f64::from(config.geometry.step_threshold_m) - expected[0]).abs() < 1e-6);
            return;
        }
        let model = r#"{"footprint_xy_m":[[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],"z_min_m":-0.05,"z_max_m":0.2,"source":"设置回归"}"#;
        for (case, up, down) in [("configured", "0.2", "0.1"), ("inherit", "", ""),
            ("invalid", "-0.1", "0.2"), ("invalid", "NaN", "0.2"), ("invalid", "0.2", "1.1")] {
            let result = isolated_loader_process()
                .args(["--exact", "surface_config::tests::directional_step_settings_reach_the_body_and_geometry_from_the_real_loader", "--nocapture"])
                .env(CHILD, case).env("NUWAX_NAV_BODY_TO_SUPPORT_Z_M", "0.4")
                .env("NUWAX_ROBOT_GEOMETRY_JSON", model)
                .env("NUWAX_NAV_MAX_STEP_UP_M", up).env("NUWAX_NAV_MAX_STEP_DOWN_M", down)
                .output().unwrap();
            assert!(result.status.success(), "台阶设置加载失败：{}", String::from_utf8_lossy(&result.stderr));
        }
    }

    #[test]
    fn step_settings_cannot_exceed_or_restore_undeclared_robot_ability() {
        let caps = NavigationCapabilities::from_json(r#"{
            "schema_version":1,"calibration_id":"台阶设置回归",
            "motion":{"model":"omnidirectional","max_forward_mps":1.0,"max_reverse_mps":0.0,
                "max_lateral_mps":0.0,"max_angular_rps":1.0,"linear_acceleration_mps2":1.0,
                "guaranteed_braking_mps2":1.0,"angular_acceleration_rps2":2.0,
                "guaranteed_angular_braking_rps2":2.0,"reaction_time_sec":0.1,
                "minimum_turning_radius_m":0.0,"can_rotate_in_place":true},
            "terrain":{"max_step_up_m":0.2,"max_step_down_m":0.1,"min_tread_depth_m":0.2,
                "max_slope_deg":18.0,"max_cross_slope_deg":18.0,"support_height_uncertainty_m":0.0,
                "max_observation_age_sec":2.0,"stairs_supported":true}}
        "#).unwrap();
        let robot = RobotGeometry {
            self_collision: None,
            footprint_xy_m: vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
            z_min_m: -0.05, z_max_m: 0.2, source: "台阶设置回归".into(),
        };
        for (up, down, accepted) in [(0.2,0.1,true), (0.1,0.0,true), (0.21,0.1,false), (0.2,0.2,false)] {
            let mut config = SurfaceNavigationConfig::new("map".into(),0.4,0.3,0.6,0.1,0.15,18.0,0.2).unwrap();
            config.apply_calibration(Some(robot.clone()),Some(caps.clone())).unwrap();
            assert_eq!(config.apply_step_overrides(Some(up),Some(down),Some(&caps)).is_ok(),accepted);
        }
        let mut absent = caps;
        absent.terrain = None;
        let mut config = SurfaceNavigationConfig::new("map".into(),0.4,0.3,0.6,0.1,0.15,18.0,0.2).unwrap();
        config.apply_calibration(Some(robot),Some(absent.clone())).unwrap();
        assert!(config.apply_step_overrides(Some(0.2),None,Some(&absent)).is_err());
        assert!(config.apply_step_overrides(None,Some(0.2),Some(&absent)).is_err());
    }

    #[test]
    fn actuator_speed_is_not_capped_by_the_legacy_controller_default() {
        const CHILD: &str = "NUWAX_TEST_SURFACE_SPEED_CHILD";
        if let Ok(expected)=std::env::var(CHILD) {
            let config=SurfaceNavigationConfig::from_env().unwrap();
            assert_eq!(config.follower.controller.speed,expected.parse::<f64>().unwrap());
            return;
        }
        for speed in ["1.0","0.35"] {
            let result=isolated_loader_process()
                .args(["--exact","surface_config::tests::actuator_speed_is_not_capped_by_the_legacy_controller_default","--nocapture"])
                .env(CHILD,speed).env("NUWAX_NAV_BODY_TO_SUPPORT_Z_M","0.4")
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
