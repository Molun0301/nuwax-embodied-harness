//! 楼梯、斜坡规划与跟随共用的外形和物理参数。

use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope};
use nuwax_nav_runtime::SurfaceFollowerConfig;
use nuwax_nav_types_messages::{MotionKinematics, MotionModel, NavigationCapabilities, RobotGeometry};

use crate::{NavStackError, NavStackResult};

/// 一次启动捕获并同时传给规划器和控制器的三维导航参数。
#[derive(Clone)]
pub struct SurfaceNavigationConfig {
    /// 支撑面、净空、台阶和坡道约束。
    pub geometry: Config,
    /// 启动时捕获的通信地址。
    pub lcm_url: String,
    /// 由统一机器人外形得到的完整机体包络。
    pub body_envelope: SurfaceBodyEnvelope,
    /// 机体参考位姿与受控速度参数。
    pub follower: SurfaceFollowerConfig,
    /// 当前后台搜索的取消信号；不进入机器人标定或地图身份。
    pub(crate) planning_cancelled: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
    /// 原生全局地图缓存与搜索实例，由导航拥有方共享。
    pub planner: std::sync::Arc<crate::surface_oriented::PctPlanner>,
}

impl SurfaceNavigationConfig {
    /// 从合成配置读取统一外形与已推导的执行器参数。
    pub fn from_runtime() -> NavStackResult<Self> {
        let error = NavStackError::InvalidConfig;
        let mut result = Self::from_config(
            nuwax_config::navigation().map_err(error)?,
            nuwax_config::robot().map_err(error)?,
            nuwax_config::derived().map_err(error)?,
        )?;
        result.lcm_url = nuwax_config::transport().map_err(error)?.lcm_url.clone();
        Ok(result)
    }
    /// 显式构造供调用方与测试共用。
    pub fn from_config(
        n: &nuwax_config::sections::Navigation,
        r: &nuwax_config::sections::Robot,
        d: &nuwax_config::sections::Derived,
    ) -> NavStackResult<Self> {
        let capabilities = NavigationCapabilities::from_config(&n.capabilities_json)
            .map_err(NavStackError::InvalidConfig)?;
        let body = RobotGeometry::from_runtime_json(&d.robot_geometry_json)
            .map_err(NavStackError::InvalidConfig)?;
        let mut config = Self::new(
            r.map_frame.clone(),
            body.clone(),
            n.surface_voxel_size_m,
            n.traversability_max_step_height_m,
            n.traversability_max_slope_deg,
            n.min_stair_tread_m,
            n.obstacle_clearance_m,
        )?;
        config.apply_calibration(body, capabilities.clone())?;
        if let Some(limits) = config.follower.motion_limits.as_mut() {
            limits.min_executable_linear_mps = d.min_executable_linear_mps;
            limits.min_executable_angular_rps = d.min_executable_angular_rps;
        }
        config.apply_step_overrides(n.max_step_up_m, n.max_step_down_m, capabilities.as_ref())?;
        config.planner = std::sync::Arc::new(crate::surface_oriented::PctPlanner::new(
            nuwax_nav_runtime::native_map::PctTerrainSource::parse(&n.pct_terrain_source)
                .map_err(NavStackError::InvalidConfig)?,
        ));
        config.follower.scan.planning_horizon = n.local_horizon_m;
        if d.actuator_max_linear_mps > 0.0 {
            let limit = d.actuator_max_linear_mps;
            config.follower.controller.speed = config
                .follower
                .motion_limits
                .as_ref()
                .map_or(limit, |m| limit.min(m.max_forward_mps));
        }
        if d.actuator_max_yaw_rps > 0.0 {
            config.follower.max_angular_speed_rps = d.actuator_max_yaw_rps;
            if let Some(limits) = config.follower.motion_limits.as_mut() {
                limits.max_angular_rps = limits.max_angular_rps.min(d.actuator_max_yaw_rps);
            }
        }
        if !d.actuator_motion_model.is_empty() {
            if d.actuator_motion_model != "omnidirectional" {
                return Err(NavStackError::InvalidConfig(
                    "不支持的执行器基本运动形式".into(),
                ));
            }
            let speed = d.actuator_max_linear_mps;
            let motion = MotionKinematics {
                model: MotionModel::Omnidirectional,
                max_forward_mps: speed,
                max_reverse_mps: speed,
                max_lateral_mps: speed,
                max_angular_rps: d.actuator_max_yaw_rps,
                min_executable_linear_mps: d.min_executable_linear_mps,
                min_executable_angular_rps: d.min_executable_angular_rps,
                minimum_turning_radius_m: 0.0,
                can_rotate_in_place: true,
            };
            motion.validate().map_err(NavStackError::InvalidConfig)?;
            config.follower.basic_kinematics = Some(motion);
        }
        tracing::info!(target:"navigation.config",event="navigation.config.loaded",profile=%d.nav_profile,
                        voxel_m=config.geometry.voxel_size,local_horizon_m=config.follower.scan.planning_horizon,
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
        let up = up.unwrap_or(self.body_envelope.max_step_up_m);
        let down = down.unwrap_or(self.body_envelope.max_step_down_m);
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
        {
            let body = &mut self.body_envelope;
            body.max_step_up_m = up;
            body.max_step_down_m = down;
            body.validate().map_err(|e| NavStackError::InvalidConfig(e.to_string()))?;
        }
        self.geometry.step_threshold_m = up.max(down) as f32;
        Ok(())
    }

    /// 显式构造便于部署校验和无硬件回归，不读取或修改进程环境。
    pub fn new(
        frame: String,
        body: RobotGeometry,
        voxel_size_m: f64,
        max_step_height_m: f64,
        max_slope_deg: f64,
        min_stair_tread_m: f64,
        obstacle_clearance_m: f64,
    ) -> NavStackResult<Self> {
        if frame != "map"
            || [voxel_size_m, min_stair_tread_m]
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
        let envelope = SurfaceBodyEnvelope {
            fixed_collision: body.self_collision,
            footprint_xy_m: body.footprint_xy_m,
            z_bounds_m: [body.z_min_m, body.z_max_m],
            body_to_support_z_m: -body.z_min_m,
            max_step_up_m: max_step_height_m,
            max_step_down_m: max_step_height_m,
            max_cross_slope_deg: max_slope_deg,
            support_uncertainty_m: 0.0,
        };
        let geometry = Config {
            world_frame: frame,
            voxel_size: voxel_size_m as f32,
            robot_height: voxel_size_m as f32,
            // 生产支撑面不能靠形态学补出原本没有观测证据的地面。
            surface_closing_radius: 0.0,
            node_spacing_m: (voxel_size_m * 3.0) as f32,
            wall_clearance_m: 0.0,
            wall_buffer_m: (envelope.radius() + 0.1) as f32,
            wall_buffer_weight: 2.0,
            step_threshold_m: max_step_height_m as f32,
            step_penalty_weight: 2.0,
            max_slope_deg: max_slope_deg as f32,
            min_step_tread_depth_m: min_stair_tread_m as f32,
            goal_tolerance: 0.2,
            viz_publish_hz: 0.0,
        };
        if !geometry.voxel_size.is_normal()
            || !geometry.min_step_tread_depth_m.is_normal()
            || !geometry.step_threshold_m.is_finite()
            || !geometry.node_spacing_m.is_normal()
            || geometry.min_step_tread_depth_m < geometry.voxel_size * 2.0
        {
            return Err(NavStackError::InvalidConfig(
                "surface geometry exceeds float32 domain or tread is below two voxels".into(),
            ));
        }
        let mut follower = SurfaceFollowerConfig::new(-body.z_min_m, obstacle_clearance_m);
        // 三维生产调度为20赫兹，速度窗口的加减速时间必须采用同一控制周期。
        follower.controller.control_frequency =
            1.0 / nuwax_nav_runtime::DEFAULT_TICK_INTERVAL.as_secs_f64();
        Ok(Self {
            lcm_url: String::new(),
            geometry,
            body_envelope: envelope,
            follower,
            planning_cancelled: None,
            planner: Default::default(),
        })
    }
    /// 两端用同一已验证标定物化外形和运动约束。
    pub fn apply_calibration(
        &mut self,
        body: RobotGeometry,
        capabilities: Option<NavigationCapabilities>,
    ) -> NavStackResult<()> {
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
        {
            self.follower.body_to_support_z_m = -body.z_min_m;
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
                let previous_radius=self.body_envelope.radius();
                let padding=(f64::from(self.geometry.wall_buffer_m)-previous_radius).max(0.0);
                self.geometry.wall_buffer_m=(envelope.radius()+padding) as f32;
            }
            self.geometry.robot_height = self.geometry.voxel_size;
            self.body_envelope = envelope;
        }
        Ok(())
    }
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
        let mut config = SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.15, 18.0, 0.2, 0.15).unwrap();
        config.apply_calibration(body, None).unwrap();
        let envelope = &config.body_envelope;
        let fixed = envelope.fixed_collision.as_ref().unwrap();
        assert!(std::sync::Arc::ptr_eq(&shared,fixed));
        assert!(std::sync::Arc::ptr_eq(fixed,&config.clone().body_envelope.fixed_collision.as_ref().unwrap()));
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
        let mut config = SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.15, 18.0, 0.2, 0.15).unwrap();
        config.apply_calibration(body, None).unwrap();
        assert!(config.body_envelope.fixed_collision.is_none());
    }

    fn loader_fixture() -> (
        nuwax_config::sections::Navigation,
        nuwax_config::sections::Robot,
        nuwax_config::sections::Derived,
    ) {
        (nuwax_config::test_support::section("navigation"),nuwax_config::test_support::section("robot"),
                    nuwax_config::sections::Derived { min_executable_linear_mps:0.0,min_executable_angular_rps:0.0,nav_profile:"pct-scan-3d".into(),actuator_max_linear_mps:0.0,
                        actuator_max_yaw_rps:0.0,actuator_motion_model:String::new(),robot_geometry_json:
                        r#"{"footprint_xy_m":[[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],"z_min_m":-0.05,"z_max_m":0.2,"source":"设置回归"}"#.into()})
    }

    #[test]
    fn directional_step_settings_reach_the_body_and_geometry_from_the_real_loader() {
        for (up, down, accepted) in [
            (Some(0.2), Some(0.1), true),
            (None, None, true),
            (Some(-0.1), Some(0.2), false),
            (Some(f64::NAN), Some(0.2), false),
            (Some(0.2), Some(1.1), false),
        ] {
            let (mut n, r, d) = loader_fixture();
            n.max_step_up_m = up;
            n.max_step_down_m = down;
            let loaded = SurfaceNavigationConfig::from_config(&n, &r, &d);
            assert_eq!(loaded.is_ok(), accepted);
            if accepted {
                let config = loaded.unwrap();
                let expected = [up.unwrap_or(0.15), down.unwrap_or(0.15)];
                assert!((config.body_envelope.max_step_up_m - expected[0]).abs() < 1e-6);
                assert!((config.body_envelope.max_step_down_m - expected[1]).abs() < 1e-6);
                assert!((f64::from(config.geometry.step_threshold_m) - expected[0]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn step_settings_cannot_exceed_or_restore_undeclared_robot_ability() {
        let caps = NavigationCapabilities::from_json(r#"{
            "schema_version":1,"calibration_id":"台阶设置回归",
            "motion":{"model":"omnidirectional","max_forward_mps":1.0,"max_reverse_mps":0.0,
                "max_lateral_mps":0.0,"max_angular_rps":1.0,"min_executable_linear_mps":0.0,"min_executable_angular_rps":0.0,"linear_acceleration_mps2":1.0,
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
            let mut config = SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.15, 18.0, 0.2, 0.15).unwrap();
            config.apply_calibration(robot.clone(), Some(caps.clone())).unwrap();
            assert_eq!(config.apply_step_overrides(Some(up),Some(down),Some(&caps)).is_ok(),accepted);
        }
        let mut absent = caps;
        absent.terrain = None;
        let mut config = SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.15, 18.0, 0.2, 0.15).unwrap();
        config.apply_calibration(robot, Some(absent.clone())).unwrap();
        assert!(config.apply_step_overrides(Some(0.2),None,Some(&absent)).is_err());
        assert!(config.apply_step_overrides(None,Some(0.2),Some(&absent)).is_err());
    }

    #[test]
    fn actuator_speed_is_not_capped_by_the_legacy_controller_default() {
        for speed in [1.0, 0.35] {
            let (n, r, mut d) = loader_fixture();
            d.actuator_max_linear_mps = speed;
            let config = SurfaceNavigationConfig::from_config(&n, &r, &d).unwrap();
            assert_eq!(config.follower.controller.speed, speed);
        }
    }

    #[test]
    fn geometry_and_calibration_are_validated_before_starting_either_owner() {
        for (frame, offset, voxel, tread, step) in [
            ("odom", 0.4, 0.1, 0.2, 0.2),
            ("map", 0.4, 0.1, 0.1, 0.2),
            ("map", 0.4, 0.1, 0.2, 1e300),
        ] {
            assert!(SurfaceNavigationConfig::new(frame.into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(offset), z_max_m: (0.6)-(offset), source: "测试外形".into() }, voxel, step, 35.0, tread, 0.15)
            .is_err());
        }
        let valid =
            SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.2, 35.0, 0.2, 0.15).unwrap();
        assert_eq!(valid.geometry.wall_clearance_m, 0.0);
        assert_eq!(valid.follower.body_to_support_z_m, 0.4);
        assert_eq!(valid.geometry.surface_closing_radius, 0.0);
    }
}
