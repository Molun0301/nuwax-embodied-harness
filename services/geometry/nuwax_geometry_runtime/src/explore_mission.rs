//! 探索预算复用两条生产规划算法；查询只持有不可变地图，不取得运动权。
use super::explore_runtime::{ExploreRobotPose, ExploreStartConfig, ExploreStopConditions};
use nuwax_exploration::mission::MissionPose;
use nuwax_exploration::ExplorationBoundary;
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_simple_planner::{Costmap, SimplePlanner, SimplePlannerConfig};
use nuwax_nav_stack::surface_config::SurfaceNavigationConfig;
use nuwax_nav_types_messages::{
    GoalOptions, NavigationCapabilities, OccupancyCell, OccupancyGrid2D, Point2, Point3, Pose3,
    Quaternion, RobotGeometry,
};
use std::sync::Arc;

/// 原地图资产拥有方将此载荷与已保存资产绑定；不保存旧子目标或准入凭据。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ExplorationCheckpoint {
    pub schema_version: u32,
    pub request_id: String,
    pub map_name: String,
    pub map_frame_id: String,
    pub origin_session_id: String,
    pub origin_session_generation: i64,
    pub origin_frame_generation: i64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub configuration: Option<ExplorationConfiguration>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation_configuration: Option<NavigationConfiguration>,
    /// V1 values are opaque read-compatible metadata, never recomputed or used as proof.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub configuration_sha256: Option<[u8; 32]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub navigation_configuration_sha256: Option<[u8; 32]>,
    pub elapsed_ns: u64,
    pub reached_goals: u32,
    pub mission: Option<nuwax_exploration::mission::MissionProgress>,
    pub return_reason: Option<nuwax_exploration::ExplorationStopReason>,
    pub sources: Vec<nuwax_exploration::mission::SourceCheckpoint>,
}
impl ExplorationCheckpoint {
    pub fn parse(payload: &str) -> Result<Self, String> {
        if payload.len() > 2_097_152 {
            return Err("探索进度超过两兆字节".into());
        }
        let value: Self =
            serde_json::from_str(payload).map_err(|e| format!("探索进度载荷无效：{e}"))?;
        if !matches!(value.schema_version, 1 | 2)
            || value.map_name.trim().is_empty()
            || value.map_name.len() > 1024
            || value.map_frame_id.trim().is_empty()
            || value.map_frame_id.len() > 255
            || value.request_id.is_empty()
            || value.request_id.len() > 1024
            || value.origin_session_id.is_empty()
            || value.origin_session_id.len() > 1024
            || value.origin_session_generation <= 0
            || value.origin_frame_generation <= 0
            || value.elapsed_ns > 365 * 24 * 3600 * 1_000_000_000_u64
            || value.sources.is_empty()
            || value.sources.len() > 16
            || value
                .sources
                .iter()
                .any(|s| s.name.len() > 128 || s.payload.len() > 1_048_576)
        {
            return Err("探索进度版本、地图身份或容量无效".into());
        }
        match (&value.configuration, &value.navigation_configuration) {
            (Some(configuration), Some(navigation)) => {
                configuration.validate()?;
                navigation.validate()?;
            }
            (None, None) if value.schema_version == 1
                && value.configuration_sha256.is_some()
                && value.navigation_configuration_sha256.is_some() => {}
            _ => return Err("探索进度缺少完整的实际配置".into()),
        }
        if let Some(mission) = value.mission.as_ref() {
            mission.validate()?;
        }
        if value.return_reason.is_some_and(|r| {
            r.terminal_kind() != nuwax_exploration::ExplorationTerminalKind::Succeeded
        }) || value.mission.as_ref().is_some_and(|m| m.returning)
            != value.return_reason.is_some()
        {
            return Err("探索返航阶段与结束原因不一致".into());
        }
        Ok(value)
    }
}

/// Frozen behavior settings. Profile version identifies the interpretation, not the values.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ExplorationConfiguration {
    pub profile_version: u32,
    pub source: String,
    pub strategy: String,
    pub arbitration: String,
    pub boundary: Option<Vec<[f64; 2]>>,
    pub return_policy: nuwax_exploration::mission::ReturnPolicy,
    pub stop_conditions: ExploreStopConditions,
    pub max_navigation_retries: u32,
    pub blacklist_radius_m: f64,
}
impl ExplorationConfiguration {
    pub fn capture(config: &ExploreStartConfig, surface: bool) -> Self {
        Self {
            profile_version: 1,
            source: if surface { "surface" } else { "wavefront" }.into(),
            strategy: config.strategy.clone(),
            arbitration: config.arbitration.clone(),
            boundary: config.boundary.as_ref().map(|boundary| {
                boundary.vertices().iter().map(|point| [point.x, point.y]).collect()
            }),
            return_policy: config.return_policy.clone(),
            stop_conditions: config.stop_conditions.clone(),
            max_navigation_retries: config.max_navigation_retries,
            blacklist_radius_m: config.blacklist_radius_m,
        }
    }
    fn validate(&self) -> Result<(), String> {
        if self.profile_version != 1 || !matches!(self.source.as_str(), "wavefront" | "surface") {
            return Err("探索配置版本或来源无效".into());
        }
        let boundary = self.boundary.as_ref().map(|vertices| {
            ExplorationBoundary::from_vertices(vertices.iter().map(|p| Point2::new(p[0], p[1])).collect())
                .map_err(|error| error.to_string())
        }).transpose()?;
        ExploreStartConfig {
            strategy: self.strategy.clone(),
            arbitration: self.arbitration.clone(),
            boundary,
            return_policy: self.return_policy.clone(),
            stop_conditions: self.stop_conditions.clone(),
            max_navigation_retries: self.max_navigation_retries,
            blacklist_radius_m: self.blacklist_radius_m,
            ..Default::default()
        }.validate_behavior(self.source == "surface")?;
        Ok(())
    }
    fn changed_fields(&self, current: &Self) -> Vec<&'static str> {
        let mut fields = Vec::new();
        macro_rules! changed {
            ($($field:ident),+ $(,)?) => {$(if self.$field != current.$field { fields.push(concat!("configuration.", stringify!($field))); })+};
        }
        changed!(profile_version, source, strategy, arbitration, boundary, return_policy,
            max_navigation_retries, blacklist_radius_m);
        macro_rules! stop_changed {
            ($($field:ident),+ $(,)?) => {$(if self.stop_conditions.$field != current.stop_conditions.$field { fields.push(concat!("configuration.stop_conditions.", stringify!($field))); })+};
        }
        stop_changed!(max_coverage_pct, no_frontier_timeout_ms, max_duration_ms, max_goals);
        fields
    }
}

/// Typed values previously reduced to a navigation configuration digest.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct NavigationConfiguration {
    pub profile_version: u32,
    pub capabilities: Option<NavigationCapabilities>,
    pub geometry: Option<RobotGeometry>,
    pub body_to_support_z_m: Option<f64>,
    pub base_frame: String,
}
impl NavigationConfiguration {
    pub fn capture() -> Result<Self, String> {
        let value = Self {
            profile_version: 1,
            capabilities: NavigationCapabilities::from_env()?,
            geometry: std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").ok()
                .filter(|value| !value.trim().is_empty())
                .map(|value| RobotGeometry::from_runtime_json(&value)).transpose()?,
            body_to_support_z_m: std::env::var("NUWAX_NAV_BODY_TO_SUPPORT_Z_M").ok()
                .filter(|value| !value.trim().is_empty())
                .map(|value| value.parse::<f64>().map_err(|_| "机体支撑偏移无效".to_string())).transpose()?,
            base_frame: std::env::var("NUWAX_BASE_FRAME").unwrap_or_default(),
        };
        value.validate()?;
        Ok(value)
    }
    fn validate(&self) -> Result<(), String> {
        if self.profile_version != 1 {
            return Err("导航配置版本无效".into());
        }
        if let Some(capabilities) = &self.capabilities { capabilities.validate()?; }
        if let Some(geometry) = &self.geometry { geometry.validate()?; }
        if self.body_to_support_z_m.is_some_and(|value| !value.is_finite() || value <= 0.0) {
            return Err("机体支撑偏移必须为正值".into());
        }
        Ok(())
    }
    fn changed_fields(&self, current: &Self) -> Vec<&'static str> {
        let mut fields = Vec::new();
        macro_rules! changed {
            ($($field:ident),+ $(,)?) => {$(if self.$field != current.$field { fields.push(concat!("navigation_configuration.", stringify!($field))); })+};
        }
        changed!(profile_version, capabilities, geometry, body_to_support_z_m, base_frame);
        fields
    }
}

impl ExplorationCheckpoint {
    /// Home and measured distance are facts; the old decision to return is not.
    pub fn prepare_history_for_replan(&mut self) {
        if let Some(mission) = self.mission.as_mut() { mission.returning = false; }
        self.return_reason = None;
    }

    /// True means history-only legacy resume, not proof of the original configuration.
    pub fn resume_requires_replan(
        &self,
        configuration: &ExplorationConfiguration,
        navigation: &NavigationConfiguration,
    ) -> Result<bool, String> {
        match (&self.configuration, &self.navigation_configuration) {
            (Some(saved), Some(saved_navigation)) => {
                let mut changed = saved.changed_fields(configuration);
                changed.extend(saved_navigation.changed_fields(navigation));
                if changed.is_empty() { Ok(false) } else {
                    Err(format!("探索进度配置发生变化：{}", changed.join(", ")))
                }
            }
            (None, None) if self.schema_version == 1 => Ok(true),
            _ => Err("探索进度缺少完整的实际配置".into()),
        }
    }
}

pub(crate) fn mission_pose(pose: &ExploreRobotPose) -> MissionPose {
    let q = pose.orientation;
    MissionPose {
        position: [pose.x, pose.y, pose.z],
        orientation_xyzw: [q.x, q.y, q.z, q.w],
    }
}
pub(crate) fn pose3(pose: &MissionPose) -> Pose3 {
    let [x, y, z, w] = pose.orientation_xyzw;
    Pose3 {
        position: Point3::new(pose.position[0], pose.position[1], pose.position[2]),
        orientation: Quaternion { x, y, z, w },
    }
}

/// 每次运行捕获与导航相同的明确标定，不能在任务中途按环境变化替换。
#[derive(Clone)]
pub(crate) struct MissionCalibration {
    pub planar: SimplePlannerConfig,
    pub maximum_sample_speed_mps: f64,
}
impl MissionCalibration {
    pub fn capture(surface: Option<&SurfaceNavigationConfig>) -> Result<Self, String> {
        let caps = NavigationCapabilities::from_env()?.ok_or("返航行程统计需要导航运动标定")?;
        let geometry = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON")
            .ok()
            .filter(|s| !s.trim().is_empty())
            .ok_or_else(|| "返航路线需要机体外形标定".to_string())
            .and_then(|s| RobotGeometry::from_runtime_json(&s))?;
        let planar_speed = caps
            .motion
            .max_forward_mps
            .max(caps.motion.max_reverse_mps)
            .hypot(caps.motion.max_lateral_mps);
        let mut maximum_sample_speed_mps = planar_speed;
        if let Some(config) = surface {
            let terrain = caps
                .terrain
                .as_ref()
                .ok_or("三维返航行程需要地形运动标定")?;
            if config.body_envelope.is_none() {
                return Err("三维返航路线需要完整机体包络".into());
            }
            // 计入纵横坡、踏面起伏与支撑偏移的转动速度，供拒绝坐标跳变使用。
            let gradient = terrain
                .max_slope_deg
                .to_radians()
                .tan()
                .hypot(terrain.max_cross_slope_deg.to_radians().tan())
                .max(
                    terrain.max_step_up_m.max(terrain.max_step_down_m)
                        / terrain.min_tread_depth_m.max(1e-6),
                );
            maximum_sample_speed_mps = planar_speed * (1.0 + gradient * gradient).sqrt()
                + caps.motion.max_angular_rps * config.follower.body_to_support_z_m;
        }
        Ok(Self {
            planar: SimplePlannerConfig {
                robot_geometry: Some(geometry),
                motion_limits: Some(caps.motion),
                ..Default::default()
            },
            maximum_sample_speed_mps,
        })
    }
}

/// 与实际子目标派发共用约束，返航始终保留冻结 home 的位置与朝向要求。
pub(crate) fn home_goal_options() -> GoalOptions {
    GoalOptions { allow_reverse: false, position_tolerance_m: 0.1, yaw_tolerance_rad: 0.2 }
}

pub(crate) fn frontier_goal_options(
    goal: &nuwax_nav_types_messages::visual::FrontierGoal,
    surface: bool,
    config: Option<&SurfaceNavigationConfig>,
) -> GoalOptions {
    let mut options = GoalOptions::default();
    if surface {
        options.allow_reverse = config.and_then(|config| config.follower.kinematics())
            .is_some_and(|motion| motion.max_reverse_mps > 0.0);
        if goal.metadata.get("goal_orientation_required").is_some_and(|value| value == "false") {
            options.yaw_tolerance_rad = std::f64::consts::PI;
        }
    }
    options
}

/// 单次已知地图查询的实际长度和末端；不缓存路径或承诺未来执行轨迹。
pub(crate) struct MissionRoute {
    pub length: f64,
    pub end: MissionPose,
}

/// 在原调度任务等待的阻塞计算内使用；结果只是本次已知地图的路线估计。
pub(crate) enum MissionRoutes {
    Planar {
        map: Costmap,
        config: SimplePlannerConfig,
    },
    Surface {
        map: Arc<SurfaceSnapshot>,
        config: SurfaceNavigationConfig,
        boundary: Vec<[f64; 2]>,
    },
}
impl MissionRoutes {
    pub fn planar(
        mut grid: OccupancyGrid2D,
        calibration: &MissionCalibration,
        config: &ExploreStartConfig,
    ) -> Result<Self, String> {
        if let Some(boundary) = config.boundary.as_ref() {
            for y in 0..grid.height {
                for x in 0..grid.width {
                    let min = Point2::new(
                        grid.origin.x + f64::from(x) * grid.resolution,
                        grid.origin.y + f64::from(y) * grid.resolution,
                    );
                    if !cell_in_boundary(boundary, min, grid.resolution) {
                        grid.cells[y as usize * grid.width as usize + x as usize] =
                            OccupancyCell::Occupied;
                    }
                }
            }
        }
        let map = Costmap::from_occupancy(grid, calibration.planar.inflation_radius)
            .map_err(|e| e.to_string())?;
        Ok(Self::Planar {
            map,
            config: calibration.planar.clone(),
        })
    }
    pub fn query(
        &self,
        from: &MissionPose,
        to: &MissionPose,
        options: GoalOptions,
        cancellation: Option<&std::sync::atomic::AtomicBool>,
    ) -> Result<MissionRoute, String> {
        from.validate()?;
        to.validate()?;
        // 零距离仍运行真实可通行查询；直接累计原路径，不再复制位置数组。
        match self {
            Self::Planar { map, config } => {
                let mut planner = SimplePlanner::new(config.clone()).map_err(|e| e.to_string())?;
                planner.replace_costmap(map.clone());
                let (path, end_yaw) = planner
                    .query_route_with_end_yaw(pose3(from), pose3(to), options)
                    .map_err(|e| e.to_string())?;
                let end = path.last().ok_or("返航路线为空")?;
                let orientation = Quaternion::from_yaw(end_yaw);
                Ok(MissionRoute {
                    length: path.windows(2).map(|p| ((p[0].x - p[1].x).powi(2) + (p[0].y - p[1].y).powi(2)).sqrt()).sum(),
                    end: MissionPose {
                        position: [end.x, end.y, from.position[2]],
                        orientation_xyzw: [orientation.x, orientation.y, orientation.z, orientation.w],
                    },
                })
            }
            Self::Surface { map, config, boundary } => {
                let path = nuwax_nav_stack::surface_oriented::plan_cancellable(
                    map,
                    (from.position, from.orientation_xyzw),
                    (to.position, to.orientation_xyzw),
                    options,
                    config.follower.motion_limits.as_ref(),
                    boundary,
                    30_000,
                    cancellation,
                )?;
                let end = path.last().ok_or("返航路线为空")?;
                Ok(MissionRoute {
                    length: path.windows(2).map(|p| {
                        p[0].0.iter().zip(p[1].0).map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt()
                    }).sum(),
                    end: MissionPose { position: end.0, orientation_xyzw: end.1 },
                })
            }
        }
    }
}

fn cell_in_boundary(boundary: &ExplorationBoundary, min: Point2, resolution: f64) -> bool {
    // 完整格的四角和边界线段均在多边形内，凹多边形不能跨越缺口。
    let corners = [
        min,
        Point2::new(min.x + resolution, min.y),
        Point2::new(min.x + resolution, min.y + resolution),
        Point2::new(min.x, min.y + resolution),
    ];
    if !corners.iter().all(|p| boundary.contains(*p)) {
        return false;
    }
    let vertices = boundary.vertices();
    !vertices.iter().enumerate().any(|(i, a)| {
        let b = vertices[(i + 1) % vertices.len()];
        let mut lo: f64 = 0.0;
        let mut hi: f64 = 1.0;
        for (start, delta, min) in [(a.x, b.x - a.x, min.x), (a.y, b.y - a.y, min.y)] {
            let lower = min + resolution * 1e-8;
            let upper = min + resolution - resolution * 1e-8;
            if delta.abs() < 1e-15 {
                if start <= lower || start >= upper {
                    return false;
                }
            } else {
                let u = (lower - start) / delta;
                let v = (upper - start) / delta;
                lo = lo.max(u.min(v));
                hi = hi.min(u.max(v));
            }
        }
        lo < hi
    })
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    fn navigation_configuration() -> NavigationConfiguration {
        NavigationConfiguration {
            profile_version: 1,
            capabilities: None,
            geometry: None,
            body_to_support_z_m: None,
            base_frame: String::new(),
        }
    }
    fn checkpoint() -> ExplorationCheckpoint {
        ExplorationCheckpoint {
            schema_version: 2,
            request_id: "checkpoint-parent".into(), map_name: "map".into(), map_frame_id: "map".into(),
            origin_session_id: "session".into(), origin_session_generation: 1, origin_frame_generation: 1,
            configuration: Some(ExplorationConfiguration::capture(&ExploreStartConfig::default(), false)),
            navigation_configuration: Some(navigation_configuration()),
            configuration_sha256: None, navigation_configuration_sha256: None,
            elapsed_ns: 27, reached_goals: 2, mission: None, return_reason: None,
            sources: vec![nuwax_exploration::mission::SourceCheckpoint { name: "wavefront".into(), payload: "opaque-source".into() }],
        }
    }
    #[test]
    fn checkpoint_v2_roundtrips_actual_settings_without_digest_fields() {
        let saved = checkpoint();
        let payload = serde_json::to_string(&saved).unwrap();
        assert!(!payload.contains("sha256"));
        let parsed = ExplorationCheckpoint::parse(&payload).unwrap();
        assert_eq!(parsed.schema_version, 2);
        assert!(!parsed.resume_requires_replan(saved.configuration.as_ref().unwrap(), saved.navigation_configuration.as_ref().unwrap()).unwrap());
        let mut config = saved.configuration.clone().unwrap();
        config.max_navigation_retries += 1;
        assert!(parsed.resume_requires_replan(&config, saved.navigation_configuration.as_ref().unwrap()).unwrap_err().contains("configuration.max_navigation_retries"));
        config = saved.configuration.clone().unwrap();
        config.stop_conditions.max_goals = Some(5);
        assert!(parsed.resume_requires_replan(&config, saved.navigation_configuration.as_ref().unwrap()).unwrap_err().contains("configuration.stop_conditions.max_goals"));
        let mut navigation = saved.navigation_configuration.clone().unwrap();
        navigation.body_to_support_z_m = Some(0.45);
        assert!(parsed.resume_requires_replan(saved.configuration.as_ref().unwrap(), &navigation).unwrap_err().contains("navigation_configuration.body_to_support_z_m"));
    }
    #[test]
    fn checkpoint_v2_compares_values_even_when_calibration_id_is_unchanged() {
        let mut saved = checkpoint();
        let motion = nuwax_nav_types_messages::MotionLimits {
            model: nuwax_nav_types_messages::MotionModel::Differential,
            max_forward_mps: 0.5, max_reverse_mps: 0.2, max_lateral_mps: 0.0, max_angular_rps: 1.0,
            linear_acceleration_mps2: 1.0, guaranteed_braking_mps2: 1.0,
            angular_acceleration_rps2: 1.0, guaranteed_angular_braking_rps2: 1.0,
            reaction_time_sec: 0.1, minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
        };
        saved.navigation_configuration.as_mut().unwrap().capabilities = Some(NavigationCapabilities {
            schema_version: 1, calibration_id: "same-calibration".into(), motion, observation: None, terrain: None,
        });
        let payload = serde_json::to_string(&saved).unwrap();
        let parsed = ExplorationCheckpoint::parse(&payload).unwrap();
        let mut current = parsed.navigation_configuration.clone().unwrap();
        current.capabilities.as_mut().unwrap().motion.max_forward_mps = 0.6;
        assert!(parsed.resume_requires_replan(parsed.configuration.as_ref().unwrap(), &current).unwrap_err().contains("navigation_configuration.capabilities"));
    }
    #[test]
    fn checkpoint_legacy_metadata_is_opaque_and_settings_if_present_are_compared() {
        let mut legacy = checkpoint();
        let configuration = legacy.configuration.take().unwrap();
        let navigation = legacy.navigation_configuration.take().unwrap();
        legacy.schema_version = 1;
        legacy.configuration_sha256 = Some([0x37; 32]);
        legacy.navigation_configuration_sha256 = Some([0x91; 32]);
        let payload = serde_json::to_string(&legacy).unwrap();
        let parsed = ExplorationCheckpoint::parse(&payload).unwrap();
        assert_eq!(serde_json::to_string(&parsed).unwrap(), payload);
        assert!(parsed.resume_requires_replan(&configuration, &navigation).unwrap());
        legacy.configuration = Some(configuration.clone());
        legacy.navigation_configuration = Some(navigation.clone());
        let parsed = ExplorationCheckpoint::parse(&serde_json::to_string(&legacy).unwrap()).unwrap();
        assert!(!parsed.resume_requires_replan(&configuration, &navigation).unwrap());
        let mut changed = configuration;
        changed.blacklist_radius_m += 0.5;
        assert!(parsed.resume_requires_replan(&changed, &navigation).is_err());
    }
    #[test]
    fn checkpoint_legacy_replan_retains_home_and_distance_for_current_return_budget() {
        let mut saved = checkpoint();
        let mut mission = nuwax_exploration::mission::MissionProgress::new(MissionPose {
            position: [12.5, -3.25, 2.0], orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }).unwrap();
        mission.travelled_m = 9.0;
        mission.returning = true;
        saved.mission = Some(mission.clone());
        saved.return_reason = Some(nuwax_exploration::ExplorationStopReason::DistanceBudgetReached);
        saved.prepare_history_for_replan();
        let restored = saved.mission.unwrap();
        assert_eq!(restored.home, mission.home);
        assert_eq!(restored.travelled_m, 9.0);
        assert!(!restored.returning);
        assert_eq!(saved.return_reason, None);
        let current_policy = nuwax_exploration::mission::ReturnPolicy {
            maximum_distance_m: Some(10.0), reserve_distance_m: 0.0, return_on_completion: true,
        };
        // The runtime checks the candidate route budget before dispatching a new goal.
        assert!(current_policy.should_return(restored.travelled_m, 2.0).unwrap());
        assert!(!current_policy.can_visit(restored.travelled_m, 0.5, 2.0));
    }
    #[test]
    fn checkpoint_v2_rejects_incomplete_or_invalid_actual_settings() {
        let saved = checkpoint();
        let value = serde_json::to_value(saved).unwrap();
        for field in ["configuration", "navigation_configuration"] {
            let mut invalid = value.clone();
            invalid.as_object_mut().unwrap().remove(field);
            assert!(ExplorationCheckpoint::parse(&invalid.to_string()).is_err());
        }
        let mut invalid = value.clone();
        invalid["navigation_configuration"]["body_to_support_z_m"] = serde_json::json!(-0.5);
        assert!(ExplorationCheckpoint::parse(&invalid.to_string()).is_err());
        let mut invalid = value;
        invalid["configuration"]["profile_version"] = serde_json::json!(2);
        assert!(ExplorationCheckpoint::parse(&invalid.to_string()).is_err());
    }
    pub(crate) fn calibration() -> MissionCalibration {
        MissionCalibration {
            maximum_sample_speed_mps: 2.0,
            planar: SimplePlannerConfig {
                robot_geometry: Some(RobotGeometry {
                    footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                    z_min_m: -0.1,
                    z_max_m: 0.1,
                    source: "路线仿真".into(),
                }),
                ..Default::default()
            },
        }
    }
    fn pose(x: f64, y: f64) -> MissionPose {
        MissionPose {
            position: [x, y, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }
    #[test]
    fn mission_route_measures_wall_detour_and_rejects_unknown_home() {
        let mut grid = OccupancyGrid2D::new(
            Point2::new(0.0, 0.0),
            0.1,
            60,
            60,
            vec![OccupancyCell::Free; 3600],
        );
        for y in 0..45 {
            grid.set(30, y, OccupancyCell::Occupied);
        }
        let route =
            MissionRoutes::planar(grid.clone(), &calibration(), &ExploreStartConfig::default())
                .unwrap();
        let length = route.query(&pose(2.0, 2.0), &pose(4.0, 2.0), home_goal_options(), None).unwrap().length;
        assert!(length > 5.0, "绕墙路线不能退化为2米直线，实际{length}");
        assert_eq!(
            route.query(&pose(2.0, 2.0), &pose(2.0, 2.0), home_goal_options(), None).unwrap().length,
            0.0
        );
        grid.set(40, 20, OccupancyCell::Unknown);
        let route =
            MissionRoutes::planar(grid, &calibration(), &ExploreStartConfig::default()).unwrap();
        assert!(route.query(&pose(2.0, 2.0), &pose(4.05, 2.05), home_goal_options(), None).is_err());
    }
    #[test]
    fn mission_boundary_rejects_concave_notch_through_cell() {
        let boundary = ExplorationBoundary::from_vertices(vec![
            Point2::new(0.0, 0.0),
            Point2::new(4.0, 0.0),
            Point2::new(4.0, 4.0),
            Point2::new(2.6, 4.0),
            Point2::new(2.6, 2.4),
            Point2::new(2.4, 2.4),
            Point2::new(2.4, 4.0),
            Point2::new(0.0, 4.0),
        ])
        .unwrap();
        assert!(!cell_in_boundary(&boundary, Point2::new(2.0, 2.0), 1.0));
        assert!(cell_in_boundary(&boundary, Point2::new(1.0, 1.0), 1.0));
    }
}
