//! 探索预算复用两条生产规划算法；查询只持有不可变地图，不取得运动权。
use super::explore_runtime::{ExploreRobotPose, ExploreStartConfig};
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
    pub configuration_sha256: [u8; 32],
    pub navigation_configuration_sha256: [u8; 32],
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
        if value.schema_version != 1
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

/// 规范请求摘要只包含探索行为，恢复时必须与冻结请求一致。
pub(crate) fn configuration_digest(config: &ExploreStartConfig) -> [u8; 32] {
    let payload = serde_json::json!({"strategy":config.strategy,"arbitration":config.arbitration,
        "boundary":config.boundary.as_ref().map(|b|b.vertices().iter().map(|p|[p.x,p.y]).collect::<Vec<_>>()),
        "return_policy":config.return_policy,"max_navigation_retries":config.max_navigation_retries,"blacklist_radius_m":config.blacklist_radius_m,
        "stop_conditions":{"max_coverage_pct":config.stop_conditions.max_coverage_pct,"no_frontier_timeout_ms":config.stop_conditions.no_frontier_timeout_ms,
            "max_duration_ms":config.stop_conditions.max_duration_ms,"max_goals":config.stop_conditions.max_goals}});
    crate::map_checksum::sha256_bytes(&serde_json::to_vec(&payload).expect("配置已验证"))
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
    pub fn distance(&self, from: &MissionPose, to: &MissionPose) -> Result<f64, String> {
        from.validate()?;
        to.validate()?;
        // 与实际子目标同容差；零距离仍须运行真实可通行查询。
        let options = GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.1,
            yaw_tolerance_rad: 0.2,
        };
        let points = match self {
            Self::Planar { map, config } => {
                let mut planner = SimplePlanner::new(config.clone()).map_err(|e| e.to_string())?;
                planner.replace_costmap(map.clone());
                let path = planner
                    .query_route(pose3(from), pose3(to), options)
                    .map_err(|e| e.to_string())?;
                path.iter()
                    .map(|p| [p.x, p.y, from.position[2]])
                    .collect::<Vec<_>>()
            }
            Self::Surface {
                map,
                config,
                boundary,
            } => nuwax_nav_stack::surface_oriented::plan(
                map,
                (from.position, from.orientation_xyzw),
                (to.position, to.orientation_xyzw),
                options,
                config.follower.motion_limits.as_ref(),
                boundary,
                30_000,
            )?
            .into_iter()
            .map(|p| p.0)
            .collect(),
        };
        if points.is_empty() {
            return Err("返航路线为空".into());
        }
        Ok(points
            .windows(2)
            .map(|p| {
                p[0].iter()
                    .zip(p[1])
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
            })
            .sum())
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
        let length = route.distance(&pose(2.0, 2.0), &pose(4.0, 2.0)).unwrap();
        assert!(length > 5.0, "绕墙路线不能退化为2米直线，实际{length}");
        assert_eq!(
            route.distance(&pose(2.0, 2.0), &pose(2.0, 2.0)).unwrap(),
            0.0
        );
        grid.set(40, 20, OccupancyCell::Unknown);
        let route =
            MissionRoutes::planar(grid, &calibration(), &ExploreStartConfig::default()).unwrap();
        assert!(route.distance(&pose(2.0, 2.0), &pose(4.05, 2.05)).is_err());
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
