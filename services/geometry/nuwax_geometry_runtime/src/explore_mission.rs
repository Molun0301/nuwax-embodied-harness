//! 探索返航复用原有二维和三维路线查询，不持有会话或运动权限。
use crate::explorer::ExploreStartConfig;
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
    pub fn capture(
        surface: Option<&SurfaceNavigationConfig>,
        caps: Option<&NavigationCapabilities>,
        geometry: &RobotGeometry,
    ) -> Result<Self, String> {
        let caps = caps.ok_or("返航行程统计需要导航运动标定")?;
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
                robot_geometry: Some(geometry.clone()),
                motion_limits: Some(caps.motion.clone()),
                ..Default::default()
            },
            maximum_sample_speed_mps,
        })
    }
}

/// 与实际子目标派发共用约束，返航始终保留冻结 home 的位置与朝向要求。
pub(crate) fn home_goal_options() -> GoalOptions {
    GoalOptions {
        allow_reverse: false,
        position_tolerance_m: 0.1,
        yaw_tolerance_rad: 0.2,
    }
}

pub(crate) fn frontier_goal_options(
    goal: &nuwax_nav_types_messages::visual::FrontierGoal,
    surface: bool,
    config: Option<&SurfaceNavigationConfig>,
) -> GoalOptions {
    let mut options = GoalOptions::default();
    if surface {
        options.allow_reverse = config
            .and_then(|config| config.follower.kinematics())
            .is_some_and(|motion| motion.max_reverse_mps > 0.0);
        if goal
            .metadata
            .get("goal_orientation_required")
            .is_some_and(|value| value == "false")
        {
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
/// 规划或细化耗尽资源不能当成路线不可达的证据。
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum MissionRouteError {
    Inconclusive(String),
    Failed(String),
}
impl MissionRouteError {
    pub(crate) fn is_inconclusive(&self) -> bool {
        matches!(self, Self::Inconclusive(_))
    }
}
impl From<String> for MissionRouteError {
    fn from(message: String) -> Self {
        Self::Failed(message)
    }
}
impl From<nuwax_nav_stack::surface_oriented::ReferencePlanningFailure> for MissionRouteError {
    fn from(error: nuwax_nav_stack::surface_oriented::ReferencePlanningFailure) -> Self {
        if error.is_inconclusive() {
            Self::Inconclusive(error.to_string())
        } else {
            Self::Failed(error.to_string())
        }
    }
}
impl From<&str> for MissionRouteError {
    fn from(message: &str) -> Self {
        message.to_string().into()
    }
}
impl std::ops::Deref for MissionRouteError {
    type Target = str;
    fn deref(&self) -> &str {
        match self {
            Self::Inconclusive(message) | Self::Failed(message) => message,
        }
    }
}
impl std::fmt::Display for MissionRouteError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(self)
    }
}

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
    ) -> Result<MissionRoute, MissionRouteError> {
        self.query_recording(from, to, options, cancellation, &mut |_| {})
    }

    /// 保留本次成功查询的真实路线，供同一选择事务核对最新输入依赖。
    pub fn query_recording(
        &self,
        from: &MissionPose,
        to: &MissionPose,
        options: GoalOptions,
        cancellation: Option<&std::sync::atomic::AtomicBool>,
        record: &mut impl FnMut(&[([f64; 3], [f64; 4])]),
    ) -> Result<MissionRoute, MissionRouteError> {
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
                    length: path
                        .windows(2)
                        .map(|p| ((p[0].x - p[1].x).powi(2) + (p[0].y - p[1].y).powi(2)).sqrt())
                        .sum(),
                    end: MissionPose {
                        position: [end.x, end.y, from.position[2]],
                        orientation_xyzw: [
                            orientation.x,
                            orientation.y,
                            orientation.z,
                            orientation.w,
                        ],
                    },
                })
            }
            Self::Surface {
                map,
                config,
                boundary,
            } => {
                let path = nuwax_nav_stack::surface_oriented::plan_reference_typed(
                    map,
                    &config.planner,
                    (from.position, from.orientation_xyzw),
                    (to.position, to.orientation_xyzw),
                    options,
                    config.follower.body_to_support_z_m,
                    boundary,
                    cancellation,
                )?;
                let end = path.last().ok_or("返航路线为空")?;
                record(&path);
                Ok(MissionRoute {
                    length: path
                        .windows(2)
                        .map(|p| {
                            p[0].0
                                .iter()
                                .zip(p[1].0)
                                .map(|(a, b)| (a - b).powi(2))
                                .sum::<f64>()
                                .sqrt()
                        })
                        .sum(),
                    end: MissionPose {
                        position: end.0,
                        orientation_xyzw: end.1,
                    },
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
#[path = "explore_mission_tests.rs"]
mod tests;
