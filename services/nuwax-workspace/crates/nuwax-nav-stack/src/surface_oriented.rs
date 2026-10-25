//! PCT原生全局路线适配；全局参考通过原SCAN窗口进入局部执行。
use std::sync::{Arc, Mutex, atomic::{AtomicBool}};
use nuwax_nav_mls_planner::SurfaceSnapshot;
use nuwax_nav_runtime::native_map;
use nuwax_nav_types_messages::GoalOptions;

pub type BodyPose = ([f64; 3], [f64; 4]);

struct CachedPlanner {
    source: SurfaceSnapshot,
    boundary: Vec<[f64; 2]>,
    offset: f64,
    /// 原生内核的XY格边长，端点格判断与内核用同一个值。
    resolution: f64,
    planner: nuwax_nav_pct_native::Planner,
}

/// 每个导航拥有方保留一份原生地图，位姿和目标变化只重新搜索。
#[derive(Default)]
pub struct PctPlanner {
    cached: Mutex<Option<CachedPlanner>>,
}
impl PctPlanner {
    pub fn route(&self, map: &SurfaceSnapshot, start: BodyPose, goal: BodyPose,
        options: GoalOptions, boundary: &[[f64; 2]], offset: f64,
        cancelled: Option<&AtomicBool>) -> Result<Vec<BodyPose>, String> {
        let mut cache = self.cached.lock().map_err(|error| error.to_string())?;
        let reusable = cache.as_ref().is_some_and(|current|
            current.source.same_body_query_view(map) && current.boundary == boundary && current.offset == offset);
        if !reusable {
            let (config, cells, bands) = native_map::pct_input(map, offset, boundary)?;
            let resolution = config.resolution;
            let planner = nuwax_nav_pct_native::Planner::from_cells(config, &cells, &bands)?;
            tracing::debug!(target:"navigation.pct", event="navigation.pct.map",
                cells=cells.len(), occupied_bands=bands.len(), "PCT原生输入构建完成");
            *cache = Some(CachedPlanner { source: map.clone(), boundary: boundary.to_vec(), offset,
                resolution, planner });
        }
        let cached = cache.as_mut().expect("原生地图已创建");
        let route = match cancelled {
            Some(flag) => cached.planner.plan_cancellable(start.0, goal.0, flag)?,
            None => cached.planner.plan(start.0, goal.0)?,
        };
        tracing::debug!(elapsed_ms=route.elapsed_ms, expanded=route.expanded,
            points=route.path.len(), "PCT原生全局搜索完成");
        let mut path = route.path;
        if let Some(last) = path.last_mut() {
            *last = goal_bound_endpoint(*last, goal.0, cached.resolution);
        }
        let mut poses: Vec<BodyPose> = path.iter().enumerate().map(|(index, &point)| {
            let next = path.get(index + 1).copied().unwrap_or(point);
            let yaw = (next[1] - point[1]).atan2(next[0] - point[0]);
            (point, [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()])
        }).collect();
        if let Some(first) = poses.first_mut() { *first = start; }
        if options.requires_orientation() {
            if let Some(last) = poses.last_mut() { last.1 = goal.1; }
        }
        Ok(poses)
    }
}

/// 业务目标落在PCT端点所在地面格内（含格边与格角）时，末点XY取业务目标，高度保留PCT由实测地面
/// 推出的机体高度；目标在格外时端点不变，不在缺测处补路。端点格号与原生内核相同，用内核同一个
/// 格边长（由f32体素尺寸换算的值）按`floor(坐标/格边长)`计算。原生`snap_endpoint`遇到等距候选时
/// 保留先遍历到的索引较小的格，格角或格边上的目标会绑定到该格，而`floor`把边界坐标归入索引较大的格，
/// 因此按闭区间判断目标是否在端点格内。
fn goal_bound_endpoint(endpoint: [f64; 3], goal: [f64; 3], resolution: f64) -> [f64; 3] {
    let inside = (0..2).all(|axis| {
        let cell = (endpoint[axis] / resolution).floor();
        goal[axis] >= cell * resolution && goal[axis] <= (cell + 1.0) * resolution
    });
    if inside { [goal[0], goal[1], endpoint[2]] } else { endpoint }
}

/// 复用现有探索计算任务的取消信号，原生搜索逐次展开读取。
pub fn with_owner_cancellation<T>(flag: Arc<AtomicBool>, operation: impl FnOnce() -> T) -> T {
    nuwax_nav_pct_native::with_planning_cancellation(flag, operation)
}

#[derive(Debug)]
pub struct ReferencePlanningFailure(pub String);
impl ReferencePlanningFailure {
    pub fn is_inconclusive(&self) -> bool { self.0.contains("取消") }
}
impl std::fmt::Display for ReferencePlanningFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { formatter.write_str(&self.0) }
}
impl std::error::Error for ReferencePlanningFailure {}

pub fn plan_reference_typed(map: &SurfaceSnapshot, planner: &PctPlanner,
    start: BodyPose, goal: BodyPose, options: GoalOptions, offset: f64, boundary: &[[f64; 2]],
    cancelled: Option<&AtomicBool>) -> Result<Vec<BodyPose>, ReferencePlanningFailure> {
    planner.route(map, start, goal, options, boundary, offset, cancelled).map_err(ReferencePlanningFailure)
}

pub fn plan_reference_cancellable(map: &SurfaceSnapshot, planner: &PctPlanner,
    start: BodyPose, goal: BodyPose, options: GoalOptions, offset: f64, boundary: &[[f64; 2]],
    cancelled: Option<&AtomicBool>) -> Result<Vec<BodyPose>, String> {
    plan_reference_typed(map, planner, start, goal, options, offset, boundary, cancelled).map_err(|error| error.to_string())
}

pub fn plan(map: &SurfaceSnapshot, start: BodyPose, goal: BodyPose,
    options: GoalOptions, offset: f64, boundary: &[[f64; 2]]) -> Result<Vec<BodyPose>, String> {
    plan_reference_cancellable(map, &PctPlanner::default(), start, goal, options, offset, boundary, None)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changed_measured_height_refreshes_cached_native_reference_inside_same_voxel() {
        let config = crate::surface_config::SurfaceNavigationConfig::new(
            "map".into(),0.4,0.3,0.6,0.1,0.15,18.0,0.2).unwrap();
        let points = |height| (-20..60).flat_map(|x| (-20..20).map(move |y|
            ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,height))).collect::<Vec<_>>();
        let mut builder = nuwax_nav_mls_planner::SurfaceMapBuilder::new(config.geometry);
        let first = builder.build(&points(0.01),0.05).unwrap().snapshot;
        let planner = PctPlanner::default();
        let start = ([0.0,0.0,0.41],[0.0,0.0,0.0,1.0]);
        let goal = ([1.0,0.0,0.41],start.1);
        let route = planner.route(&first,start,goal,GoalOptions::default(),&[],0.4,None).unwrap();
        let changed = builder.build(&points(0.02),0.05).unwrap().snapshot;
        assert!(!first.same_body_query_view(&changed));
        let refreshed = planner.route(&changed,start,goal,GoalOptions::default(),&[],0.4,None).unwrap();
        assert!(route.iter().skip(1).all(|pose|(pose.0[2]-0.41).abs()<1e-6));
        assert!(refreshed.iter().skip(1).all(|pose|(pose.0[2]-0.42).abs()<1e-6));
    }

    #[test]
    fn route_end_takes_goal_xy_only_inside_the_bound_ground_cell() {
        // 0.1米格的实测平地覆盖x∈[-1,3)、y∈[-1,1)。
        let config = crate::surface_config::SurfaceNavigationConfig::new(
            "map".into(),0.4,0.3,0.6,0.1,0.15,18.0,0.2).unwrap();
        let points = (-20..60).flat_map(|x| (-20..20).map(move |y|
            ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,0.01))).collect::<Vec<_>>();
        let map = nuwax_nav_mls_planner::SurfaceMapBuilder::new(config.geometry).build(&points,0.05).unwrap().snapshot;
        let planner = PctPlanner::default();
        let start = ([0.0,0.0,0.41],[0.0,0.0,0.0,1.0]);
        // 格角目标与四格中心等距，内核绑定其中一格的中心；目标仍在该格边界上，末点取目标XY。
        let corner = planner.route(&map,start,([1.2,0.0,0.41],start.1),GoalOptions::default(),&[],0.4,None).unwrap();
        let end = corner.last().unwrap().0;
        assert_eq!([end[0],end[1]],[1.2,0.0]);
        assert!((end[2]-0.41).abs()<1e-6,"末点高度来自实测地面：{end:?}");
        // 目标在实测地面外，内核绑定到最近的边缘格；目标不在该格内，末点保持格中心，不补路。
        let outside = planner.route(&map,start,([3.2,0.0,0.41],start.1),GoalOptions::default(),&[],0.4,None).unwrap();
        let end = outside.last().unwrap().0;
        assert!((end[0]-2.95).abs()<1e-6 && end[1]!=0.0,"末点应为边缘格中心：{end:?}");
        assert!((end[2]-0.41).abs()<1e-6);
    }
}
