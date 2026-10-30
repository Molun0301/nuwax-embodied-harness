//! 已完整重核候选的提交依据；在当前输入上核验已选路线和目标。
use super::*;
use nuwax_nav_pct_native::{Config, Planner};
use std::collections::BTreeSet;

/// 已完成完整精确路线复核的候选，在最新地图上的实际依赖。
#[derive(Clone, Debug)]
pub struct SurfaceSelectionQualification {
    source: SurfaceSnapshot,
    native: Arc<Config>,
    terrain_source: nuwax_nav_runtime::native_map::PctTerrainSource,
    frontier: FrontierCandidate,
    goal: FrontierGoal,
    model: Option<nuwax_nav_types_messages::ObservationModel>,
    target_radius: f64,
    targets: Vec<([i32; 3], [f64; 3])>,
    visible: Option<BTreeSet<[i32; 3]>>,
    routes: Vec<([f64; 3], [f64; 3])>,
    checked: Arc<Mutex<Option<(Weak<SurfaceSnapshot>, Option<&'static str>)>>>,
}

impl SurfaceSelectionQualification {
    /// 提交依据只属于实际被选中的同一候选。
    pub fn matches_goal(&self, goal: &FrontierGoal) -> bool {
        &self.goal == goal
    }

    pub(super) fn new(
        source: &SurfaceSnapshot,
        native: &CachedFrontierPlanner,
        frontier: FrontierCandidate,
        goal: &FrontierGoal,
        model: Option<nuwax_nav_types_messages::ObservationModel>,
        targets: &[nuwax_explore_routing::observation::ObservationTarget],
        view: Option<&nuwax_explore_routing::observation::ViewEstimate>,
        start: [f64; 3],
    ) -> Self {
        let size = f64::from(source.config().voxel_size);
        let lookahead =
            (source
                .body_envelope()
                .map_or(f64::from(source.config().wall_clearance_m), |body| {
                    body.radius()
                        .max(f64::from(source.config().wall_clearance_m))
                })
                / size)
                .ceil()
                + 2.0;
        let target_radius = model.as_ref().map_or(lookahead * size + size, |model| {
            model.max_range_m
                + model
                    .sensor_translation_body_m
                    .into_iter()
                    .map(|v| v * v)
                    .sum::<f64>()
                    .sqrt()
        });
        let mut result = Self {
            source: source.clone(),
            native: native.input.clone(),
            terrain_source: native.terrain_source,
            frontier,
            goal: goal.clone(),
            model,
            target_radius,
            targets: Vec::new(),
            visible: view.map(|view| view.cells.clone()),
            routes: vec![(
                start,
                [
                    goal.pose.pose.position.x,
                    goal.pose.pose.position.y,
                    goal.pose.pose.position.z,
                ],
            )],
            checked: Arc::new(Mutex::new(None)),
        };
        result.targets = result.target_cells(targets);
        result
    }

    /// 同一计算中的返航、候选往返路线一起按原查询端点复核。
    pub fn include_route(&mut self, path: impl IntoIterator<Item = [f64; 3]>) {
        let mut points = path.into_iter();
        if let Some(first) = points.next() {
            let route = (first, points.last().unwrap_or(first));
            if !self.routes.contains(&route) {
                self.routes.push(route);
            }
        }
        self.checked = Arc::new(Mutex::new(None));
    }

    fn target_cells(
        &self,
        targets: &[nuwax_explore_routing::observation::ObservationTarget],
    ) -> Vec<([i32; 3], [f64; 3])> {
        let p = self.goal.pose.pose.position;
        let center = if self.model.is_some() {
            [p.x, p.y, p.z]
        } else {
            self.frontier.support_xyz
        };
        targets
            .iter()
            .filter(|target| {
                (0..2)
                    .all(|axis| (target.position[axis] - center[axis]).abs() <= self.target_radius)
            })
            .map(|target| (target.cell, target.position))
            .collect()
    }

    /// 返回最新已安装输入中首先影响所选结果的变化；不刷新来源时刻。
    pub fn invalidation(&self, current: &FrozenSurfaceInput) -> Option<&'static str> {
        let Some(current) = current.snapshot() else {
            return Some("surface_presence");
        };
        let mut checked = self.checked.lock().unwrap();
        if let Some((source, reason)) = checked.as_ref() {
            if source.ptr_eq(&Arc::downgrade(current)) {
                return *reason;
            }
        }
        // 提交核对使用私有查询缓存，保持来源快照的缓存归属。
        let started = std::time::Instant::now();
        let reason = self.check(&current.fork_for_planning());
        tracing::info!(
            event = "exploration.selection.current_input_checked",
            elapsed_ms = started.elapsed().as_secs_f64() * 1000.0,
            routes = self.routes.len(),
            invalidation = reason,
            "已选结果完成当前输入提交资格核对"
        );
        *checked = Some((Arc::downgrade(current), reason));
        reason
    }

    fn check(&self, current: &SurfaceSnapshot) -> Option<&'static str> {
        if self.source.body_envelope() != current.body_envelope()
            || self.source.config().voxel_size != current.config().voxel_size
        {
            return Some("selected_query_parameters");
        }
        let (config, cells, bands) = match nuwax_nav_runtime::native_map::pct_input_for_source(
            current,
            self.native.body_to_ground,
            &self.native.boundary,
            self.terrain_source,
        ) {
            Ok(input) => input,
            Err(_) => return Some("selected_route_input_unavailable"),
        };
        let before = &self.native;
        let parameters = |c: &Config| {
            [
                c.resolution,
                c.point_resolution,
                c.robot_height,
                c.body_to_ground,
                c.max_step,
                c.max_step_down,
                c.max_slope,
                c.footprint_radius,
                c.body_z_min,
                c.body_z_max,
                c.search_margin,
            ]
        };
        if parameters(before) != parameters(&config)
            || before.footprint != config.footprint
            || before.boundary != config.boundary
        {
            return Some("selected_query_parameters");
        }
        // 几何细化只触发当前输入的完整精确查询，数值变化本身不是阻断。
        // 仅查询已选路线端点，不重新排名前沿，也不追加重试。
        let mut planner = match Planner::from_cells(config, &cells, &bands) {
            Ok(planner) => planner,
            Err(_) => return Some("selected_route_input_unavailable"),
        };
        for (index, &(from, to)) in self.routes.iter().enumerate() {
            let route = match planner.plan(from, to) {
                Ok(route) => route,
                Err(_) => return Some("selected_route_blocked"),
            };
            // 所选候选保留独立完整复核的原端点容差；返航沿原完整查询合同。
            if index == 0
                && (route.path.len() < 2
                    || route.path.last().is_none_or(|&end| {
                        distance(end, to) > f64::from(current.config().voxel_size) * 0.25
                    }))
            {
                return Some("selected_route_blocked");
            }
        }
        let boundary = &before.boundary;
        let candidates = match current.frontier_candidates(boundary) {
            Ok(candidates) => candidates,
            Err(_) => return Some("selected_frontier_unavailable"),
        };
        if !candidates.iter().any(|candidate| {
            [candidate.cell.0, candidate.cell.1, candidate.cell.2] == self.frontier.frontier_cell
                && candidate.gain == self.frontier.gain
        }) {
            return Some("selected_frontier_geometry");
        }
        let targets = match current.observation_targets(boundary) {
            Ok(targets) => targets,
            Err(_) => return Some("selected_observation_unavailable"),
        };
        // 已标定候选以下面的实际可见单元核对收益；视域外的全图目标不属于该结论。
        if self.model.is_none() && self.targets != self.target_cells(&targets) {
            return Some("selected_observation_targets");
        }
        let p = self.goal.pose.pose.position;
        let q = self.goal.pose.pose.orientation;
        let (body, rotation) = ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]);
        if current.body_envelope().is_some() && current.validate_body_pose(body, rotation).is_err()
        {
            return Some("selected_body_pose");
        }
        if let Some(model) = &self.model {
            match current.estimate_observation_view(model, body, rotation, &targets) {
                Ok(view) if Some(&view.cells) == self.visible.as_ref() => {}
                _ => return Some("selected_observation_view"),
            }
        }
        None
    }
}
