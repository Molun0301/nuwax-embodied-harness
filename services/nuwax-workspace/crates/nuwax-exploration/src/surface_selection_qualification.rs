//! 已完整重核候选的提交依据；只比较实际路线和目标依赖，不重新搜索路线。
use super::*;
use nuwax_nav_pct_native::{Band, Cell, Config};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug)]
pub(super) struct NativeSelectionInput {
    pub config: Config,
    pub cells: Vec<Cell>,
    pub bands: Vec<Band>,
}

/// 已完成完整精确路线复核的候选，在最新地图上的实际依赖。
#[derive(Clone, Debug)]
pub struct SurfaceSelectionQualification {
    source: SurfaceSnapshot,
    native: Arc<NativeSelectionInput>,
    terrain_source: nuwax_nav_runtime::native_map::PctTerrainSource,
    frontier: FrontierCandidate,
    goal: FrontierGoal,
    model: Option<nuwax_nav_types_messages::ObservationModel>,
    target_radius: f64,
    targets: Vec<([i32; 3], [f64; 3])>,
    visible: Option<BTreeSet<[i32; 3]>>,
    route_columns: BTreeMap<i64, Vec<(i64, i64)>>,
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
        path: &[[f64; 3]],
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
            route_columns: BTreeMap::new(),
            checked: Arc::new(Mutex::new(None)),
        };
        result.targets = result.target_cells(targets);
        result.include_route(path.iter().copied());
        result.include_route(std::iter::once(start));
        result
    }

    /// 同一计算中的返航、候选往返路线必须一起保留其实际依赖。
    pub fn include_route(&mut self, path: impl IntoIterator<Item = [f64; 3]>) {
        let config = &self.native.config;
        // 原生转移的足迹、坡度/台阶邻列及原净距软代价读取范围。
        let radius = config
            .footprint
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(config.footprint_radius, f64::max)
            + 4.0 * config.resolution
            + 2.0 * config.resolution.max(0.05);
        let endpoint_radius = 11.0 * config.resolution;
        let mut first = None;
        let mut last = None;
        for point in path {
            let xy = [point[0], point[1]];
            first.get_or_insert(xy);
            self.include_window(last.unwrap_or(xy), xy, radius);
            last = Some(xy);
        }
        // 原生端点在正负10格内选择实测层，新近支撑也会改变起止绑定。
        for xy in first.into_iter().chain(last) {
            self.include_window(xy, xy, endpoint_radius);
        }
        for intervals in self.route_columns.values_mut() {
            intervals.sort_unstable();
            let mut merged: Vec<(i64, i64)> = Vec::new();
            for &(start, end) in intervals.iter() {
                if let Some(previous) = merged.last_mut().filter(|previous| start <= previous.1 + 1)
                {
                    previous.1 = previous.1.max(end);
                } else {
                    merged.push((start, end));
                }
            }
            *intervals = merged;
        }
        self.checked = Arc::new(Mutex::new(None));
    }

    fn include_window(&mut self, from: [f64; 2], to: [f64; 2], radius: f64) {
        let size = self.native.config.resolution;
        // 列的闭区间与段相触也纳入；向外保留一格，覆盖格边及坐标舍入。
        let min: [i64; 2] = std::array::from_fn(|axis| {
            ((from[axis].min(to[axis]) - radius) / size).floor() as i64 - 1
        });
        let max: [i64; 2] = std::array::from_fn(|axis| {
            ((from[axis].max(to[axis]) + radius) / size).floor() as i64 + 1
        });
        for x in min[0]..=max[0] {
            self.route_columns
                .entry(x)
                .or_default()
                .push((min[1], max[1]));
        }
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
        let reason = self.check(&current.fork_for_planning());
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
            self.native.config.body_to_ground,
            &self.native.config.boundary,
            self.terrain_source,
        ) {
            Ok(input) => input,
            Err(_) => return Some("selected_route_input_unavailable"),
        };
        let before = &self.native.config;
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
        // 只比较逐段读取列的全部高度，不把路线外接矩形的空角当成依赖。
        let inside = |x: i32, y: i32| {
            self.route_columns
                .get(&i64::from(x))
                .is_some_and(|intervals| {
                    intervals
                        .iter()
                        .any(|&(start, end)| start <= i64::from(y) && i64::from(y) <= end)
                })
        };
        let cell_values = |cells: &[Cell]| {
            let mut values: Vec<_> = cells
                .iter()
                .filter(|c| inside(c.x, c.y))
                .map(|c| (c.x, c.y, c.ground.to_bits(), c.ceiling.to_bits()))
                .collect();
            values.sort_unstable();
            values
        };
        let band_values = |bands: &[Band]| {
            let mut values: Vec<_> = bands
                .iter()
                .filter(|b| inside(b.x, b.y))
                .map(|b| (b.x, b.y, b.low.to_bits(), b.high.to_bits()))
                .collect();
            values.sort_unstable();
            values
        };
        if cell_values(&self.native.cells) != cell_values(&cells)
            || band_values(&self.native.bands) != band_values(&bands)
        {
            return Some("selected_route_geometry");
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
