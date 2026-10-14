//! 前沿与区域代价共用同一有向支撑图和边界约束。
use super::*;

/// 同一不可变有向支撑图上的目标距离场，只引导搜索，不授予机体通行许可。
#[derive(Clone)]
pub struct SurfaceGoalCosts {
    costs: Arc<AHashMap<VoxelKey, f64>>,
}

impl SurfaceGoalCosts {
    /// 返回支撑格到最近目标格的有向代价；不连通时为空。
    pub fn cost(&self, cell: VoxelKey) -> Option<f64> {
        self.costs.get(&cell).copied()
    }
}

/// 几何拥有方只保留最近一个目标场及八组区域间代价；不保留旧地图引用。
#[derive(Default)]
pub(super) struct RoutingCache {
    goal: Option<CachedGoalCosts>,
    travel: std::collections::VecDeque<CachedTravelCosts>,
    pub(super) centering: Option<Arc<Vec<f64>>>,
    #[cfg(test)]
    pub(super) centering_builds: usize,
}
struct CachedGoalCosts {
    targets: Vec<CellId>,
    boundary: Vec<[f64; 2]>,
    costs: SurfaceGoalCosts,
}
struct CachedTravelCosts {
    start: CellId,
    targets: Vec<Option<CellId>>,
    boundary: Vec<[f64; 2]>,
    costs: Vec<f64>,
}

impl SurfaceSnapshot {
    /// 读取近墙精确软项与同层全域居中项的较大值；均有限，不参与硬通行许可。
    pub fn soft_traversal_multiplier(&self, cell: VoxelKey) -> f64 {
        let config=self.config();
        if config.wall_buffer_m<=0.0 || config.wall_buffer_weight<=0.0 {return 1.0;}
        let Some(id)=self.data.graph.cells.id(cell) else {return 1.0;};
        let distance=self.data.graph.wall_state.dist.get(id as usize).copied()
            .unwrap_or(config.wall_clearance_m+config.wall_buffer_m);
        // 即使调用者位于旧点图硬边缘，软项也只排序，不能返回无穷大关闭真实机体通道。
        let near=f64::from(penalty_of(distance.max(config.wall_clearance_m),config.wall_clearance_m,
            config.wall_buffer_m,config.wall_buffer_weight));
        let near=if near.is_finite() {near}else{1.0+f64::from(config.wall_buffer_weight)};
        near.max(centering::tail_multiplier(self,cell))
    }

    /// 从多个目标反向计算代价，保留上下阶的原有方向和边界。
    pub fn goal_travel_costs(
        &self,
        targets: &[[f64; 3]],
        boundary: &[[f64; 2]],
    ) -> Result<SurfaceGoalCosts, SurfaceError> {
        validate_boundary(boundary)?;
        let cells = &self.data.graph.cells;
        let mut target_ids: Vec<_> = targets
            .iter()
            .filter_map(|&target| {
                if !inside_boundary([target[0], target[1]], boundary) {
                    return None;
                }
                self.locate_support(target)
                    .ok()
                    .and_then(|location| cells.id(location.cell))
            })
            .collect();
        target_ids.sort_unstable();
        target_ids.dedup();
        if let Some(cached) = self
            .data
            .routing_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .goal
            .as_ref()
            .filter(|cached| cached.targets == target_ids && cached.boundary == boundary)
        {
            return Ok(cached.costs.clone());
        }
        let root = cells.slot_capacity();
        // 连续反向边避免每个支撑格各分配一个 Vec；边界为空时无需提取世界坐标。
        let mut offsets = vec![0_usize; root + 2];
        for (_, edges) in cells.iter() {
            for edge in edges.iter().filter(|edge| edge.cost.is_finite()) {
                offsets[edge.dest as usize + 1] += 1;
            }
        }
        offsets[root + 1] = target_ids.len();
        for i in 1..offsets.len() {
            offsets[i] += offsets[i - 1];
        }
        let mut incoming = vec![(0_u32, 0.0_f32); offsets[root + 1]];
        let mut cursor = offsets[..=root].to_vec();
        for (id, edges) in cells.iter() {
            for edge in edges.iter().filter(|edge| edge.cost.is_finite()) {
                let dest = edge.dest as usize;
                incoming[cursor[dest]] = (id, edge.cost);
                cursor[dest] += 1;
            }
        }
        for &id in &target_ids {
            incoming[cursor[root]] = (id, 0.0);
            cursor[root] += 1;
        }
        let distances =
            nuwax_explore_routing::shortest_costs(root + 1, root, None, |id, output| {
                for &(next, cost) in &incoming[offsets[id]..offsets[id + 1]] {
                    if !boundary.is_empty() && id != root {
                        let from = self.cell_position(cells.coord(next));
                        let to = self.cell_position(cells.coord(id as CellId));
                        if !segment_inside_boundary([from[0], from[1]], [to[0], to[1]], boundary) {
                            continue;
                        }
                    }
                    output.push((next as usize, f64::from(cost)));
                }
            });
        let result = SurfaceGoalCosts {
            costs: Arc::new(
                cells
                    .ids()
                    .filter_map(|id| {
                        let cost = distances[id as usize];
                        cost.is_finite().then_some((cells.coord(id), cost))
                    })
                    .collect(),
            ),
        };
        let previous = self
            .data
            .routing_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .goal
            .replace(CachedGoalCosts {
                targets: target_ids,
                boundary: boundary.to_vec(),
                costs: result.clone(),
            });
        // 旧距离场的最后一个引用可能释放整张表，放在拥有方锁之外。
        drop(previous);
        Ok(result)
    }

    /// 在与前沿查询相同的有向安全图上，一次计算到多处支撑点的路径长度。
    /// 不可达目标返回无穷，非法输入返回错误；不能把直线距离当作绕障代价。
    pub fn frontier_travel_costs(
        &self,
        start: [f64; 3],
        targets: &[[f64; 3]],
        boundary: &[[f64; 2]],
    ) -> Result<Vec<f64>, SurfaceError> {
        validate_boundary(boundary)?;
        let location = self.locate_support(start)?;
        if !inside_boundary([start[0], start[1]], boundary) {
            return Err(SurfaceError::InvalidBoundary);
        }
        let cells = &self.data.graph.cells;
        let start_id = cells
            .id(location.cell)
            .ok_or(SurfaceError::MissingSupport)?;
        let goals: Vec<_> = targets
            .iter()
            .map(|&target| {
                if !inside_boundary([target[0], target[1]], boundary) {
                    return None;
                }
                self.locate_support(target)
                    .ok()
                    .and_then(|goal| cells.id(goal.cell).map(|id| (id, goal.cell)))
            })
            .collect();
        let target_ids: Vec<_> = goals.iter().map(|goal| goal.map(|(id, _)| id)).collect();
        let cached = self
            .data
            .routing_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .travel
            .iter()
            .find(|entry| {
                entry.start == start_id && entry.targets == target_ids && entry.boundary == boundary
            })
            .map(|entry| entry.costs.clone());
        let costs = cached.unwrap_or_else(|| {
            let ids: Vec<_> = target_ids.iter().flatten().map(|&id| id as usize).collect();
            let distances = self.frontier_distances(start_id, boundary, Some(&ids));
            let costs: Vec<_> = target_ids
                .iter()
                .map(|id| id.map_or(f64::INFINITY, |id| distances[id as usize]))
                .collect();
            let mut cache = self
                .data
                .routing_cache
                .lock()
                .unwrap_or_else(|e| e.into_inner());
            // 区域调度最多八组；只保留所请求的目标值，不缓存八份全图距离。
            if cache.travel.len() >= 8 {
                cache.travel.pop_front();
            }
            cache.travel.push_back(CachedTravelCosts {
                start: start_id,
                targets: target_ids,
                boundary: boundary.to_vec(),
                costs: costs.clone(),
            });
            costs
        });
        let start_offset = frontier_point_distance(start, self.cell_position(location.cell));
        Ok(goals
            .iter()
            .zip(targets)
            .zip(costs)
            .map(|((goal, target), cost)| {
                goal.map_or(f64::INFINITY, |(_, cell)| {
                    cost + start_offset + frontier_point_distance(*target, self.cell_position(cell))
                })
            })
            .collect())
    }

    pub(super) fn frontier_distances(
        &self,
        start: CellId,
        boundary: &[[f64; 2]],
        targets: Option<&[usize]>,
    ) -> Vec<f64> {
        let cells = &self.data.graph.cells;
        nuwax_explore_routing::shortest_costs(
            cells.slot_capacity(),
            start as usize,
            targets,
            |id, output| {
                if boundary.is_empty() {
                    output.extend(
                        cells
                            .neighbors(id as CellId)
                            .iter()
                            .filter(|edge| edge.cost.is_finite())
                            .map(|edge| (edge.dest as usize, f64::from(edge.base_cost))),
                    );
                    return;
                }
                let from = self.cell_position(cells.coord(id as CellId));
                for edge in cells.neighbors(id as CellId) {
                    if !edge.cost.is_finite() {
                        continue;
                    }
                    let to = self.cell_position(cells.coord(edge.dest));
                    if segment_inside_boundary([from[0], from[1]], [to[0], to[1]], boundary) {
                        output.push((edge.dest as usize, f64::from(edge.base_cost)));
                    }
                }
            },
        )
    }
}

pub(super) fn frontier_point_distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> SurfaceSnapshot {
        let config = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.25,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.01,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 0.0,
            step_threshold_m: 0.21,
            max_slope_deg: 18.0,
            min_step_tread_depth_m: 0.2,
            goal_tolerance: 0.1,
            step_penalty_weight: 1.0,
            viz_publish_hz: 0.0,
        };
        let points: Vec<_> = (0..40)
            .flat_map(|x| {
                (0..16).map(move |y| {
                    (
                        (x as f32 + 0.5) * 0.1,
                        (y as f32 + 0.5) * 0.1,
                        if x < 20 { 0.025 } else { 0.175 },
                    )
                })
            })
            .collect();
        SurfaceSnapshot::from_points(&points, config, 0.05)
            .unwrap()
            .with_body_envelope(SurfaceBodyEnvelope {
                footprint_xy_m: vec![[-0.1, -0.08], [0.1, -0.08], [0.1, 0.08], [-0.1, 0.08]],
                z_bounds_m: [-0.05, 0.1],
                body_to_support_z_m: 0.25,
                max_step_up_m: 0.1,
                max_step_down_m: 0.2,
                max_cross_slope_deg: 18.0,
                support_uncertainty_m: 0.0,
            })
            .unwrap()
    }

    #[test]
    fn cached_reverse_costs_preserve_direction_boundary_and_geometry() {
        let map = map();
        let low = map.locate_support([1.05, 0.85, 0.05]).unwrap();
        let high = map.locate_support([3.05, 0.85, 0.2]).unwrap();
        let first = map.goal_travel_costs(&[low.support_xyz], &[]).unwrap();
        assert!(first.cost(high.cell).is_some(), "允许从高台下行");
        let same = map
            .goal_travel_costs(&[low.support_xyz, low.support_xyz], &[])
            .unwrap();
        assert!(Arc::ptr_eq(&first.costs, &same.costs));
        let observed =
            map.with_observations(Some(Arc::new(SurfaceObservations::new(0.05, 32).unwrap())));
        assert!(Arc::ptr_eq(
            &first.costs,
            &observed
                .goal_travel_costs(&[low.support_xyz], &[])
                .unwrap()
                .costs
        ));
        let reverse = map.goal_travel_costs(&[high.support_xyz], &[]).unwrap();
        assert!(reverse.cost(low.cell).is_none(), "不能把可下行误作可上行");
        for start in [low.support_xyz, high.support_xyz] {
            for target in [low.support_xyz, high.support_xyz] {
                let expected = map.frontier_travel_costs(start, &[target], &[]).unwrap()[0];
                let start_location = map.locate_support(start).unwrap();
                let target_location = map.locate_support(target).unwrap();
                let offset = frontier_point_distance(start, map.cell_position(start_location.cell))
                    + frontier_point_distance(target, map.cell_position(target_location.cell));
                let actual = map
                    .goal_travel_costs(&[target], &[])
                    .unwrap()
                    .cost(start_location.cell);
                assert_eq!(actual.is_some(), expected.is_finite());
                if let Some(actual) = actual {
                    let graph=&map.data.graph;
                    let start_id=graph.cells.id(start_location.cell).unwrap();
                    let target_id=graph.cells.id(target_location.cell).unwrap();
                    let weighted=nuwax_explore_routing::shortest_costs(graph.cells.slot_capacity(),start_id as usize,None,
                        |id,out|out.extend(graph.cells.neighbors(id as CellId).iter().filter(|edge|edge.cost.is_finite())
                            .map(|edge|(edge.dest as usize,f64::from(edge.cost)))));
                    assert!((actual-weighted[target_id as usize]).abs()<1e-7,
                        "目标场消费实际加权边，不能与未加权前沿路长混为一谈");
                    assert!(actual+offset>=expected-1e-7);
                }
            }
        }
        let boundary = [[0.1, 0.1], [1.9, 0.1], [1.9, 1.5], [0.1, 1.5]];
        let bounded = map
            .goal_travel_costs(&[low.support_xyz], &boundary)
            .unwrap();
        assert!(bounded.cost(high.cell).is_none());
        let changed = map
            .with_virtual_obstacles([0., 0.], 0.1, 40, 16, &vec![true; 640])
            .unwrap();
        assert!(changed
            .goal_travel_costs(&[low.support_xyz], &[])
            .unwrap()
            .cost(high.cell)
            .is_none());
        assert!(first.cost(high.cell).is_some(), "旧场不受新几何修改影响");
    }

    #[test]
    fn regional_cost_cache_retains_exact_endpoint_offsets_and_boundary() {
        let map = map();
        let start = [0.85, 0.85, 0.05];
        let goal = [1.55, 0.85, 0.05];
        let first = map.frontier_travel_costs(start, &[goal], &[]).unwrap()[0];
        let shifted = [start[0] + 0.02, start[1], start[2]];
        let shifted_goal = [goal[0], goal[1] + 0.01, goal[2]];
        let actual = map
            .frontier_travel_costs(shifted, &[shifted_goal], &[])
            .unwrap()[0];
        assert!(
            (actual - first - 0.03).abs() < 1e-6,
            "同格查询仍计入真实端点偏移"
        );
        assert_eq!(map.data.routing_cache.lock().unwrap().travel.len(), 1);
        let boundary = [[0.1, 0.1], [1.2, 0.1], [1.2, 1.5], [0.1, 1.5]];
        assert!(!map
            .frontier_travel_costs(start, &[goal], &boundary)
            .unwrap()[0]
            .is_finite());
    }
    #[test]
    fn soft_goal_costs_match_forward_weighted_edges_and_preserve_reachability() {
        let base=map();let mut config=base.config().clone();
        config.wall_clearance_m=0.0;config.wall_buffer_m=0.45;config.wall_buffer_weight=2.0;
        let points:Vec<_>=(0..40).flat_map(|x|(0..20).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        let preferred=SurfaceSnapshot::from_points(&points,config.clone(),0.1).unwrap();
        config.wall_buffer_weight=0.0;
        let plain=SurfaceSnapshot::from_points(&points,config,0.1).unwrap();
        assert_eq!(preferred.cell_keys().collect::<AHashSet<_>>(),plain.cell_keys().collect::<AHashSet<_>>());
        let edge=preferred.locate_support([0.55,0.15,0.1]).unwrap();
        let centre=preferred.locate_support([0.55,0.95,0.1]).unwrap();
        assert!(preferred.soft_traversal_multiplier(edge.cell)>preferred.soft_traversal_multiplier(centre.cell));
        let target=preferred.locate_support([3.05,0.95,0.1]).unwrap();
        let goal=preferred.goal_travel_costs(&[target.support_xyz],&[]).unwrap();
        let graph=&preferred.data.graph;
        let target_id=graph.cells.id(target.cell).unwrap();
        for from in [edge,centre] {
            let start_id=graph.cells.id(from.cell).unwrap();
            let distances=nuwax_explore_routing::shortest_costs(graph.cells.slot_capacity(),start_id as usize,None,
                |id,out|out.extend(graph.cells.neighbors(id as CellId).iter().filter(|edge|edge.cost.is_finite())
                    .map(|edge|(edge.dest as usize,f64::from(edge.cost)))));
            let cost=goal.cost(from.cell).unwrap();
            assert!((cost-distances[target_id as usize]).abs()<1e-7,
                "反向目标场必须与正向实际加权边一致，不能退回裸距离");
            assert!(cost>=plain.goal_travel_costs(&[target.support_xyz],&[]).unwrap().cost(from.cell).unwrap());
            assert_eq!(preferred.neighbors(from.cell),plain.neighbors(from.cell));
        }
    }

}
