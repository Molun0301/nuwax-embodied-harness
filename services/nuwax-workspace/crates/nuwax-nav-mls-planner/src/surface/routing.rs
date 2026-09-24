//! 前沿与区域代价共用同一有向支撑图和边界约束。
use super::*;

/// 同一不可变有向支撑图上的目标距离场，只引导搜索，不授予机体通行许可。
pub struct SurfaceGoalCosts {
    costs: AHashMap<VoxelKey, f64>,
}

impl SurfaceGoalCosts {
    /// 返回支撑格到最近目标格的有向代价；不连通时为空。
    pub fn cost(&self, cell: VoxelKey) -> Option<f64> {
        self.costs.get(&cell).copied()
    }
}

impl SurfaceSnapshot {
    /// 从多个目标反向计算代价，保留上下阶的原有方向和边界。
    pub fn goal_travel_costs(
        &self,
        targets: &[[f64; 3]],
        boundary: &[[f64; 2]],
    ) -> Result<SurfaceGoalCosts, SurfaceError> {
        validate_boundary(boundary)?;
        let cells = &self.data.graph.cells;
        let root = cells.slot_capacity();
        let mut incoming = vec![Vec::new(); root + 1];
        for (id, edges) in cells.iter() {
            let from = self.cell_position(cells.coord(id));
            for edge in edges.iter().filter(|edge| edge.cost.is_finite()) {
                let to = self.cell_position(cells.coord(edge.dest));
                if segment_inside_boundary([from[0], from[1]], [to[0], to[1]], boundary) {
                    incoming[edge.dest as usize].push((id as usize, f64::from(edge.base_cost)));
                }
            }
        }
        for &target in targets {
            if !inside_boundary([target[0], target[1]], boundary) {
                continue;
            }
            if let Ok(location) = self.locate_support(target) {
                if let Some(id) = cells.id(location.cell) {
                    incoming[root].push((id as usize, 0.0));
                }
            }
        }
        let distances =
            nuwax_explore_routing::shortest_costs(root + 1, root, None, |id, output| {
                output.extend_from_slice(&incoming[id])
            });
        Ok(SurfaceGoalCosts {
            costs: cells
                .ids()
                .filter_map(|id| {
                    let cost = distances[id as usize];
                    cost.is_finite().then_some((cells.coord(id), cost))
                })
                .collect(),
        })
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
        let ids: Vec<_> = goals.iter().flatten().map(|&(id, _)| id as usize).collect();
        let distances = self.frontier_distances(start_id, boundary, Some(&ids));
        let start_offset = frontier_point_distance(start, self.cell_position(location.cell));
        Ok(goals
            .iter()
            .zip(targets)
            .map(|(goal, target)| {
                goal.map_or(f64::INFINITY, |(id, cell)| {
                    distances[id as usize]
                        + start_offset
                        + frontier_point_distance(*target, self.cell_position(cell))
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
