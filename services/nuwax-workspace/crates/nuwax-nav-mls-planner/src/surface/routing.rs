//! 前沿与区域代价共用同一有向支撑图和边界约束。
use super::*;

impl SurfaceSnapshot {
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
