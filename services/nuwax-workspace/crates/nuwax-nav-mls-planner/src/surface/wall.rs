//! 墙代价只在配置缓冲距离内传播；局部重算包含完整来源外圈，不借旧边界值续接。
use super::*;
use std::collections::VecDeque;

fn limit(config: &Config) -> f32 {
    (config.wall_clearance_m + config.wall_buffer_m).max(config.voxel_size)
}

#[derive(Clone, Copy)]
struct WallEdge {
    cell: VoxelKey,
    direction: (i32, i32),
}

impl WallEdge {
    fn distance(self, centre: VoxelKey, size: f32) -> f32 {
        let x = (self.cell.0 - centre.0) as f32;
        let y = (self.cell.1 - centre.1) as f32;
        let (dx, dy) = self.direction;
        if dx != 0 {
            (x + dx as f32 * 0.5).hypot((y.abs() - 0.5).max(0.0)) * size
        } else {
            (y + dy as f32 * 0.5).hypot((x.abs() - 0.5).max(0.0)) * size
        }
    }
}

/// 以单元真实边界为障碍位置，计算水平欧氏距离；不把边界格中心当成墙。
fn distances(
    graph: &PlannerGraph,
    domain: &AHashSet<CellId>,
    config: &Config,
    occupied: &ColumnIz,
    heights: &HeightBounds,
) -> AHashMap<CellId, f32> {
    let cells = &graph.cells;
    let size = config.voxel_size;
    let cap = limit(config);
    let ramp_jump = config.max_slope_deg.to_radians().tan() * size + size;
    let height_cells = (config.step_threshold_m.max(ramp_jump) / size).ceil() as i32;
    let mut result: AHashMap<_, _> = domain.iter().map(|&id| (id, cap)).collect();
    let mut queue = VecDeque::new();
    let mut seen = AHashSet::new();
    for &id in domain {
        let cell = cells.coord(id);
        let height = heights
            .get(&cell)
            .map_or((cell.2 as f32 + 1.0) * size, |b| b[1]);
        let mut edges = Vec::new();
        for &(dx, dy) in &DIRECTIONS {
            // 墙边仍按全图邻列判定；domain只限制传播范围。
            let connected = graph
                .surface_lookup
                .get(&(cell.0 + dx, cell.1 + dy))
                .into_iter()
                .flatten()
                .any(|&z| {
                    (z - cell.2).abs() <= height_cells
                        && cells.id((cell.0 + dx, cell.1 + dy, z)).is_some()
                });
            if connected
                || !column_obstructs(
                    occupied,
                    heights,
                    (cell.0 + dx, cell.1 + dy),
                    height,
                    config,
                )
            {
                continue;
            }
            edges.push(WallEdge {
                cell,
                direction: (dx, dy),
            });
        }
        if edges.is_empty() {
            continue;
        }
        // 每个带墙边单元独立搜索，不按其它墙边的距离截断连通路径。
        queue.clear();
        seen.clear();
        queue.push_back(id);
        seen.insert(id);
        while let Some(current_id) = queue.pop_front() {
            let current = cells.coord(current_id);
            let nearest = result.get_mut(&current_id).unwrap();
            for edge in &edges {
                *nearest = nearest.min(edge.distance(current, size));
            }
            for &(dx, dy) in &DIRECTIONS {
                for &z in graph
                    .surface_lookup
                    .get(&(current.0 + dx, current.1 + dy))
                    .into_iter()
                    .flatten()
                {
                    if (z - current.2).abs() > height_cells {
                        continue;
                    }
                    let next = (current.0 + dx, current.1 + dy, z);
                    let Some(next_id) = cells.id(next) else {
                        continue;
                    };
                    let x = (next.0 - cell.0) as f32;
                    let y = (next.1 - cell.1) as f32;
                    if x.hypot(y) * size <= cap + size
                        && domain.contains(&next_id)
                        && seen.insert(next_id)
                    {
                        queue.push_back(next_id);
                    }
                }
            }
        }
    }
    result
}
/// Original occupied heights, independent of terrain graph connectivity.
pub(super) fn column_obstructs(occupied:&ColumnIz,heights:&HeightBounds,
    column:(i32,i32),ground:f32,config:&Config)->bool {
    occupied.get(&column).is_some_and(|levels|levels.iter().any(|&z| {
        let [low,high]=heights.get(&(column.0,column.1,z)).copied()
            .unwrap_or([z as f32*config.voxel_size,(z as f32+1.0)*config.voxel_size]);
        high>ground+1e-6 && low<ground+config.robot_height-1e-6
    }))
}

fn scale(graph: &mut PlannerGraph, config: &Config, ids: impl IntoIterator<Item = CellId>) {
    for id in ids {
        let source = penalty_of(
            graph.wall_state.dist[id as usize],
            config.wall_clearance_m,
            config.wall_buffer_m,
            config.wall_buffer_weight,
        );
        for edge in graph.cells.edges_mut(id) {
            let target = penalty_of(
                graph.wall_state.dist[edge.dest as usize],
                config.wall_clearance_m,
                config.wall_buffer_m,
                config.wall_buffer_weight,
            );
            edge.cost =
                edge.base_cost * (source + target) / 2.0 + config.step_penalty_weight * edge.rise;
        }
    }
}
pub(super) fn full(graph: &mut PlannerGraph, config: &Config,occupied:&ColumnIz,heights:&HeightBounds) {
    let cap = limit(config);
    let domain = graph.cells.ids().collect::<AHashSet<_>>();
    let distances = distances(graph, &domain, config,occupied,heights);
    graph.wall_state.dist = vec![cap; graph.cells.slot_capacity()];
    for (id, distance) in distances {
        graph.wall_state.dist[id as usize] = distance;
    }
    scale(graph, config, domain);
}
fn expand(columns: &AHashSet<(i32, i32)>, radius: i32) -> AHashSet<(i32, i32)> {
    let mut out = AHashSet::new();
    for &(x, y) in columns {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                out.insert((x + dx, y + dy));
            }
        }
    }
    out
}
fn ids(graph: &PlannerGraph, columns: &AHashSet<(i32, i32)>) -> AHashSet<CellId> {
    columns
        .iter()
        .flat_map(|&(x, y)| {
            graph
                .surface_lookup
                .get(&(x, y))
                .into_iter()
                .flatten()
                .filter_map(move |&z| graph.cells.id((x, y, z)))
        })
        .collect()
}
/// 返回局部可能改变边代价的列；外圈提供完整墙边来源。
pub(super) fn update(
    graph: &mut PlannerGraph,
    config: &Config,
    changed: &AHashSet<(i32, i32)>,
    occupied: &ColumnIz,
    heights: &HeightBounds,
) -> (AHashSet<(i32, i32)>, bool) {
    let cap = limit(config);
    let radius = ((cap / config.voxel_size).ceil() + 1.0) as i32;
    let inner = expand(changed, radius);
    let outer = expand(changed, radius * 2);
    let domain = ids(graph, &outer);
    let distances = distances(graph, &domain, config, occupied, heights);
    graph
        .wall_state
        .dist
        .resize(graph.cells.slot_capacity(), cap);
    for id in ids(graph, &inner) {
        graph.wall_state.dist[id as usize] = *distances.get(&id).unwrap_or(&cap);
    }
    // 边代价取两端，额外一列覆盖所有进入内圈的有向边。
    let costs = expand(changed, radius + 1);
    let cost_ids = ids(graph, &costs);
    scale(graph, config, cost_ids);
    (costs, false)
}
