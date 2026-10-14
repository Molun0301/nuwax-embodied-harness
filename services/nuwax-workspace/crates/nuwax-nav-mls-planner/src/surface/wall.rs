//! 墙代价只在配置缓冲距离内传播；局部重算包含完整来源外圈，不借旧边界值续接。
use super::*;
use std::collections::VecDeque;

fn limit(config: &Config) -> f32 {
    (config.wall_clearance_m + config.wall_buffer_m).max(config.voxel_size)
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
    let mut result = AHashMap::with_capacity(domain.len());
    let mut queue = VecDeque::new();
    let mut seen = AHashSet::new();
    for &origin in domain {
        let centre = cells.coord(origin);
        let mut nearest = cap;
        queue.clear();
        seen.clear();
        queue.push_back(origin);
        seen.insert(origin);
        while let Some(id) = queue.pop_front() {
            let cell = cells.coord(id);
            let mut directions = [false; 4];
            // 坡阶不可达与墙/未知空间是两种约束。避墙距离读取真实邻列，不能
            // 将剖面不足或方向性运动限制再次按机器人宽度向内膨胀。
            for (index, &(sx, sy)) in DIRECTIONS.iter().enumerate() {
                for &z in graph
                    .surface_lookup
                    .get(&(cell.0 + sx, cell.1 + sy))
                    .into_iter()
                    .flatten()
                {
                    if (z - cell.2).abs() > height_cells {
                        continue;
                    }
                    let next = (cell.0 + sx, cell.1 + sy, z);
                    let Some(next_id) = cells.id(next) else {
                        continue;
                    };
                    directions[index] = true;
                    let dx = (next.0 - centre.0) as f32;
                    let dy = (next.1 - centre.1) as f32;
                    if dx.hypot(dy) * size <= cap + size
                        && domain.contains(&next_id)
                        && seen.insert(next_id)
                    {
                        queue.push_back(next_id);
                    }
                }
            }
            for (index, &(dx, dy)) in DIRECTIONS.iter().enumerate() {
                let height=heights.get(&cell).map_or((cell.2 as f32+1.0)*size,|b|b[1]);
                if directions[index] || !column_obstructs(occupied,heights,
                    (cell.0+dx,cell.1+dy),height,config) {
                    // Unknown ground and a disconnected floor above/below the
                    // declared body-height interval are not physical walls.
                    continue;
                }
                let x = (cell.0 - centre.0) as f32;
                let y = (cell.1 - centre.1) as f32;
                let distance = if dx != 0 {
                    (x + dx as f32 * 0.5).hypot((y.abs() - 0.5).max(0.0))
                } else {
                    (y + dy as f32 * 0.5).hypot((x.abs() - 0.5).max(0.0))
                } * size;
                nearest = nearest.min(distance);
            }
            if nearest <= size * 0.5 {
                break;
            }
        }
        result.insert(origin, nearest);
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
/// 返回实际可能改变边代价的列；大窗口显式回退完整重算。
pub(super) fn update(
    graph: &mut PlannerGraph,
    config: &Config,
    changed: &AHashSet<(i32, i32)>,
    occupied:&ColumnIz,
    heights:&HeightBounds,
) -> (AHashSet<(i32, i32)>, bool) {
    let cap = limit(config);
    let radius_cells = (cap / config.voxel_size).ceil() + 1.0;
    if radius_cells > 128.0 {
        full(graph, config,occupied,heights);
        return (graph.surface_lookup.keys().copied().collect(), true);
    }
    let radius = radius_cells as i32;
    let work = changed
        .len()
        .saturating_mul((4 * radius as usize + 3).pow(2));
    if work > 500_000 {
        full(graph, config,occupied,heights);
        return (graph.surface_lookup.keys().copied().collect(), true);
    }
    let inner = expand(changed, radius);
    let outer = expand(changed, radius * 2);
    let domain = ids(graph, &outer);
    let distances = distances(graph, &domain, config,occupied,heights);
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
