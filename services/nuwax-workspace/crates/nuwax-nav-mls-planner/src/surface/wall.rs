//! 墙代价只在配置缓冲距离内传播；局部重算包含完整来源外圈，不借旧边界值续接。
use super::*;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Copy)]
struct Queue(f32, CellId);
impl PartialEq for Queue {
    fn eq(&self, b: &Self) -> bool {
        self.0.to_bits() == b.0.to_bits() && self.1 == b.1
    }
}
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Ord for Queue {
    fn cmp(&self, b: &Self) -> Ordering {
        b.0.total_cmp(&self.0).then_with(|| b.1.cmp(&self.1))
    }
}

fn limit(config: &Config) -> f32 {
    (config.wall_clearance_m + config.wall_buffer_m).max(config.voxel_size)
}
fn wall_adjacent(cells: &SurfaceCells, id: CellId) -> bool {
    let c = cells.coord(id);
    let mut mask = 0;
    for e in cells.neighbors(id) {
        let n = cells.coord(e.dest);
        mask |= match (n.0 - c.0, n.1 - c.1) {
            (-1, 0) => 1,
            (1, 0) => 2,
            (0, -1) => 4,
            (0, 1) => 8,
            _ => 0,
        };
    }
    mask != 15
}
fn distances(cells: &SurfaceCells, domain: &AHashSet<CellId>, cap: f32) -> AHashMap<CellId, f32> {
    let mut heap = BinaryHeap::new();
    let mut distances = AHashMap::new();
    for &id in domain {
        if wall_adjacent(cells, id) {
            distances.insert(id, 0.0);
            heap.push(Queue(0.0, id));
        }
    }
    while let Some(Queue(cost, id)) = heap.pop() {
        if cost > *distances.get(&id).unwrap_or(&cap) {
            continue;
        }
        for edge in cells.neighbors(id) {
            let next = cost + edge.base_cost;
            if next < cap
                && domain.contains(&edge.dest)
                && next < *distances.get(&edge.dest).unwrap_or(&cap)
            {
                distances.insert(edge.dest, next);
                heap.push(Queue(next, edge.dest));
            }
        }
    }
    distances
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
pub(super) fn full(graph: &mut PlannerGraph, config: &Config) {
    let cap = limit(config);
    let domain = graph.cells.ids().collect::<AHashSet<_>>();
    let distances = distances(&graph.cells, &domain, cap);
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
) -> (AHashSet<(i32, i32)>, bool) {
    let cap = limit(config);
    let radius_cells = (cap / config.voxel_size).ceil() + 1.0;
    if radius_cells > 128.0 {
        full(graph, config);
        return (graph.surface_lookup.keys().copied().collect(), true);
    }
    let radius = radius_cells as i32;
    let work = changed
        .len()
        .saturating_mul((4 * radius as usize + 3).pow(2));
    if work > 500_000 {
        full(graph, config);
        return (graph.surface_lookup.keys().copied().collect(), true);
    }
    let inner = expand(changed, radius);
    let outer = expand(changed, radius * 2);
    let domain = ids(graph, &outer);
    let distances = distances(&graph.cells, &domain, cap);
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
