//! 分区有向连通图：保存真实跨区入口，查询时只细化经过区域的完整支撑图。
use super::*;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

const SIDE: i32 = 32;
type TileKey = (i32, i32);
type RegionId = CellId;

#[derive(Clone, Copy)]
struct Portal {
    from: CellId,
    to: CellId,
    target: RegionId,
    cost: f32,
}
#[derive(Default)]
struct Tile {
    region: AHashMap<CellId, RegionId>,
    representatives: AHashMap<RegionId, VoxelKey>,
}
/// 分区和入口独立共享；修改一个区域不会克隆远场的细粒度索引。
#[derive(Default, Clone)]
pub(super) struct RegionGraph {
    tiles: AHashMap<TileKey, Arc<Tile>>,
    outgoing: AHashMap<TileKey, Arc<AHashMap<RegionId, Vec<Portal>>>>,
}
#[derive(Default, Debug, Clone, Copy)]
pub(super) struct QueryStats {
    pub regions: usize,
    pub fine_cells: usize,
}

fn tile((x, y, _): VoxelKey) -> TileKey {
    (x.div_euclid(SIDE), y.div_euclid(SIDE))
}
fn horizontal_distance(a: VoxelKey, b: VoxelKey, size: f32) -> f32 {
    ((a.0 as f64 - b.0 as f64).hypot(a.1 as f64 - b.1 as f64) * f64::from(size)) as f32
}
#[derive(Clone, Copy)]
struct Queue(f32, CellId);
impl PartialEq for Queue {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits() && self.1 == other.1
    }
}
impl Eq for Queue {}
impl PartialOrd for Queue {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Queue {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .total_cmp(&self.0)
            .then_with(|| other.1.cmp(&self.1))
    }
}

impl RegionGraph {
    pub(super) fn full(graph: &PlannerGraph, size: f32) -> Self {
        let mut groups = AHashMap::<TileKey, Vec<CellId>>::new();
        for id in graph.cells.ids() {
            groups
                .entry(tile(graph.cells.coord(id)))
                .or_default()
                .push(id);
        }
        let mut result = Self::default();
        for (key, ids) in groups {
            result
                .tiles
                .insert(key, Arc::new(build_tile(&graph.cells, &ids)));
        }
        let keys = result.tiles.keys().copied().collect::<Vec<_>>();
        for key in keys {
            result.rebuild_links(key, &graph.cells, size);
        }
        result
    }
    pub(super) fn updated(
        &self,
        graph: &PlannerGraph,
        changed: &AHashSet<TileKey>,
        size: f32,
    ) -> Self {
        let mut next = self.clone();
        for &(tx, ty) in changed {
            let mut ids = Vec::new();
            for x in tx * SIDE..(tx + 1) * SIDE {
                for y in ty * SIDE..(ty + 1) * SIDE {
                    if let Some(zs) = graph.surface_lookup.get(&(x, y)) {
                        for &z in zs {
                            if let Some(id) = graph.cells.id((x, y, z)) {
                                ids.push(id);
                            }
                        }
                    }
                }
            }
            if ids.is_empty() {
                next.tiles.remove(&(tx, ty));
                next.outgoing.remove(&(tx, ty));
            } else {
                next.tiles
                    .insert((tx, ty), Arc::new(build_tile(&graph.cells, &ids)));
            }
        }
        let mut links = changed.clone();
        for &(x, y) in changed {
            for (dx, dy) in DIRECTIONS {
                links.insert((x + dx, y + dy));
            }
        }
        for key in links {
            next.rebuild_links(key, &graph.cells, size);
        }
        next
    }
    pub(super) fn tiles_for_columns(
        columns: impl IntoIterator<Item = (i32, i32)>,
    ) -> AHashSet<TileKey> {
        columns
            .into_iter()
            .map(|(x, y)| (x.div_euclid(SIDE), y.div_euclid(SIDE)))
            .collect()
    }
    fn region(&self, cells: &SurfaceCells, id: CellId) -> Option<RegionId> {
        self.tiles
            .get(&tile(cells.coord(id)))?
            .region
            .get(&id)
            .copied()
    }
    fn rebuild_links(&mut self, key: TileKey, cells: &SurfaceCells, size: f32) {
        let Some(own) = self.tiles.get(&key) else {
            self.outgoing.remove(&key);
            return;
        };
        let mut best = AHashMap::<(RegionId, RegionId), Portal>::new();
        for (&from, &source) in &own.region {
            for edge in cells.neighbors(from).iter().filter(|e| e.cost.is_finite()) {
                let Some(target) = self.region(cells, edge.dest) else {
                    continue;
                };
                if target == source {
                    continue;
                }
                let destination = self
                    .tiles
                    .get(&tile(cells.coord(edge.dest)))
                    .expect("目标区域存在");
                let cost =
                    horizontal_distance(own.representatives[&source], cells.coord(from), size)
                        + edge.cost
                        + horizontal_distance(
                            cells.coord(edge.dest),
                            destination.representatives[&target],
                            size,
                        );
                let portal = Portal {
                    from,
                    to: edge.dest,
                    target,
                    cost,
                };
                best.entry((source, target))
                    .and_modify(|old| {
                        if cost < old.cost
                            || (cost == old.cost
                                && (cells.coord(from), cells.coord(edge.dest))
                                    < (cells.coord(old.from), cells.coord(old.to)))
                        {
                            *old = portal;
                        }
                    })
                    .or_insert(portal);
            }
        }
        let mut links = AHashMap::<RegionId, Vec<Portal>>::new();
        for ((source, _), portal) in best {
            links.entry(source).or_default().push(portal);
        }
        for portals in links.values_mut() {
            portals.sort_by_key(|p| p.target);
        }
        self.outgoing.insert(key, Arc::new(links));
    }
    pub(super) fn path(
        &self,
        cells: &SurfaceCells,
        start: CellId,
        goal: CellId,
        size: f32,
    ) -> Result<(Vec<CellId>, QueryStats), SurfaceError> {
        let source = self.region(cells, start).ok_or(SurfaceError::NoPath)?;
        let target = self.region(cells, goal).ok_or(SurfaceError::NoPath)?;
        let mut stats = QueryStats::default();
        let mut heap = BinaryHeap::from([Queue(0.0, source)]);
        let mut distances = AHashMap::from_iter([(source, 0.0)]);
        let mut predecessors = AHashMap::<RegionId, (RegionId, Portal)>::new();
        let mut found = false;
        while let Some(Queue(cost, id)) = heap.pop() {
            if cost > distances[&id] {
                continue;
            }
            stats.regions += 1;
            if id == target {
                found = true;
                break;
            }
            let Some(links) = self
                .outgoing
                .get(&tile(cells.coord(id)))
                .and_then(|t| t.get(&id))
            else {
                continue;
            };
            for &portal in links {
                let next = cost + portal.cost;
                if next < *distances.get(&portal.target).unwrap_or(&f32::INFINITY) {
                    distances.insert(portal.target, next);
                    predecessors.insert(portal.target, (id, portal));
                    heap.push(Queue(next, portal.target));
                }
            }
        }
        if !found {
            return Err(SurfaceError::NoPath);
        }
        let mut portals = Vec::new();
        let mut cursor = target;
        while cursor != source {
            let &(previous, portal) = predecessors.get(&cursor).ok_or(SurfaceError::NoPath)?;
            portals.push(portal);
            cursor = previous;
        }
        portals.reverse();
        let mut result = vec![start];
        let mut cursor = start;
        for portal in portals {
            let region = self.region(cells, cursor).ok_or(SurfaceError::NoPath)?;
            let (part, visited) = sparse_path(cells, cursor, portal.from, size, |id| {
                self.region(cells, id) == Some(region)
            })?;
            stats.fine_cells += visited;
            result.extend(part.into_iter().skip(1));
            result.push(portal.to);
            cursor = portal.to;
        }
        let (part, visited) = sparse_path(cells, cursor, goal, size, |id| {
            self.region(cells, id) == Some(target)
        })?;
        stats.fine_cells += visited;
        result.extend(part.into_iter().skip(1));
        Ok((result, stats))
    }
}

/// 只为真正访问的细图单元分配距离；可用于有边界约束的完整安全回退。
pub(super) fn sparse_path(
    cells: &SurfaceCells,
    start: CellId,
    goal: CellId,
    size: f32,
    permits: impl Fn(CellId) -> bool,
) -> Result<(Vec<CellId>, usize), SurfaceError> {
    let heuristic = |id| horizontal_distance(cells.coord(id), cells.coord(goal), size);
    let mut heap = BinaryHeap::from([Queue(heuristic(start), start)]);
    let mut distance = AHashMap::from_iter([(start, 0.0)]);
    let mut pred = AHashMap::new();
    let mut visited = 0;
    while let Some(Queue(score, id)) = heap.pop() {
        let cost = distance[&id];
        if score > cost + heuristic(id) {
            continue;
        }
        visited += 1;
        if id == goal {
            let mut result = vec![goal];
            let mut cursor = goal;
            while cursor != start {
                cursor = *pred.get(&cursor).ok_or(SurfaceError::NoPath)?;
                result.push(cursor);
            }
            result.reverse();
            return Ok((result, visited));
        }
        for edge in cells.neighbors(id) {
            if !edge.cost.is_finite() || !permits(edge.dest) {
                continue;
            }
            let next = cost + edge.cost;
            if next < *distance.get(&edge.dest).unwrap_or(&f32::INFINITY) {
                distance.insert(edge.dest, next);
                pred.insert(edge.dest, id);
                heap.push(Queue(next + heuristic(edge.dest), edge.dest));
            }
        }
    }
    Err(SurfaceError::NoPath)
}

fn build_tile(cells: &SurfaceCells, ids: &[CellId]) -> Tile {
    // 两次迭代遍历求强连通分量；保留单向坡阶，避免无向洪泛伪造返程。
    let members: AHashSet<_> = ids.iter().copied().collect();
    let mut reverse = AHashMap::<CellId, Vec<CellId>>::new();
    for &id in ids {
        for edge in cells.neighbors(id) {
            if edge.cost.is_finite() && members.contains(&edge.dest) {
                reverse.entry(edge.dest).or_default().push(id);
            }
        }
    }
    let mut ordered = ids.to_vec();
    ordered.sort_unstable_by_key(|&id| cells.coord(id));
    let mut visited = AHashSet::new();
    let mut finish = Vec::with_capacity(ids.len());
    for &id in &ordered {
        if !visited.insert(id) {
            continue;
        }
        let mut stack = vec![(id, 0)];
        while let Some((node, next)) = stack.last_mut() {
            let edges = cells.neighbors(*node);
            if *next == edges.len() {
                finish.push(*node);
                stack.pop();
                continue;
            }
            let edge = edges[*next];
            *next += 1;
            if edge.cost.is_finite() && members.contains(&edge.dest) && visited.insert(edge.dest) {
                stack.push((edge.dest, 0));
            }
        }
    }
    visited.clear();
    let mut result = Tile::default();
    for &id in finish.iter().rev() {
        if !visited.insert(id) {
            continue;
        }
        let mut component = Vec::new();
        let mut stack = vec![id];
        while let Some(node) = stack.pop() {
            component.push(node);
            for &previous in reverse.get(&node).into_iter().flatten() {
                if visited.insert(previous) {
                    stack.push(previous);
                }
            }
        }
        let mut center = [0.0_f64; 3];
        for &member in &component {
            let (x, y, z) = cells.coord(member);
            center[0] += f64::from(x);
            center[1] += f64::from(y);
            center[2] += f64::from(z);
        }
        for value in &mut center {
            *value /= component.len() as f64;
        }
        let distance = |id| {
            let (x, y, z) = cells.coord(id);
            (f64::from(x) - center[0]).powi(2)
                + (f64::from(y) - center[1]).powi(2)
                + (f64::from(z) - center[2]).powi(2)
        };
        let representative = *component
            .iter()
            .min_by(|&&a, &&b| {
                distance(a)
                    .total_cmp(&distance(b))
                    .then_with(|| cells.coord(a).cmp(&cells.coord(b)))
            })
            .expect("非空分量");
        for member in component {
            result.region.insert(member, representative);
        }
        result
            .representatives
            .insert(representative, cells.coord(representative));
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    fn graph(length: i32) -> PlannerGraph {
        let mut graph = PlannerGraph::new();
        let coordinates: Vec<_> = (0..length)
            .flat_map(|x| (0..8).flat_map(move |y| [(x, y, 0), (x, y, 30)]))
            .collect();
        build_surface_lookup(&coordinates, &mut graph.surface_lookup);
        build_surface_cells(&mut graph.cells, &graph.surface_lookup, 0.1, 1);
        graph
    }
    fn reference(cells: &SurfaceCells, start: CellId, goal: CellId) -> bool {
        let mut stack = vec![start];
        let mut seen = AHashSet::from_iter([start]);
        while let Some(id) = stack.pop() {
            if id == goal {
                return true;
            }
            for e in cells.neighbors(id) {
                if e.cost.is_finite() && seen.insert(e.dest) {
                    stack.push(e.dest);
                }
            }
        }
        false
    }
    fn verify(routes: &RegionGraph, graph: &PlannerGraph) {
        let samples: Vec<_> = [
            (2, 3, 0),
            (17, 3, 0),
            (33, 3, 0),
            (47, 3, 0),
            (62, 3, 0),
            (2, 3, 30),
            (62, 3, 30),
        ]
        .into_iter()
        .map(|c| graph.cells.id(c).unwrap())
        .collect();
        for &from in &samples {
            for &to in &samples {
                let actual = routes.path(&graph.cells, from, to, 0.1);
                assert_eq!(
                    actual.is_ok(),
                    reference(&graph.cells, from, to),
                    "{:?}->{:?}",
                    graph.cells.coord(from),
                    graph.cells.coord(to)
                );
                if let Ok((path, _)) = actual {
                    assert_eq!(path.first(), Some(&from));
                    assert_eq!(path.last(), Some(&to));
                    for pair in path.windows(2) {
                        assert!(graph
                            .cells
                            .neighbors(pair[0])
                            .iter()
                            .any(|e| e.dest == pair[1] && e.cost.is_finite()));
                    }
                }
            }
        }
    }
    #[test]
    fn directed_components_cross_tile_links_and_layer_identity_match_full_graph() {
        let mut graph = graph(64);
        let mut routes = RegionGraph::full(&graph, 0.1);
        verify(&routes, &graph);
        for boundary in [15, 31, 47] {
            let ids = graph.cells.ids().collect::<Vec<_>>();
            for id in ids {
                let from = graph.cells.coord(id);
                let edges = graph
                    .cells
                    .neighbors(id)
                    .iter()
                    .copied()
                    .filter(|e| {
                        !(from.0 == boundary + 1 && graph.cells.coord(e.dest).0 == boundary)
                    })
                    .collect();
                graph.cells.replace_edges(id, edges);
            }
            let dirty = RegionGraph::tiles_for_columns([(boundary, 3), (boundary + 1, 3)]);
            routes = routes.updated(&graph, &dirty, 0.1);
            verify(&routes, &graph);
            verify(&RegionGraph::full(&graph, 0.1), &graph);
        }
    }
    #[test]
    fn local_query_and_unchanged_tiles_are_independent_of_far_map_size() {
        for length in [64, 256, 1024] {
            let mut graph = graph(length);
            let routes = RegionGraph::full(&graph, 0.1);
            let from = graph.cells.id((3, 3, 0)).unwrap();
            let to = graph.cells.id((20, 3, 0)).unwrap();
            let (_, stats) = routes.path(&graph.cells, from, to, 0.1).unwrap();
            assert_eq!(stats.regions, 1);
            assert!(stats.fine_cells <= 32, "{stats:?}");
            let blocked = graph.cells.id((35, 3, 0)).unwrap();
            graph.cells.replace_edges(blocked, Vec::new());
            let next = routes.updated(&graph, &AHashSet::from_iter([(1, 0)]), 0.1);
            assert!(Arc::ptr_eq(&routes.tiles[&(0, 0)], &next.tiles[&(0, 0)]));
            assert!(!Arc::ptr_eq(&routes.tiles[&(1, 0)], &next.tiles[&(1, 0)]));
            assert!(routes.path(&graph.cells, from, to, 0.1).is_ok());
        }
    }
}
