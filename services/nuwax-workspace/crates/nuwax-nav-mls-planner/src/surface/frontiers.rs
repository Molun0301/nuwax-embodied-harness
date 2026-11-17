//! 前沿几何索引与局部失效；不缓存机器人可达关系或路径代价。
use super::*;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Mutex;

/// 最近一次实际前沿几何计算工作量；可达图搜索另计。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurfaceFrontierCacheStats {
    /// 实际重新评估视线/支撑的单元数。
    pub evaluated_cells: usize,
    /// 从旧几何索引原样继承的前沿数。
    pub retained_frontiers: usize,
    /// 新索引中的几何前沿总数，可能包含当前机器人不可达的部分。
    pub frontier_count: usize,
    /// 是否使用了局部变化范围。
    pub incremental: bool,
    /// 本次重新连通聚类的前沿格数（脏范围内的新前沿格加受影响簇的原有格）。
    pub reclustered_cells: usize,
    /// 开口宽度不小于机体宽度的簇数。
    pub candidate_clusters: usize,
}

/// 开口宽度不小于机体宽度的前沿簇；簇号只用于同一索引内分组。
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceFrontierCluster {
    /// 簇号。
    pub id: u32,
    /// 簇内全部格两两之间的最大格心平面距离（米）。
    pub span_m: f64,
    /// 开口宽度（米）：簇里紧邻未知列的格两两之间的最大格心平面距离加一个格边长。
    pub opening_m: f64,
    /// 簇收益：簇的格数 × 格面积。
    pub gain: f64,
    /// 簇内前沿格，按格号排序；收益取簇收益，路程未求（无穷）。
    pub frontiers: Vec<SurfaceFrontier>,
}

pub(super) type CacheSlot = Mutex<Option<CachedFrontiers>>;

#[derive(Clone)]
pub(super) struct CachedFrontiers {
    seed: Arc<SurfaceFrontierIndex>,
    dirty: Option<BTreeMap<[i32; 2], DirtyBounds>>,
}

pub struct SurfaceFrontierIndex {
    boundary: Vec<[f64; 2]>,
    /// 达标簇（开口宽度不小于机体宽度）里的前沿格 → 所属簇的收益；选点候选、途中换目标与
    /// `reachable_frontiers` 只读它。
    pub(super) gains: BTreeMap<VoxelKey, f64>,
    /// 全部前沿格（含未达标的簇）→ 所属簇号与是否紧邻未知列。
    cells: BTreeMap<VoxelKey, FrontierCell>,
    /// 簇号 → 簇；增量更新时未受影响的簇共用原对象。
    clusters: BTreeMap<u32, Arc<FrontierClusterCells>>,
    next_cluster: u32,
    stats: SurfaceFrontierCacheStats,
}

#[derive(Clone, Copy)]
struct FrontierCell {
    cluster: u32,
    /// 四邻域里有未知列（簇的开口线）。
    opening: bool,
}

struct FrontierClusterCells {
    cells: Vec<VoxelKey>,
    span_m: f64,
    opening_m: f64,
    candidate: bool,
}

/// 由探索适配器持有的查询结果；仅计算调用者请求的簇。
pub struct SurfaceFrontierQuery {
    index: Arc<SurfaceFrontierIndex>,
    affected: BTreeSet<u32>,
    clusters: BTreeMap<u32, FilteredFrontierCluster>,
}

struct FilteredFrontierCluster {
    changed: bool,
    parts: Vec<Arc<FrontierClusterCells>>,
}

#[derive(Clone, Copy)]
struct DirtyBounds {
    min_x: i64,
    max_x: i64,
    min_y: i64,
    max_y: i64,
}
impl DirtyBounds {
    fn contains(self, cell: VoxelKey) -> bool {
        (self.min_x..=self.max_x).contains(&i64::from(cell.0))
            && (self.min_y..=self.max_y).contains(&i64::from(cell.1))
    }
    fn column_ranges(
        self,
        scan_slots: usize,
    ) -> Option<(std::ops::RangeInclusive<i32>, std::ops::RangeInclusive<i32>)> {
        let min_x = self.min_x.max(i64::from(i32::MIN));
        let max_x = self.max_x.min(i64::from(i32::MAX));
        let min_y = self.min_y.max(i64::from(i32::MIN));
        let max_y = self.max_y.min(i64::from(i32::MAX));
        // 比较实际循环次数；稀疏/远隔变化仍走槽位扫描，避免枚举巨大的空矩形。
        let columns = (max_x - min_x + 1) as u128 * (max_y - min_y + 1) as u128;
        (columns <= scan_slots as u128)
            .then_some((min_x as i32..=max_x as i32, min_y as i32..=max_y as i32))
    }

    fn union(self, other: Self) -> Self {
        Self {
            min_x: self.min_x.min(other.min_x),
            max_x: self.max_x.max(other.max_x),
            min_y: self.min_y.min(other.min_y),
            max_y: self.max_y.max(other.max_y),
        }
    }
}

// 脏范围只可能影响所在块及其半径覆盖的邻块。
fn dirty_contains(blocks: &BTreeMap<[i32; 2], DirtyBounds>, cell: VoxelKey, radius: i64) -> bool {
    let ring = (radius + 31) / 32;
    let block = [i64::from(cell.0.div_euclid(32)), i64::from(cell.1.div_euclid(32))];
    (block[0] - ring..=block[0] + ring).any(|x|
        (block[1] - ring..=block[1] + ring).any(|y|
            blocks.get(&[x as i32, y as i32]).is_some_and(|bounds| bounds.contains(cell))))
}

impl SurfaceSnapshot {
    /// 所选簇在到达半径内的前沿格对应的未知列；只标记这些列，不排除邻近视点。
    pub fn reached_frontier_columns(&self, cells: &[[i32; 3]], position: [f64; 3], radius: f64,
        boundary: &[[f64; 2]]) -> BTreeSet<[i32; 3]> {
        cells.iter().filter(|cell| {
            let p = self.cell_position((cell[0], cell[1], cell[2]));
            (p[0] - position[0]).hypot(p[1] - position[1]).hypot(p[2] - position[2]) <= radius
        }).flat_map(|cell| self.frontier_unknown_cells((cell[0], cell[1], cell[2]), boundary))
            .map(|cell| [cell.0, cell.1, cell.2]).collect()
    }

    /// 由标记列沿原前瞻方向反查可能受影响的簇，不遍历全图前沿。
    pub fn frontier_query(&self, index: Arc<SurfaceFrontierIndex>, reached: &BTreeSet<[i32; 3]>) -> SurfaceFrontierQuery {
        let mut affected = BTreeSet::new();
        for &[x, y, _] in reached {
            for (dx, dy) in DIRECTIONS {
                for distance in 1..=self.frontier_lookahead() {
                    let (x, y) = (x - dx * distance, y - dy * distance);
                    affected.extend(index.cells.range((x, y, i32::MIN)..=(x, y, i32::MAX)).map(|(_, info)| info.cluster));
                }
            }
        }
        SurfaceFrontierQuery { index, affected, clusters: BTreeMap::new() }
    }

    fn filtered_frontier_cluster<'a>(&self, query: &'a mut SurfaceFrontierQuery, id: u32,
        reached: &BTreeSet<[i32; 3]>) -> &'a FilteredFrontierCluster {
        if !query.clusters.contains_key(&id) {
            let original = Arc::clone(&query.index.clusters[&id]);
            let mut changed = false;
            let mut remaining = BTreeMap::new();
            if query.affected.contains(&id) {
                for &cell in &original.cells {
                    let unknown = self.frontier_unknown_cells(cell, &query.index.boundary);
                    changed |= unknown.iter().any(|c| reached.contains(&[c.0, c.1, c.2]));
                    let unknown: Vec<_> = unknown.into_iter().filter(|c| !reached.contains(&[c.0, c.1, c.2])).collect();
                    if !unknown.is_empty() {
                        remaining.insert(cell, unknown.iter().any(|c| (c.0 - cell.0).abs() + (c.1 - cell.1).abs() == 1));
                    }
                }
            }
            let mut parts = Vec::new();
            if !changed { parts.push(original); }
            let size = f64::from(self.config().voxel_size);
            while changed && !remaining.is_empty() {
                let seed = remaining.pop_first().unwrap();
                let mut members = vec![seed];
                let mut cursor = 0;
                while cursor < members.len() {
                    let cell = members[cursor].0;
                    cursor += 1;
                    let mut found = Vec::new();
                    for_each_neighbor(&remaining, cell, self.data.candidate_step_cells, |key, _| found.push(key));
                    members.extend(found.into_iter().map(|key| (key, remaining.remove(&key).unwrap())));
                }
                let opening: Vec<_> = members.iter().filter(|(_, opening)| *opening).map(|(c, _)| [c.0, c.1]).collect();
                let opening_m = if opening.is_empty() { 0.0 } else { planar_diameter(opening.into_iter()) * size + size };
                if opening_m + 1e-9 < self.frontier_body_width() { continue; }
                let span_m = planar_diameter(members.iter().map(|(c, _)| [c.0, c.1])) * size;
                let mut cells: Vec<_> = members.into_iter().map(|(cell, _)| cell).collect();
                cells.sort_unstable();
                parts.push(Arc::new(FrontierClusterCells { cells, span_m, opening_m, candidate: true }));
            }
            query.clusters.insert(id, FilteredFrontierCluster { changed, parts });
        }
        &query.clusters[&id]
    }

    /// 选点请求全部达标簇，复用途中核对已算出的局部结果。
    pub fn frontier_clusters_from_query(&self, query: &mut SurfaceFrontierQuery, reached: &BTreeSet<[i32; 3]>)
        -> Vec<SurfaceFrontierCluster> {
        let index = Arc::clone(&query.index);
        let mut next_id = index.next_cluster;
        let area = f64::from(self.config().voxel_size).powi(2);
        let mut result = Vec::new();
        for (&id, _) in index.clusters.iter().filter(|(_, cluster)| cluster.candidate) {
            let filtered = self.filtered_frontier_cluster(query, id, reached);
            for part in &filtered.parts {
                let gain = part.cells.len() as f64 * area;
                let id = if filtered.changed { let id = next_id; next_id += 1; id } else { id };
                result.push(SurfaceFrontierCluster { id, span_m: part.span_m, opening_m: part.opening_m, gain,
                    frontiers: part.cells.iter().map(|&cell| SurfaceFrontier {
                        cell, support_xyz: self.cell_position(cell), gain, travel_cost: f64::INFINITY,
                    }).collect() });
            }
        }
        result
    }

    /// 独立几何调用也使用同一反查逻辑；生产适配器保留查询对象以便复用。
    pub fn frontier_clusters_with_reached(&self, boundary: &[[f64; 2]], reached: &BTreeSet<[i32; 3]>)
        -> Result<Vec<SurfaceFrontierCluster>, SurfaceError> {
        validate_boundary(boundary)?;
        let mut query = self.frontier_query(self.frontier_index(boundary), reached);
        Ok(self.frontier_clusters_from_query(&mut query, reached))
    }

    /// 观察目标也去掉到过仍未知的列，和过滤后的前沿使用同一个未知列定义。
    pub fn observation_targets_with_reached(&self, boundary: &[[f64; 2]], reached: &BTreeSet<[i32; 3]>)
        -> Result<Vec<nuwax_explore_routing::observation::ObservationTarget>, SurfaceError> {
        Ok(self.observation_targets(boundary)?.into_iter().filter(|target| !reached.contains(&target.cell)).collect())
    }

    /// 途中核对先查询半径内的格，只请求这些格所属簇的过滤结果。
    pub fn frontier_near_from_query(&self, query: &mut SurfaceFrontierQuery, cell: [i32; 3],
        horizontal_m: f64, vertical_m: f64, reached: &BTreeSet<[i32; 3]>) -> bool {
        let center = self.cell_position((cell[0], cell[1], cell[2]));
        let reach = (horizontal_m / f64::from(self.config().voxel_size)).ceil() as i32 + 1;
        let mut nearby = BTreeMap::<u32, Vec<VoxelKey>>::new();
        for x in cell[0].saturating_sub(reach)..=cell[0].saturating_add(reach) {
            for (&key, _) in query.index.gains.range((x, cell[1].saturating_sub(reach), i32::MIN)
                ..=(x, cell[1].saturating_add(reach), i32::MAX)) {
                let p = self.cell_position(key);
                if (p[0] - center[0]).hypot(p[1] - center[1]) <= horizontal_m + 1e-9
                    && (p[2] - center[2]).abs() <= vertical_m + 1e-9 {
                    nearby.entry(query.index.cells[&key].cluster).or_default().push(key);
                }
            }
        }
        nearby.into_iter().any(|(id, cells)| self.filtered_frontier_cluster(query, id, reached).parts.iter()
            .any(|part| cells.iter().any(|cell| part.cells.binary_search(cell).is_ok())))
    }

    /// 在环境快照上准备可复用的前沿几何；当前机体视图只重算局部变化。
    pub fn prepare_frontier_geometry(
        &self,
        boundary: &[[f64; 2]],
    ) -> Result<SurfaceFrontierCacheStats, SurfaceError> {
        validate_boundary(boundary)?;
        Ok(self.frontier_index(boundary).stats)
    }

    /// Exact planning inputs for retaining a frontier search's resource stage.
    /// Observation receipt metadata alone does not restart work; terrain, body,
    /// masks and the independently retained recent-obstacle view still do.
    pub fn same_frontier_query_geometry(&self, other: &Self) -> bool {
        self.same_geometry(other)
            && match (&self.observations, &other.observations) {
                (Some(a), Some(b)) => a.same_geometry(b),
                (None, None) => true,
                _ => false,
            }
            && match (&self.recent_obstacles, &other.recent_obstacles) {
                (Some(a), Some(b)) => Arc::ptr_eq(a, b),
                (None, None) => true,
                _ => false,
            }
    }

    /// 达标前沿簇及其格，只是几何视点种子；可达性由当前规划器核验。
    pub fn frontier_clusters(
        &self,
        boundary: &[[f64; 2]],
    ) -> Result<Vec<SurfaceFrontierCluster>, SurfaceError> {
        validate_boundary(boundary)?;
        let index = self.frontier_index(boundary);
        let area = f64::from(self.config().voxel_size).powi(2);
        Ok(index.clusters.iter().filter(|(_, cluster)| cluster.candidate).map(|(&id, cluster)| {
            let gain = cluster.cells.len() as f64 * area;
            SurfaceFrontierCluster {
                id,
                span_m: cluster.span_m,
                opening_m: cluster.opening_m,
                gain,
                frontiers: cluster.cells.iter().map(|&cell| SurfaceFrontier {
                    support_xyz: self.cell_position(cell),
                    gain,
                    travel_cost: f64::INFINITY,
                    cell,
                }).collect(),
            }
        }).collect())
    }

    /// 某格中心水平`horizontal_m`、竖直`vertical_m`以内是否还有达标簇的前沿格；索引与选点共用，只局部重算变化范围。
    pub fn frontier_near(
        &self,
        cell: [i32; 3],
        horizontal_m: f64,
        vertical_m: f64,
        boundary: &[[f64; 2]],
    ) -> Result<bool, SurfaceError> {
        validate_boundary(boundary)?;
        let center = self.cell_position((cell[0], cell[1], cell[2]));
        let reach = (horizontal_m / f64::from(self.config().voxel_size)).ceil() as i32 + 1;
        let index = self.frontier_index(boundary);
        let columns = (cell[0].saturating_sub(reach), i32::MIN, i32::MIN)
            ..=(cell[0].saturating_add(reach), i32::MAX, i32::MAX);
        Ok(index.gains.range(columns).any(|(&key, _)| {
            let point = self.cell_position(key);
            (point[0] - center[0]).hypot(point[1] - center[1]) <= horizontal_m + 1e-9
                && (point[2] - center[2]).abs() <= vertical_m + 1e-9
        }))
    }

    /// 只读取已完成计算的统计；等待下一次查询的局部更新返回空。
    pub fn frontier_cache_stats(&self) -> Option<SurfaceFrontierCacheStats> {
        self.data
            .frontier_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .as_ref()
            .filter(|cache| cache.dirty.is_none())
            .map(|cache| cache.seed.stats)
    }

    /// 继承同一构建链的前沿索引。`changes`必须列全自旧快照以来变化的物理列。
    /// 仅供拥有完整列差异的构建器调用；配置/禁区不一致则放弃继承。
    /// 按变化块累计局部失效范围，视线计算延后至查询，不保留快照链。
    pub fn inherit_frontier_cache(&self, previous: &Self, changes: &[[i32; 2]]) {
        if Arc::ptr_eq(&self.data, &previous.data)
            || self
                .data
                .frontier_cache
                .lock()
                .unwrap_or_else(|error| error.into_inner())
                .is_some()
        {
            return;
        }
        let Some(mut cache) = previous
            .data
            .frontier_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .clone()
        else {
            return;
        };
        if !self.frontier_inputs_match(previous) {
            return;
        }
        if !changes.is_empty() {
            // 视线前瞻 + 坡阶侧向剖面 + 净空查询共同决定局部依赖范围，覆盖全部高度。
            let radius = self.frontier_dirty_radius();
            let blocks = cache.dirty.get_or_insert_with(BTreeMap::new);
            for &[x, y] in changes {
                let block = [x.div_euclid(32), y.div_euclid(32)];
                let bounds = DirtyBounds {
                    min_x: i64::from(x) - radius, max_x: i64::from(x) + radius,
                    min_y: i64::from(y) - radius, max_y: i64::from(y) + radius,
                };
                blocks.entry(block).and_modify(|old| *old = old.union(bounds)).or_insert(bounds);
            }
        }
        let mut current = self
            .data
            .frontier_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        if current.is_none() {
            *current = Some(cache);
        }
    }

    fn frontier_inputs_match(&self, previous: &Self) -> bool {
        let a = self.config();
        let b = previous.config();
        if self.data.body_envelope != previous.data.body_envelope
            || a.world_frame != b.world_frame
            || self.source_voxel_size_m() != previous.source_voxel_size_m()
            || [
                a.voxel_size,
                a.robot_height,
                a.wall_clearance_m,
                a.wall_buffer_m,
                a.wall_buffer_weight,
                a.step_threshold_m,
                a.max_slope_deg,
                a.min_step_tread_depth_m,
                a.surface_closing_radius,
                a.node_spacing_m,
                a.step_penalty_weight,
                a.goal_tolerance,
            ] != [
                b.voxel_size,
                b.robot_height,
                b.wall_clearance_m,
                b.wall_buffer_m,
                b.wall_buffer_weight,
                b.step_threshold_m,
                b.max_slope_deg,
                b.min_step_tread_depth_m,
                b.surface_closing_radius,
                b.node_spacing_m,
                b.step_penalty_weight,
                b.goal_tolerance,
            ]
        {
            return false;
        }
        match (
            &self.data.virtual_obstacles,
            &previous.data.virtual_obstacles,
        ) {
            (None, None) => true,
            (Some(a), Some(b)) => {
                Arc::ptr_eq(a, b)
                    || (a.origin == b.origin
                        && a.resolution == b.resolution
                        && a.width == b.width
                        && a.height == b.height
                        && a.integral == b.integral)
            }
            _ => false,
        }
    }

    pub fn frontier_index(&self, boundary: &[[f64; 2]]) -> Arc<SurfaceFrontierIndex> {
        let cache = self
            .data
            .frontier_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .clone()
            .filter(|cache| cache.seed.boundary == boundary);
        if let Some(cache) = &cache {
            if cache.dirty.is_none() {
                return Arc::clone(&cache.seed);
            }
        }
        let dirty = cache.as_ref().and_then(|cache| cache.dirty.as_ref());
        let radius = self.frontier_dirty_radius();
        // 继承脏范围外的前沿格；脏范围内原有前沿格所在的簇要重新聚类。
        let mut touched = BTreeSet::new();
        let mut cells: BTreeMap<_, _> = cache.as_ref().map_or_else(BTreeMap::new, |cache| {
            cache.seed.cells.iter()
                .filter(|(cell, info)| {
                    let inside = dirty_contains(dirty.unwrap(), **cell, radius);
                    if inside { touched.insert(info.cluster); }
                    !inside
                })
                .map(|(&cell, &info)| (cell, info))
                .collect()
        });
        let retained_frontiers = cells.len();
        let mut evaluated_cells = 0;
        // 脏范围内重新评估出的前沿格 → 是否紧邻未知列。
        let mut fresh = BTreeMap::new();
        let mut evaluate = |cell: VoxelKey| {
            evaluated_cells += 1;
            let unknown = self.frontier_unknown_cells(cell, boundary);
            if !unknown.is_empty() {
                fresh.insert(cell, unknown.iter().any(|u| (u.0 - cell.0).abs() + (u.1 - cell.1).abs() == 1));
            }
        };
        if let Some(blocks) = dirty {
            let mut affected = BTreeSet::new();
            for bounds in blocks.values() {
                if let Some((xs, ys)) = bounds.column_ranges(self.data.graph.cells.slot_capacity()) {
                    for x in xs {
                        for y in ys.clone() {
                            for &z in self.data.graph.surface_lookup.get(&(x, y)).into_iter().flatten() {
                                affected.insert((x, y, z));
                            }
                        }
                    }
                } else {
                    affected.extend(self.cell_keys().filter(|cell| bounds.contains(*cell)));
                }
            }
            for cell in affected { evaluate(cell); }
        } else {
            for cell in self.cell_keys() { evaluate(cell); }
        }
        let cluster_started = std::time::Instant::now();
        let step = self.data.candidate_step_cells;
        // 与新前沿格相邻的原有簇也受影响（可能合并）。
        for &cell in fresh.keys() {
            for_each_neighbor(&cells, cell, step, |_, info| { touched.insert(info.cluster); });
        }
        let (mut clusters, mut gains, mut next_cluster) = cache.as_ref().map_or_else(
            || (BTreeMap::new(), BTreeMap::new(), 0),
            |cache| (cache.seed.clusters.clone(), cache.seed.gains.clone(), cache.seed.next_cluster));
        // 受影响的簇整体拆散，仍是前沿的格与新前沿格一起重新连通。
        let mut pool = fresh;
        for id in &touched {
            let cluster = clusters.remove(id).expect("前沿格所属的簇都在索引里");
            for cell in &cluster.cells {
                gains.remove(cell);
                if let Some(info) = cells.remove(cell) {
                    pool.insert(*cell, info.opening);
                }
            }
        }
        let reclustered_cells = pool.len();
        let width = self.frontier_body_width();
        let size = f64::from(self.config().voxel_size);
        let area = size.powi(2);
        while let Some((seed, opening)) = pool.pop_first() {
            let mut members = vec![(seed, opening)];
            let mut next = 0;
            while next < members.len() {
                let (cell, _) = members[next];
                next += 1;
                let mut found = Vec::new();
                for_each_neighbor(&pool, cell, step, |neighbor, _| found.push(neighbor));
                members.extend(found.into_iter().map(|neighbor| (neighbor, pool.remove(&neighbor).unwrap())));
            }
            let span_m = planar_diameter(members.iter().map(|(cell, _)| [cell.0, cell.1])) * size;
            let opening_cells: Vec<_> = members.iter().filter(|(_, opening)| *opening).map(|(cell, _)| [cell.0, cell.1]).collect();
            // 簇里没有紧邻未知列的格时开口宽度为零。
            let opening_m = if opening_cells.is_empty() {
                0.0
            } else {
                planar_diameter(opening_cells.into_iter()) * size + size
            };
            let candidate = opening_m + 1e-9 >= width;
            let id = next_cluster;
            next_cluster += 1;
            let mut member_cells: Vec<_> = members.iter().map(|(cell, _)| *cell).collect();
            member_cells.sort_unstable();
            for &(cell, opening) in &members {
                cells.insert(cell, FrontierCell { cluster: id, opening });
                if candidate {
                    gains.insert(cell, member_cells.len() as f64 * area);
                }
            }
            clusters.insert(id, Arc::new(FrontierClusterCells { cells: member_cells, span_m, opening_m, candidate }));
        }
        let cluster_ms = cluster_started.elapsed().as_secs_f64() * 1000.0;
        let index = Arc::new(SurfaceFrontierIndex {
            boundary: boundary.to_vec(),
            stats: SurfaceFrontierCacheStats {
                evaluated_cells,
                retained_frontiers,
                frontier_count: cells.len(),
                incremental: dirty.is_some(),
                reclustered_cells,
                candidate_clusters: clusters.values().filter(|cluster| cluster.candidate).count(),
            },
            gains,
            cells,
            clusters,
            next_cluster,
        });
        *self
            .data
            .frontier_cache
            .lock()
            .unwrap_or_else(|error| error.into_inner()) = Some(CachedFrontiers {
            seed: Arc::clone(&index),
            dirty: None,
        });
        // 仅真实重建发出工作量事件；命中旧索引不重复报告上一次统计。
        tracing::trace!(target: "navigation.surface.frontier.rebuild",
            evaluated_cells, retained_frontiers, incremental = dirty.is_some(), reclustered_cells, cluster_ms,
            "前沿几何索引重建");
        index
    }

    /// 机体宽度：有外形时取外形包络矩形的横向宽度，没有外形时取规划器所用圆的直径（2×`wall_clearance_m`）。
    /// 前沿簇的最小尺寸与探索选点的候选抽稀共用。
    pub fn frontier_body_width(&self) -> f64 {
        self.data.body_envelope.as_ref().map_or(2.0 * f64::from(self.config().wall_clearance_m), |body| {
            let (low, high) = body.footprint_xy_m.iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), point| (low.min(point[1]), high.max(point[1])));
            high - low
        })
    }

    fn frontier_dirty_radius(&self) -> i64 {
        i64::from(self.frontier_lookahead() + self.data.profile_cells + 3)
            + (self.config().wall_clearance_m / self.config().voxel_size).ceil() as i64
    }

    fn frontier_lookahead(&self) -> i32 {
        let radius = self
            .data
            .body_envelope
            .as_ref()
            .map_or(f64::from(self.config().wall_clearance_m), |body| {
                body.radius().max(f64::from(self.config().wall_clearance_m))
            });
        ((radius / f64::from(self.config().voxel_size)).ceil() as i32 + 2).min(MAX_PROFILE_CELLS)
    }
    /// 当前几何前沿的未知支撑单元，作为可观察收益样本；不表示已有真实支撑。
    pub fn observation_targets(
        &self,
        boundary: &[[f64; 2]],
    ) -> Result<Vec<nuwax_explore_routing::observation::ObservationTarget>, SurfaceError> {
        validate_boundary(boundary)?;
        let mut cells = std::collections::BTreeSet::new();
        for &cell in self.frontier_index(boundary).cells.keys() {
            cells.extend(self.frontier_unknown_cells(cell, boundary));
        }
        Ok(cells
            .into_iter()
            .map(
                |cell| nuwax_explore_routing::observation::ObservationTarget {
                    cell: [cell.0, cell.1, cell.2],
                    position: self.cell_position(cell),
                },
            )
            .collect())
    }

    /// 传感器视域采用完整机体姿态与外参；空间遮挡读取同一快照占据柱。
    pub fn estimate_observation_view(
        &self,
        model: &nuwax_explore_routing::observation::ObservationModel,
        body: [f64; 3],
        rotation: [f64; 4],
        targets: &[nuwax_explore_routing::observation::ObservationTarget],
    ) -> Result<nuwax_explore_routing::observation::ViewEstimate, String> {
        nuwax_explore_routing::observation::estimate_view(
            model,
            body,
            rotation,
            targets,
            |from, to| {
                nuwax_explore_routing::observation::voxel_view_is_clear(
                    from,
                    to,
                    f64::from(self.config().voxel_size),
                    65_536,
                    |cell| {
                        self.data
                            .occupied_columns
                            .get(&(cell[0], cell[1]))
                            .is_some_and(|zs| zs.binary_search(&cell[2]).is_ok())
                    },
                )
            },
        )
    }

    fn frontier_unknown_cells(&self, cell: VoxelKey, boundary: &[[f64; 2]]) -> AHashSet<VoxelKey> {
        let lookahead = self.frontier_lookahead();
        let mut unknown = AHashSet::new();
        // Discovery records possible information gain, not graph-clearance
        // permission. Consumers must establish an actual route/body sweep.
        let origin = self.cell_position(cell);
        if !inside_boundary([origin[0], origin[1]], boundary)
            || !self.navigation_column_allowed([cell.0, cell.1], f64::from(self.config().voxel_size)) {
            return unknown;
        }
        for (dx, dy) in DIRECTIONS {
            let mut current = cell;
            let mut support_height = origin[2];
            let mut first_unknown = None;
            for _ in 0..lookahead {
                let (nx, ny) = (current.0 + dx, current.1 + dy);
                if self.data.virtual_obstacles.as_ref().is_some_and(|mask| {
                    !mask.permits_cell(
                        (nx, ny, current.2),
                        f64::from(self.config().voxel_size),
                        0.0,
                    )
                }) {
                    break;
                }
                let from = self.cell_position(current);
                let point = self.cell_position((nx, ny, current.2));
                if !segment_inside_boundary([from[0], from[1]], [point[0], point[1]], boundary) {
                    break;
                }
                let next = nearest_z(
                    &self.data.graph.surface_lookup,
                    nx,
                    ny,
                    current.2,
                    self.data.candidate_step_cells,
                );
                // 缺测后若先遇到此层可接续支撑，结束这一段落差证据查询。
                if first_unknown.is_some() && next.is_some() { break; }
                match next {
                    Some(next) if self.has_geometric_edge(current, next) => {
                        current = next;
                        support_height = self.cell_position(current)[2];
                    },
                    Some(_) => break,
                    None => {
                        let clearance =
                            (self.config().robot_height / self.config().voxel_size).ceil() as i32;
                        let occupied =
                            self.data.occupied_columns.get(&(nx, ny)).is_some_and(|zs| {
                                zs.iter()
                                    .any(|&z| z > current.2 && z <= current.2 + clearance)
                            });
                        if occupied { break; }
                        // 下层有实测地面，是此层无支撑的证据；沿用原 lookahead，不扩大前瞻。
                        let step = f64::from(self.data.candidate_step_cells) * f64::from(self.config().voxel_size);
                        if self.data.graph.surface_lookup.get(&(nx, ny)).is_some_and(|zs|
                            zs.iter().any(|&z| self.cell_position((nx, ny, z))[2] < support_height - step)) {
                            first_unknown = None;
                            break;
                        }
                        first_unknown.get_or_insert((nx, ny, current.2));
                        current = (nx, ny, current.2);
                    }
                }
            }
            if let Some(cell) = first_unknown { unknown.insert(cell); }
        }
        unknown
    }
}

/// 平面8邻接、支撑高度差不超过接续台阶高度（`candidate_step_cells`格）的前沿格。
fn for_each_neighbor<V>(cells: &BTreeMap<VoxelKey, V>, cell: VoxelKey, step: i32, mut visit: impl FnMut(VoxelKey, &V)) {
    for dx in -1..=1 {
        for dy in -1..=1 {
            if dx == 0 && dy == 0 {
                continue;
            }
            let (x, y) = (cell.0 + dx, cell.1 + dy);
            for (&neighbor, value) in cells.range((x, y, cell.2 - step)..=(x, y, cell.2 + step)) {
                visit(neighbor, value);
            }
        }
    }
}

/// 平面格点两两之间的最大距离（格）：先求凸包（单调链），再在凸包顶点间两两比较。
fn planar_diameter(points: impl Iterator<Item = [i32; 2]>) -> f64 {
    let mut points: Vec<[i64; 2]> = points.map(|p| [i64::from(p[0]), i64::from(p[1])]).collect();
    points.sort_unstable();
    points.dedup();
    let cross = |o: [i64; 2], a: [i64; 2], b: [i64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut hull: Vec<[i64; 2]> = Vec::new();
    let extend = |hull: &mut Vec<[i64; 2]>, point: [i64; 2], start: usize| {
        while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], point) <= 0 {
            hull.pop();
        }
        hull.push(point);
    };
    for &point in &points {
        extend(&mut hull, point, 0);
    }
    let start = hull.len() - 1;
    for &point in points.iter().rev().skip(1) {
        extend(&mut hull, point, start);
    }
    let mut best = 0_i64;
    for (i, a) in hull.iter().enumerate() {
        for b in &hull[i + 1..] {
            best = best.max((a[0] - b[0]).pow(2) + (a[1] - b[1]).pow(2));
        }
    }
    (best as f64).sqrt()
}

#[cfg(test)]
#[path = "frontiers_tests.rs"]
mod tests;

#[cfg(test)]
mod bounds_tests {
    use super::{DirtyBounds, dirty_contains};
    use std::collections::BTreeMap;

    #[test]
    fn nearby_block_lookup_matches_all_dirty_bounds_at_negative_edges_and_multiple_rings() {
        for radius in [0, 1, 31, 32, 33, 65] {
            let mut blocks = BTreeMap::new();
            for [x, y] in [[-65, -32], [-33, 31], [0, 0], [31, 32], [65, -65], [3200, 3200]] {
                let bounds = DirtyBounds { min_x: x - radius, max_x: x + radius,
                    min_y: y - radius, max_y: y + radius };
                blocks.entry([(x as i32).div_euclid(32), (y as i32).div_euclid(32)])
                    .and_modify(|old: &mut DirtyBounds| *old = old.union(bounds)).or_insert(bounds);
            }
            for x in -135..136 {
                for y in -135..136 {
                    let cell = (x, y, 3);
                    assert_eq!(dirty_contains(&blocks, cell, radius), blocks.values().any(|bounds| bounds.contains(cell)),
                        "半径{radius}，单元{cell:?}");
                }
            }
        }
    }


    #[test]
    fn inclusive_negative_bounds_and_broad_scan_fallback() {
        let bounds = DirtyBounds {
            min_x: -2,
            max_x: 1,
            min_y: -1,
            max_y: 1,
        };
        let (xs, ys) = bounds.column_ranges(12).unwrap();
        assert_eq!(xs.collect::<Vec<_>>(), [-2, -1, 0, 1]);
        assert_eq!(ys.collect::<Vec<_>>(), [-1, 0, 1]);
        assert!(bounds.column_ranges(11).is_none());
        let broad = DirtyBounds {
            min_x: i64::from(i32::MIN) - 50,
            max_x: i64::from(i32::MAX) + 50,
            min_y: i64::from(i32::MIN) - 50,
            max_y: i64::from(i32::MAX) + 50,
        };
        assert!(broad.column_ranges(usize::MAX).is_none());
    }
}

// 前沿可达距离保留在前沿组件，不再维护独立导航路线缓存。

impl SurfaceSnapshot {
    pub(super) fn frontier_distances(
        &self,
        start: CellId,
        boundary: &[[f64; 2]],
    ) -> Vec<f64> {
        let cells = &self.data.graph.cells;
        nuwax_explore_routing::shortest_costs(
            cells.slot_capacity(),
            start as usize,
            None,
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
mod e3c_tests {
    use super::*;

    fn drop_map(lower: Option<f32>, gap: bool) -> SurfaceSnapshot {
        let points: Vec<_> = (0..24).flat_map(|x| (0..12).filter_map(move |y| {
            let z = if x < 10 { 0.85 } else { lower? };
            if gap && x == 10 { return None; }
            Some(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z))
        })).collect();
        SurfaceSnapshot::from_points(&points, Config {
            voxel_size: 0.1, wall_clearance_m: 0.1, surface_closing_radius: 0.0,
            step_threshold_m: 0.2, robot_height: 0.45, wall_buffer_m: 0.1, wall_buffer_weight: 0.0,
            min_step_tread_depth_m: 0.2, world_frame: "map".into(), node_spacing_m: 0.3,
            max_slope_deg: 25.0, step_penalty_weight: 1.0, goal_tolerance: 0.1, viz_publish_hz: 0.0
        }, 0.1).unwrap()
    }

    #[test]
    fn lower_returns_remove_drop_frontiers_within_the_existing_lookahead() {
        for gap in [false, true] {
            let map = drop_map(Some(0.05), gap);
            assert!(!map.frontier_unknown_cells((9, 5, 8), &[]).iter().any(|c| c.0 >= 10 && c.1 == 5),
                "下行边缘或隔一列的下层回波都是此层无支撑的证据");
        }
        let map = drop_map(None, false);
        assert!(map.frontier_unknown_cells((9, 5, 8), &[]).contains(&(10, 5, 8)), "没有下层回波仍是前沿");
    }

    #[test]
    fn small_drop_is_support_and_support_before_lower_evidence_keeps_the_gap_unknown() {
        let map = drop_map(Some(0.75), false);
        assert!(!map.frontier_unknown_cells((9, 5, 8), &[]).contains(&(10, 5, 8)), "小台阶按可接续支撑处理");
        let points: Vec<_> = (0..24).flat_map(|x| (0..12).filter_map(move |y| {
            if x == 10 { return None; }
            Some(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, if x <= 11 { 0.85 } else { 0.05 }))
        })).collect();
        let map = SurfaceSnapshot::from_points(&points, map.config().clone(), 0.1).unwrap();
        assert!(map.frontier_unknown_cells((9, 5, 8), &[]).contains(&(10, 5, 8)), "先碰到同层支撑，缺测列仍未知");
    }

    #[test]
    fn reached_reverse_lookup_and_nearby_query_only_materialize_requested_clusters() {
        let config = drop_map(None, false).config().clone();
        let points: Vec<_> = (0..80_i32).flat_map(|x| (0..80_i32).map(move |y| (x, y)))
            .filter(|&(x, y)| ![(25, 25), (55, 55)].iter().any(|&(hx, hy)| (x - hx).abs() < 2 && (y - hy).abs() < 2))
            .map(|(x, y)| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05)).collect();
        let map = SurfaceSnapshot::from_points(&points, config, 0.1).unwrap();
        let boundary = [[0.3, 0.3], [7.7, 0.3], [7.7, 7.7], [0.3, 7.7]];
        let original = map.frontier_clusters(&boundary).unwrap();
        assert_eq!(original.len(), 2);
        let first = &original[0];
        let other = &original[1];
        let seed = &first.frontiers[0];
        let reached = map.reached_frontier_columns(&[[seed.cell.0, seed.cell.1, seed.cell.2]], seed.support_xyz, 0.2, &boundary);
        assert!(!reached.is_empty());
        let index = map.frontier_index(&boundary);
        let actual: BTreeSet<_> = index.cells.iter().filter(|(cell, _)|
            map.frontier_unknown_cells(**cell, &boundary).iter().any(|c| reached.contains(&[c.0, c.1, c.2])))
            .map(|(_, info)| info.cluster).collect();
        let mut query = map.frontier_query(index, &reached);
        assert!(actual.is_subset(&query.affected), "反查必须覆盖逐格核对得到的全部受影响簇");
        assert!(query.affected.contains(&first.id) && !query.affected.contains(&other.id));
        let cell = other.frontiers[0].cell;
        assert!(map.frontier_near_from_query(&mut query, [cell.0, cell.1, cell.2], 0.3, 0.3, &reached));
        assert_eq!(query.clusters.keys().copied().collect::<BTreeSet<_>>(), BTreeSet::from([other.id]),
            "途中核对不计算半径外被标记影响的另一个簇");
        let combined = map.frontier_clusters_from_query(&mut query, &reached);
        assert_eq!(combined, map.frontier_clusters_with_reached(&boundary, &reached).unwrap(),
            "先局部核对再选点与直接全量查询的分簇和收益必须一致");
        assert_eq!(query.clusters.len(), 2, "选点再补齐其它簇，复用已核对的簇");
    }

}
