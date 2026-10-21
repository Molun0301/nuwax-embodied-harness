//! 前沿几何索引与局部失效；不缓存机器人可达关系或路径代价。
use super::*;
use std::collections::BTreeMap;
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
}

pub(super) type CacheSlot = Mutex<Option<CachedFrontiers>>;

#[derive(Clone)]
pub(super) struct CachedFrontiers {
    seed: Arc<FrontierIndex>,
    dirty: Option<DirtyBounds>,
}

pub(super) struct FrontierIndex {
    boundary: Vec<[f64; 2]>,
    pub(super) gains: BTreeMap<VoxelKey, f64>,
    stats: SurfaceFrontierCacheStats,
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

impl SurfaceSnapshot {
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

    /// Geometric viewpoint seeds only. Reachability must be checked by the active
    /// planner; absence of an observed start cell does not invalidate this query.
    pub fn frontier_candidates(
        &self,
        boundary: &[[f64; 2]],
    ) -> Result<Vec<SurfaceFrontier>, SurfaceError> {
        validate_boundary(boundary)?;
        Ok(self.frontier_index(boundary).gains.iter().map(|(&cell, &gain)| {
            SurfaceFrontier {
                support_xyz: self.cell_position(cell),
                gain,
                travel_cost: f64::INFINITY,
                cell,
            }
        }).collect())
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
    /// 连续更新仅合并一个失效包围盒，实际视线计算延后至前沿查询，不保留快照链。
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
            let radius = i64::from(self.frontier_lookahead() + self.data.profile_cells + 3)
                + (self.config().wall_clearance_m / self.config().voxel_size).ceil() as i64;
            let dirty = DirtyBounds {
                min_x: changes.iter().map(|p| i64::from(p[0])).min().unwrap() - radius,
                max_x: changes.iter().map(|p| i64::from(p[0])).max().unwrap() + radius,
                min_y: changes.iter().map(|p| i64::from(p[1])).min().unwrap() - radius,
                max_y: changes.iter().map(|p| i64::from(p[1])).max().unwrap() + radius,
            };
            cache.dirty = Some(cache.dirty.map_or(dirty, |old| old.union(dirty)));
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

    pub(super) fn frontier_index(&self, boundary: &[[f64; 2]]) -> Arc<FrontierIndex> {
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
        let dirty = cache.as_ref().and_then(|cache| cache.dirty);
        let mut gains: BTreeMap<_, _> = cache.as_ref().map_or_else(BTreeMap::new, |cache| {
            cache
                .seed
                .gains
                .iter()
                .filter(|(cell, _)| !dirty.unwrap().contains(**cell))
                .map(|(&cell, &gain)| (cell, gain))
                .collect()
        });
        let retained_frontiers = gains.len();
        let mut evaluated_cells = 0;
        let mut evaluate = |cell| {
            evaluated_cells += 1;
            let gain = self.frontier_gain(cell, boundary);
            if gain > 0.0 {
                gains.insert(cell, gain);
            }
        };
        if let Some((xs, ys)) =
            dirty.and_then(|bounds| bounds.column_ranges(self.data.graph.cells.slot_capacity()))
        {
            for x in xs {
                for y in ys.clone() {
                    for &z in self
                        .data
                        .graph
                        .surface_lookup
                        .get(&(x, y))
                        .into_iter()
                        .flatten()
                    {
                        evaluate((x, y, z));
                    }
                }
            }
        } else {
            for cell in self.cell_keys() {
                if dirty.is_none_or(|bounds| bounds.contains(cell)) {
                    evaluate(cell);
                }
            }
        }
        let index = Arc::new(FrontierIndex {
            boundary: boundary.to_vec(),
            stats: SurfaceFrontierCacheStats {
                evaluated_cells,
                retained_frontiers,
                frontier_count: gains.len(),
                incremental: dirty.is_some(),
            },
            gains,
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
            evaluated_cells, retained_frontiers, incremental = dirty.is_some(),
            "前沿几何索引重建");
        index
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
    fn frontier_gain(&self, cell: VoxelKey, boundary: &[[f64; 2]]) -> f64 {
        self.frontier_unknown_cells(cell, boundary).len() as f64
            * f64::from(self.config().voxel_size).powi(2)
    }

    /// 当前几何前沿的未知支撑单元，作为可观察收益样本；不表示已有真实支撑。
    pub fn observation_targets(
        &self,
        boundary: &[[f64; 2]],
    ) -> Result<Vec<nuwax_explore_routing::observation::ObservationTarget>, SurfaceError> {
        validate_boundary(boundary)?;
        let mut cells = std::collections::BTreeSet::new();
        for &cell in self.frontier_index(boundary).gains.keys() {
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
                match next {
                    Some(next) if self.has_geometric_edge(current, next) => current = next,
                    Some(_) => break,
                    None => {
                        let clearance =
                            (self.config().robot_height / self.config().voxel_size).ceil() as i32;
                        let occupied =
                            self.data.occupied_columns.get(&(nx, ny)).is_some_and(|zs| {
                                zs.iter()
                                    .any(|&z| z > current.2 && z <= current.2 + clearance)
                            });
                        if !occupied {
                            unknown.insert((nx, ny, current.2));
                        }
                        break;
                    }
                }
            }
        }
        unknown
    }
}

#[cfg(test)]
#[path = "frontiers_tests.rs"]
mod tests;

#[cfg(test)]
mod bounds_tests {
    use super::DirtyBounds;

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
