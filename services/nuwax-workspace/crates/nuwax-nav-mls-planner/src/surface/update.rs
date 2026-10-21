//! 共享表面更新：先比较物理占据列，再局部重建支撑与严格邻接。

use super::*;
use crate::adjacency::rebuild_edges_around;
use crate::surfaces::standable_in_column;

/// 本次表面更新采用的计算路径。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceBuildKind {
    /// 首帧、精度变化或大范围变化的完整构建。
    Full,
    /// 物理体素集合没有变化，直接复用不可变快照。
    Reused,
    /// 局部更新支撑、邻接、有限墙代价和分区路线。
    Incremental,
}

/// 实际工作量，不参与地图或任务身份。
#[derive(Debug, Clone, Copy)]
pub struct SurfaceBuildStats {
    /// 构建路径。
    pub kind: SurfaceBuildKind,
    /// 收到的原始点数。
    pub input_points: usize,
    /// 物理占据变化的列数。
    pub changed_columns: usize,
    /// 局部新增/删除的支撑单元数；全量时为构建的全部单元数。
    pub changed_surface_cells: usize,
    /// 本次重新检查严格坡阶连接的单元数。
    pub checked_cells: usize,
    /// 是否因首次构建或大窗口回退而执行全局墙代价刷新。
    pub global_cost_refresh: bool,
}

/// 新快照及其几何变化范围；外层继续绑定现有地图修订和源时间。
pub struct SurfaceMapBuild {
    /// 可供规划、执行和探索共同只读消费的物理表面。
    pub snapshot: Arc<SurfaceSnapshot>,
    /// 真正变化的水平列，包含被完全删除的列。
    pub changed_columns: Vec<[i32; 2]>,
    /// 计算路径和工作量。
    pub stats: SurfaceBuildStats,
}

/// 一条完整规划列的替换值，空列表表示删除。高度来自精细源体素的实际上下界。
#[derive(Debug, Clone)]
pub struct SurfaceColumnUpdate {
    /// 规划分辨率下的水平列。
    pub column: [i32; 2],
    /// 严格递增的占据高度索引及真实高度范围。
    pub cells: Vec<(i32, [f32; 2])>,
}

/// 单一计算拥有方持有的构建器，不拥有业务地图身份或输入时效。
pub struct SurfaceMapBuilder {
    config: Config,
    body: Option<SurfaceBodyEnvelope>,
    current: Option<Arc<SurfaceSnapshot>>,
}

impl SurfaceMapBuilder {
    /// 捕获本拥有方的固定物理配置，实际输入时校验精度与计算预算。
    pub fn new(config: Config) -> Self {
        Self::configured(config, None)
    }

    /// 固定机体能力与物理配置共同参与每次局部代价更新。
    pub fn configured(config: Config, body: Option<SurfaceBodyEnvelope>) -> Self {
        Self {
            config,
            body,
            current: None,
        }
    }

    /// 撤销内部复用基线；已交付的不可变快照不受影响。
    pub fn reset(&mut self) {
        self.current = None;
    }

    /// 将现有完整点云适配成列变化；失败不安装候选，不返回旧图冒充新图。
    pub fn build(
        &mut self,
        points: &[(f32, f32, f32)],
        source_voxel_size_m: f32,
    ) -> Result<SurfaceMapBuild, SurfaceError> {
        validate_config(&self.config, source_voxel_size_m)?;
        let columns = voxel_columns(points, self.config.voxel_size)?;
        let heights = height_bounds(points, self.config.voxel_size, source_voxel_size_m);
        let mut changes: Vec<_> = columns
            .iter()
            .filter_map(|(column, zs)| {
                self.current
                    .as_ref()
                    .is_none_or(|previous| previous.data.occupied_columns.get(column) != Some(zs))
                    .then_some(*column)
            })
            .collect();
        if let Some(previous) = self.current.as_ref() {
            changes.extend(
                previous
                    .data
                    .occupied_columns
                    .keys()
                    .filter(|column| !columns.contains_key(column))
                    .copied(),
            );
        }
        if let Some(previous) = self.current.as_ref() {
            for (cell, bounds) in &heights {
                if previous.data.height_bounds.get(cell) != Some(bounds) {
                    changes.push((cell.0, cell.1));
                }
            }
            for cell in previous.data.height_bounds.keys() {
                if !heights.contains_key(cell) {
                    changes.push((cell.0, cell.1));
                }
            }
        }
        changes.sort_unstable();
        changes.dedup();
        let count = columns.values().map(Vec::len).sum();
        let mut result = self.install_columns(
            columns,
            heights,
            changes,
            source_voxel_size_m,
            points.len(),
            count,
        )?;
        let snapshot = result
            .snapshot
            .as_ref()
            .clone()
            .with_navigation_measurements(points);
        let same_measurements = match (
            &snapshot.navigation_measurements,
            &result.snapshot.navigation_measurements,
        ) {
            (Some(left), Some(right)) => Arc::ptr_eq(left, right),
            (None, None) => true,
            _ => false,
        };
        if !same_measurements {
            result.snapshot = Arc::new(snapshot);
            self.current = Some(Arc::clone(&result.snapshot));
        }
        Ok(result)
    }

    /// 适配由同一完整根导出的脏列。重置用于首次输入、切域和精度变化。
    /// 校验和候选构建失败均不推进基线；调用者只在成功后记录根。
    /// 空的完整根或删除最后一列是有效的未知地图。调用者必须先验证根的完整性与域；
    /// 这不把缺帧、清空事件或损坏点云转成有效输入。
    pub fn build_column_updates(
        &mut self,
        updates: &[SurfaceColumnUpdate],
        source_voxel_size_m: f32,
        reset: bool,
        input_points: usize,
    ) -> Result<SurfaceMapBuild, SurfaceError> {
        validate_config(&self.config, source_voxel_size_m)?;
        let previous = self.current.as_ref().filter(|_| !reset);
        if previous
            .is_some_and(|p| p.source_voxel_size_m().to_bits() != source_voxel_size_m.to_bits())
        {
            return Err(SurfaceError::InvalidConfig(
                "精度变化必须提供完整列集".into(),
            ));
        }
        let mut columns =
            previous.map_or_else(ColumnIz::default, |p| (*p.data.occupied_columns).clone());
        let mut heights =
            previous.map_or_else(HeightBounds::default, |p| (*p.data.height_bounds).clone());
        let mut count = previous.map_or(0, |p| p.data.occupied_voxels);
        let mut seen = AHashSet::new();
        let mut changes = Vec::new();
        for update in updates {
            let key = (update.column[0], update.column[1]);
            if !seen.insert(key)
                || key.0.abs_diff(0) > i32::MAX as u32 / 2
                || key.1.abs_diff(0) > i32::MAX as u32 / 2
            {
                return Err(SurfaceError::InvalidPointCloud);
            }
            // Dirty source blocks also carry unchanged planning columns. Compare
            // borrowed cells before copying any persistent-map pages or z vectors.
            let old = columns.get(&key).map_or(&[][..], Vec::as_slice);
            let mut changed = old.len() != update.cells.len();
            let mut last_z = None;
            for (index, &(z, bounds)) in update.cells.iter().enumerate() {
                if last_z.is_some_and(|old| old >= z)
                    || !bounds.iter().all(|v| v.is_finite())
                    || bounds[0] >= bounds[1]
                    || z.abs_diff(0) > i32::MAX as u32 / 2
                {
                    return Err(SurfaceError::InvalidPointCloud);
                }
                last_z = Some(z);
                changed |= old.get(index) != Some(&z);
                if self.config.voxel_size.to_bits() != source_voxel_size_m.to_bits() {
                    changed |= heights.get(&(key.0, key.1, z)) != Some(&bounds);
                }
            }
            if !changed {
                continue;
            }
            let zs: Vec<_> = update.cells.iter().map(|&(z, _)| z).collect();
            count += zs.len();
            if let Some(old) = columns.remove(&key) {
                count -= old.len();
                for z in old {
                    heights.remove(&(key.0, key.1, z));
                }
            }
            if !zs.is_empty() {
                columns.insert(key, zs);
            }
            if self.config.voxel_size.to_bits() != source_voxel_size_m.to_bits() {
                for &(z, bounds) in &update.cells {
                    heights.insert((key.0, key.1, z), bounds);
                }
            }
            changes.push(key);
        }
        if count > MAX_SOURCE_POINTS {
            return Err(SurfaceError::InvalidPointCloud);
        }
        if reset {
            // 仅本次构建隔离旧基线，失败后恢复原计算状态。
            let old = self.current.take();
            let result = self.install_columns(
                columns,
                heights,
                changes,
                source_voxel_size_m,
                input_points,
                count,
            );
            if result.is_err() {
                self.current = old;
            }
            result
        } else {
            self.install_columns(
                columns,
                heights,
                changes,
                source_voxel_size_m,
                input_points,
                count,
            )
        }
    }

    fn install_columns(
        &mut self,
        columns: ColumnIz,
        heights: HeightBounds,
        changes: Vec<(i32, i32)>,
        source_voxel_size_m: f32,
        input_points: usize,
        occupied_voxels: usize,
    ) -> Result<SurfaceMapBuild, SurfaceError> {
        let (profile_cells, candidate_step_cells) =
            validate_config(&self.config, source_voxel_size_m)?;
        if let Some(body) = &self.body {
            body.validate()?;
        }
        let same_precision = self.current.as_ref().is_some_and(|previous| {
            previous.source_voxel_size_m().to_bits() == source_voxel_size_m.to_bits()
        });
        if changes.is_empty() && same_precision {
            return Ok(SurfaceMapBuild {
                snapshot: Arc::clone(self.current.as_ref().expect("已有相同精度基线")),
                changed_columns: Vec::new(),
                stats: SurfaceBuildStats {
                    kind: SurfaceBuildKind::Reused,
                    input_points,
                    changed_columns: 0,
                    changed_surface_cells: 0,
                    checked_cells: 0,
                    global_cost_refresh: false,
                },
            });
        }
        // 坡阶分类只读沿四个方向的踏面轮廓；额外一列覆盖边的另一端。
        let reach = profile_cells + 1;
        let work = changes.len().saturating_mul(1 + 4 * reach as usize);
        let local_budget = (columns.len() / 2).max(64).min(200_000);
        let incremental = same_precision && work <= local_budget;
        let (snapshot, changed_surface_cells, checked_cells, global_cost_refresh) = if incremental {
            update_columns(
                self.current.as_ref().expect("已确认基线"),
                columns,
                heights,
                &changes,
                profile_cells,
                candidate_step_cells,
                occupied_voxels,
            )?
        } else {
            let snapshot = from_columns(
                columns,
                heights,
                self.config.clone(),
                source_voxel_size_m,
                self.body.as_ref(),
            )?;
            let count = snapshot.data.graph.cells.ids().count();
            (snapshot, count, count, true)
        };
        let snapshot = Arc::new(snapshot);
        let stats = SurfaceBuildStats {
            kind: if incremental {
                SurfaceBuildKind::Incremental
            } else {
                SurfaceBuildKind::Full
            },
            input_points,
            changed_columns: changes.len(),
            changed_surface_cells,
            checked_cells,
            global_cost_refresh,
        };
        self.current = Some(Arc::clone(&snapshot));
        Ok(SurfaceMapBuild {
            snapshot,
            changed_columns: changes.into_iter().map(|(x, y)| [x, y]).collect(),
            stats,
        })
    }
}

pub(super) fn voxel_columns(
    points: &[(f32, f32, f32)],
    size: f32,
) -> Result<ColumnIz, SurfaceError> {
    if points.len() > MAX_SOURCE_POINTS {
        return Err(SurfaceError::InvalidPointCloud);
    }
    let mut columns = ColumnIz::default();
    for &(x, y, z) in points {
        if ![x, y, z]
            .into_iter()
            .all(|value| value.is_finite() && (value / size).abs() < i32::MAX as f32 * 0.5)
        {
            return Err(SurfaceError::InvalidPointCloud);
        }
        let (x, y, z) = voxelize((x, y, z), size);
        columns.entry((x, y)).or_default().push(z);
    }
    for (_, zs) in columns.iter_mut() {
        zs.sort_unstable();
        zs.dedup();
    }
    Ok(columns)
}

pub(super) fn height_bounds(
    points: &[(f32, f32, f32)],
    size: f32,
    source_size: f32,
) -> HeightBounds {
    if size.to_bits() == source_size.to_bits() {
        return HeightBounds::default();
    }
    let mut result = HeightBounds::default();
    for &(x, y, z) in points {
        let cell = voxelize((x, y, z), size);
        let fine = (f64::from(z) / f64::from(source_size)).floor();
        let low = (fine * f64::from(source_size)) as f32;
        let high = ((fine + 1.0) * f64::from(source_size)) as f32;
        result
            .entry(cell)
            .and_modify(|bounds: &mut [f32; 2]| {
                bounds[0] = bounds[0].min(low);
                bounds[1] = bounds[1].max(high);
            })
            .or_insert([low, high]);
    }
    result
}
pub(super) fn column_supports(
    x: i32,
    y: i32,
    zs: &[i32],
    config: &Config,
    bounds: &HeightBounds,
    out: &mut Vec<VoxelKey>,
) {
    if bounds.is_empty() {
        standable_in_column(
            x,
            y,
            zs,
            (config.robot_height / config.voxel_size).ceil() as i32,
            out,
        );
        return;
    }
    for (i, &z) in zs.iter().enumerate() {
        let top = bounds
            .get(&(x, y, z))
            .map_or((z as f32 + 1.0) * config.voxel_size, |bounds| bounds[1]);
        let clear = zs.get(i + 1).is_none_or(|&next| {
            bounds
                .get(&(x, y, next))
                .map_or(next as f32 * config.voxel_size, |bounds| bounds[0])
                - top
                + 1e-6
                >= config.robot_height
        });
        if clear {
            out.push((x, y, z));
        }
    }
}
fn refresh_heights(graph: &mut PlannerGraph, bounds: &HeightBounds, size: f32) {
    if bounds.is_empty() {
        return;
    }
    let positions: Vec<_> = (0..graph.cells.slot_capacity())
        .map(|id| graph.cells.coord(id as CellId))
        .collect();
    let height = |cell: VoxelKey| {
        bounds
            .get(&cell)
            .map_or((cell.2 as f32 + 1.0) * size, |b| b[1])
    };
    for (id, edges) in graph.cells.iter_edges_mut() {
        let from = positions[id as usize];
        for edge in edges {
            let to = positions[edge.dest as usize];
            let rise = height(to) - height(from);
            edge.rise = rise.abs();
            edge.base_cost = ((to.0 - from.0) as f32 * size)
                .hypot((to.1 - from.1) as f32 * size)
                .hypot(rise);
            edge.cost = edge.base_cost;
        }
    }
}

pub(super) fn from_columns(
    columns: ColumnIz,
    heights: HeightBounds,
    config: Config,
    source_voxel_size_m: f32,
    model: Option<&SurfaceBodyEnvelope>,
) -> Result<SurfaceSnapshot, SurfaceError> {
    let (profile_cells, candidate_step_cells) = validate_config(&config, source_voxel_size_m)?;
    let mut surfaces = Vec::with_capacity(columns.len());
    for (&(x, y), zs) in &columns {
        column_supports(x, y, zs, &config, &heights, &mut surfaces);
    }
    let mut graph = PlannerGraph::new();
    build_surface_lookup(&surfaces, &mut graph.surface_lookup);
    build_surface_cells(
        &mut graph.cells,
        &graph.surface_lookup,
        config.voxel_size,
        candidate_step_cells,
    );
    let coords: Vec<_> = (0..graph.cells.slot_capacity())
        .map(|id| graph.cells.coord(id as CellId))
        .collect();
    for (id, edges) in graph.cells.iter_edges_mut() {
        edges.retain(|edge| {
            classify_transition(
                &graph.surface_lookup,
                coords[id as usize],
                coords[edge.dest as usize],
                &config,
                &heights,
                source_voxel_size_m,
                profile_cells,
                candidate_step_cells,
            )
            .is_some()
        });
    }
    refresh_heights(&mut graph, &heights, config.voxel_size);
    wall::full(&mut graph,&config,&columns,&heights);
    if let Some(model) = model {
        model.validate()?;
        let ids = graph.cells.ids().collect::<Vec<_>>();
        body::apply_directions(
            &mut graph,
            &heights,
            &config,
            source_voxel_size_m,
            model,
            &ids,
        );
    }
    let graph = Arc::new(graph);
    let occupied_voxels = columns.values().map(Vec::len).sum();
    Ok(SurfaceSnapshot {
        navigation_measurements: None,
        ground_evidence:GroundEvidence::MeasuredSurface,
        recent_obstacles:None,
        observations: None,
        body_query_cache: Default::default(),
        data: Arc::new(SurfaceData {
            frontier_cache: Default::default(),
            body_cache: Default::default(),
            body_envelope: model.cloned().map(Arc::new),
            height_bounds: Arc::new(heights),
            physical_graph: Arc::clone(&graph),
            graph,
            occupied_columns: Arc::new(columns),
            occupied_voxels,
            virtual_obstacles: None,
            virtual_blocked_cells: Arc::new(AHashSet::new()),
            config,
            source_voxel_size_m,
            profile_cells,
            candidate_step_cells,
        }),
    })
}

fn update_columns(
    previous: &SurfaceSnapshot,
    columns: ColumnIz,
    heights: HeightBounds,
    changes: &[(i32, i32)],
    profile_cells: i32,
    candidate_step_cells: i32,
    occupied_voxels: usize,
) -> Result<(SurfaceSnapshot, usize, usize, bool), SurfaceError> {
    let config = previous.config();
    let old_graph = &previous.data.physical_graph;
    let mut added = Vec::new();
    let mut removed = Vec::new();
    let mut new_supports = Vec::with_capacity(changes.len());
    for &(x, y) in changes {
        let mut surface = Vec::new();
        if let Some(zs) = columns.get(&(x, y)) {
            column_supports(x, y, zs, config, &heights, &mut surface);
        }
        let zs: Vec<_> = surface.into_iter().map(|point| point.2).collect();
        let old = old_graph
            .surface_lookup
            .get(&(x, y))
            .map_or(&[][..], Vec::as_slice);
        added.extend(
            zs.iter()
                .filter(|z| old.binary_search(z).is_err())
                .map(|&z| (x, y, z)),
        );
        removed.extend(
            old.iter()
                .filter(|z| zs.binary_search(z).is_err())
                .map(|&z| (x, y, z)),
        );
        new_supports.push(((x, y), zs));
    }
    let changed_cells = added.len() + removed.len();
    let mut checked_cells = 0;
    let changed_heights = changes.iter().any(|&(x, y)| {
        old_graph
            .surface_lookup
            .get(&(x, y))
            .into_iter()
            .flatten()
            .any(|&z| previous.data.height_bounds.get(&(x, y, z)) != heights.get(&(x, y, z)))
    });
    let mut global_refresh = false;
    let changed_wall_geometry=changes.iter().any(|&(x,y)|DIRECTIONS.iter().any(|&(dx,dy)| {
        old_graph.surface_lookup.get(&(x+dx,y+dy)).into_iter().flatten().any(|&z| {
            let cell=(x+dx,y+dy,z);
            let ground=heights.get(&cell).map_or((z as f32+1.0)*config.voxel_size,|b|b[1]);
            wall::column_obstructs(&previous.data.occupied_columns,&previous.data.height_bounds,(x,y),ground,config)
                != wall::column_obstructs(&columns,&heights,(x,y),ground,config)
        })
    }));
    let graph = if changed_cells == 0 && !changed_heights && !changed_wall_geometry {
        // 占据列变化也要交付新快照，前沿可见性不能继续读旧占据证据。
        Arc::clone(old_graph)
    } else {
        let mut graph = (**old_graph).clone();
        for (column, zs) in new_supports {
            if zs.is_empty() {
                graph.surface_lookup.remove(&column);
            } else {
                graph.surface_lookup.insert(column, zs);
            }
        }
        for &cell in &removed {
            graph.cells.remove(cell);
        }
        for &cell in &added {
            graph.cells.insert(cell);
        }
        let reach = profile_cells + 1;
        let mut affected_columns = AHashSet::new();
        for &(x, y) in changes {
            affected_columns.insert((x, y));
            for offset in 1..=reach {
                for (dx, dy) in DIRECTIONS {
                    affected_columns.insert((x + dx * offset, y + dy * offset));
                }
            }
        }
        let mut seeds = removed;
        for column in &affected_columns {
            if let Some(zs) = graph.surface_lookup.get(&column) {
                seeds.extend(zs.iter().map(|&z| (column.0, column.1, z)));
            }
        }
        let affected = rebuild_edges_around(
            &mut graph.cells,
            &graph.surface_lookup,
            &seeds,
            config.voxel_size,
            candidate_step_cells,
        );
        checked_cells = affected.len();
        for &id in &affected {
            let from = graph.cells.coord(id);
            let edges = graph
                .cells
                .neighbors(id)
                .iter()
                .copied()
                .filter(|edge| {
                    classify_transition(
                        &graph.surface_lookup,
                        from,
                        graph.cells.coord(edge.dest),
                        config,
                        &heights,
                        previous.source_voxel_size_m(),
                        profile_cells,
                        candidate_step_cells,
                    )
                    .is_some()
                })
                .collect();
            graph.cells.replace_edges(id, edges);
        }
        // 只刷新重新建立的边；远场已带墙代价的边不得被重置成基础代价。
        for &id in &affected {
            let from = graph.cells.coord(id);
            let from_height = heights
                .get(&from)
                .map_or((from.2 as f32 + 1.0) * config.voxel_size, |b| b[1]);
            let edges = graph
                .cells
                .neighbors(id)
                .iter()
                .map(|edge| {
                    let to = graph.cells.coord(edge.dest);
                    let to_height = heights
                        .get(&to)
                        .map_or((to.2 as f32 + 1.0) * config.voxel_size, |b| b[1]);
                    let mut edge = *edge;
                    edge.rise = (to_height - from_height).abs();
                    edge.base_cost = ((to.0 - from.0) as f32 * config.voxel_size)
                        .hypot((to.1 - from.1) as f32 * config.voxel_size)
                        .hypot(edge.rise);
                    edge
                })
                .collect();
            graph.cells.replace_edges(id, edges);
        }
        for &id in &affected {
            let c = graph.cells.coord(id);
            affected_columns.insert((c.0, c.1));
        }
        let (cost_columns, global) = wall::update(&mut graph, config, &affected_columns,&columns,&heights);
        global_refresh = global;
        if let Some(model) = previous.data.body_envelope.as_deref() {
            let mut ids = Vec::new();
            for &(x, y) in &cost_columns {
                if let Some(zs) = graph.surface_lookup.get(&(x, y)) {
                    for &z in zs {
                        if let Some(id) = graph.cells.id((x, y, z)) {
                            ids.push(id);
                        }
                    }
                }
            }
            body::apply_directions(
                &mut graph,
                &heights,
                config,
                previous.source_voxel_size_m(),
                model,
                &ids,
            );
        }
        Arc::new(graph)
    };
    let snapshot = SurfaceSnapshot {
        navigation_measurements: None,
        ground_evidence:previous.ground_evidence,
        recent_obstacles:previous.recent_obstacles.clone(),
        observations: None,
        body_query_cache: Default::default(),
        data: Arc::new(SurfaceData {
            frontier_cache: Default::default(),
            body_cache: Default::default(),
            body_envelope: previous.data.body_envelope.clone(),
            height_bounds: Arc::new(heights),
            physical_graph: Arc::clone(&graph),
            graph,
            occupied_columns: Arc::new(columns),
            occupied_voxels,
            virtual_obstacles: None,
            virtual_blocked_cells: Arc::new(AHashSet::new()),
            config: config.clone(),
            source_voxel_size_m: previous.source_voxel_size_m(),
            profile_cells,
            candidate_step_cells,
        }),
    };
    let changes: Vec<_> = changes.iter().map(|&(x, y)| [x, y]).collect();
    snapshot.inherit_frontier_cache(previous, &changes);
    Ok((snapshot, changed_cells, checked_cells, global_refresh))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{BTreeMap, BTreeSet};

    const SIZE: f32 = 0.1;
    fn config() -> Config {
        Config {
            world_frame: "map".into(),
            voxel_size: SIZE,
            robot_height: 0.6,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.1,
            wall_buffer_m: 0.3,
            wall_buffer_weight: 2.0,
            step_threshold_m: 0.2,
            max_slope_deg: 25.0,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        }
    }
    fn point((x, y, z): VoxelKey) -> (f32, f32, f32) {
        (
            (x as f32 + 0.5) * SIZE,
            (y as f32 + 0.5) * SIZE,
            (z as f32 + 0.5) * SIZE,
        )
    }
    fn points(cells: &BTreeSet<VoxelKey>) -> Vec<(f32, f32, f32)> {
        cells.iter().copied().map(point).collect()
    }
    fn scene() -> BTreeSet<VoxelKey> {
        (0..60)
            .flat_map(|x| (0..32).flat_map(move |y| [(x, y, 0), (x, y, 30)]))
            .collect()
    }
    fn assert_equivalent(actual: &SurfaceSnapshot, expected: &SurfaceSnapshot) {
        assert_eq!(actual.data.occupied_columns, expected.data.occupied_columns);
        assert_eq!(
            actual.data.graph.surface_lookup,
            expected.data.graph.surface_lookup
        );
        let a = &actual.data.graph;
        let b = &expected.data.graph;
        assert_eq!(
            actual.cell_keys().collect::<BTreeSet<_>>(),
            expected.cell_keys().collect()
        );
        for (id, edges) in a.cells.iter() {
            let key = a.cells.coord(id);
            let other = b.cells.id(key).unwrap();
            let map_edges = |graph: &PlannerGraph,
                             edges: &[crate::adjacency::Edge]|
             -> BTreeMap<VoxelKey, (f32, f32)> {
                edges
                    .iter()
                    .map(|edge| (graph.cells.coord(edge.dest), (edge.base_cost, edge.cost)))
                    .collect()
            };
            let left = map_edges(a, edges);
            let right = map_edges(b, b.cells.neighbors(other));
            assert_eq!(
                left.keys().collect::<Vec<_>>(),
                right.keys().collect::<Vec<_>>(),
                "邻接不同: {key:?}"
            );
            for (dest, (base, cost)) in left {
                let (other_base, other_cost) = right[&dest];
                assert!((base - other_base).abs() < 1e-5);
                assert!(
                    cost == other_cost || (cost - other_cost).abs() < 1e-4,
                    "代价不同: {key:?} -> {dest:?}: {cost} / {other_cost}"
                );
            }
            let left = a.wall_state.dist[id as usize];
            let right = b.wall_state.dist[other as usize];
            assert!(
                left == right || (left - right).abs() < 1e-4,
                "墙距不同: {key:?}: {left} / {right}"
            );
        }
    }

    #[test]
    fn captured_body_limits_update_locally_and_explicit_replacement_reopens_direction() {
        let model = SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.15, -0.1], [0.15, -0.1], [0.15, 0.1], [-0.15, 0.1]],
            z_bounds_m: [-0.1, 0.2],
            body_to_support_z_m: 0.35,
            max_step_up_m: 0.21,
            max_step_down_m: 0.1,
            max_cross_slope_deg: 15.0,
            support_uncertainty_m: 0.0,
        };
        let mut points: Vec<_> = (0..80)
            .flat_map(|x| {
                (0..24).map(move |y| {
                    (
                        (x as f32 + 0.5) * SIZE,
                        (y as f32 + 0.5) * SIZE,
                        if x < 32 { 0.025 } else { 0.225 },
                    )
                })
            })
            .collect();
        let mut builder = SurfaceMapBuilder::configured(config(), Some(model.clone()));
        let old = builder.build(&points, 0.05).unwrap().snapshot;
        let low = [1.05, 1.05, 0.05];
        let high = [5.05, 1.05, 0.25];
        assert!(old.test_connected(low, high));
        assert!(!old.test_connected(high, low));
        points.retain(|&p| voxelize(p, SIZE) != (50, 12, 2));
        let updated = builder.build(&points, 0.05).unwrap();
        assert!(!updated.stats.global_cost_refresh);
        let reference = SurfaceSnapshot::from_points(&points, config(), 0.05)
            .unwrap()
            .with_body_envelope(model.clone())
            .unwrap();
        assert_equivalent(&updated.snapshot, &reference);
        assert!(!updated.snapshot.test_connected(high, low));
        let mut replacement = model;
        replacement.max_step_down_m = 0.3;
        let replaced = updated.snapshot.with_body_envelope(replacement).unwrap();
        assert!(replaced.test_connected(high, low));
        assert!(!old.test_connected(high, low));
    }

    #[test]
    fn bounded_wall_updates_match_physical_rectangles_after_obstacle_removal() {
        let mut cells = scene();
        let mut builder = SurfaceMapBuilder::new(config());
        for (key, add) in [
            ((25, 16, 4), true),
            ((26, 16, 4), true),
            ((25, 16, 4), false),
            ((26, 16, 4), false),
            ((31, 16, 0), false),
            ((31, 16, 0), true),
        ] {
            if add {
                cells.insert(key);
            } else {
                cells.remove(&key);
            }
            let snapshot = builder.build(&points(&cells), SIZE).unwrap().snapshot;
            let graph = &snapshot.data.graph;
            // 用场景的真实矩形边界作为独立真值，不再与旧的边界格中心距离对账。
            let expected = |id: CellId| {
                let (x,y,z)=graph.cells.coord(id);
                let mut distance=0.4_f32;
                if z==0 {
                    for (bx,by) in [(25,16),(26,16),(31,16)] {
                        if !cells.contains(&(bx,by,4)) {continue;}
                        let dx=((x-bx) as f32).abs().mul_add(1.0,-0.5).max(0.0);
                        let dy=((y-by) as f32).abs().mul_add(1.0,-0.5).max(0.0);
                        distance=distance.min(dx.hypot(dy)*SIZE);
                    }
                }
                distance
            };
            for id in graph.cells.ids() {
                assert!((graph.wall_state.dist[id as usize]-expected(id)).abs()<1e-5,
                    "墙距不符合物理边界: {:?}",graph.cells.coord(id));
                let penalty=|id| penalty_of(expected(id),0.1,0.3,2.0);
                for edge in graph.cells.neighbors(id) {
                    let expected_cost=edge.base_cost*(penalty(id)+penalty(edge.dest))/2.0+edge.rise;
                    assert!(edge.cost==expected_cost || (edge.cost-expected_cost).abs()<1e-5,
                        "edge {:?}->{:?}: actual={} expected={}, dist=({},{}), rectangle=({},{})",
                        graph.cells.coord(id),graph.cells.coord(edge.dest),edge.cost,expected_cost,
                        graph.wall_state.dist[id as usize],graph.wall_state.dist[edge.dest as usize],
                        expected(id),expected(edge.dest));
                }
            }
        }
    }

    #[test]
    fn wall_distance_reads_real_occupied_height_even_without_surface_changes() {
        let mut cells=scene();
        cells.remove(&(25,16,0));
        cells.insert((25,16,10));
        let mut builder=SurfaceMapBuilder::new(config());
        let open=builder.build(&points(&cells),SIZE).unwrap().snapshot;
        let at=|map:&SurfaceSnapshot| {
            let id=map.data.graph.cells.id((24,16,0)).unwrap();
            map.data.graph.wall_state.dist[id as usize]
        };
        assert_eq!(at(&open),0.4,"separate upper floors and missing lower floor are not walls");
        cells.insert((25,16,6));
        let low=builder.build(&points(&cells),SIZE).unwrap().snapshot;
        assert_eq!(open.data.graph.surface_lookup,low.data.graph.surface_lookup,
            "the added body-height return has no newly standable top");
        assert!((at(&low)-SIZE*0.5).abs()<1e-6,"real low-top occupancy seeds a wall");
        let reference=SurfaceSnapshot::from_points(&points(&cells),config(),SIZE).unwrap();
        assert_equivalent(&low,&reference);
        cells.remove(&(25,16,6));
        let restored=builder.build(&points(&cells),SIZE).unwrap().snapshot;
        assert_eq!(at(&restored),0.4);
        for z in 1..20 {cells.insert((25,16,z));}
        let tall=builder.build(&points(&cells),SIZE).unwrap().snapshot;
        assert!((at(&tall)-SIZE*0.5).abs()<1e-6,
            "a wall with its exposed top above robot height still blocks at body height");
    }

    #[test]
    fn explicit_column_replacements_match_full_reference_and_preserve_failed_baseline() {
        let source = 0.025;
        let mut data = points(&scene());
        let mut builder = SurfaceMapBuilder::new(config());
        let prepare = |points: &[(f32, f32, f32)], changed: &[(i32, i32)]| {
            let columns = voxel_columns(points, SIZE).unwrap();
            let heights = height_bounds(points, SIZE, source);
            changed
                .iter()
                .map(|&(x, y)| SurfaceColumnUpdate {
                    column: [x, y],
                    cells: columns
                        .get(&(x, y))
                        .into_iter()
                        .flatten()
                        .map(|&z| (z, heights[&(x, y, z)]))
                        .collect(),
                })
                .collect::<Vec<_>>()
        };
        let keys = voxel_columns(&data, SIZE)
            .unwrap()
            .keys()
            .copied()
            .collect::<Vec<_>>();
        let first = builder
            .build_column_updates(&prepare(&data, &keys), source, true, data.len())
            .unwrap();
        assert_equivalent(
            &first.snapshot,
            &SurfaceSnapshot::from_points(&data, config(), source).unwrap(),
        );
        for (column, z) in [
            ((20, 16), Some(0.375)),
            ((20, 16), Some(0.35)),
            ((30, 16), None),
            ((20, 16), None),
        ] {
            if let Some(z) = z {
                data.retain(|&(x, y, h)| {
                    voxelize((x, y, h), SIZE).0 != column.0
                        || voxelize((x, y, h), SIZE).1 != column.1
                        || h < 0.15
                });
                data.push((
                    (column.0 as f32 + 0.5) * SIZE,
                    (column.1 as f32 + 0.5) * SIZE,
                    z,
                ));
            } else {
                data.retain(|&p| {
                    let key = voxelize(p, SIZE);
                    (key.0, key.1) != column
                });
            }
            let updates = prepare(&data, &[column]);
            let actual = builder
                .build_column_updates(&updates, source, false, 100)
                .unwrap();
            assert_equivalent(
                &actual.snapshot,
                &SurfaceSnapshot::from_points(&data, config(), source).unwrap(),
            );
            let bad = [SurfaceColumnUpdate {
                column: [0, 0],
                cells: vec![(0, [f32::NAN, 1.0])],
            }];
            assert!(builder.build_column_updates(&bad, source, true, 1).is_err());
            assert!(Arc::ptr_eq(
                &actual.snapshot,
                &builder
                    .build_column_updates(&[], source, false, 0)
                    .unwrap()
                    .snapshot
            ));
        }
    }

    #[test]
    fn authoritative_column_updates_can_remove_last_column_without_mutating_old_snapshot() {
        let mut builder=SurfaceMapBuilder::new(config());
        let original=builder.build_column_updates(&[SurfaceColumnUpdate {
            column:[0,0],cells:vec![(0,[0.0,SIZE])],
        }],SIZE,true,1).unwrap().snapshot;
        let empty=builder.build_column_updates(&[SurfaceColumnUpdate {
            column:[0,0],cells:vec![],
        }],SIZE,false,0).unwrap();
        assert_eq!(empty.changed_columns,vec![[0,0]]);
        assert_eq!(empty.snapshot.data.occupied_voxels,0);
        assert_eq!(empty.snapshot.cell_keys().count(),0);
        assert_eq!(empty.snapshot.query_ground([0.05,0.05],0.1).height,None);
        assert_eq!(original.data.occupied_voxels,1);
        assert!(Arc::ptr_eq(&empty.snapshot,&builder.build_column_updates(&[],SIZE,false,0).unwrap().snapshot));
        // The authoritative column API accepts a complete first empty root too.
        let mut fresh=SurfaceMapBuilder::new(config());
        assert_eq!(fresh.build_column_updates(&[],SIZE,true,0).unwrap().snapshot.cell_keys().count(),0);
        // 有效空点数组同样表示未知地图，显式来源清空由外层拥有方撤销。
        let raw_empty=builder.build(&[],SIZE).unwrap().snapshot;
        assert_eq!(raw_empty.data.occupied_voxels,0);
        assert_eq!(raw_empty.cell_keys().count(),0);
        assert!(builder.build_column_updates(&[SurfaceColumnUpdate {
            column:[0,0],cells:vec![(0,[f32::NAN,SIZE])],
        }],SIZE,true,1).is_err());
        assert!(Arc::ptr_eq(&empty.snapshot,&builder.build_column_updates(&[],SIZE,false,0).unwrap().snapshot));
    }

    #[test]
    fn unchanged_dirty_columns_still_validate_and_mixed_changes_preserve_baseline() {
        let data = points(&scene());
        let source = SIZE * 0.5;
        let columns = voxel_columns(&data, SIZE).unwrap();
        let bounds = height_bounds(&data, SIZE, source);
        let mut updates: Vec<_> = columns
            .iter()
            .map(|(&(x, y), zs)| SurfaceColumnUpdate {
                column: [x, y],
                cells: zs.iter().map(|&z| (z, bounds[&(x, y, z)])).collect(),
            })
            .collect();
        let mut builder = SurfaceMapBuilder::new(config());
        let first = builder.build(&data, source).unwrap().snapshot;
        let reused = builder
            .build_column_updates(&updates, source, false, data.len())
            .unwrap();
        assert_eq!(reused.stats.kind, SurfaceBuildKind::Reused);
        assert!(Arc::ptr_eq(&first, &reused.snapshot));
        let removed = updates[0].column;
        updates[0].cells.clear();
        // A later invalid unchanged-looking column must not commit an earlier deletion.
        let duplicate = updates[1].clone();
        updates.push(duplicate);
        assert!(builder
            .build_column_updates(&updates, source, false, data.len())
            .is_err());
        assert!(Arc::ptr_eq(
            &first,
            &builder
                .build_column_updates(&[], source, false, 0)
                .unwrap()
                .snapshot
        ));
        updates.pop();
        let actual = builder
            .build_column_updates(&updates, source, false, data.len())
            .unwrap();
        assert_eq!(actual.changed_columns, vec![removed]);
        let remaining: Vec<_> = data
            .into_iter()
            .filter(|&p| {
                let (x, y, _) = voxelize(p, SIZE);
                [x, y] != removed
            })
            .collect();
        assert_equivalent(
            &actual.snapshot,
            &SurfaceSnapshot::from_points(&remaining, config(), source).unwrap(),
        );
        assert_eq!(first.data.occupied_columns.len(), columns.len());
        // Validation still applies even when the source/planner precision ignores bounds.
        let mut same_precision = SurfaceMapBuilder::new(config());
        same_precision.build(&remaining, SIZE).unwrap();
        let invalid = [SurfaceColumnUpdate {
            column: updates[1].column,
            cells: vec![(0, [f32::NAN, 1.0])],
        }];
        assert!(same_precision
            .build_column_updates(&invalid, SIZE, false, 1)
            .is_err());
    }

    #[test]
    fn changed_measurements_keep_source_heights_and_reuse_unchanged_graph() {
        let data = points(&scene());
        let mut builder = SurfaceMapBuilder::new(config());
        let first = builder.build(&data, SIZE).unwrap();
        let mut reordered: Vec<_> = data
            .iter()
            .rev()
            .map(|&(x, y, z)| (x + 0.01, y + 0.01, z + 0.01))
            .collect();
        reordered.extend_from_slice(&data[..20]);
        let next = builder.build(&reordered, SIZE).unwrap();
        assert_eq!(next.stats.kind, SurfaceBuildKind::Reused);
        assert!(!Arc::ptr_eq(&first.snapshot, &next.snapshot));
        assert!(Arc::ptr_eq(&first.snapshot.data.graph, &next.snapshot.data.graph));
        let before=first.snapshot.navigation_measured_points().unwrap();
        let after=next.snapshot.navigation_measured_points().unwrap();
        assert_eq!(before.len(),data.len());assert_eq!(after.len(),before.len());
        assert_eq!(before[0],[f64::from(data[0].0),f64::from(data[0].1),f64::from(data[0].2)]);
        let expected_x=(f64::from(data[0].0)+f64::from(data[0].0+0.01))*0.5;
        assert!((after[0][0]-expected_x).abs()<1e-12,"保留同源体素内两次实测点的均值");
        assert_ne!(before[0],after[0]);
        assert!(!next.stats.global_cost_refresh);
    }

    #[test]
    fn incremental_columns_match_full_rebuild_through_deletion_reconnection_and_layers() {
        let mut cells = scene();
        let original = points(&cells);
        let mut builder = SurfaceMapBuilder::new(config());
        let old = builder.build(&original, SIZE).unwrap().snapshot;
        let old_signature = old.cell_keys().collect::<BTreeSet<_>>();
        for (cell, insert) in [
            ((20, 16, 4), true),
            ((30, 16, 0), false),
            ((20, 16, 30), false),
            ((20, 16, 4), false),
            ((30, 16, 0), true),
            ((20, 16, 30), true),
            ((5, 5, 0), false),
            ((55, 25, 0), false),
            ((5, 5, 0), true),
            ((55, 25, 0), true),
        ] {
            if insert {
                cells.insert(cell);
            } else {
                cells.remove(&cell);
            }
            let input = points(&cells);
            let updated = builder.build(&input, SIZE).unwrap();
            assert_eq!(updated.stats.kind, SurfaceBuildKind::Incremental);
            assert!(updated.stats.checked_cells < old_signature.len() / 3);
            let full = SurfaceSnapshot::from_points(&input, config(), SIZE).unwrap();
            assert_equivalent(&updated.snapshot, &full);
            for z in [0.1, 3.1] {
                let from = [1.05, 1.05, z];
                let to = [4.55, 2.05, z];
                assert_eq!(
                    updated.snapshot.test_connected(from, to),
                    full.test_connected(from, to)
                );
                assert_eq!(
                    updated
                        .snapshot
                        .reachable_frontiers(from, &[])
                        .unwrap()
                        .iter()
                        .map(|f| f.cell)
                        .collect::<BTreeSet<_>>(),
                    full.reachable_frontiers(from, &[])
                        .unwrap()
                        .iter()
                        .map(|f| f.cell)
                        .collect()
                );
            }
            assert_eq!(old.cell_keys().collect::<BTreeSet<_>>(), old_signature);
        }
        assert_equivalent(
            &old,
            &SurfaceSnapshot::from_points(&original, config(), SIZE).unwrap(),
        );
    }

    #[test]
    fn occupied_evidence_can_change_without_rebuilding_support_graph() {
        let mut cells = scene();
        let mut builder = SurfaceMapBuilder::new(config());
        let old = builder.build(&points(&cells), SIZE).unwrap().snapshot;
        cells.insert((15, 15, -1));
        let next = builder.build(&points(&cells), SIZE).unwrap();
        assert_eq!(next.stats.kind, SurfaceBuildKind::Incremental);
        assert_eq!(next.stats.changed_surface_cells, 0);
        assert!(!next.stats.global_cost_refresh);
        assert!(Arc::ptr_eq(
            &old.data.physical_graph,
            &next.snapshot.data.physical_graph
        ));
        assert!(!Arc::ptr_eq(&old, &next.snapshot));
        assert!(!old.data.occupied_columns[&(15, 15)].contains(&-1));
        assert!(next.snapshot.data.occupied_columns[&(15, 15)].contains(&-1));
    }

    #[test]
    fn stair_profile_changes_revalidate_connections_beyond_adjacent_column() {
        let mut cells: BTreeSet<_> = (0..72)
            .flat_map(|x| {
                let z = if x < 12 {
                    0
                } else if x < 48 {
                    ((x - 12) / 6 + 1) * 2
                } else {
                    12
                };
                (0..24).map(move |y| (x, y, z))
            })
            .collect();
        let mut builder = SurfaceMapBuilder::new(config());
        builder.build(&points(&cells), SIZE).unwrap();
        for x in [10, 11, 12, 13, 14, 15, 16] {
            let z = if x < 12 { 0 } else { 2 };
            cells.remove(&(x, 12, z));
            let input = points(&cells);
            let next = builder.build(&input, SIZE).unwrap();
            assert_eq!(next.stats.kind, SurfaceBuildKind::Incremental);
            assert_equivalent(
                &next.snapshot,
                &SurfaceSnapshot::from_points(&input, config(), SIZE).unwrap(),
            );
        }
    }

    #[test]
    fn failed_input_does_not_commit_and_precision_or_large_changes_rebuild() {
        let cells = scene();
        let data = points(&cells);
        let mut builder = SurfaceMapBuilder::new(config());
        let first = builder.build(&data, SIZE).unwrap();
        assert!(builder.build(&[(f32::NAN, 0.0, 0.0)], SIZE).is_err());
        assert!(builder.build(&data, SIZE * 2.0).is_err());
        assert!(Arc::ptr_eq(
            &first.snapshot,
            &builder.build(&data, SIZE).unwrap().snapshot
        ));
        let precision = builder.build(&data, SIZE * 0.5).unwrap();
        assert_eq!(precision.stats.kind, SurfaceBuildKind::Full);
        assert_eq!(precision.snapshot.source_voxel_size_m(), SIZE * 0.5);
        let shifted: Vec<_> = data.iter().map(|&(x, y, z)| (x + 20.0, y, z)).collect();
        assert_eq!(
            builder.build(&shifted, SIZE * 0.5).unwrap().stats.kind,
            SurfaceBuildKind::Full
        );
        builder.reset();
        assert_eq!(
            builder.build(&data, SIZE).unwrap().stats.kind,
            SurfaceBuildKind::Full
        );
    }
}

#[cfg(test)]
mod replay_diagnostics {
    use super::*;

    #[test]
    #[ignore = "需要显式提供合成闭环保存的PCD与支撑位置"]
    fn inspect_saved_surface_with_full_reference() {
        let bytes = std::fs::read(std::env::var("NUWAX_SURFACE_REPLAY_PCD").unwrap()).unwrap();
        let marker = b"DATA binary\n";
        let offset = bytes
            .windows(marker.len())
            .position(|value| value == marker)
            .unwrap()
            + marker.len();
        let points: Vec<_> = bytes[offset..]
            .chunks_exact(12)
            .map(|point| {
                (
                    f32::from_le_bytes(point[0..4].try_into().unwrap()),
                    f32::from_le_bytes(point[4..8].try_into().unwrap()),
                    f32::from_le_bytes(point[8..12].try_into().unwrap()),
                )
            })
            .collect();
        let config = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.3,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.15,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            step_threshold_m: 0.15,
            max_slope_deg: 18.0,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 2.0,
            goal_tolerance: 0.2,
            viz_publish_hz: 0.0,
        };
        let point: [f64; 3] =
            serde_json::from_str(&std::env::var("NUWAX_SURFACE_REPLAY_POSITION").unwrap()).unwrap();
        let surface = SurfaceSnapshot::from_points(&points, config, 0.05).unwrap();
        println!("full_reference={:?}", surface.locate_support(point));
        let x = (point[0] / 0.1).floor() as i32;
        let y = (point[1] / 0.1).floor() as i32;
        for dx in -2..=2 {
            for dy in -2..=2 {
                let column = (x + dx, y + dy);
                let values: Vec<_> = surface
                    .data
                    .graph
                    .surface_lookup
                    .get(&column)
                    .into_iter()
                    .flatten()
                    .map(|&z| {
                        let id = surface
                            .data
                            .graph
                            .cells
                            .id((column.0, column.1, z))
                            .unwrap();
                        (z, surface.data.graph.wall_state.dist[id as usize])
                    })
                    .collect();
                println!(
                    "column={column:?} occupied={:?} support_clearance={values:?}",
                    surface.data.occupied_columns.get(&column)
                );
            }
        }
    }
}
