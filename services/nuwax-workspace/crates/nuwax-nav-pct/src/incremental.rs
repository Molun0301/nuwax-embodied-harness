//! Persistent column edits. The caller supplies a complete delta and its immutable basis.
//! Distances may change arbitrarily far away; only topology uses a one-column stencil.
use crate::cancellation::checkpoint;
use crate::{Cell, Tomogram};
use ahash::{AHashMap, AHashSet};
#[cfg(feature = "mls-adapter")]
use nuwax_nav_mls_planner::SurfaceSnapshot;
use std::cmp::Ordering;
use std::collections::BTreeMap;

/// `None` deletes the column; an empty vector retains a known, non-walkable column.
#[derive(Clone, Debug)]
pub struct ColumnUpdate {
    pub column: [i32; 2],
    pub bands: Option<Vec<[f64; 2]>>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct IncrementalStats {
    pub rebuilt: bool,
    pub columns_prepared: usize,
    pub topology_recomputed: usize,
    pub distance_invalidated: usize,
    pub distance_relaxed: usize,
    pub terrain_replayed: usize,
    pub upper_relaxed: usize,
}

/// The source owner must include removed columns and native ground-record changes.
/// Map replacement, relocation, masks and incomplete/coalesced-away deltas use Rebuild.
#[cfg(feature = "mls-adapter")]
#[derive(Clone, Copy, Debug)]
pub enum SurfaceDelta<'a> {
    Rebuild,
    Unchanged {
        basis: &'a SurfaceSnapshot,
    },
    Columns {
        basis: &'a SurfaceSnapshot,
        resolution: f64,
        columns: &'a [[i32; 2]],
    },
}

#[derive(Clone, Copy, Debug)]
struct FloatKey(f64);
impl PartialEq for FloatKey {
    fn eq(&self, other: &Self) -> bool {
        self.0.to_bits() == other.0.to_bits()
    }
}
impl Eq for FloatKey {}
impl PartialOrd for FloatKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for FloatKey {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

/// Counted extrema avoid scanning every cell after removing a boundary/minimum.
#[derive(Clone, Debug, Default)]
pub(crate) struct Indices {
    resolution_bits: u64,
    cell_basis: Option<std::sync::Arc<()>>,
    x: im::OrdMap<i32, usize>,
    y: im::OrdMap<i32, usize>,
    heights: im::OrdMap<FloatKey, usize>,
    costs: im::OrdMap<FloatKey, usize>,
}
fn add<K: Ord + Clone>(index: &mut im::OrdMap<K, usize>, key: K) {
    let count = index.get(&key).copied().unwrap_or(0);
    index.insert(key, count + 1);
}
fn remove<K: Ord + Clone>(index: &mut im::OrdMap<K, usize>, key: K) {
    let count = *index.get(&key).expect("indexed value");
    if count == 1 {
        index.remove(&key);
    } else {
        index.insert(key, count - 1);
    }
}
impl Indices {
    pub(crate) fn from_map(map: &Tomogram) -> Result<Self, String> {
        let mut result = Self {
            resolution_bits: map.resolution.to_bits(),
            cell_basis: Some(map.cells.basis().clone()),
            ..Self::default()
        };
        for &xy in &map.known_columns {
            checkpoint()?;
            result.add_column(xy);
        }
        for cell in &map.cells {
            checkpoint()?;
            result.add_cell(cell);
        }
        Ok(result)
    }
    fn add_column(&mut self, xy: [i32; 2]) {
        add(&mut self.x, xy[0]);
        add(&mut self.y, xy[1]);
    }
    fn remove_column(&mut self, xy: [i32; 2]) {
        remove(&mut self.x, xy[0]);
        remove(&mut self.y, xy[1]);
    }
    fn add_cell(&mut self, cell: &Cell) {
        add(&mut self.heights, FloatKey(cell.ground));
        add(&mut self.costs, FloatKey(cell.cost));
    }
    fn remove_cell(&mut self, cell: &Cell) {
        remove(&mut self.heights, FloatKey(cell.ground));
        remove(&mut self.costs, FloatKey(cell.cost));
    }
}

/// Sparse mapping through dense swap-removals; unaffected cell IDs remain identities.
#[derive(Default)]
pub(crate) struct IdRemap {
    old_to_new: AHashMap<usize, Option<usize>>,
    new_to_old: AHashMap<usize, Option<usize>>,
}
impl IdRemap {
    pub(crate) fn old_to_new(&self, id: usize) -> Option<usize> {
        self.old_to_new.get(&id).copied().unwrap_or(Some(id))
    }
    pub(crate) fn new_to_old(&self, id: usize) -> Option<usize> {
        self.new_to_old.get(&id).copied().unwrap_or(Some(id))
    }
}

fn same_cell_geometry(a: &Cell, b: &Cell) -> bool {
    a.layer == b.layer
        && a.ground.to_bits() == b.ground.to_bits()
        && a.ground_interval.map(f64::to_bits) == b.ground_interval.map(f64::to_bits)
        && a.ceiling.to_bits() == b.ceiling.to_bits()
        && a.ground_approximation == b.ground_approximation
        && {
            #[cfg(feature = "mls-adapter")]
            {
                a.source_ground == b.source_ground
            }
            #[cfg(not(feature = "mls-adapter"))]
            {
                true
            }
        }
}

fn adjacent_columns(xy: [i32; 2], columns: &mut AHashSet<[i32; 2]>) {
    for dx in -1..=1 {
        for dy in -1..=1 {
            if let (Some(x), Some(y)) = (xy[0].checked_add(dx), xy[1].checked_add(dy)) {
                columns.insert([x, y]);
            }
        }
    }
}

impl Tomogram {
    /// Derive another immutable graph without copying the complete graph. Cold builds
    /// and incremental replay share a stable geometry seed order, independent of dense
    /// storage IDs. FIFO relaxation, neighbor order and epsilon remain unchanged.
    pub fn with_column_updates(
        &self,
        updates: impl IntoIterator<Item = ColumnUpdate>,
    ) -> Result<(Self, IncrementalStats), String> {
        #[cfg(feature = "mls-adapter")]
        if self.source.is_some() {
            return Err("源地图必须通过带 basis 的地表更新接入".into());
        }
        self.edit_columns(updates, |_, _| None)
    }

    fn edit_columns(
        &self,
        updates: impl IntoIterator<Item = ColumnUpdate>,
        measured: impl Fn([i32; 2], &[[f64; 2]]) -> Option<Vec<crate::tomogram::GroundCandidate>>,
    ) -> Result<(Self, IncrementalStats), String> {
        checkpoint()?;
        self.check_incremental_basis()?;
        let mut updates_by_column = BTreeMap::new();
        for update in updates {
            checkpoint()?;
            updates_by_column.insert(update.column, update.bands);
        }
        let mut stats = IncrementalStats::default();
        // Clone only persistent roots. Existing Arc owners continue using the old graph.
        let mut next = self.clone();
        let mut remap = IdRemap::default();
        let mut topology_columns = AHashSet::new();
        let mut removed_successors = AHashSet::new();
        // Removal precedes insertion so every swap source still has an old identity.
        let mut prepared = Vec::new();
        for (column, bands) in updates_by_column {
            checkpoint()?;
            stats.columns_prepared += 1;
            let local = if let Some(bands) = bands {
                Some(Self::from_columns_with_layers(
                    self.resolution,
                    self.source_quantum,
                    [(column, bands)],
                    self.body_height,
                    self.half_width,
                    self.step_up,
                    self.step_down,
                    0.0,
                    self.clearance_radius,
                    self.occupied_volume,
                    &measured,
                )?)
            } else {
                None
            };
            let unchanged = match &local {
                None => !self.known_columns.contains(&column),
                Some(local) => {
                    self.known_columns.contains(&column)
                        && self.occupancy.get(&column) == local.occupancy.get(&column)
                        && {
                            let old = self.columns.get(&column).map_or(&[][..], Vec::as_slice);
                            old.len() == local.cells.len()
                                && old
                                    .iter()
                                    .zip(&local.cells)
                                    .all(|(&id, cell)| same_cell_geometry(&self.cells[id], cell))
                        }
                }
            };
            if unchanged {
                continue;
            }
            adjacent_columns(column, &mut topology_columns);
            while let Some(id) = next
                .columns
                .get(&column)
                .and_then(|ids| ids.last())
                .copied()
            {
                checkpoint()?;
                let origin = remap
                    .new_to_old(id)
                    .expect("only old cells before insertion");
                removed_successors.extend(self.cells[origin].neighbors.iter().copied());
                let removed = next.cells[id].clone();
                next.seed_order.remove(&removed, id)?;
                next.indices.remove_cell(&removed);
                next.columns
                    .get_mut(&column)
                    .unwrap()
                    .retain(|&value| value != id);
                let last = next.cells.len() - 1;
                remap.old_to_new.insert(origin, None);
                if id != last {
                    let moved_origin = remap
                        .new_to_old(last)
                        .expect("only old cells before insertion");
                    let moved = next.cells[last].clone();
                    next.seed_order.move_id(&moved, last, id)?;
                    adjacent_columns(moved.column, &mut topology_columns);
                    for value in next.columns.get_mut(&moved.column).unwrap() {
                        if *value == last {
                            *value = id;
                        }
                    }
                    next.cells[id] = moved;
                    remap.old_to_new.insert(moved_origin, Some(id));
                    remap.new_to_old.insert(id, Some(moved_origin));
                }
                next.cells.pop_back();
                remap.new_to_old.remove(&last);
            }
            next.columns.remove(&column);
            if next.known_columns.contains(&column) {
                next.known_columns.remove(&column);
                next.indices.remove_column(column);
            }
            next.occupancy.remove(&column);
            prepared.push((column, local));
        }
        for (column, local) in prepared {
            checkpoint()?;
            let Some(local) = local else {
                continue;
            };
            next.known_columns.insert(column);
            next.indices.add_column(column);
            next.occupancy
                .insert(column, local.occupancy.get(&column).unwrap().clone());
            for cell in local.cells {
                checkpoint()?;
                let id = next.cells.len();
                next.seed_order.insert(&cell, id)?;
                next.indices.add_cell(&cell);
                next.cells.push_back(cell);
                next.columns.entry(column).or_default().push(id);
                remap.new_to_old.insert(id, None);
            }
        }
        let mut dirty = AHashSet::new();
        for column in topology_columns {
            checkpoint()?;
            if let Some(ids) = next.columns.get(&column) {
                dirty.extend(ids.iter().copied());
            }
        }
        // Rebuild all references to moved/removed IDs. This stencil is only topology;
        // the dependency queues below are deliberately not bounded by its radius.
        for &id in &dirty {
            checkpoint()?;
            let neighbors = next.neighbors_for(id);
            next.cells[id].neighbors = neighbors;
            stats.topology_recomputed += 1;
        }
        // Pure ID relocation does not change stable seed order or field semantics.
        // Compare only this topology stencil; no scan of unrelated graph cells.
        let mut field_dirty = AHashSet::new();
        for id in dirty {
            checkpoint()?;
            let Some(previous) = remap.new_to_old(id) else {
                field_dirty.insert(id);
                continue;
            };
            let a = &self.cells[previous];
            let b = &next.cells[id];
            let outgoing_changed = a.neighbors.len() != b.neighbors.len()
                || a.neighbors.iter().zip(&b.neighbors).any(|(&old, &new)| {
                    remap.old_to_new(old) != Some(new)
                        || self.cells[old].ground.to_bits() != next.cells[new].ground.to_bits()
                });
            let known_changed = [(1, 0), (-1, 0), (0, 1), (0, -1)]
                .into_iter()
                .any(|(dx, dy)| {
                    let column = [b.column[0] + dx, b.column[1] + dy];
                    self.known_columns.contains(&column) != next.known_columns.contains(&column)
                });
            if !same_cell_geometry(a, b) || outgoing_changed || known_changed {
                field_dirty.insert(id);
            }
        }
        // A deleted source can affect a one-way successor even when the successor's
        // own out-neighbors/base are unchanged. Preserve every such surviving root.
        for previous in removed_successors {
            if let Some(id) = remap.old_to_new(previous) {
                field_dirty.insert(id);
            }
        }
        let mut changed_costs =
            crate::incremental_fields::update(self, &mut next, &remap, &field_dirty, &mut stats)?;
        changed_costs.extend(field_dirty);
        for id in changed_costs {
            checkpoint()?;
            let cell = &next.cells[id];
            let preferred = next.clearance_radius.max(next.half_width) + next.resolution;
            let wall = preferred / (cell.clearance + next.resolution * 0.5);
            let missing = ((preferred - cell.support_clearance) / preferred).clamp(0.0, 1.0);
            let cost = 1.0 + wall * wall + 3.0 * missing * missing + cell.terrain;
            remove(&mut next.indices.costs, FloatKey(cell.cost));
            add(&mut next.indices.costs, FloatKey(cost));
            next.cells[id].cost = cost;
        }
        next.xy_bounds = next
            .indices
            .x
            .get_min()
            .zip(next.indices.y.get_min())
            .zip(next.indices.x.get_max().zip(next.indices.y.get_max()))
            .map(|((x0, y0), (x1, y1))| [[x0.0, y0.0], [x1.0, y1.0]]);
        next.ground_bounds = next
            .indices
            .heights
            .get_min()
            .zip(next.indices.heights.get_max())
            .map(|(lo, hi)| [lo.0 .0, hi.0 .0]);
        next.minimum_cost = next
            .indices
            .costs
            .get_min()
            .map_or(crate::reference::UNKNOWN_COST, |(cost, _)| {
                crate::reference::UNKNOWN_COST.min(cost.0)
            });
        next.cells.seal_indices();
        next.indices.cell_basis = Some(next.cells.basis().clone());
        checkpoint()?;
        Ok((next, stats))
    }

    fn check_incremental_basis(&self) -> Result<(), String> {
        if !self.cells.indices_current()
            || self.resolution.to_bits() != self.indices.resolution_bits
            || !self
                .indices
                .cell_basis
                .as_ref()
                .is_some_and(|basis| std::sync::Arc::ptr_eq(basis, self.cells.basis()))
        {
            return Err("层析单元或分辨率已直接修改，需重新构图后再作增量更新".into());
        }
        Ok(())
    }

    /// Opt-in source update using the same stable seed order as cold builds.
    /// Local windows currently rebuild within the same window.
    #[cfg(feature = "mls-adapter")]
    pub fn updated_from_surface(
        &self,
        source: &SurfaceSnapshot,
        delta: SurfaceDelta<'_>,
    ) -> Result<(Self, IncrementalStats), String> {
        checkpoint()?;
        let rebuild = || {
            let map = match self.window {
                Some((center, radius)) => Self::local_from_surface(source, center, radius),
                None => Self::from_surface(source),
            };
            map.map(|map| {
                (
                    map,
                    IncrementalStats {
                        rebuilt: true,
                        ..IncrementalStats::default()
                    },
                )
            })
        };
        let basis = match delta {
            SurfaceDelta::Rebuild => return rebuild(),
            SurfaceDelta::Unchanged { basis } | SurfaceDelta::Columns { basis, .. } => basis,
        };
        if !self.matches_surface(basis) || !same_query_semantics(basis, source) {
            return rebuild();
        }
        self.check_incremental_basis()?;
        if crate::same_source_geometry(basis, source) {
            let mut result = self.clone();
            result.source = Some(source.clone());
            checkpoint()?;
            return Ok((result, IncrementalStats::default()));
        }
        let SurfaceDelta::Columns {
            resolution,
            columns,
            ..
        } = delta
        else {
            return rebuild();
        };
        let fine = |s: &SurfaceSnapshot| {
            s.observations().is_some_and(|observations| {
                observations.is_complete_geometry_source()
                    && observations.voxel_size_m().to_bits() == s.source_voxel_size_m().to_bits()
            })
        };
        if !fine(basis)
            || !fine(source)
            || resolution.to_bits() != self.resolution.to_bits()
            || resolution.to_bits() != f64::from(source.source_voxel_size_m()).to_bits()
        {
            return rebuild();
        }
        let observations = source.observations().unwrap();
        let updates = columns.iter().map(|&column| {
            let bands = observations
                .column_levels(column[0], column[1])
                .map(|levels| {
                    if !source.navigation_column_allowed(column, resolution) {
                        return Vec::new();
                    }
                    levels
                        .iter()
                        .map(|&z| [f64::from(z) * resolution, f64::from(z + 1) * resolution])
                        .collect()
                });
            ColumnUpdate { column, bands }
        });
        let (mut result, stats) = self.edit_columns(updates, |column, bands| {
            crate::source_ground::candidates(source, column, resolution, bands)
        })?;
        result.source = Some(source.clone());
        checkpoint()?;
        Ok((result, stats))
    }
}

#[cfg(feature = "mls-adapter")]
fn same_query_semantics(a: &SurfaceSnapshot, b: &SurfaceSnapshot) -> bool {
    fn parameters(s: &SurfaceSnapshot) -> [u32; 13] {
        let c = s.config();
        [
            c.voxel_size,
            c.robot_height,
            c.surface_closing_radius,
            c.node_spacing_m,
            c.wall_clearance_m,
            c.wall_buffer_m,
            c.wall_buffer_weight,
            c.step_threshold_m,
            c.max_slope_deg,
            c.min_step_tread_depth_m,
            c.step_penalty_weight,
            c.goal_tolerance,
            s.source_voxel_size_m(),
        ]
        .map(f32::to_bits)
    }
    a.body_envelope() == b.body_envelope()
        && a.config().world_frame == b.config().world_frame
        && a.ground_evidence() == b.ground_evidence()
        && parameters(a) == parameters(b)
}
