//! 二维派生安全图、前沿判定和连通簇的增量维护。
use crate::detector::neighbors8;
use crate::grid::GridPoint;
use nuwax_nav_types_messages::grid::{OccupancyCell, OccupancyGrid2D};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

/// 最近一次查询的派生工作量，不含当前机器人可达搜索。
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct IncrementalFrontierStats {
    /// 本帧实际变化的原始规划格数。
    pub changed_cells: usize,
    /// 重新验证轮廓净空的中心格数。
    pub safe_cells_recomputed: usize,
    /// 重新评估局部前沿几何的格数。
    pub frontier_cells_recomputed: usize,
    /// 重新构造的连通簇数。
    pub components_rebuilt: usize,
    /// 原样保留的连通簇数。
    pub components_retained: usize,
    /// 布局改变或首次输入是否触发完整初始化。
    pub layout_reset: bool,
}

#[derive(Default)]
pub(crate) struct PlanarFrontierCache {
    input: Option<OccupancyGrid2D>,
    safe: Option<OccupancyGrid2D>,
    offsets: Vec<(i64, i64)>,
    geometric: BTreeSet<usize>,
    active: Vec<bool>,
    active_indices: BTreeSet<usize>,
    labels: Vec<u64>,
    components: BTreeMap<u64, Vec<GridPoint>>,
    next_id: u64,
    pub(crate) stats: IncrementalFrontierStats,
}

impl PlanarFrontierCache {
    pub(crate) fn update_grid(
        &mut self,
        grid: &OccupancyGrid2D,
        offsets: &[(i64, i64)],
        mut frontier: impl FnMut(GridPoint) -> bool,
    ) -> OccupancyGrid2D {
        let same_layout = self.input.as_ref().is_some_and(|old| {
            old.origin == grid.origin
                && old.resolution == grid.resolution
                && old.width == grid.width
                && old.height == grid.height
                && old.cells.len() == grid.cells.len()
        });
        let changed: Vec<_> = if same_layout {
            self.input
                .as_ref()
                .unwrap()
                .cells
                .iter()
                .zip(&grid.cells)
                .enumerate()
                .filter_map(|(i, (a, b))| (a != b).then_some(i))
                .collect()
        } else {
            (0..grid.cells.len()).collect()
        };
        self.stats = IncrementalFrontierStats {
            changed_cells: changed.len(),
            layout_reset: !same_layout,
            ..Default::default()
        };
        if !same_layout {
            self.geometric.clear();
            self.active_indices.clear();
            self.components.clear();
            self.active = vec![false; grid.cells.len()];
            self.labels = vec![0; grid.cells.len()];
            self.next_id = 1;
            self.input = Some(grid.clone());
            self.safe = Some(grid.clone());
        } else {
            let input = self.input.as_mut().unwrap();
            for &i in &changed {
                input.cells[i] = grid.cells[i];
            }
        }
        let point = |i: usize| GridPoint::new(i as u32 % grid.width, i as u32 / grid.width);
        let full_safe = !same_layout
            || self.offsets != offsets
            || changed.len().saturating_mul(offsets.len().max(1)) >= grid.cells.len();
        let dirty_safe: Vec<_> = if full_safe {
            (0..grid.cells.len()).collect()
        } else {
            changed
                .iter()
                .flat_map(|&i| {
                    let p = point(i);
                    offsets.iter().filter_map(move |&(dx, dy)| {
                        let x = i64::from(p.x).checked_sub(dx)?;
                        let y = i64::from(p.y).checked_sub(dy)?;
                        (x >= 0
                            && y >= 0
                            && x < i64::from(grid.width)
                            && y < i64::from(grid.height))
                        .then(|| (y as usize) * grid.width as usize + x as usize)
                    })
                })
                .chain(changed.iter().copied())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect()
        };
        self.offsets = offsets.to_vec();
        self.stats.safe_cells_recomputed = dirty_safe.len();
        let safe = self.safe.as_mut().unwrap();
        for i in dirty_safe {
            let p = point(i);
            let clear = offsets.iter().all(|&(dx, dy)| {
                i64::from(p.x)
                    .checked_add(dx)
                    .zip(i64::from(p.y).checked_add(dy))
                    .and_then(|(x, y)| Some((u32::try_from(x).ok()?, u32::try_from(y).ok()?)))
                    .and_then(|(x, y)| grid.get(x, y))
                    .is_some_and(OccupancyCell::is_known_traversable)
            });
            safe.cells[i] = if clear {
                grid.cells[i]
            } else {
                OccupancyCell::Inflated
            };
        }
        let dirty_frontier: Vec<_> =
            if !same_layout || changed.len().saturating_mul(9) >= grid.cells.len() {
                (0..grid.cells.len()).collect()
            } else {
                changed
                    .iter()
                    .flat_map(|&i| {
                        std::iter::once(i).chain(
                            neighbors8(point(i), grid)
                                .iter()
                                .map(|p| (p.y * grid.width + p.x) as usize),
                        )
                    })
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect()
            };
        self.stats.frontier_cells_recomputed = dirty_frontier.len();
        for i in dirty_frontier {
            if frontier(point(i)) {
                self.geometric.insert(i);
            } else {
                self.geometric.remove(&i);
            }
        }
        safe.clone()
    }

    pub(crate) fn update_components(
        &mut self,
        grid: &OccupancyGrid2D,
        mut reachable: impl FnMut(GridPoint) -> bool,
    ) -> &BTreeMap<u64, Vec<GridPoint>> {
        let point = |i: usize| GridPoint::new(i as u32 % grid.width, i as u32 / grid.width);
        let current: BTreeSet<_> = self
            .geometric
            .iter()
            .copied()
            .filter(|&i| reachable(point(i)))
            .collect();
        let changed: Vec<_> = current
            .symmetric_difference(&self.active_indices)
            .copied()
            .collect();
        let mut affected = BTreeSet::new();
        for &i in &changed {
            for index in std::iter::once(i).chain(
                neighbors8(point(i), grid)
                    .iter()
                    .map(|p| (p.y * grid.width + p.x) as usize),
            ) {
                if self.labels[index] != 0 {
                    affected.insert(self.labels[index]);
                }
            }
            self.active[i] = current.contains(&i);
        }
        let mut work = BTreeSet::new();
        for id in affected {
            if let Some(cells) = self.components.remove(&id) {
                for p in cells {
                    let i = (p.y * grid.width + p.x) as usize;
                    self.labels[i] = 0;
                    if self.active[i] {
                        work.insert(i);
                    }
                }
            }
        }
        work.extend(changed.iter().copied().filter(|&i| self.active[i]));
        self.stats.components_retained = self.components.len();
        self.stats.components_rebuilt = 0;
        for start in work {
            if self.labels[start] != 0 || !self.active[start] {
                continue;
            }
            while self.next_id == 0 || self.components.contains_key(&self.next_id) {
                self.next_id = self.next_id.checked_add(1).unwrap_or(1);
            }
            let id = self.next_id;
            self.next_id = self.next_id.checked_add(1).unwrap_or(1);
            let mut cells = Vec::new();
            let mut queue = VecDeque::from([start]);
            self.labels[start] = id;
            while let Some(i) = queue.pop_front() {
                let p = point(i);
                cells.push(p);
                for next in neighbors8(p, grid).iter() {
                    let index = (next.y * grid.width + next.x) as usize;
                    if self.active[index] && self.labels[index] == 0 {
                        self.labels[index] = id;
                        queue.push_back(index);
                    }
                }
            }
            self.components.insert(id, cells);
            self.stats.components_rebuilt += 1;
        }
        self.active_indices = current;
        &self.components
    }
}
