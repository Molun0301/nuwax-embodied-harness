//! CoveragePatrolRouter：在候选目标中估算"沿路径会新增覆盖的 visited cell 数"，选最大值。
//!
//! 不复制 A*：通过外部注入的 `PathEstimator` trait 调用 A*，避免依赖具体算法 crate。
//! 默认实现：`StraightLineEstimator` 仅作占位/测试，真实 production 应注入 `nuwax-nav-astar`。

use std::cmp::Reverse;
use std::collections::hash_map::DefaultHasher;
use std::collections::BinaryHeap;
use std::hash::{Hash, Hasher};

use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::routers::{GridSnapshot, GridSnapshotExt, PatrolRouter, PatrollingContext};
use crate::visitation_history::VisitationHistory;

/// 路径估算 trait（由调用方注入，避免依赖具体算法 crate）
pub trait PathEstimator: Send {
    /// 计算 cell center -> pose 路径长度（cell idx 数）。
    /// 用 -1 表示无路径。
    fn estimate_cell_path(
        &self,
        grid: &GridSnapshot,
        from_cell: (usize, usize),
        to_cell: (usize, usize),
    ) -> i32;
}

/// 默认直线估算：仅用于占位/测试；生产应注入 `nuwax-nav-astar`。
#[derive(Default)]
pub struct StraightLineEstimator;

impl PathEstimator for StraightLineEstimator {
    fn estimate_cell_path(
        &self,
        grid: &GridSnapshot,
        from: (usize, usize),
        to: (usize, usize),
    ) -> i32 {
        let dx = (from.0 as i64 - to.0 as i64).abs();
        let dy = (from.1 as i64 - to.1 as i64).abs();
        let octile = dx.max(dy) + ((1.4_f64 - 1.0) * dx.min(dy) as f64) as i64;
        let free_count = grid.free_mask().iter().filter(|v| **v).count();
        if free_count == 0 {
            return -1;
        }
        octile as i32
    }
}

/// Obstacle-aware 8-connected A* estimator matching the original Python grid search.
#[derive(Default)]
pub struct GridAStarEstimator;

impl PathEstimator for GridAStarEstimator {
    fn estimate_cell_path(
        &self,
        grid: &GridSnapshot,
        from: (usize, usize),
        to: (usize, usize),
    ) -> i32 {
        astar_cell_count(grid, from, to).map_or(-1, |count| count as i32)
    }
}

fn astar_cell_count(
    grid: &GridSnapshot,
    from: (usize, usize),
    to: (usize, usize),
) -> Option<usize> {
    let width = grid.width as usize;
    let height = grid.height as usize;
    let total = width.checked_mul(height)?;
    if width == 0
        || height == 0
        || grid.data.len() < total
        || from.0 >= width
        || from.1 >= height
        || to.0 >= width
        || to.1 >= height
    {
        return None;
    }
    if from == to {
        return Some(1);
    }

    const DIRECTIONS: [(isize, isize, u32); 8] = [
        (0, 1, 100),
        (1, 0, 100),
        (0, -1, 100),
        (-1, 0, 100),
        (1, 1, 142),
        (1, -1, 142),
        (-1, 1, 142),
        (-1, -1, 142),
    ];

    let start = from.1 * width + from.0;
    let goal = to.1 * width + to.0;
    let mut scores = vec![None; total];
    let mut parents = vec![None; total];
    let mut closed = vec![false; total];
    let mut open = BinaryHeap::new();
    scores[start] = Some((0_u32, 0_u32));
    open.push(Reverse((0_u32, octile_cost(from, to), start)));

    while let Some(Reverse((_priority_cost, _priority_distance, current))) = open.pop() {
        if closed[current] {
            continue;
        }
        if current == goal {
            let mut count = 1;
            let mut cursor = current;
            while let Some(parent) = parents[cursor] {
                count += 1;
                cursor = parent;
            }
            return Some(count);
        }
        closed[current] = true;
        let current_xy = (current % width, current / width);
        let (current_cost, current_distance) = scores[current]?;

        for (dx, dy, movement_cost) in DIRECTIONS {
            let next_x = current_xy.0 as isize + dx;
            let next_y = current_xy.1 as isize + dy;
            if next_x < 0 || next_y < 0 || next_x >= width as isize || next_y >= height as isize {
                continue;
            }
            let next = next_y as usize * width + next_x as usize;
            if closed[next] {
                continue;
            }
            let cell = grid.data[next];
            if !(0..100).contains(&cell) {
                continue;
            }
            let candidate = (
                current_cost.saturating_add(cell as u32),
                current_distance.saturating_add(movement_cost),
            );
            if scores[next].is_some_and(|score| candidate >= score) {
                continue;
            }
            scores[next] = Some(candidate);
            parents[next] = Some(current);
            let next_xy = (next_x as usize, next_y as usize);
            open.push(Reverse((
                candidate.0,
                candidate.1.saturating_add(octile_cost(next_xy, to)),
                next,
            )));
        }
    }
    None
}

fn octile_cost(from: (usize, usize), to: (usize, usize)) -> u32 {
    let dx = from.0.abs_diff(to.0) as u32;
    let dy = from.1.abs_diff(to.1) as u32;
    (dx + dy)
        .saturating_mul(100)
        .saturating_sub(dx.min(dy).saturating_mul(58))
}

/// Voronoi 加权采样权重（修复 rework20 H-16 占位 API）。
///
/// 对齐 Python `CoveragePatrolRouter._sampling_weights`：用 `voronoi_gradient`
/// 算出的 cost 反向作为采样权重(skeleton cell cost≈0 → weight≈100,wall cell
/// cost≈100 → weight≈0),候选点按权重采样,使 patrol 不在开阔区域聚集。
///
/// 本期仅做 API 骨架 + 简单 fallback;真正的 voronoi_gradient 实现留待 P2。
#[derive(Debug, Clone, Copy)]
pub struct VoronoiWeights<'a> {
    /// `width * height` 浮点权重(None 表示未计算)
    pub weights: Option<&'a [f64]>,
    /// width (cells)
    pub width: u32,
    /// height (cells)
    pub height: u32,
}

/// 在候选 cell 中按 voronoi 权重采样(修复 rework20 H-16 骨架)。
///
/// - `weights=None`:退化为均匀随机抽样(与旧 Rust 行为一致)
/// - `weights=Some(w)`:按 `weight / sum(weight)` 加权抽样,样本数 `n` 由调用方指定
///
/// 返回采中的 cell 索引。**PARTIAL 实现**:当前仅支持"无权重时均匀抽样",
/// voronoi 加权采样等 P2 接入 `NavigationMap::make_voronoi_gradient` 后补齐。
///
/// 对应 Python:
/// ```python
/// weights = np.clip(100.0 - voronoi_cost, 0.0, 100.0)
/// probs = weights / weights.sum() if weights.sum() > 0 else None
/// chosen = np.random.choice(len(safe_indices), size=n, replace=False, p=probs)
/// ```
#[allow(dead_code)]
pub fn weighted_sample(
    candidates: &[usize],
    weights: VoronoiWeights<'_>,
    n: usize,
    rng: &mut StdRng,
) -> Vec<usize> {
    if candidates.is_empty() || n == 0 {
        return Vec::new();
    }
    let n = n.min(candidates.len());

    // PARTIAL: voronoi 加权采样待 P2
    // 完整实现:归一化 weights[candidates] 为 probs,rng 按 probs 无放回抽 n 个。
    // 当前 fallback:等距步长抽样,保证 deterministic 但忽略权重。
    match weights.weights {
        Some(_w) => {
            // 权重已就绪但 voronoi 加权采样尚未实现 → 退化为均匀抽样 + TODO 日志
            eprintln!("weighted_sample: voronoi 加权采样待 P2 实装,当前 fallback 均匀抽样");
            let step = (candidates.len() / n.max(1)).max(1);
            candidates.iter().step_by(step).take(n).copied().collect()
        }
        None => {
            // 无权重(未注入 voronoi_gradient):均匀抽样
            let step = (candidates.len() / n.max(1)).max(1);
            let _ = rng; // rng 留给 P2 voronoi 加权采样时使用
            candidates.iter().step_by(step).take(n).copied().collect()
        }
    }
}

/// Coverage router
pub struct CoveragePatrolRouter {
    history: VisitationHistory,
    grid: Option<GridSnapshot>,
    /// 保留 rng 用于 future tie-breaking（候选分数相同时随机选）；当前 deterministic。
    #[allow(dead_code)]
    rng: StdRng,
    candidates_to_consider: usize,
    estimator: Box<dyn PathEstimator>,
}

impl CoveragePatrolRouter {
    /// 构造
    pub fn new(
        clearance_radius_m: f64,
        candidates_to_consider: usize,
        rng_seed: Option<u64>,
    ) -> Self {
        let rng = match rng_seed {
            Some(s) => StdRng::seed_from_u64(s),
            None => StdRng::from_entropy(),
        };
        Self {
            history: VisitationHistory::new(clearance_radius_m),
            grid: None,
            rng,
            candidates_to_consider,
            estimator: Box::new(GridAStarEstimator),
        }
    }

    /// 设置自定义 estimator（应用例注入 A*）
    pub fn set_estimator(&mut self, est: Box<dyn PathEstimator>) {
        self.estimator = est;
    }
}

impl PatrolRouter for CoveragePatrolRouter {
    fn update_grid(&mut self, grid: GridSnapshot) {
        self.history.update_grid(
            grid.width as usize,
            grid.height as usize,
            grid.resolution,
            grid.origin_x,
            grid.origin_y,
        );
        self.grid = Some(grid);
    }

    fn update_pose(&mut self, pose: (f64, f64, f64)) {
        self.history.handle_odom(pose.0, pose.1);
    }

    fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
        // 无 ctx 路径:历史/中心点近似。保留向后兼容。
        let vw = self.history.width();
        let vh = self.history.height();
        if vw == 0 || vh == 0 {
            return None;
        }
        let visited = self.history.visited().unwrap_or(&[]);
        let total = vw * vh;
        let mut candidates: Vec<usize> = Vec::with_capacity(total);
        for i in 0..total {
            if visited.is_empty() || !visited[i] {
                candidates.push(i);
            }
        }
        if candidates.is_empty() {
            return None;
        }
        let n = self.candidates_to_consider.min(candidates.len());
        let step = candidates.len() / n.max(1).max(1);
        let mut best: Option<((f64, f64), i32)> = None;
        for i in (0..candidates.len()).step_by(step.max(1)) {
            let idx = candidates[i];
            let row = idx / vw;
            let col = idx % vw;
            let from = (vw / 2, vh / 2);
            let path = self.estimator.estimate_cell_path(
                &self.snapshot_for_estimate(vw, vh),
                from,
                (col, row),
            );
            if path < 0 {
                continue;
            }
            if best.is_none_or(|(_, p)| path < p) {
                best = Some(((col as f64, row as f64), path));
            }
        }
        let ((col_f, row_f), _) = best?;
        let res = self.history.resolution();
        // 修复 P0-14:用真实 origin,不再硬编码 0.0
        let ox = self.history.origin_x();
        let oy = self.history.origin_y();
        let x = ox + col_f * res;
        let y = oy + row_f * res;
        Some((x, y, 0.0))
    }

    fn next_goal_with(&mut self, ctx: &PatrollingContext) -> Option<(f64, f64, f64)> {
        // 修复 P0-14:用真实 grid_origin + odom (转 grid 坐标) + free_cells + clearance
        let vw = ctx.width as usize;
        let vh = ctx.height as usize;
        if vw == 0 || vh == 0 || ctx.resolution <= 0.0 || ctx.free_cells.is_empty() {
            return None;
        }
        let visited = self.history.visited().unwrap_or(&[]);
        let mut candidates: Vec<usize> = Vec::with_capacity(ctx.free_cells.len());
        for &idx in &ctx.free_cells {
            if idx >= vw * vh {
                continue;
            }
            let is_visited = visited.get(idx).copied().unwrap_or(false);
            if is_visited {
                continue;
            }
            candidates.push(idx);
        }
        if candidates.is_empty() {
            return None;
        }
        let n = self.candidates_to_consider.min(candidates.len());
        let step = candidates.len() / n.max(1).max(1);
        // from = odom 在 grid 中的栅格坐标 (col, row)
        let odom_x = ctx.odom.pose.position.x;
        let odom_y = ctx.odom.pose.position.y;
        let from_col = ((odom_x - ctx.grid_origin.x) / ctx.resolution).floor() as isize;
        let from_row = ((odom_y - ctx.grid_origin.y) / ctx.resolution).floor() as isize;
        let from_col = from_col.clamp(0, (vw as isize) - 1) as usize;
        let from_row = from_row.clamp(0, (vh as isize) - 1) as usize;
        let from = (from_col, from_row);
        let snap = self.snapshot_for_estimate_with_origin(vw, vh, ctx);
        let mut best: Option<((f64, f64), i32)> = None;
        for i in (0..candidates.len()).step_by(step.max(1)) {
            let idx = candidates[i];
            let row = idx / vw;
            let col = idx % vw;
            let path = self.estimator.estimate_cell_path(&snap, from, (col, row));
            if path < 0 {
                continue;
            }
            if best.is_none_or(|(_, p)| path < p) {
                best = Some(((col as f64, row as f64), path));
            }
        }
        let ((col_f, row_f), _) = best?;
        let x = ctx.grid_origin.x + col_f * ctx.resolution;
        let y = ctx.grid_origin.y + row_f * ctx.resolution;
        Some((x, y, 0.0))
    }

    fn saturation(&self) -> f32 {
        self.history.saturation()
    }

    fn reset(&mut self) {
        self.history.reset();
        self.grid = None;
    }
}

impl CoveragePatrolRouter {
    fn snapshot_for_estimate(&self, vw: usize, vh: usize) -> GridSnapshot {
        if let Some(grid) = self
            .grid
            .as_ref()
            .filter(|grid| grid.width as usize == vw && grid.height as usize == vh)
        {
            return grid.clone();
        }
        if vw == 0 || vh == 0 {
            return GridSnapshot {
                width: 0,
                height: 0,
                resolution: 0.0,
                origin_x: 0.0,
                origin_y: 0.0,
                data: vec![],
            };
        }
        let res = self.history.resolution();
        GridSnapshot {
            width: vw as u32,
            height: vh as u32,
            resolution: res,
            origin_x: self.history.origin_x(),
            origin_y: self.history.origin_y(),
            data: vec![0; vw * vh],
        }
    }

    fn snapshot_for_estimate_with_origin(
        &self,
        vw: usize,
        vh: usize,
        ctx: &PatrollingContext,
    ) -> GridSnapshot {
        if let Some(grid) = self.grid.as_ref().filter(|grid| {
            grid.width as usize == vw
                && grid.height as usize == vh
                && (grid.resolution - ctx.resolution).abs() < f64::EPSILON
                && (grid.origin_x - ctx.grid_origin.x).abs() < f64::EPSILON
                && (grid.origin_y - ctx.grid_origin.y).abs() < f64::EPSILON
        }) {
            return grid.clone();
        }
        if vw == 0 || vh == 0 {
            return GridSnapshot {
                width: 0,
                height: 0,
                resolution: 0.0,
                origin_x: 0.0,
                origin_y: 0.0,
                data: vec![],
            };
        }
        let mut data = vec![100; vw * vh];
        for &index in &ctx.free_cells {
            if let Some(cell) = data.get_mut(index) {
                *cell = 0;
            }
        }
        GridSnapshot {
            width: vw as u32,
            height: vh as u32,
            resolution: ctx.resolution,
            origin_x: ctx.grid_origin.x,
            origin_y: ctx.grid_origin.y,
            data,
        }
    }
}

#[allow(dead_code)]
fn _h(_s: &str) -> u64 {
    let mut h = DefaultHasher::new();
    _s.hash(&mut h);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returns_some_goal_with_grid() {
        let mut r = CoveragePatrolRouter::new(0.5, 4, Some(7));
        let grid = GridSnapshot {
            width: 5,
            height: 5,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; 25],
        };
        r.update_grid(grid);
        let g = r.next_goal();
        assert!(g.is_some());
    }

    #[test]
    fn no_goal_without_grid() {
        let mut r = CoveragePatrolRouter::new(0.5, 4, Some(1));
        assert!(r.next_goal().is_none());
    }

    // ===== H-16 weighted_sample 单测 =====

    #[test]
    fn weighted_sample_no_weights_returns_n_from_candidates() {
        // 修复 rework20 H-16:无 voronoi 权重时,均匀抽样 n 个。
        let candidates: Vec<usize> = (0..10).collect();
        let mut rng = StdRng::seed_from_u64(42);
        let weights = VoronoiWeights {
            weights: None,
            width: 5,
            height: 5,
        };
        let picked = weighted_sample(&candidates, weights, 3, &mut rng);
        assert_eq!(picked.len(), 3);
        for &i in &picked {
            assert!(candidates.contains(&i), "picked idx must be in candidates");
        }
    }

    #[test]
    fn weighted_sample_clamps_n_to_candidates_len() {
        // n > candidates.len() 时,clamp 到 candidates.len()。
        let candidates: Vec<usize> = (0..3).collect();
        let mut rng = StdRng::seed_from_u64(7);
        let weights = VoronoiWeights {
            weights: None,
            width: 3,
            height: 3,
        };
        let picked = weighted_sample(&candidates, weights, 100, &mut rng);
        assert_eq!(picked.len(), 3);
    }

    #[test]
    fn weighted_sample_empty_returns_empty() {
        let mut rng = StdRng::seed_from_u64(0);
        let weights = VoronoiWeights {
            weights: None,
            width: 5,
            height: 5,
        };
        let picked = weighted_sample(&[], weights, 5, &mut rng);
        assert!(picked.is_empty());
    }
}
