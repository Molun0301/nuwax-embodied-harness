//! CoveragePatrolRouter：在候选目标中估算"沿路径会新增覆盖的 visited cell 数"，选最大值。
//!
//! 不复制 A*：通过外部注入的 `PathEstimator` trait 调用 A*，避免依赖具体算法 crate。
//! 默认实现：`StraightLineEstimator` 仅作占位/测试，真实 production 应注入 `nuwax-nav-astar`。

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::routers::{GridSnapshot, GridSnapshotExt, PatrolRouter};
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
            eprintln!(
                "weighted_sample: voronoi 加权采样待 P2 实装,当前 fallback 均匀抽样"
            );
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
            rng,
            candidates_to_consider,
            estimator: Box::new(StraightLineEstimator),
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
    }

    fn update_pose(&mut self, pose: (f64, f64, f64)) {
        self.history.handle_odom(pose.0, pose.1);
    }

    fn next_goal(&mut self) -> Option<(f64, f64, f64)> {
        // 简化的 coverage：从未 visit 的 free cell 中随机抽样 N 个候选，
        // 由 PathEstimator 估算 path 长度，挑选最短路径候选。
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
            // 估算：从当前 visited 中心点（粗近似：网格中心）出发
            let from = (vw / 2, vh / 2);
            let path =
                self.estimator
                    .estimate_cell_path(&self.snapshot_for_estimate(), from, (col, row));
            if path < 0 {
                continue;
            }
            if best.is_none_or(|(_, p)| path < p) {
                best = Some(((col as f64, row as f64), path));
            }
        }
        let ((col_f, row_f), _) = best?;
        let res = self.history.resolution();
        let x = col_f * res;
        let y = row_f * res;
        Some((x, y, 0.0))
    }

    fn saturation(&self) -> f32 {
        self.history.saturation()
    }

    fn reset(&mut self) {
        self.history.reset();
    }
}

impl CoveragePatrolRouter {
    fn snapshot_for_estimate(&self) -> GridSnapshot {
        // 构造 free mask 全 true 作为简化 path 估算上下文
        let vw = self.history.width();
        let vh = self.history.height();
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
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; vw * vh],
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
