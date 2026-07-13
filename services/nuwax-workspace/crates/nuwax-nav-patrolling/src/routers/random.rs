//! RandomPatrolRouter：从 free / unvisited / safe cell 随机取点。

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::routers::{GridSnapshot, PatrolRouter, PatrollingContext};
use crate::visitation_history::VisitationHistory;

/// Random router
pub struct RandomPatrolRouter {
    history: VisitationHistory,
    rng: StdRng,
}

impl RandomPatrolRouter {
    /// 构造（`rng_seed` None 使用 OS entropy）
    #[must_use]
    pub fn new(clearance_radius_m: f64, rng_seed: Option<u64>) -> Self {
        let rng = match rng_seed {
            Some(s) => StdRng::seed_from_u64(s),
            None => StdRng::from_entropy(),
        };
        Self {
            history: VisitationHistory::new(clearance_radius_m),
            rng,
        }
    }

    /// 内部：检查 cell 周围 clearance 圆盘是否都在 free_cells 内。
    /// 修复 P0-14：候选必须满足 `cell == FREE && !visited && clearance_ok`。
    fn clearance_ok(&self, idx: usize, ctx: &PatrollingContext) -> bool {
        let vw = ctx.width as usize;
        let vh = ctx.height as usize;
        if vw == 0 || vh == 0 {
            return false;
        }
        let r = self.history.clearance_radius_cells();
        let row = idx / vw;
        let col = idx % vw;
        let r_min = row.saturating_sub(r);
        let r_max = (row + r + 1).min(vh);
        let c_min = col.saturating_sub(r);
        let c_max = (col + r + 1).min(vw);
        // 把 free_cells 转成 HashSet 一次性建表，避免 O(r^2) 线性扫描
        // （调用频次 = 每 tick 一次，开销可接受）
        for ri in r_min..r_max {
            for ci in c_min..c_max {
                let n = ri * vw + ci;
                if !ctx.free_cells.contains(&n) {
                    return false;
                }
            }
        }
        true
    }
}

impl PatrolRouter for RandomPatrolRouter {
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
        // 无 ctx 路径：保留旧兼容，回退到 history-only 抽取。
        // 修复 P0-14:从 history 内部读取真实 origin_x/y,不再硬编码 0.0。
        let vw = self.history.width();
        let vh = self.history.height();
        if vw == 0 || vh == 0 {
            return None;
        }
        let res = self.history.resolution();
        let ox = self.history.origin_x();
        let oy = self.history.origin_y();
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
        let idx = candidates[self.rng.gen_range(0..candidates.len())];
        let row = idx / vw;
        let col = idx % vw;
        let x = ox + (col as f64 + 0.5) * res;
        let y = oy + (row as f64 + 0.5) * res;
        Some((x, y, 0.0))
    }

    fn next_goal_with(&mut self, ctx: &PatrollingContext) -> Option<(f64, f64, f64)> {
        // 修复 P0-14:用真实 grid_origin + odom + free_cells + clearance。
        let vw = ctx.width as usize;
        let vh = ctx.height as usize;
        if vw == 0 || vh == 0 || ctx.resolution <= 0.0 || ctx.free_cells.is_empty() {
            return None;
        }
        // 把 visited 转成 cell-visited 视图(通过 history.is_visited 反查)
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
            if !self.clearance_ok(idx, ctx) {
                continue;
            }
            candidates.push(idx);
        }
        if candidates.is_empty() {
            return None;
        }
        let idx = candidates[self.rng.gen_range(0..candidates.len())];
        let row = idx / vw;
        let col = idx % vw;
        // 真实世界坐标:grid_origin + (col + 0.5, row + 0.5) * resolution
        let x = ctx.grid_origin.x + (col as f64 + 0.5) * ctx.resolution;
        let y = ctx.grid_origin.y + (row as f64 + 0.5) * ctx.resolution;
        let _ = (self.history.width(), self.history.height()); // ensure init
        Some((x, y, 0.0))
    }

    fn saturation(&self) -> f32 {
        self.history.saturation()
    }

    fn reset(&mut self) {
        self.history.reset();
    }
}

#[allow(dead_code)]
fn _hasher_compat(s: &str) -> u64 {
    let mut h = DefaultHasher::new();
    s.hash(&mut h);
    h.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_with_same_seed() {
        let mut r1 = RandomPatrolRouter::new(0.5, Some(42));
        let mut r2 = RandomPatrolRouter::new(0.5, Some(42));
        let grid = GridSnapshot {
            width: 10,
            height: 10,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![0; 100],
        };
        r1.update_grid(grid.clone());
        r2.update_grid(grid);
        let g1 = r1.next_goal();
        let g2 = r2.next_goal();
        assert_eq!(g1, g2);
    }

    #[test]
    fn no_goal_when_no_grid() {
        let mut r = RandomPatrolRouter::new(0.5, Some(1));
        assert!(r.next_goal().is_none());
    }
}
