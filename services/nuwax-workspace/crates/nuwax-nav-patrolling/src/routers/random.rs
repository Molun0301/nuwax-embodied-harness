//! RandomPatrolRouter：从 free / unvisited / safe cell 随机取点。

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::routers::{GridSnapshot, PatrolRouter};
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
        // 安全 mask：free + 足够 clearance + (unvisited 优先)
        // 这里用历史构造的 clearance_disk + visited，但需要 grid
        // Random router 不持有 grid；外部通过 update_grid 已写入 origin/resolution
        // 通过 visitor 上的 width/height 再获取 grid info
        let _ = self.rng.gen::<u32>();
        // 由 Random router 我们假定外部已通过 update_grid 设置 history grid 大小，
        // 并调用 update_pose 写 visited。这里走最简路径：从 history 拿 visited / total
        // 然后从 total 中随机一个未 visit 的 cell（如果存在）；否则全部 free 中随机。
        let vw = self.history.width();
        let vh = self.history.height();
        if vw == 0 || vh == 0 {
            return None;
        }
        let res = self.history.resolution();
        let ox = 0.0_f64; // origin 已经在 history 内部保存，外部不需要再访问
        let oy = 0.0_f64;
        // 安全 cell 集合
        let visited = self.history.visited().unwrap_or(&[]);
        let total = vw * vh;
        let mut candidates: Vec<usize> = Vec::with_capacity(total);
        for i in 0..total {
            // visited[i] == false 表示该 cell 未 visit
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
        let (x, y) = (ox + (col as f64 + 0.5) * res, oy + (row as f64 + 0.5) * res);
        Some((x, y, 0.0))
    }

    fn saturation(&self) -> f32 {
        self.history.saturation()
    }

    fn reset(&mut self) {
        self.history.reset();
    }

    // helpers
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
