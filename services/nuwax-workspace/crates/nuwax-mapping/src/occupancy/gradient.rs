//! 栅格梯度
//! 计算 OccupancyGrid 的 2D 梯度（中央差分）+ Chamfer-8 距离变换 costmap。

use crate::occupancy::types::{OccupancyGrid, OccupancyValue};
use ndarray::Array2;

/// 栅格梯度（2D）
pub struct Gradient;

impl Gradient {
    /// 计算 Sobel 风格的梯度（简化版：中央差分）
    ///
    /// Returns `(grad_x, grad_y)`，每个都是 `Array2<f64>`，与输入同尺寸。
    #[must_use]
    pub fn sobel(grid: &OccupancyGrid) -> (Array2<f64>, Array2<f64>) {
        let h = grid.height as usize;
        let w = grid.width as usize;
        let mut gx = Array2::from_elem((h, w), 0.0_f64);
        let mut gy = Array2::from_elem((h, w), 0.0_f64);

        for r in 1..(grid.height - 1) {
            for c in 1..(grid.width - 1) {
                let v = |dr: i32, dc: i32| -> f64 {
                    grid.get((r as i32 + dr) as u32, (c as i32 + dc) as u32)
                        .map(|ov| ov as i8 as f64)
                        .unwrap_or(0.0)
                };
                // Sobel X
                gx[(r as usize, c as usize)] =
                    (v(-1, -1) + 2.0 * v(0, -1) + v(1, -1)) - (v(-1, 1) + 2.0 * v(0, 1) + v(1, 1));
                // Sobel Y
                gy[(r as usize, c as usize)] =
                    (v(-1, -1) + 2.0 * v(-1, 0) + v(-1, 1)) - (v(1, -1) + 2.0 * v(1, 0) + v(1, 1));
            }
        }

        (gx, gy)
    }

    /// 梯度幅值
    #[must_use]
    pub fn magnitude(gx: &Array2<f64>, gy: &Array2<f64>) -> Array2<f64> {
        let mut mag = Array2::from_elem(gx.raw_dim(), 0.0_f64);
        for ((r, c), gxv) in gx.indexed_iter() {
            let gyv = gy[(r, c)];
            mag[(r, c)] = (gxv * gxv + gyv * gyv).sqrt();
        }
        mag
    }
}

/// 距离变换 costmap 配置（rework22 P1 #2）
///
/// 对应 dimos Python `make_costmap` 输入参数：
/// - `resolution` — 单元分辨率（m/cell），与 OccupancyGrid.resolution 一致
/// - `robot_increase` — 机器人宽度乘数（cost 梯度从这往外扩展）
/// - `max_cost` — cost 截断上限（默认 90，对齐 nuwax-nav-astar 中 90 cap）
#[derive(Debug, Clone, Copy)]
pub struct CostMapConfig {
    /// 单元分辨率（m/cell）
    pub resolution: f64,
    /// 机器人宽度乘数（膨胀半径 = robot_increase * robot_width）
    pub robot_increase: f64,
    /// cost 上限（默认 90）
    pub max_cost: u8,
}

impl Default for CostMapConfig {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            robot_increase: 1.0,
            max_cost: 90,
        }
    }
}

/// Costmap 输出（u8 单元，与 nuwax-nav-astar `CostMap2D` 兼容）
///
/// 0 = FREE / 远离障碍
/// 255 = OCCUPIED_OR_UNKNOWN（不参与膨胀，但标记为不可走）
/// 1..max_cost = cost 梯度（越接近障碍越高）
#[derive(Debug, Clone)]
pub struct CostMapOutput {
    /// 单元值（u8）
    pub cells: Vec<u8>,
    /// 宽
    pub width: usize,
    /// 高
    pub height: usize,
    /// 分辨率（m/cell）
    pub resolution: f64,
}

impl CostMapOutput {
    /// 取某格
    #[must_use]
    pub fn get(&self, row: usize, col: usize) -> u8 {
        if row >= self.height || col >= self.width {
            return u8::MAX;
        }
        self.cells[row * self.width + col]
    }
}

/// 制造 costmap（Chamfer-8 距离变换 + cost 梯度）
///
/// 流程：
/// 1. 标记 Occupied → OCCUPIED；Free → FREE；**Unknown → OCCUPIED_OR_UNKNOWN**
///    （**不参与障碍膨胀**，仅做"不可走"标记）
/// 2. 按 `robot_increase * resolution` 半径对 Occupied 做 BFS-8 膨胀
///    （Unknown 不膨胀，避免无观测区域被误膨胀）
/// 3. Chamfer-8 两遍距离变换（forward + backward 8-邻接）→ 距离图
/// 4. cost = `max_cost - dist * 9`，clamp 到 [0, max_cost]
/// 5. OCCUPIED_OR_UNKNOWN 仍标记为 255（不参与 cost 梯度）
///
/// 返回的 `CostMapOutput` 与 `nuwax-nav-astar::CostMap2D` 协议兼容：
/// 0=FREE, 1..max_cost=梯度, 255=OCCUPIED/UNKNOWN。
#[must_use]
pub fn make_costmap(grid: &OccupancyGrid, cfg: &CostMapConfig) -> CostMapOutput {
    let w = grid.width as usize;
    let h = grid.height as usize;
    let max_cost = cfg.max_cost;
    let mut cells = vec![0u8; w * h];

    // 阶段 1：标记 Occupied/Unknown（注意 Unknown 也标记为不可走，但不参与膨胀）
    for r in 0..h {
        for c in 0..w {
            let v = grid.get(r as u32, c as u32).unwrap_or(OccupancyValue::Unknown);
            match v {
                OccupancyValue::Occupied => cells[r * w + c] = 255,
                OccupancyValue::Unknown => cells[r * w + c] = 255,
                OccupancyValue::Free => {}
            }
        }
    }

    // 阶段 2：对 Occupied 做膨胀（Unknown 不膨胀）
    let inflate_radius_cells =
        ((cfg.robot_increase.max(0.0) / cfg.resolution).ceil() as i32).max(0);
    if inflate_radius_cells > 0 {
        for r in 0..h {
            for c in 0..w {
                if cells[r * w + c] == 255 {
                    // 判断原始 grid 中这一格是不是真 Occupied（不是 Unknown）
                    let v = grid.get(r as u32, c as u32).unwrap_or(OccupancyValue::Unknown);
                    if !matches!(v, OccupancyValue::Occupied) {
                        continue;
                    }
                    let r0 = r as i32;
                    let c0 = c as i32;
                    for dr in -inflate_radius_cells..=inflate_radius_cells {
                        for dc in -inflate_radius_cells..=inflate_radius_cells {
                            let nr = r0 + dr;
                            let nc = c0 + dc;
                            if nr < 0 || nc < 0 || nr >= h as i32 || nc >= w as i32 {
                                continue;
                            }
                            if dr * dr + dc * dc > inflate_radius_cells * inflate_radius_cells {
                                continue;
                            }
                            let idx = nr as usize * w + nc as usize;
                            if cells[idx] != 255 {
                                cells[idx] = 255;
                            }
                        }
                    }
                }
            }
        }
    }

    // 阶段 3：Chamfer-8 距离变换
    const CHAMFER_DIAG: f64 = 1.4142135623730951; // √2
    let mut dist = vec![f64::INFINITY; w * h];
    for i in 0..w * h {
        if cells[i] == 255 {
            dist[i] = 0.0;
        }
    }
    // forward pass: top-left to bottom-right
    for r in 0..h {
        for c in 0..w {
            let idx = r * w + c;
            if dist[idx] == 0.0 {
                continue;
            }
            let mut best = f64::INFINITY;
            if c > 0 {
                best = best.min(dist[idx - 1] + 1.0);
            }
            if r > 0 {
                best = best.min(dist[idx - w] + 1.0);
            }
            if c > 0 && r > 0 {
                best = best.min(dist[idx - 1 - w] + CHAMFER_DIAG);
            }
            if c + 1 < w && r > 0 {
                best = best.min(dist[idx + 1 - w] + CHAMFER_DIAG);
            }
            dist[idx] = best;
        }
    }
    // backward pass: bottom-right to top-left
    for r in (0..h).rev() {
        for c in (0..w).rev() {
            let idx = r * w + c;
            if dist[idx] == 0.0 {
                continue;
            }
            let mut best = dist[idx];
            if c + 1 < w {
                best = best.min(dist[idx + 1] + 1.0);
            }
            if r + 1 < h {
                best = best.min(dist[idx + w] + 1.0);
            }
            if c + 1 < w && r + 1 < h {
                best = best.min(dist[idx + 1 + w] + CHAMFER_DIAG);
            }
            if c > 0 && r + 1 < h {
                best = best.min(dist[idx - 1 + w] + CHAMFER_DIAG);
            }
            dist[idx] = best;
        }
    }

    // 阶段 4：写 cost 梯度（非 255 cell）
    for i in 0..w * h {
        if cells[i] != 255 {
            let d = dist[i];
            let cost = if !d.is_finite() {
                0
            } else {
                let v = f64::from(max_cost) - d * 9.0;
                v.clamp(0.0, f64::from(max_cost)) as u8
            };
            cells[i] = cost;
        }
    }

    CostMapOutput {
        cells,
        width: w,
        height: h,
        resolution: cfg.resolution,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::types::OccupancyValue;

    #[test]
    fn sobel_constant_zero() {
        let g = OccupancyGrid::new(5, 5, 0.1);
        let (gx, gy) = Gradient::sobel(&g);
        // 全 Unknown(-1) → 中心差分为 0
        for r in 1..4 {
            for c in 1..4 {
                assert_eq!(gx[(r as usize, c as usize)], 0.0);
                assert_eq!(gy[(r as usize, c as usize)], 0.0);
            }
        }
    }

    #[test]
    fn sobel_detects_horizontal_edge() {
        let mut g = OccupancyGrid::new(5, 5, 0.1);
        // 在第 2 行设置一整行 Occupied
        for c in 0..5 {
            g.set(2, c, OccupancyValue::Occupied);
        }
        let (gx, gy) = Gradient::sobel(&g);
        // 行 2 上下方向有强梯度
        assert!(gy[(1, 2)].abs() > 0.0);
        assert!(gy[(3, 2)].abs() > 0.0);
        // 水平方向梯度小（因为整行都是 occupied）
        assert_eq!(gx[(2, 2)], 0.0);
    }

    #[test]
    fn magnitude_basic() {
        let gxv = ndarray::arr2(&[[3.0_f64, 0.0], [0.0, 4.0]]);
        let gyv = ndarray::arr2(&[[4.0_f64, 0.0], [0.0, 3.0]]);
        let m = Gradient::magnitude(&gxv, &gyv);
        assert!((m[(0, 0)] - 5.0).abs() < 1e-9);
        assert!((m[(1, 1)] - 5.0).abs() < 1e-9);
    }
}
