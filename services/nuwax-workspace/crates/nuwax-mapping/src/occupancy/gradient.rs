//! 栅格梯度
//! 计算 OccupancyGrid 的 2D 梯度（中央差分）。

use crate::occupancy::types::OccupancyGrid;
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
