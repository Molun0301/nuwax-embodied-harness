//! 栅格梯度
//! 计算 OccupancyGrid 的 2D 梯度（中央差分）和与 DimOS 一致的 EDT/Voronoi costmap。

use crate::occupancy::types::OccupancyGrid;
use ndarray::Array2;
use std::collections::VecDeque;

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

/// 距离变换 costmap 配置。
///
/// 对应 dimos Python `make_costmap` 输入参数：
/// - `resolution` — 单元分辨率（m/cell），与 OccupancyGrid.resolution 一致
/// - `robot_increase` — 障碍膨胀半径（米；历史字段名保留以兼容调用方）
/// - `max_cost` — 非障碍 cost 截断上限（原版为 99）
#[derive(Debug, Clone, Copy)]
pub struct CostMapConfig {
    /// 单元分辨率（m/cell）
    pub resolution: f64,
    /// 障碍膨胀半径（米）。
    pub robot_increase: f64,
    /// 非障碍 cost 上限。
    pub max_cost: u8,
}

impl Default for CostMapConfig {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            robot_increase: 1.0,
            max_cost: 99,
        }
    }
}

/// 与 Python `GradientStrategy` 对应的 cost 策略。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceCostStrategy {
    /// 到最近障碍的普通欧氏距离梯度。
    Gradient,
    /// 以不同障碍连通分量之间的 Voronoi 边作为零 cost 中线。
    Voronoi,
}

/// Costmap 输出（u8 单元，与 nuwax-nav-astar `CostMap2D` 兼容）
///
/// 0 = FREE / 远离障碍
/// 1..=99 = cost 梯度（越接近障碍越高）
/// 100 = OCCUPIED（含膨胀得到的障碍）
/// 255 = UNKNOWN
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

/// 使用原版 path-map 默认配置构造普通 EDT costmap。
///
/// 兼容入口保留历史签名；需要 Voronoi 时调用
/// [`make_costmap_with_strategy`]。
#[must_use]
pub fn make_costmap(grid: &OccupancyGrid, cfg: &CostMapConfig) -> CostMapOutput {
    make_costmap_with_strategy(grid, cfg, DistanceCostStrategy::Gradient, 1.5)
}

/// 构造与 DimOS `gradient` / `voronoi_gradient` 一致的 costmap。
///
/// - inflation 只以原始值 `>= 100` 的 cell 为源，使用米制圆形 kernel；
/// - inflation 后再把原始值 `50..99` 加入 gradient 障碍源；
/// - `-1` 是 UNKNOWN，不作为源，但落入 inflation 圆内时会被障碍覆盖；
/// - 距离使用精确平方欧氏距离变换，而不是 cell-space Chamfer 常量；
/// - 普通梯度和 Voronoi 都在 `max_distance_m` 外归零。
#[must_use]
pub fn make_costmap_with_strategy(
    grid: &OccupancyGrid,
    cfg: &CostMapConfig,
    strategy: DistanceCostStrategy,
    max_distance_m: f64,
) -> CostMapOutput {
    let w = grid.width as usize;
    let h = grid.height as usize;
    let resolution = if grid.resolution > 0.0 {
        grid.resolution
    } else {
        cfg.resolution.max(f64::EPSILON)
    };
    if w == 0 || h == 0 {
        return CostMapOutput {
            cells: Vec::new(),
            width: w,
            height: h,
            resolution,
        };
    }

    let unknown: Vec<bool> = grid.data.iter().map(|v| *v < 0).collect();
    let inflation_sources: Vec<bool> = grid.data.iter().map(|v| *v >= 100).collect();
    let gradient_sources: Vec<bool> = grid.data.iter().map(|v| *v >= 50).collect();

    let radius_cells = (cfg.robot_increase.max(0.0) / resolution).ceil();
    let radius2 = radius_cells * radius_cells;
    let inflated = if inflation_sources.iter().any(|source| *source) {
        let (source_dist2, _, _) = squared_edt_with_indices(&inflation_sources, w, h);
        source_dist2.iter().map(|d2| *d2 <= radius2).collect()
    } else {
        vec![false; w * h]
    };
    let obstacle_sources: Vec<bool> = gradient_sources
        .iter()
        .zip(&inflated)
        .map(|(gradient, inflated)| *gradient || *inflated)
        .collect();
    if !obstacle_sources.iter().any(|source| *source) {
        let cells = unknown
            .iter()
            .map(|is_unknown| if *is_unknown { 255 } else { 0 })
            .collect();
        return CostMapOutput {
            cells,
            width: w,
            height: h,
            resolution,
        };
    }

    let (obstacle_dist2, nearest_x, nearest_y) = squared_edt_with_indices(&obstacle_sources, w, h);

    let cells = match strategy {
        DistanceCostStrategy::Gradient => gradient_costs(
            &obstacle_sources,
            &unknown,
            &obstacle_dist2,
            resolution,
            max_distance_m,
            cfg.max_cost,
        ),
        DistanceCostStrategy::Voronoi => {
            let (labels, cluster_count) = connected_component_labels(&obstacle_sources, w, h);
            if cluster_count <= 1 {
                gradient_costs(
                    &obstacle_sources,
                    &unknown,
                    &obstacle_dist2,
                    resolution,
                    max_distance_m,
                    cfg.max_cost,
                )
            } else {
                voronoi_costs(
                    &obstacle_sources,
                    &unknown,
                    &obstacle_dist2,
                    &nearest_x,
                    &nearest_y,
                    &labels,
                    w,
                    h,
                    resolution,
                    max_distance_m,
                    cfg.max_cost,
                )
            }
        }
    };

    CostMapOutput {
        cells,
        width: w,
        height: h,
        resolution,
    }
}

fn gradient_costs(
    obstacles: &[bool],
    unknown: &[bool],
    obstacle_dist2: &[f64],
    resolution: f64,
    max_distance_m: f64,
    max_cost: u8,
) -> Vec<u8> {
    let max_distance_m = max_distance_m.max(f64::EPSILON);
    (0..obstacles.len())
        .map(|idx| {
            if obstacles[idx] {
                return 100;
            }
            if unknown[idx] {
                return 255;
            }
            let distance_m = obstacle_dist2[idx].sqrt() * resolution;
            let ratio = 1.0 - (distance_m / max_distance_m).clamp(0.0, 1.0);
            ((ratio * 100.0) as u8).min(max_cost.min(99))
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn voronoi_costs(
    obstacles: &[bool],
    unknown: &[bool],
    obstacle_dist2: &[f64],
    nearest_x: &[usize],
    nearest_y: &[usize],
    labels: &[u32],
    width: usize,
    height: usize,
    resolution: f64,
    max_distance_m: f64,
    max_cost: u8,
) -> Vec<u8> {
    let nearest_labels: Vec<u32> = (0..labels.len())
        .map(|idx| labels[nearest_y[idx] * width + nearest_x[idx]])
        .collect();
    let mut edges = vec![false; labels.len()];
    for row in 0..height {
        for col in 0..width {
            let idx = row * width + col;
            if obstacles[idx] {
                continue;
            }
            let own = nearest_labels[idx];
            'neighbors: for dr in -1_i32..=1 {
                for dc in -1_i32..=1 {
                    let nr = (row as i32 + dr).clamp(0, height as i32 - 1) as usize;
                    let nc = (col as i32 + dc).clamp(0, width as i32 - 1) as usize;
                    if nearest_labels[nr * width + nc] != own {
                        edges[idx] = true;
                        break 'neighbors;
                    }
                }
            }
        }
    }
    if !edges.iter().any(|v| *v) {
        return gradient_costs(
            obstacles,
            unknown,
            obstacle_dist2,
            resolution,
            max_distance_m,
            max_cost,
        );
    }

    let (voronoi_dist2, _, _) = squared_edt_with_indices(&edges, width, height);
    let max_distance_cells = max_distance_m.max(0.0) / resolution;
    (0..obstacles.len())
        .map(|idx| {
            if obstacles[idx] {
                return 100;
            }
            if unknown[idx] {
                return 255;
            }
            let obstacle_distance = obstacle_dist2[idx].sqrt();
            if obstacle_distance > max_distance_cells {
                return 0;
            }
            let voronoi_distance = voronoi_dist2[idx].sqrt();
            let total = obstacle_distance + voronoi_distance;
            let raw = if total > 0.0 {
                (99.0 * voronoi_distance / total) as u8
            } else {
                0
            };
            raw.min(max_cost.min(99))
        })
        .collect()
}

fn connected_component_labels(mask: &[bool], width: usize, height: usize) -> (Vec<u32>, u32) {
    let mut labels = vec![0_u32; mask.len()];
    let mut next_label = 0_u32;
    for start in 0..mask.len() {
        if !mask[start] || labels[start] != 0 {
            continue;
        }
        next_label += 1;
        labels[start] = next_label;
        let mut queue = VecDeque::from([start]);
        while let Some(idx) = queue.pop_front() {
            let row = idx / width;
            let col = idx % width;
            for (dr, dc) in [(-1_i32, 0_i32), (1, 0), (0, -1), (0, 1)] {
                let nr = row as i32 + dr;
                let nc = col as i32 + dc;
                if nr < 0 || nc < 0 || nr >= height as i32 || nc >= width as i32 {
                    continue;
                }
                let neighbor = nr as usize * width + nc as usize;
                if mask[neighbor] && labels[neighbor] == 0 {
                    labels[neighbor] = next_label;
                    queue.push_back(neighbor);
                }
            }
        }
    }
    (labels, next_label)
}

fn squared_edt_with_indices(
    seeds: &[bool],
    width: usize,
    height: usize,
) -> (Vec<f64>, Vec<usize>, Vec<usize>) {
    let mut vertical = vec![f64::INFINITY; width * height];
    let mut vertical_arg = vec![usize::MAX; width * height];
    for col in 0..width {
        let input: Vec<f64> = (0..height)
            .map(|row| {
                if seeds[row * width + col] {
                    0.0
                } else {
                    f64::INFINITY
                }
            })
            .collect();
        let (distance, arg) = edt_1d(&input);
        for row in 0..height {
            vertical[row * width + col] = distance[row];
            vertical_arg[row * width + col] = arg[row];
        }
    }

    let mut distance = vec![f64::INFINITY; width * height];
    let mut nearest_x = vec![usize::MAX; width * height];
    let mut nearest_y = vec![usize::MAX; width * height];
    for row in 0..height {
        let input = &vertical[row * width..(row + 1) * width];
        let (row_distance, arg_x) = edt_1d(input);
        for col in 0..width {
            let source_col = arg_x[col];
            distance[row * width + col] = row_distance[col];
            nearest_x[row * width + col] = source_col;
            nearest_y[row * width + col] = vertical_arg[row * width + source_col];
        }
    }
    (distance, nearest_x, nearest_y)
}

fn edt_1d(input: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let finite: Vec<usize> = input
        .iter()
        .enumerate()
        .filter_map(|(idx, value)| value.is_finite().then_some(idx))
        .collect();
    if finite.is_empty() {
        return (
            vec![f64::INFINITY; input.len()],
            vec![usize::MAX; input.len()],
        );
    }

    let mut sites = vec![0_usize; finite.len()];
    let mut boundaries = vec![0.0_f64; finite.len() + 1];
    let mut k = 0_usize;
    sites[0] = finite[0];
    boundaries[0] = f64::NEG_INFINITY;
    boundaries[1] = f64::INFINITY;
    for &q in finite.iter().skip(1) {
        let mut site = sites[k];
        let mut intersection = ((input[q] + (q * q) as f64) - (input[site] + (site * site) as f64))
            / (2.0 * (q as f64 - site as f64));
        while intersection <= boundaries[k] && k > 0 {
            k -= 1;
            site = sites[k];
            intersection = ((input[q] + (q * q) as f64) - (input[site] + (site * site) as f64))
                / (2.0 * (q as f64 - site as f64));
        }
        k += 1;
        sites[k] = q;
        boundaries[k] = intersection;
        boundaries[k + 1] = f64::INFINITY;
    }

    let mut distance = vec![0.0_f64; input.len()];
    let mut arg = vec![0_usize; input.len()];
    k = 0;
    for q in 0..input.len() {
        while boundaries[k + 1] < q as f64 {
            k += 1;
        }
        let site = sites[k];
        let delta = q as f64 - site as f64;
        distance[q] = delta * delta + input[site];
        arg[q] = site;
    }
    (distance, arg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::occupancy::types::OccupancyValue;

    fn all_free(width: u32, height: u32, resolution: f64) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(width, height, resolution);
        for row in 0..height {
            for col in 0..width {
                grid.set(row, col, OccupancyValue::Free);
            }
        }
        grid
    }

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

    #[test]
    fn edt_gradient_uses_meter_distance() {
        let mut grid = all_free(9, 9, 0.5);
        grid.set(4, 4, OccupancyValue::Occupied);
        let output = make_costmap_with_strategy(
            &grid,
            &CostMapConfig {
                resolution: 0.5,
                robot_increase: 0.0,
                max_cost: 99,
            },
            DistanceCostStrategy::Gradient,
            1.5,
        );

        assert_eq!(output.get(4, 4), 100);
        assert_eq!(output.get(4, 5), 66);
        assert_eq!(output.get(4, 6), 33);
        assert_eq!(output.get(4, 7), 0);
    }

    #[test]
    fn circular_inflation_can_cover_unknown_without_using_it_as_a_source() {
        let mut grid = all_free(9, 9, 0.1);
        grid.set(4, 4, OccupancyValue::Occupied);
        grid.set(4, 5, OccupancyValue::Unknown);
        grid.set(1, 1, OccupancyValue::Unknown);
        let output = make_costmap(
            &grid,
            &CostMapConfig {
                resolution: 0.1,
                robot_increase: 0.1,
                max_cost: 99,
            },
        );

        assert_eq!(output.get(4, 5), 100, "inflation must cover nearby unknown");
        assert_eq!(output.get(1, 1), 255, "remote unknown stays unknown");
    }

    #[test]
    fn inflation_uses_only_occupied_cells_before_gradient_thresholding() {
        let mut grid = all_free(9, 9, 1.0);
        let occupied = grid.index(4, 4);
        let intermediate_cost = grid.index(1, 1);
        grid.data[occupied] = 100;
        grid.data[intermediate_cost] = 50;

        let output = make_costmap(
            &grid,
            &CostMapConfig {
                resolution: 1.0,
                robot_increase: 1.0,
                max_cost: 99,
            },
        );

        assert_eq!(output.get(4, 5), 100, "occupied cell must inflate");
        assert_eq!(output.get(1, 1), 100, "50 is a gradient obstacle source");
        assert!(
            output.get(1, 2) < 100,
            "50..99 cells become gradient sources only after inflation"
        );
    }

    #[test]
    fn original_python_gradient_and_voronoi_fixture_match_every_cell() {
        let mut grid = all_free(9, 7, 0.5);
        for (row, col, value) in [(0, 4, -1), (1, 4, 50), (3, 1, 100), (3, 2, -1), (3, 7, 100)] {
            let index = grid.index(row, col);
            grid.data[index] = value;
        }
        let config = CostMapConfig {
            resolution: 0.5,
            robot_increase: 0.5,
            max_cost: 99,
        };

        let gradient =
            make_costmap_with_strategy(&grid, &config, DistanceCostStrategy::Gradient, 1.5);
        let expected_gradient = vec![
            25, 33, 25, 52, 255, 52, 25, 33, 25, 52, 66, 52, 66, 100, 66, 52, 66, 52, 66, 100, 66,
            52, 66, 52, 66, 100, 66, 100, 100, 100, 66, 33, 66, 100, 100, 100, 66, 100, 66, 52, 25,
            52, 66, 100, 66, 52, 66, 52, 25, 5, 25, 52, 66, 52, 25, 33, 25, 5, 0, 5, 25, 33, 25,
        ];
        assert_eq!(gradient.cells, expected_gradient);

        let voronoi =
            make_costmap_with_strategy(&grid, &config, DistanceCostStrategy::Voronoi, 1.5);
        let expected_voronoi = vec![
            46, 33, 0, 0, 255, 0, 0, 0, 30, 57, 49, 0, 0, 100, 0, 0, 0, 41, 66, 100, 0, 0, 0, 0, 0,
            100, 57, 100, 100, 100, 0, 0, 0, 100, 100, 100, 73, 100, 57, 41, 0, 0, 49, 100, 73, 71,
            73, 57, 30, 0, 0, 41, 66, 67, 63, 59, 46, 25, 0, 0, 30, 49, 56,
        ];
        assert_eq!(voronoi.cells, expected_voronoi);
    }

    #[test]
    fn voronoi_strategy_places_zero_cost_between_obstacle_clusters() {
        let mut grid = all_free(9, 7, 0.5);
        grid.set(3, 1, OccupancyValue::Occupied);
        grid.set(3, 7, OccupancyValue::Occupied);
        let output = make_costmap_with_strategy(
            &grid,
            &CostMapConfig {
                resolution: 0.5,
                robot_increase: 0.0,
                max_cost: 99,
            },
            DistanceCostStrategy::Voronoi,
            1.5,
        );

        assert_eq!(output.get(3, 4), 0);
        assert!(output.get(3, 3) < output.get(3, 2));
        assert_eq!(output.get(3, 1), 100);
        assert_eq!(output.get(3, 7), 100);
    }
}
