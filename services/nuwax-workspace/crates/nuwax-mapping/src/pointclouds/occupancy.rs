//! 点云 → 占据栅格
//!
//! The three kernels mirror
//! `/home/ml/Deeprob/dimos/dimos/mapping/pointclouds/occupancy.py`:
//! terrain-gradient `HeightCost`, dilated-ground `General`, and rounded-cell
//! `Simple`. All public entry points dispatch through the same kernels.

use std::borrow::Cow;

use crate::cost_mapper::PointCloud2Like;
use crate::models::{Costmap, MapMeta};
use nalgebra::geometry::Translation;
use nalgebra::{Isometry3, Point3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

// =====================================================================
// 三套算法枚举 + 配置（rework22 P1 #3）
// =====================================================================

/// Point-cloud occupancy algorithm from the original DimOS registry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PointCloudOccAlgo {
    /// Terrain cost derived from the Sobel magnitude of a smoothed height map.
    HeightCost,
    /// Ground/obstacle projection followed by circular free-space dilation.
    General,
    /// Two-pass rounded-cell ground/obstacle projection.
    Simple,
}

impl Default for PointCloudOccAlgo {
    fn default() -> Self {
        // 默认对齐 dimos `pointcloud_occupancy.py:height_cost`
        Self::HeightCost
    }
}

/// Algorithm parameters from the original occupancy config dataclasses.
/// Resolution and map metadata remain properties of the builder/output grid.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PointCloudOccConfig {
    /// Selected kernel.
    pub algo: PointCloudOccAlgo,
    /// Ground points are strictly below this height (`General`/`Simple`).
    #[serde(rename = "min_height", alias = "min_height_m")]
    pub min_height_m: f64,
    /// Obstacle points are at or below this height (`General`/`Simple`).
    #[serde(rename = "max_height", alias = "max_height_m")]
    pub max_height_m: f64,
    /// A vertical gap above this value selects the lower surface (`HeightCost`).
    #[serde(rename = "can_pass_under", alias = "can_pass_under_m")]
    pub can_pass_under_m: f64,
    /// Height change per cell that maps to cost 100 (`HeightCost`).
    #[serde(rename = "can_climb", alias = "can_climb_m")]
    pub can_climb_m: f64,
    /// Height changes below this value are treated as noise (`HeightCost`).
    #[serde(rename = "ignore_noise", alias = "ignore_noise_m")]
    pub ignore_noise_m: f64,
    /// Gaussian sigma in cells; zero disables interpolation (`HeightCost`).
    #[serde(rename = "smoothing", alias = "smoothing_sigma")]
    pub smoothing_sigma: f64,
    /// Circular free-space dilation radius (`General`).
    #[serde(rename = "mark_free_radius", alias = "mark_free_radius_m")]
    pub mark_free_radius_m: f64,
    /// Retained from `SimpleOccupancyConfig`; the original kernel does not use it.
    pub closing_iterations: u32,
    /// Retained from `SimpleOccupancyConfig`; the original kernel does not use it.
    pub closing_connectivity: u8,
}

impl Default for PointCloudOccConfig {
    fn default() -> Self {
        Self {
            algo: PointCloudOccAlgo::HeightCost,
            min_height_m: 0.1,
            max_height_m: 2.0,
            can_pass_under_m: 0.6,
            can_climb_m: 0.15,
            ignore_noise_m: 0.05,
            smoothing_sigma: 1.0,
            mark_free_radius_m: 0.4,
            closing_iterations: 1,
            closing_connectivity: 2,
        }
    }
}

impl PointCloudOccConfig {
    fn validate(&self, resolution: f64) -> Result<(), MappingError> {
        if !resolution.is_finite() || resolution <= 0.0 {
            return Err(MappingError::InvalidConfig(
                "resolution must be finite and positive",
            ));
        }
        let finite = [
            self.min_height_m,
            self.max_height_m,
            self.can_pass_under_m,
            self.can_climb_m,
            self.ignore_noise_m,
            self.smoothing_sigma,
            self.mark_free_radius_m,
        ]
        .into_iter()
        .all(f64::is_finite);
        if !finite {
            return Err(MappingError::InvalidConfig(
                "occupancy parameters must be finite",
            ));
        }
        if self.max_height_m < self.min_height_m {
            return Err(MappingError::InvalidConfig(
                "max_height_m must be at least min_height_m",
            ));
        }
        if self.can_pass_under_m < 0.0
            || self.can_climb_m <= 0.0
            || self.ignore_noise_m < 0.0
            || self.smoothing_sigma < 0.0
        {
            return Err(MappingError::InvalidConfig(
                "height-cost parameters are outside their valid ranges",
            ));
        }
        Ok(())
    }
}

/// Pointcloud occupancy mapping 错误类型
#[derive(Debug, Error)]
pub enum MappingError {
    /// 配置参数无效
    #[error("invalid config: {0}")]
    InvalidConfig(&'static str),
}

// =====================================================================
// CostMap2D：与 nuwax-nav-astar `CostMap2D` 协议兼容的最小 u8 容器
// =====================================================================
//
// 不引 nuwax-nav-astar 依赖（避免 cleaves 进 mapping 接口）。
// 这里直接用 mapping 板块内的 `Costmap`，对外同时暴露
// `pointclouds::occupancy::CostMap2D` 别名，方便上层按 spec 要求选择。

/// CostMap2D（u8 单元，与 nuwax-nav-astar `CostMap2D` 协议兼容）
pub type CostMap2D = Costmap;

/// CostMap2D 构造辅助（测试 / 上层用）
#[must_use]
pub fn make_costmap2d(width: u32, height: u32, resolution: f64) -> CostMap2D {
    let meta = MapMeta::new(
        resolution,
        width,
        height,
        Isometry3::from_parts(Translation::from([0.0, 0.0, 0.0]), nalgebra::one()),
    );
    // data 默认全 0 (FREE)；函数内部 fill 255（UNKNOWN）以语义对齐 mapping
    let mut cm = Costmap::new(meta);
    let n = (width as usize) * (height as usize);
    cm.data.fill(255u8);
    debug_assert_eq!(cm.data.len(), n);
    cm
}

// =====================================================================
// `project_to_occupancy` — 三套算法统一入口
// =====================================================================

/// 把点云投影到 `CostMap2D`，按 `cfg.algo` 选 kernel
///
/// 入参：
/// - `cloud` — `PointCloud2Like`（`SimplePointCloud2` 或上层实现）
/// - `grid` — `CostMap2D`（按需 mutate；cell 坐标 < grid 尺寸才会写入）
/// - `cfg` — 算法 + 阈值配置
///
/// The caller owns the fixed grid metadata. Cell generation itself is exactly
/// the same implementation used by [`PointCloudOccupancyBuilder`].
pub fn project_to_occupancy<P: PointCloud2Like + ?Sized>(
    cloud: &P,
    grid: &mut CostMap2D,
    cfg: &PointCloudOccConfig,
) -> Result<(), MappingError> {
    if grid.metadata.width == 0 || grid.metadata.height == 0 {
        return Err(MappingError::InvalidConfig("empty costmap"));
    }
    if grid.data.len() != (grid.metadata.width as usize) * (grid.metadata.height as usize) {
        return Err(MappingError::InvalidConfig(
            "costmap data length does not match metadata",
        ));
    }
    cfg.validate(grid.metadata.resolution)?;

    let builder = PointCloudOccupancyBuilder::new(
        cloud.sensor_origin(),
        grid.metadata.resolution,
        grid.metadata.width,
        grid.metadata.height,
        (
            grid.metadata.origin.translation.x,
            grid.metadata.origin.translation.y,
        ),
    );
    let projected = builder.build_with_config(cloud.points(), cfg);
    for (target, source) in grid.data.iter_mut().zip(projected.data) {
        *target = if source < 0 { 255 } else { source as u8 };
    }

    Ok(())
}

/// 同一内核的高度代价派生缓存；最多保留一份剖面和结果，不作为地图真值。
#[derive(Clone, Debug, Default)]
pub struct HeightCostCache {
    entry: Option<std::sync::Arc<CachedHeightCost>>,
}

#[derive(Debug)]
struct CachedHeightCost {
    width: u32,
    height: u32,
    resolution: f64,
    origin_xy: (f64, f64),
    config: PointCloudOccConfig,
    height_bits: Vec<u32>,
    data: Vec<i8>,
}

/// Build an occupancy grid using the original DimOS automatic map bounds.
///
/// Finite XYZ points define the XY extent. The function adds 1 m padding on
/// every side, rounds each dimension up with `ceil`, and propagates the cloud
/// frame and timestamp into the returned occupancy grid. When no finite point
/// remains, it returns the original-compatible 1x1 unknown grid.
///
/// # Errors
/// Returns [`MappingError`] for invalid configuration or unrepresentable grid
/// dimensions.
pub fn build_occupancy_auto<P: PointCloud2Like + ?Sized>(
    cloud: &P,
    resolution: f64,
    cfg: &PointCloudOccConfig,
) -> Result<crate::occupancy::types::OccupancyGrid, MappingError> {
    build_occupancy_auto_impl(cloud, resolution, cfg, None, false)
}

/// 高度剖面、边界和配置逐项相同时复用平滑/梯度结果；帧与时间仍来自当前输入。
///
/// # Errors
/// 配置和边界校验与无缓存入口相同。
pub fn build_occupancy_auto_cached<P: PointCloud2Like + ?Sized>(
    cloud: &P,
    resolution: f64,
    cfg: &PointCloudOccConfig,
    cache: &mut HeightCostCache,
    parallel_projection: bool,
) -> Result<crate::occupancy::types::OccupancyGrid, MappingError> {
    build_occupancy_auto_impl(cloud, resolution, cfg, Some(cache), parallel_projection)
}

fn build_occupancy_auto_impl<P: PointCloud2Like + ?Sized>(
    cloud: &P,
    resolution: f64,
    cfg: &PointCloudOccConfig,
    cache: Option<&mut HeightCostCache>,
    parallel_projection: bool,
) -> Result<crate::occupancy::types::OccupancyGrid, MappingError> {
    cfg.validate(resolution)?;
    let source_points = cloud.points();
    let mut min_x = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    let mut has_finite_point = false;
    let mut all_points_finite = true;
    // 首次扫描同时收集自动边界；全有限输入直接借用源切片，异常输入才过滤复制。
    for &point in source_points {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            all_points_finite = false;
            continue;
        }
        has_finite_point = true;
        min_x = min_x.min(point.x);
        max_x = max_x.max(point.x);
        min_y = min_y.min(point.y);
        max_y = max_y.max(point.y);
    }

    if !has_finite_point {
        let mut grid = crate::occupancy::types::OccupancyGrid::new(1, 1, resolution);
        grid.frame_id = cloud.frame_id().to_string();
        grid.ts = cloud.timestamp();
        return Ok(grid);
    }

    let points: Cow<'_, [Point3<f64>]> = if all_points_finite {
        Cow::Borrowed(source_points)
    } else {
        Cow::Owned(
            source_points
                .iter()
                .copied()
                .filter(|point| point.x.is_finite() && point.y.is_finite() && point.z.is_finite())
                .collect(),
        )
    };

    // Keep the auto-bounds origin on the resolution lattice.  Without this
    // quantization, millimetre-level edge noise changes the cell phase and
    // reprojects the entire map on every frame.
    let (origin_x, width) = auto_grid_axis_bounds(min_x - 1.0, max_x + 1.0, resolution)?;
    let (origin_y, height) = auto_grid_axis_bounds(min_y - 1.0, max_y + 1.0, resolution)?;
    if u64::from(width) * u64::from(height) > u64::from(u32::MAX) {
        return Err(MappingError::InvalidConfig(
            "auto-bounds grid exceeds occupancy storage capacity",
        ));
    }

    let builder = PointCloudOccupancyBuilder::new(
        cloud.sensor_origin(),
        resolution,
        width,
        height,
        (origin_x, origin_y),
    );
    let mut grid = match cache {
        Some(cache) if cfg.algo == PointCloudOccAlgo::HeightCost => {
            builder.build_height_cost_cached(&points, cfg, Some(cache), parallel_projection)
        }
        Some(cache) => {
            cache.entry = None;
            builder.build_with_config(&points, cfg)
        }
        None => builder.build_with_config(&points, cfg),
    };
    grid.frame_id = cloud.frame_id().to_string();
    grid.ts = cloud.timestamp();
    Ok(grid)
}

fn auto_grid_axis_bounds(
    minimum: f64,
    maximum: f64,
    resolution: f64,
) -> Result<(f64, u32), MappingError> {
    if !minimum.is_finite() || !maximum.is_finite() || !resolution.is_finite() || resolution <= 0.0
    {
        return Err(MappingError::InvalidConfig(
            "auto-bounds dimension is not representable",
        ));
    }
    let min_cell = (minimum / resolution).floor();
    let max_cell = (maximum / resolution).ceil();
    let cells = max_cell - min_cell;
    if !min_cell.is_finite()
        || !max_cell.is_finite()
        || !cells.is_finite()
        || cells < 1.0
        || cells > f64::from(u32::MAX)
    {
        return Err(MappingError::InvalidConfig(
            "auto-bounds dimension is not representable",
        ));
    }
    Ok((min_cell * resolution, cells as u32))
}

fn reflect_index(mut index: i32, length: usize) -> usize {
    debug_assert!(length > 0);
    let length = length as i32;
    while index < 0 || index >= length {
        index = if index < 0 {
            -index - 1
        } else {
            2 * length - index - 1
        };
    }
    index as usize
}

fn correlate_axis(
    input: &[f32],
    width: usize,
    height: usize,
    axis: usize,
    weights: &[f64],
) -> Vec<f32> {
    let mut output = vec![0.0_f32; input.len()];
    if input.is_empty() || weights.is_empty() {
        return output;
    }
    let radius = (weights.len() / 2) as i32;
    let mut values = vec![0.0_f64; width];
    if axis == 0 {
        for row in 0..height {
            values.fill(0.0);
            for (index, weight) in weights.iter().enumerate() {
                let source = reflect_index(row as i32 + index as i32 - radius, height);
                let source_row = &input[source * width..(source + 1) * width];
                // 连续列独立计算，允许编译器向量化；每个像素仍按原权重顺序双精度累加。
                for (value, point) in values.iter_mut().zip(source_row) {
                    *value += f64::from(*point) * weight;
                }
            }
            for (target, value) in output[row * width..(row + 1) * width]
                .iter_mut()
                .zip(&values)
            {
                *target = *value as f32;
            }
        }
    } else {
        // 每行只扩展一次反射边界，把内层索引查找改为连续窗口读取。
        let sources: Vec<_> = (0..width + weights.len() - 1)
            .map(|position| reflect_index(position as i32 - radius, width))
            .collect();
        let mut padded = vec![0.0_f32; sources.len()];
        for row in 0..height {
            values.fill(0.0);
            let source_row = &input[row * width..(row + 1) * width];
            for (point, source) in padded.iter_mut().zip(&sources) {
                *point = source_row[*source];
            }
            for (index, weight) in weights.iter().enumerate() {
                for (value, point) in values.iter_mut().zip(&padded[index..index + width]) {
                    *value += f64::from(*point) * weight;
                }
            }
            for (target, value) in output[row * width..(row + 1) * width]
                .iter_mut()
                .zip(&values)
            {
                *target = *value as f32;
            }
        }
    }
    output
}

fn gaussian_filter_reflect(input: &[f32], width: usize, height: usize, sigma: f64) -> Vec<f32> {
    if sigma <= 1.0e-15 {
        return input.to_vec();
    }
    let radius = (4.0 * sigma + 0.5).floor() as i32;
    let mut weights: Vec<f64> = (-radius..=radius)
        .map(|offset| (-(f64::from(offset * offset)) / (2.0 * sigma * sigma)).exp())
        .collect();
    let sum: f64 = weights.iter().sum();
    for weight in &mut weights {
        *weight /= sum;
    }

    // scipy.ndimage.gaussian_filter processes axis 0 before axis 1 and
    // stores each intermediate back into the input dtype (float32 here).
    let vertical = correlate_axis(input, width, height, 0, &weights);
    correlate_axis(&vertical, width, height, 1, &weights)
}

fn sobel_axis(input: &[f32], width: usize, height: usize, axis: usize) -> Vec<f32> {
    let derivative = correlate_axis(input, width, height, axis, &[-1.0, 0.0, 1.0]);
    correlate_axis(&derivative, width, height, 1 - axis, &[1.0, 2.0, 1.0])
}

// =====================================================================
// `PointCloudOccupancyBuilder`
// =====================================================================

/// Fixed-metadata adapter around the original DimOS occupancy kernels.
pub struct PointCloudOccupancyBuilder {
    /// 传感器 origin
    pub sensor_origin: Point3<f64>,
    /// 分辨率
    pub resolution: f64,
    /// 栅格宽
    pub width: u32,
    /// 栅格高
    pub height: u32,
    /// origin XY
    pub origin_xy: (f64, f64),
}

impl PointCloudOccupancyBuilder {
    /// 构造
    #[must_use]
    pub fn new(
        sensor_origin: Point3<f64>,
        resolution: f64,
        width: u32,
        height: u32,
        origin_xy: (f64, f64),
    ) -> Self {
        Self {
            sensor_origin,
            resolution,
            width,
            height,
            origin_xy,
        }
    }

    /// Build an occupancy grid with the original default `HeightCost` kernel.
    #[must_use]
    pub fn build(&self, points: &[Point3<f64>]) -> crate::occupancy::types::OccupancyGrid {
        self.build_with_config(points, &PointCloudOccConfig::default())
    }

    /// 使用显式算法配置构建固定边界的占据栅格。
    #[must_use]
    pub fn build_with_config(
        &self,
        points: &[Point3<f64>],
        cfg: &PointCloudOccConfig,
    ) -> crate::occupancy::types::OccupancyGrid {
        match cfg.algo {
            PointCloudOccAlgo::HeightCost => self.build_height_cost(points, cfg),
            PointCloudOccAlgo::General => self.build_general(points, cfg),
            PointCloudOccAlgo::Simple => self.build_simple(points, cfg),
        }
    }

    fn empty_grid(&self) -> crate::occupancy::types::OccupancyGrid {
        let mut grid =
            crate::occupancy::types::OccupancyGrid::new(self.width, self.height, self.resolution);
        grid.origin = [self.origin_xy.0, self.origin_xy.1, 0.0];
        grid
    }

    fn build_general(
        &self,
        points: &[Point3<f64>],
        cfg: &PointCloudOccConfig,
    ) -> crate::occupancy::types::OccupancyGrid {
        use crate::occupancy::types::OccupancyValue;
        let mut grid = self.empty_grid();

        for &point in points {
            if point.z < cfg.min_height_m {
                if let Some((row, col)) = self.world_to_cell(point) {
                    grid.set(row, col, OccupancyValue::Free);
                }
            }
        }
        for &point in points {
            if point.z >= cfg.min_height_m && point.z <= cfg.max_height_m {
                if let Some((row, col)) = self.world_to_cell(point) {
                    grid.set(row, col, OccupancyValue::Occupied);
                }
            }
        }

        if cfg.mark_free_radius_m <= 0.0 {
            return grid;
        }
        let free_sources: Vec<(u32, u32)> = (0..self.height)
            .flat_map(|row| (0..self.width).map(move |col| (row, col)))
            .filter(|(row, col)| matches!(grid.get(*row, *col), Some(OccupancyValue::Free)))
            .collect();
        let radius = (cfg.mark_free_radius_m / self.resolution).ceil() as i32;
        for (row, col) in free_sources {
            for dr in -radius..=radius {
                for dc in -radius..=radius {
                    if dr * dr + dc * dc > radius * radius {
                        continue;
                    }
                    let nr = row as i32 + dr;
                    let nc = col as i32 + dc;
                    if nr < 0 || nc < 0 || nr >= self.height as i32 || nc >= self.width as i32 {
                        continue;
                    }
                    if !matches!(
                        grid.get(nr as u32, nc as u32),
                        Some(OccupancyValue::Occupied)
                    ) {
                        grid.set(nr as u32, nc as u32, OccupancyValue::Free);
                    }
                }
            }
        }
        grid
    }

    fn build_simple(
        &self,
        points: &[Point3<f64>],
        cfg: &PointCloudOccConfig,
    ) -> crate::occupancy::types::OccupancyGrid {
        use crate::occupancy::types::OccupancyValue;
        let mut grid = self.empty_grid();
        for &point in points {
            let value = if point.z < cfg.min_height_m {
                OccupancyValue::Free
            } else if point.z <= cfg.max_height_m {
                OccupancyValue::Occupied
            } else {
                continue;
            };
            let Some((row, col)) = self.world_to_cell_rounded(point) else {
                continue;
            };
            let index = (row * self.width + col) as usize;
            if value == OccupancyValue::Free {
                // 占据点优先，等价于原始地面扫描后再执行障碍扫描。
                if grid.data[index] != OccupancyValue::Occupied as i8 {
                    grid.data[index] = OccupancyValue::Free as i8;
                }
            } else {
                grid.data[index] = OccupancyValue::Occupied as i8;
            }
        }
        grid
    }

    fn build_height_cost(
        &self,
        points: &[Point3<f64>],
        cfg: &PointCloudOccConfig,
    ) -> crate::occupancy::types::OccupancyGrid {
        self.build_height_cost_cached(points, cfg, None, false)
    }

    fn build_height_cost_cached(
        &self,
        points: &[Point3<f64>],
        cfg: &PointCloudOccConfig,
        cache: Option<&mut HeightCostCache>,
        parallel_projection: bool,
    ) -> crate::occupancy::types::OccupancyGrid {
        let mut grid = self.empty_grid();
        let width = self.width as usize;
        let height = self.height as usize;
        let len = width * height;
        let parallel_projection = parallel_projection
            && points.len() >= 2_000_000
            && std::thread::available_parallelism().is_ok_and(|count| count.get() >= 4);
        let (mut min_height, max_height) = self.height_extrema(points, parallel_projection);

        let mut heights = vec![f32::NAN; len];
        let mut observed = vec![false; len];
        for idx in 0..len {
            if min_height[idx].is_nan() {
                continue;
            }
            heights[idx] = if max_height[idx] - min_height[idx] > cfg.can_pass_under_m as f32 {
                min_height[idx]
            } else {
                max_height[idx]
            };
            observed[idx] = true;
        }

        if !observed.iter().any(|value| *value) {
            return grid;
        }

        // 精确比较浮点位模式，包括缺测和正负零；不以散列碰撞或近似相等跳过计算。
        if let Some(entry) = cache.as_ref().and_then(|cache| cache.entry.as_ref()) {
            if entry.width == self.width
                && entry.height == self.height
                && entry.resolution == self.resolution
                && entry.origin_xy == self.origin_xy
                && entry.config == *cfg
                && entry.height_bits.len() == heights.len()
                && entry
                    .height_bits
                    .iter()
                    .zip(&heights)
                    .all(|(bits, value)| *bits == value.to_bits())
            {
                grid.data.copy_from_slice(&entry.data);
                return grid;
            }
        }
        let height_bits = cache
            .as_ref()
            .map(|_| heights.iter().map(|height| height.to_bits()).collect());

        if cfg.smoothing_sigma > 0.0 {
            // 高度和权重的输入都已不再需要 min/max；直接复用 min_height 和 heights，
            // 避免为两份整图输入再分配 height_map_filled/weights。
            for idx in 0..len {
                if observed[idx] {
                    min_height[idx] = 1.0;
                } else {
                    min_height[idx] = 0.0;
                    heights[idx] = 0.0;
                }
            }
            let smoothed_heights =
                gaussian_filter_reflect(&heights, width, height, cfg.smoothing_sigma);
            let smoothed_weights =
                gaussian_filter_reflect(&min_height, width, height, cfg.smoothing_sigma);
            for idx in 0..len {
                if !observed[idx] && smoothed_weights[idx] > 0.01 {
                    heights[idx] = smoothed_heights[idx] / smoothed_weights[idx];
                    observed[idx] = true;
                }
            }
        } else {
            // Sobel 输入必须把未观测 cell 视为 0，与原 height_for_gradient 相同。
            for idx in 0..len {
                if !observed[idx] {
                    heights[idx] = 0.0;
                }
            }
        }

        let sobel_x = sobel_axis(&heights, width, height, 1);
        let sobel_y = sobel_axis(&heights, width, height, 0);
        let resolution = self.resolution as f32;
        let gradient_scale = 8.0 * resolution;

        for row in 1..self.height.saturating_sub(1) {
            for col in 1..self.width.saturating_sub(1) {
                let idx = (row * self.width + col) as usize;
                // Sobel实际消费整个3×3邻域，斜角缺测也必须保留未知。
                // 仅检查十字邻域会把缺测填充值0当作真实高度，制造随世界高度变化的假台阶。
                let valid = (row - 1..=row + 1)
                    .all(|r| (col - 1..=col + 1).all(|c| observed[(r * self.width + c) as usize]));
                if !valid {
                    continue;
                }
                let gradient_x = sobel_x[idx] / gradient_scale;
                let gradient_y = sobel_y[idx] / gradient_scale;
                let height_change =
                    (gradient_x * gradient_x + gradient_y * gradient_y).sqrt() * resolution;
                let filtered = if height_change < cfg.ignore_noise_m as f32 {
                    0.0
                } else {
                    height_change
                };
                let cost = ((filtered / cfg.can_climb_m as f32) * 100.0).clamp(0.0, 100.0) as i8;
                grid.data[idx] = cost;
            }
        }
        if let Some(cache) = cache {
            cache.entry = Some(std::sync::Arc::new(CachedHeightCost {
                width: self.width,
                height: self.height,
                resolution: self.resolution,
                origin_xy: self.origin_xy,
                config: *cfg,
                height_bits: height_bits.expect("缓存输入剖面已捕获"),
                data: grid.data.clone(),
            }));
        }
        grid
    }

    fn height_extrema_serial(&self, points: &[Point3<f64>]) -> (Vec<f32>, Vec<f32>) {
        let mut min_height = vec![f32::NAN; self.width as usize * self.height as usize];
        let mut max_height = vec![f32::NAN; self.width as usize * self.height as usize];
        for &point in points {
            let Some((row, col)) = self.world_to_cell_rounded(point) else {
                continue;
            };
            let idx = (row * self.width + col) as usize;
            let point_height = point.z as f32;
            if min_height[idx].is_nan() || point_height < min_height[idx] {
                min_height[idx] = point_height;
            }
            if max_height[idx].is_nan() || point_height > max_height[idx] {
                max_height[idx] = point_height;
            }
        }

        (min_height, max_height)
    }

    fn height_extrema(&self, points: &[Point3<f64>], parallel: bool) -> (Vec<f32>, Vec<f32>) {
        if !parallel {
            return self.height_extrema_serial(points);
        }
        // 只增加一个作用域工作线程，创建失败直接串行；退出此作用域前完成全部等待。
        std::thread::scope(|scope| {
            let (first, second) = points.split_at(points.len() / 2);
            let worker = std::thread::Builder::new()
                .name("pgo-height".into())
                .spawn_scoped(scope, || self.height_extrema_serial(first));
            let Ok(worker) = worker else {
                return self.height_extrema_serial(points);
            };
            let (second_min, second_max) = self.height_extrema_serial(second);
            let (mut min_height, mut max_height) = worker
                .join()
                .unwrap_or_else(|panic| std::panic::resume_unwind(panic));
            // 按原输入左右半段顺序归并，相等值保留先出现者，正负零也与串行相同。
            for index in 0..min_height.len() {
                if min_height[index].is_nan() || second_min[index] < min_height[index] {
                    min_height[index] = second_min[index];
                }
                if max_height[index].is_nan() || second_max[index] > max_height[index] {
                    max_height[index] = second_max[index];
                }
            }
            (min_height, max_height)
        })
    }

    /// World → cell (row, col)
    fn world_to_cell(&self, p: Point3<f64>) -> Option<(u32, u32)> {
        if self.width == 0 || self.height == 0 {
            return None;
        }
        let column = ((p.x - self.origin_xy.0) / self.resolution) as i32;
        let row = ((p.y - self.origin_xy.1) / self.resolution) as i32;
        Some((
            row.clamp(0, self.height as i32 - 1) as u32,
            column.clamp(0, self.width as i32 - 1) as u32,
        ))
    }

    fn world_to_cell_rounded(&self, p: Point3<f64>) -> Option<(u32, u32)> {
        let inverse_resolution = 1.0 / self.resolution;
        let c = ((p.x - self.origin_xy.0) * inverse_resolution + 0.5) as i32;
        let r = ((p.y - self.origin_xy.1) * inverse_resolution + 0.5) as i32;
        if r < 0 || c < 0 || r >= self.height as i32 || c >= self.width as i32 {
            return None;
        }
        Some((r as u32, c as u32))
    }

    /// ray cast 方向辅助
    #[must_use]
    pub fn ray_direction(&self, p: Point3<f64>) -> nalgebra::Vector3<f64> {
        p - self.sensor_origin
    }
}

// =====================================================================
// 测试
// =====================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cost_mapper::SimplePointCloud2;

    #[test]
    fn reflected_filter_matches_scalar_reference_bit_for_bit() {
        // 保留修改前的标量计算作为独立参照，覆盖窄图、大核、边缘与两条滤波轴。
        for (width, height) in [(1, 1), (2, 3), (17, 11), (127, 91)] {
            let input: Vec<f32> = (0..width * height)
                .map(|i| ((i as f64 * 1.713).sin() * 17.0) as f32)
                .collect();
            for weights in [
                vec![-1.0, 0.0, 1.0],
                vec![1.0, 2.0, 1.0],
                (-8..=8).map(|x| (-f64::from(x * x) / 8.0).exp()).collect(),
            ] {
                for axis in [0, 1] {
                    let actual = correlate_axis(&input, width, height, axis, &weights);
                    for row in 0..height {
                        for column in 0..width {
                            let mut expected = 0.0_f64;
                            for (index, weight) in weights.iter().enumerate() {
                                let offset = index as i32 - (weights.len() / 2) as i32;
                                let (r, c) = if axis == 0 {
                                    (reflect_index(row as i32 + offset, height), column)
                                } else {
                                    (row, reflect_index(column as i32 + offset, width))
                                };
                                expected += f64::from(input[r * width + c]) * weight;
                            }
                            assert_eq!(
                                actual[row * width + column].to_bits(),
                                (expected as f32).to_bits()
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn height_cache_reuses_only_exact_profiles_and_preserves_metadata() {
        let mut cache = HeightCostCache::default();
        let mut config = PointCloudOccConfig::default();
        let points: Vec<_> = (-8..=8)
            .flat_map(|x| {
                (-8..=8).map(move |y| {
                    Point3::new(
                        x as f64 * 0.05,
                        y as f64 * 0.05,
                        if x > 2 && y > 2 { 0.3 } else { -0.5 },
                    )
                })
            })
            .collect();
        let mut cloud =
            SimplePointCloud2::with_metadata(points.clone(), Point3::origin(), "first", 1.0);
        let first = build_occupancy_auto_cached(&cloud, 0.05, &config, &mut cache, false).unwrap();
        let entry = cache.entry.clone().unwrap();
        cloud.points.extend(points.clone());
        cloud.points.reverse();
        cloud.timestamp = 2.0;
        cloud.frame_id = "second".into();
        let repeated =
            build_occupancy_auto_cached(&cloud, 0.05, &config, &mut cache, false).unwrap();
        assert!(
            std::sync::Arc::ptr_eq(&entry, cache.entry.as_ref().unwrap()),
            "相同剖面未复用"
        );
        assert_eq!(first.data, repeated.data);
        assert_eq!(repeated.ts, 2.0);
        assert_eq!(repeated.frame_id, "second");
        for change in 0..6 {
            match change {
                0 => cloud.points.push(Point3::new(0.0, 0.0, 0.1)),
                1 => cloud.points.retain(|p| p.x != 0.0 || p.y != 0.0),
                2 => config.smoothing_sigma = 0.0,
                3 => cloud.points.push(Point3::new(-1.23, 2.34, -0.0)),
                4 => cloud.points.push(Point3::new(f64::NAN, 0.0, 0.0)),
                _ => config.algo = PointCloudOccAlgo::Simple,
            }
            let cached =
                build_occupancy_auto_cached(&cloud, 0.05, &config, &mut cache, false).unwrap();
            let reference = build_occupancy_auto(&cloud, 0.05, &config).unwrap();
            assert_eq!(
                cached.data, reference.data,
                "剖面/配置变化后出现旧代价：{change}"
            );
            assert_eq!(cached.origin, reference.origin);
            assert_eq!(
                (cached.width, cached.height),
                (reference.width, reference.height)
            );
            assert_eq!(cached.ts, reference.ts);
            assert_eq!(cached.frame_id, reference.frame_id);
        }
        assert!(cache.entry.is_none());
    }

    #[test]
    fn parallel_height_extrema_preserve_serial_bits_and_occupancy() {
        let points: Vec<_> = (0..70_000)
            .map(|index| {
                Point3::new(
                    -1.0 + (index % 43) as f64 * 0.05,
                    -1.0 + (index / 43 % 41) as f64 * 0.05,
                    match index % 7 {
                        0 => -0.0,
                        1 => 0.0,
                        _ => (index % 19) as f64 * 0.031,
                    },
                )
            })
            .collect();
        let builder = PointCloudOccupancyBuilder::new(Point3::origin(), 0.05, 67, 71, (-1.1, -1.1));
        let serial = builder.height_extrema(&points, false);
        let parallel = builder.height_extrema(&points, true);
        for (expected, actual) in serial
            .0
            .iter()
            .chain(&serial.1)
            .zip(parallel.0.iter().chain(&parallel.1))
        {
            assert_eq!(expected.to_bits(), actual.to_bits(), "并行改变了原点序极值");
        }
        let cloud = SimplePointCloud2::with_metadata(
            points.into_iter().cycle().take(2_000_000).collect(),
            Point3::origin(),
            "map",
            4.0,
        );
        let config = PointCloudOccConfig::default();
        let mut cache = HeightCostCache::default();
        let parallel =
            build_occupancy_auto_cached(&cloud, 0.05, &config, &mut cache, true).unwrap();
        let serial = build_occupancy_auto(&cloud, 0.05, &config).unwrap();
        assert_eq!(parallel.data, serial.data);
        assert_eq!(parallel.origin, serial.origin);
        assert_eq!(
            (parallel.width, parallel.height),
            (serial.width, serial.height)
        );
    }

    #[test]
    #[ignore = "不同窗口规模的作用域并行投影成本，使用release单独运行"]
    fn height_projection_parallel_scale_benchmark() {
        let builder =
            PointCloudOccupancyBuilder::new(Point3::origin(), 0.05, 201, 201, (-5.0, -5.0));
        for count in [100_000, 500_000, 1_000_000, 2_000_000] {
            let points: Vec<_> = (0..count)
                .map(|index| {
                    Point3::new(
                        -4.0 + (index % 161) as f64 * 0.05,
                        -4.0 + (index / 161 % 161) as f64 * 0.05,
                        (index % 19) as f64 * 0.031,
                    )
                })
                .collect();
            for parallel in [false, true] {
                let started = std::time::Instant::now();
                for _ in 0..30 {
                    std::hint::black_box(
                        builder.height_extrema(std::hint::black_box(&points), parallel),
                    );
                }
                eprintln!(
                    "投影成本 points={count} parallel={parallel} mean_ms={:.3}",
                    started.elapsed().as_secs_f64() * 1000.0 / 30.0
                );
            }
        }
    }

    fn small_grid() -> CostMap2D {
        make_costmap2d(11, 11, 0.1)
    }

    #[test]
    fn small_grid_constructor() {
        let g = small_grid();
        assert_eq!(g.metadata.width, 11);
        assert_eq!(g.metadata.height, 11);
        // make_costmap2d 默认全 255（UNKNOWN）
        assert_eq!(g.get(0, 0), Some(255));
    }

    #[test]
    fn height_cost_requires_full_gradient_support_and_is_height_translation_invariant() {
        let builder = PointCloudOccupancyBuilder::new(Point3::origin(), 0.05, 13, 13, (0.0, 0.0));
        let config = PointCloudOccConfig::default();
        let below = builder.build_with_config(&[Point3::new(0.3, 0.3, -0.45)], &config);
        let above = builder.build_with_config(&[Point3::new(0.3, 0.3, 0.55)], &config);
        assert_eq!(
            below.data, above.data,
            "只平移世界高度不能改变平地的通行代价"
        );
        assert_eq!(
            below.data[below.index(6, 6)],
            0,
            "完整邻域支撑的平面仍为自由"
        );
        assert_eq!(
            below.data[below.index(7, 7)],
            -1,
            "十字邻域有效但斜角无观测时，不能把默认高度0当成台阶"
        );

        let points: Vec<_> = (0..13)
            .flat_map(|row| {
                (0..13).map(move |col| {
                    Point3::new(
                        f64::from(col) * 0.05,
                        f64::from(row) * 0.05,
                        if col < 7 { -0.45 } else { 0.55 },
                    )
                })
            })
            .collect();
        let step = builder.build_with_config(&points, &config);
        assert_eq!(
            step.data[step.index(6, 6)],
            100,
            "完整观测的真实高度突变仍必须保留"
        );
    }

    #[test]
    fn auto_bounds_origin_is_aligned_to_resolution_lattice() {
        let resolution = 0.05;
        let cloud = SimplePointCloud2::with_metadata(
            vec![
                Point3::new(-5.027_830_739, -6.992_968_040, 0.0),
                Point3::new(6.401_233_101, 2.145_098_701, 0.5),
            ],
            Point3::origin(),
            "map",
            123.0,
        );

        let grid = build_occupancy_auto(
            &cloud,
            resolution,
            &PointCloudOccConfig {
                algo: PointCloudOccAlgo::General,
                ..PointCloudOccConfig::default()
            },
        )
        .expect("valid aligned auto bounds");

        for origin in [grid.origin[0], grid.origin[1]] {
            let lattice_index = origin / resolution;
            assert!(
                (lattice_index - lattice_index.round()).abs() < 1.0e-9,
                "origin={origin} resolution={resolution}"
            );
        }
    }

    #[test]
    fn subcell_edge_noise_does_not_shift_auto_bounds_lattice() {
        let resolution = 0.05;
        let config = PointCloudOccConfig {
            algo: PointCloudOccAlgo::General,
            ..PointCloudOccConfig::default()
        };
        let build = |minimum: f64, maximum: f64| {
            let cloud = SimplePointCloud2::with_metadata(
                vec![
                    Point3::new(minimum, minimum, 0.0),
                    Point3::new(maximum, maximum, 0.5),
                ],
                Point3::origin(),
                "map",
                1.0,
            );
            build_occupancy_auto(&cloud, resolution, &config).expect("valid auto bounds")
        };

        let first = build(-0.021, 0.421);
        let jittered = build(-0.022, 0.422);

        assert_eq!(first.origin, jittered.origin);
        assert_eq!(
            (first.width, first.height),
            (jittered.width, jittered.height)
        );
    }

    #[test]
    fn auto_bounds_borrow_and_filter_paths_preserve_grid_output() {
        let resolution = 0.1;
        let finite_points = vec![
            Point3::new(-0.4, -0.4, 0.0),
            Point3::new(0.0, 0.0, 0.2),
            Point3::new(0.4, 0.4, 0.8),
        ];
        let mut points_with_invalid = vec![finite_points[0]];
        points_with_invalid.push(Point3::new(f64::NAN, 0.0, 0.0));
        points_with_invalid.push(finite_points[1]);
        points_with_invalid.push(Point3::new(0.0, f64::INFINITY, 0.0));
        points_with_invalid.push(finite_points[2]);

        for algo in [
            PointCloudOccAlgo::General,
            PointCloudOccAlgo::Simple,
            PointCloudOccAlgo::HeightCost,
        ] {
            let config = PointCloudOccConfig {
                algo,
                smoothing_sigma: 0.0,
                ..PointCloudOccConfig::default()
            };
            let clean = SimplePointCloud2::with_metadata(
                finite_points.clone(),
                Point3::origin(),
                "map",
                7.0,
            );
            let filtered = SimplePointCloud2::with_metadata(
                points_with_invalid.clone(),
                Point3::origin(),
                "map",
                7.0,
            );
            let clean_grid =
                build_occupancy_auto(&clean, resolution, &config).expect("finite cloud must build");
            let filtered_grid = build_occupancy_auto(&filtered, resolution, &config)
                .expect("non-finite points must be filtered");
            assert_eq!(clean_grid.origin, filtered_grid.origin);
            assert_eq!(
                (clean_grid.width, clean_grid.height),
                (filtered_grid.width, filtered_grid.height)
            );
            assert_eq!(clean_grid.data, filtered_grid.data);
            assert_eq!(clean_grid.frame_id, filtered_grid.frame_id);
            assert_eq!(clean_grid.ts, filtered_grid.ts);
        }
    }
}
