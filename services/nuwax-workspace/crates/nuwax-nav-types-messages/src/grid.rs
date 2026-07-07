//! 占据栅格、代价地图与高程/地形图
//!
//! 约定:
//! - `world_to_cell` 使用 `floor((x - origin) / resolution)`,越界返回 `None`
//! - `cell_center(ix, iy)` 返回格子中心点(非左下角)
//! - `CostMap2D` 中不可通行成本统一 `f32::INFINITY`,NaN 按 `CostMapPolicy` 处理
//! - 默认 policy:`nan_treatment=InfiniteCost`、`unknown_treatment=Unknown`、`occupied_threshold=100.0`

use serde::{Deserialize, Serialize};

use crate::geometry::Point2;

/// 单元状态(基础枚举,等价 Python cost map 的离散状态)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OccupancyCell {
    /// 未知(unknown)
    #[default]
    Unknown,
    /// 空闲(free)
    Free,
    /// 占据(occupied)
    Occupied,
    /// 已被 inflation 抬升的 cost
    Inflated,
    /// 一般 cost(0..=100,与 C++ costmap 等价)
    Cost(u8),
}

/// 2D 占据栅格
///
/// 坐标约定:
/// - `origin` 为栅格左下角对应的世界坐标
/// - `resolution` 单位:米 / cell
/// - `cells` 按行优先(`cells[ix + iy * width]`)存储
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OccupancyGrid2D {
    /// 世界坐标原点点
    pub origin: Point2,
    /// 分辨率(米/cell)
    pub resolution: f64,
    /// 宽度(格子数)
    pub width: u32,
    /// 高度(格子数)
    pub height: u32,
    /// 单元格数据(按行优先,长度 = width * height)
    pub cells: Vec<OccupancyCell>,
}

impl OccupancyGrid2D {
    /// 构造新栅格;若 cells 长度与 width/height 不匹配,panic(由调用方保证)
    #[must_use]
    pub fn new(
        origin: Point2,
        resolution: f64,
        width: u32,
        height: u32,
        cells: Vec<OccupancyCell>,
    ) -> Self {
        assert_eq!(
            cells.len(),
            (width as usize) * (height as usize),
            "cells length mismatch with width*height"
        );
        Self {
            origin,
            resolution: if resolution > 0.0 { resolution } else { 1.0 },
            width,
            height,
            cells,
        }
    }

    /// 全部填充为给定状态
    #[must_use]
    pub fn filled(
        origin: Point2,
        resolution: f64,
        width: u32,
        height: u32,
        fill: OccupancyCell,
    ) -> Self {
        let cells = vec![fill; (width as usize) * (height as usize)];
        Self::new(origin, resolution, width, height, cells)
    }

    /// 索引(行优先)
    #[must_use]
    pub fn index(&self, ix: u32, iy: u32) -> usize {
        ix as usize + iy as usize * self.width as usize
    }

    /// 越界安全读
    #[must_use]
    pub fn get(&self, ix: u32, iy: u32) -> Option<OccupancyCell> {
        if ix >= self.width || iy >= self.height {
            return None;
        }
        self.cells.get(self.index(ix, iy)).copied()
    }

    /// 越界安全写
    pub fn set(&mut self, ix: u32, iy: u32, value: OccupancyCell) -> bool {
        if ix >= self.width || iy >= self.height {
            return false;
        }
        let i = self.index(ix, iy);
        self.cells[i] = value;
        true
    }

    /// 世界坐标 → 栅格坐标(向下取整)
    ///
    /// 越界返回 `None`,不 panic。
    #[must_use]
    pub fn world_to_cell(&self, point: Point2) -> Option<(u32, u32)> {
        if self.resolution <= 0.0 {
            return None;
        }
        let fx = (point.x - self.origin.x) / self.resolution;
        let fy = (point.y - self.origin.y) / self.resolution;
        let ix = fx.floor() as i64;
        let iy = fy.floor() as i64;
        if ix < 0 || iy < 0 || ix >= self.width as i64 || iy >= self.height as i64 {
            return None;
        }
        Some((ix as u32, iy as u32))
    }

    /// 栅格坐标 → 世界坐标(返回格子中心)
    #[must_use]
    pub fn cell_center(&self, ix: u32, iy: u32) -> Option<Point2> {
        if ix >= self.width || iy >= self.height {
            return None;
        }
        let _half = self.resolution * 0.5;
        Some(Point2 {
            x: self.origin.x + (ix as f64 + 0.5) * self.resolution,
            y: self.origin.y + (iy as f64 + 0.5) * self.resolution,
        })
    }

    /// 世界坐标 → 格子中心(便捷组合)
    #[must_use]
    pub fn snap_world_to_cell_center(&self, point: Point2) -> Option<Point2> {
        let (ix, iy) = self.world_to_cell(point)?;
        self.cell_center(ix, iy)
    }
}

/// 单元 cost 处理策略
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "value", rename_all = "snake_case")]
pub enum CostCellTreatment {
    /// 视为 unknown
    Unknown,
    /// 视为 free(0 cost)
    Free,
    /// 视为 occupied(infinite cost)
    Occupied,
    /// 直接是 infinite
    #[default]
    InfiniteCost,
    /// 视为给定具体 cost
    Cost(f32),
}

/// CostMap 解析策略
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CostMapPolicy {
    /// NaN cell 怎么处理
    pub nan_treatment: CostCellTreatment,
    /// 原始 occupancy = Unknown 的 cell 怎么处理
    pub unknown_treatment: CostCellTreatment,
    /// 视为不可通行的 cost 阈值(默认 100.0)
    pub occupied_threshold: f32,
}

impl Default for CostMapPolicy {
    fn default() -> Self {
        Self {
            nan_treatment: CostCellTreatment::InfiniteCost,
            unknown_treatment: CostCellTreatment::Unknown,
            occupied_threshold: 100.0,
        }
    }
}

impl CostMapPolicy {
    /// 按策略解析原始 f32 cost(可能为 NaN / 负数 / 越界)
    #[must_use]
    pub fn resolve(&self, raw: f32, occupancy: OccupancyCell) -> f32 {
        if raw.is_nan() {
            return self.treat(self.nan_treatment);
        }
        match occupancy {
            OccupancyCell::Unknown => self.treat(self.unknown_treatment),
            _ => {
                if raw >= self.occupied_threshold {
                    f32::INFINITY
                } else if raw < 0.0 {
                    // 负 cost 钳到 0
                    0.0
                } else {
                    raw
                }
            }
        }
    }

    fn treat(&self, t: CostCellTreatment) -> f32 {
        match t {
            CostCellTreatment::Unknown => 0.0,
            CostCellTreatment::Free => 0.0,
            CostCellTreatment::Occupied | CostCellTreatment::InfiniteCost => f32::INFINITY,
            CostCellTreatment::Cost(v) => v,
        }
    }
}

/// 2D 代价地图
///
/// 与 `OccupancyGrid2D` 同坐标约定;`costs` 长度 = width * height。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CostMap2D {
    /// 原点
    pub origin: Point2,
    /// 分辨率(米/cell)
    pub resolution: f64,
    /// 宽度(格子数)
    pub width: u32,
    /// 高度(格子数)
    pub height: u32,
    /// cost 数据(行优先)
    pub costs: Vec<f32>,
    /// cost 解析策略
    pub policy: CostMapPolicy,
}

impl CostMap2D {
    /// 由 `OccupancyGrid2D` + 原始 f32 cost 数组构造
    #[must_use]
    pub fn from_grid_with_costs(grid: &OccupancyGrid2D, costs: &[f32]) -> Self {
        let mut out = Vec::with_capacity(grid.cells.len());
        for (cell, raw) in grid.cells.iter().zip(costs.iter()) {
            out.push(grid_default_policy().resolve(*raw, *cell));
        }
        Self {
            origin: grid.origin,
            resolution: grid.resolution,
            width: grid.width,
            height: grid.height,
            costs: out,
            policy: grid_default_policy(),
        }
    }

    /// 索引(行优先)
    #[must_use]
    pub fn index(&self, ix: u32, iy: u32) -> usize {
        ix as usize + iy as usize * self.width as usize
    }

    /// 越界安全读 cost
    #[must_use]
    pub fn cost(&self, ix: u32, iy: u32) -> Option<f32> {
        if ix >= self.width || iy >= self.height {
            return None;
        }
        self.costs.get(self.index(ix, iy)).copied()
    }

    /// 应用给定 policy 重写所有 cost
    pub fn apply_policy(&mut self, grid: &OccupancyGrid2D, raw: &[f32], policy: CostMapPolicy) {
        debug_assert_eq!(raw.len(), self.costs.len());
        debug_assert_eq!(grid.cells.len(), self.costs.len());
        for (i, (cell, r)) in grid.cells.iter().zip(raw.iter()).enumerate() {
            self.costs[i] = policy.resolve(*r, *cell);
        }
        self.policy = policy;
    }
}

fn grid_default_policy() -> CostMapPolicy {
    CostMapPolicy::default()
}

/// 单格子高程采样
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HeightCell {
    /// 高程(米)
    pub elevation: f32,
    /// 估计方差
    pub variance: f32,
    /// 是否可信(没有有效观测时为 false)
    pub valid: bool,
}

impl Default for HeightCell {
    fn default() -> Self {
        Self {
            elevation: 0.0,
            variance: 0.0,
            valid: false,
        }
    }
}

/// 2D 高程图
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeightMap {
    /// 原点
    pub origin: Point2,
    /// 分辨率(米/cell)
    pub resolution: f64,
    /// 宽度
    pub width: u32,
    /// 高度
    pub height: u32,
    /// 高程数据(行优先,长度 = width * height)
    pub cells: Vec<HeightCell>,
}

impl HeightMap {
    /// 构造新图
    #[must_use]
    pub fn new(
        origin: Point2,
        resolution: f64,
        width: u32,
        height: u32,
        cells: Vec<HeightCell>,
    ) -> Self {
        assert_eq!(cells.len(), (width as usize) * (height as usize));
        Self {
            origin,
            resolution,
            width,
            height,
            cells,
        }
    }

    /// 全部 invalid 的高程图
    #[must_use]
    pub fn empty(origin: Point2, resolution: f64, width: u32, height: u32) -> Self {
        let cells = vec![HeightCell::default(); (width as usize) * (height as usize)];
        Self::new(origin, resolution, width, height, cells)
    }

    /// 是否任何 cell 有效
    #[must_use]
    pub fn has_any_valid(&self) -> bool {
        self.cells.iter().any(|c| c.valid)
    }
}

/// 扩展地形图(在 `HeightMap` 之上加 traversability)
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TerrainMap {
    /// 内核高程图
    pub height: HeightMap,
    /// traversability(0.0 完全不可通行,1.0 完全可通行)
    pub traversability: Vec<f32>,
    /// 可选:每个 cell 距最近障碍的米数(由上游 terrain analysis 提供)
    pub clearance_m: Option<Vec<f32>>,
}

impl TerrainMap {
    /// 构造新地形图(校验长度)
    pub fn new(height: HeightMap, traversability: Vec<f32>, clearance_m: Option<Vec<f32>>) -> Self {
        assert_eq!(traversability.len(), height.cells.len());
        if let Some(c) = &clearance_m {
            assert_eq!(c.len(), height.cells.len());
        }
        Self {
            height,
            traversability,
            clearance_m,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_grid() -> OccupancyGrid2D {
        let cells = vec![OccupancyCell::Free; 16];
        OccupancyGrid2D::new(Point2::new(-2.0, -2.0), 1.0, 4, 4, cells)
    }

    #[test]
    fn grid_world_to_cell_basic() {
        let g = sample_grid();
        assert_eq!(g.world_to_cell(Point2::new(-2.0, -2.0)), Some((0, 0)));
        assert_eq!(g.world_to_cell(Point2::new(-1.5, -1.5)), Some((0, 0)));
        assert_eq!(g.world_to_cell(Point2::new(0.0, 0.0)), Some((2, 2)));
        assert_eq!(g.world_to_cell(Point2::new(1.9, 1.9)), Some((3, 3)));
    }

    #[test]
    fn grid_world_to_cell_out_of_bounds() {
        let g = sample_grid();
        assert_eq!(g.world_to_cell(Point2::new(-3.0, 0.0)), None);
        assert_eq!(g.world_to_cell(Point2::new(10.0, 10.0)), None);
    }

    #[test]
    fn grid_cell_center_not_corner() {
        let g = sample_grid();
        let p = g.cell_center(0, 0).unwrap();
        assert!((p.x - (-1.5)).abs() < 1e-9);
        assert!((p.y - (-1.5)).abs() < 1e-9);
    }

    #[test]
    fn grid_set_and_get() {
        let mut g = sample_grid();
        assert!(g.set(1, 1, OccupancyCell::Occupied));
        assert_eq!(g.get(1, 1), Some(OccupancyCell::Occupied));
        assert_eq!(g.get(10, 0), None);
    }

    #[test]
    fn costmap_default_policy_resolves_nan() {
        let g = sample_grid();
        let raw = vec![f32::NAN; 16];
        let cm = CostMap2D::from_grid_with_costs(&g, &raw);
        for c in cm.costs {
            assert!(c.is_infinite());
        }
    }

    #[test]
    fn costmap_default_policy_resolves_unknown_to_unknown_cost() {
        let mut g = sample_grid();
        g.set(0, 0, OccupancyCell::Unknown);
        let raw = vec![0.0_f32; 16];
        let cm = CostMap2D::from_grid_with_costs(&g, &raw);
        // unknown cell under default policy -> unknown_treatment=Unknown -> 0
        assert_eq!(cm.cost(0, 0), Some(0.0));
    }

    #[test]
    fn costmap_over_threshold_is_infinite() {
        let g = sample_grid();
        let raw = vec![200.0_f32; 16];
        let cm = CostMap2D::from_grid_with_costs(&g, &raw);
        for c in cm.costs {
            assert!(c.is_infinite());
        }
    }

    #[test]
    fn costmap_policy_swap_changes_unknown_treatment() {
        let mut g = sample_grid();
        g.set(1, 1, OccupancyCell::Unknown);
        let raw = vec![0.0_f32; 16];
        let mut cm = CostMap2D::from_grid_with_costs(&g, &raw);
        let new_policy = CostMapPolicy {
            nan_treatment: CostCellTreatment::InfiniteCost,
            unknown_treatment: CostCellTreatment::InfiniteCost,
            occupied_threshold: 100.0,
        };
        cm.apply_policy(&g, &raw, new_policy);
        assert!(cm.cost(1, 1).unwrap().is_infinite());
    }

    #[test]
    fn grid_serde_roundtrip() {
        let g = sample_grid();
        let s = serde_json::to_string(&g).unwrap();
        let back: OccupancyGrid2D = serde_json::from_str(&s).unwrap();
        assert_eq!(back.origin, g.origin);
        assert_eq!(back.width, g.width);
        assert_eq!(back.height, g.height);
        assert_eq!(back.cells.len(), g.cells.len());
    }

    #[test]
    fn height_map_empty_has_no_valid() {
        let h = HeightMap::empty(Point2::new(0.0, 0.0), 0.5, 2, 2);
        assert!(!h.has_any_valid());
    }

    #[test]
    fn height_map_valid_cell_detected() {
        let cells = vec![
            HeightCell::default(),
            HeightCell {
                elevation: 1.0,
                variance: 0.1,
                valid: true,
            },
        ];
        let h = HeightMap::new(Point2::new(0.0, 0.0), 1.0, 2, 1, cells);
        assert!(h.has_any_valid());
    }

    #[test]
    fn terrain_map_new_validates_lengths() {
        let h = HeightMap::empty(Point2::new(0.0, 0.0), 1.0, 2, 2);
        let t = vec![0.5_f32; 4];
        let cm = Some(vec![1.0_f32; 4]);
        let m = TerrainMap::new(h, t, cm);
        assert_eq!(m.traversability.len(), 4);
    }
}
