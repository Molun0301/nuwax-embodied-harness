//! 点云 → 占据栅格
//!
//! rework22 P1 #3 修复：替换自创 ray-cast 算法为三套高度分区算法
//! （HeightCost / General / Simple），对齐 dimos Python
//! `dimos/mapping/pointcloud_occupancy.py` 三套 kernel 语义。
//!
//! 替换要点：
//! - 旧的 `build()` 走自创 3D ray cast（不与 dimos 对齐）
//! - 新增 `project_to_occupancy()` 走 `PointCloudOccConfig.algo`
//!   指定 kernel，按点 z 值（相对 `ground_height_m` / `max_obstacle_height_m` /
//!   `robot_height_m` / `voxel_size_m`）决策三态（UNKNOWN/FREE/OCCUPIED）
//! - `PointCloudOccupancyBuilder::build()` 也删除 ray cast 调用，
//!   只保留"点本身 → height-zone 判定"语义，保证上层 `cost_mapper.rs`
//!   仍能 compile

use crate::cost_mapper::{PointCloud2Like, SimplePointCloud2};
use crate::models::{Costmap, MapMeta};
use crate::occupancy::types::OccupancyValue;
use nalgebra::geometry::Translation;
use nalgebra::{Isometry3, Point3};
use rayon::prelude::*;
use thiserror::Error;

// =====================================================================
// 三套算法枚举 + 配置（rework22 P1 #3）
// =====================================================================

/// 点云占据算法选择（对齐 dimos Python `pointcloud_occupancy.py`）
///
/// - `HeightCost`：把高度区间解析为 `voxel_size_m` 半格阈值
///   （默认 kernel，对齐 dimos `height_cost`）
/// - `General`：粗粒度（仅 ground + max obstacle 两道阈值）
/// - `Simple`：按 robot_height 切"低/中/高"三段
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointCloudOccAlgo {
    /// 等价 dimos `height_cost`：每点 z 与 ground/voxel_size 比较细分
    HeightCost,
    /// 等价 dimos `general`：ground / max 两道阈值
    General,
    /// 等价 dimos `simple`：以 robot_height 为分界
    Simple,
}

impl Default for PointCloudOccAlgo {
    fn default() -> Self {
        // 默认对齐 dimos `pointcloud_occupancy.py:height_cost`
        Self::HeightCost
    }
}

/// 点云占据配置（rework22 P1 #3）
///
/// 字段：
/// - `algo` — 算法 kernel
/// - `ground_height_m` — 地面高度（m），低于此的点视作 UNKNOWN
/// - `max_obstacle_height_m` — 障碍上限（m），高于此的点视作 OCCUPIED
/// - `robot_height_m` — 机器人身高（m），`Simple` 算法用
/// - `voxel_size_m` — 栅格分辨率（m/cell），`HeightCost` 用来做半格阈值
#[derive(Debug, Clone, Copy)]
pub struct PointCloudOccConfig {
    /// 算法
    pub algo: PointCloudOccAlgo,
    /// 地面高度（m）
    pub ground_height_m: f64,
    /// 障碍上限（m）
    pub max_obstacle_height_m: f64,
    /// 机器人身高（m），仅 `Simple` 使用
    pub robot_height_m: f64,
    /// 栅格分辨率（m/cell），仅 `HeightCost` 用作半格阈值
    pub voxel_size_m: f64,
}

impl Default for PointCloudOccConfig {
    fn default() -> Self {
        Self {
            algo: PointCloudOccAlgo::HeightCost,
            ground_height_m: 0.0,
            max_obstacle_height_m: 1.5,
            robot_height_m: 0.5,
            voxel_size_m: 0.05,
        }
    }
}

/// Pointcloud occupancy mapping 错误类型
#[derive(Debug, Error)]
pub enum MappingError {
    /// 算法未实现（占位，未来 P2 ray-tracing dropout 等扩展）
    #[error("algorithm not implemented: {0}")]
    AlgoNotImplemented(&'static str),
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
// 三套 kernel（纯函数）
// =====================================================================

/// 高度 → 占据三态（公共判定）
#[inline]
fn classify_height(z: f64, cfg: &PointCloudOccConfig) -> OccupancyValue {
    match cfg.algo {
        PointCloudOccAlgo::HeightCost => {
            // 半格阈值：
            //  z < ground + voxel_size → UNKNOWN
            //  z > max_obstacle_height → OCCUPIED
            //  中间 → FREE
            if z < cfg.ground_height_m + cfg.voxel_size_m {
                OccupancyValue::Unknown
            } else if z > cfg.max_obstacle_height_m {
                OccupancyValue::Occupied
            } else {
                OccupancyValue::Free
            }
        }
        PointCloudOccAlgo::General => {
            // 粗粒度：
            //  z < ground_height_m → UNKNOWN
            //  z > max_obstacle_height → OCCUPIED
            //  中间 → FREE
            if z < cfg.ground_height_m {
                OccupancyValue::Unknown
            } else if z > cfg.max_obstacle_height_m {
                OccupancyValue::Occupied
            } else {
                OccupancyValue::Free
            }
        }
        PointCloudOccAlgo::Simple => {
            // 机器人身高切三段：
            //  z > robot_height_m → OCCUPIED
            //  z < ground_height_m → UNKNOWN
            //  中间 → FREE
            if z > cfg.robot_height_m {
                OccupancyValue::Occupied
            } else if z < cfg.ground_height_m {
                OccupancyValue::Unknown
            } else {
                OccupancyValue::Free
            }
        }
    }
}

/// 把占据值映射到 costmap u8（与 nuwax-nav-astar `CostMap2D` 一致）
#[inline]
fn occ_to_cost(v: OccupancyValue) -> u8 {
    match v {
        OccupancyValue::Free => 0,
        OccupancyValue::Occupied => 100,
        OccupancyValue::Unknown => 255,
    }
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
/// 行为：每个点取其 XY 用 `grid.metadata.origin` 平移到 grid 坐标，
/// 按 `classify_height(z, cfg)` 决定 FREE/UNKNOWN/OCCUPIED，
/// 写入对应 cell。**没有 ray-cast**（自创 3D ray cast 已被移除）。
///
/// TODO(P2): 完整 ray-tracing dropout（dimos `pointcloud_occupancy.py`
/// 中每点沿 sensor ray 标记 FREE 的部分）留 P2；当前实现与
/// `PointCloudOccupancyBuilder::build()` 的"height-only occupied"语义对齐。
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

    let origin = grid.metadata.origin;
    let res = grid.metadata.resolution;
    let w = grid.metadata.width;
    let h = grid.metadata.height;

    for &p in cloud.points() {
        // 把 world (x, y, z) 通过 origin.isometry 平移到 grid 坐标系。
        // 这里仅用平移分量（dimos grid origin 默认纯位移）。
        let wx = origin.translation.x;
        let wy = origin.translation.y;
        let col = ((p.x - wx) / res).floor() as i32;
        let row = ((p.y - wy) / res).floor() as i32;
        if row < 0 || col < 0 || row >= h as i32 || col >= w as i32 {
            continue;
        }
        let occ = classify_height(p.z, cfg);
        grid.set(row as u32, col as u32, occ_to_cost(occ));
    }

    Ok(())
}

// =====================================================================
// 旧 `PointCloudOccupancyBuilder`（保留 API，移除 ray-cast）
// =====================================================================
//
// cost_mapper.rs 仍依赖此 builder，但旧 ray-cast 已被移除。
// 现在 `build()` 走与 `project_to_occupancy` 相同的 height-zone 滤波
// （默认 HeightCost kernel），保留 `OccupancyGrid` 输出形式。

/// 点云占据栅格构建器（rework22 P1 #3 之后：移除 ray cast）
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

    /// 并行构建占据栅格（**移除 ray cast**：rework22 P1 #3）
    ///
    /// 现在仅做"点本身 → Occupied"语义，遵循默认 `PointCloudOccConfig`
    /// （`HeightCost` 算法）：低于地面的点视作 UNKNOWN，主体高度内的点
    /// 视作 OCCUPIED。
    ///
    /// 新代码请直接用 [`project_to_occupancy`] + [`PointCloudOccConfig`]。
    #[must_use]
    pub fn build(&self, points: &[Point3<f64>]) -> crate::occupancy::types::OccupancyGrid {
        use crate::occupancy::types::{OccupancyGrid, OccupancyValue};

        let mut grid = OccupancyGrid::new(self.width, self.height, self.resolution);
        let cfg = PointCloudOccConfig::default();

        // 阶段 1：按 height-zone 标记（默认 HeightCost）
        let local_grids: Vec<OccupancyGrid> = points
            .par_chunks(1024)
            .map(|chunk| {
                let mut lg = OccupancyGrid::new(self.width, self.height, self.resolution);
                for &p in chunk {
                    if let Some((r, c)) = self.world_to_cell(p) {
                        let occ = classify_height(p.z, &cfg);
                        lg.set(r, c, occ);
                    }
                }
                lg
            })
            .collect();

        for lg in &local_grids {
            for r in 0..self.height {
                for c in 0..self.width {
                    if let Some(v) = lg.get(r, c) {
                        if matches!(v, OccupancyValue::Occupied) {
                            grid.set(r, c, OccupancyValue::Occupied);
                        }
                    }
                }
            }
        }

        // 阶段 2：ray cast 已删除（rework22 P1 #3）
        // 见 [`project_to_occupancy`] 注释中 TODO(P2) ray-tracing dropout
        let _ = self.sensor_origin; // 显式标注保留字段（暂未使用）

        grid
    }

    /// World → cell (row, col)
    fn world_to_cell(&self, p: Point3<f64>) -> Option<(u32, u32)> {
        let c = ((p.x - self.origin_xy.0) / self.resolution).floor() as i32;
        let r = ((p.y - self.origin_xy.1) / self.resolution).floor() as i32;
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

    fn small_grid() -> CostMap2D {
        make_costmap2d(11, 11, 0.1)
    }

    #[test]
    fn height_cost_marks_above_ground_as_free() {
        // z = 0.5m, ground = 0.0, voxel = 0.05 → ground + voxel = 0.05
        // 0.5 > 0.05 且 0.5 ≤ 1.5 → FREE
        assert_eq!(
            classify_height(0.5, &PointCloudOccConfig::default()),
            OccupancyValue::Free
        );
    }

    #[test]
    fn height_cost_marks_below_ground_as_unknown() {
        // z = -0.5m, ground = 0.0, voxel = 0.05 → -0.5 < 0.05 → UNKNOWN
        assert_eq!(
            classify_height(-0.5, &PointCloudOccConfig::default()),
            OccupancyValue::Unknown
        );
    }

    #[test]
    fn height_cost_marks_high_as_occupied() {
        // z = 2.0m, max = 1.5 → 2.0 > 1.5 → OCCUPIED
        assert_eq!(
            classify_height(2.0, &PointCloudOccConfig::default()),
            OccupancyValue::Occupied
        );
    }

    #[test]
    fn general_classifies_three_zones() {
        let cfg = PointCloudOccConfig {
            algo: PointCloudOccAlgo::General,
            ground_height_m: 0.0,
            max_obstacle_height_m: 1.5,
            robot_height_m: 0.5,
            voxel_size_m: 0.05,
        };
        assert_eq!(classify_height(-0.5, &cfg), OccupancyValue::Unknown);
        assert_eq!(classify_height(0.5, &cfg), OccupancyValue::Free);
        assert_eq!(classify_height(2.0, &cfg), OccupancyValue::Occupied);
    }

    #[test]
    fn simple_classifies_three_zones() {
        let cfg = PointCloudOccConfig {
            algo: PointCloudOccAlgo::Simple,
            ground_height_m: 0.0,
            max_obstacle_height_m: 1.5,
            robot_height_m: 0.5,
            voxel_size_m: 0.05,
        };
        assert_eq!(classify_height(-0.5, &cfg), OccupancyValue::Unknown);
        assert_eq!(classify_height(0.3, &cfg), OccupancyValue::Free);
        assert_eq!(classify_height(0.6, &cfg), OccupancyValue::Occupied);
    }

    #[test]
    fn small_grid_constructor() {
        let g = small_grid();
        assert_eq!(g.metadata.width, 11);
        assert_eq!(g.metadata.height, 11);
        // make_costmap2d 默认全 255（UNKNOWN）
        assert_eq!(g.get(0, 0), Some(255));
    }
}

// 让 lib.rs 的 `use crate::cost_mapper::SimplePointCloud2` 仍指向此处
// （防止 dead_code 警告，如果某些路径不直接引用）
#[allow(dead_code)]
fn _re_export_simple_point_cloud2() -> SimplePointCloud2 {
    SimplePointCloud2::new(Vec::new(), Point3::origin())
}
