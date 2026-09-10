//! CostMapper
//!
//! SSOT：`/home/ml/Deeprob/dimos/dimos/mapping/costmapper.py`（Config: algo + OccupancyConfig）
//!
//! 修复要点（见 docs/.../20_rework20_全仓审计_对dimos原始.md §2.1 H-5/H-6）：
//! - 原 Rust `CostMapperConfig` 是 Nav2 cost layer 数值常量（lethal_cost=254 等），
//!   **不在 dimos Python 协议中**——Python `CostMapper.config.algo` 是 selector
//!   字符串（如 `"height_cost"`），`OccupancyConfig` 是对应算法参数。
//! - 数值常量（lethal=254 / inscribed=253 / no_info=255）属 Nav2 costmap_2d
//!   协议，不属于 mapping 接口；本期保留为 `Nav2CostLayer`（独立类型）以避免
//!   误用，但 CostMapperConfig 改为 selector 风格。
//!
//! Point-cloud inputs dispatch to the original `height_cost`, `general`, and
//! `simple` occupancy kernels through [`PointCloudOccConfig`].

use crate::models::{Costmap, MapMeta};
use crate::occupancy::types::{MapMetaData, OccupancyGrid, OccupancyValue};
use crate::pointclouds::occupancy::{
    build_occupancy_auto, MappingError, PointCloudOccAlgo, PointCloudOccConfig,
};
use crate::pointclouds::PointCloudOccupancyBuilder;
use nalgebra::{Isometry3, Point3};
use serde::{Deserialize, Serialize};

#[cfg(test)]
use crate::occupancy::types::Pose;

/// 算法 selector（Python `Config.algo` 取值）
///
/// 对应 `dimos.mapping.pointclouds.occupancy.OCCUPANCY_ALGOS` 的 key。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum CostAlgoSelector {
    /// 高度切片成本（Python `HeightCostConfig`，默认）
    #[default]
    HeightCost,
    /// 原版通用 obstacle/ground occupancy。
    General,
    /// 原版 morphology-closing simple occupancy。
    Simple,
    /// 旧配置别名；运行时映射到 [`Self::General`]。
    Probability,
    /// 旧配置别名；运行时映射到 [`Self::Simple`]。
    Probabilistic,
}

/// CostMapper selector, exact point-cloud kernel parameters, and safe radius.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyConfig {
    /// 选中算法
    pub algo: CostAlgoSelector,
    /// Original algorithm-specific point-cloud parameters.
    #[serde(default)]
    pub pointcloud: PointCloudOccConfig,
    /// 初始安全半径（m），对应 Python `initial_safe_radius_meters`
    pub initial_safe_radius_meters: f32,
}

impl Default for OccupancyConfig {
    fn default() -> Self {
        Self {
            algo: CostAlgoSelector::default(),
            pointcloud: PointCloudOccConfig::default(),
            initial_safe_radius_meters: 0.0,
        }
    }
}

/// CostMapper 配置（Python `CostMapper.config` 直映射）
#[derive(Debug, Clone)]
pub struct CostMapperConfig {
    /// 算法 selector + 参数
    pub config: OccupancyConfig,
}

/// Nav2 风格 cost layer 数值常量（独立于 mapping 接口）
///
/// 这些值（254/253/255）属 Nav2 `nav2_costmap_2d::Costmap2D` 协议，**不在**
/// dimos mapping 协议中。如果未来要把 Costmap 喂给 Nav2，应在 navigation
/// 适配层做转换，而不是在 mapping 接口暴露。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Nav2CostLayer {
    /// lethal cost（占据）
    pub lethal_cost: u8,
    /// inscribed cost（不可行走内切）
    pub inscribed_cost: u8,
    /// circumscribed cost（外切膨胀）
    pub circumscribed_cost: u8,
    /// free space cost
    pub free_space_cost: u8,
    /// no information cost
    pub no_information_cost: u8,
}

impl Nav2CostLayer {
    /// Nav2 风格默认配置
    #[must_use]
    pub fn nav2_default() -> Self {
        Self {
            lethal_cost: 254,
            inscribed_cost: 253,
            circumscribed_cost: 128,
            free_space_cost: 0,
            no_information_cost: 255,
        }
    }
}

impl CostMapperConfig {
    /// Python 默认（HeightCost 算法）
    #[must_use]
    pub fn dimos_default() -> Self {
        Self {
            config: OccupancyConfig::default(),
        }
    }
}

/// CostMapper
pub struct CostMapper {
    config: CostMapperConfig,
}

impl CostMapper {
    /// 构造
    #[must_use]
    pub fn new(config: CostMapperConfig) -> Self {
        Self { config }
    }

    /// 把占据栅格转换为 Costmap（dimos OccupancyGrid 直映射）
    #[must_use]
    pub fn convert(&self, grid: &OccupancyGrid) -> Costmap {
        let meta = MapMeta::from(grid.to_meta());
        let mut costmap = Costmap::new(meta);

        // 保留原始 int8 cost：-1 → 255，0..=100 原样进入 Costmap。
        for r in 0..grid.height {
            for c in 0..grid.width {
                let raw = grid.data[(r * grid.width + c) as usize];
                let cost = if raw < 0 { 255 } else { raw as u8 };
                costmap.set(r, c, cost);
            }
        }

        costmap
    }

    /// 当前配置
    #[must_use]
    pub fn config(&self) -> &CostMapperConfig {
        &self.config
    }
}

// =====================================================================
// PointCloud2 接 CostMapper 适配层（P0-09 修复 H-5/H-6）
// =====================================================================

/// PointCloud2 抽象（不依赖 ROS message 类型）
///
/// 对齐 `sensor_msgs/PointCloud2` 的最小字段集：xyz 浮点字段 + sensor origin。
/// 测试 / 上层（Lcm/ros-bridge）可实现此 trait 把任意输入投影到 cost_mapper。
/// 本期不引入 `sensor-msgs` 依赖以避免 cleaves ROS 协议进 mapping 接口。
pub trait PointCloud2Like {
    /// 点列表（已变换到世界坐标系）
    fn points(&self) -> &[Point3<f64>];
    /// Sensor origin retained for adapters that need the acquisition pose.
    fn sensor_origin(&self) -> Point3<f64>;
    /// Point-cloud coordinate frame. Legacy adapters default to `world`.
    fn frame_id(&self) -> &str {
        "world"
    }
    /// Point-cloud timestamp in seconds. Legacy adapters default to zero.
    fn timestamp(&self) -> f64 {
        0.0
    }
}

/// 内置简单实现：内存中的 xyz + sensor origin
#[derive(Debug, Clone)]
pub struct SimplePointCloud2 {
    /// xyz 点列表
    pub points: Vec<Point3<f64>>,
    /// 传感器 origin
    pub sensor_origin: Point3<f64>,
    /// Coordinate frame propagated by auto-bounds occupancy conversion.
    pub frame_id: String,
    /// Timestamp in seconds propagated by auto-bounds occupancy conversion.
    pub timestamp: f64,
}

impl SimplePointCloud2 {
    /// 构造
    #[must_use]
    pub fn new(points: Vec<Point3<f64>>, sensor_origin: Point3<f64>) -> Self {
        Self {
            points,
            sensor_origin,
            frame_id: "world".to_string(),
            timestamp: 0.0,
        }
    }

    /// Construct an in-memory cloud with frame and timestamp metadata.
    #[must_use]
    pub fn with_metadata(
        points: Vec<Point3<f64>>,
        sensor_origin: Point3<f64>,
        frame_id: impl Into<String>,
        timestamp: f64,
    ) -> Self {
        Self {
            points,
            sensor_origin,
            frame_id: frame_id.into(),
            timestamp,
        }
    }
}

impl PointCloud2Like for SimplePointCloud2 {
    fn points(&self) -> &[Point3<f64>] {
        &self.points
    }
    fn sensor_origin(&self) -> Point3<f64> {
        self.sensor_origin
    }
    fn frame_id(&self) -> &str {
        &self.frame_id
    }
    fn timestamp(&self) -> f64 {
        self.timestamp
    }
}

fn selected_pointcloud_config(occupancy_cfg: &OccupancyConfig) -> PointCloudOccConfig {
    let mut pointcloud_config = occupancy_cfg.pointcloud;
    pointcloud_config.algo = match occupancy_cfg.algo {
        CostAlgoSelector::HeightCost => PointCloudOccAlgo::HeightCost,
        CostAlgoSelector::General | CostAlgoSelector::Probability => PointCloudOccAlgo::General,
        CostAlgoSelector::Simple | CostAlgoSelector::Probabilistic => PointCloudOccAlgo::Simple,
    };
    pointcloud_config
}

fn apply_initial_safe_radius(grid: &mut OccupancyGrid, radius_meters: f32) {
    let radius_m = f64::from(radius_meters.max(0.0));
    if radius_m <= 0.0 || grid.data.is_empty() {
        return;
    }
    let effective_radius = radius_m + grid.resolution * 0.5;
    let radius2 = effective_radius * effective_radius;
    for row in 0..grid.height {
        for col in 0..grid.width {
            let world_x = f64::from(col) * grid.resolution + grid.origin[0];
            let world_y = f64::from(row) * grid.resolution + grid.origin[1];
            if world_x * world_x + world_y * world_y <= radius2 {
                grid.set(row, col, OccupancyValue::Free);
            }
        }
    }
}

impl CostMapper {
    /// 从 PointCloud2 + OccupancyConfig 构造 CostMapper（完整 pipeline）
    ///
    /// 流程：
    /// 1. `PointCloudOccupancyBuilder` projects points into an `OccupancyGrid`;
    /// 2. 依据 `OccupancyConfig.algo` 分派 HeightCost / General / Simple；
    /// 3. 按原版 CostMapper 语义，把机器人世界原点附近的 cell 清成 FREE。
    ///
    /// 返回：(CostMapper, Costmap) —— caller 同时拿到 CostMapper 实例
    /// （便于后续 in-place 更新）和最终 Costmap。
    #[must_use]
    pub fn from_pointcloud<P: PointCloud2Like>(
        cloud: &P,
        occupancy_cfg: &OccupancyConfig,
        width: u32,
        height: u32,
        resolution: f64,
        origin_xy: (f64, f64),
    ) -> (Self, Costmap) {
        // 阶段 1：pointcloud → OccupancyGrid
        let builder = PointCloudOccupancyBuilder::new(
            cloud.sensor_origin(),
            resolution,
            width,
            height,
            origin_xy,
        );
        let pointcloud_config = selected_pointcloud_config(occupancy_cfg);
        let mut grid = builder.build_with_config(cloud.points(), &pointcloud_config);
        apply_initial_safe_radius(&mut grid, occupancy_cfg.initial_safe_radius_meters);

        // 阶段 4：OccGrid → Costmap（走现有 convert）
        let mut normalized_config = occupancy_cfg.clone();
        normalized_config.pointcloud = pointcloud_config;
        let mapper = Self::new(CostMapperConfig {
            config: normalized_config,
        });
        let costmap = mapper.convert(&grid);
        (mapper, costmap)
    }

    /// Build original 1 m padded bounds and preserve cloud metadata.
    ///
    /// The returned occupancy grid retains `frame_id` and timestamp, while the
    /// costmap carries the same resolution, dimensions, and origin. Existing
    /// fixed-bounds callers can continue using [`Self::from_pointcloud`].
    ///
    /// # Errors
    /// Returns [`MappingError`] for invalid resolution/configuration or bounds
    /// that cannot be represented by the Rust grid container.
    pub fn from_pointcloud_auto<P: PointCloud2Like + ?Sized>(
        cloud: &P,
        occupancy_cfg: &OccupancyConfig,
        resolution: f64,
    ) -> Result<(Self, OccupancyGrid, Costmap), MappingError> {
        let grid = Self::occupancy_from_pointcloud_auto(cloud, occupancy_cfg, resolution)?;

        let mut normalized_config = occupancy_cfg.clone();
        normalized_config.pointcloud = selected_pointcloud_config(occupancy_cfg);
        let mapper = Self::new(CostMapperConfig {
            config: normalized_config,
        });
        let costmap = mapper.convert(&grid);
        Ok((mapper, grid, costmap))
    }

    /// 只生成占据栅格；不为只消费栅格的建图拥有方分配并逐格转换未使用的代价图。
    ///
    /// # Errors
    /// 配置、分辨率或地图范围无效时返回与完整转换入口相同的错误。
    pub fn occupancy_from_pointcloud_auto<P: PointCloud2Like + ?Sized>(
        cloud: &P,
        occupancy_cfg: &OccupancyConfig,
        resolution: f64,
    ) -> Result<OccupancyGrid, MappingError> {
        let pointcloud_config = selected_pointcloud_config(occupancy_cfg);
        let mut grid = build_occupancy_auto(cloud, resolution, &pointcloud_config)?;
        apply_initial_safe_radius(&mut grid, occupancy_cfg.initial_safe_radius_meters);
        Ok(grid)
    }
}

/// 从 OccupancyConfig 推断的算法名（dimos Python `Config.algo`）
impl CostAlgoSelector {
    /// dimos Python 字符串（如 `"height_cost"`）
    #[must_use]
    pub fn as_dimos_str(&self) -> &'static str {
        match self {
            Self::HeightCost => "height_cost",
            Self::General => "general",
            Self::Simple => "simple",
            Self::Probability => "probability",
            Self::Probabilistic => "probabilistic",
        }
    }

    /// 从 dimos 字符串解析（未知值 fallback 到 HeightCost）
    #[must_use]
    pub fn from_dimos_str(s: &str) -> Self {
        match s {
            "height_cost" => Self::HeightCost,
            "general" => Self::General,
            "simple" => Self::Simple,
            "probability" => Self::Probability,
            "probabilistic" => Self::Probabilistic,
            _ => Self::HeightCost,
        }
    }
}

// 平移 vector 单元测试辅助

impl Default for CostMapper {
    fn default() -> Self {
        Self::new(CostMapperConfig::dimos_default())
    }
}

// `MapMeta::from(MapMetaData)` 桥接：dimos OccupancyGrid.info ↔ nuwax Costmap.metadata
impl crate::models::MapMeta {
    /// 从 dimos `MapMetaData` 构造（origin 用 Isometry3 平移分量）
    pub(crate) fn from(info: MapMetaData) -> Self {
        Self {
            map_load_time: info.map_load_time,
            resolution: info.resolution,
            width: info.width,
            height: info.height,
            origin: Isometry3::translation(
                info.origin.position[0],
                info.origin.position[1],
                info.origin.position[2],
            ),
        }
    }
}

// `OccupancyGrid.to_meta()` 返回 MapMetaData，给 cost_mapper 用；上面已声明。

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dimos_selector_default_is_height_cost() {
        let cfg = CostMapperConfig::dimos_default();
        assert_eq!(cfg.config.algo, CostAlgoSelector::HeightCost);
        assert!((cfg.config.initial_safe_radius_meters - 0.0).abs() < f32::EPSILON);
    }

    #[test]
    fn convert_occupied_to_100() {
        let mut g = OccupancyGrid::new(4, 4, 0.1);
        g.set(2, 2, OccupancyValue::Occupied);
        let cm = CostMapper::default();
        let costmap = cm.convert(&g);
        // dimos 直接映射 Occupied → 100（不是 Nav2 254）
        assert_eq!(costmap.get(2, 2), Some(100));
    }

    #[test]
    fn convert_unknown_to_255() {
        let g = OccupancyGrid::new(2, 2, 0.1);
        let cm = CostMapper::default();
        let costmap = cm.convert(&g);
        assert_eq!(costmap.get(0, 0), Some(255));
    }

    #[test]
    fn convert_free_to_zero() {
        let mut g = OccupancyGrid::new(2, 2, 0.1);
        g.set(1, 1, OccupancyValue::Free);
        let cm = CostMapper::default();
        let costmap = cm.convert(&g);
        assert_eq!(costmap.get(1, 1), Some(0));
    }

    #[test]
    fn nav2_cost_layer_independent() {
        let n = Nav2CostLayer::nav2_default();
        assert_eq!(n.lethal_cost, 254);
        assert_eq!(n.inscribed_cost, 253);
        assert_eq!(n.no_information_cost, 255);
    }

    #[test]
    fn map_meta_origin_uses_pose_translation() {
        let p = Pose {
            position: [1.0, 2.0, 3.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        let info = MapMetaData::new(0.05, 10, 10, p);
        let m = crate::models::MapMeta::from(info);
        let t = m.origin.translation;
        assert!((t.x - 1.0).abs() < 1e-9);
        assert!((t.y - 2.0).abs() < 1e-9);
        assert!((t.z - 3.0).abs() < 1e-9);
    }
}
