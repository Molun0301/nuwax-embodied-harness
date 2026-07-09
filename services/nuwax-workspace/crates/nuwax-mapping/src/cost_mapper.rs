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
//! 已知 P2 follow-up（见审计文档）：
//! - 真接 `dimos.mapping.pointclouds.occupancy.OCCUPANCY_ALGOS` dict 算法表
//!   （height_cost / probability / probabilistic），而不是 u8 数值映射。

use crate::models::{Costmap, MapMeta};
use crate::occupancy::types::{MapMetaData, OccupancyGrid, OccupancyValue};
use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};

#[cfg(test)]
use crate::occupancy::types::Pose;

/// 算法 selector（Python `Config.algo` 取值）
///
/// 对应 `dimos.mapping.pointclouds.occupancy.OCCUPANCY_ALGOS` 的 key。
/// 当前仅声明枚举；真实算法实现留 P2。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum CostAlgoSelector {
    /// 高度切片成本（Python `HeightCostConfig`，默认）
    #[default]
    HeightCost,
    /// 占据概率（Python `ProbabilityCostConfig`）
    Probability,
    /// 概率式占据（Python `ProbabilisticOccupancyConfig`）
    Probabilistic,
}

/// 算法参数占位（对应 Python `OccupancyConfig` 子类）
///
/// Python 各 algo 字段不同（height_min/height_max/...），本期仅保留算法名 +
/// 公共 initial_safe_radius_meters；具体字段迁移留 P2。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OccupancyConfig {
    /// 选中算法
    pub algo: CostAlgoSelector,
    /// 初始安全半径（m），对应 Python `initial_safe_radius_meters`
    pub initial_safe_radius_meters: f32,
}

impl Default for OccupancyConfig {
    fn default() -> Self {
        Self {
            algo: CostAlgoSelector::default(),
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

        // dimos OccupancyGrid 用 IntEnum 三态；Costmap (Nav2 u8) 直接映射。
        // 这里只做 protocol-level 1:1 转换（不在 mapping 接口硬编 Nav2 cost）。
        for r in 0..grid.height {
            for c in 0..grid.width {
                let val = grid.get(r, c).unwrap_or(OccupancyValue::Unknown);
                let cost: u8 = match val {
                    OccupancyValue::Free => 0,
                    OccupancyValue::Occupied => 100,
                    OccupancyValue::Unknown => 255,
                };
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

// 旧的 `CostMapperConfig::nav2_default()` 兼容垫片，标记 deprecated。
// 旧消费方（nav stack 适配层）仍可调用；dimos-side 改用 `dimos_default()`。
#[allow(deprecated)]
impl CostMapperConfig {
    /// 兼容垫片：返回 dimos 默认（已不再带 Nav2 数值常量）
    #[deprecated(
        since = "0.2.0",
        note = "Nav2 数值常量不在 dimos 协议中；请用 `dimos_default()` + nav stack 适配层 `Nav2CostLayer`"
    )]
    #[must_use]
    pub fn nav2_default() -> Self {
        Self::dimos_default()
    }
}

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
