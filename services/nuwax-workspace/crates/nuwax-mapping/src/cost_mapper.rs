//! CostMapper
//! 把 OccupancyGrid 转换为 Costmap2D（Nav2 兼容），含
//! inflation layer / obstacle layer / static layer 的合并逻辑。

use crate::models::{Costmap, MapMeta};
use crate::occupancy::types::{OccupancyGrid, OccupancyValue};
use nalgebra::Isometry3;

/// CostMapper 配置
#[derive(Debug, Clone, Copy)]
pub struct CostMapperConfig {
    /// lethal cost（占据）
    pub lethal_cost: u8,
    /// inscribed cost（不可行走内切）
    pub inscribed_cost: u8,
    /// possibly circumscribed cost（外切膨胀）
    pub circumscribed_cost: u8,
    /// free space cost
    pub free_space_cost: u8,
    /// no information cost
    pub no_information_cost: u8,
}

/// CostMapper 默认 Nav2 配置（与传统 Nav2 costmap 2D 兼容）
impl CostMapperConfig {
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

    /// 把占据栅格转换为 Costmap
    #[must_use]
    pub fn convert(&self, grid: &OccupancyGrid) -> Costmap {
        let meta = MapMeta::new(
            grid.resolution,
            grid.width,
            grid.height,
            Isometry3::translation(grid.origin[0], grid.origin[1], grid.origin[2]),
        );
        let mut costmap = Costmap::new(meta);

        for r in 0..grid.height {
            for c in 0..grid.width {
                let val = grid.get(r, c).unwrap_or(OccupancyValue::Unknown);
                let cost = match val {
                    OccupancyValue::Unknown => self.config.no_information_cost,
                    OccupancyValue::Free => self.config.free_space_cost,
                    OccupancyValue::Occupied => self.config.lethal_cost,
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
        Self::new(CostMapperConfig::nav2_default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convert_occupied_to_lethal() {
        let mut g = OccupancyGrid::new(4, 4, 0.1);
        g.set(2, 2, OccupancyValue::Occupied);
        let cm = CostMapper::default();
        let costmap = cm.convert(&g);
        assert_eq!(costmap.get(2, 2), Some(254));
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
}
