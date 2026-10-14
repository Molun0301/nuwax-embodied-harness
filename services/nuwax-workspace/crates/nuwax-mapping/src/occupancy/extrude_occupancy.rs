//! 3D 占据栅格膨胀
//! 把 2D OccupancyGrid 沿 Z 轴（高度）扩展为多层（3D）占据体，
//! 用于多层建筑的 3D 路径规划。

use crate::occupancy::types::OccupancyGrid;

/// 3D 占据体膨胀器
pub struct ExtrudeOccupancy;

impl ExtrudeOccupancy {
    /// 把 2D 栅格沿 Z 轴按 `num_layers` 层叠加
    ///
    /// Returns `Vec<OccupancyGrid>`，每层代表一个 Z 切片（共享同 origin XY）。
    #[must_use]
    pub fn extrude_layers(
        grid: &OccupancyGrid,
        num_layers: u32,
        layer_height: f64,
    ) -> Vec<OccupancyGrid> {
        let mut out = Vec::with_capacity(num_layers as usize);
        for layer_idx in 0..num_layers {
            let mut layer = grid.clone();
            // 修改 origin z 反映当前层
            layer.origin[2] = layer_idx as f64 * layer_height;
            out.push(layer);
        }
        out
    }

    /// 高度过滤：保留指定 Z 范围内的 cell，移除其他
    #[must_use]
    pub fn filter_by_height(
        layers: &[OccupancyGrid],
        min_z: f64,
        max_z: f64,
    ) -> Vec<OccupancyGrid> {
        layers
            .iter()
            .filter(|g| g.origin[2] >= min_z && g.origin[2] <= max_z)
            .cloned()
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extrude_3_layers() {
        let g = OccupancyGrid::new(2, 2, 0.1);
        let layers = ExtrudeOccupancy::extrude_layers(&g, 3, 0.5);
        assert_eq!(layers.len(), 3);
        assert_eq!(layers[0].origin[2], 0.0);
        assert_eq!(layers[1].origin[2], 0.5);
        assert_eq!(layers[2].origin[2], 1.0);
    }

    #[test]
    fn filter_by_height() {
        let g = OccupancyGrid::new(2, 2, 0.1);
        let layers = ExtrudeOccupancy::extrude_layers(&g, 5, 0.5);
        let filtered = ExtrudeOccupancy::filter_by_height(&layers, 0.5, 1.5);
        assert_eq!(filtered.len(), 3); // z = 0.5, 1.0, 1.5
    }
}
