//! GridPoint + ClassificationGrid(扁平 Vec,行优先;无 HashMap 分配)。

use nuwax_nav_types_messages::grid::OccupancyCell;

use crate::classification::PointClassification;

/// Convert the typed cell representation back to the numeric occupancy value
/// used by the Python implementation.
pub(crate) const fn occupancy_value(cell: OccupancyCell) -> i16 {
    match cell {
        OccupancyCell::Unknown => -1,
        OccupancyCell::Free => 0,
        OccupancyCell::Occupied | OccupancyCell::Inflated => 100,
        OccupancyCell::Cost(value) => value as i16,
    }
}

/// 栅格内 2D 坐标(行优先索引 `y * width + x`)。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridPoint {
    /// x(列)
    pub x: u32,
    /// y(行)
    pub y: u32,
}

impl GridPoint {
    /// 构造(不做边界检查,调用方保证在 grid 内)
    #[must_use]
    pub const fn new(x: u32, y: u32) -> Self {
        Self { x, y }
    }
}

/// 平铺的分类缓存(避免每次重建 HashMap)。
#[derive(Debug, Clone)]
pub(crate) struct ClassificationGrid {
    /// 宽度
    width: u32,
    /// 高度
    height: u32,
    /// 分类位(行优先)
    classes: Vec<PointClassification>,
}

impl ClassificationGrid {
    /// 构造(预先清零为 NO_INFORMATION)
    #[must_use]
    pub(crate) fn new(width: u32, height: u32) -> Self {
        let len = (width as usize) * (height as usize);
        Self {
            width,
            height,
            classes: vec![PointClassification::NO_INFORMATION; len],
        }
    }

    /// 越界安全索引,越界返回 `None`
    pub(crate) fn index(&self, x: u32, y: u32) -> Option<usize> {
        if x >= self.width || y >= self.height {
            return None;
        }
        Some((y as usize) * (self.width as usize) + (x as usize))
    }

    /// 越界安全读
    pub(crate) fn get(&self, x: u32, y: u32) -> PointClassification {
        match self.index(x, y) {
            Some(i) => self.classes[i],
            None => PointClassification::NO_INFORMATION,
        }
    }

    /// 越界安全写
    pub(crate) fn set(&mut self, x: u32, y: u32, flags: PointClassification) {
        if let Some(i) = self.index(x, y) {
            self.classes[i] = flags;
        }
    }

    /// 按位或写入(越界不 panic)
    pub(crate) fn insert_flags(&mut self, x: u32, y: u32, flags: PointClassification) {
        if let Some(i) = self.index(x, y) {
            self.classes[i] |= flags;
        }
    }

    /// 越界安全 contains
    pub(crate) fn contains_flags(&self, x: u32, y: u32, flags: PointClassification) -> bool {
        self.get(x, y).contains(flags)
    }

    /// 全部清零为 NO_INFORMATION
    #[allow(dead_code)] // 内部 reset API, 当前调用方仅在测试中使用
    pub(crate) fn clear(&mut self) {
        for c in &mut self.classes {
            *c = PointClassification::NO_INFORMATION;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn index_round_trip() {
        let g = ClassificationGrid::new(4, 3);
        for y in 0..3 {
            for x in 0..4 {
                let i = g.index(x, y).unwrap();
                assert_eq!(i, y as usize * 4 + x as usize);
            }
        }
    }

    #[test]
    fn out_of_bounds_returns_no_information_for_get() {
        let g = ClassificationGrid::new(2, 2);
        assert_eq!(g.get(5, 5), PointClassification::NO_INFORMATION);
        assert_eq!(
            g.get(u32::MAX, u32::MAX),
            PointClassification::NO_INFORMATION
        );
    }

    #[test]
    fn out_of_bounds_set_does_not_panic() {
        let mut g = ClassificationGrid::new(2, 2);
        g.set(10, 10, PointClassification::MAP_OPEN);
        // 应不修改任何 cell
        assert!(g.get(0, 0).is_empty());
    }

    #[test]
    fn insert_flags_combines_with_existing() {
        let mut g = ClassificationGrid::new(2, 2);
        g.set(0, 0, PointClassification::MAP_OPEN);
        g.insert_flags(0, 0, PointClassification::FRONTIER_OPEN);
        let v = g.get(0, 0);
        assert!(v.contains(PointClassification::MAP_OPEN));
        assert!(v.contains(PointClassification::FRONTIER_OPEN));
    }

    #[test]
    fn clear_resets_all() {
        let mut g = ClassificationGrid::new(2, 2);
        g.set(0, 0, PointClassification::MAP_OPEN);
        g.set(1, 1, PointClassification::FRONTIER_CLOSED);
        g.clear();
        assert!(g.get(0, 0).is_empty());
        assert!(g.get(1, 1).is_empty());
    }
}
