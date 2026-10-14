//! Costmap：栅格化带历史最大高度的障碍物表示。
//!
//! 仅当新点高度大于该 cell 已有最大高度时覆盖；阻塞缓存由脏标记触发重建。

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::inflation::{blocked_at_inflation, inflate_obstacles};
use crate::NavError;
use nuwax_nav_types_messages::grid::OCCUPANCY_OBSTACLE_THRESHOLD;
use nuwax_nav_types_messages::{
    GridFootprint, OccupancyCell, OccupancyGrid2D, Point2, RobotGeometry,
};

/// Costmap：稀疏 + 历史最大高度模型。
#[derive(Debug, Clone)]
pub struct Costmap {
    cell_size: f64,
    obstacle_height: f64,
    inflation_radius: f64,
    heights: HashMap<(i64, i64), f64>,
    blocked: HashSet<(i64, i64)>,
    blocked_dirty: bool,
    /// 有界占据图输入保留真实原点、分辨率和未知区；地形输入不携带该约束。
    occupancy: Option<OccupancyGrid2D>,
    /// 有界地图的物理轮廓下限，不能通过卡住降级或后续地图更新减小。
    occupancy_inflation_floor: f64,
    /// 本次规划使用的真实机体轮廓，仅存在于当前位置规划副本。
    robot_footprint: Option<GridFootprint>,
    current_robot_position: Option<Point2>,
}

/// 完整占据输入的消费变化；不表示原生构图或传输已经是差量。
#[derive(Debug, Default)]
pub struct OccupancyUpdateStats {
    /// 与上次完整输入不同的格数。
    pub changed_cells: usize,
    /// 覆盖变化的32格边长块数。
    pub changed_blocks: usize,
    /// 进入或退出障碍类别的格数。
    pub obstacle_changes: usize,
    /// 首帧或布局、半径变化重新构建。
    pub full_rebuild: bool,
}

impl Costmap {
    /// 构造 Costmap。`cell_size > 0`，`inflation_radius >= 0`。
    /// # Errors
    /// 当 `cell_size <= 0` 或 `inflation_radius < 0` 时返回 `NavError::InvalidInput`。
    pub fn new(
        cell_size: f64,
        obstacle_height: f64,
        inflation_radius: f64,
    ) -> Result<Self, NavError> {
        if cell_size <= 0.0 {
            return Err(NavError::InvalidInput {
                field: "cell_size",
                reason: format!("must be positive, got {cell_size}"),
            });
        }
        if inflation_radius < 0.0 {
            return Err(NavError::InvalidInput {
                field: "inflation_radius",
                reason: format!("must be non-negative, got {inflation_radius}"),
            });
        }
        Ok(Self {
            cell_size,
            obstacle_height,
            inflation_radius,
            heights: HashMap::new(),
            blocked: HashSet::new(),
            blocked_dirty: true,
            occupancy: None,
            occupancy_inflation_floor: 0.0,
            robot_footprint: None,
            current_robot_position: None,
        })
    }

    /// 从已有 heights map 构造（用于复制/克隆场景）。仍会触发 dirty。
    #[must_use]
    pub fn from_heights(
        cell_size: f64,
        obstacle_height: f64,
        inflation_radius: f64,
        heights: HashMap<(i64, i64), f64>,
    ) -> Self {
        Self {
            cell_size,
            obstacle_height,
            inflation_radius,
            heights,
            blocked: HashSet::new(),
            blocked_dirty: true,
            occupancy: None,
            occupancy_inflation_floor: 0.0,
            robot_footprint: None,
            current_robot_position: None,
        }
    }

    /// 直接消费占据栅格，不将障碍格重新解释为依赖机器人高度的地形点。
    pub fn from_occupancy(grid: OccupancyGrid2D, inflation_radius: f64) -> Result<Self, NavError> {
        if !grid.origin.x.is_finite()
            || !grid.origin.y.is_finite()
            || !grid.resolution.is_finite()
            || grid.resolution <= 0.0
            || grid.width == 0
            || grid.height == 0
            || (grid.width as usize).checked_mul(grid.height as usize) != Some(grid.cells.len())
            || !inflation_radius.is_finite()
        {
            return Err(NavError::InvalidInput {
                field: "occupancy",
                reason: "invalid occupancy geometry, cells or inflation radius".into(),
            });
        }
        let mut map = Self::new(grid.resolution, 1.0, inflation_radius)?;
        for y in 0..grid.height {
            for x in 0..grid.width {
                if matches!(
                    grid.get(x, y),
                    Some(OccupancyCell::Occupied | OccupancyCell::Inflated)
                ) || matches!(grid.get(x, y), Some(OccupancyCell::Cost(value)) if value >= OCCUPANCY_OBSTACLE_THRESHOLD)
                {
                    map.heights.insert((i64::from(x), i64::from(y)), 1.0);
                }
            }
        }
        map.occupancy = Some(grid);
        map.occupancy_inflation_floor = inflation_radius;
        Ok(map)
    }

    /// 只读原占据视图；规划副本上的自体处理沿既有接口完成。
    pub fn occupancy_grid(&self) -> Option<&OccupancyGrid2D> {
        self.occupancy.as_ref()
    }

    /// 同布局只更新真实变化的障碍索引；删除和未知变化不继承历史最大高度。
    pub fn replace_occupancy(&mut self, grid: OccupancyGrid2D, inflation_radius: f64) -> Result<OccupancyUpdateStats, NavError> {
        let compatible = self.occupancy.as_ref().is_some_and(|old| {
            old.width == grid.width && old.height == grid.height && old.resolution == grid.resolution
                && old.origin == grid.origin && old.cells.len() == grid.cells.len()
                && self.robot_footprint.is_none() && self.inflation_radius == inflation_radius
        });
        if !compatible {
            let replacement = Self::from_occupancy(grid, inflation_radius.max(self.occupancy_inflation_floor))?;
            let stats = OccupancyUpdateStats { changed_cells: replacement.occupancy.as_ref().unwrap().cells.len(),
                changed_blocks: (replacement.occupancy.as_ref().unwrap().width as usize).div_ceil(32) * (replacement.occupancy.as_ref().unwrap().height as usize).div_ceil(32),
                obstacle_changes: replacement.heights.len(), full_rebuild: true };
            *self = replacement;
            return Ok(stats);
        }
        let obstacle = |cell: OccupancyCell| matches!(cell, OccupancyCell::Occupied | OccupancyCell::Inflated)
            || matches!(cell, OccupancyCell::Cost(value) if value >= OCCUPANCY_OBSTACLE_THRESHOLD);
        let mut stats = OccupancyUpdateStats::default();
        let mut blocks = BTreeSet::new();
        let old = self.occupancy.as_ref().unwrap();
        for (row, (before, after)) in old.cells.chunks(grid.width as usize).zip(grid.cells.chunks(grid.width as usize)).enumerate() {
            for (block, (a, b)) in before.chunks(32).zip(after.chunks(32)).enumerate() {
                if a == b { continue; }
                blocks.insert((block, row / 32));
                for (offset, (&previous, &value)) in a.iter().zip(b).enumerate() {
                    if previous == value { continue; }
                    stats.changed_cells += 1;
                    if obstacle(previous) != obstacle(value) {
                        let key = ((block * 32 + offset) as i64, row as i64);
                        if obstacle(value) { self.heights.insert(key, 1.0); } else { self.heights.remove(&key); }
                        stats.obstacle_changes += 1;
                    }
                }
            }
        }
        stats.changed_blocks = blocks.len();
        self.blocked_dirty |= stats.obstacle_changes > 0;
        self.occupancy = Some(grid);
        Ok(stats)
    }

    /// 从原地图生成当前位置的规划视图，自体清除不会积累或写回输入。
    pub fn planning_view(
        &self,
        position: Point2,
        yaw: f64,
        geometry: &RobotGeometry,
    ) -> Result<Self, NavError> {
        let mut view = self.clone();
        if let Some(grid) = view.occupancy.as_mut() {
            let footprint = geometry
                .grid_footprint(grid.resolution, yaw, grid.width, grid.height)
                .map_err(|reason| NavError::InvalidInput {
                    field: "robot_geometry",
                    reason,
                })?;
            footprint.clear_current_occupancy(grid, position);
            // 高度表是同一占据图的派生表示，不能残留自体障碍。
            view.heights.retain(|&(x, y), _| {
                u32::try_from(x)
                    .ok()
                    .zip(u32::try_from(y).ok())
                    .and_then(|(x, y)| grid.get(x, y))
                    .is_none_or(|value| !value.is_known_traversable())
            });
            view.robot_footprint = Some(footprint);
            view.current_robot_position = Some(position);
        }
        Ok(view)
    }

    /// 已知自由格才能成为有界地图上的通行点，未知和图外不能被端点解堵放行。
    #[must_use]
    pub fn is_known_free(&self, ix: i64, iy: i64) -> bool {
        let Some(grid) = self.occupancy.as_ref() else {
            return true;
        };
        if ix < 0 || iy < 0 || ix >= i64::from(grid.width) || iy >= i64::from(grid.height) {
            return false;
        }
        grid.get(ix as u32, iy as u32)
            .is_some_and(OccupancyCell::is_known_traversable)
    }

    /// 是否由有界占据图构建。
    #[must_use]
    pub fn has_occupancy(&self) -> bool {
        self.occupancy.is_some()
    }

    /// 有界地图保留已接纳的物理轮廓下限，旧地形输入没有该约束。
    pub(crate) fn minimum_inflation_radius(&self) -> f64 {
        self.occupancy_inflation_floor
    }

    pub(crate) fn preserve_inflation_floor(&mut self, radius: f64) {
        if self.has_occupancy() {
            self.occupancy_inflation_floor = self.occupancy_inflation_floor.max(radius);
            self.set_inflation_radius(self.inflation_radius);
        }
    }

    pub(crate) fn effective_radius(&self, radius: f64) -> f64 {
        radius.max(self.occupancy_inflation_floor)
    }

    /// 候选中心的整个离散圆盘必须已知自由，避免把所有未知格展开成障碍哈希表。
    pub(crate) fn has_footprint_clearance(&self, ix: i64, iy: i64, radius: f64) -> bool {
        let Some(grid) = self.occupancy.as_ref() else {
            return true;
        };
        if let Some(footprint) = self.robot_footprint.as_ref() {
            if let Some(position) = self
                .current_robot_position
                .filter(|position| self.world_to_cell(position.x, position.y) == (ix, iy))
            {
                return footprint.is_clear_at(grid, position);
            }
            return u32::try_from(ix)
                .ok()
                .zip(u32::try_from(iy).ok())
                .is_some_and(|(x, y)| footprint.is_clear(grid, x, y));
        }
        let radius = (self.effective_radius(radius) / self.cell_size).ceil() as i64;
        if radius < 0
            || radius >= i64::from(grid.width)
            || radius >= i64::from(grid.height)
            || ix < radius
            || iy < radius
            || ix >= i64::from(grid.width) - radius
            || iy >= i64::from(grid.height) - radius
        {
            return false;
        }
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy > radius * radius {
                    continue;
                }
                let cell = (ix + dx, iy + dy);
                if !self.is_known_free(cell.0, cell.1)
                    || self
                        .heights
                        .get(&cell)
                        .is_some_and(|height| *height >= self.obstacle_height)
                {
                    return false;
                }
            }
        }
        true
    }

    /// 规划与近距离到达共用端点检查，避免快捷终态绕过地图约束。
    pub(crate) fn permits_endpoints(
        &self,
        start: (i64, i64),
        goal: (i64, i64),
        inflation_radius: f64,
    ) -> bool {
        self.has_footprint_clearance(start.0, start.1, inflation_radius)
            && self.has_footprint_clearance(goal.0, goal.1, inflation_radius)
    }

    /// 真实起终点共享规划副本，保留其格内位置。
    pub(crate) fn permits_positions(&self, start: Point2, goal: Point2, radius: f64) -> bool {
        if let (Some(grid), Some(footprint)) =
            (self.occupancy.as_ref(), self.robot_footprint.as_ref())
        {
            return footprint.is_clear_at(grid, start) && footprint.is_clear_at(grid, goal);
        }
        self.permits_endpoints(
            self.world_to_cell(start.x, start.y),
            self.world_to_cell(goal.x, goal.y),
            radius,
        )
    }

    /// 发布前按真实路径方向检查模型扫掠，防止全局与局部使用不同朝向。
    pub(crate) fn path_has_observed_clearance(
        &self,
        path: &[Point2],
        start_yaw: f64,
        final_yaw: Option<f64>,
    ) -> bool {
        let (Some(grid), Some(footprint)) =
            (self.occupancy.as_ref(), self.robot_footprint.as_ref())
        else {
            return true;
        };
        let mut yaw = start_yaw;
        for segment in path.windows(2) {
            let start = segment[0];
            let end = segment[1];
            let heading = if start.distance(end) > 1e-9 {
                (end.y - start.y).atan2(end.x - start.x)
            } else {
                yaw
            };
            if !footprint.visit_motion(grid.origin, (start, yaw), (end, heading), |x, y| {
                grid.get(x, y)
                    .is_some_and(|value| value.is_known_traversable())
            }) {
                return false;
            }
            yaw = heading;
        }
        if let (Some(&end), Some(target)) = (path.last(), final_yaw) {
            return footprint.visit_motion(grid.origin, (end, yaw), (end, target), |x, y| {
                grid.get(x, y)
                    .is_some_and(|value| value.is_known_traversable())
            });
        }
        true
    }

    fn origin(&self) -> Point2 {
        self.occupancy
            .as_ref()
            .map_or(Point2::new(0.0, 0.0), |grid| grid.origin)
    }

    /// cell 边长（米）。
    #[must_use]
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }

    /// 障碍高度阈值（米）。
    #[must_use]
    pub fn obstacle_height(&self) -> f64 {
        self.obstacle_height
    }

    /// 当前默认膨胀半径（米）。
    #[must_use]
    pub fn inflation_radius(&self) -> f64 {
        self.inflation_radius
    }

    /// 修改默认膨胀半径。修改后立即触发 dirty。
    pub fn set_inflation_radius(&mut self, radius: f64) {
        if !radius.is_finite() || radius < 0.0 {
            // 与构造保持一致：负数禁止，但本方法为 setter 仅记录 trace 并忽略。
            tracing::warn!(radius, "ignored negative inflation_radius in setter");
            return;
        }
        self.inflation_radius = self.effective_radius(radius);
        self.blocked_dirty = true;
    }

    /// 世界坐标 → cell 索引。
    #[must_use]
    pub fn world_to_cell(&self, x: f64, y: f64) -> (i64, i64) {
        (
            ((x - self.origin().x) / self.cell_size).floor() as i64,
            ((y - self.origin().y) / self.cell_size).floor() as i64,
        )
    }

    /// cell 索引 → 世界坐标（cell 中心）。
    #[must_use]
    pub fn cell_to_world(&self, ix: i64, iy: i64) -> (f64, f64) {
        (
            self.origin().x + (ix as f64 + 0.5) * self.cell_size,
            self.origin().y + (iy as f64 + 0.5) * self.cell_size,
        )
    }

    /// 记录一个障碍候选点；高度超过历史最大值才覆盖。
    pub fn update(&mut self, x: f64, y: f64, height: f64) {
        let key = self.world_to_cell(x, y);
        let prev = self.heights.get(&key).copied().unwrap_or(f64::NEG_INFINITY);
        if height > prev {
            self.heights.insert(key, height);
            self.blocked_dirty = true;
        }
    }

    /// 清空所有 cell 高度与阻塞缓存。
    pub fn clear(&mut self) {
        self.heights.clear();
        self.blocked.clear();
        self.blocked_dirty = false;
        self.occupancy = None;
        self.occupancy_inflation_floor = 0.0;
        self.robot_footprint = None;
        self.current_robot_position = None;
    }

    /// 是否被阻塞（按当前默认 inflation_radius）。
    #[must_use]
    pub fn is_blocked(&self, ix: i64, iy: i64) -> bool {
        if self.has_occupancy() {
            !self.has_footprint_clearance(ix, iy, self.inflation_radius)
        } else {
            self.blocked_cells().contains(&(ix, iy))
        }
    }

    /// 触发重建（外部修改 heights 时调用）。
    pub fn mark_dirty(&mut self) {
        self.blocked_dirty = true;
    }

    /// 返回当前默认 inflation 下的所有阻塞 cell。返回克隆，故原缓存不受影响。
    #[must_use]
    pub fn blocked_cells(&self) -> HashSet<(i64, i64)> {
        if self.blocked_dirty {
            inflate_obstacles(
                &self.heights,
                self.obstacle_height,
                self.inflation_radius,
                self.cell_size,
            )
            // 这里没有副作用；由于 `&self`，不能修改自身缓存。
            // 修复方案：mutable 版本走 `blocked_cells_mut`。
        } else {
            self.blocked.clone()
        }
    }

    /// 返回当前默认 inflation 下的所有阻塞 cell（mut 版本，会触发脏重建写回）。
    pub fn blocked_cells_mut(&mut self) -> &HashSet<(i64, i64)> {
        self.rebuild_blocked();
        &self.blocked
    }

    /// 用 heights 重算阻塞集合并清 dirty。
    pub fn rebuild_blocked(&mut self) {
        self.blocked = inflate_obstacles(
            &self.heights,
            self.obstacle_height,
            self.inflation_radius,
            self.cell_size,
        );
        self.blocked_dirty = false;
    }

    /// 在指定 inflation_radius 下计算阻塞集合（**不修改** costmap 的缓存与半径）。
    #[must_use]
    pub fn blocked_at(&self, inflation_radius: f64) -> HashSet<(i64, i64)> {
        blocked_at_inflation(self, inflation_radius)
    }

    /// Heights 引用（只读访问，用于可视化/导出）。
    #[must_use]
    pub fn heights(&self) -> &HashMap<(i64, i64), f64> {
        &self.heights
    }
}

#[cfg(test)]
mod occupancy_update_tests {
    use super::*;
    #[test]
    fn incremental_occupancy_matches_fresh_build_after_deletions_and_layout_changes() {
        let mut grid = OccupancyGrid2D::new(Point2::new(-1.25, -0.75), 0.1, 70, 65, vec![OccupancyCell::Free; 4550]);
        let mut incremental = Costmap::from_occupancy(grid.clone(), 0.15).unwrap();
        for frame in 0..80 {
            let previous = incremental.clone();
            for i in 0..20 {
                let index = (frame * 67 + i * 53) % grid.cells.len();
                grid.cells[index] = match (frame + i) % 5 { 0 => OccupancyCell::Occupied, 1 => OccupancyCell::Free, 2 => OccupancyCell::Unknown, 3 => OccupancyCell::Cost(56), _ => OccupancyCell::Cost(12) };
            }
            if frame == 30 { grid.origin.x += 0.07; }
            if frame == 60 { grid.resolution = 0.08; }
            let stats = incremental.replace_occupancy(grid.clone(), 0.15).unwrap();
            assert_eq!(stats.full_rebuild, frame == 30 || frame == 60);
            let rebuilt = Costmap::from_occupancy(grid.clone(), 0.15).unwrap();
            assert_eq!(incremental.heights(), rebuilt.heights());
            assert_eq!(incremental.blocked_cells(), rebuilt.blocked_cells());
            for i in 0..100 {
                let x = ((i * 13 + frame) % grid.width as usize) as i64;
                let y = ((i * 7 + frame) % grid.height as usize) as i64;
                assert_eq!(incremental.is_blocked(x, y), rebuilt.is_blocked(x, y));
            }
            assert_ne!(previous.occupancy_grid().unwrap().cells, grid.cells);
        }
        let stats = incremental.replace_occupancy(grid.clone(), 0.15).unwrap();
        assert_eq!((stats.changed_cells, stats.changed_blocks, stats.obstacle_changes), (0, 0, 0));
        let before = incremental.clone();
        grid.cells.pop();
        assert!(incremental.replace_occupancy(grid, 0.15).is_err());
        assert_eq!(before.heights(), incremental.heights());
        assert_eq!(before.occupancy_grid().unwrap().cells, incremental.occupancy_grid().unwrap().cells);
    }
    #[test]
    fn changed_block_contains_both_sides_of_a_removed_door() {
        let mut grid = OccupancyGrid2D::new(Point2::new(0.0, 0.0), 0.1, 64, 64, vec![OccupancyCell::Free; 4096]);
        for y in 0..64 { grid.set(31, y, OccupancyCell::Occupied); }
        let mut map = Costmap::from_occupancy(grid.clone(), 0.2).unwrap();
        map.rebuild_blocked();
        assert!(map.is_blocked(32, 31));
        for y in 28..36 { grid.set(31, y, OccupancyCell::Free); }
        let stats = map.replace_occupancy(grid, 0.2).unwrap();
        assert_eq!(stats.changed_blocks, 2);
        assert_eq!(stats.obstacle_changes, 8);
        assert!(!map.is_blocked(32, 31));
        assert!(!map.blocked_cells().contains(&(32, 31)));
    }
}

#[cfg(test)]
mod occupancy_update_measurements {
    use super::*;
    #[test]
    #[ignore = "固定规模派生基准，按任务显式运行"]
    fn fixed_local_change_retains_large_map_obstacle_index() {
        for side in [256, 1024, 2048] {
            let mut grid = OccupancyGrid2D::new(Point2::new(0.0, 0.0), 0.05, side, side, vec![OccupancyCell::Free; side as usize * side as usize]);
            for (i, cell) in grid.cells.iter_mut().enumerate() { if i % 10 == 0 { *cell = OccupancyCell::Occupied; } }
            let mut cache = Costmap::from_occupancy(grid.clone(), 0.15).unwrap();
            let mut full_times = Vec::new(); let mut local_times = Vec::new();
            for frame in 0..11 {
                for y in 40..45 { for x in 40..45 { grid.set(x, y, if frame % 2 == 0 { OccupancyCell::Occupied } else { OccupancyCell::Free }); } }
                let input = grid.clone(); let start = std::time::Instant::now();
                let full = Costmap::from_occupancy(input, 0.15).unwrap(); full_times.push(start.elapsed().as_micros());
                let input = grid.clone(); let start = std::time::Instant::now();
                let stats = cache.replace_occupancy(input, 0.15).unwrap(); local_times.push(start.elapsed().as_micros());
                assert_eq!(cache.heights(), full.heights()); assert_eq!(stats.changed_blocks, 1); assert!(!stats.full_rebuild);
            }
            full_times.sort_unstable(); local_times.sort_unstable();
            println!("cells={} full_build_us={} incremental_build_us={}", grid.cells.len(), full_times[5], local_times[5]);
        }
    }
}
