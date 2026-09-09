//! Costmap：栅格化带历史最大高度的障碍物表示。
//!
//! 仅当新点高度大于该 cell 已有最大高度时覆盖；阻塞缓存由脏标记触发重建。

use std::collections::{HashMap, HashSet};

use crate::inflation::{blocked_at_inflation, inflate_obstacles};
use crate::NavError;
use nuwax_nav_types_messages::grid::OCCUPANCY_OBSTACLE_THRESHOLD;
use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D, Point2};

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
