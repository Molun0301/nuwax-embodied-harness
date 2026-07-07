//! Costmap：栅格化带历史最大高度的障碍物表示。
//!
//! 仅当新点高度大于该 cell 已有最大高度时覆盖；阻塞缓存由脏标记触发重建。

use std::collections::{HashMap, HashSet};

use crate::inflation::{blocked_at_inflation, inflate_obstacles};
use crate::NavError;

/// Costmap：稀疏 + 历史最大高度模型。
#[derive(Debug, Clone)]
pub struct Costmap {
    cell_size: f64,
    obstacle_height: f64,
    inflation_radius: f64,
    heights: HashMap<(i64, i64), f64>,
    blocked: HashSet<(i64, i64)>,
    blocked_dirty: bool,
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
        }
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
        if radius < 0.0 {
            // 与构造保持一致：负数禁止，但本方法为 setter 仅记录 trace 并忽略。
            tracing::warn!(radius, "ignored negative inflation_radius in setter");
            return;
        }
        self.inflation_radius = radius;
        self.blocked_dirty = true;
    }

    /// 世界坐标 → cell 索引。
    #[must_use]
    pub fn world_to_cell(&self, x: f64, y: f64) -> (i64, i64) {
        (
            (x / self.cell_size).floor() as i64,
            (y / self.cell_size).floor() as i64,
        )
    }

    /// cell 索引 → 世界坐标（cell 中心）。
    #[must_use]
    pub fn cell_to_world(&self, ix: i64, iy: i64) -> (f64, f64) {
        (
            (ix as f64 + 0.5) * self.cell_size,
            (iy as f64 + 0.5) * self.cell_size,
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
    }

    /// 是否被阻塞（按当前默认 inflation_radius）。
    #[must_use]
    pub fn is_blocked(&self, ix: i64, iy: i64) -> bool {
        self.blocked_cells().contains(&(ix, iy))
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
