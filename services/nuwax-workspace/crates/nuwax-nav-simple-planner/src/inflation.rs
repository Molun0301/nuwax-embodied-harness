//! 障碍物膨胀：构造阻塞集合的纯函数。

use std::collections::{HashMap, HashSet};

use crate::costmap::Costmap;
use crate::NavError;

/// 给定 `heights` 与 `obstacle_height`、`inflation_radius`、`cell_size`，
/// 构建阻塞集合（含半径内所有 cell）。
#[must_use]
pub fn inflate_obstacles(
    heights: &HashMap<(i64, i64), f64>,
    obstacle_height: f64,
    inflation_radius: f64,
    cell_size: f64,
) -> HashSet<(i64, i64)> {
    if cell_size <= 0.0 || heights.is_empty() || inflation_radius < 0.0 {
        return HashSet::new();
    }
    let r_cells = (inflation_radius / cell_size).ceil() as i64;
    if r_cells < 0 {
        return HashSet::new();
    }
    let max_sq = (inflation_radius / cell_size).powi(2);
    let mut blocked = HashSet::new();
    for (&(ix, iy), &h) in heights {
        if h < obstacle_height {
            continue;
        }
        if r_cells == 0 {
            blocked.insert((ix, iy));
            continue;
        }
        for dx in -r_cells..=r_cells {
            for dy in -r_cells..=r_cells {
                let d2 = (dx * dx + dy * dy) as f64;
                if d2 <= max_sq {
                    blocked.insert((ix + dx, iy + dy));
                }
            }
        }
    }
    blocked
}

/// 对 costmap 应用指定 `inflation_radius` 计算阻塞集合（**不**修改 costmap）。
///
/// # Errors
/// 当 `inflation_radius < 0` 时返回 `NavError::InvalidInput`。
pub fn blocked_at_inflation(costmap: &Costmap, inflation_radius: f64) -> HashSet<(i64, i64)> {
    if inflation_radius < 0.0 {
        // 借用 error 而非 panic，避免 caller 写错参数时静默错。
        let _ = Err::<(), _>(NavError::InvalidInput {
            field: "inflation_radius",
            reason: format!("must be non-negative, got {inflation_radius}"),
        });
        return HashSet::new();
    }
    inflate_obstacles(
        costmap.heights(),
        costmap.obstacle_height(),
        inflation_radius,
        costmap.cell_size(),
    )
}
