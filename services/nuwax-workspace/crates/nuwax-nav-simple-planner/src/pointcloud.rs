//! 点云分类：把 z-相对地面 > 0 的点写入 costmap（pure function）。

use crate::costmap::Costmap;
use crate::Point3;

/// 把 `points` 中 z-相对地面为正的点写入 costmap。
///
/// 返回 costmap 是否被修改（dirty）。
#[must_use]
pub fn classify_points(points: &[Point3], costmap: &mut Costmap, ground_z: f64) -> bool {
    let mut best_per_cell: std::collections::HashMap<(i64, i64), f64> =
        std::collections::HashMap::new();
    for p in points {
        let rel_z = p.z - ground_z;
        if rel_z <= 0.0 {
            continue;
        }
        let key = costmap.world_to_cell(p.x, p.y);
        let entry = best_per_cell.entry(key).or_insert(f64::NEG_INFINITY);
        if rel_z > *entry {
            *entry = rel_z;
        }
    }
    let mut dirty = false;
    for (k, h) in best_per_cell {
        let prev = costmap
            .heights()
            .get(&k)
            .copied()
            .unwrap_or(f64::NEG_INFINITY);
        if h > prev {
            dirty = true;
            let (x, y) = costmap.cell_to_world(k.0, k.1);
            costmap.update(x, y, h);
        }
    }
    dirty
}
