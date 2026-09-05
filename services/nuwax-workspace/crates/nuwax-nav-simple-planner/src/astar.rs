//! 8 邻接 octile heuristic A* 实现。

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// A* 状态节点（最小堆序：f 升序；f 相同时按 counter 升序保证确定性）。
#[derive(Debug)]
struct State {
    /// f = g + h
    f: f64,
    /// g 累计 cell cost
    g: f64,
    /// 节点坐标
    cell: (i64, i64),
    /// 插入顺序号（保证 tie-break 稳定）
    counter: u64,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f && self.counter == other.counter
    }
}
impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // 反向：BinaryHeap 是 max-heap，因此 a.cmp(b) < 0 表示 a 优先级高
        other
            .f
            .partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// 8 邻接偏移 + 几何步长 cost。
const NEIGHBOURS: [(i64, i64, f64); 8] = [
    (-1, -1, std::f64::consts::SQRT_2),
    (-1, 0, 1.0),
    (-1, 1, std::f64::consts::SQRT_2),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, -1, std::f64::consts::SQRT_2),
    (1, 0, 1.0),
    (1, 1, std::f64::consts::SQRT_2),
];

/// octile 启发。
#[inline]
#[must_use]
pub fn heuristic(cell: (i64, i64), goal: (i64, i64)) -> f64 {
    let dx = (cell.0 - goal.0).unsigned_abs() as f64;
    let dy = (cell.1 - goal.1).unsigned_abs() as f64;
    dx + dy + (std::f64::consts::SQRT_2 - 2.0) * dx.min(dy)
}

/// 在 8 邻接栅格上做 A*。
///
/// - `is_blocked` 给定 cell 是否被阻挡；`start` 与 `goal` 即使被阻挡仍允许通行
///   （由 caller 决定，本函数不做特例）。
/// - `max_expansions` 控制扩展上限；超出返回 `None`。
#[must_use]
pub fn astar<F>(
    start: (i64, i64),
    goal: (i64, i64),
    is_blocked: F,
    max_expansions: usize,
) -> Option<Vec<(i64, i64)>>
where
    F: Fn(i64, i64) -> bool,
{
    if start == goal {
        return Some(vec![start]);
    }

    let mut heap: BinaryHeap<State> = BinaryHeap::new();
    let mut g_score: HashMap<(i64, i64), f64> = HashMap::new();
    let mut came_from: HashMap<(i64, i64), (i64, i64)> = HashMap::new();
    let mut counter: u64 = 0;

    g_score.insert(start, 0.0);
    heap.push(State {
        f: heuristic(start, goal),
        g: 0.0,
        cell: start,
        counter,
    });

    let mut expansions: usize = 0;
    while let Some(state) = heap.pop() {
        expansions += 1;
        if expansions > max_expansions {
            return None;
        }
        let current = state.cell;
        if current == goal {
            let mut path = vec![current];
            let mut c = current;
            while let Some(&prev) = came_from.get(&c) {
                path.push(prev);
                c = prev;
            }
            path.reverse();
            return Some(path);
        }

        // 若堆中已有更优 g_score，跳过
        let cur_g = match g_score.get(&current) {
            Some(&g) if (g - state.g).abs() < 1e-9 => g,
            Some(&g) if g < state.g => g,
            _ => continue,
        };

        for (dx, dy, step) in NEIGHBOURS {
            let nb = (current.0 + dx, current.1 + dy);
            if is_blocked(nb.0, nb.1) {
                continue;
            }
            // 对角线两侧必须可通行，不能从接触的障碍格之间穿过。
            if dx != 0
                && dy != 0
                && (is_blocked(current.0 + dx, current.1) || is_blocked(current.0, current.1 + dy))
            {
                continue;
            }
            let tentative = cur_g + step;
            let existing = g_score.get(&nb).copied().unwrap_or(f64::INFINITY);
            if tentative < existing {
                came_from.insert(nb, current);
                g_score.insert(nb, tentative);
                counter += 1;
                heap.push(State {
                    f: tentative + heuristic(nb, goal),
                    g: tentative,
                    cell: nb,
                    counter,
                });
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagonal_cannot_escape_through_blocked_corners() {
        let blocked = |x, y| !matches!((x, y), (0, 0) | (1, 1));
        assert!(astar((0, 0), (1, 1), blocked, 100).is_none());
    }
}
