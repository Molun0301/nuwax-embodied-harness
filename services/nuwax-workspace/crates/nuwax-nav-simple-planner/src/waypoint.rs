//! lookahead：沿 cached path 找出至少 distance 米远的前导点。

use crate::Point2;

/// 沿 `path` 从机器人当前位置往前找到第一个距离不小于 `distance` 米的点。
///
/// - 空路径：返回 `(rx, ry)`。
/// - 路径不足以达到距离：返回最后一个点。
#[must_use]
pub fn lookahead(path: &[Point2], rx: f64, ry: f64, distance: f64) -> (f64, f64) {
    if path.is_empty() {
        return (rx, ry);
    }
    // 找最接近 robot 的 path index
    let mut best_idx = 0usize;
    let mut best_d2 = f64::INFINITY;
    for (i, p) in path.iter().enumerate() {
        let d2 = (p.x - rx).powi(2) + (p.y - ry).powi(2);
        if d2 < best_d2 {
            best_d2 = d2;
            best_idx = i;
        }
    }
    let d2_target = distance * distance;
    for p in &path[best_idx..] {
        if (p.x - rx).powi(2) + (p.y - ry).powi(2) >= d2_target {
            return (p.x, p.y);
        }
    }
    let last = path.last().expect("non-empty path checked above");
    (last.x, last.y)
}
