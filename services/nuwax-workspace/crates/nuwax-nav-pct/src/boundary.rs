//! 通用平面多边形数学；层析核心不通过表面规划器取得边界判定。
/// 精确核对线段是否留在任务多边形内，供全局与近场共用。
pub fn segment_inside_boundary(from: [f64; 2], to: [f64; 2], boundary: &[[f64; 2]]) -> bool {
    if boundary.is_empty() {
        return true;
    }
    if !inside_boundary(from, boundary) || !inside_boundary(to, boundary) {
        return false;
    }
    let direction = [to[0] - from[0], to[1] - from[1]];
    let mut intervals = vec![0.0, 1.0];
    for index in 0..boundary.len() {
        let a = boundary[index];
        let b = boundary[(index + 1) % boundary.len()];
        let edge = [b[0] - a[0], b[1] - a[1]];
        let denominator = direction[0] * edge[1] - direction[1] * edge[0];
        if denominator.abs() <= 1e-12 {
            continue;
        }
        let offset = [a[0] - from[0], a[1] - from[1]];
        let along = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator;
        let across = (offset[0] * direction[1] - offset[1] * direction[0]) / denominator;
        if (0.0..=1.0).contains(&along) && (0.0..=1.0).contains(&across) {
            intervals.push(along);
        }
    }
    intervals.sort_by(f64::total_cmp);
    intervals.windows(2).all(|pair| {
        let midpoint = (pair[0] + pair[1]) * 0.5;
        inside_boundary(
            [
                from[0] + direction[0] * midpoint,
                from[1] + direction[1] * midpoint,
            ],
            boundary,
        )
    })
}

/// 空边界表示不限制；多边形边缘属于任务范围。
pub fn inside_boundary(point: [f64; 2], boundary: &[[f64; 2]]) -> bool {
    if boundary.is_empty() {
        return true;
    }
    let mut inside = false;
    for index in 0..boundary.len() {
        let a = boundary[index];
        let b = boundary[(index + 1) % boundary.len()];
        let cross = (point[0] - a[0]) * (b[1] - a[1]) - (point[1] - a[1]) * (b[0] - a[0]);
        if cross.abs() <= 1e-9
            && point[0] >= a[0].min(b[0]) - 1e-9
            && point[0] <= a[0].max(b[0]) + 1e-9
            && point[1] >= a[1].min(b[1]) - 1e-9
            && point[1] <= a[1].max(b[1]) + 1e-9
        {
            return true;
        }
        if (a[1] > point[1]) != (b[1] > point[1])
            && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }
    inside
}

