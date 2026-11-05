//! 在线去动态和窗口索引共用的体素坐标。

pub(crate) type Cell = (i32, i32, i32);
pub(crate) const CELL_SIZE: f64 = 0.1;

pub(crate) fn cell(point: [f64; 3]) -> Option<Cell> {
    let v = point.map(|x| (x / CELL_SIZE).floor());
    v.iter()
        .all(|x| x.is_finite() && *x > i32::MIN as f64 + 2. && *x < i32::MAX as f64 - 2.)
        .then(|| (v[0] as i32, v[1] as i32, v[2] as i32))
}

#[cfg(test)]
#[path = "dynamic_scene_tests.rs"]
pub(crate) mod tests;
