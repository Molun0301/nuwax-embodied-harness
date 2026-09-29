//! 均匀三次B样条：控制点优化与固定起终点边界，不在显示端伪造平滑。
use nuwax_nav_pct::Tomogram;
pub type Point = [f64; 3];
fn length(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
pub fn resample(points: &[Point], spacing: f64) -> Vec<Point> {
    let mut result = vec![points[0]];
    for pair in points.windows(2) {
        let n = (length(pair[0], pair[1]) / spacing).ceil().max(1.0) as usize;
        for i in 1..=n {
            let t = i as f64 / n as f64;
            result.push(std::array::from_fn(|j| {
                pair[0][j] * (1.0 - t) + pair[1][j] * t
            }));
        }
    }
    result
}
fn clearance(map: &Tomogram, p: Point) -> f64 {
    map.cell_at(p).map_or(0.0, |id| map.cells[id].clearance)
}
/// 近场固定迭代数；位置参考、二阶平滑及障碍距离共同优化。
pub fn optimize(
    reference: &[Point],
    map: &Tomogram,
    radius: f64,
    centers: [f64; 2],
    iterations: usize,
    initial_heading: Option<f64>,
) -> Vec<Point> {
    let mut controls = resample(reference, 0.25_f64.max(map.resolution));
    if controls.len() < 3 {
        return controls;
    }
    if let Some(heading) = initial_heading {
        let lead = length(controls[0], controls[1]).clamp(0.1, 0.3);
        controls[1] = [
            controls[0][0] + lead * heading.cos(),
            controls[0][1] + lead * heading.sin(),
            controls[0][2],
        ];
    }
    let original = controls.clone();
    let h = map.resolution;
    for _ in 0..iterations {
        let previous = controls.clone();
        for i in (if initial_heading.is_some() { 2 } else { 1 })..controls.len() - 1 {
            let p = previous[i];
            let heading = (previous[i + 1][1] - previous[i - 1][1])
                .atan2(previous[i + 1][0] - previous[i - 1][0]);
            // 与机体偏航一起转动的前后双圆柱，避免长机体仅检查中心圆。
            let body_clearance = |point: Point| {
                centers
                    .iter()
                    .map(|&offset| {
                        clearance(
                            map,
                            [
                                point[0] + offset * heading.cos(),
                                point[1] + offset * heading.sin(),
                                point[2],
                            ],
                        )
                        .min(radius + map.resolution * 3.0)
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            let c = body_clearance(p);
            let mut next = p;
            for axis in 0..2 {
                let mut a = p;
                let mut b = p;
                a[axis] += h;
                b[axis] -= h;
                let gradient = (body_clearance(a) - body_clearance(b)) / (2.0 * h);
                let smooth = (previous[i - 1][axis] + previous[i + 1][axis] - 2.0 * p[axis]) * 0.18;
                let reference = (original[i][axis] - p[axis]) * 0.12;
                let repulsion = (radius + map.resolution - c).max(0.0) * gradient * 0.2;
                next[axis] += (smooth + reference + repulsion).clamp(-h * 0.35, h * 0.35);
            }
            if let Some(id) = map.cell_at(next) {
                next[2] = map.cells[id].ground;
                if map.segment_clear(previous[i - 1], next, &[])
                    && map.segment_clear(next, previous[i + 1], &[])
                {
                    controls[i] = next;
                }
            }
        }
    }
    controls
}
/// 重复端控制点使曲线从实际起点出发、终止于局部目标。
pub fn sample(controls: &[Point], spacing: f64) -> Vec<Point> {
    if controls.len() < 3 {
        return resample(controls, spacing);
    }
    let mut p = vec![controls[0]; 2];
    p.extend_from_slice(controls);
    p.extend_from_slice(&[*controls.last().unwrap(); 2]);
    let mut result = Vec::new();
    for window in p.windows(4) {
        let n = ((length(window[0], window[1])
            + length(window[1], window[2])
            + length(window[2], window[3]))
            / spacing)
            .ceil()
            .max(2.0) as usize;
        for i in 0..n {
            let t = i as f64 / n as f64;
            let u = 1.0 - t;
            let weights = [
                u * u * u / 6.0,
                (3.0 * t * t * t - 6.0 * t * t + 4.0) / 6.0,
                (-3.0 * t * t * t + 3.0 * t * t + 3.0 * t + 1.0) / 6.0,
                t * t * t / 6.0,
            ];
            let value =
                std::array::from_fn(|axis| (0..4).map(|j| weights[j] * window[j][axis]).sum());
            if result
                .last()
                .is_none_or(|&last| length(last, value) > spacing * 0.2)
            {
                result.push(value);
            }
        }
    }
    result.push(*controls.last().unwrap());
    result
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn open_ground_without_known_walls_keeps_finite_control_points() {
        let map = Tomogram::from_columns(
            0.1,
            (-20..21).flat_map(|x| (-20..21).map(move |y| ([x, y], vec![[-0.1, 0.0]]))),
            0.3,
            0.15,
            0.1,
            0.1,
            20.0,
        )
        .unwrap();
        assert!(map.cells.iter().all(|cell| cell.clearance.is_infinite()));
        let controls = optimize(
            &[[0.0, 0.0, 0.0], [0.5, 0.3, 0.0], [1.0, 0.0, 0.0]],
            &map,
            0.15,
            [-0.15, 0.15],
            12,
            Some(0.0),
        );
        assert!(controls.len() > 3);
        assert!(controls.iter().flatten().all(|value| value.is_finite()));
        assert!(sample(&controls, 0.025).len() > 20);
    }
    #[test]
    fn cubic_curve_preserves_endpoints_and_rounds_corner() {
        let p = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
        let points = sample(&p, 0.025);
        assert_eq!(points[0], p[0]);
        assert_eq!(*points.last().unwrap(), p[2]);
        assert!(points.iter().all(|v| v[0] >= -1e-12
            && v[0] <= 1.0 + 1e-12
            && v[1] >= -1e-12
            && v[1] <= 1.0 + 1e-12));
        assert!(points
            .iter()
            .any(|v| v[0] > 0.6 && v[0] < 1.0 && v[1] > 0.0 && v[1] < 0.4));
    }
}
