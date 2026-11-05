
use nalgebra::Point3;
pub(crate) const ORIGIN: [f64; 3] = [0.03, 0.03, 0.65];

/// 独立解析射线与地面、墙面、直立圆柱求交，返回每个回波的真实动态标签。
pub(crate) fn corridor(person_y: Option<f64>, phase: f64) -> (Vec<Point3<f64>>, Vec<bool>) {
    let mut points = Vec::new();
    let mut labels = Vec::new();
    for row in 0..61 {
        let elevation = -0.55 + row as f64 * 0.017 + phase * 0.002;
        for col in 0..141 {
            let azimuth = -0.7 + col as f64 * 0.01 + phase * 0.002;
            let direction = [
                elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(),
                elevation.sin(),
            ];
            let mut distance = (8.0 - ORIGIN[0]) / direction[0];
            if direction[2] < 0.0 {
                distance = distance.min(-ORIGIN[2] / direction[2]);
            }
            if direction[2] > 0.0 {
                distance = distance.min((3.0 - ORIGIN[2]) / direction[2]);
            }
            if direction[1] != 0.0 {
                let side = if direction[1] > 0.0 { 3.0 } else { -3.0 };
                distance = distance.min((side - ORIGIN[1]) / direction[1]);
            }
            let mut dynamic = false;
            if let Some(y) = person_y {
                let dx = ORIGIN[0] - 3.0;
                let dy = ORIGIN[1] - y;
                let a = direction[0].powi(2) + direction[1].powi(2);
                let b = 2.0 * (dx * direction[0] + dy * direction[1]);
                let c = dx * dx + dy * dy - 0.25_f64.powi(2);
                let discriminant = b * b - 4.0 * a * c;
                if discriminant >= 0.0 {
                    let hit = (-b - discriminant.sqrt()) / (2.0 * a);
                    let z = ORIGIN[2] + hit * direction[2];
                    if hit > 0.0 && hit < distance && (0.12..=1.8).contains(&z) {
                        distance = hit;
                        dynamic = true;
                    }
                }
            }
            points.push(Point3::from(std::array::from_fn::<_, 3, _>(|i| {
                ORIGIN[i] + distance * direction[i]
            })));
            labels.push(dynamic);
        }
    }
    (points, labels)
}
