//! 凸柱与轴对齐占据盒的分离轴判定，独立于地图和导航实现。
use super::{rotation, BodyModel, BodyPose, Matrix3, V};

pub(crate) struct Prism {
    pub rotation: Matrix3<f64>,
    pub min: [f64; 3],
    pub max: [f64; 3],
    hull: Vec<[f64; 2]>,
    axes: Vec<(V, f64, f64)>,
}
impl Prism {
    pub fn new(model: &BodyModel, pose: BodyPose) -> Result<Self, String> {
        let rotation = rotation(pose.1)?;
        let center = V::from(pose.0);
        let vertices: Vec<V> = model
            .footprint
            .iter()
            .flat_map(|p| {
                model
                    .z_bounds
                    .map(|z| center + rotation * V::new(p[0], p[1], z))
            })
            .collect();
        let min =
            std::array::from_fn(|i| vertices.iter().map(|v| v[i]).fold(f64::INFINITY, f64::min));
        let max = std::array::from_fn(|i| {
            vertices
                .iter()
                .map(|v| v[i])
                .fold(f64::NEG_INFINITY, f64::max)
        });
        let world = [V::x(), V::y(), V::z()];
        let mut directions = world.to_vec();
        let up = rotation.column(2).into_owned();
        directions.push(up);
        for axis in world {
            directions.push(up.cross(&axis));
        }
        for i in 0..model.footprint.len() {
            let a = model.footprint[i];
            let b = model.footprint[(i + 1) % model.footprint.len()];
            let edge = rotation * V::new(b[0] - a[0], b[1] - a[1], 0.0);
            directions.push(edge.cross(&up));
            for axis in world {
                directions.push(edge.cross(&axis));
            }
        }
        let axes = directions
            .into_iter()
            .filter(|v| v.norm_squared() > 1e-24)
            .map(|v| {
                let v = v.normalize();
                let low = vertices
                    .iter()
                    .map(|p| v.dot(p))
                    .fold(f64::INFINITY, f64::min);
                let high = vertices
                    .iter()
                    .map(|p| v.dot(p))
                    .fold(f64::NEG_INFINITY, f64::max);
                (v, low, high)
            })
            .collect();
        let mut points: Vec<_> = vertices.iter().map(|v| [v.x, v.y]).collect();
        points.sort_by(|a, b| a[0].total_cmp(&b[0]).then_with(|| a[1].total_cmp(&b[1])));
        points.dedup();
        let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
            (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        };
        // 上下凸包分别构建，避免回程扫描弹掉已完成的下凸包。
        let half = |values: Vec<[f64; 2]>| {
            let mut out = Vec::new();
            for p in values {
                while out.len() >= 2 && cross(out[out.len() - 2], out[out.len() - 1], p) <= 0.0 {
                    out.pop();
                }
                out.push(p);
            }
            out
        };
        let mut lower = half(points.clone());
        let mut upper = half(points.into_iter().rev().collect());
        lower.pop();
        upper.pop();
        lower.extend(upper);
        Ok(Self {
            rotation,
            min,
            max,
            hull: lower,
            axes,
        })
    }
    pub fn intersects(&self, min: [f64; 3], max: [f64; 3], padding: f64) -> bool {
        let center = (V::from(min) + V::from(max)) * 0.5;
        let half = (V::from(max) - V::from(min)) * 0.5;
        self.axes.iter().all(|(axis, lo, hi)| {
            let value = axis.dot(&center);
            let span = axis.abs().dot(&half) + padding;
            value + span >= lo - 1e-10 && value - span <= hi + 1e-10
        })
    }
    pub fn column_overlap(&self, key: [i32; 2], size: f64) -> bool {
        let mut polygon = self.hull.clone();
        for (axis, bound, sign) in [
            (0, f64::from(key[0]) * size, 1.0),
            (0, f64::from(key[0] + 1) * size, -1.0),
            (1, f64::from(key[1]) * size, 1.0),
            (1, f64::from(key[1] + 1) * size, -1.0),
        ] {
            let mut next = Vec::new();
            for (&a, &b) in polygon.iter().zip(polygon.iter().cycle().skip(1)) {
                let da = (a[axis] - bound) * sign;
                let db = (b[axis] - bound) * sign;
                if da >= 0.0 {
                    next.push(a);
                }
                if (da >= 0.0) != (db >= 0.0) {
                    let t = da / (da - db);
                    next.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
                }
            }
            polygon = next;
            if polygon.len() < 3 {
                return false;
            }
        }
        polygon
            .iter()
            .zip(polygon.iter().cycle().skip(1))
            .map(|(a, b)| a[0] * b[1] - a[1] * b[0])
            .sum::<f64>()
            .abs()
            > 1e-14
    }
}
