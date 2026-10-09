//! Analytic diagnostic world: no navigation imports or inferred support surfaces.
pub type Body = ([f64; 3], [f64; 4]);
#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub a: f64,
    pub b: f64,
    pub c: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct Solid {
    pub low: [f64; 3],
    pub high: [f64; 3],
}
#[derive(Clone, Debug)]
pub struct World {
    pub plane: Plane,
    pub solids: Vec<Solid>,
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let t = cross([q[0], q[1], q[2]], v).map(|v| 2.0 * v);
    let u = cross([q[0], q[1], q[2]], t);
    std::array::from_fn(|i| v[i] + q[3] * t[i] + u[i])
}
pub fn heading(body: Body) -> f64 {
    let [x, y, z, w] = body.1;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}
fn quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();
    [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]
}
impl Plane {
    pub fn height(self, xy: [f64; 2]) -> f64 {
        self.a * xy[0] + self.b * xy[1] + self.c
    }
    pub fn normal(self) -> [f64; 3] {
        let d = (1.0 + self.a * self.a + self.b * self.b).sqrt();
        [-self.a / d, -self.b / d, 1.0 / d]
    }
    pub fn body(self, xy: [f64; 2], yaw: f64, d: f64) -> Body {
        let pitch = -(self.a * yaw.cos() + self.b * yaw.sin()).atan();
        let roll = ((-self.a * yaw.sin() + self.b * yaw.cos()) * pitch.cos()).atan();
        (
            [xy[0], xy[1], self.height(xy) + d / self.normal()[2]],
            quaternion(roll, pitch, yaw),
        )
    }
    pub fn contact_error(self, body: Body, d: f64) -> f64 {
        let support = rotate(body.1, [0.0, 0.0, -d]);
        let p = std::array::from_fn(|i| body.0[i] + support[i]);
        (dot(self.normal(), p) - self.c * self.normal()[2]).abs()
    }
}
/// Exact planar SE(2) integration for a held body-frame twist, followed by
/// stance on the independently defined plane. Does not consult the point cloud.
pub fn advance(world: &World, from: Body, v: [f64; 3], dt: f64, d: f64) -> Body {
    let angle = v[2] * dt;
    let (a, b) = if v[2].abs() < 1e-12 {
        (dt, 0.0)
    } else {
        (angle.sin() / v[2], 2.0 * (angle / 2.0).sin().powi(2) / v[2])
    };
    let dx = a * v[0] - b * v[1];
    let dy = b * v[0] + a * v[1];
    let yaw = heading(from);
    world.plane.body(
        [
            from.0[0] + yaw.cos() * dx - yaw.sin() * dy,
            from.0[1] + yaw.sin() * dx + yaw.cos() * dy,
        ],
        yaw + angle,
        d,
    )
}
pub fn actuate(old: [f64; 3], command: [f64; 3], dt: f64) -> ([f64; 3], [f64; 3]) {
    let dx = command[0] - old[0];
    let dy = command[1] - old[1];
    let s = (dt / dx.hypot(dy).max(1e-12)).min(1.0);
    let next = [
        old[0] + s * dx,
        old[1] + s * dy,
        old[2] + (command[2] - old[2]).clamp(-2.0 * dt, 2.0 * dt),
    ];
    (next, std::array::from_fn(|i| (old[i] + next[i]) * 0.5))
}
fn vertices(body: Body, footprint: &[[f64; 2]], z: [f64; 2]) -> Vec<[f64; 3]> {
    z.into_iter()
        .flat_map(|z| {
            footprint.iter().map(move |p| {
                let r = rotate(body.1, [p[0], p[1], z]);
                std::array::from_fn(|i| body.0[i] + r[i])
            })
        })
        .collect()
}
/// Full convex-prism / axis-aligned-box separating-axis test. The expansion is
/// a conservative sweep enclosure, not clearance subtracted from an obstacle.
fn intersects(
    body: Body,
    footprint: &[[f64; 2]],
    z: [f64; 2],
    solid: Solid,
    expansion: f64,
) -> bool {
    let points = vertices(body, footprint, z);
    let axes_world = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let vertical = rotate(body.1, [0., 0., 1.]);
    let mut axes = axes_world.to_vec();
    axes.push(vertical);
    for edge in 0..footprint.len() {
        let a = footprint[edge];
        let b = footprint[(edge + 1) % footprint.len()];
        let e = rotate(body.1, [b[0] - a[0], b[1] - a[1], 0.]);
        axes.push(cross(e, vertical));
        for w in axes_world {
            axes.push(cross(e, w));
        }
    }
    for w in axes_world {
        axes.push(cross(vertical, w));
    }
    for axis in axes {
        if dot(axis, axis) < 1e-24 {
            continue;
        }
        let lo = points
            .iter()
            .map(|p| dot(*p, axis))
            .fold(f64::INFINITY, f64::min);
        let hi = points
            .iter()
            .map(|p| dot(*p, axis))
            .fold(f64::NEG_INFINITY, f64::max);
        let centre = std::array::from_fn(|i| (solid.low[i] + solid.high[i]) * 0.5);
        let radius = (0..3)
            .map(|i| axis[i].abs() * ((solid.high[i] - solid.low[i]) * 0.5 + expansion))
            .sum::<f64>();
        let mid = dot(centre, axis);
        if hi < mid - radius || lo > mid + radius {
            return false;
        }
    }
    true
}
impl World {
    pub fn check_motion(
        &self,
        from: Body,
        v: [f64; 3],
        dt: f64,
        d: f64,
        footprint: &[[f64; 2]],
        z: [f64; 2],
    ) -> Result<usize, String> {
        let radius = footprint
            .iter()
            .map(|p| p[0].hypot(p[1]).hypot(z[0].abs().max(z[1].abs())))
            .fold(0.0, f64::max);
        let nz = self.plane.normal()[2];
        // |d centre/dt| <= |vxy|/nz; yaw-to-plane-frame rotation rate
        // <= |omega|/nz^2. This bounds every vertex between sample poses.
        let speed = v[0].hypot(v[1]) / nz + radius * v[2].abs() / (nz * nz);
        let n = (speed * dt / 0.002).ceil().max(1.0) as usize;
        let margin = speed * dt / (2.0 * n as f64);
        for i in 0..=n {
            let pose = advance(self, from, v, dt * i as f64 / n as f64, d);
            if self.plane.contact_error(pose, d) > 1e-9 {
                return Err("analytic_contact_constraint".into());
            }
            for (j, solid) in self.solids.iter().enumerate() {
                if intersects(pose, footprint, z, *solid, margin) {
                    return Err(format!("independent_swept_solid_{j}"));
                }
            }
        }
        Ok(n + 1)
    }
    /// Quantize the predefined world independently for navigation observations.
    /// Ground top faces use nearest 5cm level (<=2.5cm vertical error).
    /// Box observations contain all intersecting source cells.
    pub fn observations(&self) -> Vec<(f32, f32, f32)> {
        let q = f64::from(0.05_f32);
        let mut cells = std::collections::BTreeSet::new();
        for ix in -40..180 {
            for iy in -60..60 {
                let xy = [(ix as f64 + 0.5) * q, (iy as f64 + 0.5) * q];
                cells.insert([ix, iy, (self.plane.height(xy) / q).round() as i32 - 1]);
            }
        }
        for b in &self.solids {
            let low = b.low.map(|v| (v / q).floor() as i32);
            let high = b.high.map(|v| (v / q).ceil() as i32);
            for x in low[0]..high[0] {
                for y in low[1]..high[1] {
                    for z in low[2]..high[2] {
                        cells.insert([x, y, z]);
                    }
                }
            }
        }
        cells
            .into_iter()
            .map(|p| {
                let p = p.map(|v| ((v as f64 + 0.5) * q) as f32);
                (p[0], p[1], p[2])
            })
            .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn world() -> World {
        World {
            plane: Plane {
                a: 0.15,
                b: -0.08,
                c: 0.12,
            },
            solids: vec![],
        }
    }
    const FOOT: [[f64; 2]; 4] = [[-0.3, -0.15], [0.3, -0.15], [0.3, 0.15], [-0.3, 0.15]];
    #[test]
    fn plane_geometry_at_multiple_headings() {
        for plane in [
            Plane {
                a: 0.,
                b: 0.,
                c: 0.,
            },
            world().plane,
        ] {
            for yaw in [-2.6, -0.8, 0., 1.2, 3.0] {
                let p = plane.body([0.37, -0.21], yaw, 0.4);
                let n = rotate(p.1, [0., 0., 1.]);
                assert!(n
                    .iter()
                    .zip(plane.normal())
                    .all(|(a, b)| (a - b).abs() < 1e-12));
                assert!(plane.contact_error(p, 0.4) < 1e-12);
                assert!(
                    (p.0[2]
                        - plane.height([p.0[0], p.0[1]])
                        - 0.4 * (1. + plane.a.powi(2) + plane.b.powi(2)).sqrt())
                    .abs()
                        < 1e-12
                );
                assert!((heading(p) - yaw).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn planar_arc_matches_closed_form_and_subdivision() {
        let w = world();
        let start = w.plane.body([0., 0.], 0., 0.4);
        let v = [0.5, 0., 0.4];
        let end = advance(&w, start, v, 2., 0.4);
        assert!((end.0[0] - 1.25 * 0.8_f64.sin()).abs() < 1e-12);
        assert!((end.0[1] - 1.25 * (1. - 0.8_f64.cos())).abs() < 1e-12);
        let mut p = start;
        for _ in 0..40 {
            p = advance(&w, p, v, 0.05, 0.4);
        }
        assert!(p.0.iter().zip(end.0).all(|(a, b)| (a - b).abs() < 1e-11));
        for _ in 0..40 {
            p = advance(&w, p, v.map(|x| -x), 0.05, 0.4);
        }
        assert!(p.0.iter().zip(start.0).all(|(a, b)| (a - b).abs() < 1e-11));
    }
    #[test]
    fn rotation_preserves_height_and_world_normal() {
        let w = world();
        let p = w.plane.body([1., 2.], 0.6, 0.4);
        let q = advance(&w, p, [0., 0., 0.5], 2., 0.4);
        assert_eq!(p.0, q.0);
        assert!(w.plane.contact_error(q, 0.4) < 1e-12);
    }
    #[test]
    fn acceleration_limits_and_stopping() {
        let mut v = [0.; 3];
        for _ in 0..10 {
            v = actuate(v, [0.5, 0., 0.8], 0.05).0;
        }
        assert!((v[0] - 0.5).abs() < 1e-12);
        for _ in 0..10 {
            v = actuate(v, [0.; 3], 0.05).0;
        }
        assert!(v.iter().all(|x| x.abs() < 1e-12));
    }
    #[test]
    fn independent_collision_detects_sweep_between_clear_endpoints() {
        let w = World {
            plane: Plane {
                a: 0.,
                b: 0.,
                c: 0.,
            },
            solids: vec![Solid {
                low: [0.48, -0.1, 0.2],
                high: [0.52, 0.1, 0.6],
            }],
        };
        let p = w.plane.body([0., 0.], 0., 0.4);
        assert!(!intersects(p, &FOOT, [-0.1, 0.1], w.solids[0], 0.));
        assert!(!intersects(
            w.plane.body([1., 0.], 0., 0.4),
            &FOOT,
            [-0.1, 0.1],
            w.solids[0],
            0.
        ));
        assert!(w
            .check_motion(p, [1., 0., 0.], 1., 0.4, &FOOT, [-0.1, 0.1])
            .is_err());
        assert!(w
            .check_motion(
                w.plane.body([0., 1.], 0., 0.4),
                [1., 0., 0.],
                1.,
                0.4,
                &FOOT,
                [-0.1, 0.1]
            )
            .is_ok());
    }
    #[test]
    fn step_platforms_and_riser_have_known_geometry_only() {
        let low = Plane {
            a: 0.,
            b: 0.,
            c: 0.,
        };
        let high = Plane { c: 0.1, ..low };
        let step = Solid {
            low: [0., -1., 0.],
            high: [2., 1., 0.1],
        };
        assert_eq!(low.body([-1., 0.], 0., 0.4).0[2], 0.4);
        assert_eq!(high.body([1., 0.], 0., 0.4).0[2], 0.5);
        assert!(intersects(
            ([-0.05, 0., 0.05], [0., 0., 0., 1.]),
            &FOOT,
            [-0.02, 0.02],
            step,
            0.
        ));
        assert!(!intersects(
            high.body([1., 0.], 0., 0.4),
            &FOOT,
            [-0.1, 0.1],
            step,
            0.
        ));
        // No gait/transition model: no cross-step motion or arrival assertion.
    }
    #[test]
    fn observation_quantization_does_not_move_world() {
        let w = world();
        let p = w.plane.body([0.37, 0.11], 1., 0.4);
        for (x, y, z) in w.observations() {
            let q = f64::from(0.05_f32);
            assert!(
                (f64::from(z) + q / 2. - w.plane.height([x as f64, y as f64])).abs()
                    < q / 2. + 1e-6
            );
        }
        assert_eq!(p, w.plane.body([0.37, 0.11], 1., 0.4));
    }
}
