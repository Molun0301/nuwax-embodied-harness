//! 原始体素局部几何试验：暴露带顶、凸柱机体与自适应扫掠，无MLS依赖。
mod shapes;
#[cfg(test)]
mod tests;
use ahash::AHashMap;
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
pub type BodyPose = ([f64; 3], [f64; 4]);
type V = Vector3<f64>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyModel {
    pub footprint: Vec<[f64; 2]>,
    pub z_bounds: [f64; 2],
    pub support_offset: f64,
    pub step_up: f64,
    pub step_down: f64,
    pub max_cross_slope_degrees: f64,
}
impl BodyModel {
    pub fn radius(&self) -> f64 {
        self.footprint
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f64::max)
    }
    fn validate(&self) -> Result<(), String> {
        if self.footprint.len() < 3
            || !self
                .footprint
                .iter()
                .flatten()
                .chain(self.z_bounds.iter())
                .chain([
                    &self.support_offset,
                    &self.step_up,
                    &self.step_down,
                    &self.max_cross_slope_degrees,
                ])
                .all(|v| v.is_finite())
            || self.z_bounds[0] >= self.z_bounds[1]
            || self.support_offset <= 0.0
            || self.step_up < 0.0
            || self.step_down < 0.0
        {
            return Err("原始体素机体参数无效".into());
        }
        if !(0.0..90.0).contains(&self.max_cross_slope_degrees) || self.radius() <= 0.0 {
            return Err("机体半径或横坡标定无效".into());
        }
        let area = self
            .footprint
            .iter()
            .zip(self.footprint.iter().cycle().skip(1))
            .map(|(a, b)| a[0] * b[1] - a[1] * b[0])
            .sum::<f64>();
        if area <= 1e-12 {
            return Err("机体足迹面积无效".into());
        }
        for i in 0..self.footprint.len() {
            let a = self.footprint[i];
            let b = self.footprint[(i + 1) % self.footprint.len()];
            let c = self.footprint[(i + 2) % self.footprint.len()];
            if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) < -1e-10 {
                return Err("机体足迹须为逆时针凸多边形".into());
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
struct Band {
    low: f64,
    top: f64,
}
pub struct VoxelMap {
    pub model: BodyModel,
    pub resolution: f64,
    columns: AHashMap<[i32; 2], Vec<Band>>,
    surface_measurements: bool,
}
fn rotation(q: [f64; 4]) -> Result<Matrix3<f64>, String> {
    if !q.into_iter().all(f64::is_finite) || q.iter().map(|v| v * v).sum::<f64>() < 1e-20 {
        return Err("机体朝向无效".into());
    }
    Ok(
        UnitQuaternion::new_normalize(Quaternion::new(q[3], q[0], q[1], q[2]))
            .to_rotation_matrix()
            .into_inner(),
    )
}
fn quaternion(r: Matrix3<f64>) -> [f64; 4] {
    let q = UnitQuaternion::from_matrix(&r).into_inner();
    [q.i, q.j, q.k, q.w]
}
fn yaw(p: BodyPose) -> f64 {
    (2.0 * (p.1[3] * p.1[2] + p.1[0] * p.1[1]))
        .atan2(1.0 - 2.0 * (p.1[1] * p.1[1] + p.1[2] * p.1[2]))
}
fn angle(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (V::from(a) - V::from(b)).norm()
}

impl VoxelMap {
    pub fn from_points(
        points: &[(f32, f32, f32)],
        resolution: f32,
        model: BodyModel,
    ) -> Result<Self, String> {
        model.validate()?;
        let q = f64::from(resolution);
        if !q.is_finite() || q <= 0.0 {
            return Err("源体素精度无效".into());
        }
        let mut columns = AHashMap::<[i32; 2], Vec<i32>>::new();
        for &(x, y, z) in points {
            let p = [x, y, z].map(f64::from);
            if !p
                .into_iter()
                .all(|v| v.is_finite() && (v / q).abs() < f64::from(i32::MAX / 2))
            {
                return Err("点云坐标无效".into());
            }
            let p = p.map(|v| (v / q).floor() as i32);
            columns.entry([p[0], p[1]]).or_default().push(p[2]);
        }
        let columns = columns
            .into_iter()
            .map(|(key, mut levels)| {
                levels.sort_unstable();
                levels.dedup();
                let mut bands: Vec<Band> = Vec::new();
                for z in levels {
                    let low = f64::from(z) * q;
                    let top = f64::from(z + 1) * q;
                    if let Some(b) = bands.last_mut().filter(|b| (b.top - low).abs() < q * 1e-6) {
                        b.top = top;
                    } else {
                        bands.push(Band { low, top });
                    }
                }
                (key, bands)
            })
            .collect();
        Ok(Self {
            model,
            resolution: q,
            columns,
            surface_measurements: false,
        })
    }
    /// 点云测量带允许带内接触拟合；实体占据输入保持默认暴露顶面，二者不能混同。
    pub fn with_surface_measurements(mut self) -> Self {
        self.surface_measurements = true;
        self
    }

    fn ground(&self, key: [i32; 2], reference: f64) -> Option<Band> {
        self.columns
            .get(&key)?
            .iter()
            .copied()
            .filter(|b| {
                b.top - reference <= self.model.step_up + self.resolution + 1e-9
                    && reference - b.top <= self.model.step_down + self.resolution + 1e-9
            })
            .min_by(|a, b| {
                (a.top - reference)
                    .abs()
                    .total_cmp(&(b.top - reference).abs())
                    .then_with(|| a.top.total_cmp(&b.top))
            })
    }
    /// 接触响应取邻近主表面，腿部通行则允许同列达到声明阶高的暴露踏面。
    /// 两者不能混成“最近带以上都是障碍”，否则小台阶会与下探足端包络相撞。
    fn support_ground(&self, key: [i32; 2], reference: f64) -> Option<Band> {
        self.columns
            .get(&key)?
            .iter()
            .copied()
            .filter(|b| {
                b.top - reference <= self.model.step_up + self.resolution + 1e-9
                    && reference - b.top <= self.model.step_down + self.resolution + 1e-9
            })
            .max_by(|a, b| a.top.total_cmp(&b.top))
    }

    /// 暴露带顶决定接触面；实心阶体内部不作为可选接触区间。
    pub fn ground_frame(&self, xy: [f64; 2], reference: f64) -> Result<(f64, [f64; 3]), String> {
        if !reference.is_finite()
            || !xy
                .into_iter()
                .all(|v| v.is_finite() && (v / self.resolution).abs() < f64::from(i32::MAX / 4))
        {
            return Err("接触查询坐标无效".into());
        }
        let radius = self.model.radius() + self.resolution;
        let q = self.resolution;
        let key = xy.map(|v| (v / q).floor() as i32);
        let n = (radius / q).ceil() as i32;
        let mut system = Matrix3::zeros();
        let mut rhs = V::zeros();
        let mut count = 0;
        let mut intervals = Vec::new();
        let mut common_low = f64::NEG_INFINITY;
        let mut common_high = f64::INFINITY;
        for x in key[0] - n..=key[0] + n {
            for y in key[1] - n..=key[1] + n {
                let dx = (f64::from(x) + 0.5) * q - xy[0];
                let dy = (f64::from(y) + 0.5) * q - xy[1];
                let d2 = dx * dx + dy * dy;
                if d2 >= radius * radius {
                    continue;
                }
                let Some(band) = self.ground([x, y], reference) else {
                    continue;
                };
                let w = (1.0 - d2 / (radius * radius)).powi(2);
                let row = V::new(dx, dy, 1.0);
                let low = if self.surface_measurements {
                    (band.low + q).min(band.top)
                } else {
                    band.top
                };
                common_low = common_low.max(low);
                common_high = common_high.min(band.top);
                system += row * row.transpose() * w;
                rhs += row * (w * (low + band.top) * 0.5);
                count += 1;
                intervals.push((row, w, low, band.top));
            }
        }
        if count < 3 {
            return Err("原始体素接触面观测不足".into());
        }
        if self.surface_measurements && common_low <= common_high {
            return Ok((reference.clamp(common_low, common_high), [0.0, 0.0, 1.0]));
        }
        let solver = system.lu();
        let mut solved = solver.solve(&rhs).ok_or("接触面观测共线")?;
        if self.surface_measurements {
            for _ in 0..24 {
                let mut rhs = V::zeros();
                for &(row, w, low, high) in &intervals {
                    rhs += row * (w * row.dot(&solved).clamp(low, high));
                }
                let next = solver.solve(&rhs).ok_or("接触面观测共线")?;
                let change = (next - solved).norm();
                solved = next;
                if change < 1e-11 {
                    break;
                }
            }
        }
        let normal = V::new(-solved.x, -solved.y, 1.0).normalize();
        Ok((solved.z, normal.into()))
    }

    pub fn project_motion(
        &self,
        from: BodyPose,
        xy: [f64; 2],
        heading: f64,
    ) -> Result<BodyPose, String> {
        if !xy.into_iter().chain([heading]).all(f64::is_finite) {
            return Err("候选位姿无效".into());
        }
        if xy == [from.0[0], from.0[1]] && angle(heading - yaw(from)).abs() < 1e-12 {
            self.validate_pose(from)?;
            return Ok(from);
        }
        let actual = rotation(from.1)?;
        let normal = actual.column(2).into_owned();
        if normal.z <= 0.0 {
            return Err("机体倒置".into());
        }
        let reference = from.0[2] - self.model.support_offset / normal.z;
        let (a, na) = self.ground_frame([from.0[0], from.0[1]], reference)?;
        let (b, nb) = if xy == [from.0[0], from.0[1]] {
            (a, na)
        } else {
            self.ground_frame(xy, a)?
        };
        let na = V::from(na);
        let nb = V::from(nb);
        let transport = UnitQuaternion::rotation_between(&na, &nb).ok_or("接触法向无法连续接续")?;
        let up = transport * normal;
        let forward = V::new(
            heading.cos() * up.z,
            heading.sin() * up.z,
            -heading.cos() * up.x - heading.sin() * up.y,
        )
        .normalize();
        let left = up.cross(&forward);
        let r = Matrix3::from_columns(&[forward, left, up]);
        let contact = |n: V, u: V| self.model.support_offset * n.dot(&u) / n.z;
        let z = if xy == [from.0[0], from.0[1]] {
            from.0[2]
        } else {
            from.0[2] + b - a + contact(nb, up) - contact(na, normal)
        };
        let pose = ([xy[0], xy[1], z], quaternion(r));
        self.validate_pose(pose)?;
        Ok(pose)
    }

    pub fn validate_pose(&self, pose: BodyPose) -> Result<(), String> {
        if !pose
            .0
            .into_iter()
            .all(|v| v.is_finite() && (v / self.resolution).abs() < f64::from(i32::MAX / 4))
        {
            return Err("机体位置无效".into());
        }
        let prism = shapes::Prism::new(&self.model, pose)?;
        let up = prism.rotation.column(2);
        if up.z <= 0.0 {
            return Err("机体倒置".into());
        }
        let q = self.resolution;
        let low = prism.min.map(|v| (v / q).floor() as i32);
        let high = prism.max.map(|v| (v / q).floor() as i32);
        if (i64::from(high[0]) - i64::from(low[0]) + 1)
            * (i64::from(high[1]) - i64::from(low[1]) + 1)
            > 16384
        {
            return Err("机体查询范围无效".into());
        }
        for x in low[0]..=high[0] {
            for y in low[1]..=high[1] {
                if !prism.column_overlap([x, y], q) {
                    continue;
                }
                let reference = pose.0[2]
                    - (up.x * ((f64::from(x) + 0.5) * q - pose.0[0])
                        + up.y * ((f64::from(y) + 0.5) * q - pose.0[1])
                        + self.model.support_offset)
                        / up.z;
                let ground = self
                    .support_ground([x, y], reference)
                    .ok_or_else(|| format!("缺少足迹支撑列({x},{y})"))?;
                if ground.top > pose.0[2] + q {
                    return Err("机体主体与地面相交".into());
                }
                for band in &self.columns[&[x, y]] {
                    if band.top <= ground.top + 1e-9 || band.low > prism.max[2] {
                        continue;
                    }
                    if prism.intersects(
                        [f64::from(x) * q, f64::from(y) * q, band.low],
                        [f64::from(x + 1) * q, f64::from(y + 1) * q, band.top],
                        0.0,
                    ) {
                        return Err(format!(
                            "原始占据碰撞({x},{y},{:.3}..{:.3})",
                            band.low, band.top
                        ));
                    }
                }
            }
        }
        let (_, normal) = self.ground_frame(
            [pose.0[0], pose.0[1]],
            pose.0[2] - self.model.support_offset / up.z,
        )?;
        let cross = (-normal[0] * yaw(pose).sin() + normal[1] * yaw(pose).cos()).abs() / normal[2];
        if cross.atan().to_degrees() > self.model.max_cross_slope_degrees + 1e-6 {
            return Err("原始地形横坡超限".into());
        }
        Ok(())
    }

    pub fn validate_motion(&self, from: BodyPose, to: BodyPose) -> Result<(), String> {
        self.validate_pose(from)?;
        self.validate_pose(to)?;
        self.interval(from, to, 0)
    }
    fn interval(&self, a: BodyPose, b: BodyPose, depth: usize) -> Result<(), String> {
        let qa = UnitQuaternion::new_normalize(Quaternion::new(a.1[3], a.1[0], a.1[1], a.1[2]));
        let qb = UnitQuaternion::new_normalize(Quaternion::new(b.1[3], b.1[0], b.1[1], b.1[2]));
        let qm = qa.slerp(&qb, 0.5).into_inner();
        let mid = (
            std::array::from_fn(|i| (a.0[i] + b.0[i]) * 0.5),
            [qm.i, qm.j, qm.k, qm.w],
        );
        self.validate_pose(mid)?;
        let body_radius = self.model.radius().hypot(
            self.model.z_bounds[0]
                .abs()
                .max(self.model.z_bounds[1].abs()),
        );
        let padding =
            distance(a.0, b.0) * 0.5 + 2.0 * body_radius * (qa.angle_to(&qb) * 0.25).sin();
        if self.swept_clear(mid, padding)? {
            return Ok(());
        }
        if depth >= 14 {
            return Err("扫掠接触区间未能证明净空".into());
        }
        self.interval(a, mid, depth + 1)?;
        self.interval(mid, b, depth + 1)
    }
    /// 中点凸柱加顶点位移上界包住整段旋转和平移；有歧义时细分，不凭端点放行。
    fn swept_clear(&self, pose: BodyPose, padding: f64) -> Result<bool, String> {
        let prism = shapes::Prism::new(&self.model, pose)?;
        let q = self.resolution;
        let up = prism.rotation.column(2);
        let low = prism.min.map(|v| ((v - padding) / q).floor() as i32);
        let high = prism.max.map(|v| ((v + padding) / q).floor() as i32);
        for x in low[0]..=high[0] {
            for y in low[1]..=high[1] {
                let Some(bands) = self.columns.get(&[x, y]) else {
                    if prism.column_overlap([x, y], q) {
                        return Ok(false);
                    } else {
                        continue;
                    }
                };
                let reference = pose.0[2]
                    - (up.x * ((f64::from(x) + 0.5) * q - pose.0[0])
                        + up.y * ((f64::from(y) + 0.5) * q - pose.0[1])
                        + self.model.support_offset)
                        / up.z;
                let ground = self.support_ground([x, y], reference);
                for band in bands {
                    if ground.is_some_and(|g| band.top <= g.top + 1e-9)
                        || band.low > prism.max[2] + padding
                        || band.top < prism.min[2] - padding
                    {
                        continue;
                    }
                    if prism.intersects(
                        [f64::from(x) * q, f64::from(y) * q, band.low],
                        [f64::from(x + 1) * q, f64::from(y + 1) * q, band.top],
                        padding,
                    ) {
                        return Ok(false);
                    }
                }
            }
        }
        // 支撑区域仍按源精度采样；空列没有被当成新地面。
        Ok(padding <= self.resolution * 0.25)
    }

    pub fn primitive(
        &self,
        start: BodyPose,
        length: f64,
        turn: f64,
    ) -> Result<Vec<BodyPose>, String> {
        self.primitive_until(start, length, turn, None)
    }
    fn primitive_until(
        &self,
        start: BodyPose,
        length: f64,
        turn: f64,
        arrival: Option<([f64; 3], f64)>,
    ) -> Result<Vec<BodyPose>, String> {
        if !length.is_finite() || !turn.is_finite() || length < 0.0 {
            return Err("局部运动参数无效".into());
        }
        self.validate_pose(start)?;
        let n = ((length + turn.abs() * self.model.radius()) / (self.resolution * 0.5))
            .ceil()
            .max(1.0) as usize;
        if n > 4096 || length < 0.0 {
            return Err("局部运动超出范围".into());
        }
        let initial = yaw(start);
        let mut poses = vec![start];
        for i in 1..=n {
            if arrival.is_some_and(|(goal, tolerance)| {
                distance(poses.last().unwrap().0, goal) < tolerance
            }) {
                break;
            }
            let t = i as f64 / n as f64;
            let theta = turn * t;
            let (x, y) = if turn.abs() < 1e-10 {
                (length * t, 0.0)
            } else {
                (
                    length / turn * theta.sin(),
                    length / turn * (1.0 - theta.cos()),
                )
            };
            let next = self.project_motion(
                *poses.last().unwrap(),
                [
                    start.0[0] + x * initial.cos() - y * initial.sin(),
                    start.0[1] + x * initial.sin() + y * initial.cos(),
                ],
                initial + theta,
            )?;
            poses.push(next);
        }
        for pair in poses.windows(2) {
            self.validate_motion(pair[0], pair[1])?;
        }
        Ok(poses)
    }

    /// 独立局部候选：直弦、原折线和走廊平移折线；不读取MLS路线或运动许可。
    pub fn plan_reference(
        &self,
        start: BodyPose,
        reference: &[[f64; 3]],
        corridor: f64,
    ) -> Result<Vec<BodyPose>, String> {
        self.plan_reference_with_goal(start, reference, corridor, None)
    }
    pub fn plan_reference_with_goal(
        &self,
        start: BodyPose,
        reference: &[[f64; 3]],
        corridor: f64,
        arrival: Option<([f64; 3], f64)>,
    ) -> Result<Vec<BodyPose>, String> {
        self.validate_pose(start)?;
        if reference.len() < 2 {
            return Err("参考路线不足".into());
        }
        let end = *reference.last().unwrap();
        let mut candidates = vec![vec![reference[0], end], reference.to_vec()];
        let width = self
            .model
            .footprint
            .iter()
            .map(|p| p[1].abs())
            .fold(0.0, f64::max);
        for shift in [width, -width, width * 2.0, -width * 2.0] {
            if shift.abs() > corridor {
                continue;
            }
            let mut points = reference.to_vec();
            for i in 1..points.len() {
                let direction = (reference[i][1] - reference[i - 1][1])
                    .atan2(reference[i][0] - reference[i - 1][0]);
                points[i][0] -= direction.sin() * shift;
                points[i][1] += direction.cos() * shift;
            }
            candidates.push(points);
        }
        let mut reason = String::new();
        for candidate in candidates {
            let mut path = vec![start];
            let mut failed = false;
            for point in candidate.iter().skip(1) {
                if arrival.is_some_and(|(goal, tolerance)| {
                    distance(path.last().unwrap().0, goal) < tolerance
                }) {
                    break;
                }
                let from = *path.last().unwrap();
                let heading = (point[1] - from.0[1]).atan2(point[0] - from.0[0]);
                let result = (|| {
                    let turn = self.primitive(from, 0.0, angle(heading - yaw(from)))?;
                    let line = self.primitive_until(
                        *turn.last().unwrap(),
                        (point[0] - from.0[0]).hypot(point[1] - from.0[1]),
                        0.0,
                        arrival,
                    )?;
                    Ok::<_, String>(
                        turn.into_iter()
                            .skip(1)
                            .chain(line.into_iter().skip(1))
                            .collect::<Vec<_>>(),
                    )
                })();
                match result {
                    Ok(points) => path.extend(points),
                    Err(error) => {
                        reason = error;
                        failed = true;
                        break;
                    }
                }
            }
            if !failed {
                let last = path.last().unwrap();
                if let Some((goal, tolerance)) = arrival {
                    if distance(last.0, goal) < tolerance {
                        return Ok(path);
                    } else {
                        continue;
                    }
                }
                let tail = reference[reference.len() - 2];
                let tangent = (end[1] - tail[1]).atan2(end[0] - tail[0]);
                let dx = last.0[0] - end[0];
                let dy = last.0[1] - end[1];
                if dx.hypot(dy) <= corridor
                    && dx * tangent.cos() + dy * tangent.sin() >= -1e-6
                    && (last.0[2] - self.model.support_offset - end[2]).abs()
                        <= self.model.step_up.max(self.model.step_down) + self.resolution
                    && angle(yaw(*last) - tangent).cos() > 0.0
                {
                    return Ok(path);
                }
                reason = "候选没有连到参考末端的同层前向截面".into();
            }
        }
        Err(format!("独立体素候选未通过：{reason}"))
    }
}
