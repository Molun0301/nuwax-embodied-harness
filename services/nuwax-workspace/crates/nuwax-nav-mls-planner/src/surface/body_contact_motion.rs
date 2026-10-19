//! 连续刚体检查：区间分离证书与实际相交位姿，不用固定采样判安全。
use super::*;
use std::collections::{HashMap, VecDeque};
use std::sync::Arc;

type Pose = ([f64; 3], [f64; 4]);

struct PreparedPose {
    query: RigidBodyQuery,
}

/// 一次 sweep 的源几何、位姿和区间证书准备，全部占据共享，不跨 owner 留存。
pub(in crate::surface) struct RigidMotionQuery {
    primitives: Arc<[RigidPrimitive]>,
    from: Pose,
    to: Pose,
    radius: f64,
    frames: HashMap<u64, PreparedPose>,
    intervals: HashMap<(u64, u64), Arc<[[f64; 3]]>>,
    work: usize,
}

impl RigidMotionQuery {
    pub(in crate::surface) fn new(
        primitives: Arc<[RigidPrimitive]>,
        from: Pose,
        to: Pose,
    ) -> Result<Self, SurfaceError> {
        finite(from.0)?;
        finite(to.0)?;
        let qa = normalized(from.1)?;
        let mut qb = normalized(to.1)?;
        if qa.iter().zip(qb).map(|(a, b)| a * b).sum::<f64>() < 0.0 {
            qb = qb.map(|value| -value);
        }
        let radius = primitives
            .iter()
            .map(|primitive| {
                let local_radius = match primitive.shape {
                    RigidShape::Box { size_m } => norm(size_m.map(|value| value * 0.5)),
                    RigidShape::Sphere { radius_m } => radius_m,
                    RigidShape::Cylinder { radius_m, length_m } => radius_m.hypot(length_m * 0.5),
                };
                norm(primitive.translation) + local_radius
            })
            .fold(0.0, f64::max);
        Ok(Self {
            primitives,
            from: (from.0, qa),
            to: (to.0, qb),
            radius,
            frames: HashMap::new(),
            intervals: HashMap::new(),
            work: 0,
        })
    }

    /// 整段候选外界；同窄相位误差尺度向外舍入，不依赖 whole-body 包含实体。
    pub(in crate::surface) fn bounds(&self) -> Result<([f64; 3], [f64; 3]), SurfaceError> {
        let scale = self.radius.max(max_abs(self.from.0)).max(max_abs(self.to.0));
        let allowance = candidate_allowance(scale);
        let low = std::array::from_fn(|axis| {
            (self.from.0[axis].min(self.to.0[axis]) - self.radius - allowance).next_down()
        });
        let high = std::array::from_fn(|axis| {
            (self.from.0[axis].max(self.to.0[axis]) + self.radius + allowance).next_up()
        });
        finite(low)?;
        finite(high)?;
        Ok((low, high))
    }

    pub(in crate::surface) fn pose(&self, t: f64) -> Pose {
        let xyz = std::array::from_fn(|axis| {
            if self.from.0[axis] == self.to.0[axis] {
                self.from.0[axis]
            } else {
                self.from.0[axis] * (1.0 - t) + self.to.0[axis] * t
            }
        });
        let q: [f64; 4] =
            std::array::from_fn(|axis| self.from.1[axis] * (1.0 - t) + self.to.1[axis] * t);
        let length = q.iter().map(|value| value * value).sum::<f64>().sqrt();
        (xyz, q.map(|value| value / length))
    }

    fn frame(&mut self, t: f64) -> Result<&PreparedPose, SurfaceError> {
        let key = t.to_bits();
        if !self.frames.contains_key(&key) {
            let query = RigidBodyQuery::new(&self.primitives, self.pose(t))?;
            self.frames.insert(key, PreparedPose { query });
        }
        Ok(self.frames.get(&key).expect("当前 sweep 已准备位姿"))
    }

    /// 每个实体相对中点集合的逐世界轴位移上界。只依赖本段运动，所有占据复用。
    fn interval_padding(&mut self, lo: f64, hi: f64) -> Result<Arc<[[f64; 3]]>, SurfaceError> {
        let key = (lo.to_bits(), hi.to_bits());
        if let Some(value) = self.intervals.get(&key) {
            return Ok(value.clone());
        }
        let mid = lo + (hi - lo) * 0.5;
        let a = self.pose(lo);
        let b = self.pose(hi);
        let center = self.pose(mid);
        let arc = RotationArc::new(a.1, b.1, center.1)?;
        let translation: [f64; 3] = std::array::from_fn(|i| {
            (a.0[i] - center.0[i]).abs().max((b.0[i] - center.0[i]).abs())
        });
        let mut values = Vec::with_capacity(self.primitives.len());
        for primitive in self.primitives.iter() {
            let rotational = match primitive.shape {
                RigidShape::Sphere { .. } => arc.point_displacement(primitive.translation),
                RigidShape::Box { size_m } => {
                    let mut bound = [0.0_f64; 3];
                    // 凸体的每个点都是八角点的凸组合；角点逐轴界覆盖整个盒子。
                    for bits in 0..8 {
                        let corner = std::array::from_fn(|axis| {
                            primitive.translation[axis] + (0..3).map(|i| {
                                primitive.axes[i][axis] * size_m[i] * if bits & (1 << i) == 0 { -0.5 } else { 0.5 }
                            }).sum::<f64>()
                        });
                        let displacement = arc.point_displacement(corner);
                        for i in 0..3 { bound[i] = bound[i].max(displacement[i]); }
                    }
                    bound
                }
                RigidShape::Cylinder { radius_m, length_m } => {
                    let mut bound = arc.point_displacement(primitive.translation);
                    // 圆柱集合由中心、轴线、半径和长度决定；绕自身轴的 roll 没有位移。
                    // 两轴最小旋转的 Hausdorff 距离 <= 外接球半径 * 单位轴弦长。
                    // 同一最短旋转圆弧上，轴到中点的最大弦长在区间两端。
                    let axis = primitive.axes[2];
                    let middle = world(arc.middle, [0.0; 3], axis);
                    let first = world(arc.first, [0.0; 3], axis);
                    let last = world(arc.last, [0.0; 3], axis);
                    let chord = norm(std::array::from_fn(|i| first[i] - middle[i]))
                        .max(norm(std::array::from_fn(|i| last[i] - middle[i])));
                    let shape_motion = if chord == 0.0 { 0.0 } else {
                        (radius_m.hypot(length_m * 0.5) * chord * (1.0 + ROUNDING_FACTOR)).next_up()
                    };
                    for value in &mut bound { *value += shape_motion; }
                    bound
                }
            };
            let padding = std::array::from_fn(|i| {
                let value = translation[i] + rotational[i];
                // 集合/坐标严格不动时直接复用窄相位；不再叠加第二份绝对容差。
                if value == 0.0 { 0.0 } else {
                    (value + ROUNDING_FACTOR * value.max(a.0[i].abs()).max(b.0[i].abs())).next_up()
                }
            });
            finite(padding)?;
            values.push(padding);
        }
        let result: Arc<[[f64; 3]]> = values.into();
        self.intervals.insert(key, result.clone());
        Ok(result)
    }

    /// 返回任一个真实相交时刻；None 只在所有子区间均有分离证书时返回。
    /// min/max 允许相等，故真实 recent 点不扩成占据体素。
    pub(in crate::surface) fn intersection(
        &mut self,
        min: [f64; 3],
        max: [f64; 3],
        interval: [f64; 2],
    ) -> Result<Option<f64>, SurfaceError> {
        finite(min)?;
        finite(max)?;
        if (0..3).any(|axis| min[axis] > max[axis])
            || !interval.into_iter().all(f64::is_finite)
            || interval[0] < 0.0 || interval[1] > 1.0 || interval[0] > interval[1]
        { return Err(SurfaceError::InvalidPose); }
        if self.primitives.is_empty() { return Ok(None); }
        for t in interval {
            if self.frame(t)?.query.intersects_voxel(min, max)? { return Ok(Some(t)); }
        }
        // 宽度优先可先找到真实闭接触，不在接触边缘一侧无限逼近而饿死另一侧。
        let mut pending = VecDeque::from([interval]);
        let mut unresolved = false;
        while let Some([lo, hi]) = pending.pop_front() {
            self.work += 1;
            let mid = lo + (hi - lo) * 0.5;
            if self.frame(mid)?.query.intersects_voxel(min, max)? { return Ok(Some(mid)); }
            let paddings = self.interval_padding(lo, hi)?;
            let mut separated = true;
            for (index, padding) in paddings.iter().enumerate() {
                let expanded_min = std::array::from_fn(|i| {
                    if padding[i] == 0.0 { min[i] } else { (min[i] - padding[i]).next_down() }
                });
                let expanded_max = std::array::from_fn(|i| {
                    if padding[i] == 0.0 { max[i] } else { (max[i] + padding[i]).next_up() }
                });
                // K(t) 与 O 相交 => K(mid) 与 O 的逐轴 padding 扩张相交。
                // 因此本次不相交是整个时间区间的分离证书。
                if self.frame(mid)?.query.primitive_intersects_voxel(index, expanded_min, expanded_max)? {
                    separated = false;
                    break;
                }
            }
            if separated { continue; }
            if mid == lo || mid == hi { unresolved = true; continue; }
            pending.push_back([lo, mid]);
            pending.push_back([mid, hi]);
        }
        if unresolved {
            // 这是有限浮点时间分辨率的实际限制，不是占据，也不是支撑完整性门。
            return Err(SurfaceError::InvalidConfig(
                "continuous rigid collision reached floating-point interval limit; clearance unresolved".into()));
        }
        Ok(None)
    }

    #[cfg(test)]
    pub(super) fn work(&self) -> usize { self.work }
    #[cfg(test)]
    pub(super) fn prepared_poses(&self) -> usize { self.frames.len() }
    #[cfg(test)]
    pub(super) fn prepared_intervals(&self) -> usize { self.intervals.len() }
}

/// 最短 nlerp 只改变沿同一个旋转圆弧的速度；坐标极值可用 sin/cos 驻点求尽。
struct RotationArc {
    first: [[f64; 3]; 3],
    middle: [[f64; 3]; 3],
    last: [[f64; 3]; 3],
    axis: [f64; 3],
    angle: f64,
}
impl RotationArc {
    fn new(a: [f64; 4], b: [f64; 4], middle: [f64; 4]) -> Result<Self, SurfaceError> {
        // 相同实际旋转不进行相对四元数乘加；任意倾斜 q 也保持严格恒等。
        if a == b {
            return Ok(Self { first:rotation(a)?,middle:rotation(middle)?,last:rotation(b)?,
                axis:[0.0;3],angle:0.0 });
        }
        // b * conjugate(a) 是世界坐标中的相对旋转。
        let vector = [
            -b[3]*a[0]+b[0]*a[3]-b[1]*a[2]+b[2]*a[1],
            -b[3]*a[1]+b[0]*a[2]+b[1]*a[3]-b[2]*a[0],
            -b[3]*a[2]-b[0]*a[1]+b[1]*a[0]+b[2]*a[3],
        ];
        let length = norm(vector);
        let scalar = a.iter().zip(b).map(|(a,b)|a*b).sum::<f64>();
        Ok(Self {
            first: rotation(a)?, middle: rotation(middle)?, last: rotation(b)?,
            axis: if length == 0.0 { [0.0; 3] } else { vector.map(|v| v / length) },
            angle: if length == 0.0 { 0.0 } else { 2.0 * length.atan2(scalar.max(0.0)) },
        })
    }
    fn point_displacement(&self, local: [f64; 3]) -> [f64; 3] {
        let first = world(self.first, [0.0; 3], local);
        let middle = world(self.middle, [0.0; 3], local);
        let last = world(self.last, [0.0; 3], local);
        if self.angle == 0.0 { return std::array::from_fn(|i| {
            (first[i]-middle[i]).abs().max((last[i]-middle[i]).abs())
        }); }
        let parallel = self.axis.map(|v| v * dot(self.axis, first));
        let cosine: [f64; 3] = std::array::from_fn(|i| first[i] - parallel[i]);
        let sine = cross(self.axis, first);
        std::array::from_fn(|i| {
            let mut bound = (first[i]-middle[i]).abs().max((last[i]-middle[i]).abs());
            if cosine[i] == 0.0 && sine[i] == 0.0 { return bound; }
            let stationary = sine[i].atan2(cosine[i]);
            for k in -1..=2 {
                let theta = stationary + f64::from(k) * std::f64::consts::PI;
                if theta >= 0.0 && theta <= self.angle {
                    let value = parallel[i] + cosine[i] * theta.cos() + sine[i] * theta.sin();
                    bound = bound.max((value-middle[i]).abs());
                }
            }
            // 覆盖相对轴归一化、点积与三角运算舍入；零转动坐标保持严格零。
            (bound + ROUNDING_FACTOR * norm(local)).next_up()
        })
    }
}

fn normalized(q: [f64; 4]) -> Result<[f64; 4], SurfaceError> {
    rotation(q)?;
    let scale = q.into_iter().map(f64::abs).fold(0.0, f64::max);
    let q = q.map(|value| value / scale);
    let length = q.iter().map(|value| value * value).sum::<f64>().sqrt();
    Ok(q.map(|value| value / length))
}
