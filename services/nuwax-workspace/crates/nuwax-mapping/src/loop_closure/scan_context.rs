//! 极坐标高度描述子：环均值检索地点，扇区移位估计航向；结果仅作为几何配准初值。

use nalgebra::{Point3, UnitQuaternion};

const RINGS: usize = 20;
const SECTORS: usize = 60;
const RADIUS_M: f64 = 30.0;
const MIN_SECTORS: usize = 12;
const MAX_DISTANCE: f64 = 0.15;

#[derive(Debug, Clone)]
pub(super) struct ScanContext {
    heights: Box<[f32; RINGS * SECTORS]>,
    ring_key: [f32; RINGS],
    sector_norm: [f32; SECTORS],
    occupied_sectors: usize,
    pub level_from_body: UnitQuaternion<f64>,
}

impl ScanContext {
    pub fn new(points: &[Point3<f64>], odom_rotation: UnitQuaternion<f64>) -> Self {
        let (_, _, yaw) = odom_rotation.euler_angles();
        let level_from_body = UnitQuaternion::from_euler_angles(0.0, 0.0, -yaw) * odom_rotation;
        let mut heights = Box::new([0.0_f32; RINGS * SECTORS]);
        for point in points {
            let p = level_from_body * point;
            let radius = p.x.hypot(p.y);
            if !(0.5..RADIUS_M).contains(&radius) || !p.z.is_finite() {
                continue;
            }
            // 旋转产生的舍入误差不能把恰好位于环边界的同一点随机移到前一环。
            let ring = ((radius / RADIUS_M * RINGS as f64 + 1e-9) as usize).min(RINGS - 1);
            let sector = (p.y.atan2(p.x).rem_euclid(std::f64::consts::TAU) / std::f64::consts::TAU
                * SECTORS as f64) as usize
                % SECTORS;
            // 正高度编码保留低矮机器人周围的地面；空格仍明确为零。
            let height = (p.z + 2.0).clamp(0.01, 30.0) as f32;
            let cell = &mut heights[sector * RINGS + ring];
            *cell = cell.max(height);
        }
        let mut ring_key = [0.0; RINGS];
        let mut sector_norm = [0.0_f32; SECTORS];
        for sector in 0..SECTORS {
            for ring in 0..RINGS {
                let h = heights[sector * RINGS + ring];
                ring_key[ring] += h / SECTORS as f32;
                sector_norm[sector] += h * h;
            }
            sector_norm[sector] = sector_norm[sector].sqrt();
        }
        let occupied_sectors = sector_norm.iter().filter(|&&n| n > 0.0).count();
        Self {
            heights,
            ring_key,
            sector_norm,
            occupied_sectors,
            level_from_body,
        }
    }

    pub fn ring_distance(&self, other: &Self) -> f64 {
        self.ring_key
            .iter()
            .zip(other.ring_key)
            .map(|(&a, b)| f64::from(a - b).powi(2))
            .sum()
    }

    /// 返回将当前水平坐标转到历史水平坐标的航向。扇区不足和旋转对称场景不猜航向。
    pub fn align_to(&self, historic: &Self) -> Option<(f64, f64)> {
        if self.occupied_sectors < MIN_SECTORS || historic.occupied_sectors < MIN_SECTORS {
            return None;
        }
        let mut scores = [f64::INFINITY; SECTORS];
        for (shift, score) in scores.iter_mut().enumerate() {
            let mut similarity = 0.0;
            let mut support = 0;
            for sector in 0..SECTORS {
                let target = (sector + shift) % SECTORS;
                let norm = self.sector_norm[sector] * historic.sector_norm[target];
                if norm <= 0.0 {
                    continue;
                }
                let dot: f32 = (0..RINGS)
                    .map(|ring| {
                        self.heights[sector * RINGS + ring]
                            * historic.heights[target * RINGS + ring]
                    })
                    .sum();
                similarity += f64::from(dot / norm).clamp(0.0, 1.0);
                support += 1;
            }
            if support >= MIN_SECTORS
                && support * 2 >= self.occupied_sectors.max(historic.occupied_sectors)
            {
                *score = 1.0 - similarity / support as f64;
            }
        }
        let (shift, &score) = scores
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))?;
        if score > MAX_DISTANCE {
            return None;
        }
        // 相邻扇区属于量化误差；远处近似同分说明航向不可辨识。
        if scores.iter().enumerate().any(|(other, &value)| {
            let delta = shift.abs_diff(other);
            delta.min(SECTORS - delta) > 2 && value < score + 0.01
        }) {
            return None;
        }
        Some((score, shift as f64 * std::f64::consts::TAU / SECTORS as f64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_recovers_return_yaw_and_rejects_uninformative_clouds() {
        let world: Vec<_> = (0..SECTORS)
            .flat_map(|sector| {
                (0..3).map(move |k| {
                    let a = (sector as f64 + 0.3) * std::f64::consts::TAU / SECTORS as f64;
                    let r = 2.0 + ((sector * 7 + k * 13) % 17) as f64;
                    Point3::new(r * a.cos(), r * a.sin(), (sector % 5) as f64 * 0.3)
                })
            })
            .collect();
        let rotation = UnitQuaternion::from_euler_angles(0.1, -0.07, std::f64::consts::PI);
        let body: Vec<_> = world.iter().map(|p| rotation.inverse() * p).collect();
        let historic = ScanContext::new(&world, UnitQuaternion::identity());
        let current = ScanContext::new(&body, rotation);
        let (distance, yaw) = current.align_to(&historic).unwrap();
        assert!(distance < 1e-5);
        assert!((yaw - std::f64::consts::PI).abs() < 1e-8);
        let recovered = historic.level_from_body.inverse()
            * UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
            * current.level_from_body;
        assert!(recovered.angle_to(&rotation) < 1e-8);
        assert!(ScanContext::new(&[], UnitQuaternion::identity())
            .align_to(&historic)
            .is_none());
        let circle: Vec<_> = (0..SECTORS)
            .map(|s| {
                let a = (s as f64 + 0.3) * std::f64::consts::TAU / SECTORS as f64;
                Point3::new(5.0 * a.cos(), 5.0 * a.sin(), 0.0)
            })
            .collect();
        let symmetric = ScanContext::new(&circle, UnitQuaternion::identity());
        assert!(symmetric.align_to(&symmetric).is_none());
    }
}
