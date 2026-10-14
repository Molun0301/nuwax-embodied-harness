//! 极坐标高度描述子：环均值检索地点，扇区移位估计航向；结果仅作为几何配准初值。

use nalgebra::{Point3, UnitQuaternion};

const RINGS: usize = 20;
const SECTORS: usize = 60;

/// 扫描上下文的描述范围和检索准入；环数、扇区数仍采用固定布局。
#[derive(Debug, Clone, Copy)]
pub struct ScanContextConfig {
    /// 描述子接纳点的水平半径上限，单位米。
    pub radius_m: f64,
    /// 有效扇区和共同支撑扇区的最低数量。
    pub min_sectors: usize,
    /// 接纳地点匹配的余弦距离上限。
    pub max_distance: f64,
    /// 最佳航向与远处其它航向至少相差的分数。
    pub ambiguity_margin: f64,
    /// 环键粗检索之后精比的历史描述子数量。
    pub candidate_count: usize,
}

impl Default for ScanContextConfig {
    fn default() -> Self {
        Self {
            radius_m: 30.0,
            min_sectors: 12,
            max_distance: 0.15,
            ambiguity_margin: 0.01,
            candidate_count: 6,
        }
    }
}

impl ScanContextConfig {
    pub(super) fn is_valid(self) -> bool {
        self.radius_m.is_finite() && self.radius_m > 0.5 && self.radius_m <= 200.0
            && (3..=SECTORS).contains(&self.min_sectors)
            && self.max_distance.is_finite() && (0.0..=1.0).contains(&self.max_distance)
            && self.ambiguity_margin.is_finite() && self.ambiguity_margin > 0.0
            && self.ambiguity_margin <= 1.0
            && (1..=64).contains(&self.candidate_count)
    }
}

#[derive(Debug, Clone)]
pub(super) struct ScanContext {
    heights: Box<[f32; RINGS * SECTORS]>,
    ring_key: [f32; RINGS],
    sector_norm: [f32; SECTORS],
    occupied_sectors: usize,
    pub level_from_body: UnitQuaternion<f64>,
}

impl ScanContext {
    pub fn new(points: &[Point3<f64>], odom_rotation: UnitQuaternion<f64>, config: ScanContextConfig) -> Self {
        let (_, _, yaw) = odom_rotation.euler_angles();
        let level_from_body = UnitQuaternion::from_euler_angles(0.0, 0.0, -yaw) * odom_rotation;
        let mut heights = Box::new([0.0_f32; RINGS * SECTORS]);
        for point in points {
            let p = level_from_body * point;
            let radius = p.x.hypot(p.y);
            if !(0.5..config.radius_m).contains(&radius) || !p.z.is_finite() {
                continue;
            }
            // 旋转产生的舍入误差不能把恰好位于环边界的同一点随机移到前一环。
            let ring = ((radius / config.radius_m * RINGS as f64 + 1e-9) as usize).min(RINGS - 1);
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
    pub fn align_to(&self, historic: &Self, config: ScanContextConfig) -> Result<(f64, f64), &'static str> {
        if self.occupied_sectors < config.min_sectors || historic.occupied_sectors < config.min_sectors {
            return Err("insufficient_sectors");
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
            if support >= config.min_sectors
                && support * 2 >= self.occupied_sectors.max(historic.occupied_sectors)
            {
                *score = 1.0 - similarity / support as f64;
            }
        }
        let (shift, &score) = scores
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1)).ok_or("insufficient_sectors")?;
        if !score.is_finite() {
            return Err("insufficient_common_sectors");
        }
        if score > config.max_distance {
            return Err("descriptor_distance");
        }
        // 相邻扇区属于量化误差；远处近似同分说明航向不可辨识。
        if scores.iter().enumerate().any(|(other, &value)| {
            let delta = shift.abs_diff(other);
            delta.min(SECTORS - delta) > 2 && value < score + config.ambiguity_margin
        }) {
            return Err("ambiguous_yaw");
        }
        Ok((score, shift as f64 * std::f64::consts::TAU / SECTORS as f64))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configured_radius_excludes_far_geometry_and_preserves_return_yaw() {
        let config = ScanContextConfig { radius_m: 12.0, ..Default::default() };
        let near: Vec<_> = (0..SECTORS).map(|s| {
            let angle = (s as f64 + 0.3) * std::f64::consts::TAU / SECTORS as f64;
            let radius = 2.0 + (s * 7 % 13) as f64 * 0.6;
            Point3::new(radius * angle.cos(), radius * angle.sin(), (s * 11 % 17) as f64 * 0.1)
        }).collect();
        let mut mixed = near.clone();
        mixed.push(Point3::new(13.0, 0.0, 50.0));
        let historic = ScanContext::new(&near, UnitQuaternion::identity(), config);
        let cropped = ScanContext::new(&mixed, UnitQuaternion::identity(), config);
        assert_eq!(historic.heights, cropped.heights);
        let rotation = UnitQuaternion::from_euler_angles(0.1, -0.07, std::f64::consts::PI);
        let body: Vec<_> = near.iter().map(|p| rotation.inverse() * p).collect();
        let current = ScanContext::new(&body, rotation, config);
        let (distance, yaw) = current.align_to(&historic, config).unwrap();
        assert!(distance < 1e-5);
        assert!((yaw - std::f64::consts::PI).abs() < 1e-8);
    }

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
        let config = ScanContextConfig::default();
        let historic = ScanContext::new(&world, UnitQuaternion::identity(), config);
        let current = ScanContext::new(&body, rotation, config);
        let (distance, yaw) = current.align_to(&historic, config).unwrap();
        assert!(distance < 1e-5);
        assert!((yaw - std::f64::consts::PI).abs() < 1e-8);
        let recovered = historic.level_from_body.inverse()
            * UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
            * current.level_from_body;
        assert!(recovered.angle_to(&rotation) < 1e-8);
        assert!(ScanContext::new(&[], UnitQuaternion::identity(), config)
            .align_to(&historic, config)
            .is_err());
        let circle: Vec<_> = (0..SECTORS)
            .map(|s| {
                let a = (s as f64 + 0.3) * std::f64::consts::TAU / SECTORS as f64;
                Point3::new(5.0 * a.cos(), 5.0 * a.sin(), 0.0)
            })
            .collect();
        let symmetric = ScanContext::new(&circle, UnitQuaternion::identity(), config);
        assert_eq!(symmetric.align_to(&symmetric, config), Err("ambiguous_yaw"));
    }
}
