//! 同步扫描记录共用的位姿与间隔选帧策略。
use super::ScanMetadata;

#[derive(Clone, Copy, Debug)]
pub struct SavePolicy {
    pub translation_m: f64,
    pub rotation_deg: f64,
    pub interval_s: f64,
}
#[cfg(test)]
impl Default for SavePolicy {
    fn default() -> Self {
        Self::from_config(&nuwax_config::test_support::section("geometry"))
    }
}
impl SavePolicy {
    pub fn from_runtime() -> Result<Self, String> {
        Ok(Self::from_config(nuwax_config::geometry()?))
    }
    pub fn from_config(g: &nuwax_config::sections::Geometry) -> Self {
        Self {
            translation_m: g.raw_scan_translation_m,
            rotation_deg: g.raw_scan_rotation_deg,
            interval_s: g.raw_scan_interval_s,
        }
    }
    pub fn selects(&self, previous: Option<&ScanMetadata>, current: &ScanMetadata) -> bool {
        let Some(p) = previous else {
            return true;
        };
        let a = nalgebra::Matrix4::from_column_slice(&p.odom_from_body);
        let b = nalgebra::Matrix4::from_column_slice(&current.odom_from_body);
        let distance = (a.fixed_view::<3, 1>(0, 3) - b.fixed_view::<3, 1>(0, 3)).norm();
        let relative = a.fixed_view::<3, 3>(0, 0).transpose() * b.fixed_view::<3, 3>(0, 0);
        let angle = ((relative.trace() - 1.0) * 0.5)
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        distance >= self.translation_m
            || angle >= self.rotation_deg
            || current
                .source_timestamp_ns
                .saturating_sub(p.source_timestamp_ns) as f64
                >= self.interval_s * 1e9
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn metadata(stamp: u64, x: f64, angle: f64) -> ScanMetadata {
        let pose = nalgebra::Isometry3::new(
            nalgebra::Vector3::new(x, 0., 0.),
            nalgebra::Vector3::new(0., 0., angle.to_radians()),
        );
        ScanMetadata {
            source_timestamp_ns: stamp,
            source_sequence: stamp,
            sensor: "lidar".into(),
            source_generation: 1,
            odom_from_body: pose.to_homogeneous().as_slice().try_into().unwrap(),
            map_from_odom: nalgebra::Matrix4::identity().as_slice().try_into().unwrap(),
            sensor_origin_body: Some([0.28945, 0., -0.046825]),
            observation_minimum_range_m: None,
            point_count: 1,
        }
    }
    #[test]
    fn policy_uses_full_pose_or_source_interval_from_last_selected_frame() {
        let p = SavePolicy::default();
        let start = metadata(1, 0., 0.);
        assert!(p.selects(None, &start));
        assert!(!p.selects(Some(&start), &metadata(500_000_001, 0.09, 2.9)));
        assert!(p.selects(Some(&start), &metadata(500_000_001, 0.101, 0.)));
        assert!(p.selects(Some(&start), &metadata(500_000_001, 0., 3.01)));
        assert!(p.selects(Some(&start), &metadata(1_000_000_001, 0., 0.)));
    }
}
