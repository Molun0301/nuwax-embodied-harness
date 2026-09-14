//! 真实采样时间配对和极坐标归一；禁止用接收时刻替换源时间。
use nuwax_codec::lcm::planar::PlanarScan;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
#[path = "navigation_rays.rs"]
mod navigation_rays;
pub use navigation_rays::{navigation_rays, NavigationRay};
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TimedPose {
    pub timestamp_ns: u64,
    pub clock_domain: String,
    pub clock_id: String,
    pub odom_frame_id: String,
    pub body_frame_id: String,
    pub pose: [f64; 3],
}
#[derive(Debug, Default)]
pub struct PoseHistory {
    samples: VecDeque<TimedPose>,
}
impl PoseHistory {
    pub fn clear(&mut self) {
        self.samples.clear();
    }
    pub fn push(&mut self, sample: TimedPose) -> Result<(), String> {
        if sample.timestamp_ns == 0
            || sample.pose.iter().any(|v| !v.is_finite())
            || sample.odom_frame_id.is_empty()
            || sample.body_frame_id.is_empty()
        {
            return Err("二维里程计字段非法".into());
        }
        if !matches!(
            sample.clock_domain.as_str(),
            "unix_ns" | "monotonic_ns" | "device_ns"
        ) || sample.clock_domain == "device_ns" && sample.clock_id.is_empty()
        {
            return Err("二维里程计时钟非法".into());
        }
        if let Some(last) = self.samples.back() {
            if last.clock_domain != sample.clock_domain
                || last.clock_id != sample.clock_id
                || last.odom_frame_id != sample.odom_frame_id
                || last.body_frame_id != sample.body_frame_id
            {
                return Err("二维里程计来源中途更换".into());
            }
            if &sample == last {
                return Ok(());
            }
            if sample.timestamp_ns <= last.timestamp_ns {
                return Err("二维里程计时间未递增".into());
            }
        }
        let newest = sample.timestamp_ns;
        self.samples.push_back(sample);
        while self.samples.len() > 2
            && (self.samples.len() > 4096
                || newest.saturating_sub(self.samples[1].timestamp_ns) > 5_000_000_000)
        {
            self.samples.pop_front();
        }
        Ok(())
    }
    pub fn at(&self, stamp: u64, domain: &str, clock: &str) -> Result<Option<[f64; 3]>, String> {
        let Some(first) = self.samples.front() else {
            return Ok(None);
        };
        if first.clock_domain != domain || first.clock_id != clock {
            return Err("扫描与里程计时钟不一致".into());
        }
        let after_index = match self
            .samples
            .binary_search_by_key(&stamp, |s| s.timestamp_ns)
        {
            Ok(index) => return Ok(Some(self.samples[index].pose)),
            Err(index) => index,
        };
        if after_index == 0 || after_index >= self.samples.len() {
            return Ok(None);
        }
        let before = &self.samples[after_index - 1];
        let after = &self.samples[after_index];
        if after.timestamp_ns - before.timestamp_ns > 200_000_000 {
            return Err("扫描配对的里程计缺测超过200毫秒".into());
        }
        let ratio = (stamp - before.timestamp_ns) as f64
            / (after.timestamp_ns - before.timestamp_ns) as f64;
        let yaw = (after.pose[2] - before.pose[2] + std::f64::consts::PI)
            .rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        Ok(Some([
            before.pose[0] + ratio * (after.pose[0] - before.pose[0]),
            before.pose[1] + ratio * (after.pose[1] - before.pose[1]),
            (before.pose[2] + ratio * yaw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI,
        ]))
    }
}
/// 将具备真实逐束时间的扫描补偿到起始雷达坐标；返回空值表示等待真实里程计覆盖。
pub fn deskew_scan(
    scan: &PlanarScan,
    history: &PoseHistory,
    base_from_laser: [f64; 3],
) -> Result<Option<PlanarScan>, String> {
    scan.validate().map_err(|e| e.to_string())?;
    if scan.motion_compensated
        || scan.time_offsets_quality != 1
        || scan.time_offsets_ns_le.is_empty()
    {
        return Ok(Some(scan.clone()));
    }
    if scan.timestamp_ns <= 0 || base_from_laser.iter().any(|v| !v.is_finite()) {
        return Err("逐束补偿的源时间或外参非法".into());
    }
    let source = scan.timestamp_ns as u64;
    let offsets: Vec<u64> = scan
        .time_offsets_ns_le
        .chunks_exact(8)
        .map(|b| u64::from_le_bytes(b.try_into().unwrap()))
        .collect();
    let end = source
        .checked_add(offsets.iter().copied().max().unwrap_or(0))
        .filter(|v| *v <= i64::MAX as u64)
        .ok_or("逐束时间越界")?;
    let Some(reference) = history.at(source, &scan.clock_domain, &scan.clock_id)? else {
        return Ok(None);
    };
    if history
        .at(end, &scan.clock_domain, &scan.clock_id)?
        .is_none()
    {
        return Ok(None);
    }
    let reference_from_odom = inverse(compose(reference, base_from_laser));
    let mut output = scan.clone();
    output.angular_layout = 2;
    output.angle_min_rad = 0.0;
    output.angle_increment_rad = 0.0;
    output.angles_f32_le = Vec::with_capacity(offsets.len() * 4);
    for (index, offset) in offsets.into_iter().enumerate() {
        let angle = if scan.angular_layout == 1 {
            scan.angle_min_rad + scan.angle_increment_rad * index as f64
        } else {
            f32::from_le_bytes(
                scan.angles_f32_le[index * 4..index * 4 + 4]
                    .try_into()
                    .unwrap(),
            ) as f64
        };
        if scan.beam_states_u8[index] != 1 {
            output
                .angles_f32_le
                .extend_from_slice(&(angle as f32).to_le_bytes());
            continue;
        }
        let Some(pose) = history.at(source + offset, &scan.clock_domain, &scan.clock_id)? else {
            return Ok(None);
        };
        let delta = compose(reference_from_odom, compose(pose, base_from_laser));
        let range = f32::from_le_bytes(
            scan.ranges_f32_le[index * 4..index * 4 + 4]
                .try_into()
                .unwrap(),
        ) as f64;
        let point = [
            delta[0] + range * (angle + delta[2]).cos(),
            delta[1] + range * (angle + delta[2]).sin(),
        ];
        let corrected = point[0].hypot(point[1]);
        output
            .angles_f32_le
            .extend_from_slice(&(point[1].atan2(point[0]) as f32).to_le_bytes());
        if !corrected.is_finite() || corrected < scan.range_min_m || corrected > scan.range_max_m {
            output.beam_states_u8[index] = 0;
        }
        output.ranges_f32_le[index * 4..index * 4 + 4]
            .copy_from_slice(&(corrected as f32).to_le_bytes());
    }
    output.motion_compensated = true;
    output.validate().map_err(|e| e.to_string())?;
    Ok(Some(output))
}

pub fn normalize(
    scan: &PlanarScan,
    min: f64,
    increment: f64,
    count: usize,
) -> Result<Vec<f64>, String> {
    scan.validate().map_err(|e| e.to_string())?;
    if !min.is_finite()
        || !increment.is_finite()
        || increment <= 0.0
        || !(16..=32768).contains(&count)
        || increment * (count - 1) as f64 > std::f64::consts::TAU + 1e-5
    {
        return Err("目标等角参数非法".into());
    }
    let full = (increment * count as f64 - std::f64::consts::TAU).abs() < 1e-5;
    let mut output = vec![f64::NAN; count];
    for i in 0..scan.beam_count as usize {
        if scan.beam_states_u8[i] != 1 {
            continue;
        }
        let angle = if scan.angular_layout == 1 {
            scan.angle_min_rad + scan.angle_increment_rad * i as f64
        } else {
            f32::from_le_bytes(scan.angles_f32_le[i * 4..i * 4 + 4].try_into().unwrap()) as f64
        };
        let mut relative = (angle - min).rem_euclid(std::f64::consts::TAU);
        if (relative - std::f64::consts::TAU).abs() < 1e-6 {
            relative = 0.0;
        }
        let mut index = (relative / increment + 0.5).floor() as usize;
        if full {
            index %= count;
        }
        if index >= count {
            continue;
        }
        let range =
            f32::from_le_bytes(scan.ranges_f32_le[i * 4..i * 4 + 4].try_into().unwrap()) as f64;
        if !output[index].is_finite() || range < output[index] {
            output[index] = range;
        }
    }
    Ok(output)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pose_json_transport_preserves_every_float_bit() {
        let pose=TimedPose{timestamp_ns:1,clock_domain:"unix_ns".into(),clock_id:String::new(),odom_frame_id:"odom".into(),body_frame_id:"base".into(),
            pose:[1.9941239292061599,0.18971662663702862,0.09424777960769379]};
        let restored:TimedPose=serde_json::from_slice(&serde_json::to_vec(&pose).unwrap()).unwrap();
        for (a,b) in pose.pose.iter().zip(restored.pose) {assert_eq!(a.to_bits(),b.to_bits());}
    }
    #[test]
    fn interpolation_preserves_clock_and_short_yaw_arc() {
        let mut h = PoseHistory::default();
        let a = TimedPose {
            timestamp_ns: 1_000_000_000,
            clock_domain: "unix_ns".into(),
            clock_id: "".into(),
            odom_frame_id: "odom".into(),
            body_frame_id: "base_link".into(),
            pose: [0., 0., 3.1],
        };
        h.push(a.clone()).unwrap();
        let mut b = a;
        b.timestamp_ns += 100_000_000;
        b.pose = [1., 0., -3.1];
        h.push(b).unwrap();
        let p = h.at(1_050_000_000, "unix_ns", "").unwrap().unwrap();
        assert_eq!(p[0], 0.5);
        assert!((p[2].abs() - std::f64::consts::PI).abs() < 1e-9);
        assert!(h.at(1_050_000_000, "monotonic_ns", "").is_err());
        assert_eq!(h.at(1_500_000_000, "unix_ns", "").unwrap(), None);
    }
}

/// 平面刚体组合，参数依次为米、米、弧度。
pub(crate) fn compose(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    let (s, c) = a[2].sin_cos();
    [
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        (a[2] + b[2] + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI,
    ]
}
/// 平面刚体逆变换。
pub(crate) fn inverse(a: [f64; 3]) -> [f64; 3] {
    let (s, c) = a[2].sin_cos();
    [-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]]
}

#[cfg(test)]
mod deskew_tests {
    use super::*;
    fn moving_wall_scan() -> PlanarScan {
        let count = 360usize;
        let mut ranges = Vec::new();
        let mut offsets = Vec::new();
        for index in 0..count {
            let offset = index as u64 * 100_000_000 / (count - 1) as u64;
            let fraction = offset as f64 / 100_000_000.0;
            let yaw = 0.12 * fraction;
            let sensor_x = 1.0 * fraction + 0.22 * yaw.cos() + 0.08 * yaw.sin();
            let angle = -0.6 + 1.2 * index as f64 / (count - 1) as f64;
            // 独立几何依据：所有回波击中世界坐标x=4的静态墙面。
            let range = (4.0 - sensor_x) / (yaw + 0.15 + angle).cos();
            ranges.extend_from_slice(&(range as f32).to_le_bytes());
            offsets.extend_from_slice(&offset.to_le_bytes());
        }
        PlanarScan {
            runtime_generation: 1,
            sensor_id: "laser".into(),
            source_sequence: 1,
            timestamp_ns: 1_000_000_000,
            clock_domain: "unix_ns".into(),
            clock_id: String::new(),
            frame_id: "laser".into(),
            calibration_ref: "cal".into(),
            beam_count: count as i32,
            angular_layout: 1,
            angle_min_rad: -0.6,
            angle_increment_rad: 1.2 / (count - 1) as f64,
            range_min_m: 0.05,
            range_max_m: 30.,
            scan_duration_ns: 100_000_000,
            timestamp_quality: 1,
            time_offsets_quality: 1,
            timing_uncertainty_ns: 0,
            motion_compensated: false,
            ranges_f32_le: ranges,
            angles_f32_le: Vec::new(),
            beam_states_u8: vec![1; count],
            intensities_f32_le: Vec::new(),
            time_offsets_ns_le: offsets,
        }
    }
    #[test]
    fn deskew_keeps_moving_sensor_returns_on_the_same_physical_wall() {
        let scan = moving_wall_scan();
        let mut history = PoseHistory::default();
        let sample = |timestamp_ns, pose| TimedPose {
            timestamp_ns,
            clock_domain: "unix_ns".into(),
            clock_id: String::new(),
            odom_frame_id: "odom".into(),
            body_frame_id: "base".into(),
            pose,
        };
        history.push(sample(1_000_000_000, [0., 0., 0.])).unwrap();
        assert!(
            deskew_scan(&scan, &history, [0.22, -0.08, 0.15])
                .unwrap()
                .is_none(),
            "不能为扫描尾部外推里程计"
        );
        history
            .push(sample(1_100_000_000, [1.0, 0.1, 0.12]))
            .unwrap();
        let corrected = deskew_scan(&scan, &history, [0.22, -0.08, 0.15])
            .unwrap()
            .unwrap();
        assert!(corrected.motion_compensated);
        assert_eq!(corrected.timestamp_ns, scan.timestamp_ns);
        let rays = navigation_rays(&scan, Some(&history), [0.22, -0.08, 0.15]).unwrap().unwrap();
        assert_eq!(rays.len(), scan.beam_count as usize);
        for ray in &rays {
            assert!((ray.endpoint_base_m[0] - 4.0).abs() < 1e-5);
            assert!(ray.trace_free);
        }
        assert!(rays.last().unwrap().origin_base_m[0] - rays[0].origin_base_m[0] > 0.9);

        let mut uncompensated_error = 0.0f64;
        for index in 0..scan.beam_count as usize {
            let angle = f32::from_le_bytes(
                corrected.angles_f32_le[index * 4..index * 4 + 4]
                    .try_into()
                    .unwrap(),
            ) as f64;
            let range = f32::from_le_bytes(
                corrected.ranges_f32_le[index * 4..index * 4 + 4]
                    .try_into()
                    .unwrap(),
            ) as f64;
            assert!((0.22 + range * (angle + 0.15).cos() - 4.0).abs() < 1e-5);
            let raw = f32::from_le_bytes(
                scan.ranges_f32_le[index * 4..index * 4 + 4]
                    .try_into()
                    .unwrap(),
            ) as f64;
            uncompensated_error = uncompensated_error.max(
                (0.22
                    + raw
                        * (scan.angle_min_rad + index as f64 * scan.angle_increment_rad + 0.15)
                            .cos()
                    - 4.0)
                    .abs(),
            );
        }
        assert!(uncompensated_error > 0.2);
        let once = deskew_scan(&corrected, &PoseHistory::default(), [0.22, -0.08, 0.15])
            .unwrap()
            .unwrap();
        assert_eq!(
            once.ranges_f32_le, corrected.ranges_f32_le,
            "已有补偿不得再变换一次"
        );
        let mut estimated = scan.clone();
        estimated.time_offsets_quality = 2;
        assert!(
            !deskew_scan(&estimated, &history, [0.22, -0.08, 0.15])
                .unwrap()
                .unwrap()
                .motion_compensated
        );
    }
}
