//! 逐束自由射线保留发射原点；已补偿的极坐标端点不能代替这些原点。
use super::{compose, inverse, PoseHistory};
use nuwax_codec::lcm::planar::PlanarScan;

/// 一条扫描光束在起始机体参考坐标中的实际或受限证据。
#[derive(Debug, Clone)]
pub struct NavigationRay {
    pub origin_base_m: [f64; 2],
    pub endpoint_base_m: [f64; 2],
    pub hit: bool,
    /// 只有发射原点和终点具有相同采样依据时才提供自由证据。
    pub trace_free: bool,
    /// 有逐束时序时为束时间，否则保留扫描起点这个保守时间下界。
    pub source_timestamp_ns: u64,
}

/// 缺少真实运动历史覆盖返回None，等待同一会话补齐；未知逐束时间不伪造自由原点。
pub fn navigation_rays(
    scan: &PlanarScan,
    history: Option<&PoseHistory>,
    base_from_laser: [f64; 3],
) -> Result<Option<Vec<NavigationRay>>, String> {
    scan.validate().map_err(|e| e.to_string())?;
    if scan.timestamp_ns <= 0 || base_from_laser.iter().any(|v| !v.is_finite()) {
        return Err("射线来源时间或外参无效".into());
    }
    let instant = scan.scan_duration_ns == 0;
    let timed = scan.time_offsets_quality == 1 && !scan.time_offsets_ns_le.is_empty();
    let offsets: Vec<_> = if timed {
        scan.time_offsets_ns_le
            .chunks_exact(8)
            .map(|b| u64::from_le_bytes(b.try_into().unwrap()))
            .collect()
    } else {
        vec![0; scan.beam_count as usize]
    };
    let source = scan.timestamp_ns as u64;
    let reference = if !instant && timed && history.is_some() {
        let history = history.unwrap();
        let Some(pose) = history.at(source, &scan.clock_domain, &scan.clock_id)? else {
            return Ok(None);
        };
        let end = source
            .checked_add(offsets.iter().copied().max().unwrap_or(0))
            .ok_or("射线时间溢出")?;
        if history
            .at(end, &scan.clock_domain, &scan.clock_id)?
            .is_none()
        {
            return Ok(None);
        }
        Some(inverse(pose))
    } else {
        None
    };
    let mut rays = Vec::with_capacity(offsets.len());
    for (beam, &offset) in offsets.iter().enumerate() {
        let state = scan.beam_states_u8[beam];
        if state == 0 {
            continue;
        }
        // 长扫描的已补偿无返回只剩角度和共同量程，不能还原原始光束终点。
        if scan.motion_compensated && !instant && state == 2 {
            continue;
        }
        let stamp = source.checked_add(offset).ok_or("射线时间溢出")?;
        let laser = if let Some(reference) = reference {
            let Some(pose) = history
                .unwrap()
                .at(stamp, &scan.clock_domain, &scan.clock_id)?
            else {
                return Ok(None);
            };
            compose(reference, compose(pose, base_from_laser))
        } else {
            base_from_laser
        };
        let angle = if scan.angular_layout == 1 {
            scan.angle_min_rad + beam as f64 * scan.angle_increment_rad
        } else {
            f32::from_le_bytes(
                scan.angles_f32_le[beam * 4..beam * 4 + 4]
                    .try_into()
                    .unwrap(),
            ) as f64
        };
        let range = if state == 2 {
            scan.range_max_m
        } else {
            f32::from_le_bytes(
                scan.ranges_f32_le[beam * 4..beam * 4 + 4]
                    .try_into()
                    .unwrap(),
            ) as f64
        };
        let endpoint_frame = if scan.motion_compensated {
            base_from_laser
        } else {
            laser
        };
        let endpoint = [
            endpoint_frame[0] + range * (angle + endpoint_frame[2]).cos(),
            endpoint_frame[1] + range * (angle + endpoint_frame[2]).sin(),
        ];
        if endpoint.iter().any(|v| !v.is_finite()) {
            return Err("射线终点溢出".into());
        }
        rays.push(NavigationRay {
            origin_base_m: [laser[0], laser[1]],
            endpoint_base_m: endpoint,
            hit: state == 1,
            trace_free: instant || reference.is_some(),
            source_timestamp_ns: stamp,
        });
    }
    Ok(Some(rays))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::TimedPose;
    fn scan() -> PlanarScan {
        PlanarScan {
            runtime_generation: 1,
            sensor_id: "laser".into(),
            source_sequence: 1,
            timestamp_ns: 1_000_000_000,
            clock_domain: "unix_ns".into(),
            clock_id: String::new(),
            frame_id: "laser".into(),
            calibration_ref: "test".into(),
            beam_count: 2,
            angular_layout: 2,
            angle_min_rad: 0.0,
            angle_increment_rad: 0.0,
            range_min_m: 0.05,
            range_max_m: 3.0,
            scan_duration_ns: 100_000_000,
            timestamp_quality: 1,
            time_offsets_quality: 1,
            timing_uncertainty_ns: 0,
            motion_compensated: false,
            ranges_f32_le: [2.0_f32, 2.0]
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect(),
            angles_f32_le: vec![0; 8],
            beam_states_u8: vec![0, 1],
            intensities_f32_le: vec![],
            time_offsets_ns_le: [0_u64, 100_000_000]
                .into_iter()
                .flat_map(u64::to_le_bytes)
                .collect(),
        }
    }
    fn history() -> PoseHistory {
        let mut history = PoseHistory::default();
        for (time, pose) in [
            (1_000_000_000, [0.0, 0.0, 0.0]),
            (1_100_000_000, [1.0, 0.0, 0.0]),
        ] {
            history
                .push(TimedPose {
                    timestamp_ns: time,
                    clock_domain: "unix_ns".into(),
                    clock_id: String::new(),
                    odom_frame_id: "odom".into(),
                    body_frame_id: "base".into(),
                    pose,
                })
                .unwrap();
        }
        history
    }
    #[test]
    fn moving_hit_and_no_return_keep_their_actual_origins_and_range() {
        let mut scan = scan();
        assert!(!navigation_rays(&scan, None, [0.0; 3]).unwrap().unwrap()[0].trace_free);
        assert!(
            navigation_rays(&scan, Some(&PoseHistory::default()), [0.0; 3])
                .unwrap()
                .is_none()
        );
        let history = history();
        let rays = navigation_rays(&scan, Some(&history), [0.2, 0.0, 0.0])
            .unwrap()
            .unwrap();
        assert_eq!(rays.len(), 1);
        assert_eq!(rays[0].origin_base_m, [1.2, 0.0]);
        assert_eq!(rays[0].endpoint_base_m, [3.2, 0.0]);
        assert!(rays[0].trace_free);
        scan.beam_states_u8[1] = 2;
        let rays = navigation_rays(&scan, Some(&history), [0.2, 0.0, 0.0])
            .unwrap()
            .unwrap();
        assert!(!rays[0].hit);
        assert_eq!(rays[0].endpoint_base_m, [4.2, 0.0]);
        assert_eq!(rays[0].source_timestamp_ns, 1_100_000_000);
    }
    #[test]
    fn compensated_returns_are_not_transformed_twice_and_unknown_timing_cannot_clear() {
        let raw = scan();
        let history = history();
        let corrected = crate::input::deskew_scan(&raw, &history, [0.0; 3])
            .unwrap()
            .unwrap();
        let rays = navigation_rays(&corrected, Some(&history), [0.0; 3])
            .unwrap()
            .unwrap();
        assert_eq!(rays[0].origin_base_m, [1.0, 0.0]);
        assert!((rays[0].endpoint_base_m[0] - 3.0).abs() < 1e-6);
        let mut unknown = raw;
        unknown.time_offsets_quality = 2;
        assert!(!navigation_rays(&unknown, None, [0.0; 3]).unwrap().unwrap()[0].trace_free);
        unknown.time_offsets_ns_le.clear();
        unknown.time_offsets_quality = 0;
        unknown.scan_duration_ns = 0;
        assert!(navigation_rays(&unknown, None, [0.0; 3]).unwrap().unwrap()[0].trace_free);
    }
}
