//! 二维实时障碍只形成导航派生观测；规范地图身份和源时间由Geometry拥有方核对。
use crate::{lcm_runtime::GeometryRuntimeHandle, slam2d_owner::Binding};
use nuwax_codec::lcm::{
    planar::PlanarScan,
    planar_navigation::{self, PlanarNavigationMap},
    schema,
};
use nuwax_slam_2d_runtime::protocol::{Estimate, Session};
use std::{
    collections::VecDeque,
    sync::{
        atomic::{AtomicI64, Ordering},
        Arc,
    },
    time::{SystemTime, UNIX_EPOCH},
};

#[derive(Default)]
pub(crate) struct Owner {
    scans: parking_lot::Mutex<VecDeque<(u64, u64, Arc<PlanarScan>)>>,
    odometry: parking_lot::Mutex<(u64,nuwax_slam_2d_runtime::input::PoseHistory)>,
    revision: AtomicI64,
    last_published: parking_lot::Mutex<Option<(u64, u64)>>,
    source_stamp: AtomicI64,
}
impl Owner {
    pub(crate) fn capture(&self, generation: u64, epoch: u64, scan: PlanarScan) {
        let mut scans = self.scans.lock();
        if scans.len() >= 16 {
            scans.pop_front();
        }
        scans.push_back((generation, epoch, Arc::new(scan)));
    }
    pub(crate) fn capture_odometry(&self,generation:u64,sample:nuwax_slam_2d_runtime::input::TimedPose)->Result<(),String> {
        let mut history=self.odometry.lock();
        if history.0!=generation {history.0=generation;history.1.clear();}
        history.1.push(sample)
    }
    fn next_revision(&self) -> Result<i64, String> {
        self.revision
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |v| v.checked_add(1))
            .map(|v| v + 1)
            .map_err(|_| "二维导航观测修订耗尽".into())
    }
    pub(crate) fn fresh(&self) -> bool {
        let stamp = self.source_stamp.load(Ordering::Acquire);
        stamp > 0 && now_ns().saturating_sub(stamp) <= planar_navigation::MAX_SOURCE_AGE_NS
    }
}
fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos().min(i64::MAX as u128) as i64)
        .unwrap_or(0)
}

pub(crate) async fn revoke(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    session: &Session,
) -> Result<(), String> {
    binding.navigation.source_stamp.store(0, Ordering::Release);
    let value = PlanarNavigationMap {
        runtime_generation: binding.client.identity().runtime_generation as i64,
        session_id: session.session_id.clone(),
        session_generation: session.session_generation as i64,
        observation_revision: binding.navigation.next_revision()?,
        cleared: true,
        ..Default::default()
    };
    let body = planar_navigation::encode_body(&value).map_err(|e| e.to_string())?;
    publish(bridge, binding, body).await
}
async fn publish(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    body: Vec<u8>,
) -> Result<(), String> {
    let frame = crate::typed_codec::build_typed_frame(
        binding.navigation_binding.0.clone(),
        binding.navigation_binding.1.clone(),
        body,
        0,
    );
    tokio::time::timeout(
        std::time::Duration::from_millis(500),
        bridge.transport.publish(frame),
    )
    .await
    .map_err(|_| "二维导航观测发布超时".to_string())?
    .map_err(|e| e.to_string())
}
pub(crate) async fn observe(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    session: &Session,
    estimate: &Estimate,
    epoch: u64,
) -> Result<(), String> {
    if !matches!(
        estimate.disposition.as_str(),
        "initialized" | "accepted" | "motion_skipped"
    ) {
        return Ok(());
    }
    let timestamp = binding.settings.unix_stamp(estimate.source_timestamp_ns)? as i64;
    if timestamp > now_ns() + 50_000_000
        || now_ns().saturating_sub(timestamp) > planar_navigation::MAX_SOURCE_AGE_NS
    {
        return Ok(());
    }
    if binding.navigation.last_published.lock().as_ref()
        == Some(&(session.session_generation, estimate.source_sequence))
    {
        return Ok(());
    }
    let scan = binding
        .navigation
        .scans
        .lock()
        .iter()
        .find(|(generation, source_epoch, scan)| {
            *generation == session.session_generation
                && *source_epoch == epoch
                && scan.source_sequence as u64 == estimate.source_sequence
                && scan.timestamp_ns as u64 == estimate.source_timestamp_ns
        })
        .map(|(_, _, scan)| scan.clone());
    let Some(scan) = scan else { return Ok(()) };
    let scan=if session.odometry_source==nuwax_slam_2d_runtime::protocol::OdometrySource::Device && !scan.motion_compensated && scan.time_offsets_quality==1 {
        let owner=binding.navigation.clone();let source=scan.clone();let generation=session.session_generation;
        let offset=session.core_config["base_from_laser"].as_array().ok_or("二维导航外参缺失")?;
        let extrinsic=[offset[0].as_f64().ok_or("二维外参非法")?,offset[1].as_f64().ok_or("二维外参非法")?,offset[2].as_f64().ok_or("二维外参非法")?];
        let corrected=tokio::task::spawn_blocking(move|| {
            let history=owner.odometry.lock();
            if history.0!=generation {return Ok(None);}
            nuwax_slam_2d_runtime::input::deskew_scan(&source,&history.1,extrinsic)
        }).await.map_err(|e|e.to_string())??;
        let Some(corrected)=corrected else {return Ok(())};Arc::new(corrected)
    } else {scan};
    let (grid, zones, transform, authority) = {
        let _lifecycle = bridge.mapping_lifecycle_lock.lock().await;
        let Some(adapter) = &bridge.mapping_adapter else {
            return Ok(());
        };
        let a = adapter.lock();
        let current = a.snapshot();
        if !current.active
            || !a.planar_ready
            || a.planar_epoch != epoch
            || a.planar_revision != estimate.graph_revision
            || current.session_id != session.session_id
            || current.session_generation != session.session_generation
        {
            return Ok(());
        }
        let Some(authority) = bridge
            .committed_map_authority
            .lock()
            .map_err(|_| "规范地图身份锁失效")?
            .clone()
        else {
            return Ok(());
        };
        if authority.session_id != current.session_id
            || authority.session_generation != current.session_generation as i64
            || authority.revision != current.revision as i64
        {
            return Ok(());
        }
        let Some((grid, zones)) = a.planar_navigation_source() else {
            return Ok(());
        };
        let Some(transform) = a.map_odom_transform_snapshot() else {
            return Ok(());
        };
        (grid, zones, transform, authority)
    };
    let extrinsic = binding.settings.template.core_config["base_from_laser"]
        .as_array()
        .ok_or("缺少二维外参")?;
    let extrinsic = [
        extrinsic[0].as_f64().unwrap(),
        extrinsic[1].as_f64().unwrap(),
        extrinsic[2].as_f64().unwrap(),
    ];
    let odom = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(estimate.odom_from_base[0], estimate.odom_from_base[1], 0.0),
        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, estimate.odom_from_base[2]),
    );
    let pose = transform.effective_isometry() * odom;
    let map_pose = [
        pose.translation.x,
        pose.translation.y,
        pose.rotation.euler_angles().2,
    ];
    let source_stamp = grid.grid.ts;
    let merged = tokio::task::spawn_blocking(move || {
        let mut grid = grid.grid.clone();
        trace_scan(&mut grid, &scan, map_pose, extrinsic);
        let raw = crate::lcm_runtime::MapState {
            width: grid.width,
            height: grid.height,
            resolution: grid.resolution as f32,
            origin_x: grid.origin[0],
            origin_y: grid.origin[1],
            frame_id: grid.frame_id.clone(),
            revision: 0,
            timestamp_ns: timestamp as u64,
            data: grid.data,
        };
        match zones {
            Some(zones) => crate::forbidden_zones::compose_effective_map(&raw, &zones),
            None => Ok(raw),
        }
    })
    .await
    .map_err(|e| e.to_string())??;
    let mut value = PlanarNavigationMap {
        runtime_generation: binding.client.identity().runtime_generation as i64,
        session_id: session.session_id.clone(),
        session_generation: session.session_generation as i64,
        map_revision: authority.revision,
        map_payload_sha256: authority.payload_sha256,
        observation_revision: 0,
        source_sequence: estimate.source_sequence as i64,
        source_timestamp_ns: timestamp,
        cleared: false,
        grid: schema::OccupancyGrid {
            header: schema::Header {
                seq: 0,
                stamp_sec: (timestamp / 1_000_000_000) as i32,
                stamp_nsec: (timestamp % 1_000_000_000) as i32,
                frame_id: merged.frame_id,
            },
            info: schema::MapMetaData {
                map_load_time_sec: source_stamp as i32,
                map_load_time_nsec: ((source_stamp.fract()) * 1e9) as i32,
                resolution: merged.resolution,
                width: merged.width as i32,
                height: merged.height as i32,
                origin: schema::Pose {
                    position: [merged.origin_x, merged.origin_y, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
            data: merged.data,
        },
    };
    let _lifecycle = bridge.mapping_lifecycle_lock.lock().await;
    let adapter = bridge.mapping_adapter.as_ref().ok_or("二维适配器不可用")?;
    {
        let a = adapter.lock();
        let current = a.snapshot();
        if !current.active
            || !a.planar_ready
            || a.planar_epoch != epoch
            || binding.fence.snapshot().epoch != epoch
            || current.session_generation != session.session_generation
            || current.revision != authority.revision as u64
            || a.planar_revision != estimate.graph_revision
            || now_ns().saturating_sub(timestamp) > planar_navigation::MAX_SOURCE_AGE_NS
        {
            return Ok(());
        }
    }
    value.observation_revision = binding.navigation.next_revision()?;
    let body = planar_navigation::encode_body(&value).map_err(|e| e.to_string())?;
    publish(bridge, binding, body).await?;
    binding
        .navigation
        .source_stamp
        .store(timestamp, Ordering::Release);
    *binding.navigation.last_published.lock() =
        Some((session.session_generation, estimate.source_sequence));
    Ok(())
}

/// 只覆盖本帧实际测到的自由射线和回波，坏测量保留原地图未知/占据状态。
fn trace_scan(
    grid: &mut nuwax_mapping::OccupancyGrid,
    scan: &PlanarScan,
    pose: [f64; 3],
    extrinsic: [f64; 3],
) {
    let (sin, cos) = pose[2].sin_cos();
    let origin = [
        pose[0] + cos * extrinsic[0] - sin * extrinsic[1],
        pose[1] + sin * extrinsic[0] + cos * extrinsic[1],
    ];
    let index = |x: f64, y: f64| -> Option<usize> {
        let x = ((x - grid.origin[0]) / grid.resolution).floor() as i64;
        let y = ((y - grid.origin[1]) / grid.resolution).floor() as i64;
        (x >= 0 && y >= 0 && x < grid.width as i64 && y < grid.height as i64)
            .then(|| (y as usize) * grid.width as usize + x as usize)
    };
    let mut endpoints = Vec::new();
    for beam in 0..scan.beam_count as usize {
        let state = scan.beam_states_u8[beam];
        if state == 0 {
            continue;
        }
        let measured = f32::from_le_bytes(
            scan.ranges_f32_le[beam * 4..beam * 4 + 4]
                .try_into()
                .unwrap(),
        ) as f64;
        let range = if state == 2 {
            scan.range_max_m
        } else {
            measured
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
        let angle = pose[2] + extrinsic[2] + angle;
        let (dy, dx) = angle.sin_cos();
        let steps = (range / (grid.resolution * 0.5)).ceil() as usize;
        let limit = ((grid.width + grid.height) as usize * 3).min(steps);
        for step in 0..limit {
            let d = (step as f64) * grid.resolution * 0.5;
            let Some(cell) = index(origin[0] + d * dx, origin[1] + d * dy) else {
                break;
            };
            grid.data[cell] = 0;
        }
        if state == 1 {
            if let Some(cell) = index(origin[0] + range * dx, origin[1] + range * dy) {
                endpoints.push(cell);
            }
        }
    }
    for cell in endpoints {
        grid.data[cell] = 100;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn scan(state: u8) -> PlanarScan {
        PlanarScan {
            runtime_generation: 1,
            sensor_id: "laser".into(),
            source_sequence: 0,
            timestamp_ns: 1,
            clock_domain: "unix_ns".into(),
            clock_id: String::new(),
            frame_id: "laser".into(),
            calibration_ref: "cal".into(),
            beam_count: 1,
            angular_layout: 1,
            angle_min_rad: 0.0,
            angle_increment_rad: 1.0,
            range_min_m: 0.05,
            range_max_m: 3.0,
            scan_duration_ns: 1,
            timestamp_quality: 1,
            time_offsets_quality: 0,
            timing_uncertainty_ns: -1,
            motion_compensated: false,
            ranges_f32_le: 2f32.to_le_bytes().to_vec(),
            angles_f32_le: vec![],
            beam_states_u8: vec![state],
            intensities_f32_le: vec![],
            time_offsets_ns_le: vec![],
        }
    }
    #[test]
    fn current_scan_clears_moved_obstacle_and_preserves_unobserved_cells() {
        let mut grid = nuwax_mapping::OccupancyGrid::new(4, 3, 1.0);
        grid.data[5] = 100;
        trace_scan(&mut grid, &scan(1), [0.5, 1.5, 0.0], [0.0, 0.0, 0.0]);
        assert_eq!(grid.data[5], 0);
        assert_eq!(grid.data[6], 100);
        assert_eq!(grid.data[0], -1);
        let before = grid.clone();
        trace_scan(&mut grid, &scan(0), [0.5, 1.5, 0.0], [0.0, 0.0, 0.0]);
        assert_eq!(grid.data, before.data);
    }
    #[test]
    fn obstacle_projection_uses_robot_rotation_and_real_sensor_translation() {
        let mut grid = nuwax_mapping::OccupancyGrid::new(4, 4, 1.0);
        trace_scan(
            &mut grid,
            &scan(1),
            [0.5, 0.5, std::f64::consts::FRAC_PI_2],
            [1.0, 0.0, 0.0],
        );
        assert_eq!(grid.data[4], 0);
        assert_eq!(grid.data[12], 100);
        assert_eq!(grid.data[0], -1);
    }
}
