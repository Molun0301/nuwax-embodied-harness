//! 二维实时障碍只形成导航派生观测；规范地图身份和源时间由Geometry拥有方核对。
mod mask_cache;
mod local_observation;
mod rays;
#[cfg(test)]
use rays::trace_rays;
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
    mask_cache: parking_lot::Mutex<mask_cache::ForbiddenMaskCache>,
    local_observation: parking_lot::Mutex<local_observation::LocalObservation>,
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
    /// 在原会话历史中取得已发布位姿的真实设备采样；不使用外推补造时间。
    pub(crate) fn odometry_at(&self,generation:u64,timestamp_ns:u64,domain:&str,clock:&str)->Result<Option<[f64;3]>,String> {
        let history=self.odometry.lock();
        if history.0!=generation {return Ok(None);}
        history.1.at(timestamp_ns,domain,clock)
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
        map_frame_generation: bridge.mapping_adapter.as_ref().map_or(0, |a| i64::try_from(a.lock().global_frame_generation()).unwrap_or(-1)),
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
struct ObservationInput {
    scan: Arc<PlanarScan>,
    odom_from_base: [f64;3],
    basis_revision: u64,
    minimum_rays: usize,
}
impl Owner {
    // 调用方已核对原生新鲜匹配依据；局部扫描只使用同会话实际历史，不冒充新的匹配结果。
    fn latest_device_observation(&self,session:&Session,basis:&Estimate,epoch:u64,latest:&nuwax_slam_2d_runtime::input::TimedPose)->Result<Option<ObservationInput>,String> {
        if session.odometry_source!=nuwax_slam_2d_runtime::protocol::OdometrySource::Device
            || !matches!(basis.disposition.as_str(),"initialized"|"accepted"|"motion_skipped")
            || basis.source_timestamp_ns==0 || basis.clock_domain!=session.clock_domain || basis.clock_id!=session.clock_id
            || latest.timestamp_ns<basis.source_timestamp_ns || latest.timestamp_ns-basis.source_timestamp_ns>500_000_000
            || latest.clock_domain!=session.clock_domain || latest.clock_id!=session.clock_id
            || latest.odom_frame_id!=session.odom_frame_id || latest.body_frame_id!=session.body_frame_id {
            return Ok(None);
        }
        let scans=self.scans.lock();let history=self.odometry.lock();
        if history.0!=session.session_generation {return Ok(None);}
        for (generation,source_epoch,scan) in scans.iter().rev() {
            if *generation!=session.session_generation || *source_epoch!=epoch || scan.validate().is_err()
                || scan.sensor_id!=session.sensor_id || scan.frame_id!=session.frame_id || scan.calibration_ref!=session.calibration_ref
                || scan.clock_domain!=session.clock_domain || scan.clock_id!=session.clock_id || scan.timestamp_quality==3
                || (scan.source_sequence as u64)<basis.source_sequence {continue;}
            let end=(scan.timestamp_ns as u64).checked_add(scan.scan_duration_ns as u64);
            if end.is_none_or(|end|end>latest.timestamp_ns) {continue;}
            let minimum_rays=16_usize.max(scan.beam_count as usize/10);
            if scan.beam_states_u8.iter().filter(|&&state|matches!(state,1|2)).count()<minimum_rays {continue;}
            let Some(odom_from_base)=history.1.at(scan.timestamp_ns as u64,&scan.clock_domain,&scan.clock_id)? else {continue;};
            return Ok(Some(ObservationInput{scan:Arc::clone(scan),odom_from_base,basis_revision:basis.graph_revision,minimum_rays}));
        }
        Ok(None)
    }
}

pub(crate) async fn observe_device_latest(
    bridge:&GeometryRuntimeHandle,binding:&Binding,session:&Session,basis:&Estimate,
    latest:&nuwax_slam_2d_runtime::input::TimedPose,epoch:u64,
)->Result<(),String> {
    let Some(input)=binding.navigation.latest_device_observation(session,basis,epoch,latest)? else{return Ok(());};
    observe_source(bridge,binding,session,input,epoch).await
}

pub(crate) async fn observe(
    bridge: &GeometryRuntimeHandle,
    binding: &Binding,
    session: &Session,
    estimate: &Estimate,
    epoch: u64,
) -> Result<(), String> {
    if !matches!(estimate.disposition.as_str(),"initialized"|"accepted"|"motion_skipped") {return Ok(());}
    let scan=binding.navigation.scans.lock().iter().find(|(generation,source_epoch,scan)|
        *generation==session.session_generation && *source_epoch==epoch
        && scan.source_sequence as u64==estimate.source_sequence && scan.timestamp_ns as u64==estimate.source_timestamp_ns)
        .map(|(_,_,scan)|Arc::clone(scan));
    let Some(scan)=scan else{return Ok(());};
    observe_source(bridge,binding,session,ObservationInput{scan,odom_from_base:estimate.odom_from_base,basis_revision:estimate.graph_revision,minimum_rays:0},epoch).await
}

/// Freeze canonical occupancy provenance from its owner, independently of scan time.
fn committed_map_identity(
    receipt: &crate::mapping_commit::OccupancyCommitReceipt,
    session: &crate::mapping_adapter::MappingSessionSnapshot,
    frame_id: &str,
) -> Result<schema::CommittedMapIdentity, String> {
    if receipt.session_id != session.session_id
        || receipt.session_generation != session.session_generation
        || receipt.occupancy_revision != session.revision
        || receipt.session_id.trim().is_empty()
        || receipt.session_id.trim() != receipt.session_id
        || receipt.session_generation == 0
        || receipt.occupancy_revision == 0
        || receipt.commit_id.boot_nonce == 0
        || receipt.commit_id.sequence == 0
        || receipt.backend_generation == 0
        || frame_id.trim().is_empty()
        || frame_id.trim() != frame_id
    {
        return Err("二维规范图与真实占据提交收据不一致".into());
    }
    Ok(schema::CommittedMapIdentity {
        session_id: receipt.session_id.clone(),
        session_generation: i64::try_from(receipt.session_generation).map_err(|_| "二维会话代际越界")?,
        revision: i64::try_from(receipt.occupancy_revision).map_err(|_| "二维地图修订越界")?,
        commit_boot_nonce: receipt.commit_id.boot_nonce.to_be_bytes(),
        commit_sequence: i64::try_from(receipt.commit_id.sequence).map_err(|_| "二维占据提交序号越界")?,
        source_timestamp_ns: i64::try_from(receipt.source_timestamp_ns).map_err(|_| "二维占据源时间越界")?,
        frame_id: frame_id.to_owned(),
    })
}

async fn observe_source(bridge:&GeometryRuntimeHandle,binding:&Binding,session:&Session,input:ObservationInput,epoch:u64)->Result<(),String> {
    let ObservationInput{scan,odom_from_base,basis_revision,minimum_rays}=input;
    let source_sequence=scan.source_sequence as u64;let source_timestamp_ns=scan.timestamp_ns as u64;
    let timestamp=binding.settings.unix_stamp(source_timestamp_ns)? as i64;
    if timestamp>now_ns()+50_000_000 || now_ns().saturating_sub(timestamp)>planar_navigation::MAX_SOURCE_AGE_NS {return Ok(());}
    if binding.navigation.last_published.lock().as_ref().is_some_and(|&(generation,sequence)|generation==session.session_generation&&sequence>=source_sequence) {return Ok(());}
    let offset=session.core_config["base_from_laser"].as_array().ok_or("二维导航外参缺失")?;
    let extrinsic=[offset[0].as_f64().ok_or("二维外参非法")?,offset[1].as_f64().ok_or("二维外参非法")?,offset[2].as_f64().ok_or("二维外参非法")?];
    let owner=binding.navigation.clone();let generation=session.session_generation;
    let device_odometry=session.odometry_source==nuwax_slam_2d_runtime::protocol::OdometrySource::Device;
    let rays=tokio::task::spawn_blocking(move|| {
        let history=owner.odometry.lock();
        if device_odometry && history.0!=generation {return Ok(None);}
        nuwax_slam_2d_runtime::input::navigation_rays(&scan,if device_odometry {Some(&history.1)} else {None},extrinsic)
    }).await.map_err(|e|e.to_string())??;
    let Some(rays)=rays else {return Ok(())};
    if rays.len()<minimum_rays {return Ok(());}
    let (grid, zones, transform, authority, receipt) = {
        let _lifecycle = bridge.mapping_lifecycle_lock.lock().await;
        let Some(adapter) = &bridge.mapping_adapter else {
            return Ok(());
        };
        let a = adapter.lock();
        let current = a.snapshot();
        if !current.active
            || !a.planar_ready
            || a.planar_epoch != epoch
            || a.planar_revision > basis_revision
            || current.session_id != session.session_id
            || current.session_generation != session.session_generation
        {
            return Ok(());
        }
        let Some(receipt) = a.latest_occupancy_commit_receipt() else {
            return Ok(());
        };
        let Some((grid, zones)) = a.planar_navigation_source() else {
            return Ok(());
        };
        let authority = committed_map_identity(&receipt, &current, &grid.grid.frame_id)?;
        let Some(transform) = a.map_odom_transform_snapshot() else {
            return Ok(());
        };
        (grid, zones, transform, authority, receipt)
    };
    let odom = nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::new(odom_from_base[0], odom_from_base[1], 0.0),
        nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, odom_from_base[2]),
    );
    let pose = transform.effective_isometry() * odom;
    let map_pose = [
        pose.translation.x,
        pose.translation.y,
        pose.rotation.euler_angles().2,
    ];
    let source_stamp = grid.grid.ts;
    let navigation = Arc::clone(&binding.navigation);
    let merged = tokio::task::spawn_blocking(move || {
        let grid = navigation.local_observation.lock().compose(&grid.grid, &rays, map_pose)?;
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
        navigation.mask_cache.lock().apply(raw, zones.as_ref())
    })
    .await
    .map_err(|e| e.to_string())??;
    let mut value = PlanarNavigationMap {
        map_frame_generation: i64::try_from(transform.generation).map_err(|_| "二维地图坐标代际越界")?,
        runtime_generation: binding.client.identity().runtime_generation as i64,
        session_id: session.session_id.clone(),
        session_generation: session.session_generation as i64,
        map_revision: authority.revision,
        map_boot_nonce: authority.commit_boot_nonce,
        map_commit_sequence: authority.commit_sequence,
        map_source_timestamp_ns: authority.source_timestamp_ns,
        observation_revision: 0,
        source_sequence: source_sequence as i64,
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
    let _projection = bridge.lock_explore_projection_owned().await;
    let _lifecycle = bridge.mapping_lifecycle_lock.lock().await;
    let adapter = bridge.mapping_adapter.as_ref().ok_or("二维适配器不可用")?;
    {
        let a = adapter.lock();
        let current = a.snapshot();
        if !current.active
            || !a.planar_ready
            || a.planar_epoch != epoch
            || binding.fence.snapshot().epoch != epoch
            || binding.fence.snapshot().active
            || current.session_id != session.session_id
            || current.session_generation != session.session_generation
            || current.revision != authority.revision as u64
            || a.latest_occupancy_commit_receipt().as_ref() != Some(&receipt)
            || a.planar_revision > basis_revision
            || now_ns().saturating_sub(timestamp) > planar_navigation::MAX_SOURCE_AGE_NS
            || a.global_frame_generation() != value.map_frame_generation as u64
            || binding.navigation.source_stamp.load(Ordering::Acquire)>=timestamp
            || binding.navigation.last_published.lock().as_ref().is_some_and(|&(generation,sequence)|
                generation==session.session_generation && sequence>=source_sequence)
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
        Some((session.session_generation, source_sequence));
    drop(_lifecycle);
    bridge.explore.ingest_planar_navigation(&value).await?;
    Ok(())
}

/// 只覆盖本帧实际测到的自由射线和回波，坏测量保留原地图未知/占据状态。
#[cfg(test)]
fn trace_scan(grid:&mut nuwax_mapping::OccupancyGrid,scan:&PlanarScan,pose:[f64;3],extrinsic:[f64;3]) {
    let rays=nuwax_slam_2d_runtime::input::navigation_rays(scan,None,extrinsic).unwrap().unwrap();
    trace_rays(grid,&rays,pose).unwrap();
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
            scan_duration_ns: 0,
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
    fn canonical_identity_uses_exact_owner_receipt_and_rejects_metadata_mismatch() {
        use crate::mapping_adapter::{MappingSessionPhase, MappingSessionSnapshot};
        use crate::mapping_commit::{MappingCommitId, OccupancyCommitReceipt};
        let session = MappingSessionSnapshot {
            phase: MappingSessionPhase::Active,
            session_id: "planar-receipt".into(), map_name: "test-map".into(),
            session_generation: 3, revision: 7, active: true, idempotent_replay: false,
            legacy_checkpoint_reconstructed: false, mapping_backend_kind: "slam_toolbox".into(),
        };
        let receipt = OccupancyCommitReceipt {
            commit_id: MappingCommitId { boot_nonce: 0x1234, sequence: 11 },
            session_id: session.session_id.clone(), session_generation: 3,
            occupancy_revision: 7, source_timestamp_ns: 123_456_789,
            backend_kind: crate::mapping_backend::MappingBackendKind::SlamToolbox,
            backend_generation: 2,
        };
        let identity = committed_map_identity(&receipt, &session, "map").unwrap();
        assert_eq!(identity.commit_boot_nonce, receipt.commit_id.boot_nonce.to_be_bytes());
        assert_eq!(identity.commit_sequence, 11);
        assert_eq!(identity.revision, 7);
        assert_eq!(identity.source_timestamp_ns, 123_456_789);
        for fault in 0..8 {
            let mut invalid = receipt.clone();
            match fault {
                0 => invalid.session_id = "different-session".into(),
                1 => invalid.session_generation += 1,
                2 => invalid.occupancy_revision += 1,
                3 => invalid.commit_id.boot_nonce = 0,
                4 => invalid.commit_id.sequence = 0,
                5 => invalid.commit_id.sequence = u64::MAX,
                6 => invalid.source_timestamp_ns = u64::MAX,
                _ => invalid.backend_generation = 0,
            }
            assert!(committed_map_identity(&invalid, &session, "map").is_err(), "fault {fault}");
        }
    }

    #[test]
    fn fresh_device_scan_uses_real_history_without_fabricating_native_match() {
        use nuwax_slam_2d_runtime::{input::TimedPose,protocol::OdometrySource};
        let session=Session{session_id:"current".into(),map_name:"测试图".into(),session_generation:3,sensor_id:"laser".into(),frame_id:"laser".into(),
            map_frame_id:"map".into(),odom_frame_id:"odom".into(),body_frame_id:"body".into(),clock_domain:"unix_ns".into(),clock_id:String::new(),
            calibration_ref:"cal".into(),odometry_source:OdometrySource::Device,core_config:serde_json::json!({"base_from_laser":[0.0,0.0,0.0]})};
        let basis=Estimate{source_timestamp_ns:1_000_000_000,clock_domain:"unix_ns".into(),clock_id:String::new(),source_sequence:5,
            graph_revision:8,optimizations:0,anchor_odom_from_base:[0.0;3],odom_from_base:[99.0;3],map_from_base:[0.0;3],map_from_odom:[0.0;3],
            covariance:[0.0;9],matching_response:1.0,local_response:1.0,disposition:"accepted".into(),node_count:1,edge_count:0};
        let first=TimedPose{timestamp_ns:1_000_000_000,clock_domain:"unix_ns".into(),clock_id:String::new(),odom_frame_id:"odom".into(),body_frame_id:"body".into(),pose:[0.0;3]};
        let mut latest=first.clone();latest.timestamp_ns=1_200_000_000;latest.pose=[0.2,0.0,0.0];
        let mut observation=scan(1);observation.beam_count=36;observation.angle_increment_rad=0.1;
        observation.ranges_f32_le=(0..36).flat_map(|_|2f32.to_le_bytes()).collect();observation.beam_states_u8=vec![1;36];
        observation.timestamp_ns=1_100_000_000;observation.source_sequence=7;
        observation.validate().unwrap();
        let owner=Owner::default();owner.capture_odometry(3,first.clone()).unwrap();owner.capture_odometry(3,latest.clone()).unwrap();
        owner.capture(3,9,observation.clone());
        let input=owner.latest_device_observation(&session,&basis,9,&latest).unwrap().unwrap();
        assert_eq!(input.scan.source_sequence,7);assert_eq!(input.scan.timestamp_ns,1_100_000_000);
        assert_eq!(input.odom_from_base,[0.1,0.0,0.0]);assert_eq!(input.basis_revision,8);
        assert_eq!(basis.source_sequence,5);assert_eq!(basis.odom_from_base,[99.0;3]);
        assert!(owner.latest_device_observation(&session,&basis,10,&latest).unwrap().is_none());
        for fault in 0..4 {
            let invalid=Owner::default();invalid.capture_odometry(3,first.clone()).unwrap();invalid.capture_odometry(3,latest.clone()).unwrap();
            let mut changed=observation.clone();
            match fault {
                0=>changed.calibration_ref="其它标定".into(),
                1=>changed.timestamp_quality=3,
                2=>changed.beam_states_u8.fill(0),
                _=>changed.scan_duration_ns=200_000_000,
            }
            invalid.capture(3,9,changed);
            assert!(invalid.latest_device_observation(&session,&basis,9,&latest).unwrap().is_none(),"故障{fault}");
        }
        latest.timestamp_ns=1_600_000_000;
        assert!(owner.latest_device_observation(&session,&basis,9,&latest).unwrap().is_none());
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
