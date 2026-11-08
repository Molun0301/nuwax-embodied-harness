//! Independent live obstacle evidence. Persistent terrain never renews this source.
use super::*;
use nuwax_nav_runtime::{ExecutionMap,RegisteredFrame,RegisteredFrameDomain,RegisteredFrameIdentity};
use nuwax_nav_runtime::surface_changes::SurfaceMapUpdate;

pub(crate) struct RecentSourceState {
    max_age:Option<Duration>,
    snapshot:Option<Arc<SurfaceSnapshot>>,
    latest:Option<(String,i64)>,
    registered_frame:Option<Arc<RegisteredFrame>>,
    enabled:bool,
}
impl RecentSourceState {
    pub(crate) fn new(max_age:Option<Duration>)->Self {
        Self {max_age,snapshot:None,latest:None,registered_frame:None,enabled:max_age.is_some()}
    }
    pub(crate) async fn reset(&mut self,scheduler:&nuwax_nav_runtime::NavSchedulerHandle)
        ->Result<(),nuwax_nav_runtime::RuntimeError> {
        self.snapshot=None;
        self.latest=None;
        self.registered_frame=None;
        scheduler.update_registered_obstacle_source(None,!self.enabled,None).await
    }
    pub(crate) fn attach_terrain(&mut self,map:Arc<SurfaceSnapshot>)->Arc<SurfaceSnapshot> {
        let map=if let Some(previous)=&self.snapshot {
            Arc::new(map.with_recent_obstacles_from(previous))
        } else {map};
        self.snapshot=Some(map.clone());
        map
    }
    pub(crate) async fn consume(&mut self,body:&[u8],authority:Option<&GuardedMapAuthority>,
        scheduler:&nuwax_nav_runtime::NavSchedulerHandle)->Result<(),String> {
        let packet=nuwax_codec::lcm::recent_cloud::decode_body(body).map_err(|error|error.to_string())?;
        let session_changed = self.latest.as_ref().is_some_and(|(session, _)| *session != packet.session_id);
        if packet.cloud.is_none() || session_changed {
            self.clear_observations(scheduler).await?;
        }
        let Some(cloud) = packet.cloud else { return Ok(()); };
        if self.latest.as_ref().is_some_and(|(_, sequence)| packet.sequence <= *sequence) {
            return Ok(());
        }
        let authority = authority.ok_or("recent cloud has no admitted terrain domain")?;
        if packet.session_id != authority.session_id || cloud.header.frame_id != authority.frame_id {
            return Err("recent cloud belongs to a different map session or frame".into());
        }
        let basis = self.snapshot.as_ref().ok_or("recent cloud has no ready terrain")?;
        let source_time = i64::from(cloud.header.stamp_sec) * 1_000_000_000
            + i64::from(cloud.header.stamp_nsec);
        let points = point_cloud_from_schema(cloud).map_err(|error| error.to_string())?;
        let raw_points: Arc<[[f64; 3]]> = points.points.into_iter()
            .map(|p| [p.x, p.y, p.z]).collect::<Vec<_>>().into();
        let registered_frame = packet.registration.map(|pose| Arc::new(RegisteredFrame {
            identity: RegisteredFrameIdentity {
                domain: RegisteredFrameDomain { session_id: packet.session_id.clone() },
                sequence: packet.sequence,
                timestamp_ns: source_time,
            },
            points_world: Arc::clone(&raw_points),
            sensor_origin_world: pose.sensor_origin_map,
            ground_point_indices: packet.ground_point_indices.clone().into(),
            body_position_world: pose.body_position_map,
            body_orientation_xyzw: pose.body_orientation_xyzw,
            paired_odometry_timestamp_ns: pose.paired_odometry_timestamp_ns,
        }));
        let retention = self.max_age.map_or(0, |age| age.as_nanos().min(u128::from(u64::MAX)) as u64);
        let snapshot = Arc::new(basis.with_recent_obstacle_points(
            packet.sequence as u64, source_time as u64, retention, &packet.ground_point_indices,
            raw_points.iter().copied().enumerate().map(|(index, p)| (index as u32, p)),
        ).map_err(|error| error.to_string())?);
        scheduler.update_registered_obstacle_source(Some(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
            snapshot:snapshot.clone(),change_basis:Some(basis.clone()),changes:Arc::new(SurfaceGeometryChanges::Empty),
        })),true,registered_frame.clone()).await.map_err(|error|error.to_string())?;
        self.enabled=true;
        self.snapshot=Some(snapshot);
        self.registered_frame=registered_frame;
        self.latest = Some((packet.session_id, packet.sequence));
        Ok(())
    }

    async fn clear_observations(&mut self, scheduler: &nuwax_nav_runtime::NavSchedulerHandle) -> Result<(), String> {
        self.latest = None;
        self.enabled = true;
        self.registered_frame = None;
        let update = self.snapshot.as_ref().map(|basis| SurfaceMapUpdate {
            snapshot: Arc::new(basis.clear_recent_obstacles()),
            change_basis: Some(basis.clone()),
            changes: Arc::new(SurfaceGeometryChanges::Empty),
        });
        if let Some(update) = &update {
            self.snapshot = Some(update.snapshot.clone());
        }
        scheduler.update_registered_obstacle_source(update.map(ExecutionMap::SurfaceUpdate), false, None)
            .await.map_err(|error| error.to_string())
    }
}

#[cfg(test)]
#[path = "recent_ingestion_diagnostics.rs"]
mod ingestion_diagnostics;

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_codec::lcm::recent_cloud;
    #[tokio::test]
    async fn recent_consumer_ignores_old_sequence_and_clears_on_session_change() {
        let authority=GuardedMapAuthority { session_id: "live-source-test".into(), revision: 1, frame_id: "map".into() };
        let config=crate::surface_config::SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.2, 35.0, 0.2).unwrap();
        let points:Vec<_>=(0..40).flat_map(|x|(0..30).map(move|y|((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
        let map=Arc::new(SurfaceSnapshot::from_points(&points,config.geometry,0.1).unwrap());
        let planner=nuwax_nav_runtime::SurfaceFollower::new(config.follower).unwrap();
        let scheduler=nuwax_nav_runtime::NavScheduler::new(
            crate::lcm_dataflow::guarded_tests::RecordingMotion(Arc::new(Mutex::new(Vec::new()))),
            planner,nuwax_nav_runtime::SchedulerConfig::default()).unwrap();
        let (handle,task)=nuwax_nav_runtime::spawn_scheduler(scheduler);
        let now=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as i64;
        let cloud=schema::PointCloud2 {
            header:schema::Header{seq:1,stamp_sec:(now/1_000_000_000) as i32,stamp_nsec:(now%1_000_000_000) as i32,frame_id:"map".into()},
            height:1,width:1,fields:["x","y","z"].into_iter().enumerate().map(|(i,name)|schema::PointField {
                name:name.into(),offset:(i*4) as i32,datatype:7,count:1,
            }).collect(),is_bigendian:false,point_step:12,row_step:12,
            data:[0.5_f32,1.0,0.5].into_iter().flat_map(f32::to_le_bytes).collect(),is_dense:true,
        };
        let mut packet=recent_cloud::RecentCloud3D {
            registration:None,
            session_id:"live-source-test".into(),ground_point_indices:vec![],sequence: 1, cloud: Some(cloud),
        };
        let mut state=RecentSourceState::new(Some(Duration::from_secs(5)));
        state.attach_terrain(map.clone());
        state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.unwrap();
        assert!(nuwax_nav_runtime::native_map::same_source_geometry(&map,state.snapshot.as_ref().unwrap()));
        let accepted = state.latest.clone();
        state.consume(&recent_cloud::encode_body(&packet).unwrap(), Some(&authority), &handle).await.unwrap();
        state.attach_terrain(map);
        assert_eq!(state.latest, accepted);
        let next = GuardedMapAuthority { session_id: "新会话".into(), ..authority };
        packet.session_id = next.session_id.clone();
        state.consume(&recent_cloud::encode_body(&packet).unwrap(), Some(&next), &handle).await.unwrap();
        assert_eq!(state.latest, Some((next.session_id.clone(), 1)));
        packet.cloud = None;
        state.consume(&recent_cloud::encode_body(&packet).unwrap(), Some(&next), &handle).await.unwrap();
        assert!(state.latest.is_none());
        assert!(!handle.status().planner_inputs_fresh);
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }
    #[tokio::test]
    async fn paired_registered_scan_preserves_world_points_pose_and_time_without_latest_pose() {
        let authority=GuardedMapAuthority { session_id: "live-source-test".into(), revision: 1, frame_id: "map".into() };
        let config=crate::surface_config::SurfaceNavigationConfig::new("map".into(), nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.3)/2., -(0.3)/2.], [(0.3)/2., -(0.3)/2.], [(0.3)/2., (0.3)/2.], [-(0.3)/2., (0.3)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }, 0.1, 0.2, 35.0, 0.2).unwrap();
        let points:Vec<_>=(0..40).flat_map(|x|(0..30).map(move|y|((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
        let map=Arc::new(SurfaceSnapshot::from_points(&points,config.geometry,0.1).unwrap());
        let planner=nuwax_nav_runtime::SurfaceFollower::new(config.follower).unwrap();
        let scheduler=nuwax_nav_runtime::NavScheduler::new(
            crate::lcm_dataflow::guarded_tests::RecordingMotion(Arc::new(Mutex::new(Vec::new()))),
            planner,nuwax_nav_runtime::SchedulerConfig::default()).unwrap();
        let (handle,task)=nuwax_nav_runtime::spawn_scheduler(scheduler);
        let now=std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as i64;
        let cloud=schema::PointCloud2 {
            header:schema::Header{seq:1,stamp_sec:(now/1_000_000_000) as i32,stamp_nsec:(now%1_000_000_000) as i32,frame_id:"map".into()},
            height:1,width:1,fields:["x","y","z"].into_iter().enumerate().map(|(i,name)|schema::PointField {
                name:name.into(),offset:(i*4) as i32,datatype:7,count:1,
            }).collect(),is_bigendian:false,point_step:12,row_step:12,
            data:[0.5_f32,1.0,0.5].into_iter().flat_map(f32::to_le_bytes).collect(),is_dense:true,
        };
        let mut packet=recent_cloud::RecentCloud3D {
            registration:Some(recent_cloud::RegisteredCloudPose {
                sensor_origin_map:[5.,7.,9.],body_position_map:[4.,6.,8.],
                body_orientation_xyzw:[0.,0.,std::f64::consts::FRAC_1_SQRT_2,std::f64::consts::FRAC_1_SQRT_2],
                paired_odometry_timestamp_ns:now,
            }),
            session_id:"live-source-test".into(),ground_point_indices:vec![],sequence: 1, cloud: Some(cloud),
        };
        let mut state=RecentSourceState::new(Some(Duration::from_secs(5)));
        state.attach_terrain(map.clone());
        state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.unwrap();
        assert!(nuwax_nav_runtime::native_map::same_source_geometry(&map,state.snapshot.as_ref().unwrap()));
        let frame=state.registered_frame.as_ref().unwrap().clone();
        assert_eq!(frame.points_world.as_ref(),&[[0.5,1.0,0.5]]);
        assert!(frame.ground_point_indices.is_empty());
        assert_eq!(frame.sensor_origin_world,[5.,7.,9.]);
        assert_eq!(frame.body_position_world,[4.,6.,8.]);
        assert_eq!(frame.paired_odometry_timestamp_ns,now);
        assert_eq!(frame.identity.domain.session_id, authority.session_id);
        state.consume(&recent_cloud::encode_body(&packet).unwrap(), Some(&authority), &handle).await.unwrap();
        assert!(Arc::ptr_eq(state.registered_frame.as_ref().unwrap(), &frame));
        state.attach_terrain(map);
        assert!(Arc::ptr_eq(state.registered_frame.as_ref().unwrap(), &frame));

        packet.sequence += 1;
        packet.ground_point_indices = vec![0];
        state.consume(&recent_cloud::encode_body(&packet).unwrap(), Some(&authority), &handle).await.unwrap();
        assert_eq!(state.registered_frame.as_ref().unwrap().ground_point_indices.as_ref(), &[0]);
        packet.registration = None;
        packet.cloud = None;
        packet.ground_point_indices.clear();
        state.consume(&recent_cloud::encode_body(&packet).unwrap(), Some(&authority), &handle).await.unwrap();
        assert!(state.registered_frame.is_none());
        assert!(!handle.status().planner_inputs_fresh);
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }
}
