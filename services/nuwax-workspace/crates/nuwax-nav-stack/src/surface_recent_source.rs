//! Independent live obstacle evidence. Persistent terrain never renews this source.
use super::*;
use nuwax_nav_runtime::{CommandDispatchBound,ExecutionMap};
use nuwax_nav_runtime::surface_changes::SurfaceMapUpdate;

pub(crate) struct RecentSourceState {
    max_age:Option<Duration>,
    snapshot:Option<Arc<SurfaceSnapshot>>,
    watermark:Option<(i64,i64,i64)>,
    enabled:bool,
}
impl RecentSourceState {
    pub(crate) fn new(max_age:Option<Duration>)->Self {
        Self {max_age,snapshot:None,watermark:None,enabled:max_age.is_some()}
    }
    pub(crate) async fn reset(&mut self,scheduler:&nuwax_nav_runtime::NavSchedulerHandle)
        ->Result<(),nuwax_nav_runtime::RuntimeError> {
        self.snapshot=None;
        self.watermark=None;
        scheduler.update_obstacle_source(None,!self.enabled,None).await
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
        let authority=authority.ok_or("recent cloud has no admitted terrain domain")?;
        if packet.producer_runtime_generation!=authority.producer_runtime_generation
            || packet.session_id!=authority.committed_map.session_id
            || packet.session_generation!=authority.committed_map.session_generation
            || packet.map_frame_generation!=authority.map_frame_generation
            || packet.commit_boot_nonce!=authority.commit_boot_nonce {
            return Err("recent cloud belongs to a different navigation domain".into());
        }
        let Some(cloud)=packet.cloud else {
            self.enabled=true;
            let update=self.snapshot.as_ref().map(|basis| {
                let snapshot=Arc::new(basis.clear_recent_obstacles());
                SurfaceMapUpdate {snapshot,change_basis:Some(basis.clone()),changes:Arc::new(SurfaceGeometryChanges::Empty)}
            });
            if let Some(update)=&update {self.snapshot=Some(update.snapshot.clone());}
            scheduler.update_obstacle_source(update.map(ExecutionMap::SurfaceUpdate),false,None).await
                .map_err(|error|error.to_string())?;
            return Ok(());
        };
        if cloud.map_frame_id!=authority.committed_map.frame_id || cloud.source_timestamp_ns<=0 {
            return Err("recent cloud frame or source time is invalid".into());
        }
        if self.watermark.is_some_and(|(generation,sequence,time)|
            cloud.runtime_generation<generation || cloud.source_timestamp_ns<=time
            || (cloud.runtime_generation==generation && cloud.source_sequence<=sequence)) {
            return Err("recent cloud source replay rejected".into());
        }
        let basis=self.snapshot.as_ref().ok_or("recent cloud has no ready terrain")?;
        let source_time=cloud.source_timestamp_ns as u64;
        let bound=if let Some(age)=self.max_age {
            // Same absolute source clock qualification as the existing pose path;
            // receipt time is never used to rejuvenate a frozen/replayed scan.
            guarded_pose_source_deadline(cloud.source_timestamp_ns,age,std::time::SystemTime::now(),tokio::time::Instant::now())
                .map_err(|error|error.to_string())?;
            Some(CommandDispatchBound::from_source_timestamp(source_time,age))
        } else {None};
        let points=point_cloud_from_schema(cloud.cloud).map_err(|error|error.to_string())?;
        if points.points.len()!=cloud.emitted_point_count as usize
            || points.points.iter().any(|p|![p.x,p.y,p.z].into_iter().all(f64::is_finite)) {
            return Err("recent cloud contains invalid admitted points".into());
        }
        let retention=self.max_age.map_or(0,|age|age.as_nanos().min(u128::from(u64::MAX)) as u64);
        let cleared;
        let previous=if self.watermark.is_some_and(|(generation,_,_)|generation!=cloud.runtime_generation) {
            cleared=basis.clear_recent_obstacles(); &cleared
        } else {basis.as_ref()};
        let snapshot=Arc::new(previous.with_recent_obstacle_points(cloud.source_sequence as u64,
            source_time,retention,&packet.ground_point_indices,
            points.points.into_iter().enumerate().map(|(index,p)|(index as u32,[p.x,p.y,p.z])))
            .map_err(|error|error.to_string())?);
        scheduler.update_obstacle_source(Some(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
            snapshot:snapshot.clone(),change_basis:Some(basis.clone()),changes:Arc::new(SurfaceGeometryChanges::Empty),
        })),true,bound).await.map_err(|error|error.to_string())?;
        self.enabled=true;
        self.snapshot=Some(snapshot);
        self.watermark=Some((cloud.runtime_generation,cloud.source_sequence,cloud.source_timestamp_ns));
        Ok(())
    }
}

#[cfg(test)]
#[path = "recent_ingestion_diagnostics.rs"]
mod ingestion_diagnostics;

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_codec::lcm::{lio,recent_cloud};
    #[tokio::test]
    async fn recent_consumer_checks_domain_replay_and_never_uses_map_time() {
        let authority=GuardedMapAuthority {
            committed_map:schema::CommittedMapIdentity {
                session_id:"live-source-test".into(),session_generation:1,revision:1,
                commit_boot_nonce:[7;16],commit_sequence:1,source_timestamp_ns:1,frame_id:"map".into(),
            },producer_runtime_generation:17,map_frame_generation:1,cloud_revision:1,
            cloud_occupancy_revision:1,cloud_commit_sequence:1,cloud_source_timestamp_ns:1,commit_boot_nonce:[7;16],
        };
        let config=crate::surface_config::SurfaceNavigationConfig::new("map".into(),0.4,0.3,0.6,0.1,0.2,35.0,0.2).unwrap();
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
            producer_runtime_generation:17,session_id:"live-source-test".into(),session_generation:1,map_frame_generation:1,
            commit_boot_nonce:[7;16],ground_point_indices:vec![],cloud:Some(lio::LioCloudEnvelope {
                schema_version:1,backend:lio::LIO_BACKEND_DEVICE,runtime_generation:2,source_sequence:1,
                source_timestamp_ns:now,correlated_odometry_source_sequence:1,correlated_odometry_source_timestamp_ns:now,
                sensor_frame_id:"sensor".into(),body_frame_id:"body".into(),odom_frame_id:"odom".into(),map_frame_id:"map".into(),
                registration_state:lio::LIO_REGISTRATION_REGISTERED_MAP,source_point_count:1,emitted_point_count:1,cloud,
            }),
        };
        let mut state=RecentSourceState::new(Some(Duration::from_secs(5)));
        state.attach_terrain(map.clone());
        state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.unwrap();
        assert!(nuwax_nav_pct::same_source_geometry(&map,state.snapshot.as_ref().unwrap()));
        let accepted=state.watermark;
        assert!(state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.is_err());
        packet.cloud.as_mut().unwrap().source_sequence+=1;
        packet.cloud.as_mut().unwrap().source_timestamp_ns+=1;
        packet.cloud.as_mut().unwrap().cloud.header.seq+=1;
        let time=packet.cloud.as_ref().unwrap().source_timestamp_ns;
        packet.cloud.as_mut().unwrap().cloud.header.stamp_sec=(time/1_000_000_000) as i32;
        packet.cloud.as_mut().unwrap().cloud.header.stamp_nsec=(time%1_000_000_000) as i32;
        packet.commit_boot_nonce=[8;16];
        assert!(state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.is_err());
        assert_eq!(state.watermark,accepted);
        // Updating static terrain neither changes the live watermark nor its deadline.
        state.attach_terrain(map);
        assert_eq!(state.watermark,accepted);
        packet.commit_boot_nonce=[7;16];
        packet.cloud=None;
        state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.unwrap();
        assert!(!handle.status().planner_inputs_fresh);
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }
}
