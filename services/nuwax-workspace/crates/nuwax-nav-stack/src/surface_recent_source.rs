//! Independent live obstacle evidence. Persistent terrain never renews this source.
use super::*;
use nuwax_nav_runtime::{CommandDispatchBound,ExecutionMap,RegisteredFrame,RegisteredFrameDomain,RegisteredFrameIdentity};
use nuwax_nav_runtime::surface_changes::SurfaceMapUpdate;

pub(crate) struct RecentSourceState {
    max_age:Option<Duration>,
    snapshot:Option<Arc<SurfaceSnapshot>>,
    watermark:Option<(i64,i64,i64)>,
    registered_frame:Option<Arc<RegisteredFrame>>,
    enabled:bool,
}
impl RecentSourceState {
    pub(crate) fn new(max_age:Option<Duration>)->Self {
        Self {max_age,snapshot:None,watermark:None,registered_frame:None,enabled:max_age.is_some()}
    }
    pub(crate) async fn reset(&mut self,scheduler:&nuwax_nav_runtime::NavSchedulerHandle)
        ->Result<(),nuwax_nav_runtime::RuntimeError> {
        self.snapshot=None;
        self.watermark=None;
        self.registered_frame=None;
        scheduler.update_registered_obstacle_source(None,!self.enabled,None,None).await
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
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        let decode=scheduler.qualification_timing().map(|t|t.span("recent.codec_decode",0));
        let packet=nuwax_codec::lcm::recent_cloud::decode_body(body).map_err(|error|error.to_string())?;
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        drop(decode);
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        if let (Some(t),Some(cloud))=(scheduler.qualification_timing(),packet.cloud.as_ref()) {
            t.mark("recent.codec_decoded",cloud.source_timestamp_ns);
        }
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
            self.registered_frame=None;
            let update=self.snapshot.as_ref().map(|basis| {
                let snapshot=Arc::new(basis.clear_recent_obstacles());
                SurfaceMapUpdate {snapshot,change_basis:Some(basis.clone()),changes:Arc::new(SurfaceGeometryChanges::Empty)}
            });
            if let Some(update)=&update {self.snapshot=Some(update.snapshot.clone());}
            scheduler.update_registered_obstacle_source(update.map(ExecutionMap::SurfaceUpdate),false,None,None).await
                .map_err(|error|error.to_string())?;
            return Ok(());
        };
        if cloud.map_frame_id!=authority.committed_map.frame_id || cloud.source_timestamp_ns<=0 {
            return Err("recent cloud frame or source time is invalid".into());
        }
        let correction_changed=self.registered_frame.as_ref().zip(packet.registration.as_ref())
            .is_some_and(|(old,new)|old.identity.domain.registration_correction_revision!=new.registration_correction_revision);
        if self.watermark.is_some_and(|(generation,sequence,time)|
            cloud.runtime_generation<generation || cloud.source_timestamp_ns<=time
            || (cloud.runtime_generation==generation && cloud.source_sequence<=sequence)) {
            if correction_changed {
                // A correction does not turn an old acquisition into a new scan.
                // Revoke the obsolete map immediately, then wait for a real new frame.
                self.registered_frame=None;
                let update=self.snapshot.as_ref().map(|basis|SurfaceMapUpdate {
                    snapshot:Arc::new(basis.clear_recent_obstacles()),change_basis:Some(basis.clone()),
                    changes:Arc::new(SurfaceGeometryChanges::Empty),
                });
                if let Some(update)=&update {self.snapshot=Some(update.snapshot.clone());}
                scheduler.update_registered_obstacle_source(update.map(ExecutionMap::SurfaceUpdate),false,None,None)
                    .await.map_err(|error|error.to_string())?;
            }
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
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        if let Some(t)=scheduler.qualification_timing() {t.count("recent.emitted_points",cloud.source_timestamp_ns,cloud.emitted_point_count as u64);t.mark("recent.source_admitted",cloud.source_timestamp_ns);}
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        let xyz=scheduler.qualification_timing().map(|t|t.span("recent.xyz_materialize",cloud.source_timestamp_ns));
        let points=point_cloud_from_schema(cloud.cloud).map_err(|error|error.to_string())?;
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        drop(xyz);
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        let finite=scheduler.qualification_timing().map(|t|t.span("recent.finite_scan",cloud.source_timestamp_ns));
        if points.points.len()!=cloud.emitted_point_count as usize
            || points.points.iter().any(|p|![p.x,p.y,p.z].into_iter().all(f64::is_finite)) {
            return Err("recent cloud contains invalid admitted points".into());
        }
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        drop(finite);
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        let build=scheduler.qualification_timing().map(|t|t.span("recent.recent_frame_build",cloud.source_timestamp_ns));
        // Keep every admitted return (including ground) in its existing world axes.
        // The paired pose came from registration, never from current nav odometry.
        let raw_points:Arc<[[f64;3]]>=points.points.into_iter().map(|p|[p.x,p.y,p.z]).collect::<Vec<_>>().into();
        let registered_frame=packet.registration.map(|pose|Arc::new(RegisteredFrame {
            identity:RegisteredFrameIdentity {
                domain:RegisteredFrameDomain {
                    producer_runtime_generation:packet.producer_runtime_generation,
                    session_id:packet.session_id.clone(),session_generation:packet.session_generation,
                    map_frame_generation:packet.map_frame_generation,commit_boot_nonce:packet.commit_boot_nonce,
                    registration_correction_revision:pose.registration_correction_revision,
                    backend:cloud.backend,runtime_generation:cloud.runtime_generation,
                },sequence:cloud.source_sequence,timestamp_ns:cloud.source_timestamp_ns,
            },
            points_world:Arc::clone(&raw_points),sensor_origin_world:pose.sensor_origin_map,
            body_position_world:pose.body_position_map,body_orientation_xyzw:pose.body_orientation_xyzw,
            paired_odometry_sequence:cloud.correlated_odometry_source_sequence,
            paired_odometry_timestamp_ns:cloud.correlated_odometry_source_timestamp_ns,
            dispatch_bound:bound,
        }));
        let retention=self.max_age.map_or(0,|age|age.as_nanos().min(u128::from(u64::MAX)) as u64);
        let cleared;
        let previous=if correction_changed || self.watermark.is_some_and(|(generation,_,_)|generation!=cloud.runtime_generation) {
            cleared=basis.clear_recent_obstacles(); &cleared
        } else {basis.as_ref()};
        let snapshot=Arc::new(previous.with_recent_obstacle_points(cloud.source_sequence as u64,
            source_time,retention,&packet.ground_point_indices,
            raw_points.iter().copied().enumerate().map(|(index,p)|(index as u32,p)))
            .map_err(|error|error.to_string())?);
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        drop(build);
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        let ack=scheduler.qualification_timing().map(|t|t.span("recent.scheduler_ack",cloud.source_timestamp_ns));
        scheduler.update_registered_obstacle_source(Some(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {
            snapshot:snapshot.clone(),change_basis:Some(basis.clone()),changes:Arc::new(SurfaceGeometryChanges::Empty),
        })),true,bound,registered_frame.clone()).await.map_err(|error|error.to_string())?;
        #[cfg(all(test,feature="surface-recovery-qualification"))]
        drop(ack);
        self.enabled=true;
        self.snapshot=Some(snapshot);
        self.registered_frame=registered_frame;
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
            registration:None,
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
        assert!(nuwax_nav_runtime::native_map::same_source_geometry(&map,state.snapshot.as_ref().unwrap()));
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
    #[tokio::test]
    async fn paired_registered_scan_preserves_world_points_pose_and_time_without_latest_pose() {
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
            registration:Some(recent_cloud::RegisteredCloudPose {
                sensor_origin_map:[5.,7.,9.],body_position_map:[4.,6.,8.],
                body_orientation_xyzw:[0.,0.,std::f64::consts::FRAC_1_SQRT_2,std::f64::consts::FRAC_1_SQRT_2],
                registration_correction_revision:3,
            }),
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
        assert!(nuwax_nav_runtime::native_map::same_source_geometry(&map,state.snapshot.as_ref().unwrap()));
        let frame=state.registered_frame.as_ref().unwrap().clone();
        assert_eq!(frame.points_world.as_ref(),&[[0.5,1.0,0.5]]);
        assert_eq!(frame.sensor_origin_world,[5.,7.,9.]);
        assert_eq!(frame.body_position_world,[4.,6.,8.]);
        assert_eq!(frame.paired_odometry_timestamp_ns,now);
        assert_eq!(frame.identity.domain.producer_runtime_generation,17);
        assert_eq!(frame.identity.domain.runtime_generation,2);
        assert_eq!(frame.identity.domain.registration_correction_revision,3);
        assert!(frame.dispatch_bound.is_some());
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
        assert!(Arc::ptr_eq(state.registered_frame.as_ref().unwrap(),&frame));
        // Updating static terrain neither changes the live watermark nor its deadline.
        state.attach_terrain(map);
        assert_eq!(state.watermark,accepted);
        assert!(Arc::ptr_eq(state.registered_frame.as_ref().unwrap(),&frame));
        packet.commit_boot_nonce=[7;16];
        // A newer real acquisition may establish a new correction domain.
        packet.registration.as_mut().unwrap().registration_correction_revision=4;
        state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.unwrap();
        assert_eq!(state.registered_frame.as_ref().unwrap().identity.domain.registration_correction_revision,4);
        let accepted_new=state.watermark;
        // The same acquisition under another correction is not fresh. It must
        // revoke old native history while waiting for a genuinely new scan.
        packet.registration.as_mut().unwrap().registration_correction_revision=5;
        assert!(state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.is_err());
        assert_eq!(state.watermark,accepted_new);
        assert!(state.registered_frame.is_none());
        assert!(!handle.status().planner_inputs_fresh);
        packet.registration=None;
        packet.cloud=None;
        state.consume(&recent_cloud::encode_body(&packet).unwrap(),Some(&authority),&handle).await.unwrap();
        assert!(!handle.status().planner_inputs_fresh);
        assert!(state.registered_frame.is_none());
        state.reset(&handle).await.unwrap();
        assert!(state.watermark.is_none());
        handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    }
}
