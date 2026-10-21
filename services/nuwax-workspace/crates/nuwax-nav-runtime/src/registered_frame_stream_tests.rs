// Production-mode SurfaceFollower + real asynchronous scheduler/worker. Only the
// motion-port acknowledgements and stationary sensor inputs are synthetic. This
// is not a hardware or physics test, and static-map fixture mode stays disabled.
#[derive(Clone)]
struct RegisteredStreamMotion {
    inner:FakeMotion,
    measured:Arc<Mutex<MeasuredMotion>>,
    issued:Arc<Mutex<Vec<(std::time::Instant,Twist2)>>>,
}
#[async_trait]
impl NavMotionPort for RegisteredStreamMotion {
    fn measured_motion(&self)->Option<MeasuredMotion> {Some(self.measured.lock().unwrap().clone())}
    async fn acquire(&mut self,id:&str,a:&NavMotionAdmission)->Result<LeaseProof,MotionPortError> {self.inner.acquire(id,a).await}
    async fn renew(&mut self,id:&str)->Result<(),MotionPortError> {self.inner.renew(id).await}
    async fn submit_twist(&mut self,id:&str,c:Twist2)->Result<AppliedProof,MotionPortError> {
        let proof=self.inner.submit_twist(id,c).await?;
        self.issued.lock().unwrap().push((std::time::Instant::now(),c));Ok(proof)
    }
    async fn submit_twist_bounded(&mut self,id:&str,c:Twist2,b:Option<CommandDispatchBound>)->Result<BoundedTwistOutcome,MotionPortError> {
        if b.is_some_and(CommandDispatchBound::expired_now) {return Ok(BoundedTwistOutcome::ExpiredNotApplied);}
        self.submit_twist(id,c).await.map(BoundedTwistOutcome::Applied)
    }
    async fn revoke_and_zero(&mut self,id:&str)->Result<FenceProof,MotionPortError> {
        let proof=self.inner.revoke_and_zero(id).await?;
        self.issued.lock().unwrap().push((std::time::Instant::now(),Twist2::ZERO));Ok(proof)
    }
}
#[tokio::test]
async fn author_a_production_registered_stream_20hz_publishes_and_revocation_stops() {
    registered_stream_diagnostic(false).await;
}
#[tokio::test]
async fn author_a_occlusion_consistent_wall_stream_diagnostic() {
    registered_stream_diagnostic(true).await;
}
fn behind_stream_wall(point:[f64;3])->bool {
    // Segment from the same-frame sensor origin (.5,1,.5) to the return.
    if point[0]<=1. {return false;}
    let t=0.5/(point[0]-0.5);
    let y=1.+t*(point[1]-1.);let z=0.5+t*(point[2]-0.5);
    (0.0..=1.9).contains(&y)&&(0.1..=1.1).contains(&z)
}
async fn registered_stream_diagnostic(occlusion_consistent:bool) {
    let (config,base)=crate::native_test_helpers::fixture();
    let follower=SurfaceFollower::new(config).unwrap(); // production raw-frame mode
    let worker=planner_worker::PlannerWorker::new(follower).unwrap();
    let measured=Arc::new(Mutex::new(crate::motion_feedback::tests::measured([0.;3],[0.;3])));
    let issued=Arc::new(Mutex::new(Vec::new()));
    let port=RegisteredStreamMotion {inner:FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
        measured:measured.clone(),issued:issued.clone()};
    let scheduler=NavScheduler::new(port,worker,SchedulerConfig::default()).unwrap();
    let (handle,task)=spawn_scheduler(scheduler);
    let mut pose=map_pose();pose.pose.position=nuwax_nav_types_messages::Point3::new(0.5,1.,0.5);
    pose.stamp.nanos=crate::command_dispatch::unix_ns().unwrap() as i64;
    handle.update_odometry(pose.clone()).await.unwrap();
    handle.update_costmap(CostmapObservation{costmap:ExecutionMap::Surface(base.clone()),source_timestamp_ns:pose.stamp.nanos}).await.unwrap();
    let mut goal=pose.clone();goal.pose.position.x=3.;
    let mut request=session();request.goal=ExecutionGoal::Body(goal.clone());
    handle.start_session(request).await.unwrap();
    let mut route=path();route.path=ExecutionPath::Surface(Path3D{poses:vec![pose.clone(),goal]});
    handle.submit_path(route).await.unwrap();
    let ground:Vec<[f64;3]>=(0..65).flat_map(|ix|(0..35).map(move|iy|[(ix as f64+0.5)*0.1,(iy as f64+0.5)*0.1,0.05])).collect();
    let mut snapshot=base;
    let began=std::time::Instant::now();
    let mut timeline=Vec::new();
    for tick in 0..60_i64 {
        tokio::time::sleep_until(tokio::time::Instant::from_std(began+Duration::from_millis(tick as u64*50))).await;
        let stamp=crate::command_dispatch::unix_ns().unwrap();
        pose.stamp.nanos=stamp as i64;
        let mut observation=crate::motion_feedback::tests::measured([0.;3],[0.;3]);
        observation.source_timestamp_ns=stamp;
        observation.valid_until_monotonic_ns=observation.received_monotonic_ns+250_000_000;
        *measured.lock().unwrap()=observation;
        handle.update_odometry(pose.clone()).await.unwrap();
        let mut points:Vec<_>=ground.iter().copied().filter(|point|
            !(occlusion_consistent && tick>=40 && behind_stream_wall(*point))).collect();
        let labels:Vec<u32>=(0..points.len() as u32).collect();
        if occlusion_consistent && tick>=40 {
            assert!(points.iter().all(|point|!behind_stream_wall(*point)));
            assert!(points.len()<ground.len(),"the physical wall occludes distant ground returns");
        }
        // Original liveness fixture retains its historical returns. The separate
        // visibility-consistent diagnostic removes hidden returns, never writes
        // occupancy labels, and does not claim that a nonzero response is safe.
        if tick>=40 {
            for iy in 0..20 {for iz in 1..12 {points.push([1.,iy as f64*0.1,iz as f64*0.1]);}}
        }
        let next=Arc::new(snapshot.with_recent_obstacle_points(tick as u64+1,stamp,250_000_000,&labels,
            points.iter().copied().enumerate().map(|(i,p)|(i as u32,p))).unwrap());
        let bound=CommandDispatchBound::from_source_timestamp(stamp,Duration::from_millis(250));
        let mut frame=(*crate::registered_frame::test_frame(tick+1)).clone();
        frame.identity.timestamp_ns=stamp as i64;frame.points_world=points.into();
        frame.paired_odometry_timestamp_ns=stamp as i64;frame.paired_odometry_sequence=tick+1;
        frame.dispatch_bound=Some(bound);
        handle.update_registered_obstacle_source(Some(ExecutionMap::SurfaceUpdate(crate::surface_changes::SurfaceMapUpdate {
            snapshot:next.clone(),change_basis:Some(snapshot),changes:Arc::new(crate::surface_changes::SurfaceGeometryChanges::Empty),
        })),true,Some(bound),Some(Arc::new(frame))).await.unwrap();
        snapshot=next;
        let status=handle.status();
        timeline.push((tick,status.phase,status.last_applied_twist,status.message));
        if status.last_terminal.is_some() {break;}
    }
    handle.update_registered_obstacle_source(None,false,None,None).await.unwrap();
    let terminal=wait_for_terminal(&handle,TerminalOutcome::SafetyFailed).await;
    assert_eq!(terminal.last_applied_twist,Some(Twist2::ZERO));
    assert!(terminal.last_zero_ack_id.is_some());
    handle.shutdown().await.unwrap();task.await.unwrap().unwrap();
    let samples=issued.lock().unwrap();
    let positive:Vec<_>=samples.iter().filter(|(_,c)|*c!=Twist2::ZERO).collect();
    let first=positive.first().map(|(t,_)|t.duration_since(began).as_secs_f64());
    let max_gap=positive.windows(2).map(|v|v[1].0.duration_since(v[0].0).as_secs_f64()).fold(0.,f64::max);
    println!("REGISTERED_STREAM first_nonzero_sec={first:?} nonzero={} total={} max_gap_sec={max_gap:.6} synthetic_stationary_measurement=true occlusion_consistent={occlusion_consistent}",positive.len(),samples.len());
    for (tick,phase,command,reason) in timeline {println!("REGISTERED_STREAM tick={tick} phase={phase:?} applied={command:?} reason={reason}");}
    assert!(!positive.is_empty(),"连续原帧及同状态测量流下未发布过原A非零命令");
}
