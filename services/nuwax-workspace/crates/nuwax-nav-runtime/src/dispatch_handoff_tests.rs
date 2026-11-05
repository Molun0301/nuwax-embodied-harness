// Included inside lib.rs tests so the existing in-memory lease/fence fixtures
// remain the same. No sockets, devices, or optimistic APPLIED acknowledgements.
struct DispatchTestPort {
    inner:FakeMotion,
    expire_first:bool,
    fail_bounded:bool,
    bounded_attempts:usize,
    applied:Vec<Twist2>,
}
#[async_trait]
impl NavMotionPort for DispatchTestPort {
    async fn acquire(&mut self, generation:u64) -> Result<u64,MotionPortError> { self.inner.acquire(generation).await }
    async fn velocity(&mut self, command:Twist2) -> Result<(),MotionPortError> {
        self.inner.velocity(command).await?; self.applied.push(command); Ok(())
    }
    async fn velocity_bounded(&mut self, command:Twist2, bound:Option<CommandDispatchBound>) -> Result<BoundedTwistOutcome,MotionPortError> {
        if let Some(bound)=bound {
            self.bounded_attempts+=1;
            if self.fail_bounded { return Err(MotionPortError::new("注入传输失败")); }
            if self.expire_first || bound.expired_now() { self.expire_first=false; return Ok(BoundedTwistOutcome::ExpiredNotApplied); }
        }
        self.velocity(command).await?; Ok(BoundedTwistOutcome::Applied)
    }
    async fn release(&mut self) -> Result<(),MotionPortError> { self.inner.release().await }

}
fn handoff_bound(ns:u64)->CommandDispatchBound {
    let anchor=crate::command_dispatch::ObservationClockAnchor::capture();
    CommandDispatchBound::from_anchor(anchor,anchor.unix_ns+ns)
}
fn dispatch_scheduler()->NavScheduler<DispatchTestPort,FakePlanner> {
    let port=DispatchTestPort{inner:FakeMotion(Arc::new(Mutex::new(FakeMotionState::default()))),
        expire_first:false,fail_bounded:false,bounded_attempts:0,applied:Vec::new()};
    let mut scheduler=NavScheduler::new(port,FakePlanner::commanding(),SchedulerConfig::default()).unwrap();
    scheduler.session=Some(session());scheduler.lease_active=true;scheduler.status.phase=SchedulerPhase::Controlling;
    scheduler
}


#[tokio::test]
async fn dispatch_receiver_expiry_applies_exactly_one_zero_and_fresh_command_recovers() {
    let mut scheduler=dispatch_scheduler();scheduler.motion.expire_first=true;
    let (status,_)=watch::channel(scheduler.status.clone());
    let command=Twist2{linear:Point2::new(0.1,0.0),angular_z:0.0};
    assert!(!scheduler.apply_controller_twist_with_bound(&status,command,Some(handoff_bound(1_000_000_000))).await.unwrap());
    assert_eq!(scheduler.motion.bounded_attempts,1);assert_eq!(scheduler.motion.applied,vec![Twist2::ZERO]);
    assert!(scheduler.apply_controller_twist_with_bound(&status,command,Some(handoff_bound(1_000_000_000))).await.unwrap());
    assert_eq!(scheduler.motion.bounded_attempts,2);assert_eq!(scheduler.motion.applied,vec![Twist2::ZERO,command]);
}
#[tokio::test]
async fn dispatch_ambiguous_failure_keeps_fail_closed_fence() {
    let mut scheduler=dispatch_scheduler();scheduler.motion.fail_bounded=true;
    let (status,_)=watch::channel(scheduler.status.clone());
    let command=Twist2{linear:Point2::new(0.1,0.0),angular_z:0.0};
    assert!(scheduler.apply_controller_twist_with_bound(&status,command,Some(handoff_bound(1_000_000_000))).await.is_err());
    assert!(scheduler.motion.applied.is_empty());
    assert!(scheduler.motion.inner.0.lock().unwrap().calls.iter().any(|call|call.starts_with("release:")));
}

pub(super) async fn assert_real_source_bound_reaches_scheduler(
    worker:planner_worker::PlannerWorker, step:PlannerStep, source_timestamp_ns:u64, max_age:Duration,
) {
    let template=dispatch_scheduler();
    let mut scheduler=NavScheduler::new(template.motion,worker,SchedulerConfig::default()).unwrap();
    scheduler.session=Some(session());scheduler.lease_active=true;
    scheduler.current_execution_identity=Some(Arc::new(execution_identity_for("goal-1",1)));
    let intent=scheduler.wrap_local_intent(step).unwrap();
    assert!(intent.dispatch_bound.is_none(), "static ground must not issue a per-cell execution lease");
    let bound=CommandDispatchBound::from_source_timestamp(source_timestamp_ns,max_age);
    assert!(!bound.expired_now(), "actual sensor source must still be healthy");
    scheduler.obstacle_source_bound=Some(bound);
    let LocalExecutionPayload::Command(command)=intent.payload else {panic!("expected real command");};
    assert_ne!(command,Twist2::ZERO);
    let (status,_)=watch::channel(scheduler.status.clone());
    assert!(scheduler.apply_controller_twist_with_bound(&status,command,intent.dispatch_bound).await.unwrap());
    assert_eq!(scheduler.motion.bounded_attempts,1);
    assert_eq!(scheduler.motion.applied,vec![command]);
    scheduler.planner.stop();
}

struct DispatchPlanner {inner:FakePlanner,bound:Option<CommandDispatchBound>}
impl NavPlanner for DispatchPlanner {
    fn replace_path(&mut self,path:Path2D) {self.inner.replace_path(path);}
    fn update_odometry(&mut self,pose:PoseStamped) {self.inner.update_odometry(pose);}
    fn update_costmap(&mut self,map:CostMap2D) {self.inner.update_costmap(map);}
    fn step(&mut self)->PlannerStep {self.bound=Some(handoff_bound(1_000_000_000));self.inner.step()}
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound> {self.bound.take()}
    fn stop(&mut self) {self.bound=None;self.inner.stop();}
}
#[tokio::test]
async fn dispatch_control_tick_expiry_preserves_wait_goal_route_until_recovery_or_timeout() {
    for recover in [false,true] {
        let template=dispatch_scheduler();
        let mut scheduler=NavScheduler::new(template.motion,DispatchPlanner{inner:FakePlanner::commanding(),bound:None},
            SchedulerConfig{path_wait_timeout:Duration::from_secs(2),..SchedulerConfig::default()}).unwrap();
        let (status,_)=watch::channel(scheduler.status.clone());
        scheduler.observe_odometry(&status,map_pose(),None).unwrap();
        scheduler.observe_costmap(&status,costmap_at(1_000_000_000),Instant::now()).unwrap();
        scheduler.start_session(&status,session()).await.unwrap();
        let original_wait=scheduler.path_wait_started_at;
        scheduler.submit_path(&status,path()).await.unwrap();
        let identity=scheduler.current_execution_identity.clone();let route=scheduler.current_path.clone();
        for _ in 0..3 {
            scheduler.motion.expire_first=true;
            scheduler.control_tick(&status).await.unwrap();
            assert_eq!(scheduler.path_wait_started_at,original_wait);
            assert_eq!(scheduler.current_execution_identity,identity);assert_eq!(scheduler.current_path,route);
            assert!(scheduler.session.is_some());assert_eq!(scheduler.status.phase,SchedulerPhase::Controlling);
            assert_eq!(scheduler.status.last_applied_twist,Some(Twist2::ZERO));
        }
        assert_eq!(scheduler.motion.applied,vec![Twist2::ZERO;3]);
        if recover {
            scheduler.control_tick(&status).await.unwrap();
            assert!(scheduler.path_wait_started_at.is_none());
            assert_ne!(scheduler.status.last_applied_twist,Some(Twist2::ZERO));
            assert_eq!(scheduler.current_execution_identity,identity);assert_eq!(scheduler.current_path,route);
        }else{
            scheduler.path_wait_started_at=Some(Instant::now()-Duration::from_secs(3));
            scheduler.motion.expire_first=true;scheduler.control_tick(&status).await.unwrap();
            assert_eq!(scheduler.status.last_terminal,Some(TerminalOutcome::PlannerFailed));

        }
    }
}
