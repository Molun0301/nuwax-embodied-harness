// Real strong controller on the existing worker, with only initial publication gated.
#[derive(Default)]
struct RecentRecheckCounts {full:usize,rechecks:usize,recheck_us:Vec<u128>}
struct RecentRecheckProbe {
    inner:crate::SurfaceFollower,
    counts:Arc<Mutex<RecentRecheckCounts>>,
    first:Option<(std::sync::mpsc::Sender<PlannerStep>,std::sync::mpsc::Receiver<()>)>,
}
impl NavPlanner for RecentRecheckProbe {
    fn configure_goal(&mut self,p:&PoseStamped,o:GoalOptions)->Result<(),RuntimeError>{self.inner.configure_goal(p,o)}
    fn configure_boundary(&mut self,b:&[[f64;2]]){self.inner.configure_boundary(b)}
    fn replace_path(&mut self,p:Path2D){self.inner.replace_path(p)}
    fn replace_execution_path(&mut self,p:ExecutionPath,y:Option<Vec<f64>>,g:PoseStamped)->Result<(),RuntimeError>{self.inner.replace_execution_path(p,y,g)}
    fn update_odometry(&mut self,p:PoseStamped){self.inner.update_odometry(p)}
    fn update_costmap(&mut self,m:CostMap2D){self.inner.update_costmap(m)}
    fn update_execution_map(&mut self,m:ExecutionMap)->Result<(),RuntimeError>{self.inner.update_execution_map(m)}
    fn observe_applied_command(&mut self,c:Twist2){self.inner.observe_applied_command(c)}
    fn step(&mut self,w:f64)->PlannerStep {
        self.counts.lock().unwrap().full+=1;
        let result=self.inner.step(w);
        if let Some((tx,rx))=self.first.take(){tx.send(result.clone()).unwrap();let _=rx.recv_timeout(Duration::from_secs(2));}
        result
    }
    fn revalidate_selected_command(&mut self,c:Twist2)->Option<PlannerStep>{
        let start=std::time::Instant::now();let result=self.inner.revalidate_selected_command(c);
        let mut counts=self.counts.lock().unwrap();counts.rechecks+=1;counts.recheck_us.push(start.elapsed().as_micros());result
    }
    fn revalidate_selected_command_at_current_state(&mut self,c:Twist2)->Option<PlannerStep>{
        let start=std::time::Instant::now();let result=self.inner.revalidate_selected_command_at_current_state(c);
        let mut counts=self.counts.lock().unwrap();counts.rechecks+=1;counts.recheck_us.push(start.elapsed().as_micros());result
    }
    fn take_command_dispatch_bound(&mut self)->Option<CommandDispatchBound>{self.inner.take_command_dispatch_bound()}
    fn stop(&mut self){self.inner.stop()}
}
fn recent_recheck_fixture()->(PlannerWorker,Arc<SurfaceSnapshot>,Arc<Mutex<RecentRecheckCounts>>,
    std::sync::mpsc::Receiver<PlannerStep>,std::sync::mpsc::Sender<()>) {
    let map=Arc::new(crate::surface_follower::test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope {
        footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0,
    }).unwrap());
    let mut config=crate::SurfaceFollowerConfig::new(0.4);
    config.motion_limits=Some(nuwax_nav_types_messages::MotionLimits {
        model:MotionModel::Differential,max_forward_mps:0.5,max_reverse_mps:0.0,max_lateral_mps:0.0,
        max_angular_rps:0.8,linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
        angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
        minimum_turning_radius_m:0.0,can_rotate_in_place:true,
    });
    let counts=Arc::new(Mutex::new(RecentRecheckCounts::default()));
    let (tx,rx)=std::sync::mpsc::channel();let(release,held)=std::sync::mpsc::channel();
    let mut worker=PlannerWorker::new(RecentRecheckProbe {inner:crate::SurfaceFollower::new(config).unwrap(),
        counts:counts.clone(),first:Some((tx,held))}).unwrap();
    let start=route_pose(0.5,0);let goal=route_pose(3.0,0);
    worker.configure_goal(&goal,GoalOptions::default()).unwrap();
    worker.update_execution_map(ExecutionMap::Surface(map.clone())).unwrap();worker.update_odometry(start.clone());
    worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![start,goal.clone()]}),None,goal).unwrap();
    worker.step(0.3);
    (worker,map,counts,rx,release)
}
fn publish_recent_for_recheck(worker:&mut PlannerWorker,previous:&Arc<SurfaceSnapshot>,sequence:u64,point:[f64;3])->Arc<SurfaceSnapshot> {
    let next=Arc::new(previous.with_recent_obstacle_points(sequence,sequence*1_000_000_000,0,&[0],[(0,point)]).unwrap());
    worker.update_execution_map(ExecutionMap::SurfaceUpdate(surface_changes::SurfaceMapUpdate {planar_reference:None,
        snapshot:next.clone(),change_basis:Some(previous.clone()),changes:Arc::new(surface_changes::SurfaceGeometryChanges::Empty),
    })).unwrap();next
}
fn wait_recent_rechecked(worker:&PlannerWorker) {
    let deadline=std::time::Instant::now()+Duration::from_secs(2);
    loop {let ready={let input=worker.shared.0.lock().unwrap();input.result.is_some()&&input.result_map_version==input.map_version};
        if ready{return;} assert!(std::time::Instant::now()<deadline,"latest selected trajectory was not rechecked");
        std::thread::sleep(Duration::from_millis(1));}
}
#[test]
fn latest_recent_rechecks_inflight_and_published_selection_without_reranking_or_renewing_deadline() {
    let(mut worker,mut map,counts,computed,release)=recent_recheck_fixture();
    assert!(matches!(computed.recv_timeout(Duration::from_secs(2)).unwrap(),PlannerStep::Command(c) if c!=Twist2::ZERO));
    map=publish_recent_for_recheck(&mut worker,&map,1,[10.0,10.0,0.5]);
    release.send(()).unwrap();wait_recent_rechecked(&worker);
    let original=worker.shared.0.lock().unwrap().result.as_ref().unwrap().0;
    for sequence in 2..5 {
        std::thread::sleep(Duration::from_millis(20));
        map=publish_recent_for_recheck(&mut worker,&map,sequence,[10.0+sequence as f64*0.0001,10.0,0.5]);
        wait_recent_rechecked(&worker);
        assert_eq!(worker.shared.0.lock().unwrap().result.as_ref().unwrap().0,original);
    }
    let c=counts.lock().unwrap();assert_eq!(c.full,1);assert!(c.rechecks>=4);
    eprintln!("RECENT_FINAL_RECHECK full={} rechecks={} microseconds={:?}",c.full,c.rechecks,c.recheck_us);drop(c);
    assert!(matches!(worker.step(0.3),PlannerStep::Command(c) if c!=Twist2::ZERO));worker.stop();
}
#[test]
fn latest_recent_obstacle_rejects_inflight_and_already_published_nonzero_command() {
    for published in [false,true] {
        let(mut worker,map,counts,computed,release)=recent_recheck_fixture();
        assert!(matches!(computed.recv_timeout(Duration::from_secs(2)).unwrap(),PlannerStep::Command(c) if c!=Twist2::ZERO));
        if published {release.send(()).unwrap();wait_recent_rechecked(&worker);}
        publish_recent_for_recheck(&mut worker,&map,1,[0.5,1.0,0.5]);
        if !published {release.send(()).unwrap();}
        wait_recent_rechecked(&worker);
        assert!(!matches!(worker.step(0.3),PlannerStep::Command(c) if c!=Twist2::ZERO));
        assert_eq!(counts.lock().unwrap().full,1);worker.stop();
    }
}

#[test]
fn latest_recent_high_frequency_microjitter_keeps_nonzero_one_shot_output() {
    let(mut worker,mut map,counts,computed,release)=recent_recheck_fixture();
    computed.recv_timeout(Duration::from_secs(2)).unwrap();release.send(()).unwrap();
    let mut nonzero=0;let started=std::time::Instant::now();
    for sequence in 1..=30 {
        map=publish_recent_for_recheck(&mut worker,&map,sequence,[10.0+sequence as f64*0.0001,10.0,0.5]);
        let until=std::time::Instant::now()+Duration::from_millis(20);
        while std::time::Instant::now()<until {
            if let PlannerStep::Command(command)=worker.step(0.3) {
                if command!=Twist2::ZERO {nonzero+=1;worker.observe_applied_command(command);}
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    let c=counts.lock().unwrap();
    eprintln!("RECENT_MICROJITTER frames=30 wall_ms={} issued={} full={} rechecks={} recheck_us={:?}",started.elapsed().as_millis(),nonzero,c.full,c.rechecks,c.recheck_us);
    assert!(nonzero>=3,"continuous source refresh starved all selected commands");
    assert!(c.rechecks>0);drop(c);worker.stop();
}

#[test]
fn latest_recent_recheck_preserves_original_expiration() {
    let(mut worker,map,counts,computed,release)=recent_recheck_fixture();
    computed.recv_timeout(Duration::from_secs(2)).unwrap();release.send(()).unwrap();wait_recent_rechecked(&worker);
    let original=worker.shared.0.lock().unwrap().result.as_ref().unwrap().0;
    publish_recent_for_recheck(&mut worker,&map,1,[10.0001,10.0,0.5]);wait_recent_rechecked(&worker);
    assert_eq!(counts.lock().unwrap().full,1);
    std::thread::sleep((original+DEFAULT_CONTROL_HORIZON+Duration::from_millis(5))
        .saturating_duration_since(std::time::Instant::now()));
    assert!(!matches!(worker.step(0.3),PlannerStep::Command(c) if c!=Twist2::ZERO),"recheck renewed the old issue window");
    worker.stop();
}

#[test]
fn latest_recent_pose_change_rechecks_current_state_without_reranking() {
    for published in [false,true] {
        let(mut worker,map,counts,computed,release)=recent_recheck_fixture();
        assert!(matches!(computed.recv_timeout(Duration::from_secs(2)).unwrap(),PlannerStep::Command(c) if c!=Twist2::ZERO));
        if published {release.send(()).unwrap();wait_recent_rechecked(&worker);}
        worker.update_odometry(route_pose(0.6,0));
        publish_recent_for_recheck(&mut worker,&map,1,[10.0,10.0,0.5]);
        if !published {release.send(()).unwrap();}
        wait_recent_rechecked(&worker);
        let input=worker.shared.0.lock().unwrap();
        assert_eq!(input.result_pose_version,input.pose_version);
        drop(input);
        let observed=counts.lock().unwrap();
        assert!(observed.rechecks>=1,"a changed pose requires a new full stopping check");
        assert_eq!(observed.full,1,"current-state check must not rerank, published={published}");
        drop(observed);
        assert!(matches!(worker.step(0.3),PlannerStep::Command(c) if c!=Twist2::ZERO));
        worker.stop();
    }
}

#[test]
fn latest_received_pose_recheck_keeps_twenty_hz_moving_progress() {
    let(mut worker,_map,counts,computed,release)=recent_recheck_fixture();
    computed.recv_timeout(Duration::from_secs(2)).unwrap();release.send(()).unwrap();wait_recent_rechecked(&worker);
    let started=std::time::Instant::now();let mut next_pose=started;let mut next_control=started+Duration::from_millis(12);
    let mut integrated=started;let mut x=0.5;let mut command=Twist2::ZERO;let mut nonzero=0;let mut controls=0;let mut poses=0;
    while started.elapsed()<Duration::from_millis(1200) {
        let now=std::time::Instant::now();x+=command.linear.x*now.duration_since(integrated).as_secs_f64();integrated=now;
        if now>=next_pose {
            let mut pose=route_pose(x,poses);let ns=crate::command_proof_trace::unix_ns();
            pose.stamp=Timestamp::from_unix((ns/1_000_000_000)as i64,(ns%1_000_000_000)as u32);
            worker.update_odometry(pose);poses+=1;next_pose+=Duration::from_millis(25);
        }
        if now>=next_control {
            command=match worker.step(0.3) {PlannerStep::Command(c)=>c,_=>Twist2::ZERO};
            worker.observe_applied_command(command);controls+=1;
            if command!=Twist2::ZERO {nonzero+=1;}
            next_control+=Duration::from_millis(50);
        }
        std::thread::sleep(Duration::from_micros(500));
    }
    let c=counts.lock().unwrap();
    eprintln!("MOVING_CURRENT_RECHECK poses={poses} controls={controls} nonzero={nonzero} travelled={} full={} rechecks={} recheck_us={:?}",x-0.5,c.full,c.rechecks,c.recheck_us);
    assert!(poses>=45 && controls>=23 && nonzero>=12 && x-0.5>0.05,"current-pose checks starved ordinary moving control");
    assert!(c.rechecks>0);drop(c);worker.stop();
}
