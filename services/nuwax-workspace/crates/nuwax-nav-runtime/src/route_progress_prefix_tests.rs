
fn install_progress_prefix(follower:&mut RouteFollower, poses:Vec<PoseStamped>, held:bool)->PlannerGate {
    let (started,starts)=mpsc::channel();let (release,released)=mpsc::channel();
    follower.cancel_planning();
    follower.owner=Some(PlanningOwner::new(move|mut request:PlanRequest,_|{
        started.send(()).unwrap();if held {let _=released.recv_timeout(Duration::from_secs(5));}
        request.materialize_reference().unwrap();
        PlanResult {request,outcome:Ok(Candidate {partial:true,path:Path3D{poses:poses.clone()},elapsed_ms:0.0})}
    }).unwrap());follower.dirty=true;PlannerGate{starts,release}
}
fn deliver_progress_prefix(follower:&mut RouteFollower) {
    follower.retry_after=Instant::now();follower.schedule(false);
    wait_for(||follower.owner.as_ref().unwrap().completion_ready_for_test());follower.consume_plan();
}
#[test]
fn progress_prefix_records_actual_coverage_and_keeps_original_goal_at_its_end() {
    let mut follower=open_follower();let goal=follower.goal.clone();let route=follower.route.clone();
    let _gate=install_progress_prefix(&mut follower,vec![comparison_pose(0.5,0),comparison_pose(1.2,0)],false);
    deliver_progress_prefix(&mut follower);
    assert_eq!(follower.local_plans(),1);assert!((follower.local_end-0.7).abs()<1e-9);
    assert!(follower.local_end<*follower.cumulative.last().unwrap());
    assert_eq!(follower.goal,goal);assert_eq!(follower.route,route);
    follower.update_odometry(comparison_pose(1.2,1));
    assert!(matches!(follower.controller.step(0.3),PlannerStep::ReplanRequired(_)),
        "有限端点未到原goal时必须接续，不能发布成功");
    install_failures(&mut follower,typed_failures());complete_failure(&mut follower);
    for _ in 0..3 {assert!(matches!(follower.step(0.3),PlannerStep::Pending(_)),
        "无后继时保持待定，不能用零速Command续期原等待预算");}
    assert_eq!(follower.goal,goal);
}
#[test]
fn progress_prefix_rejects_tiny_motion_lateral_motion_and_closed_loops() {
    let mut lateral=comparison_pose(0.5,0);lateral.pose.position.y=1.5;
    let mut turned=comparison_pose(1.1,0);turned.pose.orientation=Quaternion::from_yaw(std::f64::consts::PI);
    let mut returned=comparison_pose(0.5,0);returned.pose.orientation=turned.pose.orientation;
    for path in [vec![comparison_pose(0.5,0),comparison_pose(0.52,0)],
        vec![comparison_pose(0.5,0),lateral],
        vec![comparison_pose(0.5,0),comparison_pose(1.1,0),turned,returned]] {
        let mut follower=open_follower();let goal=follower.goal.clone();
        let _gate=install_progress_prefix(&mut follower,path,false);deliver_progress_prefix(&mut follower);
        assert_eq!(follower.local_plans(),0);assert!(follower.local_path().is_none());
        assert!(follower.admitted_generation.is_none());assert_eq!(follower.goal,goal);
    }
}
#[test]
fn progress_prefix_repeated_or_already_passed_endpoint_cannot_renew_admission() {
    let mut follower=open_follower();let path=vec![comparison_pose(0.5,0),comparison_pose(1.2,0)];
    let _gate=install_progress_prefix(&mut follower,path.clone(),false);deliver_progress_prefix(&mut follower);
    assert_eq!(follower.local_plans(),1);
    follower.update_odometry(comparison_pose(1.2,1));follower.advance_progress();
    let _gate=install_progress_prefix(&mut follower,path.clone(),false);deliver_progress_prefix(&mut follower);
    assert_eq!(follower.local_plans(),1,"重复末点不能被计作新的前向覆盖");
    let mut follower=open_follower();let gate=install_progress_prefix(&mut follower,path,true);
    follower.schedule(false);gate.started();follower.update_odometry(comparison_pose(1.4,1));follower.advance_progress();
    gate.release();wait_for(||follower.owner.as_ref().unwrap().completion_ready_for_test());follower.consume_plan();
    assert_eq!(follower.local_plans(),0,"请求期间已越过的有限末点必须淘汰");
    assert_eq!(follower.pose.as_ref().unwrap().pose.position.x,1.4);
}
#[test]
fn progress_prefix_cancel_and_changed_source_discard_held_candidates() {
    for change in 0..3 {
        let mut follower=open_follower();let gate=install_progress_prefix(&mut follower,
            vec![comparison_pose(0.5,0),comparison_pose(1.2,0)],true);
        follower.schedule(false);gate.started();let epoch=follower.owner.as_ref().unwrap().epoch();
        match change {0=>follower.stop(),1=>follower.configure_goal(&comparison_pose(4.0,1),GoalOptions::default()).unwrap(),
            _=>follower.update_execution_map(ExecutionMap::Surface(comparison_map(true,None))).unwrap()};
        assert!(follower.owner.as_ref().unwrap().epoch()>epoch);gate.release();
        wait_for(||follower.owner.as_ref().unwrap().is_idle_for_test());follower.consume_plan();
        assert_eq!(follower.local_plans(),0);assert!(follower.local_path().is_none());assert!(follower.admitted_generation.is_none());
    }
}
#[test]
fn progress_prefix_requires_braking_calibration_and_keeps_current_stop_guard() {
    let mut uncalibrated=open_follower();
    uncalibrated.config.basic_kinematics=uncalibrated.config.kinematics();
    uncalibrated.config.motion_limits=None;
    let _gate=install_progress_prefix(&mut uncalibrated,vec![comparison_pose(0.5,0),comparison_pose(1.2,0)],false);
    deliver_progress_prefix(&mut uncalibrated);assert_eq!(uncalibrated.local_plans(),0);
    let mut follower=open_follower();let _gate=install_progress_prefix(&mut follower,
        vec![comparison_pose(0.5,0),comparison_pose(1.2,0)],false);deliver_progress_prefix(&mut follower);
    let source=Arc::new(follower.map.as_ref().unwrap().with_recent_obstacle_points(
        7,1_000_000_000,0,&[],[(0,[0.75,1.0,0.5])]).unwrap());
    let pose=comparison_pose(0.5,0);source.validate_body_pose(body(&pose).0,body(&pose).1).unwrap();
    let moving=Twist2{linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0};
    let (safe,reason)=crate::surface_follower::SurfaceFollower::replay_stopping_trajectory_for_test(
        &follower.config,source.clone(),pose,moving,moving,&[]).unwrap();
    assert!(!safe,"前方回波在实际制动尾段内，有限路径资格不能放行：{reason:?}");
    follower.observe_applied_command(moving);follower.update_execution_map(ExecutionMap::Surface(source)).unwrap();
    // 地图更新可能先撤销旧命令资格；重新取控制结果仍不得发出旧非零速度。
    let decision=follower.step(0.3);
    assert!(!matches!(decision,PlannerStep::Command(v)if v.linear.x>0.0));
    assert!(!matches!(decision,PlannerStep::Succeeded));
}
