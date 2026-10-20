// Deterministic first-delivery regression. The real body guard computes once;
// only its publication is gated. Geometry, timing thresholds and safety oracle
// are unchanged. This is distinct from reusing an already consumed command.
fn run_first_delivery_pose_falsifier(change_before_publication:bool) {
        use nuwax_nav_mls_planner::{SurfaceObservation,SurfaceObservations};
        use nuwax_nav_types_messages::{MotionLimits,Point2};
        use nuwax_nav_control::PlanarPose;
        let unix_ns=||std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64;
        let stamped=|x:f64| {let mut p=route_pose(x,0);let ns=unix_ns();p.stamp=Timestamp::from_unix((ns/1_000_000_000)as i64,(ns%1_000_000_000)as u32);p};
        let limits=MotionLimits{model:MotionModel::Differential,max_forward_mps:0.5,max_reverse_mps:0.0,
            max_lateral_mps:0.0,max_angular_rps:0.8,linear_acceleration_mps2:1.0,guaranteed_braking_mps2:1.0,
            angular_acceleration_rps2:2.0,guaranteed_angular_braking_rps2:2.0,reaction_time_sec:0.05,
            minimum_turning_radius_m:0.0,can_rotate_in_place:true};
        let source=crate::surface_follower::test_snapshot(0.0,false).with_body_envelope(SurfaceBodyEnvelope{
            fixed_collision: None,
            footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0}).unwrap();
        let sampled=unix_ns();
        let mut blocks=std::collections::BTreeMap::<_,std::collections::BTreeMap<_,_>>::new();
        for (x,y,z) in source.cell_keys(){let key=[x,y,z];blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
            SurfaceObservation{observed_ns:if x>=7{sampled.saturating_sub(2_000_000_000)}else{sampled},hits:2});}
        let mut observations=SurfaceObservations::new(source.source_voxel_size_m(),32).unwrap();
        for (key,block) in blocks{observations.replace_block(key,block).unwrap();}
        observations.mark_complete_geometry_source();
        let source=Arc::new(source.with_observations(Some(Arc::new(observations)))
            .with_recent_obstacle_points(1,sampled,0,&[],[(0,[0.75,1.0,0.5])]).unwrap());
        let mut config=SurfaceFollowerConfig::new(0.4);config.local_planning_enabled=false;
        config.controller.control_frequency=20.0;config.controller.speed=0.5;
        config.motion_limits=Some(limits.clone());config.max_observation_age_sec=Some(1.0);
        assert!(!config.controller.simulation);
        let (computed,receive)=std::sync::mpsc::channel();let (release,wait)=std::sync::mpsc::channel();
        let mut harness=RouteWorkerHarness{worker:PlannerWorker::new(GatedRouteFollower{
            refresh_gate: None,
            inner:RouteFollower::new(config).unwrap(),computed,release:wait,map:None,boundary:Vec::new(),
            odometry:None,goal_updates:0,path_updates:0}).unwrap(),computed:receive,release,clear:source.clone()};
        let start=stamped(0.533);let goal=stamped(2.0);
        source.validate_body_pose([0.533,1.0,0.5],[0.0,0.0,0.0,1.0]).unwrap();
        assert!(source.validate_body_pose([0.56,1.0,0.5],[0.0,0.0,0.0,1.0]).is_err(),
            "real forward obstacle must separate initial clearance from unsafe continued travel");
        let previous=Twist2{linear:Point2::new(0.1,0.0),angular_z:0.0};
        harness.worker.update_execution_map(ExecutionMap::Surface(source.clone())).unwrap();
        harness.worker.configure_goal(&goal,GoalOptions::default()).unwrap();
        harness.worker.update_odometry(start.clone());
        harness.worker.replace_execution_path(ExecutionPath::Surface(Path3D{poses:vec![start.clone(),goal.clone()]}),None,goal).unwrap();
        harness.worker.observe_applied_command(previous);harness.worker.step(0.3);
        let initial=harness.next_computation();
        let PlannerStep::Command(first)=initial.outcome else{panic!("initial guarded command missing: {:?}",initial.outcome);};
        assert!(first.linear.x>=0.09 && first.angular_z.abs()<1e-9,"fixture must establish the maximal safe initial command: {first:?}");

    // The previously applied 0.1m/s command advances 8mm in 80ms, well
    // inside the unchanged 250ms result lease. Do not issue the selected
    // candidate until after this real interval and newer pose receipt.
    if !change_before_publication {
        harness.release.send(()).unwrap();wait_for_source_result(&harness.worker);
    }
    std::thread::sleep(Duration::from_millis(80));
    let current=stamped(start.pose.position.x+previous.linear.x*0.08);
    harness.worker.update_odometry(current.clone());
    if change_before_publication {
        harness.release.send(()).unwrap();wait_for_source_result(&harness.worker);
    }
    let (old_version,new_version)={let input=harness.worker.shared.0.lock().unwrap();
        (input.result_pose_version,input.pose_version)};
    let delivered=harness.worker.step(0.3);
    let pose=PlanarPose{x:current.pose.position.x,y:1.0,yaw:0.0};
    let body=|p:PlanarPose|([p.x,p.y,0.5],[0.0,0.0,0.0,1.0]);
    let view=source.with_current_body_support(body(pose).0,body(pose).1).unwrap();
    let check=|a:PlanarPose,b:PlanarPose,_elapsed:f64| {
        let Ok(projected)=view.body_pose_after_motion(body(a),[b.x,b.y],b.yaw) else{return false;};
        view.validate_body_motion(body(a),projected,&[]).is_ok()
    };
    let safe=nuwax_nav_control::check_stopping_trajectory_with_period_check(pose,previous,first,&limits,
        0.05,f64::from(source.source_voxel_size_m())*0.5,source.body_envelope().unwrap().radius(),check,check).is_some();
    let remaining=harness.worker.delivered_command_deadline_for_test()
        .map(|deadline|deadline.saturating_duration_since(std::time::Instant::now()));
    eprintln!("FIRST_DELIVERY before_publication={change_before_publication} compute_pose={:?} latest_pose={:?} result_pose_version={old_version} current_pose_version={new_version} candidate={first:?} delivered={delivered:?} current_full_stopping_safe={safe} remaining={remaining:?}",initial.odometry,current);
    harness.worker.stop();let _=harness.release.send(());
    assert!(!safe,"counterexample must distinguish original from current-pose authority");
    assert!(!matches!(delivered,PlannerStep::Command(command) if command!=Twist2::ZERO),
        "first delivery may not extend old-pose authority into an unsafe current-pose stopping trajectory");
}

#[test]
#[ignore="targeted first-delivery stale-pose regression; preserves wall-off diagnostic"]
fn first_command_pose_change_during_compute_requires_current_stopping_permission() {
    run_first_delivery_pose_falsifier(true);
}
#[test]
#[ignore="targeted first-delivery stale-pose regression; preserves wall-off diagnostic"]
fn first_command_pose_change_after_publish_requires_current_stopping_permission() {
    run_first_delivery_pose_falsifier(false);
}
