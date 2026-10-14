// Included in route_follower::tests to exercise the real ON owner and admission gates.
fn selective_scene(extra: &[[i32;3]]) -> Arc<SurfaceSnapshot> {
    let base=comparison_map(false,None);
    let size=base.source_voxel_size_m();
    let mut config=base.config().clone();config.wall_clearance_m=0.0;
    let points:Vec<_>=base.cell_keys().map(|(x,y,z)|[x,y,z]).chain(extra.iter().copied())
        .map(|p|((p[0] as f32+0.5)*size,(p[1] as f32+0.5)*size,(p[2] as f32+0.5)*size)).collect();
    Arc::new(SurfaceSnapshot::from_points(&points,config,size).unwrap()
        .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap())
}
fn selective_update(old:&Arc<SurfaceSnapshot>, next:Arc<SurfaceSnapshot>, columns:&[[i32;2]])->SurfaceMapUpdate {
    SurfaceMapUpdate {planar_reference:None,change_basis:Some(old.clone()),changes:Arc::new(SurfaceGeometryChanges::Columns {
        fine_size:next.source_voxel_size_m(),fine:columns.to_vec(),
        planner_size:next.config().voxel_size,coarse:columns.to_vec(),
    }),snapshot:next}
}
#[test]
fn selective_far_updates_keep_held_on_candidate_and_progress() {
    let mut follower=open_follower();
    follower.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
    follower.update_odometry(comparison_pose(0.8,1));follower.advance_progress();
    let progress=follower.progress_m();assert!(progress>0.0);
    let gate=install_held_candidate(&mut follower);
    follower.schedule(false);gate.started();
    let epoch=follower.owner.as_ref().unwrap().epoch();let calls=follower.local_planning_calls();
    let mut old=follower.map.clone().unwrap();
    // Includes deleting the last occupied cell of a far column, not just additions.
    for (cells,columns) in [
        (vec![[100,100,5]],vec![[100,100]]),
        (vec![[110,100,5]],vec![[100,100],[110,100]]),
        (vec![],vec![[110,100]]),
    ] {
        let next=selective_scene(&cells);
        assert!(!nuwax_nav_pct::same_source_geometry(&old,&next));
        follower.update_execution_map(ExecutionMap::SurfaceUpdate(selective_update(&old,next.clone(),&columns))).unwrap();
        assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch);
        assert_eq!(follower.local_planning_calls(),calls);
        assert_eq!(follower.progress_m(),progress);
        old=next;
    }
    assert_eq!(follower.source_tickets.iter().flatten().next().unwrap().1.changes.len(),3);
    gate.release();wait_for(||follower.owner.as_ref().unwrap().completion_ready_for_test());
    follower.consume_plan();assert_eq!(follower.local_plans(),1);
    assert_eq!(follower.progress_m(),progress);
    assert!(matches!(follower.step(0.3),PlannerStep::Command(c) if c.linear.x>0.0));
    assert_eq!(follower.local_planning_calls(),calls);
}
#[test]
fn selective_related_then_far_and_unknown_cannot_revive_old_candidate() {
    for mode in 0..4 {
        let mut follower=open_follower();
        let initial=selective_scene(&[]);
        let observed=comparison_map(false,Some(1_000_000_000));
        let initial=Arc::new(initial.with_observations(observed.observations().cloned()));
        follower.update_execution_map(ExecutionMap::Surface(initial)).unwrap();
        let gate=install_held_candidate(&mut follower);
        follower.schedule(false);gate.started();let epoch=follower.owner.as_ref().unwrap().epoch();
        let old=follower.map.clone().unwrap();
        let mut update=selective_update(&old,selective_scene(&[[100,100,5]]),&[[100,100]]);
        match mode {
            // Fine-only on-route occupancy changes while the entire coarse Arc is unchanged.
            0=>{
                let mut fine=old.observations().unwrap().as_ref().clone();
                let mut block=std::collections::BTreeMap::new();
                for (x,y,z) in old.cell_keys() {
                    let key=[x,y,z];
                    if key.map(|v|v.div_euclid(32))==[0,0,0] {
                        block.insert(key,nuwax_nav_mls_planner::SurfaceObservation {observed_ns:1_000_000_000,hits:2});
                    }
                }
                block.insert([10,10,4],nuwax_nav_mls_planner::SurfaceObservation {observed_ns:1_000_000_000,hits:2});
                fine.replace_block([0,0,0],block).unwrap();
                update.snapshot=Arc::new(old.with_observations(Some(Arc::new(fine))));
                assert!(old.same_geometry(&update.snapshot));
                assert!(!nuwax_nav_pct::same_source_geometry(&old,&update.snapshot));
                update.changes=Arc::new(SurfaceGeometryChanges::Columns {fine_size:old.source_voxel_size_m(),
                    fine:vec![[10,10]],planner_size:old.config().voxel_size,coarse:vec![]});
            },
            1=>update.changes=Arc::new(SurfaceGeometryChanges::Unknown),
            2=>update.change_basis=None,
            _=>{let mut model=old.body_envelope().unwrap().clone();model.z_bounds_m[1]+=0.05;
                update.snapshot=Arc::new(update.snapshot.with_body_envelope(model).unwrap());},
        }
        follower.update_execution_map(ExecutionMap::SurfaceUpdate(update)).unwrap();
        assert!(follower.owner.as_ref().unwrap().epoch()>epoch);
        let old=follower.map.clone().unwrap();
        follower.update_execution_map(ExecutionMap::SurfaceUpdate(selective_update(&old,selective_scene(&[[110,100,5]]),&[[100,100],[110,100]]))).unwrap();
        gate.release();std::thread::sleep(Duration::from_millis(10));follower.consume_plan();
        assert_eq!(follower.local_plans(),0,"later disjoint input revived rejected mode {mode}");
        assert!(follower.local_path().is_none());
    }
}
#[test]
fn selective_proof_accumulates_reverted_related_deletion_and_enforces_budget() {
    let first=selective_scene(&[]);let route=Path3D {poses:vec![comparison_pose(0.5,0),comparison_pose(3.0,0)]};
    let mut ticket=SurfaceJobTicket::new(Some(first.clone()));
    let changed=selective_scene(&[[10,10,5]]);
    let related=selective_update(&first,changed.clone(),&[[10,10]]);
    ticket.observe(&changed,related.change_basis.as_deref(),&related.changes);
    // Reverting/deleting the intervening obstacle must not erase its delta.
    let reverted=selective_update(&changed,first.clone(),&[[10,10]]);
    ticket.observe(&first,reverted.change_basis.as_deref(),&reverted.changes);
    assert!(!ticket.permits_path(&first,&route));assert_eq!(ticket.changes.len(),2);
    let mut unknown=SurfaceJobTicket::new(Some(first.clone()));
    unknown.observe(&first,Some(&first),&Arc::new(SurfaceGeometryChanges::Unknown));
    unknown.observe(&first,Some(&first),&Arc::new(SurfaceGeometryChanges::Empty));
    assert!(!unknown.permits_path(&first,&route));
    let limit=crate::surface_changes::MAX_REFERENCE_SCOPE_WORK;
    let mut bounded=SurfaceJobTicket::new(Some(first.clone()));
    bounded.observe(&first,Some(&first),&Arc::new(SurfaceGeometryChanges::Columns {
        fine_size:first.source_voxel_size_m(),fine:vec![[100,100];limit],
        planner_size:first.config().voxel_size,coarse:vec![[100,100]],
    }));
    assert!(bounded.unknown);assert!(!bounded.permits_path(&first,&route));
}
#[test]
fn selective_coalescing_preserves_all_batches_and_missing_baseline_is_unknown() {
    let first=selective_scene(&[]);let middle=selective_scene(&[[10,10,5]]);let last=selective_scene(&[[100,100,5]]);
    let mut update=selective_update(&first,middle.clone(),&[[10,10]]);
    update.append(selective_update(&middle,last.clone(),&[[10,10],[100,100]]));
    let mut proof=SurfaceJobTicket::new(Some(first.clone()));
    proof.observe(&update.snapshot,update.change_basis.as_deref(),&update.changes);
    assert_eq!(proof.columns,6);
    assert!(!proof.permits_path(&last,&Path3D {poses:vec![comparison_pose(0.5,0),comparison_pose(3.0,0)]}));
    let mut gap=selective_update(&first,middle.clone(),&[[10,10]]);
    gap.append(selective_update(&first,last,&[[100,100]]));
    assert!(matches!(gap.changes.as_ref(),SurfaceGeometryChanges::Unknown));
}
#[test]
fn selective_latest_related_wall_still_brakes_an_admitted_on_route() {
    let mut follower=open_follower();
    follower.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
    assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0));
    follower.observe_applied_command(Twist2 {linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0});
    let progress=follower.progress_m();let old=follower.map.clone().unwrap();
    let cells:Vec<_>=(0..35).flat_map(|y|(1..8).map(move|z|[7,y,z])).collect();
    let columns:Vec<_>=(0..35).map(|y|[7,y]).collect();
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(selective_update(&old,selective_scene(&cells),&columns))).unwrap();
    follower.update_odometry(comparison_pose(0.5,1));
    let step=follower.step(0.3);
    assert!(!matches!(step,PlannerStep::Command(c) if c.linear.x>0.0),"fresh wall lost execution authority: {step:?}");
    assert_eq!(follower.progress_m(),progress);
}
#[test]
fn selective_active_and_replaced_pending_keep_their_own_complete_proofs() {
    let mut follower=open_follower();
    follower.scan.refresh_distance_m=0.25;
    follower.update_execution_map(ExecutionMap::Surface(selective_scene(&[]))).unwrap();
    let gate=install_held_candidate(&mut follower);
    follower.schedule(false);gate.started();let active=follower.submitted.unwrap().0;
    // The active job remains held while two pending submissions coalesce.
    for x in [0.9,1.3] {
        follower.update_odometry(comparison_pose(x,1));follower.advance_progress();
        follower.retry_after=Instant::now();follower.schedule(false);
    }
    let pending=follower.submitted.unwrap().0;
    let ids:Vec<_>=follower.source_tickets.iter().flatten().map(|(id,_)|*id).collect();
    assert_eq!(ids,vec![active,pending]);assert_eq!(pending,active+2);
    let epoch=follower.owner.as_ref().unwrap().epoch();
    let old=follower.map.clone().unwrap();
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(selective_update(&old,selective_scene(&[[100,100,5]]),&[[100,100]]))).unwrap();
    assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch);
    assert!(follower.source_tickets.iter().flatten().all(|(_,ticket)|ticket.changes.len()==1));
    gate.release();wait_for(||follower.owner.as_ref().unwrap().completion_ready_for_test());
    follower.consume_plan();assert_eq!(follower.local_plans(),1,"active proof was overwritten by pending");
    gate.started();gate.release();wait_for(||follower.owner.as_ref().unwrap().completion_ready_for_test());
    follower.consume_plan();assert_eq!(follower.local_plans(),2);
    assert!(follower.source_tickets.iter().all(Option::is_none));
    follower.retry_after=Instant::now();follower.schedule(false);gate.started();
    follower.update_odometry(comparison_pose(1.7,2));
    follower.retry_after=Instant::now();follower.schedule(false);
    assert_eq!(follower.source_tickets.iter().flatten().count(),2);
    follower.cancel_planning();assert!(follower.source_tickets.iter().all(Option::is_none));
    gate.release();wait_for(||follower.owner.as_ref().unwrap().is_idle_for_test());
    assert!(gate.starts.try_recv().is_err(),"cancelled pending request must not start");
    follower.consume_plan();assert_eq!(follower.local_plans(),2);
}

#[test]
fn recent_obstacle_updates_keep_terrain_and_trigger_bounded_detour_then_recover() {
    let mut follower=open_follower();
    assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0));
    let basis=follower.map.clone().unwrap();
    let epoch=follower.owner.as_ref().unwrap().epoch();
    let calls=follower.local_planning_calls();
    let far=Arc::new(basis.with_recent_obstacle_points(1,1_000_000_000,0,&[],
        [(0,[10.0,10.0,0.5])]).unwrap());
    let publish=|follower:&mut RouteFollower,previous:Arc<SurfaceSnapshot>,next:Arc<SurfaceSnapshot>| {
        assert!(nuwax_nav_pct::same_source_geometry(&previous,&next));
        follower.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {planar_reference:None,
            snapshot:next,change_basis:Some(previous),changes:Arc::new(SurfaceGeometryChanges::Empty),
        })).unwrap();
    };
    publish(&mut follower,basis,far.clone());
    assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch);
    assert_eq!(follower.local_planning_calls(),calls);
    // An obstacle ahead of the current prism intersects the actual stopping tail.
    let wall=Arc::new(far.with_recent_obstacle_points(2,2_000_000_000,0,&[],
        (0..5).map(|i|(i,[0.73,0.8+f64::from(i)*0.1,0.5]))).unwrap());
    publish(&mut follower,far,wall.clone());
    follower.controller.observe_applied_command(nuwax_nav_types_messages::Twist2 {
        linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0,
    });
    follower.retry_after=Instant::now();
    let rejected=follower.step(0.3);
    assert!(matches!(rejected,PlannerStep::Pending(_)),"{rejected:?}");
    assert!(follower.execution_wait_reason().is_some_and(|reason|reason.contains("当前轨迹受阻")),"original rejection must remain visible during local recovery");
    assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch,"live obstacle must not cancel terrain work");
    assert!(follower.local_planning_calls()>calls,"guard rejection must schedule detour work");
    let submitted=follower.local_planning_calls();
    assert!(follower.submitted.is_some_and(|(_,_,detour)|detour));
    // A genuinely later scan removes the obstacle without changing terrain.
    let cleared=Arc::new(wall.with_recent_obstacle_points(3,3_000_000_000,0,&[0],
        [(0,[0.73,1.0,0.05])]).unwrap());
    publish(&mut follower,wall,cleared);
    follower.controller.observe_applied_command(nuwax_nav_types_messages::Twist2::ZERO);
    // This fixture uses simulation time; advance its real pose clock past braking.
    follower.update_odometry(comparison_pose(0.5,20));
    // A newly admitted detour may begin with an in-place turn even after the
    // obstacle clears. Execute that turn rather than repeatedly asking a frozen
    // yaw for a forward command. Pose stamps advance only by real elapsed time.
    let began=Instant::now();
    let deadline=began+Duration::from_secs(5);
    let mut previous=began;
    let mut actual=nuwax_nav_control::PlanarPose{x:0.5,y:1.0,yaw:0.0};
    let mut applied=nuwax_nav_types_messages::Twist2::ZERO;
    let limits=follower.config.motion_limits.clone().unwrap();
    let period=1.0/follower.config.controller.control_frequency;
    let map=follower.map.clone().unwrap();
    let body_at=|p:nuwax_nav_control::PlanarPose| {
        let q=Quaternion::from_yaw(p.yaw);
        ([p.x,p.y,0.5],[q.x,q.y,q.z,q.w])
    };
    loop {
        let now=Instant::now();
        let next=nuwax_nav_control::integrate(actual,applied,now.duration_since(previous).as_secs_f64());
        map.validate_body_motion(body_at(actual),body_at(next),&[]).expect("actual cleared-obstacle recovery edge");
        actual=next;previous=now;
        let elapsed=now.duration_since(began);
        let mut observed=comparison_pose(actual.x,20);
        observed.pose.position.y=actual.y;
        observed.pose.orientation=Quaternion::from_yaw(actual.yaw);
        observed.stamp=Timestamp::from_unix(2+elapsed.as_secs() as i64,elapsed.subsec_nanos());
        follower.update_odometry(observed);
        follower.observe_applied_command(applied);
        let step=follower.step(0.3);
        if let PlannerStep::Command(command)=step {
            let check=|| |a:nuwax_nav_control::PlanarPose,b:nuwax_nav_control::PlanarPose,_:f64| {
                map.validate_body_motion(body_at(a),body_at(b),&[])
                    .and_then(|_|map.body_query_with_current_support(body_at(a).0,body_at(a).1)
                        .and_then(|view|view.validate_body_motion(body_at(a),body_at(b),&[]))).is_ok()
            };
            assert!(nuwax_nav_control::check_stopping_trajectory_with_period_check(actual,applied,command,&limits,
                period,f64::from(map.source_voxel_size_m())*0.5,map.body_envelope().unwrap().radius(),check(),check()).is_some(),
                "cleared-obstacle recovery command must retain its complete braking tail");
            applied=command;
            if command.linear.x>0.0 {break;}
        } else {applied=nuwax_nav_types_messages::Twist2::ZERO;}
        assert!(Instant::now()<deadline,"cleared recent obstacle never recovered: {step:?}");
        std::thread::sleep(Duration::from_secs_f64(period));
    }
    assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch);
    assert!(follower.local_planning_calls()<=submitted+2,"recovery must remain bounded");
}

#[test]
fn old_ground_blind_footprint_start_does_not_require_per_cell_hits_or_ttl() {
    for enabled in [false,true] {
        let mut config=comparison_config(Some(enabled));
        config.max_observation_age_sec=Some(1.0);
        let mut follower=RouteFollower::new(config).unwrap();
        // Stored terrain is known; this controller has no fresh foot-cell record.
        // The independent scheduler is responsible for whole sensor-stream health.
        follower.update_execution_map(ExecutionMap::Surface(comparison_map(false,None))).unwrap();
        comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(3.0,0));
        assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0),
            "stored support must permit normal startup, local planning={enabled}");
    }
}

#[test]
fn persistent_recent_single_obstacle_can_admit_a_short_local_detour() {
    let mut follower=RouteFollower::new(comparison_config(Some(true))).unwrap();
    let base=comparison_map(false,None);
    follower.update_execution_map(ExecutionMap::Surface(base.clone())).unwrap();
    comparison_route(&mut follower,comparison_pose(0.5,0),comparison_pose(1.0,0));
    assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(_)));
    let previous_plans=follower.local_plans();
    let obstacle=Arc::new(base.with_recent_obstacle_points(1,1_000_000_000,0,&[],[(0,[0.73,1.03,0.5])]).unwrap());
    assert!(nuwax_nav_pct::same_source_geometry(&base,&obstacle));
    follower.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {planar_reference:None,
        snapshot:obstacle,change_basis:Some(base),changes:Arc::new(SurfaceGeometryChanges::Empty),
    })).unwrap();
    follower.controller.observe_applied_command(nuwax_nav_types_messages::Twist2 {
        linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0,
    });
    follower.retry_after=Instant::now();
    assert!(matches!(follower.step(0.3),PlannerStep::Pending(_)));
    assert!(follower.execution_wait_reason().is_some_and(|reason|reason.contains("当前轨迹受阻")));
    follower.controller.observe_applied_command(nuwax_nav_types_messages::Twist2::ZERO);
    follower.update_odometry(comparison_pose(0.5,20));
    let deadline=Instant::now()+Duration::from_secs(5);
    loop {
        let step=follower.step(0.3);
        if follower.local_plans()>previous_plans
            && matches!(step,PlannerStep::Command(c) if c!=nuwax_nav_types_messages::Twist2::ZERO) {break;}
        assert!(Instant::now()<deadline,"short persistent obstacle detour unavailable: {step:?}, {:?}",follower.local_failures());
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(follower.local_path().unwrap().poses.iter().any(|p|(p.pose.position.y-1.0).abs()>0.1),
        "accepted route must actually go around the persistent obstacle");
}

#[test]
fn recent_wide_close_obstacle_admits_bounded_live_detour() {
    let mut follower=open_follower();
    assert!(matches!(ready_step(&mut follower,0.3),PlannerStep::Command(c) if c.linear.x>0.0));
    let basis=follower.map.clone().unwrap();
    let epoch=follower.owner.as_ref().unwrap().epoch();
    let calls=follower.local_planning_calls();
    let far=Arc::new(basis.with_recent_obstacle_points(1,1_000_000_000,0,&[],
        [(0,[10.0,10.0,0.5])]).unwrap());
    let publish=|follower:&mut RouteFollower,previous:Arc<SurfaceSnapshot>,next:Arc<SurfaceSnapshot>| {
        assert!(nuwax_nav_pct::same_source_geometry(&previous,&next));
        follower.update_execution_map(ExecutionMap::SurfaceUpdate(SurfaceMapUpdate {planar_reference:None,
            snapshot:next,change_basis:Some(previous),changes:Arc::new(SurfaceGeometryChanges::Empty),
        })).unwrap();
    };
    publish(&mut follower,basis,far.clone());
    assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch);
    assert_eq!(follower.local_planning_calls(),calls);
    // An obstacle ahead of the current prism intersects the actual stopping tail.
    let wall=Arc::new(far.with_recent_obstacle_points(2,2_000_000_000,0,&[],
        (0..5).map(|i|(i,[0.73,0.8+f64::from(i)*0.1,0.5]))).unwrap());
    publish(&mut follower,far,wall.clone());
    follower.controller.observe_applied_command(nuwax_nav_types_messages::Twist2 {
        linear:nuwax_nav_types_messages::Point2::new(0.5,0.0),angular_z:0.0,
    });
    follower.retry_after=Instant::now();
    let rejected=follower.step(0.3);
    assert!(matches!(rejected,PlannerStep::Pending(_)),"{rejected:?}");
    assert!(follower.execution_wait_reason().is_some_and(|reason|reason.contains("当前轨迹受阻")),"original rejection must remain visible during local recovery");
    assert_eq!(follower.owner.as_ref().unwrap().epoch(),epoch,"live obstacle must not cancel terrain work");
    assert!(follower.local_planning_calls()>calls,"guard rejection must schedule detour work");
    let submitted=follower.local_planning_calls();
    assert!(follower.submitted.is_some_and(|(_,_,detour)|detour));
    follower.controller.observe_applied_command(nuwax_nav_types_messages::Twist2::ZERO);
    follower.update_odometry(comparison_pose(0.5,20));
    let previous_plans=follower.local_plans();
    let deadline=Instant::now()+Duration::from_secs(5);
    loop {
        let step=follower.step(0.3);
        if follower.local_plans()>previous_plans
            && matches!(step,PlannerStep::Command(c) if c!=nuwax_nav_types_messages::Twist2::ZERO) {
            assert!(follower.local_path().unwrap().poses.iter().any(|p|(p.pose.position.y-1.0).abs()>0.1));
            break;
        }
        assert!(Instant::now()<deadline,
            "persistent wide close obstacle, start=(0.5,1,0.5), goal=(3,1,0.5), no randomness: {step:?}, failures={:?}",follower.local_failures());
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn recent_wide_close_obstacle_has_a_real_swept_geometry_witness_route() {
    let base=comparison_map(false,None);
    let map=base.with_recent_obstacle_points(1,1_000_000_000,0,&[],
        (0..5).map(|i|(i,[0.73,0.8+f64::from(i)*0.1,0.5]))).unwrap();
    let half=std::f64::consts::FRAC_PI_2;
    let poses=[(0.5,1.0,0.0),(0.5,1.0,half),(0.5,1.5,half),(0.5,1.5,0.0),
        (1.0,1.5,0.0),(1.0,1.5,-half),(1.0,1.0,-half),(1.0,1.0,0.0),(3.0,1.0,0.0)];
    let body=|(x,y,yaw):(f64,f64,f64)|([x,y,0.5],[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()]);
    for (index,pair) in poses.windows(2).enumerate() {
        let mut witness=None;
        map.validate_body_motion_with_witness(body(pair[0]),body(pair[1]),&[],&mut witness)
            .unwrap_or_else(|error|panic!("witness segment {index} failed: {error}; {witness:?}"));
    }
}
