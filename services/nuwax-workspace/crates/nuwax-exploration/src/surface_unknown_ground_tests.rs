// Unknown navigation references never manufacture observed floor evidence.
fn unknown_layer_map() -> Arc<SurfaceSnapshot> {
    Arc::new(SurfaceSnapshot::from_points(&[(10.05, 10.05, 0.05)], map().config().clone(), 0.1)
        .unwrap().with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.08,-0.08],[0.08,-0.08],[0.08,0.08],[-0.08,0.08]],
            z_bounds_m: [-0.05,0.2], body_to_support_z_m: 0.3,
            max_step_up_m: 0.2, max_step_down_m: 0.2, max_cross_slope_deg: 15.0,
            support_uncertainty_m: 0.0,
        }).unwrap())
}

#[test]
fn unknown_layer_selects_a_bounded_goal_without_changing_ground_evidence() {
    use nuwax_nav_mls_planner::surface::GroundQueryState;
    let snapshot = unknown_layer_map();
    let reference = [1.05, 1.05, 0.1];
    let before_cells: Vec<_> = snapshot.cell_keys().collect();
    assert_eq!(snapshot.query_ground([reference[0], reference[1]], reference[2]).state,
        GroundQueryState::Unknown);
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(snapshot.clone()).unwrap();
    let mut adapter = source(receiver, 1);
    let selected = goal(adapter.poll_goal(&state(0.4, 1)).unwrap());
    let p = selected.pose.pose.position;
    let q = selected.pose.pose.orientation;
    let body = ([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);
    let selected_reference = snapshot.support_position_from_body(body.0, body.1, 0.3).unwrap();
    assert!(distance(reference, selected_reference) > 0.1);
    assert!(distance(reference, selected_reference) <= 0.4 + 1e-6);
    assert!((p.z - 0.4).abs() < 1e-6);
    assert_eq!(snapshot.query_ground([p.x,p.y], 0.1).state, GroundQueryState::Unknown);
    assert!(snapshot.query_ground([p.x,p.y], 0.1).height.is_none());
    assert_eq!(snapshot.cell_keys().collect::<Vec<_>>(), before_cells);
    assert!(snapshot.observations().is_none());
    assert!(matches!(snapshot.locate_support(reference),
        Err(nuwax_nav_mls_planner::SurfaceError::MissingSupport)));
    snapshot.validate_body_motion(([1.05,1.05,0.4],[0.,0.,0.,1.]), body, &[]).unwrap();
    assert_eq!(selected.metadata.get("ground_reference_policy").map(String::as_str),
        Some("known_terrain_or_calibrated_current_layer"));
}

#[test]
fn unknown_layer_keeps_calibrated_observation_gain_and_prepared_ownership() {
    let snapshot = unknown_layer_map();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(snapshot.clone()).unwrap();
    let input = receiver.freeze().unwrap().fork_for_planning();
    let mut adapter = source(receiver, 1);
    adapter.set_observation_model(volume_model()).unwrap();
    let before = committed(&adapter);
    let preview = adapter.prepare_poll_with_planning_input(&state(0.4, 1), &input).unwrap();
    let selected = goal(preview.outcome().as_ref().unwrap().clone());
    assert!(selected.information_gain.unwrap() > 0.0);
    assert_eq!(committed(&adapter), before);
    adapter.discard_prepared_poll().unwrap();
    assert_eq!(committed(&adapter), before);
    let again = adapter.prepare_poll_with_planning_input(&state(0.4, 1), &input).unwrap();
    assert_eq!(again.outcome().as_ref().unwrap(), &ExplorationPoll::Goal(selected));
    adapter.commit_prepared_poll().unwrap();
    assert!(snapshot.frontier_cache_stats().is_none(), "private query cache must remain private");
    assert!(snapshot.observations().is_none());
}

fn unknown_corridor_boundary() -> Vec<[f64; 2]> {
    vec![[0.9,0.9],[1.9,0.9],[1.9,1.2],[0.9,1.2]]
}

fn unknown_corridor_cost(snapshot: &SurfaceSnapshot) -> Option<f64> {
    let tomogram = nuwax_nav_pct::Tomogram::from_surface(snapshot).unwrap();
    frontier_route_cost(&tomogram, snapshot, [1.05,1.05,0.1], [1.75,1.05,0.1],
        ([1.05,1.05,0.4],[0.,0.,0.,1.]), ([1.75,1.05,0.4],[0.,0.,0.,1.]),
        0.3, &unknown_corridor_boundary(), 0.4, &FrontierRouteBudget::default(), MAX_FRONTIER_ROUTE_STATES)
}

#[test]
fn unknown_frontier_routes_still_reject_recent_obstacles_and_virtual_boundaries() {
    let snapshot = unknown_layer_map();
    assert!(unknown_corridor_cost(&snapshot).is_some());
    let points: Vec<_> = (7..14).enumerate().map(|(index, y)|
        (index as u32, [1.4, (y as f64 + 0.5) * 0.1, 0.4])).collect();
    let recent = snapshot.with_recent_obstacle_points(1,1,0,&[],points).unwrap();
    assert!(unknown_corridor_cost(&recent).is_none(), "a blind floor cannot clear a recent wall");
    let cleared = recent.clear_recent_obstacles();
    assert!(nuwax_nav_pct::same_source_geometry(&recent, &cleared),
        "recent obstacle changes do not rebuild the persistent tomogram");
    assert!(!recent.same_frontier_query_geometry(&cleared));
    assert!(unknown_corridor_cost(&cleared).is_some(), "the same unknown route recovers after the actual wall clears");
    let mut blocked = vec![false; 30 * 30];
    for y in 0..30 { blocked[y * 30 + 14] = true; }
    let forbidden = snapshot.with_virtual_obstacles([0.,0.],0.1,30,30,&blocked).unwrap();
    assert!(unknown_corridor_cost(&forbidden).is_none(), "unknown expansion must not cross a virtual wall");
}

#[test]
fn unknown_candidate_sampling_is_finite_deduplicated_and_inside_boundary() {
    let snapshot = unknown_layer_map();
    let boundary = unknown_corridor_boundary();
    let geometry = frontier_geometry(&snapshot,[1.05,1.05,0.1],&boundary,Some(&volume_model())).unwrap();
    assert!(geometry.candidates.len() <= UNKNOWN_SAMPLE_DIRECTIONS * 4);
    assert!(!geometry.candidates.is_empty());
    let keys: std::collections::BTreeSet<_> = geometry.targets.iter().map(|target|target.cell).collect();
    assert_eq!(keys.len(), geometry.targets.len());
    assert!(geometry.candidates.iter().all(|candidate|
        nuwax_nav_pct::inside_boundary([candidate.support_xyz[0],candidate.support_xyz[1]],&boundary)));
}

#[test]
fn blocked_frontier_routes_share_one_conservative_state_budget() {
    let snapshot = unknown_layer_map();
    let wall = (7..14).enumerate().map(|(index, y)|
        (index as u32, [1.4, (y as f64 + 0.5) * 0.1, 0.4]));
    let snapshot = snapshot.with_recent_obstacle_points(1,1,0,&[],wall).unwrap();
    let tomogram = nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    let budget = FrontierRouteBudget::default();
    let began = std::time::Instant::now();
    for _ in 0..MAX_FRONTIER_ROUTE_QUERIES + 10 {
        assert!(frontier_route_cost(&tomogram, &snapshot,
            [1.05,1.05,0.1], [1.75,1.05,0.1],
            ([1.05,1.05,0.4],[0.,0.,0.,1.]), ([1.75,1.05,0.4],[0.,0.,0.,1.]),
            0.3, &unknown_corridor_boundary(), 0.4, &budget, MAX_FRONTIER_ROUTE_STATES).is_none());
    }
    assert!(budget.exhausted());
    assert_eq!(budget.remaining.get(), 0);
    assert_eq!(budget.queries.get(), MAX_FRONTIER_SEARCH_STATES / MAX_FRONTIER_ROUTE_STATES);
    eprintln!("blocked_unknown_routes elapsed_ms={} queries={} charged_states={}",
        began.elapsed().as_secs_f64()*1000.0, budget.queries.get(),
        MAX_FRONTIER_SEARCH_STATES-budget.remaining.get());
}

#[test]
#[ignore = "bounded release diagnostic for unknown and enclosed exploration polls"]
fn unknown_frontier_poll_work_profile() {
    for closed in [false, true] {
        let raw = unknown_layer_map();
        let snapshot = if closed {
            let walls = (7..14).flat_map(|i| {
                let p = (i as f64 + 0.5) * 0.1;
                [[0.85,p,0.4],[1.25,p,0.4],[p,0.85,0.4],[p,1.25,0.4]]
            });
            Arc::new(raw.with_recent_obstacle_points(1,1,0,&[],
                walls.enumerate().map(|(i,p)|(i as u32,p))).unwrap())
        } else { raw };
        for sample in 0..3 {
            let receiver=SurfaceInputReceiver::new();
            receiver.observe(snapshot.clone()).unwrap();
            let mut adapter=source(receiver,1);
            let began=std::time::Instant::now();
            let result=adapter.poll_goal(&state(0.4,1)).unwrap();
            let elapsed=began.elapsed().as_secs_f64()*1000.0;
            match result {
                ExplorationPoll::Goal(goal) => {
                    let charged:usize=goal.metadata["route_state_budget_charged"].parse().unwrap();
                    assert!(charged<=MAX_FRONTIER_SEARCH_STATES);
                    println!("{{\"closed\":{closed},\"sample\":{sample},\"elapsed_ms\":{elapsed},\"queries\":{},\"expanded_successes\":{},\"charged_states\":{charged}}}",
                        goal.metadata["route_query_count"],goal.metadata["route_expanded_successes"]);
                }
                result=>println!("{{\"closed\":{closed},\"sample\":{sample},\"elapsed_ms\":{elapsed},\"outcome\":\"{result:?}\"}}"),
            }
        }
    }
}

#[test]
fn inconclusive_candidate_budget_grows_across_receipts_and_pose_jitter() {
    let snapshot = unknown_layer_map();
    let snapshot = Arc::new(snapshot.with_recent_obstacle_points(1,1,0,&[],
        [(0,[1.25,1.05,0.4])]).unwrap());
    let receiver=SurfaceInputReceiver::new();
    receiver.observe(snapshot.clone()).unwrap();
    let mut adapter=source(receiver.clone(),1);
    adapter.deferred_route=Some(DeferredFrontierRoute {
        source:snapshot.as_ref().clone(),
        candidate:FrontierCandidate {support_xyz:[1.65,1.05,0.1],gain:0.01,travel_cost:0.6},
        start_body:[1.05,1.05,0.4],start_orientation:[0.,0.,0.,1.],resume_cursor:usize::MAX,unresolved:false,state_limit:1,terminal:false,
    });
    let before=committed(&adapter);
    let prepared=adapter.prepare_poll(&state(0.4,1)).unwrap();
    assert_eq!(prepared.outcome().as_ref().unwrap(),&ExplorationPoll::Pending);
    assert_eq!(committed(&adapter),before);
    adapter.discard_prepared_poll().unwrap();
    assert_eq!(committed(&adapter),before);
    let mut previous=1;
    let mut selected=None;
    for round in 0..16 {
        // New receipt and Arc wrapper, same physical source geometry.
        receiver.observe(Arc::new(snapshot.as_ref().clone())).unwrap();
        let mut current=state(0.4,round+2);
        current.robot_pose.pose.position.x+=round as f64*0.001;
        match adapter.poll_goal(&current).unwrap() {
            ExplorationPoll::Pending=>{
                let next=adapter.deferred_route.as_ref().expect("retain inconclusive candidate");
                assert!(next.state_limit>previous,"same geometry must advance, not reset its budget");
                assert_eq!(next.start_body,[1.05,1.05,0.4]);
                previous=next.state_limit;
            }
            ExplorationPoll::Goal(goal)=>{selected=Some(goal);break;}
            other=>panic!("bounded detour must eventually resolve: {other:?}"),
        }
    }
    let selected=selected.expect("progressive search should reach a small detour");
    assert!(adapter.deferred_route.is_none());
    assert!(selected.metadata["route_state_budget_charged"].parse::<usize>().unwrap()
        <= selected.metadata["route_state_budget_limit"].parse::<usize>().unwrap());
}

#[test]
fn exhausted_resource_stage_is_explicit_and_does_not_revoke_source() {
    let snapshot=unknown_layer_map();
    let snapshot=Arc::new(snapshot.with_recent_obstacle_points(1,1,0,&[],
        [(0,[5.0,5.0,0.4])]).unwrap());
    let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot.clone()).unwrap();
    let mut adapter=source(receiver.clone(),1);
    adapter.deferred_route=Some(DeferredFrontierRoute {
        source:snapshot.as_ref().clone(),
        candidate:FrontierCandidate {support_xyz:[1.65,1.05,0.1],gain:0.01,travel_cost:0.6},
        start_body:[1.05,1.05,0.4],start_orientation:[0.,0.,0.,1.],resume_cursor:usize::MAX,unresolved:true,state_limit:nuwax_nav_pct::SearchLimits::default().max_states,terminal:true,
    });
    let result=adapter.poll_goal(&state(0.4,1)).unwrap();
    assert_eq!(result,ExplorationPoll::Stop(crate::ExplorationStopReason::PlanningResourceLimited));
    assert!(adapter.diagnostics().last_error.as_deref().is_some_and(|reason|
        reason.starts_with("ResourceLimited:") && reason.contains("inconclusive")));
    assert!(!adapter.last_no_frontier);
    assert!(receiver.snapshot().unwrap().is_some());
    assert!(adapter.active);
    // An independently changed recent-obstacle view must release a terminal
    // resource stage even if the persistent terrain map has not changed.
    receiver.observe(Arc::new(snapshot.clear_recent_obstacles())).unwrap();
    assert!(matches!(adapter.poll_goal(&state(0.4,2)).unwrap(), ExplorationPoll::Goal(_)));
    assert!(adapter.deferred_route.is_none());
}

#[test]
fn unknown_to_known_ground_keeps_the_selected_goal_until_exact_feedback() {
    let unknown=unknown_layer_map();
    let receiver=SurfaceInputReceiver::new();receiver.observe(unknown.clone()).unwrap();
    let mut adapter=source(receiver.clone(),1);
    let selected=goal(adapter.poll_goal(&state(0.4,1)).unwrap());
    let known=Arc::new(map().as_ref().clone()
        .with_body_envelope(unknown.body_envelope().unwrap().clone()).unwrap());
    assert!(known.query_ground([1.05,1.05],0.1).height.is_some());
    receiver.observe(known).unwrap();
    assert_eq!(adapter.poll_goal(&state(0.4,2)).unwrap(),ExplorationPoll::Pending);
    assert_eq!(adapter.diagnostics().last_goal_id.as_deref(),Some(selected.id.as_str()));
    assert_eq!(adapter.selected.as_ref().unwrap().goal,selected);
    assert!(adapter.history.is_empty());
}

#[test]
fn calibrated_height_bias_keeps_exact_body_endpoint_checks() {
    let shape=unknown_layer_map().body_envelope().unwrap().clone();
    let snapshot=map().as_ref().clone().with_body_envelope(shape).unwrap();
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    for bias in [-0.02,0.0,0.02] {
        let from=([1.05,1.05,0.4+bias],[0.,0.,0.,1.]);
        let to=snapshot.body_pose_after_motion(from,[1.65,1.05],0.0).unwrap();
        assert!((to.0[2]-from.0[2]).abs()<1e-6, "actual calibration bias is retained");
        let start=snapshot.support_position_from_body(from.0,from.1,0.3).unwrap();
        let goal=snapshot.support_position_from_body(to.0,to.1,0.3).unwrap();
        assert!(frontier_route_cost(&tomogram,&snapshot,start,goal,from,to,0.3,&[],0.4,
            &FrontierRouteBudget::default(),MAX_FRONTIER_ROUTE_STATES).is_some(),
            "declared tolerance must not turn valid {bias}m calibration bias into missing ground");
        let blocked=snapshot.with_recent_obstacle_points(1,1,0,&[],[(0,to.0)]).unwrap();
        assert!(frontier_route_cost(&tomogram,&blocked,start,goal,from,to,0.3,&[],0.4,
            &FrontierRouteBudget::default(),MAX_FRONTIER_ROUTE_STATES).is_none(),
            "reference tolerance cannot replace the exact collision-blocked body endpoint");
    }
}

#[test]
fn geometric_seed_discovery_does_not_inherit_observed_graph_clearance_admission() {
    let mut config=map().config().clone();config.wall_clearance_m=10.0;
    let points:Vec<_>=(0..20).flat_map(|x|(0..20).map(move|y|
        ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
    let shape=unknown_layer_map().body_envelope().unwrap().clone();
    let snapshot=Arc::new(SurfaceSnapshot::from_points(&points,config,0.1).unwrap()
        .with_body_envelope(shape).unwrap());
    let boundary=vec![[0.5,0.5],[2.1,0.5],[2.1,1.5],[0.5,1.5]];
    // The actual body fits; oversized legacy graph clearance is not a physical
    // collision. All fallback rings lie outside this boundary, requiring seeds.
    snapshot.validate_body_pose([1.05,1.05,0.4],[0.,0.,0.,1.]).unwrap();
    assert!(!snapshot.frontier_candidates(&boundary).unwrap().is_empty());
    let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot.clone()).unwrap();
    let mut adapter=source(receiver,1);
    adapter.configure_constraints(Some(ExplorationBoundary::from_vertices(
        boundary.iter().map(|p|Point2::new(p[0],p[1])).collect()).unwrap()),1,0.3).unwrap();
    let selected=goal(adapter.poll_goal(&state(0.4,1)).unwrap());
    let p=selected.pose.pose.position;let q=selected.pose.pose.orientation;
    snapshot.validate_body_motion(([1.05,1.05,0.4],[0.,0.,0.,1.]),
        ([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]),&boundary).unwrap();
}

#[test]
fn definitive_final_candidate_exhaustion_does_not_leave_an_unresumable_wait() {
    let mut config=map().config().clone();
    config.wall_clearance_m=0.2; // Keep the four-reference 0.4 m sampling horizon.
    let mut envelope=unknown_layer_map().body_envelope().unwrap().clone();
    envelope.footprint_xy_m=vec![[-0.02,-0.02],[0.02,-0.02],[0.02,0.02],[-0.02,0.02]];
    let raw=SurfaceSnapshot::from_points(&[(10.05,10.05,0.05)],config,0.1)
        .unwrap().with_body_envelope(envelope).unwrap();
    let snapshot=Arc::new(raw.with_recent_obstacle_points(1,1,0,&[],
        [(0,[0.95,1.05,0.4]),(1,[1.15,1.05,0.4])]).unwrap());
    // Four finite reference candidates fit inside this narrow strip. Actual
    // recent walls isolate the collision-free start from all four endpoints;
    // the fourth definitive failure exactly spends the initial reservation.
    let boundary=vec![[0.6,1.049],[1.5,1.049],[1.5,1.051],[0.6,1.051]];
    let geometry=frontier_geometry(&snapshot,[1.05,1.05,0.1],&boundary,None).unwrap();
    assert_eq!(geometry.candidates.len(),MAX_FRONTIER_SEARCH_STATES/MAX_FRONTIER_ROUTE_STATES);
    let current=([1.05,1.05,0.4],[0.,0.,0.,1.]);
    snapshot.validate_body_pose(current.0,current.1).unwrap();
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    let budget=FrontierRouteBudget::default();
    for candidate in &geometry.candidates {
        let point=candidate.support_xyz;
        let yaw=(point[1]-current.0[1]).atan2(point[0]-current.0[0]);
        let endpoint=snapshot.body_pose_after_motion(current,[point[0],point[1]],yaw).unwrap();
        snapshot.validate_body_pose(endpoint.0,endpoint.1).unwrap();
        assert!(frontier_route_cost(&tomogram,&snapshot,[1.05,1.05,0.1],point,
            current,endpoint,0.3,&boundary,geometry.lookahead_m,&budget,MAX_FRONTIER_ROUTE_STATES).is_none());
        assert!(!budget.last_inconclusive.get(),"a real wall and physical boundary seal this route");
    }
    assert_eq!(budget.queries.get(),4);
    assert_eq!(budget.remaining.get(),0);
    let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot).unwrap();
    let mut adapter=source(receiver,1);
    adapter.configure_constraints(Some(ExplorationBoundary::from_vertices(
        boundary.iter().map(|p|Point2::new(p[0],p[1])).collect()).unwrap()),1,0.3).unwrap();
    assert_eq!(adapter.poll_goal(&state(0.4,1)).unwrap(),ExplorationPoll::NoFrontier);
    assert!(adapter.deferred_route.is_none());
    assert_eq!(adapter.route_cursor,0);
    assert_eq!(adapter.poll_goal(&state(0.4,2)).unwrap(),ExplorationPoll::NoFrontier);
}

fn planar_unknown_map() -> SurfaceSnapshot {
    SurfaceSnapshot::from_points(&[(10.05,10.05,0.05)],map().config().clone(),0.1).unwrap()
}

fn poll_planar_candidate(snapshot: Arc<SurfaceSnapshot>, target: [f64;3],
    boundary: &[[f64;2]]) -> ExplorationPoll {
    let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot.clone()).unwrap();
    let mut adapter=source(receiver,1);
    adapter.configure_constraints(Some(ExplorationBoundary::from_vertices(
        boundary.iter().map(|p|Point2::new(p[0],p[1])).collect()).unwrap()),1,0.3).unwrap();
    adapter.deferred_route=Some(DeferredFrontierRoute {
        source:snapshot.as_ref().clone(),
        candidate:FrontierCandidate {support_xyz:target,gain:0.01,travel_cost:0.8},
        start_body:[1.05,1.05,0.4],start_orientation:[0.,0.,0.,1.],resume_cursor:usize::MAX,unresolved:false,state_limit:MAX_FRONTIER_ROUTE_STATES,terminal:false,
    });
    adapter.poll_goal(&state(0.4,1)).unwrap()
}

#[test]
fn planar_frontier_selection_matches_current_recent_sweep_and_recovers() {
    let raw=planar_unknown_map();
    let boundary=unknown_corridor_boundary();
    let start=[1.05,1.05,0.1];let target=[1.75,1.05,0.1];
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&raw).unwrap();
    let projected=tomogram.project_reference_motion(start,[target[0],target[1]],&boundary).unwrap();
    raw.validate_planar_recent_motion(start,projected).unwrap();
    assert!(matches!(poll_planar_candidate(Arc::new(raw.clone()),target,&boundary),ExplorationPoll::Goal(_)));
    let wall=(7..14).enumerate().map(|(i,y)|(i as u32,[1.4,(y as f64+0.5)*0.1,0.4]));
    let blocked=raw.with_recent_obstacle_points(1,1,0,&[],wall).unwrap();
    assert!(blocked.validate_planar_recent_motion(start,projected).is_err());
    assert_eq!(poll_planar_candidate(Arc::new(blocked.clone()),target,&boundary),ExplorationPoll::NoFrontier);
    // The cached static reference is deliberately reused with the new source.
    assert!(frontier_route_cost(&tomogram,&blocked,start,target,
        ([1.05,1.05,0.4],[0.,0.,0.,1.]),([1.75,1.05,0.4],[0.,0.,0.,1.]),
        0.3,&boundary,0.4,&FrontierRouteBudget::default(),MAX_FRONTIER_ROUTE_STATES).is_none());
    let cleared=blocked.clear_recent_obstacles();
    cleared.validate_planar_recent_motion(start,projected).unwrap();
    assert!(matches!(poll_planar_candidate(Arc::new(cleared),target,&boundary),ExplorationPoll::Goal(_)));
}

#[test]
fn planar_frontier_selection_matches_known_narrow_corridor_rejection() {
    let mut points:Vec<_>=(8..21).flat_map(|x|(8..13).map(move|y|
        ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
    for x in 13..16 {for y in [9,11] {for z in 1..7 {
        points.push(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,(z as f32+0.5)*0.1));
    }}}
    let snapshot=Arc::new(SurfaceSnapshot::from_points(&points,map().config().clone(),0.1).unwrap());
    let boundary=vec![[0.9,0.9],[2.0,0.9],[2.0,1.2],[0.9,1.2]];
    let start=[1.05,1.05,0.1];let target=[1.85,1.05,0.1];
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    assert!(tomogram.project_reference_motion(start,[start[0],start[1]],&boundary).is_ok());
    assert!(tomogram.project_reference_motion(target,[target[0],target[1]],&boundary).is_ok());
    assert!(tomogram.project_reference_motion(start,[target[0],target[1]],&boundary).is_err(),
        "the configured circular body cannot fit between the known walls");
    assert_eq!(poll_planar_candidate(snapshot,target,&boundary),ExplorationPoll::NoFrontier);
}

#[test]
fn planar_routes_preserve_calibrated_height_bias_and_exact_blocked_endpoints() {
    let snapshot=map();
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    for bias in [-0.02,0.0,0.02] {
        let start=[1.05,1.05,0.1+bias];let target=[1.65,1.05,0.1+bias];
        let from=([start[0],start[1],start[2]+0.3],[0.,0.,0.,1.]);
        let to=([target[0],target[1],target[2]+0.3],[0.,0.,0.,1.]);
        assert!(frontier_route_cost(&tomogram,&snapshot,start,target,from,to,0.3,&[],0.4,
            &FrontierRouteBudget::default(),MAX_FRONTIER_ROUTE_STATES).is_some());
        let blocked=snapshot.with_recent_obstacle_points(1,1,0,&[],[(0,to.0)]).unwrap();
        assert!(frontier_route_cost(&tomogram,&blocked,start,target,from,to,0.3,&[],0.4,
            &FrontierRouteBudget::default(),MAX_FRONTIER_ROUTE_STATES).is_none());
    }
}

#[test]
fn planar_candidate_discovery_preserves_recent_obstacle_detours() {
    let raw=planar_unknown_map();
    let snapshot=Arc::new(raw.with_recent_obstacle_points(1,1,0,&[],[(0,[1.4,1.05,0.4])]).unwrap());
    let boundary=vec![[0.7,0.6],[2.1,0.6],[2.1,1.5],[0.7,1.5]];
    let start=[1.05,1.05,0.1];let target=[1.75,1.05,0.1];
    assert!(snapshot.validate_planar_recent_motion(start,target).is_err(),"the straight connector is blocked");
    let selected=goal(poll_planar_candidate(snapshot,target,&boundary));
    assert!((selected.pose.pose.position.x-target[0]).abs()<1e-6);
    assert!((selected.pose.pose.position.y-target[1]).abs()<1e-6);
    assert!(selected.metadata["route_expanded_successes"].parse::<usize>().unwrap()>0,
        "the shared planner must establish a detour rather than rejecting candidate discovery");
}

#[test]
fn deferred_unreachable_or_capped_candidate_preserves_the_reachable_suffix() {
    let raw=unknown_layer_map();
    let wall=(7..14).enumerate().map(|(i,y)|(i as u32,[1.4,(y as f64+0.5)*0.1,0.4]));
    let snapshot=Arc::new(raw.with_recent_obstacle_points(1,1,0,&[],wall).unwrap());
    let boundary=unknown_corridor_boundary();let reference=[1.05,1.05,0.1];
    let current=([1.05,1.05,0.4],[0.,0.,0.,1.]);
    let mut candidates=frontier_geometry(&snapshot,reference,&boundary,None).unwrap().candidates;
    candidates.sort_by(|a,b|(b.gain/(1.0+b.travel_cost)).total_cmp(&(a.gain/(1.0+a.travel_cost)))
        .then_with(||a.support_xyz[0].total_cmp(&b.support_xyz[0]))
        .then_with(||a.support_xyz[1].total_cmp(&b.support_xyz[1]))
        .then_with(||a.support_xyz[2].total_cmp(&b.support_xyz[2])));
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    // B must pass the actual candidate heading and shared path admission, not
    // merely a yaw-zero endpoint check near the recent wall.
    let resume_cursor=candidates.iter().rposition(|candidate| {
        let yaw=(candidate.support_xyz[1]-reference[1]).atan2(candidate.support_xyz[0]-reference[0]);
        let Ok(pose)=snapshot.body_pose_after_motion(current,
            [candidate.support_xyz[0],candidate.support_xyz[1]],yaw) else {return false;};
        let projected=snapshot.support_position_from_body(pose.0,pose.1,0.3).unwrap();
        distance(reference,projected)>f64::from(snapshot.config().goal_tolerance)
            && frontier_route_cost(&tomogram,&snapshot,reference,projected,current,pose,0.3,&boundary,0.4,
                &FrontierRouteBudget::default(),MAX_FRONTIER_ROUTE_STATES).is_some()
    }).unwrap();
    assert!(resume_cursor>0,"B is beyond the earlier candidate cutoff");
    for capped in [false,true] {
        let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot.clone()).unwrap();
        let mut adapter=source(receiver,1);
        adapter.configure_constraints(Some(ExplorationBoundary::from_vertices(
            boundary.iter().map(|p|Point2::new(p[0],p[1])).collect()).unwrap()),1,0.3).unwrap();
        adapter.deferred_route=Some(DeferredFrontierRoute {
            source:snapshot.as_ref().clone(),
            candidate:FrontierCandidate {support_xyz:[1.75,1.05,0.1],gain:0.01,travel_cost:0.7},
            start_body:current.0,start_orientation:current.1,resume_cursor,unresolved:capped,
            state_limit:if capped {nuwax_nav_pct::SearchLimits::default().max_states}else{1},terminal:capped,
        });
        let mut selected=None;let mut saw_definitive_resume=false;
        for stamp in 1..20 {
            let before=adapter.deferred_route.as_ref().map(|route|
                (route.candidate.support_xyz,route.resume_cursor,route.state_limit,route.terminal,route.unresolved));
            let poll=adapter.poll_goal(&state(0.4,stamp)).unwrap();
            match poll {
                ExplorationPoll::Goal(goal)=>{selected=Some(goal);break;}
                ExplorationPoll::Pending=>{
                    if let Some(route)=&adapter.deferred_route {
                        assert_eq!(route.resume_cursor,resume_cursor);
                        saw_definitive_resume|=route.terminal&&!route.unresolved;
                    }
                }
                result=>panic!("A cannot certify that the unvisited B is unreachable: capped={capped} stamp={stamp} before={before:?} {result:?}"),
            }
        }
        assert!(selected.is_some(),"the reachable suffix must eventually be queried");
        if !capped {assert!(saw_definitive_resume,"the expensive A is definitively rejected before resuming B");}
        assert!(adapter.history.is_empty());
        assert!(adapter.deferred_route.is_none());
    }
}

#[test]
fn finite_frontier_disk_does_not_certify_a_long_wall_unreachable() {
    let raw=planar_unknown_map();
    let wall=(-1..22).enumerate().map(|(i,y)|(i as u32,[1.35,(y as f64+0.5)*0.1,0.4]));
    let snapshot=Arc::new(raw.with_recent_obstacle_points(1,1,0,&[],wall).unwrap());
    let boundary=vec![[-0.1,-0.3],[2.8,-0.3],[2.8,2.5],[-0.1,2.5]];
    let start=[1.05,1.05,0.1];let target=[1.65,1.05,0.1];
    let from=([1.05,1.05,0.4],[0.,0.,0.,1.]);let to=([1.65,1.05,0.4],[0.,0.,0.,1.]);
    let tomogram=nuwax_nav_pct::Tomogram::from_surface(&snapshot).unwrap();
    let short=FrontierRouteBudget::default();
    assert!(frontier_route_cost(&tomogram,&snapshot,start,target,from,to,0.3,&boundary,0.1,
        &short,MAX_FRONTIER_ROUTE_STATES).is_none());
    assert!(short.last_domain_limited.get(),"an artificial disk is not the physical mission boundary");
    let larger=FrontierRouteBudget::with_states(nuwax_nav_pct::SearchLimits::default().max_states);
    assert!(frontier_route_cost(&tomogram,&snapshot,start,target,from,to,0.3,&boundary,2.0,
        &larger,nuwax_nav_pct::SearchLimits::default().max_states).is_some(),"the outer detour really exists");
    assert_eq!(poll_planar_candidate(snapshot,target,&boundary),
        ExplorationPoll::Stop(crate::ExplorationStopReason::PlanningResourceLimited));
}

#[test]
fn planar_unknown_edge_candidate_keeps_the_current_known_layer_bias() {
    let snapshot=map();
    for bias in [-0.02,0.02] {
        let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot.clone()).unwrap();
        let mut adapter=source(receiver,1);
        adapter.deferred_route=Some(DeferredFrontierRoute {
            source:snapshot.as_ref().clone(),
            candidate:FrontierCandidate {support_xyz:[2.15,1.05,0.1],gain:0.01,travel_cost:1.1},
            start_body:[1.05,1.05,0.4+bias],start_orientation:[0.,0.,0.,1.],
            resume_cursor:usize::MAX,unresolved:false,state_limit:MAX_FRONTIER_ROUTE_STATES,terminal:false,
        });
        let selected=goal(adapter.poll_goal(&state(0.4+bias,1)).unwrap());
        assert!((selected.pose.pose.position.z-(0.4+bias)).abs()<1e-6);
        assert!(snapshot.query_ground([2.15,1.05],0.1).height.is_none());
    }
}
