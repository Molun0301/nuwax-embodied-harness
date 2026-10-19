// Fixed existing Scan inputs; real C1-a source owners are prepared separately.
fn c1b_route(map:&SurfaceSnapshot,start:BodyPose,to:[f64;2])->nuwax_nav_pct::ground_route::PlannedGroundRoute {
    use nuwax_nav_pct::ground_route::{GroundRouteStart,GroundRouteGoal};
    eprintln!("C1B_REAL_ROUTE start={start:?} requested_xy={to:?}");
    let result=nuwax_nav_pct::PctPlanner::default().ground_route_with_limits_and_penalties(map,start,
        GroundRouteStart {xy:[start.0[0],start.0[1]],ground_z:None},
        GroundRouteGoal {xy:to,requested_ground_z:None},0.0,&[],None,None,
        nuwax_nav_pct::SearchLimits::default(),&[]);
    eprintln!("C1B_REAL_ROUTE result={:?}",result.as_ref().map(|r|(&r.points,&r.stats)));
    result.expect("independent real Ground route fixture")
}
fn c1b_compare(map:&SurfaceSnapshot,tomo:&Tomogram,start:BodyPose,reference:&[[f64;3]],
    arrival:Option<([f64;3],f64)>,boundary:&[[f64;2]],progress:Option<ProgressPrefixPolicy<'_>>)
    ->MappedLocalTrajectory {
    let route=c1b_route(map,start,[reference.last().unwrap()[0],reference.last().unwrap()[1]]);
    c1b_compare_window(map,tomo,start,reference,arrival,boundary,progress,route.reference.full_window().unwrap())
}
fn c1b_compare_window(map:&SurfaceSnapshot,tomo:&Tomogram,start:BodyPose,reference:&[[f64;3]],
    arrival:Option<([f64;3],f64)>,boundary:&[[f64;2]],progress:Option<ProgressPrefixPolicy<'_>>,
    window:nuwax_nav_pct::ground_route::GroundRouteWindow<'_>)->MappedLocalTrajectory {
    let ground=map.body_query().start_ground_state(start).unwrap();
    GROUND_MAPPING_WORK.set(0);
    let old=plan_with_progress_from_ground(map,tomo,start,&ground,reference,&motion(),&ScanConfig::default(),
        None,arrival,boundary,false,false,progress).unwrap();
    assert_eq!(GROUND_MAPPING_WORK.get(),0,"Body never enters new mapping allocations/source walk");
    let mapped=plan_mapped_ground(map,tomo,start,&ground,reference,&motion(),&ScanConfig::default(),None,
        arrival,boundary,false,false,progress,window).unwrap();
    assert_eq!(old.poses,mapped.trajectory.poses);assert_eq!(old.partial,mapped.trajectory.partial);
    assert_eq!(old.smoothed,mapped.trajectory.smoothed);
    assert_eq!(mapped.binding.edges().len(),old.poses.len()-1);
    assert_eq!(GROUND_MAPPING_WORK.get(),old.poses.len()-1);
    for (i,pair) in old.poses.windows(2).enumerate() {
        assert_eq!(mapped.binding.edges()[i].motion(),[pair[0],pair[1]]);
        let (from,to,steps)=mapped.binding.candidate().connection(i).unwrap();
        assert_eq!(from,[pair[0].0[0],pair[0].0[1]]);assert_eq!(to,[pair[1].0[0],pair[1].0[1]]);
        assert!(steps>=1);
        for sample in 0..=steps {
            assert!(mapped.binding.candidate().sample(i,sample).unwrap().point.ground_z.is_none());
        }
    }
    mapped
}
#[test]
fn c1b_03_real_winner_parent_connector_and_partial() {
    let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
    let start=([-1.0,0.2,0.4],[0.,0.,0.,1.]);let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];
    let direct=c1b_compare(&map,&tomo,start,&reference,Some(([0.1,0.2,0.4],0.1)),&[],None);
    assert_eq!(direct.binding.family(),CandidateFamily::Direct);
    let (map,start)=fine_escape_scene(false,[1,0],false,2);let tomo=Tomogram::from_surface(&map).unwrap();
    let goal=[start.0[0]+0.01,start.0[1]+0.5,start.0[2]];
    let reference=[[start.0[0],start.0[1],0.0],[goal[0],goal[1],0.0]];
    // The original full-goal PCT premise failed on this unchanged scene: its
    // fixed-yaw grid cannot express the validated 1 cm departure before turning.
    // This separate intent window is the existing fixture's real straight
    // departure, prepared before Scan. It is not a full Ground route to goal.
    let departure=c1b_route(&map,start,[goal[0],start.0[1]]);
    assert_eq!(departure.points.last().unwrap().xy,[goal[0],start.0[1]]);
    let search=c1b_compare_window(&map,&tomo,start,&reference,Some((goal,0.015)),&[],None,
        departure.reference.full_window().unwrap());
    assert!(!search.binding.planned_join());assert!(!search.binding.covers_original_goal());
    assert_eq!(search.binding.family(),CandidateFamily::Search);assert_eq!(local_search::last_work().0,1);
    assert!(search.binding.edges().iter().any(|edge|matches!(edge.origin(),CandidateEdgeOrigin::Parent {node} if node>0)));
    assert!(search.binding.edges().iter().any(|edge|matches!(edge.origin(),CandidateEdgeOrigin::Connector {node} if node>0)));
    assert!(!search.trajectory.partial);
    let (map,tomo,start,reference,boundary)=progress_prefix_fixture();
    // Full-goal physical PCT did succeed on this scene, but took 53,280
    // expansions before the interrupted qualification reached this mapping.
    // The bounded control instead prepares the original minimum-progress
    // distance as an explicit real, wall-before initial window. It does not
    // replace the original Scan reference, boundary, arrival or default budget.
    let prefix=c1b_route(&map,start,[start.0[0]+0.375,start.0[1]]);
    let partial=c1b_compare_window(&map,&tomo,start,&reference,Some(([1.,0.2,0.4],0.1)),&boundary,
        Some(ProgressPrefixPolicy {reference:&reference,minimum_distance_m:0.375}),
        prefix.reference.full_window().unwrap());
    assert_eq!(local_search::last_work().0,256);assert!(partial.trajectory.partial);
    assert_eq!(partial.binding.family(),CandidateFamily::Search);
    assert!(!partial.binding.planned_join());assert!(!partial.binding.covers_original_goal());
    assert!(partial.binding.edges().iter().any(|edge|matches!(edge.origin(),CandidateEdgeOrigin::Parent {..})));
    assert!(!partial.binding.edges().iter().any(|edge|matches!(edge.origin(),CandidateEdgeOrigin::Connector {..})));
    eprintln!("families observed: Direct, Search parent+connector, Search partial; StepTurn/Rejoin/Spline/Passage not covered by this fixed group");
}
#[test]
fn c1b_04_original_sampling_window_and_body_zero_sidecar() {
    let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
    let start=([-1.0,0.2,0.4],[0.,0.,0.,1.]);let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];
    let mapped=c1b_compare(&map,&tomo,start,&reference,None,&[],None);
    let edge=mapped.binding.edges().iter().position(|e|e.motion()[0].0!=e.motion()[1].0).unwrap();
    let (_,_,steps)=mapped.binding.candidate().connection(edge).unwrap();
    let cut=0.5/steps as f64;
    let slice=mapped.binding.slice(edge,cut,1.0).unwrap();
    assert_eq!(slice.original_sample(0.0),Some((0,0.5)));
    assert_eq!(slice.original_sample(1.0),Some((steps,0.0)));
    assert_eq!(slice.binding().candidate().connection(edge).unwrap().2,steps);
    assert!(mapped.binding.slice(edge,0.8,0.2).is_none());
    let route=c1b_route(&map,start,[0.1,0.2]);let ground=map.body_query().start_ground_state(start).unwrap();
    let cancelled=cancellation::after_checkpoints(0,||plan_mapped_ground(&map,&tomo,start,&ground,&reference,
        &motion(),&ScanConfig::default(),None,None,&[],false,false,None,route.reference.full_window().unwrap()));
    assert!(matches!(cancelled,Err(GroundCandidateError::Planning(LocalPlanError {kind:LocalPlanFailureKind::Cancelled,..}))));
    let selected=LocalTrajectory {partial:mapped.trajectory.partial,poses:mapped.trajectory.poses.clone(),
        smoothed:mapped.trajectory.smoothed,elapsed_ms:mapped.trajectory.elapsed_ms};
    let expected=selected.poses.clone();
    let cancelled=cancellation::after_checkpoints(0,||map_selected_candidate(selected,
        CandidateOrigins {family:CandidateFamily::Direct,parts:vec![]},route.reference.full_window().unwrap(),&[]));
    match cancelled {Err(GroundCandidateError::Cancelled {trajectory,error})=> {
        assert_eq!(error.kind,LocalPlanFailureKind::Cancelled);assert_eq!(trajectory.poses,expected);
    },other=>panic!("post-winner cancellation lost typed cause: {other:?}")}
    eprintln!("window is original grid; same-pose future-state search dominance NOT_COVERED: reference does not yet affect physical successors");
}
