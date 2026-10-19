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

// C1-c2 fixed source-backed controls. Historical C1-b fixtures/results above
// retain their post-winner-only meaning. These enter actual successor projection.
#[derive(Clone,Copy)]
enum C2NativeFixture { LowCeiling, SingleStep, BinShift }
fn c2_native_scene(two_floors:bool)->SurfaceSnapshot {
    c2_native_fixture(if two_floors {C2NativeFixture::LowCeiling}else{C2NativeFixture::SingleStep})
}
fn c2_native_fixture(fixture:C2NativeFixture)->SurfaceSnapshot {
    use nuwax_map_chunks::{ColumnGroundRecords,HeightfieldSurface,Domain,Receiver,Repository,Transfer,Writer};
    use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
    use std::collections::BTreeSet;
    let q=0.05_f32;
    let mut writer=Writer::new(Repository::temporary().unwrap(),q,4*1024*1024).unwrap();
    let mut points=Vec::new();let mut columns=Vec::new();
    for x in -8..17 {for y in -8..17 {
        let levels=match fixture {
            C2NativeFixture::LowCeiling=>vec![(0,0),(if x>=1 {6}else{5},0)],
            C2NativeFixture::SingleStep=>vec![(if x>=0 {3}else{0},0)],
            C2NativeFixture::BinShift=>vec![(0,7),(11,if x>=1 {7}else{0})],
        };
        let mut heightfields=Vec::new();
        for (z,bin) in levels {
            writer.observe([[x,y,z]],20).unwrap();
            // Historical fixtures retain their original occupancy-centre points.
            // The independent bin-shift fixture also locates its raw geometry
            // point at the same actual q/8 bin used by the admitted statistics.
            let fraction=if matches!(fixture,C2NativeFixture::BinShift) {(bin as f32+0.5)/8.0}else{0.5};
            points.push(((x as f32+0.5)*q,(y as f32+0.5)*q,(z as f32+fraction)*q));
            let mut counts=[0;8];counts[bin]=1000;
            heightfields.push(HeightfieldSurface {column:HeightfieldColumn::from_counts([x,y],q,
                BTreeMap::from([(z,counts)])).unwrap().unwrap(),samples:vec![]});
        }
        columns.push(([x,y],ColumnGroundRecords {classified_empty:false,legacy:vec![],heightfields}));
    }}
    writer.replace_ground_columns(columns.iter().map(|(xy,c)|(*xy,c))).unwrap();
    let root=writer.snapshot(Domain {session:"c1c2-native-fixed".into(),generation:1,
        frame_generation:0,producer_generation:1},1,20).unwrap();
    let mut receiver=Receiver::new(Repository::temporary().unwrap());
    receiver.receive(&Transfer {root:root.encode().unwrap(),included:BTreeMap::new()},&root.domain,
        |_,r|writer.repository().read_object(r.object,nuwax_map_chunks::MAX_BLOCK_BYTES)).unwrap();
    let mut observations=SurfaceObservations::new(q,32).unwrap();let mut parts=Vec::new();
    for r in &root.blocks {
        let (block,records)=receiver.ground_parts(r).unwrap();
        observations.replace_block(r.key,block.iter().map(|(&local,e)|
            (nuwax_map_chunks::global_key(r.key,local).unwrap(),SurfaceObservation {observed_ns:e.observed_ns,hits:e.hits})).collect()).unwrap();
        parts.push((r.key,records));
    }
    for xy in parts.iter().map(|(key,_)|[key[0],key[1]]).collect::<BTreeSet<_>>() {
        let columns=parts.iter().filter(|(key,_)|key[..2]==xy).cloned().collect::<Vec<_>>();
        observations.replace_heightfield_columns(xy,&columns);
    }
    observations.mark_complete_geometry_source();
    let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.2,surface_closing_radius:0.0,
        node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.2,wall_buffer_weight:2.0,
        max_slope_deg:18.0,step_threshold_m:0.21,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    let model=SurfaceBodyEnvelope {fixed_collision:None,
        footprint_xy_m:vec![[-0.01,-0.01],[0.01,-0.01],[0.01,0.01],[-0.01,0.01]],
        z_bounds_m:[-0.1,0.1],body_to_support_z_m:0.4,max_step_up_m:0.21,max_step_down_m:0.21,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    SurfaceSnapshot::from_points(&points,config,q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}
fn c2_route(planner:&nuwax_nav_pct::PctPlanner,map:&SurfaceSnapshot,start:BodyPose,height:Option<f64>,to:[f64;2])
    ->nuwax_nav_pct::ground_route::PlannedGroundRoute {
    use nuwax_nav_pct::ground_route::{GroundRouteStart,GroundRouteGoal};
    planner.ground_route_with_limits_and_penalties(map,start,
        GroundRouteStart {xy:[start.0[0],start.0[1]],ground_z:height},
        GroundRouteGoal {xy:to,requested_ground_z:None},0.0,&[],None,None,
        nuwax_nav_pct::SearchLimits::default(),&[]).expect("fixed real navigation source/initial route")
}
fn c2_state<'a>(map:&SurfaceSnapshot,start:BodyPose,route:&'a nuwax_nav_pct::ground_route::PlannedGroundRoute)->MotionState<'a> {
    MotionState {physical:map.body_query().start_ground_state(start).unwrap(),
        navigation:Some(route.reference.full_window().unwrap().from().walk_state())}
}
fn c2_assert_binding(mapped:&MappedLocalTrajectory) {
    assert_eq!(mapped.binding.edges().len(),mapped.trajectory.poses.len()-1);
    for (i,pair) in mapped.trajectory.poses.windows(2).enumerate() {
        assert_eq!(mapped.binding.edges()[i].motion(),[pair[0],pair[1]]);
        let (from,to,steps)=mapped.binding.candidate().connection(i).unwrap();
        assert_eq!(from,[pair[0].0[0],pair[0].0[1]]);assert_eq!(to,[pair[1].0[0],pair[1].0[1]]);
        assert!(steps>=1);assert!(mapped.binding.candidate().sample(i,steps).is_some());
    }
}
#[test]
fn c1c2_01_real_associations_change_successors_before_semantic_dominance() {
    // Two earlier fixtures are preserved with their evidence: actualZ=1 was
    // statically outside lower support tolerance; lower0/upper5 at actualZ=.625
    // really produced only the upper PCT cell (required headroom is .5).
    // This separately approved boundary fixture keeps the original model.
    let map=c2_native_fixture(C2NativeFixture::BinShift);let q=f64::from(0.05_f32);
    let xy=[0.025,0.025];let to=[0.125,0.025];
    let actual=([xy[0],xy[1],0.705],[0.,0.,0.,1.]);
    let observations=map.observations().unwrap();
    let records:Vec<_>=observations.ground_records([0,0]).collect();assert_eq!(records.len(),2);
    let endpoints:Vec<_>=observations.ground_records([2,0]).collect();assert_eq!(endpoints.len(),2);
    assert_eq!(records[0].members().collect::<Vec<_>>(),vec![0]);
    assert_eq!(records[1].members().collect::<Vec<_>>(),vec![11]);
    assert_eq!(endpoints[1].members().collect::<Vec<_>>(),vec![11]);
    let low=records[0].height_at(xy).unwrap();let high=records[1].height_at(xy).unwrap();
    let end_low=endpoints[0].height_at(to).unwrap();let end_high=endpoints[1].height_at(to).unwrap();
    assert_eq!(low,7.5*q/8.0);assert_eq!(high,(11.0+0.5/8.0)*q);
    assert_eq!(end_low,low);assert_eq!(end_high,(11.0+7.5/8.0)*q);
    let model=map.body_envelope().unwrap();let required=model.body_to_support_z_m+model.z_bounds_m[1];
    assert!(11.0*q-low>required);
    let support=actual.0[2]-model.body_to_support_z_m;
    assert!((support-low).abs()<model.max_step_up_m+q);assert!((support-high).abs()<model.max_step_down_m+q);
    assert!(actual.0[2]+model.z_bounds_m[0]>12.0*q);
    let tomo=Tomogram::from_surface(&map).unwrap();
    let low_id=tomo.cell_at([xy[0],xy[1],low]).unwrap();let high_id=tomo.cell_at([xy[0],xy[1],high]).unwrap();
    assert_ne!(low_id,high_id);
    assert_eq!(tomo.cells[low_id].source_ground,Some(records[0].key()));
    assert_eq!(tomo.cells[high_id].source_ground,Some(records[1].key()));
    eprintln!("C1C2_THIRD_FIXTURE actual={actual:?} h=[{low:?},{high:?}] end_h=[{end_low:?},{end_high:?}] source_keys=[{:?},{:?}] ids=[{low_id},{high_id}] lower_clearance={:?} required={required:?}",
        records[0].key(),records[1].key(),11.0*q-low);
    let planner=nuwax_nav_pct::PctPlanner::default();
    let lower=c2_route(&planner,&map,actual,Some(low),to);let upper=c2_route(&planner,&map,actual,Some(high),to);
    let a=c2_state(&map,actual,&lower);let b=c2_state(&map,actual,&upper);
    assert_eq!(a.physical,b.physical);assert_ne!(a.navigation.unwrap().semantic_key(),b.navigation.unwrap().semantic_key());
    // Accepting an upper walk through lower's owner proves the underlying
    // Tomogram is the same actual owner, rather than two invented layer IDs.
    lower.reference.freeze_candidate(vec![b.navigation.unwrap().walk_xy(xy,&[],None).unwrap()]).unwrap();
    let query=map.body_query();let ea=project_edge(&query,&a,actual,to,0.0,&[]).unwrap();
    let eb=project_edge(&query,&b,actual,to,0.0,&[]).unwrap();
    check_projected_motion(&query,actual,&ea.physical,&[]).unwrap();
    check_projected_motion(&query,actual,&eb.physical,&[]).unwrap();
    assert_eq!(ea.physical.pose().0[2],actual.0[2]);
    assert!((eb.physical.pose().0[2]-actual.0[2]-7.0*q/8.0).abs()<1e-12);
    assert_ne!(ea.physical.pose(),eb.physical.pose());
    local_search::assert_navigation_dominance(actual,&a.physical,a.navigation.unwrap(),b.navigation.unwrap());
    eprintln!("C1C2_REAL_SUCCESSORS low={low:?} high={high:?} a={:?} b={:?}",ea.physical.pose(),eb.physical.pose());
}
#[test]
fn c1c2_02_unanchored_and_carried_keep_distinct_keys_and_unknown_feasible() {
    let map=coarse_open_floor();let start=([100.,100.,0.55],[0.,0.,0.,1.]);let to=[100.1,100.];
    let planner=nuwax_nav_pct::PctPlanner::default();
    let free=c2_route(&planner,&map,start,None,to);let carry=c2_route(&planner,&map,start,Some(0.15),to);
    let a=c2_state(&map,start,&free);let b=c2_state(&map,start,&carry);
    assert_eq!(a.navigation.unwrap().planned_ground_z(),None);assert_eq!(b.navigation.unwrap().planned_ground_z(),Some(0.15));
    local_search::assert_navigation_dominance(start,&a.physical,a.navigation.unwrap(),b.navigation.unwrap());
    let query=map.body_query();let mut state=a.clone();let mut segments=Some(Vec::new());
    let end=advance_state(&query,&mut state,start,to,0.0,&[],&mut segments).unwrap();
    assert_eq!(end.0[2],start.0[2]);assert_eq!(state.navigation.unwrap().planned_ground_z(),None);
    let (wall,start)=fine_escape_scene(false,[1,0],false,2);
    let route=c1b_route(&wall,start,[start.0[0]+0.01,start.0[1]]);let mut state=c2_state(&wall,start,&route);
    let key=state.navigation.unwrap().semantic_key();let physical=state.physical.clone();let mut segments=Some(Vec::new());
    let result=advance_state(&wall.body_query(),&mut state,start,[start.0[0]+0.25,start.0[1]],0.0,&[],&mut segments);
    assert!(result.is_err(),"actual fixed wall still rejects full physical edge");
    assert_eq!(state.navigation.unwrap().semantic_key(),key);assert_eq!(state.physical,physical);assert!(segments.unwrap().is_empty());
}
#[test]
fn c1c2_03_direct_uses_current_recent_and_failed_branch_is_private() {
    let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
    let start=([-1.,0.2,0.4],[0.,0.,0.,1.]);let reference=[[-1.,0.2],[0.1,0.2]];
    let route=c1b_route(&map,start,reference[1]);let ground=map.body_query().start_ground_state(start).unwrap();
    GROUND_MAPPING_WORK.set(0);GROUND_EDGE_WORK.set([0;3]);
    let result=plan_driven_ground(&map,&tomo,start,&ground,&reference,&motion(),&ScanConfig::default(),None,
        Some((reference[1],0.1)),&[],false,false,None,route.reference.full_window().unwrap()).unwrap();
    assert_eq!(result.binding.family(),CandidateFamily::Direct);c2_assert_binding(&result);
    assert_eq!(GROUND_MAPPING_WORK.get(),0);assert_eq!(GROUND_EDGE_WORK.get()[2],1);
    let recent=map.with_recent_obstacle_points(1,1_000_000_000,0,&[],[(0,[-0.95,0.2,0.4])]).unwrap();
    let mut state=c2_state(&map,start,&route);let original=state.physical.clone();let key=state.navigation.unwrap().semantic_key();
    let mut segments=Some(Vec::new());
    let error=advance_state(&recent.body_query(),&mut state,start,[-0.95,0.2],0.0,&[],&mut segments).unwrap_err();
    assert!(error.body_witness.is_some(),"current recent must reach actual witness path");
    assert_eq!(state.physical,original);assert_eq!(state.navigation.unwrap().semantic_key(),key);assert!(segments.as_ref().unwrap().is_empty());
    advance_state(&map.body_query(),&mut state,start,[-0.95,0.2],0.0,&[],&mut segments).unwrap();
    eprintln!("C1C2_CURRENT_RECENT {error:?}; work={:?}",GROUND_EDGE_WORK.get());
}
#[test]
fn c1c2_04_search_keeps_real_parent_connector_walks() {
    let (map,start)=fine_escape_scene(false,[1,0],false,2);let tomo=Tomogram::from_surface(&map).unwrap();
    let goal=[start.0[0]+0.01,start.0[1]+0.5];let reference=[[start.0[0],start.0[1]],goal];
    let departure=c1b_route(&map,start,[goal[0],start.0[1]]);let ground=map.body_query().start_ground_state(start).unwrap();
    GROUND_MAPPING_WORK.set(0);GROUND_EDGE_WORK.set([0;3]);
    let result=plan_driven_ground(&map,&tomo,start,&ground,&reference,&motion(),&ScanConfig::default(),None,
        Some((goal,0.015)),&[],false,false,None,departure.reference.full_window().unwrap()).unwrap();
    assert_eq!(result.binding.family(),CandidateFamily::Search);c2_assert_binding(&result);
    assert!(result.binding.edges().iter().any(|e|matches!(e.origin(),CandidateEdgeOrigin::Parent {node} if node>0)));
    assert!(result.binding.edges().iter().any(|e|matches!(e.origin(),CandidateEdgeOrigin::Connector {node} if node>0)));
    assert!(!result.binding.covers_original_goal());assert_eq!(GROUND_MAPPING_WORK.get(),0);
    eprintln!("C1C2_SEARCH work={:?} edges={} walkwork={:?}",local_search::last_work(),result.binding.edges().len(),GROUND_EDGE_WORK.get());
}
#[test]
fn c1c2_05_prefix_budget_and_cancel_do_not_consume_actual_cursor() {
    let (map,tomo,start,old_reference,boundary)=progress_prefix_fixture();let reference=old_reference.map(|p|[p[0],p[1]]);
    let route=c1b_route(&map,start,[start.0[0]+0.375,start.0[1]]);let window=route.reference.full_window().unwrap();
    let key=window.from().walk_state().semantic_key();let ground=map.body_query().start_ground_state(start).unwrap();
    let run=||plan_driven_ground(&map,&tomo,start,&ground,&reference,&motion(),&ScanConfig::default(),None,
        Some(([1.,0.2],0.1)),&boundary,false,false,Some(GroundProgressPrefixPolicy {reference:&reference,minimum_distance_m:0.375}),window);
    let result=run().unwrap();assert!(result.trajectory.partial);assert_eq!(local_search::last_work().0,256);
    assert!(!result.binding.covers_original_goal());c2_assert_binding(&result);
    assert_eq!(window.from().walk_state().semantic_key(),key);
    let cancelled=cancellation::after_checkpoints(5,run);
    assert!(matches!(cancelled,Err(GroundCandidateError::Planning(LocalPlanError {kind:LocalPlanFailureKind::Cancelled,..}))));
    assert_eq!(window.from().walk_state().semantic_key(),key);
}
#[test]
fn c1c2_07_spline_and_step_turn_proposals_keep_independent_xy() {
    let map=coarse_open_floor();let start=([100.,100.,0.55],[0.,0.,0.,1.]);
    let route=c1b_route(&map,start,[101.,100.]);let nav=route.reference.full_window().unwrap().from().walk_state();
    let tomo=Tomogram::from_surface(&map).unwrap();let reference=[[100.,100.],[100.5,100.2],[101.,100.]];
    spline::take_ground_proposal_walks();
    let controls=spline::optimize_navigation(&reference,&tomo,0.1,[-0.1,0.1],2,None,Some(nav),&[]).unwrap();
    let proposals=spline::take_ground_proposal_walks();assert!(proposals>0);assert!(controls.iter().flatten().all(|v|v.is_finite()));
    let samples=spline::sample(&controls,0.05).unwrap();let state=c2_state(&map,start,&route);
    let result=project_state(&map,&map.body_query(),&state,start,&samples,&motion(),true,None,None,&[],&std::cell::Cell::new(None)).unwrap();
    assert!(result.poses.iter().all(|p|p.0[2]==start.0[2]));
    assert!(result.navigation.unwrap().iter().all(|s|s.end().planned_ground_z().is_none()));
    let map=c2_native_scene(false);let planner=nuwax_nav_pct::PctPlanner::default();
    let xy=[-0.075,0.025];let low=map.query_ground(xy,0.0).height.unwrap();
    let start=([xy[0],xy[1],low+0.43],[0.,0.,0.,1.]);let reference=[xy,[0.025,0.025],[0.025,0.475]];
    let route=c2_route(&planner,&map,start,Some(low),[0.025,0.025]);let state=c2_state(&map,start,&route);
    let guide=step_turn::guide_state(&map,&map.body_query(),&state,start,&reference,&ScanConfig::default(),false,&[])
        .unwrap().expect("fixed native step plus turn has proposal");
    let samples=spline::resample(&guide,0.025).unwrap();
    let path=project_state(&map,&map.body_query(),&state,start,&samples,&motion(),false,None,None,&[],&std::cell::Cell::new(None)).unwrap();
    assert!(path.navigation.unwrap().len()>1);
    eprintln!("C1C2_PROPOSALS spline_walks={proposals} step_samples={}; complete winner family Rejoin/Passage/optimized Spline NOT_COVERED",path.poses.len());
}
#[test]
fn c1c2_08_original_grid_zero_motion_and_body_no_navigation_work() {
    let map=coarse_open_floor();let tomo=Tomogram::from_surface(&map).unwrap();
    let start=([-1.,0.2,0.4],[0.,0.,0.,1.]);let route=c1b_route(&map,start,[0.1,0.2]);
    let ground=map.body_query().start_ground_state(start).unwrap();let reference=[[-1.,0.2],[0.1,0.2]];
    let result=plan_driven_ground(&map,&tomo,start,&ground,&reference,&motion(),&ScanConfig::default(),None,None,&[],false,false,None,
        route.reference.full_window().unwrap()).unwrap();
    c2_assert_binding(&result);let edge=result.binding.edges().iter().position(|e|e.motion()[0].0!=e.motion()[1].0).unwrap();
    let (_,_,steps)=result.binding.candidate().connection(edge).unwrap();
    let slice=result.binding.slice(edge,0.5/steps as f64,1.0).unwrap();assert_eq!(slice.original_sample(0.0),Some((0,0.5)));
    assert_eq!(slice.original_sample(1.0),Some((steps,0.0)));
    let zero=plan_driven_ground(&map,&tomo,start,&ground,&[reference[0]],&motion(),&ScanConfig::default(),None,
        Some((reference[0],0.0)),&[],false,false,None,route.reference.full_window().unwrap()).unwrap();
    assert!(zero.trajectory.poses.iter().all(|p|*p==start));assert_eq!(zero.binding.edges().len(),1);
    assert!(!zero.binding.covers_original_goal());
    GROUND_EDGE_WORK.set([0;3]);spline::take_ground_proposal_walks();GROUND_MAPPING_WORK.set(0);
    plan_with_progress_from_ground(&map,&tomo,start,&ground,&[[-1.,0.2,0.0],[0.1,0.2,0.0]],&motion(),&ScanConfig::default(),
        None,None,&[],false,false,None).unwrap();
    assert_eq!(GROUND_EDGE_WORK.get(),[0;3]);assert_eq!(spline::take_ground_proposal_walks(),0);assert_eq!(GROUND_MAPPING_WORK.get(),0);
}
