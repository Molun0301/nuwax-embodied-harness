
fn progress_prefix_fixture() -> (SurfaceSnapshot, Tomogram, BodyPose, [[f64;3];2], [[f64;2];4]) {
    let map=scene_with_extra_cells(Some(0),0.0,&[]);
    let tomo=Tomogram::from_surface(&map).unwrap();
    let start=([-1.0,0.2,0.4],[0.0,0.0,0.0,1.0]);
    (map,tomo,start,[[-1.0,0.2,0.0],[1.0,0.2,0.0]],
        [[-1.5,-0.5],[1.5,-0.5],[1.5,0.9],[-1.5,0.9]])
}
#[test]
fn progress_prefix_dead_end_keeps_original_goal_unreached_and_every_edge_admitted() {
    let (map,tomo,start,reference,boundary)=progress_prefix_fixture();
    let goal=[1.0,0.2,0.4];let caps=motion();
    let run=|policy|local_search::with_heuristic_mode(local_search::HeuristicMode::ScaledSum(4.0),||
        local_search::with_expansion_limit(12,||local_search::plan_progress(&map,&tomo,start,&reference,
            &caps,None,Some((goal,0.1)),&boundary,3.0,0.9,false,DEFAULT_SEARCH_EXPANSION_LIMIT,policy)));
    let legacy=run(None).unwrap_err();assert_eq!(legacy.kind,LocalPlanFailureKind::BudgetExhausted);
    let path=run(Some(ProgressPrefixPolicy{reference:&reference,minimum_distance_m:0.375})).unwrap();
    assert!(path.partial,"死胡同前的安全有限段不能伪装完整连接");
    assert!(distance3(path.poses.last().unwrap().0,goal)>0.1);
    assert!(path.poses.iter().all(|p|p.0[0]<0.0),"不能穿过覆盖整个边界的真实墙");
    assert!(path.poses.last().unwrap().0[0]-start.0[0]>=map.body_envelope().unwrap().radius());
    for p in path.poses.windows(2) {map.validate_body_motion(p[0],p[1],&boundary).unwrap();}
    assert_eq!(local_search::last_work().0,12,"有限段继续使用同一展开预算");
}
#[test]
fn progress_prefix_requires_original_forward_progress_and_control_separation() {
    let (map,tomo,start,reference,boundary)=progress_prefix_fixture();let reverse=[reference[1],reference[0]];
    for (policy_reference,minimum) in [(&reference[..],2.0),(&reverse[..],0.375)] {
        let error=local_search::with_heuristic_mode(local_search::HeuristicMode::ScaledSum(4.0),||
            local_search::with_expansion_limit(12,||local_search::plan_progress(&map,&tomo,start,&reference,
                &motion(),None,Some(([1.0,0.2,0.4],0.1)),&boundary,3.0,0.9,false,DEFAULT_SEARCH_EXPANSION_LIMIT,
                Some(ProgressPrefixPolicy{reference:policy_reference,minimum_distance_m:minimum})))).unwrap_err();
        assert_eq!(error.kind,LocalPlanFailureKind::BudgetExhausted);
        assert!(error.blocked.is_none());
    }
}
#[test]
fn progress_prefix_cancellation_and_invalid_policy_never_become_candidates() {
    let (map,tomo,start,reference,boundary)=progress_prefix_fixture();let cfg=ScanConfig::default();
    let policy=ProgressPrefixPolicy{reference:&reference,minimum_distance_m:0.375};
    let cancelled=cancellation::after_checkpoints(5,||plan_with_progress(&map,&tomo,start,&reference,
        &motion(),&cfg,None,Some(([1.0,0.2,0.4],0.1)),&boundary,false,false,Some(policy))).unwrap_err();
    assert_eq!(cancelled.kind,LocalPlanFailureKind::Cancelled);
    for minimum in [f64::NAN,-1.0] {
        let invalid=plan_with_progress(&map,&tomo,start,&reference,&motion(),&cfg,None,None,&[],false,false,
            Some(ProgressPrefixPolicy{reference:&reference,minimum_distance_m:minimum})).unwrap_err();
        assert_eq!(invalid.kind,LocalPlanFailureKind::Unclassified);
    }
}
#[test]
fn progress_prefix_policy_preserves_a_complete_connection() {
    let map=scene(None);let tomo=Tomogram::from_surface(&map).unwrap();
    let reference=[[-1.0,0.2,0.0],[0.1,0.2,0.0]];let start=(reference[0].map(|v|v),[0.0,0.0,0.0,1.0]);
    let start=([start.0[0],start.0[1],0.4],start.1);
    let result=plan_with_progress(&map,&tomo,start,&reference,&motion(),&ScanConfig::default(),None,
        Some(([0.1,0.2,0.4],0.1)),&[],false,false,
        Some(ProgressPrefixPolicy{reference:&reference,minimum_distance_m:0.375})).unwrap();
    assert!(!result.partial);assert!(distance3(result.poses.last().unwrap().0,[0.1,0.2,0.4])<0.1);
}
