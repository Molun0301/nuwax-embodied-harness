
#[test]
fn progress_prefix_nominal_window_does_not_use_the_current_pose_connector() {
    let (_,request,_)=fixture();
    let pose=|x:f64,y:f64|{let mut p=request.pose.clone();p.pose.position=Point3::new(x,y,0.5);p};
    let source=ReferenceSource {route:Arc::new(Path3D{poses:vec![pose(0.0,0.0),pose(1.0,0.0),pose(1.0,2.0)]}),
        cumulative:Arc::new(vec![0.0,1.0,3.0]),goal:Some(pose(1.0,2.0)),options:GoalOptions::default(),offset:0.4,progress:0.5};
    let current=pose(0.5,0.4);let (reference,_,_,_)=source.materialize(&current,2.0).unwrap();
    assert_eq!(&reference[0][..2],&[0.5,0.4]);
    assert!((reference[0][2]-0.1).abs()<1e-12);
    let nominal=source.nominal_reference(&reference);
    assert!((nominal[0][0]-0.5).abs()<1e-12 && nominal[0][1]==0.0);
    assert_eq!(&nominal[1..],&reference[1..],"只有原窗口起点与当前位置接头分开");
}
#[test]
fn progress_prefix_changed_coverage_context_cannot_reuse_a_cached_negative() {
    for change_end in [false,true] {
        let (mut kernel,mut request,cancel)=fixture();seed_exact_failures(&mut kernel,&request);
        if change_end {request.end+=0.1;}else{request.progress+=0.1;}
        let result=kernel.compute_inner(&request,&cancel);
        assert!(result.is_ok(),"原覆盖上下文的缓存拒绝不能覆盖新的窗口进度：{:?}",result.err());
    }
}

#[test]
fn progress_prefix_minimum_uses_original_goal_options_instead_of_startup_defaults() {
    let (kernel,mut request,_)=fixture();
    let half_source=f64::from(request.source.source_voxel_size_m())*0.5;
    assert!((kernel.progress_minimum_distance(&request)-(0.35+half_source)).abs()<1e-12);
    let mut goal=request.pose.clone();goal.pose.position.x=3.0;
    request.reference_source=Some(ReferenceSource {route:Arc::new(Path3D{poses:vec![request.pose.clone(),goal.clone()]}),
        cumulative:Arc::new(vec![0.0,2.5]),goal:Some(goal),options:GoalOptions{position_tolerance_m:0.8,..GoalOptions::default()},
        offset:0.4,progress:0.0});
    assert!((kernel.progress_minimum_distance(&request)-(0.8+half_source)).abs()<1e-12);
}
