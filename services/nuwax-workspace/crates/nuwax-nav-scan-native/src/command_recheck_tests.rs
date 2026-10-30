// 已选指令复核共用原生响应与归零制动扫掠，不推进轨迹状态机。
#[test]
fn command_recheck_keeps_command_without_advancing_local_planning() {
    let route=[[0.,0.,0.35],[2.,0.,0.35]];
    let mut req=request(&route,&[]);
    let mut planner=fixture_planner();
    for tick in 0..4 {
        req.odometry_timestamp_ns=1_000_000_000+tick*100_000_000;
        planner.step(&req).unwrap();
    }
    let plans=planner.plan_count();
    assert!(plans>0,"夹具先建立真实原生轨迹");
    assert!(planner.command_recheck_ready());
    for _ in 0..3 {
        req.odometry_timestamp_ns+=100_000_000;
        let checked=planner.recheck_command(&req,[0.1,0.,0.],[0.;3],0.).unwrap();
        assert_eq!(checked.command_forward,0.1);
        assert_eq!(checked.command_lateral,0.);
        assert_eq!(checked.command_yaw_rate,0.);
        assert_eq!(checked.command_duration,req.config.sample_dt);
        assert!(checked.samples.is_empty());
        assert!(!checked.replanned);
        assert_eq!(planner.plan_count(),plans,"命令复核不能推进重规划");
    }
    planner.reset_execution();
    assert!(!planner.command_recheck_ready(),"执行退休不能被已选指令复核越过");
}

#[test]
fn command_recheck_uses_raw_braking_and_rejects_body_penetration() {
    let route=[[0.,0.,0.35],[2.,0.,0.35]];
    let wall=[[0.65,0.,0.35]];
    let req=request(&route,&wall);
    let mut planner=fixture_planner();
    let checked=planner.recheck_command(&req,[0.1,0.,0.],[0.;3],0.).unwrap();
    assert_eq!(checked.command_forward,0.1);
    let error=planner.recheck_command(&req,[0.1,0.,0.],[0.8,0.,0.],0.).unwrap_err();
    assert!(error.contains("Current motion and braking sweep"),"小命令不能代替原始高速的制动证明: {error}");
    // 当前实体位于机身前沿内0.04米，复核仍必须拒绝。
    let penetrating=[[0.31,0.,0.35]];
    let req=request(&route,&penetrating);
    assert!(planner.recheck_command(&req,[0.1,0.,0.],[0.;3],0.).is_err());
}
