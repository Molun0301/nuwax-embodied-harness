// 本组固定全局参考的实际进度反例，不启动地图计算或运动设备。
fn runtime_folded_route() -> Vec<PoseStamped> {
    [[0.5, 1.0], [1.5, 1.0], [1.5, 1.1], [0.5, 1.1], [0.5, 1.2]]
        .into_iter()
        .map(|[x, y]| {
            let mut pose = comparison_pose(x, 0);
            pose.pose.position.y = y;
            pose
        })
        .collect()
}

fn runtime_reference_only_follower(actual: PoseStamped) -> RouteFollower {
    let poses = runtime_folded_route();
    runtime_reference_route_follower(poses, actual)
}

fn runtime_reference_route_follower(poses: Vec<PoseStamped>, actual: PoseStamped) -> RouteFollower {
    let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
    let goal = poses.last().unwrap().clone();
    follower.update_odometry(actual);
    follower.configure_goal(&goal, GoalOptions::default()).unwrap();
    follower.replace_execution_path(ExecutionPath::Surface(Path3D { poses }), None, goal).unwrap();
    follower
}

#[test]
fn runtime_route_initial_progress_does_not_jump_to_nearby_return_leg() {
    let mut actual = comparison_pose(0.5, 0);
    actual.pose.position.y = 1.09;
    let follower = runtime_reference_only_follower(actual);
    assert!(follower.progress_m() <= follower.config.max_path_deviation_m,
        "首段仅偏离9厘米，不能因回程更近而跳过两米路线；实际进度={}", follower.progress_m());
}

#[test]
fn runtime_route_nine_centimeter_motion_does_not_consume_two_meter_fold() {
    let mut follower = runtime_reference_only_follower(comparison_pose(0.5, 0));
    assert_eq!(follower.progress_m(), 0.0);
    let mut actual = comparison_pose(0.5, 1);
    actual.pose.position.y = 1.09;
    follower.update_odometry(actual.clone());
    follower.advance_progress();
    let maximum = 0.09 + follower.config.controller.goal_tolerance;
    assert!(follower.progress_m() <= maximum,
        "实际只移动9厘米，不能消费两米回折；实际进度={}", follower.progress_m());
    let retained = follower.progress_m();
    for _ in 0..10 {
        follower.update_odometry(actual.clone());
        follower.advance_progress();
        assert_eq!(follower.progress_m(), retained, "相同位姿不能通过重复轮询累计进度");
    }
}

#[test]
fn runtime_route_projection_does_not_pass_end_of_search_window() {
    let mut follower = RouteFollower::new(comparison_config(Some(true))).unwrap();
    follower.scan.horizon_m = 1.0;
    follower.route = Some(Arc::new(Path3D { poses: vec![comparison_pose(0.5, 0), comparison_pose(10.5, 0)] }));
    follower.cumulative = Arc::new(vec![0.0, 10.0]);
    follower.pose = Some(comparison_pose(8.5, 0));
    follower.advance_progress();
    assert!(follower.progress_m() <= 1.0,
        "长段仅起点进入搜索窗口时，段内投影也必须裁至窗口；实际进度={}", follower.progress_m());
}

#[test]
fn runtime_route_straight_motion_keeps_actual_progress() {
    let start = comparison_pose(0.5, 0);
    let mut follower = runtime_reference_route_follower(
        vec![start.clone(), comparison_pose(3.0, 0)], start);
    for tick in 1..=20 {
        let traveled = f64::from(tick) * 0.1;
        let actual = comparison_pose(0.5 + traveled, tick);
        follower.update_odometry(actual.clone());
        follower.advance_progress();
        assert!((follower.progress_m() - traveled).abs() < 1e-9,
            "正常直路必须消费真实位移：tick={tick}, progress={}", follower.progress_m());
        follower.update_odometry(actual);
        follower.advance_progress();
        assert!((follower.progress_m() - traveled).abs() < 1e-9,
            "重复采样不能新增直路进度");
    }
}

#[test]
fn runtime_route_curve_motion_preserves_continuous_segment_progress() {
    let poses: Vec<_> = (0..=20).map(|tick| {
        let angle = f64::from(tick) * std::f64::consts::FRAC_PI_2 / 20.0;
        let mut pose = comparison_pose(0.5 + angle.cos(), tick);
        pose.pose.position.y = 1.0 + angle.sin();
        pose.pose.orientation = Quaternion::from_yaw(angle + std::f64::consts::FRAC_PI_2);
        pose
    }).collect();
    let mut follower = runtime_reference_route_follower(poses.clone(), poses[0].clone());
    let mut traveled = 0.0;
    for pair in poses.windows(2) {
        traveled += pair[0].pose.position.distance(pair[1].pose.position);
        follower.update_odometry(pair[1].clone());
        follower.advance_progress();
        assert!((follower.progress_m() - traveled).abs() < 1e-9,
            "连续曲线不能因窗口缩小而遗漏已到段端：progress={}, expected={traveled}", follower.progress_m());
    }
}

#[test]
fn runtime_route_replacement_resets_reference_progress_without_losing_new_motion() {
    let start = comparison_pose(0.5, 0);
    let mut follower = runtime_reference_route_follower(
        vec![start.clone(), comparison_pose(3.0, 0)], start);
    follower.update_odometry(comparison_pose(1.5, 1));
    follower.advance_progress();
    assert!((follower.progress_m() - 1.0).abs() < 1e-9);

    let start = comparison_pose(1.5, 1);
    let mut goal = start.clone();
    goal.pose.position.y = 2.0;
    follower.replace_execution_path(ExecutionPath::Surface(Path3D {
        poses: vec![start.clone(), goal.clone()],
    }), None, goal).unwrap();
    assert_eq!(follower.progress_m(), 0.0, "新路线不能继承旧参考弧长");
    follower.advance_progress();
    assert_eq!(follower.progress_m(), 0.0, "同一实际位姿不能消费新路线");

    let mut actual = start;
    actual.pose.position.y += 0.1;
    actual.stamp = comparison_pose(1.5, 2).stamp;
    follower.update_odometry(actual);
    follower.advance_progress();
    assert!((follower.progress_m() - 0.1).abs() < 1e-9,
        "换线后仍须按真实移动推进：{}", follower.progress_m());
}

#[test]
fn runtime_route_millimeter_pose_jitter_does_not_consume_fold() {
    for jitter in [0.001, 0.009] {
        let mut follower = runtime_reference_only_follower(comparison_pose(0.5, 0));
        for tick in 1..=2000 {
            let mut actual = comparison_pose(0.5 + if tick % 2 == 0 { jitter } else { -jitter }, tick);
            actual.pose.position.y = 1.09;
            follower.update_odometry(actual);
            follower.advance_progress();
            assert!(follower.progress_m() <= jitter + 1e-9,
                "原折返输入仅在起点附近抖动，不能累计消费远处回程：jitter={jitter}, tick={tick}, progress={}",
                follower.progress_m());
        }
    }
}

#[test]
fn runtime_route_local_admission_revocation_does_not_rearm_initial_window() {
    let mut actual = comparison_pose(0.5, 0);
    actual.pose.position.y = 1.09;
    let mut follower = runtime_reference_only_follower(actual);
    // 已消费的前缀与当前横向绕行位置都属于同一条路线；许可撤销不产生新位移。
    follower.progress = 0.8;
    for _ in 0..3 {
        follower.revoke_local_admission();
        follower.advance_progress();
        assert_eq!(follower.progress_m(), 0.8,
            "仅撤销局部许可不能再次发放新路线首段接续范围");
    }
}
