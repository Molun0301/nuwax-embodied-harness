// 真实线程拥有者、融合表面与原生跟随器一起运行；按墙钟控制拍更新输入和积分实测响应。
#[test]
fn current_surface_updates_every_one_to_three_seconds_allow_motion_and_arrival() {
    let (mut config, source) = crate::native_test_helpers::fixture();
    config.controller.speed = 0.5;
    config.controller.nerf_speed = 1.;
    config.scan.planning_horizon = 5.;
    let mut worker = PlannerWorker::new(SurfaceFollower::new(config).unwrap()).unwrap();
    let mut pose = route_pose(0.5, 0);
    let goal = route_pose(4.5, 0);
    let options = GoalOptions {
        position_tolerance_m: 0.1,
        ..GoalOptions::default()
    };
    worker.configure_goal(&goal, options).unwrap();
    worker
        .update_execution_map(ExecutionMap::Surface(source.clone()))
        .unwrap();
    worker.update_odometry(pose.clone());
    worker
        .replace_execution_path(
            ExecutionPath::Surface(Path3D {
                poses: vec![pose.clone(), goal.clone()],
                ..Path3D::default()
            }),
            None,
            goal.clone(),
        )
        .unwrap();
    let mut speed = 0.;
    let mut angular = 0.;
    let mut yaw = 0.;
    let mut updates = 0;
    let mut next_update = 20;
    let mut moving = 0;
    for tick in 0..300 {
        let began = std::time::Instant::now();
        pose.stamp = route_pose(0., tick).stamp;
        if tick == next_update {
            updates += 1;
            // 远处实体每次都有真实几何变化；不是仅换一个相同地图序号。
            let map = Arc::new(
                source
                    .with_recent_obstacle_points(
                        1,
                        updates,
                        0,
                        &[],
                        [(0, [5.5, 2.5 + updates as f64 * 0.01, 0.5])],
                    )
                    .unwrap(),
            );
            worker
                .update_execution_map(ExecutionMap::Surface(map))
                .unwrap();
            next_update += 20 * (updates % 3 + 1) as i64;
        }
        let mut frame = (*crate::registered_frame::test_frame(tick + 1)).clone();
        frame.identity.timestamp_ns = pose.stamp.nanos;
        frame.body_position_world = [pose.pose.position.x, pose.pose.position.y, 0.5];
        frame.sensor_origin_world = [pose.pose.position.x, pose.pose.position.y, 0.8];
        frame.points_world = vec![[pose.pose.position.x, pose.pose.position.y, 0.05]].into();
        frame.body_orientation_xyzw = [0., 0., (yaw / 2.0_f64).sin(), (yaw / 2.0_f64).cos()];
        frame.ground_point_indices = vec![0].into();
        frame.paired_odometry_timestamp_ns = pose.stamp.nanos;
        worker.observe_registered_frame(Some(Arc::new(frame)));
        worker.update_odometry(pose.clone());
        let mut motion =
            crate::motion_feedback::tests::measured([speed, 0., 0.], [0., 0., angular]);
        motion.orientation_wxyz = Some([(yaw / 2.).cos(), 0., 0., (yaw / 2.).sin()]);
        motion.source_timestamp_ns = pose.stamp.nanos as u64;
        worker.observe_measured_motion(Some(motion));
        let deadline = began + Duration::from_millis(50);
        let outcome = loop {
            let outcome = worker.step();
            if !matches!(outcome, PlannerStep::Pending(_)) || std::time::Instant::now() >= deadline
            {
                break outcome;
            }
            std::thread::sleep(Duration::from_millis(1));
        };
        let command = match outcome {
            PlannerStep::Succeeded => {
                assert!(
                    updates >= 3 && moving > 20,
                    "必须覆盖1、2、3秒连续换图并真实前进"
                );
                assert!(
                    (pose.pose.position.x - goal.pose.position.x)
                        .hypot(pose.pose.position.y - goal.pose.position.y)
                        < 0.1
                );
                worker.stop();
                return;
            }
            PlannerStep::Command(command) => command,
            PlannerStep::Pending(_) => Twist2::ZERO,
            other => panic!("开阔路线不能因当前表面更新而失败：{other:?}"),
        };
        assert!(command.linear.y.abs() < 1e-8 && command.angular_z.abs() <= 0.8);
        if command.linear.x > 0. {
            moving += 1;
        }
        let next = speed + (command.linear.x - speed).clamp(-0.5 * 0.05, 0.5 * 0.05);
        let next_w = angular + (command.angular_z - angular).clamp(-0.05, 0.05);
        let mean_w = (angular + next_w) * 0.5;
        let mean_v = (speed + next) * 0.5;
        let (dx, dy) = if mean_w.abs() > 1e-9 {
            (
                mean_v * (mean_w * 0.05).sin() / mean_w,
                mean_v * (1. - (mean_w * 0.05).cos()) / mean_w,
            )
        } else {
            (mean_v * 0.05, 0.)
        };
        pose.pose.position.x += yaw.cos() * dx - yaw.sin() * dy;
        pose.pose.position.y += yaw.sin() * dx + yaw.cos() * dy;
        yaw += mean_w * 0.05;
        angular = next_w;
        speed = next;
        pose.pose.orientation = Quaternion::from_yaw(yaw);
        worker.observe_applied_command(command);
        if let Some(remaining) = deadline.checked_duration_since(std::time::Instant::now()) {
            std::thread::sleep(remaining);
        }
    }
    panic!(
        "原控制拍与交付期限内未到达：x={}，换图{}次，非零{}拍",
        pose.pose.position.x, updates, moving
    );
}
