// 三个执行单元的回归；只在独立测试线程注入规划耗时，不启动平台。
use super::*;
use nuwax_nav_types_messages::{NavFrameId, Point3, Pose3, Quaternion, Timestamp};
use std::time::{Duration, Instant};
fn pose(x: f64, y: f64, z: f64) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(1, 0),
        pose: Pose3 {
            position: Point3::new(x, y, z),
            orientation: Quaternion::IDENTITY,
        },
    }
}
fn ready_follower() -> SurfaceFollower {
    let (mut config, source) = crate::native_test_helpers::fixture();
    config.scan.planning_horizon = 5.;
    let mut follower = crate::native_test_helpers::author_static_map_follower(config);
    let start = pose(0.5, 1., 0.5);
    let goal = pose(5., 1., 0.5);
    follower
        .configure_goal(
            &goal,
            GoalOptions {
                position_tolerance_m: 0.03,
                ..GoalOptions::default()
            },
        )
        .unwrap();
    follower
        .update_execution_map(ExecutionMap::Surface(source))
        .unwrap();
    follower.update_odometry(start.clone());
    follower.observe_measured_motion(Some(crate::motion_feedback::tests::measured(
        [0.3, 0., 0.],
        [0.; 3],
    )));
    follower
        .replace_execution_path(
            ExecutionPath::Surface(Path3D {
                poses: vec![start, goal.clone()],
            }),
            None,
            goal,
        )
        .unwrap();
    let until = Instant::now() + Duration::from_secs(3);
    while follower.active.is_none() {
        assert!(
            Instant::now() < until,
            "真实初始规划没有交付：{:?}",
            follower.wait_reason
        );
        follower.step();
        std::thread::sleep(Duration::from_millis(1));
    }
    follower
}
fn next_pose(f: &mut SurfaceFollower, position: [f64; 3]) {
    let mut current = f.odometry.as_ref().unwrap().clone();
    current.stamp.nanos += 50_000_000;
    current.pose.position = Point3::new(position[0], position[1], position[2]);
    f.update_odometry(current);
}
fn delayed_planner(
    f: &mut SurfaceFollower,
    delay: Duration,
) -> Arc<std::sync::Mutex<Option<Trajectory>>> {
    let result = Arc::new(std::sync::Mutex::new(None));
    let saved = result.clone();
    let mut planner = nuwax_nav_scan_native::ScanPlanner::default();
    f.plan_worker = PlanWorker::with_computer(move |job| {
        std::thread::sleep(delay);
        let curve = planner.plan(
            &job.config,
            &job.map.snapshot,
            &job.reference,
            &job.state,
            job.active.as_ref(),
            job.phase,
        )?;
        *saved.lock().unwrap() = Some(curve.clone());
        Ok(curve)
    })
    .unwrap();
    result
}
#[test]
fn two_second_plan_keeps_tracking_and_next_tick_installs_projected_original_curve() {
    let mut f = ready_follower();
    let result = delayed_planner(&mut f, Duration::from_secs(2));
    next_pose(&mut f, [0.9, 1., 0.5]);
    f.applied = Twist2 {
        linear: Point2::new(0.3, 0.),
        angular_z: 0.,
    };
    f.needs_plan = true;
    let started = Instant::now();
    let mut count = 0;
    let mut previous_position = SurfaceFollower::position(f.odometry.as_ref().unwrap());
    while !f.plan_worker.ready() {
        let PlannerStep::Command(command) = f.step() else {
            panic!("旧轨迹跟踪被慢规划打断")
        };
        assert!(
            command.linear.x > 0.,
            "慢规划期间每拍仍须生成非零速度：{:?}",
            f.wait_reason
        );
        f.observe_applied_command(command);
        previous_position[0] += command.linear.x * 0.05;
        next_pose(&mut f, previous_position);
        count += 1;
        std::thread::sleep(Duration::from_millis(50));
        assert!(started.elapsed() < Duration::from_secs(4));
    }
    assert!(started.elapsed() >= Duration::from_secs(2));
    assert!(count >= 35, "两秒内实际跟踪拍数={count}");
    let curve = result.lock().unwrap().clone().unwrap();
    let before = f.plans;
    let original = curve.samples(0., 0.005);
    let expected = original
        .iter()
        .min_by(|a, b| {
            let d = |p: &nuwax_nav_scan_native::TrajectorySample| {
                (0..3)
                    .map(|i| (p.position[i] - previous_position[i]).powi(2))
                    .sum::<f64>()
            };
            d(a).total_cmp(&d(b))
        })
        .unwrap()
        .time;
    assert!(matches!(f.step(), PlannerStep::Command(_)));
    assert_eq!(f.plans, before + 1, "新轨迹必须下一拍切换");
    assert!(
        f.phase > 0.1 && (f.phase - expected).abs() < 0.01,
        "相位{}，当前位置投影{}",
        f.phase,
        expected
    );
    let installed = f.active.as_ref().unwrap().samples(0., 0.005);
    assert_eq!(installed.len(), original.len());
    for (a, b) in installed.iter().zip(original) {
        assert_eq!(a.position, b.position);
        assert_eq!(a.velocity, b.velocity);
    }
    eprintln!(
        "N2A2 慢规划耗时={:.3}s，连续非零跟踪{}拍，切换相位={:.6}s，独立投影={expected:.6}s",
        started.elapsed().as_secs_f64(),
        count,
        f.phase
    );
}
#[test]
fn deviation_over_constant_threshold_requests_local_plan() {
    let mut f = ready_follower();
    delayed_planner(&mut f, Duration::from_millis(200));
    next_pose(&mut f, [0.5, 1.21, 0.5]);
    let step = f.step();
    assert_eq!(step, PlannerStep::Command(Twist2::ZERO));
    assert!(f.active.is_none() && f.planning);
    assert!(f.wait_reason.as_ref().unwrap().contains("偏离"));
}
#[test]
fn remaining_two_point_five_seconds_requests_once_and_coalesces() {
    let mut f = ready_follower();
    delayed_planner(&mut f, Duration::from_millis(500));
    let samples = f.active.as_ref().unwrap().samples(0., 0.01);
    let duration = samples.last().unwrap().time;
    let at = samples
        .iter()
        .min_by(|a, b| {
            (a.time - (duration - 2.49))
                .abs()
                .total_cmp(&(b.time - (duration - 2.49)).abs())
        })
        .unwrap();
    let position = at.position;
    next_pose(&mut f, position);
    f.needs_plan = false;
    assert!(matches!(f.step(), PlannerStep::Command(_)));
    assert!(f.early_requested && f.planning);
    let calls = f.local_planning_calls();
    for _ in 0..3 {
        f.step();
        assert_eq!(f.local_planning_calls(), calls);
    }
}
#[test]
fn final_no_candidate_zeroes_and_requests_plan_from_raw_motion() {
    let mut f = ready_follower();
    delayed_planner(&mut f, Duration::from_millis(200));
    let map = f.map.as_ref().unwrap();
    let config = f.native_config(map);
    let mut owner = nuwax_nav_scan_native::MapOwner::new(&config).unwrap();
    owner.seed_occupied(&[[0.525, 1.225, 0.5]], [0.5, 1., 0.5]);
    owner.update_surface(&[], &[], &[], &[]).unwrap();
    let snapshot = owner.snapshot();
    f.epoch += 1;
    f.map_worker.retire(f.epoch, false);
    f.prepared = Some(PreparedMap {
        epoch: f.epoch,
        source: map.clone(),
        snapshot,
        config,
        changed: false,
        checked_curve: None,
    });
    // 机身外的障碍使实测横速的完整制动路径碰撞，所有候选均无效。
    f.observe_measured_motion(Some(crate::motion_feedback::tests::measured(
        [0., 0.4, 0.],
        [0.; 3],
    )));
    f.needs_plan = false;
    let step = f.step();
    assert_eq!(step, PlannerStep::Command(Twist2::ZERO));
    assert!(f.planning && f.active.is_none());
    assert!(f.wait_reason.as_ref().unwrap().contains("合法候选"));
}
#[test]
fn stop_retires_slow_plan_without_waiting_or_replaying_result() {
    let mut f = ready_follower();
    delayed_planner(&mut f, Duration::from_millis(250));
    f.needs_plan = true;
    f.step();
    let started = Instant::now();
    f.stop();
    assert!(started.elapsed() < Duration::from_millis(50));
    std::thread::sleep(Duration::from_millis(300));
    assert!(matches!(f.step(), PlannerStep::Pending(_)));
    assert!(f.active.is_none());
}

#[test]
fn distant_new_curve_keeps_valid_old_curve_and_requests_again() {
    let mut f = ready_follower();
    let old = f.active.clone().unwrap();
    let mut planner = nuwax_nav_scan_native::ScanPlanner::default();
    f.plan_worker = PlanWorker::with_computer(move |job| {
        let mut state = job.state;
        state.position[1] += 1.;
        let reference: Vec<_> = job
            .reference
            .iter()
            .map(|p| [p[0], p[1] + 1., p[2]])
            .collect();
        planner.plan(&job.config, &job.map.snapshot, &reference, &state, None, 0.)
    })
    .unwrap();
    f.needs_plan = true;
    f.step();
    let until = Instant::now() + Duration::from_secs(2);
    while !f.plan_worker.ready() {
        assert!(Instant::now() < until);
        std::thread::sleep(Duration::from_millis(1));
    }
    let installed = f.plans;
    let PlannerStep::Command(command) = f.step() else {
        panic!("旧轨迹应继续跟踪")
    };
    assert!(command.linear.x > 0.);
    assert_eq!(f.plans, installed);
    assert!(f.active.as_ref().unwrap().same(&old));
    assert!(f.pending_curve.is_none() && f.planning);
}
