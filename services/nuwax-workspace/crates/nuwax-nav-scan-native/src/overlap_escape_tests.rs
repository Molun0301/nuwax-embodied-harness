// 独立矩形/体素分离轴度量，不调用原生脱困的穿透计算。
fn escape_cell_depth(c: &Config, obstacle: [f64; 3], p: [f64; 3], yaw: f64) -> f64 {
    let (s, co) = yaw.sin_cos();
    let r = c.resolution / 2.;
    let x = (obstacle[0] / c.resolution).floor() * c.resolution + r;
    let y = (obstacle[1] / c.resolution).floor() * c.resolution + r;
    let dx = x - p[0] - co * c.body_center_x + s * c.body_center_y;
    let dy = y - p[1] - s * c.body_center_x - co * c.body_center_y;
    [
        c.body_length / 2. * co.abs() + c.body_width / 2. * s.abs() + r - dx.abs(),
        c.body_length / 2. * s.abs() + c.body_width / 2. * co.abs() + r - dy.abs(),
        c.body_length / 2. + r * (co.abs() + s.abs()) - (co * dx + s * dy).abs(),
        c.body_width / 2. + r * (co.abs() + s.abs()) - (-s * dx + co * dy).abs(),
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min)
}

fn check_escape_sweep(c: &Config, obstacles: &[[f64; 3]], samples: &[TrajectorySample]) {
    assert!(samples.len() >= 2);
    let stopped = samples.last().unwrap();
    assert_eq!(stopped.velocity, [0.; 3]);
    assert_eq!(stopped.yaw_rate, 0.);
    for pair in samples.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let dt = b.time - a.time;
        let local = |sample: &TrajectorySample| {
            let (s, co) = sample.yaw.sin_cos();
            [
                co * sample.velocity[0] + s * sample.velocity[1],
                -s * sample.velocity[0] + co * sample.velocity[1],
            ]
        };
        let (va, vb) = (local(a), local(b));
        assert!((vb[0] - va[0]).hypot(vb[1] - va[1]) <= c.max_acceleration * dt + 1e-10);
        assert!((b.yaw_rate - a.yaw_rate).abs() <= c.max_yaw_acceleration * dt + 1e-10);
        let (u, v, w) = (
            (va[0] + vb[0]) / 2.,
            (va[1] + vb[1]) / 2.,
            (a.yaw_rate + b.yaw_rate) / 2.,
        );
        let mut previous: Vec<_> = obstacles
            .iter()
            .map(|&cell| escape_cell_depth(c, cell, a.position, a.yaw).max(0.))
            .collect();
        // 每个执行/制动区间取100个点，独立积分并核对真实最小穿透非增。
        for part in 1..=100 {
            let t = dt * part as f64 / 100.;
            let angle = w * t;
            let (dx, dy) = if w.abs() < 1e-9 {
                (u * t, v * t)
            } else {
                (
                    (u * angle.sin() + v * (angle.cos() - 1.)) / w,
                    (u * (1. - angle.cos()) + v * angle.sin()) / w,
                )
            };
            let (s, co) = a.yaw.sin_cos();
            let p = [
                a.position[0] + co * dx - s * dy,
                a.position[1] + s * dx + co * dy,
                a.position[2],
            ];
            for (index, &cell) in obstacles.iter().enumerate() {
                let now = escape_cell_depth(c, cell, p, a.yaw + angle).max(0.);
                assert!(
                    now <= previous[index] + 1e-10,
                    "执行或制动中穿透增大：原{}，新{now}",
                    previous[index]
                );
                previous[index] = now;
            }
        }
    }
}

fn run_shallow_escape(mut req: PlanRequest<'_>) -> ([f64; 3], usize) {
    let mut planner = registered_fixture(&req);
    let mut first = [0.; 3];
    for tick in 0..400 {
        req.odometry_timestamp_ns = tick * 50_000_000;
        let out = planner
            .step(&req)
            .expect("可行的浅重叠必须算出有界脱困动作");
        if tick == 0 {
            first = [
                out.command_forward,
                out.command_lateral,
                out.command_yaw_rate,
            ];
        }
        assert!(out.command_forward >= -req.config.max_reverse_velocity.max(0.04));
        assert!(out.command_forward <= req.config.max_velocity);
        assert!(out.command_lateral.abs() <= req.config.max_lateral_velocity);
        assert!(out.command_yaw_rate.abs() <= req.config.max_yaw_rate);
        check_escape_sweep(&req.config, req.obstacles, &out.samples);
        let actual = &out.samples[1];
        req.start_position = actual.position;
        req.start_velocity = Some(actual.velocity);
        req.start_yaw = actual.yaw;
        req.start_yaw_rate = Some(actual.yaw_rate);
        if req
            .obstacles
            .iter()
            .all(|&cell| escape_cell_depth(&req.config, cell, actual.position, actual.yaw) < -1e-10)
        {
            return (first, tick as usize + 1);
        }
    }
    panic!("原预算与新实测进展下未完全退出浅重叠");
}

#[test]
fn shallow_escape_covers_forward_reverse_and_side_wall() {
    for (start, cell, expected) in [
        ([0.29, 0.025, 0.35], [-0.025, 0.025, 0.35], [1., 0.]),
        ([-0.29, 0.025, 0.35], [0.025, 0.025, 0.35], [-1., 0.]),
        ([0.025, 0.09, 0.35], [0.025, -0.025, 0.35], [0., 1.]),
    ] {
        let route = [start, [1., 0.025, 0.35]];
        let obstacles = [cell];
        let mut req = shallow_surface_request(&route, &obstacles);
        req.config.body_length = 0.6;
        req.config.sample_dt = 0.05;
        req.config.max_acceleration = 0.5;
        let (command, _) = run_shallow_escape(req);
        assert!(command[0] * expected[0] + command[1] * expected[1] > 0.);
        assert_eq!(command[2], 0.);
    }
}

#[test]
fn stopped_noise_starts_escape_then_depth_change_keeps_the_budget() {
    let route=[[0.295,0.025,0.35],[1.,0.025,0.35]];
    let obstacles=[[-0.025,0.025,0.35]];
    let mut req=shallow_surface_request(&route,&obstacles);
    req.config.body_length=0.6;
    req.config.body_width=0.32;
    req.config.sample_dt=0.05;
    req.start_velocity=Some([-0.001,0.,0.]);
    req.odometry_timestamp_ns=1_000_000_000;
    let mut planner=registered_fixture(&req);
    let first=planner.step(&req).unwrap();
    assert!(first.command_forward>0.,"停稳口径内的测速噪声按零速证明起步");
    req.start_position[0]-=0.003336350098702423;
    req.odometry_timestamp_ns+=100_000_000;
    let changed=planner.step(&req).unwrap();
    assert!(changed.command_forward>0.,"测速噪声内的数值增深只更新基准，继续安全退出");
    check_escape_sweep(&req.config,req.obstacles,&changed.samples);
    req.odometry_timestamp_ns=3_100_000_000;
    let expired=planner.step(&req).unwrap_err();
    assert!(expired.contains("budget exhausted"),"更新基准不能续领预算：{expired}");
}

#[test]
fn issued_escape_measured_inward_motion_stops_then_rebuilds() {
    let route=[[0.295,0.025,0.35],[1.,0.025,0.35]];
    let obstacles=[[-0.025,0.025,0.35]];
    let mut req=shallow_surface_request(&route,&obstacles);
    req.config.body_length=0.6;req.config.body_width=0.32;req.config.sample_dt=0.05;
    let mut planner=registered_fixture(&req);
    assert!(planner.step(&req).unwrap().command_forward>0.);
    req.odometry_timestamp_ns+=50_000_000;
    req.start_velocity=Some([-0.03,0.,0.]);
    let failure=planner.step(&req).unwrap_err();
    assert!(failure.contains("measured escape penetration increased"),"实测真的向内必须当拍停：{failure}");
    req.odometry_timestamp_ns+=50_000_000;
    assert!(planner.step(&req).unwrap_err().contains("waiting for measured stop"));
    req.odometry_timestamp_ns+=50_000_000;
    req.start_velocity=Some([0.;3]);
    assert!(planner.step(&req).unwrap().command_forward>0.);
}

#[test]
fn unissued_depth_updates_still_reject_more_than_three_voxels() {
    let route=[[0.295,0.025,0.35],[1.,0.025,0.35]];
    let obstacles=[[-0.025,0.025,0.35]];
    let mut req=shallow_surface_request(&route,&obstacles);
    req.config.body_length=0.6;
    req.config.body_width=0.6;
    req.start_velocity=Some([-0.001,0.,0.]);
    let mut planner=registered_fixture(&req);
    assert!(planner.step(&req).is_ok());
    req.start_position[0]-=0.16;
    req.odometry_timestamp_ns+=50_000_000;
    assert!(planner.step(&req).unwrap_err().contains("exceeds localization and mapping allowance"));
}

#[test]
fn issued_escape_depth_increase_stops_then_rebuilds_after_measured_stop() {
    let route=[[0.295,0.025,0.35],[1.,0.025,0.35]];
    let obstacles=[[-0.025,0.025,0.35]];
    let mut req=shallow_surface_request(&route,&obstacles);
    req.config.body_length=0.6;req.config.body_width=0.6;
    let mut planner=registered_fixture(&req);
    let first=planner.step(&req).unwrap();
    assert!(first.command_forward!=0.||first.command_lateral!=0.||first.command_yaw_rate!=0.);
    req.start_position[0]-=0.16;req.odometry_timestamp_ns+=50_000_000;
    assert!(planner.step(&req).unwrap_err().contains("exceeds localization and mapping allowance"));
    req.start_position=route[0];req.odometry_timestamp_ns+=50_000_000;
    let resumed=planner.step(&req).unwrap();
    assert!(resumed.command_forward>0.,"停稳后按当前浅接触重新证明");
    check_escape_sweep(&req.config,&obstacles,&resumed.samples);
}

#[test]
fn shallow_escape_can_reverse_at_escape_speed_when_navigation_reverse_is_disabled() {
    let route = [[-0.29, 0.025, 0.35], [1., 0.025, 0.35]];
    let obstacles = [[0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &obstacles);
    req.config.body_length = 0.6;
    req.config.sample_dt = 0.05;
    req.config.max_acceleration = 0.5;
    req.config.max_reverse_velocity = 0.;
    let (command, ticks) = run_shallow_escape(req);
    assert!(command[0] < 0. && command[0] >= -0.04);
    assert_eq!(command[1], 0.);
    assert_eq!(command[2], 0.);
    assert!(ticks > 0);
}

#[test]
fn shallow_escape_covers_both_in_place_turns_without_lateral_motion() {
    for sign in [-1., 1.] {
        let route = [[0.015, -sign * 0.01, 0.35], [1., 0., 0.35]];
        let obstacles = [[0.325, -sign * 0.125, 0.35]];
        let mut req = shallow_surface_request(&route, &obstacles);
        req.config.body_length = 0.6;
        req.config.sample_dt = 0.05;
        req.config.max_acceleration = 0.5;
        req.config.max_reverse_velocity = 0.;
        req.config.max_lateral_velocity = 0.;
        let (command, _) = run_shallow_escape(req);
        assert!(command[0]>=-0.04 && command[0]<=0.);
        assert_eq!(command[1], 0.);
        assert!(command[2]*sign>0. || (command[0]<0. && command[2]==0.));
    }
}

#[test]
fn shallow_escape_combines_translation_and_turn_for_opposing_contacts() {
    let route = [[0., 0., 0.35], [1., 0., 0.35]];
    let obstacles = [[0.225, 0.225, 0.35], [-0.225, 0.125, 0.35]];
    let mut req = shallow_surface_request(&route, &obstacles);
    req.config.body_length = 0.402;
    req.config.body_width = 0.5;
    req.config.sample_dt = 0.05;
    req.config.max_acceleration = 0.5;
    req.config.max_yaw_acceleration = 20.;
    req.config.max_reverse_velocity = 0.;
    req.config.max_lateral_velocity = 0.;
    let (command, _) = run_shallow_escape(req);
    assert!(
        command[0] > 0. && command[2] > 0.,
        "两个反向接触使单独前进和单独转向都增深，必须采用组合：{command:?}"
    );
    assert_eq!(command[1], 0.);
}

// 第七轮实际配置和控制状态，见N6-19b归档；仅重放两成员冻结几何，不代替现场回放。
fn n6_20_r7_chain() -> (Config, Vec<MotionState>) {
    let config = Config {
        resolution: 0.05000000074505806,
        body_length: 0.6789949835614777,
        body_width: 0.32800000000271845,
        body_height: 0.4192421636189938,
        body_to_ground: 0.40456216361899383,
        max_velocity: 0.999999999999993,
        max_acceleration: 0.5,
        max_yaw_rate: 0.9999999999999929,
        planning_horizon: 5.0,
        sample_dt: 0.05,
        body_center_x: 0.026072508220619156,
        body_center_y: 1.2490009027033011e-15,
        body_z_min: -0.40456216361899383,
        body_z_max: 0.01467999999999997,
        max_step_up: 0.15000000596046448,
        max_step_down: 0.15000000596046448,
        max_slope: 0.3141592653589793,
        max_yaw_acceleration: 20.0,
        max_reverse_velocity: 0.0,
        max_lateral_velocity: 0.1744299999999988,
        obstacle_clearance_m: 0.15, min_executable_linear_mps: 0., min_executable_angular_rps: 0.,
    };
    let states = vec![
        MotionState { position: [4.72722390133869, 0.42264081558236116, 0.01171238755107227], velocity: [0.06811423959850842, 0.008768917491102436, 0.008467873319189185], yaw: -0.03328296349112102, yaw_rate: 0.010022572526697686, timestamp_ns: 1794298148785357237, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.72722390133869, 0.42264081558236116, 0.01171238755107227], velocity: [0.035887512554491144, 0.0016603284179003222, 0.003158228531914008], yaw: -0.03328296349112102, yaw_rate: 0.006510557240651799, timestamp_ns: 1794298148785357237, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.72722390133869, 0.42264081558236116, 0.01171238755107227], velocity: [0.020013526008807115, 7.993200711820137e-05, 0.001013686975710591], yaw: -0.03328296349112102, yaw_rate: 0.004919954283740841, timestamp_ns: 1794298148785357237, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.722007082196579, 0.4382828527061481, 0.011661074650343618], velocity: [0.0057499880503056695, -1.305471447636869e-05, 0.000681039640515301], yaw: -0.03076372481142398, yaw_rate: 0.011582977103143663, timestamp_ns: 1794298148885357380, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.722007082196579, 0.4382828527061481, 0.011661074650343618], velocity: [-0.003290523345176596, -1.058517319030769e-05, -0.0010164079900234345], yaw: -0.03076372481142398, yaw_rate: 0.006363378504906166, timestamp_ns: 1794298148885357380, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.722007082196579, 0.4382828527061481, 0.011661074650343618], velocity: [-0.009266453706292895, -0.0006108299925772378, -0.002244369902946832], yaw: -0.03076372481142398, yaw_rate: 0.008138715621088815, timestamp_ns: 1794298148885357380, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.722007082196579, 0.4382828527061481, 0.011661074650343618], velocity: [-0.01712719854156793, -0.0009175525514785795, -0.0022842615443546283], yaw: -0.03076372481142398, yaw_rate: 0.007402043367983856, timestamp_ns: 1794298148885357380, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
        MotionState { position: [4.733302210106727, 0.459441763074898, 0.007176286139628278], velocity: [-0.02166703597346221, -0.0009381550518706292, -0.0026977672346478704], yaw: -0.025614626253419256, yaw_rate: 0.00794038421763137, timestamp_ns: 1794298149085357189, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() },
    ];
    (config, states)
}

#[test]
fn n6_20_r7_transient_clear_admits_new_member_with_safe_braking() {
    let (config, states) = n6_20_r7_chain();
    let r = config.resolution;
    let old = [101.5 * r, 4.5 * r, 0.5 * r];
    let new = [102.5 * r, 9.5 * r, -3.5 * r];
    let mut map = MapOwner::new(&config).unwrap();
    map.seed_occupied(&[], states[0].position);
    let mut tracker = ScanTracker::default();
    for (i, state) in states.iter().enumerate() {
        let obstacles = if i < 3 { vec![old] } else { vec![old, new] };
        map.update_surface(&obstacles, &[], &[], &[]).unwrap();
        let out = tracker.track_and_check(&config, &map.snapshot(), state, [0.; 3], None).unwrap();
        if i < 3 || i == states.len() - 1 {
            assert!(!out.recovery_samples.is_empty(), "第{i}个真实状态应产出有界恢复动作");
            check_escape_sweep(&config, &obstacles, &out.recovery_samples);
        } else {
            assert!(out.recovery_samples.is_empty(), "真实短暂清除帧没有重叠");
        }
    }
    let depth = escape_cell_depth(&config, new, states[7].position, states[7].yaw);
    assert!((depth - 0.00003119510568594386).abs() < 1e-12);
}

#[test]
fn n6_20_transient_clear_does_not_reissue_budget_and_half_second_ends_event() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let obstacle = [-0.025, 0.025, 0.35];
    let req = shallow_surface_request(&route, &[]);
    let mut map = MapOwner::new(&req.config).unwrap();
    map.seed_occupied(&[], req.start_position);
    let mut tracker = ScanTracker::default();
    let mut state = MotionState { position: req.start_position, measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() };
    let mut issued = 0;
    let failure = loop {
        tracker.clear_trajectory();
        map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
        match tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None) {
            Ok(out) => { assert!(!out.recovery_samples.is_empty()); issued += 1; }
            Err(error) => break error,
        }
        assert!(issued < 40, "清除抖动不能反复获得未确认行程预算");
        map.update_surface(&[], &[], &[], &[]).unwrap();
        tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
        // 每拍有新来源时间，但没有实测退出进展；单拍清除不能重复发预算。
        state.timestamp_ns += 50_000_000;
    };
    assert!(failure.message.contains("budget exhausted") || failure.message.contains("no monotone"));
    map.update_surface(&[], &[], &[], &[]).unwrap();
    let clear_start = state.timestamp_ns + 100_000_000;
    state.timestamp_ns = clear_start;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    state.timestamp_ns = clear_start + 499_999_999;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
    assert_eq!(tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap_err().message, failure.message);
    // 仅朝向入口遇到重叠也要打断连续清除，不能跨过这些占据拍累计半秒。
    map.update_surface(&[], &[], &[], &[]).unwrap();
    state.timestamp_ns = clear_start + 600_000_000;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
    state.timestamp_ns = clear_start + 700_000_000;
    assert!(tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], Some(0.)).is_err());
    map.update_surface(&[], &[], &[], &[]).unwrap();
    state.timestamp_ns = clear_start + 1_100_000_000;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
    state.timestamp_ns = clear_start + 1_200_000_000;
    assert_eq!(tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap_err().message, failure.message);
    // 连续半秒无重叠后，原地也可以建立新事件。
    map.update_surface(&[], &[], &[], &[]).unwrap();
    state.timestamp_ns = clear_start + 1_300_000_000;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    state.timestamp_ns = clear_start + 1_800_000_000;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
    let out = tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    assert!(!out.recovery_samples.is_empty());
    check_escape_sweep(&req.config, &[obstacle], &out.recovery_samples);
}

#[test]
fn n6_20_no_safe_candidate_stops_current_tick_and_rebuilds_when_stopped() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let obstacle = [-0.025, 0.025, 0.35];
    let req = shallow_surface_request(&route, &[]);
    let mut map = MapOwner::new(&req.config).unwrap();
    map.seed_occupied(&[], req.start_position);
    map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let mut tracker = ScanTracker::default();
    let mut state = MotionState {
        position: req.start_position, velocity: [-0.02, 0., 0.],
        measured_velocity: 1, measured_yaw_rate: 1, ..Default::default()
    };
    let first = tracker.track_and_check(&req.config, &snapshot, &state, [0.; 3], None).unwrap_err();
    assert!(first.message.contains("no safe escape candidate"));
    state.velocity = [0.; 3];
    state.timestamp_ns += 50_000_000;
    let safe = tracker.track_and_check(&req.config, &snapshot, &state, [0.; 3], None).unwrap();
    check_escape_sweep(&req.config, &[obstacle], &safe.recovery_samples);
    state.velocity = [-0.02, 0., 0.];
    state.timestamp_ns += 50_000_000;
    let fault = tracker.track_and_check(&req.config, &snapshot, &state, [0.; 3], None).unwrap_err();
    assert!(fault.message.contains("no safe escape candidate"));
    state.velocity = [0.; 3];
    state.timestamp_ns += 50_000_000;
    tracker.clear_trajectory();
    let resumed=tracker.track_and_check(&req.config,&snapshot,&state,[0.;3],None).unwrap();
    check_escape_sweep(&req.config,&[obstacle],&resumed.recovery_samples);
    tracker.clear();
    let safe = tracker.track_and_check(&req.config, &snapshot, &state, [0.; 3], None).unwrap();
    check_escape_sweep(&req.config, &[obstacle], &safe.recovery_samples);
}

#[test]
fn n6_20_healthy_event_ends_after_half_second_of_source_clear() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let obstacle = [-0.025, 0.025, 0.35];
    let req = shallow_surface_request(&route, &[]);
    let mut map = MapOwner::new(&req.config).unwrap();
    map.seed_occupied(&[], req.start_position);
    let mut tracker = ScanTracker::default();
    let mut state = MotionState {
        position: req.start_position, measured_velocity: 1, measured_yaw_rate: 1,
        ..Default::default()
    };
    map.update_surface(&[obstacle], &[], &[], &[]).unwrap();
    let safe = tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    check_escape_sweep(&req.config, &[obstacle], &safe.recovery_samples);
    map.update_surface(&[], &[], &[], &[]).unwrap();
    state.timestamp_ns = 100_000_000;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    state.timestamp_ns = 600_000_000;
    tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    // 原地结束事件后，另一侧的接触须重新选择方向，不能继承旧方向或距离解锁条件。
    let opposite = [0.175, 0.025, 0.35];
    map.update_surface(&[opposite], &[], &[], &[]).unwrap();
    let safe = tracker.track_and_check(&req.config, &map.snapshot(), &state, [0.; 3], None).unwrap();
    assert!(safe.command[0] < 0.);
    check_escape_sweep(&req.config, &[opposite], &safe.recovery_samples);
}

#[test]
fn escape_repeated_source_reuses_only_the_proven_number_of_commands() {
    let route=[[0.295,0.025,0.35],[1.,0.025,0.35]];
    let obstacles=[[-0.025,0.025,0.35]];
    let mut req=shallow_surface_request(&route,&obstacles);
    req.config.body_length=0.6;
    req.config.sample_dt=0.05;
    let mut planner=registered_fixture(&req);
    let first=planner.step(&req).unwrap();
    let repeated=planner.step(&req).unwrap();
    assert_eq!([repeated.command_forward,repeated.command_lateral,repeated.command_yaw_rate],[0.;3],"无上一来源时K=1");
    req.odometry_timestamp_ns+=100_000_000;
    let next=planner.step(&req).unwrap();
    assert!(next.command_forward>0.);
    check_escape_sweep(&req.config,&obstacles,&next.samples);
    assert!(next.samples.len()>=3,"证明须包含两拍命令前进和制动");
    let repeat=planner.step(&req).unwrap();
    assert_eq!([repeat.command_forward,repeat.command_lateral,repeat.command_yaw_rate],
        [next.command_forward,next.command_lateral,next.command_yaw_rate],"K=2的第二次调用沿用已证明命令");
    check_escape_sweep(&req.config,&obstacles,&repeat.samples);
    let beyond=planner.step(&req).unwrap();
    assert_eq!([beyond.command_forward,beyond.command_lateral,beyond.command_yaw_rate],[0.;3],"超出K必须停车");
    assert!(first.command_forward>0.);
}
