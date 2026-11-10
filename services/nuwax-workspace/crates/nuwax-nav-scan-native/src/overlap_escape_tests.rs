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
        assert!(out.command_forward >= -req.config.max_reverse_velocity);
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
fn shallow_escape_uses_tangent_plateau_when_reverse_is_disabled() {
    let route = [[-0.29, 0.025, 0.35], [1., 0.025, 0.35]];
    let obstacles = [[0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &obstacles);
    req.config.body_length = 0.6;
    req.config.sample_dt = 0.05;
    req.config.max_acceleration = 0.5;
    req.config.max_reverse_velocity = 0.;
    let (command, ticks) = run_shallow_escape(req);
    assert_eq!(command[0], 0.);
    assert!(command[1] < 0.);
    assert_eq!(command[2], 0.);
    assert!(ticks > 40, "实测切向退出应跨越最初两秒窗口，不能只验证首拍");
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
        assert_eq!(command[0], 0.);
        assert_eq!(command[1], 0.);
        assert!(command[2] * sign > 0.);
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
