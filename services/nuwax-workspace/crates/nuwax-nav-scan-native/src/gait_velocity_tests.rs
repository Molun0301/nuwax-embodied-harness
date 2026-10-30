// 独立执行器跟踪行进命令，额外叠加真实步态横摆的速度和位移。
// 位置与测速同源，不以平滑后的速度积分来掩盖真实横摆。
#[test]
fn gait_velocity_straight_route_reduces_reversals_and_arrives() {
    let mut smooth = true;
    for frequency in [1., 1.5, 2.] {
        let route = [[0., 0., 0.35], [3., 0., 0.35]];
        let mut req = request(&route, &[]);
        req.config.max_velocity = 0.3;
        req.config.max_lateral_velocity = 0.2;
        req.config.max_acceleration = 0.5;
        req.config.sample_dt = 0.05;
        req.config.planning_horizon = 2.;
        let mut planner = fixture_planner();
        let mut slow_position = req.start_position;
        let mut slow_velocity = [0_f64; 2];
        let mut angular = 0_f64;
        let omega = 2. * std::f64::consts::PI * frequency;
        let mut lateral = Vec::new();
        let mut arrived = false;
        let mut arrival_tick = 0;
        for tick in 0..600 {
            let t = tick as f64 * req.config.sample_dt;
            req.odometry_timestamp_ns = 1_000_000_000 + tick * 50_000_000;
            req.start_position = [slow_position[0],
                slow_position[1] + 0.15 / omega * (1. - (omega * t).cos()), 0.35];
            req.start_velocity = Some([slow_velocity[0],
                slow_velocity[1] + 0.15 * (omega * t).sin(), 0.]);
            req.start_yaw_rate = Some(angular);
            let out = planner.step(&req).unwrap();
            assert_device_box(&out, &req.config);
            if t >= 2. { lateral.push(out.command_lateral); }
            let (c, s) = (req.start_yaw.cos(), req.start_yaw.sin());
            let target = [c * out.command_forward - s * out.command_lateral,
                s * out.command_forward + c * out.command_lateral];
            let delta = [target[0] - slow_velocity[0], target[1] - slow_velocity[1]];
            let distance = delta[0].hypot(delta[1]);
            let scale = if distance == 0. { 1. } else {
                (req.config.max_acceleration * out.command_duration / distance).min(1.)
            };
            for axis in 0..2 {
                let next = slow_velocity[axis] + scale * delta[axis];
                slow_position[axis] += (slow_velocity[axis] + next) * 0.5 * out.command_duration;
                slow_velocity[axis] = next;
            }
            let next_angular = angular + (out.command_yaw_rate - angular).clamp(
                -req.config.max_yaw_acceleration * out.command_duration,
                req.config.max_yaw_acceleration * out.command_duration);
            req.start_yaw += (angular + next_angular) * 0.5 * out.command_duration;
            angular = next_angular;
            if (req.start_position[0] - 3.).hypot(req.start_position[1]) < 0.15 {
                arrived = true; arrival_tick = tick; break;
            }
        }
        let flips = |threshold: f64| {
            let signs: Vec<_> = lateral.iter().filter(|v| v.abs() > threshold)
                .map(|v| v.signum()).collect();
            signs.windows(2).filter(|s| s[0] != s[1]).count()
        };
        let crossings = flips(0.02);
        let reversals = flips(0.08);
        let seconds = lateral.len() as f64 * req.config.sample_dt;
        let rms = (lateral.iter().map(|v| v * v).sum::<f64>() / lateral.len() as f64).sqrt();
        eprintln!("步态{frequency}赫兹：到达={arrived}，拍数={arrival_tick}，统计{seconds:.2}秒，跨零={crossings}，强反向={reversals}，反向率={:.4}/秒，横移均方根={rms:.6}", reversals as f64 / seconds);
        assert!(arrived, "有步态横摆时仍须在原600拍内到达：{:?}", req.start_position);
        // 每周期两次是步态同频翻转；要求至少抑制八成，并保留轨迹本身的转向纠偏。
        smooth &= crossings as f64 / seconds < frequency * 0.4
            && reversals as f64 / seconds < frequency * 0.4;
    }
    assert!(smooth, "导航横移仍跟随步态翻转");
}

#[test]
fn gait_velocity_intentional_turn_responds_on_the_next_tick() {
    let mut req = request(&[], &[]);
    device_limit_config(&mut req);
    let mut planner = fixture_planner();
    planner.step(&req).unwrap();
    req.odometry_timestamp_ns += 50_000_000;
    req.start_velocity = Some([0., 0.15, 0.]);
    req.goal_yaw = Some(0.5);
    let out = planner.step(&req).unwrap();
    assert_eq!(out.command_lateral, 0., "有意原地转向不能被步态横速带成横移");
    assert_eq!(out.command_forward, 0.);
    assert!(out.command_yaw_rate > 0. && out.command_yaw_rate
        <= req.config.max_yaw_acceleration * req.config.sample_dt);
    eprintln!("有意转向：{}秒内响应，角速{}弧度/秒", out.command_duration, out.command_yaw_rate);
}

#[test]
fn gait_velocity_raw_lateral_braking_is_not_replaced_by_the_smooth_state() {
    for sign in [-1., 1.] {
        let obstacles = [[0.025, sign * 0.125, 0.35]];
        let mut req = request(&[], &obstacles);
        device_limit_config(&mut req);
        req.config.resolution = 0.05;
        req.config.body_length = 0.1;
        req.config.body_width = 0.1;
        let mut planner = fixture_planner();
        assert!(planner.step(&req).is_ok(), "原始静止状态的机体及停车扫掠可行");
        req.odometry_timestamp_ns += 50_000_000;
        req.start_velocity = Some([0., sign * 0.4, 0.]);
        // 平滑值此时仅约0.02米/秒，命令起点仍为零；原始横速的刹停距离
        // 为0.16米，必须被真实扫掠拒绝，不能用平滑值证明停车安全。
        let error = planner.step(&req).unwrap_err();
        assert!(error.contains("braking sweep"), "真实横移制动碰撞必须拒绝：{error}");
    }
}

#[test]
fn gait_velocity_repeated_sample_and_reset_keep_the_command_origin() {
    let mut req = request(&[], &[]);
    device_limit_config(&mut req);
    let mut planner = fixture_planner();
    req.start_velocity = Some([0., 0.15, 0.]);
    let first = planner.step(&req).unwrap();
    assert!((first.command_lateral - 0.125).abs() < 1e-12);
    let repeated = planner.step(&req).unwrap();
    assert_eq!(first.command_lateral, repeated.command_lateral);
    req.odometry_timestamp_ns += 50_000_000;
    req.start_velocity = Some([0., -0.15, 0.]);
    let next = planner.step(&req).unwrap();
    assert!((next.command_lateral - 0.1).abs() < 1e-12,
        "下一拍从已发命令推进，不跟随实测横速翻转");
    assert_eq!(planner.step(&req).unwrap().command_lateral, next.command_lateral);
    planner.reset_execution();
    let reset = planner.step(&req).unwrap();
    assert!((reset.command_lateral + 0.125).abs() < 1e-12,
        "执行退休后必须使用新的真实初始条件");
    req.odometry_timestamp_ns += 50_000_000;
    req.start_yaw = 0.7;
    req.start_velocity = Some([
        -req.start_yaw.sin() * reset.command_lateral,
        req.start_yaw.cos() * reset.command_lateral, 0.]);
    let turned = planner.step(&req).unwrap();
    assert!((turned.command_lateral + 0.1).abs() < 1e-12);
    assert!(turned.command_forward.abs() < 1e-12,
        "转向后的命令起点必须仍是上一拍机体系请求");
}

#[test]
fn gait_velocity_planning_boundary_uses_slow_feedback_and_reset_clears_it() {
    let route = [[0., 0., 0.35], [3., 0., 0.35]];
    let mut req = request(&[], &[]);
    device_limit_config(&mut req);
    req.odometry_timestamp_ns = 1_000_000_000;
    let mut planner = fixture_planner();
    planner.step(&req).unwrap();
    req.reference = &route;
    req.start_velocity = Some([0., 0.15, 0.]);
    let mut planned = false;
    for _ in 0..8 {
        req.odometry_timestamp_ns += 50_000_000;
        let out = planner.step(&req).unwrap();
        if let Some(first) = out.samples.first() {
            assert_eq!(out.motion_boundary_sources[0], MotionBoundarySource::SmoothedMeasured);
            assert!(first.velocity[1].abs() < 0.05,
                "规划边界仍带入0.15米/秒的瞬时横速：{first:?}");
            planned = true;
            break;
        }
    }
    assert!(planned, "定向测试必须真正生成局部曲线");
    planner.reset_execution();
    let mut reset_planned = false;
    for _ in 0..8 {
        req.odometry_timestamp_ns += 50_000_000;
        let out = planner.step(&req).unwrap();
        if let Some(first) = out.samples.first() {
            assert!(first.velocity[1] > 0.1, "重置后保留真实非零初始状态：{first:?}");
            reset_planned = true;
            break;
        }
    }
    assert!(reset_planned);
}
