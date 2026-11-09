include!("short_seed_checks.rs");

// 全局图已知墙体、局部概率图暂未观测墙体时，完整参考仍必须决定绕行方向。
#[test]
fn global_detour_is_not_replaced_by_a_straight_local_curve() {
    check_detour_curve(5.);
}

#[test]
fn local_detour_window_keeps_the_global_corner() {
    check_detour_curve(3.);
}

#[test]
fn stationary_centimeter_approach_produces_a_dynamically_feasible_curve() {
    let mut cfg = config();
    cfg.max_velocity = 0.3;
    cfg.max_acceleration = 1.;
    cfg.sample_dt = 0.05;
    // 窄门走完后的剩余距离量级，包含平面短程和原机体与地面参考的高度差。
    for distance in [0.02, 0.0325, 0.05] {
        let route = [[0., 0., 0.4], [distance, 0., 0.395]];
        let mut map = MapOwner::new(&cfg).unwrap();
        map.seed_occupied(&[], route[0]);
        map.update_surface(&[], &[], &[], &[]).unwrap();
        let state = MotionState { position: route[0], measured_velocity: 1,
            measured_yaw_rate: 1, timestamp_ns: 1_000_000_000, ..MotionState::default() };
        let curve = ScanPlanner::default().plan(&cfg, &map.snapshot(), &route, &state, None, 0.)
            .expect("静止几厘米接近必须在原求解调用中产出可执行轨迹");
        let samples = curve.samples(0., 0.001);
        for sample in &samples {
            let speed = sample.velocity.iter().map(|v| v*v).sum::<f64>().sqrt();
            let acceleration = sample.acceleration.iter().map(|v| v*v).sum::<f64>().sqrt();
            assert!(speed <= cfg.max_velocity + 1e-8, "短程速度越界：{sample:?}");
            assert!(acceleration <= cfg.max_acceleration + 1e-8, "短程加速度越界：{sample:?}");
        }
        assert!(samples.last().unwrap().position[0] > distance * 0.9,
            "短程曲线必须实际覆盖剩余接近距离：{samples:?}");
    }
}

#[test]
fn stationary_short_seed_uses_the_fitted_spline_dynamic_bound() {
    let mut cfg = config();
    cfg.max_velocity = 0.5;
    cfg.max_acceleration = 0.5;
    for distance in [0.03_f64, 0.4] {
        let route = [[0., 0., 0.4], [distance, 0., 0.4]];
        let mut map = MapOwner::new(&cfg).unwrap();
        map.seed_occupied(&[], route[0]);
        map.update_surface(&[], &[], &[], &[]).unwrap();
        let state = MotionState { position: route[0], measured_velocity: 1,
            measured_yaw_rate: 1, timestamp_ns: 1_000_000_000, ..MotionState::default() };
        let curve = ScanPlanner::default().plan(&cfg, &map.snapshot(), &route, &state, None, 0.)
            .expect("紧化短程初值必须通过原规划接纳检查");
        let samples = curve.samples(0., 0.001);
        let before = (5. * distance / cfg.max_velocity).max((20. * distance / cfg.max_acceleration).sqrt());
        let after = samples.last().unwrap().time;
        eprintln!("短程 {distance} 米：原五次初值={before:.9} 秒，现轨迹={after:.9} 秒");
        assert!(after < before * 0.75, "拟合后的紧界应明显缩短短程：{after}/{before}");
        for sample in &samples {
            assert!(sample.velocity.iter().map(|v| v*v).sum::<f64>().sqrt() <= cfg.max_velocity + 1e-8);
            assert!(sample.acceleration.iter().map(|v| v*v).sum::<f64>().sqrt() <= cfg.max_acceleration + 1e-8);
        }
        assert!(samples.last().unwrap().position[0] > distance * 0.9);
    }
}

#[test]
fn slow_retimed_detour_accelerates_and_advances_under_command_integration() {
    let route = [[0., 0., 0.35], [0.4, 0., 0.45], [0.4, -0.4, 0.45], [2.4, -0.4, 0.45], [2.4, -1.7, 0.45]];
    let mut cfg = config();
    cfg.planning_horizon = 5.;
    cfg.sample_dt = 0.05;
    cfg.max_velocity = 0.5;
    cfg.max_acceleration = 0.5;
    let mut map = MapOwner::new(&cfg).unwrap();
    map.seed_occupied(&[], route[0]);
    map.update_surface(&[], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let mut state = MotionState {
        position: route[0], timestamp_ns: 1_000_000_000,
        measured_velocity: 1, measured_yaw_rate: 1, ..MotionState::default()
    };
    let curve = ScanPlanner::default().plan(&cfg, &snapshot, &route, &state, None, 0.).unwrap();
    let samples = curve.samples(0., cfg.sample_dt);
    let speed = |s: &TrajectorySample| s.velocity[0].hypot(s.velocity[1]);
    let normal_speed = samples.iter().map(speed).fold(0., f64::max);
    eprintln!("轨迹时长={}，起步速度={}，正常速度={normal_speed}", samples.last().unwrap().time, speed(&samples[1]));
    assert!(samples.last().unwrap().time > 10., "覆盖重定时后的长绕行");
    assert!(speed(&samples[1]) < 0.005 && normal_speed > 0.2,
        "覆盖慢起步和后续正常速度：起步={}，正常={normal_speed}", speed(&samples[1]));
    let mut tracker = ScanTracker::default();
    tracker.install(&curve);
    // 机体高度相对参考低2厘米，仍在原偏离阈值内；平面底盘不积分竖直速度。
    state.position[2] -= 0.02;
    let mut previous = [0.; 3];
    let mut max_speed: f64 = 0.;
    let mut phase = 0.;
    // 不重规划、不移动轨迹；实际位置仅由下发命令进行平面运动学积分。
    for tick in 0..160 {
        state.timestamp_ns = 1_000_000_000 + tick * 50_000_000;
        let tracked = tracker.track_and_check(&cfg, &snapshot, &state, previous, None).unwrap();
        assert!(!tracked.invalid() && tracked.deviation <= 0.2, "第{tick}拍：{tracked:?}");
        if tick == 0 {
            let expected = samples[1].velocity[0] + 0.8 * (samples[1].position[0] - state.position[0]);
            assert!(tracked.installed && tracked.phase == 0. && (tracked.command[0] - expected).abs() < 1e-10,
                "新轨应从投影初始化后执行首个控制周期：{tracked:?}");
        }
        let [vx, vy, w] = tracked.command;
        max_speed = max_speed.max(vx.hypot(vy));
        let (cs, sn) = (state.yaw.cos(), state.yaw.sin());
        state.velocity = [cs * vx - sn * vy, sn * vx + cs * vy, 0.];
        state.position[0] += state.velocity[0] * cfg.sample_dt;
        state.position[1] += state.velocity[1] * cfg.sample_dt;
        state.yaw += w * cfg.sample_dt;
        state.yaw_rate = w;
        let mut arc = 0.;
        let mut nearest = (f64::INFINITY, 0.);
        for pair in route.windows(2) {
            let delta: [f64; 3] = std::array::from_fn(|i| pair[1][i] - pair[0][i]);
            let length2: f64 = delta.iter().map(|v| v * v).sum();
            let fraction = ((0..3).map(|i| (state.position[i] - pair[0][i]) * delta[i]).sum::<f64>() / length2).clamp(0., 1.);
            let distance: f64 = (0..3).map(|i| (state.position[i] - pair[0][i] - fraction * delta[i]).powi(2)).sum();
            if distance < nearest.0 { nearest = (distance, arc + fraction * length2.sqrt()); }
            arc += length2.sqrt();
        }
        state.route_progress = nearest.1;
        previous = tracked.command;
        phase = tracked.phase;
    }
    eprintln!("慢起步160拍：最大速度={max_speed:.6}，正常速度={normal_speed:.6}，位置={:?}，实际投影={phase:.6}", state.position);
    assert!(max_speed >= normal_speed * 0.7, "应在160拍内升到轨迹正常速度：{max_speed}/{normal_speed}");
    assert!(state.position[0] > 0.8 && phase > 3., "实际运动必须沿路线前进：{:?}，投影={phase}", state.position);
}

#[test]
fn reference_time_waits_at_deviation_bound_and_restarts_at_installed_projection() {
    let route = [[0., 0., 0.35], [4., 0., 0.35]];
    let mut cfg = config();
    cfg.sample_dt = 0.05;
    cfg.planning_horizon = 5.;
    // 慢参考曲线配合原设备限速，使控制量未饱和，能独立核对实际取样点。
    let mut planning = cfg;
    planning.max_velocity = 0.1;
    let mut map = MapOwner::new(&cfg).unwrap();
    map.seed_occupied(&[], route[0]);
    map.update_surface(&[], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let mut state = MotionState { position: route[0], measured_velocity: 1,
        measured_yaw_rate: 1, timestamp_ns: 1_000_000_000, ..MotionState::default() };
    let curve = ScanPlanner::default().plan(&planning, &snapshot, &route, &state, None, 0.).unwrap();
    let samples = curve.samples(0., cfg.sample_dt);
    let bound = samples.iter().take_while(|s| s.position[0] <= 0.2).last().unwrap();
    let expected = bound.velocity[0] + 0.8 * bound.position[0];
    let mut tracker = ScanTracker::default();
    tracker.install(&curve);
    let mut previous = [0.; 3];
    for tick in 0..400 {
        state.timestamp_ns = 1_000_000_000 + tick * 50_000_000;
        let tracked = tracker.track_and_check(&cfg, &snapshot, &state, previous, None).unwrap();
        assert!(!tracked.invalid() && !tracked.replan_early());
        assert_eq!(tracked.phase, 0., "静止反馈不能被期望时钟记成实际行程");
        assert!((tracked.remaining - samples.last().unwrap().time).abs() < 1e-10);
        if tick >= 300 {
            assert!((tracked.command[0] - expected).abs() < 1e-8,
                "候选参考点超出20厘米后必须停表：{}，期望={expected}", tracked.command[0]);
        }
        previous = tracked.command;
    }
    // 实际位置前移后，参考不能落在实际投影之后；重装原轨也不能继承旧期望时间。
    state.position = samples.iter().find(|s| s.position[0] > 0.8).unwrap().position;
    state.route_progress = state.position[0];
    state.timestamp_ns += 50_000_000;
    let advanced = tracker.track_and_check(&cfg, &snapshot, &state, [0.1, 0., 0.], None).unwrap();
    assert!(advanced.phase > bound.time && advanced.command[0] > 0.);
    let actual = curve.samples(advanced.phase, cfg.sample_dt)[0];
    let at_projection = actual.velocity[0] + 0.8 * (actual.position[0] - state.position[0]);
    assert!((advanced.command[0] - at_projection).abs() < 1e-8,
        "追平实际投影时不能再多前探一拍：{advanced:?}，投影处期望={at_projection}");
    tracker.install(&curve);
    let mut fresh = ScanTracker::default();
    fresh.install(&curve);
    let installed = tracker.track_and_check(&cfg, &snapshot, &state, [0.; 3], None).unwrap();
    let original = fresh.track_and_check(&cfg, &snapshot, &state, [0.; 3], None).unwrap();
    assert!(installed.installed && original.installed);
    assert_eq!(installed.phase, original.phase);
    assert_eq!(installed.command, original.command);
}

#[test]
fn reference_end_does_not_stop_before_actual_projection_finishes() {
    let route = [[0., 0., 0.35], [1., 0., 0.35]];
    let mut cfg = config();
    cfg.sample_dt = 0.05;
    let mut map = MapOwner::new(&cfg).unwrap();
    map.seed_occupied(&[], route[0]);
    map.update_surface(&[], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let mut state = MotionState { position: route[0], measured_velocity: 1,
        measured_yaw_rate: 1, timestamp_ns: 1_000_000_000, ..MotionState::default() };
    let curve = ScanPlanner::default().plan(&cfg, &snapshot, &route, &state, None, 0.).unwrap();
    let endpoint = *curve.samples(0., cfg.sample_dt).last().unwrap();
    let expected = endpoint.velocity[0] + 0.8 * (endpoint.position[0] - 0.9);
    eprintln!("实际样条末端={endpoint:?}，末端追踪期望={expected}");
    let mut tracker = ScanTracker::default();
    tracker.install(&curve);
    state.position[0] = 0.9;
    state.route_progress = 0.9;
    let mut previous = [0.; 3];
    for tick in 0..120 {
        state.timestamp_ns += 50_000_000;
        let tracked = tracker.track_and_check(&cfg, &snapshot, &state, previous, None).unwrap();
        assert!(!tracked.invalid() && tracked.remaining > 0.);
        if tick >= 100 {
            assert!((tracked.command[0] - expected).abs() < 1e-8 && tracked.command[0] > 0.05,
                "期望到末端后仍须追踪剩余实际位置，不能提前置零：{tracked:?}");
        }
        previous = tracked.command;
    }
}

fn check_detour_curve(horizon: f64) {
    let route = [[0., 0., 0.35], [2.4, 0., 0.35], [2.4, -1.7, 0.35]];
    let mut req = request(&route, &[]);
    req.config.planning_horizon = horizon;
    let mut planner = fixture_planner();
    let mut curve = None;
    for tick in 0..20 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let result = planner.step(&req).unwrap();
        if !result.samples.is_empty() {
            curve = Some(result);
            break;
        }
    }
    let curve = curve.expect("原局部求解器必须生成绕行轨迹，不能以永久等待代替跟随");
    assert!(curve.samples.len() > 2);
    let mut yaw = 0.;
    for sample in &curve.samples {
        if sample.velocity[0].hypot(sample.velocity[1]) > 1e-8 {
            yaw = sample.velocity[1].atan2(sample.velocity[0]);
        }
        // 独立矩形分离轴判断：全局墙体为[0.8,1.8]×[-1.1,-0.3]，
        // 原参考的两个线段与完整机身均有净空，首尾连线则穿墙。
        let (cs, sn) = (yaw.cos(), yaw.sin());
        let dx = 1.3 - sample.position[0];
        let dy = -0.7 - sample.position[1];
        let overlaps = dx.abs() < 0.35 * cs.abs() + 0.15 * sn.abs() + 0.5
            && dy.abs() < 0.35 * sn.abs() + 0.15 * cs.abs() + 0.4
            && (dx * cs + dy * sn).abs() < 0.35 + 0.5 * cs.abs() + 0.4 * sn.abs()
            && (-dx * sn + dy * cs).abs() < 0.15 + 0.5 * sn.abs() + 0.4 * cs.abs();
        assert!(!overlaps, "局部曲线切过全局参考绕开的已知墙体：{sample:?}");
    }
    let end = curve.samples.last().unwrap().position;
    assert!(end.into_iter().zip(curve.local_target).all(|(a,b)|(a-b).abs()<1e-6));
    if horizon == 5. {
        assert!((end[0] - 2.4).hypot(end[1] + 1.7) < 1e-6);
    }
}

#[test]
fn replanning_starts_at_the_measured_body_when_execution_lags() {
    let route = [[0., 0., 0.35], [4., 0., 0.35]];
    let updated_route = [[0., 0., 0.35], [4.1, 0., 0.35]];
    let mut req = request(&route, &[]);
    let mut planner = fixture_planner();
    let mut plans = 0;
    // 真实反馈保持静止，不能将已发布命令冒充已完成位移；连续推进原调度时刻。
    for tick in 0..80 {
        // 新全局参考强制执行中重规划；静止反馈不能由时间流逝伪装成已行走一米。
        if tick == 20 { req.reference = &updated_route; }
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let result = planner.step(&req).unwrap();
        if result.replanned && !result.samples.is_empty() {
            plans += 1;
            let p = result.samples[0].position;
            assert!(p[0].hypot(p[1]) <= req.config.max_velocity * req.config.sample_dt + 1e-6,
                "新轨迹的首位置必须绑定实际机体，不能续用旧计划位置：{p:?}");
        }
    }
    assert!(plans >= 2, "必须覆盖至少一次执行中重规划，不能只检查初次轨迹");
}

#[test]
fn global_detour_commands_reach_the_goal_without_cutting_the_wall() {
    let route = [[0., 0., 0.35], [2.4, 0., 0.35], [2.4, -1.7, 0.35]];
    let mut req = request(&route, &[]);
    req.config.planning_horizon = 5.;
    let mut planner = fixture_planner();
    // 沿用原闭环夹具600拍及实际命令积分，不把导出样条当成实际运动。
    for tick in 0..600 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let result = planner.step(&req).unwrap();
        let yaw = req.start_yaw;
        let world = req.start_velocity.unwrap();
        let previous = [yaw.cos()*world[0]+yaw.sin()*world[1], -yaw.sin()*world[0]+yaw.cos()*world[1]];
        let next = [result.command_forward,result.command_lateral];
        let dt = result.command_duration;
        assert!((next[0]-previous[0]).hypot(next[1]-previous[1]) <= req.config.max_acceleration*dt+1e-8);
        let v = [(previous[0]+next[0])/2., (previous[1]+next[1])/2.];
        let w = (req.start_yaw_rate.unwrap()+result.command_yaw_rate)/2.;
        let (a,b) = if w.abs()>1e-9 {((w*dt).sin()/w,(1.-(w*dt).cos())/w)} else {(dt,0.)};
        let dx=a*v[0]-b*v[1];let dy=b*v[0]+a*v[1];
        req.start_position[0]+=yaw.cos()*dx-yaw.sin()*dy;
        req.start_position[1]+=yaw.sin()*dx+yaw.cos()*dy;
        req.start_yaw+=w*dt;
        req.start_yaw_rate=Some(result.command_yaw_rate);
        req.start_velocity=Some([next[0]*req.start_yaw.cos()-next[1]*req.start_yaw.sin(),
            next[0]*req.start_yaw.sin()+next[1]*req.start_yaw.cos(),0.]);
        req.start_acceleration=None;
        let (cs,sn)=(req.start_yaw.cos(),req.start_yaw.sin());
        let dx=1.3-req.start_position[0];let dy=-0.7-req.start_position[1];
        let overlaps=dx.abs()<0.35*cs.abs()+0.15*sn.abs()+0.5
            && dy.abs()<0.35*sn.abs()+0.15*cs.abs()+0.4
            && (dx*cs+dy*sn).abs()<0.35+0.5*cs.abs()+0.4*sn.abs()
            && (-dx*sn+dy*cs).abs()<0.15+0.5*sn.abs()+0.4*cs.abs();
        assert!(!overlaps,"第{tick}拍实际机身切过全局墙体：{:?}",req.start_position);
        if (req.start_position[0]-2.4).hypot(req.start_position[1]+1.7)<0.15 {return;}
    }
    panic!("原600拍内没有沿绕行到达：{:?}",req.start_position);
}

// 固定线上原始47点PCT路线，不压缩拐角。
fn recorded_g1_route() -> [[f64; 3]; 47] {
    [
        [0.0008563928652691871, -0.0013738620895377847, 0.017955000894286918],
        [0.37500000558793545, -0.02500000037252903, 0.021329461809344474],
        [0.4250000063329935, -0.02500000037252903, 0.021329461809344474],
        [0.47500000707805157, -0.02500000037252903, 0.021329461809344474],
        [0.5250000078231096, -0.02500000037252903, 0.021329461809344474],
        [0.5750000085681677, -0.02500000037252903, 0.021329461809344474],
        [0.6250000093132257, -0.02500000037252903, 0.021329461809344474],
        [0.6750000100582838, -0.02500000037252903, 0.021329461809344474],
        [0.7250000108033419, -0.02500000037252903, 0.021329461809344474],
        [0.7750000115483999, -0.02500000037252903, 0.021329461809344474],
        [0.825000012293458, -0.02500000037252903, 0.021329461809344474],
        [0.875000013038516, -0.02500000037252903, 0.021329461809344474],
        [0.9250000137835741, 0.02500000037252903, 0.021329461809344474],
        [0.9750000145286322, 0.07500000111758709, 0.021329461809344474],
        [1.0250000152736902, 0.12500000186264515, 0.021329461809344474],
        [1.0750000160187483, 0.1750000026077032, 0.021329461809344474],
        [1.1250000167638063, 0.1750000026077032, 0.021329461809344474],
        [1.1750000175088644, 0.22500000335276127, 0.021329461809344474],
        [1.2250000182539225, 0.27500000409781933, 0.021329461809344474],
        [1.2750000189989805, 0.3250000048428774, 0.021329461809344474],
        [1.2750000189989805, 0.37500000558793545, 0.021329461809344474],
        [1.3250000197440386, 0.4250000063329935, 0.021329461809344474],
        [1.3250000197440386, 0.47500000707805157, 0.021329461809344474],
        [1.3250000197440386, 0.5250000078231096, 0.021329461809344474],
        [1.3250000197440386, 0.5750000085681677, 0.021329461809344474],
        [1.3250000197440386, 0.6250000093132257, 0.021329461809344474],
        [1.2750000189989805, 0.6750000100582838, 0.021329461809344474],
        [1.2750000189989805, 0.7250000108033419, 0.021329461809344474],
        [1.2250000182539225, 0.7750000115483999, 0.021329461809344474],
        [1.1750000175088644, 0.825000012293458, 0.021329461809344474],
        [1.1250000167638063, 0.825000012293458, 0.021329461809344474],
        [1.0750000160187483, 0.825000012293458, 0.021329461809344474],
        [1.0250000152736902, 0.825000012293458, 0.021329461809344474],
        [0.9750000145286322, 0.825000012293458, 0.021329461809344474],
        [0.9250000137835741, 0.825000012293458, 0.021329461809344474],
        [0.875000013038516, 0.825000012293458, 0.021329461809344474],
        [0.825000012293458, 0.825000012293458, 0.021329461809344474],
        [0.7750000115483999, 0.7750000115483999, 0.021329461809344474],
        [0.7250000108033419, 0.7750000115483999, 0.021329461809344474],
        [0.6750000100582838, 0.7750000115483999, 0.021329461809344474],
        [0.6250000093132257, 0.7750000115483999, 0.021329461809344474],
        [0.5750000085681677, 0.7750000115483999, 0.021329461809344474],
        [0.5250000078231096, 0.7750000115483999, 0.021329461809344474],
        [0.47500000707805157, 0.7250000108033419, 0.021329461809344474],
        [0.4250000063329935, 0.7250000108033419, 0.021329461809344474],
        [0.37500000558793545, 0.7750000115483999, 0.021329461809344474],
        [0.3250000048428774, 0.825000012293458, 0.021329461809344474],
    ]
}

#[test]
fn recorded_g1_stationary_start_produces_motion() {
    let route = recorded_g1_route();
    // 精确零、数值级微动、线上原采样；加速度缺测交由原初始化语义处理。
    for velocity in [[0.; 3], [1e-5, -1e-5, 0.],
        [0.0003822354977337651, 0.00022059553913678998, 0.01688324614029815]] {
        check_recorded_start(&route, velocity, false);
    }
}

#[test]
fn recorded_g1_registered_start_keeps_nonzero_velocity_boundary() {
    let route = recorded_g1_route();
    // 非零速度足以区分真正保留实测导数与偷偷归零。
    check_recorded_start(&route, [0.12, 0., 0.01], true);
}

fn check_recorded_start(route: &[[f64; 3]], velocity: [f64; 3], registered: bool) {
    let mut req = request(route, &[]);
    req.start_position = route[0];
    req.start_velocity = Some(velocity);
    req.start_acceleration = None;
    req.config.planning_horizon = 3.5;
    req.config.max_velocity = 0.5;
    req.config.max_acceleration = 0.5;
    req.config.sample_dt = 0.05;
    let mut planner = if registered { Planner::new() } else { fixture_planner() };
    if registered {
        // 真实注册帧入口：实测远处回波初始化概率图，不用静态地图开关。
        planner.ingest_registered_frame(&req.config, &[[0., 0., -1.]],
            [0., 0., 1.], req.start_position, 1_000_000_000).unwrap();
    }
    for tick in 0..20 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 50_000_000;
        let result = planner.step(&req).unwrap();
        if !result.samples.is_empty() && result.command_forward.hypot(result.command_lateral) > 1e-6 {
            assert_eq!(result.motion_boundary_sources[0], MotionBoundarySource::SmoothedMeasured);
            // 首次加速度沿用原初始化来源，不能把没有的观测伪装成实测。
            assert_eq!(result.motion_boundary_sources[1], MotionBoundarySource::InitialZeroAssumption);
            let first = result.samples[0];
            let delta = first.velocity.into_iter().zip(velocity).map(|(a,b)|(a-b).powi(2)).sum::<f64>().sqrt();
            assert!(delta <= req.config.max_acceleration * req.config.sample_dt + 1e-8,
                "首个执行采样必须兑现实测速度边界：{velocity:?} -> {first:?}");
            for sample in &result.samples {
                let speed = sample.velocity.iter().map(|v|v*v).sum::<f64>().sqrt();
                let acc = sample.acceleration.iter().map(|v|v*v).sum::<f64>().sqrt();
                assert!(speed <= req.config.max_velocity + 1e-8);
                assert!(acc <= req.config.max_acceleration + 1e-8);
            }
            return;
        }
    }
    panic!("静止或微动且输入有效时，原局部求解预算内仍未产生轨迹与平移命令：{velocity:?}");
}


#[test]
fn obstacle_adjacent_zigzag_over_capacity_shortens_local_target() {
    // 2米窗口最多32个控制点；每个5厘米转折都超过简化阈值，需保留的锚点超过10个。
    // 第8段含30厘米直段，可在末端参考速度下完成原动力学检查。
    let mut route = vec![[0., 0., 0.35], [0.1, 0., 0.35]];
    for i in 1..=60 {
        route.push([0.1 + i as f64 * 0.05 + if i >= 8 { 0.3 } else { 0. }, if i % 2 == 1 { 0.04 } else { 0. }, 0.35]);
    }
    // 绕行末段回到前段附近；缩短后的目标进入原全局目标制动区。
    route.push([0.625, 0.02, 0.35]);
    let obstacles: Vec<_> = (0..=320).flat_map(|x| {
        [-0.025, 0.065].map(move |y| [x as f64 * 0.01, y, 0.35])
    }).collect();
    let mut cfg = config();
    cfg.resolution = 0.01;
    cfg.body_length = 0.015;
    cfg.body_width = 0.015;
    cfg.max_velocity = 0.2;
    let mut map = MapOwner::new(&cfg).unwrap();
    map.update_surface(&obstacles, &[], &[], &[]).unwrap();
    let state = MotionState {
        position: route[0], timestamp_ns: 1_000_000_000,
        measured_velocity: 1, measured_yaw_rate: 1, ..MotionState::default()
    };
    let trajectory = ScanPlanner::default().plan(&cfg, &map.snapshot(), &route, &state, None, 0.)
        .expect("贴障参考锚点超过容量时应缩短本次目标并产出轨迹");
    let view = trajectory.inspect(0., 0.01);
    assert!(view.local_target[0] > 0.1 && view.local_target[0] < 1., "应交付窗口前段：{view:?}");
    let end = view.samples.last().unwrap();
    assert!((end.position[0] - view.local_target[0]).hypot(end.position[1] - view.local_target[1]) < 1e-6);
    // 原窗口末端远离全局目标、参考速度非零；缩短后必须采用原制动区的零速边界。
    assert!(end.velocity[0].hypot(end.velocity[1]) < 1e-8, "末端速度应按缩短位置的制动距离获取");
    for sample in &view.samples {
        let speed = sample.velocity.iter().map(|v| v*v).sum::<f64>().sqrt();
        let acceleration = sample.acceleration.iter().map(|v| v*v).sum::<f64>().sqrt();
        assert!(speed <= cfg.max_velocity + 1e-8);
        assert!(acceleration <= cfg.max_acceleration + 1e-8);
        assert!(sample.position[1] > -0.015 && sample.position[1] < 0.055);
    }
}
