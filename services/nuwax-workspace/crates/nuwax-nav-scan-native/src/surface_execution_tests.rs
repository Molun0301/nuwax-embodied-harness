// 融合层沿公开生产入口验证；注册帧只提供当前来源，不能替代融合障碍。
fn registered_fixture(req: &PlanRequest<'_>) -> Planner {
    let mut planner = Planner::new();
    planner
        .ingest_registered_frame(
            &req.config,
            &[[0., 0., -1.]],
            [0., 0., 1.],
            req.start_position,
            req.odometry_timestamp_ns,
        )
        .unwrap();
    planner
}

fn shallow_surface_request<'a>(
    route: &'a [[f64; 3]],
    obstacles: &'a [[f64; 3]],
) -> PlanRequest<'a> {
    let mut req = request(route, obstacles);
    req.config.resolution = 0.05;
    req.config.body_length = 0.2;
    req.config.body_width = 0.2;
    req.start_position = route[0];
    req
}

#[test]
fn registered_surface_overlap_escapes_across_updates_and_execution_retirement() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [-0.025, 0.025, 0.35];
    let updates: Vec<_> = (0..20)
        .map(|tick| {
            let mut obstacles = [contact, [2. + tick as f64 * 0.05, 2., 0.35]];
            if tick % 2 == 0 {
                obstacles.reverse();
            }
            obstacles
        })
        .collect();
    let mut req = shallow_surface_request(&route, &[]);
    let mut planner = registered_fixture(&req);
    let initial = req.start_position;
    for obstacles in &updates {
        // 无关远处几何每拍变化、输入顺序也变化，原接触体积和事件预算保持。
        req.obstacles = obstacles;
        req.odometry_timestamp_ns += 100_000_000;
        planner.notify_source_changed();
        planner.reset_execution();
        let out = planner
            .step(&req)
            .expect("浅融合接触必须允许完整证明后的有界退出");
        assert!(
            out.command_forward > 0. && out.command_lateral == 0. && out.command_yaw_rate == 0.
        );
        let old = req.start_velocity.unwrap()[0];
        assert!(
            (out.command_forward - old).abs() <= req.config.max_acceleration * req.config.sample_dt
        );
        req.start_position[0] += (old + out.command_forward) * 0.5 * req.config.sample_dt;
        req.start_velocity = Some([out.command_forward, 0., 0.]);
        assert!(req.start_position[0] - initial[0] <= req.config.resolution);
        if req.start_position[0] - req.config.body_length / 2. > 0. {
            return;
        }
        req.obstacles = &[];
    }
    panic!("原时间和距离预算内未离开初始融合接触");
}

#[test]
fn registered_surface_failed_initialization_does_not_poison_later_shallow_escape() {
    let route = [[0.195, 0.025, 0.35], [0.6, 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    req.config.body_length = 0.4;
    req.config.body_width = 0.4;
    let mut planner = registered_fixture(&req);
    req.start_position[0] = 0.04;
    let failure = planner.step(&req).unwrap_err();
    assert!(failure.contains("localization and mapping allowance"));
    assert!(failure.contains("BODY_OVERLAP_RECOVERY_FAILED:"));
    planner.reset_execution();
    req.start_position = route[0];
    let mut cleared = false;
    let mut normal_motion = false;
    for tick in 1..100 {
        req.odometry_timestamp_ns = tick * 50_000_000;
        let out = planner.step(&req).expect("未动作的初始化拒绝不能毒化后续浅接触");
        let old = req.start_velocity.unwrap();
        let before = req.start_position[0];
        req.start_position[0] += (old[0] + out.command_forward) * 0.025;
        req.start_position[1] += (old[1] + out.command_lateral) * 0.025;
        req.start_yaw += out.command_yaw_rate * 0.05;
        req.start_velocity = Some([out.command_forward, out.command_lateral, 0.]);
        if !cleared {
            assert!(req.start_position[0] > before);
            assert_eq!(out.command_lateral, 0.);
            assert_eq!(out.command_yaw_rate, 0.);
            assert!(req.start_position[0] - route[0][0] <= req.config.resolution);
            // 独立轴向矩形边界：退出前不能增加与初始格的相交。
            for sample in &out.samples {
                assert!(sample.position[0] >= before);
            }
            cleared = req.start_position[0] - req.config.body_length / 2. > 0.;
        } else if out.command_forward > 0. && planner.plan_count() > 0 {
            normal_motion = true;
            break;
        }
    }
    assert!(cleared && normal_motion, "退出初始格后必须继续正常规划和前进");
}

#[test]
fn registered_surface_failed_first_proof_can_retry_without_rearming_issued_budget() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    let mut planner = registered_fixture(&req);
    req.start_yaw_rate = None;
    let missing = planner.step(&req).unwrap_err();
    assert!(missing.contains("requires measured velocity and yaw rate"));
    assert!(!missing.contains("BODY_OVERLAP_RECOVERY_FAILED:"),
        "输入缺测仍可恢复，不能误标为脱困失败终态");
    req.start_yaw_rate = Some(0.);
    req.start_velocity = Some([-0.02, 0., 0.]);
    assert!(planner.step(&req).unwrap_err().contains("no monotone"));
    planner.reset_execution();
    req.start_velocity = Some([0.; 3]);
    req.odometry_timestamp_ns = 50_000_000;
    assert!(planner.step(&req).unwrap().command_forward > 0.);
    req.odometry_timestamp_ns = 2_050_000_001;
    let exhausted = planner.step(&req).unwrap_err();
    assert!(exhausted.contains("budget exhausted"));
    planner.reset_execution();
    req.odometry_timestamp_ns += 50_000_000;
    assert_eq!(planner.step(&req).unwrap_err(), exhausted,
        "发出动作后的耗尽首因必须跨目标保留，不能重置额度或被泛化");
    // 满足旧的两格净离开条件后才是新事件；新失败不能继承旧事件首因。
    req.start_position[0] = 0.3;
    planner.step(&req).unwrap();
    planner.reset_execution();
    req.start_position = route[0];
    req.odometry_timestamp_ns += 50_000_000;
    assert!(planner.step(&req).unwrap().command_forward > 0.);
    req.odometry_timestamp_ns -= 1;
    let next_failure = planner.step(&req).unwrap_err();
    assert!(next_failure.contains("source clock regressed"));
    assert!(!next_failure.contains("budget exhausted"));
}

#[test]
fn registered_surface_escape_rejects_new_band_deep_overlap_and_inward_motion() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    let mut planner = registered_fixture(&req);
    assert!(planner.step(&req).unwrap().command_forward > 0.);
    // 同列同体素但高度带不同，不能借旧身份穿过新实体。
    let new_band = [contact[0], [-0.025, 0.025, 0.36]];
    req.obstacles = &new_band;
    req.odometry_timestamp_ns += 100_000_000;
    assert!(planner.step(&req).unwrap_err().contains("no monotone"));
    req.obstacles = &contact;
    req.config.body_length = 0.4;
    req.config.body_width = 0.4;
    req.start_position[0] = 0.04;
    assert!(registered_fixture(&req)
        .step(&req)
        .unwrap_err()
        .contains("localization and mapping allowance"));
    req.config.body_length = 0.2;
    req.config.body_width = 0.2;
    req.start_position = route[0];
    req.start_velocity = Some([-0.02, 0., 0.]);
    assert!(registered_fixture(&req)
        .step(&req)
        .unwrap_err()
        .contains("no monotone"));
}

#[test]
fn registered_surface_escape_keeps_time_distance_and_geometry_generation_limits() {
    let route = [[0.095, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    let mut planner = registered_fixture(&req);
    assert!(planner.step(&req).is_ok());
    planner.reset_execution();
    req.odometry_timestamp_ns = 2_000_000_001;
    assert!(planner.step(&req).unwrap_err().contains("budget exhausted"));
    req.odometry_timestamp_ns = 0;
    let mut planner = registered_fixture(&req);
    let mut exhausted = false;
    for _ in 0..20 {
        planner.reset_execution();
        match planner.step(&req) {
            Ok(out) => assert!(out.command_forward > 0.),
            Err(_) => {
                exhausted = true;
                break;
            }
        }
    }
    assert!(exhausted, "重复来源时间和新目标不能反复续发逃逸命令");
    let mut planner = registered_fixture(&req);
    assert!(planner.step(&req).is_ok());
    planner.reset_execution();
    req.config.resolution = 0.1;
    assert!(planner.configure(&req.config).unwrap());
    assert!(planner
        .step(&req)
        .unwrap_err()
        .contains("registered source frame"));
}

#[test]
fn registered_surface_localization_scale_overlap_exits_with_monotone_measured_depth() {
    for penetration in [0.026, 0.04, 0.06, 0.08, 0.10] {
        let route = [[0.3 - penetration, 0.025, 0.35], [1., 0.025, 0.35]];
        let contact = [[-0.025, 0.025, 0.35]];
        let mut req = shallow_surface_request(&route, &contact);
        req.config.body_length = 0.6;
        req.config.body_width = 0.6;
        let mut planner = registered_fixture(&req);
        let mut clear = false;
        for tick in 0..100 {
            req.odometry_timestamp_ns = tick * 50_000_000;
            // 新目标不能重置初始接触和已发运动预算。
            planner.reset_execution();
            let before = req.start_position[0];
            let out = planner.step(&req).expect("定位误差量级的接触应逐拍低速退出");
            assert!(out.command_forward > 0. && out.command_forward <= 0.04);
            assert_eq!(out.command_lateral, 0.);
            assert_eq!(out.command_yaw_rate, 0.);
            for sample in &out.samples {
                // 独立矩形边界：原格右边为0，左机身边界持续右移。
                assert!(sample.position[0] >= before);
            }
            let actual = &out.samples[1];
            req.start_position = actual.position;
            req.start_velocity = Some(actual.velocity);
            assert!(req.start_position[0] > before);
            if req.start_position[0] > 0.3 {
                clear = true;
                break;
            }
        }
        assert!(clear, "穿透{penetration}米应完全退出初始占据");
    }
}

#[test]
fn registered_surface_measured_exit_progress_outlives_initial_escape_budget() {
    let route = [[0.2, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    req.config.body_length = 0.6;
    req.config.body_width = 0.6;
    req.config.sample_dt = 0.05;
    req.config.max_acceleration = 0.5;
    let mut planner = registered_fixture(&req);
    for tick in 0..200 {
        // 每次实际执行整拍及其制动，再提供新的静止实测；行程和进展均来自已证样本。
        req.odometry_timestamp_ns = tick * 100_000_000;
        planner.reset_execution();
        let before = req.start_position[0];
        let out = planner.step(&req).expect("持续实测退出不能耗尽最初的总时间或总位移预算");
        assert!(out.command_forward > 0. && out.command_forward <= 0.04);
        assert_eq!(out.command_lateral, 0.);
        assert_eq!(out.command_yaw_rate, 0.);
        let actual = out.samples.last().unwrap();
        req.start_position = actual.position;
        req.start_velocity = Some(actual.velocity);
        assert!(req.start_position[0] > before);
        assert_eq!(actual.velocity, [0.; 3]);
        if req.start_position[0] > 0.3 {
            assert!(req.odometry_timestamp_ns > 5_050_000_000);
            return;
        }
    }
    panic!("实测穿透持续减小时应最终完全退出");
}

#[test]
fn registered_surface_deeper_escape_stops_before_new_occupied_wall() {
    let route = [[0.22, 0.025, 0.35], [1., 0.025, 0.35]];
    let mut obstacles = vec![[-0.025, 0.025, 0.35]];
    for row in -6..=6 {
        obstacles.push([0.575, (row as f64 + 0.5) * 0.05, 0.35]);
    }
    let mut req = shallow_surface_request(&route, &obstacles);
    req.config.body_length = 0.6;
    req.config.body_width = 0.6;
    let mut planner = registered_fixture(&req);
    for tick in 0..100 {
        req.odometry_timestamp_ns = tick * 50_000_000;
        match planner.step(&req) {
            Ok(out) => {
                for sample in &out.samples {
                    // 新墙左面为0.55米，整拍及制动样本均不能触墙。
                    assert!(sample.position[0] + 0.3 < 0.55);
                }
                req.start_position = out.samples[1].position;
                req.start_velocity = Some(out.samples[1].velocity);
            }
            Err(error) => {
                assert!(error.contains("BODY_OVERLAP_RECOVERY_FAILED:"));
                assert!(error.contains("no monotone"));
                assert!(req.start_position[0] < 0.3, "退出通道受阻不能假报已脱困");
                return;
            }
        }
    }
    panic!("退出方向被新占据截断时必须报原脱困失败");
}

#[test]
fn registered_surface_overlap_above_three_voxels_is_rejected() {
    let route = [[0.149, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    req.config.body_length = 0.6;
    req.config.body_width = 0.6;
    let error = registered_fixture(&req).step(&req).unwrap_err();
    assert!(error.contains("BODY_OVERLAP_RECOVERY_FAILED:"));
    assert!(error.contains("penetration exceeds localization and mapping allowance"));
}

#[test]
fn registered_surface_deeper_escape_rejects_increased_measured_penetration() {
    let route = [[0.22, 0.025, 0.35], [1., 0.025, 0.35]];
    let contact = [[-0.025, 0.025, 0.35]];
    let mut req = shallow_surface_request(&route, &contact);
    req.config.body_length = 0.6;
    req.config.body_width = 0.6;
    let mut planner = registered_fixture(&req);
    assert!(planner.step(&req).unwrap().command_forward > 0.);
    req.odometry_timestamp_ns += 50_000_000;
    req.start_position[0] -= 0.001;
    let error = planner.step(&req).unwrap_err();
    assert!(error.contains("measured escape penetration increased"));
    planner.reset_execution();
    assert_eq!(planner.step(&req).unwrap_err(), error);
}

#[test]
fn registered_surface_keeps_subvoxel_reference_turn_before_planning() {
    // 现场同类形状：最后2.5厘米转折虽小，提前沿斜线接终点会使前伸外包碰到桌边。
    let route = [
        [-0.0181, 0.355, 0.02],
        [0.725, -0.375, 0.02],
        [0.725, -1.175, 0.02],
        [2.675, -1.175, 0.02],
        [2.75, -1.15, 0.02],
    ];
    let table = [[2.375, -0.925, -0.7]];
    let mut req = request(&route, &[]);
    req.ground = &table;
    req.start_position = route[0];
    req.config.resolution = 0.05;
    req.config.body_length = 0.48294712657127936;
    req.config.body_width = 0.4366338904117285;
    req.config.body_center_x = 0.21245294699457518;
    req.config.body_center_y = 0.0024584103419466874;
    req.config.body_to_ground = 0.9432044618093445;
    req.config.body_z_max = 0.3386525932121276;
    req.config.max_step_up = 0.;
    req.config.max_velocity = 0.2;
    req.config.max_acceleration = 0.5;
    req.config.planning_horizon = 5.;
    let mut planner = registered_fixture(&req);
    for tick in 0..20 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        planner.notify_source_changed();
        let out = planner.step(&req).unwrap();
        if out.samples.is_empty() {
            continue;
        }
        let mut heading = 0.;
        for sample in &out.samples {
            assert!(sample.velocity[0].hypot(sample.velocity[1]) <= 0.2 + 1e-8);
            assert!(
                sample.acceleration[0].hypot(sample.acceleration[1])
                    <= 0.5 + 1e-8
            );
            if sample.velocity[0].hypot(sample.velocity[1]) > 1e-8 {
                heading = sample.velocity[1].atan2(sample.velocity[0]);
            }
            let (c, s) = (heading.cos(), heading.sin());
            let dx = table[0][0] - sample.position[0] - c * req.config.body_center_x
                + s * req.config.body_center_y;
            let dy = table[0][1]
                - sample.position[1]
                - s * req.config.body_center_x
                - c * req.config.body_center_y;
            let hx = req.config.body_length / 2.;
            let hy = req.config.body_width / 2.;
            let r = req.config.resolution / 2.;
            let overlap = dx.abs() <= hx * c.abs() + hy * s.abs() + r
                && dy.abs() <= hx * s.abs() + hy * c.abs() + r
                && (c * dx + s * dy).abs() <= hx + r * (c.abs() + s.abs())
                && (-s * dx + c * dy).abs() <= hy + r * (c.abs() + s.abs());
            assert!(
                !overlap,
                "当前已知桌边不能因细小转折被抹去而相交：{sample:?}"
            );
        }
        let last = out.samples.last().unwrap().position;
        assert!((last[0] - route[4][0]).hypot(last[1] - route[4][1]) < 1e-8);
        assert!(out.command_forward.hypot(out.command_lateral) + out.command_yaw_rate.abs() > 0.);
        return;
    }
    panic!("可行的细小参考转折不能导致反复精化失败并保持零速");
}

#[test]
fn registered_future_slow_corner_does_not_borrow_current_body_heading() {
    let route = [
        [0., 0., 0.02],
        [0.725, 0., 0.02],
        [0.725, -1.175, 0.02],
        [-0.5, -1.175, 0.02],
    ];
    let table = [[0.975, -1.175, -0.7]];
    let mut req = request(&route, &[]);
    req.ground = &table;
    req.start_position = route[0];
    req.config.resolution = 0.05;
    req.config.body_length = 0.48294712657127936;
    req.config.body_width = 0.4366338904117285;
    req.config.body_center_x = 0.21245294699457518;
    req.config.body_center_y = 0.0024584103419466874;
    req.config.body_to_ground = 0.9432044618093445;
    req.config.body_z_max = 0.3386525932121276;
    req.config.max_step_up = 0.;
    req.config.max_velocity = 0.2;
    req.config.max_acceleration = 0.5;
    req.config.planning_horizon = 5.;
    let mut planner = registered_fixture(&req);
    let mut first = None;
    let mut moving = false;
    for tick in 0..20 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        planner.notify_source_changed();
        let result = planner.step(&req).unwrap();
        moving |= result.command_forward.hypot(result.command_lateral) > 0.;
        if planner.plan_count() > 0 {
            let count = *first.get_or_insert(planner.plan_count());
            assert!(
                planner.plan_count() <= count+1,
                "远处减速段不能拿当前机体朝向反复制造碰撞并重规划"
            );
        }
    }
    assert!(
        first.is_some() && moving,
        "当前起点与参考均可行时必须有可执行运动"
    );
}
