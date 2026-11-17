// 独立几何判据：旋转矩形与门框矩形的欧氏最短距离，不调用规划器碰撞接口。
fn doorway_body_distance(position: [f64; 3], yaw: f64, cfg: &Config, gap: f64) -> f64 {
    body_distance_to_door(position, yaw, cfg, [1.5, 1.6], [[-2., 0.], [gap, 2.]])
}

fn body_distance_to_door(position: [f64; 3], yaw: f64, cfg: &Config,
    wall_x: [f64; 2], bands: [[f64; 2]; 2]) -> f64 {
    let (c, s) = (yaw.cos(), yaw.sin());
    let body: Vec<_> = [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
        .into_iter()
        .map(|(a, b)| {
            let x = cfg.body_center_x + a * cfg.body_length / 2.;
            let y = cfg.body_center_y + b * cfg.body_width / 2.;
            [position[0] + c * x - s * y, position[1] + s * x + c * y]
        })
        .collect();
    fn point_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
        let v = [b[0] - a[0], b[1] - a[1]];
        let t = (((p[0] - a[0]) * v[0] + (p[1] - a[1]) * v[1]) / (v[0] * v[0] + v[1] * v[1]))
            .clamp(0., 1.);
        (p[0] - a[0] - t * v[0]).hypot(p[1] - a[1] - t * v[1])
    }
    let mut distance = f64::INFINITY;
    for [low, high] in bands {
        let wall = [[wall_x[0], low], [wall_x[1], low], [wall_x[1], high], [wall_x[0], high]];
        let separated = [[1., 0.], [0., 1.], [c, s], [-s, c]]
            .into_iter()
            .any(|axis| {
                let project = |points: &[[f64; 2]]| {
                    points
                        .iter()
                        .map(|p| p[0] * axis[0] + p[1] * axis[1])
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                            (lo.min(v), hi.max(v))
                        })
                };
                let (a, b) = project(&body);
                let (d, e) = project(&wall);
                b < d || e < a
            });
        if !separated {
            return 0.;
        }
        for i in 0..4 {
            for j in 0..4 {
                distance = distance.min(point_segment(body[i], wall[j], wall[(j + 1) % 4]));
                distance = distance.min(point_segment(wall[i], body[j], body[(j + 1) % 4]));
            }
        }
    }
    distance
}

fn g1_doorway_clearance_case(length: f64, width: f64, gap: f64, oblique: bool) {
    let q = 0.05;
    let start = [0., if oblique { -0.35 } else { 0.1 }, 0.35];
    let goal = [3., 0.1, 0.35];
    let cells: Vec<_> = (-20..81)
        .flat_map(|x| {
            (-40..41).map(move |y| nuwax_nav_pct_native::Cell {
                x,
                y,
                ground: 0.,
                ceiling: f64::INFINITY,
            })
        })
        .collect();
    let bands: Vec<_> = (30..32)
        .flat_map(|x| {
            (-40..40)
                .filter(move |&y| y < 0 || y >= (gap / q).round() as i32)
                .map(move |y| nuwax_nav_pct_native::Band {
                    x,
                    y,
                    low: 0.,
                    high: 1.5,
                })
        })
        .collect();
    let pct_cfg = nuwax_nav_pct_native::Config {
        resolution: q,
        point_resolution: q,
        body_to_ground: 0.35,
        footprint: vec![
            [-length / 2., -width / 2.],
            [length / 2., -width / 2.],
            [length / 2., width / 2.],
            [-length / 2., width / 2.],
        ],
        ..Default::default()
    };
    let mut global = nuwax_nav_pct_native::Planner::from_cells(pct_cfg, &cells, &bands).unwrap();
    let result = global.plan(start, goal);
    if gap == 0.6 {
        assert!(result
            .unwrap_err()
            .contains("PCT未找到满足已知占据与机体尺寸的路径"));
        eprintln!("0.6米门：可用中心[0.2785,0.3215]不含5厘米格中心，原硬判定拒绝");
        return;
    }
    let route = result.unwrap().path;
    assert!(
        route
            .iter()
            .filter(|p| p[0] >= 1.45 && p[0] <= 1.65)
            .all(|p| (p[1] - gap / 2.).abs() <= q / 2. + 1e-8),
        "全局必须走门洞中线：{route:?}"
    );
    let obstacles: Vec<_> = bands
        .iter()
        .map(|b| [(b.x as f64 + 0.5) * q, (b.y as f64 + 0.5) * q, 0.35])
        .collect();
    let mut req = request(&route, &obstacles);
    req.config.resolution = q;
    req.config.body_length = length;
    req.config.body_width = width;
    req.config.max_velocity = 0.3;
    req.config.max_acceleration = 0.5;
    req.config.planning_horizon = 5.;
    req.config.sample_dt = 0.1;
    req.start_position = start;
    // 当前融合层独立提供门框，注册雷达帧不含门框，覆盖实际生产的两层查询。
    let mut planner = registered_fixture(&req);
    let mut minimum = f64::INFINITY;
    let mut passed_opening = false;
    for tick in 0..600 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let out = planner.step(&req).unwrap_or_else(|e| {
            panic!(
                "第{tick}拍 {:?} yaw={}：{e}",
                req.start_position, req.start_yaw
            )
        });
        let yaw = req.start_yaw;
        let old = req.start_velocity.unwrap();
        let previous = [
            yaw.cos() * old[0] + yaw.sin() * old[1],
            -yaw.sin() * old[0] + yaw.cos() * old[1],
        ];
        let next = [out.command_forward, out.command_lateral];
        let dt = out.command_duration;
        assert!(
            (next[0] - previous[0]).hypot(next[1] - previous[1])
                <= req.config.max_acceleration * dt + 1e-8,
            "tick={tick},pose={:?},old={previous:?},new={next:?},phase={},plans={}",req.start_position,planner.phase,planner.plan_count()
        );
        assert!(
            (out.command_yaw_rate - req.start_yaw_rate.unwrap()).abs()
                <= req.config.max_yaw_acceleration * dt + 1e-8
        );
        let v = [(previous[0] + next[0]) / 2., (previous[1] + next[1]) / 2.];
        let w = (req.start_yaw_rate.unwrap() + out.command_yaw_rate) / 2.;
        // 每条实际命令分为10段独立积分并检查机体余量，不能只验样条或离散控制拍。
        for _ in 0..10 {
            let step = dt / 10.;
            let a = req.start_yaw;
            let (u, z) = if w.abs() > 1e-9 {
                ((w * step).sin() / w, (1. - (w * step).cos()) / w)
            } else {
                (step, 0.)
            };
            let dx = u * v[0] - z * v[1];
            let dy = z * v[0] + u * v[1];
            req.start_position[0] += a.cos() * dx - a.sin() * dy;
            req.start_position[1] += a.sin() * dx + a.cos() * dy;
            req.start_yaw += w * step;
            let d = doorway_body_distance(req.start_position, req.start_yaw, &req.config, gap);
            minimum = minimum.min(d);
            if req.start_position[0] >= 1.5 && req.start_position[0] <= 1.6 {
                assert!(
                    req.start_position[1] > 0. && req.start_position[1] < gap,
                    "实际命令必须穿过原门洞：{:?}",
                    req.start_position
                );
                passed_opening = true;
            }
            assert!(d > 0., "第{tick}拍实际机体相交：{:?}", req.start_position);
            if gap >= 0.8 {
                assert!(
                    d >= q,
                    "第{tick}拍可行宽门余量不足：{d}，{:?}",
                    req.start_position
                );
            }
        }
        req.start_velocity = Some([
            next[0] * req.start_yaw.cos() - next[1] * req.start_yaw.sin(),
            next[0] * req.start_yaw.sin() + next[1] * req.start_yaw.cos(),
            0.,
        ]);
        req.start_yaw_rate = Some(out.command_yaw_rate);
        req.start_acceleration = None;
        if (req.start_position[0] - goal[0]).hypot(req.start_position[1] - goal[1]) < 0.15 {
            assert!(passed_opening, "到达必须经过门洞");
            eprintln!("门宽{gap}，机体{length}×{width}，斜进{oblique}：{tick}拍到达，最小余量{minimum:.6}米");
            return;
        }
    }
    panic!(
        "原600拍内没有到达：{:?}，最小余量{minimum}",
        req.start_position
    );
}

#[test]
fn g1_doorway_clearance_both_postures_reach_with_one_voxel_margin() {
    for (length, width) in [(0.291, 0.557), (0.483, 0.437)] {
        g1_doorway_clearance_case(length, width, 0.8, false);
    }
}

#[test]
fn g1_doorway_clearance_oblique_approach_does_not_cut_the_jamb() {
    for (length, width) in [(0.291, 0.557), (0.483, 0.437)] {
        g1_doorway_clearance_case(length, width, 0.8, true);
    }
}

#[test]
fn g1_doorway_clearance_narrow_passage_keeps_soft_margin_traversable() {
    g1_doorway_clearance_case(0.291, 0.557, 0.6, false);
    g1_doorway_clearance_case(0.291, 0.557, 0.65, false);
}

fn check_off_center_reference_clearance(route: &[[f64; 3]]) {
    // 原直线外包距下门框仅1.15厘米，硬检查可行；局部软项必须主动离开它。
    let obstacles: Vec<_> = (30..32)
        .flat_map(|x| {
            (-40..40)
                .filter(|&y| y < 0 || y >= 16)
                .map(move |y| [(x as f64 + 0.5) * 0.05, (y as f64 + 0.5) * 0.05, 0.35])
        })
        .collect();
    let mut req = request(route, &obstacles);
    req.start_position = route[0];
    req.config.resolution = 0.05;
    req.config.body_length = 0.291;
    req.config.body_width = 0.557;
    req.config.max_velocity = 0.3;
    req.config.max_acceleration = 0.5;
    req.config.planning_horizon = 5.;
    let mut planner = registered_fixture(&req);
    for tick in 0..20 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let out = planner.step(&req).unwrap();
        if out.samples.is_empty() {
            continue;
        }
        let crossing: Vec<_> = out
            .samples
            .iter()
            .filter(|s| s.position[0] >= 1.4 && s.position[0] <= 1.7)
            .collect();
        assert!(!crossing.is_empty(), "轨迹必须实际经过门框区间");
        for sample in crossing {
            let clearance = doorway_body_distance(sample.position, sample.yaw, &req.config, 0.8);
            assert!(
                clearance >= 0.05,
                "未相交参考也应主动保留体素余量：{clearance}，{sample:?}"
            );
        }
        return;
    }
    panic!("原20拍内没有生成经过门洞的轨迹");
}

#[test]
fn g1_doorway_clearance_repels_an_uncolliding_off_center_reference() {
    check_off_center_reference_clearance(&[[0., 0.29, 0.35], [3., 0.29, 0.35]]);
    // 后半段绕行触发原参考直交分支，前半段贴门框同样必须主动留余量。
    check_off_center_reference_clearance(&[[0., 0.29, 0.35], [2.4, 0.29, 0.35], [2.4, 1., 0.35]]);
}
