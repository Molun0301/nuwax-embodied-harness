#[test]
fn g1_door_clearance_uses_body_envelope_and_keeps_narrow_connectivity() {
    // 整片地面均为实测，墙体只通过实体区间进入；不能靠缺地面代价间接避墙。
    for (length, width) in [(0.291, 0.557), (0.483, 0.437)] {
        for gap in [0.8, 0.6, 0.65] {
            let mut cfg = config();
            cfg.resolution = 0.05;
            cfg.footprint = vec![
                [-length / 2., -width / 2.],
                [length / 2., -width / 2.],
                [length / 2., width / 2.],
                [-length / 2., width / 2.],
            ];
            let cells: Vec<_> = (-20..81)
                .flat_map(|x| {
                    (-40..41).map(move |y| Cell {
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
                        .filter(move |&y| y < 0 || y >= (gap / 0.05_f64).round() as i32)
                        .map(move |y| Band {
                            x,
                            y,
                            low: 0.,
                            high: 1.5,
                        })
                })
                .collect();
            let mut planner = Planner::from_cells(cfg, &cells, &bands).unwrap();
            let result = planner.plan([0., 0.1, 0.4], [3., 0.1, 0.4]);
            if gap == 0.6 && width == 0.557 {
                // 可用中心为[0.2785,0.3215]，5厘米格中心仅有0.275/0.325，均硬碰撞。
                assert!(result
                    .unwrap_err()
                    .contains("PCT未找到满足已知占据与机体尺寸的路径"));
                continue;
            }
            let route = result.unwrap();
            let crossing: Vec<_> = route
                .path
                .iter()
                .filter(|p| p[0] >= 1.45 && p[0] <= 1.65)
                .collect();
            assert!(!crossing.is_empty());
            assert!(
                crossing
                    .iter()
                    .all(|p| (p[1] - gap / 2.).abs() <= 0.025 + 1e-8),
                "门宽{gap}、机体{length}×{width}必须选中线附近：{crossing:?}"
            );
        }
    }
}

#[test]
fn thin_door_clearance_keeps_the_straight_center_route_without_heading_loops() {
    let q = f64::from(0.01_f32);
    let width = 32. * q;
    let middle = 33. * q / 2.;
    let cfg = Config {
        resolution: q,
        point_resolution: q,
        body_to_ground: 0.4,
        robot_height: 0.1,
        body_z_min: -0.15,
        body_z_max: 0.15,
        max_step: 0.,
        max_step_down: 0.,
        footprint: vec![
            [-0.3, -width / 2.],
            [0.3, -width / 2.],
            [0.3, width / 2.],
            [-0.3, width / 2.],
        ],
        ..Default::default()
    };
    let cells: Vec<_> = (-100..100)
        .flat_map(|x| {
            (-50..100).map(move |y| Cell {
                x,
                y,
                ground: -q / 2.,
                ceiling: f64::INFINITY,
            })
        })
        .collect();
    let bands: Vec<_> = (0..4)
        .flat_map(|x| {
            (-50..100).filter(|&y| y < 0 || y >= 33).map(move |y| Band {
                x,
                y,
                low: 0.,
                high: 80. * q,
            })
        })
        .collect();
    let mut planner = Planner::from_cells(cfg, &cells, &bands).unwrap();
    let route = planner
        .plan([-0.5, middle, 0.4], [0.5, middle, 0.4])
        .unwrap();
    assert!(
        route
            .path
            .iter()
            .all(|p| (p[1] - middle).abs() <= q / 2. + 1e-8),
        "全姿态软余量不得奖励门前门后的朝向回环：{:?}",
        route.path
    );
    let length: f64 = route
        .path
        .windows(2)
        .map(|e| (e[1][0] - e[0][0]).hypot(e[1][1] - e[0][1]))
        .sum();
    assert!(
        length <= 1. + 2. * q,
        "保留端点格量化误差后仍应是直接中线：{length}"
    );
}
