#[test]
fn terminal_heading_uses_the_existing_proportional_gain_while_turning_or_translating() {
    let mut c = config();
    c.max_yaw_acceleration = 20.;
    let state = MotionState {position:[0.,0.,0.4],measured_velocity:1,
        measured_yaw_rate:1,timestamp_ns:1_000_000_000,..Default::default()};
    let mut map = MapOwner::new(&c).unwrap();
    map.seed_occupied(&[],state.position);
    map.update_surface(&[],&[],&[],&[]).unwrap();
    let snapshot = map.snapshot();
    for position in [None, Some([0.1,0.,0.4])] {
        for error in [-0.04, 0.04] {
            let tracked = ScanTracker::default().track_terminal_position(
                &c,&snapshot,&state,[0.;3],Some(error),position).unwrap();
            assert!(!tracked.invalid(),"空图末段应有合法候选：{tracked:?}");
            assert!((tracked.command[2]-1.5*error).abs()<1e-12,
                "小误差采用现有kp_yaw=1.5，避免偏航加速度平方根给出过大期望：{tracked:?}");
        }
    }
}

#[test]
fn pillar_stop_uses_the_configured_clearance_without_a_turn_point() {
    let mut c = config();
    c.resolution = 0.02;
    c.body_length = 0.6;
    c.body_width = 0.32;
    c.body_to_ground = 0.4;
    c.max_reverse_velocity = 0.;
    let goal = [0., 0., 0.4];
    let yaw = std::f64::consts::FRAC_PI_2;
    let column: Vec<_> = (11..17)
        .flat_map(|x| {
            (-5..5).flat_map(move |y| {
                (0..40).map(move |z| {
                    [
                        (x as f64 + 0.5) * 0.02,
                        (y as f64 + 0.5) * 0.02,
                        (z as f64 + 0.5) * 0.02,
                    ]
                })
            })
        })
        .collect();
    let mut map = MapOwner::new(&c).unwrap();
    map.seed_occupied(&column, goal);
    map.update_surface(&[], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let selected = snapshot.terminal_points(&c, goal, yaw, 0.15);
    assert!(selected.stop[0].hypot(selected.stop[1]) <= 0.12);
    assert_eq!(selected.stop[2], goal[2]);
    let stop = snapshot.pose_clearance(selected.stop, yaw);
    assert_eq!(stop.occupied, 0);
    assert!(stop.body_clearance >= c.obstacle_clearance_m, "{selected:?} {stop:?}");
    c.max_lateral_velocity = 0.;
    assert_eq!(snapshot.terminal_points(&c,goal,yaw,0.15),selected);
}
