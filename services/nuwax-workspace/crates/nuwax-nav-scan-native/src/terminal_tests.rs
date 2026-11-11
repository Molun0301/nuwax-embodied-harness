#[test]
fn pillar_terminal_points_leave_turning_room_and_require_the_actual_lateral_channel() {
    let mut c = config();
    c.resolution = 0.02;
    c.body_length = 0.6;
    c.body_width = 0.32;
    c.body_to_ground = 0.4;
    c.max_reverse_velocity = 0.;
    let goal = [0., 0., 0.4];
    let yaw = std::f64::consts::FRAC_PI_2;
    let route = [[-1., 0., 0.4], goal];
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
    let selected = snapshot.terminal_points(&c, goal, yaw, 0.15, &route);
    assert!(selected.stop[0].hypot(selected.stop[1]) <= 0.12);
    assert_eq!(selected.stop[2], goal[2]);
    let stop = snapshot.pose_clearance(selected.stop, yaw);
    assert_eq!(stop.occupied, 0);
    assert!(stop.body_clearance >= 0.10, "{selected:?} {stop:?}");
    let radius = (c.body_length / 2. + c.body_center_x.abs())
        .hypot(c.body_width / 2. + c.body_center_y.abs());
    assert!(
        stop.field_distance < radius + 0.05,
        "这个停点必须确实转不开"
    );
    assert_ne!(selected.turn, selected.stop);
    assert!(snapshot.pose_clearance(selected.turn, yaw).field_distance >= radius + 0.05);
    assert!((0.0..=1.5).contains(&(1. - selected.route_arc)));
    let length = (selected.stop[0] - selected.turn[0]).hypot(selected.stop[1] - selected.turn[1]);
    for k in 0..=(length / (c.resolution / 2.)).ceil() as usize {
        let fraction = ((k as f64 * c.resolution / 2.) / length).min(1.);
        let p = std::array::from_fn(|i| {
            selected.turn[i] + fraction * (selected.stop[i] - selected.turn[i])
        });
        assert_eq!(
            snapshot.pose_clearance(p, yaw).occupied,
            0,
            "固定目标朝向的整条进点直线必须可放下"
        );
    }
    let dx = selected.stop[0] - selected.turn[0];
    let dy = selected.stop[1] - selected.turn[1];
    assert!((-yaw.sin() * dx + yaw.cos() * dy).abs() > 1e-8);
    assert!(yaw.cos() * dx + yaw.sin() * dy >= -1e-8);
    c.max_lateral_velocity = 0.;
    let without_lateral = snapshot.terminal_points(&c, goal, yaw, 0.15, &route);
    assert_eq!(without_lateral.stop, selected.stop);
    assert_eq!(without_lateral.turn, without_lateral.stop);
}
