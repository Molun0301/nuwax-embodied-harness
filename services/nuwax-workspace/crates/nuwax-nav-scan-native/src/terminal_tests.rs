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
    assert!(selected.stop[0].hypot(selected.stop[1]) <= 0.075);
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
    assert_eq!(without_lateral.stop, goal);
    assert_eq!(without_lateral.turn, goal);
}

#[test]
fn pillar_terminal_point_projects_away_a_small_reverse_component() {
    let mut c = config();
    c.resolution = 0.02;
    c.body_length = 0.6;
    c.body_width = 0.32;
    c.body_to_ground = 0.4;
    c.max_reverse_velocity = 0.;
    let goal = [0., 0., 0.4];
    let yaw = 96_f64.to_radians();
    let route = [[-1., 0., 0.4], goal];
    let column: Vec<_> = (11..17).flat_map(|x| (-5..5).flat_map(move |y|
        (0..40).map(move |z| [(x as f64+0.5)*0.02,(y as f64+0.5)*0.02,(z as f64+0.5)*0.02]))).collect();
    let mut map = MapOwner::new(&c).unwrap();
    map.seed_occupied(&column, goal);
    map.update_surface(&[], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let points = snapshot.terminal_points(&c, goal, yaw, 0.15, &route);
    let q = points.route_point;
    let original_forward = yaw.cos()*(points.stop[0]-q[0])+yaw.sin()*(points.stop[1]-q[1]);
    assert!(original_forward < 0. && original_forward.abs() < 0.05, "{points:?}");
    assert_ne!(points.turn, q);
    assert!((q[0]-(-1.+points.route_arc)).abs()<1e-12 && q[1]==0.);
    assert!((points.turn[0]-q[0]-original_forward*yaw.cos()).abs()<1e-12);
    assert!((points.turn[1]-q[1]-original_forward*yaw.sin()).abs()<1e-12);
    let remaining_forward = yaw.cos()*(points.stop[0]-points.turn[0])+yaw.sin()*(points.stop[1]-points.turn[1]);
    assert!(remaining_forward.abs()<1e-12, "投影后不能再需要倒退：{remaining_forward} {points:?}");
    let radius = (c.body_length/2.+c.body_center_x.abs()).hypot(c.body_width/2.+c.body_center_y.abs());
    assert!(snapshot.pose_clearance(points.turn,yaw).field_distance >= radius+0.05);
    let length=(0..3).map(|i|(points.stop[i]-points.turn[i]).powi(2)).sum::<f64>().sqrt();
    for k in 0..=(length/(c.resolution/2.)).ceil() as usize {
        let t=((k as f64*c.resolution/2.)/length).min(1.);
        let point=std::array::from_fn(|i|points.turn[i]+t*(points.stop[i]-points.turn[i]));
        assert_eq!(snapshot.pose_clearance(point,yaw).occupied,0);
    }
}
