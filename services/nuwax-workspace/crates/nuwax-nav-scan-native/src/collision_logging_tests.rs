#[test]
fn three_native_failures_log_the_actual_cells_once() {
    use std::io::{self, Write};
    use std::sync::Mutex;

    struct Capture(Arc<Mutex<Vec<u8>>>);
    impl Write for Capture {
        fn write(&mut self, data: &[u8]) -> io::Result<usize> {
            self.0.lock().unwrap().write(data)
        }
        fn flush(&mut self) -> io::Result<()> { Ok(()) }
    }
    let output = Arc::new(Mutex::new(Vec::new()));
    let writer = output.clone();
    let subscriber = tracing_subscriber::fmt().without_time().json()
        .with_writer(move || Capture(writer.clone())).finish();
    tracing::subscriber::with_default(subscriber, || {
        let c = config();
        let start = [0.05, 0.05, 0.35];
        let goal = [1.95, 0.05, 0.35];
        let upright = [0., 0., 0., 1.];
        let state = MotionState { position: start, measured_velocity: 1, measured_yaw_rate: 1,
            timestamp_ns: 1_000_000_000, ..Default::default() };

        // 机体当前未碰墙，真实前向速度使所有候选的制动扫掠都撞到概率占据体素。
        let mut map = MapOwner::new(&c).unwrap();
        let wall = |x| (-55..=55).map(move |y| [x, (y as f64 + 0.5) * 0.1, 0.35]).collect::<Vec<_>>();
        map.seed_occupied(&wall(0.55), start);
        map.update_surface(&[], &[], &[], &[]).unwrap();
        let snapshot = map.snapshot();
        let moving = MotionState { velocity: [0.6, 0., 0.], ..state };
        assert_eq!(snapshot.pose_clearance(start, 0.).occupied, 0);
        let mut tracker = ScanTracker::default();
        for _ in 0..2 {
            let tracked = tracker.track_terminal_position(&c, &snapshot, &moving, [0.6, 0., 0.],
                Some(0.), Some(goal), Some((goal, upright))).unwrap();
            assert!(tracked.no_candidate());
            assert_eq!(tracked.command, [0.; 3]);
        }

        // 起终点都空闲，表面墙截断整个局部地图，局部样条无法通过。
        let mut map = MapOwner::new(&c).unwrap();
        map.seed_occupied(&[], start);
        map.update_surface(&wall(0.95), &[], &[], &[]).unwrap();
        let snapshot = map.snapshot();
        assert_eq!(snapshot.pose_clearance(start, 0.).occupied, 0);
        assert_eq!(snapshot.pose_clearance(goal, 0.).occupied, 0);
        assert!(ScanPlanner::default().plan(&c, &snapshot, &[start, goal], &state, None, 0., None).is_err());

        // 已知地面贯通；禁行整列垂向无界，在全局搜索中把路线切断。
        use nuwax_nav_pct_native::{Band, Cell, Config as PctConfig, Planner as PctPlanner};
        let cells = (0..20).flat_map(|x| (-4..=4).map(move |y| Cell {
            x, y, ground: 0., ceiling: f64::INFINITY,
        })).collect::<Vec<_>>();
        let bands = (-4..=4).map(|y| Band {
            x: 8, y, low: f64::NEG_INFINITY, high: f64::INFINITY,
        }).collect::<Vec<_>>();
        let mut pct = PctPlanner::from_cells(PctConfig {
            resolution: 0.1, ..Default::default()
        }, &cells, &bands).unwrap();
        let reason = pct.plan(([0.35, 0.05, 0.4], upright), ([1.65, 0.05, 0.4], upright)).unwrap_err();
        assert_eq!(reason, "PCT未找到满足已知占据与机体尺寸的路径");
    });
    let data = output.lock().unwrap();
    let logs = std::str::from_utf8(&data).unwrap();
    let entries = logs.lines().map(|line| serde_json::from_str::<serde_json::Value>(line).unwrap())
        .collect::<Vec<_>>();
    assert_eq!(entries.len(), 3, "{logs}");
    for (entry, (stage, source)) in entries.iter().zip([
        ("局部命令检查", "占据体素"), ("局部轨迹求解", "表面接触"), ("PCT全局规划", "业务禁行"),
    ]) {
        let fields = &entry["fields"];
        assert_eq!(entry["level"], "INFO");
        assert_eq!(fields["stage"], stage);
        assert!(fields["ground_z_m"].as_f64().unwrap().abs() < 1e-9);
        let cells: Vec<serde_json::Value> = serde_json::from_str(fields["blocking_cells"].as_str().unwrap()).unwrap();
        assert!(!cells.is_empty() && cells.len() <= 10, "{entry}");
        assert!(cells.iter().any(|cell| cell["source"] == source), "{entry}");
    }
    eprint!("{logs}");
}
