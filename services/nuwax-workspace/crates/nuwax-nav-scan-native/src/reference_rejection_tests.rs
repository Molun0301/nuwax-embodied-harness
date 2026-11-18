#[test]
fn n4_rejected_reference_preserves_its_first_native_reason_without_retrying() {
    let c = config();
    let state = MotionState { position: [0., 0., 0.35], timestamp_ns: 1_000_000_000,
        measured_velocity: 1, measured_yaw_rate: 1, ..Default::default() };
    let route = [state.position, [1., 0., 0.35]];
    let mut map = MapOwner::new(&c).unwrap();
    map.seed_occupied(&[], state.position);
    map.update_surface(&[[1.025, 0.025, 0.35]], &[], &[], &[]).unwrap();
    let snapshot = map.snapshot();
    let mut planner = ScanPlanner::default();
    let first = planner.plan(&c, &snapshot, &route, &state, None, 0., None)
        .err().expect("终点占据必须保留原生拒绝");
    assert!(first.contains("Original final target is occupied"), "{first}");
    for tick in 1..=3 {
        let next = MotionState { timestamp_ns: state.timestamp_ns + tick * 50_000_000, ..state };
        let error = planner.plan(&c, &snapshot, &route, &next, None, 0., None).err().unwrap();
        assert_eq!(error, format!("原参考路线未被接纳：{first}"),
            "相同参考保持未接纳，附上首次原因而非重新接纳");
    }
    let changed = [state.position, [0.5, 0., 0.35]];
    if let Err(error) = planner.plan(&c, &snapshot, &changed, &state, None, 0., None) {
        assert!(!error.contains("原参考路线未被接纳") && !error.contains(&first),
            "新参考不带旧参考的拒绝原因：{error}");
    }
}
