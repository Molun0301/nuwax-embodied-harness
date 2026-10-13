// Worker exchange probes; these synthetic planners do not grant geometry permission.
use std::sync::atomic::{AtomicUsize, Ordering};

struct SnapshotProbe {
    maps: Arc<AtomicUsize>,
    boundaries: Arc<AtomicUsize>,
    steps: Arc<AtomicUsize>,
    boundary: Vec<[f64; 2]>,
    expected_boundary: Vec<[f64; 2]>,
}
impl NavPlanner for SnapshotProbe {
    fn replace_path(&mut self, _: Path2D) {}
    fn update_odometry(&mut self, _: PoseStamped) {}
    fn update_costmap(&mut self, _: CostMap2D) {
        self.maps.fetch_add(1, Ordering::SeqCst);
    }
    fn configure_boundary(&mut self, boundary: &[[f64; 2]]) {
        self.boundary = boundary.to_vec();
        self.boundaries.fetch_add(1, Ordering::SeqCst);
    }
    fn step(&mut self, _: f64) -> PlannerStep {
        assert_eq!(self.boundary, self.expected_boundary);
        self.steps.fetch_add(1, Ordering::SeqCst);
        PlannerStep::Command(Twist2 {
            linear: nuwax_nav_types_messages::Point2::new(0.2, 0.0),
            angular_z: 0.0,
        })
    }
    fn stop(&mut self) {
        self.boundary.clear();
    }
}
fn snapshot_probe(side: usize) -> (PlannerWorker, [Arc<AtomicUsize>; 3]) {
    let counts = std::array::from_fn(|_| Arc::new(AtomicUsize::new(0)));
    let boundary = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]];
    let mut worker = PlannerWorker::new(SnapshotProbe {
        maps: counts[0].clone(),
        boundaries: counts[1].clone(),
        steps: counts[2].clone(),
        boundary: Vec::new(),
        expected_boundary: boundary.clone(),
    })
    .unwrap();
    worker.configure_boundary(&boundary);
    worker.update_costmap(CostMap2D::new_all_free("map", side, side, 0.1));
    (worker, counts)
}
fn consume_snapshot_command(worker: &mut PlannerWorker, waits: &mut Vec<u128>) {
    match worker.step(0.3) {
        PlannerStep::Command(c) => {
            assert_eq!(c.linear.x, 0.2);
            return;
        }
        PlannerStep::Pending(_) => {}
        other => panic!("unexpected worker result: {other:?}"),
    }
    let deadline = std::time::Instant::now() + Duration::from_secs(3);
    loop {
        let start = std::time::Instant::now();
        let input = worker.shared.0.lock().unwrap();
        let waited = start.elapsed().as_nanos();
        let ready = input.result.is_some();
        drop(input);
        waits.push(waited);
        if ready {
            break;
        }
        assert!(std::time::Instant::now() < deadline);
        std::thread::yield_now();
    }
    assert!(matches!(worker.step(0.3), PlannerStep::Command(c) if c.linear.x == 0.2));
}
#[test]
fn worker_snapshot_installs_map_once_and_reinstalls_boundary_after_goal_reset() {
    let (mut worker, counts) = snapshot_probe(8);
    let mut waits = Vec::new();
    for _ in 0..4 {
        consume_snapshot_command(&mut worker, &mut waits);
    }
    assert_eq!(counts[0].load(Ordering::SeqCst), 1);
    let boundary_count = counts[1].load(Ordering::SeqCst);
    worker
        .configure_goal(&route_pose(0.5, 1), Default::default())
        .unwrap();
    consume_snapshot_command(&mut worker, &mut waits);
    assert_eq!(counts[0].load(Ordering::SeqCst), 1);
    assert_eq!(counts[1].load(Ordering::SeqCst), boundary_count + 1);
    worker.update_odometry(route_pose(0.6, 2));
    worker.update_costmap(CostMap2D::new_all_free("map", 8, 8, 0.1));
    wait_for_source_result(&worker);
    assert!(matches!(worker.step(0.3), PlannerStep::Command(c) if c.linear.x == 0.2));
    assert_eq!(counts[0].load(Ordering::SeqCst), 2);
    assert_eq!(counts[2].load(Ordering::SeqCst), 6);
}
#[test]
#[ignore = "synthetic worker snapshot cost; run isolated release processes"]
fn worker_unchanged_planar_snapshot_cost() {
    const SIDE: usize = 2048;
    const ROUNDS: usize = 256;
    let (mut worker, counts) = snapshot_probe(SIDE);
    let mut waits = Vec::with_capacity(100_000);
    consume_snapshot_command(&mut worker, &mut waits);
    waits.clear();
    let start = std::time::Instant::now();
    for _ in 0..ROUNDS {
        consume_snapshot_command(&mut worker, &mut waits);
    }
    let elapsed = start.elapsed();
    assert_eq!(counts[0].load(Ordering::SeqCst), 1);
    assert_eq!(counts[2].load(Ordering::SeqCst), ROUNDS + 1);
    waits.sort_unstable();
    eprintln!("WORKER_SNAPSHOT side={SIDE} rounds={ROUNDS} elapsed_ms={:.3} map_installs={} steps={} lock_samples={} lock_wait_p95_us={:.3} lock_wait_max_us={:.3}",
        elapsed.as_secs_f64()*1000.0, counts[0].load(Ordering::SeqCst), counts[2].load(Ordering::SeqCst), waits.len(),
        waits[waits.len()*95/100] as f64/1000.0, *waits.last().unwrap() as f64/1000.0);
}
