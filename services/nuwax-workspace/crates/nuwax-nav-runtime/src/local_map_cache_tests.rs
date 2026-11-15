//! 同一输入序列并排驱动原地图线程与分层地图线程。
use super::*;
use nuwax_nav_scan_native::{ScanTracker, TrajectorySample};
use std::time::Instant;

fn prepare(worker: &MapWorker, input: &MapInput) -> (PreparedMap, f64) {
    let began = Instant::now();
    worker.submit(input.clone());
    loop {
        if let Some(result) = worker.take() {
            return (result.unwrap(), began.elapsed().as_secs_f64() * 1000.);
        }
        assert!(began.elapsed().as_secs() < 3, "地图线程未交付");
        std::thread::yield_now();
    }
}

fn samples(curve: &Trajectory) -> Vec<Vec<u64>> {
    curve
        .samples(0., 0.05)
        .iter()
        .map(|s: &TrajectorySample| {
            [s.time]
                .into_iter()
                .chain(s.position)
                .chain(s.velocity)
                .chain(s.acceleration)
                .chain([s.yaw, s.yaw_rate])
                .map(f64::to_bits)
                .collect()
        })
        .collect()
}

fn stream(ticks: usize, curves: bool) -> (Vec<f64>, Vec<f64>) {
    let (_, mut base) = crate::native_test_helpers::fixture();
    let old = full_reference::baseline_worker(None).unwrap();
    let new = MapWorker::new(None).unwrap();
    let mut input = MapInput {
        epoch: 1,
        source: base.clone(),
        config: crate::native_test_helpers::fixture().0.scan,
        boundary: Arc::new(Vec::new()),
        pose: [0.55, 1.05, 0.5],
        yaw: 0.,
        frame: None,
        active: None,
        active_snapshot: None,
        phase: 0.,
    };
    let mut old_times = Vec::new();
    let mut new_times = Vec::new();
    let mut old_planner = ScanPlanner::default();
    let mut new_planner = ScanPlanner::default();
    let mut old_tracker = ScanTracker::default();
    let mut new_tracker = ScanTracker::default();
    let mut previous = [0.; 3];
    let mut successful = 0;
    let mut previous_snapshot: Option<CollisionSnapshot> = None;
    let mut reused = 0;
    for tick in 0..ticks {
        input.pose[0] = 0.55 + tick as f64 * 0.012;
        if curves && tick == 8 {
            input.boundary = Arc::new(vec![[-2., -2.], [8., -2.], [8., 8.], [-2., 8.]]);
        }
        if curves && tick == 12 {
            let mut mask = vec![false; 10000];
            mask[45 * 100 + 65] = true;
            base = Arc::new(
                base.with_virtual_obstacles([-2., -2.], 0.1, 100, 100, &mask)
                    .unwrap(),
            );
        }
        if curves && tick == 16 {
            base = crate::surface_follower::test_snapshot(0., false);
        }
        if tick % 2 == 0 {
            let mut frame = (*crate::registered_frame::test_frame(tick as i64 + 1)).clone();
            frame.identity.timestamp_ns = 1_000_000_000 + tick as i64 * 50_000_000;
            frame.sensor_origin_world = input.pose;
            frame.body_position_world = input.pose;
            frame.points_world = Arc::from([[input.pose[0], 1.05, 0.05], [4.5, 2.5, 0.6]]);
            frame.ground_point_indices = Arc::from([0]);
            input.source = Arc::new(
                base.with_recent_obstacle_points(
                    tick as u64 + 1,
                    frame.identity.timestamp_ns as u64,
                    100_000_000,
                    &[0],
                    frame
                        .points_world
                        .iter()
                        .copied()
                        .enumerate()
                        .map(|(i, p)| (i as u32, p)),
                )
                .unwrap(),
            );
            input.frame = Some(Arc::new(frame));
        }
        let (before, old_ms) = prepare(&old, &input);
        let (after, new_ms) = prepare(&new, &input);
        for dx in [-0.2, 0., 0.4, 1.] {
            for yaw in [-0.3, 0., 0.7] {
                let p = [input.pose[0] + dx, input.pose[1], input.pose[2]];
                let a = before.snapshot.pose_clearance(p, yaw);
                let b = after.snapshot.pose_clearance(p, yaw);
                assert_eq!(
                    (
                        a.occupied,
                        a.field_distance.to_bits(),
                        a.body_clearance.to_bits()
                    ),
                    (
                        b.occupied,
                        b.field_distance.to_bits(),
                        b.body_clearance.to_bits()
                    ),
                    "第{tick}拍"
                );
            }
        }
        if tick % 2 == 1 {
            old_times.push(old_ms);
            new_times.push(new_ms);
            reused += usize::from(
                previous_snapshot
                    .as_ref()
                    .is_some_and(|s| s.same(&after.snapshot)),
            );
        }
        previous_snapshot = Some(after.snapshot.clone());
        if curves {
            let state = MotionState {
                position: input.pose,
                yaw: input.yaw,
                timestamp_ns: 1_000_000_000 + tick as i64 * 50_000_000,
                measured_velocity: 1,
                measured_yaw_rate: 1,
                ..Default::default()
            };
            let route = [
                input.pose,
                [input.pose[0] + 1.1, input.pose[1], input.pose[2]],
            ];
            let a = old_planner.plan(
                &input.config,
                &before.snapshot,
                &route,
                &state,
                None,
                0.,
                None,
            );
            let b = new_planner.plan(
                &input.config,
                &after.snapshot,
                &route,
                &state,
                None,
                0.,
                None,
            );
            match (a, b) {
                (Ok(a), Ok(b)) => {
                    assert_eq!(samples(&a), samples(&b), "第{tick}拍曲线");
                    old_tracker.install(&a);
                    new_tracker.install(&b);
                    let a = old_tracker
                        .track_and_check(&input.config, &before.snapshot, &state, previous, None)
                        .unwrap();
                    let b = new_tracker
                        .track_and_check(&input.config, &after.snapshot, &state, previous, None)
                        .unwrap();
                    assert_eq!(a.command, b.command, "第{tick}拍命令");
                    assert_eq!(
                        (a.reason, a.phase, a.installed),
                        (b.reason, b.phase, b.installed)
                    );
                    previous = a.command;
                    successful += 1;
                }
                (Err(a), Err(b)) => assert_eq!(a, b),
                _ => panic!("第{tick}拍规划结果不一致"),
            }
        }
    }
    assert!(reused > 0, "同帧普通位姿必须复用原快照");
    if curves {
        assert_eq!(successful, ticks, "不能用成对规划失败替代曲线命令等价");
    }
    (old_times, new_times)
}

#[test]
fn pose_stream_reuses_snapshot_and_matches_old_queries_curves_and_commands() {
    stream(24, true);
}

#[test]
#[ignore = "显式运行局部地图逐位姿处理耗时"]
fn n2_local_map_pose_benchmark() {
    let (mut old, mut new) = stream(100, false);
    old.sort_by(f64::total_cmp);
    new.sort_by(f64::total_cmp);
    eprintln!("N2局部地图同帧位姿{}条：原版中位样本={:.6}ms，新版中位样本={:.6}ms，原版={old:?}，新版={new:?}",
        old.len(), old[old.len()/2], new[new.len()/2]);
}

#[test]
fn pose_ground_view_matches_full_map_and_keeps_published_view_immutable() {
    let ground = [[0.025, 0.325, 0.12], [0.025, 0.325, 0.04]];
    let mut input = ground_tests::input(ground_tests::source(&ground, &[[15, 15, 8]]));
    input.config.max_step_up = 0.2;
    let before = full_reference::baseline_worker(None).unwrap();
    let after = MapWorker::new(None).unwrap();
    let first = prepare(&after, &input).0.snapshot;
    let original = first.pose_clearance([0., 0.325, 0.4], 0.);
    for (position, yaw) in [
        ([0., 0., 0.4], 0.),
        ([0., 0., 0.4], 1.57),
        ([0., 0., 0.6], 1.57),
        ([0., -0.04, 0.4], 0.),
        ([0., -0.001, 0.4], 0.),
    ] {
        input.pose = position;
        input.yaw = yaw;
        let a = prepare(&before, &input).0;
        let b = prepare(&after, &input).0;
        for x in [-0.2, 0., 0.4] {
            for y in [0., 0.225, 0.325] {
                for z in [0.4, 0.6] {
                    let a = a.snapshot.pose_clearance([x, y, z], yaw);
                    let b = b.snapshot.pose_clearance([x, y, z], yaw);
                    assert_eq!(
                        (
                            a.occupied,
                            a.field_distance.to_bits(),
                            a.body_clearance.to_bits()
                        ),
                        (
                            b.occupied,
                            b.field_distance.to_bits(),
                            b.body_clearance.to_bits()
                        )
                    );
                }
            }
        }
    }
    let retained = first.pose_clearance([0., 0.325, 0.4], 0.);
    assert_eq!(
        (original.occupied, original.body_clearance),
        (retained.occupied, retained.body_clearance)
    );
}
