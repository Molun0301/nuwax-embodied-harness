// Real planner owner + scheduler + memq wiring. The build boundary is held or
// fails deterministically; FineMapReader's typed transfer transaction is tested
// separately with real immutable roots and blocks in surface_chunks.
use super::*;
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};

struct Probe {
    failing_revision: AtomicI64,
    invalid: AtomicBool,
    hold: AtomicBool,
    recover: AtomicBool,
    attempts: Mutex<Vec<(i64, Instant)>>,
}
impl Probe {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            failing_revision: AtomicI64::new(-1),
            invalid: AtomicBool::new(false),
            hold: AtomicBool::new(false),
            recover: AtomicBool::new(false),
            attempts: Mutex::new(Vec::new()),
        })
    }
    fn computer(self: &Arc<Self>) -> SurfaceMapComputer {
        let probe = Arc::clone(self);
        Arc::new(move |message, received_at, config, cache| {
            probe
                .attempts
                .lock()
                .unwrap()
                .push((message.cloud_revision, received_at));
            if message.cloud_revision == probe.failing_revision.load(Ordering::Acquire)
                && !probe.recover.load(Ordering::Acquire)
            {
                let deadline = Instant::now() + Duration::from_secs(5);
                while probe.hold.load(Ordering::Acquire) {
                    assert!(
                        Instant::now() < deadline,
                        "map build return barrier expired"
                    );
                    std::thread::sleep(Duration::from_millis(1));
                }
                return PreparedSurfaceMap {
                    authority: map_authority(&message),
                    received_at,
                    source_timestamp_ns: message.source_timestamp_ns,
                    awaiting_build: !probe.invalid.load(Ordering::Acquire),
                    snapshot: Err("injected typed fine-source failure".into()),
                    change_basis: None,
                    geometry_changes: Arc::new(SurfaceGeometryChanges::Unknown),
                };
            }
            prepare_surface_map(message, received_at, config, cache)
        })
    }
    fn count(&self, revision: i64) -> usize {
        self.attempts
            .lock()
            .unwrap()
            .iter()
            .filter(|(r, _)| *r == revision)
            .count()
    }
}

async fn wait(condition: impl Fn() -> bool) {
    tokio::time::timeout(Duration::from_secs(4), async {
        while !condition() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .expect("map-refresh owner barrier was not reached");
}

async fn active(
    probe: &Arc<Probe>,
    ttl: Duration,
) -> (Harness, schema::NavigationMap3D, schema::GoalEnvelope, i64) {
    let mut harness = Harness::start_with_components(
        Arc::new(body_path),
        Duration::from_millis(25),
        config(),
        true,
        Some(probe.computer()),
        nuwax_nav_runtime::SchedulerConfig {
            costmap_ttl: ttl,
            ..Default::default()
        },
    )
    .await;
    harness.wait_ready().await;
    let map = harness.freeze_map().await;
    let goal = goal_for(&map, 1, [3.2, 1.55, 2.5]);
    harness.goal(&goal).await;
    let path = harness.path_for(&goal).await;
    harness.wait_controlling().await;
    (harness, map, goal, path.path_revision)
}

#[tokio::test]
async fn incomplete_refresh_retains_route_and_source_then_retries_atomically() {
    let probe = Probe::new();
    let (mut harness, initial, goal, revision) = active(&probe, Duration::ZERO).await;
    let mut first = map_message(
        initial.cloud_revision + 1,
        &initial.committed_map.session_id,
    );
    first.virtual_obstacles = Some(virtual_mask(&first, false));
    harness.map(&first).await;
    wait(|| {
        harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .authority
            .cloud_revision
            == first.cloud_revision
    })
    .await;
    let retained = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    let mut next = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
    next.virtual_obstacles = Some(virtual_mask(&next, false));
    probe
        .failing_revision
        .store(next.cloud_revision, Ordering::Release);
    harness.map(&next).await;
    wait(|| probe.count(next.cloud_revision) == 1).await;
    tokio::time::sleep(Duration::from_millis(150)).await;
    let current = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    assert_eq!(current.authority, retained.authority);
    assert_eq!(current.received_at, retained.received_at);
    assert_eq!(current.source_timestamp_ns, retained.source_timestamp_ns);
    assert!(Arc::ptr_eq(
        current.snapshot.as_ref().unwrap(),
        retained.snapshot.as_ref().unwrap()
    ));
    assert_eq!(harness.scheduler.status().path_revision, revision as u64);
    assert_eq!(
        harness.scheduler.status().phase,
        SchedulerPhase::Controlling
    );
    assert_eq!(
        harness.scheduler.status().costmap_source_timestamp_ns,
        retained.source_timestamp_ns
    );
    assert!(harness.scheduler.status().costmap_receipt_age_ms >= 100);
    assert_eq!(
        probe.count(next.cloud_revision),
        1,
        "10ms owner tick must not busy-retry"
    );
    probe.recover.store(true, Ordering::Release);
    wait(|| {
        harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .authority
            .cloud_revision
            == next.cloud_revision
    })
    .await;
    let installed = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    let receipts: Vec<_> = probe
        .attempts
        .lock()
        .unwrap()
        .iter()
        .filter(|(r, _)| *r == next.cloud_revision)
        .map(|(_, receipt)| *receipt)
        .collect();
    assert_eq!(receipts.len(), 2);
    assert_eq!(receipts[0], receipts[1]);
    assert_eq!(installed.received_at, receipts[0]);
    assert_eq!(installed.source_timestamp_ns, next.source_timestamp_ns);
    assert!(installed.received_at.elapsed() >= Duration::from_secs(1));
    wait(|| harness.scheduler.status().costmap_source_timestamp_ns == next.source_timestamp_ns)
        .await;
    assert!(harness.scheduler.status().costmap_receipt_age_ms >= 1000);
    assert_eq!(harness.scheduler.status().path_revision, revision as u64);
    assert!(
        tokio::time::timeout(Duration::from_millis(80), harness.paths.next())
            .await
            .is_err()
    );
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}

#[tokio::test]
async fn incomplete_refresh_retry_never_replaces_a_newer_pending_root() {
    let probe = Probe::new();
    let (mut harness, first, goal, _) = active(&probe, Duration::ZERO).await;
    let incomplete = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
    probe
        .failing_revision
        .store(incomplete.cloud_revision, Ordering::Release);
    harness.map(&incomplete).await;
    wait(|| probe.count(incomplete.cloud_revision) == 1).await;
    let newer = map_message(
        incomplete.cloud_revision + 1,
        &first.committed_map.session_id,
    );
    harness.map(&newer).await;
    wait(|| {
        harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .authority
            .cloud_revision
            == newer.cloud_revision
    })
    .await;
    tokio::time::sleep(Duration::from_millis(1100)).await;
    assert_eq!(probe.count(incomplete.cloud_revision), 1);
    assert_eq!(probe.count(newer.cloud_revision), 1);
    assert_eq!(
        harness.scheduler.status().costmap_source_timestamp_ns,
        newer.source_timestamp_ns
    );
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}

#[tokio::test]
async fn incomplete_refresh_does_not_launder_configured_source_expiry() {
    let probe = Probe::new();
    let (mut harness, first, _, _) = active(&probe, Duration::from_millis(500)).await;
    let next = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
    probe
        .failing_revision
        .store(next.cloud_revision, Ordering::Release);
    harness.map(&next).await;
    wait(|| probe.count(next.cloud_revision) >= 2).await;
    let status = harness.scheduler.status();
    assert!(!status.planner_inputs_fresh);
    assert!(status.last_terminal.is_none() && status.session_id.is_some());
    assert_eq!(
        status.costmap_source_timestamp_ns,
        first.source_timestamp_ns
    );
    assert!(status.costmap_receipt_age_ms >= 1000);
    assert_eq!(status.last_applied_twist, Some(Twist2::ZERO));
    assert!(status.last_zero_ack_id.is_none());
    harness.shutdown().await;
}

#[tokio::test]
async fn incomplete_refresh_recovery_does_not_resurrect_cancelled_goal() {
    let probe = Probe::new();
    let (mut harness, first, goal, _) = active(&probe, Duration::ZERO).await;
    let next = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
    probe
        .failing_revision
        .store(next.cloud_revision, Ordering::Release);
    harness.map(&next).await;
    wait(|| probe.count(next.cloud_revision) == 1).await;
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    let applied = harness.scheduler.status().applied_commands;
    probe.recover.store(true, Ordering::Release);
    wait(|| {
        harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .authority
            .cloud_revision
            == next.cloud_revision
    })
    .await;
    assert!(
        tokio::time::timeout(Duration::from_millis(100), harness.paths.next())
            .await
            .is_err()
    );
    assert_eq!(
        harness.scheduler.status().last_terminal,
        Some(TerminalOutcome::Cancelled)
    );
    assert_eq!(harness.scheduler.status().applied_commands, applied);
    harness.shutdown().await;
}

#[tokio::test]
async fn incomplete_refresh_late_completion_cannot_cross_clear_or_domain_fence() {
    for clear in [false, true] {
        let probe = Probe::new();
        let (mut harness, first, _, _) = active(&probe, Duration::ZERO).await;
        let next = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
        probe
            .failing_revision
            .store(next.cloud_revision, Ordering::Release);
        probe.hold.store(true, Ordering::Release);
        harness.map(&next).await;
        wait(|| probe.count(next.cloud_revision) == 1).await;
        let mut fence = map_message(next.cloud_revision + 1, &first.committed_map.session_id);
        if clear {
            fence.cleared = true;
            fence.cloud.width = 0;
            fence.cloud.row_step = 0;
            fence.cloud.data.clear();
        } else {
            fence.map_frame_generation += 1;
        }
        harness.map(&fence).await;
        wait(|| {
            harness
                .hub
                .latest
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .authority
                .cloud_revision
                == fence.cloud_revision
        })
        .await;
        assert!(harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .snapshot
            .is_err());
        probe.hold.store(false, Ordering::Release);
        wait(|| probe.count(fence.cloud_revision) == 1).await;
        tokio::time::sleep(Duration::from_millis(1100)).await;
        assert_eq!(probe.count(next.cloud_revision), 1);
        assert_eq!(
            harness
                .hub
                .latest
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .authority
                .cloud_revision,
            fence.cloud_revision
        );
        assert_ne!(
            harness.scheduler.status().phase,
            SchedulerPhase::Controlling
        );
        assert!(harness.scheduler.status().last_zero_ack_id.is_some());
        harness.shutdown().await;
    }
}

#[tokio::test]
async fn incomplete_refresh_pauses_while_invalid_source_retires_route() {
    for invalid in [false, true] {
        let probe = Probe::new();
        let (mut harness, first, _, _) = active(&probe, Duration::ZERO).await;
        let mut next = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
        if !invalid {
            next.virtual_obstacles = Some(virtual_mask(&next, true));
        }
        probe
            .failing_revision
            .store(next.cloud_revision, Ordering::Release);
        probe.invalid.store(invalid, Ordering::Release);
        harness.map(&next).await;
        wait(|| {
            harness
                .hub
                .latest
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .snapshot
                .is_err()
        })
        .await;
        wait(|| {
            let status = harness.scheduler.status();
            if invalid {
                status.last_terminal.is_some() && status.last_zero_ack_id.is_some()
            } else {
                status.last_terminal.is_none() && status.last_applied_twist == Some(Twist2::ZERO)
                    && !status.planner_inputs_fresh
            }
        })
        .await;
        assert_eq!(
            harness
                .hub
                .latest
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .awaiting_build,
            !invalid
        );
        if invalid {
            tokio::time::sleep(Duration::from_millis(1100)).await;
            assert_eq!(
                probe.count(next.cloud_revision),
                1,
                "invalid input must not be retried"
            );
        }
        harness.shutdown().await;
    }
}

#[test]
fn incomplete_refresh_mask_guard_ignores_only_receipt_clocks() {
    let mut first = map_message(1, "same-mask");
    first.virtual_obstacles = Some(virtual_mask(&first, false));
    let mut next = map_message(2, "same-mask");
    next.virtual_obstacles = Some(virtual_mask(&next, false));
    let old = first.virtual_obstacles.as_ref().unwrap();
    let new = next.virtual_obstacles.as_ref().unwrap();
    assert_ne!((old.info.map_load_time_sec, old.info.map_load_time_nsec),
        (new.info.map_load_time_sec, new.info.map_load_time_nsec),
        "fixture must reproduce Geometry's changing production load clock");
    assert!(same_retained_mask(&first, &next));
    let mutations: [fn(&mut schema::OccupancyGrid); 8] = [
        |mask| mask.header.frame_id = "foreign".into(),
        |mask| mask.info.resolution *= 2.0,
        |mask| mask.info.width += 1,
        |mask| mask.info.height += 1,
        |mask| mask.info.origin.position[0] += 1.0,
        |mask| mask.info.origin.position[2] += 1.0,
        |mask| mask.info.origin.orientation_xyzw = [0.0, 0.0, 1.0, 0.0],
        |mask| mask.data[0] = 100,
    ];
    for mutate in mutations {
        let mut changed = next.clone();
        mutate(changed.virtual_obstacles.as_mut().unwrap());
        assert!(!same_retained_mask(&first, &changed));
    }
    next.virtual_obstacles = None;
    assert!(!same_retained_mask(&first, &next));
}

#[tokio::test]
async fn incomplete_refresh_keeps_pose_and_recent_source_deadlines_independent() {
    for recent in [false, true] {
        let probe = Probe::new();
        let (mut harness, first, _, _) = active(&probe, Duration::ZERO).await;
        let next = map_message(first.cloud_revision + 1, &first.committed_map.session_id);
        probe
            .failing_revision
            .store(next.cloud_revision, Ordering::Release);
        harness.map(&next).await;
        wait(|| probe.count(next.cloud_revision) == 1).await;
        let now = current_unix_timestamp_ns().unwrap();
        if recent {
            // Same scheduler command bound installed by RecentSourceState;
            // terrain's intentionally unbounded TTL must never renew this clock.
            harness
                .scheduler
                .update_obstacle_source(
                    None,
                    true,
                    Some(
                        nuwax_nav_runtime::CommandDispatchBound::from_source_timestamp(
                            now as u64,
                            Duration::from_millis(300),
                        ),
                    ),
                )
                .await
                .unwrap();
        } else {
            harness.feed.lock().unwrap().frozen_pose_stamp = Some(now);
        }
        wait(|| {
            let status = harness.scheduler.status();
            !status.planner_inputs_fresh
                && status.last_applied_twist == Some(Twist2::ZERO)
                && status.last_terminal.is_none()
        })
        .await;
        wait(|| probe.count(next.cloud_revision) >= 2).await;
        assert_eq!(
            harness.scheduler.status().costmap_source_timestamp_ns,
            first.source_timestamp_ns
        );
        assert!(harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .snapshot
            .is_ok());
        assert!(!harness.scheduler.status().planner_inputs_fresh);
        harness.shutdown().await;
    }
}
