// 真实规划拥有方与调度器接线；对象补取的原子准入由独立块测试覆盖。
use super::*;
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};

struct Probe {
    failing_revision: AtomicI64,
    hold: AtomicBool,
    recover: AtomicBool,
    attempts: Mutex<Vec<(i64, Instant)>>,
    started: Mutex<Vec<(i64, Instant)>>,
}
impl Probe {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            failing_revision: AtomicI64::new(-1),
            hold: AtomicBool::new(false),
            recover: AtomicBool::new(false),
            attempts: Mutex::new(Vec::new()),
            started: Mutex::new(Vec::new()),
        })
    }
    fn computer(self: &Arc<Self>) -> SurfaceMapComputer {
        let probe = Arc::clone(self);
        Arc::new(move |message, received_at, config, cache| {
            probe.started.lock().unwrap().push((message.revision, Instant::now()));
            probe
                .attempts
                .lock()
                .unwrap()
                .push((message.revision, received_at));
            if (message.revision == probe.failing_revision.load(Ordering::Acquire)
                || probe.failing_revision.load(Ordering::Acquire) == i64::MAX)
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
                    awaiting_build: true,
                    snapshot: Err("候选补取或构建失败".into()),
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
) -> (Harness, schema::NavigationMap3D, schema::NavigationGoal, i64) {
    let mut harness = Harness::start_with_components(
        Arc::new(|m,p,g,c|body_path(m,p,g,c).map(SurfacePlan::complete)),
        Duration::from_millis(25),
        config(),
        true,
        Some(probe.computer()),
        nuwax_nav_runtime::SchedulerConfig::default(),
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
    let (mut harness, initial, goal, revision) = active(&probe).await;
    let mut first = map_message(
        initial.revision + 1,
        &initial.session_id,
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
            .revision
            == first.revision
    })
    .await;
    let retained = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    let mut next = map_message(first.revision + 1, &first.session_id);
    next.virtual_obstacles = Some(virtual_mask(&next, false));
    probe
        .failing_revision
        .store(next.revision, Ordering::Release);
    harness.map(&next).await;
    wait(|| probe.count(next.revision) == 1).await;
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
        probe.count(next.revision),
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
            .revision
            == next.revision
    })
    .await;
    let installed = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    let receipts: Vec<_> = probe
        .attempts
        .lock()
        .unwrap()
        .iter()
        .filter(|(r, _)| *r == next.revision)
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
    let (mut harness, first, goal, _) = active(&probe).await;
    let incomplete = map_message(first.revision + 1, &first.session_id);
    probe
        .failing_revision
        .store(incomplete.revision, Ordering::Release);
    probe.hold.store(true, Ordering::Release);
    harness.map(&incomplete).await;
    wait(|| probe.count(incomplete.revision) == 1).await;
    let newer = map_message(
        incomplete.revision + 1,
        &first.session_id,
    );
    harness.map(&newer).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    let released = Instant::now();
    probe.hold.store(false, Ordering::Release);
    wait(|| {
        harness
            .hub
            .latest
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .authority
            .revision
            == newer.revision
    })
    .await;
    let started = probe.started.lock().unwrap().iter().find(|(r,_)| *r == newer.revision).unwrap().1;
    assert!(started.duration_since(released) < Duration::from_millis(500));
    tokio::time::sleep(Duration::from_millis(1100)).await;
    assert_eq!(probe.count(incomplete.revision), 1);
    assert_eq!(probe.count(newer.revision), 1);
    assert_eq!(
        harness.scheduler.status().costmap_source_timestamp_ns,
        newer.source_timestamp_ns
    );
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}

#[tokio::test]
async fn incomplete_refresh_recovery_does_not_resurrect_cancelled_goal() {
    let probe = Probe::new();
    let (mut harness, first, goal, _) = active(&probe).await;
    let next = map_message(first.revision + 1, &first.session_id);
    probe
        .failing_revision
        .store(next.revision, Ordering::Release);
    harness.map(&next).await;
    wait(|| probe.count(next.revision) == 1).await;
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
            .revision
            == next.revision
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
        let (mut harness, first, _, _) = active(&probe).await;
        let next = map_message(first.revision + 1, &first.session_id);
        probe
            .failing_revision
            .store(next.revision, Ordering::Release);
        probe.hold.store(true, Ordering::Release);
        harness.map(&next).await;
        wait(|| probe.count(next.revision) == 1).await;
        let mut fence = map_message(next.revision + 1, &first.session_id);
        if clear {
            fence.cleared = true;
            fence.cloud.width = 0;
            fence.cloud.row_step = 0;
            fence.cloud.data.clear();
        } else {
            fence.session_id = "新地图会话".into();
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
                .revision
                == fence.revision
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
        if !clear {
            wait(|| probe.count(fence.revision) == 1).await;
        }
        tokio::time::sleep(Duration::from_millis(1100)).await;
        assert_eq!(probe.count(fence.revision), usize::from(!clear));
        assert_eq!(probe.count(next.revision), 1);
        assert_eq!(
            harness
                .hub
                .latest
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .authority
                .revision,
            fence.revision
        );
        assert_ne!(
            harness.scheduler.status().phase,
            SchedulerPhase::Controlling
        );

        harness.shutdown().await;
    }
}

#[tokio::test]
async fn incomplete_refresh_does_not_renew_pose_source_deadline() {
    {
        let probe = Probe::new();
        let (mut harness, first, _, _) = active(&probe).await;
        let next = map_message(first.revision + 1, &first.session_id);
        probe
            .failing_revision
            .store(next.revision, Ordering::Release);
        harness.map(&next).await;
        wait(|| probe.count(next.revision) == 1).await;
        let now = current_unix_timestamp_ns().unwrap();
        harness.feed.lock().unwrap().frozen_pose_stamp=Some(now);
        tokio::time::timeout(Duration::from_secs(4), async {
            loop {
                let status = harness.scheduler.status();
                if !status.planner_inputs_fresh
                    && status.last_applied_twist == Some(Twist2::ZERO)
                    && status.last_terminal.is_none() { break; }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.unwrap_or_else(|_| panic!("位姿来源期限未保持: {:?}", harness.scheduler.status()));
        wait(|| probe.count(next.revision) >= 2).await;
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

#[tokio::test]
async fn candidate_failure_without_installed_map_waits_then_retries_same_root() {
    let probe = Probe::new();
    probe.failing_revision.store(i64::MAX, Ordering::Release);
    let mut harness = Harness::start_with_components(
        Arc::new(|m,p,g,c|body_path(m,p,g,c).map(SurfacePlan::complete)),
        Duration::from_millis(25), config(), true, Some(probe.computer()),
        nuwax_nav_runtime::SchedulerConfig::default(),
    ).await;
    wait(|| harness.hub.latest.lock().unwrap().is_some()).await;
    let first = harness.freeze_map().await;
    let goal = goal_for(&first, 1, [3.2, 1.55, 2.5]);
    harness.goal(&goal).await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    let update = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    assert!(update.snapshot.is_err() && update.awaiting_build);
    assert!(harness.scheduler.status().last_terminal.is_none());
    assert_eq!(harness.scheduler.status().applied_commands, 0);
    assert!(harness.calls.lock().unwrap().is_empty(), "无完整地图时目标应留在入口等待，不能取得租约或发速度");
    probe.recover.store(true, Ordering::Release);
    wait(|| harness.hub.latest.lock().unwrap().as_ref().is_some_and(|m| m.snapshot.is_ok())).await;
    let times: Vec<_> = probe.started.lock().unwrap().iter().filter(|(r,_)| *r == first.revision).map(|(_,at)| *at).collect();
    assert_eq!(times.len(), 2);
    assert!(times[1].duration_since(times[0]) >= Duration::from_secs(1));
    harness.path_for(&goal).await;
    harness.wait_controlling().await;
    assert!(harness.scheduler.status().last_terminal.is_none());
    harness.shutdown().await;
}

#[tokio::test]
async fn complete_candidate_build_failure_keeps_the_installed_map_and_active_goal() {
    use nuwax_map_chunks::{Repository,Writer,Transfer,MAX_BLOCK_BYTES};
    let probe = Probe::new();
    let (mut harness, first, goal, path_revision) = active(&probe).await;
    let installed = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    let mut next = map_message(first.revision+1,&first.session_id);
    let repository=Repository::temporary().unwrap();
    let mut writer=Writer::new(repository.clone(),0.2,4*1024*1024).unwrap();
    writer.observe([[0,0,0]],next.source_timestamp_ns as u64).unwrap();
    next.source_voxel_size=0.2;
    next.cloud.width=0;next.cloud.row_step=0;next.cloud.data.clear();
    let root=writer.snapshot(schema::navigation_map_3d_domain(&next),next.revision as u64,next.source_timestamp_ns as u64).unwrap();
    next.fine=Some(Transfer {root:root.encode().unwrap(),included:root.blocks.iter().map(|b|
        (b.object,repository.read_object(b.object,MAX_BLOCK_BYTES).unwrap())).collect()});
    let mut cache=SurfaceGeometryCache::configured(&config());
    let failed=prepare_surface_map(next.clone(),Instant::now(),&config(),&mut cache);
    assert!(failed.snapshot.is_err() && failed.awaiting_build,"对象已齐，规划精度不兼容仍只放弃候选");
    harness.map(&next).await;
    wait(||probe.count(next.revision)==1).await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    let retained=harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    assert_eq!(retained.authority,installed.authority);
    assert!(Arc::ptr_eq(retained.snapshot.as_ref().unwrap(),installed.snapshot.as_ref().unwrap()));
    let status=harness.scheduler.status();
    assert_eq!(status.phase,SchedulerPhase::Controlling);
    assert_eq!(status.path_revision,path_revision as u64);
    assert!(status.last_terminal.is_none());
    harness.cancel(&goal).await;harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}
