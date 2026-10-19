use super::*;
use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
use std::sync::atomic::{AtomicUsize, Ordering};

fn resource_error() -> NavStackError {
    NavStackError::Planning(nuwax_nav_pct::PlanningFailure::BudgetExhausted(
        "test work allowance consumed; 未证明无路".into(),
    ))
}
fn source_with_receipt_refresh() -> (Arc<SurfaceSnapshot>, Arc<SurfaceSnapshot>) {
    let cfg = config();
    let points: Vec<_> = (0..10)
        .flat_map(|x| (0..10).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025)))
        .collect();
    let base = SurfaceSnapshot::from_points(&points, cfg.geometry.clone(), 0.1).unwrap();
    let mut first = SurfaceObservations::new(0.1, 32).unwrap();
    let values = |stamp| {
        (0..10)
            .flat_map(|x| {
                (0..10).map(move |y| {
                    (
                        [x, y, 0],
                        SurfaceObservation {
                            observed_ns: stamp,
                            hits: stamp as u16,
                        },
                    )
                })
            })
            .collect()
    };
    first.replace_block([0, 0, 0], values(1)).unwrap();
    first.mark_complete_geometry_source();
    let mut refreshed = first.clone();
    refreshed.replace_block([0, 0, 0], values(2)).unwrap();
    refreshed.mark_complete_geometry_source();
    let first = Arc::new(base.with_observations(Some(Arc::new(first))));
    let refreshed = Arc::new(first.with_observations(Some(Arc::new(refreshed))));
    assert!(nuwax_nav_pct::same_source_geometry(&first, &refreshed));
    assert!(!Arc::ptr_eq(
        first.observations().unwrap(),
        refreshed.observations().unwrap()
    ));
    (first, refreshed)
}
fn start() -> Pose3 {
    Pose3 {
        position: Point3::new(0.25, 0.25, 0.5),
        orientation: Quaternion::IDENTITY,
    }
}
#[test]
fn resource_failure_progresses_once_and_jitter_or_receipts_do_not_reset_it() {
    let (source, receipt) = source_with_receipt_refresh();
    let mut cfg = config();
    cfg.follower.pct = Arc::new(
        nuwax_nav_pct::PctPlanner::default()
            .with_search_limits(nuwax_nav_pct::SearchLimits { max_states: 60_000 }),
    );
    assert_eq!(planning_budget::initial_budget(&cfg), 30_000);
    let first = FailedPlanningInput::new(source.clone(), start(), 30_000, &resource_error());
    assert!(first.resource_limited());
    assert!(first.can_increase(&cfg));
    assert_eq!(first.next_budget(&source, start(), &cfg), Some(60_000));
    let cap = FailedPlanningInput::new(source.clone(), start(), 60_000, &resource_error());
    assert!(!cap.can_increase(&cfg));
    assert_eq!(cap.next_budget(&receipt, start(), &cfg), None);
    let mut moved = start();
    moved.position.x += cfg.follower.scan.refresh_distance_m * 0.9;
    assert_eq!(cap.next_budget(&receipt, moved, &cfg), None);
    moved.position.x += cfg.follower.scan.refresh_distance_m * 0.2;
    assert_eq!(cap.next_budget(&receipt, moved, &cfg), Some(60_000));
    let mut rotated = start();
    rotated.orientation = Quaternion::from_yaw(cfg.follower.controller.orientation_tolerance * 0.5);
    assert_eq!(cap.next_budget(&receipt, rotated, &cfg), None);
    rotated.orientation = Quaternion::from_yaw(cfg.follower.controller.orientation_tolerance * 1.1);
    assert_eq!(cap.next_budget(&receipt, rotated, &cfg), Some(60_000));
    rotated.orientation = Quaternion {
        x: 0.,
        y: 0.,
        z: 0.,
        w: -2.,
    };
    assert_eq!(
        cap.next_budget(&receipt, rotated, &cfg),
        None,
        "antipodes/scaling are not new starts"
    );
    let mut blocked = vec![false; 100];
    blocked[55] = true;
    let changed = source
        .with_virtual_obstacles([0., 0.], 0.1, 10, 10, &blocked)
        .unwrap();
    assert_eq!(cap.next_budget(&changed, start(), &cfg), Some(30_000));
}
#[test]
fn non_resource_failures_keep_the_existing_exact_input_memo() {
    let (source, receipt) = source_with_receipt_refresh();
    let cfg = config();
    let failed = FailedPlanningInput::new(
        source.clone(),
        start(),
        30_000,
        &module_error("pct global planning", "known incompatible terrain"),
    );
    assert!(!failed.resource_limited());
    assert_eq!(failed.next_budget(&source, start(), &cfg), None);
    assert_eq!(failed.next_budget(&receipt, start(), &cfg), Some(30_000));
    let mut jitter = start();
    jitter.position.x += 0.001;
    assert_eq!(failed.next_budget(&source, jitter, &cfg), Some(30_000));
}
// Exercise the real PCT resource contract at the dispatched allowance. The
// distinct-state and expansion counts may differ; either cap is inconclusive.
fn exhausted_real_search(maximum: usize) -> NavStackError {
    let map = nuwax_nav_pct::Tomogram::from_columns(0.1, [], 0.4, 0.0, 0.1, 0.1, 20.0).unwrap();
    let accepts = |a: [f64; 3], b: [f64; 3]| !(a[0].min(b[0]) <= 0.3 && a[0].max(b[0]) >= 0.3);
    let error = map
        .plan_bounded_with_segment_filter_and_limits_typed(
            [0.05, 0.05, 0.0],
            [0.55, 0.05, 0.0],
            0.0,
            &[],
            None,
            None,
            &[],
            &accepts,
            nuwax_nav_pct::SearchLimits {
                max_states: maximum,
            },
        )
        .unwrap_err();
    assert!(
        matches!(error, nuwax_nav_pct::PlanningFailure::BudgetExhausted(_)),
        "{error}"
    );
    assert!(
        error.to_string().contains(&format!("上限{maximum}")),
        "{error}"
    );
    NavStackError::Planning(error)
}
async fn wait_for_calls(calls: &AtomicUsize, count: usize) {
    tokio::time::timeout(Duration::from_secs(4), async {
        while calls.load(Ordering::SeqCst) < count {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    })
    .await
    .expect("full search attempt was not dispatched");
}
#[tokio::test]
async fn owner_escalates_budget_without_pose_jitter_loops_and_retries_meaningful_motion_at_cap() {
    async fn wait_for_completed_cap(harness: &Harness, completed: &AtomicUsize, count: usize) {
        tokio::time::timeout(Duration::from_secs(4), async {
            loop {
                // 计数只证明计算返回；还要等原拥有方接收结果，开始原重试冷却。
                let received = harness
                    .hub
                    .planning
                    .borrow()
                    .as_ref()
                    .is_some_and(|(_, _, message)| message.starts_with("搜索工作预算已用尽"));
                if completed.load(Ordering::SeqCst) >= count && received {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        })
        .await
        .expect("full search cap result was not completed and received");
    }
    let calls = Arc::new(AtomicUsize::new(0));
    let completed = Arc::new(AtomicUsize::new(0));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let count = calls.clone();
    let finished = completed.clone();
    let budgets = seen.clone();
    let computer: SurfacePathComputer = Arc::new(move |_, _, _, cfg| {
        budgets
            .lock()
            .unwrap()
            .push(cfg.planning_search_budget.unwrap());
        count.fetch_add(1, Ordering::SeqCst);
        let error = exhausted_real_search(cfg.planning_search_budget.unwrap());
        finished.fetch_add(1, Ordering::SeqCst);
        Err(error)
    });
    let cfg = config();
    let distance = cfg.follower.scan.refresh_distance_m;
    let mut harness =
        Harness::start_with_config_rate(computer, Duration::from_millis(25), cfg).await;
    harness.wait_ready().await;
    let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [3.2, 1.55, 2.5]);
    harness.goal(&goal).await;
    wait_for_calls(&calls, 2).await;
    let maximum = nuwax_nav_pct::SearchLimits::default().max_states;
    assert_eq!(*seen.lock().unwrap(), vec![30_000, maximum]);
    wait_for_completed_cap(&harness, &completed, 2).await;
    harness.feed.lock().unwrap().position[0] += distance * 0.01;
    tokio::time::sleep(Duration::from_millis(1200)).await;
    assert_eq!(
        calls.load(Ordering::SeqCst),
        2,
        "fresh receipts and pose jitter cannot restart an exhausted cap"
    );
    let status = harness.scheduler.status();
    assert_eq!(status.session_id.as_deref(), Some(goal.session_id.as_str()));
    assert!(status.message.contains("未证明无路"), "{}", status.message);
    harness.feed.lock().unwrap().position[0] += distance * 1.1;
    wait_for_calls(&calls, 3).await;
    assert_eq!(*seen.lock().unwrap(), vec![30_000, maximum, maximum]);
    wait_for_completed_cap(&harness, &completed, 3).await;
    tokio::time::sleep(Duration::from_millis(1100)).await;
    assert_eq!(calls.load(Ordering::SeqCst), 3);
    // A new goal owns a fresh initial allowance, never the retired goal's cap.
    let next = goal_for(&harness.feed.lock().unwrap().map, 2, [3.25, 1.55, 2.5]);
    harness.goal(&next).await;
    wait_for_calls(&calls, 4).await;
    assert_eq!(seen.lock().unwrap()[3], 30_000);
    harness.shutdown().await;
}
#[tokio::test]
async fn budget_progression_cannot_extend_an_explicit_original_path_wait_deadline() {
    let calls = Arc::new(AtomicUsize::new(0));
    let count = calls.clone();
    let computer: SurfacePathComputer = Arc::new(move |_, _, _, cfg| {
        count.fetch_add(1, Ordering::SeqCst);
        Err(exhausted_real_search(cfg.planning_search_budget.unwrap()))
    });
    let mut harness = Harness::start_with_components(
        computer,
        Duration::from_millis(25),
        config(),
        true,
        None,
        None,
        nuwax_nav_runtime::SchedulerConfig {
            path_wait_timeout: Duration::from_millis(2000),
            ..nuwax_nav_runtime::SchedulerConfig::default()
        },
    )
    .await;
    harness.wait_ready().await;
    let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [3.2, 1.55, 2.5]);
    let began = Instant::now();
    harness.goal(&goal).await;
    wait_for_calls(&calls, 2).await;
    tokio::time::timeout(
        Duration::from_millis(2500).saturating_sub(began.elapsed()),
        async {
            while harness.scheduler.status().phase != SchedulerPhase::Failed {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        },
    )
    .await
    .expect("budget progression extended the original request deadline");
    let message = harness.scheduler.status().message;
    assert!(
        message.contains("planner path timeout after 2000ms"),
        "{message}"
    );
    assert!(
        message.contains("BudgetExhausted") && message.contains("未证明无路"),
        "{message}"
    );
    assert_eq!(calls.load(Ordering::SeqCst), 2);
    harness.shutdown().await;
}

#[test]
fn budget_and_domain_status_are_typed_not_inferred_from_diagnostic_text() {
    let (source, receipt) = source_with_receipt_refresh();
    let cfg = config();
    let misleading = module_error(
        "pct global planning",
        "BudgetExhausted: ordinary input text",
    );
    let input = FailedPlanningInput::new(source.clone(), start(), 30_000, &misleading);
    assert!(!input.resource_limited());
    let typed = NavStackError::Planning(nuwax_nav_pct::PlanningFailure::BudgetExhausted(
        "localized wording".into(),
    ));
    let budget = FailedPlanningInput::new(source.clone(), start(), 30_000, &typed);
    assert!(budget.resource_limited());
    assert!(budget.can_increase(&cfg));
    let typed = NavStackError::Planning(nuwax_nav_pct::PlanningFailure::SearchDomainLimited(
        "need geometry".into(),
    ));
    let domain = FailedPlanningInput::new(source, start(), 30_000, &typed);
    assert!(domain.domain_limited());
    assert!(!domain.can_increase(&cfg));
    assert_eq!(domain.next_budget(&receipt, start(), &cfg), None);
    let mut jitter = start();
    jitter.position.x += cfg.follower.scan.refresh_distance_m * 0.1;
    assert_eq!(domain.next_budget(&receipt, jitter, &cfg), None);
    jitter.position.x += cfg.follower.scan.refresh_distance_m;
    assert_eq!(domain.next_budget(&receipt, jitter, &cfg), Some(30_000));
}
