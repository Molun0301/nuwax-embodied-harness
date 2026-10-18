// Diagnostic regressions for 2327860. Include inside runtime lib.rs tests module.
// They use the real PlannerWorker but gate only deterministic test computation.
struct DeliveryGapPlanner {
    applied: Twist2,
    entered: std::sync::mpsc::Sender<()>,
    release: std::sync::mpsc::Receiver<()>,
    gated: bool,
    blocked_pose: bool,
    blocked_map: bool,
}

fn delivery_gap_command() -> Twist2 {
    Twist2 { linear: Point2::new(0.1, 0.0), angular_z: 0.0 }
}

impl NavPlanner for DeliveryGapPlanner {
    fn replace_path(&mut self, _: Path2D) {}
    fn update_odometry(&mut self, pose: PoseStamped) { self.blocked_pose = pose.pose.position.x > 0.5; }
    fn update_costmap(&mut self, map: CostMap2D) { self.blocked_map = map.width() != 40; }
    fn observe_applied_command(&mut self, command: Twist2) { self.applied = command; }
    fn step(&mut self, _: f64) -> PlannerStep {
        if !self.gated && self.applied != Twist2::ZERO {
            self.gated = true;
            self.entered.send(()).unwrap();
            self.release.recv_timeout(Duration::from_secs(3)).expect("release gated current computation");
        }
        PlannerStep::Command(if self.blocked_pose || self.blocked_map { Twist2::ZERO } else { delivery_gap_command() })
    }
    fn stop(&mut self) {}
}

async fn delivery_gap_fixture(tick_interval: Duration) -> (
    NavScheduler<FakeMotion, planner_worker::PlannerWorker>,
    watch::Sender<SchedulerStatus>,
    std::sync::mpsc::Sender<()>,
) {
    let (entered, starts) = std::sync::mpsc::channel();
    let (release, finish) = std::sync::mpsc::channel();
    let worker = planner_worker::PlannerWorker::new(DeliveryGapPlanner {
        applied: Twist2::ZERO, entered, release: finish, gated: false, blocked_pose: false, blocked_map: false,
    }).unwrap();
    let motion = FakeMotion(Arc::new(Mutex::new(FakeMotionState::default())));
    let mut scheduler = NavScheduler::new(motion, worker,
        SchedulerConfig { tick_interval, ..SchedulerConfig::default() }).unwrap();
    let (status, _) = watch::channel(scheduler.status.clone());
    scheduler.observe_odometry(&status, map_pose(), None).unwrap();
    scheduler.observe_costmap(&status, costmap_at(1), Instant::now()).unwrap();
    scheduler.start_session(&status, session()).await.unwrap();
    scheduler.submit_path(&status, path()).await.unwrap();
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while scheduler.status.last_applied_twist != Some(delivery_gap_command()) {
        assert!(std::time::Instant::now() < deadline, "initial nonzero command missing");
        scheduler.control_tick(&status).await.unwrap();
        if scheduler.status.last_applied_twist != Some(delivery_gap_command()) {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }
    starts.recv_timeout(Duration::from_secs(1)).expect("worker computing after actual nonzero feedback");
    (scheduler, status, release)
}

#[tokio::test]
async fn delivery_gap_red_pose_expiry_during_wait_must_not_issue_nonzero() {
    let (mut scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
    let applied_before = scheduler.status.applied_commands;
    scheduler.pose_valid_until = Some(Instant::now() + Duration::from_millis(70));
    let (handle, task) = spawn_scheduler(scheduler);
    let mut status = handle.subscribe_status();
    let publisher = std::thread::spawn(move || {
        // The first scheduler tick is at 50 ms; publication follows expiry.
        std::thread::sleep(Duration::from_millis(85));
        release.send(()).unwrap();
    });
    let began = std::time::Instant::now();
    let result = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let snapshot = status.borrow().clone();
            if snapshot.applied_commands > applied_before || snapshot.last_terminal.is_some() {
                break snapshot;
            }
            status.changed().await.unwrap();
        }
    }).await.unwrap();
    let elapsed = began.elapsed();
    let applied = result.last_applied_twist;
    let terminal = result.last_terminal;
    publisher.join().unwrap();
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
    eprintln!("DELIVERY_GAP expired_pose elapsed_ms={} applied={applied:?} terminal={terminal:?}", elapsed.as_secs_f64()*1000.0);
    assert_ne!(applied, Some(delivery_gap_command()), "source deadline expired during worker wait before nonzero submission");
    assert_eq!(terminal, Some(TerminalOutcome::SafetyFailed));
}

#[tokio::test]
async fn delivery_gap_red_urgent_cancel_must_interrupt_waiting_computation() {
    let (scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
    let (handle, task) = spawn_scheduler(scheduler);
    let urgent = handle.urgent_tx.clone();
    let (reply, response) = oneshot::channel();
    let (measured, measurement) = oneshot::channel();
    let sender = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(70));
        let began = std::time::Instant::now();
        urgent.blocking_send(UrgentCommand::Cancel { session_id: "goal-1".into(), reply }).unwrap();
        response.blocking_recv().unwrap().unwrap();
        measured.send(began.elapsed()).unwrap();
    });
    let publisher = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(120));
        release.send(()).unwrap();
    });
    let elapsed = tokio::time::timeout(Duration::from_secs(1), measurement).await.unwrap().unwrap();
    assert_eq!(handle.status().last_terminal, Some(TerminalOutcome::Cancelled));
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
    sender.join().unwrap();
    publisher.join().unwrap();
    eprintln!("DELIVERY_GAP cancel elapsed_ms={}", elapsed.as_secs_f64()*1000.0);
    assert!(elapsed < Duration::from_millis(20), "urgent cancellation was stalled behind a synchronous worker wait: {elapsed:?}");
}

#[tokio::test]
async fn delivery_gap_red_wait_must_honor_configured_control_period() {
    let mut observations = Vec::new();
    for tick_interval in [Duration::from_millis(5), DEFAULT_TICK_INTERVAL] {
        let (scheduler, _, release) = delivery_gap_fixture(tick_interval).await;
        let applied_before = scheduler.status.applied_commands;
        let (handle, task) = spawn_scheduler(scheduler);
        let mut status = handle.subscribe_status();
        let began = std::time::Instant::now();
        let applied = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                if snapshot.applied_commands > applied_before { break snapshot.last_applied_twist; }
                status.changed().await.unwrap();
            }
        }).await.unwrap();
        let elapsed = began.elapsed();
        release.send(()).unwrap();
        handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
        eprintln!("DELIVERY_GAP cadence configured_ms={} actual_ms={} applied={applied:?}", tick_interval.as_millis(), elapsed.as_secs_f64()*1000.0);
        assert_eq!(applied, Some(Twist2::ZERO), "timeout must keep zero semantics");
        observations.push((tick_interval, elapsed));
    }
    assert!(observations.iter().all(|(configured, elapsed)| *elapsed < *configured * 2 + Duration::from_millis(20)),
        "wait consumed more than configured tick plus scheduling allowance: {observations:?}");
}

#[tokio::test]
async fn delivery_gap_current_result_keeps_progress_before_next_tick() {
    let (scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
    let applied_before = scheduler.status.applied_commands;
    let (handle, task) = spawn_scheduler(scheduler);
    let mut status = handle.subscribe_status();
    let publisher = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(70));
        release.send(()).unwrap();
    });
    let began = std::time::Instant::now();
    let result = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let snapshot = status.borrow().clone();
            if snapshot.applied_commands > applied_before { break snapshot; }
            status.changed().await.unwrap();
        }
    }).await.unwrap();
    let elapsed = began.elapsed();
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
    publisher.join().unwrap();
    eprintln!("DELIVERY_GAP progress elapsed_ms={} applied={:?}", elapsed.as_secs_f64()*1000.0, result.last_applied_twist);
    assert_eq!(result.last_applied_twist, Some(delivery_gap_command()));
    assert!(elapsed < Duration::from_millis(95), "current result waited for another tick: {elapsed:?}");
}


#[tokio::test]
async fn delivery_gap_queued_sources_are_admitted_before_ready_publication() {
    for moved_pose in [false, true] {
        let (mut scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
        let applied_before = scheduler.status.applied_commands;
        let publication = scheduler.planner.prepare_control_step(0.3, false).unwrap();
        let (handle, task) = spawn_scheduler(scheduler);
        let mut status = handle.subscribe_status();
        tokio::time::sleep(Duration::from_millis(60)).await;
        // This current-thread test queues the source and releases the OS worker
        // without yielding. Both events are ready when the scheduler next polls.
        let kind = if moved_pose {
            let mut pose = map_pose_at(2);
            pose.pose.position.x = 1.0;
            InputKind::UpdateOdometry(pose, None, Instant::now())
        } else {
            InputKind::UpdateCostmap(CostmapObservation {
                costmap: ExecutionMap::Planar(CostMap2D::new_all_free("map", 41, 40, 0.1)),
                source_timestamp_ns: 2,
            }, Instant::now())
        };
        let (reply, acknowledged) = oneshot::channel();
        handle.input_tx.try_send(InputCommand { kind, reply }).unwrap();
        release.send(()).unwrap();
        let until = std::time::Instant::now() + Duration::from_secs(1);
        while !publication.has_changed().unwrap() {
            assert!(std::time::Instant::now() < until, "gated publication did not arrive");
            std::thread::yield_now();
        }
        acknowledged.await.unwrap().unwrap();
        let result = tokio::time::timeout(Duration::from_secs(1), async {
            loop {
                let snapshot = status.borrow().clone();
                if snapshot.applied_commands > applied_before { break snapshot; }
                status.changed().await.unwrap();
            }
        }).await.unwrap();
        handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
        assert_eq!(result.last_applied_twist, Some(Twist2::ZERO), "queued source={moved_pose}");
        assert_eq!(result.applied_commands, applied_before + 1, "an old command crossed the admission boundary");
    }
}

#[tokio::test]
async fn delivery_gap_input_flood_cannot_extend_original_deadline() {
    let (scheduler, _, release) = delivery_gap_fixture(Duration::from_millis(5)).await;
    let applied_before = scheduler.status.applied_commands;
    let (handle, task) = spawn_scheduler(scheduler);
    let mut status = handle.subscribe_status();
    let input = handle.input_tx.clone();
    let active = Arc::new(std::sync::atomic::AtomicBool::new(true));
    let sent = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let producer = {
        let active = active.clone();
        let sent = sent.clone();
        std::thread::spawn(move || {
            while active.load(Ordering::Relaxed) {
                let (reply, _) = oneshot::channel();
                if input.blocking_send(InputCommand {
                    kind: InputKind::PlanningDiagnostic("other-goal".into(),
                        execution_identity_for("other-goal", 0).goal_key(), "queued".into()),
                    reply,
                }).is_err() { break; }
                sent.fetch_add(1, Ordering::Relaxed);
            }
        })
    };
    let began = std::time::Instant::now();
    let result = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let snapshot = status.borrow().clone();
            if snapshot.applied_commands > applied_before { break snapshot; }
            status.changed().await.unwrap();
        }
    }).await.unwrap();
    let elapsed = began.elapsed();
    active.store(false, Ordering::Relaxed);
    release.send(()).unwrap();
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
    producer.join().unwrap();
    eprintln!("DELIVERY_GAP input_flood elapsed_ms={} inputs={}", elapsed.as_secs_f64()*1000.0, sent.load(Ordering::Relaxed));
    assert_eq!(result.last_applied_twist, Some(Twist2::ZERO));
    assert!(sent.load(Ordering::Relaxed) > INPUT_CAPACITY);
    assert!(elapsed < Duration::from_millis(80), "refilling input tail starved the control deadline: {elapsed:?}");
}

#[tokio::test]
async fn delivery_gap_cancelled_window_cannot_control_replacement_session() {
    let (scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
    let (handle, task) = spawn_scheduler(scheduler);
    let began = Instant::now();
    tokio::time::sleep_until(began + Duration::from_millis(60)).await;
    handle.cancel("goal-1").await.unwrap();
    handle.start_session(session_for("goal-2")).await.unwrap();
    handle.submit_path(path_for("goal-2", 1)).await.unwrap();
    release.send(()).unwrap();
    tokio::time::sleep_until(began + Duration::from_millis(85)).await;
    assert_eq!(handle.status().session_id.as_deref(), Some("goal-2"));
    assert_eq!(handle.status().applied_commands, 0, "old deferred window drove the replacement session before its tick");
    let mut status = handle.subscribe_status();
    let result = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            let snapshot = status.borrow().clone();
            if snapshot.last_applied_twist == Some(delivery_gap_command()) { break snapshot; }
            status.changed().await.unwrap();
        }
    }).await.unwrap();
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
    assert_eq!(result.session_id.as_deref(), Some("goal-2"));
    assert_eq!(result.phase, SchedulerPhase::Controlling);
}

#[tokio::test]
async fn delivery_gap_renewal_runs_while_publication_is_pending() {
    let (mut scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
    scheduler.config.renew_interval = Duration::from_millis(60);
    let applied_before = scheduler.status.applied_commands;
    let calls = scheduler.motion.0.clone();
    let (handle, task) = spawn_scheduler(scheduler);
    tokio::time::sleep(Duration::from_millis(75)).await;
    let pending = handle.status();
    let renewed = calls.lock().unwrap().calls.iter().any(|call| call.starts_with("renew:"));
    release.send(()).unwrap();
    handle.shutdown().await.unwrap(); task.await.unwrap().unwrap();
    assert!(renewed, "renewal waited behind geometry");
    assert_eq!(pending.applied_commands, applied_before, "test must observe the open publication window");
}


// Explicit observation of the unchanged urgent-cancel regression. This does not
// replace its ordinary-suite result or change its 20 ms latency requirement.
#[cfg(feature = "surface-recovery-qualification")]
#[tokio::test]
#[ignore = "explicit urgent-cancel timeline; retains the original latency assertion"]
async fn delivery_gap_observe_urgent_cancel_timeline() {
    let (scheduler, _, release) = delivery_gap_fixture(DEFAULT_TICK_INTERVAL).await;
    // A small, preallocated session; every lost record is reported below. The
    // existing sink uses try_lock and never waits for its recording buffer.
    let trace = crate::qualification_timing::TraceSession::start(256);
    let scheduler = scheduler.with_qualification_timing(trace.sink());
    let (handle, task) = spawn_scheduler(scheduler);
    let urgent = handle.urgent_tx.clone();
    let (reply, response) = oneshot::channel();
    let (measured, measurement) = oneshot::channel();
    let sender = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(70));
        let began = std::time::Instant::now();
        urgent.blocking_send(UrgentCommand::Cancel { session_id: "goal-1".into(), reply }).unwrap();
        let queued = std::time::Instant::now();
        response.blocking_recv().unwrap().unwrap();
        let received = std::time::Instant::now();
        let elapsed = received.duration_since(began);
        // Keep B -> R as the original wall-time metric. No observer or printing
        // runs on this thread inside that interval, apart from the Q clock read.
        let _ = measured.send(elapsed);
        (began, queued, received)
    });
    let publisher = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(120));
        let before_release = std::time::Instant::now();
        release.send(()).unwrap();
        let after_release = std::time::Instant::now();
        (before_release, after_release)
    });
    let measurement_result = tokio::time::timeout(Duration::from_secs(1), measurement).await;
    let terminal_before_shutdown = handle.status().last_terminal;
    // Retain outcomes until after cleanup so a latency failure or a missing
    // measurement cannot suppress the diagnostic report. Shutdown's worker join
    // stays outside the B -> R measurement, as in the original test.
    let shutdown_result = handle.shutdown().await;
    let task_result = task.await;
    let sender_result = sender.join();
    let publisher_result = publisher.join();
    let mut report = trace.finish();
    report.records.sort_by_key(|record| record.at_ns);
    let at_ns = |at: std::time::Instant| at.duration_since(report.origin).as_nanos();
    eprintln!(
        "DELIVERY_GAP_CANCEL_TRACE capacity={} records={} record_capacity_bytes={} incomplete={} dropped_contended={} dropped_overflow={} dropped_poisoned={} dropped_closed_race={} open_spans_at_finish={} finish_buffer_unavailable={}",
        report.capacity, report.records.len(), report.record_capacity_bytes, report.incomplete,
        report.dropped_contended, report.dropped_overflow, report.dropped_poisoned,
        report.dropped_closed_race, report.open_spans_at_finish, report.finish_buffer_unavailable,
    );
    eprintln!(
        "DELIVERY_GAP_CANCEL_OUTCOME measurement={measurement_result:?} terminal_before_shutdown={terminal_before_shutdown:?} shutdown={shutdown_result:?} task={task_result:?}",
    );
    match &sender_result {
        Ok((began, queued, received)) => eprintln!(
            "DELIVERY_GAP_CANCEL_SENDER B_ns={} Q_ns={} R_ns={} send_call_ms={} receive_call_ms={} elapsed_ms={}",
            at_ns(*began), at_ns(*queued), at_ns(*received),
            queued.duration_since(*began).as_secs_f64() * 1000.0,
            received.duration_since(*queued).as_secs_f64() * 1000.0,
            received.duration_since(*began).as_secs_f64() * 1000.0,
        ),
        Err(_) => eprintln!("DELIVERY_GAP_CANCEL_SENDER missing=true reason=sender_thread_panicked"),
    }
    match &publisher_result {
        Ok((before_release, after_release)) => eprintln!(
            "DELIVERY_GAP_CANCEL_RELEASE P0_ns={} P1_ns={}",
            at_ns(*before_release), at_ns(*after_release),
        ),
        Err(_) => eprintln!("DELIVERY_GAP_CANCEL_RELEASE missing=true reason=publisher_thread_panicked"),
    }
    eprintln!(
        "DELIVERY_GAP_CANCEL_BOUNDARIES Q is sender-side send return, not actor dequeue; P0/P1 bracket release send, not worker receipt; no exact reply.send marker; terminal span end to R is an unresolved response tail",
    );
    for record in &report.records {
        eprintln!(
            "DELIVERY_GAP_CANCEL_EVENT at_ns={} phase={} source={} thread={:?} data={:?}",
            record.at_ns, record.phase, record.source, record.thread, record.data,
        );
    }
    let elapsed = measurement_result.expect("urgent-cancel measurement timed out")
        .expect("urgent-cancel measurement sender closed");
    assert_eq!(terminal_before_shutdown, Some(TerminalOutcome::Cancelled));
    shutdown_result.unwrap();
    task_result.unwrap().unwrap();
    sender_result.unwrap();
    publisher_result.unwrap();
    assert!(elapsed < Duration::from_millis(20),
        "urgent cancellation exceeded the original latency requirement: {elapsed:?}");
    assert!(!report.incomplete,
        "urgent-cancel timeline is incomplete; do not assign latency from a partial trace");
}
