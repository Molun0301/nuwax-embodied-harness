// Fact-only qualification of the actual outer Inputs.path Arc<Route>.
// Included under planner_worker::tests; all geometry and payloads are original.
// No artificial Drop, payload-size inflation, timing sample, or production hook.
use super::*;

#[derive(Debug)]
struct RetiredInputFacts {
    epoch: u64,
    path_absent: bool,
    goal_absent: bool,
    result_absent: bool,
    requested_absent: bool,
    control_width_absent: bool,
    display_empty: bool,
}

fn input_facts(worker: &PlannerWorker) -> RetiredInputFacts {
    let input = worker.shared.0.lock().unwrap();
    RetiredInputFacts {
        epoch: input.epoch,
        path_absent: input.path.is_none(),
        goal_absent: input.goal.is_none(),
        result_absent: input.result.is_none(),
        requested_absent: input.requested.is_none(),
        control_width_absent: input.control_width.is_none(),
        display_empty: input.display_path.is_empty(),
    }
}

fn assert_stopped(facts: &RetiredInputFacts, expected_epoch: u64) {
    assert_eq!(facts.epoch, expected_epoch);
    assert!(facts.path_absent && facts.goal_absent && facts.result_absent);
    assert!(facts.requested_absent && facts.control_width_absent);
    assert!(facts.display_empty);
}

fn wait_for_idle_outer_path(worker: &PlannerWorker) {
    // This deadline bounds a failed test; it is not a performance threshold.
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        let ready = {
            let input = worker.shared.0.lock().unwrap();
            !input.closed && !input.lifecycle_stop && !input.lifecycle_update
                && input.requested.is_none() && input.result.is_some()
                && input.path.as_ref().is_some_and(|path| Arc::strong_count(path) == 1)
        };
        if ready { return; }
        assert!(std::time::Instant::now() < deadline, "real worker did not finish its path-owning iteration");
        std::thread::sleep(Duration::from_millis(1));
    }
}

fn wait_for_stopped_iteration(worker: &PlannerWorker, outer: &std::sync::Weak<Route>) {
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        let complete = {
            let input = worker.shared.0.lock().unwrap();
            outer.strong_count() == 0 && !input.lifecycle_stop
                && input.path.is_none() && input.goal.is_none()
                && input.result.is_none() && input.requested.is_none()
        };
        if complete { return; }
        assert!(std::time::Instant::now() < deadline, "old worker iteration did not retire after normal release");
        std::thread::sleep(Duration::from_millis(1));
    }
}

#[test]
fn real_route_idle_stop_retires_last_outer_path_arc() {
    let mut harness = RouteWorkerHarness::new();
    let shared = Arc::clone(&harness.worker.shared); // Inputs owner only; never clone Route.
    harness.worker.step(0.3);
    let computed = harness.next_computation();
    assert_positive_route_computation(&computed);
    assert_eq!(computed.goal_updates, 1);
    assert_eq!(computed.path_updates, 1);
    let initial_path = {
        let input = harness.worker.shared.0.lock().unwrap();
        input.path.as_ref().map(|path| {
            let points = match &path.0 { ExecutionPath::Surface(surface) => Some(surface.poses.len()), _ => None };
            (Arc::downgrade(path), input.epoch, Arc::strong_count(path), points)
        })
    };
    let (outer, epoch, held_count, points) = initial_path.expect("actual installed outer path");
    let points = points.expect("real Surface fixture expected");
    assert_eq!(held_count, 2, "Inputs plus original worker local; inner follower owns a deep-cloned path");
    assert_eq!(points, 2, "use the existing legal path without artificial inflation");

    harness.release.send(()).unwrap();
    wait_for_idle_outer_path(&harness.worker);
    let idle_path = {
        let input = harness.worker.shared.0.lock().unwrap();
        input.path.as_ref().map(|path| (Arc::as_ptr(path), input.epoch, Arc::strong_count(path)))
    };
    let (idle_ptr, idle_epoch, idle_count) = idle_path.expect("idle path remains installed");
    assert_eq!(idle_ptr, outer.as_ptr());
    assert_eq!(idle_epoch, epoch);
    harness.worker.stop();
    let stopped = input_facts(&harness.worker);
    let after_stop_count = outer.strong_count();
    wait_for_stopped_iteration(&harness.worker, &outer);
    let before_cleanup = input_facts(&harness.worker);

    // A zero strong count alone can precede payload destruction. Only original
    // synchronous stop/Drop return plus its source scope establishes completion.
    // Existing Harness Drop sends a spare release, original worker Drop closes
    // and joins. No manual take/join, detached owner, or abort fallback.
    // Original Drop/join return, not Weak zero alone, closes the teardown claim.
    drop(harness);
    let (closed, final_empty) = {
        let input = shared.0.lock().unwrap();
        (input.closed, input.result.is_none() && input.path.is_none() && input.goal.is_none())
    };
    eprintln!("PATH_RETIREMENT_FACT idle initial_strong={held_count} before_stop_strong={idle_count} after_stop_strong={after_stop_count} points={points} closed={closed} final_empty={final_empty} cleanup_returned=true");
    assert_eq!(idle_count, 1);
    assert_eq!(after_stop_count, 0, "the original stop assignment retired the last outer Route payload");
    assert_stopped(&stopped, epoch + 1);
    assert_stopped(&before_cleanup, epoch + 1);
    assert!(closed && final_empty);
    assert_eq!(outer.strong_count(), 0);
    // Weak retains only the Arc control block. This test makes no allocation-time claim.
    assert!(outer.upgrade().is_none());
}

#[test]
fn real_route_inflight_stop_defers_outer_path_retirement_to_worker() {
    let mut harness = RouteWorkerHarness::new();
    let shared = Arc::clone(&harness.worker.shared);
    harness.worker.step(0.3);
    let computed = harness.next_computation();
    assert_positive_route_computation(&computed);
    assert_eq!(computed.goal_updates, 1);
    assert_eq!(computed.path_updates, 1);
    let initial_path = {
        let input = harness.worker.shared.0.lock().unwrap();
        input.path.as_ref().map(|path| {
            let points = match &path.0 { ExecutionPath::Surface(surface) => Some(surface.poses.len()), _ => None };
            (Arc::downgrade(path), input.epoch, Arc::strong_count(path), points)
        })
    };
    let (outer, epoch, held_count, points) = initial_path.expect("actual installed outer path");
    let points = points.expect("real Surface fixture expected");
    assert_eq!(held_count, 2);
    assert_eq!(points, 2);

    // Do not release the existing outer step gate before stop. Holding only a
    // PlanKernel gate would not establish that the outer Route local is alive.
    harness.worker.stop();
    let stopped = input_facts(&harness.worker);
    let after_stop_count = outer.strong_count();
    let stop_still_queued = harness.worker.shared.0.lock().unwrap().lifecycle_stop;
    harness.release.send(()).unwrap();
    wait_for_stopped_iteration(&harness.worker, &outer);
    let before_cleanup = input_facts(&harness.worker);
    let after_worker_count = outer.strong_count();

    // Original Drop/join return, not Weak zero alone, closes the teardown claim.
    drop(harness);
    let (closed, final_empty) = {
        let input = shared.0.lock().unwrap();
        (input.closed, input.result.is_none() && input.path.is_none() && input.goal.is_none())
    };
    eprintln!("PATH_RETIREMENT_FACT inflight initial_strong={held_count} after_stop_strong={after_stop_count} after_worker_strong={after_worker_count} stop_still_queued={stop_still_queued} points={points} closed={closed} final_empty={final_empty} cleanup_returned=true");
    assert_eq!(after_stop_count, 1, "original in-flight worker local is the remaining strong owner");
    assert!(stop_still_queued, "closed outer gate must precede its next lifecycle read");
    assert_eq!(after_worker_count, 0);
    assert_stopped(&stopped, epoch + 1);
    assert_stopped(&before_cleanup, epoch + 1);
    assert!(closed && final_empty, "late positive computation must never republish after stop");
    assert_eq!(outer.strong_count(), 0);
    assert!(outer.upgrade().is_none());
}
