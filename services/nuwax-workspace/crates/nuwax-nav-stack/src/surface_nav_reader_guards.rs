// TESTS-ONLY CANDIDATE, NOT COMPILED OR RUN by its author.
// Install as src/surface_nav_reader_guards.rs and apply nav-reader-hooks.patch
// AFTER dependency-backoff.candidate.patch. The hook diff is candidate-relative.
// Run in the validator's existing liblcm-enabled build, with the environment
// supplied externally: NUWAX_LCM_URL=memq://. No RPC server is registered.
// Selector prefix: lcm_dataflow::surface_dataflow::tests::nav_reader_guards::
//   real_reader_same_missing_watermark_after_failure_keeps_original_deadline
//   real_reader_same_missing_watermark_pending_inflight_keeps_original_deadline
//   real_reader_complete_successor_after_failure_starts_before_original_deadline
//   real_reader_complete_successor_pending_inflight_starts_before_original_deadline
// Example (from services/nuwax-workspace, using the validator's existing target):
// NUWAX_LCM_URL=memq:// cargo test -p nuwax-nav-stack --features mls --lib \
//   lcm_dataflow::surface_dataflow::tests::nav_reader_guards:: -- --test-threads=1
// For one full selector, append --exact to the libtest arguments instead.
// This qualifies real Nav consumer ingress, FineMapReader, owner scheduling,
// source retention and cleanup. It does not qualify sensor-to-commit lineage.
// costmap_ttl remains ZERO: these are owner-backoff/retained-identity guards,
// not source-TTL qualification. No source or receipt clock is refreshed.
use super::*;
use futures::FutureExt;
use nuwax_map_chunks::{Key, Manifest, ObjectIdentity, Repository, Transfer, Writer, MAX_BLOCK_BYTES};
use std::sync::atomic::{AtomicBool, Ordering};

type Missing = (ObjectIdentity, Key);

#[derive(Clone, Copy, Debug)]
pub(crate) struct Ingress {
    revision: i64,
    received_at: Instant,
    at: Instant,
    retry_after: Instant,
    dependency_present: bool,
    in_flight: bool,
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct Started {
    revision: i64,
    received_at: Instant,
    at: Instant,
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct Finished {
    revision: i64,
    received_at: Instant,
    awaiting: bool,
    success: bool,
    missing: Option<Missing>,
}
#[derive(Clone, Copy, Debug)]
pub(crate) struct RetryDecision {
    revision: i64,
    received_at: Instant,
    original_deadline: Instant,
    effective_deadline: Instant,
    pending_revision: Option<i64>,
    missing: Option<Missing>,
    retained: bool,
    dependency_present: bool,
}
#[derive(Default)]
struct Observations {
    ingress: Vec<Ingress>,
    started: Vec<Started>,
    finished: Vec<Finished>,
    decisions: Vec<RetryDecision>,
}

// Disabled for all existing tests. These hooks neither fabricate a build result
// nor touch the Reader, cache, root, source timestamps or production deadline.
#[derive(Default)]
pub(crate) struct ReaderGuardProbe {
    enabled: AtomicBool,
    events: Mutex<Observations>,
    hold_revision: Mutex<Option<i64>>,
    release: std::sync::Condvar,
}
impl ReaderGuardProbe {
    pub(crate) fn accepted(&self, revision: i64, received_at: Instant,
        retry_after: Instant, dependency_present: bool, in_flight: bool) {
        if !self.enabled.load(Ordering::Acquire) { return; }
        let at = Instant::now();
        self.events.lock().unwrap().ingress.push(Ingress {
            revision, received_at, at, retry_after,
            dependency_present, in_flight,
        });
    }
    pub(crate) fn started(&self, revision: i64, received_at: Instant) {
        if !self.enabled.load(Ordering::Acquire) { return; }
        let at = Instant::now();
        self.events.lock().unwrap().started.push(Started { revision, received_at, at });
    }
    pub(crate) fn finished(&self, revision: i64, received_at: Instant,
        awaiting: bool, success: bool, missing: Option<Missing>) {
        if !self.enabled.load(Ordering::Acquire) { return; }
        self.events.lock().unwrap().finished.push(Finished {
            revision, received_at, awaiting, success, missing,
        });
        // Hold ONLY after the real Reader/cache operation has returned. The
        // original JoinSet remains in flight and continues owning this worker.
        let held = self.hold_revision.lock().unwrap();
        let (held, timeout) = self.release.wait_timeout_while(
            held, Duration::from_secs(15), |held| *held == Some(revision),
        ).unwrap();
        assert!(!timeout.timed_out() || *held != Some(revision),
            "test-only real-Reader return barrier expired");
    }
    pub(crate) fn decision(&self, revision: i64, received_at: Instant,
        original_deadline: Instant, effective_deadline: Instant,
        pending_revision: Option<i64>, missing: Option<Missing>,
        retained: bool, dependency_present: bool) {
        if !self.enabled.load(Ordering::Acquire) { return; }
        self.events.lock().unwrap().decisions.push(RetryDecision {
            revision, received_at, original_deadline, effective_deadline,
            pending_revision, missing, retained, dependency_present,
        });
    }
    fn count(&self, revision: i64) -> usize {
        self.events.lock().unwrap().started.iter().filter(|e| e.revision == revision).count()
    }
}
struct ReturnGate(Arc<ReaderGuardProbe>);
impl ReturnGate {
    fn arm(probe: &Arc<ReaderGuardProbe>, revision: i64) -> Self {
        assert!(probe.hold_revision.lock().unwrap().replace(revision).is_none());
        Self(Arc::clone(probe))
    }
}
impl Drop for ReturnGate {
    fn drop(&mut self) {
        *self.0.hold_revision.lock().unwrap_or_else(|error| error.into_inner()) = None;
        self.0.release.notify_all();
    }
}

async fn observed<T>(mut read: impl FnMut() -> Option<T>) -> T {
    tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            if let Some(value) = read() { return value; }
            // Polling only: every scheduling assertion below compares recorded
            // worker/owner Instants, never a guessed sleep or elapsed threshold.
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }).await.expect("real Nav Reader/owner observation did not arrive")
}
async fn ingress(harness: &Harness, message: &schema::NavigationMap3D) -> Ingress {
    harness.map(message).await;
    observed(|| harness.hub.reader_guard.events.lock().unwrap().ingress.iter()
        .find(|e| e.revision == message.cloud_revision).copied()).await
}
async fn finished(probe: &ReaderGuardProbe, revision: i64) -> Finished {
    observed(|| probe.events.lock().unwrap().finished.iter()
        .find(|e| e.revision == revision).copied()).await
}
async fn decision(probe: &ReaderGuardProbe, revision: i64) -> RetryDecision {
    observed(|| probe.events.lock().unwrap().decisions.iter()
        .find(|e| e.revision == revision).copied()).await
}
async fn started(probe: &ReaderGuardProbe, revision: i64) -> Started {
    observed(|| probe.events.lock().unwrap().started.iter()
        .find(|e| e.revision == revision).copied()).await
}
fn installed(harness: &Harness) -> PreparedSurfaceMap {
    harness.hub.latest.lock().unwrap().as_ref().unwrap().clone()
}
async fn installed_revision(harness: &Harness, revision: i64) -> PreparedSurfaceMap {
    observed(|| harness.hub.latest.lock().unwrap().as_ref()
        .filter(|m| m.authority.cloud_revision == revision && m.snapshot.is_ok()).cloned()).await
}

struct FineSeries {
    first: schema::NavigationMap3D,
    partial: schema::NavigationMap3D,
    watermarks: Vec<schema::NavigationMap3D>,
    complete: schema::NavigationMap3D,
    missing: Missing,
}
impl FineSeries {
    fn new(seed: &schema::NavigationMap3D) -> Self {
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
        let mut basis = map_message(seed.cloud_revision + 1, &seed.committed_map.session_id);
        basis.source_voxel_size = 0.05;
        basis.cloud.width = 0;
        basis.cloud.row_step = 0;
        basis.cloud.data.clear();
        basis.virtual_obstacles = Some(virtual_mask(&basis, false));
        writer.observe((0..80).flat_map(|x| (0..60).map(move |y| [x, y, 40])),
            basis.source_timestamp_ns as u64).unwrap();
        let root = writer.snapshot(schema::navigation_map_3d_domain(&basis),
            basis.cloud_revision as u64, basis.source_timestamp_ns as u64).unwrap();
        let first = Self::message(&basis, &repository, &root, true);
        assert!(root.blocks.iter().all(|b| first.fine.as_ref().unwrap().included.contains_key(&b.object)));

        // One real geometry edit assigns one new immutable object. The complete
        // initial root was already all-inline, and this object's ID is absent
        // from it, so the Reader cannot satisfy the missing object from cache.
        writer.remove([[70, 50, 40]]).unwrap();
        let failed_root = writer.snapshot(root.domain.clone(), root.revision + 1,
            root.source_timestamp_ns).unwrap();
        let changed: Vec<_> = failed_root.blocks.iter().filter(|next|
            !root.blocks.iter().any(|old| old.object == next.object)).collect();
        assert_eq!(changed.len(), 1, "fixture must omit exactly one new object");
        assert!(failed_root.geometry_revision > root.geometry_revision);
        let missing = (changed[0].object, changed[0].key);
        let mut partial = Self::message(&basis, &repository, &failed_root, true);
        assert!(partial.fine.as_mut().unwrap().included.remove(&missing.0).is_some());
        schema::validate_navigation_map_3d(&partial).unwrap();

        let watermark_source = Arc::new(nuwax_map_chunks::LayerSource::Auxiliary {
            sensor_id: "nav-real-reader-guard".into(), clock_id: "fixture-clock".into(),
            runtime_generation: 1, session_generation: 1,
        });
        let mut watermarks = Vec::new();
        let mut previous = Arc::clone(&failed_root);
        for sequence in 1..=3 {
            writer.record_source_watermark(Arc::clone(&watermark_source),
                failed_root.source_timestamp_ns, sequence).unwrap();
            let next = writer.snapshot(failed_root.domain.clone(), failed_root.revision + sequence,
                failed_root.source_timestamp_ns).unwrap();
            assert_eq!(next.blocks, failed_root.blocks);
            assert_eq!(next.geometry_revision, failed_root.geometry_revision);
            assert_eq!(next.source_timestamp_ns, failed_root.source_timestamp_ns);
            assert_ne!(next.source_watermarks, previous.source_watermarks);
            // The immutable blocks did not change. An empty inline delta leaves
            // precisely the same uncached object O unavailable to the Reader.
            let message = Self::message(&partial, &repository, &next, false);
            assert!(message.fine.as_ref().unwrap().included.is_empty());
            assert_eq!(message.committed_map, partial.committed_map);
            assert_eq!(message.commit_sequence, partial.commit_sequence);
            assert_eq!(message.source_timestamp_ns, partial.source_timestamp_ns);
            assert_eq!(message.virtual_obstacles, partial.virtual_obstacles);
            watermarks.push(message);
            previous = next;
        }

        // A different block changes. O remains referenced, and the full inline
        // successor actually supplies O as well as every other referenced block.
        writer.remove([[5, 50, 40]]).unwrap();
        let complete_root = writer.snapshot(previous.domain.clone(), previous.revision + 1,
            previous.source_timestamp_ns).unwrap();
        assert!(complete_root.geometry_revision > failed_root.geometry_revision);
        assert!(complete_root.blocks.iter().any(|b| b.object == missing.0));
        let complete = Self::message(&partial, &repository, &complete_root, true);
        assert!(complete.fine.as_ref().unwrap().included.contains_key(&missing.0));
        assert!(complete_root.blocks.iter().all(|b|
            complete.fine.as_ref().unwrap().included.contains_key(&b.object)));
        Self { first, partial, watermarks, complete, missing }
    }
    fn message(basis: &schema::NavigationMap3D, repository: &Repository,
        root: &Manifest, all_inline: bool) -> schema::NavigationMap3D {
        let mut message = basis.clone();
        message.cloud_revision = i64::try_from(root.revision).unwrap();
        message.cloud.header.seq = i32::try_from(root.revision).unwrap();
        let included = root.blocks.iter().filter(|_| all_inline).map(|block|
            (block.object, repository.read_object(block.object, MAX_BLOCK_BYTES).unwrap())).collect();
        message.fine = Some(Transfer { root: root.encode().unwrap(), included });
        schema::validate_navigation_map_3d(&message).unwrap();
        message
    }
}

async fn start_reader_harness() -> Harness {
    // No process-wide set_var/remove_var, external network, fake fetcher, cache
    // injection or map_computer override. Each Reader owns its real memq context.
    // This prerequisite precedes resource creation. Once construction returns,
    // ALL fixture setup, preconditions and business assertions are caught below.
    assert_eq!(std::env::var("NUWAX_LCM_URL").ok().as_deref(), Some("memq://"),
        "invoke the selected tests with NUWAX_LCM_URL=memq:// externally");
    Harness::start_with_components(Arc::new(body_path),
        Duration::from_millis(25), config(), true, None, None,
        nuwax_nav_runtime::SchedulerConfig { costmap_ttl: Duration::ZERO, ..Default::default() }).await
}

async fn active(harness: &mut Harness) -> (FineSeries, PreparedSurfaceMap, schema::GoalEnvelope, i64) {
    assert!(harness.hub.map_computer.is_none());
    harness.wait_ready().await;
    let seed = harness.freeze_map().await;
    harness.hub.reader_guard.enabled.store(true, Ordering::Release);
    // Prepare every Writer publication before any failed-input retry deadline.
    let series = FineSeries::new(&seed);
    let received = ingress(&harness, &series.first).await;
    let retained = installed_revision(&harness, series.first.cloud_revision).await;
    assert_eq!(retained.received_at, received.received_at);
    let built = finished(&harness.hub.reader_guard, series.first.cloud_revision).await;
    assert!(built.success && !built.awaiting && built.missing.is_none());
    let goal = goal_for(&series.first, 1, [3.2, 1.55, 2.5]);
    harness.goal(&goal).await;
    let path = harness.path_for(&goal).await;
    harness.wait_controlling().await;
    observed(|| (harness.scheduler.status().costmap_source_timestamp_ns
        == retained.source_timestamp_ns).then_some(())).await;
    (series, retained, goal, path.path_revision)
}

fn panic_text(payload: &(dyn std::any::Any + Send)) -> &str {
    payload.downcast_ref::<String>().map(String::as_str)
        .or_else(|| payload.downcast_ref::<&str>().copied()).unwrap_or("non-string panic payload")
}

// A narrow replacement for calling Harness::shutdown after assertions. Keep its
// original feeder -> executor -> planner order and APIs, but attempt every step
// after a business panic or a cleanup error. Never substitute test-owned aborts
// for the original planner/scheduler owners' shutdown and join contracts.
async fn cleanup_and_resume(harness: &mut Harness,
    business: Result<(), Box<dyn std::any::Any + Send>>) {
    let mut evidence = Vec::new();
    let mut cleanup_failed = false;
    // The in-scope ReturnGate has already dropped when catch_unwind resolves.
    // Releasing again is harmless and makes cleanup independent of that Drop.
    *harness.hub.reader_guard.hold_revision.lock()
        .unwrap_or_else(|error| error.into_inner()) = None;
    harness.hub.reader_guard.release.notify_all();
    evidence.push("Reader return barrier released".to_string());

    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| harness.feeder.abort())) {
        Ok(()) => evidence.push("feeder abort requested".into()),
        Err(payload) => {
            cleanup_failed = true;
            evidence.push(format!("feeder abort panicked: {}", panic_text(payload.as_ref())));
        }
    }
    match std::panic::AssertUnwindSafe(async { (&mut harness.feeder).await }).catch_unwind().await {
        Ok(Ok(())) => evidence.push("feeder joined: completed".into()),
        Ok(Err(error)) if error.is_cancelled() => evidence.push("feeder joined: cancellation observed".into()),
        Ok(Err(error)) => {
            cleanup_failed = true;
            evidence.push(format!("feeder joined with failure: {error}"));
        }
        Err(payload) => {
            cleanup_failed = true;
            evidence.push(format!("feeder join panicked: {}", panic_text(payload.as_ref())));
        }
    }
    for (name, slot) in [("executor", &mut harness.executor), ("planner", &mut harness.planner)] {
        let Some(mut owner) = slot.take() else {
            cleanup_failed = true;
            evidence.push(format!("{name}: owner missing before explicit cleanup"));
            continue;
        };
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| owner.shutdown())) {
            Ok(Ok(())) => evidence.push(format!("{name} shutdown/join returned Ok")),
            Ok(Err(error)) => {
                cleanup_failed = true;
                evidence.push(format!("{name} shutdown/join failed: {error}"));
            }
            Err(payload) => {
                cleanup_failed = true;
                evidence.push(format!("{name} shutdown/join panicked: {}", panic_text(payload.as_ref())));
            }
        }
        // Check status even if this owner's shutdown returned an error/panic,
        // then continue to the other owner regardless of the status result.
        let stopped = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| owner.is_running())) {
            Ok(Ok(false)) => {
                evidence.push(format!("{name} status: stopped"));
                true
            }
            Ok(Ok(true)) => {
                cleanup_failed = true;
                evidence.push(format!("{name} status: STILL RUNNING"));
                false
            }
            Ok(Err(error)) => {
                cleanup_failed = true;
                evidence.push(format!("{name} status failed: {error}"));
                false
            }
            Err(payload) => {
                cleanup_failed = true;
                evidence.push(format!("{name} status panicked: {}", panic_text(payload.as_ref())));
                false
            }
        };
        // Like the original shutdown method, this path takes each owner. It
        // records shutdown/join and status separately, then catches even Drop,
        // so a later Harness::Drop cannot replace the original business panic.
        match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| drop(owner))) {
            Ok(()) => evidence.push(format!("{name} owner released; verified_stopped={stopped}")),
            Err(payload) => {
                cleanup_failed = true;
                evidence.push(format!("{name} owner drop panicked: {}", panic_text(payload.as_ref())));
            }
        }
    }
    let evidence = evidence.join("; ");
    eprintln!("NAV_READER_GUARD_CLEANUP business_panicked={} cleanup_failed={cleanup_failed}: {evidence}",
        business.is_err());
    match business {
        Err(payload) => std::panic::resume_unwind(payload),
        Ok(()) => assert!(!cleanup_failed, "Nav Reader guard cleanup failed: {evidence}"),
    }
}

fn assert_retained(harness: &Harness, retained: &PreparedSurfaceMap, path_revision: i64) {
    let current = installed(harness);
    assert_eq!(current.authority, retained.authority);
    assert_eq!(current.received_at, retained.received_at);
    assert_eq!(current.source_timestamp_ns, retained.source_timestamp_ns);
    assert!(Arc::ptr_eq(current.snapshot.as_ref().unwrap(), retained.snapshot.as_ref().unwrap()));
    let status = harness.scheduler.status();
    assert_eq!(status.costmap_source_timestamp_ns, retained.source_timestamp_ns);
    assert_eq!(status.path_revision, path_revision as u64);
    assert_eq!(status.phase, SchedulerPhase::Controlling);
}
fn assert_real_failure(failed: Finished, received: Ingress, missing: Missing) {
    assert_eq!(failed.received_at, received.received_at);
    assert!(failed.awaiting && !failed.success);
    assert_eq!(failed.missing, Some(missing), "original Reader must identify O, not a fake Err");
}

async fn same_missing_watermarks(queued_while_inflight: bool) {
    let mut harness = start_reader_harness().await;
    let business = std::panic::AssertUnwindSafe(async {
        let (series, retained, goal, path_revision) = active(&mut harness).await;
        let probe = Arc::clone(&harness.hub.reader_guard);
        // This gate drops before catch_unwind completes, before explicit cleanup.
        let gate = queued_while_inflight.then(|| ReturnGate::arm(&probe, series.partial.cloud_revision));
        let receipt = ingress(&harness, &series.partial).await;
        assert_real_failure(finished(&probe, series.partial.cloud_revision).await, receipt, series.missing);
        let before = if queued_while_inflight { None }
            else { Some(decision(&probe, series.partial.cloud_revision).await) };
        for watermark in &series.watermarks {
            let admitted = ingress(&harness, watermark).await;
            if let Some(backoff) = before {
                assert!(admitted.at < backoff.original_deadline, "fixture missed the original retry interval");
                assert_eq!(admitted.retry_after, backoff.original_deadline,
                    "same O watermark must neither restart nor bypass the original deadline");
                assert!(admitted.dependency_present);
                assert!(!admitted.in_flight);
            } else {
                assert!(admitted.in_flight, "successor must really be pending before the old worker returns");
            }
        }
        drop(gate);
        let backoff = decision(&probe, series.partial.cloud_revision).await;
        assert_eq!(backoff.received_at, receipt.received_at);
        assert_eq!(backoff.missing, Some(series.missing));
        assert!(backoff.retained && backoff.dependency_present);
        assert_eq!(backoff.effective_deadline, backoff.original_deadline);
        if queued_while_inflight {
            assert_eq!(backoff.pending_revision, Some(series.watermarks.last().unwrap().cloud_revision));
        } else {
            assert_eq!(backoff.pending_revision, Some(series.partial.cloud_revision));
        }
        assert_retained(&harness, &retained, path_revision);
        let latest = series.watermarks.last().unwrap();
        let retry = started(&probe, latest.cloud_revision).await;
        assert!(retry.at >= backoff.original_deadline,
            "same O watermark worker started before the exact established deadline");
        let latest_ingress = probe.events.lock().unwrap().ingress.iter()
            .find(|e| e.revision == latest.cloud_revision).copied().unwrap();
        assert_eq!(retry.received_at, latest_ingress.received_at);
        assert_eq!(probe.count(series.partial.cloud_revision), 1);
        assert_eq!(probe.count(latest.cloud_revision), 1, "latest pending must eventually start");
        for skipped in &series.watermarks[..series.watermarks.len() - 1] {
            assert_eq!(probe.count(skipped.cloud_revision), 0, "older pending root must not be resurrected");
        }
        assert_retained(&harness, &retained, path_revision);
        harness.cancel(&goal).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
    }).catch_unwind().await;
    cleanup_and_resume(&mut harness, business).await;
}

async fn complete_successor(queued_while_inflight: bool) {
    let mut harness = start_reader_harness().await;
    let business = std::panic::AssertUnwindSafe(async {
        let (series, retained, goal, path_revision) = active(&mut harness).await;
        let probe = Arc::clone(&harness.hub.reader_guard);
        let gate = queued_while_inflight.then(|| ReturnGate::arm(&probe, series.partial.cloud_revision));
        let receipt = ingress(&harness, &series.partial).await;
        assert_real_failure(finished(&probe, series.partial.cloud_revision).await, receipt, series.missing);
        let before = if queued_while_inflight { None }
            else { Some(decision(&probe, series.partial.cloud_revision).await) };
        assert_retained(&harness, &retained, path_revision);
        let successor_receipt = ingress(&harness, &series.complete).await;
        if let Some(backoff) = before {
            assert!(successor_receipt.at < backoff.original_deadline, "fixture missed original retry interval");
            assert!(successor_receipt.retry_after < backoff.original_deadline);
            assert!(!successor_receipt.dependency_present);
        } else {
            assert!(successor_receipt.in_flight);
        }
        drop(gate);
        let backoff = decision(&probe, series.partial.cloud_revision).await;
        assert_eq!(backoff.received_at, receipt.received_at);
        assert_eq!(backoff.missing, Some(series.missing));
        assert!(backoff.retained);
        if queued_while_inflight {
            assert_eq!(backoff.pending_revision, Some(series.complete.cloud_revision));
            assert!(backoff.effective_deadline < backoff.original_deadline);
            assert!(!backoff.dependency_present);
        } else {
            assert_eq!(backoff.effective_deadline, backoff.original_deadline);
            assert!(backoff.dependency_present);
        }
        let attempt = started(&probe, series.complete.cloud_revision).await;
        assert!(attempt.at < backoff.original_deadline,
            "complete successor inherited the failed root's exact one-second deadline");
        assert_eq!(attempt.received_at, successor_receipt.received_at);
        let current = installed_revision(&harness, series.complete.cloud_revision).await;
        let built = finished(&probe, series.complete.cloud_revision).await;
        assert!(built.success && !built.awaiting && built.missing.is_none());
        assert_eq!(current.received_at, successor_receipt.received_at);
        assert_eq!(current.source_timestamp_ns, series.complete.source_timestamp_ns);
        assert_eq!(current.authority, map_authority(&series.complete));
        assert_eq!(probe.count(series.partial.cloud_revision), 1);
        assert_eq!(probe.count(series.complete.cloud_revision), 1);
        harness.cancel(&goal).await;
        harness.terminal(TerminalOutcome::Cancelled).await;
    }).catch_unwind().await;
    cleanup_and_resume(&mut harness, business).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_reader_same_missing_watermark_after_failure_keeps_original_deadline() {
    same_missing_watermarks(false).await;
}
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_reader_same_missing_watermark_pending_inflight_keeps_original_deadline() {
    same_missing_watermarks(true).await;
}
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_reader_complete_successor_after_failure_starts_before_original_deadline() {
    complete_successor(false).await;
}
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn real_reader_complete_successor_pending_inflight_starts_before_original_deadline() {
    complete_successor(true).await;
}
