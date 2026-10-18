// UNCOMPILED, OUT-OF-REPOSITORY N2 FIRST-BATCH DRAFT.
// Include after surface_route_change_scope_qualification.rs inside the SAME
// surface_selective_refresh_tests module. Reuses its real FineFixture and types.
// No production policy switch: NEH_PENDING_ONLY_V2_ARM selects expected assertions only.
use nuwax_nav_runtime::route_pending_only_nohold_qualification::{
    N2Arm, N2CaseKind, N2KernelEvent, N2NoHoldCase, N2Publication, N2Report,
    N2SourceMeta, N2_CONTROL_PERIOD, N2_MAP_PERIOD, N2_POSE_PERIOD, N2_WINDOW,
};
use std::sync::mpsc as n2_mpsc;

fn n2_arm() -> N2Arm {
    match std::env::var("NEH_PENDING_ONLY_V2_ARM").as_deref() {
        Ok("baseline") => N2Arm::Baseline,
        Ok("candidate") => N2Arm::Candidate,
        _ => panic!("set NEH_PENDING_ONLY_V2_ARM=baseline|candidate for the verified binary; this is not a policy switch"),
    }
}

// Quiet true Writer/cache build. All data comes from FineFixture::next; no
// synthesized Columns, timestamp rewriting, point movement, or budget change.
fn n2_build(cache: &mut SurfaceGeometryCache, fixture: &mut FineFixture)
    -> Result<N2Publication, String> {
    let acquisition_before = Instant::now();
    let message = fixture.next();
    let acquisition_after = Instant::now();
    let stamp = message.source_timestamp_ns;
    if stamp <= 0 || message.committed_map.source_timestamp_ns != stamp
        || i64::from(message.cloud.header.stamp_sec) * 1_000_000_000
            + i64::from(message.cloud.header.stamp_nsec) != stamp {
        return Err("original Writer message timestamp chain is inconsistent".into());
    }
    let root = nuwax_map_chunks::Manifest::decode(&message.fine.as_ref().ok_or("missing actual fine transfer")?.root)
        .map_err(|e| e.to_string())?;
    if root.source_timestamp_ns != stamp as u64 || root.revision != message.cloud_revision as u64
        || root.domain != schema::navigation_map_3d_domain(&message)
        || root.voxel_size_m.to_bits() != message.source_voxel_size.to_bits() {
        return Err("actual immutable root does not match original message".into());
    }
    let update = cache.build_with_changes(&message).map_err(|e| e.to_string())?;
    let observed_ns = update.snapshot.observations().and_then(|o| o.get([251,31,40]))
        .ok_or("actual source lacks fixed floor observation probe")?.observed_ns;
    if observed_ns != stamp as u64 { return Err("actual floor observation differs from Writer source timestamp".into()); }
    let built_at = Instant::now();
    Ok(N2Publication { update: scope_runtime(update), meta: N2SourceMeta { source_ns: stamp,
        revision: message.cloud_revision, observed_ns, acquisition_before, acquisition_after, built_at } })
}

fn n2_require_combined_columns(initial: &N2Publication, damage: &N2Publication) -> Result<(), String> {
    if damage.meta.source_ns <= initial.meta.source_ns { return Err("damage did not advance actual source time".into()); }
    let Some(basis) = &damage.update.change_basis else { return Err("damage lacks actual predecessor".into()); };
    if !nuwax_nav_pct::same_source_geometry(basis, &initial.update.snapshot) {
        return Err("damage basis is not original walled S0".into());
    }
    let SurfaceGeometryChanges::Columns { fine_size, fine, planner_size, coarse } = damage.update.changes.as_ref()
        else { return Err("combined fixed points did not produce complete real Columns".into()); };
    if fine_size.to_bits() != damage.update.snapshot.source_voxel_size_m().to_bits()
        || planner_size.to_bits() != damage.update.snapshot.config().voxel_size.to_bits() {
        return Err("fine/coarse dimensions do not match actual snapshot".into());
    }
    let expected_fine = BTreeSet::from([[31,31], [283,47]]);
    if fine.len() != 2 || fine.iter().copied().collect::<BTreeSet<_>>() != expected_fine {
        return Err(format!("actual same-batch fine columns differ: {fine:?}"));
    }
    let expected_coarse: BTreeSet<_> = [SCOPE_PREFIX, SCOPE_DETOUR_HIT].map(|cell| {
        let xyz = cell.map(|v| ((f64::from(v) + 0.5) * f64::from(*fine_size)) as f32);
        let q = xyz.map(|v| (v / *planner_size).floor() as i32);
        [q[0], q[1]]
    }).into_iter().collect();
    if coarse.len() != expected_coarse.len() || coarse.iter().copied().collect::<BTreeSet<_>>() != expected_coarse {
        return Err(format!("actual combined coarse columns differ: {coarse:?}"));
    }
    for cell in [SCOPE_PREFIX, SCOPE_DETOUR_HIT] {
        let observation = damage.update.snapshot.observations().and_then(|o| o.get(cell))
            .ok_or("fixed combined cell absent from actual Writer snapshot")?;
        if observation.observed_ns != damage.meta.source_ns as u64 {
            return Err("fixed combined cells did not share original Writer observation timestamp".into());
        }
    }
    Ok(())
}

struct N2PreparedSource {
    cache: SurfaceGeometryCache,
    fixture: FineFixture,
    initial: N2Publication,
    damage: N2Publication,
}
fn n2_prepare_source(setup: &SurfaceNavigationConfig) -> Result<N2PreparedSource, String> {
    let mut cache = SurfaceGeometryCache::configured(setup);
    let mut fixture = scope_floor();
    let _floor = n2_build(&mut cache, &mut fixture)?;
    fixture.cells.extend((280..=283).flat_map(|x| (0..=42).flat_map(move |y| (44..=54).map(move |z| [x,y,z]))));
    let initial = n2_build(&mut cache, &mut fixture)?;
    // Both original, immutable points enter ONE Writer publication.
    fixture.cells.extend([SCOPE_PREFIX, SCOPE_DETOUR_HIT]);
    let damage = n2_build(&mut cache, &mut fixture)?;
    n2_require_combined_columns(&initial, &damage)?;
    Ok(N2PreparedSource { cache, fixture, initial, damage })
}

#[test]
#[ignore = "N2 combined fixed-point geometry precheck; real kernel hold only here, never timing"]
fn scope_pending_only_v2_n_geometry_precheck() {
    let arm = n2_arm();
    let setup = configured(true, true);
    let mut prepared = n2_prepare_source(&setup).unwrap();
    let mut case = HeldKernelCase::capture(setup.follower.clone(), prepared.initial.update.snapshot.clone(),
        scope_pose(12.55), scope_route(), scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&case);
    assert!(Arc::ptr_eq(&case.actual().source, &prepared.initial.update.snapshot));
    let proof = scope_proof(&case, &prepared.damage.update);
    assert!(!proof.unknown);
    assert!(!proof.permits_path(&prepared.damage.update.snapshot, case.global_route()),
        "combined fixture must trigger the ORIGINAL global-related cancellation");
    assert!(!proof.permits_path(&prepared.damage.update.snapshot, &case.actual().path));
    let audit = case.audit_on(prepared.damage.update.snapshot.clone()).unwrap();
    scope_require_current_safety(&audit);
    assert!(!audit.latest_edges_valid && audit.first_new_edge_rejection.is_some(),
        "combined fixture must physically damage the actual real A, not only overlap its AABB");
    let before = case.lifecycle();
    case.publish(prepared.damage.update.clone()).unwrap();
    let after = case.lifecycle();
    assert!(after.epoch > before.epoch, "both pending-only arms retain original related cancel");
    let finished = case.finish().unwrap();
    assert_eq!(finished.lifecycle_before_cleanup.plans, 0);
    assert!(finished.cancelled_before_owner_return && !finished.completion_consumed);
    // Separate actual same-geometry latest-source recovery prerequisite. Obtain
    // new genuine Empty observations; do not change the held damage timestamp.
    let latest = n2_build(&mut prepared.cache, &mut prepared.fixture).unwrap();
    assert!(matches!(latest.update.changes.as_ref(), SurfaceGeometryChanges::Empty));
    assert!(nuwax_nav_pct::same_source_geometry(&prepared.damage.update.snapshot, &latest.update.snapshot));
    let recovery = HeldKernelCase::capture(setup.follower, latest.update.snapshot.clone(),
        scope_pose(12.55), scope_route(), scope_pose(30.55)).unwrap();
    scope_require_full_current_window(&recovery);
    let recovery_audit = recovery.audit_on(latest.update.snapshot).unwrap();
    scope_require_current_safety(&recovery_audit);
    assert!(recovery_audit.latest_edges_valid);
    let recovered = recovery.finish().unwrap();
    assert_eq!(recovered.lifecycle_before_cleanup.plans, 1);
    assert!(matches!(recovered.command_after_admission, Some(nuwax_nav_runtime::PlannerStep::Command(c)) if c != Twist2::ZERO));
    eprintln!("N2_PRECHECK arm={arm:?} fixture=N2_M1_M3_COMBINED_DAMAGE same_batch_writer=true original_points_unchanged=true actual_A_full=true actual_A_hit={:?} current_body_connector_stop_allowed=true latest_real_recovery_admitted=true all_precheck_times_excluded=true hardware_delivery_not_claimed=true", audit.first_new_edge_rejection);
}

// Only the source producer waits for a planned acquisition time; the planning
// worker is never held. This is a private test fixture, not a new runtime queue.
struct N2SourceProducer {
    stop: n2_mpsc::Sender<()>,
    start: Option<n2_mpsc::Sender<Instant>>,
    thread: Option<std::thread::JoinHandle<Result<(), String>>>,
}
impl N2SourceProducer {
    fn spawn(mut cache: SurfaceGeometryCache, mut fixture: FineFixture,
        previous: Arc<SurfaceSnapshot>) -> (Self, n2_mpsc::Receiver<Result<(Instant, N2Publication), String>>) {
        let (start, started) = n2_mpsc::channel::<Instant>();
        let (stop, stopped) = n2_mpsc::channel::<()>();
        let (send, receive) = n2_mpsc::sync_channel(16);
        let thread = std::thread::spawn(move || {
            let origin = started.recv().map_err(|_| "source producer start disconnected")?;
            let mut previous = previous;
            let mut due = origin + N2_MAP_PERIOD;
            loop {
                let wait = due.saturating_duration_since(Instant::now());
                match stopped.recv_timeout(wait) {
                    Ok(()) | Err(n2_mpsc::RecvTimeoutError::Disconnected) => return Ok(()),
                    Err(n2_mpsc::RecvTimeoutError::Timeout) => {}
                }
                let publication = n2_build(&mut cache, &mut fixture)?;
                if !matches!(publication.update.changes.as_ref(), SurfaceGeometryChanges::Empty)
                    || !nuwax_nav_pct::same_source_geometry(&previous, &publication.update.snapshot)
                    || publication.update.change_basis.as_ref().is_none_or(|basis|
                        !nuwax_nav_pct::same_source_geometry(&previous, basis)) {
                    return Err("continuation is not a real linked Empty observation".into());
                }
                previous = publication.update.snapshot.clone();
                send.try_send(Ok((due, publication))).map_err(|e| format!("source channel incomplete: {e}"))?;
                due += N2_MAP_PERIOD;
                // A slow source does not burst-rewrite timestamps to catch up.
                // The later actual acquisition and due stamp retain its jitter.
                while due <= Instant::now() { due += N2_MAP_PERIOD; }
            }
        });
        (Self { stop, start: Some(start), thread: Some(thread) }, receive)
    }
    fn begin(&mut self, t0: Instant) -> Result<(), String> {
        self.start.take().ok_or("source producer already started")?.send(t0).map_err(|e| e.to_string())
    }
    fn finish(&mut self) -> Result<(), String> {
        self.start.take(); // disconnect a producer if case begin failed
        let _ = self.stop.send(());
        match self.thread.take() {
            Some(thread) => thread.join().map_err(|_| "source producer panicked".to_string())?,
            None => Ok(()),
        }
    }
}
impl Drop for N2SourceProducer { fn drop(&mut self) { let _ = self.finish(); } }

fn n2_advance_due(due: &mut Instant, period: Duration, now: Instant) -> u64 {
    *due += period;
    let mut skipped = 0;
    while *due <= now { *due += period; skipped += 1; }
    skipped
}

fn n2_run_case(arm: N2Arm, kind: N2CaseKind) -> Result<N2Report, String> {
    let setup = configured(true, true);
    // This is before source preparation, owner creation, counter start and t0.
    eprintln!("N2_CONFIG case={kind:?} scan={:?} controller_frequency_hz={} max_observation_age_sec={:?} motion_limits={:?} original_configured_fixture=true scheduler_pose_ttl=NOT_EXERCISED scheduler_costmap_ttl=NOT_EXERCISED scheduler_path_wait=NOT_EXERCISED",
        setup.follower.scan, setup.follower.controller.control_frequency,
        setup.follower.max_observation_age_sec, setup.follower.motion_limits);
    let N2PreparedSource { cache, fixture, initial, damage } = n2_prepare_source(&setup)?;
    let mut case = N2NoHoldCase::prepare(arm, kind, setup.follower,
        initial.update.snapshot.clone(), &initial.meta, scope_pose(12.55), scope_route(), scope_pose(30.55))?;
    // Prepare a source producer before t0; it has no access to PlanningOwner.
    let (mut producer, receive) = N2SourceProducer::spawn(cache, fixture, damage.update.snapshot.clone());
    let work = (|| -> Result<(), String> {
        let t0 = case.begin()?;
        let damage_due = t0 + if kind == N2CaseKind::OnlyActive { N2_CONTROL_PERIOD } else { Duration::ZERO };
        let mut damage = Some(damage);
        // The next original input action remains immediate initial-slot damage.
        if kind == N2CaseKind::InitialSlotObservation {
            case.publish(damage.take().unwrap(), true, damage_due)?;
        }
        producer.begin(t0)?;
        let mut pose_due = t0 + N2_POSE_PERIOD;
        let mut control_due = t0 + N2_CONTROL_PERIOD;
        let until = t0 + N2_WINDOW;
        let mut queued: Option<(Instant, N2Publication)> = None;
        loop {
            let now = Instant::now();
            if now >= until { break; }
            if damage.is_some() && now >= damage_due {
                case.publish(damage.take().unwrap(), true, damage_due)?;
            }
            // Never install an Empty successor before its prebuilt D predecessor.
            if damage.is_none() {
                if let Some((due, publication)) = queued.take() { case.publish(publication, false, due)?; }
                for _ in 0..16 {
                    match receive.try_recv() {
                        Ok(Ok((due, publication))) => case.publish(publication, false, due)?,
                        Ok(Err(e)) => return Err(e),
                        Err(n2_mpsc::TryRecvError::Empty) => break,
                        Err(n2_mpsc::TryRecvError::Disconnected) => return Err("source producer disconnected during measurement".into()),
                    }
                }
            }
            if Instant::now() >= until { break; }
            if Instant::now() >= pose_due {
                // Genuine fresh sampling at this fixed pose, not a rewritten old
                // packet. No physical advancement or applied command is claimed.
                case.update_pose(scope_pose(12.55))?;
                let skipped = n2_advance_due(&mut pose_due, N2_POSE_PERIOD, Instant::now());
                case.skipped_pose(skipped);
            }
            if Instant::now() >= control_due {
                case.tick(control_due)?;
                let skipped = n2_advance_due(&mut control_due, N2_CONTROL_PERIOD, Instant::now());
                case.skipped_control(skipped);
            }
            case.drain_events()?;
            if case.endpoints_seen() { break; }
            let mut next = pose_due.min(control_due).min(until);
            if damage.is_some() { next = next.min(damage_due); }
            match receive.recv_timeout(next.saturating_duration_since(Instant::now())) {
                Ok(Ok(item)) => {
                    if queued.is_some() { return Err("source observation queue unexpectedly occupied".into()); }
                    queued = Some(item);
                }
                Ok(Err(e)) => return Err(e),
                Err(n2_mpsc::RecvTimeoutError::Timeout) => {}
                Err(n2_mpsc::RecvTimeoutError::Disconnected) => return Err("source producer ended before measurement".into()),
            }
        }
        Ok(())
    })();
    case.end_measurement();
    if let Err(e) = work { case.note_issue(e); }
    if let Err(e) = producer.finish() { case.note_issue(e); }
    Ok(case.finish())
}

fn n2_offset(at: Instant, origin: Instant) -> i128 {
    if at >= origin { at.duration_since(origin).as_nanos() as i128 }
    else { -(origin.duration_since(at).as_nanos() as i128) }
}
fn n2_observed_clock_tick() -> Option<Duration> {
    // A diagnostic lower observed positive delta, not a permissive latency bound.
    let mut minimum = None;
    let mut previous = Instant::now();
    for _ in 0..1024 {
        let now = Instant::now();
        let delta = now.duration_since(previous);
        if !delta.is_zero() { minimum = Some(minimum.map_or(delta, |old: Duration| old.min(delta))); }
        previous = now;
    }
    minimum
}
fn n2_print_report(report: &N2Report) {
    let origin = report.origin;
    let initial = &report.initial_source;
    eprintln!("N2_INITIAL_SOURCE case={:?} source_ns={} revision={} observed_ns={} acquisition_before_ns={} acquisition_after_ns={} built_at_ns={}",
        report.case, initial.source_ns, initial.revision, initial.observed_ns,
        n2_offset(initial.acquisition_before, origin), n2_offset(initial.acquisition_after, origin),
        n2_offset(initial.built_at, origin));
    for receipt in &report.receipts {
        eprintln!("N2_RECEIPT case={:?} at_ns={} id={} epoch={} source_ns={} source={:p} pose_stamp_ns={} detour={} generation={} progress={} after_damage={}",
            report.case, n2_offset(receipt.observed_at, origin), receipt.identity.id, receipt.epoch,
            receipt.source_ns, Arc::as_ptr(&receipt.identity.source), receipt.identity.pose.stamp.nanos,
            receipt.identity.detour, receipt.identity.generation, receipt.progress, receipt.after_damage);
    }
    if let Some(pop) = report.first_dequeue {
        eprintln!("N2_FIRST_DEQUEUE case={:?} id={} epoch={} before_ns={} after_ns={} observed_original_pending_take=true observer_affects_race=true",
            report.case, pop.id, pop.epoch, n2_offset(pop.before,origin), n2_offset(pop.after,origin));
    }
    for event in &report.kernel {
        match event {
            N2KernelEvent::Entry { at, identity, source_ns } => eprintln!(
                "N2_ENTRY case={:?} at_ns={} ordinal={} matched_request_id={:?} source_ns={:?} source={:p} pose_stamp_ns={} detour={} generation={} epoch={}",
                report.case, n2_offset(*at, origin), identity.compute_ordinal,
                report.entry_receipt(identity, *source_ns).map(|r| r.identity.id), source_ns,
                Arc::as_ptr(&identity.source), identity.pose.stamp.nanos, identity.detour, identity.generation, identity.epoch),
            N2KernelEvent::Return { at, ordinal, outcome, cancelled } => eprintln!(
                "N2_RETURN case={:?} at_ns={} ordinal={ordinal} outcome={outcome:?} cancelled={cancelled} during_cleanup={}",
                report.case, n2_offset(*at, origin), *at > report.ended),
        }
    }
    for map in &report.maps {
        eprintln!("N2_MAP case={:?} damage={} due_ns={} began_ns={} finished_ns={} source_ns={} revision={} observed_ns={} acquisition_before_ns={} acquisition_after_ns={} built_at_ns={} epoch_before={} epoch_after={} tickets_before={} tickets_after={} submitted_before={:?} submitted_after={:?}",
            report.case, map.damage, n2_offset(map.due, origin), n2_offset(map.began, origin), n2_offset(map.finished, origin),
            map.source.source_ns, map.source.revision, map.source.observed_ns,
            n2_offset(map.source.acquisition_before, origin), n2_offset(map.source.acquisition_after, origin),
            n2_offset(map.source.built_at, origin), map.epoch_before, map.epoch_after, map.tickets_before, map.tickets_after,
            map.submitted_before, map.submitted_after);
    }
    for tick in &report.controls {
        eprintln!("N2_CONTROL case={:?} due_ns={} entered_ns={} returned_ns={} elapsed_ns={} due_delay_ns={} kind={:?} admission={:?} generation={} plans={} tickets={} bound_present={} bound_deadline_ns={:?} bound_expired_at_return={} bound_expired_at_observation={} map_source_ns={} pose_stamp_ns={}",
            report.case, n2_offset(tick.due, origin), n2_offset(tick.entered, origin), n2_offset(tick.returned, origin),
            tick.returned.duration_since(tick.entered).as_nanos(), tick.entered.saturating_duration_since(tick.due).as_nanos(),
            tick.kind, tick.admission, tick.generation, tick.plans, tick.tickets, tick.bound_present,
            tick.bound_deadline.map(|at| n2_offset(at, origin)), tick.bound_expired_at_return, tick.bound_expired_at_observation,
            tick.map_source_ns, tick.pose_stamp_ns);
    }
    let latest_detail = report.first_latest_entry_detail();
    let latest = latest_detail.map(|(at, _)| at);
    let damage = report.damage();
    let nonzero = report.first_nonzero.as_ref();
    // A source timestamp is Unix time. Never subtract it from Instant. Actual
    // acquisition is bracketed by the two Instant reads around the true Writer
    // call; report both bounds and preserve the original Unix/source_ns too.
    let entry_source = latest_detail.and_then(|(_, receipt)| report.maps.iter()
        .find(|map| map.source.source_ns == receipt.source_ns));
    let entry_age = latest.zip(entry_source).map(|(at, map)|
        (n2_offset(at, map.source.acquisition_after), n2_offset(at, map.source.acquisition_before)));
    let nonzero_source = nonzero.and_then(|command| report.maps.iter()
        .find(|map| map.source.source_ns == command.map_source_ns));
    let nonzero_age = nonzero.zip(nonzero_source).map(|(command, map)|
        (n2_offset(command.at, map.source.acquisition_after), n2_offset(command.at, map.source.acquisition_before)));
    let newest_installed_at_entry = latest.and_then(|at| report.maps.iter()
        .filter(|map| map.finished <= at).max_by_key(|map| map.finished)).map(|map| map.source.source_ns);
    eprintln!("N2_SOURCE_AGE case={:?} selected_entry_source_ns={:?} entry_source_acquisition_age_ns_lower_upper={entry_age:?} newest_fully_installed_at_entry_source_ns={newest_installed_at_entry:?} first_nonzero_current_source_ns={:?} nonzero_current_source_acquisition_age_ns_lower_upper={nonzero_age:?} origin_damage_to_entry_ns={:?} origin_damage_acquisition_to_entry_ns_lower_upper={:?} origin_damage_acquisition_to_nonzero_ns_lower_upper={:?}",
        report.case, latest_detail.map(|(_, receipt)| receipt.source_ns), nonzero.map(|command| command.map_source_ns),
        latest.zip(damage).map(|(at, map)| n2_offset(at, map.finished)),
        latest.zip(damage).map(|(at, map)| (n2_offset(at, map.source.acquisition_after), n2_offset(at, map.source.acquisition_before))),
        nonzero.zip(damage).map(|(command, map)| (n2_offset(command.at, map.source.acquisition_after), n2_offset(command.at, map.source.acquisition_before))));
    eprintln!("N2_SUMMARY arm={:?} case={:?} end_ns={} observed_receipts={} replace_lock_attempts={:?} request_builder_calls={:?} observer_drops={} skipped_pose={} skipped_control={} latest_entry_ns={:?} installed_to_entry_ns={:?} first_checked_nonzero_ns={:?} installed_to_nonzero_ns={:?} nonzero_detail={nonzero:?} only_active_at_damage_start={} initial_pending_coverage={} max_control_ns={:?} max_due_delay_ns={:?} timed_out_or_missing_endpoint={} issues={:?} cleanup_returned={} no_hold=true actual_cpu=UNMEASURED bridge_delivery=UNMEASURED hardware=UNMEASURED acceptance_pass=false",
        report.arm, report.case, n2_offset(report.ended, origin), report.receipts.len(),
        report.costs_after.0.checked_sub(report.costs_before.0), report.costs_after.1.checked_sub(report.costs_before.1),
        report.observer_drops, report.skipped_pose_due, report.skipped_control_due,
        latest.map(|at| n2_offset(at, origin)), latest.zip(damage).map(|(at,m)| n2_offset(at,m.finished)),
        nonzero.map(|n| n2_offset(n.at,origin)), nonzero.zip(damage).map(|(n,m)| n2_offset(n.at,m.finished)),
        report.only_active_at_damage_start(), report.initial_pending_coverage(),
        report.controls.iter().map(|c| c.returned.duration_since(c.entered).as_nanos()).max(),
        report.controls.iter().map(|c| c.entered.saturating_duration_since(c.due).as_nanos()).max(),
        latest.is_none() || nonzero.is_none(), report.issues, report.cleanup_returned);
}

#[test]
#[ignore = "one ABBA process observation: fixed N2a plus initial-slot attempt; not acceptance or CPU PASS"]
fn scope_pending_only_v2_n_no_hold_two_cases() {
    let arm = n2_arm();
    let observed_tick = n2_observed_clock_tick();
    eprintln!("N2_MANIFEST arm={arm:?} control_ms=50 pose_ms=25 empty_map_ms=200 observation_s=30 source_prebuilt=true initial_pending_dequeue_observer=first_actual_pop_interval min_observed_clock_delta={observed_tick:?} four_process_ABBA_orchestration=external no_in_test_reruns=true");
    // Plain synchronous test: same control thread for both TLS snapshots, no
    // other follower computes requests on this thread inside either window.
    let first = n2_run_case(arm, N2CaseKind::OnlyActive).unwrap();
    n2_print_report(&first);
    let first_validation = first.validate_observations();
    assert!(first_validation.is_ok(), "N2a integrity failed: {first_validation:?}");
    let second = n2_run_case(arm, N2CaseKind::InitialSlotObservation).unwrap();
    n2_print_report(&second);
    let second_validation = second.validate_observations();
    // Basic integrity errors fail this one observation process. Missing natural
    // coverage/endpoints remain explicitly reported, never silently rerun.
    assert!(second_validation.is_ok(), "N2 initial-slot observation integrity failed: {second_validation:?}");
    eprintln!("N2_PROCESS_COMPLETE arm={arm:?} cases=2 only_active_covered={} initial_pending_coverage={} harness_integrity_checked=true performance_acceptance=false no_equivalence_claim=true no_cpu_claim=true",
        first.only_active_at_damage_start(), second.initial_pending_coverage());
}
