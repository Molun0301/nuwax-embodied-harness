// OUT-OF-REPOSITORY Q2 DRAFT. Include AFTER stack_n2 in the same existing
// surface_selective_refresh_tests module. The two tests run in separate processes.
use nuwax_nav_runtime::route_pending_only_nohold_qualification::{Q2NoHoldCase, Q2Report};

struct Q2PreparedSource {
    cache: SurfaceGeometryCache,
    fixture: FineFixture,
    initial: N2Publication,
    s1: N2Publication,
    s2: N2Publication,
}
fn q2_prepare_source(setup: &SurfaceNavigationConfig) -> Result<Q2PreparedSource, String> {
    let mut cache = SurfaceGeometryCache::configured(setup);
    let mut fixture = scope_floor();
    let _floor = n2_build(&mut cache, &mut fixture)?;
    fixture.cells.extend((280..=283).flat_map(|x| (0..=42).flat_map(move |y| (44..=54).map(move |z| [x,y,z]))));
    let initial = n2_build(&mut cache, &mut fixture)?;
    let s1 = n2_build(&mut cache, &mut fixture)?;
    if !matches!(s1.update.changes.as_ref(), SurfaceGeometryChanges::Empty)
        || !nuwax_nav_pct::same_source_geometry(&initial.update.snapshot, &s1.update.snapshot)
        || s1.update.change_basis.as_ref().is_none_or(|basis|
            !nuwax_nav_pct::same_source_geometry(&initial.update.snapshot, basis)) {
        return Err("S1 is not an actual linked S0 Empty successor".into());
    }
    fixture.cells.insert(SCOPE_DETOUR_HIT);
    let s2 = n2_build(&mut cache, &mut fixture)?;
    if s1.meta.source_ns <= initial.meta.source_ns || s2.meta.source_ns <= s1.meta.source_ns
        || s2.update.change_basis.as_ref().is_none_or(|basis|
            !nuwax_nav_pct::same_source_geometry(&s1.update.snapshot, basis)) {
        return Err("Q actual Writer predecessor/source chain is invalid".into());
    }
    let SurfaceGeometryChanges::Columns { fine_size, fine, planner_size, coarse } = s2.update.changes.as_ref()
        else { return Err("S2 did not produce actual complete Columns".into()); };
    if fine_size.to_bits() != s2.update.snapshot.source_voxel_size_m().to_bits()
        || planner_size.to_bits() != s2.update.snapshot.config().voxel_size.to_bits()
        || fine.as_slice() != [[283,47]] {
        return Err("S2 changed fixed fine point or dimensions".into());
    }
    let xyz = SCOPE_DETOUR_HIT.map(|v| ((f64::from(v) + 0.5) * f64::from(*fine_size)) as f32);
    let q = xyz.map(|v| (v / *planner_size).floor() as i32);
    if coarse.as_slice() != [[q[0],q[1]]] { return Err("S2 actual coarse columns differ".into()); }
    if s2.update.snapshot.observations().and_then(|o| o.get(SCOPE_DETOUR_HIT))
        .is_none_or(|o| o.observed_ns != s2.meta.observed_ns) {
        return Err("S2 fixed point observation not from actual Writer acquisition".into());
    }
    Ok(Q2PreparedSource { cache, fixture, initial, s1, s2 })
}

fn q2_run_case(arm: N2Arm) -> Result<Q2Report, String> {
    let setup = configured(true, true);
    eprintln!("Q2_CONFIG scan={:?} controller_frequency_hz={} max_observation_age_sec={:?} motion_limits={:?} original_Q1_wall_route_points=true physical_closed_loop=false scheduler_pose_ttl=NOT_EXERCISED scheduler_costmap_ttl=NOT_EXERCISED scheduler_path_wait=NOT_EXERCISED",
        setup.follower.scan, setup.follower.controller.control_frequency,
        setup.follower.max_observation_age_sec, setup.follower.motion_limits);
    let Q2PreparedSource { cache, fixture, initial, s1, s2 } = q2_prepare_source(&setup)?;
    let mut case = Q2NoHoldCase::prepare(arm, setup.follower,
        initial.update.snapshot.clone(), &initial.meta, scope_pose(12.55), scope_route(), scope_pose(30.55))?;
    // The cache already contains S2. Do not run this producer at t0+200:
    // doing so would leak S2 geometry into a purported S0/S1 Empty.
    let (mut producer, receive) = N2SourceProducer::spawn(cache, fixture, s2.update.snapshot.clone());
    let work = (|| -> Result<(), String> {
        let t0 = case.begin()?;
        // N2 producer's first acquisition is start + 200ms. Setting its fixed
        // source cadence origin to t0+200 yields first=400, then600/800... .
        // This does not alter controller/measurement origin or source timestamps.
        producer.begin(t0 + N2_MAP_PERIOD)?;
        let s1_due = t0 + Duration::from_millis(250);
        let s2_due = t0 + Duration::from_millis(275);
        let until = t0 + N2_WINDOW;
        let mut s1 = Some(s1);
        let mut s2 = Some(s2);
        let mut pose_due = t0 + N2_POSE_PERIOD;
        let mut control_due = t0 + N2_CONTROL_PERIOD;
        let mut pose_x = 12.55;
        let mut queued: Option<(Instant, N2Publication)> = None;
        loop {
            let now = Instant::now();
            if now >= until { break; }
            // There is one existing bounded producer channel and at most one
            // delivery item, just as N/R11; no second request/event queue.
            if s2.is_none() {
                if let Some((due, publication)) = queued.take() { case.publish_empty(publication, due)?; }
                for _ in 0..16 {
                    match receive.try_recv() {
                        Ok(Ok((due, publication))) => case.publish_empty(publication, due)?,
                        Ok(Err(e)) => return Err(e),
                        Err(n2_mpsc::TryRecvError::Empty) => break,
                        Err(n2_mpsc::TryRecvError::Disconnected) => return Err("source producer disconnected during Q measurement".into()),
                    }
                }
            }
            if Instant::now() >= until { break; }
            // Preserve scheduled input order even when execution is late. Never
            // wait for entry/return/compute, synthesize an extra step, or shift t0.
            let first_input = pose_due.min(control_due)
                .min(if s1.is_some() { s1_due } else { until })
                .min(if s2.is_some() { s2_due } else { until });
            if s1.is_some() && s1_due <= first_input && now >= s1_due {
                case.publish_s1(s1.take().unwrap(), s1_due)?;
                case.move_pose_once(scope_pose(13.55))?;
                pose_x = 13.55;
                if pose_due == s1_due {
                    let skipped = n2_advance_due(&mut pose_due, N2_POSE_PERIOD, Instant::now());
                    case.skipped_pose(skipped);
                }
                if Instant::now() >= s2_due { case.note_uncovered("S1_POSE_DELIVERY_MISSED_275_WINDOW".into()); }
                continue;
            }
            if s2.is_some() && s2_due <= first_input && now >= s2_due {
                // update returns -> original D.finished -> exactly one probe ->
                // normal loop. No step/take/consume/schedule in this interval.
                case.publish_s2(s2.take().unwrap(), s2_due)?;
                if Instant::now() >= t0 + Duration::from_millis(300) {
                    case.note_uncovered("S2_OR_PROBE_MISSED_ORIGINAL_300_DUE".into());
                }
                continue;
            }
            if pose_due <= control_due && now >= pose_due {
                case.update_pose(scope_pose(pose_x))?;
                let skipped = n2_advance_due(&mut pose_due, N2_POSE_PERIOD, Instant::now());
                case.skipped_pose(skipped);
                continue;
            }
            if now >= control_due {
                case.tick(control_due)?;
                let skipped = n2_advance_due(&mut control_due, N2_CONTROL_PERIOD, Instant::now());
                case.skipped_control(skipped);
            }
            case.drain_events()?;
            if case.endpoints_seen() { break; }
            let mut next = pose_due.min(control_due).min(until);
            if s1.is_some() { next = next.min(s1_due); }
            if s2.is_some() { next = next.min(s2_due); }
            match receive.recv_timeout(next.saturating_duration_since(Instant::now())) {
                Ok(Ok(item)) => {
                    if queued.is_some() { return Err("source observation delivery item unexpectedly occupied".into()); }
                    queued = Some(item);
                }
                Ok(Err(e)) => return Err(e),
                Err(n2_mpsc::RecvTimeoutError::Timeout) => {}
                Err(n2_mpsc::RecvTimeoutError::Disconnected) => return Err("source producer ended before Q measurement".into()),
            }
        }
        Ok(())
    })();
    // Every post-spawn Result path reaches these, including begin failure.
    // Stop producer (wake recv_timeout/start) and JOIN before dropping follower.
    // End measurement before cleanup; retain late kernel returns separately.
    case.end_measurement();
    if let Err(e) = work { case.note_issue(e); }
    if let Err(e) = producer.finish() { case.note_issue(e); }
    Ok(case.finish())
}

fn q2_print_report(report: &Q2Report) {
    n2_print_report(&report.common);
    let origin = report.common.origin;
    for (label, step) in [("P250", report.step250), ("REFRESH300", report.step300)] {
        if let Some(step) = step {
            eprintln!("Q2_STEP label={label} due_ns={} entered_ns={} returned_ns={} accepted_delta={:?} before_retry_ns={} after_retry_ns={} before={:?} after={:?}",
                n2_offset(step.due, origin), n2_offset(step.entered, origin), n2_offset(step.returned, origin),
                step.after.next.checked_sub(step.before.next),
                n2_offset(step.before.retry_after, origin), n2_offset(step.after.retry_after, origin), step.before, step.after);
        }
    }
    if let Some(probe) = report.pending_observation {
        eprintln!("Q2_PENDING_OBSERVATION expected_id={} expected_epoch={} before_ns={} after_ns={} elapsed_ns={} queued_state={:?} original_exchange_lock_count=1 no_fixture_consumer_between_S2_probe_and_next_step=true P_dequeue_time=UNOBSERVED owner_replace_linearization_observed=false",
            probe.expected_id, probe.expected_epoch, n2_offset(probe.before, origin), n2_offset(probe.after, origin),
            probe.after.duration_since(probe.before).as_nanos(), probe.queued_state);
    }
    eprintln!("Q2_TARGET arm={:?} s1_source_ns={:?} s1_finished_ns={:?} original_p_receipt_id_epoch_retry_ns={:?} before_s2={:?} after_s2={:?} target_uncovered={:?} indirect_receipt_state_only=true direct_retired_return_observed=false actual_entry_receipt_pairing_required=true owner_publish_timestamp=UNOBSERVED owner_dequeue_P_timestamp=UNOBSERVED performance_acceptance=false",
        report.common.arm, report.s1_source_ns, report.s1_finished.map(|at| n2_offset(at, origin)),
        report.p_receipt.map(|(id, epoch, retry)| (id, epoch, n2_offset(retry, origin))),
        report.before_s2, report.after_s2, report.target_uncovered_reasons());
}

#[test]
#[ignore = "one separate fixed Q2 ABBA observation; no hold, rerun, or performance PASS"]
fn scope_pending_only_v2_q_no_hold_fixed_250_275() {
    let arm = n2_arm();
    let observed_tick = n2_observed_clock_tick();
    eprintln!("Q2_MANIFEST arm={arm:?} control_ms=50 pose_ms=25 original_S1_due_ms=250 original_S2_due_ms=275 first_post_S2_Empty_due_ms=400 next_Empty_period_ms=200 observation_s=30 min_observed_clock_delta={observed_tick:?} no_in_test_reruns=true actual_source_prebuilt=true raw_source_clock_unchanged=true first_dequeue_probe_disabled=true");
    let report = q2_run_case(arm).unwrap();
    q2_print_report(&report);
    // Natural target misses do not panic, rerun or erase real recovery endpoints.
    // Integrity failures remain failures. N-specific epoch/cost/single-ticket
    // checks deliberately do not apply to the separate Q process.
    let integrity = report.validate_integrity();
    assert!(integrity.is_ok(), "Q2 integrity failed: {integrity:?}");
    eprintln!("Q2_PROCESS_COMPLETE arm={arm:?} target_uncovered={:?} cleanup_returned={} no_equivalence_claim=true no_cpu_claim=true hardware_delivery_not_claimed=true",
        report.target_uncovered_reasons(), report.common.cleanup_returned);
}
