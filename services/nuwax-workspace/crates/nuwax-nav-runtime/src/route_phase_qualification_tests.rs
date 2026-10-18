// First-request fixed-input observations. Existing strategy and fixture values
// stay unchanged; no owner gate, motion simulation, or repeated geometry replay.
mod route_phase_qualification {
    use super::*;

    fn first_request(case: Case, detour: bool, bounded_wall: bool) -> (bool, bool) {
        let mut follower = prepare_case(case);
        if bounded_wall {
            // Separate, predeclared dead-end input. The same original short
            // wall now spans this task boundary; never change it between arms.
            // No geometry/capability/budget changes or iterative fixture tuning.
            follower.configure_boundary(&[[-1.7, -0.2], [1.7, -0.2], [1.7, 0.5], [-1.7, 0.5]]);
        }
        let request = snapshot_request(&follower, detour);
        assert_eq!(request.scan.search_expansion_limit, 256);
        let mut kernel = PlanKernel::new(follower.config.clone());
        let cancel = crate::planning_owner::Cancellation {
            epoch: Arc::new(AtomicU64::new(1)), expected: 1,
        };
        let dispatch = tracing::Dispatch::new(probe::Subscriber);
        let (result, wall, audit) = tracing::dispatcher::with_default(&dispatch, ||
            compute_observed(&mut kernel, request, &cancel));
        assert!(!audit.scan_phases.is_empty(),
            "PROFILE_DISABLED: explicitly build the diagnostic with --features nuwax-nav-scan/profile; do not change release defaults");
        let observed_calls = std::array::from_fn::<_,3,_>(|i|
            audit.scan_phases.iter().map(|phase| phase.calls[i]).sum::<u64>());
        assert_eq!(observed_calls, audit.profile_calls,
            "phase counters must partition existing Scan calls without replay");
        assert_eq!(result.request.scan.search_expansion_limit, 256);
        assert!(audit.reused_failures.is_empty(), "qualification must use a fresh kernel");
        let kind = match &result.outcome { Ok(path) if path.partial => "partial", Ok(_) => "full", Err(_) => "failure" };
        let is_partial = result.outcome.as_ref().is_ok_and(|path| path.partial);
        let is_full = result.outcome.as_ref().is_ok_and(|path| !path.partial);
        let expected = if bounded_wall {
            if detour { !is_full } else { is_partial }
        } else if detour {
            result.outcome.as_ref().is_ok_and(|candidate| !candidate.partial)
                && audit.pct.iter().any(|work| work.stats.is_some())
                && audit.branches.iter().any(|branch| branch.detour && branch.follow_reference
                    && branch.reference_changed && branch.partial == Some(false))
        } else {
            result.outcome.as_ref().is_ok_and(|candidate| candidate.partial)
                && audit.pct.is_empty()
                && audit.branches.len() == 1 && !audit.branches[0].detour
        };
        if let Ok(candidate) = &result.outcome {
            assert_eq!(pose_bits(body(candidate.path.poses.first().expect("nonempty candidate"))),
                pose_bits(body(&result.request.pose)), "candidate must retain actual request start");
        }
        eprintln!("KERNEL_PHASE_QUALIFICATION case={case:?} detour={detour} bounded_wall={bounded_wall} qualification={} kind={kind} kernel_wall_ms={} source_pose={:?} progress={} end={} audit={audit:?}",
            if expected {"QUALIFIED"} else {"NOT_QUALIFIED"}, wall.as_secs_f64()*1000.0,
            body(&result.request.pose), result.request.progress, result.request.end);
        #[cfg(feature = "body-query-profile")]
        {
            use nuwax_nav_mls_planner::body_query_profile::{Counter, Record, Stage};
            // Serialize only after compute and capture have ended; no hot-path log.
            let record = |r: Record| serde_json::json!({
                "wall_nanos": r.wall_nanos, "calls": r.calls, "counters": r.counters.as_slice(),
            });
            let phases: Vec<_> = audit.scan_phases.iter().map(|p| serde_json::json!({
                "branch": p.branch,
                "branch_detour": audit.branches.get(p.branch).map(|b| b.detour),
                "phase": p.phase, "exit": p.exit, "selection": p.selection,
                "complete": p.complete, "scan_elapsed_ms": p.elapsed_ms,
                "scan_calls": p.calls, "scan_ms": p.milliseconds,
                "before": p.body_before.map(&record), "after": p.body_after.map(&record),
            })).collect();
            eprintln!("BODY_QUERY_PROFILE {}", serde_json::json!({
                "case": format!("{case:?}"), "request_detour": detour,
                "bounded_wall": bounded_wall, "expansion_limit": 256,
                "qualification": if expected {"QUALIFIED"} else {"NOT_QUALIFIED"},
                "kind": kind, "owner": "synchronous_plan_kernel_thread",
                "candidate_geometry_bits": result.outcome.as_ref().ok().map(|candidate| serde_json::json!({
                    "partial": candidate.partial,
                    "poses": candidate.path.poses.iter().map(|pose| pose_bits(body(pose))).collect::<Vec<_>>(),
                })),
                "duration": "instrumented_wall_not_cpu",
                "stage_labels": Stage::ALL.map(Stage::label),
                "counter_labels": Counter::ALL.map(Counter::label).as_slice(),
                "total": audit.body_total.map(record), "phases": phases,
            }));
        }
        // No current-pose/admission or real-time performance claim is made here.
        // The existing owner/control tests remain the source of those results.
        (is_partial, is_full)
    }

    #[test]
    #[ignore = "explicit wall phase diagnostic; requires nuwax-nav-scan/profile"]
    fn wide_initial_nominal_and_detour_phases() {
        for detour in [false, true] { first_request(Case::WideApproach, detour, false); }
    }
    #[test]
    #[ignore = "explicit wall phase diagnostic; requires nuwax-nav-scan/profile"]
    fn short_initial_nominal_and_detour_phases() {
        for detour in [true, false] { first_request(Case::ShortApproach, detour, false); }
    }

    #[test]
    #[ignore = "explicit separate bounded-wall partial-only eligibility; unchanged default 256"]
    fn bounded_wall_default_partial_only_phases() {
        let nominal = first_request(Case::WideApproach, false, true);
        let detour = first_request(Case::WideApproach, true, true);
        eprintln!("KERNEL_PHASE_PARTIAL_ONLY qualification={} nominal_partial={} detour_full={}",
            if nominal.0 && !detour.1 {"QUALIFIED"} else {"NOT_QUALIFIED"}, nominal.0, detour.1);
        // Finite observed branch outcomes are not a physical no-route proof.
        // NOT_QUALIFIED is reported as-is; do not tune the 256 budget or input.
    }
}
