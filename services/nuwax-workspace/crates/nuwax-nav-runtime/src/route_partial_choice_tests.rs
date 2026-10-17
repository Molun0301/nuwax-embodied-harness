// E selection-only tests: synthetic Full/probe outputs exercise the actual kernel branch,
// PCT call site, two failure slots and cancellation handling. They are NOT swept
// path witnesses, captured-input replays, admission tests, or performance claims.
mod partial_choice {
    use super::*;
    use nuwax_nav_scan::{LocalPlanFailureKind as Kind, LocalTrajectory};
    use std::collections::VecDeque;
    use std::sync::atomic::Ordering;

    struct Step {
        detour: bool,
        reference_only: bool,
        result: Result<Option<LocalTrajectory>, LocalPlanError>,
        cancel_after: Option<(Arc<AtomicU64>, u64)>,
    }
    fn poses(marker: f64) -> Vec<BodyPose> {
        vec![([marker, -0.0, 0.5], [-0.0, 0.0, 0.0, 1.0]),
            ([marker + 0.125, 0.25, 0.5], [0.0, 0.0, 0.0, 1.0])]
    }
    fn path(partial: bool, marker: f64) -> Result<LocalTrajectory, LocalPlanError> {
        Ok(LocalTrajectory { partial, poses: poses(marker), smoothed: false,
            elapsed_ms: marker + 0.125 })
    }
    fn error(kind: Kind, reason: &str) -> Result<LocalTrajectory, LocalPlanError> {
        Err(LocalPlanError { kind, closest_pose: None, reason: reason.into(),
            blocked: None, body_witness: None })
    }
    fn step(detour: bool, result: Result<LocalTrajectory, LocalPlanError>) -> Step {
        Step { detour, reference_only: false, result: result.map(Some), cancel_after: None }
    }
    fn cancel_step(detour: bool, result: Result<LocalTrajectory, LocalPlanError>, cancel: &Cancellation) -> Step {
        Step { detour, reference_only: false, result: result.map(Some), cancel_after: Some((cancel.epoch.clone(), cancel.expected + 1)) }
    }
    fn optional(result: Result<Option<LocalTrajectory>, LocalPlanError>) -> Step {
        Step { detour: true, reference_only: true, result, cancel_after: None }
    }
    fn optional_full(marker: f64) -> Step { optional(path(false, marker).map(Some)) }
    fn optional_miss() -> Step { optional(Ok(None)) }
    fn cancel_optional(result: Result<Option<LocalTrajectory>, LocalPlanError>, cancel: &Cancellation) -> Step {
        Step { cancel_after: Some((cancel.epoch.clone(), cancel.expected + 1)), ..optional(result) }
    }
    fn script(kernel: &mut PlanKernel, steps: Vec<Step>) {
        let mut steps = VecDeque::from(steps);
        kernel.scan_hook = Some(Box::new(move |detour, reference_only| {
            let step = steps.pop_front().expect("unexpected extra Scan branch");
            assert_eq!(detour, step.detour);
            assert_eq!(reference_only, step.reference_only, "optional partial fallback must stop before search");
            if let Some((epoch, next)) = step.cancel_after { epoch.store(next, Ordering::Release); }
            step.result
        }));
    }
    fn run(kernel: &mut PlanKernel, request: &PlanRequest, cancel: &Cancellation)
        -> (Result<Candidate, Vec<LocalPlanError>>, priority_probe::Frame)
    {
        priority_probe::begin();
        let result = kernel.compute_inner(request, cancel);
        (result, priority_probe::end())
    }
    fn branches(frame: &priority_probe::Frame) -> Vec<bool> {
        frame.branches.iter().map(|b| b.detour).collect()
    }
    fn assert_path(result: Result<Candidate, Vec<LocalPlanError>>, request: &PlanRequest,
        partial: bool, marker: f64)
    {
        let result = result.unwrap_or_else(|errors| panic!("expected candidate: {errors:?}"));
        assert_eq!(result.partial, partial);
        assert_eq!(result.elapsed_ms.to_bits(), (marker + 0.125).to_bits());
        assert_eq!(result.path.poses.iter().map(|p| pose_bits(body(p))).collect::<Vec<_>>(),
            poses(marker).into_iter().map(pose_bits).collect::<Vec<_>>());
        for pose in result.path.poses {
            assert_eq!(pose.frame_id, request.pose.frame_id);
            assert_eq!(pose.stamp, request.pose.stamp);
        }
    }
    fn cache(kernel: &PlanKernel) -> Vec<(bool, String)> {
        kernel.failed.iter().map(|f| (f.detour, f.failure.reason.clone())).collect()
    }

    #[test]
    fn nominal_partial_selects_detour_full() {
        let (mut kernel, request, cancel) = fixture();
        script(&mut kernel, vec![step(false, path(true, 10.0)), optional_full(20.0)]);
        let (result, work) = run(&mut kernel, &request, &cancel);
        assert_path(result, &request, false, 20.0);
        assert_eq!(branches(&work), [false, true]);
        assert_eq!(work.pct.len(), 1);
        assert!(kernel.failed.is_empty());
    }

    #[test]
    fn reference_miss_keeps_original_nominal_without_caching_failure() {
        let (mut kernel, request, cancel) = fixture();
        script(&mut kernel, vec![step(false, path(true, 10.0)), optional_miss()]);
        let (result, work) = run(&mut kernel, &request, &cancel);
        assert_path(result, &request, true, 10.0);
        assert_eq!(branches(&work), [false, true]);
        assert_eq!(work.pct.len(), 1);
        assert!(kernel.failed.is_empty());
        assert!(work.branches[1].reference_miss);
        assert!(work.branches[1].partial.is_none() && work.branches[1].failure.is_none());
    }

    #[test]
    fn nominal_partial_survives_probe_errors_without_caching() {
        for kind in [Kind::BudgetExhausted, Kind::SearchExhausted, Kind::Unclassified] {
            let (mut kernel, request, cancel) = fixture();
            script(&mut kernel, vec![step(false, path(true, 10.0)), optional(error(kind, "probe").map(Some))]);
            let (result, work) = run(&mut kernel, &request, &cancel);
            assert_path(result, &request, true, 10.0);
            assert_eq!(branches(&work), [false, true]);
            assert_eq!(work.pct.len(), 1);
            assert!(kernel.failed.is_empty(), "no probe error is a complete-search negative");
            assert_eq!(work.branches[1].failure, Some(kind));
            assert!(!work.branches[1].reference_miss);
        }
    }

    #[test]
    fn nominal_full_returns_without_detour_or_failure_cache_mutation() {
        let (mut kernel, request, cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        kernel.failed.retain(|f| f.detour);
        let before = cache(&kernel);
        script(&mut kernel, vec![step(false, path(false, 10.0))]);
        let (result, work) = run(&mut kernel, &request, &cancel);
        assert_path(result, &request, false, 10.0);
        assert_eq!(branches(&work), [false]);
        assert!(work.pct.is_empty() && work.reused_failures.is_empty());
        assert_eq!(cache(&kernel), before);
    }

    #[test]
    fn detour_only_request_retains_each_original_result() {
        for variant in 0..4 {
            let (mut kernel, mut request, cancel) = fixture();
            request.detour = true;
            let outcome = match variant {
                0 => path(false, 20.0), 1 => path(true, 20.0),
                2 => error(Kind::SearchExhausted, "detour"),
                _ => error(Kind::Cancelled, "detour cancelled"),
            };
            script(&mut kernel, vec![step(true, outcome)]);
            let (result, work) = run(&mut kernel, &request, &cancel);
            if variant < 2 { assert_path(result, &request, variant == 1, 20.0); }
            else {
                let errors = result.err().expect("original detour error");
                assert_eq!(errors.len(), 1);
                assert_eq!(errors[0].kind, if variant == 2 { Kind::SearchExhausted } else { Kind::Cancelled });
            }
            assert_eq!(branches(&work), [true]);
            assert_eq!(work.pct.len(), 1);
            assert_eq!(kernel.failed.len(), usize::from(variant == 2));
        }
    }

    #[test]
    fn nominal_error_retains_original_detour_success_or_error_order() {
        for variant in 0..4 {
            let (mut kernel, request, cancel) = fixture();
            let outcome = match variant {
                0 => path(false, 20.0), 1 => path(true, 20.0),
                2 => error(Kind::SearchExhausted, "detour"),
                _ => error(Kind::Cancelled, "detour cancelled"),
            };
            script(&mut kernel, vec![step(false, error(Kind::BudgetExhausted, "nominal")), step(true, outcome)]);
            let (result, work) = run(&mut kernel, &request, &cancel);
            if variant < 2 { assert_path(result, &request, variant == 1, 20.0); }
            else {
                let errors = result.err().expect("ordered errors");
                assert_eq!(errors.iter().map(|f| f.reason.as_str()).collect::<Vec<_>>(),
                    ["nominal", if variant == 2 { "detour" } else { "detour cancelled" }]);
            }
            assert_eq!(branches(&work), [false, true]);
            assert_eq!(work.pct.len(), 1);
            let mut expected = vec![(false, "nominal".into())];
            if variant == 2 { expected.push((true, "detour".into())); }
            assert_eq!(cache(&kernel), expected);
        }
    }

    #[test]
    fn nominal_partial_reuses_detour_failure_before_pct_without_rewriting_slots() {
        let (mut kernel, request, cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        // Keep both slots; only the nominal slot is deliberately a different key.
        kernel.failed[0].reference[0][0] += 0.01;
        let before = cache(&kernel);
        script(&mut kernel, vec![step(false, path(true, 10.0))]);
        let (result, work) = run(&mut kernel, &request, &cancel);
        assert_path(result, &request, true, 10.0);
        assert_eq!(branches(&work), [false]);
        assert!(work.pct.is_empty());
        assert_eq!(work.reused_failures, [(true, Kind::BudgetExhausted)]);
        assert_eq!(cache(&kernel), before);
    }

    #[test]
    fn probe_error_preserves_both_full_failure_slots_and_is_retried() {
        let (mut kernel, request, cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        for f in &mut kernel.failed { f.reference[0][0] += 0.01; }
        let before = cache(&kernel);
        script(&mut kernel, vec![step(false, path(true, 10.0)),
            optional(error(Kind::Unclassified, "probe error").map(Some)),
            step(false, path(true, 10.0)), optional(error(Kind::Unclassified, "probe error").map(Some))]);
        for _ in 0..2 {
            let (result, work) = run(&mut kernel, &request, &cancel);
            assert_path(result, &request, true, 10.0);
            assert_eq!(branches(&work), [false, true]);
            assert_eq!(work.pct.len(), 1);
            assert!(work.reused_failures.is_empty());
            assert_eq!(cache(&kernel), before);
        }
    }

    #[test]
    fn probe_miss_preserves_both_full_failure_slots_and_is_retried() {
        let (mut kernel, request, cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        for f in &mut kernel.failed { f.reference[0][0] += 0.01; }
        let before = cache(&kernel);
        script(&mut kernel, vec![step(false, path(true, 10.0)), optional_miss(),
            step(false, path(true, 10.0)), optional_miss()]);
        for _ in 0..2 {
            let (result, work) = run(&mut kernel, &request, &cancel);
            assert_path(result, &request, true, 10.0);
            assert_eq!(branches(&work), [false, true]);
            assert_eq!(work.pct.len(), 1);
            assert!(work.reused_failures.is_empty());
            assert!(work.branches[1].reference_miss);
            assert_eq!(cache(&kernel), before);
        }
    }

    #[test]
    fn cached_nominal_failure_still_allows_original_detour_partial() {
        let (mut kernel, request, cancel) = fixture();
        seed_exact_failures(&mut kernel, &request);
        kernel.failed.retain(|f| !f.detour);
        script(&mut kernel, vec![step(true, path(true, 20.0))]);
        let (result, work) = run(&mut kernel, &request, &cancel);
        assert_path(result, &request, true, 20.0);
        assert_eq!(branches(&work), [true]);
        assert_eq!(work.reused_failures, [(false, Kind::BudgetExhausted)]);
        assert_eq!(work.pct.len(), 1);
        assert_eq!(cache(&kernel), [(false, "exact cached failure".into())]);
    }

    #[test]
    fn explicit_cancelled_result_never_falls_back_or_enters_another_branch() {
        for detour_cancel in [false, true] {
            let (mut kernel, request, cancel) = fixture();
            let mut steps = Vec::new();
            if detour_cancel { steps.push(step(false, path(true, 10.0))); }
            steps.push(if detour_cancel { optional(error(Kind::Cancelled, "explicit cancel").map(Some)) }
                else { step(false, error(Kind::Cancelled, "explicit cancel")) });
            script(&mut kernel, steps);
            let (result, work) = run(&mut kernel, &request, &cancel);
            let errors = result.err().expect("cancellation cannot yield a partial");
            assert_eq!(errors.len(), 1);
            assert_eq!(errors[0].kind, Kind::Cancelled);
            assert_eq!(branches(&work), if detour_cancel { vec![false, true] } else { vec![false] });
            assert!(kernel.failed.is_empty());
        }
    }

    #[test]
    fn epoch_change_after_nominal_partial_prevents_detour_and_fallback() {
        let (mut kernel, request, cancel) = fixture();
        script(&mut kernel, vec![cancel_step(false, path(true, 10.0), &cancel)]);
        let (result, work) = run(&mut kernel, &request, &cancel);
        assert_eq!(result.err().expect("cancelled")[0].kind, Kind::Cancelled);
        assert_eq!(branches(&work), [false]);
        assert!(work.pct.is_empty() && kernel.failed.is_empty());
    }

    #[test]
    fn epoch_change_after_detour_cannot_publish_full_or_saved_partial() {
        for variant in 0..3 {
            let (mut kernel, request, cancel) = fixture();
            let outcome = match variant {
                0 => path(false, 20.0).map(Some), 1 => Ok(None),
                _ => error(Kind::Unclassified, "ordinary failure before observed cancel").map(Some),
            };
            script(&mut kernel, vec![step(false, path(true, 10.0)), cancel_optional(outcome, &cancel)]);
            let (result, work) = run(&mut kernel, &request, &cancel);
            assert_eq!(result.err().expect("cancelled")[0].kind, Kind::Cancelled);
            assert_eq!(branches(&work), [false, true]);
            // Probe errors never enter the complete-search cache, even when
            // an epoch transition is observed after the call.
            assert!(kernel.failed.is_empty());
        }
    }

    #[test]
    fn real_clear_nominal_full_keeps_zero_detour_work() {
        let (mut kernel, request, cancel) = fixture();
        assert!(kernel.scan_hook.is_none());
        let (result, work) = run(&mut kernel, &request, &cancel);
        let result = result.unwrap_or_else(|errors| panic!("real clear fixture: {errors:?}"));
        assert!(!result.partial);
        assert_eq!(branches(&work), [false]);
        assert!(work.pct.is_empty() && kernel.failed.is_empty());
        for pair in result.path.poses.windows(2) {
            request.source.validate_body_motion(body(&pair[0]), body(&pair[1]), &request.boundary).unwrap();
        }
    }
}
