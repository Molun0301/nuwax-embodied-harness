// Existing geometry, real Scan entry points. No changed budgets or alternate quick code.
mod reference_probe {
    use super::*;

    fn normal(outcome: Result<ReferenceCandidateOutcome, LocalPlanError>)
        -> Result<LocalTrajectory, LocalPlanError>
    {
        outcome.map(|outcome| match outcome {
            ReferenceCandidateOutcome::Complete(path) => path,
            ReferenceCandidateOutcome::Miss => panic!("expected Complete or a real error"),
        })
    }
    fn assert_same(a: &Result<LocalTrajectory, LocalPlanError>, b: &Result<LocalTrajectory, LocalPlanError>) {
        crate::polyline_reuse_tests::assert_same_result(a, b);
        if let (Ok(a), Ok(b)) = (a, b) { assert_eq!(a.partial, b.partial); }
    }
    fn audit(map: &SurfaceSnapshot, path: &LocalTrajectory, start: BodyPose,
        arrival: Option<([f64; 3], f64)>, final_yaw: Option<f64>, boundary: &[[f64; 2]])
    {
        assert!(!path.partial);
        assert_eq!(path.poses.first().unwrap().0.map(f64::to_bits), start.0.map(f64::to_bits));
        assert_eq!(path.poses.first().unwrap().1.map(f64::to_bits), start.1.map(f64::to_bits));
        for pair in path.poses.windows(2) { map.validate_body_motion(pair[0], pair[1], boundary).unwrap(); }
        let end = *path.poses.last().unwrap();
        if let Some((goal, tolerance)) = arrival { assert!(distance3(end.0, goal) < tolerance); }
        if let Some(target) = final_yaw { assert!(angle(yaw(end.1) - target).abs() < 0.1); }
    }
    fn assert_no_search() {
        assert_eq!(local_search::last_work(), (0, 0, 0), "probe must not enter local search");
        #[cfg(feature="profile")]
        {
            let phases = profile::take_phases();
            assert!(!phases.iter().any(|p| matches!(p.phase,
                "search_prepare" | "root_connector" | "search_remaining")));
            let calls = std::array::from_fn::<_, 3, _>(|i| phases.iter().map(|p| p.calls[i]).sum::<u64>());
            assert_eq!(calls, profile::calls());
        }
    }
    #[test]
    fn quick_complete_matches_full_and_original_geometry() {
        let base = coarse_open_floor();
        let tomo = Tomogram::from_surface(&base).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let reference = [[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]];
        let config = ScanConfig::default();
        let caps = motion();
        for arrival in [None, Some(([0.1, 0.2, 0.4], 0.1))] {
            for follow_reference in [false, true] {
                let final_yaw = arrival.map(|_| 0.2);
                let full_map = base.fork_for_planning();
                let full = plan_with_progress(&full_map, &tomo, start, &reference, &caps, &config,
                    final_yaw, arrival, &[], follow_reference, false,
                    Some(ProgressPrefixPolicy { reference: &reference, minimum_distance_m: 0.375 }));
                let ordinary = plan(&base.fork_for_planning(), &tomo, start, &reference, &caps, &config,
                    final_yaw, arrival, &[], follow_reference, false);
                assert_same(&full, &ordinary);
                local_search::reset_last_work();
                let probe = normal(probe_reference_candidates(&base.fork_for_planning(), &tomo, start,
                    &reference, &caps, &config, final_yaw, arrival, &[], follow_reference, false,
                    Some(ProgressPrefixPolicy { reference: &reference, minimum_distance_m: 0.375 })));
                assert_no_search();
                assert_same(&full, &probe);
                audit(&base, probe.as_ref().unwrap(), start, arrival, final_yaw, &[]);
                if arrival.is_none() {
                    // Independent existing full-window contract, not just equality
                    // between two entries which share candidate generation.
                    let model = base.body_envelope().unwrap();
                    let length = reference.windows(2).map(|pair|
                        (pair[1][0]-pair[0][0]).hypot(pair[1][1]-pair[0][1])).sum();
                    let tangent = (reference[1][1]-reference[0][1])
                        .atan2(reference[1][0]-reference[0][0]);
                    assert!(local_search::reference_handoff(&base,*probe.as_ref().unwrap().poses.last().unwrap(),
                        &reference, model.body_to_support_z_m,
                        (config.reference_corridor_m*0.9).min(config.horizon_m*0.3),
                        (model.body_to_support_z_m+model.z_bounds_m[1])*0.5, length, tangent));
                }
            }
        }
    }
    #[test]
    fn reference_miss_stops_before_search_and_preserves_polyline_reuse() {
        let base = coarse_open_floor();
        let tomo = Tomogram::from_surface(&base).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let reference = [[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]];
        let boundary = [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]];
        for arrival in [None, Some(([0.1, 0.2, 0.4], 0.1))] {
            let caps = motion();
            let config = ScanConfig::default();
            let full = plan(&base.fork_for_planning(), &tomo, start, &reference, &caps, &config,
                None, arrival, &boundary, true, false);
            assert_eq!(full.as_ref().unwrap_err().kind, LocalPlanFailureKind::SearchExhausted);
            let progress_none = plan_with_progress(&base.fork_for_planning(), &tomo, start, &reference,
                &caps, &config, None, arrival, &boundary, true, false, None);
            assert_same(&full, &progress_none);
            local_search::reset_last_work();
            let probe = crate::polyline_reuse_tests::with_original(false, ||
                probe_reference_candidates(&base.fork_for_planning(), &tomo, start, &reference,
                    &caps, &config, None, arrival, &boundary, true, false, None));
            assert!(matches!(probe, Ok(ReferenceCandidateOutcome::Miss)), "{probe:?}");
            assert_eq!(crate::polyline_reuse_tests::projections(), if arrival.is_none() { 1 } else { 2 });
            #[cfg(feature="profile")]
            {
                let phases = profile::take_phases();
                assert_eq!(phases.last().unwrap().phase, "quick_polyline_arrival");
                assert_eq!(phases.last().unwrap().exit, "reference_miss");
                assert!(!phases.iter().any(|p| matches!(p.phase,
                    "search_prepare" | "root_connector" | "search_remaining")));
                let calls = std::array::from_fn::<_, 3, _>(|i| phases.iter().map(|p| p.calls[i]).sum::<u64>());
                assert_eq!(calls, profile::calls());
            }
            assert_eq!(local_search::last_work(), (0, 0, 0));
        }
    }
    #[test]
    fn reference_miss_can_discard_a_real_remaining_full() {
        // Same existing one-expansion scene as phase_qualification. This records
        // E's deliberate lost opportunity, without changing motion or budgets.
        let (base, start) = fine_escape_scene(false, [1, 0], false, 2);
        let tomo = Tomogram::from_surface(&base).unwrap();
        let goal = [start.0[0] + 0.01, start.0[1] + 0.5, start.0[2]];
        let reference = [[start.0[0], start.0[1], 0.0], [goal[0], goal[1], 0.0]];
        let config = ScanConfig::default();
        assert_eq!(config.search_expansion_limit, 256);
        let caps = motion();
        local_search::reset_last_work();
        let full = plan(&base.fork_for_planning(), &tomo, start, &reference, &caps, &config,
            None, Some((goal, 0.015)), &[], false, false).unwrap();
        assert_eq!(local_search::last_work().0, 1);
        audit(&base, &full, start, Some((goal, 0.015)), None, &[]);
        local_search::reset_last_work();
        let probe = probe_reference_candidates(&base.fork_for_planning(), &tomo, start, &reference,
            &caps, &config, None, Some((goal, 0.015)), &[], false, false, None);
        assert!(matches!(probe, Ok(ReferenceCandidateOutcome::Miss)), "{probe:?}");
        assert_no_search();
    }
    #[test]
    fn original_input_errors_and_cancellation_are_not_reference_misses() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let reference = [[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]];
        let config = ScanConfig::default();
        for variant in 0..5 {
            let mut caps = motion();
            if variant == 4 { caps.max_forward_mps = f64::NAN; }
            let input = if variant == 3 { &[][..] } else { &reference[..] };
            let policy = match variant {
                0 => Some(ProgressPrefixPolicy { reference: &reference, minimum_distance_m: f64::NAN }),
                1 => Some(ProgressPrefixPolicy { reference: &reference, minimum_distance_m: -1.0 }),
                2 => Some(ProgressPrefixPolicy { reference: &[], minimum_distance_m: 0.375 }),
                _ => None,
            };
            let full = plan_with_progress(&map, &tomo, start, input, &caps, &config,
                None, None, &[], false, false, policy);
            let probe = normal(probe_reference_candidates(&map, &tomo, start, input, &caps, &config,
                None, None, &[], false, false, policy));
            assert_eq!(full.as_ref().unwrap_err().kind, LocalPlanFailureKind::Unclassified);
            assert_same(&full, &probe);
        }
        for checkpoints in [0, 8, 80] {
            let cancelled = cancellation::after_checkpoints(checkpoints, ||
                probe_reference_candidates(&map, &tomo, start, &reference, &motion(), &config,
                    None, None, &[], false, false, None)).unwrap_err();
            assert_eq!(cancelled.kind, LocalPlanFailureKind::Cancelled);
            assert!(cancelled.closest_pose.is_none() && cancelled.blocked.is_none() && cancelled.body_witness.is_none());
        }
    }
    #[test]
    #[ignore = "existing default-256 partial fixture; explicit cost characterization"]
    fn default_partial_full_entry_retained_but_probe_stops_at_miss() {
        let (base, tomo, start, reference, boundary) = progress_prefix_fixture();
        let caps = motion();
        let config = ScanConfig::default();
        assert_eq!(config.search_expansion_limit, 256);
        let progress = Some(ProgressPrefixPolicy { reference: &reference, minimum_distance_m: 0.375 });
        local_search::reset_last_work();
        let full = plan_with_progress(&base.fork_for_planning(), &tomo, start, &reference,
            &caps, &config, None, Some(([1.0, 0.2, 0.4], 0.1)), &boundary, false, false, progress).unwrap();
        assert!(full.partial);
        assert_eq!(local_search::last_work().0, 256);
        for pair in full.poses.windows(2) { base.validate_body_motion(pair[0], pair[1], &boundary).unwrap(); }
        local_search::reset_last_work();
        let probe = probe_reference_candidates(&base.fork_for_planning(), &tomo, start, &reference,
            &caps, &config, None, Some(([1.0, 0.2, 0.4], 0.1)), &boundary, false, false, progress);
        assert!(matches!(probe, Ok(ReferenceCandidateOutcome::Miss)), "{probe:?}");
        assert_no_search();
    }
}
