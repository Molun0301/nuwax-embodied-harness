// Opt-in fixed-input characterization. No budget/heuristic overrides, capability
// changes, planner copies, path replay, or extra geometry calls for observation.
mod phase_qualification {
    use super::*;

    fn report(
        label: &str,
        entry: &str,
        qualified: bool,
        result: &Result<LocalTrajectory, LocalPlanError>,
        phases: &[profile::PhaseRecord],
        work: (usize, usize, usize),
    ) {
        assert!(!phases.is_empty(), "PROFILE_DISABLED: use the explicit Scan profile feature");
        let phase_calls = std::array::from_fn::<_, 3, _>(|i| phases.iter().map(|p| p.calls[i]).sum::<u64>());
        assert_eq!(phase_calls, profile::calls(), "phase partition must not overlap or lose geometry calls");
        let path_kind = match result { Ok(path) if path.partial => "partial", Ok(_) => "full", Err(_) => "failure" };
        eprintln!("PHASE_QUALIFICATION label={label} entry={entry} qualification={} path_kind={path_kind} expanded={} nodes={} frontier_peak={} result={:?} phases={phases:?}",
            if qualified {"QUALIFIED"} else {"NOT_QUALIFIED"}, work.0, work.1, work.2,
            result.as_ref().map(|path| (path.poses.len(), path.poses.last())).map_err(|error| (error.kind, &error.reason)));
        // A passing characterization test means observations are coherent. It
        // does not turn NOT_QUALIFIED into a positive path/phase witness.
    }

    fn run_public(
        map: &SurfaceSnapshot, tomo: &Tomogram, start: BodyPose, reference: &[[f64; 3]],
        arrival: Option<([f64; 3], f64)>, boundary: &[[f64; 2]],
        progress: Option<ProgressPrefixPolicy<'_>>,
    ) -> (Result<LocalTrajectory, LocalPlanError>, Vec<profile::PhaseRecord>, (usize, usize, usize)) {
        let config = ScanConfig::default();
        assert_eq!(config.search_expansion_limit, 256);
        local_search::reset_last_work();
        let result = plan_with_progress(map, tomo, start, reference, &motion(), &config,
            None, arrival, boundary, false, false, progress);
        let phases = profile::take_phases();
        (result, phases, local_search::last_work())
    }

    #[test]
    #[ignore = "explicit phase qualification; requires Scan profile; no timing threshold"]
    fn open_straight_public_complete_phase() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let start = ([-1.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let reference = [[-1.0, 0.2, 0.0], [0.1, 0.2, 0.0]];
        let (result, phases, work) = run_public(&map, &tomo, start, &reference,
            Some(([0.1, 0.2, 0.4], 0.1)), &[], None);
        let qualified = result.as_ref().is_ok_and(|p| !p.partial)
            && phases.iter().any(|p| p.phase == "quick_direct" && p.complete)
            && !phases.iter().any(|p| p.phase == "root_connector" || p.phase == "search_remaining");
        report("open_straight", "scan_public", qualified, &result, &phases, work);
    }

    #[test]
    #[ignore = "explicit phase qualification; observes public precedence without disabling rotation"]
    fn open_arc_target_public_entry_classification() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let heading = 0.5_f64;
        let start = ([0.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let goal = [heading.cos(), 0.2 + heading.sin(), 0.4];
        let reference = [[0.0, 0.2, 0.0], [goal[0], goal[1], 0.0]];
        let (result, phases, work) = run_public(&map, &tomo, start, &reference,
            Some((goal, 0.001)), &[], None);
        let root_arc = phases.iter().any(|p| p.phase == "root_connector" && p.complete && p.selection == "arc");
        report("open_arc_target", "scan_public_root_arc_eligibility", root_arc, &result, &phases, work);
        // With current declared in-place rotation, an earlier direct connector
        // may succeed. Report that fact; do not disable it to force root arc.
    }

    #[test]
    #[ignore = "explicit private-search root-arc attribution; not a public-entry witness"]
    fn open_root_arc_existing_search_phase() {
        let map = coarse_open_floor();
        let tomo = Tomogram::from_surface(&map).unwrap();
        let heading = 0.5_f64;
        let start = ([0.0, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
        let goal = [heading.cos(), 0.2 + heading.sin(), 0.4];
        let reference = [[0.0, 0.2, 0.0], [goal[0], goal[1], 0.0]];
        let config = ScanConfig::default();
        let caps = motion();
        assert_eq!(config.search_expansion_limit, 256);
        local_search::reset_last_work();
        // The existing private function has no public Scan Scope. Supply only
        // its diagnostic counter scope, not a different planning algorithm.
        let scope = profile::Scope::begin(start, &reference, &caps, &config, None, Some((goal, 0.001)), false, false);
        let began = std::time::Instant::now();
        let result = local_search::plan_progress(&map, &tomo, start, &reference, &caps,
            None, Some((goal, 0.001)), &[], config.horizon_m, config.reference_corridor_m,
            false, config.search_expansion_limit, None).map(|path| LocalTrajectory {
                poses: path.poses, partial: path.partial, smoothed: false,
                elapsed_ms: began.elapsed().as_secs_f64() * 1000.0,
            });
        if result.is_ok() { profile::success(); }
        drop(scope);
        let phases = profile::take_phases();
        let work = local_search::last_work();
        let qualified = result.as_ref().is_ok_and(|path| !path.partial) && work.0 == 0
            && phases.iter().any(|p| p.phase == "root_connector" && p.complete && p.selection == "arc");
        report("open_root_arc", "existing_private_search", qualified, &result, &phases, work);
    }

    #[test]
    #[ignore = "explicit public-entry eligibility of existing one-expansion geometry"]
    fn one_centimetre_departure_public_phase_qualification() {
        // Exact existing matched one-expansion geometry and capabilities. The
        // original helper establishes a swept witness in its own existing test.
        let (map, start) = fine_escape_scene(false, [1, 0], false, 2);
        let tomo = Tomogram::from_surface(&map).unwrap();
        let goal = [start.0[0] + 0.01, start.0[1] + 0.5, start.0[2]];
        let reference = [[start.0[0], start.0[1], 0.0], [goal[0], goal[1], 0.0]];
        let (result, phases, work) = run_public(&map, &tomo, start, &reference,
            Some((goal, 0.015)), &[], None);
        let qualified = result.as_ref().is_ok_and(|p| !p.partial) && work.0 == 1
            && phases.iter().any(|p| p.phase == "root_connector" && !p.complete)
            && phases.iter().any(|p| p.phase == "search_remaining" && p.complete);
        report("one_cm_departure", "scan_public", qualified, &result, &phases, work);
    }

    #[test]
    #[ignore = "explicit default-256 partial qualification; not a proof of physical no-route"]
    fn partial_progress_default_budget_public_phase() {
        let (map, tomo, start, reference, boundary) = progress_prefix_fixture();
        let (result, phases, work) = run_public(&map, &tomo, start, &reference,
            Some(([1.0, 0.2, 0.4], 0.1)), &boundary,
            Some(ProgressPrefixPolicy { reference: &reference, minimum_distance_m: 0.375 }));
        let qualified = result.as_ref().is_ok_and(|p| p.partial) && work.0 == 256
            && phases.iter().any(|p| p.phase == "search_remaining" && p.exit == "partial");
        report("bounded_wall_partial", "scan_public_only_not_kernel_detour", qualified, &result, &phases, work);
        // This measures the nominal public Scan at the unchanged default budget.
        // It does not claim that kernel detour has no complete candidate.
    }
}
