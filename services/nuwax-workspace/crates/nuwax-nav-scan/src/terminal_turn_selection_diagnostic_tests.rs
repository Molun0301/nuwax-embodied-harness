//! P2 terminal-selection diagnostics, using real geometry and production search.
//! A successful public result closes the failure hypothesis for this fixture.
//! Only the existing zero-expansion diagnostic overrides a production resource limit.
//! All witnesses use the production projection, rotation, and complete body sweep.
use super::*;
use crate::{distance3, rotate, LocalPlanFailureKind, ScanConfig, DEFAULT_SEARCH_EXPANSION_LIMIT};
use nuwax_nav_mls_planner::SurfaceBodyEnvelope;

const GOAL: [f64; 3] = [0.355, 0.0, 0.4];
const TOLERANCE: f64 = 0.35;
const DEEP_X: f64 = 0.021;

fn fixture(center_blocked: bool) -> (SurfaceSnapshot, Tomogram, BodyPose, [[f64; 3]; 2]) {
    // Reuse fine_escape_scene's exact observed rear pins, body, and forward wall.
    // Start is after the pin's center column, so the tomogram center-line precheck
    // does not reject the connector merely because it crosses the pin column.
    let mut extra = Vec::new();
    for z in 25..55 {
        extra.push([-29, 6, z]);
        extra.push([-29, -7, z]);
        if center_blocked {
            for y in -50..20 {
                extra.push([37, y, z]);
            }
        }
    }
    let map = crate::tests::scene_with_extra_cells(None, 0.0, &extra)
        .with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![
                [-0.3, -0.0599],
                [0.3, -0.0599],
                [0.3, 0.0599],
                [-0.3, 0.0599],
            ],
            z_bounds_m: [-0.15, 0.15],
            body_to_support_z_m: 0.4,
            max_step_up_m: 0.0,
            max_step_down_m: 0.0,
            max_cross_slope_deg: 18.0,
            support_uncertainty_m: 0.0,
        })
        .unwrap();
    let start = ([-0.089, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]);
    let reference = [[start.0[0], start.0[1], 0.0], [GOAL[0], GOAL[1], 0.0]];
    let tomo = Tomogram::from_surface(&map).unwrap();
    (map, tomo, start, reference)
}

fn verify(
    map: &SurfaceSnapshot,
    start: BodyPose,
    path: &[BodyPose],
    goal: [f64; 3],
    final_yaw: Option<f64>,
) {
    assert_eq!(path.first(), Some(&start));
    for pose in path {
        map.validate_body_pose(pose.0, pose.1)
            .expect("returned standing pose");
    }
    for edge in path.windows(2) {
        map.validate_body_motion(edge[0], edge[1], &[])
            .expect("independent full-body resweep");
    }
    let end = *path.last().unwrap();
    assert!(
        distance3(end.0, goal) < TOLERANCE,
        "original 3D arrival ball"
    );
    if let Some(y) = final_yaw {
        assert!(
            angle(yaw(end.1) - y).abs() < 0.1,
            "original final-yaw contract"
        );
    }
}

fn report(label: &str, result: &Result<Vec<BodyPose>, LocalPlanError>) {
    let work = last_work();
    match result {
        Ok(path) => {
            let caps = crate::tests::motion();
            let (mut length, mut turn, mut time) = (0.0, 0.0, 0.0);
            for edge in path.windows(2) {
                let d = distance3(edge[0].0, edge[1].0);
                let a = angle(yaw(edge[1].1) - yaw(edge[0].1)).abs();
                length += d; turn += a;
                time += (d / caps.max_forward_mps).max(a / caps.max_angular_rps);
            }
            eprintln!("TERMINAL_RESULT label={label} success=true work={work:?} samples={} end={:?} length={length:.9} turn={turn:.9} nominal_time={time:.9}", path.len(), path.last());
        }
        Err(error) => eprintln!("TERMINAL_RESULT label={label} success=false work={work:?} kind={:?} closest={:?} error={error}", error.kind, error.closest_pose),
    }
}

fn witness(
    map: &SurfaceSnapshot,
    start: BodyPose,
    final_yaw: f64,
    center_blocked: bool,
) -> (BodyPose, BodyPose) {
    assert!(
        distance3(start.0, GOAL) >= TOLERANCE,
        "must start outside the ball"
    );
    map.validate_body_pose(start.0, start.1)
        .expect("initial standing pose");
    assert_eq!(
        map.validate_body_pose(GOAL, start.1).is_err(),
        center_blocked
    );
    let length = GOAL[0] - start.0[0];
    let in_goal = |p: BodyPose| distance3(p.0, GOAL) < TOLERANCE;
    // Actual production stop logic, not a hand-picked radius intersection.
    let line = probe_motion_until(map, start, length, 0.0, &[], in_goal)
        .expect("first entry's full incoming sweep");
    let first = *line.last().unwrap();
    assert!(in_goal(first));
    assert!(
        line[..line.len() - 1].iter().all(|p| !in_goal(*p)),
        "actual first sampled entry"
    );
    assert!(
        first.0[0] < DEEP_X,
        "otherwise this fixture does not isolate deeper selection"
    );
    let target_q = [0.0, 0.0, (final_yaw * 0.5).sin(), (final_yaw * 0.5).cos()];
    map.validate_body_pose(first.0, target_q)
        .expect("terminal endpoint stands; rejection must retain the full turn sweep");
    let mut rejected = vec![first];
    let failure = rotate(map, &mut rejected, final_yaw, &crate::tests::motion(), &[])
        .expect_err("both production rotation directions must fail at first entry");
    assert_eq!(
        rejected,
        vec![first],
        "failed rotation must not mutate the admitted prefix"
    );

    // Continue along exactly the same admitted straight line. No planner result is
    // consulted when constructing this deeper terminal witness.
    let mut path = vec![start];
    path.extend(line);
    path.extend(
        probe_motion(map, first, DEEP_X - first.0[0], 0.0, &[])
            .expect("same-line continuation inside the ball"),
    );
    let deep = *path.last().unwrap();
    assert!((deep.0[0] - DEEP_X).abs() < 1e-12 && in_goal(deep));
    rotate(map, &mut path, final_yaw, &crate::tests::motion(), &[])
        .expect("deeper complete terminal rotation");
    verify(map, start, &path, GOAL, Some(final_yaw));
    eprintln!("TERMINAL_WITNESS center_blocked={center_blocked} final_yaw={final_yaw:.9} first={first:?} deep={deep:?} first_turn_error={failure} samples={} resweep=pass", path.len());
    (first, deep)
}

fn run_search(
    map: &SurfaceSnapshot,
    tomo: &Tomogram,
    start: BodyPose,
    reference: &[[f64; 3]],
    final_yaw: Option<f64>,
    goal: [f64; 3],
) -> Result<Vec<BodyPose>, LocalPlanError> {
    reset_last_work();
    let config = ScanConfig::default();
    let result = plan(
        map,
        tomo,
        start,
        reference,
        &crate::tests::motion(),
        final_yaw,
        Some((goal, TOLERANCE)),
        &[],
        config.horizon_m,
        config.reference_corridor_m,
        false,
        config.search_expansion_limit,
    );
    if let Ok(path) = &result {
        let g = projected_cost(
            start,
            &path[1..],
            map.body_envelope().unwrap().body_to_support_z_m,
            tomo,
            reference,
            &crate::tests::motion(),
        );
        eprintln!("TERMINAL_PATH_COST source=local_search actual_projected_g={g:.9}");
    }
    result
}

#[test]
fn terminal_turn_selection_geometry_and_zero_expansion_diagnostic() {
    for center_blocked in [true, false] {
        let (map, tomo, start, reference) = fixture(center_blocked);
        for final_yaw in [std::f64::consts::FRAC_PI_2, -std::f64::consts::FRAC_PI_2] {
            let (first, deep) = witness(&map, start, final_yaw, center_blocked);
            let result = with_expansion_limit(0, || {
                run_search(&map, &tomo, start, &reference, Some(final_yaw), GOAL)
            });
            report(
                &format!("zero_first_entry_blocked_center={center_blocked}_yaw={final_yaw}"),
                &result,
            );
            assert_eq!(last_work().0, 0);
            match &result {
                // A success is a disproof of this analytic limitation, not a test failure.
                Ok(path) => verify(&map, start, path, GOAL, Some(final_yaw)),
                Err(error) => assert_eq!(error.kind, LocalPlanFailureKind::BudgetExhausted),
            }
            // Isolate selection from feasibility: the same goal, same body, same
            // center obstacle and same final yaw succeed from the witnessed deeper pose.
            let deep_result = with_expansion_limit(0, || {
                run_search(&map, &tomo, deep, &reference, Some(final_yaw), GOAL)
            });
            report("zero_deep_positive_control", &deep_result);
            verify(
                &map,
                deep,
                deep_result
                    .as_ref()
                    .expect("deeper terminal is fully legal"),
                GOAL,
                Some(final_yaw),
            );
            assert_eq!(last_work().0, 0);

            // Removing only the terminal-yaw request must retain first-entry success,
            // including when the click center is physically occupied.
            let no_yaw =
                with_expansion_limit(0, || run_search(&map, &tomo, start, &reference, None, GOAL));
            report("zero_no_final_yaw_positive_control", &no_yaw);
            let no_yaw = no_yaw.unwrap();
            verify(&map, start, &no_yaw, GOAL, None);
            assert!(distance3(no_yaw.last().unwrap().0, first.0) < 1e-12);
        }
    }
}

#[test]
fn terminal_turn_selection_original_budget_and_public_eligibility_diagnostic() {
    let config = ScanConfig::default();
    assert_eq!(
        config.search_expansion_limit,
        DEFAULT_SEARCH_EXPANSION_LIMIT
    );
    assert_eq!(config.search_expansion_limit, 256);
    let final_yaw = std::f64::consts::FRAC_PI_2;
    for center_blocked in [true, false] {
        let (map, tomo, start, reference) = fixture(center_blocked);
        witness(&map, start, final_yaw, center_blocked);
        let result = run_search(
            &map.fork_for_planning(),
            &tomo,
            start,
            &reference,
            Some(final_yaw),
            GOAL,
        );
        report(
            &format!("production_budget_center_blocked={center_blocked}"),
            &result,
        );
        if let Ok(path) = &result {
            verify(&map, start, path, GOAL, Some(final_yaw));
        } else {
            assert!(matches!(
                result.as_ref().unwrap_err().kind,
                LocalPlanFailureKind::BudgetExhausted | LocalPlanFailureKind::SearchExhausted
            ));
        }

        // Public-entry qualification is measured, never forced. In particular,
        // reachable center must be solved by the already-fixed projection branch.
        reset_last_work();
        let public = crate::plan_with_progress(
            &map.fork_for_planning(),
            &tomo,
            start,
            &reference,
            &crate::tests::motion(),
            &config,
            Some(final_yaw),
            Some((GOAL, TOLERANCE)),
            &[],
            false,
            false,
            None,
        );
        let entered_fallback = last_work().1 > 0;
        let public = public.map(|path| {
            assert!(!path.partial);
            path.poses
        });
        report(
            &format!("public_center_blocked={center_blocked}_fallback={entered_fallback}"),
            &public,
        );
        if let Ok(path) = &public {
            verify(&map, start, path, GOAL, Some(final_yaw));
            let g = projected_cost(
                start,
                &path[1..],
                map.body_envelope().unwrap().body_to_support_z_m,
                &tomo,
                &reference,
                &crate::tests::motion(),
            );
            eprintln!("TERMINAL_PATH_COST source=public actual_projected_g={g:.9}");
        } else {
            assert!(matches!(
                public.as_ref().unwrap_err().kind,
                LocalPlanFailureKind::BudgetExhausted | LocalPlanFailureKind::SearchExhausted
            ));
        }
        eprintln!(
            "TERMINAL_CONCLUSION center_blocked={center_blocked} public_success={} fallback_entered={entered_fallback} interpretation={}",
            public.is_ok(),
            if public.is_ok() { "public_failure_hypothesis_closed_for_fixture" }
            else if entered_fallback { "public_failure_observed_requires_review" }
            else { "ineligible_no_fallback" },
        );
        if !center_blocked {
            let path = public.expect("reachable reference endpoint is already handled");
            assert!(!entered_fallback, "do not re-report endpoint-priority work");
            assert!(distance3(path.last().unwrap().0, GOAL) < 1e-9);
        }
    }
}

#[test]
fn terminal_turn_selection_preserves_reached_and_3d_contracts() {
    let (map, tomo, start, reference) = fixture(true);
    let final_yaw = std::f64::consts::FRAC_PI_2;
    let (_, deep) = witness(&map, start, final_yaw, true);
    let mut terminal = vec![deep];
    rotate(&map, &mut terminal, final_yaw, &crate::tests::motion(), &[]).unwrap();
    let reached = *terminal.last().unwrap();
    let result = with_expansion_limit(0, || {
        run_search(&map, &tomo, reached, &reference, Some(final_yaw), GOAL)
    });
    assert_eq!(
        result.unwrap(),
        vec![reached],
        "fully arrived pose must not drive toward occupied center"
    );

    // On the only floor, the 0.5 m vertical difference alone exceeds the original
    // 0.35 m radius. Never replace the 3D goal test with the XY precheck's disk.
    let raised_goal = [GOAL[0], GOAL[1], 0.9];
    let raised = with_expansion_limit(0, || {
        run_search(&map, &tomo, deep, &reference, Some(final_yaw), raised_goal)
    });
    report("zero_unreachable_height_negative_control", &raised);
    assert!(raised.is_err());
    assert_eq!(
        raised.unwrap_err().kind,
        LocalPlanFailureKind::BudgetExhausted
    );
}
