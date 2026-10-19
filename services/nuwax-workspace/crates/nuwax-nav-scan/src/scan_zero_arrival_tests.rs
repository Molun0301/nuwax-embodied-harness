// Tests-only draft, NOT installed, compiled, or run.
// Intended inclusion inside nuwax-nav-scan/src/lib.rs's existing `mod tests`:
//     include!("scan_zero_arrival_tests.rs");
// Reuses the existing real scene/motion fixtures; no production edits or budgets.

fn zero_arrival_fixture() -> (SurfaceSnapshot, Tomogram, BodyPose, [[f64; 3]; 1]) {
    let map = scene(None);
    let start = ([0.5, 0.2, 0.4], [0.0, 0.0, 0.0, 1.0]);
    map.validate_body_pose(start.0, start.1).expect("real valid initial body");
    let tomogram = Tomogram::from_surface(&map).expect("real existing scene tomogram");
    let reference = [[start.0[0], start.0[1], start.0[2] - 0.4]];
    (map, tomogram, start, reference)
}

fn assert_exact_stationary_arrival(map: &SurfaceSnapshot, start: BodyPose, path: &LocalTrajectory) {
    assert!(!path.partial, "original exact goal needs no progress-only prefix");
    assert_eq!(path.poses, vec![start], "an already exact pose needs no motion");
    for pose in &path.poses {
        map.validate_body_pose(pose.0, pose.1).expect("independent real pose check");
    }
    for edge in path.poses.windows(2) {
        map.validate_body_motion(edge[0], edge[1], &[]).expect("real full-body sweep");
    }
    // Empty edge set is intentional: this is arrival at a valid stationary pose,
    // not evidence of successful movement, braking, or real-robot execution.
}

#[test]
fn positive_arrival_exact_stationary_pose_control() {
    let (map, tomo, start, reference) = zero_arrival_fixture();
    let config = ScanConfig::default();
    assert_eq!(config.search_expansion_limit, DEFAULT_SEARCH_EXPANSION_LIMIT);
    let path = plan(&map, &tomo, start, &reference, &motion(), &config,
        Some(0.0), Some((start.0, 0.1)), &[], false, false).unwrap();
    assert_exact_stationary_arrival(&map, start, &path);
}

#[test]
fn zero_arrival_exact_stationary_pose_public_scan_must_complete() {
    let options = nuwax_nav_types_messages::GoalOptions {
        position_tolerance_m: 0.0, yaw_tolerance_rad: 0.2, allow_reverse: false,
    };
    options.validate().expect("zero is explicitly supported by original goal options");
    let (map, tomo, start, reference) = zero_arrival_fixture();
    let config = ScanConfig::default();
    assert_eq!(config.search_expansion_limit, DEFAULT_SEARCH_EXPANSION_LIMIT);
    let result = plan(&map, &tomo, start, &reference, &motion(), &config,
        Some(0.0), Some((start.0, options.position_tolerance_m)), &[], false, false);
    // Target RED: real public Full Scan currently has only distance < 0 success.
    // No failure kind or expansion count is guessed before the validator runs it.
    let path = result.expect("exact original zero-radius goal must remain reachable");
    assert_exact_stationary_arrival(&map, start, &path);
}

#[test]
fn zero_arrival_exact_stationary_pose_projection_must_complete() {
    let (map, _tomo, start, reference) = zero_arrival_fixture();
    let blocked = std::cell::Cell::new(None);
    let result = project(&map, start, &reference, &motion(), false,
        Some(0.0), Some((start.0, 0.0)), &[], &blocked);
    // Fast deterministic localization: project_to_region's valid-empty branch.
    // This does not substitute for the separate public Full Scan test above.
    assert_eq!(result.expect("exact original point is in the zero-radius goal"), vec![start]);
}

#[test]
fn zero_arrival_projection_must_not_admit_nearby_nonzero_error() {
    let (map, _tomo, start, reference) = zero_arrival_fixture();
    let (query, ground) = bind_query(&map, start).expect("zero arrival ground owner");
    for delta in [f64::EPSILON, 5e-7, 1e-6] {
        let mut goal = start.0;
        goal[0] += delta;
        assert!(distance3(start.0, goal) > 0.0, "no underflow/equal-input fake control");
        let blocked = std::cell::Cell::new(None);
        let result = project_to_region(&map, &query, &ground, start, &reference, &motion(), false,
            Some(0.0), Some((goal, 0.0)), &[], &blocked);
        assert!(result.is_err(), "do not fix exact zero by widening the original goal: {delta}");
    }
}

#[test]
fn positive_arrival_boundary_projection_current_strict_characterization() {
    let (map, _tomo, start, reference) = zero_arrival_fixture();
    let (query, ground) = bind_query(&map, start).expect("positive arrival ground owner");
    let goal = [start.0[0] + 0.25, start.0[1], start.0[2]];
    assert_eq!(distance3(start.0, goal), 0.25);
    let blocked = std::cell::Cell::new(None);
    let result = project_to_region(&map, &query, &ground, start, &reference, &motion(), false,
        Some(0.0), Some((goal, 0.25)), &[], &blocked);
    assert!(result.is_err(), "records today's strict positive Scan boundary, not a proposed policy");
}
