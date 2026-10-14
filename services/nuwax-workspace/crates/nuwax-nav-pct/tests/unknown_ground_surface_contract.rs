//! Independent public-API integration tests. Synthetic geometry is isolated from
//! the archived root365 replay; these tests never import simulator world state.
#![cfg(feature = "mls-adapter")]
use nuwax_nav_mls_planner::{
    SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations, SurfaceSnapshot,
    mls_planner::Config,
};
use nuwax_nav_pct::{PctPlanner, SearchLimits, Tomogram};
use nuwax_nav_types_messages::GoalOptions;
use std::{collections::BTreeMap, sync::Arc};

type Pose = ([f64; 3], [f64; 4]);
const FINE: f32 = 0.05;
const BOUNDARY: [[f64; 2]; 4] = [[-1.0, -0.2], [1.0, -0.2], [1.0, 0.2], [-1.0, 0.2]];
fn pose(x: f64) -> Pose {
    ([x, 0.025, 0.5], [0.0, 0.0, 0.0, 1.0])
}
fn config() -> Config {
    Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.1,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.0,
        wall_buffer_weight: 0.0,
        step_threshold_m: 0.05,
        step_penalty_weight: 1.0,
        max_slope_deg: 18.0,
        min_step_tread_depth_m: 0.2,
        goal_tolerance: 0.02,
        viz_publish_hz: 0.0,
    }
}
fn fine_scene(keys: impl IntoIterator<Item = [i32; 3]>, cfg: Config) -> SurfaceSnapshot {
    let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    let mut points: Vec<_> = keys
        .into_iter()
        .map(|key| {
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 1,
                        hits: 1,
                    },
                );
            (
                (key[0] as f32 + 0.5) * FINE,
                (key[1] as f32 + 0.5) * FINE,
                (key[2] as f32 + 0.5) * FINE,
            )
        })
        .collect();
    // SurfaceSnapshot::from_points requires a nonempty allocation seed. For an
    // authoritative empty fine source, retain a historical coarse seed far outside
    // the bounded task, then attach complete-empty observations. It contributes
    // no fine occupancy or ground: tests separately assert the tomogram is empty.
    if points.is_empty() {
        points.push((100.025, 100.025, 100.025));
    }
    let mut observations = SurfaceObservations::new(FINE, 32).unwrap();
    for (key, values) in blocks {
        observations.replace_block(key, values).unwrap();
    }
    observations.mark_complete_geometry_source();
    SurfaceSnapshot::from_points(&points, cfg, FINE)
        .unwrap()
        .with_observations(Some(Arc::new(observations)))
}
fn scene(keys: impl IntoIterator<Item = [i32; 3]>) -> SurfaceSnapshot {
    fine_scene(keys, config())
        .with_body_envelope(SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.08, -0.06], [0.08, -0.06], [0.08, 0.06], [-0.08, 0.06]],
            z_bounds_m: [-0.1, 0.2],
            body_to_support_z_m: 0.5,
            max_step_up_m: 0.05,
            max_step_down_m: 0.05,
            max_cross_slope_deg: 18.0,
            support_uncertainty_m: 0.0,
        })
        .unwrap()
}
fn route(
    planner: &PctPlanner,
    map: &SurfaceSnapshot,
    a: Pose,
    b: Pose,
) -> Result<Vec<Pose>, String> {
    let mut options = GoalOptions::default();
    options.position_tolerance_m = 0.02;
    planner.body_route(map, a, b, options, &BOUNDARY, None)
}
fn islands() -> Vec<[i32; 3]> {
    (-20..20)
        .filter(|x| *x < -10 || *x > 10)
        .flat_map(|x| (-5..5).map(move |y| [x, y, -1]))
        .collect()
}

#[test]
fn unknown_start_gap_and_end_do_not_mutate_cached_observed_geometry() {
    let map = scene(islands());
    let planner = PctPlanner::default();
    let cached = planner.map(&map).unwrap();
    let cells = cached.cells.len();
    let observation = map.observations().unwrap().clone();
    for (start, end) in [(-0.8, 0.8), (0.025, 0.8), (-0.8, 0.025)] {
        let path = route(&planner, &map, pose(start), pose(end)).unwrap();
        assert!(path.len() > 1, "nontrivial request needs a real route");
        assert_eq!(path.first(), Some(&pose(start)));
        assert!((path.last().unwrap().0[0] - end).abs() < 0.06);
        assert!(path.iter().all(|p| (p.0[2] - 0.5).abs() < 0.051));
    }
    assert_eq!(
        cached.cells.len(),
        cells,
        "request-only unknown nodes must not become terrain"
    );
    assert!(Arc::ptr_eq(&cached, &planner.map(&map).unwrap()));
    assert!(Arc::ptr_eq(&observation, map.observations().unwrap()));
}

#[test]
fn all_unknown_fine_geometry_routes_without_fabricating_observed_cells() {
    let map = scene([]);
    let planner = PctPlanner::default();
    let before = planner.map(&map).unwrap();
    assert!(before.cells.is_empty());
    let path = route(&planner, &map, pose(-0.8), pose(0.8)).unwrap();
    assert!(path.len() > 1);
    assert!(path.iter().all(|p| p.0[2] == 0.5));
    assert!(planner.map(&map).unwrap().cells.is_empty());
}

#[test]
fn forbidden_mask_in_unobserved_columns_is_still_a_hard_barrier() {
    let base = scene(islands());
    let planner = PctPlanner::default();
    assert!(route(&planner, &base, pose(-0.8), pose(0.8)).is_ok());
    let mut mask = vec![false; 40 * 10];
    for y in 0..10 {
        for x in 19..22 {
            mask[y * 40 + x] = true;
        }
    }
    let blocked = base
        .with_virtual_obstacles([-1.0, -0.25], 0.05, 40, 10, &mask)
        .unwrap();
    assert!(!blocked.navigation_column_allowed([0, 0], 0.05));
    assert!(
        route(&planner, &blocked, pose(-0.8), pose(0.8)).is_err(),
        "absent physical source columns must not hide forbidden-mask coverage"
    );
    let restored = blocked
        .with_virtual_obstacles([-1.0, -0.25], 0.05, 40, 10, &vec![false; 400])
        .unwrap();
    assert!(route(&planner, &restored, pose(-0.8), pose(0.8)).is_ok());
}

#[test]
fn known_wall_and_low_top_cannot_be_reinterpreted_as_unknown_floor() {
    for (label, zs) in [
        ("wall", (0..22).collect::<Vec<_>>()),
        ("low top", vec![10, 11]),
    ] {
        let mut keys = islands();
        for x in -1..=1 {
            for y in -5..=5 {
                for &z in &zs {
                    keys.push([x, y, z]);
                }
            }
        }
        let map = scene(keys);
        assert!(
            route(&PctPlanner::default(), &map, pose(-0.8), pose(0.8)).is_err(),
            "{label}"
        );
        assert!(
            map.validate_body_motion(pose(-0.8), pose(0.8), &BOUNDARY)
                .is_err(),
            "{label} must remain an actual swept-volume collision"
        );
    }
}

#[test]
fn recent_obstacle_rejects_actual_motion_even_with_reusable_global_route() {
    let base = scene([]);
    let planner = PctPlanner::default();
    let original = route(&planner, &base, pose(-0.8), pose(0.8)).unwrap();
    assert!(
        base.validate_body_motion(pose(-0.8), pose(0.8), &BOUNDARY)
            .is_ok()
    );
    let recent = base
        .with_recent_obstacle_points(1, 100, 0, &[], [(0, [0.025, 0.025, 0.55])])
        .unwrap();
    assert!(
        recent
            .validate_body_motion(pose(-0.8), pose(0.8), &BOUNDARY)
            .is_err()
    );
    assert_eq!(
        original,
        route(&planner, &base, pose(-0.8), pose(0.8)).unwrap()
    );
}

#[test]
fn observed_cliff_after_unknown_gap_is_not_erased_by_reference_continuation() {
    let keys = (-20..20)
        .filter(|x| *x < -4 || *x > 4)
        .flat_map(|x| (-5..=5).map(move |y| [x, y, if x < 0 { -1 } else { -11 }]));
    let map = scene(keys);
    let start = pose(-0.8);
    let mut lower = pose(0.8);
    lower.0[2] = 0.0;
    assert!(
        route(&PctPlanner::default(), &map, start, lower).is_err(),
        "known half-meter drop exceeds both configured five-centimeter step limits"
    );
    assert!(
        route(&PctPlanner::default(), &map, start, pose(0.8)).is_err(),
        "the original height cannot float above the known lower floor as unknown"
    );
}

#[test]
fn unknown_gap_cannot_connect_disjoint_known_building_layers() {
    let keys = (-20..20).filter(|x| *x < -4 || *x > 4).flat_map(|x| {
        (-5..=5).flat_map(move |y| {
            [[-1, 19]]
                .into_iter()
                .flat_map(move |levels| levels.map(|z| [x, y, z]))
        })
    });
    let map = scene(keys);
    let mut upper = pose(0.8);
    upper.0[2] = 1.5;
    assert!(
        route(&PctPlanner::default(), &map, pose(-0.8), upper).is_err(),
        "unknown reference must not interpolate toward a different requested floor"
    );
}

#[test]
fn cropped_local_source_never_converts_unmaterialized_wall_into_unknown() {
    // A horizon-5 local planner can request a 7.4 m guide. The prior 6.5 m
    // materialization excludes this actual fine wall, while an 8 m crop includes it.
    let wall =
        (133..=135).flat_map(|x| (-5..=5).flat_map(move |y| (0..22).map(move |z| [x, y, z])));
    let source = scene(wall);
    let start = [0.025, 0.025, 0.0];
    let goal = [7.4, 0.025, 0.0];
    let boundary = [[-1.0, -0.2], [7.6, -0.2], [7.6, 0.2], [-1.0, 0.2]];
    let old_crop = Tomogram::local_from_surface(&source, [0.0, 0.0], 6.5).unwrap();
    let old_error = old_crop
        .plan(start, goal, 0.02, &boundary, None)
        .unwrap_err();
    assert!(
        old_error.contains("SearchDomainLimited"),
        "missing crop must be inconclusive: {old_error}"
    );
    let large_crop = Tomogram::local_from_surface(&source, [0.0, 0.0], 8.0).unwrap();
    let wall_error = large_crop
        .plan(start, goal, 0.02, &boundary, None)
        .unwrap_err();
    assert!(
        !wall_error.contains("SearchDomainLimited"),
        "wall lies inside materialized domain: {wall_error}"
    );
}

#[test]
fn no_envelope_reference_keeps_known_tread_limits_and_calibration_through_unknown() {
    // These are legacy planar callers: no fabricated body envelope is attached.
    // The 5 cm rise is within the configured step limit, while a 10 cm landing
    // violates the independently configured 20 cm minimum tread depth.
    let source = |shallow: bool| {
        let keys = (-20..20).flat_map(|x| {
            (-5..=5).map(move |y| {
                let raised = if shallow { (0..2).contains(&x) } else { x >= 0 };
                [x, y, if raised { 0 } else { -1 }]
            })
        });
        fine_scene(keys, config())
    };
    let start_reference = [-0.8, 0.025, 0.012];
    let start = ([start_reference[0], start_reference[1], 0.512], pose(0.0).1);
    let options = GoalOptions { position_tolerance_m: 0.005, ..GoalOptions::default() };
    for shallow in [false, true] {
        let map = source(shallow);
        assert!(map.body_envelope().is_none());
        let planner = PctPlanner::default();
        let cached = planner.map(&map).unwrap();
        let projected = cached.project_reference_motion(start_reference, [0.8, 0.025], &BOUNDARY);
        let goal_z = if shallow { 0.512 } else { 0.562 };
        let goal = ([0.8, 0.025, goal_z], start.1);
        let route = planner.reference_body_route_with_offset(
            &map, start, goal, options, &BOUNDARY, None, SearchLimits::default(), 0.5,
        );
        if shallow {
            assert!(projected.is_err(), "known shallow tread cannot be relabeled unknown");
            assert!(route.is_err(), "no-envelope route must use the same known tread contract");
        } else {
            let projected = projected.expect("a complete permitted step remains traversable");
            assert!((projected[2] - 0.062).abs() < 1e-6, "calibration bias survives the real rise");
            let route = route.expect("complete permitted step has a legacy reference route");
            assert!(route.len() > 1);
            assert!((route.last().unwrap().0[2] - goal_z).abs() < 0.005);
        }
        assert!(Arc::ptr_eq(&cached, &planner.map(&map).unwrap()));
    }
    let unknown = fine_scene([], config());
    let planner = PctPlanner::default();
    let cached = planner.map(&unknown).unwrap();
    assert!(cached.cells.is_empty());
    assert_eq!(
        cached.project_reference_motion(start_reference, [0.8, 0.025], &BOUNDARY).unwrap(),
        [0.8, 0.025, 0.012],
    );
    let route = planner.reference_body_route_with_offset(
        &unknown, start, ([0.8, 0.025, 0.512], start.1), options, &BOUNDARY, None,
        SearchLimits::default(), 0.5,
    ).unwrap();
    assert!(route.len() > 1);
    assert!(route.iter().all(|pose| (pose.0[2] - 0.512).abs() < 1e-9));
    assert!(cached.cells.is_empty(), "planar traversal must not fabricate a floor");
}
