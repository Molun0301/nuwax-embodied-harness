// Configured disk/height compatibility mode; these are not full-body proofs.
use super::*;

fn fixture(
    points: &[(f32, f32, f32)],
) -> (
    SurfaceSnapshot,
    SurfaceNavigationConfig,
    PoseStamped,
    schema::GoalEnvelope,
) {
    let config = config();
    let map = SurfaceSnapshot::from_points(points, config.geometry.clone(), 0.1).unwrap();
    assert!(map.body_envelope().is_none());
    let mut goal = goal_for(&map_message(1, "legacy-reference"), 1, [1.0, 0.0, 0.45]);
    goal.options.position_tolerance_m = 0.05;
    let mut start = pose_from_schema(&goal.goal).unwrap();
    start.pose.position.x = 0.0;
    (map, config, start, goal)
}
#[test]
fn legacy_reference_plans_unknown_start_gap_and_endpoint_without_invented_envelope() {
    let remote = [(3.05, 3.05, 0.05)];
    let ends: Vec<_> = (-5..16)
        .filter(|x| *x < 2 || *x > 8)
        .flat_map(|x| (-3..4).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05)))
        .collect();
    for (label, points) in [
        ("unknown start and endpoint", remote.as_slice()),
        ("unknown gap", ends.as_slice()),
    ] {
        let (map, config, start, goal) = fixture(points);
        let path = body_path(&map, &start, &goal, &config)
            .unwrap_or_else(|error| panic!("{label}: {error}"));
        assert!(path.poses.len() >= 2);
        assert_eq!(path.poses[0].pose, start.pose);
        assert!(
            path.poses
                .last()
                .unwrap()
                .pose
                .position
                .distance(Point3::new(1.0, 0.0, 0.45))
                <= goal.options.position_tolerance_m
        );
        let pct = config.follower.pct.map(&map).unwrap();
        let mut support = [0.0, 0.0, 0.05];
        for pose in path.poses.iter().skip(1) {
            support = pct
                .project_reference_motion(
                    support,
                    [pose.pose.position.x, pose.pose.position.y],
                    &[],
                )
                .unwrap();
            assert!(
                (support[2] + config.follower.body_to_support_z_m - pose.pose.position.z).abs()
                    < 1e-6
            );
        }
    }
}
#[test]
fn legacy_reference_retains_actual_wall_low_top_and_goal_height_constraints() {
    for wall in [true, false] {
        let mut points = vec![(3.05, 3.05, 0.05)];
        for x in 4..=6 {
            for y in -3..=3 {
                if wall {
                    for z in 0..=8 {
                        points.push((
                            (x as f32 + 0.5) * 0.1,
                            (y as f32 + 0.5) * 0.1,
                            (z as f32 + 0.5) * 0.1,
                        ));
                    }
                } else {
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.45));
                }
            }
        }
        let (map, config, start, mut goal) = fixture(&points);
        goal.boundary_points = vec![
            [-0.2, -0.15, 0.0],
            [1.2, -0.15, 0.0],
            [1.2, 0.15, 0.0],
            [-0.2, 0.15, 0.0],
        ];
        assert!(
            body_path(&map, &start, &goal, &config).is_err(),
            "wall={wall}"
        );
    }
    let (map, config, start, mut goal) = fixture(&[(3.05, 3.05, 0.05)]);
    goal.goal.pose.position[2] = 2.0;
    assert!(body_path(&map, &start, &goal, &config).is_err());
}
