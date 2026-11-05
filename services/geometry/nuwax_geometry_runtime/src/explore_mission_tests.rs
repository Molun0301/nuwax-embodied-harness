use super::*;
pub(crate) fn calibration() -> MissionCalibration {
    MissionCalibration {
        maximum_sample_speed_mps: 2.0,
        planar: SimplePlannerConfig {
            robot_geometry: Some(RobotGeometry {
                self_collision: None,
                footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                z_min_m: -0.1,
                z_max_m: 0.1,
                source: "路线仿真".into(),
            }),
            ..Default::default()
        },
    }
}
fn pose(x: f64, y: f64) -> MissionPose {
    MissionPose {
        position: [x, y, 0.0],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
    }
}
#[test]
fn mission_route_measures_wall_detour_and_rejects_unknown_home() {
    let mut grid = OccupancyGrid2D::new(
        Point2::new(0.0, 0.0),
        0.1,
        60,
        60,
        vec![OccupancyCell::Free; 3600],
    );
    for y in 0..45 {
        grid.set(30, y, OccupancyCell::Occupied);
    }
    let route = MissionRoutes::planar(grid.clone(), &calibration(), &ExploreStartConfig::default())
        .unwrap();
    let length = route
        .query(&pose(2.0, 2.0), &pose(4.0, 2.0), home_goal_options(), None)
        .unwrap()
        .length;
    assert!(length > 5.0, "绕墙路线不能退化为2米直线，实际{length}");
    assert_eq!(
        route
            .query(&pose(2.0, 2.0), &pose(2.0, 2.0), home_goal_options(), None)
            .unwrap()
            .length,
        0.0
    );
    grid.set(40, 20, OccupancyCell::Unknown);
    let route =
        MissionRoutes::planar(grid, &calibration(), &ExploreStartConfig::default()).unwrap();
    assert!(route
        .query(
            &pose(2.0, 2.0),
            &pose(4.05, 2.05),
            home_goal_options(),
            None
        )
        .is_err());
}
#[test]
fn mission_boundary_rejects_concave_notch_through_cell() {
    let boundary = ExplorationBoundary::from_vertices(vec![
        Point2::new(0.0, 0.0),
        Point2::new(4.0, 0.0),
        Point2::new(4.0, 4.0),
        Point2::new(2.6, 4.0),
        Point2::new(2.6, 2.4),
        Point2::new(2.4, 2.4),
        Point2::new(2.4, 4.0),
        Point2::new(0.0, 4.0),
    ])
    .unwrap();
    assert!(!cell_in_boundary(&boundary, Point2::new(2.0, 2.0), 1.0));
    assert!(cell_in_boundary(&boundary, Point2::new(1.0, 1.0), 1.0));
}
