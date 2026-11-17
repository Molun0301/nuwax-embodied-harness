    use nuwax_nav_simple_planner::module::{Module, ModuleOutput, SimplePlannerModule};

    use super::*;

    #[test]
    fn published_path_preserves_goal_heading_and_uses_segment_headings() {
        let mut goal = planner_test_goal("heading", 1);
        goal.goal.pose.position[2] = 3.0;
        let half_yaw = std::f64::consts::FRAC_PI_4;
        goal.goal.pose.orientation_xyzw = [0.0, 0.0, half_yaw.sin(), half_yaw.cos()];
        schema::validate_navigation_goal(&goal).unwrap();
        let path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::from_unix(12, 0),
            vec![Point2::new(0.5, 0.5), Point2::new(2.5, 0.5)],
        );
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                    path_yaws: None,                goal: &goal,
                planner_runtime_generation: 1,
                path_revision: 1,
                path: &path,
                boundary_safe: true,
            },
            12_000_000_000,
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            envelope.path.poses[0].pose.orientation_xyzw,
            [0.0, 0.0, 0.0, 1.0]
        );
        assert_eq!(
            envelope.path.poses[1].pose.orientation_xyzw,
            goal.goal.pose.orientation_xyzw
        );
        assert!(envelope
            .path
            .poses
            .iter()
            .all(|pose| pose.pose.position[2] == 3.0));
    }

    #[test]
    fn simple_occupancy_uses_persistent_map_cells_not_cloud_history() {
        let body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: 4,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.5,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    position: [-1.0, -2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0, -1, 50, 49],
        })
        .expect("encode occupancy fixture");

        use nuwax_nav_types_messages::OccupancyCell;
        let grid = decode_simple_occupancy(&body).expect("decode persistent occupancy");
        assert_eq!(grid.origin, Point2::new(-1.0, -2.0));
        assert_eq!(grid.resolution, 0.5);
        assert_eq!(
            grid.cells,
            vec![
                OccupancyCell::Free,
                OccupancyCell::Unknown,
                OccupancyCell::Occupied,
                OccupancyCell::Free
            ]
        );
    }

    #[test]
    fn planning_failure_diagnostic_path_is_not_executable() {
        use nuwax_nav_simple_planner::module::HoldReason;

        assert!(!simple_hold_path_is_executable(HoldReason::PlanningFailed));
        assert!(simple_hold_path_is_executable(HoldReason::StopRequested));
        assert!(simple_hold_path_is_executable(HoldReason::GoalReached));
        assert!(simple_hold_path_is_executable(HoldReason::LifecycleStopped));
    }

    #[test]
    fn simple_planner_rejects_routes_through_unknown_cells() {
        let mut data = vec![-1; 25];
        data[12] = 0;
        data[14] = 0; // 起终点均为空闲，但中间未知带没有可通行证据。
        let body = schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 5,
                height: 5,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data,
        })
        .expect("encode frontier-style occupancy fixture");
        let grid = decode_simple_occupancy(&body).expect("decode unknown cells");

        let mut planner =
            SimplePlannerModule::with_config(nuwax_nav_simple_planner::SimplePlannerConfig {
                cell_size: 1.0,
                inflation_radius: 0.0,
                ..nuwax_nav_simple_planner::SimplePlannerConfig::default()
            })
            .expect("frontier planner config");
        Module::start(&mut planner).expect("start simple planner");
        planner.on_occupancy_grid(grid).expect("frontier occupancy");
        Module::on_odometry(
            &mut planner,
            PoseStamped::from_pose(
                Pose3 {
                    position: Point3::new(2.5, 2.5, 0.0),
                    ..Pose3::default()
                },
                NavFrameId::Map,
            ),
        )
        .expect("map pose");
        Module::on_goal(
            &mut planner,
            PointStamped::from_xyz(NavFrameId::Map, 4.5, 2.5, 0.0),
        )
        .expect("frontier goal");

        assert!(matches!(
            Module::tick(&mut planner).expect("planner tick"),
            ModuleOutput::SafetyHold {
                reason: nuwax_nav_simple_planner::module::HoldReason::PlanningFailed,
                ..
            }
        ));
    }

    #[test]
    fn clicked_goal_does_not_replace_authoritative_planner_pose() {
        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(1.0, 2.0, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(7, 0);
        Module::on_odometry(&mut planner, robot_pose).expect("authoritative map pose");

        let clicked = clicked_goal_pose(&schema::PointStamped {
            header: schema::Header {
                stamp_sec: 8,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            point: [5.0, 2.0, 0.0],
        })
        .expect("clicked goal");
        Module::on_goal(
            &mut planner,
            PointStamped::new(clicked.frame_id, clicked.stamp, clicked.pose.position),
        )
        .expect("goal only");

        let first = match Module::tick(&mut planner).expect("planner tick") {
            ModuleOutput::Plan { path, .. } | ModuleOutput::SafetyHold { path, .. } => {
                path.points[0]
            }
            output => panic!("expected path-bearing output, got {output:?}"),
        };
        assert!(first.distance(Point2::new(1.0, 2.0)) < 0.1);
        assert!(first.distance(Point2::new(5.0, 2.0)) > 3.0);
    }

    #[test]
    fn simple_goal_arriving_before_costmap_is_applied_once_with_full_envelope() {
        let goal = planner_test_goal("simple-goal", 4);
        let expected_key = goal.goal_key();
        let expected_boundary = goal.boundary_points.clone();
        let ready = decode_navigation_goal_input(&schema::encode_navigation_goal_body(&goal).unwrap()).unwrap();
        assert_eq!(ready.goal_key(), expected_key);
        assert_eq!(ready.boundary_points, expected_boundary);

        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start simple planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(21, 0);
        Module::on_odometry(&mut planner, robot_pose).expect("simple map pose");
        let (applied, boundary) =
            match apply_simple_planner_goal(&mut planner, ready, &planner_test_grid_body(21))
                .expect("apply simple buffered goal")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("valid simple goal rejected: {reason}"),
            };
        assert_eq!(applied, goal);
        assert!(boundary.is_some());
        assert!(matches!(
            Module::tick(&mut planner).expect("simple planner tick"),
            ModuleOutput::Plan { .. } | ModuleOutput::SafetyHold { .. }
        ));
    }

    #[test]
    fn simple_stop_holds_until_a_replacement_goal() {
        let mut module = SimplePlannerModule::new();
        Module::start(&mut module).unwrap();
        let mut pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        pose.stamp = Timestamp::from_unix(30, 0);
        Module::on_odometry(&mut module, pose).unwrap();
        for _ in 0..2 {
            Module::on_stop_movement(&mut module, true).unwrap();
            assert!(matches!(
                Module::tick(&mut module).unwrap(),
                ModuleOutput::SafetyHold {
                    reason: nuwax_nav_simple_planner::module::HoldReason::StopRequested,
                    ..
                }
            ));
        }
        let next = planner_test_goal("replacement", 2);
        assert!(matches!(
            apply_simple_planner_goal(&mut module, next, &planner_test_grid_body(30)).unwrap(),
            OwnerInput::Accepted(_)
        ));
        assert!(matches!(
            Module::tick(&mut module).unwrap(),
            ModuleOutput::Plan { .. } | ModuleOutput::SafetyHold { .. }
        ));
    }

    #[test]
    fn simple_new_goal_ignores_map_metadata_and_builds_correlated_path() {
        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start simple planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(12, 0);
        Module::on_odometry(&mut planner, robot_pose).expect("simple map pose");

        planner
            .on_occupancy_grid(
                decode_simple_occupancy(&planner_test_grid_body(12)).expect("map 12"),
            )
            .expect("apply map 12");
        let goal_12 = planner_test_goal("simple-12", 1);
        let ready_12 = goal_12.clone();
        let (applied_12, boundary_12) =
            match apply_simple_planner_goal(&mut planner, ready_12, &planner_test_grid_body(12))
                .expect("apply goal 12")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 12 rejected: {reason}"),
            };
        assert_eq!(applied_12, goal_12);
        assert!(boundary_12.is_some());
        let goal_13 = planner_test_goal("simple-13", 2);
        let ready_13 = goal_13.clone();
        let (applied_13, boundary_13) =
            match apply_simple_planner_goal(&mut planner, ready_13, &planner_test_grid_body(12))
                .expect("apply new goal with an independently sequenced costmap")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 13 rejected: {reason}"),
            };
        let active_goal = Some(applied_13);
        let active_boundary = boundary_13;
        assert_eq!(
            active_goal.as_ref().map(|goal| goal.session_id.as_str()),
            Some("simple-13")
        );
        assert!(active_boundary.is_some());
        let path = match Module::tick(&mut planner).expect("simple map-b tick") {
            ModuleOutput::Plan { path, .. } | ModuleOutput::SafetyHold { path, .. } => path,
            output => panic!("expected map-b path, got {output:?}"),
        };
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                    path_yaws: None,                goal: active_goal.as_ref().expect("active map-b goal"),
                planner_runtime_generation: 23,
                path_revision: 1,
                path: &path,
                boundary_safe: path_respects_navigation_boundary(
                    &path,
                    active_boundary.as_ref(),
                    0.5,
                ),
            },
            13_000_000_000,
        )
        .expect("build map-b PathEnvelope")
        .expect("map-b path stays inside boundary");
        assert_eq!(
            envelope.goal_key(),
            active_goal.as_ref().unwrap().goal_key()
        );
    }

    #[test]
    fn simple_owner_drops_bad_pose_and_grid_then_accepts_new_valid_inputs() {
        let mut planner = SimplePlannerModule::new();
        Module::start(&mut planner).expect("start simple planner");
        assert!(matches!(
            decode_planner_pose_input(&[0x44], "simple_planner.map_pose")
                .expect("bad pose is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let pose_body = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 30,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [0.5, 0.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode valid simple pose");
        let pose = match decode_planner_pose_input(&pose_body, "simple_planner.map_pose")
            .expect("valid simple pose")
        {
            OwnerInput::Accepted(pose) => pose,
            OwnerInput::Rejected(reason) => panic!("valid simple pose rejected: {reason}"),
        };
        Module::on_odometry(&mut planner, pose).expect("simple planner accepts valid pose");

        assert!(matches!(
            decode_planner_grid_input(&[0x44], "simple_planner.occupancy")
                .expect("bad grid is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let grid_body = planner_test_grid_body(30);
        assert!(matches!(
            decode_planner_grid_input(&grid_body, "simple_planner.occupancy")
                .expect("valid simple grid"),
            OwnerInput::Accepted(_)
        ));
        let occupancy = decode_simple_occupancy(&grid_body).expect("decode valid simple grid");
        planner
            .on_occupancy_grid(occupancy)
            .expect("simple planner accepts valid grid");
    }
