    use super::*;

    #[tokio::test]
    async fn astar_real_owner_retains_goal_costmap_until_pose_and_publishes_exact_path() {
        static NEXT_ASTAR_OWNER_ID: std::sync::atomic::AtomicU64 =
            std::sync::atomic::AtomicU64::new(0);
        let id = NEXT_ASTAR_OWNER_ID.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let prefix = format!("/test/astar-cold/{}/{id}", std::process::id());
        let node = TaskChannels { id: format!("astar-cold-owner-{id}"), inputs: vec![format!("{prefix}/pose"),format!("{prefix}/costmap"),format!("{prefix}/goal"),format!("{prefix}/stop")], outputs: vec![format!("{prefix}/waypoint"),format!("{prefix}/path"),format!("{prefix}/reached"),format!("{prefix}/failed")] };
        let transport = LcmTransport::new_real("memq://").expect("real LCM memq transport");
        let mut paths = typed_stream(&transport, &node.outputs[1], "nuwax_msgs.PathEnvelope")
            .await
            .expect("path subscription");
        let (shutdown_tx, shutdown_rx) = oneshot::channel();
        let (ready_tx, ready_rx) = std_mpsc::sync_channel(1);
        let owner = tokio::spawn(run_astar_planner_with_transport(node.clone(), Arc::from("base_link"), transport.clone(), shutdown_rx, ready_tx));
        tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                match ready_rx.try_recv() {
                    Ok(Ok(())) => break,
                    Ok(Err(reason)) => panic!("A* owner startup failed: {reason}"),
                    Err(std_mpsc::TryRecvError::Empty) => {
                        assert!(!owner.is_finished(), "A* owner exited before ready");
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                    Err(std_mpsc::TryRecvError::Disconnected) => {
                        panic!("A* owner dropped readiness channel")
                    }
                }
            }
        })
        .await
        .expect("A* owner readiness timeout");

        let goal = planner_test_goal("astar-cold-22", 1);
        publish_body(
            &transport,
            &node.inputs[2],
            "nuwax_msgs.NavigationGoal",
            schema::encode_navigation_goal_body(&goal).expect("encode goal"),
        )
        .await
        .expect("publish goal first");
        publish_body(
            &transport,
            &node.inputs[1],
            "nav_msgs.OccupancyGrid",
            planner_test_grid_body(22),
        )
        .await
        .expect("publish costmap second");
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(
            !owner.is_finished(),
            "recoverable missing pose must not terminate the A* owner"
        );

        let pose = schema::PoseStamped {
            header: schema::Header {
                seq: 22,
                stamp_sec: 22,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [0.5, 0.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        };
        publish_body(
            &transport,
            &node.inputs[0],
            "geometry_msgs.PoseStamped",
            schema::encode_pose_stamped_body(&pose).expect("encode pose"),
        )
        .await
        .expect("publish pose third");

        let frame = tokio::time::timeout(Duration::from_secs(2), paths.next())
            .await
            .expect("A* owner did not publish after the late pose")
            .expect("path stream closed")
            .expect("path transport error");
        let path = schema::decode_path_envelope_body(&frame.payload).expect("decode path envelope");
        assert_eq!(path.session_id, goal.session_id);
        assert_eq!(path.path_revision, 1);
        assert_eq!(path.goal_key(), goal.goal_key());

        shutdown_tx.send(()).expect("A* owner shutdown receiver");
        tokio::time::timeout(Duration::from_secs(3), owner)
            .await
            .expect("A* owner shutdown timeout")
            .expect("A* owner join")
            .expect("A* owner result");
    }

    fn grid_body(frame: &str, orientation_xyzw: [f64; 4]) -> Vec<u8> {
        schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                frame_id: frame.into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 0.1,
                width: 2,
                height: 2,
                origin: schema::Pose {
                    orientation_xyzw,
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 4],
        })
        .expect("encode test grid")
    }

    fn path_body(path_frame: &str, pose_frame: &str) -> Vec<u8> {
        let header = schema::Header {
            stamp_sec: 7,
            stamp_nsec: 8,
            frame_id: path_frame.into(),
            ..schema::Header::default()
        };
        schema::encode_path_body(&schema::Path {
            header,
            poses: vec![schema::PoseStamped {
                header: schema::Header {
                    frame_id: pose_frame.into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [1.0, 2.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        })
        .expect("encode test path")
    }

    #[test]
    fn astar_costmap_rejects_non_map_frame() {
        let error = decode_astar_costmap(&grid_body("odom", [0.0, 0.0, 0.0, 1.0]))
            .expect_err("frame mismatch must fail");
        assert!(error.to_string().contains("frame must be map"));
    }

    #[test]
    fn astar_costmap_rejects_unrepresentable_origin_rotation() {
        let error = decode_astar_costmap(&grid_body("map", [0.0, 0.0, 0.1, 0.995]))
            .expect_err("rotated origin must fail");
        assert!(error.to_string().contains("rotated or non-planar"));
    }

    #[test]
    fn path_decode_preserves_header_frame_and_stamp() {
        let path = decode_path_body(&path_body("warehouse", "warehouse")).expect("decode path");
        assert_eq!(path.frame_id, NavFrameId::Custom("warehouse".into()));
        assert_eq!(path.stamp, Timestamp::from_unix(7, 8));
        assert_eq!(path.points, vec![Point2::new(1.0, 2.0)]);
    }

    #[test]
    fn path_decode_rejects_pose_frame_mismatch() {
        let error = decode_path_body(&path_body("map", "odom"))
            .expect_err("path pose/header mismatch must fail");
        assert!(error.to_string().contains("Path pose frame does not match"));
    }

    #[test]
    fn astar_goal_arriving_before_costmap_is_applied_once_with_full_envelope() {
        use nuwax_nav_astar::module::{Module, NavigationInterface, ReplanningAStarPlannerModule};

        let goal = planner_test_goal("astar-goal", 5);
        let expected_key = goal.goal_key();
        let expected_boundary = goal.boundary_points.clone();
        let ready = decode_navigation_goal_input(&schema::encode_navigation_goal_body(&goal).unwrap()).unwrap();
        assert_eq!(ready.goal_key(), expected_key);
        assert_eq!(ready.boundary_points, expected_boundary);

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::new(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.5)/2., -(0.5)/2.], [(0.5)/2., -(0.5)/2.], [(0.5)/2., (0.5)/2.], [-(0.5)/2., (0.5)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }));
        NavigationInterface::start(&mut planner).expect("start A* planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(22, 0);
        Module::on_odom(&mut planner, robot_pose).expect("A* map pose");
        let (applied, boundary, resolution) =
            match apply_astar_planner_goal(&mut planner, ready, &planner_test_grid_body(22))
                .expect("apply A* buffered goal")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("valid A* goal rejected: {reason}"),
            };
        assert_eq!(applied, goal);
        assert!(boundary.is_some());
        assert_eq!(resolution, 1.0);
        let _ = Module::tick(&mut planner).expect("A* planner tick");
    }

    #[test]
    fn astar_new_goal_ignores_map_metadata_and_builds_correlated_path() {
        use nuwax_nav_astar::module::{
            Module, ModuleOutput, NavigationInterface, ReplanningAStarPlannerModule,
        };

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::new(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.5)/2., -(0.5)/2.], [(0.5)/2., -(0.5)/2.], [(0.5)/2., (0.5)/2.], [-(0.5)/2., (0.5)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }));
        NavigationInterface::start(&mut planner).expect("start A* planner");
        let mut robot_pose = PoseStamped::from_pose(
            Pose3 {
                position: Point3::new(0.5, 0.5, 0.0),
                orientation: nuwax_nav_types_messages::Quaternion::IDENTITY,
                ..Pose3::default()
            },
            NavFrameId::Map,
        );
        robot_pose.stamp = Timestamp::from_unix(12, 0);
        Module::on_odom(&mut planner, robot_pose).expect("A* map pose");

        Module::on_costmap(
            &mut planner,
            decode_astar_costmap(&planner_test_grid_body(12)).expect("map 12"),
        )
        .expect("apply map 12");
        let goal_12 = planner_test_goal("astar-12", 1);
        let ready_12 = goal_12.clone();
        let (applied_12, boundary_12, _) =
            match apply_astar_planner_goal(&mut planner, ready_12, &planner_test_grid_body(12))
                .expect("apply goal 12")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 12 rejected: {reason}"),
            };
        assert_eq!(applied_12, goal_12);
        assert!(boundary_12.is_some());
        let goal_13 = planner_test_goal("astar-13", 2);
        let ready_13 = goal_13.clone();
        let (applied_13, boundary_13, _) =
            match apply_astar_planner_goal(&mut planner, ready_13, &planner_test_grid_body(12))
                .expect("apply new goal with an independently sequenced costmap")
            {
                OwnerInput::Accepted(applied) => applied,
                OwnerInput::Rejected(reason) => panic!("goal 13 rejected: {reason}"),
            };
        let active_goal = Some(applied_13);
        let active_boundary = boundary_13;
        assert_eq!(
            active_goal.as_ref().map(|goal| goal.session_id.as_str()),
            Some("astar-13")
        );
        assert!(active_boundary.is_some());
        let update = match Module::tick(&mut planner).expect("A* map-b tick") {
            ModuleOutput::Navigation(update) | ModuleOutput::SafetyHold(update) => update,
        };
        let path = update.path.expect("A* map-b path");
        let envelope = build_versioned_path_envelope(
            VersionedPathPublication {
                    path_yaws: None,                goal: active_goal.as_ref().expect("active map-b goal"),
                planner_runtime_generation: 29,
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
        .expect("build A* map-b PathEnvelope")
        .unwrap_or_else(|| {
            let map = decode_astar_costmap_with_boundary(&planner_test_grid_body(12), active_boundary.as_ref()).unwrap();
            let raw = nuwax_nav_astar::astar::min_cost_astar(&map, Point2::new(0.5, 0.5), Point2::new(4.5, 4.5),
                &nuwax_nav_astar::astar::AStarOptions::default());
            panic!("A* map-b path stays inside boundary: path={path:?}, boundary={active_boundary:?}, raw_astar={raw:?}");
        });
        assert_eq!(
            envelope.goal_key(),
            active_goal.as_ref().unwrap().goal_key()
        );
    }

    #[test]
    fn astar_owner_drops_bad_pose_and_grid_then_accepts_new_valid_inputs() {
        use nuwax_nav_astar::module::{Module, NavigationInterface, ReplanningAStarPlannerModule};

        let mut planner =
            ReplanningAStarPlannerModule::new(nuwax_nav_astar::GlobalPlannerConfig::new(nuwax_nav_types_messages::RobotGeometry { self_collision: None, footprint_xy_m: vec![[ -(0.5)/2., -(0.5)/2.], [(0.5)/2., -(0.5)/2.], [(0.5)/2., (0.5)/2.], [-(0.5)/2., (0.5)/2.]], z_min_m: -(0.4), z_max_m: (0.6)-(0.4), source: "测试外形".into() }));
        NavigationInterface::start(&mut planner).expect("start A* planner");
        assert!(matches!(
            decode_planner_pose_input(&[0x55], "astar_planner.map_pose")
                .expect("bad pose is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let pose_body = schema::encode_pose_stamped_body(&schema::PoseStamped {
            header: schema::Header {
                stamp_sec: 31,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            pose: schema::Pose {
                position: [0.5, 0.5, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        })
        .expect("encode valid A* pose");
        let pose = match decode_planner_pose_input(&pose_body, "astar_planner.map_pose")
            .expect("valid A* pose")
        {
            OwnerInput::Accepted(pose) => pose,
            OwnerInput::Rejected(reason) => panic!("valid A* pose rejected: {reason}"),
        };
        Module::on_odom(&mut planner, pose).expect("A* planner accepts valid pose");

        assert!(matches!(
            decode_planner_grid_input(&[0x55], "astar_planner.costmap")
                .expect("bad grid is recoverable"),
            OwnerInput::Rejected(_)
        ));
        let grid_body = planner_test_grid_body(31);
        assert!(matches!(
            decode_planner_grid_input(&grid_body, "astar_planner.costmap").expect("valid A* grid"),
            OwnerInput::Accepted(_)
        ));
        let costmap = decode_astar_costmap(&grid_body).expect("decode valid A* grid");
        Module::on_costmap(&mut planner, costmap).expect("A* planner accepts valid grid");
    }
