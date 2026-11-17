    use super::*;
    use nuwax_nav_astar::navigation_map::OCCUPIED;

    fn free_grid_body() -> Vec<u8> {
        schema::encode_occupancy_grid_body(&schema::OccupancyGrid {
            header: schema::Header {
                seq: 17,
                stamp_sec: 1,
                frame_id: "map".into(),
                ..schema::Header::default()
            },
            info: schema::MapMetaData {
                resolution: 1.0,
                width: 7,
                height: 7,
                origin: schema::Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..schema::Pose::default()
                },
                ..schema::MapMetaData::default()
            },
            data: vec![0; 49],
        })
        .expect("encode free grid")
    }

    fn concave_boundary_points() -> Vec<[f64; 3]> {
        vec![
            [0.0, 0.0, 0.0],
            [6.0, 0.0, 0.0],
            [6.0, 6.0, 0.0],
            [4.0, 6.0, 0.0],
            [4.0, 2.0, 0.0],
            [2.0, 2.0, 0.0],
            [2.0, 6.0, 0.0],
            [0.0, 6.0, 0.0],
            [0.0, 0.0, 0.0],
        ]
    }

    fn goal(owner_kind: i32, boundary_points: Vec<[f64; 3]>) -> schema::NavigationGoal {
        schema::NavigationGoal {
boundary_points: boundary_points,
goal: schema::PoseStamped {
                header: schema::Header {
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                pose: schema::Pose {
                    position: [5.5, 5.5, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            },
..schema::NavigationGoal::default()
}
    }

    #[test]
    fn concave_boundary_rejects_shortcut_path_with_inside_endpoints() {
        let goal = goal(
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            concave_boundary_points(),
        );
        let boundary = navigation_boundary_from_goal(&goal)
            .expect("valid boundary")
            .expect("non-empty boundary");
        let path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(1.5, 5.5), Point2::new(5.5, 5.5)],
        );

        assert!(boundary.contains(Point2::new(1.5, 5.5)));
        assert!(boundary.contains(Point2::new(5.5, 5.5)));
        assert!(!path_respects_navigation_boundary(
            &path,
            Some(&boundary),
            0.5,
        ));

        let map = decode_astar_costmap_with_boundary(&free_grid_body(), Some(&boundary))
            .expect("masked A* map");
        assert_eq!(map.get(3, 5), OCCUPIED, "concave notch must be masked");
        assert_eq!(map.get(1, 5), 0, "left arm remains navigable");

        let grid = decode_simple_occupancy_with_boundary(&free_grid_body(), Some(&boundary))
            .expect("masked simple occupancy");
        assert_eq!(
            grid.get(3, 5),
            Some(nuwax_nav_types_messages::OccupancyCell::Occupied)
        );
    }

    #[test]
    fn boundary_does_not_cross_goal_lease_reuse() {
        let exploration = goal(
            schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD,
            concave_boundary_points(),
        );
        let exploration_boundary = navigation_boundary_from_goal(&exploration)
            .expect("valid exploration boundary")
            .expect("non-empty exploration boundary");
        let masked =
            decode_astar_costmap_with_boundary(&free_grid_body(), Some(&exploration_boundary))
                .expect("masked exploration map");
        assert_eq!(masked.get(3, 5), OCCUPIED);

        let public = goal(schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV, Vec::new());
        let public_boundary = navigation_boundary_from_goal(&public).expect("public goal");
        assert!(public_boundary.is_none());
        let restored =
            decode_astar_costmap_with_boundary(&free_grid_body(), public_boundary.as_ref())
                .expect("unmasked public map");
        assert_eq!(restored.get(3, 5), 0, "old exploration mask must not leak");
    }

    #[test]
    fn public_nav_is_not_constrained_by_a_boundary_mask() {
        let public = goal(schema::NAVIGATION_OWNER_KIND_PUBLIC_NAV, Vec::new());
        let boundary = navigation_boundary_from_goal(&public).expect("public goal");
        assert!(boundary.is_none());
        let map = decode_astar_costmap_with_boundary(&free_grid_body(), boundary.as_ref())
            .expect("public map");
        assert!((0..map.height()).all(|y| (0..map.width()).all(|x| map.get(x, y) == 0)));

        let path = Path2D::stamped(
            NavFrameId::Map,
            Timestamp::default(),
            vec![Point2::new(0.5, 0.5), Point2::new(6.5, 6.5)],
        );
        assert!(path_respects_navigation_boundary(
            &path,
            boundary.as_ref(),
            0.5,
        ));
    }
