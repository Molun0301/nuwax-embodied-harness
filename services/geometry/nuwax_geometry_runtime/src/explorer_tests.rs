use super::*;

// 进度行为测试复用真正的波前源，固定外形只在这一组串行测试中安装。
static ENV: std::sync::Mutex<()> = std::sync::Mutex::new(());
struct GeometryEnv(Option<String>);
impl GeometryEnv {
    fn install() -> Self {
        let old = std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").ok();
        std::env::set_var(
            "NUWAX_ROBOT_GEOMETRY_JSON",
            r#"{"footprint_xy_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],"z_min_m":-0.2,"z_max_m":0.2,"source":"探索行为测试"}"#,
        );
        Self(old)
    }
}
impl Drop for GeometryEnv {
    fn drop(&mut self) {
        match &self.0 {
            Some(value) => std::env::set_var("NUWAX_ROBOT_GEOMETRY_JSON", value),
            None => std::env::remove_var("NUWAX_ROBOT_GEOMETRY_JSON"),
        }
    }
}
fn request() -> proto::ExplorationRequest {
    proto::ExplorationRequest {
        motion_generation: 17,
        max_navigation_retries: 1,
        blacklist_radius_m: 0.5,
        ..Default::default()
    }
}
fn pose() -> proto::Pose {
    from_mission(
        &MissionPose {
            position: [2.0, 2.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        },
        "map",
    )
}
fn grid() -> proto::OccupancyGrid {
    let mut data = vec![255_u8; 80 * 80];
    for y in 10..60 {
        for x in 10..60 {
            data[y * 80 + x] = 0;
        }
    }
    proto::OccupancyGrid {
        map_name: "住宅".into(),
        meta: Some(proto::OccupancyGridMeta {
            width: 80,
            height: 80,
            resolution: 0.1,
            origin_x: 0.0,
            origin_y: 0.0,
            frame_id: "map".into(),
            timestamp_ns: unix_ns(),
        }),
        data,
    }
}
#[test]
fn paused_progress_restores_source_history_and_counts_in_same_map() {
    let _lock = ENV.lock().unwrap();
    let _env = GeometryEnv::install();
    let mut explorer = Explorer::new("住宅".into(), "map".into(), false).unwrap();
    explorer.start("first".into(), request()).unwrap();
    let tick = explorer.tick(Some(pose()), Some(grid()), None).unwrap();
    let (goal_id, _) = tick.goal.expect("实际波前选择应产生探索目标");
    explorer
        .navigation_feedback(&proto::NavigationStatus {
            goal_id,
            status: proto::GoalStatus::Succeeded as i32,
            ..Default::default()
        })
        .unwrap();
    explorer.stop().unwrap();
    let progress = explorer.progress().unwrap();
    let saved: Progress = serde_json::from_str(&progress.progress_json).unwrap();
    assert_eq!(saved.reached_goals, 1);
    assert!(!saved.sources.is_empty());
    let mut resumed = Explorer::new("住宅".into(), "map".into(), false).unwrap();
    let mut next = request();
    next.resume_progress_json = progress.progress_json.clone();
    let status = resumed.start("second".into(), next).unwrap();
    assert_eq!(status.goals_completed, 1);
    let restored: Progress =
        serde_json::from_str(&resumed.progress().unwrap().progress_json).unwrap();
    assert_eq!(restored.sources[0].payload, saved.sources[0].payload);
    let mut wrong_map = Explorer::new("其它地图".into(), "map".into(), false).unwrap();
    let mut next = request();
    next.resume_progress_json = progress.progress_json.clone();
    assert_eq!(
        wrong_map.start("wrong".into(), next).unwrap_err().code(),
        tonic::Code::FailedPrecondition
    );
}
#[test]
fn stopped_child_ends_exploration_and_duration_completion_cancels_child() {
    let _lock = ENV.lock().unwrap();
    let _env = GeometryEnv::install();
    let mut explorer = Explorer::new("住宅".into(), "map".into(), false).unwrap();
    explorer.start("mission".into(), request()).unwrap();
    let tick = explorer.tick(Some(pose()), Some(grid()), None).unwrap();
    let (id, _) = tick.goal.unwrap();
    explorer
        .navigation_feedback(&proto::NavigationStatus {
            goal_id: id,
            status: proto::GoalStatus::Stopped as i32,
            ..Default::default()
        })
        .unwrap();
    assert_eq!(
        explorer.tick(None, None, None).unwrap().status.phase,
        proto::ExplorationPhase::Stopped as i32
    );
    let mut next = request();
    next.stop_conditions = Some(proto::ExplorationStopConditions {
        max_goals: Some(0),
        ..Default::default()
    });
    explorer.start("done".into(), next).unwrap();
    assert_eq!(
        explorer
            .tick(Some(pose()), Some(grid()), None)
            .unwrap()
            .status
            .phase,
        proto::ExplorationPhase::Succeeded as i32
    );
}

#[test]
fn stale_pose_waits_and_fresh_pose_resumes_without_new_start() {
    let _lock = ENV.lock().unwrap();
    let _env = GeometryEnv::install();
    let mut explorer = Explorer::new("住宅".into(), "map".into(), false).unwrap();
    explorer.start("recovery".into(), request()).unwrap();
    let mut old = pose();
    old.timestamp_ns = unix_ns() - 2_000_000_000;
    let waiting = explorer.tick(Some(old), Some(grid()), None).unwrap();
    assert!(waiting.goal.is_none());
    assert_eq!(
        waiting.status.phase,
        proto::ExplorationPhase::Running as i32
    );
    let resumed = explorer.tick(Some(pose()), Some(grid()), None).unwrap();
    assert!(resumed.goal.is_some());
    assert_eq!(resumed.status.exploration_id, "recovery");
}

#[test]
fn forbidden_mask_changes_surface_input_without_changing_physical_snapshot() {
    let points: Vec<_> = (0..30)
        .flat_map(|x| (0..30).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.0)))
        .collect();
    let physical = Arc::new(
        SurfaceSnapshot::from_points(
            &points,
            SurfaceNavigationConfig::new(
                "map".into(),
                RobotGeometry {
                    self_collision: None,
                    footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
                    z_min_m: -0.2,
                    z_max_m: 0.2,
                    source: "禁区行为测试".into(),
                },
                0.1,
                0.15,
                18.0,
                0.2,
            )
            .unwrap()
            .geometry,
            0.1,
        )
        .unwrap(),
    );
    let mask = proto::OccupancyGrid {
        map_name: "住宅".into(),
        meta: Some(proto::OccupancyGridMeta {
            width: 1,
            height: 1,
            resolution: 0.1,
            origin_x: 1.0,
            origin_y: 1.0,
            frame_id: "map".into(),
            timestamp_ns: unix_ns(),
        }),
        data: vec![100],
    };
    assert!(physical.navigation_column_allowed([10, 10], 0.1));
    let effective = apply_forbidden(physical.clone(), Some(mask)).unwrap();
    assert!(!effective.navigation_column_allowed([10, 10], 0.1));
    assert!(physical.navigation_column_allowed([10, 10], 0.1));
}
