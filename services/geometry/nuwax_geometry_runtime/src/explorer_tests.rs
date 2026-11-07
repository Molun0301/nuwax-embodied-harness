use super::*;

const TEST_GEOMETRY:&str=r#"{"footprint_xy_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],"z_min_m":-0.2,"z_max_m":0.2,"source":"探索行为测试"}"#;
fn make_explorer(
    map_name: String,
    map_frame: String,
    surface_mode: bool,
) -> Result<Explorer, Status> {
    let n = nuwax_config::test_support::section("navigation");
    let r = nuwax_config::test_support::section("robot");
    let d = nuwax_config::sections::Derived {
        nav_profile: "pct-scan-3d".into(),
        actuator_max_linear_mps: 0.0,
        actuator_max_yaw_rps: 0.0,
        actuator_motion_model: String::new(),
        robot_geometry_json: TEST_GEOMETRY.into(),
    };
    Explorer::from_config(map_name, map_frame, surface_mode, &n, &r, &d)
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
    let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
    explorer.start("first".into(), request()).unwrap();
    let tick = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
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
    let mut resumed = make_explorer("住宅".into(), "map".into(), false).unwrap();
    let mut next = request();
    next.resume_progress_json = progress.progress_json.clone();
    let status = resumed.start("second".into(), next).unwrap();
    assert_eq!(status.goals_completed, 1);
    let restored: Progress =
        serde_json::from_str(&resumed.progress().unwrap().progress_json).unwrap();
    assert_eq!(restored.sources[0].payload, saved.sources[0].payload);
    let mut wrong_map = make_explorer("其它地图".into(), "map".into(), false).unwrap();
    let mut next = request();
    next.resume_progress_json = progress.progress_json.clone();
    assert_eq!(
        wrong_map.start("wrong".into(), next).unwrap_err().code(),
        tonic::Code::FailedPrecondition
    );
}
#[test]
fn stopped_child_ends_exploration_and_duration_completion_cancels_child() {
    let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
    explorer.start("mission".into(), request()).unwrap();
    let tick = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    let (id, _) = tick.goal.unwrap();
    explorer
        .navigation_feedback(&proto::NavigationStatus {
            goal_id: id,
            status: proto::GoalStatus::Stopped as i32,
            ..Default::default()
        })
        .unwrap();
    assert_eq!(
        explorer.tick(None, None, Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap().status.phase,
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
            .tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
            .unwrap()
            .status
            .phase,
        proto::ExplorationPhase::Succeeded as i32
    );
}

#[test]
fn stale_pose_waits_and_fresh_pose_resumes_without_new_start() {
    let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
    explorer.start("recovery".into(), request()).unwrap();
    let mut old = pose();
    old.timestamp_ns = unix_ns() - 2_000_000_000;
    let waiting = explorer.tick(Some(old), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(waiting.goal.is_none());
    assert!(waiting.status.message.contains("等待地图位姿"));
    assert!(waiting.status.message.contains("来源年龄="));
    assert_eq!(
        waiting.status.phase,
        proto::ExplorationPhase::Running as i32
    );
    let resumed = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
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

fn started_with_max_goals(max: u32) -> Explorer {
    let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
    let mut start = request();
    start.stop_conditions = Some(proto::ExplorationStopConditions {
        max_goals: Some(max),
        ..Default::default()
    });
    explorer.start("行为验收".into(), start).unwrap();
    explorer
}
fn feedback(explorer: &mut Explorer, id: String, status: proto::GoalStatus) {
    explorer
        .navigation_feedback(&proto::NavigationStatus {
            goal_id: id,
            status: status as i32,
            message: "真实调度结果".into(),
            ..Default::default()
        })
        .unwrap();
}
#[test]
fn max_goals_waits_for_active_child_terminal_before_stopping() {
    let mut explorer = started_with_max_goals(1);
    let first = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    let (id, _) = first.goal.unwrap();
    let waiting = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(waiting.goal.is_none() && waiting.cancel_goal.is_none());
    assert_eq!(waiting.status.goals_completed, 0);
    assert_eq!(
        waiting.status.phase,
        proto::ExplorationPhase::Running as i32
    );
    feedback(&mut explorer, id, proto::GoalStatus::Succeeded);
    let done = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(done.status.phase, proto::ExplorationPhase::Succeeded as i32);
    assert_eq!(
        explorer.stop_reason,
        Some(ExplorationStopReason::MaxGoalsReached)
    );
    assert!(done.goal.is_none() && explorer.child.is_none());
}
#[test]
fn max_goals_does_not_count_failed_child_as_reached() {
    let mut explorer = started_with_max_goals(1);
    let (id, _) = explorer
        .tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap()
        .goal
        .unwrap();
    feedback(&mut explorer, id, proto::GoalStatus::Failed);
    assert_eq!(explorer.reached_goals, 0);
    let next = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_ne!(
        explorer.stop_reason,
        Some(ExplorationStopReason::MaxGoalsReached)
    );
    assert_ne!(next.status.phase, proto::ExplorationPhase::Succeeded as i32);
}
#[test]
fn coverage_stop_counts_only_cells_inside_requested_boundary() {
    for inside_known in [false, true] {
        let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
        let mut request = request();
        request.boundary = [[0., 0.], [2., 0.], [2., 2.], [0., 2.]]
            .into_iter()
            .enumerate()
            .map(|(i, a)| {
                let b = [[0., 0.], [2., 0.], [2., 2.], [0., 2.]][(i + 1) % 4];
                proto::ExplorationBoundarySegment {
                    start_x: a[0],
                    start_y: a[1],
                    end_x: b[0],
                    end_y: b[1],
                }
            })
            .collect();
        request.stop_conditions = Some(proto::ExplorationStopConditions {
            max_coverage_pct: Some(50.),
            ..Default::default()
        });
        explorer.start("边界覆盖率".into(), request).unwrap();
        let mut observed = grid();
        observed.data.fill(if inside_known { 255 } else { 0 });
        for y in 0..20 {
            for x in 0..20 {
                observed.data[y * 80 + x] = if inside_known { 0 } else { 255 };
            }
        }
        let update = explorer.tick(Some(pose()), Some(observed), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
        assert_eq!(
            explorer.stop_reason == Some(ExplorationStopReason::CoverageReached),
            inside_known,
            "{}",
            update.status.message
        );
    }
}
#[test]
fn default_no_frontier_stability_window_finishes_with_fresh_map_input() {
    let mut explorer = started_with_max_goals(10);
    let mut complete = grid();
    complete.data.fill(0);
    let first = explorer
        .tick(Some(pose()), Some(complete.clone()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap();
    assert!(first.goal.is_none());
    assert_eq!(first.status.phase, proto::ExplorationPhase::Running as i32);
    explorer.no_frontier_since = Some(Instant::now() - Duration::from_secs(6));
    complete.meta.as_mut().unwrap().timestamp_ns = unix_ns();
    let done = explorer.tick(Some(pose()), Some(complete), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(done.status.phase, proto::ExplorationPhase::Succeeded as i32);
    assert_eq!(
        explorer.stop_reason,
        Some(ExplorationStopReason::NoFrontierTimeout)
    );
}
#[test]
fn unavailable_or_stale_map_never_completes_no_frontier_window() {
    let mut explorer = started_with_max_goals(10);
    let mut complete = grid();
    complete.data.fill(0);
    explorer
        .tick(Some(pose()), Some(complete.clone()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap();
    explorer.no_frontier_since = Some(Instant::now() - Duration::from_secs(6));
    let mut stale = complete.clone();
    stale.meta.as_mut().unwrap().timestamp_ns = unix_ns() - 3_000_000_000;
    let waiting = explorer.tick(Some(pose()), Some(stale), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(
        waiting.status.phase,
        proto::ExplorationPhase::Running as i32
    );
    assert!(explorer.no_frontier_since.is_none());
    assert!(waiting.status.message.contains("等待占据栅格"));
    assert!(waiting.status.message.contains("来源年龄="));
    let fresh = explorer.tick(Some(pose()), Some(complete), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(fresh.status.phase, proto::ExplorationPhase::Running as i32);
}
#[test]
fn costmap_refresh_keeps_active_child_and_next_goal_progresses() {
    let mut explorer = started_with_max_goals(2);
    let (first, _) = explorer
        .tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap()
        .goal
        .unwrap();
    let mut refreshed = grid();
    refreshed.data[30 * 80 + 30] = 100;
    let update = explorer.tick(Some(pose()), Some(refreshed), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(update.goal.is_none() && update.cancel_goal.is_none());
    assert_eq!(explorer.child.as_ref().unwrap().id, first);
    feedback(&mut explorer, first, proto::GoalStatus::Succeeded);
    let next = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(next.goal.is_some(), "{}", next.status.message);
    assert_eq!(next.status.goals_completed, 1);
}
fn enable_mission(explorer: &mut Explorer, maximum: Option<f64>) {
    explorer.config.as_mut().unwrap().return_policy = ReturnPolicy {
        maximum_distance_m: maximum,
        reserve_distance_m: 0.,
        return_on_completion: true,
    };
    explorer.calibration = Some(MissionCalibration {
        planar: nuwax_nav_simple_planner::SimplePlannerConfig {
            robot_geometry: Some(RobotGeometry::from_runtime_json(TEST_GEOMETRY).unwrap()),
            ..Default::default()
        },
    });
}
#[test]
fn mission_budget_dispatches_home_and_requires_matching_terminal() {
    let mut explorer = started_with_max_goals(10);
    enable_mission(&mut explorer, Some(0.01));
    let selected = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(selected.goal.is_none());
    let returned = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    let (id, target) = returned.goal.expect("余额不足后应派回程");
    assert!(returned.status.returning);
    assert_eq!(target.pose.unwrap().position, pose().position);
    assert_eq!(target.position_tolerance_m, Some(0.1));
    assert_eq!(target.yaw_tolerance_rad, Some(0.2));
    feedback(
        &mut explorer,
        "旧子目标".into(),
        proto::GoalStatus::Succeeded,
    );
    assert!(explorer.child.is_some());
    feedback(&mut explorer, id, proto::GoalStatus::Succeeded);
    assert_eq!(
        explorer.status("").phase,
        proto::ExplorationPhase::Succeeded as i32
    );
    assert_eq!(
        explorer.stop_reason,
        Some(ExplorationStopReason::DistanceBudgetReached)
    );
}
#[test]
fn mission_active_goal_reserves_current_return_route_before_limit() {
    let mut explorer = started_with_max_goals(10);
    let (old, _) = explorer
        .tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap()
        .goal
        .unwrap();
    enable_mission(&mut explorer, Some(1.5));
    let mut mission = MissionProgress::new(MissionPose {
        position: [1.3, 2., 0.],
        orientation_xyzw: [0., 0., 0., 1.],
    })
    .unwrap();
    mission.travelled_m = 1.;
    explorer.mission = Some(mission);
    let returned = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(returned.cancel_goal, Some(old));
    assert!(returned.goal.is_some() && returned.status.returning);
    assert_eq!(
        explorer.return_reason,
        Some(ExplorationStopReason::DistanceBudgetReached)
    );
}
#[test]
fn mission_unreachable_home_fails_without_return_dispatch() {
    let mut explorer = started_with_max_goals(10);
    enable_mission(&mut explorer, None);
    explorer.mission = Some(
        MissionProgress::new(MissionPose {
            position: [7., 7., 0.],
            orientation_xyzw: [0., 0., 0., 1.],
        })
        .unwrap(),
    );
    let failed = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(failed.status.phase, proto::ExplorationPhase::Failed as i32);
    assert!(failed.goal.is_none() && explorer.child.is_none());
}
#[test]
fn mission_return_failures_exhaust_original_retry_budget() {
    let mut explorer = started_with_max_goals(10);
    enable_mission(&mut explorer, Some(0.01));
    explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    for attempt in 0..2 {
        let returned = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
        let (id, _) = returned.goal.unwrap();
        feedback(&mut explorer, id, proto::GoalStatus::Failed);
        assert_eq!(
            explorer.status("").phase,
            if attempt == 0 {
                proto::ExplorationPhase::Running as i32
            } else {
                proto::ExplorationPhase::Failed as i32
            }
        );
    }
}
#[test]
fn user_stop_after_costmap_update_clears_active_child_and_keeps_progress() {
    let mut explorer = started_with_max_goals(10);
    explorer
        .tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap()
        .goal
        .unwrap();
    let mut refreshed = grid();
    refreshed.data[50 * 80 + 50] = 100;
    explorer.tick(Some(pose()), Some(refreshed), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert_eq!(
        explorer.stop().unwrap().phase,
        proto::ExplorationPhase::Stopped as i32
    );
    assert!(explorer.child.is_none());
    assert!(!explorer.progress().unwrap().progress_json.is_empty());
}
#[test]
fn permanent_dispatch_rejection_still_applies_goal_failure_policy() {
    let mut explorer = started_with_max_goals(1);
    let (id, _) = explorer
        .tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(()))
        .unwrap()
        .goal
        .unwrap();
    feedback(&mut explorer, id, proto::GoalStatus::Failed);
    let progress: Progress =
        serde_json::from_str(&explorer.progress().unwrap().progress_json).unwrap();
    assert_eq!(progress.reached_goals, 0);
    let history: serde_json::Value = serde_json::from_str(&progress.sources[0].payload).unwrap();
    assert!(
        !history[1]["failed_goals"].as_array().unwrap().is_empty(),
        "{history}"
    );
}

pub(crate) fn blocking_fixture(
    entered: tokio::sync::oneshot::Sender<()>,
    release: std::sync::mpsc::Receiver<()>,
) -> Explorer {
    struct Source {
        entered: Option<tokio::sync::oneshot::Sender<()>>,
        release: std::sync::mpsc::Receiver<()>,
    }
    impl nuwax_exploration::ExplorationGoalSource for Source {
        fn source_name(&self) -> std::borrow::Cow<'static, str> {
            "wavefront".into()
        }
        fn begin(&mut self) -> nuwax_exploration::ExploreResult<()> {
            Ok(())
        }
        fn end(&mut self) -> nuwax_exploration::ExploreResult<()> {
            Ok(())
        }
        fn poll_goal(
            &mut self,
            _: &ExplorationStateSnapshot<'_>,
        ) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
            if let Some(entered) = self.entered.take() {
                let _ = entered.send(());
                self.release.recv().unwrap();
            }
            Ok(ExplorationPoll::NoFrontier)
        }
        fn checkpoint(&self) -> nuwax_exploration::ExploreResult<String> {
            Ok("{}".into())
        }
        fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
            nuwax_exploration::ExplorationSourceDiagnostics {
                source_name: "wavefront".into(),
                is_active: true,
                last_goal_id: None,
                last_error: None,
            }
        }
    }
    let mut explorer = make_explorer("未保存图".into(), "map".into(), false).unwrap();
    explorer.id = "阻塞计算".into();
    explorer.started = Some(Instant::now());
    explorer.config = Some(ExploreStartConfig::default());
    explorer.controller.add_source(Source {
        entered: Some(entered),
        release,
    });
    explorer.controller.begin_exploration().unwrap();
    explorer
}

#[test]
fn mission_unreachable_candidate_is_not_budget_exhaustion() {
    struct Candidate {
        failures: Arc<std::sync::Mutex<Vec<ExplorationGoalFeedback>>>,
    }
    impl nuwax_exploration::ExplorationGoalSource for Candidate {
        fn source_name(&self) -> std::borrow::Cow<'static, str> {
            "wavefront".into()
        }
        fn begin(&mut self) -> nuwax_exploration::ExploreResult<()> {
            Ok(())
        }
        fn end(&mut self) -> nuwax_exploration::ExploreResult<()> {
            Ok(())
        }
        fn poll_goal(
            &mut self,
            state: &ExplorationStateSnapshot<'_>,
        ) -> nuwax_exploration::ExploreResult<ExplorationPoll> {
            let mut target = state.robot_pose.clone();
            target.pose.position.x = if self.failures.lock().unwrap().is_empty() {
                7.
            } else {
                2.5
            };
            Ok(ExplorationPoll::Goal(
                nuwax_nav_types_messages::visual::FrontierGoal::new("候选", target, 1.),
            ))
        }
        fn report_goal_feedback(
            &mut self,
            _: &str,
            feedback: ExplorationGoalFeedback,
        ) -> nuwax_exploration::ExploreResult<()> {
            self.failures.lock().unwrap().push(feedback);
            Ok(())
        }
        fn diagnostics(&self) -> nuwax_exploration::ExplorationSourceDiagnostics {
            nuwax_exploration::ExplorationSourceDiagnostics {
                source_name: "wavefront".into(),
                is_active: true,
                last_goal_id: None,
                last_error: None,
            }
        }
    }
    let mut explorer = started_with_max_goals(10);
    enable_mission(&mut explorer, Some(100.));
    let failures = Arc::new(std::sync::Mutex::new(Vec::new()));
    explorer.controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    explorer.controller.add_source(Candidate {
        failures: failures.clone(),
    });
    explorer.controller.begin_exploration().unwrap();
    let unreachable = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(unreachable.goal.is_none());
    assert!(!unreachable.status.returning);
    assert!(matches!(
        failures.lock().unwrap()[0],
        ExplorationGoalFeedback::Failed(_)
    ));
    let reachable = explorer.tick(Some(pose()), Some(grid()), Err("曲面快照尚未生成，来源年龄未知".into()), Ok(())).unwrap();
    assert!(reachable.goal.is_some(), "{}", reachable.status.message);
    assert!(!reachable.status.returning);
}
#[test]
fn surface_goals_preserve_real_floor_height_arrival_heading_and_source() {
    let config = SurfaceNavigationConfig::new(
        "map".into(),
        RobotGeometry {
            self_collision: None,
            footprint_xy_m: vec![[-0.1, -0.1], [0.1, -0.1], [0.1, 0.1], [-0.1, 0.1]],
            z_min_m: -0.5,
            z_max_m: 0.5,
            source: "原曲面行为样例".into(),
        },
        0.1,
        0.2,
        25.,
        0.3,
    )
    .unwrap();
    let points: Vec<_> = [0., 3.]
        .into_iter()
        .flat_map(|level| {
            (0..20).flat_map(move |x| {
                (0..20).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, level + 0.05))
            })
        })
        .collect();
    let mut geometry = config.geometry.clone();
    geometry.robot_height = 0.45;
    let surface = Arc::new(SurfaceSnapshot::from_points(&points, geometry, 0.1).unwrap());
    for floor in [0., 3.] {
        let mut explorer = started_with_max_goals(10);
        explorer.surface_config = Some(config.clone());
        let mut source = SurfaceGoalSourceAdapter::new(
            "surface",
            config.follower.body_to_support_z_m,
            explorer.surface_input.clone(),
        )
        .unwrap();
        source.configure_constraints(None, 1, 0.5).unwrap();
        explorer.controller =
            ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
        explorer.controller.add_source(source);
        explorer.controller.begin_exploration().unwrap();
        let mut position = pose();
        position.position = Some(proto::Vec3 {
            x: 1.05,
            y: 1.05,
            z: floor + 0.55,
        });
        position.orientation = Some(proto::Quat {
            x: 0.05_f64.sin(),
            y: 0.,
            z: 0.,
            w: 0.05_f64.cos(),
        });
        let update = explorer
            .tick(Some(position.clone()), Some(grid()), Ok(surface.clone()), Ok(()))
            .unwrap();
        let (_, target) = update.goal.expect("实际曲面源应产生目标");
        let target = target.pose.unwrap();
        assert!(
            target.position.as_ref().unwrap().z > floor + 0.4
                && target.position.as_ref().unwrap().z < floor + 0.7
        );
        let arrival = target.orientation.unwrap();
        assert_eq!((arrival.x, arrival.y), (0., 0.));
        assert!(((arrival.z * arrival.z + arrival.w * arrival.w).sqrt() - 1.).abs() < 1e-12);
        assert_ne!(Some(arrival), position.orientation);
        assert_eq!(target.frame_id, "map");
        assert_eq!(
            explorer.controller.diagnostics().selected_source.as_deref(),
            Some("surface")
        );
    }
}

#[test]
fn travel_counts_without_return_calibration_and_includes_position_jumps() {
    for returning in [false, true] {
        let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
        explorer.start("里程".into(), request()).unwrap();
        if returning { enable_mission(&mut explorer, None); }
        let stamp = unix_ns();
        let sample = |x, offset| {
            let mut sample = pose();
            sample.timestamp_ns = stamp + offset;
            sample.position.as_mut().unwrap().x = x;
            sample
        };
        let first = explorer.tick(Some(sample(2., 0)), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
        assert_eq!(first.status.traveled_distance_m, 0.);
        let second = explorer.tick(Some(sample(2.5, 1)), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
        assert_eq!(second.status.traveled_distance_m, 0.5);
        assert_eq!(second.status.phase, proto::ExplorationPhase::Running as i32);
        let jump = explorer.tick(Some(sample(4., 2)), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
        assert_eq!(jump.status.traveled_distance_m, 2.);
        assert_eq!(jump.status.phase, proto::ExplorationPhase::Running as i32);
        let saved: Progress = serde_json::from_str(&explorer.progress().unwrap().progress_json).unwrap();
        assert_eq!(saved.mission.unwrap().travelled_m, 2.);
    }
}

#[test]
fn forbidden_input_wait_does_not_finish_or_cancel_exploration() {
    let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
    explorer.start("等待禁区".into(), request()).unwrap();
    let first = explorer.tick(Some(pose()), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
    let child = first.goal.unwrap().0;
    explorer.input_wait_since = Some(Instant::now() - Duration::from_secs(31));
    explorer.no_frontier_since = Some(Instant::now() - Duration::from_secs(6));
    let waiting = explorer.tick(None, None, Err("禁行区格式无效".into()), Err("禁行区格式无效".into())).unwrap();
    assert_eq!(waiting.status.phase, proto::ExplorationPhase::Running as i32);
    assert!(waiting.status.message.contains("等待禁行区输入：禁行区格式无效"));
    assert!(waiting.goal.is_none() && waiting.cancel_goal.is_none());
    assert_eq!(explorer.child.as_ref().unwrap().id, child);
    assert!(explorer.input_wait_since.is_none() && explorer.no_frontier_since.is_none());
    explorer.config.as_mut().unwrap().stop_conditions.max_duration_ms = Some(0);
    let stopped = explorer.tick(None, None, Err("禁行区格式无效".into()), Err("禁行区格式无效".into())).unwrap();
    assert_eq!(stopped.status.phase, proto::ExplorationPhase::Succeeded as i32);
    assert_eq!(stopped.cancel_goal.as_deref(), Some(child.as_str()), "输入等待不能绕过已配置的停止条件");
}

#[test]
fn frontier_count_keeps_last_completed_computation_while_selection_is_pending() {
    use nuwax_exploration::{ExplorationGoalSource, ExplorationSourceDiagnostics, ExploreResult};
    use nuwax_nav_types_messages::visual::FrontierGoal;
    struct Source { step: usize, frontiers: Vec<FrontierGoal> }
    impl ExplorationGoalSource for Source {
        fn source_name(&self) -> std::borrow::Cow<'static, str> { "wavefront".into() }
        fn begin(&mut self) -> ExploreResult<()> { Ok(()) }
        fn end(&mut self) -> ExploreResult<()> { Ok(()) }
        fn poll_goal(&mut self, state: &ExplorationStateSnapshot<'_>) -> ExploreResult<ExplorationPoll> {
            self.step += 1;
            if self.step == 1 {
                let goal = FrontierGoal::new("前沿", state.robot_pose.clone(), 1.);
                self.frontiers = vec![goal.clone(), goal.clone()];
                Ok(ExplorationPoll::Goal(goal))
            } else {
                self.frontiers.clear();
                Ok(if self.step == 2 { ExplorationPoll::Pending } else { ExplorationPoll::NoFrontier })
            }
        }
        fn frontier_snapshot(&self) -> Vec<FrontierGoal> { self.frontiers.clone() }
        fn diagnostics(&self) -> ExplorationSourceDiagnostics {
            ExplorationSourceDiagnostics { source_name: "wavefront".into(), is_active: true, last_goal_id: None, last_error: None }
        }
    }
    let mut explorer = make_explorer("住宅".into(), "map".into(), false).unwrap();
    explorer.start("前沿数量".into(), request()).unwrap();
    explorer.controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    explorer.controller.add_source(Source { step: 0, frontiers: Vec::new() });
    explorer.controller.begin_exploration().unwrap();
    let complete = explorer.tick(Some(pose()), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
    assert_eq!(complete.status.frontier_count, 2);
    feedback(&mut explorer, complete.goal.unwrap().0, proto::GoalStatus::Succeeded);
    let pending = explorer.tick(Some(pose()), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
    assert_eq!(pending.status.message, "正在选择前沿");
    assert!(explorer.controller.frontier_snapshot().is_empty());
    assert_eq!(pending.status.frontier_count, 2);
    let empty = explorer.tick(Some(pose()), Some(grid()), Err("二维地图".into()), Ok(())).unwrap();
    assert_eq!(empty.status.frontier_count, 0);
    assert_eq!(empty.status.phase, proto::ExplorationPhase::Running as i32);
}
