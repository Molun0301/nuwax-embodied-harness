// Test-only frozen sequential oracle from 6d97f1175866b9eabdcb0b73933129f4271e9dc3.
// The original poll, unavailable helper, and atomic observation helper are copied
// verbatim; only poll_goal is renamed to legacy_poll_goal. Do not make this oracle
// call evaluate_poll or the prepared source APIs. Included inside surface_adapter::tests.

impl SurfaceInputReceiver {
    fn observation(&self) -> ExploreResult<(Option<Arc<SurfaceSnapshot>>, u64)> {
        let current = self.current.read().map_err(|_| input_lock_error())?;
        Ok((current.clone(), self.notifications.load(Ordering::Acquire)))
    }

}

impl SurfaceGoalSourceAdapter {
    fn unavailable(&mut self, reason: impl Into<String>) -> ExplorationPoll {
        let reason = reason.into();
        self.last_error = Some(reason.clone());
        self.last_frontiers.clear();
        ExplorationPoll::SourceUnavailable(reason)
    }

    fn legacy_poll_goal(
        &mut self,
        state: &ExplorationStateSnapshot<'_>,
    ) -> ExploreResult<ExplorationPoll> {
        if !self.active {
            return Ok(self.unavailable("surface source is not active"));
        }
        // 精确导航反馈之前不能另派目标，输入撤销由根拥有方取消该子导航。
        if self.selected.is_some() {
            return Ok(ExplorationPoll::Pending);
        }
        let (snapshot, notification) = self.receiver.observation()?;
        let Some(snapshot) = snapshot else {
            return Ok(self.unavailable("surface map input is unavailable"));
        };
        if state.robot_pose.frame_id.as_str() != snapshot.config().world_frame {
            return Ok(self.unavailable("body pose frame does not match surface map frame"));
        }
        let position = state.robot_pose.pose.position;
        let orientation = state.robot_pose.pose.orientation;
        let quaternion = [orientation.x, orientation.y, orientation.z, orientation.w];
        let source_snapshot = Arc::clone(&snapshot);
        let same_geometry = self
            .last_snapshot
            .as_ref()
            .is_some_and(|previous| Weak::ptr_eq(previous, &Arc::downgrade(&source_snapshot)));
        // 当前占据区依赖实际机体高度；层内接触可能关联同一层顶，缓存不能
        // 因此合并不同的真实位置、姿态或仅比较所在格。
        let same_query = self.last_query_body == Some([position.x, position.y, position.z])
            && self.last_query_orientation == Some(quaternion);
        if same_geometry && same_query && self.last_no_frontier {
            self.last_notification = Some(notification);
            self.last_error = None;
            return Ok(ExplorationPoll::NoFrontier);
        }
        let new_snapshot = self.last_notification != Some(notification) || !same_geometry;
        if !new_snapshot && same_query && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        let boundary: Vec<_> = self.boundary.as_ref().map_or_else(Vec::new, |boundary| {
            boundary
                .vertices()
                .iter()
                .map(|point| [point.x, point.y])
                .collect()
        });
        // 几何索引留在环境快照上，供下一次机体视图和地图增量继承。
        // 未变化的等待查询在上述通知检查处返回，不重建当前足迹。
        snapshot.prepare_frontier_geometry(&boundary)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let snapshot = match snapshot.with_current_body_support(
            [position.x, position.y, position.z], quaternion,
        ) {
            Ok(view) => Arc::new(view),
            Err(error) => return Ok(self.unavailable(format!("current body support evidence: {error}"))),
        };
        let robot = match snapshot.locate_support_from_body(
            [position.x, position.y, position.z],
            quaternion,
            self.body_to_support_z_m,
        ) {
            Ok(location) => location,
            Err(error) => {
                return Ok(
                    self.unavailable(format!("body pose has no valid surface support: {error}"))
                )
            }
        };
        if !new_snapshot && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        let mut candidates = match snapshot.reachable_frontiers(robot.support_xyz, &boundary) {
            Ok(candidates) => candidates,
            Err(error) => {
                return Ok(
                    self.unavailable(format!("surface frontier query is unavailable: {error}"))
                )
            }
        };
        self.last_snapshot = Some(Arc::downgrade(&source_snapshot));
        self.last_notification = Some(notification);
        self.last_query_body = Some([position.x, position.y, position.z]);
        self.last_query_orientation = Some(quaternion);
        self.last_frontiers.clear();
        let minimum_distance = f64::from(
            snapshot
                .config()
                .goal_tolerance
                .max(snapshot.config().voxel_size),
        );
        // 默认3米策略半径不能跨越另一楼层；高度只覆盖局部可跨台阶及量化差。
        let vertical_radius =
            f64::from(snapshot.config().step_threshold_m) + f64::from(snapshot.config().voxel_size);
        let observation_targets = if self.observation_model.is_some() {
            snapshot
                .observation_targets(&boundary)
                .map_err(|error| ExploreError::SourceError(error.to_string()))?
        } else {
            Vec::new()
        };
        let geometric_candidate_count=candidates.len();
        if !new_snapshot {
            candidates.retain(|candidate|self.retry_support.is_some_and(|retry|
                distance(retry,candidate.support_xyz)<=f64::from(snapshot.config().voxel_size)*0.25));
        }
        let retry_matches=|point:[f64;3]|self.retry_support.is_some_and(|retry|
            distance(retry,point)<=f64::from(snapshot.config().voxel_size)*0.25);
        candidates.sort_by(|a,b| {
            retry_matches(b.support_xyz).cmp(&retry_matches(a.support_xyz))
                .then_with(||(b.gain/(1.0+b.travel_cost)).total_cmp(&(a.gain/(1.0+a.travel_cost))))
                .then_with(||a.support_xyz[0].total_cmp(&b.support_xyz[0]))
                .then_with(||a.support_xyz[1].total_cmp(&b.support_xyz[1]))
                .then_with(||a.support_xyz[2].total_cmp(&b.support_xyz[2]))
        });
        let mut admissible = Vec::new();
        let mut accepted_regions=std::collections::BTreeSet::new();
        let mut evaluated_candidate_count=0;
        for mut candidate in candidates {
            if self.excluded(candidate.support_xyz, vertical_radius)
                || distance(robot.support_xyz, candidate.support_xyz) <= minimum_distance
            {
                continue;
            }
            let region=nuwax_explore_routing::region_key(candidate.support_xyz);
            // 无视域标定时收益不会再改变；同区域后续候选必然被区域路由丢弃。
            // 首个候选无效时继续检查下一项，不能把候选上限当作无前沿证明。
            if self.observation_model.is_none() && accepted_regions.contains(&region)
                && !retry_matches(candidate.support_xyz) {continue;}
            evaluated_candidate_count+=1;
            let base_yaw = (candidate.support_xyz[1] - robot.support_xyz[1])
                .atan2(candidate.support_xyz[0] - robot.support_xyz[0]);
            let directional_view = self.observation_model.as_ref().is_some_and(|model| {
                model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6
            });
            // 机体能否停留与传感器视场独立；直指前沿不适合时仍尝试沿边缘停留。
            let turns = if snapshot.body_envelope().is_some() || directional_view { 4 } else { 1 };
            let mut best = None;
            for i in 0..turns {
                let yaw = base_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                let candidate_quaternion = if snapshot.body_envelope().is_some() {
                    match snapshot.body_orientation_at(candidate.support_xyz, yaw) {
                        Ok(q) => q,
                        Err(_) => continue,
                    }
                } else if self.observation_model.is_some() {
                    [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]
                } else {
                    quaternion
                };
                let body = match snapshot.body_position_from_support(
                    candidate.support_xyz,
                    candidate_quaternion,
                    self.body_to_support_z_m,
                ) {
                    Ok(p) => p,
                    Err(_) => continue,
                };
                if snapshot.body_envelope().is_some()
                    && snapshot
                        .validate_body_pose(body, candidate_quaternion)
                        .is_err()
                {
                    continue;
                }
                let view = self
                    .observation_model
                    .as_ref()
                    .map(|model| {
                        snapshot.estimate_observation_view(
                            model,
                            body,
                            candidate_quaternion,
                            &observation_targets,
                        )
                    })
                    .transpose()
                    .map_err(ExploreError::SourceError)?;
                if view.as_ref().is_some_and(|view| view.cells.is_empty()) {
                    continue;
                }
                let count = view.as_ref().map_or(1, |view| view.cells.len());
                if best.as_ref().is_none_or(
                    |(_, _, _, old): &(
                        [f64; 3],
                        [f64; 4],
                        Option<nuwax_explore_routing::observation::ViewEstimate>,
                        usize,
                    )| count > *old,
                ) {
                    best = Some((body, candidate_quaternion, view, count));
                }
                // 无方向性视场只需首个可行视点，避免普通候选多做三次几何与射线查询。
                if !directional_view {
                    break;
                }
            }
            let Some((body, candidate_quaternion, view, _)) = best else {
                continue;
            };
            if let Some(view) = &view {
                candidate.gain =
                    view.cells.len() as f64 * f64::from(snapshot.config().voxel_size).powi(2);
            }
            // 支撑点和完整姿态转换后的机体目标都在候选阶段约束，不能选中后才丢掉。
            if self.boundary.as_ref().is_some_and(|boundary| {
                !boundary.contains(nuwax_nav_types_messages::geometry::Point2::new(
                    body[0], body[1],
                ))
            }) {
                continue;
            }
            admissible.push((candidate, (body, candidate_quaternion), view));
            accepted_regions.insert(region);
            if self.observation_model.is_some() && admissible.len()>=64 {break;}
        }
        let retry = self.retry_support;
        if !new_snapshot {
            // 旧图只允许重试原视点，不能借失败反馈把其它旧候选算作新增益。
            admissible.retain(|(candidate, _, _)| {
                retry.is_some_and(|retry| {
                    distance(retry, candidate.support_xyz)
                        <= f64::from(snapshot.config().voxel_size) * 0.25
                })
            });
            if admissible.is_empty() {
                self.last_no_frontier = false;
                return Ok(ExplorationPoll::Pending);
            }
        }
        let routing_candidates: Vec<_> = admissible
            .iter()
            .map(|(candidate, _, _)| nuwax_explore_routing::Candidate {
                position: candidate.support_xyz,
                gain: candidate.gain,
                travel_cost: candidate.travel_cost,
            })
            .collect();
        let retry_index = retry.and_then(|retry| {
            admissible.iter().position(|(candidate, _, _)| {
                distance(retry, candidate.support_xyz)
                    <= f64::from(snapshot.config().voxel_size) * 0.25
            })
        });
        let observed: Vec<_> = admissible
            .iter()
            .map(|(_, _, view)| {
                view.as_ref()
                    .map(|view| view.cells.clone())
                    .unwrap_or_default()
            })
            .collect();
        let selection = if retry_index.is_none() {
            self.regional_schedule.select_observed(
                &routing_candidates,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    let targets: Vec<_> = goals
                        .iter()
                        .map(|&i| routing_candidates[i].position)
                        .collect();
                    snapshot
                        .frontier_travel_costs(
                            routing_candidates[source].position,
                            &targets,
                            &boundary,
                        )
                        .unwrap_or_else(|_| vec![f64::INFINITY; targets.len()])
                },
            )
        } else {
            None
        };
        let selected_index =
            retry_index.or_else(|| selection.as_ref().map(|selection| selection.index));
        let Some((candidate, (body, candidate_quaternion), view)) =
            selected_index.and_then(|index| admissible.get(index))
        else {
            self.retry_support = None;
            self.last_no_frontier = true;
            self.last_error = None;
            return Ok(ExplorationPoll::NoFrontier);
        };
        self.retry_support = None;
        self.next_goal_id = self.next_goal_id.checked_add(1).ok_or_else(|| {
            ExploreError::SourceError("surface frontier goal identifier exhausted".into())
        })?;
        let travel = candidate.travel_cost;
        let mut goal = FrontierGoal::new(
            format!(
                "{}-surface-{:016x}-{}",
                self.name, state.now.nanos, self.next_goal_id
            ),
            PoseStamped {
                frame_id: state.robot_pose.frame_id.clone(),
                stamp: state.now,
                pose: Pose3 {
                    position: Point3::new(body[0], body[1], body[2]),
                    orientation: nuwax_nav_types_messages::Quaternion {
                        x: candidate_quaternion[0],
                        y: candidate_quaternion[1],
                        z: candidate_quaternion[2],
                        w: candidate_quaternion[3],
                    },
                },
            },
            (candidate.gain / (1.0 + travel)) as f32,
        );
        goal.information_gain = Some(candidate.gain as f32);
        if let Some(view) = view {
            goal.metadata.insert(
                "gain_basis".into(),
                "calibrated_visible_support_cells".into(),
            );
            goal.metadata
                .insert("visibility_rays".into(), view.rays.to_string());
            goal.metadata.insert(
                "visibility_budget_limited".into(),
                view.budget_limited.to_string(),
            );
        }
        goal.frontier_area_m2 = Some(candidate.gain as f32);
        goal.distance_cost = Some(travel as f32);
        goal.producer = "nuwax-exploration/surface".into();
        goal.metadata.insert("geometric_candidate_count".into(),geometric_candidate_count.to_string());
        goal.metadata.insert("evaluated_candidate_count".into(),evaluated_candidate_count.to_string());
        goal.metadata.insert("admissible_candidate_count".into(),admissible.len().to_string());
        goal.metadata.insert("goal_orientation_required".into(),
            self.observation_model.as_ref().is_some_and(|model|
                model.horizontal_fov_rad<std::f64::consts::TAU-1e-6).to_string());
        goal.metadata.insert(
            "selection_method".into(),
            if retry_index.is_some() {
                "navigation_retry"
            } else {
                "regional_route"
            }
            .into(),
        );
        if let Some(selection) = selection {
            goal.metadata.insert(
                "region_wait_rounds".into(),
                selection.priority_wait_rounds.to_string(),
            );
            goal.metadata.insert(
                "route_regions".into(),
                selection.regional_order.len().to_string(),
            );
            goal.metadata.insert(
                "available_regions".into(),
                selection.region_count.to_string(),
            );
            goal.metadata
                .insert("route_cost_m".into(), selection.route_cost.to_string());
        }
        goal.stamp = state.now;
        self.last_frontiers = vec![goal.clone()];
        self.selected = Some(SelectedGoal {
            goal: goal.clone(),
            support_xyz: candidate.support_xyz,
            vertical_radius_m: vertical_radius,
        });
        self.last_no_frontier = false;
        self.last_error = None;
        Ok(ExplorationPoll::Goal(goal))
    }

}

fn comparable_surface_outcome(
    outcome: &ExploreResult<ExplorationPoll>,
) -> Result<ExplorationPoll, String> {
    outcome.clone().map_err(|error| error.to_string())
}

fn accept_matches_legacy_surface(
    prepared: &mut SurfaceGoalSourceAdapter,
    reference: &mut SurfaceGoalSourceAdapter,
    current: &ExplorationStateSnapshot<'_>,
) -> ExploreResult<ExplorationPoll> {
    let before = committed(prepared);
    let expected = reference.legacy_poll_goal(current);
    let preview = prepared.prepare_poll(current).unwrap();
    assert_eq!(
        comparable_surface_outcome(preview.outcome()),
        comparable_surface_outcome(&expected)
    );
    assert_eq!(
        committed(prepared),
        before,
        "preparation changed committed surface state"
    );
    let actual = prepared.commit_prepared_poll();
    assert_eq!(
        comparable_surface_outcome(&actual),
        comparable_surface_outcome(&expected)
    );
    assert_eq!(
        committed(prepared),
        committed(reference),
        "accepted poll diverged from frozen sequential reference"
    );
    actual
}

fn feedback_matches_legacy_surface(
    prepared: &mut SurfaceGoalSourceAdapter,
    reference: &mut SurfaceGoalSourceAdapter,
    goal_id: &str,
    feedback: ExplorationGoalFeedback,
) {
    prepared
        .report_goal_feedback(goal_id, feedback.clone())
        .unwrap();
    reference.report_goal_feedback(goal_id, feedback).unwrap();
    assert_eq!(committed(prepared), committed(reference));
}

#[test]
fn frozen_legacy_reference_covers_goal_pending_retry_and_feedback_transitions() {
    for calibrated in [false, true] {
        let receiver = SurfaceInputReceiver::new();
        let snapshot = map();
        receiver.observe(snapshot.clone()).unwrap();
        let mut prepared = source(receiver.clone(), 1);
        let mut reference = source(receiver.clone(), 1);
        for adapter in [&mut prepared, &mut reference] {
            adapter.record_feedback([30.0, 30.0, 0.1], 0.3, true);
            adapter.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
            if calibrated {
                adapter.set_observation_model(volume_model()).unwrap();
            }
        }
        let first = goal(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 1)).unwrap(),
        );
        assert_eq!(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 2)).unwrap(),
            ExplorationPoll::Pending
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            "obsolete-goal",
            ExplorationGoalFeedback::Reached,
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            &first.id,
            ExplorationGoalFeedback::Failed("first attempt".into()),
        );
        let retry = goal(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 3)).unwrap(),
        );
        assert_eq!(retry.pose.pose.position, first.pose.pose.position);
        assert_ne!(retry.id, first.id);
        assert_eq!(
            retry.metadata.get("selection_method").map(String::as_str),
            Some("navigation_retry")
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            &retry.id,
            ExplorationGoalFeedback::Failed("retry exhausted".into()),
        );
        assert_eq!(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 4)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.observe(snapshot.clone()).unwrap();
        let alternative = goal(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 5)).unwrap(),
        );
        assert!(
            alternative
                .pose
                .pose
                .position
                .distance(first.pose.pose.position)
                > 0.3
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            &alternative.id,
            ExplorationGoalFeedback::InputUnavailable("awaiting input".into()),
        );
        assert_eq!(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 6)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.observe(snapshot.clone()).unwrap();
        let reached = goal(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 7)).unwrap(),
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            &reached.id,
            ExplorationGoalFeedback::Reached,
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            &reached.id,
            ExplorationGoalFeedback::Reached,
        );
        assert_eq!(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 8)).unwrap(),
            ExplorationPoll::Pending
        );
        receiver.observe(map()).unwrap();
        let cancelled = goal(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 9)).unwrap(),
        );
        feedback_matches_legacy_surface(
            &mut prepared,
            &mut reference,
            &cancelled.id,
            ExplorationGoalFeedback::Cancelled,
        );
        receiver.clear().unwrap();
        assert!(matches!(
            accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 10)).unwrap(),
            ExplorationPoll::SourceUnavailable(_)
        ));
    }
}

#[test]
fn frozen_legacy_reference_covers_no_frontier_cached_notifications_and_unavailable() {
    let receiver = SurfaceInputReceiver::new();
    let mut prepared = source(receiver.clone(), 1);
    let mut reference = source(receiver.clone(), 1);
    let boundary = ExplorationBoundary::from_vertices(vec![
        Point2::new(0.5, 0.5),
        Point2::new(1.5, 0.5),
        Point2::new(1.5, 1.5),
        Point2::new(0.5, 1.5),
    ])
    .unwrap();
    for adapter in [&mut prepared, &mut reference] {
        adapter
            .configure_constraints(Some(boundary.clone()), 1, 0.3)
            .unwrap();
        adapter.record_feedback([30.0, 30.0, 0.1], 0.3, true);
        adapter.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
        adapter.last_frontiers = vec![FrontierGoal::new(
            "old-frontier".to_owned(),
            state(0.4, 0).robot_pose,
            1.0,
        )];
        adapter.last_error = Some("previous failure".into());
        adapter.retry_support = Some([30.0, 30.0, 0.1]);
    }
    assert!(matches!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 1)).unwrap(),
        ExplorationPoll::SourceUnavailable(_)
    ));
    let snapshot = map();
    receiver.observe(snapshot.clone()).unwrap();
    assert_eq!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 2)).unwrap(),
        ExplorationPoll::NoFrontier
    );
    assert_eq!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 3)).unwrap(),
        ExplorationPoll::NoFrontier
    );
    receiver.observe(snapshot).unwrap();
    assert_eq!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 4)).unwrap(),
        ExplorationPoll::NoFrontier
    );
    assert!(matches!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(1.4, 5)).unwrap(),
        ExplorationPoll::SourceUnavailable(_)
    ));
    assert_eq!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 6)).unwrap(),
        ExplorationPoll::NoFrontier
    );
    prepared.end().unwrap();
    reference.end().unwrap();
    assert!(matches!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 7)).unwrap(),
        ExplorationPoll::SourceUnavailable(_)
    ));
}

#[test]
fn frozen_legacy_reference_preserves_partial_error_writes_and_retry_consumption() {
    let receiver = SurfaceInputReceiver::new();
    let snapshot = map();
    receiver.observe(snapshot.clone()).unwrap();
    let mut prepared = source(receiver.clone(), 1);
    let mut reference = source(receiver.clone(), 1);
    let first =
        goal(accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 1)).unwrap());
    feedback_matches_legacy_surface(
        &mut prepared,
        &mut reference,
        &first.id,
        ExplorationGoalFeedback::Cancelled,
    );
    for adapter in [&mut prepared, &mut reference] {
        adapter.next_goal_id = u64::MAX;
        adapter.last_frontiers = vec![first.clone()];
        adapter.retry_support = Some([30.0, 30.0, 0.1]);
        adapter.record_feedback([30.0, 30.0, 0.1], 0.3, true);
        adapter.regional_schedule.record_outcome([30.0, 30.0, 0.1]);
    }
    receiver.observe(snapshot).unwrap();
    let before = committed(&prepared);
    let discarded_error = prepared.prepare_poll(&state(0.4, 2)).unwrap();
    assert!(
        matches!(discarded_error.outcome(), Err(ExploreError::SourceError(message)) if message.contains("identifier exhausted"))
    );
    prepared.discard_prepared_poll().unwrap();
    assert_eq!(committed(&prepared), before);
    assert!(
        matches!(accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 2)),
        Err(ExploreError::SourceError(message)) if message.contains("identifier exhausted"))
    );
    assert_eq!(prepared.next_goal_id, u64::MAX);
    assert!(prepared.last_frontiers.is_empty());
    assert!(prepared.retry_support.is_none());
    assert_eq!(prepared.last_error, before.diagnostics.last_error);
    assert_ne!(prepared.checkpoint().unwrap(), before.checkpoint);
    assert_eq!(
        accept_matches_legacy_surface(&mut prepared, &mut reference, &state(0.4, 3)).unwrap(),
        ExplorationPoll::Pending
    );
}
