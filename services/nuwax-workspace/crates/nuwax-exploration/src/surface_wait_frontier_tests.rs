// Independent full-poll wait evaluator, updated for unknown references, with
// a test-only frontier-warmup alternative. Settlement remains transactional.
impl SurfaceGoalSourceAdapter {
    fn wait_reference_evaluate<const SKIP: bool>(
        &self,
        state: &ExplorationStateSnapshot<'_>,
        input: &FrozenSurfaceInput,
        query_view: Option<&Arc<SurfaceSnapshot>>,
        delta: &mut SurfacePollDelta,
    ) -> ExploreResult<ExplorationPoll> {
        if !self.active {
            return Ok(delta.unavailable("surface source is not active"));
        }
        // 精确导航反馈之前不能另派目标，输入撤销由根拥有方取消该子导航。
        if self.selected.is_some() {
            return Ok(ExplorationPoll::Pending);
        }
        let notification = input.notification;
        let Some(source_snapshot) = input.snapshot.as_ref() else {
            return Ok(delta.unavailable("surface map input is unavailable"));
        };
        if state.robot_pose.frame_id.as_str() != source_snapshot.config().world_frame {
            return Ok(delta.unavailable("body pose frame does not match surface map frame"));
        }
        let position = state.robot_pose.pose.position;
        let orientation = state.robot_pose.pose.orientation;
        let quaternion = [orientation.x, orientation.y, orientation.z, orientation.w];
        let same_geometry = self
            .last_snapshot
            .as_ref()
            .is_some_and(|previous| Weak::ptr_eq(previous, &Arc::downgrade(source_snapshot)));
        // 当前占据区依赖实际机体高度；层内接触可能关联同一层顶，缓存不能
        // 因此合并不同的真实位置、姿态或仅比较所在格。
        let same_query = self.last_query_body == Some([position.x, position.y, position.z])
            && self.last_query_orientation == Some(quaternion);
        if same_geometry && same_query && self.last_no_frontier {
            delta.last_notification = Some(Some(notification));
            delta.last_error = Some(None);
            return Ok(ExplorationPoll::NoFrontier);
        }
        let new_snapshot = self.last_notification != Some(notification) || !same_geometry;
        let deferred = self.deferred_route.as_ref().filter(|deferred| {
            deferred.source.same_frontier_query_geometry(source_snapshot.as_ref())
                && distance(deferred.start_body, [position.x, position.y, position.z])
                    <= f64::from(source_snapshot.config().goal_tolerance.max(source_snapshot.config().voxel_size))
        });
        let active_deferred = deferred.filter(|route| !route.terminal);
        let continuing = (!new_snapshot && same_query && self.route_cursor > 0)
            || deferred.is_some_and(|route| route.terminal);
        let selecting = new_snapshot || continuing || deferred.is_some();
        if !selecting && same_query && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        let boundary: Vec<_> = self.boundary.as_ref().map_or_else(Vec::new, |boundary| {
            boundary
                .vertices()
                .iter()
                .map(|point| [point.x, point.y])
                .collect()
        });
        // Existing synchronous callers retain their cache behavior. A compute owner
        // may supply private caches, but all identity tests and consumption stamps
        // still refer to the original frozen source, never its planning fork.
        let snapshot = query_view.unwrap_or(source_snapshot);
        // An unchanged-input wait cannot select a frontier. Its private fork
        // will be discarded, so only retain the required body/support checks
        // below. Synchronous callers keep their original cache behavior.
        if !SKIP || query_view.is_none() || selecting || self.retry_support.is_some() {
            snapshot.prepare_frontier_geometry(&boundary)
                .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        }
        let current_body = ([position.x, position.y, position.z], quaternion);
        if snapshot.body_envelope().is_some() {
            if let Err(error) = snapshot.validate_body_pose(current_body.0, current_body.1) {
                return Ok(delta.unavailable(format!("current body geometry: {error}")));
            }
        }
        // This is only a calibrated planning reference. It neither inserts a
        // floor nor requires an observed support cell under the current body.
        let robot_support = match snapshot.support_position_from_body(
            current_body.0, current_body.1, self.body_to_support_z_m,
        ) {
            Ok(reference) => reference,
            Err(error) => return Ok(delta.unavailable(format!("invalid body reference: {error}"))),
        };
        if !nuwax_nav_pct::inside_boundary([robot_support[0], robot_support[1]], &boundary) {
            return Ok(delta.unavailable("current body reference is outside exploration boundary"));
        }
        if !selecting && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        // Continue one coherent candidate ordering across receipt updates and
        // small pose jitter; motion checks still use the actual current pose.
        let sampling_reference = deferred.map_or(Ok(robot_support), |route|
            snapshot.support_position_from_body(route.start_body, route.start_orientation,
                self.body_to_support_z_m))
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let geometry = frontier_geometry(snapshot, sampling_reference, &boundary, self.observation_model.as_ref())
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let mut candidates = geometry.candidates;
        let observation_targets = geometry.targets;
        // One immutable PCT map serves every candidate and regional route query
        // in this poll. Unknown states remain request-local inside PCT searches.
        let tomogram = nuwax_nav_pct::Tomogram::from_surface(snapshot)
            .map_err(ExploreError::SourceError)?;
        let planar_bias = if snapshot.body_envelope().is_none() {
            tomogram.calibrated_reference_height(robot_support)
                .map(|height| robot_support[2] - height)
        } else { None };
        delta.last_snapshot = Some(Some(Arc::downgrade(source_snapshot)));
        delta.last_notification = Some(Some(notification));
        delta.last_query_body = Some(Some([position.x, position.y, position.z]));
        delta.last_query_orientation = Some(Some(quaternion));
        delta.last_frontiers = Some(Vec::new());
        let minimum_distance = f64::from(
            snapshot
                .config()
                .goal_tolerance
                .max(snapshot.config().voxel_size),
        );
        // 默认3米策略半径不能跨越另一楼层；高度只覆盖局部可跨台阶及量化差。
        let vertical_radius =
            f64::from(snapshot.config().step_threshold_m) + f64::from(snapshot.config().voxel_size);
        let geometric_candidate_count=candidates.len();
        if !selecting {
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
        if let Some(route) = active_deferred { candidates = vec![route.candidate.clone()]; }
        let mut admissible = Vec::new();
        let mut accepted_regions=std::collections::BTreeSet::new();
        let mut evaluated_candidate_count=0;
        let mut route_query_count=0;
        let state_limit = active_deferred.map_or(MAX_FRONTIER_ROUTE_STATES, |route| route.state_limit);
        let route_budget = FrontierRouteBudget::with_states(MAX_FRONTIER_SEARCH_STATES.max(state_limit));
        let mut next_deferred = None;
        let mut route_budget_limited=false;
        let mut next_route_cursor = active_deferred.map_or(0, |route| route.resume_cursor);
        let cursor = if active_deferred.is_some() { 0 } else {
            deferred.map_or(if continuing { self.route_cursor } else { 0 }, |route| route.resume_cursor)
        };
        let retrying_deferred = active_deferred.is_some();
        let deferred_resume_cursor = active_deferred.map(|route| route.resume_cursor);
        let previous_inconclusive = deferred.map_or(continuing && self.route_inconclusive,
            |route| route.unresolved);
        let deferred_start_body = deferred.map_or(current_body.0, |old| old.start_body);
        let deferred_start_orientation = deferred.map_or(current_body.1, |old| old.start_orientation);
        let continuation = |resume_cursor, unresolved| DeferredFrontierRoute {
            source_unavailable: None,
            source: source_snapshot.as_ref().clone(),
            candidate: FrontierCandidate { support_xyz: robot_support, gain: 0.0, travel_cost: 0.0 },
            start_body: deferred_start_body, start_orientation: deferred_start_orientation,
            state_limit: nuwax_nav_pct::SearchLimits::default().max_states,
            terminal: true, resume_cursor, unresolved,
        };
        delta.route_cursor = Some(0);
        delta.route_inconclusive = Some(false);
        delta.deferred_route = Some(None);
        for (ordinal, mut candidate) in candidates.into_iter().enumerate().skip(cursor) {
            if nuwax_nav_pct::planning_cancelled() {
                return Err(ExploreError::SourceError("frontier planning cancelled".into()));
            }
            if self.excluded(candidate.support_xyz, vertical_radius)
                || distance(robot_support, candidate.support_xyz) <= minimum_distance
            {
                continue;
            }
            let region=nuwax_explore_routing::region_key(candidate.support_xyz);
            // 无视域标定时收益不会再改变；同区域后续候选必然被区域路由丢弃。
            // 首个候选无效时继续检查下一项，不能把候选上限当作无前沿证明。
            if self.observation_model.is_none() && accepted_regions.contains(&region)
                && !retry_matches(candidate.support_xyz) {continue;}
            if self.observation_model.is_some() && !retry_matches(candidate.support_xyz)
                && admissible.iter().filter(|(candidate, _, _): &&(FrontierCandidate, BodyPose,
                    Option<nuwax_explore_routing::observation::ViewEstimate>)|
                    nuwax_explore_routing::region_key(candidate.support_xyz) == region).count() >= 2 {
                continue;
            }
            if evaluated_candidate_count == MAX_FRONTIER_CANDIDATE_EVALUATIONS {
                route_budget_limited = true;
                next_route_cursor = ordinal;
                break;
            }
            evaluated_candidate_count+=1;
            let base_yaw = (candidate.support_xyz[1] - robot_support[1])
                .atan2(candidate.support_xyz[0] - robot_support[0]);
            let directional_view = self.observation_model.as_ref().is_some_and(|model| {
                model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6
            });
            // 机体能否停留与传感器视场独立；直指前沿不适合时仍尝试沿边缘停留。
            let turns = if snapshot.body_envelope().is_some() || directional_view { 4 } else { 1 };
            let mut best = None;
            for i in 0..turns {
                let yaw = base_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                let (body, candidate_quaternion) = if snapshot.body_envelope().is_some() {
                    match snapshot.body_pose_after_motion(current_body,
                        [candidate.support_xyz[0], candidate.support_xyz[1]], yaw) {
                        Ok(pose) => pose,
                        Err(_) => continue,
                    }
                } else {
                    let q = if self.observation_model.is_some() {
                        [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]
                    } else {
                        quaternion
                    };
                    let Some(bias) = planar_bias else { continue; };
                    // Endpoint discovery must not demand a straight clear route.
                    // Preserve the same calibration bias at known and unknown
                    // candidates; the shared planner establishes their paths.
                    let nominal_z = if tomogram.cell_at(candidate.support_xyz).is_none()
                        && (candidate.support_xyz[2] - sampling_reference[2]).abs() <= vertical_radius {
                        // Unknown edge seeds may use their observed neighbour's
                        // height; the unobserved destination still carries the
                        // current calibrated plane, including its actual bias.
                        robot_support[2] - bias
                    } else {
                        candidate.support_xyz[2] + robot_support[2] - sampling_reference[2] - bias
                    };
                    let nominal = [candidate.support_xyz[0], candidate.support_xyz[1], nominal_z];
                    let Some(height) = tomogram.calibrated_reference_height(nominal) else { continue; };
                    let carried = [nominal[0], nominal[1], height + bias];
                    let reference = match tomogram.project_reference_motion(carried,
                        [carried[0], carried[1]], &boundary) {
                        Ok(reference) => reference,
                        Err(_) => continue,
                    };
                    if snapshot.validate_planar_recent_motion(reference, reference).is_err() { continue; }
                    match snapshot.body_position_from_support(
                        reference, q, self.body_to_support_z_m,
                    ) {
                        Ok(body) => (body, q),
                        Err(_) => continue,
                    }
                };
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
            // Projection must retain the candidate's layer. In particular, an
            // upper-floor seed cannot silently turn into a visited lower-floor
            // viewpoint and bypass the pre-projection history check.
            let projected = snapshot.support_position_from_body(
                body, candidate_quaternion, self.body_to_support_z_m,
            ).map_err(|error| ExploreError::SourceError(error.to_string()))?;
            if (projected[2] - candidate.support_xyz[2]).abs() > vertical_radius
                || self.excluded(projected, vertical_radius)
                || distance(robot_support, projected) <= minimum_distance {
                continue;
            }
            candidate.support_xyz = projected;
            if route_budget.exhausted() {
                route_budget_limited = true;
                next_route_cursor = ordinal;
                break;
            }
            route_query_count += 1;
            let Some(cost) = frontier_route_cost(
                &tomogram, snapshot, robot_support, candidate.support_xyz,
                current_body, (body, candidate_quaternion), self.body_to_support_z_m,
                &boundary, geometry.lookahead_m, &route_budget, state_limit,
            ) else {
                if route_budget.last_inconclusive.get() && next_deferred.is_none() {
                    let upper = nuwax_nav_pct::SearchLimits::default().max_states;
                    let terminal = route_budget.last_domain_limited.get() || state_limit >= upper;
                    next_deferred = Some(DeferredFrontierRoute {
            source_unavailable: None,
                        source: source_snapshot.as_ref().clone(),
                        candidate: candidate.clone(), start_body: deferred_start_body,
                        start_orientation: deferred_start_orientation,
                        state_limit: if terminal { state_limit } else { state_limit.saturating_mul(2).min(upper) },
                        terminal,
                        resume_cursor: deferred_resume_cursor.unwrap_or(ordinal + 1),
                        unresolved: previous_inconclusive || terminal,
                    });
                    // Do not consume other candidates while this one is
                    // unresolved: its untouched suffix is resumed explicitly.
                    break;
                }
                if retrying_deferred { break; }
                continue;
            };
            candidate.travel_cost = cost;
            admissible.push((candidate, (body, candidate_quaternion), view));
            accepted_regions.insert(region);
            if admissible.len() >= 8 { break; }
        }
        let retry = self.retry_support;
        if !selecting {
            // 旧图只允许重试原视点，不能借失败反馈把其它旧候选算作新增益。
            admissible.retain(|(candidate, _, _)| {
                retry.is_some_and(|retry| {
                    distance(retry, candidate.support_xyz)
                        <= f64::from(snapshot.config().voxel_size) * 0.25
                })
            });
            if admissible.is_empty() {
                delta.last_no_frontier = Some(false);
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
        #[cfg(test)]
        VIEW_TRANSFER.with(|counts| counts.set((admissible.len(),
            admissible.iter().filter(|(_, _, view)| view.as_ref().is_some_and(|v| !v.cells.is_empty())).count(),
            admissible.iter().map(|(_, _, view)| view.as_ref().map_or(0, |v| v.cells.len())).sum())));
        let observed: Vec<_> = admissible
            .iter()
            .map(|(_, _, view)| {
                view.as_ref()
                    .map(|view| view.cells.clone())
                    .unwrap_or_default()
            })
            .collect();
        let selection = if retry_index.is_none() {
            let (selection, regional_delta) = self.regional_schedule.prepare_select_observed(
                &routing_candidates,
                self.observation_model.as_ref().map(|_| observed.as_slice()),
                |source, goals| {
                    goals.iter().map(|&target| {
                        frontier_route_cost(
                            &tomogram, snapshot,
                            routing_candidates[source].position, routing_candidates[target].position,
                            admissible[source].1, admissible[target].1, self.body_to_support_z_m,
                            &boundary, geometry.lookahead_m, &route_budget, MAX_FRONTIER_ROUTE_STATES,
                        ).unwrap_or(f64::INFINITY)
                    }).collect()
                },
            );
            delta.regional = Some(regional_delta);
            selection
        } else {
            None
        };
        if nuwax_nav_pct::planning_cancelled() {
            return Err(ExploreError::SourceError("frontier planning cancelled".into()));
        }
        // An exactly spent final reservation is complete, not unfinished work.
        // The candidate loop already records a cursor when another query remains.
        let selected_index =
            retry_index.or_else(|| selection.as_ref().map(|selection| selection.index));
        let Some((candidate, (body, candidate_quaternion), view)) =
            selected_index.and_then(|index| admissible.get(index))
        else {
            if let Some(deferred) = next_deferred {
                let terminal = deferred.terminal;
                let limit = deferred.state_limit;
                let resume_cursor = deferred.resume_cursor;
                let unresolved = deferred.unresolved;
                delta.deferred_route = Some(Some(deferred));
                delta.route_cursor = Some(resume_cursor);
                delta.route_inconclusive = Some(unresolved);
                delta.last_no_frontier = Some(false);
                if terminal && resume_cursor >= geometric_candidate_count {
                    delta.last_error = Some(Some(format!(
                        "ResourceLimited: frontier route remains inconclusive within its bounded domain ({limit} state allowance)",
                    )));
                    return Ok(ExplorationPoll::Stop(crate::ExplorationStopReason::PlanningResourceLimited));
                }
                delta.last_error = Some(Some(if terminal {
                    "frontier route remains inconclusive within its bounded domain; checking remaining candidates".into()
                } else {
                    format!("frontier route search is inconclusive; continuing the candidate with {limit} states")
                }));
                return Ok(ExplorationPoll::Pending);
            }
            let unresolved = previous_inconclusive || route_budget.inconclusive.get();
            if next_route_cursor > 0 && next_route_cursor < geometric_candidate_count {
                delta.deferred_route = Some(Some(continuation(next_route_cursor, unresolved)));
                delta.route_cursor = Some(next_route_cursor);
                delta.route_inconclusive = Some(unresolved);
                delta.last_no_frontier = Some(false);
                delta.last_error = Some(Some("frontier search is continuing the remaining candidates".into()));
                return Ok(ExplorationPoll::Pending);
            }
            if unresolved {
                delta.deferred_route = Some(Some(continuation(geometric_candidate_count, true)));
                delta.route_cursor = Some(geometric_candidate_count);
                delta.route_inconclusive = Some(true);
                delta.last_no_frontier = Some(false);
                delta.last_error = Some(Some(
                    "ResourceLimited: candidate search is complete but at least one route remains inconclusive".into()));
                return Ok(ExplorationPoll::Stop(crate::ExplorationStopReason::PlanningResourceLimited));
            }
            delta.retry_support = Some(None);
            delta.last_no_frontier = Some(true);
            delta.last_error = Some(None);
            return Ok(ExplorationPoll::NoFrontier);
        };
        delta.retry_support = Some(None);
        let next_goal_id = self.next_goal_id.checked_add(1).ok_or_else(|| {
            ExploreError::SourceError("surface frontier goal identifier exhausted".into())
        })?;
        delta.next_goal_id = Some(next_goal_id);
        let travel = candidate.travel_cost;
        let mut goal = FrontierGoal::new(
            format!(
                "{}-surface-{:016x}-{}",
                self.name, state.now.nanos, next_goal_id
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
        goal.metadata.insert("ground_reference_policy".into(), "known_terrain_or_calibrated_current_layer".into());
        goal.metadata.insert("route_query_count".into(), route_budget.queries.get().to_string());
        goal.metadata.insert("candidate_route_query_count".into(), route_query_count.to_string());
        goal.metadata.insert("route_state_budget_limit".into(), route_budget.total.to_string());
        goal.metadata.insert("route_expanded_successes".into(), route_budget.expanded.get().to_string());
        goal.metadata.insert("route_state_budget_charged".into(), (route_budget.total - route_budget.remaining.get()).to_string());
        goal.metadata.insert("route_budget_limited".into(), route_budget_limited.to_string());
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
        delta.last_frontiers = Some(vec![goal.clone()]);
        delta.selected = Some(Some(SelectedGoal {
            goal: goal.clone(),
            support_xyz: candidate.support_xyz,
            vertical_radius_m: vertical_radius,
        }));
        delta.last_no_frontier = Some(false);
        delta.last_error = Some(None);
        Ok(ExplorationPoll::Goal(goal))
    }

}

fn prepare_wait_variant(adapter:&mut SurfaceGoalSourceAdapter,state:&ExplorationStateSnapshot<'_>,input:&FrozenSurfacePlanningInput,which:usize)->PreparedExplorationPoll {
    if which==2 {return adapter.prepare_poll_with_planning_input(state,input).unwrap();}
    adapter.ensure_no_prepared_poll().unwrap();
    let mut delta=SurfacePollDelta::default();
    let result=if which==0 {
        adapter.wait_reference_evaluate::<false>(state,&input.original,input.query_view.as_ref(),&mut delta)
    } else {
        adapter.wait_reference_evaluate::<true>(state,&input.original,input.query_view.as_ref(),&mut delta)
    };
    adapter.stage_poll(result,delta)
}

fn wait_map(width:i32,blind:bool)->Arc<SurfaceSnapshot> {
    let center=f64::from(width)*0.05+0.05;
    let points:Vec<_>=(0..width).flat_map(|x|(0..width).filter_map(move|y| {
        let p=((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05);
        (!blind || (f64::from(p.0)-center).abs()>0.3 || (f64::from(p.1)-center).abs()>0.12).then_some(p)
    })).collect();
    Arc::new(SurfaceSnapshot::from_points(&points,map().config().clone(),0.1).unwrap()
        .with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m:vec![[-0.3,-0.12],[0.3,-0.12],[0.3,0.12],[-0.3,0.12]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,max_step_up_m:0.2,max_step_down_m:0.2,
            max_cross_slope_deg:15.0,support_uncertainty_m:0.0,
        }).unwrap())
}

fn waiting_adapter(receiver:SurfaceInputReceiver,current:&ExplorationStateSnapshot<'_>,retry:bool)->SurfaceGoalSourceAdapter {
    let mut adapter=source(receiver,1);
    let first=goal(adapter.poll_goal(current).unwrap());
    adapter.report_goal_feedback(&first.id,if retry {ExplorationGoalFeedback::Failed("fixture retry".into())}else{ExplorationGoalFeedback::Reached}).unwrap();
    adapter
}

#[test]
fn private_wait_skips_only_unused_frontier_work_preserving_errors_and_settlement() {
    for case in 0..10 {
        let snapshot=wait_map(20,case==9);let receiver=SurfaceInputReceiver::new();
        receiver.observe(snapshot.clone()).unwrap();
        let current=state(0.4,1);
        let mut adapters:Vec<_>=(0..3).map(|_|waiting_adapter(receiver.clone(),&current,case==6)).collect();
        let mut next=current.clone();next.now=Timestamp::from_unix(2,0);next.robot_pose.pose.position.x+=0.03;
        match case {
            1=>next.robot_pose.pose.position.x-=0.03,
            2=>next.robot_pose.pose.position.x=-10.0,
            3=>next.robot_pose.pose.orientation=Quaternion{x:0.0,y:0.0,z:0.0,w:0.0},
            4=>receiver.clear().unwrap(),
            5=>receiver.observe(snapshot.clone()).unwrap(),
            7=>for adapter in &mut adapters {adapter.configure_constraints(Some(ExplorationBoundary::from_vertices(vec![
                Point2::new(0.8,0.8),Point2::new(1.3,0.8),Point2::new(1.3,1.3),Point2::new(0.8,1.3)]).unwrap()),1,0.3).unwrap();},
            8=>for adapter in &mut adapters {
                // Fault injection through configuration: even malformed boundary
                // input resets consumed stamps and retains the original error.
                adapter.configure_constraints(Some(ExplorationBoundary{vertices:vec![Point2::new(0.0,0.0),Point2::new(1.0,0.0)]}),1,0.3).unwrap();
            },
            _=>{},
        }
        let mut outcomes=Vec::new();let mut settled=Vec::new();let mut scans=Vec::new();
        for (which,adapter) in adapters.iter_mut().enumerate() {
            let captured=receiver.freeze().unwrap();let input=captured.fork_for_planning();
            let before=committed(adapter);
            let preview=prepare_wait_variant(adapter,&next,&input,which);
            assert_eq!(committed(adapter),before,"prepare is private: case {case} variant {which}");
            outcomes.push(comparable_surface_outcome(preview.outcome()));
            scans.push(input.query_snapshot().and_then(|s|s.frontier_cache_stats()).map(|s|s.evaluated_cells));
            adapter.discard_prepared_poll().unwrap();assert_eq!(committed(adapter),before);
            // Exact same source/input after discard must remain available.
            let again=prepare_wait_variant(adapter,&next,&input,which);
            assert_eq!(comparable_surface_outcome(again.outcome()),outcomes[which]);
            let result=adapter.commit_prepared_poll();
            assert_eq!(comparable_surface_outcome(&result),outcomes[which]);
            settled.push(committed(adapter));
        }
        assert_eq!(outcomes[0],outcomes[1],"case {case}");assert_eq!(outcomes[0],outcomes[2],"case {case}");
        assert_eq!(settled[0],settled[1],"case {case}");assert_eq!(settled[0],settled[2],"case {case}");
        if matches!(case,0|2|3|9) {assert!(scans[0].is_some());assert!(scans[1].is_none());assert!(scans[2].is_none());}
        if matches!(case,5|6|7) {assert!(scans[0].is_some());assert!(scans[1].is_some());}
        if case==8 {assert!(outcomes[0].is_err(),"malformed reconfiguration must still fail");}
        eprintln!("wait_equivalence case={case} outcomes={:?} frontier_scans={scans:?}",outcomes[0]);
    }
}

#[test]
#[ignore="fixed paired full private-fork wait polling; no adaptive samples"]
fn private_wait_frontier_profile() {
    for (width,blind) in [(20,false),(80,false),(160,false),(20,true)] {
        let snapshot=wait_map(width,blind);let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot).unwrap();
        let mut current=state(0.4,1);current.robot_pose.pose.position.x=f64::from(width)*0.05+0.05;current.robot_pose.pose.position.y=current.robot_pose.pose.position.x;
        let mut adapters=[waiting_adapter(receiver.clone(),&current,false),waiting_adapter(receiver.clone(),&current,false)];
        let mut moved=current.clone();moved.robot_pose.pose.position.x+=0.03;moved.now=Timestamp::from_unix(2,0);
        for pair in 0..3 {for arm in if pair%2==0 {[0,1]}else{[1,0]} {
            let adapter=&mut adapters[arm];let before=committed(adapter);let captured=receiver.freeze().unwrap();
            let started=std::time::Instant::now();
            let input=captured.fork_for_planning();
            let preview=prepare_wait_variant(adapter,&moved,&input,arm);
            adapter.discard_prepared_poll().unwrap();
            let us=started.elapsed().as_secs_f64()*1e6;
            assert!(matches!(preview.outcome(),Ok(ExplorationPoll::Pending)));assert_eq!(committed(adapter),before);
            let evaluated=input.query_snapshot().unwrap().frontier_cache_stats().map_or(0,|s|s.evaluated_cells);
            if arm==0 {assert!(evaluated>0);}else{assert_eq!(evaluated,0);}
            println!("{{\"width\":{width},\"blind\":{blind},\"pair\":{pair},\"candidate\":{},\"us\":{us},\"evaluated_cells\":{evaluated}}}",arm==1);
        }}
    }
}

#[test]
#[ignore="real source binary qualification; fixed two samples per cohort"]
fn private_wait_production_profile() {
    // Runner uses three independent alternating baseline/candidate process pairs.
    // Mode 0 waits with changed pose; 1 is new input; 2 retries; 3 is same pose.
    for (width,blind,mode) in [(20,false,0),(80,false,0),(160,false,0),(20,true,0),
        (80,false,1),(20,false,2),(20,false,3)] {
        let snapshot=wait_map(width,blind);let receiver=SurfaceInputReceiver::new();receiver.observe(snapshot.clone()).unwrap();
        let mut current=state(0.4,1);current.robot_pose.pose.position.x=f64::from(width)*0.05+0.05;current.robot_pose.pose.position.y=current.robot_pose.pose.position.x;
        let mut adapter=waiting_adapter(receiver.clone(),&current,mode==2);
        let mut moved=current.clone();moved.now=Timestamp::from_unix(2,0);
        if mode!=3 {moved.robot_pose.pose.position.x+=0.03;}
        if mode==1 {receiver.observe(snapshot).unwrap();}
        for sample in 0..2 {
            let before=committed(&adapter);let captured=receiver.freeze().unwrap();
            let started=std::time::Instant::now();
            let input=captured.fork_for_planning();
            let preview=adapter.prepare_poll_with_planning_input(&moved,&input).unwrap();
            adapter.discard_prepared_poll().unwrap();
            let us=started.elapsed().as_secs_f64()*1e6;
            assert_eq!(committed(&adapter),before);
            if matches!(mode,0|3) {assert!(matches!(preview.outcome(),Ok(ExplorationPoll::Pending)));}
            else {assert!(matches!(preview.outcome(),Ok(ExplorationPoll::Goal(_))));}
            let evaluated=input.query_snapshot().unwrap().frontier_cache_stats().map_or(0,|s|s.evaluated_cells);
            if matches!(mode,1|2) {assert!(evaluated>0);}
            println!("{}",serde_json::json!({"width":width,"blind":blind,"mode":mode,"sample":sample,
                "us":us,"evaluated_cells":evaluated,"outcome":format!("{:?}",comparable_surface_outcome(preview.outcome()))}));
        }
    }
}
