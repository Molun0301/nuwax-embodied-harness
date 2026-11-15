//! 修改前的选点实现：每次选点从整张图新建原生规划器，候选与区域代价逐对查询。
//! 只用于与生产实现在同样地图与状态序列下并排比较选点结果和单次选点耗时。
use super::*;
use nuwax_nav_runtime::native_map::PctTerrainSource;
use std::sync::Mutex;

#[derive(Clone)]
pub(super) struct CachedFrontierPlanner {
    terrain_source: PctTerrainSource,
    offset: f64,
    boundary: Vec<[f64; 2]>,
    resolution: f64,
    measured: Arc<std::collections::BTreeMap<[i32; 2], Vec<f64>>>,
    pub(super) planner: Arc<Mutex<nuwax_nav_pct_native::Planner>>,
    input: Arc<nuwax_nav_pct_native::Config>,
}
#[derive(Default)]
pub(super) struct NativeFrontierCache {
    state: Mutex<Option<CachedFrontierPlanner>>,
}
impl std::fmt::Debug for NativeFrontierCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("NativeFrontierCache")
            .finish_non_exhaustive()
    }
}
impl NativeFrontierCache {
    pub(super) fn get(
        &self,
        snapshot: &SurfaceSnapshot,
        offset: f64,
        boundary: &[[f64; 2]],
        terrain_source: PctTerrainSource,
    ) -> Result<CachedFrontierPlanner, String> {
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err("PCT规划已取消".into());
        }
        let mut state = self
            .state
            .lock()
            .map_err(|_| "前沿原生地图缓存锁已失效".to_string())?;
        if let Some(cached) = state.as_ref().filter(|cached| {
            cached.offset == offset
                && cached.boundary == boundary
                && cached.terrain_source == terrain_source
        }) {
            return Ok(cached.clone());
        }
        let (config, cells, bands) = nuwax_nav_runtime::native_map::pct_input_for_source(
            snapshot,
            offset,
            boundary,
            terrain_source,
        )?;
        let started = std::time::Instant::now();
        tracing::debug!(target: "navigation.exploration.compute", cells = cells.len(), bands = bands.len(), "开始构建前沿原生规划器");
        let mut measured = std::collections::BTreeMap::<[i32; 2], Vec<f64>>::new();
        for cell in &cells {
            measured
                .entry([cell.x, cell.y])
                .or_default()
                .push(cell.ground);
        }
        let resolution = config.resolution;
        let planner = nuwax_nav_pct_native::Planner::from_cells(config.clone(), &cells, &bands)?;
        tracing::debug!(target: "navigation.exploration.compute", elapsed_ms = started.elapsed().as_secs_f64() * 1000.0, "前沿原生规划器构建完成");
        tracing::debug!(target:"navigation.pct",event="navigation.pct.map",cells=cells.len(),
                occupied_bands=bands.len(),"探索PCT原生输入构建完成");
        let cached = CachedFrontierPlanner {
            terrain_source,
            offset,
            boundary: boundary.to_vec(),
            resolution,
            measured: Arc::new(measured),
            planner: Arc::new(Mutex::new(planner)),
            input: Arc::new(config),
        };
        *state = Some(cached.clone());
        Ok(cached)
    }
}

/// 旧实现的轮询入口：与生产 `poll_goal`、`poll_current_input` 相同的状态推进，只替换选点计算。
pub(super) fn legacy_poll_goal(
    source: &mut SurfaceGoalSourceAdapter,
    state: &ExplorationStateSnapshot<'_>,
    terrain_source: PctTerrainSource,
) -> ExploreResult<ExplorationPoll> {
    let outcome = legacy_poll_current_input(source, state, terrain_source)?;
    if outcome == ExplorationPoll::NoFrontier && source.reintroduce_failed_frontiers() {
        return legacy_poll_current_input(source, state, terrain_source);
    }
    Ok(outcome)
}

fn legacy_poll_current_input(
    source: &mut SurfaceGoalSourceAdapter,
    state: &ExplorationStateSnapshot<'_>,
    terrain_source: PctTerrainSource,
) -> ExploreResult<ExplorationPoll> {
    if !source.active {
        let reason = "surface source is not active".to_string();
        source.last_error = Some(reason.clone());
        source.last_frontiers.clear();
        return Ok(ExplorationPoll::SourceUnavailable(reason));
    }
    if source.selected.is_some() { return Ok(ExplorationPoll::Pending); }
    let input = source.receiver.freeze()?;
    let mut delta = SurfacePollDelta::default();
    let outcome = source.legacy_evaluate_poll(state, &input, &mut delta, terrain_source);
    delta.commit(source);
    outcome
}

impl SurfaceGoalSourceAdapter {
    fn legacy_evaluate_poll(
        &self,
        state: &ExplorationStateSnapshot<'_>,
        input: &FrozenSurfaceInput,
        delta: &mut SurfacePollDelta,
        terrain_source: PctTerrainSource,
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
            deferred
                .source
                .same_frontier_query_geometry(source_snapshot.as_ref())
                && distance(deferred.start_body, [position.x, position.y, position.z])
                    <= f64::from(
                        source_snapshot
                            .config()
                            .goal_tolerance
                            .max(source_snapshot.config().voxel_size),
                    )
        });
        let continuing =
            (!new_snapshot && same_query && self.route_cursor > 0) || deferred.is_some();
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
        let snapshot = source_snapshot;
        let compute_started = std::time::Instant::now();
        snapshot
            .prepare_frontier_geometry(&boundary)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let geometry_ms = compute_started.elapsed().as_secs_f64() * 1000.0;
        let current_body = ([position.x, position.y, position.z], quaternion);
        // This is only a calibrated planning reference. It neither inserts a
        // floor nor requires an observed support cell under the current body.
        let robot_support = match snapshot.support_position_from_body(
            current_body.0,
            current_body.1,
            self.body_to_support_z_m,
        ) {
            Ok(reference) => reference,
            Err(error) => return Ok(delta.unavailable(format!("invalid body reference: {error}"))),
        };
        if !inside_boundary([robot_support[0], robot_support[1]], &boundary) {
            return Ok(delta.unavailable("current body reference is outside exploration boundary"));
        }
        if !selecting && self.retry_support.is_none() {
            return Ok(ExplorationPoll::Pending);
        }
        // Continue one coherent candidate ordering across receipt updates and
        // small pose jitter; motion checks still use the actual current pose.
        let sampling_reference = deferred
            .map_or(Ok(robot_support), |route| {
                snapshot.support_position_from_body(
                    route.start_body,
                    route.start_orientation,
                    self.body_to_support_z_m,
                )
            })
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let geometry = frontier_geometry(
            snapshot,
            sampling_reference,
            &boundary,
            self.observation_model.as_ref(),
        )
        .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        let mut candidates = geometry.candidates;
        let observation_targets = geometry.targets;
        let frontier_ms = compute_started.elapsed().as_secs_f64() * 1000.0 - geometry_ms;
        let native_started = std::time::Instant::now();
        // 前沿路线使用与导航相同的原生实测层与占据输入，缺测不补造可通行节点。
        let uncached = NativeFrontierCache::default();
        let native = uncached
            .get(snapshot, self.body_to_support_z_m, &boundary, terrain_source)
            .map_err(ExploreError::SourceError)?;
        for candidate in &mut candidates {
            let xy = [
                (candidate.support_xyz[0] / native.resolution).floor() as i32,
                (candidate.support_xyz[1] / native.resolution).floor() as i32,
            ];
            if let Some(heights) = native.measured.get(&xy) {
                if let Some(height) = heights.iter().min_by(|a, b| {
                    (*a - candidate.support_xyz[2])
                        .abs()
                        .total_cmp(&(*b - candidate.support_xyz[2]).abs())
                }) {
                    candidate.support_xyz[2] = *height;
                }
            }
        }
        let native_input_ms = native_started.elapsed().as_secs_f64() * 1000.0;
        let planner = native.planner.clone();
        let lock_started = std::time::Instant::now();
        let mut planner_guard = planner.lock().map_err(|_| {
            ExploreError::SourceError("native frontier planner lock is poisoned".into())
        })?;
        let planner_lock_ms = lock_started.elapsed().as_secs_f64() * 1000.0;
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
        delta.frontier_count = Some(candidates.iter()
            .filter(|candidate| !self.excluded(candidate.support_xyz, vertical_radius)).count());
        let geometric_candidate_count = candidates.len();
        if !selecting {
            candidates.retain(|candidate| {
                self.retry_support.is_some_and(|retry| {
                    distance(retry, candidate.support_xyz)
                        <= f64::from(snapshot.config().voxel_size) * 0.25
                })
            });
        }
        let retry_matches = |point: [f64; 3]| {
            self.retry_support.is_some_and(|retry| {
                distance(retry, point) <= f64::from(snapshot.config().voxel_size) * 0.25
            })
        };
        candidates.sort_by(|a, b| {
            retry_matches(b.support_xyz)
                .cmp(&retry_matches(a.support_xyz))
                .then_with(|| {
                    (b.gain / (1.0 + b.travel_cost)).total_cmp(&(a.gain / (1.0 + a.travel_cost)))
                })
                .then_with(|| a.support_xyz[0].total_cmp(&b.support_xyz[0]))
                .then_with(|| a.support_xyz[1].total_cmp(&b.support_xyz[1]))
                .then_with(|| a.support_xyz[2].total_cmp(&b.support_xyz[2]))
        });
        let native_ms = native_started.elapsed().as_secs_f64() * 1000.0;
        let ranking_started = std::time::Instant::now();
        let mut evaluated_candidate_count = 0;
        // 保留原128候选和64次路线查询额度。
        let mut route_budget = FrontierRouteBudget { reserved: 1, ..Default::default() };
        let mut route_query_count = 0;
        let mut admissible = Vec::new();
        let mut accepted_regions = std::collections::BTreeSet::new();
        let rotation_offset = deferred.map(|route|route.rotation_offset).unwrap_or(0)
            % candidates.len().max(1);
        candidates.rotate_left(rotation_offset);
        let mut search = planner_guard.multi_goal_search().map_err(ExploreError::SourceError)?;
        let mut route_budget_limited = false;
        let mut next_route_cursor = 0;
        let mut examined_route_cursor = 0;
        let cursor = deferred.map_or(if continuing { self.route_cursor } else { 0 }, |route| {
            route.resume_cursor
        });
        let continuation = |resume_cursor| DeferredFrontierRoute {
            source: source_snapshot.as_ref().clone(),
            start_body: deferred.map_or(current_body.0, |old| old.start_body),
            start_orientation: deferred.map_or(current_body.1, |old| old.start_orientation),
            resume_cursor,
            rotation_offset,
        };
        delta.route_cursor = Some(0);
        delta.deferred_route = Some(None);
        for (ordinal, mut candidate) in candidates.into_iter().enumerate().skip(cursor) {
            examined_route_cursor = ordinal + 1;
            if nuwax_nav_pct_native::planning_cancelled() {
                return Err(ExploreError::SourceError(
                    "frontier planning cancelled".into(),
                ));
            }
            if self.excluded(candidate.support_xyz, vertical_radius)
                || distance(robot_support, candidate.support_xyz) <= minimum_distance
            {
                continue;
            }
            let region = nuwax_explore_routing::region_key(candidate.support_xyz);
            // 无视域标定时收益不会再改变；同区域后续候选必然被区域路由丢弃。
            // 首个候选无效时继续检查下一项，不能把候选上限当作无前沿证明。
            if self.observation_model.is_none()
                && accepted_regions.contains(&region)
                && !retry_matches(candidate.support_xyz)
            {
                continue;
            }
            if self.observation_model.is_some()
                && !retry_matches(candidate.support_xyz)
                && admissible
                    .iter()
                    .filter(
                        |(candidate, _, _): &&(
                            FrontierCandidate,
                            BodyPose,
                            Option<nuwax_explore_routing::observation::ViewEstimate>,
                        )| {
                            nuwax_explore_routing::region_key(candidate.support_xyz) == region
                        },
                    )
                    .count()
                    >= 2
            {
                continue;
            }
            if evaluated_candidate_count == MAX_FRONTIER_CANDIDATE_EVALUATIONS {
                route_budget_limited = true;
                next_route_cursor = ordinal;
                break;
            }
            evaluated_candidate_count += 1;
            let base_yaw = (candidate.support_xyz[1] - robot_support[1])
                .atan2(candidate.support_xyz[0] - robot_support[0]);
            let directional_view = self
                .observation_model
                .as_ref()
                .is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6);
            // 机体能否停留与传感器视场独立；直指前沿不适合时仍尝试沿边缘停留。
            let turns = if snapshot.body_envelope().is_some() || directional_view {
                4
            } else {
                1
            };
            let mut best = None;
            for i in 0..turns {
                let yaw = base_yaw + i as f64 * std::f64::consts::FRAC_PI_2;
                let candidate_quaternion =
                    if self.observation_model.is_some() || snapshot.body_envelope().is_some() {
                        [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]
                    } else {
                        quaternion
                    };
                let body = snapshot
                    .body_position_from_support(
                        candidate.support_xyz,
                        candidate_quaternion,
                        self.body_to_support_z_m,
                    )
                    .map_err(|error| ExploreError::SourceError(error.to_string()))?;
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
            let Some((mut body, mut candidate_quaternion, mut view, _)) = best else {
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
            if route_budget.exhausted() {
                route_budget_limited = true;
                next_route_cursor = ordinal;
                break;
            }
            route_query_count += 1;
            let Some((cost, endpoint, route_yaw)) =
                legacy_frontier_rank_cost(&mut search, current_body.0, body, &route_budget)
            else {
                continue;
            };
            body = endpoint;
            if !directional_view {
                candidate_quaternion = [0.0, 0.0, (route_yaw * 0.5).sin(), (route_yaw * 0.5).cos()];
            }
            candidate.support_xyz = snapshot
                .support_position_from_body(body, candidate_quaternion, self.body_to_support_z_m)
                .map_err(|error| ExploreError::SourceError(error.to_string()))?;
            if self.excluded(candidate.support_xyz, vertical_radius) {
                continue;
            }
            if let Some(model) = self.observation_model.as_ref() {
                view = Some(
                    snapshot
                        .estimate_observation_view(
                            model,
                            body,
                            candidate_quaternion,
                            &observation_targets,
                        )
                        .map_err(ExploreError::SourceError)?,
                );
                candidate.gain = view.as_ref().map_or(0, |v| v.cells.len()) as f64
                    * f64::from(snapshot.config().voxel_size).powi(2);
            }
            candidate.travel_cost = cost;
            admissible.push((candidate, (body, candidate_quaternion), view));
            accepted_regions.insert(region);
            if admissible.len() >= 8 {
                break;
            }
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
                if nuwax_nav_pct_native::planning_cancelled() {
                    return Err(ExploreError::SourceError(
                        "frontier planning cancelled".into(),
                    ));
                }
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
                    goals
                        .iter()
                        .map(|&target| {
                            legacy_frontier_rank_cost(
                                &mut search,
                                admissible[source].1 .0,
                                admissible[target].1 .0,
                                &route_budget,
                            )
                            .map_or(f64::INFINITY, |(cost, _, _)| cost)
                        })
                        .collect()
                },
            );
            delta.regional = Some(regional_delta);
            selection
        } else {
            None
        };
        if nuwax_nav_pct_native::planning_cancelled() {
            return Err(ExploreError::SourceError(
                "frontier planning cancelled".into(),
            ));
        }
        // An exactly spent final reservation is complete, not unfinished work.
        // The candidate loop already records a cursor when another query remains.
        let selected_index =
            retry_index.or_else(|| selection.as_ref().map(|selection| selection.index));
        let Some((mut candidate, (mut body, mut candidate_quaternion), mut view)) =
            selected_index.and_then(|index| admissible.get(index)).cloned()
        else {
            if next_route_cursor > 0 && next_route_cursor < geometric_candidate_count {
                delta.deferred_route = Some(Some(continuation(next_route_cursor)));
                delta.route_cursor = Some(next_route_cursor);
                delta.last_no_frontier = Some(false);
                delta.last_error = Some(Some("前沿路线继续处理剩余候选".into()));
                return Ok(ExplorationPoll::Pending);
            }
            delta.retry_support = Some(None);
            delta.last_no_frontier = Some(true);
            delta.last_error = Some(None);
            return Ok(ExplorationPoll::NoFrontier);
        };
        let ranking_ms = ranking_started.elapsed().as_secs_f64() * 1000.0;
        let exact_started = std::time::Instant::now();
        route_budget.reserved = 0;
        let Some((travel, endpoint, yaw)) = legacy_frontier_route_cost(&mut search, current_body.0, body, &route_budget) else {
            // 全图与目标窗口的层表示不同可能使精确核验失败；继续剩余候选，
            // 不把资源额度或单次表示差异记为导航失败，也不重算刚完成的批次。
            delta.deferred_route = Some(Some(continuation(examined_route_cursor)));
            delta.route_cursor = Some(examined_route_cursor);
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        };
        body = endpoint;
        if !self.observation_model.as_ref().is_some_and(|model| model.horizontal_fov_rad < std::f64::consts::TAU - 1e-6) {
            candidate_quaternion = [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()];
        }
        candidate.support_xyz = snapshot.support_position_from_body(body, candidate_quaternion, self.body_to_support_z_m)
            .map_err(|error| ExploreError::SourceError(error.to_string()))?;
        if let Some(model) = self.observation_model.as_ref() {
            view = Some(snapshot.estimate_observation_view(model, body, candidate_quaternion, &observation_targets)
                .map_err(ExploreError::SourceError)?);
            candidate.gain = view.as_ref().map_or(0, |v| v.cells.len()) as f64 * f64::from(snapshot.config().voxel_size).powi(2);
        }
        if self.excluded(candidate.support_xyz, vertical_radius)
            || view.as_ref().is_some_and(|view| view.cells.is_empty()) {
            delta.deferred_route = Some(Some(continuation(examined_route_cursor)));
            delta.route_cursor = Some(examined_route_cursor);
            delta.last_no_frontier = Some(false);
            return Ok(ExplorationPoll::Pending);
        }
        candidate.travel_cost = travel;
        let mut goal=self.make_candidate_goal(state,&candidate,(body,candidate_quaternion),view.as_ref(),travel,delta)?;
        for (key,value) in [
            ("route_cost_basis","full_transition_source_tree_then_selected_astar".into()),
            ("route_exact_queries",(search.stats().exact_queries).to_string()),
            ("route_source_searches",search.stats().source_searches.to_string()),
            ("route_query_count",route_budget.queries.get().to_string()),
            ("candidate_route_query_count",route_query_count.to_string()),
            ("route_expanded_successes",route_budget.expanded.get().to_string()),
            ("route_budget_limited",route_budget_limited.to_string()),
            ("geometric_candidate_count",geometric_candidate_count.to_string()),
            ("evaluated_candidate_count",evaluated_candidate_count.to_string()),
            ("admissible_candidate_count",admissible.len().to_string()),
            ("selection_method",if retry_index.is_some(){"navigation_retry"}else{"regional_route"}.into()),
        ] {goal.metadata.insert(key.into(),value);}
        if let Some(selection)=selection {
            for (key,value) in [
                ("region_wait_rounds",selection.priority_wait_rounds.to_string()),
                ("route_regions",selection.regional_order.len().to_string()),
                ("available_regions",selection.region_count.to_string()),
                ("route_cost_m",selection.route_cost.to_string()),
            ] {goal.metadata.insert(key.into(),value);}
        }
        tracing::info!(event="exploration.selection.stages", captured_now_ns=state.now.nanos, geometry_ms, frontier_ms, native_ms, native_input_ms, planner_lock_ms, ranking_ms,
            exact_ms=exact_started.elapsed().as_secs_f64()*1000.0,
            candidates=geometric_candidate_count, targets=observation_targets.len(),
            route_queries=route_budget.queries.get(), expanded=route_budget.expanded.get(),
            "完整前沿评估阶段耗时");
        Ok(self.stage_selected_goal(goal,candidate.support_xyz,vertical_radius,candidate.frontier_cell,source_snapshot,delta))
    }
}

fn legacy_frontier_route_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3],
    goal: [f64; 3],
    budget: &FrontierRouteBudget,
) -> Option<(f64, [f64; 3], f64)> {
    legacy_frontier_query_cost(search, start, goal, budget, false)
}

pub(super) fn legacy_frontier_rank_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3], goal: [f64; 3], budget: &FrontierRouteBudget,
) -> Option<(f64, [f64; 3], f64)> {
    legacy_frontier_query_cost(search, start, goal, budget, true)
}

fn legacy_frontier_query_cost(
    search: &mut nuwax_nav_pct_native::MultiGoalSearch<'_>,
    start: [f64; 3], goal: [f64; 3], budget: &FrontierRouteBudget, rank: bool,
) -> Option<(f64, [f64; 3], f64)> {
    if budget.exhausted() {
        return None;
    }
    // 保留逻辑目标预留次数；转移复用不能让本轮跳过原预算或游标。
    budget.queries.set(budget.queries.get() + 1);
    let started = std::time::Instant::now();
    let result = if rank { search.rank_costs(start, &[goal]).remove(0) } else { search.cost(start, goal) };
    tracing::debug!(target: "navigation.exploration.compute", elapsed_ms = started.elapsed().as_secs_f64() * 1000.0,
        query = budget.queries.get(), ?start, ?goal,
        expanded = result.as_ref().ok().map(|route| route.expanded), error = ?result.as_ref().err(),
        checks = search.stats().checks, hits = search.stats().hits,
        source_searches = search.stats().source_searches, rejected = search.stats().rejected, "前沿共享路线查询完成");
    tracing::debug!(target: "navigation.exploration.compute", stats = ?search.stats(), "前沿搜索累计分段");
    let route = result.ok()?;
    budget.expanded.set(budget.expanded.get() + route.expanded);
    Some((route.cost, route.endpoint, route.yaw))
}
