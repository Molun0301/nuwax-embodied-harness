// Diagnostic only: real kernel, owner, controller and calibrated feedback.
// No planner gates, synthetic Candidate values, changed budgets or tolerances.
// All wall timing is instrumented; fixed-step model time is reported separately.
mod partial_route_priority_tests {
    include!("route_partial_rejoin_tests.rs");
    include!("route_repeated_detour_tests.rs");
    include!("route_phase_qualification_tests.rs");
    use super::*;
    use crate::route_planning::priority_probe as probe;
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::Point2;
    use std::collections::BTreeMap;
    use std::sync::{atomic::AtomicU64, Mutex};

    #[derive(Clone, Copy, Debug)]
    enum Case {
        // Wider safe approach: can partial deliver useful movement early?
        WideApproach,
        // Short approach: what really happens when its prefix runs out?
        ShortApproach,
    }
    impl Case {
        fn start_x(self) -> f64 {
            match self {
                Self::WideApproach => -1.2,
                Self::ShortApproach => -0.75,
            }
        }
    }

    fn static_short_wall() -> Arc<SurfaceSnapshot> {
        use nuwax_nav_mls_planner::mls_planner::Config;
        let size = 0.01_f32;
        // Exact flat-source construction and body from Scan's existing fixture;
        // only the support extent and finite wall geometry differ.
        let mut cells: Vec<[i32; 3]> = (-180..181)
            .flat_map(|x| (-125..156).map(move |y| [x, y, -1]))
            .collect();
        cells.extend(
            (0..4).flat_map(|x| (-25..55).flat_map(move |y| (0..65).map(move |z| [x, y, z]))),
        );
        let points: Vec<_> = cells
            .iter()
            .map(|p| {
                (
                    (p[0] as f32 + 0.5) * size,
                    (p[1] as f32 + 0.5) * size,
                    (p[2] as f32 + 0.5) * size,
                )
            })
            .collect();
        let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
        for key in cells {
            blocks
                .entry(key.map(|v| v.div_euclid(32)))
                .or_default()
                .insert(
                    key,
                    SurfaceObservation {
                        observed_ns: 1,
                        hits: 2,
                    },
                );
        }
        let mut observations = SurfaceObservations::new(size, 32).unwrap();
        for (key, values) in blocks {
            observations.replace_block(key, values).unwrap();
        }
        observations.mark_complete_geometry_source();
        let cfg = Config {
            world_frame: "map".into(),
            voxel_size: 0.1,
            robot_height: 0.55,
            surface_closing_radius: 0.0,
            node_spacing_m: 0.3,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.1,
            wall_buffer_weight: 2.0,
            max_slope_deg: 18.0,
            step_threshold_m: 0.0,
            min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        Arc::new(
            SurfaceSnapshot::from_points(&points, cfg, size)
                .unwrap()
                .with_body_envelope(SurfaceBodyEnvelope {
                    footprint_xy_m: vec![[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]],
                    z_bounds_m: [-0.15, 0.15],
                    body_to_support_z_m: 0.4,
                    max_step_up_m: 0.0,
                    max_step_down_m: 0.0,
                    max_cross_slope_deg: 18.0,
                    support_uncertainty_m: 0.0,
                })
                .unwrap()
                .with_observations(Some(Arc::new(observations))),
        )
    }

    fn pose(x: f64, nanos: i64) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(
                nanos.div_euclid(1_000_000_000),
                nanos.rem_euclid(1_000_000_000) as u32,
            ),
            pose: Pose3 {
                position: Point3::new(x, 0.15, 0.4),
                orientation: Quaternion::IDENTITY,
            },
        }
    }

    fn prepare_case(case: Case) -> RouteFollower {
        let config = comparison_config(Some(true));
        assert_eq!(
            config.scan.search_expansion_limit,
            nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT
        );
        let mut follower = RouteFollower::new(config).unwrap();
        follower.configure_boundary(&[[-1.7, -1.15], [1.7, -1.15], [1.7, 1.45], [-1.7, 1.45]]);
        follower
            .update_execution_map(ExecutionMap::Surface(static_short_wall()))
            .unwrap();
        let start = pose(case.start_x(), 1_000_000_000);
        let goal = pose(1.1, 1_000_000_000);
        follower.update_odometry(start.clone());
        follower
            .configure_goal(&goal, GoalOptions::default())
            .unwrap();
        follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: vec![start, goal.clone()],
                }),
                None,
                goal,
            )
            .unwrap();
        assert_eq!(follower.local_planning_calls(), 0);
        assert!(follower.submitted.is_none());
        assert!(
            follower
                .map
                .as_ref()
                .unwrap()
                .validate_body_motion(
                    body(follower.pose.as_ref().unwrap()),
                    body(follower.goal.as_ref().unwrap()),
                    &follower.boundary
                )
                .is_err(),
            "fixture old straight route must actually intersect the short wall"
        );
        follower
    }

    // Mirrors only request construction at route_follower.rs:255-259. It does
    // not bypass PlanKernel::compute's actual normalization/materialization.
    fn snapshot_request(follower: &RouteFollower, detour: bool) -> PlanRequest {
        PlanRequest {
            source: follower.map.as_ref().unwrap().clone(),
            pose: follower.pose.as_ref().unwrap().clone(),
            boundary: follower.boundary.clone(),
            reference: Vec::new(),
            reference_source: Some(ReferenceSource {
                route: follower.route.as_ref().unwrap().clone(),
                cumulative: follower.cumulative.clone(),
                goal: follower.goal.clone(),
                options: follower.options,
                offset: follower.config.body_to_support_z_m,
                progress: follower.progress,
            }),
            final_yaw: None,
            arrival: None,
            end: 0.0,
            progress: follower.progress,
            generation: follower.route_generation,
            straight: false,
            detour,
            scan: follower.scan.clone(),
        }
    }

    fn duplicate_request(r: &PlanRequest) -> PlanRequest {
        PlanRequest {
            source: r.source.clone(),
            pose: r.pose.clone(),
            boundary: r.boundary.clone(),
            reference: r.reference.clone(),
            reference_source: r.reference_source.as_ref().map(|s| ReferenceSource {
                route: s.route.clone(),
                cumulative: s.cumulative.clone(),
                goal: s.goal.clone(),
                options: s.options,
                offset: s.offset,
                progress: s.progress,
            }),
            final_yaw: r.final_yaw,
            arrival: r.arrival,
            end: r.end,
            progress: r.progress,
            generation: r.generation,
            straight: r.straight,
            detour: r.detour,
            scan: r.scan.clone(),
        }
    }

    fn independently_check_full_candidate(r: &PlanRequest, c: &Candidate) -> Result<(), String> {
        if c.partial {
            return Err("detour returned a partial candidate".into());
        }
        let first = c.path.poses.first().ok_or("empty detour candidate")?;
        if pose_bits(body(first)) != pose_bits(body(&r.pose)) {
            return Err("detour changed its requested actual start".into());
        }
        for edge in c.path.poses.windows(2) {
            r.source
                .validate_body_motion(body(&edge[0]), body(&edge[1]), &r.boundary)
                .map_err(|e| format!("independent detour edge failed: {e}"))?;
        }
        let source = r
            .reference_source
            .as_ref()
            .ok_or("missing original source")?;
        let goal = source.goal.as_ref().ok_or("missing original goal")?;
        let end = c.path.poses.last().unwrap();
        if end.pose.position.distance(goal.pose.position) >= source.options.position_tolerance_m {
            return Err("detour did not reach the original 3D arrival ball".into());
        }
        let d = end.pose.orientation.yaw() - goal.pose.orientation.yaw();
        let yaw_error = d.sin().atan2(d.cos()).abs();
        if source.options.requires_orientation() && yaw_error >= source.options.yaw_tolerance_rad {
            return Err("detour did not satisfy original final yaw".into());
        }
        Ok(())
    }

    fn compute_observed(
        kernel: &mut PlanKernel,
        request: PlanRequest,
        cancel: &crate::planning_owner::Cancellation,
    ) -> (PlanResult, Duration, probe::Frame) {
        probe::begin();
        #[cfg(feature = "body-query-profile")]
        let body_capture = probe::BodyCapture::begin();
        let began = Instant::now();
        let result = kernel.compute(request, cancel);
        let elapsed = began.elapsed();
        #[cfg(feature = "body-query-profile")]
        body_capture.finish();
        let audit = probe::end();
        (result, elapsed, audit)
    }

    fn detour_witness(case: Case) {
        let follower = prepare_case(case);
        let request = snapshot_request(&follower, true);
        assert!(
            request
                .source
                .recent_obstacle_hints([[-8.0, -8.0], [8.0, 8.0]], body(&request.pose))
                .unwrap()
                .is_empty(),
            "fixture must retain the natural empty recent overlay"
        );
        let cancel = crate::planning_owner::Cancellation {
            epoch: Arc::new(AtomicU64::new(1)),
            expected: 1,
        };
        let mut kernel = PlanKernel::new(follower.config.clone());
        let dispatch = tracing::Dispatch::new(probe::Subscriber);
        let (result, wall, audit) = tracing::dispatcher::with_default(&dispatch, || {
            compute_observed(&mut kernel, request, &cancel)
        });
        eprintln!(
            "PARTIAL_PRIORITY_WITNESS case={case:?} kernel_wall_ms={} work={audit:?}",
            wall.as_secs_f64() * 1000.0
        );
        let candidate = result.outcome.unwrap_or_else(|e| {
            panic!("FIXTURE_UNQUALIFIED case={case:?}: initial detour failed {e:?}")
        });
        independently_check_full_candidate(&result.request, &candidate)
            .unwrap_or_else(|e| panic!("FIXTURE_UNQUALIFIED case={case:?}: {e}"));
        assert!(
            audit.pct.iter().any(|work| work.stats.is_some()),
            "FIXTURE_UNQUALIFIED: detour PCT did not succeed"
        );
        assert!(
            audit.branches.iter().any(|b| b.detour
                && b.follow_reference
                && b.reference_changed
                && b.partial == Some(false)),
            "FIXTURE_UNQUALIFIED: no actual changed-reference complete detour"
        );
        eprintln!(
            "PARTIAL_PRIORITY_WITNESS_ACCEPTED case={case:?} poses={} start={:?} end={:?}",
            candidate.path.poses.len(),
            body(candidate.path.poses.first().unwrap()),
            body(candidate.path.poses.last().unwrap())
        );
        // Follower/source/kernel all drop here. Each timed arm rebuilds its source.
    }

    struct ComputeRecord {
        id: u64,
        epoch: u64,
        input: PlanRequest,
        began: Instant,
        finished: Instant,
        kernel_wall: Duration,
        audit: probe::Frame,
        path: Option<Path3D>,
        partial: Option<bool>,
        failures: Vec<LocalPlanError>,
        forced_initial_detour: bool,
    }
    type Records = Arc<Mutex<Vec<ComputeRecord>>>;
    fn observe_owner(follower: &mut RouteFollower, detour_first: bool) -> Records {
        assert_eq!(follower.local_planning_calls(), 0);
        assert!(follower.submitted.is_none());
        let records: Records = Arc::new(Mutex::new(Vec::new()));
        let sink = records.clone();
        let mut kernel = PlanKernel::new(follower.config.clone());
        let dispatch = tracing::Dispatch::new(probe::Subscriber);
        let mut first = true;
        follower.owner = Some(
            PlanningOwner::new(move |mut request: PlanRequest, cancel| {
                let forced_initial_detour = first && detour_first;
                first = false;
                if forced_initial_detour {
                    request.detour = true;
                }
                let (id, epoch) = crate::planning_owner::current_request_for_test()
                    .expect("real owner request context");
                let began = Instant::now();
                let (result, kernel_wall, audit) =
                    tracing::dispatcher::with_default(&dispatch, || {
                        compute_observed(&mut kernel, request, cancel)
                    });
                let finished = Instant::now();
                let (path, partial, failures) = match &result.outcome {
                    Ok(c) => (Some(c.path.clone()), Some(c.partial), Vec::new()),
                    Err(e) => (None, None, e.clone()),
                };
                sink.lock().unwrap().push(ComputeRecord {
                    id,
                    epoch,
                    input: duplicate_request(&result.request),
                    began,
                    finished,
                    kernel_wall,
                    audit,
                    path,
                    partial,
                    failures,
                    forced_initial_detour,
                });
                result
            })
            .unwrap(),
        );
        records
    }

    #[derive(Debug)]
    struct TickChange {
        tick: u64,
        model_seconds: f64,
        wall_ms: f64,
        progress: f64,
        local_end: f64,
        local_plans: u64,
        submitted: Option<(u64, Point3, bool)>,
        next_request: u64,
        pose: nuwax_nav_scan::BodyPose,
        applied: Twist2,
        result: String,
        wait_reason: Option<String>,
    }
    #[derive(Debug)]
    struct Summary {
        case: Case,
        detour_first: bool,
        arrived: bool,
        stopped: bool,
        stop_reason: String,
        final_pose: nuwax_nav_scan::BodyPose,
        final_applied: Twist2,
        feedback_dt_s: f64,
        model_seconds: f64,
        wall_seconds: f64,
        arrival_wall_ms: Option<f64>,
        first_admission_ms: Option<f64>,
        first_nonzero_ms: Option<f64>,
        first_motion_ms: Option<f64>,
        first_complete_admission_ms: Option<f64>,
        kernel_wall_ms: f64,
        control_wall_ms: f64,
        actual_sweep_wall_ms: f64,
        pct_calls: usize,
        pct_expanded_successful_routes: usize,
        pct_failed_stats_unavailable: usize,
        scan_calls: usize,
        scan_expanded: u64,
        scan_candidates: u64,
        scan_search_sweeps: u64,
        partial_admissions: usize,
        total_admissions: usize,
        computes: usize,
        actual_length: f64,
        actual_abs_turn: f64,
        stationary_repeat_pairs: usize,
        stationary_failed_input_pairs: usize,
        stationary_same_failure_signature_pairs: usize,
        initial_nominal_partial_generated: bool,
        first_nominal_partial: bool,
        nonzero_commands_while_partial: u64,
        overdue_ticks: u64,
        max_tick_ms: f64,
    }

    fn stamped(current: nuwax_nav_scan::BodyPose, model_elapsed: Duration) -> PoseStamped {
        PoseStamped {
            frame_id: NavFrameId::Map,
            stamp: Timestamp::from_unix(
                1 + model_elapsed.as_secs() as i64,
                model_elapsed.subsec_nanos(),
            ),
            pose: Pose3 {
                position: Point3::new(current.0[0], current.0[1], current.0[2]),
                orientation: Quaternion {
                    x: current.1[0],
                    y: current.1[1],
                    z: current.1[2],
                    w: current.1[3],
                },
            },
        }
    }
    fn model_edge(
        source: &SurfaceSnapshot,
        current: nuwax_nav_scan::BodyPose,
        applied: Twist2,
        command: Twist2,
        limits: &nuwax_nav_types_messages::MotionLimits,
        boundary: &[[f64; 2]],
        dt: f64,
    ) -> (nuwax_nav_scan::BodyPose, Twist2, Duration) {
        assert!(
            command.linear.x.is_finite()
                && command.linear.y.is_finite()
                && command.angular_z.is_finite()
        );
        assert!(command.linear.x >= 0.0 && command.linear.x <= limits.max_forward_mps + 1e-9);
        assert_eq!(command.linear.y, 0.0);
        assert!(command.angular_z.abs() <= limits.max_angular_rps + 1e-9);
        let linear_rate = if command.linear.x < applied.linear.x {
            limits.guaranteed_braking_mps2
        } else {
            limits.linear_acceleration_mps2
        };
        let angular_rate = if command.angular_z.abs() < applied.angular_z.abs() {
            limits.guaranteed_angular_braking_rps2
        } else {
            limits.angular_acceleration_rps2
        };
        let next_applied = Twist2 {
            linear: Point2::new(
                applied.linear.x
                    + (command.linear.x - applied.linear.x)
                        .clamp(-linear_rate * dt, linear_rate * dt),
                0.0,
            ),
            angular_z: applied.angular_z
                + (command.angular_z - applied.angular_z)
                    .clamp(-angular_rate * dt, angular_rate * dt),
        };
        let mean = Twist2 {
            linear: Point2::new((applied.linear.x + next_applied.linear.x) * 0.5, 0.0),
            angular_z: (applied.angular_z + next_applied.angular_z) * 0.5,
        };
        let next = nuwax_nav_control::integrate(
            nuwax_nav_control::PlanarPose {
                x: current.0[0],
                y: current.0[1],
                yaw: stamped(current, Duration::ZERO).pose.orientation.yaw(),
            },
            mean,
            dt,
        );
        let actual = (
            [next.x, next.y, current.0[2]],
            [0.0, 0.0, (next.yaw * 0.5).sin(), (next.yaw * 0.5).cos()],
        );
        let began = Instant::now();
        source.validate_body_motion(current,actual,boundary)
            .unwrap_or_else(|e|panic!("UNSAFE_ACTUAL_EDGE {e}: {current:?} -> {actual:?}; applied={applied:?}; command={command:?}"));
        (actual, next_applied, began.elapsed())
    }

    fn run_arm(case: Case, detour_first: bool) -> Summary {
        let mut follower = prepare_case(case);
        let source = follower.map.as_ref().unwrap().clone();
        let boundary = follower.boundary.clone();
        let goal = follower.goal.clone().unwrap();
        let options = follower.options;
        let original_route = follower.route.clone().unwrap();
        let limits = follower.config.motion_limits.clone().unwrap();
        let dt = 1.0 / follower.config.controller.control_frequency;
        assert!(dt.is_finite() && dt > 0.0);
        let period = Duration::from_secs_f64(dt);
        // All feedback clocks follow the actual config, without overriding it.
        // Preserve the original intended 900 * 0.05s model observation window.
        let model_watchdog = Duration::from_secs(45);
        let snapshot_period = Duration::from_secs(1);
        let mut model_elapsed = Duration::ZERO;
        let mut next_snapshot = Duration::ZERO;
        let records = observe_owner(&mut follower, detour_first);
        let dispatch = tracing::Dispatch::new(probe::Subscriber);
        let mut current = body(follower.pose.as_ref().unwrap());
        let mut applied = Twist2::ZERO;
        let mut tick = 0_u64;
        let mut arrived = false;
        let mut stopped = false;
        let mut reason = "WATCHDOG_INCONCLUSIVE".to_owned();
        let mut arrival_ms = None;
        let mut first_nonzero = None;
        let mut first_motion = None;
        let mut nonzero_commands_while_partial = 0_u64;
        let mut control_wall = Duration::ZERO;
        let mut sweep_wall = Duration::ZERO;
        let mut length = 0.0;
        let mut turning = 0.0;
        let mut late = 0;
        let mut max_tick = 0.0_f64;
        let mut changes = Vec::new();
        let mut last_state = None;
        probe::begin_control();
        let began = Instant::now();
        tracing::dispatcher::with_default(&dispatch, || {
            // A diagnostic watchdog, never a product reachability conclusion.
            while model_elapsed < model_watchdog && began.elapsed() < Duration::from_secs(180) {
                let tick_begin = Instant::now();
                let snapshot_due = model_elapsed >= next_snapshot;
                if snapshot_due {
                    next_snapshot = model_elapsed + snapshot_period;
                }
                let mut command = Twist2::ZERO;
                if !arrived {
                    follower.observe_applied_command(applied);
                    follower.update_odometry(stamped(current, model_elapsed));
                    // Republishing unchanged evidence must not reset the route or owner.
                    if tick > 0 && snapshot_due {
                        follower
                            .update_execution_map(ExecutionMap::Surface(source.clone()))
                            .unwrap();
                    }
                    let step_begin = Instant::now();
                    let mut step = follower.step(0.32);
                    if let PlannerStep::Command(selected) = step {
                        if let Some(rechecked) =
                            follower.revalidate_selected_command_at_current_state(selected)
                        {
                            step = rechecked;
                        }
                    }
                    control_wall += step_begin.elapsed();
                    assert_eq!(
                        follower.goal.as_ref(),
                        Some(&goal),
                        "prefix must not replace the original goal"
                    );
                    assert_eq!(
                        follower.route.as_ref(),
                        Some(&original_route),
                        "partial refresh must retain global route"
                    );
                    let state = (
                        follower.next_request,
                        follower.local_plans(),
                        follower.admitted_generation,
                        matches!(step, PlannerStep::Pending(_)),
                    );
                    if last_state != Some(state) || snapshot_due {
                        changes.push(TickChange {
                            tick,
                            model_seconds: model_elapsed.as_secs_f64(),
                            wall_ms: began.elapsed().as_secs_f64() * 1000.0,
                            progress: follower.progress,
                            local_end: follower.local_end,
                            local_plans: follower.local_plans(),
                            submitted: follower.submitted,
                            next_request: follower.next_request,
                            pose: current,
                            applied,
                            result: format!("{step:?}"),
                            wait_reason: follower.execution_wait_reason(),
                        });
                        last_state = Some(state);
                    }
                    match step {
                        PlannerStep::Command(v) => {
                            command = v;
                            if v != Twist2::ZERO
                                && follower.local_end + 1e-6 < *follower.cumulative.last().unwrap()
                            {
                                nonzero_commands_while_partial += 1;
                            }
                            if v != Twist2::ZERO && first_nonzero.is_none() {
                                first_nonzero = Some(began.elapsed().as_secs_f64() * 1000.0);
                            }
                        }
                        PlannerStep::Pending(_) => {}
                        PlannerStep::Succeeded => {
                            assert!(
                                current_position(current).distance(goal.pose.position)
                                    < options.position_tolerance_m,
                                "a prefix endpoint must never satisfy the original goal"
                            );
                            arrived = true;
                            arrival_ms = Some(began.elapsed().as_secs_f64() * 1000.0);
                            reason = "ORIGINAL_GOAL_REACHED".to_owned();
                            follower.stop();
                        }
                        other => {
                            reason = format!("PLANNER_STOP_REQUIRES_REVIEW {other:?}");
                            break;
                        }
                    }
                }
                let (next, next_applied, swept) =
                    model_edge(&source, current, applied, command, &limits, &boundary, dt);
                sweep_wall += swept;
                let distance = current_position(current).distance(current_position(next));
                length += distance;
                let dyaw = stamped(next, Duration::ZERO).pose.orientation.yaw()
                    - stamped(current, Duration::ZERO).pose.orientation.yaw();
                turning += dyaw.sin().atan2(dyaw.cos()).abs();
                if (distance > 0.0 || dyaw != 0.0) && first_motion.is_none() {
                    first_motion = Some(began.elapsed().as_secs_f64() * 1000.0);
                }
                current = next;
                applied = next_applied;
                tick += 1;
                model_elapsed += period;
                if arrived && applied == Twist2::ZERO {
                    stopped = true;
                    break;
                }
                let elapsed = tick_begin.elapsed();
                max_tick = max_tick.max(elapsed.as_secs_f64() * 1000.0);
                if elapsed < period {
                    std::thread::sleep(period - elapsed);
                } else {
                    late += 1;
                }
            }
        });
        let admissions = probe::end_control();
        // Count any compute already in progress, even if cancellation retires it.
        follower.stop();
        drop(follower);
        let wall = began.elapsed().as_secs_f64();
        let records = records.lock().unwrap();
        let initial_nominal_partial_generated = records.first().is_some_and(|r| {
            !r.input.detour
                && r.audit.branches.len() == 1
                && !r.audit.branches[0].detour
                && r.partial == Some(true)
        });
        let first_nominal_partial = initial_nominal_partial_generated
            && records
                .first()
                .is_some_and(|r| admissions.iter().any(|a| a.request_id == r.id && a.partial));
        let repeat_pairs = records
            .windows(2)
            .filter(|pair| {
                let (a, b) = (&pair[0], &pair[1]);
                a.partial == Some(true)
                    && b.partial == Some(true)
                    && a.input.source.same_body_query_view(&b.input.source)
                    && pose_bits(body(&a.input.pose)) == pose_bits(body(&b.input.pose))
                    && a.input.progress.to_bits() == b.input.progress.to_bits()
                    && a.input.end.to_bits() == b.input.end.to_bits()
                    && a.input.generation == b.input.generation
                    && a.path
                        .as_ref()
                        .and_then(|p| p.poses.last())
                        .map(|p| pose_bits(body(p)))
                        == b.path
                            .as_ref()
                            .and_then(|p| p.poses.last())
                            .map(|p| pose_bits(body(p)))
            })
            .count();
        // Independent from the legacy partial-to-partial counter. No movement
        // epsilon: these compare the recorded exact request inputs and failures.
        let same_failed_inputs = |a: &ComputeRecord, b: &ComputeRecord| {
            a.partial.is_none() && b.partial.is_none()
                && !a.failures.is_empty() && !b.failures.is_empty()
                && a.epoch == b.epoch
                && a.input.source.same_body_query_view(&b.input.source)
                && pose_bits(body(&a.input.pose)) == pose_bits(body(&b.input.pose))
                && a.input.progress.to_bits() == b.input.progress.to_bits()
                && a.input.end.to_bits() == b.input.end.to_bits()
                && a.input.generation == b.input.generation
                && a.input.detour == b.input.detour && a.input.straight == b.input.straight
                && exact_points(&a.input.boundary, &b.input.boundary)
                && exact_points(&a.input.reference, &b.input.reference)
                && a.input.final_yaw.map(f64::to_bits) == b.input.final_yaw.map(f64::to_bits)
                && a.input.arrival.map(|(p,t)|(p.map(f64::to_bits),t.to_bits()))
                    == b.input.arrival.map(|(p,t)|(p.map(f64::to_bits),t.to_bits()))
        };
        let failed_input_pairs = records.windows(2)
            .filter(|pair|same_failed_inputs(&pair[0],&pair[1])).count();
        let same_failure_signature_pairs = records.windows(2).filter(|pair| {
            same_failed_inputs(&pair[0],&pair[1])
                && pair[0].failures.len()==pair[1].failures.len()
                && pair[0].failures.iter().zip(&pair[1].failures)
                    .all(|(a,b)|a.kind==b.kind && a.reason==b.reason
                        && a.closest_pose.map(pose_bits)==b.closest_pose.map(pose_bits))
        }).count();
        if arrived && stopped {
            assert!(
                current_position(current).distance(goal.pose.position)
                    < options.position_tolerance_m,
                "post-arrival calibrated braking left original arrival ball"
            );
            let yaw = stamped(current, Duration::ZERO).pose.orientation.yaw()
                - goal.pose.orientation.yaw();
            assert!(
                !options.requires_orientation()
                    || yaw.sin().atan2(yaw.cos()).abs() < options.yaw_tolerance_rad
            );
        }
        for r in records.iter() {
            if let Some(path) = &r.path {
                for edge in path.poses.windows(2) {
                    r.input
                        .source
                        .validate_body_motion(body(&edge[0]), body(&edge[1]), &r.input.boundary)
                        .unwrap();
                }
            }
            eprintln!("PARTIAL_PRIORITY_COMPUTE case={case:?} detour_first={detour_first} id={} epoch={} requested_detour={} forced_initial={} start={:?} progress={} end={} begin_ms={} finish_ms={} kernel_wall_ms={} partial={:?} endpoint={:?} failures={:?} audit={:?}",
                r.id,r.epoch,r.input.detour,r.forced_initial_detour,body(&r.input.pose),r.input.progress,r.input.end,
                r.began.duration_since(began).as_secs_f64()*1000.0,r.finished.duration_since(began).as_secs_f64()*1000.0,
                r.kernel_wall.as_secs_f64()*1000.0,r.partial,r.path.as_ref().and_then(|p|p.poses.last()).map(body),r.failures,r.audit);
        }
        for change in changes {
            eprintln!(
                "PARTIAL_PRIORITY_STATE case={case:?} detour_first={detour_first} {change:?}"
            );
        }
        for a in &admissions {
            eprintln!("PARTIAL_PRIORITY_ADMISSION case={case:?} detour_first={detour_first} id={} at_ms={} partial={} covered={} requested={}",
            a.request_id,a.at.duration_since(began).as_secs_f64()*1000.0,a.partial,a.covered_end,a.requested_end);
        }
        let summary = Summary {
            case,
            detour_first,
            arrived,
            stopped,
            stop_reason: reason,
            final_pose: current,
            final_applied: applied,
            feedback_dt_s: dt,
            model_seconds: model_elapsed.as_secs_f64(),
            wall_seconds: wall,
            arrival_wall_ms: arrival_ms,
            first_admission_ms: admissions
                .first()
                .map(|a| a.at.duration_since(began).as_secs_f64() * 1000.0),
            first_nonzero_ms: first_nonzero,
            first_motion_ms: first_motion,
            first_complete_admission_ms: admissions
                .iter()
                .find(|a| !a.partial)
                .map(|a| a.at.duration_since(began).as_secs_f64() * 1000.0),
            kernel_wall_ms: records
                .iter()
                .map(|r| r.kernel_wall.as_secs_f64() * 1000.0)
                .sum(),
            control_wall_ms: control_wall.as_secs_f64() * 1000.0,
            actual_sweep_wall_ms: sweep_wall.as_secs_f64() * 1000.0,
            pct_calls: records.iter().map(|r| r.audit.pct.len()).sum(),
            pct_expanded_successful_routes: records
                .iter()
                .flat_map(|r| &r.audit.pct)
                .filter_map(|p| p.stats.as_ref())
                .map(|p| p.expanded)
                .sum(),
            pct_failed_stats_unavailable: records
                .iter()
                .flat_map(|r| &r.audit.pct)
                .filter(|p| p.stats.is_none())
                .count(),
            scan_calls: records
                .iter()
                .flat_map(|r| &r.audit.branches)
                .filter(|b| !b.cached_negative)
                .count(),
            scan_expanded: records
                .iter()
                .flat_map(|r| &r.audit.searches)
                .map(|s| s.expanded)
                .sum(),
            scan_candidates: records
                .iter()
                .flat_map(|r| &r.audit.searches)
                .map(|s| s.candidates)
                .sum(),
            scan_search_sweeps: records
                .iter()
                .flat_map(|r| &r.audit.searches)
                .map(|s| s.sweeps)
                .sum(),
            partial_admissions: admissions.iter().filter(|a| a.partial).count(),
            total_admissions: admissions.len(),
            computes: records.len(),
            actual_length: length,
            actual_abs_turn: turning,
            stationary_repeat_pairs: repeat_pairs,
            stationary_failed_input_pairs: failed_input_pairs,
            stationary_same_failure_signature_pairs: same_failure_signature_pairs,
            initial_nominal_partial_generated,
            first_nominal_partial,
            nonzero_commands_while_partial,
            overdue_ticks: late,
            max_tick_ms: max_tick,
        };
        eprintln!("PARTIAL_PRIORITY_SUMMARY {summary:?}");
        summary
    }
    fn current_position(p: nuwax_nav_scan::BodyPose) -> Point3 {
        Point3::new(p.0[0], p.0[1], p.0[2])
    }

    fn audit_case(case: Case) {
        detour_witness(case);
        let normal = run_arm(case, false);
        let control = run_arm(case, true);
        if normal.arrived && normal.stopped && normal.first_nominal_partial {
            eprintln!("PARTIAL_PRIORITY_VERDICT case={case:?} DEADLOCK_HYPOTHESIS_CLOSED_IN_THIS_CASE masking_exercised=true partial_admissions={} computes={} stationary_repeat_pairs={}",
                normal.partial_admissions,normal.computes,normal.stationary_repeat_pairs);
        } else if normal.arrived && normal.stopped {
            eprintln!("PARTIAL_PRIORITY_VERDICT case={case:?} MASKING_NOT_EXERCISED_IN_FEEDBACK_CHAIN no_partial_deadlock_conclusion");
        } else if normal.first_nominal_partial
            && normal.stationary_repeat_pairs > 0
            && control.arrived
        {
            eprintln!("PARTIAL_PRIORITY_VERDICT case={case:?} OBSERVED_REPEAT_REQUIRES_REVIEW no_physical_impossibility_claim");
        } else if normal.first_nominal_partial {
            eprintln!("PARTIAL_PRIORITY_VERDICT case={case:?} NO_ARRIVAL_WITH_INITIAL_PARTIAL_IN_OBSERVATION_WINDOW stationary_failed_input_pairs={} stationary_same_failure_signature_pairs={} no_infinite_deadlock_claim",
                normal.stationary_failed_input_pairs,normal.stationary_same_failure_signature_pairs);
        } else {
            eprintln!(
                "PARTIAL_PRIORITY_VERDICT case={case:?} INCONCLUSIVE_OR_MASKING_NOT_EXERCISED"
            );
        }
        assert!(
            control.arrived && control.stopped,
            "INCONCLUSIVE_DETOUR_CONTROL_NOT_ARRIVED: {control:?}"
        );
        assert!(normal.arrived && normal.stopped,"INCONCLUSIVE_NORMAL_NOT_ARRIVED, inspect actual branches and guard outcomes: {normal:?}");
    }
    #[test]
    #[ignore = "serialized real-owner partial priority qualification and feedback audit"]
    fn wide_approach_real_owner_priority_audit() {
        audit_case(Case::WideApproach);
    }
    #[test]
    #[ignore = "serialized real-owner partial priority qualification and feedback audit"]
    fn short_approach_real_owner_priority_audit() {
        audit_case(Case::ShortApproach);
    }
}
