// Tests-only ON integration, included inside runtime lib.rs `mod tests`.
// Base supplied by parent: 6fb55ea0a55e918b0fec0e95bb5562131b743eea.
// Exact stationary acceptance, not convergence, RPC, hardware, or timing qualification.
mod zero_goal_on_integration {
    use super::*;
    use crate::route_planning::priority_probe as probe;
    use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceObservations};
    use nuwax_nav_types_messages::{GoalOptions, Point3, Pose3, Quaternion, Timestamp};
    use std::sync::atomic::{AtomicBool, Ordering};

    #[derive(Clone)]
    struct RequestEvidence {
        ticket: Option<(u64, u64)>, // original owner request ID / epoch, not a hash
        generation: u64,
        same_source: bool,
        complete_source: bool,
        pose: PoseStamped,
        original_goal: Option<PoseStamped>,
        original_route: Option<Path3D>,
        options: Option<GoalOptions>,
        reference_pending: bool,
        detour: bool,
        search_limit: usize,
    }
    struct AdmissionEvidence {
        receipt: probe::Admission,
        path: Path3D,
    }
    #[derive(Default)]
    struct Audit {
        requests: Vec<RequestEvidence>,
        admissions: Vec<AdmissionEvidence>,
        overflow: bool,
        actual_successes: usize,
        nonzero_computations: usize,
        stops: usize,
    }
    // Delegation only: no generated result, no replacement kernel, no publication gate.
    struct ObservedRoute {
        inner: RouteFollower,
        audit: Arc<Mutex<Audit>>,
        dropped: Arc<AtomicBool>,
    }
    impl NavPlanner for ObservedRoute {
        fn configure_goal(&mut self, p: &PoseStamped, o: GoalOptions) -> Result<(), RuntimeError> {
            self.inner.configure_goal(p, o)
        }
        fn configure_boundary(&mut self, b: &[[f64; 2]]) { self.inner.configure_boundary(b) }
        fn replace_path(&mut self, p: Path2D) { self.inner.replace_path(p) }
        fn replace_execution_path(&mut self, p: ExecutionPath, y: Option<Vec<f64>>, g: PoseStamped)
            -> Result<(), RuntimeError> { self.inner.replace_execution_path(p, y, g) }
        fn update_odometry(&mut self, p: PoseStamped) { self.inner.update_odometry(p) }
        fn update_costmap(&mut self, m: CostMap2D) { self.inner.update_costmap(m) }
        fn update_execution_map(&mut self, m: ExecutionMap) -> Result<(), RuntimeError> {
            self.inner.update_execution_map(m)
        }
        fn invalidate_execution_map(&mut self) { self.inner.invalidate_execution_map() }
        fn observe_applied_command(&mut self, c: Twist2) { self.inner.observe_applied_command(c) }
        fn revalidate_selected_command(&mut self, c: Twist2) -> Option<PlannerStep> {
            self.inner.revalidate_selected_command(c)
        }
        fn revalidate_selected_command_at_current_state(&mut self, c: Twist2) -> Option<PlannerStep> {
            self.inner.revalidate_selected_command_at_current_state(c)
        }
        fn take_command_dispatch_bound(&mut self) -> Option<CommandDispatchBound> {
            self.inner.take_command_dispatch_bound()
        }
        fn execution_wait_reason(&self) -> Option<String> { self.inner.execution_wait_reason() }
        fn local_path_for_display(&self) -> Vec<[f64; 3]> { self.inner.local_path_for_display() }
        fn prepare_control_step(&mut self, width: f64, continuing_wait: bool) -> Option<watch::Receiver<()>> {
            self.inner.prepare_control_step(width, continuing_wait)
        }
        fn step(&mut self, width: f64) -> PlannerStep {
            // The existing event is emitted only after actual candidate admission.
            // Capture on this real worker thread; TLS in the test thread would be wrong.
            let dispatch = tracing::Dispatch::new(probe::Subscriber);
            probe::begin_control();
            let outcome = tracing::dispatcher::with_default(&dispatch, || self.inner.step(width));
            let receipts = probe::end_control();
            let path = self.inner.local_path().cloned();
            let mut a = self.audit.lock().unwrap();
            for receipt in receipts {
                if a.admissions.len() < 4 {
                    if let Some(path) = &path {
                        a.admissions.push(AdmissionEvidence { receipt, path: path.clone() });
                    } else { a.overflow = true; }
                } else { a.overflow = true; }
            }
            a.actual_successes += usize::from(outcome == PlannerStep::Succeeded);
            a.nonzero_computations += usize::from(matches!(&outcome,
                PlannerStep::Command(c) if *c != Twist2::ZERO));
            drop(a);
            outcome
        }
        fn stop(&mut self) {
            self.inner.stop();
            self.audit.lock().unwrap().stops += 1;
        }
    }
    impl Drop for ObservedRoute {
        fn drop(&mut self) { self.dropped.store(true, Ordering::Release); }
    }
    type Scheduler = NavScheduler<FakeMotion, planner_worker::PlannerWorker>;
    // No test-owned blocked threads. Unwind signals production Stop, then the
    // original worker/owner destructors close exchanges and attempt their joins.
    // Their join Results are ignored by production Drop: no clean-exit claim.
    struct StopOnDrop(Scheduler);
    impl Drop for StopOnDrop {
        fn drop(&mut self) { self.0.planner.stop(); }
    }
    fn sampled_pose(sample_ns: u64) -> PoseStamped {
        PoseStamped {
            frame_id: nuwax_nav_types_messages::NavFrameId::Map,
            stamp: Timestamp::from_unix((sample_ns / 1_000_000_000) as i64,
                (sample_ns % 1_000_000_000) as u32),
            pose: Pose3 { position: Point3::new(0.5, 1.0, 0.5), orientation: Quaternion::IDENTITY },
        }
    }

    #[tokio::test]
    async fn exact_zero_goal_pct_scan_on_worker_scheduler_requires_stopped_fresh_pose() {
        let (config, base, keys) = crate::route_moving_worker_tests::fixture(true, 0.05, 6.0);
        assert!(config.local_planning_enabled);
        assert!(!config.controller.simulation);
        assert!(config.motion_limits.is_some());
        assert_eq!(config.max_observation_age_sec, Some(1.0));
        assert_eq!(config.scan.search_expansion_limit, nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT);
        let options = GoalOptions { position_tolerance_m: 0.0, yaw_tolerance_rad: 0.0, allow_reverse: false };
        options.validate().unwrap();
        assert!(options.requires_orientation());

        // One actual synthetic complete-floor acquisition, after expensive base
        // fixture creation. Every occupied input key is represented; no invented
        // unknown cells, fabricated hits, or restamping an old source to extend age.
        let acquired_at = Instant::now();
        let sample_ns = crate::command_dispatch::unix_ns().unwrap();
        let mut observations = SurfaceObservations::new(base.source_voxel_size_m(), 32).unwrap();
        let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
        for &key in keys.iter() {
            blocks.entry(key.map(|v| v.div_euclid(32))).or_default()
                .insert(key, SurfaceObservation { observed_ns: sample_ns, hits: 1 });
        }
        for (key, block) in blocks { observations.replace_block(key, block).unwrap(); }
        observations.mark_complete_geometry_source();
        assert!(observations.is_complete_geometry_source());
        for &key in keys.iter() {
            let observed = observations.get(key).expect("complete source retains every actual input key");
            assert_eq!(observed.observed_ns, sample_ns);
            assert_eq!(observed.hits, 1);
        }
        let map = Arc::new(base.with_observations(Some(Arc::new(observations))));
        let goal = sampled_pose(sample_ns);
        let original = crate::route_planning::body(&goal);
        map.validate_body_pose(original.0, original.1).unwrap();

        // Actual PCT producer. With raw position0 its < tolerance shortcut is
        // false, so the original tomogram route path runs. Preserve its result.
        let pct_route = config.pct.body_route(&map, original, original, options, &[], None).unwrap();
        assert_eq!(pct_route, vec![original]);
        let route = Path3D { poses: pct_route.iter().map(|&(p, q)| {
            let mut pose = goal.clone();
            pose.pose.position = Point3::new(p[0], p[1], p[2]);
            pose.pose.orientation = Quaternion { x: q[0], y: q[1], z: q[2], w: q[3] };
            pose
        }).collect() };
        // Do not duplicate the point or patch in a hand-authored successful path.
        assert_eq!(route.poses.len(), 1);
        let audit = Arc::new(Mutex::new(Audit::default()));
        let request_audit = audit.clone();
        let expected_source = map.clone();
        let follower = RouteFollower::new_with_planning_hook(config.clone(), Box::new(move |r| {
            let evidence = RequestEvidence {
                ticket: crate::planning_owner::current_request_for_test(), generation: r.generation,
                same_source: Arc::ptr_eq(&r.source, &expected_source),
                complete_source: r.source.observations().is_some_and(|o| o.is_complete_geometry_source()),
                pose: r.pose.clone(), original_goal: r.reference_source.as_ref().and_then(|s| s.goal.clone()),
                original_route: r.reference_source.as_ref().map(|s| s.route.as_ref().clone()),
                options: r.reference_source.as_ref().map(|s| s.options),
                reference_pending: r.reference.is_empty() && r.arrival.is_none() && r.final_yaw.is_none(),
                detour: r.detour, search_limit: r.scan.search_expansion_limit,
            };
            let mut a = request_audit.lock().unwrap();
            if a.requests.len() < 4 { a.requests.push(evidence); } else { a.overflow = true; }
        })).unwrap();
        let dropped = Arc::new(AtomicBool::new(false));
        let worker = planner_worker::PlannerWorker::new(ObservedRoute {
            inner: follower, audit: audit.clone(), dropped: dropped.clone(),
        }).unwrap();
        let motion = Arc::new(Mutex::new(FakeMotionState::default()));
        let scheduler = NavScheduler::new(FakeMotion(motion.clone()), worker, SchedulerConfig::default()).unwrap();
        let mut guard = StopOnDrop(scheduler);
        let scheduler = &mut guard.0;
        let (status, _) = watch::channel(scheduler.status.clone());
        let pose_deadline = acquired_at + Duration::from_secs(1);
        scheduler.observe_odometry_at(&status, goal.clone(), Some(pose_deadline), acquired_at).unwrap();
        scheduler.observe_costmap(&status, CostmapObservation {
            costmap: ExecutionMap::Surface(map.clone()), source_timestamp_ns: sample_ns as i64,
        }, Instant::now()).unwrap();
        let source_bound = CommandDispatchBound::from_source_timestamp(sample_ns, Duration::from_secs(1));
        assert!(!source_bound.expired_now(), "fixture/PCT preparation used the original source lifetime");
        // Enter through the production source-input handler, not field assignment.
        let (reply, response) = oneshot::channel();
        scheduler.handle_input(&status, InputCommand {
            kind: InputKind::UpdateObstacleSource(None, true, Some(source_bound)), reply,
        }).await;
        response.await.unwrap().unwrap();
        let mut request = session();
        request.goal = ExecutionGoal::Body(goal.clone()); request.options = options;
        scheduler.start_session(&status, request.clone()).await.unwrap();
        let mut versioned = path(); versioned.path = ExecutionPath::Surface(route.clone());
        scheduler.submit_path(&status, versioned).await.unwrap();
        let accepted_identity = scheduler.current_execution_identity.clone().unwrap();
        assert!(scheduler.lease_active);
        assert_eq!(scheduler.status.guard_generation, request.motion_admission.guard_generation);
        assert!(scheduler.stale_input_reason(Instant::now()).is_none());

        let deadline = pose_deadline.min(Instant::from_std(source_bound.monotonic_deadline().unwrap()));
        let mut next_renew = Instant::now() + scheduler.config.renew_interval;
        while scheduler.arrival_confirmation.is_none() && scheduler.status.last_terminal.is_none() {
            assert!(Instant::now() < deadline, "no complete ON arrival within original input lifetime");
            if Instant::now() >= next_renew && scheduler.should_renew() {
                scheduler.renew().await.unwrap(); next_renew += scheduler.config.renew_interval;
            }
            scheduler.session_tick(&status).await.unwrap();
            assert!(scheduler.status.last_applied_twist.is_none_or(|c| c == Twist2::ZERO));
            if scheduler.arrival_confirmation.is_none() && scheduler.status.last_terminal.is_none() {
                tokio::time::sleep(DEFAULT_TICK_INTERVAL).await;
            }
        }
        assert!(scheduler.status.last_terminal.is_none(), "failure/expiry is not qualified arrival");
        let pending = scheduler.arrival_confirmation.as_ref().expect("real worker arrival, then scheduler fence");
        validate_fence(&pending.proof).unwrap();
        assert_eq!(pending.proof.zero_ack_id, "zero-ack");
        assert_eq!(pending.proof.applied_monotonic_ns, 100);
        assert_eq!(pending.proof.lease, FenceLeaseState::Idle);
        assert!(Arc::ptr_eq(pending.execution_identity.as_ref().unwrap(), &accepted_identity),
            "stop proof keeps the originally admitted path execution identity");
        let confirmed_at = pending.confirmed_at;
        assert!(!scheduler.lease_active);
        assert_eq!(scheduler.session.as_ref(), Some(&request));
        assert!(scheduler.pose_acquired_at.unwrap() < confirmed_at);
        scheduler.session_tick(&status).await.unwrap();
        assert!(scheduler.status.last_terminal.is_none(), "pre-stop sample must never finalize success");

        // The modeled plant has remained stationary. Sample it anew NOW, after
        // the actual scheduler stop proof; do not increment an old stamp by hand.
        let after_acquired = Instant::now();
        let after_ns = crate::command_dispatch::unix_ns().unwrap();
        let after = sampled_pose(after_ns);
        assert!(after_acquired > confirmed_at);
        assert!(after_ns > sample_ns);
        assert_eq!(after.pose, goal.pose);
        scheduler.observe_odometry_at(&status, after, Some(after_acquired + Duration::from_secs(1)), after_acquired).unwrap();
        assert!(!source_bound.expired_now(), "original source must still be live at final confirmation");
        scheduler.session_tick(&status).await.unwrap();
        assert_eq!(scheduler.status.last_terminal, Some(TerminalOutcome::Succeeded));
        assert_eq!(scheduler.status.last_applied_twist, Some(Twist2::ZERO));
        assert_eq!(scheduler.status.last_zero_ack_id.as_deref(), Some("zero-ack"));
        assert!(scheduler.session.is_none());
        assert!(!scheduler.lease_active);
        drop(guard); // production close/join; no publication gate can block cleanup
        assert!(dropped.load(Ordering::Acquire));

        let calls = motion.lock().unwrap().calls.clone();
        assert_eq!(calls.iter().filter(|c| c.starts_with("acquire:")).count(), 1);
        assert_eq!(calls.iter().filter(|c| c.starts_with("fence:")).count(), 1);
        let a = audit.lock().unwrap();
        assert!(!a.overflow);
        assert_eq!(a.requests.len(), 1);
        assert_eq!(a.admissions.len(), 1);
        assert!(a.actual_successes > 0);
        assert_eq!(a.nonzero_computations, 0);
        let r = &a.requests[0]; let c = &a.admissions[0];
        assert!(r.same_source && r.complete_source && r.reference_pending);
        assert!(!r.detour, "original PCT reference is not a forced detour recomputation");
        assert_eq!(r.search_limit, nuwax_nav_scan::DEFAULT_SEARCH_EXPANSION_LIMIT);
        assert_eq!(r.pose.pose, goal.pose);
        assert_eq!(r.original_goal.as_ref().unwrap().pose, goal.pose);
        assert_eq!(r.options.unwrap(), options);
        assert_eq!(r.original_route.as_ref().unwrap(), &route);
        let (request_id, epoch) = r.ticket.expect("actual planning-owner request identity");
        assert!(epoch > 0 && r.generation > 0);
        assert_eq!(c.receipt.request_id, request_id);
        assert!(!c.receipt.partial, "actual admitted candidate must be full");
        assert_eq!(c.receipt.covered_end, 0.0);
        assert_eq!(c.receipt.requested_end, 0.0);
        assert_eq!(c.path.poses.len(), 1);
        assert_eq!(crate::route_planning::body(&c.path.poses[0]), original);
        map.validate_body_pose(original.0, original.1).unwrap();
        eprintln!("ZERO_GOAL_ON_QUALIFIED request_id={request_id} epoch={epoch} generation={} pct_points=1 admitted_points=1 partial=false raw_position=0 raw_yaw=0 nonzero=0 fence_count=1 fresh_after_stop=true worker_cleanup_returned=true", r.generation);
    }
}
