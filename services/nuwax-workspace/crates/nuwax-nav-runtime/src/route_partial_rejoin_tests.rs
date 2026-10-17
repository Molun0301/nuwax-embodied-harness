// Frozen observed poses; real PlanKernel path reused unchanged by both arms.
// This isolates production SurfaceFollower re-admission, not owner scheduling.
mod frozen_rejoin_tests {
    use super::*;
    use crate::surface_follower::{rejoin_probe, SurfaceFollower};

    fn frozen_request_pose(case: Case) -> (nuwax_nav_scan::BodyPose, f64) {
        match case {
            Case::WideApproach => (([-0.625, 0.15, 0.4], [0.0, 0.0, 0.0, 1.0]), 0.575),
            Case::ShortApproach => (
                (
                    [-0.45977023959198365, 0.15258247380354076, 0.4],
                    [0.0, 0.0, 0.04498481403765994, 0.9989876708478425],
                ),
                0.29022976040801635,
            ),
        }
    }

    fn frozen_late_pose(case: Case) -> nuwax_nav_scan::BodyPose {
        match case {
            Case::WideApproach => (
                [-0.33401961360183036, 0.15117473073791413, 0.4],
                [0.0, 0.0, 0.029995500202495466, 0.9995500337489875],
            ),
            Case::ShortApproach => (
                [-0.33607583041988814, 0.17963204671760727, 0.4],
                [0.0, 0.0, 0.09115734222015381, 0.9958365021223905],
            ),
        }
    }

    #[derive(Debug)]
    struct RejoinOutcome {
        accepted: bool,
        admission_progress: Option<f64>,
        admission_index: Option<usize>,
        first_selected: Option<rejoin_probe::Target>,
        minimum_selected_path_index: Option<usize>,
        minimum_selected_target_index: Option<usize>,
        first_turn_followed_by_translation: bool,
        rejected_before_motion: bool,
        stop_reason: String,
        first_command_wall_ms: Option<f64>,
        wall_ms: f64,
        control_wall_ms: f64,
        actual_sweep_wall_ms: f64,
        model_seconds: f64,
        final_pose: nuwax_nav_scan::BodyPose,
        final_applied: Twist2,
    }

    fn initial_turn_end(path: &Path3D) -> usize {
        let first = path.poses.first().expect("qualified nonempty candidate");
        path.poses
            .iter()
            .enumerate()
            .take_while(|(_, p)| {
                (p.pose.position.x - first.pose.position.x)
                    .hypot(p.pose.position.y - first.pose.position.y)
                    < 1e-5
            })
            .map(|(i, _)| i)
            .last()
            .unwrap()
    }

    // Only an independent geometric diagnostic of relocating the same turn.
    // This is never injected as a production gate or a controller trajectory.
    fn relocated_turn_witness(
        source: &SurfaceSnapshot,
        path: &Path3D,
        turn_end: usize,
        from: nuwax_nav_scan::BodyPose,
        boundary: &[[f64; 2]],
    ) -> Result<(), String> {
        let initial = path.poses[0].pose.orientation.yaw();
        let end = path.poses[turn_end].pose.orientation.yaw();
        let direction = ((end - initial).sin().atan2((end - initial).cos())).signum();
        let mut previous = from;
        for p in &path.poses[1..=turn_end] {
            let previous_yaw = stamped(previous, Duration::ZERO).pose.orientation.yaw();
            let d = p.pose.orientation.yaw() - previous_yaw;
            if d.sin().atan2(d.cos()) * direction <= 0.0 {
                continue;
            }
            let mut relocated = p.clone();
            relocated.pose.position = current_position(from);
            let next = body(&relocated);
            source
                .validate_body_motion(previous, next, boundary)
                .map_err(|e| format!("{e}; from={previous:?}; relocated={next:?}"))?;
            previous = next;
        }
        Ok(())
    }

    fn execute_first_turn(
        case: Case,
        late: bool,
        config: SurfaceFollowerConfig,
        source: Arc<SurfaceSnapshot>,
        boundary: &[[f64; 2]],
        goal: PoseStamped,
        options: GoalOptions,
        path: &Path3D,
        turn_end: usize,
        initial: nuwax_nav_scan::BodyPose,
    ) -> RejoinOutcome {
        let dt = 1.0 / config.controller.control_frequency;
        let period = Duration::from_secs_f64(dt);
        let limits = config.motion_limits.clone().unwrap();
        let mut controller = SurfaceFollower::new(config).unwrap();
        controller.configure_boundary(boundary);
        controller
            .update_execution_map(ExecutionMap::Surface(source.clone()))
            .unwrap();
        controller.configure_goal(&goal, options).unwrap();
        controller.update_odometry(stamped(initial, Duration::ZERO));
        controller.observe_applied_command(Twist2::ZERO);
        source
            .validate_body_motion(initial, initial, boundary)
            .unwrap();
        let began = Instant::now();
        rejoin_probe::begin();
        let admission =
            controller.replace_execution_path(ExecutionPath::Surface(path.clone()), None, goal);
        let admission_trace = rejoin_probe::end();
        let admission_progress=admission_trace.admissions.last().and_then(|a|a.progress_after);
        let admission_index=admission_trace.admissions.last().and_then(|a|a.path_index_after);
        eprintln!("FROZEN_REJOIN_ADMISSION case={case:?} late={late} result={admission:?} trace={admission_trace:?}");
        if let Err(error) = admission {
            assert!(
                matches!(error, RuntimeError::PathRejected { .. }),
                "UNQUALIFIED_ADMISSION_ERROR {error}"
            );
            return RejoinOutcome {
                accepted: false,
                admission_progress,admission_index,first_selected:None,
                minimum_selected_path_index:None,minimum_selected_target_index:None,
                first_turn_followed_by_translation: false,
                rejected_before_motion: true,
                stop_reason: format!("REJECTED_AT_ADMISSION {error}"),
                first_command_wall_ms: None,
                wall_ms: began.elapsed().as_secs_f64() * 1000.0,
                control_wall_ms: 0.0,
                actual_sweep_wall_ms: 0.0,
                model_seconds: 0.0,
                final_pose: initial,
                final_applied: Twist2::ZERO,
            };
        }
        let mut current = initial;
        let mut applied = Twist2::ZERO;
        let mut model_elapsed = Duration::ZERO;
        let mut first_command = None;
        let mut control_wall = Duration::ZERO;
        let mut sweep_wall = Duration::ZERO;
        let mut progressed = false;
        let mut first_selected=None;
        let mut minimum_selected_path_index:Option<usize>=None;
        let mut minimum_selected_target_index:Option<usize>=None;
        let mut rejected_before_motion = false;
        let mut reason = "MODEL_WATCHDOG_INCONCLUSIVE".to_owned();
        // No asynchronous owner is running here. Fixed-step feedback is an
        // isolated state-transition diagnostic; wall and model times stay distinct.
        while model_elapsed < Duration::from_secs(45) && began.elapsed() < Duration::from_secs(180)
        {
            controller.observe_applied_command(applied);
            controller.update_odometry(stamped(current, model_elapsed));
            rejoin_probe::begin();
            let control_began = Instant::now();
            let mut step = controller.step(0.32);
            if let PlannerStep::Command(command) = step {
                if let Some(rechecked) =
                    controller.revalidate_selected_command_at_current_state(command)
                {
                    step = rechecked;
                }
            }
            control_wall += control_began.elapsed();
            let frame = rejoin_probe::end();
            for target in &frame.targets {
                if first_selected.is_none() {first_selected=Some(target.clone());}
                minimum_selected_path_index=Some(minimum_selected_path_index.map_or(target.path_index,|i|i.min(target.path_index)));
                minimum_selected_target_index=Some(minimum_selected_target_index.map_or(target.target_index,|i|i.min(target.target_index)));
            }
            eprintln!("FROZEN_REJOIN_STEP case={case:?} late={late} model_s={} actual={current:?} applied={applied:?} result={step:?} trace={frame:?}",model_elapsed.as_secs_f64());
            let command = match step {
                PlannerStep::Command(command) => command,
                other => {
                    rejected_before_motion =
                        matches!(other, PlannerStep::ReplanRequired(_)) && first_command.is_none();
                    reason = format!("STOP_BEFORE_FIRST_TURN_TRANSLATION {other:?}");
                    break;
                }
            };
            if command != Twist2::ZERO && first_command.is_none() {
                first_command = Some(began.elapsed().as_secs_f64() * 1000.0);
            }
            let (next, next_applied, swept) =
                model_edge(&source, current, applied, command, &limits, boundary, dt);
            sweep_wall += swept;
            // Advance is observed, not granted: actual selected index has passed
            // the original same-XY turn and a validated model edge translates.
            progressed = frame
                .targets
                .iter()
                .any(|t| !t.rotation && t.target_index > turn_end)
                && current_position(next).distance(current_position(current)) > 0.0;
            current = next;
            applied = next_applied;
            model_elapsed += period;
            if progressed {
                reason = "FIRST_TURN_FOLLOWED_BY_VALIDATED_TRANSLATION".into();
                break;
            }
        }
        controller.stop();
        while applied != Twist2::ZERO {
            let (next, next_applied, swept) = model_edge(
                &source,
                current,
                applied,
                Twist2::ZERO,
                &limits,
                boundary,
                dt,
            );
            current = next;
            applied = next_applied;
            sweep_wall += swept;
            model_elapsed += period;
        }
        let result = RejoinOutcome {
            accepted: true,
            admission_progress,admission_index,first_selected,
            minimum_selected_path_index,minimum_selected_target_index,
            first_turn_followed_by_translation: progressed,
            rejected_before_motion,
            stop_reason: reason,
            first_command_wall_ms: first_command,
            wall_ms: began.elapsed().as_secs_f64() * 1000.0,
            control_wall_ms: control_wall.as_secs_f64() * 1000.0,
            actual_sweep_wall_ms: sweep_wall.as_secs_f64() * 1000.0,
            model_seconds: model_elapsed.as_secs_f64(),
            final_pose: current,
            final_applied: applied,
        };
        eprintln!("FROZEN_REJOIN_OUTCOME case={case:?} late={late} {result:?}");
        result
    }

    fn audit_rejoin(case: Case, index_only: bool) {
        let mut follower = prepare_case(case);
        let (requested, progress) = frozen_request_pose(case);
        let late = frozen_late_pose(case);
        // Initialize the recorded request snapshot. No pose jump occurs inside
        // either feedback arm, and no synthetic Candidate is injected.
        follower.update_odometry(stamped(requested, Duration::ZERO));
        follower.progress = progress;
        let request = snapshot_request(&follower, false);
        let cancel = crate::planning_owner::Cancellation {
            epoch: Arc::new(AtomicU64::new(1)),
            expected: 1,
        };
        let mut kernel = PlanKernel::new(follower.config.clone());
        let dispatch = tracing::Dispatch::new(probe::Subscriber);
        let (result, wall, audit) = tracing::dispatcher::with_default(&dispatch, || {
            compute_observed(&mut kernel, request, &cancel)
        });
        let candidate = result
            .outcome
            .unwrap_or_else(|errors| panic!("FROZEN_REQUEST_UNQUALIFIED {errors:?}"));
        independently_check_full_candidate(&result.request, &candidate)
            .unwrap_or_else(|error| panic!("FROZEN_REQUEST_UNQUALIFIED {error}"));
        assert!(
            audit
                .branches
                .iter()
                .any(|b| b.detour && b.reference_changed && b.partial == Some(false)),
            "FROZEN_REQUEST_UNQUALIFIED: must reproduce the actual complete detour branch"
        );
        let path = candidate.path;
        let turn_end = initial_turn_end(&path);
        assert!(
            turn_end > 0 && turn_end + 1 < path.poses.len(),
            "FROZEN_REQUEST_UNQUALIFIED: needs initial turn then translation"
        );
        let turn_yaw =
            path.poses[turn_end].pose.orientation.yaw() - path.poses[0].pose.orientation.yaw();
        assert!(
            turn_yaw.sin().atan2(turn_yaw.cos()).abs() > 1e-6,
            "FROZEN_REQUEST_UNQUALIFIED: no initial rotation intent"
        );
        let source = result.request.source.clone();
        let boundary = result.request.boundary.clone();
        let original_turn = relocated_turn_witness(&source, &path, turn_end, requested, &boundary);
        let moved_turn = relocated_turn_witness(&source, &path, turn_end, late, &boundary);
        assert!(
            original_turn.is_ok(),
            "FROZEN_REQUEST_UNQUALIFIED: original turn is not independently clear"
        );
        eprintln!("FROZEN_REJOIN_PATH case={case:?} kernel_wall_ms={} audit={audit:?} start={requested:?} late={late:?} poses={} first_turn_end={turn_end} pivot={:?} turn_target={:?} original_turn={original_turn:?} relocated_turn={moved_turn:?} first_turn_poses={:?}",
            wall.as_secs_f64()*1000.0,path.poses.len(),body(&path.poses[0]),body(&path.poses[turn_end]),
            path.poses[..=turn_end].iter().map(body).collect::<Vec<_>>());
        let goal = follower.goal.clone().unwrap();
        let options = follower.options;
        let early = execute_first_turn(
            case,
            false,
            follower.config.clone(),
            source.clone(),
            &boundary,
            goal.clone(),
            options,
            &path,
            turn_end,
            requested,
        );
        assert!(
            early.accepted && early.first_turn_followed_by_translation,
            "EARLY_POSITIVE_UNQUALIFIED {early:?}"
        );
        let delayed = execute_first_turn(
            case,
            true,
            follower.config.clone(),
            source,
            &boundary,
            goal,
            options,
            &path,
            turn_end,
            late,
        );
        if index_only {
            assert!(delayed.accepted,"INDEX_AUDIT_UNQUALIFIED_ADMISSION {delayed:?}");
            assert_eq!(early.admission_index,Some(1),"early original pivot semantics changed");
            let progress=delayed.admission_progress.expect("recorded successful admission");
            let mut before=0.0;
            let owners:Vec<_>=path.poses.windows(2).enumerate().filter_map(|(i,pair)| {
                let length=pair[0].pose.position.distance(pair[1].pose.position);
                let after=before+length;
                let owns=length>f64::EPSILON && before<progress && progress<after;
                before=after;
                owns.then_some(i+1)
            }).collect();
            assert_eq!(owners.len(),1,
                "INDEX_AUDIT_UNQUALIFIED: this frozen test needs a unique interior projection; endpoint ties have a separate regression");
            let expected=owners[0];
            assert!(expected>turn_end,"INDEX_AUDIT_UNQUALIFIED: frozen projection did not pass the old pivot");
            assert_eq!(delayed.admission_index,Some(expected),
                "PROJECTED_SEGMENT_REPLAYED: admission progress and consumed segment disagree");
            let selected=delayed.first_selected.as_ref().expect("qualified controller target observation");
            assert!(delayed.minimum_selected_path_index.is_some_and(|i|i>=expected)
                && delayed.minimum_selected_target_index.is_some_and(|i|i>=expected),
                "PREPROJECTION_PIVOT_REPLAYED: {delayed:?}");
            assert_ne!(selected.target.pose.position,path.poses[0].pose.position,
                "actual selected target must leave the discarded old pivot");
            eprintln!("PROJECTION_INDEX_VERDICT case={case:?} expected_index={expected} progress={progress} first_selected={selected:?} translated={} rejected_before_motion={} stop_reason={} no_liveness_claim",
                delayed.first_turn_followed_by_translation,delayed.rejected_before_motion,delayed.stop_reason);
            return;
        }
        if delayed.accepted
            && !delayed.first_turn_followed_by_translation
            && !delayed.rejected_before_motion
            && delayed.first_command_wall_ms.is_none()
        {
            panic!("INCONCLUSIVE_NO_FIRST_NONZERO_COMMAND: {delayed:?}; no demonstrated old-turn execution");
        }
        // Proposed progress regression, not a claim that today's advisory
        // admission already promises whole-path executability. A fix may reject
        // the stale join before motion or produce an actually executable join;
        // An explicit replan before any nonzero command is equally acceptable:
        // advisory admission need not become a new whole-route permission gate.
        // This test does not demand disabling all partial candidates.
        assert!(!delayed.accepted || delayed.rejected_before_motion || delayed.first_turn_followed_by_translation,
            "ADMITTED_REJOIN_LOSES_FIRST_TURN_EXECUTABILITY {delayed:?}; collision guard remains authoritative; no infinite-deadlock claim");
    }

    #[test]
    #[ignore = "serialized frozen actual-pose re-admission progress regression"]
    fn wide_late_rejoin_preserves_executable_first_turn() {
        audit_rejoin(Case::WideApproach, false);
    }

    #[test]
    #[ignore = "serialized frozen actual-pose re-admission progress regression"]
    fn short_late_rejoin_preserves_executable_first_turn() {
        audit_rejoin(Case::ShortApproach, false);
    }

    #[test]
    #[ignore = "serialized real-planner frozen-pose projection-index audit"]
    fn wide_late_rejoin_does_not_replay_preprojection_pivot() {
        audit_rejoin(Case::WideApproach, true);
    }

    #[test]
    #[ignore = "serialized real-planner frozen-pose projection-index audit"]
    fn short_late_rejoin_does_not_replay_preprojection_pivot() {
        audit_rejoin(Case::ShortApproach, true);
    }
}
