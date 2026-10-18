// DRAFT: include from explore_compute_backend_tests.rs after static review.
// No production candidate is included. The existing tracing observer surrounds
// the complete prepare_request body, not source.prepare_poll. No wall timing.
use crate::explore_runtime::tests::{
    capture_surface_owner_work, watermark_owner_config, watermark_owner_publication,
    watermark_owner_source,
};

// Reuse the exact strict nonzero unknown-space route from
// mission_surface_unknown_query_keeps_requested_home_yaw_and_real_endpoint.
// This is truthful unknown-space, not evidence of known-ground traversal.
fn homeward_fact_unknown_context(mut context: ComputeContext) -> ComputeContext {
    context.robot_pose.pose.position = Point3::new(-10.0, -10.0, 0.4);
    context
}

fn homeward_fact_progress(context: &ComputeContext) -> MissionProgress {
    let ComputeInput::Surface { original, config } = &context.input else { panic!("surface fixture") };
    let snapshot = original.snapshot().unwrap();
    let start = mission_pose_from_stamped(&context.robot_pose);
    assert!(snapshot.locate_support_from_body(start.position, start.orientation_xyzw,
        config.follower.body_to_support_z_m).is_err(), "explicitly unknown-space");
    let mut goal = start.clone();
    goal.position[0] += 0.5;
    let q = Quaternion::from_yaw(std::f64::consts::FRAC_PI_2);
    goal.orientation_xyzw = [q.x, q.y, q.z, q.w];
    let distance = start.position.iter().zip(goal.position).map(|(a, b)| (a - b).powi(2))
        .sum::<f64>().sqrt();
    assert!(distance > home_goal_options().position_tolerance_m + 0.2,
        "must reach PCT map lookup, not the near-zero shortcut");
    assert!(snapshot.body_envelope().is_some());
    assert_eq!(snapshot.body_envelope().unwrap().body_to_support_z_m,
        config.follower.body_to_support_z_m);
    MissionProgress::new(goal).unwrap()
}

// Called only AFTER all observed prepares in a sequence. This deliberately
// uses a separate planner: no probe warms/replaces the tested navigation PCT.
// These existing fixtures have default search limits and no local penalties.
// Backend exposes only bool, so this validates the fixture's complete reference
// path/end/length and projection, not an inaccessible backend-internal path.
fn homeward_fact_cold_oracle(context: &ComputeContext, progress: &MissionProgress) {
    let mut cold = context.clone();
    let ComputeInput::Surface { config, .. } = &mut cold.input else { unreachable!() };
    let old_pct = config.follower.pct.clone();
    config.follower.pct = Default::default();
    assert!(!Arc::ptr_eq(&config.follower.pct, &old_pct));
    let planning = cold.input.fork_for_planning();
    let routes = cold.routes(planning.as_ref()).unwrap();
    let start = mission_pose_from_stamped(&cold.robot_pose);
    let options = home_goal_options();
    let MissionRoutes::Surface { map, config, boundary } = &routes else { unreachable!() };
    let path = nuwax_nav_stack::surface_oriented::plan_reference_cancellable(map, &config.follower.pct,
        (start.position, start.orientation_xyzw), (progress.home.position, progress.home.orientation_xyzw),
        options, config.follower.motion_limits.as_ref(), boundary, 30_000, None).unwrap();
    let expected_end = path.last().unwrap();
    let expected_length: f64 = path.windows(2).map(|p|
        p[0].0.iter().zip(p[1].0).map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt()).sum();
    let route = routes.query(&start, &progress.home, options, None).unwrap();
    assert_eq!(route.length, expected_length);
    assert_eq!(route.end.position, expected_end.0);
    assert_eq!(route.end.orientation_xyzw, expected_end.1);
    assert!(route.length >= 0.4);
    let distance = route.end.position.iter().zip(progress.home.position)
        .map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
    assert!(distance <= options.position_tolerance_m);
    let [x, y, z, w] = route.end.orientation_xyzw;
    let yaw = (2.0 * (x * y + w * z)).atan2(1.0 - 2.0 * (y * y + z * z));
    assert!((yaw - std::f64::consts::FRAC_PI_2).abs() <= options.yaw_tolerance_rad);
    assert!(!cold.config.return_policy.should_return(progress.travelled_m, route.length).unwrap());
}

// All state snapshots and checks are outside the existing actual-work observer.
// Return (frontier rebuilds, frontier evaluated cells, PCT builds, PCT cells).
fn homeward_fact_prepare(
    backend: &mut ExploreComputeBackend,
    context: &ComputeContext,
    progress: &MissionProgress,
    label: &str,
) -> (usize, usize, usize, usize) {
    let history = checkpoint_payloads(&backend.controller);
    let diagnostics = backend.controller.diagnostics();
    let frontiers = backend.controller.frontier_snapshot();
    let (preview, rebuilds, cells, maps, map_cells) = capture_surface_owner_work(||
        backend.prepare_request(None, ComputeRequest::HomewardBudget(HomewardBudgetRequest {
            context: context.clone(), progress: progress.clone(),
        }), || false));
    assert!(matches!(&preview, ComputeProjection::HomewardBudget(Ok(false))), "{label}: {preview:?}");
    assert!(!backend.prepared.as_ref().unwrap().controller_prepared);
    let expected = match &context.input {
        ComputeInput::Surface { original, .. } => original,
        _ => unreachable!(),
    };
    let captured = backend.prepared.as_ref().unwrap()._original_surface.as_ref().unwrap();
    assert!(Arc::ptr_eq(captured.snapshot().unwrap(), expected.snapshot().unwrap()));
    assert_eq!(captured.notification(), expected.notification());
    assert_eq!(checkpoint_payloads(&backend.controller), history);
    let accepted = finalized(backend);
    assert!(matches!(accepted.projection, ComputeProjection::HomewardBudget(Ok(false))), "{label}");
    assert_eq!(checkpoint_payloads(&backend.controller), history);
    assert_eq!(accepted.committed.diagnostics, diagnostics);
    assert_eq!(accepted.committed.frontiers, frontiers);
    assert_eq!((rebuilds, cells), (0, 0), "route-only work must not enumerate frontiers");
    eprintln!("homeward_refresh_fact label={label} frontier_builds={rebuilds} frontier_cells={cells} pct_builds={maps} pct_cells={map_cells}");
    (rebuilds, cells, maps, map_cells)
}

#[test]
fn surface_homeward_same_context_repeats_actual_tomogram_build_baseline() {
    // Construct once. Rebuilding surface_reuse_context per round would allocate
    // another navigation PctPlanner and manufacture an unrelated cache miss.
    let mut context = homeward_fact_unknown_context(cancellable_surface_context());
    Arc::make_mut(&mut context.config).return_policy.maximum_distance_m = Some(100.0);
    let ComputeInput::Surface { original, config } = &context.input else { unreachable!() };
    let original = original.clone();
    let pct = config.follower.pct.clone();
    let explore_config = context.config.clone();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(original.snapshot().unwrap().clone()).unwrap();
    let mut backend = ExploreComputeBackend::new(surface_controller(receiver));
    let progress = homeward_fact_progress(&context);
    let mut work = Vec::new();
    for label in ["same_first", "same_second"] {
        let ComputeInput::Surface { original: current, config } = &context.input else { unreachable!() };
        assert!(Arc::ptr_eq(&context.config, &explore_config));
        assert!(Arc::ptr_eq(&config.follower.pct, &pct));
        assert!(Arc::ptr_eq(current.snapshot().unwrap(), original.snapshot().unwrap()));
        assert_eq!(current.notification(), original.notification());
        work.push(homeward_fact_prepare(&mut backend, &context, &progress, label));
    }
    homeward_fact_cold_oracle(&context, &progress);
    backend.lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled)).result.unwrap();
    assert_eq!(work.iter().map(|v| v.2).collect::<Vec<_>>(), vec![1, 1],
        "baseline fact, not a candidate-performance assertion");
    assert!(work[0].3 > 0);
    assert_eq!(work[0].3, work[1].3);
}

#[test]
fn surface_homeward_between_selections_preserves_original_source_progress_baseline() {
    let mut context = cancellable_surface_context();
    Arc::make_mut(&mut context.config).return_policy.maximum_distance_m = Some(100.0);
    let ComputeInput::Surface { original, config } = &context.input else { unreachable!() };
    let snapshot = original.snapshot().unwrap().clone();
    let pct = config.follower.pct.clone();
    let explore_config = context.config.clone();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(snapshot.clone()).unwrap();
    let mut backend = ExploreComputeBackend::new(surface_controller(receiver.clone()));
    // Only the Homeward query pose differs; source, Explore config and navigation PCT are shared.
    let homeward_context = homeward_fact_unknown_context(context.clone());
    let progress = homeward_fact_progress(&homeward_context);
    let (first, _, _, first_maps, first_cells) = capture_surface_owner_work(||
        backend.prepare_request(None, selection_request(&context), || false));
    let first_preview = selection(first).poll.unwrap();
    let first_goal = match selection(finalized(&mut backend).projection).poll.unwrap() {
        ExplorationPoll::Goal(goal) => goal,
        other => panic!("real initial Surface selection required: {other:?}"),
    };
    assert_eq!(first_preview, ExplorationPoll::Goal(first_goal.clone()));
    let query = Arc::downgrade(backend.surface_planning.as_ref().unwrap().1.query_snapshot().unwrap());
    let homeward_work = homeward_fact_prepare(&mut backend, &homeward_context, &progress, "between_selections");
    assert!(std::sync::Weak::ptr_eq(&query,
        &Arc::downgrade(backend.surface_planning.as_ref().unwrap().1.query_snapshot().unwrap())));
    // The ordinary exact input-interruption feedback does not blacklist a goal;
    // a real receiver.observe notification enables the next source selection.
    let before_feedback = checkpoint_payloads(&backend.controller);
    backend.feedback(ComputeFeedback { goal_id: first_goal.id.clone(),
        feedback: ExplorationGoalFeedback::InputUnavailable("homeward fact fixture".into()),
    }).result.unwrap();
    assert_eq!(checkpoint_payloads(&backend.controller), before_feedback);
    receiver.observe(snapshot).unwrap();
    let refreshed = receiver.freeze().unwrap();
    let ComputeInput::Surface { original, config } = &mut context.input else { unreachable!() };
    assert_eq!(refreshed.notification(), original.notification() + 1);
    assert!(original.same_source_view(&refreshed));
    *original = refreshed;
    assert!(Arc::ptr_eq(&config.follower.pct, &pct));
    assert!(Arc::ptr_eq(&context.config, &explore_config));
    let (second, _, _, second_maps, second_cells) = capture_surface_owner_work(||
        backend.prepare_request(None, selection_request(&context), || false));
    let second_preview = selection(second).poll.unwrap();
    let second_goal = match selection(finalized(&mut backend).projection).poll.unwrap() {
        ExplorationPoll::Goal(goal) => goal,
        other => panic!("new receipt must enable a real second selection: {other:?}"),
    };
    assert_eq!(second_preview, ExplorationPoll::Goal(second_goal.clone()));
    assert_ne!(first_goal.id, second_goal.id);
    let pair = &backend.surface_planning.as_ref().unwrap().1;
    assert!(std::sync::Weak::ptr_eq(&query, &Arc::downgrade(pair.query_snapshot().unwrap())));
    assert_eq!(pair.original_input().notification(), receiver.freeze().unwrap().notification());
    homeward_fact_cold_oracle(&homeward_context, &progress);
    backend.lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled)).result.unwrap();
    assert!(query.upgrade().is_none());
    assert_eq!((first_maps, homeward_work.2, second_maps), (1, 1, 0));
    assert!(first_cells > 0 && homeward_work.3 > 0);
    assert_eq!(second_cells, 0);
    eprintln!("homeward_refresh_fact interleave=selection,homeward,selection pct_builds={first_maps},{},{} pct_cells={first_cells},{},{}",
        homeward_work.2, second_maps, homeward_work.3, second_cells);
}

#[test]
fn surface_homeward_real_writer_metadata_and_scatter_build_facts_baseline() {
    // Same bounded real-Writer fixture as the existing metadata/scatter tests:
    // 50 x 30 occupied columns and exactly one two-sample measured record.
    let config = watermark_owner_config();
    let pct = config.follower.pct.clone();
    let repository = nuwax_map_chunks::Repository::temporary().unwrap();
    let mut writer = nuwax_map_chunks::Writer::new(repository, 0.1, 4 * 1024 * 1024).unwrap();
    let source = watermark_owner_source();
    let timestamp = now_unix_nanos();
    let key = [15, 15, 0];
    let block = nuwax_map_chunks::block_key(key);
    let anchor = nuwax_map_chunks::local_key(key);
    writer.observe((0..50).flat_map(|x| (0..30).map(move |y| [x, y, 0])), timestamp).unwrap();
    let layers = |spread: f64| std::collections::BTreeMap::from([(anchor,
        nuwax_map_chunks::LayerObservation {
            members: vec![anchor], plane_origin: [1.55, 1.55, 0.025], normal: [0.0, 0.0, 1.0],
            samples: [0.025 - spread, 0.025 + spread].into_iter().enumerate().map(|(index, z)|
                nuwax_map_chunks::LayerSample { source: source.clone(), point: [1.55, 1.55, z],
                    source_sequence: 1, source_timestamp_ns: timestamp, point_index: index as u32 }).collect(),
        })]);
    writer.replace_layer_observations(block, layers(0.001)).unwrap();
    writer.record_source_watermark(source.clone(), timestamp, 1).unwrap();
    let (message, sealed) = watermark_owner_publication(&mut writer, None, 1, timestamp);
    let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
    let first = cache.build(&message).unwrap();
    let receiver = SurfaceInputReceiver::new();
    receiver.observe(first.clone()).unwrap();
    let initial = receiver.freeze().unwrap();
    let mut context = homeward_fact_unknown_context(context());
    context.input = ComputeInput::Surface { original: initial.clone(), config: config.clone() };
    Arc::make_mut(&mut context.config).return_policy.maximum_distance_m = Some(100.0);
    let explore_config = context.config.clone();
    let mut source_adapter = SurfaceGoalSourceAdapter::new("surface",
        config.follower.body_to_support_z_m, receiver.clone()).unwrap();
    source_adapter.configure_constraints(None, 1, 0.3).unwrap();
    let mut controller = ExplorationController::new(ExplorationArbitrationPolicy::PreferWavefront);
    controller.add_source(source_adapter);
    controller.begin_exploration().unwrap();
    let mut backend = ExploreComputeBackend::new(controller);
    let progress = homeward_fact_progress(&context);
    let initial_context = context.clone();
    let mut work = vec![homeward_fact_prepare(&mut backend, &context, &progress, "writer_initial")];
    let original_evidence = first.observations().unwrap().get(key).unwrap();
    let original_statistics = first.observations().unwrap().ground_layers([15, 15]).next().unwrap().statistics.clone();
    let next_time = now_unix_nanos().max(timestamp + 1);
    writer.observe([key], next_time).unwrap();
    writer.record_source_watermark(source.clone(), next_time, 2).unwrap();
    let (metadata_message, metadata_sealed) = watermark_owner_publication(&mut writer,
        Some(sealed.manifest().as_ref()), 2, next_time);
    let metadata = cache.build(&metadata_message).unwrap();
    receiver.observe(metadata.clone()).unwrap();
    let metadata_input = receiver.freeze().unwrap();
    assert!(!initial.same_source_view(&metadata_input));
    assert!(initial.same_planning_query_view(&metadata_input));
    let evidence = metadata.observations().unwrap().get(key).unwrap();
    assert_eq!(evidence.observed_ns, next_time);
    assert_eq!(evidence.hits, original_evidence.hits + 1);
    assert_eq!(metadata.observations().unwrap().ground_layers([15, 15]).next().unwrap().statistics,
        original_statistics);
    let ComputeInput::Surface { original, .. } = &mut context.input else { unreachable!() };
    *original = metadata_input.clone();
    work.push(homeward_fact_prepare(&mut backend, &context, &progress, "writer_raw_equivalent"));
    let metadata_context = context.clone();
    writer.replace_layer_observations(block, layers(0.01)).unwrap();
    let (scatter_message, scatter_sealed) = watermark_owner_publication(&mut writer,
        Some(metadata_sealed.manifest().as_ref()), 3, next_time + 1);
    assert_eq!(metadata_sealed.manifest().geometry_revision, scatter_sealed.manifest().geometry_revision);
    let scatter = cache.build(&scatter_message).unwrap();
    assert_eq!(scatter.observations().unwrap().get(key), Some(evidence));
    assert_ne!(scatter.observations().unwrap().ground_layers([15, 15]).next().unwrap().statistics,
        original_statistics);
    assert!(metadata.same_geometry(&scatter));
    assert!(metadata.observations().unwrap().same_geometry(scatter.observations().unwrap()));
    receiver.observe(scatter).unwrap();
    let changed = receiver.freeze().unwrap();
    assert!(!metadata_input.same_planning_query_view(&changed));
    let ComputeInput::Surface { original, config } = &mut context.input else { unreachable!() };
    *original = changed;
    assert!(Arc::ptr_eq(&config.follower.pct, &pct));
    assert!(Arc::ptr_eq(&context.config, &explore_config));
    work.push(homeward_fact_prepare(&mut backend, &context, &progress, "writer_scatter_changed"));
    work.push(homeward_fact_prepare(&mut backend, &context, &progress, "writer_scatter_repeat"));
    for oracle_context in [&initial_context, &metadata_context, &context] {
        homeward_fact_cold_oracle(oracle_context, &progress);
    }
    backend.lifecycle(ComputeLifecycle::Stop(ExplorationStopReason::UserCancelled)).result.unwrap();
    assert_eq!(work.iter().map(|v| v.2).collect::<Vec<_>>(), vec![1, 1, 1, 1]);
    assert!(work.iter().all(|v| v.3 > 0));
    assert_eq!(first.observations().unwrap().get(key), Some(original_evidence));
}
