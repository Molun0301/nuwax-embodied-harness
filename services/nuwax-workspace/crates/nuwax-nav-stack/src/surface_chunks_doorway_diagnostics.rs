// Captured doorway diagnostics included in the parent test module.
// Diagnostic replay only: success means the original rejection was reproduced.
// This is deliberately not a desired-behavior/old-fail-new-pass regression yet.
// Uses the full original source and normal public follower APIs; never replans.
// Run with NEH_DOOR_REPLAY_PATH pointing at an immutable captured JSON file.
#[test]
#[ignore = "diagnostic replay: requires the captured dense doorway input trace"]
fn captured_dense_doorway_reproduces_exact_connector_rejection() {
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceMapBuilder};
    use nuwax_nav_runtime::{
        ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, SurfaceFollower,
        SurfaceFollowerConfig,
    };
    use nuwax_nav_types_messages::{
        GoalOptions, MotionLimits, MotionModel, NavFrameId, Path3D, Point2, Point3,
        Pose3, PoseStamped, Quaternion, Timestamp, Twist2,
    };

    type Body = ([f64; 3], [f64; 4]);
    type BodyBits = ([u64; 3], [u64; 4]);
    type InputBits = (BodyBits, [u64; 3], i64);
    #[derive(serde::Deserialize)]
    struct Recipe {
        gap: i32,
        revision: u64,
        fine_resolution_bits: u32,
        keys_count: usize,
        config: serde_json::Value,
        config_float_bits: serde_json::Value,
        footprint_xy_bits: Vec<[u64; 2]>,
        body_scalar_bits: [u64; 7],
    }
    #[derive(serde::Deserialize)]
    struct Trace {
        schema: String,
        heading: f64,
        aligned: bool,
        tick: usize,
        time_ns: i64,
        outcome: String,
        simulation: bool,
        control_frequency: f64,
        speed: f64,
        allow_reverse: bool,
        position_tolerance_m: f64,
        yaw_tolerance_rad: f64,
        source_recipe: Recipe,
        route_bits: Vec<BodyBits>,
        input_bits: Vec<InputBits>,
        current_bits: BodyBits,
        goal_bits: BodyBits,
        applied_bits: [u64; 3],
    }
    let total_started = Instant::now();
    let path = std::env::var("NEH_DOOR_REPLAY_PATH").unwrap_or_else(|_| {
        "/workspace/shared/neh-validation/doorway/primary-failure-inputs.json".into()
    });
    let bytes = std::fs::read(&path).expect("capture must exist; never substitute a synthetic trace");
    let trace: Trace = serde_json::from_slice(&bytes).expect("valid bit-exact input trace");
    assert_eq!(trace.schema, "dense-doorway-input-trace-v1");
    assert_eq!(trace.input_bits.len(), trace.tick + 1, "trace must start at tick zero");
    assert!(trace.route_bits.len() >= 2);
    assert!(trace.outcome.starts_with("ReplanRequired("), "wrong captured outcome");
    assert!(trace.outcome.contains("无法从实际站姿按前进圆弧或转向接续"),
        "fixture must reproduce the selected-target connector rejection");
    for field in ["initial_path_index=", "initial_alignment_index=", "initial_progress=",
        "selected_index=", "alignment_index=", "path_len=", "target=", "previous=", "applied="] {
        assert!(trace.outcome.contains(field), "capture is missing context: {field}");
    }
    let decode_body = |b: BodyBits| (b.0.map(f64::from_bits), b.1.map(f64::from_bits));
    let body_bits = |p: Body| (p.0.map(f64::to_bits), p.1.map(f64::to_bits));
    let twist = |bits: [u64; 3]| {
        let a = bits.map(f64::from_bits);
        Twist2 { linear: Point2::new(a[0], a[1]), angular_z: a[2] }
    };
    let twist_bits = |a: Twist2| [a.linear.x, a.linear.y, a.angular_z].map(f64::to_bits);
    let stamped = |body: Body, time: i64| PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(time / 1_000_000_000, (time % 1_000_000_000) as u32),
        pose: Pose3 {
            position: Point3::new(body.0[0], body.0[1], body.0[2]),
            orientation: Quaternion { x: body.1[0], y: body.1[1], z: body.1[2], w: body.1[3] },
        },
    };
    let first = *trace.input_bits.first().unwrap();
    let last = *trace.input_bits.last().unwrap();
    assert_eq!(first.2, 1_000_000_000);
    assert_eq!(first.1, twist_bits(Twist2::ZERO));
    assert_eq!(last.0, trace.current_bits);
    assert_eq!(last.1, trace.applied_bits);
    assert_eq!(last.2, trace.time_ns);
    for (tick, input) in trace.input_bits.iter().enumerate() {
        assert_eq!(input.2, first.2 + tick as i64 * 50_000_000, "timestamp at tick {tick}");
        let body = decode_body(input.0);
        assert!(body.0.into_iter().chain(body.1).all(f64::is_finite));
        assert!(input.1.map(f64::from_bits).into_iter().all(f64::is_finite));
    }

    // The recipe is the complete original revision-1 source, not a cropped map.
    let recipe = &trace.source_recipe;
    assert_eq!((recipe.revision, recipe.gap), (1, 33));
    assert_eq!(recipe.fine_resolution_bits, 0.01_f32.to_bits());
    let fine = f32::from_bits(recipe.fine_resolution_bits);
    let field = |name: &str| {
        let bits = recipe.config_float_bits[name].as_u64().expect("config f32 bit field");
        f32::from_bits(u32::try_from(bits).expect("config bits fit u32"))
    };
    // Also pin the original recipe; an unrelated capture cannot silently qualify.
    let original_config = [
        ("voxel_size", 0.1_f32), ("robot_height", 0.1),
        ("surface_closing_radius", 0.0), ("node_spacing_m", 0.3),
        ("wall_clearance_m", 0.0), ("wall_buffer_m", 0.4),
        ("wall_buffer_weight", 2.0), ("max_slope_deg", 18.0),
        ("step_threshold_m", 0.0), ("min_step_tread_depth_m", 0.2),
        ("step_penalty_weight", 2.0), ("goal_tolerance", 0.1),
        ("viz_publish_hz", 0.0),
    ];
    for (name, expected) in original_config {
        assert_eq!(field(name).to_bits(), expected.to_bits(), "original config {name}");
    }
    assert_eq!(recipe.config["world_frame"].as_str(), Some("map"));
    let config = Config {
        world_frame: recipe.config["world_frame"].as_str().unwrap().to_owned(),
        voxel_size: field("voxel_size"), robot_height: field("robot_height"),
        surface_closing_radius: field("surface_closing_radius"),
        node_spacing_m: field("node_spacing_m"), wall_clearance_m: field("wall_clearance_m"),
        wall_buffer_m: field("wall_buffer_m"), wall_buffer_weight: field("wall_buffer_weight"),
        max_slope_deg: field("max_slope_deg"), step_threshold_m: field("step_threshold_m"),
        min_step_tread_depth_m: field("min_step_tread_depth_m"),
        step_penalty_weight: field("step_penalty_weight"),
        goal_tolerance: field("goal_tolerance"), viz_publish_hz: field("viz_publish_hz"),
    };
    let width = 32.0 * f64::from(fine);
    let original_footprint: Vec<[f64; 2]> = vec![
        [-0.3, -width / 2.0], [0.3, -width / 2.0],
        [0.3, width / 2.0], [-0.3, width / 2.0],
    ];
    assert_eq!(recipe.footprint_xy_bits,
        original_footprint.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>());
    assert_eq!(recipe.body_scalar_bits,
        [-0.15_f64, 0.15, 0.4, 0.0, 0.0, 18.0, 0.0].map(f64::to_bits));
    let scalar = recipe.body_scalar_bits.map(f64::from_bits);
    let envelope = SurfaceBodyEnvelope {
        footprint_xy_m: recipe.footprint_xy_bits.iter().map(|p| p.map(f64::from_bits)).collect(),
        z_bounds_m: [scalar[0], scalar[1]], body_to_support_z_m: scalar[2],
        max_step_up_m: scalar[3], max_step_down_m: scalar[4],
        max_cross_slope_deg: scalar[5], support_uncertainty_m: scalar[6],
    };
    let build_started = Instant::now();
    let repository = Repository::temporary().unwrap();
    let mut reader = reader(repository.clone());
    let gap = recipe.gap;
    let mut keys: BTreeSet<Key> = (-100..100)
        .flat_map(|x| (-50..100).map(move |y| [x, y, -1])).collect();
    keys.extend((0..4).flat_map(|x| (-50..100).filter(move |&y| y < 0 || y >= gap)
        .flat_map(move |y| (0..80).map(move |z| [x, y, z]))));
    assert_eq!(keys.len(), recipe.keys_count);
    assert_eq!(keys.len(), 30_000 + 4 * (150 - gap as usize) * 80);
    let root = install(&repository, &keys, fine, recipe.revision);
    let block_count = keys.iter().map(|&key| block_key(key)).collect::<BTreeSet<_>>().len();
    assert_eq!(root.blocks.len(), block_count);
    assert_eq!(root.blocks.iter().map(|b| b.points as usize).sum::<usize>(), recipe.keys_count);
    assert_eq!(root.voxel_size_m.to_bits(), recipe.fine_resolution_bits);
    let observations = reader.observations(&root, None).unwrap();
    for &key in &keys {
        let evidence = observations.get(key).expect("every original fine source key is retained");
        assert_eq!((evidence.observed_ns, evidence.hits), (recipe.revision, 1));
    }
    let (updates, count) = reader.columns(&root, None, config.voxel_size).unwrap();
    assert_eq!(count, recipe.keys_count);
    assert_columns(&updates, &keys, fine, config.voxel_size);
    let mut builder = SurfaceMapBuilder::configured(config, Some(envelope));
    let built = builder.build_column_updates(&updates, fine, true, count).unwrap();
    assert_eq!(built.stats.input_points, recipe.keys_count);
    let build_stats = built.stats;
    let map = Arc::new(built.snapshot.with_observations(Some(Arc::clone(&observations))));
    assert_eq!(map.source_voxel_size_m().to_bits(), recipe.fine_resolution_bits);
    assert!(Arc::ptr_eq(map.observations().unwrap(), &observations));
    let cfg = map.config();
    for (name, value) in [
        ("voxel_size", cfg.voxel_size), ("robot_height", cfg.robot_height),
        ("surface_closing_radius", cfg.surface_closing_radius), ("node_spacing_m", cfg.node_spacing_m),
        ("wall_clearance_m", cfg.wall_clearance_m), ("wall_buffer_m", cfg.wall_buffer_m),
        ("wall_buffer_weight", cfg.wall_buffer_weight), ("max_slope_deg", cfg.max_slope_deg),
        ("step_threshold_m", cfg.step_threshold_m), ("min_step_tread_depth_m", cfg.min_step_tread_depth_m),
        ("step_penalty_weight", cfg.step_penalty_weight), ("goal_tolerance", cfg.goal_tolerance),
        ("viz_publish_hz", cfg.viz_publish_hz),
    ] {
        assert_eq!(value.to_bits(), field(name).to_bits(), "rebuilt map config {name}");
    }
    let rebuilt_body = map.body_envelope().expect("original full body envelope");
    assert_eq!(rebuilt_body.footprint_xy_m.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>(),
        recipe.footprint_xy_bits);
    assert_eq!([rebuilt_body.z_bounds_m[0], rebuilt_body.z_bounds_m[1],
        rebuilt_body.body_to_support_z_m, rebuilt_body.max_step_up_m, rebuilt_body.max_step_down_m,
        rebuilt_body.max_cross_slope_deg, rebuilt_body.support_uncertainty_m].map(f64::to_bits),
        recipe.body_scalar_bits);
    let build_elapsed = build_started.elapsed();

    // Identical original control parameters. Captured decimal metadata is only
    // checked against these literals; it is not used to reconstruct geometry.
    assert!(trace.simulation && trace.allow_reverse);
    assert_eq!(trace.control_frequency.to_bits(), 20.0_f64.to_bits());
    assert_eq!(trace.speed.to_bits(), 0.3_f64.to_bits());
    assert_eq!(trace.position_tolerance_m.to_bits(), 0.03_f64.to_bits());
    assert_eq!(trace.yaw_tolerance_rad.to_bits(), 0.05_f64.to_bits());
    let motion = MotionLimits {
        model: MotionModel::Omnidirectional, max_forward_mps: 0.3,
        max_reverse_mps: 0.3, max_lateral_mps: 0.3, max_angular_rps: 0.8,
        linear_acceleration_mps2: 1.0, guaranteed_braking_mps2: 1.0,
        angular_acceleration_rps2: 2.0, guaranteed_angular_braking_rps2: 2.0,
        reaction_time_sec: 0.05, minimum_turning_radius_m: 0.0, can_rotate_in_place: true,
    };
    let options = GoalOptions { allow_reverse: true, position_tolerance_m: 0.03, yaw_tolerance_rad: 0.05 };
    let mut config = SurfaceFollowerConfig::new(0.4);
    config.controller.speed = 0.3;
    config.controller.control_frequency = 20.0;
    config.controller.simulation = true;
    config.motion_limits = Some(motion);
    config.max_angular_speed_rps = 0.8;
    let mut follower = SurfaceFollower::new(config).unwrap();
    let goal = decode_body(trace.goal_bits);
    let route: Vec<Body> = trace.route_bits.iter().copied().map(decode_body).collect();
    follower.configure_goal(&stamped(goal, first.2), options).unwrap();
    follower.update_execution_map(ExecutionMap::Surface(Arc::clone(&map))).unwrap();
    follower.update_odometry(stamped(decode_body(first.0), first.2));
    follower.replace_execution_path(ExecutionPath::Surface(Path3D {
        poses: route.iter().copied().map(|p| stamped(p, first.2)).collect(),
    }), None, stamped(goal, first.2)).unwrap();

    let replay_started = Instant::now();
    for (tick, &(pose_bits, applied_bits, time)) in trace.input_bits.iter().enumerate() {
        let current = decode_body(pose_bits);
        let applied = twist(applied_bits);
        // Tick zero has already been initialized, exactly as in the original.
        // Later ticks replay public feedback in the original order; no internal
        // path-index/alignment/progress state is patched or fabricated.
        if tick != 0 {
            follower.observe_applied_command(applied);
            follower.update_odometry(stamped(current, time));
        }
        let step_started = Instant::now();
        let outcome = follower.step(0.32);
        let step_elapsed = step_started.elapsed();
        if tick == trace.tick {
            let observed = format!("{outcome:?}");
            eprintln!("DOOR_REPLAY path={path} tick={tick} heading={:?} aligned={} route_len={} \
                source_keys={} source_blocks={} source_bits={} build={build_elapsed:?} \
                replay={:?} final_step={step_elapsed:?} total={:?} stats={build_stats:?}\n\
                current_bits={pose_bits:?} applied_bits={applied_bits:?}\n\
                expected={}\nobserved={observed}", trace.heading, trace.aligned, route.len(),
                keys.len(), root.blocks.len(), map.source_voxel_size_m().to_bits(),
                replay_started.elapsed(), total_started.elapsed(), trace.outcome);
            // Includes initial path/alignment/progress, final target selection,
            // previous/target full poses and actual applied command context.
            assert_eq!(observed, trace.outcome, "the full original failure context must reproduce");

            // Classify only AFTER replay, so extra queries cannot warm its caches.
            // A valid stance/support does not prove a kinematically valid connector.
            let selected: usize = trace.outcome.split("selected_index=").nth(1).unwrap()
                .split_whitespace().next().unwrap().parse().unwrap();
            assert!(selected < route.len());
            let target = route[selected];
            let current_view = map.with_current_body_support(current.0, current.1);
            let view = current_view.as_ref().unwrap_or(map.as_ref());
            let current_pose = view.validate_body_pose(current.0, current.1);
            let current_support = view.locate_support_from_body(current.0, current.1, 0.4);
            let target_pose = view.validate_body_pose(target.0, target.1);
            let target_support = view.locate_support_from_body(target.0, target.1, 0.4);
            let chord = view.validate_body_motion(current, target, &[]);
            eprintln!("DOOR_REPLAY_GEOMETRY selected_index={selected} target_bits={:?} \
                current_view_error={:?} current_pose={current_pose:?} current_support={current_support:?} \
                target_pose={target_pose:?} target_support={target_support:?} straight_chord={chord:?}; \
                connector_rejection_is_the_exact_follower_outcome_above",
                body_bits(target), current_view.as_ref().err());
            assert!(current_pose.is_ok(), "recorded connector failure must not hide current-body rejection");
            return;
        }
        let command = match outcome {
            PlannerStep::Command(command) => command,
            PlannerStep::Succeeded => Twist2::ZERO,
            other => panic!("earlier-than-capture divergence at tick {tick}: {other:?}"),
        };
        assert!(command.linear.x >= 0.0 && command.linear.y == 0.0);
        // The next recorded input is authoritative. Independently check that
        // the replayed command would have produced it under the original plant.
        let next_applied = Twist2 {
            linear: Point2::new(applied.linear.x + (command.linear.x - applied.linear.x).clamp(-0.05, 0.05), 0.0),
            angular_z: applied.angular_z + (command.angular_z - applied.angular_z).clamp(-0.1, 0.1),
        };
        let mean = Twist2 {
            linear: Point2::new((applied.linear.x + next_applied.linear.x) * 0.5, 0.0),
            angular_z: (applied.angular_z + next_applied.angular_z) * 0.5,
        };
        let yaw = 2.0 * current.1[2].atan2(current.1[3]);
        let next = nuwax_nav_control::integrate(nuwax_nav_control::PlanarPose {
            x: current.0[0], y: current.0[1], yaw,
        }, mean, 0.05);
        let actual = ([next.x, next.y, 0.4], [0.0, 0.0, (next.yaw / 2.0).sin(), (next.yaw / 2.0).cos()]);
        let recorded_next = trace.input_bits[tick + 1];
        assert_eq!(twist_bits(next_applied), recorded_next.1, "applied feedback diverged after tick {tick}");
        assert_eq!(body_bits(actual), recorded_next.0, "plant pose diverged after tick {tick}");
        // Keep the original full-body check of every actually integrated edge.
        map.validate_body_motion(current, decode_body(recorded_next.0), &[])
            .unwrap_or_else(|e| panic!("captured actual motion invalid after tick {tick}: {e:?}"));
    }
    unreachable!("the final recorded input must return its exact failure");
}

// Captured doorway diagnostics included in the parent test module.
// Static probes only. The original full replay has already matched exactly.
// This is deliberately not a desired-behavior/old-fail-new-pass regression yet.
// Uses the full original source and normal public follower APIs; never replans.
// Run with NEH_DOOR_REPLAY_PATH pointing at an immutable captured JSON file.
#[test]
#[ignore = "diagnostic replay: requires the captured dense doorway input trace"]
fn captured_dense_doorway_static_connector_probes() {
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceMapBuilder};
    use nuwax_nav_runtime::{
        ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, SurfaceFollower,
        SurfaceFollowerConfig,
    };
    use nuwax_nav_types_messages::{
        GoalOptions, MotionLimits, MotionModel, NavFrameId, Path3D, Point2, Point3,
        Pose3, PoseStamped, Quaternion, Timestamp, Twist2,
    };

    type Body = ([f64; 3], [f64; 4]);
    type BodyBits = ([u64; 3], [u64; 4]);
    type InputBits = (BodyBits, [u64; 3], i64);
    #[derive(serde::Deserialize)]
    struct Recipe {
        gap: i32,
        revision: u64,
        fine_resolution_bits: u32,
        keys_count: usize,
        config: serde_json::Value,
        config_float_bits: serde_json::Value,
        footprint_xy_bits: Vec<[u64; 2]>,
        body_scalar_bits: [u64; 7],
    }
    #[derive(serde::Deserialize)]
    struct Trace {
        schema: String,
        heading: f64,
        aligned: bool,
        tick: usize,
        time_ns: i64,
        outcome: String,
        simulation: bool,
        control_frequency: f64,
        speed: f64,
        allow_reverse: bool,
        position_tolerance_m: f64,
        yaw_tolerance_rad: f64,
        source_recipe: Recipe,
        route_bits: Vec<BodyBits>,
        input_bits: Vec<InputBits>,
        current_bits: BodyBits,
        goal_bits: BodyBits,
        applied_bits: [u64; 3],
    }
    let total_started = Instant::now();
    let path = std::env::var("NEH_DOOR_REPLAY_PATH").unwrap_or_else(|_| {
        "/workspace/shared/neh-validation/doorway/primary-failure-inputs.json".into()
    });
    let bytes = std::fs::read(&path).expect("capture must exist; never substitute a synthetic trace");
    let trace: Trace = serde_json::from_slice(&bytes).expect("valid bit-exact input trace");
    assert_eq!(trace.schema, "dense-doorway-input-trace-v1");
    assert_eq!(trace.input_bits.len(), trace.tick + 1, "trace must start at tick zero");
    assert!(trace.route_bits.len() >= 2);
    assert!(trace.outcome.starts_with("ReplanRequired("), "wrong captured outcome");
    assert!(trace.outcome.contains("无法从实际站姿按前进圆弧或转向接续"),
        "fixture must reproduce the selected-target connector rejection");
    for field in ["initial_path_index=", "initial_alignment_index=", "initial_progress=",
        "selected_index=", "alignment_index=", "path_len=", "target=", "previous=", "applied="] {
        assert!(trace.outcome.contains(field), "capture is missing context: {field}");
    }
    let decode_body = |b: BodyBits| (b.0.map(f64::from_bits), b.1.map(f64::from_bits));
    let body_bits = |p: Body| (p.0.map(f64::to_bits), p.1.map(f64::to_bits));
    let twist = |bits: [u64; 3]| {
        let a = bits.map(f64::from_bits);
        Twist2 { linear: Point2::new(a[0], a[1]), angular_z: a[2] }
    };
    let twist_bits = |a: Twist2| [a.linear.x, a.linear.y, a.angular_z].map(f64::to_bits);
    let stamped = |body: Body, time: i64| PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(time / 1_000_000_000, (time % 1_000_000_000) as u32),
        pose: Pose3 {
            position: Point3::new(body.0[0], body.0[1], body.0[2]),
            orientation: Quaternion { x: body.1[0], y: body.1[1], z: body.1[2], w: body.1[3] },
        },
    };
    let first = *trace.input_bits.first().unwrap();
    let last = *trace.input_bits.last().unwrap();
    assert_eq!(first.2, 1_000_000_000);
    assert_eq!(first.1, twist_bits(Twist2::ZERO));
    assert_eq!(last.0, trace.current_bits);
    assert_eq!(last.1, trace.applied_bits);
    assert_eq!(last.2, trace.time_ns);
    for (tick, input) in trace.input_bits.iter().enumerate() {
        assert_eq!(input.2, first.2 + tick as i64 * 50_000_000, "timestamp at tick {tick}");
        let body = decode_body(input.0);
        assert!(body.0.into_iter().chain(body.1).all(f64::is_finite));
        assert!(input.1.map(f64::from_bits).into_iter().all(f64::is_finite));
    }

    // The recipe is the complete original revision-1 source, not a cropped map.
    let recipe = &trace.source_recipe;
    assert_eq!((recipe.revision, recipe.gap), (1, 33));
    assert_eq!(recipe.fine_resolution_bits, 0.01_f32.to_bits());
    let fine = f32::from_bits(recipe.fine_resolution_bits);
    let field = |name: &str| {
        let bits = recipe.config_float_bits[name].as_u64().expect("config f32 bit field");
        f32::from_bits(u32::try_from(bits).expect("config bits fit u32"))
    };
    // Also pin the original recipe; an unrelated capture cannot silently qualify.
    let original_config = [
        ("voxel_size", 0.1_f32), ("robot_height", 0.1),
        ("surface_closing_radius", 0.0), ("node_spacing_m", 0.3),
        ("wall_clearance_m", 0.0), ("wall_buffer_m", 0.4),
        ("wall_buffer_weight", 2.0), ("max_slope_deg", 18.0),
        ("step_threshold_m", 0.0), ("min_step_tread_depth_m", 0.2),
        ("step_penalty_weight", 2.0), ("goal_tolerance", 0.1),
        ("viz_publish_hz", 0.0),
    ];
    for (name, expected) in original_config {
        assert_eq!(field(name).to_bits(), expected.to_bits(), "original config {name}");
    }
    assert_eq!(recipe.config["world_frame"].as_str(), Some("map"));
    let config = Config {
        world_frame: recipe.config["world_frame"].as_str().unwrap().to_owned(),
        voxel_size: field("voxel_size"), robot_height: field("robot_height"),
        surface_closing_radius: field("surface_closing_radius"),
        node_spacing_m: field("node_spacing_m"), wall_clearance_m: field("wall_clearance_m"),
        wall_buffer_m: field("wall_buffer_m"), wall_buffer_weight: field("wall_buffer_weight"),
        max_slope_deg: field("max_slope_deg"), step_threshold_m: field("step_threshold_m"),
        min_step_tread_depth_m: field("min_step_tread_depth_m"),
        step_penalty_weight: field("step_penalty_weight"),
        goal_tolerance: field("goal_tolerance"), viz_publish_hz: field("viz_publish_hz"),
    };
    let width = 32.0 * f64::from(fine);
    let original_footprint: Vec<[f64; 2]> = vec![
        [-0.3, -width / 2.0], [0.3, -width / 2.0],
        [0.3, width / 2.0], [-0.3, width / 2.0],
    ];
    assert_eq!(recipe.footprint_xy_bits,
        original_footprint.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>());
    assert_eq!(recipe.body_scalar_bits,
        [-0.15_f64, 0.15, 0.4, 0.0, 0.0, 18.0, 0.0].map(f64::to_bits));
    let scalar = recipe.body_scalar_bits.map(f64::from_bits);
    let envelope = SurfaceBodyEnvelope {
        footprint_xy_m: recipe.footprint_xy_bits.iter().map(|p| p.map(f64::from_bits)).collect(),
        z_bounds_m: [scalar[0], scalar[1]], body_to_support_z_m: scalar[2],
        max_step_up_m: scalar[3], max_step_down_m: scalar[4],
        max_cross_slope_deg: scalar[5], support_uncertainty_m: scalar[6],
    };
    let build_started = Instant::now();
    let repository = Repository::temporary().unwrap();
    let mut reader = reader(repository.clone());
    let gap = recipe.gap;
    let mut keys: BTreeSet<Key> = (-100..100)
        .flat_map(|x| (-50..100).map(move |y| [x, y, -1])).collect();
    keys.extend((0..4).flat_map(|x| (-50..100).filter(move |&y| y < 0 || y >= gap)
        .flat_map(move |y| (0..80).map(move |z| [x, y, z]))));
    assert_eq!(keys.len(), recipe.keys_count);
    assert_eq!(keys.len(), 30_000 + 4 * (150 - gap as usize) * 80);
    let root = install(&repository, &keys, fine, recipe.revision);
    let block_count = keys.iter().map(|&key| block_key(key)).collect::<BTreeSet<_>>().len();
    assert_eq!(root.blocks.len(), block_count);
    assert_eq!(root.blocks.iter().map(|b| b.points as usize).sum::<usize>(), recipe.keys_count);
    assert_eq!(root.voxel_size_m.to_bits(), recipe.fine_resolution_bits);
    let observations = reader.observations(&root, None).unwrap();
    for &key in &keys {
        let evidence = observations.get(key).expect("every original fine source key is retained");
        assert_eq!((evidence.observed_ns, evidence.hits), (recipe.revision, 1));
    }
    let (updates, count) = reader.columns(&root, None, config.voxel_size).unwrap();
    assert_eq!(count, recipe.keys_count);
    assert_columns(&updates, &keys, fine, config.voxel_size);
    let mut builder = SurfaceMapBuilder::configured(config, Some(envelope));
    let built = builder.build_column_updates(&updates, fine, true, count).unwrap();
    assert_eq!(built.stats.input_points, recipe.keys_count);
    let build_stats = built.stats;
    let map = Arc::new(built.snapshot.with_observations(Some(Arc::clone(&observations))));
    assert_eq!(map.source_voxel_size_m().to_bits(), recipe.fine_resolution_bits);
    assert!(Arc::ptr_eq(map.observations().unwrap(), &observations));
    let cfg = map.config();
    for (name, value) in [
        ("voxel_size", cfg.voxel_size), ("robot_height", cfg.robot_height),
        ("surface_closing_radius", cfg.surface_closing_radius), ("node_spacing_m", cfg.node_spacing_m),
        ("wall_clearance_m", cfg.wall_clearance_m), ("wall_buffer_m", cfg.wall_buffer_m),
        ("wall_buffer_weight", cfg.wall_buffer_weight), ("max_slope_deg", cfg.max_slope_deg),
        ("step_threshold_m", cfg.step_threshold_m), ("min_step_tread_depth_m", cfg.min_step_tread_depth_m),
        ("step_penalty_weight", cfg.step_penalty_weight), ("goal_tolerance", cfg.goal_tolerance),
        ("viz_publish_hz", cfg.viz_publish_hz),
    ] {
        assert_eq!(value.to_bits(), field(name).to_bits(), "rebuilt map config {name}");
    }
    let rebuilt_body = map.body_envelope().expect("original full body envelope");
    assert_eq!(rebuilt_body.footprint_xy_m.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>(),
        recipe.footprint_xy_bits);
    assert_eq!([rebuilt_body.z_bounds_m[0], rebuilt_body.z_bounds_m[1],
        rebuilt_body.body_to_support_z_m, rebuilt_body.max_step_up_m, rebuilt_body.max_step_down_m,
        rebuilt_body.max_cross_slope_deg, rebuilt_body.support_uncertainty_m].map(f64::to_bits),
        recipe.body_scalar_bits);
    let build_elapsed = build_started.elapsed();

    let current = decode_body(trace.current_bits);
    let applied = twist(trace.applied_bits);
    let time = trace.time_ns;
    let route: Vec<Body> = trace.route_bits.iter().copied().map(decode_body).collect();
    let selected: usize = trace.outcome.split("selected_index=").nth(1).unwrap()
        .split_whitespace().next().unwrap().parse().unwrap();
    let current_view = map.with_current_body_support(current.0, current.1).unwrap();
    let view = &current_view;
    eprintln!("DOOR_STATIC_SOURCE build={build_elapsed:?} keys={} blocks={} stats={build_stats:?}", keys.len(), root.blocks.len());
    // First require that the original full outcome reproduced unchanged.
    // These are STATIC GEOMETRY/CONNECTOR DIAGNOSTICS ONLY. They do not validate a
    // command at the captured nonzero velocity, acceleration limits or full stop.
    {
        use nuwax_nav_astar::controllers::angle_diff;
        use nuwax_nav_control::PlanarPose;
        use nuwax_nav_mls_planner::SurfaceSnapshot;
    
        #[derive(Debug)]
        struct ConnectorFailure {
            stage: &'static str,
            substep: usize,
            substeps: usize,
            previous: Body,
            requested_xy_yaw: [f64; 3],
            projected: Option<Body>,
            reason: String,
        }
        type Probe = Result<Body, ConnectorFailure>;
        fn pose_yaw(pose: Body) -> f64 {
            let q = pose.1;
            Quaternion { x: q[0], y: q[1], z: q[2], w: q[3] }.yaw()
        }
        fn failed(stage: &'static str, index: usize, count: usize, previous: Body,
            requested: [f64; 3], projected: Option<Body>, reason: String) -> ConnectorFailure {
            ConnectorFailure { stage, substep: index, substeps: count, previous,
                requested_xy_yaw: requested, projected, reason }
        }
        // Exact connection_endpoint sampling/height/sweep semantics, with the
        // discarded error and first failing sample retained for diagnostics.
        fn sampled_endpoint(map: &SurfaceSnapshot, from: Body, next: &PoseStamped) -> Probe {
            let initial_yaw = pose_yaw(from);
            let requested = [next.pose.position.x, next.pose.position.y, next.pose.orientation.yaw()];
            let model = map.body_envelope().ok_or_else(||
                failed("missing-envelope", 0, 0, from, requested, None, String::new()))?;
            let turn = angle_diff(next.pose.orientation.yaw(), initial_yaw);
            let dx = next.pose.position.x - from.0[0];
            let dy = next.pose.position.y - from.0[1];
            let distance = dx.hypot(dy);
            let count = ((distance + model.radius() * turn.abs())
                / (f64::from(map.source_voxel_size_m()) * 0.5))
                .max(turn.abs() / 0.05).ceil().max(1.0) as usize;
            if count > 4096 {
                return Err(failed("substep-budget", 0, count, from, requested, None, String::new()));
            }
            let mut previous = from;
            for index in 1..=count {
                let fraction = index as f64 / count as f64;
                let xy = [from.0[0] + dx * fraction, from.0[1] + dy * fraction];
                let yaw = initial_yaw + turn * fraction;
                let requested = [xy[0], xy[1], yaw];
                let mut projected = map.body_pose_after_motion(previous, xy, yaw)
                    .map_err(|e| failed("projection", index, count, previous, requested, None, format!("{e:?}")))?;
                if distance < 1e-9 { projected.0[2] = previous.0[2]; }
                map.validate_body_motion(previous, projected, &[])
                    .map_err(|e| failed("sweep", index, count, previous, requested, Some(projected), format!("{e:?}")))?;
                previous = projected;
            }
            Ok(previous)
        }
        // Exact forward_arc_connects geometry. `Ok` carries the final sampled body
        // rather than discarding it; no target-yaw approval is added or removed.
        fn sampled_arc(map: &SurfaceSnapshot, from: Body, next: &PoseStamped) -> Probe {
            let yaw = pose_yaw(from);
            let dx = next.pose.position.x - from.0[0];
            let dy = next.pose.position.y - from.0[1];
            let distance = dx.hypot(dy);
            let error = angle_diff(dy.atan2(dx), yaw);
            let requested = [next.pose.position.x, next.pose.position.y, yaw + 2.0 * error];
            let model = map.body_envelope().ok_or_else(||
                failed("missing-envelope", 0, 0, from, requested, None, String::new()))?;
            if error.abs() >= std::f64::consts::FRAC_PI_2 {
                return Err(failed("forward-half-plane", 0, 0, from, requested, None,
                    format!("bearing_error={error:?}")));
            }
            let angle = 2.0 * error;
            let length = if error.abs() < 1e-9 { distance } else { distance * error / error.sin() };
            let count = ((length + model.radius() * angle.abs())
                / (f64::from(map.source_voxel_size_m()) * 0.5))
                .max(angle.abs() / 0.05).ceil().max(1.0) as usize;
            if count > 4096 {
                return Err(failed("substep-budget", 0, count, from, requested, None, String::new()));
            }
            let start = PlanarPose { x: from.0[0], y: from.0[1], yaw };
            let motion = Twist2 { linear: Point2::new(length, 0.0), angular_z: angle };
            let mut previous = from;
            for index in 1..=count {
                let planar = nuwax_nav_control::integrate(start, motion, index as f64 / count as f64);
                let requested = [planar.x, planar.y, planar.yaw];
                let projected = map.body_pose_after_motion(previous, [planar.x, planar.y], planar.yaw)
                    .map_err(|e| failed("projection", index, count, previous, requested, None, format!("{e:?}")))?;
                map.validate_body_motion(previous, projected, &[])
                    .map_err(|e| failed("sweep", index, count, previous, requested, Some(projected), format!("{e:?}")))?;
                previous = projected;
            }
            Ok(previous)
        }
        fn horizontal(a: &PoseStamped, b: &PoseStamped) -> f64 {
            (a.pose.position.x - b.pose.position.x).hypot(a.pose.position.y - b.pose.position.y)
        }
        fn rotation(a: &PoseStamped, b: &PoseStamped) -> bool {
            horizontal(a, b) < 1e-5 && angle_diff(a.pose.orientation.yaw(), b.pose.orientation.yaw()).abs() > 1e-6
        }
        // This capture reaches the nonrotation, no-latch branch. Re-run that branch's
        // index-only lookahead rules exactly, not a distance-only approximation.
        fn nonrotation_lookahead(path: &[PoseStamped], path_index: usize,
            position: Point3, lookahead_m: f64) -> usize {
            let target = &path[path_index];
            let previous = &path[path_index - 1];
            assert!(!rotation(target, previous));
            let mut index = path_index;
            let mut turn_direction = angle_diff(target.pose.orientation.yaw(), previous.pose.orientation.yaw());
            if turn_direction.abs() <= 1e-6 { turn_direction = 0.0; }
            while index + 1 < path.len() {
                let a = &path[index];
                let b = &path[index + 1];
                if horizontal(a, b) < 1e-5 && !rotation(a, b) {
                    if position.distance(b.pose.position) > lookahead_m { break; }
                    index += 1;
                    continue;
                }
                let next_turn = angle_diff(b.pose.orientation.yaw(), a.pose.orientation.yaw());
                if next_turn.abs() > 1e-6 {
                    if turn_direction * next_turn < 0.0 { break; }
                    if turn_direction == 0.0 { turn_direction = next_turn; }
                }
                let direction = [target.pose.position.x - previous.pose.position.x,
                    target.pose.position.y - previous.pose.position.y];
                let next_direction = [b.pose.position.x - a.pose.position.x,
                    b.pose.position.y - a.pose.position.y];
                let lengths = direction[0].hypot(direction[1]) * next_direction[0].hypot(next_direction[1]);
                if horizontal(a, b) < 1e-5
                    && angle_diff(b.pose.orientation.yaw(), a.pose.orientation.yaw()).abs() > 1e-6
                    || direction[0] * next_direction[0] + direction[1] * next_direction[1]
                        < lengths * 0.25_f64.cos()
                    || angle_diff(b.pose.orientation.yaw(), target.pose.orientation.yaw()).abs() > 0.25
                    || position.distance(b.pose.position) > lookahead_m {
                    break;
                }
                index += 1;
            }
            index
        }
        fn report(label: &str, started: Instant, result: &Probe) {
            match result {
                Ok(at) => eprintln!("DOOR_CONNECTOR label={label} STATIC_ONLY result=ok endpoint={at:?} elapsed={:?}", started.elapsed()),
                Err(error) => eprintln!("DOOR_CONNECTOR label={label} STATIC_ONLY result=reject failure={error:?} elapsed={:?}", started.elapsed()),
            }
        }
    
        let diagnostic_started = Instant::now();
        let path_index: usize = trace.outcome.split("局部路径点").nth(1).unwrap()
            .split("无法").next().unwrap().parse().unwrap();
        assert_eq!(path_index, selected, "this capture must have exhausted its farther arc lookahead");
        assert!(trace.outcome.contains(" initial_alignment_index=None "));
        assert!(trace.outcome.contains(" alignment_index=None "));
        let poses: Vec<_> = route.iter().copied().map(|p| stamped(p, time)).collect();
        let position = Point3::new(current.0[0], current.0[1], current.0[2]);
        // Same original default config and effective cruise computation.
        let defaults = SurfaceFollowerConfig::new(0.4);
        let cruise = 0.3 * defaults.controller.nerf_speed.min(1.0);
        let lookahead_m = defaults.lookahead_m.max(cruise / 1.5 + f64::from(view.config().voxel_size));
        let ceiling = nonrotation_lookahead(&poses, path_index, position, lookahead_m);
        eprintln!("DOOR_CONNECTOR_START STATIC_ONLY path_index={path_index} selected={selected} \
            original_lookahead_ceiling={ceiling} lookahead_m={lookahead_m:?} current={current:?} \
            actual_applied={applied:?}; no moving-command or braking permission is inferred");
    
        // Test every original bounded lookahead target. Farthest first makes a
        // potentially viable farther-bearing alternative visible immediately.
        let mut viable_arcs = Vec::new();
        let mut viable_bearings = Vec::new();
        let mut viable_bearings_with_route_yaw = Vec::new();
        for index in (path_index..=ceiling).rev() {
            let target = &poses[index];
            let bearing = (target.pose.position.y - position.y).atan2(target.pose.position.x - position.x);
            let started = Instant::now();
            let arc = sampled_arc(view, current, target);
            if arc.is_ok() { viable_arcs.push(index); }
            report(&format!("target={index}:original-forward-arc"), started, &arc);
    
            let mut translated = target.clone();
            translated.pose.orientation = Quaternion::from_yaw(bearing);
            let mut turned = translated.clone();
            turned.pose.position = position;
            let started = Instant::now();
            let turn = sampled_endpoint(view, current, &turned);
            report(&format!("target={index}:bearing={bearing:?}:bearing-turn"), started, &turn);
            if let Ok(at_turn) = turn {
                let started = Instant::now();
                let translation = sampled_endpoint(view, at_turn, &translated);
                if translation.is_ok() { viable_bearings.push(index); }
                report(&format!("target={index}:bearing-translation"), started, &translation);
                if let Ok(at_target) = translation {
                    // Extra stage is separate: runtime's bearing-alignment probe
                    // does NOT require restoring route yaw inside that probe.
                    let started = Instant::now();
                    let restore = sampled_endpoint(view, at_target, target);
                    if restore.is_ok() { viable_bearings_with_route_yaw.push(index); }
                    report(&format!("target={index}:restore-planned-yaw"), started, &restore);
                }
            }
        }
    
        // Alternative requested by the owner: yaw exactly zero at the actual XY,
        // then forward at constant actual Y. No lateral motion or pose snapping.
        let mut zero_yaw_stance = stamped(current, time);
        zero_yaw_stance.pose.orientation = Quaternion::from_yaw(0.0);
        let started = Instant::now();
        let tangent_turn = sampled_endpoint(view, current, &zero_yaw_stance);
        report("tangent:zero-yaw-turn-at-actual-xy", started, &tangent_turn);
        if let Ok(aligned) = tangent_turn {
            // Compare the exact selected X with the farthest original lookahead X.
            for index in [selected, ceiling] {
                let mut projected = poses[index].clone();
                projected.pose.position.y = position.y;
                projected.pose.orientation = Quaternion::from_yaw(0.0);
                assert!(projected.pose.position.x > position.x);
                let started = Instant::now();
                let translated = sampled_endpoint(view, aligned, &projected);
                report(&format!("target={index}:zero-yaw-forward-constant-actual-y"), started, &translated);
                let started = Instant::now();
                let arc = sampled_arc(view, aligned, &projected);
                report(&format!("target={index}:zero-yaw-constant-y-forward-arc"), started, &arc);
            }
        }
        viable_arcs.sort_unstable();
        viable_bearings.sort_unstable();
        viable_bearings_with_route_yaw.sort_unstable();
        eprintln!("DOOR_CONNECTOR_SUMMARY STATIC_ONLY original_arc_ok={viable_arcs:?} \
            bearing_turn_then_translation_ok={viable_bearings:?} \
            bearing_then_restore_route_yaw_ok={viable_bearings_with_route_yaw:?} \
            total={:?}; nonzero-velocity reaction/braking checks remain untested by these probes",
            diagnostic_started.elapsed());
    }
}

