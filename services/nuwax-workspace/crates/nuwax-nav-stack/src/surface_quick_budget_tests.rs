// Direct production-function cost diagnostic. No transport, controller, or actuator.
use super::*;
use std::sync::atomic::AtomicBool;

// A thread-scoped observer uses an existing production event immediately before
// PCT's cache-publication checkpoint. No public test hook or transport substitute.
struct MapBuiltObserver {
    cancel: Option<Arc<AtomicBool>>,
    builds: Arc<std::sync::atomic::AtomicUsize>,
}
impl tracing::Subscriber for MapBuiltObserver {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool { true }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct EventName(bool);
        impl tracing::field::Visit for EventName {
            fn record_debug(&mut self, _: &tracing::field::Field, _: &dyn std::fmt::Debug) {}
            fn record_str(&mut self, field: &tracing::field::Field, value: &str) {
                self.0 |= field.name() == "event" && value == "navigation.pct.map";
            }
        }
        let mut name = EventName(false);
        event.record(&mut name);
        if name.0 {
            self.builds.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if let Some(flag) = &self.cancel {
                flag.store(true, std::sync::atomic::Ordering::Release);
            }
        }
    }
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
}

#[test]
fn owned_flag_body_path_binding_rejects_preinstall_and_preserves_prior_cache() {
    use std::sync::atomic::{AtomicUsize, Ordering};
    let (surface, start, goal) = fixture(false);
    let previous = surface.clone().with_ground_evidence(
        nuwax_nav_mls_planner::surface::GroundEvidence::OccupiedVolume);
    assert!(!nuwax_nav_pct::same_source_geometry(&surface, &previous));
    for budget in [0, 30_000] {
        for populated in [false, true] {
            let mut config = calibrated_config(false);
            let old = populated.then(|| config.follower.pct.map(&previous).unwrap());
            let cancelled = Arc::new(AtomicBool::new(false));
            config.planning_cancelled = Some(cancelled.clone());
            let builds = Arc::new(AtomicUsize::new(0));
            let result = tracing::subscriber::with_default(MapBuiltObserver {
                cancel: Some(cancelled.clone()), builds: builds.clone(),
            }, || body_path_with_budget(&surface, &start, &goal, &config, budget));
            assert_eq!(builds.load(Ordering::Relaxed), 1, "the actual map-built event must fire");
            assert!(cancelled.load(Ordering::Acquire));
            assert!(result.unwrap_err().to_string().contains("层析规划已取消"));
            assert!(!nuwax_nav_pct::planning_cancelled(), "body_path must restore worker TLS");
            let fresh = Arc::new(AtomicBool::new(false));
            config.planning_cancelled = Some(fresh);
            tracing::subscriber::with_default(MapBuiltObserver {
                cancel: None, builds: builds.clone(),
            }, || {
                if let Some(old) = old {
                    assert!(Arc::ptr_eq(&old, &config.follower.pct.map(&previous).unwrap()));
                    assert_eq!(builds.load(Ordering::Relaxed), 1, "the old cache must survive");
                }
                let retry = body_path_with_budget(&surface, &start, &goal, &config, budget).unwrap();
                assert!(!retry.poses.is_empty());
                assert_eq!(builds.load(Ordering::Relaxed), 2, "cancelled construction must not install its map");
                let repeated = body_path_with_budget(&surface, &start, &goal, &config, budget).unwrap();
                assert_eq!(retry, repeated);
                assert_eq!(builds.load(Ordering::Relaxed), 2, "a successful complete map must remain reusable");
            });
            assert!(cancelled.load(Ordering::Acquire), "a fresh request must not reset the old flag");
        }
    }
}

#[test]
fn owned_flag_preserves_full_paths_and_quick_zero_is_direct_only() {
    let mut paths = Vec::new();
    for (name, detour, penalty) in [("direct", false, false), ("detour", true, false), ("penalty", false, true)] {
        let (surface, start, goal) = fixture(detour);
        let bits = |path: &Path3D| path.poses.iter().map(|pose| {
            let p = pose.pose.position;
            let q = pose.pose.orientation;
            ([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()],
                [q.x.to_bits(), q.y.to_bits(), q.z.to_bits(), q.w.to_bits()],
                pose.frame_id.clone(), pose.stamp)
        }).collect::<Vec<_>>();
        let baseline = body_path_with_budget(&surface, &start, &goal, &calibrated_config(penalty), 30_000).unwrap();
        let expected = bits(&baseline);
        for budget in [0, 30_000] {
            let mut configured = calibrated_config(penalty);
            configured.planning_cancelled = Some(Arc::new(AtomicBool::new(false)));
            let cold = body_path_with_budget(&surface, &start, &goal, &configured, budget);
            let warm = body_path_with_budget(&surface, &start, &goal, &configured, budget);
            if budget == 0 && (detour || penalty) {
                let cold = cold.unwrap_err().to_string();
                assert!(cold.contains("BudgetExhausted"), "{name}: {cold}");
                assert_eq!(cold, warm.unwrap_err().to_string());
            } else {
                let (cold, warm) = (cold.unwrap(), warm.unwrap());
                assert_eq!(bits(&cold), bits(&warm), "{name} cold/warm budget {budget}");
                if budget > 0 { assert_eq!(bits(&cold), expected, "{name} full budget"); }
                else { assert_eq!(cold.poses.len(), 2, "zero only validates the direct chord"); }
            }
            assert!(!nuwax_nav_pct::planning_cancelled());
        }
        let quality = metrics(&baseline, &goal);
        if detour { assert!(quality["max_lateral_deviation_m"].as_f64().unwrap() > 1.4); }
        if penalty { assert!(quality["max_lateral_deviation_m"].as_f64().unwrap() > 0.3); }
        paths.push(expected);
    }
    assert_ne!(paths[0], paths[1], "the physical detour must remain meaningful");
    assert_ne!(paths[0], paths[2], "the local penalty must still affect the route");
}

fn calibrated_config(penalty: bool) -> SurfaceNavigationConfig {
    let mut config = SurfaceNavigationConfig::new(
        "map".into(), 0.4, 0.3, 0.6, 0.1, 0.15, 18.0, 0.2,
    ).unwrap();
    config.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        footprint_xy_m: vec![[-0.15, -0.15], [0.15, -0.15], [0.15, 0.15], [-0.15, 0.15]],
        z_min_m: -0.05,
        z_max_m: 0.2,
        source: "quick-budget synthetic fixture".into(),
    }), None).unwrap();
    // Construct a new owner every time: cloning SurfaceNavigationConfig shares PCT.
    if penalty {
        config.follower.pct.report_local_blockage([4.0, 3.0, 0.1], 1.0);
    }
    config
}

fn fixture(detour: bool) -> (SurfaceSnapshot, PoseStamped, schema::GoalEnvelope) {
    let config = calibrated_config(false);
    let mut points = Vec::new();
    for x in 0..80 {
        for y in 0..60 {
            points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05));
        }
    }
    if detour {
        for x in 38..42 {
            for y in 0..45 {
                for z in 1..9 {
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, (z as f32 + 0.5) * 0.1));
                }
            }
        }
    }
    let surface = SurfaceSnapshot::from_points(&points, config.geometry, 0.1).unwrap()
        .with_body_envelope(config.body_envelope.unwrap()).unwrap();
    assert!(surface.body_envelope().is_some());
    let mut goal = goal_for(&map_message(1, "quick-budget"), 1, [7.0, 3.0, 0.5]);
    goal.options = nuwax_nav_types_messages::GoalOptions {
        allow_reverse: false, position_tolerance_m: 0.12, yaw_tolerance_rad: 0.15,
    };
    goal.goal.header.stamp_sec = 1;
    goal.goal.header.stamp_nsec = 0;
    let mut start = pose_from_schema(&goal.goal).unwrap();
    start.pose.position.x = 1.0;
    (surface, start, goal)
}

fn metrics(path: &Path3D, goal: &schema::GoalEnvelope) -> serde_json::Value {
    let length: f64 = path.poses.windows(2)
        .map(|pair| pair[0].pose.position.distance(pair[1].pose.position)).sum();
    let last = path.poses.last().unwrap().pose.position;
    let target = goal.goal.pose.position;
    let error = last.distance(Point3::new(target[0], target[1], target[2]));
    assert!(error <= goal.options.position_tolerance_m + 1e-6);
    assert!(path.poses.len() >= 2);
    serde_json::json!({"poses":path.poses.len(), "length_m":length, "endpoint_error_m":error,
        "max_lateral_deviation_m":path.poses.iter().map(|p|(p.pose.position.y-3.0).abs()).fold(0.0,f64::max)})
}

#[test]
#[ignore = "manual direct quick/full cost baseline; no transport or movement qualification"]
fn calibrated_quick_and_full_budget_cost_baseline() {
    let reverse = std::env::var_os("NEH_QUICK_BUDGET_REVERSE").is_some();
    let budgets = if reverse { [30_000, 0] } else { [0, 30_000] };
    for (name, detour, penalty) in [("direct", false, false), ("detour", true, false), ("penalty", false, true)] {
        let (surface, start, goal) = fixture(detour);
        for budget in budgets {
            let config = calibrated_config(penalty);
            let began = Instant::now();
            let cold = body_path_with_budget(&surface, &start, &goal, &config, budget);
            let cold_ns = began.elapsed().as_nanos();
            let began = Instant::now();
            let warm = body_path_with_budget(&surface, &start, &goal, &config, budget);
            let warm_ns = began.elapsed().as_nanos();
            if budget == 0 && (detour || penalty) {
                let reason = cold.unwrap_err().to_string();
                assert!(reason.contains("BudgetExhausted"));
                assert_eq!(reason, warm.unwrap_err().to_string());
                eprintln!("QUICK_BUDGET {}", serde_json::json!({"case":name,"budget":budget,
                    "cold_ns":cold_ns,"warm_ns":warm_ns,"inconclusive":reason}));
                continue;
            }
            let (cold, warm) = (cold.unwrap(), warm.unwrap());
            assert_eq!(cold, warm);
            let quality = metrics(&cold, &goal);
            let path_evidence: Vec<_> = cold.poses.iter().map(|p| {
                let b = p.pose.position;
                let q = p.pose.orientation;
                serde_json::json!({"position_bits":[b.x.to_bits(),b.y.to_bits(),b.z.to_bits()],
                    "quaternion_bits":[q.x.to_bits(),q.y.to_bits(),q.z.to_bits(),q.w.to_bits()],
                    "frame":format!("{:?}",p.frame_id),"stamp_ns":p.stamp.nanos,
                    "clock":format!("{:?}",p.stamp.clock)})
            }).collect();
            if detour { assert!(quality["max_lateral_deviation_m"].as_f64().unwrap() > 1.4); }
            if penalty { assert!(quality["max_lateral_deviation_m"].as_f64().unwrap() > 0.3); }

            eprintln!("QUICK_BUDGET {}", serde_json::json!({"case":name,"budget":budget,
                "cold_ns":cold_ns,"warm_ns":warm_ns,"quality":quality,"path_evidence":path_evidence}));
        }
        // Independent stage probe after both timed arms, on its own fresh owner;
        // do not subtract this separate measurement from body-call timing.
        let stage_config = calibrated_config(penalty);
        let began = Instant::now();
        let map = stage_config.follower.pct.map(&surface).unwrap();
        let map_ns = began.elapsed().as_nanos();
        let chord_clear = map.segment_clear([1.0,3.0,0.1], [7.0,3.0,0.1], &[]);
        assert_eq!(chord_clear, !detour);
        let route = map.plan_bounded([1.0,3.0,0.1], [7.0,3.0,0.1],
            goal.options.position_tolerance_m, &[], None, None,
            &stage_config.follower.pct.local_costs()).unwrap();
        assert!(route.points.windows(2).all(|p|map.segment_clear(p[0],p[1],&[])));
        eprintln!("QUICK_BUDGET {}", serde_json::json!({"case":name,"stage_probe":true,
            "map_only_ns":map_ns,"search_and_reduce_ms":route.stats.elapsed_ms,
            "expanded":route.stats.expanded,"cells":map.cells.len(),"chord_clear":chord_clear}));
    }
    let (surface, start, goal) = fixture(false);
    for budget in budgets {
        let mut config = calibrated_config(false);
        config.planning_cancelled = Some(Arc::new(AtomicBool::new(true)));
        let began = Instant::now();
        let cold = body_path_with_budget(&surface, &start, &goal, &config, budget);
        let cold_ns = began.elapsed().as_nanos();
        let began = Instant::now();
        let warm = body_path_with_budget(&surface, &start, &goal, &config, budget);
        let warm_ns = began.elapsed().as_nanos();
        assert!(cold.as_ref().unwrap_err().to_string().contains("取消"));
        assert_eq!(cold.unwrap_err().to_string(), warm.unwrap_err().to_string());
        eprintln!("QUICK_BUDGET {}", serde_json::json!({"case":"pre_cancelled",
            "budget":budget,"cold_ns":cold_ns,"warm_ns":warm_ns}));
    }
}

#[test]
fn mission_owner_flag_cancels_pct_construction_before_cache_install() {
    use std::sync::atomic::{AtomicUsize, Ordering};
    let (surface, start, goal) = fixture(false);
    let config = calibrated_config(false);
    let flag = Arc::new(AtomicBool::new(false));
    let builds = Arc::new(AtomicUsize::new(0));
    let body = |pose: &PoseStamped| {
        let p=pose.pose.position; let q=pose.pose.orientation;
        ([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
    };
    let goal_pose=pose_from_schema(&goal.goal).unwrap();
    let result = tracing::subscriber::with_default(MapBuiltObserver {
        cancel:Some(flag.clone()), builds:builds.clone(),
    }, || crate::surface_oriented::with_owner_cancellation(flag.clone(), ||
        crate::surface_oriented::plan_reference_cancellable(&surface,&config.follower.pct,
            body(&start),body(&goal_pose),goal.options,None,&[],30_000,None)));
    assert_eq!(builds.load(Ordering::Relaxed),1);
    assert!(result.unwrap_err().contains("取消"));
    assert!(!nuwax_nav_pct::planning_cancelled());
    tracing::subscriber::with_default(MapBuiltObserver {cancel:None,builds:builds.clone()}, || {
        config.follower.pct.map(&surface).unwrap();
    });
    assert_eq!(builds.load(Ordering::Relaxed),2,"cancelled map construction must not be cached");
}
