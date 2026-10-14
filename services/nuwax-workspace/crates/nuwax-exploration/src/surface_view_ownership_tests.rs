// Bounded actual-selection diagnostic. No allocator or content-hash instrumentation.
thread_local! {
    pub(super) static VIEW_TRANSFER: std::cell::Cell<(usize, usize, usize)> = const { std::cell::Cell::new((0, 0, 0)) };
}

#[test]
#[ignore = "serial paired release diagnostic; prints one JSON record per bounded sample"]
fn surface_view_ownership_profile() {
    // Fixed plan: seven cohorts, two measured preparations per cohort. Caller runs
    // three alternating baseline/candidate process pairs, never an adaptive loop.
    for (width, rays, directional) in [
        (20, 0, false), (20, 16, false), (20, 128, false),
        (40, 128, false), (40, 128, true),
        (80, 512, false), (80, 512, true),
    ] {
        let config = map().config().clone();
        let mut points = Vec::new();
        for level in [0.0, 3.0] {
            for x in 0..width {
                for y in 0..width {
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, level + 0.05));
                }
            }
        }
        let snapshot = Arc::new(SurfaceSnapshot::from_points(&points, config, 0.1).unwrap());
        let receiver = SurfaceInputReceiver::new();
        receiver.observe(snapshot).unwrap();
        let configure = || {
            let mut adapter = source(receiver.clone(), 1);
            adapter.configure_constraints(None, 1, 0.001).unwrap();
            if rays != 0 {
                let mut model = volume_model();
                model.max_rays = rays;
                model.max_range_m = 10.0;
                if directional { model.horizontal_fov_rad = std::f64::consts::FRAC_PI_2; }
                adapter.set_observation_model(model).unwrap();
            }
            adapter
        };
        let mut current = state(0.4, 1);
        current.robot_pose.pose.position.x = f64::from(width) * 0.05 + 0.05;
        current.robot_pose.pose.position.y = f64::from(width) * 0.05 + 0.05;
        // Full oracle/state comparisons are outside timing. This also verifies
        // score, ordering and all selected-view metadata against the frozen path.
        let mut reference = configure();
        let expected = reference.legacy_poll_goal(&current);
        let mut accepted = configure();
        let planning = receiver.freeze().unwrap().fork_for_planning();
        let before = committed(&accepted);
        let preview = accepted.prepare_poll_with_planning_input(&current, &planning).unwrap();
        assert_eq!(comparable_surface_outcome(preview.outcome()), comparable_surface_outcome(&expected));
        assert_eq!(committed(&accepted), before);
        assert_eq!(comparable_surface_outcome(&accepted.commit_prepared_poll()), comparable_surface_outcome(&expected));
        assert_eq!(committed(&accepted), committed(&reference));
        assert!(matches!(&expected, Ok(ExplorationPoll::Goal(_))), "cohort must actually select a goal");
        for sample in 0..2 {
            let mut adapter = configure();
            let before = committed(&adapter);
            let planning = receiver.freeze().unwrap().fork_for_planning();
            VIEW_TRANSFER.with(|counts| counts.set((0, 0, 0)));
            let started = std::time::Instant::now();
            let preview = adapter.prepare_poll_with_planning_input(&current, &planning).unwrap();
            adapter.discard_prepared_poll().unwrap();
            let micros = started.elapsed().as_secs_f64() * 1e6;
            assert_eq!(committed(&adapter), before);
            assert_eq!(comparable_surface_outcome(preview.outcome()), comparable_surface_outcome(&expected));
            let (admissible, nonempty_sets, cells) = VIEW_TRANSFER.with(std::cell::Cell::get);
            println!("{}", serde_json::json!({
                "width": width, "rays": rays, "directional": directional, "sample": sample,
                "prepare_discard_us": micros, "admissible": admissible,
                "nonempty_sets": nonempty_sets, "cells": cells,
                "baseline_duplicate_key_payload_bytes": cells * std::mem::size_of::<[i32; 3]>(),
                "outcome": format!("{:?}", comparable_surface_outcome(preview.outcome())),
            }));
        }
    }
}
