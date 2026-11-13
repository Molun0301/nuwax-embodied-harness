use super::*;

#[test]
fn invalid_new_mask_never_reuses_or_replaces_the_complete_baseline() {
    for variant in ["dimension_limit", "extent"] {
        let mut message=map_message(1,"mask-invalid");
        message.virtual_obstacles=Some(virtual_mask(&message,false));
        let mut cache=SurfaceGeometryCache::new(config().geometry);
        let original=cache.build(&message).unwrap();
        advance_mask(&mut message,false);
        let mut invalid=message.clone();
        let mask=invalid.virtual_obstacles.as_mut().unwrap();
        match variant {
            "dimension_limit"=>{mask.info.width=65_537;mask.info.height=1;mask.data=vec![0;65_537];},
            "extent"=>mask.info.origin.position[0]=f64::MAX,
            _=>unreachable!(),
        }
        assert!(cache.build(&invalid).is_err(),"{variant} returned a stale success");
        let recovered=cache.build(&message).unwrap();
        assert!(Arc::ptr_eq(&original,&recovered),"{variant} advanced failed metadata");
    }
}

#[tokio::test]
async fn held_global_owner_keeps_unchanged_mask_work_but_changed_mask_retires_it() {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
    let entered = Arc::new(AtomicBool::new(false));
    let retired = Arc::new(AtomicBool::new(false));
    let calls = Arc::new(AtomicUsize::new(0));
    let (a, b, c) = (entered.clone(), retired.clone(), calls.clone());
    let computer: SurfacePathComputer = Arc::new(move |map, pose, goal, configured| {
        if c.fetch_add(1, Ordering::SeqCst) == 0 {
            a.store(true, Ordering::SeqCst);
            let deadline=Instant::now()+Duration::from_secs(10);
            while !configured.planning_cancelled.as_ref().unwrap().load(Ordering::Acquire) {
                if Instant::now()>=deadline {return Err(module_error("held mask owner","test barrier expired"));}
                std::thread::sleep(Duration::from_millis(1));
            }
            b.store(true, Ordering::SeqCst);
            return Err(module_error("held mask owner", "retired"));
        }
        body_path(map, pose, goal, configured).map(SurfacePlan::complete)
    });
    let mut configured = config();
    configured.apply_calibration(nuwax_nav_types_messages::RobotGeometry {
        self_collision: None,
        footprint_xy_m: vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        z_min_m: -0.05, z_max_m: 0.2, source: "mask reuse owner fixture".into(),
    }, None).unwrap();
    let mut harness = Harness::start_with_config_rate(computer, Duration::from_millis(25), configured).await;
    {
        let mut feed = harness.feed.lock().unwrap();
        feed.map.virtual_obstacles = Some(virtual_mask(&feed.map, false));
    }
    harness.wait_ready().await;
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let ready = harness.hub.latest.lock().unwrap().as_ref().and_then(|m|m.snapshot.as_ref().ok())
                .is_some_and(|s| !s.navigation_column_allowed([-1, -1], 0.1));
            if ready { break; }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await.unwrap();
    let goal = goal_for(&harness.feed.lock().unwrap().map, 1, [3.2,1.55,2.5]);
    harness.goal(&goal).await;
    tokio::time::timeout(Duration::from_secs(3), async {
        while !entered.load(Ordering::SeqCst) { tokio::time::sleep(Duration::from_millis(5)).await; }
    }).await.unwrap();
    let initial = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    tokio::time::timeout(Duration::from_secs(3), async {
        loop {
            let current = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
            if current.authority.revision >= initial.authority.revision + 3 {
                assert!(initial.snapshot.as_ref().unwrap().same_geometry(current.snapshot.as_ref().unwrap()));
                assert!(current.received_at > initial.received_at);
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await.unwrap();
    assert_eq!(calls.load(Ordering::SeqCst), 1);
    assert!(!retired.load(Ordering::SeqCst));
    assert!(tokio::time::timeout(Duration::from_millis(50), harness.paths.next()).await.is_err(),
        "identical mask revisions must not create a competing quick lane");
    {
        let mut feed = harness.feed.lock().unwrap();
        feed.map.virtual_obstacles.as_mut().unwrap().data[2 * 40 + 35] = 100;
    }
    let path = harness.path_for(&goal).await;
    assert!(path.path.poses.len() > 1);
    tokio::time::timeout(Duration::from_secs(2), async {
        while !retired.load(Ordering::SeqCst) { tokio::time::sleep(Duration::from_millis(5)).await; }
    }).await.unwrap();
    let changed = harness.hub.latest.lock().unwrap().as_ref().unwrap().clone();
    assert!(!initial.snapshot.as_ref().unwrap().same_geometry(changed.snapshot.as_ref().unwrap()));
    harness.cancel(&goal).await;
    harness.terminal(TerminalOutcome::Cancelled).await;
    harness.shutdown().await;
}

#[test]
fn new_physical_updates_reapply_only_cell_proven_retained_masks() {
    for variant in ["same_cells", "changed_cells", "layout", "domain"] {
        let mut message = map_message(1, "retained-mask-update");
        message.virtual_obstacles = Some(virtual_mask(&message, true));
        let mut cache = SurfaceGeometryCache::new(config().geometry);
        let previous = cache.build(&message).unwrap();
        let old_physical = crate::surface_map::tests::physical_snapshot(&cache).unwrap();
        advance_mask(&mut message, true);
        advance_scan(&mut message);
        // Replace a real physical floor support with a higher obstacle column.
        let z_offset = (20 * 30 + 15) * 12 + 8;
        message.cloud.data[z_offset..z_offset + 4].copy_from_slice(&2.85_f32.to_le_bytes());
        match variant {
            "changed_cells" => message.virtual_obstacles.as_mut().unwrap().data[0] = 100,
            "layout" => message.virtual_obstacles.as_mut().unwrap().info.origin.position[0] = -0.1,
            "domain" => message.session_id = "retained-mask-new-domain".into(),
            _ => {}
        }
        let actual = cache.build(&message).unwrap();
        let physical = crate::surface_map::tests::physical_snapshot(&cache).unwrap();
        assert!(!Arc::ptr_eq(&old_physical, &physical), "{variant}");
        assert!(!actual.same_geometry(&previous), "{variant}");
        let mask = message.virtual_obstacles.as_ref().unwrap();
        let blocked: Vec<_> = mask.data.iter().map(|&value| value == 100).collect();
        let expected = physical.with_virtual_obstacles(
            [mask.info.origin.position[0], mask.info.origin.position[1]],
            f64::from(mask.info.resolution), mask.info.width as u32, mask.info.height as u32,
            &blocked).unwrap();
        assert!(actual.matches_virtual_obstacles(
            [mask.info.origin.position[0], mask.info.origin.position[1]],
            f64::from(mask.info.resolution), mask.info.width as u32, mask.info.height as u32,
            blocked.iter().copied()), "{variant}");
        let actual_columns: std::collections::BTreeMap<_,_> = actual.navigation_height_columns().collect();
        let expected_columns: std::collections::BTreeMap<_,_> = expected.navigation_height_columns().collect();
        assert_eq!(actual_columns, expected_columns, "{variant}");
        for (column, _) in actual_columns {
            assert_eq!(actual.navigation_column_allowed(column, 0.1),
                expected.navigation_column_allowed(column, 0.1));
        }
        let actual=super::test_source::full_surface_fixture(&actual);
        let expected=super::test_source::full_surface_fixture(&expected);
        let mut actual_cells: Vec<_> = actual.cell_keys().collect();
        let mut expected_cells: Vec<_> = expected.cell_keys().collect();
        actual_cells.sort_unstable(); expected_cells.sort_unstable();
        assert_eq!(actual_cells, expected_cells, "{variant}");
        for cell in actual_cells {
            let mut a = actual.neighbors(cell); let mut b = expected.neighbors(cell);
            a.sort_unstable(); b.sort_unstable();
            assert_eq!(a, b, "{variant} {cell:?}");
            for next in a { assert_eq!(actual.transition(cell, next), expected.transition(cell, next)); }
            assert_eq!(actual.navigation_column_allowed([cell.0, cell.1], 0.1),
                expected.navigation_column_allowed([cell.0, cell.1], 0.1));
        }
    }
}
