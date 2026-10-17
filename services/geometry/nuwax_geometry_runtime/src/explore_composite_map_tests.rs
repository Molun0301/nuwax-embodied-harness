// A NavigationMap3D publication is a composite: the occupancy/mask receipt can
// advance while the immutable cloud remains the same. Use the real map cache.
#[tokio::test]
async fn surface_composite_mask_receipts_advance_without_rebuilding_cloud() {
    let (handle, _) = configured_handle();
    let config = surface_config("map");
    handle
        .configure_surface_source(Ok(config.clone()), 37, [7; 16])
        .await
        .unwrap();
    let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
    let mut first = surface_message("map", 1);
    first.virtual_obstacles = Some(schema::OccupancyGrid {
        header: first.cloud.header.clone(),
        info: schema::MapMetaData {
            map_load_time_sec: first.cloud.header.stamp_sec,
            map_load_time_nsec: first.cloud.header.stamp_nsec,
            resolution: 0.1,
            width: 50,
            height: 30,
            origin: schema::Pose {
                position: [0.0; 3],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        },
        data: vec![0; 1500],
    });
    let first_ticket = handle
        .begin_surface_map_update(&first, now_monotonic_nanos())
        .await
        .unwrap()
        .unwrap();
    let initial = cache.build(&first).unwrap();
    assert!(
        handle
            .finish_surface_map_update(first_ticket.clone(), Ok(initial.clone()))
            .await
    );

    let mut metadata = first.clone();
    let advance = |message: &mut schema::NavigationMap3D| {
        message.committed_map.revision += 1;
        message.committed_map.commit_sequence += 1;
        message.committed_map.source_timestamp_ns += 1;
        let mask = message.virtual_obstacles.as_mut().unwrap();
        mask.header.seq = message.committed_map.revision as i32;
        mask.header.stamp_sec = (message.committed_map.source_timestamp_ns / 1_000_000_000) as i32;
        mask.header.stamp_nsec = (message.committed_map.source_timestamp_ns % 1_000_000_000) as i32;
        mask.info.map_load_time_sec = mask.header.stamp_sec;
        mask.info.map_load_time_nsec = mask.header.stamp_nsec;
    };
    advance(&mut metadata);
    schema::validate_navigation_map_3d(&metadata).unwrap();
    let retained = cache.build(&metadata).unwrap();
    assert!(
        Arc::ptr_eq(&initial, &retained),
        "receipt clocks do not rebuild geometry"
    );
    assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
    let metadata_received = now_monotonic_nanos();
    let metadata_ticket = handle
        .begin_surface_map_update(&metadata, metadata_received)
        .await
        .expect("new occupancy receipt with an unchanged cloud must be admitted")
        .unwrap();
    let mut wrong_domain = metadata.clone();
    advance(&mut wrong_domain);
    wrong_domain.map_frame_generation += 1;
    assert!(
        handle
            .begin_surface_map_update(&wrong_domain, now_monotonic_nanos())
            .await
            .is_err(),
        "same-root receipt advancement must not cross map domains"
    );
    let mut wrong_commit = metadata.clone();
    advance(&mut wrong_commit);
    wrong_commit.committed_map.commit_sequence -= 1;
    schema::validate_navigation_map_3d(&wrong_commit).unwrap();
    assert!(
        handle
            .begin_surface_map_update(&wrong_commit, now_monotonic_nanos())
            .await
            .is_err(),
        "occupancy revision cannot advance without a newer owner commit"
    );

    let mut changed = metadata.clone();
    advance(&mut changed);
    changed.virtual_obstacles.as_mut().unwrap().data.fill(100);
    let changed_ticket = handle
        .begin_surface_map_update(&changed, now_monotonic_nanos())
        .await
        .unwrap()
        .unwrap();
    let blocked = cache.build(&changed).unwrap();
    assert_eq!(
        cache.last_build_stats().unwrap().input_points,
        0,
        "mask derivation reuses physical cloud"
    );
    assert!(initial.locate_support([1.5, 1.5, 0.0]).is_ok());
    assert!(blocked.locate_support([1.5, 1.5, 0.0]).is_err());
    // A newer same-domain mask may already be queued. It must not starve a
    // valid completion, but once installed it must fence the older result.
    assert!(
        handle
            .finish_surface_map_update(metadata_ticket.clone(), Ok(retained))
            .await
    );
    assert_eq!(
        handle.inner.lock().await.committed_map.as_ref(),
        Some(&metadata.committed_map)
    );
    assert_eq!(
        handle.inner.lock().await.costmap_received_monotonic_ns,
        metadata_received
    );

    assert!(
        handle
            .finish_surface_map_update(changed_ticket, Ok(blocked.clone()))
            .await
    );
    assert!(
        !handle
            .finish_surface_map_update(metadata_ticket, Ok(initial.clone()))
            .await,
        "late old mask result must not restore the formerly free region"
    );
    assert!(
        !handle
            .finish_surface_map_update(first_ticket, Ok(initial))
            .await
    );
    assert!(handle
        .begin_surface_map_update(&metadata, now_monotonic_nanos())
        .await
        .is_err());
    assert!(handle
        .begin_surface_map_update(&changed, now_monotonic_nanos())
        .await
        .is_err());
    let guard = handle.inner.lock().await;
    assert_eq!(guard.committed_map.as_ref(), Some(&changed.committed_map));
    assert!(Arc::ptr_eq(
        &guard
            .surface_mode
            .as_ref()
            .unwrap()
            .receiver
            .snapshot()
            .unwrap()
            .unwrap(),
        &blocked
    ));
}
