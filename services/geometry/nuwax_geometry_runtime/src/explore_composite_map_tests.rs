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

// 真实 composite builder 与真实探索 source 共同驱动，不冻结新提交的身份。
async fn install_surface_composite_owner_fixture(
    handle: &ExploreRuntimeHandle,
) -> (
    schema::NavigationMap3D,
    SurfaceMapTicket,
    Arc<SurfaceSnapshot>,
    nuwax_nav_stack::surface_map::SurfaceMapCache,
) {
    let config = surface_config("map");
    handle
        .configure_surface_source(Ok(config.clone()), 37, [7; 16])
        .await
        .unwrap();
    let mut message = surface_message("map", 1);
    message.virtual_obstacles = Some(schema::OccupancyGrid {
        header: message.cloud.header.clone(),
        info: schema::MapMetaData {
            map_load_time_sec: message.cloud.header.stamp_sec,
            map_load_time_nsec: message.cloud.header.stamp_nsec,
            resolution: 0.1,
            width: 90,
            height: 70,
            // 禁区范围留出边缘，避免空遮罩把原场景所有未知前沿裁掉。
            origin: schema::Pose {
                position: [-2.0, -2.0, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        },
        data: vec![0; 90 * 70],
    });
    let ticket = owner_watchdog(handle.begin_surface_map_update(&message, now_monotonic_nanos()))
        .await
        .unwrap()
        .unwrap();
    let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&config);
    let snapshot = cache.build(&message).unwrap();
    assert!(
        owner_watchdog(handle.finish_surface_map_update(ticket.clone(), Ok(snapshot.clone())))
            .await
    );
    (message, ticket, snapshot, cache)
}

fn advance_surface_composite_commit(message: &mut schema::NavigationMap3D) {
    // 与原 composite-map 回归相同：只推进占据拥有方的真实提交。
    message.committed_map.revision += 1;
    message.committed_map.commit_sequence += 1;
    message.committed_map.source_timestamp_ns += 1;
    let mask = message.virtual_obstacles.as_mut().unwrap();
    mask.header.seq = message.committed_map.revision as i32;
    mask.header.stamp_sec = (message.committed_map.source_timestamp_ns / 1_000_000_000) as i32;
    mask.header.stamp_nsec = (message.committed_map.source_timestamp_ns % 1_000_000_000) as i32;
    mask.info.map_load_time_sec = mask.header.stamp_sec;
    mask.info.map_load_time_nsec = mask.header.stamp_nsec;
    schema::validate_navigation_map_3d(message).unwrap();
}

// 预期红断言放在本函数之后；无论候选被接受还是作废，都先结算原 worker。
async fn settle_surface_composite_owner_fixture(handle: &ExploreRuntimeHandle, request_id: &str) {
    let child = handle
        .inner
        .lock()
        .await
        .active_child
        .as_ref()
        .map(|child| (child.request_id.clone(), child.lease.clone()));
    if let Some((child_id, lease)) = child {
        owner_watchdog(handle.report_navigation_result(
            &child_id,
            ChildNavigationResult::Reached(terminal_proof(&lease)),
        ))
        .await
        .unwrap();
        owner_watchdog(handle.await_compute_idle()).await.unwrap();
    }
    let active = handle.inner.lock().await.logical_active;
    if active {
        owner_watchdog(handle.stop(request_id)).await;
    }
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

async fn surface_owner_new_commit_same_view_preserves_goal(after_accept: bool) {
    let (handle, sink) = configured_handle();
    let (first, _, snapshot, mut cache) = install_surface_composite_owner_fixture(&handle).await;
    let request_id = if after_accept {
        "owner-composite-after-accept"
    } else {
        "owner-composite-before-accept"
    };
    assert_eq!(
        handle.start(request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let prepare = Arc::new(OwnerRuntimeGate::default());
    let commit = Arc::new(OwnerRuntimeGate::default());
    let _prepare_release = OwnerRuntimeGateRelease(prepare.clone());
    let _commit_release = OwnerRuntimeGateRelease(commit.clone());
    let receiver = install_gated_real_surface_source(
        &handle,
        audit.clone(),
        prepare.clone(),
        after_accept.then(|| commit.clone()),
    )
    .await;
    let original = receiver.freeze().unwrap();
    let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
    let tick = tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    });
    owner_watchdog(prepare.entered()).await;
    assert_surface_owner_capture(
        &audit.lock().unwrap().captures[0],
        &snapshot,
        original.notification(),
        &pose,
    );
    if after_accept {
        prepare.release();
        owner_watchdog(commit.entered()).await;
    }
    let mut metadata = first.clone();
    advance_surface_composite_commit(&mut metadata);
    assert_eq!(metadata.cloud_revision, first.cloud_revision);
    assert_eq!(metadata.cloud, first.cloud);
    assert_ne!(metadata.committed_map, first.committed_map);
    let receipt = now_monotonic_nanos();
    let ticket = owner_watchdog(handle.begin_surface_map_update(&metadata, receipt))
        .await
        .unwrap()
        .unwrap();
    let retained = cache.build(&metadata).unwrap();
    assert!(
        Arc::ptr_eq(&retained, &snapshot),
        "新提交必须真正复用同一个完整源"
    );
    assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
    assert!(owner_watchdog(handle.finish_surface_map_update(ticket, Ok(retained))).await);
    let refreshed = receiver.freeze().unwrap();
    assert!(original.same_source_view(&refreshed));
    assert_eq!(refreshed.notification(), original.notification() + 1);
    {
        let guard = handle.inner.lock().await;
        assert_eq!(guard.committed_map.as_ref(), Some(&metadata.committed_map));
        assert_eq!(guard.costmap_received_monotonic_ns, receipt);
        assert!(guard.logical_active);
    }
    assert!(sink.submitted.lock().unwrap().is_empty());
    prepare.release();
    commit.release();
    let result = owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let (captures, commits, discards, feedback) = {
        let audit = audit.lock().unwrap();
        (
            audit.captures.clone(),
            audit.commits.clone(),
            audit.discards,
            audit.feedback.clone(),
        )
    };
    let submitted = sink.submitted.lock().unwrap().clone();
    let cancelled = sink.cancelled.lock().unwrap().clone();
    // 此处捕获运行结果；cleanup 的 Reached 反馈不计入被审查的选择阶段。
    settle_surface_composite_owner_fixture(&handle, request_id).await;

    assert_eq!(
        (captures.len(), commits.len(), discards),
        (1, 1, 0),
        "同域新提交没有改变完整 Surface 输入，不应重做已完成的选择：{}",
        result.message,
    );
    assert!(
        feedback.is_empty(),
        "新提交不能使已接受候选被 Cancelled 退役：{feedback:?}"
    );
    assert_eq!(result.status, STATUS_ACTIVE, "{}", result.message);
    assert_eq!(submitted.len(), 1);
    assert!(cancelled.is_empty());
    assert_eq!(submitted[0].request_id, format!("{request_id}.nav.1"));
    assert_eq!(
        submitted[0].frontier_goal_id,
        surface_owner_goal(&commits[0]).id
    );
    assert_eq!(
        submitted[0].intent.committed_map, first.committed_map,
        "原计算的 GoalIntent 来源必须保留，不能伪装成使用新提交计算",
    );
    assert_ne!(submitted[0].intent.committed_map, metadata.committed_map);
    assert_surface_owner_capture(&captures[0], &snapshot, original.notification(), &pose);
}

#[tokio::test]
async fn compute_owner_surface_new_commit_same_view_before_accept_keeps_goal() {
    surface_owner_new_commit_same_view_preserves_goal(false).await;
}

#[tokio::test]
async fn compute_owner_surface_new_commit_same_view_after_accept_keeps_goal() {
    surface_owner_new_commit_same_view_preserves_goal(true).await;
}

#[derive(Clone, Copy, Debug)]
enum CompositeOwnerChangedSource {
    Mask,
    Observations,
    RecentObstacles,
}

async fn surface_owner_new_commit_changed_source_discards(change: CompositeOwnerChangedSource) {
    let (handle, sink) = configured_handle();
    let (first, _, snapshot, mut cache) = install_surface_composite_owner_fixture(&handle).await;
    let request_id = format!("owner-composite-reject-{change:?}");
    assert_eq!(
        handle.start(&request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let prepare = Arc::new(OwnerRuntimeGate::default());
    let _prepare_release = OwnerRuntimeGateRelease(prepare.clone());
    let receiver =
        install_gated_real_surface_source(&handle, audit.clone(), prepare.clone(), None).await;
    let original = receiver.freeze().unwrap();
    let before = owner_watchdog(handle.inspect_compute_checkpoint())
        .await
        .unwrap()[0]
        .payload
        .clone();
    let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
    let tick = tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    });
    owner_watchdog(prepare.entered()).await;
    assert_surface_owner_capture(
        &audit.lock().unwrap().captures[0],
        &snapshot,
        original.notification(),
        &pose,
    );
    let mut next = first.clone();
    advance_surface_composite_commit(&mut next);
    if matches!(change, CompositeOwnerChangedSource::Mask) {
        next.virtual_obstacles.as_mut().unwrap().data.fill(100);
    }
    let ticket = owner_watchdog(handle.begin_surface_map_update(&next, now_monotonic_nanos()))
        .await
        .unwrap()
        .unwrap();
    let built = cache.build(&next).unwrap();
    assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
    let replacement = match change {
        CompositeOwnerChangedSource::Mask => {
            assert!(!built.same_geometry(&snapshot));
            assert!(built.locate_support([1.5, 1.5, 0.0]).is_err());
            built
        }
        CompositeOwnerChangedSource::Observations => {
            assert!(Arc::ptr_eq(&built, &snapshot));
            let changed =
                Arc::new(built.with_observations(Some(surface_owner_observations(&next))));
            assert!(changed.same_geometry(&snapshot));
            assert!(snapshot.observations().is_none());
            assert!(changed.observations().is_some());
            changed
        }
        CompositeOwnerChangedSource::RecentObstacles => {
            assert!(Arc::ptr_eq(&built, &snapshot));
            let from = [pose.x, pose.y, pose.z];
            let to = [pose.x + 0.8, pose.y, pose.z];
            let changed = Arc::new(
                built
                    .with_recent_obstacle_points(
                        1,
                        next.committed_map.source_timestamp_ns as u64,
                        0,
                        &[],
                        [(0, [pose.x + 0.4, pose.y, pose.z + 0.2])],
                    )
                    .unwrap(),
            );
            assert!(changed.same_geometry(&snapshot));
            assert!(snapshot.validate_planar_recent_motion(from, to).is_ok());
            assert!(changed.validate_planar_recent_motion(from, to).is_err());
            changed
        }
    };
    // observations/recent 由独立源附加；此处验证 owner 资格，非声称测试了真实传感器接入。
    assert!(!replacement.same_body_query_view(&snapshot));
    assert!(owner_watchdog(handle.finish_surface_map_update(ticket, Ok(replacement))).await);
    assert!(!original.same_source_view(&receiver.freeze().unwrap()));
    assert_eq!(
        handle.inner.lock().await.committed_map.as_ref(),
        Some(&next.committed_map)
    );
    prepare.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let (captures, commits, discards, feedback) = {
        let audit = audit.lock().unwrap();
        (
            audit.captures.len(),
            audit.commits.len(),
            audit.discards,
            audit.feedback.clone(),
        )
    };
    let after = owner_watchdog(handle.inspect_compute_checkpoint())
        .await
        .unwrap()[0]
        .payload
        .clone();
    let submitted = sink.submitted.lock().unwrap().len();
    let cancelled = sink.cancelled.lock().unwrap().len();
    let (active, child_sequence, reached) = {
        let guard = handle.inner.lock().await;
        (
            guard.logical_active,
            guard.child_sequence,
            guard.reached_goals,
        )
    };
    settle_surface_composite_owner_fixture(&handle, &request_id).await;

    assert_eq!((captures, commits, discards), (1, 0, 1), "{change:?}");
    assert!(
        feedback.is_empty(),
        "未接受的旧候选不能得到失败或取消反馈：{feedback:?}"
    );
    assert_eq!(after, before, "变化输入不得污染源历史");
    assert_eq!(
        (submitted, cancelled, child_sequence, reached),
        (0, 0, 0, 0)
    );
    assert!(active, "同域输入变化不应直接终止探索会话");
}

#[tokio::test]
async fn compute_owner_surface_new_commit_changed_mask_discards_goal() {
    surface_owner_new_commit_changed_source_discards(CompositeOwnerChangedSource::Mask).await;
}

#[tokio::test]
async fn compute_owner_surface_new_commit_changed_observations_discards_goal() {
    surface_owner_new_commit_changed_source_discards(CompositeOwnerChangedSource::Observations)
        .await;
}

#[tokio::test]
async fn compute_owner_surface_new_commit_changed_recent_obstacles_discards_goal() {
    surface_owner_new_commit_changed_source_discards(CompositeOwnerChangedSource::RecentObstacles)
        .await;
}

async fn surface_owner_new_commit_changed_domain_stops(frame_only: bool) {
    let (handle, sink) = configured_handle();
    let (first, first_ticket, snapshot, mut cache) =
        install_surface_composite_owner_fixture(&handle).await;
    let request_id = if frame_only {
        "owner-composite-frame"
    } else {
        "owner-composite-session"
    };
    assert_eq!(
        handle.start(request_id, "prefer_wavefront").await.status,
        STATUS_ACCEPTED
    );
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let prepare = Arc::new(OwnerRuntimeGate::default());
    let _prepare_release = OwnerRuntimeGateRelease(prepare.clone());
    let receiver =
        install_gated_real_surface_source(&handle, audit.clone(), prepare.clone(), None).await;
    let original = receiver.freeze().unwrap();
    let pose = surface_pose(&snapshot, 0.0, Quaternion::IDENTITY);
    let tick = tokio::spawn({
        let handle = handle.clone();
        let pose = pose.clone();
        async move { handle.tick_with_pose(pose).await }
    });
    owner_watchdog(prepare.entered()).await;
    assert_surface_owner_capture(
        &audit.lock().unwrap().captures[0],
        &snapshot,
        original.notification(),
        &pose,
    );
    let mut next = first.clone();
    advance_surface_composite_commit(&mut next);
    // 域切换必须跨 cloud fence，不能伪造同 root 的坐标代际变化。
    next.cloud_revision += 1;
    next.cloud.header.seq = next.cloud_revision as i32;
    if frame_only {
        next.map_frame_generation += 1;
    } else {
        next.committed_map.session_id = "surface-composite-replacement".into();
        next.committed_map.session_generation += 1;
    }
    schema::validate_navigation_map_3d(&next).unwrap();
    let ticket = owner_watchdog(handle.begin_surface_map_update(&next, now_monotonic_nanos()))
        .await
        .unwrap()
        .unwrap();
    assert!(!handle.inner.lock().await.logical_active);
    assert!(receiver.snapshot().unwrap().is_none());
    assert!(
        !owner_watchdog(handle.finish_surface_map_update(first_ticket, Ok(snapshot.clone()))).await,
        "迟到旧域图不得恢复已撤销来源"
    );
    let changed = cache.build(&next).unwrap();
    assert!(owner_watchdog(handle.finish_surface_map_update(ticket, Ok(changed))).await);
    prepare.release();
    owner_watchdog(tick).await.unwrap();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    let (captures, commits, discards, feedback) = {
        let audit = audit.lock().unwrap();
        (
            audit.captures.len(),
            audit.commits.len(),
            audit.discards,
            audit.feedback.clone(),
        )
    };
    let (active, stop_reason, child_sequence) = {
        let guard = handle.inner.lock().await;
        (
            guard.logical_active,
            guard.last_stop_reason.clone(),
            guard.child_sequence,
        )
    };
    let submitted = sink.submitted.lock().unwrap().len();
    let cancelled = sink.cancelled.lock().unwrap().len();
    settle_surface_composite_owner_fixture(&handle, request_id).await;

    assert_eq!((captures, commits, discards), (1, 0, 1));
    assert!(feedback.is_empty());
    assert_eq!((submitted, cancelled, child_sequence), (0, 0, 0));
    assert!(!active);
    assert_eq!(stop_reason, "map_revision_changed");
}

#[tokio::test]
async fn compute_owner_surface_new_commit_changed_session_stops_goal() {
    surface_owner_new_commit_changed_domain_stops(false).await;
}

#[tokio::test]
async fn compute_owner_surface_new_commit_changed_frame_stops_goal() {
    surface_owner_new_commit_changed_domain_stops(true).await;
}
