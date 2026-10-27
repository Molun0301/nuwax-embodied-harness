// 验证停车未确认不会计为前沿失败，业务结束后原停车拥有方仍保留回收责任。
#[tokio::test]
async fn unconfirmed_child_stop_ends_business_without_retrying_or_blacklisting() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
    handle.start("unconfirmed-child-stop", "prefer_wavefront").await;
    let active = handle.tick((2.0, 2.0)).await;
    let lease = active_child_lease(&handle).await;
    let history = handle.inner.lock().await.compute.history;
    let mut wrong = terminal_proof(&lease);
    wrong.lease.goal_generation += 1;
    assert!(handle.report_navigation_stop_unconfirmed(&lease.session_id, wrong,
        "MOTION_STOP_UNCONFIRMED".into()).await.is_err());
    assert_eq!(handle.snapshot().child_task_id, active.child_task_id);
    assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
    let failed = handle.report_navigation_stop_unconfirmed(&lease.session_id,
        terminal_proof(&lease), "MOTION_STOP_UNCONFIRMED: 实际速度仍过高".into()).await.unwrap();
    assert_eq!(failed.status, STATUS_FAILED);
    assert!(failed.message.contains("MOTION_STOP_UNCONFIRMED"));
    assert!(failed.parking_pending);
    assert!(failed.child_task_id.is_empty());
    assert!(!handle.inner.lock().await.logical_active);
    assert!(handle.blocks_mapping_mutation().await);
    assert_eq!(handle.orphan_cancellations.retired_cancellation(&lease), Some(None));
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    assert!(!handle.is_active().await);
    assert_eq!(handle.inner.lock().await.compute.history, history,
        "未停车不得增加前沿反馈历史或黑名单");
    for _ in 0..3 {
        assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_FAILED);
    }
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    // 停车拥有方最终取得原租约的真实取消票据和停稳证明，业务失败保持不变。
    handle.orphan_cancellations.retain_failed_business(lease.clone(), Some(17));
    assert!(handle.report_navigation_result(&lease.session_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 18))).await.is_err());
    assert!(handle.blocks_mapping_mutation().await);
    handle.report_navigation_result(&lease.session_id,
        ChildNavigationResult::Cancelled(zero_proof(&lease, 17))).await.unwrap();
    assert!(!handle.blocks_mapping_mutation().await);
    assert_eq!(handle.snapshot().status, STATUS_FAILED);
    assert_eq!(handle.start("after-confirmed-stop", "prefer_wavefront").await.status, STATUS_ACCEPTED);
    assert!(!handle.tick((2.0, 2.0)).await.child_task_id.is_empty());
    assert_eq!(sink.submitted.lock().unwrap().len(), 2);
    stop_owner_runtime_child(&handle, "after-confirmed-stop").await;
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test]
async fn repeated_pending_and_source_refresh_cannot_restart_selection_deadline() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
    handle.start("selection-stalled", "prefer_wavefront").await;
    // 导航暂未就绪时已选中的请求必须保留，重试不能重新起算等候期限。
    sink.submit_errors.lock().unwrap().push_back(
        ChildNavigationSubmitError::RetryableReadiness("等待旧停车回收".into()));
    assert_eq!(handle.tick((2.0, 2.0)).await.status, STATUS_PENDING);
    let since = handle.inner.lock().await.selection_wait_since.unwrap();
    assert!(handle.inner.lock().await.pending_child_dispatch.is_some());
    ingest_committed_costmap_fixture(&handle, frontier_costmap(2), 2).await.unwrap();
    handle.record_pose_wait("短暂失鲜".into()).await;
    assert_eq!(handle.inner.lock().await.selection_wait_since, Some(since));
    handle.inner.lock().await.selection_wait_since = Some(Instant::now() - DEFAULT_SELECTION_PROGRESS_TIMEOUT);
    let failed = handle.tick((2.0, 2.0)).await;
    assert_eq!(failed.status, STATUS_FAILED);
    assert_eq!(failed.last_stop_reason, "planning_resource_limited");
    assert!(failed.message.contains("exploration_selection_stalled"));
    assert!(failed.message.contains("尚未确定"));
    assert!(!handle.is_active().await);
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn scheduler_stops_inflight_selection_at_the_same_progress_deadline() {
    let (handle, sink) = configured_handle();
    let (_, _, snapshot) = install_surface_fixture(&handle, "map").await;
    handle.start("selection-inflight-stalled", "prefer_wavefront").await;
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    install_gated_real_surface_source(&handle, Arc::new(StdMutex::new(SurfaceOwnerAudit::default())), gate.clone(), None).await;
    let provider = Arc::new(RecoveringPoseProvider {
        fresh: AtomicBool::new(true),
        pose: surface_pose(&snapshot, 0.0, Quaternion::IDENTITY),
    });
    let scheduler = handle.spawn_scheduler(provider, Duration::from_millis(5));
    owner_watchdog(gate.entered()).await;
    handle.inner.lock().await.selection_wait_since = Some(Instant::now() - DEFAULT_SELECTION_PROGRESS_TIMEOUT);
    owner_watchdog(async {
        while handle.snapshot().status != STATUS_FAILED {
            tokio::task::yield_now().await;
        }
    }).await;
    assert!(handle.snapshot().message.contains("exploration_selection_stalled"));
    gate.release();
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    assert!(sink.submitted.lock().unwrap().is_empty());
    owner_watchdog(scheduler.shutdown()).await.unwrap();
}

#[tokio::test]
async fn endless_source_pending_expires_but_a_dispatched_child_clears_the_wait() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
    handle.start("source-pending-stalled", "prefer_wavefront").await;
    let mut source = OwnerRuntimeSource::new(Arc::new(StdMutex::new(Vec::new())));
    source.selected = Some("source-work-never-finishes".into());
    install_owner_runtime_source(&handle, source).await;
    for _ in 0..3 {
        let pending = handle.tick((2.0, 2.0)).await;
        assert_eq!(pending.status, STATUS_PENDING);
        assert!(pending.message.contains("awaiting work completion"));
    }
    assert!(handle.inner.lock().await.no_frontier_since.is_none());
    handle.inner.lock().await.selection_wait_since = Some(Instant::now() - DEFAULT_SELECTION_PROGRESS_TIMEOUT);
    let failed = handle.tick((2.0, 2.0)).await;
    assert_eq!(failed.status, STATUS_FAILED);
    assert!(failed.message.contains("exploration_selection_stalled"));
    assert!(sink.submitted.lock().unwrap().is_empty());
    owner_watchdog(handle.await_compute_idle()).await.unwrap();
    handle.start("source-pending-recovered", "prefer_wavefront").await;
    let active = handle.tick((2.0, 2.0)).await;
    assert_eq!(active.status, STATUS_ACTIVE);
    assert!(handle.inner.lock().await.selection_wait_since.is_none());
    handle.refresh_compute_status(DEFAULT_SELECTION_PROGRESS_TIMEOUT * 2).await;
    assert_eq!(handle.snapshot().status, STATUS_ACTIVE);
    stop_owner_runtime_child(&handle, "source-pending-recovered").await;
    handle.request_compute_shutdown().await;
    owner_watchdog(handle.compute_service.join()).await.unwrap();
}
