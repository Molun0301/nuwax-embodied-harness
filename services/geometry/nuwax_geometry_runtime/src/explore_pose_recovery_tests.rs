// 使用真实表面选择器和受控输入时序，验证计算、等待与派发的同一份决策。
use std::sync::atomic::AtomicBool;

struct RecoveringPoseProvider {
    fresh: AtomicBool,
    pose: ExploreRobotPose,
}

#[async_trait]
impl ExplorePoseProvider for RecoveringPoseProvider {
    async fn current_pose(&self) -> Result<ExploreRobotPose, String> {
        if !self.fresh.load(Ordering::Acquire) {
            return Err("测试位姿源持续过期".into());
        }
        let mut pose = self.pose.clone();
        pose.source_timestamp_ns = now_unix_nanos();
        pose.received_monotonic_ns = now_monotonic_nanos();
        Ok(pose)
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn scheduler_refreshes_slow_compute_and_dispatches_retained_goal_after_pose_recovery() {
    let (handle, sink) = configured_handle();
    let (_, _, snapshot) = install_surface_fixture(&handle, "map").await;
    let request_id = "pose-recovery-retained";
    handle.start(request_id, "prefer_wavefront").await;
    let audit = Arc::new(StdMutex::new(SurfaceOwnerAudit::default()));
    let gate = Arc::new(OwnerRuntimeGate::default());
    let _release = OwnerRuntimeGateRelease(gate.clone());
    install_gated_real_surface_source(&handle, audit.clone(), gate.clone(), None).await;
    let provider = Arc::new(RecoveringPoseProvider {
        fresh: AtomicBool::new(true),
        pose: surface_pose(&snapshot, 0.0, Quaternion::IDENTITY),
    });
    let scheduler = handle.spawn_scheduler(provider.clone(), Duration::from_millis(5));
    owner_watchdog(gate.entered()).await;
    tokio::time::sleep(Duration::from_millis(120)).await;
    let first = handle.snapshot();
    assert!(first.message.contains("正在评估探索前沿"), "{}", first.message);
    tokio::time::sleep(Duration::from_millis(120)).await;
    assert_ne!(handle.snapshot().message, first.message);
    provider.fresh.store(false, Ordering::Release);
    gate.release();
    let retained = owner_watchdog(async {
        loop {
            if let Some(request) = handle.inner.lock().await.pending_child_dispatch.clone() { break request; }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await;
    owner_watchdog(async {
        while !handle.snapshot().message.contains("目标决策已保留") {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await;
    let waiting = handle.snapshot();
    assert!(sink.submitted.lock().unwrap().is_empty());
    tokio::time::sleep(Duration::from_millis(230)).await;
    assert_ne!(handle.snapshot().message, waiting.message);
    assert_eq!(handle.inner.lock().await.pending_child_dispatch.as_ref(), Some(&retained));
    {
        let audit = audit.lock().unwrap();
        assert_eq!((audit.captures.len(), audit.commits.len()), (1, 1));
    }
    provider.fresh.store(true, Ordering::Release);
    owner_watchdog(async {
        while handle.snapshot().child_task_id.is_empty() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await;
    assert_eq!(sink.submitted.lock().unwrap().len(), 1);
    assert_eq!(sink.submitted.lock().unwrap()[0], retained);
    assert_eq!(audit.lock().unwrap().captures.len(), 1);
    handle.stop(request_id).await;
    scheduler.shutdown().await.unwrap();
}

#[tokio::test]
async fn persistent_stale_pose_refreshes_status_and_ends_without_recomputing() {
    let (handle, sink) = configured_handle();
    ingest_committed_costmap_fixture(&handle, frontier_costmap(1), 1).await.unwrap();
    handle.start("pose-timeout", "prefer_wavefront").await;
    let first = handle.record_pose_wait("同一源时间戳过期".into()).await;
    assert!(first.message.contains("等待新鲜输入"));
    assert!(!first.message.contains("目标决策已保留"));
    handle.inner.lock().await.pose_wait_since = Some(Instant::now() - Duration::from_millis(250));
    let updated = handle.record_pose_wait("同一源时间戳过期".into()).await;
    assert_ne!(updated.message, first.message);
    handle.inner.lock().await.pose_wait_since = Some(Instant::now() - DEFAULT_POSE_RECOVERY_TIMEOUT);
    let expired = handle.record_pose_wait("同一源时间戳过期".into()).await;
    assert_eq!(expired.status, STATUS_FAILED);
    assert!(expired.message.contains("等待已到期限"));
    assert!(!handle.is_active().await);
    assert!(sink.submitted.lock().unwrap().is_empty());
    handle.request_compute_shutdown().await;
    handle.compute_service.join().await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn retained_goal_is_not_dispatched_after_map_revocation_while_pose_waits() {
    let (handle, sink) = configured_handle();
    let (message, _, snapshot) = install_surface_fixture(&handle, "map").await;
    handle.start("pose-map-revoked", "prefer_wavefront").await;
    let provider = Arc::new(RecoveringPoseProvider {
        fresh: AtomicBool::new(false),
        pose: surface_pose(&snapshot, 0.0, Quaternion::IDENTITY),
    });
    let tick = tokio::spawn({
        let handle = handle.clone(); let provider = provider.clone();
        async move { handle.tick_generation(handle.generation.load(Ordering::SeqCst), provider.pose.clone(), Some(provider.as_ref())).await }
    });
    owner_watchdog(async {
        while handle.inner.lock().await.pending_child_dispatch.is_none() {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await;
    let mut clear = message;
    clear.cleared = true;
    clear.cloud_revision = 2;
    clear.source_timestamp_ns = 0;
    clear.commit_sequence = 0;
    clear.committed_map.revision = 0;
    clear.committed_map.commit_boot_nonce = [0; 16];
    clear.committed_map.commit_sequence = 0;
    clear.committed_map.source_timestamp_ns = 0;
    clear.cloud_occupancy_revision = 0;
    clear.cloud = schema::PointCloud2 { header: schema::Header { seq: 2, frame_id: "map".into(), ..Default::default() }, ..Default::default() };
    handle.begin_surface_map_update(&clear, now_monotonic_nanos()).await.unwrap();
    provider.fresh.store(true, Ordering::Release);
    owner_watchdog(tick).await.unwrap();
    assert!(sink.submitted.lock().unwrap().is_empty());
    assert_eq!(handle.snapshot().last_stop_reason, "map_revision_changed");
    handle.request_compute_shutdown().await;
    handle.compute_service.join().await.unwrap();
}
