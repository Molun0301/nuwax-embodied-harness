#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn oversized_ground_block_keeps_runtime_alive_and_new_session_recovers() {
    let mut handle = build_handle_for_tests_without_cloud_writer().await.unwrap();
    let mut fixture = NavigationMap3DFixture::new();
    fixture.context.producer_runtime_generation = handle.nav.producer_runtime_generation();
    let baseline = fixture.commit(1_000_000_000);
    let old_root = fixture.writer.lock().as_ref().unwrap().committed_fine_cloud().unwrap();
    let old_bytes = old_root.repository.read_object(old_root.manifest().blocks[0].object,
        nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap();
    let backend = MappingFeedRecordingBackend::new(LcmTopic::new("/unused/block-failure").unwrap());
    handle.transport = LcmTransport::new(Arc::new(backend.clone()));
    handle.mapping_adapter = Some(fixture.context.mapping.clone());
    handle.cloud_map_writer = fixture.writer.clone();
    handle.state = fixture.state.clone();
    handle.committed_map_authority = fixture.context.committed_map_authority.clone();
    let (control, registrations) = GeometryRuntimeTaskControl::channel();
    let mut owner = GeometryRuntimeTaskOwner::start(control.clone(), registrations);
    handle.task_control = Some(control.clone());
    // 真实生产配置的探索来源按同一持久运行代际和提交时钟绑定。
    if handle.nav.configured_profile() == crate::nav_runtime::PROFILE_MLS3D {
        handle.explore.configure_surface_source(Ok(nuwax_nav_stack::surface_config::SurfaceNavigationConfig::new(
            "map".into(),0.5,0.2,0.6,0.1,0.15,18.0,0.2).unwrap()),
            handle.nav.producer_runtime_generation(), fixture.context.mapping.lock().mapping_commit_boot_nonce().to_be_bytes()).await.unwrap();
    }
    fixture.writer.lock().as_mut().unwrap().stage_oversized_ground_block(2_000_000_000);
    let mapping = handle.mapping_adapter.as_ref().unwrap();
    let outcome = ingest_mapping_points_with_backend(mapping, &handle.mapping_lifecycle_lock,
        &handle.explore_projection_gate, &handle.committed_map_authority, &handle.map_revision_latest_tx,
        &handle.cloud_map_writer, &[Point3::new(1.5,0.,0.)], 12, 3_000_000_000, "map", 3_000_000_000,
        &handle.state, MappingCloudSource::Legacy, None, None, None, Some(&control), None, None,
        Some(handle.clone())).await;
    assert!(matches!(outcome, Err(nuwax_mapping::SessionError::MapSourceUnavailable { .. })), "{:?}", outcome.as_ref().err());
    assert!(!mapping.lock().is_active());
    assert!(handle.committed_map_authority.lock().unwrap().is_none());
    assert!(handle.map_revision_latest_tx.borrow().is_none());
    let fence = {
        let guard = fixture.writer.lock(); let writer = guard.as_ref().unwrap();
        assert_eq!(writer.committed_metadata().unwrap().revision, baseline.metadata.revision);
        assert_eq!(writer.last_cloud_receipt().unwrap(), baseline.receipt);
        assert_eq!(writer.committed_fine_cloud().unwrap().manifest().identity(), old_root.manifest().identity());
        assert!(writer.source_failure().unwrap().1.contains("byte budget"));
        assert_eq!(writer.block_write_failure().unwrap().key, [10,10,0]);
        writer.source_failure().unwrap().0
    };
    assert_eq!(old_root.repository.read_object(old_root.manifest().blocks[0].object,
        nuwax_map_chunks::MAX_BLOCK_BYTES).unwrap(), old_bytes);
    assert!(latest_canonical_cloud_publication(&fixture.writer).unwrap().is_none(), "失败后不再重发旧图");
    let frames = backend.published.lock().unwrap().clone();
    let frame = frames.iter().find(|f|f.topic.channel()=="/nav/map3d").expect("真实来源撤销消息");
    let message = schema::decode_navigation_map_3d_body(&frame.payload).unwrap();
    assert!(message.cleared && message.ground_block_failed);
    assert_eq!(message.cloud_revision as u64, fence);
    assert_eq!(message.source_timestamp_ns, 0, "撤销不能伪造新观测");
    assert!(schema::navigation_map_clear_reason(&message).contains("mapping_ground_block_write_failed"));
    // 后续扫描不重试失败块；监督器仍接纳工作，关闭也能正常收尾。
    for _ in 0..3 {
        let result = ingest_mapping_points_with_backend(mapping, &handle.mapping_lifecycle_lock,
            &handle.explore_projection_gate, &handle.committed_map_authority, &handle.map_revision_latest_tx,
            &handle.cloud_map_writer, &[Point3::new(2.,0.,0.)],12,4_000_000_000,"map",4_000_000_000,
            &handle.state, MappingCloudSource::Legacy,None,None,None,Some(&control),None,None,Some(handle.clone())).await.unwrap();
        assert!(result.cloud_commit.is_none());
    }
    assert_eq!(backend.published.lock().unwrap().len(), frames.len());
    assert!(owner.completed_failure().is_none());
    let generation = mapping.lock().start_session("recovered-session", "test-map").unwrap().session_generation;
    let clear_revision = fixture.writer.lock().as_mut().unwrap().begin_session(generation).unwrap().0;
    assert!(clear_revision > fence);
    let result = ingest_mapping_points_with_backend(mapping, &handle.mapping_lifecycle_lock,
        &handle.explore_projection_gate, &handle.committed_map_authority, &handle.map_revision_latest_tx,
        &handle.cloud_map_writer, &[Point3::new(0.5,0.,0.)],12,5_000_000_000,"map",5_000_000_000,
        &handle.state, MappingCloudSource::Legacy,None,None,None,Some(&control),None,None,Some(handle.clone())).await.unwrap();
    assert!(result.cloud_commit.is_some());
    assert!(fixture.writer.lock().as_ref().unwrap().source_failure().is_none());
    let retirement = mapping.lock().stop_session_detached("recovered-session").unwrap().retirement;
    finish_backend_retirement(mapping,&handle.mapping_lifecycle_lock,retirement).await.unwrap();
    let stopped = owner.shutdown().await;
    assert!(stopped.producers_settled()); assert!(stopped.result.is_ok(), "{:?}", stopped.result);
}
