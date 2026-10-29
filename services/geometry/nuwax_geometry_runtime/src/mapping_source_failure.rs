//! 原受管建图任务内发布块失败撤销；不创建第二个后台拥有方。
use super::*;

pub(super) async fn notify(handle: &GeometryRuntimeHandle, generation: u64) -> Result<(), String> {
    let mapping = handle.mapping_adapter.as_ref().ok_or("mapping failure has no owner")?;
    let message = {
        let _projection = handle.explore_projection_gate.lock().await;
        let _lifecycle = handle.mapping_lifecycle_lock.lock().await;
        let (session, frame_id, frame_generation, nonce) = {
            let adapter = mapping.lock();
            (adapter.snapshot(), adapter.configured_map_frame_id().to_owned(),
                adapter.global_frame_generation(), adapter.mapping_commit_boot_nonce())
        };
        // 退休完成后，新会话可能先取得生命周期锁；旧失败不得撤销新来源。
        if session.session_generation != generation { return Ok(()); }
        if session.active { return Err("failed mapping session is still active".into()); }
        let (revision, voxel_size, reason) = {
            let guard = handle.cloud_map_writer.lock();
            let writer = guard.as_ref().ok_or("failed mapping cloud owner disappeared")?;
            let (fence, reason) = writer.source_failure().ok_or("mapping failure fence is absent")?;
            let metadata = writer.committed_metadata().ok_or("failed mapping snapshot is absent")?;
            (fence, metadata.voxel_size, reason.to_owned())
        };
        tracing::error!(session_generation = generation, cloud_revision = revision, %reason,
            "地面块提交失败：保留旧快照，锁存会话失败并撤销导航来源");
        let message = schema::NavigationMap3D {
            producer_runtime_generation: handle.nav.producer_runtime_generation() as i64,
            committed_map: schema::CommittedMapIdentity {
                session_id: session.session_id, session_generation: generation as i64,
                frame_id: frame_id.clone(), ..Default::default()
            },
            map_frame_generation: frame_generation as i64,
            cloud_revision: revision as i64, source_voxel_size: voxel_size,
            commit_boot_nonce: nonce.to_be_bytes(), cleared: true, ground_block_failed: true,
            cloud: schema::PointCloud2 {
                header: schema::Header { seq: revision as i32, frame_id, ..Default::default() },
                ..Default::default()
            },
            ..Default::default()
        };
        let payload = schema::encode_navigation_map_3d_body(&message).map_err(|e| e.to_string())?;
        let frame = crate::typed_codec::build_typed_frame(
            LcmTopic::new("/nav/map3d").map_err(|e| e.to_string())?,
            LcmMessageType::new("nuwax_msgs.NavigationMap3D").map_err(|e| e.to_string())?, payload, revision);
        tokio::time::timeout(MAP_FANOUT_SINK_TIMEOUT, handle.transport.publish(frame)).await
            .map_err(|_| "mapping source failure notification timed out".to_owned())?
            .map_err(|e| e.to_string())?;
        message
    };
    if handle.nav.configured_profile() == crate::nav_runtime::PROFILE_MLS3D {
        // 原探索拥有方负责结束业务和关联停车，不制造停车回执。
        if let Err(error) = handle.explore.begin_surface_map_update_with_retained_source(
            &message, crate::explore_runtime::now_monotonic_nanos(), None).await {
            if mapping.lock().snapshot().session_generation == generation { return Err(error); }
        }
    }
    Ok(())
}
