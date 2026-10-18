// Tests-only actual-entry qualification. Included inside explore_runtime::tests.
// Expected RED on 09999631. No synthetic build error, manual snapshot or fake RPC.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_map_missing_real_object_keeps_installed_source_and_child() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, sink) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.expect("real map input owner");
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (old_source, old_identity, old_receipt) = {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            assert!(mode.input_ready);
            (mode.receiver.snapshot().unwrap().unwrap(), guard.committed_map.clone(), guard.costmap_received_monotonic_ns)
        };
        let start = handle.start("f1-real-incomplete", "prefer_wavefront").await;
        assert_eq!(start.status, STATUS_ACCEPTED, "{}", start.message);
        let active = handle.tick_with_pose(surface_pose(&old_source, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let old_lease = active_child_lease(&handle).await;
        let old_generation = handle.inner.lock().await.generation;
        assert_eq!(sink.submitted.lock().unwrap().len(), 1);
        let next = input.publish_partial().await.unwrap();
        input.wait_finished(next.cloud_revision).await.unwrap();
        // Capture BEFORE shutdown: shutdown correctly clears this input itself.
        let (ready, current, identity, receipt, logical_active, generation, lease, pending, input_error) = {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            (mode.input_ready, mode.receiver.snapshot().unwrap(), guard.committed_map.clone(),
             guard.costmap_received_monotonic_ns, guard.logical_active, guard.generation,
             guard.active_child.as_ref().map(|child| child.lease.clone()),
             guard.pending_child_cancellation.as_ref().and_then(|p| p.ticket.clone()), mode.input_error.clone())
        };
        let cancelled = sink.cancelled.lock().unwrap().clone();
        eprintln!("F1 actual-entry observation: ready={ready}, same_source={}, identity={identity:?}, receipt={receipt}/{old_receipt}, active={logical_active}, generation={generation}/{old_generation}, old_lease={old_lease:?}, lease={lease:?}, pending={pending:?}, cancelled={cancelled:?}, input_error={input_error:?}",
            current.as_ref().is_some_and(|source| Arc::ptr_eq(source, &old_source)));
        // These are the intended contract assertions; current code fails here.
        assert!(ready, "real missing-object RPC revoked the complete source: {input_error:?}");
        assert!(current.as_ref().is_some_and(|source| Arc::ptr_eq(source, &old_source)));
        assert_eq!(identity, old_identity);
        assert_eq!(receipt, old_receipt, "retry cannot renew original map receipt TTL");
        assert!(logical_active);
        assert_eq!(generation, old_generation);
        assert_eq!(lease.as_ref(), Some(&old_lease));
        assert!(pending.is_none());
        assert!(cancelled.is_empty());
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-real-incomplete").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}

async fn f1_finish_all_owners(
    input: &mut crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture,
    handle: &ExploreRuntimeHandle,
    parent: &str,
) {
    let geometry = input.shutdown().await;
    let stop = scheme_a_race_wait("F1 Stop", handle.stop(parent)).await;
    let cancel = scheme_a_race_wait("F1 cancellation ticket", async {
        handle.inner.lock().await.pending_child_cancellation.as_ref().and_then(|p| p.ticket.clone())
    }).await;
    let report = match cancel.as_ref().ok().and_then(|ticket| ticket.as_ref()) {
        Some(ticket) => Some(scheme_a_race_wait("F1 exact ZEROED", handle.report_navigation_result(
            &ticket.lease.session_id,
            ChildNavigationResult::Cancelled(zero_proof(&ticket.lease, ticket.cancel_generation)),
        )).await),
        None => None,
    };
    let idle = tokio::time::timeout(Duration::from_secs(8), handle.await_compute_idle()).await;
    let shutdown = scheme_a_race_wait("F1 compute shutdown", handle.request_compute_shutdown()).await;
    let joined = tokio::time::timeout(Duration::from_secs(8), handle.compute_service.join()).await;
    eprintln!("F1 all-owner cleanup: geometry={geometry:?}, stop={stop:?}, cancel={cancel:?}, report={report:?}, idle={idle:?}, shutdown={shutdown:?}, joined={joined:?}");
    assert!(geometry.is_ok()); assert!(stop.is_ok()); assert!(cancel.is_ok());
    assert!(matches!(report, None | Some(Ok(Ok(_)))));
    assert!(matches!(idle, Ok(Ok(())))); assert!(shutdown.is_ok());
    assert!(matches!(joined, Ok(Ok(()))));
}

async fn f1_wait_latest(handle: &ExploreRuntimeHandle, expected: &schema::NavigationMap3D) {
    scheme_a_race_wait("F1 actual ingress admission", async {
        loop {
            let accepted = handle.inner.lock().await.surface_mode.as_ref().and_then(|mode| mode.latest.as_ref())
                .is_some_and(|ticket| ticket.cloud_revision == expected.cloud_revision
                    && ticket.committed_map == expected.committed_map);
            if accepted { break; }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }).await.unwrap();
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_newer_inline_root_does_not_inherit_old_backoff() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    for during_inflight in [true, false] {
        let (handle, _) = configured_handle();
        let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
        let result = std::panic::AssertUnwindSafe(async {
            input.wait_finished(input.first.cloud_revision).await.unwrap();
            let old_receipt = handle.inner.lock().await.costmap_received_monotonic_ns;
            let partial = input.publish_partial().await.unwrap();
            f1_wait_latest(&handle, &partial).await;
            if !during_inflight { input.wait_finished(partial.cloud_revision).await.unwrap(); }
            let next = input.prepare_complete_candidate().await.unwrap();
            // No provider recovery claim: this higher root is all authentic inline.
            if during_inflight {
                assert!(!input.probe.events().contains(&format!("surface-map-finish-cloud:{}", partial.cloud_revision)),
                    "fixture did not establish a newer pending root before old completion");
            }
            input.publish_candidate(&next).await.unwrap();
            f1_wait_latest(&handle, &next).await;
            if during_inflight {
                assert!(!input.probe.events().contains(&format!("surface-map-finish-cloud:{}", partial.cloud_revision)),
                    "fixture did not admit the newer root before old completion");
                input.wait_finished(partial.cloud_revision).await.unwrap();
            }
            // This is a test latency discriminator against the exact old 1 s
            // backoff, not a production TTL. Same-geometry complete build is warm.
            tokio::time::timeout(Duration::from_millis(750), input.wait_finished(next.cloud_revision))
                .await.expect("new root waited for an older candidate's retry backoff").unwrap();
            let guard = handle.inner.lock().await;
            assert_eq!(guard.committed_map.as_ref(), Some(&next.committed_map));
            assert!(guard.costmap_received_monotonic_ns >= old_receipt);
            assert!(guard.surface_mode.as_ref().unwrap().input_ready);
            assert!(guard.pending_child_cancellation.is_none());
        }).catch_unwind().await;
        f1_finish_all_owners(&mut input, &handle, "f1-newer-inline").await;
        if let Err(panic) = result { std::panic::resume_unwind(panic); }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_retries_preserve_receipt_and_require_actual_installed_basis() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, _) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let result = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (installed, receipt, identity) = {
            let guard = handle.inner.lock().await;
            (guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(),
                guard.costmap_received_monotonic_ns, guard.committed_map.clone())
        };
        // Real codec input through a second real cache, not Arc::new(snapshot).
        // This is a qualification-boundary test, not a claim that the single
        // production consumer necessarily creates this ownership mismatch.
        let mut other_cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&surface_config("map"));
        let other_source = other_cache.build_typed(&input.first).unwrap();
        assert!(!Arc::ptr_eq(&installed, &other_source));
        let partial = input.publish_partial().await.unwrap();
        f1_wait_latest(&handle, &partial).await;
        let ticket = handle.inner.lock().await.surface_mode.as_ref().unwrap().latest.as_ref().unwrap().clone();
        assert!(handle.can_defer_incomplete_surface_map(&ticket, &installed).await);
        assert!(!handle.can_defer_incomplete_surface_map(&ticket, &other_source).await);
        input.wait_finish_count(partial.cloud_revision, 2).await.unwrap();
        // Completion counts do not establish elapsed TTL. Wait only the real
        // remainder of the original receipt deadline; do not alter either clock.
        scheme_a_race_wait("F1 original receipt TTL naturally expires", async {
            let ttl_ns = DEFAULT_COSTMAP_RECEIPT_TTL.as_nanos() as u64;
            loop {
                let age = now_monotonic_nanos().saturating_sub(receipt);
                if age > ttl_ns { break; }
                tokio::time::sleep(Duration::from_nanos(ttl_ns.saturating_sub(age).saturating_add(1))).await;
            }
        }).await.unwrap();
        {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.costmap_received_monotonic_ns, receipt);
            assert_eq!(guard.committed_map, identity);
            assert!(Arc::ptr_eq(&guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(), &installed));
            assert!(now_monotonic_nanos().saturating_sub(receipt) > DEFAULT_COSTMAP_RECEIPT_TTL.as_nanos() as u64);
        }
        let start = handle.start("f1-stale-retry", "prefer_wavefront").await;
        assert_eq!(start.status, STATUS_ACCEPTED, "{}", start.message);
        let waiting = handle.tick_with_pose(surface_pose(&installed, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(waiting.status, STATUS_PENDING, "{}", waiting.message);
        assert!(waiting.child_task_id.is_empty());
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-stale-retry").await;
    if let Err(panic) = result { std::panic::resume_unwind(panic); }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_late_completion_cannot_cross_real_clear_or_domain() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    for publish_clear in [true, false] {
        let (handle, _) = configured_handle();
        let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
        let result = std::panic::AssertUnwindSafe(async {
            input.wait_finished(input.first.cloud_revision).await.unwrap();
            let partial = input.publish_partial().await.unwrap();
            f1_wait_latest(&handle, &partial).await;
            let old_ticket = handle.inner.lock().await.surface_mode.as_ref().unwrap().latest.as_ref().unwrap().clone();
            let old_source = handle.inner.lock().await.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap();
            let next = input.switch_mapping_domain(publish_clear).await.unwrap();
            assert_eq!(next.cleared, publish_clear);
            f1_wait_latest(&handle, &next).await;
            assert!(!input.probe.events().contains(&format!("surface-map-finish-cloud:{}", partial.cloud_revision)),
                "fixture did not admit clear/new domain before old completion");
            assert!(!handle.can_defer_incomplete_surface_map(&old_ticket, &old_source).await);
            input.wait_finished(partial.cloud_revision).await.unwrap();
            if !publish_clear { input.wait_finished(next.cloud_revision).await.unwrap(); }
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            if publish_clear {
                assert!(mode.receiver.snapshot().unwrap().is_none());
                assert!(guard.committed_map.is_none());
                assert_eq!(guard.costmap_received_monotonic_ns, 0);
            } else { assert_eq!(guard.committed_map.as_ref(), Some(&next.committed_map)); }
            assert!(mode.latest.as_ref().is_some_and(|ticket| ticket.committed_map == next.committed_map));
        }).catch_unwind().await;
        f1_finish_all_owners(&mut input, &handle, "f1-domain-fence").await;
        if let Err(panic) = result { std::panic::resume_unwind(panic); }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_changed_mask_and_real_receiver_invalid_still_stop_original_child() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    use nuwax_nav_stack::surface_map::SurfaceMapBuildError;
    for invalid_progression in [false, true] {
        let (handle, sink) = configured_handle();
        let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
        let result = std::panic::AssertUnwindSafe(async {
            input.wait_finished(input.first.cloud_revision).await.unwrap();
            let source = handle.inner.lock().await.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap();
            let started = handle.start("f1-fatal-boundary", "prefer_wavefront").await;
            assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
            let active = handle.tick_with_pose(surface_pose(&source, 0.0, Quaternion::IDENTITY)).await;
            assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
            let original_lease = active_child_lease(&handle).await;
            let mut candidate = input.prepare_complete_candidate().await.unwrap();
            let first = nuwax_map_chunks::Manifest::decode(&input.first.fine.as_ref().unwrap().root).unwrap();
            let mut next = nuwax_map_chunks::Manifest::decode(&candidate.fine.as_ref().unwrap().root).unwrap();
            let changed = next.blocks.iter_mut().find(|new| first.blocks.iter().any(|old|
                old.key == new.key && old.object != new.object)).expect("real new observation object");
            if invalid_progression {
                let previous = first.blocks.iter().find(|old| old.key == changed.key).unwrap();
                assert_eq!(changed.geometry_revision, previous.geometry_revision,
                    "fixture must repeat geometry before testing evidence revision corruption");
                changed.block_revision = previous.block_revision;
                candidate.fine.as_mut().unwrap().root = next.encode().unwrap();
                // Structural schema admission still succeeds. The actual Receiver
                // must reject changed-object evidence rollback against its old root.
                schema::validate_navigation_map_3d(&candidate).unwrap();
                let mut control = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&surface_config("map"));
                control.build_typed(&input.first).unwrap();
                assert!(matches!(control.build_typed(&candidate), Err(SurfaceMapBuildError::Invalid(_))));
            } else {
                candidate.fine.as_mut().unwrap().included.remove(&changed.object);
                // Deliberate protocol-level mask mutation; real fine root/objects
                // are retained. This does not claim a zones-store publisher test.
                if let Some(mask) = candidate.virtual_obstacles.as_mut() {
                    let first_cell = mask.data.first_mut().unwrap();
                    *first_cell = if *first_cell == 100 { 0 } else { 100 };
                } else {
                    let stamp = candidate.committed_map.source_timestamp_ns;
                    candidate.virtual_obstacles = Some(schema::OccupancyGrid {
                        header: schema::Header { seq: candidate.committed_map.revision as i32,
                            stamp_sec: (stamp / 1_000_000_000) as i32,
                            stamp_nsec: (stamp % 1_000_000_000) as i32,
                            frame_id: candidate.committed_map.frame_id.clone() },
                        info: schema::MapMetaData { resolution: 0.25, width: 1, height: 1,
                            origin: schema::Pose { position: [10.0, 10.0, 0.0], orientation_xyzw: [0.0, 0.0, 0.0, 1.0] },
                            ..schema::MapMetaData::default() }, data: vec![100],
                    });
                }
                schema::validate_navigation_map_3d(&candidate).unwrap();
            }
            input.publish_candidate(&candidate).await.unwrap();
            input.wait_finished(candidate.cloud_revision).await.unwrap();
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            assert!(!mode.input_ready);
            assert!(mode.receiver.snapshot().unwrap().is_none());
            assert!(guard.committed_map.is_none());
            assert_eq!(guard.costmap_received_monotonic_ns, 0);
            assert!(guard.active_child.as_ref().is_some_and(|child| child.lease == original_lease),
                "original lease remains until correlated ZEROED, never replaced");
            assert!(guard.pending_child_cancellation.as_ref().and_then(|pending| pending.ticket.as_ref())
                .is_some_and(|ticket| ticket.lease == original_lease));
            assert_eq!(*sink.cancelled.lock().unwrap(), vec![original_lease.session_id]);
        }).catch_unwind().await;
        f1_finish_all_owners(&mut input, &handle, "f1-fatal-boundary").await;
        if let Err(panic) = result { std::panic::resume_unwind(panic); }
    }
}
