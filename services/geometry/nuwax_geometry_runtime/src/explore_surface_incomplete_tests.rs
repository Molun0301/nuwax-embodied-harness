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

// Append to the existing tests-only explore_surface_incomplete_tests.rs.
// NOT COMPILED/RUN. Uses real consumer + cache + FineMapReader + isolated memq
// no-provider failure. Test clocks merely record actual events; no fake time,
// shortened retry, injected build error, or sleep-based arrival assumption.

fn f1_surface_clock_events(
    probe: &crate::lcm_runtime::GeometryTopologyTestControl,
    kind: &str,
) -> Vec<(i64, u128)> {
    let prefix = format!("surface-clock:{kind}:");
    probe.events().into_iter().filter_map(|event| {
        let rest = event.strip_prefix(&prefix)?;
        let (revision, stamp) = rest.split_once(':')?;
        Some((revision.parse().ok()?, stamp.parse().ok()?))
    }).collect()
}

async fn f1_wait_actual_surface_event(
    input: &crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture,
    kind: &str,
    revision: i64,
) -> u128 {
    tokio::time::timeout(Duration::from_secs(8), async {
        loop {
            if let Some((_, at)) = f1_surface_clock_events(&input.probe, kind)
                .into_iter().find(|(observed, _)| *observed == revision) {
                return at;
            }
            // Only poll recorded actual owner/worker events. Sleep does not
            // establish arrival, build start, or deadline expiry.
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }).await.unwrap_or_else(|_| panic!("real surface {kind} event missing for {revision}"))
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_watermark_burst_keeps_missing_object_deadline() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, _) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (source, receipt, identity) = {
            let guard = handle.inner.lock().await;
            (guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(),
                guard.costmap_received_monotonic_ns, guard.committed_map.clone())
        };
        // All roots/bodies are constructed before the failed candidate is sent.
        let (partial, burst) = input.prepare_watermark_retry_series(3, false).await.unwrap();
        input.publish_candidate(&partial).await.unwrap();
        f1_wait_latest(&handle, &partial).await;
        input.wait_finished(partial.cloud_revision).await.unwrap();
        let deadlines: Vec<_> = f1_surface_clock_events(&input.probe, "backoff")
            .into_iter().filter(|(revision, _)| *revision == partial.cloud_revision).collect();
        assert_eq!(deadlines.len(), 1, "must capture the first real retry assignment");
        let original_deadline = deadlines[0].1;
        {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            assert!(mode.input_ready, "initial incomplete must qualify for retained source");
            assert!(Arc::ptr_eq(&mode.receiver.snapshot().unwrap().unwrap(), &source));
            assert_eq!(guard.costmap_received_monotonic_ns, receipt);
            assert_eq!(guard.committed_map, identity);
        }
        let before_starts = f1_surface_clock_events(&input.probe, "build").len();
        for message in &burst {
            input.publish_candidate(message).await.unwrap();
            f1_wait_latest(&handle, message).await;
            let admitted = f1_wait_actual_surface_event(&input, "ingress", message.cloud_revision).await;
            assert!(admitted < original_deadline,
                "fixture failed to admit the burst inside the original backoff");
        }
        let latest = burst.last().unwrap().cloud_revision;
        let _ = f1_wait_actual_surface_event(&input, "build", latest).await;
        input.wait_finished(latest).await.unwrap();
        let starts = f1_surface_clock_events(&input.probe, "build");
        let after: Vec<_> = starts[before_starts..].iter().copied().collect();
        assert!(!after.is_empty());
        // Compare recorded real worker entry against the recorded exact owner
        // deadline. No sleep duration is used to infer either side.
        let premature: Vec<_> = after.iter().filter(|(_, at)| *at < original_deadline).copied().collect();
        eprintln!("F1 watermark dependency starts={after:?} original_deadline={original_deadline} premature={premature:?}");
        {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            assert!(mode.input_ready);
            assert!(Arc::ptr_eq(&mode.receiver.snapshot().unwrap().unwrap(), &source));
            assert_eq!(guard.costmap_received_monotonic_ns, receipt);
            assert_eq!(guard.committed_map, identity);
        }
        assert!(premature.is_empty(),
            "new watermark roots retried the same missing immutable object before its original 1s deadline");
        // Every ingress was before the deadline: latest-only coalescing should
        // avoid a separate build for intermediate watermark candidates.
        assert_eq!(after[0].0, latest);
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-watermark-dependency").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_supplied_missing_object_bypasses_old_deadline() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, _) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (partial, mut complete) = input.prepare_watermark_retry_series(1, true).await.unwrap();
        let complete = complete.remove(0);
        input.publish_candidate(&partial).await.unwrap();
        f1_wait_latest(&handle, &partial).await;
        input.wait_finished(partial.cloud_revision).await.unwrap();
        let deadline = f1_surface_clock_events(&input.probe, "backoff").into_iter()
            .find(|(revision, _)| *revision == partial.cloud_revision).unwrap().1;
        input.publish_candidate(&complete).await.unwrap();
        f1_wait_latest(&handle, &complete).await;
        let admitted = f1_wait_actual_surface_event(&input, "ingress", complete.cloud_revision).await;
        assert!(admitted < deadline,
            "fixture failed to admit the complete control before the original deadline");
        let started = f1_wait_actual_surface_event(&input, "build", complete.cloud_revision).await;
        input.wait_finished(complete.cloud_revision).await.unwrap();
        {
            let guard = handle.inner.lock().await;
            assert_eq!(guard.committed_map.as_ref(), Some(&complete.committed_map));
            assert_eq!(guard.surface_mode.as_ref().unwrap().installed_revision,
                Some((complete.cloud_revision, complete.committed_map.revision)));
            assert!(guard.surface_mode.as_ref().unwrap().input_ready);
            assert!(guard.pending_child_cancellation.is_none());
        }
        assert!(started < deadline,
            "complete successor inherited an already repaired dependency's delay");
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-supplied-dependency").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}

// Append inside explore_runtime::tests, after explore_surface_incomplete_tests.rs.
// Tests-only. Uses its real F1 fixture, clocks, and exact all-owner cleanup.
// Protocol-level receipt/root/body mutations are qualification inputs, not a
// claim that a production publisher can emit inconsistent lineage. No manual
// SurfaceSnapshot, injected build error, provider, fake time, or shortened retry.
// Authoring validation: read-only API inspection only; NOT COMPILED OR RUN.

async fn f1_guard_event_after(
    input: &crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture,
    kind: &str,
    revision: i64,
    prior_event_count: usize,
) -> u128 {
    tokio::time::timeout(Duration::from_secs(8), async {
        loop {
            if let Some((_, at)) = f1_surface_clock_events(&input.probe, kind)
                .into_iter().skip(prior_event_count)
                .find(|(observed, _)| *observed == revision) {
                return at;
            }
            // Poll recorded original-owner/worker events, never infer ingress.
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    }).await.unwrap_or_else(|_| panic!("missing new actual {kind} event for {revision}"))
}

fn f1_guard_first_deadline(
    input: &crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture,
    revision: i64,
) -> u128 {
    let deadlines: Vec<_> = f1_surface_clock_events(&input.probe, "backoff")
        .into_iter().filter(|(observed, _)| *observed == revision).collect();
    assert_eq!(deadlines.len(), 1, "original failed completion must establish one deadline");
    deadlines[0].1
}

fn f1_guard_mask_header(message: &mut schema::NavigationMap3D) {
    if let Some(mask) = message.virtual_obstacles.as_mut() {
        let map = &message.committed_map;
        mask.header.seq = i32::try_from(map.revision).unwrap();
        mask.header.frame_id = map.frame_id.clone();
        mask.header.stamp_sec = (map.source_timestamp_ns / 1_000_000_000) as i32;
        mask.header.stamp_nsec = (map.source_timestamp_ns % 1_000_000_000) as i32;
    }
}

fn f1_guard_advance_occupancy_only(message: &mut schema::NavigationMap3D) {
    // Explicit protocol-boundary input. The cloud receipt/root and missing O
    // stay exact; only the containing occupancy commit advances.
    message.committed_map.revision = message.committed_map.revision.checked_add(1).unwrap();
    message.committed_map.commit_sequence = message.committed_map.commit_sequence.checked_add(1).unwrap();
    f1_guard_mask_header(message);
    schema::validate_navigation_map_3d(message).unwrap();
}

async fn f1_guard_assert_original_child_revoked(
    handle: &ExploreRuntimeHandle,
    sink: &RecordingNavSink,
    lease: &ChildNavigationLease,
) {
    let guard = handle.inner.lock().await;
    let mode = guard.surface_mode.as_ref().unwrap();
    assert!(!mode.input_ready);
    assert!(mode.input_error.is_some());
    assert!(mode.receiver.snapshot().unwrap().is_none());
    assert!(guard.committed_map.is_none());
    assert_eq!(guard.costmap_received_monotonic_ns, 0);
    assert!(guard.active_child.as_ref().is_some_and(|child| &child.lease == lease),
        "the original lease remains owned until its correlated ZEROED proof");
    assert!(guard.pending_child_cancellation.as_ref().and_then(|pending| pending.ticket.as_ref())
        .is_some_and(|ticket| &ticket.lease == lease));
    assert_eq!(*sink.cancelled.lock().unwrap(), vec![lease.session_id.clone()]);
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_watermark_queued_during_failed_build_establishes_deadline() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, _) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (source, receipt, identity) = {
            let guard = handle.inner.lock().await;
            (guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(),
                guard.costmap_received_monotonic_ns, guard.committed_map.clone())
        };
        let (partial, mut successors) = input.prepare_watermark_retry_series(1, false).await.unwrap();
        let successor = successors.remove(0);
        let before_starts = f1_surface_clock_events(&input.probe, "build").len();
        input.publish_candidate(&partial).await.unwrap();
        let failed_start = f1_guard_event_after(&input, "build", partial.cloud_revision, before_starts).await;
        let finish = format!("surface-map-finish-cloud:{}", partial.cloud_revision);
        assert!(!input.probe.events().contains(&finish), "fixture missed the real failed build window");
        input.publish_candidate(&successor).await.unwrap();
        f1_wait_latest(&handle, &successor).await;
        let admitted = f1_wait_actual_surface_event(&input, "ingress", successor.cloud_revision).await;
        assert!(failed_start <= admitted);
        assert!(!input.probe.events().contains(&finish),
            "successor must be actually admitted while the original RPC build is still running");
        input.wait_finished(partial.cloud_revision).await.unwrap();
        let deadline = f1_guard_first_deadline(&input, partial.cloud_revision);
        // Ingress preceded completion, so its clock must be at least one full
        // production backoff interval before the completion-established target.
        assert!(deadline >= admitted + Duration::from_secs(1).as_nanos());
        let retried = f1_wait_actual_surface_event(&input, "build", successor.cloud_revision).await;
        assert!(retried >= deadline, "pending watermark erased the failed completion's interval");
        input.wait_finished(successor.cloud_revision).await.unwrap();
        let starts = f1_surface_clock_events(&input.probe, "build");
        let after: Vec<_> = starts[before_starts..].iter().copied().collect();
        assert_eq!(after.len(), 2, "one failed build and one latest-only retry");
        assert_eq!(after[0].0, partial.cloud_revision);
        assert_eq!(after[1].0, successor.cloud_revision);
        let guard = handle.inner.lock().await;
        assert!(guard.surface_mode.as_ref().unwrap().input_ready);
        assert!(Arc::ptr_eq(&guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(), &source));
        assert_eq!(guard.costmap_received_monotonic_ns, receipt);
        assert_eq!(guard.committed_map, identity);
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-queued-watermark-dependency").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_same_cloud_new_occupancy_keeps_original_deadline() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, _) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (source, receipt, identity) = {
            let guard = handle.inner.lock().await;
            (guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(),
                guard.costmap_received_monotonic_ns, guard.committed_map.clone())
        };
        let (partial, _) = input.prepare_watermark_retry_series(1, false).await.unwrap();
        let mut newer_occupancy = partial.clone();
        f1_guard_advance_occupancy_only(&mut newer_occupancy);
        assert_eq!(newer_occupancy.cloud_revision, partial.cloud_revision);
        assert_eq!(newer_occupancy.cloud_occupancy_revision, partial.cloud_occupancy_revision);
        assert_eq!(newer_occupancy.commit_sequence, partial.commit_sequence);
        assert_eq!(newer_occupancy.fine, partial.fine);
        input.publish_candidate(&partial).await.unwrap();
        input.wait_finished(partial.cloud_revision).await.unwrap();
        let deadline = f1_guard_first_deadline(&input, partial.cloud_revision);
        let ingress_count = f1_surface_clock_events(&input.probe, "ingress").len();
        let build_count = f1_surface_clock_events(&input.probe, "build").len();
        // Deliberately position the stimulus partway into the one-second window.
        // This delay proves NO arrival or outcome: both are established below by
        // actual original-owner/worker timestamps. It distinguishes restarting
        // a fresh one-second timer from keeping the original absolute target.
        tokio::time::sleep(Duration::from_millis(300)).await;
        input.publish_candidate(&newer_occupancy).await.unwrap();
        f1_wait_latest(&handle, &newer_occupancy).await;
        let admitted = f1_guard_event_after(&input, "ingress", newer_occupancy.cloud_revision, ingress_count).await;
        assert!(admitted < deadline, "fixture missed the original backoff window");
        assert!(admitted > deadline.saturating_sub(Duration::from_millis(850).as_nanos()),
            "fixture must leave a meaningful gap between old and restarted deadlines");
        let retried = f1_guard_event_after(&input, "build", newer_occupancy.cloud_revision, build_count).await;
        assert!(retried >= deadline, "same-cloud occupancy commit shortened the original deadline");
        assert!(retried < admitted + Duration::from_secs(1).as_nanos(),
            "same-cloud occupancy commit restarted/extended the original one-second deadline");
        input.wait_finish_count(partial.cloud_revision, 2).await.unwrap();
        // The next actual failure gets a new interval; ingress itself did not
        // create an extra failure/deadline assignment.
        let deadlines: Vec<_> = f1_surface_clock_events(&input.probe, "backoff")
            .into_iter().filter(|(revision, _)| *revision == partial.cloud_revision).collect();
        assert_eq!(deadlines.len(), 2);
        assert_eq!(deadlines[0].1, deadline);
        assert_eq!(f1_surface_clock_events(&input.probe, "build").len(), build_count + 1);
        let guard = handle.inner.lock().await;
        assert!(guard.surface_mode.as_ref().unwrap().input_ready);
        assert!(Arc::ptr_eq(&guard.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap(), &source));
        assert_eq!(guard.costmap_received_monotonic_ns, receipt);
        assert_eq!(guard.committed_map, identity);
        assert_eq!(guard.surface_mode.as_ref().unwrap().latest.as_ref().unwrap().committed_map,
            newer_occupancy.committed_map);
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-same-cloud-occupancy-dependency").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}

async fn f1_guard_backoff_revocation(changed_mask: bool) {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    let (handle, sink) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let mut installed_message = input.first.clone();
        if changed_mask && installed_message.virtual_obstacles.is_none() {
            // Install a real cache-derived masked source through the actual
            // consumer. This is an explicit protocol-level mask fixture, not
            // zones-store producer lineage. Cover the actual [0,5] x [0,3]
            // floor plus 1m margin: the original mask contract blocks its exterior.
            installed_message.virtual_obstacles = Some(schema::OccupancyGrid {
                header: schema::Header::default(),
                info: schema::MapMetaData { resolution: 0.25, width: 28, height: 20,
                    origin: schema::Pose { position: [-1.0, -1.0, 0.0],
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0] },
                    ..schema::MapMetaData::default() },
                data: vec![0; 28 * 20],
            });
            f1_guard_advance_occupancy_only(&mut installed_message);
            input.publish_candidate(&installed_message).await.unwrap();
            f1_wait_latest(&handle, &installed_message).await;
            input.wait_finish_count(installed_message.cloud_revision, 2).await.unwrap();
        }
        let (mut partial, mut successors) = input.prepare_watermark_retry_series(1, false).await.unwrap();
        let mut successor = successors.remove(0);
        if changed_mask {
            partial.virtual_obstacles = installed_message.virtual_obstacles.clone();
            successor.virtual_obstacles = installed_message.virtual_obstacles.clone();
            f1_guard_mask_header(&mut partial);
            f1_guard_mask_header(&mut successor);
            let before = successor.virtual_obstacles.as_ref().unwrap().clone();
            // This cell covers actual floor at x=[4.5,4.75], y=[2.5,2.75],
            // away from the original [1.5,1.5] pose. Only its 0/100 bit changes.
            let cell = successor.virtual_obstacles.as_mut().unwrap().data.get_mut(14 * 28 + 22).unwrap();
            *cell = if *cell == 100 { 0 } else { 100 };
            let after = successor.virtual_obstacles.as_ref().unwrap();
            assert_eq!(before.header, after.header);
            assert_eq!(before.info, after.info);
            assert_ne!(before.data, after.data, "test must change actual mask contents only");
        }
        schema::validate_navigation_map_3d(&partial).unwrap();
        schema::validate_navigation_map_3d(&successor).unwrap();
        let (source, receipt, identity, installed_revision) = {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            assert!(mode.input_ready);
            assert_eq!(guard.committed_map.as_ref(), Some(&installed_message.committed_map));
            assert!(guard.costmap_received_monotonic_ns > 0);
            (mode.receiver.snapshot().unwrap().unwrap(), guard.costmap_received_monotonic_ns,
                guard.committed_map.clone(), mode.installed_revision)
        };
        // Construct the mismatch with the real builder before the deadline.
        let other_source = if changed_mask { None } else {
            let mut cache = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&surface_config("map"));
            let other = cache.build_typed(&installed_message).unwrap();
            assert!(!Arc::ptr_eq(&source, &other));
            Some(other)
        };
        let started = handle.start("f1-backoff-revocation", "prefer_wavefront").await;
        assert_eq!(started.status, STATUS_ACCEPTED, "{}", started.message);
        let active = handle.tick_with_pose(surface_pose(&source, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let lease = active_child_lease(&handle).await;
        input.publish_candidate(&partial).await.unwrap();
        input.wait_finished(partial.cloud_revision).await.unwrap();
        let deadline = f1_guard_first_deadline(&input, partial.cloud_revision);
        {
            let guard = handle.inner.lock().await;
            let mode = guard.surface_mode.as_ref().unwrap();
            assert!(mode.input_ready);
            assert!(Arc::ptr_eq(&mode.receiver.snapshot().unwrap().unwrap(), &source));
            assert_eq!(guard.costmap_received_monotonic_ns, receipt);
            assert_eq!(guard.committed_map, identity);
            assert_eq!(mode.installed_revision, installed_revision);
            assert!(guard.active_child.as_ref().is_some_and(|child| child.lease == lease));
            assert!(guard.pending_child_cancellation.is_none());
            assert!(sink.cancelled.lock().unwrap().is_empty());
            if let Some(other) = other_source.as_ref() {
                // Actual receiver Arc replacement, not merely asking the defer
                // helper about an uninstalled Arc. No receipt/ticket/clock edit.
                // This is a qualification boundary, not a claimed production
                // single-consumer ownership transition.
                mode.receiver.replace(Some(other.clone())).unwrap();
                assert!(Arc::ptr_eq(&mode.receiver.snapshot().unwrap().unwrap(), other));
            }
        }
        input.publish_candidate(&successor).await.unwrap();
        f1_wait_latest(&handle, &successor).await;
        let admitted = f1_wait_actual_surface_event(&input, "ingress", successor.cloud_revision).await;
        assert!(admitted < deadline, "fixture missed the actual backoff window");
        let build = f1_wait_actual_surface_event(&input, "build", successor.cloud_revision).await;
        assert!(build < deadline, "changed mask/actual installed Arc inherited an obsolete delay");
        input.wait_finished(successor.cloud_revision).await.unwrap();
        f1_guard_assert_original_child_revoked(&handle, &sink, &lease).await;
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-backoff-revocation").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_changed_mask_during_backoff_bypasses_deadline_and_revokes() {
    f1_guard_backoff_revocation(true).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_actual_installed_source_mismatch_bypasses_deadline_and_revokes() {
    f1_guard_backoff_revocation(false).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn surface_incomplete_supplied_missing_body_reaches_receiver_invalid_before_deadline() {
    use futures::FutureExt;
    use crate::lcm_runtime::mapping_feed_tests::F1RealEntryFixture;
    use nuwax_nav_stack::surface_map::SurfaceMapBuildError;
    let (handle, sink) = configured_handle();
    let mut input = F1RealEntryFixture::start(handle.clone()).await.unwrap();
    let exercise = std::panic::AssertUnwindSafe(async {
        input.wait_finished(input.first.cloud_revision).await.unwrap();
        let (partial, mut complete) = input.prepare_watermark_retry_series(1, true).await.unwrap();
        let mut malformed = complete.remove(0);
        let initial_root = nuwax_map_chunks::Manifest::decode(&input.first.fine.as_ref().unwrap().root).unwrap();
        let root = nuwax_map_chunks::Manifest::decode(&partial.fine.as_ref().unwrap().root).unwrap();
        let missing: Vec<_> = root.blocks.iter().filter(|block|
            !initial_root.blocks.iter().any(|old| old.object == block.object)
                && !partial.fine.as_ref().unwrap().included.contains_key(&block.object)).collect();
        assert_eq!(missing.len(), 1, "test must supply exactly the original failed dependency");
        let reference = missing[0];
        let old = initial_root.blocks.iter().find(|old| old.key == reference.key).unwrap();
        assert_eq!(reference.geometry_revision, old.geometry_revision);
        let fine = malformed.fine.as_mut().unwrap();
        let authentic = fine.included.get(&reference.object).unwrap();
        let mut block = nuwax_map_chunks::decode_block(authentic).unwrap();
        let old_local = *block.keys().next().unwrap();
        let new_local = (0_u16..32768).find(|key| !block.contains_key(key)).unwrap();
        let evidence = block.remove(&old_local).unwrap();
        block.insert(new_local, evidence);
        let wrong_membership = nuwax_map_chunks::encode_block(&block).unwrap();
        assert_eq!(nuwax_map_chunks::block_summary(authentic).unwrap(),
            nuwax_map_chunks::block_summary(&wrong_membership).unwrap());
        fine.included.insert(reference.object, wrong_membership);
        // Valid wire/block structure, correct object membership/count/time,
        // but false unchanged-geometry claim. Schema must admit it; the original
        // Receiver progression check must reject it. This avoids accidentally
        // testing only encoder/schema rejection of malformed bytes.
        schema::validate_navigation_map_3d(&malformed).unwrap();
        let encoded = schema::encode_navigation_map_3d_body(&malformed).unwrap();
        assert_eq!(schema::decode_navigation_map_3d_body(&encoded).unwrap(), malformed);
        let mut control = nuwax_nav_stack::surface_map::SurfaceMapCache::configured(&surface_config("map"));
        control.build_typed(&input.first).unwrap();
        let invalid_reason = match control.build_typed(&malformed) {
            Err(SurfaceMapBuildError::Invalid(reason)) => reason,
            Err(other) => panic!("supplied-body control was not Receiver Invalid: {other}"),
            Ok(_) => panic!("malformed supplied dependency passed the real Receiver"),
        };
        assert!(invalid_reason.contains("changed members need advanced geometry revision"),
            "unexpected original Receiver rejection: {invalid_reason}");
        let source = handle.inner.lock().await.surface_mode.as_ref().unwrap().receiver.snapshot().unwrap().unwrap();
        let start = handle.start("f1-malformed-supplied-dependency", "prefer_wavefront").await;
        assert_eq!(start.status, STATUS_ACCEPTED, "{}", start.message);
        let active = handle.tick_with_pose(surface_pose(&source, 0.0, Quaternion::IDENTITY)).await;
        assert_eq!(active.status, STATUS_ACTIVE, "{}", active.message);
        let lease = active_child_lease(&handle).await;
        input.publish_candidate(&partial).await.unwrap();
        input.wait_finished(partial.cloud_revision).await.unwrap();
        let deadline = f1_guard_first_deadline(&input, partial.cloud_revision);
        input.publish_candidate(&malformed).await.unwrap();
        f1_wait_latest(&handle, &malformed).await;
        let admitted = f1_wait_actual_surface_event(&input, "ingress", malformed.cloud_revision).await;
        assert!(admitted < deadline, "fixture missed the actual backoff window");
        let build = f1_wait_actual_surface_event(&input, "build", malformed.cloud_revision).await;
        assert!(build < deadline, "supplied bytes must trigger validation before the obsolete deadline");
        input.wait_finished(malformed.cloud_revision).await.unwrap();
        f1_guard_assert_original_child_revoked(&handle, &sink, &lease).await;
        assert_eq!(handle.inner.lock().await.surface_mode.as_ref().unwrap().input_error.as_deref(),
            Some(invalid_reason.as_str()));
    }).catch_unwind().await;
    f1_finish_all_owners(&mut input, &handle, "f1-malformed-supplied-dependency").await;
    if let Err(panic) = exercise { std::panic::resume_unwind(panic); }
}
