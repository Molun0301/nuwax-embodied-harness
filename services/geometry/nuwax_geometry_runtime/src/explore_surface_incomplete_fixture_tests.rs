// Tests-only actual-entry qualification. Included inside lcm_runtime::mapping_feed_tests.
// This module must be pub(crate) under its existing #[cfg(test)] only.
// Uses the real canonical Writer/seal/publication codec and original consumer.
pub(crate) struct F1RealEntryFixture {
    fixture: NavigationMap3DFixture,
    transport: LcmTransport,
    owner: GeometryRuntimeTaskOwner,
    _installation: GeometryTopologyTestInstallation,
    pub(crate) probe: Arc<GeometryTopologyTestControl>,
    pub(crate) first: schema::NavigationMap3D,
    points: Vec<Point3<f64>>,
    stamp: u64,
}

impl F1RealEntryFixture {
    pub(crate) async fn start(
        explore: crate::explore_runtime::ExploreRuntimeHandle,
    ) -> Result<Self, String> {
        let fixture = NavigationMap3DFixture::new();
        // PCT按0.05米精细体素逐格搜索实测地面，缺测格不连通；地面点须铺满每个精细格。
        let points: Vec<_> = (0..60).flat_map(|x| (0..60).map(move |y|
            Point3::new((x as f64 + 0.5) * 0.05, (y as f64 + 0.5) * 0.05, 0.0)))
            .collect();
        let stamp = crate::explore_runtime::now_unix_nanos();
        fixture.commit_points(stamp, &points);
        let (_, _, publication) = latest_canonical_cloud_publication(&fixture.writer)?.ok_or("initial publication missing")?;
        let prepared = navigation_map_3d_publication_metadata(&fixture.context, &publication, &fixture.state, "map").await?;
        let frame = build_navigation_map_3d_publication_frame(&publication, &prepared, None)?;
        let first = schema::decode_navigation_map_3d_body(&frame.payload).map_err(|e| e.to_string())?;
        let root = nuwax_map_chunks::Manifest::decode(&first.fine.as_ref().ok_or("initial fine root missing")?.root)?;
        if !root.blocks.iter().all(|b| first.fine.as_ref().unwrap().included.contains_key(&b.object)) {
            return Err("initial fixture must fit the real inline budget; do not fill missing objects by a fake fetch".into());
        }
        let config = nuwax_nav_stack::surface_config::SurfaceNavigationConfig::new(
            "map".into(), 0.5, 0.2, 0.6, 0.1, 0.15, 18.0, 0.2).map_err(|e| e.to_string())?;
        explore.configure_surface_source(Ok(config.clone()), 37, first.commit_boot_nonce).await?;
        let transport = LcmTransport::new_real("memq://").map_err(|e| e.to_string())?;
        // Same transport/context for map publish and owned subscribe. The
        // FineMapReader RPC has another isolated memq context and NO provider.
        // 补取地址直接交给消费者缓存，不读也不改进程环境变量。
        let subscription = transport.subscribe_owned(
            LcmTopic::new("/nav/map3d").map_err(|e| e.to_string())?,
            Some(LcmMessageType::new("nuwax_msgs.NavigationMap3D").map_err(|e| e.to_string())?),
        ).await.map_err(|e| e.to_string())?;
        let probe = GeometryTopologyTestControl::new();
        let installation = probe.install();
        let (control, registrations) = GeometryRuntimeTaskControl::channel_with_topology_control(Some(probe.clone()));
        let mut owner = GeometryRuntimeTaskOwner::start(control.clone(), registrations);
        if let Err(error) = control.register_persistent("surface-map-consumer", run_owned_surface_map_consumer(
            subscription, explore,
            nuwax_nav_stack::surface_map::SurfaceMapCache::configured_with_lcm_url(&config, "memq://"),
            control.clone(), control.shutdown_request_receiver())) {
            let _ = owner.shutdown().await;
            return Err(error);
        }
        let result = transport.publish(frame).await.map_err(|e| e.to_string());
        if let Err(error) = result { let _ = owner.shutdown().await; return Err(error); }
        Ok(Self { fixture, transport, owner, _installation: installation, probe, first, points, stamp })
    }

    pub(crate) async fn publish_partial(&mut self) -> Result<schema::NavigationMap3D, String> {
        let mut candidate = self.prepare_complete_candidate().await?;
        let first_root = nuwax_map_chunks::Manifest::decode(&self.first.fine.as_ref().unwrap().root)?;
        let next_root = nuwax_map_chunks::Manifest::decode(&candidate.fine.as_ref().ok_or("candidate fine root missing")?.root)?;
        if first_root.domain != next_root.domain || candidate.cloud_revision <= self.first.cloud_revision {
            return Err("fixture did not produce a strictly newer same-domain root".into());
        }
        let mask_same = match (&self.first.virtual_obstacles, &candidate.virtual_obstacles) {
            (None, None) => true,
            (Some(a), Some(b)) => a.header.frame_id == b.header.frame_id && a.info.width == b.info.width
                && a.info.height == b.info.height && a.info.resolution.to_bits() == b.info.resolution.to_bits()
                && a.info.origin == b.info.origin && a.data == b.data,
            _ => false,
        };
        if !mask_same { return Err("fixture changed virtual mask; not the F1 narrow case".into()); }
        let absent = next_root.blocks.iter().find(|next|
            !first_root.blocks.iter().any(|old| old.object == next.object)
                && candidate.fine.as_ref().unwrap().included.contains_key(&next.object))
            .ok_or("repeat observations produced no new inline immutable object")?.object;
        // Omission is a legal transfer (the real 128 KiB budget does this).
        // Keep the authentic root and every remaining object's exact bytes.
        candidate.fine.as_mut().unwrap().included.remove(&absent);
        schema::validate_navigation_map_3d(&candidate)?;
        self.publish_candidate(&candidate).await?;
        Ok(candidate)
    }

    pub(crate) async fn wait_finished(&self, revision: i64) -> Result<(), String> {
        let wanted = format!("surface-map-finish-cloud:{revision}");
        tokio::time::timeout(Duration::from_secs(8), async {
            loop {
                if self.probe.events().iter().any(|e| e == &wanted) { break; }
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.map_err(|_| format!("original consumer did not finish revision {revision}; events={:?}", self.probe.events()))
    }

    pub(crate) async fn shutdown(&mut self) -> Result<(), String> {
        // The original Geometry supervisor joins the retained blocking builder,
        // the persistent consumer, and its owned unsubscribe acknowledgement.
        let result = self.owner.shutdown().await;
        if !result.producers_settled() { return Err("Geometry producers not settled".into()); }
        result.result
    }
}

impl F1RealEntryFixture {
    pub(crate) async fn prepare_complete_candidate(&mut self) -> Result<schema::NavigationMap3D, String> {
        let previous = self.stamp;
        self.stamp = tokio::time::timeout(Duration::from_secs(2), async {
            loop {
                let now = crate::explore_runtime::now_unix_nanos();
                let elapsed = now.saturating_sub(previous);
                let interval = crate::mapping_backend::MAPPING_CANDIDATE_INTERVAL_NS;
                if now > previous && elapsed >= interval { break now; }
                tokio::time::sleep(Duration::from_nanos(interval.saturating_sub(elapsed).max(1))).await;
            }
        }).await.map_err(|_| "real mapping interval did not elapse".to_owned())?;
        self.fixture.commit_points(self.stamp, &self.points);
        self.current_canonical_candidate().await
    }

    async fn current_canonical_candidate(&self) -> Result<schema::NavigationMap3D, String> {
        let (_, _, publication) = latest_canonical_cloud_publication(&self.fixture.writer)?.ok_or("canonical publication missing")?;
        let prepared = navigation_map_3d_publication_metadata(&self.fixture.context, &publication, &self.fixture.state, "map").await?;
        let frame = build_navigation_map_3d_publication_frame(&publication, &prepared, None)?;
        let message = schema::decode_navigation_map_3d_body(&frame.payload).map_err(|e| e.to_string())?;
        if let Some(fine) = message.fine.as_ref() {
            let root = nuwax_map_chunks::Manifest::decode(&fine.root)?;
            if !root.blocks.iter().all(|b| fine.included.contains_key(&b.object)) {
                return Err("complete control must fit original inline budget".into());
            }
        }
        Ok(message)
    }

    pub(crate) async fn publish_candidate(&self, message: &schema::NavigationMap3D) -> Result<(), String> {
        let payload = schema::encode_navigation_map_3d_body(message).map_err(|e| e.to_string())?;
        let frame = crate::typed_codec::build_typed_frame(
            LcmTopic::new("/nav/map3d").map_err(|e| e.to_string())?,
            LcmMessageType::new("nuwax_msgs.NavigationMap3D").map_err(|e| e.to_string())?,
            payload, message.cloud_revision as u64,
        );
        self.transport.publish(frame).await.map_err(|e| e.to_string())
    }

    pub(crate) async fn switch_mapping_domain(&mut self, publish_clear: bool) -> Result<schema::NavigationMap3D, String> {
        let generation = {
            let mut mapping = self.fixture.context.mapping.lock();
            let old = mapping.snapshot().session_id;
            mapping.stop_session(&old).map_err(|e| e.to_string())?;
            let next = format!("f1-new-domain-{}", uuid::Uuid::new_v4());
            mapping.start_session(&next, "f1-new-domain-map").map_err(|e| e.to_string())?;
            mapping.snapshot().session_generation
        };
        self.fixture.writer.lock().as_mut().unwrap().begin_session(generation)
            .ok_or("actual new session did not reserve canonical clear")?;
        let message = if publish_clear {
            // Actual producer clear, not an empty hand-built source snapshot.
            self.current_canonical_candidate().await?
        } else {
            // Test the consumer's live-domain-change branch explicitly. The
            // publisher's separate clear-before-live ordering has existing tests.
            self.prepare_complete_candidate().await?
        };
        if message.cloud_revision <= self.first.cloud_revision {
            return Err("same Writer did not advance cloud revision across session".into());
        }
        self.publish_candidate(&message).await?;
        Ok(message)
    }

    pub(crate) async fn wait_finish_count(&self, revision: i64, count: usize) -> Result<(), String> {
        let wanted = format!("surface-map-finish-cloud:{revision}");
        tokio::time::timeout(Duration::from_secs(8), async {
            while self.probe.events().iter().filter(|event| *event == &wanted).count() < count {
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
        }).await.map_err(|_| format!("missing actual finish {revision} count {count}"))
    }
}

// Append inside the EXISTING tests-only F1 fixture module.
// Writer-generated watermark roots enter the real publication codec and original
// consumer. No manual SurfaceSnapshot, synthetic build failure, or fake RPC.
// Scope: consumer ingress/retry qualification, not a claim that this helper
// exercises the full production sensor-to-canonical-commit lineage.
impl F1RealEntryFixture {
    pub(crate) async fn prepare_watermark_retry_series(
        &mut self,
        count: u64,
        complete_last: bool,
    ) -> Result<(schema::NavigationMap3D, Vec<schema::NavigationMap3D>), String> {
        use nuwax_map_chunks::{LayerSource, Manifest, Writer};
        if count == 0 { return Err("watermark series must be nonempty".into()); }
        let complete = self.prepare_complete_candidate().await?;
        let initial = Manifest::decode(&self.first.fine.as_ref().ok_or("initial fine root missing")?.root)?;
        let failed = Manifest::decode(&complete.fine.as_ref().ok_or("candidate fine root missing")?.root)?;
        if initial.domain != failed.domain || complete.cloud_revision <= self.first.cloud_revision {
            return Err("fixture did not produce a strictly newer same-domain root".into());
        }
        let mask_same = match (&self.first.virtual_obstacles, &complete.virtual_obstacles) {
            (None, None) => true,
            (Some(a), Some(b)) => a.header.frame_id == b.header.frame_id && a.info.width == b.info.width
                && a.info.height == b.info.height && a.info.resolution.to_bits() == b.info.resolution.to_bits()
                && a.info.origin == b.info.origin && a.data == b.data,
            _ => false,
        };
        if !mask_same { return Err("fixture changed virtual mask; not the F1 narrow case".into()); }
        let sealed = self.fixture.writer.lock().as_ref()
            .and_then(CloudMapShmWriter::committed_fine_cloud).ok_or("canonical sealed root missing")?;
        if **sealed.manifest() != failed { return Err("fixture lost canonical candidate lineage".into()); }
        let missing = failed.blocks.iter().find(|next|
            !initial.blocks.iter().any(|old| old.object == next.object)
                && complete.fine.as_ref().unwrap().included.contains_key(&next.object))
            .ok_or("candidate has no new inline object to omit")?.object;
        let mut partial = complete.clone();
        partial.fine.as_mut().unwrap().included.remove(&missing);
        schema::validate_navigation_map_3d(&partial)?;

        // Restore an authentic canonical root into a real Writer with the same
        // object repository, then change ONLY source-watermark metadata. This
        // happens before publishing the failed input or starting its deadline.
        let mut writer = Writer::new(sealed.repository.clone(), failed.voxel_size_m, 4 * 1024 * 1024)?;
        writer.restore(&failed)?;
        let source = Arc::new(LayerSource::Auxiliary {
            sensor_id: "f1-watermark-retry-fixture".into(), clock_id: "fixture-clock".into(),
            runtime_generation: 1, session_generation: 1,
        });
        let mut previous = sealed;
        let mut messages = Vec::new();
        for index in 1..=count {
            writer.record_source_watermark(source.clone(), failed.source_timestamp_ns, index)?;
            let next = Arc::new(writer.seal(failed.domain.clone(), failed.revision + index,
                failed.source_timestamp_ns)?);
            if next.manifest().blocks != failed.blocks
                || next.manifest().geometry_revision != failed.geometry_revision
                || next.manifest().source_timestamp_ns != failed.source_timestamp_ns
                || next.manifest().source_watermarks == previous.manifest().source_watermarks {
                return Err("fixture is not a pure source-watermark publication".into());
            }
            // Real production inline delta deliberately carries no unchanged
            // object, including the object the consumer still lacks.
            let mut transfer = navigation_map_transfer::prepare(&next, Some(previous.manifest()))?;
            if !transfer.included.is_empty() || transfer.included.contains_key(&missing) {
                return Err("watermark delta unexpectedly supplied missing object".into());
            }
            if complete_last && index == count {
                transfer = navigation_map_transfer::prepare(&next, None)?;
                if !next.manifest().blocks.iter().all(|block| transfer.included.contains_key(&block.object)) {
                    return Err("complete control exceeds original inline budget".into());
                }
            }
            let mut message = complete.clone();
            message.cloud_revision = i64::try_from(next.manifest().revision).map_err(|_| "root revision overflow")?;
            message.cloud.header.seq = i32::try_from(message.cloud_revision).map_err(|_| "cloud header overflow")?;
            // Keep the authentic complete candidate's entire occupancy/cloud
            // receipt and mask unchanged. Only publication revision/header and
            // Writer source-watermark root advance. This isolates watermark
            // admission rather than relying on a synthetic occupancy advance.
            message.fine = Some(transfer);
            schema::validate_navigation_map_3d(&message)?;
            messages.push(message);
            previous = next;
        }
        Ok((partial, messages))
    }
}
