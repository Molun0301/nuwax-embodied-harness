//! 二维图沿公共激活账本、规范地图提交及同一进程拥有方完成切换。
use super::*;
use nuwax_slam_2d_runtime::{
    archive,
    protocol::{Operation, Snapshot},
};

impl GeometryRuntimeService {
    pub(super) async fn activate_planar_map(
        &self,
        request: ActivateMapRequest,
        class: ActivationRequestIdentityClass,
    ) -> Result<Response<MappingSessionState>, Status> {
        if !class.is_modern() {
            return Err(Status::invalid_argument("二维完整图要求明确的现代激活身份"));
        }
        let backend = request
            .mapping_backend_checkpoint
            .as_ref()
            .ok_or_else(|| Status::invalid_argument("二维图缺少完整检查点"))?;
        validate_planar_checkpoint(backend).map_err(Status::invalid_argument)?;
        let occupancy = request
            .checkpoint
            .as_ref()
            .ok_or_else(|| Status::invalid_argument("二维图缺少物理栅格"))?;
        if request.cloud_checkpoint.is_some()
            || backend.canonical_occupancy_checkpoint.as_ref() != Some(occupancy)
        {
            return Err(Status::invalid_argument("二维激活栅格与完整图检查点不一致"));
        }
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("二维适配器不可用"))?
            .clone();
        let binding = adapter
            .lock()
            .planar
            .clone()
            .ok_or_else(|| Status::unavailable("二维拥有方不可用"))?;
        decode_checkpoint_grid(occupancy.clone(), &binding.settings.template.map_frame_id)
            .map_err(Status::invalid_argument)?;
        let occupancy_sha =
            canonical_occupancy_sha256(occupancy).map_err(Status::invalid_argument)?;
        let backend_sha =
            canonical_backend_sha256(backend, &binding.settings.template.map_frame_id)
                .map_err(Status::invalid_argument)?;
        let manifest = class.asset_manifest_sha256().unwrap();
        let bundle = class.asset_bundle_sha256().unwrap();
        if crate::map_checksum::sha256_bytes(&canonical_asset_bundle_preimage(
            manifest,
            occupancy_sha,
            None,
            Some(backend_sha),
        )) != bundle
        {
            return Err(Status::invalid_argument(
                "ACTIVATE_MAP_ASSET_BUNDLE_SHA256_MISMATCH",
            ));
        }
        let request_sha = crate::map_checksum::sha256_bytes(
            &canonical_activation_request_preimage(
                &class,
                self.activation_receipts.runtime_generation(),
                &request.session_id,
                &request.map_name,
                occupancy_sha,
                None,
                Some(backend_sha),
            )
            .map_err(Status::invalid_argument)?,
        );
        let _transition = self.mapping_transition.lock().await;
        let _operation = binding.operation.lock().await;
        self.ensure_running_admission("ActivateMap").await?;
        let epoch = binding.fence.snapshot();
        if epoch.active || self.bridge.explore.blocks_mapping_mutation().await {
            return Err(Status::failed_precondition("二维图切换被重定位或探索占用"));
        }
        let current = adapter.lock().snapshot();
        let mut seed = crate::correction_coordinator::ActivationReceiptSeed {
            activation_request_id: class.request_id().into(),
            request_key: request_sha.into(),
            checkpoint_owner: None,
            asset_manifest_sha256: Some(manifest),
            asset_bundle_sha256: Some(bundle),
            modern: true,
            immutable_target: None,
            planar_target: None,
        };
        if self.activation_receipts.has_request(class.request_id()) {
            return match self.activation_receipts.stage_or_replay(
                seed,
                mapping_session_state(current, "二维激活重放"),
                false,
            ) {
                crate::correction_coordinator::ActivationLedgerAdmission::Terminal(state) => {
                    Ok(Response::new(*state))
                }
                _ => Err(Status::unavailable("二维激活存在未封存的前序操作")),
            };
        }
        let (session_generation, backend_generation) = match adapter
            .lock()
            .prepare_session_start_admission(&request.session_id, &request.map_name)
            .map_err(mapping_lifecycle_status)?
        {
            crate::mapping_adapter::MappingSessionStartAdmission::Ready {
                session_generation,
                backend_generation,
                ..
            } => (session_generation, backend_generation),
            _ => return Err(Status::already_exists("二维激活目标会话已活动")),
        };
        let reference = backend.graph_artifact.as_ref().unwrap().clone();
        let mut file = self
            .graph_artifacts
            .lock()
            .open(&reference)
            .map_err(Status::failed_precondition)?;
        let (file, metadata, cells) = tokio::task::spawn_blocking(move || {
            let metadata = archive::verify(&mut file)?;
            let (_, cells) = archive::grid(&mut file)?;
            Ok::<_, String>((file, metadata, cells))
        })
        .await
        .map_err(|e| Status::internal(e.to_string()))?
        .map_err(Status::invalid_argument)?;
        let meta = occupancy.meta.as_ref().unwrap();
        if metadata.session.session_id != backend.source_session_id
            || metadata.session.session_generation != backend.source_session_generation
            || metadata.width != meta.width
            || metadata.height != meta.height
            || metadata.resolution as f32 != meta.resolution
            || metadata.origin != [meta.origin_x, meta.origin_y]
            || metadata.source_unix_timestamp_ns != meta.timestamp_ns
            || cells.iter().map(|v| *v as u8).collect::<Vec<_>>() != occupancy.data
        {
            return Err(Status::invalid_argument(
                "二维档案来源或栅格与资产检查点不一致",
            ));
        }
        let session = binding.session(&request.session_id, &request.map_name, session_generation);
        metadata
            .compatible_with(&session)
            .map_err(Status::invalid_argument)?;
        let target = Arc::new(crate::planar_artifacts::Target {
            file: Arc::new(file),
            size: reference.size,
            sha: reference.sha256.as_slice().try_into().unwrap(),
            metadata: metadata.clone(),
        });
        let candidate = binding
            .prepare_candidate(
                target.open().map_err(Status::internal)?,
                target.size,
                target.sha,
                session.clone(),
                true,
                metadata.estimate.map_from_base,
            )
            .await
            .map_err(Status::failed_precondition)?;
        let mut estimate = metadata.estimate.clone();
        estimate.graph_revision = 1;
        let snapshot = Snapshot {
            snapshot_id: 1,
            session: session.clone(),
            estimate,
            width: metadata.width,
            height: metadata.height,
            resolution: metadata.resolution,
            origin: metadata.origin,
            payload_size: metadata.members[2].size,
            data_sha256: metadata.members[2].sha256,
            file_name: String::new(),
        };
        let preparation = adapter.lock().prepare_planar_activation(
            snapshot,
            cells,
            metadata.source_unix_timestamp_ns,
            backend_generation,
        );
        let preparation = tokio::task::spawn_blocking(move || preparation.compute())
            .await
            .map_err(|e| e.to_string())
            .and_then(|r| r);
        let prepared = match preparation {
            Ok(v) => v,
            Err(e) => {
                binding
                    .discard_candidate(&candidate.identity().owner_nonce)
                    .await
                    .map_err(Status::unavailable)?;
                return Err(Status::failed_precondition(e));
            }
        };
        seed.planar_target = Some(target);
        let mut token = match self.activation_receipts.stage_or_replay(
            seed,
            mapping_session_state(current.clone(), "二维图已验证"),
            true,
        ) {
            crate::correction_coordinator::ActivationLedgerAdmission::Execute(t) => t,
            crate::correction_coordinator::ActivationLedgerAdmission::Terminal(s) => {
                binding
                    .discard_candidate(&candidate.identity().owner_nonce)
                    .await
                    .map_err(Status::unavailable)?;
                return Ok(Response::new(*s));
            }
        };
        let mut promoted = false;
        let result = async {
            token = self
                .activation_receipts
                .reserve_backend_generation(&token, backend_generation)
                .ok_or("二维激活后端代次预留失败")?;
            let identity =
                crate::map_checksum::sha256_bytes(&canonical_activation_identity_preimage(
                    request_sha,
                    self.activation_receipts.runtime_generation(),
                    session_generation,
                    backend_generation,
                    "slam_toolbox",
                    Some(backend_sha),
                    manifest,
                    bundle,
                )?);
            token = self
                .activation_receipts
                .bind_activation_identity(&token, identity, session_generation)
                .ok_or("二维激活最终身份绑定失败")?;
            let response = candidate
                .execute(Operation::SetPaused {
                    session_id: request.session_id.clone(),
                    session_generation,
                    paused: false,
                })
                .await?;
            if let Some(e) = response.error {
                return Err(e);
            }
            let mut projection = self.bridge.lock_explore_projection_owned().await;
            let lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
            self.ensure_running_admission("ActivateMap")
                .await
                .map_err(|e| e.to_string())?;
            if binding.fence.snapshot() != epoch
                || !self.activation_receipts.validate_token(&token)
                || adapter.lock().snapshot() != current
            {
                return Err("二维激活准备期间会话或围栏变化".into());
            }
            if let Some(writer) = self.bridge.occupancy_shm.as_ref() {
                writer
                    .lock()
                    .begin_session(session_generation)
                    .map_err(|e| e.to_string())?;
            }
            let _started = adapter
                .lock()
                .start_session(&request.session_id, &request.map_name)
                .map_err(|e| format!("{e:?}"))?;
            promoted = true;
            binding.promote(&candidate)?;
            self.bridge
                .revoke_committed_map_projection_locked(&mut projection)?;
            let mut shared = self.bridge.state.write().await;
            {
                let mut a = adapter.lock();
                a.planar_ready = true;
                a.planar_localization = true;
                a.planar_epoch = epoch.epoch;
            }
            let (map, event) = adapter
                .lock()
                .commit_planar_snapshot(prepared, shared.map.revision)?;
            adapter.lock().record_checkpoint_activation(identity)?;
            shared.map = map;
            shared.pose = Default::default();
            shared.transforms.clear();
            shared.tf_tree = Arc::new(nuwax_tf::TfTree::new());
            binding.install_sensor_edge(&mut shared);
            shared.pose_changed.send_replace(());
            drop(shared);
            *self.bridge.occupancy_shm_current.write().await = None;
            let receipt = adapter
                .lock()
                .latest_occupancy_commit_receipt()
                .ok_or("二维激活规范收据缺失")?;
            let target_receipt = self
                .activation_receipts
                .store_planar_proof(&token, receipt)
                .ok_or("二维目标收据绑定失败")?;
            record_activation_journal(
                &self.activation_receipts,
                &token,
                request_sha,
                crate::correction_coordinator::ActivationJournalStep::OccupancyInstall,
                crate::correction_coordinator::ActivationJournalStatus::Applied,
                Some(occupancy_sha.into()),
                None,
            );
            let _ = self.bridge.map_revision_tx.send(event);
            let mut state = mapping_session_state(
                adapter.lock().snapshot(),
                "二维完整图已激活，等待真实扫描定位",
            );
            state.planar_mode = "localization".into();
            let sealed = self
                .activation_receipts
                .seal(&token, Some(target_receipt.into()), state)
                .ok_or("二维激活封存失败")?;
            drop(lifecycle);
            drop(projection);
            Ok::<_, String>(sealed)
        }
        .await;
        match result {
            Ok(sealed) => Ok(Response::new(sealed)),
            Err(mut error) => {
                if promoted {
                    let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
                    let _ = adapter.lock().force_stop_started_session_detached(
                        &request.session_id,
                        session_generation,
                    );
                    adapter.lock().planar_ready = false;
                    drop(_lifecycle);
                    let _ = candidate
                        .execute(Operation::Stop {
                            session_id: request.session_id,
                            session_generation,
                        })
                        .await;
                } else if let Err(e) = binding
                    .discard_candidate(&candidate.identity().owner_nonce)
                    .await
                {
                    error = format!("{error}；候选回收失败：{e}");
                }
                let disposition = if promoted {
                    ActivationReceiptDisposition::InvalidatedPostcommit
                } else {
                    ActivationReceiptDisposition::InvalidatedPrecommit
                };
                let state = self.activation_receipts.invalidate(
                    &token,
                    disposition,
                    &error,
                    mapping_session_state(adapter.lock().snapshot(), &error),
                );
                Ok(Response::new(state))
            }
        }
    }
}

impl GeometryRuntimeService {
    pub(super) async fn run_planar_relocalization(
        &self,
        request: RelocalizeRequest,
        context: crate::correction_coordinator::PlanarCommandContext,
    ) -> crate::correction_coordinator::PlanarCommandOutcome {
        use crate::correction_coordinator::PlanarCommandOutcome;
        let failed = |detail: String| PlanarCommandOutcome {
            disposition: RelocalizeDisposition::AlgorithmRejected,
            detail,
            correction_revision: 0,
            evidence: Vec::new(),
        };
        let adapter = match self.bridge.mapping_adapter.as_ref() {
            Some(a) => a.clone(),
            None => return failed("二维适配器不可用".into()),
        };
        let binding = match adapter.lock().planar.clone() {
            Some(b) => b,
            None => return failed("二维拥有方不可用".into()),
        };
        let _transition = self.mapping_transition.lock().await;
        let _operation = binding.operation.lock().await;
        let current = adapter.lock().snapshot();
        let current_nonce = binding.client.identity().owner_nonce;
        let mut candidate = None;
        let mut paused = false;
        let mut applied = false;
        let mut failure_disposition = RelocalizeDisposition::AlgorithmRejected;
        let result=async {
            failure_disposition=RelocalizeDisposition::TargetUnavailable;
            let (target,activated)=self.activation_receipts.current_sealed_planar_target().ok_or("二维重定位没有已封存完整图")?;
            failure_disposition=RelocalizeDisposition::TargetInvalidated;
            if request.runtime_generation!=activated.runtime_generation || request.session_id!=activated.session_id || request.map_name!=activated.map_name
                || request.session_generation!=activated.session_generation || request.backend_generation!=activated.backend_generation
                || request.activation_identity_sha256!=activated.activation_identity_sha256 || request.target_receipt_sha256!=activated.target_receipt_sha256
                || request.asset_manifest_sha256!=activated.asset_manifest_sha256 || request.asset_bundle_sha256!=activated.asset_bundle_sha256
                || request.target_graph_sha256.as_slice()!=target.sha || current.session_id!=request.session_id || !current.active {
                return Err("二维重定位目标或会话身份已变化".into());
            }
            failure_disposition=RelocalizeDisposition::OwnerFailed;
            let status=binding.client.status().await?;
            if !status.localization {return Err("二维重定位要求已激活图的定位模式".into());}
            let session=binding.session(&current.session_id,&current.map_name,current.session_generation);
            binding.control_until_settled(Operation::SetPaused{session_id:current.session_id.clone(),session_generation:current.session_generation,paused:true}).await?;
            paused=true;
            crate::planar_navigation::revoke(&self.bridge,&binding,&session).await?;
            let seed=request.planar_seed.as_ref().ok_or("二维重定位初始位置缺失")?;
            let client=binding.prepare_candidate(target.open()?,target.size,target.sha,session.clone(),true,[seed.x,seed.y,seed.yaw]).await?;
            candidate=Some(client.clone());
            let response=client.execute(Operation::SetPaused{session_id:session.session_id.clone(),session_generation:session.session_generation,paused:false}).await?;
            if let Some(e)=response.error {return Err(e);}
            let arm=now_ns();binding.arm_relocalization_input(client.clone(),session.session_generation,context.epoch,arm);
            let mut accepted=Vec::new();let mut last_sequence=None;
            let estimate=loop {
                let epoch=binding.fence.snapshot();
                if !epoch.active || epoch.epoch!=context.epoch || std::time::Instant::now()>=context.deadline {return Err("二维重定位已取消或超过逻辑期限".into());}
                let status=client.status().await?;
                if let Some(estimate)=status.last_estimate {
                    if last_sequence!=Some(estimate.source_sequence) {
                        last_sequence=Some(estimate.source_sequence);
                        if estimate.disposition=="accepted" && estimate.matching_response>=0.3 {
                            if let Some((stamp,sha))=binding.relocation_scan_proof(context.epoch,estimate.source_sequence) {
                                if now_ns().saturating_sub(stamp)<=500_000_000 {
                                    accepted.push((estimate.source_sequence,stamp,sha));
                                    if accepted.len()>=3 {break estimate;}
                                }
                            }
                        } else if estimate.disposition=="match_rejected" {accepted.clear();}
                    }
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            };
            let response=client.execute(Operation::SetPaused{session_id:session.session_id.clone(),session_generation:session.session_generation,paused:true}).await?;
            if let Some(e)=response.error {return Err(e);}
            {
                let _lifecycle=self.bridge.mapping_lifecycle_lock.lock().await;
                binding.clear_relocalization_input(context.epoch);
            }
            let response=client.execute(Operation::SetPaused{session_id:session.session_id.clone(),session_generation:session.session_generation,paused:false}).await?;
            if let Some(e)=response.error {return Err(e);}
            let _projection=self.bridge.lock_explore_projection_owned().await;
            let _lifecycle=self.bridge.mapping_lifecycle_lock.lock().await;
            self.ensure_running_admission("Relocalize").await.map_err(|e|e.to_string())?;
            let latest=self.activation_receipts.current_sealed_planar_target().ok_or("二维重定位目标已退役")?;
            if latest.1.activation_identity_sha256!=activated.activation_identity_sha256 || adapter.lock().snapshot()!=current
                || binding.client.identity().owner_nonce!=current_nonce || binding.fence.snapshot().epoch!=context.epoch
                || now_ns().saturating_sub(binding.settings.unix_stamp(estimate.source_timestamp_ns)?)>500_000_000 {
                return Err("二维重定位提交前来源、会话或目标变化".into());
            }
            if !context.claim().await {return Err("二维重定位未取得原账本提交权".into());}
            binding.promote(&client)?;
            applied=true;
            let transform=adapter.lock().apply_planar_relocalization(&estimate)?;
            adapter.lock().planar_epoch=context.epoch.checked_add(1).ok_or("二维重定位围栏耗尽")?;
            let timestamp=binding.settings.unix_stamp(estimate.source_timestamp_ns)?;
            let odom=PoseState{x:estimate.odom_from_base[0],y:estimate.odom_from_base[1],qz:(estimate.odom_from_base[2]*0.5).sin(),qw:(estimate.odom_from_base[2]*0.5).cos(),
                frame_id:session.odom_frame_id.clone(),timestamp_ns:timestamp,..Default::default()};
            let observation=crate::lcm_runtime::canonical_mapping_observation_from_isometry(&session.map_frame_id,&session.odom_frame_id,
                transform.generation,&transform.effective_isometry(),&odom,&session.body_frame_id)?;
            let mut shared=self.bridge.state.write().await;
            crate::lcm_runtime::apply_canonical_mapping_observation_locked(&mut shared,&observation);
            binding.install_sensor_edge(&mut shared);drop(shared);
            crate::slam2d_owner::publish_tf(&self.bridge,&binding,observation.tf_payload,timestamp).await?;
            let evidence=serde_json::to_vec(&serde_json::json!({"schema_version":1,"target_kind":"planar_graph",
                "request_identity_sha256":context.request_identity,"target_graph_sha256":target.sha,"target_receipt_sha256":request.target_receipt_sha256,
                "runtime_generation":request.runtime_generation,"session_id":request.session_id,"session_generation":request.session_generation,
                "previous_process":current_nonce,"committed_process":client.identity().owner_nonce,"mode_epoch":context.epoch,
                "correction_revision":transform.correction_revision,"frame_generation":transform.generation,"accepted_scans":accepted,
                "source_unix_ns":timestamp,"committed_unix_ns":now_ns(),"estimate":estimate})).map_err(|e|e.to_string())?;
            Ok::<_,String>(PlanarCommandOutcome{disposition:RelocalizeDisposition::Applied,detail:"二维指定位置经三次新扫描匹配并提交".into(),correction_revision:transform.correction_revision,evidence})
        }.await;
        binding.clear_relocalization_input(context.epoch);
        if !applied {
            if let Some(client) = candidate {
                if let Err(e) = binding
                    .discard_candidate(&client.identity().owner_nonce)
                    .await
                {
                    return failed(e);
                }
            }
            if paused {
                if let Err(e) = binding
                    .control_until_settled(Operation::SetPaused {
                        session_id: current.session_id.clone(),
                        session_generation: current.session_generation,
                        paused: false,
                    })
                    .await
                {
                    adapter.lock().planar_ready = false;
                    return failed(e);
                }
            }
            let available = binding
                .client
                .status()
                .await
                .is_ok_and(|status| status.active)
                && self
                    .ensure_running_admission("RelocalizeCleanup")
                    .await
                    .is_ok();
            let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
            let mut a = adapter.lock();
            if a.snapshot() == current {
                a.planar_ready = available;
                let epoch = binding.fence.snapshot();
                a.planar_epoch = if epoch.active {
                    epoch.epoch.saturating_add(1)
                } else {
                    epoch.epoch
                };
            }
        }
        match result {
            Ok(outcome) => outcome,
            Err(e) => {
                if applied {
                    {
                        let mut a = adapter.lock();
                        a.planar_ready = false;
                        a.planar_error = Some(e.clone());
                    }
                    let _ = self.activation_receipts.retire_for_mapping_change();
                    PlanarCommandOutcome {
                        disposition: RelocalizeDisposition::OwnerFailed,
                        detail: e,
                        correction_revision: 0,
                        evidence: Vec::new(),
                    }
                } else {
                    PlanarCommandOutcome {
                        disposition: failure_disposition,
                        detail: e,
                        correction_revision: 0,
                        evidence: Vec::new(),
                    }
                }
            }
        }
    }
}

impl GeometryRuntimeService {
    pub(super) async fn continue_planar_mapping(
        &self,
        request: StartMappingRequest,
        cancellation: watch::Receiver<bool>,
    ) -> Result<Response<MappingSessionState>, Status> {
        let adapter = self
            .bridge
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| Status::unavailable("二维适配器不可用"))?
            .clone();
        let binding = adapter
            .lock()
            .planar
            .clone()
            .ok_or_else(|| Status::unavailable("二维拥有方不可用"))?;
        let _operation = binding.operation.lock().await;
        let epoch = binding.fence.snapshot();
        if epoch.active {
            return Err(Status::failed_precondition("重定位期间不能续建"));
        }
        if self.bridge.explore.blocks_mapping_mutation().await {
            return Err(Status::failed_precondition("探索仍拥有当前地图"));
        }
        let current = adapter.lock().snapshot();
        let status = binding.client.status().await.map_err(Status::unavailable)?;
        let estimate = status
            .last_estimate
            .as_ref()
            .filter(|e| e.disposition != "match_rejected")
            .ok_or_else(|| Status::failed_precondition("二维续建前须通过真实扫描确认当前位置"))?;
        let (generation, backend) = adapter
            .lock()
            .prepare_planar_continuation(&request.session_id, &request.map_name)
            .map_err(Status::failed_precondition)?;
        binding
            .control_until_settled(Operation::SetPaused {
                session_id: current.session_id.clone(),
                session_generation: current.session_generation,
                paused: true,
            })
            .await
            .map_err(Status::unavailable)?;
        let mut candidate = None;
        let mut promoted = false;
        let result =
            async {
                let exported = binding
                    .control_until_settled(Operation::ExportGraph {
                        session_id: current.session_id.clone(),
                        session_generation: current.session_generation,
                        unix_offset_ns: i64::try_from(binding.settings.unix_offset_ns)
                            .map_err(|_| "二维时钟偏移越界")?,
                    })
                    .await?
                    .artifact
                    .ok_or("二维续建缺少完整图")?;
                let file = binding.client.current().open_artifact(&exported).await;
                binding
                    .control_until_settled(Operation::ReleaseArtifact {
                        artifact: exported.clone(),
                    })
                    .await?;
                let mut file = file?;
                let (file, metadata, cells) = tokio::task::spawn_blocking(move || {
                    let metadata = archive::verify(&mut file)?;
                    let (_, cells) = archive::grid(&mut file)?;
                    Ok::<_, String>((file, metadata, cells))
                })
                .await
                .map_err(|e| e.to_string())??;
                let session = binding.session(&request.session_id, &request.map_name, generation);
                let client = binding
                    .prepare_candidate(
                        file,
                        exported.payload_size,
                        exported.sha256,
                        session.clone(),
                        false,
                        estimate.map_from_base,
                    )
                    .await?;
                candidate = Some(client.clone());
                let mut graph_estimate = metadata.estimate;
                graph_estimate.graph_revision = 1;
                let snapshot = Snapshot {
                    snapshot_id: 1,
                    session: session.clone(),
                    estimate: graph_estimate,
                    width: metadata.width,
                    height: metadata.height,
                    resolution: metadata.resolution,
                    origin: metadata.origin,
                    payload_size: metadata.members[2].size,
                    data_sha256: metadata.members[2].sha256,
                    file_name: String::new(),
                };
                let preparation = adapter.lock().prepare_planar_activation(
                    snapshot,
                    cells,
                    metadata.source_unix_timestamp_ns,
                    backend,
                );
                let prepared = tokio::task::spawn_blocking(move || preparation.compute())
                    .await
                    .map_err(|e| e.to_string())??;
                let response = client
                    .execute(Operation::SetPaused {
                        session_id: session.session_id,
                        session_generation: generation,
                        paused: false,
                    })
                    .await?;
                if let Some(e) = response.error {
                    return Err(e);
                }
                let mut projection = self.bridge.lock_explore_projection_owned().await;
                let _lifecycle = self.bridge.mapping_lifecycle_lock.lock().await;
                self.ensure_running_admission("StartMapping")
                    .await
                    .map_err(|e| e.to_string())?;
                if *cancellation.borrow()
                    || binding.fence.snapshot() != epoch
                    || adapter.lock().snapshot() != current
                {
                    return Err("二维续建已取消或会话围栏变化".into());
                }
                crate::planar_navigation::revoke(
                    &self.bridge,
                    &binding,
                    &binding.session(
                        &current.session_id,
                        &current.map_name,
                        current.session_generation,
                    ),
                )
                .await?;
                self.activation_receipts.retire_for_mapping_change()?;
                if let Some(writer) = self.bridge.occupancy_shm.as_ref() {
                    writer
                        .lock()
                        .begin_session(generation)
                        .map_err(|e| e.to_string())?;
                }
                adapter
                    .lock()
                    .start_planar_continuation(&request.session_id, &request.map_name)?;
                promoted = true;
                binding.promote(&client)?;
                self.bridge
                    .revoke_committed_map_projection_locked(&mut projection)?;
                {
                    let mut a = adapter.lock();
                    a.planar_ready = true;
                    a.planar_epoch = epoch.epoch;
                }
                let mut shared = self.bridge.state.write().await;
                let (map, event) = adapter
                    .lock()
                    .commit_planar_snapshot(prepared, shared.map.revision)?;
                shared.map = map;
                shared.pose = Default::default();
                shared.transforms.clear();
                shared.tf_tree = Arc::new(nuwax_tf::TfTree::new());
                binding.install_sensor_edge(&mut shared);
                shared.pose_changed.send_replace(());
                drop(shared);
                *self.bridge.occupancy_shm_current.write().await = None;
                let _ = self.bridge.map_revision_tx.send(event);
                let snapshot = adapter.lock().snapshot();
                Ok::<_, String>(self.mapping_state_with_receipt(
                    snapshot,
                    "二维图已进入续建，新会话等待真实扫描匹配",
                ))
            }
            .await;
        if result.is_err() {
            if !promoted {
                if let Some(client) = candidate {
                    binding
                        .discard_candidate(&client.identity().owner_nonce)
                        .await
                        .map_err(Status::unavailable)?;
                }
                if status.active {
                    binding
                        .control_until_settled(Operation::SetPaused {
                            session_id: current.session_id,
                            session_generation: current.session_generation,
                            paused: false,
                        })
                        .await
                        .map_err(Status::unavailable)?;
                }
            } else {
                let _ = adapter
                    .lock()
                    .force_stop_started_session_detached(&request.session_id, generation);
                adapter.lock().planar_ready = false;
            }
        }
        result
            .map(Response::new)
            .map_err(Status::failed_precondition)
    }
}
