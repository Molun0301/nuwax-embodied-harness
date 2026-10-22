//! 二维图沿公共激活账本、规范地图提交及同一进程拥有方完成切换。
use super::*;
use nuwax_slam_2d_runtime::{
    archive,
    protocol::{Operation, Snapshot},
};

impl GeometryRuntimeService {
    pub(super) async fn activate_planar_map(
        &self,
        mut request: ActivateMapRequest,
        class: ActivationRequestIdentityClass,
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
        let assigned_owner = {
            let metadata = &class;
            let current = mapping_session_state(adapter.lock().snapshot(), "二维编号激活准入");
            if let Some(terminal) = self.activation_receipts.assigned_terminal_before_admission(metadata, current) {
                return Ok(Response::new(terminal));
            }
            let session_id = request.session_id.clone();
            let map_name = request.map_name.clone();
            let admitted = crate::activation_request::ValidatedActivationRequestV2::admit(
                std::mem::take(&mut request), self.activation_receipts.runtime_generation(),
                crate::mapping_backend::MappingBackendKind::SlamToolbox, &binding.settings.template.map_frame_id,
                |reference| self.graph_artifacts.lock().open(reference), &self.activation_checkpoint_owners,
            ).map_err(Status::invalid_argument)?;
            request.session_id = session_id;
            request.map_name = map_name;
            admitted.into_parts().1
        };
        let graph = assigned_owner.graph().ok_or_else(|| Status::invalid_argument("二维编号检查点缺少图拥有方"))?;
        let occupancy = assigned_owner.occupancy();
        let source_session_id = assigned_owner.occupancy_receipt().session_id.as_str();
        let source_session_generation = assigned_owner.occupancy_receipt().session_generation;
        let reference = graph.reference();
        if occupancy.meta.as_ref().is_none_or(|meta| meta.frame_id != binding.settings.template.map_frame_id) {
            return Err(Status::invalid_argument("二维图坐标系与运行时不兼容"));
        }
        let request_key = class.clone();
        let _transition = self.mapping_transition.lock().await;
        let _operation = binding.operation.lock().await;
        self.ensure_running_admission("ActivateMap").await?;
        let epoch = binding.fence.snapshot();
        if epoch.active || self.bridge.explore.blocks_mapping_mutation().await {
            return Err(Status::failed_precondition("二维图切换被重定位或探索占用"));
        }
        let current = adapter.lock().snapshot();
        let mut seed = crate::correction_coordinator::ActivationReceiptSeed {
            activation_request_id: class.activation_request_id().into(),
            request_key: request_key.clone(),
            checkpoint_owner: Some(assigned_owner.clone()),
            planar_target: None,
        };
        if self.activation_receipts.has_request(class.activation_request_id()) {
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
        let reference = reference.clone();
        use std::os::fd::AsRawFd;
        let mut file = std::fs::File::open(format!("/proc/self/fd/{}", assigned_owner.graph().unwrap().file().as_raw_fd()))
            .map_err(|error| Status::internal(error.to_string()))?;
        let (file, metadata, cells) = tokio::task::spawn_blocking(move || {
            let (metadata, cells) = archive::grid_structure(&mut file)?;
            Ok::<_, String>((file, metadata, cells))
        })
        .await
        .map_err(|e| Status::internal(e.to_string()))?
        .map_err(Status::invalid_argument)?;
        let meta = occupancy.meta.as_ref().unwrap();
        let source_session = match &metadata {
            archive::ArchiveMetadata::Legacy(source) => &source.session,
            archive::ArchiveMetadata::Assigned(assigned) => &assigned.source.session,
        };
        if source_session.session_id != source_session_id
            || source_session.session_generation != source_session_generation
            || metadata.dimensions().0 != meta.width
            || metadata.dimensions().1 != meta.height
            || metadata.resolution() as f32 != meta.resolution
            || metadata.origin() != [meta.origin_x, meta.origin_y]
            || metadata.source_unix_timestamp_ns() != meta.timestamp_ns
            || !cells.iter().map(|v| *v as u8).eq(occupancy.data.iter().copied())
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
            identity: crate::planar_artifacts::TargetIdentity::Assigned(reference.clone()),
            metadata: metadata.clone(),
        });
        let candidate = binding
            .prepare_candidate(
                target.open().map_err(Status::internal)?,
                target.size,
                target.identity.runtime_identity(),
                session.clone(),
                true,
                metadata.estimate().map_from_base,
            )
            .await
            .map_err(Status::failed_precondition)?;
        let mut estimate = metadata.estimate().clone();
        estimate.graph_revision = 1;
        let snapshot = Snapshot {
            snapshot_id: 1,
            session: session.clone(),
            estimate,
            width: metadata.dimensions().0,
            height: metadata.dimensions().1,
            resolution: metadata.resolution(),
            origin: metadata.origin(),
            payload_size: u64::from(metadata.dimensions().0) * u64::from(metadata.dimensions().1),
            file_name: String::new(),
        };
        let preparation = adapter.lock().prepare_planar_activation(
            snapshot,
            cells,
            metadata.source_unix_timestamp_ns(),
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
            let identity = InstalledActivationIdentityV2 {
                request: class.clone(), runtime_generation: self.activation_receipts.runtime_generation(),
                lifecycle_epoch: self.activation_receipts.lifecycle_epoch(), session_generation,
                backend_kind: crate::mapping_backend::MappingBackendKind::SlamToolbox, backend_generation,
            };
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
            adapter.lock().record_checkpoint_activation(request_key.clone())?;
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
                .store_planar_proof(&token, receipt.clone())
                .ok_or("二维目标收据绑定失败")?;
            let _ = self.bridge.map_revision_tx.send(event);
            let mut state = mapping_session_state(
                adapter.lock().snapshot(),
                "二维完整图已激活，等待真实扫描定位",
            );
            state.planar_mode = "localization".into();
            let sealed = self
                .activation_receipts
                .seal(&token, target_receipt, state)
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
            evidence: None,
        };
        let identity = context.request_identity.clone();
        if crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2::admit(request)
            .is_ok_and(|request| request == identity && request.is_planar()) == false {
            return failed("二维重定位编号请求不一致".into());
        }
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
            let installed = activated.activation_receipt_v2.as_ref().and_then(|receipt| receipt.target.as_ref());
            let activation = &identity.target().activation;
            if installed != Some(&identity.target().to_wire())
                || !matches!(&target.identity, crate::planar_artifacts::TargetIdentity::Assigned(reference)
                    if reference.member_identity.as_ref() == identity.target().to_wire().source_graph_member.as_ref())
                || current.session_id != activation.request.session_id()
                || current.map_name != activation.request.map_name()
                || current.session_generation != activation.session_generation || !current.active {
                return Err("二维重定位目标或会话身份已变化".into());
            }
            failure_disposition=RelocalizeDisposition::OwnerFailed;
            let status=binding.client.status().await?;
            if !status.localization {return Err("二维重定位要求已激活图的定位模式".into());}
            let session=binding.session(&current.session_id,&current.map_name,current.session_generation);
            binding.control_until_settled(Operation::SetPaused{session_id:current.session_id.clone(),session_generation:current.session_generation,paused:true}).await?;
            paused=true;
            crate::planar_navigation::revoke(&self.bridge,&binding,&session).await?;
            let seed=identity.planar_seed().ok_or("二维重定位初始位置缺失")?;
            let client=binding.prepare_candidate(target.open()?,target.size,target.identity.runtime_identity(),session.clone(),true,[seed.x,seed.y,seed.yaw]).await?;
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
                            if let Some(source)=binding.relocation_scan_proof(context.epoch,session.session_generation,&client.identity().owner_nonce,estimate.source_sequence) {
                                if source.source_timestamp_ns == estimate.source_timestamp_ns && now_ns().saturating_sub(source.source_unix_ns)<=500_000_000 {
                                    accepted.push(PlanarRelocalizationScanV2 {
                                        process_owner: source.process_owner, session_generation: source.session_generation,
                                        mode_epoch: source.mode_epoch, source_sequence: source.source_sequence,
                                        source_timestamp_ns: source.source_timestamp_ns, source_unix_ns: source.source_unix_ns,
                                        matching_response_bits: estimate.matching_response.to_bits(),
                                    });
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
            if latest.1.activation_receipt_v2!=activated.activation_receipt_v2 || !Arc::ptr_eq(&latest.0,&target) || adapter.lock().snapshot()!=current
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
            let evidence = AssignedPlanarRelocalizationEvidenceV2 {
                schema_version: 2, phase: AssignedRelocalizationEvidencePhaseV2::Finalized as i32,
                previous_process: current_nonce.clone(), committed_process: client.identity().owner_nonce.clone(),
                mode_epoch: context.epoch, accepted_scans: accepted,
                estimate: Some(planar_relocalization_estimate(&estimate)), frame_generation: transform.generation,
                source_unix_ns: timestamp, committed_unix_ns: now_ns(), ..Default::default()
            };
            Ok::<_,String>(PlanarCommandOutcome{disposition:RelocalizeDisposition::Applied,detail:"二维指定位置经三次新扫描匹配并提交".into(),correction_revision:transform.correction_revision,evidence:Some(evidence)})
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
                        evidence: None,
                    }
                } else {
                    PlanarCommandOutcome {
                        disposition: failure_disposition,
                        detail: e,
                        correction_revision: 0,
                        evidence: None,
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
                    let (metadata, cells) = archive::grid_structure(&mut file)?;
                    Ok::<_, String>((file, metadata, cells))
                })
                .await
                .map_err(|e| e.to_string())??;
                let session = binding.session(&request.session_id, &request.map_name, generation);
                let client = binding
                    .prepare_candidate(
                        file,
                        exported.payload_size,
                        exported.identity.clone(),
                        session.clone(),
                        false,
                        estimate.map_from_base,
                    )
                    .await?;
                candidate = Some(client.clone());
                let mut graph_estimate = metadata.estimate().clone();
                graph_estimate.graph_revision = 1;
                let snapshot = Snapshot {
                    snapshot_id: 1,
                    session: session.clone(),
                    estimate: graph_estimate,
                    width: metadata.dimensions().0,
                    height: metadata.dimensions().1,
                    resolution: metadata.resolution(),
                    origin: metadata.origin(),
                    payload_size: u64::from(metadata.dimensions().0) * u64::from(metadata.dimensions().1),
                            file_name: String::new(),
                };
                let preparation = adapter.lock().prepare_planar_activation(
                    snapshot,
                    cells,
                    metadata.source_unix_timestamp_ns(),
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

fn planar_relocalization_estimate(value: &nuwax_slam_2d_runtime::protocol::Estimate) -> PlanarRelocalizationEstimateV2 {
    PlanarRelocalizationEstimateV2 {
        source_timestamp_ns: value.source_timestamp_ns, clock_domain: value.clock_domain.clone(), clock_id: value.clock_id.clone(),
        source_sequence: value.source_sequence, graph_revision: value.graph_revision, optimizations: value.optimizations,
        anchor_odom_from_base_bits: value.anchor_odom_from_base.map(f64::to_bits).to_vec(),
        odom_from_base_bits: value.odom_from_base.map(f64::to_bits).to_vec(),
        map_from_base_bits: value.map_from_base.map(f64::to_bits).to_vec(),
        map_from_odom_bits: value.map_from_odom.map(f64::to_bits).to_vec(),
        covariance_bits: value.covariance.map(f64::to_bits).to_vec(), matching_response_bits: value.matching_response.to_bits(),
        local_response_bits: value.local_response.to_bits(), disposition: value.disposition.clone(), node_count: value.node_count, edge_count: value.edge_count,
    }
}

impl GeometryRuntimeService {
    /// Caller holds the original lifecycle fence. Read the live native-backed
    /// shared observation; a retained terminal never supplies a substitute pose.
    pub(super) async fn assigned_planar_localization_sample(
        &self, terminal: &crate::relocalization_terminal_v2::AssignedRelocalizationTerminalV2,
    ) -> Result<AssignedLocalizationSampleV2, String> {
        let body = terminal.result().assigned_planar_v2.as_ref().ok_or("ASSIGNED_PLANAR_SAMPLE_TERMINAL_ABSENT")?;
        if !matches!(RelocalizeDisposition::try_from(body.disposition), Ok(RelocalizeDisposition::Applied | RelocalizeDisposition::AlreadyApplied)) {
            return Err("ASSIGNED_PLANAR_SAMPLE_TERMINAL_NOT_APPLIED".into());
        }
        let evidence = body.evidence.as_ref().ok_or("ASSIGNED_PLANAR_SAMPLE_EVIDENCE_ABSENT")?;
        let owner = evidence.owner.as_ref().ok_or("ASSIGNED_PLANAR_SAMPLE_OWNER_ABSENT")?;
        let activation = &terminal.request().target().activation;
        let adapter = self.bridge.mapping_adapter.as_ref().ok_or("ASSIGNED_PLANAR_SAMPLE_MAPPING_ABSENT")?;
        let binding = adapter.lock().planar.clone().ok_or("ASSIGNED_PLANAR_SAMPLE_PROCESS_ABSENT")?;
        let current_owner = binding.client.identity();
        let fence = binding.fence.snapshot();
        if fence.active || Some(fence.epoch) != evidence.mode_epoch.checked_add(1)
            || current_owner.owner_nonce != evidence.committed_process
            || !self.relocalization_ingress.as_ref().is_some_and(|ingress| ingress.owns_receipt(owner)) {
            return Err("ASSIGNED_PLANAR_SAMPLE_OWNER_REPLACED".into());
        }
        let (_, installed) = self.activation_receipts.current_sealed_planar_target().ok_or("ASSIGNED_PLANAR_SAMPLE_TARGET_RETIRED")?;
        if installed.activation_receipt_v2.as_ref().and_then(|receipt| receipt.target.as_ref()) != Some(&terminal.request().target().to_wire()) {
            return Err("ASSIGNED_PLANAR_SAMPLE_TARGET_REPLACED".into());
        }
        let status = binding.client.status().await?;
        if !status.active || !status.localization || status.paused || status.stopping || status.last_error.is_some()
            || status.session.as_ref().is_none_or(|session| session.session_id != activation.request.session_id()
                || session.map_name != activation.request.map_name() || session.session_generation != activation.session_generation
                || session.map_frame_id != owner.map_frame_id || session.odom_frame_id != owner.odom_frame_id || session.body_frame_id != owner.body_frame_id)
            || status.last_estimate.as_ref().is_none_or(|estimate| estimate.disposition == "match_rejected") {
            return Err("ASSIGNED_PLANAR_SAMPLE_NATIVE_OWNER_UNAVAILABLE".into());
        }
        let shared = self.bridge.state.read().await;
        {
            let mapping = adapter.lock();
            let session = mapping.snapshot();
            let registration = mapping.map_odom_transform_snapshot().ok_or("ASSIGNED_PLANAR_SAMPLE_REGISTRATION_ABSENT")?;
            let receipt = mapping.latest_occupancy_commit_receipt().ok_or("ASSIGNED_PLANAR_SAMPLE_RECEIPT_ABSENT")?;
            if !session.active || !mapping.planar_ready || !mapping.planar_localization || mapping.planar_epoch != fence.epoch
                || session.session_id != activation.request.session_id() || session.map_name != activation.request.map_name()
                || session.session_generation != activation.session_generation || receipt.backend_generation != activation.backend_generation
                || mapping.global_frame_generation() != evidence.frame_generation || registration.generation != evidence.frame_generation
                || registration.correction_revision != body.correction_revision || registration.map_frame_id != owner.map_frame_id
                || registration.odom_frame_id != owner.odom_frame_id {
                return Err("ASSIGNED_PLANAR_SAMPLE_MAPPING_OWNER_REPLACED".into());
            }
        }
        let map_edge = shared.tf_tree.lookup(&owner.odom_frame_id).ok().and_then(crate::lcm_runtime::transform_to_entry)
            .ok_or("ASSIGNED_PLANAR_SAMPLE_MAP_TF_ABSENT")?;
        let odom_edge = shared.tf_tree.lookup(&owner.body_frame_id).ok().and_then(crate::lcm_runtime::transform_to_entry)
            .ok_or("ASSIGNED_PLANAR_SAMPLE_ODOM_TF_ABSENT")?;
        let pose = &shared.pose;
        let now = now_ns();
        if pose.timestamp_ns == 0 || pose.timestamp_ns < evidence.source_unix_ns
            || pose.timestamp_ns > now.saturating_add(50_000_000) || now.saturating_sub(pose.timestamp_ns) > 500_000_000
            || pose.frame_id != owner.map_frame_id || shared.map.frame_id != owner.map_frame_id
            || map_edge.parent != owner.map_frame_id || map_edge.child != owner.odom_frame_id
            || odom_edge.parent != owner.odom_frame_id || odom_edge.child != owner.body_frame_id
            || map_edge.stamp_ns != pose.timestamp_ns || odom_edge.stamp_ns != pose.timestamp_ns
            || binding.client.identity() != current_owner || binding.fence.snapshot() != fence
            || !self.relocalization_ingress.as_ref().is_some_and(|ingress| ingress.owns_receipt(owner)) {
            return Err("ASSIGNED_PLANAR_SAMPLE_POSE_OR_TF_STALE".into());
        }
        let transform = |edge: crate::lcm_runtime::TransformEntry| Transform {
            parent_frame: edge.parent, child_frame: edge.child,
            translation: Some(Vec3 { x: edge.tx, y: edge.ty, z: edge.tz }),
            rotation: Some(Quat { x: edge.rx, y: edge.ry, z: edge.rz, w: edge.rw }), timestamp_ns: edge.stamp_ns,
        };
        Ok(AssignedLocalizationSampleV2 {
            schema_version: 2, terminal: Some(terminal.to_wire()),
            pose: Some(Pose { position: Some(Vec3 { x: pose.x, y: pose.y, z: pose.z }),
                orientation: Some(Quat { x: pose.qx, y: pose.qy, z: pose.qz, w: pose.qw }),
                frame_id: pose.frame_id.clone(), timestamp_ns: pose.timestamp_ns }),
            map_from_odom: Some(transform(map_edge)), odom_from_body: Some(transform(odom_edge)),
            correction_owner: Some(owner.clone()), correction_revision: body.correction_revision,
            map_frame_generation: evidence.frame_generation,
        })
    }
}
