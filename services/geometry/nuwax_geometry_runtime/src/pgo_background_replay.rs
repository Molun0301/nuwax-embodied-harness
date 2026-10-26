//! 普通PGO沿原拥有方推进磁盘重建；等待写盘和计算期间不持生命周期锁。
use super::*;
use crate::correction_coordinator::{
    CanonicalCorrectionApplyError as ApplyError, CanonicalCorrectionTransaction,
    CorrectionRejectReason as Reject, CorrectionRequestCancellation,
};
use crate::mapping_backend::{MappingBackendComputeConfig, MappingBackendSpec};
use crate::pgo_map::{DiskReplayInput, RebuiltMap};

impl GeometryCanonicalCorrectionApplier {
    pub(super) async fn prepare_background_disk_pgo(
        &self,
        transaction: &CanonicalCorrectionTransaction,
        cancellation: &CorrectionRequestCancellation,
        context: PgoApplyContext,
        input: DiskReplayInput,
        spec: MappingBackendSpec,
        config: MappingBackendComputeConfig,
        backend_generation: u64,
    ) -> Result<(RebuiltMap, (u64, u64, u64)), ApplyError> {
        let tail = || {
            let mapping = context.mapping.lock();
            let session = mapping.snapshot();
            if !session.active
                || session.session_id != transaction.mapping_session_id
                || session.session_generation != transaction.mapping_session_generation
            {
                return Err(ApplyError::PreparationRejected(
                    Reject::MappingSessionMismatch,
                ));
            }
            if mapping.global_frame_generation() != transaction.map_frame_generation {
                return Err(ApplyError::PreparationRejected(
                    Reject::MapFrameGenerationMismatch,
                ));
            }
            let anchor = mapping
                .map_odom_transform_snapshot()
                .ok_or_else(|| ApplyError::Failed("PGO地图锚点缺失".into()))?;
            if anchor.correction_revision.checked_add(1) != Some(transaction.correction_revision) {
                return Err(ApplyError::PreparationRejected(Reject::OutOfOrderCandidate));
            }
            if !canonical_isometry_near(
                &anchor.base_isometry(),
                &transaction.base_anchor_map_from_odom,
            ) {
                return Err(ApplyError::Failed("PGO地图锚点不匹配".into()));
            }
            let (_, _, _, generation) = mapping
                .pgo_replay_input_since(u64::MAX)
                .map_err(pgo_preparation_error)?;
            if generation != backend_generation {
                return Err(ApplyError::PreparationRejected(
                    Reject::BackendGenerationMismatch,
                ));
            }
            mapping
                .pgo_disk_tail()
                .map_err(pgo_preparation_error)?
                .ok_or_else(|| ApplyError::Failed("PGO磁盘帧来源已改变".into()))
        };
        let terminal = || {
            context
                .cloud_writer
                .lock()
                .as_ref()
                .map(|writer| writer.pgo_terminal_dufo())
                .ok_or_else(|| ApplyError::Failed("PGO点云拥有方缺失".into()))
        };
        let start = |initial: DiskReplayInput, tree: crate::dynamic_backend::OnlineDufo| {
            let trajectory = transaction.trajectory.clone();
            let anchor = transaction.base_anchor_map_from_odom;
            let frame = self.map_frame_id.clone();
            let cancelled = cancellation.clone();
            tokio::task::spawn_blocking(move || {
                initial
                    .start(
                        trajectory,
                        anchor,
                        spec,
                        config,
                        frame,
                        Duration::MAX,
                        tree,
                        &|| cancelled.is_cancelled(),
                    )
                    .map(|(builder, cursor)| (builder.into_background(), cursor))
            })
        };
        let tree = terminal()?;
        let mut evidence_revision = tree.free_revision();
        let (mut builder, mut cursor) = start(input.clone(), tree)
            .await
            .map_err(|e| ApplyError::Failed(format!("PGO磁盘准备任务失败：{e}")))?
            .map_err(pgo_preparation_error)?;
        let mut prepared = None;
        loop {
            builder
                .check(&|| cancellation.is_cancelled())
                .map_err(pgo_preparation_error)?;
            let latest = tail()?;
            let current_revision = context
                .cloud_writer
                .lock()
                .as_ref()
                .ok_or_else(|| ApplyError::Failed("PGO点云拥有方缺失".into()))?
                .pgo_free_revision();
            if current_revision != evidence_revision {
                // 无关新增空单元不重置进度；只有可能撤销已重放静态点或变化缺口才重读。
                let (tree, changes) = context
                    .cloud_writer
                    .lock()
                    .as_ref()
                    .ok_or_else(|| ApplyError::Failed("PGO点云拥有方缺失".into()))?
                    .pgo_terminal_dufo_since(evidence_revision);
                evidence_revision = tree.free_revision();
                if let Err(tree) = builder.update_terminal(tree, changes.as_deref()) {
                    let initial = input.clone();
                    let trajectory = transaction.trajectory.clone();
                    let anchor = transaction.base_anchor_map_from_odom;
                    let frame = self.map_frame_id.clone();
                    let cancelled = cancellation.clone();
                    (builder, cursor) = tokio::task::spawn_blocking(move || {
                        drop(builder);
                        drop(cursor);
                        initial
                            .start(
                                trajectory,
                                anchor,
                                spec,
                                config,
                                frame,
                                Duration::MAX,
                                tree,
                                &|| cancelled.is_cancelled(),
                            )
                            .map(|(builder, cursor)| (builder.into_background(), cursor))
                    })
                    .await
                    .map_err(|e| ApplyError::Failed(format!("PGO证据更新准备失败：{e}")))?
                    .map_err(pgo_preparation_error)?;
                    prepared = None;
                }
            }
            let previous = builder.latest_timestamp_ns();
            let cancelled = cancellation.clone();
            let (next_cursor, next_builder, ready) = tokio::task::spawn_blocking(move || {
                cursor.append_slice(builder, &latest, &|| cancelled.is_cancelled())
            })
            .await
            .map_err(|e| ApplyError::Failed(format!("PGO磁盘分片失败：{e}")))?
            .map_err(pgo_preparation_error)?;
            cursor = next_cursor;
            builder = next_builder;
            if builder.latest_timestamp_ns() != previous {
                prepared = None;
            }
            if !ready {
                tokio::select! {
                    biased;
                    _ = cancellation.cancelled() => return Err(ApplyError::Cancelled),
                    _ = tokio::time::sleep(crate::pgo_map::REPLAY_YIELD) => {},
                }
                continue;
            }
            if prepared.is_none() {
                let cancelled = cancellation.clone();
                let (advanced, output) = tokio::task::spawn_blocking(move || {
                    builder.advance(&[], &|| cancelled.is_cancelled())
                })
                .await
                .map_err(|e| ApplyError::Failed(format!("PGO磁盘物化失败：{e}")))?
                .map_err(pgo_preparation_error)?;
                builder = advanced;
                prepared = Some(output);
            }
            let guard = tokio::select! {
                biased;
                _ = cancellation.cancelled() => return Err(ApplyError::Cancelled),
                guard = context.lifecycle_lock.clone().lock_owned() => guard,
            };
            let latest = tail()?;
            let current_revision = context
                .cloud_writer
                .lock()
                .as_ref()
                .ok_or_else(|| ApplyError::Failed("PGO点云拥有方缺失".into()))?
                .pgo_free_revision();
            let output = prepared.as_ref().unwrap();
            if latest.source_timestamp_ns == output.source_timestamp_ns
                && current_revision == evidence_revision
                && context
                    .mapping
                    .lock()
                    .pgo_replay_tail_is_current(output.source_timestamp_ns)
            {
                // 完整性可以在写盘队列溢出时立即失效，提交边界最后再复核一次。
                input.source.check_complete().map_err(|_| {
                    pgo_preparation_error(crate::pgo_map::RebuildError::HistoryUnavailable)
                })?;
                let expected = {
                    let shared = self.state.read().await;
                    (
                        shared.map.revision,
                        shared.map.timestamp_ns,
                        shared.cloud_map_revision,
                    )
                };
                let rebuilt = builder
                    .finish(prepared.take().unwrap(), &|| cancellation.is_cancelled())
                    .map_err(pgo_preparation_error)?;
                cancellation.retain_background_lifecycle_guard(guard);
                return Ok((rebuilt, expected));
            }
            drop(guard);
            tokio::select! {
                biased;
                _ = cancellation.cancelled() => return Err(ApplyError::Cancelled),
                _ = tokio::time::sleep(crate::pgo_map::REPLAY_YIELD) => {},
            }
        }
    }
}
