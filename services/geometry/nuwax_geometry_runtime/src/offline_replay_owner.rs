//! Geometry 库私有离线重放 owner。

use std::time::{SystemTime, UNIX_EPOCH};

use nalgebra::Point3;
use nuwax_mapping::OccupancyGrid;

use super::mapping_backend::{
    MappingBackendCandidate, MappingBackendCompletion, MappingBackendComputeConfig,
    MappingBackendInput, MappingBackendKind, MappingBackendSpec, MappingBackendWorker,
    MappingComputeToken,
};
use super::mapping_commit::{MappingCommitClock, MappingCommitId};
use super::offline_canonical_cloud::{
    OfflineCanonicalCloud, OfflineCanonicalCloudSnapshot,
    OFFLINE_CANONICAL_CLOUD_PAYLOAD_SCHEMA_VERSION,
};
use super::offline_replay::{
    OfflineBackendConfigV1, OfflineBackendKind, OfflineGeometryFacade, OfflineGeoreferenceV1,
    OfflineMapSnapshotSeedV2, OfflineReplayFrameV1, OfflineReplayIdentityV2,
    OfflineReplayRequestV1, OfflineReplayValidator, ValidatedOfflineReplayFrameV1,
};
use super::persistent_occupancy::PersistentOccupancyGrid;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplayBarrierIdentityV1 {
    pub replay_id: String,
    pub backend_generation: u64,
    pub last_source_sequence: u64,
    pub mapping_commit_sequence: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OfflineReplayProcessingReceiptV1 {
    pub barrier: ReplayBarrierIdentityV1,
    pub committed_frame_count: u64,
    pub committed_point_count: u64,
    pub flush_committed: bool,
}

#[derive(Clone, Debug)]
pub struct OfflineGeometrySnapshotV1 {
    pub barrier: ReplayBarrierIdentityV1,
    pub checkpoint: Vec<u8>,
    pub committed_transformed_accepted_cloud: Vec<Point3<f64>>,
    pub backend_occupancy_observation: Option<OccupancyGrid>,
    pub processing_receipt: OfflineReplayProcessingReceiptV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OfflineCanonicalCommitIdV1 {
    pub boot_nonce: u128,
    pub sequence: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OfflineCanonicalOccupancyReceiptV1 {
    pub commit_id: OfflineCanonicalCommitIdV1,
    pub session_id: String,
    pub session_generation: u64,
    pub occupancy_revision: u64,
    pub source_timestamp_ns: u64,
    pub backend_kind: OfflineBackendKind,
    pub backend_generation: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OfflineCanonicalCloudReceiptV1 {
    pub commit_id: OfflineCanonicalCommitIdV1,
    pub session_generation: u64,
    pub occupancy_revision: u64,
    pub cloud_revision: u64,
    pub source_timestamp_ns: u64,
    pub point_count: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OfflineCanonicalOccupancyAssetProjectionV2 {
    pub raw_data_size: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub frame_id: String,
    pub source_occupancy_revision: u64,
    pub timestamp_ns: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OfflineCanonicalCloudAssetProjectionV2 {
    pub xyz_f32_le: Vec<u8>,
    pub xyz_size: u64,
    pub frame_id: String,
    pub source_cloud_revision: u64,
    pub point_count: u32,
    pub voxel_size: f32,
    pub payload_schema_version: u32,
}

/// Receipt-linked export view: the backend checkpoint is taken after processing
/// flush; occupancy/cloud still describe the last canonical commit, not the same
/// serialized backend state.
#[derive(Clone, Debug, PartialEq)]
pub struct OfflineCanonicalCheckpointProjectionV2 {
    pub schema_version: u32,
    pub created_at_ns: u64,
    pub source_session_id: String,
    pub source_session_generation: u64,
    pub backend_kind: OfflineBackendKind,
    /// Backend checkpoint exported after the processing flush, associated with this commit.
    pub checkpoint_size: u64,
    pub occupancy_receipt: OfflineCanonicalOccupancyReceiptV1,
    pub cloud_receipt: OfflineCanonicalCloudReceiptV1,
    pub occupancy_asset: OfflineCanonicalOccupancyAssetProjectionV2,
    pub cloud_asset: OfflineCanonicalCloudAssetProjectionV2,
}

#[derive(Clone, Debug)]
pub struct OfflineGeometrySnapshotV3 {
    pub schema_version: u32,
    pub barrier: ReplayBarrierIdentityV1,
    pub identity: OfflineReplayIdentityV2,
    pub backend_kind: OfflineBackendKind,
    pub backend_config: OfflineBackendConfigV1,
    pub georeference: OfflineGeoreferenceV1,
    pub last_source_timestamp_ns: u64,
    pub checkpoint: Vec<u8>,
    pub canonical_raw_occupancy: OccupancyGrid,
    pub committed_transformed_accepted_cloud: Vec<Point3<f64>>,
    pub processing_receipt: OfflineReplayProcessingReceiptV1,
    pub map_snapshot_seed: OfflineMapSnapshotSeedV2,
    pub canonical_checkpoint_projection: OfflineCanonicalCheckpointProjectionV2,
}

struct FinishedReplayBarrier {
    v3: OfflineGeometrySnapshotV3,
    backend_occupancy_observation: Option<OccupancyGrid>,
}

pub struct GeometryOfflineReplayOwner {
    validator: Option<OfflineReplayValidator>,
    worker: Option<MappingBackendWorker>,
    client: super::mapping_backend::MappingBackendClient,
    replay_id: String,
    backend_generation: u64,
    backend_kind: OfflineBackendKind,
    map_frame_id: String,
    accepted_cloud: Vec<Point3<f64>>,
    occupancy_observation: Option<OccupancyGrid>,
    canonical_occupancy: Option<PersistentOccupancyGrid>,
    commit_clock: MappingCommitClock,
    occupancy_revision: u64,
    canonical_cloud: OfflineCanonicalCloud,
    last_occupancy_receipt: Option<OfflineCanonicalOccupancyReceiptV1>,
    last_cloud_receipt: Option<OfflineCanonicalCloudReceiptV1>,
    last_cloud_snapshot: Option<OfflineCanonicalCloudSnapshot>,
    committed_frame_count: u64,
    intake_open: bool,
}

impl GeometryOfflineReplayOwner {
    pub fn start(request: OfflineReplayRequestV1) -> Result<Self, String> {
        let replay_id = format!("{}:{}", request.map_name, request.source_id);
        let validator = OfflineGeometryFacade::start(request).map_err(|error| error.to_string())?;
        let backend_kind = validator.request().backend_kind();
        let map_frame_id = validator.request().map_frame_id().to_owned();
        let (spec, config) = backend_config(validator.request().backend_config())?;
        let backend_generation = 1;
        let worker = MappingBackendWorker::start(spec, backend_generation, config)?;
        let client = worker.client();
        Ok(Self {
            validator: Some(validator),
            worker: Some(worker),
            client,
            replay_id,
            backend_generation,
            backend_kind,
            map_frame_id,
            accepted_cloud: Vec::new(),
            occupancy_observation: None,
            canonical_occupancy: None,
            commit_clock: MappingCommitClock::new(),
            occupancy_revision: 0,
            canonical_cloud: OfflineCanonicalCloud::new(),
            last_occupancy_receipt: None,
            last_cloud_receipt: None,
            last_cloud_snapshot: None,
            committed_frame_count: 0,
            intake_open: true,
        })
    }

    pub async fn submit_frame(&mut self, frame: OfflineReplayFrameV1) -> Result<(), String> {
        if !self.intake_open {
            return Err("offline replay intake is closed".to_owned());
        }
        let validator = self
            .validator
            .take()
            .ok_or_else(|| "offline replay owner is unavailable".to_owned())?;
        let (validated, next_validator) = match validator.validate_frame(frame) {
            Ok(value) => value,
            Err(error) => return self.fail(error.to_string()),
        };
        let sequence = validated.source_sequence();
        let timestamp = validated.source_timestamp_ns();
        let (origin, points) = transform_frame(&validated);
        let input = MappingBackendInput {
            session_id: self.replay_id.clone(),
            session_generation: 1,
            backend_generation: self.backend_generation,
            frame_sequence: sequence,
            source_timestamp_ns: timestamp,
            received_at_ns: timestamp,
            registered_frame_id: validated.map_frame_id().to_owned(),
            sensor_origin_map: origin,
            registered_points_map: points,
        };
        let receiver = match self
            .client
            .submit(self.token(sequence, timestamp, u128::from(sequence)), input)
        {
            Ok(receiver) => receiver,
            Err(error) => {
                return self.fail(format!("mapping backend submit failed: {error:?}"));
            }
        };
        let completion = match receiver.await {
            Ok(completion) => completion,
            Err(_) => return self.fail("mapping backend completion dropped".to_owned()),
        };
        if let Err(error) = self.commit_frame(completion, sequence, timestamp) {
            return self.fail(error);
        }
        self.validator = Some(next_validator);
        Ok(())
    }

    pub async fn finish(mut self) -> Result<OfflineGeometrySnapshotV1, String> {
        self.intake_open = false;
        let result = self.finish_barrier().await.map(project_v1);
        finish_with_exact_join(&mut self, result)
    }

    pub async fn finish_v3(mut self) -> Result<OfflineGeometrySnapshotV3, String> {
        self.intake_open = false;
        let result = self.finish_barrier().await.map(|finished| finished.v3);
        finish_with_exact_join(&mut self, result)
    }

    pub fn cancel(mut self) -> Result<(), String> {
        self.intake_open = false;
        self.join()
    }

    pub fn join(&mut self) -> Result<(), String> {
        self.worker
            .take()
            .map_or(Ok(()), MappingBackendWorker::stop_and_join)
    }

    async fn finish_barrier(&mut self) -> Result<FinishedReplayBarrier, String> {
        let validator = self
            .validator
            .take()
            .ok_or_else(|| "offline replay owner is unavailable".to_owned())?;
        let last = validator
            .last_source_sequence()
            .ok_or_else(|| "offline replay contains no frames".to_owned())?;
        let timestamp = validator.last_source_timestamp_ns().unwrap_or_default();
        let flush_id = u128::from(last) + 1;
        let completion = self
            .client
            .flush(self.token(last, timestamp, flush_id))
            .map_err(|error| format!("mapping backend flush failed: {error:?}"))?
            .await
            .map_err(|_| "mapping backend flush completion dropped".to_owned())?;
        self.commit_flush(completion, last, timestamp, flush_id)?;

        let checkpoint = self
            .client
            .export_checkpoint()
            .map_err(|error| format!("mapping backend checkpoint failed: {error:?}"))?
            .await
            .map_err(|_| "mapping backend checkpoint dropped".to_owned())??;
        let canonical_raw_occupancy = self
            .canonical_occupancy
            .as_ref()
            .ok_or_else(|| "offline replay produced no canonical occupancy".to_owned())?
            .grid
            .clone();
        let occupancy_receipt = self
            .last_occupancy_receipt
            .clone()
            .ok_or_else(|| "offline replay produced no canonical occupancy receipt".to_owned())?;
        let cloud_receipt = self
            .last_cloud_receipt
            .clone()
            .ok_or_else(|| "offline replay produced no canonical cloud receipt".to_owned())?;
        let cloud_snapshot = self
            .last_cloud_snapshot
            .clone()
            .ok_or_else(|| "offline replay produced no canonical cloud snapshot".to_owned())?;
        let created_at_ns = current_unix_time_ns()?;
        let canonical_checkpoint_projection = canonical_checkpoint_projection(
            &self.replay_id,
            self.backend_kind,
            created_at_ns,
            &checkpoint,
            &canonical_raw_occupancy,
            occupancy_receipt,
            cloud_receipt,
            cloud_snapshot,
        )?;
        let barrier = ReplayBarrierIdentityV1 {
            replay_id: self.replay_id.clone(),
            backend_generation: self.backend_generation,
            last_source_sequence: last,
            mapping_commit_sequence: self.committed_frame_count,
        };
        let processing_receipt = OfflineReplayProcessingReceiptV1 {
            barrier: barrier.clone(),
            committed_frame_count: self.committed_frame_count,
            committed_point_count: self.accepted_cloud.len() as u64,
            flush_committed: true,
        };
        let identity = validator.request().identity_v2();
        let map_snapshot_seed = validator
            .request()
            .map_snapshot_seed_v2(self.committed_frame_count);
        let backend_config = validator.request().backend_config().clone();
        let georeference = validator.request().georeference().clone();
        Ok(FinishedReplayBarrier {
            v3: OfflineGeometrySnapshotV3 {
                schema_version: 3,
                barrier,
                identity,
                backend_kind: self.backend_kind,
                backend_config,
                georeference,
                last_source_timestamp_ns: timestamp,
                checkpoint,
                canonical_raw_occupancy,
                committed_transformed_accepted_cloud: std::mem::take(&mut self.accepted_cloud),
                processing_receipt,
                map_snapshot_seed,
                canonical_checkpoint_projection,
            },
            backend_occupancy_observation: self.occupancy_observation.take(),
        })
    }

    fn token(&self, sequence: u64, timestamp: u64, token_id: u128) -> MappingComputeToken {
        MappingComputeToken {
            token_id,
            session_id: self.replay_id.clone(),
            session_generation: 1,
            backend_generation: self.backend_generation,
            source_frame_sequence: sequence,
            source_timestamp_ns: timestamp,
            prepared_at_ns: timestamp,
            base_occupancy_revision: self.committed_frame_count,
        }
    }

    fn validate_identity(
        &self,
        token: &MappingComputeToken,
        sequence: u64,
        timestamp: u64,
        token_id: u128,
    ) -> Result<(), String> {
        if token.token_id != token_id
            || token.session_id != self.replay_id
            || token.session_generation != 1
            || token.backend_generation != self.backend_generation
            || token.source_frame_sequence != sequence
            || token.source_timestamp_ns != timestamp
            || token.base_occupancy_revision != self.committed_frame_count
        {
            Err("mapping backend completion identity mismatch".to_owned())
        } else {
            Ok(())
        }
    }

    fn commit_frame(
        &mut self,
        completion: MappingBackendCompletion,
        sequence: u64,
        timestamp: u64,
    ) -> Result<(), String> {
        let MappingBackendCompletion {
            token,
            accepted_points_map,
            result,
            resolution,
        } = completion;
        if let Err(error) =
            self.validate_identity(&token, sequence, timestamp, u128::from(sequence))
        {
            let _ = resolution.abort();
            return Err(error);
        }
        let candidate = match result {
            Ok(Some(candidate)) => candidate,
            Ok(None) => {
                let _ = resolution.abort();
                return Err("mapping backend frame produced no occupancy candidate".to_owned());
            }
            Err(error) => {
                let _ = resolution.abort();
                return Err(error);
            }
        };
        let staged = match self.stage_candidate(&candidate, sequence, timestamp) {
            Ok(staged) => staged,
            Err(error) => {
                let _ = resolution.abort();
                return Err(error);
            }
        };
        let mut staged_cloud = match self.canonical_cloud.stage_frame(&accepted_points_map) {
            Ok(staged) => staged,
            Err(error) => {
                let _ = resolution.abort();
                return Err(error);
            }
        };
        let cloud_snapshot = match staged_cloud.commit_snapshot() {
            Ok(snapshot) => snapshot,
            Err(error) => {
                let _ = resolution.abort();
                return Err(error);
            }
        };
        let occupancy_revision = self
            .occupancy_revision
            .checked_add(1)
            .ok_or_else(|| "offline canonical occupancy revision exhausted".to_owned())?;
        let committed_frame_count = self
            .committed_frame_count
            .checked_add(1)
            .ok_or_else(|| "offline committed frame count exhausted".to_owned())?;
        let committed_point_count = self
            .accepted_cloud
            .len()
            .checked_add(accepted_points_map.len())
            .ok_or_else(|| "offline committed point count exhausted".to_owned())?;
        u64::try_from(committed_point_count)
            .map_err(|_| "offline committed point count exceeds u64".to_owned())?;
        self.accepted_cloud
            .try_reserve(accepted_points_map.len())
            .map_err(|error| format!("offline accepted cloud allocation failed: {error}"))?;
        let receipt_session_id = self.replay_id.clone();
        commit_fail_closed(resolution)?;
        let commit_id = offline_commit_id(self.commit_clock.reserve()?);
        let occupancy_receipt = OfflineCanonicalOccupancyReceiptV1 {
            commit_id,
            session_id: receipt_session_id,
            session_generation: 1,
            occupancy_revision,
            source_timestamp_ns: timestamp,
            backend_kind: self.backend_kind,
            backend_generation: self.backend_generation,
        };
        let cloud_receipt = OfflineCanonicalCloudReceiptV1 {
            commit_id,
            session_generation: 1,
            occupancy_revision,
            cloud_revision: cloud_snapshot.revision,
            source_timestamp_ns: timestamp,
            point_count: cloud_snapshot.point_count,
        };
        self.canonical_occupancy = Some(staged);
        self.canonical_cloud = staged_cloud;
        self.occupancy_revision = occupancy_revision;
        self.last_occupancy_receipt = Some(occupancy_receipt);
        self.last_cloud_receipt = Some(cloud_receipt);
        self.last_cloud_snapshot = Some(cloud_snapshot);
        self.committed_frame_count = committed_frame_count;
        self.accepted_cloud.extend(accepted_points_map);
        self.occupancy_observation = Some(candidate.live_window_occupancy);
        Ok(())
    }

    fn commit_flush(
        &mut self,
        completion: MappingBackendCompletion,
        sequence: u64,
        timestamp: u64,
        token_id: u128,
    ) -> Result<(), String> {
        let MappingBackendCompletion {
            token,
            accepted_points_map,
            result,
            resolution,
        } = completion;
        if let Err(error) = self.validate_identity(&token, sequence, timestamp, token_id) {
            let _ = resolution.abort();
            return Err(error);
        }
        if !accepted_points_map.is_empty() {
            let _ = resolution.abort();
            return Err("mapping backend flush returned accepted frame points".to_owned());
        }
        let candidate = match result {
            Ok(candidate) => candidate,
            Err(error) => {
                let _ = resolution.abort();
                return Err(error);
            }
        };
        if let Some(candidate) = candidate.as_ref() {
            if let Err(error) = self.validate_candidate_identity(candidate, sequence, timestamp) {
                let _ = resolution.abort();
                return Err(error);
            }
        }
        commit_fail_closed(resolution)?;
        if let Some(candidate) = candidate {
            self.occupancy_observation = Some(candidate.live_window_occupancy);
        }
        Ok(())
    }

    fn validate_candidate_identity(
        &self,
        candidate: &MappingBackendCandidate,
        sequence: u64,
        timestamp: u64,
    ) -> Result<(), String> {
        let expected_kind = match self.backend_kind {
            OfflineBackendKind::Voxel => MappingBackendKind::Voxel,
            OfflineBackendKind::Raytracing => MappingBackendKind::Raytracing,
        };
        if candidate.session_id != self.replay_id
            || candidate.session_generation != 1
            || candidate.backend_generation != self.backend_generation
            || candidate.source_frame_sequence != sequence
            || candidate.source_timestamp_ns != timestamp
            || candidate.received_at_ns != timestamp
            || candidate.backend_kind != expected_kind
            || candidate.live_window_occupancy.frame_id != self.map_frame_id
            || candidate.latest_frame_occupancy.frame_id != self.map_frame_id
        {
            Err("mapping backend candidate identity mismatch".to_owned())
        } else {
            Ok(())
        }
    }

    fn stage_candidate(
        &self,
        candidate: &MappingBackendCandidate,
        sequence: u64,
        timestamp: u64,
    ) -> Result<PersistentOccupancyGrid, String> {
        self.validate_candidate_identity(candidate, sequence, timestamp)?;
        PersistentOccupancyGrid::stage_observation(
            self.canonical_occupancy.as_ref(), &candidate.latest_frame_occupancy)
    }

    fn fail<T>(&mut self, error: String) -> Result<T, String> {
        self.intake_open = false;
        match self.join() {
            Ok(()) => Err(error),
            Err(join_error) => Err(format!("{error}; exact join failed: {join_error}")),
        }
    }
}

fn offline_commit_id(commit_id: MappingCommitId) -> OfflineCanonicalCommitIdV1 {
    OfflineCanonicalCommitIdV1 {
        boot_nonce: commit_id.boot_nonce,
        sequence: commit_id.sequence,
    }
}

fn current_unix_time_ns() -> Result<u64, String> {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| "system clock is before unix epoch".to_owned())?
        .as_nanos();
    let timestamp =
        u64::try_from(nanos).map_err(|_| "unix timestamp exceeds u64 nanoseconds".to_owned())?;
    if timestamp == 0 {
        return Err("unix timestamp must be positive".to_owned());
    }
    Ok(timestamp)
}

#[allow(clippy::too_many_arguments)]
fn canonical_checkpoint_projection(
    replay_id: &str,
    backend_kind: OfflineBackendKind,
    created_at_ns: u64,
    checkpoint: &[u8],
    occupancy: &OccupancyGrid,
    occupancy_receipt: OfflineCanonicalOccupancyReceiptV1,
    cloud_receipt: OfflineCanonicalCloudReceiptV1,
    cloud_snapshot: OfflineCanonicalCloudSnapshot,
) -> Result<OfflineCanonicalCheckpointProjectionV2, String> {
    if created_at_ns == 0
        || checkpoint.is_empty()
        || replay_id.trim().is_empty()
        || occupancy_receipt.session_id != replay_id
        || occupancy_receipt.session_generation != 1
        || occupancy_receipt.backend_generation != 1
        || occupancy_receipt.backend_kind != backend_kind
        || occupancy_receipt.commit_id != cloud_receipt.commit_id
        || cloud_receipt.session_generation != 1
        || cloud_receipt.occupancy_revision != occupancy_receipt.occupancy_revision
        || cloud_receipt.source_timestamp_ns != occupancy_receipt.source_timestamp_ns
        || cloud_receipt.cloud_revision != cloud_snapshot.revision
        || cloud_receipt.point_count != cloud_snapshot.point_count
        || occupancy_receipt.commit_id.boot_nonce == 0
        || occupancy_receipt.commit_id.sequence == 0
        || occupancy_receipt.occupancy_revision == 0
        || cloud_receipt.cloud_revision == 0
        || cloud_snapshot.point_count == 0
        || !cloud_snapshot.voxel_size.is_finite()
        || cloud_snapshot.voxel_size <= 0.0
    {
        return Err("offline canonical checkpoint projection identity mismatch".to_owned());
    }
    let expected_cloud_size = usize::try_from(cloud_snapshot.point_count)
        .ok()
        .and_then(|count| count.checked_mul(12))
        .ok_or_else(|| "offline canonical cloud payload size overflow".to_owned())?;
    if cloud_snapshot.xyz_f32_le.len() != expected_cloud_size {
        return Err("offline canonical cloud payload length mismatch".to_owned());
    }
    let expected_occupancy_size = usize::try_from(occupancy.width)
        .ok()
        .and_then(|width| {
            usize::try_from(occupancy.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| "offline canonical occupancy dimensions overflow".to_owned())?;
    if occupancy.data.is_empty() || occupancy.data.len() != expected_occupancy_size {
        return Err("offline canonical occupancy payload length mismatch".to_owned());
    }
    let checkpoint_size = u64::try_from(checkpoint.len())
        .map_err(|_| "offline checkpoint size exceeds u64".to_owned())?;
    let raw_data_size = u64::try_from(occupancy.data.len())
        .map_err(|_| "offline canonical occupancy size exceeds u64".to_owned())?;
    let xyz_size = u64::try_from(cloud_snapshot.xyz_f32_le.len())
        .map_err(|_| "offline canonical cloud size exceeds u64".to_owned())?;
    let occupancy_asset = OfflineCanonicalOccupancyAssetProjectionV2 {
        raw_data_size,
        width: occupancy.width,
        height: occupancy.height,
        resolution: occupancy.resolution,
        origin_x: occupancy.origin[0],
        origin_y: occupancy.origin[1],
        frame_id: occupancy.frame_id.clone(),
        source_occupancy_revision: occupancy_receipt.occupancy_revision,
        timestamp_ns: occupancy_receipt.source_timestamp_ns,
    };
    let cloud_asset = OfflineCanonicalCloudAssetProjectionV2 {
        xyz_size,
        frame_id: occupancy.frame_id.clone(),
        source_cloud_revision: cloud_receipt.cloud_revision,
        point_count: cloud_snapshot.point_count,
        voxel_size: cloud_snapshot.voxel_size,
        payload_schema_version: OFFLINE_CANONICAL_CLOUD_PAYLOAD_SCHEMA_VERSION,
        xyz_f32_le: cloud_snapshot.xyz_f32_le,
    };
    Ok(OfflineCanonicalCheckpointProjectionV2 {
        schema_version: 2,
        created_at_ns,
        source_session_id: replay_id.to_owned(),
        source_session_generation: 1,
        backend_kind,
        checkpoint_size,
        occupancy_receipt,
        cloud_receipt,
        occupancy_asset,
        cloud_asset,
    })
}

fn finish_with_exact_join<T>(
    owner: &mut GeometryOfflineReplayOwner,
    result: Result<T, String>,
) -> Result<T, String> {
    let join = owner.join();
    match (result, join) {
        (Ok(snapshot), Ok(())) => Ok(snapshot),
        (Err(error), Ok(())) | (Ok(_), Err(error)) => Err(error),
        (Err(error), Err(join_error)) => Err(format!("{error}; exact join failed: {join_error}")),
    }
}

fn project_v1(finished: FinishedReplayBarrier) -> OfflineGeometrySnapshotV1 {
    let OfflineGeometrySnapshotV3 {
        barrier,
        checkpoint,
        committed_transformed_accepted_cloud,
        processing_receipt,
        ..
    } = finished.v3;
    OfflineGeometrySnapshotV1 {
        barrier,
        checkpoint,
        committed_transformed_accepted_cloud,
        backend_occupancy_observation: finished.backend_occupancy_observation,
        processing_receipt,
    }
}

fn commit_fail_closed(
    resolution: super::mapping_backend::MappingBackendCommitHandle,
) -> Result<(), String> {
    resolution.commit().map_err(|error| {
        format!(
            "mapping backend commit delivery failed; terminal abort-unavailable cleanup failure: {error}"
        )
    })
}

fn backend_config(
    config: &OfflineBackendConfigV1,
) -> Result<(MappingBackendSpec, MappingBackendComputeConfig), String> {
    Ok(match config {
        OfflineBackendConfigV1::Voxel(config) => (
            MappingBackendSpec::Voxel,
            MappingBackendComputeConfig {
                resolution: config.resolution_m,
                initial_safe_radius_meters: config.initial_safe_radius_m as f32,
                window_capacity: config.window_capacity,
            },
        ),
        OfflineBackendConfigV1::Raytracing(config) => (
            MappingBackendSpec::raytracing(
                config.voxel_size_m as f32,
                config.max_range_m as f32,
                config.ray_subsample,
                config.shadow_depth_m as f32,
                config.grace_depth_m as f32,
                config.min_health,
                config.max_health,
                config.graze_cos as f32,
                config.recency_window,
            ),
            MappingBackendComputeConfig {
                resolution: config.resolution_m,
                initial_safe_radius_meters: 0.0,
                window_capacity: 1,
            },
        ),
    })
}

fn transform_frame(frame: &ValidatedOfflineReplayFrameV1) -> ([f64; 3], Vec<Point3<f64>>) {
    let translation = frame.translation_xyz_m();
    let q = frame.rotation_xyzw();
    let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q[3], q[0], q[1], q[2],
    ));
    let offset = nalgebra::Vector3::new(translation[0], translation[1], translation[2]);
    (
        translation,
        frame
            .points_xyz()
            .iter()
            .map(|point| rotation.transform_point(&Point3::new(point.x, point.y, point.z)) + offset)
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::offline_replay::{OfflinePointXyzV1, OfflinePoseV1, OfflineReplayConfigV1, OfflineVoxelConfigV1, OfflineRaytracingConfigV1};

    #[tokio::test]
    async fn offline_owner_accumulates_horizontal_observations_and_keeps_full_cloud() {
        for backend in [
            OfflineBackendConfigV1::Voxel(OfflineVoxelConfigV1 {
                resolution_m:0.05, voxel_size_m:0.05, initial_safe_radius_m:0.0, window_capacity:100,
            }),
            OfflineBackendConfigV1::Raytracing(OfflineRaytracingConfigV1 {
                resolution_m:0.05, voxel_size_m:0.05, max_range_m:30.0, ray_subsample:1,
                shadow_depth_m:0.2, grace_depth_m:0.2, min_health:1, max_health:10,
                graze_cos:0.0, recency_window:10,
            }),
        ] {
            let kind = match &backend { OfflineBackendConfigV1::Voxel(_) => OfflineBackendKind::Voxel,
                OfflineBackendConfigV1::Raytracing(_) => OfflineBackendKind::Raytracing };
            let mut owner = GeometryOfflineReplayOwner::start(OfflineReplayRequestV1 {
                map_name:"horizontal-test".into(), source_id:"paired-scans".into(),
                config:OfflineReplayConfigV1 {
                    map_frame_id:"map".into(), backend_kind:kind, backend,
                    georeference:OfflineGeoreferenceV1 {
                        schema_version:1, map_name:"horizontal-test".into(), map_frame_id:"map".into(),
                        source_crs:"EPSG:4326".into(), projected_crs:"EPSG:3857".into(),
                        axis_convention:"projected_x_east_y_north_z_up".into(),
                        altitude_policy:"map_plane_constant".into(), anchor_lat_deg:0.0, anchor_lng_deg:0.0,
                        anchor_easting_m:0.0, anchor_northing_m:0.0, map_origin_x_m:0.0, map_origin_y_m:0.0,
                        map_from_projected_yaw_rad:0.0, map_plane_z_m:0.0,
                    },
                },
            }).unwrap();
            for sequence in 1..=2 {
                owner.submit_frame(OfflineReplayFrameV1 {
                    source_sequence:sequence, source_timestamp_ns:sequence as i64 * 100_000_000,
                    sensor_frame_id:"lidar".into(), map_frame_id:"map".into(),
                    pose_t_map_sensor:OfflinePoseV1 { translation_xyz_m:[10.025,0.025,2.0], rotation_xyzw:[0.0,0.0,0.0,1.0] },
                    points_xyz:vec![OfflinePointXyzV1{x:2.0,y:0.0,z:0.0}, OfflinePointXyzV1{x:1.0,y:0.0,z:-0.4}, OfflinePointXyzV1{x:1.0,y:0.0,z:2.0}],
                }).await.unwrap();
                assert_eq!(owner.canonical_occupancy.as_ref().unwrap().grid.data.contains(&100), sequence == 2);
            }
            let snapshot = owner.finish_v3().await.unwrap();
            assert_eq!(snapshot.processing_receipt.committed_frame_count,2);
            assert!(snapshot.canonical_raw_occupancy.data.contains(&0));
            assert!(snapshot.canonical_raw_occupancy.data.contains(&100));
            assert!(snapshot.canonical_raw_occupancy.origin[0] >= 9.0);
            assert!(snapshot.committed_transformed_accepted_cloud.iter().any(|point|point.z < 1.7));
            assert!(snapshot.committed_transformed_accepted_cloud.iter().any(|point|point.z > 3.9));
            let checkpoint = &snapshot.canonical_checkpoint_projection;
            assert_eq!(checkpoint.occupancy_receipt.commit_id,checkpoint.cloud_receipt.commit_id);
        }
    }
}
