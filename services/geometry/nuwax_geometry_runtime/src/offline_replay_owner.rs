//! Geometry 库私有离线重放 owner。

use nalgebra::Point3;
use nuwax_mapping::OccupancyGrid;
use super::mapping_backend::{MappingBackendCompletion, MappingBackendComputeConfig, MappingBackendInput, MappingBackendSpec, MappingBackendWorker, MappingComputeToken};
use super::offline_replay::{OfflineBackendConfigV1, OfflineGeometryFacade, OfflineReplayFrameV1, OfflineReplayRequestV1, OfflineReplayValidator, ValidatedOfflineReplayFrameV1};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReplayBarrierIdentityV1 { pub replay_id: String, pub backend_generation: u64, pub last_source_sequence: u64, pub mapping_commit_sequence: u64 }
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OfflineReplayProcessingReceiptV1 { pub barrier: ReplayBarrierIdentityV1, pub committed_frame_count: u64, pub committed_point_count: u64, pub flush_committed: bool }
#[derive(Clone, Debug)]
pub struct OfflineGeometrySnapshotV1 { pub barrier: ReplayBarrierIdentityV1, pub checkpoint: Vec<u8>, pub committed_transformed_accepted_cloud: Vec<Point3<f64>>, pub backend_occupancy_observation: Option<OccupancyGrid>, pub processing_receipt: OfflineReplayProcessingReceiptV1 }

pub struct GeometryOfflineReplayOwner { validator: Option<OfflineReplayValidator>, worker: Option<MappingBackendWorker>, client: super::mapping_backend::MappingBackendClient, replay_id: String, backend_generation: u64, accepted_cloud: Vec<Point3<f64>>, occupancy_observation: Option<OccupancyGrid>, committed_frame_count: u64, intake_open: bool }

impl GeometryOfflineReplayOwner {
    pub fn start(request: OfflineReplayRequestV1) -> Result<Self, String> {
        let replay_id = format!("{}:{}", request.map_name, request.source_id);
        let validator = OfflineGeometryFacade::start(request).map_err(|error| error.to_string())?;
        let (spec, config) = backend_config(validator.request().backend_config())?;
        let backend_generation = 1; let worker = MappingBackendWorker::start(spec, backend_generation, config)?; let client = worker.client();
        Ok(Self { validator: Some(validator), worker: Some(worker), client, replay_id, backend_generation, accepted_cloud: Vec::new(), occupancy_observation: None, committed_frame_count: 0, intake_open: true })
    }
    pub async fn submit_frame(&mut self, frame: OfflineReplayFrameV1) -> Result<(), String> {
        if !self.intake_open { return Err("offline replay intake is closed".to_owned()); }
        let validator = self.validator.take().ok_or_else(|| "offline replay owner is unavailable".to_owned())?;
        let (validated, next_validator) = match validator.validate_frame(frame) { Ok(value) => value, Err(error) => return self.fail(error.to_string()) };
        let sequence = validated.source_sequence(); let timestamp = validated.source_timestamp_ns(); let (origin, points) = transform_frame(&validated);
        let input = MappingBackendInput { session_id: self.replay_id.clone(), session_generation: 1, backend_generation: self.backend_generation, frame_sequence: sequence, source_timestamp_ns: timestamp, received_at_ns: timestamp, registered_frame_id: validated.map_frame_id().to_owned(), sensor_origin_map: origin, registered_points_map: points };
        let receiver = match self.client.submit(self.token(sequence, timestamp, u128::from(sequence)), input) { Ok(receiver) => receiver, Err(error) => return self.fail(format!("mapping backend submit failed: {error:?}")) };
        let completion = match receiver.await { Ok(completion) => completion, Err(_) => return self.fail("mapping backend completion dropped".to_owned()) };
        if let Err(error) = self.commit_frame(completion, sequence, timestamp) { return self.fail(error); }
        self.validator = Some(next_validator); Ok(())
    }
    pub async fn finish(mut self) -> Result<OfflineGeometrySnapshotV1, String> {
        self.intake_open = false; let result = self.finish_barrier().await; let join = self.join();
        match (result, join) { (Ok(snapshot), Ok(())) => Ok(snapshot), (Err(error), Ok(())) | (Ok(_), Err(error)) => Err(error), (Err(error), Err(join_error)) => Err(format!("{error}; exact join failed: {join_error}")) }
    }
    pub fn cancel(mut self) -> Result<(), String> { self.intake_open = false; self.join() }
    pub fn join(&mut self) -> Result<(), String> { self.worker.take().map_or(Ok(()), MappingBackendWorker::stop_and_join) }
    async fn finish_barrier(&mut self) -> Result<OfflineGeometrySnapshotV1, String> {
        let validator = self.validator.take().ok_or_else(|| "offline replay owner is unavailable".to_owned())?;
        let last = validator.last_source_sequence().ok_or_else(|| "offline replay contains no frames".to_owned())?; let timestamp = validator.last_source_timestamp_ns().unwrap_or_default(); let flush_id = u128::from(last) + 1;
        let completion = self.client.flush(self.token(last, timestamp, flush_id)).map_err(|error| format!("mapping backend flush failed: {error:?}"))?.await.map_err(|_| "mapping backend flush completion dropped".to_owned())?;
        self.commit_flush(completion, last, timestamp, flush_id)?;
        let checkpoint = self.client.export_checkpoint().map_err(|error| format!("mapping backend checkpoint failed: {error:?}"))?.await.map_err(|_| "mapping backend checkpoint dropped".to_owned())??;
        let barrier = ReplayBarrierIdentityV1 { replay_id: self.replay_id.clone(), backend_generation: self.backend_generation, last_source_sequence: last, mapping_commit_sequence: self.committed_frame_count };
        let processing_receipt = OfflineReplayProcessingReceiptV1 { barrier: barrier.clone(), committed_frame_count: self.committed_frame_count, committed_point_count: self.accepted_cloud.len() as u64, flush_committed: true };
        Ok(OfflineGeometrySnapshotV1 { barrier, checkpoint, committed_transformed_accepted_cloud: std::mem::take(&mut self.accepted_cloud), backend_occupancy_observation: self.occupancy_observation.take(), processing_receipt })
    }
    fn token(&self, sequence: u64, timestamp: u64, token_id: u128) -> MappingComputeToken { MappingComputeToken { token_id, session_id: self.replay_id.clone(), session_generation: 1, backend_generation: self.backend_generation, source_frame_sequence: sequence, source_timestamp_ns: timestamp, prepared_at_ns: timestamp, base_occupancy_revision: self.committed_frame_count } }
    fn validate_identity(&self, token: &MappingComputeToken, sequence: u64, timestamp: u64, token_id: u128) -> Result<(), String> {
        if token.token_id != token_id || token.session_id != self.replay_id || token.session_generation != 1 || token.backend_generation != self.backend_generation || token.source_frame_sequence != sequence || token.source_timestamp_ns != timestamp || token.base_occupancy_revision != self.committed_frame_count { Err("mapping backend completion identity mismatch".to_owned()) } else { Ok(()) }
    }
    fn commit_frame(&mut self, completion: MappingBackendCompletion, sequence: u64, timestamp: u64) -> Result<(), String> {
        let MappingBackendCompletion { token, accepted_points_map, result, resolution } = completion;
        if let Err(error) = self.validate_identity(&token, sequence, timestamp, u128::from(sequence)) { let _ = resolution.abort(); return Err(error); }
        let candidate = match result { Ok(candidate) => candidate, Err(error) => { let _ = resolution.abort(); return Err(error); } };
        commit_fail_closed(resolution)?; self.committed_frame_count += 1; self.accepted_cloud.extend(accepted_points_map);
        if let Some(candidate) = candidate { self.occupancy_observation = Some(candidate.live_window_occupancy); } Ok(())
    }
    fn commit_flush(&mut self, completion: MappingBackendCompletion, sequence: u64, timestamp: u64, token_id: u128) -> Result<(), String> {
        let MappingBackendCompletion { token, accepted_points_map, result, resolution } = completion;
        if let Err(error) = self.validate_identity(&token, sequence, timestamp, token_id) { let _ = resolution.abort(); return Err(error); }
        if !accepted_points_map.is_empty() { let _ = resolution.abort(); return Err("mapping backend flush returned accepted frame points".to_owned()); }
        let candidate = match result { Ok(candidate) => candidate, Err(error) => { let _ = resolution.abort(); return Err(error); } };
        commit_fail_closed(resolution)?; if let Some(candidate) = candidate { self.occupancy_observation = Some(candidate.live_window_occupancy); } Ok(())
    }
    fn fail<T>(&mut self, error: String) -> Result<T, String> { self.intake_open = false; match self.join() { Ok(()) => Err(error), Err(join_error) => Err(format!("{error}; exact join failed: {join_error}")) } }
}
fn commit_fail_closed(resolution: super::mapping_backend::MappingBackendCommitHandle) -> Result<(), String> { resolution.commit().map_err(|error| format!("mapping backend commit delivery failed; terminal abort-unavailable cleanup failure: {error}")) }
fn backend_config(config: &OfflineBackendConfigV1) -> Result<(MappingBackendSpec, MappingBackendComputeConfig), String> {
    Ok(match config {
        OfflineBackendConfigV1::Voxel(config) => (MappingBackendSpec::Voxel, MappingBackendComputeConfig { resolution: config.resolution_m, occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost, initial_safe_radius_meters: config.initial_safe_radius_m as f32, window_capacity: config.window_capacity }),
        OfflineBackendConfigV1::Raytracing(config) => (MappingBackendSpec::raytracing(config.voxel_size_m as f32, config.max_range_m as f32, config.ray_subsample, config.shadow_depth_m as f32, config.grace_depth_m as f32, config.min_health, config.max_health, config.graze_cos as f32, config.recency_window), MappingBackendComputeConfig { resolution: config.resolution_m, occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost, initial_safe_radius_meters: 0.0, window_capacity: 1 }),
    })
}
fn transform_frame(frame: &ValidatedOfflineReplayFrameV1) -> ([f64; 3], Vec<Point3<f64>>) { let translation = frame.translation_xyz_m(); let q = frame.rotation_xyzw(); let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[3], q[0], q[1], q[2])); let offset = nalgebra::Vector3::new(translation[0], translation[1], translation[2]); (translation, frame.points_xyz().iter().map(|point| rotation.transform_point(&Point3::new(point.x, point.y, point.z)) + offset).collect()) }
