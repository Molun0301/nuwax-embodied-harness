//! Stage3 离线候选资产的纯内存导出。

use std::collections::HashSet;

use thiserror::Error;

use super::offline_replay::{
    OfflineBackendConfigV1, OfflineBackendKind, OfflineGeoreferenceV1, OfflineMapSnapshotSeedV2,
    OfflineReplayIdentityV2,
};
use super::offline_replay_owner::{OfflineGeometrySnapshotV2, OfflineReplayProcessingReceiptV1};
use nuwax_mapping_raytracing::voxel_ray_tracer::{
    decode_voxel_map_checkpoint_for_config, Config as RayConfig, VOXEL_MAP_CHECKPOINT_ENTRY_BYTES,
    VOXEL_MAP_CHECKPOINT_HEADER_BYTES,
};

const NAV2_YAML_PATH: &str = "geometry/nuwax_geometry_runtime/navigation/map.yaml";
const NAV2_PGM_PATH: &str = "geometry/nuwax_geometry_runtime/navigation/map.pgm";
const PCD_PATH: &str = "geometry/nuwax_geometry_runtime/pcd/map.pcd";
const CHECKPOINT_MAGIC: &[u8; 8] = b"NWMBCKP\0";
const CHECKPOINT_SCHEMA_VERSION: u32 = 1;
// 逐字镜像 `mapping_backend.rs::MAX_RETAINED_VOXEL_POINTS` 的容量合同。
const MAX_RETAINED_VOXEL_POINTS: usize =
    (super::mapping_backend::MAX_MAPPING_BACKEND_CHECKPOINT_BYTES - 48) / 24;

#[derive(Clone, Debug)]
pub struct OfflineCandidateAssetsV1 {
    pub schema_version: u32,
    pub snapshot_schema_version: u32,
    pub identity: OfflineReplayIdentityV2,
    pub backend_kind: OfflineBackendKind,
    pub last_source_timestamp_ns: u64,
    pub georeference: OfflineGeoreferenceV1,
    pub processing_receipt: OfflineReplayProcessingReceiptV1,
    pub map_snapshot_seed: OfflineMapSnapshotSeedV2,
    pub deferred: Vec<DeferredAssetMemberV1>,
    pub members: Vec<CandidateAssetMemberV1>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CandidateAssetMemberV1 {
    pub relative_path_intent: String,
    pub media_type: &'static str,
    pub encoding: CandidateAssetEncodingV1,
    pub bytes: Vec<u8>,
    pub receipt: AssetReceiptV1,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssetReceiptV1 {
    pub relative_path_intent: String,
    pub media_type: &'static str,
    pub encoding: CandidateAssetEncodingV1,
    pub size_bytes: u64,
    pub sha256_hex: String,
    pub source_binding: &'static str,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CandidateAssetEncodingV1 {
    Nav2OccupancyYamlV1,
    Nav2OccupancyPgmP5V1,
    PcdXyzBinaryV1,
    CheckpointObjectV1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeferredAssetMemberV1 {
    CheckpointSelectorManifestStage4,
    TopLevelManifestStage4,
    MappingDatabaseStage5,
    FinalMapSnapshotStage5,
    MapAssetBundleStage5,
}

#[derive(Debug, Error, Eq, PartialEq)]
pub enum OfflineAssetExportError {
    #[error("offline snapshot schema_version must be 2")]
    InvalidSnapshotSchema,
    #[error("offline snapshot barrier and processing receipt are inconsistent")]
    InvalidProcessingReceipt,
    #[error("offline snapshot identity bindings are inconsistent")]
    IdentityMismatch,
    #[error("offline occupancy dimensions or data length are invalid")]
    InvalidOccupancyDimensions,
    #[error("offline occupancy contains an invalid value at index {index}")]
    InvalidOccupancyValue { index: usize },
    #[error("offline occupancy contains an invalid numeric field")]
    InvalidOccupancyNumber,
    #[error("offline occupancy quaternion does not produce a finite yaw")]
    InvalidOccupancyYaw,
    #[error("offline candidate cloud must not be empty")]
    EmptyPointCloud,
    #[error("offline candidate cloud point {point_index} is not finite")]
    InvalidPoint { point_index: usize },
    #[error("offline candidate cloud point {point_index} cannot be represented as finite f32")]
    InvalidPointF32 { point_index: usize },
    #[error("offline checkpoint header is invalid")]
    InvalidCheckpointHeader,
    #[error("offline checkpoint backend kind does not match the snapshot")]
    CheckpointBackendMismatch,
    #[error("offline candidate contains a duplicate relative path")]
    DuplicatePath,
    #[error("offline candidate asset size cannot be represented as u64")]
    AssetSizeOverflow,
}

pub fn export_offline_candidate_assets_v1(
    snapshot: &OfflineGeometrySnapshotV2,
) -> Result<OfflineCandidateAssetsV1, OfflineAssetExportError> {
    validate_snapshot(snapshot)?;

    let pgm = encode_nav2_pgm(snapshot)?;
    let yaml = encode_nav2_yaml(snapshot)?.into_bytes();
    let pcd = encode_pcd(snapshot)?;
    let checkpoint_digest = hex_digest(&nuwax_codec::sha256_bytes(&snapshot.checkpoint));
    let backend_name = match snapshot.backend_kind {
        OfflineBackendKind::Voxel => "voxel",
        OfflineBackendKind::Raytracing => "raytracing",
    };
    let checkpoint_path =
        format!("mapping_backend/{backend_name}/objects/sha256-{checkpoint_digest}.bin");

    let members = vec![
        member(
            NAV2_YAML_PATH,
            "application/x-yaml",
            CandidateAssetEncodingV1::Nav2OccupancyYamlV1,
            yaml,
            "canonical_raw_occupancy",
        )?,
        member(
            NAV2_PGM_PATH,
            "image/x-portable-graymap",
            CandidateAssetEncodingV1::Nav2OccupancyPgmP5V1,
            pgm,
            "canonical_raw_occupancy",
        )?,
        member(
            PCD_PATH,
            "application/vnd.pointclouds.pcd",
            CandidateAssetEncodingV1::PcdXyzBinaryV1,
            pcd,
            "committed_transformed_accepted_cloud",
        )?,
        member(
            &checkpoint_path,
            "application/octet-stream",
            CandidateAssetEncodingV1::CheckpointObjectV1,
            snapshot.checkpoint.clone(),
            "mapping_backend_checkpoint",
        )?,
    ];
    let members = sort_and_validate_members(members)?;

    Ok(OfflineCandidateAssetsV1 {
        schema_version: 1,
        snapshot_schema_version: snapshot.schema_version,
        identity: snapshot.identity.clone(),
        backend_kind: snapshot.backend_kind,
        last_source_timestamp_ns: snapshot.last_source_timestamp_ns,
        georeference: snapshot.georeference.clone(),
        processing_receipt: snapshot.processing_receipt.clone(),
        map_snapshot_seed: snapshot.map_snapshot_seed.clone(),
        deferred: vec![
            DeferredAssetMemberV1::CheckpointSelectorManifestStage4,
            DeferredAssetMemberV1::TopLevelManifestStage4,
            DeferredAssetMemberV1::MappingDatabaseStage5,
            DeferredAssetMemberV1::FinalMapSnapshotStage5,
            DeferredAssetMemberV1::MapAssetBundleStage5,
        ],
        members,
    })
}

fn validate_snapshot(snapshot: &OfflineGeometrySnapshotV2) -> Result<(), OfflineAssetExportError> {
    if snapshot.schema_version != 2 {
        return Err(OfflineAssetExportError::InvalidSnapshotSchema);
    }
    let receipt = &snapshot.processing_receipt;
    if receipt.barrier != snapshot.barrier
        || !receipt.flush_committed
        || receipt.committed_frame_count != snapshot.barrier.mapping_commit_sequence
        || receipt.committed_point_count
            != u64::try_from(snapshot.committed_transformed_accepted_cloud.len())
                .map_err(|_| OfflineAssetExportError::InvalidProcessingReceipt)?
    {
        return Err(OfflineAssetExportError::InvalidProcessingReceipt);
    }
    let identity = &snapshot.identity;
    if snapshot.barrier.replay_id != format!("{}:{}", identity.map_name, identity.source_id)
        || snapshot.georeference.map_name != identity.map_name
        || snapshot.georeference.map_frame_id != identity.map_frame_id
        || snapshot.map_snapshot_seed.map_name != identity.map_name
        || snapshot.map_snapshot_seed.map_frame_id != identity.map_frame_id
        || snapshot.map_snapshot_seed.update_sequence != receipt.committed_frame_count
        || snapshot.canonical_raw_occupancy.frame_id != identity.map_frame_id
    {
        return Err(OfflineAssetExportError::IdentityMismatch);
    }

    let grid = &snapshot.canonical_raw_occupancy;
    let cell_count = usize::try_from(grid.width)
        .ok()
        .and_then(|width| {
            usize::try_from(grid.height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .filter(|_| grid.width != 0 && grid.height != 0)
        .ok_or(OfflineAssetExportError::InvalidOccupancyDimensions)?;
    if grid.data.len() != cell_count {
        return Err(OfflineAssetExportError::InvalidOccupancyDimensions);
    }
    if let Some(index) = grid
        .data
        .iter()
        .position(|value| !(-1..=100).contains(value))
    {
        return Err(OfflineAssetExportError::InvalidOccupancyValue { index });
    }
    if !grid.resolution.is_finite()
        || grid.resolution <= 0.0
        || grid.origin.iter().any(|value| !value.is_finite())
        || grid
            .origin_orientation
            .iter()
            .any(|value| !value.is_finite())
    {
        return Err(OfflineAssetExportError::InvalidOccupancyNumber);
    }
    occupancy_yaw(grid.origin_orientation)?;

    if snapshot.committed_transformed_accepted_cloud.is_empty() {
        return Err(OfflineAssetExportError::EmptyPointCloud);
    }
    for (point_index, point) in snapshot
        .committed_transformed_accepted_cloud
        .iter()
        .enumerate()
    {
        if !point.coords.iter().all(|value| value.is_finite()) {
            return Err(OfflineAssetExportError::InvalidPoint { point_index });
        }
        if ![point.x as f32, point.y as f32, point.z as f32]
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(OfflineAssetExportError::InvalidPointF32 { point_index });
        }
    }
    validate_checkpoint(snapshot)
}

fn validate_checkpoint(
    snapshot: &OfflineGeometrySnapshotV2,
) -> Result<(), OfflineAssetExportError> {
    let payload = snapshot.checkpoint.as_slice();
    if payload.is_empty()
        || payload.len() > super::mapping_backend::MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
    {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    let header = payload
        .get(..16)
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    if &header[..8] != CHECKPOINT_MAGIC
        || read_u32(payload, 8)? != CHECKPOINT_SCHEMA_VERSION
        || header[13] != 0
        || header[14] != 0
        || header[15] != 0
    {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    let expected_kind = match snapshot.backend_kind {
        OfflineBackendKind::Voxel => 0,
        OfflineBackendKind::Raytracing => 1,
    };
    if header[12] != expected_kind {
        return Err(OfflineAssetExportError::CheckpointBackendMismatch);
    }
    match (&snapshot.backend_config, snapshot.backend_kind) {
        (OfflineBackendConfigV1::Voxel(config), OfflineBackendKind::Voxel) => {
            validate_voxel_checkpoint(payload, config.window_capacity)
        }
        (OfflineBackendConfigV1::Raytracing(config), OfflineBackendKind::Raytracing) => {
            validate_ray_checkpoint(payload, config)
        }
        _ => Err(OfflineAssetExportError::CheckpointBackendMismatch),
    }
}

fn validate_voxel_checkpoint(
    payload: &[u8],
    selected_capacity: usize,
) -> Result<(), OfflineAssetExportError> {
    const HEADER_BYTES: usize = 48;
    let capacity = usize::try_from(read_u64(payload, 16)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let point_count = usize::try_from(read_u64(payload, 24)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let candidate_count = usize::try_from(read_u64(payload, 32)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let retained_capacity = capacity.min(MAX_RETAINED_VOXEL_POINTS);
    if capacity != selected_capacity
        || point_count > retained_capacity
        || candidate_count > retained_capacity
    {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    let expected = point_count
        .checked_mul(24)
        .and_then(|bytes| HEADER_BYTES.checked_add(bytes))
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    if payload.len() != expected {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    for offset in (HEADER_BYTES..expected).step_by(8) {
        if !f64::from_bits(read_u64(payload, offset)?).is_finite() {
            return Err(OfflineAssetExportError::InvalidCheckpointHeader);
        }
    }
    Ok(())
}

fn validate_ray_checkpoint(
    payload: &[u8],
    selected: &super::offline_replay::OfflineRaytracingConfigV1,
) -> Result<(), OfflineAssetExportError> {
    const HEADER_BYTES: usize = 96;
    let expected_config = [
        (selected.voxel_size_m as f32).to_bits(),
        (selected.max_range_m as f32).to_bits(),
        selected.ray_subsample,
        (selected.shadow_depth_m as f32).to_bits(),
        (selected.grace_depth_m as f32).to_bits(),
        u32::from_le_bytes(selected.min_health.to_le_bytes()),
        u32::from_le_bytes(selected.max_health.to_le_bytes()),
        (selected.graze_cos as f32).to_bits(),
        selected.recency_window,
        0,
        0,
        95.0_f32.to_bits(),
    ];
    for (index, expected) in expected_config.iter().enumerate() {
        if read_u32(payload, 16 + index * 4)? != *expected {
            return Err(OfflineAssetExportError::InvalidCheckpointHeader);
        }
    }
    let candidate_count = usize::try_from(read_u64(payload, 64)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let voxel_len = usize::try_from(read_u64(payload, 80)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let occupancy_len = usize::try_from(read_u64(payload, 88)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let body_len = voxel_len
        .checked_add(occupancy_len)
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    if payload.len()
        != HEADER_BYTES
            .checked_add(body_len)
            .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?
    {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    let max_voxels = (super::mapping_backend::MAX_MAPPING_BACKEND_CHECKPOINT_BYTES
        - HEADER_BYTES
        - VOXEL_MAP_CHECKPOINT_HEADER_BYTES)
        / VOXEL_MAP_CHECKPOINT_ENTRY_BYTES;
    if candidate_count > max_voxels {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    let config = RayConfig {
        voxel_size: selected.voxel_size_m as f32,
        max_range: selected.max_range_m as f32,
        ray_subsample: selected.ray_subsample,
        shadow_depth: selected.shadow_depth_m as f32,
        grace_depth: selected.grace_depth_m as f32,
        min_health: selected.min_health,
        max_health: selected.max_health,
        graze_cos: selected.graze_cos as f32,
        recency_window: selected.recency_window,
        emit_every: 0,
        global_emit_every: 0,
        region_percentile: 95.0,
    };
    decode_voxel_map_checkpoint_for_config(
        &payload[HEADER_BYTES..HEADER_BYTES + voxel_len],
        max_voxels,
        &config,
    )
    .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    if occupancy_len != 0 {
        validate_checkpoint_occupancy(&payload[HEADER_BYTES + voxel_len..])?;
    }
    Ok(())
}

fn validate_checkpoint_occupancy(payload: &[u8]) -> Result<(), OfflineAssetExportError> {
    const FIXED_BYTES: usize = 92;
    let width = read_u32(payload, 0)?;
    let height = read_u32(payload, 4)?;
    let resolution = f64::from_bits(read_u64(payload, 8)?);
    for offset in (16..72).step_by(8) {
        if !f64::from_bits(read_u64(payload, offset)?).is_finite() {
            return Err(OfflineAssetExportError::InvalidCheckpointHeader);
        }
    }
    let ts = f64::from_bits(read_u64(payload, 72)?);
    let frame_len = usize::try_from(read_u32(payload, 80)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let data_len = usize::try_from(read_u64(payload, 84)?)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let expected_cells = usize::try_from(width)
        .ok()
        .and_then(|w| usize::try_from(height).ok().and_then(|h| w.checked_mul(h)))
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    let frame_end = FIXED_BYTES
        .checked_add(frame_len)
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    let expected_len = frame_end
        .checked_add(data_len)
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    if payload.len() != expected_len || frame_end > payload.len() {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    let frame_bytes = payload
        .get(FIXED_BYTES..frame_end)
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    let frame_id = std::str::from_utf8(frame_bytes)
        .map_err(|_| OfflineAssetExportError::InvalidCheckpointHeader)?;
    let data = payload
        .get(frame_end..expected_len)
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    if width == 0
        || height == 0
        || data_len != expected_cells
        || !resolution.is_finite()
        || resolution <= 0.0
        || !ts.is_finite()
        || frame_id.trim().is_empty()
        || data
            .iter()
            .any(|value| !(-1..=100).contains(&(*value as i8)))
    {
        return Err(OfflineAssetExportError::InvalidCheckpointHeader);
    }
    Ok(())
}

fn read_u32(payload: &[u8], offset: usize) -> Result<u32, OfflineAssetExportError> {
    let bytes = payload
        .get(offset..offset.saturating_add(4))
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    Ok(u32::from_le_bytes(bytes))
}

fn read_u64(payload: &[u8], offset: usize) -> Result<u64, OfflineAssetExportError> {
    let bytes = payload
        .get(offset..offset.saturating_add(8))
        .and_then(|bytes| bytes.try_into().ok())
        .ok_or(OfflineAssetExportError::InvalidCheckpointHeader)?;
    Ok(u64::from_le_bytes(bytes))
}

fn encode_nav2_pgm(
    snapshot: &OfflineGeometrySnapshotV2,
) -> Result<Vec<u8>, OfflineAssetExportError> {
    let grid = &snapshot.canonical_raw_occupancy;
    let mut bytes = format!("P5\n{} {}\n255\n", grid.width, grid.height).into_bytes();
    let width = usize::try_from(grid.width)
        .map_err(|_| OfflineAssetExportError::InvalidOccupancyDimensions)?;
    let height = usize::try_from(grid.height)
        .map_err(|_| OfflineAssetExportError::InvalidOccupancyDimensions)?;
    for row in (0..height).rev() {
        let start = row
            .checked_mul(width)
            .ok_or(OfflineAssetExportError::InvalidOccupancyDimensions)?;
        for value in &grid.data[start..start + width] {
            bytes.push(if *value < 0 {
                205
            } else if *value >= 65 {
                0
            } else {
                254
            });
        }
    }
    Ok(bytes)
}

fn encode_nav2_yaml(
    snapshot: &OfflineGeometrySnapshotV2,
) -> Result<String, OfflineAssetExportError> {
    let grid = &snapshot.canonical_raw_occupancy;
    let yaw = occupancy_yaw(grid.origin_orientation)?;
    Ok(format!(
        "image: map.pgm\nmode: trinary\nresolution: {}\norigin: [{}, {}, {}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        python_g12(grid.resolution),
        python_g12(grid.origin[0]),
        python_g12(grid.origin[1]),
        python_g12(yaw),
    ))
}

fn occupancy_yaw(rotation: [f64; 4]) -> Result<f64, OfflineAssetExportError> {
    let [x, y, z, w] = rotation;
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    if yaw.is_finite() {
        Ok(yaw)
    } else {
        Err(OfflineAssetExportError::InvalidOccupancyYaw)
    }
}

fn encode_pcd(snapshot: &OfflineGeometrySnapshotV2) -> Result<Vec<u8>, OfflineAssetExportError> {
    let point_count = snapshot.committed_transformed_accepted_cloud.len();
    let mut bytes = format!(
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {point_count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {point_count}\nDATA binary\n"
    )
    .into_bytes();
    for point in &snapshot.committed_transformed_accepted_cloud {
        bytes.extend_from_slice(&(point.x as f32).to_le_bytes());
        bytes.extend_from_slice(&(point.y as f32).to_le_bytes());
        bytes.extend_from_slice(&(point.z as f32).to_le_bytes());
    }
    Ok(bytes)
}

fn member(
    path: &str,
    media_type: &'static str,
    encoding: CandidateAssetEncodingV1,
    bytes: Vec<u8>,
    source_binding: &'static str,
) -> Result<CandidateAssetMemberV1, OfflineAssetExportError> {
    let size_bytes =
        u64::try_from(bytes.len()).map_err(|_| OfflineAssetExportError::AssetSizeOverflow)?;
    let relative_path_intent = path.to_owned();
    let receipt = AssetReceiptV1 {
        relative_path_intent: relative_path_intent.clone(),
        media_type,
        encoding,
        size_bytes,
        sha256_hex: hex_digest(&nuwax_codec::sha256_bytes(&bytes)),
        source_binding,
    };
    Ok(CandidateAssetMemberV1 {
        relative_path_intent,
        media_type,
        encoding,
        bytes,
        receipt,
    })
}

fn sort_and_validate_members(
    mut members: Vec<CandidateAssetMemberV1>,
) -> Result<Vec<CandidateAssetMemberV1>, OfflineAssetExportError> {
    members.sort_by(|left, right| {
        left.relative_path_intent
            .as_bytes()
            .cmp(right.relative_path_intent.as_bytes())
    });
    let mut paths = HashSet::with_capacity(members.len());
    if members
        .iter()
        .any(|member| !paths.insert(member.relative_path_intent.as_bytes()))
    {
        return Err(OfflineAssetExportError::DuplicatePath);
    }
    Ok(members)
}

fn hex_digest(digest: &[u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(64);
    for byte in digest {
        encoded.push(char::from(HEX[usize::from(byte >> 4)]));
        encoded.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    encoded
}

fn python_g12(value: f64) -> String {
    if value == 0.0 {
        return if value.is_sign_negative() { "-0" } else { "0" }.to_owned();
    }
    let scientific = format!("{value:.11e}");
    let (mantissa, exponent_text) = scientific
        .split_once('e')
        .expect("Rust scientific formatting always contains an exponent");
    let exponent = exponent_text
        .parse::<i32>()
        .expect("Rust scientific formatting always emits an integer exponent");
    if (-4..12).contains(&exponent) {
        let decimals = usize::try_from(11_i32.saturating_sub(exponent)).unwrap_or_default();
        trim_decimal(format!("{value:.decimals$}"))
    } else {
        let mantissa = trim_decimal(mantissa.to_owned());
        format!("{mantissa}e{exponent:+03}")
    }
}

fn trim_decimal(mut value: String) -> String {
    if value.contains('.') {
        while value.ends_with('0') {
            value.pop();
        }
        if value.ends_with('.') {
            value.pop();
        }
    }
    value
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;
    use nuwax_mapping::OccupancyGrid;

    use super::*;
    use crate::offline_replay::{
        OfflinePointXyzV1, OfflinePoseV1, OfflineRaytracingConfigV1, OfflineReplayConfigV1,
        OfflineReplayFrameV1, OfflineReplayRequestV1, OfflineVoxelConfigV1,
    };
    use crate::offline_replay_owner::{GeometryOfflineReplayOwner, ReplayBarrierIdentityV1};

    fn georeference() -> OfflineGeoreferenceV1 {
        OfflineGeoreferenceV1 {
            schema_version: 1,
            map_name: "map".to_owned(),
            map_frame_id: "map_frame".to_owned(),
            source_crs: "EPSG:4326".to_owned(),
            projected_crs: "EPSG:3857".to_owned(),
            axis_convention: "projected_x_east_y_north_z_up".to_owned(),
            anchor_lat_deg: 0.0,
            anchor_lng_deg: 0.0,
            anchor_easting_m: 0.0,
            anchor_northing_m: 0.0,
            map_origin_x_m: 0.0,
            map_origin_y_m: 0.0,
            map_from_projected_yaw_rad: 0.0,
            altitude_policy: "map_plane_constant".to_owned(),
            map_plane_z_m: 0.0,
        }
    }

    fn request(kind: OfflineBackendKind) -> OfflineReplayRequestV1 {
        let backend = match kind {
            OfflineBackendKind::Voxel => OfflineBackendConfigV1::Voxel(OfflineVoxelConfigV1 {
                resolution_m: 0.05,
                voxel_size_m: 0.1,
                initial_safe_radius_m: 0.0,
                window_capacity: 8,
            }),
            OfflineBackendKind::Raytracing => {
                OfflineBackendConfigV1::Raytracing(OfflineRaytracingConfigV1 {
                    resolution_m: 0.05,
                    voxel_size_m: 0.1,
                    max_range_m: 10.0,
                    ray_subsample: 1,
                    shadow_depth_m: 0.2,
                    grace_depth_m: 0.2,
                    min_health: 0,
                    max_health: 1,
                    graze_cos: 0.7,
                    recency_window: 15,
                })
            }
        };
        OfflineReplayRequestV1 {
            map_name: "map".to_owned(),
            source_id: "source".to_owned(),
            config: OfflineReplayConfigV1 {
                map_frame_id: "map_frame".to_owned(),
                backend_kind: kind,
                backend,
                georeference: georeference(),
            },
        }
    }

    fn frame(sequence: u64) -> OfflineReplayFrameV1 {
        OfflineReplayFrameV1 {
            source_sequence: sequence,
            source_timestamp_ns: i64::try_from(sequence.saturating_mul(1_000_000_000)).unwrap(),
            sensor_frame_id: "lidar".to_owned(),
            map_frame_id: "map_frame".to_owned(),
            pose_t_map_sensor: OfflinePoseV1 {
                translation_xyz_m: [0.0, 0.0, 0.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            points_xyz: vec![
                OfflinePointXyzV1 {
                    x: 1.0,
                    y: 0.0,
                    z: 1.0,
                };
                3
            ],
        }
    }

    async fn real_snapshot(kind: OfflineBackendKind) -> OfflineGeometrySnapshotV2 {
        let mut owner = GeometryOfflineReplayOwner::start(request(kind)).unwrap();
        for sequence in 1..=3 {
            owner.submit_frame(frame(sequence)).await.unwrap();
        }
        owner.finish_v2().await.unwrap()
    }

    fn synthetic_snapshot() -> OfflineGeometrySnapshotV2 {
        let barrier = ReplayBarrierIdentityV1 {
            replay_id: "map:source".to_owned(),
            backend_generation: 1,
            last_source_sequence: 1,
            mapping_commit_sequence: 1,
        };
        let mut checkpoint = b"NWMBCKP\0".to_vec();
        checkpoint.extend_from_slice(&1_u32.to_le_bytes());
        checkpoint.extend_from_slice(&[0, 0, 0, 0]);
        checkpoint.extend_from_slice(&8_u64.to_le_bytes());
        checkpoint.extend_from_slice(&0_u64.to_le_bytes());
        checkpoint.extend_from_slice(&2_u64.to_le_bytes());
        checkpoint.extend_from_slice(&1_u64.to_le_bytes());
        let mut grid = OccupancyGrid::new(2, 3, 0.05);
        grid.data = vec![-1, 0, 64, 65, 100, 1];
        grid.origin = [1.25, -2.5, 0.0];
        grid.origin_orientation = [
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_4.sin(),
            std::f64::consts::FRAC_PI_4.cos(),
        ];
        grid.frame_id = "map_frame".to_owned();
        OfflineGeometrySnapshotV2 {
            schema_version: 2,
            barrier: barrier.clone(),
            identity: OfflineReplayIdentityV2 {
                map_name: "map".to_owned(),
                source_id: "source".to_owned(),
                map_frame_id: "map_frame".to_owned(),
            },
            backend_kind: OfflineBackendKind::Voxel,
            backend_config: request(OfflineBackendKind::Voxel).config.backend,
            georeference: georeference(),
            last_source_timestamp_ns: 1,
            checkpoint,
            canonical_raw_occupancy: grid,
            committed_transformed_accepted_cloud: vec![
                Point3::new(
                    1.234_567_890_123,
                    -2.500_000_119_209_289_6,
                    3.750_000_238_418_579,
                ),
                Point3::new(
                    -4.125_000_476_837_158,
                    5.500_000_953_674_316,
                    -6.750_001_907_348_633,
                ),
            ],
            processing_receipt: OfflineReplayProcessingReceiptV1 {
                barrier,
                committed_frame_count: 1,
                committed_point_count: 2,
                flush_committed: true,
            },
            map_snapshot_seed: OfflineMapSnapshotSeedV2 {
                map_name: "map".to_owned(),
                map_frame_id: "map_frame".to_owned(),
                update_sequence: 1,
            },
        }
    }

    #[test]
    fn non_square_nav2_assets_and_python_pcd_oracle_are_exact() {
        let candidate = export_offline_candidate_assets_v1(&synthetic_snapshot()).unwrap();
        let pgm = candidate
            .members
            .iter()
            .find(|member| member.relative_path_intent == NAV2_PGM_PATH)
            .unwrap();
        assert_eq!(
            pgm.bytes,
            [b"P5\n2 3\n255\n".as_slice(), &[0, 254, 254, 0, 205, 254]].concat()
        );
        let yaml = candidate
            .members
            .iter()
            .find(|member| member.relative_path_intent == NAV2_YAML_PATH)
            .unwrap();
        assert_eq!(
            std::str::from_utf8(&yaml.bytes).unwrap(),
            "image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [1.25, -2.5, 1.57079632679]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
        );
        let pcd = candidate
            .members
            .iter()
            .find(|member| member.relative_path_intent == PCD_PATH)
            .unwrap();
        let header = "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n";
        let values = [
            1.234_567_890_123_f64,
            -2.500_000_119_209_289_6,
            3.750_000_238_418_579,
            -4.125_000_476_837_158,
            5.500_000_953_674_316,
            -6.750_001_907_348_633,
        ];
        let mut python_oracle = header.as_bytes().to_vec();
        for value in values {
            python_oracle.extend_from_slice(&(value as f32).to_le_bytes());
        }
        assert_eq!(pcd.bytes, python_oracle);
        let points_json = "[[1.234567890123,-2.5000001192092896,3.750000238418579],[-4.125000476837158,5.500000953674316,-6.750001907348633]]";
        let python = std::process::Command::new(
            "/home/ml/Deeprob/nuwax_robot_bridge/.venv_model_download/bin/python",
        )
        .args([
            "-c",
            "import pathlib,sys,tempfile,numpy as np; sys.path[:0]=[sys.argv[1],sys.argv[2]]; from pointcloud_map import write_pcd_binary_xyz,read_pcd_xyz; p=pathlib.Path(tempfile.mkstemp(suffix='.pcd')[1]); write_pcd_binary_xyz(p,np.asarray(__import__('json').loads(sys.argv[3]),dtype=np.float64)); read_pcd_xyz(p); sys.stdout.buffer.write(p.read_bytes()); p.unlink()",
            env!("CARGO_MANIFEST_DIR"),
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../.."),
            points_json,
        ])
        .output()
        .unwrap();
        assert!(
            python.status.success(),
            "{}",
            String::from_utf8_lossy(&python.stderr)
        );
        assert_eq!(pcd.bytes, python.stdout);
        for member in &candidate.members {
            assert_eq!(
                member.receipt.relative_path_intent,
                member.relative_path_intent
            );
            assert_eq!(member.receipt.media_type, member.media_type);
            assert_eq!(member.receipt.encoding, member.encoding);
            assert_eq!(member.receipt.size_bytes, member.bytes.len() as u64);
            assert_eq!(
                member.receipt.sha256_hex,
                hex_digest(&nuwax_codec::sha256_bytes(&member.bytes))
            );
            assert!(!member.receipt.source_binding.is_empty());
        }
        assert_eq!(
            candidate.deferred,
            vec![
                DeferredAssetMemberV1::CheckpointSelectorManifestStage4,
                DeferredAssetMemberV1::TopLevelManifestStage4,
                DeferredAssetMemberV1::MappingDatabaseStage5,
                DeferredAssetMemberV1::FinalMapSnapshotStage5,
                DeferredAssetMemberV1::MapAssetBundleStage5,
            ]
        );
    }

    #[tokio::test]
    async fn real_stage2_voxel_and_ray_snapshots_export_repeatably() {
        for kind in [OfflineBackendKind::Voxel, OfflineBackendKind::Raytracing] {
            let snapshot = real_snapshot(kind).await;
            let first = export_offline_candidate_assets_v1(&snapshot).unwrap();
            let second = export_offline_candidate_assets_v1(&snapshot).unwrap();
            assert_eq!(first.members, second.members);
            assert_eq!(first.members.len(), 4);
            let checkpoint = first
                .members
                .iter()
                .find(|member| member.encoding == CandidateAssetEncodingV1::CheckpointObjectV1)
                .unwrap();
            assert_eq!(checkpoint.bytes, snapshot.checkpoint);
            assert_eq!(
                checkpoint.receipt.sha256_hex,
                hex_digest(&nuwax_codec::sha256_bytes(&snapshot.checkpoint))
            );
            assert!(checkpoint.relative_path_intent.starts_with(match kind {
                OfflineBackendKind::Voxel => "mapping_backend/voxel/objects/sha256-",
                OfflineBackendKind::Raytracing => {
                    "mapping_backend/raytracing/objects/sha256-"
                }
            }));
            assert!(first.members.windows(2).all(|pair| {
                pair[0].relative_path_intent.as_bytes() < pair[1].relative_path_intent.as_bytes()
            }));
        }
    }

    #[tokio::test]
    async fn complete_checkpoint_validation_rejects_truncation_tail_and_wrong_config() {
        for kind in [OfflineBackendKind::Voxel, OfflineBackendKind::Raytracing] {
            let snapshot = real_snapshot(kind).await;
            for length in [16, snapshot.checkpoint.len() - 1] {
                let mut invalid = snapshot.clone();
                invalid.checkpoint.truncate(length);
                assert_eq!(
                    export_offline_candidate_assets_v1(&invalid).unwrap_err(),
                    OfflineAssetExportError::InvalidCheckpointHeader
                );
            }
            let mut trailing = snapshot.clone();
            trailing.checkpoint.push(0);
            assert_eq!(
                export_offline_candidate_assets_v1(&trailing).unwrap_err(),
                OfflineAssetExportError::InvalidCheckpointHeader
            );
            let mut oversized = snapshot.clone();
            oversized.checkpoint =
                vec![0; crate::mapping_backend::MAX_MAPPING_BACKEND_CHECKPOINT_BYTES + 1];
            assert_eq!(
                export_offline_candidate_assets_v1(&oversized).unwrap_err(),
                OfflineAssetExportError::InvalidCheckpointHeader
            );
            if kind == OfflineBackendKind::Voxel {
                let mut retained_overflow = snapshot.clone();
                retained_overflow.checkpoint[16..24].copy_from_slice(&u64::MAX.to_le_bytes());
                retained_overflow.checkpoint[24..32].copy_from_slice(&0_u64.to_le_bytes());
                retained_overflow.checkpoint[32..40]
                    .copy_from_slice(&(MAX_RETAINED_VOXEL_POINTS as u64 + 1).to_le_bytes());
                assert_eq!(
                    export_offline_candidate_assets_v1(&retained_overflow).unwrap_err(),
                    OfflineAssetExportError::InvalidCheckpointHeader
                );
            }
            let mut wrong_config = snapshot;
            match &mut wrong_config.backend_config {
                OfflineBackendConfigV1::Voxel(config) => config.window_capacity += 1,
                OfflineBackendConfigV1::Raytracing(config) => config.voxel_size_m *= 2.0,
            }
            assert_eq!(
                export_offline_candidate_assets_v1(&wrong_config).unwrap_err(),
                OfflineAssetExportError::InvalidCheckpointHeader
            );
        }
    }

    #[test]
    fn hostile_occupancy_lengths_return_error_without_panicking() {
        let mut oversized_frame = vec![0_u8; 92];
        oversized_frame[0..4].copy_from_slice(&1_u32.to_le_bytes());
        oversized_frame[4..8].copy_from_slice(&1_u32.to_le_bytes());
        oversized_frame[8..16].copy_from_slice(&1.0_f64.to_bits().to_le_bytes());
        oversized_frame[80..84].copy_from_slice(&u32::MAX.to_le_bytes());
        oversized_frame[84..92].copy_from_slice(&1_u64.to_le_bytes());
        let result = std::panic::catch_unwind(|| validate_checkpoint_occupancy(&oversized_frame));
        assert_eq!(
            result.unwrap(),
            Err(OfflineAssetExportError::InvalidCheckpointHeader)
        );

        let mut overflowing_total = oversized_frame;
        overflowing_total[80..84].copy_from_slice(&0_u32.to_le_bytes());
        overflowing_total[84..92].copy_from_slice(&u64::MAX.to_le_bytes());
        let result = std::panic::catch_unwind(|| validate_checkpoint_occupancy(&overflowing_total));
        assert_eq!(
            result.unwrap(),
            Err(OfflineAssetExportError::InvalidCheckpointHeader)
        );
    }

    #[test]
    fn python_g12_matches_python_subprocess_boundaries() {
        let values = [
            0.0,
            -0.0,
            1.0,
            -1.0,
            0.000_099_999_999_999_9,
            0.0001,
            0.000_123_456_789_012_3,
            9_999_999_999.99,
            99_999_999_999.9,
            1_000_000_000_000.0,
            -1.234_567_890_123e-12,
        ];
        let python = std::process::Command::new("python3")
            .args([
                "-c",
                "import sys; print('\\n'.join('%.12g' % float(v) for v in sys.argv[1:]))",
            ])
            .args(values.iter().map(|value| format!("{value:?}")))
            .output()
            .unwrap();
        assert!(python.status.success());
        let expected = String::from_utf8(python.stdout).unwrap();
        let actual = values
            .iter()
            .map(|value| python_g12(*value))
            .collect::<Vec<_>>()
            .join("\n")
            + "\n";
        assert_eq!(actual, expected);
    }

    #[test]
    fn duplicate_path_guard_uses_the_production_helper() {
        let bytes = vec![1];
        let first = member(
            "same",
            "application/octet-stream",
            CandidateAssetEncodingV1::CheckpointObjectV1,
            bytes.clone(),
            "test",
        )
        .unwrap();
        let second = member(
            "same",
            "application/octet-stream",
            CandidateAssetEncodingV1::CheckpointObjectV1,
            bytes,
            "test",
        )
        .unwrap();
        assert_eq!(
            sort_and_validate_members(vec![first, second]).unwrap_err(),
            OfflineAssetExportError::DuplicatePath
        );
    }
}
