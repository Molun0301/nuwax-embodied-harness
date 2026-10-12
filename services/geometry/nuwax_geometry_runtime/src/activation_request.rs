//! Consuming, immutable activation-v2 admission. No lifecycle or production route lives here.
//!
//! Request metadata is small and independent of checkpoint ownership. A live checkpoint owns
//! each incoming payload allocation once; the interner owns only weak references. In particular,
//! retaining request metadata after retirement does not retain backend, occupancy or cloud bytes.
#![cfg_attr(not(test), allow(dead_code))]

use std::collections::HashMap;
use std::fs::File;
use std::os::unix::fs::FileExt;
use std::sync::{Arc, Weak};

use parking_lot::Mutex;
use prost::Message;

use crate::activation_identity::{
    cloud_receipt_from_wire, occupancy_receipt_from_wire, AssetIdentityV2, CanonicalCloudTargetV2,
    CheckpointIdentityV2, CheckpointMemberIdentityV2,
};
use crate::mapping_backend::MappingBackendKind;
use crate::mapping_commit::{CloudCommitReceipt, OccupancyCommitReceipt};
use crate::proto;

/// Exact, lightweight ledger material. This type never owns checkpoint bytes or file handles.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ActivationRequestMetadataV2 {
    activation_request_id: String,
    session_id: String,
    map_name: String,
    asset_identity: AssetIdentityV2,
    checkpoint_identity: CheckpointIdentityV2,
    expected_runtime_generation: u64,
}

impl ActivationRequestMetadataV2 {
    /// Validate only immutable scalar identity before looking up permanent tombstones.
    pub(crate) fn from_wire(
        request: &proto::ActivateMapRequest,
        expected_runtime_generation: u64,
    ) -> Result<Self, String> {
        if request.activation_contract_version != 2
            || expected_runtime_generation == 0
            || request.expected_runtime_generation != expected_runtime_generation
            || !request.asset_manifest_sha256.is_empty()
            || !request.asset_bundle_sha256.is_empty()
            || request.checkpoint.is_some()
            || request.cloud_checkpoint.is_some()
        {
            return Err("ACTIVATE_MAP_V2_IDENTITY_FIELDS_MIXED_OR_STALE".into());
        }
        crate::server::validate_canonical_protocol_id(
            &request.activation_request_id,
            "activation_request_id",
        )?;
        crate::server::validate_canonical_protocol_id(&request.session_id, "session_id")?;
        crate::server::validate_canonical_text(&request.map_name, "map_name")?;
        if request.map_name.chars().any(char::is_control) {
            return Err("map_name must not contain control characters".into());
        }
        let asset_identity = AssetIdentityV2::try_from(
            request
                .asset_identity
                .as_ref()
                .ok_or("v2 asset_identity is required")?,
        )?;
        let checkpoint = request
            .mapping_backend_checkpoint
            .as_ref()
            .ok_or("v2 mapping_backend_checkpoint is required")?;
        if checkpoint.schema_version != 2 {
            return Err("v2 activation requires assigned checkpoint schema 2".into());
        }
        Ok(Self {
            activation_request_id: request.activation_request_id.clone(),
            session_id: request.session_id.clone(),
            map_name: request.map_name.clone(),
            asset_identity,
            checkpoint_identity: CheckpointIdentityV2::parse(&checkpoint.checkpoint_id)?,
            expected_runtime_generation,
        })
    }
    /// Decode the existing receipt metadata without manufacturing checkpoint bytes.
    pub(crate) fn from_receipt_wire(value: proto::ActivationRequestMetadataV2) -> Result<Self, String> {
        crate::server::validate_canonical_protocol_id(&value.activation_request_id, "activation_request_id")?;
        crate::server::validate_canonical_protocol_id(&value.session_id, "session_id")?;
        crate::server::validate_canonical_text(&value.map_name, "map_name")?;
        if value.map_name.chars().any(char::is_control) || value.expected_runtime_generation == 0 {
            return Err("assigned activation receipt metadata is incomplete".into());
        }
        Ok(Self {
            activation_request_id: value.activation_request_id,
            session_id: value.session_id,
            map_name: value.map_name,
            asset_identity: AssetIdentityV2::try_from(value.asset_identity.as_ref().ok_or("assigned asset identity is missing")?)?,
            checkpoint_identity: CheckpointIdentityV2::parse(&value.checkpoint_id)?,
            expected_runtime_generation: value.expected_runtime_generation,
        })
    }
    pub(crate) fn activation_request_id(&self) -> &str {
        &self.activation_request_id
    }
    pub(crate) fn session_id(&self) -> &str {
        &self.session_id
    }
    pub(crate) fn map_name(&self) -> &str {
        &self.map_name
    }
    pub(crate) fn asset_identity(&self) -> AssetIdentityV2 {
        self.asset_identity
    }
    pub(crate) fn checkpoint_identity(&self) -> CheckpointIdentityV2 {
        self.checkpoint_identity
    }
    pub(crate) fn expected_runtime_generation(&self) -> u64 {
        self.expected_runtime_generation
    }
}

/// A read-only descriptor opened from the sealed transfer store. Its independent file handle
/// keeps the archive alive after transfer release/expiry, without keeping a transfer lease alive.
#[derive(Clone, Debug)]
pub(crate) struct SealedArtifactOwnerV2 {
    reference: proto::GraphArtifactRef,
    file: Arc<File>,
}

impl SealedArtifactOwnerV2 {
    fn pin(
        reference: proto::GraphArtifactRef,
        pin_artifact: &mut impl FnMut(&proto::GraphArtifactRef) -> Result<File, String>,
    ) -> Result<Self, String> {
        let file = pin_artifact(&reference)?;
        let metadata = file.metadata().map_err(|error| error.to_string())?;
        if !metadata.is_file() || metadata.len() != reference.size {
            return Err("sealed artifact file differs from its exact reference".into());
        }
        Ok(Self {
            reference,
            file: Arc::new(file),
        })
    }

    pub(crate) fn reference(&self) -> &proto::GraphArtifactRef {
        &self.reference
    }
    pub(crate) fn file(&self) -> &File {
        &self.file
    }

    fn exact_eq(&self, other: &Self) -> Result<bool, String> {
        if self.reference != other.reference {
            return Ok(false);
        }
        if Arc::ptr_eq(&self.file, &other.file) {
            return Ok(true);
        }
        exact_file_bytes(&self.file, &other.file, self.reference.size)
    }
}

/// Compare independent sealed owners without seeking their shared cursors, serializing an
/// archive, hashing, or allocating an archive-sized buffer. Reuse the existing transfer budget.
fn exact_file_bytes(left: &File, right: &File, size: u64) -> Result<bool, String> {
    if left.metadata().map_err(|error| error.to_string())?.len() != size
        || right.metadata().map_err(|error| error.to_string())?.len() != size
    {
        return Err("sealed artifact length changed during replay comparison".into());
    }
    let capacity = size.min(nuwax_slam_2d_runtime::archive::CHUNK_BYTES as u64) as usize;
    let mut left_bytes = vec![0; capacity];
    let mut right_bytes = vec![0; capacity];
    let mut offset = 0;
    while offset < size {
        let length = (size - offset).min(capacity as u64) as usize;
        left.read_exact_at(&mut left_bytes[..length], offset)
            .map_err(|error| error.to_string())?;
        right
            .read_exact_at(&mut right_bytes[..length], offset)
            .map_err(|error| error.to_string())?;
        if left_bytes[..length] != right_bytes[..length] {
            return Ok(false);
        }
        offset += length as u64;
    }
    Ok(true)
}

fn exact_optional_artifact(
    left: Option<&SealedArtifactOwnerV2>,
    right: Option<&SealedArtifactOwnerV2>,
) -> Result<bool, String> {
    match (left, right) {
        (Some(left), Some(right)) => left.exact_eq(right),
        (None, None) => Ok(true),
        _ => Ok(false),
    }
}

fn exact_occupancy(left: &proto::OccupancyGridData, right: &proto::OccupancyGridData) -> bool {
    let (Some(left_meta), Some(right_meta)) = (&left.meta, &right.meta) else {
        return false;
    };
    left_meta.width == right_meta.width
        && left_meta.height == right_meta.height
        && left_meta.resolution.to_bits() == right_meta.resolution.to_bits()
        && left_meta.origin_x.to_bits() == right_meta.origin_x.to_bits()
        && left_meta.origin_y.to_bits() == right_meta.origin_y.to_bits()
        && left_meta.frame_id == right_meta.frame_id
        && left_meta.revision == right_meta.revision
        && left_meta.timestamp_ns == right_meta.timestamp_ns
        && left.data == right.data
}

// Deliberately not Clone: the only clone operation is on the enclosing Arc handle.
#[derive(Debug)]
struct CanonicalCheckpointDataV2 {
    identity: CheckpointIdentityV2,
    backend_kind: MappingBackendKind,
    backend_payload_schema_version: u32,
    created_at_ns: u64,
    backend_member: Option<CheckpointMemberIdentityV2>,
    occupancy_member: CheckpointMemberIdentityV2,
    payload: Vec<u8>,
    occupancy: proto::OccupancyGridData,
    cloud: Option<CanonicalCloudTargetV2>,
    graph_member: Option<CheckpointMemberIdentityV2>,
    graph: Option<SealedArtifactOwnerV2>,
    fine_map: Option<SealedArtifactOwnerV2>,
    dense_cloud: Option<SealedArtifactOwnerV2>,
}

/// An immutable source owner shared across requests, assets and activation attempts.
#[derive(Clone, Debug)]
pub(crate) struct CanonicalCheckpointOwnerV2(Arc<CanonicalCheckpointDataV2>);

#[derive(Clone, Debug)]
pub(crate) struct WeakCheckpointOwnerV2(Weak<CanonicalCheckpointDataV2>);

impl WeakCheckpointOwnerV2 {
    pub(crate) fn upgrade(&self) -> Option<CanonicalCheckpointOwnerV2> {
        self.0.upgrade().map(CanonicalCheckpointOwnerV2)
    }
}

impl CanonicalCheckpointOwnerV2 {
    fn admit(
        mut checkpoint: proto::MappingBackendCheckpoint,
        expected_backend: MappingBackendKind,
        expected_map_frame: &str,
        retained: Option<&Self>,
        pin_artifact: &mut impl FnMut(&proto::GraphArtifactRef) -> Result<File, String>,
    ) -> Result<Self, String> {
        // Finish all request/checkpoint validation before invoking the pin callback.
        crate::server::validate_assigned_mapping_checkpoint(&checkpoint, expected_backend)?;
        crate::server::validate_canonical_protocol_id(
            &checkpoint.source_session_id,
            "checkpoint source_session_id",
        )?;
        let identity = CheckpointIdentityV2::parse(&checkpoint.checkpoint_id)?;
        let occupancy_receipt = occupancy_receipt_from_wire(
            checkpoint
                .occupancy_receipt
                .as_ref()
                .ok_or("occupancy receipt required")?,
        )?;
        let occupancy_member = CheckpointMemberIdentityV2::try_from(
            checkpoint
                .occupancy_identity
                .take()
                .ok_or("occupancy member required")?,
        )?;
        let occupancy = checkpoint
            .canonical_occupancy_checkpoint
            .take()
            .ok_or("canonical occupancy required")?;
        let (backend_member, cloud, graph_member) = match expected_backend {
            MappingBackendKind::SlamToolbox => {
                let member = CheckpointMemberIdentityV2::try_from(
                    checkpoint.graph_artifact.as_ref()
                        .and_then(|reference| reference.member_identity.clone())
                        .ok_or("planar graph member required")?,
                )?;
                if member.frame_id() != expected_map_frame {
                    return Err("planar target frame must match the canonical map frame".into());
                }
                (None, None, Some(member))
            }
            MappingBackendKind::Voxel | MappingBackendKind::Raytracing => {
                let cloud_receipt = cloud_receipt_from_wire(
                    checkpoint.cloud_receipt.as_ref().ok_or("cloud receipt required")?,
                )?;
                let backend_member = CheckpointMemberIdentityV2::try_from(
                    checkpoint.backend_identity.take().ok_or("backend member required")?,
                )?;
                let cloud = CanonicalCloudTargetV2::admit(
                    checkpoint.canonical_cloud_checkpoint.take().ok_or("canonical cloud required")?,
                    identity,
                    &occupancy_receipt,
                    &cloud_receipt,
                    expected_map_frame,
                )?;
                (Some(backend_member), Some(cloud), None)
            }
        };

        let (fine_map, dense_cloud, graph) = if let Some(retained) = retained {
            // An assigned object already has one authoritative owner. Transfer leases may
            // expire, or even be re-created under a conflicting ID, without changing it.
            // Do not consult that temporary namespace for retries of a live source.
            if checkpoint.fine_map_artifact.as_ref() != retained.fine_map().map(|v| v.reference())
                || checkpoint.dense_cloud_artifact.as_ref()
                    != retained.dense_cloud().map(|v| v.reference())
                || checkpoint.graph_artifact.as_ref() != retained.graph().map(|v| v.reference())
            {
                return Err("assigned checkpoint artifact descriptor replay conflict".into());
            }
            (retained.0.fine_map.clone(), retained.0.dense_cloud.clone(), retained.0.graph.clone())
        } else {
            // Store::open validates the exact reference and sealed state. Its lock need only
            // cover each callback, never validation, equality or the interner critical section.
            let fine_map = checkpoint
                .fine_map_artifact
                .take()
                .map(|reference| SealedArtifactOwnerV2::pin(reference, pin_artifact))
                .transpose()?;
            let dense_cloud = checkpoint
                .dense_cloud_artifact
                .take()
                .map(|reference| SealedArtifactOwnerV2::pin(reference, pin_artifact))
                .transpose()?;
            let graph = checkpoint
                .graph_artifact
                .take()
                .map(|reference| SealedArtifactOwnerV2::pin(reference, pin_artifact))
                .transpose()?;
            (fine_map, dense_cloud, graph)
        };
        Ok(Self(Arc::new(CanonicalCheckpointDataV2 {
            identity,
            backend_kind: expected_backend,
            backend_payload_schema_version: checkpoint.backend_payload_schema_version,
            created_at_ns: checkpoint.created_at_ns,
            backend_member,
            occupancy_member,
            payload: checkpoint.payload,
            occupancy,
            cloud,
            graph_member,
            graph,
            fine_map,
            dense_cloud,
        })))
    }

    pub(crate) fn identity(&self) -> CheckpointIdentityV2 {
        self.0.identity
    }
    pub(crate) fn backend_kind(&self) -> MappingBackendKind {
        self.0.backend_kind
    }
    pub(crate) fn backend_payload_schema_version(&self) -> u32 {
        self.0.backend_payload_schema_version
    }
    pub(crate) fn created_at_ns(&self) -> u64 {
        self.0.created_at_ns
    }
    pub(crate) fn backend_member(&self) -> &CheckpointMemberIdentityV2 {
        self.0.backend_member.as_ref().expect("spatial checkpoint backend member")
    }
    pub(crate) fn occupancy_member(&self) -> &CheckpointMemberIdentityV2 {
        &self.0.occupancy_member
    }
    pub(crate) fn occupancy_receipt(&self) -> &OccupancyCommitReceipt {
        self.0.occupancy_member.occupancy_receipt()
    }
    pub(crate) fn cloud_receipt(&self) -> &CloudCommitReceipt {
        self.backend_member()
            .cloud_receipt()
            .expect("validated cloud receipt")
    }
    pub(crate) fn backend_payload(&self) -> &[u8] {
        &self.0.payload
    }
    pub(crate) fn occupancy(&self) -> &proto::OccupancyGridData {
        &self.0.occupancy
    }
    pub(crate) fn cloud(&self) -> &CanonicalCloudTargetV2 {
        self.0.cloud.as_ref().expect("spatial checkpoint cloud target")
    }
    pub(crate) fn graph(&self) -> Option<&SealedArtifactOwnerV2> {
        self.0.graph.as_ref()
    }
    pub(crate) fn source_member(&self) -> &CheckpointMemberIdentityV2 {
        match self.backend_kind() {
            MappingBackendKind::SlamToolbox => self.0.graph_member.as_ref().expect("planar graph member"),
            MappingBackendKind::Voxel | MappingBackendKind::Raytracing => self.cloud().member(),
        }
    }
    pub(crate) fn fine_map(&self) -> Option<&SealedArtifactOwnerV2> {
        self.0.fine_map.as_ref()
    }
    pub(crate) fn dense_cloud(&self) -> Option<&SealedArtifactOwnerV2> {
        self.0.dense_cloud.as_ref()
    }
    pub(crate) fn downgrade(&self) -> WeakCheckpointOwnerV2 {
        WeakCheckpointOwnerV2(Arc::downgrade(&self.0))
    }
    pub(crate) fn shares_source_with(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    pub(crate) fn validate_replay(&self, candidate: &Self) -> Result<(), String> {
        if self.shares_source_with(candidate) {
            return Ok(());
        }
        let left = &self.0;
        let right = &candidate.0;
        if left.identity != right.identity
            || left.backend_kind != right.backend_kind
            || left.backend_payload_schema_version != right.backend_payload_schema_version
            || left.created_at_ns != right.created_at_ns
            || left.backend_member != right.backend_member
            || left.occupancy_member != right.occupancy_member
            || left.payload != right.payload
            || !exact_occupancy(&left.occupancy, &right.occupancy)
            || left.cloud != right.cloud
            || left.graph_member != right.graph_member
            // Graph ownership is pinned by the sealed Store and retained on replay. The
            // assigned descriptor is compared here; do not rescan whole archives by ID.
            || left.graph.as_ref().map(|value| value.reference())
                != right.graph.as_ref().map(|value| value.reference())
            || !exact_optional_artifact(left.fine_map.as_ref(), right.fine_map.as_ref())?
            || !exact_optional_artifact(left.dense_cloud.as_ref(), right.dense_cloud.as_ref())?
        {
            return Err("assigned checkpoint replay conflict".into());
        }
        Ok(())
    }
}

/// Weak-only source interning, not a lifecycle ledger. Dead entries are removed on admission
/// and can be explicitly swept on retirement. No payload comparison occurs under this lock.
#[derive(Default)]
pub(crate) struct CheckpointOwnerInternerV2 {
    owners: Mutex<HashMap<CheckpointIdentityV2, Weak<CanonicalCheckpointDataV2>>>,
}

impl CheckpointOwnerInternerV2 {
    pub(crate) fn lookup(
        &self,
        identity: CheckpointIdentityV2,
    ) -> Option<CanonicalCheckpointOwnerV2> {
        self.owners
            .lock()
            .get(&identity)
            .and_then(Weak::upgrade)
            .map(CanonicalCheckpointOwnerV2)
    }

    pub(crate) fn intern(
        &self,
        candidate: CanonicalCheckpointOwnerV2,
    ) -> Result<CanonicalCheckpointOwnerV2, String> {
        let retained = {
            let mut owners = self.owners.lock();
            owners.retain(|_, owner| owner.strong_count() != 0);
            if let Some(retained) = owners.get(&candidate.identity()).and_then(Weak::upgrade) {
                CanonicalCheckpointOwnerV2(retained)
            } else {
                // This insertion is serialized with the lookup. A concurrent first admission
                // upgrades this winner and compares against it rather than replacing it.
                owners.insert(candidate.identity(), Arc::downgrade(&candidate.0));
                return Ok(candidate);
            }
        };
        retained.validate_replay(&candidate)?;
        // Dropping the candidate frees its three incoming allocations and independent files.
        Ok(retained)
    }

    pub(crate) fn prune_expired(&self) {
        self.owners
            .lock()
            .retain(|_, owner| owner.strong_count() != 0);
    }
}

#[derive(Clone, Debug)]
pub(crate) struct ValidatedActivationRequestV2 {
    metadata: ActivationRequestMetadataV2,
    checkpoint: CanonicalCheckpointOwnerV2,
}

impl ValidatedActivationRequestV2 {
    /// `pin_artifact` must call the sealed Store::open for the supplied exact reference.
    /// Pass a callback that takes/releases the store lock per call, not a prelocked store.
    pub(crate) fn admit(
        mut request: proto::ActivateMapRequest,
        expected_runtime_generation: u64,
        expected_backend: MappingBackendKind,
        expected_map_frame: &str,
        mut pin_artifact: impl FnMut(&proto::GraphArtifactRef) -> Result<File, String>,
        interner: &CheckpointOwnerInternerV2,
    ) -> Result<Self, String> {
        crate::server::validate_mapping_checkpoint_envelope_size(
            "ActivateMap v2 request",
            request.encoded_len(),
        )?;
        let metadata =
            ActivationRequestMetadataV2::from_wire(&request, expected_runtime_generation)?;
        let checkpoint = request
            .mapping_backend_checkpoint
            .take()
            .ok_or("v2 mapping_backend_checkpoint is required")?;
        let identity = CheckpointIdentityV2::parse(&checkpoint.checkpoint_id)?;
        // The upgraded handle pins the authoritative owner throughout validation/comparison.
        let retained = interner.lookup(identity);
        let checkpoint = CanonicalCheckpointOwnerV2::admit(
            checkpoint,
            expected_backend,
            expected_map_frame,
            retained.as_ref(),
            &mut pin_artifact,
        )?;
        Ok(Self {
            metadata,
            checkpoint: interner.intern(checkpoint)?,
        })
    }

    pub(crate) fn metadata(&self) -> &ActivationRequestMetadataV2 {
        &self.metadata
    }
    pub(crate) fn checkpoint(&self) -> &CanonicalCheckpointOwnerV2 {
        &self.checkpoint
    }
    pub(crate) fn into_parts(self) -> (ActivationRequestMetadataV2, CanonicalCheckpointOwnerV2) {
        (self.metadata, self.checkpoint)
    }
    pub(crate) fn validate_replay(&self, candidate: &Self) -> Result<(), String> {
        if self.metadata != candidate.metadata {
            return Err("activation v2 request replay conflict".into());
        }
        self.checkpoint.validate_replay(&candidate.checkpoint)
    }
}

impl crate::mapping_backend::MappingBackendEncodedOwner for CanonicalCheckpointOwnerV2 {
    fn checkpoint_bytes(&self) -> &[u8] { self.backend_payload() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{CheckpointMemberRole, GraphArtifactTransferRequest};
    use std::io::{Read, Seek, SeekFrom, Write};

    const CHECKPOINT_ID: &str = "11111111111111111111111111111111";
    const ASSET_ID: &str = "22222222222222222222222222222222";

    fn wire_request() -> proto::ActivateMapRequest {
        let commit = proto::MappingCommitIdentity {
            boot_nonce: vec![1; 16],
            sequence: 5,
        };
        let occupancy = proto::CanonicalOccupancyCommitReceipt {
            commit_id: Some(commit.clone()),
            session_id: "source".into(),
            session_generation: 2,
            occupancy_revision: 7,
            source_timestamp_ns: 100,
            backend_kind: "voxel".into(),
            backend_generation: 3,
        };
        let cloud = proto::CanonicalCloudCommitReceipt {
            commit_id: Some(commit),
            session_generation: 2,
            occupancy_revision: 7,
            cloud_revision: 9,
            source_timestamp_ns: 100,
            point_count: 1,
        };
        let member = |role| proto::CheckpointMemberIdentity {
            checkpoint_id: CHECKPOINT_ID.into(),
            role: role as i32,
            occupancy_receipt: Some(occupancy.clone()),
            cloud_receipt: Some(cloud.clone()),
            frame_id: "map".into(),
            frame_generation: 0,
            producer_generation: 0,
        };
        let payload = [
            b"NWMBCKP\0".as_slice(),
            &1_u32.to_le_bytes(),
            &[0; 4],
            &[0; 40],
        ]
        .concat();
        let checkpoint = proto::MappingBackendCheckpoint {
            schema_version: 2,
            backend_kind: "voxel".into(),
            source_session_generation: 2,
            payload_size: payload.len() as u64,
            payload,
            created_at_ns: 101,
            source_session_id: "source".into(),
            checkpoint_id: CHECKPOINT_ID.into(),
            backend_payload_schema_version: 1,
            backend_identity: Some(member(CheckpointMemberRole::BackendState)),
            occupancy_identity: Some(member(CheckpointMemberRole::CanonicalOccupancy)),
            canonical_occupancy_checkpoint: Some(proto::OccupancyGridData {
                meta: Some(proto::OccupancyGridMeta {
                    width: 1,
                    height: 1,
                    resolution: 0.05,
                    origin_x: 0.0,
                    origin_y: 0.0,
                    frame_id: "map".into(),
                    revision: 7,
                    timestamp_ns: 100,
                }),
                data: vec![0],
            }),
            canonical_cloud_checkpoint: Some(proto::CloudMapCheckpoint {
                frame_id: "map".into(),
                source_revision: 9,
                point_count: 1,
                voxel_size: 0.05,
                xyz_f32_le: [1_f32, 2.0, 3.0]
                    .into_iter()
                    .flat_map(f32::to_le_bytes)
                    .collect(),
                payload_schema_version: 2,
                member_identity: Some(member(CheckpointMemberRole::CanonicalCloud)),
                ..Default::default()
            }),
            occupancy_receipt: Some(occupancy),
            cloud_receipt: Some(cloud),
            ..Default::default()
        };
        proto::ActivateMapRequest {
            session_id: "destination".into(),
            map_name: "Map One".into(),
            activation_request_id: "activate-1".into(),
            expected_runtime_generation: 17,
            activation_contract_version: 2,
            asset_identity: Some(proto::AssetIdentityV2 {
                asset_id: ASSET_ID.into(),
            }),
            mapping_backend_checkpoint: Some(checkpoint),
            ..Default::default()
        }
    }

    fn admit(
        request: proto::ActivateMapRequest,
        interner: &CheckpointOwnerInternerV2,
    ) -> Result<ValidatedActivationRequestV2, String> {
        ValidatedActivationRequestV2::admit(
            request,
            17,
            MappingBackendKind::Voxel,
            "map",
            |_| Err("fixture unexpectedly requested an artifact".into()),
            interner,
        )
    }

    fn add_dense(
        request: &mut proto::ActivateMapRequest,
        store: &mut crate::planar_artifacts::Store,
    ) {
        let checkpoint = request.mapping_backend_checkpoint.as_mut().unwrap();
        let mut member = checkpoint.backend_identity.clone().unwrap();
        member.role = CheckpointMemberRole::DenseCloud as i32;
        let receipt = cloud_receipt_from_wire(checkpoint.cloud_receipt.as_ref().unwrap()).unwrap();
        let mut dense = crate::dense_cloud::DenseCloud::new().unwrap();
        dense
            .append(&[
                nalgebra::Point3::new(1.0, 2.0, 3.0),
                nalgebra::Point3::new(4.0, 5.0, 6.0),
            ])
            .unwrap();
        let (file, size) = dense
            .snapshot(&receipt, "source")
            .unwrap()
            .archive("map")
            .unwrap();
        checkpoint.dense_cloud_artifact = Some(store.insert(file, size, member).unwrap());
    }

    fn transfer(
        store: &mut crate::planar_artifacts::Store,
        reference: &proto::GraphArtifactRef,
        action: u32,
        offset: u64,
        data: Vec<u8>,
    ) {
        store
            .transfer(GraphArtifactTransferRequest {
                artifact: Some(reference.clone()),
                action,
                offset,
                data,
            })
            .unwrap();
    }

    fn raw_file(bytes: &[u8]) -> File {
        let path = std::env::temp_dir().join(format!(
            "activation-v2-test-{}",
            uuid::Uuid::new_v4().simple()
        ));
        let mut file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .open(&path)
            .unwrap();
        std::fs::remove_file(path).unwrap();
        file.write_all(bytes).unwrap();
        file
    }

    #[test]
    fn assigned_v2_request_moves_all_three_payload_allocations_once() {
        let request = wire_request();
        let checkpoint = request.mapping_backend_checkpoint.as_ref().unwrap();
        let backend_pointer = checkpoint.payload.as_ptr();
        let occupancy_pointer = checkpoint
            .canonical_occupancy_checkpoint
            .as_ref()
            .unwrap()
            .data
            .as_ptr();
        let cloud_pointer = checkpoint
            .canonical_cloud_checkpoint
            .as_ref()
            .unwrap()
            .xyz_f32_le
            .as_ptr();
        let interner = CheckpointOwnerInternerV2::default();
        let admitted = admit(request, &interner).unwrap();
        let cloned = admitted.clone();
        for request in [&admitted, &cloned] {
            assert_eq!(
                request.checkpoint().backend_payload().as_ptr(),
                backend_pointer
            );
            assert_eq!(
                request.checkpoint().occupancy().data.as_ptr(),
                occupancy_pointer
            );
            assert_eq!(
                request.checkpoint().cloud().xyz_f32_le().as_ptr(),
                cloud_pointer
            );
        }
        assert!(admitted
            .checkpoint()
            .shares_source_with(cloned.checkpoint()));
        admitted.validate_replay(&cloned).unwrap();
    }

    #[test]
    fn assigned_v2_repeated_and_new_request_ids_share_one_checkpoint_owner() {
        let interner = CheckpointOwnerInternerV2::default();
        let first = admit(wire_request(), &interner).unwrap();
        for index in 0..16 {
            let mut request = wire_request();
            if index > 0 {
                request.activation_request_id = format!("activation-{index}");
            }
            let next = admit(request, &interner).unwrap();
            assert!(first.checkpoint().shares_source_with(next.checkpoint()));
            assert_eq!(interner.owners.lock().len(), 1);
            if index == 0 {
                first.validate_replay(&next).unwrap();
            } else {
                assert!(first.validate_replay(&next).is_err());
            }
        }
    }

    #[test]
    fn assigned_v2_exact_request_metadata_is_not_normalized_or_substituted() {
        let interner = CheckpointOwnerInternerV2::default();
        let first = admit(wire_request(), &interner).unwrap();
        for mutation in 0..4 {
            let mut wire = wire_request();
            match mutation {
                0 => wire.activation_request_id.push('2'),
                1 => wire.session_id.push('2'),
                2 => wire.map_name.push('2'),
                _ => wire.asset_identity.as_mut().unwrap().asset_id = "3".repeat(32),
            }
            let other = admit(wire, &interner).unwrap();
            assert!(first.checkpoint().shares_source_with(other.checkpoint()));
            assert!(first.validate_replay(&other).is_err());
        }
        assert_eq!(first.metadata().activation_request_id(), "activate-1");
        assert_eq!(first.metadata().session_id(), "destination");
        assert_eq!(first.metadata().map_name(), "Map One");
        assert_eq!(
            first.metadata().asset_identity(),
            AssetIdentityV2::parse(ASSET_ID).unwrap()
        );
        assert_eq!(
            first.metadata().checkpoint_identity(),
            CheckpointIdentityV2::parse(CHECKPOINT_ID).unwrap()
        );
        assert_eq!(first.metadata().expected_runtime_generation(), 17);
    }

    #[test]
    fn assigned_v2_same_checkpoint_id_rejects_changed_body_or_exact_metadata() {
        let interner = CheckpointOwnerInternerV2::default();
        let first = admit(wire_request(), &interner).unwrap();
        for mutation in 0..8 {
            let mut request = wire_request();
            let checkpoint = request.mapping_backend_checkpoint.as_mut().unwrap();
            match mutation {
                0 => checkpoint.payload[20] = 1,
                1 => checkpoint.created_at_ns += 1,
                2 => {
                    checkpoint
                        .canonical_occupancy_checkpoint
                        .as_mut()
                        .unwrap()
                        .data[0] = 1
                }
                3 => {
                    checkpoint
                        .canonical_occupancy_checkpoint
                        .as_mut()
                        .unwrap()
                        .meta
                        .as_mut()
                        .unwrap()
                        .origin_x = -0.0
                }
                4 => {
                    checkpoint
                        .canonical_occupancy_checkpoint
                        .as_mut()
                        .unwrap()
                        .meta
                        .as_mut()
                        .unwrap()
                        .origin_y = -0.0
                }
                5 => {
                    checkpoint
                        .canonical_occupancy_checkpoint
                        .as_mut()
                        .unwrap()
                        .meta
                        .as_mut()
                        .unwrap()
                        .resolution = 0.1
                }
                6 => checkpoint
                    .canonical_cloud_checkpoint
                    .as_mut()
                    .unwrap()
                    .xyz_f32_le[..4]
                    .copy_from_slice(&4_f32.to_le_bytes()),
                _ => {
                    checkpoint
                        .canonical_cloud_checkpoint
                        .as_mut()
                        .unwrap()
                        .voxel_size = 0.1
                }
            }
            assert!(
                admit(request, &interner)
                    .unwrap_err()
                    .contains("replay conflict"),
                "mutation {mutation}"
            );
        }
        assert_eq!(first.checkpoint().created_at_ns(), 101);
        assert_eq!(first.checkpoint().backend_payload()[20], 0);
        assert_eq!(interner.owners.lock().len(), 1);
    }

    #[test]
    fn assigned_v2_same_checkpoint_id_rejects_coherent_changed_member_receipts() {
        let interner = CheckpointOwnerInternerV2::default();
        let first = admit(wire_request(), &interner).unwrap();
        let mut request = wire_request();
        let checkpoint = request.mapping_backend_checkpoint.as_mut().unwrap();
        checkpoint
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .backend_generation += 1;
        checkpoint
            .backend_identity
            .as_mut()
            .unwrap()
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .backend_generation += 1;
        checkpoint
            .occupancy_identity
            .as_mut()
            .unwrap()
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .backend_generation += 1;
        checkpoint
            .canonical_cloud_checkpoint
            .as_mut()
            .unwrap()
            .member_identity
            .as_mut()
            .unwrap()
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .backend_generation += 1;
        // Every descriptor remains correctly bound; only the admitted immutable identity changed.
        crate::server::validate_assigned_mapping_checkpoint(checkpoint, MappingBackendKind::Voxel)
            .unwrap();
        assert!(admit(request, &interner)
            .unwrap_err()
            .contains("replay conflict"));
        assert_eq!(first.checkpoint().occupancy_receipt().backend_generation, 3);
    }

    #[test]
    fn assigned_v2_rejects_malformed_admission_before_any_artifact_pin() {
        let mut store = crate::planar_artifacts::Store::default();
        let mut good = wire_request();
        add_dense(&mut good, &mut store);
        let interner = CheckpointOwnerInternerV2::default();
        for mutation in 0..27 {
            let mut request = good.clone();
            match mutation {
                0 => request.activation_contract_version = 1,
                1 => request.expected_runtime_generation = 16,
                2 => request.expected_runtime_generation = 0,
                3 => request.activation_request_id = "bad request".into(),
                4 => request.session_id = " bad".into(),
                5 => request.map_name.clear(),
                6 => request.map_name = "Map\nOne".into(),
                7 => request.asset_identity = None,
                8 => request.asset_identity.as_mut().unwrap().asset_id = "0".repeat(32),
                9 => request.asset_manifest_sha256 = vec![1; 32],
                10 => request.asset_bundle_sha256 = vec![1; 32],
                11 => request.checkpoint = Some(proto::OccupancyGridData::default()),
                12 => request.cloud_checkpoint = Some(proto::CloudMapCheckpoint::default()),
                13 => request.mapping_backend_checkpoint = None,
                _ => {
                    let checkpoint = request.mapping_backend_checkpoint.as_mut().unwrap();
                    match mutation {
                        14 => checkpoint.schema_version = 1,
                        15 => checkpoint.content_sha256 = vec![1; 32],
                        16 => checkpoint.backend_kind = "raytracing".into(),
                        17 => checkpoint.source_session_id = "other".into(),
                        18 => checkpoint.backend_payload_schema_version = 2,
                        19 => checkpoint.payload[0] ^= 1,
                        20 => {
                            checkpoint.backend_identity.as_mut().unwrap().checkpoint_id =
                                "3".repeat(32)
                        }
                        21 => {
                            checkpoint.occupancy_identity.as_mut().unwrap().role =
                                CheckpointMemberRole::BackendState as i32
                        }
                        22 => {
                            checkpoint
                                .canonical_cloud_checkpoint
                                .as_mut()
                                .unwrap()
                                .frame_id = "odom".into()
                        }
                        23 => {
                            checkpoint
                                .canonical_cloud_checkpoint
                                .as_mut()
                                .unwrap()
                                .xyz_sha256 = "legacy".into()
                        }
                        24 => checkpoint
                            .canonical_cloud_checkpoint
                            .as_mut()
                            .unwrap()
                            .xyz_f32_le[..4]
                            .copy_from_slice(&f32::NAN.to_le_bytes()),
                        25 => {
                            checkpoint
                                .dense_cloud_artifact
                                .as_mut()
                                .unwrap()
                                .member_identity
                                .as_mut()
                                .unwrap()
                                .frame_id = "odom".into()
                        }
                        _ => {
                            checkpoint
                                .dense_cloud_artifact
                                .as_mut()
                                .unwrap()
                                .member_identity
                                .as_mut()
                                .unwrap()
                                .occupancy_receipt
                                .as_mut()
                                .unwrap()
                                .occupancy_revision += 1
                        }
                    }
                }
            }
            let mut pins = 0;
            let result = ValidatedActivationRequestV2::admit(
                request,
                17,
                MappingBackendKind::Voxel,
                "map",
                |reference| {
                    pins += 1;
                    store.open(reference)
                },
                &interner,
            );
            assert!(result.is_err(), "mutation {mutation}");
            assert_eq!(pins, 0, "mutation {mutation}");
            assert!(interner.owners.lock().is_empty());
        }
    }

    #[test]
    fn assigned_v2_selected_runtime_backend_and_frame_are_checked_before_pin() {
        for (runtime, backend, frame) in [
            (0, MappingBackendKind::Voxel, "map"),
            (18, MappingBackendKind::Voxel, "map"),
            (17, MappingBackendKind::Raytracing, "map"),
            (17, MappingBackendKind::SlamToolbox, "map"),
            (17, MappingBackendKind::Voxel, "odom"),
        ] {
            let result = ValidatedActivationRequestV2::admit(
                wire_request(),
                runtime,
                backend,
                frame,
                |_| panic!("invalid request pinned an artifact"),
                &CheckpointOwnerInternerV2::default(),
            );
            assert!(result.is_err());
        }
    }

    #[test]
    fn assigned_v2_weak_interner_and_metadata_release_all_source_ownership() {
        let interner = CheckpointOwnerInternerV2::default();
        let admitted = admit(wire_request(), &interner).unwrap();
        let weak = admitted.checkpoint().downgrade();
        let (metadata, owner) = admitted.into_parts();
        assert!(weak.upgrade().is_some());
        drop(owner);
        assert!(weak.upgrade().is_none());
        assert_eq!(
            metadata.checkpoint_identity(),
            CheckpointIdentityV2::parse(CHECKPOINT_ID).unwrap()
        );
        interner.prune_expired();
        assert!(interner.owners.lock().is_empty());
    }

    #[test]
    fn assigned_v2_next_admission_sweeps_dead_weak_entries() {
        let interner = CheckpointOwnerInternerV2::default();
        let admitted = admit(wire_request(), &interner).unwrap();
        let weak = admitted.checkpoint().downgrade();
        drop(admitted);
        assert!(weak.upgrade().is_none());
        let mut request = wire_request();
        let checkpoint = request.mapping_backend_checkpoint.as_mut().unwrap();
        checkpoint.checkpoint_id = "3".repeat(32);
        checkpoint.backend_identity.as_mut().unwrap().checkpoint_id =
            checkpoint.checkpoint_id.clone();
        checkpoint
            .occupancy_identity
            .as_mut()
            .unwrap()
            .checkpoint_id = checkpoint.checkpoint_id.clone();
        checkpoint
            .canonical_cloud_checkpoint
            .as_mut()
            .unwrap()
            .member_identity
            .as_mut()
            .unwrap()
            .checkpoint_id = checkpoint.checkpoint_id.clone();
        let next = admit(request, &interner).unwrap();
        assert_eq!(interner.owners.lock().len(), 1);
        assert_eq!(
            next.checkpoint().identity(),
            CheckpointIdentityV2::parse(&"3".repeat(32)).unwrap()
        );
    }

    #[test]
    fn assigned_v2_pinned_archive_outlives_transfer_release_and_is_read_only() {
        let mut store = crate::planar_artifacts::Store::default();
        let mut request = wire_request();
        add_dense(&mut request, &mut store);
        let reference = request
            .mapping_backend_checkpoint
            .as_ref()
            .unwrap()
            .dense_cloud_artifact
            .clone()
            .unwrap();
        let interner = CheckpointOwnerInternerV2::default();
        let admitted = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            MappingBackendKind::Voxel,
            "map",
            |reference| store.open(reference),
            &interner,
        )
        .unwrap();
        let owner = admitted.checkpoint().dense_cloud().unwrap();
        let mut magic = [0; 8];
        owner.file().read_exact_at(&mut magic, 0).unwrap();
        assert_eq!(&magic, b"NUWAXDC1");
        assert!(owner.file().write_at(b"x", 0).is_err());
        transfer(&mut store, &reference, 5, 0, Vec::new());
        assert!(store.open(&reference).is_err());
        owner.file().read_exact_at(&mut magic, 0).unwrap();
        assert_eq!(&magic, b"NUWAXDC1");
        admitted
            .checkpoint()
            .validate_replay(&admitted.checkpoint().clone())
            .unwrap();
        let replay = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            MappingBackendKind::Voxel,
            "map",
            |_| panic!("live replay must not consult an expired transfer"),
            &interner,
        )
        .unwrap();
        admitted.validate_replay(&replay).unwrap();
        assert!(admitted
            .checkpoint()
            .shares_source_with(replay.checkpoint()));
        request
            .mapping_backend_checkpoint
            .as_mut()
            .unwrap()
            .dense_cloud_artifact
            .as_mut()
            .unwrap()
            .id = "3".repeat(32);
        assert!(ValidatedActivationRequestV2::admit(
            request,
            17,
            MappingBackendKind::Voxel,
            "map",
            |_| panic!("changed descriptor must not select another transfer"),
            &interner,
        )
        .unwrap_err()
        .contains("descriptor replay conflict"));
    }

    #[test]
    fn assigned_v2_archive_reupload_under_same_reference_requires_exact_bytes() {
        let mut store = crate::planar_artifacts::Store::default();
        let mut request = wire_request();
        add_dense(&mut request, &mut store);
        let reference = request
            .mapping_backend_checkpoint
            .as_ref()
            .unwrap()
            .dense_cloud_artifact
            .clone()
            .unwrap();
        let interner = CheckpointOwnerInternerV2::default();
        let first = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            MappingBackendKind::Voxel,
            "map",
            |reference| store.open(reference),
            &interner,
        )
        .unwrap();
        let duplicate = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            MappingBackendKind::Voxel,
            "map",
            |reference| store.open(reference),
            &interner,
        )
        .unwrap();
        assert!(first
            .checkpoint()
            .shares_source_with(duplicate.checkpoint()));
        let mut file = store.open(&reference).unwrap();
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).unwrap();
        // Preserve the archive's source metadata and exact bounds while changing its body.
        let points = bytes.len() - 24;
        bytes[points..].rotate_left(12);
        transfer(&mut store, &reference, 5, 0, Vec::new());
        transfer(&mut store, &reference, 1, 0, Vec::new());
        transfer(&mut store, &reference, 2, 0, bytes);
        transfer(&mut store, &reference, 3, 0, Vec::new());
        // Same-reference requests name the original immutable object, not a recycled lease.
        let replay = ValidatedActivationRequestV2::admit(
            request.clone(),
            17,
            MappingBackendKind::Voxel,
            "map",
            |_| panic!("live replay must not consult a recycled transfer"),
            &interner,
        )
        .unwrap();
        assert!(first.checkpoint().shares_source_with(replay.checkpoint()));
        // A separately admitted file really presents a body and must be compared byte-for-byte.
        let independent = ValidatedActivationRequestV2::admit(
            request,
            17,
            MappingBackendKind::Voxel,
            "map",
            |reference| store.open(reference),
            &CheckpointOwnerInternerV2::default(),
        )
        .unwrap();
        assert!(interner
            .intern(independent.checkpoint().clone())
            .unwrap_err()
            .contains("replay conflict"));
        assert!(first
            .validate_replay(&independent)
            .unwrap_err()
            .contains("replay conflict"));
    }

    #[test]
    fn assigned_v2_unsealed_archive_never_becomes_an_interned_owner() {
        let mut store = crate::planar_artifacts::Store::default();
        let mut request = wire_request();
        add_dense(&mut request, &mut store);
        let reference = request
            .mapping_backend_checkpoint
            .as_ref()
            .unwrap()
            .dense_cloud_artifact
            .clone()
            .unwrap();
        transfer(&mut store, &reference, 5, 0, Vec::new());
        transfer(&mut store, &reference, 1, 0, Vec::new());
        let interner = CheckpointOwnerInternerV2::default();
        assert!(ValidatedActivationRequestV2::admit(
            request,
            17,
            MappingBackendKind::Voxel,
            "map",
            |reference| store.open(reference),
            &interner,
        )
        .is_err());
        assert!(interner.owners.lock().is_empty());
    }

    #[test]
    fn assigned_v2_file_comparison_crosses_transfer_chunks_without_moving_cursors() {
        let bytes = vec![7; nuwax_slam_2d_runtime::archive::CHUNK_BYTES + 19];
        let mut left = raw_file(&bytes);
        let mut right = raw_file(&bytes);
        left.seek(SeekFrom::Start(3)).unwrap();
        right.seek(SeekFrom::Start(11)).unwrap();
        assert!(exact_file_bytes(&left, &right, bytes.len() as u64).unwrap());
        assert_eq!(left.stream_position().unwrap(), 3);
        assert_eq!(right.stream_position().unwrap(), 11);
        right.write_all_at(&[8], bytes.len() as u64 - 1).unwrap();
        assert!(!exact_file_bytes(&left, &right, bytes.len() as u64).unwrap());
        assert!(exact_file_bytes(&left, &right, bytes.len() as u64 + 1).is_err());
    }

    #[test]
    fn assigned_v2_concurrent_first_admissions_share_the_winning_source() {
        let interner = Arc::new(CheckpointOwnerInternerV2::default());
        let barrier = Arc::new(std::sync::Barrier::new(3));
        let mut workers = Vec::new();
        for _ in 0..2 {
            let interner = interner.clone();
            let barrier = barrier.clone();
            workers.push(std::thread::spawn(move || {
                barrier.wait();
                admit(wire_request(), &interner).unwrap()
            }));
        }
        barrier.wait();
        let right = workers.pop().unwrap().join().unwrap();
        let left = workers.pop().unwrap().join().unwrap();
        assert!(left.checkpoint().shares_source_with(right.checkpoint()));
        assert_eq!(interner.owners.lock().len(), 1);
    }

    #[test]
    fn assigned_v2_concurrent_conflicting_first_admissions_do_not_replace_winner() {
        let interner = Arc::new(CheckpointOwnerInternerV2::default());
        let barrier = Arc::new(std::sync::Barrier::new(3));
        let mut workers = Vec::new();
        for value in [1, 2] {
            let interner = interner.clone();
            let barrier = barrier.clone();
            workers.push(std::thread::spawn(move || {
                let mut request = wire_request();
                request.mapping_backend_checkpoint.as_mut().unwrap().payload[20] = value;
                barrier.wait();
                admit(request, &interner)
            }));
        }
        barrier.wait();
        let right = workers.pop().unwrap().join().unwrap();
        let left = workers.pop().unwrap().join().unwrap();
        assert_ne!(left.is_ok(), right.is_ok());
        let winner = left.or(right).unwrap();
        assert_eq!(interner.owners.lock().len(), 1);
        assert!(matches!(winner.checkpoint().backend_payload()[20], 1 | 2));
    }
}

#[cfg(test)]
pub(crate) mod planar_test_support {
    use super::*;
    use nuwax_slam_2d_runtime::{archive, protocol};

    /// Real NWS2D002 producer and sealed Store; no legacy writer or digest fixture.
    pub(crate) fn fixture() -> (proto::ActivateMapRequest, crate::planar_artifacts::Store, archive::AssignedMetadata) {
        let checkpoint_id = "11111111111111111111111111111111";
        let directory = std::env::temp_dir().join(format!("assigned-planar-owner-{}", uuid::Uuid::new_v4().simple()));
        std::fs::create_dir(&directory).unwrap();
        let names = ["graph.posegraph", "graph.data", "grid.i8"];
        for (name, bytes) in names.into_iter().zip([b"graph".as_slice(), b"scans".as_slice(), &[0]]) {
            std::fs::write(directory.join(name), bytes).unwrap();
        }
        let metadata = archive::AssignedMetadata {
            checkpoint_id: checkpoint_id.into(),
            source: archive::Metadata {
                version: 2,
                session: protocol::Session {
                    session_id: "source".into(), map_name: "Map One".into(), session_generation: 2,
                    sensor_id: "laser".into(), frame_id: "laser".into(), map_frame_id: "map".into(),
                    odom_frame_id: "odom".into(), body_frame_id: "base".into(), clock_domain: "unix_ns".into(),
                    clock_id: String::new(), calibration_ref: "calibration".into(),
                    odometry_source: protocol::OdometrySource::Device,
                    core_config: serde_json::json!({"base_from_laser": [0.0, 0.0, 0.0]}),
                },
                estimate: protocol::Estimate { graph_revision: 11, source_timestamp_ns: 100, ..Default::default() },
                source_unix_timestamp_ns: 100, width: 1, height: 1, resolution: 0.05, origin: [0.0, 0.0],
                members: [
                    archive::AssignedMember { name: names[0].into(), size: 5 },
                    archive::AssignedMember { name: names[1].into(), size: 5 },
                    archive::AssignedMember { name: names[2].into(), size: 1 },
                ],
            },
        };
        let archive_path = directory.join("checkpoint.nws");
        let paths = names.map(|name| directory.join(name));
        let descriptor = archive::write_assigned(&archive_path, &metadata, paths.each_ref().map(|path| path.as_path())).unwrap();
        let occupancy = proto::CanonicalOccupancyCommitReceipt {
            commit_id: Some(proto::MappingCommitIdentity { boot_nonce: vec![1; 16], sequence: 5 }),
            session_id: "source".into(), session_generation: 2, occupancy_revision: 7,
            source_timestamp_ns: 100, backend_kind: "slam_toolbox".into(), backend_generation: 3,
        };
        let member = |role| proto::CheckpointMemberIdentity {
            checkpoint_id: checkpoint_id.into(), role: role as i32, occupancy_receipt: Some(occupancy.clone()),
            cloud_receipt: None, frame_id: "map".into(), frame_generation: 0, producer_generation: 0,
        };
        let mut store = crate::planar_artifacts::Store::default();
        let reference = store.insert(File::open(&archive_path).unwrap(), descriptor.payload_size,
            member(proto::CheckpointMemberRole::PlanarGraph)).unwrap();
        std::fs::remove_dir_all(directory).unwrap();
        let checkpoint = proto::MappingBackendCheckpoint {
            schema_version: 2, backend_kind: "slam_toolbox".into(), source_session_generation: 2,
            created_at_ns: 101, source_session_id: "source".into(), payload_size: descriptor.payload_size,
            checkpoint_id: checkpoint_id.into(), backend_payload_schema_version: 2,
            occupancy_identity: Some(member(proto::CheckpointMemberRole::CanonicalOccupancy)),
            occupancy_receipt: Some(occupancy), graph_artifact: Some(reference), target_kind: "planar_graph".into(),
            canonical_occupancy_checkpoint: Some(proto::OccupancyGridData {
                meta: Some(proto::OccupancyGridMeta { width: 1, height: 1, resolution: 0.05, origin_x: 0.0,
                    origin_y: 0.0, frame_id: "map".into(), revision: 7, timestamp_ns: 100 }), data: vec![0],
            }),
            ..Default::default()
        };
        (proto::ActivateMapRequest {
            session_id: "destination".into(), map_name: "Map One".into(), activation_request_id: "activate-planar".into(),
            expected_runtime_generation: 17, activation_contract_version: 2,
            asset_identity: Some(proto::AssetIdentityV2 { asset_id: "22222222222222222222222222222222".into() }),
            mapping_backend_checkpoint: Some(checkpoint), ..Default::default()
        }, store, metadata)
    }

    pub(crate) fn admit(wire: proto::ActivateMapRequest, store: &mut crate::planar_artifacts::Store,
        interner: &CheckpointOwnerInternerV2) -> ValidatedActivationRequestV2 {
        ValidatedActivationRequestV2::admit(wire, 17, MappingBackendKind::SlamToolbox, "map",
            |reference| store.open(reference), interner).unwrap()
    }

    #[test]
    fn assigned_planar_owner_pins_single_graph_and_reuses_it_after_release() {
        use std::os::unix::fs::FileExt;
        let (wire, mut store, _) = fixture();
        let replay_wire = wire.clone();
        let checkpoint = wire.mapping_backend_checkpoint.as_ref().unwrap();
        let grid_pointer = checkpoint.canonical_occupancy_checkpoint.as_ref().unwrap().data.as_ptr();
        let reference = checkpoint.graph_artifact.clone().unwrap();
        let interner = CheckpointOwnerInternerV2::default();
        let admitted = admit(wire, &mut store, &interner);
        let owner = admitted.checkpoint();
        assert_eq!(owner.occupancy().data.as_ptr(), grid_pointer);
        assert!(owner.backend_payload().is_empty());
        assert!(owner.0.cloud.is_none());
        assert!(owner.0.backend_member.is_none());
        assert_eq!(owner.source_member().role(), proto::CheckpointMemberRole::PlanarGraph);
        assert_eq!(owner.occupancy_receipt().backend_kind, MappingBackendKind::SlamToolbox);
        store.transfer(proto::GraphArtifactTransferRequest { artifact: Some(reference), action: 5, ..Default::default() }).unwrap();
        let replay = ValidatedActivationRequestV2::admit(replay_wire, 17, MappingBackendKind::SlamToolbox, "map",
            |_| panic!("retained graph replay must not reopen the transfer namespace"), &interner).unwrap();
        assert!(owner.shares_source_with(replay.checkpoint()));
        let mut magic = [0; 8];
        owner.graph().unwrap().file().read_exact_at(&mut magic, 0).unwrap();
        assert_eq!(&magic, b"NWS2D002");
        assert!(owner.graph().unwrap().file().write_at(b"x", 0).is_err());
    }

    #[test]
    fn assigned_planar_owner_descriptor_conflict_is_rejected_before_repinning() {
        let (wire, mut store, _) = fixture();
        let interner = CheckpointOwnerInternerV2::default();
        let original = admit(wire.clone(), &mut store, &interner);
        let mut changed = wire;
        changed.mapping_backend_checkpoint.as_mut().unwrap().graph_artifact.as_mut().unwrap().id = "33333333333333333333333333333333".into();
        assert!(ValidatedActivationRequestV2::admit(changed, 17, MappingBackendKind::SlamToolbox, "map",
            |_| panic!("a live owner descriptor conflict must precede pinning"), &interner).is_err());
        assert_eq!(original.checkpoint().source_member().checkpoint_id(), original.checkpoint().identity());
    }

    #[test]
    fn assigned_planar_target_receipt_uses_graph_and_no_cloud() {
        use crate::activation_contract::{InstalledActivationIdentityV2, InstalledTargetReceiptV2};
        let (wire, mut store, _) = fixture();
        let admitted = admit(wire, &mut store, &CheckpointOwnerInternerV2::default());
        let owner = admitted.checkpoint();
        let activation = InstalledActivationIdentityV2 {
            request: admitted.metadata().clone(), runtime_generation: 17, lifecycle_epoch: 1,
            session_generation: 23, backend_kind: MappingBackendKind::SlamToolbox, backend_generation: 19,
        };
        let receipt = InstalledTargetReceiptV2 {
            activation, source_member: owner.source_member().clone(), cloud_receipt: None,
            occupancy_receipt: OccupancyCommitReceipt {
                commit_id: crate::mapping_commit::MappingCommitId { boot_nonce: 31, sequence: 32 },
                session_id: "destination".into(), session_generation: 23, occupancy_revision: 33,
                source_timestamp_ns: 34, backend_kind: MappingBackendKind::SlamToolbox, backend_generation: 19,
            },
        };
        assert!(receipt.validate(owner));
        let wire = receipt.to_wire();
        assert!(wire.source_cloud_member.is_none());
        assert!(wire.cloud_receipt.is_none());
        assert_eq!(wire.source_graph_member.unwrap().role, proto::CheckpointMemberRole::PlanarGraph as i32);
        let mut changed = receipt.clone();
        changed.occupancy_receipt.backend_generation += 1;
        assert!(!changed.validate(owner));
        changed = receipt.clone();
        changed.source_member = owner.occupancy_member().clone();
        assert!(!changed.validate(owner));
        changed = receipt;
        changed.cloud_receipt = Some(CloudCommitReceipt {
            commit_id: changed.occupancy_receipt.commit_id, session_generation: 23,
            occupancy_revision: 33, cloud_revision: 35, source_timestamp_ns: 34, point_count: 1,
        });
        assert!(!changed.validate(owner));
    }
}
