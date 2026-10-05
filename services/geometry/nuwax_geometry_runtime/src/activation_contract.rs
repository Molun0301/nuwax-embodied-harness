//! Discriminated activation identities shared by the existing activation transaction.
//!
//! Assigned identities retain only validated metadata and existing generation/commit receipts.
//! No conversion maps them into a legacy digest. Payload ownership remains outside these types.
#![cfg_attr(not(test), allow(dead_code))]

use crate::activation_identity::CheckpointMemberIdentityV2;
use crate::activation_request::{ActivationRequestMetadataV2, CanonicalCheckpointOwnerV2};
use crate::mapping_backend::MappingBackendKind;
use crate::mapping_commit::{CloudCommitReceipt, MappingCommitId, OccupancyCommitReceipt};
use crate::proto;

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ActivationRequestKey {
    LegacyDigest([u8; 32]),
    Assigned(ActivationRequestMetadataV2),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct InstalledActivationIdentityV2 {
    pub(crate) request: ActivationRequestMetadataV2,
    pub(crate) runtime_generation: u64,
    pub(crate) lifecycle_epoch: u64,
    pub(crate) session_generation: u64,
    pub(crate) backend_kind: MappingBackendKind,
    pub(crate) backend_generation: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ActivationIdentity {
    LegacyDigest([u8; 32]),
    Assigned(InstalledActivationIdentityV2),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct InstalledTargetReceiptV2 {
    pub(crate) activation: InstalledActivationIdentityV2,
    /// Frozen source descriptor; never replaced with an installed receipt.
    pub(crate) source_cloud_member: CheckpointMemberIdentityV2,
    pub(crate) occupancy_receipt: OccupancyCommitReceipt,
    pub(crate) cloud_receipt: CloudCommitReceipt,
}

impl InstalledTargetReceiptV2 {
    pub(crate) fn validate(&self, owner: &CanonicalCheckpointOwnerV2) -> bool {
        let activation = &self.activation;
        let occupancy = &self.occupancy_receipt;
        let cloud = &self.cloud_receipt;
        activation.runtime_generation > 0
            && activation.runtime_generation == activation.request.expected_runtime_generation()
            && activation.lifecycle_epoch > 0
            && activation.session_generation > 0
            && activation.backend_generation > 0
            && activation.backend_kind == owner.backend_kind()
            && activation.request.checkpoint_identity() == owner.identity()
            && self.source_cloud_member == *owner.cloud().member()
            && occupancy.session_id == activation.request.session_id()
            && occupancy.session_generation == activation.session_generation
            && occupancy.backend_kind == activation.backend_kind
            && occupancy.backend_generation == activation.backend_generation
            && occupancy.commit_id.boot_nonce != 0
            && occupancy.commit_id.sequence > 0
            && occupancy.occupancy_revision > 0
            && occupancy.source_timestamp_ns > 0
            && cloud.commit_id == occupancy.commit_id
            && cloud.session_generation == occupancy.session_generation
            && cloud.occupancy_revision == occupancy.occupancy_revision
            && cloud.source_timestamp_ns == occupancy.source_timestamp_ns
            && cloud.cloud_revision > 0
            && cloud.point_count == owner.cloud().point_count()
    }

    pub(crate) fn to_wire(&self) -> proto::InstalledTargetReceiptV2 {
        proto::InstalledTargetReceiptV2 {
            activation: Some(self.activation.to_wire()),
            source_cloud_member: Some(member_to_wire(&self.source_cloud_member)),
            occupancy_receipt: Some(occupancy_to_wire(&self.occupancy_receipt)),
            cloud_receipt: Some(cloud_to_wire(&self.cloud_receipt)),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum TargetReceipt {
    LegacyDigest([u8; 32]),
    Assigned(InstalledTargetReceiptV2),
}

macro_rules! legacy_digest {
    ($name:ident) => {
        impl $name {
            pub(crate) fn legacy_digest(&self) -> Option<[u8; 32]> {
                match self {
                    Self::LegacyDigest(value) => Some(*value),
                    Self::Assigned(_) => None,
                }
            }
        }
        impl From<[u8; 32]> for $name {
            fn from(value: [u8; 32]) -> Self {
                Self::LegacyDigest(value)
            }
        }
    };
}
legacy_digest!(ActivationRequestKey);
legacy_digest!(ActivationIdentity);
legacy_digest!(TargetReceipt);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum ActivationJournalStep {
    BackendRestoreSubmission,
    SessionStart,
    OccupancyShmBegin,
    OccupancyInstall,
    CloudInstall,
    BackendCommit,
    ActivationRecord,
    SharedStateCommit,
    CacheClears,
    MapEvent,
    CloudEvent,
    TypedGlobalMap,
    RetirementPrepared,
    Seal,
    TargetReceiptVerification,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ActivationJournalKey {
    LegacyDigest([u8; 32]),
    Assigned {
        request: ActivationRequestMetadataV2,
        step: ActivationJournalStep,
    },
}

impl From<[u8; 32]> for ActivationJournalKey {
    fn from(value: [u8; 32]) -> Self {
        Self::LegacyDigest(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ActivationJournalVerifierV2 {
    BackendGeneration(u64),
    SessionGeneration(u64),
    Occupancy(OccupancyCommitReceipt),
    Cloud(CloudCommitReceipt),
    Activation(InstalledActivationIdentityV2),
    Target(InstalledTargetReceiptV2),
    CompletedStep(ActivationJournalKey),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ActivationJournalVerifier {
    LegacyDigest([u8; 32]),
    Assigned(ActivationJournalVerifierV2),
}

impl From<[u8; 32]> for ActivationJournalVerifier {
    fn from(value: [u8; 32]) -> Self {
        Self::LegacyDigest(value)
    }
}

/// Execution-only read access. The assigned variant shares the canonical owner; it cannot
/// be converted to a legacy snapshot or copied into a digest-bearing envelope.
#[derive(Clone, Debug)]
pub(crate) enum ActivationTargetSource {
    Legacy(std::sync::Arc<crate::pointcloud::ImmutableActivatedTarget>),
    Assigned(CanonicalCheckpointOwnerV2),
}

impl ActivationTargetSource {
    pub(crate) fn legacy_digest(&self) -> Option<[u8; 32]> {
        match self {
            Self::Legacy(value) => Some(value.xyz_sha256),
            Self::Assigned(_) => None,
        }
    }
    pub(crate) fn frame_id(&self) -> &str {
        match self {
            Self::Legacy(value) => &value.frame_id,
            Self::Assigned(value) => value.cloud().member().frame_id(),
        }
    }
    pub(crate) fn source_revision(&self) -> u64 {
        match self {
            Self::Legacy(value) => value.source_revision,
            Self::Assigned(value) => value.cloud().source_revision(),
        }
    }
    pub(crate) fn point_count(&self) -> u32 {
        match self {
            Self::Legacy(value) => value.point_count,
            Self::Assigned(value) => value.cloud().point_count(),
        }
    }
    pub(crate) fn voxel_size(&self) -> f32 {
        match self {
            Self::Legacy(value) => value.voxel_size,
            Self::Assigned(value) => value.cloud().voxel_size(),
        }
    }
    pub(crate) fn xyz_f32_le(&self) -> &[u8] {
        match self {
            Self::Legacy(value) => &value.xyz_f32_le,
            Self::Assigned(value) => value.cloud().xyz_f32_le(),
        }
    }
}

pub(crate) fn request_to_wire(
    value: &ActivationRequestMetadataV2,
) -> proto::ActivationRequestMetadataV2 {
    proto::ActivationRequestMetadataV2 {
        activation_request_id: value.activation_request_id().into(),
        session_id: value.session_id().into(),
        map_name: value.map_name().into(),
        asset_identity: Some(value.asset_identity().to_wire()),
        checkpoint_id: value.checkpoint_identity().to_wire_id(),
        expected_runtime_generation: value.expected_runtime_generation(),
    }
}

impl InstalledActivationIdentityV2 {
    pub(crate) fn to_wire(&self) -> proto::InstalledActivationIdentityV2 {
        proto::InstalledActivationIdentityV2 {
            request: Some(request_to_wire(&self.request)),
            runtime_generation: self.runtime_generation,
            lifecycle_epoch: self.lifecycle_epoch,
            session_generation: self.session_generation,
            backend_kind: self.backend_kind.stable_name().into(),
            backend_generation: self.backend_generation,
        }
    }
}

fn commit_to_wire(value: MappingCommitId) -> proto::MappingCommitIdentity {
    proto::MappingCommitIdentity {
        boot_nonce: value.boot_nonce.to_be_bytes().to_vec(),
        sequence: value.sequence,
    }
}

pub(crate) fn occupancy_to_wire(
    value: &OccupancyCommitReceipt,
) -> proto::CanonicalOccupancyCommitReceipt {
    proto::CanonicalOccupancyCommitReceipt {
        commit_id: Some(commit_to_wire(value.commit_id)),
        session_id: value.session_id.clone(),
        session_generation: value.session_generation,
        occupancy_revision: value.occupancy_revision,
        source_timestamp_ns: value.source_timestamp_ns,
        backend_kind: value.backend_kind.stable_name().into(),
        backend_generation: value.backend_generation,
    }
}

pub(crate) fn cloud_to_wire(value: &CloudCommitReceipt) -> proto::CanonicalCloudCommitReceipt {
    proto::CanonicalCloudCommitReceipt {
        commit_id: Some(commit_to_wire(value.commit_id)),
        session_generation: value.session_generation,
        occupancy_revision: value.occupancy_revision,
        cloud_revision: value.cloud_revision,
        source_timestamp_ns: value.source_timestamp_ns,
        point_count: value.point_count,
    }
}

pub(crate) fn member_to_wire(
    value: &CheckpointMemberIdentityV2,
) -> proto::CheckpointMemberIdentity {
    proto::CheckpointMemberIdentity {
        checkpoint_id: value.checkpoint_id().to_wire_id(),
        role: value.role() as i32,
        occupancy_receipt: Some(occupancy_to_wire(value.occupancy_receipt())),
        cloud_receipt: value.cloud_receipt().map(cloud_to_wire),
        frame_id: value.frame_id().into(),
        frame_generation: value.frame_generation(),
        producer_generation: value.producer_generation(),
    }
}

impl ActivationRequestKey {
    pub(crate) fn assigned(&self) -> Option<&ActivationRequestMetadataV2> {
        match self {
            Self::Assigned(value) => Some(value),
            Self::LegacyDigest(_) => None,
        }
    }
}

impl ActivationIdentity {
    pub(crate) fn assigned(&self) -> Option<&InstalledActivationIdentityV2> {
        match self {
            Self::Assigned(value) => Some(value),
            Self::LegacyDigest(_) => None,
        }
    }
}

impl TargetReceipt {
    pub(crate) fn assigned(&self) -> Option<&InstalledTargetReceiptV2> {
        match self {
            Self::Assigned(value) => Some(value),
            Self::LegacyDigest(_) => None,
        }
    }
}
