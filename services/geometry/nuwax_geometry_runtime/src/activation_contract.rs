//! 激活事务共享既有分配身份和规范提交收据。

use crate::activation_identity::CheckpointMemberIdentityV2;
use crate::activation_request::ActivationRequestMetadataV2;
use crate::mapping_backend::MappingBackendKind;
use crate::mapping_commit::{CloudCommitReceipt, MappingCommitId, OccupancyCommitReceipt};
use crate::proto;

pub(crate) type ActivationRequestKey = ActivationRequestMetadataV2;
pub(crate) type ActivationIdentity = InstalledActivationIdentityV2;
pub(crate) type TargetReceipt = InstalledTargetReceiptV2;

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
pub(crate) struct InstalledTargetReceiptV2 {
    pub(crate) activation: InstalledActivationIdentityV2,
    pub(crate) source_member: CheckpointMemberIdentityV2,
    pub(crate) occupancy_receipt: OccupancyCommitReceipt,
    pub(crate) cloud_receipt: Option<CloudCommitReceipt>,
}
impl InstalledTargetReceiptV2 {
    pub(crate) fn to_wire(&self) -> proto::InstalledTargetReceiptV2 {
        proto::InstalledTargetReceiptV2 {
            activation: Some(self.activation.to_wire()),
            source_cloud_member: (self.activation.backend_kind != MappingBackendKind::SlamToolbox)
                .then(|| member_to_wire(&self.source_member)),
            source_graph_member: (self.activation.backend_kind == MappingBackendKind::SlamToolbox)
                .then(|| member_to_wire(&self.source_member)),
            occupancy_receipt: Some(occupancy_to_wire(&self.occupancy_receipt)),
            cloud_receipt: self.cloud_receipt.as_ref().map(cloud_to_wire),
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
