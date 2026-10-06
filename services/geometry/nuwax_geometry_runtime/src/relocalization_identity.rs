//! Exact assigned relocalization metadata for the paired production actor.
//! This module never owns point/cloud payloads, allocates request IDs, or hashes content.
#![cfg_attr(not(test), allow(dead_code))]

use std::sync::{Arc, OnceLock};
use crate::activation_contract::{InstalledActivationIdentityV2, InstalledTargetReceiptV2};
use crate::activation_identity::{CheckpointMemberIdentityV2, cloud_receipt_from_wire, occupancy_receipt_from_wire};
use crate::activation_request::ActivationRequestMetadataV2;
use crate::mapping_backend::MappingBackendKind;
use crate::proto;

/// Validate the assigned boundary before ingress lookup or any legacy hashing.
pub(crate) fn validate_live_request_boundary(request: &proto::RelocalizeRequest) -> Result<(), String> {
    if request.schema_version == 2 {
        return AssignedCloudRelocalizeRequestIdentityV2::admit(request.clone()).map(|_| ());
    }
    if request.schema_version != 1 {
        return Err("RELOCALIZATION_SCHEMA_VERSION_UNSUPPORTED".into());
    }
    if request.assigned_target.is_some() {
        return Err("RELOCALIZATION_ASSIGNED_FIELDS_MIXED_WITH_LEGACY".into());
    }
    Ok(())
}

#[derive(Clone, Debug)]
pub(crate) struct CanonicalRelocalizationPolicyV2(Arc<proto::RelocalizationPolicyDescriptorV2>);

impl PartialEq for CanonicalRelocalizationPolicyV2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0) || self.0 == other.0
    }
}
impl Eq for CanonicalRelocalizationPolicyV2 {}

impl CanonicalRelocalizationPolicyV2 {
    pub(crate) fn current_cloud() -> Self {
        static CURRENT: OnceLock<CanonicalRelocalizationPolicyV2> = OnceLock::new();
        CURRENT.get_or_init(|| Self(Arc::new(proto::RelocalizationPolicyDescriptorV2 {
            schema_version: 2,
            source_policy_version: crate::correction_coordinator::RELOCALIZATION_SOURCE_POLICY_VERSION,
            source_parameters: crate::correction_coordinator::relocalization_source_policy_preimage_v1(),
            deadline_policy_version: crate::correction_coordinator::RELOCALIZATION_DEADLINE_POLICY_VERSION_V1,
            deadline_parameters: crate::correction_coordinator::relocalization_deadline_policy_preimage_v1(),
            algorithm_options_version: crate::correction_coordinator::RELOCALIZATION_ALGORITHM_OPTIONS_VERSION,
            algorithm_parameters: crate::correction_coordinator::relocalization_algorithm_options_preimage_v1(),
        }))).clone()
    }

    fn admit(value: proto::RelocalizationPolicyDescriptorV2) -> Result<Self, String> {
        let current = Self::current_cloud();
        if value != *current.0 {
            return Err("RELOCALIZATION_EXACT_POLICY_PARAMETERS_MISMATCH".into());
        }
        // Drop redundant incoming parameter buffers; all admitted requests share
        // the one existing production policy descriptor, not a new policy clock.
        Ok(current)
    }

    pub(crate) fn to_wire(&self) -> proto::RelocalizationPolicyDescriptorV2 {
        self.0.as_ref().clone()
    }
}

pub(crate) fn activation_from_wire(value: proto::InstalledActivationIdentityV2) -> Result<InstalledActivationIdentityV2, String> {
    let request = ActivationRequestMetadataV2::from_receipt_wire(value.request.ok_or("assigned activation request is absent")?)?;
    let backend_kind = match value.backend_kind.as_str() {
        "voxel" => MappingBackendKind::Voxel,
        "raytracing" => MappingBackendKind::Raytracing,
        _ => return Err("ASSIGNED_CLOUD_RELOCALIZATION_BACKEND_UNSUPPORTED".into()),
    };
    if value.runtime_generation == 0 || value.runtime_generation != request.expected_runtime_generation()
        || value.lifecycle_epoch == 0 || value.session_generation == 0 || value.backend_generation == 0 {
        return Err("assigned activation generations differ from their request".into());
    }
    Ok(InstalledActivationIdentityV2 { request, runtime_generation: value.runtime_generation,
        lifecycle_epoch: value.lifecycle_epoch, session_generation: value.session_generation,
        backend_kind, backend_generation: value.backend_generation })
}

pub(crate) fn target_from_wire(value: proto::InstalledTargetReceiptV2) -> Result<InstalledTargetReceiptV2, String> {
    let activation = activation_from_wire(value.activation.ok_or("assigned activation identity is absent")?)?;
    let source_cloud_member = CheckpointMemberIdentityV2::try_from(value.source_cloud_member.ok_or("source cloud member is absent")?)?;
    let occupancy_receipt = occupancy_receipt_from_wire(value.occupancy_receipt.as_ref().ok_or("installed occupancy receipt is absent")?)?;
    let cloud_receipt = cloud_receipt_from_wire(value.cloud_receipt.as_ref().ok_or("installed cloud receipt is absent")?)?;
    let source_cloud = source_cloud_member.cloud_receipt().ok_or("source cloud receipt is absent")?;
    if source_cloud_member.role() != proto::CheckpointMemberRole::CanonicalCloud
        || source_cloud_member.checkpoint_id() != activation.request.checkpoint_identity()
        || source_cloud_member.occupancy_receipt().backend_kind != activation.backend_kind
        || occupancy_receipt.session_id != activation.request.session_id()
        || occupancy_receipt.session_generation != activation.session_generation
        || occupancy_receipt.backend_kind != activation.backend_kind
        || occupancy_receipt.backend_generation != activation.backend_generation
        || cloud_receipt.commit_id != occupancy_receipt.commit_id
        || cloud_receipt.session_generation != occupancy_receipt.session_generation
        || cloud_receipt.occupancy_revision != occupancy_receipt.occupancy_revision
        || cloud_receipt.source_timestamp_ns != occupancy_receipt.source_timestamp_ns
        || cloud_receipt.point_count != source_cloud.point_count {
        return Err("assigned relocalization target receipt binding differs".into());
    }
    Ok(InstalledTargetReceiptV2 { activation, source_cloud_member, occupancy_receipt, cloud_receipt })
}

#[derive(Debug, PartialEq, Eq)]
struct AssignedCloudRelocalizeRequestData {
    request_id: String,
    target: InstalledTargetReceiptV2,
    policy: CanonicalRelocalizationPolicyV2,
    logical_timeout_ms: u64,
}

/// Immutable small request comparison material. Payload ownership belongs to the
/// already sealed target snapshot and forthcoming admitted source owner.
#[derive(Clone, Debug)]
pub(crate) struct AssignedCloudRelocalizeRequestIdentityV2(Arc<AssignedCloudRelocalizeRequestData>);

impl PartialEq for AssignedCloudRelocalizeRequestIdentityV2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0) || self.0 == other.0
    }
}
impl Eq for AssignedCloudRelocalizeRequestIdentityV2 {}

impl AssignedCloudRelocalizeRequestIdentityV2 {
    /// Durable identities must already use the canonical normalized descriptor.
    /// Public request/query admission still accepts the defined zero timeout.
    pub(crate) fn admit_canonical(request: proto::RelocalizeRequest) -> Result<Self, String> {
        use prost::Message;
        let encoded = request.encode_to_vec();
        let identity = Self::admit(request)?;
        if identity.to_wire().encode_to_vec() != encoded {
            return Err("RELOCALIZATION_ASSIGNED_REQUEST_NON_CANONICAL".into());
        }
        Ok(identity)
    }

    pub(crate) fn admit(request: proto::RelocalizeRequest) -> Result<Self, String> {
        if request.schema_version != 2 {
            return Err("RELOCALIZATION_ASSIGNED_SCHEMA_UNSUPPORTED".into());
        }
        crate::correction_coordinator::validate_relocalization_request_id(&request.request_id)?;
        if request.runtime_generation != 0 || !request.session_id.is_empty() || !request.map_name.is_empty()
            || request.session_generation != 0 || request.backend_generation != 0
            || !request.activation_identity_sha256.is_empty() || !request.target_receipt_sha256.is_empty()
            || request.target_cloud_revision != 0 || !request.target_cloud_sha256.is_empty()
            || !request.asset_manifest_sha256.is_empty() || !request.asset_bundle_sha256.is_empty()
            || request.source_policy_version != 0 || !request.source_policy_sha256.is_empty()
            || request.deadline_policy_version != 0 || !request.deadline_policy_sha256.is_empty()
            || request.logical_timeout_ms != 0 || request.algorithm_options_version != 0
            || !request.algorithm_options_sha256.is_empty() || !request.target_kind.is_empty()
            || !request.target_graph_sha256.is_empty() || request.planar_seed.is_some() {
            return Err("RELOCALIZATION_ASSIGNED_FIELDS_MIXED_WITH_LEGACY".into());
        }
        let Some(proto::relocalize_request::AssignedTarget::AssignedCloudV2(body)) = request.assigned_target else {
            return Err("RELOCALIZATION_ASSIGNED_TARGET_ABSENT_OR_UNSUPPORTED".into());
        };
        let logical_timeout_ms = crate::correction_coordinator::normalize_relocalization_logical_timeout_ms(body.logical_timeout_ms)?;
        let policy = CanonicalRelocalizationPolicyV2::admit(body.policy.ok_or("assigned policy parameters are absent")?)?;
        let target = target_from_wire(body.target.ok_or("assigned target receipt is absent")?)?;
        Ok(Self(Arc::new(AssignedCloudRelocalizeRequestData { request_id: request.request_id,
            target, policy, logical_timeout_ms })))
    }

    pub(crate) fn request_id(&self) -> &str { &self.0.request_id }
    pub(crate) fn target(&self) -> &InstalledTargetReceiptV2 { &self.0.target }
    pub(crate) fn logical_timeout_ms(&self) -> u64 { self.0.logical_timeout_ms }
    pub(crate) fn matches_target(&self, target: &InstalledTargetReceiptV2) -> bool { self.target() == target }
    pub(crate) fn to_wire(&self) -> proto::RelocalizeRequest {
        proto::RelocalizeRequest { schema_version: 2, request_id: self.0.request_id.clone(),
            assigned_target: Some(proto::relocalize_request::AssignedTarget::AssignedCloudV2(
                proto::AssignedCloudRelocalizeRequestV2 { target: Some(self.0.target.to_wire()),
                    policy: Some(self.0.policy.to_wire()), logical_timeout_ms: self.0.logical_timeout_ms })),
            ..Default::default() }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn wire() -> proto::RelocalizeRequest {
        let metadata = proto::ActivationRequestMetadataV2 { activation_request_id: "mapload:fixture".into(),
            session_id: "installed".into(), map_name: "room".into(),
            asset_identity: Some(proto::AssetIdentityV2 { asset_id: "a".repeat(32) }),
            checkpoint_id: "b".repeat(32), expected_runtime_generation: 3 };
        let activation = proto::InstalledActivationIdentityV2 { request: Some(metadata), runtime_generation: 3,
            lifecycle_epoch: 4, session_generation: 5, backend_kind: "voxel".into(), backend_generation: 6 };
        let source_occupancy = proto::CanonicalOccupancyCommitReceipt {
            commit_id: Some(proto::MappingCommitIdentity { boot_nonce: vec![7;16], sequence: 9 }),
            session_id: "source".into(), session_generation: 8, occupancy_revision: 10,
            source_timestamp_ns: 11, backend_kind: "voxel".into(), backend_generation: 2 };
        let source_cloud = proto::CanonicalCloudCommitReceipt { commit_id: source_occupancy.commit_id.clone(),
            session_generation: 8, occupancy_revision: 10, cloud_revision: 12, source_timestamp_ns: 11, point_count: 2 };
        let installed_occupancy = proto::CanonicalOccupancyCommitReceipt {
            commit_id: Some(proto::MappingCommitIdentity { boot_nonce: vec![1;16], sequence: 2 }),
            session_id: "installed".into(), session_generation: 5, occupancy_revision: 3,
            source_timestamp_ns: 22, backend_kind: "voxel".into(), backend_generation: 6 };
        let installed_cloud = proto::CanonicalCloudCommitReceipt { commit_id: installed_occupancy.commit_id.clone(),
            session_generation: 5, occupancy_revision: 3, cloud_revision: 4, source_timestamp_ns: 22, point_count: 2 };
        proto::RelocalizeRequest { schema_version: 2, request_id: "reloc:exact".into(),
            assigned_target: Some(proto::relocalize_request::AssignedTarget::AssignedCloudV2(proto::AssignedCloudRelocalizeRequestV2 {
                target: Some(proto::InstalledTargetReceiptV2 { activation: Some(activation),
                    source_cloud_member: Some(proto::CheckpointMemberIdentity { checkpoint_id: "b".repeat(32),
                        role: proto::CheckpointMemberRole::CanonicalCloud as i32,
                        occupancy_receipt: Some(source_occupancy), cloud_receipt: Some(source_cloud),
                        frame_id: "map".into(), frame_generation: 0, producer_generation: 0 }),
                    occupancy_receipt: Some(installed_occupancy), cloud_receipt: Some(installed_cloud) }),
                policy: Some(CanonicalRelocalizationPolicyV2::current_cloud().to_wire()), logical_timeout_ms: 0 })),
            ..Default::default() }
    }

    fn body(request: &mut proto::RelocalizeRequest) -> &mut proto::AssignedCloudRelocalizeRequestV2 {
        match request.assigned_target.as_mut().unwrap() {
            proto::relocalize_request::AssignedTarget::AssignedCloudV2(body) => body,
        }
    }

    #[test]
    fn live_boundary_admits_exact_assigned_and_rejects_mixed_requests() {
        let mut request = wire();
        assert_eq!(validate_live_request_boundary(&request), Ok(()));
        request.schema_version = 1;
        assert_eq!(validate_live_request_boundary(&request), Err("RELOCALIZATION_ASSIGNED_FIELDS_MIXED_WITH_LEGACY".into()));
        request.assigned_target = None;
        assert_eq!(validate_live_request_boundary(&request), Ok(()));
        request.schema_version = 3;
        assert_eq!(validate_live_request_boundary(&request), Err("RELOCALIZATION_SCHEMA_VERSION_UNSUPPORTED".into()));
    }

    #[test]
    fn normalized_request_roundtrip_is_exact_and_clones_share_metadata_owner() {
        let admitted = AssignedCloudRelocalizeRequestIdentityV2::admit(wire()).unwrap();
        assert_eq!(admitted.logical_timeout_ms(), 30_000);
        let cloned = admitted.clone();
        assert!(Arc::ptr_eq(&admitted.0, &cloned.0));
        use prost::Message;
        let encoded = admitted.to_wire().encode_to_vec();
        let decoded = proto::RelocalizeRequest::decode(encoded.as_slice()).unwrap();
        assert_eq!(AssignedCloudRelocalizeRequestIdentityV2::admit(decoded).unwrap(), admitted);
        let mut explicit = wire(); body(&mut explicit).logical_timeout_ms = 30_000;
        let explicit = AssignedCloudRelocalizeRequestIdentityV2::admit(explicit).unwrap();
        assert_eq!(explicit, admitted);
        assert!(Arc::ptr_eq(&explicit.0.policy.0, &admitted.0.policy.0));
        assert_eq!(admitted.request_id(), "reloc:exact");
    }

    #[test]
    fn exact_request_conflicts_preserve_existing_normalization_and_limits() {
        let first = AssignedCloudRelocalizeRequestIdentityV2::admit(wire()).unwrap();
        let mut changed = wire(); body(&mut changed).logical_timeout_ms = 250;
        assert_ne!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).unwrap(), first);
        for timeout in [1, 249, 30_001, u64::MAX] {
            let mut changed = wire(); body(&mut changed).logical_timeout_ms = timeout;
            assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        }
        let mut changed = wire(); body(&mut changed).target.as_mut().unwrap().activation.as_mut().unwrap().lifecycle_epoch += 1;
        assert_ne!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).unwrap(), first);
    }

    #[test]
    fn durable_identity_requires_normalization_but_admission_accepts_default() {
        assert!(AssignedCloudRelocalizeRequestIdentityV2::admit_canonical(wire()).is_err());
        let admitted = AssignedCloudRelocalizeRequestIdentityV2::admit(wire()).unwrap();
        assert_eq!(AssignedCloudRelocalizeRequestIdentityV2::admit_canonical(admitted.to_wire()).unwrap(), admitted);
    }

    #[test]
    fn complete_policy_parameters_are_compared_without_digest_substitution() {
        let mutations: [fn(&mut proto::RelocalizationPolicyDescriptorV2); 4] = [
            |p| p.source_parameters[0] ^= 1,
            |p| p.deadline_parameters.push(0),
            |p| p.algorithm_parameters = vec![1;32],
            |p| p.source_policy_version += 1,
        ];
        for mutate in mutations {
            let mut changed = wire(); mutate(body(&mut changed).policy.as_mut().unwrap());
            assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        }
    }

    #[test]
    fn missing_mixed_and_unsupported_live_descriptors_reject() {
        let mut changed = wire(); changed.schema_version = 1;
        assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        let mut changed = wire(); changed.runtime_generation = 3;
        assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        let mut changed = wire(); changed.activation_identity_sha256 = vec![1;32];
        assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        let mut changed = wire(); changed.assigned_target = None;
        assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        let mut changed = wire(); body(&mut changed).policy = None;
        assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
    }

    #[test]
    fn target_receipt_parent_role_and_installed_pair_cannot_be_substituted() {
        let mutations: [fn(&mut proto::InstalledTargetReceiptV2); 5] = [
            |t| t.source_cloud_member.as_mut().unwrap().checkpoint_id = "c".repeat(32),
            |t| t.source_cloud_member.as_mut().unwrap().role = proto::CheckpointMemberRole::FineMap as i32,
            |t| t.cloud_receipt.as_mut().unwrap().point_count += 1,
            |t| t.occupancy_receipt.as_mut().unwrap().commit_id.as_mut().unwrap().sequence += 1,
            |t| t.activation.as_mut().unwrap().backend_kind = "slam_toolbox".into(),
        ];
        for mutate in mutations {
            let mut changed = wire(); mutate(body(&mut changed).target.as_mut().unwrap());
            assert!(AssignedCloudRelocalizeRequestIdentityV2::admit(changed).is_err());
        }
    }
}
