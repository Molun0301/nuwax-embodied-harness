//! Assigned relocalization terminal/query codecs for the existing actor and store.
//!
//! These owners retain bounded lineage and outcome metadata, not source XYZ or
//! checkpoint bytes. Reopening a record verifies retained exact bindings; it
//! cannot attest to source content after that content's owner has been released.
//! No clock, actor, store, request identifier, or content digest is created here.

use std::sync::Arc;

use prost::Message;
use nuwax_codec::lcm::lio::{LIO_BACKEND_DEVICE, LIO_BACKEND_FASTLIO2, LIO_BACKEND_POINTLIO};

use crate::activation_identity::{cloud_receipt_from_wire, occupancy_receipt_from_wire};
use crate::correction_coordinator::{
    validate_relocalization_request_id, RELOCALIZATION_MAX_AGGREGATE_POINTS,
    RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES, RELOCALIZATION_MAX_EMITTED_POINTS,
    RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES, RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES,
    RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES, RELOCALIZATION_MAX_FRAMES,
    RELOCALIZATION_MAX_PROOF_BYTES, RELOCALIZATION_MAX_TERMINAL_BYTES,
    RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES,
};
use crate::proto;
use crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2;

const DURABLE_DOMAIN: &[u8] = b"nuwax.geometry.relocalize.assigned-terminal.v2\0";
const FILE_PREFIX: &str = "assigned-v2.";
const FILE_SUFFIX: &str = ".terminal";

/// The paired coordinator supplies the real V3 decoder and binding checks.
/// During capture it must check the still-owned source and actual outcome;
/// during reopen it checks the canonical V3 body and retained lineage/outcome.
/// Neither mode may manufacture missing receipts or claim source attestation.
/// With no validator, only a claim-free PreAdmission terminal is admissible.
pub(crate) type AssignedTerminalValidatorV2<'a> = dyn Fn(
        &AssignedCloudRelocalizeRequestIdentityV2,
        &proto::AssignedCloudRelocalizeResultV2,
    ) -> Result<(), String>
    + 'a;

#[derive(Debug)]
struct AssignedTerminalDataV2 {
    request: AssignedCloudRelocalizeRequestIdentityV2,
    result: proto::RelocalizeResult,
    encoded: Vec<u8>,
}

/// Clones and retained retries share both the result and its original byte owner.
/// The only copying accessor is `to_wire`, for an actual transport response.
#[derive(Clone, Debug)]
pub(crate) struct AssignedRelocalizationTerminalV2(Arc<AssignedTerminalDataV2>);

impl PartialEq for AssignedRelocalizationTerminalV2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0) || self.0.encoded == other.0.encoded
    }
}
impl Eq for AssignedRelocalizationTerminalV2 {}

impl AssignedRelocalizationTerminalV2 {
    pub(crate) fn new(
        result: proto::RelocalizeResult,
        validator: Option<&AssignedTerminalValidatorV2<'_>>,
    ) -> Result<Self, String> {
        let request = validate_result(&result, validator)?;
        let encoded = result.encode_to_vec();
        Ok(Self(Arc::new(AssignedTerminalDataV2 { request, result, encoded })))
    }

    pub(crate) fn from_encoded(
        encoded: Vec<u8>,
        validator: Option<&AssignedTerminalValidatorV2<'_>>,
    ) -> Result<Self, String> {
        bounded(encoded.len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "TERMINAL")?;
        let result = proto::RelocalizeResult::decode(encoded.as_slice())
            .map_err(|_| "RELOCALIZATION_ASSIGNED_TERMINAL_DECODE_INVALID")?;
        let request = validate_result(&result, validator)?;
        if result.encode_to_vec() != encoded {
            return Err("RELOCALIZATION_ASSIGNED_TERMINAL_NON_CANONICAL".into());
        }
        Ok(Self(Arc::new(AssignedTerminalDataV2 { request, result, encoded })))
    }

    pub(crate) fn request(&self) -> &AssignedCloudRelocalizeRequestIdentityV2 { &self.0.request }
    pub(crate) fn result(&self) -> &proto::RelocalizeResult { &self.0.result }
    pub(crate) fn encoded(&self) -> &[u8] { &self.0.encoded }
    pub(crate) fn to_wire(&self) -> proto::RelocalizeResult { self.0.result.clone() }
    pub(crate) fn body(&self) -> &proto::AssignedCloudRelocalizeResultV2 {
        self.0.result.assigned_cloud_v2.as_ref().expect("validated assigned body")
    }

    /// Deterministic binary record: fixed domain, BE u32 request length, canonical
    /// typed request protobuf, BE u32 terminal length, canonical terminal bytes.
    /// There is no JSON float conversion or digest-based filename/proof.
    pub(crate) fn encode_durable(&self) -> Result<Vec<u8>, String> {
        let request = self.request().to_wire().encode_to_vec();
        let total = DURABLE_DOMAIN.len().checked_add(8)
            .and_then(|n| n.checked_add(request.len()))
            .and_then(|n| n.checked_add(self.encoded().len()))
            .ok_or("RELOCALIZATION_ASSIGNED_DURABLE_LENGTH_OVERFLOW")?;
        bounded(total, RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES, "DURABLE_RECORD")?;
        let mut out = Vec::with_capacity(total);
        out.extend_from_slice(DURABLE_DOMAIN);
        push_segment(&mut out, &request)?;
        push_segment(&mut out, self.encoded())?;
        Ok(out)
    }

    pub(crate) fn decode_durable(
        bytes: &[u8],
        validator: Option<&AssignedTerminalValidatorV2<'_>>,
    ) -> Result<Self, String> {
        bounded(bytes.len(), RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES, "DURABLE_RECORD")?;
        let mut cursor = bytes.strip_prefix(DURABLE_DOMAIN)
            .ok_or("RELOCALIZATION_ASSIGNED_DURABLE_SCHEMA_UNSUPPORTED")?;
        let request_bytes = take_segment(&mut cursor)?;
        let terminal_bytes = take_segment(&mut cursor)?;
        if !cursor.is_empty() {
            return Err("RELOCALIZATION_ASSIGNED_DURABLE_TRAILING_BYTES".into());
        }
        let request_wire = proto::RelocalizeRequest::decode(request_bytes)
            .map_err(|_| "RELOCALIZATION_ASSIGNED_DURABLE_REQUEST_INVALID")?;
        if request_wire.encode_to_vec() != request_bytes {
            return Err("RELOCALIZATION_ASSIGNED_DURABLE_REQUEST_NON_CANONICAL".into());
        }
        let request = AssignedCloudRelocalizeRequestIdentityV2::admit_canonical(request_wire)?;
        let terminal = Self::from_encoded(terminal_bytes.to_vec(), validator)?;
        if terminal.request() != &request {
            return Err("RELOCALIZATION_ASSIGNED_DURABLE_REQUEST_CONFLICT".into());
        }
        Ok(terminal)
    }

    pub(crate) fn decode_durable_named(
        filename: &str,
        bytes: &[u8],
        validator: Option<&AssignedTerminalValidatorV2<'_>>,
    ) -> Result<Self, String> {
        let request_id = assigned_terminal_request_id_from_filename_v2(filename)?;
        let terminal = Self::decode_durable(bytes, validator)?;
        if terminal.request().request_id() != request_id {
            return Err("RELOCALIZATION_ASSIGNED_DURABLE_FILENAME_REQUEST_CONFLICT".into());
        }
        Ok(terminal)
    }
}

/// Reversible bounded filename; callers still use the existing root, lease,
/// fsync/no-replace publication and shared capacity accounting.
pub(crate) fn assigned_terminal_filename_v2(request_id: &str) -> Result<String, String> {
    validate_relocalization_request_id(request_id)?;
    Ok(format!("{FILE_PREFIX}{request_id}{FILE_SUFFIX}"))
}

pub(crate) fn assigned_terminal_request_id_from_filename_v2(name: &str) -> Result<&str, String> {
    let id = name.strip_prefix(FILE_PREFIX).and_then(|s| s.strip_suffix(FILE_SUFFIX))
        .ok_or("RELOCALIZATION_ASSIGNED_DURABLE_FILENAME_INVALID")?;
    validate_relocalization_request_id(id)?;
    Ok(id)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct AssignedRelocalizationTerminalQueryV2 {
    request: AssignedCloudRelocalizeRequestIdentityV2,
}

impl AssignedRelocalizationTerminalQueryV2 {
    pub(crate) fn admit(query: proto::GetRelocalizationTerminalRequest) -> Result<Self, String> {
        bounded(query.encoded_len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "QUERY")?;
        if query.schema_version != 2 || !query.request_identity_sha256.is_empty()
            || !query.activation_identity_sha256.is_empty() {
            return Err("RELOCALIZATION_ASSIGNED_QUERY_SCHEMA_OR_LEGACY_FIELDS_INVALID".into());
        }
        let request = AssignedCloudRelocalizeRequestIdentityV2::admit(
            query.assigned_request_v2.ok_or("RELOCALIZATION_ASSIGNED_QUERY_REQUEST_ABSENT")?,
        )?;
        if query.request_id != request.request_id() {
            return Err("RELOCALIZATION_ASSIGNED_QUERY_REQUEST_ID_MISMATCH".into());
        }
        Ok(Self { request })
    }

    pub(crate) fn request(&self) -> &AssignedCloudRelocalizeRequestIdentityV2 { &self.request }

    pub(crate) fn to_wire(&self) -> proto::GetRelocalizationTerminalRequest {
        proto::GetRelocalizationTerminalRequest {
            schema_version: 2, request_id: self.request.request_id().to_owned(),
            assigned_request_v2: Some(self.request.to_wire()), ..Default::default()
        }
    }

    /// The observation is supplied by the same existing ledger; no new counter
    /// or epoch is allocated, and missing terminals never get a zero receipt.
    pub(crate) fn response(
        &self,
        status: proto::RelocalizationTerminalStatus,
        terminal: Option<&AssignedRelocalizationTerminalV2>,
        ledger_observation: Option<proto::RelocalizationLedgerObservation>,
    ) -> Result<proto::GetRelocalizationTerminalResponse, String> {
        self.validate_status(status, terminal)?;
        Ok(proto::GetRelocalizationTerminalResponse {
            schema_version: 2, request_id: self.request.request_id().to_owned(),
            status: status as i32, terminal: terminal.map(AssignedRelocalizationTerminalV2::to_wire),
            ledger_observation, assigned_request_v2: Some(self.request.to_wire()),
            ..Default::default()
        })
    }

    pub(crate) fn validate_response(
        &self,
        response: proto::GetRelocalizationTerminalResponse,
        validator: Option<&AssignedTerminalValidatorV2<'_>>,
    ) -> Result<Option<AssignedRelocalizationTerminalV2>, String> {
        bounded(response.encoded_len(), RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES, "QUERY_RESPONSE")?;
        if response.schema_version != 2 || response.request_id != self.request.request_id()
            || !response.request_identity_sha256.is_empty() || !response.activation_identity_sha256.is_empty() {
            return Err("RELOCALIZATION_ASSIGNED_QUERY_RESPONSE_BINDING_INVALID".into());
        }
        let request = AssignedCloudRelocalizeRequestIdentityV2::admit_canonical(
            response.assigned_request_v2.ok_or("RELOCALIZATION_ASSIGNED_QUERY_RESPONSE_REQUEST_ABSENT")?,
        )?;
        if request != self.request {
            return Err("RELOCALIZATION_ASSIGNED_QUERY_RESPONSE_REQUEST_CONFLICT".into());
        }
        let status = proto::RelocalizationTerminalStatus::try_from(response.status)
            .map_err(|_| "RELOCALIZATION_ASSIGNED_QUERY_STATUS_INVALID")?;
        let terminal = response.terminal.map(|value| AssignedRelocalizationTerminalV2::new(value, validator)).transpose()?;
        self.validate_status(status, terminal.as_ref())?;
        Ok(terminal)
    }

    fn validate_status(
        &self,
        status: proto::RelocalizationTerminalStatus,
        terminal: Option<&AssignedRelocalizationTerminalV2>,
    ) -> Result<(), String> {
        use proto::RelocalizationTerminalStatus as Status;
        if status == Status::Unspecified || (status == Status::Terminal) != terminal.is_some() {
            return Err("RELOCALIZATION_ASSIGNED_QUERY_STATUS_TERMINAL_MISMATCH".into());
        }
        if terminal.is_some_and(|value| value.request() != &self.request) {
            return Err("RELOCALIZATION_ASSIGNED_QUERY_TERMINAL_REQUEST_CONFLICT".into());
        }
        Ok(())
    }
}

fn validate_result(
    result: &proto::RelocalizeResult,
    validator: Option<&AssignedTerminalValidatorV2<'_>>,
) -> Result<AssignedCloudRelocalizeRequestIdentityV2, String> {
    bounded(result.encoded_len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "TERMINAL")?;
    if result.schema_version != 2 {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_SCHEMA_UNSUPPORTED".into());
    }
    if !result.request_identity_sha256.is_empty() || result.runtime_generation != 0
        || !result.session_id.is_empty() || !result.map_name.is_empty() || result.session_generation != 0
        || result.backend_generation != 0 || !result.activation_identity_sha256.is_empty()
        || !result.target_receipt_sha256.is_empty() || result.target_cloud_revision != 0
        || !result.target_cloud_sha256.is_empty() || !result.asset_manifest_sha256.is_empty()
        || !result.asset_bundle_sha256.is_empty() || result.source_policy_version != 0
        || !result.source_policy_sha256.is_empty() || result.deadline_policy_version != 0
        || !result.deadline_policy_sha256.is_empty() || result.logical_timeout_ms != 0
        || result.algorithm_options_version != 0 || !result.algorithm_options_sha256.is_empty()
        || result.disposition != 0 || result.candidate_sequence != 0 || !result.candidate_payload_sha256.is_empty()
        || !result.evidence_sha256.is_empty() || result.correction_revision != 0 || !result.terminal_proof_sha256.is_empty()
        || !result.detail.is_empty() || !result.candidate_payload_preimage_v1.is_empty()
        || !result.evidence_disclosure_v1.is_empty() || !result.target_kind.is_empty()
        || !result.target_graph_sha256.is_empty() {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_MIXED_WITH_LEGACY".into());
    }
    let body = result.assigned_cloud_v2.as_ref().ok_or("RELOCALIZATION_ASSIGNED_TERMINAL_BODY_ABSENT")?;
    let request = AssignedCloudRelocalizeRequestIdentityV2::admit_canonical(
        body.request.clone().ok_or("RELOCALIZATION_ASSIGNED_TERMINAL_REQUEST_ABSENT")?,
    )?;
    if result.request_id != request.request_id() {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_REQUEST_ID_MISMATCH".into());
    }
    use proto::RelocalizeDisposition as Disposition;
    let disposition = Disposition::try_from(body.disposition)
        .map_err(|_| "RELOCALIZATION_ASSIGNED_TERMINAL_DISPOSITION_INVALID")?;
    if disposition == Disposition::Unspecified {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_DISPOSITION_INVALID".into());
    }
    let applied = matches!(disposition, Disposition::Applied | Disposition::AlreadyApplied);
    // A canonical rejection may observe an existing nonzero revision. Preserve
    // the actual finalizer outcome instead of fabricating a zero revision.
    if (applied && body.correction_revision == 0)
        || (!applied && disposition != Disposition::Rejected && body.correction_revision != 0) {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_REVISION_OUTCOME_MISMATCH".into());
    }
    bounded(body.candidate_payload_v3.len(), RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES, "CANDIDATE")?;
    let evidence = body.evidence.as_ref().ok_or("RELOCALIZATION_ASSIGNED_TERMINAL_EVIDENCE_ABSENT")?;
    validate_evidence(&request, evidence)?;
    use proto::AssignedRelocalizationEvidencePhaseV2 as Phase;
    let phase = Phase::try_from(evidence.phase).map_err(|_| "RELOCALIZATION_ASSIGNED_EVIDENCE_PHASE_INVALID")?;
    if phase == Phase::PreAdmission && body.correction_revision != 0 {
        return Err("RELOCALIZATION_ASSIGNED_PRE_ADMISSION_HAS_REVISION".into());
    }
    if disposition == Disposition::AlgorithmRejected
        && !evidence.algorithm.as_ref().is_some_and(|algorithm| !algorithm.accepted) {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_ALGORITHM_REJECTION_INCOMPLETE".into());
    }
    if applied && (body.candidate_payload_v3.is_empty() || phase != Phase::Finalized
        || !evidence.algorithm.as_ref().is_some_and(|a| a.accepted)) {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_APPLIED_EVIDENCE_INCOMPLETE".into());
    }
    if !body.candidate_payload_v3.is_empty()
        && (!matches!(phase, Phase::AlgorithmCompleted | Phase::Finalized)
            || !evidence.algorithm.as_ref().is_some_and(|a| a.accepted)) {
        return Err("RELOCALIZATION_ASSIGNED_TERMINAL_CANDIDATE_EVIDENCE_INCOMPLETE".into());
    }
    let requires_validator = phase != Phase::PreAdmission || applied || !body.candidate_payload_v3.is_empty();
    if requires_validator {
        validator.ok_or("RELOCALIZATION_ASSIGNED_TERMINAL_VALIDATOR_REQUIRED")?(&request, body)?;
    } else if let Some(validator) = validator {
        validator(&request, body)?;
    }
    Ok(request)
}

fn validate_evidence(
    request: &AssignedCloudRelocalizeRequestIdentityV2,
    evidence: &proto::AssignedRelocalizationEvidenceV2,
) -> Result<(), String> {
    use proto::AssignedRelocalizationEvidencePhaseV2 as Phase;
    if evidence.schema_version != 2 {
        return Err("RELOCALIZATION_ASSIGNED_EVIDENCE_SCHEMA_UNSUPPORTED".into());
    }
    bounded(evidence.encoded_len(), RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES, "EVIDENCE")?;
    bounded(evidence.source_frames.len(), RELOCALIZATION_MAX_FRAMES, "SOURCE_FRAMES")?;
    // Apply the same existing prefix/suffix budgets to their actual typed parts.
    // Detail is in the terminal body, so only the terminal budget bounds it.
    let algorithm_size = evidence.algorithm.as_ref().map_or(0, Message::encoded_len);
    bounded(algorithm_size, RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES, "ALGORITHM_EVIDENCE")?;
    // Subtract only algorithm content, conservatively leaving its tag/length in
    // the prefix budget, without cloning any evidence or source payload.
    bounded(evidence.encoded_len() - algorithm_size, RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES, "SOURCE_EVIDENCE")?;
    let phase = Phase::try_from(evidence.phase).map_err(|_| "RELOCALIZATION_ASSIGNED_EVIDENCE_PHASE_INVALID")?;
    match phase {
        Phase::Unspecified => return Err("RELOCALIZATION_ASSIGNED_EVIDENCE_PHASE_INVALID".into()),
        Phase::PreAdmission if evidence.admission_ordinal.is_some() || evidence.owner.is_some()
            || !evidence.source_frames.is_empty() || evidence.algorithm.is_some() => {
                return Err("RELOCALIZATION_ASSIGNED_PRE_ADMISSION_HAS_CLAIMS".into());
            }
        Phase::PreAdmission => return Ok(()),
        _ => {}
    }
    if evidence.admission_ordinal.is_none_or(|ordinal| ordinal == 0) {
        return Err("RELOCALIZATION_ASSIGNED_ADMISSION_ORDINAL_ABSENT".into());
    }
    let owner = evidence.owner.as_ref().ok_or("RELOCALIZATION_ASSIGNED_OWNER_RECEIPT_ABSENT")?;
    if owner.runtime_generation != request.target().activation.runtime_generation
        || owner.correction_backend_generation == 0 || owner.correction_lease_boot_nonce.len() != 16
        || owner.correction_lease_boot_nonce.iter().all(|b| *b == 0) || owner.correction_lease_generation == 0 {
        return Err("RELOCALIZATION_ASSIGNED_OWNER_RECEIPT_INVALID".into());
    }
    validate_frames(&owner.map_frame_id, &owner.odom_frame_id, &owner.body_frame_id)?;
    // An admitted failure may precede target capture. Once source selection is
    // claimed, its map frame must be the requested installed target frame.
    if !evidence.source_frames.is_empty()
        && owner.map_frame_id != request.target().source_cloud_member.frame_id() {
        return Err("RELOCALIZATION_ASSIGNED_SOURCE_TARGET_FRAME_MISMATCH".into());
    }
    match phase {
        Phase::Admitted if !evidence.source_frames.is_empty() || evidence.algorithm.is_some() =>
            return Err("RELOCALIZATION_ASSIGNED_ADMITTED_PHASE_HAS_LATER_CLAIMS".into()),
        Phase::SourceSelected if evidence.source_frames.is_empty() || evidence.algorithm.is_some() =>
            return Err("RELOCALIZATION_ASSIGNED_SOURCE_PHASE_INVALID".into()),
        Phase::AlgorithmCompleted | Phase::Finalized if evidence.source_frames.is_empty() || evidence.algorithm.is_none() =>
            return Err("RELOCALIZATION_ASSIGNED_ALGORITHM_PHASE_INVALID".into()),
        _ => {}
    }
    let mut previous_key = None;
    let mut source_scope = None;
    let mut aggregate_points = 0_u64;
    for frame in &evidence.source_frames {
        bounded(frame.encoded_len(), RELOCALIZATION_MAX_PROOF_BYTES, "FRAME_RECEIPT")?;
        let occupancy = occupancy_receipt_from_wire(frame.occupancy_receipt.as_ref()
            .ok_or("RELOCALIZATION_ASSIGNED_SOURCE_OCCUPANCY_RECEIPT_ABSENT")?)?;
        let cloud = cloud_receipt_from_wire(frame.cloud_receipt.as_ref()
            .ok_or("RELOCALIZATION_ASSIGNED_SOURCE_CLOUD_RECEIPT_ABSENT")?)?;
        // Lagging cloud baselines are deliberately supported. The paired real
        // validator decides receipt ordering; no same-commit receipt is invented.
        if occupancy.session_id != request.target().activation.request.session_id()
            || occupancy.session_generation != request.target().activation.session_generation
            || occupancy.backend_kind != request.target().activation.backend_kind
            || occupancy.backend_generation != request.target().activation.backend_generation
            || cloud.session_generation != occupancy.session_generation || frame.cloud_revision != cloud.cloud_revision
            || frame.map_frame_generation == 0 || frame.runtime_generation == 0
            || frame.source_sequence == 0 || frame.correlated_odom_source_sequence == 0
            || frame.admitted_point_count == 0 || frame.admitted_point_count > RELOCALIZATION_MAX_EMITTED_POINTS as u64
            || frame.map_frame_id != owner.map_frame_id || frame.odom_frame_id != owner.odom_frame_id
            || frame.body_frame_id != owner.body_frame_id || frame.points_frame_id != owner.body_frame_id {
            return Err("RELOCALIZATION_ASSIGNED_SOURCE_RECEIPT_BINDING_INVALID".into());
        }
        if !matches!(frame.lio_backend, LIO_BACKEND_DEVICE | LIO_BACKEND_FASTLIO2 | LIO_BACKEND_POINTLIO) {
            return Err("RELOCALIZATION_ASSIGNED_SOURCE_BACKEND_INVALID".into());
        }
        if frame.relocalization_mode_epoch.is_some_and(|epoch| epoch == 0
            || epoch != frame.frame_capture_relocalization_epoch
            || owner.relocalization_epoch != Some(epoch)) {
            return Err("RELOCALIZATION_ASSIGNED_SOURCE_EPOCH_MISMATCH".into());
        }
        validate_pose(frame.odom_from_body.as_ref())?;
        validate_pose(frame.base_anchor_map_from_odom.as_ref())?;
        validate_pose(frame.registration_map_from_odom.as_ref())?;
        let geometry = frame.map_geometry.as_ref().ok_or("RELOCALIZATION_ASSIGNED_MAP_GEOMETRY_ABSENT")?;
        if geometry.width == 0 || geometry.height == 0 || !f64::from_bits(geometry.resolution_m_bits).is_finite()
            || f64::from_bits(geometry.resolution_m_bits) <= 0.0
            || !f64::from_bits(geometry.origin_x_m_bits).is_finite() || !f64::from_bits(geometry.origin_y_m_bits).is_finite() {
            return Err("RELOCALIZATION_ASSIGNED_MAP_GEOMETRY_INVALID".into());
        }
        let scope = (occupancy.commit_id.boot_nonce, frame.map_frame_generation,
            frame.lio_backend, frame.runtime_generation);
        if source_scope.is_some_and(|previous| previous != scope) {
            return Err("RELOCALIZATION_ASSIGNED_SOURCE_SCOPE_MISMATCH".into());
        }
        source_scope = Some(scope);
        let key = (occupancy.commit_id.boot_nonce, occupancy.commit_id.sequence,
            occupancy.occupancy_revision, frame.cloud_revision, frame.source_timestamp_ns,
            frame.source_sequence, frame.correlated_odom_source_sequence);
        if previous_key.is_some_and(|previous| previous >= key) {
            return Err("RELOCALIZATION_ASSIGNED_SOURCE_ORDER_INVALID".into());
        }
        previous_key = Some(key);
        aggregate_points = aggregate_points.checked_add(frame.admitted_point_count)
            .ok_or("RELOCALIZATION_ASSIGNED_SOURCE_POINT_COUNT_OVERFLOW")?;
    }
    if aggregate_points > RELOCALIZATION_MAX_AGGREGATE_POINTS as u64 {
        return Err("RELOCALIZATION_ASSIGNED_SOURCE_POINT_BOUND_EXCEEDED".into());
    }
    if let Some(algorithm) = &evidence.algorithm {
        validate_pose(algorithm.estimated_transform.as_ref())?;
        if algorithm.accepted != algorithm.proposed_correction.is_some() {
            return Err("RELOCALIZATION_ASSIGNED_ALGORITHM_CORRECTION_MISMATCH".into());
        }
        if algorithm.proposed_correction.is_some() {
            validate_pose(algorithm.proposed_correction.as_ref())?;
        }
        // Failed algorithms may report infinity; preserve exact output bits.
        // A claimed acceptance must have finite nonnegative metrics.
        // Existing local ICP accepts a finite estimate even if the iterative
        // solver reports non-convergence; recording it must not change policy.
        if algorithm.accepted && (!f64::from_bits(algorithm.mse_bits).is_finite() || f64::from_bits(algorithm.mse_bits) < 0.0
            || !f64::from_bits(algorithm.fitness_bits).is_finite() || f64::from_bits(algorithm.fitness_bits) < 0.0) {
            return Err("RELOCALIZATION_ASSIGNED_ALGORITHM_ACCEPTED_METRICS_INVALID".into());
        }
    }
    Ok(())
}

fn validate_frames(map: &str, odom: &str, body: &str) -> Result<(), String> {
    if [map, odom, body].iter().any(|s| s.is_empty() || s.trim() != *s || s.contains('\0'))
        || map == odom || map == body || odom == body {
        return Err("RELOCALIZATION_ASSIGNED_FRAME_IDS_INVALID".into());
    }
    Ok(())
}

fn validate_pose(pose: Option<&proto::RelocalizationPoseBitsV2>) -> Result<(), String> {
    let pose = pose.ok_or("RELOCALIZATION_ASSIGNED_POSE_ABSENT")?;
    if pose.translation_m_bits.len() != 3 || pose.rotation_xyzw_bits.len() != 4
        || pose.translation_m_bits.iter().chain(&pose.rotation_xyzw_bits)
            .any(|bits| !f64::from_bits(*bits).is_finite()) {
        return Err("RELOCALIZATION_ASSIGNED_POSE_INVALID".into());
    }
    // Quaternion norm/translation policy and source equality belong to the real
    // existing V3 validator. The codec preserves rather than normalizes bits.
    Ok(())
}

fn bounded(actual: usize, maximum: usize, field: &str) -> Result<(), String> {
    if actual > maximum { Err(format!("RELOCALIZATION_ASSIGNED_{field}_BOUND_EXCEEDED")) }
    else { Ok(()) }
}

fn push_segment(out: &mut Vec<u8>, bytes: &[u8]) -> Result<(), String> {
    bounded(bytes.len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "DURABLE_SEGMENT")?;
    let length = u32::try_from(bytes.len()).map_err(|_| "RELOCALIZATION_ASSIGNED_DURABLE_LENGTH_OVERFLOW")?;
    out.extend_from_slice(&length.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

fn take_segment<'a>(cursor: &mut &'a [u8]) -> Result<&'a [u8], String> {
    let (length, rest) = cursor.split_at_checked(4).ok_or("RELOCALIZATION_ASSIGNED_DURABLE_TRUNCATED")?;
    let length = u32::from_be_bytes(length.try_into().map_err(|_| "RELOCALIZATION_ASSIGNED_DURABLE_TRUNCATED")?) as usize;
    bounded(length, RELOCALIZATION_MAX_TERMINAL_BYTES, "DURABLE_SEGMENT")?;
    let (value, rest) = rest.split_at_checked(length).ok_or("RELOCALIZATION_ASSIGNED_DURABLE_TRUNCATED")?;
    *cursor = rest;
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proto::AssignedRelocalizationEvidencePhaseV2 as Phase;
    use proto::RelocalizationTerminalStatus as Status;
    use proto::RelocalizeDisposition as Disposition;

    fn request() -> AssignedCloudRelocalizeRequestIdentityV2 {
        AssignedCloudRelocalizeRequestIdentityV2::admit(crate::relocalization_identity::tests::wire()).unwrap()
    }

    fn pre_admission() -> proto::RelocalizeResult {
        let request = request();
        proto::RelocalizeResult {
            schema_version: 2, request_id: request.request_id().to_owned(),
            assigned_cloud_v2: Some(proto::AssignedCloudRelocalizeResultV2 {
                request: Some(request.to_wire()), disposition: Disposition::TargetUnavailable as i32,
                correction_revision: 0, detail: "RELOCALIZATION_TARGET_UNAVAILABLE".into(),
                candidate_payload_v3: Vec::new(), evidence: Some(proto::AssignedRelocalizationEvidenceV2 {
                    schema_version: 2, phase: Phase::PreAdmission as i32, ..Default::default()
                }),
            }), ..Default::default()
        }
    }

    fn pose() -> proto::RelocalizationPoseBitsV2 {
        proto::RelocalizationPoseBitsV2 {
            translation_m_bits: vec![0_f64.to_bits(); 3],
            rotation_xyzw_bits: vec![0_f64.to_bits(), 0_f64.to_bits(), 0_f64.to_bits(), 1_f64.to_bits()],
        }
    }

    fn source_selected() -> proto::RelocalizeResult {
        let mut result = pre_admission();
        let request = request();
        let target = request.target().to_wire();
        let body = result.assigned_cloud_v2.as_mut().unwrap();
        body.disposition = Disposition::Cancelled as i32;
        body.detail = "RELOCALIZATION_LAST_WAITER_CANCELLED".into();
        body.evidence = Some(proto::AssignedRelocalizationEvidenceV2 {
            schema_version: 2, phase: Phase::SourceSelected as i32, admission_ordinal: Some(1),
            owner: Some(proto::RelocalizationOwnerReceiptV2 {
                runtime_generation: 3, correction_backend_generation: 9,
                correction_lease_boot_nonce: vec![4; 16], correction_lease_generation: 8,
                map_frame_id: "map".into(), odom_frame_id: "odom".into(), body_frame_id: "body".into(),
                relocalization_epoch: Some(3),
            }),
            source_frames: vec![proto::RelocalizationSourceReceiptV2 {
                occupancy_receipt: target.occupancy_receipt, cloud_receipt: target.cloud_receipt,
                cloud_revision: 4, map_frame_generation: 11, lio_backend: 1, runtime_generation: 3,
                source_sequence: 4, source_timestamp_ns: 22, admitted_point_count: 2,
                correlated_odom_source_sequence: 4, correlated_odom_source_timestamp_ns: 22,
                odom_from_body: Some(pose()), map_geometry: Some(proto::RelocalizationMapGeometryV2 {
                    width: 2, height: 2, resolution_m_bits: 0.1_f64.to_bits(),
                    origin_x_m_bits: 0_f64.to_bits(), origin_y_m_bits: 0_f64.to_bits(),
                }),
                base_anchor_map_from_odom: Some(pose()), registration_map_from_odom: Some(pose()),
                map_frame_id: "map".into(), odom_frame_id: "odom".into(), body_frame_id: "body".into(),
                points_frame_id: "body".into(), registration_correction_revision: 0,
                frame_capture_relocalization_epoch: 3, relocalization_mode_epoch: Some(3),
            }], algorithm: None,
        });
        result
    }

    // Test-only lineage policy for codec cases. Deliberately cannot approve any
    // candidate or applied outcome; production uses the coordinator's V3 gate.
    fn fixture_lineage_validator(
        request: &AssignedCloudRelocalizeRequestIdentityV2,
        body: &proto::AssignedCloudRelocalizeResultV2,
    ) -> Result<(), String> {
        if request.request_id() != "reloc:exact" || !body.candidate_payload_v3.is_empty()
            || body.disposition != Disposition::Cancelled as i32 || body.correction_revision != 0 {
            return Err("fixture accepts only cancelled lineage".into());
        }
        Ok(())
    }

    fn body(result: &mut proto::RelocalizeResult) -> &mut proto::AssignedCloudRelocalizeResultV2 {
        result.assigned_cloud_v2.as_mut().unwrap()
    }
    fn evidence(result: &mut proto::RelocalizeResult) -> &mut proto::AssignedRelocalizationEvidenceV2 {
        body(result).evidence.as_mut().unwrap()
    }

    #[test]
    fn capture_moves_original_encoded_allocation_and_retry_shares_owner() {
        let encoded = pre_admission().encode_to_vec();
        let pointer = encoded.as_ptr();
        let terminal = AssignedRelocalizationTerminalV2::from_encoded(encoded, None).unwrap();
        assert_eq!(terminal.encoded().as_ptr(), pointer);
        let weak = Arc::downgrade(&terminal.0);
        let retries: Vec<_> = (0..64).map(|_| terminal.clone()).collect();
        assert!(retries.iter().all(|retry| Arc::ptr_eq(&retry.0, &terminal.0)));
        assert!(retries.iter().all(|retry| retry.encoded().as_ptr() == pointer));
        drop(terminal);
        assert!(weak.upgrade().is_some());
        drop(retries);
        assert!(weak.upgrade().is_none());
    }

    #[test]
    fn exact_replay_and_durable_reopen_preserve_identical_bytes() {
        let terminal = AssignedRelocalizationTerminalV2::new(pre_admission(), None).unwrap();
        let record = terminal.encode_durable().unwrap();
        assert_eq!(record, terminal.clone().encode_durable().unwrap());
        let reopened = AssignedRelocalizationTerminalV2::decode_durable(&record, None).unwrap();
        assert_eq!(reopened, terminal);
        assert_eq!(reopened.encoded(), terminal.encoded());
        assert!(!Arc::ptr_eq(&reopened.0, &terminal.0));
        assert_eq!(reopened.body().request.as_ref().unwrap().schema_version, 2);
        assert!(reopened.result().terminal_proof_sha256.is_empty());
        let filename = assigned_terminal_filename_v2(terminal.request().request_id()).unwrap();
        assert_eq!(AssignedRelocalizationTerminalV2::decode_durable_named(&filename, &record, None).unwrap(), terminal);
        let wrong_filename = assigned_terminal_filename_v2("different-request").unwrap();
        assert!(AssignedRelocalizationTerminalV2::decode_durable_named(&wrong_filename, &record, None).is_err());
        let mut conflicting = pre_admission();
        body(&mut conflicting).detail.push('!');
        assert_ne!(terminal, AssignedRelocalizationTerminalV2::new(conflicting, None).unwrap());
    }

    #[test]
    fn source_elided_lineage_requires_real_validation_on_capture_and_reopen() {
        assert!(AssignedRelocalizationTerminalV2::new(source_selected(), None).is_err());
        let terminal = AssignedRelocalizationTerminalV2::new(source_selected(), Some(&fixture_lineage_validator)).unwrap();
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&terminal.encode_durable().unwrap(), None).is_err());
        let reopened = AssignedRelocalizationTerminalV2::decode_durable(
            &terminal.encode_durable().unwrap(), Some(&fixture_lineage_validator),
        ).unwrap();
        assert_eq!(reopened, terminal);
    }

    #[test]
    fn independent_pose_and_geometry_comparisons_use_raw_bits() {
        let original = source_selected();
        let first = AssignedRelocalizationTerminalV2::new(original.clone(), Some(&fixture_lineage_validator)).unwrap();
        let identical = AssignedRelocalizationTerminalV2::new(original.clone(), Some(&fixture_lineage_validator)).unwrap();
        assert_eq!(first, identical);
        let mut changed = original.clone();
        evidence(&mut changed).source_frames[0].odom_from_body.as_mut().unwrap().translation_m_bits[0] = (-0.0_f64).to_bits();
        let second = AssignedRelocalizationTerminalV2::new(changed, Some(&fixture_lineage_validator)).unwrap();
        assert_ne!(first, second);
        assert_eq!(AssignedRelocalizationTerminalV2::decode_durable(&second.encode_durable().unwrap(), Some(&fixture_lineage_validator)).unwrap(), second);
        let mut changed = original;
        evidence(&mut changed).source_frames[0].map_geometry.as_mut().unwrap().origin_x_m_bits = (-0.0_f64).to_bits();
        assert_ne!(first, AssignedRelocalizationTerminalV2::new(changed, Some(&fixture_lineage_validator)).unwrap());
    }

    #[test]
    fn actual_lagging_cloud_receipt_is_preserved_without_same_commit_synthesis() {
        let mut result = source_selected();
        let frame = &mut evidence(&mut result).source_frames[0];
        let original_cloud = frame.cloud_receipt.clone();
        let occupancy = frame.occupancy_receipt.as_mut().unwrap();
        occupancy.commit_id.as_mut().unwrap().sequence += 1;
        occupancy.occupancy_revision += 1;
        occupancy.source_timestamp_ns += 1;
        let terminal = AssignedRelocalizationTerminalV2::new(result, Some(&fixture_lineage_validator)).unwrap();
        assert_eq!(terminal.body().evidence.as_ref().unwrap().source_frames[0].cloud_receipt, original_cloud);
    }

    #[test]
    fn algorithm_output_bits_survive_reopen_without_numeric_normalization() {
        let mut result = source_selected();
        evidence(&mut result).phase = Phase::AlgorithmCompleted as i32;
        evidence(&mut result).algorithm = Some(proto::RelocalizationAlgorithmReceiptV2 {
            converged: false, accepted: false, mse_bits: 0_f64.to_bits(),
            fitness_bits: 0x7ff8_0000_0000_0001, iterations: 1, feature_matches: 0,
            ransac_inliers: 0, estimated_transform: Some(pose()), proposed_correction: None,
        });
        let original = AssignedRelocalizationTerminalV2::new(result.clone(), Some(&fixture_lineage_validator)).unwrap();
        evidence(&mut result).algorithm.as_mut().unwrap().mse_bits = (-0.0_f64).to_bits();
        let signed_zero = AssignedRelocalizationTerminalV2::new(result.clone(), Some(&fixture_lineage_validator)).unwrap();
        assert_ne!(original, signed_zero);
        evidence(&mut result).algorithm.as_mut().unwrap().fitness_bits = 0x7ff8_0000_0000_0002;
        let different_nan = AssignedRelocalizationTerminalV2::new(result, Some(&fixture_lineage_validator)).unwrap();
        assert_ne!(signed_zero, different_nan);
        assert_eq!(AssignedRelocalizationTerminalV2::decode_durable(
            &different_nan.encode_durable().unwrap(), Some(&fixture_lineage_validator),
        ).unwrap(), different_nan);
    }

    #[test]
    fn applied_and_candidate_claims_never_pass_without_paired_validator() {
        let mut result = source_selected();
        body(&mut result).disposition = Disposition::Applied as i32;
        body(&mut result).correction_revision = 1;
        body(&mut result).candidate_payload_v3 = vec![0, 0, 0, 3];
        evidence(&mut result).phase = Phase::Finalized as i32;
        evidence(&mut result).algorithm = Some(proto::RelocalizationAlgorithmReceiptV2 {
            converged: true, accepted: true, mse_bits: 0_f64.to_bits(), fitness_bits: 1_f64.to_bits(),
            iterations: 1, feature_matches: 1, ransac_inliers: 1,
            estimated_transform: Some(pose()), proposed_correction: Some(pose()),
        });
        assert!(AssignedRelocalizationTerminalV2::new(result.clone(), None).is_err());
        assert!(AssignedRelocalizationTerminalV2::new(result.clone(), Some(&fixture_lineage_validator)).is_err());
        let rejected_by_v3 = |_: &AssignedCloudRelocalizeRequestIdentityV2, _: &proto::AssignedCloudRelocalizeResultV2|
            Err("real V3 decoder rejects truncated body".to_owned());
        assert_eq!(AssignedRelocalizationTerminalV2::new(result, Some(&rejected_by_v3)).unwrap_err().as_str(),
            "real V3 decoder rejects truncated body");
    }

    #[test]
    fn pre_admission_rejection_cannot_claim_a_revision_without_a_validator() {
        let mut malformed = pre_admission();
        body(&mut malformed).disposition = Disposition::Rejected as i32;
        body(&mut malformed).correction_revision = 1;
        assert_eq!(AssignedRelocalizationTerminalV2::new(malformed.clone(), None).unwrap_err(),
            "RELOCALIZATION_ASSIGNED_PRE_ADMISSION_HAS_REVISION");
        let encoded = malformed.encode_to_vec();
        assert_eq!(AssignedRelocalizationTerminalV2::from_encoded(encoded.clone(), None).unwrap_err(),
            "RELOCALIZATION_ASSIGNED_PRE_ADMISSION_HAS_REVISION");
        let mut durable = DURABLE_DOMAIN.to_vec();
        push_segment(&mut durable, &request().to_wire().encode_to_vec()).unwrap();
        push_segment(&mut durable, &encoded).unwrap();
        assert_eq!(AssignedRelocalizationTerminalV2::decode_durable(&durable, None).unwrap_err(),
            "RELOCALIZATION_ASSIGNED_PRE_ADMISSION_HAS_REVISION");
    }

    #[test]
    fn query_normalizes_default_timeout_and_never_fills_missing_receipts() {
        let wire = crate::relocalization_identity::tests::wire();
        let query = AssignedRelocalizationTerminalQueryV2::admit(proto::GetRelocalizationTerminalRequest {
            schema_version: 2, request_id: wire.request_id.clone(), assigned_request_v2: Some(wire), ..Default::default()
        }).unwrap();
        assert_eq!(query.request().logical_timeout_ms(), 30_000);
        let terminal = AssignedRelocalizationTerminalV2::new(pre_admission(), None).unwrap();
        let observation = proto::RelocalizationLedgerObservation { observation_schema_version: 1,
            admitted_command_count: 2, observation_epoch: 9, ..Default::default() };
        for status in [Status::Pending, Status::NotFound, Status::IdentityConflict] {
            let response = query.response(status, None, Some(observation.clone())).unwrap();
            assert!(response.terminal.is_none());
            assert!(response.request_identity_sha256.is_empty());
            assert!(response.activation_identity_sha256.is_empty());
            assert_eq!(response.ledger_observation, Some(observation.clone()));
            assert_eq!(response.assigned_request_v2, Some(query.request().to_wire()));
            assert!(query.validate_response(response, None).unwrap().is_none());
        }
        let response = query.response(Status::Terminal, Some(&terminal), Some(observation)).unwrap();
        assert_eq!(query.validate_response(response, None).unwrap(), Some(terminal));
        assert!(query.response(Status::Terminal, None, None).is_err());
        assert!(query.response(Status::Unspecified, None, None).is_err());
        let mut missing = query.to_wire(); missing.assigned_request_v2 = None;
        assert!(AssignedRelocalizationTerminalQueryV2::admit(missing).is_err());
    }

    #[test]
    fn query_response_exact_target_and_policy_skew_is_rejected() {
        let query = AssignedRelocalizationTerminalQueryV2::admit(proto::GetRelocalizationTerminalRequest {
            schema_version: 2, request_id: request().request_id().to_owned(),
            assigned_request_v2: Some(request().to_wire()), ..Default::default()
        }).unwrap();
        let mut response = query.response(Status::NotFound, None, None).unwrap();
        let proto::relocalize_request::AssignedTarget::AssignedCloudV2(target) =
            response.assigned_request_v2.as_mut().unwrap().assigned_target.as_mut().unwrap();
        target.target.as_mut().unwrap().activation.as_mut().unwrap().lifecycle_epoch += 1;
        assert!(query.validate_response(response, None).is_err());
        let mut query_wire = query.to_wire(); query_wire.request_identity_sha256 = vec![1; 32];
        assert!(AssignedRelocalizationTerminalQueryV2::admit(query_wire).is_err());
        let mut query_wire = query.to_wire(); query_wire.schema_version = 1;
        assert!(AssignedRelocalizationTerminalQueryV2::admit(query_wire).is_err());
        let mut query_wire = query.to_wire(); query_wire.request_id.push('x');
        assert!(AssignedRelocalizationTerminalQueryV2::admit(query_wire).is_err());
    }

    #[test]
    fn legacy_mixing_missing_fields_and_malformed_receipts_fail_closed() {
        let changes: [fn(&mut proto::RelocalizeResult); 9] = [
            |r| r.schema_version = 1,
            |r| r.request_id.push('x'),
            |r| r.disposition = Disposition::Applied as i32,
            |r| r.evidence_sha256 = vec![1; 32],
            |r| r.candidate_payload_preimage_v1 = vec![1],
            |r| r.assigned_cloud_v2 = None,
            |r| body(r).request = None,
            |r| body(r).evidence = None,
            |r| body(r).correction_revision = 1,
        ];
        for change in changes {
            let mut result = pre_admission(); change(&mut result);
            assert!(AssignedRelocalizationTerminalV2::new(result, None).is_err());
        }
        let changes: [fn(&mut proto::RelocalizeResult); 9] = [
            |r| evidence(r).owner.as_mut().unwrap().correction_lease_boot_nonce.clear(),
            |r| evidence(r).source_frames[0].occupancy_receipt = None,
            |r| evidence(r).source_frames[0].cloud_receipt = None,
            |r| evidence(r).source_frames[0].cloud_revision += 1,
            |r| evidence(r).source_frames[0].admitted_point_count = 0,
            |r| { evidence(r).source_frames[0].odom_from_body.as_mut().unwrap().rotation_xyzw_bits.pop(); },
            |r| evidence(r).source_frames[0].odom_from_body.as_mut().unwrap().translation_m_bits[0] = f64::NAN.to_bits(),
            |r| evidence(r).source_frames[0].relocalization_mode_epoch = Some(5),
            |r| { let duplicate = evidence(r).source_frames[0].clone(); evidence(r).source_frames.push(duplicate); },
        ];
        for change in changes {
            let mut result = source_selected(); change(&mut result);
            assert!(AssignedRelocalizationTerminalV2::new(result, Some(&fixture_lineage_validator)).is_err());
        }
    }

    #[test]
    fn selected_source_cannot_reopen_under_a_different_target_frame() {
        let mut malformed = source_selected();
        let request = body(&mut malformed).request.as_mut().unwrap();
        let proto::relocalize_request::AssignedTarget::AssignedCloudV2(target) = request.assigned_target.as_mut().unwrap();
        target.target.as_mut().unwrap().source_cloud_member.as_mut().unwrap().frame_id = "different_map".into();
        let request_bytes = body(&mut malformed).request.as_ref().unwrap().encode_to_vec();
        let terminal_bytes = malformed.encode_to_vec();
        assert_eq!(AssignedRelocalizationTerminalV2::new(malformed, Some(&fixture_lineage_validator)).unwrap_err(),
            "RELOCALIZATION_ASSIGNED_SOURCE_TARGET_FRAME_MISMATCH");
        assert_eq!(AssignedRelocalizationTerminalV2::from_encoded(terminal_bytes.clone(), Some(&fixture_lineage_validator)).unwrap_err(),
            "RELOCALIZATION_ASSIGNED_SOURCE_TARGET_FRAME_MISMATCH");
        let mut durable = DURABLE_DOMAIN.to_vec();
        push_segment(&mut durable, &request_bytes).unwrap();
        push_segment(&mut durable, &terminal_bytes).unwrap();
        assert_eq!(AssignedRelocalizationTerminalV2::decode_durable(&durable, Some(&fixture_lineage_validator)).unwrap_err(),
            "RELOCALIZATION_ASSIGNED_SOURCE_TARGET_FRAME_MISMATCH");
    }

    #[test]
    fn source_domains_are_checked_before_sequence_ordering() {
        let changes: [fn(&mut proto::RelocalizationSourceReceiptV2); 4] = [
            |frame| { frame.occupancy_receipt.as_mut().unwrap().commit_id.as_mut().unwrap().boot_nonce = vec![9;16];
                frame.cloud_receipt.as_mut().unwrap().commit_id.as_mut().unwrap().boot_nonce = vec![9;16]; },
            |frame| frame.map_frame_generation += 1,
            |frame| frame.lio_backend = LIO_BACKEND_FASTLIO2,
            |frame| frame.runtime_generation += 1,
        ];
        for change in changes {
            let mut result = source_selected();
            let mut second = evidence(&mut result).source_frames[0].clone();
            // Keep identical ordering keys where possible: a changed producer
            // namespace is neither an old duplicate nor a lexically newer scan.
            change(&mut second);
            evidence(&mut result).source_frames.push(second);
            assert_eq!(AssignedRelocalizationTerminalV2::new(result, Some(&fixture_lineage_validator))
                .unwrap_err(), "RELOCALIZATION_ASSIGNED_SOURCE_SCOPE_MISMATCH");
        }
    }

    #[test]
    fn durable_codec_rejects_truncation_unknown_fields_skew_and_oversized_lengths() {
        let terminal = AssignedRelocalizationTerminalV2::new(pre_admission(), None).unwrap();
        let record = terminal.encode_durable().unwrap();
        for length in [0, DURABLE_DOMAIN.len(), DURABLE_DOMAIN.len() + 3, record.len() - 1] {
            assert!(AssignedRelocalizationTerminalV2::decode_durable(&record[..length], None).is_err());
        }
        let mut trailing = record.clone(); trailing.push(0);
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&trailing, None).is_err());
        let mut oversized = record.clone();
        oversized[DURABLE_DOMAIN.len()..DURABLE_DOMAIN.len() + 4].copy_from_slice(&u32::MAX.to_be_bytes());
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&oversized, None).is_err());
        let mut terminal_unknown = terminal.encoded().to_vec(); terminal_unknown.extend_from_slice(&[0xf8, 0x07, 1]);
        assert!(AssignedRelocalizationTerminalV2::from_encoded(terminal_unknown, None).is_err());
        let mut request_wire = request().to_wire(); request_wire.request_id.push('x');
        let mut skewed = DURABLE_DOMAIN.to_vec();
        push_segment(&mut skewed, &request_wire.encode_to_vec()).unwrap();
        push_segment(&mut skewed, terminal.encoded()).unwrap();
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&skewed, None).is_err());
        let mut unnormalized = DURABLE_DOMAIN.to_vec();
        push_segment(&mut unnormalized, &crate::relocalization_identity::tests::wire().encode_to_vec()).unwrap();
        push_segment(&mut unnormalized, terminal.encoded()).unwrap();
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&unnormalized, None).is_err());
    }

    #[test]
    fn bounded_terminal_frame_and_candidate_bytes_use_existing_budgets() {
        let mut result = pre_admission(); body(&mut result).detail = "x".repeat(RELOCALIZATION_MAX_TERMINAL_BYTES);
        assert!(AssignedRelocalizationTerminalV2::new(result, None).is_err());
        let mut result = pre_admission(); body(&mut result).candidate_payload_v3 = vec![0; RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES + 1];
        assert!(AssignedRelocalizationTerminalV2::new(result, None).is_err());
        let mut result = source_selected();
        let frame = evidence(&mut result).source_frames[0].clone();
        evidence(&mut result).source_frames = vec![frame; RELOCALIZATION_MAX_FRAMES + 1];
        assert!(AssignedRelocalizationTerminalV2::new(result, Some(&fixture_lineage_validator)).is_err());
        let mut result = source_selected();
        evidence(&mut result).source_frames[0].map_frame_id = "x".repeat(RELOCALIZATION_MAX_PROOF_BYTES);
        assert!(AssignedRelocalizationTerminalV2::new(result, Some(&fixture_lineage_validator)).is_err());
        // No invented detail<=4096 restriction: its actual enclosing cap applies.
        let mut result = pre_admission(); body(&mut result).detail = "x".repeat(RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES + 1);
        assert!(AssignedRelocalizationTerminalV2::new(result, None).is_ok());
    }

    #[test]
    fn filename_is_reversible_bounded_and_rejects_path_or_digest_substitution() {
        for id in ["reloc:exact", ".", "..", "ABC_09.-:", &"a".repeat(128)] {
            let name = assigned_terminal_filename_v2(id).unwrap();
            assert!(name.len() < 255);
            assert_eq!(assigned_terminal_request_id_from_filename_v2(&name).unwrap(), id);
        }
        for id in ["", "../outside", "x/y", "x\\y", "x\0y", "é", &"a".repeat(129)] {
            assert!(assigned_terminal_filename_v2(id).is_err());
        }
        assert!(assigned_terminal_request_id_from_filename_v2("abc.json").is_err());
        assert!(assigned_terminal_request_id_from_filename_v2("assigned-v2..terminal").is_err());
    }
}
