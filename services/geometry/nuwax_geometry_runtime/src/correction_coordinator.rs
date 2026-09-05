//! Geometry-owned correction candidate contract and library PGO runtime owner.
//!
//! This module began as the G004-A preflight contract. G004-B retains that
//! fail-closed candidate boundary and adds the Geometry-owned canonical apply
//! transaction plus the retained production transport lifecycle.

#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "the correction contract retains validation helpers beyond the production call surface"
    )
)]
mod preflight_only_contract {

    use std::collections::{HashMap, HashSet};
    use std::ffi::OsString;
    use std::fs::{self, File, OpenOptions};
    use std::io::{Read, Write};
    use std::os::unix::fs::MetadataExt;
    use std::path::{Path, PathBuf};
    use std::sync::{
        atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering},
        mpsc as std_mpsc, Arc,
    };
    use std::time::{Duration, Instant};

    use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
    use nix::fcntl::{Flock, FlockArg};
    use nuwax_codec::lcm::native_pgo as native_pgo_codec;
    use nuwax_mapping::loop_closure::{
        LoopClosureCoordinator, LoopClosurePipelineConfig, ProcessFrameResult,
    };
    use nuwax_mapping::relocalization::{
        relocation_pipeline_with_config, RelocalizationConfig, RelocalizationResult,
    };
    use nuwax_nav_pgo_bridge::{
        OnlinePgoConfig, OnlinePgoIngress, OnlinePgoOwner, OnlinePgoStopCause,
        ValidatedOnlinePgoDisposition,
    };
    use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
    use nuwax_transport::{LcmTransport, OwnedLcmSubscription};
    use prost::Message;
    use serde::{Deserialize, Serialize};
    use thiserror::Error;
    use tokio::sync::{mpsc, oneshot, watch, Notify, OwnedSemaphorePermit, Semaphore};

    use crate::lcm_runtime::ValidatedFrameBindings;
    use crate::map_checksum::sha256_bytes;
    use crate::mapping_commit::MappingCommitId;
    use crate::owner_profile::CorrectionBackend;

    const CORRECTION_SCHEMA_VERSION_V1: i32 = 1;
    const CORRECTION_BACKEND_LIBRARY_PGO: i32 = 1;
    const CORRECTION_BACKEND_NATIVE_PGO: i32 = 2;
    const CORRECTION_BACKEND_RELOCALIZATION: i32 = 3;
    const CORRECTION_TRANSFORM_ABSOLUTE_LEFT_SE3: i32 = 1;
    const CORRECTION_CANDIDATE_FINGERPRINT_V1: [u8; 8] = 0x8510_78c0_24d5_b010_u64.to_be_bytes();
    const CORRECTION_CANDIDATE_MESSAGE_TYPE: &str = "nuwax_msgs.CorrectionCandidate";
    const MAX_IDENTITY_STRING_BYTES: usize = 255;
    const MAX_WIRE_PAYLOAD_BYTES: usize = 4_096;
    const MAX_REGISTERED_CLOUD_POINTS: u64 = 100_000;
    const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-6;
    const MAX_TRANSLATION_ABS_METERS: f64 = 1_000_000.0;
    const CORRECTION_INPUT_QUEUE_CAPACITY: usize = 1;
    const CORRECTION_PHASE_A_OPEN: u8 = 0;
    const CORRECTION_PHASE_A_INTERNAL_CLOSED: u8 = 1;
    const CORRECTION_PHASE_A_PROCESS_STOP_REQUESTED: u8 = 2;
    const CORRECTION_PHASE_A_PROCESS_STOP_FAILED: u8 = 3;
    const CORRECTION_PHASE_A_PROCESS_STOP_SUCCEEDED: u8 = 4;
    const CORRECTION_PUBLISH_TIMEOUT: Duration = Duration::from_secs(2);
    const CORRECTION_APPLY_ACK_TIMEOUT: Duration = Duration::from_secs(5);
    const CORRECTION_WORKER_STOP_TIMEOUT: Duration = Duration::from_secs(3);
    const CORRECTION_SHUTDOWN_ATTEMPT_TIMEOUT: Duration = Duration::from_secs(3);
    const CORRECTION_DISCARD_TIMEOUT: Duration = Duration::from_millis(250);
    const CANONICAL_FINALIZATION_DEADLINE: Duration = Duration::from_secs(5);
    const CORRECTION_CLEANUP_QUEUE_CAPACITY: usize = 4;
    const RELOCALIZATION_COMMAND_QUEUE_CAPACITY: usize = 16;
    const RELOCALIZATION_MAX_WAITERS: usize = 16;
    const RELOCALIZATION_MAX_COMMAND_RECORDS: usize = 1_024;
    const RELOCALIZATION_MAX_TERMINAL_BYTES: usize = 65_536;
    const RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES: usize =
        2 * RELOCALIZATION_MAX_TERMINAL_BYTES + 128 + 3 * 64 + 512;
    const RELOCALIZATION_TERMINAL_TEMP_PREFIX: &str = ".nuwax-relocalization-terminal-store-v4.";
    const RELOCALIZATION_TERMINAL_TEMP_SUFFIX: &str = ".tmp";
    const RELOCALIZATION_TERMINAL_LEASE_PREFIX: &str = ".nuwax-relocalization-owner-v4.";
    const RELOCALIZATION_TERMINAL_LEASE_SUFFIX: &str = ".lease";
    const RELOCALIZATION_TERMINAL_CLEANUP_LOCK_NAME: &str = ".cleanup.lock";
    const RELOCALIZATION_TERMINAL_OWNER_TOKEN_BYTES: usize = 16;
    const RELOCALIZATION_TERMINAL_ATTEMPT_TOKEN_BYTES: usize = 16;
    const RELOCALIZATION_TERMINAL_STORE_DIR_ENV: &str =
        "NUWAX_GEOMETRY_RELOCALIZATION_TERMINAL_STORE_DIR";
    const RELOCALIZATION_MAX_CONFLICTS_PER_RECORD: usize = 64;
    const RELOCALIZATION_MAX_FRAME_PROOF_SCOPES: usize = 8;
    const RELOCALIZATION_MAX_FRAME_PROOFS_PER_SCOPE: usize = 16;
    const RELOCALIZATION_MAX_FRAME_PROOFS_TOTAL: usize = 64;
    const RELOCALIZATION_SCHEMA_VERSION_V1: u32 = 1;
    const RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS: u64 = 250;
    const RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS: u64 = 30_000;
    const RELOCALIZATION_ACCUMULATION_TIMEOUT: Duration = Duration::from_secs(2);
    const RELOCALIZATION_MAX_FRAMES: usize = 8;
    const RELOCALIZATION_MIN_AGGREGATE_POINTS: usize = 50_000;
    const RELOCALIZATION_MAX_AGGREGATE_POINTS: usize = 250_000;
    const RELOCALIZATION_MAX_EMITTED_POINTS: usize = 100_000;
    const RELOCALIZATION_MAX_PER_FRAME_F64_BYTES: usize = 2_400_000;
    const RELOCALIZATION_MAX_AGGREGATE_F64_BYTES: usize = 6_000_000;
    const RELOCALIZATION_MAX_PROOF_BYTES: usize = 4_096;
    const RELOCALIZATION_MAX_ENVELOPE_DATA_BYTES: usize = 25_600_000;
    const RELOCALIZATION_TARGET_POLL_INTERVAL: Duration = Duration::from_millis(10);
    const RELOCALIZATION_EVIDENCE_DOMAIN_V1: &[u8] = b"nuwax.geometry.relocalize.evidence.v1\0";
    const RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1: &[u8] =
        b"nuwax.geometry.relocalize.evidence-disclosure.v1\0";
    const RELOCALIZATION_EVIDENCE_COMPOSITE_DOMAIN_V1: &[u8] =
        b"nuwax.geometry.relocalize.evidence-composite.v1\0";
    const RELOCALIZATION_SOURCE_COMMITMENT_DOMAIN_V1: &[u8] =
        b"nuwax.geometry.relocalize.source-commitment.v1\0";
    const RELOCALIZATION_TERMINAL_PROOF_DOMAIN_V1: &[u8] =
        b"nuwax.geometry.relocalize.terminal-proof.v1\0";
    const RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES: usize = MAX_WIRE_PAYLOAD_BYTES;
    const RELOCALIZATION_MAX_INLINE_EVIDENCE_PREIMAGE_BYTES: usize = 4_096;
    const RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES: usize = 16_384;
    const RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES: usize = 4_096;
    const RELOCALIZATION_MAX_CANONICAL_SOURCE_BYTES: usize =
        8 + RELOCALIZATION_MAX_EMITTED_POINTS * 3 * std::mem::size_of::<f64>();
    const RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES: usize =
        RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES + RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES + 256;
    const RELOCALIZATION_MAX_INLINE_EVIDENCE_DISCLOSURE_BYTES: usize =
        RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1.len()
            + std::mem::size_of::<u32>()
            + std::mem::size_of::<u8>()
            + std::mem::size_of::<u64>()
            + RELOCALIZATION_MAX_INLINE_EVIDENCE_PREIMAGE_BYTES;
    const RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_TERMINAL: usize =
        RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES + RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES;
    const RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_COMMAND_RECORD: usize =
        RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_TERMINAL
            + (RELOCALIZATION_MAX_CONFLICTS_PER_RECORD + 1)
                * RELOCALIZATION_MAX_INLINE_EVIDENCE_DISCLOSURE_BYTES;
    // StoredRelocalizationResult keeps both the protobuf value and its exact
    // encoded replay bytes, so disclosure payloads have a conservative 2x cap.
    const RELOCALIZATION_MAX_RETAINED_DISCLOSURE_BYTES: usize = 2
        * RELOCALIZATION_MAX_COMMAND_RECORDS
        * RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_COMMAND_RECORD;
    pub(crate) const ACTIVATION_RECEIPT_SCHEMA_VERSION_V1: u32 = 1;
    pub(crate) const RELOCALIZATION_SOURCE_POLICY_VERSION_V1: u32 = 1;
    pub(crate) const RELOCALIZATION_DEADLINE_POLICY_VERSION_V1: u32 = 1;
    pub(crate) const RELOCALIZATION_ALGORITHM_OPTIONS_VERSION_V1: u32 = 1;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum CorrectionCandidateBackend {
        LibraryPgo,
        NativePgo,
        Relocalization,
    }

    impl CorrectionCandidateBackend {
        const fn wire_code(self) -> i32 {
            match self {
                Self::LibraryPgo => CORRECTION_BACKEND_LIBRARY_PGO,
                Self::NativePgo => CORRECTION_BACKEND_NATIVE_PGO,
                Self::Relocalization => CORRECTION_BACKEND_RELOCALIZATION,
            }
        }

        fn from_wire_code(value: i32) -> Result<Self, CorrectionWireError> {
            match value {
                CORRECTION_BACKEND_LIBRARY_PGO => Ok(Self::LibraryPgo),
                CORRECTION_BACKEND_NATIVE_PGO => Ok(Self::NativePgo),
                CORRECTION_BACKEND_RELOCALIZATION => Ok(Self::Relocalization),
                _ => Err(CorrectionWireError::Invalid(format!(
                    "unknown correction backend code {value}"
                ))),
            }
        }
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum CorrectionTransformSemantics {
        /// Full SE(3) absolute left correction: `corrected_pose = C * odom_pose`.
        ///
        /// This value is a backend proposal. It is not a canonical `map <- odom`
        /// transform and it is not an incremental correction delta.
        AbsoluteLeftCorrectionSe3,
    }

    impl CorrectionTransformSemantics {
        const fn wire_code(self) -> i32 {
            match self {
                Self::AbsoluteLeftCorrectionSe3 => CORRECTION_TRANSFORM_ABSOLUTE_LEFT_SE3,
            }
        }

        fn from_wire_code(value: i32) -> Result<Self, CorrectionWireError> {
            match value {
                CORRECTION_TRANSFORM_ABSOLUTE_LEFT_SE3 => Ok(Self::AbsoluteLeftCorrectionSe3),
                _ => Err(CorrectionWireError::Invalid(format!(
                    "unsupported correction transform semantics {value}"
                ))),
            }
        }
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct CorrectionLeaseId {
        pub(crate) boot_nonce: u128,
        pub(crate) generation: u64,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct RegisteredCloudProof {
        pub(crate) source_sequence: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) point_count: u64,
        pub(crate) envelope_sha256: [u8; 32],
        pub(crate) body_points_sha256: [u8; 32],
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct CorrelatedOdomProof {
        pub(crate) source_sequence: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) pose_sha256: [u8; 32],
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub(crate) struct CommittedMapProof {
        pub(crate) source_timestamp_ns: u64,
        pub(crate) width: u64,
        pub(crate) height: u64,
        pub(crate) resolution_m: f64,
        pub(crate) origin_x_m: f64,
        pub(crate) origin_y_m: f64,
        pub(crate) payload_sha256: [u8; 32],
        pub(crate) boundary_sha256: [u8; 32],
    }

    #[derive(Clone, Debug, PartialEq)]
    pub(crate) struct CorrectionCandidateIdentity {
        pub(crate) backend: CorrectionCandidateBackend,
        pub(crate) transform_semantics: CorrectionTransformSemantics,
        pub(crate) runtime_generation: u64,
        pub(crate) backend_generation: u64,
        pub(crate) candidate_sequence: u64,
        pub(crate) mapping_session_id: String,
        pub(crate) mapping_session_generation: u64,
        pub(crate) map_frame_generation: u64,
        pub(crate) mapping_commit_id: MappingCommitId,
        pub(crate) occupancy_revision: u64,
        pub(crate) cloud_revision: u64,
        pub(crate) registered_cloud: RegisteredCloudProof,
        pub(crate) correlated_odom: CorrelatedOdomProof,
        pub(crate) committed_map: CommittedMapProof,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) map_frame_id: String,
        pub(crate) odom_frame_id: String,
        pub(crate) body_frame_id: String,
        pub(crate) points_frame_id: String,
        pub(crate) correction_lease: CorrectionLeaseId,
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    /// Full SE(3) absolute correction proposal `C`.
    ///
    /// No planar projection is permitted. Geometry must not interpret this as
    /// either canonical `map <- odom` or an incremental delta.
    pub(crate) struct CorrectionTransform {
        pub(crate) translation_m: [f64; 3],
        pub(crate) rotation_xyzw: [f64; 4],
    }

    impl CorrectionTransform {
        fn from_isometry(transform: &Isometry3<f64>) -> Self {
            let quaternion = transform.rotation.quaternion();
            Self {
                translation_m: transform.translation.vector.into(),
                rotation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
            }
        }
    }

    #[derive(Clone, Debug, PartialEq)]
    pub(crate) struct CorrectionCandidate {
        pub(crate) identity: CorrectionCandidateIdentity,
        pub(crate) transform: CorrectionTransform,
        pub(crate) payload_sha256: [u8; 32],
    }

    impl CorrectionCandidate {
        pub(crate) fn seal(
            identity: CorrectionCandidateIdentity,
            transform: CorrectionTransform,
        ) -> Result<Self, CorrectionRejectReason> {
            let mut candidate = Self {
                identity,
                transform,
                payload_sha256: [0; 32],
            };
            candidate.validate_fields()?;
            candidate.payload_sha256 = candidate.expected_payload_sha256()?;
            Ok(candidate)
        }

        fn expected_payload_sha256(&self) -> Result<[u8; 32], CorrectionRejectReason> {
            Ok(sha256_bytes(&self.encode_body_prefix()?))
        }

        fn validate_integrity(&self) -> Result<(), CorrectionRejectReason> {
            self.validate_fields()?;
            if self.payload_sha256 != self.expected_payload_sha256()? {
                return Err(CorrectionRejectReason::PayloadHashMismatch);
            }
            Ok(())
        }

        fn validate_fields(&self) -> Result<(), CorrectionRejectReason> {
            let identity = &self.identity;
            if identity.runtime_generation == 0 {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "runtime_generation must be positive",
                ));
            }
            if identity.backend_generation == 0 {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "backend_generation must be positive",
                ));
            }
            if identity.candidate_sequence == 0 {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "candidate_sequence must be positive",
                ));
            }
            validate_identity_string("mapping_session_id", &identity.mapping_session_id)?;
            if identity.mapping_session_generation == 0 {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "mapping_session_generation must be positive",
                ));
            }
            if identity.map_frame_generation == 0 {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "map_frame_generation must be positive",
                ));
            }
            if identity.mapping_commit_id.boot_nonce == 0
                || identity.mapping_commit_id.sequence == 0
            {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "mapping commit identity must be complete",
                ));
            }
            if identity.occupancy_revision == 0 || identity.cloud_revision == 0 {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "occupancy and cloud revisions must be independently positive",
                ));
            }
            let cloud = identity.registered_cloud;
            if cloud.source_sequence == 0
                || cloud.point_count == 0
                || cloud.point_count > MAX_REGISTERED_CLOUD_POINTS
                || !digest_is_present(&cloud.envelope_sha256)
                || !digest_is_present(&cloud.body_points_sha256)
            {
                return Err(CorrectionRejectReason::InvalidRegisteredCloudProof);
            }
            let odom = identity.correlated_odom;
            if odom.source_sequence == 0
                || odom.source_timestamp_ns > cloud.source_timestamp_ns
                || !digest_is_present(&odom.pose_sha256)
            {
                return Err(CorrectionRejectReason::InvalidCorrelatedOdomProof);
            }
            let map = identity.committed_map;
            let map_timestamp_is_valid = match identity.backend {
                CorrectionCandidateBackend::Relocalization => {
                    map.source_timestamp_ns <= cloud.source_timestamp_ns
                }
                CorrectionCandidateBackend::LibraryPgo | CorrectionCandidateBackend::NativePgo => {
                    map.source_timestamp_ns == cloud.source_timestamp_ns
                }
            };
            if !map_timestamp_is_valid
                || map.width == 0
                || map.height == 0
                || map.width > u64::from(u32::MAX)
                || map.height > u64::from(u32::MAX)
                || map.width.checked_mul(map.height).is_none()
                || !map.resolution_m.is_finite()
                || map.resolution_m <= 0.0
                || !map.origin_x_m.is_finite()
                || !map.origin_y_m.is_finite()
                || !digest_is_present(&map.payload_sha256)
                || !digest_is_present(&map.boundary_sha256)
            {
                return Err(CorrectionRejectReason::InvalidCommittedMapProof);
            }
            if identity.source_timestamp_ns != cloud.source_timestamp_ns {
                return Err(CorrectionRejectReason::InvalidRegisteredCloudProof);
            }
            validate_identity_string("map_frame_id", &identity.map_frame_id)?;
            validate_identity_string("odom_frame_id", &identity.odom_frame_id)?;
            validate_identity_string("body_frame_id", &identity.body_frame_id)?;
            validate_identity_string("points_frame_id", &identity.points_frame_id)?;
            if identity.map_frame_id == identity.odom_frame_id
                || identity.map_frame_id == identity.body_frame_id
                || identity.odom_frame_id == identity.body_frame_id
            {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "map, odom, and body frames must be distinct",
                ));
            }
            if identity.points_frame_id != identity.body_frame_id {
                return Err(CorrectionRejectReason::UnsupportedPointsFrame);
            }
            if map.boundary_sha256 != expected_map_boundary_sha256(&identity.map_frame_id, &map)? {
                return Err(CorrectionRejectReason::MapBoundaryHashMismatch);
            }
            if identity.correction_lease.boot_nonce == 0
                || identity.correction_lease.generation == 0
            {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "correction lease identity must be complete",
                ));
            }
            if identity.runtime_generation > i64::MAX as u64
                || identity.backend_generation > i64::MAX as u64
                || identity.candidate_sequence > i64::MAX as u64
                || identity.mapping_session_generation > i64::MAX as u64
                || identity.map_frame_generation > i64::MAX as u64
                || identity.mapping_commit_id.sequence > i64::MAX as u64
                || identity.occupancy_revision > i64::MAX as u64
                || identity.cloud_revision > i64::MAX as u64
                || cloud.source_sequence > i64::MAX as u64
                || cloud.source_timestamp_ns > i64::MAX as u64
                || cloud.point_count > i64::MAX as u64
                || odom.source_sequence > i64::MAX as u64
                || odom.source_timestamp_ns > i64::MAX as u64
                || map.source_timestamp_ns > i64::MAX as u64
                || map.width > i64::MAX as u64
                || map.height > i64::MAX as u64
                || identity.source_timestamp_ns > i64::MAX as u64
                || identity.correction_lease.generation > i64::MAX as u64
            {
                return Err(CorrectionRejectReason::InvalidIdentity(
                    "candidate integer exceeds positive int64 wire range",
                ));
            }

            if self
                .transform
                .translation_m
                .iter()
                .chain(self.transform.rotation_xyzw.iter())
                .any(|value| !value.is_finite())
            {
                return Err(CorrectionRejectReason::NonFiniteTransform);
            }
            if self
                .transform
                .translation_m
                .iter()
                .any(|value| value.abs() > MAX_TRANSLATION_ABS_METERS)
            {
                return Err(CorrectionRejectReason::TranslationOutOfBounds);
            }
            let norm_squared = self
                .transform
                .rotation_xyzw
                .iter()
                .map(|value| value * value)
                .sum::<f64>();
            if (norm_squared - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
                return Err(CorrectionRejectReason::InvalidRotation);
            }
            Ok(())
        }

        fn encode_body_prefix(&self) -> Result<Vec<u8>, CorrectionRejectReason> {
            let identity = &self.identity;
            let mut out = Vec::with_capacity(512);
            out.extend_from_slice(&CORRECTION_SCHEMA_VERSION_V1.to_be_bytes());
            out.extend_from_slice(&identity.backend.wire_code().to_be_bytes());
            out.extend_from_slice(&identity.transform_semantics.wire_code().to_be_bytes());
            encode_positive_i64(identity.runtime_generation, &mut out)?;
            encode_positive_i64(identity.backend_generation, &mut out)?;
            encode_positive_i64(identity.candidate_sequence, &mut out)?;
            encode_lcm_string(&identity.mapping_session_id, &mut out)?;
            encode_positive_i64(identity.mapping_session_generation, &mut out)?;
            encode_positive_i64(identity.map_frame_generation, &mut out)?;
            out.extend_from_slice(&identity.mapping_commit_id.boot_nonce.to_be_bytes());
            encode_positive_i64(identity.mapping_commit_id.sequence, &mut out)?;
            encode_positive_i64(identity.occupancy_revision, &mut out)?;
            encode_positive_i64(identity.cloud_revision, &mut out)?;
            encode_nonnegative_i64(identity.committed_map.source_timestamp_ns, &mut out)?;
            encode_positive_i64(identity.registered_cloud.source_sequence, &mut out)?;
            encode_nonnegative_i64(identity.registered_cloud.source_timestamp_ns, &mut out)?;
            encode_positive_i64(identity.registered_cloud.point_count, &mut out)?;
            out.extend_from_slice(&identity.registered_cloud.envelope_sha256);
            out.extend_from_slice(&identity.registered_cloud.body_points_sha256);
            encode_positive_i64(identity.correlated_odom.source_sequence, &mut out)?;
            encode_nonnegative_i64(identity.correlated_odom.source_timestamp_ns, &mut out)?;
            out.extend_from_slice(&identity.correlated_odom.pose_sha256);
            encode_positive_i64(identity.committed_map.width, &mut out)?;
            encode_positive_i64(identity.committed_map.height, &mut out)?;
            out.extend_from_slice(&identity.committed_map.resolution_m.to_be_bytes());
            out.extend_from_slice(&identity.committed_map.origin_x_m.to_be_bytes());
            out.extend_from_slice(&identity.committed_map.origin_y_m.to_be_bytes());
            out.extend_from_slice(&identity.committed_map.payload_sha256);
            out.extend_from_slice(&identity.committed_map.boundary_sha256);
            encode_nonnegative_i64(identity.source_timestamp_ns, &mut out)?;
            encode_lcm_string(&identity.map_frame_id, &mut out)?;
            encode_lcm_string(&identity.odom_frame_id, &mut out)?;
            encode_lcm_string(&identity.body_frame_id, &mut out)?;
            encode_lcm_string(&identity.points_frame_id, &mut out)?;
            out.extend_from_slice(&identity.correction_lease.boot_nonce.to_be_bytes());
            encode_positive_i64(identity.correction_lease.generation, &mut out)?;
            for value in self.transform.translation_m {
                out.extend_from_slice(&value.to_be_bytes());
            }
            for value in self.transform.rotation_xyzw {
                out.extend_from_slice(&value.to_be_bytes());
            }
            Ok(out)
        }

        /// Encode the generated LCM body without its top-level fingerprint.
        ///
        /// This is the only encoding accepted by `RawLcmFrameBuilder::payload`;
        /// `nuwax-transport` prepends the registered fingerprint exactly once.
        pub(crate) fn encode_body(&self) -> Result<Vec<u8>, CorrectionWireError> {
            self.validate_integrity()
                .map_err(|reason| CorrectionWireError::Invalid(reason.to_string()))?;
            let mut out = self
                .encode_body_prefix()
                .map_err(|reason| CorrectionWireError::Invalid(reason.to_string()))?;
            out.extend_from_slice(&self.payload_sha256);
            if out.len() > MAX_WIRE_PAYLOAD_BYTES - CORRECTION_CANDIDATE_FINGERPRINT_V1.len() {
                return Err(CorrectionWireError::Invalid(
                    "correction candidate exceeds wire size bound".into(),
                ));
            }
            Ok(out)
        }

        /// Encode a complete generated LCM message: one fingerprint plus body.
        pub(crate) fn encode_wire(&self) -> Result<Vec<u8>, CorrectionWireError> {
            let body = self.encode_body()?;
            let mut out =
                Vec::with_capacity(CORRECTION_CANDIDATE_FINGERPRINT_V1.len() + body.len());
            out.extend_from_slice(&CORRECTION_CANDIDATE_FINGERPRINT_V1);
            out.extend_from_slice(&body);
            Ok(out)
        }

        pub(crate) fn decode_wire(encoded: &[u8]) -> Result<Self, CorrectionWireError> {
            if encoded.len() > MAX_WIRE_PAYLOAD_BYTES {
                return Err(CorrectionWireError::Invalid(
                    "correction candidate exceeds wire size bound".into(),
                ));
            }
            let (fingerprint, body) = encoded
                .split_at_checked(CORRECTION_CANDIDATE_FINGERPRINT_V1.len())
                .ok_or(CorrectionWireError::Truncated {
                    need: CORRECTION_CANDIDATE_FINGERPRINT_V1.len(),
                    have: encoded.len(),
                })?;
            if fingerprint != CORRECTION_CANDIDATE_FINGERPRINT_V1 {
                return Err(CorrectionWireError::Invalid(
                    "correction candidate fingerprint mismatch".into(),
                ));
            }
            Self::decode_body(body)
        }

        /// Decode a generated LCM body after the transport checked and stripped
        /// the single top-level fingerprint.
        pub(crate) fn decode_body(encoded: &[u8]) -> Result<Self, CorrectionWireError> {
            if encoded.len() > MAX_WIRE_PAYLOAD_BYTES - CORRECTION_CANDIDATE_FINGERPRINT_V1.len() {
                return Err(CorrectionWireError::Invalid(
                    "correction candidate exceeds wire size bound".into(),
                ));
            }
            let mut cursor = WireCursor::new(encoded);
            let schema_version = cursor.read_i32()?;
            if schema_version != CORRECTION_SCHEMA_VERSION_V1 {
                return Err(CorrectionWireError::Invalid(format!(
                    "unsupported correction schema version {schema_version}"
                )));
            }
            let backend = CorrectionCandidateBackend::from_wire_code(cursor.read_i32()?)?;
            let transform_semantics =
                CorrectionTransformSemantics::from_wire_code(cursor.read_i32()?)?;
            let runtime_generation = cursor.read_positive_u64("runtime_generation")?;
            let backend_generation = cursor.read_positive_u64("backend_generation")?;
            let candidate_sequence = cursor.read_positive_u64("candidate_sequence")?;
            let mapping_session_id = cursor.read_lcm_string("mapping_session_id")?;
            let mapping_session_generation =
                cursor.read_positive_u64("mapping_session_generation")?;
            let map_frame_generation = cursor.read_positive_u64("map_frame_generation")?;
            let mapping_commit_id = MappingCommitId {
                boot_nonce: u128::from_be_bytes(cursor.read_array::<16>()?),
                sequence: cursor.read_positive_u64("mapping_commit_sequence")?,
            };
            let occupancy_revision = cursor.read_positive_u64("occupancy_revision")?;
            let cloud_revision = cursor.read_positive_u64("cloud_revision")?;
            let committed_map_source_timestamp_ns =
                cursor.read_nonnegative_u64("committed_map_source_timestamp_ns")?;
            let registered_cloud = RegisteredCloudProof {
                source_sequence: cursor.read_positive_u64("registered_cloud_source_sequence")?,
                source_timestamp_ns: cursor
                    .read_nonnegative_u64("registered_cloud_source_timestamp_ns")?,
                point_count: cursor.read_positive_u64("registered_cloud_point_count")?,
                envelope_sha256: cursor.read_array::<32>()?,
                body_points_sha256: cursor.read_array::<32>()?,
            };
            let correlated_odom = CorrelatedOdomProof {
                source_sequence: cursor.read_positive_u64("correlated_odom_source_sequence")?,
                source_timestamp_ns: cursor
                    .read_nonnegative_u64("correlated_odom_source_timestamp_ns")?,
                pose_sha256: cursor.read_array::<32>()?,
            };
            let committed_map = CommittedMapProof {
                source_timestamp_ns: committed_map_source_timestamp_ns,
                width: cursor.read_positive_u64("committed_map_width")?,
                height: cursor.read_positive_u64("committed_map_height")?,
                resolution_m: cursor.read_f64()?,
                origin_x_m: cursor.read_f64()?,
                origin_y_m: cursor.read_f64()?,
                payload_sha256: cursor.read_array::<32>()?,
                boundary_sha256: cursor.read_array::<32>()?,
            };
            let source_timestamp_ns = cursor.read_nonnegative_u64("source_timestamp_ns")?;
            let map_frame_id = cursor.read_lcm_string("map_frame_id")?;
            let odom_frame_id = cursor.read_lcm_string("odom_frame_id")?;
            let body_frame_id = cursor.read_lcm_string("body_frame_id")?;
            let points_frame_id = cursor.read_lcm_string("points_frame_id")?;
            let correction_lease = CorrectionLeaseId {
                boot_nonce: u128::from_be_bytes(cursor.read_array::<16>()?),
                generation: cursor.read_positive_u64("correction_lease_generation")?,
            };
            let mut translation_m = [0.0; 3];
            for value in &mut translation_m {
                *value = cursor.read_f64()?;
            }
            let mut rotation_xyzw = [0.0; 4];
            for value in &mut rotation_xyzw {
                *value = cursor.read_f64()?;
            }
            let payload_sha256 = cursor.read_array::<32>()?;
            cursor.finish()?;
            let candidate = Self {
                identity: CorrectionCandidateIdentity {
                    backend,
                    transform_semantics,
                    runtime_generation,
                    backend_generation,
                    candidate_sequence,
                    mapping_session_id,
                    mapping_session_generation,
                    map_frame_generation,
                    mapping_commit_id,
                    occupancy_revision,
                    cloud_revision,
                    registered_cloud,
                    correlated_odom,
                    committed_map,
                    source_timestamp_ns,
                    map_frame_id,
                    odom_frame_id,
                    body_frame_id,
                    points_frame_id,
                    correction_lease,
                },
                transform: CorrectionTransform {
                    translation_m,
                    rotation_xyzw,
                },
                payload_sha256,
            };
            candidate
                .validate_integrity()
                .map_err(|reason| CorrectionWireError::Invalid(reason.to_string()))?;
            Ok(candidate)
        }
    }

    fn validate_identity_string(
        field: &'static str,
        value: &str,
    ) -> Result<(), CorrectionRejectReason> {
        if value.is_empty()
            || value.trim() != value
            || value.len() > MAX_IDENTITY_STRING_BYTES
            || value.contains('\0')
        {
            return Err(CorrectionRejectReason::InvalidIdentity(field));
        }
        Ok(())
    }

    fn encode_positive_i64(value: u64, out: &mut Vec<u8>) -> Result<(), CorrectionRejectReason> {
        if value == 0 {
            return Err(CorrectionRejectReason::InvalidIdentity(
                "positive wire integer was zero",
            ));
        }
        encode_nonnegative_i64(value, out)
    }

    fn encode_nonnegative_i64(value: u64, out: &mut Vec<u8>) -> Result<(), CorrectionRejectReason> {
        let value = i64::try_from(value).map_err(|_| {
            CorrectionRejectReason::InvalidIdentity("candidate integer exceeds int64 wire range")
        })?;
        out.extend_from_slice(&value.to_be_bytes());
        Ok(())
    }

    fn encode_lcm_string(value: &str, out: &mut Vec<u8>) -> Result<(), CorrectionRejectReason> {
        validate_identity_string("wire string", value)?;
        let encoded_len = value
            .len()
            .checked_add(1)
            .and_then(|length| i32::try_from(length).ok())
            .ok_or(CorrectionRejectReason::InvalidIdentity(
                "wire string length overflow",
            ))?;
        out.extend_from_slice(&encoded_len.to_be_bytes());
        out.extend_from_slice(value.as_bytes());
        out.push(0);
        Ok(())
    }

    fn digest_is_present(digest: &[u8; 32]) -> bool {
        digest.iter().any(|byte| *byte != 0)
    }

    pub(crate) fn expected_map_boundary_sha256(
        map_frame_id: &str,
        proof: &CommittedMapProof,
    ) -> Result<[u8; 32], CorrectionRejectReason> {
        let mut canonical = b"nuwax.correction.committed-map-boundary.v1\0".to_vec();
        encode_lcm_string(map_frame_id, &mut canonical)?;
        encode_positive_i64(proof.width, &mut canonical)?;
        encode_positive_i64(proof.height, &mut canonical)?;
        canonical.extend_from_slice(&proof.resolution_m.to_be_bytes());
        canonical.extend_from_slice(&proof.origin_x_m.to_be_bytes());
        canonical.extend_from_slice(&proof.origin_y_m.to_be_bytes());
        Ok(sha256_bytes(&canonical))
    }

    pub(crate) fn body_points_sha256(
        points: &[Point3<f64>],
    ) -> Result<[u8; 32], CorrectionOwnerError> {
        let point_count = u64::try_from(points.len())
            .map_err(|_| CorrectionOwnerError::InvalidRegisteredCloudProof)?;
        if point_count == 0 || point_count > MAX_REGISTERED_CLOUD_POINTS {
            return Err(CorrectionOwnerError::InvalidRegisteredCloudProof);
        }
        let mut canonical = b"nuwax.correction.body-points.v1\0".to_vec();
        canonical.reserve(8 + points.len() * 24);
        canonical.extend_from_slice(&point_count.to_be_bytes());
        for point in points {
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err(CorrectionOwnerError::InvalidRegisteredCloudProof);
            }
            canonical.extend_from_slice(&point.x.to_be_bytes());
            canonical.extend_from_slice(&point.y.to_be_bytes());
            canonical.extend_from_slice(&point.z.to_be_bytes());
        }
        Ok(sha256_bytes(&canonical))
    }

    pub(crate) fn odom_pose_sha256(
        pose: &Isometry3<f64>,
    ) -> Result<[u8; 32], CorrectionOwnerError> {
        let transform = CorrectionTransform::from_isometry(pose);
        if transform
            .translation_m
            .iter()
            .chain(transform.rotation_xyzw.iter())
            .any(|value| !value.is_finite())
        {
            return Err(CorrectionOwnerError::InvalidCorrelatedOdomProof);
        }
        let mut canonical = b"nuwax.correction.odom-from-body.v1\0".to_vec();
        canonical.reserve(56);
        for value in transform.translation_m {
            canonical.extend_from_slice(&value.to_be_bytes());
        }
        for value in transform.rotation_xyzw {
            canonical.extend_from_slice(&value.to_be_bytes());
        }
        Ok(sha256_bytes(&canonical))
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) enum CorrectionRejectReason {
        Stopped,
        InvalidIdentity(&'static str),
        NonFiniteTransform,
        TranslationOutOfBounds,
        InvalidRotation,
        PayloadHashMismatch,
        BackendMismatch,
        RuntimeGenerationMismatch,
        BackendGenerationMismatch,
        CorrectionLeaseMismatch,
        MappingSessionMismatch,
        MappingSessionGenerationMismatch,
        MapFrameGenerationMismatch,
        MappingCommitMismatch,
        OccupancyRevisionMismatch,
        CloudRevisionMismatch,
        RegisteredCloudProofMismatch,
        CorrelatedOdomProofMismatch,
        CommittedMapProofMismatch,
        TransformSemanticsMismatch,
        FrameMismatch,
        InvalidRegisteredCloudProof,
        InvalidCorrelatedOdomProof,
        InvalidCommittedMapProof,
        UnsupportedPointsFrame,
        MapBoundaryHashMismatch,
        NoStagedFrame,
        ReplayConflict,
        ApplyInFlight,
        OutOfOrderCandidate,
        SourceTimestampRegressed,
    }

    impl std::fmt::Display for CorrectionRejectReason {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                Self::InvalidIdentity(detail) => write!(formatter, "invalid identity: {detail}"),
                other => write!(formatter, "{other:?}"),
            }
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) enum CorrectionDisposition {
        Prepared,
        Applied,
        AlreadyApplied,
        Rejected(CorrectionRejectReason),
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) struct CorrectionReceipt {
        pub(crate) candidate_sequence: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) payload_sha256: [u8; 32],
        pub(crate) disposition: CorrectionDisposition,
    }

    #[derive(Clone, Debug, PartialEq)]
    pub(crate) struct CorrectionBoundary {
        pub(crate) backend: CorrectionCandidateBackend,
        pub(crate) transform_semantics: CorrectionTransformSemantics,
        pub(crate) runtime_generation: u64,
        pub(crate) backend_generation: u64,
        pub(crate) mapping_session_id: String,
        pub(crate) mapping_session_generation: u64,
        pub(crate) map_frame_generation: u64,
        pub(crate) mapping_commit_id: MappingCommitId,
        pub(crate) occupancy_revision: u64,
        pub(crate) cloud_revision: u64,
        pub(crate) registered_cloud: RegisteredCloudProof,
        pub(crate) correlated_odom: CorrelatedOdomProof,
        pub(crate) committed_map: CommittedMapProof,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) map_frame_id: String,
        pub(crate) odom_frame_id: String,
        pub(crate) body_frame_id: String,
        pub(crate) points_frame_id: String,
        pub(crate) correction_lease: CorrectionLeaseId,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct ReplayCursor {
        candidate_sequence: u64,
        source_timestamp_ns: u64,
        payload_sha256: [u8; 32],
    }

    #[derive(Default)]
    pub(crate) struct CorrectionCoordinator {
        stopped: bool,
        pending: Option<ReplayCursor>,
        last_applied: Option<ReplayCursor>,
    }

    impl CorrectionCoordinator {
        pub(crate) fn prepare(
            &mut self,
            candidate: &CorrectionCandidate,
            boundary: &CorrectionBoundary,
        ) -> CorrectionReceipt {
            let cursor = ReplayCursor {
                candidate_sequence: candidate.identity.candidate_sequence,
                source_timestamp_ns: candidate.identity.source_timestamp_ns,
                payload_sha256: candidate.payload_sha256,
            };
            let disposition = self.prepare_inner(candidate, boundary, cursor);
            if disposition == CorrectionDisposition::Prepared {
                self.pending = Some(cursor);
            }
            CorrectionReceipt {
                candidate_sequence: candidate.identity.candidate_sequence,
                source_timestamp_ns: candidate.identity.source_timestamp_ns,
                payload_sha256: candidate.payload_sha256,
                disposition,
            }
        }

        fn prepare_inner(
            &self,
            candidate: &CorrectionCandidate,
            boundary: &CorrectionBoundary,
            cursor: ReplayCursor,
        ) -> CorrectionDisposition {
            if self.stopped {
                return CorrectionDisposition::Rejected(CorrectionRejectReason::Stopped);
            }
            if let Err(reason) = candidate.validate_integrity() {
                return CorrectionDisposition::Rejected(reason);
            }
            let identity = &candidate.identity;
            let mismatch = if identity.backend != boundary.backend {
                Some(CorrectionRejectReason::BackendMismatch)
            } else if identity.transform_semantics != boundary.transform_semantics {
                Some(CorrectionRejectReason::TransformSemanticsMismatch)
            } else if identity.runtime_generation != boundary.runtime_generation {
                Some(CorrectionRejectReason::RuntimeGenerationMismatch)
            } else if identity.backend_generation != boundary.backend_generation {
                Some(CorrectionRejectReason::BackendGenerationMismatch)
            } else if identity.correction_lease != boundary.correction_lease {
                Some(CorrectionRejectReason::CorrectionLeaseMismatch)
            } else if identity.mapping_session_id != boundary.mapping_session_id {
                Some(CorrectionRejectReason::MappingSessionMismatch)
            } else if identity.mapping_session_generation != boundary.mapping_session_generation {
                Some(CorrectionRejectReason::MappingSessionGenerationMismatch)
            } else if identity.map_frame_generation != boundary.map_frame_generation {
                Some(CorrectionRejectReason::MapFrameGenerationMismatch)
            } else if identity.mapping_commit_id != boundary.mapping_commit_id {
                Some(CorrectionRejectReason::MappingCommitMismatch)
            } else if identity.occupancy_revision != boundary.occupancy_revision {
                Some(CorrectionRejectReason::OccupancyRevisionMismatch)
            } else if identity.cloud_revision != boundary.cloud_revision {
                Some(CorrectionRejectReason::CloudRevisionMismatch)
            } else if identity.registered_cloud != boundary.registered_cloud {
                Some(CorrectionRejectReason::RegisteredCloudProofMismatch)
            } else if identity.correlated_odom != boundary.correlated_odom {
                Some(CorrectionRejectReason::CorrelatedOdomProofMismatch)
            } else if identity.committed_map != boundary.committed_map
                || identity.source_timestamp_ns != boundary.source_timestamp_ns
            {
                Some(CorrectionRejectReason::CommittedMapProofMismatch)
            } else if identity.map_frame_id != boundary.map_frame_id
                || identity.odom_frame_id != boundary.odom_frame_id
                || identity.body_frame_id != boundary.body_frame_id
                || identity.points_frame_id != boundary.points_frame_id
            {
                Some(CorrectionRejectReason::FrameMismatch)
            } else {
                None
            };
            if let Some(reason) = mismatch {
                return CorrectionDisposition::Rejected(reason);
            }
            if let Some(applied) = self.last_applied {
                if cursor.candidate_sequence == applied.candidate_sequence {
                    return if cursor.payload_sha256 == applied.payload_sha256 {
                        CorrectionDisposition::AlreadyApplied
                    } else {
                        CorrectionDisposition::Rejected(CorrectionRejectReason::ReplayConflict)
                    };
                }
                if cursor.candidate_sequence < applied.candidate_sequence {
                    return CorrectionDisposition::Rejected(
                        CorrectionRejectReason::OutOfOrderCandidate,
                    );
                }
                if cursor.source_timestamp_ns <= applied.source_timestamp_ns {
                    return CorrectionDisposition::Rejected(
                        CorrectionRejectReason::SourceTimestampRegressed,
                    );
                }
            }
            if let Some(pending) = self.pending {
                if cursor == pending {
                    return CorrectionDisposition::Prepared;
                }
                return if cursor.candidate_sequence == pending.candidate_sequence {
                    CorrectionDisposition::Rejected(CorrectionRejectReason::ReplayConflict)
                } else {
                    CorrectionDisposition::Rejected(CorrectionRejectReason::ApplyInFlight)
                };
            }
            CorrectionDisposition::Prepared
        }

        pub(crate) fn commit_applied(
            &mut self,
            prepared: &CorrectionReceipt,
        ) -> Result<CorrectionReceipt, CorrectionTransitionError> {
            let cursor = ReplayCursor::from_prepared(prepared)?;
            if self.stopped {
                return Err(CorrectionTransitionError::Stopped);
            }
            if self.pending != Some(cursor) {
                return Err(CorrectionTransitionError::ReceiptMismatch);
            }
            self.pending = None;
            self.last_applied = Some(cursor);
            Ok(CorrectionReceipt {
                candidate_sequence: cursor.candidate_sequence,
                source_timestamp_ns: cursor.source_timestamp_ns,
                payload_sha256: cursor.payload_sha256,
                disposition: CorrectionDisposition::Applied,
            })
        }

        pub(crate) fn abort(
            &mut self,
            prepared: &CorrectionReceipt,
        ) -> Result<(), CorrectionTransitionError> {
            let cursor = ReplayCursor::from_prepared(prepared)?;
            if self.stopped {
                return Err(CorrectionTransitionError::Stopped);
            }
            if self.pending != Some(cursor) {
                return Err(CorrectionTransitionError::ReceiptMismatch);
            }
            self.pending = None;
            Ok(())
        }

        pub(crate) fn stop(&mut self) {
            self.stopped = true;
            self.pending = None;
        }
    }

    impl ReplayCursor {
        fn from_prepared(receipt: &CorrectionReceipt) -> Result<Self, CorrectionTransitionError> {
            if receipt.disposition != CorrectionDisposition::Prepared {
                return Err(CorrectionTransitionError::NotPrepared);
            }
            Ok(Self {
                candidate_sequence: receipt.candidate_sequence,
                source_timestamp_ns: receipt.source_timestamp_ns,
                payload_sha256: receipt.payload_sha256,
            })
        }
    }

    #[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
    pub(crate) enum CorrectionTransitionError {
        #[error("correction coordinator is stopped")]
        Stopped,
        #[error("correction receipt was not prepared")]
        NotPrepared,
        #[error("correction receipt does not match the pending candidate")]
        ReceiptMismatch,
    }

    #[cfg_attr(test, allow(dead_code))]
    #[derive(Clone, Debug)]
    pub(crate) enum CorrectionOwnerSpec {
        None,
        LibraryPgo {
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
            pipeline: LoopClosurePipelineConfig,
        },
        NativePgo {
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
            executable: std::path::PathBuf,
        },
        Relocalization {
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
        },
        Deferred {
            backend: CorrectionBackend,
        },
    }

    impl CorrectionOwnerSpec {
        pub(crate) fn preflight(
            backend: CorrectionBackend,
            frames: &ValidatedFrameBindings,
        ) -> Result<Self, String> {
            let (map_frame_id, odom_frame_id, body_frame_id) = frames.as_parts();
            Self::preflight_for_frames(backend, map_frame_id, odom_frame_id, body_frame_id)
        }

        fn preflight_for_frames(
            backend: CorrectionBackend,
            map_frame_id: &str,
            odom_frame_id: &str,
            body_frame_id: &str,
        ) -> Result<Self, String> {
            validate_preflight_frame("map", map_frame_id)?;
            validate_preflight_frame("odom", odom_frame_id)?;
            validate_preflight_frame("body", body_frame_id)?;
            if [map_frame_id, odom_frame_id, body_frame_id]
                .into_iter()
                .collect::<std::collections::HashSet<_>>()
                .len()
                != 3
            {
                return Err(
                    "CORRECTION_PREFLIGHT_FAILED map, odom, and body frames must be distinct"
                        .into(),
                );
            }
            match backend {
                CorrectionBackend::None => Ok(Self::None),
                CorrectionBackend::LibraryPgo => {
                    let pipeline = LoopClosurePipelineConfig::default();
                    LoopClosureCoordinator::new(pipeline).map_err(|error| {
                        format!("CORRECTION_PREFLIGHT_FAILED library_pgo config: {error}")
                    })?;
                    Ok(Self::LibraryPgo {
                        map_frame_id: map_frame_id.to_owned(),
                        odom_frame_id: odom_frame_id.to_owned(),
                        body_frame_id: body_frame_id.to_owned(),
                        pipeline,
                    })
                }
                CorrectionBackend::NativePgo => {
                    let executable = nuwax_nav_pgo_bridge::native_executable();
                    validate_native_pgo_executable(&executable)?;
                    Ok(Self::NativePgo {
                        map_frame_id: map_frame_id.to_owned(),
                        odom_frame_id: odom_frame_id.to_owned(),
                        body_frame_id: body_frame_id.to_owned(),
                        executable,
                    })
                }
                CorrectionBackend::Relocalization => Ok(Self::Relocalization {
                    map_frame_id: map_frame_id.to_owned(),
                    odom_frame_id: odom_frame_id.to_owned(),
                    body_frame_id: body_frame_id.to_owned(),
                }),
            }
        }

        pub(crate) const fn candidate_backend(&self) -> CorrectionCandidateBackend {
            match self {
                Self::None | Self::LibraryPgo { .. } => CorrectionCandidateBackend::LibraryPgo,
                Self::NativePgo { .. } => CorrectionCandidateBackend::NativePgo,
                Self::Relocalization { .. } => CorrectionCandidateBackend::Relocalization,
                Self::Deferred {
                    backend: CorrectionBackend::Relocalization,
                } => CorrectionCandidateBackend::Relocalization,
                Self::Deferred { .. } => CorrectionCandidateBackend::LibraryPgo,
            }
        }
    }

    fn validate_native_pgo_executable(path: &std::path::Path) -> Result<(), String> {
        let metadata = std::fs::metadata(path).map_err(|error| {
            format!(
                "CORRECTION_PREFLIGHT_FAILED native PGO executable {} is unavailable: {error}",
                path.display()
            )
        })?;
        if !metadata.is_file() {
            return Err(format!(
                "CORRECTION_PREFLIGHT_FAILED native PGO executable {} is not a file",
                path.display()
            ));
        }
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            if metadata.permissions().mode() & 0o111 == 0 {
                return Err(format!(
                    "CORRECTION_PREFLIGHT_FAILED native PGO executable {} has no executable bit",
                    path.display()
                ));
            }
        }
        Ok(())
    }

    fn validate_preflight_frame(label: &str, value: &str) -> Result<(), String> {
        if value.is_empty()
            || value.trim() != value
            || value.len() > MAX_IDENTITY_STRING_BYTES
            || value.contains('\0')
        {
            return Err(format!("CORRECTION_PREFLIGHT_FAILED invalid {label} frame"));
        }
        Ok(())
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) struct CorrectionOwnerRuntimeIdentity {
        pub(crate) runtime_generation: u64,
        pub(crate) backend_generation: u64,
        pub(crate) correction_lease: CorrectionLeaseId,
    }

    /// Shared monotonic mode fence for the mapping/correction handoff.
    ///
    /// Even values mean that relocalization is inactive; odd values mean that
    /// one relocalization admission owns the mode.  Incrementing on both
    /// edges gives mapping work a stable provenance token, so a completion
    /// that crossed an admission boundary can be aborted without touching the
    /// map.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct RelocalizationModeSnapshot {
        pub(crate) epoch: u64,
        pub(crate) active: bool,
    }

    #[derive(Default)]
    pub(crate) struct RelocalizationModeFence {
        state: AtomicU64,
    }

    impl RelocalizationModeFence {
        pub(crate) fn snapshot(&self) -> RelocalizationModeSnapshot {
            let state = self.state.load(Ordering::Acquire);
            RelocalizationModeSnapshot {
                epoch: state,
                active: state & 1 == 1,
            }
        }

        fn activate(&self) -> Result<RelocalizationModeSnapshot, &'static str> {
            let mut current = self.state.load(Ordering::Acquire);
            loop {
                if current & 1 == 1 {
                    return Err("RELOCALIZATION_MODE_ALREADY_ACTIVE");
                }
                if current >= u64::MAX - 1 {
                    return Err("RELOCALIZATION_MODE_EPOCH_EXHAUSTED");
                }
                let next = current + 1;
                match self.state.compare_exchange(
                    current,
                    next,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => {
                        return Ok(RelocalizationModeSnapshot {
                            epoch: next,
                            active: true,
                        })
                    }
                    Err(observed) => current = observed,
                }
            }
        }

        fn deactivate(&self) {
            let mut current = self.state.load(Ordering::Acquire);
            loop {
                if current & 1 == 0 {
                    return;
                }
                let next = current
                    .checked_add(1)
                    .expect("relocalization mode activation reserves a deactivation epoch");
                match self.state.compare_exchange(
                    current,
                    next,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => return,
                    Err(observed) => current = observed,
                }
            }
        }

        #[cfg(test)]
        pub(crate) fn activate_for_tests(&self) -> RelocalizationModeSnapshot {
            self.activate()
                .expect("test relocalization mode activation is monotonic")
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct CorrectionSessionIdentity {
        mapping_session_id: String,
        mapping_session_generation: u64,
        map_frame_generation: u64,
    }

    #[derive(Clone, Debug, PartialEq)]
    /// Immutable odometry identity and the exact pose used by library PGO.
    pub(crate) struct FrozenCorrelatedOdom {
        pub(crate) proof: CorrelatedOdomProof,
        pub(crate) odom_from_body: Isometry3<f64>,
    }

    #[derive(Clone, Debug)]
    /// One fully correlated input snapshot for a correction backend.
    ///
    /// `body_points` are accepted only when `points_frame_id` equals the
    /// configured body frame. The committed-map and registration evidence is
    /// identity metadata; it never changes the frame of the algorithm input.
    pub(crate) struct CorrectionFrameInput {
        pub(crate) mapping_session_id: String,
        pub(crate) mapping_session_generation: u64,
        pub(crate) mapping_commit_id: MappingCommitId,
        pub(crate) occupancy_revision: u64,
        pub(crate) cloud_revision: u64,
        pub(crate) map_frame_generation: u64,
        pub(crate) registered_cloud: RegisteredCloudProof,
        pub(crate) correlated_odom: FrozenCorrelatedOdom,
        pub(crate) committed_map: CommittedMapProof,
        pub(crate) committed_map_occupancy: Arc<[u8]>,
        pub(crate) points_frame_id: String,
        pub(crate) body_points: Vec<Point3<f64>>,
        /// Immutable canonical anchor captured before this frame was committed.
        pub(crate) base_anchor_map_from_odom: Isometry3<f64>,
        /// Fence epoch observed when this source frame crossed the LIO
        /// lifecycle admission boundary. A later epoch is a different mode
        /// generation and must not reuse this frame.
        pub(crate) frame_capture_relocalization_epoch: u64,
        /// Mode epoch captured by the producer.  Relocalization frames must
        /// carry the active epoch; ordinary correction backends leave this
        /// unset.
        pub(crate) relocalization_mode_epoch: Option<u64>,
    }

    pub(crate) struct LibraryPgoOwner {
        coordinator: LoopClosureCoordinator,
        pipeline: LoopClosurePipelineConfig,
        identity: CorrectionOwnerRuntimeIdentity,
        session: Option<CorrectionSessionIdentity>,
        map_frame_id: String,
        odom_frame_id: String,
        body_frame_id: String,
        next_candidate_sequence: u64,
        first_source_timestamp_ns: Option<u64>,
        last_source_timestamp_ns: Option<u64>,
        mapping_commit_boot_nonce: Option<u128>,
        last_commit_sequence: u64,
        last_occupancy_revision: u64,
        last_cloud_revision: u64,
    }

    /// Retained generation owner. It intentionally does not implement `Clone`.
    pub(crate) enum CorrectionOwnerHandle {
        None {
            stopped: bool,
        },
        LibraryPgo {
            stopped: bool,
            owner: Box<LibraryPgoOwner>,
        },
    }

    impl CorrectionOwnerHandle {
        pub(crate) fn start(
            spec: CorrectionOwnerSpec,
            identity: CorrectionOwnerRuntimeIdentity,
        ) -> Result<Self, CorrectionOwnerError> {
            validate_owner_identity(&identity)?;
            match spec {
                CorrectionOwnerSpec::None => Ok(Self::None { stopped: false }),
                CorrectionOwnerSpec::LibraryPgo {
                    map_frame_id,
                    odom_frame_id,
                    body_frame_id,
                    pipeline,
                } => Ok(Self::LibraryPgo {
                    stopped: false,
                    owner: Box::new(LibraryPgoOwner {
                        coordinator: LoopClosureCoordinator::new(pipeline)
                            .map_err(|error| CorrectionOwnerError::Pipeline(error.to_string()))?,
                        pipeline,
                        identity,
                        session: None,
                        map_frame_id,
                        odom_frame_id,
                        body_frame_id,
                        next_candidate_sequence: 0,
                        first_source_timestamp_ns: None,
                        last_source_timestamp_ns: None,
                        mapping_commit_boot_nonce: None,
                        last_commit_sequence: 0,
                        last_occupancy_revision: 0,
                        last_cloud_revision: 0,
                    }),
                }),
                CorrectionOwnerSpec::NativePgo { .. } => {
                    Err(CorrectionOwnerError::BackendDeferred("native_pgo"))
                }
                CorrectionOwnerSpec::Relocalization { .. } => {
                    Err(CorrectionOwnerError::BackendDeferred("relocalization"))
                }
                CorrectionOwnerSpec::Deferred { backend, .. } => {
                    Err(CorrectionOwnerError::BackendDeferred(backend.stable_name()))
                }
            }
        }

        pub(crate) fn submit_frame(
            &mut self,
            input: CorrectionFrameInput,
        ) -> Result<Option<CorrectionCandidate>, CorrectionOwnerError> {
            match self {
                Self::None { stopped: false } => {
                    let _ = input;
                    Ok(None)
                }
                Self::None { stopped: true } | Self::LibraryPgo { stopped: true, .. } => {
                    Err(CorrectionOwnerError::Stopped)
                }
                Self::LibraryPgo {
                    stopped: false,
                    owner,
                } => owner.submit_frame(input),
            }
        }

        pub(crate) fn stop(&mut self) {
            match self {
                Self::None { stopped } | Self::LibraryPgo { stopped, .. } => *stopped = true,
            }
        }

        pub(crate) fn shutdown(mut self) {
            self.stop();
        }
    }

    impl LibraryPgoOwner {
        fn submit_frame(
            &mut self,
            input: CorrectionFrameInput,
        ) -> Result<Option<CorrectionCandidate>, CorrectionOwnerError> {
            let session_transition = validate_frame_input(&input, self)?;
            if session_transition {
                self.coordinator = LoopClosureCoordinator::new(self.pipeline)
                    .map_err(|error| CorrectionOwnerError::Pipeline(error.to_string()))?;
                self.session = Some(CorrectionSessionIdentity {
                    mapping_session_id: input.mapping_session_id.clone(),
                    mapping_session_generation: input.mapping_session_generation,
                    map_frame_generation: input.map_frame_generation,
                });
                self.first_source_timestamp_ns = None;
                self.last_source_timestamp_ns = None;
                self.mapping_commit_boot_nonce = None;
                self.last_commit_sequence = 0;
                self.last_occupancy_revision = 0;
                self.last_cloud_revision = 0;
            }
            let origin = *self
                .first_source_timestamp_ns
                .get_or_insert(input.registered_cloud.source_timestamp_ns);
            let relative_timestamp = input
                .registered_cloud
                .source_timestamp_ns
                .checked_sub(origin)
                .ok_or(CorrectionOwnerError::SourceTimestampRegressed)?
                as f64
                / 1_000_000_000.0;
            let result = self
                .coordinator
                .process_frame(
                    relative_timestamp,
                    input.correlated_odom.odom_from_body,
                    &input.body_points,
                )
                .map_err(|error| CorrectionOwnerError::Pipeline(error.to_string()))?;
            self.last_source_timestamp_ns = Some(input.registered_cloud.source_timestamp_ns);
            self.mapping_commit_boot_nonce = Some(input.mapping_commit_id.boot_nonce);
            self.last_commit_sequence = input.mapping_commit_id.sequence;
            self.last_occupancy_revision = input.occupancy_revision;
            self.last_cloud_revision = input.cloud_revision;
            if !matches!(result, ProcessFrameResult::LoopClosed { .. }) {
                return Ok(None);
            }
            let correction = self
                .coordinator
                .corrections()
                .correction_at(relative_timestamp)
                .map_err(|error| CorrectionOwnerError::Pipeline(error.to_string()))?;
            let next_candidate_sequence = self
                .next_candidate_sequence
                .checked_add(1)
                .ok_or(CorrectionOwnerError::CandidateSequenceExhausted)?;
            let candidate = CorrectionCandidate::seal(
                correction_candidate_identity(
                    CorrectionCandidateBackend::LibraryPgo,
                    &self.identity,
                    next_candidate_sequence,
                    &self.map_frame_id,
                    &self.odom_frame_id,
                    &self.body_frame_id,
                    &input,
                ),
                CorrectionTransform::from_isometry(&correction),
            )
            .map_err(CorrectionOwnerError::CandidateRejected)?;
            self.next_candidate_sequence = next_candidate_sequence;
            Ok(Some(candidate))
        }
    }

    fn validate_owner_identity(
        identity: &CorrectionOwnerRuntimeIdentity,
    ) -> Result<(), CorrectionOwnerError> {
        if identity.runtime_generation == 0
            || identity.backend_generation == 0
            || identity.runtime_generation > i64::MAX as u64
            || identity.backend_generation > i64::MAX as u64
            || identity.correction_lease.boot_nonce == 0
            || identity.correction_lease.generation == 0
            || identity.correction_lease.generation > i64::MAX as u64
        {
            return Err(CorrectionOwnerError::InvalidOwnerIdentity);
        }
        Ok(())
    }

    fn validate_frame_input(
        input: &CorrectionFrameInput,
        owner: &LibraryPgoOwner,
    ) -> Result<bool, CorrectionOwnerError> {
        if input.mapping_commit_id.boot_nonce == 0
            || input.mapping_commit_id.sequence == 0
            || input.mapping_session_id.is_empty()
            || input.mapping_session_id.trim() != input.mapping_session_id
            || input.mapping_session_id.len() > MAX_IDENTITY_STRING_BYTES
            || input.mapping_session_id.contains('\0')
            || input.mapping_session_generation == 0
            || input.map_frame_generation == 0
            || input.mapping_session_generation > i64::MAX as u64
            || input.map_frame_generation > i64::MAX as u64
            || input.occupancy_revision == 0
            || input.cloud_revision == 0
            || input.mapping_commit_id.sequence > i64::MAX as u64
            || input.occupancy_revision > i64::MAX as u64
            || input.cloud_revision > i64::MAX as u64
        {
            return Err(CorrectionOwnerError::InvalidMappingIdentity);
        }
        if input.points_frame_id != owner.body_frame_id {
            return Err(CorrectionOwnerError::UnsupportedPointsFrame);
        }
        if !is_finite_isometry(&input.base_anchor_map_from_odom) {
            return Err(CorrectionOwnerError::InvalidBaseAnchor);
        }
        let point_count = u64::try_from(input.body_points.len())
            .map_err(|_| CorrectionOwnerError::InvalidRegisteredCloudProof)?;
        if input.registered_cloud.source_sequence == 0
            || input.registered_cloud.point_count != point_count
            || input.registered_cloud.point_count == 0
            || input.registered_cloud.point_count > MAX_REGISTERED_CLOUD_POINTS
            || input.registered_cloud.source_sequence > i64::MAX as u64
            || input.registered_cloud.source_timestamp_ns > i64::MAX as u64
            || !digest_is_present(&input.registered_cloud.envelope_sha256)
            || input.registered_cloud.body_points_sha256 != body_points_sha256(&input.body_points)?
        {
            return Err(CorrectionOwnerError::InvalidRegisteredCloudProof);
        }
        if input.correlated_odom.proof.source_sequence == 0
            || input.correlated_odom.proof.source_sequence > i64::MAX as u64
            || input.correlated_odom.proof.source_timestamp_ns > i64::MAX as u64
            || input.correlated_odom.proof.source_timestamp_ns
                > input.registered_cloud.source_timestamp_ns
            || input.correlated_odom.proof.pose_sha256
                != odom_pose_sha256(&input.correlated_odom.odom_from_body)?
        {
            return Err(CorrectionOwnerError::InvalidCorrelatedOdomProof);
        }
        let map = input.committed_map;
        let expected_map_cells = map
            .width
            .checked_mul(map.height)
            .and_then(|cells| usize::try_from(cells).ok());
        if map.source_timestamp_ns != input.registered_cloud.source_timestamp_ns
            || map.width == 0
            || map.height == 0
            || map.width > u64::from(u32::MAX)
            || map.height > u64::from(u32::MAX)
            || map.source_timestamp_ns > i64::MAX as u64
            || map.width.checked_mul(map.height).is_none()
            || !map.resolution_m.is_finite()
            || map.resolution_m <= 0.0
            || !map.origin_x_m.is_finite()
            || !map.origin_y_m.is_finite()
            || !digest_is_present(&map.payload_sha256)
            || expected_map_cells != Some(input.committed_map_occupancy.len())
            || map.payload_sha256 != sha256_bytes(input.committed_map_occupancy.as_ref())
            || map.boundary_sha256
                != expected_map_boundary_sha256(&owner.map_frame_id, &map)
                    .map_err(CorrectionOwnerError::CandidateRejected)?
        {
            return Err(CorrectionOwnerError::InvalidCommittedMapProof);
        }
        let session_transition = match owner.session.as_ref() {
            None => true,
            Some(session)
                if input.mapping_session_generation < session.mapping_session_generation =>
            {
                return Err(CorrectionOwnerError::StaleMappingSession);
            }
            Some(session)
                if input.mapping_session_generation == session.mapping_session_generation
                    && (input.mapping_session_id != session.mapping_session_id
                        || input.map_frame_generation != session.map_frame_generation) =>
            {
                return Err(CorrectionOwnerError::MappingSessionIdentityChanged);
            }
            Some(session)
                if input.mapping_session_generation == session.mapping_session_generation =>
            {
                false
            }
            Some(_) => true,
        };
        if !session_transition
            && owner
                .mapping_commit_boot_nonce
                .is_some_and(|expected| input.mapping_commit_id.boot_nonce != expected)
        {
            return Err(CorrectionOwnerError::MappingCommitNamespaceChanged);
        }
        if !session_transition
            && (input.mapping_commit_id.sequence <= owner.last_commit_sequence
                || input.occupancy_revision <= owner.last_occupancy_revision
                || input.cloud_revision <= owner.last_cloud_revision)
        {
            return Err(CorrectionOwnerError::OutOfOrderMappingInput);
        }
        if !session_transition
            && owner
                .last_source_timestamp_ns
                .is_some_and(|last| input.registered_cloud.source_timestamp_ns <= last)
        {
            return Err(CorrectionOwnerError::SourceTimestampRegressed);
        }
        Ok(session_transition)
    }

    #[derive(Debug, Error, PartialEq, Eq)]
    pub(crate) enum CorrectionOwnerError {
        #[error("correction backend `{0}` remains deferred until its production owner is wired")]
        BackendDeferred(&'static str),
        #[error("correction owner is stopped")]
        Stopped,
        #[error("invalid correction owner identity")]
        InvalidOwnerIdentity,
        #[error("invalid mapping identity")]
        InvalidMappingIdentity,
        #[error("correction input belongs to a stale mapping session")]
        StaleMappingSession,
        #[error("mapping session identity changed without a newer generation")]
        MappingSessionIdentityChanged,
        #[error("library PGO accepts only body-frame points")]
        UnsupportedPointsFrame,
        #[error("canonical base anchor is not a finite full-SE(3) transform")]
        InvalidBaseAnchor,
        #[error("registered cloud proof does not match the body-frame points")]
        InvalidRegisteredCloudProof,
        #[error("correlated odometry proof does not match the frozen odometry pose")]
        InvalidCorrelatedOdomProof,
        #[error("committed map proof is incomplete or inconsistent")]
        InvalidCommittedMapProof,
        #[error("mapping input is duplicated or out of order")]
        OutOfOrderMappingInput,
        #[error("mapping commit boot nonce changed within one correction owner")]
        MappingCommitNamespaceChanged,
        #[error("source timestamp regressed or was duplicated")]
        SourceTimestampRegressed,
        #[error("library PGO pipeline failed: {0}")]
        Pipeline(String),
        #[error("correction candidate sequence exhausted")]
        CandidateSequenceExhausted,
        #[error("correction candidate rejected: {0}")]
        CandidateRejected(CorrectionRejectReason),
    }

    fn is_finite_isometry(transform: &Isometry3<f64>) -> bool {
        let transform = CorrectionTransform::from_isometry(transform);
        transform
            .translation_m
            .iter()
            .chain(transform.rotation_xyzw.iter())
            .all(|value| value.is_finite())
    }

    #[derive(Debug, Error, PartialEq, Eq)]
    pub(crate) enum CorrectionWireError {
        #[error("truncated correction candidate: need {need} bytes, have {have}")]
        Truncated { need: usize, have: usize },
        #[error("invalid correction candidate wire: {0}")]
        Invalid(String),
        #[error("correction candidate has {0} trailing bytes")]
        Trailing(usize),
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) enum CorrectionSubmitDisposition {
        NoCandidate,
        Rejected {
            reason_code: i32,
            reason: String,
        },
        CandidateProcessed {
            candidate_sequence: u64,
            payload_sha256: [u8; 32],
            disposition: CorrectionDisposition,
        },
    }

    #[derive(Debug, Error)]
    pub(crate) enum CorrectionRuntimeError {
        #[error("correction admission is closed")]
        Stopped,
        #[error("correction ingress worker is unavailable")]
        IngressClosed,
        #[error("correction ingress response was canceled")]
        ResponseCanceled,
        #[error("correction candidate owner rejected the frame: {0}")]
        CandidateOwner(#[from] CorrectionOwnerError),
        #[error("native PGO owner failed: {0}")]
        NativePgo(String),
        #[error("correction private transport failed: {0}")]
        Transport(String),
        #[error("correction candidate apply failed: {0}")]
        Apply(String),
        #[error("correction worker join failed: {0}")]
        WorkerJoin(String),
        #[error("correction worker shutdown timed out: {0}")]
        WorkerStopTimeout(&'static str),
        #[error("correction shutdown failed: {0}")]
        Shutdown(String),
        #[error("canonical correction finalizer was poisoned: {0}")]
        FinalizerPoisoned(String),
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[repr(u8)]
    pub(crate) enum CanonicalOperationPhase {
        Pending = 0,
        Cancelled = 1,
        Claimed = 2,
        AckedOk = 3,
        StateCommitted = 4,
        CoordinatorFinalized = 5,
        AckedErr = 6,
        Poisoned = 7,
    }

    impl CanonicalOperationPhase {
        fn from_u8(value: u8) -> Self {
            match value {
                0 => Self::Pending,
                1 => Self::Cancelled,
                2 => Self::Claimed,
                3 => Self::AckedOk,
                4 => Self::StateCommitted,
                5 => Self::CoordinatorFinalized,
                6 => Self::AckedErr,
                7 => Self::Poisoned,
                _ => unreachable!("invalid canonical operation phase"),
            }
        }

        fn is_post_claim(self) -> bool {
            matches!(
                self,
                Self::Claimed
                    | Self::AckedOk
                    | Self::StateCommitted
                    | Self::CoordinatorFinalized
                    | Self::AckedErr
                    | Self::Poisoned
            )
        }
    }

    type CanonicalFailStopHook = Arc<dyn Fn(&str) + Send + Sync + 'static>;

    struct CanonicalWatchdog {
        disarm: std_mpsc::Sender<()>,
        worker: std::thread::JoinHandle<()>,
    }

    struct CanonicalOperationControlInner {
        phase: AtomicU8,
        phase_tx: watch::Sender<CanonicalOperationPhase>,
        poison_detail: parking_lot::Mutex<Option<String>>,
        known_finalized: AtomicBool,
        fail_stop_invoked: AtomicBool,
        fail_stop: CanonicalFailStopHook,
        watchdog: parking_lot::Mutex<Option<CanonicalWatchdog>>,
    }

    #[derive(Clone)]
    pub(crate) struct CanonicalOperationControl {
        inner: Arc<CanonicalOperationControlInner>,
    }

    impl CanonicalOperationControl {
        fn production() -> Self {
            Self::with_fail_stop(Arc::new(|detail| {
                tracing::error!(%detail, "canonical correction finalizer fail-stop");
                std::process::exit(7);
            }))
        }

        fn with_fail_stop(fail_stop: CanonicalFailStopHook) -> Self {
            let (phase_tx, _) = watch::channel(CanonicalOperationPhase::Pending);
            Self {
                inner: Arc::new(CanonicalOperationControlInner {
                    phase: AtomicU8::new(CanonicalOperationPhase::Pending as u8),
                    phase_tx,
                    poison_detail: parking_lot::Mutex::new(None),
                    known_finalized: AtomicBool::new(false),
                    fail_stop_invoked: AtomicBool::new(false),
                    fail_stop,
                    watchdog: parking_lot::Mutex::new(None),
                }),
            }
        }

        pub(crate) fn phase(&self) -> CanonicalOperationPhase {
            CanonicalOperationPhase::from_u8(self.inner.phase.load(Ordering::Acquire))
        }

        fn subscribe(&self) -> watch::Receiver<CanonicalOperationPhase> {
            self.inner.phase_tx.subscribe()
        }

        fn transition(
            &self,
            expected: CanonicalOperationPhase,
            next: CanonicalOperationPhase,
        ) -> Result<(), String> {
            self.inner
                .phase
                .compare_exchange(
                    expected as u8,
                    next as u8,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .map_err(|actual| {
                    format!(
                        "canonical operation phase mismatch: expected {expected:?}, actual {:?}",
                        CanonicalOperationPhase::from_u8(actual)
                    )
                })?;
            self.inner.phase_tx.send_replace(next);
            Ok(())
        }

        pub(crate) fn cancel_pending(&self) -> bool {
            self.transition(
                CanonicalOperationPhase::Pending,
                CanonicalOperationPhase::Cancelled,
            )
            .is_ok()
        }

        pub(crate) fn try_claim(&self, cancellation: &CorrectionRequestCancellation) -> bool {
            let deadline = Instant::now() + CANONICAL_FINALIZATION_DEADLINE;
            if !cancellation.try_begin_canonical_commit() {
                self.cancel_pending();
                return false;
            }
            if self
                .transition(
                    CanonicalOperationPhase::Pending,
                    CanonicalOperationPhase::Claimed,
                )
                .is_err()
            {
                self.poison("canonical claim lost its pending operation");
                return false;
            }
            self.start_watchdog_once(deadline);
            true
        }

        pub(crate) fn mark_acked_ok(&self) -> Result<(), String> {
            self.transition(
                CanonicalOperationPhase::Claimed,
                CanonicalOperationPhase::AckedOk,
            )
        }

        pub(crate) fn mark_acked_err(&self) -> Result<(), String> {
            self.transition(
                CanonicalOperationPhase::Claimed,
                CanonicalOperationPhase::AckedErr,
            )
        }

        pub(crate) fn mark_state_committed(&self) -> Result<(), String> {
            self.transition(
                CanonicalOperationPhase::AckedOk,
                CanonicalOperationPhase::StateCommitted,
            )
        }

        pub(crate) fn mark_coordinator_finalized(&self) -> Result<(), String> {
            self.transition(
                CanonicalOperationPhase::StateCommitted,
                CanonicalOperationPhase::CoordinatorFinalized,
            )?;
            self.inner.known_finalized.store(true, Ordering::Release);
            self.disarm_watchdog();
            Ok(())
        }

        fn finish_acked_err(&self) {
            self.inner.known_finalized.store(true, Ordering::Release);
            self.disarm_watchdog();
        }

        pub(crate) fn poison(&self, detail: impl Into<String>) {
            let detail = detail.into();
            loop {
                let phase = self.phase();
                if self.inner.known_finalized.load(Ordering::Acquire)
                    || matches!(
                        phase,
                        CanonicalOperationPhase::Cancelled
                            | CanonicalOperationPhase::CoordinatorFinalized
                    )
                {
                    return;
                }
                if phase == CanonicalOperationPhase::Poisoned {
                    break;
                }
                if self
                    .inner
                    .phase
                    .compare_exchange(
                        phase as u8,
                        CanonicalOperationPhase::Poisoned as u8,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok()
                {
                    *self.inner.poison_detail.lock() = Some(detail.clone());
                    self.inner
                        .phase_tx
                        .send_replace(CanonicalOperationPhase::Poisoned);
                    break;
                }
            }
            if !self.inner.fail_stop_invoked.swap(true, Ordering::AcqRel) {
                (self.inner.fail_stop)(&detail);
            }
        }

        fn poison_detail(&self) -> String {
            self.inner
                .poison_detail
                .lock()
                .clone()
                .unwrap_or_else(|| "canonical operation poisoned".to_owned())
        }

        fn is_known_finalized(&self) -> bool {
            self.inner.known_finalized.load(Ordering::Acquire)
        }

        #[cfg_attr(test, allow(dead_code))]
        fn start_handoff_watchdog(&self, deadline: Instant) {
            self.start_watchdog_once(deadline);
        }

        fn start_watchdog_once(&self, deadline: Instant) {
            let mut watchdog_slot = self.inner.watchdog.lock();
            if watchdog_slot.is_some() {
                return;
            }
            let (disarm, receiver) = std_mpsc::channel();
            let control = self.clone();
            let worker = std::thread::Builder::new()
                .name("canonical-finalizer-watchdog".to_owned())
                .spawn(move || {
                    let remaining = deadline.saturating_duration_since(Instant::now());
                    if matches!(
                        receiver.recv_timeout(remaining),
                        Err(std_mpsc::RecvTimeoutError::Timeout)
                    ) {
                        control
                            .poison("canonical finalization exceeded the 5s wall-clock deadline");
                    }
                })
                .expect("canonical finalizer watchdog thread must start");
            *watchdog_slot = Some(CanonicalWatchdog { disarm, worker });
        }

        fn disarm_watchdog(&self) {
            let Some(watchdog) = self.inner.watchdog.lock().take() else {
                return;
            };
            let _ = watchdog.disarm.send(());
            if watchdog.worker.thread().id() != std::thread::current().id() {
                let _ = watchdog.worker.join();
            }
        }
    }

    const REQUEST_ACTIVE: u8 = 0;
    const REQUEST_CANCELLED: u8 = 1;
    const REQUEST_COMMITTING: u8 = 2;
    const REQUEST_COMMITTING_CANCELLED: u8 = 3;
    const REQUEST_COMMITTED: u8 = 4;
    const REQUEST_COMMITTED_CANCELLED: u8 = 5;

    struct CorrectionRequestCancellationInner {
        phase: AtomicU8,
        notify: tokio::sync::Notify,
        operation: CanonicalOperationControl,
    }

    /// Request-scoped cancellation shared by ingress, transport, sink, and
    /// canonical apply. Starting the canonical commit is atomic with cancel.
    #[derive(Clone)]
    pub(crate) struct CorrectionRequestCancellation {
        inner: Arc<CorrectionRequestCancellationInner>,
    }

    impl CorrectionRequestCancellation {
        pub(crate) fn new() -> Self {
            Self {
                inner: Arc::new(CorrectionRequestCancellationInner {
                    phase: AtomicU8::new(REQUEST_ACTIVE),
                    notify: tokio::sync::Notify::new(),
                    operation: CanonicalOperationControl::production(),
                }),
            }
        }

        #[cfg(test)]
        fn with_fail_stop(fail_stop: CanonicalFailStopHook) -> Self {
            Self {
                inner: Arc::new(CorrectionRequestCancellationInner {
                    phase: AtomicU8::new(REQUEST_ACTIVE),
                    notify: tokio::sync::Notify::new(),
                    operation: CanonicalOperationControl::with_fail_stop(fail_stop),
                }),
            }
        }

        pub(crate) fn operation(&self) -> CanonicalOperationControl {
            self.inner.operation.clone()
        }

        pub(crate) fn cancel(&self) {
            loop {
                let phase = self.inner.phase.load(Ordering::Acquire);
                let cancelled = match phase {
                    REQUEST_ACTIVE => REQUEST_CANCELLED,
                    REQUEST_COMMITTING => REQUEST_COMMITTING_CANCELLED,
                    REQUEST_COMMITTED => REQUEST_COMMITTED_CANCELLED,
                    REQUEST_CANCELLED
                    | REQUEST_COMMITTING_CANCELLED
                    | REQUEST_COMMITTED_CANCELLED => return,
                    _ => unreachable!("invalid correction request phase"),
                };
                if self
                    .inner
                    .phase
                    .compare_exchange(phase, cancelled, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                {
                    self.inner.notify.notify_waiters();
                    return;
                }
            }
        }

        pub(crate) fn is_cancelled(&self) -> bool {
            matches!(
                self.inner.phase.load(Ordering::Acquire),
                REQUEST_CANCELLED | REQUEST_COMMITTING_CANCELLED | REQUEST_COMMITTED_CANCELLED
            )
        }

        pub(crate) async fn cancelled(&self) {
            if self.is_cancelled() {
                return;
            }
            let notified = self.inner.notify.notified();
            tokio::pin!(notified);
            notified.as_mut().enable();
            if self.is_cancelled() {
                return;
            }
            notified.await;
        }

        /// Claims the non-cancellable canonical publish/commit window. A
        /// concurrent cancel either wins first or records cancellation while
        /// allowing this already-linearized transaction to finish.
        pub(crate) fn try_begin_canonical_commit(&self) -> bool {
            self.inner
                .phase
                .compare_exchange(
                    REQUEST_ACTIVE,
                    REQUEST_COMMITTING,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
        }

        pub(crate) fn finish_canonical_commit(&self) {
            loop {
                let phase = self.inner.phase.load(Ordering::Acquire);
                let finished = match phase {
                    REQUEST_COMMITTING => REQUEST_COMMITTED,
                    REQUEST_COMMITTING_CANCELLED => REQUEST_COMMITTED_CANCELLED,
                    _ => return,
                };
                if self
                    .inner
                    .phase
                    .compare_exchange(phase, finished, Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
                {
                    return;
                }
            }
        }

        fn canonical_commit_started(&self) -> bool {
            matches!(
                self.inner.phase.load(Ordering::Acquire),
                REQUEST_COMMITTING
                    | REQUEST_COMMITTING_CANCELLED
                    | REQUEST_COMMITTED
                    | REQUEST_COMMITTED_CANCELLED
            )
        }
    }

    struct SubmitCancellationGuard {
        cancellation: CorrectionRequestCancellation,
        armed: bool,
    }

    impl SubmitCancellationGuard {
        fn new(cancellation: CorrectionRequestCancellation) -> Self {
            Self {
                cancellation,
                armed: true,
            }
        }

        fn disarm(&mut self) {
            self.armed = false;
        }
    }

    impl Drop for SubmitCancellationGuard {
        fn drop(&mut self) {
            if self.armed {
                self.cancellation.cancel();
            }
        }
    }

    #[async_trait::async_trait]
    pub(crate) trait CorrectionCandidateSink: Send + Sync + 'static {
        async fn stage_frame(&self, _input: &CorrectionFrameInput) -> Result<(), String> {
            Ok(())
        }

        async fn discard_frame(&self, _input: &CorrectionFrameInput) {}

        async fn apply_candidate(
            &self,
            candidate: CorrectionCandidate,
            cancellation: CorrectionRequestCancellation,
        ) -> Result<CorrectionDisposition, String>;

        async fn recover_abandoned_operation(&self, _cancellation: &CorrectionRequestCancellation) {
        }

        async fn canonical_correction_revision(&self) -> u64 {
            0
        }

        async fn stop(&self) {}
    }

    #[derive(Clone, Debug, PartialEq)]
    pub(crate) struct CanonicalCorrectionTransaction {
        pub(crate) runtime_generation: u64,
        pub(crate) backend_generation: u64,
        pub(crate) backend: CorrectionCandidateBackend,
        pub(crate) candidate_sequence: u64,
        pub(crate) payload_sha256: [u8; 32],
        pub(crate) correction_revision: u64,
        pub(crate) mapping_session_id: String,
        pub(crate) mapping_session_generation: u64,
        pub(crate) mapping_commit_id: MappingCommitId,
        pub(crate) occupancy_revision: u64,
        pub(crate) cloud_revision: u64,
        pub(crate) map_frame_generation: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) committed_map_source_timestamp_ns: u64,
        pub(crate) map_frame_id: String,
        pub(crate) odom_frame_id: String,
        pub(crate) body_frame_id: String,
        pub(crate) correction_lease: CorrectionLeaseId,
        pub(crate) base_anchor_map_from_odom: Isometry3<f64>,
        pub(crate) absolute_left_correction: Isometry3<f64>,
        pub(crate) effective_map_from_odom: Isometry3<f64>,
        pub(crate) frozen_odom_from_body: Isometry3<f64>,
    }

    #[async_trait::async_trait]
    pub(crate) trait CanonicalCorrectionApplier: Send + Sync + 'static {
        async fn apply_transaction(
            &self,
            transaction: CanonicalCorrectionTransaction,
            cancellation: CorrectionRequestCancellation,
        ) -> Result<(), CanonicalCorrectionApplyError>;
    }

    #[derive(Debug, Error, PartialEq, Eq)]
    pub(crate) enum CanonicalCorrectionApplyError {
        #[error("correction request was canceled")]
        Cancelled,
        #[error("{0}")]
        Failed(String),
    }

    #[derive(Clone)]
    struct StagedCorrectionFrame {
        boundary: CorrectionBoundary,
        base_anchor_map_from_odom: Isometry3<f64>,
        frozen_odom_from_body: Isometry3<f64>,
    }

    struct GeometryCorrectionState {
        coordinator: CorrectionCoordinator,
        session: Option<CorrectionSessionIdentity>,
        staged: Option<StagedCorrectionFrame>,
        last_applied: Option<StagedCorrectionFrame>,
        in_flight: Option<InFlightCorrection>,
        correction_revision: u64,
        stop_requested: bool,
        stopped: bool,
    }

    struct InFlightCorrection {
        prepared: CorrectionReceipt,
        staged: StagedCorrectionFrame,
        correction_revision: u64,
        cancellation: CorrectionRequestCancellation,
    }

    pub(crate) struct GeometryCorrectionSink {
        owner_identity: CorrectionOwnerRuntimeIdentity,
        backend: CorrectionCandidateBackend,
        map_frame_id: String,
        odom_frame_id: String,
        body_frame_id: String,
        applier: Arc<dyn CanonicalCorrectionApplier>,
        state: tokio::sync::Mutex<GeometryCorrectionState>,
        in_flight_changed: tokio::sync::Notify,
    }

    impl GeometryCorrectionSink {
        pub(crate) fn new(
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: impl Into<String>,
            odom_frame_id: impl Into<String>,
            body_frame_id: impl Into<String>,
            applier: Arc<dyn CanonicalCorrectionApplier>,
        ) -> Result<Self, String> {
            Self::new_for_backend(
                owner_identity,
                CorrectionCandidateBackend::LibraryPgo,
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                applier,
            )
        }

        pub(crate) fn new_for_spec(
            owner_identity: CorrectionOwnerRuntimeIdentity,
            spec: &CorrectionOwnerSpec,
            map_frame_id: impl Into<String>,
            odom_frame_id: impl Into<String>,
            body_frame_id: impl Into<String>,
            applier: Arc<dyn CanonicalCorrectionApplier>,
        ) -> Result<Self, String> {
            Self::new_for_backend(
                owner_identity,
                spec.candidate_backend(),
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                applier,
            )
        }

        fn new_for_backend(
            owner_identity: CorrectionOwnerRuntimeIdentity,
            backend: CorrectionCandidateBackend,
            map_frame_id: impl Into<String>,
            odom_frame_id: impl Into<String>,
            body_frame_id: impl Into<String>,
            applier: Arc<dyn CanonicalCorrectionApplier>,
        ) -> Result<Self, String> {
            validate_owner_identity(&owner_identity).map_err(|error| error.to_string())?;
            let map_frame_id = map_frame_id.into();
            let odom_frame_id = odom_frame_id.into();
            let body_frame_id = body_frame_id.into();
            validate_preflight_frame("map", &map_frame_id)?;
            validate_preflight_frame("odom", &odom_frame_id)?;
            validate_preflight_frame("body", &body_frame_id)?;
            if [
                map_frame_id.as_str(),
                odom_frame_id.as_str(),
                body_frame_id.as_str(),
            ]
            .into_iter()
            .collect::<std::collections::HashSet<_>>()
            .len()
                != 3
            {
                return Err("correction apply frames must be distinct".to_owned());
            }
            Ok(Self {
                owner_identity,
                backend,
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                applier,
                state: tokio::sync::Mutex::new(GeometryCorrectionState {
                    coordinator: CorrectionCoordinator::default(),
                    session: None,
                    staged: None,
                    last_applied: None,
                    in_flight: None,
                    correction_revision: 0,
                    stop_requested: false,
                    stopped: false,
                }),
                in_flight_changed: tokio::sync::Notify::new(),
            })
        }

        async fn stage(&self, input: &CorrectionFrameInput) -> Result<(), String> {
            let staged = staged_correction_frame(
                self.backend,
                &self.owner_identity,
                &self.map_frame_id,
                &self.odom_frame_id,
                &self.body_frame_id,
                input,
            )?;
            let mut state = self.state.lock().await;
            if state.stop_requested || state.stopped {
                return Err("correction apply sink is stopped".to_owned());
            }
            if state.in_flight.is_some() {
                return Err("a correction apply is in flight".to_owned());
            }
            let incoming_session = CorrectionSessionIdentity {
                mapping_session_id: staged.boundary.mapping_session_id.clone(),
                mapping_session_generation: staged.boundary.mapping_session_generation,
                map_frame_generation: staged.boundary.map_frame_generation,
            };
            match state.session.as_ref() {
                Some(current)
                    if incoming_session.mapping_session_generation
                        < current.mapping_session_generation =>
                {
                    return Err("correction frame belongs to a stale mapping session".to_owned());
                }
                Some(current)
                    if incoming_session.mapping_session_generation
                        == current.mapping_session_generation
                        && incoming_session != *current =>
                {
                    return Err(
                        "mapping session identity changed without a newer generation".to_owned(),
                    );
                }
                Some(current)
                    if incoming_session.mapping_session_generation
                        == current.mapping_session_generation => {}
                _ => {
                    state.coordinator = CorrectionCoordinator::default();
                    state.session = Some(incoming_session);
                    state.staged = None;
                    state.last_applied = None;
                    state.in_flight = None;
                    state.correction_revision = 0;
                }
            }
            if let Some(existing) = &state.staged {
                if existing.boundary == staged.boundary
                    && existing.base_anchor_map_from_odom == staged.base_anchor_map_from_odom
                    && existing.frozen_odom_from_body == staged.frozen_odom_from_body
                {
                    return Ok(());
                }
                return Err("a different correction frame is already staged".to_owned());
            }
            state.staged = Some(staged);
            Ok(())
        }

        async fn discard(&self, input: &CorrectionFrameInput) {
            let mut state = self.state.lock().await;
            let should_discard = state.staged.as_ref().is_some_and(|staged| {
                staged.boundary.mapping_commit_id == input.mapping_commit_id
                    && staged.boundary.registered_cloud == input.registered_cloud
                    && staged.boundary.correlated_odom == input.correlated_odom.proof
            });
            if should_discard {
                state.staged = None;
            }
        }

        pub(crate) async fn correction_revision(&self) -> u64 {
            self.state.lock().await.correction_revision
        }

        fn finalize_stop_locked(state: &mut GeometryCorrectionState) {
            if state.stop_requested && state.in_flight.is_none() && !state.stopped {
                state.staged = None;
                state.coordinator.stop();
                state.stopped = true;
            }
        }
    }

    #[async_trait::async_trait]
    impl CorrectionCandidateSink for GeometryCorrectionSink {
        async fn stage_frame(&self, input: &CorrectionFrameInput) -> Result<(), String> {
            self.stage(input).await
        }

        async fn discard_frame(&self, input: &CorrectionFrameInput) {
            self.discard(input).await;
        }

        async fn apply_candidate(
            &self,
            candidate: CorrectionCandidate,
            cancellation: CorrectionRequestCancellation,
        ) -> Result<CorrectionDisposition, String> {
            let operation = cancellation.operation();
            let transaction = {
                let mut state = self.state.lock().await;
                if cancellation.is_cancelled() || state.stop_requested || state.stopped {
                    return Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::Stopped,
                    ));
                }
                if state.in_flight.is_some() {
                    return Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::ApplyInFlight,
                    ));
                }
                let Some(staged) = state
                    .staged
                    .as_ref()
                    .or(state.last_applied.as_ref())
                    .cloned()
                else {
                    return Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::NoStagedFrame,
                    ));
                };
                let absolute_left_correction = correction_transform_isometry(candidate.transform)?;
                let effective_map_from_odom =
                    staged.base_anchor_map_from_odom * absolute_left_correction;
                if !is_finite_isometry(&effective_map_from_odom) {
                    return Err("effective correction transform is non-finite".to_owned());
                }
                let correction_revision = state
                    .correction_revision
                    .checked_add(1)
                    .ok_or_else(|| "correction revision exhausted".to_owned())?;
                let prepared = state.coordinator.prepare(&candidate, &staged.boundary);
                match &prepared.disposition {
                    CorrectionDisposition::Rejected(_) | CorrectionDisposition::AlreadyApplied => {
                        return Ok(prepared.disposition);
                    }
                    CorrectionDisposition::Applied => {
                        return Err("coordinator prepare returned an applied receipt".to_owned());
                    }
                    CorrectionDisposition::Prepared => {}
                }
                let transaction = CanonicalCorrectionTransaction {
                    runtime_generation: candidate.identity.runtime_generation,
                    backend_generation: candidate.identity.backend_generation,
                    backend: candidate.identity.backend,
                    candidate_sequence: candidate.identity.candidate_sequence,
                    payload_sha256: candidate.payload_sha256,
                    correction_revision,
                    mapping_session_id: candidate.identity.mapping_session_id.clone(),
                    mapping_session_generation: candidate.identity.mapping_session_generation,
                    mapping_commit_id: candidate.identity.mapping_commit_id,
                    occupancy_revision: candidate.identity.occupancy_revision,
                    cloud_revision: candidate.identity.cloud_revision,
                    map_frame_generation: candidate.identity.map_frame_generation,
                    source_timestamp_ns: candidate.identity.source_timestamp_ns,
                    committed_map_source_timestamp_ns: candidate
                        .identity
                        .committed_map
                        .source_timestamp_ns,
                    map_frame_id: candidate.identity.map_frame_id.clone(),
                    odom_frame_id: candidate.identity.odom_frame_id.clone(),
                    body_frame_id: candidate.identity.body_frame_id.clone(),
                    correction_lease: candidate.identity.correction_lease,
                    base_anchor_map_from_odom: staged.base_anchor_map_from_odom,
                    absolute_left_correction,
                    effective_map_from_odom,
                    frozen_odom_from_body: staged.frozen_odom_from_body,
                };
                state.in_flight = Some(InFlightCorrection {
                    prepared,
                    staged,
                    correction_revision,
                    cancellation: cancellation.clone(),
                });
                transaction
            };

            let apply_result = self
                .applier
                .apply_transaction(transaction, cancellation.clone())
                .await;
            let mut state = self.state.lock().await;
            let Some(reservation) = state.in_flight.take() else {
                if operation.phase().is_post_claim() {
                    operation.poison("canonical correction reservation was lost after claim");
                }
                Self::finalize_stop_locked(&mut state);
                drop(state);
                self.in_flight_changed.notify_waiters();
                return Err("canonical correction reservation was lost".to_owned());
            };
            let mut retain_poisoned = false;
            let result = match (apply_result, operation.phase()) {
                (Ok(()), CanonicalOperationPhase::StateCommitted) => {
                    match state.coordinator.commit_applied(&reservation.prepared) {
                        Ok(applied) => {
                            state.correction_revision = reservation.correction_revision;
                            state.last_applied = Some(reservation.staged.clone());
                            state.staged = None;
                            match operation.mark_coordinator_finalized() {
                                Ok(()) => {
                                    reservation.cancellation.finish_canonical_commit();
                                    Ok(applied.disposition)
                                }
                                Err(error) => {
                                    operation.poison(error.clone());
                                    retain_poisoned = true;
                                    Err(error)
                                }
                            }
                        }
                        Err(error) => {
                            let error = error.to_string();
                            operation.poison(format!(
                                "coordinator finalize failed after state commit: {error}"
                            ));
                            retain_poisoned = true;
                            Err(error)
                        }
                    }
                }
                (
                    Err(CanonicalCorrectionApplyError::Failed(error)),
                    CanonicalOperationPhase::AckedErr,
                ) => match state.coordinator.abort(&reservation.prepared) {
                    Ok(()) => {
                        operation.finish_acked_err();
                        reservation.cancellation.finish_canonical_commit();
                        Err(error)
                    }
                    Err(abort_error) => {
                        let detail = format!("{error}; abort failed: {abort_error}");
                        operation.poison(detail.clone());
                        retain_poisoned = true;
                        Err(detail)
                    }
                },
                (Ok(()) | Err(CanonicalCorrectionApplyError::Cancelled), phase)
                    if !phase.is_post_claim() =>
                {
                    operation.cancel_pending();
                    match state.coordinator.abort(&reservation.prepared) {
                        Ok(()) => {
                            state.staged = None;
                            Ok(CorrectionDisposition::Rejected(
                                CorrectionRejectReason::Stopped,
                            ))
                        }
                        Err(error) => Err(error.to_string()),
                    }
                }
                (Err(CanonicalCorrectionApplyError::Failed(error)), phase)
                    if !phase.is_post_claim() =>
                {
                    operation.cancel_pending();
                    match state.coordinator.abort(&reservation.prepared) {
                        Ok(()) => Err(error),
                        Err(abort_error) => Err(format!("{error}; abort failed: {abort_error}")),
                    }
                }
                (result, phase) => {
                    let detail =
                        format!("canonical operation ended in unsafe phase {phase:?}: {result:?}");
                    operation.poison(detail.clone());
                    retain_poisoned = true;
                    Err(detail)
                }
            };
            if retain_poisoned {
                state.in_flight = Some(reservation);
            }
            Self::finalize_stop_locked(&mut state);
            drop(state);
            self.in_flight_changed.notify_waiters();
            result
        }

        async fn recover_abandoned_operation(&self, cancellation: &CorrectionRequestCancellation) {
            let operation = cancellation.operation();
            if operation.phase().is_post_claim() {
                operation.poison("post-claim canonical operation was abandoned");
                return;
            }
            operation.cancel_pending();
            let mut state = self.state.lock().await;
            let should_abort = state.in_flight.as_ref().is_some_and(|in_flight| {
                Arc::ptr_eq(&in_flight.cancellation.inner, &cancellation.inner)
            });
            if should_abort {
                if let Some(reservation) = state.in_flight.take() {
                    let _ = state.coordinator.abort(&reservation.prepared);
                }
            }
            Self::finalize_stop_locked(&mut state);
            drop(state);
            self.in_flight_changed.notify_waiters();
        }

        async fn canonical_correction_revision(&self) -> u64 {
            self.correction_revision().await
        }

        async fn stop(&self) {
            loop {
                let notified = self.in_flight_changed.notified();
                tokio::pin!(notified);
                notified.as_mut().enable();
                let cancellation = {
                    let mut state = self.state.lock().await;
                    state.stop_requested = true;
                    if let Some(in_flight) = state.in_flight.as_ref() {
                        let operation = in_flight.cancellation.operation();
                        if operation.phase() == CanonicalOperationPhase::Poisoned {
                            operation.poison("stop observed a poisoned canonical operation");
                            drop(state);
                            std::future::pending::<()>().await;
                            unreachable!("poisoned correction stop cannot return");
                        }
                    }
                    Self::finalize_stop_locked(&mut state);
                    if state.stopped {
                        return;
                    }
                    state
                        .in_flight
                        .as_ref()
                        .map(|in_flight| in_flight.cancellation.clone())
                };
                if let Some(cancellation) = cancellation {
                    cancellation.cancel();
                }
                notified.await;
            }
        }
    }

    fn staged_correction_frame(
        backend: CorrectionCandidateBackend,
        owner: &CorrectionOwnerRuntimeIdentity,
        map_frame_id: &str,
        odom_frame_id: &str,
        body_frame_id: &str,
        input: &CorrectionFrameInput,
    ) -> Result<StagedCorrectionFrame, String> {
        if input.mapping_session_id.is_empty()
            || input.mapping_session_id.trim() != input.mapping_session_id
            || input.mapping_session_id.len() > MAX_IDENTITY_STRING_BYTES
            || input.mapping_session_id.contains('\0')
            || input.mapping_session_generation == 0
            || input.map_frame_generation == 0
            || input.points_frame_id != body_frame_id
            || !is_finite_isometry(&input.base_anchor_map_from_odom)
            || input.registered_cloud.body_points_sha256
                != body_points_sha256(&input.body_points).map_err(|error| error.to_string())?
            || input.correlated_odom.proof.pose_sha256
                != odom_pose_sha256(&input.correlated_odom.odom_from_body)
                    .map_err(|error| error.to_string())?
            || input
                .committed_map
                .width
                .checked_mul(input.committed_map.height)
                .and_then(|cells| usize::try_from(cells).ok())
                != Some(input.committed_map_occupancy.len())
            || input.committed_map.payload_sha256
                != sha256_bytes(input.committed_map_occupancy.as_ref())
        {
            return Err("correction apply frame proof mismatch".to_owned());
        }
        let identity = correction_candidate_identity(
            backend,
            owner,
            1,
            map_frame_id,
            odom_frame_id,
            body_frame_id,
            input,
        );
        CorrectionCandidate::seal(
            identity.clone(),
            CorrectionTransform::from_isometry(&Isometry3::identity()),
        )
        .map_err(|error| error.to_string())?;
        Ok(StagedCorrectionFrame {
            boundary: CorrectionBoundary {
                backend: identity.backend,
                transform_semantics: identity.transform_semantics,
                runtime_generation: identity.runtime_generation,
                backend_generation: identity.backend_generation,
                mapping_session_id: identity.mapping_session_id,
                mapping_session_generation: identity.mapping_session_generation,
                map_frame_generation: identity.map_frame_generation,
                mapping_commit_id: identity.mapping_commit_id,
                occupancy_revision: identity.occupancy_revision,
                cloud_revision: identity.cloud_revision,
                registered_cloud: identity.registered_cloud,
                correlated_odom: identity.correlated_odom,
                committed_map: identity.committed_map,
                source_timestamp_ns: identity.source_timestamp_ns,
                map_frame_id: identity.map_frame_id,
                odom_frame_id: identity.odom_frame_id,
                body_frame_id: identity.body_frame_id,
                points_frame_id: identity.points_frame_id,
                correction_lease: identity.correction_lease,
            },
            base_anchor_map_from_odom: input.base_anchor_map_from_odom,
            frozen_odom_from_body: input.correlated_odom.odom_from_body,
        })
    }

    fn correction_candidate_identity(
        backend: CorrectionCandidateBackend,
        owner: &CorrectionOwnerRuntimeIdentity,
        candidate_sequence: u64,
        map_frame_id: &str,
        odom_frame_id: &str,
        body_frame_id: &str,
        input: &CorrectionFrameInput,
    ) -> CorrectionCandidateIdentity {
        CorrectionCandidateIdentity {
            backend,
            transform_semantics: CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3,
            runtime_generation: owner.runtime_generation,
            backend_generation: owner.backend_generation,
            candidate_sequence,
            mapping_session_id: input.mapping_session_id.clone(),
            mapping_session_generation: input.mapping_session_generation,
            map_frame_generation: input.map_frame_generation,
            mapping_commit_id: input.mapping_commit_id,
            occupancy_revision: input.occupancy_revision,
            cloud_revision: input.cloud_revision,
            registered_cloud: input.registered_cloud,
            correlated_odom: input.correlated_odom.proof,
            committed_map: input.committed_map,
            source_timestamp_ns: input.registered_cloud.source_timestamp_ns,
            map_frame_id: map_frame_id.to_owned(),
            odom_frame_id: odom_frame_id.to_owned(),
            body_frame_id: body_frame_id.to_owned(),
            points_frame_id: input.points_frame_id.clone(),
            correction_lease: owner.correction_lease,
        }
    }

    fn correction_transform_isometry(
        transform: CorrectionTransform,
    ) -> Result<Isometry3<f64>, String> {
        let [x, y, z, w] = transform.rotation_xyzw;
        let quaternion = Quaternion::new(w, x, y, z);
        let norm = quaternion.norm();
        if !norm.is_finite() || (norm * norm - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
            return Err("correction quaternion is invalid".to_owned());
        }
        let [tx, ty, tz] = transform.translation_m;
        let isometry = Isometry3::from_parts(
            Translation3::new(tx, ty, tz),
            UnitQuaternion::new_normalize(quaternion),
        );
        if !is_finite_isometry(&isometry) {
            return Err("correction isometry is non-finite".to_owned());
        }
        Ok(isometry)
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    struct CandidateApplyKey {
        candidate_sequence: u64,
        payload_sha256: [u8; 32],
    }

    type CandidateApplyResult = Result<CorrectionDisposition, String>;

    struct CorrectionRequestLease {
        cancellation: CorrectionRequestCancellation,
        lifecycle_guard: parking_lot::Mutex<Option<tokio::sync::OwnedMutexGuard<()>>>,
    }

    impl CorrectionRequestLease {
        fn new(
            cancellation: CorrectionRequestCancellation,
            lifecycle_guard: Option<tokio::sync::OwnedMutexGuard<()>>,
        ) -> Self {
            Self {
                cancellation,
                lifecycle_guard: parking_lot::Mutex::new(lifecycle_guard),
            }
        }

        fn cancellation(&self) -> CorrectionRequestCancellation {
            self.cancellation.clone()
        }

        fn release(&self) {
            self.lifecycle_guard.lock().take();
        }

        fn has_lifecycle_guard(&self) -> bool {
            self.lifecycle_guard.lock().is_some()
        }
    }

    struct CandidateApplyWaiter {
        sender: oneshot::Sender<CandidateApplyResult>,
        lease: Arc<CorrectionRequestLease>,
        applying: bool,
    }

    struct CandidateApplyWaiters {
        admission_open: Arc<AtomicBool>,
        pending: parking_lot::Mutex<HashMap<CandidateApplyKey, CandidateApplyWaiter>>,
        settled: Notify,
    }

    impl Default for CandidateApplyWaiters {
        fn default() -> Self {
            Self::new(Arc::new(AtomicBool::new(true)))
        }
    }

    impl CandidateApplyWaiters {
        fn new(admission_open: Arc<AtomicBool>) -> Self {
            Self {
                admission_open,
                pending: parking_lot::Mutex::new(HashMap::new()),
                settled: Notify::new(),
            }
        }

        fn register(
            &self,
            key: CandidateApplyKey,
            lease: Arc<CorrectionRequestLease>,
        ) -> Result<oneshot::Receiver<CandidateApplyResult>, String> {
            let (sender, receiver) = oneshot::channel();
            let mut pending = self.pending.lock();
            if !self.admission_open.load(Ordering::Acquire) {
                return Err("correction admission is closed".to_owned());
            }
            if pending.contains_key(&key) {
                return Err("duplicate correction apply waiter".to_owned());
            }
            pending.insert(
                key,
                CandidateApplyWaiter {
                    sender,
                    lease,
                    applying: false,
                },
            );
            Ok(receiver)
        }

        fn cancel_all_exact(&self) {
            let pending = self.pending.lock();
            for waiter in pending.values() {
                if !waiter
                    .lease
                    .cancellation
                    .operation()
                    .phase()
                    .is_post_claim()
                {
                    waiter.lease.cancellation.cancel();
                }
            }
        }

        async fn wait_settled(&self) {
            loop {
                let settled = self.settled.notified();
                tokio::pin!(settled);
                settled.as_mut().enable();
                if self.pending.lock().is_empty() {
                    return;
                }
                settled.await;
            }
        }

        fn cancellation(&self, key: CandidateApplyKey) -> Option<CorrectionRequestCancellation> {
            self.pending
                .lock()
                .get(&key)
                .map(|waiter| waiter.lease.cancellation())
        }

        fn claim_apply(&self, key: CandidateApplyKey) -> Option<Arc<CorrectionRequestLease>> {
            let mut pending = self.pending.lock();
            let waiter = pending.get_mut(&key)?;
            if waiter.applying {
                return None;
            }
            waiter.applying = true;
            Some(Arc::clone(&waiter.lease))
        }

        /// Atomically removes a waiter that the subscriber has not claimed.
        /// `true` means an apply is already running and must be joined.
        fn apply_claimed_or_remove(&self, key: CandidateApplyKey) -> bool {
            let mut pending = self.pending.lock();
            let (claimed, removed) = match pending.get(&key).map(|waiter| waiter.applying) {
                Some(true) => (true, false),
                Some(false) => {
                    pending.remove(&key);
                    (false, true)
                }
                None => (false, false),
            };
            drop(pending);
            if removed {
                self.settled.notify_waiters();
            }
            claimed
        }

        fn complete(&self, key: CandidateApplyKey, result: CandidateApplyResult) {
            if let Some(waiter) = self.pending.lock().remove(&key) {
                let _ = waiter.sender.send(result);
                self.settled.notify_waiters();
            }
        }

        fn remove(&self, key: CandidateApplyKey) {
            if self.pending.lock().remove(&key).is_some() {
                self.settled.notify_waiters();
            }
        }
    }

    fn canonical_push_string(out: &mut Vec<u8>, value: &str) {
        let length = u32::try_from(value.len()).expect("frozen canonical string fits u32");
        out.extend_from_slice(&length.to_be_bytes());
        out.extend_from_slice(value.as_bytes());
    }

    fn canonical_push_bytes(out: &mut Vec<u8>, value: &[u8]) {
        let length = u64::try_from(value.len()).expect("frozen canonical bytes fit u64");
        out.extend_from_slice(&length.to_be_bytes());
        out.extend_from_slice(value);
    }

    fn canonical_push_present_bytes(out: &mut Vec<u8>, value: &[u8]) {
        if value.is_empty() {
            out.push(0);
        } else {
            out.push(1);
            canonical_push_bytes(out, value);
        }
    }

    fn canonical_push_optional_digest(out: &mut Vec<u8>, value: Option<[u8; 32]>) {
        match value {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value);
            }
            None => out.push(0),
        }
    }

    pub(crate) fn relocalization_source_policy_preimage_v1() -> Vec<u8> {
        let mut out = b"nuwax.geometry.relocalize.source-policy.v1\0".to_vec();
        for value in [1_u32, 1, 1, 8] {
            out.extend_from_slice(&value.to_be_bytes());
        }
        for value in [
            50_000_u64, 100_000, 2_400_000, 4_096, 25_600_000, 250_000, 6_000_000, 250_000,
            6_000_000, 50_000, 100_000, 1_200_000, 2_400_000, 2_000,
        ] {
            out.extend_from_slice(&value.to_be_bytes());
        }
        out.extend_from_slice(&1_u32.to_be_bytes());
        out.extend_from_slice(&1_u32.to_be_bytes());
        out
    }

    pub(crate) fn relocalization_deadline_policy_preimage_v1() -> Vec<u8> {
        let mut out = b"nuwax.geometry.relocalize.deadline-policy.v1\0".to_vec();
        out.extend_from_slice(&1_u32.to_be_bytes());
        for value in [250_u64, 30_000, 30_000, 2_000, 5_000] {
            out.extend_from_slice(&value.to_be_bytes());
        }
        for value in [1_u32, 1, 1] {
            out.extend_from_slice(&value.to_be_bytes());
        }
        out
    }

    pub(crate) fn relocalization_algorithm_options_preimage_v1() -> Vec<u8> {
        let mut out = b"nuwax.geometry.relocalize.algorithm-options.v1\0".to_vec();
        out.extend_from_slice(&1_u32.to_be_bytes());
        canonical_push_string(&mut out, "nuwax_mapping::relocation_pipeline_with_config");
        out.extend_from_slice(&1_u32.to_be_bytes());
        out.extend_from_slice(&3_u32.to_be_bytes());
        for value in [0.2_f64, 0.3, 0.8] {
            out.extend_from_slice(&value.to_be_bytes());
        }
        out.extend_from_slice(&100_u64.to_be_bytes());
        out.extend_from_slice(&4_096_u64.to_be_bytes());
        out.extend_from_slice(&3_u64.to_be_bytes());
        out.extend_from_slice(&0.1_f64.to_be_bytes());
        out.extend_from_slice(&(0.1_f64 * 1.5).to_be_bytes());
        out.extend_from_slice(&3_u64.to_be_bytes());
        out.extend_from_slice(&0.45_f64.to_be_bytes());
        out.extend_from_slice(&50_u64.to_be_bytes());
        out.extend_from_slice(&1.0e-6_f64.to_be_bytes());
        out.extend_from_slice(&(0.1_f64 * 1.5).to_be_bytes());
        out.extend_from_slice(&10.0_f64.to_be_bytes());
        out
    }

    pub(crate) fn relocalization_source_policy_sha256_v1() -> [u8; 32] {
        sha256_bytes(&relocalization_source_policy_preimage_v1())
    }

    pub(crate) fn relocalization_deadline_policy_sha256_v1() -> [u8; 32] {
        sha256_bytes(&relocalization_deadline_policy_preimage_v1())
    }

    pub(crate) fn relocalization_algorithm_options_sha256_v1() -> [u8; 32] {
        sha256_bytes(&relocalization_algorithm_options_preimage_v1())
    }

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

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum ActivationJournalStatus {
        Prepared,
        Applied,
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) struct ActivationJournalEntry {
        pub(crate) step: ActivationJournalStep,
        pub(crate) idempotency_key: [u8; 32],
        pub(crate) status: ActivationJournalStatus,
        pub(crate) verifier_receipt_sha256: Option<[u8; 32]>,
        pub(crate) typed_failure: Option<String>,
    }

    #[derive(Clone, Debug)]
    pub(crate) struct ActivationReceiptSeed {
        pub(crate) activation_request_id: String,
        pub(crate) activation_request_identity_sha256: [u8; 32],
        pub(crate) asset_manifest_sha256: Option<[u8; 32]>,
        pub(crate) asset_bundle_sha256: Option<[u8; 32]>,
        pub(crate) modern: bool,
        pub(crate) immutable_target: Option<crate::pointcloud::ImmutableActivatedTarget>,
    }

    #[derive(Clone, Debug)]
    pub(crate) struct ActivationPublicationProof {
        pub(crate) source_timestamp_ns: u64,
        pub(crate) cloud_revision: u64,
        pub(crate) point_count: u32,
        pub(crate) voxel_size: f32,
        pub(crate) cloud_receipt: crate::mapping_commit::CloudCommitReceipt,
        pub(crate) target_receipt_sha256: [u8; 32],
    }

    #[derive(Clone, Debug)]
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) struct SealedActivatedTargetSnapshot {
        pub(crate) runtime_generation: u64,
        pub(crate) lifecycle_epoch: u64,
        pub(crate) session_id: String,
        pub(crate) map_name: String,
        pub(crate) activation_request_id: String,
        pub(crate) activation_request_identity_sha256: [u8; 32],
        pub(crate) activation_identity_sha256: [u8; 32],
        pub(crate) target_receipt_sha256: [u8; 32],
        pub(crate) asset_manifest_sha256: Option<[u8; 32]>,
        pub(crate) asset_bundle_sha256: Option<[u8; 32]>,
        pub(crate) session_generation: u64,
        pub(crate) backend_generation: u64,
        pub(crate) publication_proof: ActivationPublicationProof,
        pub(crate) immutable_target: crate::pointcloud::ImmutableActivatedTarget,
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) struct ActivationToken {
        epoch: u64,
        activation_request_id: String,
        activation_request_identity_sha256: [u8; 32],
        activation_identity_sha256: Option<[u8; 32]>,
        reserved_backend_generation: Option<u64>,
    }

    #[derive(Clone, Debug)]
    pub(crate) enum ActivationLedgerAdmission {
        Execute(ActivationToken),
        Terminal(Box<crate::proto::MappingSessionState>),
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum PermanentActivationState {
        Staged,
        Sealed,
        InvalidatedPrecommit,
        InvalidatedPostcommit,
        OutcomeUnknown,
        Retired,
    }

    #[derive(Clone, Debug)]
    struct PermanentActivationRecord {
        seed: ActivationReceiptSeed,
        immutable_target: Option<crate::pointcloud::ImmutableActivatedTarget>,
        cloud_target_required: bool,
        epoch: u64,
        state: PermanentActivationState,
        activation_identity_sha256: Option<[u8; 32]>,
        target_receipt_sha256: Option<[u8; 32]>,
        session_generation: u64,
        backend_generation: u64,
        journal: Vec<ActivationJournalEntry>,
        publication_proof: Option<ActivationPublicationProof>,
        terminal: Option<crate::proto::MappingSessionState>,
    }

    #[derive(Default)]
    struct ActivatedTargetReceiptLedger {
        records: HashMap<String, PermanentActivationRecord>,
        current: Option<String>,
    }

    struct ActivatedTargetReceiptInner {
        runtime_generation: u64,
        lifecycle_epoch: AtomicU64,
        phase_a_retired: AtomicBool,
        retired_epoch: AtomicU64,
        ledger: parking_lot::Mutex<ActivatedTargetReceiptLedger>,
        #[cfg(test)]
        seal_before_ledger_gate: parking_lot::Mutex<Option<ActivationSealTestGate>>,
    }

    #[cfg(test)]
    struct ActivationSealTestGate {
        entered: Arc<std::sync::Barrier>,
        release: Arc<std::sync::Barrier>,
    }

    #[derive(Clone)]
    pub(crate) struct ActivatedTargetReceiptControl {
        inner: Arc<ActivatedTargetReceiptInner>,
    }

    impl ActivatedTargetReceiptControl {
        pub(crate) fn new(runtime_generation: u64) -> Self {
            assert!(
                runtime_generation > 0,
                "runtime generation must be positive"
            );
            Self {
                inner: Arc::new(ActivatedTargetReceiptInner {
                    runtime_generation,
                    lifecycle_epoch: AtomicU64::new(1),
                    phase_a_retired: AtomicBool::new(false),
                    retired_epoch: AtomicU64::new(0),
                    ledger: parking_lot::Mutex::new(ActivatedTargetReceiptLedger::default()),
                    #[cfg(test)]
                    seal_before_ledger_gate: parking_lot::Mutex::new(None),
                }),
            }
        }

        pub(crate) fn runtime_generation(&self) -> u64 {
            self.inner.runtime_generation
        }

        pub(crate) fn lifecycle_epoch(&self) -> u64 {
            self.inner.lifecycle_epoch.load(Ordering::Acquire)
        }

        fn phase_snapshot(&self) -> (bool, u64, u64) {
            (
                self.inner.phase_a_retired.load(Ordering::Acquire),
                self.inner.lifecycle_epoch.load(Ordering::Acquire),
                self.inner.retired_epoch.load(Ordering::Acquire),
            )
        }

        fn token_matches_locked(
            &self,
            token: &ActivationToken,
            ledger: &ActivatedTargetReceiptLedger,
        ) -> bool {
            if ledger.current.as_deref() != Some(token.activation_request_id.as_str()) {
                return false;
            }
            ledger
                .records
                .get(&token.activation_request_id)
                .is_some_and(|record| {
                    record.state == PermanentActivationState::Staged
                        && record.epoch == token.epoch
                        && record.seed.activation_request_identity_sha256
                            == token.activation_request_identity_sha256
                        && record.activation_identity_sha256 == token.activation_identity_sha256
                        && (token.reserved_backend_generation.is_none()
                            || record.backend_generation
                                == token.reserved_backend_generation.unwrap())
                })
        }

        fn token_matches_current_epoch_locked(
            &self,
            token: &ActivationToken,
            ledger: &ActivatedTargetReceiptLedger,
        ) -> bool {
            let phase = self.phase_snapshot();
            !phase.0 && phase.1 == token.epoch && self.token_matches_locked(token, ledger)
        }

        fn with_staged_record<R>(
            &self,
            token: &ActivationToken,
            operation: impl FnOnce(&PermanentActivationRecord) -> R,
        ) -> Option<R> {
            let ledger = self.inner.ledger.lock();
            if !self.token_matches_current_epoch_locked(token, &ledger) {
                return None;
            }
            ledger
                .records
                .get(&token.activation_request_id)
                .map(operation)
        }

        fn with_staged_record_mut<R>(
            &self,
            token: &ActivationToken,
            operation: impl FnOnce(&mut PermanentActivationRecord) -> R,
        ) -> Option<R> {
            let mut ledger = self.inner.ledger.lock();
            if !self.token_matches_current_epoch_locked(token, &ledger) {
                return None;
            }
            ledger
                .records
                .get_mut(&token.activation_request_id)
                .map(operation)
        }

        fn release_immutable_target(record: &mut PermanentActivationRecord) {
            record.immutable_target = None;
            record.seed.immutable_target = None;
            if let Some(terminal) = record.terminal.as_mut() {
                terminal.relocalization_target_available = false;
            }
        }

        fn has_complete_target_proof(record: &PermanentActivationRecord) -> bool {
            if !record.seed.modern
                || record.session_generation == 0
                || record.backend_generation == 0
                || record.activation_identity_sha256.is_none()
            {
                return false;
            }
            let (Some(target), Some(target_receipt_sha256), Some(proof)) = (
                record.immutable_target.as_ref(),
                record.target_receipt_sha256,
                record.publication_proof.as_ref(),
            ) else {
                return false;
            };
            proof.target_receipt_sha256 == target_receipt_sha256
                && proof.point_count == target.point_count
                && proof.voxel_size.to_bits() == target.voxel_size.to_bits()
                && proof.cloud_receipt.session_generation == record.session_generation
                && proof.cloud_receipt.cloud_revision == proof.cloud_revision
                && proof.cloud_receipt.source_timestamp_ns == proof.source_timestamp_ns
                && proof.cloud_receipt.point_count == target.point_count
        }

        fn sealed_target_available(record: &PermanentActivationRecord) -> bool {
            record.state == PermanentActivationState::Sealed
                && Self::has_complete_target_proof(record)
        }

        fn retire_record(
            &self,
            record: &mut PermanentActivationRecord,
            fallback: crate::proto::MappingSessionState,
        ) -> crate::proto::MappingSessionState {
            record.state = PermanentActivationState::Retired;
            Self::release_immutable_target(record);
            let mut terminal = record.terminal.clone().unwrap_or(fallback);
            self.apply_record(
                &mut terminal,
                record,
                crate::proto::ActivationReceiptDisposition::Retired,
                false,
            );
            terminal.success = false;
            terminal.relocalization_target_available = false;
            terminal.activation_receipt_disposition =
                crate::proto::ActivationReceiptDisposition::Retired as i32;
            terminal.message = "ACTIVATION_IDENTITY_RETIRED".to_owned();
            record.terminal = Some(terminal.clone());
            terminal
        }

        pub(crate) fn validate_token(&self, token: &ActivationToken) -> bool {
            self.with_staged_record(token, |_| ()).is_some()
        }

        pub(crate) fn stage_or_replay(
            &self,
            mut seed: ActivationReceiptSeed,
            mut base_state: crate::proto::MappingSessionState,
            allow_new: bool,
        ) -> ActivationLedgerAdmission {
            let before = self.phase_snapshot();
            if before.0 {
                self.apply_policy_identity(&mut base_state);
                base_state.success = false;
                base_state.idempotent_replay = true;
                base_state.activation_receipt_disposition =
                    crate::proto::ActivationReceiptDisposition::Retired as i32;
                base_state.activation_request_id = seed.activation_request_id;
                base_state.activation_request_identity_sha256 =
                    seed.activation_request_identity_sha256.to_vec();
                base_state.message = "ACTIVATION_IDENTITY_RETIRED".to_owned();
                return ActivationLedgerAdmission::Terminal(Box::new(base_state));
            }

            let mut ledger = self.inner.ledger.lock();
            let replay_is_current =
                ledger.current.as_deref() == Some(seed.activation_request_id.as_str());
            if let Some(record) = ledger.records.get(&seed.activation_request_id) {
                let after = self.phase_snapshot();
                if before != after || after.0 {
                    drop(ledger);
                    return self.stage_or_replay(seed, base_state, allow_new);
                }
                if record.seed.activation_request_identity_sha256
                    != seed.activation_request_identity_sha256
                    || record.state == PermanentActivationState::Staged
                        && record.immutable_target != seed.immutable_target
                {
                    self.apply_record(
                        &mut base_state,
                        record,
                        crate::proto::ActivationReceiptDisposition::Conflict,
                        false,
                    );
                    base_state.success = false;
                    base_state.idempotent_replay = true;
                    base_state.message = "ACTIVATE_MAP_REPLAY_CONFLICT".to_owned();
                    return ActivationLedgerAdmission::Terminal(Box::new(base_state));
                }
                return match record.state {
                    PermanentActivationState::Staged if replay_is_current => {
                        ActivationLedgerAdmission::Execute(ActivationToken {
                            epoch: record.epoch,
                            activation_request_id: seed.activation_request_id,
                            activation_request_identity_sha256: seed
                                .activation_request_identity_sha256,
                            activation_identity_sha256: record.activation_identity_sha256,
                            reserved_backend_generation: (record.backend_generation > 0)
                                .then_some(record.backend_generation),
                        })
                    }
                    PermanentActivationState::Sealed => {
                        let mut terminal = record
                            .terminal
                            .clone()
                            .unwrap_or_else(crate::proto::MappingSessionState::default);
                        let target_available = replay_is_current
                            && !after.0
                            && record.epoch == after.1
                            && Self::sealed_target_available(record);
                        self.apply_record(
                            &mut terminal,
                            record,
                            crate::proto::ActivationReceiptDisposition::SealedReplay,
                            target_available,
                        );
                        terminal.idempotent_replay = true;
                        terminal.message = "ACTIVATION_SEALED_REPLAY".to_owned();
                        ActivationLedgerAdmission::Terminal(Box::new(terminal))
                    }
                    PermanentActivationState::Staged
                    | PermanentActivationState::InvalidatedPrecommit
                    | PermanentActivationState::InvalidatedPostcommit
                    | PermanentActivationState::OutcomeUnknown
                    | PermanentActivationState::Retired => {
                        let mut terminal = record.terminal.clone().unwrap_or(base_state);
                        self.apply_record(
                            &mut terminal,
                            record,
                            crate::proto::ActivationReceiptDisposition::Retired,
                            false,
                        );
                        terminal.success = false;
                        terminal.idempotent_replay = true;
                        terminal.message = "ACTIVATION_IDENTITY_RETIRED".to_owned();
                        ActivationLedgerAdmission::Terminal(Box::new(terminal))
                    }
                };
            }

            if !allow_new {
                self.apply_policy_identity(&mut base_state);
                base_state.success = false;
                base_state.idempotent_replay = true;
                base_state.activation_receipt_disposition =
                    crate::proto::ActivationReceiptDisposition::Conflict as i32;
                base_state.activation_request_id = seed.activation_request_id;
                base_state.activation_request_identity_sha256 =
                    seed.activation_request_identity_sha256.to_vec();
                base_state.message = "ACTIVATE_MAP_ACTIVE_IDENTITY_CONFLICT".to_owned();
                return ActivationLedgerAdmission::Terminal(Box::new(base_state));
            }

            let epoch = before.1;
            let request_id = seed.activation_request_id.clone();
            let request_identity = seed.activation_request_identity_sha256;
            let immutable_target = seed.immutable_target.take();
            if let Some(previous_request_id) = ledger.current.clone() {
                if let Some(previous) = ledger.records.get_mut(&previous_request_id) {
                    self.retire_record(previous, crate::proto::MappingSessionState::default());
                }
            }
            debug_assert!(
                ledger
                    .records
                    .values()
                    .all(|record| record.immutable_target.is_none()),
                "only the current activation may retain immutable target bytes"
            );
            ledger.records.insert(
                request_id.clone(),
                PermanentActivationRecord {
                    seed,
                    cloud_target_required: immutable_target.is_some(),
                    immutable_target,
                    epoch,
                    state: PermanentActivationState::Staged,
                    activation_identity_sha256: None,
                    target_receipt_sha256: None,
                    session_generation: 0,
                    backend_generation: 0,
                    journal: Vec::new(),
                    publication_proof: None,
                    terminal: None,
                },
            );
            ledger.current = Some(request_id.clone());
            debug_assert!(
                ledger
                    .records
                    .values()
                    .filter(|record| record.immutable_target.is_some())
                    .count()
                    <= 1,
                "activation receipt ledger retained more than one immutable target"
            );
            let after = self.phase_snapshot();
            if before != after || after.0 {
                let mut terminal = ledger
                    .records
                    .get_mut(&request_id)
                    .map(|record| self.retire_record(record, base_state.clone()))
                    .unwrap_or(base_state);
                terminal.idempotent_replay = true;
                terminal.activation_request_id = request_id;
                terminal.activation_request_identity_sha256 = request_identity.to_vec();
                return ActivationLedgerAdmission::Terminal(Box::new(terminal));
            }
            ActivationLedgerAdmission::Execute(ActivationToken {
                epoch,
                activation_request_id: request_id,
                activation_request_identity_sha256: request_identity,
                activation_identity_sha256: None,
                reserved_backend_generation: None,
            })
        }

        pub(crate) fn reserve_backend_generation(
            &self,
            token: &ActivationToken,
            backend_generation: u64,
        ) -> Option<ActivationToken> {
            if backend_generation == 0 {
                return None;
            }
            self.with_staged_record_mut(token, |record| {
                record.backend_generation = backend_generation;
                let mut token = token.clone();
                token.reserved_backend_generation = Some(backend_generation);
                token
            })
        }

        pub(crate) fn bind_activation_identity(
            &self,
            token: &ActivationToken,
            activation_identity_sha256: [u8; 32],
            session_generation: u64,
        ) -> Option<ActivationToken> {
            if session_generation == 0 {
                return None;
            }
            self.with_staged_record_mut(token, |record| {
                record.activation_identity_sha256 = Some(activation_identity_sha256);
                record.session_generation = session_generation;
                let mut token = token.clone();
                token.activation_identity_sha256 = Some(activation_identity_sha256);
                token
            })
        }

        pub(crate) fn journal(
            &self,
            token: &ActivationToken,
            entry: ActivationJournalEntry,
        ) -> bool {
            self.with_staged_record_mut(token, |record| {
                if let Some(existing) = record
                    .journal
                    .iter_mut()
                    .find(|existing| existing.step == entry.step)
                {
                    if existing.idempotency_key != entry.idempotency_key {
                        return false;
                    }
                    *existing = entry;
                } else {
                    record.journal.push(entry);
                }
                true
            })
            .unwrap_or(false)
        }

        pub(crate) fn journal_step_applied(
            &self,
            token: &ActivationToken,
            step: ActivationJournalStep,
        ) -> bool {
            self.with_staged_record(token, |record| {
                record.journal.iter().any(|entry| {
                    entry.step == step && entry.status == ActivationJournalStatus::Applied
                })
            })
            .unwrap_or(false)
        }

        pub(crate) fn store_publication_proof(
            &self,
            token: &ActivationToken,
            proof: ActivationPublicationProof,
        ) -> bool {
            self.with_staged_record_mut(token, |record| {
                record.target_receipt_sha256 = Some(proof.target_receipt_sha256);
                record.publication_proof = Some(proof);
            })
            .is_some()
        }

        pub(crate) fn publication_proof(
            &self,
            token: &ActivationToken,
        ) -> Option<ActivationPublicationProof> {
            self.with_staged_record(token, |record| record.publication_proof.clone())
                .flatten()
        }

        pub(crate) fn immutable_target(
            &self,
            token: &ActivationToken,
        ) -> Option<crate::pointcloud::ImmutableActivatedTarget> {
            self.with_staged_record(token, |record| record.immutable_target.clone())
                .flatten()
        }

        #[cfg_attr(not(test), allow(dead_code))]
        pub(crate) fn current_sealed_target_snapshot(
            &self,
        ) -> Option<SealedActivatedTargetSnapshot> {
            let before = self.phase_snapshot();
            if before.0 {
                return None;
            }
            let snapshot = {
                let ledger = self.inner.ledger.lock();
                let during = self.phase_snapshot();
                if during != before || during.0 {
                    return None;
                }
                let request_id = ledger.current.as_ref()?;
                let record = ledger.records.get(request_id)?;
                if record.epoch != during.1 || !Self::sealed_target_available(record) {
                    return None;
                }
                let terminal = record.terminal.as_ref()?;
                if terminal.session_id.is_empty() || terminal.map_name.is_empty() {
                    return None;
                }
                SealedActivatedTargetSnapshot {
                    runtime_generation: self.inner.runtime_generation,
                    lifecycle_epoch: record.epoch,
                    session_id: terminal.session_id.clone(),
                    map_name: terminal.map_name.clone(),
                    activation_request_id: record.seed.activation_request_id.clone(),
                    activation_request_identity_sha256: record
                        .seed
                        .activation_request_identity_sha256,
                    activation_identity_sha256: record.activation_identity_sha256?,
                    target_receipt_sha256: record.target_receipt_sha256?,
                    asset_manifest_sha256: record.seed.asset_manifest_sha256,
                    asset_bundle_sha256: record.seed.asset_bundle_sha256,
                    session_generation: record.session_generation,
                    backend_generation: record.backend_generation,
                    publication_proof: record.publication_proof.clone()?,
                    immutable_target: record.immutable_target.clone()?,
                }
            };
            let after = self.phase_snapshot();
            if after != before || after.0 {
                return None;
            }
            let ledger = self.inner.ledger.lock();
            let current = ledger.current.as_ref()?;
            let record = ledger.records.get(current)?;
            if current != &snapshot.activation_request_id
                || self.phase_snapshot() != after
                || record.epoch != snapshot.lifecycle_epoch
                || !Self::sealed_target_available(record)
                || record.activation_identity_sha256 != Some(snapshot.activation_identity_sha256)
                || record.target_receipt_sha256 != Some(snapshot.target_receipt_sha256)
                || record.immutable_target.as_ref() != Some(&snapshot.immutable_target)
            {
                return None;
            }
            Some(snapshot)
        }

        #[cfg(test)]
        pub(crate) fn install_seal_before_ledger_gate(
            &self,
            entered: Arc<std::sync::Barrier>,
            release: Arc<std::sync::Barrier>,
        ) {
            *self.inner.seal_before_ledger_gate.lock() =
                Some(ActivationSealTestGate { entered, release });
        }

        #[cfg(test)]
        fn wait_for_seal_before_ledger_gate(&self) {
            let gate = self.inner.seal_before_ledger_gate.lock().take();
            if let Some(gate) = gate {
                gate.entered.wait();
                gate.release.wait();
            }
        }

        #[cfg(test)]
        pub(crate) fn retained_immutable_target_bytes(&self) -> usize {
            self.inner
                .ledger
                .lock()
                .records
                .values()
                .filter_map(|record| record.immutable_target.as_ref())
                .fold(0_usize, |total, target| {
                    total.saturating_add(target.xyz_f32_le.len())
                })
        }

        #[cfg(test)]
        pub(crate) fn retained_immutable_target_bytes_for(&self, request_id: &str) -> usize {
            self.inner
                .ledger
                .lock()
                .records
                .get(request_id)
                .and_then(|record| record.immutable_target.as_ref())
                .map_or(0, |target| target.xyz_f32_le.len())
        }

        pub(crate) fn seal(
            &self,
            token: &ActivationToken,
            target_receipt_sha256: Option<[u8; 32]>,
            mut state: crate::proto::MappingSessionState,
        ) -> Option<crate::proto::MappingSessionState> {
            #[cfg(test)]
            self.wait_for_seal_before_ledger_gate();
            self.with_staged_record_mut(token, |record| {
                record.target_receipt_sha256 = target_receipt_sha256;
                let modern = record.seed.modern;
                // 二维资产仍须完整激活身份；原本携带三维目标的请求不能靠丢失目标降级成功。
                if modern
                    && (record.session_generation == 0
                        || record.backend_generation == 0
                        || record.activation_identity_sha256.is_none()
                        || (record.cloud_target_required && !Self::has_complete_target_proof(record)))
                {
                    return None;
                }
                record.state = PermanentActivationState::Sealed;
                let disposition = if modern {
                    crate::proto::ActivationReceiptDisposition::Sealed
                } else {
                    crate::proto::ActivationReceiptDisposition::LegacyUnbound
                };
                let target_available = Self::sealed_target_available(record);
                self.apply_record(&mut state, record, disposition, target_available);
                if !target_available {
                    Self::release_immutable_target(record);
                }
                record.terminal = Some(state.clone());
                Some(state)
            })
            .flatten()
        }

        pub(crate) fn invalidate(
            &self,
            token: &ActivationToken,
            disposition: crate::proto::ActivationReceiptDisposition,
            detail: &str,
            mut state: crate::proto::MappingSessionState,
        ) -> crate::proto::MappingSessionState {
            let permanent_state = match disposition {
                crate::proto::ActivationReceiptDisposition::InvalidatedPrecommit => {
                    PermanentActivationState::InvalidatedPrecommit
                }
                crate::proto::ActivationReceiptDisposition::InvalidatedPostcommit => {
                    PermanentActivationState::InvalidatedPostcommit
                }
                _ => PermanentActivationState::OutcomeUnknown,
            };
            if let Some(terminal) = self.with_staged_record_mut(token, |record| {
                let mut terminal = state.clone();
                record.state = permanent_state;
                self.apply_record(&mut terminal, record, disposition, false);
                terminal.success = false;
                terminal.message = detail.to_owned();
                Self::release_immutable_target(record);
                record.terminal = Some(terminal.clone());
                terminal
            }) {
                return terminal;
            }
            self.apply_policy_identity(&mut state);
            state.success = false;
            state.activation_receipt_disposition = disposition as i32;
            state.message = detail.to_owned();
            state
        }

        pub(crate) fn retire_for_mapping_change(&self) -> Result<u64, &'static str> {
            let mut ledger = self.inner.ledger.lock();
            let current = self.inner.lifecycle_epoch.load(Ordering::Acquire);
            if self.inner.phase_a_retired.load(Ordering::Acquire) {
                return Ok(current);
            }
            let next = current
                .checked_add(1)
                .ok_or("activation lifecycle epoch exhausted")?;
            if let Some(request_id) = ledger.current.clone() {
                if let Some(record) = ledger.records.get_mut(&request_id) {
                    self.retire_record(record, crate::proto::MappingSessionState::default());
                }
            }
            self.inner.lifecycle_epoch.store(next, Ordering::Release);
            Ok(next)
        }

        fn retire_for_phase_a_first_closer(
            &self,
            claim_first_closer: impl FnOnce() -> bool,
        ) -> bool {
            let mut ledger = self.inner.ledger.lock();
            if !claim_first_closer() {
                return false;
            }
            self.inner.phase_a_retired.store(true, Ordering::Release);
            let retired_epoch = self.inner.lifecycle_epoch.load(Ordering::Acquire);
            self.inner
                .retired_epoch
                .store(retired_epoch, Ordering::Release);
            self.inner.lifecycle_epoch.store(
                retired_epoch.checked_add(1).unwrap_or(retired_epoch),
                Ordering::Release,
            );
            if let Some(request_id) = ledger.current.clone() {
                if let Some(record) = ledger.records.get_mut(&request_id) {
                    self.retire_record(record, crate::proto::MappingSessionState::default());
                }
            }
            true
        }

        pub(crate) fn decorate_current_state(&self, state: &mut crate::proto::MappingSessionState) {
            self.apply_policy_identity(state);
            let before = self.phase_snapshot();
            let ledger = self.inner.ledger.lock();
            let record = ledger
                .current
                .as_ref()
                .and_then(|request_id| ledger.records.get(request_id));
            let after = self.phase_snapshot();
            if before != after {
                state.relocalization_target_available = false;
                state.activation_receipt_disposition =
                    crate::proto::ActivationReceiptDisposition::Retired as i32;
                return;
            }
            let Some(record) = record else {
                return;
            };
            let disposition = if after.0 || record.epoch != after.1 {
                crate::proto::ActivationReceiptDisposition::Retired
            } else {
                match record.state {
                    PermanentActivationState::Staged => {
                        crate::proto::ActivationReceiptDisposition::Staged
                    }
                    PermanentActivationState::Sealed if record.seed.modern => {
                        crate::proto::ActivationReceiptDisposition::Sealed
                    }
                    PermanentActivationState::Sealed => {
                        crate::proto::ActivationReceiptDisposition::LegacyUnbound
                    }
                    PermanentActivationState::InvalidatedPrecommit => {
                        crate::proto::ActivationReceiptDisposition::InvalidatedPrecommit
                    }
                    PermanentActivationState::InvalidatedPostcommit => {
                        crate::proto::ActivationReceiptDisposition::InvalidatedPostcommit
                    }
                    PermanentActivationState::OutcomeUnknown => {
                        crate::proto::ActivationReceiptDisposition::OutcomeUnknown
                    }
                    PermanentActivationState::Retired => {
                        crate::proto::ActivationReceiptDisposition::Retired
                    }
                }
            };
            let available =
                !after.0 && record.epoch == after.1 && Self::sealed_target_available(record);
            self.apply_record(state, record, disposition, available);
        }

        fn apply_policy_identity(&self, state: &mut crate::proto::MappingSessionState) {
            state.activation_receipt_schema_version = ACTIVATION_RECEIPT_SCHEMA_VERSION_V1;
            state.runtime_generation = self.inner.runtime_generation;
            state.source_policy_version = RELOCALIZATION_SOURCE_POLICY_VERSION_V1;
            state.source_policy_sha256 = relocalization_source_policy_sha256_v1().to_vec();
            state.deadline_policy_version = RELOCALIZATION_DEADLINE_POLICY_VERSION_V1;
            state.deadline_policy_sha256 = relocalization_deadline_policy_sha256_v1().to_vec();
            state.algorithm_options_version = RELOCALIZATION_ALGORITHM_OPTIONS_VERSION_V1;
            state.algorithm_options_sha256 = relocalization_algorithm_options_sha256_v1().to_vec();
        }

        fn apply_record(
            &self,
            state: &mut crate::proto::MappingSessionState,
            record: &PermanentActivationRecord,
            disposition: crate::proto::ActivationReceiptDisposition,
            target_available: bool,
        ) {
            self.apply_policy_identity(state);
            state.activation_receipt_disposition = disposition as i32;
            state.backend_generation = record.backend_generation;
            state.activation_request_id = record.seed.activation_request_id.clone();
            state.activation_request_identity_sha256 =
                record.seed.activation_request_identity_sha256.to_vec();
            state.activation_identity_sha256 = if record.seed.modern {
                record
                    .activation_identity_sha256
                    .map(|digest| digest.to_vec())
                    .unwrap_or_default()
            } else {
                Vec::new()
            };
            state.target_receipt_sha256 = record
                .target_receipt_sha256
                .map(|digest| digest.to_vec())
                .unwrap_or_default();
            state.asset_manifest_sha256 = record
                .seed
                .asset_manifest_sha256
                .map(|digest| digest.to_vec())
                .unwrap_or_default();
            state.asset_bundle_sha256 = record
                .seed
                .asset_bundle_sha256
                .map(|digest| digest.to_vec())
                .unwrap_or_default();
            state.relocalization_target_available = target_available;
        }
    }

    /// Cloneable Phase A admission/cancellation fence. It never owns worker
    /// joins; the retained correction owner remains solely responsible for
    /// terminal teardown.
    #[derive(Clone)]
    pub(crate) struct CorrectionPhaseAControl {
        admission_open: Arc<AtomicBool>,
        admission_tx: watch::Sender<bool>,
        apply_waiters: Arc<CandidateApplyWaiters>,
        close_state: Arc<AtomicU8>,
        process_stop_changed: Arc<Notify>,
        activated_targets: ActivatedTargetReceiptControl,
        command_claim_fence: Arc<parking_lot::Mutex<()>>,
        relocalization_mode: Arc<RelocalizationModeFence>,
        #[cfg(test)]
        close_after_admission_gate: Arc<parking_lot::Mutex<Option<ActivationSealTestGate>>>,
    }

    pub(crate) struct CorrectionProcessStopAttempt {
        phase_a: CorrectionPhaseAControl,
        fail_on_drop: bool,
        completed: bool,
    }

    fn correction_frame_admission_fence() -> &'static parking_lot::Mutex<()> {
        static FENCE: std::sync::OnceLock<parking_lot::Mutex<()>> = std::sync::OnceLock::new();
        FENCE.get_or_init(|| parking_lot::Mutex::new(()))
    }

    impl CorrectionPhaseAControl {
        fn new(
            admission_open: Arc<AtomicBool>,
            admission_tx: watch::Sender<bool>,
            apply_waiters: Arc<CandidateApplyWaiters>,
            runtime_generation: u64,
        ) -> Self {
            Self::new_with_relocalization_mode_fence(
                admission_open,
                admission_tx,
                apply_waiters,
                runtime_generation,
                Arc::new(RelocalizationModeFence::default()),
            )
        }

        fn new_with_relocalization_mode_fence(
            admission_open: Arc<AtomicBool>,
            admission_tx: watch::Sender<bool>,
            apply_waiters: Arc<CandidateApplyWaiters>,
            runtime_generation: u64,
            relocalization_mode: Arc<RelocalizationModeFence>,
        ) -> Self {
            Self {
                admission_open,
                admission_tx,
                apply_waiters,
                close_state: Arc::new(AtomicU8::new(CORRECTION_PHASE_A_OPEN)),
                process_stop_changed: Arc::new(Notify::new()),
                activated_targets: ActivatedTargetReceiptControl::new(runtime_generation),
                command_claim_fence: Arc::new(parking_lot::Mutex::new(())),
                relocalization_mode,
                #[cfg(test)]
                close_after_admission_gate: Arc::new(parking_lot::Mutex::new(None)),
            }
        }

        pub(crate) fn activated_target_receipts(&self) -> ActivatedTargetReceiptControl {
            self.activated_targets.clone()
        }

        fn command_claim_fence(&self) -> Arc<parking_lot::Mutex<()>> {
            Arc::clone(&self.command_claim_fence)
        }

        fn relocalization_mode(&self) -> Arc<RelocalizationModeFence> {
            Arc::clone(&self.relocalization_mode)
        }

        pub(crate) fn begin(&self) {
            self.close_with_state(CORRECTION_PHASE_A_INTERNAL_CLOSED);
        }

        pub(crate) fn begin_process_stop(&self) -> CorrectionProcessStopAttempt {
            let fail_on_drop = self.close_with_state(CORRECTION_PHASE_A_PROCESS_STOP_REQUESTED);
            CorrectionProcessStopAttempt {
                phase_a: self.clone(),
                fail_on_drop,
                completed: false,
            }
        }

        fn close_with_state(&self, requested_state: u8) -> bool {
            let _frame_admission_guard = correction_frame_admission_fence().lock();
            let claim_fence = Arc::clone(&self.command_claim_fence);
            let _claim_guard = claim_fence.lock();
            let first_closer = self
                .close_state
                .compare_exchange(
                    CORRECTION_PHASE_A_OPEN,
                    requested_state,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok();
            self.admission_open.store(false, Ordering::Release);
            self.admission_tx.send_replace(false);
            #[cfg(test)]
            self.wait_for_close_after_admission_gate();
            if first_closer {
                let retired = self
                    .activated_targets
                    .retire_for_phase_a_first_closer(|| true);
                debug_assert!(retired);
            }
            self.apply_waiters.cancel_all_exact();
            first_closer
        }

        #[cfg(test)]
        fn install_close_after_admission_gate(
            &self,
            entered: Arc<std::sync::Barrier>,
            release: Arc<std::sync::Barrier>,
        ) {
            *self.close_after_admission_gate.lock() =
                Some(ActivationSealTestGate { entered, release });
        }

        #[cfg(test)]
        fn wait_for_close_after_admission_gate(&self) {
            let gate = self.close_after_admission_gate.lock().take();
            if let Some(gate) = gate {
                gate.entered.wait();
                gate.release.wait();
            }
        }

        fn process_stop_requested(&self) -> bool {
            matches!(
                self.close_state.load(Ordering::Acquire),
                CORRECTION_PHASE_A_PROCESS_STOP_REQUESTED
                    | CORRECTION_PHASE_A_PROCESS_STOP_FAILED
                    | CORRECTION_PHASE_A_PROCESS_STOP_SUCCEEDED
            )
        }

        pub(crate) async fn wait_process_stop_failed(&self) {
            loop {
                let changed = self.process_stop_changed.notified();
                tokio::pin!(changed);
                changed.as_mut().enable();
                if self.close_state.load(Ordering::Acquire)
                    == CORRECTION_PHASE_A_PROCESS_STOP_FAILED
                {
                    return;
                }
                changed.await;
            }
        }

        pub(crate) fn process_stop_failed(&self) -> bool {
            self.close_state.load(Ordering::Acquire) == CORRECTION_PHASE_A_PROCESS_STOP_FAILED
        }

        async fn wait_process_stop_result(&self) -> Result<(), ()> {
            loop {
                let changed = self.process_stop_changed.notified();
                tokio::pin!(changed);
                changed.as_mut().enable();
                match self.close_state.load(Ordering::Acquire) {
                    CORRECTION_PHASE_A_PROCESS_STOP_SUCCEEDED => return Ok(()),
                    CORRECTION_PHASE_A_INTERNAL_CLOSED | CORRECTION_PHASE_A_PROCESS_STOP_FAILED => {
                        return Err(())
                    }
                    CORRECTION_PHASE_A_PROCESS_STOP_REQUESTED => changed.await,
                    _ => return Err(()),
                }
            }
        }

        pub(crate) async fn wait_settled(&self) {
            self.apply_waiters.wait_settled().await;
        }

        pub(crate) fn is_open(&self) -> bool {
            self.admission_open.load(Ordering::Acquire)
        }

        #[cfg(test)]
        pub(crate) fn for_tests() -> Self {
            let admission_open = Arc::new(AtomicBool::new(true));
            let (admission_tx, _) = watch::channel(true);
            let apply_waiters = Arc::new(CandidateApplyWaiters::new(Arc::clone(&admission_open)));
            Self::new(admission_open, admission_tx, apply_waiters, 1)
        }
    }

    impl CorrectionProcessStopAttempt {
        pub(crate) fn owns_transition(&self) -> bool {
            self.fail_on_drop
        }

        pub(crate) async fn wait_for_owner_result(&self) -> Result<(), ()> {
            self.phase_a.wait_process_stop_result().await
        }

        pub(crate) fn complete(mut self) {
            if self.fail_on_drop
                && self
                    .phase_a
                    .close_state
                    .compare_exchange(
                        CORRECTION_PHASE_A_PROCESS_STOP_REQUESTED,
                        CORRECTION_PHASE_A_PROCESS_STOP_SUCCEEDED,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    )
                    .is_ok()
            {
                self.phase_a.process_stop_changed.notify_waiters();
            }
            self.completed = true;
        }
    }

    impl Drop for CorrectionProcessStopAttempt {
        fn drop(&mut self) {
            if !self.fail_on_drop || self.completed {
                return;
            }
            if self
                .phase_a
                .close_state
                .compare_exchange(
                    CORRECTION_PHASE_A_PROCESS_STOP_REQUESTED,
                    CORRECTION_PHASE_A_PROCESS_STOP_FAILED,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                )
                .is_ok()
            {
                self.phase_a.process_stop_changed.notify_waiters();
            }
        }
    }

    struct CorrectionFrameRequest {
        input: CorrectionFrameInput,
        response: oneshot::Sender<Result<CorrectionSubmitDisposition, CorrectionRuntimeError>>,
        cancellation: CorrectionRequestCancellation,
        lease: Arc<CorrectionRequestLease>,
    }

    struct CandidateProducerContext {
        publisher: Option<LcmTransport>,
        binding: Option<(LcmTopic, LcmMessageType)>,
        finalizer: CanonicalPublishFinalizerIngress,
        apply_waiters: Arc<CandidateApplyWaiters>,
        sink: Arc<dyn CorrectionCandidateSink>,
        cleanup: CorrectionCleanupIngress,
        apply_ack_timeout: Duration,
        phase_a: CorrectionPhaseAControl,
        #[cfg(test)]
        publish_gate: Option<Arc<CandidatePublishGate>>,
        #[cfg(test)]
        finalizer_outcome_gate: Option<Arc<CandidateFinalizerOutcomeGate>>,
    }

    enum CandidateDelivery {
        PrivateCorrectionTransport,
        DirectFinalizer,
    }

    enum CandidateProduction {
        Cancelled,
        NoCandidate,
        Rejected {
            reason_code: i32,
            reason: String,
        },
        Candidate {
            candidate: Box<CorrectionCandidate>,
            delivery: CandidateDelivery,
        },
    }

    struct NativePgoCandidateOwner {
        owner: Option<OnlinePgoOwner>,
        ingress: OnlinePgoIngress,
        identity: CorrectionOwnerRuntimeIdentity,
        map_frame_id: String,
        odom_frame_id: String,
        body_frame_id: String,
    }

    enum CorrectionCandidateOwner {
        Local(CorrectionOwnerHandle),
        NativePgo(Box<NativePgoCandidateOwner>),
    }

    impl CorrectionCandidateOwner {
        async fn start(
            spec: CorrectionOwnerSpec,
            identity: CorrectionOwnerRuntimeIdentity,
            provider: &str,
        ) -> Result<Self, CorrectionRuntimeError> {
            match spec {
                CorrectionOwnerSpec::NativePgo {
                    map_frame_id,
                    odom_frame_id,
                    body_frame_id,
                    executable,
                } => {
                    validate_owner_identity(&identity)?;
                    let runtime_generation =
                        i64::try_from(identity.runtime_generation).map_err(|_| {
                            CorrectionRuntimeError::NativePgo(
                                "runtime generation exceeds i64".to_owned(),
                            )
                        })?;
                    let backend_generation =
                        i64::try_from(identity.backend_generation).map_err(|_| {
                            CorrectionRuntimeError::NativePgo(
                                "backend generation exceeds i64".to_owned(),
                            )
                        })?;
                    let mut config =
                        OnlinePgoConfig::new(provider, runtime_generation, backend_generation);
                    config.executable = executable;
                    let (owner, ingress) = OnlinePgoOwner::start(config)
                        .await
                        .map_err(|error| CorrectionRuntimeError::NativePgo(error.to_string()))?;
                    Ok(Self::NativePgo(Box::new(NativePgoCandidateOwner {
                        owner: Some(owner),
                        ingress,
                        identity,
                        map_frame_id,
                        odom_frame_id,
                        body_frame_id,
                    })))
                }
                local => Ok(Self::Local(CorrectionOwnerHandle::start(local, identity)?)),
            }
        }

        fn is_finished(&self) -> bool {
            match self {
                Self::Local(_) => false,
                Self::NativePgo(native) => native
                    .owner
                    .as_ref()
                    .is_none_or(OnlinePgoOwner::is_finished),
            }
        }

        async fn submit_frame(
            &mut self,
            input: CorrectionFrameInput,
            cancellation: &CorrectionRequestCancellation,
        ) -> Result<CandidateProduction, CorrectionRuntimeError> {
            match self {
                Self::Local(owner) => owner
                    .submit_frame(input)
                    .map(|candidate| match candidate {
                        Some(candidate) => CandidateProduction::Candidate {
                            candidate: Box::new(candidate),
                            delivery: CandidateDelivery::PrivateCorrectionTransport,
                        },
                        None => CandidateProduction::NoCandidate,
                    })
                    .map_err(CorrectionRuntimeError::CandidateOwner),
                Self::NativePgo(native) => native.submit_frame(input, cancellation).await,
            }
        }

        async fn shutdown(self) -> Result<(), CorrectionRuntimeError> {
            match self {
                Self::Local(owner) => {
                    owner.shutdown();
                    Ok(())
                }
                Self::NativePgo(mut native) => {
                    let owner = native.owner.take().ok_or_else(|| {
                        CorrectionRuntimeError::NativePgo(
                            "native PGO retained owner was already consumed".to_owned(),
                        )
                    })?;
                    let shutdown = owner
                        .shutdown()
                        .await
                        .map_err(|error| CorrectionRuntimeError::NativePgo(error.to_string()))?;
                    match shutdown.cause {
                        OnlinePgoStopCause::Requested => Ok(()),
                        OnlinePgoStopCause::Terminal(error) => {
                            Err(CorrectionRuntimeError::NativePgo(error.to_string()))
                        }
                    }
                }
            }
        }
    }

    impl NativePgoCandidateOwner {
        async fn submit_frame(
            &mut self,
            input: CorrectionFrameInput,
            cancellation: &CorrectionRequestCancellation,
        ) -> Result<CandidateProduction, CorrectionRuntimeError> {
            let request = native_pgo_request(
                &self.identity,
                &self.map_frame_id,
                &self.odom_frame_id,
                &self.body_frame_id,
                &input,
            )?;
            let submit = self.ingress.submit(request);
            tokio::pin!(submit);
            let validated = tokio::select! {
                biased;
                _ = cancellation.cancelled() => {
                    let owner = self.owner.as_mut().ok_or_else(|| {
                        CorrectionRuntimeError::NativePgo(
                            "native PGO retained owner is unavailable".to_owned(),
                        )
                    })?;
                    owner
                        .cancel_in_flight()
                        .await
                        .map_err(|error| CorrectionRuntimeError::NativePgo(error.to_string()))?;
                    return Ok(CandidateProduction::Cancelled);
                }
                result = &mut submit => result
                    .map_err(|error| CorrectionRuntimeError::NativePgo(error.to_string()))?,
            };
            match validated.disposition() {
                ValidatedOnlinePgoDisposition::NoCandidate => Ok(CandidateProduction::NoCandidate),
                ValidatedOnlinePgoDisposition::Rejected => {
                    let result = validated.result();
                    Ok(CandidateProduction::Rejected {
                        reason_code: result.rejection_reason_code,
                        reason: result.rejection_reason.clone(),
                    })
                }
                ValidatedOnlinePgoDisposition::Correction => {
                    let result = validated.result();
                    let candidate_sequence =
                        u64::try_from(result.request.identity.request_sequence).map_err(|_| {
                            CorrectionRuntimeError::NativePgo(
                                "validated request sequence is not positive".to_owned(),
                            )
                        })?;
                    let transform = validated.transform().ok_or_else(|| {
                        CorrectionRuntimeError::NativePgo(
                            "validated correction lacks a transform".to_owned(),
                        )
                    })?;
                    let candidate = CorrectionCandidate::seal(
                        correction_candidate_identity(
                            CorrectionCandidateBackend::NativePgo,
                            &self.identity,
                            candidate_sequence,
                            &self.map_frame_id,
                            &self.odom_frame_id,
                            &self.body_frame_id,
                            &input,
                        ),
                        CorrectionTransform {
                            translation_m: transform.translation_m(),
                            rotation_xyzw: transform.rotation_xyzw(),
                        },
                    )
                    .map_err(|error| CorrectionRuntimeError::NativePgo(error.to_string()))?;
                    Ok(CandidateProduction::Candidate {
                        candidate: Box::new(candidate),
                        delivery: CandidateDelivery::DirectFinalizer,
                    })
                }
            }
        }
    }

    fn native_pgo_request(
        owner: &CorrectionOwnerRuntimeIdentity,
        map_frame_id: &str,
        odom_frame_id: &str,
        body_frame_id: &str,
        input: &CorrectionFrameInput,
    ) -> Result<native_pgo_codec::NativePgoRequest, CorrectionRuntimeError> {
        let checked_i64 = |label: &'static str, value: u64| {
            i64::try_from(value)
                .map_err(|_| CorrectionRuntimeError::NativePgo(format!("{label} exceeds i64")))
        };
        let coordinate_count = input
            .body_points
            .len()
            .checked_mul(3)
            .and_then(|count| i32::try_from(count).ok())
            .ok_or_else(|| {
                CorrectionRuntimeError::NativePgo(
                    "registered body coordinate count exceeds i32".to_owned(),
                )
            })?;
        let map_cell_count = i32::try_from(input.committed_map_occupancy.len()).map_err(|_| {
            CorrectionRuntimeError::NativePgo("committed map cell count exceeds i32".to_owned())
        })?;
        let mut registered_body_points_xyz = Vec::with_capacity(input.body_points.len() * 3);
        for point in &input.body_points {
            registered_body_points_xyz.extend_from_slice(&[point.x, point.y, point.z]);
        }
        let odom = CorrectionTransform::from_isometry(&input.correlated_odom.odom_from_body);
        Ok(native_pgo_codec::NativePgoRequest {
            schema_version: native_pgo_codec::NATIVE_PGO_SCHEMA_VERSION_V1,
            identity: native_pgo_codec::NativePgoRequestIdentity {
                backend: native_pgo_codec::NATIVE_PGO_BACKEND,
                runtime_generation: checked_i64("runtime generation", owner.runtime_generation)?,
                backend_generation: checked_i64("backend generation", owner.backend_generation)?,
                request_sequence: 0,
                mapping_session_id: input.mapping_session_id.clone(),
                mapping_session_generation: checked_i64(
                    "mapping session generation",
                    input.mapping_session_generation,
                )?,
                map_frame_generation: checked_i64(
                    "map frame generation",
                    input.map_frame_generation,
                )?,
                mapping_commit_id: native_pgo_codec::NativePgoMappingCommitId {
                    boot_nonce: input.mapping_commit_id.boot_nonce.to_be_bytes(),
                    sequence: checked_i64(
                        "mapping commit sequence",
                        input.mapping_commit_id.sequence,
                    )?,
                },
                occupancy_revision: checked_i64("occupancy revision", input.occupancy_revision)?,
                cloud_revision: checked_i64("cloud revision", input.cloud_revision)?,
                registered_cloud: native_pgo_codec::NativePgoRegisteredCloudProof {
                    source_sequence: checked_i64(
                        "registered cloud sequence",
                        input.registered_cloud.source_sequence,
                    )?,
                    source_timestamp_ns: checked_i64(
                        "registered cloud timestamp",
                        input.registered_cloud.source_timestamp_ns,
                    )?,
                    point_count: checked_i64(
                        "registered cloud point count",
                        input.registered_cloud.point_count,
                    )?,
                    envelope_sha256: input.registered_cloud.envelope_sha256,
                    body_points_sha256: input.registered_cloud.body_points_sha256,
                },
                correlated_odom: native_pgo_codec::NativePgoCorrelatedOdomProof {
                    source_sequence: checked_i64(
                        "correlated odometry sequence",
                        input.correlated_odom.proof.source_sequence,
                    )?,
                    source_timestamp_ns: checked_i64(
                        "correlated odometry timestamp",
                        input.correlated_odom.proof.source_timestamp_ns,
                    )?,
                    pose_sha256: input.correlated_odom.proof.pose_sha256,
                },
                committed_map: native_pgo_codec::NativePgoCommittedMapProof {
                    source_timestamp_ns: checked_i64(
                        "committed map timestamp",
                        input.committed_map.source_timestamp_ns,
                    )?,
                    width: checked_i64("committed map width", input.committed_map.width)?,
                    height: checked_i64("committed map height", input.committed_map.height)?,
                    resolution_m: input.committed_map.resolution_m,
                    origin_x_m: input.committed_map.origin_x_m,
                    origin_y_m: input.committed_map.origin_y_m,
                    payload_sha256: input.committed_map.payload_sha256,
                    boundary_sha256: input.committed_map.boundary_sha256,
                },
                source_timestamp_ns: checked_i64(
                    "source timestamp",
                    input.registered_cloud.source_timestamp_ns,
                )?,
                map_frame_id: map_frame_id.to_owned(),
                odom_frame_id: odom_frame_id.to_owned(),
                body_frame_id: body_frame_id.to_owned(),
                points_frame_id: input.points_frame_id.clone(),
                correction_lease: native_pgo_codec::NativePgoCorrectionLeaseId {
                    boot_nonce: owner.correction_lease.boot_nonce.to_be_bytes(),
                    generation: checked_i64(
                        "correction lease generation",
                        owner.correction_lease.generation,
                    )?,
                },
            },
            registered_body_coordinate_count: coordinate_count,
            registered_body_points_xyz,
            frozen_odom_translation_m: odom.translation_m,
            frozen_odom_rotation_xyzw: odom.rotation_xyzw,
            committed_map_cell_count: map_cell_count,
            committed_map_occupancy: input
                .committed_map_occupancy
                .iter()
                .map(|byte| *byte as i8)
                .collect(),
            request_payload_sha256: [0; 32],
        })
    }

    #[cfg(test)]
    #[derive(Default)]
    struct CandidatePublishGate {
        reached: tokio::sync::Notify,
        release: tokio::sync::Notify,
        publish_calls: std::sync::atomic::AtomicU64,
    }

    #[cfg(test)]
    #[derive(Default)]
    struct CandidateFinalizerOutcomeGate {
        reached: tokio::sync::Notify,
        release: tokio::sync::Notify,
    }

    /// Cloneable admission endpoint; the retained lifetime owner remains non-Clone.
    #[derive(Clone)]
    pub(crate) struct CorrectionFrameIngress {
        sender: mpsc::Sender<CorrectionFrameRequest>,
        admission_open: Arc<AtomicBool>,
        admission_rx: watch::Receiver<bool>,
        relocalization_mode: Arc<RelocalizationModeFence>,
    }

    impl CorrectionFrameIngress {
        pub(crate) async fn submit(
            &self,
            input: CorrectionFrameInput,
        ) -> Result<CorrectionSubmitDisposition, CorrectionRuntimeError> {
            self.submit_inner(input, None).await
        }

        pub(crate) async fn submit_with_lifecycle_guard(
            &self,
            input: CorrectionFrameInput,
            lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
        ) -> Result<CorrectionSubmitDisposition, CorrectionRuntimeError> {
            self.submit_inner(input, Some(lifecycle_guard)).await
        }

        async fn submit_inner(
            &self,
            input: CorrectionFrameInput,
            lifecycle_guard: Option<tokio::sync::OwnedMutexGuard<()>>,
        ) -> Result<CorrectionSubmitDisposition, CorrectionRuntimeError> {
            if !self.admission_open.load(Ordering::Acquire) {
                return Err(CorrectionRuntimeError::Stopped);
            }
            let current_mode = self.relocalization_mode.snapshot();
            if current_mode.epoch != input.frame_capture_relocalization_epoch
                || input.relocalization_mode_epoch.is_some()
                    && input.relocalization_mode_epoch
                        != Some(input.frame_capture_relocalization_epoch)
            {
                // The producer crossed a mode edge after capturing this
                // frame. Do not enqueue it where it could stage evidence
                // or produce a correction for the next command.
                return Ok(CorrectionSubmitDisposition::NoCandidate);
            }
            let cancellation = CorrectionRequestCancellation::new();
            let mut cancellation_guard = SubmitCancellationGuard::new(cancellation.clone());
            let lease = Arc::new(CorrectionRequestLease::new(
                cancellation.clone(),
                lifecycle_guard,
            ));
            let (response, response_rx) = oneshot::channel();
            let request = CorrectionFrameRequest {
                input,
                response,
                cancellation,
                lease,
            };
            let mut admission_rx = self.admission_rx.clone();
            let permit = tokio::select! {
                biased;
                _ = wait_for_admission_close(&mut admission_rx) => {
                    return Err(CorrectionRuntimeError::Stopped);
                }
                result = self.sender.reserve() => {
                    result.map_err(|_| CorrectionRuntimeError::IngressClosed)?
                }
            };
            {
                let _admission_guard = correction_frame_admission_fence().lock();
                if !self.admission_open.load(Ordering::Acquire) {
                    return Err(CorrectionRuntimeError::Stopped);
                }
                permit.send(request);
            }
            let mut response_rx = response_rx;
            tokio::select! {
                biased;
                result = &mut response_rx => {
                    match result {
                        Ok(result) => {
                            cancellation_guard.disarm();
                            result
                        }
                        Err(_) => Err(CorrectionRuntimeError::ResponseCanceled),
                    }
                }
                _ = wait_for_admission_close(&mut admission_rx) => {
                    cancellation_guard.cancellation.cancel();
                    match response_rx.await {
                        Ok(result) => {
                            cancellation_guard.disarm();
                            result
                        }
                        Err(_) => Err(CorrectionRuntimeError::Stopped),
                    }
                }
            }
        }

        pub(crate) fn is_open(&self) -> bool {
            self.admission_open.load(Ordering::Acquire)
        }

        pub(crate) fn relocalization_mode_snapshot(&self) -> RelocalizationModeSnapshot {
            self.relocalization_mode.snapshot()
        }

        pub(crate) fn relocalization_mode_fence(&self) -> Arc<RelocalizationModeFence> {
            Arc::clone(&self.relocalization_mode)
        }
    }

    async fn wait_for_admission_close(admission_rx: &mut watch::Receiver<bool>) {
        while *admission_rx.borrow_and_update() {
            if admission_rx.changed().await.is_err() {
                return;
            }
        }
    }

    async fn wait_for_shutdown(shutdown_rx: &mut watch::Receiver<bool>) {
        while !*shutdown_rx.borrow_and_update() {
            if shutdown_rx.changed().await.is_err() {
                return;
            }
        }
    }

    struct CorrectionCleanupJob {
        input: CorrectionFrameInput,
    }

    #[derive(Clone)]
    struct CorrectionCleanupIngress {
        sender: mpsc::Sender<CorrectionCleanupJob>,
        sink: Arc<dyn CorrectionCandidateSink>,
    }

    struct CorrectionCleanupSupervisor {
        ingress: CorrectionCleanupIngress,
        shutdown_tx: watch::Sender<bool>,
        worker: Option<tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
    }

    #[allow(dead_code)]
    impl CorrectionCleanupSupervisor {
        fn start(sink: Arc<dyn CorrectionCandidateSink>) -> Self {
            let (sender, receiver) = mpsc::channel(CORRECTION_CLEANUP_QUEUE_CAPACITY);
            let (shutdown_tx, shutdown_rx) = watch::channel(false);
            let worker = tokio::spawn(run_correction_cleanup_worker(
                receiver,
                shutdown_rx,
                Arc::clone(&sink),
            ));
            Self {
                ingress: CorrectionCleanupIngress { sender, sink },
                shutdown_tx,
                worker: Some(worker),
            }
        }

        fn ingress(&self) -> CorrectionCleanupIngress {
            self.ingress.clone()
        }

        fn is_finished(&self) -> bool {
            self.worker
                .as_ref()
                .is_some_and(tokio::task::JoinHandle::is_finished)
        }

        fn request_shutdown(&self) {
            self.shutdown_tx.send_replace(true);
        }

        async fn shutdown(&mut self) -> Result<(), CorrectionRuntimeError> {
            self.request_shutdown();
            let Some(worker) = self.worker.as_mut() else {
                return Ok(());
            };
            let deadline = Instant::now() + CORRECTION_WORKER_STOP_TIMEOUT;
            match settle_retained_correction_worker("correction cleanup", worker, deadline).await {
                Some(result) => {
                    self.worker.take();
                    result
                }
                None => Err(CorrectionRuntimeError::WorkerStopTimeout(
                    "correction cleanup",
                )),
            }
        }
    }

    impl Drop for CorrectionCleanupSupervisor {
        fn drop(&mut self) {
            self.request_shutdown();
        }
    }

    async fn schedule_discard(
        cleanup: &CorrectionCleanupIngress,
        input: CorrectionFrameInput,
    ) -> Result<(), CorrectionRuntimeError> {
        let job = CorrectionCleanupJob { input };
        if let Err(error) = cleanup.sender.send(job).await {
            let job = error.0;
            tokio::time::timeout(
                CORRECTION_DISCARD_TIMEOUT,
                cleanup.sink.discard_frame(&job.input),
            )
            .await
            .map_err(|_| CorrectionRuntimeError::WorkerStopTimeout("correction cleanup"))?;
        }
        Ok(())
    }

    async fn run_correction_cleanup_worker(
        mut jobs: mpsc::Receiver<CorrectionCleanupJob>,
        mut shutdown_rx: watch::Receiver<bool>,
        sink: Arc<dyn CorrectionCandidateSink>,
    ) -> Result<(), CorrectionRuntimeError> {
        loop {
            let job = tokio::select! {
                biased;
                _ = wait_for_shutdown(&mut shutdown_rx) => {
                    jobs.close();
                    while let Some(job) = jobs.recv().await {
                        tokio::time::timeout(
                            CORRECTION_DISCARD_TIMEOUT,
                            sink.discard_frame(&job.input),
                        )
                        .await
                        .map_err(|_| CorrectionRuntimeError::WorkerStopTimeout("correction cleanup"))?;
                    }
                    return Ok(());
                }
                job = jobs.recv() => match job {
                    Some(job) => job,
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
            };
            tokio::time::timeout(CORRECTION_DISCARD_TIMEOUT, sink.discard_frame(&job.input))
                .await
                .map_err(|_| CorrectionRuntimeError::WorkerStopTimeout("correction cleanup"))?;
        }
    }

    struct CanonicalFinalizerJob {
        key: CandidateApplyKey,
        candidate: CorrectionCandidate,
        lease: Arc<CorrectionRequestLease>,
        release_lease_on_completion: bool,
        outcome: oneshot::Sender<Result<(), String>>,
    }

    #[derive(Clone)]
    struct CanonicalPublishFinalizerIngress {
        sender: mpsc::Sender<CanonicalFinalizerJob>,
    }

    struct CanonicalPublishFinalizerSupervisor {
        ingress: CanonicalPublishFinalizerIngress,
        shutdown_tx: watch::Sender<bool>,
        worker: Option<tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
    }

    impl CanonicalPublishFinalizerSupervisor {
        fn start(
            sink: Arc<dyn CorrectionCandidateSink>,
            apply_waiters: Arc<CandidateApplyWaiters>,
        ) -> Self {
            let (job_tx, job_rx) = mpsc::channel(CORRECTION_INPUT_QUEUE_CAPACITY);
            let (shutdown_tx, shutdown_rx) = watch::channel(false);
            let worker = tokio::spawn(run_canonical_publish_finalizer(
                job_rx,
                shutdown_rx,
                sink,
                apply_waiters,
            ));
            Self {
                ingress: CanonicalPublishFinalizerIngress { sender: job_tx },
                shutdown_tx,
                worker: Some(worker),
            }
        }

        fn ingress(&self) -> CanonicalPublishFinalizerIngress {
            self.ingress.clone()
        }

        fn is_finished(&self) -> bool {
            self.worker
                .as_ref()
                .is_some_and(tokio::task::JoinHandle::is_finished)
        }

        fn request_shutdown(&self) {
            self.shutdown_tx.send_replace(true);
        }

        async fn shutdown(&mut self) -> Result<(), CorrectionRuntimeError> {
            self.request_shutdown();
            let Some(worker) = self.worker.as_mut() else {
                return Ok(());
            };
            let deadline = Instant::now() + CORRECTION_WORKER_STOP_TIMEOUT;
            match settle_retained_correction_worker("canonical finalizer", worker, deadline).await {
                Some(result) => {
                    self.worker.take();
                    result
                }
                None => Err(CorrectionRuntimeError::WorkerStopTimeout(
                    "canonical finalizer",
                )),
            }
        }
    }

    async fn wait_for_operation_poison(phase_rx: &mut watch::Receiver<CanonicalOperationPhase>) {
        loop {
            if *phase_rx.borrow_and_update() == CanonicalOperationPhase::Poisoned {
                return;
            }
            if phase_rx.changed().await.is_err() {
                return;
            }
        }
    }

    async fn run_canonical_finalizer_job(
        job: CanonicalFinalizerJob,
        sink: Arc<dyn CorrectionCandidateSink>,
        apply_waiters: &CandidateApplyWaiters,
    ) -> Result<(), CorrectionRuntimeError> {
        let cancellation = job.lease.cancellation();
        let operation = cancellation.operation();
        let mut phase_rx = operation.subscribe();
        let operation_sink = Arc::clone(&sink);
        let operation_cancellation = cancellation.clone();
        let candidate = job.candidate;
        let mut worker = tokio::spawn(async move {
            operation_sink
                .apply_candidate(candidate, operation_cancellation)
                .await
        });
        let joined = tokio::select! {
            biased;
            result = &mut worker => result,
            _ = wait_for_operation_poison(&mut phase_rx) => {
                worker.abort();
                let _ = worker.await;
                let detail = operation.poison_detail();
                apply_waiters.complete(job.key, Err(detail.clone()));
                let _ = job.outcome.send(Err(detail.clone()));
                std::mem::forget(job.lease);
                return Err(CorrectionRuntimeError::FinalizerPoisoned(detail));
            }
        };

        let result = match joined {
            Ok(result) => result,
            Err(join_error)
                if operation.phase().is_post_claim() && !operation.is_known_finalized() =>
            {
                let detail = format!("post-claim canonical operation join failed: {join_error}");
                operation.poison(detail.clone());
                operation.disarm_watchdog();
                apply_waiters.complete(job.key, Err(detail.clone()));
                let _ = job.outcome.send(Err(detail.clone()));
                std::mem::forget(job.lease);
                return Err(CorrectionRuntimeError::FinalizerPoisoned(detail));
            }
            Err(join_error) => {
                if !operation.phase().is_post_claim() {
                    operation.cancel_pending();
                    sink.recover_abandoned_operation(&cancellation).await;
                }
                let detail = format!("canonical operation join failed: {join_error}");
                if job.release_lease_on_completion {
                    job.lease.release();
                }
                apply_waiters.complete(job.key, Err(detail.clone()));
                let _ = job.outcome.send(Err(detail.clone()));
                return Err(CorrectionRuntimeError::WorkerJoin(detail));
            }
        };

        let phase = operation.phase();
        if phase.is_post_claim() && !operation.is_known_finalized() {
            let detail = format!("canonical operation returned in unsafe phase {phase:?}");
            operation.poison(detail.clone());
            operation.disarm_watchdog();
            apply_waiters.complete(job.key, Err(detail.clone()));
            let _ = job.outcome.send(Err(detail.clone()));
            std::mem::forget(job.lease);
            return Err(CorrectionRuntimeError::FinalizerPoisoned(detail));
        }
        if phase == CanonicalOperationPhase::Pending {
            operation.cancel_pending();
        }
        if job.release_lease_on_completion {
            job.lease.release();
        }
        apply_waiters.complete(job.key, result.clone());
        match result {
            Ok(_) => {
                let _ = job.outcome.send(Ok(()));
                Ok(())
            }
            Err(error) => {
                let _ = job.outcome.send(Err(error.clone()));
                Err(CorrectionRuntimeError::Apply(error))
            }
        }
    }

    async fn settle_queued_finalizer_jobs(
        jobs: &mut mpsc::Receiver<CanonicalFinalizerJob>,
        apply_waiters: &CandidateApplyWaiters,
        detail: &str,
    ) {
        jobs.close();
        while let Some(job) = jobs.recv().await {
            job.lease.cancellation().operation().cancel_pending();
            if job.release_lease_on_completion {
                job.lease.release();
            }
            apply_waiters.complete(job.key, Err(detail.to_owned()));
            let _ = job.outcome.send(Err(detail.to_owned()));
        }
    }

    async fn run_canonical_publish_finalizer(
        mut jobs: mpsc::Receiver<CanonicalFinalizerJob>,
        mut shutdown_rx: watch::Receiver<bool>,
        sink: Arc<dyn CorrectionCandidateSink>,
        apply_waiters: Arc<CandidateApplyWaiters>,
    ) -> Result<(), CorrectionRuntimeError> {
        loop {
            let job = tokio::select! {
                biased;
                _ = wait_for_shutdown(&mut shutdown_rx) => return Ok(()),
                job = jobs.recv() => match job {
                    Some(job) => job,
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
            };
            if let Err(error) =
                run_canonical_finalizer_job(job, Arc::clone(&sink), &apply_waiters).await
            {
                settle_queued_finalizer_jobs(&mut jobs, &apply_waiters, &error.to_string()).await;
                return Err(error);
            }
        }
    }

    #[derive(Debug, Error, Clone, PartialEq, Eq)]
    pub(crate) enum RelocalizationCommandError {
        #[error("relocalization command admission is closed")]
        Stopped,
        #[error("relocalization command owner is unavailable")]
        OwnerUnavailable,
        #[error("relocalization command admission response was lost")]
        ResponseLost,
    }

    struct RelocalizationWaiterSignal {
        detached: AtomicBool,
        waiter_id: u64,
        detached_tx: mpsc::UnboundedSender<RelocalizationWaiterEvent>,
    }

    struct RelocalizationWaiterCapacityLease {
        permit: parking_lot::Mutex<Option<OwnedSemaphorePermit>>,
    }

    impl RelocalizationWaiterCapacityLease {
        fn new(permit: OwnedSemaphorePermit) -> Self {
            Self {
                permit: parking_lot::Mutex::new(Some(permit)),
            }
        }

        fn release(&self) {
            self.permit.lock().take();
        }
    }

    enum RelocalizationWaiterEvent {
        Detached(u64),
    }

    pub(crate) struct RelocalizationCommandWaiter {
        response: Option<oneshot::Receiver<crate::proto::RelocalizeResult>>,
        signal: Arc<RelocalizationWaiterSignal>,
        _capacity_lease: Arc<RelocalizationWaiterCapacityLease>,
        completed: bool,
    }

    impl RelocalizationCommandWaiter {
        pub(crate) async fn wait(
            mut self,
        ) -> Result<crate::proto::RelocalizeResult, RelocalizationCommandError> {
            let response = self
                .response
                .take()
                .ok_or(RelocalizationCommandError::ResponseLost)?;
            let result = response
                .await
                .map_err(|_| RelocalizationCommandError::ResponseLost)?;
            self.completed = true;
            Ok(result)
        }

        pub(crate) fn cancel_preclaim(&self) {
            if !self.signal.detached.swap(true, Ordering::AcqRel) {
                let _ = self
                    .signal
                    .detached_tx
                    .send(RelocalizationWaiterEvent::Detached(self.signal.waiter_id));
            }
        }
    }

    impl Drop for RelocalizationCommandWaiter {
        fn drop(&mut self) {
            if !self.completed && !self.signal.detached.swap(true, Ordering::AcqRel) {
                let _ = self
                    .signal
                    .detached_tx
                    .send(RelocalizationWaiterEvent::Detached(self.signal.waiter_id));
            }
        }
    }

    struct RelocalizationAdmissionRequest {
        request: crate::proto::RelocalizeRequest,
        capacity_lease: Arc<RelocalizationWaiterCapacityLease>,
        response: oneshot::Sender<Result<RelocalizationCommandWaiter, RelocalizationCommandError>>,
        mapping_lifecycle_guard: Option<tokio::sync::OwnedMutexGuard<()>>,
    }

    struct RelocalizationTerminalQueryRequest {
        request_id: String,
        request_identity_sha256: [u8; 32],
        activation_identity_sha256: [u8; 32],
        response: oneshot::Sender<Result<RelocalizationTerminalQuerySnapshot, &'static str>>,
    }

    #[derive(Clone)]
    pub(crate) struct RelocalizationCommandIngress {
        sender: mpsc::Sender<RelocalizationAdmissionRequest>,
        terminal_query_sender: mpsc::Sender<RelocalizationTerminalQueryRequest>,
        terminal_store: RelocalizationTerminalStore,
        admission_open: Arc<AtomicBool>,
        admission_rx: watch::Receiver<bool>,
        waiter_capacity: Arc<Semaphore>,
    }

    impl RelocalizationCommandIngress {
        pub(crate) async fn admit(
            &self,
            request: crate::proto::RelocalizeRequest,
        ) -> Result<RelocalizationCommandWaiter, RelocalizationCommandError> {
            self.admit_inner(request, None).await
        }

        pub(crate) async fn admit_with_lifecycle_guard(
            &self,
            request: crate::proto::RelocalizeRequest,
            lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
        ) -> Result<RelocalizationCommandWaiter, RelocalizationCommandError> {
            self.admit_inner(request, Some(lifecycle_guard)).await
        }

        async fn admit_inner(
            &self,
            request: crate::proto::RelocalizeRequest,
            mapping_lifecycle_guard: Option<tokio::sync::OwnedMutexGuard<()>>,
        ) -> Result<RelocalizationCommandWaiter, RelocalizationCommandError> {
            if !self.admission_open.load(Ordering::Acquire) {
                return Err(RelocalizationCommandError::Stopped);
            }
            let mut capacity_admission_rx = self.admission_rx.clone();
            let waiter_permit = tokio::select! {
                biased;
                _ = wait_for_admission_close(&mut capacity_admission_rx) => {
                    return Err(RelocalizationCommandError::Stopped);
                }
                result = Arc::clone(&self.waiter_capacity).acquire_owned() => {
                    result.map_err(|_| RelocalizationCommandError::OwnerUnavailable)?
                }
            };
            let capacity_lease = Arc::new(RelocalizationWaiterCapacityLease::new(waiter_permit));
            let (response, response_rx) = oneshot::channel();
            let admission = RelocalizationAdmissionRequest {
                request,
                capacity_lease,
                response,
                mapping_lifecycle_guard,
            };
            let mut admission_rx = self.admission_rx.clone();
            tokio::select! {
                biased;
                _ = wait_for_admission_close(&mut admission_rx) => {
                    return Err(RelocalizationCommandError::Stopped);
                }
                result = self.sender.send(admission) => {
                    result.map_err(|_| RelocalizationCommandError::OwnerUnavailable)?;
                }
            }
            tokio::select! {
                biased;
                result = response_rx => {
                    result.map_err(|_| RelocalizationCommandError::ResponseLost)?
                }
                _ = wait_for_admission_close(&mut admission_rx) => {
                    Err(RelocalizationCommandError::Stopped)
                }
            }
        }

        pub(crate) async fn terminal_snapshot(
            &self,
            request_id: &str,
            request_identity_sha256: [u8; 32],
            activation_identity_sha256: [u8; 32],
        ) -> Result<RelocalizationTerminalQuerySnapshot, &'static str> {
            let (response, response_rx) = oneshot::channel();
            self.terminal_query_sender
                .send(RelocalizationTerminalQueryRequest {
                    request_id: request_id.to_owned(),
                    request_identity_sha256,
                    activation_identity_sha256,
                    response,
                })
                .await
                .map_err(|_| "RELOCALIZATION_TERMINAL_QUERY_OWNER_UNAVAILABLE")?;
            response_rx
                .await
                .map_err(|_| "RELOCALIZATION_TERMINAL_QUERY_RESPONSE_LOST")?
        }

        #[cfg(test)]
        pub(crate) fn terminal_query_fixture_for_tests(root: PathBuf) -> Self {
            let (sender, _receiver) = mpsc::channel(1);
            let (terminal_query_sender, mut terminal_query_receiver) =
                mpsc::channel::<RelocalizationTerminalQueryRequest>(1);
            let terminal_store = RelocalizationTerminalStore::open_at(root)
                .expect("open terminal query fixture store");
            let query_store = terminal_store.clone();
            tokio::spawn(async move {
                while let Some(query) = terminal_query_receiver.recv().await {
                    let terminal = query_store.snapshot(
                        &query.request_id,
                        query.request_identity_sha256,
                        query.activation_identity_sha256,
                    );
                    let _ = query.response.send(Ok(RelocalizationTerminalQuerySnapshot {
                        terminal,
                        ledger_observation: crate::proto::RelocalizationLedgerObservation {
                            observation_schema_version: 1,
                            observation_epoch: 1,
                            ..Default::default()
                        },
                    }));
                }
            });
            let (_admission_tx, admission_rx) = watch::channel(true);
            Self {
                sender,
                terminal_query_sender,
                terminal_store,
                admission_open: Arc::new(AtomicBool::new(true)),
                admission_rx,
                waiter_capacity: Arc::new(Semaphore::new(RELOCALIZATION_MAX_WAITERS)),
            }
        }

        #[cfg(test)]
        pub(crate) fn mark_terminal_query_pending_for_tests(
            &self,
            request_id: &str,
            request_identity_sha256: [u8; 32],
            activation_identity_sha256: [u8; 32],
        ) {
            self.terminal_store.state.lock().pending.insert(
                request_id.to_owned(),
                (request_identity_sha256, activation_identity_sha256),
            );
        }

        #[cfg(test)]
        pub(crate) fn persist_terminal_query_result_for_tests(
            &self,
            result: crate::proto::RelocalizeResult,
        ) {
            let request_identity_sha256: [u8; 32] = result
                .request_identity_sha256
                .as_slice()
                .try_into()
                .expect("request identity");
            let activation_identity_sha256: [u8; 32] = result
                .activation_identity_sha256
                .as_slice()
                .try_into()
                .expect("activation identity");
            let request_id = result.request_id.clone();
            let terminal = StoredRelocalizationResult::new(result);
            self.terminal_store
                .persist_terminal(
                    &request_id,
                    request_identity_sha256,
                    activation_identity_sha256,
                    &terminal,
                )
                .expect("persist terminal query fixture result");
        }

        pub(crate) fn is_open(&self) -> bool {
            self.admission_open.load(Ordering::Acquire)
        }
    }

    #[derive(Clone)]
    struct ValidatedRelocalizeRequest {
        request: crate::proto::RelocalizeRequest,
        identity_sha256: [u8; 32],
        activation_identity_sha256: [u8; 32],
        target_receipt_sha256: [u8; 32],
        target_cloud_sha256: [u8; 32],
        asset_manifest_sha256: [u8; 32],
        asset_bundle_sha256: [u8; 32],
        source_policy_sha256: [u8; 32],
        deadline_policy_sha256: [u8; 32],
        algorithm_options_sha256: [u8; 32],
    }

    fn relocalization_digest(value: &[u8], field: &'static str) -> Result<[u8; 32], String> {
        value
            .try_into()
            .map_err(|_| format!("{field} must contain exactly 32 bytes"))
    }

    fn validate_relocalization_string(field: &'static str, value: &str) -> Result<(), String> {
        if value.is_empty()
            || value.trim() != value
            || value.len() > MAX_IDENTITY_STRING_BYTES
            || value.contains('\0')
        {
            return Err(format!("{field} is not a canonical identity string"));
        }
        Ok(())
    }

    pub(crate) fn validate_relocalization_request_id(value: &str) -> Result<(), String> {
        if value.is_empty()
            || value.len() > 128
            || !value.bytes().all(|byte| {
                byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b':' | b'-')
            })
        {
            return Err("RELOCALIZATION_REQUEST_ID_INVALID".to_owned());
        }
        Ok(())
    }

    fn normalize_relocalization_logical_timeout_ms(value: u64) -> Result<u64, String> {
        match value {
            0 => Ok(RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS),
            RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS..=RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS => {
                Ok(value)
            }
            _ => Err("RELOCALIZATION_LOGICAL_TIMEOUT_INVALID".to_owned()),
        }
    }

    fn canonical_relocalization_request_preimage(
        request: &crate::proto::RelocalizeRequest,
    ) -> Result<Vec<u8>, String> {
        validate_relocalization_request_id(&request.request_id)?;
        validate_relocalization_string("session_id", &request.session_id)?;
        validate_relocalization_string("map_name", &request.map_name)?;
        let activation_identity =
            relocalization_digest(&request.activation_identity_sha256, "activation identity")?;
        let target_receipt =
            relocalization_digest(&request.target_receipt_sha256, "target receipt")?;
        let target_cloud = relocalization_digest(&request.target_cloud_sha256, "target cloud")?;
        let asset_manifest =
            relocalization_digest(&request.asset_manifest_sha256, "asset manifest")?;
        let asset_bundle = relocalization_digest(&request.asset_bundle_sha256, "asset bundle")?;
        let source_policy = relocalization_digest(&request.source_policy_sha256, "source policy")?;
        let deadline_policy =
            relocalization_digest(&request.deadline_policy_sha256, "deadline policy")?;
        let algorithm_options =
            relocalization_digest(&request.algorithm_options_sha256, "algorithm options")?;

        let mut out = b"nuwax.geometry.relocalize.request.v1\0".to_vec();
        out.extend_from_slice(&request.schema_version.to_be_bytes());
        canonical_push_string(&mut out, &request.request_id);
        out.extend_from_slice(&request.runtime_generation.to_be_bytes());
        canonical_push_string(&mut out, &request.session_id);
        canonical_push_string(&mut out, &request.map_name);
        out.extend_from_slice(&request.session_generation.to_be_bytes());
        out.extend_from_slice(&request.backend_generation.to_be_bytes());
        out.extend_from_slice(&activation_identity);
        out.extend_from_slice(&target_receipt);
        out.extend_from_slice(&request.target_cloud_revision.to_be_bytes());
        out.extend_from_slice(&target_cloud);
        out.extend_from_slice(&asset_manifest);
        out.extend_from_slice(&asset_bundle);
        out.extend_from_slice(&request.source_policy_version.to_be_bytes());
        out.extend_from_slice(&source_policy);
        out.extend_from_slice(&request.deadline_policy_version.to_be_bytes());
        out.extend_from_slice(&deadline_policy);
        out.extend_from_slice(
            &normalize_relocalization_logical_timeout_ms(request.logical_timeout_ms)?.to_be_bytes(),
        );
        out.extend_from_slice(&request.algorithm_options_version.to_be_bytes());
        out.extend_from_slice(&algorithm_options);
        Ok(out)
    }

    fn validate_relocalize_request(
        mut request: crate::proto::RelocalizeRequest,
    ) -> Result<ValidatedRelocalizeRequest, String> {
        request.logical_timeout_ms =
            normalize_relocalization_logical_timeout_ms(request.logical_timeout_ms)?;
        let preimage = canonical_relocalization_request_preimage(&request)?;
        if request.schema_version != RELOCALIZATION_SCHEMA_VERSION_V1 {
            return Err("RELOCALIZATION_SCHEMA_VERSION_UNSUPPORTED".to_owned());
        }
        if request.runtime_generation == 0
            || request.session_generation == 0
            || request.backend_generation == 0
            || request.target_cloud_revision == 0
        {
            return Err("RELOCALIZATION_IDENTITY_GENERATION_INVALID".to_owned());
        }
        let activation_identity_sha256 =
            relocalization_digest(&request.activation_identity_sha256, "activation identity")?;
        let target_receipt_sha256 =
            relocalization_digest(&request.target_receipt_sha256, "target receipt")?;
        let target_cloud_sha256 =
            relocalization_digest(&request.target_cloud_sha256, "target cloud")?;
        let asset_manifest_sha256 =
            relocalization_digest(&request.asset_manifest_sha256, "asset manifest")?;
        let asset_bundle_sha256 =
            relocalization_digest(&request.asset_bundle_sha256, "asset bundle")?;
        let source_policy_sha256 =
            relocalization_digest(&request.source_policy_sha256, "source policy")?;
        let deadline_policy_sha256 =
            relocalization_digest(&request.deadline_policy_sha256, "deadline policy")?;
        let algorithm_options_sha256 =
            relocalization_digest(&request.algorithm_options_sha256, "algorithm options")?;
        if request.source_policy_version != RELOCALIZATION_SOURCE_POLICY_VERSION_V1
            || source_policy_sha256 != relocalization_source_policy_sha256_v1()
            || request.deadline_policy_version != RELOCALIZATION_DEADLINE_POLICY_VERSION_V1
            || deadline_policy_sha256 != relocalization_deadline_policy_sha256_v1()
            || request.algorithm_options_version != RELOCALIZATION_ALGORITHM_OPTIONS_VERSION_V1
            || algorithm_options_sha256 != relocalization_algorithm_options_sha256_v1()
        {
            return Err("RELOCALIZATION_POLICY_IDENTITY_MISMATCH".to_owned());
        }
        Ok(ValidatedRelocalizeRequest {
            request,
            identity_sha256: sha256_bytes(&preimage),
            activation_identity_sha256,
            target_receipt_sha256,
            target_cloud_sha256,
            asset_manifest_sha256,
            asset_bundle_sha256,
            source_policy_sha256,
            deadline_policy_sha256,
            algorithm_options_sha256,
        })
    }

    fn invalid_relocalization_request_identity(
        request: &crate::proto::RelocalizeRequest,
    ) -> [u8; 32] {
        let mut bytes = b"nuwax.geometry.relocalize.invalid-request.v1\0".to_vec();
        bytes.extend_from_slice(&request.encode_to_vec());
        sha256_bytes(&bytes)
    }

    #[derive(Clone)]
    struct StoredRelocalizationResult {
        result: crate::proto::RelocalizeResult,
        encoded: Vec<u8>,
        encoded_sha256: [u8; 32],
    }

    impl StoredRelocalizationResult {
        fn new(result: crate::proto::RelocalizeResult) -> Self {
            let encoded = result.encode_to_vec();
            let encoded_sha256 = sha256_bytes(&encoded);
            Self {
                result,
                encoded,
                encoded_sha256,
            }
        }
    }

    #[allow(clippy::large_enum_variant)]
    #[derive(Clone, Debug)]
    pub(crate) enum RelocalizationTerminalSnapshot {
        Terminal(crate::proto::RelocalizeResult),
        Pending,
        NotFound,
        IdentityConflict,
    }

    pub(crate) struct RelocalizationTerminalQuerySnapshot {
        pub(crate) terminal: RelocalizationTerminalSnapshot,
        pub(crate) ledger_observation: crate::proto::RelocalizationLedgerObservation,
    }

    #[derive(Serialize, Deserialize)]
    struct DurableRelocalizationTerminalV1 {
        schema_version: u32,
        request_id: String,
        request_identity_sha256: String,
        activation_identity_sha256: String,
        terminal_hex: String,
        terminal_sha256: String,
    }

    #[derive(Clone)]
    struct DurableRelocalizationTerminal {
        request_identity_sha256: [u8; 32],
        activation_identity_sha256: [u8; 32],
        terminal: StoredRelocalizationResult,
    }

    struct RelocalizationTerminalStoreState {
        terminals: HashMap<String, DurableRelocalizationTerminal>,
        pending: HashMap<String, ([u8; 32], [u8; 32])>,
    }

    struct RelocalizationTerminalOwnerLease {
        root: PathBuf,
        owner_token: String,
        _lock: Flock<File>,
    }

    impl Drop for RelocalizationTerminalOwnerLease {
        fn drop(&mut self) {
            let cleanup_lock = match lock_terminal_cleanup(&self.root) {
                Ok(lock) => lock,
                Err(error) => {
                    tracing::warn!(error = %error, "terminal owner lease cleanup deferred");
                    return;
                }
            };
            if terminal_owner_has_temporary(&self.root, &self.owner_token) != Ok(false) {
                drop(cleanup_lock);
                return;
            }
            let lease_path = self.root.join(terminal_lease_name(&self.owner_token));
            match fs::remove_file(lease_path) {
                Ok(()) => {
                    if let Err(error) = fsync_directory(&self.root) {
                        tracing::warn!(error = %error, "terminal owner lease cleanup deferred");
                    }
                }
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => {
                    tracing::warn!(error = %error, "terminal owner lease cleanup deferred");
                }
            }
        }
    }

    #[derive(Clone)]
    pub(crate) struct RelocalizationTerminalStore {
        root: Arc<PathBuf>,
        owner_token: Arc<String>,
        _owner_lease: Arc<RelocalizationTerminalOwnerLease>,
        state: Arc<parking_lot::Mutex<RelocalizationTerminalStoreState>>,
        #[cfg(test)]
        _test_root: Option<Arc<tests::RelocalizationTerminalTestRoot>>,
    }

    fn encode_hex(bytes: &[u8]) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(bytes.len() * 2);
        for byte in bytes {
            out.push(char::from(HEX[usize::from(byte >> 4)]));
            out.push(char::from(HEX[usize::from(byte & 0x0f)]));
        }
        out
    }

    fn decode_hex(value: &str, max_bytes: usize) -> Result<Vec<u8>, String> {
        if value.len() % 2 != 0 || value.len() / 2 > max_bytes {
            return Err("RELOCALIZATION_TERMINAL_STORE_HEX_INVALID".to_owned());
        }
        fn digit(value: u8) -> Option<u8> {
            match value {
                b'0'..=b'9' => Some(value - b'0'),
                b'a'..=b'f' => Some(value - b'a' + 10),
                _ => None,
            }
        }
        value
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let high = digit(pair[0])
                    .ok_or_else(|| "RELOCALIZATION_TERMINAL_STORE_HEX_INVALID".to_owned())?;
                let low = digit(pair[1])
                    .ok_or_else(|| "RELOCALIZATION_TERMINAL_STORE_HEX_INVALID".to_owned())?;
                Ok((high << 4) | low)
            })
            .collect()
    }

    fn read_terminal_record_bytes(
        mut reader: impl Read,
        metadata_len: u64,
    ) -> Result<Vec<u8>, String> {
        if metadata_len > RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES as u64 {
            return Err("RELOCALIZATION_TERMINAL_STORE_RECORD_TOO_LARGE".to_owned());
        }
        let mut bytes = Vec::new();
        reader
            .by_ref()
            .take((RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?;
        if bytes.len() > RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES {
            return Err("RELOCALIZATION_TERMINAL_STORE_RECORD_TOO_LARGE".to_owned());
        }
        Ok(bytes)
    }

    fn read_terminal_record(path: &Path) -> Result<Vec<u8>, String> {
        let file = File::open(path)
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?;
        let metadata_len = file
            .metadata()
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?
            .len();
        read_terminal_record_bytes(file, metadata_len)
    }

    fn terminal_store_root_from(
        explicit: Option<OsString>,
        state_directory: Option<OsString>,
        runtime_data: Option<OsString>,
        current_directory: impl FnOnce() -> std::io::Result<PathBuf>,
    ) -> Result<PathBuf, String> {
        let root = if let Some(value) = explicit {
            let root = PathBuf::from(value);
            if root.as_os_str().is_empty() {
                return Err("RELOCALIZATION_TERMINAL_STORE_PATH_INVALID".to_owned());
            }
            root
        } else if let Some(value) = state_directory {
            let root = std::env::split_paths(&value)
                .next()
                .ok_or_else(|| "RELOCALIZATION_TERMINAL_STORE_PATH_INVALID".to_owned())?;
            return if root.is_absolute() {
                Ok(root.join("relocalization_terminals_v1"))
            } else {
                Err("RELOCALIZATION_TERMINAL_STORE_PATH_NOT_ABSOLUTE".to_owned())
            };
        } else {
            let root = PathBuf::from(runtime_data.unwrap_or_else(|| OsString::from("runtime_data")));
            if root.as_os_str().is_empty() {
                return Err("RELOCALIZATION_TERMINAL_STORE_PATH_INVALID".to_owned());
            }
            root.join("geometry").join("relocalization_terminals_v1")
        };
        if root.is_absolute() {
            return Ok(root);
        }
        // 仅相对路径需要当前目录；绝对部署路径不增加工作目录可访问性的前置。
        let current_directory = current_directory()
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_CWD_UNAVAILABLE:{error}"))?;
        Ok(current_directory.join(root))
    }

    fn terminal_store_root() -> Result<PathBuf, String> {
        terminal_store_root_from(
            std::env::var_os(RELOCALIZATION_TERMINAL_STORE_DIR_ENV),
            std::env::var_os("STATE_DIRECTORY"),
            std::env::var_os("NUWAX_RUNTIME_DATA_DIR"),
            std::env::current_dir,
        )
    }

    fn fsync_directory(path: &Path) -> Result<(), String> {
        File::open(path)
            .and_then(|directory| directory.sync_all())
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_FSYNC_FAILED:{error}"))
    }

    fn lock_terminal_cleanup(root: &Path) -> Result<Flock<File>, String> {
        let lock_path = root.join(RELOCALIZATION_TERMINAL_CLEANUP_LOCK_NAME);
        let lock_file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&lock_path)
            .map_err(|error| {
                format!("RELOCALIZATION_TERMINAL_STORE_CLEANUP_LOCK_OPEN_FAILED:{error}")
            })?;
        let lock_file_metadata = lock_file.metadata().map_err(|error| {
            format!("RELOCALIZATION_TERMINAL_STORE_CLEANUP_LOCK_OPEN_FAILED:{error}")
        })?;
        let lock = Flock::lock(lock_file, FlockArg::LockExclusive).map_err(|(_, error)| {
            format!("RELOCALIZATION_TERMINAL_STORE_CLEANUP_LOCK_FAILED:{error}")
        })?;
        let path_metadata = fs::metadata(&lock_path).map_err(|error| {
            format!("RELOCALIZATION_TERMINAL_STORE_CLEANUP_LOCK_OPEN_FAILED:{error}")
        })?;
        if lock_file_metadata.dev() != path_metadata.dev()
            || lock_file_metadata.ino() != path_metadata.ino()
        {
            return Err("RELOCALIZATION_TERMINAL_STORE_CLEANUP_LOCK_IDENTITY_CHANGED".to_owned());
        }
        lock.sync_all().map_err(|error| {
            format!("RELOCALIZATION_TERMINAL_STORE_CLEANUP_LOCK_FSYNC_FAILED:{error}")
        })?;
        fsync_directory(root)?;
        Ok(lock)
    }

    fn terminal_owner_token() -> String {
        uuid::Uuid::new_v4().simple().to_string()
    }

    fn terminal_lease_name(owner_token: &str) -> String {
        format!(
            "{RELOCALIZATION_TERMINAL_LEASE_PREFIX}{owner_token}{RELOCALIZATION_TERMINAL_LEASE_SUFFIX}"
        )
    }

    fn terminal_owner_token_from_lease_name(file_name: &str) -> Option<&str> {
        let token = file_name
            .strip_prefix(RELOCALIZATION_TERMINAL_LEASE_PREFIX)?
            .strip_suffix(RELOCALIZATION_TERMINAL_LEASE_SUFFIX)?;
        is_terminal_token(token, RELOCALIZATION_TERMINAL_OWNER_TOKEN_BYTES).then_some(token)
    }

    fn is_terminal_token(value: &str, bytes: usize) -> bool {
        value.len() == bytes * 2 && decode_hex(value, bytes).is_ok()
    }

    fn terminal_owner_token_from_temporary_name(file_name: &str) -> Option<&str> {
        let body = file_name
            .strip_prefix(RELOCALIZATION_TERMINAL_TEMP_PREFIX)?
            .strip_suffix(RELOCALIZATION_TERMINAL_TEMP_SUFFIX)?;
        let mut fields = body.split('.');
        let (Some(owner), Some(attempt), Some(request), Some(record), None) = (
            fields.next(),
            fields.next(),
            fields.next(),
            fields.next(),
            fields.next(),
        ) else {
            return None;
        };
        (is_terminal_token(owner, RELOCALIZATION_TERMINAL_OWNER_TOKEN_BYTES)
            && is_terminal_token(attempt, RELOCALIZATION_TERMINAL_ATTEMPT_TOKEN_BYTES)
            && is_terminal_token(request, 32)
            && is_terminal_token(record, 32))
        .then_some(owner)
    }

    fn is_owned_terminal_temporary_name(file_name: &str) -> bool {
        terminal_owner_token_from_temporary_name(file_name).is_some()
    }

    fn terminal_owner_has_temporary(root: &Path, owner_token: &str) -> Result<bool, String> {
        let entries = fs::read_dir(root)
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?;
        for entry in entries {
            let entry = entry
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?;
            let file_name = entry.file_name();
            if file_name
                .to_str()
                .and_then(terminal_owner_token_from_temporary_name)
                == Some(owner_token)
            {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn cleanup_terminal_owner(
        root: &Path,
        owner_token: &str,
        _cleanup_lock: &Flock<File>,
    ) -> Result<usize, String> {
        let lease_path = root.join(terminal_lease_name(owner_token));
        let lease_file = match OpenOptions::new().read(true).write(true).open(&lease_path) {
            Ok(file) => file,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                return Err("RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_MISSING".to_owned())
            }
            Err(error) => {
                return Err(format!(
                    "RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_OPEN_FAILED:{error}"
                ))
            }
        };
        let lease = match Flock::lock(lease_file, FlockArg::LockExclusiveNonblock) {
            Ok(lease) => lease,
            Err((lease_file, error))
                if error == nix::errno::Errno::EWOULDBLOCK
                    || error == nix::errno::Errno::EAGAIN =>
            {
                drop(lease_file);
                return Ok(0);
            }
            Err((lease_file, error)) => {
                drop(lease_file);
                return Err(format!(
                    "RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_LOCK_FAILED:{error}"
                ));
            }
        };
        let entries = fs::read_dir(root)
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?;
        let mut removed = 0_usize;
        for entry in entries {
            let file_name = entry.file_name();
            if file_name
                .to_str()
                .and_then(terminal_owner_token_from_temporary_name)
                == Some(owner_token)
            {
                remove_terminal_temporary(&entry.path(), root)?;
                removed += 1;
            }
        }
        if !terminal_owner_has_temporary(root, owner_token)? {
            match fs::remove_file(&lease_path) {
                Ok(()) => fsync_directory(root)?,
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(_) => return Ok(removed),
            }
        }
        drop(lease);
        Ok(removed)
    }

    fn cleanup_failed_terminal_owner_lease(
        root: &Path,
        lease_path: &Path,
        primary: String,
        _cleanup_lock: &Flock<File>,
    ) -> String {
        match fs::remove_file(lease_path) {
            Ok(()) => match fsync_directory(root) {
                Ok(()) => primary,
                Err(_) => format!("{primary}|OWNER_LEASE_CLEANUP_FAILED"),
            },
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => primary,
            Err(_) => format!("{primary}|OWNER_LEASE_CLEANUP_FAILED"),
        }
    }

    fn create_terminal_owner_lease(
        root: &Path,
        cleanup_lock: &Flock<File>,
    ) -> Result<(String, Flock<File>), String> {
        for _ in 0..16 {
            let owner_token = terminal_owner_token();
            let lease_path = root.join(terminal_lease_name(&owner_token));
            let lease_file = match OpenOptions::new()
                .read(true)
                .write(true)
                .create_new(true)
                .open(&lease_path)
            {
                Ok(file) => file,
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => {
                    return Err(format!(
                        "RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_CREATE_FAILED:{error}"
                    ))
                }
            };
            let lease = match Flock::lock(lease_file, FlockArg::LockExclusiveNonblock) {
                Ok(lease) => lease,
                Err((lease_file, error)) => {
                    drop(lease_file);
                    let primary =
                        format!("RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_LOCK_FAILED:{error}");
                    return Err(cleanup_failed_terminal_owner_lease(
                        root,
                        &lease_path,
                        primary,
                        cleanup_lock,
                    ));
                }
            };
            if let Err(error) = lease.sync_all() {
                let primary =
                    format!("RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_FSYNC_FAILED:{error}");
                drop(lease);
                return Err(cleanup_failed_terminal_owner_lease(
                    root,
                    &lease_path,
                    primary,
                    cleanup_lock,
                ));
            }
            if let Err(primary) = fsync_directory(root) {
                drop(lease);
                return Err(cleanup_failed_terminal_owner_lease(
                    root,
                    &lease_path,
                    primary,
                    cleanup_lock,
                ));
            }
            return Ok((owner_token, lease));
        }
        Err("RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_COLLISION".to_owned())
    }

    fn remove_terminal_temporary(path: &Path, root: &Path) -> Result<(), String> {
        match fs::remove_file(path) {
            Ok(()) => fsync_directory(root),
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
            Err(error) => Err(format!(
                "RELOCALIZATION_TERMINAL_STORE_TEMP_CLEANUP_FAILED:{error}"
            )),
        }
    }

    fn combine_terminal_persist_results(
        write_result: Result<(), String>,
        cleanup_result: Result<(), String>,
    ) -> Result<(), String> {
        match (write_result, cleanup_result) {
            (Err(primary), Err(_)) => Err(format!("{primary}|TEMP_CLEANUP_FAILED")),
            (Err(primary), Ok(())) => Err(primary),
            (Ok(()), Err(cleanup)) => {
                tracing::warn!(error = %cleanup, "durable terminal temp cleanup deferred");
                Ok(())
            }
            (Ok(()), Ok(())) => Ok(()),
        }
    }

    impl RelocalizationTerminalStore {
        #[cfg(not(test))]
        fn open() -> Result<Self, String> {
            Self::open_at(terminal_store_root()?)
        }

        #[cfg(test)]
        fn open() -> Result<Self, String> {
            let root = Arc::new(tests::RelocalizationTerminalTestRoot::new());
            let mut store = Self::open_at(root.path().to_path_buf())?;
            store._test_root = Some(root);
            Ok(store)
        }

        fn open_at(root: PathBuf) -> Result<Self, String> {
            fs::create_dir_all(&root)
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_OPEN_FAILED:{error}"))?;
            fsync_directory(&root)?;
            {
                let cleanup_lock = lock_terminal_cleanup(&root)?;
                let mut entries = fs::read_dir(&root)
                    .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|error| {
                        format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}")
                    })?;
                entries.sort_by_key(std::fs::DirEntry::file_name);
                let mut owner_tokens = HashSet::new();
                for entry in &entries {
                    let file_name = entry.file_name();
                    if let Some(owner_token) = file_name
                        .to_str()
                        .and_then(terminal_owner_token_from_temporary_name)
                        .or_else(|| {
                            file_name
                                .to_str()
                                .and_then(terminal_owner_token_from_lease_name)
                        })
                    {
                        owner_tokens.insert(owner_token.to_owned());
                    }
                }
                for owner_token in owner_tokens {
                    cleanup_terminal_owner(&root, &owner_token, &cleanup_lock)?;
                }
                let retained_temporary_count = fs::read_dir(&root)
                    .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?
                    .filter_map(Result::ok)
                    .filter(|entry| {
                        entry
                            .file_name()
                            .to_str()
                            .map(is_owned_terminal_temporary_name)
                            .unwrap_or(false)
                    })
                    .count();
                if retained_temporary_count > RELOCALIZATION_MAX_COMMAND_RECORDS {
                    return Err("RELOCALIZATION_TERMINAL_STORE_TEMP_RETENTION_EXCEEDED".to_owned());
                }
            }
            let mut terminals = HashMap::new();
            let entries = fs::read_dir(&root)
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?
                .collect::<Result<Vec<_>, _>>()
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_READ_FAILED:{error}"))?;
            for entry in entries {
                let path = entry.path();
                if path.extension().and_then(|value| value.to_str()) != Some("json") {
                    continue;
                }
                if terminals.len() >= RELOCALIZATION_MAX_COMMAND_RECORDS {
                    return Err("RELOCALIZATION_TERMINAL_STORE_RETENTION_EXCEEDED".to_owned());
                }
                let bytes = read_terminal_record(&path)?;
                let record: DurableRelocalizationTerminalV1 = serde_json::from_slice(&bytes)
                    .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID".to_owned())?;
                if record.schema_version != 1
                    || validate_relocalization_request_id(&record.request_id).is_err()
                {
                    return Err("RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID".to_owned());
                }
                let request_identity_sha256: [u8; 32] =
                    decode_hex(&record.request_identity_sha256, 32)?
                        .try_into()
                        .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID".to_owned())?;
                let activation_identity_sha256: [u8; 32] =
                    decode_hex(&record.activation_identity_sha256, 32)?
                        .try_into()
                        .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID".to_owned())?;
                let encoded = decode_hex(&record.terminal_hex, RELOCALIZATION_MAX_TERMINAL_BYTES)?;
                if encode_hex(&sha256_bytes(&encoded)) != record.terminal_sha256 {
                    return Err("RELOCALIZATION_TERMINAL_STORE_DIGEST_MISMATCH".to_owned());
                }
                let result = crate::proto::RelocalizeResult::decode(encoded.as_slice())
                    .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_TERMINAL_INVALID".to_owned())?;
                if result.request_id != record.request_id
                    || result.request_identity_sha256.as_slice() != request_identity_sha256
                    || result.activation_identity_sha256.as_slice() != activation_identity_sha256
                {
                    return Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_MISMATCH".to_owned());
                }
                let terminal = StoredRelocalizationResult::new(result);
                if terminal.encoded != encoded {
                    return Err("RELOCALIZATION_TERMINAL_STORE_NON_CANONICAL".to_owned());
                }
                if terminals
                    .insert(
                        record.request_id,
                        DurableRelocalizationTerminal {
                            request_identity_sha256,
                            activation_identity_sha256,
                            terminal,
                        },
                    )
                    .is_some()
                {
                    return Err("RELOCALIZATION_TERMINAL_STORE_DUPLICATE".to_owned());
                }
            }
            let cleanup_lock = lock_terminal_cleanup(&root)?;
            let (owner_token, owner_lock) = create_terminal_owner_lease(&root, &cleanup_lock)?;
            drop(cleanup_lock);
            let owner_lease = RelocalizationTerminalOwnerLease {
                root: root.clone(),
                owner_token: owner_token.clone(),
                _lock: owner_lock,
            };
            Ok(Self {
                root: Arc::new(root),
                owner_token: Arc::new(owner_token),
                _owner_lease: Arc::new(owner_lease),
                state: Arc::new(parking_lot::Mutex::new(RelocalizationTerminalStoreState {
                    terminals,
                    pending: HashMap::new(),
                })),
                #[cfg(test)]
                _test_root: None,
            })
        }

        fn mark_pending(&self, request: &ValidatedRelocalizeRequest) -> Result<(), String> {
            let mut state = self.state.lock();
            if let Some(existing) = state.terminals.get(&request.request.request_id) {
                return if existing.request_identity_sha256 == request.identity_sha256
                    && existing.activation_identity_sha256 == request.activation_identity_sha256
                {
                    Ok(())
                } else {
                    Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".to_owned())
                };
            }
            match state.pending.get(&request.request.request_id) {
                Some(identity)
                    if *identity
                        != (request.identity_sha256, request.activation_identity_sha256) =>
                {
                    Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".to_owned())
                }
                _ => {
                    state.pending.insert(
                        request.request.request_id.clone(),
                        (request.identity_sha256, request.activation_identity_sha256),
                    );
                    Ok(())
                }
            }
        }

        fn persist_terminal(
            &self,
            request_id: &str,
            request_identity_sha256: [u8; 32],
            activation_identity_sha256: [u8; 32],
            terminal: &StoredRelocalizationResult,
        ) -> Result<(), String> {
            if terminal.encoded.len() > RELOCALIZATION_MAX_TERMINAL_BYTES {
                return Err("RELOCALIZATION_TERMINAL_STORE_TERMINAL_TOO_LARGE".to_owned());
            }
            let mut state = self.state.lock();
            if let Some(existing) = state.terminals.get(request_id) {
                return if existing.request_identity_sha256 == request_identity_sha256
                    && existing.activation_identity_sha256 == activation_identity_sha256
                    && existing.terminal.encoded == terminal.encoded
                {
                    Ok(())
                } else {
                    Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".to_owned())
                };
            }
            if state.terminals.len() >= RELOCALIZATION_MAX_COMMAND_RECORDS {
                return Err("RELOCALIZATION_TERMINAL_STORE_RETENTION_EXCEEDED".to_owned());
            }
            let record = DurableRelocalizationTerminalV1 {
                schema_version: 1,
                request_id: request_id.to_owned(),
                request_identity_sha256: encode_hex(&request_identity_sha256),
                activation_identity_sha256: encode_hex(&activation_identity_sha256),
                terminal_hex: encode_hex(&terminal.encoded),
                terminal_sha256: encode_hex(&terminal.encoded_sha256),
            };
            let bytes = serde_json::to_vec(&record)
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_ENCODE_FAILED:{error}"))?;
            let request_digest = encode_hex(&sha256_bytes(request_id.as_bytes()));
            let file_name = format!("{request_digest}.json");
            let final_path = self.root.join(file_name);
            let attempt = terminal_owner_token();
            let temporary_path = self.root.join(format!(
                "{RELOCALIZATION_TERMINAL_TEMP_PREFIX}{}.{}.{}.{}{RELOCALIZATION_TERMINAL_TEMP_SUFFIX}",
                self.owner_token,
                attempt,
                request_digest,
                encode_hex(&sha256_bytes(&bytes))
            ));
            let mut temporary = OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&temporary_path)
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_CREATE_FAILED:{error}"))?;
            let write_result = (|| {
                temporary.write_all(&bytes).map_err(|error| {
                    format!("RELOCALIZATION_TERMINAL_STORE_WRITE_FAILED:{error}")
                })?;
                temporary.sync_all().map_err(|error| {
                    format!("RELOCALIZATION_TERMINAL_STORE_FSYNC_FAILED:{error}")
                })?;
                drop(temporary);
                match fs::hard_link(&temporary_path, &final_path) {
                    Ok(()) => {}
                    Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
                        let existing = read_terminal_record(&final_path)?;
                        if existing != bytes {
                            return Err(
                                "RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".to_owned()
                            );
                        }
                    }
                    Err(error) => {
                        return Err(format!(
                            "RELOCALIZATION_TERMINAL_STORE_PUBLISH_FAILED:{error}"
                        ))
                    }
                }
                fsync_directory(&self.root)
            })();
            let cleanup_result = remove_terminal_temporary(&temporary_path, &self.root);
            combine_terminal_persist_results(write_result, cleanup_result)?;
            state.terminals.insert(
                request_id.to_owned(),
                DurableRelocalizationTerminal {
                    request_identity_sha256,
                    activation_identity_sha256,
                    terminal: terminal.clone(),
                },
            );
            state.pending.remove(request_id);
            Ok(())
        }

        pub(crate) fn snapshot(
            &self,
            request_id: &str,
            request_identity_sha256: [u8; 32],
            activation_identity_sha256: [u8; 32],
        ) -> RelocalizationTerminalSnapshot {
            let state = self.state.lock();
            if let Some(terminal) = state.terminals.get(request_id) {
                return if terminal.request_identity_sha256 == request_identity_sha256
                    && terminal.activation_identity_sha256 == activation_identity_sha256
                {
                    RelocalizationTerminalSnapshot::Terminal(terminal.terminal.result.clone())
                } else {
                    RelocalizationTerminalSnapshot::IdentityConflict
                };
            }
            match state.pending.get(request_id) {
                Some(identity)
                    if *identity == (request_identity_sha256, activation_identity_sha256) =>
                {
                    RelocalizationTerminalSnapshot::Pending
                }
                Some(_) => RelocalizationTerminalSnapshot::IdentityConflict,
                None => RelocalizationTerminalSnapshot::NotFound,
            }
        }
    }

    struct RelocalizationLedgerRecord {
        request: crate::proto::RelocalizeRequest,
        identity_sha256: [u8; 32],
        admission_ordinal: u64,
        logical_deadline: Instant,
        claim_phase: RelocalizationClaimPhase,
        waiter_ids: HashSet<u64>,
        terminal: Option<StoredRelocalizationResult>,
        conflicts: HashMap<[u8; 32], StoredRelocalizationResult>,
        conflict_overflow: Option<([u8; 32], StoredRelocalizationResult)>,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum RelocalizationClaimPhase {
        Preclaim,
        Postclaim,
    }

    #[derive(Clone)]
    enum RelocalizationLedgerAdmission {
        Started {
            admission_ordinal: u64,
            logical_deadline: Instant,
        },
        Joined {
            admission_ordinal: u64,
            logical_deadline: Instant,
        },
        Replay(StoredRelocalizationResult),
        Conflict(StoredRelocalizationResult),
        ConflictOverflow(StoredRelocalizationResult),
        Busy,
        CapacityExceeded(&'static str),
    }

    struct RelocalizationTerminalWaiters {
        waiter_ids: Vec<u64>,
        result: StoredRelocalizationResult,
    }

    struct RelocalizationCommandLedger {
        records: HashMap<String, RelocalizationLedgerRecord>,
        active_request_id: Option<String>,
        next_admission_ordinal: u64,
        admitted_command_count: u64,
        claimed_command_count: u64,
        observation_epoch: u64,
        relocalization_mode: Arc<RelocalizationModeFence>,
    }

    impl Default for RelocalizationCommandLedger {
        fn default() -> Self {
            let mut epoch_bytes = [0_u8; 8];
            epoch_bytes.copy_from_slice(&uuid::Uuid::new_v4().as_bytes()[..8]);
            Self {
                records: HashMap::new(),
                active_request_id: None,
                next_admission_ordinal: 0,
                admitted_command_count: 0,
                claimed_command_count: 0,
                observation_epoch: u64::from_be_bytes(epoch_bytes).max(1),
                relocalization_mode: Arc::new(RelocalizationModeFence::default()),
            }
        }
    }

    impl RelocalizationCommandLedger {
        fn admit<F>(
            &mut self,
            request: &ValidatedRelocalizeRequest,
            waiter_id: u64,
            now: Instant,
            conflict_result: F,
        ) -> Result<RelocalizationLedgerAdmission, &'static str>
        where
            F: FnOnce() -> StoredRelocalizationResult,
        {
            if waiter_id == 0 {
                return Err("RELOCALIZATION_WAITER_ID_INVALID");
            }
            if let Some(record) = self.records.get(&request.request.request_id) {
                if record.identity_sha256 == request.identity_sha256 {
                    if let Some(terminal) = &record.terminal {
                        return Ok(RelocalizationLedgerAdmission::Replay(terminal.clone()));
                    }
                    let admission_ordinal = record.admission_ordinal;
                    let logical_deadline = record.logical_deadline;
                    let record = self
                        .records
                        .get_mut(&request.request.request_id)
                        .expect("relocalization record remains present");
                    record.waiter_ids.insert(waiter_id);
                    return Ok(RelocalizationLedgerAdmission::Joined {
                        admission_ordinal,
                        logical_deadline,
                    });
                }
                if let Some(conflict) = record.conflicts.get(&request.identity_sha256) {
                    return Ok(RelocalizationLedgerAdmission::Conflict(conflict.clone()));
                }
                if let Some((overflow_identity, conflict)) = &record.conflict_overflow {
                    if *overflow_identity == request.identity_sha256 {
                        return Ok(RelocalizationLedgerAdmission::Conflict(conflict.clone()));
                    }
                    return Ok(RelocalizationLedgerAdmission::CapacityExceeded(
                        "RELOCALIZATION_COMMAND_CONFLICT_OVERFLOW_CLOSED",
                    ));
                }
                if record.conflicts.len() >= RELOCALIZATION_MAX_CONFLICTS_PER_RECORD {
                    let conflict = conflict_result();
                    self.records
                        .get_mut(&request.request.request_id)
                        .expect("relocalization record remains present")
                        .conflict_overflow = Some((request.identity_sha256, conflict.clone()));
                    return Ok(RelocalizationLedgerAdmission::ConflictOverflow(conflict));
                }
                let conflict = conflict_result();
                self.records
                    .get_mut(&request.request.request_id)
                    .expect("relocalization record remains present")
                    .conflicts
                    .insert(request.identity_sha256, conflict.clone());
                return Ok(RelocalizationLedgerAdmission::Conflict(conflict));
            }

            if self.active_request_id.is_some() {
                return Ok(RelocalizationLedgerAdmission::Busy);
            }
            // Keep one bounded record slot reserved for the typed capacity
            // terminal. That terminal closes this runtime generation after
            // it is written, so retries remain replayable without eviction.
            if self.records.len() >= RELOCALIZATION_MAX_COMMAND_RECORDS.saturating_sub(1) {
                return Ok(RelocalizationLedgerAdmission::CapacityExceeded(
                    "RELOCALIZATION_COMMAND_LEDGER_CAPACITY_EXCEEDED",
                ));
            }
            let admission_ordinal = self
                .next_admission_ordinal
                .checked_add(1)
                .ok_or("RELOCALIZATION_ADMISSION_ORDINAL_EXHAUSTED")?;
            let admitted_command_count = self
                .admitted_command_count
                .checked_add(1)
                .ok_or("RELOCALIZATION_ADMITTED_COMMAND_COUNT_EXHAUSTED")?;
            let logical_deadline = now
                .checked_add(Duration::from_millis(request.request.logical_timeout_ms))
                .ok_or("RELOCALIZATION_LOGICAL_DEADLINE_OVERFLOW")?;
            self.relocalization_mode.activate()?;
            self.next_admission_ordinal = admission_ordinal;
            self.admitted_command_count = admitted_command_count;
            self.active_request_id = Some(request.request.request_id.clone());
            self.records.insert(
                request.request.request_id.clone(),
                RelocalizationLedgerRecord {
                    request: request.request.clone(),
                    identity_sha256: request.identity_sha256,
                    admission_ordinal,
                    logical_deadline,
                    claim_phase: RelocalizationClaimPhase::Preclaim,
                    waiter_ids: HashSet::from([waiter_id]),
                    terminal: None,
                    conflicts: HashMap::new(),
                    conflict_overflow: None,
                },
            );
            Ok(RelocalizationLedgerAdmission::Started {
                admission_ordinal,
                logical_deadline,
            })
        }

        fn store_capacity_terminal(
            &mut self,
            request: &ValidatedRelocalizeRequest,
            now: Instant,
            terminal: StoredRelocalizationResult,
        ) -> Result<(), &'static str> {
            if self.records.contains_key(&request.request.request_id) {
                return Err("RELOCALIZATION_COMMAND_CAPACITY_RECORD_ALREADY_EXISTS");
            }
            if self.active_request_id.is_some() {
                return Err("RELOCALIZATION_COMMAND_CAPACITY_WHILE_ACTIVE");
            }
            if self.records.len() >= RELOCALIZATION_MAX_COMMAND_RECORDS {
                return Err("RELOCALIZATION_COMMAND_LEDGER_CAPACITY_EXCEEDED");
            }
            let admitted_command_count = self
                .admitted_command_count
                .checked_add(1)
                .ok_or("RELOCALIZATION_ADMITTED_COMMAND_COUNT_EXHAUSTED")?;
            self.records.insert(
                request.request.request_id.clone(),
                RelocalizationLedgerRecord {
                    request: request.request.clone(),
                    identity_sha256: request.identity_sha256,
                    admission_ordinal: 0,
                    logical_deadline: now,
                    claim_phase: RelocalizationClaimPhase::Preclaim,
                    waiter_ids: HashSet::new(),
                    terminal: Some(terminal),
                    conflicts: HashMap::new(),
                    conflict_overflow: None,
                },
            );
            self.admitted_command_count = admitted_command_count;
            Ok(())
        }

        fn mark_postclaim(
            &mut self,
            request_id: &str,
            identity_sha256: [u8; 32],
        ) -> Result<(), &'static str> {
            let record = self
                .records
                .get_mut(request_id)
                .ok_or("RELOCALIZATION_LEDGER_RECORD_MISSING")?;
            if record.identity_sha256 != identity_sha256 || record.terminal.is_some() {
                return Err("RELOCALIZATION_LEDGER_IDENTITY_MISMATCH");
            }
            if record.claim_phase == RelocalizationClaimPhase::Preclaim {
                self.claimed_command_count = self
                    .claimed_command_count
                    .checked_add(1)
                    .ok_or("RELOCALIZATION_CLAIMED_COMMAND_COUNT_EXHAUSTED")?;
                record.claim_phase = RelocalizationClaimPhase::Postclaim;
            }
            Ok(())
        }

        fn detach_waiter(
            &mut self,
            request_id: &str,
            identity_sha256: [u8; 32],
            waiter_id: u64,
        ) -> Result<bool, &'static str> {
            let record = self
                .records
                .get_mut(request_id)
                .ok_or("RELOCALIZATION_LEDGER_RECORD_MISSING")?;
            if record.identity_sha256 != identity_sha256 || record.terminal.is_some() {
                return Err("RELOCALIZATION_LEDGER_IDENTITY_MISMATCH");
            }
            record.waiter_ids.remove(&waiter_id);
            Ok(record.waiter_ids.is_empty()
                && record.claim_phase == RelocalizationClaimPhase::Preclaim)
        }

        fn settle_terminal(
            &mut self,
            request_id: &str,
            identity_sha256: [u8; 32],
            terminal: StoredRelocalizationResult,
        ) -> Result<RelocalizationTerminalWaiters, &'static str> {
            let record = self
                .records
                .get_mut(request_id)
                .ok_or("RELOCALIZATION_LEDGER_RECORD_MISSING")?;
            if record.identity_sha256 != identity_sha256 {
                return Err("RELOCALIZATION_LEDGER_IDENTITY_MISMATCH");
            }
            if let Some(stored) = &record.terminal {
                if stored.encoded == terminal.encoded {
                    return Ok(RelocalizationTerminalWaiters {
                        waiter_ids: Vec::new(),
                        result: stored.clone(),
                    });
                }
                return Err("RELOCALIZATION_TERMINAL_CONFLICT");
            }
            record.terminal = Some(terminal.clone());
            let mut waiter_ids = record.waiter_ids.drain().collect::<Vec<_>>();
            waiter_ids.sort_unstable();
            if self.active_request_id.as_deref() == Some(request_id) {
                self.active_request_id = None;
                self.relocalization_mode.deactivate();
            }
            Ok(RelocalizationTerminalWaiters {
                waiter_ids,
                result: terminal,
            })
        }

        fn observation(
            &self,
        ) -> Result<crate::proto::RelocalizationLedgerObservation, &'static str> {
            let terminal_record_count =
                self.records.values().try_fold(0_u64, |count, record| {
                    if record.terminal.is_some() {
                        count
                            .checked_add(1)
                            .ok_or("RELOCALIZATION_TERMINAL_RECORD_COUNT_EXHAUSTED")
                    } else {
                        Ok(count)
                    }
                })?;
            let pending_record_count = self.records.values().try_fold(0_u64, |count, record| {
                if record.terminal.is_none() {
                    count
                        .checked_add(1)
                        .ok_or("RELOCALIZATION_PENDING_RECORD_COUNT_EXHAUSTED")
                } else {
                    Ok(count)
                }
            })?;
            let active_waiter_count = self.records.values().try_fold(0_u64, |count, record| {
                let record_waiters = u64::try_from(record.waiter_ids.len())
                    .map_err(|_| "RELOCALIZATION_ACTIVE_WAITER_COUNT_EXHAUSTED")?;
                count
                    .checked_add(record_waiters)
                    .ok_or("RELOCALIZATION_ACTIVE_WAITER_COUNT_EXHAUSTED")
            })?;
            let record_count = u64::try_from(self.records.len())
                .map_err(|_| "RELOCALIZATION_RECORD_COUNT_EXHAUSTED")?;
            if terminal_record_count
                .checked_add(pending_record_count)
                .ok_or("RELOCALIZATION_RECORD_COUNT_EXHAUSTED")?
                != record_count
                || self.observation_epoch == 0
            {
                return Err("RELOCALIZATION_LEDGER_OBSERVATION_INCONSISTENT");
            }
            Ok(crate::proto::RelocalizationLedgerObservation {
                observation_schema_version: 1,
                admitted_command_count: self.admitted_command_count,
                claimed_command_count: self.claimed_command_count,
                terminal_record_count,
                pending_record_count,
                active_waiter_count,
                observation_epoch: self.observation_epoch,
            })
        }

        fn terminal(&self, request_id: &str) -> Option<&StoredRelocalizationResult> {
            self.records
                .get(request_id)
                .and_then(|record| record.terminal.as_ref())
        }

        fn request(&self, request_id: &str) -> Option<&crate::proto::RelocalizeRequest> {
            self.records.get(request_id).map(|record| &record.request)
        }

        fn active_identity(
            &self,
        ) -> Option<(String, [u8; 32], u64, Instant, RelocalizationClaimPhase)> {
            let request_id = self.active_request_id.as_ref()?;
            let record = self.records.get(request_id)?;
            Some((
                request_id.clone(),
                record.identity_sha256,
                record.admission_ordinal,
                record.logical_deadline,
                record.claim_phase,
            ))
        }
    }

    impl Drop for RelocalizationCommandLedger {
        fn drop(&mut self) {
            self.relocalization_mode.deactivate();
        }
    }

    struct RelocalizationRuntimeContext {
        frames: mpsc::Receiver<CorrectionFrameRequest>,
        phase_a: CorrectionPhaseAControl,
        finalizer: CanonicalPublishFinalizerIngress,
        apply_waiters: Arc<CandidateApplyWaiters>,
        sink: Arc<dyn CorrectionCandidateSink>,
        cleanup: CorrectionCleanupIngress,
        #[cfg(test)]
        compute_delay_ms: Arc<AtomicU64>,
        #[cfg(test)]
        compute_panic: Arc<AtomicBool>,
    }

    struct RelocalizationOwnerConfig {
        phase_a: CorrectionPhaseAControl,
        owner_identity: CorrectionOwnerRuntimeIdentity,
        map_frame_id: String,
        odom_frame_id: String,
        body_frame_id: String,
        runtime: Option<RelocalizationRuntimeContext>,
        terminal_store: RelocalizationTerminalStore,
    }

    struct RelocalizationAccumulation {
        request: ValidatedRelocalizeRequest,
        admission_ordinal: u64,
        mode_epoch: u64,
        logical_deadline: Instant,
        accumulation_deadline: Instant,
        snapshot: SealedActivatedTargetSnapshot,
        arm_watermark: Option<RelocalizationFrameKey>,
        armed_map_watermarks: HashMap<u64, RelocalizationFrameKey>,
        accepted: Vec<AccumulatedRelocalizationFrame>,
        aggregate_input_points: usize,
        session_map_frame_generation: Option<u64>,
        mapping_commit_boot_nonce: Option<u128>,
    }

    struct RelocalizationComputedCommand {
        request: ValidatedRelocalizeRequest,
        admission_ordinal: u64,
        logical_deadline: Instant,
        source: SealedRelocalizationSource,
        output: RelocalizationComputeOutput,
    }

    enum RelocalizationExecution {
        Accumulating(Box<RelocalizationAccumulation>),
        Computing {
            request: Box<ValidatedRelocalizeRequest>,
            admission_ordinal: u64,
            logical_deadline: Instant,
        },
        Reserving {
            computed: Option<Box<RelocalizationComputedCommand>>,
        },
        Staging {
            computed: Option<Box<RelocalizationComputedCommand>>,
            permit: Option<mpsc::OwnedPermit<CanonicalFinalizerJob>>,
        },
        Finalizing {
            computed: Option<Box<RelocalizationComputedCommand>>,
            candidate: Box<CorrectionCandidate>,
        },
    }

    enum RelocalizationTaskEvent {
        Computed {
            request_id: String,
            identity_sha256: [u8; 32],
            source: Box<SealedRelocalizationSource>,
            output: RelocalizationComputeOutput,
        },
        PermitReserved(Result<mpsc::OwnedPermit<CanonicalFinalizerJob>, &'static str>),
        Staged(Result<(), String>),
        Finalized {
            apply: Result<CandidateApplyResult, oneshot::error::RecvError>,
            outcome: Result<Result<(), String>, oneshot::error::RecvError>,
            correction_revision: u64,
        },
        DiscardScheduled(Result<(), CorrectionRuntimeError>),
    }

    struct RelocalizationOwnerSupervisor {
        ingress: RelocalizationCommandIngress,
        worker: Option<tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
    }

    impl RelocalizationOwnerSupervisor {
        fn start(
            phase_a: CorrectionPhaseAControl,
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
        ) -> Self {
            Self::start_inner(
                phase_a,
                owner_identity,
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                None,
            )
        }

        #[allow(clippy::too_many_arguments)]
        fn start_runtime(
            frames: mpsc::Receiver<CorrectionFrameRequest>,
            phase_a: CorrectionPhaseAControl,
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
            finalizer: CanonicalPublishFinalizerIngress,
            apply_waiters: Arc<CandidateApplyWaiters>,
            sink: Arc<dyn CorrectionCandidateSink>,
            cleanup: CorrectionCleanupIngress,
            #[cfg(test)] compute_delay_ms: Arc<AtomicU64>,
            #[cfg(test)] compute_panic: Arc<AtomicBool>,
        ) -> Self {
            let runtime = RelocalizationRuntimeContext {
                frames,
                phase_a: phase_a.clone(),
                finalizer,
                apply_waiters,
                sink,
                cleanup,
                #[cfg(test)]
                compute_delay_ms,
                #[cfg(test)]
                compute_panic,
            };
            Self::start_inner(
                phase_a,
                owner_identity,
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                Some(runtime),
            )
        }

        fn start_inner(
            phase_a: CorrectionPhaseAControl,
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
            runtime: Option<RelocalizationRuntimeContext>,
        ) -> Self {
            let (sender, receiver) = mpsc::channel(RELOCALIZATION_COMMAND_QUEUE_CAPACITY);
            let (terminal_query_sender, terminal_query_receiver) =
                mpsc::channel(RELOCALIZATION_COMMAND_QUEUE_CAPACITY);
            let (detached_tx, detached_rx) = mpsc::unbounded_channel();
            let terminal_store = RelocalizationTerminalStore::open()
                .expect("durable relocalization terminal store must open before admission");
            let ingress = RelocalizationCommandIngress {
                sender,
                terminal_query_sender,
                terminal_store: terminal_store.clone(),
                admission_open: Arc::clone(&phase_a.admission_open),
                admission_rx: phase_a.admission_tx.subscribe(),
                waiter_capacity: Arc::new(Semaphore::new(RELOCALIZATION_MAX_WAITERS)),
            };
            let worker = tokio::spawn(run_relocalization_command_owner(
                receiver,
                terminal_query_receiver,
                detached_rx,
                detached_tx,
                RelocalizationOwnerConfig {
                    phase_a,
                    owner_identity,
                    map_frame_id,
                    odom_frame_id,
                    body_frame_id,
                    runtime,
                    terminal_store,
                },
            ));
            Self {
                ingress,
                worker: Some(worker),
            }
        }

        fn ingress(&self) -> RelocalizationCommandIngress {
            self.ingress.clone()
        }

        fn is_finished(&self) -> bool {
            self.worker
                .as_ref()
                .is_none_or(tokio::task::JoinHandle::is_finished)
        }

        async fn shutdown(&mut self) -> Result<(), CorrectionRuntimeError> {
            let Some(worker) = self.worker.as_mut() else {
                return Ok(());
            };
            let deadline = Instant::now() + CORRECTION_WORKER_STOP_TIMEOUT;
            match settle_retained_correction_worker(
                "relocalization command owner",
                worker,
                deadline,
            )
            .await
            {
                Some(result) => {
                    self.worker.take();
                    result
                }
                None => Err(CorrectionRuntimeError::WorkerStopTimeout(
                    "relocalization command owner",
                )),
            }
        }
    }

    struct RelocalizationCommandActor {
        ledger: RelocalizationCommandLedger,
        waiter_responses: HashMap<u64, oneshot::Sender<crate::proto::RelocalizeResult>>,
        waiter_requests: HashMap<u64, (String, [u8; 32], Arc<RelocalizationWaiterCapacityLease>)>,
        next_waiter_id: u64,
        owner_identity: CorrectionOwnerRuntimeIdentity,
        map_frame_id: String,
        odom_frame_id: String,
        body_frame_id: String,
        execution: Option<RelocalizationExecution>,
        last_validated_frames: HashMap<RelocalizationFrameScope, RelocalizationFrameKey>,
        validated_frame_proofs:
            HashMap<(RelocalizationFrameScope, RelocalizationFrameKey), [u8; 32]>,
        next_candidate_sequence: u64,
        terminal_store: RelocalizationTerminalStore,
    }

    impl RelocalizationCommandActor {
        fn new(
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
        ) -> Self {
            Self::new_with_terminal_store(
                owner_identity,
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                RelocalizationTerminalStore::open()
                    .expect("durable relocalization terminal store must open"),
            )
        }

        fn new_with_terminal_store(
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
            terminal_store: RelocalizationTerminalStore,
        ) -> Self {
            Self {
                ledger: RelocalizationCommandLedger::default(),
                waiter_responses: HashMap::new(),
                waiter_requests: HashMap::new(),
                next_waiter_id: 0,
                owner_identity,
                map_frame_id,
                odom_frame_id,
                body_frame_id,
                execution: None,
                last_validated_frames: HashMap::new(),
                validated_frame_proofs: HashMap::new(),
                next_candidate_sequence: 0,
                terminal_store,
            }
        }

        fn handle_terminal_query(&self, query: RelocalizationTerminalQueryRequest) {
            let terminal = self.terminal_store.snapshot(
                &query.request_id,
                query.request_identity_sha256,
                query.activation_identity_sha256,
            );
            let result = self.ledger.observation().map(|ledger_observation| {
                RelocalizationTerminalQuerySnapshot {
                    terminal,
                    ledger_observation,
                }
            });
            let _ = query.response.send(result);
        }

        fn next_waiter_id(&mut self) -> Result<u64, RelocalizationCommandError> {
            self.next_waiter_id = self
                .next_waiter_id
                .checked_add(1)
                .ok_or(RelocalizationCommandError::OwnerUnavailable)?;
            Ok(self.next_waiter_id)
        }

        fn immediate_waiter(
            waiter_id: u64,
            result: crate::proto::RelocalizeResult,
            capacity_lease: Arc<RelocalizationWaiterCapacityLease>,
            detached_tx: &mpsc::UnboundedSender<RelocalizationWaiterEvent>,
        ) -> RelocalizationCommandWaiter {
            let (response, response_rx) = oneshot::channel();
            let _ = response.send(result);
            RelocalizationCommandWaiter {
                response: Some(response_rx),
                signal: Arc::new(RelocalizationWaiterSignal {
                    detached: AtomicBool::new(false),
                    waiter_id,
                    detached_tx: detached_tx.clone(),
                }),
                _capacity_lease: capacity_lease,
                completed: false,
            }
        }

        fn retained_waiter(
            &mut self,
            waiter_id: u64,
            request: &ValidatedRelocalizeRequest,
            capacity_lease: Arc<RelocalizationWaiterCapacityLease>,
            detached_tx: &mpsc::UnboundedSender<RelocalizationWaiterEvent>,
        ) -> RelocalizationCommandWaiter {
            let (response, response_rx) = oneshot::channel();
            self.waiter_responses.insert(waiter_id, response);
            self.waiter_requests.insert(
                waiter_id,
                (
                    request.request.request_id.clone(),
                    request.identity_sha256,
                    Arc::clone(&capacity_lease),
                ),
            );
            RelocalizationCommandWaiter {
                response: Some(response_rx),
                signal: Arc::new(RelocalizationWaiterSignal {
                    detached: AtomicBool::new(false),
                    waiter_id,
                    detached_tx: detached_tx.clone(),
                }),
                _capacity_lease: capacity_lease,
                completed: false,
            }
        }

        fn terminal_result(
            &self,
            request: &ValidatedRelocalizeRequest,
            admission_ordinal: Option<u64>,
            disposition: crate::proto::RelocalizeDisposition,
            detail: &str,
        ) -> StoredRelocalizationResult {
            let evidence = RelocalizationEvidenceV1 {
                request_identity_sha256: request.identity_sha256,
                admission_ordinal,
                normalized_logical_timeout_ms: Some(request.request.logical_timeout_ms),
                runtime_generation: request.request.runtime_generation,
                activated_mapping_backend_generation: request.request.backend_generation,
                correction_owner_backend_generation: self.owner_identity.backend_generation,
                session_id: &request.request.session_id,
                map_name: &request.request.map_name,
                session_generation: request.request.session_generation,
                map_frame_id: &self.map_frame_id,
                odom_frame_id: &self.odom_frame_id,
                body_frame_id: &self.body_frame_id,
                activation_identity_sha256: request.activation_identity_sha256,
                target_receipt_sha256: request.target_receipt_sha256,
                asset_manifest_sha256: request.asset_manifest_sha256,
                asset_bundle_sha256: request.asset_bundle_sha256,
                source_policy_version: request.request.source_policy_version,
                source_policy_sha256: request.source_policy_sha256,
                deadline_policy_version: request.request.deadline_policy_version,
                deadline_policy_sha256: request.deadline_policy_sha256,
                algorithm_options_version: request.request.algorithm_options_version,
                algorithm_options_sha256: request.algorithm_options_sha256,
                target: None,
                source: None,
                algorithm: None,
                candidate: None,
                terminal_disposition: disposition,
                correction_revision: 0,
                detail,
            };
            let evidence_preimage = canonical_relocalization_evidence_preimage_parts_v1(&evidence)
                .expect("validated preclaim evidence is canonical");
            let evidence_sha256 = sha256_bytes(&evidence_preimage.bytes);
            let evidence_disclosure_v1 = build_relocalization_evidence_disclosure_v1(
                &evidence_preimage.bytes,
                evidence_preimage.canonical_source_range,
            )
            .expect("validated preclaim evidence disclosure is bounded");
            StoredRelocalizationResult::new(build_relocalization_result(
                &request.request,
                request.identity_sha256,
                disposition,
                0,
                None,
                Vec::new(),
                evidence_sha256,
                evidence_disclosure_v1,
                0,
                detail,
            ))
        }

        fn busy_result(
            &self,
            request: &ValidatedRelocalizeRequest,
        ) -> crate::proto::RelocalizeResult {
            let mut result = self
                .terminal_result(
                    request,
                    None,
                    crate::proto::RelocalizeDisposition::Busy,
                    "RELOCALIZATION_OWNER_BUSY",
                )
                .result;
            result.terminal_proof_sha256.clear();
            result
        }

        fn invalid_result(
            &self,
            request: &crate::proto::RelocalizeRequest,
            detail: String,
        ) -> crate::proto::RelocalizeResult {
            let evidence_disclosure_v1 =
                build_relocalization_evidence_disclosure_v1(detail.as_bytes(), None)
                    .expect("invalid-request evidence disclosure is bounded");
            build_relocalization_result(
                request,
                invalid_relocalization_request_identity(request),
                crate::proto::RelocalizeDisposition::Rejected,
                0,
                None,
                Vec::new(),
                sha256_bytes(detail.as_bytes()),
                evidence_disclosure_v1,
                0,
                detail,
            )
        }

        fn settle_terminal_durable(
            &mut self,
            request_id: &str,
            identity_sha256: [u8; 32],
            terminal: StoredRelocalizationResult,
        ) -> Result<RelocalizationTerminalWaiters, CorrectionRuntimeError> {
            let request = self.ledger.request(request_id).ok_or_else(|| {
                CorrectionRuntimeError::Apply(
                    "relocalization request missing before durable terminal".to_owned(),
                )
            })?;
            let activation_identity_sha256 =
                relocalization_digest(&request.activation_identity_sha256, "activation identity")
                    .map_err(CorrectionRuntimeError::Apply)?;
            self.terminal_store
                .persist_terminal(
                    request_id,
                    identity_sha256,
                    activation_identity_sha256,
                    &terminal,
                )
                .map_err(CorrectionRuntimeError::Apply)?;
            self.ledger
                .settle_terminal(request_id, identity_sha256, terminal)
                .map_err(|detail| CorrectionRuntimeError::Apply(detail.to_owned()))
        }

        fn notify_terminal(&mut self, terminal: RelocalizationTerminalWaiters) {
            for waiter_id in terminal.waiter_ids {
                if let Some((_, _, capacity_lease)) = self.waiter_requests.remove(&waiter_id) {
                    capacity_lease.release();
                }
                if let Some(response) = self.waiter_responses.remove(&waiter_id) {
                    let _ = response.send(terminal.result.result.clone());
                }
            }
        }

        fn drop_synchronous_preclaim_execution(&mut self) {
            if matches!(
                self.execution,
                Some(RelocalizationExecution::Accumulating(_))
            ) {
                self.execution.take();
            }
        }

        fn clear_frame_proofs(&mut self) {
            self.last_validated_frames.clear();
            self.validated_frame_proofs.clear();
        }

        fn settle_active(
            &mut self,
            disposition: crate::proto::RelocalizeDisposition,
            detail: &str,
        ) -> Result<bool, CorrectionRuntimeError> {
            let Some((request_id, identity_sha256, ordinal, _, phase)) =
                self.ledger.active_identity()
            else {
                return Ok(false);
            };
            if phase == RelocalizationClaimPhase::Postclaim {
                return Ok(false);
            }
            let request = self.ledger.request(&request_id).cloned().ok_or_else(|| {
                CorrectionRuntimeError::Apply("relocalization request missing".to_owned())
            })?;
            let request =
                validate_relocalize_request(request).map_err(CorrectionRuntimeError::Apply)?;
            let terminal = self.terminal_result(&request, Some(ordinal), disposition, detail);
            let terminal = self.settle_terminal_durable(&request_id, identity_sha256, terminal)?;
            self.notify_terminal(terminal);
            self.clear_frame_proofs();
            Ok(true)
        }

        fn begin_runtime_execution(
            &mut self,
            request: ValidatedRelocalizeRequest,
            admission_ordinal: u64,
            mode_epoch: u64,
            logical_deadline: Instant,
            runtime: &RelocalizationRuntimeContext,
        ) -> Result<(), CorrectionRuntimeError> {
            let armed_map_watermarks = self
                .last_validated_frames
                .iter()
                .filter_map(|(scope, key)| {
                    (scope.mapping_session_id == request.request.session_id
                        && scope.mapping_session_generation == request.request.session_generation)
                        .then_some((scope.map_frame_generation, *key))
                })
                .collect();
            self.clear_frame_proofs();
            let Some(snapshot) = runtime
                .phase_a
                .activated_target_receipts()
                .current_sealed_target_snapshot()
            else {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::TargetUnavailable,
                    "RELOCALIZATION_TARGET_UNAVAILABLE",
                )?;
                return Ok(());
            };
            if let Err((disposition, detail)) =
                snapshot_matches_relocalize_request(&snapshot, &request, &self.map_frame_id)
            {
                self.settle_active(disposition, detail)?;
                return Ok(());
            }
            let now = Instant::now();
            let accumulation_deadline = now
                .checked_add(RELOCALIZATION_ACCUMULATION_TIMEOUT)
                .unwrap_or(logical_deadline)
                .min(logical_deadline);
            self.execution = Some(RelocalizationExecution::Accumulating(Box::new(
                RelocalizationAccumulation {
                    request,
                    admission_ordinal,
                    mode_epoch,
                    logical_deadline,
                    accumulation_deadline,
                    snapshot,
                    arm_watermark: None,
                    armed_map_watermarks,
                    accepted: Vec::new(),
                    aggregate_input_points: 0,
                    session_map_frame_generation: None,
                    mapping_commit_boot_nonce: None,
                },
            )));
            Ok(())
        }

        fn handle_admission(
            &mut self,
            admission: RelocalizationAdmissionRequest,
            detached_tx: &mpsc::UnboundedSender<RelocalizationWaiterEvent>,
            runtime: Option<&RelocalizationRuntimeContext>,
            phase_a: &CorrectionPhaseAControl,
        ) -> Result<(), CorrectionRuntimeError> {
            let RelocalizationAdmissionRequest {
                request: raw_request,
                capacity_lease,
                response,
                mapping_lifecycle_guard,
            } = admission;
            // Production gRPC admission supplies this guard.  It remains
            // alive through ledger activation and the admission response, so
            // no mapping completion can pass its final commit boundary while
            // the mode fence changes to active.
            let _mapping_lifecycle_guard = mapping_lifecycle_guard;
            let validated = validate_relocalize_request(raw_request.clone());
            let claim_fence = phase_a.command_claim_fence();
            let claim_guard = claim_fence.lock();
            if !phase_a.is_open() {
                let _ = response.send(Err(RelocalizationCommandError::Stopped));
                return Ok(());
            }
            let waiter_id = match self.next_waiter_id() {
                Ok(waiter_id) => waiter_id,
                Err(error) => {
                    let _ = response.send(Err(error));
                    return Ok(());
                }
            };
            let request = match validated {
                Ok(request) => request,
                Err(detail) => {
                    let waiter = Self::immediate_waiter(
                        waiter_id,
                        self.invalid_result(&raw_request, detail),
                        Arc::clone(&capacity_lease),
                        detached_tx,
                    );
                    let _ = response.send(Ok(waiter));
                    return Ok(());
                }
            };
            if self.execution.is_some()
                && self.ledger.request(&request.request.request_id).is_none()
            {
                let waiter = Self::immediate_waiter(
                    waiter_id,
                    self.busy_result(&request),
                    Arc::clone(&capacity_lease),
                    detached_tx,
                );
                let _ = response.send(Ok(waiter));
                return Ok(());
            }
            let conflict = self.terminal_result(
                &request,
                None,
                crate::proto::RelocalizeDisposition::ReplayConflict,
                "RELOCALIZATION_REPLAY_CONFLICT",
            );
            let outcome = self
                .ledger
                .admit(&request, waiter_id, Instant::now(), || conflict)
                .map_err(|detail| CorrectionRuntimeError::Apply(detail.to_owned()))?;
            let mut started = None;
            let mut close_after_reply = false;
            let waiter = match outcome {
                RelocalizationLedgerAdmission::Started {
                    admission_ordinal,
                    logical_deadline,
                } => {
                    self.terminal_store
                        .mark_pending(&request)
                        .map_err(CorrectionRuntimeError::Apply)?;
                    let mode_snapshot = phase_a.relocalization_mode().snapshot();
                    if !mode_snapshot.active {
                        return Err(CorrectionRuntimeError::Apply(
                            "relocalization mode did not activate with ledger admission".to_owned(),
                        ));
                    }
                    started = Some((admission_ordinal, mode_snapshot.epoch, logical_deadline));
                    self.retained_waiter(
                        waiter_id,
                        &request,
                        Arc::clone(&capacity_lease),
                        detached_tx,
                    )
                }
                RelocalizationLedgerAdmission::Joined { .. } => self.retained_waiter(
                    waiter_id,
                    &request,
                    Arc::clone(&capacity_lease),
                    detached_tx,
                ),
                RelocalizationLedgerAdmission::Replay(result)
                | RelocalizationLedgerAdmission::Conflict(result) => Self::immediate_waiter(
                    waiter_id,
                    result.result,
                    Arc::clone(&capacity_lease),
                    detached_tx,
                ),
                RelocalizationLedgerAdmission::ConflictOverflow(result) => {
                    close_after_reply = true;
                    Self::immediate_waiter(
                        waiter_id,
                        result.result,
                        Arc::clone(&capacity_lease),
                        detached_tx,
                    )
                }
                RelocalizationLedgerAdmission::Busy => Self::immediate_waiter(
                    waiter_id,
                    self.busy_result(&request),
                    Arc::clone(&capacity_lease),
                    detached_tx,
                ),
                RelocalizationLedgerAdmission::CapacityExceeded(detail) => {
                    if detail == "RELOCALIZATION_COMMAND_WAITER_CAPACITY_EXCEEDED" {
                        let admission_ordinal = self
                            .ledger
                            .records
                            .get(&request.request.request_id)
                            .map(|record| record.admission_ordinal)
                            .ok_or_else(|| {
                                CorrectionRuntimeError::Apply(
                                    "relocalization waiter capacity record missing".to_owned(),
                                )
                            })?;
                        let terminal = self.terminal_result(
                            &request,
                            Some(admission_ordinal),
                            crate::proto::RelocalizeDisposition::OwnerFailed,
                            detail,
                        );
                        let pending = self.settle_terminal_durable(
                            &request.request.request_id,
                            request.identity_sha256,
                            terminal,
                        )?;
                        let result = pending.result.result.clone();
                        self.notify_terminal(pending);
                        self.drop_synchronous_preclaim_execution();
                        Self::immediate_waiter(
                            waiter_id,
                            result,
                            Arc::clone(&capacity_lease),
                            detached_tx,
                        )
                    } else {
                        let terminal = self.terminal_result(
                            &request,
                            None,
                            crate::proto::RelocalizeDisposition::OwnerFailed,
                            detail,
                        );
                        self.terminal_store
                            .persist_terminal(
                                &request.request.request_id,
                                request.identity_sha256,
                                request.activation_identity_sha256,
                                &terminal,
                            )
                            .map_err(CorrectionRuntimeError::Apply)?;
                        self.ledger
                            .store_capacity_terminal(&request, Instant::now(), terminal.clone())
                            .map_err(|detail| CorrectionRuntimeError::Apply(detail.to_owned()))?;
                        close_after_reply = true;
                        Self::immediate_waiter(
                            waiter_id,
                            terminal.result,
                            Arc::clone(&capacity_lease),
                            detached_tx,
                        )
                    }
                }
            };
            if let (Some((admission_ordinal, mode_epoch, logical_deadline)), Some(runtime)) =
                (started, runtime)
            {
                self.begin_runtime_execution(
                    request,
                    admission_ordinal,
                    mode_epoch,
                    logical_deadline,
                    runtime,
                )?;
            }
            let _ = response.send(Ok(waiter));
            drop(claim_guard);
            if close_after_reply {
                phase_a.begin();
            }
            Ok(())
        }

        fn reply_frame(
            response: oneshot::Sender<Result<CorrectionSubmitDisposition, CorrectionRuntimeError>>,
            disposition: CorrectionSubmitDisposition,
        ) {
            let _ = response.send(Ok(disposition));
        }

        fn reject_frame(request: CorrectionFrameRequest, reason: impl Into<String>) {
            let reason = reason.into();
            Self::reply_frame(
                request.response,
                CorrectionSubmitDisposition::Rejected {
                    reason_code: crate::proto::RelocalizeDisposition::OwnerFailed as i32,
                    reason,
                },
            );
        }

        fn record_frame_proof(
            &mut self,
            input: &CorrectionFrameInput,
            key: RelocalizationFrameKey,
            evidence_sha256: [u8; 32],
        ) -> Result<bool, &'static str> {
            let scope = RelocalizationFrameScope::from_input(input);
            let proof_key = (scope.clone(), key);
            if let Some(existing) = self.validated_frame_proofs.get(&proof_key) {
                return if *existing == evidence_sha256 {
                    Ok(true)
                } else {
                    Err("RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT")
                };
            }
            let scope_is_new = !self.last_validated_frames.contains_key(&scope);
            if scope_is_new
                && self.last_validated_frames.len() >= RELOCALIZATION_MAX_FRAME_PROOF_SCOPES
            {
                return Err("RELOCALIZATION_FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED");
            }
            if self.validated_frame_proofs.len() >= RELOCALIZATION_MAX_FRAME_PROOFS_TOTAL {
                return Err("RELOCALIZATION_FRAME_PROOF_TOTAL_CAPACITY_EXCEEDED");
            }
            if self
                .validated_frame_proofs
                .keys()
                .filter(|(existing_scope, _)| existing_scope == &scope)
                .count()
                >= RELOCALIZATION_MAX_FRAME_PROOFS_PER_SCOPE
            {
                return Err("RELOCALIZATION_FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED");
            }
            self.validated_frame_proofs
                .insert(proof_key, evidence_sha256);
            let last = self.last_validated_frames.entry(scope).or_insert(key);
            if key > *last {
                *last = key;
            }
            Ok(false)
        }

        fn handle_frame(
            &mut self,
            request: CorrectionFrameRequest,
            runtime: &RelocalizationRuntimeContext,
            tasks: &mut tokio::task::JoinSet<RelocalizationTaskEvent>,
        ) -> Result<(), CorrectionRuntimeError> {
            if !matches!(
                self.execution,
                Some(RelocalizationExecution::Accumulating(_))
            ) {
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            let mode_epoch = match &self.execution {
                Some(RelocalizationExecution::Accumulating(accumulation)) => {
                    accumulation.mode_epoch
                }
                _ => unreachable!("accumulating execution checked above"),
            };
            if runtime.phase_a.relocalization_mode().snapshot()
                != (RelocalizationModeSnapshot {
                    epoch: mode_epoch,
                    active: true,
                })
                || request.input.frame_capture_relocalization_epoch != mode_epoch
                || request.input.relocalization_mode_epoch != Some(mode_epoch)
            {
                // A frame produced before admission, or after the active
                // mode changed, is not a source frame for this command.
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            let (key, evidence_sha256) = match relocalization_frame_evidence(
                &self.owner_identity,
                &self.map_frame_id,
                &self.odom_frame_id,
                &self.body_frame_id,
                &request.input,
            ) {
                Ok(evidence) => evidence,
                Err(detail) => {
                    self.settle_active(crate::proto::RelocalizeDisposition::OwnerFailed, &detail)?;
                    self.drop_synchronous_preclaim_execution();
                    Self::reject_frame(request, detail);
                    return Ok(());
                }
            };
            match self.record_frame_proof(&request.input, key, evidence_sha256) {
                Ok(true) => {
                    Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                    return Ok(());
                }
                Ok(false) => {}
                Err(detail) => {
                    self.settle_active(crate::proto::RelocalizeDisposition::OwnerFailed, detail)?;
                    self.drop_synchronous_preclaim_execution();
                    Self::reject_frame(request, detail);
                    runtime.phase_a.begin();
                    return Err(CorrectionRuntimeError::Apply(detail.to_owned()));
                }
            }

            let Some(execution) = self.execution.take() else {
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            };
            let RelocalizationExecution::Accumulating(mut accumulation) = execution else {
                self.execution = Some(execution);
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            };

            let current_snapshot = runtime
                .phase_a
                .activated_target_receipts()
                .current_sealed_target_snapshot();
            if !current_snapshot
                .as_ref()
                .is_some_and(|current| same_sealed_target(current, &accumulation.snapshot))
            {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::TargetInvalidated,
                    "RELOCALIZATION_TARGET_INVALIDATED_DURING_ACCUMULATION",
                )?;
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            if request.input.mapping_session_id != accumulation.request.request.session_id
                || request.input.mapping_session_generation
                    != accumulation.request.request.session_generation
            {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::SessionInvalidated,
                    "RELOCALIZATION_FRAME_SESSION_INVALIDATED",
                )?;
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            if accumulation
                .session_map_frame_generation
                .is_some_and(|generation| generation != request.input.map_frame_generation)
                || accumulation
                    .mapping_commit_boot_nonce
                    .is_some_and(|nonce| nonce != request.input.mapping_commit_id.boot_nonce)
            {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::SessionInvalidated,
                    "RELOCALIZATION_FRAME_GENERATION_INVALIDATED",
                )?;
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            if accumulation.session_map_frame_generation.is_none() {
                accumulation.arm_watermark = accumulation
                    .armed_map_watermarks
                    .get(&request.input.map_frame_generation)
                    .copied();
            }
            let last_accepted_key = accumulation
                .accepted
                .last()
                .map(|frame| relocalization_frame_key(&frame.input));
            if accumulation
                .arm_watermark
                .is_some_and(|watermark| key <= watermark)
                || last_accepted_key.is_some_and(|last| key <= last)
            {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }

            let frame_points = request.input.body_points.len();
            let Some(aggregate_input_points) = accumulation
                .aggregate_input_points
                .checked_add(frame_points)
            else {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::OwnerFailed,
                    "RELOCALIZATION_AGGREGATE_POINT_COUNT_OVERFLOW",
                )?;
                Self::reject_frame(request, "RELOCALIZATION_AGGREGATE_POINT_COUNT_OVERFLOW");
                return Ok(());
            };
            if aggregate_input_points > RELOCALIZATION_MAX_AGGREGATE_POINTS {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::InsufficientSource,
                    "RELOCALIZATION_AGGREGATE_POINT_BOUND_EXCEEDED",
                )?;
                Self::reject_frame(request, "RELOCALIZATION_AGGREGATE_POINT_BOUND_EXCEEDED");
                return Ok(());
            }
            accumulation.aggregate_input_points = aggregate_input_points;
            accumulation
                .session_map_frame_generation
                .get_or_insert(request.input.map_frame_generation);
            accumulation
                .mapping_commit_boot_nonce
                .get_or_insert(request.input.mapping_commit_id.boot_nonce);
            accumulation.accepted.push(AccumulatedRelocalizationFrame {
                evidence_sha256,
                input: request.input.clone(),
            });

            let minimum_reached = aggregate_input_points >= RELOCALIZATION_MIN_AGGREGATE_POINTS;
            let fresh_boundary = request.lease.has_lifecycle_guard();
            let maximum_reached = aggregate_input_points == RELOCALIZATION_MAX_AGGREGATE_POINTS;
            if maximum_reached && !fresh_boundary {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::InsufficientSource,
                    "RELOCALIZATION_MAXIMUM_AGGREGATE_FRESH_BOUNDARY_UNAVAILABLE",
                )?;
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            let cutoff_reason = if maximum_reached {
                Some(RelocalizationCutoffReasonV1::MaximumAggregateReached)
            } else {
                (minimum_reached && fresh_boundary)
                    .then_some(RelocalizationCutoffReasonV1::MinimumAggregateReached)
            };
            if cutoff_reason.is_none() && accumulation.accepted.len() >= RELOCALIZATION_MAX_FRAMES {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::InsufficientSource,
                    if minimum_reached {
                        "RELOCALIZATION_FRESH_BOUNDARY_UNAVAILABLE"
                    } else {
                        "RELOCALIZATION_INSUFFICIENT_SOURCE_MAX_FRAMES"
                    },
                )?;
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            let Some(cutoff_reason) = cutoff_reason else {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            };

            let request_metadata = accumulation.request.clone();
            let request_id = request_metadata.request.request_id.clone();
            let identity_sha256 = request_metadata.identity_sha256;
            let admission_ordinal = accumulation.admission_ordinal;
            let logical_deadline = accumulation.logical_deadline;
            Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
            let source = match seal_relocalization_source(
                *accumulation,
                request.input,
                request.lease,
                cutoff_reason,
            ) {
                Ok(source) => source,
                Err(detail) => {
                    self.settle_active(crate::proto::RelocalizeDisposition::OwnerFailed, &detail)?;
                    return Ok(());
                }
            };
            self.execution = Some(RelocalizationExecution::Computing {
                request: Box::new(request_metadata),
                admission_ordinal,
                logical_deadline,
            });
            let source_points = Arc::clone(&source.source_points);
            let target_points = Arc::clone(&source.target_points);
            let base_anchor = source.boundary_input.base_anchor_map_from_odom;
            #[cfg(test)]
            let compute_delay_ms = runtime.compute_delay_ms.load(Ordering::Acquire);
            #[cfg(test)]
            let compute_panic = runtime.compute_panic.load(Ordering::Acquire);
            tasks.spawn_blocking(move || RelocalizationTaskEvent::Computed {
                request_id,
                identity_sha256,
                source: Box::new(source),
                output: {
                    #[cfg(test)]
                    {
                        if compute_delay_ms > 0 {
                            std::thread::sleep(Duration::from_millis(compute_delay_ms));
                        }
                        assert!(!compute_panic, "injected relocalization compute panic");
                    }
                    run_relocalization_compute(
                        source_points.as_ref(),
                        target_points.as_ref(),
                        base_anchor,
                    )
                },
            });
            Ok(())
        }

        fn completed_terminal_result(
            &self,
            computed: &RelocalizationComputedCommand,
            disposition: crate::proto::RelocalizeDisposition,
            candidate: Option<&CorrectionCandidate>,
            correction_revision: u64,
            detail: &str,
        ) -> Result<StoredRelocalizationResult, CorrectionRuntimeError> {
            let source = &computed.source;
            let evidence = RelocalizationEvidenceV1 {
                request_identity_sha256: computed.request.identity_sha256,
                admission_ordinal: Some(computed.admission_ordinal),
                normalized_logical_timeout_ms: Some(computed.request.request.logical_timeout_ms),
                runtime_generation: computed.request.request.runtime_generation,
                activated_mapping_backend_generation: computed.request.request.backend_generation,
                correction_owner_backend_generation: self.owner_identity.backend_generation,
                session_id: &computed.request.request.session_id,
                map_name: &computed.request.request.map_name,
                session_generation: computed.request.request.session_generation,
                map_frame_id: &self.map_frame_id,
                odom_frame_id: &self.odom_frame_id,
                body_frame_id: &self.body_frame_id,
                activation_identity_sha256: computed.request.activation_identity_sha256,
                target_receipt_sha256: computed.request.target_receipt_sha256,
                asset_manifest_sha256: computed.request.asset_manifest_sha256,
                asset_bundle_sha256: computed.request.asset_bundle_sha256,
                source_policy_version: computed.request.request.source_policy_version,
                source_policy_sha256: computed.request.source_policy_sha256,
                deadline_policy_version: computed.request.request.deadline_policy_version,
                deadline_policy_sha256: computed.request.deadline_policy_sha256,
                algorithm_options_version: computed.request.request.algorithm_options_version,
                algorithm_options_sha256: computed.request.algorithm_options_sha256,
                target: Some(RelocalizationTargetEvidenceV1 {
                    snapshot: &source.snapshot,
                    decoded_point_count: source.target_decoded_point_count,
                    decoded_f64_bytes: source.target_decoded_f64_bytes,
                    decoded_sha256: source.target_decoded_sha256,
                }),
                source: Some(RelocalizationSourceEvidenceV1 {
                    arm_watermark: source.arm_watermark,
                    cutoff_reason: source.cutoff_reason,
                    last_accepted_key: source
                        .frames
                        .last()
                        .map(|frame| relocalization_frame_key(&frame.input)),
                    frames: &source.frames,
                    aggregate_input_points: source.aggregate_input_points,
                    aggregate_input_f64_bytes: source.aggregate_input_f64_bytes,
                    post_transform_points: source.post_transform_points,
                    post_transform_f64_bytes: source.post_transform_f64_bytes,
                    emitted_points: u64::try_from(source.source_points.len()).map_err(|_| {
                        CorrectionRuntimeError::Apply(
                            "relocalization emitted point count overflow".to_owned(),
                        )
                    })?,
                    canonical_source_bytes: &source.canonical_source_bytes,
                    source_sha256: source.source_sha256,
                    boundary_key: relocalization_frame_key(&source.boundary_input),
                    boundary_base_anchor_map_from_odom: &source
                        .boundary_input
                        .base_anchor_map_from_odom,
                }),
                algorithm: Some(RelocalizationAlgorithmEvidenceV1 {
                    result: &computed.output.result,
                    accepted: computed.output.correction.is_some(),
                    correction: computed.output.correction.as_ref(),
                }),
                candidate: candidate
                    .map(|candidate| RelocalizationCandidateEvidenceV1 { candidate }),
                terminal_disposition: disposition,
                correction_revision,
                detail,
            };
            let evidence_preimage = canonical_relocalization_evidence_preimage_parts_v1(&evidence)
                .map_err(CorrectionRuntimeError::Apply)?;
            let evidence_sha256 = canonical_relocalization_evidence_sha256_v1(&evidence_preimage)
                .map_err(CorrectionRuntimeError::Apply)?;
            let evidence_disclosure_v1 = build_relocalization_evidence_disclosure_v1(
                &evidence_preimage.bytes,
                evidence_preimage.canonical_source_range,
            )
            .map_err(CorrectionRuntimeError::Apply)?;
            let candidate_payload_preimage_v1 =
                relocalization_candidate_payload_preimage_v1(candidate)
                    .map_err(CorrectionRuntimeError::Apply)?;
            Ok(StoredRelocalizationResult::new(
                build_relocalization_result(
                    &computed.request.request,
                    computed.request.identity_sha256,
                    disposition,
                    candidate.map_or(0, |candidate| candidate.identity.candidate_sequence),
                    candidate.map(|candidate| candidate.payload_sha256),
                    candidate_payload_preimage_v1,
                    evidence_sha256,
                    evidence_disclosure_v1,
                    correction_revision,
                    detail,
                ),
            ))
        }

        fn settle_completed(
            &mut self,
            computed: &RelocalizationComputedCommand,
            disposition: crate::proto::RelocalizeDisposition,
            candidate: Option<&CorrectionCandidate>,
            correction_revision: u64,
            detail: &str,
        ) -> Result<(), CorrectionRuntimeError> {
            let terminal = self.completed_terminal_result(
                computed,
                disposition,
                candidate,
                correction_revision,
                detail,
            )?;
            let terminal = self.settle_terminal_durable(
                &computed.request.request.request_id,
                computed.request.identity_sha256,
                terminal,
            )?;
            self.notify_terminal(terminal);
            self.clear_frame_proofs();
            Ok(())
        }

        fn current_target_matches(
            runtime: &RelocalizationRuntimeContext,
            computed: &RelocalizationComputedCommand,
        ) -> bool {
            runtime
                .phase_a
                .activated_target_receipts()
                .current_sealed_target_snapshot()
                .is_some_and(|snapshot| same_sealed_target(&snapshot, &computed.source.snapshot))
        }

        fn schedule_staged_discard(
            computed: &RelocalizationComputedCommand,
            runtime: &RelocalizationRuntimeContext,
            tasks: &mut tokio::task::JoinSet<RelocalizationTaskEvent>,
        ) {
            let cleanup = runtime.cleanup.clone();
            let input = computed.source.boundary_input.clone();
            tasks.spawn(async move {
                RelocalizationTaskEvent::DiscardScheduled(schedule_discard(&cleanup, input).await)
            });
        }

        fn preclaim_is_valid(
            runtime: &RelocalizationRuntimeContext,
            computed: &RelocalizationComputedCommand,
        ) -> bool {
            runtime.phase_a.is_open()
                && Instant::now() < computed.logical_deadline
                && computed.source.boundary_lease.has_lifecycle_guard()
                && Self::current_target_matches(runtime, computed)
        }

        fn handle_task_event(
            &mut self,
            event: RelocalizationTaskEvent,
            runtime: &RelocalizationRuntimeContext,
            tasks: &mut tokio::task::JoinSet<RelocalizationTaskEvent>,
        ) -> Result<(), CorrectionRuntimeError> {
            match event {
                RelocalizationTaskEvent::Computed {
                    request_id,
                    identity_sha256,
                    source,
                    output,
                } => {
                    let Some(RelocalizationExecution::Computing {
                        request,
                        admission_ordinal,
                        logical_deadline,
                    }) = self.execution.take()
                    else {
                        return Ok(());
                    };
                    if request.request.request_id != request_id
                        || request.identity_sha256 != identity_sha256
                    {
                        runtime.phase_a.begin();
                        return Err(CorrectionRuntimeError::Apply(
                            "relocalization compute identity mismatch".to_owned(),
                        ));
                    }
                    let computed = RelocalizationComputedCommand {
                        request: *request,
                        admission_ordinal,
                        logical_deadline,
                        source: *source,
                        output,
                    };
                    if self
                        .ledger
                        .terminal(&computed.request.request.request_id)
                        .is_some()
                    {
                        return Ok(());
                    }
                    if computed.output.correction.is_none() {
                        self.settle_completed(
                            &computed,
                            crate::proto::RelocalizeDisposition::AlgorithmRejected,
                            None,
                            0,
                            "RELOCALIZATION_ALGORITHM_REJECTED",
                        )?;
                        return Ok(());
                    }
                    if !Self::preclaim_is_valid(runtime, &computed) {
                        self.settle_completed(
                            &computed,
                            if Instant::now() >= computed.logical_deadline {
                                crate::proto::RelocalizeDisposition::DeadlineExceeded
                            } else if !runtime.phase_a.is_open() {
                                crate::proto::RelocalizeDisposition::OwnerStopped
                            } else {
                                crate::proto::RelocalizeDisposition::TargetInvalidated
                            },
                            None,
                            0,
                            "RELOCALIZATION_PRECLAIM_REVALIDATION_FAILED",
                        )?;
                        return Ok(());
                    }
                    let sender = runtime.finalizer.sender.clone();
                    tasks.spawn(async move {
                        RelocalizationTaskEvent::PermitReserved(
                            sender
                                .reserve_owned()
                                .await
                                .map_err(|_| "RELOCALIZATION_FINALIZER_INGRESS_CLOSED"),
                        )
                    });
                    self.execution = Some(RelocalizationExecution::Reserving {
                        computed: Some(Box::new(computed)),
                    });
                }
                RelocalizationTaskEvent::PermitReserved(permit) => {
                    let Some(RelocalizationExecution::Reserving { mut computed }) =
                        self.execution.take()
                    else {
                        return Ok(());
                    };
                    let computed = *computed.take().ok_or_else(|| {
                        CorrectionRuntimeError::Apply(
                            "relocalization computed state missing".to_owned(),
                        )
                    })?;
                    let permit = match permit {
                        Ok(permit) => permit,
                        Err(detail) => {
                            self.settle_completed(
                                &computed,
                                crate::proto::RelocalizeDisposition::OwnerFailed,
                                None,
                                0,
                                detail,
                            )?;
                            return Ok(());
                        }
                    };
                    if self
                        .ledger
                        .terminal(&computed.request.request.request_id)
                        .is_some()
                    {
                        return Ok(());
                    }
                    if !Self::preclaim_is_valid(runtime, &computed) {
                        self.settle_completed(
                            &computed,
                            crate::proto::RelocalizeDisposition::TargetInvalidated,
                            None,
                            0,
                            "RELOCALIZATION_PRESTAGE_REVALIDATION_FAILED",
                        )?;
                        return Ok(());
                    }
                    let sink = Arc::clone(&runtime.sink);
                    let input = computed.source.boundary_input.clone();
                    tasks.spawn(async move {
                        RelocalizationTaskEvent::Staged(sink.stage_frame(&input).await)
                    });
                    self.execution = Some(RelocalizationExecution::Staging {
                        computed: Some(Box::new(computed)),
                        permit: Some(permit),
                    });
                }
                RelocalizationTaskEvent::Staged(stage_result) => {
                    let Some(RelocalizationExecution::Staging {
                        mut computed,
                        mut permit,
                    }) = self.execution.take()
                    else {
                        return Ok(());
                    };
                    let computed = *computed.take().ok_or_else(|| {
                        CorrectionRuntimeError::Apply(
                            "relocalization staging state missing".to_owned(),
                        )
                    })?;
                    let permit = permit.take().ok_or_else(|| {
                        CorrectionRuntimeError::Apply(
                            "relocalization finalizer permit missing".to_owned(),
                        )
                    })?;
                    if let Err(detail) = stage_result {
                        Self::schedule_staged_discard(&computed, runtime, tasks);
                        self.settle_completed(
                            &computed,
                            crate::proto::RelocalizeDisposition::OwnerFailed,
                            None,
                            0,
                            &detail,
                        )?;
                        return Ok(());
                    }
                    if self
                        .ledger
                        .terminal(&computed.request.request.request_id)
                        .is_some()
                        || !Self::preclaim_is_valid(runtime, &computed)
                    {
                        if self
                            .ledger
                            .terminal(&computed.request.request.request_id)
                            .is_none()
                        {
                            self.settle_completed(
                                &computed,
                                crate::proto::RelocalizeDisposition::TargetInvalidated,
                                None,
                                0,
                                "RELOCALIZATION_CLAIM_REVALIDATION_FAILED",
                            )?;
                        }
                        Self::schedule_staged_discard(&computed, runtime, tasks);
                        return Ok(());
                    }

                    let claim_fence = runtime.phase_a.command_claim_fence();
                    let _claim_guard = claim_fence.lock();
                    if !Self::preclaim_is_valid(runtime, &computed) {
                        drop(_claim_guard);
                        self.settle_completed(
                            &computed,
                            crate::proto::RelocalizeDisposition::TargetInvalidated,
                            None,
                            0,
                            "RELOCALIZATION_CLAIM_FENCE_REVALIDATION_FAILED",
                        )?;
                        Self::schedule_staged_discard(&computed, runtime, tasks);
                        return Ok(());
                    }
                    let candidate_sequence =
                        self.next_candidate_sequence.checked_add(1).ok_or_else(|| {
                            CorrectionRuntimeError::Apply(
                                "relocalization candidate sequence exhausted".to_owned(),
                            )
                        })?;
                    let correction = computed.output.correction.as_ref().ok_or_else(|| {
                        CorrectionRuntimeError::Apply(
                            "accepted relocalization correction missing".to_owned(),
                        )
                    })?;
                    let candidate = CorrectionCandidate::seal(
                        correction_candidate_identity(
                            CorrectionCandidateBackend::Relocalization,
                            &self.owner_identity,
                            candidate_sequence,
                            &self.map_frame_id,
                            &self.odom_frame_id,
                            &self.body_frame_id,
                            &computed.source.boundary_input,
                        ),
                        CorrectionTransform::from_isometry(correction),
                    )
                    .map_err(|error| CorrectionRuntimeError::Apply(error.to_string()))?;
                    let key = CandidateApplyKey {
                        candidate_sequence,
                        payload_sha256: candidate.payload_sha256,
                    };
                    let apply_rx = match runtime
                        .apply_waiters
                        .register(key, Arc::clone(&computed.source.boundary_lease))
                    {
                        Ok(apply_rx) => apply_rx,
                        Err(detail) => {
                            drop(_claim_guard);
                            self.settle_completed(
                                &computed,
                                crate::proto::RelocalizeDisposition::OwnerFailed,
                                None,
                                0,
                                &detail,
                            )?;
                            Self::schedule_staged_discard(&computed, runtime, tasks);
                            return Ok(());
                        }
                    };
                    let Some(lease) = runtime.apply_waiters.claim_apply(key) else {
                        runtime.apply_waiters.remove(key);
                        drop(_claim_guard);
                        self.settle_completed(
                            &computed,
                            crate::proto::RelocalizeDisposition::OwnerFailed,
                            None,
                            0,
                            "RELOCALIZATION_APPLY_WAITER_CLAIM_FAILED",
                        )?;
                        Self::schedule_staged_discard(&computed, runtime, tasks);
                        return Ok(());
                    };
                    if let Err(detail) = self.ledger.mark_postclaim(
                        &computed.request.request.request_id,
                        computed.request.identity_sha256,
                    ) {
                        runtime.apply_waiters.remove(key);
                        lease.release();
                        drop(_claim_guard);
                        runtime.phase_a.begin();
                        return Err(CorrectionRuntimeError::Apply(detail.to_owned()));
                    }
                    let (outcome, outcome_rx) = oneshot::channel();
                    permit.send(CanonicalFinalizerJob {
                        key,
                        candidate: candidate.clone(),
                        lease,
                        release_lease_on_completion: true,
                        outcome,
                    });
                    self.next_candidate_sequence = candidate_sequence;
                    drop(_claim_guard);
                    let sink = Arc::clone(&runtime.sink);
                    tasks.spawn(async move {
                        let (apply, outcome) = tokio::join!(apply_rx, outcome_rx);
                        let correction_revision = sink.canonical_correction_revision().await;
                        RelocalizationTaskEvent::Finalized {
                            apply,
                            outcome,
                            correction_revision,
                        }
                    });
                    self.execution = Some(RelocalizationExecution::Finalizing {
                        computed: Some(Box::new(computed)),
                        candidate: Box::new(candidate),
                    });
                }
                RelocalizationTaskEvent::Finalized {
                    apply,
                    outcome,
                    correction_revision,
                } => {
                    let Some(RelocalizationExecution::Finalizing {
                        mut computed,
                        candidate,
                    }) = self.execution.take()
                    else {
                        return Ok(());
                    };
                    let computed = *computed.take().ok_or_else(|| {
                        CorrectionRuntimeError::Apply(
                            "relocalization finalizing state missing".to_owned(),
                        )
                    })?;
                    match (apply, outcome) {
                        (Ok(Ok(CorrectionDisposition::Applied)), Ok(Ok(()))) => {
                            self.settle_completed(
                                &computed,
                                crate::proto::RelocalizeDisposition::Applied,
                                Some(candidate.as_ref()),
                                correction_revision,
                                "RELOCALIZATION_APPLIED",
                            )?;
                        }
                        (Ok(Ok(CorrectionDisposition::AlreadyApplied)), Ok(Ok(()))) => {
                            self.settle_completed(
                                &computed,
                                crate::proto::RelocalizeDisposition::AlreadyApplied,
                                Some(candidate.as_ref()),
                                correction_revision,
                                "RELOCALIZATION_ALREADY_APPLIED",
                            )?;
                        }
                        (Ok(Ok(CorrectionDisposition::Rejected(reason))), Ok(Ok(()))) => {
                            let detail = reason.to_string();
                            self.settle_completed(
                                &computed,
                                crate::proto::RelocalizeDisposition::Rejected,
                                Some(candidate.as_ref()),
                                correction_revision,
                                &detail,
                            )?;
                        }
                        (apply, outcome) => {
                            let detail = format!(
                                "post-claim relocalization finalizer mismatch: apply={apply:?}, outcome={outcome:?}"
                            );
                            runtime.phase_a.begin();
                            return Err(CorrectionRuntimeError::FinalizerPoisoned(detail));
                        }
                    }
                }
                RelocalizationTaskEvent::DiscardScheduled(result) => {
                    if let Err(error) = result {
                        runtime.phase_a.begin();
                        return Err(error);
                    }
                }
            }
            Ok(())
        }

        fn handle_detached(&mut self, waiter_id: u64) -> Result<(), CorrectionRuntimeError> {
            self.waiter_responses.remove(&waiter_id);
            let Some((request_id, identity_sha256, capacity_lease)) =
                self.waiter_requests.remove(&waiter_id)
            else {
                return Ok(());
            };
            capacity_lease.release();
            let cancel = self
                .ledger
                .detach_waiter(&request_id, identity_sha256, waiter_id)
                .map_err(|detail| CorrectionRuntimeError::Apply(detail.to_owned()))?;
            if cancel {
                self.settle_active(
                    crate::proto::RelocalizeDisposition::Cancelled,
                    "RELOCALIZATION_LAST_WAITER_CANCELLED",
                )?;
                self.drop_synchronous_preclaim_execution();
            }
            Ok(())
        }
    }

    async fn run_relocalization_command_owner(
        mut admissions: mpsc::Receiver<RelocalizationAdmissionRequest>,
        mut terminal_queries: mpsc::Receiver<RelocalizationTerminalQueryRequest>,
        mut detached: mpsc::UnboundedReceiver<RelocalizationWaiterEvent>,
        detached_tx: mpsc::UnboundedSender<RelocalizationWaiterEvent>,
        config: RelocalizationOwnerConfig,
    ) -> Result<(), CorrectionRuntimeError> {
        let RelocalizationOwnerConfig {
            phase_a,
            owner_identity,
            map_frame_id,
            odom_frame_id,
            body_frame_id,
            runtime,
            terminal_store,
        } = config;
        let mut actor = RelocalizationCommandActor::new_with_terminal_store(
            owner_identity,
            map_frame_id,
            odom_frame_id,
            body_frame_id,
            terminal_store,
        );
        actor.ledger.relocalization_mode = phase_a.relocalization_mode();
        match runtime {
            Some(runtime) => {
                run_relocalization_runtime_loop(
                    &mut actor,
                    &mut admissions,
                    &mut terminal_queries,
                    &mut detached,
                    &detached_tx,
                    &phase_a,
                    runtime,
                )
                .await
            }
            None => {
                run_relocalization_ledger_loop(
                    &mut actor,
                    &mut admissions,
                    &mut terminal_queries,
                    &mut detached,
                    &detached_tx,
                    &phase_a,
                )
                .await
            }
        }
    }

    async fn run_relocalization_ledger_loop(
        actor: &mut RelocalizationCommandActor,
        admissions: &mut mpsc::Receiver<RelocalizationAdmissionRequest>,
        terminal_queries: &mut mpsc::Receiver<RelocalizationTerminalQueryRequest>,
        detached: &mut mpsc::UnboundedReceiver<RelocalizationWaiterEvent>,
        detached_tx: &mpsc::UnboundedSender<RelocalizationWaiterEvent>,
        phase_a: &CorrectionPhaseAControl,
    ) -> Result<(), CorrectionRuntimeError> {
        let mut admission_rx = phase_a.admission_tx.subscribe();
        loop {
            let deadline = actor.ledger.active_identity().map(|active| active.3);
            tokio::select! {
                biased;
                _ = wait_for_admission_close(&mut admission_rx) => {
                    admissions.close();
                    actor.settle_active(
                        crate::proto::RelocalizeDisposition::OwnerStopped,
                        "RELOCALIZATION_OWNER_STOPPED_PRECLAIM",
                    )?;
                    return Ok(());
                }
                query = terminal_queries.recv() => match query {
                    Some(query) => actor.handle_terminal_query(query),
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                event = detached.recv() => match event {
                    Some(RelocalizationWaiterEvent::Detached(waiter_id)) => {
                        actor.handle_detached(waiter_id)?;
                    }
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                admission = admissions.recv() => match admission {
                    Some(admission) => {
                        actor.handle_admission(admission, detached_tx, None, phase_a)?
                    }
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                _ = wait_until_optional_instant(deadline), if deadline.is_some() => {
                    actor.settle_active(
                        crate::proto::RelocalizeDisposition::DeadlineExceeded,
                        "RELOCALIZATION_LOGICAL_DEADLINE_EXCEEDED_PRECLAIM",
                    )?;
                }
            }
        }
    }

    async fn run_relocalization_runtime_loop(
        actor: &mut RelocalizationCommandActor,
        admissions: &mut mpsc::Receiver<RelocalizationAdmissionRequest>,
        terminal_queries: &mut mpsc::Receiver<RelocalizationTerminalQueryRequest>,
        detached: &mut mpsc::UnboundedReceiver<RelocalizationWaiterEvent>,
        detached_tx: &mpsc::UnboundedSender<RelocalizationWaiterEvent>,
        phase_a: &CorrectionPhaseAControl,
        mut runtime: RelocalizationRuntimeContext,
    ) -> Result<(), CorrectionRuntimeError> {
        let mut admission_rx = phase_a.admission_tx.subscribe();
        let mut target_poll = tokio::time::interval(RELOCALIZATION_TARGET_POLL_INTERVAL);
        target_poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut tasks = tokio::task::JoinSet::new();
        let mut shutting_down = false;
        loop {
            if shutting_down && actor.execution.is_none() && tasks.is_empty() {
                return Ok(());
            }
            let logical_deadline = actor.ledger.active_identity().map(|active| active.3);
            let accumulation_deadline = match actor.execution.as_ref() {
                Some(RelocalizationExecution::Accumulating(accumulation)) => {
                    Some(accumulation.accumulation_deadline)
                }
                _ => None,
            };
            let timer_deadline = match (logical_deadline, accumulation_deadline) {
                (Some(logical), Some(accumulation)) => Some(logical.min(accumulation)),
                (Some(deadline), None) | (None, Some(deadline)) => Some(deadline),
                (None, None) => None,
            };
            tokio::select! {
                biased;
                _ = wait_for_admission_close(&mut admission_rx), if !shutting_down => {
                    shutting_down = true;
                    admissions.close();
                    runtime.frames.close();
                    actor.settle_active(
                        crate::proto::RelocalizeDisposition::OwnerStopped,
                        "RELOCALIZATION_OWNER_STOPPED_PRECLAIM",
                    )?;
                    actor.drop_synchronous_preclaim_execution();
                }
                _ = wait_until_optional_instant(timer_deadline), if timer_deadline.is_some() => {
                    let now = Instant::now();
                    if logical_deadline.is_some_and(|deadline| now >= deadline) {
                        actor.settle_active(
                            crate::proto::RelocalizeDisposition::DeadlineExceeded,
                            "RELOCALIZATION_LOGICAL_DEADLINE_EXCEEDED_PRECLAIM",
                        )?;
                        actor.drop_synchronous_preclaim_execution();
                    } else if accumulation_deadline.is_some_and(|deadline| now >= deadline) {
                        actor.settle_active(
                            crate::proto::RelocalizeDisposition::InsufficientSource,
                            "RELOCALIZATION_ACCUMULATION_TIMEOUT_INSUFFICIENT_SOURCE",
                        )?;
                        actor.drop_synchronous_preclaim_execution();
                    }
                }
                joined = tasks.join_next(), if !tasks.is_empty() => match joined {
                    Some(Ok(event)) => actor.handle_task_event(event, &runtime, &mut tasks)?,
                    Some(Err(error)) => {
                        actor.settle_active(
                            crate::proto::RelocalizeDisposition::OwnerFailed,
                            "RELOCALIZATION_RETAINED_TASK_FAILED",
                        )?;
                        phase_a.begin();
                        return Err(CorrectionRuntimeError::WorkerJoin(format!(
                            "relocalization retained task join failed: {error}"
                        )));
                    }
                    None => {}
                },
                query = terminal_queries.recv() => match query {
                    Some(query) => actor.handle_terminal_query(query),
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                event = detached.recv() => match event {
                    Some(RelocalizationWaiterEvent::Detached(waiter_id)) => {
                        actor.handle_detached(waiter_id)?;
                    }
                    None if shutting_down => {}
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                _ = target_poll.tick(), if !shutting_down => {
                    let invalidated = match actor.execution.as_ref() {
                        Some(RelocalizationExecution::Accumulating(accumulation)) => runtime
                            .phase_a
                            .activated_target_receipts()
                            .current_sealed_target_snapshot()
                            .is_none_or(|snapshot| {
                                !same_sealed_target(&snapshot, &accumulation.snapshot)
                            }),
                        _ => false,
                    };
                    if invalidated {
                        actor.settle_active(
                            crate::proto::RelocalizeDisposition::TargetInvalidated,
                            "RELOCALIZATION_TARGET_INVALIDATED_DURING_ACCUMULATION",
                        )?;
                        actor.drop_synchronous_preclaim_execution();
                    }
                }
                frame = runtime.frames.recv(), if !shutting_down => match frame {
                    Some(frame) => actor.handle_frame(frame, &runtime, &mut tasks)?,
                    None => {
                        phase_a.begin();
                        return Err(CorrectionRuntimeError::IngressClosed);
                    }
                },
                admission = admissions.recv(), if !shutting_down => match admission {
                    Some(admission) => {
                        actor.handle_admission(admission, detached_tx, Some(&runtime), phase_a)?
                    }
                    None => {
                        phase_a.begin();
                        return Err(CorrectionRuntimeError::IngressClosed);
                    }
                },
            }
        }
    }

    async fn wait_until_optional_instant(deadline: Option<Instant>) {
        match deadline {
            Some(deadline) => {
                tokio::time::sleep_until(tokio::time::Instant::from_std(deadline)).await
            }
            None => std::future::pending::<()>().await,
        }
    }

    // 字段标签与 protobuf 字段号一致。字段 27 在计算原像时必须为空，
    // 以消除 evidence -> terminal proof 的摘要环。
    fn canonical_relocalization_terminal_proof_preimage_v1(
        result: &crate::proto::RelocalizeResult,
    ) -> Result<Vec<u8>, String> {
        if !result.terminal_proof_sha256.is_empty() {
            return Err("RELOCALIZATION_TERMINAL_PROOF_MUST_BE_CLEARED".to_owned());
        }

        let mut out = RELOCALIZATION_TERMINAL_PROOF_DOMAIN_V1.to_vec();
        out.extend_from_slice(&RELOCALIZATION_SCHEMA_VERSION_V1.to_be_bytes());

        macro_rules! tag {
            ($value:expr) => {
                out.extend_from_slice(&($value).to_be_bytes())
            };
        }

        tag!(1_u32);
        out.extend_from_slice(&result.schema_version.to_be_bytes());
        tag!(2_u32);
        canonical_push_string(&mut out, &result.request_id);
        tag!(3_u32);
        canonical_push_present_bytes(&mut out, &result.request_identity_sha256);
        tag!(4_u32);
        out.extend_from_slice(&result.runtime_generation.to_be_bytes());
        tag!(5_u32);
        canonical_push_string(&mut out, &result.session_id);
        tag!(6_u32);
        canonical_push_string(&mut out, &result.map_name);
        tag!(7_u32);
        out.extend_from_slice(&result.session_generation.to_be_bytes());
        tag!(8_u32);
        out.extend_from_slice(&result.backend_generation.to_be_bytes());
        tag!(9_u32);
        canonical_push_present_bytes(&mut out, &result.activation_identity_sha256);
        tag!(10_u32);
        canonical_push_present_bytes(&mut out, &result.target_receipt_sha256);
        tag!(11_u32);
        out.extend_from_slice(&result.target_cloud_revision.to_be_bytes());
        tag!(12_u32);
        canonical_push_present_bytes(&mut out, &result.target_cloud_sha256);
        tag!(13_u32);
        canonical_push_present_bytes(&mut out, &result.asset_manifest_sha256);
        tag!(14_u32);
        canonical_push_present_bytes(&mut out, &result.asset_bundle_sha256);
        tag!(15_u32);
        out.extend_from_slice(&result.source_policy_version.to_be_bytes());
        tag!(16_u32);
        canonical_push_present_bytes(&mut out, &result.source_policy_sha256);
        tag!(17_u32);
        out.extend_from_slice(&result.deadline_policy_version.to_be_bytes());
        tag!(18_u32);
        canonical_push_present_bytes(&mut out, &result.deadline_policy_sha256);
        tag!(19_u32);
        out.extend_from_slice(&result.logical_timeout_ms.to_be_bytes());
        tag!(20_u32);
        out.extend_from_slice(&result.algorithm_options_version.to_be_bytes());
        tag!(21_u32);
        canonical_push_present_bytes(&mut out, &result.algorithm_options_sha256);
        tag!(22_u32);
        out.extend_from_slice(&result.disposition.to_be_bytes());
        tag!(23_u32);
        out.extend_from_slice(&result.candidate_sequence.to_be_bytes());
        tag!(24_u32);
        canonical_push_present_bytes(&mut out, &result.candidate_payload_sha256);
        tag!(25_u32);
        canonical_push_present_bytes(&mut out, &result.evidence_sha256);
        tag!(26_u32);
        out.extend_from_slice(&result.correction_revision.to_be_bytes());
        tag!(27_u32);
        out.push(0);
        tag!(28_u32);
        canonical_push_string(&mut out, &result.detail);
        Ok(out)
    }

    #[allow(clippy::too_many_arguments)]
    fn build_relocalization_result(
        request: &crate::proto::RelocalizeRequest,
        request_identity_sha256: [u8; 32],
        disposition: crate::proto::RelocalizeDisposition,
        candidate_sequence: u64,
        candidate_payload_sha256: Option<[u8; 32]>,
        candidate_payload_preimage_v1: Vec<u8>,
        evidence_sha256: [u8; 32],
        evidence_disclosure_v1: Vec<u8>,
        correction_revision: u64,
        detail: impl Into<String>,
    ) -> crate::proto::RelocalizeResult {
        let detail = detail.into();
        let mut result = crate::proto::RelocalizeResult {
            schema_version: RELOCALIZATION_SCHEMA_VERSION_V1,
            request_id: request.request_id.clone(),
            request_identity_sha256: request_identity_sha256.to_vec(),
            runtime_generation: request.runtime_generation,
            session_id: request.session_id.clone(),
            map_name: request.map_name.clone(),
            session_generation: request.session_generation,
            backend_generation: request.backend_generation,
            activation_identity_sha256: request.activation_identity_sha256.clone(),
            target_receipt_sha256: request.target_receipt_sha256.clone(),
            target_cloud_revision: request.target_cloud_revision,
            target_cloud_sha256: request.target_cloud_sha256.clone(),
            asset_manifest_sha256: request.asset_manifest_sha256.clone(),
            asset_bundle_sha256: request.asset_bundle_sha256.clone(),
            source_policy_version: request.source_policy_version,
            source_policy_sha256: request.source_policy_sha256.clone(),
            deadline_policy_version: request.deadline_policy_version,
            deadline_policy_sha256: request.deadline_policy_sha256.clone(),
            logical_timeout_ms: request.logical_timeout_ms,
            algorithm_options_version: request.algorithm_options_version,
            algorithm_options_sha256: request.algorithm_options_sha256.clone(),
            disposition: disposition as i32,
            candidate_sequence,
            candidate_payload_sha256: candidate_payload_sha256
                .map(|digest| digest.to_vec())
                .unwrap_or_default(),
            evidence_sha256: evidence_sha256.to_vec(),
            correction_revision,
            terminal_proof_sha256: Vec::new(),
            detail,
            candidate_payload_preimage_v1,
            evidence_disclosure_v1,
        };
        let proof = canonical_relocalization_terminal_proof_preimage_v1(&result)
            .expect("new relocalization result has a clear terminal proof field");
        result.terminal_proof_sha256 = sha256_bytes(&proof).to_vec();
        result
    }

    fn snapshot_matches_relocalize_request(
        snapshot: &SealedActivatedTargetSnapshot,
        request: &ValidatedRelocalizeRequest,
        map_frame_id: &str,
    ) -> Result<(), (crate::proto::RelocalizeDisposition, &'static str)> {
        use crate::proto::RelocalizeDisposition as Disposition;
        if request.request.runtime_generation != snapshot.runtime_generation {
            return Err((Disposition::StaleSession, "RELOCALIZATION_RUNTIME_STALE"));
        }
        if request.request.session_generation < snapshot.session_generation {
            return Err((Disposition::StaleSession, "RELOCALIZATION_SESSION_STALE"));
        }
        if request.request.session_id != snapshot.session_id
            || request.request.map_name != snapshot.map_name
            || request.request.session_generation != snapshot.session_generation
        {
            return Err((
                Disposition::SessionInvalidated,
                "RELOCALIZATION_SESSION_IDENTITY_MISMATCH",
            ));
        }
        if request.request.backend_generation != snapshot.backend_generation
            || request.activation_identity_sha256 != snapshot.activation_identity_sha256
            || request.target_receipt_sha256 != snapshot.target_receipt_sha256
            || request.request.target_cloud_revision != snapshot.immutable_target.source_revision
            || request.target_cloud_sha256 != snapshot.immutable_target.xyz_sha256
            || Some(request.asset_manifest_sha256) != snapshot.asset_manifest_sha256
            || Some(request.asset_bundle_sha256) != snapshot.asset_bundle_sha256
            || snapshot.immutable_target.frame_id != map_frame_id
        {
            return Err((
                Disposition::TargetInvalidated,
                "RELOCALIZATION_TARGET_IDENTITY_MISMATCH",
            ));
        }
        Ok(())
    }

    fn same_sealed_target(
        left: &SealedActivatedTargetSnapshot,
        right: &SealedActivatedTargetSnapshot,
    ) -> bool {
        left.runtime_generation == right.runtime_generation
            && left.lifecycle_epoch == right.lifecycle_epoch
            && left.session_id == right.session_id
            && left.map_name == right.map_name
            && left.activation_request_id == right.activation_request_id
            && left.activation_request_identity_sha256 == right.activation_request_identity_sha256
            && left.activation_identity_sha256 == right.activation_identity_sha256
            && left.target_receipt_sha256 == right.target_receipt_sha256
            && left.asset_manifest_sha256 == right.asset_manifest_sha256
            && left.asset_bundle_sha256 == right.asset_bundle_sha256
            && left.session_generation == right.session_generation
            && left.backend_generation == right.backend_generation
            && left.publication_proof.source_timestamp_ns
                == right.publication_proof.source_timestamp_ns
            && left.publication_proof.cloud_revision == right.publication_proof.cloud_revision
            && left.publication_proof.point_count == right.publication_proof.point_count
            && left.publication_proof.voxel_size.to_bits()
                == right.publication_proof.voxel_size.to_bits()
            && left.publication_proof.cloud_receipt == right.publication_proof.cloud_receipt
            && left.publication_proof.target_receipt_sha256
                == right.publication_proof.target_receipt_sha256
            && left.immutable_target == right.immutable_target
    }

    #[derive(Clone, Copy, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
    struct RelocalizationFrameKey {
        mapping_commit_boot_nonce: u128,
        mapping_commit_sequence: u64,
        occupancy_revision: u64,
        cloud_revision: u64,
        registered_cloud_source_timestamp_ns: u64,
        registered_cloud_source_sequence: u64,
        correlated_odom_source_sequence: u64,
    }

    #[derive(Clone, Debug, Hash, PartialEq, Eq)]
    struct RelocalizationFrameScope {
        mapping_session_id: String,
        mapping_session_generation: u64,
        map_frame_generation: u64,
    }

    impl RelocalizationFrameScope {
        fn from_input(input: &CorrectionFrameInput) -> Self {
            Self {
                mapping_session_id: input.mapping_session_id.clone(),
                mapping_session_generation: input.mapping_session_generation,
                map_frame_generation: input.map_frame_generation,
            }
        }
    }

    fn relocalization_frame_key(input: &CorrectionFrameInput) -> RelocalizationFrameKey {
        RelocalizationFrameKey {
            mapping_commit_boot_nonce: input.mapping_commit_id.boot_nonce,
            mapping_commit_sequence: input.mapping_commit_id.sequence,
            occupancy_revision: input.occupancy_revision,
            cloud_revision: input.cloud_revision,
            registered_cloud_source_timestamp_ns: input.registered_cloud.source_timestamp_ns,
            registered_cloud_source_sequence: input.registered_cloud.source_sequence,
            correlated_odom_source_sequence: input.correlated_odom.proof.source_sequence,
        }
    }

    fn push_isometry(out: &mut Vec<u8>, transform: &Isometry3<f64>) {
        let transform = CorrectionTransform::from_isometry(transform);
        push_correction_transform(out, &transform);
    }

    fn push_correction_transform(out: &mut Vec<u8>, transform: &CorrectionTransform) {
        for value in transform.translation_m {
            out.extend_from_slice(&value.to_be_bytes());
        }
        for value in transform.rotation_xyzw {
            out.extend_from_slice(&value.to_be_bytes());
        }
    }

    fn push_relocalization_frame_key(out: &mut Vec<u8>, key: RelocalizationFrameKey) {
        out.extend_from_slice(&key.mapping_commit_boot_nonce.to_be_bytes());
        out.extend_from_slice(&key.mapping_commit_sequence.to_be_bytes());
        out.extend_from_slice(&key.occupancy_revision.to_be_bytes());
        out.extend_from_slice(&key.cloud_revision.to_be_bytes());
        out.extend_from_slice(&key.registered_cloud_source_timestamp_ns.to_be_bytes());
        out.extend_from_slice(&key.registered_cloud_source_sequence.to_be_bytes());
        out.extend_from_slice(&key.correlated_odom_source_sequence.to_be_bytes());
    }

    fn push_optional_relocalization_frame_key(
        out: &mut Vec<u8>,
        key: Option<RelocalizationFrameKey>,
    ) {
        match key {
            Some(key) => {
                out.push(1);
                push_relocalization_frame_key(out, key);
            }
            None => out.push(0),
        }
    }

    #[cfg_attr(test, allow(dead_code))]
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[repr(u32)]
    enum RelocalizationCutoffReasonV1 {
        MinimumAggregateReached = 1,
        MaximumFramesReached = 2,
        MaximumAggregateReached = 3,
        AccumulationTimeout = 4,
        LogicalDeadline = 5,
        LastWaiterCancelled = 6,
        PhaseAStopped = 7,
        SessionInvalidated = 8,
        TargetInvalidated = 9,
        FrameIntegrityFailure = 10,
        HardBoundExceeded = 11,
        AlgorithmRejected = 12,
        Finalized = 13,
        OwnerFailed = 14,
    }

    struct RelocalizationTargetEvidenceV1<'a> {
        snapshot: &'a SealedActivatedTargetSnapshot,
        decoded_point_count: u64,
        decoded_f64_bytes: u64,
        decoded_sha256: [u8; 32],
    }

    struct RelocalizationSourceEvidenceV1<'a> {
        arm_watermark: Option<RelocalizationFrameKey>,
        cutoff_reason: RelocalizationCutoffReasonV1,
        last_accepted_key: Option<RelocalizationFrameKey>,
        frames: &'a [AccumulatedRelocalizationFrame],
        aggregate_input_points: u64,
        aggregate_input_f64_bytes: u64,
        post_transform_points: u64,
        post_transform_f64_bytes: u64,
        emitted_points: u64,
        canonical_source_bytes: &'a [u8],
        source_sha256: [u8; 32],
        boundary_key: RelocalizationFrameKey,
        boundary_base_anchor_map_from_odom: &'a Isometry3<f64>,
    }

    struct RelocalizationAlgorithmEvidenceV1<'a> {
        result: &'a RelocalizationResult,
        accepted: bool,
        correction: Option<&'a Isometry3<f64>>,
    }

    struct RelocalizationCandidateEvidenceV1<'a> {
        candidate: &'a CorrectionCandidate,
    }

    struct RelocalizationEvidenceV1<'a> {
        request_identity_sha256: [u8; 32],
        admission_ordinal: Option<u64>,
        normalized_logical_timeout_ms: Option<u64>,
        runtime_generation: u64,
        activated_mapping_backend_generation: u64,
        correction_owner_backend_generation: u64,
        session_id: &'a str,
        map_name: &'a str,
        session_generation: u64,
        map_frame_id: &'a str,
        odom_frame_id: &'a str,
        body_frame_id: &'a str,
        activation_identity_sha256: [u8; 32],
        target_receipt_sha256: [u8; 32],
        asset_manifest_sha256: [u8; 32],
        asset_bundle_sha256: [u8; 32],
        source_policy_version: u32,
        source_policy_sha256: [u8; 32],
        deadline_policy_version: u32,
        deadline_policy_sha256: [u8; 32],
        algorithm_options_version: u32,
        algorithm_options_sha256: [u8; 32],
        target: Option<RelocalizationTargetEvidenceV1<'a>>,
        source: Option<RelocalizationSourceEvidenceV1<'a>>,
        algorithm: Option<RelocalizationAlgorithmEvidenceV1<'a>>,
        candidate: Option<RelocalizationCandidateEvidenceV1<'a>>,
        terminal_disposition: crate::proto::RelocalizeDisposition,
        correction_revision: u64,
        detail: &'a str,
    }

    fn relocalization_candidate_payload_preimage_v1(
        candidate: Option<&CorrectionCandidate>,
    ) -> Result<Vec<u8>, String> {
        let Some(candidate) = candidate else {
            return Ok(Vec::new());
        };
        let preimage = candidate
            .encode_body_prefix()
            .map_err(|reason| reason.to_string())?;
        if preimage.len() > RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES
            || sha256_bytes(&preimage) != candidate.payload_sha256
        {
            return Err("RELOCALIZATION_CANDIDATE_DISCLOSURE_INVALID".to_owned());
        }
        Ok(preimage)
    }

    fn relocalization_source_commitment_record_v1(source: &[u8]) -> Result<Vec<u8>, String> {
        let mut out = RELOCALIZATION_SOURCE_COMMITMENT_DOMAIN_V1.to_vec();
        out.extend_from_slice(&RELOCALIZATION_SCHEMA_VERSION_V1.to_be_bytes());
        out.extend_from_slice(
            &u64::try_from(source.len())
                .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                .to_be_bytes(),
        );
        out.extend_from_slice(&sha256_bytes(source));
        Ok(out)
    }

    fn build_relocalization_evidence_disclosure_v1(
        preimage: &[u8],
        canonical_source_range: Option<std::ops::Range<usize>>,
    ) -> Result<Vec<u8>, String> {
        let mut out = RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1.to_vec();
        out.extend_from_slice(&RELOCALIZATION_SCHEMA_VERSION_V1.to_be_bytes());
        match canonical_source_range {
            None => {
                if preimage.len() > RELOCALIZATION_MAX_INLINE_EVIDENCE_PREIMAGE_BYTES {
                    return Err("RELOCALIZATION_INLINE_EVIDENCE_DISCLOSURE_TOO_LARGE".to_owned());
                }
                out.push(0);
                out.extend_from_slice(
                    &u64::try_from(preimage.len())
                        .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                        .to_be_bytes(),
                );
                out.extend_from_slice(preimage);
            }
            Some(source_range) => {
                if source_range.start > source_range.end || source_range.end > preimage.len() {
                    return Err("RELOCALIZATION_EVIDENCE_SOURCE_RANGE_INVALID".to_owned());
                }
                let prefix = &preimage[..source_range.start];
                let source = &preimage[source_range.clone()];
                let suffix = &preimage[source_range.end..];
                if prefix.len() > RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES
                    || source.is_empty()
                    || source.len() > RELOCALIZATION_MAX_CANONICAL_SOURCE_BYTES
                    || suffix.len() > RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES
                {
                    return Err("RELOCALIZATION_EVIDENCE_DISCLOSURE_BOUND_EXCEEDED".to_owned());
                }
                let source_sha256 = sha256_bytes(source);
                let source_commitment = relocalization_source_commitment_record_v1(source)?;
                if !prefix.ends_with(&source_commitment) {
                    return Err("RELOCALIZATION_EVIDENCE_SOURCE_COMMITMENT_UNBOUND".to_owned());
                }

                out.push(1);
                for length in [preimage.len(), prefix.len(), source.len(), suffix.len()] {
                    out.extend_from_slice(
                        &u64::try_from(length)
                            .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                            .to_be_bytes(),
                    );
                }
                out.extend_from_slice(prefix);
                out.extend_from_slice(&source_sha256);
                out.extend_from_slice(suffix);
            }
        }
        if out.len() > RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES {
            return Err("RELOCALIZATION_EVIDENCE_DISCLOSURE_TOO_LARGE".to_owned());
        }
        Ok(out)
    }

    struct CanonicalRelocalizationEvidencePreimageV1 {
        bytes: Vec<u8>,
        canonical_source_range: Option<std::ops::Range<usize>>,
    }

    fn canonical_relocalization_evidence_sha256_v1(
        preimage: &CanonicalRelocalizationEvidencePreimageV1,
    ) -> Result<[u8; 32], String> {
        let Some(source_range) = preimage.canonical_source_range.clone() else {
            return Ok(sha256_bytes(&preimage.bytes));
        };
        if source_range.start > source_range.end || source_range.end > preimage.bytes.len() {
            return Err("RELOCALIZATION_EVIDENCE_SOURCE_RANGE_INVALID".to_owned());
        }
        let prefix = &preimage.bytes[..source_range.start];
        let source = &preimage.bytes[source_range.clone()];
        let suffix = &preimage.bytes[source_range.end..];
        let full_length = prefix
            .len()
            .checked_add(source.len())
            .and_then(|value| value.checked_add(suffix.len()))
            .ok_or("RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?;
        let source_commitment = relocalization_source_commitment_record_v1(source)?;
        if !prefix.ends_with(&source_commitment) {
            return Err("RELOCALIZATION_EVIDENCE_SOURCE_COMMITMENT_UNBOUND".to_owned());
        }

        let mut composable = RELOCALIZATION_EVIDENCE_COMPOSITE_DOMAIN_V1.to_vec();
        composable.extend_from_slice(&RELOCALIZATION_SCHEMA_VERSION_V1.to_be_bytes());
        composable.extend_from_slice(&0_u32.to_be_bytes());
        composable.extend_from_slice(
            &u64::try_from(full_length)
                .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                .to_be_bytes(),
        );
        composable.extend_from_slice(&1_u32.to_be_bytes());
        composable.extend_from_slice(
            &u64::try_from(prefix.len())
                .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                .to_be_bytes(),
        );
        composable.extend_from_slice(prefix);
        composable.extend_from_slice(&2_u32.to_be_bytes());
        composable.extend_from_slice(
            &u64::try_from(source.len())
                .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                .to_be_bytes(),
        );
        composable.extend_from_slice(&sha256_bytes(source));
        composable.extend_from_slice(&3_u32.to_be_bytes());
        composable.extend_from_slice(
            &u64::try_from(suffix.len())
                .map_err(|_| "RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?
                .to_be_bytes(),
        );
        composable.extend_from_slice(suffix);
        Ok(sha256_bytes(&composable))
    }

    // v1 数值统一大端；字符串为 u32 长度加 UTF-8；字节串为 u64 长度加原字节；
    // 可选段以单字节 0/1 标记；f32/f64 使用原始 IEEE-754 位。顶层字段标签固定为
    // 1..28，嵌套段按编码语句顺序固定，列表先编码 u32 元素数。
    fn canonical_relocalization_evidence_preimage_v1(
        evidence: &RelocalizationEvidenceV1<'_>,
    ) -> Result<Vec<u8>, String> {
        canonical_relocalization_evidence_preimage_parts_v1(evidence).map(|preimage| preimage.bytes)
    }

    fn canonical_relocalization_evidence_preimage_parts_v1(
        evidence: &RelocalizationEvidenceV1<'_>,
    ) -> Result<CanonicalRelocalizationEvidencePreimageV1, String> {
        let mut out = RELOCALIZATION_EVIDENCE_DOMAIN_V1.to_vec();
        let mut canonical_source_range = None;
        out.extend_from_slice(&RELOCALIZATION_SCHEMA_VERSION_V1.to_be_bytes());

        macro_rules! tag {
            ($value:expr) => {
                out.extend_from_slice(&($value).to_be_bytes())
            };
        }

        tag!(1_u32);
        out.extend_from_slice(&evidence.request_identity_sha256);
        tag!(2_u32);
        match evidence.admission_ordinal {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value.to_be_bytes());
            }
            None => out.push(0),
        }
        tag!(3_u32);
        match evidence.normalized_logical_timeout_ms {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value.to_be_bytes());
            }
            None => out.push(0),
        }
        tag!(4_u32);
        out.extend_from_slice(&evidence.runtime_generation.to_be_bytes());
        tag!(5_u32);
        out.extend_from_slice(&evidence.activated_mapping_backend_generation.to_be_bytes());
        tag!(6_u32);
        out.extend_from_slice(&evidence.correction_owner_backend_generation.to_be_bytes());
        tag!(7_u32);
        canonical_push_string(&mut out, evidence.session_id);
        tag!(8_u32);
        canonical_push_string(&mut out, evidence.map_name);
        tag!(9_u32);
        out.extend_from_slice(&evidence.session_generation.to_be_bytes());
        tag!(10_u32);
        canonical_push_string(&mut out, evidence.map_frame_id);
        tag!(11_u32);
        canonical_push_string(&mut out, evidence.odom_frame_id);
        tag!(12_u32);
        canonical_push_string(&mut out, evidence.body_frame_id);
        tag!(13_u32);
        out.extend_from_slice(&evidence.activation_identity_sha256);
        tag!(14_u32);
        out.extend_from_slice(&evidence.target_receipt_sha256);
        tag!(15_u32);
        out.extend_from_slice(&evidence.asset_manifest_sha256);
        tag!(16_u32);
        out.extend_from_slice(&evidence.asset_bundle_sha256);
        tag!(17_u32);
        out.extend_from_slice(&evidence.source_policy_version.to_be_bytes());
        out.extend_from_slice(&evidence.source_policy_sha256);
        tag!(18_u32);
        out.extend_from_slice(&evidence.deadline_policy_version.to_be_bytes());
        out.extend_from_slice(&evidence.deadline_policy_sha256);
        tag!(19_u32);
        out.extend_from_slice(&evidence.algorithm_options_version.to_be_bytes());
        out.extend_from_slice(&evidence.algorithm_options_sha256);

        tag!(20_u32);
        match evidence.target.as_ref() {
            Some(target) => {
                out.push(1);
                let snapshot = target.snapshot;
                out.extend_from_slice(b"target.v1\0");
                out.extend_from_slice(&snapshot.runtime_generation.to_be_bytes());
                out.extend_from_slice(&snapshot.lifecycle_epoch.to_be_bytes());
                canonical_push_string(&mut out, &snapshot.session_id);
                canonical_push_string(&mut out, &snapshot.map_name);
                out.extend_from_slice(&snapshot.session_generation.to_be_bytes());
                out.extend_from_slice(&snapshot.backend_generation.to_be_bytes());
                canonical_push_string(&mut out, &snapshot.activation_request_id);
                out.extend_from_slice(&snapshot.activation_request_identity_sha256);
                out.extend_from_slice(&snapshot.activation_identity_sha256);
                out.extend_from_slice(&snapshot.target_receipt_sha256);
                canonical_push_optional_digest(&mut out, snapshot.asset_manifest_sha256);
                canonical_push_optional_digest(&mut out, snapshot.asset_bundle_sha256);
                canonical_push_string(&mut out, &snapshot.immutable_target.frame_id);
                out.extend_from_slice(&snapshot.immutable_target.source_revision.to_be_bytes());
                out.extend_from_slice(&snapshot.immutable_target.point_count.to_be_bytes());
                out.extend_from_slice(
                    &snapshot.immutable_target.voxel_size.to_bits().to_be_bytes(),
                );
                out.extend_from_slice(
                    &snapshot
                        .immutable_target
                        .payload_schema_version
                        .to_be_bytes(),
                );
                out.extend_from_slice(
                    &u64::try_from(snapshot.immutable_target.xyz_f32_le.len())
                        .map_err(|_| "RELOCALIZATION_TARGET_LENGTH_OVERFLOW")?
                        .to_be_bytes(),
                );
                out.extend_from_slice(&snapshot.immutable_target.xyz_sha256);
                out.extend_from_slice(&target.decoded_point_count.to_be_bytes());
                out.extend_from_slice(&target.decoded_f64_bytes.to_be_bytes());
                out.extend_from_slice(&target.decoded_sha256);
                let publication = &snapshot.publication_proof;
                out.extend_from_slice(&publication.source_timestamp_ns.to_be_bytes());
                out.extend_from_slice(&publication.cloud_revision.to_be_bytes());
                out.extend_from_slice(&publication.point_count.to_be_bytes());
                out.extend_from_slice(&publication.voxel_size.to_bits().to_be_bytes());
                let receipt = &publication.cloud_receipt;
                out.extend_from_slice(&receipt.commit_id.boot_nonce.to_be_bytes());
                out.extend_from_slice(&receipt.commit_id.sequence.to_be_bytes());
                out.extend_from_slice(&receipt.session_generation.to_be_bytes());
                out.extend_from_slice(&receipt.occupancy_revision.to_be_bytes());
                out.extend_from_slice(&receipt.cloud_revision.to_be_bytes());
                out.extend_from_slice(&receipt.source_timestamp_ns.to_be_bytes());
                out.extend_from_slice(&receipt.point_count.to_be_bytes());
                out.extend_from_slice(&publication.target_receipt_sha256);
            }
            None => out.push(0),
        }

        tag!(21_u32);
        match evidence.source.as_ref() {
            Some(source) => {
                out.push(1);
                out.extend_from_slice(b"source.v1\0");
                push_optional_relocalization_frame_key(&mut out, source.arm_watermark);
                out.extend_from_slice(&(source.cutoff_reason as u32).to_be_bytes());
                push_optional_relocalization_frame_key(&mut out, source.last_accepted_key);
                out.extend_from_slice(
                    &u32::try_from(source.frames.len())
                        .map_err(|_| "RELOCALIZATION_FRAME_COUNT_OVERFLOW")?
                        .to_be_bytes(),
                );
                for frame in source.frames {
                    out.extend_from_slice(b"frame.v1\0");
                    let input = &frame.input;
                    push_relocalization_frame_key(&mut out, relocalization_frame_key(input));
                    canonical_push_string(&mut out, &input.mapping_session_id);
                    out.extend_from_slice(&input.mapping_session_generation.to_be_bytes());
                    out.extend_from_slice(&input.map_frame_generation.to_be_bytes());
                    out.extend_from_slice(&input.registered_cloud.point_count.to_be_bytes());
                    out.extend_from_slice(&input.registered_cloud.envelope_sha256);
                    out.extend_from_slice(&input.registered_cloud.body_points_sha256);
                    out.extend_from_slice(
                        &input
                            .correlated_odom
                            .proof
                            .source_timestamp_ns
                            .to_be_bytes(),
                    );
                    out.extend_from_slice(&input.correlated_odom.proof.pose_sha256);
                    out.extend_from_slice(&input.committed_map.source_timestamp_ns.to_be_bytes());
                    out.extend_from_slice(&input.committed_map.width.to_be_bytes());
                    out.extend_from_slice(&input.committed_map.height.to_be_bytes());
                    out.extend_from_slice(
                        &input.committed_map.resolution_m.to_bits().to_be_bytes(),
                    );
                    out.extend_from_slice(&input.committed_map.origin_x_m.to_bits().to_be_bytes());
                    out.extend_from_slice(&input.committed_map.origin_y_m.to_bits().to_be_bytes());
                    out.extend_from_slice(&input.committed_map.payload_sha256);
                    out.extend_from_slice(&input.committed_map.boundary_sha256);
                    canonical_push_string(&mut out, &input.points_frame_id);
                    push_isometry(&mut out, &input.correlated_odom.odom_from_body);
                    push_isometry(&mut out, &input.base_anchor_map_from_odom);
                    out.extend_from_slice(&frame.evidence_sha256);
                }
                out.extend_from_slice(&source.aggregate_input_points.to_be_bytes());
                out.extend_from_slice(&source.aggregate_input_f64_bytes.to_be_bytes());
                out.extend_from_slice(&source.post_transform_points.to_be_bytes());
                out.extend_from_slice(&source.post_transform_f64_bytes.to_be_bytes());
                out.extend_from_slice(&source.emitted_points.to_be_bytes());
                if sha256_bytes(source.canonical_source_bytes) != source.source_sha256 {
                    return Err("RELOCALIZATION_EVIDENCE_SOURCE_DIGEST_MISMATCH".to_owned());
                }
                out.extend_from_slice(&relocalization_source_commitment_record_v1(
                    source.canonical_source_bytes,
                )?);
                let source_start = out.len();
                out.extend_from_slice(source.canonical_source_bytes);
                let source_end = source_start
                    .checked_add(source.canonical_source_bytes.len())
                    .ok_or("RELOCALIZATION_EVIDENCE_LENGTH_OVERFLOW")?;
                canonical_source_range = Some(source_start..source_end);
                push_relocalization_frame_key(&mut out, source.boundary_key);
                push_isometry(&mut out, source.boundary_base_anchor_map_from_odom);
            }
            None => out.push(0),
        }

        tag!(22_u32);
        match evidence.algorithm.as_ref() {
            Some(algorithm) => {
                out.push(1);
                out.extend_from_slice(b"algorithm.v1\0");
                out.push(u8::from(algorithm.result.registration.converged));
                out.extend_from_slice(&algorithm.result.registration.mse.to_bits().to_be_bytes());
                out.extend_from_slice(
                    &u64::try_from(algorithm.result.registration.iterations)
                        .map_err(|_| "RELOCALIZATION_ITERATION_COUNT_OVERFLOW")?
                        .to_be_bytes(),
                );
                out.extend_from_slice(
                    &u64::try_from(algorithm.result.feature_matches)
                        .map_err(|_| "RELOCALIZATION_FEATURE_COUNT_OVERFLOW")?
                        .to_be_bytes(),
                );
                out.extend_from_slice(
                    &u64::try_from(algorithm.result.ransac_inliers)
                        .map_err(|_| "RELOCALIZATION_INLIER_COUNT_OVERFLOW")?
                        .to_be_bytes(),
                );
                out.extend_from_slice(&algorithm.result.fitness.to_bits().to_be_bytes());
                push_isometry(&mut out, &algorithm.result.registration.transformation);
                out.push(u8::from(algorithm.accepted));
                match algorithm.correction {
                    Some(correction) => {
                        out.push(1);
                        push_isometry(&mut out, correction);
                    }
                    None => out.push(0),
                }
            }
            None => out.push(0),
        }

        tag!(23_u32);
        match evidence.candidate.as_ref() {
            Some(candidate) => {
                out.push(1);
                out.extend_from_slice(b"candidate.v1\0");
                let candidate = candidate.candidate;
                out.extend_from_slice(&candidate.identity.backend.wire_code().to_be_bytes());
                out.extend_from_slice(
                    &candidate
                        .identity
                        .transform_semantics
                        .wire_code()
                        .to_be_bytes(),
                );
                out.extend_from_slice(&candidate.identity.runtime_generation.to_be_bytes());
                out.extend_from_slice(&candidate.identity.backend_generation.to_be_bytes());
                out.extend_from_slice(&candidate.identity.candidate_sequence.to_be_bytes());
                canonical_push_string(&mut out, &candidate.identity.mapping_session_id);
                out.extend_from_slice(&candidate.identity.mapping_session_generation.to_be_bytes());
                out.extend_from_slice(&candidate.identity.map_frame_generation.to_be_bytes());
                push_relocalization_frame_key(
                    &mut out,
                    RelocalizationFrameKey {
                        mapping_commit_boot_nonce: candidate.identity.mapping_commit_id.boot_nonce,
                        mapping_commit_sequence: candidate.identity.mapping_commit_id.sequence,
                        occupancy_revision: candidate.identity.occupancy_revision,
                        cloud_revision: candidate.identity.cloud_revision,
                        registered_cloud_source_timestamp_ns: candidate
                            .identity
                            .registered_cloud
                            .source_timestamp_ns,
                        registered_cloud_source_sequence: candidate
                            .identity
                            .registered_cloud
                            .source_sequence,
                        correlated_odom_source_sequence: candidate
                            .identity
                            .correlated_odom
                            .source_sequence,
                    },
                );
                out.extend_from_slice(
                    &candidate.identity.correction_lease.boot_nonce.to_be_bytes(),
                );
                out.extend_from_slice(
                    &candidate.identity.correction_lease.generation.to_be_bytes(),
                );
                push_correction_transform(&mut out, &candidate.transform);
                out.extend_from_slice(&candidate.payload_sha256);
            }
            None => out.push(0),
        }
        tag!(24_u32);
        out.extend_from_slice(&(evidence.terminal_disposition as i32).to_be_bytes());
        tag!(25_u32);
        out.extend_from_slice(&evidence.correction_revision.to_be_bytes());
        tag!(26_u32);
        canonical_push_string(&mut out, evidence.detail);
        tag!(27_u32);
        out.extend_from_slice(b"terminal-proof-follows\0");
        tag!(28_u32);
        out.push(0);
        Ok(CanonicalRelocalizationEvidencePreimageV1 {
            bytes: out,
            canonical_source_range,
        })
    }

    fn relocalization_evidence_sha256_v1(
        evidence: &RelocalizationEvidenceV1<'_>,
    ) -> Result<[u8; 32], String> {
        let preimage = canonical_relocalization_evidence_preimage_parts_v1(evidence)?;
        canonical_relocalization_evidence_sha256_v1(&preimage)
    }

    fn relocalization_frame_evidence(
        owner: &CorrectionOwnerRuntimeIdentity,
        map_frame_id: &str,
        odom_frame_id: &str,
        body_frame_id: &str,
        input: &CorrectionFrameInput,
    ) -> Result<(RelocalizationFrameKey, [u8; 32]), String> {
        staged_correction_frame(
            CorrectionCandidateBackend::Relocalization,
            owner,
            map_frame_id,
            odom_frame_id,
            body_frame_id,
            input,
        )?;
        let body_bytes = input
            .body_points
            .len()
            .checked_mul(3)
            .and_then(|count| count.checked_mul(std::mem::size_of::<f64>()))
            .ok_or_else(|| "RELOCALIZATION_FRAME_BODY_SIZE_OVERFLOW".to_owned())?;
        if body_bytes > RELOCALIZATION_MAX_PER_FRAME_F64_BYTES {
            return Err("RELOCALIZATION_FRAME_BODY_SIZE_EXCEEDED".to_owned());
        }
        let envelope_bound = input
            .body_points
            .len()
            .checked_mul(crate::pointcloud::POINTCLOUD_MAX_POINT_STEP_BYTES as usize)
            .ok_or_else(|| "RELOCALIZATION_ENVELOPE_SIZE_OVERFLOW".to_owned())?;
        if envelope_bound > RELOCALIZATION_MAX_ENVELOPE_DATA_BYTES {
            return Err("RELOCALIZATION_ENVELOPE_SIZE_EXCEEDED".to_owned());
        }
        let key = relocalization_frame_key(input);
        let mut proof = b"nuwax.geometry.relocalize.frame-evidence.v1\0".to_vec();
        canonical_push_string(&mut proof, &input.mapping_session_id);
        proof.extend_from_slice(&input.mapping_session_generation.to_be_bytes());
        proof.extend_from_slice(&input.map_frame_generation.to_be_bytes());
        proof.extend_from_slice(&key.mapping_commit_boot_nonce.to_be_bytes());
        proof.extend_from_slice(&key.mapping_commit_sequence.to_be_bytes());
        proof.extend_from_slice(&key.occupancy_revision.to_be_bytes());
        proof.extend_from_slice(&key.cloud_revision.to_be_bytes());
        proof.extend_from_slice(&key.registered_cloud_source_timestamp_ns.to_be_bytes());
        proof.extend_from_slice(&key.registered_cloud_source_sequence.to_be_bytes());
        proof.extend_from_slice(&key.correlated_odom_source_sequence.to_be_bytes());
        proof.extend_from_slice(&input.registered_cloud.point_count.to_be_bytes());
        proof.extend_from_slice(&input.registered_cloud.envelope_sha256);
        proof.extend_from_slice(&input.registered_cloud.body_points_sha256);
        proof.extend_from_slice(
            &input
                .correlated_odom
                .proof
                .source_timestamp_ns
                .to_be_bytes(),
        );
        proof.extend_from_slice(&input.correlated_odom.proof.pose_sha256);
        proof.extend_from_slice(&input.committed_map.source_timestamp_ns.to_be_bytes());
        proof.extend_from_slice(&input.committed_map.width.to_be_bytes());
        proof.extend_from_slice(&input.committed_map.height.to_be_bytes());
        proof.extend_from_slice(&input.committed_map.resolution_m.to_be_bytes());
        proof.extend_from_slice(&input.committed_map.origin_x_m.to_be_bytes());
        proof.extend_from_slice(&input.committed_map.origin_y_m.to_be_bytes());
        proof.extend_from_slice(&input.committed_map.payload_sha256);
        proof.extend_from_slice(&input.committed_map.boundary_sha256);
        canonical_push_string(&mut proof, &input.points_frame_id);
        push_isometry(&mut proof, &input.correlated_odom.odom_from_body);
        push_isometry(&mut proof, &input.base_anchor_map_from_odom);
        if proof.len() > RELOCALIZATION_MAX_PROOF_BYTES {
            return Err("RELOCALIZATION_FRAME_PROOF_SIZE_EXCEEDED".to_owned());
        }
        Ok((key, sha256_bytes(&proof)))
    }

    struct AccumulatedRelocalizationFrame {
        evidence_sha256: [u8; 32],
        input: CorrectionFrameInput,
    }

    struct SealedRelocalizationSource {
        boundary_input: CorrectionFrameInput,
        boundary_lease: Arc<CorrectionRequestLease>,
        snapshot: SealedActivatedTargetSnapshot,
        arm_watermark: Option<RelocalizationFrameKey>,
        cutoff_reason: RelocalizationCutoffReasonV1,
        frames: Vec<AccumulatedRelocalizationFrame>,
        aggregate_input_points: u64,
        aggregate_input_f64_bytes: u64,
        post_transform_points: u64,
        post_transform_f64_bytes: u64,
        source_points: Arc<[Point3<f64>]>,
        target_points: Arc<[Point3<f64>]>,
        canonical_source_bytes: Vec<u8>,
        source_sha256: [u8; 32],
        target_decoded_point_count: u64,
        target_decoded_f64_bytes: u64,
        target_decoded_sha256: [u8; 32],
    }

    fn seal_relocalization_source(
        mut accumulation: RelocalizationAccumulation,
        boundary_input: CorrectionFrameInput,
        boundary_lease: Arc<CorrectionRequestLease>,
        cutoff_reason: RelocalizationCutoffReasonV1,
    ) -> Result<SealedRelocalizationSource, String> {
        if !boundary_lease.has_lifecycle_guard() {
            return Err("RELOCALIZATION_BOUNDARY_LIFECYCLE_GUARD_MISSING".to_owned());
        }
        accumulation
            .accepted
            .sort_by_key(|frame| relocalization_frame_key(&frame.input));
        let boundary_key = relocalization_frame_key(&boundary_input);
        if accumulation
            .accepted
            .last()
            .map(|frame| relocalization_frame_key(&frame.input))
            != Some(boundary_key)
        {
            return Err("RELOCALIZATION_BOUNDARY_IS_NOT_MAXIMUM_FRAME".to_owned());
        }
        let aggregate_input_f64_bytes = accumulation
            .aggregate_input_points
            .checked_mul(3)
            .and_then(|count| count.checked_mul(std::mem::size_of::<f64>()))
            .ok_or_else(|| "RELOCALIZATION_AGGREGATE_BYTES_OVERFLOW".to_owned())?;
        if aggregate_input_f64_bytes > RELOCALIZATION_MAX_AGGREGATE_F64_BYTES {
            return Err("RELOCALIZATION_AGGREGATE_BYTES_EXCEEDED".to_owned());
        }
        let mut ordered = Vec::with_capacity(accumulation.aggregate_input_points);
        for frame in &accumulation.accepted {
            for body_point in &frame.input.body_points {
                let odom_point = frame.input.correlated_odom.odom_from_body * body_point;
                if !odom_point.x.is_finite()
                    || !odom_point.y.is_finite()
                    || !odom_point.z.is_finite()
                {
                    return Err("RELOCALIZATION_SOURCE_POINT_NONFINITE".to_owned());
                }
                ordered.push(odom_point);
            }
        }
        if ordered.len() != accumulation.aggregate_input_points {
            return Err("RELOCALIZATION_AGGREGATE_POINT_COUNT_MISMATCH".to_owned());
        }
        let post_transform_f64_bytes = ordered
            .len()
            .checked_mul(3)
            .and_then(|count| count.checked_mul(std::mem::size_of::<f64>()))
            .ok_or_else(|| "RELOCALIZATION_POST_TRANSFORM_BYTES_OVERFLOW".to_owned())?;
        if post_transform_f64_bytes > RELOCALIZATION_MAX_AGGREGATE_F64_BYTES {
            return Err("RELOCALIZATION_POST_TRANSFORM_BYTES_EXCEEDED".to_owned());
        }
        let post_transform_points = ordered.len();
        let (source_points, canonical_source_bytes, source_sha256) =
            stable_stratified_source(ordered)?;
        let (target_points, target_decoded_sha256) =
            decode_relocalization_target(&accumulation.snapshot.immutable_target)?;
        let target_decoded_f64_bytes = target_points
            .len()
            .checked_mul(3)
            .and_then(|count| count.checked_mul(std::mem::size_of::<f64>()))
            .ok_or_else(|| "RELOCALIZATION_TARGET_DECODED_BYTES_OVERFLOW".to_owned())?;
        let target_decoded_point_count =
            u64::from(accumulation.snapshot.immutable_target.point_count);
        Ok(SealedRelocalizationSource {
            boundary_input,
            boundary_lease,
            snapshot: accumulation.snapshot,
            arm_watermark: accumulation.arm_watermark,
            cutoff_reason,
            frames: accumulation.accepted,
            aggregate_input_points: u64::try_from(accumulation.aggregate_input_points)
                .map_err(|_| "RELOCALIZATION_AGGREGATE_POINT_COUNT_OVERFLOW".to_owned())?,
            aggregate_input_f64_bytes: u64::try_from(aggregate_input_f64_bytes)
                .map_err(|_| "RELOCALIZATION_AGGREGATE_BYTES_OVERFLOW".to_owned())?,
            post_transform_points: u64::try_from(post_transform_points)
                .map_err(|_| "RELOCALIZATION_POST_TRANSFORM_POINT_COUNT_OVERFLOW".to_owned())?,
            post_transform_f64_bytes: u64::try_from(post_transform_f64_bytes)
                .map_err(|_| "RELOCALIZATION_POST_TRANSFORM_BYTES_OVERFLOW".to_owned())?,
            source_points: Arc::from(source_points),
            target_points: Arc::from(target_points),
            canonical_source_bytes,
            source_sha256,
            target_decoded_point_count,
            target_decoded_f64_bytes: u64::try_from(target_decoded_f64_bytes)
                .map_err(|_| "RELOCALIZATION_TARGET_DECODED_BYTES_OVERFLOW".to_owned())?,
            target_decoded_sha256,
        })
    }

    fn decode_relocalization_target(
        target: &crate::pointcloud::ImmutableActivatedTarget,
    ) -> Result<(Vec<Point3<f64>>, [u8; 32]), String> {
        let point_count = usize::try_from(target.point_count)
            .map_err(|_| "RELOCALIZATION_TARGET_POINT_COUNT_OVERFLOW".to_owned())?;
        if point_count < 3
            || point_count.checked_mul(12) != Some(target.xyz_f32_le.len())
            || sha256_bytes(&target.xyz_f32_le) != target.xyz_sha256
        {
            return Err("RELOCALIZATION_TARGET_BYTES_INVALID".to_owned());
        }
        let mut points = Vec::with_capacity(point_count);
        let mut canonical = Vec::with_capacity(8 + point_count.saturating_mul(24));
        canonical.extend_from_slice(&(point_count as u64).to_be_bytes());
        for bytes in target.xyz_f32_le.chunks_exact(12) {
            let point = Point3::new(
                f64::from(f32::from_le_bytes(bytes[0..4].try_into().unwrap())),
                f64::from(f32::from_le_bytes(bytes[4..8].try_into().unwrap())),
                f64::from(f32::from_le_bytes(bytes[8..12].try_into().unwrap())),
            );
            if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
                return Err("RELOCALIZATION_TARGET_POINT_NONFINITE".to_owned());
            }
            canonical.extend_from_slice(&point.x.to_be_bytes());
            canonical.extend_from_slice(&point.y.to_be_bytes());
            canonical.extend_from_slice(&point.z.to_be_bytes());
            points.push(point);
        }
        Ok((points, sha256_bytes(&canonical)))
    }

    type StableRelocalizationSource = (Vec<Point3<f64>>, Vec<u8>, [u8; 32]);

    fn stable_stratified_source(
        ordered: Vec<Point3<f64>>,
    ) -> Result<StableRelocalizationSource, String> {
        if ordered.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS
            || ordered.len() > RELOCALIZATION_MAX_AGGREGATE_POINTS
        {
            return Err("RELOCALIZATION_SOURCE_POINT_COUNT_INVALID".to_owned());
        }
        let emitted_count = ordered.len().min(RELOCALIZATION_MAX_EMITTED_POINTS);
        let mut emitted = Vec::with_capacity(emitted_count);
        if ordered.len() <= emitted_count {
            emitted = ordered;
        } else {
            let count = ordered.len() as u128;
            let emitted_count_u128 = emitted_count as u128;
            for index in 0..emitted_count {
                let source_index = (index as u128)
                    .checked_mul(count)
                    .ok_or_else(|| "RELOCALIZATION_REDUCTION_INDEX_OVERFLOW".to_owned())?
                    / emitted_count_u128;
                emitted.push(ordered[source_index as usize]);
            }
        }
        let canonical_bytes_len = emitted
            .len()
            .checked_mul(24)
            .ok_or_else(|| "RELOCALIZATION_SOURCE_BYTES_OVERFLOW".to_owned())?;
        if !(1_200_000..=2_400_000).contains(&canonical_bytes_len) {
            return Err("RELOCALIZATION_EMITTED_SOURCE_BYTES_INVALID".to_owned());
        }
        let mut canonical = Vec::with_capacity(8 + canonical_bytes_len);
        canonical.extend_from_slice(&(emitted.len() as u64).to_be_bytes());
        for point in &emitted {
            canonical.extend_from_slice(&point.x.to_be_bytes());
            canonical.extend_from_slice(&point.y.to_be_bytes());
            canonical.extend_from_slice(&point.z.to_be_bytes());
        }
        let digest = sha256_bytes(&canonical);
        Ok((emitted, canonical, digest))
    }

    struct RelocalizationComputeOutput {
        result: RelocalizationResult,
        correction: Option<Isometry3<f64>>,
    }

    fn run_relocalization_compute(
        source: &[Point3<f64>],
        target: &[Point3<f64>],
        base_anchor_map_from_odom: Isometry3<f64>,
    ) -> RelocalizationComputeOutput {
        let result =
            relocation_pipeline_with_config(source, target, RelocalizationConfig::default());
        let accepted = result.registration.converged
            && result.fitness >= RelocalizationConfig::default().min_final_fitness
            && result.registration.mse.is_finite()
            && is_finite_isometry(&result.registration.transformation);
        let correction = accepted
            .then(|| base_anchor_map_from_odom.inverse() * result.registration.transformation);
        RelocalizationComputeOutput { result, correction }
    }

    type CorrectionWorkerJoin = Result<Result<(), CorrectionRuntimeError>, tokio::task::JoinError>;

    async fn wait_for_correction_future_until<F>(deadline: Instant, future: F) -> Option<F::Output>
    where
        F: std::future::Future,
    {
        let remaining = deadline.saturating_duration_since(Instant::now());
        tokio::time::timeout(remaining, future).await.ok()
    }

    async fn settle_retained_correction_worker(
        label: &'static str,
        worker: &mut tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>,
        deadline: Instant,
    ) -> Option<Result<(), CorrectionRuntimeError>> {
        let joined = if worker.is_finished() {
            Some((&mut *worker).await)
        } else {
            wait_for_correction_future_until(deadline, &mut *worker).await
        }?;
        Some(match joined {
            Ok(result) => result,
            Err(error) => Err(CorrectionRuntimeError::WorkerJoin(format!(
                "{label}: {error}"
            ))),
        })
    }

    #[derive(Debug, PartialEq, Eq)]
    pub(crate) struct CorrectionTransportShutdownSettled {
        sticky_error: Option<String>,
    }

    impl CorrectionTransportShutdownSettled {
        fn clean() -> Self {
            Self { sticky_error: None }
        }

        fn failed(detail: impl Into<String>) -> Self {
            Self {
                sticky_error: Some(detail.into()),
            }
        }

        pub(crate) fn into_sticky_error(self) -> Option<String> {
            self.sticky_error
        }
    }

    pub(crate) struct CorrectionTransportShutdownPending {
        detail: String,
        owner: Box<CorrectionTransportOwner>,
    }

    impl std::fmt::Debug for CorrectionTransportShutdownPending {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter
                .debug_struct("CorrectionTransportShutdownPending")
                .field("detail", &self.detail)
                .field("has_pending_owner", &true)
                .finish()
        }
    }

    impl std::fmt::Display for CorrectionTransportShutdownPending {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str(&self.detail)
        }
    }

    impl CorrectionTransportShutdownPending {
        fn new(detail: impl Into<String>, owner: CorrectionTransportOwner) -> Self {
            Self {
                detail: detail.into(),
                owner: Box::new(owner),
            }
        }

        pub(crate) fn into_owner(self) -> Box<CorrectionTransportOwner> {
            self.owner
        }
    }

    struct CorrectionShutdownState {
        phase_a_settled: bool,
        sink_stop_started: bool,
        sink_stop_worker: Option<tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
        sticky_failures: Vec<String>,
    }

    impl CorrectionShutdownState {
        fn new() -> Self {
            Self {
                phase_a_settled: false,
                sink_stop_started: false,
                sink_stop_worker: None,
                sticky_failures: Vec::new(),
            }
        }

        fn record(&mut self, error: impl Into<String>) {
            let error = error.into();
            if !self
                .sticky_failures
                .iter()
                .any(|existing| existing == &error)
            {
                self.sticky_failures.push(error);
            }
        }

        fn detail(&self) -> Option<String> {
            (!self.sticky_failures.is_empty()).then(|| self.sticky_failures.join("; "))
        }
    }

    async fn wait_optional_correction_worker(
        worker: Option<&mut tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
    ) -> CorrectionWorkerJoin {
        match worker {
            Some(worker) => worker.await,
            None => std::future::pending().await,
        }
    }

    /// Retained, generation-scoped real-LCM owner for correction production.
    ///
    /// Library candidates cross two private libLCM contexts. Subscription ACK
    /// completes before `start` returns. Shutdown closes admission first, then
    /// advances a bounded retained settlement attempt. A pending attempt returns
    /// this same owner with every unfinished worker and finalizer handle intact.
    pub(crate) struct CorrectionTransportOwner {
        ingress: CorrectionFrameIngress,
        phase_a: CorrectionPhaseAControl,
        subscriber_shutdown_tx: Option<watch::Sender<bool>>,
        subscriber_worker: Option<tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
        finalizer: CanonicalPublishFinalizerSupervisor,
        cleanup: CorrectionCleanupSupervisor,
        producer_worker: Option<tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>>,
        relocalization: Option<RelocalizationOwnerSupervisor>,
        binding: Option<(LcmTopic, LcmMessageType)>,
        independent_real_contexts: bool,
        #[cfg(test)]
        apply_waiters: Arc<CandidateApplyWaiters>,
        #[cfg(test)]
        relocalization_compute_delay_ms: Arc<AtomicU64>,
        #[cfg(test)]
        relocalization_compute_panic: Arc<AtomicBool>,
        sink: Arc<dyn CorrectionCandidateSink>,
        terminal: Option<Result<(), String>>,
        shutdown_state: CorrectionShutdownState,
    }

    impl CorrectionTransportOwner {
        pub(crate) async fn start(
            spec: CorrectionOwnerSpec,
            identity: CorrectionOwnerRuntimeIdentity,
            provider: &str,
            sink: Arc<dyn CorrectionCandidateSink>,
        ) -> Result<Self, CorrectionRuntimeError> {
            Self::start_with_relocalization_mode_fence(
                spec,
                identity,
                provider,
                sink,
                Arc::new(RelocalizationModeFence::default()),
            )
            .await
        }

        pub(crate) async fn start_with_relocalization_mode_fence(
            spec: CorrectionOwnerSpec,
            identity: CorrectionOwnerRuntimeIdentity,
            provider: &str,
            sink: Arc<dyn CorrectionCandidateSink>,
            relocalization_mode: Arc<RelocalizationModeFence>,
        ) -> Result<Self, CorrectionRuntimeError> {
            let relocalization_frames = match &spec {
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id,
                    odom_frame_id,
                    body_frame_id,
                } => Some((
                    map_frame_id.clone(),
                    odom_frame_id.clone(),
                    body_frame_id.clone(),
                )),
                _ => None,
            };
            let candidate_owner = if relocalization_frames.is_some() {
                None
            } else {
                Some(
                    CorrectionCandidateOwner::start(spec.clone(), identity.clone(), provider)
                        .await?,
                )
            };
            let (binding, publisher, subscription, independent_real_contexts) = match spec {
                CorrectionOwnerSpec::None => (None, None, None, false),
                CorrectionOwnerSpec::LibraryPgo { .. } => {
                    if provider.is_empty() || provider.trim() != provider || provider.contains('\0')
                    {
                        return Err(CorrectionRuntimeError::Transport(
                            "invalid private correction LCM provider".to_owned(),
                        ));
                    }
                    let binding = correction_candidate_binding(identity.runtime_generation)?;
                    let subscriber = LcmTransport::new_real(provider)
                        .map_err(|error| CorrectionRuntimeError::Transport(error.to_string()))?;
                    let subscription = subscriber
                        .subscribe_owned(binding.0.clone(), Some(binding.1.clone()))
                        .await
                        .map_err(|error| CorrectionRuntimeError::Transport(error.to_string()))?;
                    let publisher = match LcmTransport::new_real(provider) {
                        Ok(publisher) => publisher,
                        Err(error) => {
                            let unsubscribe_error = subscription.unsubscribe().await.err();
                            let detail = match unsubscribe_error {
                                Some(unsubscribe_error) => format!(
                                    "publisher context: {error}; rollback unsubscribe: {unsubscribe_error}"
                                ),
                                None => format!("publisher context: {error}"),
                            };
                            return Err(CorrectionRuntimeError::Transport(detail));
                        }
                    };
                    let publisher_context = publisher.shared_lcm_context().ok_or_else(|| {
                        CorrectionRuntimeError::Transport(
                            "publisher lacks a real LCM context".to_owned(),
                        )
                    })?;
                    let subscriber_context = subscriber.shared_lcm_context().ok_or_else(|| {
                        CorrectionRuntimeError::Transport(
                            "subscriber lacks a real LCM context".to_owned(),
                        )
                    })?;
                    if Arc::ptr_eq(&publisher_context, &subscriber_context) {
                        let unsubscribe_error = subscription.unsubscribe().await.err();
                        return Err(CorrectionRuntimeError::Transport(match unsubscribe_error {
                            Some(error) => format!(
                                "private correction contexts alias; rollback unsubscribe: {error}"
                            ),
                            None => "private correction contexts alias".to_owned(),
                        }));
                    }
                    (Some(binding), Some(publisher), Some(subscription), true)
                }
                CorrectionOwnerSpec::NativePgo { .. }
                | CorrectionOwnerSpec::Relocalization { .. } => (None, None, None, true),
                CorrectionOwnerSpec::Deferred { backend } => {
                    return Err(CorrectionRuntimeError::CandidateOwner(
                        CorrectionOwnerError::BackendDeferred(backend.stable_name()),
                    ));
                }
            };

            let admission_open = Arc::new(AtomicBool::new(true));
            let (admission_tx, admission_rx) = watch::channel(true);
            let (request_tx, request_rx) = mpsc::channel(CORRECTION_INPUT_QUEUE_CAPACITY);
            let apply_waiters = Arc::new(CandidateApplyWaiters::new(Arc::clone(&admission_open)));
            #[cfg(test)]
            let relocalization_compute_delay_ms = Arc::new(AtomicU64::new(0));
            #[cfg(test)]
            let relocalization_compute_panic = Arc::new(AtomicBool::new(false));
            let phase_a = CorrectionPhaseAControl::new_with_relocalization_mode_fence(
                Arc::clone(&admission_open),
                admission_tx.clone(),
                Arc::clone(&apply_waiters),
                identity.runtime_generation,
                relocalization_mode,
            );
            let finalizer = CanonicalPublishFinalizerSupervisor::start(
                Arc::clone(&sink),
                Arc::clone(&apply_waiters),
            );
            let cleanup = CorrectionCleanupSupervisor::start(Arc::clone(&sink));
            let finalizer_ingress = finalizer.ingress();
            let cleanup_ingress = cleanup.ingress();
            let ingress = CorrectionFrameIngress {
                sender: request_tx,
                admission_open: Arc::clone(&admission_open),
                admission_rx: admission_rx.clone(),
                relocalization_mode: phase_a.relocalization_mode(),
            };
            let (producer_worker, relocalization) = match relocalization_frames {
                Some((map_frame_id, odom_frame_id, body_frame_id)) => (
                    None,
                    Some(RelocalizationOwnerSupervisor::start_runtime(
                        request_rx,
                        phase_a.clone(),
                        identity.clone(),
                        map_frame_id,
                        odom_frame_id,
                        body_frame_id,
                        finalizer_ingress.clone(),
                        Arc::clone(&apply_waiters),
                        Arc::clone(&sink),
                        cleanup_ingress,
                        #[cfg(test)]
                        Arc::clone(&relocalization_compute_delay_ms),
                        #[cfg(test)]
                        Arc::clone(&relocalization_compute_panic),
                    )),
                ),
                None => {
                    let producer_binding = binding.clone();
                    let candidate_owner = candidate_owner.ok_or({
                        CorrectionRuntimeError::CandidateOwner(
                            CorrectionOwnerError::BackendDeferred("missing candidate owner"),
                        )
                    })?;
                    (
                        Some(tokio::spawn(run_candidate_producer(
                            candidate_owner,
                            request_rx,
                            admission_rx,
                            CandidateProducerContext {
                                publisher,
                                binding: producer_binding,
                                finalizer: finalizer_ingress.clone(),
                                apply_waiters: Arc::clone(&apply_waiters),
                                sink: Arc::clone(&sink),
                                cleanup: cleanup_ingress,
                                apply_ack_timeout: CORRECTION_APPLY_ACK_TIMEOUT,
                                phase_a: phase_a.clone(),
                                #[cfg(test)]
                                publish_gate: None,
                                #[cfg(test)]
                                finalizer_outcome_gate: None,
                            },
                        ))),
                        None,
                    )
                }
            };
            let (subscriber_shutdown_tx, subscriber_worker) = match subscription {
                Some(subscription) => {
                    let (shutdown_tx, shutdown_rx) = watch::channel(false);
                    let worker = tokio::spawn(run_candidate_subscriber(
                        subscription,
                        shutdown_rx,
                        Arc::clone(&apply_waiters),
                        finalizer_ingress,
                    ));
                    (Some(shutdown_tx), Some(worker))
                }
                None => (None, None),
            };
            Ok(Self {
                ingress,
                phase_a,
                subscriber_shutdown_tx,
                subscriber_worker,
                finalizer,
                cleanup,
                producer_worker,
                relocalization,
                binding,
                independent_real_contexts,
                #[cfg(test)]
                apply_waiters,
                #[cfg(test)]
                relocalization_compute_delay_ms,
                #[cfg(test)]
                relocalization_compute_panic,
                sink,
                terminal: None,
                shutdown_state: CorrectionShutdownState::new(),
            })
        }

        pub(crate) fn ingress(&self) -> CorrectionFrameIngress {
            self.ingress.clone()
        }

        pub(crate) fn phase_a_control(&self) -> CorrectionPhaseAControl {
            self.phase_a.clone()
        }

        pub(crate) fn relocalization_ingress(&self) -> Option<RelocalizationCommandIngress> {
            self.relocalization
                .as_ref()
                .map(RelocalizationOwnerSupervisor::ingress)
        }

        #[cfg(test)]
        fn set_relocalization_compute_delay_for_tests(&self, delay: Duration) {
            self.relocalization_compute_delay_ms.store(
                u64::try_from(delay.as_millis()).unwrap_or(u64::MAX),
                Ordering::Release,
            );
        }

        #[cfg(test)]
        fn set_relocalization_compute_panic_for_tests(&self, panic: bool) {
            self.relocalization_compute_panic
                .store(panic, Ordering::Release);
        }

        pub(crate) fn binding(&self) -> Option<(LcmTopic, LcmMessageType)> {
            self.binding.clone()
        }

        pub(crate) fn has_independent_real_contexts(&self) -> bool {
            self.independent_real_contexts
        }

        pub(crate) fn is_finished(&self) -> bool {
            self.terminal.is_some()
                || self
                    .producer_worker
                    .as_ref()
                    .is_some_and(tokio::task::JoinHandle::is_finished)
                || self
                    .subscriber_worker
                    .as_ref()
                    .is_some_and(tokio::task::JoinHandle::is_finished)
                || self
                    .relocalization
                    .as_ref()
                    .is_some_and(RelocalizationOwnerSupervisor::is_finished)
                || self.finalizer.is_finished()
                || self.cleanup.is_finished()
        }

        pub(crate) async fn wait_terminal(&mut self) -> Result<(), String> {
            if let Some(result) = &self.terminal {
                return result.clone();
            }
            enum WorkerTerminal {
                Producer(CorrectionWorkerJoin),
                Subscriber(CorrectionWorkerJoin),
                Finalizer(CorrectionWorkerJoin),
                Relocalization(CorrectionWorkerJoin),
                Cleanup(CorrectionWorkerJoin),
            }
            let producer = self.producer_worker.as_mut();
            let subscriber = self.subscriber_worker.as_mut();
            let finalizer = self.finalizer.worker.as_mut();
            let relocalization = self
                .relocalization
                .as_mut()
                .and_then(|owner| owner.worker.as_mut());
            let cleanup = self.cleanup.worker.as_mut();
            if producer.is_none()
                && subscriber.is_none()
                && finalizer.is_none()
                && relocalization.is_none()
                && cleanup.is_none()
            {
                let result = Err("correction owner has no retained worker".to_owned());
                self.terminal = Some(result.clone());
                return result;
            }
            let terminal = tokio::select! {
                result = wait_optional_correction_worker(producer) => WorkerTerminal::Producer(result),
                result = wait_optional_correction_worker(subscriber) => WorkerTerminal::Subscriber(result),
                result = wait_optional_correction_worker(finalizer) => WorkerTerminal::Finalizer(result),
                result = wait_optional_correction_worker(relocalization) => WorkerTerminal::Relocalization(result),
                result = wait_optional_correction_worker(cleanup) => WorkerTerminal::Cleanup(result),
            };
            let (label, result) = match terminal {
                WorkerTerminal::Producer(result) => {
                    self.producer_worker.take();
                    ("producer", result)
                }
                WorkerTerminal::Subscriber(result) => {
                    self.subscriber_worker.take();
                    ("subscriber", result)
                }
                WorkerTerminal::Finalizer(result) => {
                    self.finalizer.worker.take();
                    ("canonical finalizer", result)
                }
                WorkerTerminal::Relocalization(result) => {
                    if let Some(relocalization) = self.relocalization.as_mut() {
                        relocalization.worker.take();
                    }
                    ("relocalization command owner", result)
                }
                WorkerTerminal::Cleanup(result) => {
                    self.cleanup.worker.take();
                    ("correction cleanup", result)
                }
            };
            self.close_admission();
            let result = match result {
                Ok(Ok(()))
                    if matches!(label, "producer" | "relocalization command owner")
                        && self.phase_a.process_stop_requested() =>
                {
                    Ok(())
                }
                Ok(Ok(())) => Err(format!("correction {label} exited unexpectedly")),
                Ok(Err(error)) => Err(error.to_string()),
                Err(error) => Err(format!("correction {label} join failed: {error}")),
            };
            self.terminal = Some(result.clone());
            result
        }

        fn close_admission(&self) {
            self.phase_a.begin();
        }

        pub(crate) async fn shutdown(
            mut self,
        ) -> Result<CorrectionTransportShutdownSettled, CorrectionTransportShutdownPending>
        {
            self.close_admission();
            if let Some(Err(error)) = &self.terminal {
                self.shutdown_state.record(error.clone());
            }
            let deadline = Instant::now() + CORRECTION_SHUTDOWN_ATTEMPT_TIMEOUT;

            if !self.shutdown_state.phase_a_settled {
                if wait_for_correction_future_until(deadline, self.phase_a.wait_settled())
                    .await
                    .is_none()
                {
                    let detail = self.pending_shutdown_detail("Phase-A waiter settlement");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                }
                self.shutdown_state.phase_a_settled = true;
            }

            if let Some(shutdown_tx) = self.subscriber_shutdown_tx.take() {
                shutdown_tx.send_replace(true);
            }
            let had_relocalization = self.relocalization.is_some();
            if had_relocalization {
                let relocalization_result = match self
                    .relocalization
                    .as_mut()
                    .and_then(|owner| owner.worker.as_mut())
                {
                    Some(worker) => {
                        settle_retained_correction_worker(
                            "relocalization command owner",
                            worker,
                            deadline,
                        )
                        .await
                    }
                    None => Some(Ok(())),
                };
                let Some(relocalization_result) = relocalization_result else {
                    let detail =
                        self.pending_shutdown_detail("relocalization command owner final join");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                if let Some(relocalization) = self.relocalization.as_mut() {
                    relocalization.worker.take();
                }
                if let Err(error) = relocalization_result {
                    self.shutdown_state.record(error.to_string());
                }

                debug_assert!(self.subscriber_worker.is_none());
                debug_assert!(self.producer_worker.is_none());

                self.cleanup.request_shutdown();
                let cleanup_result = match self.cleanup.worker.as_mut() {
                    Some(worker) => {
                        settle_retained_correction_worker("correction cleanup", worker, deadline)
                            .await
                    }
                    None => Some(Ok(())),
                };
                let Some(cleanup_result) = cleanup_result else {
                    let detail = self.pending_shutdown_detail("correction cleanup final join");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                self.cleanup.worker.take();
                if let Err(error) = cleanup_result {
                    self.shutdown_state.record(error.to_string());
                }

                self.finalizer.request_shutdown();
                let finalizer_result = match self.finalizer.worker.as_mut() {
                    Some(worker) => {
                        settle_retained_correction_worker("canonical finalizer", worker, deadline)
                            .await
                    }
                    None => Some(Ok(())),
                };
                let Some(finalizer_result) = finalizer_result else {
                    let detail = self.pending_shutdown_detail("canonical finalizer final join");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                self.finalizer.worker.take();
                if let Err(error) = finalizer_result {
                    self.shutdown_state.record(error.to_string());
                }

                if let Err(detail) = self.settle_sink_stop(deadline).await {
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                }
            } else {
                // Stop is the in-flight canonical fence. It settles before the
                // private subscriber/finalizer/producer chain is consumed.
                if let Err(detail) = self.settle_sink_stop(deadline).await {
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                }

                let subscriber_result = match self.subscriber_worker.as_mut() {
                    Some(worker) => {
                        settle_retained_correction_worker("subscriber", worker, deadline).await
                    }
                    None => Some(Ok(())),
                };
                let Some(subscriber_result) = subscriber_result else {
                    let detail = self.pending_shutdown_detail("subscriber final join/unsubscribe");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                self.subscriber_worker.take();
                if let Err(error) = subscriber_result {
                    self.shutdown_state.record(error.to_string());
                }

                self.finalizer.request_shutdown();
                let finalizer_result = match self.finalizer.worker.as_mut() {
                    Some(worker) => {
                        settle_retained_correction_worker("canonical finalizer", worker, deadline)
                            .await
                    }
                    None => Some(Ok(())),
                };
                let Some(finalizer_result) = finalizer_result else {
                    let detail = self.pending_shutdown_detail("canonical finalizer final join");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                self.finalizer.worker.take();
                if let Err(error) = finalizer_result {
                    self.shutdown_state.record(error.to_string());
                }

                let producer_result = match self.producer_worker.as_mut() {
                    Some(worker) => {
                        settle_retained_correction_worker("producer", worker, deadline).await
                    }
                    None => Some(Ok(())),
                };
                let Some(producer_result) = producer_result else {
                    let detail = self.pending_shutdown_detail("producer final join");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                self.producer_worker.take();
                if let Err(error) = producer_result {
                    self.shutdown_state.record(error.to_string());
                }

                self.cleanup.request_shutdown();
                let cleanup_result = match self.cleanup.worker.as_mut() {
                    Some(worker) => {
                        settle_retained_correction_worker("correction cleanup", worker, deadline)
                            .await
                    }
                    None => Some(Ok(())),
                };
                let Some(cleanup_result) = cleanup_result else {
                    let detail = self.pending_shutdown_detail("correction cleanup final join");
                    return Err(CorrectionTransportShutdownPending::new(detail, self));
                };
                self.cleanup.worker.take();
                if let Err(error) = cleanup_result {
                    self.shutdown_state.record(error.to_string());
                }
            }

            match self.shutdown_state.detail() {
                Some(detail) => Ok(CorrectionTransportShutdownSettled::failed(format!(
                    "correction shutdown failed: {detail}"
                ))),
                None => Ok(CorrectionTransportShutdownSettled::clean()),
            }
        }

        #[cfg(test)]
        pub(crate) fn inject_terminal_failure_for_shutdown_tests(
            &mut self,
            detail: impl Into<String>,
        ) {
            self.terminal = Some(Err(detail.into()));
        }

        async fn settle_sink_stop(&mut self, deadline: Instant) -> Result<(), String> {
            if !self.shutdown_state.sink_stop_started {
                let sink = Arc::clone(&self.sink);
                self.shutdown_state.sink_stop_worker = Some(tokio::spawn(async move {
                    sink.stop().await;
                    Ok::<(), CorrectionRuntimeError>(())
                }));
                self.shutdown_state.sink_stop_started = true;
            }
            let result = match self.shutdown_state.sink_stop_worker.as_mut() {
                Some(worker) => {
                    settle_retained_correction_worker("canonical correction sink", worker, deadline)
                        .await
                }
                None => Some(Ok(())),
            };
            let Some(result) = result else {
                return Err(self.pending_shutdown_detail("canonical correction sink stop"));
            };
            self.shutdown_state.sink_stop_worker.take();
            if let Err(error) = result {
                self.shutdown_state.record(error.to_string());
            }
            Ok(())
        }

        fn pending_shutdown_detail(&self, stage: &str) -> String {
            let pending = format!(
                "correction shutdown attempt timed out at {stage}; same retained owner must be retried"
            );
            match self.shutdown_state.detail() {
                Some(sticky) => format!("{sticky}; {pending}"),
                None => pending,
            }
        }
    }

    impl Drop for CorrectionTransportOwner {
        fn drop(&mut self) {
            self.close_admission();
            if let Some(shutdown_tx) = &self.subscriber_shutdown_tx {
                shutdown_tx.send_replace(true);
            }
            self.finalizer.request_shutdown();
            self.cleanup.request_shutdown();
        }
    }

    fn correction_candidate_binding(
        runtime_generation: u64,
    ) -> Result<(LcmTopic, LcmMessageType), CorrectionRuntimeError> {
        let topic_name = format!("/c/l/{runtime_generation}");
        let topic = LcmTopic::new(&topic_name)
            .map_err(|error| CorrectionRuntimeError::Transport(error.to_string()))?;
        let message_type = LcmMessageType::new(CORRECTION_CANDIDATE_MESSAGE_TYPE)
            .map_err(|error| CorrectionRuntimeError::Transport(error.to_string()))?;
        if message_type.fingerprint() != CORRECTION_CANDIDATE_FINGERPRINT_V1 {
            return Err(CorrectionRuntimeError::Transport(
                "correction candidate fingerprint registry mismatch".to_owned(),
            ));
        }
        Ok((topic, message_type))
    }

    enum CandidateApplyWaitOutcome {
        Ack(Result<CandidateApplyResult, oneshot::error::RecvError>),
        CancelledBeforeCommit,
        AdmissionClosedBeforeCommit,
        TimedOutBeforeCommit,
    }

    async fn await_candidate_apply(
        mut apply_rx: oneshot::Receiver<CandidateApplyResult>,
        cancellation: &CorrectionRequestCancellation,
        admission_rx: &mut watch::Receiver<bool>,
        timeout: Duration,
        apply_waiters: &CandidateApplyWaiters,
        apply_key: CandidateApplyKey,
    ) -> CandidateApplyWaitOutcome {
        let deadline = tokio::time::sleep(timeout);
        tokio::pin!(deadline);
        let event = tokio::select! {
            biased;
            _ = cancellation.cancelled() => CandidateApplyWaitOutcome::CancelledBeforeCommit,
            _ = wait_for_admission_close(admission_rx) => {
                cancellation.cancel();
                CandidateApplyWaitOutcome::AdmissionClosedBeforeCommit
            }
            result = &mut apply_rx => CandidateApplyWaitOutcome::Ack(result),
            _ = &mut deadline => {
                cancellation.cancel();
                CandidateApplyWaitOutcome::TimedOutBeforeCommit
            }
        };
        match event {
            CandidateApplyWaitOutcome::CancelledBeforeCommit
                if cancellation.canonical_commit_started()
                    || apply_waiters.apply_claimed_or_remove(apply_key) =>
            {
                CandidateApplyWaitOutcome::Ack(apply_rx.await)
            }
            CandidateApplyWaitOutcome::AdmissionClosedBeforeCommit
            | CandidateApplyWaitOutcome::TimedOutBeforeCommit
                if cancellation.canonical_commit_started() =>
            {
                CandidateApplyWaitOutcome::Ack(apply_rx.await)
            }
            other => other,
        }
    }

    async fn run_candidate_producer(
        mut owner: CorrectionCandidateOwner,
        mut requests: mpsc::Receiver<CorrectionFrameRequest>,
        mut admission_rx: watch::Receiver<bool>,
        context: CandidateProducerContext,
    ) -> Result<(), CorrectionRuntimeError> {
        let run_result =
            run_candidate_producer_loop(&mut owner, &mut requests, &mut admission_rx, context)
                .await;
        while let Ok(request) = requests.try_recv() {
            request.cancellation.cancel();
            let _ = request.response.send(Err(CorrectionRuntimeError::Stopped));
        }
        let shutdown_result = owner.shutdown().await;
        match (run_result, shutdown_result) {
            (Ok(()), Ok(())) => Ok(()),
            (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
            (Err(error), Err(shutdown)) => Err(CorrectionRuntimeError::Shutdown(format!(
                "{error}; native/local candidate owner shutdown: {shutdown}"
            ))),
        }
    }

    async fn wait_candidate_owner_finished(owner: &CorrectionCandidateOwner) {
        if matches!(owner, CorrectionCandidateOwner::Local(_)) {
            std::future::pending::<()>().await;
        }
        loop {
            if owner.is_finished() {
                return;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
    }

    const CANDIDATE_FINALIZATION_EVENT_NAME: &str = "nuwax_correction_candidate_finalized_v1";

    #[derive(Debug, serde::Serialize, PartialEq, Eq)]
    enum CandidateFinalizationDisposition {
        Applied,
        AlreadyApplied,
        Rejected,
    }

    #[derive(Debug, serde::Serialize, PartialEq, Eq)]
    struct CandidateFinalizationEvent {
        event: &'static str,
        runtime_generation: u64,
        backend_generation: u64,
        session_generation: u64,
        candidate_sequence: u64,
        payload_sha256: String,
        disposition: CandidateFinalizationDisposition,
        #[serde(skip_serializing_if = "Option::is_none")]
        reject_reason: Option<String>,
        candidate_processed_delivered: bool,
    }

    impl CandidateFinalizationEvent {
        fn new(
            candidate: &CorrectionCandidate,
            disposition: &CorrectionDisposition,
            candidate_processed_delivered: bool,
        ) -> Self {
            let (disposition, reject_reason) = match disposition {
                CorrectionDisposition::Applied => (CandidateFinalizationDisposition::Applied, None),
                CorrectionDisposition::AlreadyApplied => {
                    (CandidateFinalizationDisposition::AlreadyApplied, None)
                }
                CorrectionDisposition::Rejected(reason) => (
                    CandidateFinalizationDisposition::Rejected,
                    Some(reason.to_string()),
                ),
                CorrectionDisposition::Prepared => {
                    unreachable!("Prepared is not a final correction disposition")
                }
            };
            Self {
                event: CANDIDATE_FINALIZATION_EVENT_NAME,
                runtime_generation: candidate.identity.runtime_generation,
                backend_generation: candidate.identity.backend_generation,
                session_generation: candidate.identity.mapping_session_generation,
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: digest_hex(&candidate.payload_sha256),
                disposition,
                reject_reason,
                candidate_processed_delivered,
            }
        }
    }

    impl std::fmt::Display for CandidateFinalizationEvent {
        fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let encoded = serde_json::to_string(self).map_err(|_| std::fmt::Error)?;
            formatter.write_str(&encoded)
        }
    }

    fn digest_hex(digest: &[u8; 32]) -> String {
        use std::fmt::Write as _;

        let mut encoded = String::with_capacity(64);
        for byte in digest {
            write!(&mut encoded, "{byte:02x}").expect("writing to a String cannot fail");
        }
        encoded
    }

    fn respond_and_report_candidate_finalization(
        response: oneshot::Sender<Result<CorrectionSubmitDisposition, CorrectionRuntimeError>>,
        candidate: &CorrectionCandidate,
        disposition: CorrectionDisposition,
    ) -> CandidateFinalizationEvent {
        let mut event = CandidateFinalizationEvent::new(candidate, &disposition, false);
        event.candidate_processed_delivered = response
            .send(Ok(CorrectionSubmitDisposition::CandidateProcessed {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
                disposition,
            }))
            .is_ok();
        tracing::info!(
            target: "nuwax_geometry_runtime::correction_finalization",
            "{event}"
        );
        event
    }

    async fn run_candidate_producer_loop(
        owner: &mut CorrectionCandidateOwner,
        requests: &mut mpsc::Receiver<CorrectionFrameRequest>,
        admission_rx: &mut watch::Receiver<bool>,
        context: CandidateProducerContext,
    ) -> Result<(), CorrectionRuntimeError> {
        let CandidateProducerContext {
            publisher,
            binding,
            finalizer,
            apply_waiters,
            sink,
            cleanup,
            apply_ack_timeout,
            phase_a,
            #[cfg(test)]
            publish_gate,
            #[cfg(test)]
            finalizer_outcome_gate,
        } = context;
        loop {
            let request = tokio::select! {
                biased;
                _ = wait_for_admission_close(admission_rx) => break,
                _ = wait_candidate_owner_finished(owner) => {
                    phase_a.begin();
                    return Err(CorrectionRuntimeError::NativePgo(
                        "native PGO retained owner terminated unexpectedly".to_owned(),
                    ));
                }
                request = requests.recv() => {
                    let Some(request) = request else {
                        phase_a.begin();
                        return Err(CorrectionRuntimeError::IngressClosed);
                    };
                    request
                }
            };
            if request.cancellation.is_cancelled() {
                continue;
            }
            let stage_result = tokio::select! {
                biased;
                _ = request.cancellation.cancelled() => {
                    sink.discard_frame(&request.input).await;
                    continue;
                }
                result = sink.stage_frame(&request.input) => result,
            };
            if let Err(error) = stage_result {
                request.cancellation.cancel();
                phase_a.begin();
                let _ = request
                    .response
                    .send(Err(CorrectionRuntimeError::Apply(error.clone())));
                schedule_discard(&cleanup, request.input).await?;
                return Err(CorrectionRuntimeError::Apply(error));
            }
            if request.cancellation.is_cancelled() {
                sink.discard_frame(&request.input).await;
                continue;
            }
            let staged_input = request.input.clone();
            let production = match owner
                .submit_frame(request.input, &request.cancellation)
                .await
            {
                Ok(production) => production,
                Err(CorrectionRuntimeError::CandidateOwner(error)) => {
                    request.cancellation.cancel();
                    sink.discard_frame(&staged_input).await;
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::CandidateOwner(error)));
                    continue;
                }
                Err(error) => {
                    let detail = error.to_string();
                    request.cancellation.cancel();
                    phase_a.begin();
                    sink.discard_frame(&staged_input).await;
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::NativePgo(detail.clone())));
                    return Err(CorrectionRuntimeError::NativePgo(detail));
                }
            };
            if request.cancellation.is_cancelled() {
                sink.discard_frame(&staged_input).await;
                continue;
            }
            let (candidate, delivery) = match production {
                CandidateProduction::Cancelled => {
                    sink.discard_frame(&staged_input).await;
                    continue;
                }
                CandidateProduction::NoCandidate => {
                    sink.discard_frame(&staged_input).await;
                    let _ = request
                        .response
                        .send(Ok(CorrectionSubmitDisposition::NoCandidate));
                    continue;
                }
                CandidateProduction::Rejected {
                    reason_code,
                    reason,
                } => {
                    sink.discard_frame(&staged_input).await;
                    let _ = request
                        .response
                        .send(Ok(CorrectionSubmitDisposition::Rejected {
                            reason_code,
                            reason,
                        }));
                    continue;
                }
                CandidateProduction::Candidate {
                    candidate,
                    delivery,
                } => (*candidate, delivery),
            };
            let apply_key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let apply_rx = match apply_waiters.register(apply_key, Arc::clone(&request.lease)) {
                Ok(apply_rx) => apply_rx,
                Err(_) if !phase_a.is_open() => {
                    request.cancellation.cancel();
                    let _ = request.response.send(Err(CorrectionRuntimeError::Stopped));
                    schedule_discard(&cleanup, staged_input).await?;
                    break;
                }
                Err(error) => {
                    request.cancellation.cancel();
                    phase_a.begin();
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::Apply(error.clone())));
                    schedule_discard(&cleanup, staged_input).await?;
                    return Err(CorrectionRuntimeError::Apply(error));
                }
            };
            #[cfg(test)]
            if let Some(gate) = publish_gate.as_ref() {
                gate.reached.notify_one();
                tokio::select! {
                    biased;
                    _ = request.cancellation.cancelled() => {
                        apply_waiters.remove(apply_key);
                        sink.discard_frame(&staged_input).await;
                        continue;
                    }
                    _ = gate.release.notified() => {}
                }
                gate.publish_calls.fetch_add(1, Ordering::AcqRel);
            }
            if request.cancellation.is_cancelled() {
                apply_waiters.remove(apply_key);
                sink.discard_frame(&staged_input).await;
                continue;
            }
            match delivery {
                CandidateDelivery::PrivateCorrectionTransport => {
                    let Some(publisher) = publisher.as_ref() else {
                        let detail = "candidate produced without a private publisher".to_owned();
                        apply_waiters.remove(apply_key);
                        request.cancellation.cancel();
                        phase_a.begin();
                        let _ = request
                            .response
                            .send(Err(CorrectionRuntimeError::Transport(detail.clone())));
                        schedule_discard(&cleanup, staged_input).await?;
                        return Err(CorrectionRuntimeError::Transport(detail));
                    };
                    let Some((topic, message_type)) = binding.as_ref() else {
                        let detail =
                            "candidate produced without a private typed binding".to_owned();
                        apply_waiters.remove(apply_key);
                        request.cancellation.cancel();
                        phase_a.begin();
                        let _ = request
                            .response
                            .send(Err(CorrectionRuntimeError::Transport(detail.clone())));
                        schedule_discard(&cleanup, staged_input).await?;
                        return Err(CorrectionRuntimeError::Transport(detail));
                    };
                    let seqno =
                        u32::try_from(candidate.identity.candidate_sequence).map_err(|_| {
                            CorrectionRuntimeError::Transport(
                                "candidate sequence exceeds transport u32 range".to_owned(),
                            )
                        })?;
                    let payload = candidate.encode_body().map_err(|error| {
                        CorrectionRuntimeError::Transport(format!(
                            "candidate encoding failed: {error}"
                        ))
                    })?;
                    let frame = RawLcmFrameBuilder::new(topic.clone(), message_type.clone())
                        .seqno(seqno)
                        .payload(payload)
                        .build();
                    let publish_result = tokio::select! {
                        biased;
                        _ = request.cancellation.cancelled() => {
                            apply_waiters.remove(apply_key);
                            sink.discard_frame(&staged_input).await;
                            continue;
                        }
                        result = tokio::time::timeout(
                            CORRECTION_PUBLISH_TIMEOUT,
                            publisher.publish(frame),
                        ) => result,
                    };
                    let publish_error = match publish_result {
                        Ok(Ok(())) => None,
                        Ok(Err(error)) => Some(error.to_string()),
                        Err(_) => Some("candidate publish timed out".to_owned()),
                    };
                    if let Some(detail) = publish_error {
                        apply_waiters.remove(apply_key);
                        request.cancellation.cancel();
                        phase_a.begin();
                        let _ = request
                            .response
                            .send(Err(CorrectionRuntimeError::Transport(detail.clone())));
                        schedule_discard(&cleanup, staged_input).await?;
                        return Err(CorrectionRuntimeError::Transport(detail));
                    }
                }
                CandidateDelivery::DirectFinalizer => {
                    let Some(lease) = apply_waiters.claim_apply(apply_key) else {
                        let detail = "native correction waiter could not be claimed".to_owned();
                        apply_waiters.remove(apply_key);
                        request.cancellation.cancel();
                        phase_a.begin();
                        let _ = request
                            .response
                            .send(Err(CorrectionRuntimeError::Apply(detail.clone())));
                        schedule_discard(&cleanup, staged_input).await?;
                        return Err(CorrectionRuntimeError::Apply(detail));
                    };
                    let (outcome, _outcome_rx) = oneshot::channel();
                    if let Err(error) = finalizer
                        .sender
                        .send(CanonicalFinalizerJob {
                            key: apply_key,
                            candidate: candidate.clone(),
                            lease,
                            release_lease_on_completion: true,
                            outcome,
                        })
                        .await
                    {
                        let job = error.0;
                        let detail =
                            "canonical finalizer ingress closed before native handoff".to_owned();
                        apply_waiters.remove(apply_key);
                        job.lease.cancellation.cancel();
                        job.lease.cancellation.operation().cancel_pending();
                        job.lease.release();
                        phase_a.begin();
                        let _ =
                            request
                                .response
                                .send(Err(CorrectionRuntimeError::FinalizerPoisoned(
                                    detail.clone(),
                                )));
                        schedule_discard(&cleanup, staged_input).await?;
                        return Err(CorrectionRuntimeError::FinalizerPoisoned(detail));
                    }
                }
            }
            let apply_result = await_candidate_apply(
                apply_rx,
                &request.cancellation,
                admission_rx,
                apply_ack_timeout,
                &apply_waiters,
                apply_key,
            )
            .await;
            let disposition = match apply_result {
                CandidateApplyWaitOutcome::CancelledBeforeCommit => {
                    apply_waiters.remove(apply_key);
                    sink.discard_frame(&staged_input).await;
                    continue;
                }
                CandidateApplyWaitOutcome::AdmissionClosedBeforeCommit => {
                    apply_waiters.remove(apply_key);
                    let _ = request.response.send(Err(CorrectionRuntimeError::Stopped));
                    schedule_discard(&cleanup, staged_input).await?;
                    break;
                }
                CandidateApplyWaitOutcome::TimedOutBeforeCommit => {
                    let detail = "candidate apply acknowledgement timed out".to_owned();
                    apply_waiters.remove(apply_key);
                    phase_a.begin();
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::Apply(detail.clone())));
                    schedule_discard(&cleanup, staged_input).await?;
                    return Err(CorrectionRuntimeError::Apply(detail));
                }
                CandidateApplyWaitOutcome::Ack(Ok(Ok(
                    disposition @ (CorrectionDisposition::Applied
                    | CorrectionDisposition::AlreadyApplied
                    | CorrectionDisposition::Rejected(_)),
                ))) => disposition,
                CandidateApplyWaitOutcome::Ack(Ok(Ok(CorrectionDisposition::Prepared))) => {
                    let detail =
                        "candidate sink returned Prepared without canonical apply".to_owned();
                    apply_waiters.remove(apply_key);
                    request.cancellation.cancel();
                    phase_a.begin();
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::Apply(detail.clone())));
                    schedule_discard(&cleanup, staged_input).await?;
                    return Err(CorrectionRuntimeError::Apply(detail));
                }
                CandidateApplyWaitOutcome::Ack(Ok(Err(error))) => {
                    apply_waiters.remove(apply_key);
                    request.cancellation.cancel();
                    phase_a.begin();
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::Apply(error.clone())));
                    schedule_discard(&cleanup, staged_input).await?;
                    return Err(CorrectionRuntimeError::Apply(error));
                }
                CandidateApplyWaitOutcome::Ack(Err(_)) => {
                    let detail = "candidate apply acknowledgement channel closed".to_owned();
                    apply_waiters.remove(apply_key);
                    request.cancellation.cancel();
                    phase_a.begin();
                    let _ = request
                        .response
                        .send(Err(CorrectionRuntimeError::Apply(detail.clone())));
                    schedule_discard(&cleanup, staged_input).await?;
                    return Err(CorrectionRuntimeError::Apply(detail));
                }
            };
            #[cfg(test)]
            if let Some(gate) = finalizer_outcome_gate.as_ref() {
                gate.reached.notify_one();
                gate.release.notified().await;
            }
            respond_and_report_candidate_finalization(request.response, &candidate, disposition);
        }
        Ok(())
    }

    async fn run_candidate_subscriber(
        mut subscription: OwnedLcmSubscription,
        mut shutdown_rx: watch::Receiver<bool>,
        apply_waiters: Arc<CandidateApplyWaiters>,
        finalizer: CanonicalPublishFinalizerIngress,
    ) -> Result<(), CorrectionRuntimeError> {
        let terminal = loop {
            let frame = tokio::select! {
                biased;
                changed = shutdown_rx.changed() => {
                    if changed.is_err() || *shutdown_rx.borrow_and_update() {
                        break Ok(());
                    }
                    continue;
                }
                result = subscription.recv() => {
                    match result {
                        Ok(frame) => frame,
                        Err(error) => break Err(CorrectionRuntimeError::Transport(error.to_string())),
                    }
                }
            };
            let candidate = match CorrectionCandidate::decode_body(&frame.payload) {
                Ok(candidate) => candidate,
                Err(error) => {
                    tracing::warn!(%error, "private correction candidate rejected");
                    continue;
                }
            };
            let apply_key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let Some(lease) = apply_waiters.claim_apply(apply_key) else {
                tracing::warn!(
                    candidate_sequence = apply_key.candidate_sequence,
                    "private correction candidate has no live request waiter"
                );
                continue;
            };
            let cancellation = lease.cancellation();
            if cancellation.is_cancelled() {
                cancellation.operation().cancel_pending();
                lease.release();
                apply_waiters.complete(
                    apply_key,
                    Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::Stopped,
                    )),
                );
                continue;
            }
            let (outcome, outcome_rx) = oneshot::channel();
            let job = CanonicalFinalizerJob {
                key: apply_key,
                candidate,
                lease,
                release_lease_on_completion: true,
                outcome,
            };
            if let Err(error) = finalizer.sender.send(job).await {
                let job = error.0;
                let detail = "canonical finalizer ingress closed before handoff".to_owned();
                job.lease.cancellation.cancel();
                job.lease.cancellation.operation().cancel_pending();
                job.lease.release();
                apply_waiters.complete(apply_key, Err(detail.clone()));
                break Err(CorrectionRuntimeError::FinalizerPoisoned(detail));
            }
            match outcome_rx.await {
                Ok(Ok(())) => {}
                Ok(Err(error)) => break Err(CorrectionRuntimeError::Apply(error)),
                Err(_) => {
                    break Err(CorrectionRuntimeError::FinalizerPoisoned(
                        "canonical finalizer outcome channel closed".to_owned(),
                    ));
                }
            }
        };
        let unsubscribe = subscription
            .unsubscribe()
            .await
            .map_err(|error| CorrectionRuntimeError::Transport(error.to_string()));
        match (terminal, unsubscribe) {
            (Ok(()), Ok(())) => Ok(()),
            (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
            (Err(terminal), Err(unsubscribe)) => Err(CorrectionRuntimeError::Shutdown(format!(
                "{terminal}; unsubscribe: {unsubscribe}"
            ))),
        }
    }

    struct WireCursor<'a> {
        bytes: &'a [u8],
        offset: usize,
    }

    impl<'a> WireCursor<'a> {
        fn new(bytes: &'a [u8]) -> Self {
            Self { bytes, offset: 0 }
        }

        fn read_array<const N: usize>(&mut self) -> Result<[u8; N], CorrectionWireError> {
            let end = self
                .offset
                .checked_add(N)
                .ok_or_else(|| CorrectionWireError::Invalid("wire offset overflow".into()))?;
            let slice = self
                .bytes
                .get(self.offset..end)
                .ok_or(CorrectionWireError::Truncated {
                    need: end,
                    have: self.bytes.len(),
                })?;
            self.offset = end;
            let mut out = [0; N];
            out.copy_from_slice(slice);
            Ok(out)
        }

        fn read_i32(&mut self) -> Result<i32, CorrectionWireError> {
            Ok(i32::from_be_bytes(self.read_array()?))
        }

        fn read_i64(&mut self) -> Result<i64, CorrectionWireError> {
            Ok(i64::from_be_bytes(self.read_array()?))
        }

        fn read_f64(&mut self) -> Result<f64, CorrectionWireError> {
            Ok(f64::from_be_bytes(self.read_array()?))
        }

        fn read_positive_u64(&mut self, field: &str) -> Result<u64, CorrectionWireError> {
            let value = self.read_i64()?;
            if value <= 0 {
                return Err(CorrectionWireError::Invalid(format!(
                    "{field} must be positive"
                )));
            }
            Ok(value as u64)
        }

        fn read_nonnegative_u64(&mut self, field: &str) -> Result<u64, CorrectionWireError> {
            let value = self.read_i64()?;
            if value < 0 {
                return Err(CorrectionWireError::Invalid(format!(
                    "{field} must be non-negative"
                )));
            }
            Ok(value as u64)
        }

        fn read_lcm_string(&mut self, field: &str) -> Result<String, CorrectionWireError> {
            let length = self.read_i32()?;
            let length = usize::try_from(length).map_err(|_| {
                CorrectionWireError::Invalid(format!("{field} length must be positive"))
            })?;
            if length == 0 || length > MAX_IDENTITY_STRING_BYTES + 1 {
                return Err(CorrectionWireError::Invalid(format!(
                    "{field} length is out of bounds"
                )));
            }
            let bytes = self.read_bytes(length)?;
            if bytes.last() != Some(&0) {
                return Err(CorrectionWireError::Invalid(format!(
                    "{field} is missing its LCM NUL terminator"
                )));
            }
            let value = std::str::from_utf8(&bytes[..bytes.len() - 1])
                .map_err(|_| CorrectionWireError::Invalid(format!("{field} is not UTF-8")))?;
            if value.contains('\0') {
                return Err(CorrectionWireError::Invalid(format!(
                    "{field} contains an embedded NUL"
                )));
            }
            Ok(value.to_owned())
        }

        fn read_bytes(&mut self, length: usize) -> Result<&'a [u8], CorrectionWireError> {
            let end = self
                .offset
                .checked_add(length)
                .ok_or_else(|| CorrectionWireError::Invalid("wire offset overflow".into()))?;
            let bytes = self
                .bytes
                .get(self.offset..end)
                .ok_or(CorrectionWireError::Truncated {
                    need: end,
                    have: self.bytes.len(),
                })?;
            self.offset = end;
            Ok(bytes)
        }

        fn finish(self) -> Result<(), CorrectionWireError> {
            if self.offset == self.bytes.len() {
                Ok(())
            } else {
                Err(CorrectionWireError::Trailing(
                    self.bytes.len() - self.offset,
                ))
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use std::path::PathBuf;
        use std::process::Command;
        use std::sync::Arc;
        use std::time::Duration;

        use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
        use nuwax_transport::LcmTransport;

        use super::*;

        const COMMIT_NONCE: u128 = 0x1020_3040_5060_7080_90a0_b0c0_d0e0_f001;
        const LEASE_NONCE: u128 = 0x8877_6655_4433_2211_1020_3040_5060_7080;
        static NATIVE_CORRECTION_TEST_LOCK: tokio::sync::Mutex<()> =
            tokio::sync::Mutex::const_new(());

        async fn native_correction_test_guard() -> tokio::sync::MutexGuard<'static, ()> {
            // 直接使用同次构建的 PGO 程序；发布清单由专门的发布测试覆盖。
            NATIVE_CORRECTION_TEST_LOCK.lock().await
        }

        pub(super) struct RelocalizationTerminalTestRoot(PathBuf);

        impl RelocalizationTerminalTestRoot {
            pub(super) fn new() -> Self {
                Self(std::env::temp_dir().join(format!(
                    "nuwax-relocalization-test-{}-{}",
                    std::process::id(),
                    uuid::Uuid::new_v4(),
                )))
            }

            pub(super) fn path(&self) -> &Path {
                &self.0
            }
        }

        impl Drop for RelocalizationTerminalTestRoot {
            fn drop(&mut self) {
                // 存储的最后一份共享引用释放租约后，才删除该测试独占的终态目录。
                if let Err(error) = fs::remove_dir_all(&self.0) {
                    if error.kind() != std::io::ErrorKind::NotFound {
                        tracing::warn!(%error, "重定位测试终态目录清理失败");
                    }
                }
            }
        }

        const GENERATED_PAYLOAD_SHA256: [u8; 32] = [
            0xb9, 0xc5, 0xa8, 0x81, 0x10, 0xd9, 0x96, 0x8b, 0x19, 0x09, 0xa3, 0x63, 0xc8, 0x1e,
            0xa0, 0x61, 0xe8, 0x36, 0xc0, 0x16, 0x54, 0xc5, 0xb5, 0x9f, 0x51, 0xdc, 0xea, 0x3c,
            0xe3, 0xa6, 0x49, 0xe3,
        ];
        const GENERATED_BODY_SHA256: [u8; 32] = [
            0xbf, 0x04, 0xc6, 0xcb, 0x93, 0xa5, 0x61, 0xb5, 0xc0, 0xca, 0x23, 0xda, 0x94, 0x1b,
            0x9a, 0xd4, 0xb2, 0x67, 0x55, 0x08, 0x7e, 0x91, 0x82, 0x10, 0xea, 0x5e, 0x65, 0xf3,
            0x20, 0x94, 0x1e, 0xb8,
        ];
        const GENERATED_WIRE_SHA256: [u8; 32] = [
            0xd0, 0xf7, 0xb3, 0x8d, 0xe8, 0xd8, 0x8a, 0x2c, 0x2c, 0x1c, 0xdc, 0x2f, 0xe8, 0xc6,
            0xcb, 0x51, 0x26, 0x04, 0xb8, 0xc2, 0xd8, 0x25, 0x80, 0xf2, 0x83, 0x20, 0xe8, 0x3f,
            0x98, 0xee, 0xb0, 0x52,
        ];

        fn exact_hex(bytes: &[u8]) -> String {
            bytes.iter().map(|byte| format!("{byte:02x}")).collect()
        }

        #[test]
        fn relocalization_request_id_uses_canonical_ascii_contract() {
            assert!(validate_relocalization_request_id("req.A_z:9-0").is_ok());
            for invalid in ["", " request", "request ", "request/child", "请求", "a\n"] {
                assert_eq!(
                    validate_relocalization_request_id(invalid),
                    Err("RELOCALIZATION_REQUEST_ID_INVALID".to_owned())
                );
            }
            assert_eq!(
                validate_relocalization_request_id(&"a".repeat(129)),
                Err("RELOCALIZATION_REQUEST_ID_INVALID".to_owned())
            );
        }

        #[test]
        fn relocalization_terminal_store_root_resolves_deployment_and_local_paths() {
            let current_directory = || Ok(PathBuf::from("/workspace"));
            assert_eq!(
                terminal_store_root_from(
                    Some(OsString::from("/override/store")),
                    Some(OsString::from("/var/lib/nuwax/geometry-runtime:/ignored")),
                    Some(OsString::from("/runtime-data")),
                    || panic!("绝对终态目录不应读取当前工作目录"),
                ),
                Ok(PathBuf::from("/override/store"))
            );
            assert_eq!(
                terminal_store_root_from(
                    None,
                    Some(OsString::from("/var/lib/nuwax/geometry-runtime:/ignored")),
                    Some(OsString::from("/runtime-data")),
                    || panic!("绝对部署目录不应读取当前工作目录"),
                ),
                Ok(PathBuf::from(
                    "/var/lib/nuwax/geometry-runtime/relocalization_terminals_v1"
                ))
            );
            assert_eq!(
                terminal_store_root_from(
                    None,
                    None,
                    Some(OsString::from("/runtime-data")),
                    || panic!("绝对运行数据目录不应读取当前工作目录"),
                ),
                Ok(PathBuf::from(
                    "/runtime-data/geometry/relocalization_terminals_v1"
                ))
            );
            assert_eq!(
                terminal_store_root_from(
                    Some(OsString::from("relative")),
                    Some(OsString::from("/state")),
                    Some(OsString::from("/runtime-data")),
                    current_directory,
                ),
                Ok(PathBuf::from("/workspace/relative"))
            );
            assert_eq!(
                terminal_store_root_from(
                    None,
                    Some(OsString::from("relative:/state")),
                    None,
                    current_directory,
                ),
                Err("RELOCALIZATION_TERMINAL_STORE_PATH_NOT_ABSOLUTE".to_owned())
            );
            assert_eq!(
                terminal_store_root_from(None, None, None, current_directory),
                Ok(PathBuf::from(
                    "/workspace/runtime_data/geometry/relocalization_terminals_v1"
                ))
            );
            assert_eq!(
                terminal_store_root_from(
                    None,
                    None,
                    Some(OsString::from("relative-data")),
                    current_directory,
                ),
                Ok(PathBuf::from(
                    "/workspace/relative-data/geometry/relocalization_terminals_v1"
                ))
            );
            assert_eq!(
                terminal_store_root_from(
                    Some(OsString::new()),
                    None,
                    None,
                    current_directory,
                ),
                Err("RELOCALIZATION_TERMINAL_STORE_PATH_INVALID".to_owned())
            );
        }

        #[test]
        fn relocalization_default_and_relative_roots_persist_and_reopen() {
            for (explicit, runtime_data, expected) in [
                (
                    None,
                    None,
                    "runtime_data/geometry/relocalization_terminals_v1",
                ),
                (Some("relative-terminals"), None, "relative-terminals"),
                (
                    None,
                    Some("relative-data"),
                    "relative-data/geometry/relocalization_terminals_v1",
                ),
            ] {
                let current_directory = RelocalizationTerminalTestRoot::new();
                let root = terminal_store_root_from(
                    explicit.map(OsString::from),
                    None,
                    runtime_data.map(OsString::from),
                    || Ok(current_directory.path().to_path_buf()),
                )
                .expect("默认和相对目录必须按当前工作目录解析");
                assert_eq!(root, current_directory.path().join(expected));
                let request_identity = [3_u8; 32];
                let activation_identity = [5_u8; 32];
                let result = crate::proto::RelocalizeResult {
                    schema_version: 1,
                    request_id: "req-relative-store-reopen".to_owned(),
                    request_identity_sha256: request_identity.to_vec(),
                    activation_identity_sha256: activation_identity.to_vec(),
                    disposition: crate::proto::RelocalizeDisposition::Applied as i32,
                    terminal_proof_sha256: vec![7; 32],
                    ..Default::default()
                };
                {
                    // 直接覆盖生产解析结果与真实存储，不经过测试专用的默认存储夹具。
                    let store = RelocalizationTerminalStore::open_at(root.clone())
                        .expect("按生产解析目录打开终态存储");
                    store
                        .persist_terminal(
                            &result.request_id,
                            request_identity,
                            activation_identity,
                            &StoredRelocalizationResult::new(result.clone()),
                        )
                        .expect("真实落盘重定位终态");
                }
                let reopened = RelocalizationTerminalStore::open_at(root)
                    .expect("释放后重新打开同一生产解析目录");
                match reopened.snapshot(
                    &result.request_id,
                    request_identity,
                    activation_identity,
                ) {
                    RelocalizationTerminalSnapshot::Terminal(replayed) => {
                        assert_eq!(replayed.encode_to_vec(), result.encode_to_vec());
                    }
                    other => panic!("重新打开终态不一致：{other:?}"),
                }
            }
        }

        #[test]
        fn durable_relocalization_terminal_store_reopens_and_detects_identity_conflict() {
            let root = std::env::temp_dir().join(format!(
                "nuwax-relocalization-terminal-store-{}-{}",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .expect("system clock")
                    .as_nanos(),
            ));
            let request_identity = [3_u8; 32];
            let activation_identity = [5_u8; 32];
            let result = crate::proto::RelocalizeResult {
                schema_version: 1,
                request_id: "req-store-reopen".to_owned(),
                request_identity_sha256: request_identity.to_vec(),
                activation_identity_sha256: activation_identity.to_vec(),
                disposition: crate::proto::RelocalizeDisposition::Applied as i32,
                terminal_proof_sha256: vec![7; 32],
                ..Default::default()
            };
            let terminal = StoredRelocalizationResult::new(result.clone());
            {
                let store = RelocalizationTerminalStore::open_at(root.clone())
                    .expect("open terminal store");
                store
                    .persist_terminal(
                        &result.request_id,
                        request_identity,
                        activation_identity,
                        &terminal,
                    )
                    .expect("persist terminal");
            }
            let reopened =
                RelocalizationTerminalStore::open_at(root.clone()).expect("reopen terminal store");
            match reopened.snapshot(&result.request_id, request_identity, activation_identity) {
                RelocalizationTerminalSnapshot::Terminal(replayed) => {
                    assert_eq!(replayed.encode_to_vec(), result.encode_to_vec());
                }
                other => panic!("unexpected reopened snapshot: {other:?}"),
            }
            assert!(matches!(
                reopened.snapshot(&result.request_id, [9; 32], activation_identity),
                RelocalizationTerminalSnapshot::IdentityConflict
            ));
            fs::remove_dir_all(root).expect("remove terminal store");
        }

        #[test]
        fn durable_relocalization_terminal_store_rejects_noncanonical_record_request_id() {
            let root = std::env::temp_dir().join(format!(
                "nuwax-relocalization-terminal-invalid-id-{}-{}",
                std::process::id(),
                terminal_owner_token(),
            ));
            let request_id = "req-store-invalid-id";
            let request_identity = [3_u8; 32];
            let activation_identity = [5_u8; 32];
            let terminal = StoredRelocalizationResult::new(crate::proto::RelocalizeResult {
                schema_version: 1,
                request_id: request_id.to_owned(),
                request_identity_sha256: request_identity.to_vec(),
                activation_identity_sha256: activation_identity.to_vec(),
                disposition: crate::proto::RelocalizeDisposition::Applied as i32,
                terminal_proof_sha256: vec![7; 32],
                ..Default::default()
            });
            {
                let store = RelocalizationTerminalStore::open_at(root.clone())
                    .expect("open terminal store");
                store
                    .persist_terminal(request_id, request_identity, activation_identity, &terminal)
                    .expect("persist terminal");
            }
            let path = root.join(format!(
                "{}.json",
                encode_hex(&sha256_bytes(request_id.as_bytes()))
            ));
            let mut record: DurableRelocalizationTerminalV1 =
                serde_json::from_slice(&fs::read(&path).expect("read terminal record"))
                    .expect("decode terminal record");
            record.request_id = " invalid".to_owned();
            fs::write(
                &path,
                serde_json::to_vec(&record).expect("encode invalid terminal record"),
            )
            .expect("write invalid terminal record");

            assert_eq!(
                RelocalizationTerminalStore::open_at(root.clone())
                    .err()
                    .expect("invalid request ID must fail closed"),
                "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID"
            );
            fs::remove_dir_all(root).expect("remove terminal store");
        }

        #[test]
        fn durable_relocalization_terminal_store_cleans_only_unlocked_owner() {
            let root = std::env::temp_dir().join(format!(
                "nuwax-relocalization-terminal-temp-cleanup-{}-{}",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .expect("system clock")
                    .as_nanos(),
            ));
            fs::create_dir_all(&root).expect("create terminal store root");
            let digest = "0".repeat(64);
            let active_owner = terminal_owner_token();
            let active_lease_path = root.join(terminal_lease_name(&active_owner));
            let active_lease_file = OpenOptions::new()
                .read(true)
                .write(true)
                .create_new(true)
                .open(&active_lease_path)
                .expect("create active lease");
            let active_lease = Flock::lock(active_lease_file, FlockArg::LockExclusiveNonblock)
                .expect("lock active lease");
            let active = root.join(format!(
                "{RELOCALIZATION_TERMINAL_TEMP_PREFIX}{}.{}.{}.{}{RELOCALIZATION_TERMINAL_TEMP_SUFFIX}",
                active_owner,
                terminal_owner_token(),
                digest,
                digest,
            ));
            fs::write(&active, b"active").expect("write active temp");

            let orphan_owner = terminal_owner_token();
            let orphan_lease = root.join(terminal_lease_name(&orphan_owner));
            fs::write(&orphan_lease, b"").expect("write orphan lease");
            let orphan = root.join(format!(
                "{RELOCALIZATION_TERMINAL_TEMP_PREFIX}{}.{}.{}.{}{RELOCALIZATION_TERMINAL_TEMP_SUFFIX}",
                orphan_owner,
                terminal_owner_token(),
                digest,
                digest,
            ));
            fs::write(&orphan, b"orphan").expect("write orphan temp");

            RelocalizationTerminalStore::open_at(root.clone()).expect("open terminal store");

            assert!(active.exists());
            assert!(active_lease_path.exists());
            assert!(!orphan.exists());
            assert!(!orphan_lease.exists());
            drop(active_lease);
            fs::remove_dir_all(root).expect("remove terminal store");
        }

        #[test]
        fn durable_relocalization_terminal_store_lock_unknown_is_retained() {
            let root = std::env::temp_dir().join(format!(
                "nuwax-relocalization-terminal-lock-unknown-{}-{}",
                std::process::id(),
                terminal_owner_token(),
            ));
            fs::create_dir_all(&root).expect("create terminal store root");
            let owner = terminal_owner_token();
            let digest = "0".repeat(64);
            let temporary = root.join(format!(
                "{RELOCALIZATION_TERMINAL_TEMP_PREFIX}{}.{}.{}.{}{RELOCALIZATION_TERMINAL_TEMP_SUFFIX}",
                owner,
                terminal_owner_token(),
                digest,
                digest,
            ));
            fs::write(&temporary, b"unknown").expect("write unknown temp");

            let cleanup_lock = lock_terminal_cleanup(&root).expect("lock terminal cleanup");
            assert_eq!(
                cleanup_terminal_owner(&root, &owner, &cleanup_lock),
                Err("RELOCALIZATION_TERMINAL_STORE_OWNER_LEASE_MISSING".to_owned())
            );
            drop(cleanup_lock);
            assert!(temporary.exists());
            fs::remove_dir_all(root).expect("remove terminal store");
        }

        #[test]
        fn durable_relocalization_terminal_store_incarnations_do_not_collide() {
            let first = terminal_owner_token();
            let second = terminal_owner_token();
            assert_ne!(first, second);
            assert!(is_terminal_token(
                &first,
                RELOCALIZATION_TERMINAL_OWNER_TOKEN_BYTES
            ));
            assert!(is_terminal_token(
                &second,
                RELOCALIZATION_TERMINAL_OWNER_TOKEN_BYTES
            ));
            assert_ne!(terminal_lease_name(&first), terminal_lease_name(&second));
        }

        #[test]
        fn durable_relocalization_terminal_store_preserves_primary_on_cleanup_failure() {
            assert_eq!(
                combine_terminal_persist_results(
                    Err("RELOCALIZATION_TERMINAL_STORE_WRITE_FAILED:disk".to_owned()),
                    Err("RELOCALIZATION_TERMINAL_STORE_TEMP_CLEANUP_FAILED:denied".to_owned()),
                ),
                Err(
                    "RELOCALIZATION_TERMINAL_STORE_WRITE_FAILED:disk|TEMP_CLEANUP_FAILED"
                        .to_owned()
                )
            );
            assert_eq!(
                combine_terminal_persist_results(
                    Ok(()),
                    Err("RELOCALIZATION_TERMINAL_STORE_TEMP_CLEANUP_FAILED:denied".to_owned()),
                ),
                Ok(())
            );
        }

        #[test]
        fn durable_relocalization_terminal_store_removes_temporary_on_publish_conflict() {
            let root = std::env::temp_dir().join(format!(
                "nuwax-relocalization-terminal-temp-conflict-{}-{}",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .expect("system clock")
                    .as_nanos(),
            ));
            let request_id = "req-store-conflict-cleanup";
            let request_identity = [3_u8; 32];
            let activation_identity = [5_u8; 32];
            let result = crate::proto::RelocalizeResult {
                schema_version: 1,
                request_id: request_id.to_owned(),
                request_identity_sha256: request_identity.to_vec(),
                activation_identity_sha256: activation_identity.to_vec(),
                disposition: crate::proto::RelocalizeDisposition::Applied as i32,
                terminal_proof_sha256: vec![7; 32],
                ..Default::default()
            };
            let terminal = StoredRelocalizationResult::new(result);
            let store =
                RelocalizationTerminalStore::open_at(root.clone()).expect("open terminal store");
            let final_path = root.join(format!(
                "{}.json",
                encode_hex(&sha256_bytes(request_id.as_bytes()))
            ));
            fs::write(&final_path, b"different terminal").expect("write conflicting object");

            assert_eq!(
                store.persist_terminal(
                    request_id,
                    request_identity,
                    activation_identity,
                    &terminal,
                ),
                Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".to_owned())
            );
            let owned_temporary_count = fs::read_dir(&root)
                .expect("read terminal store")
                .map(|entry| entry.expect("read terminal entry"))
                .filter(|entry| {
                    entry
                        .file_name()
                        .to_str()
                        .map(is_owned_terminal_temporary_name)
                        .unwrap_or(false)
                })
                .count();
            assert_eq!(owned_temporary_count, 0);
            assert_eq!(
                fs::read(&final_path).expect("read conflicting object"),
                b"different terminal"
            );
            fs::remove_dir_all(root).expect("remove terminal store");
        }

        #[test]
        fn terminal_record_reader_enforces_fixed_limit_and_growth_boundary() {
            let at_limit = vec![b'x'; RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES];
            assert_eq!(
                read_terminal_record_bytes(
                    std::io::Cursor::new(at_limit.clone()),
                    RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES as u64,
                )
                .expect("fixed-limit reader accepts exactly the byte limit"),
                at_limit
            );
            assert_eq!(
                read_terminal_record_bytes(
                    std::io::Cursor::new(vec![b'x'; RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES + 1]),
                    (RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES + 1) as u64,
                ),
                Err("RELOCALIZATION_TERMINAL_STORE_RECORD_TOO_LARGE".to_owned())
            );
            assert_eq!(
                read_terminal_record_bytes(
                    std::io::Cursor::new(vec![b'x'; RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES + 1]),
                    RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES as u64,
                ),
                Err("RELOCALIZATION_TERMINAL_STORE_RECORD_TOO_LARGE".to_owned()),
                "bounded read must detect growth after metadata inspection"
            );
        }

        #[test]
        fn durable_relocalization_terminal_store_bounds_publish_conflict_read() {
            let root = std::env::temp_dir().join(format!(
                "nuwax-relocalization-terminal-bounded-conflict-{}-{}",
                std::process::id(),
                terminal_owner_token(),
            ));
            let request_id = "req-store-bounded-conflict";
            let request_identity = [3_u8; 32];
            let activation_identity = [5_u8; 32];
            let terminal = StoredRelocalizationResult::new(crate::proto::RelocalizeResult {
                schema_version: 1,
                request_id: request_id.to_owned(),
                request_identity_sha256: request_identity.to_vec(),
                activation_identity_sha256: activation_identity.to_vec(),
                disposition: crate::proto::RelocalizeDisposition::Applied as i32,
                terminal_proof_sha256: vec![7; 32],
                ..Default::default()
            });
            let store =
                RelocalizationTerminalStore::open_at(root.clone()).expect("open terminal store");
            let final_path = root.join(format!(
                "{}.json",
                encode_hex(&sha256_bytes(request_id.as_bytes()))
            ));
            let oversized = File::create(&final_path).expect("create oversized conflicting object");
            oversized
                .set_len((RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES + 1) as u64)
                .expect("extend oversized conflicting object");
            drop(oversized);

            assert_eq!(
                store.persist_terminal(
                    request_id,
                    request_identity,
                    activation_identity,
                    &terminal,
                ),
                Err("RELOCALIZATION_TERMINAL_STORE_RECORD_TOO_LARGE".to_owned())
            );
            fs::remove_dir_all(root).expect("remove terminal store");
        }

        #[test]
        fn relocalization_terminal_proof_v1_matches_independent_full_result_golden() {
            let result = crate::proto::RelocalizeResult {
                schema_version: 1,
                request_id: "req-golden".to_owned(),
                request_identity_sha256: vec![1; 32],
                runtime_generation: 2,
                session_id: "session-golden".to_owned(),
                map_name: "map-golden".to_owned(),
                session_generation: 3,
                backend_generation: 4,
                activation_identity_sha256: vec![5; 32],
                target_receipt_sha256: vec![6; 32],
                target_cloud_revision: 7,
                target_cloud_sha256: vec![8; 32],
                asset_manifest_sha256: vec![9; 32],
                asset_bundle_sha256: vec![10; 32],
                source_policy_version: 1,
                source_policy_sha256: vec![11; 32],
                deadline_policy_version: 1,
                deadline_policy_sha256: vec![12; 32],
                logical_timeout_ms: 30_000,
                algorithm_options_version: 1,
                algorithm_options_sha256: vec![13; 32],
                disposition: crate::proto::RelocalizeDisposition::Applied as i32,
                candidate_sequence: 14,
                candidate_payload_sha256: vec![15; 32],
                evidence_sha256: vec![16; 32],
                correction_revision: 17,
                terminal_proof_sha256: Vec::new(),
                detail: "APPLIED".to_owned(),
                candidate_payload_preimage_v1: vec![0x29; 19],
                evidence_disclosure_v1: vec![0x30; 23],
            };
            let preimage = canonical_relocalization_terminal_proof_preimage_v1(&result)
                .expect("clear proof field has a canonical preimage");
            assert_eq!(preimage.len(), 745);
            assert_eq!(
                exact_hex(&preimage),
                "6e757761782e67656f6d657472792e72656c6f63616c697a652e7465726d696e616c2d70726f6f662e763100000000010000000100000001000000020000000a7265712d676f6c64656e000000030100000000000000200101010101010101010101010101010101010101010101010101010101010101000000040000000000000002000000050000000e73657373696f6e2d676f6c64656e000000060000000a6d61702d676f6c64656e0000000700000000000000030000000800000000000000040000000901000000000000002005050505050505050505050505050505050505050505050505050505050505050000000a01000000000000002006060606060606060606060606060606060606060606060606060606060606060000000b00000000000000070000000c01000000000000002008080808080808080808080808080808080808080808080808080808080808080000000d01000000000000002009090909090909090909090909090909090909090909090909090909090909090000000e0100000000000000200a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0000000f00000001000000100100000000000000200b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0000001100000001000000120100000000000000200c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0c0000001300000000000075300000001400000001000000150100000000000000200d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d0d000000160000000100000017000000000000000e000000180100000000000000200f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0000001901000000000000002010101010101010101010101010101010101010101010101010101010101010100000001a00000000000000110000001b000000001c000000074150504c494544"
            );
            assert_eq!(
                crate::map_checksum::sha256_hex(&preimage),
                "c64dac3ba805655b262490a0db376552bfe41c1043a749eaf4819ef306a00358"
            );

            let mut already_hashed = result;
            already_hashed.terminal_proof_sha256 = vec![0xaa; 32];
            assert!(canonical_relocalization_terminal_proof_preimage_v1(&already_hashed).is_err());
        }

        #[test]
        fn relocalization_composite_sha256_matches_independent_oracle_8515_cases() {
            let mut case_count = 0_usize;
            for case in 0..8_515 {
                case_count += 1;
                let source = (0..=case % 521)
                    .map(|offset| ((case * 37 + offset * 19) & 0xff) as u8)
                    .collect::<Vec<_>>();
                let mut prefix = b"independent-prefix".to_vec();
                prefix.extend_from_slice(&(case as u64).to_be_bytes());
                prefix.extend_from_slice(
                    &relocalization_source_commitment_record_v1(&source).unwrap(),
                );
                let mut suffix = b"independent-suffix".to_vec();
                suffix.extend_from_slice(&(case as u64).rotate_left(17).to_be_bytes());
                let mut bytes = prefix.clone();
                let source_start = bytes.len();
                bytes.extend_from_slice(&source);
                let source_end = bytes.len();
                bytes.extend_from_slice(&suffix);
                let preimage = CanonicalRelocalizationEvidencePreimageV1 {
                    bytes,
                    canonical_source_range: Some(source_start..source_end),
                };

                let mut independent = RELOCALIZATION_EVIDENCE_COMPOSITE_DOMAIN_V1.to_vec();
                independent.extend_from_slice(&RELOCALIZATION_SCHEMA_VERSION_V1.to_be_bytes());
                independent.extend_from_slice(&0_u32.to_be_bytes());
                independent.extend_from_slice(
                    &u64::try_from(prefix.len() + source.len() + suffix.len())
                        .unwrap()
                        .to_be_bytes(),
                );
                independent.extend_from_slice(&1_u32.to_be_bytes());
                independent.extend_from_slice(&u64::try_from(prefix.len()).unwrap().to_be_bytes());
                independent.extend_from_slice(&prefix);
                independent.extend_from_slice(&2_u32.to_be_bytes());
                independent.extend_from_slice(&u64::try_from(source.len()).unwrap().to_be_bytes());
                independent.extend_from_slice(&sha256_bytes(&source));
                independent.extend_from_slice(&3_u32.to_be_bytes());
                independent.extend_from_slice(&u64::try_from(suffix.len()).unwrap().to_be_bytes());
                independent.extend_from_slice(&suffix);
                assert_eq!(
                    canonical_relocalization_evidence_sha256_v1(&preimage).unwrap(),
                    sha256_bytes(&independent),
                    "case={case}"
                );
            }
            assert_eq!(case_count, 8_515);
        }

        #[test]
        fn relocalization_source_elided_disclosure_has_hard_maximum_bound() {
            let source = vec![0x22; RELOCALIZATION_MAX_CANONICAL_SOURCE_BYTES];
            let source_commitment = relocalization_source_commitment_record_v1(&source).unwrap();
            let mut prefix =
                vec![0x11; RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES - source_commitment.len()];
            prefix.extend_from_slice(&source_commitment);
            let mut preimage = Vec::with_capacity(
                prefix.len() + source.len() + RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES,
            );
            preimage.extend_from_slice(&prefix);
            let source_start = preimage.len();
            preimage.extend_from_slice(&source);
            let source_end = preimage.len();
            let mut suffix = Vec::new();
            suffix.resize(RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES, 0x33);
            preimage.extend_from_slice(&suffix);

            let disclosure = build_relocalization_evidence_disclosure_v1(
                &preimage,
                Some(source_start..source_end),
            )
            .expect("maximum bounded source has a compact disclosure");
            assert!(disclosure.len() <= RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES);
            assert!(preimage.len() > disclosure.len() * 100);
            assert_eq!(
                disclosure[RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1.len() + 4],
                1
            );

            assert!(build_relocalization_evidence_disclosure_v1(
                &[
                    prefix,
                    vec![0x44; RELOCALIZATION_MAX_CANONICAL_SOURCE_BYTES + 1],
                    suffix,
                ]
                .concat(),
                Some(
                    RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES
                        ..RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES
                            + RELOCALIZATION_MAX_CANONICAL_SOURCE_BYTES
                            + 1
                ),
            )
            .is_err());
        }

        #[test]
        fn relocalization_disclosure_retention_bounds_are_fixed() {
            assert_eq!(RELOCALIZATION_MAX_INLINE_EVIDENCE_DISCLOSURE_BYTES, 4_158);
            assert_eq!(RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_TERMINAL, 24_832);
            assert_eq!(
                RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_COMMAND_RECORD,
                295_102
            );
            assert_eq!(RELOCALIZATION_MAX_RETAINED_DISCLOSURE_BYTES, 604_368_896);
        }

        #[test]
        fn relocalization_candidate_disclosure_is_the_exact_payload_preimage() {
            let candidate = sample_candidate(1, 200);
            let preimage = relocalization_candidate_payload_preimage_v1(Some(&candidate)).unwrap();
            assert!(!preimage.is_empty());
            assert!(preimage.len() <= RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES);
            assert_eq!(sha256_bytes(&preimage), candidate.payload_sha256);
            assert!(preimage
                .windows(std::mem::size_of::<u128>())
                .any(|window| window == LEASE_NONCE.to_be_bytes()));
            assert!(relocalization_candidate_payload_preimage_v1(None)
                .unwrap()
                .is_empty());
        }

        #[test]
        fn relocalization_evidence_v1_matches_independent_full_execution_golden() {
            let target_receipt_sha256 = [0x33; 32];
            let snapshot = SealedActivatedTargetSnapshot {
                runtime_generation: 2,
                lifecycle_epoch: 3,
                session_id: "s".to_owned(),
                map_name: "m".to_owned(),
                activation_request_id: "a".to_owned(),
                activation_request_identity_sha256: [0x31; 32],
                activation_identity_sha256: [0x32; 32],
                target_receipt_sha256,
                asset_manifest_sha256: Some([0x34; 32]),
                asset_bundle_sha256: Some([0x35; 32]),
                session_generation: 4,
                backend_generation: 5,
                publication_proof: ActivationPublicationProof {
                    source_timestamp_ns: 6,
                    cloud_revision: 7,
                    point_count: 1,
                    voxel_size: 0.5,
                    cloud_receipt: crate::mapping_commit::CloudCommitReceipt {
                        commit_id: MappingCommitId {
                            boot_nonce: 8,
                            sequence: 9,
                        },
                        session_generation: 4,
                        occupancy_revision: 10,
                        cloud_revision: 7,
                        source_timestamp_ns: 6,
                        point_count: 1,
                    },
                    target_receipt_sha256,
                },
                immutable_target: crate::pointcloud::ImmutableActivatedTarget {
                    frame_id: "map".to_owned(),
                    source_revision: 11,
                    point_count: 1,
                    voxel_size: 0.5,
                    xyz_f32_le: (0_u8..12).collect(),
                    xyz_sha256: [0x35; 32],
                    payload_schema_version: 1,
                },
            };
            let registered_cloud = RegisteredCloudProof {
                source_sequence: 17,
                source_timestamp_ns: 16,
                point_count: 2,
                envelope_sha256: [0x41; 32],
                body_points_sha256: [0x42; 32],
            };
            let correlated_odom = CorrelatedOdomProof {
                source_sequence: 18,
                source_timestamp_ns: 15,
                pose_sha256: [0x43; 32],
            };
            let committed_map = CommittedMapProof {
                source_timestamp_ns: 16,
                width: 21,
                height: 22,
                resolution_m: 0.25,
                origin_x_m: -1.5,
                origin_y_m: 2.5,
                payload_sha256: [0x44; 32],
                boundary_sha256: [0x45; 32],
            };
            let boundary_base_anchor = Isometry3::translation(-4.0, 5.0, -6.0);
            let frames = vec![AccumulatedRelocalizationFrame {
                evidence_sha256: [0x46; 32],
                input: CorrectionFrameInput {
                    mapping_session_id: "fs".to_owned(),
                    mapping_session_generation: 19,
                    mapping_commit_id: MappingCommitId {
                        boot_nonce: 12,
                        sequence: 13,
                    },
                    occupancy_revision: 14,
                    cloud_revision: 15,
                    map_frame_generation: 20,
                    registered_cloud,
                    correlated_odom: FrozenCorrelatedOdom {
                        proof: correlated_odom,
                        odom_from_body: Isometry3::translation(1.0, 2.0, 3.0),
                    },
                    committed_map,
                    committed_map_occupancy: Arc::from(Vec::<u8>::new()),
                    points_frame_id: "base".to_owned(),
                    body_points: Vec::new(),
                    base_anchor_map_from_odom: boundary_base_anchor,
                    frame_capture_relocalization_epoch: 1,
                    relocalization_mode_epoch: Some(1),
                },
            }];
            let boundary_key = relocalization_frame_key(&frames[0].input);
            let algorithm_result = RelocalizationResult {
                registration: nuwax_mapping::relocalization::IcpResult {
                    transformation: Isometry3::translation(7.0, 8.0, 9.0),
                    mse: 0.125,
                    iterations: 23,
                    converged: true,
                },
                feature_matches: 24,
                ransac_inliers: 25,
                fitness: 0.75,
            };
            let correction = Isometry3::translation(-1.0, -2.0, -3.0);
            let candidate = CorrectionCandidate {
                identity: CorrectionCandidateIdentity {
                    backend: CorrectionCandidateBackend::Relocalization,
                    transform_semantics: CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3,
                    runtime_generation: 2,
                    backend_generation: 99,
                    candidate_sequence: 26,
                    mapping_session_id: "fs".to_owned(),
                    mapping_session_generation: 19,
                    map_frame_generation: 20,
                    mapping_commit_id: MappingCommitId {
                        boot_nonce: 12,
                        sequence: 13,
                    },
                    occupancy_revision: 14,
                    cloud_revision: 15,
                    registered_cloud,
                    correlated_odom,
                    committed_map,
                    source_timestamp_ns: 16,
                    map_frame_id: "map".to_owned(),
                    odom_frame_id: "odom".to_owned(),
                    body_frame_id: "base".to_owned(),
                    points_frame_id: "base".to_owned(),
                    correction_lease: CorrectionLeaseId {
                        boot_nonce: 27,
                        generation: 28,
                    },
                },
                transform: CorrectionTransform::from_isometry(&correction),
                payload_sha256: [0x48; 32],
            };
            let source_bytes = [0xaa, 0xbb, 0xcc];
            let evidence = RelocalizationEvidenceV1 {
                request_identity_sha256: [0x21; 32],
                admission_ordinal: Some(29),
                normalized_logical_timeout_ms: Some(30_000),
                runtime_generation: 2,
                activated_mapping_backend_generation: 5,
                correction_owner_backend_generation: 99,
                session_id: "s",
                map_name: "m",
                session_generation: 4,
                map_frame_id: "map",
                odom_frame_id: "odom",
                body_frame_id: "base",
                activation_identity_sha256: [0x32; 32],
                target_receipt_sha256,
                asset_manifest_sha256: [0x34; 32],
                asset_bundle_sha256: [0x35; 32],
                source_policy_version: 1,
                source_policy_sha256: [0x51; 32],
                deadline_policy_version: 1,
                deadline_policy_sha256: [0x52; 32],
                algorithm_options_version: 1,
                algorithm_options_sha256: [0x53; 32],
                target: Some(RelocalizationTargetEvidenceV1 {
                    snapshot: &snapshot,
                    decoded_point_count: 1,
                    decoded_f64_bytes: 24,
                    decoded_sha256: [0x36; 32],
                }),
                source: Some(RelocalizationSourceEvidenceV1 {
                    arm_watermark: None,
                    cutoff_reason: RelocalizationCutoffReasonV1::MinimumAggregateReached,
                    last_accepted_key: Some(boundary_key),
                    frames: &frames,
                    aggregate_input_points: 2,
                    aggregate_input_f64_bytes: 48,
                    post_transform_points: 2,
                    post_transform_f64_bytes: 48,
                    emitted_points: 2,
                    canonical_source_bytes: &source_bytes,
                    source_sha256: sha256_bytes(&source_bytes),
                    boundary_key,
                    boundary_base_anchor_map_from_odom: &boundary_base_anchor,
                }),
                algorithm: Some(RelocalizationAlgorithmEvidenceV1 {
                    result: &algorithm_result,
                    accepted: true,
                    correction: Some(&correction),
                }),
                candidate: Some(RelocalizationCandidateEvidenceV1 {
                    candidate: &candidate,
                }),
                terminal_disposition: crate::proto::RelocalizeDisposition::Applied,
                correction_revision: 30,
                detail: "OK",
            };
            assert_ne!(
                evidence.activated_mapping_backend_generation,
                evidence.correction_owner_backend_generation
            );
            let preimage = canonical_relocalization_evidence_preimage_v1(&evidence)
                .expect("full evidence fixture is encodable");
            assert_eq!(preimage.len(), 2_221);
            assert_ne!(
                exact_hex(&preimage),
                "6e757761782e67656f6d657472792e72656c6f63616c697a652e65766964656e63652e763100000000010000000121212121212121212121212121212121212121212121212121212121212121210000000201000000000000001d0000000301000000000000753000000004000000000000000200000005000000000000000500000006000000000000006300000007000000017300000008000000016d0000000900000000000000040000000a000000036d61700000000b000000046f646f6d0000000c00000004626173650000000d32323232323232323232323232323232323232323232323232323232323232320000000e33333333333333333333333333333333333333333333333333333333333333330000000f343434343434343434343434343434343434343434343434343434343434343400000010353535353535353535353535353535353535353535353535353535353535353500000011000000015151515151515151515151515151515151515151515151515151515151515151000000120000000152525252525252525252525252525252525252525252525252525252525252520000001300000001535353535353535353535353535353535353535353535353535353535353535300000014017461726765742e763100000000000000000200000000000000030000000173000000016d000000000000000400000000000000050000000161313131313131313131313131313131313131313131313131313131313131313132323232323232323232323232323232323232323232323232323232323232323333333333333333333333333333333333333333333333333333333333333333013434343434343434343434343434343434343434343434343434343434343434013535353535353535353535353535353535353535353535353535353535353535000000036d6170000000000000000b000000013f00000000000001000000000000000c353535353535353535353535353535353535353535353535353535353535353500000000000000010000000000000018363636363636363636363636363636363636363636363636363636363636363600000000000000060000000000000007000000013f0000000000000000000000000000000000000800000000000000090000000000000004000000000000000a000000000000000700000000000000060000000133333333333333333333333333333333333333333333333333333333333333330000001501736f757263652e7631000000000001010000000000000000000000000000000c000000000000000d000000000000000e000000000000000f000000000000001000000000000000110000000000000012000000016672616d652e7631000000000000000000000000000000000c000000000000000d000000000000000e000000000000000f00000000000000100000000000000011000000000000001200000002667300000000000000130000000000000014000000000000000241414141414141414141414141414141414141414141414141414141414141414242424242424242424242424242424242424242424242424242424242424242000000000000000f43434343434343434343434343434343434343434343434343434343434343430000000000000010000000000000001500000000000000163fd0000000000000bff800000000000040040000000000004444444444444444444444444444444444444444444444444444444444444444454545454545454545454545454545454545454545454545454545454545454500000004626173653ff0000000000000400000000000000040080000000000000000000000000000000000000000000000000000000000003ff0000000000000c0100000000000004014000000000000c0180000000000000000000000000000000000000000000000000000000000003ff00000000000004646464646464646464646464646464646464646464646464646464646464646000000000000000200000000000000300000000000000002000000000000003000000000000000020000000000000003aabbcc6302c5ce60f3079cff01ef770792a240edf870195e17045398a4c728167fb1150000000000000000000000000000000c000000000000000d000000000000000e000000000000000f000000000000001000000000000000110000000000000012c0100000000000004014000000000000c0180000000000000000000000000000000000000000000000000000000000003ff00000000000000000001601616c676f726974686d2e763100013fc00000000000000000000000000017000000000000001800000000000000193fe8000000000000401c000000000000402000000000000040220000000000000000000000000000000000000000000000000000000000003ff00000000000000101bff0000000000000c000000000000000c0080000000000000000000000000000000000000000000000000000000000003ff0000000000000000000170163616e6469646174652e763100000000030000000100000000000000020000000000000063000000000000001a000000026673000000000000001300000000000000140000000000000000000000000000000c000000000000000d000000000000000e000000000000000f0000000000000010000000000000001100000000000000120000000000000000000000000000001b000000000000001cbff0000000000000c000000000000000c0080000000000000000000000000000000000000000000000000000000000003ff00000000000004848484848484848484848484848484848484848484848484848484848484848000000180000000100000019000000000000001e0000001a000000024f4b0000001b7465726d696e616c2d70726f6f662d666f6c6c6f7773000000001c00"
            );
            assert_eq!(
                crate::map_checksum::sha256_hex(&preimage),
                "39f48623882635a76451a4f835c474c92b629226f89c537ab937eac896662d73"
            );
            let composable_sha256 = relocalization_evidence_sha256_v1(&evidence).unwrap();
            assert_eq!(
                exact_hex(&composable_sha256),
                "af545f03b558c6303a76ae475548d35c306d60f2776e14484bb7852343bb3bfa"
            );
            assert_ne!(composable_sha256, sha256_bytes(&preimage));
        }

        #[test]
        fn relocalization_policy_v1_preimages_and_hashes_match_fixed_literals() {
            let source = relocalization_source_policy_preimage_v1();
            assert_eq!(source.len(), 179);
            assert_eq!(
                exact_hex(&source),
                "6e757761782e67656f6d657472792e72656c6f63616c697a652e736f757263652d706f6c6963792e76310000000001000000010000000100000008000000000000c35000000000000186a00000000000249f000000000000001000000000000186a000000000000003d09000000000005b8d80000000000003d09000000000005b8d80000000000000c35000000000000186a00000000000124f800000000000249f0000000000000007d00000000100000001"
            );
            assert_eq!(
                crate::map_checksum::sha256_hex(&source),
                "b0e3c49266e73179cee77207a64680ee719e497886db18ea2267093c94a76aa8"
            );

            let deadline = relocalization_deadline_policy_preimage_v1();
            assert_eq!(deadline.len(), 101);
            assert_eq!(
                exact_hex(&deadline),
                "6e757761782e67656f6d657472792e72656c6f63616c697a652e646561646c696e652d706f6c6963792e7631000000000100000000000000fa0000000000007530000000000000753000000000000007d00000000000001388000000010000000100000001"
            );
            assert_eq!(
                crate::map_checksum::sha256_hex(&deadline),
                "cf2793085656ef81fadf12a80fb1147973ec71a77735a4a3e600ac1345b59c70"
            );

            let algorithm = relocalization_algorithm_options_preimage_v1();
            assert_eq!(algorithm.len(), 221);
            assert_eq!(
                exact_hex(&algorithm),
                "6e757761782e67656f6d657472792e72656c6f63616c697a652e616c676f726974686d2d6f7074696f6e732e763100000000010000002e6e757761785f6d617070696e673a3a72656c6f636174696f6e5f706970656c696e655f776974685f636f6e66696700000001000000033fc999999999999a3fd33333333333333fe999999999999a0000000000000064000000000000100000000000000000033fb999999999999a3fc333333333333400000000000000033fdccccccccccccd00000000000000323eb0c6f7a0b5ed8d3fc33333333333344024000000000000"
            );
            assert_eq!(
                crate::map_checksum::sha256_hex(&algorithm),
                "f8b5ef8b359eeb0129f6d523244ea8bca35102143f3bce28e619598749276090"
            );
        }

        fn relocalization_command_request(
            request_id: &str,
            target_cloud_revision: u64,
        ) -> crate::proto::RelocalizeRequest {
            crate::proto::RelocalizeRequest {
                schema_version: RELOCALIZATION_SCHEMA_VERSION_V1,
                request_id: request_id.to_owned(),
                runtime_generation: 2,
                session_id: "session-ledger".to_owned(),
                map_name: "map-ledger".to_owned(),
                session_generation: 3,
                backend_generation: 4,
                activation_identity_sha256: vec![5; 32],
                target_receipt_sha256: vec![6; 32],
                target_cloud_revision,
                target_cloud_sha256: vec![8; 32],
                asset_manifest_sha256: vec![9; 32],
                asset_bundle_sha256: vec![10; 32],
                source_policy_version: RELOCALIZATION_SOURCE_POLICY_VERSION_V1,
                source_policy_sha256: relocalization_source_policy_sha256_v1().to_vec(),
                deadline_policy_version: RELOCALIZATION_DEADLINE_POLICY_VERSION_V1,
                deadline_policy_sha256: relocalization_deadline_policy_sha256_v1().to_vec(),
                logical_timeout_ms: 1_000,
                algorithm_options_version: RELOCALIZATION_ALGORITHM_OPTIONS_VERSION_V1,
                algorithm_options_sha256: relocalization_algorithm_options_sha256_v1().to_vec(),
            }
        }

        fn relocalization_command_terminal(
            request: &ValidatedRelocalizeRequest,
            disposition: crate::proto::RelocalizeDisposition,
            detail: &'static str,
        ) -> StoredRelocalizationResult {
            let evidence_disclosure_v1 =
                build_relocalization_evidence_disclosure_v1(detail.as_bytes(), None).unwrap();
            StoredRelocalizationResult::new(build_relocalization_result(
                &request.request,
                request.identity_sha256,
                disposition,
                0,
                None,
                Vec::new(),
                sha256_bytes(detail.as_bytes()),
                evidence_disclosure_v1,
                0,
                detail,
            ))
        }

        #[tokio::test]
        async fn relocalization_disclosure_replay_is_byte_identical() {
            let raw_request = relocalization_command_request("req-disclosure-replay", 7);
            let request = validate_relocalize_request(raw_request.clone()).unwrap();
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut actor = RelocalizationCommandActor::new(
                owner_identity(),
                "robot_map".to_owned(),
                "robot_odom".to_owned(),
                "robot_body".to_owned(),
            );
            actor.ledger.relocalization_mode = phase_a.relocalization_mode();
            let (detached_tx, _detached_rx) = mpsc::unbounded_channel();
            let first_permit = Arc::new(Semaphore::new(1)).try_acquire_owned().unwrap();
            let (first_response, first_response_rx) = oneshot::channel();
            actor
                .handle_admission(
                    RelocalizationAdmissionRequest {
                        request: raw_request.clone(),
                        capacity_lease: Arc::new(RelocalizationWaiterCapacityLease::new(
                            first_permit,
                        )),
                        response: first_response,
                        mapping_lifecycle_guard: None,
                    },
                    &detached_tx,
                    None,
                    &phase_a,
                )
                .unwrap();
            let first_waiter = first_response_rx.await.unwrap().unwrap();
            let terminal = actor.terminal_result(
                &request,
                Some(1),
                crate::proto::RelocalizeDisposition::InsufficientSource,
                "RELOCALIZATION_INSUFFICIENT_SOURCE",
            );
            let pending = actor
                .ledger
                .settle_terminal(
                    &request.request.request_id,
                    request.identity_sha256,
                    terminal,
                )
                .unwrap();
            let stored = actor.ledger.terminal(&raw_request.request_id).unwrap();
            let stored_encoded = stored.encoded.clone();
            let stored_encoded_sha256 = stored.encoded_sha256;
            actor.notify_terminal(pending);
            let first = first_waiter.wait().await.unwrap();

            let replay_permit = Arc::new(Semaphore::new(1)).try_acquire_owned().unwrap();
            let (replay_response, replay_response_rx) = oneshot::channel();
            actor
                .handle_admission(
                    RelocalizationAdmissionRequest {
                        request: raw_request,
                        capacity_lease: Arc::new(RelocalizationWaiterCapacityLease::new(
                            replay_permit,
                        )),
                        response: replay_response,
                        mapping_lifecycle_guard: None,
                    },
                    &detached_tx,
                    None,
                    &phase_a,
                )
                .unwrap();
            let replay = replay_response_rx
                .await
                .unwrap()
                .unwrap()
                .wait()
                .await
                .unwrap();

            let first_encoded = first.encode_to_vec();
            let replay_encoded = replay.encode_to_vec();
            assert_eq!(first_encoded, stored_encoded);
            assert_eq!(replay_encoded, stored_encoded);
            assert_eq!(first_encoded, replay_encoded);
            assert_eq!(sha256_bytes(&stored_encoded), stored_encoded_sha256);
        }

        #[test]
        fn relocalization_command_ledger_join_preserves_first_logical_deadline() {
            let request = validate_relocalize_request(relocalization_command_request("req-a", 7))
                .expect("valid request");
            let now = Instant::now();
            let mut ledger = RelocalizationCommandLedger::default();
            let started = ledger
                .admit(&request, 1, now, || unreachable!("no conflict"))
                .expect("first admission");
            let (ordinal, first_deadline) = match started {
                RelocalizationLedgerAdmission::Started {
                    admission_ordinal,
                    logical_deadline,
                } => (admission_ordinal, logical_deadline),
                _ => panic!("first request must start"),
            };
            assert_eq!(ordinal, 1);
            assert_eq!(first_deadline, now + Duration::from_secs(1));

            for waiter_id in 2..=17 {
                let joined = ledger
                    .admit(
                        &request,
                        waiter_id,
                        now + Duration::from_millis(900),
                        || unreachable!("same identity cannot conflict"),
                    )
                    .expect("same identity joins instead of BUSY");
                match joined {
                    RelocalizationLedgerAdmission::Joined {
                        admission_ordinal,
                        logical_deadline,
                    } => {
                        assert_eq!(admission_ordinal, ordinal);
                        assert_eq!(logical_deadline, first_deadline);
                    }
                    RelocalizationLedgerAdmission::Busy => {
                        panic!("same-ID same-identity waiter {waiter_id} must not be BUSY")
                    }
                    _ => panic!("same identity must join"),
                }
            }
        }

        #[test]
        fn relocalization_ledger_observation_counts_only_new_admission_and_first_claim() {
            let request = validate_relocalize_request(relocalization_command_request("req-a", 7))
                .expect("valid request");
            let mut ledger = RelocalizationCommandLedger::default();
            let epoch = ledger
                .observation()
                .expect("initial observation")
                .observation_epoch;
            assert_ne!(epoch, 0);

            ledger
                .admit(&request, 1, Instant::now(), || unreachable!("no conflict"))
                .expect("first admission");
            ledger
                .admit(&request, 2, Instant::now(), || {
                    unreachable!("same identity")
                })
                .expect("same-ID join");
            let before_claim = ledger.observation().expect("before claim observation");
            assert_eq!(before_claim.admitted_command_count, 1);
            assert_eq!(before_claim.claimed_command_count, 0);
            assert_eq!(before_claim.terminal_record_count, 0);
            assert_eq!(before_claim.pending_record_count, 1);
            assert_eq!(before_claim.active_waiter_count, 2);

            ledger
                .mark_postclaim("req-a", request.identity_sha256)
                .expect("first claim");
            ledger
                .mark_postclaim("req-a", request.identity_sha256)
                .expect("idempotent claim");
            let terminal = relocalization_command_terminal(
                &request,
                crate::proto::RelocalizeDisposition::Cancelled,
                "RELOCALIZATION_CANCELLED",
            );
            ledger
                .settle_terminal("req-a", request.identity_sha256, terminal)
                .expect("terminal settlement");
            let settled = ledger.observation().expect("settled observation");
            assert_eq!(settled.observation_epoch, epoch);
            assert_eq!(settled.admitted_command_count, 1);
            assert_eq!(settled.claimed_command_count, 1);
            assert_eq!(settled.terminal_record_count, 1);
            assert_eq!(settled.pending_record_count, 0);
            assert_eq!(settled.active_waiter_count, 0);
        }

        #[test]
        fn relocalization_command_terminal_is_stored_before_waiter_notification_and_replays() {
            let request = validate_relocalize_request(relocalization_command_request("req-a", 7))
                .expect("valid request");
            let mut ledger = RelocalizationCommandLedger::default();
            ledger
                .admit(&request, 9, Instant::now(), || unreachable!("no conflict"))
                .expect("first admission");
            let terminal = relocalization_command_terminal(
                &request,
                crate::proto::RelocalizeDisposition::Cancelled,
                "RELOCALIZATION_LAST_WAITER_CANCELLED",
            );
            let pending_notifications = ledger
                .settle_terminal("req-a", request.identity_sha256, terminal.clone())
                .expect("terminal settlement");

            let stored_before_notify = ledger.terminal("req-a").expect("terminal is durable");
            assert_eq!(stored_before_notify.encoded, terminal.encoded);
            assert_eq!(pending_notifications.waiter_ids, vec![9]);
            assert_eq!(pending_notifications.result.encoded, terminal.encoded);

            let replay = ledger
                .admit(&request, 10, Instant::now(), || unreachable!("no conflict"))
                .expect("terminal replay");
            let replay = match replay {
                RelocalizationLedgerAdmission::Replay(result) => result,
                _ => panic!("terminal request must replay"),
            };
            assert_eq!(replay.encoded, terminal.encoded);
            assert_eq!(replay.encoded_sha256, terminal.encoded_sha256);
        }

        #[test]
        fn relocalization_command_conflict_is_permanent_and_busy_does_not_insert() {
            let request_a = validate_relocalize_request(relocalization_command_request("req-a", 7))
                .expect("valid request A");
            let request_b = validate_relocalize_request(relocalization_command_request("req-b", 7))
                .expect("valid request B");
            let conflicting_a =
                validate_relocalize_request(relocalization_command_request("req-a", 8))
                    .expect("valid conflicting request A");
            let mut ledger = RelocalizationCommandLedger::default();
            ledger
                .admit(&request_a, 1, Instant::now(), || {
                    unreachable!("no conflict")
                })
                .expect("first admission");

            assert!(matches!(
                ledger
                    .admit(&request_b, 2, Instant::now(), || unreachable!("busy"))
                    .expect("busy response"),
                RelocalizationLedgerAdmission::Busy
            ));
            assert!(ledger.request("req-b").is_none());

            let conflict_builds = std::cell::Cell::new(0_u64);
            let first_conflict = ledger
                .admit(&conflicting_a, 3, Instant::now(), || {
                    conflict_builds.set(conflict_builds.get() + 1);
                    relocalization_command_terminal(
                        &conflicting_a,
                        crate::proto::RelocalizeDisposition::ReplayConflict,
                        "RELOCALIZATION_REPLAY_CONFLICT",
                    )
                })
                .expect("first conflict");
            let second_conflict = ledger
                .admit(&conflicting_a, 4, Instant::now(), || {
                    panic!("stored conflict must replay")
                })
                .expect("stored conflict");
            let first_conflict = match first_conflict {
                RelocalizationLedgerAdmission::Conflict(result) => result,
                _ => panic!("identity mismatch must conflict"),
            };
            let second_conflict = match second_conflict {
                RelocalizationLedgerAdmission::Conflict(result) => result,
                _ => panic!("identity mismatch must replay conflict"),
            };
            assert_eq!(conflict_builds.get(), 1);
            assert_eq!(first_conflict.encoded, second_conflict.encoded);

            let terminal = relocalization_command_terminal(
                &request_a,
                crate::proto::RelocalizeDisposition::Cancelled,
                "RELOCALIZATION_CANCELLED",
            );
            ledger
                .settle_terminal("req-a", request_a.identity_sha256, terminal)
                .expect("finish active request");
            let started_b = ledger
                .admit(&request_b, 5, Instant::now(), || {
                    unreachable!("no conflict")
                })
                .expect("next request starts");
            assert!(matches!(
                started_b,
                RelocalizationLedgerAdmission::Started {
                    admission_ordinal: 2,
                    ..
                }
            ));
        }

        #[test]
        fn relocalization_command_ledger_caps_fail_closed_without_consuming_ordinal() {
            let mut ledger = RelocalizationCommandLedger::default();
            for index in 0..RELOCALIZATION_MAX_COMMAND_RECORDS.saturating_sub(1) {
                let request = validate_relocalize_request(relocalization_command_request(
                    &format!("req-cap-{index}"),
                    7,
                ))
                .expect("bounded request is valid");
                let admission = ledger
                    .admit(&request, index as u64 + 1, Instant::now(), || {
                        unreachable!("new request cannot conflict")
                    })
                    .expect("bounded request admits");
                assert!(matches!(
                    admission,
                    RelocalizationLedgerAdmission::Started { .. }
                ));
                let terminal = relocalization_command_terminal(
                    &request,
                    crate::proto::RelocalizeDisposition::Cancelled,
                    "RELOCALIZATION_CAP_FIXTURE_SETTLED",
                );
                ledger
                    .settle_terminal(
                        &request.request.request_id,
                        request.identity_sha256,
                        terminal,
                    )
                    .expect("bounded request settles");
            }
            let ordinal_at_capacity = ledger.next_admission_ordinal;
            let overflow =
                validate_relocalize_request(relocalization_command_request("req-cap-overflow", 7))
                    .expect("overflow request is valid");
            for waiter_id in [10_000_u64, 10_001] {
                assert!(matches!(
                    ledger
                        .admit(&overflow, waiter_id, Instant::now(), || {
                            unreachable!("capacity overflow cannot conflict")
                        })
                        .expect("capacity overflow is typed"),
                    RelocalizationLedgerAdmission::CapacityExceeded(
                        "RELOCALIZATION_COMMAND_LEDGER_CAPACITY_EXCEEDED"
                    )
                ));
            }
            let overflow_terminal = relocalization_command_terminal(
                &overflow,
                crate::proto::RelocalizeDisposition::OwnerFailed,
                "RELOCALIZATION_COMMAND_LEDGER_CAPACITY_EXCEEDED",
            );
            ledger
                .store_capacity_terminal(&overflow, Instant::now(), overflow_terminal.clone())
                .expect("capacity terminal is retained in reserved slot");
            let replay = ledger
                .admit(&overflow, 10_002, Instant::now(), || {
                    unreachable!("stored capacity terminal must replay")
                })
                .expect("capacity terminal replays");
            match replay {
                RelocalizationLedgerAdmission::Replay(result) => {
                    assert_eq!(result.encoded, overflow_terminal.encoded)
                }
                _ => panic!("capacity terminal must replay"),
            }
            assert_eq!(ledger.next_admission_ordinal, ordinal_at_capacity);

            let base =
                validate_relocalize_request(relocalization_command_request("req-conflict-cap", 7))
                    .expect("base conflict request is valid");
            let mut conflicts = RelocalizationCommandLedger::default();
            conflicts
                .admit(&base, 1, Instant::now(), || unreachable!("base is new"))
                .expect("base request admits");
            for index in 0..RELOCALIZATION_MAX_CONFLICTS_PER_RECORD {
                let conflicting = validate_relocalize_request(relocalization_command_request(
                    "req-conflict-cap",
                    index as u64 + 8,
                ))
                .expect("conflicting request is valid");
                assert!(matches!(
                    conflicts
                        .admit(&conflicting, index as u64 + 2, Instant::now(), || {
                            relocalization_command_terminal(
                                &conflicting,
                                crate::proto::RelocalizeDisposition::ReplayConflict,
                                "RELOCALIZATION_REPLAY_CONFLICT",
                            )
                        })
                        .expect("bounded conflict is typed"),
                    RelocalizationLedgerAdmission::Conflict(_)
                ));
            }
            let overflow_conflict = validate_relocalize_request(relocalization_command_request(
                "req-conflict-cap",
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD as u64 + 8,
            ))
            .expect("overflow conflict is valid");
            let overflow_conflict_terminal = relocalization_command_terminal(
                &overflow_conflict,
                crate::proto::RelocalizeDisposition::ReplayConflict,
                "RELOCALIZATION_REPLAY_CONFLICT",
            );
            let first_overflow = conflicts
                .admit(&overflow_conflict, 100, Instant::now(), || {
                    overflow_conflict_terminal.clone()
                })
                .expect("conflict overflow is typed");
            let second_overflow = conflicts
                .admit(&overflow_conflict, 101, Instant::now(), || {
                    overflow_conflict_terminal.clone()
                })
                .expect("overflow conflict remains permanent");
            let (first_overflow, second_overflow) = match (first_overflow, second_overflow) {
                (
                    RelocalizationLedgerAdmission::ConflictOverflow(first),
                    RelocalizationLedgerAdmission::Conflict(second),
                ) => (first, second),
                _ => panic!("overflow identity must remain a permanent conflict"),
            };
            assert_eq!(first_overflow.encoded, second_overflow.encoded);
            assert_eq!(
                first_overflow.encoded_sha256,
                second_overflow.encoded_sha256
            );
            let record = conflicts
                .records
                .get("req-conflict-cap")
                .expect("base record remains");
            let (stored_identity, stored_overflow) = record
                .conflict_overflow
                .as_ref()
                .expect("overflow terminal is retained before reply");
            assert_eq!(*stored_identity, overflow_conflict.identity_sha256);
            assert_eq!(stored_overflow.encoded, first_overflow.encoded);
            assert_eq!(
                stored_overflow.encoded_sha256,
                first_overflow.encoded_sha256
            );
            let next_conflict = validate_relocalize_request(relocalization_command_request(
                "req-conflict-cap",
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD as u64 + 9,
            ))
            .expect("next overflow conflict is valid");
            assert!(matches!(
                conflicts
                    .admit(&next_conflict, 102, Instant::now(), || {
                        panic!("bounded overflow must not build another terminal")
                    })
                    .expect("overflow closes at the bounded slot"),
                RelocalizationLedgerAdmission::CapacityExceeded(
                    "RELOCALIZATION_COMMAND_CONFLICT_OVERFLOW_CLOSED"
                )
            ));
            assert_eq!(
                conflicts
                    .records
                    .get("req-conflict-cap")
                    .expect("base record remains")
                    .conflicts
                    .len(),
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD
            );
            assert_eq!(conflicts.next_admission_ordinal, 1);
        }

        #[test]
        fn relocalization_mode_epoch_exhaustion_does_not_claim_the_ledger() {
            let request =
                validate_relocalize_request(relocalization_command_request("req-epoch-max", 7))
                    .expect("valid request");
            let mut ledger = RelocalizationCommandLedger::default();
            ledger
                .relocalization_mode
                .state
                .store(u64::MAX - 1, std::sync::atomic::Ordering::Release);

            assert!(matches!(
                ledger.admit(&request, 1, Instant::now(), || unreachable!("new request")),
                Err("RELOCALIZATION_MODE_EPOCH_EXHAUSTED")
            ));
            assert!(ledger.active_request_id.is_none());
            assert!(ledger.records.is_empty());
            assert_eq!(ledger.next_admission_ordinal, 0);
        }

        #[tokio::test]
        async fn relocalization_conflict_overflow_is_stored_before_actor_reply() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut actor = RelocalizationCommandActor::new(
                owner_identity(),
                "robot_map".to_owned(),
                "robot_odom".to_owned(),
                "robot_body".to_owned(),
            );
            let base = validate_relocalize_request(relocalization_command_request(
                "req-conflict-overflow-actor",
                7,
            ))
            .expect("base request is valid");
            actor
                .ledger
                .admit(&base, 1, Instant::now(), || unreachable!("base is new"))
                .expect("base request admits");
            for index in 0..RELOCALIZATION_MAX_CONFLICTS_PER_RECORD {
                let conflict = validate_relocalize_request(relocalization_command_request(
                    "req-conflict-overflow-actor",
                    index as u64 + 8,
                ))
                .expect("cached conflict is valid");
                let terminal = actor.terminal_result(
                    &conflict,
                    None,
                    crate::proto::RelocalizeDisposition::ReplayConflict,
                    "RELOCALIZATION_REPLAY_CONFLICT",
                );
                assert!(matches!(
                    actor
                        .ledger
                        .admit(&conflict, index as u64 + 2, Instant::now(), || terminal)
                        .expect("cached conflict admits"),
                    RelocalizationLedgerAdmission::Conflict(_)
                ));
            }

            let overflow_request = relocalization_command_request(
                "req-conflict-overflow-actor",
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD as u64 + 8,
            );
            let overflow = validate_relocalize_request(overflow_request.clone())
                .expect("overflow conflict is valid");
            let waiter_permit = Arc::new(Semaphore::new(1))
                .try_acquire_owned()
                .expect("test waiter permit");
            let capacity_lease = Arc::new(RelocalizationWaiterCapacityLease::new(waiter_permit));
            let (response, response_rx) = oneshot::channel();
            let (detached_tx, _detached_rx) = mpsc::unbounded_channel();
            actor
                .handle_admission(
                    RelocalizationAdmissionRequest {
                        request: overflow_request,
                        capacity_lease,
                        response,
                        mapping_lifecycle_guard: None,
                    },
                    &detached_tx,
                    None,
                    &phase_a,
                )
                .expect("overflow conflict is handled");

            let stored = actor
                .ledger
                .records
                .get("req-conflict-overflow-actor")
                .and_then(|record| record.conflict_overflow.as_ref())
                .expect("overflow terminal is stored before reply consumption");
            assert_eq!(stored.0, overflow.identity_sha256);
            let stored_encoded = stored.1.encoded.clone();
            let stored_encoded_sha256 = stored.1.encoded_sha256;
            let waiter = response_rx
                .await
                .expect("actor reply is delivered")
                .expect("overflow conflict returns a waiter");
            let reply = waiter.wait().await.expect("overflow terminal is delivered");
            assert_eq!(reply.encode_to_vec(), stored_encoded);
            assert_eq!(sha256_bytes(&reply.encode_to_vec()), stored_encoded_sha256);
            assert!(!phase_a.is_open(), "overflow closes further admission");

            let replay = actor
                .ledger
                .admit(&overflow, 100, Instant::now(), || {
                    panic!("stored overflow conflict must replay")
                })
                .expect("overflow conflict replays");
            match replay {
                RelocalizationLedgerAdmission::Conflict(result) => {
                    assert_eq!(result.encoded, stored_encoded);
                    assert_eq!(result.encoded_sha256, stored_encoded_sha256);
                }
                _ => panic!("stored overflow conflict must remain REPLAY_CONFLICT"),
            }
        }

        #[test]
        fn relocalization_command_last_waiter_cancels_only_before_claim() {
            let request = validate_relocalize_request(relocalization_command_request("req-a", 7))
                .expect("valid request");
            let mut preclaim = RelocalizationCommandLedger::default();
            preclaim
                .admit(&request, 1, Instant::now(), || unreachable!("no conflict"))
                .expect("first waiter");
            preclaim
                .admit(&request, 2, Instant::now(), || unreachable!("no conflict"))
                .expect("second waiter");
            assert!(!preclaim
                .detach_waiter("req-a", request.identity_sha256, 1)
                .expect("detach first waiter"));
            assert!(preclaim
                .detach_waiter("req-a", request.identity_sha256, 2)
                .expect("detach final preclaim waiter"));

            let mut postclaim = RelocalizationCommandLedger::default();
            postclaim
                .admit(&request, 3, Instant::now(), || unreachable!("no conflict"))
                .expect("postclaim request");
            postclaim
                .mark_postclaim("req-a", request.identity_sha256)
                .expect("mark claimed");
            assert!(!postclaim
                .detach_waiter("req-a", request.identity_sha256, 3)
                .expect("detach postclaim waiter"));
        }

        fn relocalization_command_supervisor(
            phase_a: CorrectionPhaseAControl,
        ) -> RelocalizationOwnerSupervisor {
            RelocalizationOwnerSupervisor::start(
                phase_a,
                owner_identity(),
                "map".to_owned(),
                "odom".to_owned(),
                "base_link".to_owned(),
            )
        }

        #[tokio::test]
        async fn relocalization_retained_command_phase_a_stores_terminal_before_reply() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut supervisor = relocalization_command_supervisor(phase_a.clone());
            let ingress = supervisor.ingress();
            let request = relocalization_command_request("req-phase-a", 7);
            let first = ingress
                .admit(request.clone())
                .await
                .expect("first retained waiter");
            let second = ingress
                .admit(request.clone())
                .await
                .expect("joined retained waiter");
            first.cancel_preclaim();
            phase_a.begin();

            let result = tokio::time::timeout(Duration::from_secs(1), second.wait())
                .await
                .expect("phase A settlement is bounded")
                .expect("phase A returns a stored terminal");
            assert_eq!(
                result.disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            assert_eq!(result.candidate_sequence, 0);
            assert_eq!(result.terminal_proof_sha256.len(), 32);
            assert_eq!(result.detail, "RELOCALIZATION_OWNER_STOPPED_PRECLAIM");
            supervisor.shutdown().await.expect("retained actor joins");
            assert!(supervisor.is_finished());
        }

        #[tokio::test]
        async fn relocalization_retained_command_deadline_replays_identical_terminal() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut supervisor = relocalization_command_supervisor(phase_a.clone());
            let ingress = supervisor.ingress();
            let mut request = relocalization_command_request("req-deadline", 7);
            request.logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            let first = ingress
                .admit(request.clone())
                .await
                .expect("deadline request admitted");
            let first = tokio::time::timeout(Duration::from_secs(1), first.wait())
                .await
                .expect("logical deadline is bounded")
                .expect("deadline terminal returned");
            assert_eq!(
                first.disposition,
                crate::proto::RelocalizeDisposition::DeadlineExceeded as i32
            );
            assert_eq!(first.candidate_sequence, 0);
            assert!(first.candidate_payload_sha256.is_empty());
            assert!(first.candidate_payload_preimage_v1.is_empty());
            assert!(
                first.evidence_disclosure_v1.len()
                    <= RELOCALIZATION_MAX_INLINE_EVIDENCE_DISCLOSURE_BYTES
            );
            assert_eq!(
                first.evidence_disclosure_v1
                    [RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1.len() + 4],
                0
            );
            assert_eq!(first.terminal_proof_sha256.len(), 32);

            let replay = ingress
                .admit(request)
                .await
                .expect("terminal replay admitted")
                .wait()
                .await
                .expect("terminal replay returned");
            assert_eq!(replay.encode_to_vec(), first.encode_to_vec());
            phase_a.begin();
            supervisor.shutdown().await.expect("retained actor joins");
        }

        #[tokio::test]
        async fn relocalization_retained_command_busy_is_transient_and_unproved() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut supervisor = relocalization_command_supervisor(phase_a.clone());
            let ingress = supervisor.ingress();
            let active = ingress
                .admit(relocalization_command_request("req-active", 7))
                .await
                .expect("active request admitted");
            let busy = ingress
                .admit(relocalization_command_request("req-busy", 7))
                .await
                .expect("busy request receives a waiter")
                .wait()
                .await
                .expect("busy response returned");
            assert_eq!(
                busy.disposition,
                crate::proto::RelocalizeDisposition::Busy as i32
            );
            assert!(busy.terminal_proof_sha256.is_empty());

            active.cancel_preclaim();
            let retried = ingress
                .admit(relocalization_command_request("req-busy", 7))
                .await
                .expect("busy request was not permanently inserted");
            phase_a.begin();
            let terminal = retried.wait().await.expect("new request is retained");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            supervisor.shutdown().await.expect("retained actor joins");
        }

        #[tokio::test]
        async fn relocalization_transport_owner_replays_target_unavailable_terminal() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 1,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "map".to_owned(),
                    odom_frame_id: "odom".to_owned(),
                    body_frame_id: "base_link".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("relocalization transport owner starts without private LCM");
            let ingress = owner
                .relocalization_ingress()
                .expect("relocalization command ingress is retained");
            assert!(ingress.is_open());
            let request = relocalization_command_request("req-target-unavailable", 7);
            let first = ingress
                .admit(request.clone())
                .await
                .expect("command admission remains typed")
                .wait()
                .await
                .expect("target-unavailable terminal is delivered");
            assert_eq!(
                first.disposition,
                crate::proto::RelocalizeDisposition::TargetUnavailable as i32
            );
            assert_eq!(first.candidate_sequence, 0);
            assert_eq!(first.terminal_proof_sha256.len(), 32);
            let replay = ingress
                .admit(request)
                .await
                .expect("terminal request is replayable")
                .wait()
                .await
                .expect("terminal replay is delivered");
            assert_eq!(replay.encode_to_vec(), first.encode_to_vec());
            owner.shutdown().await.expect("runtime owner joins cleanly");
        }

        fn install_relocalization_target_for_request(
            phase_a: &CorrectionPhaseAControl,
            request: &mut crate::proto::RelocalizeRequest,
            map_frame_id: &str,
        ) {
            let points = asymmetric_cloud();
            let mut xyz_f32_le = Vec::new();
            for point in points {
                for coordinate in [point.x, point.y, point.z] {
                    xyz_f32_le.extend_from_slice(&(coordinate as f32).to_le_bytes());
                }
            }
            let point_count = u32::try_from(xyz_f32_le.len() / 12).unwrap();
            let target_sha256 = sha256_bytes(&xyz_f32_le);
            request.target_cloud_sha256 = target_sha256.to_vec();
            let control = phase_a.activated_target_receipts();
            let seed = ActivationReceiptSeed {
                activation_request_id: "activation-relocalization-runtime".to_owned(),
                activation_request_identity_sha256: [4; 32],
                asset_manifest_sha256: Some([9; 32]),
                asset_bundle_sha256: Some([10; 32]),
                modern: true,
                immutable_target: Some(crate::pointcloud::ImmutableActivatedTarget {
                    frame_id: map_frame_id.to_owned(),
                    source_revision: request.target_cloud_revision,
                    point_count,
                    voxel_size: crate::pointcloud::VOXEL_SIZE,
                    xyz_f32_le,
                    xyz_sha256: target_sha256,
                    payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
                }),
            };
            let mut state = activation_base_state();
            state.session_id = request.session_id.clone();
            state.map_name = request.map_name.clone();
            state.session_generation = request.session_generation;
            let token = match control.stage_or_replay(seed, state.clone(), true) {
                ActivationLedgerAdmission::Execute(token) => token,
                ActivationLedgerAdmission::Terminal(_) => panic!("target must stage"),
            };
            let token = control
                .reserve_backend_generation(&token, request.backend_generation)
                .expect("backend generation binds");
            let token = control
                .bind_activation_identity(&token, [5; 32], request.session_generation)
                .expect("activation identity binds");
            assert!(control.store_publication_proof(
                &token,
                activation_publication_proof([6; 32], point_count, request.session_generation,),
            ));
            control
                .seal(&token, Some([6; 32]), state)
                .expect("target seals");
        }

        #[tokio::test]
        async fn relocalization_runtime_releases_non_cutoff_guard_and_keeps_first_deadline() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 2,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-runtime-deadline", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .expect("command ingress exists")
                .admit(request)
                .await
                .expect("command starts accumulating");
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;
            let disposition = owner
                .ingress()
                .submit_with_lifecycle_guard(active_relocalization_frame(&owner, 1, 10, 0.0), guard)
                .await
                .expect("non-cutoff frame is acknowledged");
            assert_eq!(disposition, CorrectionSubmitDisposition::NoCandidate);
            assert!(lifecycle.try_lock().is_ok());
            let terminal = tokio::time::timeout(Duration::from_secs(1), waiter.wait())
                .await
                .expect("first logical deadline remains bounded")
                .expect("deadline terminal is delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::DeadlineExceeded as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            owner.shutdown().await.expect("runtime owner joins");
        }

        #[tokio::test]
        async fn relocalization_timer_preempts_simultaneously_ready_cutoff_frame() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 14,
                },
            };
            let sink = Arc::new(RelocalizationAppliedSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-timer-frame-priority", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");

            let pattern = asymmetric_cloud();
            let mut body_points = Vec::with_capacity(RELOCALIZATION_MIN_AGGREGATE_POINTS);
            while body_points.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS {
                let remaining = RELOCALIZATION_MIN_AGGREGATE_POINTS - body_points.len();
                body_points.extend(pattern.iter().take(remaining).copied());
            }
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points = body_points;
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;

            std::thread::sleep(Duration::from_millis(
                RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS + 50,
            ));
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("simultaneously ready frame is acknowledged"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let terminal = waiter.wait().await.expect("deadline terminal is delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::DeadlineExceeded as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            assert_eq!(sink.staged.load(Ordering::Acquire), 0);
            let released =
                tokio::time::timeout(Duration::from_secs(1), Arc::clone(&lifecycle).lock_owned())
                    .await
                    .expect("non-selected frame guard is released");
            drop(released);
            owner.shutdown().await.expect("runtime owner joins");
        }

        #[derive(Default)]
        struct RelocalizationAppliedSink {
            staged: AtomicU64,
            applied: AtomicU64,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for RelocalizationAppliedSink {
            async fn stage_frame(&self, _input: &CorrectionFrameInput) -> Result<(), String> {
                self.staged.fetch_add(1, Ordering::AcqRel);
                Ok(())
            }

            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                self.applied.fetch_add(1, Ordering::AcqRel);
                Ok(CorrectionDisposition::Applied)
            }

            async fn canonical_correction_revision(&self) -> u64 {
                self.applied.load(Ordering::Acquire)
            }
        }

        #[derive(Default)]
        struct SequencedRelocalizationSink {
            stage_attempts: AtomicU64,
            apply_attempts: AtomicU64,
            applied: AtomicU64,
            discarded: AtomicU64,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for SequencedRelocalizationSink {
            async fn stage_frame(&self, _input: &CorrectionFrameInput) -> Result<(), String> {
                if self.stage_attempts.fetch_add(1, Ordering::AcqRel) == 0 {
                    Err("RELOCALIZATION_INJECTED_FIRST_STAGE_FAILURE".to_owned())
                } else {
                    Ok(())
                }
            }

            async fn discard_frame(&self, _input: &CorrectionFrameInput) {
                self.discarded.fetch_add(1, Ordering::AcqRel);
            }

            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                if self.apply_attempts.fetch_add(1, Ordering::AcqRel) == 0 {
                    self.applied.fetch_add(1, Ordering::AcqRel);
                    Ok(CorrectionDisposition::Applied)
                } else {
                    Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::Stopped,
                    ))
                }
            }

            async fn canonical_correction_revision(&self) -> u64 {
                self.applied.load(Ordering::Acquire)
            }
        }

        #[tokio::test]
        async fn relocalization_runtime_claims_once_and_settles_canonical_apply() {
            let captured = FinalizationCapturedLogs::default();
            let subscriber = tracing_subscriber::fmt()
                .without_time()
                .with_ansi(false)
                .with_target(false)
                .with_writer(captured.clone())
                .finish();
            let _default = tracing::subscriber::set_default(subscriber);
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 3,
                },
            };
            let sink = Arc::new(RelocalizationAppliedSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-runtime-applied", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .expect("command ingress exists")
                .admit(request.clone())
                .await
                .expect("command starts accumulating");
            let pattern = asymmetric_cloud();
            let mut body_points = Vec::with_capacity(RELOCALIZATION_MIN_AGGREGATE_POINTS);
            while body_points.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS {
                let remaining = RELOCALIZATION_MIN_AGGREGATE_POINTS - body_points.len();
                body_points.extend(pattern.iter().take(remaining).copied());
            }
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points = body_points;
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;
            let disposition = owner
                .ingress()
                .submit_with_lifecycle_guard(cutoff, guard)
                .await
                .expect("cutoff frame transfers its guard");
            assert_eq!(disposition, CorrectionSubmitDisposition::NoCandidate);
            assert!(lifecycle.try_lock().is_err());
            let terminal = tokio::time::timeout(Duration::from_secs(20), waiter.wait())
                .await
                .expect("retained compute and finalizer settle")
                .expect("canonical terminal is delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::Applied as i32
            );
            assert_eq!(terminal.candidate_sequence, 1);
            assert_eq!(terminal.candidate_payload_sha256.len(), 32);
            assert_eq!(
                sha256_bytes(&terminal.candidate_payload_preimage_v1),
                terminal.candidate_payload_sha256.as_slice()
            );
            assert!(
                terminal.candidate_payload_preimage_v1.len()
                    <= RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES
            );
            assert!(
                terminal.evidence_disclosure_v1.len()
                    <= RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES
            );
            assert_eq!(
                terminal.evidence_disclosure_v1
                    [RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1.len() + 4],
                1
            );
            assert_eq!(terminal.correction_revision, 1);
            assert_eq!(sink.staged.load(Ordering::Acquire), 1);
            assert_eq!(sink.applied.load(Ordering::Acquire), 1);
            assert!(lifecycle.try_lock().is_ok());
            let replay = owner
                .relocalization_ingress()
                .expect("command ingress remains open")
                .admit(request)
                .await
                .expect("applied request replays")
                .wait()
                .await
                .expect("applied replay is delivered");
            assert_eq!(replay.encode_to_vec(), terminal.encode_to_vec());
            owner.shutdown().await.expect("runtime owner joins");
            assert_eq!(
                captured
                    .snapshot()
                    .matches(CANDIDATE_FINALIZATION_EVENT_NAME)
                    .count(),
                0,
                "direct relocalization must not emit producer finalization events"
            );
        }

        #[tokio::test]
        async fn relocalization_canonical_rejection_is_typed_and_sequence_continues() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 13,
                },
            };
            let sink = Arc::new(SequencedRelocalizationSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-sequence-stage-failure", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let pattern = asymmetric_cloud();
            let mut body_points = Vec::with_capacity(RELOCALIZATION_MIN_AGGREGATE_POINTS);
            while body_points.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS {
                let remaining = RELOCALIZATION_MIN_AGGREGATE_POINTS - body_points.len();
                body_points.extend(pattern.iter().take(remaining).copied());
            }

            let first = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request.clone())
                .await
                .expect("first command accumulates");
            let mut first_cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            first_cutoff.body_points = body_points.clone();
            first_cutoff.registered_cloud.point_count = first_cutoff.body_points.len() as u64;
            first_cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&first_cutoff.body_points).unwrap();
            let first_guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(first_cutoff, first_guard)
                    .await
                    .expect("first cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let first = first.wait().await.expect("stage failure settles");
            assert_eq!(
                first.disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(first.candidate_sequence, 0);

            request.request_id = "req-sequence-applied".to_owned();
            let second = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request.clone())
                .await
                .expect("second command accumulates");
            let mut second_cutoff = active_relocalization_frame(&owner, 2, 20, 0.0);
            second_cutoff.body_points = body_points.clone();
            second_cutoff.registered_cloud.point_count = second_cutoff.body_points.len() as u64;
            second_cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&second_cutoff.body_points).unwrap();
            let second_guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(second_cutoff, second_guard)
                    .await
                    .expect("second cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let second = second.wait().await.expect("second command applies");
            assert_eq!(
                second.disposition,
                crate::proto::RelocalizeDisposition::Applied as i32
            );
            assert_eq!(second.candidate_sequence, 1);
            assert_eq!(second.correction_revision, 1);

            request.request_id = "req-sequence-rejected".to_owned();
            let third = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("third command accumulates");
            let mut third_cutoff = active_relocalization_frame(&owner, 3, 30, 0.0);
            third_cutoff.body_points = body_points;
            third_cutoff.registered_cloud.point_count = third_cutoff.body_points.len() as u64;
            third_cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&third_cutoff.body_points).unwrap();
            let third_guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(third_cutoff, third_guard)
                    .await
                    .expect("third cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let third = third.wait().await.expect("canonical rejection settles");
            assert_eq!(
                third.disposition,
                crate::proto::RelocalizeDisposition::Rejected as i32
            );
            assert_eq!(third.candidate_sequence, 2);
            assert_eq!(third.correction_revision, 1);
            assert_eq!(third.candidate_payload_sha256.len(), 32);
            assert_eq!(sink.stage_attempts.load(Ordering::Acquire), 3);
            assert_eq!(sink.apply_attempts.load(Ordering::Acquire), 2);
            assert_eq!(sink.applied.load(Ordering::Acquire), 1);
            owner.shutdown().await.expect("runtime owner joins");
            assert!(sink.discarded.load(Ordering::Acquire) >= 1);
        }

        #[derive(Default)]
        struct BlockingRelocalizationSink {
            staged: AtomicU64,
            applied: AtomicU64,
            apply_started: Notify,
            apply_release: Notify,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for BlockingRelocalizationSink {
            async fn stage_frame(&self, _input: &CorrectionFrameInput) -> Result<(), String> {
                self.staged.fetch_add(1, Ordering::AcqRel);
                Ok(())
            }

            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                self.applied.fetch_add(1, Ordering::AcqRel);
                self.apply_started.notify_one();
                self.apply_release.notified().await;
                Ok(CorrectionDisposition::Applied)
            }

            async fn canonical_correction_revision(&self) -> u64 {
                self.applied.load(Ordering::Acquire)
            }
        }

        #[tokio::test]
        async fn relocalization_postclaim_phase_a_and_response_loss_do_not_downgrade() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 4,
                },
            };
            let sink = Arc::new(BlockingRelocalizationSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-postclaim-phase-a", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let ingress = owner
                .relocalization_ingress()
                .expect("command ingress exists");
            let lost_response = ingress
                .admit(request.clone())
                .await
                .expect("first waiter starts command");
            let retained_response = ingress
                .admit(request)
                .await
                .expect("second waiter joins command");
            let pattern = asymmetric_cloud();
            let mut body_points = Vec::with_capacity(RELOCALIZATION_MIN_AGGREGATE_POINTS);
            while body_points.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS {
                let remaining = RELOCALIZATION_MIN_AGGREGATE_POINTS - body_points.len();
                body_points.extend(pattern.iter().take(remaining).copied());
            }
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points = body_points;
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            tokio::time::timeout(Duration::from_secs(20), sink.apply_started.notified())
                .await
                .expect("canonical apply reaches postclaim");
            drop(lost_response);
            phase_a.begin();
            sink.apply_release.notify_one();
            let terminal = tokio::time::timeout(Duration::from_secs(2), retained_response.wait())
                .await
                .expect("postclaim Phase A still settles")
                .expect("retained waiter receives canonical terminal");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::Applied as i32
            );
            assert_eq!(terminal.candidate_sequence, 1);
            assert_eq!(terminal.correction_revision, 1);
            assert_eq!(sink.staged.load(Ordering::Acquire), 1);
            assert_eq!(sink.applied.load(Ordering::Acquire), 1);
            owner
                .shutdown()
                .await
                .expect("postclaim owner joins cleanly");
        }

        #[tokio::test]
        async fn relocalization_full_finalizer_reserve_does_not_consume_sequence_or_stage() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 12,
                },
            };
            let sink = Arc::new(BlockingRelocalizationSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");

            let mut dummy_receivers = Vec::new();
            for (index, sequence) in [100_u64, 101].into_iter().enumerate() {
                let candidate = sample_candidate(sequence, sequence * 10);
                let key = CandidateApplyKey {
                    candidate_sequence: sequence,
                    payload_sha256: candidate.payload_sha256,
                };
                let lease = Arc::new(CorrectionRequestLease::new(
                    CorrectionRequestCancellation::new(),
                    None,
                ));
                dummy_receivers.push(
                    owner
                        .apply_waiters
                        .register(key, Arc::clone(&lease))
                        .expect("dummy waiter registers"),
                );
                let lease = owner
                    .apply_waiters
                    .claim_apply(key)
                    .expect("dummy waiter claims");
                let (outcome, _outcome_rx) = oneshot::channel();
                owner
                    .finalizer
                    .ingress
                    .sender
                    .send(CanonicalFinalizerJob {
                        key,
                        candidate,
                        lease,
                        release_lease_on_completion: true,
                        outcome,
                    })
                    .await
                    .expect("dummy finalizer job queues");
                if index == 0 {
                    tokio::time::timeout(Duration::from_secs(1), sink.apply_started.notified())
                        .await
                        .expect("first dummy job occupies finalizer worker");
                }
            }

            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-finalizer-full", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let pattern = asymmetric_cloud();
            let mut body_points = Vec::with_capacity(RELOCALIZATION_MIN_AGGREGATE_POINTS);
            while body_points.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS {
                let remaining = RELOCALIZATION_MIN_AGGREGATE_POINTS - body_points.len();
                body_points.extend(pattern.iter().take(remaining).copied());
            }
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points = body_points;
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            tokio::time::sleep(Duration::from_millis(500)).await;
            assert_eq!(sink.staged.load(Ordering::Acquire), 0);
            phase_a.begin();
            let terminal = waiter.wait().await.expect("Phase A terminal delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            sink.apply_release.notify_one();
            tokio::time::timeout(Duration::from_secs(1), sink.apply_started.notified())
                .await
                .expect("second dummy job is retained");
            sink.apply_release.notify_one();
            owner.shutdown().await.expect("all retained workers join");
            assert_eq!(sink.staged.load(Ordering::Acquire), 0);
            assert_eq!(sink.applied.load(Ordering::Acquire), 2);
            drop(dummy_receivers);
        }

        #[tokio::test]
        async fn relocalization_runtime_eight_small_frames_are_insufficient_without_staging() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 5,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-insufficient-frames", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            for sequence in 1..=RELOCALIZATION_MAX_FRAMES as u64 {
                let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
                let disposition = owner
                    .ingress()
                    .submit_with_lifecycle_guard(
                        active_relocalization_frame(&owner, sequence, sequence * 10, 0.0),
                        guard,
                    )
                    .await
                    .expect("small frame is acknowledged");
                assert_eq!(disposition, CorrectionSubmitDisposition::NoCandidate);
            }
            let terminal = tokio::time::timeout(Duration::from_secs(1), waiter.wait())
                .await
                .expect("max-frame cutoff is immediate")
                .expect("insufficient terminal is delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::InsufficientSource as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            owner.shutdown().await.expect("insufficient owner joins");
        }

        #[tokio::test]
        async fn relocalization_exact_maximum_without_fresh_guard_terminates_current_frame() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 19,
                },
            };
            let sink = Arc::new(RelocalizationAppliedSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-exact-aggregate-maximum", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            for (sequence, point_count) in [(1_u64, 100_000_usize), (2, 100_000), (3, 50_000)] {
                let mut input = active_relocalization_frame(&owner, sequence, sequence * 10, 0.0);
                input.body_points = vec![Point3::new(0.0, 0.0, 0.0); point_count];
                input.registered_cloud.point_count = point_count as u64;
                input.registered_cloud.body_points_sha256 =
                    body_points_sha256(&input.body_points).unwrap();
                assert_eq!(
                    owner
                        .ingress()
                        .submit(input)
                        .await
                        .expect("bounded frame is acknowledged"),
                    CorrectionSubmitDisposition::NoCandidate
                );
            }
            let terminal = waiter
                .wait()
                .await
                .expect("exact maximum terminates without another frame");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::InsufficientSource as i32
            );
            assert_eq!(
                terminal.detail,
                "RELOCALIZATION_MAXIMUM_AGGREGATE_FRESH_BOUNDARY_UNAVAILABLE"
            );
            assert_eq!(terminal.candidate_sequence, 0);
            assert_eq!(sink.staged.load(Ordering::Acquire), 0);
            owner.shutdown().await.expect("bounded owner joins");
        }

        #[tokio::test]
        async fn relocalization_runtime_algorithm_reject_releases_cutoff_guard() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 6,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-algorithm-reject", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points =
                vec![Point3::new(0.0, 0.0, 0.0); RELOCALIZATION_MIN_AGGREGATE_POINTS];
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let terminal = tokio::time::timeout(Duration::from_secs(5), waiter.wait())
                .await
                .expect("retained rejected compute settles")
                .expect("algorithm terminal is delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::AlgorithmRejected as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            assert!(terminal.candidate_payload_sha256.is_empty());
            assert!(terminal.candidate_payload_preimage_v1.is_empty());
            assert_eq!(terminal.evidence_sha256.len(), 32);
            assert!(
                terminal.evidence_disclosure_v1.len()
                    <= RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES
            );
            assert_eq!(
                terminal.evidence_disclosure_v1
                    [RELOCALIZATION_EVIDENCE_DISCLOSURE_DOMAIN_V1.len() + 4],
                1
            );
            assert!(lifecycle.try_lock().is_ok());
            owner
                .shutdown()
                .await
                .expect("rejected compute owner joins");
        }

        #[derive(Default)]
        struct FailingRelocalizationStageSink {
            staged: AtomicU64,
            discarded: AtomicU64,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for FailingRelocalizationStageSink {
            async fn stage_frame(&self, _input: &CorrectionFrameInput) -> Result<(), String> {
                self.staged.fetch_add(1, Ordering::AcqRel);
                Err("RELOCALIZATION_INJECTED_STAGE_FAILURE".to_owned())
            }

            async fn discard_frame(&self, _input: &CorrectionFrameInput) {
                self.discarded.fetch_add(1, Ordering::AcqRel);
            }

            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                panic!("stage failure cannot reach canonical apply")
            }
        }

        #[tokio::test]
        async fn relocalization_stage_failure_is_terminal_and_retained_discard_joins() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 7,
                },
            };
            let sink = Arc::new(FailingRelocalizationStageSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-stage-failure", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let pattern = asymmetric_cloud();
            let mut body_points = Vec::with_capacity(RELOCALIZATION_MIN_AGGREGATE_POINTS);
            while body_points.len() < RELOCALIZATION_MIN_AGGREGATE_POINTS {
                let remaining = RELOCALIZATION_MIN_AGGREGATE_POINTS - body_points.len();
                body_points.extend(pattern.iter().take(remaining).copied());
            }
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points = body_points;
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let terminal = tokio::time::timeout(Duration::from_secs(5), waiter.wait())
                .await
                .expect("stage failure settles")
                .expect("stage terminal delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            owner
                .shutdown()
                .await
                .expect("cleanup worker drains and joins");
            assert_eq!(sink.staged.load(Ordering::Acquire), 1);
            assert_eq!(sink.discarded.load(Ordering::Acquire), 1);
        }

        #[tokio::test]
        async fn relocalization_target_rollover_is_typed_preclaim_terminal() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 8,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-target-rollover", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command snapshots sealed target");
            phase_a
                .activated_target_receipts()
                .retire_for_mapping_change()
                .expect("target lifecycle advances");
            let terminal = tokio::time::timeout(Duration::from_secs(1), waiter.wait())
                .await
                .expect("target poll is bounded")
                .expect("target rollover terminal delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::TargetInvalidated as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            owner.shutdown().await.expect("rollover owner joins");
        }

        #[tokio::test]
        async fn relocalization_session_rollover_is_typed_preclaim_terminal() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 15,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-session-rollover", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command snapshots the current session");
            let mut rollover = active_relocalization_frame(&owner, 1, 10, 0.0);
            rollover.mapping_session_id = "mapping-session-next".to_owned();
            rollover.mapping_session_generation = 24;
            assert_eq!(
                owner
                    .ingress()
                    .submit(rollover)
                    .await
                    .expect("rollover frame is acknowledged"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let terminal = waiter.wait().await.expect("session terminal is delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::SessionInvalidated as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            assert_eq!(terminal.detail, "RELOCALIZATION_FRAME_SESSION_INVALIDATED");
            owner.shutdown().await.expect("rollover owner joins");
        }

        #[tokio::test]
        async fn relocalization_runtime_duplicate_is_ignored_and_same_key_conflict_fail_closes() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 9,
                },
            };
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-frame-proof-conflict", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command activates frame proof ledger");
            let first = active_relocalization_frame(&owner, 1, 10, 0.0);
            assert_eq!(
                owner.ingress().submit(first.clone()).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert_eq!(
                owner.ingress().submit(first.clone()).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            let mut conflict = first;
            conflict.body_points[0].x += 0.25;
            conflict.registered_cloud.body_points_sha256 =
                body_points_sha256(&conflict.body_points).unwrap();
            let rejected = owner
                .ingress()
                .submit(conflict)
                .await
                .expect("integrity conflict has a typed frame response");
            match rejected {
                CorrectionSubmitDisposition::Rejected { reason, .. } => {
                    assert_eq!(reason, "RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT");
                }
                other => panic!("same-key conflict must reject, got {other:?}"),
            }
            let command_terminal = waiter
                .wait()
                .await
                .expect("proof conflict stores typed command terminal first");
            assert_eq!(
                command_terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(
                command_terminal.detail,
                "RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT"
            );
            let terminal = owner
                .wait_terminal()
                .await
                .expect_err("integrity failure is retained owner terminal");
            assert!(terminal.contains("RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT"));
            let settlement = owner
                .shutdown()
                .await
                .expect("integrity failure is sticky after owner settlement");
            assert!(settlement
                .into_sticky_error()
                .expect("integrity terminal remains sticky")
                .contains("RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT"));
        }

        #[tokio::test]
        async fn relocalization_idle_frames_do_not_consume_proof_capacity() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 17,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            for sequence in 1..=RELOCALIZATION_MAX_FRAME_PROOFS_TOTAL as u64 + 2 {
                assert_eq!(
                    owner
                        .ingress()
                        .submit(frame(sequence, sequence * 10, 0.0))
                        .await
                        .expect("idle frame is ignored"),
                    CorrectionSubmitDisposition::NoCandidate
                );
            }
            owner.shutdown().await.expect("idle owner joins cleanly");
        }

        #[tokio::test]
        async fn relocalization_active_frame_proof_capacity_is_typed_fail_close() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 18,
                },
            };
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-frame-proof-capacity", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command activates proof ledger");
            assert_eq!(
                owner
                    .ingress()
                    .submit(active_relocalization_frame(&owner, 100, 1_000, 0.0))
                    .await
                    .unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            for sequence in 1..RELOCALIZATION_MAX_FRAME_PROOFS_PER_SCOPE as u64 {
                assert_eq!(
                    owner
                        .ingress()
                        .submit(active_relocalization_frame(
                            &owner,
                            sequence,
                            sequence * 10,
                            0.0,
                        ))
                        .await
                        .expect("bounded active proof is recorded"),
                    CorrectionSubmitDisposition::NoCandidate
                );
            }
            let rejected = owner
                .ingress()
                .submit(active_relocalization_frame(
                    &owner,
                    RELOCALIZATION_MAX_FRAME_PROOFS_PER_SCOPE as u64,
                    RELOCALIZATION_MAX_FRAME_PROOFS_PER_SCOPE as u64 * 10,
                    0.0,
                ))
                .await
                .expect("proof capacity returns a typed frame disposition");
            assert!(matches!(
                rejected,
                CorrectionSubmitDisposition::Rejected {
                    ref reason,
                    ..
                } if reason == "RELOCALIZATION_FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED"
            ));
            let command_terminal = waiter.wait().await.expect("capacity terminal is stored");
            assert_eq!(
                command_terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(
                command_terminal.detail,
                "RELOCALIZATION_FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED"
            );
            assert!(owner.wait_terminal().await.is_err());
            let settlement = owner
                .shutdown()
                .await
                .expect("proof-capacity failure settles the owner");
            assert!(settlement
                .into_sticky_error()
                .is_some_and(|error| error.contains("FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED")));
        }

        #[tokio::test]
        async fn relocalization_phase_a_joins_late_blocking_compute_before_guard_release() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 10,
                },
            };
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            owner.set_relocalization_compute_delay_for_tests(Duration::from_millis(200));
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-late-compute", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points =
                vec![Point3::new(0.0, 0.0, 0.0); RELOCALIZATION_MIN_AGGREGATE_POINTS];
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            phase_a.begin();
            let terminal = tokio::time::timeout(Duration::from_secs(1), waiter.wait())
                .await
                .expect("Phase A terminal is immediate")
                .expect("Phase A terminal delivered");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            assert!(lifecycle.try_lock().is_err());
            let shutdown_started = Instant::now();
            owner
                .shutdown()
                .await
                .expect("late blocking compute is joined");
            assert!(shutdown_started.elapsed() >= Duration::from_millis(100));
            assert!(lifecycle.try_lock().is_ok());
        }

        #[tokio::test]
        async fn relocalization_repeated_phase_a_is_idempotent_and_leaves_no_retained_work() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 16,
                },
            };
            let sink = Arc::new(RelocalizationAppliedSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                sink.clone(),
            )
            .await
            .expect("runtime owner starts");
            owner.set_relocalization_compute_delay_for_tests(Duration::from_millis(100));
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-repeated-phase-a", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points =
                vec![Point3::new(0.0, 0.0, 0.0); RELOCALIZATION_MIN_AGGREGATE_POINTS];
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );

            let mut closers = Vec::new();
            for _ in 0..8 {
                let closer = phase_a.clone();
                closers.push(tokio::spawn(async move {
                    closer.begin();
                }));
            }
            for closer in closers {
                closer.await.expect("Phase A closer joins");
            }
            phase_a.begin();
            let terminal = waiter.wait().await.expect("one terminal is retained");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            assert_eq!(terminal.candidate_sequence, 0);
            assert_eq!(sink.staged.load(Ordering::Acquire), 0);
            tokio::time::timeout(Duration::from_secs(2), owner.shutdown())
                .await
                .expect("repeated Phase A leaves a bounded owner join")
                .expect("owner joins cleanly");
            let released =
                tokio::time::timeout(Duration::from_secs(1), Arc::clone(&lifecycle).lock_owned())
                    .await
                    .expect("retained compute guard is released");
            drop(released);
        }

        #[tokio::test]
        async fn relocalization_compute_panic_stores_terminal_before_owner_failure() {
            let owner_identity = CorrectionOwnerRuntimeIdentity {
                runtime_generation: 2,
                backend_generation: 99,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 11,
                },
            };
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity,
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("runtime owner starts");
            owner.set_relocalization_compute_panic_for_tests(true);
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request("req-compute-panic", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            request.logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let mut cutoff = active_relocalization_frame(&owner, 1, 10, 0.0);
            cutoff.body_points =
                vec![Point3::new(0.0, 0.0, 0.0); RELOCALIZATION_MIN_AGGREGATE_POINTS];
            cutoff.registered_cloud.point_count = cutoff.body_points.len() as u64;
            cutoff.registered_cloud.body_points_sha256 =
                body_points_sha256(&cutoff.body_points).unwrap();
            let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(
                owner
                    .ingress()
                    .submit_with_lifecycle_guard(cutoff, guard)
                    .await
                    .expect("cutoff transfers"),
                CorrectionSubmitDisposition::NoCandidate
            );
            let terminal = tokio::time::timeout(Duration::from_secs(2), waiter.wait())
                .await
                .expect("panic terminal is bounded")
                .expect("panic terminal is stored before actor exit");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(terminal.detail, "RELOCALIZATION_RETAINED_TASK_FAILED");
            assert_eq!(terminal.candidate_sequence, 0);
            let owner_error = owner
                .wait_terminal()
                .await
                .expect_err("retained task panic fails the owner");
            assert!(owner_error.contains("relocalization retained task join failed"));
            let settlement = owner
                .shutdown()
                .await
                .expect("retained task panic settles with a sticky failure");
            assert!(settlement
                .into_sticky_error()
                .is_some_and(|error| error.contains("relocalization retained task join failed")));
        }

        #[tokio::test]
        async fn relocalization_source_seal_uses_each_frame_odom_and_retains_boundary_guard() {
            let target_points = [
                [0.0_f32, 0.0, 0.0],
                [1.0_f32, 0.0, 0.0],
                [0.0_f32, 1.0, 0.0],
            ];
            let mut target_bytes = Vec::new();
            for point in target_points {
                for coordinate in point {
                    target_bytes.extend_from_slice(&coordinate.to_le_bytes());
                }
            }
            let target = crate::pointcloud::ImmutableActivatedTarget {
                frame_id: "map".to_owned(),
                source_revision: 7,
                point_count: 3,
                voxel_size: crate::pointcloud::VOXEL_SIZE,
                xyz_sha256: sha256_bytes(&target_bytes),
                xyz_f32_le: target_bytes,
                payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
            };
            let target_receipt_sha256 = [6; 32];
            let snapshot = SealedActivatedTargetSnapshot {
                runtime_generation: 2,
                lifecycle_epoch: 1,
                session_id: "session-ledger".to_owned(),
                map_name: "map-ledger".to_owned(),
                activation_request_id: "activation-source".to_owned(),
                activation_request_identity_sha256: [4; 32],
                activation_identity_sha256: [5; 32],
                target_receipt_sha256,
                asset_manifest_sha256: Some([9; 32]),
                asset_bundle_sha256: Some([10; 32]),
                session_generation: 3,
                backend_generation: 4,
                publication_proof: activation_publication_proof(target_receipt_sha256, 3, 3),
                immutable_target: target,
            };
            let mut first = frame(1, 10, 0.0);
            first.correlated_odom.odom_from_body = Isometry3::translation(1.0, 0.0, 0.0);
            first.body_points = vec![Point3::new(0.0, 0.0, 0.0); 25_000];
            let mut second = frame(2, 20, 0.0);
            second.correlated_odom.odom_from_body = Isometry3::translation(0.0, 2.0, 0.0);
            second.body_points = vec![Point3::new(0.0, 0.0, 0.0); 25_000];
            let request =
                validate_relocalize_request(relocalization_command_request("req-source-seal", 7))
                    .expect("request fixture is canonical");
            let accumulation = RelocalizationAccumulation {
                request,
                admission_ordinal: 1,
                mode_epoch: 1,
                logical_deadline: Instant::now() + Duration::from_secs(1),
                accumulation_deadline: Instant::now() + Duration::from_secs(1),
                snapshot,
                arm_watermark: None,
                armed_map_watermarks: HashMap::new(),
                accepted: vec![
                    AccumulatedRelocalizationFrame {
                        evidence_sha256: [1; 32],
                        input: first,
                    },
                    AccumulatedRelocalizationFrame {
                        evidence_sha256: [2; 32],
                        input: second.clone(),
                    },
                ],
                aggregate_input_points: 50_000,
                session_map_frame_generation: Some(second.map_frame_generation),
                mapping_commit_boot_nonce: Some(second.mapping_commit_id.boot_nonce),
            };
            let lifecycle = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            let lease = Arc::new(CorrectionRequestLease::new(
                CorrectionRequestCancellation::new(),
                Some(lifecycle),
            ));
            let sealed = seal_relocalization_source(
                accumulation,
                second,
                Arc::clone(&lease),
                RelocalizationCutoffReasonV1::MinimumAggregateReached,
            )
            .expect("source seals deterministically");
            assert!(sealed.boundary_lease.has_lifecycle_guard());
            assert_eq!(sealed.source_points.len(), 50_000);
            assert_eq!(sealed.source_points[0], Point3::new(1.0, 0.0, 0.0));
            assert_eq!(sealed.source_points[25_000], Point3::new(0.0, 2.0, 0.0));
            assert_eq!(sealed.target_points.len(), 3);
        }

        #[tokio::test]
        async fn relocalization_source_hard_bounds_fail_closed() {
            assert_eq!(
                RELOCALIZATION_MAX_PER_FRAME_F64_BYTES,
                MAX_REGISTERED_CLOUD_POINTS as usize * 3 * std::mem::size_of::<f64>()
            );
            assert_eq!(
                RELOCALIZATION_MAX_AGGREGATE_F64_BYTES,
                RELOCALIZATION_MAX_AGGREGATE_POINTS * 3 * std::mem::size_of::<f64>()
            );

            let mut at_frame_limit = frame(1, 10, 0.0);
            at_frame_limit.body_points =
                vec![Point3::new(0.0, 0.0, 0.0); MAX_REGISTERED_CLOUD_POINTS as usize];
            at_frame_limit.registered_cloud.point_count =
                u64::try_from(at_frame_limit.body_points.len()).unwrap();
            at_frame_limit.registered_cloud.body_points_sha256 =
                body_points_sha256(&at_frame_limit.body_points).unwrap();
            assert!(relocalization_frame_evidence(
                &owner_identity(),
                "robot_map",
                "robot_odom",
                "robot_body",
                &at_frame_limit,
            )
            .is_ok());

            let mut above_frame_limit = frame(2, 20, 0.0);
            above_frame_limit.body_points =
                vec![Point3::new(0.0, 0.0, 0.0); MAX_REGISTERED_CLOUD_POINTS as usize + 1];
            above_frame_limit.registered_cloud.point_count =
                u64::try_from(above_frame_limit.body_points.len()).unwrap();
            above_frame_limit.registered_cloud.body_points_sha256 = [1; 32];
            assert!(relocalization_frame_evidence(
                &owner_identity(),
                "robot_map",
                "robot_odom",
                "robot_body",
                &above_frame_limit,
            )
            .is_err());

            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut request = relocalization_command_request("req-aggregate-hard-bound", 7);
            request.session_id = "mapping-session-owner".to_owned();
            request.session_generation = 23;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let snapshot = phase_a
                .activated_target_receipts()
                .current_sealed_target_snapshot()
                .expect("sealed target snapshot");
            let request = validate_relocalize_request(request).expect("request is canonical");
            let mut boundary = frame(3, 30, 0.0);
            boundary.body_points = vec![Point3::new(0.0, 0.0, 0.0)];
            boundary.registered_cloud.point_count = 1;
            boundary.registered_cloud.body_points_sha256 =
                body_points_sha256(&boundary.body_points).unwrap();
            let accumulation = RelocalizationAccumulation {
                request,
                admission_ordinal: 1,
                mode_epoch: 1,
                logical_deadline: Instant::now() + Duration::from_secs(1),
                accumulation_deadline: Instant::now() + Duration::from_secs(1),
                snapshot,
                arm_watermark: None,
                armed_map_watermarks: HashMap::new(),
                accepted: vec![AccumulatedRelocalizationFrame {
                    evidence_sha256: [2; 32],
                    input: boundary.clone(),
                }],
                aggregate_input_points: RELOCALIZATION_MAX_AGGREGATE_POINTS + 1,
                session_map_frame_generation: Some(boundary.map_frame_generation),
                mapping_commit_boot_nonce: Some(boundary.mapping_commit_id.boot_nonce),
            };
            let lifecycle = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            let lease = Arc::new(CorrectionRequestLease::new(
                CorrectionRequestCancellation::new(),
                Some(lifecycle),
            ));
            assert_eq!(
                seal_relocalization_source(
                    accumulation,
                    boundary,
                    lease,
                    RelocalizationCutoffReasonV1::MinimumAggregateReached,
                )
                .err()
                .expect("aggregate byte bound must fail closed"),
                "RELOCALIZATION_AGGREGATE_BYTES_EXCEEDED"
            );
        }

        #[test]
        fn relocalization_stable_stratified_source_has_fixed_indices() {
            let ordered = (0..100_001)
                .map(|index| Point3::new(index as f64, 0.0, 0.0))
                .collect::<Vec<_>>();
            let (emitted, canonical, digest) =
                stable_stratified_source(ordered).expect("bounded source reduces");
            assert_eq!(emitted.len(), 100_000);
            assert_eq!(emitted[0].x, 0.0);
            assert_eq!(emitted[50_000].x, 50_000.0);
            assert_eq!(emitted[99_999].x, 99_999.0);
            assert_eq!(canonical.len(), 8 + 100_000 * 24);
            assert_eq!(digest, sha256_bytes(&canonical));
        }

        #[test]
        fn relocalization_frame_proof_ledger_is_scoped_and_conflict_permanent() {
            let mut actor = RelocalizationCommandActor::new(
                owner_identity(),
                "robot_map".to_owned(),
                "robot_odom".to_owned(),
                "robot_body".to_owned(),
            );
            let input = frame(1, 10, 0.0);
            let key = relocalization_frame_key(&input);
            assert!(!actor.record_frame_proof(&input, key, [1; 32]).unwrap());
            assert!(actor.record_frame_proof(&input, key, [1; 32]).unwrap());
            assert_eq!(
                actor.record_frame_proof(&input, key, [2; 32]),
                Err("RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT")
            );

            let mut next_session = input;
            next_session.mapping_session_id = "mapping-session-next".to_owned();
            next_session.mapping_session_generation += 1;
            assert!(!actor
                .record_frame_proof(&next_session, key, [2; 32])
                .unwrap());
        }

        fn activation_seed(request_id: &str, identity_byte: u8) -> ActivationReceiptSeed {
            ActivationReceiptSeed {
                activation_request_id: request_id.to_owned(),
                activation_request_identity_sha256: [identity_byte; 32],
                asset_manifest_sha256: Some([2; 32]),
                asset_bundle_sha256: Some([3; 32]),
                modern: true,
                immutable_target: None,
            }
        }

        fn activation_seed_with_target(
            request_id: &str,
            identity_byte: u8,
            point_count: u32,
        ) -> ActivationReceiptSeed {
            let xyz_f32_le = vec![identity_byte; point_count as usize * 12];
            let mut seed = activation_seed(request_id, identity_byte);
            seed.immutable_target = Some(crate::pointcloud::ImmutableActivatedTarget {
                frame_id: "map".to_owned(),
                source_revision: u64::from(identity_byte) + 1,
                point_count,
                voxel_size: crate::pointcloud::VOXEL_SIZE,
                xyz_sha256: crate::map_checksum::sha256_bytes(&xyz_f32_le),
                xyz_f32_le,
                payload_schema_version: crate::pointcloud::CM_PAYLOAD_SCHEMA_VERSION,
            });
            seed
        }

        fn activation_base_state() -> crate::proto::MappingSessionState {
            crate::proto::MappingSessionState {
                success: true,
                phase: "ACTIVE".to_owned(),
                message: "fixture".to_owned(),
                session_id: "session".to_owned(),
                map_name: "map".to_owned(),
                session_generation: 11,
                revision: 7,
                active: true,
                ..crate::proto::MappingSessionState::default()
            }
        }

        fn activation_publication_proof(
            target_receipt_sha256: [u8; 32],
            point_count: u32,
            session_generation: u64,
        ) -> ActivationPublicationProof {
            ActivationPublicationProof {
                source_timestamp_ns: 17,
                cloud_revision: 19,
                point_count,
                voxel_size: crate::pointcloud::VOXEL_SIZE,
                cloud_receipt: crate::mapping_commit::CloudCommitReceipt {
                    commit_id: crate::mapping_commit::MappingCommitId {
                        boot_nonce: 23,
                        sequence: 29,
                    },
                    session_generation,
                    occupancy_revision: 13,
                    cloud_revision: 19,
                    source_timestamp_ns: 17,
                    point_count,
                },
                target_receipt_sha256,
            }
        }

        #[test]
        fn activation_ledger_distinguishes_2d_from_lost_3d_target() {
            for has_target in [false, true] {
                let control = ActivatedTargetReceiptControl::new(7);
                let seed = if has_target {
                    activation_seed_with_target("act-1", 1, 1)
                } else {
                    activation_seed("act-1", 1)
                };
                let token = match control.stage_or_replay(seed, activation_base_state(), true) {
                    ActivationLedgerAdmission::Execute(token) => token,
                    _ => panic!("first admission must execute"),
                };
                let token = control.reserve_backend_generation(&token, 13).unwrap();
                let token = control.bind_activation_identity(&token, [4; 32], 11).unwrap();
                if has_target {
                    control.with_staged_record_mut(&token, |record| record.immutable_target = None);
                }
                let sealed = control.seal(&token, None, activation_base_state());
                if has_target {
                    assert!(sealed.is_none(), "丢失三维目标不能降级为二维激活成功");
                } else {
                    let sealed = sealed.expect("二维资产可封存独立激活身份");
                    assert!(sealed.success);
                    assert!(!sealed.relocalization_target_available);
                    assert!(sealed.target_receipt_sha256.is_empty());
                }
            }
        }

        #[test]
        fn activation_ledger_seals_replays_conflicts_and_permanently_retires() {
            let control = ActivatedTargetReceiptControl::new(7);
            let seed = activation_seed_with_target("act-1", 1, 1);
            let token = match control.stage_or_replay(seed.clone(), activation_base_state(), true) {
                ActivationLedgerAdmission::Execute(token) => token,
                ActivationLedgerAdmission::Terminal(_) => panic!("first admission must execute"),
            };
            let staged_replay =
                match control.stage_or_replay(seed.clone(), activation_base_state(), false) {
                    ActivationLedgerAdmission::Execute(token) => token,
                    ActivationLedgerAdmission::Terminal(_) => panic!("staged replay must resume"),
                };
            assert_eq!(staged_replay, token);

            let token = control.reserve_backend_generation(&token, 13).unwrap();
            let token = control
                .bind_activation_identity(&token, [4; 32], 11)
                .unwrap();
            assert!(control
                .store_publication_proof(&token, activation_publication_proof([5; 32], 1, 11),));
            let sealed = control
                .seal(&token, Some([5; 32]), activation_base_state())
                .unwrap();
            assert!(sealed.success);
            assert!(sealed.relocalization_target_available);
            assert_eq!(
                sealed.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Sealed as i32
            );

            let replay = match control.stage_or_replay(seed.clone(), activation_base_state(), false)
            {
                ActivationLedgerAdmission::Terminal(state) => state,
                ActivationLedgerAdmission::Execute(_) => panic!("sealed replay cannot execute"),
            };
            assert!(replay.success);
            assert!(replay.idempotent_replay);
            assert!(replay.relocalization_target_available);
            assert_eq!(
                replay.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::SealedReplay as i32
            );

            let conflict = match control.stage_or_replay(
                activation_seed("act-1", 9),
                activation_base_state(),
                false,
            ) {
                ActivationLedgerAdmission::Terminal(state) => state,
                ActivationLedgerAdmission::Execute(_) => panic!("conflict cannot execute"),
            };
            assert!(!conflict.success);
            assert_eq!(
                conflict.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Conflict as i32
            );

            let epoch = control.lifecycle_epoch();
            assert_eq!(control.retire_for_mapping_change().unwrap(), epoch + 1);
            let retired = match control.stage_or_replay(seed, activation_base_state(), true) {
                ActivationLedgerAdmission::Terminal(state) => state,
                ActivationLedgerAdmission::Execute(_) => {
                    panic!("permanent retired identity cannot restage")
                }
            };
            assert!(!retired.success);
            assert!(!retired.relocalization_target_available);
            assert_eq!(
                retired.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Retired as i32
            );
        }

        #[test]
        fn activation_terminal_records_release_all_immutable_target_bytes() {
            let control = ActivatedTargetReceiptControl::new(7);
            let mut seeds = Vec::new();
            for index in 0_u8..12 {
                let request_id = format!("target-release-{index}");
                let point_count = u32::from(index) + 1;
                let expected_bytes = point_count as usize * 12;
                let seed = activation_seed_with_target(&request_id, index + 1, point_count);
                seeds.push(seed.clone());
                let token =
                    match control.stage_or_replay(seed.clone(), activation_base_state(), true) {
                        ActivationLedgerAdmission::Execute(token) => token,
                        ActivationLedgerAdmission::Terminal(_) => {
                            panic!("new target identity must stage")
                        }
                    };
                assert_eq!(control.retained_immutable_target_bytes(), expected_bytes);
                for prior_request_id in &seeds[..seeds.len() - 1] {
                    assert_eq!(
                        control.retained_immutable_target_bytes_for(
                            &prior_request_id.activation_request_id,
                        ),
                        0,
                        "new current must release prior target {}",
                        prior_request_id.activation_request_id
                    );
                }
                let token = control
                    .reserve_backend_generation(&token, u64::from(index) + 20)
                    .unwrap();
                let token = control
                    .bind_activation_identity(&token, [index + 20; 32], u64::from(index) + 30)
                    .unwrap();
                let sealed = (index + 1) % 4 == 0;
                match (index + 1) % 4 {
                    0 => {
                        let target_receipt_sha256 = [index + 40; 32];
                        assert!(control.store_publication_proof(
                            &token,
                            activation_publication_proof(
                                target_receipt_sha256,
                                point_count,
                                u64::from(index) + 30,
                            ),
                        ));
                        control
                            .seal(&token, Some(target_receipt_sha256), activation_base_state())
                            .expect("seal target identity");
                    }
                    1 => {
                        control.invalidate(
                            &token,
                            crate::proto::ActivationReceiptDisposition::InvalidatedPrecommit,
                            "typed precommit terminal",
                            activation_base_state(),
                        );
                    }
                    2 => {
                        control.invalidate(
                            &token,
                            crate::proto::ActivationReceiptDisposition::OutcomeUnknown,
                            "typed outcome-unknown terminal",
                            activation_base_state(),
                        );
                    }
                    _ => {
                        control.retire_for_mapping_change().unwrap();
                    }
                }
                assert_eq!(
                    control.retained_immutable_target_bytes(),
                    if sealed { expected_bytes } else { 0 },
                    "only a current sealed identity may retain raw target bytes at index={index}"
                );
                let replay = match control.stage_or_replay(seed, activation_base_state(), true) {
                    ActivationLedgerAdmission::Terminal(state) => state,
                    ActivationLedgerAdmission::Execute(_) => {
                        panic!("terminal target identity must never restage")
                    }
                };
                assert_eq!(
                    replay.activation_receipt_disposition,
                    if sealed {
                        crate::proto::ActivationReceiptDisposition::SealedReplay as i32
                    } else {
                        crate::proto::ActivationReceiptDisposition::Retired as i32
                    }
                );
                assert_eq!(replay.relocalization_target_available, sealed);
            }
            let current = seeds.last().expect("twelve activation seeds");
            let current_bytes = current
                .immutable_target
                .as_ref()
                .expect("current target")
                .xyz_f32_le
                .len();
            assert_eq!(control.retained_immutable_target_bytes(), current_bytes);
            for seed in &seeds[..seeds.len() - 1] {
                assert_eq!(
                    control.retained_immutable_target_bytes_for(&seed.activation_request_id),
                    0
                );
                let replay =
                    match control.stage_or_replay(seed.clone(), activation_base_state(), true) {
                        ActivationLedgerAdmission::Terminal(state) => state,
                        ActivationLedgerAdmission::Execute(_) => {
                            panic!("replaced identity cannot execute")
                        }
                    };
                assert_eq!(
                    replay.activation_receipt_disposition,
                    crate::proto::ActivationReceiptDisposition::Retired as i32
                );
                assert!(!replay.relocalization_target_available);
            }
            assert_eq!(
                control.retained_immutable_target_bytes_for(&current.activation_request_id),
                current_bytes
            );
        }

        #[test]
        fn current_sealed_target_snapshot_is_atomic_and_replacement_releases_old_bytes() {
            let control = ActivatedTargetReceiptControl::new(7);
            let first_seed = activation_seed_with_target("sealed-snapshot-first", 61, 2);
            let first_target = first_seed
                .immutable_target
                .clone()
                .expect("first immutable target");
            let first_token =
                match control.stage_or_replay(first_seed.clone(), activation_base_state(), true) {
                    ActivationLedgerAdmission::Execute(token) => token,
                    ActivationLedgerAdmission::Terminal(_) => panic!("first target must stage"),
                };
            let first_token = control
                .reserve_backend_generation(&first_token, 43)
                .unwrap();
            let first_token = control
                .bind_activation_identity(&first_token, [62; 32], 47)
                .unwrap();
            assert!(control.store_publication_proof(
                &first_token,
                activation_publication_proof([63; 32], 2, 47),
            ));
            control
                .seal(&first_token, Some([63; 32]), activation_base_state())
                .expect("first target seal");

            let snapshot = control
                .current_sealed_target_snapshot()
                .expect("current sealed target snapshot");
            assert_eq!(snapshot.runtime_generation, 7);
            assert_eq!(snapshot.lifecycle_epoch, control.lifecycle_epoch());
            assert_eq!(snapshot.activation_request_id, "sealed-snapshot-first");
            assert_eq!(snapshot.activation_request_identity_sha256, [61; 32]);
            assert_eq!(snapshot.activation_identity_sha256, [62; 32]);
            assert_eq!(snapshot.target_receipt_sha256, [63; 32]);
            assert_eq!(snapshot.asset_manifest_sha256, Some([2; 32]));
            assert_eq!(snapshot.asset_bundle_sha256, Some([3; 32]));
            assert_eq!(snapshot.session_generation, 47);
            assert_eq!(snapshot.backend_generation, 43);
            assert_eq!(snapshot.publication_proof.target_receipt_sha256, [63; 32]);
            assert_eq!(snapshot.immutable_target, first_target);

            let mut detached_snapshot = snapshot.clone();
            detached_snapshot.immutable_target.xyz_f32_le[0] ^= 0xff;
            assert_eq!(
                control
                    .current_sealed_target_snapshot()
                    .expect("ledger snapshot remains immutable")
                    .immutable_target,
                first_target
            );
            drop(detached_snapshot);
            drop(snapshot);

            let second_seed = activation_seed_with_target("sealed-snapshot-second", 71, 3);
            let second_target = second_seed
                .immutable_target
                .clone()
                .expect("second immutable target");
            let second_token =
                match control.stage_or_replay(second_seed.clone(), activation_base_state(), true) {
                    ActivationLedgerAdmission::Execute(token) => token,
                    ActivationLedgerAdmission::Terminal(_) => panic!("second target must stage"),
                };
            assert!(control.current_sealed_target_snapshot().is_none());
            assert_eq!(
                control.retained_immutable_target_bytes_for("sealed-snapshot-first"),
                0
            );
            assert_eq!(
                control.retained_immutable_target_bytes(),
                second_target.xyz_f32_le.len()
            );
            let first_replay =
                match control.stage_or_replay(first_seed, activation_base_state(), true) {
                    ActivationLedgerAdmission::Terminal(state) => state,
                    ActivationLedgerAdmission::Execute(_) => {
                        panic!("replaced sealed identity cannot execute")
                    }
                };
            assert_eq!(
                first_replay.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Retired as i32
            );
            assert!(!first_replay.relocalization_target_available);

            let second_token = control
                .reserve_backend_generation(&second_token, 53)
                .unwrap();
            let second_token = control
                .bind_activation_identity(&second_token, [72; 32], 59)
                .unwrap();
            assert!(control.store_publication_proof(
                &second_token,
                activation_publication_proof([73; 32], 3, 59),
            ));
            control
                .seal(&second_token, Some([73; 32]), activation_base_state())
                .expect("second target seal");
            let second_snapshot = control
                .current_sealed_target_snapshot()
                .expect("replacement sealed target snapshot");
            assert_eq!(
                second_snapshot.activation_request_id,
                "sealed-snapshot-second"
            );
            assert_eq!(second_snapshot.immutable_target, second_target);
            assert_eq!(
                control.retained_immutable_target_bytes(),
                second_target.xyz_f32_le.len()
            );

            control.retire_for_mapping_change().unwrap();
            assert!(control.current_sealed_target_snapshot().is_none());
            assert_eq!(control.retained_immutable_target_bytes(), 0);
        }

        #[test]
        fn phase_a_linearization_prevents_late_seal_or_token_mutation() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let control = phase_a.activated_target_receipts();
            let seed = activation_seed_with_target("phase-a-seal-race", 7, 4);
            let token = match control.stage_or_replay(seed.clone(), activation_base_state(), true) {
                ActivationLedgerAdmission::Execute(token) => token,
                ActivationLedgerAdmission::Terminal(_) => panic!("target must stage"),
            };
            let token = control.reserve_backend_generation(&token, 13).unwrap();
            let token = control
                .bind_activation_identity(&token, [8; 32], 11)
                .unwrap();
            let entered = Arc::new(std::sync::Barrier::new(2));
            let release = Arc::new(std::sync::Barrier::new(2));
            control.install_seal_before_ledger_gate(Arc::clone(&entered), Arc::clone(&release));
            let seal_control = control.clone();
            let seal_token = token.clone();
            let seal_thread = std::thread::spawn(move || {
                seal_control.seal(&seal_token, Some([9; 32]), activation_base_state())
            });
            entered.wait();
            let epoch = control.lifecycle_epoch();
            phase_a.begin();
            assert_eq!(control.lifecycle_epoch(), epoch + 1);
            assert!(!control.validate_token(&token));
            assert!(!control.journal(
                &token,
                ActivationJournalEntry {
                    step: ActivationJournalStep::Seal,
                    idempotency_key: [10; 32],
                    status: ActivationJournalStatus::Applied,
                    verifier_receipt_sha256: Some([11; 32]),
                    typed_failure: None,
                },
            ));
            assert!(!control.store_publication_proof(
                &token,
                ActivationPublicationProof {
                    source_timestamp_ns: 1,
                    cloud_revision: 2,
                    point_count: 4,
                    voxel_size: crate::pointcloud::VOXEL_SIZE,
                    cloud_receipt: crate::mapping_commit::CloudCommitReceipt {
                        commit_id: crate::mapping_commit::MappingCommitId {
                            boot_nonce: 3,
                            sequence: 4,
                        },
                        session_generation: 11,
                        occupancy_revision: 1,
                        cloud_revision: 2,
                        source_timestamp_ns: 1,
                        point_count: 4,
                    },
                    target_receipt_sha256: [12; 32],
                },
            ));
            assert!(control.publication_proof(&token).is_none());
            assert!(control.immutable_target(&token).is_none());
            assert_eq!(control.retained_immutable_target_bytes(), 0);
            release.wait();
            assert!(seal_thread.join().unwrap().is_none());
            let replay = match control.stage_or_replay(seed, activation_base_state(), true) {
                ActivationLedgerAdmission::Terminal(state) => state,
                ActivationLedgerAdmission::Execute(_) => {
                    panic!("Phase-A-retired token cannot restage")
                }
            };
            assert!(!replay.success);
            assert_eq!(
                replay.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Retired as i32
            );
        }

        #[test]
        fn phase_a_concurrent_first_closer_retires_exactly_one_epoch() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let receipts = phase_a.activated_target_receipts();
            let before = receipts.lifecycle_epoch();
            let mut closers = Vec::new();
            for _ in 0..16 {
                let phase_a = phase_a.clone();
                closers.push(std::thread::spawn(move || phase_a.begin()));
            }
            for closer in closers {
                closer.join().unwrap();
            }
            assert_eq!(receipts.lifecycle_epoch(), before + 1);
            phase_a.begin();
            assert_eq!(receipts.lifecycle_epoch(), before + 1);

            let mut state = activation_base_state();
            receipts.decorate_current_state(&mut state);
            assert!(!state.relocalization_target_available);
            assert_eq!(state.runtime_generation, 1);
        }

        #[derive(Clone, Default)]
        struct FinalizationCapturedLogs(Arc<std::sync::Mutex<Vec<u8>>>);

        struct FinalizationCapturedLogWriter(Arc<std::sync::Mutex<Vec<u8>>>);

        impl FinalizationCapturedLogs {
            fn snapshot(&self) -> String {
                String::from_utf8(self.0.lock().expect("captured log mutex").clone())
                    .expect("tracing output is UTF-8")
            }
        }

        impl std::io::Write for FinalizationCapturedLogWriter {
            fn write(&mut self, buffer: &[u8]) -> std::io::Result<usize> {
                self.0
                    .lock()
                    .expect("captured log mutex")
                    .extend_from_slice(buffer);
                Ok(buffer.len())
            }

            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }

        impl<'writer> tracing_subscriber::fmt::MakeWriter<'writer> for FinalizationCapturedLogs {
            type Writer = FinalizationCapturedLogWriter;

            fn make_writer(&'writer self) -> Self::Writer {
                FinalizationCapturedLogWriter(Arc::clone(&self.0))
            }
        }

        fn committed_map(timestamp_ns: u64, map_frame_id: &str) -> CommittedMapProof {
            let mut proof = CommittedMapProof {
                source_timestamp_ns: timestamp_ns,
                width: 101,
                height: 79,
                resolution_m: 0.05,
                origin_x_m: -3.25,
                origin_y_m: 4.5,
                payload_sha256: [0x44; 32],
                boundary_sha256: [0; 32],
            };
            proof.boundary_sha256 = expected_map_boundary_sha256(map_frame_id, &proof).unwrap();
            proof
        }

        fn assert_fresh_lcm_gen_decodes(wire: &[u8]) {
            let version = Command::new("lcm-gen")
                .arg("--version")
                .output()
                .expect("lcm-gen must be installed for schema parity");
            assert!(version.status.success());
            assert!(String::from_utf8_lossy(&version.stdout).contains("lcm-gen 1.5.2"));

            let temp = std::env::temp_dir()
                .join(format!("nuwax-correction-lcmgen-{}", uuid::Uuid::new_v4()));
            std::fs::create_dir_all(&temp).unwrap();
            let schema = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../nuwax-workspace/lcmtypes/nuwax_msgs/CorrectionCandidate.lcm");
            let generated = Command::new("lcm-gen")
                .args(["-p", "--ppath"])
                .arg(&temp)
                .arg(&schema)
                .output()
                .expect("fresh lcm-gen code generation");
            assert!(
                generated.status.success(),
                "fresh lcm-gen failed: {}",
                String::from_utf8_lossy(&generated.stderr)
            );
            let wire_path = temp.join("candidate.bin");
            std::fs::write(&wire_path, wire).unwrap();
            let script = r#"
import pathlib
import sys
sys.path.insert(0, sys.argv[1])
from nuwax_msgs.CorrectionCandidate import CorrectionCandidate
wire = pathlib.Path(sys.argv[2]).read_bytes()
fingerprint = bytes.fromhex(sys.argv[3])
assert wire[:8] == fingerprint
assert wire[8:16] != fingerprint
candidate = CorrectionCandidate.decode(wire)
assert candidate.encode() == wire
assert candidate.schema_version == 1
assert candidate.transform_semantics == 1
assert candidate.body_frame_id == candidate.points_frame_id == "robot_body"
assert candidate.occupancy_revision != candidate.cloud_revision
"#;
            let decoded = Command::new("python3")
                .arg("-c")
                .arg(script)
                .arg(&temp)
                .arg(&wire_path)
                .arg(
                    CORRECTION_CANDIDATE_FINGERPRINT_V1
                        .iter()
                        .map(|byte| format!("{byte:02x}"))
                        .collect::<String>(),
                )
                .output()
                .expect("run fresh generated Python decoder");
            let _ = std::fs::remove_dir_all(&temp);
            assert!(
                decoded.status.success(),
                "fresh generated decode failed: {}",
                String::from_utf8_lossy(&decoded.stderr)
            );
        }

        fn sample_candidate(sequence: u64, timestamp_ns: u64) -> CorrectionCandidate {
            CorrectionCandidate::seal(
                CorrectionCandidateIdentity {
                    backend: CorrectionCandidateBackend::LibraryPgo,
                    transform_semantics: CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3,
                    runtime_generation: 11,
                    backend_generation: 12,
                    candidate_sequence: sequence,
                    mapping_session_id: "mapping-session-a".into(),
                    mapping_session_generation: 13,
                    map_frame_generation: 14,
                    mapping_commit_id: MappingCommitId {
                        boot_nonce: COMMIT_NONCE,
                        sequence: 15,
                    },
                    occupancy_revision: 16,
                    cloud_revision: 7,
                    registered_cloud: RegisteredCloudProof {
                        source_sequence: 17,
                        source_timestamp_ns: timestamp_ns,
                        point_count: 60,
                        envelope_sha256: [0x11; 32],
                        body_points_sha256: [0x22; 32],
                    },
                    correlated_odom: CorrelatedOdomProof {
                        source_sequence: 18,
                        source_timestamp_ns: timestamp_ns.saturating_sub(1),
                        pose_sha256: [0x33; 32],
                    },
                    committed_map: committed_map(timestamp_ns, "robot_map"),
                    source_timestamp_ns: timestamp_ns,
                    map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(),
                    body_frame_id: "robot_body".into(),
                    points_frame_id: "robot_body".into(),
                    correction_lease: CorrectionLeaseId {
                        boot_nonce: LEASE_NONCE,
                        generation: 19,
                    },
                },
                CorrectionTransform {
                    translation_m: [1.0, -2.0, 0.5],
                    rotation_xyzw: [0.5, 0.5, 0.5, 0.5],
                },
            )
            .expect("valid candidate")
        }

        #[test]
        fn candidate_finalization_event_format_is_exact_for_typed_outcomes() {
            let candidate = sample_candidate(1, 100);
            let reject_detail = "field=\"mapping\\identity\"\ncontrol=\u{0001}";
            let expected_reject_reason = format!("invalid identity: {reject_detail}");
            let applied =
                CandidateFinalizationEvent::new(&candidate, &CorrectionDisposition::Applied, true);
            let already_applied = CandidateFinalizationEvent::new(
                &candidate,
                &CorrectionDisposition::AlreadyApplied,
                true,
            );
            let rejected = CandidateFinalizationEvent::new(
                &candidate,
                &CorrectionDisposition::Rejected(CorrectionRejectReason::InvalidIdentity(
                    reject_detail,
                )),
                true,
            );

            assert_eq!(
                applied.to_string(),
                r#"{"event":"nuwax_correction_candidate_finalized_v1","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"payload_sha256":"b9c5a88110d9968b1909a363c81ea061e836c01654c5b59f51dcea3ce3a649e3","disposition":"Applied","candidate_processed_delivered":true}"#
            );
            assert_eq!(
                already_applied.to_string(),
                r#"{"event":"nuwax_correction_candidate_finalized_v1","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"payload_sha256":"b9c5a88110d9968b1909a363c81ea061e836c01654c5b59f51dcea3ce3a649e3","disposition":"AlreadyApplied","candidate_processed_delivered":true}"#
            );
            let rejected_json = rejected.to_string();
            assert_eq!(
                rejected_json,
                r#"{"event":"nuwax_correction_candidate_finalized_v1","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"payload_sha256":"b9c5a88110d9968b1909a363c81ea061e836c01654c5b59f51dcea3ce3a649e3","disposition":"Rejected","reject_reason":"invalid identity: field=\"mapping\\identity\"\ncontrol=\u0001","candidate_processed_delivered":true}"#
            );
            let decoded: serde_json::Value =
                serde_json::from_str(&rejected_json).expect("finalization event is valid JSON");
            assert_eq!(decoded["disposition"].as_str(), Some("Rejected"));
            assert_eq!(
                decoded["reject_reason"].as_str(),
                Some(expected_reject_reason.as_str())
            );
            assert_eq!(
                decoded["candidate_processed_delivered"].as_bool(),
                Some(true)
            );
        }

        #[test]
        fn candidate_finalization_event_reports_dropped_submitter_response() {
            let candidate = sample_candidate(1, 100);
            let captured = FinalizationCapturedLogs::default();
            let subscriber = tracing_subscriber::fmt()
                .without_time()
                .with_ansi(false)
                .with_target(false)
                .with_writer(captured.clone())
                .finish();
            let event = tracing::subscriber::with_default(subscriber, || {
                let (response, receiver) = oneshot::channel();
                drop(receiver);
                respond_and_report_candidate_finalization(
                    response,
                    &candidate,
                    CorrectionDisposition::Applied,
                )
            });

            assert_eq!(
                event.to_string(),
                r#"{"event":"nuwax_correction_candidate_finalized_v1","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"payload_sha256":"b9c5a88110d9968b1909a363c81ea061e836c01654c5b59f51dcea3ce3a649e3","disposition":"Applied","candidate_processed_delivered":false}"#
            );
            let output = captured.snapshot();
            assert_eq!(output.matches(CANDIDATE_FINALIZATION_EVENT_NAME).count(), 1);
            assert!(output.contains(&event.to_string()));
        }

        #[tokio::test]
        async fn producer_emits_finalization_event_only_after_exact_waiter_outcome() {
            let captured = FinalizationCapturedLogs::default();
            let subscriber = tracing_subscriber::fmt()
                .without_time()
                .with_ansi(false)
                .with_target(false)
                .with_writer(captured.clone())
                .finish();
            let _default = tracing::subscriber::set_default(subscriber);
            let harness = start_direct_producer(Duration::from_secs(2));
            let (apply_key, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;

            assert!(!captured
                .snapshot()
                .contains(CANDIDATE_FINALIZATION_EVENT_NAME));
            harness
                .apply_waiters
                .complete(apply_key, Ok(CorrectionDisposition::Applied));
            assert_eq!(
                tokio::time::timeout(Duration::from_secs(2), submitter)
                    .await
                    .expect("submitter observes exact finalizer outcome")
                    .expect("submitter task joins")
                    .expect("resolved candidate is reported"),
                CorrectionSubmitDisposition::CandidateProcessed {
                    candidate_sequence: apply_key.candidate_sequence,
                    payload_sha256: apply_key.payload_sha256,
                    disposition: CorrectionDisposition::Applied,
                }
            );
            tokio::time::timeout(Duration::from_secs(1), async {
                while !captured
                    .snapshot()
                    .contains(CANDIDATE_FINALIZATION_EVENT_NAME)
                {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("post-finalizer event is emitted");
            let output = captured.snapshot();
            assert_eq!(output.matches(CANDIDATE_FINALIZATION_EVENT_NAME).count(), 1);
            assert!(output.contains("\"candidate_processed_delivered\":true"));

            harness.admission_open.store(false, Ordering::Release);
            harness.admission_tx.send_replace(false);
            tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer shutdown is bounded")
                .expect("producer joins")
                .expect("admission close is a normal producer exit");
        }

        async fn assert_post_outcome_process_stop_preserves_disposition(
            disposition: CorrectionDisposition,
        ) {
            let (expected_disposition, expected_reject_reason) = match &disposition {
                CorrectionDisposition::Applied => ("Applied", None),
                CorrectionDisposition::AlreadyApplied => ("AlreadyApplied", None),
                CorrectionDisposition::Rejected(reason) => ("Rejected", Some(reason.to_string())),
                CorrectionDisposition::Prepared => {
                    panic!("test requires an exact finalizer disposition")
                }
            };
            let captured = FinalizationCapturedLogs::default();
            let subscriber = tracing_subscriber::fmt()
                .without_time()
                .with_ansi(false)
                .with_target(false)
                .with_writer(captured.clone())
                .finish();
            let _default = tracing::subscriber::set_default(subscriber);
            let gate = Arc::new(CandidateFinalizerOutcomeGate::default());
            let harness = start_direct_producer_with_finalizer_outcome_gate(
                Duration::from_secs(2),
                Arc::clone(&gate),
            );
            let (apply_key, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;

            harness
                .apply_waiters
                .complete(apply_key, Ok(disposition.clone()));
            tokio::time::timeout(Duration::from_secs(1), gate.reached.notified())
                .await
                .expect("producer observes the exact finalizer disposition");
            assert!(!captured
                .snapshot()
                .contains(CANDIDATE_FINALIZATION_EVENT_NAME));

            let process_stop = harness.phase_a.begin_process_stop();
            gate.release.notify_one();
            assert_eq!(
                tokio::time::timeout(Duration::from_secs(2), submitter)
                    .await
                    .expect("submitter observes the authoritative finalizer disposition")
                    .expect("submitter task joins")
                    .expect("resolved candidate is reported"),
                CorrectionSubmitDisposition::CandidateProcessed {
                    candidate_sequence: apply_key.candidate_sequence,
                    payload_sha256: apply_key.payload_sha256,
                    disposition,
                }
            );
            tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer stop is bounded")
                .expect("producer joins")
                .expect("post-outcome Process Stop is a normal producer exit");
            process_stop.complete();

            let output = captured.snapshot();
            assert_eq!(output.matches(CANDIDATE_FINALIZATION_EVENT_NAME).count(), 1);
            assert!(output.contains(&format!(
                "\"candidate_sequence\":{}",
                apply_key.candidate_sequence
            )));
            assert!(output.contains(&format!(
                "\"payload_sha256\":\"{}\"",
                digest_hex(&apply_key.payload_sha256)
            )));
            assert!(output.contains(&format!("\"disposition\":\"{expected_disposition}\"")));
            if let Some(reject_reason) = expected_reject_reason {
                assert!(output.contains(&format!(
                    "\"reject_reason\":{}",
                    serde_json::to_string(&reject_reason).expect("reject reason is JSON")
                )));
            }
            assert!(output.contains("\"candidate_processed_delivered\":true"));
        }

        #[tokio::test]
        async fn applied_outcome_remains_authoritative_after_process_stop() {
            assert_post_outcome_process_stop_preserves_disposition(CorrectionDisposition::Applied)
                .await;
        }

        #[tokio::test]
        async fn rejected_outcome_remains_authoritative_after_process_stop() {
            assert_post_outcome_process_stop_preserves_disposition(
                CorrectionDisposition::Rejected(CorrectionRejectReason::InvalidIdentity(
                    "post-outcome Process Stop race",
                )),
            )
            .await;
        }

        #[tokio::test]
        async fn process_stop_before_finalizer_outcome_reports_stopped_without_event() {
            let captured = FinalizationCapturedLogs::default();
            let subscriber = tracing_subscriber::fmt()
                .without_time()
                .with_ansi(false)
                .with_target(false)
                .with_writer(captured.clone())
                .finish();
            let _default = tracing::subscriber::set_default(subscriber);
            let harness = start_direct_producer(Duration::from_secs(2));
            let (_, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;

            let process_stop = harness.phase_a.begin_process_stop();
            assert!(matches!(
                tokio::time::timeout(Duration::from_secs(2), submitter)
                    .await
                    .expect("submitter observes pre-outcome Process Stop")
                    .expect("submitter task joins"),
                Err(CorrectionRuntimeError::Stopped)
            ));
            tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer stop is bounded")
                .expect("producer joins")
                .expect("pre-outcome Process Stop is a normal producer exit");
            process_stop.complete();

            assert_eq!(
                captured
                    .snapshot()
                    .matches(CANDIDATE_FINALIZATION_EVENT_NAME)
                    .count(),
                0
            );
        }

        fn boundary(candidate: &CorrectionCandidate) -> CorrectionBoundary {
            let identity = &candidate.identity;
            CorrectionBoundary {
                backend: identity.backend,
                transform_semantics: identity.transform_semantics,
                runtime_generation: identity.runtime_generation,
                backend_generation: identity.backend_generation,
                mapping_session_id: identity.mapping_session_id.clone(),
                mapping_session_generation: identity.mapping_session_generation,
                map_frame_generation: identity.map_frame_generation,
                mapping_commit_id: identity.mapping_commit_id,
                occupancy_revision: identity.occupancy_revision,
                cloud_revision: identity.cloud_revision,
                registered_cloud: identity.registered_cloud,
                correlated_odom: identity.correlated_odom,
                committed_map: identity.committed_map,
                source_timestamp_ns: identity.source_timestamp_ns,
                map_frame_id: identity.map_frame_id.clone(),
                odom_frame_id: identity.odom_frame_id.clone(),
                body_frame_id: identity.body_frame_id.clone(),
                points_frame_id: identity.points_frame_id.clone(),
                correction_lease: identity.correction_lease,
            }
        }

        #[test]
        fn preflight_supports_none_and_library_without_admitting_deferred_backends() {
            assert!(matches!(
                CorrectionOwnerSpec::preflight_for_frames(
                    CorrectionBackend::None,
                    "map",
                    "odom",
                    "base_link"
                ),
                Ok(CorrectionOwnerSpec::None)
            ));
            assert!(matches!(
                CorrectionOwnerSpec::preflight_for_frames(
                    CorrectionBackend::LibraryPgo,
                    "map",
                    "odom",
                    "base_link"
                ),
                Ok(CorrectionOwnerSpec::LibraryPgo { .. })
            ));
            assert!(matches!(
                CorrectionOwnerSpec::preflight_for_frames(
                    CorrectionBackend::NativePgo,
                    "map",
                    "odom",
                    "base_link"
                ),
                Ok(CorrectionOwnerSpec::NativePgo { .. })
            ));
            assert!(CorrectionOwnerSpec::preflight_for_frames(
                CorrectionBackend::LibraryPgo,
                " map",
                "odom",
                "base_link"
            )
            .is_err());
        }

        #[test]
        fn coordinator_advances_replay_cursor_only_after_applied_commit() {
            let candidate = sample_candidate(1, 100);
            let exact = boundary(&candidate);
            let mut coordinator = CorrectionCoordinator::default();
            let prepared = coordinator.prepare(&candidate, &exact);
            assert_eq!(prepared.disposition, CorrectionDisposition::Prepared);
            assert_eq!(
                coordinator.prepare(&candidate, &exact).disposition,
                CorrectionDisposition::Prepared,
                "an exact in-flight retry is idempotent"
            );
            coordinator.abort(&prepared).unwrap();
            let prepared_again = coordinator.prepare(&candidate, &exact);
            assert_eq!(prepared_again.disposition, CorrectionDisposition::Prepared);
            let applied = coordinator.commit_applied(&prepared_again).unwrap();
            assert_eq!(applied.disposition, CorrectionDisposition::Applied);
            assert_eq!(
                coordinator.prepare(&candidate, &exact).disposition,
                CorrectionDisposition::AlreadyApplied,
                "an exact replay after APPLIED is idempotent"
            );

            let mut conflicting = candidate.clone();
            conflicting.transform.translation_m[0] += 0.25;
            conflicting.payload_sha256 = conflicting.expected_payload_sha256().unwrap();
            assert_eq!(
                coordinator.prepare(&conflicting, &exact).disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::ReplayConflict)
            );

            let next = sample_candidate(2, 101);
            let next_boundary = boundary(&next);
            let next_prepared = coordinator.prepare(&next, &next_boundary);
            assert_eq!(next_prepared.disposition, CorrectionDisposition::Prepared);
            let later = sample_candidate(3, 102);
            assert_eq!(
                coordinator.prepare(&later, &boundary(&later)).disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::ApplyInFlight)
            );
            coordinator.abort(&next_prepared).unwrap();
            let next_retry = coordinator.prepare(&next, &next_boundary);
            assert_eq!(next_retry.disposition, CorrectionDisposition::Prepared);
            coordinator.commit_applied(&next_retry).unwrap();
            assert_eq!(
                coordinator.prepare(&candidate, &exact).disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::OutOfOrderCandidate)
            );
            let regressed = sample_candidate(3, 100);
            assert_eq!(
                coordinator
                    .prepare(&regressed, &boundary(&regressed))
                    .disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::SourceTimestampRegressed)
            );
        }

        #[test]
        fn coordinator_rejects_every_identity_boundary_mismatch() {
            let candidate = sample_candidate(1, 100);
            let cases: Vec<(CorrectionBoundary, CorrectionRejectReason)> = vec![
                (
                    {
                        let mut value = boundary(&candidate);
                        value.backend = CorrectionCandidateBackend::NativePgo;
                        value
                    },
                    CorrectionRejectReason::BackendMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.transform_semantics =
                            CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3;
                        value.points_frame_id.push('x');
                        value
                    },
                    CorrectionRejectReason::FrameMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.runtime_generation += 1;
                        value
                    },
                    CorrectionRejectReason::RuntimeGenerationMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.backend_generation += 1;
                        value
                    },
                    CorrectionRejectReason::BackendGenerationMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.correction_lease.generation += 1;
                        value
                    },
                    CorrectionRejectReason::CorrectionLeaseMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.mapping_session_id.push('x');
                        value
                    },
                    CorrectionRejectReason::MappingSessionMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.mapping_session_generation += 1;
                        value
                    },
                    CorrectionRejectReason::MappingSessionGenerationMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.map_frame_generation += 1;
                        value
                    },
                    CorrectionRejectReason::MapFrameGenerationMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.mapping_commit_id.sequence += 1;
                        value
                    },
                    CorrectionRejectReason::MappingCommitMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.occupancy_revision += 1;
                        value
                    },
                    CorrectionRejectReason::OccupancyRevisionMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.cloud_revision += 1;
                        value
                    },
                    CorrectionRejectReason::CloudRevisionMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.registered_cloud.envelope_sha256[0] ^= 1;
                        value
                    },
                    CorrectionRejectReason::RegisteredCloudProofMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.correlated_odom.pose_sha256[0] ^= 1;
                        value
                    },
                    CorrectionRejectReason::CorrelatedOdomProofMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.committed_map.payload_sha256[0] ^= 1;
                        value
                    },
                    CorrectionRejectReason::CommittedMapProofMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.map_frame_id.push('x');
                        value
                    },
                    CorrectionRejectReason::FrameMismatch,
                ),
            ];
            for (boundary, expected) in cases {
                let mut coordinator = CorrectionCoordinator::default();
                assert_eq!(
                    coordinator.prepare(&candidate, &boundary).disposition,
                    CorrectionDisposition::Rejected(expected)
                );
            }
        }

        #[test]
        fn candidate_hash_transform_and_stop_fences_fail_closed() {
            let candidate = sample_candidate(1, 100);
            let exact = boundary(&candidate);

            let mut bad_hash = candidate.clone();
            bad_hash.payload_sha256[0] ^= 1;
            assert_eq!(
                CorrectionCoordinator::default()
                    .prepare(&bad_hash, &exact)
                    .disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::PayloadHashMismatch)
            );

            let mut non_finite = candidate.clone();
            non_finite.transform.translation_m[0] = f64::NAN;
            assert_eq!(
                CorrectionCoordinator::default()
                    .prepare(&non_finite, &exact)
                    .disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::NonFiniteTransform)
            );

            let mut bad_rotation = candidate.clone();
            bad_rotation.transform.rotation_xyzw = [0.0; 4];
            assert_eq!(
                CorrectionCoordinator::default()
                    .prepare(&bad_rotation, &exact)
                    .disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::InvalidRotation)
            );

            let mut coordinator = CorrectionCoordinator::default();
            coordinator.stop();
            assert_eq!(
                coordinator.prepare(&candidate, &exact).disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::Stopped)
            );
        }

        fn owner_identity() -> CorrectionOwnerRuntimeIdentity {
            CorrectionOwnerRuntimeIdentity {
                runtime_generation: 21,
                backend_generation: 22,
                correction_lease: CorrectionLeaseId {
                    boot_nonce: LEASE_NONCE,
                    generation: 24,
                },
            }
        }

        fn asymmetric_cloud() -> Vec<Point3<f64>> {
            (0..5)
                .flat_map(|x| {
                    (0..4).flat_map(move |y| {
                        (0..3).map(move |z| {
                            Point3::new(
                                x as f64 * 0.41 + (y * y + z) as f64 * 0.013,
                                y as f64 * 0.43 + (x * z + z * z) as f64 * 0.017,
                                z as f64 * 0.37 + (x * x + y) as f64 * 0.011,
                            )
                        })
                    })
                })
                .collect()
        }

        fn frame(sequence: u64, timestamp_ns: u64, x: f64) -> CorrectionFrameInput {
            let body_points = asymmetric_cloud();
            let odom_from_body = Isometry3::translation(x, 0.0, 0.0);
            let mut occupancy = vec![0_u8; 101 * 79];
            occupancy[..4].copy_from_slice(&[0, 20, 80, 255]);
            let committed_map_occupancy: Arc<[u8]> = Arc::from(occupancy);
            let mut committed_map = committed_map(timestamp_ns, "robot_map");
            committed_map.payload_sha256 = sha256_bytes(committed_map_occupancy.as_ref());
            committed_map.boundary_sha256 =
                expected_map_boundary_sha256("robot_map", &committed_map).unwrap();
            let registered_cloud = RegisteredCloudProof {
                source_sequence: sequence + 100,
                source_timestamp_ns: timestamp_ns,
                point_count: body_points.len() as u64,
                envelope_sha256: sha256_bytes(&sequence.to_be_bytes()),
                body_points_sha256: body_points_sha256(&body_points).unwrap(),
            };
            CorrectionFrameInput {
                mapping_session_id: "mapping-session-owner".into(),
                mapping_session_generation: 23,
                mapping_commit_id: MappingCommitId {
                    boot_nonce: COMMIT_NONCE,
                    sequence,
                },
                occupancy_revision: sequence + 100,
                cloud_revision: sequence,
                map_frame_generation: 25,
                registered_cloud,
                correlated_odom: FrozenCorrelatedOdom {
                    proof: CorrelatedOdomProof {
                        source_sequence: sequence + 200,
                        source_timestamp_ns: timestamp_ns,
                        pose_sha256: odom_pose_sha256(&odom_from_body).unwrap(),
                    },
                    odom_from_body,
                },
                committed_map,
                committed_map_occupancy,
                points_frame_id: "robot_body".into(),
                body_points,
                base_anchor_map_from_odom: Isometry3::identity(),
                frame_capture_relocalization_epoch: 0,
                relocalization_mode_epoch: None,
            }
        }

        fn active_relocalization_frame(
            owner: &CorrectionTransportOwner,
            sequence: u64,
            timestamp_ns: u64,
            x: f64,
        ) -> CorrectionFrameInput {
            let mode = owner.ingress().relocalization_mode_snapshot();
            assert!(
                mode.active,
                "fixture requires an active relocalization epoch"
            );
            let mut input = frame(sequence, timestamp_ns, x);
            input.frame_capture_relocalization_epoch = mode.epoch;
            input.relocalization_mode_epoch = Some(mode.epoch);
            input
        }

        #[test]
        fn native_request_projects_the_complete_frozen_geometry_boundary() {
            let identity = owner_identity();
            let input = frame(7, 123_000_000_000, 0.25);
            let request =
                native_pgo_request(&identity, "robot_map", "robot_odom", "robot_body", &input)
                    .expect("complete native request");

            assert_eq!(
                request.identity.backend,
                native_pgo_codec::NATIVE_PGO_BACKEND
            );
            assert_eq!(request.identity.runtime_generation, 21);
            assert_eq!(request.identity.backend_generation, 22);
            assert_eq!(request.identity.request_sequence, 0);
            assert_eq!(
                request.identity.mapping_session_id,
                input.mapping_session_id
            );
            assert_eq!(request.identity.mapping_commit_id.sequence, 7);
            assert_eq!(
                request.identity.mapping_commit_id.boot_nonce,
                input.mapping_commit_id.boot_nonce.to_be_bytes()
            );
            assert_eq!(
                request.identity.committed_map.payload_sha256,
                input.committed_map.payload_sha256
            );
            assert_eq!(
                request.identity.committed_map.boundary_sha256,
                input.committed_map.boundary_sha256
            );
            assert_eq!(
                request.identity.correction_lease.boot_nonce,
                identity.correction_lease.boot_nonce.to_be_bytes()
            );
            assert_eq!(
                request.registered_body_coordinate_count as usize,
                input.body_points.len() * 3
            );
            assert_eq!(
                request.registered_body_points_xyz,
                input
                    .body_points
                    .iter()
                    .flat_map(|point| [point.x, point.y, point.z])
                    .collect::<Vec<_>>()
            );
            assert_eq!(
                &request.committed_map_occupancy[..4],
                &[0_i8, 20_i8, 80_i8, -1_i8]
            );
            assert_eq!(
                request.committed_map_cell_count as usize,
                input.committed_map_occupancy.len()
            );
            assert_eq!(request.frozen_odom_translation_m, [0.25, 0.0, 0.0]);
            assert_eq!(request.frozen_odom_rotation_xyzw, [0.0, 0.0, 0.0, 1.0]);
            assert_eq!(request.request_payload_sha256, [0; 32]);
        }

        #[derive(Default)]
        struct NativeFinalizingSink {
            staged: parking_lot::Mutex<Vec<u64>>,
            discarded: parking_lot::Mutex<Vec<u64>>,
            applied: parking_lot::Mutex<Vec<CorrectionCandidate>>,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for NativeFinalizingSink {
            async fn stage_frame(&self, input: &CorrectionFrameInput) -> Result<(), String> {
                self.staged.lock().push(input.mapping_commit_id.sequence);
                Ok(())
            }

            async fn discard_frame(&self, input: &CorrectionFrameInput) {
                let sequence = input.mapping_commit_id.sequence;
                self.staged.lock().retain(|staged| *staged != sequence);
                self.discarded.lock().push(sequence);
            }

            async fn apply_candidate(
                &self,
                candidate: CorrectionCandidate,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                let operation = cancellation.operation();
                if !operation.try_claim(&cancellation) {
                    return Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::Stopped,
                    ));
                }
                operation.mark_acked_ok()?;
                operation.mark_state_committed()?;
                operation.mark_coordinator_finalized()?;
                cancellation.finish_canonical_commit();
                self.staged
                    .lock()
                    .retain(|staged| *staged != candidate.identity.mapping_commit_id.sequence);
                self.applied.lock().push(candidate);
                Ok(CorrectionDisposition::Applied)
            }
        }

        #[tokio::test]
        async fn captured_frame_epoch_is_rechecked_before_enqueue() {
            let sink = Arc::new(NativeFinalizingSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                sink.clone(),
            )
            .await
            .expect("correction owner starts");
            let ingress = owner.ingress();
            let captured = ingress.relocalization_mode_snapshot();
            assert!(!captured.active);

            let _activated = ingress.relocalization_mode_fence().activate_for_tests();
            let input = frame(7, 7_000_000_000, 0.0);
            assert_eq!(input.frame_capture_relocalization_epoch, captured.epoch);
            assert_eq!(
                ingress.submit(input).await.expect("fenced submission"),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert!(sink.staged.lock().is_empty());
            owner.shutdown().await.expect("correction owner shutdown");
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
        async fn actual_native_owner_returns_typed_dispositions_and_direct_finalizes_candidate() {
            let _release_env = native_correction_test_guard().await;
            let provider = crate::lcm_runtime::test_lcm_url();
            let sink = Arc::new(NativeFinalizingSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::NativePgo {
                    map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(),
                    body_frame_id: "robot_body".into(),
                    executable: nuwax_nav_pgo_bridge::native_executable(),
                },
                owner_identity(),
                &provider,
                sink.clone(),
            )
            .await
            .expect("actual native PGO owner READY");
            assert!(owner.has_independent_real_contexts());
            assert!(owner.binding().is_none());
            let ingress = owner.ingress();

            let mut rejected = frame(1, 1_000_000_000, 0.0);
            rejected.body_points.truncate(3);
            rejected.registered_cloud.point_count = rejected.body_points.len() as u64;
            rejected.registered_cloud.body_points_sha256 =
                body_points_sha256(&rejected.body_points).unwrap();
            match ingress.submit(rejected).await.unwrap() {
                CorrectionSubmitDisposition::Rejected {
                    reason_code,
                    reason,
                } => {
                    assert_eq!(
                        reason_code,
                        native_pgo_codec::NATIVE_PGO_REJECTION_RESOURCE_LIMIT
                    );
                    assert!(reason.contains("too few filtered points"));
                }
                disposition => panic!("expected typed native rejection, got {disposition:?}"),
            }

            assert_eq!(
                ingress.submit(frame(2, 2_000_000_000, 0.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert_eq!(
                ingress.submit(frame(3, 63_000_000_000, 2.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            let processed = ingress
                .submit(frame(4, 124_000_000_000, 0.2))
                .await
                .expect("native correction request");
            let (candidate_sequence, payload_sha256) = match processed {
                CorrectionSubmitDisposition::CandidateProcessed {
                    candidate_sequence,
                    payload_sha256,
                    disposition: CorrectionDisposition::Applied,
                } => (candidate_sequence, payload_sha256),
                disposition => panic!("expected applied native candidate, got {disposition:?}"),
            };
            assert_eq!(candidate_sequence, 4);
            assert_ne!(payload_sha256, [0; 32]);
            {
                let applied = sink.applied.lock();
                assert_eq!(applied.len(), 1);
                assert_eq!(
                    applied[0].identity.backend,
                    CorrectionCandidateBackend::NativePgo
                );
                assert_eq!(applied[0].identity.candidate_sequence, candidate_sequence);
                assert_eq!(applied[0].payload_sha256, payload_sha256);
                applied[0].validate_integrity().unwrap();
            }
            assert_eq!(*sink.discarded.lock(), vec![1, 2, 3]);
            assert!(sink.staged.lock().is_empty());

            owner.shutdown().await.expect("native owner clean shutdown");
            assert!(!ingress.is_open());
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
        async fn native_finalizer_handoff_failure_cleans_waiter_stage_and_generation() {
            let _release_env = native_correction_test_guard().await;
            let provider = crate::lcm_runtime::test_lcm_url();
            let sink = Arc::new(NativeFinalizingSink::default());
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::NativePgo {
                    map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(),
                    body_frame_id: "robot_body".into(),
                    executable: nuwax_nav_pgo_bridge::native_executable(),
                },
                owner_identity(),
                &provider,
                sink.clone(),
            )
            .await
            .expect("actual native PGO owner READY");
            owner
                .finalizer
                .shutdown()
                .await
                .expect("injected finalizer closure");
            let ingress = owner.ingress();

            assert_eq!(
                ingress.submit(frame(1, 1_000_000_000, 0.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert_eq!(
                ingress.submit(frame(2, 62_000_000_000, 2.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            let error = ingress
                .submit(frame(3, 123_000_000_000, 0.2))
                .await
                .expect_err("closed finalizer must fail the generation");
            assert!(matches!(
                error,
                CorrectionRuntimeError::FinalizerPoisoned(_)
            ));
            assert!(!ingress.is_open());
            tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    if sink.staged.lock().is_empty() {
                        break;
                    }
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("staged native frame discard");
            assert_eq!(*sink.discarded.lock(), vec![1, 2, 3]);
            assert!(owner.apply_waiters.pending.lock().is_empty());
            assert!(sink.applied.lock().is_empty());

            let terminal = owner
                .wait_terminal()
                .await
                .expect_err("producer retains finalizer handoff failure");
            assert!(terminal.contains("canonical finalizer ingress closed"));
            let settlement = owner
                .shutdown()
                .await
                .expect("finalizer handoff failure settles the owner");
            assert!(settlement
                .into_sticky_error()
                .expect("finalizer failure remains sticky")
                .contains("canonical finalizer ingress closed"));
        }

        fn transport_library_spec() -> CorrectionOwnerSpec {
            CorrectionOwnerSpec::LibraryPgo {
                map_frame_id: "robot_map".into(),
                odom_frame_id: "robot_odom".into(),
                body_frame_id: "robot_body".into(),
                pipeline: LoopClosurePipelineConfig {
                    keyframe_translation: 0.1,
                    keyframe_rotation: 0.05,
                    loop_search_radius: 0.5,
                    min_loop_separation_secs: 30.0,
                    min_keyframe_points: 10,
                    max_loop_mse: 1.0e-8,
                    ..LoopClosurePipelineConfig::default()
                },
            }
        }

        struct RecordingCandidateSink {
            sender: mpsc::UnboundedSender<CorrectionCandidate>,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for RecordingCandidateSink {
            async fn apply_candidate(
                &self,
                candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                self.sender
                    .send(candidate)
                    .map_err(|_| "recording correction sink is closed".to_owned())?;
                Ok(CorrectionDisposition::Applied)
            }
        }

        #[derive(Default)]
        struct ProducerStageSink {
            staged_sequences: parking_lot::Mutex<Vec<u64>>,
            discarded_sequences: parking_lot::Mutex<Vec<u64>>,
            block_discard_sequence: std::sync::atomic::AtomicU64,
            discard_started: tokio::sync::Notify,
            discard_release: tokio::sync::Notify,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for ProducerStageSink {
            async fn stage_frame(&self, input: &CorrectionFrameInput) -> Result<(), String> {
                self.staged_sequences
                    .lock()
                    .push(input.mapping_commit_id.sequence);
                Ok(())
            }

            async fn discard_frame(&self, input: &CorrectionFrameInput) {
                let sequence = input.mapping_commit_id.sequence;
                if self.block_discard_sequence.load(Ordering::Acquire) == sequence {
                    self.discard_started.notify_one();
                    self.discard_release.notified().await;
                }
                self.staged_sequences
                    .lock()
                    .retain(|staged| *staged != sequence);
                self.discarded_sequences.lock().push(sequence);
            }

            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                panic!("direct producer harness has no candidate subscriber")
            }
        }

        struct DirectProducerHarness {
            ingress: CorrectionFrameIngress,
            admission_open: Arc<AtomicBool>,
            admission_tx: watch::Sender<bool>,
            phase_a: CorrectionPhaseAControl,
            apply_waiters: Arc<CandidateApplyWaiters>,
            sink: Arc<ProducerStageSink>,
            _finalizer_rx: mpsc::Receiver<CanonicalFinalizerJob>,
            _cleanup: CorrectionCleanupSupervisor,
            worker: tokio::task::JoinHandle<Result<(), CorrectionRuntimeError>>,
        }

        fn start_direct_producer(apply_ack_timeout: Duration) -> DirectProducerHarness {
            start_direct_producer_with_gate(apply_ack_timeout, None)
        }

        fn start_direct_producer_with_gate(
            apply_ack_timeout: Duration,
            publish_gate: Option<Arc<CandidatePublishGate>>,
        ) -> DirectProducerHarness {
            start_direct_producer_with_gates(apply_ack_timeout, publish_gate, None)
        }

        fn start_direct_producer_with_finalizer_outcome_gate(
            apply_ack_timeout: Duration,
            finalizer_outcome_gate: Arc<CandidateFinalizerOutcomeGate>,
        ) -> DirectProducerHarness {
            start_direct_producer_with_gates(apply_ack_timeout, None, Some(finalizer_outcome_gate))
        }

        fn start_direct_producer_with_gates(
            apply_ack_timeout: Duration,
            publish_gate: Option<Arc<CandidatePublishGate>>,
            finalizer_outcome_gate: Option<Arc<CandidateFinalizerOutcomeGate>>,
        ) -> DirectProducerHarness {
            let identity = owner_identity();
            let owner = CorrectionOwnerHandle::start(transport_library_spec(), identity.clone())
                .expect("library candidate owner");
            let provider = crate::lcm_runtime::test_lcm_url();
            let publisher =
                LcmTransport::new_real(&provider).expect("isolated real-LCM publisher context");
            let binding = correction_candidate_binding(identity.runtime_generation)
                .expect("private correction binding");
            let admission_open = Arc::new(AtomicBool::new(true));
            let (admission_tx, admission_rx) = watch::channel(true);
            let (request_tx, request_rx) = mpsc::channel(CORRECTION_INPUT_QUEUE_CAPACITY);
            let apply_waiters = Arc::new(CandidateApplyWaiters::new(Arc::clone(&admission_open)));
            let phase_a = CorrectionPhaseAControl::new(
                Arc::clone(&admission_open),
                admission_tx.clone(),
                Arc::clone(&apply_waiters),
                1,
            );
            let sink = Arc::new(ProducerStageSink::default());
            let cleanup_sink: Arc<dyn CorrectionCandidateSink> = sink.clone();
            let cleanup = CorrectionCleanupSupervisor::start(cleanup_sink);
            let (finalizer_tx, finalizer_rx) = mpsc::channel(CORRECTION_INPUT_QUEUE_CAPACITY);
            let ingress = CorrectionFrameIngress {
                sender: request_tx,
                admission_open: Arc::clone(&admission_open),
                admission_rx: admission_rx.clone(),
                relocalization_mode: phase_a.relocalization_mode(),
            };
            let worker = tokio::spawn(run_candidate_producer(
                CorrectionCandidateOwner::Local(owner),
                request_rx,
                admission_rx,
                CandidateProducerContext {
                    publisher: Some(publisher),
                    binding: Some(binding),
                    finalizer: CanonicalPublishFinalizerIngress {
                        sender: finalizer_tx,
                    },
                    apply_waiters: Arc::clone(&apply_waiters),
                    sink: sink.clone(),
                    cleanup: cleanup.ingress(),
                    apply_ack_timeout,
                    phase_a: phase_a.clone(),
                    publish_gate,
                    finalizer_outcome_gate,
                },
            ));
            DirectProducerHarness {
                ingress,
                admission_open,
                admission_tx,
                phase_a,
                apply_waiters,
                sink,
                _finalizer_rx: finalizer_rx,
                _cleanup: cleanup,
                worker,
            }
        }

        async fn submit_candidate_until_apply_wait(
            ingress: &CorrectionFrameIngress,
            apply_waiters: &CandidateApplyWaiters,
        ) -> (
            CandidateApplyKey,
            tokio::task::JoinHandle<Result<CorrectionSubmitDisposition, CorrectionRuntimeError>>,
        ) {
            assert_eq!(
                ingress.submit(frame(1, 0, 0.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert_eq!(
                ingress.submit(frame(2, 10_000_000_000, 2.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            let ingress = ingress.clone();
            let submitter =
                tokio::spawn(async move { ingress.submit(frame(3, 60_000_000_000, 0.4)).await });
            let apply_key = tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    if let Some(key) = { apply_waiters.pending.lock().keys().next().copied() } {
                        break key;
                    }
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("candidate apply waiter registration is bounded");
            (apply_key, submitter)
        }

        async fn assert_candidate_frame_discarded(
            apply_waiters: &CandidateApplyWaiters,
            sink: &ProducerStageSink,
        ) {
            assert!(apply_waiters.pending.lock().is_empty());
            let cleanup = tokio::time::timeout(Duration::from_secs(1), async {
                loop {
                    let cleanup_finished = sink.staged_sequences.lock().is_empty()
                        && sink.discarded_sequences.lock().contains(&3);
                    if cleanup_finished {
                        break;
                    }
                    tokio::task::yield_now().await;
                }
            })
            .await;
            if cleanup.is_err() {
                let pending_keys = apply_waiters
                    .pending
                    .lock()
                    .keys()
                    .copied()
                    .collect::<Vec<_>>();
                let staged_sequences = sink.staged_sequences.lock().clone();
                let discarded_sequences = sink.discarded_sequences.lock().clone();
                panic!(
                    "scheduled candidate discard did not complete within 1 second after terminal \
                     observation: pending_keys={pending_keys:?}, \
                     staged_sequences={staged_sequences:?}, \
                     discarded_sequences={discarded_sequences:?}"
                );
            }
            assert!(sink.staged_sequences.lock().is_empty());
            assert!(sink.discarded_sequences.lock().contains(&3));
        }

        struct RecordingCanonicalApplier {
            fail_next: AtomicBool,
            transactions: tokio::sync::Mutex<Vec<CanonicalCorrectionTransaction>>,
        }

        impl RecordingCanonicalApplier {
            fn new(fail_next: bool) -> Self {
                Self {
                    fail_next: AtomicBool::new(fail_next),
                    transactions: tokio::sync::Mutex::new(Vec::new()),
                }
            }

            async fn transactions(&self) -> Vec<CanonicalCorrectionTransaction> {
                self.transactions.lock().await.clone()
            }
        }

        #[async_trait::async_trait]
        impl CanonicalCorrectionApplier for RecordingCanonicalApplier {
            async fn apply_transaction(
                &self,
                transaction: CanonicalCorrectionTransaction,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<(), CanonicalCorrectionApplyError> {
                if self.fail_next.swap(false, Ordering::AcqRel) {
                    return Err(CanonicalCorrectionApplyError::Failed(
                        "injected canonical apply failure".to_owned(),
                    ));
                }
                let operation = cancellation.operation();
                if !operation.try_claim(&cancellation) {
                    return Err(CanonicalCorrectionApplyError::Cancelled);
                }
                operation
                    .mark_acked_ok()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                self.transactions.lock().await.push(transaction);
                operation
                    .mark_state_committed()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                Ok(())
            }
        }

        #[derive(Default)]
        struct GatedCanonicalApplier {
            before_publish: tokio::sync::Notify,
            allow_publish: tokio::sync::Notify,
            publish_started: tokio::sync::Notify,
            finish_publish: tokio::sync::Notify,
            published: std::sync::atomic::AtomicU64,
        }

        #[async_trait::async_trait]
        impl CanonicalCorrectionApplier for GatedCanonicalApplier {
            async fn apply_transaction(
                &self,
                _transaction: CanonicalCorrectionTransaction,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<(), CanonicalCorrectionApplyError> {
                self.before_publish.notify_one();
                tokio::select! {
                    biased;
                    _ = cancellation.cancelled() => {
                        return Err(CanonicalCorrectionApplyError::Cancelled);
                    }
                    _ = self.allow_publish.notified() => {}
                }
                let operation = cancellation.operation();
                if !operation.try_claim(&cancellation) {
                    return Err(CanonicalCorrectionApplyError::Cancelled);
                }
                self.publish_started.notify_one();
                self.finish_publish.notified().await;
                self.published.fetch_add(1, Ordering::AcqRel);
                operation
                    .mark_acked_ok()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                operation
                    .mark_state_committed()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                Ok(())
            }
        }

        struct DelayedGeometrySink {
            inner: Arc<GeometryCorrectionSink>,
            apply_entered: tokio::sync::Notify,
            apply_release: tokio::sync::Notify,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for DelayedGeometrySink {
            async fn stage_frame(&self, input: &CorrectionFrameInput) -> Result<(), String> {
                self.inner.stage_frame(input).await
            }

            async fn discard_frame(&self, input: &CorrectionFrameInput) {
                self.inner.discard_frame(input).await;
            }

            async fn apply_candidate(
                &self,
                candidate: CorrectionCandidate,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                self.apply_entered.notify_one();
                self.apply_release.notified().await;
                self.inner.apply_candidate(candidate, cancellation).await
            }

            async fn stop(&self) {
                self.inner.stop().await;
            }
        }

        fn candidate_for_frame(
            input: &CorrectionFrameInput,
            sequence: u64,
            correction: Isometry3<f64>,
        ) -> CorrectionCandidate {
            let owner = owner_identity();
            CorrectionCandidate::seal(
                CorrectionCandidateIdentity {
                    backend: CorrectionCandidateBackend::LibraryPgo,
                    transform_semantics: CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3,
                    runtime_generation: owner.runtime_generation,
                    backend_generation: owner.backend_generation,
                    candidate_sequence: sequence,
                    mapping_session_id: input.mapping_session_id.clone(),
                    mapping_session_generation: input.mapping_session_generation,
                    map_frame_generation: input.map_frame_generation,
                    mapping_commit_id: input.mapping_commit_id,
                    occupancy_revision: input.occupancy_revision,
                    cloud_revision: input.cloud_revision,
                    registered_cloud: input.registered_cloud,
                    correlated_odom: input.correlated_odom.proof,
                    committed_map: input.committed_map,
                    source_timestamp_ns: input.registered_cloud.source_timestamp_ns,
                    map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(),
                    body_frame_id: "robot_body".into(),
                    points_frame_id: "robot_body".into(),
                    correction_lease: owner.correction_lease,
                },
                CorrectionTransform::from_isometry(&correction),
            )
            .unwrap()
        }

        fn assert_isometry_close(actual: &Isometry3<f64>, expected: &Isometry3<f64>) {
            assert!(
                (actual.translation.vector - expected.translation.vector).norm() < 1.0e-9,
                "translation differs: actual={actual:?}, expected={expected:?}"
            );
            assert!(
                actual.rotation.angle_to(&expected.rotation) < 1.0e-9,
                "rotation differs: actual={actual:?}, expected={expected:?}"
            );
        }

        #[tokio::test]
        async fn canonical_transaction_uses_nonzero_anchor_and_absolute_c_without_accumulation() {
            let applier = Arc::new(RecordingCanonicalApplier::new(false));
            let sink = GeometryCorrectionSink::new(
                owner_identity(),
                "robot_map",
                "robot_odom",
                "robot_body",
                applier.clone(),
            )
            .unwrap();
            let base_anchor = Isometry3::from_parts(
                Translation3::new(8.0, -3.0, 1.5),
                UnitQuaternion::from_euler_angles(0.2, -0.15, 0.4),
            );
            let correction_one = Isometry3::from_parts(
                Translation3::new(-0.6, 0.2, 0.4),
                UnitQuaternion::from_euler_angles(-0.1, 0.25, -0.3),
            );
            let correction_two = Isometry3::from_parts(
                Translation3::new(0.3, -0.7, 0.9),
                UnitQuaternion::from_euler_angles(0.35, -0.2, 0.15),
            );

            let mut first_input = frame(1, 100, 0.0);
            first_input.base_anchor_map_from_odom = base_anchor;
            let first_candidate = candidate_for_frame(&first_input, 1, correction_one);
            sink.stage_frame(&first_input).await.unwrap();
            assert_eq!(
                sink.apply_candidate(first_candidate, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Applied
            );

            let mut second_input = frame(2, 200, 1.0);
            second_input.base_anchor_map_from_odom = base_anchor;
            second_input.cloud_revision = 42;
            let second_candidate = candidate_for_frame(&second_input, 2, correction_two);
            sink.stage_frame(&second_input).await.unwrap();
            assert_eq!(
                sink.apply_candidate(
                    second_candidate.clone(),
                    CorrectionRequestCancellation::new(),
                )
                .await
                .unwrap(),
                CorrectionDisposition::Applied
            );

            let transactions = applier.transactions().await;
            assert_eq!(transactions.len(), 2);
            assert_eq!(transactions[0].correction_revision, 1);
            assert_eq!(transactions[1].correction_revision, 2);
            assert_ne!(
                transactions[1].correction_revision,
                transactions[1].occupancy_revision
            );
            assert_ne!(
                transactions[1].correction_revision,
                transactions[1].cloud_revision
            );
            assert_isometry_close(
                &transactions[0].effective_map_from_odom,
                &(base_anchor * correction_one),
            );
            assert_isometry_close(
                &transactions[1].effective_map_from_odom,
                &(base_anchor * correction_two),
            );
            let incorrectly_accumulated = transactions[0].effective_map_from_odom * correction_two;
            assert!(
                (transactions[1].effective_map_from_odom.translation.vector
                    - incorrectly_accumulated.translation.vector)
                    .norm()
                    > 0.1
                    || transactions[1]
                        .effective_map_from_odom
                        .rotation
                        .angle_to(&incorrectly_accumulated.rotation)
                        > 0.1
            );

            assert_eq!(
                sink.apply_candidate(second_candidate, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::AlreadyApplied
            );
            assert_eq!(applier.transactions().await.len(), 2);
            assert_eq!(sink.correction_revision().await, 2);
        }

        #[tokio::test]
        async fn canonical_transaction_rejects_identity_mismatch_and_abort_allows_exact_retry() {
            let applier = Arc::new(RecordingCanonicalApplier::new(true));
            let sink = GeometryCorrectionSink::new(
                owner_identity(),
                "robot_map",
                "robot_odom",
                "robot_body",
                applier.clone(),
            )
            .unwrap();
            let mut input = frame(1, 100, 0.0);
            input.base_anchor_map_from_odom = Isometry3::translation(5.0, 1.0, -2.0);
            let candidate = candidate_for_frame(
                &input,
                1,
                Isometry3::from_parts(
                    Translation3::new(0.2, 0.3, 0.4),
                    UnitQuaternion::from_euler_angles(0.1, 0.2, 0.3),
                ),
            );
            assert_eq!(
                sink.apply_candidate(candidate.clone(), CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Rejected(CorrectionRejectReason::NoStagedFrame)
            );
            assert_eq!(sink.correction_revision().await, 0);
            assert!(applier.transactions().await.is_empty());
            sink.stage_frame(&input).await.unwrap();

            let mut wrong_lease_identity = candidate.identity.clone();
            wrong_lease_identity.correction_lease.generation += 1;
            let wrong_lease =
                CorrectionCandidate::seal(wrong_lease_identity, candidate.transform).unwrap();
            assert_eq!(
                sink.apply_candidate(wrong_lease, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Rejected(CorrectionRejectReason::CorrectionLeaseMismatch)
            );
            assert_eq!(sink.correction_revision().await, 0);
            assert!(applier.transactions().await.is_empty());

            assert_eq!(
                sink.apply_candidate(candidate.clone(), CorrectionRequestCancellation::new())
                    .await,
                Err("injected canonical apply failure".to_owned())
            );
            assert_eq!(sink.correction_revision().await, 0);
            assert_eq!(
                sink.apply_candidate(candidate.clone(), CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Applied
            );
            assert_eq!(sink.correction_revision().await, 1);
            assert_eq!(applier.transactions().await.len(), 1);

            let mut conflict_transform = candidate.transform;
            conflict_transform.translation_m[0] += 0.25;
            let conflict =
                CorrectionCandidate::seal(candidate.identity.clone(), conflict_transform)
                    .expect("self-consistent conflicting replay");
            assert_eq!(
                sink.apply_candidate(conflict, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Rejected(CorrectionRejectReason::ReplayConflict)
            );
            assert_eq!(applier.transactions().await.len(), 1);

            sink.stop().await;
            assert_eq!(
                sink.apply_candidate(candidate, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Rejected(CorrectionRejectReason::Stopped)
            );
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn cancellation_before_canonical_publish_preserves_revision() {
            let applier = Arc::new(GatedCanonicalApplier::default());
            let sink = Arc::new(
                GeometryCorrectionSink::new(
                    owner_identity(),
                    "robot_map",
                    "robot_odom",
                    "robot_body",
                    applier.clone(),
                )
                .unwrap(),
            );
            let input = frame(1, 100, 0.0);
            let candidate = candidate_for_frame(&input, 1, Isometry3::translation(1.0, 0.0, 0.0));
            sink.stage_frame(&input).await.unwrap();
            let cancellation = CorrectionRequestCancellation::new();
            let apply_sink = Arc::clone(&sink);
            let apply_cancellation = cancellation.clone();
            let apply = tokio::spawn(async move {
                apply_sink
                    .apply_candidate(candidate, apply_cancellation)
                    .await
            });
            tokio::time::timeout(Duration::from_secs(1), applier.before_publish.notified())
                .await
                .expect("applier reaches cancellable pre-publish window");
            cancellation.cancel();
            assert_eq!(
                tokio::time::timeout(Duration::from_secs(1), apply)
                    .await
                    .expect("pre-publish cancellation is bounded")
                    .expect("apply task joins")
                    .expect("cancellation is a disposition"),
                CorrectionDisposition::Rejected(CorrectionRejectReason::Stopped)
            );
            assert_eq!(sink.correction_revision().await, 0);
            assert_eq!(applier.published.load(Ordering::Acquire), 0);
            tokio::time::timeout(Duration::from_secs(1), sink.stop())
                .await
                .expect("stop after pre-publish cancellation is bounded");
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn publish_started_is_linearized_and_stop_joins_without_late_output() {
            let applier = Arc::new(GatedCanonicalApplier::default());
            let sink = Arc::new(
                GeometryCorrectionSink::new(
                    owner_identity(),
                    "robot_map",
                    "robot_odom",
                    "robot_body",
                    applier.clone(),
                )
                .unwrap(),
            );
            let input = frame(1, 100, 0.0);
            let candidate = candidate_for_frame(&input, 1, Isometry3::translation(1.0, 0.0, 0.0));
            sink.stage_frame(&input).await.unwrap();
            let cancellation = CorrectionRequestCancellation::new();
            let apply_sink = Arc::clone(&sink);
            let apply_cancellation = cancellation.clone();
            let apply = tokio::spawn(async move {
                apply_sink
                    .apply_candidate(candidate, apply_cancellation)
                    .await
            });
            tokio::time::timeout(Duration::from_secs(1), applier.before_publish.notified())
                .await
                .expect("applier reaches pre-publish window");
            applier.allow_publish.notify_one();
            tokio::time::timeout(Duration::from_secs(1), applier.publish_started.notified())
                .await
                .expect("canonical publish starts");
            cancellation.cancel();

            let stop_sink = Arc::clone(&sink);
            let stop = tokio::spawn(async move { stop_sink.stop().await });
            tokio::task::yield_now().await;
            assert!(!stop.is_finished(), "stop must join the linearized publish");
            applier.finish_publish.notify_one();
            assert_eq!(
                tokio::time::timeout(Duration::from_secs(1), apply)
                    .await
                    .expect("linearized apply completion is bounded")
                    .expect("apply task joins")
                    .expect("linearized apply succeeds"),
                CorrectionDisposition::Applied
            );
            tokio::time::timeout(Duration::from_secs(1), stop)
                .await
                .expect("stop joins completed publish")
                .expect("stop task joins");
            assert_eq!(sink.correction_revision().await, 1);
            assert_eq!(applier.published.load(Ordering::Acquire), 1);
            tokio::time::sleep(Duration::from_millis(25)).await;
            assert_eq!(
                applier.published.load(Ordering::Acquire),
                1,
                "stop returned before late canonical output quiesced"
            );
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn candidate_apply_timeout_is_terminal_and_discards_staged_frame() {
            let harness = start_direct_producer(Duration::from_millis(250));
            let (_, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;

            let submit_error = tokio::time::timeout(Duration::from_secs(2), submitter)
                .await
                .expect("submitter observes bounded apply timeout")
                .expect("submitter task joins")
                .expect_err("missing apply acknowledgement is terminal");
            assert!(matches!(
                submit_error,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "candidate apply acknowledgement timed out"
            ));
            let terminal = tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer terminates after apply timeout")
                .expect("producer task joins")
                .expect_err("apply timeout fails the producer closed");
            assert!(matches!(
                terminal,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "candidate apply acknowledgement timed out"
            ));
            assert_candidate_frame_discarded(&harness.apply_waiters, &harness.sink).await;
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn apply_timeout_returns_before_blocked_discard_cleanup() {
            let harness = start_direct_producer(Duration::from_millis(100));
            harness
                .sink
                .block_discard_sequence
                .store(3, Ordering::Release);
            let (_, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;

            let submit_error = tokio::time::timeout(Duration::from_secs(1), submitter)
                .await
                .expect("ACK timeout response is independent of discard")
                .expect("submitter task joins")
                .expect_err("missing ACK is terminal");
            assert!(matches!(
                submit_error,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "candidate apply acknowledgement timed out"
            ));
            let sink = Arc::clone(&harness.sink);
            let terminal = tokio::time::timeout(Duration::from_secs(1), harness.worker)
                .await
                .expect("producer terminal is independent of discard")
                .expect("producer task joins")
                .expect_err("ACK timeout fails the producer");
            assert!(matches!(
                terminal,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "candidate apply acknowledgement timed out"
            ));
            tokio::time::timeout(Duration::from_secs(1), sink.discard_started.notified())
                .await
                .expect("bounded cleanup starts");
            assert!(sink.staged_sequences.lock().contains(&3));
            sink.discard_release.notify_one();
            tokio::time::timeout(Duration::from_secs(1), async {
                while !sink.discarded_sequences.lock().contains(&3) {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("released cleanup completes");
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn aborted_submit_before_private_publish_cancels_exact_waiter() {
            let gate = Arc::new(CandidatePublishGate::default());
            let harness =
                start_direct_producer_with_gate(Duration::from_secs(2), Some(Arc::clone(&gate)));
            let (apply_key, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;
            tokio::time::timeout(Duration::from_secs(1), gate.reached.notified())
                .await
                .expect("producer reaches pre-publish barrier");
            let cancellation = harness
                .apply_waiters
                .cancellation(apply_key)
                .expect("exact waiter token");

            submitter.abort();
            assert!(submitter
                .await
                .expect_err("submit is aborted")
                .is_cancelled());
            tokio::time::timeout(Duration::from_secs(1), cancellation.cancelled())
                .await
                .expect("submit RAII drop cancels exact request");
            gate.release.notify_one();
            tokio::time::timeout(Duration::from_secs(1), async {
                while !harness.apply_waiters.pending.lock().is_empty()
                    || harness.sink.staged_sequences.lock().contains(&3)
                {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("cancelled request is removed and discarded");
            assert_eq!(gate.publish_calls.load(Ordering::Acquire), 0);

            harness.admission_open.store(false, Ordering::Release);
            harness.admission_tx.send_replace(false);
            tokio::time::timeout(Duration::from_secs(1), harness.worker)
                .await
                .expect("producer shutdown is bounded")
                .expect("producer joins")
                .expect("external request cancellation is nonterminal");
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn candidate_apply_ack_loss_is_terminal_and_discards_staged_frame() {
            let harness = start_direct_producer(Duration::from_secs(2));
            let (apply_key, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;
            harness.apply_waiters.remove(apply_key);

            let submit_error = tokio::time::timeout(Duration::from_secs(2), submitter)
                .await
                .expect("submitter observes acknowledgement loss")
                .expect("submitter task joins")
                .expect_err("lost apply acknowledgement is terminal");
            assert!(matches!(
                submit_error,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "candidate apply acknowledgement channel closed"
            ));
            let terminal = tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer terminates after acknowledgement loss")
                .expect("producer task joins")
                .expect_err("acknowledgement loss fails the producer closed");
            assert!(matches!(
                terminal,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "candidate apply acknowledgement channel closed"
            ));
            assert_candidate_frame_discarded(&harness.apply_waiters, &harness.sink).await;
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn admission_close_preempts_apply_wait_and_exits_normally() {
            let harness = start_direct_producer(Duration::from_secs(2));
            let (_, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;
            harness.admission_open.store(false, Ordering::Release);
            harness.admission_tx.send_replace(false);

            assert!(matches!(
                tokio::time::timeout(Duration::from_secs(2), submitter)
                    .await
                    .expect("submitter observes admission close")
                    .expect("submitter task joins"),
                Err(CorrectionRuntimeError::Stopped)
            ));
            tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer stop is bounded")
                .expect("producer task joins")
                .expect("admission close is a normal producer exit");
            assert_candidate_frame_discarded(&harness.apply_waiters, &harness.sink).await;
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn canonical_apply_error_remains_terminal_and_discards_staged_frame() {
            let harness = start_direct_producer(Duration::from_secs(2));
            let (apply_key, submitter) =
                submit_candidate_until_apply_wait(&harness.ingress, &harness.apply_waiters).await;
            harness.apply_waiters.complete(
                apply_key,
                Err("injected canonical apply failure".to_owned()),
            );

            let submit_error = tokio::time::timeout(Duration::from_secs(2), submitter)
                .await
                .expect("submitter observes canonical apply failure")
                .expect("submitter task joins")
                .expect_err("canonical apply failure is terminal");
            assert!(matches!(
                submit_error,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "injected canonical apply failure"
            ));
            let terminal = tokio::time::timeout(Duration::from_secs(2), harness.worker)
                .await
                .expect("producer terminates after canonical apply failure")
                .expect("producer task joins")
                .expect_err("canonical apply failure fails the producer closed");
            assert!(matches!(
                terminal,
                CorrectionRuntimeError::Apply(detail)
                    if detail == "injected canonical apply failure"
            ));
            assert_candidate_frame_discarded(&harness.apply_waiters, &harness.sink).await;
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn aborted_submit_after_private_publish_never_reaches_canonical_applier() {
            let provider = crate::lcm_runtime::test_lcm_url();
            let applier = Arc::new(RecordingCanonicalApplier::new(false));
            let geometry = Arc::new(
                GeometryCorrectionSink::new(
                    owner_identity(),
                    "robot_map",
                    "robot_odom",
                    "robot_body",
                    applier.clone(),
                )
                .unwrap(),
            );
            let sink = Arc::new(DelayedGeometrySink {
                inner: Arc::clone(&geometry),
                apply_entered: tokio::sync::Notify::new(),
                apply_release: tokio::sync::Notify::new(),
            });
            let owner = CorrectionTransportOwner::start(
                transport_library_spec(),
                owner_identity(),
                &provider,
                sink.clone(),
            )
            .await
            .expect("private correction owner starts");
            let ingress = owner.ingress();
            assert_eq!(
                ingress.submit(frame(1, 0, 0.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert_eq!(
                ingress.submit(frame(2, 10_000_000_000, 2.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            let submit_ingress = ingress.clone();
            let submitter =
                tokio::spawn(
                    async move { submit_ingress.submit(frame(3, 60_000_000_000, 0.4)).await },
                );
            tokio::time::timeout(Duration::from_secs(2), sink.apply_entered.notified())
                .await
                .expect("private candidate was published and delivered");
            let cancellation = owner
                .apply_waiters
                .pending
                .lock()
                .values()
                .next()
                .expect("live exact waiter")
                .lease
                .cancellation();
            submitter.abort();
            assert!(submitter
                .await
                .expect_err("submit is aborted")
                .is_cancelled());
            tokio::time::timeout(Duration::from_secs(1), cancellation.cancelled())
                .await
                .expect("aborted submit cancels delivered request");
            sink.apply_release.notify_one();
            tokio::time::timeout(Duration::from_secs(2), async {
                while !owner.apply_waiters.pending.lock().is_empty() {
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("cancelled delivered candidate is retired");
            assert_eq!(geometry.correction_revision().await, 0);
            assert!(applier.transactions().await.is_empty());
            owner.shutdown().await.expect("cancelled owner quiesces");
            assert_eq!(geometry.correction_revision().await, 0);
            assert!(applier.transactions().await.is_empty());
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn transport_owner_acks_dual_context_and_quiesces_before_late_output() {
            let provider = crate::lcm_runtime::test_lcm_url();
            let (candidate_tx, mut candidate_rx) = mpsc::unbounded_channel();
            let sink = Arc::new(RecordingCandidateSink {
                sender: candidate_tx,
            });
            let owner = CorrectionTransportOwner::start(
                transport_library_spec(),
                owner_identity(),
                &provider,
                sink,
            )
            .await
            .expect("library transport owner starts after subscription ACK");
            assert!(owner.has_independent_real_contexts());
            let binding = owner.binding().expect("library private typed binding");
            assert!(binding
                .0
                .channel()
                .ends_with(&owner_identity().runtime_generation.to_string()));
            assert_eq!(binding.1.fingerprint(), CORRECTION_CANDIDATE_FINGERPRINT_V1);

            let ingress = owner.ingress();
            assert!(ingress.is_open());
            assert_eq!(
                ingress.submit(frame(1, 0, 0.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert_eq!(
                ingress.submit(frame(2, 10_000_000_000, 2.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            let processed = ingress
                .submit(frame(3, 60_000_000_000, 0.4))
                .await
                .expect("loop candidate crosses private publisher");
            let candidate = tokio::time::timeout(Duration::from_secs(2), candidate_rx.recv())
                .await
                .expect("private subscriber receive timeout")
                .expect("recorded candidate");
            assert_eq!(
                processed,
                CorrectionSubmitDisposition::CandidateProcessed {
                    candidate_sequence: candidate.identity.candidate_sequence,
                    payload_sha256: candidate.payload_sha256,
                    disposition: CorrectionDisposition::Applied,
                }
            );
            candidate.validate_integrity().unwrap();

            owner
                .shutdown()
                .await
                .expect("unsubscribe, drain, and join are acknowledged");
            assert!(!ingress.is_open());
            assert!(matches!(
                ingress.submit(frame(4, 70_000_000_000, 1.0)).await,
                Err(CorrectionRuntimeError::Stopped)
            ));

            let late_publisher =
                LcmTransport::new_real(&provider).expect("late real LCM publisher context");
            let late_candidate = sample_candidate(9, 90_000_000_000);
            late_publisher
                .publish(
                    RawLcmFrameBuilder::new(binding.0, binding.1)
                        .seqno(9)
                        .payload(late_candidate.encode_body().unwrap())
                        .build(),
                )
                .await
                .expect("late transport output is allowed but unobserved");
            tokio::time::sleep(Duration::from_millis(150)).await;
            assert!(matches!(
                candidate_rx.try_recv(),
                Err(mpsc::error::TryRecvError::Empty | mpsc::error::TryRecvError::Disconnected)
            ));
        }

        #[tokio::test]
        async fn none_transport_owner_is_retained_silent_and_needs_no_lcm_context() {
            let (candidate_tx, mut candidate_rx) = mpsc::unbounded_channel();
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "not-a-provider-used-by-none",
                Arc::new(RecordingCandidateSink {
                    sender: candidate_tx,
                }),
            )
            .await
            .expect("none owner has no transport side effect");
            assert!(!owner.has_independent_real_contexts());
            assert!(owner.binding().is_none());
            let ingress = owner.ingress();
            assert_eq!(
                ingress.submit(frame(1, 0, 0.0)).await.unwrap(),
                CorrectionSubmitDisposition::NoCandidate
            );
            assert!(matches!(
                candidate_rx.try_recv(),
                Err(mpsc::error::TryRecvError::Empty | mpsc::error::TryRecvError::Disconnected)
            ));
            owner.shutdown().await.unwrap();
            assert!(matches!(
                ingress.submit(frame(2, 1, 0.0)).await,
                Err(CorrectionRuntimeError::Stopped)
            ));
        }

        #[tokio::test]
        async fn transport_owner_uses_provided_relocalization_mode_fence() {
            let (candidate_tx, _candidate_rx) = mpsc::unbounded_channel();
            let relocalization_mode = Arc::new(RelocalizationModeFence::default());
            let owner = CorrectionTransportOwner::start_with_relocalization_mode_fence(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                Arc::new(RecordingCandidateSink {
                    sender: candidate_tx,
                }),
                Arc::clone(&relocalization_mode),
            )
            .await
            .expect("none owner accepts a provided relocalization mode fence");

            assert!(Arc::ptr_eq(
                &owner.ingress().relocalization_mode_fence(),
                &relocalization_mode,
            ));
            assert!(Arc::ptr_eq(
                &owner.phase_a_control().relocalization_mode(),
                &relocalization_mode,
            ));
            owner.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn none_process_stop_is_the_only_clean_producer_terminal() {
            let (candidate_tx, _candidate_rx) = mpsc::unbounded_channel();
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                Arc::new(RecordingCandidateSink {
                    sender: candidate_tx,
                }),
            )
            .await
            .expect("none correction owner");
            let phase_a = owner.phase_a_control();
            let process_stop = phase_a.begin_process_stop();

            tokio::time::timeout(Duration::from_secs(1), owner.wait_terminal())
                .await
                .expect("none producer closes after Process Stop")
                .expect("caller-requested Process Stop is a clean producer terminal");
            process_stop.complete();
            owner
                .shutdown()
                .await
                .expect("clean Process Stop remains clean through shutdown");
        }

        #[tokio::test]
        async fn internal_close_precedes_and_cannot_be_blessed_by_late_process_stop() {
            let (candidate_tx, _candidate_rx) = mpsc::unbounded_channel();
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                Arc::new(RecordingCandidateSink {
                    sender: candidate_tx,
                }),
            )
            .await
            .expect("none correction owner");
            let phase_a = owner.phase_a_control();
            phase_a.begin();
            let late_process_stop = phase_a.begin_process_stop();
            assert_eq!(late_process_stop.wait_for_owner_result().await, Err(()));

            let terminal = tokio::time::timeout(Duration::from_secs(1), owner.wait_terminal())
                .await
                .expect("internal close producer terminal")
                .expect_err("internal admission closure is never a graceful Process Stop");
            assert!(terminal.contains("correction producer exited unexpectedly"));
            late_process_stop.complete();
            let settlement = owner
                .shutdown()
                .await
                .expect("internal close terminal settles the owner");
            assert!(settlement
                .into_sticky_error()
                .expect("internal close terminal survives shutdown")
                .contains("correction producer exited unexpectedly"));
        }

        #[tokio::test]
        async fn unexpected_clean_producer_exit_is_retained_as_fatal() {
            let (candidate_tx, _candidate_rx) = mpsc::unbounded_channel();
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                Arc::new(RecordingCandidateSink {
                    sender: candidate_tx,
                }),
            )
            .await
            .expect("none correction owner");
            let original = owner
                .producer_worker
                .take()
                .expect("retained producer worker");
            original.abort();
            let _ = original.await;
            owner.producer_worker = Some(tokio::spawn(async { Ok(()) }));

            let terminal = owner
                .wait_terminal()
                .await
                .expect_err("unsolicited clean producer exit is fatal");
            assert!(terminal.contains("correction producer exited unexpectedly"));
            let settlement = owner
                .shutdown()
                .await
                .expect("unexpected producer exit still settles the owner");
            assert!(settlement
                .into_sticky_error()
                .expect("unexpected clean terminal survives shutdown")
                .contains("correction producer exited unexpectedly"));
        }

        #[tokio::test]
        async fn dropped_first_process_stop_attempt_reports_failure_once() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let first = phase_a.begin_process_stop();
            let duplicate = phase_a.begin_process_stop();
            assert!(
                tokio::time::timeout(
                    Duration::from_millis(25),
                    phase_a.wait_process_stop_failed()
                )
                .await
                .is_err(),
                "a duplicate Stop cannot fail the first in-flight attempt"
            );
            drop(first);
            tokio::time::timeout(
                Duration::from_millis(100),
                phase_a.wait_process_stop_failed(),
            )
            .await
            .expect("dropping the first Process Stop attempt publishes failure");
            assert_eq!(
                duplicate.wait_for_owner_result().await,
                Err(()),
                "a duplicate Stop observes the first attempt's retained failure"
            );
            drop(duplicate);

            let internal = CorrectionPhaseAControl::for_tests();
            internal.begin();
            drop(internal.begin_process_stop());
            assert!(
                tokio::time::timeout(
                    Duration::from_millis(25),
                    internal.wait_process_stop_failed()
                )
                .await
                .is_err(),
                "a late Process Stop cannot overwrite INTERNAL_CLOSED"
            );
        }

        struct FailingStageSink;

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for FailingStageSink {
            async fn stage_frame(&self, _input: &CorrectionFrameInput) -> Result<(), String> {
                Err("injected correction staging failure".to_owned())
            }

            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                _cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                panic!("a frame rejected during staging cannot produce a candidate")
            }
        }

        #[tokio::test]
        async fn owner_terminal_is_sticky_but_shutdown_returns_settled_without_owner() {
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("none owner starts without transport I/O");
            let ingress = owner.ingress();

            let submit_error = ingress
                .submit(frame(1, 0, 0.0))
                .await
                .expect_err("staging failure must reach the submitting caller");
            assert!(submit_error
                .to_string()
                .contains("injected correction staging failure"));
            let terminal = tokio::time::timeout(Duration::from_secs(1), owner.wait_terminal())
                .await
                .expect("terminal observation must be bounded")
                .expect_err("producer failure is terminal");
            assert!(terminal.contains("injected correction staging failure"));
            assert!(owner.is_finished());
            assert!(!ingress.is_open());
            assert!(matches!(
                ingress.submit(frame(2, 1, 0.0)).await,
                Err(CorrectionRuntimeError::Stopped)
            ));

            let settlement = owner
                .shutdown()
                .await
                .expect("terminal failure does not make a settled owner pending");
            assert!(settlement
                .into_sticky_error()
                .expect("retained terminal cause must survive shutdown")
                .contains("injected correction staging failure"));
        }

        #[test]
        fn none_is_silent_and_library_owner_emits_only_sealed_loop_candidate() {
            let mut none = CorrectionOwnerHandle::start(
                CorrectionOwnerSpec::preflight_for_frames(
                    CorrectionBackend::None,
                    "map",
                    "odom",
                    "base_link",
                )
                .unwrap(),
                owner_identity(),
            )
            .unwrap();
            assert!(none.submit_frame(frame(1, 0, 0.0)).unwrap().is_none());
            none.stop();
            assert_eq!(
                none.submit_frame(frame(2, 1, 0.0)),
                Err(CorrectionOwnerError::Stopped)
            );

            let pipeline = LoopClosurePipelineConfig {
                keyframe_translation: 0.1,
                keyframe_rotation: 0.05,
                loop_search_radius: 0.5,
                min_loop_separation_secs: 30.0,
                min_keyframe_points: 10,
                max_loop_mse: 1.0e-8,
                ..LoopClosurePipelineConfig::default()
            };
            let mut library = CorrectionOwnerHandle::start(
                CorrectionOwnerSpec::LibraryPgo {
                    map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(),
                    body_frame_id: "robot_body".into(),
                    pipeline,
                },
                owner_identity(),
            )
            .unwrap();
            let mut wrong_points_frame = frame(1, 0, 0.0);
            wrong_points_frame.points_frame_id = "robot_map".into();
            assert_eq!(
                library.submit_frame(wrong_points_frame),
                Err(CorrectionOwnerError::UnsupportedPointsFrame)
            );
            let mut changed_points_after_proof = frame(1, 0, 0.0);
            changed_points_after_proof.body_points[0].x += 0.25;
            assert_eq!(
                library.submit_frame(changed_points_after_proof),
                Err(CorrectionOwnerError::InvalidRegisteredCloudProof)
            );
            assert!(library.submit_frame(frame(1, 0, 0.0)).unwrap().is_none());
            let mut wrong_namespace = frame(2, 10_000_000_000, 2.0);
            wrong_namespace.mapping_commit_id.boot_nonce += 1;
            assert_eq!(
                library.submit_frame(wrong_namespace),
                Err(CorrectionOwnerError::MappingCommitNamespaceChanged)
            );
            let mut changed_odom_after_proof = frame(2, 10_000_000_000, 2.0);
            changed_odom_after_proof.correlated_odom.odom_from_body =
                Isometry3::translation(3.0, 0.0, 0.0);
            assert_eq!(
                library.submit_frame(changed_odom_after_proof),
                Err(CorrectionOwnerError::InvalidCorrelatedOdomProof)
            );
            assert!(library
                .submit_frame(frame(2, 10_000_000_000, 2.0))
                .unwrap()
                .is_none());
            let candidate = library
                .submit_frame(frame(3, 60_000_000_000, 0.4))
                .unwrap()
                .expect("loop candidate");
            assert_eq!(
                candidate.identity.backend,
                CorrectionCandidateBackend::LibraryPgo
            );
            assert_eq!(candidate.identity.candidate_sequence, 1);
            assert_eq!(candidate.identity.mapping_commit_id.sequence, 3);
            assert_eq!(candidate.identity.occupancy_revision, 103);
            assert_eq!(candidate.identity.cloud_revision, 3);
            assert_eq!(candidate.identity.points_frame_id, "robot_body");
            assert_eq!(
                candidate.identity.transform_semantics,
                CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3
            );
            assert!(candidate.transform.translation_m[0] < -0.2);
            candidate.validate_integrity().expect("sealed candidate");

            library.stop();
            assert_eq!(
                library.submit_frame(frame(4, 70_000_000_000, 1.0)),
                Err(CorrectionOwnerError::Stopped)
            );
            library.shutdown();
        }

        #[test]
        fn library_owner_resets_pipeline_for_newer_mapping_session_generation() {
            let mut library =
                CorrectionOwnerHandle::start(transport_library_spec(), owner_identity())
                    .expect("library owner");

            assert!(library.submit_frame(frame(1, 0, 0.0)).unwrap().is_none());
            assert!(library
                .submit_frame(frame(2, 10_000_000_000, 2.0))
                .unwrap()
                .is_none());
            let first_candidate = library
                .submit_frame(frame(3, 60_000_000_000, 0.4))
                .unwrap()
                .expect("first session loop candidate");
            assert_eq!(first_candidate.identity.candidate_sequence, 1);
            assert_eq!(first_candidate.identity.mapping_session_generation, 23);

            let restarted_frame = |sequence: u64, timestamp_ns: u64, x: f64| {
                let mut input = frame(sequence, timestamp_ns, x);
                input.mapping_session_id = "mapping-session-restarted".into();
                input.mapping_session_generation = 24;
                input.map_frame_generation = 26;
                input.mapping_commit_id.boot_nonce = COMMIT_NONCE + 1;
                input.occupancy_revision = sequence;
                input.cloud_revision = sequence;
                input
            };
            assert!(library
                .submit_frame(restarted_frame(1, 0, 0.0))
                .unwrap()
                .is_none());
            assert_eq!(
                library.submit_frame(frame(4, 70_000_000_000, 1.0)),
                Err(CorrectionOwnerError::StaleMappingSession)
            );
            assert!(library
                .submit_frame(restarted_frame(2, 10_000_000_000, 2.0))
                .unwrap()
                .is_none());
            let restarted_candidate = library
                .submit_frame(restarted_frame(3, 60_000_000_000, 0.4))
                .unwrap()
                .expect("restarted session loop candidate");
            assert_eq!(restarted_candidate.identity.candidate_sequence, 2);
            assert_eq!(
                restarted_candidate.identity.mapping_session_id,
                "mapping-session-restarted"
            );
            assert_eq!(restarted_candidate.identity.mapping_session_generation, 24);
            assert_eq!(restarted_candidate.identity.map_frame_generation, 26);
            assert_eq!(
                restarted_candidate.identity.mapping_commit_id.boot_nonce,
                COMMIT_NONCE + 1
            );
            restarted_candidate
                .validate_integrity()
                .expect("restarted candidate remains sealed");
        }

        #[test]
        fn identity_proofs_and_full_se3_semantics_fail_closed_without_projection() {
            let candidate = sample_candidate(1, 100);
            assert_eq!(candidate.transform.translation_m[2], 0.5);
            assert_eq!(candidate.transform.rotation_xyzw, [0.5, 0.5, 0.5, 0.5]);

            let mut wrong_points = candidate.identity.clone();
            wrong_points.points_frame_id = wrong_points.map_frame_id.clone();
            assert_eq!(
                CorrectionCandidate::seal(wrong_points, candidate.transform),
                Err(CorrectionRejectReason::UnsupportedPointsFrame)
            );

            let mut wrong_boundary = candidate.identity.clone();
            wrong_boundary.committed_map.boundary_sha256[0] ^= 1;
            assert_eq!(
                CorrectionCandidate::seal(wrong_boundary, candidate.transform),
                Err(CorrectionRejectReason::MapBoundaryHashMismatch)
            );

            let mut missing_cloud_proof = candidate.identity.clone();
            missing_cloud_proof.registered_cloud.envelope_sha256 = [0; 32];
            assert_eq!(
                CorrectionCandidate::seal(missing_cloud_proof, candidate.transform),
                Err(CorrectionRejectReason::InvalidRegisteredCloudProof)
            );
        }

        #[test]
        fn generated_schema_wire_roundtrip_and_hostile_bytes_are_rejected() {
            let candidate = sample_candidate(1, 100);
            let body = candidate.encode_body().expect("encode candidate body");
            let encoded = candidate.encode_wire().expect("encode candidate");
            assert_eq!(&encoded[..8], &CORRECTION_CANDIDATE_FINGERPRINT_V1);
            assert_eq!(&encoded[8..], body);
            assert_ne!(&body[..8], &CORRECTION_CANDIDATE_FINGERPRINT_V1);
            assert_eq!(candidate.payload_sha256, GENERATED_PAYLOAD_SHA256);
            assert_eq!(body.len(), 541);
            assert_eq!(encoded.len(), 549);
            assert_eq!(sha256_bytes(&body), GENERATED_BODY_SHA256);
            assert_eq!(sha256_bytes(&encoded), GENERATED_WIRE_SHA256);
            assert_eq!(CorrectionCandidate::decode_body(&body).unwrap(), candidate);
            assert_eq!(
                CorrectionCandidate::decode_wire(&encoded).unwrap(),
                candidate
            );

            let mut bad_fingerprint = encoded.clone();
            bad_fingerprint[0] ^= 1;
            assert!(CorrectionCandidate::decode_wire(&bad_fingerprint).is_err());
            let mut unsupported_semantics = encoded.clone();
            unsupported_semantics[16..20].copy_from_slice(&2_i32.to_be_bytes());
            assert!(CorrectionCandidate::decode_wire(&unsupported_semantics).is_err());
            let mut double_fingerprint = CORRECTION_CANDIDATE_FINGERPRINT_V1.to_vec();
            double_fingerprint.extend_from_slice(&encoded);
            assert!(CorrectionCandidate::decode_wire(&double_fingerprint).is_err());
            assert!(CorrectionCandidate::decode_wire(&body).is_err());
            assert!(CorrectionCandidate::decode_body(&encoded).is_err());
            let mut bad_hash = encoded.clone();
            *bad_hash.last_mut().unwrap() ^= 1;
            assert!(CorrectionCandidate::decode_wire(&bad_hash).is_err());
            let mut trailing = encoded.clone();
            trailing.push(0);
            assert!(matches!(
                CorrectionCandidate::decode_wire(&trailing),
                Err(CorrectionWireError::Trailing(1))
            ));
            assert!(CorrectionCandidate::decode_wire(&encoded[..encoded.len() - 1]).is_err());
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn correction_candidate_crosses_independent_real_liblcm_contexts() {
            let provider = crate::lcm_runtime::test_lcm_url();
            let publisher = LcmTransport::new_real(&provider).expect("real LCM publisher context");
            let subscriber =
                LcmTransport::new_real(&provider).expect("independent real LCM subscriber context");
            assert_eq!(publisher.backend_name(), "real-lcm");
            assert_eq!(subscriber.backend_name(), "real-lcm");
            let publisher_context = publisher.shared_lcm_context().expect("publisher context");
            let subscriber_context = subscriber.shared_lcm_context().expect("subscriber context");
            assert!(!Arc::ptr_eq(&publisher_context, &subscriber_context));

            let topic = LcmTopic::new("/geometry/correction/candidate").unwrap();
            let message_type = LcmMessageType::new(CORRECTION_CANDIDATE_MESSAGE_TYPE).unwrap();
            assert_eq!(
                message_type.fingerprint(),
                CORRECTION_CANDIDATE_FINGERPRINT_V1
            );
            let mut subscription = subscriber
                .subscribe_owned(topic.clone(), Some(message_type.clone()))
                .await
                .expect("acknowledged typed subscription");
            let candidate = sample_candidate(7, 123_456_789);
            publisher
                .publish(
                    RawLcmFrameBuilder::new(topic, message_type)
                        .seqno(7)
                        .payload(candidate.encode_body().unwrap())
                        .build(),
                )
                .await
                .expect("real LCM publish");
            let received = tokio::time::timeout(Duration::from_secs(2), subscription.recv())
                .await
                .expect("real LCM receive timeout")
                .expect("real LCM receive");
            assert_eq!(
                CorrectionCandidate::decode_body(&received.payload).unwrap(),
                candidate
            );
            assert_eq!(received.payload, candidate.encode_body().unwrap());
            let mut generated_wire = CORRECTION_CANDIDATE_FINGERPRINT_V1.to_vec();
            generated_wire.extend_from_slice(&received.payload);
            assert_eq!(generated_wire, candidate.encode_wire().unwrap());
            assert_fresh_lcm_gen_decodes(&generated_wire);
            subscription
                .unsubscribe()
                .await
                .expect("acknowledged unsubscribe");
        }

        struct PanicOnceCanonicalApplier {
            after_claim: bool,
            panic_next: AtomicBool,
        }

        #[async_trait::async_trait]
        impl CanonicalCorrectionApplier for PanicOnceCanonicalApplier {
            async fn apply_transaction(
                &self,
                _transaction: CanonicalCorrectionTransaction,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<(), CanonicalCorrectionApplyError> {
                let operation = cancellation.operation();
                if self.panic_next.swap(false, Ordering::AcqRel) {
                    if self.after_claim {
                        assert!(operation.try_claim(&cancellation));
                    }
                    panic!("injected canonical operation panic");
                }
                if !operation.try_claim(&cancellation) {
                    return Err(CanonicalCorrectionApplyError::Cancelled);
                }
                operation
                    .mark_acked_ok()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                operation
                    .mark_state_committed()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                Ok(())
            }
        }

        struct HandoffFinalizingSink {
            entered: tokio::sync::Notify,
            release: tokio::sync::Notify,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for HandoffFinalizingSink {
            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                let operation = cancellation.operation();
                if !operation.try_claim(&cancellation) {
                    return Ok(CorrectionDisposition::Rejected(
                        CorrectionRejectReason::Stopped,
                    ));
                }
                self.entered.notify_one();
                self.release.notified().await;
                operation.mark_acked_ok()?;
                operation.mark_state_committed()?;
                operation.mark_coordinator_finalized()?;
                cancellation.finish_canonical_commit();
                Ok(CorrectionDisposition::Applied)
            }
        }

        fn phase_a_fixture() -> (CorrectionPhaseAControl, Arc<CandidateApplyWaiters>) {
            let admission_open = Arc::new(AtomicBool::new(true));
            let (admission_tx, _) = watch::channel(true);
            let waiters = Arc::new(CandidateApplyWaiters::new(Arc::clone(&admission_open)));
            (
                CorrectionPhaseAControl::new(admission_open, admission_tx, Arc::clone(&waiters), 1),
                waiters,
            )
        }

        #[tokio::test]
        async fn phase_a_closes_admission_cancels_exact_unclaimed_waiter_and_is_idempotent() {
            let (phase_a, waiters) = phase_a_fixture();
            let cancellation = CorrectionRequestCancellation::new();
            let lease = Arc::new(CorrectionRequestLease::new(cancellation.clone(), None));
            let key = CandidateApplyKey {
                candidate_sequence: 1,
                payload_sha256: [0x11; 32],
            };
            let apply_rx = waiters.register(key, lease).expect("admitted exact waiter");

            phase_a.begin();
            phase_a.begin();

            assert!(!phase_a.is_open());
            assert!(cancellation.is_cancelled());
            assert!(waiters
                .register(
                    CandidateApplyKey {
                        candidate_sequence: 2,
                        payload_sha256: [0x22; 32],
                    },
                    Arc::new(CorrectionRequestLease::new(
                        CorrectionRequestCancellation::new(),
                        None,
                    )),
                )
                .is_err());
            assert!(!waiters.apply_claimed_or_remove(key));
            assert!(apply_rx.await.is_err());
            tokio::time::timeout(Duration::from_millis(100), phase_a.wait_settled())
                .await
                .expect("unclaimed exact waiter settles");
        }

        #[tokio::test]
        async fn phase_a_waits_for_claimed_precommit_waiter_to_settle_safely() {
            let (phase_a, waiters) = phase_a_fixture();
            let cancellation = CorrectionRequestCancellation::new();
            let lease = Arc::new(CorrectionRequestLease::new(cancellation.clone(), None));
            let key = CandidateApplyKey {
                candidate_sequence: 1,
                payload_sha256: [0x33; 32],
            };
            let apply_rx = waiters
                .register(key, Arc::clone(&lease))
                .expect("admitted exact waiter");
            let claimed = waiters.claim_apply(key).expect("claimed waiter lease");

            phase_a.begin();

            assert!(cancellation.is_cancelled());
            assert!(
                tokio::time::timeout(Duration::from_millis(25), phase_a.wait_settled())
                    .await
                    .is_err()
            );
            claimed.release();
            waiters.complete(
                key,
                Ok(CorrectionDisposition::Rejected(
                    CorrectionRejectReason::Stopped,
                )),
            );
            assert_eq!(
                apply_rx.await.unwrap().unwrap(),
                CorrectionDisposition::Rejected(CorrectionRejectReason::Stopped)
            );
            tokio::time::timeout(Duration::from_millis(100), phase_a.wait_settled())
                .await
                .expect("claimed pre-commit waiter settles through its finalizer");
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn phase_a_allows_post_claim_canonical_work_to_finish_before_settlement() {
            let (phase_a, waiters) = phase_a_fixture();
            let sink = Arc::new(HandoffFinalizingSink {
                entered: tokio::sync::Notify::new(),
                release: tokio::sync::Notify::new(),
            });
            let mut finalizer =
                CanonicalPublishFinalizerSupervisor::start(sink.clone(), Arc::clone(&waiters));
            let candidate = sample_candidate(1, 100);
            let cancellation = CorrectionRequestCancellation::new();
            let operation = cancellation.operation();
            let lease = Arc::new(CorrectionRequestLease::new(cancellation, None));
            let key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let apply_rx = waiters
                .register(key, Arc::clone(&lease))
                .expect("admitted exact waiter");
            let claimed = waiters.claim_apply(key).expect("claimed waiter lease");
            let (outcome, outcome_rx) = oneshot::channel();
            finalizer
                .ingress()
                .sender
                .send(CanonicalFinalizerJob {
                    key,
                    candidate,
                    lease: claimed,
                    release_lease_on_completion: true,
                    outcome,
                })
                .await
                .expect("canonical finalizer handoff");
            tokio::time::timeout(Duration::from_secs(1), sink.entered.notified())
                .await
                .expect("canonical operation reached post-claim phase");

            phase_a.begin();

            assert_eq!(operation.phase(), CanonicalOperationPhase::Claimed);
            assert!(
                tokio::time::timeout(Duration::from_millis(25), phase_a.wait_settled())
                    .await
                    .is_err()
            );
            sink.release.notify_one();
            assert_eq!(
                apply_rx.await.unwrap().unwrap(),
                CorrectionDisposition::Applied
            );
            outcome_rx.await.unwrap().unwrap();
            tokio::time::timeout(Duration::from_millis(100), phase_a.wait_settled())
                .await
                .expect("post-claim canonical work settles before Phase A completes");
            assert_eq!(
                operation.phase(),
                CanonicalOperationPhase::CoordinatorFinalized
            );
            finalizer.shutdown().await.unwrap();
        }

        struct AckThenHangSink {
            acked: tokio::sync::Notify,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for AckThenHangSink {
            async fn apply_candidate(
                &self,
                _candidate: CorrectionCandidate,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                let operation = cancellation.operation();
                assert!(operation.try_claim(&cancellation));
                operation.mark_acked_ok()?;
                self.acked.notify_one();
                std::future::pending::<()>().await;
                unreachable!("hung canonical operation cannot resume")
            }
        }

        struct AckErrorOnceApplier {
            fail_next: AtomicBool,
        }

        #[async_trait::async_trait]
        impl CanonicalCorrectionApplier for AckErrorOnceApplier {
            async fn apply_transaction(
                &self,
                _transaction: CanonicalCorrectionTransaction,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<(), CanonicalCorrectionApplyError> {
                let operation = cancellation.operation();
                if !operation.try_claim(&cancellation) {
                    return Err(CanonicalCorrectionApplyError::Cancelled);
                }
                if self.fail_next.swap(false, Ordering::AcqRel) {
                    operation
                        .mark_acked_err()
                        .map_err(CanonicalCorrectionApplyError::Failed)?;
                    return Err(CanonicalCorrectionApplyError::Failed(
                        "injected canonical publish ACK error".to_owned(),
                    ));
                }
                operation
                    .mark_acked_ok()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                operation
                    .mark_state_committed()
                    .map_err(CanonicalCorrectionApplyError::Failed)?;
                Ok(())
            }
        }

        fn test_fail_stop_counter() -> (Arc<std::sync::atomic::AtomicU64>, CanonicalFailStopHook) {
            let calls = Arc::new(std::sync::atomic::AtomicU64::new(0));
            let hook_calls = Arc::clone(&calls);
            let hook = Arc::new(move |_detail: &str| {
                hook_calls.fetch_add(1, Ordering::AcqRel);
            });
            (calls, hook)
        }

        #[test]
        fn canonical_claim_and_cancel_have_exactly_one_winner() {
            for _ in 0..64 {
                let (calls, hook) = test_fail_stop_counter();
                let cancellation = CorrectionRequestCancellation::with_fail_stop(hook);
                let operation = cancellation.operation();
                let barrier = Arc::new(std::sync::Barrier::new(3));
                let cancel_barrier = Arc::clone(&barrier);
                let cancel_token = cancellation.clone();
                let cancel_worker = std::thread::spawn(move || {
                    cancel_barrier.wait();
                    cancel_token.cancel();
                });
                let claim_barrier = Arc::clone(&barrier);
                let claim_token = cancellation.clone();
                let claim_operation = operation.clone();
                let claim_worker = std::thread::spawn(move || {
                    claim_barrier.wait();
                    claim_operation.try_claim(&claim_token)
                });
                barrier.wait();
                cancel_worker.join().unwrap();
                let claimed = claim_worker.join().unwrap();
                if claimed {
                    assert_eq!(operation.phase(), CanonicalOperationPhase::Claimed);
                    operation.mark_acked_err().unwrap();
                    operation.finish_acked_err();
                    cancellation.finish_canonical_commit();
                } else {
                    assert_eq!(operation.phase(), CanonicalOperationPhase::Cancelled);
                }
                assert_eq!(calls.load(Ordering::Acquire), 0);
            }
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn pre_claim_panic_aborts_reservation_releases_lease_and_exact_retry_succeeds() {
            let applier = Arc::new(PanicOnceCanonicalApplier {
                after_claim: false,
                panic_next: AtomicBool::new(true),
            });
            let sink = Arc::new(
                GeometryCorrectionSink::new(
                    owner_identity(),
                    "robot_map",
                    "robot_odom",
                    "robot_body",
                    applier,
                )
                .unwrap(),
            );
            let input = frame(1, 100, 0.0);
            let candidate = candidate_for_frame(&input, 1, Isometry3::identity());
            sink.stage_frame(&input).await.unwrap();
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let guard = Arc::clone(&lifecycle).lock_owned().await;
            let (calls, hook) = test_fail_stop_counter();
            let cancellation = CorrectionRequestCancellation::with_fail_stop(hook);
            let operation = cancellation.operation();
            let lease = Arc::new(CorrectionRequestLease::new(
                cancellation.clone(),
                Some(guard),
            ));
            let key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let waiters = Arc::new(CandidateApplyWaiters::default());
            let apply_rx = waiters.register(key, Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key).unwrap();
            let (outcome, outcome_rx) = oneshot::channel();
            let sink_trait: Arc<dyn CorrectionCandidateSink> = sink.clone();
            let error = run_canonical_finalizer_job(
                CanonicalFinalizerJob {
                    key,
                    candidate: candidate.clone(),
                    lease: claimed_lease,
                    release_lease_on_completion: true,
                    outcome,
                },
                sink_trait,
                &waiters,
            )
            .await
            .expect_err("pre-claim panic is a finalizer terminal");
            assert!(matches!(error, CorrectionRuntimeError::WorkerJoin(_)));
            assert_eq!(operation.phase(), CanonicalOperationPhase::Cancelled);
            assert_eq!(calls.load(Ordering::Acquire), 0);
            assert!(apply_rx.await.unwrap().is_err());
            assert!(outcome_rx.await.unwrap().is_err());
            assert!(lifecycle.try_lock().is_ok(), "request lease was released");
            assert_eq!(
                sink.apply_candidate(candidate, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Applied
            );
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn post_claim_panic_poison_invokes_fail_stop_and_stop_cannot_succeed() {
            let applier = Arc::new(PanicOnceCanonicalApplier {
                after_claim: true,
                panic_next: AtomicBool::new(true),
            });
            let sink = Arc::new(
                GeometryCorrectionSink::new(
                    owner_identity(),
                    "robot_map",
                    "robot_odom",
                    "robot_body",
                    applier,
                )
                .unwrap(),
            );
            let input = frame(1, 100, 0.0);
            let candidate = candidate_for_frame(&input, 1, Isometry3::identity());
            sink.stage_frame(&input).await.unwrap();
            let (calls, hook) = test_fail_stop_counter();
            let cancellation = CorrectionRequestCancellation::with_fail_stop(hook);
            let operation = cancellation.operation();
            let lease = Arc::new(CorrectionRequestLease::new(cancellation, None));
            let key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let waiters = Arc::new(CandidateApplyWaiters::default());
            let _apply_rx = waiters.register(key, Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key).unwrap();
            let (outcome, _outcome_rx) = oneshot::channel();
            let sink_trait: Arc<dyn CorrectionCandidateSink> = sink.clone();
            let error = run_canonical_finalizer_job(
                CanonicalFinalizerJob {
                    key,
                    candidate,
                    lease: claimed_lease,
                    release_lease_on_completion: true,
                    outcome,
                },
                sink_trait,
                &waiters,
            )
            .await
            .expect_err("post-claim panic poisons finalizer");
            assert!(matches!(
                error,
                CorrectionRuntimeError::FinalizerPoisoned(_)
            ));
            assert_eq!(operation.phase(), CanonicalOperationPhase::Poisoned);
            assert_eq!(calls.load(Ordering::Acquire), 1);
            assert!(
                tokio::time::timeout(Duration::from_millis(100), sink.stop())
                    .await
                    .is_err()
            );
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn subscriber_abort_after_handoff_does_not_drop_canonical_future() {
            let sink = Arc::new(HandoffFinalizingSink {
                entered: tokio::sync::Notify::new(),
                release: tokio::sync::Notify::new(),
            });
            let waiters = Arc::new(CandidateApplyWaiters::default());
            let mut supervisor =
                CanonicalPublishFinalizerSupervisor::start(sink.clone(), Arc::clone(&waiters));
            let candidate = sample_candidate(1, 100);
            let (calls, hook) = test_fail_stop_counter();
            let cancellation = CorrectionRequestCancellation::with_fail_stop(hook);
            let operation = cancellation.operation();
            let lease = Arc::new(CorrectionRequestLease::new(cancellation, None));
            let key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let apply_rx = waiters.register(key, Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key).unwrap();
            let (outcome, outcome_rx) = oneshot::channel();
            supervisor
                .ingress()
                .sender
                .send(CanonicalFinalizerJob {
                    key,
                    candidate,
                    lease: claimed_lease,
                    release_lease_on_completion: true,
                    outcome,
                })
                .await
                .unwrap();
            let subscriber = tokio::spawn(outcome_rx);
            tokio::time::timeout(Duration::from_secs(1), sink.entered.notified())
                .await
                .unwrap();
            subscriber.abort();
            assert!(subscriber.await.unwrap_err().is_cancelled());
            sink.release.notify_one();
            assert_eq!(
                apply_rx.await.unwrap().unwrap(),
                CorrectionDisposition::Applied
            );
            assert_eq!(
                operation.phase(),
                CanonicalOperationPhase::CoordinatorFinalized
            );
            assert_eq!(calls.load(Ordering::Acquire), 0);
            supervisor.shutdown().await.unwrap();
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
        async fn acked_publish_not_resumed_triggers_wall_clock_fail_stop() {
            let sink = Arc::new(AckThenHangSink {
                acked: tokio::sync::Notify::new(),
            });
            let waiters = Arc::new(CandidateApplyWaiters::default());
            let mut supervisor =
                CanonicalPublishFinalizerSupervisor::start(sink.clone(), Arc::clone(&waiters));
            let candidate = sample_candidate(1, 100);
            let (calls, hook) = test_fail_stop_counter();
            let cancellation = CorrectionRequestCancellation::with_fail_stop(hook);
            let operation = cancellation.operation();
            let lease = Arc::new(CorrectionRequestLease::new(cancellation, None));
            let key = CandidateApplyKey {
                candidate_sequence: candidate.identity.candidate_sequence,
                payload_sha256: candidate.payload_sha256,
            };
            let apply_rx = waiters.register(key, Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key).unwrap();
            let (outcome, outcome_rx) = oneshot::channel();
            supervisor
                .ingress()
                .sender
                .send(CanonicalFinalizerJob {
                    key,
                    candidate,
                    lease: claimed_lease,
                    release_lease_on_completion: true,
                    outcome,
                })
                .await
                .unwrap();
            tokio::time::timeout(Duration::from_secs(1), sink.acked.notified())
                .await
                .unwrap();
            let worker_result =
                tokio::time::timeout(Duration::from_secs(7), supervisor.worker.as_mut().unwrap())
                    .await
                    .expect("wall-clock watchdog deadline")
                    .expect("finalizer worker joins")
                    .expect_err("poisoned finalizer is terminal");
            supervisor.worker.take();
            assert!(matches!(
                worker_result,
                CorrectionRuntimeError::FinalizerPoisoned(_)
            ));
            assert_eq!(operation.phase(), CanonicalOperationPhase::Poisoned);
            assert_eq!(calls.load(Ordering::Acquire), 1);
            assert!(apply_rx.await.unwrap().is_err());
            assert!(outcome_rx.await.unwrap().is_err());
        }

        #[tokio::test]
        async fn canonical_publish_ack_error_aborts_and_exact_retry_succeeds() {
            let sink = GeometryCorrectionSink::new(
                owner_identity(),
                "robot_map",
                "robot_odom",
                "robot_body",
                Arc::new(AckErrorOnceApplier {
                    fail_next: AtomicBool::new(true),
                }),
            )
            .unwrap();
            let input = frame(1, 100, 0.0);
            let candidate = candidate_for_frame(&input, 1, Isometry3::identity());
            sink.stage_frame(&input).await.unwrap();
            let first = CorrectionRequestCancellation::new();
            assert_eq!(
                sink.apply_candidate(candidate.clone(), first.clone()).await,
                Err("injected canonical publish ACK error".to_owned())
            );
            assert_eq!(first.operation().phase(), CanonicalOperationPhase::AckedErr);
            assert_eq!(sink.correction_revision().await, 0);
            assert_eq!(
                sink.apply_candidate(candidate, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Applied
            );
            assert_eq!(sink.correction_revision().await, 1);
        }

        #[tokio::test]
        async fn repeated_concurrent_stop_is_quiet_and_idempotent() {
            let sink = Arc::new(
                GeometryCorrectionSink::new(
                    owner_identity(),
                    "robot_map",
                    "robot_odom",
                    "robot_body",
                    Arc::new(RecordingCanonicalApplier::new(false)),
                )
                .unwrap(),
            );
            let mut stops = Vec::new();
            for _ in 0..4 {
                let sink = Arc::clone(&sink);
                stops.push(tokio::spawn(async move { sink.stop().await }));
            }
            for stop in stops {
                stop.await.unwrap();
            }
            sink.stop().await;
            assert!(sink.stage_frame(&frame(1, 100, 0.0)).await.is_err());
        }

        #[tokio::test]
        async fn supervisor_panic_is_retained_owner_terminal() {
            let (candidate_tx, _candidate_rx) = mpsc::unbounded_channel();
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::None,
                owner_identity(),
                "unused-by-none",
                Arc::new(RecordingCandidateSink {
                    sender: candidate_tx,
                }),
            )
            .await
            .unwrap();
            let original = owner.finalizer.worker.take().unwrap();
            original.abort();
            let _ = original.await;
            owner.finalizer.worker = Some(tokio::spawn(async {
                panic!("injected canonical finalizer supervisor panic");
            }));
            let terminal = owner
                .wait_terminal()
                .await
                .expect_err("supervisor panic is an owner terminal");
            assert!(terminal.contains("canonical finalizer join failed"));
            assert!(owner.is_finished());
            let settlement = owner
                .shutdown()
                .await
                .expect("panicked supervisor is joined during shutdown");
            assert!(settlement
                .into_sticky_error()
                .is_some_and(|error| error.contains("canonical finalizer join failed")));
        }

        #[tokio::test]
        async fn correction_cleanup_worker_loss_surfaces_to_owner_terminal() {
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity(),
                "",
                Arc::new(FailingStageSink),
            )
            .await
            .expect("relocalization owner starts");
            owner
                .cleanup
                .worker
                .as_ref()
                .expect("cleanup worker is retained")
                .abort();
            let terminal = tokio::time::timeout(Duration::from_secs(1), owner.wait_terminal())
                .await
                .expect("cleanup loss is observed")
                .expect_err("cleanup worker loss is terminal");
            assert!(terminal.contains("correction cleanup join failed"));
            let settlement = owner
                .shutdown()
                .await
                .expect("cleanup worker loss is settled during shutdown");
            assert!(settlement
                .into_sticky_error()
                .expect("cleanup loss remains sticky")
                .contains("correction cleanup join failed"));
        }

        #[tokio::test]
        async fn correction_cleanup_worker_timeout_surfaces_to_owner_terminal() {
            let sink = Arc::new(ProducerStageSink::default());
            sink.block_discard_sequence.store(31, Ordering::Release);
            let mut owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization {
                    map_frame_id: "robot_map".to_owned(),
                    odom_frame_id: "robot_odom".to_owned(),
                    body_frame_id: "robot_body".to_owned(),
                },
                owner_identity(),
                "",
                sink.clone(),
            )
            .await
            .expect("relocalization owner starts");
            let cleanup = owner.cleanup.ingress();
            let discard =
                tokio::spawn(async move { schedule_discard(&cleanup, frame(31, 0, 0.0)).await });
            tokio::time::timeout(Duration::from_secs(1), sink.discard_started.notified())
                .await
                .expect("cleanup timeout job starts");
            let terminal = tokio::time::timeout(Duration::from_secs(2), owner.wait_terminal())
                .await
                .expect("cleanup timeout is observed")
                .expect_err("cleanup timeout is terminal");
            assert!(terminal.contains("correction cleanup"));
            assert!(matches!(discard.await.expect("discard task joins"), Ok(())));
            let settlement = owner
                .shutdown()
                .await
                .expect("timed-out cleanup worker reaches a settled failure");
            assert!(settlement
                .into_sticky_error()
                .is_some_and(|error| error.contains("correction cleanup")));
        }

        #[tokio::test]
        async fn schedule_discard_direct_fallback_timeout_is_not_swallowed() {
            let sink = Arc::new(ProducerStageSink::default());
            sink.block_discard_sequence.store(41, Ordering::Release);
            let mut cleanup = CorrectionCleanupSupervisor::start(sink.clone());
            let worker = cleanup.worker.take().expect("cleanup worker is retained");
            worker.abort();
            let _ = worker.await;
            let result = schedule_discard(&cleanup.ingress(), frame(41, 0, 0.0)).await;
            assert!(matches!(
                result,
                Err(CorrectionRuntimeError::WorkerStopTimeout(
                    "correction cleanup"
                ))
            ));
        }

        #[test]
        fn relocalization_phase_a_closes_admission_before_target_retire() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let entered = Arc::new(std::sync::Barrier::new(2));
            let release = Arc::new(std::sync::Barrier::new(2));
            phase_a.install_close_after_admission_gate(Arc::clone(&entered), Arc::clone(&release));
            let close_phase_a = phase_a.clone();
            let closer = std::thread::spawn(move || close_phase_a.begin());
            entered.wait();

            let (response, response_rx) = oneshot::channel();
            let waiter_permit = Arc::new(Semaphore::new(1))
                .try_acquire_owned()
                .expect("test waiter permit");
            let capacity_lease = Arc::new(RelocalizationWaiterCapacityLease::new(waiter_permit));
            let (detached_tx, _detached_rx) = mpsc::unbounded_channel();
            let admission = RelocalizationAdmissionRequest {
                request: relocalization_command_request("phase-a-window", 7),
                capacity_lease,
                response,
                mapping_lifecycle_guard: None,
            };
            let actor_phase_a = phase_a.clone();
            let actor_thread = std::thread::spawn(move || {
                let mut actor = RelocalizationCommandActor::new(
                    owner_identity(),
                    "robot_map".to_owned(),
                    "robot_odom".to_owned(),
                    "robot_body".to_owned(),
                );
                let result = actor.handle_admission(admission, &detached_tx, None, &actor_phase_a);
                (actor, result)
            });
            release.wait();
            closer.join().expect("phase-a closer joins");
            let (actor, result) = actor_thread.join().expect("admission actor joins");
            assert!(result.is_ok());
            assert!(matches!(
                response_rx.blocking_recv().expect("admission response"),
                Err(RelocalizationCommandError::Stopped)
            ));
            assert!(actor.ledger.records.is_empty());
            assert_eq!(actor.ledger.next_admission_ordinal, 0);
        }

        #[tokio::test]
        async fn relocalization_seventeenth_same_id_waiter_backpressures_then_replays() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut owner = RelocalizationOwnerSupervisor::start(
                phase_a.clone(),
                owner_identity(),
                "robot_map".to_owned(),
                "robot_odom".to_owned(),
                "robot_body".to_owned(),
            );
            let ingress = owner.ingress();
            let mut request = relocalization_command_request("seventeenth-waiter", 7);
            request.logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            let mut waiters = Vec::new();
            for _ in 0..RELOCALIZATION_MAX_WAITERS {
                waiters.push(
                    ingress
                        .admit(request.clone())
                        .await
                        .expect("bounded waiter admits"),
                );
            }
            let pending_ingress = ingress.clone();
            let pending_request = request.clone();
            let seventeenth =
                tokio::spawn(async move { pending_ingress.admit(pending_request).await });
            tokio::task::yield_now().await;
            assert!(!seventeenth.is_finished(), "17th waiter must backpressure");
            let seventeenth = tokio::time::timeout(Duration::from_secs(1), seventeenth)
                .await
                .expect("stored terminal releases waiter capacity")
                .expect("17th admission task joins")
                .expect("17th waiter replays instead of BUSY");
            let terminal = seventeenth.wait().await.expect("replayed waiter terminal");
            assert_eq!(
                terminal.disposition,
                crate::proto::RelocalizeDisposition::DeadlineExceeded as i32
            );
            phase_a.begin();
            drop(waiters);
            owner
                .shutdown()
                .await
                .expect("ledger-only owner shuts down cleanly");
        }

        #[test]
        fn canonical_production_fail_stop_exits_with_7() {
            const CHILD_ENV: &str = "NUWAX_CANONICAL_FAIL_STOP_EXIT7_CHILD";
            if std::env::var_os(CHILD_ENV).is_some() {
                CanonicalOperationControl::production()
                    .poison("subprocess proof for production fail-stop");
                panic!("production fail-stop returned");
            }
            let status = Command::new(std::env::current_exe().unwrap())
                .arg("--exact")
                .arg(
                    "correction_coordinator::preflight_only_contract::tests::canonical_production_fail_stop_exits_with_7",
                )
                .arg("--nocapture")
                .env(CHILD_ENV, "1")
                .status()
                .expect("spawn current test executable");
            assert_eq!(status.code(), Some(7));
        }

        #[tokio::test]
        async fn relocalization_join_timeout_retains_same_worker_for_retry() {
            let release = Arc::new(Notify::new());
            let worker_release = Arc::clone(&release);
            let mut worker = tokio::spawn(async move {
                worker_release.notified().await;
                Ok::<(), CorrectionRuntimeError>(())
            });

            let first = settle_retained_correction_worker(
                "relocalization command owner",
                &mut worker,
                Instant::now() + Duration::from_millis(20),
            )
            .await;
            assert!(first.is_none());
            assert!(!worker.is_finished());

            release.notify_one();
            let retry = settle_retained_correction_worker(
                "relocalization command owner",
                &mut worker,
                Instant::now() + Duration::from_secs(1),
            )
            .await
            .expect("retry joins the retained worker");
            assert!(retry.is_ok());
        }
    }
}

pub(crate) use preflight_only_contract::{
    body_points_sha256, expected_map_boundary_sha256, odom_pose_sha256,
    relocalization_algorithm_options_sha256_v1, relocalization_deadline_policy_sha256_v1,
    relocalization_source_policy_sha256_v1, validate_relocalization_request_id,
    ActivatedTargetReceiptControl, ActivationJournalEntry, ActivationJournalStatus,
    ActivationJournalStep, ActivationLedgerAdmission, ActivationPublicationProof,
    ActivationReceiptSeed, ActivationToken, CanonicalCorrectionApplier,
    CanonicalCorrectionApplyError, CanonicalCorrectionTransaction, CommittedMapProof,
    CorrectionCandidateBackend, CorrectionFrameIngress, CorrectionFrameInput, CorrectionLeaseId,
    CorrectionOwnerRuntimeIdentity, CorrectionOwnerSpec, CorrectionPhaseAControl,
    CorrectionProcessStopAttempt, CorrectionRequestCancellation, CorrectionRuntimeError,
    CorrectionTransportOwner, CorrelatedOdomProof, FrozenCorrelatedOdom, GeometryCorrectionSink,
    RegisteredCloudProof, RelocalizationCommandError, RelocalizationCommandIngress,
    RelocalizationCommandWaiter, RelocalizationModeFence, RelocalizationTerminalSnapshot,
    RELOCALIZATION_ALGORITHM_OPTIONS_VERSION_V1, RELOCALIZATION_DEADLINE_POLICY_VERSION_V1,
    RELOCALIZATION_SOURCE_POLICY_VERSION_V1,
};

#[cfg(test)]
pub(crate) use preflight_only_contract::{
    CorrectionCandidate, CorrectionCandidateSink, CorrectionDisposition,
};
