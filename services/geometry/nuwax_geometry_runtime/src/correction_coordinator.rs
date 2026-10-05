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
    use nuwax_mapping::loop_closure::options::{PgoOptions, PGO_OPTIONS_ENV};
    use nuwax_mapping::relocalization::{
        evaluate_alignment, relocation_pipeline_with_config, IcpConfig, IcpResult, IcpSolver,
        RelocalizationConfig, RelocalizationResult,
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

    use crate::activation_contract::{
        ActivationIdentity, ActivationJournalKey, ActivationJournalVerifier,
        ActivationJournalVerifierV2, ActivationRequestKey, ActivationTargetSource,
        InstalledActivationIdentityV2, InstalledTargetReceiptV2, TargetReceipt,
    };
    use crate::activation_request::{
        ActivationRequestMetadataV2, CanonicalCheckpointOwnerV2, ValidatedActivationRequestV2,
    };
    use crate::lcm_runtime::ValidatedFrameBindings;
    use crate::map_checksum::sha256_bytes;
    use crate::mapping_commit::MappingCommitId;
    use crate::owner_profile::{CorrectionBackend, LioBackend};

    const CORRECTION_SCHEMA_VERSION_V3: i32 = 3;
    const CORRECTION_BACKEND_LIBRARY_PGO: i32 = 1;
    const CORRECTION_BACKEND_NATIVE_PGO: i32 = 2;
    const CORRECTION_BACKEND_RELOCALIZATION: i32 = 3;
    const CORRECTION_TRANSFORM_ABSOLUTE_LEFT_SE3: i32 = 1;
    const CORRECTION_CANDIDATE_FINGERPRINT_V3: [u8; 8] = 0xcaf86342b91bf47du64.to_be_bytes();
    const CORRECTION_CANDIDATE_MESSAGE_TYPE: &str = "nuwax_msgs.CorrectionCandidate";
    const MAX_IDENTITY_STRING_BYTES: usize = 255;
    const MAX_WIRE_PAYLOAD_BYTES: usize = 256 * 1024;
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
    pub(crate) const RELOCALIZATION_MAX_TERMINAL_BYTES: usize = 65_536;
    pub(crate) const RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES: usize =
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
    pub(crate) const RELOCALIZATION_MAX_FRAMES: usize = 8;
    const RELOCALIZATION_MIN_AGGREGATE_POINTS: usize = 50_000;
    // 原匹配算法的数学输入下限；五万点只是收集目标，不代表几何质量。
    const RELOCALIZATION_MIN_SOURCE_POINTS: usize = 3;
    pub(crate) const RELOCALIZATION_MAX_AGGREGATE_POINTS: usize = 250_000;
    pub(crate) const RELOCALIZATION_MAX_EMITTED_POINTS: usize = 100_000;
    const RELOCALIZATION_MAX_PER_FRAME_F64_BYTES: usize = 2_400_000;
    const RELOCALIZATION_MAX_AGGREGATE_F64_BYTES: usize = 6_000_000;
    pub(crate) const RELOCALIZATION_MAX_PROOF_BYTES: usize = 4_096;
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
    pub(crate) const RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES: usize = 4096;
    const RELOCALIZATION_MAX_INLINE_EVIDENCE_PREIMAGE_BYTES: usize = 4_096;
    pub(crate) const RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES: usize = 16_384;
    pub(crate) const RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES: usize = 4_096;
    const RELOCALIZATION_MAX_CANONICAL_SOURCE_BYTES: usize =
        8 + RELOCALIZATION_MAX_EMITTED_POINTS * 3 * std::mem::size_of::<f64>();
    pub(crate) const RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES: usize =
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
    pub(crate) const RELOCALIZATION_SOURCE_POLICY_VERSION: u32 = 3;
    pub(crate) const RELOCALIZATION_DEADLINE_POLICY_VERSION_V1: u32 = 1;
    pub(crate) const RELOCALIZATION_ALGORITHM_OPTIONS_VERSION: u32 = 2;

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

    /// Existing admitted LIO source receipt, not a content digest.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) struct RegisteredCloudProof {
        pub(crate) backend: i32,
        pub(crate) runtime_generation: u64,
        pub(crate) source_sequence: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) point_count: u64,
    }

    impl RegisteredCloudProof {
        pub(crate) fn from_admitted(source: &crate::lio_ingress::AdmittedRegisteredScan) -> Result<Self, CorrectionOwnerError> {
            let identity = source.source_identity();
            Ok(Self {
                backend: identity.backend,
                runtime_generation: identity.runtime_generation.try_into().map_err(|_| CorrectionOwnerError::InvalidRegisteredCloudProof)?,
                source_sequence: identity.source_sequence.try_into().map_err(|_| CorrectionOwnerError::InvalidRegisteredCloudProof)?,
                source_timestamp_ns: identity.source_timestamp_ns.try_into().map_err(|_| CorrectionOwnerError::InvalidRegisteredCloudProof)?,
                point_count: source.body_points().len().try_into().map_err(|_| CorrectionOwnerError::InvalidRegisteredCloudProof)?,
            })
        }
    }

    #[derive(Clone, Copy, Debug)]
    pub(crate) struct CorrelatedOdomProof {
        pub(crate) source_sequence: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) translation_m: [f64; 3],
        pub(crate) rotation_xyzw: [f64; 4],
    }

    impl CorrelatedOdomProof {
        pub(crate) fn from_registration(binding: &crate::lio_ingress::RegistrationBinding) -> Result<Self, CorrectionOwnerError> {
            let pose = &binding.frozen_odom_from_body;
            let q = pose.rotation.quaternion();
            Ok(Self {
                source_sequence: binding.correlated_odometry.source_sequence.try_into().map_err(|_| CorrectionOwnerError::InvalidCorrelatedOdomProof)?,
                source_timestamp_ns: binding.correlated_odometry.source_timestamp_ns.try_into().map_err(|_| CorrectionOwnerError::InvalidCorrelatedOdomProof)?,
                translation_m: pose.translation.vector.into(),
                rotation_xyzw: [q.i, q.j, q.k, q.w],
            })
        }

        fn matches_pose(&self, pose: &Isometry3<f64>) -> bool {
            let q = pose.rotation.quaternion();
            self.translation_m.iter().zip(pose.translation.vector.iter()).all(|(a,b)| a.to_bits() == b.to_bits())
                && self.rotation_xyzw.iter().zip([q.i,q.j,q.k,q.w]).all(|(a,b)| a.to_bits() == b.to_bits())
        }
    }

    impl PartialEq for CorrelatedOdomProof {
        fn eq(&self, other: &Self) -> bool {
            self.source_sequence == other.source_sequence
                && self.source_timestamp_ns == other.source_timestamp_ns
                && self.translation_m.iter().zip(other.translation_m).all(|(a,b)| a.to_bits() == b.to_bits())
                && self.rotation_xyzw.iter().zip(other.rotation_xyzw).all(|(a,b)| a.to_bits() == b.to_bits())
        }
    }
    impl Eq for CorrelatedOdomProof {}

    #[derive(Clone, Copy, Debug)]
    pub(crate) struct CommittedMapProof {
        pub(crate) backend_kind: crate::mapping_backend::MappingBackendKind,
        pub(crate) backend_generation: u64,
        pub(crate) source_timestamp_ns: u64,
        pub(crate) width: u64,
        pub(crate) height: u64,
        pub(crate) resolution_m: f64,
        pub(crate) origin_x_m: f64,
        pub(crate) origin_y_m: f64,
    }

    impl PartialEq for CommittedMapProof {
        fn eq(&self, other: &Self) -> bool {
            self.backend_kind == other.backend_kind && self.backend_generation == other.backend_generation
                && self.source_timestamp_ns == other.source_timestamp_ns
                && self.width == other.width && self.height == other.height
                && self.resolution_m.to_bits() == other.resolution_m.to_bits()
                && self.origin_x_m.to_bits() == other.origin_x_m.to_bits()
                && self.origin_y_m.to_bits() == other.origin_y_m.to_bits()
        }
    }
    impl Eq for CommittedMapProof {}

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

    // 私有拥有方只在校验完成后接收整份候选；克隆共享原有轨迹分配。
    mod candidate_ownership {
        use super::*;

        #[derive(Clone, Debug)]
        pub(crate) struct CorrectionCandidate(Arc<CorrectionCandidateData>);

        // 仅用于只读 Deref；不提供 Clone、DerefMut 或生产环境的可变拥有方入口。
        #[derive(Debug)]
        pub(crate) struct CorrectionCandidateData {
            pub(super) identity: CorrectionCandidateIdentity,
            pub(super) transform: CorrectionTransform,
            pub(super) trajectory: Vec<native_pgo_codec::OptimizedKeyframe>,
        }

        impl std::ops::Deref for CorrectionCandidate {
            type Target = CorrectionCandidateData;

            fn deref(&self) -> &Self::Target {
                &self.0
            }
        }

        impl PartialEq for CorrectionCandidate {
            fn eq(&self, other: &Self) -> bool {
                Arc::ptr_eq(&self.0, &other.0) || self.0.exactly_matches(&other.0)
            }
        }

        impl Eq for CorrectionCandidate {}

        impl CorrectionCandidate {
            pub(crate) fn seal(
                identity: CorrectionCandidateIdentity,
                transform: CorrectionTransform,
            ) -> Result<Self, CorrectionRejectReason> {
                Self::seal_with_trajectory(identity, transform, Vec::new())
            }

            pub(crate) fn seal_with_trajectory(
                identity: CorrectionCandidateIdentity,
                transform: CorrectionTransform,
                trajectory: Vec<native_pgo_codec::OptimizedKeyframe>,
            ) -> Result<Self, CorrectionRejectReason> {
                let candidate = CorrectionCandidateData {
                    identity,
                    transform,
                    trajectory,
                };
                candidate.validate_fields()?;
                Ok(Self(Arc::new(candidate)))
            }

            pub(crate) fn identity(&self) -> &CorrectionCandidateIdentity {
                &self.0.identity
            }

            #[cfg(test)]
            pub(super) fn owner_for_test(&self) -> &Arc<CorrectionCandidateData> {
                &self.0
            }

            #[cfg(test)]
            pub(super) fn from_unvalidated_test_parts(
                identity: CorrectionCandidateIdentity,
                transform: CorrectionTransform,
                trajectory: Vec<native_pgo_codec::OptimizedKeyframe>,
            ) -> Self {
                Self(Arc::new(CorrectionCandidateData {
                    identity,
                    transform,
                    trajectory,
                }))
            }

            // 反例必须建立新的拥有方，不能修改已经发布或共享的候选。
            #[cfg(test)]
            pub(super) fn edited_for_test(
                &self,
                edit: impl FnOnce(&mut CorrectionCandidateData),
            ) -> Self {
                let mut data = CorrectionCandidateData {
                    identity: self.identity.clone(),
                    transform: self.transform,
                    trajectory: self.trajectory.clone(),
                };
                edit(&mut data);
                Self(Arc::new(data))
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
                if out.len() > MAX_WIRE_PAYLOAD_BYTES - CORRECTION_CANDIDATE_FINGERPRINT_V3.len() {
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
                    Vec::with_capacity(CORRECTION_CANDIDATE_FINGERPRINT_V3.len() + body.len());
                out.extend_from_slice(&CORRECTION_CANDIDATE_FINGERPRINT_V3);
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
                    .split_at_checked(CORRECTION_CANDIDATE_FINGERPRINT_V3.len())
                    .ok_or(CorrectionWireError::Truncated {
                        need: CORRECTION_CANDIDATE_FINGERPRINT_V3.len(),
                        have: encoded.len(),
                    })?;
                if fingerprint != CORRECTION_CANDIDATE_FINGERPRINT_V3 {
                    return Err(CorrectionWireError::Invalid(
                        "correction candidate fingerprint mismatch".into(),
                    ));
                }
                Self::decode_body(body)
            }

            /// Decode a generated LCM body after the transport checked and stripped
            /// the single top-level fingerprint.
            pub(crate) fn decode_body(encoded: &[u8]) -> Result<Self, CorrectionWireError> {
                if encoded.len()
                    > MAX_WIRE_PAYLOAD_BYTES - CORRECTION_CANDIDATE_FINGERPRINT_V3.len()
                {
                    return Err(CorrectionWireError::Invalid(
                        "correction candidate exceeds wire size bound".into(),
                    ));
                }
                let mut cursor = WireCursor::new(encoded);
                let schema_version = cursor.read_i32()?;
                if schema_version != CORRECTION_SCHEMA_VERSION_V3 {
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
                let mapping_backend_kind = match cursor.read_lcm_string("mapping_backend_kind")?.as_str() {
                    "voxel" => crate::mapping_backend::MappingBackendKind::Voxel,
                    "raytracing" => crate::mapping_backend::MappingBackendKind::Raytracing,
                    "slam_toolbox" => crate::mapping_backend::MappingBackendKind::SlamToolbox,
                    _ => return Err(CorrectionWireError::Invalid("unsupported mapping backend receipt".into())),
                };
                let mapping_backend_generation = cursor.read_positive_u64("mapping_backend_generation")?;
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
                    backend: cursor.read_i32()?,
                    runtime_generation: cursor.read_positive_u64("registered_cloud_runtime_generation")?,
                    source_sequence: cursor
                        .read_positive_u64("registered_cloud_source_sequence")?,
                    source_timestamp_ns: cursor
                        .read_nonnegative_u64("registered_cloud_source_timestamp_ns")?,
                    point_count: cursor.read_positive_u64("registered_cloud_point_count")?,

                };
                let correlated_odom = CorrelatedOdomProof {
                    source_sequence: cursor.read_positive_u64("correlated_odom_source_sequence")?,
                    source_timestamp_ns: cursor
                        .read_nonnegative_u64("correlated_odom_source_timestamp_ns")?,
                    translation_m: [cursor.read_f64()?, cursor.read_f64()?, cursor.read_f64()?],
                    rotation_xyzw: [cursor.read_f64()?, cursor.read_f64()?, cursor.read_f64()?, cursor.read_f64()?],
                };
                let committed_map = CommittedMapProof {
                    backend_kind: mapping_backend_kind,
                    backend_generation: mapping_backend_generation,
                    source_timestamp_ns: committed_map_source_timestamp_ns,
                    width: cursor.read_positive_u64("committed_map_width")?,
                    height: cursor.read_positive_u64("committed_map_height")?,
                    resolution_m: cursor.read_f64()?,
                    origin_x_m: cursor.read_f64()?,
                    origin_y_m: cursor.read_f64()?,

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
                let trajectory_start = cursor.offset;
                let trajectory_count = cursor.read_i32()?;
                if !(0..=2000).contains(&trajectory_count) {
                    return Err(CorrectionWireError::Invalid(
                        "invalid trajectory count".into(),
                    ));
                }
                let trajectory_end = cursor.offset + trajectory_count as usize * 64;
                let trajectory = native_pgo_codec::decode_optimized_keyframes(
                    cursor.bytes.get(trajectory_start..trajectory_end).ok_or(
                        CorrectionWireError::Truncated {
                            need: trajectory_end,
                            have: cursor.bytes.len(),
                        },
                    )?,
                )
                .map_err(|error| CorrectionWireError::Invalid(error.to_string()))?;
                cursor.offset = trajectory_end;
                cursor.finish()?;
                let candidate = CorrectionCandidateData {
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
                    trajectory,
                };
                candidate
                    .validate_integrity()
                    .map_err(|reason| CorrectionWireError::Invalid(reason.to_string()))?;
                Ok(Self(Arc::new(candidate)))
            }
        }

        impl CorrectionCandidateData {
            fn exactly_matches(&self, other: &Self) -> bool {
                let left = &self.identity;
                let right = &other.identity;
                left.backend == right.backend
                    && left.transform_semantics == right.transform_semantics
                    && left.runtime_generation == right.runtime_generation
                    && left.backend_generation == right.backend_generation
                    && left.candidate_sequence == right.candidate_sequence
                    && left.mapping_session_id == right.mapping_session_id
                    && left.mapping_session_generation == right.mapping_session_generation
                    && left.map_frame_generation == right.map_frame_generation
                    && left.mapping_commit_id == right.mapping_commit_id
                    && left.occupancy_revision == right.occupancy_revision
                    && left.cloud_revision == right.cloud_revision
                    && left.registered_cloud == right.registered_cloud
                    && left.correlated_odom == right.correlated_odom
                    && left.committed_map == right.committed_map
                    && left.source_timestamp_ns == right.source_timestamp_ns
                    && left.map_frame_id == right.map_frame_id
                    && left.odom_frame_id == right.odom_frame_id
                    && left.body_frame_id == right.body_frame_id
                    && left.points_frame_id == right.points_frame_id
                    && left.correction_lease == right.correction_lease
                    && same_float_bits(
                        &self.transform.translation_m,
                        &other.transform.translation_m,
                    )
                    && same_float_bits(
                        &self.transform.rotation_xyzw,
                        &other.transform.rotation_xyzw,
                    )
                    && self.trajectory.len() == other.trajectory.len()
                    && self
                        .trajectory
                        .iter()
                        .zip(&other.trajectory)
                        .all(|(left, right)| {
                            left.source_timestamp_ns == right.source_timestamp_ns
                                && same_float_bits(&left.translation_m, &right.translation_m)
                                && same_float_bits(&left.rotation_xyzw, &right.rotation_xyzw)
                        })

            }

            pub(super) fn validate_integrity(&self) -> Result<(), CorrectionRejectReason> {
                self.validate_fields()
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
                    || !matches!(cloud.backend, 1 | 2 | 3)
                    || cloud.runtime_generation == 0
                {
                    return Err(CorrectionRejectReason::InvalidRegisteredCloudProof);
                }
                let odom = identity.correlated_odom;
                if odom.source_sequence == 0
                    || odom.source_timestamp_ns > cloud.source_timestamp_ns
                    || odom.translation_m.iter().chain(odom.rotation_xyzw.iter()).any(|value| !value.is_finite())
                    || (odom.rotation_xyzw.iter().map(|value| value * value).sum::<f64>() - 1.0).abs() > QUATERNION_NORM_TOLERANCE
                {
                    return Err(CorrectionRejectReason::InvalidCorrelatedOdomProof);
                }
                let map = identity.committed_map;
                let map_timestamp_is_valid = match identity.backend {
                    CorrectionCandidateBackend::Relocalization => {
                        map.source_timestamp_ns <= cloud.source_timestamp_ns
                    }
                    CorrectionCandidateBackend::LibraryPgo
                    | CorrectionCandidateBackend::NativePgo => {
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
                    || map.backend_generation == 0
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
                    || cloud.runtime_generation > i64::MAX as u64
                    || map.backend_generation > i64::MAX as u64
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
                native_pgo_codec::validate_optimized_keyframes(&self.trajectory).map_err(|_| {
                    CorrectionRejectReason::InvalidIdentity("invalid optimized trajectory")
                })?;
                if self.identity.backend == CorrectionCandidateBackend::Relocalization
                    && !self.trajectory.is_empty()
                {
                    return Err(CorrectionRejectReason::InvalidIdentity(
                        "relocalization cannot rebuild mapping trajectory",
                    ));
                }
                if self.trajectory.last().is_some_and(|frame| {
                    frame.source_timestamp_ns as u64 != self.identity.source_timestamp_ns
                }) {
                    return Err(CorrectionRejectReason::InvalidIdentity(
                        "trajectory does not end at source frame",
                    ));
                }
                Ok(())
            }

            pub(super) fn encode_body_prefix(&self) -> Result<Vec<u8>, CorrectionRejectReason> {
                let identity = &self.identity;
                let mut out = Vec::with_capacity(512);
                out.extend_from_slice(&CORRECTION_SCHEMA_VERSION_V3.to_be_bytes());
                out.extend_from_slice(&identity.backend.wire_code().to_be_bytes());
                out.extend_from_slice(&identity.transform_semantics.wire_code().to_be_bytes());
                encode_positive_i64(identity.runtime_generation, &mut out)?;
                encode_positive_i64(identity.backend_generation, &mut out)?;
                encode_positive_i64(identity.candidate_sequence, &mut out)?;
                encode_lcm_string(&identity.mapping_session_id, &mut out)?;
                encode_positive_i64(identity.mapping_session_generation, &mut out)?;
                encode_lcm_string(identity.committed_map.backend_kind.stable_name(), &mut out)?;
                encode_positive_i64(identity.committed_map.backend_generation, &mut out)?;
                encode_positive_i64(identity.map_frame_generation, &mut out)?;
                out.extend_from_slice(&identity.mapping_commit_id.boot_nonce.to_be_bytes());
                encode_positive_i64(identity.mapping_commit_id.sequence, &mut out)?;
                encode_positive_i64(identity.occupancy_revision, &mut out)?;
                encode_positive_i64(identity.cloud_revision, &mut out)?;
                encode_nonnegative_i64(identity.committed_map.source_timestamp_ns, &mut out)?;
                out.extend_from_slice(&identity.registered_cloud.backend.to_be_bytes());
                encode_positive_i64(identity.registered_cloud.runtime_generation, &mut out)?;
                encode_positive_i64(identity.registered_cloud.source_sequence, &mut out)?;
                encode_nonnegative_i64(identity.registered_cloud.source_timestamp_ns, &mut out)?;
                encode_positive_i64(identity.registered_cloud.point_count, &mut out)?;
                encode_positive_i64(identity.correlated_odom.source_sequence, &mut out)?;
                encode_nonnegative_i64(identity.correlated_odom.source_timestamp_ns, &mut out)?;
                for value in identity.correlated_odom.translation_m { out.extend_from_slice(&value.to_be_bytes()); }
                for value in identity.correlated_odom.rotation_xyzw { out.extend_from_slice(&value.to_be_bytes()); }
                encode_positive_i64(identity.committed_map.width, &mut out)?;
                encode_positive_i64(identity.committed_map.height, &mut out)?;
                out.extend_from_slice(&identity.committed_map.resolution_m.to_be_bytes());
                out.extend_from_slice(&identity.committed_map.origin_x_m.to_be_bytes());
                out.extend_from_slice(&identity.committed_map.origin_y_m.to_be_bytes());
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
                out.extend_from_slice(
                    &native_pgo_codec::encode_optimized_keyframes(&self.trajectory).map_err(
                        |_| CorrectionRejectReason::InvalidIdentity("invalid optimized trajectory"),
                    )?,
                );
                Ok(out)
            }
        }

        fn same_float_bits<const N: usize>(left: &[f64; N], right: &[f64; N]) -> bool {
            left.iter()
                .zip(right)
                .all(|(left, right)| left.to_bits() == right.to_bits())
        }
    }

    pub(crate) use candidate_ownership::CorrectionCandidate;

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

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) enum CorrectionRejectReason {
        Stopped,
        RebuildBudgetExceeded,
        IncompleteScanHistory,
        InvalidIdentity(&'static str),
        NonFiniteTransform,
        TranslationOutOfBounds,
        InvalidRotation,
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
        pub(crate) candidate: CorrectionCandidate,
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

    #[derive(Clone, Debug, PartialEq, Eq)]
    struct ReplayCursor {
        candidate_sequence: u64,
        source_timestamp_ns: u64,
        candidate: CorrectionCandidate,
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
                candidate: candidate.clone(),
            };
            let disposition = self.prepare_inner(candidate, boundary, &cursor);
            if disposition == CorrectionDisposition::Prepared {
                self.pending = Some(cursor);
            }
            CorrectionReceipt {
                candidate_sequence: candidate.identity.candidate_sequence,
                source_timestamp_ns: candidate.identity.source_timestamp_ns,
                candidate: candidate.clone(),
                disposition,
            }
        }

        fn prepare_inner(
            &self,
            candidate: &CorrectionCandidate,
            boundary: &CorrectionBoundary,
            cursor: &ReplayCursor,
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
            if let Some(applied) = self.last_applied.as_ref() {
                if cursor.candidate_sequence == applied.candidate_sequence {
                    return if cursor.candidate == applied.candidate {
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
            if let Some(pending) = self.pending.as_ref() {
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
            if self.pending.as_ref() != Some(&cursor) {
                return Err(CorrectionTransitionError::ReceiptMismatch);
            }
            self.pending = None;
            self.last_applied = Some(cursor.clone());
            Ok(CorrectionReceipt {
                candidate_sequence: cursor.candidate_sequence,
                source_timestamp_ns: cursor.source_timestamp_ns,
                candidate: cursor.candidate,
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
            if self.pending.as_ref() != Some(&cursor) {
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
            if receipt.candidate_sequence != receipt.candidate.identity.candidate_sequence
                || receipt.source_timestamp_ns != receipt.candidate.identity.source_timestamp_ns {
                return Err(CorrectionTransitionError::ReceiptMismatch);
            }
            Ok(Self {
                candidate_sequence: receipt.candidate_sequence,
                source_timestamp_ns: receipt.source_timestamp_ns,
                candidate: receipt.candidate.clone(),
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
            lio_backend: LioBackend,
            frames: &ValidatedFrameBindings,
        ) -> Result<Self, String> {
            let (map_frame_id, odom_frame_id, body_frame_id) = frames.as_parts();
            let mut spec = Self::preflight_for_frames(
                backend, lio_backend, map_frame_id, odom_frame_id, body_frame_id,
            )?;
            if let Self::LibraryPgo { pipeline, .. } = &mut spec {
                // 共同启动脚本已合并配置文件；只在预检捕获，运行中的图不热改参数。
                let raw = match std::env::var(PGO_OPTIONS_ENV) {
                    Ok(raw) => raw,
                    Err(std::env::VarError::NotPresent) => String::new(),
                    Err(error) => return Err(format!("CORRECTION_PREFLIGHT_FAILED {PGO_OPTIONS_ENV}: {error}")),
                };
                *pipeline = PgoOptions::from_json(&raw)
                    .and_then(|options| options.apply_to(*pipeline))
                    .map_err(|error| format!("CORRECTION_PREFLIGHT_FAILED {PGO_OPTIONS_ENV}: {error}"))?;
                tracing::info!(event = "pgo.config.loaded", lio_backend = ?lio_backend,
                    effective_config = ?pipeline, "已捕获库内PGO启动参数，修改配置后重启生效");
            }
            Ok(spec)
        }

        fn preflight_for_frames(
            backend: CorrectionBackend,
            lio_backend: LioBackend,
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
                // 不运行自动回环校正时，仍保留旧地图按需重定位的原拥有方。
                // 只有显式命令才收集匹配来源并提交校正，普通建图不产生候选。
                CorrectionBackend::None => Ok(Self::Relocalization {
                    map_frame_id: map_frame_id.to_owned(),
                    odom_frame_id: odom_frame_id.to_owned(),
                    body_frame_id: body_frame_id.to_owned(),
                }),
                CorrectionBackend::LibraryPgo => {
                    let pipeline = match lio_backend {
                        LioBackend::PointLio => LoopClosurePipelineConfig::for_point_lio(),
                        _ => LoopClosurePipelineConfig::default(),
                    };
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
        pub(crate) cloud_receipt: crate::mapping_commit::CloudCommitReceipt,
        pub(crate) map_frame_generation: u64,
        pub(crate) registered_cloud: RegisteredCloudProof,
        pub(crate) correlated_odom: FrozenCorrelatedOdom,
        pub(crate) committed_map: CommittedMapProof,
        pub(crate) committed_map_occupancy: Arc<[u8]>,
        pub(crate) points_frame_id: String,
        pub(crate) body_points: crate::correction_points::CorrectionPoints,
        /// The private admitted scan that owns these exact points and frozen source metadata.
        pub(crate) source_owner: crate::lio_ingress::AdmittedRegisteredScan,
        /// 同一接纳扫描的静态子集；原始点与来源证明保留，回环算法只消费该子集。
        pub(crate) static_point_indices: Option<Arc<[u32]>>,
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

    impl CorrectionFrameInput {
        pub(crate) fn validate_source_ownership(&self) -> Result<(), CorrectionOwnerError> {
            let binding = self.source_owner.binding();
            if self.registered_cloud != RegisteredCloudProof::from_admitted(&self.source_owner)?
                || self.body_points != *self.source_owner.body_points()
                || !self.body_points.all_finite()
                || self.mapping_session_id != binding.mapping_session_id
                || self.mapping_session_generation != binding.mapping_session_generation
                || self.map_frame_generation != binding.map_frame_generation
                || self.points_frame_id != self.source_owner.body_frame_id()
                || binding.frame_capture_relocalization_epoch != Some(self.frame_capture_relocalization_epoch)
                || self.correlated_odom.proof != CorrelatedOdomProof::from_registration(binding)?
                || !self.correlated_odom.proof.matches_pose(&self.correlated_odom.odom_from_body)
                || !same_isometry_bits(&self.base_anchor_map_from_odom, &binding.base_anchor_map_from_odom)
                || self.cloud_receipt.commit_id.boot_nonce == 0
                || self.cloud_receipt.commit_id.sequence == 0
                || self.cloud_receipt.occupancy_revision == 0
                || self.cloud_receipt.cloud_revision == 0
                || self.cloud_receipt.session_generation != self.mapping_session_generation
                || self.cloud_receipt.cloud_revision != self.cloud_revision
                || self.cloud_receipt.commit_id.boot_nonce != self.mapping_commit_id.boot_nonce
                || self.cloud_receipt.commit_id.sequence > self.mapping_commit_id.sequence
                || self.cloud_receipt.occupancy_revision > self.occupancy_revision {
                return Err(CorrectionOwnerError::InvalidRegisteredCloudProof);
            }
            Ok(())
        }

        pub(crate) fn exactly_matches(&self, other: &Self) -> bool {
            self.mapping_session_id == other.mapping_session_id
                && self.mapping_session_generation == other.mapping_session_generation
                && self.mapping_commit_id == other.mapping_commit_id
                && self.occupancy_revision == other.occupancy_revision
                && self.cloud_revision == other.cloud_revision
                && self.cloud_receipt == other.cloud_receipt
                && self.map_frame_generation == other.map_frame_generation
                && self.registered_cloud == other.registered_cloud
                && self.correlated_odom.proof == other.correlated_odom.proof
                && same_isometry_bits(&self.correlated_odom.odom_from_body, &other.correlated_odom.odom_from_body)
                && self.committed_map == other.committed_map
                && (Arc::ptr_eq(&self.committed_map_occupancy, &other.committed_map_occupancy)
                    || self.committed_map_occupancy == other.committed_map_occupancy)
                && self.points_frame_id == other.points_frame_id
                && self.body_points == other.body_points
                && self.source_owner == other.source_owner
                && self.static_point_indices == other.static_point_indices
                && same_isometry_bits(&self.base_anchor_map_from_odom, &other.base_anchor_map_from_odom)
                && self.frame_capture_relocalization_epoch == other.frame_capture_relocalization_epoch
                && self.relocalization_mode_epoch == other.relocalization_mode_epoch
        }

        fn pgo_points(&self) -> Result<std::borrow::Cow<'_, [Point3<f64>]>, CorrectionOwnerError> {
            let Some(indices) = &self.static_point_indices else {
                return Ok(std::borrow::Cow::Borrowed(&self.body_points));
            };
            if indices.windows(2).any(|pair| pair[0] >= pair[1])
                || indices.last().is_some_and(|index| *index as usize >= self.body_points.len()) {
                return Err(CorrectionOwnerError::InvalidRegisteredCloudProof);
            }
            Ok(std::borrow::Cow::Owned(indices.iter().map(|index| self.body_points[*index as usize]).collect()))
        }
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
        keyframe_source_timestamps_ns: Vec<u64>,
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
                        keyframe_source_timestamps_ns: Vec::new(),
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
            let static_points = input.pgo_points()?;
            if session_transition {
                self.coordinator = LoopClosureCoordinator::new(self.pipeline)
                    .map_err(|error| CorrectionOwnerError::Pipeline(error.to_string()))?;
                self.session = Some(CorrectionSessionIdentity {
                    mapping_session_id: input.mapping_session_id.clone(),
                    mapping_session_generation: input.mapping_session_generation,
                    map_frame_generation: input.map_frame_generation,
                });
                self.first_source_timestamp_ns = None;
                self.keyframe_source_timestamps_ns.clear();
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
                    &static_points,
                )
                .map_err(|error| CorrectionOwnerError::Pipeline(error.to_string()))?;
            if matches!(
                result,
                ProcessFrameResult::KeyframeAdded { .. } | ProcessFrameResult::LoopClosed { .. }
            ) {
                self.keyframe_source_timestamps_ns
                    .push(input.registered_cloud.source_timestamp_ns);
                tracing::info!(
                    event = "pgo.keyframe",
                    source_timestamp_ns = input.registered_cloud.source_timestamp_ns,
                    session_generation = input.mapping_session_generation,
                    keyframe_id = self.keyframe_source_timestamps_ns.len() - 1,
                    point_count = static_points.len(),
                    source_point_count = input.body_points.len(),
                    odom_translation_m = ?input.correlated_odom.odom_from_body.translation.vector,
                    odom_rotation_xyzw = ?input.correlated_odom.odom_from_body.rotation.coords,
                    "记录回环关键帧的原始里程计依据"
                );
            }
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
            let trajectory = self
                .coordinator
                .corrections()
                .keyframes()
                .iter()
                .zip(&self.keyframe_source_timestamps_ns)
                .map(|(frame, stamp)| {
                    let transform = CorrectionTransform::from_isometry(&frame.optimized_pose);
                    native_pgo_codec::OptimizedKeyframe {
                        source_timestamp_ns: *stamp as i64,
                        translation_m: transform.translation_m,
                        rotation_xyzw: transform.rotation_xyzw,
                    }
                })
                .collect();
            let candidate = CorrectionCandidate::seal_with_trajectory(
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
                trajectory,
            )
            .map_err(CorrectionOwnerError::CandidateRejected)?;
            self.next_candidate_sequence = next_candidate_sequence;
            if let Some(record) = self.coordinator.loops().last() {
                tracing::info!(
                    event = "pgo.loop.candidate",
                    candidate_sequence = next_candidate_sequence,
                    source_timestamp_ns = input.registered_cloud.source_timestamp_ns,
                    historic_id = record.from,
                    current_id = record.to,
                    source_inliers = record.source_inliers,
                    target_inliers = record.target_inliers,
                    source_overlap = record.source_overlap,
                    target_overlap = record.target_overlap,
                    mse = record.mse,
                    correction_translation_m = ?correction.translation.vector,
                    correction_rotation_deg = correction.rotation.angle().to_degrees(),
                    correction_rotation_xyzw = ?correction.rotation.coords,
                    measurement_translation_m = ?record.measurement.translation.vector,
                    measurement_rotation_xyzw = ?record.measurement.rotation.coords,
                    odometry_innovation = record.odometry_innovation,
                    "库内回环生成位姿校正候选"
                );
            }
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
            || input.validate_source_ownership().is_err()
        {
            return Err(CorrectionOwnerError::InvalidRegisteredCloudProof);
        }
        if input.correlated_odom.proof.source_sequence == 0
            || input.correlated_odom.proof.source_sequence > i64::MAX as u64
            || input.correlated_odom.proof.source_timestamp_ns > i64::MAX as u64
            || input.correlated_odom.proof.source_timestamp_ns
                > input.registered_cloud.source_timestamp_ns
            || !input.correlated_odom.proof.matches_pose(&input.correlated_odom.odom_from_body)
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
            || map.backend_generation == 0
            || expected_map_cells != Some(input.committed_map_occupancy.len())
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

    fn same_isometry_bits(left: &Isometry3<f64>, right: &Isometry3<f64>) -> bool {
        let a = CorrectionTransform::from_isometry(left);
        let b = CorrectionTransform::from_isometry(right);
        a.translation_m.iter().zip(b.translation_m).all(|(x,y)| x.to_bits() == y.to_bits())
            && a.rotation_xyzw.iter().zip(b.rotation_xyzw).all(|(x,y)| x.to_bits() == y.to_bits())
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
            candidate: CorrectionCandidate,
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
        background_mapping: AtomicBool,
        background_preparing: AtomicBool,
        background_prepare_notify: Notify,
        background_mode: std::sync::OnceLock<(Arc<RelocalizationModeFence>, u64)>,
        // 后台准备完成后才领取；最终处理租约负责释放，提交中毒时继续保留围栏。
        background_lifecycle_guard: parking_lot::Mutex<Option<tokio::sync::OwnedMutexGuard<()>>>,
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
                    background_mapping: AtomicBool::new(false),
                    background_preparing: AtomicBool::new(false),
                    background_prepare_notify: Notify::new(),
                    background_mode: std::sync::OnceLock::new(),
                    background_lifecycle_guard: parking_lot::Mutex::new(None),
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
                    background_mapping: AtomicBool::new(false),
                    background_preparing: AtomicBool::new(false),
                    background_prepare_notify: Notify::new(),
                    background_mode: std::sync::OnceLock::new(),
                    background_lifecycle_guard: parking_lot::Mutex::new(None),
                }),
            }
        }

        pub(crate) fn operation(&self) -> CanonicalOperationControl {
            self.inner.operation.clone()
        }

        pub(crate) fn for_background_mapping() -> Self {
            let cancellation = Self::new();
            cancellation
                .inner
                .background_mapping
                .store(true, Ordering::Release);
            cancellation
        }

        pub(crate) fn is_background_mapping(&self) -> bool {
            self.inner.background_mapping.load(Ordering::Acquire)
        }

        pub(crate) fn begin_background_preparation(&self) {
            assert!(self.is_background_mapping());
            self.inner
                .background_preparing
                .store(true, Ordering::Release);
            self.inner.background_prepare_notify.notify_waiters();
        }

        async fn background_preparation_started(&self) {
            loop {
                let notified = self.inner.background_prepare_notify.notified();
                tokio::pin!(notified);
                notified.as_mut().enable();
                if self.inner.background_preparing.load(Ordering::Acquire) {
                    return;
                }
                notified.await;
            }
        }

        pub(crate) fn retain_background_lifecycle_guard(
            &self,
            guard: tokio::sync::OwnedMutexGuard<()>,
        ) {
            let previous = self.inner.background_lifecycle_guard.lock().replace(guard);
            assert!(previous.is_none(), "后台校正不能重复领取生命周期围栏");
        }

        fn release_background_lifecycle_guard(&self) {
            self.inner.background_lifecycle_guard.lock().take();
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
            ) || self
                .inner
                .background_mode
                .get()
                .is_some_and(|(fence, epoch)| {
                    let mode = fence.snapshot();
                    mode.active || mode.epoch != *epoch
                })
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
        pub(crate) candidate: CorrectionCandidate,
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
        /// 冻结里程计自己的采样时间，可能略早于对应扫描。
        pub(crate) frozen_odom_source_timestamp_ns: u64,
        pub(crate) trajectory: Vec<native_pgo_codec::OptimizedKeyframe>,
    }

    impl CanonicalCorrectionTransaction {
        pub(crate) fn matches_candidate_owner(&self) -> bool {
            let identity = self.candidate.identity();
            self.candidate.validate_integrity().is_ok()
                && self.runtime_generation == identity.runtime_generation
                && self.backend_generation == identity.backend_generation
                && self.backend == identity.backend
                && self.candidate_sequence == identity.candidate_sequence
                && self.mapping_session_id == identity.mapping_session_id
                && self.mapping_session_generation == identity.mapping_session_generation
                && self.mapping_commit_id == identity.mapping_commit_id
                && self.occupancy_revision == identity.occupancy_revision
                && self.cloud_revision == identity.cloud_revision
                && self.map_frame_generation == identity.map_frame_generation
                && self.source_timestamp_ns == identity.source_timestamp_ns
                && self.committed_map_source_timestamp_ns == identity.committed_map.source_timestamp_ns
                && self.map_frame_id == identity.map_frame_id
                && self.odom_frame_id == identity.odom_frame_id
                && self.body_frame_id == identity.body_frame_id
                && self.correction_lease == identity.correction_lease
                && self.frozen_odom_source_timestamp_ns == identity.correlated_odom.source_timestamp_ns
                && identity.correlated_odom.matches_pose(&self.frozen_odom_from_body)
                && correction_transform_isometry(self.candidate.transform)
                    .is_ok_and(|expected| same_isometry_bits(&expected, &self.absolute_left_correction))
                && self.trajectory.len() == self.candidate.trajectory.len()
                && self.trajectory.iter().zip(&self.candidate.trajectory).all(|(left, right)| {
                    left.source_timestamp_ns == right.source_timestamp_ns
                        && left.translation_m.iter().zip(right.translation_m).all(|(a, b)| a.to_bits() == b.to_bits())
                        && left.rotation_xyzw.iter().zip(right.rotation_xyzw).all(|(a, b)| a.to_bits() == b.to_bits())
                })
        }
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
        /// 仅用于领取规范提交前可确认回滚的准备拒绝；不能用于提交结果不确定的失败。
        #[error("correction preparation rejected: {0}")]
        PreparationRejected(CorrectionRejectReason),
        #[error("{0}")]
        Failed(String),
    }

    #[derive(Clone)]
    struct StagedCorrectionFrame {
        // Only the existing staged/in-flight/last-applied slots retain this
        // owner. Its point and occupancy allocations remain shared.
        input: Arc<CorrectionFrameInput>,
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
            loop {
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
                let existing = Arc::clone(&existing.input);
                drop(state);
                // Independent replay may compare a whole grid. Never hold the
                // state mutex while comparing retained immutable payloads.
                let same = existing.exactly_matches(&staged.input);
                let state = self.state.lock().await;
                if state.stop_requested || state.stopped {
                    return Err("correction apply sink is stopped".to_owned());
                }
                if state.in_flight.is_some() {
                    return Err("a correction apply is in flight".to_owned());
                }
                if !state.staged.as_ref().is_some_and(|current| Arc::ptr_eq(&current.input, &existing)) {
                    continue;
                }
                if same { return Ok(()); }
                return Err("a different correction frame is already staged".to_owned());
            }
            state.staged = Some(staged);
            return Ok(());
            }
        }

        async fn discard(&self, input: &CorrectionFrameInput) {
            let existing = self.state.lock().await.staged.as_ref()
                .map(|staged| Arc::clone(&staged.input));
            let Some(existing) = existing else { return; };
            if !existing.exactly_matches(input) { return; }
            let mut state = self.state.lock().await;
            if state.staged.as_ref().is_some_and(|staged| Arc::ptr_eq(&staged.input, &existing)) {
                state.staged = None;
            }
        }

        pub(crate) async fn correction_revision(&self) -> u64 {
            self.state.lock().await.correction_revision
        }

        fn finalize_stop_locked(state: &mut GeometryCorrectionState) {
            if state.stop_requested && state.in_flight.is_none() && !state.stopped {
                state.staged = None;
                state.last_applied = None;
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
                    candidate: candidate.clone(),
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
                    frozen_odom_source_timestamp_ns: candidate
                        .identity
                        .correlated_odom
                        .source_timestamp_ns,
                    trajectory: candidate.trajectory.clone(),
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
                (Err(CanonicalCorrectionApplyError::PreparationRejected(reason)), phase)
                    if !phase.is_post_claim() =>
                {
                    operation.cancel_pending();
                    match state.coordinator.abort(&reservation.prepared) {
                        Ok(()) => {
                            state.staged = None;
                            tracing::warn!(
                                event = "geometry.correction.preparation_rejected",
                                %reason,
                                correction_revision = state.correction_revision,
                                "校正准备已回滚，保留当前有效地图与位姿"
                            );
                            Ok(CorrectionDisposition::Rejected(reason))
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
            || input.validate_source_ownership().is_err()
            || input
                .committed_map
                .width
                .checked_mul(input.committed_map.height)
                .and_then(|cells| usize::try_from(cells).ok())
                != Some(input.committed_map_occupancy.len())

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
            input: Arc::new(input.clone()),
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
        runtime_generation: u64,
        backend_generation: u64,
        lease_boot_nonce: u128,
        lease_generation: u64,
        candidate_sequence: u64,
    }

    impl From<&CorrectionCandidate> for CandidateApplyKey {
        fn from(candidate: &CorrectionCandidate) -> Self {
            let identity = &candidate.identity;
            Self { runtime_generation: identity.runtime_generation,
                backend_generation: identity.backend_generation,
                lease_boot_nonce: identity.correction_lease.boot_nonce,
                lease_generation: identity.correction_lease.generation,
                candidate_sequence: identity.candidate_sequence }
        }
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
            self.cancellation.release_background_lifecycle_guard();
        }

        fn has_lifecycle_guard(&self) -> bool {
            self.lifecycle_guard.lock().is_some()
        }
    }

    struct CandidateApplyWaiter {
        candidate: CorrectionCandidate,
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
            candidate: CorrectionCandidate,
            lease: Arc<CorrectionRequestLease>,
        ) -> Result<oneshot::Receiver<CandidateApplyResult>, String> {
            if key != CandidateApplyKey::from(&candidate) {
                return Err("correction apply waiter receipt differs from its owner".into());
            }
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
                    candidate,
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

        fn claim_apply(&self, key: CandidateApplyKey, candidate: &CorrectionCandidate) -> Option<Arc<CorrectionRequestLease>> {
            let mut pending = self.pending.lock();
            let waiter = pending.get_mut(&key)?;
            if waiter.candidate != *candidate || waiter.applying {
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
        for value in [RELOCALIZATION_SOURCE_POLICY_VERSION, 1, 1, 8] {
            out.extend_from_slice(&value.to_be_bytes());
        }
        for value in [
            0_u64, 100_000, 2_400_000, 4_096, 25_600_000, 250_000, 6_000_000, 250_000,
            6_000_000, 0, 100_000, 0, 2_400_000, 2_000,
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
        out.extend_from_slice(&RELOCALIZATION_ALGORITHM_OPTIONS_VERSION.to_be_bytes());
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
        canonical_push_string(&mut out, "current-anchor-local-icp-then-global;submit-functional-estimate");
        out.extend_from_slice(&0.8_f64.to_be_bytes());
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

    pub(crate) fn planar_source_policy_sha256()->[u8;32] {
        sha256_bytes(b"nuwax.planar-source.v1;fresh-after-arm;max-age-ns=500000000;accepted-scans=3;strict-sequence;matched-odometry")
    }
    pub(crate) fn planar_algorithm_policy_sha256()->[u8;32] {
        sha256_bytes(b"nuwax.planar-relocalization.v1;karto-2.10.0;explicit-se2-seed;candidate-process;static-graph;canonical-tf")
    }

    pub(crate) use crate::activation_contract::ActivationJournalStep;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum ActivationJournalStatus {
        Prepared,
        Applied,
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) struct ActivationJournalEntry {
        pub(crate) step: ActivationJournalStep,
        pub(crate) idempotency_key: ActivationJournalKey,
        pub(crate) status: ActivationJournalStatus,
        pub(crate) verifier_receipt: Option<ActivationJournalVerifier>,
        pub(crate) typed_failure: Option<String>,
    }

    #[derive(Clone, Debug)]
    pub(crate) struct ActivationReceiptSeed {
        pub(crate) planar_target: Option<Arc<crate::planar_artifacts::Target>>,
        pub(crate) activation_request_id: String,
        pub(crate) request_key: ActivationRequestKey,
        pub(crate) asset_manifest_sha256: Option<[u8; 32]>,
        pub(crate) asset_bundle_sha256: Option<[u8; 32]>,
        pub(crate) modern: bool,
        pub(crate) immutable_target: Option<crate::pointcloud::ImmutableActivatedTarget>,
        pub(crate) checkpoint_owner: Option<CanonicalCheckpointOwnerV2>,
    }

    #[derive(Clone, Debug)]
    pub(crate) struct ActivationPublicationProof {
        pub(crate) source_timestamp_ns: u64,
        pub(crate) cloud_revision: u64,
        pub(crate) point_count: u32,
        pub(crate) voxel_size: f32,
        pub(crate) cloud_receipt: crate::mapping_commit::CloudCommitReceipt,
        pub(crate) target_receipt: TargetReceipt,
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
        pub(crate) immutable_target: Arc<crate::pointcloud::ImmutableActivatedTarget>,
    }

    /// Assigned authority keeps the installed descriptors and the original immutable source
    /// owner together. Capturing or cloning a snapshot never copies checkpoint/cloud bytes.
    /// This internal read surface does not enable assigned relocalization availability.
    #[derive(Clone, Debug)]
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) struct SealedAssignedActivatedTargetSnapshot {
        pub(crate) activation_identity: InstalledActivationIdentityV2,
        pub(crate) target_receipt: InstalledTargetReceiptV2,
        pub(crate) publication_proof: ActivationPublicationProof,
        pub(crate) checkpoint_owner: CanonicalCheckpointOwnerV2,
    }

    impl ActivationPublicationProof {
        fn same_descriptors(&self, other: &Self) -> bool {
            self.source_timestamp_ns == other.source_timestamp_ns
                && self.cloud_revision == other.cloud_revision
                && self.point_count == other.point_count
                && self.voxel_size.to_bits() == other.voxel_size.to_bits()
                && self.cloud_receipt == other.cloud_receipt
                && self.target_receipt == other.target_receipt
        }
    }

    #[derive(Clone, Debug, PartialEq, Eq)]
    pub(crate) struct ActivationToken {
        epoch: u64,
        activation_request_id: String,
        request_key: ActivationRequestKey,
        activation_identity: Option<ActivationIdentity>,
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
        immutable_target: Option<Arc<crate::pointcloud::ImmutableActivatedTarget>>,
        checkpoint_owner: Option<CanonicalCheckpointOwnerV2>,
        cloud_target_required: bool,
        planar_target_required: bool,
        planar_receipt: Option<crate::mapping_commit::OccupancyCommitReceipt>,
        epoch: u64,
        state: PermanentActivationState,
        activation_identity: Option<ActivationIdentity>,
        target_receipt: Option<TargetReceipt>,
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
        #[cfg(test)]
        replay_before_compare_gate: parking_lot::Mutex<Option<ActivationSealTestGate>>,
        #[cfg(test)]
        snapshot_before_recheck_gate: parking_lot::Mutex<Option<ActivationSealTestGate>>,
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
                    #[cfg(test)]
                    replay_before_compare_gate: parking_lot::Mutex::new(None),
                    #[cfg(test)]
                    snapshot_before_recheck_gate: parking_lot::Mutex::new(None),
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
                        && record.seed.request_key == token.request_key
                        && record.activation_identity == token.activation_identity
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
            record.checkpoint_owner = None;
            record.seed.immutable_target = None;
            record.seed.checkpoint_owner = None;
            record.seed.planar_target = None;
            record.planar_receipt = None;
            if let Some(terminal) = record.terminal.as_mut() {
                terminal.relocalization_target_available = false;
            }
        }

        fn has_complete_target_proof(record: &PermanentActivationRecord) -> bool {
            if !record.seed.modern
                || record.session_generation == 0
                || record.backend_generation == 0
                || record.activation_identity.is_none()
            {
                return false;
            }
            match &record.seed.request_key {
                ActivationRequestKey::Assigned(metadata) => {
                    let (
                        Some(owner),
                        Some(ActivationIdentity::Assigned(activation)),
                        Some(TargetReceipt::Assigned(receipt)),
                        Some(proof),
                    ) = (
                        record.checkpoint_owner.as_ref(),
                        record.activation_identity.as_ref(),
                        record.target_receipt.as_ref(),
                        record.publication_proof.as_ref(),
                    )
                    else {
                        return false;
                    };
                    // Only small, immutable descriptors are compared while holding the ledger lock.
                    activation.request == *metadata
                        && activation.lifecycle_epoch == record.epoch
                        && activation.session_generation == record.session_generation
                        && activation.backend_generation == record.backend_generation
                        && receipt.activation == *activation
                        && receipt.validate(owner)
                        && proof.target_receipt == *record.target_receipt.as_ref().unwrap()
                        && proof.cloud_receipt == receipt.cloud_receipt
                        && proof.point_count == owner.cloud().point_count()
                        && proof.voxel_size.to_bits() == owner.cloud().voxel_size().to_bits()
                        && proof.cloud_revision == receipt.cloud_receipt.cloud_revision
                        && proof.source_timestamp_ns == receipt.cloud_receipt.source_timestamp_ns
                }
                ActivationRequestKey::LegacyDigest(_) => {
                    let Some(ActivationIdentity::LegacyDigest(activation)) =
                        record.activation_identity.as_ref()
                    else {
                        return false;
                    };
                    if let Some(target) = record.seed.planar_target.as_ref() {
                        return record.planar_receipt.as_ref().is_some_and(|receipt| {
                            receipt.session_generation == record.session_generation
                                && receipt.backend_generation == record.backend_generation
                                && receipt.backend_kind
                                    == crate::mapping_backend::MappingBackendKind::SlamToolbox
                                && record.target_receipt
                                    == Some(TargetReceipt::LegacyDigest(
                                        target.receipt(*activation, receipt),
                                    ))
                        });
                    }
                    let (Some(target), Some(TargetReceipt::LegacyDigest(receipt)), Some(proof)) = (
                        record.immutable_target.as_ref(),
                        record.target_receipt.as_ref(),
                        record.publication_proof.as_ref(),
                    ) else {
                        return false;
                    };
                    proof.target_receipt == TargetReceipt::LegacyDigest(*receipt)
                        && proof.point_count == target.point_count
                        && proof.voxel_size.to_bits() == target.voxel_size.to_bits()
                        && proof.cloud_receipt.session_generation == record.session_generation
                        && proof.cloud_receipt.cloud_revision == proof.cloud_revision
                        && proof.cloud_receipt.source_timestamp_ns == proof.source_timestamp_ns
                        && proof.cloud_receipt.point_count == target.point_count
                }
            }
        }

        fn sealed_target_available(record: &PermanentActivationRecord) -> bool {
            // Relocalization has not migrated to assigned identities yet. Keep its legacy snapshot
            // and wire proof surface closed even when a schema-2 activation has sealed correctly.
            matches!(
                record.seed.request_key,
                ActivationRequestKey::LegacyDigest(_)
            ) && record.state == PermanentActivationState::Sealed
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

        pub(crate) fn has_request(&self, id: &str) -> bool {
            self.inner.ledger.lock().records.contains_key(id)
        }

        /// Consult permanent small tombstones before admission can reopen any checkpoint files.
        /// Current live staged/sealed records still require exact source comparison at admission.
        pub(crate) fn assigned_terminal_before_admission(
            &self,
            metadata: &ActivationRequestMetadataV2,
            mut state: crate::proto::MappingSessionState,
        ) -> Option<crate::proto::MappingSessionState> {
            let ledger = self.inner.ledger.lock();
            let key = ActivationRequestKey::Assigned(metadata.clone());
            let phase = self.phase_snapshot();
            let record = ledger.records.get(metadata.activation_request_id());
            let conflict = record.is_some_and(|record| record.seed.request_key != key);
            let retired = phase.0
                || record.is_some_and(|record| {
                    !matches!(
                        record.state,
                        PermanentActivationState::Staged | PermanentActivationState::Sealed
                    ) || record.epoch != phase.1
                        || ledger.current.as_deref() != Some(metadata.activation_request_id())
                });
            if !conflict && !retired {
                return None;
            }
            // Format according to the incoming assigned schema even for cross-version conflicts.
            self.apply_request_key(&mut state, metadata.activation_request_id(), &key);
            if !conflict {
                if let Some(record) = record {
                    self.apply_record(
                        &mut state,
                        record,
                        crate::proto::ActivationReceiptDisposition::Retired,
                        false,
                    );
                }
            }
            state.success = false;
            state.idempotent_replay = true;
            state.activation_receipt_disposition = if conflict {
                crate::proto::ActivationReceiptDisposition::Conflict as i32
            } else {
                crate::proto::ActivationReceiptDisposition::Retired as i32
            };
            state.message = if conflict {
                "ACTIVATE_MAP_REPLAY_CONFLICT"
            } else {
                "ACTIVATION_IDENTITY_RETIRED"
            }
            .into();
            Some(state)
        }

        pub(crate) fn stage_assigned_or_replay(
            &self,
            request: ValidatedActivationRequestV2,
            state: crate::proto::MappingSessionState,
            allow_new: bool,
        ) -> ActivationLedgerAdmission {
            let (metadata, checkpoint_owner) = request.into_parts();
            self.stage_or_replay(
                ActivationReceiptSeed {
                    activation_request_id: metadata.activation_request_id().into(),
                    request_key: ActivationRequestKey::Assigned(metadata),
                    asset_manifest_sha256: None,
                    asset_bundle_sha256: None,
                    modern: true,
                    immutable_target: None,
                    planar_target: None,
                    checkpoint_owner: Some(checkpoint_owner),
                },
                state,
                allow_new,
            )
        }

        pub(crate) fn stage_or_replay(
            &self,
            mut seed: ActivationReceiptSeed,
            mut base_state: crate::proto::MappingSessionState,
            allow_new: bool,
        ) -> ActivationLedgerAdmission {
            let seed_valid = match &seed.request_key {
                ActivationRequestKey::LegacyDigest(_) => seed.checkpoint_owner.is_none(),
                ActivationRequestKey::Assigned(metadata) => {
                    seed.modern
                        && seed.activation_request_id == metadata.activation_request_id()
                        && metadata.expected_runtime_generation() == self.runtime_generation()
                        && seed.asset_manifest_sha256.is_none()
                        && seed.asset_bundle_sha256.is_none()
                        && seed.immutable_target.is_none()
                        && seed.planar_target.is_none()
                        && seed
                            .checkpoint_owner
                            .as_ref()
                            .is_some_and(|owner| owner.identity() == metadata.checkpoint_identity())
                }
            };
            if !seed_valid {
                self.apply_seed(&mut base_state, &seed);
                base_state.success = false;
                base_state.activation_receipt_disposition =
                    crate::proto::ActivationReceiptDisposition::Conflict as i32;
                base_state.message = "ACTIVATE_MAP_IDENTITY_SCHEMA_CONFLICT".into();
                return ActivationLedgerAdmission::Terminal(Box::new(base_state));
            }
            loop {
                let before = self.phase_snapshot();
                if before.0 {
                    self.apply_seed(&mut base_state, &seed);
                    base_state.success = false;
                    base_state.idempotent_replay = true;
                    base_state.activation_receipt_disposition =
                        crate::proto::ActivationReceiptDisposition::Retired as i32;
                    base_state.message = "ACTIVATION_IDENTITY_RETIRED".into();
                    return ActivationLedgerAdmission::Terminal(Box::new(base_state));
                }
                // Snapshot only shared owners and small metadata. Byte/file equality must never
                // run under the ledger mutex. Recheck lifecycle/state/owner before admission.
                let replay = {
                    let ledger = self.inner.ledger.lock();
                    ledger
                        .records
                        .get(&seed.activation_request_id)
                        .map(|record| {
                            (
                                record.epoch,
                                record.state,
                                record.seed.request_key.clone(),
                                record.immutable_target.clone(),
                                record.checkpoint_owner.clone(),
                                record.seed.planar_target.clone(),
                            )
                        })
                };
                #[cfg(test)]
                {
                    let gate = self.inner.replay_before_compare_gate.lock().take();
                    if let Some(gate) = gate {
                        gate.entered.wait();
                        gate.release.wait();
                    }
                }
                let exact_source =
                    replay
                        .as_ref()
                        .is_none_or(|(_, state, key, target, owner, planar)| {
                            if key != &seed.request_key {
                                return false;
                            }
                            if !matches!(
                                state,
                                PermanentActivationState::Staged | PermanentActivationState::Sealed
                            ) {
                                return true;
                            }
                            match key {
                                ActivationRequestKey::LegacyDigest(_) => {
                                    *state != PermanentActivationState::Staged
                                        || (target.as_deref() == seed.immutable_target.as_ref()
                                            && planar == &seed.planar_target)
                                }
                                ActivationRequestKey::Assigned(_) => {
                                    match (owner.as_ref(), seed.checkpoint_owner.as_ref()) {
                                        (Some(retained), Some(candidate)) => {
                                            retained.validate_replay(candidate).is_ok()
                                        }
                                        _ => false,
                                    }
                                }
                            }
                        });
                let mut ledger = self.inner.ledger.lock();
                let after = self.phase_snapshot();
                if before != after || after.0 {
                    continue;
                }
                let replay_is_current =
                    ledger.current.as_deref() == Some(seed.activation_request_id.as_str());
                if let Some(record) = ledger.records.get(&seed.activation_request_id) {
                    let Some((epoch, status, key, target, owner, planar)) = replay.as_ref() else {
                        continue;
                    };
                    let source_unchanged = match (&record.checkpoint_owner, owner) {
                        (Some(left), Some(right)) => left.shares_source_with(right),
                        (None, None) => true,
                        _ => false,
                    } && match (&record.immutable_target, target) {
                        (Some(left), Some(right)) => Arc::ptr_eq(left, right),
                        (None, None) => true,
                        _ => false,
                    } && match (&record.seed.planar_target, planar) {
                        (Some(left), Some(right)) => Arc::ptr_eq(left, right),
                        (None, None) => true,
                        _ => false,
                    };
                    if record.epoch != *epoch
                        || record.state != *status
                        || record.seed.request_key != *key
                        || !source_unchanged
                    {
                        continue;
                    }
                    if !exact_source {
                        if matches!(seed.request_key, ActivationRequestKey::Assigned(_)) {
                            self.apply_seed(&mut base_state, &seed);
                            base_state.activation_receipt_disposition =
                                crate::proto::ActivationReceiptDisposition::Conflict as i32;
                        } else {
                            self.apply_record(
                                &mut base_state,
                                record,
                                crate::proto::ActivationReceiptDisposition::Conflict,
                                false,
                            );
                        }
                        base_state.success = false;
                        base_state.idempotent_replay = true;
                        base_state.message = "ACTIVATE_MAP_REPLAY_CONFLICT".into();
                        return ActivationLedgerAdmission::Terminal(Box::new(base_state));
                    }
                    return match record.state {
                        PermanentActivationState::Staged
                            if replay_is_current && record.epoch == after.1 =>
                        {
                            ActivationLedgerAdmission::Execute(ActivationToken {
                                epoch: record.epoch,
                                activation_request_id: seed.activation_request_id,
                                request_key: seed.request_key,
                                activation_identity: record.activation_identity.clone(),
                                reserved_backend_generation: (record.backend_generation > 0)
                                    .then_some(record.backend_generation),
                            })
                        }
                        PermanentActivationState::Sealed => {
                            let mut terminal = record.terminal.clone().unwrap_or_default();
                            let available = replay_is_current
                                && record.epoch == after.1
                                && Self::sealed_target_available(record);
                            self.apply_record(
                                &mut terminal,
                                record,
                                crate::proto::ActivationReceiptDisposition::SealedReplay,
                                available,
                            );
                            terminal.idempotent_replay = true;
                            terminal.message = "ACTIVATION_SEALED_REPLAY".into();
                            ActivationLedgerAdmission::Terminal(Box::new(terminal))
                        }
                        _ => {
                            let mut terminal = record.terminal.clone().unwrap_or(base_state);
                            self.apply_record(
                                &mut terminal,
                                record,
                                crate::proto::ActivationReceiptDisposition::Retired,
                                false,
                            );
                            terminal.success = false;
                            terminal.idempotent_replay = true;
                            terminal.message = "ACTIVATION_IDENTITY_RETIRED".into();
                            ActivationLedgerAdmission::Terminal(Box::new(terminal))
                        }
                    };
                }
                if replay.is_some() {
                    continue;
                }
                if !allow_new {
                    self.apply_seed(&mut base_state, &seed);
                    base_state.success = false;
                    base_state.idempotent_replay = true;
                    base_state.activation_receipt_disposition =
                        crate::proto::ActivationReceiptDisposition::Conflict as i32;
                    base_state.message = "ACTIVATE_MAP_ACTIVE_IDENTITY_CONFLICT".into();
                    return ActivationLedgerAdmission::Terminal(Box::new(base_state));
                }
                let epoch = before.1;
                let request_id = seed.activation_request_id.clone();
                let request_key = seed.request_key.clone();
                let immutable_target = seed.immutable_target.take().map(Arc::new);
                let checkpoint_owner = seed.checkpoint_owner.take();
                if let Some(previous_id) = ledger.current.clone() {
                    if let Some(previous) = ledger.records.get_mut(&previous_id) {
                        self.retire_record(previous, crate::proto::MappingSessionState::default());
                    }
                }
                debug_assert!(ledger
                    .records
                    .values()
                    .all(|record| record.immutable_target.is_none()
                        && record.checkpoint_owner.is_none()));
                ledger.records.insert(
                    request_id.clone(),
                    PermanentActivationRecord {
                        planar_target_required: seed.planar_target.is_some(),
                        planar_receipt: None,
                        cloud_target_required: immutable_target.is_some()
                            || checkpoint_owner.is_some(),
                        seed,
                        immutable_target,
                        checkpoint_owner,
                        epoch,
                        state: PermanentActivationState::Staged,
                        activation_identity: None,
                        target_receipt: None,
                        session_generation: 0,
                        backend_generation: 0,
                        journal: Vec::new(),
                        publication_proof: None,
                        terminal: None,
                    },
                );
                ledger.current = Some(request_id.clone());
                return ActivationLedgerAdmission::Execute(ActivationToken {
                    epoch,
                    activation_request_id: request_id,
                    request_key,
                    activation_identity: None,
                    reserved_backend_generation: None,
                });
            }
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
                if record.backend_generation > 0 && record.backend_generation != backend_generation
                {
                    return None;
                }
                record.backend_generation = backend_generation;
                let mut token = token.clone();
                token.reserved_backend_generation = Some(backend_generation);
                Some(token)
            })
            .flatten()
        }

        pub(crate) fn bind_activation_identity(
            &self,
            token: &ActivationToken,
            activation_identity: impl Into<ActivationIdentity>,
            session_generation: u64,
        ) -> Option<ActivationToken> {
            if session_generation == 0 {
                return None;
            }
            let identity = activation_identity.into();
            self.with_staged_record_mut(token, |record| {
                let valid = match (&record.seed.request_key, &identity) {
                    (
                        ActivationRequestKey::LegacyDigest(_),
                        ActivationIdentity::LegacyDigest(_),
                    ) => true,
                    (
                        ActivationRequestKey::Assigned(request),
                        ActivationIdentity::Assigned(activation),
                    ) => {
                        activation.request == *request
                            && activation.runtime_generation == self.runtime_generation()
                            && activation.lifecycle_epoch == record.epoch
                            && activation.session_generation == session_generation
                            && activation.backend_generation == record.backend_generation
                            && activation.backend_generation > 0
                            && record.checkpoint_owner.as_ref().is_some_and(|owner| {
                                activation.backend_kind == owner.backend_kind()
                            })
                    }
                    _ => false,
                };
                if !valid
                    || record
                        .activation_identity
                        .as_ref()
                        .is_some_and(|bound| bound != &identity)
                {
                    return None;
                }
                record.activation_identity = Some(identity.clone());
                record.session_generation = session_generation;
                let mut token = token.clone();
                token.activation_identity = Some(identity);
                Some(token)
            })
            .flatten()
        }

        pub(crate) fn bind_assigned_activation_identity(
            &self,
            token: &ActivationToken,
            session_generation: u64,
        ) -> Option<ActivationToken> {
            let identity = self
                .with_staged_record(token, |record| {
                    let ActivationRequestKey::Assigned(request) = &record.seed.request_key else {
                        return None;
                    };
                    Some(InstalledActivationIdentityV2 {
                        request: request.clone(),
                        runtime_generation: self.runtime_generation(),
                        lifecycle_epoch: record.epoch,
                        session_generation,
                        backend_kind: record.checkpoint_owner.as_ref()?.backend_kind(),
                        backend_generation: record.backend_generation,
                    })
                })
                .flatten()?;
            self.bind_activation_identity(
                token,
                ActivationIdentity::Assigned(identity),
                session_generation,
            )
        }

        pub(crate) fn activation_identity(
            &self,
            token: &ActivationToken,
        ) -> Option<ActivationIdentity> {
            self.with_staged_record(token, |record| record.activation_identity.clone())
                .flatten()
        }

        pub(crate) fn journal(
            &self,
            token: &ActivationToken,
            entry: ActivationJournalEntry,
        ) -> bool {
            self.with_staged_record_mut(token, |record| {
                let key_matches = match (&record.seed.request_key, &entry.idempotency_key) {
                    (
                        ActivationRequestKey::LegacyDigest(_),
                        ActivationJournalKey::LegacyDigest(_),
                    ) => entry.verifier_receipt.as_ref().is_none_or(|value| {
                        matches!(value, ActivationJournalVerifier::LegacyDigest(_))
                    }),
                    (
                        ActivationRequestKey::Assigned(request),
                        ActivationJournalKey::Assigned {
                            request: keyed,
                            step,
                        },
                    ) => {
                        request == keyed
                            && *step == entry.step
                            && entry
                                .verifier_receipt
                                .as_ref()
                                .is_some_and(|value| match value {
                                    ActivationJournalVerifier::Assigned(verifier) => {
                                        Self::assigned_verifier_matches(record, entry.step, verifier)
                                    }
                                    _ => false,
                                })
                    }
                    _ => false,
                };
                if !key_matches {
                    return false;
                }
                if let Some(existing) = record
                    .journal
                    .iter_mut()
                    .find(|existing| existing.step == entry.step)
                {
                    if existing.idempotency_key != entry.idempotency_key {
                        return false;
                    }
                    if matches!(record.seed.request_key, ActivationRequestKey::Assigned(_))
                        && existing.status == ActivationJournalStatus::Applied
                    {
                        return *existing == entry;
                    }
                    *existing = entry;
                } else {
                    record.journal.push(entry);
                }
                true
            })
            .unwrap_or(false)
        }

        fn assigned_verifier_matches(
            record: &PermanentActivationRecord,
            step: ActivationJournalStep,
            verifier: &ActivationJournalVerifierV2,
        ) -> bool {
            let ActivationRequestKey::Assigned(request) = &record.seed.request_key else {
                return false;
            };
            use ActivationJournalStep as Step;
            use ActivationJournalVerifierV2 as Verifier;
            match (step, verifier) {
                (
                    Step::BackendRestoreSubmission | Step::BackendCommit | Step::RetirementPrepared,
                    Verifier::BackendGeneration(value),
                ) => *value > 0 && *value == record.backend_generation,
                (Step::OccupancyShmBegin, Verifier::SessionGeneration(value)) => {
                    *value > 0 && *value == record.session_generation
                }
                (
                    Step::OccupancyInstall
                    | Step::SharedStateCommit
                    | Step::CacheClears
                    | Step::MapEvent,
                    Verifier::Occupancy(value),
                ) => {
                    value.session_id == request.session_id()
                        && value.session_generation == record.session_generation
                        && value.session_generation > 0
                        && value.backend_generation == record.backend_generation
                        && value.backend_generation > 0
                        && value.commit_id.boot_nonce != 0
                        && value.commit_id.sequence > 0
                        && value.occupancy_revision > 0
                        && value.source_timestamp_ns > 0
                        && record
                            .checkpoint_owner
                            .as_ref()
                            .is_some_and(|owner| owner.backend_kind() == value.backend_kind)
                        && record.publication_proof.as_ref().is_none_or(|proof| {
                            matches!(&proof.target_receipt, TargetReceipt::Assigned(receipt)
                                if receipt.occupancy_receipt == *value)
                        })
                }
                (
                    Step::CloudInstall | Step::CloudEvent | Step::TypedGlobalMap,
                    Verifier::Cloud(value),
                ) => {
                    // Cloud effects occur only after publication proof storage. Match its exact
                    // installed receipt, not merely plausible positive revision/generation fields.
                    record
                        .publication_proof
                        .as_ref()
                        .is_some_and(|proof| proof.cloud_receipt == *value)
                }
                (
                    Step::SessionStart | Step::ActivationRecord | Step::Seal,
                    Verifier::Activation(value),
                ) => {
                    record.activation_identity.as_ref()
                        == Some(&ActivationIdentity::Assigned(value.clone()))
                }
                (Step::TargetReceiptVerification, Verifier::Target(value)) => {
                    record.target_receipt.as_ref() == Some(&TargetReceipt::Assigned(value.clone()))
                }
                // A matching key, another step's valid receipt, or CompletedStep is not proof
                // of this effect. Prepared and Applied both retain the step's typed verifier.
                _ => false,
            }
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
                let valid = match (&record.seed.request_key, &proof.target_receipt) {
                    (ActivationRequestKey::LegacyDigest(_), TargetReceipt::LegacyDigest(_)) => true,
                    (ActivationRequestKey::Assigned(_), TargetReceipt::Assigned(receipt)) => {
                        record.activation_identity.as_ref()
                            == Some(&ActivationIdentity::Assigned(receipt.activation.clone()))
                            && record.checkpoint_owner.as_ref().is_some_and(|owner| {
                                receipt.validate(owner)
                                    && proof.point_count == owner.cloud().point_count()
                                    && proof.voxel_size.to_bits()
                                        == owner.cloud().voxel_size().to_bits()
                            })
                            && proof.cloud_receipt == receipt.cloud_receipt
                            && proof.cloud_revision == receipt.cloud_receipt.cloud_revision
                            && proof.source_timestamp_ns
                                == receipt.cloud_receipt.source_timestamp_ns
                    }
                    _ => false,
                };
                if !valid
                    || record
                        .target_receipt
                        .as_ref()
                        .is_some_and(|stored| stored != &proof.target_receipt)
                {
                    return false;
                }
                record.target_receipt = Some(proof.target_receipt.clone());
                record.publication_proof = Some(proof);
                true
            })
            .unwrap_or(false)
        }

        pub(crate) fn publication_proof(
            &self,
            token: &ActivationToken,
        ) -> Option<ActivationPublicationProof> {
            self.with_staged_record(token, |record| record.publication_proof.clone())
                .flatten()
        }

        pub(crate) fn store_planar_proof(
            &self,
            token: &ActivationToken,
            receipt: crate::mapping_commit::OccupancyCommitReceipt,
        ) -> Option<[u8; 32]> {
            self.with_staged_record_mut(token, |record| {
                let target = record.seed.planar_target.as_ref()?;
                let digest = target.receipt(
                    record.activation_identity.as_ref()?.legacy_digest()?,
                    &receipt,
                );
                record.planar_receipt = Some(receipt);
                record.target_receipt = Some(TargetReceipt::LegacyDigest(digest));
                Some(digest)
            })
            .flatten()
        }
        pub(crate) fn current_sealed_planar_target(
            &self,
        ) -> Option<(
            Arc<crate::planar_artifacts::Target>,
            crate::proto::MappingSessionState,
        )> {
            let before = self.phase_snapshot();
            if before.0 {
                return None;
            }
            let ledger = self.inner.ledger.lock();
            let record = ledger.records.get(ledger.current.as_ref()?)?;
            if self.phase_snapshot() != before
                || record.epoch != before.1
                || !Self::sealed_target_available(record)
            {
                return None;
            }
            Some((record.seed.planar_target.clone()?, record.terminal.clone()?))
        }
        pub(crate) fn immutable_target(
            &self,
            token: &ActivationToken,
        ) -> Option<crate::pointcloud::ImmutableActivatedTarget> {
            // Compatibility API is legacy-only. Its existing copy occurs after releasing the lock.
            self.with_staged_record(token, |record| record.immutable_target.clone())
                .flatten()
                .map(|target| (*target).clone())
        }

        pub(crate) fn checkpoint_owner(
            &self,
            token: &ActivationToken,
        ) -> Option<CanonicalCheckpointOwnerV2> {
            self.with_staged_record(token, |record| record.checkpoint_owner.clone())
                .flatten()
        }

        pub(crate) fn target_source(
            &self,
            token: &ActivationToken,
        ) -> Option<ActivationTargetSource> {
            self.with_staged_record(token, |record| match &record.seed.request_key {
                ActivationRequestKey::LegacyDigest(_) => record
                    .immutable_target
                    .clone()
                    .map(ActivationTargetSource::Legacy),
                ActivationRequestKey::Assigned(_) => record
                    .checkpoint_owner
                    .clone()
                    .map(ActivationTargetSource::Assigned),
            })
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
                let record = ledger.records.get(ledger.current.as_ref()?)?;
                if self.phase_snapshot() != before
                    || record.epoch != before.1
                    || !Self::sealed_target_available(record)
                {
                    return None;
                }
                let terminal = record.terminal.as_ref()?;
                if terminal.session_id.is_empty() || terminal.map_name.is_empty() {
                    return None;
                }
                // Capture only the immutable owner handle and small descriptors. All digest
                // extraction is explicit and fails closed for Assigned variants.
                SealedActivatedTargetSnapshot {
                    runtime_generation: self.runtime_generation(),
                    lifecycle_epoch: record.epoch,
                    session_id: terminal.session_id.clone(),
                    map_name: terminal.map_name.clone(),
                    activation_request_id: record.seed.activation_request_id.clone(),
                    activation_request_identity_sha256: record.seed.request_key.legacy_digest()?,
                    activation_identity_sha256: record
                        .activation_identity
                        .as_ref()?
                        .legacy_digest()?,
                    target_receipt_sha256: record.target_receipt.as_ref()?.legacy_digest()?,
                    asset_manifest_sha256: record.seed.asset_manifest_sha256,
                    asset_bundle_sha256: record.seed.asset_bundle_sha256,
                    session_generation: record.session_generation,
                    backend_generation: record.backend_generation,
                    publication_proof: record.publication_proof.clone()?,
                    immutable_target: record.immutable_target.clone()?,
                }
            };
            #[cfg(test)]
            self.wait_for_snapshot_before_recheck_gate();
            let ledger = self.inner.ledger.lock();
            let current = ledger.current.as_ref()?;
            let record = ledger.records.get(current)?;
            let terminal = record.terminal.as_ref()?;
            if current != &snapshot.activation_request_id
                || self.phase_snapshot() != before
                || record.epoch != snapshot.lifecycle_epoch
                || !Self::sealed_target_available(record)
                || record.seed.activation_request_id != snapshot.activation_request_id
                || record.seed.request_key.legacy_digest()
                    != Some(snapshot.activation_request_identity_sha256)
                || record.seed.asset_manifest_sha256 != snapshot.asset_manifest_sha256
                || record.seed.asset_bundle_sha256 != snapshot.asset_bundle_sha256
                || terminal.session_id != snapshot.session_id
                || terminal.map_name != snapshot.map_name
                || record.session_generation != snapshot.session_generation
                || record.backend_generation != snapshot.backend_generation
                || record
                    .activation_identity
                    .as_ref()
                    .and_then(ActivationIdentity::legacy_digest)
                    != Some(snapshot.activation_identity_sha256)
                || record
                    .target_receipt
                    .as_ref()
                    .and_then(TargetReceipt::legacy_digest)
                    != Some(snapshot.target_receipt_sha256)
                || !record
                    .publication_proof
                    .as_ref()
                    .is_some_and(|proof| proof.same_descriptors(&snapshot.publication_proof))
                || !record
                    .immutable_target
                    .as_ref()
                    .is_some_and(|retained| Arc::ptr_eq(retained, &snapshot.immutable_target))
            {
                return None;
            }
            Some(snapshot)
        }

        #[cfg_attr(not(test), allow(dead_code))]
        pub(crate) fn current_sealed_assigned_target_snapshot(
            &self,
        ) -> Option<SealedAssignedActivatedTargetSnapshot> {
            let before = self.phase_snapshot();
            if before.0 {
                return None;
            }
            let snapshot = {
                let ledger = self.inner.ledger.lock();
                let record = ledger.records.get(ledger.current.as_ref()?)?;
                if self.phase_snapshot() != before
                    || record.epoch != before.1
                    || record.state != PermanentActivationState::Sealed
                    || !Self::has_complete_target_proof(record)
                {
                    return None;
                }
                let activation = record.activation_identity.as_ref()?.assigned()?;
                let terminal = record.terminal.as_ref()?;
                if activation.runtime_generation != self.runtime_generation()
                    || record.seed.activation_request_id
                        != activation.request.activation_request_id()
                    || terminal.session_id != activation.request.session_id()
                    || terminal.map_name != activation.request.map_name()
                    || terminal.session_generation != activation.session_generation
                {
                    return None;
                }
                SealedAssignedActivatedTargetSnapshot {
                    activation_identity: activation.clone(),
                    target_receipt: record.target_receipt.as_ref()?.assigned()?.clone(),
                    publication_proof: record.publication_proof.clone()?,
                    checkpoint_owner: record.checkpoint_owner.clone()?,
                }
            };
            #[cfg(test)]
            self.wait_for_snapshot_before_recheck_gate();
            let ledger = self.inner.ledger.lock();
            let current = ledger.current.as_ref()?;
            let record = ledger.records.get(current)?;
            let terminal = record.terminal.as_ref()?;
            let activation = &snapshot.activation_identity;
            if current != activation.request.activation_request_id()
                || self.phase_snapshot() != before
                || record.epoch != activation.lifecycle_epoch
                || record.state != PermanentActivationState::Sealed
                || !Self::has_complete_target_proof(record)
                || record.seed.activation_request_id != activation.request.activation_request_id()
                || record.seed.request_key.assigned() != Some(&activation.request)
                || terminal.session_id != activation.request.session_id()
                || terminal.map_name != activation.request.map_name()
                || terminal.session_generation != activation.session_generation
                || record
                    .activation_identity
                    .as_ref()
                    .and_then(ActivationIdentity::assigned)
                    != Some(activation)
                || record
                    .target_receipt
                    .as_ref()
                    .and_then(TargetReceipt::assigned)
                    != Some(&snapshot.target_receipt)
                || !record
                    .publication_proof
                    .as_ref()
                    .is_some_and(|proof| proof.same_descriptors(&snapshot.publication_proof))
                || !record
                    .checkpoint_owner
                    .as_ref()
                    .is_some_and(|retained| retained.shares_source_with(&snapshot.checkpoint_owner))
            {
                return None;
            }
            Some(snapshot)
        }

        #[cfg(test)]
        fn wait_for_snapshot_before_recheck_gate(&self) {
            let gate = self.inner.snapshot_before_recheck_gate.lock().take();
            if let Some(gate) = gate {
                gate.entered.wait();
                gate.release.wait();
            }
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
            target_receipt: Option<TargetReceipt>,
            mut state: crate::proto::MappingSessionState,
        ) -> Option<crate::proto::MappingSessionState> {
            #[cfg(test)]
            self.wait_for_seal_before_ledger_gate();
            self.with_staged_record_mut(token, |record| {
                if record
                    .target_receipt
                    .as_ref()
                    .is_some_and(|stored| Some(stored) != target_receipt.as_ref())
                {
                    return None;
                }
                if !matches!(
                    (&record.seed.request_key, &target_receipt),
                    (_, None)
                        | (
                            ActivationRequestKey::LegacyDigest(_),
                            Some(TargetReceipt::LegacyDigest(_))
                        )
                        | (
                            ActivationRequestKey::Assigned(_),
                            Some(TargetReceipt::Assigned(_))
                        )
                ) {
                    return None;
                }
                let modern = record.seed.modern;
                if let ActivationRequestKey::Assigned(request) = &record.seed.request_key {
                    if state.session_id != request.session_id()
                        || state.map_name != request.map_name()
                        || state.session_generation != record.session_generation
                    {
                        return None;
                    }
                }
                // 二维资产仍须完整激活身份；原本携带三维目标的请求不能靠丢失目标降级成功。
                if modern
                    && (record.session_generation == 0
                        || record.backend_generation == 0
                        || record.activation_identity.is_none()
                        || ((record.cloud_target_required || record.planar_target_required)
                            && !Self::has_complete_target_proof(record)))
                {
                    return None;
                }
                record.target_receipt = target_receipt;
                record.state = PermanentActivationState::Sealed;
                let disposition = if modern {
                    crate::proto::ActivationReceiptDisposition::Sealed
                } else {
                    crate::proto::ActivationReceiptDisposition::LegacyUnbound
                };
                let target_available = Self::sealed_target_available(record);
                self.apply_record(&mut state, record, disposition, target_available);
                if !Self::has_complete_target_proof(record) {
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
            self.apply_request_key(&mut state, &token.activation_request_id, &token.request_key);
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
            let before = self.phase_snapshot();
            let ledger = self.inner.ledger.lock();
            let record = ledger
                .current
                .as_ref()
                .and_then(|request_id| ledger.records.get(request_id));
            let after = self.phase_snapshot();
            if before != after {
                if let Some(record) = record {
                    self.apply_record(state, record, crate::proto::ActivationReceiptDisposition::Retired, false);
                } else {
                    self.apply_policy_identity(state);
                }
                state.relocalization_target_available = false;
                state.activation_receipt_disposition =
                    crate::proto::ActivationReceiptDisposition::Retired as i32;
                return;
            }
            let Some(record) = record else {
                self.apply_policy_identity(state);
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
            state.source_policy_version = RELOCALIZATION_SOURCE_POLICY_VERSION;
            state.source_policy_sha256 = relocalization_source_policy_sha256_v1().to_vec();
            state.deadline_policy_version = RELOCALIZATION_DEADLINE_POLICY_VERSION_V1;
            state.deadline_policy_sha256 = relocalization_deadline_policy_sha256_v1().to_vec();
            state.algorithm_options_version = RELOCALIZATION_ALGORITHM_OPTIONS_VERSION;
            state.algorithm_options_sha256 = relocalization_algorithm_options_sha256_v1().to_vec();
        }

        fn apply_request_key(
            &self,
            state: &mut crate::proto::MappingSessionState,
            request_id: &str,
            key: &ActivationRequestKey,
        ) {
            state.activation_request_id = request_id.into();
            state.activation_identity_sha256.clear();
            state.target_receipt_sha256.clear();
            state.asset_manifest_sha256.clear();
            state.asset_bundle_sha256.clear();
            state.target_graph_sha256.clear();
            state.target_kind.clear();
            state.relocalization_target_available = false;
            match key {
                ActivationRequestKey::LegacyDigest(digest) => {
                    self.apply_policy_identity(state);
                    state.activation_request_identity_sha256 = digest.to_vec();
                    state.activation_receipt_v2 = None;
                }
                ActivationRequestKey::Assigned(request) => {
                    // Do not call any application/policy hashing on the assigned path.
                    state.activation_receipt_schema_version = 2;
                    state.runtime_generation = self.runtime_generation();
                    state.activation_request_identity_sha256.clear();
                    state.source_policy_version = 0;
                    state.source_policy_sha256.clear();
                    state.deadline_policy_version = 0;
                    state.deadline_policy_sha256.clear();
                    state.algorithm_options_version = 0;
                    state.algorithm_options_sha256.clear();
                    state.activation_receipt_v2 = Some(crate::proto::ActivationReceiptV2 {
                        request: Some(crate::activation_contract::request_to_wire(request)),
                        activation: None,
                        target: None,
                    });
                }
            }
        }

        fn apply_seed(
            &self,
            state: &mut crate::proto::MappingSessionState,
            seed: &ActivationReceiptSeed,
        ) {
            self.apply_request_key(state, &seed.activation_request_id, &seed.request_key);
            if matches!(seed.request_key, ActivationRequestKey::LegacyDigest(_)) {
                state.asset_manifest_sha256 = seed
                    .asset_manifest_sha256
                    .map(|value| value.to_vec())
                    .unwrap_or_default();
                state.asset_bundle_sha256 = seed
                    .asset_bundle_sha256
                    .map(|value| value.to_vec())
                    .unwrap_or_default();
            }
        }

        fn apply_record(
            &self,
            state: &mut crate::proto::MappingSessionState,
            record: &PermanentActivationRecord,
            disposition: crate::proto::ActivationReceiptDisposition,
            target_available: bool,
        ) {
            self.apply_seed(state, &record.seed);
            state.activation_receipt_disposition = disposition as i32;
            state.backend_generation = record.backend_generation;
            match &record.seed.request_key {
                ActivationRequestKey::LegacyDigest(_) => {
                    state.activation_identity_sha256 = if record.seed.modern {
                        record
                            .activation_identity
                            .as_ref()
                            .and_then(ActivationIdentity::legacy_digest)
                            .map(|value| value.to_vec())
                            .unwrap_or_default()
                    } else {
                        Vec::new()
                    };
                    state.target_receipt_sha256 = record
                        .target_receipt
                        .as_ref()
                        .and_then(TargetReceipt::legacy_digest)
                        .map(|value| value.to_vec())
                        .unwrap_or_default();
                    state.relocalization_target_available = target_available;
                    if let Some(target) = record.seed.planar_target.as_ref() {
                        state.target_kind = "planar_graph".into();
                        state.target_graph_sha256 = target.sha.to_vec();
                    }
                    if record.planar_target_required {
                        state.source_policy_sha256 = planar_source_policy_sha256().to_vec();
                        state.algorithm_options_sha256 = planar_algorithm_policy_sha256().to_vec();
                    }
                }
                ActivationRequestKey::Assigned(_) => {
                    let wire = state
                        .activation_receipt_v2
                        .as_mut()
                        .expect("assigned receipt initialized");
                    wire.activation = match &record.activation_identity {
                        Some(ActivationIdentity::Assigned(value)) => Some(value.to_wire()),
                        _ => None,
                    };
                    wire.target = match &record.target_receipt {
                        Some(TargetReceipt::Assigned(value)) => Some(value.to_wire()),
                        _ => None,
                    };
                }
            }
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

    /// 普通建图的最新校正输入；原串行生产任务消费，不在实时回调中等待历史计算。
    struct BackgroundCorrectionQueue {
        pending: parking_lot::Mutex<Option<CorrectionFrameInput>>,
        ready: Notify,
        mode: Arc<RelocalizationModeFence>,
    }

    impl BackgroundCorrectionQueue {
        fn replace(&self, input: CorrectionFrameInput) {
            let mut pending = self.pending.lock();
            if pending.as_ref().is_some_and(|current| {
                current.mapping_session_generation > input.mapping_session_generation
                    || (current.mapping_session_generation == input.mapping_session_generation
                        && current.registered_cloud.source_timestamp_ns
                            >= input.registered_cloud.source_timestamp_ns)
            }) {
                return;
            }
            *pending = Some(input);
            drop(pending);
            self.ready.notify_one();
        }

        async fn receive(queue: &Option<Arc<Self>>) -> CorrectionFrameRequest {
            let Some(queue) = queue else {
                return std::future::pending().await;
            };
            loop {
                let notified = queue.ready.notified();
                if let Some(input) = queue.pending.lock().take() {
                    let cancellation = CorrectionRequestCancellation::for_background_mapping();
                    let _ = cancellation.inner.background_mode.set((
                        Arc::clone(&queue.mode),
                        input.frame_capture_relocalization_epoch,
                    ));
                    let lease = Arc::new(CorrectionRequestLease::new(cancellation.clone(), None));
                    // 后台输入的最终结果由原拥有方日志报告，没有同步等待者。
                    let (response, _receiver) = oneshot::channel();
                    return CorrectionFrameRequest {
                        input,
                        response,
                        cancellation,
                        lease,
                    };
                }
                notified.await;
            }
        }
    }

    struct CandidateProducerContext {
        background: Option<Arc<BackgroundCorrectionQueue>>,
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
        Local(Option<CorrectionOwnerHandle>),
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
                local => Ok(Self::Local(Some(CorrectionOwnerHandle::start(
                    local, identity,
                )?))),
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
                Self::Local(owner) => {
                    let mut local = owner.take().ok_or_else(|| {
                        CorrectionRuntimeError::WorkerJoin("库内回环拥有方缺失".into())
                    })?;
                    let cancelled = cancellation.clone();
                    // 描述子、配准及图优化都有实际计算成本；保持一个在途拥有方，
                    // 在阻塞工作执行器运行，关闭分支必须等待同一个任务返回。
                    let (restored, result) = tokio::task::spawn_blocking(move || {
                        let result = if cancelled.is_cancelled() {
                            Ok(None)
                        } else {
                            local.submit_frame(input)
                        };
                        (local, result)
                    })
                    .await
                    .map_err(|error| CorrectionRuntimeError::WorkerJoin(error.to_string()))?;
                    *owner = Some(restored);
                    if cancellation.is_cancelled() {
                        return Ok(CandidateProduction::Cancelled);
                    }
                    result
                        .map(|candidate| match candidate {
                            Some(candidate) => CandidateProduction::Candidate {
                                candidate: Box::new(candidate),
                                delivery: CandidateDelivery::PrivateCorrectionTransport,
                            },
                            None => CandidateProduction::NoCandidate,
                        })
                        .map_err(CorrectionRuntimeError::CandidateOwner)
                }
                Self::NativePgo(native) => native.submit_frame(input, cancellation).await,
            }
        }

        async fn shutdown(self) -> Result<(), CorrectionRuntimeError> {
            match self {
                Self::Local(owner) => {
                    // 图和关键帧析构也由原工作拥有方收尾，避免大历史释放占住异步线程。
                    tokio::task::spawn_blocking(move || {
                        if let Some(owner) = owner {
                            owner.shutdown();
                        }
                    })
                    .await
                    .map_err(|error| CorrectionRuntimeError::WorkerJoin(error.to_string()))?;
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
            if input.static_point_indices.as_ref().is_some_and(|indices| indices.is_empty()) {
                input.validate_source_ownership()?;
                return Ok(CandidateProduction::NoCandidate);
            }
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
                    let candidate = CorrectionCandidate::seal_with_trajectory(
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
                        result.trajectory.clone(),
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
        input.validate_source_ownership()?;
        let static_points = input.pgo_points()?;
        let coordinate_count = static_points.len()
            .checked_mul(3)
            .and_then(|count| i32::try_from(count).ok())
            .ok_or_else(|| {
                CorrectionRuntimeError::NativePgo(
                    "registered body coordinate count exceeds i32".to_owned(),
                )
            })?;
        let mut registered_body_points_xyz = Vec::with_capacity(static_points.len() * 3);
        for point in static_points.iter() {
            registered_body_points_xyz.extend_from_slice(&[point.x, point.y, point.z]);
        }
        Ok(native_pgo_codec::NativePgoRequest {
            schema_version: native_pgo_codec::NATIVE_PGO_SCHEMA_VERSION_V3,
            identity: native_pgo_codec::NativePgoRequestIdentity {
                backend: native_pgo_codec::NATIVE_PGO_BACKEND,
                runtime_generation: checked_i64("runtime generation", owner.runtime_generation)?,
                backend_generation: checked_i64("backend generation", owner.backend_generation)?,
                request_sequence: 0,
                mapping_backend_kind: input.committed_map.backend_kind.stable_name().to_owned(),
                mapping_backend_generation: checked_i64("mapping backend generation", input.committed_map.backend_generation)?,
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
                    backend: input.registered_cloud.backend,
                    runtime_generation: checked_i64("LIO runtime generation", input.registered_cloud.runtime_generation)?,
                    source_point_count: checked_i64("admitted cloud point count", input.registered_cloud.point_count)?,
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
                        static_points.len() as u64,
                    )?,
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
                    translation_m: input.correlated_odom.proof.translation_m,
                    rotation_xyzw: input.correlated_odom.proof.rotation_xyzw,
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
        background: Option<Arc<BackgroundCorrectionQueue>>,
        sender: mpsc::Sender<CorrectionFrameRequest>,
        admission_open: Arc<AtomicBool>,
        admission_rx: watch::Receiver<bool>,
        relocalization_mode: Arc<RelocalizationModeFence>,
    }

    impl CorrectionFrameIngress {
        pub(crate) fn supports_background_mapping(&self) -> bool {
            self.background.is_some()
        }

        pub(crate) fn submit_mapping_background(
            &self,
            input: CorrectionFrameInput,
        ) -> Result<(), CorrectionRuntimeError> {
            let _admission_guard = correction_frame_admission_fence().lock();
            if !self.admission_open.load(Ordering::Acquire) {
                return Err(CorrectionRuntimeError::Stopped);
            }
            let mode = self.relocalization_mode.snapshot();
            if mode.active
                || mode.epoch != input.frame_capture_relocalization_epoch
                || input.relocalization_mode_epoch.is_some()
            {
                return Ok(());
            }
            let queue = self
                .background
                .as_ref()
                .ok_or(CorrectionRuntimeError::IngressClosed)?;
            queue.replace(input);
            Ok(())
        }

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
        #[error("invalid relocalization request: {0}")]
        InvalidRequest(String),
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

    pub(crate) struct PlanarCommandOutcome {
        pub disposition:crate::proto::RelocalizeDisposition,
        pub detail:String,
        pub correction_revision:u64,
        pub evidence:Vec<u8>,
    }
    pub(crate) type PlanarCommandWork = Box<dyn FnOnce(PlanarCommandContext)->std::pin::Pin<Box<dyn std::future::Future<Output=PlanarCommandOutcome>+Send>>+Send>;
    struct PlanarClaim {request_id:String,identity:[u8;32],reply:oneshot::Sender<bool>}
    pub(crate) struct PlanarCommandContext {
        pub epoch:u64,
        pub deadline:Instant,
        pub request_identity:[u8;32],
        request_id:String,
        claim:mpsc::Sender<PlanarClaim>,
    }
    impl PlanarCommandContext {
        pub async fn claim(&self)->bool {
            let (tx,rx)=oneshot::channel();
            if self.claim.send(PlanarClaim{request_id:self.request_id.clone(),identity:self.request_identity,reply:tx}).await.is_err() {return false;}
            rx.await.unwrap_or(false)
        }
    }
    struct RelocalizationAdmissionRequest {
        planar_work: Option<PlanarCommandWork>,
        request: crate::proto::RelocalizeRequest,
        capacity_lease: Arc<RelocalizationWaiterCapacityLease>,
        response: oneshot::Sender<Result<RelocalizationCommandWaiter, RelocalizationCommandError>>,
        mapping_lifecycle_guard: Option<tokio::sync::OwnedMutexGuard<()>>,
    }

    struct RelocalizationTerminalQueryRequest {
        request_id: String,
        identity: RelocalizationRequestIdentity,
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
            self.admit_inner(request, None, None).await
        }

        pub(crate) async fn admit_with_lifecycle_guard(
            &self,
            request: crate::proto::RelocalizeRequest,
            lifecycle_guard: tokio::sync::OwnedMutexGuard<()>,
        ) -> Result<RelocalizationCommandWaiter, RelocalizationCommandError> {
            self.admit_inner(request, Some(lifecycle_guard), None).await
        }

        pub(crate) async fn admit_planar(&self,request:crate::proto::RelocalizeRequest,lifecycle:tokio::sync::OwnedMutexGuard<()>,work:PlanarCommandWork)->Result<RelocalizationCommandWaiter,RelocalizationCommandError> {
            self.admit_inner(request,Some(lifecycle),Some(work)).await
        }
        async fn admit_inner(
            &self,
            request: crate::proto::RelocalizeRequest,
            mapping_lifecycle_guard: Option<tokio::sync::OwnedMutexGuard<()>>,
            planar_work: Option<PlanarCommandWork>,
        ) -> Result<RelocalizationCommandWaiter, RelocalizationCommandError> {
            validate_relocalize_request(request.clone()).map_err(RelocalizationCommandError::InvalidRequest)?;
            if planar_work.is_some() { return Err(RelocalizationCommandError::InvalidRequest("RELOCALIZATION_ASSIGNED_TARGET_ABSENT_OR_UNSUPPORTED".into())); }
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
                planar_work,
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
            &self, request_id: &str, request_identity_sha256: [u8; 32], activation_identity_sha256: [u8; 32],
        ) -> Result<RelocalizationTerminalQuerySnapshot, &'static str> {
            self.terminal_snapshot_exact(request_id, RelocalizationRequestIdentity::LegacyV1 {
                request_alias: request_identity_sha256, activation_alias: activation_identity_sha256,
            }).await
        }

        pub(crate) async fn terminal_snapshot_assigned(
            &self, query: &AssignedRelocalizationTerminalQueryV2,
        ) -> Result<RelocalizationTerminalQuerySnapshot, &'static str> {
            self.terminal_snapshot_exact(query.request().request_id(),
                RelocalizationRequestIdentity::Assigned(query.request().clone())).await
        }

        async fn terminal_snapshot_exact(
            &self, request_id: &str, identity: RelocalizationRequestIdentity,
        ) -> Result<RelocalizationTerminalQuerySnapshot, &'static str> {
            let (response, response_rx) = oneshot::channel();
            self.terminal_query_sender.send(RelocalizationTerminalQueryRequest {
                request_id: request_id.to_owned(), identity, response,
            }).await.map_err(|_| "RELOCALIZATION_TERMINAL_QUERY_OWNER_UNAVAILABLE")?;
            response_rx.await.map_err(|_| "RELOCALIZATION_TERMINAL_QUERY_RESPONSE_LOST")?
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
                    let terminal = query_store.snapshot_exact(&query.request_id, &query.identity);
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
                RelocalizationRequestIdentity::LegacyV1 {
                    request_alias: request_identity_sha256, activation_alias: activation_identity_sha256,
                },
            );
        }

        #[cfg(test)]
        pub(crate) fn persist_terminal_query_result_for_tests(&self, result: crate::proto::RelocalizeResult) {
            // Explicit historical test fixture import, never a live V1 writer.
            let request_alias = result.request_identity_sha256.as_slice().try_into().expect("request alias");
            let activation_alias = result.activation_identity_sha256.as_slice().try_into().expect("activation alias");
            let encoded = result.encode_to_vec();
            self.terminal_store.state.lock().terminals.insert(result.request_id.clone(), DurableRelocalizationTerminal {
                identity: RelocalizationRequestIdentity::LegacyV1 { request_alias, activation_alias },
                terminal: StoredRelocalizationResult::LegacyReadOnly(Arc::new(LegacyRelocalizationTerminal { result, encoded })),
            });
        }

        pub(crate) fn is_open(&self) -> bool {
            self.admission_open.load(Ordering::Acquire)
        }
    }

    use crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2;
    use crate::relocalization_terminal_v2::{
        AssignedRelocalizationTerminalV2, AssignedRelocalizationTerminalQueryV2,
        assigned_terminal_filename_v2,
    };

    /// One identity domain in the existing ledger. Legacy values are opaque
    /// read-only aliases; no assigned request is converted into a digest.
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum RelocalizationRequestIdentity {
        LegacyV1 { request_alias: [u8; 32], activation_alias: [u8; 32] },
        Assigned(AssignedCloudRelocalizeRequestIdentityV2),
    }

    #[derive(Clone)]
    struct ValidatedRelocalizeRequest {
        identity: AssignedCloudRelocalizeRequestIdentityV2,
    }
    impl ValidatedRelocalizeRequest {
        fn key(&self) -> RelocalizationRequestIdentity {
            RelocalizationRequestIdentity::Assigned(self.identity.clone())
        }
        fn request_id(&self) -> &str { self.identity.request_id() }
        fn target(&self) -> &InstalledTargetReceiptV2 { self.identity.target() }
        fn timeout_ms(&self) -> u64 { self.identity.logical_timeout_ms() }
    }

    pub(crate) fn validate_relocalization_request_id(value: &str) -> Result<(), String> {
        if value.is_empty() || value.len() > 128 || !value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b':' | b'-')
        }) { return Err("RELOCALIZATION_REQUEST_ID_INVALID".into()); }
        Ok(())
    }
    pub(crate) fn normalize_relocalization_logical_timeout_ms(value: u64) -> Result<u64, String> {
        match value {
            0 => Ok(RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS),
            RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS..=RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS => Ok(value),
            _ => Err("RELOCALIZATION_LOGICAL_TIMEOUT_INVALID".into()),
        }
    }
    fn validate_relocalize_request(request: crate::proto::RelocalizeRequest)
        -> Result<ValidatedRelocalizeRequest, String>
    {
        // The outer availability gate remains closed until the paired caller
        // chain is qualified. This actor never executes legacy live commands.
        AssignedCloudRelocalizeRequestIdentityV2::admit(request)
            .map(|identity| ValidatedRelocalizeRequest { identity })
    }

    #[derive(Clone)]
    enum StoredRelocalizationResult {
        LegacyReadOnly(Arc<LegacyRelocalizationTerminal>),
        Assigned(AssignedRelocalizationTerminalV2),
    }
    struct LegacyRelocalizationTerminal {
        result: crate::proto::RelocalizeResult,
        encoded: Vec<u8>,
    }
    impl StoredRelocalizationResult {
        fn result(&self) -> &crate::proto::RelocalizeResult {
            match self { Self::LegacyReadOnly(value) => &value.result, Self::Assigned(value) => value.result() }
        }
        fn encoded(&self) -> &[u8] {
            match self { Self::LegacyReadOnly(value) => &value.encoded, Self::Assigned(value) => value.encoded() }
        }
        fn to_wire(&self) -> crate::proto::RelocalizeResult { self.result().clone() }
    }


    #[allow(clippy::large_enum_variant)]
    #[derive(Clone, Debug)]
    pub(crate) enum RelocalizationTerminalSnapshot {
        Terminal(crate::proto::RelocalizeResult),
        AssignedTerminal(AssignedRelocalizationTerminalV2),
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
        identity: RelocalizationRequestIdentity,
        terminal: StoredRelocalizationResult,
    }

    struct RelocalizationTerminalStoreState {
        terminals: HashMap<String, DurableRelocalizationTerminal>,
        pending: HashMap<String, RelocalizationRequestIdentity>,
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
            let root =
                PathBuf::from(runtime_data.unwrap_or_else(|| OsString::from("runtime_data")));
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
        let assigned_fields = body.split('.').collect::<Vec<_>>();
        if let [owner, attempt, "assigned-v2"] = assigned_fields.as_slice() {
            return (is_terminal_token(owner, RELOCALIZATION_TERMINAL_OWNER_TOKEN_BYTES)
                && is_terminal_token(attempt, RELOCALIZATION_TERMINAL_ATTEMPT_TOKEN_BYTES))
                .then_some(*owner);
        }
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
                let filename = entry.file_name();
                let filename = filename.to_str().ok_or("RELOCALIZATION_TERMINAL_STORE_FILENAME_INVALID")?;
                let assigned = filename.starts_with("assigned-") || path.extension().and_then(|value| value.to_str()) == Some("terminal");
                if !assigned && path.extension().and_then(|value| value.to_str()) != Some("json") {
                    continue;
                }
                if terminals.len() >= RELOCALIZATION_MAX_COMMAND_RECORDS {
                    return Err("RELOCALIZATION_TERMINAL_STORE_RETENTION_EXCEEDED".into());
                }
                let bytes = read_terminal_record(&path)?;
                let (request_id, durable) = if assigned {
                    let terminal = AssignedRelocalizationTerminalV2::decode_durable_named(
                        filename, &bytes, Some(&validate_assigned_terminal_retained),
                    )?;
                    (terminal.request().request_id().to_owned(), DurableRelocalizationTerminal {
                        identity: RelocalizationRequestIdentity::Assigned(terminal.request().clone()),
                        terminal: StoredRelocalizationResult::Assigned(terminal),
                    })
                } else {
                    read_legacy_relocalization_terminal(&bytes)?
                };
                if terminals.insert(request_id, durable).is_some() {
                    return Err("RELOCALIZATION_TERMINAL_STORE_DUPLICATE".into());
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
            let identity = request.key();
            if let Some(existing) = state.terminals.get(request.request_id()) {
                return if existing.identity == identity { Ok(()) }
                    else { Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into()) };
            }
            if state.pending.get(request.request_id()).is_some_and(|existing| existing != &identity) {
                return Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into());
            }
            state.pending.insert(request.request_id().to_owned(), identity);
            Ok(())
        }

        fn persist_terminal(
            &self, request_id: &str, identity: &RelocalizationRequestIdentity,
            terminal: &StoredRelocalizationResult,
        ) -> Result<(), String> {
            let (RelocalizationRequestIdentity::Assigned(request), StoredRelocalizationResult::Assigned(assigned))
                = (identity, terminal) else {
                return Err("RELOCALIZATION_LEGACY_TERMINAL_WRITE_UNSUPPORTED".into());
            };
            if request.request_id() != request_id || assigned.request() != request {
                return Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into());
            }
            let mut state = self.state.lock();
            if let Some(existing) = state.terminals.get(request_id) {
                return if &existing.identity == identity && existing.terminal.encoded() == terminal.encoded() {
                    Ok(())
                } else { Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into()) };
            }
            if state.terminals.len() >= RELOCALIZATION_MAX_COMMAND_RECORDS {
                return Err("RELOCALIZATION_TERMINAL_STORE_RETENTION_EXCEEDED".into());
            }
            if state.pending.get(request_id).is_some_and(|existing| existing != identity) {
                return Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into());
            }
            let bytes = assigned.encode_durable()?;
            let final_path = self.root.join(assigned_terminal_filename_v2(request_id)?);
            // Existing owner/attempt uniqueness, with no request/content digest.
            let attempt = terminal_owner_token();
            let temporary_path = self.root.join(format!(
                "{RELOCALIZATION_TERMINAL_TEMP_PREFIX}{}.{attempt}.assigned-v2{RELOCALIZATION_TERMINAL_TEMP_SUFFIX}",
                self.owner_token,
            ));
            let mut temporary = OpenOptions::new().write(true).create_new(true).open(&temporary_path)
                .map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_CREATE_FAILED:{error}"))?;
            let write_result = (|| {
                temporary.write_all(&bytes).map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_WRITE_FAILED:{error}"))?;
                temporary.sync_all().map_err(|error| format!("RELOCALIZATION_TERMINAL_STORE_FSYNC_FAILED:{error}"))?;
                drop(temporary);
                match fs::hard_link(&temporary_path, &final_path) {
                    Ok(()) => {},
                    Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
                        if read_terminal_record(&final_path)? != bytes {
                            return Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into());
                        }
                    },
                    Err(error) => return Err(format!("RELOCALIZATION_TERMINAL_STORE_PUBLISH_FAILED:{error}")),
                }
                fsync_directory(&self.root)
            })();
            let cleanup_result = remove_terminal_temporary(&temporary_path, &self.root);
            combine_terminal_persist_results(write_result, cleanup_result)?;
            state.terminals.insert(request_id.to_owned(), DurableRelocalizationTerminal {
                identity: identity.clone(), terminal: terminal.clone(),
            });
            state.pending.remove(request_id);
            Ok(())
        }

        fn snapshot_exact(&self, request_id: &str, identity: &RelocalizationRequestIdentity)
            -> RelocalizationTerminalSnapshot
        {
            let state = self.state.lock();
            if let Some(terminal) = state.terminals.get(request_id) {
                return if &terminal.identity == identity {
                    match &terminal.terminal {
                        StoredRelocalizationResult::LegacyReadOnly(_) => RelocalizationTerminalSnapshot::Terminal(terminal.terminal.to_wire()),
                        StoredRelocalizationResult::Assigned(value) => RelocalizationTerminalSnapshot::AssignedTerminal(value.clone()),
                    }
                } else { RelocalizationTerminalSnapshot::IdentityConflict };
            }
            match state.pending.get(request_id) {
                Some(existing) if existing == identity => RelocalizationTerminalSnapshot::Pending,
                Some(_) => RelocalizationTerminalSnapshot::IdentityConflict,
                None => RelocalizationTerminalSnapshot::NotFound,
            }
        }

        /// Explicit historical read. Aliases are opaque and never recomputed.
        pub(crate) fn snapshot(&self, request_id: &str, request_alias: [u8; 32], activation_alias: [u8; 32])
            -> RelocalizationTerminalSnapshot
        {
            self.snapshot_exact(request_id, &RelocalizationRequestIdentity::LegacyV1 { request_alias, activation_alias })
        }
    }

    /// Reopening legacy bytes checks canonical protobuf shape and stored alias
    /// bindings only. A stored checksum is opaque: this is not content attestation
    /// and no historical V1 record is upgraded to assigned authority.
    fn read_legacy_relocalization_terminal(bytes: &[u8]) -> Result<(String, DurableRelocalizationTerminal), String> {
        let record: DurableRelocalizationTerminalV1 = serde_json::from_slice(bytes)
            .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID")?;
        if record.schema_version != 1 { return Err("RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID".into()); }
        validate_relocalization_request_id(&record.request_id)?;
        let request_alias = decode_hex(&record.request_identity_sha256, 32)?.try_into()
            .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID")?;
        let activation_alias = decode_hex(&record.activation_identity_sha256, 32)?.try_into()
            .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID")?;
        let opaque_checksum = decode_hex(&record.terminal_sha256, 32)?;
        if opaque_checksum.len() != 32 { return Err("RELOCALIZATION_TERMINAL_STORE_RECORD_INVALID".into()); }
        let encoded = decode_hex(&record.terminal_hex, RELOCALIZATION_MAX_TERMINAL_BYTES)?;
        let result = crate::proto::RelocalizeResult::decode(encoded.as_slice())
            .map_err(|_| "RELOCALIZATION_TERMINAL_STORE_TERMINAL_INVALID")?;
        if result.schema_version != 1 || result.assigned_cloud_v2.is_some()
            || result.request_id != record.request_id
            || result.request_identity_sha256.as_slice() != request_alias
            || result.activation_identity_sha256.as_slice() != activation_alias
            || crate::proto::RelocalizeDisposition::try_from(result.disposition).is_err()
            || result.disposition == crate::proto::RelocalizeDisposition::Unspecified as i32 {
            return Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_MISMATCH".into());
        }
        if result.encode_to_vec() != encoded { return Err("RELOCALIZATION_TERMINAL_STORE_NON_CANONICAL".into()); }
        Ok((record.request_id, DurableRelocalizationTerminal {
            identity: RelocalizationRequestIdentity::LegacyV1 { request_alias, activation_alias },
            terminal: StoredRelocalizationResult::LegacyReadOnly(Arc::new(LegacyRelocalizationTerminal { result, encoded })),
        }))
    }


    struct RelocalizationLedgerAdmissionMetadata {
        request: ValidatedRelocalizeRequest,
        admission_ordinal: u64,
        logical_deadline: Instant,
        claim_phase: RelocalizationClaimPhase,
    }

    #[allow(clippy::large_enum_variant)]
    enum RelocalizationLedgerOrigin {
        Admitted(RelocalizationLedgerAdmissionMetadata),
        // Durable V1 terminals do not retain the original request or a
        // process-local Instant. Never reconstruct either from a retry.
        RestoredTerminal,
    }

    struct RelocalizationLedgerRecord {
        origin: RelocalizationLedgerOrigin,
        identity: RelocalizationRequestIdentity,
        waiter_ids: HashSet<u64>,
        terminal: Option<StoredRelocalizationResult>,
        conflicts: Vec<(RelocalizationRequestIdentity, StoredRelocalizationResult)>,
        conflict_overflow: Option<(RelocalizationRequestIdentity, StoredRelocalizationResult)>,
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
        fn from_terminal_store(store: &RelocalizationTerminalStore) -> Self {
            let mut ledger = Self::default();
            // open_at has already verified canonical bytes, exact bindings,
            // uniqueness, and the existing command-record retention bound.
            // Restore into the same bounded ledger before any admission can
            // activate the mode fence, including conflict-first retries.
            ledger.records = store
                .state
                .lock()
                .terminals
                .iter()
                .map(|(request_id, durable)| {
                    (
                        request_id.clone(),
                        RelocalizationLedgerRecord {
                            origin: RelocalizationLedgerOrigin::RestoredTerminal,
                            identity: durable.identity.clone(),
                            waiter_ids: HashSet::new(),
                            terminal: Some(durable.terminal.clone()),
                            conflicts: Vec::new(),
                            conflict_overflow: None,
                        },
                    )
                })
                .collect();
            ledger
        }

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
            if let Some(record) = self.records.get(request.request_id()) {
                if record.identity == request.key()
                {
                    if let Some(terminal) = &record.terminal {
                        return Ok(RelocalizationLedgerAdmission::Replay(terminal.clone()));
                    }
                    let RelocalizationLedgerOrigin::Admitted(metadata) = &record.origin else {
                        return Err("RELOCALIZATION_RESTORED_TERMINAL_MISSING");
                    };
                    let admission_ordinal = metadata.admission_ordinal;
                    let logical_deadline = metadata.logical_deadline;
                    let record = self
                        .records
                        .get_mut(request.request_id())
                        .expect("relocalization record remains present");
                    record.waiter_ids.insert(waiter_id);
                    return Ok(RelocalizationLedgerAdmission::Joined {
                        admission_ordinal,
                        logical_deadline,
                    });
                }
                if let Some(conflict) = record.conflicts.iter().find(|(identity, _)| identity == &request.key()).map(|(_, terminal)| terminal) {
                    return Ok(RelocalizationLedgerAdmission::Conflict(conflict.clone()));
                }
                if let Some((overflow_identity, conflict)) = &record.conflict_overflow {
                    if *overflow_identity == request.key() {
                        return Ok(RelocalizationLedgerAdmission::Conflict(conflict.clone()));
                    }
                    return Ok(RelocalizationLedgerAdmission::CapacityExceeded(
                        "RELOCALIZATION_COMMAND_CONFLICT_OVERFLOW_CLOSED",
                    ));
                }
                if record.conflicts.len() >= RELOCALIZATION_MAX_CONFLICTS_PER_RECORD {
                    let conflict = conflict_result();
                    self.records
                        .get_mut(request.request_id())
                        .expect("relocalization record remains present")
                        .conflict_overflow = Some((request.key(), conflict.clone()));
                    return Ok(RelocalizationLedgerAdmission::ConflictOverflow(conflict));
                }
                let conflict = conflict_result();
                self.records
                    .get_mut(request.request_id())
                    .expect("relocalization record remains present")
                    .conflicts
                    .push((request.key(), conflict.clone()));
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
                .checked_add(Duration::from_millis(request.timeout_ms()))
                .ok_or("RELOCALIZATION_LOGICAL_DEADLINE_OVERFLOW")?;
            self.relocalization_mode.activate()?;
            self.next_admission_ordinal = admission_ordinal;
            self.admitted_command_count = admitted_command_count;
            self.active_request_id = Some(request.request_id().to_owned());
            self.records.insert(
                request.request_id().to_owned(),
                RelocalizationLedgerRecord {
                    origin: RelocalizationLedgerOrigin::Admitted(
                        RelocalizationLedgerAdmissionMetadata {
                            request: request.clone(),
                            admission_ordinal,
                            logical_deadline,
                            claim_phase: RelocalizationClaimPhase::Preclaim,
                        },
                    ),
                    identity: request.key(),
                    waiter_ids: HashSet::from([waiter_id]),
                    terminal: None,
                    conflicts: Vec::new(),
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
            if self.records.contains_key(request.request_id()) {
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
                request.request_id().to_owned(),
                RelocalizationLedgerRecord {
                    origin: RelocalizationLedgerOrigin::Admitted(
                        RelocalizationLedgerAdmissionMetadata {
                            request: request.clone(),
                            admission_ordinal: 0,
                            logical_deadline: now,
                            claim_phase: RelocalizationClaimPhase::Preclaim,
                        },
                    ),
                    identity: request.key(),
                    waiter_ids: HashSet::new(),
                    terminal: Some(terminal),
                    conflicts: Vec::new(),
                    conflict_overflow: None,
                },
            );
            self.admitted_command_count = admitted_command_count;
            Ok(())
        }

        fn mark_postclaim(
            &mut self,
            request_id: &str,
            identity: &RelocalizationRequestIdentity,
        ) -> Result<(), &'static str> {
            let record = self
                .records
                .get_mut(request_id)
                .ok_or("RELOCALIZATION_LEDGER_RECORD_MISSING")?;
            if &record.identity != identity || record.terminal.is_some() {
                return Err("RELOCALIZATION_LEDGER_IDENTITY_MISMATCH");
            }
            let RelocalizationLedgerOrigin::Admitted(metadata) = &mut record.origin else {
                return Err("RELOCALIZATION_RESTORED_TERMINAL_MISSING");
            };
            if metadata.claim_phase == RelocalizationClaimPhase::Preclaim {
                self.claimed_command_count = self
                    .claimed_command_count
                    .checked_add(1)
                    .ok_or("RELOCALIZATION_CLAIMED_COMMAND_COUNT_EXHAUSTED")?;
                metadata.claim_phase = RelocalizationClaimPhase::Postclaim;
            }
            Ok(())
        }

        fn detach_waiter(
            &mut self,
            request_id: &str,
            identity: &RelocalizationRequestIdentity,
            waiter_id: u64,
        ) -> Result<bool, &'static str> {
            let record = self
                .records
                .get_mut(request_id)
                .ok_or("RELOCALIZATION_LEDGER_RECORD_MISSING")?;
            if &record.identity != identity || record.terminal.is_some() {
                return Err("RELOCALIZATION_LEDGER_IDENTITY_MISMATCH");
            }
            let RelocalizationLedgerOrigin::Admitted(metadata) = &record.origin else {
                return Err("RELOCALIZATION_RESTORED_TERMINAL_MISSING");
            };
            record.waiter_ids.remove(&waiter_id);
            Ok(record.waiter_ids.is_empty()
                && metadata.claim_phase == RelocalizationClaimPhase::Preclaim)
        }

        fn settle_terminal(
            &mut self,
            request_id: &str,
            identity: &RelocalizationRequestIdentity,
            terminal: StoredRelocalizationResult,
        ) -> Result<RelocalizationTerminalWaiters, &'static str> {
            let record = self
                .records
                .get_mut(request_id)
                .ok_or("RELOCALIZATION_LEDGER_RECORD_MISSING")?;
            if &record.identity != identity {
                return Err("RELOCALIZATION_LEDGER_IDENTITY_MISMATCH");
            }
            if let Some(stored) = &record.terminal {
                if stored.encoded() == terminal.encoded() {
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
            // Observation counters describe this actor generation. Restored
            // terminals still consume ledger capacity, but cannot count as
            // newly admitted or claimed commands (nor their terminal records).
            let terminal_record_count =
                self.records.values().try_fold(0_u64, |count, record| {
                    if matches!(&record.origin, RelocalizationLedgerOrigin::Admitted(_))
                        && record.terminal.is_some()
                    {
                        count
                            .checked_add(1)
                            .ok_or("RELOCALIZATION_TERMINAL_RECORD_COUNT_EXHAUSTED")
                    } else {
                        Ok(count)
                    }
                })?;
            let pending_record_count = self.records.values().try_fold(0_u64, |count, record| {
                if matches!(&record.origin, RelocalizationLedgerOrigin::Admitted(_))
                    && record.terminal.is_none()
                {
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
            let record_count = u64::try_from(
                self.records
                    .values()
                    .filter(|record| {
                        matches!(&record.origin, RelocalizationLedgerOrigin::Admitted(_))
                    })
                    .count(),
            )
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

        fn request(&self, request_id: &str) -> Option<&ValidatedRelocalizeRequest> {
            match &self.records.get(request_id)?.origin {
                RelocalizationLedgerOrigin::Admitted(metadata) => Some(&metadata.request),
                RelocalizationLedgerOrigin::RestoredTerminal => None,
            }
        }

        fn active_identity(
            &self,
        ) -> Option<(String, RelocalizationRequestIdentity, u64, Instant, RelocalizationClaimPhase)> {
            let request_id = self.active_request_id.as_ref()?;
            let record = self.records.get(request_id)?;
            let RelocalizationLedgerOrigin::Admitted(metadata) = &record.origin else {
                return None;
            };
            Some((
                request_id.clone(),
                record.identity.clone(),
                metadata.admission_ordinal,
                metadata.logical_deadline,
                metadata.claim_phase,
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
        snapshot: SealedAssignedActivatedTargetSnapshot,
        arm_watermark: Option<RelocalizationFrameKey>,
        armed_map_watermarks: HashMap<RelocalizationFrameScope, RelocalizationFrameKey>,
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
            source_receipts: Vec<crate::proto::RelocalizationSourceReceiptV2>,
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
            identity: RelocalizationRequestIdentity,
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
                None,
            )
        }

        fn start_inner(
            phase_a: CorrectionPhaseAControl,
            owner_identity: CorrectionOwnerRuntimeIdentity,
            map_frame_id: String,
            odom_frame_id: String,
            body_frame_id: String,
            runtime: Option<RelocalizationRuntimeContext>,
            prepared_store: Option<RelocalizationTerminalStore>,
        ) -> Self {
            let (sender, receiver) = mpsc::channel(RELOCALIZATION_COMMAND_QUEUE_CAPACITY);
            let (terminal_query_sender, terminal_query_receiver) =
                mpsc::channel(RELOCALIZATION_COMMAND_QUEUE_CAPACITY);
            let (detached_tx, detached_rx) = mpsc::unbounded_channel();
            let terminal_store = prepared_store.unwrap_or_else(||RelocalizationTerminalStore::open()
                .expect("durable relocalization terminal store must open before admission"));
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
        waiter_requests: HashMap<u64, (String, RelocalizationRequestIdentity, Arc<RelocalizationWaiterCapacityLease>)>,
        next_waiter_id: u64,
        owner_identity: CorrectionOwnerRuntimeIdentity,
        map_frame_id: String,
        odom_frame_id: String,
        body_frame_id: String,
        execution: Option<RelocalizationExecution>,
        last_validated_frames: HashMap<RelocalizationFrameScope, RelocalizationFrameKey>,
        validated_frame_proofs:
            HashMap<(RelocalizationFrameScope, RelocalizationFrameKey), std::sync::Weak<CorrectionFrameInput>>,
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
                ledger: RelocalizationCommandLedger::from_terminal_store(&terminal_store),
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
            let terminal = self.terminal_store.snapshot_exact(&query.request_id, &query.identity);
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
                    request.request_id().to_owned(),
                    request.key(),
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
            &self, request: &ValidatedRelocalizeRequest, admission_ordinal: Option<u64>,
            disposition: crate::proto::RelocalizeDisposition, detail: &str,
        ) -> Result<StoredRelocalizationResult, CorrectionRuntimeError> {
            let mut evidence = self.assigned_evidence(request, admission_ordinal, None, None)?;
            if admission_ordinal.is_some() {
                match self.execution.as_ref() {
                    Some(RelocalizationExecution::Accumulating(accumulation)) if accumulation.request.key() == request.key() => {
                        for frame in &accumulation.accepted {
                            evidence.source_frames.push(assigned_source_receipt(&frame.input, &self.map_frame_id,
                                &self.odom_frame_id, &self.body_frame_id).map_err(CorrectionRuntimeError::Apply)?);
                        }
                    },
                    Some(RelocalizationExecution::Computing { request: active, source_receipts, .. }) if active.key() == request.key() => {
                        evidence.source_frames = source_receipts.clone();
                    },
                    Some(RelocalizationExecution::Reserving { computed: Some(computed) }
                        | RelocalizationExecution::Staging { computed: Some(computed), .. }) if computed.request.key() == request.key() => {
                        evidence = self.assigned_evidence(request, admission_ordinal, Some(&computed.source), Some(&computed.output))?;
                    },
                    _ => {},
                }
                if !evidence.source_frames.is_empty() && evidence.algorithm.is_none() {
                    evidence.phase = crate::proto::AssignedRelocalizationEvidencePhaseV2::SourceSelected as i32;
                }
            }
            let result = assigned_relocalization_result(&request.identity, disposition, 0, detail, Vec::new(), evidence);
            AssignedRelocalizationTerminalV2::new(result, Some(&validate_assigned_terminal_retained))
                .map(StoredRelocalizationResult::Assigned).map_err(CorrectionRuntimeError::Apply)
        }

        fn busy_result(&self, request: &ValidatedRelocalizeRequest)
            -> Result<crate::proto::RelocalizeResult, CorrectionRuntimeError>
        {
            Ok(self.terminal_result(request, None, crate::proto::RelocalizeDisposition::Busy,
                "RELOCALIZATION_OWNER_BUSY")?.to_wire())
        }

        fn settle_terminal_durable(
            &mut self, request_id: &str, identity: &RelocalizationRequestIdentity,
            terminal: StoredRelocalizationResult,
        ) -> Result<RelocalizationTerminalWaiters, CorrectionRuntimeError> {
            let request = self.ledger.request(request_id).ok_or_else(||
                CorrectionRuntimeError::Apply("relocalization request missing before durable terminal".into()))?;
            if &request.key() != identity { return Err(CorrectionRuntimeError::Apply("relocalization durable identity mismatch".into())); }
            self.terminal_store.persist_terminal(request_id, identity, &terminal).map_err(CorrectionRuntimeError::Apply)?;
            self.ledger.settle_terminal(request_id, identity, terminal)
                .map_err(|detail| CorrectionRuntimeError::Apply(detail.into()))
        }

        fn notify_terminal(&mut self, terminal: RelocalizationTerminalWaiters) {
            for waiter_id in terminal.waiter_ids {
                if let Some((_, _, capacity_lease)) = self.waiter_requests.remove(&waiter_id) {
                    capacity_lease.release();
                }
                if let Some(response) = self.waiter_responses.remove(&waiter_id) {
                    let _ = response.send(terminal.result.to_wire());
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
            let Some((request_id, identity, ordinal, _, phase)) =
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
            let terminal = self.terminal_result(&request, Some(ordinal), disposition, detail)?;
            let terminal = self.settle_terminal_durable(&request_id, &identity, terminal)?;
            self.notify_terminal(terminal);
            self.clear_frame_proofs();
            self.drop_synchronous_preclaim_execution();
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
                    (scope.mapping_session_id == request.target().activation.request.session_id()
                        && scope.mapping_session_generation == request.target().activation.session_generation)
                        .then_some((scope.clone(), *key))
                })
                .collect();
            self.clear_frame_proofs();
            let Some(snapshot) = runtime
                .phase_a
                .activated_target_receipts()
                .current_sealed_assigned_target_snapshot()
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
                planar_work,
            } = admission;
            // Production gRPC admission supplies this guard.  It remains
            // alive through ledger activation and the admission response, so
            // no mapping completion can pass its final commit boundary while
            // the mode fence changes to active.
            let _mapping_lifecycle_guard = mapping_lifecycle_guard;
            let validated = if planar_work.is_some() {
                Err("RELOCALIZATION_ASSIGNED_TARGET_ABSENT_OR_UNSUPPORTED".into())
            } else { validate_relocalize_request(raw_request) };
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
                    let _ = response.send(Err(RelocalizationCommandError::InvalidRequest(detail)));
                    return Ok(());
                }
            };

            if !self.ledger.records.contains_key(request.request_id())
                && request.target().activation.runtime_generation != self.owner_identity.runtime_generation {
                let terminal = self.terminal_result(&request, None,
                    crate::proto::RelocalizeDisposition::StaleSession, "RELOCALIZATION_RUNTIME_STALE")?;
                let waiter = Self::immediate_waiter(waiter_id, terminal.to_wire(), Arc::clone(&capacity_lease), detached_tx);
                let _ = response.send(Ok(waiter));
                return Ok(());
            }
            if self.execution.is_some()
                && !self.ledger.records.contains_key(request.request_id())
            {
                let waiter = Self::immediate_waiter(
                    waiter_id,
                    self.busy_result(&request)?,
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
            )?;
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
                    result.to_wire(),
                    Arc::clone(&capacity_lease),
                    detached_tx,
                ),
                RelocalizationLedgerAdmission::ConflictOverflow(result) => {
                    close_after_reply = true;
                    Self::immediate_waiter(
                        waiter_id,
                        result.to_wire(),
                        Arc::clone(&capacity_lease),
                        detached_tx,
                    )
                }
                RelocalizationLedgerAdmission::Busy => Self::immediate_waiter(
                    waiter_id,
                    self.busy_result(&request)?,
                    Arc::clone(&capacity_lease),
                    detached_tx,
                ),
                RelocalizationLedgerAdmission::CapacityExceeded(detail) => {
                    if detail == "RELOCALIZATION_COMMAND_WAITER_CAPACITY_EXCEEDED" {
                        let admission_ordinal = self
                            .ledger
                            .records
                            .get(request.request_id())
                            .and_then(|record| match &record.origin {
                                RelocalizationLedgerOrigin::Admitted(metadata) => {
                                    Some(metadata.admission_ordinal)
                                }
                                RelocalizationLedgerOrigin::RestoredTerminal => None,
                            })
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
                        )?;
                        let pending = self.settle_terminal_durable(
                            request.request_id(),
                            &request.key(),
                            terminal,
                        )?;
                        let result = pending.result.to_wire();
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
                        )?;
                        self.terminal_store
                            .persist_terminal(
                                request.request_id(),
                                &request.key(),
                                &terminal,
                            )
                            .map_err(CorrectionRuntimeError::Apply)?;
                        self.ledger
                            .store_capacity_terminal(&request, Instant::now(), terminal.clone())
                            .map_err(|detail| CorrectionRuntimeError::Apply(detail.to_owned()))?;
                        close_after_reply = true;
                        Self::immediate_waiter(
                            waiter_id,
                            terminal.to_wire(),
                            Arc::clone(&capacity_lease),
                            detached_tx,
                        )
                    }
                }
            };
            if let Some((ordinal, epoch, deadline)) = started {
                if let Some(runtime) = runtime {
                    self.begin_runtime_execution(request, ordinal, epoch, deadline, runtime)?;
                } else {
                    self.settle_active(crate::proto::RelocalizeDisposition::TargetUnavailable,
                        "RELOCALIZATION_TARGET_UNAVAILABLE")?;
                }
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
            &mut self, input: &Arc<CorrectionFrameInput>, key: RelocalizationFrameKey,
        ) -> Result<bool, &'static str> {
            let scope = RelocalizationFrameScope::from_input(input);
            let proof_key = (scope.clone(), key);
            if let Some(existing) = self.validated_frame_proofs.get(&proof_key) {
                // Released/unaccepted entries are tombstones: never claim equal
                // source bytes after releasing their owner, and never retain a
                // rejected cloud solely for duplicate comparison.
                return match existing.upgrade() {
                    Some(existing) if existing.exactly_matches(input) => Ok(true),
                    Some(_) => Err("RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT"),
                    None => Err("RELOCALIZATION_FRAME_KEY_PROOF_RELEASED"),
                };
            }
            let scope_is_new = !self.last_validated_frames.contains_key(&scope);
            if scope_is_new && self.last_validated_frames.len() >= RELOCALIZATION_MAX_FRAME_PROOF_SCOPES {
                return Err("RELOCALIZATION_FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED");
            }
            if self.validated_frame_proofs.len() >= RELOCALIZATION_MAX_FRAME_PROOFS_TOTAL {
                return Err("RELOCALIZATION_FRAME_PROOF_TOTAL_CAPACITY_EXCEEDED");
            }
            if self.validated_frame_proofs.keys().filter(|(existing_scope, _)| existing_scope == &scope).count()
                >= RELOCALIZATION_MAX_FRAME_PROOFS_PER_SCOPE {
                return Err("RELOCALIZATION_FRAME_PROOF_SCOPE_CAPACITY_EXCEEDED");
            }
            self.validated_frame_proofs.insert(proof_key, Arc::downgrade(input));
            let last = self.last_validated_frames.entry(scope).or_insert(key);
            if key > *last { *last = key; }
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
            let key = match relocalization_frame_evidence(
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
            let admitted_input = Arc::new(request.input.clone());
            match self.record_frame_proof(&admitted_input, key) {
                Ok(true) => {
                    Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                    return Ok(());
                }
                Ok(false) => {}
                Err("RELOCALIZATION_FRAME_KEY_PROOF_RELEASED") => {
                    // The old source was never accepted or has been released.
                    // This tombstone cannot establish content equality or a
                    // conflict. Drop the stale retry and allow fresh progress.
                    Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                    return Ok(());
                }
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
                .current_sealed_assigned_target_snapshot();
            if !current_snapshot
                .as_ref()
                .is_some_and(|current| same_sealed_target(current, &accumulation.snapshot))
            {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
                self.settle_active(
                    crate::proto::RelocalizeDisposition::TargetInvalidated,
                    "RELOCALIZATION_TARGET_INVALIDATED_DURING_ACCUMULATION",
                )?;
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            }
            if request.input.mapping_session_id != accumulation.request.target().activation.request.session_id()
                || request.input.mapping_session_generation != accumulation.request.target().activation.session_generation
                || request.input.committed_map.backend_kind != accumulation.request.target().activation.backend_kind
                || request.input.committed_map.backend_generation != accumulation.request.target().activation.backend_generation
            {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
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
                || accumulation.mapping_commit_boot_nonce.is_some_and(|nonce| nonce != request.input.mapping_commit_id.boot_nonce)
                || accumulation.accepted.first().is_some_and(|frame|
                    frame.input.registered_cloud.backend != request.input.registered_cloud.backend
                        || frame.input.registered_cloud.runtime_generation != request.input.registered_cloud.runtime_generation)
            {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
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
                    .get(&RelocalizationFrameScope::from_input(&request.input))
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

            if accumulation.accepted.len() >= RELOCALIZATION_MAX_FRAMES {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
                self.settle_active(crate::proto::RelocalizeDisposition::InsufficientSource,
                    "RELOCALIZATION_FRAME_COUNT_BOUND_EXCEEDED")?;
                Self::reject_frame(request, "RELOCALIZATION_FRAME_COUNT_BOUND_EXCEEDED");
                return Ok(());
            }
            let frame_points = request.input.body_points.len();
            let Some(aggregate_input_points) = accumulation
                .aggregate_input_points
                .checked_add(frame_points)
            else {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
                self.settle_active(
                    crate::proto::RelocalizeDisposition::OwnerFailed,
                    "RELOCALIZATION_AGGREGATE_POINT_COUNT_OVERFLOW",
                )?;
                Self::reject_frame(request, "RELOCALIZATION_AGGREGATE_POINT_COUNT_OVERFLOW");
                return Ok(());
            };
            if aggregate_input_points > RELOCALIZATION_MAX_AGGREGATE_POINTS {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
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
            accumulation.accepted.push(AccumulatedRelocalizationFrame { input: admitted_input });

            let cutoff_reason = request.lease.has_lifecycle_guard()
                .then_some(RelocalizationCutoffReasonV1::MinimumAggregateReached);
            let Some(cutoff_reason) = cutoff_reason else {
                self.execution = Some(RelocalizationExecution::Accumulating(accumulation));
                Self::reply_frame(request.response, CorrectionSubmitDisposition::NoCandidate);
                return Ok(());
            };

            let request_metadata = accumulation.request.clone();
            let request_id = request_metadata.request_id().to_owned();
            let identity = request_metadata.key();
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
            let source_receipts = source.frames.iter().map(|frame| assigned_source_receipt(&frame.input,
                &self.map_frame_id, &self.odom_frame_id, &self.body_frame_id)).collect::<Result<Vec<_>, _>>()
                .map_err(CorrectionRuntimeError::Apply)?;
            self.execution = Some(RelocalizationExecution::Computing {
                request: Box::new(request_metadata), admission_ordinal, logical_deadline, source_receipts,
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
                identity,
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
            &self, computed: &RelocalizationComputedCommand,
            disposition: crate::proto::RelocalizeDisposition, candidate: Option<&CorrectionCandidate>,
            correction_revision: u64, detail: &str,
        ) -> Result<StoredRelocalizationResult, CorrectionRuntimeError> {
            let mut evidence = self.assigned_evidence(&computed.request, Some(computed.admission_ordinal),
                Some(&computed.source), Some(&computed.output))?;
            if candidate.is_some() { evidence.phase = crate::proto::AssignedRelocalizationEvidencePhaseV2::Finalized as i32; }
            let candidate_payload = candidate.map(CorrectionCandidate::encode_body).transpose()
                .map_err(|error| CorrectionRuntimeError::Apply(error.to_string()))?.unwrap_or_default();
            let result = assigned_relocalization_result(&computed.request.identity, disposition,
                correction_revision, detail, candidate_payload, evidence);
            let capture = |request: &AssignedCloudRelocalizeRequestIdentityV2, body: &crate::proto::AssignedCloudRelocalizeResultV2| {
                validate_assigned_terminal_retained(request, body)?;
                // Compare every retained receipt with the still-owned source;
                // the closure captures the actual candidate and finalizer outcome.
                let expected = result.assigned_cloud_v2.as_ref().expect("assigned body");
                if request != &computed.request.identity || body != expected {
                    return Err("RELOCALIZATION_ASSIGNED_CAPTURE_OUTCOME_MISMATCH".into());
                }
                if let Some(candidate) = candidate {
                    let expected_identity = correction_candidate_identity(CorrectionCandidateBackend::Relocalization,
                        &self.owner_identity, candidate.identity.candidate_sequence, &self.map_frame_id,
                        &self.odom_frame_id, &self.body_frame_id, &computed.source.boundary_input);
                    let expected_correction = computed.output.correction.as_ref()
                        .ok_or("RELOCALIZATION_ASSIGNED_CAPTURE_CORRECTION_ABSENT")?;
                    let expected_candidate = CorrectionCandidate::seal(expected_identity,
                        CorrectionTransform::from_isometry(expected_correction)).map_err(|error| error.to_string())?;
                    if candidate != &expected_candidate {
                        return Err("RELOCALIZATION_ASSIGNED_CAPTURE_CANDIDATE_MISMATCH".into());
                    }
                }
                Ok(())
            };
            // Validate before moving the original terminal owner into storage.
            let terminal = AssignedRelocalizationTerminalV2::new(result.clone(), Some(&capture))
                .map_err(CorrectionRuntimeError::Apply)?;
            Ok(StoredRelocalizationResult::Assigned(terminal))
        }

        fn assigned_evidence(
            &self, request: &ValidatedRelocalizeRequest, admission_ordinal: Option<u64>,
            source: Option<&SealedRelocalizationSource>, output: Option<&RelocalizationComputeOutput>,
        ) -> Result<crate::proto::AssignedRelocalizationEvidenceV2, CorrectionRuntimeError> {
            use crate::proto::AssignedRelocalizationEvidencePhaseV2 as Phase;
            let mut evidence = crate::proto::AssignedRelocalizationEvidenceV2 {
                schema_version: 2, phase: Phase::PreAdmission as i32, ..Default::default()
            };
            if let Some(ordinal) = admission_ordinal {
                if ordinal == 0 { return Err(CorrectionRuntimeError::Apply("relocalization admission ordinal is zero".into())); }
                evidence.phase = Phase::Admitted as i32;
                evidence.admission_ordinal = Some(ordinal);
                evidence.owner = Some(crate::proto::RelocalizationOwnerReceiptV2 {
                    runtime_generation: self.owner_identity.runtime_generation,
                    correction_backend_generation: self.owner_identity.backend_generation,
                    correction_lease_boot_nonce: self.owner_identity.correction_lease.boot_nonce.to_be_bytes().to_vec(),
                    correction_lease_generation: self.owner_identity.correction_lease.generation,
                    map_frame_id: self.map_frame_id.clone(), odom_frame_id: self.odom_frame_id.clone(),
                    body_frame_id: self.body_frame_id.clone(),
                    relocalization_epoch: source.map(|s| s.boundary_input.frame_capture_relocalization_epoch)
                        .or_else(|| { let mode = self.ledger.relocalization_mode.snapshot(); mode.active.then_some(mode.epoch) }),
                });
            }
            if let Some(source) = source {
                if !request.identity.matches_target(&source.snapshot.target_receipt) {
                    return Err(CorrectionRuntimeError::Apply("RELOCALIZATION_ASSIGNED_CAPTURE_TARGET_MISMATCH".into()));
                }
                evidence.phase = Phase::SourceSelected as i32;
                for frame in &source.frames {
                    frame.input.validate_source_ownership().map_err(|error| CorrectionRuntimeError::Apply(error.to_string()))?;
                    evidence.source_frames.push(assigned_source_receipt(&frame.input, &self.map_frame_id,
                        &self.odom_frame_id, &self.body_frame_id).map_err(CorrectionRuntimeError::Apply)?);
                }
            }
            if let Some(output) = output {
                evidence.phase = Phase::AlgorithmCompleted as i32;
                evidence.algorithm = Some(crate::proto::RelocalizationAlgorithmReceiptV2 {
                    converged: output.result.registration.converged, accepted: output.correction.is_some(),
                    mse_bits: output.result.registration.mse.to_bits(), fitness_bits: output.result.fitness.to_bits(),
                    iterations: output.result.registration.iterations.try_into().map_err(|_| CorrectionRuntimeError::Apply("relocalization iterations overflow".into()))?,
                    feature_matches: output.result.feature_matches.try_into().map_err(|_| CorrectionRuntimeError::Apply("relocalization feature count overflow".into()))?,
                    ransac_inliers: output.result.ransac_inliers.try_into().map_err(|_| CorrectionRuntimeError::Apply("relocalization inlier count overflow".into()))?,
                    estimated_transform: Some(relocalization_pose_bits(&output.result.registration.transformation)),
                    proposed_correction: output.correction.as_ref().map(relocalization_pose_bits),
                });
            }
            Ok(evidence)
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
                computed.request.request_id(),
                &computed.request.key(),
                terminal,
            )?;
            self.notify_terminal(terminal);
            self.clear_frame_proofs();
            Ok(())
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

        fn handle_task_event(
            &mut self,
            event: RelocalizationTaskEvent,
            runtime: &RelocalizationRuntimeContext,
            tasks: &mut tokio::task::JoinSet<RelocalizationTaskEvent>,
        ) -> Result<(), CorrectionRuntimeError> {
            match event {
                RelocalizationTaskEvent::Computed {
                    request_id,
                    identity,
                    source,
                    output,
                } => {
                    let Some(RelocalizationExecution::Computing {
                        request,
                        admission_ordinal,
                        logical_deadline,
                        ..
                    }) = self.execution.take()
                    else {
                        return Ok(());
                    };
                    if request.request_id() != request_id
                        || request.key() != identity
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
                        .terminal(computed.request.request_id())
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
                        .terminal(computed.request.request_id())
                        .is_some()
                    {
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
                        .terminal(computed.request.request_id())
                        .is_some()
                    {
                        Self::schedule_staged_discard(&computed, runtime, tasks);
                        return Ok(());
                    }

                    let claim_fence = runtime.phase_a.command_claim_fence();
                    let _claim_guard = claim_fence.lock();
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
                    let key = CandidateApplyKey::from(&candidate);
                    let apply_rx = match runtime
                        .apply_waiters
                        .register(key, candidate.clone(), Arc::clone(&computed.source.boundary_lease))
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
                    let Some(lease) = runtime.apply_waiters.claim_apply(key, &candidate) else {
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
                        computed.request.request_id(),
                        &computed.request.key(),
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
            let Some((request_id, identity, capacity_lease)) =
                self.waiter_requests.remove(&waiter_id)
            else {
                return Ok(());
            };
            capacity_lease.release();
            let cancel = self
                .ledger
                .detach_waiter(&request_id, &identity, waiter_id)
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
        actor: &mut RelocalizationCommandActor, admissions: &mut mpsc::Receiver<RelocalizationAdmissionRequest>,
        terminal_queries: &mut mpsc::Receiver<RelocalizationTerminalQueryRequest>,
        detached: &mut mpsc::UnboundedReceiver<RelocalizationWaiterEvent>,
        detached_tx: &mpsc::UnboundedSender<RelocalizationWaiterEvent>, phase_a: &CorrectionPhaseAControl,
    ) -> Result<(), CorrectionRuntimeError> {
        let mut admission_rx = phase_a.admission_tx.subscribe();
        loop {
            tokio::select! { biased;
                _ = wait_for_admission_close(&mut admission_rx) => {
                    admissions.close();
                    actor.settle_active(crate::proto::RelocalizeDisposition::OwnerStopped,
                        "RELOCALIZATION_OWNER_STOPPED_PRECLAIM")?;
                    return Ok(());
                },
                query = terminal_queries.recv() => match query {
                    Some(query) => actor.handle_terminal_query(query), None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                event = detached.recv() => match event {
                    Some(RelocalizationWaiterEvent::Detached(id)) => actor.handle_detached(id)?,
                    None => return Err(CorrectionRuntimeError::IngressClosed),
                },
                admission = admissions.recv() => match admission {
                    Some(admission) => actor.handle_admission(admission, detached_tx, None, phase_a)?,
                    None => return Ok(()),
                },
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
            // 采集等待使用原请求期限；已交给功能层的计算不被提交层再次超时否决。
            let logical_deadline = match actor.execution.as_ref() {
                Some(RelocalizationExecution::Accumulating(_)) => actor.ledger.active_identity().map(|active| active.3),
                _ => None,
            };
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
                            .current_sealed_assigned_target_snapshot()
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

    fn snapshot_matches_relocalize_request(
        snapshot: &SealedAssignedActivatedTargetSnapshot, request: &ValidatedRelocalizeRequest,
        map_frame_id: &str,
    ) -> Result<(), (crate::proto::RelocalizeDisposition, &'static str)> {
        use crate::proto::RelocalizeDisposition as Disposition;
        let requested = &request.target().activation;
        let installed = &snapshot.activation_identity;
        if requested.runtime_generation != installed.runtime_generation {
            return Err((Disposition::StaleSession, "RELOCALIZATION_RUNTIME_STALE"));
        }
        if requested.session_generation < installed.session_generation {
            return Err((Disposition::StaleSession, "RELOCALIZATION_SESSION_STALE"));
        }
        if requested.request.session_id() != installed.request.session_id()
            || requested.request.map_name() != installed.request.map_name()
            || requested.session_generation != installed.session_generation {
            return Err((Disposition::SessionInvalidated, "RELOCALIZATION_SESSION_IDENTITY_MISMATCH"));
        }
        if !request.identity.matches_target(&snapshot.target_receipt)
            || !snapshot.target_receipt.validate(&snapshot.checkpoint_owner)
            || snapshot.checkpoint_owner.cloud().member().frame_id() != map_frame_id
            || snapshot.publication_proof.target_receipt != TargetReceipt::Assigned(snapshot.target_receipt.clone())
            || snapshot.publication_proof.cloud_receipt != snapshot.target_receipt.cloud_receipt {
            return Err((Disposition::TargetInvalidated, "RELOCALIZATION_TARGET_IDENTITY_MISMATCH"));
        }
        Ok(())
    }

    fn same_sealed_target(left: &SealedAssignedActivatedTargetSnapshot, right: &SealedAssignedActivatedTargetSnapshot) -> bool {
        left.activation_identity == right.activation_identity && left.target_receipt == right.target_receipt
            && left.publication_proof.same_descriptors(&right.publication_proof)
            && left.checkpoint_owner.shares_source_with(&right.checkpoint_owner)
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
        mapping_commit_boot_nonce: u128,
        mapping_backend_kind: &'static str,
        mapping_backend_generation: u64,
        lio_backend: i32,
        lio_runtime_generation: u64,
    }

    impl RelocalizationFrameScope {
        fn from_input(input: &CorrectionFrameInput) -> Self {
            Self {
                mapping_session_id: input.mapping_session_id.clone(),
                mapping_session_generation: input.mapping_session_generation,
                map_frame_generation: input.map_frame_generation,
                mapping_commit_boot_nonce: input.mapping_commit_id.boot_nonce,
                mapping_backend_kind: input.committed_map.backend_kind.stable_name(),
                mapping_backend_generation: input.committed_map.backend_generation,
                lio_backend: input.registered_cloud.backend,
                lio_runtime_generation: input.registered_cloud.runtime_generation,
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

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum RelocalizationCutoffReasonV1 { MinimumAggregateReached }

    fn relocalization_pose_bits(pose: &Isometry3<f64>) -> crate::proto::RelocalizationPoseBitsV2 {
        let pose = CorrectionTransform::from_isometry(pose);
        crate::proto::RelocalizationPoseBitsV2 {
            translation_m_bits: pose.translation_m.map(f64::to_bits).to_vec(),
            rotation_xyzw_bits: pose.rotation_xyzw.map(f64::to_bits).to_vec(),
        }
    }

    fn assigned_source_receipt(input: &CorrectionFrameInput, map: &str, odom: &str, body: &str)
        -> Result<crate::proto::RelocalizationSourceReceiptV2, String>
    {
        input.validate_source_ownership().map_err(|error| error.to_string())?;
        let binding = input.source_owner.binding();
        let occupancy = crate::mapping_commit::OccupancyCommitReceipt {
            commit_id: input.mapping_commit_id, session_id: input.mapping_session_id.clone(),
            session_generation: input.mapping_session_generation, occupancy_revision: input.occupancy_revision,
            source_timestamp_ns: input.committed_map.source_timestamp_ns,
            backend_kind: input.committed_map.backend_kind, backend_generation: input.committed_map.backend_generation,
        };
        Ok(crate::proto::RelocalizationSourceReceiptV2 {
            occupancy_receipt: Some(crate::activation_contract::occupancy_to_wire(&occupancy)),
            cloud_receipt: Some(crate::activation_contract::cloud_to_wire(&input.cloud_receipt)),
            cloud_revision: input.cloud_revision, map_frame_generation: input.map_frame_generation,
            lio_backend: input.registered_cloud.backend, runtime_generation: input.registered_cloud.runtime_generation,
            source_sequence: input.registered_cloud.source_sequence, source_timestamp_ns: input.registered_cloud.source_timestamp_ns,
            admitted_point_count: input.registered_cloud.point_count,
            correlated_odom_source_sequence: input.correlated_odom.proof.source_sequence,
            correlated_odom_source_timestamp_ns: input.correlated_odom.proof.source_timestamp_ns,
            odom_from_body: Some(relocalization_pose_bits(&input.correlated_odom.odom_from_body)),
            map_geometry: Some(crate::proto::RelocalizationMapGeometryV2 {
                width: input.committed_map.width, height: input.committed_map.height,
                resolution_m_bits: input.committed_map.resolution_m.to_bits(),
                origin_x_m_bits: input.committed_map.origin_x_m.to_bits(), origin_y_m_bits: input.committed_map.origin_y_m.to_bits(),
            }),
            base_anchor_map_from_odom: Some(relocalization_pose_bits(&input.base_anchor_map_from_odom)),
            registration_map_from_odom: Some(relocalization_pose_bits(&binding.registration_map_from_odom)),
            map_frame_id: map.into(), odom_frame_id: odom.into(), body_frame_id: body.into(), points_frame_id: input.points_frame_id.clone(),
            registration_correction_revision: binding.registration_correction_revision,
            frame_capture_relocalization_epoch: input.frame_capture_relocalization_epoch,
            relocalization_mode_epoch: input.relocalization_mode_epoch,
        })
    }

    fn assigned_relocalization_result(
        request: &AssignedCloudRelocalizeRequestIdentityV2, disposition: crate::proto::RelocalizeDisposition,
        correction_revision: u64, detail: &str, candidate_payload_v3: Vec<u8>, evidence: crate::proto::AssignedRelocalizationEvidenceV2,
    ) -> crate::proto::RelocalizeResult {
        crate::proto::RelocalizeResult {
            schema_version: 2, request_id: request.request_id().to_owned(),
            assigned_cloud_v2: Some(crate::proto::AssignedCloudRelocalizeResultV2 {
                request: Some(request.to_wire()), disposition: disposition as i32,
                correction_revision, detail: detail.to_owned(), candidate_payload_v3, evidence: Some(evidence),
            }), ..Default::default()
        }
    }

    fn assigned_pose_transform(pose: Option<&crate::proto::RelocalizationPoseBitsV2>) -> Result<CorrectionTransform, String> {
        let pose = pose.ok_or("RELOCALIZATION_ASSIGNED_POSE_ABSENT")?;
        let translation_m: [u64; 3] = pose.translation_m_bits.as_slice().try_into().map_err(|_| "RELOCALIZATION_ASSIGNED_POSE_INVALID")?;
        let rotation_xyzw: [u64; 4] = pose.rotation_xyzw_bits.as_slice().try_into().map_err(|_| "RELOCALIZATION_ASSIGNED_POSE_INVALID")?;
        let transform = CorrectionTransform { translation_m: translation_m.map(f64::from_bits), rotation_xyzw: rotation_xyzw.map(f64::from_bits) };
        correction_transform_isometry(transform)?;
        Ok(transform)
    }

    fn assigned_pose_isometry(pose: Option<&crate::proto::RelocalizationPoseBitsV2>) -> Result<Isometry3<f64>, String> {
        let transform = assigned_pose_transform(pose)?;
        let [x, y, z, w] = transform.rotation_xyzw;
        let [tx, ty, tz] = transform.translation_m;
        // Norm and finiteness were checked above. Preserve the admitted bits;
        // a second normalization would change the retained algorithm operands.
        Ok(Isometry3::from_parts(Translation3::new(tx, ty, tz),
            UnitQuaternion::new_unchecked(Quaternion::new(w, x, y, z))))
    }

    /// Durable validation checks retained exact bindings and canonical V3 body.
    /// It does not claim that released source bytes or external side effects can
    /// be independently attested after reopening. Capture additionally compares
    /// these receipts with the real admitted owners and actual finalizer outcome.
    fn validate_assigned_terminal_retained(
        request: &AssignedCloudRelocalizeRequestIdentityV2, result: &crate::proto::AssignedCloudRelocalizeResultV2,
    ) -> Result<(), String> {
        use crate::proto::{AssignedRelocalizationEvidencePhaseV2 as Phase, RelocalizeDisposition as Disposition};
        let evidence = result.evidence.as_ref().ok_or("RELOCALIZATION_ASSIGNED_EVIDENCE_ABSENT")?;
        let phase = Phase::try_from(evidence.phase).map_err(|_| "RELOCALIZATION_ASSIGNED_PHASE_INVALID")?;
        let disposition = Disposition::try_from(result.disposition).map_err(|_| "RELOCALIZATION_ASSIGNED_DISPOSITION_INVALID")?;
        if phase == Phase::PreAdmission {
            if evidence.owner.is_some() || evidence.admission_ordinal.is_some() || !evidence.source_frames.is_empty()
                || evidence.algorithm.is_some() || !result.candidate_payload_v3.is_empty() || result.correction_revision != 0
                || matches!(disposition, Disposition::Applied | Disposition::AlreadyApplied | Disposition::AlgorithmRejected) {
                return Err("RELOCALIZATION_ASSIGNED_PRE_ADMISSION_CLAIMS_INVALID".into());
            }
            return Ok(());
        }
        let owner = evidence.owner.as_ref().ok_or("RELOCALIZATION_ASSIGNED_OWNER_ABSENT")?;
        if owner.relocalization_epoch.is_none_or(|epoch| epoch == 0 || epoch % 2 == 0) {
            return Err("RELOCALIZATION_ASSIGNED_OWNER_EPOCH_INVALID".into());
        }
        let mut source_scope = None;
        for frame in &evidence.source_frames {
            let occupancy = crate::activation_identity::occupancy_receipt_from_wire(frame.occupancy_receipt.as_ref().ok_or("RELOCALIZATION_ASSIGNED_OCCUPANCY_ABSENT")?)?;
            let cloud = crate::activation_identity::cloud_receipt_from_wire(frame.cloud_receipt.as_ref().ok_or("RELOCALIZATION_ASSIGNED_CLOUD_ABSENT")?)?;
            if cloud.commit_id.boot_nonce != occupancy.commit_id.boot_nonce
                || cloud.commit_id.sequence > occupancy.commit_id.sequence
                || cloud.occupancy_revision > occupancy.occupancy_revision
                || cloud.source_timestamp_ns > occupancy.source_timestamp_ns
                || occupancy.source_timestamp_ns > frame.source_timestamp_ns
                || frame.correlated_odom_source_timestamp_ns > frame.source_timestamp_ns
                || frame.relocalization_mode_epoch != owner.relocalization_epoch
                || frame.frame_capture_relocalization_epoch != owner.relocalization_epoch.unwrap_or(0) {
                return Err("RELOCALIZATION_ASSIGNED_SOURCE_ORDER_OR_EPOCH_MISMATCH".into());
            }
            let scope = (occupancy.commit_id.boot_nonce, frame.map_frame_generation, frame.lio_backend, frame.runtime_generation);
            if source_scope.is_some_and(|expected| expected != scope) { return Err("RELOCALIZATION_ASSIGNED_SOURCE_SCOPE_MISMATCH".into()); }
            source_scope = Some(scope);
            assigned_pose_transform(frame.odom_from_body.as_ref())?;
            assigned_pose_transform(frame.base_anchor_map_from_odom.as_ref())?;
            assigned_pose_transform(frame.registration_map_from_odom.as_ref())?;
        }
        if let Some(algorithm) = &evidence.algorithm {
            assigned_pose_transform(algorithm.estimated_transform.as_ref())?;
            if algorithm.proposed_correction.is_some() { assigned_pose_transform(algorithm.proposed_correction.as_ref())?; }
            // Accepted compute can settle before a candidate is claimed (for
            // example cancellation, reserve failure, or staging failure). Its
            // algorithm receipt still binds the exact frozen source operands.
            if algorithm.accepted {
                let boundary = evidence.source_frames.last().ok_or("RELOCALIZATION_ASSIGNED_ALGORITHM_BOUNDARY_ABSENT")?;
                let transform = assigned_pose_transform(algorithm.proposed_correction.as_ref())?;
                let base = assigned_pose_isometry(boundary.base_anchor_map_from_odom.as_ref())?;
                let estimate = assigned_pose_isometry(algorithm.estimated_transform.as_ref())?;
                let derived = CorrectionTransform::from_isometry(&(base.inverse() * estimate));
                if transform.translation_m.iter().zip(derived.translation_m).any(|(a,b)| a.to_bits() != b.to_bits())
                    || transform.rotation_xyzw.iter().zip(derived.rotation_xyzw).any(|(a,b)| a.to_bits() != b.to_bits())
                    || request.target().cloud_receipt.point_count < RELOCALIZATION_MIN_SOURCE_POINTS as u32 {
                    return Err("RELOCALIZATION_ASSIGNED_ALGORITHM_BINDING_MISMATCH".into());
                }
            }
        }
        if result.candidate_payload_v3.is_empty() {
            if phase == Phase::Finalized || result.correction_revision != 0
                || matches!(disposition, Disposition::Applied | Disposition::AlreadyApplied) {
                return Err("RELOCALIZATION_ASSIGNED_OUTCOME_WITHOUT_CANDIDATE".into());
            }
            return Ok(());
        }
        if phase != Phase::Finalized || !matches!(disposition, Disposition::Applied | Disposition::AlreadyApplied | Disposition::Rejected) {
            return Err("RELOCALIZATION_ASSIGNED_CANDIDATE_OUTCOME_INVALID".into());
        }
        let candidate = CorrectionCandidate::decode_body(&result.candidate_payload_v3).map_err(|error| error.to_string())?;
        if candidate.encode_body().map_err(|error| error.to_string())? != result.candidate_payload_v3 {
            return Err("RELOCALIZATION_ASSIGNED_CANDIDATE_NON_CANONICAL".into());
        }
        let boundary = evidence.source_frames.last().ok_or("RELOCALIZATION_ASSIGNED_CANDIDATE_BOUNDARY_ABSENT")?;
        let occupancy = crate::activation_identity::occupancy_receipt_from_wire(boundary.occupancy_receipt.as_ref().ok_or("RELOCALIZATION_ASSIGNED_OCCUPANCY_ABSENT")?)?;
        let geometry = boundary.map_geometry.as_ref().ok_or("RELOCALIZATION_ASSIGNED_GEOMETRY_ABSENT")?;
        let odom = assigned_pose_transform(boundary.odom_from_body.as_ref())?;
        let lease_nonce: [u8; 16] = owner.correction_lease_boot_nonce.as_slice().try_into().map_err(|_| "RELOCALIZATION_ASSIGNED_LEASE_INVALID")?;
        let identity = CorrectionCandidateIdentity {
            backend: CorrectionCandidateBackend::Relocalization,
            transform_semantics: CorrectionTransformSemantics::AbsoluteLeftCorrectionSe3,
            runtime_generation: owner.runtime_generation, backend_generation: owner.correction_backend_generation,
            candidate_sequence: candidate.identity.candidate_sequence,
            mapping_session_id: occupancy.session_id, mapping_session_generation: occupancy.session_generation,
            map_frame_generation: boundary.map_frame_generation, mapping_commit_id: occupancy.commit_id,
            occupancy_revision: occupancy.occupancy_revision, cloud_revision: boundary.cloud_revision,
            registered_cloud: RegisteredCloudProof { backend: boundary.lio_backend, runtime_generation: boundary.runtime_generation,
                source_sequence: boundary.source_sequence, source_timestamp_ns: boundary.source_timestamp_ns, point_count: boundary.admitted_point_count },
            correlated_odom: CorrelatedOdomProof { source_sequence: boundary.correlated_odom_source_sequence,
                source_timestamp_ns: boundary.correlated_odom_source_timestamp_ns, translation_m: odom.translation_m, rotation_xyzw: odom.rotation_xyzw },
            committed_map: CommittedMapProof { backend_kind: occupancy.backend_kind, backend_generation: occupancy.backend_generation,
                source_timestamp_ns: occupancy.source_timestamp_ns, width: geometry.width, height: geometry.height,
                resolution_m: f64::from_bits(geometry.resolution_m_bits), origin_x_m: f64::from_bits(geometry.origin_x_m_bits),
                origin_y_m: f64::from_bits(geometry.origin_y_m_bits) },
            source_timestamp_ns: boundary.source_timestamp_ns, map_frame_id: owner.map_frame_id.clone(),
            odom_frame_id: owner.odom_frame_id.clone(), body_frame_id: owner.body_frame_id.clone(), points_frame_id: boundary.points_frame_id.clone(),
            correction_lease: CorrectionLeaseId { boot_nonce: u128::from_be_bytes(lease_nonce), generation: owner.correction_lease_generation },
        };
        let algorithm = evidence.algorithm.as_ref().ok_or("RELOCALIZATION_ASSIGNED_ALGORITHM_ABSENT")?;
        let transform = assigned_pose_transform(algorithm.proposed_correction.as_ref())?;
        let expected = CorrectionCandidate::seal(identity, transform).map_err(|error| error.to_string())?;
        if candidate != expected || candidate.identity.mapping_session_id != request.target().activation.request.session_id() {
            return Err("RELOCALIZATION_ASSIGNED_CANDIDATE_BINDING_MISMATCH".into());
        }
        Ok(())
    }

    fn relocalization_frame_evidence(
        owner: &CorrectionOwnerRuntimeIdentity,
        map_frame_id: &str,
        odom_frame_id: &str,
        body_frame_id: &str,
        input: &CorrectionFrameInput,
    ) -> Result<RelocalizationFrameKey, String> {
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
        input.validate_source_ownership().map_err(|error| error.to_string())?;
        let receipt = assigned_source_receipt(input, map_frame_id, odom_frame_id, body_frame_id)?;
        if receipt.encoded_len() > RELOCALIZATION_MAX_PROOF_BYTES {
            return Err("RELOCALIZATION_FRAME_PROOF_SIZE_EXCEEDED".into());
        }
        Ok(relocalization_frame_key(input))
    }

    struct AccumulatedRelocalizationFrame {
        input: Arc<CorrectionFrameInput>,
    }

    struct SealedRelocalizationSource {
        boundary_input: CorrectionFrameInput,
        boundary_lease: Arc<CorrectionRequestLease>,
        snapshot: SealedAssignedActivatedTargetSnapshot,
        frames: Vec<AccumulatedRelocalizationFrame>,
        source_points: Arc<[Point3<f64>]>,
        target_points: Arc<[Point3<f64>]>,
    }

    fn seal_relocalization_source(
        mut accumulation: RelocalizationAccumulation,
        boundary_input: CorrectionFrameInput,
        boundary_lease: Arc<CorrectionRequestLease>,
        _cutoff_reason: RelocalizationCutoffReasonV1,
    ) -> Result<SealedRelocalizationSource, String> {
        if !boundary_lease.has_lifecycle_guard() {
            return Err("RELOCALIZATION_BOUNDARY_LIFECYCLE_GUARD_MISSING".to_owned());
        }
        accumulation
            .accepted
            .sort_by_key(|frame| relocalization_frame_key(&frame.input));
        boundary_input.validate_source_ownership().map_err(|error| error.to_string())?;
        if accumulation.accepted.last().is_none_or(|frame| !frame.input.exactly_matches(&boundary_input)) {
            return Err("RELOCALIZATION_BOUNDARY_OWNER_MISMATCH".into());
        }
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
        let source_points = stable_stratified_source(ordered)?;
        let target_points = decode_relocalization_target(accumulation.snapshot.checkpoint_owner.cloud())?;
        Ok(SealedRelocalizationSource {
            boundary_input, boundary_lease, snapshot: accumulation.snapshot,
            frames: accumulation.accepted, source_points: Arc::from(source_points), target_points: Arc::from(target_points),
        })
    }

    fn decode_relocalization_target(target: &crate::activation_identity::CanonicalCloudTargetV2)
        -> Result<Vec<Point3<f64>>, String>
    {
        let count = usize::try_from(target.point_count()).map_err(|_| "RELOCALIZATION_TARGET_POINT_COUNT_OVERFLOW")?;
        if count < RELOCALIZATION_MIN_SOURCE_POINTS || count.checked_mul(12) != Some(target.xyz_f32_le().len()) {
            return Err("RELOCALIZATION_TARGET_BYTES_INVALID".into());
        }
        let mut points = Vec::with_capacity(count);
        for bytes in target.xyz_f32_le().chunks_exact(12) {
            let point = Point3::new(f64::from(f32::from_le_bytes(bytes[0..4].try_into().unwrap())),
                f64::from(f32::from_le_bytes(bytes[4..8].try_into().unwrap())),
                f64::from(f32::from_le_bytes(bytes[8..12].try_into().unwrap())));
            if point.iter().any(|coordinate| !coordinate.is_finite()) { return Err("RELOCALIZATION_TARGET_POINT_NONFINITE".into()); }
            points.push(point);
        }
        Ok(points)
    }

    fn stable_stratified_source(ordered: Vec<Point3<f64>>) -> Result<Vec<Point3<f64>>, String> {
        if ordered.len() > RELOCALIZATION_MAX_AGGREGATE_POINTS { return Err("RELOCALIZATION_SOURCE_POINT_COUNT_INVALID".into()); }
        let emitted_count = ordered.len().min(RELOCALIZATION_MAX_EMITTED_POINTS);
        let emitted = if ordered.len() <= emitted_count { ordered } else {
            let count = ordered.len() as u128;
            (0..emitted_count).map(|index| {
                let source_index = (index as u128).checked_mul(count)
                    .ok_or("RELOCALIZATION_REDUCTION_INDEX_OVERFLOW")? / emitted_count as u128;
                Ok(ordered[source_index as usize])
            }).collect::<Result<Vec<_>, String>>()?
        };
        let bytes = emitted.len().checked_mul(24).ok_or("RELOCALIZATION_SOURCE_BYTES_OVERFLOW")?;
        if bytes > RELOCALIZATION_MAX_PER_FRAME_F64_BYTES { return Err("RELOCALIZATION_EMITTED_SOURCE_BYTES_INVALID".into()); }
        Ok(emitted)
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
        let config = RelocalizationConfig::default();
        // 当前位姿已有初值时先完成局部对齐，避免每次必须从全图重新找位置。
        let seeded: Vec<_> = source.iter().map(|point| base_anchor_map_from_odom * point).collect();
        let local = IcpSolver::new(IcpConfig {
            max_correspondence_distance: config.voxel_sizes.into_iter().fold(0.0, f64::max),
            ..config.icp
        }).align(&seeded, target);
        // 求解器用无限误差表示没有求得估计；已有估计直接提交，分数只作诊断。
        if local.mse.is_finite() {
            let transform = local.transformation * base_anchor_map_from_odom;
            let score = evaluate_alignment(source, target, transform, config.fine_correspondence_distance);
            return RelocalizationComputeOutput {
                result: RelocalizationResult {
                    registration: IcpResult { transformation: transform, ..local },
                    feature_matches: 0,
                    ransac_inliers: score.inlier_count,
                    fitness: score.fitness,
                },
                correction: Some(base_anchor_map_from_odom.inverse() * transform),
            };
        }
        let result = relocation_pipeline_with_config(source, target, config);
        let correction = result.registration.converged
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
            Self::start_inner(spec,identity,provider,sink,relocalization_mode,None).await
        }

        pub(crate) async fn start_planar(identity:CorrectionOwnerRuntimeIdentity,provider:&str,sink:Arc<dyn CorrectionCandidateSink>,mode:Arc<RelocalizationModeFence>,frames:(String,String,String))->Result<Self,CorrectionRuntimeError> {
            Self::start_inner(CorrectionOwnerSpec::None,identity,provider,sink,mode,Some(frames)).await
        }

        async fn start_inner(spec:CorrectionOwnerSpec,identity:CorrectionOwnerRuntimeIdentity,provider:&str,sink:Arc<dyn CorrectionCandidateSink>,relocalization_mode:Arc<RelocalizationModeFence>,planar_frames:Option<(String,String,String)>)->Result<Self,CorrectionRuntimeError> {
            let planar_only=planar_frames.is_some();
            let mut planar_store=if planar_only {Some(RelocalizationTerminalStore::open().map_err(CorrectionRuntimeError::Apply)?)} else {None};
            let background = matches!(
                &spec,
                CorrectionOwnerSpec::LibraryPgo { .. } | CorrectionOwnerSpec::NativePgo { .. }
            )
            .then(|| {
                Arc::new(BackgroundCorrectionQueue {
                    pending: parking_lot::Mutex::new(None),
                    ready: Notify::new(),
                    mode: Arc::clone(&relocalization_mode),
                })
            });
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
                _ => planar_frames,
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
                background: background.clone(),
                sender: request_tx,
                admission_open: Arc::clone(&admission_open),
                admission_rx: admission_rx.clone(),
                relocalization_mode: phase_a.relocalization_mode(),
            };
            let (producer_worker, relocalization) = match relocalization_frames {
                Some((map_frame_id, odom_frame_id, body_frame_id)) if planar_only => {
                    drop(request_rx);
                    (None,Some(RelocalizationOwnerSupervisor::start_inner(phase_a.clone(),identity.clone(),map_frame_id,odom_frame_id,body_frame_id,None,planar_store.take())))
                },
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
                                background,
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
        if message_type.fingerprint() != CORRECTION_CANDIDATE_FINGERPRINT_V3 {
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
        let mut background_preparing = false;
        let event = loop {
            break tokio::select! {
                biased;
                _ = cancellation.cancelled() => CandidateApplyWaitOutcome::CancelledBeforeCommit,
                _ = wait_for_admission_close(admission_rx) => {
                    cancellation.cancel();
                    CandidateApplyWaitOutcome::AdmissionClosedBeforeCommit
                }
                result = &mut apply_rx => CandidateApplyWaitOutcome::Ack(result),
                _ = cancellation.background_preparation_started(),
                    if cancellation.is_background_mapping() && !background_preparing => {
                    // 交付截止时间只监督尚未到达拥有方的候选。历史计算开始后由分段取消
                    // 与原提交看门狗监督，不能在五秒时再次抹掉已经推进的后台重建。
                    background_preparing = true;
                    continue;
                }
                _ = &mut deadline, if !background_preparing => {
                    cancellation.cancel();
                    CandidateApplyWaitOutcome::TimedOutBeforeCommit
                }
            };
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
        let background = context.background.clone();
        let run_result =
            run_candidate_producer_loop(&mut owner, &mut requests, &mut admission_rx, context)
                .await;
        if let Some(queue) = background {
            queue.pending.lock().take();
        }
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

    const CANDIDATE_FINALIZATION_EVENT_NAME: &str = "nuwax_correction_candidate_finalized_v3";

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
        correction_lease_boot_nonce: String,
        correction_lease_generation: u64,
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
                correction_lease_boot_nonce: format!("{:032x}", candidate.identity.correction_lease.boot_nonce),
                correction_lease_generation: candidate.identity.correction_lease.generation,
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
                candidate: candidate.clone(),
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
            background,
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
                request = BackgroundCorrectionQueue::receive(&background) => request,
            };
            if request.cancellation.is_cancelled() {
                continue;
            }
            let stage_result = tokio::select! {
                biased;
                _ = wait_for_admission_close(admission_rx) => {
                    request.cancellation.cancel();
                    sink.discard_frame(&request.input).await;
                    break;
                }
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
            let produced = {
                let producing = owner.submit_frame(request.input, &request.cancellation);
                tokio::pin!(producing);
                tokio::select! {
                    biased;
                    _ = wait_for_admission_close(admission_rx) => {
                        request.cancellation.cancel();
                        producing.await
                    }
                    result = &mut producing => result,
                }
            };
            let production = match produced {
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
            let apply_key = CandidateApplyKey::from(&candidate);
            let apply_rx = match apply_waiters.register(apply_key, candidate.clone(), Arc::clone(&request.lease)) {
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
                    let Some(lease) = apply_waiters.claim_apply(apply_key, &candidate) else {
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
            let apply_key = CandidateApplyKey::from(&candidate);
            let Some(lease) = apply_waiters.claim_apply(apply_key, &candidate) else {
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
                terminal_store_root_from(Some(OsString::new()), None, None, current_directory,),
                Err("RELOCALIZATION_TERMINAL_STORE_PATH_INVALID".to_owned())
            );
        }

        #[test]
        fn relocalization_default_and_relative_roots_persist_and_reopen() {
            for (explicit, runtime, expected) in [
                (None, None, "runtime_data/geometry/relocalization_terminals_v1"),
                (Some("relative-terminals"), None, "relative-terminals"),
                (None, Some("relative-data"), "relative-data/geometry/relocalization_terminals_v1"),
            ] {
                let base = RelocalizationTerminalTestRoot::new();
                let root = terminal_store_root_from(explicit.map(OsString::from), None, runtime.map(OsString::from),
                    || Ok(base.path().to_path_buf())).unwrap();
                assert_eq!(root, base.path().join(expected));
                let request = validate_relocalize_request(relocalization_command_request("relative-reopen", 7)).unwrap();
                let terminal = relocalization_command_terminal(&request, crate::proto::RelocalizeDisposition::Cancelled, "cancelled");
                { let store = RelocalizationTerminalStore::open_at(root.clone()).unwrap();
                  store.persist_terminal(request.request_id(), &request.key(), &terminal).unwrap(); }
                let reopened = RelocalizationTerminalStore::open_at(root).unwrap();
                match reopened.snapshot_exact(request.request_id(), &request.key()) {
                    RelocalizationTerminalSnapshot::AssignedTerminal(value) => assert_eq!(value.encoded(), terminal.encoded()),
                    other => panic!("unexpected reopened terminal {other:?}"),
                }
            }
        }

        #[test]
        fn durable_relocalization_terminal_store_reopens_and_detects_identity_conflict() {
            let root = RelocalizationTerminalTestRoot::new();
            let request = validate_relocalize_request(relocalization_command_request("reopen-exact", 7)).unwrap();
            let terminal = relocalization_command_terminal(&request, crate::proto::RelocalizeDisposition::Cancelled, "cancelled");
            { let store = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
              store.mark_pending(&request).unwrap();
              assert!(matches!(store.snapshot_exact(request.request_id(), &request.key()), RelocalizationTerminalSnapshot::Pending));
              store.persist_terminal(request.request_id(), &request.key(), &terminal).unwrap(); }
            let reopened = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
            match reopened.snapshot_exact(request.request_id(), &request.key()) {
                RelocalizationTerminalSnapshot::AssignedTerminal(value) => assert_eq!(value.encoded(), terminal.encoded()),
                other => panic!("unexpected reopened terminal {other:?}"),
            }
            let conflict = validate_relocalize_request(relocalization_command_request("reopen-exact", 8)).unwrap();
            assert!(matches!(reopened.snapshot_exact(request.request_id(), &conflict.key()), RelocalizationTerminalSnapshot::IdentityConflict));
            assert!(matches!(reopened.snapshot(request.request_id(), [3;32], [5;32]), RelocalizationTerminalSnapshot::IdentityConflict));
        }

        #[test]
        fn durable_relocalization_terminal_store_rejects_noncanonical_record_request_id() {
            let root = RelocalizationTerminalTestRoot::new();
            import_legacy_terminal_fixture(root.path(), "legacy-id");
            let path = root.path().join("legacy-fixture.json");
            let mut record: DurableRelocalizationTerminalV1 = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
            record.request_id = " invalid".into();
            fs::write(path, serde_json::to_vec(&record).unwrap()).unwrap();
            assert_eq!(RelocalizationTerminalStore::open_at(root.path().into()).err().unwrap(), "RELOCALIZATION_REQUEST_ID_INVALID");
        }

        #[test]
        fn legacy_terminal_reads_are_opaque_alias_only_and_share_assigned_capacity() {
            let root = RelocalizationTerminalTestRoot::new();
            let historical = import_legacy_terminal_fixture(root.path(), "legacy-shared");
            let store = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
            match store.snapshot("legacy-shared", [3;32], [5;32]) {
                RelocalizationTerminalSnapshot::Terminal(value) => assert_eq!(value.encode_to_vec(), historical.encode_to_vec()),
                other => panic!("historical read mismatch {other:?}"),
            }
            let request = validate_relocalize_request(relocalization_command_request("legacy-shared", 7)).unwrap();
            let mut ledger = RelocalizationCommandLedger::from_terminal_store(&store);
            assert_eq!(ledger.records.len(), 1);
            assert!(matches!(ledger.admit(&request, 1, Instant::now(), || relocalization_command_terminal(
                &request, crate::proto::RelocalizeDisposition::ReplayConflict, "mixed identity" )).unwrap(), RelocalizationLedgerAdmission::Conflict(_)));
            assert_eq!(ledger.admitted_command_count, 0);
            assert_eq!(ledger.claimed_command_count, 0);
            assert_eq!(ledger.next_admission_ordinal, 0);
            assert!(!ledger.relocalization_mode.snapshot().active);
            let assigned = relocalization_command_terminal(&request, crate::proto::RelocalizeDisposition::Cancelled, "assigned");
            assert!(store.persist_terminal(request.request_id(), &request.key(), &assigned).is_err());
            // Altering canonical historical content with unchanged checksum is
            // accepted as an opaque read, never advertised as verified content.
            drop(store); drop(ledger);
            let path = root.path().join("legacy-fixture.json");
            let mut record: DurableRelocalizationTerminalV1 = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
            let mut changed = historical;
            changed.detail.push_str(" changed");
            record.terminal_hex = encode_hex(&changed.encode_to_vec());
            fs::write(&path, serde_json::to_vec(&record).unwrap()).unwrap();
            assert!(RelocalizationTerminalStore::open_at(root.path().into()).is_ok());
            record.terminal_hex.push_str("f80701"); // unknown protobuf field is noncanonical
            fs::write(&path, serde_json::to_vec(&record).unwrap()).unwrap();
            assert_eq!(RelocalizationTerminalStore::open_at(root.path().into()).err().unwrap(), "RELOCALIZATION_TERMINAL_STORE_NON_CANONICAL");
        }

        #[test]
        fn assigned_terminal_reopen_rejects_truncation_mixed_schema_and_duplicate_ids() {
            let root = RelocalizationTerminalTestRoot::new();
            let request = validate_relocalize_request(relocalization_command_request("duplicate-record", 7)).unwrap();
            let terminal = relocalization_command_terminal(&request, crate::proto::RelocalizeDisposition::Cancelled, "cancelled");
            { let store = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
              store.persist_terminal(request.request_id(), &request.key(), &terminal).unwrap(); }
            let path = root.path().join(assigned_terminal_filename_v2(request.request_id()).unwrap());
            let original = fs::read(&path).unwrap();
            fs::write(&path, &original[..original.len()-1]).unwrap();
            assert!(RelocalizationTerminalStore::open_at(root.path().into()).is_err());
            fs::write(&path, &original).unwrap();
            import_legacy_terminal_fixture(root.path(), request.request_id());
            assert_eq!(RelocalizationTerminalStore::open_at(root.path().into()).err().unwrap(), "RELOCALIZATION_TERMINAL_STORE_DUPLICATE");
        }

        #[test]
        fn durable_relocalization_terminal_store_removes_temporary_on_publish_conflict() {
            let root = RelocalizationTerminalTestRoot::new();
            let request = validate_relocalize_request(relocalization_command_request("publish-conflict", 7)).unwrap();
            let terminal = relocalization_command_terminal(&request, crate::proto::RelocalizeDisposition::Cancelled, "cancelled");
            let store = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
            let path = root.path().join(assigned_terminal_filename_v2(request.request_id()).unwrap());
            fs::write(&path, b"different terminal").unwrap();
            assert_eq!(store.persist_terminal(request.request_id(), &request.key(), &terminal), Err("RELOCALIZATION_TERMINAL_STORE_IDENTITY_CONFLICT".into()));
            assert_eq!(fs::read(&path).unwrap(), b"different terminal");
            assert!(!fs::read_dir(root.path()).unwrap().map(Result::unwrap).any(|entry|
                entry.file_name().to_str().is_some_and(is_owned_terminal_temporary_name)));
            assert!(store.state.lock().terminals.is_empty());
        }

        #[test]
        fn durable_relocalization_terminal_store_bounds_publish_conflict_read() {
            let root = RelocalizationTerminalTestRoot::new();
            let request = validate_relocalize_request(relocalization_command_request("bounded-conflict", 7)).unwrap();
            let terminal = relocalization_command_terminal(&request, crate::proto::RelocalizeDisposition::Cancelled, "cancelled");
            let store = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
            let path = root.path().join(assigned_terminal_filename_v2(request.request_id()).unwrap());
            File::create(&path).unwrap().set_len(RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES as u64 + 1).unwrap();
            assert_eq!(store.persist_terminal(request.request_id(), &request.key(), &terminal), Err("RELOCALIZATION_TERMINAL_STORE_RECORD_TOO_LARGE".into()));
            assert!(!fs::read_dir(root.path()).unwrap().map(Result::unwrap).any(|entry|
                entry.file_name().to_str().is_some_and(is_owned_terminal_temporary_name)));
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

        fn relocalization_request_body(request: &mut crate::proto::RelocalizeRequest) -> &mut crate::proto::AssignedCloudRelocalizeRequestV2 {
            match request.assigned_target.as_mut().unwrap() {
                crate::proto::relocalize_request::AssignedTarget::AssignedCloudV2(body) => body,
            }
        }

        fn relocalization_terminal_body(result: &crate::proto::RelocalizeResult) -> &crate::proto::AssignedCloudRelocalizeResultV2 {
            assert_eq!(result.schema_version, 2);
            assert!(result.request_identity_sha256.is_empty());
            assert!(result.terminal_proof_sha256.is_empty());
            assert!(result.candidate_payload_preimage_v1.is_empty());
            assert!(result.evidence_disclosure_v1.is_empty());
            result.assigned_cloud_v2.as_ref().unwrap()
        }

        fn relocalization_command_request(request_id: &str, target_cloud_revision: u64) -> crate::proto::RelocalizeRequest {
            let mut request = crate::relocalization_identity::tests::wire();
            request.request_id = request_id.into();
            let body = relocalization_request_body(&mut request);
            body.logical_timeout_ms = 1_000;
            let target = body.target.as_mut().unwrap();
            let activation = target.activation.as_mut().unwrap();
            activation.runtime_generation = 2;
            activation.session_generation = 1;
            activation.backend_generation = 4;
            let activation_request = activation.request.as_mut().unwrap();
            activation_request.expected_runtime_generation = 2;
            activation_request.session_id = "mapping-session-owner".into();
            activation_request.map_name = "map-ledger".into();
            let occupancy = target.occupancy_receipt.as_mut().unwrap();
            occupancy.session_id = "mapping-session-owner".into();
            occupancy.session_generation = 1;
            occupancy.backend_generation = 4;
            let cloud = target.cloud_receipt.as_mut().unwrap();
            cloud.session_generation = 1;
            cloud.cloud_revision = target_cloud_revision;
            request
        }

        fn relocalization_command_terminal(request: &ValidatedRelocalizeRequest,
            disposition: crate::proto::RelocalizeDisposition, detail: &str) -> StoredRelocalizationResult {
            let result = assigned_relocalization_result(&request.identity, disposition, 0, detail, Vec::new(),
                crate::proto::AssignedRelocalizationEvidenceV2 {
                    schema_version: 2, phase: crate::proto::AssignedRelocalizationEvidencePhaseV2::PreAdmission as i32,
                    ..Default::default()
                });
            StoredRelocalizationResult::Assigned(AssignedRelocalizationTerminalV2::new(result, None).unwrap())
        }

        fn relocalization_query(request: &ValidatedRelocalizeRequest) -> AssignedRelocalizationTerminalQueryV2 {
            AssignedRelocalizationTerminalQueryV2::admit(crate::proto::GetRelocalizationTerminalRequest {
                schema_version: 2, request_id: request.request_id().into(), assigned_request_v2: Some(request.identity.to_wire()),
                ..Default::default()
            }).unwrap()
        }

        fn import_legacy_terminal_fixture(root: &Path, request_id: &str) -> crate::proto::RelocalizeResult {
            fs::create_dir_all(root).unwrap();
            let result = crate::proto::RelocalizeResult {
                schema_version: 1, request_id: request_id.into(), request_identity_sha256: vec![3;32],
                activation_identity_sha256: vec![5;32], disposition: crate::proto::RelocalizeDisposition::Cancelled as i32,
                detail: "historical opaque alias fixture".into(), ..Default::default()
            };
            let record = DurableRelocalizationTerminalV1 {
                schema_version: 1, request_id: request_id.into(), request_identity_sha256: encode_hex(&[3;32]),
                activation_identity_sha256: encode_hex(&[5;32]), terminal_hex: encode_hex(&result.encode_to_vec()),
                terminal_sha256: encode_hex(&[7;32]),
            };
            // Name and checksum are historical opaque aliases; this is a test
            // import, not the live assigned writer.
            fs::write(root.join("legacy-fixture.json"), serde_json::to_vec(&record).unwrap()).unwrap();
            result
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
                .mark_postclaim("req-a", &request.key())
                .expect("first claim");
            ledger
                .mark_postclaim("req-a", &request.key())
                .expect("idempotent claim");
            let terminal = relocalization_command_terminal(
                &request,
                crate::proto::RelocalizeDisposition::Cancelled,
                "RELOCALIZATION_CANCELLED",
            );
            ledger
                .settle_terminal("req-a", &request.key(), terminal)
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
                .settle_terminal("req-a", &request.key(), terminal.clone())
                .expect("terminal settlement");

            let stored_before_notify = ledger.terminal("req-a").expect("terminal is durable");
            assert_eq!(stored_before_notify.encoded(), terminal.encoded());
            assert_eq!(pending_notifications.waiter_ids, vec![9]);
            assert_eq!(pending_notifications.result.encoded(), terminal.encoded());

            let replay = ledger
                .admit(&request, 10, Instant::now(), || unreachable!("no conflict"))
                .expect("terminal replay");
            let replay = match replay {
                RelocalizationLedgerAdmission::Replay(result) => result,
                _ => panic!("terminal request must replay"),
            };
            assert_eq!(replay.encoded(), terminal.encoded());
            assert_eq!(replay.encoded(), terminal.encoded());
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
            assert_eq!(first_conflict.encoded(), second_conflict.encoded());

            let terminal = relocalization_command_terminal(
                &request_a,
                crate::proto::RelocalizeDisposition::Cancelled,
                "RELOCALIZATION_CANCELLED",
            );
            ledger
                .settle_terminal("req-a", &request_a.key(), terminal)
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
                        request.request_id(),
                        &request.key(),
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
                    assert_eq!(result.encoded(), overflow_terminal.encoded())
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
            assert_eq!(first_overflow.encoded(), second_overflow.encoded());
            assert_eq!(
                first_overflow.encoded(),
                second_overflow.encoded()
            );
            let record = conflicts
                .records
                .get("req-conflict-cap")
                .expect("base record remains");
            let (stored_identity, stored_overflow) = record
                .conflict_overflow
                .as_ref()
                .expect("overflow terminal is retained before reply");
            assert_eq!(*stored_identity, overflow_conflict.key());
            assert_eq!(stored_overflow.encoded(), first_overflow.encoded());
            assert_eq!(
                stored_overflow.encoded(),
                first_overflow.encoded()
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
                ).unwrap();
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
                        planar_work: None,
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
            assert_eq!(stored.0, overflow.key());
            let stored_encoded = stored.1.encoded().to_vec();
            let stored_encoded_bytes = stored.1.encoded().to_vec();
            let waiter = response_rx
                .await
                .expect("actor reply is delivered")
                .expect("overflow conflict returns a waiter");
            let reply = waiter.wait().await.expect("overflow terminal is delivered");
            assert_eq!(reply.encode_to_vec(), stored_encoded);
            assert_eq!(reply.encode_to_vec(), stored_encoded_bytes);
            assert!(!phase_a.is_open(), "overflow closes further admission");

            let replay = actor
                .ledger
                .admit(&overflow, 100, Instant::now(), || {
                    panic!("stored overflow conflict must replay")
                })
                .expect("overflow conflict replays");
            match replay {
                RelocalizationLedgerAdmission::Conflict(result) => {
                    assert_eq!(result.encoded(), stored_encoded);
                    assert_eq!(result.encoded(), stored_encoded_bytes);
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
                .detach_waiter("req-a", &request.key(), 1)
                .expect("detach first waiter"));
            assert!(preclaim
                .detach_waiter("req-a", &request.key(), 2)
                .expect("detach final preclaim waiter"));

            let mut postclaim = RelocalizationCommandLedger::default();
            postclaim
                .admit(&request, 3, Instant::now(), || unreachable!("no conflict"))
                .expect("postclaim request");
            postclaim
                .mark_postclaim("req-a", &request.key())
                .expect("mark claimed");
            assert!(!postclaim
                .detach_waiter("req-a", &request.key(), 3)
                .expect("detach postclaim waiter"));
        }

        #[test]
        fn relocalization_restored_ledger_preserves_binding_conflicts_overflow_and_capacity() {
            let root = RelocalizationTerminalTestRoot::new();
            let request = validate_relocalize_request(relocalization_command_request(
                "req-restored-bounds",
                7,
            ))
            .unwrap();
            let terminal = relocalization_command_terminal(
                &request,
                crate::proto::RelocalizeDisposition::Cancelled,
                "RELOCALIZATION_CANCELLED",
            );
            {
                let store =
                    RelocalizationTerminalStore::open_at(root.path().to_path_buf()).unwrap();
                store
                    .persist_terminal(
                        request.request_id(),
                        &request.key(),
                        &terminal,
                    )
                    .unwrap();
            }
            let store = RelocalizationTerminalStore::open_at(root.path().to_path_buf()).unwrap();
            let mut ledger = RelocalizationCommandLedger::from_terminal_store(&store);
            let before = ledger.observation().unwrap();
            let mode_before = ledger.relocalization_mode.snapshot();
            assert!(ledger.request(request.request_id()).is_none());
            assert!(ledger.active_identity().is_none());
            assert!(matches!(
                &ledger.records[request.request_id()].origin,
                RelocalizationLedgerOrigin::RestoredTerminal
            ));
            assert_eq!(before.admitted_command_count, 0);
            assert_eq!(before.claimed_command_count, 0);
            assert_eq!(before.terminal_record_count, 0);
            assert!(ledger
                .mark_postclaim(request.request_id(), &request.key())
                .is_err());
            assert!(ledger
                .detach_waiter(request.request_id(), &request.key(), 1)
                .is_err());

            for index in 0..RELOCALIZATION_MAX_CONFLICTS_PER_RECORD {
                let changed = validate_relocalize_request(relocalization_command_request(
                    request.request_id(),
                    index as u64 + 8,
                ))
                .unwrap();
                let conflict = relocalization_command_terminal(
                    &changed,
                    crate::proto::RelocalizeDisposition::ReplayConflict,
                    "RELOCALIZATION_REPLAY_CONFLICT",
                ).unwrap();
                let first = ledger
                    .admit(&changed, index as u64 + 1, Instant::now(), || {
                        conflict.clone()
                    })
                    .unwrap();
                let repeat = ledger
                    .admit(&changed, index as u64 + 1, Instant::now(), || {
                        panic!("restored conflict must be retained")
                    })
                    .unwrap();
                match (first, repeat) {
                    (
                        RelocalizationLedgerAdmission::Conflict(first),
                        RelocalizationLedgerAdmission::Conflict(repeat),
                    ) => {
                        assert_eq!(first.encoded(), conflict.encoded());
                        assert_eq!(repeat.encoded(), first.encoded());
                    }
                    _ => panic!("restored binding must retain each bounded conflict"),
                }
            }
            let overflow = validate_relocalize_request(relocalization_command_request(
                request.request_id(),
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD as u64 + 8,
            ))
            .unwrap();
            let overflow_terminal = relocalization_command_terminal(
                &overflow,
                crate::proto::RelocalizeDisposition::ReplayConflict,
                "RELOCALIZATION_REPLAY_CONFLICT",
            );
            assert!(matches!(
                ledger
                    .admit(&overflow, 100, Instant::now(), || overflow_terminal.clone())
                    .unwrap(),
                RelocalizationLedgerAdmission::ConflictOverflow(_)
            ));
            match ledger
                .admit(&overflow, 101, Instant::now(), || {
                    panic!("overflow must replay")
                })
                .unwrap()
            {
                RelocalizationLedgerAdmission::Conflict(replayed) => {
                    assert_eq!(replayed.encoded(), overflow_terminal.encoded())
                }
                _ => panic!("restored overflow identity must remain permanent"),
            }
            let next_conflict = validate_relocalize_request(relocalization_command_request(
                request.request_id(),
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD as u64 + 9,
            ))
            .unwrap();
            assert!(matches!(
                ledger
                    .admit(&next_conflict, 102, Instant::now(), || panic!(
                        "overflow is bounded"
                    ))
                    .unwrap(),
                RelocalizationLedgerAdmission::CapacityExceeded(
                    "RELOCALIZATION_COMMAND_CONFLICT_OVERFLOW_CLOSED"
                )
            ));
            for waiter_id in [103, 104] {
                match ledger
                    .admit(&request, waiter_id, Instant::now(), || {
                        panic!("exact retry is not a conflict")
                    })
                    .unwrap()
                {
                    RelocalizationLedgerAdmission::Replay(replayed) => {
                        assert_eq!(replayed.encoded(), terminal.encoded())
                    }
                    _ => panic!("exact restored terminal survives conflict overflow"),
                }
            }
            assert_eq!(
                ledger.observation().unwrap().encode_to_vec(),
                before.encode_to_vec()
            );
            assert_eq!(ledger.relocalization_mode.snapshot(), mode_before);
            assert_eq!(ledger.next_admission_ordinal, 0);
            assert_eq!(
                ledger.records[request.request_id()].conflicts.len(),
                RELOCALIZATION_MAX_CONFLICTS_PER_RECORD
            );

            // The store binds the full assigned descriptor. A same-ID retry
            // cannot substitute the activation lifecycle binding.
            let mut activation_ledger = RelocalizationCommandLedger::from_terminal_store(&store);
            let mut changed_wire = request.identity.to_wire();
            relocalization_request_body(&mut changed_wire).target.as_mut().unwrap().activation.as_mut().unwrap().lifecycle_epoch += 1;
            let changed_activation = validate_relocalize_request(changed_wire).unwrap();
            assert!(matches!(
                activation_ledger
                    .admit(&changed_activation, 1, Instant::now(), || terminal.clone())
                    .unwrap(),
                RelocalizationLedgerAdmission::Conflict(_)
            ));
            assert_eq!(activation_ledger.admitted_command_count, 0);
            assert_eq!(activation_ledger.claimed_command_count, 0);

            // One restored record consumes one existing slot; only the first
            // genuinely new command consumes ordinal 1 and a fresh deadline.
            let mut bounded = RelocalizationCommandLedger::from_terminal_store(&store);
            for index in 0..RELOCALIZATION_MAX_COMMAND_RECORDS.saturating_sub(2) {
                let new_request = validate_relocalize_request(relocalization_command_request(
                    &format!("req-restored-cap-{index}"),
                    7,
                ))
                .unwrap();
                let now = Instant::now();
                let (ordinal, deadline) = match bounded
                    .admit(&new_request, index as u64 + 1, now, || unreachable!())
                    .unwrap()
                {
                    RelocalizationLedgerAdmission::Started {
                        admission_ordinal,
                        logical_deadline,
                    } => (admission_ordinal, logical_deadline),
                    _ => panic!("remaining bounded slot must admit"),
                };
                assert_eq!(ordinal, index as u64 + 1);
                assert_eq!(deadline, now + Duration::from_secs(1));
                assert!(
                    matches!(bounded.admit(&new_request, index as u64 + 2, now + Duration::from_millis(900), || unreachable!()).unwrap(),
                    RelocalizationLedgerAdmission::Joined { admission_ordinal, logical_deadline }
                    if admission_ordinal == ordinal && logical_deadline == deadline)
                );
                bounded
                    .settle_terminal(
                        new_request.request_id(),
                        &new_request.key(),
                        relocalization_command_terminal(
                            &new_request,
                            crate::proto::RelocalizeDisposition::Cancelled,
                            "RELOCALIZATION_CANCELLED",
                        ),
                    )
                    .unwrap();
            }
            let at_capacity = bounded.next_admission_ordinal;
            let capacity_request = validate_relocalize_request(relocalization_command_request(
                "req-restored-capacity",
                7,
            ))
            .unwrap();
            assert!(matches!(
                bounded
                    .admit(&capacity_request, 10_000, Instant::now(), || unreachable!())
                    .unwrap(),
                RelocalizationLedgerAdmission::CapacityExceeded(
                    "RELOCALIZATION_COMMAND_LEDGER_CAPACITY_EXCEEDED"
                )
            ));
            let capacity_terminal = relocalization_command_terminal(
                &capacity_request,
                crate::proto::RelocalizeDisposition::OwnerFailed,
                "RELOCALIZATION_COMMAND_LEDGER_CAPACITY_EXCEEDED",
            );
            bounded
                .store_capacity_terminal(
                    &capacity_request,
                    Instant::now(),
                    capacity_terminal.clone(),
                )
                .unwrap();
            assert_eq!(bounded.records.len(), RELOCALIZATION_MAX_COMMAND_RECORDS);
            assert_eq!(bounded.next_admission_ordinal, at_capacity);
            assert_eq!(
                bounded.admitted_command_count,
                RELOCALIZATION_MAX_COMMAND_RECORDS as u64 - 1
            );
            assert_eq!(bounded.claimed_command_count, 0);
            let observation = bounded.observation().unwrap();
            assert_eq!(observation.terminal_record_count, observation.admitted_command_count);
            assert_eq!(observation.pending_record_count, 0);
            for (request, expected) in [
                (&request, &terminal),
                (&capacity_request, &capacity_terminal),
            ] {
                match bounded
                    .admit(request, 10_001, Instant::now(), || unreachable!())
                    .unwrap()
                {
                    RelocalizationLedgerAdmission::Replay(replayed) => {
                        assert_eq!(replayed.encoded(), expected.encoded())
                    }
                    _ => panic!("full ledger must still replay existing terminals"),
                }
            }
        }

        fn install_relocalization_target_for_request(
            phase_a: &CorrectionPhaseAControl,
            request: &mut crate::proto::RelocalizeRequest,
            map_frame_id: &str,
        ) {
            install_relocalization_target_points_for_request(phase_a, request, map_frame_id, asymmetric_cloud());
        }

        fn install_relocalization_target_points_for_request(
            phase_a: &CorrectionPhaseAControl,
            request: &mut crate::proto::RelocalizeRequest,
            map_frame_id: &str,
            points: Vec<Point3<f64>>,
        ) -> SealedAssignedActivatedTargetSnapshot {
            let requested = relocalization_request_body(request).target.as_ref().unwrap().clone();
            let requested_activation = requested.activation.as_ref().unwrap();
            let requested_metadata = requested_activation.request.as_ref().unwrap();
            let cloud_revision = requested.cloud_receipt.as_ref().unwrap().cloud_revision;
            let control = phase_a.activated_target_receipts();
            let mut wire = assigned_wire_request();
            wire.expected_runtime_generation = control.runtime_generation();
            wire.activation_request_id = "activation-relocalization-runtime".into();
            wire.session_id = requested_metadata.session_id.clone();
            wire.map_name = requested_metadata.map_name.clone();
            let checkpoint = wire.mapping_backend_checkpoint.as_mut().unwrap();
            let point_count = points.len().try_into().unwrap();
            checkpoint.cloud_receipt.as_mut().unwrap().point_count = point_count;
            for member in [checkpoint.backend_identity.as_mut().unwrap(), checkpoint.occupancy_identity.as_mut().unwrap()] {
                member.frame_id = map_frame_id.into();
                member.cloud_receipt.as_mut().unwrap().point_count = point_count;
            }
            checkpoint.canonical_occupancy_checkpoint.as_mut().unwrap().meta.as_mut().unwrap().frame_id = map_frame_id.into();
            let cloud = checkpoint.canonical_cloud_checkpoint.as_mut().unwrap();
            cloud.frame_id = map_frame_id.into();
            cloud.point_count = point_count;
            cloud.xyz_f32_le = points.into_iter().flat_map(|p| [p.x as f32, p.y as f32, p.z as f32])
                .flat_map(f32::to_le_bytes).collect();
            let member = cloud.member_identity.as_mut().unwrap();
            member.frame_id = map_frame_id.into();
            member.cloud_receipt.as_mut().unwrap().point_count = point_count;
            let admitted = ValidatedActivationRequestV2::admit(
                wire, control.runtime_generation(), crate::mapping_backend::MappingBackendKind::Voxel,
                map_frame_id, |_| panic!("fixture has no artifacts"),
                &crate::activation_request::CheckpointOwnerInternerV2::default(),
            ).unwrap();
            let mut state = activation_base_state();
            state.session_id = requested_metadata.session_id.clone();
            state.map_name = requested_metadata.map_name.clone();
            state.session_generation = 1;
            let token = match control.stage_assigned_or_replay(admitted, state.clone(), true) {
                ActivationLedgerAdmission::Execute(token) => token,
                ActivationLedgerAdmission::Terminal(_) => panic!("assigned target must stage"),
            };
            let token = control.reserve_backend_generation(&token, 1).unwrap();
            let token = control.bind_assigned_activation_identity(&token, state.session_generation).unwrap();
            let mut proof = assigned_proof(&control, &token);
            proof.cloud_revision = cloud_revision;
            proof.cloud_receipt.cloud_revision = cloud_revision;
            let TargetReceipt::Assigned(receipt) = &mut proof.target_receipt else { unreachable!() };
            receipt.cloud_receipt.cloud_revision = cloud_revision;
            let target = receipt.clone();
            assert!(control.store_publication_proof(&token, proof));
            control.seal(&token, Some(TargetReceipt::Assigned(target.clone())), state).unwrap();
            relocalization_request_body(request).target = Some(target.to_wire());
            validate_relocalize_request(request.clone()).unwrap();
            control.current_sealed_assigned_target_snapshot().unwrap()
        }

        fn relocalization_terminal_candidate_sequence(result: &crate::proto::RelocalizeResult) -> u64 {
            let body = relocalization_terminal_body(result);
            if body.candidate_payload_v3.is_empty() { 0 } else {
                CorrectionCandidate::decode_body(&body.candidate_payload_v3).unwrap().identity.candidate_sequence
            }
        }

        fn active_relocalization_frame_with_points(
            owner: &CorrectionTransportOwner,
            sequence: u64,
            timestamp_ns: u64,
            pose: Isometry3<f64>,
            body_points: Vec<Point3<f64>>,
        ) -> CorrectionFrameInput {
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let fence = owner.ingress().relocalization_mode_fence();
            let mode = fence.snapshot();
            assert!(mode.active);
            let mut input = frame_from_admitted_source(sequence, admit_frame_source(
                sequence, timestamp_ns, timestamp_ns, pose,
                body_points.iter().map(|point| pose * point).collect(), &adapter, fence,
            ));
            input.relocalization_mode_epoch = Some(mode.epoch);
            input
        }

        async fn retained_relocalization_test_owner() -> CorrectionTransportOwner {
            CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization { map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(), body_frame_id: "robot_body".into() },
                CorrectionOwnerRuntimeIdentity { runtime_generation: 2, backend_generation: 99,
                    correction_lease: CorrectionLeaseId { boot_nonce: LEASE_NONCE, generation: 20 } },
                "", Arc::new(FailingStageSink),
            ).await.unwrap()
        }

        #[tokio::test]
        async fn default_profile_relocalization_replays_target_unavailable_terminal() {
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::preflight_for_frames(CorrectionBackend::None, LioBackend::PointLio,
                    "map", "odom", "base_link").unwrap(),
                CorrectionOwnerRuntimeIdentity { runtime_generation: 2, backend_generation: 99,
                    correction_lease: CorrectionLeaseId { boot_nonce: LEASE_NONCE, generation: 1 } },
                "", Arc::new(FailingStageSink),
            ).await.unwrap();
            let ingress = owner.relocalization_ingress().unwrap();
            let request = relocalization_command_request("req-default-target-unavailable", 7);
            let first = ingress.admit(request.clone()).await.unwrap().wait().await.unwrap();
            assert_eq!(relocalization_terminal_body(&first).disposition,
                crate::proto::RelocalizeDisposition::TargetUnavailable as i32);
            assert_eq!(relocalization_terminal_candidate_sequence(&first), 0);
            assert!(relocalization_terminal_body(&first).candidate_payload_v3.is_empty());
            let replay = ingress.admit(request).await.unwrap().wait().await.unwrap();
            assert_eq!(replay.encode_to_vec(), first.encode_to_vec());
            owner.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn relocalization_retained_command_phase_a_stores_terminal_before_reply() {
            let owner = retained_relocalization_test_owner().await;
            let phase_a = owner.phase_a_control();
            let ingress = owner.relocalization_ingress().unwrap();
            let mut request = relocalization_command_request("req-phase-a", 7);
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let validated = validate_relocalize_request(request.clone()).unwrap();
            let first = ingress.admit(request.clone()).await.unwrap();
            let second = ingress.admit(request).await.unwrap();
            first.cancel_preclaim();
            phase_a.begin();
            let result = tokio::time::timeout(Duration::from_secs(1), second.wait()).await.unwrap().unwrap();
            let body = relocalization_terminal_body(&result);
            assert_eq!(body.disposition, crate::proto::RelocalizeDisposition::OwnerStopped as i32);
            assert_eq!(body.detail, "RELOCALIZATION_OWNER_STOPPED_PRECLAIM");
            assert_eq!(relocalization_terminal_candidate_sequence(&result), 0);
            let RelocalizationTerminalSnapshot::AssignedTerminal(stored) = ingress.terminal_store
                .snapshot_exact(validated.request_id(), &validated.key()) else { panic!("terminal must precede reply") };
            assert_eq!(stored.encoded(), result.encode_to_vec());
            owner.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn relocalization_retained_command_deadline_replays_identical_terminal() {
            let owner = retained_relocalization_test_owner().await;
            let phase_a = owner.phase_a_control();
            let ingress = owner.relocalization_ingress().unwrap();
            let mut request = relocalization_command_request("req-deadline", 7);
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let first = tokio::time::timeout(Duration::from_secs(1),
                ingress.admit(request.clone()).await.unwrap().wait()).await.unwrap().unwrap();
            let body = relocalization_terminal_body(&first);
            assert_eq!(body.disposition, crate::proto::RelocalizeDisposition::DeadlineExceeded as i32);
            assert!(body.candidate_payload_v3.is_empty());
            let evidence = body.evidence.as_ref().unwrap();
            assert_eq!(evidence.phase, crate::proto::AssignedRelocalizationEvidencePhaseV2::Admitted as i32);
            assert!(evidence.admission_ordinal.is_some());
            assert!(evidence.owner.is_some());
            assert!(evidence.source_frames.is_empty());
            let replay = ingress.admit(request).await.unwrap().wait().await.unwrap();
            assert_eq!(replay.encode_to_vec(), first.encode_to_vec());
            owner.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn relocalization_retained_command_busy_is_transient_and_unproved() {
            let owner = retained_relocalization_test_owner().await;
            let phase_a = owner.phase_a_control();
            let ingress = owner.relocalization_ingress().unwrap();
            let mut request = relocalization_command_request("req-active", 7);
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let active = ingress.admit(request.clone()).await.unwrap();
            request.request_id = "req-busy".into();
            let busy = ingress.admit(request.clone()).await.unwrap().wait().await.unwrap();
            let body = relocalization_terminal_body(&busy);
            assert_eq!(body.disposition, crate::proto::RelocalizeDisposition::Busy as i32);
            let evidence = body.evidence.as_ref().unwrap();
            assert_eq!(evidence.phase, crate::proto::AssignedRelocalizationEvidencePhaseV2::PreAdmission as i32);
            assert!(evidence.owner.is_none() && evidence.admission_ordinal.is_none() && evidence.source_frames.is_empty());
            active.cancel_preclaim();
            let retried = ingress.admit(request).await.unwrap();
            phase_a.begin();
            let terminal = retried.wait().await.unwrap();
            assert_eq!(relocalization_terminal_body(&terminal).disposition, crate::proto::RelocalizeDisposition::OwnerStopped as i32);
            owner.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn relocalization_released_stale_proof_drops_without_blocking_fresh_guard() {
            let sink = Arc::new(RelocalizationAppliedSink::default());
            let owner = CorrectionTransportOwner::start(
                CorrectionOwnerSpec::Relocalization { map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(), body_frame_id: "robot_body".into() },
                CorrectionOwnerRuntimeIdentity { runtime_generation: 2, backend_generation: 99,
                    correction_lease: CorrectionLeaseId { boot_nonce: LEASE_NONCE, generation: 21 } },
                "", sink.clone(),
            ).await.unwrap();
            let mut request = relocalization_command_request("req-stale-proof-progress", 7);
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&owner.phase_a_control(), &mut request, "robot_map");
            let waiter = owner.relocalization_ingress().unwrap().admit(request).await.unwrap();
            let high = active_relocalization_frame_with_points(&owner, 100, 1_000, Isometry3::identity(), asymmetric_cloud());
            assert_eq!(owner.ingress().submit(high).await.unwrap(), CorrectionSubmitDisposition::NoCandidate);
            for _ in 0..2 {
                let stale = active_relocalization_frame_with_points(&owner, 1, 10, Isometry3::identity(), asymmetric_cloud());
                let stale_liveness = stale.source_owner.retention_probe_for_test();
                assert_eq!(owner.ingress().submit(stale).await.unwrap(), CorrectionSubmitDisposition::NoCandidate);
                tokio::time::timeout(Duration::from_secs(1), async {
                    while stale_liveness() { tokio::task::yield_now().await; }
                }).await.expect("stale proof tombstone never retains rejected source bytes");
                assert!(owner.ingress().is_open());
            }
            let fresh = active_relocalization_frame_with_points(&owner, 101, 1_010, Isometry3::identity(), asymmetric_cloud());
            let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
            assert_eq!(owner.ingress().submit_with_lifecycle_guard(fresh, guard).await.unwrap(), CorrectionSubmitDisposition::NoCandidate);
            let terminal = tokio::time::timeout(Duration::from_secs(20), waiter.wait()).await.unwrap().unwrap();
            let body = relocalization_terminal_body(&terminal);
            assert_eq!(body.disposition, crate::proto::RelocalizeDisposition::Applied as i32);
            assert_eq!(body.evidence.as_ref().unwrap().source_frames.len(), 2);
            assert_eq!(sink.applied.load(Ordering::Acquire), 1);
            owner.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn relocalization_runtime_without_guard_keeps_first_deadline() {
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .expect("command ingress exists")
                .admit(request)
                .await
                .expect("command starts accumulating");
            let disposition = owner
                .ingress()
                .submit(active_relocalization_frame(&owner, 1, 10, 0.0))
                .await
                .expect("non-cutoff frame is acknowledged");
            assert_eq!(disposition, CorrectionSubmitDisposition::NoCandidate);
            let terminal = tokio::time::timeout(Duration::from_secs(1), waiter.wait())
                .await
                .expect("first logical deadline remains bounded")
                .expect("deadline terminal is delivered");
            assert_eq!(
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::DeadlineExceeded as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
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
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), body_points,
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::DeadlineExceeded as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), body_points,
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::Applied as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 1);
            let terminal_body = relocalization_terminal_body(&terminal);
            let candidate = CorrectionCandidate::decode_body(&terminal_body.candidate_payload_v3).unwrap();
            assert_eq!(candidate.encode_body().unwrap(), terminal_body.candidate_payload_v3);
            assert!(terminal_body.candidate_payload_v3.len() <= RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES);
            let evidence = terminal_body.evidence.as_ref().unwrap();
            assert_eq!(evidence.phase, crate::proto::AssignedRelocalizationEvidencePhaseV2::Finalized as i32);
            assert_eq!(evidence.source_frames.len(), 1);
            assert_eq!(evidence.source_frames[0].admitted_point_count, RELOCALIZATION_MIN_AGGREGATE_POINTS as u64);
            assert_eq!(evidence.source_frames[0].runtime_generation, 31);
            assert_eq!(evidence.owner.as_ref().unwrap().runtime_generation, 2);
            assert_eq!(candidate.identity.registered_cloud.runtime_generation, 31);
            assert_eq!(evidence.source_frames[0].source_sequence, candidate.identity.registered_cloud.source_sequence);
            assert!(evidence.encoded_len() <= RELOCALIZATION_MAX_PROOF_BYTES);
            assert_eq!(relocalization_terminal_body(&terminal).correction_revision, 1);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
            let first_cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), body_points.clone(),
            );
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
                relocalization_terminal_body(&first).disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&first), 0);

            request.request_id = "req-sequence-applied".to_owned();
            let second = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request.clone())
                .await
                .expect("second command accumulates");
            let second_cutoff = active_relocalization_frame_with_points(
                &owner, 2, 20, Isometry3::identity(), body_points.clone(),
            );
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
                relocalization_terminal_body(&second).disposition,
                crate::proto::RelocalizeDisposition::Applied as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&second), 1);
            assert_eq!(relocalization_terminal_body(&second).correction_revision, 1);

            request.request_id = "req-sequence-rejected".to_owned();
            let third = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("third command accumulates");
            let third_cutoff = active_relocalization_frame_with_points(
                &owner, 3, 30, Isometry3::identity(), body_points,
            );
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
                relocalization_terminal_body(&third).disposition,
                crate::proto::RelocalizeDisposition::Rejected as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&third), 2);
            assert_eq!(relocalization_terminal_body(&third).correction_revision, 1);
            assert!(!relocalization_terminal_body(&third).candidate_payload_v3.is_empty());
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), body_points,
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::Applied as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 1);
            assert_eq!(relocalization_terminal_body(&terminal).correction_revision, 1);
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
                let key = CandidateApplyKey::from(&candidate);
                let lease = Arc::new(CorrectionRequestLease::new(
                    CorrectionRequestCancellation::new(),
                    None,
                ));
                dummy_receivers.push(
                    owner
                        .apply_waiters
                        .register(key, candidate.clone(), Arc::clone(&lease))
                        .expect("dummy waiter registers"),
                );
                let lease = owner
                    .apply_waiters
                    .claim_apply(key, &candidate)
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), body_points,
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
        async fn relocalization_sparse_window_matches_and_rejects_degenerate_geometry() {
            for degenerate in [false, true] {
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
                Arc::new(RelocalizationAppliedSink::default()),
            )
            .await
            .expect("runtime owner starts");
            let phase_a = owner.phase_a_control();
            let mut request = relocalization_command_request(if degenerate {"req-sparse-degenerate"} else {"req-sparse-valid"}, 7);
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            for sequence in 1..=RELOCALIZATION_MAX_FRAMES as u64 {
                let mut points = asymmetric_cloud();
                if degenerate { points.fill(Point3::origin()); }
                let input = active_relocalization_frame_with_points(
                    &owner, sequence, sequence * 10, Isometry3::identity(), points,
                );
                // First-guard progress is the current baseline. Accumulate
                // unguarded frames, then give the eighth frame its fresh guard.
                let disposition = if sequence == RELOCALIZATION_MAX_FRAMES as u64 {
                    let guard = Arc::new(tokio::sync::Mutex::new(())).lock_owned().await;
                    owner.ingress().submit_with_lifecycle_guard(input, guard).await
                } else {
                    owner.ingress().submit(input).await
                }.expect("small frame is acknowledged");
                assert_eq!(disposition, CorrectionSubmitDisposition::NoCandidate);
            }
            let terminal = tokio::time::timeout(Duration::from_secs(20), waiter.wait())
                .await
                .expect("稀疏窗口进入实际算法并返回终态")
                .expect("真实匹配终态已交付");
            assert_eq!(
                relocalization_terminal_body(&terminal).disposition,
                if degenerate { crate::proto::RelocalizeDisposition::AlgorithmRejected as i32 }
                else { crate::proto::RelocalizeDisposition::Applied as i32 }
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), if degenerate { 0 } else { 1 });
            owner.shutdown().await.expect("稀疏输入拥有方完成关闭");
            }
        }

        #[tokio::test]
        async fn relocalization_aggregate_maximum_without_guard_rejects_next_overflow() {
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            for (sequence, point_count) in [(1_u64, 100_000_usize), (2, 100_000), (3, 50_000)] {
                let input = active_relocalization_frame_with_points(
                    &owner, sequence, sequence * 10, Isometry3::identity(), vec![Point3::origin(); point_count],
                );
                assert_eq!(
                    owner
                        .ingress()
                        .submit(input)
                        .await
                        .expect("bounded frame is acknowledged"),
                    CorrectionSubmitDisposition::NoCandidate
                );
            }
            assert_eq!(sink.staged.load(Ordering::Acquire), 0, "no guard means no canonical stage");
            let overflow = active_relocalization_frame_with_points(
                &owner, 4, 40, Isometry3::identity(), vec![Point3::origin()],
            );
            assert!(matches!(owner.ingress().submit(overflow).await.unwrap(),
                CorrectionSubmitDisposition::Rejected { reason, .. }
                    if reason == "RELOCALIZATION_AGGREGATE_POINT_BOUND_EXCEEDED"));
            let terminal = waiter.wait().await.expect("next point beyond the exact maximum terminates");
            assert_eq!(
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::InsufficientSource as i32
            );
            assert_eq!(
                relocalization_terminal_body(&terminal).detail,
                "RELOCALIZATION_AGGREGATE_POINT_BOUND_EXCEEDED"
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(),
                vec![Point3::origin(); RELOCALIZATION_MIN_AGGREGATE_POINTS],
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::AlgorithmRejected as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
            let body = relocalization_terminal_body(&terminal);
            assert!(body.candidate_payload_v3.is_empty());
            let evidence = body.evidence.as_ref().unwrap();
            assert_eq!(evidence.phase, crate::proto::AssignedRelocalizationEvidencePhaseV2::AlgorithmCompleted as i32);
            assert_eq!(evidence.source_frames.len(), 1);
            assert!(evidence.algorithm.is_some());
            assert!(evidence.encoded_len() <= RELOCALIZATION_MAX_PROOF_BYTES);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), body_points,
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::TargetInvalidated as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command snapshots the current session");
            let mut adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            adapter.stop_session("mapping-session-owner").unwrap();
            adapter.start_session("mapping-session-next", "next-map").unwrap();
            establish_frame_source_anchor(&mut adapter, Isometry3::identity());
            let fence = owner.ingress().relocalization_mode_fence();
            let epoch = fence.snapshot().epoch;
            let mut rollover = frame_from_admitted_source(1, admit_frame_source(
                1, 10, 10, Isometry3::identity(), asymmetric_cloud(), &adapter, fence,
            ));
            rollover.relocalization_mode_epoch = Some(epoch);
            rollover.committed_map.backend_generation = adapter.backend_checkpoint_client().unwrap().backend_generation();
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::SessionInvalidated as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
            assert_eq!(relocalization_terminal_body(&terminal).detail, "RELOCALIZATION_FRAME_SESSION_INVALIDATED");
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
            let mut conflicting_points = first.body_points.to_vec();
            conflicting_points[0].x += 0.25;
            let conflict = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(), conflicting_points,
            );
            assert_eq!(relocalization_frame_key(&first), relocalization_frame_key(&conflict));
            conflict.validate_source_ownership().unwrap();
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
                relocalization_terminal_body(&command_terminal).disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(
                relocalization_terminal_body(&command_terminal).detail,
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
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
                relocalization_terminal_body(&command_terminal).disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(
                relocalization_terminal_body(&command_terminal).detail,
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(),
                vec![Point3::origin(); RELOCALIZATION_MIN_AGGREGATE_POINTS],
            );
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
                relocalization_terminal_body(&terminal).disposition,
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(),
                vec![Point3::origin(); RELOCALIZATION_MIN_AGGREGATE_POINTS],
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::OwnerStopped as i32
            );
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MAX_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let waiter = owner
                .relocalization_ingress()
                .unwrap()
                .admit(request)
                .await
                .expect("command accumulates");
            let cutoff = active_relocalization_frame_with_points(
                &owner, 1, 10, Isometry3::identity(),
                vec![Point3::origin(); RELOCALIZATION_MIN_AGGREGATE_POINTS],
            );
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
                relocalization_terminal_body(&terminal).disposition,
                crate::proto::RelocalizeDisposition::OwnerFailed as i32
            );
            assert_eq!(relocalization_terminal_body(&terminal).detail, "RELOCALIZATION_RETAINED_TASK_FAILED");
            assert_eq!(relocalization_terminal_candidate_sequence(&terminal), 0);
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
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut request = relocalization_command_request("req-source-seal", 7);
            let snapshot = install_relocalization_target_points_for_request(
                &phase_a, &mut request, "robot_map",
                vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            );
            let fence = phase_a.relocalization_mode();
            let epoch = fence.activate_for_tests().epoch;
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let make_frame = |sequence: u64, timestamp: u64, translation: [f64; 3]| {
                let pose = Isometry3::translation(translation[0], translation[1], translation[2]);
                let mut input = frame_from_admitted_source(sequence, admit_frame_source(
                    sequence, timestamp, timestamp, pose,
                    vec![pose * Point3::origin(); 25_000], &adapter, Arc::clone(&fence),
                ));
                input.relocalization_mode_epoch = Some(epoch);
                input
            };
            let first = make_frame(1, 10, [1.0, 0.0, 0.0]);
            let second = make_frame(2, 20, [0.0, 2.0, 0.0]);
            let first_owner = first.source_owner.clone();
            let second_owner = second.source_owner.clone();
            let accumulation = RelocalizationAccumulation {
                request: validate_relocalize_request(request).unwrap(),
                admission_ordinal: 1, mode_epoch: epoch,
                logical_deadline: Instant::now() + Duration::from_secs(1),
                accumulation_deadline: Instant::now() + Duration::from_secs(1),
                snapshot, arm_watermark: None, armed_map_watermarks: HashMap::new(),
                accepted: vec![AccumulatedRelocalizationFrame { input: Arc::new(first) },
                    AccumulatedRelocalizationFrame { input: Arc::new(second.clone()) }],
                aggregate_input_points: 50_000,
                session_map_frame_generation: Some(second.map_frame_generation),
                mapping_commit_boot_nonce: Some(second.mapping_commit_id.boot_nonce),
            };
            let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
            let lease = Arc::new(CorrectionRequestLease::new(
                CorrectionRequestCancellation::new(), Some(Arc::clone(&lifecycle).lock_owned().await),
            ));
            let sealed = seal_relocalization_source(accumulation, second, Arc::clone(&lease),
                RelocalizationCutoffReasonV1::MinimumAggregateReached).unwrap();
            assert!(sealed.boundary_lease.has_lifecycle_guard());
            assert!(lifecycle.try_lock().is_err());
            assert_eq!(sealed.frames[0].input.source_owner, first_owner);
            assert_eq!(sealed.frames[1].input.source_owner, second_owner);
            assert_eq!(sealed.source_points.len(), 50_000);
            assert_eq!(sealed.source_points[0], Point3::new(1.0, 0.0, 0.0));
            assert_eq!(sealed.source_points[25_000], Point3::new(0.0, 2.0, 0.0));
            assert_eq!(sealed.target_points.len(), 3);
            drop(sealed);
            lease.release();
            assert!(lifecycle.try_lock().is_ok());
        }

        #[tokio::test]
        async fn relocalization_source_hard_bounds_fail_closed() {
            assert_eq!(RELOCALIZATION_MAX_PER_FRAME_F64_BYTES,
                MAX_REGISTERED_CLOUD_POINTS as usize * 3 * std::mem::size_of::<f64>());
            assert_eq!(RELOCALIZATION_MAX_AGGREGATE_F64_BYTES,
                RELOCALIZATION_MAX_AGGREGATE_POINTS * 3 * std::mem::size_of::<f64>());
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let fence = Arc::new(RelocalizationModeFence::default());
            let at_frame_limit = frame_from_admitted_source(1, admit_frame_source(
                1, 10, 10, Isometry3::identity(),
                vec![Point3::origin(); MAX_REGISTERED_CLOUD_POINTS as usize], &adapter, Arc::clone(&fence),
            ));
            assert!(relocalization_frame_evidence(&owner_identity(), "robot_map", "robot_odom", "robot_body", &at_frame_limit).is_ok());
            // Real LIO admission already caps scans at100k; a hostile larger
            // frame must fail rather than inventing an above-limit capability.
            let mut above_frame_limit = at_frame_limit.clone();
            above_frame_limit.body_points = above_frame_limit.body_points.edited_for_test(|points| points.push(Point3::origin()));
            above_frame_limit.registered_cloud.point_count += 1;
            assert!(relocalization_frame_evidence(&owner_identity(), "robot_map", "robot_odom", "robot_body", &above_frame_limit).is_err());
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut request = relocalization_command_request("req-aggregate-hard-bound", 7);
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let snapshot = phase_a.activated_target_receipts().current_sealed_assigned_target_snapshot().unwrap();
            let fence = phase_a.relocalization_mode();
            let epoch = fence.activate_for_tests().epoch;
            let mut boundary = frame_from_admitted_source(3, admit_frame_source(
                3, 30, 30, Isometry3::identity(), vec![Point3::origin()], &adapter, fence,
            ));
            boundary.relocalization_mode_epoch = Some(epoch);
            let accumulation = RelocalizationAccumulation {
                request: validate_relocalize_request(request).unwrap(), admission_ordinal: 1, mode_epoch: epoch,
                logical_deadline: Instant::now() + Duration::from_secs(1),
                accumulation_deadline: Instant::now() + Duration::from_secs(1),
                snapshot, arm_watermark: None, armed_map_watermarks: HashMap::new(),
                accepted: vec![AccumulatedRelocalizationFrame { input: Arc::new(boundary.clone()) }],
                aggregate_input_points: RELOCALIZATION_MAX_AGGREGATE_POINTS + 1,
                session_map_frame_generation: Some(boundary.map_frame_generation),
                mapping_commit_boot_nonce: Some(boundary.mapping_commit_id.boot_nonce),
            };
            let lease = Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(),
                Some(Arc::new(tokio::sync::Mutex::new(())).lock_owned().await)));
            assert_eq!(seal_relocalization_source(accumulation, boundary, lease,
                RelocalizationCutoffReasonV1::MinimumAggregateReached).err().unwrap(),
                "RELOCALIZATION_AGGREGATE_BYTES_EXCEEDED");
        }

        fn assigned_finalized_terminal_fixture() -> crate::proto::RelocalizeResult {
            let fence = Arc::new(RelocalizationModeFence::default());
            let mode = fence.activate().unwrap();
            let mut input = frame_with_mode_fence(1, 1002, 0.0, fence);
            input.relocalization_mode_epoch = Some(mode.epoch);
            let mut wire = relocalization_command_request("typed-finalized", 7);
            let target = relocalization_request_body(&mut wire).target.as_mut().unwrap();
            target.activation.as_mut().unwrap().backend_generation = input.committed_map.backend_generation;
            target.occupancy_receipt.as_mut().unwrap().backend_generation = input.committed_map.backend_generation;
            target.cloud_receipt.as_mut().unwrap().point_count = 3;
            target.source_cloud_member.as_mut().unwrap().cloud_receipt.as_mut().unwrap().point_count = 3;
            target.source_cloud_member.as_mut().unwrap().frame_id = "robot_map".into();
            let request = validate_relocalize_request(wire).unwrap();
            let owner = CorrectionOwnerRuntimeIdentity { runtime_generation: 2, ..owner_identity() };
            let transform = CorrectionTransform::from_isometry(&Isometry3::identity());
            let candidate = CorrectionCandidate::seal(correction_candidate_identity(CorrectionCandidateBackend::Relocalization,
                &owner, 1, "robot_map", "robot_odom", "robot_body", &input), transform).unwrap();
            let evidence = crate::proto::AssignedRelocalizationEvidenceV2 {
                schema_version: 2, phase: crate::proto::AssignedRelocalizationEvidencePhaseV2::Finalized as i32,
                admission_ordinal: Some(1), owner: Some(crate::proto::RelocalizationOwnerReceiptV2 {
                    runtime_generation: owner.runtime_generation, correction_backend_generation: owner.backend_generation,
                    correction_lease_boot_nonce: owner.correction_lease.boot_nonce.to_be_bytes().to_vec(),
                    correction_lease_generation: owner.correction_lease.generation,
                    map_frame_id: "robot_map".into(), odom_frame_id: "robot_odom".into(), body_frame_id: "robot_body".into(),
                    relocalization_epoch: Some(mode.epoch),
                }),
                source_frames: vec![assigned_source_receipt(&input, "robot_map", "robot_odom", "robot_body").unwrap()],
                algorithm: Some(crate::proto::RelocalizationAlgorithmReceiptV2 {
                    converged: false, accepted: true, mse_bits: 0_f64.to_bits(), fitness_bits: 1_f64.to_bits(),
                    iterations: 1, feature_matches: 0, ransac_inliers: input.registered_cloud.point_count,
                    estimated_transform: Some(relocalization_pose_bits(&Isometry3::identity())),
                    proposed_correction: Some(relocalization_pose_bits(&Isometry3::identity())),
                }),
            };
            assigned_relocalization_result(&request.identity, crate::proto::RelocalizeDisposition::Applied,
                7, "applied fixture", candidate.encode_body().unwrap(), evidence)
        }

        #[test]
        fn assigned_terminal_retained_validator_checks_real_v3_candidate_receipt_and_outcome() {
            let result = assigned_finalized_terminal_fixture();
            let terminal = AssignedRelocalizationTerminalV2::new(result.clone(), Some(&validate_assigned_terminal_retained)).unwrap();
            let retained = terminal.clone();
            assert_eq!(retained.encoded().as_ptr(), terminal.encoded().as_ptr());
            let reopened = AssignedRelocalizationTerminalV2::decode_durable(&terminal.encode_durable().unwrap(),
                Some(&validate_assigned_terminal_retained)).unwrap();
            assert_eq!(reopened.encoded(), terminal.encoded());
            let changes: [fn(&mut crate::proto::AssignedCloudRelocalizeResultV2); 9] = [
                |b| b.candidate_payload_v3.push(0),
                |b| b.evidence.as_mut().unwrap().owner.as_mut().unwrap().correction_lease_generation += 1,
                |b| b.evidence.as_mut().unwrap().source_frames[0].source_sequence += 1,
                |b| b.evidence.as_mut().unwrap().source_frames[0].cloud_revision += 1,
                |b| b.evidence.as_mut().unwrap().source_frames[0].odom_from_body.as_mut().unwrap().translation_m_bits[0] = (-0_f64).to_bits(),
                |b| b.evidence.as_mut().unwrap().algorithm.as_mut().unwrap().proposed_correction.as_mut().unwrap().translation_m_bits[0] = 1_f64.to_bits(),
                |b| b.evidence.as_mut().unwrap().algorithm.as_mut().unwrap().estimated_transform.as_mut().unwrap().translation_m_bits[0] = 2_f64.to_bits(),
                |b| b.disposition = crate::proto::RelocalizeDisposition::Cancelled as i32,
                |b| b.correction_revision = 0,
            ];
            for change in changes {
                let mut changed = result.clone();
                change(changed.assigned_cloud_v2.as_mut().unwrap());
                assert!(AssignedRelocalizationTerminalV2::new(changed, Some(&validate_assigned_terminal_retained)).is_err());
            }
            // A rejection can report the current canonical revision; it did not
            // create a new revision, so retaining that observation is honest.
            let mut rejected = result;
            rejected.assigned_cloud_v2.as_mut().unwrap().disposition = crate::proto::RelocalizeDisposition::Rejected as i32;
            assert!(AssignedRelocalizationTerminalV2::new(rejected, Some(&validate_assigned_terminal_retained)).is_ok());
        }

        #[test]
        fn assigned_accepted_no_candidate_receipt_binds_algorithm_before_durable_reopen() {
            let mut valid = assigned_finalized_terminal_fixture();
            let body = valid.assigned_cloud_v2.as_mut().unwrap();
            body.disposition = crate::proto::RelocalizeDisposition::Cancelled as i32;
            body.correction_revision = 0;
            body.candidate_payload_v3.clear();
            body.evidence.as_mut().unwrap().phase = crate::proto::AssignedRelocalizationEvidencePhaseV2::AlgorithmCompleted as i32;
            let terminal = AssignedRelocalizationTerminalV2::new(valid.clone(), Some(&validate_assigned_terminal_retained)).unwrap();
            let reopened = AssignedRelocalizationTerminalV2::decode_durable(&terminal.encode_durable().unwrap(),
                Some(&validate_assigned_terminal_retained)).unwrap();
            assert_eq!(terminal.encoded(), reopened.encoded());
            for mutate_estimate in [true, false] {
                let mut changed = valid.clone();
                let algorithm = changed.assigned_cloud_v2.as_mut().unwrap().evidence.as_mut().unwrap().algorithm.as_mut().unwrap();
                let pose = if mutate_estimate { algorithm.estimated_transform.as_mut() } else { algorithm.proposed_correction.as_mut() }.unwrap();
                pose.translation_m_bits[0] = 1_f64.to_bits();
                let request_bytes = changed.assigned_cloud_v2.as_ref().unwrap().request.as_ref().unwrap().encode_to_vec();
                let terminal_bytes = changed.encode_to_vec();
                assert_eq!(AssignedRelocalizationTerminalV2::new(changed, Some(&validate_assigned_terminal_retained)).unwrap_err(),
                    "RELOCALIZATION_ASSIGNED_ALGORITHM_BINDING_MISMATCH");
                let mut durable = b"nuwax.geometry.relocalize.assigned-terminal.v2\0".to_vec();
                durable.extend_from_slice(&(request_bytes.len() as u32).to_be_bytes());
                durable.extend_from_slice(&request_bytes);
                durable.extend_from_slice(&(terminal_bytes.len() as u32).to_be_bytes());
                durable.extend_from_slice(&terminal_bytes);
                assert_eq!(AssignedRelocalizationTerminalV2::decode_durable(&durable, Some(&validate_assigned_terminal_retained)).unwrap_err(),
                    "RELOCALIZATION_ASSIGNED_ALGORITHM_BINDING_MISMATCH");
            }
        }

        #[test]
        fn relocalization_frame_proof_ledger_is_scoped_and_conflict_permanent() {
            let mut actor = RelocalizationCommandActor::new(owner_identity(), "robot_map".into(), "robot_odom".into(), "robot_body".into());
            let input = Arc::new(frame(1, 10, 0.0));
            let key = relocalization_frame_key(&input);
            assert!(!actor.record_frame_proof(&input, key).unwrap());
            assert!(actor.record_frame_proof(&Arc::new((*input).clone()), key).unwrap());
            let mut changed = (*input).clone();
            let mut occupancy = changed.committed_map_occupancy.to_vec();
            occupancy[0] ^= 1;
            changed.committed_map_occupancy = occupancy.into();
            assert_eq!(actor.record_frame_proof(&Arc::new(changed), key), Err("RELOCALIZATION_FRAME_KEY_PROOF_CONFLICT"));
            assert!(actor.record_frame_proof(&input, key).unwrap());
            let weak = Arc::downgrade(&input);
            drop(input);
            assert!(weak.upgrade().is_none(), "proof cache cannot retain rejected clouds");
            assert_eq!(actor.record_frame_proof(&Arc::new(frame(1, 10, 0.0)), key), Err("RELOCALIZATION_FRAME_KEY_PROOF_RELEASED"));
        }

        #[test]
        fn relocalization_cache_scopes_real_lio_producer_domains_without_cross_ordering() {
            let mut actor = RelocalizationCommandActor::new(owner_identity(), "robot_map".into(), "robot_odom".into(), "robot_body".into());
            let first = Arc::new(frame(1, 10, 0.0));
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let ingress = crate::lio_ingress::CommonLioIngress::new(crate::lio_ingress::DeviceLioIngressSpec::new(
                32, "robot_sensor", "robot_body", "robot_odom", "robot_map").unwrap());
            ingress.bind_relocalization_mode_fence(Arc::new(RelocalizationModeFence::default())).unwrap();
            let source = admit_frame_source_with_ingress(1, 10, 10, Isometry3::identity(),
                asymmetric_cloud(), &adapter, &ingress);
            let second = Arc::new(frame_from_admitted_source(1, source));
            first.validate_source_ownership().unwrap(); second.validate_source_ownership().unwrap();
            assert_eq!(relocalization_frame_key(&first), relocalization_frame_key(&second));
            assert_ne!(RelocalizationFrameScope::from_input(&first), RelocalizationFrameScope::from_input(&second));
            assert!(!actor.record_frame_proof(&first, relocalization_frame_key(&first)).unwrap());
            assert!(!actor.record_frame_proof(&second, relocalization_frame_key(&second)).unwrap());
            assert_eq!(actor.last_validated_frames.len(), 2);
        }

        #[test]
        fn relocalization_stable_stratified_source_has_fixed_indices() {
            let ordered = (0..100_001).map(|index| Point3::new(index as f64, 0.0, 0.0)).collect();
            let emitted = stable_stratified_source(ordered).unwrap();
            assert_eq!(emitted.len(), 100_000);
            assert_eq!(emitted[0].x, 0.0);
            assert_eq!(emitted[50_000].x, 50_000.0);
            assert_eq!(emitted[99_999].x, 99_999.0);
            assert!(stable_stratified_source(vec![Point3::origin(); RELOCALIZATION_MAX_AGGREGATE_POINTS+1]).is_err());
        }

        #[tokio::test]
        async fn relocalization_actor_restart_replays_before_conflict_busy_and_side_effects() {
            let root = RelocalizationTerminalTestRoot::new();
            let result = assigned_finalized_terminal_fixture();
            let terminal = AssignedRelocalizationTerminalV2::new(result.clone(), Some(&validate_assigned_terminal_retained)).unwrap();
            let request = ValidatedRelocalizeRequest { identity: terminal.request().clone() };
            { let store = RelocalizationTerminalStore::open_at(root.path().into()).unwrap();
              store.persist_terminal(request.request_id(), &request.key(), &StoredRelocalizationResult::Assigned(terminal)).unwrap(); }
            let path = root.path().join(assigned_terminal_filename_v2(request.request_id()).unwrap());
            let persisted = fs::read(&path).unwrap();
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut supervisor = RelocalizationOwnerSupervisor::start_inner(phase_a.clone(),
                CorrectionOwnerRuntimeIdentity { runtime_generation: 2, ..owner_identity() },
                "robot_map".into(), "robot_odom".into(), "robot_body".into(), None,
                Some(RelocalizationTerminalStore::open_at(root.path().into()).unwrap()));
            let ingress = supervisor.ingress();
            let before = ingress.terminal_snapshot_assigned(&relocalization_query(&request)).await.unwrap().ledger_observation;
            let mut conflict_wire = request.identity.to_wire();
            relocalization_request_body(&mut conflict_wire).target.as_mut().unwrap().activation.as_mut().unwrap().lifecycle_epoch += 1;
            let conflict = ingress.admit(conflict_wire.clone()).await.unwrap().wait().await.unwrap();
            assert_eq!(relocalization_terminal_body(&conflict).disposition, crate::proto::RelocalizeDisposition::ReplayConflict as i32);
            assert_eq!(ingress.admit(conflict_wire).await.unwrap().wait().await.unwrap().encode_to_vec(), conflict.encode_to_vec());
            for _ in 0..3 {
                assert_eq!(ingress.admit(request.identity.to_wire()).await.unwrap().wait().await.unwrap().encode_to_vec(), result.encode_to_vec());
            }
            let after = ingress.terminal_snapshot_assigned(&relocalization_query(&request)).await.unwrap();
            assert!(matches!(after.terminal, RelocalizationTerminalSnapshot::AssignedTerminal(_)));
            assert_eq!(before.encode_to_vec(), after.ledger_observation.encode_to_vec());
            assert_eq!(after.ledger_observation.admitted_command_count, 0);
            assert_eq!(after.ledger_observation.claimed_command_count, 0);
            assert_eq!(phase_a.relocalization_mode().snapshot(), RelocalizationModeSnapshot { epoch: 0, active: false });
            assert_eq!(fs::read(path).unwrap(), persisted);
            phase_a.begin(); supervisor.shutdown().await.unwrap();
        }

        #[tokio::test]
        async fn relocalization_legacy_mixed_live_versions_fail_before_waiter_or_source_admission() {
            let phase_a = CorrectionPhaseAControl::for_tests();
            let mut supervisor = RelocalizationOwnerSupervisor::start(phase_a.clone(),
                CorrectionOwnerRuntimeIdentity { runtime_generation: 2, ..owner_identity() },
                "robot_map".into(), "robot_odom".into(), "robot_body".into());
            let ingress = supervisor.ingress();
            let mut mixed = relocalization_command_request("mixed", 7);
            mixed.activation_identity_sha256 = vec![1;32];
            for request in [crate::proto::RelocalizeRequest { schema_version: 1, request_id: "legacy".into(), ..Default::default() },
                crate::proto::RelocalizeRequest { schema_version: 3, request_id: "unsupported".into(), ..Default::default() }, mixed] {
                assert!(matches!(ingress.admit(request).await, Err(RelocalizationCommandError::InvalidRequest(_))));
            }
            assert_eq!(ingress.waiter_capacity.available_permits(), RELOCALIZATION_MAX_WAITERS);
            assert!(ingress.terminal_store.state.lock().terminals.is_empty());
            assert!(!phase_a.relocalization_mode().snapshot().active);
            let called = Arc::new(AtomicBool::new(false));
            let marker = called.clone();
            let work: PlanarCommandWork = Box::new(move |_| { marker.store(true, Ordering::Release); Box::pin(async {
                PlanarCommandOutcome { disposition: crate::proto::RelocalizeDisposition::Rejected, detail: "must not run".into(), correction_revision: 0, evidence: Vec::new() }
            }) });
            assert!(matches!(ingress.admit_planar(relocalization_command_request("planar-gated", 7),
                Arc::new(tokio::sync::Mutex::new(())).lock_owned().await, work).await, Err(RelocalizationCommandError::InvalidRequest(_))));
            assert!(!called.load(Ordering::Acquire));
            phase_a.begin(); supervisor.shutdown().await.unwrap();
        }


        #[test]
        fn relocalization_disclosure_retention_bounds_are_fixed() {
            assert_eq!(RELOCALIZATION_MAX_INLINE_EVIDENCE_DISCLOSURE_BYTES, 4_158);
            assert_eq!(RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_TERMINAL, 24_832);
            assert_eq!(RELOCALIZATION_MAX_DISCLOSURE_BYTES_PER_COMMAND_RECORD, 295_102);
            assert_eq!(RELOCALIZATION_MAX_RETAINED_DISCLOSURE_BYTES, 604_368_896);
            assert_eq!(RELOCALIZATION_MAX_COMMAND_RECORDS, 1024);
            assert_eq!(RELOCALIZATION_MAX_WAITERS, 16);
            assert_eq!(RELOCALIZATION_MAX_CONFLICTS_PER_RECORD, 64);
            assert_eq!(RELOCALIZATION_MAX_FRAMES, 8);
        }

        #[test]
        fn relocalization_policy_parameters_preserve_current_exact_bytes_without_hashes() {
            let source = relocalization_source_policy_preimage_v1();
            let domain = b"nuwax.geometry.relocalize.source-policy.v1\0";
            assert_eq!(&source[..domain.len()], domain);
            assert_eq!(&source[domain.len()..domain.len()+4], &3_u32.to_be_bytes());
            assert_eq!(&source[domain.len()+16..domain.len()+24], &0_u64.to_be_bytes());
            let expected = [0_u64, 100_000, 2_400_000, 4_096, 25_600_000, 250_000, 6_000_000,
                250_000, 6_000_000, 0, 100_000, 0, 2_400_000, 2_000];
            for (index, value) in expected.into_iter().enumerate() {
                let offset = domain.len() + 16 + index * 8;
                assert_eq!(&source[offset..offset+8], &value.to_be_bytes());
            }
            let deadline = relocalization_deadline_policy_preimage_v1();
            assert_eq!(exact_hex(&deadline), "6e757761782e67656f6d657472792e72656c6f63616c697a652e646561646c696e652d706f6c6963792e7631000000000100000000000000fa0000000000007530000000000000753000000000000007d00000000000001388000000010000000100000001");
            let algorithm = relocalization_algorithm_options_preimage_v1();
            let algorithm_domain = b"nuwax.geometry.relocalize.algorithm-options.v1\0";
            assert_eq!(&algorithm[algorithm_domain.len()..algorithm_domain.len()+4], &2_u32.to_be_bytes());
            assert_eq!(&algorithm[algorithm.len()-8..], &0.8_f64.to_be_bytes());
            let policy = crate::relocalization_identity::CanonicalRelocalizationPolicyV2::current_cloud().to_wire();
            assert_eq!(policy.source_parameters, source);
            assert_eq!(policy.deadline_parameters, deadline);
            assert_eq!(policy.algorithm_parameters, algorithm);
        }

        #[test]
        fn relocalization_disclosure_replay_is_byte_identical() {
            let terminal = AssignedRelocalizationTerminalV2::new(assigned_finalized_terminal_fixture(),
                Some(&validate_assigned_terminal_retained)).unwrap();
            let encoded = terminal.encoded().as_ptr();
            let retries = (0..64).map(|_| terminal.clone()).collect::<Vec<_>>();
            assert!(retries.iter().all(|value| value.encoded().as_ptr() == encoded));
            assert!(retries.iter().all(|value| value.to_wire().encode_to_vec() == terminal.encoded()));
            let reopened = AssignedRelocalizationTerminalV2::decode_durable(&terminal.encode_durable().unwrap(),
                Some(&validate_assigned_terminal_retained)).unwrap();
            assert_eq!(reopened.encoded(), terminal.encoded());
        }

        #[test]
        fn relocalization_typed_source_elided_evidence_retains_existing_hard_bounds() {
            let original = assigned_finalized_terminal_fixture();
            let mut oversized = original.clone();
            oversized.assigned_cloud_v2.as_mut().unwrap().candidate_payload_v3 = vec![0; RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES+1];
            assert!(AssignedRelocalizationTerminalV2::new(oversized, Some(&validate_assigned_terminal_retained)).unwrap_err().contains("BOUND_EXCEEDED"));
            let mut oversized = original.clone();
            let evidence = oversized.assigned_cloud_v2.as_mut().unwrap().evidence.as_mut().unwrap();
            evidence.source_frames = vec![evidence.source_frames[0].clone(); RELOCALIZATION_MAX_FRAMES+1];
            assert!(AssignedRelocalizationTerminalV2::new(oversized, Some(&validate_assigned_terminal_retained)).unwrap_err().contains("BOUND_EXCEEDED"));
            let mut oversized = original.clone();
            oversized.assigned_cloud_v2.as_mut().unwrap().detail = "x".repeat(RELOCALIZATION_MAX_TERMINAL_BYTES);
            assert!(AssignedRelocalizationTerminalV2::new(oversized, Some(&validate_assigned_terminal_retained)).unwrap_err().contains("BOUND_EXCEEDED"));
            let mut changed = original;
            changed.assigned_cloud_v2.as_mut().unwrap().evidence.as_mut().unwrap().source_frames[0].admitted_point_count = RELOCALIZATION_MAX_AGGREGATE_POINTS as u64 + 1;
            assert!(AssignedRelocalizationTerminalV2::new(changed, Some(&validate_assigned_terminal_retained)).is_err());
        }


        fn activation_seed(request_id: &str, identity_byte: u8) -> ActivationReceiptSeed {
            ActivationReceiptSeed {
                checkpoint_owner: None,
                planar_target: None,
                activation_request_id: request_id.to_owned(),
                request_key: ActivationRequestKey::LegacyDigest([identity_byte; 32]),
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
                target_receipt: TargetReceipt::LegacyDigest(target_receipt_sha256),
            }
        }

        fn assigned_base_state() -> crate::proto::MappingSessionState {
            crate::proto::MappingSessionState {
                session_id: "destination".into(),
                map_name: "Map One".into(),
                session_generation: 23,
                ..activation_base_state()
            }
        }

        fn assigned_wire_request() -> crate::proto::ActivateMapRequest {
            let commit = crate::proto::MappingCommitIdentity {
                boot_nonce: vec![1; 16],
                sequence: 5,
            };
            let occupancy = crate::proto::CanonicalOccupancyCommitReceipt {
                commit_id: Some(commit.clone()),
                session_id: "source".into(),
                session_generation: 2,
                occupancy_revision: 7,
                source_timestamp_ns: 100,
                backend_kind: "voxel".into(),
                backend_generation: 3,
            };
            let cloud = crate::proto::CanonicalCloudCommitReceipt {
                commit_id: Some(commit),
                session_generation: 2,
                occupancy_revision: 7,
                cloud_revision: 9,
                source_timestamp_ns: 100,
                point_count: 1,
            };
            let member =
                |role: crate::proto::CheckpointMemberRole| crate::proto::CheckpointMemberIdentity {
                    checkpoint_id: "11111111111111111111111111111111".into(),
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
            let checkpoint = crate::proto::MappingBackendCheckpoint {
                schema_version: 2,
                backend_kind: "voxel".into(),
                source_session_generation: 2,
                payload_size: payload.len() as u64,
                payload,
                created_at_ns: 101,
                source_session_id: "source".into(),
                checkpoint_id: "11111111111111111111111111111111".into(),
                backend_payload_schema_version: 1,
                backend_identity: Some(member(crate::proto::CheckpointMemberRole::BackendState)),
                occupancy_identity: Some(member(
                    crate::proto::CheckpointMemberRole::CanonicalOccupancy,
                )),
                canonical_occupancy_checkpoint: Some(crate::proto::OccupancyGridData {
                    meta: Some(crate::proto::OccupancyGridMeta {
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
                canonical_cloud_checkpoint: Some(crate::proto::CloudMapCheckpoint {
                    frame_id: "map".into(),
                    source_revision: 9,
                    point_count: 1,
                    voxel_size: 0.05,
                    xyz_f32_le: [1_f32, 2.0, 3.0]
                        .into_iter()
                        .flat_map(f32::to_le_bytes)
                        .collect(),
                    payload_schema_version: 2,
                    member_identity: Some(member(
                        crate::proto::CheckpointMemberRole::CanonicalCloud,
                    )),
                    ..Default::default()
                }),
                occupancy_receipt: Some(occupancy),
                cloud_receipt: Some(cloud),
                ..Default::default()
            };
            crate::proto::ActivateMapRequest {
                session_id: "destination".into(),
                map_name: "Map One".into(),
                activation_request_id: "activate-1".into(),
                expected_runtime_generation: 17,
                activation_contract_version: 2,
                asset_identity: Some(crate::proto::AssetIdentityV2 {
                    asset_id: "22222222222222222222222222222222".into(),
                }),
                mapping_backend_checkpoint: Some(checkpoint),
                ..Default::default()
            }
        }

        fn assigned_request(
            wire: crate::proto::ActivateMapRequest,
        ) -> ValidatedActivationRequestV2 {
            ValidatedActivationRequestV2::admit(
                wire,
                17,
                crate::mapping_backend::MappingBackendKind::Voxel,
                "map",
                |_| panic!("fixture has no artifacts"),
                &crate::activation_request::CheckpointOwnerInternerV2::default(),
            )
            .expect("valid assigned request")
        }

        fn assigned_token(
            control: &ActivatedTargetReceiptControl,
            request: ValidatedActivationRequestV2,
        ) -> ActivationToken {
            match control.stage_assigned_or_replay(request, assigned_base_state(), true) {
                ActivationLedgerAdmission::Execute(token) => token,
                ActivationLedgerAdmission::Terminal(state) => {
                    panic!("expected execute: {}", state.message)
                }
            }
        }

        fn assigned_proof(
            control: &ActivatedTargetReceiptControl,
            token: &ActivationToken,
        ) -> ActivationPublicationProof {
            let activation = control
                .activation_identity(token)
                .unwrap()
                .assigned()
                .unwrap()
                .clone();
            let owner = control.checkpoint_owner(token).unwrap();
            let occupancy_receipt = crate::mapping_commit::OccupancyCommitReceipt {
                commit_id: MappingCommitId {
                    boot_nonce: 31,
                    sequence: 32,
                },
                session_id: activation.request.session_id().into(),
                session_generation: activation.session_generation,
                occupancy_revision: 33,
                source_timestamp_ns: 34,
                backend_kind: activation.backend_kind,
                backend_generation: activation.backend_generation,
            };
            let cloud_receipt = crate::mapping_commit::CloudCommitReceipt {
                commit_id: occupancy_receipt.commit_id,
                session_generation: activation.session_generation,
                occupancy_revision: occupancy_receipt.occupancy_revision,
                cloud_revision: 35,
                source_timestamp_ns: occupancy_receipt.source_timestamp_ns,
                point_count: owner.cloud().point_count(),
            };
            let target_receipt = InstalledTargetReceiptV2 {
                activation,
                source_cloud_member: owner.cloud().member().clone(),
                occupancy_receipt,
                cloud_receipt: cloud_receipt.clone(),
            };
            ActivationPublicationProof {
                source_timestamp_ns: cloud_receipt.source_timestamp_ns,
                cloud_revision: cloud_receipt.cloud_revision,
                point_count: cloud_receipt.point_count,
                voxel_size: owner.cloud().voxel_size(),
                cloud_receipt,
                target_receipt: TargetReceipt::Assigned(target_receipt),
            }
        }

        fn assigned_bound_token(
            control: &ActivatedTargetReceiptControl,
            request: ValidatedActivationRequestV2,
        ) -> ActivationToken {
            let token = assigned_token(control, request);
            let token = control.reserve_backend_generation(&token, 19).unwrap();
            control
                .bind_assigned_activation_identity(&token, 23)
                .unwrap()
        }

        fn assert_assigned_wire_has_no_legacy_digests(state: &crate::proto::MappingSessionState) {
            assert_eq!(state.activation_receipt_schema_version, 2);
            assert!(state.activation_receipt_v2.is_some());
            assert!(state.activation_request_identity_sha256.is_empty());
            assert!(state.activation_identity_sha256.is_empty());
            assert!(state.target_receipt_sha256.is_empty());
            assert!(state.asset_manifest_sha256.is_empty());
            assert!(state.asset_bundle_sha256.is_empty());
            assert!(state.source_policy_sha256.is_empty());
            assert!(state.deadline_policy_sha256.is_empty());
            assert!(state.algorithm_options_sha256.is_empty());
            assert!(state.target_graph_sha256.is_empty());
            assert!(!state.relocalization_target_available);
        }

        #[test]
        fn assigned_ledger_staged_replay_compares_exact_sources_and_reuses_current_owner() {
            let control = ActivatedTargetReceiptControl::new(17);
            let original = assigned_request(assigned_wire_request());
            let owner = original.checkpoint().clone();
            let token = assigned_token(&control, original.clone());
            let replay = assigned_request(assigned_wire_request());
            assert!(!owner.shares_source_with(replay.checkpoint()));
            assert_eq!(assigned_token(&control, replay), token);
            assert!(owner.shares_source_with(&control.checkpoint_owner(&token).unwrap()));
            let source = control.target_source(&token).unwrap();
            assert_eq!(
                source.xyz_f32_le().as_ptr(),
                owner.cloud().xyz_f32_le().as_ptr()
            );
            assert!(source.legacy_digest().is_none());
            assert!(control.immutable_target(&token).is_none());
            for member in ["cloud", "occupancy", "backend"] {
                let mut changed = assigned_wire_request();
                let checkpoint = changed.mapping_backend_checkpoint.as_mut().unwrap();
                match member {
                    "cloud" => checkpoint
                        .canonical_cloud_checkpoint
                        .as_mut()
                        .unwrap()
                        .xyz_f32_le[..4]
                        .copy_from_slice(&4_f32.to_le_bytes()),
                    "occupancy" => {
                        checkpoint
                            .canonical_occupancy_checkpoint
                            .as_mut()
                            .unwrap()
                            .data[0] = 1
                    }
                    _ => *checkpoint.payload.last_mut().unwrap() = 1,
                }
                let result = control.stage_assigned_or_replay(
                    assigned_request(changed),
                    assigned_base_state(),
                    false,
                );
                let ActivationLedgerAdmission::Terminal(state) = result else {
                    panic!("altered source executed");
                };
                assert_eq!(
                    state.activation_receipt_disposition,
                    crate::proto::ActivationReceiptDisposition::Conflict as i32
                );
                assert_assigned_wire_has_no_legacy_digests(&state);
            }
            assert!(control.validate_token(&token));
        }

        #[test]
        fn assigned_ledger_sealed_replay_retains_one_owner_and_rejects_altered_source() {
            let control = ActivatedTargetReceiptControl::new(17);
            let request = assigned_request(assigned_wire_request());
            let weak = request.checkpoint().downgrade();
            let source_bytes = request.checkpoint().cloud().xyz_f32_le().as_ptr();
            let token = assigned_bound_token(&control, request);
            let proof = assigned_proof(&control, &token);
            let receipt = proof.target_receipt.clone();
            assert!(control.current_sealed_assigned_target_snapshot().is_none());
            assert!(control.store_publication_proof(&token, proof));
            let sealed = control
                .seal(&token, Some(receipt), assigned_base_state())
                .unwrap();
            assert!(sealed.success);
            assert_assigned_wire_has_no_legacy_digests(&sealed);
            assert!(sealed.activation_receipt_v2.unwrap().target.is_some());
            assert!(
                weak.upgrade().is_some(),
                "sealed assigned activation retains its canonical owner"
            );
            assert!(control.current_sealed_target_snapshot().is_none());
            let snapshot = control.current_sealed_assigned_target_snapshot().unwrap();
            assert_eq!(
                snapshot.checkpoint_owner.cloud().xyz_f32_le().as_ptr(),
                source_bytes
            );
            assert_eq!(
                snapshot.activation_identity,
                snapshot.target_receipt.activation
            );
            assert_eq!(snapshot.activation_identity.runtime_generation, 17);
            assert_eq!(
                snapshot.activation_identity.lifecycle_epoch,
                control.lifecycle_epoch()
            );
            assert_eq!(
                snapshot.publication_proof.target_receipt.assigned(),
                Some(&snapshot.target_receipt)
            );
            let snapshot_copy = snapshot.clone();
            assert!(snapshot
                .checkpoint_owner
                .shares_source_with(&snapshot_copy.checkpoint_owner));
            {
                let ledger = control.inner.ledger.lock();
                let record = ledger.records.get("activate-1").unwrap();
                assert!(record.seed.checkpoint_owner.is_none());
                assert!(record.immutable_target.is_none());
                assert!(record
                    .checkpoint_owner
                    .as_ref()
                    .unwrap()
                    .shares_source_with(&snapshot.checkpoint_owner));
                assert_eq!(
                    ledger
                        .records
                        .values()
                        .filter(|record| record.checkpoint_owner.is_some())
                        .count(),
                    1
                );
            }
            let candidate = assigned_request(assigned_wire_request());
            let candidate_weak = candidate.checkpoint().downgrade();
            assert!(!candidate
                .checkpoint()
                .shares_source_with(&snapshot.checkpoint_owner));
            let ActivationLedgerAdmission::Terminal(replay) =
                control.stage_assigned_or_replay(candidate, assigned_base_state(), false)
            else {
                panic!("sealed record executed");
            };
            assert_eq!(
                replay.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::SealedReplay as i32
            );
            assert!(
                candidate_weak.upgrade().is_none(),
                "replay must release the independent input owner"
            );
            assert_assigned_wire_has_no_legacy_digests(&replay);
            assert!(control
                .current_sealed_assigned_target_snapshot()
                .unwrap()
                .checkpoint_owner
                .shares_source_with(&snapshot.checkpoint_owner));
            let mut changed = assigned_wire_request();
            changed
                .mapping_backend_checkpoint
                .as_mut()
                .unwrap()
                .canonical_cloud_checkpoint
                .as_mut()
                .unwrap()
                .xyz_f32_le[..4]
                .copy_from_slice(&5_f32.to_le_bytes());
            let ActivationLedgerAdmission::Terminal(conflict) = control.stage_assigned_or_replay(
                assigned_request(changed),
                assigned_base_state(),
                false,
            ) else {
                panic!("altered sealed source executed");
            };
            assert_eq!(
                conflict.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Conflict as i32
            );
            assert!(control
                .current_sealed_assigned_target_snapshot()
                .unwrap()
                .checkpoint_owner
                .shares_source_with(&snapshot.checkpoint_owner));
            control.retire_for_mapping_change().unwrap();
            assert!(control.current_sealed_assigned_target_snapshot().is_none());
            assert!(
                weak.upgrade().is_some(),
                "retired source remains owned by live snapshots"
            );
            assert_eq!(
                snapshot.checkpoint_owner.cloud().xyz_f32_le().as_ptr(),
                source_bytes
            );
            drop(snapshot);
            assert!(
                weak.upgrade().is_some(),
                "cloned snapshot retains the same owner"
            );
            drop(snapshot_copy);
            assert!(
                weak.upgrade().is_none(),
                "permanent receipt metadata must not pin source bytes"
            );
        }

        #[test]
        fn assigned_target_snapshot_rechecks_owner_descriptors_and_lifecycle_without_ledger_lock() {
            for change in [
                "request",
                "activation",
                "receipt",
                "proof",
                "owner",
                "terminal",
                "retire",
                "phase-a",
                "replacement",
            ] {
                let control = ActivatedTargetReceiptControl::new(17);
                assert!(control.current_sealed_assigned_target_snapshot().is_none());
                let request = assigned_request(assigned_wire_request());
                let weak = request.checkpoint().downgrade();
                let token = assigned_bound_token(&control, request);
                let proof = assigned_proof(&control, &token);
                let receipt = proof.target_receipt.clone();
                assert!(control.store_publication_proof(&token, proof));
                assert!(control.current_sealed_assigned_target_snapshot().is_none());
                control
                    .seal(&token, Some(receipt), assigned_base_state())
                    .unwrap();
                assert!(control.current_sealed_assigned_target_snapshot().is_some());
                let entered = Arc::new(std::sync::Barrier::new(2));
                let release = Arc::new(std::sync::Barrier::new(2));
                *control.inner.snapshot_before_recheck_gate.lock() = Some(ActivationSealTestGate {
                    entered: entered.clone(),
                    release: release.clone(),
                });
                let worker = control.clone();
                let join =
                    std::thread::spawn(move || worker.current_sealed_assigned_target_snapshot());
                entered.wait();
                assert!(
                    control.inner.ledger.try_lock().is_some(),
                    "snapshot capture must release the ledger before recheck"
                );
                match change {
                    "retire" => {
                        control.retire_for_mapping_change().unwrap();
                    }
                    "phase-a" => {
                        assert!(control.retire_for_phase_a_first_closer(|| true));
                    }
                    "replacement" => {
                        let mut replacement = assigned_wire_request();
                        replacement.activation_request_id = "activate-2".into();
                        assigned_token(&control, assigned_request(replacement));
                    }
                    _ => {
                        // Independent admission creates an equal source with a distinct owner.
                        // Swapping it must fail snapshot freshness even if every descriptor matches.
                        let independent = assigned_request(assigned_wire_request());
                        let mut ledger = control.inner.ledger.lock();
                        let record = ledger.records.get_mut("activate-1").unwrap();
                        match change {
                            "request" => record.seed.activation_request_id = "changed".into(),
                            "activation" => {
                                let Some(ActivationIdentity::Assigned(identity)) =
                                    record.activation_identity.as_mut()
                                else {
                                    unreachable!()
                                };
                                identity.backend_generation += 1;
                            }
                            "receipt" => {
                                let Some(TargetReceipt::Assigned(receipt)) =
                                    record.target_receipt.as_mut()
                                else {
                                    unreachable!()
                                };
                                receipt.cloud_receipt.cloud_revision += 1;
                            }
                            "proof" => {
                                record.publication_proof.as_mut().unwrap().cloud_revision += 1
                            }
                            "owner" => {
                                assert!(!record
                                    .checkpoint_owner
                                    .as_ref()
                                    .unwrap()
                                    .shares_source_with(independent.checkpoint()));
                                record.checkpoint_owner = Some(independent.checkpoint().clone());
                                assert!(ActivatedTargetReceiptControl::has_complete_target_proof(
                                    record
                                ));
                            }
                            "terminal" => {
                                record.terminal.as_mut().unwrap().map_name = "changed".into()
                            }
                            _ => unreachable!(),
                        }
                    }
                }
                assert!(
                    weak.upgrade().is_some(),
                    "capture must retain its source until recheck after {change}"
                );
                release.wait();
                assert!(
                    join.join().unwrap().is_none(),
                    "snapshot escaped changed {change}"
                );
                if matches!(change, "retire" | "phase-a" | "replacement" | "owner") {
                    assert!(
                        weak.upgrade().is_none(),
                        "failed snapshot pinned the old source after {change}"
                    );
                }
                control.retire_for_mapping_change().unwrap();
                assert!(
                    weak.upgrade().is_none(),
                    "retired snapshot source leaked after {change}"
                );
            }
        }

        #[test]
        fn legacy_target_snapshot_rechecks_owner_descriptors_and_lifecycle_without_ledger_lock() {
            for change in [
                "request",
                "proof",
                "owner",
                "terminal",
                "retire",
                "phase-a",
                "replacement",
            ] {
                let control = ActivatedTargetReceiptControl::new(7);
                let token = match control.stage_or_replay(
                    activation_seed_with_target("snapshot-race", 61, 2),
                    activation_base_state(),
                    true,
                ) {
                    ActivationLedgerAdmission::Execute(token) => token,
                    ActivationLedgerAdmission::Terminal(_) => panic!("target must stage"),
                };
                let token = control.reserve_backend_generation(&token, 43).unwrap();
                let token = control
                    .bind_activation_identity(&token, [62; 32], 47)
                    .unwrap();
                assert!(control.store_publication_proof(
                    &token,
                    activation_publication_proof([63; 32], 2, 47)
                ));
                control
                    .seal(
                        &token,
                        Some(TargetReceipt::LegacyDigest([63; 32])),
                        activation_base_state(),
                    )
                    .unwrap();
                let weak = Arc::downgrade(
                    &control
                        .current_sealed_target_snapshot()
                        .unwrap()
                        .immutable_target,
                );
                let entered = Arc::new(std::sync::Barrier::new(2));
                let release = Arc::new(std::sync::Barrier::new(2));
                *control.inner.snapshot_before_recheck_gate.lock() = Some(ActivationSealTestGate {
                    entered: entered.clone(),
                    release: release.clone(),
                });
                let worker = control.clone();
                let join = std::thread::spawn(move || worker.current_sealed_target_snapshot());
                entered.wait();
                assert!(control.inner.ledger.try_lock().is_some());
                match change {
                    "retire" => {
                        control.retire_for_mapping_change().unwrap();
                    }
                    "phase-a" => {
                        assert!(control.retire_for_phase_a_first_closer(|| true));
                    }
                    "replacement" => {
                        assert!(matches!(
                            control.stage_or_replay(
                                activation_seed_with_target("snapshot-replacement", 71, 3),
                                activation_base_state(),
                                true,
                            ),
                            ActivationLedgerAdmission::Execute(_)
                        ));
                    }
                    _ => {
                        let mut ledger = control.inner.ledger.lock();
                        let record = ledger.records.get_mut("snapshot-race").unwrap();
                        match change {
                            "request" => {
                                record.seed.request_key =
                                    ActivationRequestKey::LegacyDigest([99; 32])
                            }
                            "proof" => {
                                let proof = record.publication_proof.as_mut().unwrap();
                                proof.source_timestamp_ns += 1;
                                proof.cloud_receipt.source_timestamp_ns = proof.source_timestamp_ns;
                            }
                            "owner" => {
                                record.immutable_target = Some(Arc::new(
                                    record.immutable_target.as_deref().unwrap().clone(),
                                ))
                            }
                            "terminal" => {
                                record.terminal.as_mut().unwrap().session_id = "changed".into()
                            }
                            _ => unreachable!(),
                        }
                        assert!(ActivatedTargetReceiptControl::has_complete_target_proof(
                            record
                        ));
                    }
                }
                assert!(weak.upgrade().is_some());
                release.wait();
                assert!(
                    join.join().unwrap().is_none(),
                    "legacy snapshot escaped changed {change}"
                );
                if matches!(change, "retire" | "phase-a" | "replacement" | "owner") {
                    assert!(weak.upgrade().is_none());
                }
                control.retire_for_mapping_change().unwrap();
                assert!(weak.upgrade().is_none());
            }
        }

        #[test]
        fn assigned_ledger_rejects_small_key_and_cross_version_reuse() {
            let control = ActivatedTargetReceiptControl::new(17);
            let original = assigned_request(assigned_wire_request());
            assigned_token(&control, original.clone());
            for field in ["session", "map", "asset"] {
                let mut changed = assigned_wire_request();
                match field {
                    "session" => changed.session_id = "different-session".into(),
                    "map" => changed.map_name = "Another Map".into(),
                    _ => {
                        changed.asset_identity.as_mut().unwrap().asset_id =
                            "33333333333333333333333333333333".into()
                    }
                }
                let request = assigned_request(changed);
                let state = control
                    .assigned_terminal_before_admission(request.metadata(), assigned_base_state())
                    .unwrap();
                assert_eq!(
                    state.activation_receipt_disposition,
                    crate::proto::ActivationReceiptDisposition::Conflict as i32
                );
                assert_assigned_wire_has_no_legacy_digests(&state);
            }
            let mut legacy = activation_seed("activate-1", 1);
            legacy.modern = false;
            assert!(matches!(
                control.stage_or_replay(legacy, assigned_base_state(), true),
                ActivationLedgerAdmission::Terminal(_)
            ));
            let legacy_control = ActivatedTargetReceiptControl::new(17);
            let legacy = activation_seed("activate-1", 1);
            assert!(matches!(
                legacy_control.stage_or_replay(legacy, assigned_base_state(), true),
                ActivationLedgerAdmission::Execute(_)
            ));
            let state = legacy_control
                .assigned_terminal_before_admission(original.metadata(), assigned_base_state())
                .unwrap();
            assert_assigned_wire_has_no_legacy_digests(&state);
            assert_eq!(
                state.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Conflict as i32
            );
        }

        #[test]
        fn assigned_ledger_rejects_cross_schema_proof_and_installed_receipt_mismatch() {
            let control = ActivatedTargetReceiptControl::new(17);
            let token = assigned_bound_token(&control, assigned_request(assigned_wire_request()));
            assert!(!control
                .store_publication_proof(&token, activation_publication_proof([5; 32], 1, 23)));
            assert!(control
                .bind_activation_identity(&token, [6; 32], 23)
                .is_none());
            let good = assigned_proof(&control, &token);
            for field in ["commit", "occupancy", "backend", "source"] {
                let mut bad = good.clone();
                let TargetReceipt::Assigned(receipt) = &mut bad.target_receipt else {
                    unreachable!()
                };
                match field {
                    "commit" => receipt.cloud_receipt.commit_id.sequence += 1,
                    "occupancy" => receipt.cloud_receipt.occupancy_revision += 1,
                    "backend" => {
                        receipt.activation.backend_kind =
                            crate::mapping_backend::MappingBackendKind::Raytracing
                    }
                    _ => {
                        receipt.activation.request = assigned_request({
                            let mut request = assigned_wire_request();
                            request.map_name = "Other Map".into();
                            request
                        })
                        .metadata()
                        .clone()
                    }
                }
                assert!(!control.store_publication_proof(&token, bad));
            }
            assert!(control.store_publication_proof(&token, good.clone()));
            assert!(control
                .seal(
                    &token,
                    Some(TargetReceipt::LegacyDigest([7; 32])),
                    assigned_base_state()
                )
                .is_none());
            assert!(control
                .seal(&token, Some(good.target_receipt), assigned_base_state())
                .is_some());
        }

        #[test]
        fn assigned_ledger_terminal_paths_release_owner_and_never_reexecute() {
            for path in [
                "precommit",
                "postcommit",
                "unknown",
                "mapping-change",
                "replace",
            ] {
                let control = ActivatedTargetReceiptControl::new(17);
                let request = assigned_request(assigned_wire_request());
                let metadata = request.metadata().clone();
                let weak = request.checkpoint().downgrade();
                let token = assigned_token(&control, request);
                match path {
                    "mapping-change" => {
                        control.retire_for_mapping_change().unwrap();
                    }
                    "replace" => {
                        let mut replacement = assigned_wire_request();
                        replacement.activation_request_id = "activate-2".into();
                        assigned_token(&control, assigned_request(replacement));
                    }
                    _ => {
                        let disposition = match path {
                            "precommit" => {
                                crate::proto::ActivationReceiptDisposition::InvalidatedPrecommit
                            }
                            "postcommit" => {
                                crate::proto::ActivationReceiptDisposition::InvalidatedPostcommit
                            }
                            _ => crate::proto::ActivationReceiptDisposition::OutcomeUnknown,
                        };
                        let state = control.invalidate(
                            &token,
                            disposition,
                            "terminal",
                            assigned_base_state(),
                        );
                        assert_assigned_wire_has_no_legacy_digests(&state);
                    }
                }
                assert!(weak.upgrade().is_none(), "source pinned after {path}");
                assert!(!control.validate_token(&token));
                assert!(control.checkpoint_owner(&token).is_none());
                let retired = control
                    .assigned_terminal_before_admission(&metadata, assigned_base_state())
                    .unwrap();
                assert_eq!(
                    retired.activation_receipt_disposition,
                    crate::proto::ActivationReceiptDisposition::Retired as i32
                );
                assert_assigned_wire_has_no_legacy_digests(&retired);
                let ActivationLedgerAdmission::Terminal(_) = control.stage_assigned_or_replay(
                    assigned_request(assigned_wire_request()),
                    assigned_base_state(),
                    true,
                ) else {
                    panic!("retired tombstone executed after {path}");
                };
            }
        }

        #[test]
        fn assigned_ledger_phase_a_seal_barrier_releases_owner_and_rejects_late_mutation() {
            let control = ActivatedTargetReceiptControl::new(17);
            let request = assigned_request(assigned_wire_request());
            let metadata = request.metadata().clone();
            let weak = request.checkpoint().downgrade();
            let token = assigned_bound_token(&control, request);
            let proof = assigned_proof(&control, &token);
            let receipt = proof.target_receipt.clone();
            assert!(control.store_publication_proof(&token, proof));
            let entered = Arc::new(std::sync::Barrier::new(2));
            let release = Arc::new(std::sync::Barrier::new(2));
            control.install_seal_before_ledger_gate(entered.clone(), release.clone());
            let worker = control.clone();
            let worker_token = token.clone();
            let join = std::thread::spawn(move || {
                worker.seal(&worker_token, Some(receipt), assigned_base_state())
            });
            entered.wait();
            let epoch = control.lifecycle_epoch();
            assert!(control.retire_for_phase_a_first_closer(|| true));
            assert_eq!(control.lifecycle_epoch(), epoch + 1);
            assert!(weak.upgrade().is_none());
            assert!(!control.validate_token(&token));
            assert!(control
                .bind_assigned_activation_identity(&token, 23)
                .is_none());
            release.wait();
            assert!(join.join().unwrap().is_none());
            let state = control
                .assigned_terminal_before_admission(&metadata, assigned_base_state())
                .unwrap();
            assert_assigned_wire_has_no_legacy_digests(&state);
        }

        #[test]
        fn assigned_replay_comparison_does_not_hold_ledger_or_revive_retired_owner() {
            let control = ActivatedTargetReceiptControl::new(17);
            let request = assigned_request(assigned_wire_request());
            let weak = request.checkpoint().downgrade();
            assigned_token(&control, request);
            let candidate = assigned_request(assigned_wire_request());
            let entered = Arc::new(std::sync::Barrier::new(2));
            let release = Arc::new(std::sync::Barrier::new(2));
            *control.inner.replay_before_compare_gate.lock() = Some(ActivationSealTestGate {
                entered: entered.clone(),
                release: release.clone(),
            });
            let worker = control.clone();
            let join = std::thread::spawn(move || {
                worker.stage_assigned_or_replay(candidate, assigned_base_state(), true)
            });
            entered.wait();
            assert!(
                control.inner.ledger.try_lock().is_some(),
                "source comparison must not pin the ledger mutex"
            );
            control.retire_for_mapping_change().unwrap();
            release.wait();
            let ActivationLedgerAdmission::Terminal(state) = join.join().unwrap() else {
                panic!("racing replay revived retired owner");
            };
            assert_eq!(
                state.activation_receipt_disposition,
                crate::proto::ActivationReceiptDisposition::Retired as i32
            );
            assert!(weak.upgrade().is_none());
        }

        #[test]
        fn assigned_ledger_journal_requires_matching_typed_request_and_verifier() {
            let control = ActivatedTargetReceiptControl::new(17);
            let request = assigned_request(assigned_wire_request());
            let metadata = request.metadata().clone();
            let token = assigned_bound_token(&control, request);
            let key = ActivationJournalKey::Assigned {
                request: metadata,
                step: ActivationJournalStep::SessionStart,
            };
            let entry = ActivationJournalEntry {
                step: ActivationJournalStep::SessionStart,
                idempotency_key: key,
                status: ActivationJournalStatus::Applied,
                verifier_receipt: Some(ActivationJournalVerifier::Assigned(
                    ActivationJournalVerifierV2::Activation(
                        control
                            .activation_identity(&token)
                            .unwrap()
                            .assigned()
                            .unwrap()
                            .clone(),
                    ),
                )),
                typed_failure: None,
            };
            assert!(control.journal(&token, entry.clone()));
            assert!(control.journal_step_applied(&token, ActivationJournalStep::SessionStart));
            let mut bad = entry.clone();
            bad.idempotency_key = ActivationJournalKey::LegacyDigest([1; 32]);
            assert!(!control.journal(&token, bad));
            let mut bad = entry.clone();
            bad.verifier_receipt = Some(ActivationJournalVerifier::LegacyDigest([2; 32]));
            assert!(!control.journal(&token, bad));
            let mut bad = entry;
            bad.verifier_receipt = Some(ActivationJournalVerifier::Assigned(
                ActivationJournalVerifierV2::SessionGeneration(24),
            ));
            assert!(!control.journal(&token, bad));
        }

        #[test]
        fn assigned_journal_rejects_swapped_step_completed_key_and_missing_verifier() {
            use ActivationJournalStep as Step;
            use ActivationJournalVerifierV2 as Verifier;
            let control = ActivatedTargetReceiptControl::new(17);
            let request = assigned_request(assigned_wire_request());
            let metadata = request.metadata().clone();
            let token = assigned_bound_token(&control, request);
            let proof = assigned_proof(&control, &token);
            let target = proof.target_receipt.assigned().unwrap().clone();
            let activation = target.activation.clone();
            let occupancy = target.occupancy_receipt.clone();
            let cloud = target.cloud_receipt.clone();
            assert!(control.store_publication_proof(&token, proof));
            let cases = [
                (
                    Step::BackendRestoreSubmission,
                    Verifier::BackendGeneration(19),
                ),
                (Step::BackendCommit, Verifier::BackendGeneration(19)),
                (Step::RetirementPrepared, Verifier::BackendGeneration(19)),
                (Step::SessionStart, Verifier::Activation(activation.clone())),
                (
                    Step::ActivationRecord,
                    Verifier::Activation(activation.clone()),
                ),
                (Step::Seal, Verifier::Activation(activation)),
                (Step::OccupancyShmBegin, Verifier::SessionGeneration(23)),
                (
                    Step::OccupancyInstall,
                    Verifier::Occupancy(occupancy.clone()),
                ),
                (
                    Step::SharedStateCommit,
                    Verifier::Occupancy(occupancy.clone()),
                ),
                (Step::CacheClears, Verifier::Occupancy(occupancy.clone())),
                (Step::MapEvent, Verifier::Occupancy(occupancy)),
                (Step::CloudInstall, Verifier::Cloud(cloud.clone())),
                (Step::CloudEvent, Verifier::Cloud(cloud.clone())),
                (Step::TypedGlobalMap, Verifier::Cloud(cloud)),
                (Step::TargetReceiptVerification, Verifier::Target(target)),
            ];
            for (step, verifier) in cases {
                let key = ActivationJournalKey::Assigned {
                    request: metadata.clone(),
                    step,
                };
                for status in [
                    ActivationJournalStatus::Prepared,
                    ActivationJournalStatus::Applied,
                ] {
                    let entry = ActivationJournalEntry {
                        step,
                        idempotency_key: key.clone(),
                        status,
                        verifier_receipt: Some(ActivationJournalVerifier::Assigned(
                            verifier.clone(),
                        )),
                        typed_failure: None,
                    };
                    let mut missing = entry.clone();
                    missing.verifier_receipt = None;
                    assert!(!control.journal(&token, missing));
                    let mut completed = entry.clone();
                    completed.verifier_receipt = Some(ActivationJournalVerifier::Assigned(
                        Verifier::CompletedStep(key.clone()),
                    ));
                    assert!(!control.journal(&token, completed));
                    let swapped = if matches!(
                        step,
                        Step::CloudInstall | Step::CloudEvent | Step::TypedGlobalMap
                    ) {
                        Step::OccupancyShmBegin
                    } else {
                        Step::CloudInstall
                    };
                    let mut wrong_step = entry.clone();
                    wrong_step.step = swapped;
                    wrong_step.idempotency_key = ActivationJournalKey::Assigned {
                        request: metadata.clone(),
                        step: swapped,
                    };
                    assert!(
                        !control.journal(&token, wrong_step),
                        "wrong typed verifier for {swapped:?}"
                    );
                    assert!(
                        control.journal(&token, entry),
                        "legitimate {status:?} verifier for {step:?}"
                    );
                }
            }
        }

        #[test]
        fn assigned_journal_requires_exact_installed_cloud_and_occupancy_receipts() {
            use ActivationJournalStep as Step;
            use ActivationJournalVerifierV2 as Verifier;
            let control = ActivatedTargetReceiptControl::new(17);
            let request = assigned_request(assigned_wire_request());
            let metadata = request.metadata().clone();
            let token = assigned_bound_token(&control, request);
            let proof = assigned_proof(&control, &token);
            let target = proof.target_receipt.assigned().unwrap().clone();
            let entry = |step, verifier| ActivationJournalEntry {
                step,
                idempotency_key: ActivationJournalKey::Assigned {
                    request: metadata.clone(),
                    step,
                },
                status: ActivationJournalStatus::Applied,
                verifier_receipt: Some(ActivationJournalVerifier::Assigned(verifier)),
                typed_failure: None,
            };
            // A plausible cloud receipt cannot precede the validated publication proof.
            assert!(!control.journal(
                &token,
                entry(
                    Step::CloudInstall,
                    Verifier::Cloud(target.cloud_receipt.clone())
                )
            ));
            for field in ["nonce", "sequence", "revision", "timestamp"] {
                let mut occupancy = target.occupancy_receipt.clone();
                match field {
                    "nonce" => occupancy.commit_id.boot_nonce = 0,
                    "sequence" => occupancy.commit_id.sequence = 0,
                    "revision" => occupancy.occupancy_revision = 0,
                    _ => occupancy.source_timestamp_ns = 0,
                }
                assert!(!control.journal(
                    &token,
                    entry(Step::OccupancyInstall, Verifier::Occupancy(occupancy))
                ));
            }
            // Occupancy installation legitimately precedes cloud installation/proof storage.
            assert!(control.journal(
                &token,
                entry(
                    Step::OccupancyInstall,
                    Verifier::Occupancy(target.occupancy_receipt.clone())
                )
            ));
            assert!(control.store_publication_proof(&token, proof));
            for field in [
                "nonce",
                "sequence",
                "session",
                "occupancy",
                "cloud",
                "timestamp",
                "points",
            ] {
                let mut cloud = target.cloud_receipt.clone();
                match field {
                    "nonce" => cloud.commit_id.boot_nonce += 1,
                    "sequence" => cloud.commit_id.sequence += 1,
                    "session" => cloud.session_generation += 1,
                    "occupancy" => cloud.occupancy_revision += 1,
                    "cloud" => cloud.cloud_revision += 1,
                    "timestamp" => cloud.source_timestamp_ns += 1,
                    _ => cloud.point_count += 1,
                }
                assert!(
                    !control.journal(&token, entry(Step::CloudInstall, Verifier::Cloud(cloud))),
                    "changed cloud {field} proof accepted"
                );
            }
            for field in ["nonce", "sequence", "revision", "timestamp"] {
                let mut occupancy = target.occupancy_receipt.clone();
                match field {
                    "nonce" => occupancy.commit_id.boot_nonce += 1,
                    "sequence" => occupancy.commit_id.sequence += 1,
                    "revision" => occupancy.occupancy_revision += 1,
                    _ => occupancy.source_timestamp_ns += 1,
                }
                assert!(
                    !control.journal(
                        &token,
                        entry(Step::SharedStateCommit, Verifier::Occupancy(occupancy))
                    ),
                    "changed installed occupancy {field} accepted"
                );
            }
            assert!(control.journal(
                &token,
                entry(Step::CloudInstall, Verifier::Cloud(target.cloud_receipt))
            ));
            assert!(control.journal(
                &token,
                entry(
                    Step::SharedStateCommit,
                    Verifier::Occupancy(target.occupancy_receipt)
                )
            ));
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
                let token = control
                    .bind_activation_identity(&token, [4; 32], 11)
                    .unwrap();
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
                .seal(&token, Some(TargetReceipt::LegacyDigest([5; 32])), activation_base_state())
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
                            .seal(&token, Some(TargetReceipt::LegacyDigest(target_receipt_sha256)), activation_base_state())
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
                .seal(
                    &first_token,
                    Some(TargetReceipt::LegacyDigest([63; 32])),
                    activation_base_state(),
                )
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
            assert_eq!(
                snapshot.publication_proof.target_receipt.legacy_digest(),
                Some([63; 32])
            );
            assert_eq!(snapshot.immutable_target.as_ref(), &first_target);

            let snapshot_copy = snapshot.clone();
            let first_weak = Arc::downgrade(&snapshot.immutable_target);
            assert!(Arc::ptr_eq(
                &snapshot.immutable_target,
                &snapshot_copy.immutable_target
            ));
            assert!(same_sealed_target(&snapshot, &snapshot_copy));
            let current_snapshot = control.current_sealed_target_snapshot().unwrap();
            assert!(Arc::ptr_eq(
                &snapshot.immutable_target,
                &current_snapshot.immutable_target
            ));
            assert_eq!(
                snapshot.immutable_target.xyz_f32_le.as_ptr(),
                current_snapshot.immutable_target.xyz_f32_le.as_ptr()
            );
            assert!(control.current_sealed_assigned_target_snapshot().is_none());
            drop(current_snapshot);

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
            assert!(first_weak.upgrade().is_some());
            assert_eq!(snapshot.immutable_target.as_ref(), &first_target);
            drop(snapshot);
            assert!(
                first_weak.upgrade().is_some(),
                "snapshot clone retains the target allocation"
            );
            drop(snapshot_copy);
            assert!(
                first_weak.upgrade().is_none(),
                "last snapshot releases the retired allocation"
            );
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
                .seal(
                    &second_token,
                    Some(TargetReceipt::LegacyDigest([73; 32])),
                    activation_base_state(),
                )
                .expect("second target seal");
            let second_snapshot = control
                .current_sealed_target_snapshot()
                .expect("replacement sealed target snapshot");
            assert_eq!(
                second_snapshot.activation_request_id,
                "sealed-snapshot-second"
            );
            assert_eq!(second_snapshot.immutable_target.as_ref(), &second_target);
            assert_eq!(
                control.retained_immutable_target_bytes(),
                second_target.xyz_f32_le.len()
            );

            let second_weak = Arc::downgrade(&second_snapshot.immutable_target);
            control.retire_for_mapping_change().unwrap();
            assert!(control.current_sealed_target_snapshot().is_none());
            assert_eq!(control.retained_immutable_target_bytes(), 0);
            assert!(second_weak.upgrade().is_some());
            drop(second_snapshot);
            assert!(second_weak.upgrade().is_none());
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
                seal_control.seal(&seal_token, Some(TargetReceipt::LegacyDigest([9; 32])), activation_base_state())
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
                    idempotency_key: ActivationJournalKey::LegacyDigest([10; 32]),
                    status: ActivationJournalStatus::Applied,
                    verifier_receipt: Some(ActivationJournalVerifier::LegacyDigest([11; 32])),
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
                    target_receipt: TargetReceipt::LegacyDigest([12; 32]),
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

        fn committed_map(timestamp_ns: u64, _map_frame_id: &str) -> CommittedMapProof {
            CommittedMapProof {
                backend_kind: crate::mapping_backend::MappingBackendKind::Voxel,
                backend_generation: 1,
                source_timestamp_ns: timestamp_ns,
                width: 101,
                height: 79,
                resolution_m: 0.05,
                origin_x_m: -3.25,
                origin_y_m: 4.5,
            }
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
assert candidate.schema_version == 3
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
                    CORRECTION_CANDIDATE_FINGERPRINT_V3
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
                        backend: nuwax_codec::lcm::lio::LIO_BACKEND_DEVICE,
                        runtime_generation: 31,
                        source_sequence: 17,
                        source_timestamp_ns: timestamp_ns,
                        point_count: 60,
                    },
                    correlated_odom: CorrelatedOdomProof {
                        source_sequence: 18,
                        source_timestamp_ns: timestamp_ns.saturating_sub(1),
                        translation_m: [0.0, 0.0, 0.0],
                        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
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

        fn candidate_owner_test_trajectory() -> Vec<native_pgo_codec::OptimizedKeyframe> {
            vec![
                native_pgo_codec::OptimizedKeyframe {
                    source_timestamp_ns: 99,
                    translation_m: [0.0, 1.0, 2.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                native_pgo_codec::OptimizedKeyframe {
                    source_timestamp_ns: 100,
                    translation_m: [3.0, 4.0, 5.0],
                    rotation_xyzw: [0.5, 0.5, 0.5, 0.5],
                },
            ]
        }

        fn candidate_owner_test_candidate() -> CorrectionCandidate {
            let sample = sample_candidate(1, 100);
            CorrectionCandidate::seal_with_trajectory(
                sample.identity.clone(),
                sample.transform,
                candidate_owner_test_trajectory(),
            )
            .unwrap()
        }

        #[test]
        fn candidate_owner_moves_identity_and_trajectory_storage_without_copy() {
            let sample = sample_candidate(1, 100);
            let identity = sample.identity.clone();
            let session_storage = identity.mapping_session_id.as_ptr();
            let mut trajectory = Vec::with_capacity(32);
            trajectory.extend(candidate_owner_test_trajectory());
            let trajectory_storage = trajectory.as_ptr();
            let trajectory_capacity = trajectory.capacity();
            let candidate =
                CorrectionCandidate::seal_with_trajectory(identity, sample.transform, trajectory)
                    .unwrap();

            assert_eq!(
                candidate.identity.mapping_session_id.as_ptr(),
                session_storage
            );
            assert_eq!(candidate.trajectory.as_ptr(), trajectory_storage);
            assert_eq!(candidate.trajectory.capacity(), trajectory_capacity);
            assert_eq!(Arc::strong_count(candidate.owner_for_test()), 1);
        }

        #[test]
        fn candidate_owner_clones_share_complete_payload_and_release_last_owner() {
            let candidate = candidate_owner_test_candidate();
            let weak = Arc::downgrade(candidate.owner_for_test());
            let clones: Vec<_> = (0..8).map(|_| candidate.clone()).collect();
            assert_eq!(Arc::strong_count(candidate.owner_for_test()), 9);
            for clone in &clones {
                assert!(Arc::ptr_eq(
                    candidate.owner_for_test(),
                    clone.owner_for_test()
                ));
                assert!(std::ptr::eq(&candidate.identity, &clone.identity));
                assert!(std::ptr::eq(&candidate.transform, &clone.transform));
                assert_eq!(candidate.trajectory.as_ptr(), clone.trajectory.as_ptr());
            }
            drop(candidate);
            assert!(weak.upgrade().is_some());
            assert_eq!(Arc::strong_count(clones[0].owner_for_test()), 8);
            drop(clones);
            assert!(weak.upgrade().is_none());
        }

        #[test]
        fn candidate_owner_independent_and_decoded_replays_are_exact() {
            let candidate = candidate_owner_test_candidate();
            let independent = candidate_owner_test_candidate();
            let wire = candidate.encode_wire().unwrap();
            let decoded = CorrectionCandidate::decode_wire(&wire).unwrap();
            for replay in [&independent, &decoded] {
                assert!(!Arc::ptr_eq(
                    candidate.owner_for_test(),
                    replay.owner_for_test()
                ));
                assert_ne!(candidate.trajectory.as_ptr(), replay.trajectory.as_ptr());
                assert_eq!(&candidate, replay);
                assert_eq!(replay.encode_wire().unwrap(), wire);
            }
            let mut coordinator = CorrectionCoordinator::default();
            let exact = boundary(&candidate);
            let prepared = coordinator.prepare(&candidate, &exact);
            assert_eq!(prepared.disposition, CorrectionDisposition::Prepared);
            assert_eq!(
                coordinator.prepare(&independent, &exact).disposition,
                CorrectionDisposition::Prepared,
            );
            coordinator.commit_applied(&prepared).unwrap();
            assert_eq!(
                coordinator.prepare(&decoded, &exact).disposition,
                CorrectionDisposition::AlreadyApplied,
            );
        }

        #[test]
        fn candidate_owner_exact_comparison_checks_each_identity_field_without_digest_help() {
            type Edit = fn(&mut candidate_ownership::CorrectionCandidateData);
            let edits: &[(&str, Edit)] = &[
                ("backend", |data| {
                    data.identity.backend = CorrectionCandidateBackend::NativePgo
                }),
                ("runtime_generation", |data| {
                    data.identity.runtime_generation += 1
                }),
                ("backend_generation", |data| {
                    data.identity.backend_generation += 1
                }),
                ("candidate_sequence", |data| {
                    data.identity.candidate_sequence += 1
                }),
                ("mapping_session_id", |data| {
                    data.identity.mapping_session_id.push('x')
                }),
                ("mapping_session_generation", |data| {
                    data.identity.mapping_session_generation += 1
                }),
                ("map_frame_generation", |data| {
                    data.identity.map_frame_generation += 1
                }),
                ("mapping_commit_boot_nonce", |data| {
                    data.identity.mapping_commit_id.boot_nonce += 1
                }),
                ("mapping_commit_sequence", |data| {
                    data.identity.mapping_commit_id.sequence += 1
                }),
                ("occupancy_revision", |data| {
                    data.identity.occupancy_revision += 1
                }),
                ("cloud_revision", |data| data.identity.cloud_revision += 1),
                ("cloud_source_sequence", |data| {
                    data.identity.registered_cloud.source_sequence += 1
                }),
                ("cloud_source_timestamp", |data| {
                    data.identity.registered_cloud.source_timestamp_ns += 1
                }),
                ("cloud_point_count", |data| {
                    data.identity.registered_cloud.point_count += 1
                }),
                ("cloud_backend", |data| {
                    data.identity.registered_cloud.backend += 1
                }),
                ("cloud_runtime_generation", |data| {
                    data.identity.registered_cloud.runtime_generation += 1
                }),
                ("odom_source_sequence", |data| {
                    data.identity.correlated_odom.source_sequence += 1
                }),
                ("odom_source_timestamp", |data| {
                    data.identity.correlated_odom.source_timestamp_ns += 1
                }),
                ("map_source_timestamp", |data| {
                    data.identity.committed_map.source_timestamp_ns += 1
                }),
                ("map_width", |data| data.identity.committed_map.width += 1),
                ("map_height", |data| data.identity.committed_map.height += 1),
                ("map_backend_kind", |data| {
                    data.identity.committed_map.backend_kind = crate::mapping_backend::MappingBackendKind::Raytracing
                }),
                ("map_backend_generation", |data| {
                    data.identity.committed_map.backend_generation += 1
                }),
                ("source_timestamp", |data| {
                    data.identity.source_timestamp_ns += 1
                }),
                ("map_frame_id", |data| data.identity.map_frame_id.push('x')),
                ("odom_frame_id", |data| {
                    data.identity.odom_frame_id.push('x')
                }),
                ("body_frame_id", |data| {
                    data.identity.body_frame_id.push('x')
                }),
                ("points_frame_id", |data| {
                    data.identity.points_frame_id.push('x')
                }),
                ("lease_boot_nonce", |data| {
                    data.identity.correction_lease.boot_nonce += 1
                }),
                ("lease_generation", |data| {
                    data.identity.correction_lease.generation += 1
                }),
            ];
            let candidate = candidate_owner_test_candidate();
            let clone = candidate.clone();
            let original_wire = candidate.encode_wire().unwrap();
            for (field, edit) in edits {
                let changed = candidate.edited_for_test(edit);
                assert_ne!(candidate, changed, "{field}");
                assert_eq!(candidate, clone, "{field}");
                assert!(!Arc::ptr_eq(
                    candidate.owner_for_test(),
                    changed.owner_for_test()
                ));
            }
            assert_eq!(candidate.encode_wire().unwrap(), original_wire);
        }

        #[test]
        fn candidate_owner_exact_comparison_checks_every_float_bit() {
            let candidate = candidate_owner_test_candidate();
            // Transform, map, frozen odometry, and every ordered trajectory float are bit-exact.
            for index in 0..17 + candidate.trajectory.len() * 7 {
                let changed = candidate.edited_for_test(|data| {
                    let value = match index {
                        0..=2 => &mut data.transform.translation_m[index],
                        3..=6 => &mut data.transform.rotation_xyzw[index - 3],
                        7 => &mut data.identity.committed_map.resolution_m,
                        8 => &mut data.identity.committed_map.origin_x_m,
                        9 => &mut data.identity.committed_map.origin_y_m,
                        10..=12 => &mut data.identity.correlated_odom.translation_m[index - 10],
                        13..=16 => &mut data.identity.correlated_odom.rotation_xyzw[index - 13],
                        _ => {
                            let frame = &mut data.trajectory[(index - 17) / 7];
                            let component = (index - 17) % 7;
                            if component < 3 {
                                &mut frame.translation_m[component]
                            } else {
                                &mut frame.rotation_xyzw[component - 3]
                            }
                        }
                    };
                    *value = f64::from_bits(value.to_bits() ^ 1);
                });
                assert_ne!(candidate, changed, "float field {index}");
            }
        }

        #[test]
        fn candidate_owner_exact_comparison_preserves_signed_zero_and_nan_bits() {
            let candidate = candidate_owner_test_candidate().edited_for_test(|data| {
                data.transform.translation_m[0] = 0.0;
                data.identity.committed_map.origin_x_m = 0.0;
            });
            for index in 0..3 {
                let negative_zero = candidate.edited_for_test(|data| match index {
                    0 => data.transform.translation_m[0] = -0.0,
                    1 => data.identity.committed_map.origin_x_m = -0.0,
                    _ => data.trajectory[0].translation_m[0] = -0.0,
                });
                assert_ne!(candidate, negative_zero);
            }
            let nan = candidate.edited_for_test(|data| {
                data.transform.translation_m[0] = f64::from_bits(0x7ff8_0000_0000_0001);
                data.identity.committed_map.origin_x_m = f64::from_bits(0x7ff8_0000_0000_0002);
                data.trajectory[0].translation_m[0] = f64::from_bits(0x7ff8_0000_0000_0003);
            });
            let exact_nan = nan.edited_for_test(|_| {});
            assert!(!Arc::ptr_eq(
                nan.owner_for_test(),
                exact_nan.owner_for_test()
            ));
            assert_eq!(nan, exact_nan);
            let changed_nan = nan.edited_for_test(|data| {
                data.trajectory[0].translation_m[0] = f64::from_bits(0x7ff8_0000_0000_0004);
            });
            assert_ne!(nan, changed_nan);
            assert_eq!(
                nan.validate_integrity(),
                Err(CorrectionRejectReason::InvalidCommittedMapProof)
            );
        }

        #[test]
        fn candidate_owner_exact_comparison_checks_trajectory_count_order_and_timestamps() {
            let candidate = candidate_owner_test_candidate();
            let changed_count = candidate.edited_for_test(|data| {
                data.trajectory.pop();
            });
            let changed_order = candidate.edited_for_test(|data| data.trajectory.swap(0, 1));
            let changed_timestamp = candidate.edited_for_test(|data| {
                data.trajectory[0].source_timestamp_ns -= 1;
            });
            for changed in [changed_count, changed_order, changed_timestamp] {
                assert_ne!(candidate, changed);
            }
        }

        #[test]
        fn candidate_owner_decode_rejects_schema_skew_and_invalid_fields() {
            let candidate = candidate_owner_test_candidate();
            let mut legacy_schema = candidate.encode_body().unwrap();
            legacy_schema[..4].copy_from_slice(&2_i32.to_be_bytes());
            assert!(matches!(
                CorrectionCandidate::decode_body(&legacy_schema),
                Err(CorrectionWireError::Invalid(detail)) if detail == "unsupported correction schema version 2"
            ));
            let invalid = candidate.edited_for_test(|data| {
                data.transform.rotation_xyzw = [0.0; 4];
            });
            let invalid_body = invalid.encode_body_prefix().unwrap();
            assert!(matches!(
                CorrectionCandidate::decode_body(&invalid_body),
                Err(CorrectionWireError::Invalid(detail)) if detail == "InvalidRotation"
            ));
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
                r#"{"event":"nuwax_correction_candidate_finalized_v3","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"correction_lease_boot_nonce":"88776655443322111020304050607080","correction_lease_generation":19,"disposition":"Applied","candidate_processed_delivered":true}"#
            );
            assert_eq!(
                already_applied.to_string(),
                r#"{"event":"nuwax_correction_candidate_finalized_v3","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"correction_lease_boot_nonce":"88776655443322111020304050607080","correction_lease_generation":19,"disposition":"AlreadyApplied","candidate_processed_delivered":true}"#
            );
            let rejected_json = rejected.to_string();
            assert_eq!(
                rejected_json,
                r#"{"event":"nuwax_correction_candidate_finalized_v3","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"correction_lease_boot_nonce":"88776655443322111020304050607080","correction_lease_generation":19,"disposition":"Rejected","reject_reason":"invalid identity: field=\"mapping\\identity\"\ncontrol=\u0001","candidate_processed_delivered":true}"#
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
                r#"{"event":"nuwax_correction_candidate_finalized_v3","runtime_generation":11,"backend_generation":12,"session_generation":13,"candidate_sequence":1,"correction_lease_boot_nonce":"88776655443322111020304050607080","correction_lease_generation":19,"disposition":"Applied","candidate_processed_delivered":false}"#
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
            let expected_candidate = harness.apply_waiters.pending.lock()
                .get(&apply_key).unwrap().candidate.clone();

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
                    candidate: expected_candidate,
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
            let expected_candidate = harness.apply_waiters.pending.lock()
                .get(&apply_key).unwrap().candidate.clone();

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
                    candidate: expected_candidate,
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
                "\"correction_lease_boot_nonce\":\"{:032x}\"",
                apply_key.lease_boot_nonce
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
                    LioBackend::FastLio2,
                    "map",
                    "odom",
                    "base_link"
                ),
                Ok(CorrectionOwnerSpec::Relocalization { .. })
            ));
            assert!(matches!(
                CorrectionOwnerSpec::preflight_for_frames(
                    CorrectionBackend::LibraryPgo,
                    LioBackend::FastLio2,
                    "map",
                    "odom",
                    "base_link"
                ),
                Ok(CorrectionOwnerSpec::LibraryPgo { .. })
            ));
            assert!(matches!(
                CorrectionOwnerSpec::preflight_for_frames(
                    CorrectionBackend::NativePgo,
                    LioBackend::FastLio2,
                    "map",
                    "odom",
                    "base_link"
                ),
                Ok(CorrectionOwnerSpec::NativePgo { .. })
            ));
            assert!(CorrectionOwnerSpec::preflight_for_frames(
                CorrectionBackend::LibraryPgo,
                LioBackend::FastLio2,
                " map",
                "odom",
                "base_link"
            )
            .is_err());
        }

        #[test]
        fn pgo_environment_preflight_probe() {
            let Ok(expected) = std::env::var("NUWAX_PGO_PREFLIGHT_EXPECTED") else {
                return;
            };
            let frames = ValidatedFrameBindings::from_env().unwrap();
            let result = CorrectionOwnerSpec::preflight(
                CorrectionBackend::LibraryPgo, LioBackend::PointLio, &frames,
            );
            if expected == "rejected" {
                let message = result.unwrap_err();
                assert!(message.contains(PGO_OPTIONS_ENV), "{message}");
                return;
            }
            let CorrectionOwnerSpec::LibraryPgo { pipeline, .. } = result.unwrap() else {
                panic!("必须选择库内PGO");
            };
            assert_eq!(pipeline.icp.max_correspondence_distance, expected.parse::<f64>().unwrap());
            assert_eq!(pipeline.min_loop_overlap_ratio, 0.7, "未覆盖字段保留Point默认");
            println!("PGO读取成功：{}", pipeline.icp.max_correspondence_distance);
        }

        #[test]
        fn same_program_reads_changed_pgo_settings_on_each_start() {
            // 每组单独进程，避免通过全局环境污染并行用例；同一程序只换参数。
            for (raw, expected) in [
                (r#"{"max_correspondence_distance":0.22}"#, "0.22"),
                (r#"{"max_correspondence_distance":0.45}"#, "0.45"),
                ("{}", "0.3"),
                (r#"{"max_correspondence_distance":0}"#, "rejected"),
                (r#"{"not_wired":1}"#, "rejected"),
            ] {
                let output = std::process::Command::new(std::env::current_exe().unwrap())
                    .args(["--exact", "correction_coordinator::preflight_only_contract::tests::pgo_environment_preflight_probe", "--nocapture"])
                    .env(PGO_OPTIONS_ENV, raw)
                    .env("NUWAX_PGO_PREFLIGHT_EXPECTED", expected)
                    .env("NUWAX_GEOMETRY_MAP_FRAME", "map")
                    .env("NUWAX_GEOMETRY_ODOM_FRAME", "odom")
                    .env("NUWAX_BASE_FRAME", "base_link")
                    .output().unwrap();
                assert!(output.status.success(), "{}\n{}", String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr));
            }
        }

        #[test]
        fn preflight_selects_point_lio_registration_profile_only_for_point_lio() {
            for backend in [LioBackend::PointLio, LioBackend::FastLio2] {
                let CorrectionOwnerSpec::LibraryPgo { pipeline, .. } =
                    CorrectionOwnerSpec::preflight_for_frames(
                        CorrectionBackend::LibraryPgo,
                        backend,
                        "map",
                        "odom",
                        "base_link",
                    )
                    .unwrap()
                else {
                    panic!("必须选择库内回环");
                };
                let expected = if backend == LioBackend::PointLio {
                    LoopClosurePipelineConfig::for_point_lio()
                } else {
                    LoopClosurePipelineConfig::default()
                };
                assert_eq!(pipeline.max_loop_mse, expected.max_loop_mse);
                assert_eq!(
                    pipeline.min_loop_overlap_ratio,
                    expected.min_loop_overlap_ratio
                );
                assert_eq!(
                    pipeline.loop_confirmation_frames,
                    expected.loop_confirmation_frames
                );
                assert_eq!(pipeline.min_loop_interval_secs, expected.min_loop_interval_secs);
                assert_eq!(
                    pipeline.icp.max_correspondence_distance,
                    expected.icp.max_correspondence_distance
                );
            }
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

            let mut conflicting_transform = candidate.transform;
            conflicting_transform.translation_m[0] += 0.25;
            let conflicting =
                CorrectionCandidate::seal(candidate.identity.clone(), conflicting_transform)
                    .unwrap();
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
        fn coordinator_receipt_requires_exact_candidate_owner_before_abort_or_commit() {
            let candidate = candidate_owner_test_candidate();
            let mut coordinator = CorrectionCoordinator::default();
            let prepared = coordinator.prepare(&candidate, &boundary(&candidate));
            let different = candidate.edited_for_test(|data| data.trajectory[0].translation_m[0] = 0.25);
            let mut forged = prepared.clone();
            forged.candidate = different;
            assert_eq!(coordinator.abort(&forged), Err(CorrectionTransitionError::ReceiptMismatch));
            assert_eq!(coordinator.commit_applied(&forged), Err(CorrectionTransitionError::ReceiptMismatch));
            for change_sequence in [true, false] {
                let mut forged = prepared.clone();
                if change_sequence { forged.candidate_sequence += 1; }
                else { forged.source_timestamp_ns += 1; }
                assert_eq!(coordinator.commit_applied(&forged), Err(CorrectionTransitionError::ReceiptMismatch));
            }
            let mut independent_receipt = prepared.clone();
            independent_receipt.candidate = CorrectionCandidate::decode_wire(&candidate.encode_wire().unwrap()).unwrap();
            let applied = coordinator.commit_applied(&independent_receipt).unwrap();
            assert_eq!(applied.candidate, candidate);
            assert_eq!(coordinator.prepare(&candidate, &boundary(&candidate)).disposition,
                CorrectionDisposition::AlreadyApplied);
        }

        #[test]
        fn apply_waiter_key_domains_and_exact_owner_are_both_required() {
            let candidate = candidate_owner_test_candidate();
            let key = CandidateApplyKey::from(&candidate);
            let waiters = CandidateApplyWaiters::default();
            let lease = Arc::new(CorrectionRequestLease::new(CorrectionRequestCancellation::new(), None));
            for field in 0..5 {
                let mut wrong_key = key;
                match field {
                    0 => wrong_key.runtime_generation += 1,
                    1 => wrong_key.backend_generation += 1,
                    2 => wrong_key.lease_boot_nonce += 1,
                    3 => wrong_key.lease_generation += 1,
                    _ => wrong_key.candidate_sequence += 1,
                }
                assert!(waiters.register(wrong_key, candidate.clone(), lease.clone()).is_err());
                assert!(waiters.pending.lock().is_empty());
            }
            let weak = Arc::downgrade(candidate.owner_for_test());
            let receiver = waiters.register(key, candidate.clone(), lease.clone()).unwrap();
            assert!(waiters.register(key, candidate.clone(), lease.clone()).is_err());
            for different in [
                candidate.edited_for_test(|data| data.transform.translation_m[0] += 0.25),
                candidate.edited_for_test(|data| data.identity.registered_cloud.runtime_generation += 1),
                candidate.edited_for_test(|data| data.trajectory[0].translation_m[0] += 0.25),
            ] {
                assert_eq!(CandidateApplyKey::from(&different), key);
                assert!(waiters.claim_apply(key, &different).is_none());
                assert!(!waiters.pending.lock().get(&key).unwrap().applying);
            }
            let exact = CorrectionCandidate::decode_wire(&candidate.encode_wire().unwrap()).unwrap();
            drop(candidate);
            assert!(weak.upgrade().is_some(), "the waiter retains the admitted owner");
            assert!(waiters.claim_apply(key, &exact).is_some());
            assert!(waiters.claim_apply(key, &exact).is_none(), "only one claimant");
            assert!(waiters.apply_claimed_or_remove(key), "claimed owner cannot retire early");
            waiters.complete(key, Ok(CorrectionDisposition::Applied));
            assert!(weak.upgrade().is_none(), "completion releases the exact retained owner");
            assert!(waiters.pending.lock().is_empty());
            assert!(waiters.claim_apply(key, &exact).is_none(), "late owner cannot revive a retired waiter");
            drop(receiver);
        }

        #[test]
        fn admitted_frame_rejects_replaced_points_pose_owner_and_cloud_receipt() {
            let input = frame(1, 100, 0.0);
            let mut mutations = Vec::new();
            let mut points = input.clone();
            points.body_points = points.body_points.edited_for_test(|points| points[0].x += 0.25);
            mutations.push(points);
            let mut pose = input.clone();
            pose.correlated_odom.odom_from_body.translation.x += 0.25;
            mutations.push(pose);
            let mut pose_receipt = input.clone();
            pose_receipt.correlated_odom.proof.translation_m[0] += 0.25;
            mutations.push(pose_receipt);
            let mut source_owner = input.clone();
            source_owner.source_owner = frame(1, 100, 1.0).source_owner;
            mutations.push(source_owner);
            for field in 0..5 {
                let mut receipt = input.clone();
                match field {
                    0 => receipt.cloud_receipt.commit_id.boot_nonce += 1,
                    1 => receipt.cloud_receipt.commit_id.sequence += 1,
                    2 => receipt.cloud_receipt.occupancy_revision += 1,
                    3 => receipt.cloud_receipt.cloud_revision += 1,
                    _ => receipt.cloud_receipt.session_generation += 1,
                }
                mutations.push(receipt);
            }
            for changed in mutations {
                assert!(!input.exactly_matches(&changed));
                assert_eq!(changed.validate_source_ownership(), Err(CorrectionOwnerError::InvalidRegisteredCloudProof));
            }
            let exact_independent = frame(1, 100, 0.0);
            assert!(input.exactly_matches(&exact_independent));
            let mut later_occupancy = input.clone();
            later_occupancy.mapping_commit_id.sequence += 1;
            later_occupancy.occupancy_revision += 1;
            later_occupancy.validate_source_ownership().unwrap();
            assert!(!input.exactly_matches(&later_occupancy));
            let source_liveness = input.source_owner.retention_probe_for_test();
            drop(exact_independent);
            drop(input);
            assert!(source_liveness(), "cloned frame retains exact source owner");
            drop(later_occupancy);
            assert!(!source_liveness(), "source owner retires with the last frame");
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
                        value.registered_cloud.runtime_generation += 1;
                        value
                    },
                    CorrectionRejectReason::RegisteredCloudProofMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.correlated_odom.translation_m[0] = 1.0;
                        value
                    },
                    CorrectionRejectReason::CorrelatedOdomProofMismatch,
                ),
                (
                    {
                        let mut value = boundary(&candidate);
                        value.committed_map.backend_generation += 1;
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
        fn candidate_identity_transform_and_stop_fences_fail_closed() {
            let candidate = sample_candidate(1, 100);
            let exact = boundary(&candidate);

            let invalid_source = candidate.edited_for_test(|data| {
                data.identity.registered_cloud.runtime_generation = 0;
            });
            assert_eq!(
                CorrectionCoordinator::default()
                    .prepare(&invalid_source, &exact)
                    .disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::InvalidRegisteredCloudProof)
            );

            let non_finite = candidate.edited_for_test(|data| {
                data.transform.translation_m[0] = f64::NAN;
            });
            assert_eq!(
                CorrectionCoordinator::default()
                    .prepare(&non_finite, &exact)
                    .disposition,
                CorrectionDisposition::Rejected(CorrectionRejectReason::NonFiniteTransform)
            );

            let bad_rotation = candidate.edited_for_test(|data| {
                data.transform.rotation_xyzw = [0.0; 4];
            });
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

        #[test]
        fn correction_frame_clones_share_immutable_body_and_mask_allocations() {
            let original = frame(1, 10, 0.0);
            let clone = original.clone();
            assert_eq!(original.body_points.as_ptr(), clone.body_points.as_ptr());
            assert!(Arc::ptr_eq(&original.committed_map_occupancy, &clone.committed_map_occupancy));
            let conflicting = clone.body_points.edited_for_test(|points| points[0].x += 0.25);
            assert_ne!(original.body_points, conflicting);
            assert_eq!(original.body_points, clone.body_points);
            assert_eq!(original.registered_cloud, clone.registered_cloud);
            assert_eq!(original.correlated_odom, clone.correlated_odom);
        }

        fn frame_source_adapter(
            session_id: &str,
            base_anchor: Isometry3<f64>,
        ) -> crate::mapping_adapter::MappingAdapter {
            let mut adapter = crate::mapping_adapter::MappingAdapter::new_with_frames(
                crate::mapping_adapter::MappingAdapterConfig {
                    resolution: 0.05,
                    width_cells: 101,
                    height_cells: 79,
                    origin_xy: (-3.25, 4.5),
                    voxel_size: 0.05,
                    initial_safe_radius_meters: 0.0,
                },
                "robot_map", "robot_odom",
            );
            adapter.start_session(session_id, "correction-fixture-map").unwrap();
            establish_frame_source_anchor(&mut adapter, base_anchor);
            adapter
        }

        fn establish_frame_source_anchor(
            adapter: &mut crate::mapping_adapter::MappingAdapter,
            base_anchor: Isometry3<f64>,
        ) {
            // Mapping admission exposes a translation/yaw base anchor. Full-SE3
            // correction and frozen odometry remain independently exercised.
            let anchor_pose = base_anchor.inverse();
            let (roll, pitch, yaw) = anchor_pose.rotation.euler_angles();
            assert!(roll.abs() < 1.0e-12 && pitch.abs() < 1.0e-12);
            adapter.feed_pose_in_frame(
                anchor_pose.translation.x, anchor_pose.translation.y,
                anchor_pose.translation.z, yaw, "robot_odom",
            ).unwrap();
        }

        fn admit_frame_source(
            sequence: u64,
            timestamp_ns: u64,
            odom_timestamp_ns: u64,
            odom_from_body: Isometry3<f64>,
            odom_points: Vec<Point3<f64>>,
            adapter: &crate::mapping_adapter::MappingAdapter,
            mode_fence: Arc<RelocalizationModeFence>,
        ) -> crate::lio_ingress::AdmittedRegisteredScan {
            let ingress = crate::lio_ingress::CommonLioIngress::new(
                crate::lio_ingress::DeviceLioIngressSpec::new(
                    31, "robot_sensor", "robot_body", "robot_odom", "robot_map",
                ).unwrap(),
            );
            ingress.bind_relocalization_mode_fence(mode_fence).unwrap();
            admit_frame_source_with_ingress(
                sequence, timestamp_ns, odom_timestamp_ns, odom_from_body,
                odom_points, adapter, &ingress,
            )
        }

        fn admit_frame_source_with_ingress(
            sequence: u64,
            timestamp_ns: u64,
            odom_timestamp_ns: u64,
            odom_from_body: Isometry3<f64>,
            odom_points: Vec<Point3<f64>>,
            adapter: &crate::mapping_adapter::MappingAdapter,
            ingress: &crate::lio_ingress::CommonLioIngress,
        ) -> crate::lio_ingress::AdmittedRegisteredScan {
            let mut odometry = nuwax_codec::lcm::schema::Odometry::default();
            odometry.header.frame_id = "robot_odom".into();
            odometry.child_frame_id = "robot_body".into();
            odometry.pose.position = odom_from_body.translation.vector.into();
            let q = odom_from_body.rotation.quaternion();
            odometry.pose.orientation_xyzw = [q.i, q.j, q.k, q.w];
            let envelope = ingress.prepare_legacy_odometry(
                odom_timestamp_ns.try_into().unwrap(), odometry,
            ).unwrap();
            let cloud_identity = crate::lio_ingress::SourceIdentity {
                backend: envelope.backend,
                runtime_generation: envelope.runtime_generation,
                source_sequence: (sequence + 100).try_into().unwrap(),
                source_timestamp_ns: timestamp_ns.try_into().unwrap(),
            };
            let correlated = crate::lio_ingress::CorrelatedOdometry {
                source_sequence: envelope.source_sequence,
                source_timestamp_ns: envelope.source_timestamp_ns,
            };
            ingress.accept_public_odometry(envelope, 1).unwrap();
            let prepared = ingress.register_odom_points(
                cloud_identity, correlated, 2, odom_points.len(), odom_points, adapter,
            ).unwrap();
            let accepted = ingress.accept_public_cloud(prepared.encoded_for_test().as_ref(), 3).unwrap();
            accepted.scan().clone()
        }

        fn frame_from_admitted_source(
            sequence: u64,
            source: crate::lio_ingress::AdmittedRegisteredScan,
        ) -> CorrectionFrameInput {
            let binding = source.binding();
            let registered_cloud = RegisteredCloudProof::from_admitted(&source).unwrap();
            let correlated_odom = FrozenCorrelatedOdom {
                proof: CorrelatedOdomProof::from_registration(binding).unwrap(),
                odom_from_body: binding.frozen_odom_from_body,
            };
            let mut occupancy = vec![0_u8; 101 * 79];
            occupancy[..4].copy_from_slice(&[0, 20, 80, 255]);
            let mapping_commit_id = MappingCommitId { boot_nonce: COMMIT_NONCE, sequence };
            let input = CorrectionFrameInput {
                mapping_session_id: binding.mapping_session_id.clone(),
                mapping_session_generation: binding.mapping_session_generation,
                mapping_commit_id,
                occupancy_revision: sequence + 100,
                cloud_revision: sequence,
                cloud_receipt: crate::mapping_commit::CloudCommitReceipt {
                    commit_id: mapping_commit_id,
                    session_generation: binding.mapping_session_generation,
                    occupancy_revision: sequence + 100,
                    cloud_revision: sequence,
                    source_timestamp_ns: source.source_timestamp_ns(),
                    point_count: source.body_points().len().try_into().unwrap(),
                },
                map_frame_generation: binding.map_frame_generation,
                registered_cloud,
                correlated_odom,
                committed_map: committed_map(source.source_timestamp_ns(), "robot_map"),
                committed_map_occupancy: Arc::from(occupancy),
                points_frame_id: source.body_frame_id().to_owned(),
                body_points: source.body_points().clone(),
                static_point_indices: None,
                base_anchor_map_from_odom: binding.base_anchor_map_from_odom,
                frame_capture_relocalization_epoch: binding.frame_capture_relocalization_epoch.unwrap(),
                relocalization_mode_epoch: None,
                source_owner: source,
            };
            input.validate_source_ownership().unwrap();
            input
        }

        fn frame_with_mode_fence(
            sequence: u64,
            timestamp_ns: u64,
            x: f64,
            mode_fence: Arc<RelocalizationModeFence>,
        ) -> CorrectionFrameInput {
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let odom_from_body = Isometry3::translation(x, 0.0, 0.0);
            // The loop has 0.4 m drift; spacing separates true correspondences.
            let odom_points = asymmetric_cloud().iter()
                .map(|point| odom_from_body * Point3::from(point.coords * 3.0)).collect();
            frame_from_admitted_source(sequence, admit_frame_source(
                sequence, timestamp_ns, timestamp_ns, odom_from_body, odom_points,
                &adapter, mode_fence,
            ))
        }

        fn frame(sequence: u64, timestamp_ns: u64, x: f64) -> CorrectionFrameInput {
            frame_with_mode_fence(sequence, timestamp_ns, x, Arc::new(RelocalizationModeFence::default()))
        }

        fn active_relocalization_frame(
            owner: &CorrectionTransportOwner,
            sequence: u64,
            timestamp_ns: u64,
            x: f64,
        ) -> CorrectionFrameInput {
            let fence = owner.ingress().relocalization_mode_fence();
            let mode = fence.snapshot();
            assert!(mode.active, "fixture requires an active relocalization epoch");
            let mut input = frame_with_mode_fence(sequence, timestamp_ns, x, fence);
            input.relocalization_mode_epoch = Some(mode.epoch);
            input
        }

        #[test]
        fn accepted_static_scan_reaches_library_and_native_before_keyframes_and_loop_processing() {
            use crate::dynamic_scene::tests::{corridor, ORIGIN};
            let (people_scan, labels) = corridor(Some(0.0), 0.0);
            let people = labels.iter().filter(|label| **label).count();
            let threshold = people_scan.len() - people / 2;
            let pipeline = LoopClosurePipelineConfig {
                min_keyframe_points: threshold,
                min_keyframes_for_loop_search: 100,
                keyframe_translation: 0.001,
                ..LoopClosurePipelineConfig::default()
            };
            let mut library = CorrectionOwnerHandle::start(CorrectionOwnerSpec::LibraryPgo {
                map_frame_id: "robot_map".into(), odom_frame_id: "robot_odom".into(),
                body_frame_id: "robot_body".into(), pipeline,
            }, owner_identity()).unwrap();
            let mut history = crate::pgo_map::ScanHistory::default();
            let mut static_input = None;
            for sequence in 1..=7 {
                let world_points = if sequence == 7 { people_scan.clone() } else { corridor(None, 0.0).0 };
                let pose = Isometry3::translation(sequence as f64 * 0.1, 0.0, 0.0);
                let mut adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
                // Registration correction moves, but motion evidence stays in raw odometry.
                let anchor = Isometry3::translation(sequence as f64 * 2.0, -1.0, 0.3);
                let mut transform = adapter.map_odom_transform_snapshot().unwrap();
                transform.correction = anchor;
                transform.correction_revision = sequence;
                adapter.commit_relocalization_registration(transform);
                let sensor_origin_body = pose.inverse() * Point3::from(ORIGIN);
                let calibration = crate::native_lio_owner::BodyFromSensorTransform::from_json(
                    &serde_json::json!({
                        "translation_m": [sensor_origin_body.x, sensor_origin_body.y, sensor_origin_body.z],
                        "rotation_row_major": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    }).to_string(),
                ).unwrap();
                let ingress = crate::lio_ingress::NativeLioIngress::without_runtime(
                    crate::lio_ingress::DeviceLioIngressSpec::new_for_backend(
                        nuwax_codec::lcm::lio::LIO_BACKEND_FASTLIO2, 31,
                        "robot_sensor", "robot_body", "robot_odom", "robot_map",
                    ).unwrap(), calibration, Duration::from_millis(100),
                ).unwrap();
                ingress.bind_relocalization_mode_fence(Arc::new(RelocalizationModeFence::default())).unwrap();
                let source = admit_frame_source_with_ingress(
                    sequence, sequence * 100_000_000, sequence * 100_000_000,
                    pose, world_points, &adapter, ingress.common_for_test(),
                );
                history.record(&source, sequence * 100_000_000).unwrap();
                let mut input = frame_from_admitted_source(sequence, source.clone());
                input.static_point_indices = Some(history.static_point_indices(&source).unwrap());
                if sequence == 7 {
                    let indices = input.static_point_indices.as_ref().unwrap();
                    let retained_people = indices.iter().filter(|i| labels[**i as usize]).count();
                    let retained_static = indices.len() - retained_people;
                    assert!(retained_people * 5 < people);
                    assert_eq!(retained_static, people_scan.len() - people);
                    assert!(indices.len() < threshold && input.body_points.len() > threshold);
                    let raw_receipt = input.registered_cloud;
                    let raw_owner = input.source_owner.clone();
                    let mut request = native_pgo_request(&owner_identity(), "robot_map", "robot_odom", "robot_body", &input).unwrap();
                    request.identity.request_sequence = 1;
                    let request = native_pgo_codec::seal_native_pgo_request(request).unwrap();
                    let wire = native_pgo_codec::encode_native_pgo_request_wire(&request).unwrap();
                    let decoded = native_pgo_codec::decode_native_pgo_request_wire(&wire).unwrap();
                    assert_eq!(decoded.identity.registered_cloud.point_count as usize, indices.len());
                    assert_eq!(decoded.identity.registered_cloud.backend, raw_receipt.backend);
                    assert_eq!(decoded.identity.registered_cloud.runtime_generation as u64, raw_receipt.runtime_generation);
                    assert_eq!(decoded.identity.registered_cloud.source_sequence as u64, raw_receipt.source_sequence);
                    assert_eq!(decoded.registered_body_points_xyz,
                        input.pgo_points().unwrap().iter().flat_map(|p| [p.x, p.y, p.z]).collect::<Vec<_>>());
                    assert_eq!(input.registered_cloud, raw_receipt);
                    assert_eq!(input.source_owner, raw_owner);
                    input.validate_source_ownership().unwrap();
                    static_input = Some(source);
                }
                assert!(library.submit_frame(input).unwrap().is_none());
            }
            let CorrectionOwnerHandle::LibraryPgo { owner, .. } = &library else { unreachable!() };
            // Six actual keyframes precede the filtered below-threshold people scan.
            assert_eq!(owner.coordinator.graph().nodes.len(), 6);
            let last_source = static_input.expect("last admitted static selection");
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let wrong_source = admit_frame_source(
                8, 700_000_000, 700_000_000, Isometry3::identity(), people_scan,
                &adapter, Arc::new(RelocalizationModeFence::default()),
            );
            assert!(history.static_point_indices(&wrong_source).is_err());
            history.clear();
            assert!(history.static_point_indices(&last_source).is_err());
        }

        #[test]
        fn static_selection_rejects_invalid_source_and_indices_before_resetting_owner() {
            let mut library = CorrectionOwnerHandle::start(transport_library_spec(), owner_identity()).unwrap();
            library.submit_frame(frame(1, 1_000_000_000, 0.0)).unwrap();
            let mut adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            adapter.stop_session("mapping-session-owner").unwrap();
            adapter.start_session("mapping-session-newer", "newer-map").unwrap();
            establish_frame_source_anchor(&mut adapter, Isometry3::identity());
            for indices in [vec![1, 1], vec![u32::MAX], vec![2, 1]] {
                let pose = Isometry3::translation(1.0, 0.0, 0.0);
                let mut input = frame_from_admitted_source(2, admit_frame_source(
                    2, 2_000_000_000, 2_000_000_000, pose,
                    asymmetric_cloud().iter().map(|point| pose * point).collect(),
                    &adapter, Arc::new(RelocalizationModeFence::default()),
                ));
                input.validate_source_ownership().unwrap();
                assert!(input.mapping_session_generation > 1);
                input.static_point_indices = Some(indices.into());
                assert!(native_pgo_request(&owner_identity(), "robot_map", "robot_odom", "robot_body", &input).is_err());
                assert_eq!(library.submit_frame(input), Err(CorrectionOwnerError::InvalidRegisteredCloudProof));
                let CorrectionOwnerHandle::LibraryPgo { owner, .. } = &library else { unreachable!() };
                assert_eq!(owner.session.as_ref().unwrap().mapping_session_generation, 1);
                assert_eq!(owner.coordinator.graph().nodes.len(), 1);
            }
            let mut input = frame(2, 2_000_000_000, 1.0);
            input.registered_cloud.point_count += 1;
            assert!(native_pgo_request(&owner_identity(), "robot_map", "robot_odom", "robot_body", &input).is_err());
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
                request.identity.mapping_backend_kind,
                input.committed_map.backend_kind.stable_name()
            );
            assert_eq!(
                request.identity.mapping_backend_generation as u64,
                input.committed_map.backend_generation
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
            assert_eq!(request.identity.committed_map.width as u64, input.committed_map.width);
            assert_eq!(request.identity.committed_map.height as u64, input.committed_map.height);
            assert_eq!(request.identity.committed_map.resolution_m.to_bits(), input.committed_map.resolution_m.to_bits());
            assert_eq!(request.identity.committed_map.origin_x_m.to_bits(), input.committed_map.origin_x_m.to_bits());
            assert_eq!(request.identity.committed_map.origin_y_m.to_bits(), input.committed_map.origin_y_m.to_bits());
            assert_eq!(request.identity.registered_cloud.backend, input.registered_cloud.backend);
            assert_eq!(request.identity.registered_cloud.runtime_generation as u64, input.registered_cloud.runtime_generation);
            assert_ne!(request.identity.registered_cloud.runtime_generation, request.identity.runtime_generation);
            assert_eq!(request.identity.correlated_odom.translation_m, [0.25, 0.0, 0.0]);
            assert_eq!(request.identity.correlated_odom.rotation_xyzw, [0.0, 0.0, 0.0, 1.0]);
            assert_eq!(request.identity.correlated_odom.source_sequence as u64, input.correlated_odom.proof.source_sequence);
            assert_eq!(request.identity.correlated_odom.source_timestamp_ns as u64, input.correlated_odom.proof.source_timestamp_ns);
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

        #[derive(Default)]
        struct PausedBackgroundStage {
            entered: Notify,
            release: Notify,
            staged: parking_lot::Mutex<Vec<u64>>,
            discarded: parking_lot::Mutex<Vec<u64>>,
        }

        #[async_trait::async_trait]
        impl CorrectionCandidateSink for PausedBackgroundStage {
            async fn stage_frame(&self, input: &CorrectionFrameInput) -> Result<(), String> {
                let first = {
                    let mut staged = self.staged.lock();
                    staged.push(input.mapping_commit_id.sequence);
                    staged.len() == 1
                };
                if first {
                    self.entered.notify_one();
                    self.release.notified().await;
                }
                Ok(())
            }
            async fn discard_frame(&self, input: &CorrectionFrameInput) {
                self.discarded.lock().push(input.mapping_commit_id.sequence);
            }
            async fn apply_candidate(
                &self,
                _: CorrectionCandidate,
                _: CorrectionRequestCancellation,
            ) -> Result<CorrectionDisposition, String> {
                Err("两帧夹具不应产生回环候选".into())
            }
        }

        #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
        async fn background_mapping_coalesces_without_waiting_and_shutdown_cancels_pending_stage() {
            for stop_while_paused in [false, true] {
                let sink = Arc::new(PausedBackgroundStage::default());
                let owner = CorrectionTransportOwner::start(
                    transport_library_spec(),
                    owner_identity(),
                    &crate::lcm_runtime::test_lcm_url(),
                    sink.clone(),
                )
                .await
                .unwrap();
                let ingress = owner.ingress();
                assert!(ingress.supports_background_mapping());
                ingress
                    .submit_mapping_background(frame(1, 1_000_000_000, 0.0))
                    .unwrap();
                tokio::time::timeout(Duration::from_secs(2), sink.entered.notified())
                    .await
                    .unwrap();
                for sequence in 2..=100 {
                    ingress
                        .submit_mapping_background(frame(sequence, sequence * 1_000_000_000, 2.0))
                        .unwrap();
                }
                assert_eq!(sink.staged.lock().as_slice(), &[1]);
                assert_eq!(
                    ingress
                        .background
                        .as_ref()
                        .unwrap()
                        .pending
                        .lock()
                        .as_ref()
                        .unwrap()
                        .mapping_commit_id
                        .sequence,
                    100
                );
                if !stop_while_paused {
                    sink.release.notify_one();
                    tokio::time::timeout(Duration::from_secs(2), async {
                        while sink.discarded.lock().len() < 2 {
                            tokio::time::sleep(Duration::from_millis(1)).await;
                        }
                    })
                    .await
                    .unwrap();
                    assert_eq!(sink.staged.lock().as_slice(), &[1, 100]);
                }
                tokio::time::timeout(Duration::from_secs(2), owner.shutdown())
                    .await
                    .unwrap()
                    .unwrap();
                assert!(!ingress.is_open());
                assert!(ingress
                    .background
                    .as_ref()
                    .unwrap()
                    .pending
                    .lock()
                    .is_none());
                assert!(ingress
                    .submit_mapping_background(frame(101, 101_000_000_000, 2.0))
                    .is_err());
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

            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let rejected = frame_from_admitted_source(1, admit_frame_source(
                1, 1_000_000_000, 1_000_000_000, Isometry3::identity(),
                asymmetric_cloud().into_iter().take(3).collect(), &adapter,
                ingress.relocalization_mode_fence(),
            ));
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
            let (candidate_sequence, processed_candidate) = match processed {
                CorrectionSubmitDisposition::CandidateProcessed {
                    candidate_sequence,
                    candidate,
                    disposition: CorrectionDisposition::Applied,
                } => (candidate_sequence, candidate),
                disposition => panic!("expected applied native candidate, got {disposition:?}"),
            };
            assert_eq!(candidate_sequence, 4);
            assert_eq!(CandidateApplyKey::from(&processed_candidate).candidate_sequence, candidate_sequence);
            {
                let applied = sink.applied.lock();
                assert_eq!(applied.len(), 1);
                assert_eq!(
                    applied[0].identity.backend,
                    CorrectionCandidateBackend::NativePgo
                );
                assert_eq!(applied[0].identity.candidate_sequence, candidate_sequence);
                assert_eq!(applied[0], processed_candidate);
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
                    point_to_plane: false,
                    loop_confirmation_frames: 1,
                    gravity_prior_sigma_rad: None,
                    min_keyframes_for_loop_search: 3,
                    loop_submap_half_range: 0,
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
                background: None,
                sender: request_tx,
                admission_open: Arc::clone(&admission_open),
                admission_rx: admission_rx.clone(),
                relocalization_mode: phase_a.relocalization_mode(),
            };
            let worker = tokio::spawn(run_candidate_producer(
                CorrectionCandidateOwner::Local(Some(owner)),
                request_rx,
                admission_rx,
                CandidateProducerContext {
                    background: None,
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

        #[tokio::test]
        async fn background_preparation_outlives_delivery_timeout_but_keeps_cancel_and_missing_delivery(
        ) {
            for delivered in [true, false] {
                let cancellation = CorrectionRequestCancellation::for_background_mapping();
                let candidate = sample_candidate(1, 100);
            let key = CandidateApplyKey::from(&candidate);
                let waiters = CandidateApplyWaiters::default();
                let receiver = waiters
                    .register(
                        key,
                        candidate.clone(),
                        Arc::new(CorrectionRequestLease::new(cancellation.clone(), None)),
                    )
                    .unwrap();
                let (_admission, mut admission_rx) = watch::channel(true);
                let (outcome, ()) = tokio::join!(
                    await_candidate_apply(
                        receiver,
                        &cancellation,
                        &mut admission_rx,
                        Duration::from_millis(20),
                        &waiters,
                        key
                    ),
                    async {
                        if delivered {
                            tokio::time::sleep(Duration::from_millis(5)).await;
                            cancellation.begin_background_preparation();
                            // 开始准备不能唤醒取消等待者；完成时间超过原交付窗口。
                            tokio::time::sleep(Duration::from_millis(45)).await;
                            assert!(!cancellation.is_cancelled());
                            waiters.complete(key, Ok(CorrectionDisposition::Applied));
                        }
                    }
                );
                if delivered {
                    assert!(matches!(
                        outcome,
                        CandidateApplyWaitOutcome::Ack(Ok(Ok(CorrectionDisposition::Applied)))
                    ));
                } else {
                    assert!(matches!(
                        outcome,
                        CandidateApplyWaitOutcome::TimedOutBeforeCommit
                    ));
                    assert!(cancellation.is_cancelled());
                    waiters.remove(key);
                }
            }
            let cancellation = CorrectionRequestCancellation::for_background_mapping();
            cancellation.begin_background_preparation();
            let candidate = sample_candidate(2, 100);
            let key = CandidateApplyKey::from(&candidate);
            let waiters = CandidateApplyWaiters::default();
            let receiver = waiters
                .register(
                    key,
                    candidate.clone(),
                    Arc::new(CorrectionRequestLease::new(cancellation.clone(), None)),
                )
                .unwrap();
            let (_admission, mut admission_rx) = watch::channel(true);
            let (outcome, ()) = tokio::join!(
                await_candidate_apply(
                    receiver,
                    &cancellation,
                    &mut admission_rx,
                    Duration::from_millis(20),
                    &waiters,
                    key
                ),
                async {
                    tokio::time::sleep(Duration::from_millis(40)).await;
                    cancellation.cancel();
                }
            );
            assert!(matches!(
                outcome,
                CandidateApplyWaitOutcome::CancelledBeforeCommit
            ));
            assert!(waiters.pending.lock().is_empty());
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

        struct PreparationRejectingApplier {
            reject_next: AtomicBool,
            after_claim: bool,
            reason: CorrectionRejectReason,
            accepted: RecordingCanonicalApplier,
        }

        #[async_trait::async_trait]
        impl CanonicalCorrectionApplier for PreparationRejectingApplier {
            async fn apply_transaction(
                &self,
                transaction: CanonicalCorrectionTransaction,
                cancellation: CorrectionRequestCancellation,
            ) -> Result<(), CanonicalCorrectionApplyError> {
                if self.reject_next.swap(false, Ordering::AcqRel) {
                    if self.after_claim {
                        assert!(cancellation.operation().try_claim(&cancellation));
                    }
                    return Err(CanonicalCorrectionApplyError::PreparationRejected(
                        self.reason.clone(),
                    ));
                }
                self.accepted
                    .apply_transaction(transaction, cancellation)
                    .await
            }
        }

        #[tokio::test]
        async fn preparation_rejection_keeps_finalizer_alive_releases_lifecycle_and_accepts_next_candidate(
        ) {
            for reason in [
                CorrectionRejectReason::RebuildBudgetExceeded,
                CorrectionRejectReason::IncompleteScanHistory,
            ] {
                let applier = Arc::new(PreparationRejectingApplier {
                    reject_next: AtomicBool::new(true),
                    after_claim: false,
                    reason: reason.clone(),
                    accepted: RecordingCanonicalApplier::new(false),
                });
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
                let lifecycle = Arc::new(tokio::sync::Mutex::new(()));
                let waiters = Arc::new(CandidateApplyWaiters::default());
                let mut finalizer =
                    CanonicalPublishFinalizerSupervisor::start(sink.clone(), waiters.clone());
                for sequence in [1, 2] {
                    let input = frame(sequence, sequence * 100, 0.0);
                    let candidate =
                        candidate_for_frame(&input, sequence as u64, Isometry3::identity());
                    sink.stage_frame(&input).await.unwrap();
                    let cancellation = CorrectionRequestCancellation::new();
                    let lease = Arc::new(CorrectionRequestLease::new(
                        cancellation,
                        Some(lifecycle.clone().lock_owned().await),
                    ));
                    let key = CandidateApplyKey::from(&candidate);
                    let apply_rx = waiters.register(key, candidate.clone(), lease).unwrap();
                    let (outcome, outcome_rx) = oneshot::channel();
                    finalizer
                        .ingress
                        .sender
                        .send(CanonicalFinalizerJob {
                            key,
                            candidate: candidate.clone(),
                            lease: waiters.claim_apply(key, &candidate).unwrap(),
                            release_lease_on_completion: true,
                            outcome,
                        })
                        .await
                        .unwrap();
                    assert!(outcome_rx.await.unwrap().is_ok());
                    let disposition = apply_rx.await.unwrap().unwrap();
                    assert_eq!(
                        disposition,
                        if sequence == 1 {
                            CorrectionDisposition::Rejected(reason.clone())
                        } else {
                            CorrectionDisposition::Applied
                        }
                    );
                    assert_eq!(sink.correction_revision().await, sequence as u64 - 1);
                    assert!(lifecycle.try_lock().is_ok());
                    assert!(!finalizer.is_finished());
                }
                assert_eq!(applier.accepted.transactions().await.len(), 1);
                finalizer.shutdown().await.unwrap();
                sink.stop().await;
            }
        }

        #[tokio::test]
        async fn preparation_rejection_after_claim_is_still_poisoned() {
            let applier = Arc::new(PreparationRejectingApplier {
                reject_next: AtomicBool::new(true),
                after_claim: true,
                reason: CorrectionRejectReason::RebuildBudgetExceeded,
                accepted: RecordingCanonicalApplier::new(false),
            });
            let sink = GeometryCorrectionSink::new(
                owner_identity(),
                "robot_map",
                "robot_odom",
                "robot_body",
                applier,
            )
            .unwrap();
            let input = frame(1, 100, 0.0);
            sink.stage_frame(&input).await.unwrap();
            let (calls, hook) = test_fail_stop_counter();
            let cancellation = CorrectionRequestCancellation::with_fail_stop(hook);
            let operation = cancellation.operation();
            assert!(sink
                .apply_candidate(
                    candidate_for_frame(&input, 1, Isometry3::identity()),
                    cancellation,
                )
                .await
                .is_err());
            assert_eq!(operation.phase(), CanonicalOperationPhase::Poisoned);
            assert_eq!(calls.load(Ordering::Acquire), 1);
            assert_eq!(sink.correction_revision().await, 0);
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
        async fn staged_frame_exact_replay_rejects_changed_bytes_and_conflicting_discard() {
            let sink = GeometryCorrectionSink::new(
                owner_identity(), "robot_map", "robot_odom", "robot_body",
                Arc::new(RecordingCanonicalApplier::new(false)),
            ).unwrap();
            let input = frame(1, 100, 0.0);
            let source_liveness = input.source_owner.retention_probe_for_test();
            sink.stage_frame(&input).await.unwrap();
            let staged_owner = {
                let state = sink.state.lock().await;
                let staged = state.staged.as_ref().unwrap();
                assert_eq!(staged.input.body_points.as_ptr(), input.body_points.as_ptr());
                assert!(Arc::ptr_eq(&staged.input.committed_map_occupancy, &input.committed_map_occupancy));
                Arc::clone(&staged.input)
            };
            sink.stage_frame(&input.clone()).await.unwrap();
            assert!(Arc::ptr_eq(&staged_owner, &sink.state.lock().await.staged.as_ref().unwrap().input));
            let mut changed = input.clone();
            let mut occupancy = input.committed_map_occupancy.to_vec();
            occupancy[0] ^= 1;
            changed.committed_map_occupancy = Arc::from(occupancy);
            changed.validate_source_ownership().unwrap();
            assert_eq!(changed.mapping_commit_id, input.mapping_commit_id);
            assert_eq!(changed.committed_map, input.committed_map);
            assert!(sink.stage_frame(&changed).await.is_err());
            sink.discard_frame(&changed).await;
            assert!(Arc::ptr_eq(&staged_owner, &sink.state.lock().await.staged.as_ref().unwrap().input));
            drop(changed);
            sink.discard_frame(&input).await;
            assert!(sink.state.lock().await.staged.is_none());
            drop(staged_owner);
            drop(input);
            assert!(!source_liveness());
        }

        #[tokio::test]
        async fn staged_frame_discard_and_stop_release_retained_source_owners() {
            let sink = GeometryCorrectionSink::new(
                owner_identity(), "robot_map", "robot_odom", "robot_body",
                Arc::new(RecordingCanonicalApplier::new(false)),
            ).unwrap();
            let first = frame(1, 100, 0.0);
            let first_liveness = first.source_owner.retention_probe_for_test();
            sink.stage_frame(&first).await.unwrap();
            assert!(first_liveness());
            let second = frame(2, 200, 1.0);
            let second_liveness = second.source_owner.retention_probe_for_test();
            assert!(sink.stage_frame(&second).await.is_err(), "a different in-flight frame cannot replace this session's staged owner");
            sink.discard_frame(&first).await;
            drop(first);
            assert!(!first_liveness(), "exact discard releases the prior staged source");
            sink.stage_frame(&second).await.unwrap();
            let candidate = candidate_for_frame(&second, 1, Isometry3::identity());
            assert_eq!(sink.apply_candidate(candidate, CorrectionRequestCancellation::new()).await.unwrap(),
                CorrectionDisposition::Applied);
            drop(second);
            assert!(second_liveness(), "only the last-applied slot retains this source");
            sink.stop().await;
            assert!(!second_liveness(), "stop drops the last-applied owner too");
            let state = sink.state.lock().await;
            assert!(state.staged.is_none() && state.in_flight.is_none() && state.last_applied.is_none());
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
                UnitQuaternion::from_euler_angles(0.0, 0.0, 0.4),
            );
            let adapter = frame_source_adapter("mapping-session-owner", base_anchor);
            let correction_one = Isometry3::from_parts(
                Translation3::new(-0.6, 0.2, 0.4),
                UnitQuaternion::from_euler_angles(-0.1, 0.25, -0.3),
            );
            let correction_two = Isometry3::from_parts(
                Translation3::new(0.3, -0.7, 0.9),
                UnitQuaternion::from_euler_angles(0.35, -0.2, 0.15),
            );

            let first_input = frame_from_admitted_source(1, admit_frame_source(
                1, 100, 100, Isometry3::identity(), asymmetric_cloud(),
                &adapter, Arc::new(RelocalizationModeFence::default()),
            ));
            let first_candidate = candidate_for_frame(&first_input, 1, correction_one);
            sink.stage_frame(&first_input).await.unwrap();
            assert_eq!(
                sink.apply_candidate(first_candidate, CorrectionRequestCancellation::new())
                    .await
                    .unwrap(),
                CorrectionDisposition::Applied
            );

            let second_pose = Isometry3::translation(1.0, 0.0, 0.0);
            let mut second_input = frame_from_admitted_source(2, admit_frame_source(
                2, 200, 150, second_pose,
                asymmetric_cloud().iter().map(|point| second_pose * point).collect(),
                &adapter, Arc::new(RelocalizationModeFence::default()),
            ));
            second_input.cloud_revision = 42;
            second_input.cloud_receipt.cloud_revision = 42;
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
            assert_eq!(transactions[1].frozen_odom_source_timestamp_ns, 150);
            assert_eq!(transactions[1].source_timestamp_ns, 200);
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
        async fn lower_level_canonical_stage_preserves_full_se3_anchor_pose_and_trajectory() {
            let applier = Arc::new(RecordingCanonicalApplier::new(false));
            let sink = GeometryCorrectionSink::new(
                owner_identity(), "robot_map", "robot_odom", "robot_body", applier.clone(),
            ).unwrap();
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            let frozen_pose = Isometry3::from_parts(
                Translation3::new(1.2, -0.6, 0.4),
                UnitQuaternion::from_euler_angles(0.15, -0.25, 0.35),
            );
            let input = frame_from_admitted_source(1, admit_frame_source(
                1, 100, 90, frozen_pose,
                asymmetric_cloud().iter().map(|point| frozen_pose * point).collect(),
                &adapter, Arc::new(RelocalizationModeFence::default()),
            ));
            let full_anchor = Isometry3::from_parts(
                Translation3::new(8.0, -3.0, 1.5),
                UnitQuaternion::from_euler_angles(0.2, -0.15, 0.4),
            );
            let correction = Isometry3::from_parts(
                Translation3::new(-0.6, 0.2, 0.4),
                UnitQuaternion::from_euler_angles(-0.1, 0.25, -0.3),
            );
            let proposal = candidate_for_frame(&input, 1, correction);
            let trajectory = candidate_owner_test_trajectory();
            let candidate = CorrectionCandidate::seal_with_trajectory(
                proposal.identity.clone(), proposal.transform, trajectory.clone(),
            ).unwrap();
            sink.stage_frame(&input).await.unwrap();
            // Isolate the canonical stage's generic SE3 composition contract.
            // Real registration above remains yaw-anchor-only; this controlled
            // internal stage substitution is not an ingress integration claim.
            sink.state.lock().await.staged.as_mut().unwrap().base_anchor_map_from_odom = full_anchor;
            assert_eq!(sink.apply_candidate(candidate.clone(), CorrectionRequestCancellation::new()).await.unwrap(),
                CorrectionDisposition::Applied);
            let transactions = applier.transactions().await;
            let transaction = &transactions[0];
            assert!(transaction.matches_candidate_owner());
            assert_eq!(transaction.candidate, candidate);
            assert_isometry_close(&transaction.base_anchor_map_from_odom, &full_anchor);
            assert_isometry_close(&transaction.effective_map_from_odom, &(full_anchor * correction));
            assert_isometry_close(&transaction.frozen_odom_from_body, &input.correlated_odom.odom_from_body);
            assert_eq!(transaction.frozen_odom_source_timestamp_ns, 90);
            assert_eq!(transaction.trajectory, trajectory);
            assert!((transaction.effective_map_from_odom.translation.vector
                - (correction * full_anchor).translation.vector).norm() > 0.1);
            input.validate_source_ownership().unwrap();
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
            let adapter = frame_source_adapter("mapping-session-owner", Isometry3::translation(5.0, 1.0, -2.0));
            let input = frame_from_admitted_source(1, admit_frame_source(
                1, 100, 100, Isometry3::identity(), asymmetric_cloud(),
                &adapter, Arc::new(RelocalizationModeFence::default()),
            ));
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
            assert_eq!(binding.1.fingerprint(), CORRECTION_CANDIDATE_FINGERPRINT_V3);

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
                    candidate: candidate.clone(),
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
                // 这里只验证内部无候选生产器；默认配置已由按需重定位测试覆盖。
                CorrectionOwnerSpec::None,
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
                point_to_plane: false,
                loop_confirmation_frames: 1,
                gravity_prior_sigma_rad: None,
                min_keyframes_for_loop_search: 3,
                loop_submap_half_range: 0,
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
            changed_points_after_proof.body_points = changed_points_after_proof.body_points.edited_for_test(|points| points[0].x += 0.25);
            assert_eq!(
                library.submit_frame(changed_points_after_proof),
                Err(CorrectionOwnerError::InvalidRegisteredCloudProof)
            );
            assert!(library.submit_frame(frame(1, 0, 0.0)).unwrap().is_none());
            let mut wrong_namespace = frame(2, 10_000_000_000, 2.0);
            wrong_namespace.mapping_commit_id.boot_nonce += 1;
            wrong_namespace.cloud_receipt.commit_id.boot_nonce = wrong_namespace.mapping_commit_id.boot_nonce;
            assert_eq!(
                library.submit_frame(wrong_namespace),
                Err(CorrectionOwnerError::MappingCommitNamespaceChanged)
            );
            let mut changed_odom_after_proof = frame(2, 10_000_000_000, 2.0);
            changed_odom_after_proof.correlated_odom.odom_from_body =
                Isometry3::translation(3.0, 0.0, 0.0);
            assert_eq!(
                library.submit_frame(changed_odom_after_proof),
                Err(CorrectionOwnerError::InvalidRegisteredCloudProof)
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
        fn library_owner_accurate_turn_does_not_emit_pose_only_correction() {
            let mut library = CorrectionOwnerHandle::start(
                CorrectionOwnerSpec::LibraryPgo {
                    map_frame_id: "robot_map".into(),
                    odom_frame_id: "robot_odom".into(),
                    body_frame_id: "robot_body".into(),
                    pipeline: LoopClosurePipelineConfig::default(),
                },
                owner_identity(),
            )
            .unwrap();
            let world = asymmetric_cloud();
            let anchor = Isometry3::from_parts(
                Translation3::new(3.0, -2.0, 0.5),
                UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3),
            );
            let adapter = frame_source_adapter("mapping-session-owner", anchor);
            let mut effective = anchor;
            for (sequence, seconds, pose) in [
                (1, 0, Isometry3::identity()),
                (2, 10, Isometry3::translation(2.0, 0.0, 0.0)),
                (
                    3,
                    60,
                    Isometry3::from_parts(
                        Translation3::new(0.6, 0.0, 0.0),
                        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
                    ),
                ),
                (
                    4,
                    61,
                    Isometry3::from_parts(
                        Translation3::new(-0.1, 0.0, 0.0),
                        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
                    ),
                ),
                (
                    5,
                    62,
                    Isometry3::from_parts(
                        Translation3::new(-0.8, 0.0, 0.0),
                        UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI),
                    ),
                ),
            ] {
                let input = frame_from_admitted_source(sequence, admit_frame_source(
                    sequence, seconds * 1_000_000_000, seconds * 1_000_000_000,
                    pose, world.clone(), &adapter, Arc::new(RelocalizationModeFence::default()),
                ));
                // 对照原建图锚点与拥有方候选的公共位姿解释，保留上一校正以覆盖持续偏移。
                if let Some(candidate) = library.submit_frame(input).unwrap() {
                    let q = candidate.transform.rotation_xyzw;
                    effective = anchor
                        * Isometry3::from_parts(
                            Translation3::from(nalgebra::Vector3::from(
                                candidate.transform.translation_m,
                            )),
                            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                                q[3], q[0], q[1], q[2],
                            )),
                        );
                }
                assert_isometry_close(&(effective * pose), &(anchor * pose));
                for point in &world {
                    assert!(
                        (effective * pose * (pose.inverse() * point) - anchor * point).norm()
                            < 1e-8
                    );
                }
            }
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
            assert_eq!(first_candidate.identity.mapping_session_generation, 1);

            let mut adapter = frame_source_adapter("mapping-session-owner", Isometry3::identity());
            adapter.stop_session("mapping-session-owner").unwrap();
            adapter.start_session("mapping-session-restarted", "restarted-map").unwrap();
            establish_frame_source_anchor(&mut adapter, Isometry3::identity());
            let restarted_generation = adapter.snapshot().session_generation;
            let restarted_frame_generation = adapter.global_frame_generation();
            let restarted_frame = |sequence: u64, timestamp_ns: u64, x: f64| {
                let pose = Isometry3::translation(x, 0.0, 0.0);
                let odom_points = asymmetric_cloud().iter()
                    .map(|point| pose * Point3::from(point.coords * 3.0)).collect();
                let mut input = frame_from_admitted_source(sequence, admit_frame_source(
                    sequence, timestamp_ns, timestamp_ns, pose, odom_points,
                    &adapter, Arc::new(RelocalizationModeFence::default()),
                ));
                input.mapping_commit_id.boot_nonce = COMMIT_NONCE + 1;
                input.occupancy_revision = sequence;
                input.cloud_receipt.commit_id = input.mapping_commit_id;
                input.cloud_receipt.occupancy_revision = sequence;
                input.committed_map.backend_generation = adapter.backend_checkpoint_client().unwrap().backend_generation();
                input.validate_source_ownership().unwrap();
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
            assert_eq!(restarted_candidate.identity.mapping_session_generation, restarted_generation);
            assert_eq!(restarted_candidate.identity.map_frame_generation, restarted_frame_generation);
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
            wrong_boundary.committed_map.backend_generation = 0;
            assert_eq!(
                CorrectionCandidate::seal(wrong_boundary, candidate.transform),
                Err(CorrectionRejectReason::InvalidCommittedMapProof)
            );

            let mut missing_cloud_proof = candidate.identity.clone();
            missing_cloud_proof.registered_cloud.runtime_generation = 0;
            assert_eq!(
                CorrectionCandidate::seal(missing_cloud_proof, candidate.transform),
                Err(CorrectionRejectReason::InvalidRegisteredCloudProof)
            );
        }

        #[test]
        fn generated_python_v3_fixtures_match_complete_wire_both_directions() {
            // Serialized by the checked-in actual lcm-gen 1.5.2 Python class,
            // independently of this Rust encoder; generator is beside fixtures.
            for (candidate, generated_hex, expected_len) in [
                (sample_candidate(1, 100),
                    include_str!("../tests/fixtures/correction_candidate_v3_generated.hex"), 447),
                (candidate_owner_test_candidate(),
                    include_str!("../tests/fixtures/correction_candidate_v3_generated_trajectory.hex"), 575),
            ] {
                let hex = generated_hex.trim();
                assert_eq!(hex.len() % 2, 0);
                let generated_wire = hex.as_bytes().chunks_exact(2).map(|pair| {
                    u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap()
                }).collect::<Vec<_>>();
                assert_eq!(generated_wire.len(), expected_len);
                assert_eq!(&generated_wire[..8], &CORRECTION_CANDIDATE_FINGERPRINT_V3);
                assert_eq!(candidate.encode_wire().unwrap(), generated_wire);
                assert_eq!(candidate.encode_body().unwrap(), &generated_wire[8..]);
                let decoded = CorrectionCandidate::decode_wire(&generated_wire).unwrap();
                assert_eq!(decoded, candidate);
                assert_eq!(decoded.encode_wire().unwrap(), generated_wire);
                assert_eq!(CorrectionCandidate::decode_body(&generated_wire[8..]).unwrap(), candidate);
            }
        }

        #[test]
        fn generated_schema_wire_roundtrip_and_hostile_bytes_are_rejected() {
            let candidate = sample_candidate(1, 100);
            let body = candidate.encode_body().expect("encode candidate body");
            let encoded = candidate.encode_wire().expect("encode candidate");
            assert_eq!(&encoded[..8], &CORRECTION_CANDIDATE_FINGERPRINT_V3);
            assert_eq!(&encoded[8..], body);
            assert_ne!(&body[..8], &CORRECTION_CANDIDATE_FINGERPRINT_V3);
            assert_eq!(encoded.len(), body.len() + 8);
            assert_eq!(&body[..4], &3_i32.to_be_bytes());
            for old_version in [1_i32, 2_i32] {
                let mut legacy = body.clone();
                legacy[..4].copy_from_slice(&old_version.to_be_bytes());
                assert!(matches!(CorrectionCandidate::decode_body(&legacy),
                    Err(CorrectionWireError::Invalid(detail))
                        if detail == format!("unsupported correction schema version {old_version}")));
            }
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
            let mut double_fingerprint = CORRECTION_CANDIDATE_FINGERPRINT_V3.to_vec();
            double_fingerprint.extend_from_slice(&encoded);
            assert!(CorrectionCandidate::decode_wire(&double_fingerprint).is_err());
            assert!(CorrectionCandidate::decode_wire(&body).is_err());
            assert!(CorrectionCandidate::decode_body(&encoded).is_err());
            let mut invalid_trajectory_count = encoded.clone();
            let count_offset = invalid_trajectory_count.len() - 4;
            invalid_trajectory_count[count_offset..].copy_from_slice(&(-1_i32).to_be_bytes());
            assert!(CorrectionCandidate::decode_wire(&invalid_trajectory_count).is_err());
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
                CORRECTION_CANDIDATE_FINGERPRINT_V3
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
            let mut generated_wire = CORRECTION_CANDIDATE_FINGERPRINT_V3.to_vec();
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
            let candidate = sample_candidate(1, 100);
            let key = CandidateApplyKey::from(&candidate);
            let apply_rx = waiters.register(key, candidate.clone(), lease).expect("admitted exact waiter");

            phase_a.begin();
            phase_a.begin();

            assert!(!phase_a.is_open());
            assert!(cancellation.is_cancelled());
            assert!(waiters
                .register(
                    CandidateApplyKey::from(&sample_candidate(2, 101)),
                    sample_candidate(2, 101),
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
            let candidate = sample_candidate(1, 100);
            let key = CandidateApplyKey::from(&candidate);
            let apply_rx = waiters
                .register(key, candidate.clone(), Arc::clone(&lease))
                .expect("admitted exact waiter");
            let claimed = waiters.claim_apply(key, &candidate).expect("claimed waiter lease");

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
            let key = CandidateApplyKey::from(&candidate);
            let apply_rx = waiters
                .register(key, candidate.clone(), Arc::clone(&lease))
                .expect("admitted exact waiter");
            let claimed = waiters.claim_apply(key, &candidate).expect("claimed waiter lease");
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
            let key = CandidateApplyKey::from(&candidate);
            let waiters = Arc::new(CandidateApplyWaiters::default());
            let apply_rx = waiters.register(key, candidate.clone(), Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key, &candidate).unwrap();
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
            let key = CandidateApplyKey::from(&candidate);
            let waiters = Arc::new(CandidateApplyWaiters::default());
            let _apply_rx = waiters.register(key, candidate.clone(), Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key, &candidate).unwrap();
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
            let key = CandidateApplyKey::from(&candidate);
            let apply_rx = waiters.register(key, candidate.clone(), Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key, &candidate).unwrap();
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
            let key = CandidateApplyKey::from(&candidate);
            let apply_rx = waiters.register(key, candidate.clone(), Arc::clone(&lease)).unwrap();
            let claimed_lease = waiters.claim_apply(key, &candidate).unwrap();
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
                planar_work: None,
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
            let owner = CorrectionTransportOwner::start(CorrectionOwnerSpec::Relocalization {
                map_frame_id: "robot_map".into(), odom_frame_id: "robot_odom".into(), body_frame_id: "robot_body".into(),
            }, CorrectionOwnerRuntimeIdentity { runtime_generation: 2, ..owner_identity() }, "", Arc::new(FailingStageSink))
                .await.unwrap();
            let phase_a = owner.phase_a_control();
            let ingress = owner.relocalization_ingress().unwrap();
            let mut request = relocalization_command_request("seventeenth-waiter", 7);
            relocalization_request_body(&mut request).logical_timeout_ms = RELOCALIZATION_MIN_LOGICAL_TIMEOUT_MS;
            install_relocalization_target_for_request(&phase_a, &mut request, "robot_map");
            let mut waiters = Vec::new();
            for _ in 0..RELOCALIZATION_MAX_WAITERS { waiters.push(ingress.admit(request.clone()).await.unwrap()); }
            let pending_ingress = ingress.clone();
            let pending_request = request.clone();
            let seventeenth = tokio::spawn(async move { pending_ingress.admit(pending_request).await });
            tokio::task::yield_now().await;
            assert!(!seventeenth.is_finished(), "17th waiter must backpressure");
            let seventeenth = tokio::time::timeout(Duration::from_secs(1), seventeenth).await.unwrap().unwrap().unwrap();
            let terminal = seventeenth.wait().await.unwrap();
            assert_eq!(relocalization_terminal_body(&terminal).disposition, crate::proto::RelocalizeDisposition::DeadlineExceeded as i32);
            for waiter in waiters { assert_eq!(waiter.wait().await.unwrap().encode_to_vec(), terminal.encode_to_vec()); }
            assert_eq!(ingress.waiter_capacity.available_permits(), RELOCALIZATION_MAX_WAITERS);
            owner.shutdown().await.unwrap();
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
    RELOCALIZATION_MAX_TERMINAL_BYTES, RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES, RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES, RELOCALIZATION_MAX_EVIDENCE_PREFIX_BYTES, RELOCALIZATION_MAX_EVIDENCE_SUFFIX_BYTES, RELOCALIZATION_MAX_EVIDENCE_DISCLOSURE_BYTES, RELOCALIZATION_MAX_FRAMES, RELOCALIZATION_MAX_PROOF_BYTES, RELOCALIZATION_MAX_AGGREGATE_POINTS, RELOCALIZATION_MAX_EMITTED_POINTS,
    relocalization_algorithm_options_sha256_v1, relocalization_deadline_policy_sha256_v1,
    relocalization_source_policy_sha256_v1, validate_relocalization_request_id,
    relocalization_source_policy_preimage_v1, relocalization_deadline_policy_preimage_v1,
    relocalization_algorithm_options_preimage_v1, normalize_relocalization_logical_timeout_ms,
    ActivatedTargetReceiptControl, ActivationJournalEntry, ActivationJournalStatus,
    ActivationJournalStep, ActivationLedgerAdmission, ActivationPublicationProof,
    ActivationReceiptSeed, ActivationToken, CanonicalCorrectionApplier,
    CanonicalCorrectionApplyError, CanonicalCorrectionTransaction, CommittedMapProof,
    CorrectionCandidateBackend, CorrectionFrameIngress, CorrectionFrameInput, CorrectionLeaseId,
    CorrectionOwnerRuntimeIdentity, CorrectionOwnerSpec, CorrectionPhaseAControl,
    CorrectionProcessStopAttempt, CorrectionRejectReason, CorrectionRequestCancellation,
    CorrectionRuntimeError, CorrectionTransportOwner, CorrelatedOdomProof, FrozenCorrelatedOdom,
    GeometryCorrectionSink, RegisteredCloudProof, RelocalizationCommandError,
    PlanarCommandWork, PlanarCommandContext, PlanarCommandOutcome, planar_source_policy_sha256, planar_algorithm_policy_sha256,
    RelocalizationCommandIngress, RelocalizationCommandWaiter, RelocalizationModeFence,
    RelocalizationTerminalSnapshot, RELOCALIZATION_ALGORITHM_OPTIONS_VERSION,
    RELOCALIZATION_DEADLINE_POLICY_VERSION_V1, RELOCALIZATION_SOURCE_POLICY_VERSION,
};

#[cfg(test)]
pub(crate) use preflight_only_contract::{
    CorrectionCandidate, CorrectionCandidateIdentity, CorrectionTransform, CorrectionTransformSemantics, CorrectionCandidateSink, CorrectionDisposition,
};
