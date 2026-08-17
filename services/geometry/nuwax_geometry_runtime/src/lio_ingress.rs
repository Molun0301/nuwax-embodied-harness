//! Generation-scoped admission and registration fences for public LIO data.

use crate::mapping_adapter::MappingAdapter;
use crate::native_lio_owner::BodyFromSensorTransform;
use crate::{
    lcm_runtime::{
        handoff_lio_odometry, handoff_lio_odometry_with_correction,
        handoff_registered_lio_cloud_fenced, handoff_registered_lio_cloud_replay_with_correction,
        handoff_registered_lio_cloud_with_correction, GeometryCanonicalCorrectionApplier,
        GeometryRuntimeHandle, PoseState,
    },
    lio_owner::LioIngressSink,
};
use async_trait::async_trait;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use nuwax_codec::lcm::lio::{
    encode_lio_cloud_envelope, encode_lio_odometry_envelope, LioCloudEnvelope, LioOdometryEnvelope,
    LIO_BACKEND_DEVICE, LIO_BACKEND_FASTLIO2, LIO_BACKEND_POINTLIO,
    LIO_REGISTRATION_REGISTERED_MAP, LIO_SCHEMA_VERSION_V1, MAX_LIO_EMITTED_POINT_COUNT,
    MAX_LIO_SOURCE_POINT_COUNT,
};
use nuwax_codec::lcm::schema::{Header, Odometry, PointCloud2, PointField};
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
use parking_lot::Mutex;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tokio::sync::{oneshot, watch, Notify};

const POINT_FIELD_FLOAT32: u8 = 7;
const XYZ_POINT_STEP: i32 = 12;
const MAX_FRAME_ID_BYTES: usize = 255;
const MAX_PENDING_REGISTRATIONS: usize = 4;
const MAX_ACCEPTED_ODOMETRY_IDENTITIES: usize = 32;
const MAX_NATIVE_PAIR_CACHE: usize = 16;
#[cfg(not(test))]
const LIO_GENERATED_PUBLISH_TIMEOUT: Duration = Duration::from_secs(2);
#[cfg(test)]
const LIO_GENERATED_PUBLISH_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DeviceLioIngressSpec {
    pub(crate) backend: i32,
    pub(crate) runtime_generation: i64,
    pub(crate) sensor_frame_id: String,
    pub(crate) body_frame_id: String,
    pub(crate) odom_frame_id: String,
    pub(crate) map_frame_id: String,
}

impl DeviceLioIngressSpec {
    pub(crate) fn new(
        runtime_generation: i64,
        sensor_frame_id: impl Into<String>,
        body_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
        map_frame_id: impl Into<String>,
    ) -> Result<Self, LioIngressError> {
        Self::new_for_backend(
            LIO_BACKEND_DEVICE,
            runtime_generation,
            sensor_frame_id,
            body_frame_id,
            odom_frame_id,
            map_frame_id,
        )
    }

    pub(crate) fn new_for_backend(
        backend: i32,
        runtime_generation: i64,
        sensor_frame_id: impl Into<String>,
        body_frame_id: impl Into<String>,
        odom_frame_id: impl Into<String>,
        map_frame_id: impl Into<String>,
    ) -> Result<Self, LioIngressError> {
        if !matches!(
            backend,
            LIO_BACKEND_DEVICE | LIO_BACKEND_FASTLIO2 | LIO_BACKEND_POINTLIO
        ) {
            return Err(LioIngressError::UnsupportedBackend(backend));
        }
        if runtime_generation <= 0 {
            return Err(LioIngressError::InvalidGeneration(runtime_generation));
        }
        let spec = Self {
            backend,
            runtime_generation,
            sensor_frame_id: sensor_frame_id.into(),
            body_frame_id: body_frame_id.into(),
            odom_frame_id: odom_frame_id.into(),
            map_frame_id: map_frame_id.into(),
        };
        for (name, frame) in [
            ("sensor", spec.sensor_frame_id.as_str()),
            ("body", spec.body_frame_id.as_str()),
            ("odom", spec.odom_frame_id.as_str()),
            ("map", spec.map_frame_id.as_str()),
        ] {
            validate_frame(name, frame)?;
        }
        if [
            spec.sensor_frame_id.as_str(),
            spec.body_frame_id.as_str(),
            spec.odom_frame_id.as_str(),
            spec.map_frame_id.as_str(),
        ]
        .into_iter()
        .collect::<std::collections::HashSet<_>>()
        .len()
            != 4
        {
            return Err(LioIngressError::AliasedFrames);
        }
        Ok(spec)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SourceIdentity {
    pub(crate) backend: i32,
    pub(crate) runtime_generation: i64,
    pub(crate) source_sequence: i64,
    pub(crate) source_timestamp_ns: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CorrelatedOdometry {
    pub(crate) source_sequence: i64,
    pub(crate) source_timestamp_ns: i64,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AcceptedOdometry {
    pub(crate) envelope: LioOdometryEnvelope,
    pub(crate) adapter_timestamp_ns: i64,
}

#[derive(Debug)]
pub(crate) struct PreparedRegisteredCloud {
    pub(crate) envelope: LioCloudEnvelope,
    pub(crate) envelope_sha256: [u8; 32],
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AcceptedRegisteredCloud {
    pub(crate) envelope: LioCloudEnvelope,
    pub(crate) map_points: Vec<Point3<f64>>,
    pub(crate) adapter_timestamp_ns: i64,
    pub(crate) binding: RegistrationBinding,
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RegistrationBinding {
    pub(crate) runtime_generation: i64,
    pub(crate) cloud_source_sequence: i64,
    pub(crate) correlated_odometry: CorrelatedOdometry,
    pub(crate) mapping_session_id: String,
    pub(crate) mapping_session_generation: u64,
    pub(crate) map_frame_generation: u64,
    pub(crate) envelope_sha256: [u8; 32],
    pub(crate) frozen_odom_from_body: Isometry3<f64>,
    pub(crate) base_anchor_map_from_odom: Isometry3<f64>,
    /// Epoch observed when this registered cloud becomes an admitted
    /// correction source. `None` means it has not crossed that boundary.
    pub(crate) frame_capture_relocalization_epoch: Option<u64>,
}

/// Immutable mode provenance captured when a raw cloud first enters the
/// ingress path. Production registration validates this token while holding
/// the mapping lifecycle guard, so a frame cannot cross an admission edge and
/// be relabeled by its later public callback.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct RegistrationModeToken {
    frame_capture_relocalization_epoch: Option<u64>,
}

#[derive(Debug)]
struct PendingRegistration {
    binding: RegistrationBinding,
    map_points: Vec<Point3<f64>>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct StreamCursor {
    latest: Option<CorrelatedOdometry>,
}

impl StreamCursor {
    fn check(&self, identity: SourceIdentity) -> Result<(), LioIngressError> {
        if identity.source_sequence <= 0 {
            return Err(LioIngressError::InvalidSequence(identity.source_sequence));
        }
        if identity.source_timestamp_ns < 0 {
            return Err(LioIngressError::InvalidTimestamp(
                identity.source_timestamp_ns,
            ));
        }
        if let Some(latest) = self.latest {
            if identity.source_sequence <= latest.source_sequence {
                return Err(LioIngressError::NonIncreasingSequence {
                    previous: latest.source_sequence,
                    incoming: identity.source_sequence,
                });
            }
            if identity.source_timestamp_ns <= latest.source_timestamp_ns {
                return Err(LioIngressError::NonIncreasingTimestamp {
                    previous: latest.source_timestamp_ns,
                    incoming: identity.source_timestamp_ns,
                });
            }
        }
        Ok(())
    }

    fn commit(&mut self, identity: SourceIdentity) {
        self.latest = Some(CorrelatedOdometry {
            source_sequence: identity.source_sequence,
            source_timestamp_ns: identity.source_timestamp_ns,
        });
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CommonLioIngressSnapshot {
    pub(crate) runtime_generation: i64,
    pub(crate) closed: bool,
    pub(crate) latest_odometry: Option<CorrelatedOdometry>,
    pub(crate) latest_cloud: Option<CorrelatedOdometry>,
    pub(crate) latest_adapter_timestamp_ns: Option<i64>,
}

pub(crate) struct CommonLioIngress {
    spec: DeviceLioIngressSpec,
    state: Mutex<CommonLioIngressState>,
    runtime: Option<CommonLioRuntime>,
    odometry_admission: Notify,
    // Correction-aware construction binds the one fence whose epoch must be
    // captured when a source frame receives its immutable registration permit.
    relocalization_mode_fence:
        OnceLock<Arc<crate::correction_coordinator::RelocalizationModeFence>>,
}

#[derive(Clone)]
struct CommonLioRuntime {
    handle: GeometryRuntimeHandle,
    public_odometry: (LcmTopic, LcmMessageType),
    public_cloud: (LcmTopic, LcmMessageType),
    tf: (LcmTopic, LcmMessageType),
}

#[derive(Debug, Default)]
struct CommonLioIngressState {
    odometry: StreamCursor,
    cloud: StreamCursor,
    latest_adapter_timestamp_ns: Option<i64>,
    pending_registrations: HashMap<(i64, i64), PendingRegistration>,
    prepared_odometry: StreamCursor,
    prepared_cloud: StreamCursor,
    next_legacy_odometry_sequence: i64,
    next_legacy_cloud_sequence: i64,
    accepted_odometry_identities: VecDeque<CorrelatedOdometry>,
    accepted_odometry_poses: HashMap<(i64, i64), Isometry3<f64>>,
    closed: bool,
}

#[derive(Clone)]
struct NativeOdometrySample {
    identity: SourceIdentity,
    odometry: Odometry,
}

struct NativeCloudSample {
    identity: SourceIdentity,
    source_point_count: usize,
    sensor_points: Vec<Point3<f64>>,
    registration_mode_token: RegistrationModeToken,
}

struct NativeSourcePair {
    cloud: NativeCloudSample,
    odometry: NativeOdometrySample,
}

struct NativePairState {
    odometry: VecDeque<NativeOdometrySample>,
    clouds: VecDeque<NativeCloudSample>,
    probe_phase: NativeProbePhase,
    public_replay: NativePublicReplayPhase,
    next_replay_nonce: u64,
}

impl Default for NativePairState {
    fn default() -> Self {
        Self {
            odometry: VecDeque::new(),
            clouds: VecDeque::new(),
            probe_phase: NativeProbePhase::Runtime,
            public_replay: NativePublicReplayPhase::Normal,
            next_replay_nonce: 0,
        }
    }
}

#[derive(Clone)]
pub(crate) struct ValidatedStartupPair {
    odometry_identity: SourceIdentity,
    cloud_identity: SourceIdentity,
    registration_mode_token: Option<RegistrationModeToken>,
    odometry: Box<LioOdometryEnvelope>,
    frozen_odom_from_body: Isometry3<f64>,
    registered_odom_points: Arc<[Point3<f64>]>,
    source_point_count: usize,
    emitted_point_count: usize,
    odometry_payload_sha256: [u8; 32],
    registered_points_sha256: [u8; 32],
}

enum NativeProbePhase {
    Probing,
    Retained(Box<Option<ValidatedStartupPair>>),
    Transferred,
    Runtime,
    Closed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum NativeReplayFrameKind {
    Odometry,
    Cloud,
}

struct NativeReplayExpectation {
    nonce: u64,
    kind: NativeReplayFrameKind,
    identity: SourceIdentity,
    topic: LcmTopic,
    message_type: LcmMessageType,
    payload_sha256: [u8; 32],
    outcome: Option<oneshot::Sender<Result<(), String>>>,
}

enum NativePublicReplayPhase {
    Normal,
    Expecting(NativeReplayExpectation),
    Claimed {
        kind: NativeReplayFrameKind,
        nonce: u64,
        outcome: Option<oneshot::Sender<Result<(), String>>>,
    },
    Complete {
        nonce: u64,
    },
    Poisoned(String),
    Closed,
}

#[derive(Clone, Copy)]
struct NativeReplayClaim {
    kind: NativeReplayFrameKind,
    nonce: u64,
}

enum NativePublicFrameDecision {
    Normal,
    Ignore,
    Claim(NativeReplayClaim),
}

pub(crate) struct NativeLioIngress {
    common: CommonLioIngress,
    body_from_sensor: BodyFromSensorTransform,
    pair_max_skew_ns: i64,
    pair_state: Mutex<NativePairState>,
    source_pair_ready_tx: watch::Sender<bool>,
    source_pair_ready_rx: watch::Receiver<bool>,
}

enum CorrectionAwareLioIngressInner {
    Device(Arc<CommonLioIngress>),
    Native(Arc<NativeLioIngress>),
}

pub(crate) struct CorrectionAwareLioIngress {
    inner: CorrectionAwareLioIngressInner,
    correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
    correction_applier: Arc<GeometryCanonicalCorrectionApplier>,
}

enum LioRoutingState {
    Preparing,
    StartupOdometry {
        active: Arc<dyn LioIngressSink>,
        token: u64,
    },
    DeferredCloud {
        active: Arc<dyn LioIngressSink>,
        token: u64,
    },
    CloudReplaying {
        active: Arc<dyn LioIngressSink>,
        token: u64,
    },
    Active(Arc<dyn LioIngressSink>),
    Poisoned,
    Closed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LioReplayToken(u64);

#[derive(Clone, Debug, PartialEq, Eq)]
struct DeferredMappingIdentity {
    session_id: String,
    map_name: String,
    session_generation: u64,
    map_frame_generation: u64,
}

struct ClaimedStartupReplay {
    identity: DeferredMappingIdentity,
    pair: ValidatedStartupPair,
}

enum NativeDeferredReplayPhase {
    Startup(Option<ValidatedStartupPair>),
    Deferred(Option<ValidatedStartupPair>),
    Claimed(Option<ClaimedStartupReplay>),
    CloudReplaying(DeferredMappingIdentity, bool),
    Complete(DeferredMappingIdentity),
    Poisoned(String),
    Closed,
}

pub(crate) enum DeferredSessionClaim {
    Claimed,
    Complete,
}

pub(crate) struct NativeDeferredReplayControl {
    ingress: Arc<NativeLioIngress>,
    gate: Arc<LioIngressRoutingGate>,
    correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
    correction_applier: Arc<GeometryCanonicalCorrectionApplier>,
    correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
    token: LioReplayToken,
    phase: Mutex<NativeDeferredReplayPhase>,
    stop_requested: AtomicBool,
    readiness: crate::server::ReadinessAuthority,
    poison_tx: watch::Sender<Option<String>>,
    poison_diagnostics: Mutex<Vec<String>>,
}

/// Initially closed production ingress. Before activation it forwards only to
/// an unbound validation probe, so Native LIO can prove its first source pair
/// without reaching Mapping or canonical state.
pub(crate) struct LioIngressRoutingGate {
    probe: Arc<dyn LioIngressSink>,
    state: Mutex<LioRoutingState>,
    state_changed: Notify,
    next_replay_token: std::sync::atomic::AtomicU64,
}

impl LioIngressRoutingGate {
    pub(crate) fn new(probe: Arc<dyn LioIngressSink>) -> Self {
        Self {
            probe,
            state: Mutex::new(LioRoutingState::Preparing),
            state_changed: Notify::new(),
            next_replay_token: std::sync::atomic::AtomicU64::new(0),
        }
    }

    pub(crate) fn bind_and_open(&self, ingress: Arc<dyn LioIngressSink>) -> Result<(), String> {
        if Arc::ptr_eq(&self.probe, &ingress) {
            return Err(
                "LIO routing gate probe reuse requires bind_and_open_reusing_probe".to_owned(),
            );
        }
        let mut state = self.state.lock();
        match &*state {
            LioRoutingState::Preparing => {
                *state = LioRoutingState::Active(ingress);
                drop(state);
                self.probe.close();
                self.state_changed.notify_waiters();
                Ok(())
            }
            LioRoutingState::StartupOdometry { .. }
            | LioRoutingState::DeferredCloud { .. }
            | LioRoutingState::CloudReplaying { .. }
            | LioRoutingState::Active(_) => Err("LIO routing gate is already active".to_owned()),
            LioRoutingState::Poisoned | LioRoutingState::Closed => {
                Err("LIO routing gate is closed".to_owned())
            }
        }
    }

    pub(crate) fn bind_deferred(
        &self,
        active: Arc<dyn LioIngressSink>,
    ) -> Result<LioReplayToken, String> {
        if Arc::ptr_eq(&self.probe, &active) {
            return Err("LIO retained-pair replay requires a distinct active ingress".to_owned());
        }
        let token = self
            .next_replay_token
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel)
            .checked_add(1)
            .ok_or_else(|| "LIO replay token exhausted".to_owned())?;
        let mut state = self.state.lock();
        match &*state {
            LioRoutingState::Preparing => {
                *state = LioRoutingState::StartupOdometry { active, token };
                drop(state);
                self.probe.close();
                self.state_changed.notify_waiters();
                Ok(LioReplayToken(token))
            }
            LioRoutingState::StartupOdometry { .. }
            | LioRoutingState::DeferredCloud { .. }
            | LioRoutingState::CloudReplaying { .. }
            | LioRoutingState::Active(_) => Err("LIO routing gate is already active".to_owned()),
            LioRoutingState::Poisoned | LioRoutingState::Closed => {
                Err("LIO routing gate is closed".to_owned())
            }
        }
    }

    pub(crate) fn finish_startup_odometry(&self, token: LioReplayToken) -> Result<(), String> {
        let mut state = self.state.lock();
        let previous = std::mem::replace(&mut *state, LioRoutingState::Closed);
        match previous {
            LioRoutingState::StartupOdometry {
                active,
                token: actual,
            } if actual == token.0 => {
                *state = LioRoutingState::DeferredCloud {
                    active,
                    token: actual,
                };
                drop(state);
                self.state_changed.notify_waiters();
                Ok(())
            }
            LioRoutingState::StartupOdometry {
                active,
                token: actual,
            } => {
                *state = LioRoutingState::StartupOdometry {
                    active,
                    token: actual,
                };
                Err("LIO startup odometry token is stale".to_owned())
            }
            other => {
                *state = other;
                Err("LIO routing gate is not admitting startup odometry".to_owned())
            }
        }
    }

    pub(crate) fn begin_cloud_replay(&self, token: LioReplayToken) -> Result<(), String> {
        let mut state = self.state.lock();
        let previous = std::mem::replace(&mut *state, LioRoutingState::Closed);
        match previous {
            LioRoutingState::DeferredCloud {
                active,
                token: actual,
            } if actual == token.0 => {
                *state = LioRoutingState::CloudReplaying {
                    active,
                    token: actual,
                };
                drop(state);
                self.state_changed.notify_waiters();
                Ok(())
            }
            LioRoutingState::DeferredCloud {
                active,
                token: actual,
            } => {
                *state = LioRoutingState::DeferredCloud {
                    active,
                    token: actual,
                };
                Err("LIO deferred cloud token is stale".to_owned())
            }
            other => {
                *state = other;
                Err("LIO routing gate is not deferring a cloud".to_owned())
            }
        }
    }

    pub(crate) fn finish_cloud_replay(&self, token: LioReplayToken) -> Result<(), String> {
        let mut state = self.state.lock();
        let previous = std::mem::replace(&mut *state, LioRoutingState::Closed);
        match previous {
            LioRoutingState::CloudReplaying {
                active,
                token: actual,
            } if actual == token.0 => {
                *state = LioRoutingState::Active(active);
                drop(state);
                self.state_changed.notify_waiters();
                Ok(())
            }
            LioRoutingState::CloudReplaying {
                active,
                token: actual,
            } => {
                *state = LioRoutingState::CloudReplaying {
                    active,
                    token: actual,
                };
                Err("LIO cloud replay token is stale".to_owned())
            }
            other => {
                *state = other;
                Err("LIO routing gate is not replaying a cloud".to_owned())
            }
        }
    }

    pub(crate) fn poison(&self) {
        let active = {
            let mut state = self.state.lock();
            match std::mem::replace(&mut *state, LioRoutingState::Poisoned) {
                LioRoutingState::StartupOdometry { active, .. }
                | LioRoutingState::DeferredCloud { active, .. }
                | LioRoutingState::CloudReplaying { active, .. }
                | LioRoutingState::Active(active) => Some(active),
                LioRoutingState::Preparing
                | LioRoutingState::Poisoned
                | LioRoutingState::Closed => None,
            }
        };
        self.probe.close();
        if let Some(active) = active {
            active.close();
        }
        self.state_changed.notify_waiters();
    }

    pub(crate) fn is_closed(&self) -> bool {
        matches!(
            *self.state.lock(),
            LioRoutingState::Poisoned | LioRoutingState::Closed
        )
    }

    #[cfg(test)]
    pub(crate) fn is_open(&self) -> bool {
        matches!(*self.state.lock(), LioRoutingState::Active(_))
    }

    fn public_target(&self) -> Option<Arc<dyn LioIngressSink>> {
        match &*self.state.lock() {
            LioRoutingState::Preparing => Some(Arc::clone(&self.probe)),
            LioRoutingState::StartupOdometry { active, .. }
            | LioRoutingState::CloudReplaying { active, .. } => Some(Arc::clone(active)),
            LioRoutingState::DeferredCloud { .. } => None,
            LioRoutingState::Active(ingress) => Some(Arc::clone(ingress)),
            LioRoutingState::Poisoned | LioRoutingState::Closed => None,
        }
    }

    async fn legacy_target(
        &self,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Option<Arc<dyn LioIngressSink>> {
        loop {
            if *shutdown_rx.borrow() {
                return None;
            }
            let changed = self.state_changed.notified();
            match &*self.state.lock() {
                LioRoutingState::Preparing => return Some(Arc::clone(&self.probe)),
                LioRoutingState::StartupOdometry { .. }
                | LioRoutingState::DeferredCloud { .. }
                | LioRoutingState::CloudReplaying { .. } => {}
                LioRoutingState::Active(active) => return Some(Arc::clone(active)),
                LioRoutingState::Poisoned | LioRoutingState::Closed => return None,
            }
            tokio::select! {
                biased;
                _ = shutdown_rx.changed() => return None,
                _ = changed => {}
            }
        }
    }
}

impl CorrectionAwareLioIngress {
    pub(crate) fn device(
        inner: Arc<CommonLioIngress>,
        correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
        correction_applier: Arc<GeometryCanonicalCorrectionApplier>,
    ) -> Self {
        inner
            .bind_relocalization_mode_fence(correction_ingress.relocalization_mode_fence())
            .expect("one CommonLioIngress cannot serve distinct correction mode fences");
        Self {
            inner: CorrectionAwareLioIngressInner::Device(inner),
            correction_ingress,
            correction_applier,
        }
    }

    pub(crate) fn native(
        inner: Arc<NativeLioIngress>,
        correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
        correction_applier: Arc<GeometryCanonicalCorrectionApplier>,
    ) -> Self {
        inner
            .common
            .bind_relocalization_mode_fence(correction_ingress.relocalization_mode_fence())
            .expect("one NativeLioIngress cannot serve distinct correction mode fences");
        Self {
            inner: CorrectionAwareLioIngressInner::Native(inner),
            correction_ingress,
            correction_applier,
        }
    }
}

impl CommonLioIngress {
    pub(crate) fn new(spec: DeviceLioIngressSpec) -> Self {
        Self {
            spec,
            state: Mutex::new(CommonLioIngressState::default()),
            runtime: None,
            odometry_admission: Notify::new(),
            relocalization_mode_fence: OnceLock::new(),
        }
    }

    pub(crate) fn with_runtime(
        spec: DeviceLioIngressSpec,
        handle: GeometryRuntimeHandle,
        public_odometry: (LcmTopic, LcmMessageType),
        public_cloud: (LcmTopic, LcmMessageType),
        tf: (LcmTopic, LcmMessageType),
    ) -> Self {
        Self {
            spec,
            state: Mutex::new(CommonLioIngressState::default()),
            runtime: Some(CommonLioRuntime {
                handle,
                public_odometry,
                public_cloud,
                tf,
            }),
            odometry_admission: Notify::new(),
            relocalization_mode_fence: OnceLock::new(),
        }
    }

    fn bind_relocalization_mode_fence(
        &self,
        fence: Arc<crate::correction_coordinator::RelocalizationModeFence>,
    ) -> Result<(), &'static str> {
        let bound = self
            .relocalization_mode_fence
            .get_or_init(|| Arc::clone(&fence));
        if Arc::ptr_eq(bound, &fence) {
            Ok(())
        } else {
            Err("CommonLioIngress correction mode fence already differs")
        }
    }

    fn capture_relocalization_mode_epoch(&self) -> Option<u64> {
        self.relocalization_mode_fence
            .get()
            .map(|fence| fence.snapshot().epoch)
    }

    fn capture_registration_mode_token(&self) -> RegistrationModeToken {
        RegistrationModeToken {
            frame_capture_relocalization_epoch: self.capture_relocalization_mode_epoch(),
        }
    }

    fn validate_registration_mode_token(
        &self,
        token: RegistrationModeToken,
    ) -> Result<(), LioIngressError> {
        if self.capture_registration_mode_token() == token {
            Ok(())
        } else {
            Err(LioIngressError::StaleRelocalizationMode)
        }
    }

    pub(crate) fn prepare_legacy_odometry(
        &self,
        source_timestamp_ns: i64,
        mut odometry: Odometry,
    ) -> Result<LioOdometryEnvelope, LioIngressError> {
        let mut state = self.state.lock();
        let source_sequence = state
            .next_legacy_odometry_sequence
            .checked_add(1)
            .ok_or(LioIngressError::SequenceExhausted)?;
        let identity = SourceIdentity {
            backend: self.spec.backend,
            runtime_generation: self.spec.runtime_generation,
            source_sequence,
            source_timestamp_ns,
        };
        self.check_identity(&state, identity)?;
        state.prepared_odometry.check(identity)?;
        if odometry.header.frame_id != self.spec.odom_frame_id {
            return Err(LioIngressError::WrongOdometryFrame {
                expected: self.spec.odom_frame_id.clone(),
                actual: odometry.header.frame_id,
            });
        }
        if odometry.child_frame_id != self.spec.body_frame_id {
            return Err(LioIngressError::WrongBodyFrame {
                expected: self.spec.body_frame_id.clone(),
                actual: odometry.child_frame_id,
            });
        }
        odometry.header = source_header(identity, &self.spec.odom_frame_id)?;
        let envelope = LioOdometryEnvelope {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: identity.backend,
            runtime_generation: identity.runtime_generation,
            source_sequence,
            source_timestamp_ns,
            odom_frame_id: self.spec.odom_frame_id.clone(),
            body_frame_id: self.spec.body_frame_id.clone(),
            map_frame_id: self.spec.map_frame_id.clone(),
            odometry,
        };
        encode_lio_odometry_envelope(&envelope)
            .map_err(|error| LioIngressError::EnvelopeEncoding(error.to_string()))?;
        state.prepared_odometry.commit(identity);
        state.next_legacy_odometry_sequence = source_sequence;
        Ok(envelope)
    }

    fn transfer_startup_odometry_cursor(&self, identity: SourceIdentity) -> Result<(), String> {
        let mut state = self.state.lock();
        self.check_identity(&state, identity)
            .map_err(|error| error.to_string())?;
        let accepted_identity = CorrelatedOdometry {
            source_sequence: identity.source_sequence,
            source_timestamp_ns: identity.source_timestamp_ns,
        };
        if state.odometry.latest != Some(accepted_identity) {
            return Err(format!(
                "startup odometry cursor identity mismatch: accepted {:?}, transfer {:?}",
                state.odometry.latest, accepted_identity
            ));
        }
        if state.prepared_odometry.latest.is_some() || state.next_legacy_odometry_sequence != 0 {
            return Err(format!(
                "startup odometry cursor is not fresh: prepared {:?}, legacy sequence {}",
                state.prepared_odometry.latest, state.next_legacy_odometry_sequence
            ));
        }
        state.prepared_odometry.commit(identity);
        state.next_legacy_odometry_sequence = identity.source_sequence;
        Ok(())
    }

    pub(crate) fn reserve_legacy_cloud_identity(
        &self,
        source_timestamp_ns: i64,
    ) -> Result<SourceIdentity, LioIngressError> {
        let mut state = self.state.lock();
        let source_sequence = state
            .next_legacy_cloud_sequence
            .checked_add(1)
            .ok_or(LioIngressError::SequenceExhausted)?;
        let identity = SourceIdentity {
            backend: self.spec.backend,
            runtime_generation: self.spec.runtime_generation,
            source_sequence,
            source_timestamp_ns,
        };
        self.check_identity(&state, identity)?;
        state.prepared_cloud.check(identity)?;
        state.prepared_cloud.commit(identity);
        state.next_legacy_cloud_sequence = source_sequence;
        Ok(identity)
    }

    #[cfg(test)]
    fn accept_odometry(
        &self,
        identity: SourceIdentity,
        adapter_clock_ns: i64,
        mut odometry: Odometry,
    ) -> Result<AcceptedOdometry, LioIngressError> {
        let mut state = self.state.lock();
        self.check_identity(&state, identity)?;
        state.odometry.check(identity)?;
        if odometry.header.frame_id != self.spec.odom_frame_id {
            return Err(LioIngressError::WrongOdometryFrame {
                expected: self.spec.odom_frame_id.clone(),
                actual: odometry.header.frame_id,
            });
        }
        if odometry.child_frame_id != self.spec.body_frame_id {
            return Err(LioIngressError::WrongBodyFrame {
                expected: self.spec.body_frame_id.clone(),
                actual: odometry.child_frame_id,
            });
        }
        odometry.header = source_header(identity, &self.spec.odom_frame_id)?;
        let adapter_timestamp_ns = self.next_adapter_timestamp(&state, adapter_clock_ns)?;
        let envelope = LioOdometryEnvelope {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: identity.backend,
            runtime_generation: identity.runtime_generation,
            source_sequence: identity.source_sequence,
            source_timestamp_ns: identity.source_timestamp_ns,
            odom_frame_id: self.spec.odom_frame_id.clone(),
            body_frame_id: self.spec.body_frame_id.clone(),
            map_frame_id: self.spec.map_frame_id.clone(),
            odometry,
        };
        state.odometry.commit(identity);
        state.latest_adapter_timestamp_ns = Some(adapter_timestamp_ns);
        Ok(AcceptedOdometry {
            envelope,
            adapter_timestamp_ns,
        })
    }

    #[cfg(test)]
    fn accept_odometry_for_test(
        &self,
        identity: SourceIdentity,
        adapter_clock_ns: i64,
        odometry: Odometry,
    ) -> Result<AcceptedOdometry, LioIngressError> {
        let accepted = self.accept_odometry(identity, adapter_clock_ns, odometry)?;
        let correlated_identity = CorrelatedOdometry {
            source_sequence: accepted.envelope.source_sequence,
            source_timestamp_ns: accepted.envelope.source_timestamp_ns,
        };
        let frozen_odom_from_body = odometry_pose_isometry(&accepted.envelope.odometry)?;
        let mut state = self.state.lock();
        state
            .accepted_odometry_identities
            .push_back(correlated_identity);
        state.accepted_odometry_poses.insert(
            (
                correlated_identity.source_sequence,
                correlated_identity.source_timestamp_ns,
            ),
            frozen_odom_from_body,
        );
        while state.accepted_odometry_identities.len() > MAX_ACCEPTED_ODOMETRY_IDENTITIES {
            if let Some(expired) = state.accepted_odometry_identities.pop_front() {
                state
                    .accepted_odometry_poses
                    .remove(&(expired.source_sequence, expired.source_timestamp_ns));
            }
        }
        Ok(accepted)
    }

    pub(crate) fn accept_public_odometry(
        &self,
        envelope: LioOdometryEnvelope,
        adapter_clock_ns: i64,
    ) -> Result<AcceptedOdometry, LioIngressError> {
        let identity = SourceIdentity::from_odometry(&envelope);
        let mut state = self.state.lock();
        self.check_identity(&state, identity)?;
        state.odometry.check(identity)?;
        self.check_odometry_envelope(&envelope)?;
        encode_lio_odometry_envelope(&envelope)
            .map_err(|error| LioIngressError::EnvelopeEncoding(error.to_string()))?;
        let frozen_odom_from_body = odometry_pose_isometry(&envelope.odometry)?;
        let adapter_timestamp_ns = self.next_adapter_timestamp(&state, adapter_clock_ns)?;
        state.odometry.commit(identity);
        let correlated_identity = CorrelatedOdometry {
            source_sequence: identity.source_sequence,
            source_timestamp_ns: identity.source_timestamp_ns,
        };
        state
            .accepted_odometry_identities
            .push_back(correlated_identity);
        state.accepted_odometry_poses.insert(
            (
                correlated_identity.source_sequence,
                correlated_identity.source_timestamp_ns,
            ),
            frozen_odom_from_body,
        );
        while state.accepted_odometry_identities.len() > MAX_ACCEPTED_ODOMETRY_IDENTITIES {
            if let Some(expired) = state.accepted_odometry_identities.pop_front() {
                state
                    .accepted_odometry_poses
                    .remove(&(expired.source_sequence, expired.source_timestamp_ns));
            }
        }
        state.latest_adapter_timestamp_ns = Some(adapter_timestamp_ns);
        drop(state);
        self.odometry_admission.notify_waiters();
        Ok(AcceptedOdometry {
            envelope,
            adapter_timestamp_ns,
        })
    }

    pub(crate) async fn wait_for_public_odometry(
        &self,
        identity: CorrelatedOdometry,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> bool {
        loop {
            if *shutdown_rx.borrow() {
                return false;
            }
            let notified = self.odometry_admission.notified();
            if self
                .state
                .lock()
                .accepted_odometry_identities
                .contains(&identity)
            {
                return true;
            }
            tokio::select! {
                biased;
                _ = shutdown_rx.changed() => return false,
                _ = notified => {}
            }
        }
    }

    #[allow(dead_code)]
    pub(crate) fn register_cloud(
        &self,
        identity: SourceIdentity,
        correlated_odometry: CorrelatedOdometry,
        adapter_clock_ns: i64,
        cloud: PointCloud2,
        adapter: &MappingAdapter,
    ) -> Result<PreparedRegisteredCloud, LioIngressError> {
        let registration_mode_token = self.capture_registration_mode_token();
        self.register_cloud_with_mode_token(
            identity,
            correlated_odometry,
            adapter_clock_ns,
            cloud,
            adapter,
            registration_mode_token,
        )
    }

    fn register_cloud_with_mode_token(
        &self,
        identity: SourceIdentity,
        correlated_odometry: CorrelatedOdometry,
        adapter_clock_ns: i64,
        cloud: PointCloud2,
        adapter: &MappingAdapter,
        registration_mode_token: RegistrationModeToken,
    ) -> Result<PreparedRegisteredCloud, LioIngressError> {
        let source_points = parse_odom_xyz_cloud(&cloud, &self.spec)?;
        let source_point_count = source_points.len();
        self.register_odom_points_with_mode_token(
            identity,
            correlated_odometry,
            adapter_clock_ns,
            source_point_count,
            source_points,
            adapter,
            registration_mode_token,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[allow(dead_code)]
    pub(crate) fn register_odom_points(
        &self,
        identity: SourceIdentity,
        correlated_odometry: CorrelatedOdometry,
        _adapter_clock_ns: i64,
        source_point_count: usize,
        odom_points: Vec<Point3<f64>>,
        adapter: &MappingAdapter,
    ) -> Result<PreparedRegisteredCloud, LioIngressError> {
        let registration_mode_token = self.capture_registration_mode_token();
        self.register_odom_points_with_mode_token(
            identity,
            correlated_odometry,
            _adapter_clock_ns,
            source_point_count,
            odom_points,
            adapter,
            registration_mode_token,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn register_odom_points_with_mode_token(
        &self,
        identity: SourceIdentity,
        correlated_odometry: CorrelatedOdometry,
        _adapter_clock_ns: i64,
        source_point_count: usize,
        odom_points: Vec<Point3<f64>>,
        adapter: &MappingAdapter,
        registration_mode_token: RegistrationModeToken,
    ) -> Result<PreparedRegisteredCloud, LioIngressError> {
        let source_point_count_i64 =
            i64::try_from(source_point_count).map_err(|_| LioIngressError::PointCountOverflow)?;
        let emitted_point_count_i64 =
            i64::try_from(odom_points.len()).map_err(|_| LioIngressError::PointCountOverflow)?;
        if source_point_count_i64 <= 0
            || source_point_count_i64 > MAX_LIO_SOURCE_POINT_COUNT
            || emitted_point_count_i64 <= 0
            || emitted_point_count_i64 > MAX_LIO_EMITTED_POINT_COUNT
            || emitted_point_count_i64 > source_point_count_i64
        {
            return Err(LioIngressError::InvalidPointBudget {
                source_count: source_point_count_i64,
                emitted_count: emitted_point_count_i64,
            });
        }
        let mut state = self.state.lock();
        // Production callers capture the token before waiting for this
        // lifecycle guard and pass it through unchanged. Validate it while
        // the guard is held, before creating a pending registration permit.
        self.validate_registration_mode_token(registration_mode_token)?;
        self.check_identity(&state, identity)?;
        state.cloud.check(identity)?;
        if correlated_odometry.source_timestamp_ns > identity.source_timestamp_ns {
            return Err(LioIngressError::NonCausalOdometryCorrelation {
                cloud_timestamp_ns: identity.source_timestamp_ns,
                odometry_timestamp_ns: correlated_odometry.source_timestamp_ns,
            });
        }
        let frozen_odom_from_body = state
            .accepted_odometry_poses
            .get(&(
                correlated_odometry.source_sequence,
                correlated_odometry.source_timestamp_ns,
            ))
            .cloned()
            .ok_or(LioIngressError::MissingFrozenOdometry)?;
        let registered = adapter
            .register_odom_cloud_to_map(&odom_points, &self.spec.odom_frame_id)
            .map_err(|error| LioIngressError::Registration(error.to_string()))?;
        if registered.points.len() != odom_points.len() {
            return Err(LioIngressError::RegistrationPointCountMismatch {
                source_points: odom_points.len(),
                registered: registered.points.len(),
            });
        }
        let rebuilt_cloud = rebuild_xyz_cloud(
            &registered.points,
            source_header(identity, &self.spec.map_frame_id)?,
        )?;
        let emitted_point_count = i64::try_from(registered.points.len())
            .map_err(|_| LioIngressError::PointCountOverflow)?;
        let envelope = LioCloudEnvelope {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: identity.backend,
            runtime_generation: identity.runtime_generation,
            source_sequence: identity.source_sequence,
            source_timestamp_ns: identity.source_timestamp_ns,
            correlated_odometry_source_sequence: correlated_odometry.source_sequence,
            correlated_odometry_source_timestamp_ns: correlated_odometry.source_timestamp_ns,
            sensor_frame_id: self.spec.sensor_frame_id.clone(),
            body_frame_id: self.spec.body_frame_id.clone(),
            odom_frame_id: self.spec.odom_frame_id.clone(),
            map_frame_id: self.spec.map_frame_id.clone(),
            registration_state: LIO_REGISTRATION_REGISTERED_MAP,
            source_point_count: source_point_count_i64,
            emitted_point_count,
            cloud: rebuilt_cloud,
        };
        let encoded = encode_lio_cloud_envelope(&envelope)
            .map_err(|error| LioIngressError::EnvelopeEncoding(error.to_string()))?;
        let envelope_sha256 = crate::map_checksum::sha256_bytes(&encoded);
        let mapping_snapshot = adapter.snapshot();
        let base_anchor = adapter
            .map_odom_transform_snapshot()
            .ok_or(LioIngressError::MissingBaseAnchor)?;
        let base_anchor_map_from_odom = Isometry3::from_parts(
            Translation3::new(
                base_anchor.translation[0],
                base_anchor.translation[1],
                base_anchor.translation[2],
            ),
            UnitQuaternion::from_euler_angles(0.0, 0.0, base_anchor.yaw_radians),
        );
        let binding = RegistrationBinding {
            runtime_generation: identity.runtime_generation,
            cloud_source_sequence: identity.source_sequence,
            correlated_odometry,
            mapping_session_id: mapping_snapshot.session_id,
            mapping_session_generation: registered.session_generation,
            map_frame_generation: registered.frame_generation,
            envelope_sha256,
            frozen_odom_from_body,
            base_anchor_map_from_odom,
            frame_capture_relocalization_epoch: registration_mode_token
                .frame_capture_relocalization_epoch,
        };
        let key = (identity.runtime_generation, identity.source_sequence);
        if state.pending_registrations.contains_key(&key) {
            return Err(LioIngressError::DuplicatePreparedCloud {
                generation: key.0,
                sequence: key.1,
            });
        }
        while state.pending_registrations.len() >= MAX_PENDING_REGISTRATIONS {
            let oldest = state
                .pending_registrations
                .keys()
                .min()
                .copied()
                .expect("capacity check requires an existing registration");
            state.pending_registrations.remove(&oldest);
        }
        state.pending_registrations.insert(
            key,
            PendingRegistration {
                binding,
                map_points: registered.points,
            },
        );
        state.next_legacy_cloud_sequence = state
            .next_legacy_cloud_sequence
            .max(identity.source_sequence);
        Ok(PreparedRegisteredCloud {
            envelope,
            envelope_sha256,
        })
    }

    pub(crate) fn accept_public_cloud(
        &self,
        envelope: LioCloudEnvelope,
        encoded_full_bytes: &[u8],
        adapter_clock_ns: i64,
    ) -> Result<AcceptedRegisteredCloud, LioIngressError> {
        let identity = SourceIdentity::from_cloud(&envelope);
        let mut state = self.state.lock();
        self.check_identity(&state, identity)?;
        state.cloud.check(identity)?;
        self.check_cloud_envelope(&envelope)?;
        let correlated_odometry = CorrelatedOdometry {
            source_sequence: envelope.correlated_odometry_source_sequence,
            source_timestamp_ns: envelope.correlated_odometry_source_timestamp_ns,
        };
        let canonical = encode_lio_cloud_envelope(&envelope)
            .map_err(|error| LioIngressError::EnvelopeEncoding(error.to_string()))?;
        if canonical != encoded_full_bytes {
            return Err(LioIngressError::NonCanonicalEnvelope);
        }
        let envelope_sha256 = crate::map_checksum::sha256_bytes(encoded_full_bytes);
        let key = (identity.runtime_generation, identity.source_sequence);
        let pending_binding = state
            .pending_registrations
            .get(&key)
            .ok_or(LioIngressError::MissingRegistrationPermit)?
            .binding
            .clone();
        if pending_binding.runtime_generation != identity.runtime_generation
            || pending_binding.cloud_source_sequence != identity.source_sequence
            || pending_binding.correlated_odometry != correlated_odometry
            || pending_binding.envelope_sha256 != envelope_sha256
        {
            return Err(LioIngressError::PermitBindingMismatch);
        }
        if let Some(runtime) = &self.runtime {
            let adapter = runtime
                .handle
                .mapping_adapter
                .as_ref()
                .ok_or_else(|| {
                    LioIngressError::Registration("mapping adapter is unavailable".into())
                })?
                .lock();
            if adapter.snapshot().session_generation != pending_binding.mapping_session_generation
                || adapter.global_frame_generation() != pending_binding.map_frame_generation
            {
                return Err(LioIngressError::StaleMappingGeneration);
            }
        }
        let adapter_timestamp_ns = self.next_adapter_timestamp(&state, adapter_clock_ns)?;
        let pending = state
            .pending_registrations
            .remove(&key)
            .expect("checked pending registration remains under ingress lock");
        state
            .pending_registrations
            .retain(|(generation, sequence), _| {
                *generation != identity.runtime_generation || *sequence > identity.source_sequence
            });
        state.cloud.commit(identity);
        state.latest_adapter_timestamp_ns = Some(adapter_timestamp_ns);
        Ok(AcceptedRegisteredCloud {
            envelope,
            map_points: pending.map_points,
            adapter_timestamp_ns,
            binding: pending.binding,
        })
    }

    pub(crate) fn revoke_prepared_cloud(
        &self,
        runtime_generation: i64,
        source_sequence: i64,
        envelope_sha256: [u8; 32],
    ) -> bool {
        let mut state = self.state.lock();
        let key = (runtime_generation, source_sequence);
        let matches = state
            .pending_registrations
            .get(&key)
            .is_some_and(|pending| pending.binding.envelope_sha256 == envelope_sha256);
        if matches {
            state.pending_registrations.remove(&key);
        }
        matches
    }

    pub(crate) fn close(&self) {
        let mut state = self.state.lock();
        state.closed = true;
        state.pending_registrations.clear();
    }

    pub(crate) fn snapshot(&self) -> CommonLioIngressSnapshot {
        let state = self.state.lock();
        CommonLioIngressSnapshot {
            runtime_generation: self.spec.runtime_generation,
            closed: state.closed,
            latest_odometry: state.odometry.latest,
            latest_cloud: state.cloud.latest,
            latest_adapter_timestamp_ns: state.latest_adapter_timestamp_ns,
        }
    }

    fn latest_causal_odometry(&self, cloud_timestamp_ns: i64) -> Option<CorrelatedOdometry> {
        self.state
            .lock()
            .accepted_odometry_identities
            .iter()
            .rev()
            .find(|odometry| odometry.source_timestamp_ns <= cloud_timestamp_ns)
            .copied()
    }

    fn check_identity(
        &self,
        state: &CommonLioIngressState,
        identity: SourceIdentity,
    ) -> Result<(), LioIngressError> {
        if state.closed {
            return Err(LioIngressError::Closed);
        }
        if identity.backend != self.spec.backend {
            return Err(LioIngressError::WrongBackend(identity.backend));
        }
        if identity.runtime_generation != self.spec.runtime_generation {
            return Err(LioIngressError::WrongGeneration {
                expected: self.spec.runtime_generation,
                actual: identity.runtime_generation,
            });
        }
        Ok(())
    }

    fn next_adapter_timestamp(
        &self,
        state: &CommonLioIngressState,
        adapter_clock_ns: i64,
    ) -> Result<i64, LioIngressError> {
        if adapter_clock_ns < 0 {
            return Err(LioIngressError::InvalidAdapterTimestamp(adapter_clock_ns));
        }
        let after_previous = match state.latest_adapter_timestamp_ns {
            Some(previous) => previous
                .checked_add(1)
                .ok_or(LioIngressError::AdapterTimestampExhausted)?,
            None => 1,
        };
        Ok(adapter_clock_ns.max(after_previous))
    }

    fn check_odometry_envelope(
        &self,
        envelope: &LioOdometryEnvelope,
    ) -> Result<(), LioIngressError> {
        if envelope.schema_version != LIO_SCHEMA_VERSION_V1
            || envelope.odom_frame_id != self.spec.odom_frame_id
            || envelope.body_frame_id != self.spec.body_frame_id
            || envelope.map_frame_id != self.spec.map_frame_id
            || envelope.odometry.header.frame_id != self.spec.odom_frame_id
            || envelope.odometry.child_frame_id != self.spec.body_frame_id
        {
            return Err(LioIngressError::EnvelopeFrameMismatch);
        }
        Ok(())
    }

    fn check_cloud_envelope(&self, envelope: &LioCloudEnvelope) -> Result<(), LioIngressError> {
        if envelope.schema_version != LIO_SCHEMA_VERSION_V1
            || envelope.sensor_frame_id != self.spec.sensor_frame_id
            || envelope.body_frame_id != self.spec.body_frame_id
            || envelope.odom_frame_id != self.spec.odom_frame_id
            || envelope.map_frame_id != self.spec.map_frame_id
            || envelope.registration_state != LIO_REGISTRATION_REGISTERED_MAP
            || envelope.cloud.header.frame_id != self.spec.map_frame_id
        {
            return Err(LioIngressError::EnvelopeFrameMismatch);
        }
        Ok(())
    }
}

impl NativeLioIngress {
    pub(crate) fn bind_relocalization_mode_fence(
        &self,
        fence: Arc<crate::correction_coordinator::RelocalizationModeFence>,
    ) -> Result<(), &'static str> {
        self.common.bind_relocalization_mode_fence(fence)
    }

    pub(crate) fn without_runtime(
        spec: DeviceLioIngressSpec,
        body_from_sensor: BodyFromSensorTransform,
        pair_max_skew: Duration,
    ) -> Result<Self, LioIngressError> {
        if spec.backend == LIO_BACKEND_DEVICE {
            return Err(LioIngressError::UnsupportedBackend(spec.backend));
        }
        let pair_max_skew_ns = i64::try_from(pair_max_skew.as_nanos())
            .map_err(|_| LioIngressError::PairSkewOverflow)?;
        if pair_max_skew_ns <= 0 {
            return Err(LioIngressError::PairSkewOverflow);
        }
        let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
        let pair_state = NativePairState {
            probe_phase: NativeProbePhase::Probing,
            ..NativePairState::default()
        };
        Ok(Self {
            common: CommonLioIngress::new(spec),
            body_from_sensor,
            pair_max_skew_ns,
            pair_state: Mutex::new(pair_state),
            source_pair_ready_tx,
            source_pair_ready_rx,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn with_runtime(
        spec: DeviceLioIngressSpec,
        handle: GeometryRuntimeHandle,
        public_odometry: (LcmTopic, LcmMessageType),
        public_cloud: (LcmTopic, LcmMessageType),
        tf: (LcmTopic, LcmMessageType),
        body_from_sensor: BodyFromSensorTransform,
        pair_max_skew: Duration,
    ) -> Result<Self, LioIngressError> {
        if spec.backend == LIO_BACKEND_DEVICE {
            return Err(LioIngressError::UnsupportedBackend(spec.backend));
        }
        let pair_max_skew_ns = i64::try_from(pair_max_skew.as_nanos())
            .map_err(|_| LioIngressError::PairSkewOverflow)?;
        if pair_max_skew_ns <= 0 {
            return Err(LioIngressError::PairSkewOverflow);
        }
        let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
        Ok(Self {
            common: CommonLioIngress::with_runtime(spec, handle, public_odometry, public_cloud, tf),
            body_from_sensor,
            pair_max_skew_ns,
            pair_state: Mutex::new(NativePairState::default()),
            source_pair_ready_tx,
            source_pair_ready_rx,
        })
    }

    fn insert_odometry(&self, sample: NativeOdometrySample) -> Option<NativeSourcePair> {
        let mut state = self.pair_state.lock();
        state.odometry.push_back(sample);
        while state.odometry.len() > MAX_NATIVE_PAIR_CACHE {
            state.odometry.pop_front();
        }
        take_best_native_pair(&mut state, self.pair_max_skew_ns)
    }

    fn insert_cloud(&self, sample: NativeCloudSample) -> Option<NativeSourcePair> {
        let mut state = self.pair_state.lock();
        state.clouds.push_back(sample);
        while state.clouds.len() > MAX_NATIVE_PAIR_CACHE {
            state.clouds.pop_front();
        }
        take_best_native_pair(&mut state, self.pair_max_skew_ns)
    }

    async fn process_pair(
        &self,
        pair: NativeSourcePair,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let odom_points = transform_native_pair(&pair, &self.body_from_sensor)?;
        let frozen_odom_from_body =
            odometry_pose_isometry(&pair.odometry.odometry).map_err(|error| error.to_string())?;
        let odometry = LioOdometryEnvelope {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: pair.odometry.identity.backend,
            runtime_generation: pair.odometry.identity.runtime_generation,
            source_sequence: pair.odometry.identity.source_sequence,
            source_timestamp_ns: pair.odometry.identity.source_timestamp_ns,
            odom_frame_id: self.common.spec.odom_frame_id.clone(),
            body_frame_id: self.common.spec.body_frame_id.clone(),
            map_frame_id: self.common.spec.map_frame_id.clone(),
            odometry: pair.odometry.odometry.clone(),
        };
        let encoded_odometry =
            encode_lio_odometry_envelope(&odometry).map_err(|error| error.to_string())?;
        let retained = ValidatedStartupPair {
            odometry_identity: pair.odometry.identity,
            cloud_identity: pair.cloud.identity,
            registration_mode_token: Some(pair.cloud.registration_mode_token),
            odometry: Box::new(odometry),
            frozen_odom_from_body,
            registered_points_sha256: point_set_sha256(&odom_points),
            registered_odom_points: Arc::from(odom_points.clone()),
            source_point_count: pair.cloud.source_point_count,
            emitted_point_count: odom_points.len(),
            odometry_payload_sha256: crate::map_checksum::sha256_bytes(&encoded_odometry),
        };
        let probe_only = {
            let mut state = self.pair_state.lock();
            match state.probe_phase {
                NativeProbePhase::Probing => {
                    state.probe_phase = NativeProbePhase::Retained(Box::new(Some(retained)));
                    true
                }
                NativeProbePhase::Retained(_)
                | NativeProbePhase::Transferred
                | NativeProbePhase::Closed => return Ok(()),
                NativeProbePhase::Runtime => false,
            }
        };
        self.source_pair_ready_tx.send_replace(true);
        if probe_only {
            return Ok(());
        }

        let correlated_odometry = CorrelatedOdometry {
            source_sequence: pair.odometry.identity.source_sequence,
            source_timestamp_ns: pair.odometry.identity.source_timestamp_ns,
        };
        let odometry_admitted = tokio::time::timeout(
            LIO_GENERATED_PUBLISH_TIMEOUT,
            self.common
                .wait_for_public_odometry(correlated_odometry, shutdown_rx),
        )
        .await
        .map_err(|_| "native LIO public odometry admission timed out".to_string())?;
        if !odometry_admitted {
            return Ok(());
        }

        let runtime = self.common.runtime.as_ref().ok_or_else(runtime_not_bound)?;
        let Some(lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
            return Ok(());
        };
        let mapping = runtime
            .handle
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| "mapping adapter is unavailable".to_string())?;
        let prepared = {
            let adapter = mapping.lock();
            if !adapter.snapshot().active {
                return Ok(());
            }
            self.common
                .register_odom_points_with_mode_token(
                    pair.cloud.identity,
                    correlated_odometry,
                    0,
                    pair.cloud.source_point_count,
                    odom_points,
                    &adapter,
                    pair.cloud.registration_mode_token,
                )
                .map_err(|error| error.to_string())?
        };
        drop(lifecycle_guard);
        let encoded =
            encode_lio_cloud_envelope(&prepared.envelope).map_err(|error| error.to_string())?;
        let publish_result = publish_generated_body(
            &runtime.handle,
            &runtime.public_cloud,
            prepared.envelope.source_sequence,
            encoded,
        )
        .await;
        if publish_result.is_err() {
            self.common.revoke_prepared_cloud(
                prepared.envelope.runtime_generation,
                prepared.envelope.source_sequence,
                prepared.envelope_sha256,
            );
        }
        publish_result
    }

    pub(crate) fn take_validated_startup_pair(&self) -> Result<ValidatedStartupPair, String> {
        let mut state = self.pair_state.lock();
        let retained = match &mut state.probe_phase {
            NativeProbePhase::Retained(retained) => retained
                .take()
                .ok_or_else(|| "native LIO startup pair was already transferred".to_owned())?,
            NativeProbePhase::Probing => {
                return Err("native LIO startup pair is not ready".to_owned())
            }
            NativeProbePhase::Transferred => {
                return Err("native LIO startup pair was already transferred".to_owned())
            }
            NativeProbePhase::Runtime => {
                return Err("runtime-bound native LIO cannot yield a startup pair".to_owned())
            }
            NativeProbePhase::Closed => return Err("native LIO startup probe is closed".to_owned()),
        };
        state.probe_phase = NativeProbePhase::Transferred;
        state.odometry.clear();
        state.clouds.clear();
        Ok(retained)
    }

    fn install_public_expectation(
        &self,
        kind: NativeReplayFrameKind,
        identity: SourceIdentity,
        binding: &(LcmTopic, LcmMessageType),
        payload: &[u8],
    ) -> Result<(u64, oneshot::Receiver<Result<(), String>>), String> {
        let mut state = self.pair_state.lock();
        if !matches!(state.probe_phase, NativeProbePhase::Runtime) {
            return Err("native LIO replay requires a runtime-bound ingress".to_owned());
        }
        match &state.public_replay {
            NativePublicReplayPhase::Normal => {}
            NativePublicReplayPhase::Poisoned(detail) => return Err(detail.clone()),
            NativePublicReplayPhase::Closed => {
                return Err("native LIO replay ingress is closed".to_owned())
            }
            _ => return Err("native LIO already has an exact public replay in flight".to_owned()),
        }
        state.next_replay_nonce = state
            .next_replay_nonce
            .checked_add(1)
            .ok_or_else(|| "native LIO replay nonce exhausted".to_owned())?;
        let nonce = state.next_replay_nonce;
        let (outcome, outcome_rx) = oneshot::channel();
        state.public_replay = NativePublicReplayPhase::Expecting(NativeReplayExpectation {
            nonce,
            kind,
            identity,
            topic: binding.0.clone(),
            message_type: binding.1.clone(),
            payload_sha256: crate::map_checksum::sha256_bytes(payload),
            outcome: Some(outcome),
        });
        Ok((nonce, outcome_rx))
    }

    fn classify_public_frame(
        &self,
        kind: NativeReplayFrameKind,
        frame: &RawLcmFrame,
    ) -> Result<NativePublicFrameDecision, String> {
        let mut state = self.pair_state.lock();
        let expectation = match &mut state.public_replay {
            NativePublicReplayPhase::Normal => return Ok(NativePublicFrameDecision::Normal),
            NativePublicReplayPhase::Expecting(expectation) => expectation,
            NativePublicReplayPhase::Claimed { .. } | NativePublicReplayPhase::Complete { .. } => {
                return Ok(NativePublicFrameDecision::Ignore)
            }
            NativePublicReplayPhase::Poisoned(detail) => return Err(detail.clone()),
            NativePublicReplayPhase::Closed => return Ok(NativePublicFrameDecision::Ignore),
        };
        let payload_sha256 = crate::map_checksum::sha256_bytes(frame.payload.as_ref());
        if payload_sha256 != expectation.payload_sha256 {
            match decode_public_replay_identity(kind, frame.payload.as_ref()) {
                Ok(identity)
                    if identity.source_sequence != expectation.identity.source_sequence =>
                {
                    return Ok(NativePublicFrameDecision::Ignore);
                }
                Err(_)
                    if frame.header.seqno != 0
                        && i64::from(frame.header.seqno)
                            != expectation.identity.source_sequence =>
                {
                    return Ok(NativePublicFrameDecision::Ignore);
                }
                Ok(_) | Err(_) => {}
            }
        }
        let mismatch = if expectation.kind != kind {
            Some("public replay frame kind mismatch")
        } else if expectation.topic != frame.topic {
            Some("public replay topic mismatch")
        } else if expectation.message_type != frame.msg_type {
            Some("public replay message type mismatch")
        } else if expectation.payload_sha256 != payload_sha256 {
            Some("public replay payload hash mismatch")
        } else {
            None
        };
        if let Some(detail) = mismatch {
            let detail = detail.to_owned();
            let outcome = expectation.outcome.take();
            state.public_replay = NativePublicReplayPhase::Poisoned(detail.clone());
            if let Some(outcome) = outcome {
                let _ = outcome.send(Err(detail.clone()));
            }
            return Err(detail);
        }
        let claim = NativeReplayClaim {
            kind,
            nonce: expectation.nonce,
        };
        let outcome = expectation.outcome.take();
        state.public_replay = NativePublicReplayPhase::Claimed {
            kind,
            nonce: claim.nonce,
            outcome,
        };
        Ok(NativePublicFrameDecision::Claim(claim))
    }

    fn complete_public_expectation(&self, claim: NativeReplayClaim, result: &Result<(), String>) {
        let mut state = self.pair_state.lock();
        let previous = std::mem::replace(
            &mut state.public_replay,
            NativePublicReplayPhase::Poisoned(
                "native LIO public replay completion state mismatch".to_owned(),
            ),
        );
        match previous {
            NativePublicReplayPhase::Claimed {
                kind,
                nonce,
                outcome,
            } if kind == claim.kind && nonce == claim.nonce => {
                if let Some(outcome) = outcome {
                    let _ = outcome.send(result.clone());
                }
                state.public_replay = match result {
                    Ok(()) => NativePublicReplayPhase::Complete { nonce },
                    Err(error) => NativePublicReplayPhase::Poisoned(error.clone()),
                };
            }
            NativePublicReplayPhase::Poisoned(detail) => {
                state.public_replay = NativePublicReplayPhase::Poisoned(detail);
            }
            _ => {}
        }
    }

    fn finish_public_expectation(&self, nonce: u64) -> Result<(), String> {
        let mut state = self.pair_state.lock();
        match &state.public_replay {
            NativePublicReplayPhase::Complete { nonce: actual } if *actual == nonce => {
                state.public_replay = NativePublicReplayPhase::Normal;
                Ok(())
            }
            NativePublicReplayPhase::Poisoned(detail) => Err(detail.clone()),
            NativePublicReplayPhase::Closed => {
                Err("native LIO replay ingress is closed".to_owned())
            }
            _ => Err("native LIO public replay did not complete exactly once".to_owned()),
        }
    }

    fn poison_public_replay(&self, detail: impl Into<String>) {
        let detail = detail.into();
        let mut state = self.pair_state.lock();
        let previous = std::mem::replace(
            &mut state.public_replay,
            NativePublicReplayPhase::Poisoned(detail.clone()),
        );
        if let NativePublicReplayPhase::Expecting(mut expectation) = previous {
            if let Some(outcome) = expectation.outcome.take() {
                let _ = outcome.send(Err(detail));
            }
        } else if let NativePublicReplayPhase::Claimed {
            outcome: Some(outcome),
            ..
        } = previous
        {
            let _ = outcome.send(Err(detail));
        }
    }
}

fn decode_public_replay_identity(
    kind: NativeReplayFrameKind,
    payload: &[u8],
) -> Result<SourceIdentity, String> {
    match kind {
        NativeReplayFrameKind::Odometry => decode_body(
            nuwax_codec::lcm::lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
            payload,
            nuwax_codec::lcm::lio::decode_lio_odometry_envelope,
        )
        .map(|envelope| SourceIdentity::from_odometry(&envelope)),
        NativeReplayFrameKind::Cloud => decode_body(
            nuwax_codec::lcm::lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1,
            payload,
            nuwax_codec::lcm::lio::decode_lio_cloud_envelope,
        )
        .map(|envelope| SourceIdentity::from_cloud(&envelope)),
    }
}

impl NativeDeferredReplayControl {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        ingress: Arc<NativeLioIngress>,
        gate: Arc<LioIngressRoutingGate>,
        correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
        correction_applier: Arc<GeometryCanonicalCorrectionApplier>,
        correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
        token: LioReplayToken,
        pair: ValidatedStartupPair,
    ) -> Arc<Self> {
        let (poison_tx, _) = watch::channel(None);
        Arc::new(Self {
            ingress,
            gate,
            correction_ingress,
            correction_applier,
            correction_phase_a,
            token,
            phase: Mutex::new(NativeDeferredReplayPhase::Startup(Some(pair))),
            stop_requested: AtomicBool::new(false),
            readiness: crate::server::ReadinessAuthority::new(),
            poison_tx,
            poison_diagnostics: Mutex::new(Vec::new()),
        })
    }

    pub(crate) fn readiness_authority(&self) -> crate::server::ReadinessAuthority {
        self.readiness.clone()
    }

    pub(crate) async fn publish_startup_odometry(&self) -> Result<(), String> {
        let pair = {
            let phase = self.phase.lock();
            match &*phase {
                NativeDeferredReplayPhase::Startup(Some(pair)) => pair.clone(),
                NativeDeferredReplayPhase::Poisoned(detail) => return Err(detail.clone()),
                NativeDeferredReplayPhase::Closed => {
                    return Err("native deferred replay closed before startup odometry".to_owned())
                }
                _ => return Err("native startup odometry transaction is not pending".to_owned()),
            }
        };
        validate_startup_pair(&pair)?;
        let runtime = self
            .ingress
            .common
            .runtime
            .as_ref()
            .ok_or_else(runtime_not_bound)?;
        let encoded =
            encode_lio_odometry_envelope(&pair.odometry).map_err(|error| error.to_string())?;
        let payload = encoded
            .get(8..)
            .ok_or_else(|| "startup odometry encoding lacks a fingerprint".to_owned())?;
        let (nonce, outcome_rx) = self.ingress.install_public_expectation(
            NativeReplayFrameKind::Odometry,
            pair.odometry_identity,
            &runtime.public_odometry,
            payload,
        )?;
        if let Err(error) = publish_generated_body(
            &runtime.handle,
            &runtime.public_odometry,
            pair.odometry_identity.source_sequence,
            encoded,
        )
        .await
        {
            let detail = format!("startup odometry publication outcome is unknown: {error}");
            self.poison(detail.clone());
            return Err(detail);
        }
        let outcome = match tokio::time::timeout(LIO_GENERATED_PUBLISH_TIMEOUT, outcome_rx).await {
            Ok(Ok(outcome)) => outcome,
            Ok(Err(_)) => {
                let detail = "startup odometry post-handler ACK channel closed".to_owned();
                self.poison(detail.clone());
                return Err(detail);
            }
            Err(_) => {
                let detail = "startup odometry post-handler ACK timed out".to_owned();
                self.poison(detail.clone());
                return Err(detail);
            }
        };
        if let Err(error) = outcome {
            let detail = format!("startup odometry post-handler ACK failed: {error}");
            self.poison(detail.clone());
            return Err(detail);
        }
        if let Err(error) = self.ingress.finish_public_expectation(nonce) {
            self.poison(error.clone());
            return Err(error);
        }
        if let Err(error) = self.gate.finish_startup_odometry(self.token) {
            self.poison(error.clone());
            return Err(error);
        }
        let mut phase = self.phase.lock();
        let previous = std::mem::replace(&mut *phase, NativeDeferredReplayPhase::Closed);
        match previous {
            NativeDeferredReplayPhase::Startup(Some(pair)) => {
                let startup_identity = pair.odometry_identity;
                *phase = NativeDeferredReplayPhase::Deferred(Some(pair));
                drop(phase);
                if let Err(error) = self
                    .ingress
                    .common
                    .transfer_startup_odometry_cursor(startup_identity)
                {
                    let detail = format!("startup odometry cursor transfer failed: {error}");
                    self.poison(detail.clone());
                    return Err(detail);
                }
                Ok(())
            }
            NativeDeferredReplayPhase::Poisoned(detail) => {
                *phase = NativeDeferredReplayPhase::Poisoned(detail.clone());
                Err(detail)
            }
            other => {
                *phase = other;
                drop(phase);
                let detail = "startup odometry completed against a stale replay phase".to_owned();
                self.poison(detail.clone());
                Err(detail)
            }
        }
    }

    pub(crate) fn begin_phase_a(&self) {
        self.stop_requested.store(true, Ordering::Release);
        let mut phase = self.phase.lock();
        if matches!(
            *phase,
            NativeDeferredReplayPhase::Startup(_) | NativeDeferredReplayPhase::Deferred(_)
        ) {
            *phase = NativeDeferredReplayPhase::Closed;
        }
    }

    pub(crate) fn prepare_session(
        &self,
        snapshot: &crate::mapping_adapter::MappingSessionSnapshot,
        map_frame_generation: u64,
        adapter: &MappingAdapter,
    ) -> Result<DeferredSessionClaim, String> {
        if self.stop_requested.load(Ordering::Acquire) || self.gate.is_closed() {
            return Err("native deferred replay is stopping".to_owned());
        }
        let identity = DeferredMappingIdentity {
            session_id: snapshot.session_id.clone(),
            map_name: snapshot.map_name.clone(),
            session_generation: snapshot.session_generation,
            map_frame_generation,
        };
        if !snapshot.active
            || identity.session_id.is_empty()
            || identity.map_name.is_empty()
            || identity.session_generation == 0
        {
            return Err(
                "native deferred replay received an incomplete Mapping identity".to_owned(),
            );
        }
        let adapter_snapshot = adapter.snapshot();
        if !adapter_snapshot.active
            || adapter_snapshot.session_id != identity.session_id
            || adapter_snapshot.map_name != identity.map_name
            || adapter_snapshot.session_generation != identity.session_generation
            || adapter.global_frame_generation() != identity.map_frame_generation
        {
            return Err("native deferred replay Mapping identity changed before claim".to_owned());
        }
        let pair = {
            let mut phase = self.phase.lock();
            match &mut *phase {
                NativeDeferredReplayPhase::Deferred(pair) => {
                    let pair = pair
                        .take()
                        .ok_or_else(|| "native retained pair was already consumed".to_owned())?;
                    *phase = NativeDeferredReplayPhase::Claimed(None);
                    pair
                }
                NativeDeferredReplayPhase::Complete(current) if *current == identity => {
                    return Ok(DeferredSessionClaim::Complete)
                }
                NativeDeferredReplayPhase::Complete(_)
                | NativeDeferredReplayPhase::Claimed(_)
                | NativeDeferredReplayPhase::CloudReplaying(..) => {
                    return Err(
                        "native retained pair is bound to another Mapping transition".to_owned(),
                    )
                }
                NativeDeferredReplayPhase::Poisoned(detail) => return Err(detail.clone()),
                NativeDeferredReplayPhase::Startup(_) => {
                    return Err("startup odometry ACK is not complete".to_owned())
                }
                NativeDeferredReplayPhase::Closed => {
                    return Err("native deferred replay is closed".to_owned())
                }
            }
        };
        if let Err(error) = validate_startup_pair(&pair) {
            self.poison(error.clone());
            return Err(error);
        }
        let mut phase = self.phase.lock();
        if !matches!(*phase, NativeDeferredReplayPhase::Claimed(None)) {
            drop(phase);
            let detail = "deferred cloud claim lost its single-flight phase";
            self.poison(detail);
            return Err(detail.to_owned());
        }
        *phase = NativeDeferredReplayPhase::Claimed(Some(ClaimedStartupReplay { identity, pair }));
        Ok(DeferredSessionClaim::Claimed)
    }

    pub(crate) fn is_complete_for(
        &self,
        session_id: &str,
        map_name: &str,
        session_generation: u64,
        map_frame_generation: u64,
    ) -> bool {
        matches!(
            &*self.phase.lock(),
            NativeDeferredReplayPhase::Complete(identity)
                if identity.session_id == session_id
                    && identity.map_name == map_name
                    && identity.session_generation == session_generation
                    && identity.map_frame_generation == map_frame_generation
        )
    }

    pub(crate) async fn replay_claimed(&self) -> Result<(), String> {
        let mut claimed = {
            let mut phase = self.phase.lock();
            match &mut *phase {
                NativeDeferredReplayPhase::Claimed(claimed) => claimed
                    .take()
                    .ok_or_else(|| "deferred replay is already in flight".to_owned())?,
                NativeDeferredReplayPhase::Complete(_) => return Ok(()),
                NativeDeferredReplayPhase::Poisoned(detail) => return Err(detail.clone()),
                NativeDeferredReplayPhase::Closed => {
                    return Err("native deferred replay stopped before cloud replay".to_owned())
                }
                _ => {
                    return Err(
                        "native deferred replay has not claimed a Mapping session".to_owned()
                    )
                }
            }
        };
        let registration_mode_token = match claimed.pair.registration_mode_token {
            Some(token) => token,
            None => {
                let detail =
                    "native retained startup cloud lacks relocalization mode provenance".to_owned();
                self.poison(detail.clone());
                return Err(detail);
            }
        };
        if let Err(error) = self
            .ingress
            .common
            .validate_registration_mode_token(registration_mode_token)
        {
            let detail = format!("deferred replay rejected stale cloud mode provenance: {error}");
            self.poison(detail.clone());
            return Err(detail);
        }
        if self.stop_requested.load(Ordering::Acquire)
            || self.gate.is_closed()
            || !self.correction_ingress.is_open()
        {
            *self.phase.lock() = NativeDeferredReplayPhase::Closed;
            return Err("native deferred replay stopped before irreversible output".to_owned());
        }
        if let Err(error) = self.gate.begin_cloud_replay(self.token) {
            if self.stop_requested.load(Ordering::Acquire) || self.gate.is_closed() {
                *self.phase.lock() = NativeDeferredReplayPhase::Closed;
                return Err("native deferred replay stopped before irreversible output".to_owned());
            }
            self.poison(error.clone());
            return Err(error);
        }
        *self.phase.lock() =
            NativeDeferredReplayPhase::CloudReplaying(claimed.identity.clone(), false);
        if self.stop_requested.load(Ordering::Acquire)
            || self.gate.is_closed()
            || !self.correction_ingress.is_open()
        {
            *self.phase.lock() = NativeDeferredReplayPhase::Closed;
            return Err("native deferred replay stopped before irreversible output".to_owned());
        }
        let identity_matches = {
            let mut phase = self.phase.lock();
            match &mut *phase {
                NativeDeferredReplayPhase::CloudReplaying(identity, irreversible)
                    if *identity == claimed.identity =>
                {
                    *irreversible = true;
                    true
                }
                _ => false,
            }
        };
        if !identity_matches {
            return Err(self.poison_after_irreversible(
                "deferred Mapping identity changed before canonical output".to_owned(),
            ));
        }
        let pose = match pose_state_from_startup_pair(&claimed.pair) {
            Ok(pose) => pose,
            Err(error) => return Err(self.poison_after_irreversible(error)),
        };
        let runtime = match self.ingress.common.runtime.as_ref() {
            Some(runtime) => runtime,
            None => return Err(self.poison_after_irreversible(runtime_not_bound())),
        };
        match handoff_lio_odometry_with_correction(
            &runtime.handle,
            &pose,
            &claimed.pair.odometry.body_frame_id,
            &runtime.tf,
            self.correction_applier.as_ref(),
        )
        .await
        {
            Ok(true) => {}
            Ok(false) => {
                return Err(self.poison_after_irreversible(
                    "deferred canonical odometry committed no observation".to_owned(),
                ))
            }
            Err(error) => {
                return Err(self.poison_after_irreversible(format!(
                    "deferred canonical odometry/TF outcome is unknown: {error}"
                )))
            }
        }
        let correlated_odometry = CorrelatedOdometry {
            source_sequence: claimed.pair.odometry_identity.source_sequence,
            source_timestamp_ns: claimed.pair.odometry_identity.source_timestamp_ns,
        };
        let registration = {
            let _lifecycle_guard = runtime.handle.mapping_lifecycle_lock.lock().await;
            if self.stop_requested.load(Ordering::Acquire)
                || self.gate.is_closed()
                || !self.correction_ingress.is_open()
            {
                Err("deferred replay stopped after canonical odometry output".to_owned())
            } else {
                let mapping = runtime
                    .handle
                    .mapping_adapter
                    .as_ref()
                    .ok_or_else(|| "mapping adapter is unavailable".to_owned());
                match mapping {
                    Ok(mapping) => {
                        let adapter = mapping.lock();
                        let snapshot = adapter.snapshot();
                        let map_frame_generation = adapter.global_frame_generation();
                        if !snapshot.active
                            || snapshot.session_id != claimed.identity.session_id
                            || snapshot.map_name != claimed.identity.map_name
                            || snapshot.session_generation != claimed.identity.session_generation
                            || map_frame_generation == 0
                        {
                            Err("deferred Mapping identity changed after canonical odometry"
                                .to_owned())
                        } else {
                            self.ingress
                                .common
                                .register_odom_points_with_mode_token(
                                    claimed.pair.cloud_identity,
                                    correlated_odometry,
                                    0,
                                    claimed.pair.source_point_count,
                                    claimed.pair.registered_odom_points.as_ref().to_vec(),
                                    &adapter,
                                    registration_mode_token,
                                )
                                .map(|prepared| (map_frame_generation, prepared))
                                .map_err(|error| {
                                    format!("deferred cloud registration failed: {error}")
                                })
                        }
                    }
                    Err(error) => Err(error),
                }
            }
        };
        let (map_frame_generation, prepared_cloud) = match registration {
            Ok(registration) => registration,
            Err(error) => return Err(self.poison_after_irreversible(error)),
        };
        if usize::try_from(prepared_cloud.envelope.emitted_point_count).ok()
            != Some(claimed.pair.emitted_point_count)
            || prepared_cloud.envelope.source_point_count
                != i64::try_from(claimed.pair.source_point_count).unwrap_or(i64::MAX)
            || prepared_cloud.envelope.runtime_generation
                != claimed.pair.cloud_identity.runtime_generation
            || prepared_cloud.envelope.source_sequence
                != claimed.pair.cloud_identity.source_sequence
            || prepared_cloud.envelope.source_timestamp_ns
                != claimed.pair.cloud_identity.source_timestamp_ns
        {
            return Err(self.poison_after_irreversible(
                "deferred cloud preparation changed the retained identity or counts".to_owned(),
            ));
        }
        let unresolved_identity = claimed.identity.clone();
        claimed.identity.map_frame_generation = map_frame_generation;
        {
            let mut phase = self.phase.lock();
            match &mut *phase {
                NativeDeferredReplayPhase::CloudReplaying(identity, true)
                    if *identity == unresolved_identity =>
                {
                    *identity = claimed.identity.clone();
                }
                _ => {
                    drop(phase);
                    return Err(self.poison_after_irreversible(
                        "deferred replay lost its Mapping identity while resolving the frame generation"
                            .to_owned(),
                    ));
                }
            }
        }
        let encoded = match encode_lio_cloud_envelope(&prepared_cloud.envelope) {
            Ok(encoded) => encoded,
            Err(error) => {
                return Err(self.poison_after_irreversible(format!(
                    "deferred cloud encoding failed after canonical output: {error}"
                )))
            }
        };
        if crate::map_checksum::sha256_bytes(&encoded) != prepared_cloud.envelope_sha256 {
            return Err(self.poison_after_irreversible(
                "deferred cloud envelope proof changed before publication".to_owned(),
            ));
        }
        let payload = match encoded.get(8..) {
            Some(payload) => payload,
            None => {
                return Err(self.poison_after_irreversible(
                    "deferred cloud encoding lacks a fingerprint".to_owned(),
                ))
            }
        };
        let (nonce, outcome_rx) = match self.ingress.install_public_expectation(
            NativeReplayFrameKind::Cloud,
            claimed.pair.cloud_identity,
            &runtime.public_cloud,
            payload,
        ) {
            Ok(expectation) => expectation,
            Err(error) => return Err(self.poison_after_irreversible(error)),
        };
        if let Err(error) = publish_generated_body(
            &runtime.handle,
            &runtime.public_cloud,
            claimed.pair.cloud_identity.source_sequence,
            encoded,
        )
        .await
        {
            return Err(self.poison_after_irreversible(format!(
                "deferred cloud publication outcome is unknown: {error}"
            )));
        }
        let outcome = match tokio::time::timeout(LIO_GENERATED_PUBLISH_TIMEOUT, outcome_rx).await {
            Ok(Ok(outcome)) => outcome,
            Ok(Err(_)) => {
                return Err(self.poison_after_irreversible(
                    "deferred cloud post-handler ACK channel closed".to_owned(),
                ))
            }
            Err(_) => {
                return Err(self.poison_after_irreversible(
                    "deferred cloud post-handler ACK timed out".to_owned(),
                ))
            }
        };
        if let Err(error) = outcome {
            return Err(self.poison_after_irreversible(format!(
                "deferred cloud post-handler ACK failed: {error}"
            )));
        }
        if let Err(error) = self.ingress.finish_public_expectation(nonce) {
            return Err(self.poison_after_irreversible(error));
        }
        {
            let mut phase = self.phase.lock();
            match &*phase {
                NativeDeferredReplayPhase::CloudReplaying(identity, true)
                    if *identity == claimed.identity =>
                {
                    *phase = NativeDeferredReplayPhase::Complete(claimed.identity);
                }
                NativeDeferredReplayPhase::Poisoned(detail) => return Err(detail.clone()),
                _ => {
                    drop(phase);
                    return Err(self.poison_after_irreversible(
                        "deferred replay lost its exact Mapping identity before completion"
                            .to_owned(),
                    ));
                }
            }
        }
        if let Err(error) = self.gate.finish_cloud_replay(self.token) {
            return Err(self.poison_after_irreversible(error));
        }
        Ok(())
    }

    pub(crate) async fn wait_poison(&self) -> String {
        let mut poison_rx = self.poison_tx.subscribe();
        loop {
            if let Some(detail) = poison_rx.borrow().clone() {
                return detail;
            }
            if poison_rx.changed().await.is_err() {
                return "native deferred replay poison channel closed".to_owned();
            }
        }
    }

    pub(crate) fn poison_detail(&self) -> Option<String> {
        self.poison_tx.borrow().clone()
    }

    pub(crate) fn fail_generation(&self, detail: impl Into<String>) -> String {
        let detail = detail.into();
        self.poison(detail.clone());
        self.poison_detail().unwrap_or(detail)
    }

    fn poison_after_irreversible(&self, detail: String) -> String {
        self.poison(detail.clone());
        self.poison_detail().unwrap_or(detail)
    }

    fn poison(&self, detail: impl Into<String>) {
        let detail = detail.into();
        self.readiness.revoke();
        let canonical_detail = {
            let mut phase = self.phase.lock();
            match &*phase {
                NativeDeferredReplayPhase::Poisoned(existing) => existing.clone(),
                _ => {
                    *phase = NativeDeferredReplayPhase::Poisoned(detail.clone());
                    detail
                }
            }
        };
        self.ingress.poison_public_replay(canonical_detail.clone());
        self.gate.poison();
        self.correction_phase_a.begin();
        self.poison_tx.send_replace(Some(canonical_detail));
    }

    pub(crate) fn record_poison_diagnostics(&self, diagnostics: Vec<String>) {
        self.poison_diagnostics.lock().extend(diagnostics);
    }

    #[cfg(test)]
    pub(crate) fn poison_diagnostics_for_tests(&self) -> Vec<String> {
        self.poison_diagnostics.lock().clone()
    }

    #[cfg(test)]
    fn invalidate_retained_pair_for_tests(&self) {
        let mut phase = self.phase.lock();
        let NativeDeferredReplayPhase::Deferred(Some(pair)) = &mut *phase else {
            panic!("retained pair must be deferred before invalidation");
        };
        pair.odometry_payload_sha256[0] ^= 0xff;
    }
}

fn validate_startup_pair(pair: &ValidatedStartupPair) -> Result<(), String> {
    if pair.odometry_identity.backend != pair.cloud_identity.backend
        || pair.odometry_identity.runtime_generation != pair.cloud_identity.runtime_generation
        || pair.odometry_identity.source_sequence <= 0
        || pair.cloud_identity.source_sequence <= 0
        || pair.odometry_identity.source_timestamp_ns < 0
        || pair.cloud_identity.source_timestamp_ns < 0
        || pair.source_point_count == 0
        || pair.emitted_point_count == 0
        || pair.emitted_point_count > pair.source_point_count
        || pair.registered_odom_points.len() != pair.emitted_point_count
        || SourceIdentity::from_odometry(&pair.odometry) != pair.odometry_identity
        || odometry_pose_isometry(&pair.odometry.odometry).map_err(|error| error.to_string())?
            != pair.frozen_odom_from_body
    {
        return Err("validated startup pair identity/count/pose proof is inconsistent".to_owned());
    }
    let encoded =
        encode_lio_odometry_envelope(&pair.odometry).map_err(|error| error.to_string())?;
    if crate::map_checksum::sha256_bytes(&encoded) != pair.odometry_payload_sha256
        || point_set_sha256(pair.registered_odom_points.as_ref()) != pair.registered_points_sha256
    {
        return Err("validated startup pair payload proof is inconsistent".to_owned());
    }
    Ok(())
}

fn pose_state_from_startup_pair(pair: &ValidatedStartupPair) -> Result<PoseState, String> {
    validate_startup_pair(pair)?;
    let pose = &pair.odometry.odometry.pose;
    Ok(PoseState {
        x: pose.position[0],
        y: pose.position[1],
        z: pose.position[2],
        qx: pose.orientation_xyzw[0],
        qy: pose.orientation_xyzw[1],
        qz: pose.orientation_xyzw[2],
        qw: pose.orientation_xyzw[3],
        frame_id: pair.odometry.odom_frame_id.clone(),
        timestamp_ns: u64::try_from(pair.odometry_identity.source_timestamp_ns)
            .map_err(|_| "startup odometry timestamp is negative".to_owned())?,
    })
}

#[async_trait]
impl LioIngressSink for NativeLioIngress {
    fn close(&self) {
        self.common.close();
        let mut pair_state = self.pair_state.lock();
        pair_state.odometry.clear();
        pair_state.clouds.clear();
        pair_state.probe_phase = NativeProbePhase::Closed;
        let previous = std::mem::replace(
            &mut pair_state.public_replay,
            NativePublicReplayPhase::Closed,
        );
        let error = Err("native LIO replay ingress closed before exact ACK".to_owned());
        match previous {
            NativePublicReplayPhase::Expecting(mut expectation) => {
                if let Some(outcome) = expectation.outcome.take() {
                    let _ = outcome.send(error);
                }
            }
            NativePublicReplayPhase::Claimed {
                outcome: Some(outcome),
                ..
            } => {
                let _ = outcome.send(error);
            }
            _ => {}
        }
    }

    async fn wait_startup_ready(
        &self,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let mut ready_rx = self.source_pair_ready_rx.clone();
        loop {
            if *ready_rx.borrow() {
                return Ok(());
            }
            if *shutdown_rx.borrow() {
                return Err("native LIO startup stopped before the first valid source pair".into());
            }
            tokio::select! {
                biased;
                changed = shutdown_rx.changed() => {
                    if changed.is_err() || *shutdown_rx.borrow() {
                        return Err(
                            "native LIO startup stopped before the first valid source pair".into(),
                        );
                    }
                }
                changed = ready_rx.changed() => {
                    if changed.is_err() {
                        return Err(
                            "native LIO first-pair readiness channel closed unexpectedly".into(),
                        );
                    }
                }
            }
        }
    }

    async fn accept_public_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        <CommonLioIngress as LioIngressSink>::accept_public_odometry(
            &self.common,
            frame,
            shutdown_rx,
        )
        .await
    }

    async fn accept_public_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        <CommonLioIngress as LioIngressSink>::accept_public_cloud(&self.common, frame, shutdown_rx)
            .await
    }

    async fn accept_legacy_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let odometry = decode_body(
            nuwax_codec::lcm::schema::ODOMETRY_FINGERPRINT,
            &frame.payload,
            nuwax_codec::lcm::schema::decode_odometry,
        )?;
        let source_timestamp_ns = header_timestamp_ns(&odometry.header)?;
        let envelope = self
            .common
            .prepare_legacy_odometry(source_timestamp_ns, odometry)
            .map_err(|error| error.to_string())?;
        let identity = SourceIdentity::from_odometry(&envelope);
        let sample = NativeOdometrySample {
            identity,
            odometry: envelope.odometry.clone(),
        };
        if let Some(runtime) = self.common.runtime.as_ref() {
            let encoded =
                encode_lio_odometry_envelope(&envelope).map_err(|error| error.to_string())?;
            publish_generated_body(
                &runtime.handle,
                &runtime.public_odometry,
                envelope.source_sequence,
                encoded,
            )
            .await?;
        }
        if let Some(pair) = self.insert_odometry(sample) {
            self.process_pair(pair, shutdown_rx).await?;
        }
        Ok(())
    }

    async fn accept_legacy_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let registration_mode_token = self.common.capture_registration_mode_token();
        let cloud = decode_body(
            nuwax_codec::lcm::schema::POINT_CLOUD2_FINGERPRINT,
            &frame.payload,
            nuwax_codec::lcm::schema::decode_point_cloud2,
        )?;
        let source_timestamp_ns = header_timestamp_ns(&cloud.header)?;
        let source_points =
            parse_sensor_xyz_cloud(&cloud, &self.common.spec).map_err(|error| error.to_string())?;
        let source_point_count = source_points.len();
        let sensor_points = deterministic_downsample(source_points);
        let identity = self
            .common
            .reserve_legacy_cloud_identity(source_timestamp_ns)
            .map_err(|error| error.to_string())?;
        let sample = NativeCloudSample {
            identity,
            source_point_count,
            sensor_points,
            registration_mode_token,
        };
        if let Some(pair) = self.insert_cloud(sample) {
            self.process_pair(pair, shutdown_rx).await?;
        }
        Ok(())
    }
}

fn take_best_native_pair(
    state: &mut NativePairState,
    pair_max_skew_ns: i64,
) -> Option<NativeSourcePair> {
    let max_skew = u64::try_from(pair_max_skew_ns).ok()?;
    loop {
        let exact = state
            .clouds
            .iter()
            .enumerate()
            .flat_map(|(cloud_index, cloud)| {
                state
                    .odometry
                    .iter()
                    .enumerate()
                    .filter(move |(_, odometry)| {
                        cloud.identity.source_timestamp_ns == odometry.identity.source_timestamp_ns
                    })
                    .map(move |(odometry_index, odometry)| {
                        (
                            cloud.identity.source_sequence,
                            odometry.identity.source_sequence,
                            cloud_index,
                            odometry_index,
                        )
                    })
            })
            .min();
        if let Some((_, _, cloud_index, odometry_index)) = exact {
            let cloud = state.clouds.remove(cloud_index)?;
            let odometry = state.odometry.get(odometry_index)?.clone();
            return Some(NativeSourcePair { cloud, odometry });
        }

        // A cloud cannot be paired with an older odometry immediately: an
        // exact (or closer causal) odometry may still be in flight. Once the
        // strictly increasing odometry watermark reaches the cloud timestamp,
        // the newest retained odometry at or before the cloud is final. A
        // later pose is not a valid correlated-odometry proof for that cloud.
        let odometry_watermark = state
            .odometry
            .back()
            .map(|odometry| odometry.identity.source_timestamp_ns)?;
        let cloud_index = state
            .clouds
            .iter()
            .position(|cloud| cloud.identity.source_timestamp_ns <= odometry_watermark)?;
        let cloud = state.clouds.remove(cloud_index)?;
        let best_odometry = state
            .odometry
            .iter()
            .enumerate()
            .filter_map(|(odometry_index, odometry)| {
                if odometry.identity.source_timestamp_ns > cloud.identity.source_timestamp_ns {
                    return None;
                }
                let skew = cloud
                    .identity
                    .source_timestamp_ns
                    .abs_diff(odometry.identity.source_timestamp_ns);
                (skew <= max_skew).then_some((
                    skew,
                    odometry.identity.source_sequence,
                    odometry_index,
                ))
            })
            .min();
        if let Some((_, _, odometry_index)) = best_odometry {
            let odometry = state.odometry.get(odometry_index)?.clone();
            return Some(NativeSourcePair { cloud, odometry });
        }
        // This cloud is behind the odometry watermark and outside the admitted
        // skew from every retained sample, so no future sample can rescue it.
        if state.clouds.is_empty() {
            return None;
        }
    }
}

fn transform_native_pair(
    pair: &NativeSourcePair,
    body_from_sensor: &BodyFromSensorTransform,
) -> Result<Vec<Point3<f64>>, String> {
    let position = pair.odometry.odometry.pose.position;
    let orientation = pair.odometry.odometry.pose.orientation_xyzw;
    if position
        .iter()
        .chain(orientation.iter())
        .any(|value| !value.is_finite())
    {
        return Err("native LIO odometry pose contains a non-finite value".into());
    }
    let quaternion = Quaternion::new(
        orientation[3],
        orientation[0],
        orientation[1],
        orientation[2],
    );
    let norm = quaternion.norm();
    if !norm.is_finite() || (norm - 1.0).abs() > 1.0e-3 {
        return Err(format!(
            "native LIO odometry quaternion is not unit length: {norm}"
        ));
    }
    let odom_from_body = UnitQuaternion::new_normalize(quaternion);
    let translation = Vector3::new(position[0], position[1], position[2]);
    pair.cloud
        .sensor_points
        .iter()
        .map(|sensor_point| {
            let body_point = body_from_sensor.transform_point(sensor_point);
            let odom_point = odom_from_body.transform_point(&body_point) + translation;
            if odom_point.coords.iter().all(|value| value.is_finite()) {
                Ok(odom_point)
            } else {
                Err("native LIO transformed cloud contains a non-finite point".to_string())
            }
        })
        .collect()
}

fn point_set_sha256(points: &[Point3<f64>]) -> [u8; 32] {
    let mut canonical = Vec::with_capacity(points.len().saturating_mul(24));
    for point in points {
        canonical.extend_from_slice(&point.x.to_bits().to_be_bytes());
        canonical.extend_from_slice(&point.y.to_bits().to_be_bytes());
        canonical.extend_from_slice(&point.z.to_bits().to_be_bytes());
    }
    crate::map_checksum::sha256_bytes(&canonical)
}

fn odometry_pose_isometry(odometry: &Odometry) -> Result<Isometry3<f64>, LioIngressError> {
    let position = odometry.pose.position;
    let orientation = odometry.pose.orientation_xyzw;
    if position
        .iter()
        .chain(orientation.iter())
        .any(|value| !value.is_finite())
    {
        return Err(LioIngressError::InvalidOdometryPose);
    }
    let quaternion = Quaternion::new(
        orientation[3],
        orientation[0],
        orientation[1],
        orientation[2],
    );
    let norm = quaternion.norm();
    if !norm.is_finite() || (norm - 1.0).abs() > 1.0e-3 {
        return Err(LioIngressError::InvalidOdometryPose);
    }
    Ok(Isometry3::from_parts(
        Translation3::new(position[0], position[1], position[2]),
        UnitQuaternion::new_normalize(quaternion),
    ))
}

impl SourceIdentity {
    fn from_odometry(envelope: &LioOdometryEnvelope) -> Self {
        Self {
            backend: envelope.backend,
            runtime_generation: envelope.runtime_generation,
            source_sequence: envelope.source_sequence,
            source_timestamp_ns: envelope.source_timestamp_ns,
        }
    }

    fn from_cloud(envelope: &LioCloudEnvelope) -> Self {
        Self {
            backend: envelope.backend,
            runtime_generation: envelope.runtime_generation,
            source_sequence: envelope.source_sequence,
            source_timestamp_ns: envelope.source_timestamp_ns,
        }
    }
}

#[async_trait]
impl LioIngressSink for CommonLioIngress {
    fn close(&self) {
        CommonLioIngress::close(self);
    }

    async fn accept_public_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let runtime = self.runtime.as_ref().ok_or_else(runtime_not_bound)?;
        let envelope = decode_body(
            nuwax_codec::lcm::lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
            &frame.payload,
            nuwax_codec::lcm::lio::decode_lio_odometry_envelope,
        )?;
        let Some(_lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
            return Ok(());
        };
        let accepted = CommonLioIngress::accept_public_odometry(self, envelope, wall_clock_ns()?)
            .map_err(|error| error.to_string())?;
        let odometry = &accepted.envelope.odometry;
        let pose = PoseState {
            x: odometry.pose.position[0],
            y: odometry.pose.position[1],
            z: odometry.pose.position[2],
            qx: odometry.pose.orientation_xyzw[0],
            qy: odometry.pose.orientation_xyzw[1],
            qz: odometry.pose.orientation_xyzw[2],
            qw: odometry.pose.orientation_xyzw[3],
            frame_id: accepted.envelope.odom_frame_id.clone(),
            timestamp_ns: u64::try_from(accepted.envelope.source_timestamp_ns)
                .map_err(|_| "source timestamp is negative".to_string())?,
        };
        let committed = handoff_lio_odometry(
            &runtime.handle,
            &pose,
            &accepted.envelope.body_frame_id,
            &runtime.tf,
        )
        .await?;
        if !committed {
            return Err("LIO odometry handoff committed no observation".to_string());
        }
        Ok(())
    }

    async fn accept_public_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let runtime = self.runtime.as_ref().ok_or_else(runtime_not_bound)?;
        let mut encoded = Vec::with_capacity(8 + frame.payload.len());
        encoded.extend_from_slice(&nuwax_codec::lcm::lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1);
        encoded.extend_from_slice(&frame.payload);
        let envelope = nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
            .map_err(|error| error.to_string())?;
        let accepted = {
            let Some(_lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
                return Ok(());
            };
            let accepted =
                CommonLioIngress::accept_public_cloud(self, envelope, &encoded, wall_clock_ns()?)
                    .map_err(|error| error.to_string())?;
            let adapter = runtime
                .handle
                .mapping_adapter
                .as_ref()
                .ok_or_else(|| "mapping adapter is unavailable".to_string())?;
            let adapter = adapter.lock();
            if adapter.snapshot().session_generation != accepted.binding.mapping_session_generation
                || adapter.global_frame_generation() != accepted.binding.map_frame_generation
            {
                return Err("registration permit mapping generation is stale".to_string());
            }
            accepted
        };
        let committed = handoff_registered_lio_cloud_fenced(
            &runtime.handle,
            &accepted.map_points,
            XYZ_POINT_STEP as u32,
            u64::try_from(accepted.adapter_timestamp_ns)
                .map_err(|_| "adapter timestamp is negative".to_string())?,
            u64::try_from(accepted.envelope.source_timestamp_ns)
                .map_err(|_| "source timestamp is negative".to_string())?,
            &accepted.envelope.map_frame_id,
            accepted.binding.mapping_session_generation,
            accepted.binding.map_frame_generation,
        )
        .await?;
        if !committed {
            return Err("registered LIO cloud handoff committed no map revision".to_string());
        }
        Ok(())
    }

    async fn accept_legacy_odometry(
        &self,
        frame: RawLcmFrame,
        _shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let runtime = self.runtime.as_ref().ok_or_else(runtime_not_bound)?;
        let odometry = decode_body(
            nuwax_codec::lcm::schema::ODOMETRY_FINGERPRINT,
            &frame.payload,
            nuwax_codec::lcm::schema::decode_odometry,
        )?;
        let timestamp_ns = header_timestamp_ns(&odometry.header)?;
        let envelope = self
            .prepare_legacy_odometry(timestamp_ns, odometry)
            .map_err(|error| error.to_string())?;
        let encoded = encode_lio_odometry_envelope(&envelope).map_err(|error| error.to_string())?;
        publish_generated_body(
            &runtime.handle,
            &runtime.public_odometry,
            envelope.source_sequence,
            encoded,
        )
        .await
    }

    async fn accept_legacy_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        let runtime = self.runtime.as_ref().ok_or_else(runtime_not_bound)?;
        let registration_mode_token = self.capture_registration_mode_token();
        let cloud = decode_body(
            nuwax_codec::lcm::schema::POINT_CLOUD2_FINGERPRINT,
            &frame.payload,
            nuwax_codec::lcm::schema::decode_point_cloud2,
        )?;
        let source_timestamp_ns = header_timestamp_ns(&cloud.header)?;
        let identity = self
            .reserve_legacy_cloud_identity(source_timestamp_ns)
            .map_err(|error| error.to_string())?;
        let correlated_odometry = self
            .latest_causal_odometry(source_timestamp_ns)
            .ok_or_else(|| LioIngressError::MissingOdometry.to_string())?;
        let Some(_lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
            return Ok(());
        };
        let mapping = runtime
            .handle
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| "mapping adapter is unavailable".to_string())?;
        let prepared = {
            let adapter = mapping.lock();
            self.register_cloud_with_mode_token(
                identity,
                correlated_odometry,
                0,
                cloud,
                &adapter,
                registration_mode_token,
            )
            .map_err(|error| error.to_string())?
        };
        let encoded =
            encode_lio_cloud_envelope(&prepared.envelope).map_err(|error| error.to_string())?;
        let publish_result = publish_generated_body(
            &runtime.handle,
            &runtime.public_cloud,
            prepared.envelope.source_sequence,
            encoded,
        )
        .await;
        if publish_result.is_err() {
            self.revoke_prepared_cloud(
                prepared.envelope.runtime_generation,
                prepared.envelope.source_sequence,
                prepared.envelope_sha256,
            );
        }
        publish_result
    }
}

#[async_trait]
impl LioIngressSink for CorrectionAwareLioIngress {
    fn close(&self) {
        match &self.inner {
            CorrectionAwareLioIngressInner::Device(inner) => LioIngressSink::close(inner.as_ref()),
            CorrectionAwareLioIngressInner::Native(inner) => LioIngressSink::close(inner.as_ref()),
        }
    }

    async fn wait_startup_ready(
        &self,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match &self.inner {
            CorrectionAwareLioIngressInner::Device(inner) => {
                LioIngressSink::wait_startup_ready(inner.as_ref(), shutdown_rx).await
            }
            CorrectionAwareLioIngressInner::Native(inner) => {
                LioIngressSink::wait_startup_ready(inner.as_ref(), shutdown_rx).await
            }
        }
    }

    async fn accept_public_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match &self.inner {
            CorrectionAwareLioIngressInner::Device(inner) => {
                accept_public_odometry_with_correction(
                    inner,
                    frame,
                    shutdown_rx,
                    self.correction_applier.as_ref(),
                )
                .await
            }
            CorrectionAwareLioIngressInner::Native(inner) => {
                match inner.classify_public_frame(NativeReplayFrameKind::Odometry, &frame)? {
                    NativePublicFrameDecision::Normal => {
                        accept_public_odometry_with_correction(
                            &inner.common,
                            frame,
                            shutdown_rx,
                            self.correction_applier.as_ref(),
                        )
                        .await
                    }
                    NativePublicFrameDecision::Ignore => Ok(()),
                    NativePublicFrameDecision::Claim(claim) => {
                        let result = accept_startup_public_odometry(&inner.common, frame);
                        inner.complete_public_expectation(claim, &result);
                        result
                    }
                }
            }
        }
    }

    async fn accept_public_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match &self.inner {
            CorrectionAwareLioIngressInner::Device(inner) => {
                accept_public_cloud_with_correction(
                    inner,
                    frame,
                    shutdown_rx,
                    &self.correction_ingress,
                )
                .await
            }
            CorrectionAwareLioIngressInner::Native(inner) => {
                match inner.classify_public_frame(NativeReplayFrameKind::Cloud, &frame)? {
                    NativePublicFrameDecision::Normal => {
                        accept_public_cloud_with_correction(
                            &inner.common,
                            frame,
                            shutdown_rx,
                            &self.correction_ingress,
                        )
                        .await
                    }
                    NativePublicFrameDecision::Ignore => Ok(()),
                    NativePublicFrameDecision::Claim(claim) => {
                        let result = accept_public_cloud_replay_with_correction(
                            &inner.common,
                            frame,
                            shutdown_rx,
                            &self.correction_ingress,
                        )
                        .await;
                        inner.complete_public_expectation(claim, &result);
                        result
                    }
                }
            }
        }
    }

    async fn accept_legacy_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match &self.inner {
            CorrectionAwareLioIngressInner::Device(inner) => {
                LioIngressSink::accept_legacy_odometry(inner.as_ref(), frame, shutdown_rx).await
            }
            CorrectionAwareLioIngressInner::Native(inner) => {
                LioIngressSink::accept_legacy_odometry(inner.as_ref(), frame, shutdown_rx).await
            }
        }
    }

    async fn accept_legacy_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match &self.inner {
            CorrectionAwareLioIngressInner::Device(inner) => {
                LioIngressSink::accept_legacy_cloud(inner.as_ref(), frame, shutdown_rx).await
            }
            CorrectionAwareLioIngressInner::Native(inner) => {
                LioIngressSink::accept_legacy_cloud(inner.as_ref(), frame, shutdown_rx).await
            }
        }
    }
}

fn accept_startup_public_odometry(
    ingress: &CommonLioIngress,
    frame: RawLcmFrame,
) -> Result<(), String> {
    let envelope = decode_body(
        nuwax_codec::lcm::lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
        &frame.payload,
        nuwax_codec::lcm::lio::decode_lio_odometry_envelope,
    )?;
    CommonLioIngress::accept_public_odometry(ingress, envelope, wall_clock_ns()?)
        .map(|_| ())
        .map_err(|error| error.to_string())
}

async fn accept_public_cloud_replay_with_correction(
    ingress: &CommonLioIngress,
    frame: RawLcmFrame,
    shutdown_rx: &mut watch::Receiver<bool>,
    correction: &crate::correction_coordinator::CorrectionFrameIngress,
) -> Result<(), String> {
    let runtime = ingress.runtime.as_ref().ok_or_else(runtime_not_bound)?;
    let mut encoded = Vec::with_capacity(8 + frame.payload.len());
    encoded.extend_from_slice(&nuwax_codec::lcm::lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1);
    encoded.extend_from_slice(&frame.payload);
    let envelope = nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
        .map_err(|error| error.to_string())?;
    let accepted = {
        let Some(lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
            return Err("deferred cloud replay stopped before admission".to_owned());
        };
        let accepted =
            CommonLioIngress::accept_public_cloud(ingress, envelope, &encoded, wall_clock_ns()?)
                .map_err(|error| error.to_string())?;
        let adapter = runtime
            .handle
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
        let adapter = adapter.lock();
        if !adapter.snapshot().active
            || adapter.snapshot().session_id != accepted.binding.mapping_session_id
            || adapter.snapshot().session_generation != accepted.binding.mapping_session_generation
            || adapter.global_frame_generation() != accepted.binding.map_frame_generation
        {
            return Err("deferred registration permit mapping generation is stale".to_owned());
        }
        drop(adapter);
        drop(lifecycle_guard);
        accepted
    };
    let committed = handoff_registered_lio_cloud_replay_with_correction(
        &runtime.handle,
        &accepted.map_points,
        XYZ_POINT_STEP as u32,
        u64::try_from(accepted.adapter_timestamp_ns)
            .map_err(|_| "adapter timestamp is negative".to_owned())?,
        u64::try_from(accepted.envelope.source_timestamp_ns)
            .map_err(|_| "source timestamp is negative".to_owned())?,
        &accepted.envelope.map_frame_id,
        &accepted.envelope.body_frame_id,
        &accepted.binding,
        correction,
    )
    .await?;
    if !committed {
        return Err("deferred registered cloud committed no map revision".to_owned());
    }
    Ok(())
}

#[async_trait]
impl LioIngressSink for LioIngressRoutingGate {
    fn close(&self) {
        let active = {
            let mut state = self.state.lock();
            match std::mem::replace(&mut *state, LioRoutingState::Closed) {
                LioRoutingState::StartupOdometry { active, .. }
                | LioRoutingState::DeferredCloud { active, .. }
                | LioRoutingState::CloudReplaying { active, .. } => Some(active),
                LioRoutingState::Active(active) => Some(active),
                LioRoutingState::Preparing
                | LioRoutingState::Poisoned
                | LioRoutingState::Closed => None,
            }
        };
        self.probe.close();
        if let Some(active) = active {
            active.close();
        }
        self.state_changed.notify_waiters();
    }

    async fn wait_startup_ready(
        &self,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        if matches!(
            *self.state.lock(),
            LioRoutingState::Poisoned | LioRoutingState::Closed
        ) {
            return Err("LIO routing gate closed before startup readiness".to_owned());
        }
        self.probe.wait_startup_ready(shutdown_rx).await
    }

    async fn accept_public_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match self.public_target() {
            Some(target) => target.accept_public_odometry(frame, shutdown_rx).await,
            None => Ok(()),
        }
    }

    async fn accept_public_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match self.public_target() {
            Some(target) => target.accept_public_cloud(frame, shutdown_rx).await,
            None => Ok(()),
        }
    }

    async fn accept_legacy_odometry(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match self.legacy_target(shutdown_rx).await {
            Some(target) => target.accept_legacy_odometry(frame, shutdown_rx).await,
            None => Ok(()),
        }
    }

    async fn accept_legacy_cloud(
        &self,
        frame: RawLcmFrame,
        shutdown_rx: &mut watch::Receiver<bool>,
    ) -> Result<(), String> {
        match self.legacy_target(shutdown_rx).await {
            Some(target) => target.accept_legacy_cloud(frame, shutdown_rx).await,
            None => Ok(()),
        }
    }
}

async fn accept_public_odometry_with_correction(
    ingress: &CommonLioIngress,
    frame: RawLcmFrame,
    shutdown_rx: &mut watch::Receiver<bool>,
    correction: &GeometryCanonicalCorrectionApplier,
) -> Result<(), String> {
    let runtime = ingress.runtime.as_ref().ok_or_else(runtime_not_bound)?;
    let envelope = decode_body(
        nuwax_codec::lcm::lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
        &frame.payload,
        nuwax_codec::lcm::lio::decode_lio_odometry_envelope,
    )?;
    let Some(_lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
        return Ok(());
    };
    let accepted = CommonLioIngress::accept_public_odometry(ingress, envelope, wall_clock_ns()?)
        .map_err(|error| error.to_string())?;
    let odometry = &accepted.envelope.odometry;
    let pose = PoseState {
        x: odometry.pose.position[0],
        y: odometry.pose.position[1],
        z: odometry.pose.position[2],
        qx: odometry.pose.orientation_xyzw[0],
        qy: odometry.pose.orientation_xyzw[1],
        qz: odometry.pose.orientation_xyzw[2],
        qw: odometry.pose.orientation_xyzw[3],
        frame_id: accepted.envelope.odom_frame_id.clone(),
        timestamp_ns: u64::try_from(accepted.envelope.source_timestamp_ns)
            .map_err(|_| "source timestamp is negative".to_owned())?,
    };
    let committed = handoff_lio_odometry_with_correction(
        &runtime.handle,
        &pose,
        &accepted.envelope.body_frame_id,
        &runtime.tf,
        correction,
    )
    .await?;
    if !committed {
        return Err("LIO odometry handoff committed no observation".to_owned());
    }
    Ok(())
}

async fn accept_public_cloud_with_correction(
    ingress: &CommonLioIngress,
    frame: RawLcmFrame,
    shutdown_rx: &mut watch::Receiver<bool>,
    correction: &crate::correction_coordinator::CorrectionFrameIngress,
) -> Result<(), String> {
    let runtime = ingress.runtime.as_ref().ok_or_else(runtime_not_bound)?;
    let mut encoded = Vec::with_capacity(8 + frame.payload.len());
    encoded.extend_from_slice(&nuwax_codec::lcm::lio::LIO_CLOUD_ENVELOPE_FINGERPRINT_V1);
    encoded.extend_from_slice(&frame.payload);
    let envelope = nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
        .map_err(|error| error.to_string())?;
    let accepted = {
        let Some(_lifecycle_guard) = mapping_admission_guard(runtime, shutdown_rx).await else {
            return Ok(());
        };
        let accepted =
            CommonLioIngress::accept_public_cloud(ingress, envelope, &encoded, wall_clock_ns()?)
                .map_err(|error| error.to_string())?;
        let adapter = runtime
            .handle
            .mapping_adapter
            .as_ref()
            .ok_or_else(|| "mapping adapter is unavailable".to_owned())?;
        let adapter = adapter.lock();
        if adapter.snapshot().session_id != accepted.binding.mapping_session_id
            || adapter.snapshot().session_generation != accepted.binding.mapping_session_generation
            || adapter.global_frame_generation() != accepted.binding.map_frame_generation
        {
            return Err("registration permit mapping generation is stale".to_owned());
        }
        accepted
    };
    let committed = handoff_registered_lio_cloud_with_correction(
        &runtime.handle,
        &accepted.map_points,
        XYZ_POINT_STEP as u32,
        u64::try_from(accepted.adapter_timestamp_ns)
            .map_err(|_| "adapter timestamp is negative".to_owned())?,
        u64::try_from(accepted.envelope.source_timestamp_ns)
            .map_err(|_| "source timestamp is negative".to_owned())?,
        &accepted.envelope.map_frame_id,
        &accepted.envelope.body_frame_id,
        &accepted.binding,
        correction,
    )
    .await?;
    if !committed {
        return Err("registered LIO cloud handoff committed no map revision".to_owned());
    }
    Ok(())
}

async fn mapping_admission_guard<'a>(
    runtime: &'a CommonLioRuntime,
    shutdown_rx: &mut watch::Receiver<bool>,
) -> Option<tokio::sync::MutexGuard<'a, ()>> {
    if *shutdown_rx.borrow() {
        return None;
    }
    tokio::select! {
        biased;
        _ = shutdown_rx.changed() => None,
        guard = runtime.handle.mapping_lifecycle_lock.lock() => Some(guard),
    }
}

fn runtime_not_bound() -> String {
    "CommonLioIngress runtime handoff is not bound".to_string()
}

fn decode_body<T>(
    fingerprint: [u8; 8],
    body: &[u8],
    decode: impl FnOnce(&[u8]) -> Result<T, nuwax_codec::CodecError>,
) -> Result<T, String> {
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&fingerprint);
    encoded.extend_from_slice(body);
    decode(&encoded).map_err(|error| error.to_string())
}

async fn publish_generated_body(
    handle: &GeometryRuntimeHandle,
    binding: &(LcmTopic, LcmMessageType),
    source_sequence: i64,
    encoded: Vec<u8>,
) -> Result<(), String> {
    let payload = encoded
        .get(8..)
        .ok_or_else(|| "generated LIO encoding is missing its fingerprint".to_string())?
        .to_vec();
    let seqno = u32::try_from(source_sequence)
        .map_err(|_| "LIO source sequence exceeds transport sequence".to_string())?;
    let frame = RawLcmFrameBuilder::new(binding.0.clone(), binding.1.clone())
        .payload(payload)
        .seqno(seqno)
        .build();
    match tokio::time::timeout(
        LIO_GENERATED_PUBLISH_TIMEOUT,
        handle.transport.publish(frame),
    )
    .await
    {
        Ok(Ok(())) => Ok(()),
        Ok(Err(error)) => Err(error.to_string()),
        Err(_) => Err("generated LIO publication timed out".to_string()),
    }
}

fn header_timestamp_ns(header: &Header) -> Result<i64, String> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err("invalid standard-message header timestamp".to_string());
    }
    i64::from(header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(header.stamp_nsec)))
        .ok_or_else(|| "standard-message header timestamp overflow".to_string())
}

fn wall_clock_ns() -> Result<i64, String> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| format!("system clock precedes UNIX epoch: {error}"))?;
    i64::try_from(duration.as_nanos()).map_err(|_| "system clock nanoseconds overflow".to_string())
}

fn parse_odom_xyz_cloud(
    cloud: &PointCloud2,
    spec: &DeviceLioIngressSpec,
) -> Result<Vec<Point3<f64>>, LioIngressError> {
    if cloud.header.frame_id == spec.map_frame_id {
        return Err(LioIngressError::AlreadyRegisteredMapCloud);
    }
    if cloud.header.frame_id == spec.sensor_frame_id {
        return Err(LioIngressError::RawSensorCloud);
    }
    if cloud.header.frame_id != spec.odom_frame_id {
        return Err(LioIngressError::WrongCloudFrame {
            expected: spec.odom_frame_id.clone(),
            actual: cloud.header.frame_id.clone(),
        });
    }
    parse_xyz_cloud_with_limit(cloud, MAX_LIO_EMITTED_POINT_COUNT)
}

fn parse_sensor_xyz_cloud(
    cloud: &PointCloud2,
    spec: &DeviceLioIngressSpec,
) -> Result<Vec<Point3<f64>>, LioIngressError> {
    if cloud.header.frame_id != spec.sensor_frame_id {
        return Err(LioIngressError::WrongCloudFrame {
            expected: spec.sensor_frame_id.clone(),
            actual: cloud.header.frame_id.clone(),
        });
    }
    parse_xyz_cloud_with_limit(cloud, MAX_LIO_SOURCE_POINT_COUNT)
}

fn parse_xyz_cloud_with_limit(
    cloud: &PointCloud2,
    max_point_count: i64,
) -> Result<Vec<Point3<f64>>, LioIngressError> {
    if cloud.is_bigendian {
        return Err(LioIngressError::BigEndianCloud);
    }
    let height = usize::try_from(cloud.height).map_err(|_| LioIngressError::InvalidCloudLayout)?;
    let width = usize::try_from(cloud.width).map_err(|_| LioIngressError::InvalidCloudLayout)?;
    let point_step =
        usize::try_from(cloud.point_step).map_err(|_| LioIngressError::InvalidCloudLayout)?;
    let row_step =
        usize::try_from(cloud.row_step).map_err(|_| LioIngressError::InvalidCloudLayout)?;
    if height == 0 || width == 0 || point_step < XYZ_POINT_STEP as usize {
        return Err(LioIngressError::InvalidCloudLayout);
    }
    let point_count = height
        .checked_mul(width)
        .ok_or(LioIngressError::PointCountOverflow)?;
    let point_count_i64 =
        i64::try_from(point_count).map_err(|_| LioIngressError::PointCountOverflow)?;
    if point_count_i64 > max_point_count {
        return Err(LioIngressError::TooManyPoints(point_count_i64));
    }
    let expected_row_step = width
        .checked_mul(point_step)
        .ok_or(LioIngressError::InvalidCloudLayout)?;
    let expected_data_len = height
        .checked_mul(row_step)
        .ok_or(LioIngressError::InvalidCloudLayout)?;
    if row_step != expected_row_step || cloud.data.len() != expected_data_len {
        return Err(LioIngressError::InvalidCloudLayout);
    }
    for (name, offset) in [("x", 0), ("y", 4), ("z", 8)] {
        let matching = cloud
            .fields
            .iter()
            .filter(|field| field.name == name)
            .collect::<Vec<_>>();
        if matching.len() != 1
            || matching[0].offset != offset
            || matching[0].datatype != POINT_FIELD_FLOAT32
            || matching[0].count != 1
        {
            return Err(LioIngressError::InvalidXyzLayout);
        }
    }
    let mut points = Vec::with_capacity(point_count);
    for bytes in cloud.data.chunks_exact(point_step) {
        let x = f32::from_le_bytes(bytes[0..4].try_into().expect("four-byte x")) as f64;
        let y = f32::from_le_bytes(bytes[4..8].try_into().expect("four-byte y")) as f64;
        let z = f32::from_le_bytes(bytes[8..12].try_into().expect("four-byte z")) as f64;
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(LioIngressError::NonFinitePoint);
        }
        points.push(Point3::new(x, y, z));
    }
    Ok(points)
}

fn deterministic_downsample(points: Vec<Point3<f64>>) -> Vec<Point3<f64>> {
    let target = usize::try_from(MAX_LIO_EMITTED_POINT_COUNT)
        .expect("positive emitted point limit fits usize");
    if points.len() <= target {
        return points;
    }
    let source_len = points.len();
    let mut sampled = Vec::with_capacity(target);
    for sample_index in 0..target {
        let source_index = sample_index * source_len / target;
        sampled.push(points[source_index]);
    }
    sampled
}

fn rebuild_xyz_cloud(
    points: &[Point3<f64>],
    header: Header,
) -> Result<PointCloud2, LioIngressError> {
    if points.is_empty() {
        return Err(LioIngressError::InvalidCloudLayout);
    }
    let width = i32::try_from(points.len()).map_err(|_| LioIngressError::PointCountOverflow)?;
    let row_step = width
        .checked_mul(XYZ_POINT_STEP)
        .ok_or(LioIngressError::PointCountOverflow)?;
    let mut data = Vec::with_capacity(
        usize::try_from(row_step).map_err(|_| LioIngressError::PointCountOverflow)?,
    );
    for point in points {
        for coordinate in [point.x, point.y, point.z] {
            if !coordinate.is_finite()
                || coordinate < f32::MIN as f64
                || coordinate > f32::MAX as f64
            {
                return Err(LioIngressError::NonFinitePoint);
            }
            data.extend_from_slice(&(coordinate as f32).to_le_bytes());
        }
    }
    Ok(PointCloud2 {
        header,
        height: 1,
        width,
        fields: xyz_fields(),
        is_bigendian: false,
        point_step: XYZ_POINT_STEP,
        row_step,
        data,
        is_dense: true,
    })
}

fn xyz_fields() -> Vec<PointField> {
    [("x", 0), ("y", 4), ("z", 8)]
        .into_iter()
        .map(|(name, offset)| PointField {
            name: name.to_string(),
            offset,
            datatype: POINT_FIELD_FLOAT32,
            count: 1,
        })
        .collect()
}

fn source_header(identity: SourceIdentity, frame_id: &str) -> Result<Header, LioIngressError> {
    let seq = i32::try_from(identity.source_sequence)
        .map_err(|_| LioIngressError::SequenceExceedsHeader(identity.source_sequence))?;
    let stamp_sec_i64 = identity.source_timestamp_ns / 1_000_000_000;
    let stamp_nsec_i64 = identity.source_timestamp_ns % 1_000_000_000;
    let stamp_sec = i32::try_from(stamp_sec_i64)
        .map_err(|_| LioIngressError::TimestampExceedsHeader(identity.source_timestamp_ns))?;
    let stamp_nsec = i32::try_from(stamp_nsec_i64)
        .map_err(|_| LioIngressError::TimestampExceedsHeader(identity.source_timestamp_ns))?;
    Ok(Header {
        seq,
        stamp_sec,
        stamp_nsec,
        frame_id: frame_id.to_string(),
    })
}

fn validate_frame(name: &'static str, frame: &str) -> Result<(), LioIngressError> {
    if frame.is_empty()
        || frame.trim() != frame
        || frame.len() > MAX_FRAME_ID_BYTES
        || frame.contains('\0')
    {
        return Err(LioIngressError::InvalidFrame(name));
    }
    Ok(())
}

#[derive(Debug, Error, PartialEq, Eq)]
pub(crate) enum LioIngressError {
    #[error("LIO ingress is closed")]
    Closed,
    #[error("LIO runtime generation must be positive, got {0}")]
    InvalidGeneration(i64),
    #[error("unsupported LIO backend {0}")]
    UnsupportedBackend(i32),
    #[error("invalid {0} frame identifier")]
    InvalidFrame(&'static str),
    #[error("sensor, body, odom, and map frames must be pairwise distinct")]
    AliasedFrames,
    #[error("unexpected LIO backend {0}")]
    WrongBackend(i32),
    #[error("LIO runtime generation mismatch: expected {expected}, got {actual}")]
    WrongGeneration { expected: i64, actual: i64 },
    #[error("source sequence must be positive, got {0}")]
    InvalidSequence(i64),
    #[error("source sequence space exhausted")]
    SequenceExhausted,
    #[error("source timestamp must be non-negative, got {0}")]
    InvalidTimestamp(i64),
    #[error(
        "source sequence is not strictly increasing: previous {previous}, incoming {incoming}"
    )]
    NonIncreasingSequence { previous: i64, incoming: i64 },
    #[error(
        "source timestamp is not strictly increasing: previous {previous}, incoming {incoming}"
    )]
    NonIncreasingTimestamp { previous: i64, incoming: i64 },
    #[error("source sequence {0} cannot be represented by std_msgs.Header")]
    SequenceExceedsHeader(i64),
    #[error("source timestamp {0} cannot be represented by std_msgs.Header")]
    TimestampExceedsHeader(i64),
    #[error("adapter timestamp must be non-negative, got {0}")]
    InvalidAdapterTimestamp(i64),
    #[error("adapter timestamp space exhausted")]
    AdapterTimestampExhausted,
    #[error("odometry frame mismatch: expected {expected}, got {actual}")]
    WrongOdometryFrame { expected: String, actual: String },
    #[error("body frame mismatch: expected {expected}, got {actual}")]
    WrongBodyFrame { expected: String, actual: String },
    #[error("no accepted odometry is available for cloud correlation")]
    MissingOdometry,
    #[error("the exact accepted odometry pose is no longer retained")]
    MissingFrozenOdometry,
    #[error("mapping registration has no immutable map-from-odom base anchor")]
    MissingBaseAnchor,
    #[error("accepted odometry contains an invalid full-SE(3) pose")]
    InvalidOdometryPose,
    #[error(
        "cloud correlation is non-causal: cloud timestamp {cloud_timestamp_ns}, odometry timestamp {odometry_timestamp_ns}"
    )]
    NonCausalOdometryCorrelation {
        cloud_timestamp_ns: i64,
        odometry_timestamp_ns: i64,
    },
    #[error("already-registered map cloud is not valid Device ingress")]
    AlreadyRegisteredMapCloud,
    #[error("raw sensor-frame cloud is not valid Device ingress")]
    RawSensorCloud,
    #[error("cloud frame mismatch: expected {expected}, got {actual}")]
    WrongCloudFrame { expected: String, actual: String },
    #[error("big-endian PointCloud2 is not supported")]
    BigEndianCloud,
    #[error("invalid PointCloud2 dimensions, strides, or data length")]
    InvalidCloudLayout,
    #[error("PointCloud2 must contain one packed little-endian float32 x/y/z field")]
    InvalidXyzLayout,
    #[error("PointCloud2 point count overflow")]
    PointCountOverflow,
    #[error("PointCloud2 has {0} points, above the admitted limit")]
    TooManyPoints(i64),
    #[error("invalid LIO point budget: source={source_count} emitted={emitted_count}")]
    InvalidPointBudget {
        source_count: i64,
        emitted_count: i64,
    },
    #[error("native LIO pair skew duration is zero or exceeds the supported range")]
    PairSkewOverflow,
    #[error("PointCloud2 contains a non-finite or non-f32-representable point")]
    NonFinitePoint,
    #[error("mapping registration failed: {0}")]
    Registration(String),
    #[error("mapping registration changed point count from {source_points} to {registered}")]
    RegistrationPointCountMismatch {
        source_points: usize,
        registered: usize,
    },
    #[error("LIO cloud envelope encoding failed: {0}")]
    EnvelopeEncoding(String),
    #[error("LIO envelope is not the canonical full generated encoding")]
    NonCanonicalEnvelope,
    #[error("public LIO envelope frame contract mismatch")]
    EnvelopeFrameMismatch,
    #[error("registration permit is absent or was already consumed")]
    MissingRegistrationPermit,
    #[error("registration permit mapping session or frame generation is stale")]
    StaleMappingGeneration,
    #[error("registration mode changed after the source frame was captured")]
    StaleRelocalizationMode,
    #[error("cloud generation={generation} sequence={sequence} is already prepared")]
    DuplicatePreparedCloud { generation: i64, sequence: i64 },
    #[error("registration permit binding mismatch")]
    PermitBindingMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapping_adapter::{MappingAdapterConfig, MappingSessionSnapshot};
    use async_trait::async_trait;
    use nuwax_transport::{LcmBackend, LcmFrameStream, LcmTransport, TransportResult};
    use std::sync::Arc;
    use std::time::Duration;

    #[derive(Clone, Default)]
    struct RecordingBackend {
        published: Arc<std::sync::Mutex<Vec<RawLcmFrame>>>,
    }

    impl RecordingBackend {
        fn published_count(&self) -> usize {
            self.published.lock().expect("published frames").len()
        }
    }

    #[async_trait]
    impl LcmBackend for RecordingBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            self.published.lock().expect("published frames").push(frame);
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "lio-ingress-recording"
        }
    }

    #[derive(Clone, Default)]
    struct BlockingPublishBackend;

    #[async_trait]
    impl LcmBackend for BlockingPublishBackend {
        async fn publish(&self, _frame: RawLcmFrame) -> TransportResult<()> {
            std::future::pending().await
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "lio-ingress-blocking-publish"
        }
    }

    #[derive(Clone)]
    struct ReplayTransportBackend {
        published: Arc<std::sync::Mutex<Vec<RawLcmFrame>>>,
        frame_tx: tokio::sync::mpsc::UnboundedSender<RawLcmFrame>,
        fail_message_type: Arc<std::sync::Mutex<Option<String>>>,
    }

    impl ReplayTransportBackend {
        fn new() -> (Self, tokio::sync::mpsc::UnboundedReceiver<RawLcmFrame>) {
            let (frame_tx, frame_rx) = tokio::sync::mpsc::unbounded_channel();
            (
                Self {
                    published: Arc::new(std::sync::Mutex::new(Vec::new())),
                    frame_tx,
                    fail_message_type: Arc::new(std::sync::Mutex::new(None)),
                },
                frame_rx,
            )
        }

        fn published_count(&self, message_type: &str) -> usize {
            self.published
                .lock()
                .expect("replay published frames")
                .iter()
                .filter(|frame| frame.msg_type.fqn() == message_type)
                .count()
        }

        fn fail_message_type(&self, message_type: &str) {
            *self
                .fail_message_type
                .lock()
                .expect("replay failure injection") = Some(message_type.to_owned());
        }
    }

    #[async_trait]
    impl LcmBackend for ReplayTransportBackend {
        async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
            self.published
                .lock()
                .expect("replay published frames")
                .push(frame.clone());
            let _ = self.frame_tx.send(frame.clone());
            if self
                .fail_message_type
                .lock()
                .expect("replay failure injection")
                .as_deref()
                == Some(frame.msg_type.fqn())
            {
                return Err(nuwax_transport::TransportError::LcmBackend(format!(
                    "injected {} publication failure",
                    frame.msg_type.fqn()
                )));
            }
            Ok(())
        }

        async fn subscribe(
            &self,
            _topic: LcmTopic,
            _msg_type: Option<LcmMessageType>,
        ) -> LcmFrameStream {
            Box::pin(futures::stream::empty())
        }

        fn name(&self) -> &str {
            "lio-deferred-replay"
        }
    }

    #[derive(Default)]
    struct ReplayCorrectionSink {
        staged: std::sync::atomic::AtomicU64,
        discarded: std::sync::atomic::AtomicU64,
    }

    #[async_trait]
    impl crate::correction_coordinator::CorrectionCandidateSink for ReplayCorrectionSink {
        async fn stage_frame(
            &self,
            _input: &crate::correction_coordinator::CorrectionFrameInput,
        ) -> Result<(), String> {
            self.staged
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            Ok(())
        }

        async fn discard_frame(
            &self,
            _input: &crate::correction_coordinator::CorrectionFrameInput,
        ) {
            self.discarded
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
        }

        async fn apply_candidate(
            &self,
            _candidate: crate::correction_coordinator::CorrectionCandidate,
            _cancellation: crate::correction_coordinator::CorrectionRequestCancellation,
        ) -> Result<crate::correction_coordinator::CorrectionDisposition, String> {
            Err("none correction owner cannot produce a candidate".to_owned())
        }
    }

    struct DeferredReplayFixture {
        handle: GeometryRuntimeHandle,
        backend: ReplayTransportBackend,
        frame_rx: tokio::sync::mpsc::UnboundedReceiver<RawLcmFrame>,
        public_odometry: (LcmTopic, LcmMessageType),
        public_cloud: (LcmTopic, LcmMessageType),
        gate: Arc<LioIngressRoutingGate>,
        ingress: Arc<NativeLioIngress>,
        replay: Arc<NativeDeferredReplayControl>,
        correction_sink: Arc<ReplayCorrectionSink>,
        correction_ingress: crate::correction_coordinator::CorrectionFrameIngress,
        correction_owner: Option<crate::correction_coordinator::CorrectionTransportOwner>,
        correction_phase_a: crate::correction_coordinator::CorrectionPhaseAControl,
        cloud_path: std::path::PathBuf,
    }

    impl DeferredReplayFixture {
        async fn new() -> Self {
            let native_spec = DeviceLioIngressSpec::new_for_backend(
                LIO_BACKEND_FASTLIO2,
                7,
                "lidar",
                "base_link",
                "odom",
                "map",
            )
            .expect("native replay spec");
            let body_from_sensor = BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
            )
            .expect("identity body-from-sensor transform");
            let relocalization_mode_fence =
                Arc::new(crate::correction_coordinator::RelocalizationModeFence::default());
            let probe = Arc::new(
                NativeLioIngress::without_runtime(
                    native_spec.clone(),
                    body_from_sensor.clone(),
                    Duration::from_millis(1),
                )
                .expect("runtime-free native replay probe"),
            );
            probe
                .bind_relocalization_mode_fence(Arc::clone(&relocalization_mode_fence))
                .expect("bind probe correction mode fence before raw capture");
            let source_identity = SourceIdentity {
                backend: LIO_BACKEND_FASTLIO2,
                runtime_generation: 7,
                source_sequence: 1,
                source_timestamp_ns: 1_000_000_000,
            };
            let mut source_odometry = valid_odometry();
            source_odometry.pose.position = [0.5, -0.25, 0.0];
            source_odometry.header = source_header(source_identity, "odom")
                .expect("source-authentic replay odometry header");
            let mut source_cloud = xyz_cloud(
                "lidar",
                &[[0.25, 0.25, 0.0], [0.75, 0.5, 0.0], [1.25, 0.75, 0.1]],
            );
            source_cloud.header = source_header(source_identity, "lidar")
                .expect("source-authentic replay cloud header");
            let source_odometry_wire = nuwax_codec::lcm::schema::encode_odometry(&source_odometry)
                .expect("encoded replay source odometry");
            let source_cloud_wire = nuwax_codec::lcm::schema::encode_point_cloud2(&source_cloud)
                .expect("encoded replay source cloud");
            let source_odometry_binding =
                binding("/test/lio/deferred/source-odometry", "nav_msgs.Odometry");
            let source_cloud_binding =
                binding("/test/lio/deferred/source-cloud", "sensor_msgs.PointCloud2");
            let (_probe_shutdown_tx, mut probe_shutdown_rx) = watch::channel(false);
            LioIngressSink::accept_legacy_cloud(
                probe.as_ref(),
                public_frame(&source_cloud_binding, 1, &source_cloud_wire),
                &mut probe_shutdown_rx,
            )
            .await
            .expect("capture raw replay source cloud at E0");
            LioIngressSink::accept_legacy_odometry(
                probe.as_ref(),
                public_frame(&source_odometry_binding, 1, &source_odometry_wire),
                &mut probe_shutdown_rx,
            )
            .await
            .expect("seal retained raw replay pair");
            let pair = probe
                .take_validated_startup_pair()
                .expect("transfer retained replay pair");
            assert_eq!(
                pair.registration_mode_token,
                Some(RegistrationModeToken {
                    frame_capture_relocalization_epoch: Some(0),
                }),
                "bound raw probe must retain immutable E0 provenance",
            );

            let mut handle = crate::lcm_runtime::build_handle_for_tests()
                .await
                .expect("replay Geometry handle");
            handle.mapping_adapter = Some(Arc::new(parking_lot::Mutex::new(
                MappingAdapter::new_with_frames(mapping_config(), "map", "odom"),
            )));
            let cloud_path = std::env::temp_dir().join(format!(
                "nuwax-lio-deferred-replay-{}-{}.mmap",
                std::process::id(),
                uuid::Uuid::new_v4()
            ));
            *handle.cloud_map_writer.lock() = Some(
                crate::pointcloud::CloudMapShmWriter::new(&cloud_path)
                    .expect("deferred replay cloud-map writer"),
            );
            let (backend, frame_rx) = ReplayTransportBackend::new();
            handle.transport = LcmTransport::new(Arc::new(backend.clone()));
            let public_odometry = binding(
                "/test/lio/deferred/odometry",
                "nuwax_msgs.LioOdometryEnvelope",
            );
            let public_cloud = binding("/test/lio/deferred/cloud", "nuwax_msgs.LioCloudEnvelope");
            let tf = binding("/test/lio/deferred/tf", "tf2_msgs.TFMessage");
            let ingress = Arc::new(
                NativeLioIngress::with_runtime(
                    native_spec,
                    handle.clone(),
                    public_odometry.clone(),
                    public_cloud.clone(),
                    tf.clone(),
                    body_from_sensor,
                    Duration::from_millis(1),
                )
                .expect("runtime-bound native replay ingress"),
            );
            let correction_identity =
                crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
                    runtime_generation: 7,
                    backend_generation: 7,
                    correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                        boot_nonce: 0x1234,
                        generation: 7,
                    },
                };
            let correction_applier = Arc::new(
                GeometryCanonicalCorrectionApplier::new(
                    correction_identity.clone(),
                    "map",
                    "odom",
                    "base_link",
                    handle.state.clone(),
                    handle.transport.clone(),
                    tf,
                )
                .expect("deferred replay correction applier"),
            );
            let correction_sink = Arc::new(ReplayCorrectionSink::default());
            let correction_owner = crate::correction_coordinator::CorrectionTransportOwner::start_with_relocalization_mode_fence(
                    crate::correction_coordinator::CorrectionOwnerSpec::None,
                    correction_identity,
                    "unused-by-none",
                    correction_sink.clone(),
                    Arc::clone(&relocalization_mode_fence),
                )
                .await
                .expect("deferred replay correction owner");
            let correction_ingress = correction_owner.ingress();
            let correction_phase_a = correction_owner.phase_a_control();
            assert!(Arc::ptr_eq(
                &correction_ingress.relocalization_mode_fence(),
                &relocalization_mode_fence,
            ));
            let active: Arc<dyn LioIngressSink> = Arc::new(CorrectionAwareLioIngress::native(
                ingress.clone(),
                correction_ingress.clone(),
                correction_applier.clone(),
            ));
            assert!(Arc::ptr_eq(
                ingress
                    .common
                    .relocalization_mode_fence
                    .get()
                    .expect("runtime ingress correction mode fence"),
                &relocalization_mode_fence,
            ));
            let probe_sink: Arc<dyn LioIngressSink> = probe;
            let gate = Arc::new(LioIngressRoutingGate::new(probe_sink));
            let token = gate
                .bind_deferred(active)
                .expect("bind deferred replay gate");
            let replay = NativeDeferredReplayControl::new(
                ingress.clone(),
                gate.clone(),
                correction_ingress.clone(),
                correction_applier,
                correction_phase_a.clone(),
                token,
                pair,
            );
            Self {
                handle,
                backend,
                frame_rx,
                public_odometry,
                public_cloud,
                gate,
                ingress,
                replay,
                correction_sink,
                correction_ingress,
                correction_owner: Some(correction_owner),
                correction_phase_a,
                cloud_path,
            }
        }

        async fn next_frame(&mut self, binding: &(LcmTopic, LcmMessageType)) -> RawLcmFrame {
            tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    let frame = self.frame_rx.recv().await.expect("replay frame channel");
                    if frame.topic == binding.0 && frame.msg_type == binding.1 {
                        return frame;
                    }
                }
            })
            .await
            .expect("replay frame deadline")
        }

        async fn next_exact_claimed_cloud_frame(
            &mut self,
            expected_session_id: &str,
            expected_map_name: &str,
        ) -> RawLcmFrame {
            let mut frozen_owner_identity: Option<(String, String, u64)> = None;
            let mut frozen_nonzero_identity: Option<DeferredMappingIdentity> = None;
            let (
                frozen_mapping_identity,
                expected_nonce,
                expected_identity,
                expected_topic,
                expected_type,
                expected_hash,
            ) = tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    let mapping_identity = {
                        let phase = self.replay.phase.lock();
                        match &*phase {
                            NativeDeferredReplayPhase::CloudReplaying(identity, _) => {
                                assert_eq!(identity.session_id, expected_session_id);
                                assert_eq!(identity.map_name, expected_map_name);
                                assert!(identity.session_generation > 0);
                                let owner_identity = (
                                    identity.session_id.clone(),
                                    identity.map_name.clone(),
                                    identity.session_generation,
                                );
                                match &frozen_owner_identity {
                                    Some(frozen) => assert_eq!(frozen, &owner_identity),
                                    None => frozen_owner_identity = Some(owner_identity),
                                }
                                if identity.map_frame_generation == 0 {
                                    None
                                } else {
                                    match &frozen_nonzero_identity {
                                        Some(frozen) => assert_eq!(frozen, identity),
                                        None => frozen_nonzero_identity = Some(identity.clone()),
                                    }
                                    Some(identity.clone())
                                }
                            }
                            NativeDeferredReplayPhase::Complete(identity) => {
                                panic!("deferred replay completed before exact cloud: {identity:?}")
                            }
                            NativeDeferredReplayPhase::Poisoned(detail) => {
                                panic!("deferred replay poisoned before exact cloud: {detail}")
                            }
                            NativeDeferredReplayPhase::Closed => {
                                panic!("deferred replay closed before exact cloud")
                            }
                            _ => None,
                        }
                    };
                    let expectation = {
                        let state = self.ingress.pair_state.lock();
                        match &state.public_replay {
                            NativePublicReplayPhase::Expecting(expectation) => Some((
                                expectation.nonce,
                                expectation.kind,
                                expectation.identity,
                                expectation.topic.clone(),
                                expectation.message_type.clone(),
                                expectation.payload_sha256,
                            )),
                            NativePublicReplayPhase::Poisoned(detail) => {
                                panic!("public replay poisoned before exact cloud: {detail}")
                            }
                            NativePublicReplayPhase::Closed => {
                                panic!("public replay closed before exact cloud")
                            }
                            _ => None,
                        }
                    };
                    if let Some(mapping_identity) = mapping_identity {
                        if let Some((nonce, kind, identity, topic, message_type, hash)) =
                            expectation
                        {
                            assert_eq!(kind, NativeReplayFrameKind::Cloud);
                            assert_eq!(topic, self.public_cloud.0);
                            assert_eq!(message_type, self.public_cloud.1);
                            break (mapping_identity, nonce, identity, topic, message_type, hash);
                        }
                    }
                    tokio::task::yield_now().await;
                }
            })
            .await
            .expect("exact claimed cloud expectation deadline");

            let frame = tokio::time::timeout(Duration::from_secs(2), async {
                loop {
                    let frame = self.frame_rx.recv().await.expect("replay frame channel");
                    if frame.topic == expected_topic && frame.msg_type == expected_type {
                        return frame;
                    }
                }
            })
            .await
            .expect("exact claimed cloud frame deadline");
            let observed_hash = crate::map_checksum::sha256_bytes(frame.payload.as_ref());
            assert_eq!(observed_hash, expected_hash);
            let observed_identity =
                decode_public_replay_identity(NativeReplayFrameKind::Cloud, frame.payload.as_ref())
                    .expect("exact claimed cloud source identity");
            assert_eq!(observed_identity, expected_identity);

            {
                let phase = self.replay.phase.lock();
                match &*phase {
                    NativeDeferredReplayPhase::CloudReplaying(current, _)
                        if current == &frozen_mapping_identity => {}
                    NativeDeferredReplayPhase::CloudReplaying(current, _) => {
                        panic!(
                            "exact claimed cloud Mapping identity changed before callback: expected={frozen_mapping_identity:?} observed={current:?}"
                        )
                    }
                    NativeDeferredReplayPhase::Complete(current) => {
                        panic!(
                            "exact claimed cloud Mapping completed before callback: expected={frozen_mapping_identity:?} observed={current:?}"
                        )
                    }
                    NativeDeferredReplayPhase::Claimed(_) => {
                        panic!("exact claimed cloud Mapping returned to Claimed before callback")
                    }
                    NativeDeferredReplayPhase::Startup(_) => {
                        panic!("exact claimed cloud Mapping returned to Startup before callback")
                    }
                    NativeDeferredReplayPhase::Deferred(_) => {
                        panic!("exact claimed cloud Mapping returned to Deferred before callback")
                    }
                    NativeDeferredReplayPhase::Poisoned(detail) => {
                        panic!("deferred replay poisoned before exact cloud callback: {detail}")
                    }
                    NativeDeferredReplayPhase::Closed => {
                        panic!("deferred replay closed before exact cloud callback")
                    }
                }
            }
            let state = self.ingress.pair_state.lock();
            let NativePublicReplayPhase::Expecting(expectation) = &state.public_replay else {
                panic!("exact claimed cloud expectation changed before callback");
            };
            assert_eq!(expectation.nonce, expected_nonce);
            assert_eq!(expectation.kind, NativeReplayFrameKind::Cloud);
            assert_eq!(expectation.identity, expected_identity);
            assert_eq!(expectation.topic, expected_topic);
            assert_eq!(expectation.message_type, expected_type);
            assert_eq!(expectation.payload_sha256, expected_hash);
            drop(state);
            frame
        }

        async fn complete_startup_odometry(&mut self) {
            let replay = self.replay.clone();
            let publication = tokio::spawn(async move { replay.publish_startup_odometry().await });
            let binding = self.public_odometry.clone();
            let frame = self.next_frame(&binding).await;
            let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
            LioIngressSink::accept_public_odometry(self.gate.as_ref(), frame, &mut shutdown_rx)
                .await
                .expect("startup public odometry handler");
            publication
                .await
                .expect("startup odometry publication task")
                .expect("startup odometry post-handler ACK");
        }

        async fn shutdown(mut self) {
            self.replay.begin_phase_a();
            LioIngressSink::close(self.gate.as_ref());
            self.correction_phase_a.begin();
            if let Some(owner) = self.correction_owner.take() {
                owner
                    .shutdown()
                    .await
                    .expect("deferred replay correction shutdown");
            }
            *self.handle.cloud_map_writer.lock() = None;
            let _ = std::fs::remove_file(&self.cloud_path);
        }
    }

    async fn claim_fresh_mapping_session(
        fixture: &DeferredReplayFixture,
        session_id: &str,
        map_name: &str,
        exercise_conflict: bool,
    ) -> (crate::mapping_adapter::MappingSessionSnapshot, u64) {
        let _lifecycle_guard = fixture.handle.mapping_lifecycle_lock.lock().await;
        let mapping = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("deferred replay mapping adapter");
        let mut adapter = mapping.lock();
        let snapshot = adapter
            .start_session(session_id, map_name)
            .expect("fresh deferred Mapping session");
        let map_frame_generation = adapter.global_frame_generation();
        assert_eq!(
            map_frame_generation, 0,
            "fresh session must exercise generation-zero preparation"
        );
        if exercise_conflict {
            let mut conflicting = snapshot.clone();
            conflicting.map_name = format!("{map_name}-conflict");
            let error =
                match fixture
                    .replay
                    .prepare_session(&conflicting, map_frame_generation, &adapter)
                {
                    Err(error) => error,
                    Ok(_) => panic!("conflicting Mapping identity consumed the retained pair"),
                };
            assert!(error.contains("identity changed before claim"));
        }
        assert!(matches!(
            fixture
                .replay
                .prepare_session(&snapshot, map_frame_generation, &adapter)
                .expect("claim retained pair for fresh Mapping session"),
            DeferredSessionClaim::Claimed
        ));
        (snapshot, map_frame_generation)
    }

    #[tokio::test]
    async fn deferred_startup_odometry_is_exactly_once_without_canonical_side_effects() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;

        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0
        );
        assert_eq!(fixture.backend.published_count("tf2_msgs.TFMessage"), 0);
        let ingress = fixture.ingress.common.snapshot();
        assert_eq!(
            ingress.latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 1_000_000_000,
            })
        );
        assert_eq!(ingress.latest_cloud, None);
        let shared = fixture.handle.state.read().await;
        assert_eq!(shared.pose.timestamp_ns, 0);
        assert!(shared.transforms.is_empty());
        assert_eq!(shared.map.revision, 0);
        assert_eq!(shared.cloud_map_revision, 0);
        drop(shared);
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred replay mapping adapter")
                .lock();
            assert!(!adapter.snapshot().active);
            assert_eq!(adapter.global_frame_generation(), 0);
            assert_eq!(adapter.odom_count(), 0);
            assert_eq!(adapter.pointcloud_count(), 0);
        }
        assert_eq!(
            fixture
                .correction_sink
                .staged
                .load(std::sync::atomic::Ordering::Acquire),
            0
        );

        assert!(fixture.replay.publish_startup_odometry().await.is_err());
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1,
            "startup odometry retry must not publish twice"
        );
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_prepare_session_poison_revokes_shared_readiness_first() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;
        fixture.replay.invalidate_retained_pair_for_tests();
        let readiness = fixture.replay.readiness_authority();
        let poison_order = {
            let replay = fixture.replay.clone();
            let readiness = readiness.clone();
            tokio::spawn(async move {
                let detail = replay.wait_poison().await;
                (detail, readiness.is_revoked())
            })
        };

        let mapping = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("deferred replay mapping adapter")
            .clone();
        let error = {
            let _lifecycle_guard = fixture.handle.mapping_lifecycle_lock.lock().await;
            let mut adapter = mapping.lock();
            let snapshot = adapter
                .start_session("invalid-retained-session", "invalid-retained-map")
                .expect("fresh Mapping identity for invalid retained proof");
            let map_frame_generation = adapter.global_frame_generation();
            let error =
                match fixture
                    .replay
                    .prepare_session(&snapshot, map_frame_generation, &adapter)
                {
                    Ok(_) => panic!("invalid retained proof must poison during prepare_session"),
                    Err(error) => error,
                };
            adapter
                .stop_session(&snapshot.session_id)
                .expect("retire invalid retained proof Mapping session");
            error
        };

        let (poison_detail, readiness_visible) = poison_order.await.expect("poison-order observer");
        assert_eq!(poison_detail, error);
        assert!(readiness_visible);
        assert!(readiness.is_revoked());
        assert!(fixture.gate.is_closed());
        assert!(!fixture.correction_phase_a.is_open());
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_startup_odometry_accepts_real_lcm_zero_transport_sequence() {
        let mut fixture = DeferredReplayFixture::new().await;
        let replay = fixture.replay.clone();
        let publication = tokio::spawn(async move { replay.publish_startup_odometry().await });
        let binding = fixture.public_odometry.clone();
        let mut frame = fixture.next_frame(&binding).await;
        assert_eq!(frame.header.seqno, 1);
        frame.header.seqno = 0;

        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        LioIngressSink::accept_public_odometry(fixture.gate.as_ref(), frame, &mut shutdown_rx)
            .await
            .expect("real-LCM startup public odometry handler");
        publication
            .await
            .expect("startup odometry publication task")
            .expect("real-LCM startup odometry post-handler ACK");

        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1
        );
        assert_eq!(
            fixture.ingress.common.snapshot().latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 1_000_000_000,
            })
        );
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_startup_odometry_transfers_cursor_without_alias() {
        let mut fixture = DeferredReplayFixture::new().await;
        {
            let state = fixture.ingress.common.state.lock();
            assert_eq!(state.next_legacy_odometry_sequence, 0);
            assert_eq!(state.prepared_odometry.latest, None);
        }

        fixture.complete_startup_odometry().await;
        {
            let state = fixture.ingress.common.state.lock();
            assert_eq!(state.next_legacy_odometry_sequence, 1);
            assert_eq!(
                state.prepared_odometry.latest,
                Some(CorrelatedOdometry {
                    source_sequence: 1,
                    source_timestamp_ns: 1_000_000_000,
                })
            );
        }

        let second = fixture
            .ingress
            .common
            .prepare_legacy_odometry(2_000_000_000, valid_odometry())
            .expect("first live odometry after startup");
        assert_eq!(second.source_sequence, 2);
        CommonLioIngress::accept_public_odometry(&fixture.ingress.common, second, 2_000_000_000)
            .expect("first live public odometry after startup");
        let third = fixture
            .ingress
            .common
            .prepare_legacy_odometry(3_000_000_000, valid_odometry())
            .expect("second live odometry after startup");
        assert_eq!(third.source_sequence, 3);
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1,
            "cursor transfer must not republish startup odometry"
        );
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_startup_odometry_rejections_do_not_advance_cursor() {
        let mut stale_fixture = DeferredReplayFixture::new().await;
        let replay = stale_fixture.replay.clone();
        let publication = tokio::spawn(async move { replay.publish_startup_odometry().await });
        let binding = stale_fixture.public_odometry.clone();
        let startup_frame = stale_fixture.next_frame(&binding).await;
        let mut stale_envelope = decode_body(
            nuwax_codec::lcm::lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
            &startup_frame.payload,
            nuwax_codec::lcm::lio::decode_lio_odometry_envelope,
        )
        .expect("decode startup odometry for stale frame");
        stale_envelope.source_sequence = 2;
        let stale_identity = SourceIdentity::from_odometry(&stale_envelope);
        stale_envelope.odometry.header =
            source_header(stale_identity, "odom").expect("source-authentic stale odometry header");
        let stale_encoded =
            encode_lio_odometry_envelope(&stale_envelope).expect("encode stale startup odometry");
        let stale_frame = public_frame(&binding, stale_envelope.source_sequence, &stale_encoded);
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        LioIngressSink::accept_public_odometry(
            stale_fixture.gate.as_ref(),
            stale_frame,
            &mut shutdown_rx,
        )
        .await
        .expect("other-sequence startup frame is ignored");
        {
            let state = stale_fixture.ingress.common.state.lock();
            assert_eq!(state.next_legacy_odometry_sequence, 0);
            assert_eq!(state.prepared_odometry.latest, None);
            assert_eq!(state.odometry.latest, None);
        }
        LioIngressSink::accept_public_odometry(
            stale_fixture.gate.as_ref(),
            startup_frame,
            &mut shutdown_rx,
        )
        .await
        .expect("exact startup odometry remains admissible");
        publication
            .await
            .expect("startup publication task")
            .expect("exact startup odometry post-handler ACK");
        assert_eq!(
            stale_fixture
                .ingress
                .common
                .state
                .lock()
                .next_legacy_odometry_sequence,
            1
        );
        stale_fixture.shutdown().await;

        let mut hostile_fixture = DeferredReplayFixture::new().await;
        let replay = hostile_fixture.replay.clone();
        let publication = tokio::spawn(async move { replay.publish_startup_odometry().await });
        let binding = hostile_fixture.public_odometry.clone();
        let startup_frame = hostile_fixture.next_frame(&binding).await;
        let mut hostile_envelope = decode_body(
            nuwax_codec::lcm::lio::LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
            &startup_frame.payload,
            nuwax_codec::lcm::lio::decode_lio_odometry_envelope,
        )
        .expect("decode startup odometry for hostile frame");
        hostile_envelope.source_timestamp_ns += 1;
        let hostile_identity = SourceIdentity::from_odometry(&hostile_envelope);
        hostile_envelope.odometry.header = source_header(hostile_identity, "odom")
            .expect("source-authentic hostile odometry header");
        let hostile_encoded = encode_lio_odometry_envelope(&hostile_envelope)
            .expect("encode hostile startup odometry");
        let hostile_frame =
            public_frame(&binding, hostile_envelope.source_sequence, &hostile_encoded);
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let error = LioIngressSink::accept_public_odometry(
            hostile_fixture.gate.as_ref(),
            hostile_frame,
            &mut shutdown_rx,
        )
        .await
        .expect_err("same-sequence conflicting startup frame must poison replay");
        assert!(error.contains("payload hash mismatch"));
        assert!(publication
            .await
            .expect("hostile startup publication task")
            .is_err());
        {
            let state = hostile_fixture.ingress.common.state.lock();
            assert_eq!(state.next_legacy_odometry_sequence, 0);
            assert_eq!(state.prepared_odometry.latest, None);
            assert_eq!(state.odometry.latest, None);
        }
        hostile_fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_restart_uses_fresh_legacy_odometry_cursor() {
        let mut first = DeferredReplayFixture::new().await;
        first.complete_startup_odometry().await;
        assert_eq!(
            first
                .ingress
                .common
                .state
                .lock()
                .next_legacy_odometry_sequence,
            1
        );
        first.shutdown().await;

        let mut restarted = DeferredReplayFixture::new().await;
        {
            let state = restarted.ingress.common.state.lock();
            assert_eq!(state.next_legacy_odometry_sequence, 0);
            assert_eq!(state.prepared_odometry.latest, None);
        }
        restarted.complete_startup_odometry().await;
        let first_live = restarted
            .ingress
            .common
            .prepare_legacy_odometry(2_000_000_000, valid_odometry())
            .expect("first live odometry after restarted startup");
        assert_eq!(first_live.source_sequence, 2);
        restarted.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn shared_e0_probe_and_owner_accept_retained_cloud_replay() {
        let mut fixture = DeferredReplayFixture::new().await;
        let captured = fixture.correction_ingress.relocalization_mode_snapshot();
        assert_eq!(captured.epoch, 0);
        assert!(!captured.active);
        fixture.complete_startup_odometry().await;
        claim_fresh_mapping_session(&fixture, "shared-fence-session", "shared-fence-map", false)
            .await;

        let replay = fixture.replay.clone();
        let replay_task = tokio::spawn(async move { replay.replay_claimed().await });
        let cloud_binding = fixture.public_cloud.clone();
        let cloud = fixture.next_frame(&cloud_binding).await;
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        LioIngressSink::accept_public_cloud(fixture.gate.as_ref(), cloud, &mut shutdown_rx)
            .await
            .expect("shared E0 retained cloud post-handler ACK");
        replay_task
            .await
            .expect("shared E0 replay task")
            .expect("shared E0 retained cloud replay");

        assert!(fixture.gate.is_open());
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            1,
        );
        assert_eq!(
            fixture
                .correction_sink
                .staged
                .load(std::sync::atomic::Ordering::Acquire),
            1,
        );
        let mapping = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("shared E0 replay mapping adapter");
        assert_eq!(mapping.lock().pointcloud_count(), 1);
        let shared = fixture.handle.state.read().await;
        assert!(shared.map.revision > 0);
        assert!(shared.cloud_map_revision > 0);
        drop(shared);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn deferred_replay_resolves_fresh_generation_and_completes_exactly_once() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;
        let (initial_snapshot, initial_frame_generation) =
            claim_fresh_mapping_session(&fixture, "deferred-session", "deferred-map", true).await;
        assert_eq!(initial_frame_generation, 0);

        let replay = fixture.replay.clone();
        let replay_task = tokio::spawn(async move { replay.replay_claimed().await });
        let cloud_binding = fixture.public_cloud.clone();
        let cloud = fixture.next_frame(&cloud_binding).await;
        assert!(
            fixture.handle.mapping_lifecycle_lock.try_lock().is_ok(),
            "public cloud publication must not retain the lifecycle lock"
        );
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        LioIngressSink::accept_public_cloud(fixture.gate.as_ref(), cloud, &mut shutdown_rx)
            .await
            .expect("deferred public cloud post-handler ACK");
        replay_task
            .await
            .expect("deferred replay task")
            .expect("deferred replay completion");

        assert!(fixture.gate.is_open());
        let mapping = fixture
            .handle
            .mapping_adapter
            .as_ref()
            .expect("deferred replay mapping adapter");
        {
            let adapter = mapping.lock();
            let completed_snapshot = adapter.snapshot();
            let completed_frame_generation = adapter.global_frame_generation();
            assert!(fixture.replay.is_complete_for(
                "deferred-session",
                "deferred-map",
                completed_snapshot.session_generation,
                completed_frame_generation,
            ));
            assert_eq!(completed_snapshot.session_id, initial_snapshot.session_id);
            assert_eq!(
                completed_snapshot.session_generation,
                initial_snapshot.session_generation
            );
            assert!(completed_frame_generation > 0);
            assert_eq!(adapter.odom_count(), 1);
            assert_eq!(adapter.pointcloud_count(), 1);
            assert!(matches!(
                fixture
                    .replay
                    .prepare_session(&completed_snapshot, completed_frame_generation, &adapter,)
                    .expect("completed exact retry"),
                DeferredSessionClaim::Complete
            ));
        }
        fixture
            .replay
            .replay_claimed()
            .await
            .expect("completed replay is idempotent");

        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            1
        );
        assert_eq!(fixture.backend.published_count("tf2_msgs.TFMessage"), 1);
        assert_eq!(
            fixture
                .correction_sink
                .staged
                .load(std::sync::atomic::Ordering::Acquire),
            1
        );
        assert_eq!(
            fixture
                .correction_sink
                .discarded
                .load(std::sync::atomic::Ordering::Acquire),
            1
        );
        let shared = fixture.handle.state.read().await;
        assert_eq!(shared.pose.timestamp_ns, 1_000_000_000);
        assert_eq!(shared.transforms.len(), 2);
        assert!(shared.map.revision > 0);
        assert!(shared.cloud_map_revision > 0);
        drop(shared);
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_replay_rejects_retained_cloud_after_relocalization_epoch_edge() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;
        let captured = fixture.correction_ingress.relocalization_mode_snapshot();
        assert_eq!(captured.epoch, 0, "fixture cloud is captured at E0");
        assert!(!captured.active);
        let replay_epoch = fixture
            .correction_ingress
            .relocalization_mode_fence()
            .activate_for_tests();
        assert!(replay_epoch.active);
        assert_ne!(replay_epoch.epoch, captured.epoch, "replay runs at E1");

        claim_fresh_mapping_session(
            &fixture,
            "stale-retained-session",
            "stale-retained-map",
            false,
        )
        .await;
        enum ReplayRaceOutcome {
            Finished(Result<(), String>),
            Cloud(RawLcmFrame),
        }

        let replay = fixture.replay.clone();
        let mut replay_task = tokio::spawn(async move { replay.replay_claimed().await });
        let cloud_binding = fixture.public_cloud.clone();
        let outcome = tokio::time::timeout(Duration::from_secs(1), async {
            tokio::select! {
                result = &mut replay_task => {
                    ReplayRaceOutcome::Finished(result.expect("deferred replay task"))
                }
                cloud = fixture.next_frame(&cloud_binding) => ReplayRaceOutcome::Cloud(cloud),
            }
        })
        .await;
        let error = match outcome {
            Ok(ReplayRaceOutcome::Finished(result)) => {
                result.expect_err("E0 retained cloud must not replay after an E1 mode edge")
            }
            Ok(ReplayRaceOutcome::Cloud(cloud)) => {
                let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
                let handler = LioIngressSink::accept_public_cloud(
                    fixture.gate.as_ref(),
                    cloud,
                    &mut shutdown_rx,
                )
                .await;
                let replay_result =
                    tokio::time::timeout(Duration::from_secs(1), &mut replay_task).await;
                fixture.shutdown().await;
                panic!(
                    "stale E0 retained cloud reached public replay: handler={handler:?}, replay={replay_result:?}"
                );
            }
            Err(_) => {
                replay_task.abort();
                let _ = replay_task.await;
                fixture.shutdown().await;
                panic!("stale E0 retained cloud replay did not finish within one second");
            }
        };
        assert!(error.contains("stale cloud mode provenance"));
        assert!(fixture.gate.is_closed());
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0,
            "stale replay must not publish a cloud"
        );
        assert_eq!(
            fixture
                .correction_sink
                .staged
                .load(std::sync::atomic::Ordering::Acquire),
            0,
            "stale replay must not stage correction evidence"
        );
        assert_eq!(
            fixture
                .correction_sink
                .discarded
                .load(std::sync::atomic::Ordering::Acquire),
            0,
            "stale replay must not discard correction evidence"
        );
        {
            let mapping = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred replay mapping adapter")
                .lock();
            assert_eq!(mapping.pointcloud_count(), 0);
        }
        let shared = fixture.handle.state.read().await;
        assert_eq!(shared.map.revision, 0);
        assert_eq!(shared.cloud_map_revision, 0);
        drop(shared);
        assert!(fixture
            .ingress
            .common
            .state
            .lock()
            .pending_registrations
            .is_empty());
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_phase_a_discards_claim_without_irreversible_output() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;
        claim_fresh_mapping_session(&fixture, "phase-a-session", "phase-a-map", false).await;
        assert!(fixture
            .ingress
            .common
            .state
            .lock()
            .pending_registrations
            .is_empty());

        fixture.replay.begin_phase_a();
        LioIngressSink::close(fixture.gate.as_ref());
        fixture.correction_phase_a.begin();
        let error = fixture
            .replay
            .replay_claimed()
            .await
            .expect_err("Phase A must discard a pre-output claim");
        assert!(error.contains("stopped before irreversible output"));
        assert_eq!(fixture.replay.poison_detail(), None);
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioOdometryEnvelope"),
            1
        );
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0
        );
        assert_eq!(fixture.backend.published_count("tf2_msgs.TFMessage"), 0);
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred replay mapping adapter")
                .lock();
            assert_eq!(adapter.global_frame_generation(), 0);
            assert_eq!(adapter.odom_count(), 0);
            assert_eq!(adapter.pointcloud_count(), 0);
        }
        let shared = fixture.handle.state.read().await;
        assert_eq!(shared.pose.timestamp_ns, 0);
        assert_eq!(shared.map.revision, 0);
        assert_eq!(shared.cloud_map_revision, 0);
        drop(shared);
        fixture.shutdown().await;
    }

    #[tokio::test]
    async fn deferred_post_output_failure_poison_closes_all_admission() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;
        claim_fresh_mapping_session(&fixture, "poison-session", "poison-map", false).await;
        fixture.backend.fail_message_type("tf2_msgs.TFMessage");

        let error = fixture
            .replay
            .replay_claimed()
            .await
            .expect_err("post-output TF failure must poison replay");
        assert!(error.contains("canonical odometry/TF outcome is unknown"));
        assert_eq!(
            fixture.replay.poison_detail().as_deref(),
            Some(error.as_str())
        );
        assert_eq!(fixture.replay.wait_poison().await, error);
        assert!(fixture.gate.is_closed());
        assert!(!fixture.correction_phase_a.is_open());
        assert_eq!(fixture.backend.published_count("tf2_msgs.TFMessage"), 1);
        assert_eq!(
            fixture
                .backend
                .published_count("nuwax_msgs.LioCloudEnvelope"),
            0
        );
        {
            let adapter = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred replay mapping adapter")
                .lock();
            assert!(adapter.global_frame_generation() > 0);
            assert_eq!(adapter.odom_count(), 1);
            assert_eq!(adapter.pointcloud_count(), 0);
        }
        let shared = fixture.handle.state.read().await;
        assert_eq!(shared.pose.timestamp_ns, 1_000_000_000);
        assert_eq!(shared.map.revision, 0);
        assert_eq!(shared.cloud_map_revision, 0);
        drop(shared);
        fixture.shutdown().await;
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 4)]
    async fn deferred_start_mapping_joins_exact_retry_and_rejects_conflict_without_duplication() {
        let mut fixture = DeferredReplayFixture::new().await;
        fixture.complete_startup_odometry().await;
        let (shutdown, listener) = crate::server::production_shutdown_channel(
            fixture.gate.clone(),
            fixture.correction_phase_a.clone(),
            Some(fixture.replay.clone()),
        );
        let service = crate::server::GeometryRuntimeService::new_with_shutdown(
            fixture.handle.clone(),
            shutdown,
        );
        let request = |session_id: &str, map_name: &str| crate::proto::StartMappingRequest {
            session_id: session_id.to_owned(),
            map_name: map_name.to_owned(),
        };

        let first_service = service.clone();
        let mut first = tokio::spawn(async move {
            crate::proto::geometry_runtime_server::GeometryRuntime::start_mapping(
                &first_service,
                tonic::Request::new(crate::proto::StartMappingRequest {
                    session_id: "rpc-deferred-session".to_owned(),
                    map_name: "rpc-deferred-map".to_owned(),
                }),
            )
            .await
        });
        let cloud = fixture
            .next_exact_claimed_cloud_frame("rpc-deferred-session", "rpc-deferred-map")
            .await;

        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        LioIngressSink::accept_public_cloud(fixture.gate.as_ref(), cloud, &mut shutdown_rx)
            .await
            .expect("public cloud callback must not wait on the Mapping transition mutex");
        let first = tokio::time::timeout(Duration::from_secs(2), &mut first)
            .await
            .expect("first StartMapping deadline")
            .expect("first StartMapping task")
            .expect("first StartMapping")
            .into_inner();

        let retry_service = service.clone();
        let mut retry = tokio::spawn(async move {
            crate::proto::geometry_runtime_server::GeometryRuntime::start_mapping(
                &retry_service,
                tonic::Request::new(crate::proto::StartMappingRequest {
                    session_id: "rpc-deferred-session".to_owned(),
                    map_name: "rpc-deferred-map".to_owned(),
                }),
            )
            .await
        });
        let retry = tokio::time::timeout(Duration::from_secs(2), &mut retry)
            .await
            .expect("exact retry deadline")
            .expect("exact retry task")
            .expect("exact retry StartMapping")
            .into_inner();
        assert!(first.active);
        assert_eq!(first.session_id, "rpc-deferred-session");
        assert_eq!(first.map_name, "rpc-deferred-map");
        assert!(retry.active);
        assert!(retry.idempotent_replay);
        assert_eq!(retry.session_generation, first.session_generation);

        let published_odometry = fixture
            .backend
            .published_count("nuwax_msgs.LioOdometryEnvelope");
        let published_cloud = fixture
            .backend
            .published_count("nuwax_msgs.LioCloudEnvelope");
        let published_tf = fixture.backend.published_count("tf2_msgs.TFMessage");
        assert_eq!(
            (published_odometry, published_cloud, published_tf),
            (1, 1, 1)
        );

        let conflict = crate::proto::geometry_runtime_server::GeometryRuntime::start_mapping(
            &service,
            tonic::Request::new(request("rpc-conflict", "rpc-conflict-map")),
        )
        .await
        .expect_err("conflicting active Mapping identity");
        assert_eq!(conflict.code(), tonic::Code::AlreadyExists);
        assert_eq!(fixture.replay.poison_detail(), None);
        {
            let mapping = fixture
                .handle
                .mapping_adapter
                .as_ref()
                .expect("deferred replay mapping adapter")
                .lock();
            let completed = mapping.snapshot();
            assert!(fixture.replay.is_complete_for(
                "rpc-deferred-session",
                "rpc-deferred-map",
                completed.session_generation,
                mapping.global_frame_generation(),
            ));
        }
        assert_eq!(
            (
                fixture
                    .backend
                    .published_count("nuwax_msgs.LioOdometryEnvelope"),
                fixture
                    .backend
                    .published_count("nuwax_msgs.LioCloudEnvelope"),
                fixture.backend.published_count("tf2_msgs.TFMessage"),
            ),
            (published_odometry, published_cloud, published_tf),
            "retry and conflict must not duplicate deferred output"
        );

        drop(service);
        drop(listener);
        fixture.shutdown().await;
    }

    #[derive(Default)]
    struct RoutingGateSink {
        closed: std::sync::atomic::AtomicBool,
        ready_calls: std::sync::atomic::AtomicU64,
        odometry_calls: std::sync::atomic::AtomicU64,
    }

    #[async_trait]
    impl LioIngressSink for RoutingGateSink {
        fn close(&self) {
            self.closed
                .store(true, std::sync::atomic::Ordering::Release);
        }

        async fn wait_startup_ready(
            &self,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.ready_calls
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            Ok(())
        }

        async fn accept_public_odometry(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            self.odometry_calls
                .fetch_add(1, std::sync::atomic::Ordering::AcqRel);
            Ok(())
        }

        async fn accept_public_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_odometry(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }

        async fn accept_legacy_cloud(
            &self,
            _frame: RawLcmFrame,
            _shutdown_rx: &mut watch::Receiver<bool>,
        ) -> Result<(), String> {
            Ok(())
        }
    }

    fn routing_gate_frame(sequence: u32) -> RawLcmFrame {
        RawLcmFrameBuilder::new(
            LcmTopic::new("/test/lio/routing-gate").unwrap(),
            LcmMessageType::new("nuwax_msgs.LioOdometryEnvelope").unwrap(),
        )
        .seqno(sequence)
        .payload(Vec::new())
        .build()
    }

    #[tokio::test]
    async fn routing_gate_is_probe_only_until_atomic_bind_and_closes_synchronously() {
        let probe = Arc::new(RoutingGateSink::default());
        let active = Arc::new(RoutingGateSink::default());
        let gate = LioIngressRoutingGate::new(probe.clone());
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);

        assert!(!gate.is_open());
        gate.wait_startup_ready(&mut shutdown_rx).await.unwrap();
        gate.accept_public_odometry(routing_gate_frame(1), &mut shutdown_rx)
            .await
            .unwrap();
        assert_eq!(
            probe.ready_calls.load(std::sync::atomic::Ordering::Acquire),
            1
        );
        assert_eq!(
            probe
                .odometry_calls
                .load(std::sync::atomic::Ordering::Acquire),
            1
        );
        assert_eq!(
            active
                .odometry_calls
                .load(std::sync::atomic::Ordering::Acquire),
            0
        );

        gate.bind_and_open(active.clone()).unwrap();
        assert!(gate.is_open());
        assert!(probe.closed.load(std::sync::atomic::Ordering::Acquire));
        assert!(gate.bind_and_open(active.clone()).is_err());
        gate.accept_public_odometry(routing_gate_frame(2), &mut shutdown_rx)
            .await
            .unwrap();
        assert_eq!(
            probe
                .odometry_calls
                .load(std::sync::atomic::Ordering::Acquire),
            1
        );
        assert_eq!(
            active
                .odometry_calls
                .load(std::sync::atomic::Ordering::Acquire),
            1
        );

        gate.close();
        assert!(!gate.is_open());
        assert!(active.closed.load(std::sync::atomic::Ordering::Acquire));
        gate.accept_public_odometry(routing_gate_frame(3), &mut shutdown_rx)
            .await
            .unwrap();
        assert_eq!(
            active
                .odometry_calls
                .load(std::sync::atomic::Ordering::Acquire),
            1
        );
        assert!(gate.bind_and_open(active).is_err());
    }

    fn spec() -> DeviceLioIngressSpec {
        DeviceLioIngressSpec::new(7, "lidar", "base_link", "odom", "map").expect("spec")
    }

    fn identity(sequence: i64, timestamp_ns: i64) -> SourceIdentity {
        SourceIdentity {
            backend: LIO_BACKEND_DEVICE,
            runtime_generation: 7,
            source_sequence: sequence,
            source_timestamp_ns: timestamp_ns,
        }
    }

    fn odometry() -> Odometry {
        let mut odometry = Odometry {
            header: Header {
                frame_id: "odom".to_string(),
                ..Header::default()
            },
            child_frame_id: "base_link".to_string(),
            ..Odometry::default()
        };
        odometry.pose.orientation_xyzw[3] = 1.0;
        odometry
    }

    fn valid_odometry() -> Odometry {
        let mut odometry = odometry();
        odometry.pose.orientation_xyzw[3] = 1.0;
        odometry
    }

    fn binding(channel: &str, message_type: &str) -> (LcmTopic, LcmMessageType) {
        (
            LcmTopic::new(channel).expect("test topic"),
            LcmMessageType::new(message_type).expect("test message type"),
        )
    }

    #[tokio::test]
    async fn generated_publication_has_a_bounded_deadline() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test runtime handle");
        handle.transport = LcmTransport::new(Arc::new(BlockingPublishBackend));

        let error = publish_generated_body(
            &handle,
            &binding("/test/lio/bounded", "nuwax_msgs.LioCloudEnvelope"),
            1,
            vec![0; 9],
        )
        .await
        .expect_err("blocked legacy publication must time out");

        assert!(error.contains("timed out"));
    }

    fn runtime_bound_ingress(
        spec: DeviceLioIngressSpec,
        handle: GeometryRuntimeHandle,
    ) -> CommonLioIngress {
        CommonLioIngress::with_runtime(
            spec,
            handle,
            binding("/test/lio/odometry", "nuwax_msgs.LioOdometryEnvelope"),
            binding("/test/lio/cloud", "nuwax_msgs.LioCloudEnvelope"),
            binding("/test/lio/tf", "tf2_msgs.TFMessage"),
        )
    }

    fn public_frame(
        binding: &(LcmTopic, LcmMessageType),
        source_sequence: i64,
        encoded: &[u8],
    ) -> RawLcmFrame {
        RawLcmFrameBuilder::new(binding.0.clone(), binding.1.clone())
            .seqno(u32::try_from(source_sequence).expect("transport sequence"))
            .payload(encoded.get(8..).expect("generated fingerprint").to_vec())
            .build()
    }

    fn xyz_cloud(frame_id: &str, points: &[[f32; 3]]) -> PointCloud2 {
        let mut data = Vec::with_capacity(points.len() * XYZ_POINT_STEP as usize);
        for point in points {
            for coordinate in point {
                data.extend_from_slice(&coordinate.to_le_bytes());
            }
        }
        PointCloud2 {
            header: Header {
                frame_id: frame_id.to_string(),
                ..Header::default()
            },
            height: 1,
            width: points.len() as i32,
            fields: xyz_fields(),
            is_bigendian: false,
            point_step: XYZ_POINT_STEP,
            row_step: points.len() as i32 * XYZ_POINT_STEP,
            data,
            is_dense: true,
        }
    }

    fn mapping_config() -> MappingAdapterConfig {
        MappingAdapterConfig {
            resolution: 0.5,
            width_cells: 20,
            height_cells: 20,
            origin_xy: (-5.0, -5.0),
            voxel_size: 0.05,
            occupancy_algo: nuwax_mapping::cost_mapper::CostAlgoSelector::General,
            initial_safe_radius_meters: 0.0,
        }
    }

    fn active_adapter(map_frame: &str) -> MappingAdapter {
        let mut adapter = MappingAdapter::new_with_frames(mapping_config(), map_frame, "odom");
        adapter
            .start_session("lio-test-session", "lio-test-map")
            .expect("active mapping session");
        adapter
            .feed_pose_in_frame(1.0, 2.0, 0.0, 0.0, "odom")
            .expect("authoritative map <- odom anchor");
        adapter
    }

    fn adapter_registration_state(
        adapter: &MappingAdapter,
    ) -> (MappingSessionSnapshot, u64, u64, u64, u64, usize) {
        (
            adapter.snapshot(),
            adapter.global_frame_generation(),
            adapter.current_revision(),
            adapter.odom_count(),
            adapter.pointcloud_count(),
            adapter.accumulated_count(),
        )
    }

    #[derive(Debug, Clone, PartialEq)]
    struct HostileAdapterSnapshot {
        lifecycle: MappingSessionSnapshot,
        frame_generation: u64,
        global_revision: u64,
        current_revision: u64,
        session_stats: nuwax_mapping::SessionStats,
        odom_count: u64,
        pointcloud_count: u64,
        accumulated_count: usize,
    }

    fn hostile_adapter_snapshot(adapter: &MappingAdapter) -> HostileAdapterSnapshot {
        HostileAdapterSnapshot {
            lifecycle: adapter.snapshot(),
            frame_generation: adapter.global_frame_generation(),
            global_revision: adapter.global_revision(),
            current_revision: adapter.current_revision(),
            session_stats: adapter.session_stats(),
            odom_count: adapter.odom_count(),
            pointcloud_count: adapter.pointcloud_count(),
            accumulated_count: adapter.accumulated_count(),
        }
    }

    type HostilePendingRegistrationSnapshot = ((i64, i64), RegistrationBinding, Vec<Point3<f64>>);

    #[derive(Debug, Clone, PartialEq)]
    struct HostileIngressSnapshot {
        public: CommonLioIngressSnapshot,
        prepared_odometry: Option<CorrelatedOdometry>,
        prepared_cloud: Option<CorrelatedOdometry>,
        next_legacy_odometry_sequence: i64,
        next_legacy_cloud_sequence: i64,
        accepted_odometry_identities: Vec<CorrelatedOdometry>,
        pending_registrations: Vec<HostilePendingRegistrationSnapshot>,
    }

    fn hostile_ingress_snapshot(ingress: &CommonLioIngress) -> HostileIngressSnapshot {
        let state = ingress.state.lock();
        let mut pending_registrations = state
            .pending_registrations
            .iter()
            .map(|(key, pending)| (*key, pending.binding.clone(), pending.map_points.clone()))
            .collect::<Vec<_>>();
        pending_registrations.sort_by_key(|(key, _, _)| *key);
        HostileIngressSnapshot {
            public: CommonLioIngressSnapshot {
                runtime_generation: ingress.spec.runtime_generation,
                closed: state.closed,
                latest_odometry: state.odometry.latest,
                latest_cloud: state.cloud.latest,
                latest_adapter_timestamp_ns: state.latest_adapter_timestamp_ns,
            },
            prepared_odometry: state.prepared_odometry.latest,
            prepared_cloud: state.prepared_cloud.latest,
            next_legacy_odometry_sequence: state.next_legacy_odometry_sequence,
            next_legacy_cloud_sequence: state.next_legacy_cloud_sequence,
            accepted_odometry_identities: state
                .accepted_odometry_identities
                .iter()
                .copied()
                .collect(),
            pending_registrations,
        }
    }

    #[derive(Debug, Clone, Copy)]
    enum HostileLioStream {
        PublicOdometry,
        PublicCloud,
        LegacyCloud,
    }

    #[derive(Debug)]
    struct HostileLioFrameCase {
        name: &'static str,
        stream: HostileLioStream,
        frame: RawLcmFrame,
        expected_error: &'static str,
    }

    fn overwrite_hostile_i32(encoded: &mut [u8], offset: usize, value: i32) {
        encoded[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn overwrite_hostile_i64(encoded: &mut [u8], offset: usize, value: i64) {
        encoded[offset..offset + 8].copy_from_slice(&value.to_be_bytes());
    }

    fn overwrite_hostile_f64(encoded: &mut [u8], offset: usize, value: f64) {
        encoded[offset..offset + 8].copy_from_slice(&value.to_be_bytes());
    }

    fn odometry_pose_offset(envelope: &LioOdometryEnvelope) -> usize {
        8 + 4
            + 4
            + 8 * 3
            + [
                &envelope.odom_frame_id,
                &envelope.body_frame_id,
                &envelope.map_frame_id,
            ]
            .into_iter()
            .map(|value| 4 + value.len() + 1)
            .sum::<usize>()
            + 4 * 3
            + 4
            + envelope.odometry.header.frame_id.len()
            + 1
            + 4
            + envelope.odometry.child_frame_id.len()
            + 1
    }

    fn cloud_registration_offset(envelope: &LioCloudEnvelope) -> usize {
        8 + 4
            + 4
            + 8 * 5
            + [
                &envelope.sensor_frame_id,
                &envelope.body_frame_id,
                &envelope.odom_frame_id,
                &envelope.map_frame_id,
            ]
            .into_iter()
            .map(|value| 4 + value.len() + 1)
            .sum::<usize>()
    }

    struct HostileRuntimeFixture {
        handle: GeometryRuntimeHandle,
        backend: RecordingBackend,
        mapping: Arc<Mutex<MappingAdapter>>,
        ingress: CommonLioIngress,
        prepared_cloud: PreparedRegisteredCloud,
    }

    async fn hostile_runtime_fixture() -> HostileRuntimeFixture {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test runtime handle");
        let backend = RecordingBackend::default();
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        let mapping = Arc::new(Mutex::new(active_adapter("map")));
        handle.mapping_adapter = Some(mapping.clone());
        let ingress = runtime_bound_ingress(spec(), handle.clone());
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, valid_odometry())
            .expect("source-fenced odometry for hostile fixture");
        let prepared_cloud = {
            let adapter = mapping.lock();
            ingress
                .register_cloud(
                    identity(1, 0),
                    CorrelatedOdometry {
                        source_sequence: 1,
                        source_timestamp_ns: 0,
                    },
                    10,
                    xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
                    &adapter,
                )
                .expect("hostile fixture registration permit")
        };
        HostileRuntimeFixture {
            handle,
            backend,
            mapping,
            ingress,
            prepared_cloud,
        }
    }

    async fn assert_hostile_public_cloud_rejection(
        name: &str,
        fixture: &HostileRuntimeFixture,
        frame: RawLcmFrame,
        expected_error: &str,
    ) {
        let adapter_before = hostile_adapter_snapshot(&fixture.mapping.lock());
        let ingress_before = hostile_ingress_snapshot(&fixture.ingress);
        let cloud_map_revision_before = fixture.handle.state.read().await.cloud_map_revision;
        let published_before = fixture.backend.published_count();
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let error =
            match LioIngressSink::accept_public_cloud(&fixture.ingress, frame, &mut shutdown_rx)
                .await
            {
                Ok(()) => panic!("{name}: hostile frame was accepted"),
                Err(error) => error,
            };
        assert!(
            error.contains(expected_error),
            "{name}: expected error containing {expected_error:?}, got {error:?}"
        );
        assert_eq!(
            hostile_adapter_snapshot(&fixture.mapping.lock()),
            adapter_before,
            "{name}: MappingAdapter changed"
        );
        assert_eq!(
            hostile_ingress_snapshot(&fixture.ingress),
            ingress_before,
            "{name}: ingress cursor or permit changed"
        );
        assert_eq!(
            fixture.handle.state.read().await.cloud_map_revision,
            cloud_map_revision_before,
            "{name}: shared cloud-map revision changed"
        );
        assert_eq!(
            fixture.backend.published_count(),
            published_before,
            "{name}: hostile frame triggered publication"
        );
    }

    #[test]
    fn zero_timestamp_is_present_and_later_positive_timestamp_is_accepted() {
        let ingress = CommonLioIngress::new(spec());
        let first = ingress
            .accept_odometry_for_test(identity(1, 0), 100, odometry())
            .expect("timestamp zero is valid");
        let second = ingress
            .accept_odometry_for_test(identity(2, 1), 100, odometry())
            .expect("positive timestamp after zero");
        assert_eq!(first.envelope.source_timestamp_ns, 0);
        assert_eq!(second.envelope.source_timestamp_ns, 1);
        assert_eq!(first.adapter_timestamp_ns, 100);
        assert_eq!(second.adapter_timestamp_ns, 101);
    }

    #[test]
    fn legacy_prepare_does_not_advance_public_cursor() {
        let ingress = CommonLioIngress::new(spec());
        let envelope = ingress
            .prepare_legacy_odometry(0, valid_odometry())
            .expect("prepare legacy envelope");
        assert_eq!(ingress.snapshot().latest_odometry, None);
        let accepted = ingress
            .accept_public_odometry(envelope, 50)
            .expect("self-published public envelope must be admitted once");
        assert_eq!(accepted.envelope.source_timestamp_ns, 0);
        assert_eq!(
            ingress.snapshot().latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 0,
            })
        );
    }

    #[test]
    fn backend_generation_sequence_and_time_are_fenced() {
        let ingress = CommonLioIngress::new(spec());
        let mut wrong_backend = identity(1, 0);
        wrong_backend.backend = 2;
        assert_eq!(
            ingress.accept_odometry_for_test(wrong_backend, 1, odometry()),
            Err(LioIngressError::WrongBackend(2))
        );
        let mut wrong_generation = identity(1, 0);
        wrong_generation.runtime_generation = 6;
        assert_eq!(
            ingress.accept_odometry_for_test(wrong_generation, 1, odometry()),
            Err(LioIngressError::WrongGeneration {
                expected: 7,
                actual: 6
            })
        );
        ingress
            .accept_odometry_for_test(identity(1, 0), 1, odometry())
            .expect("first");
        assert!(matches!(
            ingress.accept_odometry_for_test(identity(1, 1), 2, odometry()),
            Err(LioIngressError::NonIncreasingSequence { .. })
        ));
        assert!(matches!(
            ingress.accept_odometry_for_test(identity(2, 0), 2, odometry()),
            Err(LioIngressError::NonIncreasingTimestamp { .. })
        ));
    }

    #[test]
    fn restarted_generation_rejects_old_envelope_before_cursor_mutation() {
        let old_ingress = CommonLioIngress::new(spec());
        let old_envelope = old_ingress
            .prepare_legacy_odometry(0, valid_odometry())
            .expect("old generation envelope");
        let new_spec = DeviceLioIngressSpec::new(8, "lidar", "base_link", "odom", "map")
            .expect("new generation spec");
        let new_ingress = CommonLioIngress::new(new_spec);

        assert_eq!(
            new_ingress.accept_public_odometry(old_envelope, 10),
            Err(LioIngressError::WrongGeneration {
                expected: 8,
                actual: 7,
            })
        );
        assert_eq!(new_ingress.snapshot().latest_odometry, None);

        let new_envelope = new_ingress
            .prepare_legacy_odometry(0, valid_odometry())
            .expect("new generation envelope");
        new_ingress
            .accept_public_odometry(new_envelope, 10)
            .expect("new generation accepted");
        assert_eq!(
            new_ingress.snapshot().latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 0,
            })
        );
    }

    #[test]
    fn failed_frame_admission_does_not_advance_stream() {
        let ingress = CommonLioIngress::new(spec());
        let mut wrong = odometry();
        wrong.header.frame_id = "map".to_string();
        assert!(matches!(
            ingress.accept_odometry_for_test(identity(1, 0), 1, wrong),
            Err(LioIngressError::WrongOdometryFrame { .. })
        ));
        ingress
            .accept_odometry_for_test(identity(1, 0), 1, odometry())
            .expect("same identity remains admissible");
    }

    #[test]
    fn close_is_fail_closed_and_preserves_snapshot() {
        let ingress = CommonLioIngress::new(spec());
        ingress
            .accept_odometry_for_test(identity(1, 0), 1, odometry())
            .expect("first");
        ingress.close();
        let before = ingress.snapshot();
        assert_eq!(
            ingress.accept_odometry_for_test(identity(2, 1), 2, odometry()),
            Err(LioIngressError::Closed)
        );
        assert_eq!(ingress.snapshot(), before);
    }

    #[test]
    fn cloud_parser_rejects_map_sensor_wrong_frame_and_non_finite_points() {
        let spec = spec();
        assert_eq!(
            parse_odom_xyz_cloud(&xyz_cloud("map", &[[1.0, 2.0, 3.0]]), &spec),
            Err(LioIngressError::AlreadyRegisteredMapCloud)
        );
        assert_eq!(
            parse_odom_xyz_cloud(&xyz_cloud("lidar", &[[1.0, 2.0, 3.0]]), &spec),
            Err(LioIngressError::RawSensorCloud)
        );
        assert!(matches!(
            parse_odom_xyz_cloud(&xyz_cloud("base_link", &[[1.0, 2.0, 3.0]]), &spec),
            Err(LioIngressError::WrongCloudFrame { .. })
        ));
        assert_eq!(
            parse_odom_xyz_cloud(&xyz_cloud("odom", &[[f32::NAN, 2.0, 3.0]]), &spec),
            Err(LioIngressError::NonFinitePoint)
        );
    }

    #[test]
    fn cloud_parser_strictly_checks_layout_and_rebuilds_canonical_xyz() {
        let spec = spec();
        let source = xyz_cloud("odom", &[[1.0, -2.0, 3.5], [4.0, 5.0, 6.0]]);
        let points = parse_odom_xyz_cloud(&source, &spec).expect("parse");
        let rebuilt = rebuild_xyz_cloud(
            &points,
            source_header(identity(3, 2_000_000_004), "map").expect("header"),
        )
        .expect("rebuild");
        assert_eq!(rebuilt.header.frame_id, "map");
        assert_eq!(rebuilt.header.stamp_sec, 2);
        assert_eq!(rebuilt.header.stamp_nsec, 4);
        assert_eq!(rebuilt.point_step, 12);
        assert_eq!(rebuilt.row_step, 24);
        assert_eq!(parse_odom_xyz_cloud(&source, &spec).unwrap(), points);

        let mut bad_stride = source.clone();
        bad_stride.row_step += 1;
        assert_eq!(
            parse_odom_xyz_cloud(&bad_stride, &spec),
            Err(LioIngressError::InvalidCloudLayout)
        );
        let mut duplicate_x = source;
        duplicate_x.fields.push(duplicate_x.fields[0].clone());
        assert_eq!(
            parse_odom_xyz_cloud(&duplicate_x, &spec),
            Err(LioIngressError::InvalidXyzLayout)
        );
    }

    #[test]
    fn pending_registration_revoke_is_hash_and_generation_bound_and_one_shot() {
        let binding = RegistrationBinding {
            runtime_generation: 7,
            cloud_source_sequence: 3,
            correlated_odometry: CorrelatedOdometry {
                source_sequence: 2,
                source_timestamp_ns: 10,
            },
            mapping_session_id: "binding-session".into(),
            mapping_session_generation: 11,
            map_frame_generation: 13,
            envelope_sha256: [0x5a; 32],
            frozen_odom_from_body: Isometry3::identity(),
            base_anchor_map_from_odom: Isometry3::identity(),
            frame_capture_relocalization_epoch: None,
        };
        let ingress = CommonLioIngress::new(spec());
        ingress.state.lock().pending_registrations.insert(
            (7, 3),
            PendingRegistration {
                binding,
                map_points: vec![Point3::new(1.0, 2.0, 3.0)],
            },
        );
        assert!(!ingress.revoke_prepared_cloud(7, 3, [0x6b; 32]));
        assert!(ingress.revoke_prepared_cloud(7, 3, [0x5a; 32]));
        assert!(!ingress.revoke_prepared_cloud(7, 3, [0x5a; 32]));
    }

    #[test]
    fn pending_registration_backlog_is_bounded_and_lagged_entries_are_reclaimed() {
        let ingress = CommonLioIngress::new(spec());
        let adapter = active_adapter("map");
        let correlated_odometry = CorrelatedOdometry {
            source_sequence: 1,
            source_timestamp_ns: 0,
        };
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, odometry())
            .expect("accepted odometry for registration");

        let prepared = (1..=MAX_PENDING_REGISTRATIONS + 2)
            .map(|sequence| {
                let sequence = sequence as i64;
                ingress
                    .register_cloud(
                        identity(sequence, sequence - 1),
                        correlated_odometry,
                        10 + sequence,
                        xyz_cloud("odom", &[[sequence as f32, 2.0, 3.0]]),
                        &adapter,
                    )
                    .expect("bounded pending registration")
            })
            .collect::<Vec<_>>();

        assert_eq!(
            ingress.state.lock().pending_registrations.len(),
            MAX_PENDING_REGISTRATIONS
        );
        let evicted = &prepared[0].envelope;
        let evicted_bytes = encode_lio_cloud_envelope(evicted).expect("encode evicted envelope");
        assert_eq!(
            ingress.accept_public_cloud(evicted.clone(), &evicted_bytes, 100),
            Err(LioIngressError::MissingRegistrationPermit)
        );

        let latest = &prepared[prepared.len() - 1].envelope;
        let latest_bytes = encode_lio_cloud_envelope(latest).expect("encode latest envelope");
        ingress
            .accept_public_cloud(latest.clone(), &latest_bytes, 101)
            .expect("latest retained registration remains consumable");
        assert!(
            ingress.state.lock().pending_registrations.is_empty(),
            "accepting the newest callback must reclaim every skipped older permit"
        );
    }

    #[test]
    fn real_registration_produces_one_shot_zero_timestamp_permit() {
        let ingress = CommonLioIngress::new(spec());
        let adapter = active_adapter("map");
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, odometry())
            .expect("accept odometry");
        let before = adapter_registration_state(&adapter);
        let prepared = ingress
            .register_cloud(
                identity(1, 0),
                CorrelatedOdometry {
                    source_sequence: 1,
                    source_timestamp_ns: 0,
                },
                10,
                xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
                &adapter,
            )
            .expect("read-only registration and permit");
        assert_eq!(adapter_registration_state(&adapter), before);
        assert_eq!(prepared.envelope.cloud.header.frame_id, "map");
        assert_eq!(prepared.envelope.source_timestamp_ns, 0);
        assert_eq!(prepared.envelope.correlated_odometry_source_timestamp_ns, 0);

        let envelope = prepared.envelope;
        let encoded = encode_lio_cloud_envelope(&envelope).expect("encode cloud envelope");
        let accepted = ingress
            .accept_public_cloud(envelope.clone(), &encoded, 10)
            .expect("consume matching permit");
        assert_eq!(
            accepted.binding.correlated_odometry,
            CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 0,
            }
        );
        assert_eq!(
            accepted.binding.mapping_session_generation,
            adapter.snapshot().session_generation
        );
        assert_eq!(
            accepted.binding.map_frame_generation,
            adapter.global_frame_generation()
        );
        assert_eq!(accepted.adapter_timestamp_ns, 11);
        assert!(ingress.state.lock().pending_registrations.is_empty());
        assert!(matches!(
            ingress.accept_public_cloud(envelope, &encoded, 12),
            Err(LioIngressError::NonIncreasingSequence { .. })
        ));
    }

    #[tokio::test]
    async fn registration_epoch_is_captured_before_delayed_public_handler() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test Geometry handle");
        let mapping = Arc::new(Mutex::new(active_adapter("map")));
        handle.mapping_adapter = Some(mapping.clone());
        let cloud_path = std::env::temp_dir().join(format!(
            "nuwax-lio-registration-epoch-{}-{}.mmap",
            std::process::id(),
            uuid::Uuid::new_v4()
        ));
        *handle.cloud_map_writer.lock() = Some(
            crate::pointcloud::CloudMapShmWriter::new(&cloud_path)
                .expect("canonical cloud-map writer"),
        );

        // Give the active relocalization branch a real, current receipt. With
        // the former handler-side overwrite, this turns into a false success.
        assert!(crate::lcm_runtime::handoff_registered_lio_cloud(
            &handle,
            &[Point3::new(1.0, 2.0, 0.0)],
            XYZ_POINT_STEP as u32,
            10,
            1,
            "map",
        )
        .await
        .expect("initial canonical map/cloud commit"));
        let before = {
            let state = handle.state.read().await;
            (state.map.revision, state.cloud_map_revision)
        };

        let inner = Arc::new(runtime_bound_ingress(spec(), handle.clone()));
        inner
            .accept_odometry_for_test(identity(1, 1), 11, valid_odometry())
            .expect("registered-source odometry");
        let correction_identity = crate::correction_coordinator::CorrectionOwnerRuntimeIdentity {
            runtime_generation: 7,
            backend_generation: 7,
            correction_lease: crate::correction_coordinator::CorrectionLeaseId {
                boot_nonce: 0x701,
                generation: 7,
            },
        };
        let correction_applier = Arc::new(
            GeometryCanonicalCorrectionApplier::new(
                correction_identity.clone(),
                "map",
                "odom",
                "base_link",
                handle.state.clone(),
                handle.transport.clone(),
                binding("/test/lio/epoch/tf", "tf2_msgs.TFMessage"),
            )
            .expect("correction applier"),
        );
        let correction_sink = Arc::new(ReplayCorrectionSink::default());
        let owner = crate::correction_coordinator::CorrectionTransportOwner::start(
            crate::correction_coordinator::CorrectionOwnerSpec::None,
            correction_identity,
            "unused-by-none",
            correction_sink.clone(),
        )
        .await
        .expect("correction owner");
        let correction = owner.ingress();
        let ingress = CorrectionAwareLioIngress::device(
            inner.clone(),
            correction.clone(),
            correction_applier,
        );
        let prepared = {
            let _lifecycle_guard = handle.mapping_lifecycle_lock.lock().await;
            let adapter = mapping.lock();
            inner
                .register_cloud(
                    identity(1, 2),
                    CorrelatedOdometry {
                        source_sequence: 1,
                        source_timestamp_ns: 1,
                    },
                    12,
                    xyz_cloud("odom", &[[2.0, 3.0, 0.0]]),
                    &adapter,
                )
                .expect("actual source registration under lifecycle guard")
        };
        let captured = correction.relocalization_mode_snapshot();
        assert_eq!(captured.epoch, 0);
        let activated = correction.relocalization_mode_fence().activate_for_tests();
        assert!(activated.active);
        assert_ne!(activated.epoch, captured.epoch);

        let encoded = encode_lio_cloud_envelope(&prepared.envelope).expect("canonical cloud");
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let error = LioIngressSink::accept_public_cloud(
            &ingress,
            public_frame(
                &binding("/test/lio/cloud", "nuwax_msgs.LioCloudEnvelope"),
                prepared.envelope.source_sequence,
                &encoded,
            ),
            &mut shutdown_rx,
        )
        .await
        .expect_err("pre-admission registration must not be retagged by public delivery");
        assert!(error.contains("committed no map revision"));
        assert_eq!(
            {
                let state = handle.state.read().await;
                (state.map.revision, state.cloud_map_revision)
            },
            before,
            "stale source must not alter canonical map/cloud state"
        );
        assert_eq!(
            correction_sink
                .staged
                .load(std::sync::atomic::Ordering::Acquire),
            0,
            "stale source must not stage correction evidence"
        );

        owner.shutdown().await.expect("correction owner shutdown");
        *handle.cloud_map_writer.lock() = None;
        let _ = std::fs::remove_file(cloud_path);
    }

    #[test]
    fn later_odometry_does_not_invalidate_registered_cloud_permit() {
        let ingress = CommonLioIngress::new(spec());
        let adapter = active_adapter("map");
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, odometry())
            .expect("odometry N");
        let prepared = ingress
            .register_cloud(
                identity(1, 0),
                CorrelatedOdometry {
                    source_sequence: 1,
                    source_timestamp_ns: 0,
                },
                10,
                xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
                &adapter,
            )
            .expect("cloud N permit");
        let encoded = encode_lio_cloud_envelope(&prepared.envelope).expect("encode cloud N");

        ingress
            .accept_odometry_for_test(identity(2, 1), 11, odometry())
            .expect("odometry N+1");
        assert_eq!(
            ingress.snapshot().latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 2,
                source_timestamp_ns: 1,
            })
        );

        let accepted = ingress
            .accept_public_cloud(prepared.envelope.clone(), &encoded, 12)
            .expect("immutable cloud N permit survives odometry N+1");
        assert_eq!(accepted.binding.correlated_odometry.source_sequence, 1);
        assert!(ingress.state.lock().pending_registrations.is_empty());
        assert!(matches!(
            ingress.accept_public_cloud(prepared.envelope, &encoded, 13),
            Err(LioIngressError::NonIncreasingSequence { .. })
        ));
    }

    #[test]
    fn registered_cloud_freezes_exact_correlated_odometry_pose_across_later_race() {
        let ingress = CommonLioIngress::new(spec());
        let adapter = active_adapter("map");
        let mut odometry_n = valid_odometry();
        odometry_n.pose.position = [1.25, -2.5, 0.75];
        let rotation_n = UnitQuaternion::from_euler_angles(0.2, -0.3, 0.4);
        let quaternion_n = rotation_n.quaternion();
        odometry_n.pose.orientation_xyzw = [
            quaternion_n.i,
            quaternion_n.j,
            quaternion_n.k,
            quaternion_n.w,
        ];
        let frozen_n = odometry_pose_isometry(&odometry_n).expect("odom N full-SE3 pose");
        ingress
            .accept_odometry_for_test(identity(1, 100), 10, odometry_n)
            .expect("odometry N");
        let prepared = ingress
            .register_cloud(
                identity(1, 101),
                CorrelatedOdometry {
                    source_sequence: 1,
                    source_timestamp_ns: 100,
                },
                11,
                xyz_cloud("odom", &[[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]]),
                &adapter,
            )
            .expect("cloud N immutable registration");
        let encoded =
            encode_lio_cloud_envelope(&prepared.envelope).expect("cloud N canonical bytes");

        let mut odometry_later = valid_odometry();
        odometry_later.pose.position = [9.0, 8.0, 7.0];
        let rotation_later = UnitQuaternion::from_euler_angles(-0.6, 0.5, -0.4);
        let quaternion_later = rotation_later.quaternion();
        odometry_later.pose.orientation_xyzw = [
            quaternion_later.i,
            quaternion_later.j,
            quaternion_later.k,
            quaternion_later.w,
        ];
        let frozen_later = odometry_pose_isometry(&odometry_later).expect("later full-SE3 pose");
        ingress
            .accept_odometry_for_test(identity(2, 102), 12, odometry_later)
            .expect("later odometry races before cloud callback");

        let accepted = ingress
            .accept_public_cloud(prepared.envelope, &encoded, 13)
            .expect("cloud N consumes its original registration binding");
        assert_eq!(accepted.binding.correlated_odometry.source_sequence, 1);
        assert_eq!(accepted.binding.frozen_odom_from_body, frozen_n);
        assert_ne!(accepted.binding.frozen_odom_from_body, frozen_later);
        assert_eq!(
            crate::correction_coordinator::odom_pose_sha256(
                &accepted.binding.frozen_odom_from_body
            )
            .expect("frozen pose proof"),
            crate::correction_coordinator::odom_pose_sha256(&frozen_n)
                .expect("expected pose proof")
        );
    }

    #[test]
    fn configured_map_frame_survives_registration_and_public_acceptance() {
        let configured_spec =
            DeviceLioIngressSpec::new(7, "lidar", "base_link", "odom", "robot/map")
                .expect("configured map frame");
        let ingress = CommonLioIngress::new(configured_spec);
        let adapter = active_adapter("robot/map");
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, odometry())
            .expect("zero-time odometry");
        let prepared = ingress
            .register_cloud(
                identity(1, 0),
                CorrelatedOdometry {
                    source_sequence: 1,
                    source_timestamp_ns: 0,
                },
                10,
                xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
                &adapter,
            )
            .expect("configured-frame registration");
        assert_eq!(prepared.envelope.map_frame_id, "robot/map");
        assert_eq!(prepared.envelope.cloud.header.frame_id, "robot/map");
        let encoded = encode_lio_cloud_envelope(&prepared.envelope).expect("encode cloud");
        let accepted = ingress
            .accept_public_cloud(prepared.envelope, &encoded, 11)
            .expect("configured-frame public acceptance");
        assert_eq!(accepted.envelope.map_frame_id, "robot/map");
        assert_eq!(accepted.envelope.cloud.header.frame_id, "robot/map");
        assert_eq!(
            accepted.binding.map_frame_generation,
            adapter.global_frame_generation()
        );
    }

    #[test]
    fn rejected_registrations_do_not_mutate_mapping_adapter_or_create_permits() {
        let ingress = CommonLioIngress::new(spec());
        let adapter = active_adapter("map");
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, odometry())
            .expect("accepted odometry");
        let before = adapter_registration_state(&adapter);
        let assert_rejected = |result: Result<PreparedRegisteredCloud, LioIngressError>| {
            assert!(result.is_err());
            assert_eq!(adapter_registration_state(&adapter), before);
            let state = ingress.state.lock();
            assert!(state.pending_registrations.is_empty());
            assert_eq!(state.prepared_cloud.latest, None);
        };
        let correlation = CorrelatedOdometry {
            source_sequence: 1,
            source_timestamp_ns: 0,
        };

        let mut wrong_generation = identity(1, 0);
        wrong_generation.runtime_generation = 6;
        assert_rejected(ingress.register_cloud(
            wrong_generation,
            correlation,
            10,
            xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
            &adapter,
        ));
        for frame in ["map", "lidar", "base_link"] {
            assert_rejected(ingress.register_cloud(
                identity(1, 0),
                correlation,
                10,
                xyz_cloud(frame, &[[1.0, 2.0, 3.0]]),
                &adapter,
            ));
        }
        assert_rejected(ingress.register_cloud(
            identity(1, 0),
            CorrelatedOdometry {
                source_sequence: 2,
                source_timestamp_ns: 1,
            },
            10,
            xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
            &adapter,
        ));
        assert_rejected(ingress.register_cloud(
            identity(1, 0),
            correlation,
            10,
            xyz_cloud("odom", &[[f32::NAN, 2.0, 3.0]]),
            &adapter,
        ));
        let mut oversized = xyz_cloud("odom", &[[1.0, 2.0, 3.0]]);
        oversized.width = i32::try_from(MAX_LIO_SOURCE_POINT_COUNT + 1).expect("bounded count");
        oversized.row_step = oversized.width * XYZ_POINT_STEP;
        oversized.data.clear();
        assert_rejected(ingress.register_cloud(
            identity(1, 0),
            correlation,
            10,
            oversized,
            &adapter,
        ));
    }

    #[tokio::test]
    async fn runtime_bound_odometry_admission_waits_for_lifecycle_lock() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test runtime handle");
        let backend = RecordingBackend::default();
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        let mapping = Arc::new(Mutex::new(active_adapter("map")));
        handle.mapping_adapter = Some(mapping.clone());
        let lifecycle_lock = handle.mapping_lifecycle_lock.clone();
        let public_odometry = binding("/test/lio/odometry", "nuwax_msgs.LioOdometryEnvelope");
        let producer = CommonLioIngress::new(spec());
        let envelope = producer
            .prepare_legacy_odometry(0, valid_odometry())
            .expect("public odometry envelope");
        let encoded = encode_lio_odometry_envelope(&envelope).expect("encode odometry");
        let frame = public_frame(&public_odometry, envelope.source_sequence, &encoded);
        let ingress = Arc::new(runtime_bound_ingress(spec(), handle.clone()));

        let lifecycle_guard = lifecycle_lock.lock().await;
        let task_ingress = ingress.clone();
        let mut task = tokio::spawn(async move {
            let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
            LioIngressSink::accept_public_odometry(task_ingress.as_ref(), frame, &mut shutdown_rx)
                .await
        });
        tokio::task::yield_now().await;
        assert!(
            tokio::time::timeout(Duration::from_millis(25), &mut task)
                .await
                .is_err(),
            "public odometry admission must remain blocked by the lifecycle lock"
        );
        assert_eq!(ingress.snapshot().latest_odometry, None);
        assert_eq!(mapping.lock().odom_count(), 1);

        drop(lifecycle_guard);
        tokio::time::timeout(Duration::from_secs(2), task)
            .await
            .expect("odometry task timeout")
            .expect("odometry task join")
            .expect("odometry handoff");
        assert_eq!(
            ingress.snapshot().latest_odometry,
            Some(CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 0,
            })
        );
        assert_eq!(mapping.lock().odom_count(), 2);
        let state = handle.state.read().await;
        assert_eq!(state.pose.frame_id, "map");
        assert_eq!(state.transforms.len(), 2);
        drop(state);
        assert_eq!(backend.published_count(), 1);
    }

    #[tokio::test]
    async fn hostile_lio_frames_preserve_active_mapping_adapter_state() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test runtime handle");
        let backend = RecordingBackend::default();
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        let mapping = Arc::new(Mutex::new(active_adapter("map")));
        handle.mapping_adapter = Some(mapping.clone());
        let ingress = runtime_bound_ingress(spec(), handle.clone());
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, valid_odometry())
            .expect("source-fenced odometry for hostile fixture");
        let prepared_cloud = {
            let adapter = mapping.lock();
            ingress
                .register_cloud(
                    identity(1, 0),
                    CorrelatedOdometry {
                        source_sequence: 1,
                        source_timestamp_ns: 0,
                    },
                    10,
                    xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
                    &adapter,
                )
                .expect("hostile fixture registration permit")
        };

        let public_odometry_binding =
            binding("/test/lio/odometry", "nuwax_msgs.LioOdometryEnvelope");
        let public_cloud_binding = binding("/test/lio/cloud", "nuwax_msgs.LioCloudEnvelope");
        let legacy_cloud_binding = binding("/test/lio/source/cloud", "sensor_msgs.PointCloud2");
        let producer = CommonLioIngress::new(spec());
        let odometry_envelope = producer
            .prepare_legacy_odometry(0, valid_odometry())
            .expect("canonical hostile odometry seed");
        let odometry_encoded =
            encode_lio_odometry_envelope(&odometry_envelope).expect("encode odometry seed");
        let cloud_encoded =
            encode_lio_cloud_envelope(&prepared_cloud.envelope).expect("encode cloud seed");
        let registration_offset = cloud_registration_offset(&prepared_cloud.envelope);
        let source_count_offset = registration_offset + 4;
        let emitted_count_offset = source_count_offset + 8;
        let row_step_offset = cloud_encoded
            .len()
            .checked_sub(prepared_cloud.envelope.cloud.data.len() + 1 + 4)
            .expect("cloud encoding contains row_step, data, and is_dense");
        let pose_offset = odometry_pose_offset(&odometry_envelope);
        let quaternion_offset = pose_offset + 3 * 8;
        let pose_covariance_offset = quaternion_offset + 4 * 8;
        let twist_linear_offset = pose_covariance_offset + 36 * 8;
        let twist_angular_offset = twist_linear_offset + 3 * 8;
        let twist_covariance_offset = twist_angular_offset + 3 * 8;

        let mutate_i32 = |source: &[u8], offset: usize, value: i32| {
            let mut encoded = source.to_vec();
            overwrite_hostile_i32(&mut encoded, offset, value);
            encoded
        };
        let mutate_i64 = |source: &[u8], offset: usize, value: i64| {
            let mut encoded = source.to_vec();
            overwrite_hostile_i64(&mut encoded, offset, value);
            encoded
        };
        let mutate_f64 = |source: &[u8], offset: usize, value: f64| {
            let mut encoded = source.to_vec();
            overwrite_hostile_f64(&mut encoded, offset, value);
            encoded
        };
        let mut wrong_backend_envelope = odometry_envelope.clone();
        wrong_backend_envelope.backend = LIO_BACKEND_FASTLIO2;
        let wrong_backend_encoded = encode_lio_odometry_envelope(&wrong_backend_envelope)
            .expect("known backend remains codec-valid");
        let mut wrong_generation_envelope = odometry_envelope.clone();
        wrong_generation_envelope.runtime_generation += 1;
        let wrong_generation_encoded = encode_lio_odometry_envelope(&wrong_generation_envelope)
            .expect("positive foreign generation remains codec-valid");
        let mut non_increasing_timestamp_envelope = odometry_envelope.clone();
        non_increasing_timestamp_envelope.source_sequence += 1;
        non_increasing_timestamp_envelope.odometry.header.seq += 1;
        let non_increasing_timestamp_encoded =
            encode_lio_odometry_envelope(&non_increasing_timestamp_envelope)
                .expect("non-increasing stream time remains codec-valid in isolation");
        let mut absent_permit_envelope = prepared_cloud.envelope.clone();
        absent_permit_envelope.source_sequence += 1;
        absent_permit_envelope.source_timestamp_ns += 1;
        absent_permit_envelope.cloud.header.seq += 1;
        absent_permit_envelope.cloud.header.stamp_nsec += 1;
        let absent_permit_encoded = encode_lio_cloud_envelope(&absent_permit_envelope)
            .expect("unpermitted cloud remains codec-valid");
        for (name, encoded) in [
            ("known wrong backend", &wrong_backend_encoded),
            ("wrong runtime generation", &wrong_generation_encoded),
            ("non-increasing sequence", &odometry_encoded),
            (
                "non-increasing timestamp",
                &non_increasing_timestamp_encoded,
            ),
        ] {
            nuwax_codec::lcm::lio::decode_lio_odometry_envelope(encoded)
                .unwrap_or_else(|error| panic!("{name} must pass codec decoding: {error}"));
        }
        nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&absent_permit_encoded)
            .expect("absent-permit cloud must pass codec decoding");
        let mut odometry_trailing = odometry_encoded.clone();
        odometry_trailing.push(0);
        let mut cloud_truncated = cloud_encoded.clone();
        cloud_truncated.pop().expect("cloud seed is non-empty");
        let mut impossible_fields =
            nuwax_codec::lcm::schema::encode_point_cloud2(&xyz_cloud("odom", &[[1.0, 2.0, 3.0]]))
                .expect("encode legacy cloud seed");
        overwrite_hostile_i32(&mut impossible_fields, 8, i32::MAX);

        let cases = vec![
            HostileLioFrameCase {
                name: "odometry-known-wrong-backend",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    wrong_backend_envelope.source_sequence,
                    &wrong_backend_encoded,
                ),
                expected_error: "unexpected LIO backend",
            },
            HostileLioFrameCase {
                name: "odometry-wrong-runtime-generation",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    wrong_generation_envelope.source_sequence,
                    &wrong_generation_encoded,
                ),
                expected_error: "LIO runtime generation mismatch",
            },
            HostileLioFrameCase {
                name: "odometry-non-increasing-sequence",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &odometry_encoded,
                ),
                expected_error: "source sequence is not strictly increasing",
            },
            HostileLioFrameCase {
                name: "odometry-non-increasing-timestamp",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    non_increasing_timestamp_envelope.source_sequence,
                    &non_increasing_timestamp_encoded,
                ),
                expected_error: "source timestamp is not strictly increasing",
            },
            HostileLioFrameCase {
                name: "cloud-absent-registration-permit",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    absent_permit_envelope.source_sequence,
                    &absent_permit_encoded,
                ),
                expected_error: "registration permit is absent or was already consumed",
            },
            HostileLioFrameCase {
                name: "odometry-schema-version",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_i32(&odometry_encoded, 8, 2),
                ),
                expected_error: "unsupported LIO schema_version=2",
            },
            HostileLioFrameCase {
                name: "odometry-backend",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_i32(&odometry_encoded, 12, 99),
                ),
                expected_error: "unknown LIO backend=99",
            },
            HostileLioFrameCase {
                name: "odometry-position-nan",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, pose_offset, f64::NAN),
                ),
                expected_error: "odometry pose position must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-position-infinity",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, pose_offset + 8, f64::INFINITY),
                ),
                expected_error: "odometry pose position must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-quaternion-nan",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, quaternion_offset, f64::NAN),
                ),
                expected_error: "odometry quaternion must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-zero-quaternion",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, quaternion_offset + 3 * 8, 0.0),
                ),
                expected_error: "odometry quaternion must be nonzero and unit length",
            },
            HostileLioFrameCase {
                name: "odometry-non-unit-quaternion",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, quaternion_offset + 3 * 8, 2.0),
                ),
                expected_error: "odometry quaternion must be nonzero and unit length",
            },
            HostileLioFrameCase {
                name: "odometry-linear-twist-nan",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, twist_linear_offset, f64::NAN),
                ),
                expected_error: "odometry twist must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-angular-twist-infinity",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, twist_angular_offset, f64::INFINITY),
                ),
                expected_error: "odometry twist must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-pose-covariance-nan",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, pose_covariance_offset, f64::NAN),
                ),
                expected_error: "odometry covariance must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-twist-covariance-infinity",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &mutate_f64(&odometry_encoded, twist_covariance_offset, f64::INFINITY),
                ),
                expected_error: "odometry covariance must contain only finite values",
            },
            HostileLioFrameCase {
                name: "odometry-trailing-byte",
                stream: HostileLioStream::PublicOdometry,
                frame: public_frame(
                    &public_odometry_binding,
                    odometry_envelope.source_sequence,
                    &odometry_trailing,
                ),
                expected_error: "trailing bytes",
            },
            HostileLioFrameCase {
                name: "cloud-schema-version",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &mutate_i32(&cloud_encoded, 8, 2),
                ),
                expected_error: "unsupported LIO schema_version=2",
            },
            HostileLioFrameCase {
                name: "cloud-backend",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &mutate_i32(&cloud_encoded, 12, 99),
                ),
                expected_error: "unknown LIO backend=99",
            },
            HostileLioFrameCase {
                name: "cloud-raw-registration",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &mutate_i32(
                        &cloud_encoded,
                        registration_offset,
                        nuwax_codec::lcm::lio::LIO_REGISTRATION_RAW_SENSOR,
                    ),
                ),
                expected_error: "registration_state must be REGISTERED_MAP",
            },
            HostileLioFrameCase {
                name: "cloud-truncated",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &cloud_truncated,
                ),
                expected_error: "incomplete input",
            },
            HostileLioFrameCase {
                name: "cloud-emitted-point-budget",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &mutate_i64(
                        &cloud_encoded,
                        emitted_count_offset,
                        MAX_LIO_EMITTED_POINT_COUNT + 1,
                    ),
                ),
                expected_error: "emitted_point_count must be in 1..=100000",
            },
            HostileLioFrameCase {
                name: "cloud-source-point-budget",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &mutate_i64(
                        &cloud_encoded,
                        source_count_offset,
                        MAX_LIO_SOURCE_POINT_COUNT + 1,
                    ),
                ),
                expected_error: "source_point_count must be emitted_point_count..=250000",
            },
            HostileLioFrameCase {
                name: "cloud-row-step-drift",
                stream: HostileLioStream::PublicCloud,
                frame: public_frame(
                    &public_cloud_binding,
                    prepared_cloud.envelope.source_sequence,
                    &mutate_i32(
                        &cloud_encoded,
                        row_step_offset,
                        prepared_cloud.envelope.cloud.row_step + 1,
                    ),
                ),
                expected_error: "PointCloud2 row_step must equal width*point_step",
            },
            HostileLioFrameCase {
                name: "legacy-cloud-impossible-field-count",
                stream: HostileLioStream::LegacyCloud,
                frame: public_frame(&legacy_cloud_binding, 1, &impossible_fields),
                expected_error: "fields_length 2147483647 exceeds remaining",
            },
        ];

        let adapter_before = hostile_adapter_snapshot(&mapping.lock());
        let ingress_before = hostile_ingress_snapshot(&ingress);
        assert_eq!(ingress_before.pending_registrations.len(), 1);
        let cloud_map_revision_before = handle.state.read().await.cloud_map_revision;
        let published_before = backend.published_count();
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);

        for case in cases {
            let result = match case.stream {
                HostileLioStream::PublicOdometry => {
                    LioIngressSink::accept_public_odometry(&ingress, case.frame, &mut shutdown_rx)
                        .await
                }
                HostileLioStream::PublicCloud => {
                    LioIngressSink::accept_public_cloud(&ingress, case.frame, &mut shutdown_rx)
                        .await
                }
                HostileLioStream::LegacyCloud => {
                    LioIngressSink::accept_legacy_cloud(&ingress, case.frame, &mut shutdown_rx)
                        .await
                }
            };
            let error = match result {
                Ok(()) => panic!("{}: hostile frame was accepted", case.name),
                Err(error) => error,
            };
            assert!(
                error.contains(case.expected_error),
                "{}: expected error containing {:?}, got {:?}",
                case.name,
                case.expected_error,
                error
            );
            assert_eq!(
                hostile_adapter_snapshot(&mapping.lock()),
                adapter_before,
                "{}: MappingAdapter changed",
                case.name
            );
            assert_eq!(
                hostile_ingress_snapshot(&ingress),
                ingress_before,
                "{}: ingress cursor or permit changed",
                case.name
            );
            assert_eq!(
                handle.state.read().await.cloud_map_revision,
                cloud_map_revision_before,
                "{}: shared cloud-map revision changed",
                case.name
            );
            assert_eq!(
                backend.published_count(),
                published_before,
                "{}: hostile frame triggered publication",
                case.name
            );
        }
    }

    #[tokio::test]
    async fn hostile_permit_lifecycle_preserves_active_adapter_and_runtime_state() {
        let public_cloud_binding = binding("/test/lio/cloud", "nuwax_msgs.LioCloudEnvelope");

        {
            let fixture = hostile_runtime_fixture().await;
            let mut envelope = fixture.prepared_cloud.envelope.clone();
            envelope.source_sequence += 1;
            envelope.source_timestamp_ns += 1;
            envelope.cloud.header.seq += 1;
            envelope.cloud.header.stamp_nsec += 1;
            let encoded = encode_lio_cloud_envelope(&envelope)
                .expect("absent-permit envelope remains codec-valid");
            nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
                .expect("absent-permit envelope passes codec decoding");
            let frame = public_frame(&public_cloud_binding, envelope.source_sequence, &encoded);
            assert_hostile_public_cloud_rejection(
                "absent-permit",
                &fixture,
                frame,
                "registration permit is absent or was already consumed",
            )
            .await;
        }

        {
            let fixture = hostile_runtime_fixture().await;
            let evicted_envelope = fixture.prepared_cloud.envelope.clone();
            let correlation = CorrelatedOdometry {
                source_sequence: 1,
                source_timestamp_ns: 0,
            };
            let final_sequence =
                i64::try_from(MAX_PENDING_REGISTRATIONS + 1).expect("bounded permit capacity");
            for sequence in 2..=final_sequence {
                let adapter = fixture.mapping.lock();
                fixture
                    .ingress
                    .register_cloud(
                        identity(sequence, sequence - 1),
                        correlation,
                        10 + sequence,
                        xyz_cloud("odom", &[[sequence as f32, 2.0, 3.0]]),
                        &adapter,
                    )
                    .expect("fill bounded permit backlog");
            }
            {
                let state = fixture.ingress.state.lock();
                assert_eq!(state.pending_registrations.len(), MAX_PENDING_REGISTRATIONS);
                assert!(!state.pending_registrations.contains_key(&(7, 1)));
            }
            let encoded = encode_lio_cloud_envelope(&evicted_envelope)
                .expect("evicted-permit envelope remains codec-valid");
            nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
                .expect("evicted-permit envelope passes codec decoding");
            let frame = public_frame(
                &public_cloud_binding,
                evicted_envelope.source_sequence,
                &encoded,
            );
            assert_hostile_public_cloud_rejection(
                "evicted-permit",
                &fixture,
                frame,
                "registration permit is absent or was already consumed",
            )
            .await;
            assert_eq!(
                fixture.ingress.state.lock().pending_registrations.len(),
                MAX_PENDING_REGISTRATIONS,
                "an evicted replay must not leak or shrink the bounded backlog"
            );
        }

        {
            let fixture = hostile_runtime_fixture().await;
            let envelope = fixture.prepared_cloud.envelope.clone();
            let encoded = encode_lio_cloud_envelope(&envelope)
                .expect("consumed-permit envelope remains codec-valid");
            nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
                .expect("consumed-permit envelope passes codec decoding");
            let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
            LioIngressSink::accept_public_cloud(
                &fixture.ingress,
                public_frame(&public_cloud_binding, envelope.source_sequence, &encoded),
                &mut shutdown_rx,
            )
            .await
            .expect("first delivery consumes the permit");
            assert!(fixture
                .ingress
                .state
                .lock()
                .pending_registrations
                .is_empty());
            assert_hostile_public_cloud_rejection(
                "consumed-permit",
                &fixture,
                public_frame(&public_cloud_binding, envelope.source_sequence, &encoded),
                "source sequence is not strictly increasing",
            )
            .await;
        }

        {
            let fixture = hostile_runtime_fixture().await;
            let envelope = fixture.prepared_cloud.envelope.clone();
            let encoded = encode_lio_cloud_envelope(&envelope)
                .expect("stale-permit envelope remains codec-valid");
            nuwax_codec::lcm::lio::decode_lio_cloud_envelope(&encoded)
                .expect("stale-permit envelope passes codec decoding");
            {
                let mut adapter = fixture.mapping.lock();
                adapter
                    .stop_session("lio-test-session")
                    .expect("stop fixture mapping generation");
                adapter
                    .start_session("lio-test-session-2", "lio-test-map")
                    .expect("advance fixture mapping generation");
                adapter
                    .feed_pose_in_frame(1.0, 2.0, 0.0, 0.0, "odom")
                    .expect("re-establish map <- odom anchor");
            }
            assert_hostile_public_cloud_rejection(
                "stale-permit",
                &fixture,
                public_frame(&public_cloud_binding, envelope.source_sequence, &encoded),
                "registration permit mapping session or frame generation is stale",
            )
            .await;
            assert_eq!(
                fixture.ingress.state.lock().pending_registrations.len(),
                1,
                "stale rejection must retain its one-shot permit"
            );

            *fixture.mapping.lock() = active_adapter("map");
            let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
            LioIngressSink::accept_public_cloud(
                &fixture.ingress,
                public_frame(&public_cloud_binding, envelope.source_sequence, &encoded),
                &mut shutdown_rx,
            )
            .await
            .expect("the retained permit remains consumable after generation restoration");
            assert!(fixture
                .ingress
                .state
                .lock()
                .pending_registrations
                .is_empty());
        }
    }

    #[tokio::test]
    async fn runtime_bound_inactive_odometry_sink_reports_zero_commit_without_side_effects() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test runtime handle");
        let backend = RecordingBackend::default();
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        let mapping = handle
            .mapping_adapter
            .as_ref()
            .expect("test mapping adapter")
            .clone();
        let public_odometry = binding("/test/lio/odometry", "nuwax_msgs.LioOdometryEnvelope");
        let producer = CommonLioIngress::new(spec());
        let envelope = producer
            .prepare_legacy_odometry(0, valid_odometry())
            .expect("public odometry envelope");
        let encoded = encode_lio_odometry_envelope(&envelope).expect("encode odometry");
        let frame = public_frame(&public_odometry, envelope.source_sequence, &encoded);
        let ingress = runtime_bound_ingress(spec(), handle.clone());

        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let error = LioIngressSink::accept_public_odometry(&ingress, frame, &mut shutdown_rx)
            .await
            .expect_err("inactive adapter must reject the sink handoff");
        assert!(error.contains("committed no observation"));
        let adapter_state = {
            let adapter = mapping.lock();
            (
                adapter.odom_count(),
                adapter.pointcloud_count(),
                adapter.current_revision(),
                adapter.accumulated_count(),
            )
        };
        assert_eq!(adapter_state, (0, 0, 0, 0));
        let state = handle.state.read().await;
        assert!(state.pose.frame_id.is_empty());
        assert!(state.transforms.is_empty());
        assert_eq!(state.cloud_map_revision, 0);
        drop(state);
        assert_eq!(backend.published_count(), 0);
    }

    #[tokio::test]
    async fn runtime_bound_stopped_cloud_sink_reports_zero_commit_and_consumes_permit_once() {
        let mut handle = crate::lcm_runtime::build_handle_for_tests()
            .await
            .expect("test runtime handle");
        let backend = RecordingBackend::default();
        handle.transport = LcmTransport::new(Arc::new(backend.clone()));
        let mapping = Arc::new(Mutex::new(active_adapter("map")));
        handle.mapping_adapter = Some(mapping.clone());
        let public_cloud = binding("/test/lio/cloud", "nuwax_msgs.LioCloudEnvelope");
        let ingress = runtime_bound_ingress(spec(), handle.clone());
        ingress
            .accept_odometry_for_test(identity(1, 0), 10, valid_odometry())
            .expect("source-fenced odometry");
        let prepared = {
            let adapter = mapping.lock();
            ingress
                .register_cloud(
                    identity(1, 0),
                    CorrelatedOdometry {
                        source_sequence: 1,
                        source_timestamp_ns: 0,
                    },
                    10,
                    xyz_cloud("odom", &[[1.0, 2.0, 3.0]]),
                    &adapter,
                )
                .expect("real registered cloud permit")
        };
        let encoded = encode_lio_cloud_envelope(&prepared.envelope).expect("encode cloud");
        let frame = public_frame(&public_cloud, prepared.envelope.source_sequence, &encoded);
        mapping
            .lock()
            .stop_session("lio-test-session")
            .expect("stop before public cloud delivery");
        let before = {
            let adapter = mapping.lock();
            (
                adapter.odom_count(),
                adapter.pointcloud_count(),
                adapter.current_revision(),
                adapter.accumulated_count(),
            )
        };

        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let error = LioIngressSink::accept_public_cloud(&ingress, frame.clone(), &mut shutdown_rx)
            .await
            .expect_err("stopped mapping must reject cloud zero commit");
        assert!(error.contains("committed no map revision"));
        let after = {
            let adapter = mapping.lock();
            (
                adapter.odom_count(),
                adapter.pointcloud_count(),
                adapter.current_revision(),
                adapter.accumulated_count(),
            )
        };
        assert_eq!(after, before);
        assert_eq!(handle.state.read().await.cloud_map_revision, 0);
        assert_eq!(backend.published_count(), 0);
        assert!(ingress.state.lock().pending_registrations.is_empty());
        assert!(
            LioIngressSink::accept_public_cloud(&ingress, frame, &mut shutdown_rx)
                .await
                .is_err()
        );
        assert!(ingress.state.lock().pending_registrations.is_empty());
    }

    #[test]
    fn deterministic_downsample_enforces_v2_boundaries() {
        let emitted_limit =
            usize::try_from(MAX_LIO_EMITTED_POINT_COUNT).expect("emitted limit fits usize");
        let source_limit =
            usize::try_from(MAX_LIO_SOURCE_POINT_COUNT).expect("source limit fits usize");
        assert_eq!(emitted_limit, 100_000);
        assert_eq!(source_limit, 250_000);

        let exact = (0..emitted_limit)
            .map(|index| Point3::new(index as f64, 0.0, 0.0))
            .collect::<Vec<_>>();
        assert_eq!(deterministic_downsample(exact.clone()), exact);

        for source_len in [100_001, source_limit] {
            let source = (0..source_len)
                .map(|index| Point3::new(index as f64, 0.0, 0.0))
                .collect::<Vec<_>>();
            let sampled = deterministic_downsample(source);
            assert_eq!(sampled.len(), emitted_limit);
            for (sample_index, point) in sampled.iter().enumerate() {
                let expected_source_index = sample_index * source_len / emitted_limit;
                assert_eq!(point.x, expected_source_index as f64);
            }
        }

        let mut oversized = xyz_cloud("lidar", &[[0.0, 0.0, 0.0]]);
        oversized.width =
            i32::try_from(MAX_LIO_SOURCE_POINT_COUNT + 1).expect("oversized count fits i32");
        oversized.row_step = oversized.width * XYZ_POINT_STEP;
        oversized.data.clear();
        assert_eq!(
            parse_sensor_xyz_cloud(&oversized, &spec()),
            Err(LioIngressError::TooManyPoints(250_001))
        );
    }

    #[test]
    fn native_pairing_is_order_independent() {
        let make_ingress = || {
            let native_spec = DeviceLioIngressSpec::new_for_backend(
                LIO_BACKEND_FASTLIO2,
                7,
                "lidar",
                "base_link",
                "odom",
                "map",
            )
            .expect("native spec");
            let body_from_sensor = BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
            )
            .expect("identity body-from-sensor transform");
            let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
            NativeLioIngress {
                common: CommonLioIngress::new(native_spec),
                body_from_sensor,
                pair_max_skew_ns: 10,
                pair_state: Mutex::new(NativePairState::default()),
                source_pair_ready_tx,
                source_pair_ready_rx,
            }
        };
        let native_identity = |source_sequence, source_timestamp_ns| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns,
        };

        let odometry_first = make_ingress();
        assert!(odometry_first
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(1, 1_000),
                odometry: valid_odometry(),
            })
            .is_none());
        let pair = odometry_first
            .insert_cloud(NativeCloudSample {
                identity: native_identity(1, 1_000),
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .expect("cloud pairs with prior odometry");
        assert_eq!(pair.odometry.identity.source_timestamp_ns, 1_000);
        assert_eq!(pair.cloud.identity.source_timestamp_ns, 1_000);

        let cloud_first = make_ingress();
        assert!(cloud_first
            .insert_cloud(NativeCloudSample {
                identity: native_identity(2, 2_000),
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .is_none());
        let pair = cloud_first
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(2, 2_000),
                odometry: valid_odometry(),
            })
            .expect("odometry pairs with prior cloud");
        assert_eq!(pair.odometry.identity.source_timestamp_ns, 2_000);
        assert_eq!(pair.cloud.identity.source_timestamp_ns, 2_000);
    }

    #[test]
    fn native_pairing_rejects_excess_skew() {
        let native_spec = DeviceLioIngressSpec::new_for_backend(
            LIO_BACKEND_FASTLIO2,
            7,
            "lidar",
            "base_link",
            "odom",
            "map",
        )
        .expect("native spec");
        let body_from_sensor = BodyFromSensorTransform::from_json(
            r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
        )
        .expect("identity body-from-sensor transform");
        let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
        let ingress = NativeLioIngress {
            common: CommonLioIngress::new(native_spec),
            body_from_sensor,
            pair_max_skew_ns: 100,
            pair_state: Mutex::new(NativePairState::default()),
            source_pair_ready_tx,
            source_pair_ready_rx,
        };
        let native_identity = |source_sequence, source_timestamp_ns| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns,
        };

        assert!(ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(1, 1_000),
                odometry: valid_odometry(),
            })
            .is_none());
        assert!(
            ingress
                .insert_cloud(NativeCloudSample {
                    identity: native_identity(1, 1_101),
                    source_point_count: 1,
                    sensor_points: vec![Point3::origin()],
                    registration_mode_token: RegistrationModeToken::default(),
                })
                .is_none(),
            "101 ns skew must exceed the inclusive 100 ns limit"
        );
        assert!(ingress
            .insert_cloud(NativeCloudSample {
                identity: native_identity(2, 1_100),
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .is_none());
        let pair = ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(2, 1_202),
                odometry: valid_odometry(),
            })
            .expect("watermark retains the 100 ns boundary candidate");
        assert_eq!(pair.cloud.identity.source_sequence, 2);
        let state = ingress.pair_state.lock();
        assert!(state.clouds.is_empty());
    }

    #[test]
    fn native_pairing_waits_for_exact_odometry_before_watermark() {
        let native_spec = DeviceLioIngressSpec::new_for_backend(
            LIO_BACKEND_FASTLIO2,
            7,
            "lidar",
            "base_link",
            "odom",
            "map",
        )
        .expect("native spec");
        let body_from_sensor = BodyFromSensorTransform::from_json(
            r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
        )
        .expect("identity body-from-sensor transform");
        let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
        let ingress = NativeLioIngress {
            common: CommonLioIngress::new(native_spec),
            body_from_sensor,
            pair_max_skew_ns: 10,
            pair_state: Mutex::new(NativePairState::default()),
            source_pair_ready_tx,
            source_pair_ready_rx,
        };
        let native_identity = |source_sequence, source_timestamp_ns| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns,
        };

        assert!(ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(1, 995),
                odometry: valid_odometry(),
            })
            .is_none());
        assert!(
            ingress
                .insert_cloud(NativeCloudSample {
                    identity: native_identity(1, 1_000),
                    source_point_count: 1,
                    sensor_points: vec![Point3::origin()],
                    registration_mode_token: RegistrationModeToken::default(),
                })
                .is_none(),
            "an older in-skew odometry must not greedily consume the cloud"
        );
        assert_eq!(ingress.pair_state.lock().clouds.len(), 1);

        let pair = ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(2, 1_000),
                odometry: valid_odometry(),
            })
            .expect("exact odometry finalizes the pending cloud");
        assert_eq!(pair.odometry.identity.source_sequence, 2);
        assert_eq!(pair.odometry.identity.source_timestamp_ns, 1_000);
        assert!(ingress.pair_state.lock().clouds.is_empty());
    }

    #[test]
    fn native_pairing_finalizes_nearest_fallback_at_watermark() {
        let native_spec = DeviceLioIngressSpec::new_for_backend(
            LIO_BACKEND_FASTLIO2,
            7,
            "lidar",
            "base_link",
            "odom",
            "map",
        )
        .expect("native spec");
        let body_from_sensor = BodyFromSensorTransform::from_json(
            r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
        )
        .expect("identity body-from-sensor transform");
        let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
        let ingress = NativeLioIngress {
            common: CommonLioIngress::new(native_spec),
            body_from_sensor,
            pair_max_skew_ns: 10,
            pair_state: Mutex::new(NativePairState::default()),
            source_pair_ready_tx,
            source_pair_ready_rx,
        };
        let native_identity = |source_sequence, source_timestamp_ns| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns,
        };

        assert!(ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(1, 995),
                odometry: valid_odometry(),
            })
            .is_none());
        assert!(ingress
            .insert_cloud(NativeCloudSample {
                identity: native_identity(1, 1_000),
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .is_none());

        let pair = ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(2, 1_015),
                odometry: valid_odometry(),
            })
            .expect("watermark finalizes against the nearest in-skew fallback");
        assert_eq!(pair.odometry.identity.source_sequence, 1);
        assert_eq!(pair.odometry.identity.source_timestamp_ns, 995);
        assert!(ingress.pair_state.lock().clouds.is_empty());
    }

    #[test]
    fn native_pairing_drops_cloud_without_candidate_after_watermark() {
        let native_spec = DeviceLioIngressSpec::new_for_backend(
            LIO_BACKEND_FASTLIO2,
            7,
            "lidar",
            "base_link",
            "odom",
            "map",
        )
        .expect("native spec");
        let body_from_sensor = BodyFromSensorTransform::from_json(
            r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
        )
        .expect("identity body-from-sensor transform");
        let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
        let ingress = NativeLioIngress {
            common: CommonLioIngress::new(native_spec),
            body_from_sensor,
            pair_max_skew_ns: 10,
            pair_state: Mutex::new(NativePairState::default()),
            source_pair_ready_tx,
            source_pair_ready_rx,
        };
        let native_identity = |source_sequence, source_timestamp_ns| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns,
        };

        assert!(ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(1, 980),
                odometry: valid_odometry(),
            })
            .is_none());
        assert!(ingress
            .insert_cloud(NativeCloudSample {
                identity: native_identity(1, 1_000),
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .is_none());
        assert!(ingress
            .insert_odometry(NativeOdometrySample {
                identity: native_identity(2, 1_011),
                odometry: valid_odometry(),
            })
            .is_none());

        assert!(
            ingress.pair_state.lock().clouds.is_empty(),
            "watermark must evict a cloud once no future odometry can be in skew"
        );
    }

    #[test]
    fn native_pair_caches_are_bounded_and_evict_oldest() {
        let make_ingress = || {
            let native_spec = DeviceLioIngressSpec::new_for_backend(
                LIO_BACKEND_FASTLIO2,
                7,
                "lidar",
                "base_link",
                "odom",
                "map",
            )
            .expect("native spec");
            let body_from_sensor = BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
            )
            .expect("identity body-from-sensor transform");
            let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
            NativeLioIngress {
                common: CommonLioIngress::new(native_spec),
                body_from_sensor,
                pair_max_skew_ns: 1,
                pair_state: Mutex::new(NativePairState::default()),
                source_pair_ready_tx,
                source_pair_ready_rx,
            }
        };
        let native_identity = |source_sequence| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns: source_sequence * 1_000,
        };

        let odometry_ingress = make_ingress();
        for sequence in 1..=i64::try_from(MAX_NATIVE_PAIR_CACHE + 1).unwrap() {
            assert!(odometry_ingress
                .insert_odometry(NativeOdometrySample {
                    identity: native_identity(sequence),
                    odometry: valid_odometry(),
                })
                .is_none());
        }
        let state = odometry_ingress.pair_state.lock();
        assert_eq!(state.odometry.len(), MAX_NATIVE_PAIR_CACHE);
        assert_eq!(state.odometry.front().unwrap().identity.source_sequence, 2);
        assert_eq!(
            state.odometry.back().unwrap().identity.source_sequence,
            i64::try_from(MAX_NATIVE_PAIR_CACHE + 1).unwrap()
        );
        drop(state);

        let cloud_ingress = make_ingress();
        for sequence in 1..=i64::try_from(MAX_NATIVE_PAIR_CACHE + 1).unwrap() {
            assert!(cloud_ingress
                .insert_cloud(NativeCloudSample {
                    identity: native_identity(sequence),
                    source_point_count: 1,
                    sensor_points: vec![Point3::origin()],
                    registration_mode_token: RegistrationModeToken::default(),
                })
                .is_none());
        }
        let state = cloud_ingress.pair_state.lock();
        assert_eq!(state.clouds.len(), MAX_NATIVE_PAIR_CACHE);
        assert_eq!(state.clouds.front().unwrap().identity.source_sequence, 2);
        assert_eq!(
            state.clouds.back().unwrap().identity.source_sequence,
            i64::try_from(MAX_NATIVE_PAIR_CACHE + 1).unwrap()
        );
    }

    #[test]
    fn native_origin_pose_applies_body_from_sensor_exactly_once() {
        let body_from_sensor = BodyFromSensorTransform::from_json(
            r#"{"translation_m":[1.0,2.0,3.0],"rotation_row_major":[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]}"#,
        )
        .expect("proper body-from-sensor transform");
        let native_identity = |source_sequence| SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence,
            source_timestamp_ns: source_sequence,
        };
        let origin_odometry = valid_odometry();
        assert_eq!(origin_odometry.pose.position, [0.0; 3]);
        let pair = NativeSourcePair {
            odometry: NativeOdometrySample {
                identity: native_identity(1),
                odometry: origin_odometry,
            },
            cloud: NativeCloudSample {
                identity: native_identity(1),
                source_point_count: 2,
                sensor_points: vec![Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
                registration_mode_token: RegistrationModeToken::default(),
            },
        };

        let transformed =
            transform_native_pair(&pair, &body_from_sensor).expect("valid origin pose transforms");
        assert_eq!(
            transformed,
            vec![Point3::new(1.0, 3.0, 3.0), Point3::new(0.0, 2.0, 3.0)]
        );
    }

    #[tokio::test]
    async fn runtime_free_native_probe_requires_an_authentic_valid_raw_pair_for_ready() {
        fn native_probe() -> NativeLioIngress {
            NativeLioIngress::without_runtime(
                DeviceLioIngressSpec::new_for_backend(
                    LIO_BACKEND_FASTLIO2,
                    7,
                    "lidar",
                    "base_link",
                    "odom",
                    "map",
                )
                .expect("native probe spec"),
                BodyFromSensorTransform::from_json(
                    r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
                )
                .expect("identity body-from-sensor transform"),
                Duration::from_millis(1),
            )
            .expect("runtime-free native probe")
        }

        fn source_frame(
            binding: &(LcmTopic, LcmMessageType),
            sequence: i64,
            encoded: &[u8],
        ) -> RawLcmFrame {
            public_frame(binding, sequence, encoded)
        }

        let odometry_binding = binding("/test/native/source-odom", "nav_msgs.Odometry");
        let cloud_binding = binding("/test/native/source-cloud", "sensor_msgs.PointCloud2");
        let odometry = valid_odometry();
        let odometry_wire =
            nuwax_codec::lcm::schema::encode_odometry(&odometry).expect("encoded source odometry");
        let valid_cloud_wire =
            nuwax_codec::lcm::schema::encode_point_cloud2(&xyz_cloud("lidar", &[[1.0, 2.0, 3.0]]))
                .expect("encoded source cloud");
        let invalid_cloud_wire =
            nuwax_codec::lcm::schema::encode_point_cloud2(&xyz_cloud("odom", &[[1.0, 2.0, 3.0]]))
                .expect("encoded wrong-frame source cloud");
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);

        let invalid_probe = native_probe();
        LioIngressSink::accept_legacy_odometry(
            &invalid_probe,
            source_frame(&odometry_binding, 1, &odometry_wire),
            &mut shutdown_rx,
        )
        .await
        .expect("valid raw odometry is cached");
        assert!(LioIngressSink::accept_legacy_cloud(
            &invalid_probe,
            source_frame(&cloud_binding, 1, &invalid_cloud_wire),
            &mut shutdown_rx,
        )
        .await
        .is_err());
        assert!(!*invalid_probe.source_pair_ready_rx.borrow());

        let valid_probe = native_probe();
        assert!(valid_probe.common.runtime.is_none());
        LioIngressSink::accept_legacy_cloud(
            &valid_probe,
            source_frame(&cloud_binding, 1, &valid_cloud_wire),
            &mut shutdown_rx,
        )
        .await
        .expect("valid raw cloud is cached");
        LioIngressSink::accept_legacy_odometry(
            &valid_probe,
            source_frame(&odometry_binding, 1, &odometry_wire),
            &mut shutdown_rx,
        )
        .await
        .expect("exact raw source pair reaches READY");

        assert!(*valid_probe.source_pair_ready_rx.borrow());
        assert!(valid_probe.common.runtime.is_none());
        let pair_state = valid_probe.pair_state.lock();
        assert_eq!(pair_state.odometry.len(), 1);
        assert!(pair_state.clouds.is_empty());
        drop(pair_state);
        let state = valid_probe.common.state.lock();
        assert!(state.pending_registrations.is_empty());
        assert!(state.accepted_odometry_identities.is_empty());
        assert!(state.accepted_odometry_poses.is_empty());
        drop(state);
        let snapshot = valid_probe.common.snapshot();
        assert_eq!(snapshot.latest_odometry, None);
        assert_eq!(snapshot.latest_cloud, None);
        assert_eq!(snapshot.latest_adapter_timestamp_ns, None);
    }

    #[tokio::test]
    async fn native_probe_retains_validated_exact_and_fallback_pairs() {
        fn probe() -> NativeLioIngress {
            NativeLioIngress::without_runtime(
                DeviceLioIngressSpec::new_for_backend(
                    LIO_BACKEND_FASTLIO2,
                    7,
                    "lidar",
                    "base_link",
                    "odom",
                    "map",
                )
                .expect("native probe spec"),
                BodyFromSensorTransform::from_json(
                    r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
                )
                .expect("identity body-from-sensor transform"),
                Duration::from_nanos(10),
            )
            .expect("runtime-free native probe")
        }

        fn sample(identity: SourceIdentity, position: [f64; 3]) -> NativeOdometrySample {
            let mut odometry = valid_odometry();
            odometry.pose.position = position;
            odometry.header =
                source_header(identity, "odom").expect("source-authentic odometry header");
            NativeOdometrySample { identity, odometry }
        }

        let exact_probe = probe();
        let exact_identity = SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence: 11,
            source_timestamp_ns: 1_000,
        };
        assert!(exact_probe
            .insert_odometry(sample(exact_identity, [3.0, -2.0, 0.5]))
            .is_none());
        let exact_pair = exact_probe
            .insert_cloud(NativeCloudSample {
                identity: exact_identity,
                source_point_count: 2,
                sensor_points: vec![Point3::new(1.0, 2.0, 3.0), Point3::new(-1.0, 0.0, 0.5)],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .expect("exact timestamp pair");
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        exact_probe
            .process_pair(exact_pair, &mut shutdown_rx)
            .await
            .expect("seal exact retained pair");
        let exact = exact_probe
            .take_validated_startup_pair()
            .expect("take exact retained pair");
        validate_startup_pair(&exact).expect("exact retained proof");
        assert_eq!(exact.odometry_identity, exact_identity);
        assert_eq!(exact.cloud_identity, exact_identity);
        assert_eq!(
            exact.frozen_odom_from_body.translation.vector.as_slice(),
            &[3.0, -2.0, 0.5]
        );
        assert_eq!(
            exact.registered_odom_points.as_ref(),
            &[Point3::new(4.0, 0.0, 3.5), Point3::new(2.0, -2.0, 1.0)]
        );
        assert_eq!(exact.source_point_count, 2);
        assert_eq!(exact.emitted_point_count, 2);
        assert!(exact_probe.take_validated_startup_pair().is_err());

        let fallback_probe = probe();
        let fallback_odom_identity = SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence: 21,
            source_timestamp_ns: 1_995,
        };
        let fallback_cloud_identity = SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence: 22,
            source_timestamp_ns: 2_000,
        };
        assert!(fallback_probe
            .insert_odometry(sample(fallback_odom_identity, [0.25, 0.5, -0.25]))
            .is_none());
        assert!(fallback_probe
            .insert_cloud(NativeCloudSample {
                identity: fallback_cloud_identity,
                source_point_count: 1,
                sensor_points: vec![Point3::new(0.5, -0.5, 1.0)],
                registration_mode_token: RegistrationModeToken::default(),
            })
            .is_none());
        let watermark_identity = SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence: 23,
            source_timestamp_ns: 2_015,
        };
        let fallback_pair = fallback_probe
            .insert_odometry(sample(watermark_identity, [99.0, 99.0, 99.0]))
            .expect("watermark finalizes fallback pair");
        fallback_probe
            .process_pair(fallback_pair, &mut shutdown_rx)
            .await
            .expect("seal fallback retained pair");
        let fallback = fallback_probe
            .take_validated_startup_pair()
            .expect("take fallback retained pair");
        validate_startup_pair(&fallback).expect("fallback retained proof");
        assert_eq!(fallback.odometry_identity, fallback_odom_identity);
        assert_eq!(fallback.cloud_identity, fallback_cloud_identity);
        assert_eq!(
            fallback.registered_odom_points.as_ref(),
            &[Point3::new(0.75, 0.0, 0.75)]
        );
        assert_eq!(fallback.source_point_count, 1);
        assert_eq!(fallback.emitted_point_count, 1);
        let snapshot = fallback_probe.common.snapshot();
        assert_eq!(snapshot.latest_odometry, None);
        assert_eq!(snapshot.latest_cloud, None);
        assert_eq!(snapshot.latest_adapter_timestamp_ns, None);
    }

    #[tokio::test]
    async fn native_close_clears_pair_caches_and_ready_wait_is_bounded() {
        let make_ingress = || {
            let native_spec = DeviceLioIngressSpec::new_for_backend(
                LIO_BACKEND_FASTLIO2,
                7,
                "lidar",
                "base_link",
                "odom",
                "map",
            )
            .expect("native spec");
            let body_from_sensor = BodyFromSensorTransform::from_json(
                r#"{"translation_m":[0.0,0.0,0.0],"rotation_row_major":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]}"#,
            )
            .expect("identity body-from-sensor transform");
            let (source_pair_ready_tx, source_pair_ready_rx) = watch::channel(false);
            NativeLioIngress {
                common: CommonLioIngress::new(native_spec),
                body_from_sensor,
                pair_max_skew_ns: 10,
                pair_state: Mutex::new(NativePairState::default()),
                source_pair_ready_tx,
                source_pair_ready_rx,
            }
        };
        let ingress = Arc::new(make_ingress());
        let (_shutdown_tx, mut shutdown_rx) = watch::channel(false);
        let waiter_ingress = ingress.clone();
        let mut waiter = tokio::spawn(async move {
            LioIngressSink::wait_startup_ready(waiter_ingress.as_ref(), &mut shutdown_rx).await
        });
        assert!(
            tokio::time::timeout(Duration::from_millis(25), &mut waiter)
                .await
                .is_err(),
            "READY wait must remain pending before a valid source pair"
        );
        let pair_identity = SourceIdentity {
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence: 1,
            source_timestamp_ns: 1,
        };
        let mut pair_odometry = valid_odometry();
        pair_odometry.header =
            source_header(pair_identity, "odom").expect("source-authentic odometry header");
        let pair = NativeSourcePair {
            odometry: NativeOdometrySample {
                identity: pair_identity,
                odometry: pair_odometry,
            },
            cloud: NativeCloudSample {
                identity: pair_identity,
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            },
        };
        let (_pair_shutdown_tx, mut pair_shutdown_rx) = watch::channel(true);
        ingress
            .process_pair(pair, &mut pair_shutdown_rx)
            .await
            .expect("valid pair reaches READY before stopped public admission");
        tokio::time::timeout(Duration::from_secs(1), waiter)
            .await
            .expect("READY wait timeout")
            .expect("READY waiter join")
            .expect("first-pair READY");

        {
            let mut state = ingress.pair_state.lock();
            state.odometry.push_back(NativeOdometrySample {
                identity: SourceIdentity {
                    backend: LIO_BACKEND_FASTLIO2,
                    runtime_generation: 7,
                    source_sequence: 1,
                    source_timestamp_ns: 1,
                },
                odometry: valid_odometry(),
            });
            state.clouds.push_back(NativeCloudSample {
                identity: SourceIdentity {
                    backend: LIO_BACKEND_FASTLIO2,
                    runtime_generation: 7,
                    source_sequence: 1,
                    source_timestamp_ns: 100,
                },
                source_point_count: 1,
                sensor_points: vec![Point3::origin()],
                registration_mode_token: RegistrationModeToken::default(),
            });
        }
        LioIngressSink::close(ingress.as_ref());
        {
            let state = ingress.pair_state.lock();
            assert!(state.odometry.is_empty());
            assert!(state.clouds.is_empty());
        }
        assert!(ingress.common.snapshot().closed);

        let stopping_ingress = make_ingress();
        let (shutdown_tx, mut shutdown_rx) = watch::channel(false);
        shutdown_tx.send_replace(true);
        let error = tokio::time::timeout(
            Duration::from_secs(1),
            LioIngressSink::wait_startup_ready(&stopping_ingress, &mut shutdown_rx),
        )
        .await
        .expect("shutdown READY wait timeout")
        .expect_err("shutdown must terminate READY wait");
        assert!(error.contains("stopped before the first valid source pair"));
    }
}
