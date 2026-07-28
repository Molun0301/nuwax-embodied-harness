//! Person-follow control lifecycle around the 3D detection-navigation algorithm.

use std::time::Duration;

use nuwax_codec::lcm::schema;
use thiserror::Error;

use crate::{
    BBox, CameraIntrinsics, DetectionNavigation, DetectionNavigationConfig, ImageView,
    PointCloud2Like, TfLike, Twist, VisualServoingError,
};

const POINT_FIELD_FLOAT32: u8 = 7;
const POINT_FIELD_FLOAT64: u8 = 8;

/// Production person-follow lifecycle configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersonFollowConfig {
    /// Fixed control frequency. The original owner uses 20 Hz.
    pub control_frequency_hz: u32,
    /// Legacy compatibility setting; V2 LOST inputs are terminal immediately.
    pub max_lost_frames: u32,
    /// Maximum age of a tracked bbox before it counts as a lost frame.
    pub target_stale_after: Duration,
}

impl Default for PersonFollowConfig {
    fn default() -> Self {
        Self {
            control_frequency_hz: 20,
            max_lost_frames: 15,
            target_stale_after: Duration::from_secs(1),
        }
    }
}

/// Tracker result carried by one typed input update.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingState {
    /// The bbox belongs to a target visible in this image.
    Tracked,
    /// The tracker processed an image but found no target.
    Lost,
    /// End the current session and hold zero velocity.
    Stop,
}

/// One tracked-person observation and its camera model.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackedPerson {
    /// Stable follow session identifier.
    pub session_id: String,
    /// Human-readable target description.
    pub query: String,
    /// Source image timestamp in seconds.
    pub image_timestamp: f64,
    /// Optical camera frame.
    pub camera_frame: String,
    /// Source image width.
    pub image_width: u32,
    /// Source image height.
    pub image_height: u32,
    /// Source camera intrinsics.
    pub intrinsics: CameraIntrinsics,
    /// Tracked xyxy pixel bbox.
    pub bbox: BBox,
    /// Tracker lifecycle state.
    pub state: TrackingState,
}

/// Exact source timestamp used for ordering and command correlation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TargetStamp {
    /// Whole seconds.
    pub sec: i64,
    /// Nanoseconds within the second.
    pub nsec: u32,
}

/// Observable result of one control tick.
#[derive(Debug, Clone, PartialEq)]
pub enum PersonFollowState {
    /// A target is accepted, but its cloud or TF inputs are not ready yet.
    WaitingData,
    /// A valid 3D command was produced.
    Tracking,
    /// Target is absent or stale and the session has ended.
    Lost,
    /// Session was explicitly stopped.
    Stopped,
    /// A typed input, pointcloud, TF, or projection error ended the session.
    Failed(String),
}

/// Command and lifecycle state emitted by one active control tick.
#[derive(Debug, Clone, PartialEq)]
pub struct PersonFollowOutput {
    /// Stable follow session identifier.
    pub session_id: String,
    /// Exact target sequence consumed by this command.
    pub target_seq: i64,
    /// Exact target image timestamp.
    pub target_stamp: TargetStamp,
    /// Strictly increasing owner-lifetime command sequence.
    pub command_seq: i64,
    /// Actual latest point-cloud revision observed by the controller.
    pub cloud_revision: i64,
    /// Planar command routed to the Gateway ingress.
    pub command: Twist,
    /// State associated with the command.
    pub state: PersonFollowState,
    /// Bounded human-readable state detail.
    pub reason: String,
}

/// Errors rejected before a controller can start.
#[derive(Debug, Error)]
pub enum PersonFollowError {
    /// Static lifecycle configuration is invalid.
    #[error("invalid person-follow config: {0}")]
    InvalidConfig(String),
    /// A PointCloud2 layout cannot be decoded as XYZ.
    #[error("invalid PointCloud2: {0}")]
    InvalidPointCloud(String),
}

/// Packed XYZ cloud used by [`DetectionNavigation`] at the runtime boundary.
#[derive(Debug, Clone)]
pub struct PackedPointCloud {
    frame_id: String,
    revision: i64,
    xyz: Vec<[f64; 3]>,
    packed_xyz: Vec<u8>,
    is_dense: bool,
}

impl PackedPointCloud {
    /// Construct a packed cloud from decoded XYZ points.
    #[must_use]
    pub fn from_xyz(frame_id: impl Into<String>, xyz: Vec<[f64; 3]>) -> Self {
        Self::from_xyz_with_revision(frame_id, 1, xyz)
    }

    /// Construct a packed cloud with an explicit positive map revision.
    #[must_use]
    pub fn from_xyz_with_revision(
        frame_id: impl Into<String>,
        revision: i64,
        xyz: Vec<[f64; 3]>,
    ) -> Self {
        let frame_id = frame_id.into();
        let packed_xyz = pack_xyz(&xyz);
        let is_dense = xyz
            .iter()
            .flatten()
            .all(|coordinate| coordinate.is_finite());
        Self {
            frame_id,
            revision,
            xyz,
            packed_xyz,
            is_dense,
        }
    }

    /// Decode ROS PointCloud2 rows, field offsets, endian, and float32/float64 XYZ.
    pub fn from_lcm(message: &schema::PointCloud2) -> Result<Self, PersonFollowError> {
        if message.header.frame_id.is_empty() {
            return Err(PersonFollowError::InvalidPointCloud(
                "header.frame_id is empty".into(),
            ));
        }
        let revision = i64::from(message.header.seq);
        if revision <= 0 {
            return Err(PersonFollowError::InvalidPointCloud(
                "header.seq cloud revision must be positive".into(),
            ));
        }
        let width = positive_dimension(message.width, "width")?;
        let height = positive_dimension(message.height, "height")?;
        let point_step = positive_dimension(message.point_step, "point_step")?;
        let row_step = positive_dimension(message.row_step, "row_step")?;
        let row_payload = point_step.checked_mul(width).ok_or_else(|| {
            PersonFollowError::InvalidPointCloud("point_step * width overflows".into())
        })?;
        if row_step < row_payload {
            return Err(PersonFollowError::InvalidPointCloud(format!(
                "row_step {row_step} is smaller than point payload {row_payload}"
            )));
        }
        let required = row_step.checked_mul(height).ok_or_else(|| {
            PersonFollowError::InvalidPointCloud("row_step * height overflows".into())
        })?;
        if message.data.len() < required {
            return Err(PersonFollowError::InvalidPointCloud(format!(
                "data has {} bytes, requires {required}",
                message.data.len()
            )));
        }

        let x = field_layout(message, "x", point_step)?;
        let y = field_layout(message, "y", point_step)?;
        let z = field_layout(message, "z", point_step)?;
        let mut xyz = Vec::with_capacity(width * height);
        for row in 0..height {
            for column in 0..width {
                let base = row * row_step + column * point_step;
                xyz.push([
                    read_coordinate(&message.data, base, x, message.is_bigendian)?,
                    read_coordinate(&message.data, base, y, message.is_bigendian)?,
                    read_coordinate(&message.data, base, z, message.is_bigendian)?,
                ]);
            }
        }
        let packed_xyz = pack_xyz(&xyz);
        Ok(Self {
            frame_id: message.header.frame_id.clone(),
            revision,
            xyz,
            packed_xyz,
            is_dense: message.is_dense,
        })
    }

    /// Pointcloud header frame.
    #[must_use]
    pub fn frame_id(&self) -> &str {
        &self.frame_id
    }

    /// Decoded XYZ points, primarily for boundary verification.
    #[must_use]
    pub fn xyz(&self) -> &[[f64; 3]] {
        &self.xyz
    }

    /// Monotonic map revision from `PointCloud2.header.seq`.
    #[must_use]
    pub fn revision(&self) -> i64 {
        self.revision
    }
}

impl PointCloud2Like for PackedPointCloud {
    fn xyz_bytes(&self) -> &[u8] {
        &self.packed_xyz
    }

    fn is_dense(&self) -> bool {
        self.is_dense
    }

    fn frame_id(&self) -> Option<&str> {
        Some(&self.frame_id)
    }
}

#[derive(Debug, Clone, Copy)]
struct FieldLayout {
    offset: usize,
    datatype: u8,
}

fn positive_dimension(value: i32, field: &str) -> Result<usize, PersonFollowError> {
    usize::try_from(value)
        .ok()
        .filter(|value| *value > 0)
        .ok_or_else(|| PersonFollowError::InvalidPointCloud(format!("{field} must be positive")))
}

fn field_layout(
    message: &schema::PointCloud2,
    name: &str,
    point_step: usize,
) -> Result<FieldLayout, PersonFollowError> {
    let field = message
        .fields
        .iter()
        .find(|field| field.name == name)
        .ok_or_else(|| PersonFollowError::InvalidPointCloud(format!("missing {name} field")))?;
    if field.count != 1 || !matches!(field.datatype, POINT_FIELD_FLOAT32 | POINT_FIELD_FLOAT64) {
        return Err(PersonFollowError::InvalidPointCloud(format!(
            "{name} must be one float32 or float64"
        )));
    }
    let offset = usize::try_from(field.offset)
        .map_err(|_| PersonFollowError::InvalidPointCloud(format!("{name} offset is negative")))?;
    let width = if field.datatype == POINT_FIELD_FLOAT32 {
        4
    } else {
        8
    };
    if offset.checked_add(width).is_none_or(|end| end > point_step) {
        return Err(PersonFollowError::InvalidPointCloud(format!(
            "{name} field exceeds point_step"
        )));
    }
    Ok(FieldLayout {
        offset,
        datatype: field.datatype,
    })
}

fn read_coordinate(
    data: &[u8],
    base: usize,
    field: FieldLayout,
    big_endian: bool,
) -> Result<f64, PersonFollowError> {
    let start = base + field.offset;
    match field.datatype {
        POINT_FIELD_FLOAT32 => {
            let bytes: [u8; 4] = data[start..start + 4].try_into().expect("validated field");
            let value = if big_endian {
                f32::from_be_bytes(bytes)
            } else {
                f32::from_le_bytes(bytes)
            };
            Ok(f64::from(value))
        }
        POINT_FIELD_FLOAT64 => {
            let bytes: [u8; 8] = data[start..start + 8].try_into().expect("validated field");
            Ok(if big_endian {
                f64::from_be_bytes(bytes)
            } else {
                f64::from_le_bytes(bytes)
            })
        }
        _ => Err(PersonFollowError::InvalidPointCloud(
            "unsupported coordinate datatype".into(),
        )),
    }
}

fn pack_xyz(xyz: &[[f64; 3]]) -> Vec<u8> {
    let mut packed = Vec::with_capacity(xyz.len() * 12);
    for point in xyz {
        for coordinate in point {
            packed.extend_from_slice(&(*coordinate as f32).to_le_bytes());
        }
    }
    packed
}

#[derive(Debug, Clone)]
struct TimedTarget {
    target: TrackedPerson,
    target_seq: i64,
    required_cloud_revision: i64,
    stamp: TargetStamp,
    received_at: Duration,
}

/// Stateful 20 Hz owner around the original DetectionNavigation algorithm.
pub struct PersonFollowController {
    navigation: DetectionNavigation,
    config: PersonFollowConfig,
    target: Option<TimedTarget>,
    pointcloud: Option<PackedPointCloud>,
    active: bool,
    accepted_any: bool,
    last_session_id: Option<String>,
    last_target_seq: i64,
    last_stamp: Option<TargetStamp>,
    last_tracking_state: Option<TrackingState>,
    command_seq: i64,
}

impl PersonFollowController {
    /// Construct a controller with one shared timestamp-aware TF provider.
    pub fn new(
        tf: Box<dyn TfLike>,
        navigation_config: DetectionNavigationConfig,
        config: PersonFollowConfig,
    ) -> Result<Self, PersonFollowError> {
        if config.control_frequency_hz == 0 {
            return Err(PersonFollowError::InvalidConfig(
                "control_frequency_hz must be positive".into(),
            ));
        }
        if config.target_stale_after.is_zero() {
            return Err(PersonFollowError::InvalidConfig(
                "target_stale_after must be positive".into(),
            ));
        }
        Ok(Self {
            navigation: DetectionNavigation::new(tf, navigation_config),
            config,
            target: None,
            pointcloud: None,
            active: false,
            accepted_any: false,
            last_session_id: None,
            last_target_seq: 0,
            last_stamp: None,
            last_tracking_state: None,
            command_seq: 0,
        })
    }

    /// Fixed control-loop period.
    #[must_use]
    pub fn control_period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / f64::from(self.config.control_frequency_hz))
    }

    /// Replace the latest global map cloud.
    pub fn update_pointcloud(&mut self, pointcloud: PackedPointCloud) {
        self.pointcloud = Some(pointcloud);
    }

    /// Compatibility adapter for in-process callers without V2 correlation metadata.
    pub fn update_target(&mut self, target: TrackedPerson, received_at: Duration) {
        let target_seq = self.last_target_seq.saturating_add(1).max(1);
        let sec = target.image_timestamp.floor() as i64;
        let nsec = ((target.image_timestamp - sec as f64) * 1e9).round() as u32;
        let _ = self.accept_target(
            target,
            target_seq,
            1,
            TargetStamp { sec, nsec },
            received_at,
        );
    }

    /// Validate strict V2 ordering and arm an accepted target.
    pub fn accept_target(
        &mut self,
        target: TrackedPerson,
        target_seq: i64,
        required_cloud_revision: i64,
        stamp: TargetStamp,
        received_at: Duration,
    ) -> Result<bool, PersonFollowError> {
        if target_seq <= 0 || required_cloud_revision <= 0 {
            return Err(PersonFollowError::InvalidConfig(
                "target_seq and required_cloud_revision must be positive".into(),
            ));
        }
        validate_target(&target).map_err(PersonFollowError::InvalidConfig)?;

        if let Some(last_session_id) = self.last_session_id.as_deref() {
            if target.session_id != last_session_id {
                if self.active {
                    return Ok(false);
                }
            } else {
                if self.active && self.last_tracking_state == Some(TrackingState::Stop) {
                    return Ok(false);
                }
                if target_seq <= self.last_target_seq {
                    return Ok(false);
                }
                let last_stamp = self.last_stamp.expect("session has an exact stamp");
                let same_stamp_transition = stamp == last_stamp
                    && matches!(
                        (self.last_tracking_state, target.state),
                        (Some(TrackingState::Tracked), TrackingState::Stop)
                            | (Some(TrackingState::Stop), TrackingState::Tracked)
                    );
                if stamp < last_stamp || (stamp == last_stamp && !same_stamp_transition) {
                    return Ok(false);
                }
            }
        }

        self.active = true;
        self.accepted_any = true;
        self.last_session_id = Some(target.session_id.clone());
        self.last_target_seq = target_seq;
        self.last_stamp = Some(stamp);
        self.last_tracking_state = Some(target.state);
        self.target = Some(TimedTarget {
            target,
            target_seq,
            required_cloud_revision,
            stamp,
            received_at,
        });
        Ok(true)
    }

    /// Run one active control tick. Idle owners emit nothing.
    pub fn tick(&mut self, now: Duration) -> Option<PersonFollowOutput> {
        if !self.active {
            return None;
        }
        let timed = self.target.clone()?;
        match timed.target.state {
            TrackingState::Stop => {
                return Some(self.emit(
                    &timed,
                    PersonFollowState::Stopped,
                    "target stopped",
                    None,
                    true,
                ));
            }
            TrackingState::Lost => {
                return Some(self.emit(&timed, PersonFollowState::Lost, "target lost", None, true));
            }
            TrackingState::Tracked => {}
        }
        if now.saturating_sub(timed.received_at) >= self.config.target_stale_after {
            return Some(self.emit(&timed, PersonFollowState::Lost, "target stale", None, true));
        }
        let Some(pointcloud) = self.pointcloud.as_ref() else {
            return Some(self.emit(
                &timed,
                PersonFollowState::WaitingData,
                "waiting for pointcloud",
                None,
                false,
            ));
        };
        if pointcloud.revision() < timed.required_cloud_revision {
            return Some(self.emit(
                &timed,
                PersonFollowState::WaitingData,
                "waiting for required pointcloud revision",
                None,
                false,
            ));
        }
        let result = self.compute_command(&timed.target);
        Some(match result {
            Ok(command) => self.emit(
                &timed,
                PersonFollowState::Tracking,
                "tracking",
                Some(command),
                false,
            ),
            Err(VisualServoingError::MissingTf(_)) => self.emit(
                &timed,
                PersonFollowState::WaitingData,
                "waiting for transform",
                None,
                false,
            ),
            Err(error) => self.emit(
                &timed,
                PersonFollowState::Failed(error.to_string()),
                &error.to_string(),
                None,
                true,
            ),
        })
    }

    fn compute_command(&self, target: &TrackedPerson) -> Result<Twist, VisualServoingError> {
        let pointcloud = self
            .pointcloud
            .as_ref()
            .expect("tick checks pointcloud availability");
        let image = ImageView {
            width: target.image_width,
            height: target.image_height,
            encoding: nuwax_nav_types_messages::ImageEncoding::Rgb8,
            data: &[],
        };
        self.navigation
            .compute_twist_for_detection_3d_at(
                target.bbox,
                &image,
                pointcloud,
                &target.intrinsics,
                &target.camera_frame,
                pointcloud.frame_id(),
                Some(target.image_timestamp),
            )
            .map(|command| Twist {
                linear: [command.linear[0], command.linear[1], 0.0],
                angular: [0.0, 0.0, command.angular[2]],
            })
            .and_then(|command| {
                command
                    .linear
                    .into_iter()
                    .chain(command.angular)
                    .all(f64::is_finite)
                    .then_some(command)
                    .ok_or_else(|| {
                        VisualServoingError::ProjectionFailed(
                            "computed command contains non-finite values".into(),
                        )
                    })
            })
    }

    fn emit(
        &mut self,
        target: &TimedTarget,
        state: PersonFollowState,
        reason: &str,
        command: Option<Twist>,
        terminal: bool,
    ) -> PersonFollowOutput {
        self.command_seq = self
            .command_seq
            .checked_add(1)
            .expect("command sequence exhausted");
        if terminal {
            self.active = false;
        }
        PersonFollowOutput {
            session_id: target.target.session_id.clone(),
            target_seq: target.target_seq,
            target_stamp: target.stamp,
            command_seq: self.command_seq,
            cloud_revision: self
                .pointcloud
                .as_ref()
                .map_or(0, PackedPointCloud::revision),
            command: command.unwrap_or_else(Twist::zero),
            state,
            reason: bounded_reason(reason),
        }
    }

    /// Emit exactly one correlated terminal zero during owner teardown.
    pub fn teardown(&mut self) -> Option<PersonFollowOutput> {
        if !self.accepted_any {
            return None;
        }
        self.accepted_any = false;
        let target = self.target.clone()?;
        Some(self.emit(
            &target,
            PersonFollowState::Stopped,
            "owner teardown",
            None,
            true,
        ))
    }

    /// End an accepted active session after malformed runtime input or computation failure.
    pub fn fail_active(&mut self, reason: impl Into<String>) -> Option<PersonFollowOutput> {
        if !self.active {
            return None;
        }
        let target = self.target.clone()?;
        let reason = reason.into();
        Some(self.emit(
            &target,
            PersonFollowState::Failed(reason.clone()),
            &reason,
            None,
            true,
        ))
    }
}

fn bounded_reason(reason: &str) -> String {
    const MAX_REASON_BYTES: usize = 255;
    let mut end = reason.len().min(MAX_REASON_BYTES);
    while !reason.is_char_boundary(end) {
        end -= 1;
    }
    reason[..end].to_owned()
}

fn validate_target(target: &TrackedPerson) -> Result<(), String> {
    if target.session_id.is_empty() {
        return Err("session_id is empty".into());
    }
    if target.camera_frame.is_empty() {
        return Err("camera_frame is empty".into());
    }
    if !target.image_timestamp.is_finite() || target.image_timestamp < 0.0 {
        return Err("image timestamp is invalid".into());
    }
    if target.state == TrackingState::Tracked {
        if target.image_width != target.intrinsics.width
            || target.image_height != target.intrinsics.height
        {
            return Err("image dimensions do not match camera intrinsics".into());
        }
        if !target.intrinsics.is_valid() {
            return Err("camera intrinsics are invalid".into());
        }
        if !target.bbox.is_valid() {
            return Err("bbox is invalid".into());
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PoseStamped;

    struct ReadyTf;

    impl TfLike for ReadyTf {
        fn lookup(&self, target_frame: &str, source_frame: &str) -> Option<PoseStamped> {
            match (target_frame, source_frame) {
                ("camera", "map") => Some(PoseStamped::identity("camera", 0.0, 0.0, 0.0)),
                ("map", "base_link") => Some(PoseStamped::identity("map", 0.0, 0.0, 0.0)),
                _ => None,
            }
        }
    }

    struct MissingTf;

    impl TfLike for MissingTf {
        fn lookup(&self, _target_frame: &str, _source_frame: &str) -> Option<PoseStamped> {
            None
        }
    }

    fn controller(tf: Box<dyn TfLike>) -> PersonFollowController {
        PersonFollowController::new(
            tf,
            DetectionNavigationConfig::default(),
            PersonFollowConfig::default(),
        )
        .expect("controller")
    }

    fn target(session_id: &str, state: TrackingState) -> TrackedPerson {
        TrackedPerson {
            session_id: session_id.into(),
            query: "person".into(),
            image_timestamp: 10.000_000_001,
            camera_frame: "camera".into(),
            image_width: 640,
            image_height: 480,
            intrinsics: CameraIntrinsics::new(100.0, 100.0, 320.0, 240.0, 640, 480),
            bbox: BBox::new(400.0, 220.0, 440.0, 260.0),
            state,
        }
    }

    fn stamp(nsec: u32) -> TargetStamp {
        TargetStamp { sec: 10, nsec }
    }

    fn cloud(revision: i64) -> PackedPointCloud {
        PackedPointCloud::from_xyz_with_revision("map", revision, vec![[3.0, 0.0, 3.0]; 12])
    }

    #[test]
    fn defaults_lock_twenty_hz_and_one_second_staleness() {
        let config = PersonFollowConfig::default();
        assert_eq!(config.control_frequency_hz, 20);
        assert_eq!(config.target_stale_after, Duration::from_secs(1));
    }

    #[test]
    fn missing_and_old_clouds_wait_without_ending_session() {
        let mut controller = controller(Box::new(ReadyTf));
        assert!(controller
            .accept_target(
                target("s", TrackingState::Tracked),
                1,
                2,
                stamp(1),
                Duration::ZERO
            )
            .expect("target accepted"));

        let missing = controller.tick(Duration::ZERO).expect("waiting command");
        assert_eq!(missing.state, PersonFollowState::WaitingData);
        assert_eq!(missing.cloud_revision, 0);
        assert_eq!(missing.command, Twist::zero());

        controller.update_pointcloud(cloud(1));
        let old = controller
            .tick(Duration::from_millis(50))
            .expect("waiting command");
        assert_eq!(old.state, PersonFollowState::WaitingData);
        assert_eq!(old.cloud_revision, 1);

        controller.update_pointcloud(cloud(2));
        let tracking = controller
            .tick(Duration::from_millis(100))
            .expect("tracking command");
        assert_eq!(tracking.state, PersonFollowState::Tracking);
        assert_eq!(tracking.cloud_revision, 2);
        assert_eq!(tracking.command_seq, 3);
        assert_eq!(tracking.command.linear[2], 0.0);
        assert_eq!(tracking.command.angular[0..2], [0.0, 0.0]);
    }

    #[test]
    fn missing_tf_waits_and_preserves_exact_correlation() {
        let mut controller = controller(Box::new(MissingTf));
        controller.update_pointcloud(cloud(7));
        controller
            .accept_target(
                target("session", TrackingState::Tracked),
                9,
                7,
                stamp(3),
                Duration::ZERO,
            )
            .expect("target accepted");

        let output = controller
            .tick(Duration::from_millis(50))
            .expect("waiting command");
        assert_eq!(output.state, PersonFollowState::WaitingData);
        assert_eq!(output.session_id, "session");
        assert_eq!(output.target_seq, 9);
        assert_eq!(output.target_stamp, stamp(3));
        assert_eq!(output.command_seq, 1);
        assert_eq!(output.command, Twist::zero());
    }

    #[test]
    fn duplicate_target_does_not_refresh_one_second_watchdog() {
        let mut controller = controller(Box::new(MissingTf));
        controller
            .accept_target(
                target("s", TrackingState::Tracked),
                1,
                1,
                stamp(1),
                Duration::ZERO,
            )
            .expect("target accepted");
        assert!(!controller
            .accept_target(
                target("s", TrackingState::Tracked),
                1,
                1,
                stamp(1),
                Duration::from_millis(900),
            )
            .expect("duplicate rejected"));

        let output = controller
            .tick(Duration::from_secs(1))
            .expect("stale command");
        assert_eq!(output.state, PersonFollowState::Lost);
        assert_eq!(output.command, Twist::zero());
        assert!(controller.tick(Duration::from_millis(1050)).is_none());
    }

    #[test]
    fn ordering_and_completed_session_switches_are_strict() {
        let mut controller = controller(Box::new(MissingTf));
        controller
            .accept_target(
                target("a", TrackingState::Tracked),
                1,
                1,
                stamp(1),
                Duration::ZERO,
            )
            .expect("initial target");
        assert!(!controller
            .accept_target(
                target("b", TrackingState::Tracked),
                1,
                1,
                stamp(2),
                Duration::ZERO
            )
            .expect("active session switch rejected"));
        assert!(!controller
            .accept_target(
                target("a", TrackingState::Tracked),
                2,
                1,
                stamp(1),
                Duration::ZERO
            )
            .expect("equal stamp rejected"));

        assert!(controller
            .accept_target(
                target("a", TrackingState::Stop),
                2,
                1,
                stamp(1),
                Duration::ZERO
            )
            .expect("same-stamp stop accepted"));
        assert!(!controller
            .accept_target(
                target("a", TrackingState::Tracked),
                3,
                1,
                stamp(1),
                Duration::ZERO
            )
            .expect("stop must be observed"));
        assert_eq!(
            controller.tick(Duration::ZERO).expect("stop command").state,
            PersonFollowState::Stopped
        );
        assert!(controller
            .accept_target(
                target("a", TrackingState::Tracked),
                3,
                1,
                stamp(1),
                Duration::ZERO
            )
            .expect("same-stamp restart accepted"));
        assert!(!controller
            .accept_target(
                target("a", TrackingState::Tracked),
                4,
                1,
                stamp(0),
                Duration::ZERO
            )
            .expect("older exact stamp rejected"));
    }

    #[test]
    fn teardown_is_correlated_once_and_command_sequence_continues() {
        let mut idle = controller(Box::new(MissingTf));
        assert!(idle.teardown().is_none());

        let mut controller = controller(Box::new(MissingTf));
        controller
            .accept_target(
                target("s", TrackingState::Tracked),
                4,
                1,
                stamp(8),
                Duration::ZERO,
            )
            .expect("target accepted");
        let waiting = controller.tick(Duration::ZERO).expect("waiting command");
        let terminal = controller.teardown().expect("terminal command");
        assert_eq!(terminal.state, PersonFollowState::Stopped);
        assert_eq!(terminal.session_id, "s");
        assert_eq!(terminal.target_seq, 4);
        assert_eq!(terminal.target_stamp, stamp(8));
        assert_eq!(terminal.command_seq, waiting.command_seq + 1);
        assert_eq!(terminal.command, Twist::zero());
        assert!(controller.teardown().is_none());
    }

    #[test]
    fn malformed_runtime_input_fails_only_an_accepted_active_session() {
        let mut controller = controller(Box::new(MissingTf));
        assert!(controller.fail_active("bad cloud").is_none());
        controller
            .accept_target(
                target("session", TrackingState::Tracked),
                11,
                3,
                stamp(9),
                Duration::ZERO,
            )
            .expect("target accepted");

        let failed = controller.fail_active("bad cloud").expect("failed command");
        assert_eq!(failed.state, PersonFollowState::Failed("bad cloud".into()));
        assert_eq!(failed.session_id, "session");
        assert_eq!(failed.target_seq, 11);
        assert_eq!(failed.command, Twist::zero());
        assert!(controller.tick(Duration::ZERO).is_none());
    }
}
