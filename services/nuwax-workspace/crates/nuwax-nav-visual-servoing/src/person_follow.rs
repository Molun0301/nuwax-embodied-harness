//! Person-follow control lifecycle around the 3D detection-navigation algorithm.

use std::time::Duration;

use nuwax_codec::lcm::schema;
use thiserror::Error;

use crate::{
    BBox, CameraIntrinsics, DetectionNavigation, DetectionNavigationConfig, ImageView,
    PointCloud2Like, TfLike, Twist,
};

const POINT_FIELD_FLOAT32: u8 = 7;
const POINT_FIELD_FLOAT64: u8 = 8;

/// Production person-follow lifecycle configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersonFollowConfig {
    /// Fixed control frequency. The original owner uses 20 Hz.
    pub control_frequency_hz: u32,
    /// Consecutive lost frames tolerated before ending a session.
    pub max_lost_frames: u32,
    /// Maximum age of a tracked bbox before it counts as a lost frame.
    pub target_stale_after: Duration,
}

impl Default for PersonFollowConfig {
    fn default() -> Self {
        Self {
            control_frequency_hz: 20,
            max_lost_frames: 15,
            target_stale_after: Duration::from_millis(100),
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

/// Observable result of one control tick.
#[derive(Debug, Clone, PartialEq)]
pub enum PersonFollowState {
    /// A valid 3D command was produced.
    Tracking,
    /// Target is absent or stale; zero is emitted immediately.
    Lost {
        /// Number of consecutive lost control ticks.
        frames: u32,
        /// True after the original `lost_count > 15` boundary is crossed.
        terminal: bool,
    },
    /// Session was explicitly stopped.
    Stopped,
    /// A typed input, pointcloud, TF, or projection error ended the session.
    Failed(String),
}

/// Command and lifecycle state emitted by one active control tick.
#[derive(Debug, Clone, PartialEq)]
pub struct PersonFollowOutput {
    /// Command routed to `nav_cmd_vel` by the production owner.
    pub command: Twist,
    /// State associated with the command.
    pub state: PersonFollowState,
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
    xyz: Vec<[f64; 3]>,
    packed_xyz: Vec<u8>,
    is_dense: bool,
}

impl PackedPointCloud {
    /// Construct a packed cloud from decoded XYZ points.
    #[must_use]
    pub fn from_xyz(frame_id: impl Into<String>, xyz: Vec<[f64; 3]>) -> Self {
        let frame_id = frame_id.into();
        let packed_xyz = pack_xyz(&xyz);
        let is_dense = xyz
            .iter()
            .flatten()
            .all(|coordinate| coordinate.is_finite());
        Self {
            frame_id,
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
    received_at: Duration,
}

/// Stateful 20 Hz owner around the original DetectionNavigation algorithm.
pub struct PersonFollowController {
    navigation: DetectionNavigation,
    config: PersonFollowConfig,
    target: Option<TimedTarget>,
    pointcloud: Option<PackedPointCloud>,
    active: bool,
    lost_frames: u32,
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
            lost_frames: 0,
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

    /// Replace the latest tracker update and arm its session.
    pub fn update_target(&mut self, target: TrackedPerson, received_at: Duration) {
        if target.state == TrackingState::Tracked {
            self.lost_frames = 0;
        }
        self.active = true;
        self.target = Some(TimedTarget {
            target,
            received_at,
        });
    }

    /// Run one active control tick. Idle owners emit nothing.
    pub fn tick(&mut self, now: Duration) -> Option<PersonFollowOutput> {
        if !self.active {
            return None;
        }
        let timed = self.target.clone()?;
        match timed.target.state {
            TrackingState::Stop => return Some(self.terminal(PersonFollowState::Stopped)),
            TrackingState::Lost => return Some(self.lost()),
            TrackingState::Tracked => {}
        }
        if now.saturating_sub(timed.received_at) > self.config.target_stale_after {
            return Some(self.lost());
        }
        let result = self.compute_command(&timed.target);
        Some(match result {
            Ok(command) => PersonFollowOutput {
                command,
                state: PersonFollowState::Tracking,
            },
            Err(reason) => self.terminal(PersonFollowState::Failed(reason)),
        })
    }

    fn compute_command(&self, target: &TrackedPerson) -> Result<Twist, String> {
        validate_target(target)?;
        let pointcloud = self
            .pointcloud
            .as_ref()
            .ok_or_else(|| "no pointcloud available for 3D navigation".to_string())?;
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
                linear: command.linear,
                angular: command.angular,
            })
            .map_err(|error| error.to_string())
    }

    fn lost(&mut self) -> PersonFollowOutput {
        self.lost_frames = self.lost_frames.saturating_add(1);
        let terminal = self.lost_frames > self.config.max_lost_frames;
        if terminal {
            self.active = false;
        }
        PersonFollowOutput {
            command: Twist::zero(),
            state: PersonFollowState::Lost {
                frames: self.lost_frames,
                terminal,
            },
        }
    }

    fn terminal(&mut self, state: PersonFollowState) -> PersonFollowOutput {
        self.active = false;
        self.lost_frames = 0;
        PersonFollowOutput {
            command: Twist::zero(),
            state,
        }
    }
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
    Ok(())
}
