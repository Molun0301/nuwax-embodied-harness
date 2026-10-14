//! Complete DimOS-compatible domain messages and typed-LCM conversions.
//!
//! The types in this module preserve message semantics such as covariance,
//! child frames and raw point layout. Algorithm-facing types such as
//! [`crate::Path2D`] deliberately remain smaller and are converted explicitly.

use nuwax_protocol::lcm::schema;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    ClockKind, NavFrameId, OccupancyCell, OccupancyGrid2D, Odometry, Path2D, Point2, Point3, Pose3,
    PoseStamped, Quaternion, Timestamp,
};

/// A failed semantic conversion between a domain message and its wire schema.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum DimosMessageError {
    /// A value cannot be represented by the target schema.
    #[error("field `{field}` is outside the target numeric range")]
    NumericRange {
        /// Field that failed conversion.
        field: &'static str,
    },
    /// A dynamically-sized field does not match its metadata.
    #[error("field `{field}` has length {actual}, expected {expected}")]
    LengthMismatch {
        /// Field that failed validation.
        field: &'static str,
        /// Expected length.
        expected: usize,
        /// Actual length.
        actual: usize,
    },
    /// The wire schema cannot preserve the source clock domain.
    #[error("wire header cannot preserve the {clock:?} clock domain")]
    UnsupportedClock {
        /// Unsupported clock domain.
        clock: ClockKind,
    },
    /// A point-cloud field layout is unsupported by the structured adapter.
    #[error("unsupported PointCloud2 layout: {0}")]
    UnsupportedPointLayout(&'static str),
}

/// Domain header corresponding to `std_msgs/Header`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MessageHeader {
    /// Producer sequence number.
    pub seq: i32,
    /// Message timestamp.
    pub stamp: Timestamp,
    /// Coordinate frame.
    pub frame_id: NavFrameId,
}

/// Rigid transform corresponding to `geometry_msgs/Transform`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform3 {
    /// Translation in metres.
    pub translation: Point3,
    /// Rotation quaternion.
    pub rotation: Quaternion,
}

/// Stamped parent-to-child transform.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TransformStampedMessage {
    /// Parent-frame header.
    pub header: MessageHeader,
    /// Child frame.
    pub child_frame_id: NavFrameId,
    /// Parent-to-child transform.
    pub transform: Transform3,
}

/// Ordered transform batch corresponding to `tf2_msgs/TFMessage`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct TfMessage {
    /// Transforms in publication order.
    pub transforms: Vec<TransformStampedMessage>,
}

/// Complete map metadata corresponding to `nav_msgs/MapMetaData`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MapMetadata {
    /// Time at which the map was loaded.
    pub map_load_time: Timestamp,
    /// Cell resolution in metres.
    pub resolution: f32,
    /// Width in cells.
    pub width: u32,
    /// Height in cells.
    pub height: u32,
    /// Pose of cell `(0, 0)`.
    pub origin: Pose3,
}

/// Complete `nav_msgs/OccupancyGrid` domain message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OccupancyGridMessage {
    /// Publication header.
    pub header: MessageHeader,
    /// Map metadata.
    pub info: MapMetadata,
    /// ROS occupancy values: `-1` unknown and `0..=100` known occupancy.
    pub data: Vec<i8>,
}

/// Complete `nav_msgs/Path` domain message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct PathMessage {
    /// Path-level header.
    pub header: MessageHeader,
    /// Ordered stamped poses.
    pub poses: Vec<PoseStamped>,
}

/// Row-major 6x6 covariance matrix.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Covariance6(pub [[f64; 6]; 6]);

impl Default for Covariance6 {
    fn default() -> Self {
        Self([[0.0; 6]; 6])
    }
}

/// Complete `nav_msgs/Odometry` domain message.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OdometryMessage {
    /// Parent-frame header.
    pub header: MessageHeader,
    /// Moving child frame.
    pub child_frame_id: NavFrameId,
    /// Child pose in the parent frame.
    pub pose: Pose3,
    /// Pose covariance.
    pub pose_covariance: Covariance6,
    /// Linear velocity.
    pub linear: Point3,
    /// Angular velocity.
    pub angular: Point3,
    /// Twist covariance.
    pub twist_covariance: Covariance6,
}

/// Field descriptor corresponding to `sensor_msgs/PointField`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PointField {
    /// Field name.
    pub name: String,
    /// Byte offset within one point.
    pub offset: u32,
    /// ROS PointField datatype identifier.
    pub datatype: u8,
    /// Element count.
    pub count: u32,
}

/// Complete raw `sensor_msgs/PointCloud2` domain message.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PointCloud2Message {
    /// Publication header.
    pub header: MessageHeader,
    /// Number of rows.
    pub height: u32,
    /// Number of points per row.
    pub width: u32,
    /// Point field descriptors.
    pub fields: Vec<PointField>,
    /// Whether scalars use big-endian representation.
    pub is_bigendian: bool,
    /// Bytes per point.
    pub point_step: u32,
    /// Bytes per row.
    pub row_step: u32,
    /// Raw point bytes.
    pub data: Vec<u8>,
    /// Whether all points are finite.
    pub is_dense: bool,
}

fn split_timestamp(stamp: Timestamp) -> Result<(i32, i32), DimosMessageError> {
    if stamp.clock != ClockKind::Unix {
        return Err(DimosMessageError::UnsupportedClock { clock: stamp.clock });
    }
    let sec = stamp.nanos.div_euclid(1_000_000_000);
    let nsec = stamp.nanos.rem_euclid(1_000_000_000);
    Ok((
        i32::try_from(sec).map_err(|_| DimosMessageError::NumericRange { field: "stamp_sec" })?,
        i32::try_from(nsec).map_err(|_| DimosMessageError::NumericRange {
            field: "stamp_nsec",
        })?,
    ))
}

fn join_timestamp(sec: i32, nsec: i32) -> Result<Timestamp, DimosMessageError> {
    if !(0..1_000_000_000).contains(&nsec) {
        return Err(DimosMessageError::NumericRange {
            field: "stamp_nsec",
        });
    }
    Ok(Timestamp::from_unix(sec.into(), nsec as u32))
}

fn pose_to_wire(pose: Pose3) -> schema::Pose {
    schema::Pose {
        position: [pose.position.x, pose.position.y, pose.position.z],
        orientation_xyzw: [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
    }
}

fn pose_from_wire(pose: schema::Pose) -> Pose3 {
    Pose3 {
        position: Point3::new(pose.position[0], pose.position[1], pose.position[2]),
        orientation: Quaternion {
            x: pose.orientation_xyzw[0],
            y: pose.orientation_xyzw[1],
            z: pose.orientation_xyzw[2],
            w: pose.orientation_xyzw[3],
        },
    }
}

fn covariance_to_wire(value: &Covariance6) -> [f64; 36] {
    let mut wire = [0.0; 36];
    for (row_index, row) in value.0.iter().enumerate() {
        wire[row_index * 6..(row_index + 1) * 6].copy_from_slice(row);
    }
    wire
}

fn covariance_from_wire(value: [f64; 36]) -> Covariance6 {
    let mut matrix = [[0.0; 6]; 6];
    for (row_index, row) in matrix.iter_mut().enumerate() {
        row.copy_from_slice(&value[row_index * 6..(row_index + 1) * 6]);
    }
    Covariance6(matrix)
}

fn validate_occupancy_values(data: &[i8]) -> Result<(), DimosMessageError> {
    if data
        .iter()
        .any(|value| *value != -1 && !(0..=100).contains(value))
    {
        return Err(DimosMessageError::NumericRange { field: "data" });
    }
    Ok(())
}

fn point_field_size(datatype: u8) -> Option<u32> {
    match datatype {
        1 | 2 => Some(1),
        3 | 4 => Some(2),
        5..=7 => Some(4),
        8 => Some(8),
        _ => None,
    }
}

fn validate_point_cloud_layout(
    height: u32,
    width: u32,
    fields: &[PointField],
    point_step: u32,
    row_step: u32,
    data_len: usize,
) -> Result<(), DimosMessageError> {
    if width == 0 && row_step != 0 {
        return Err(DimosMessageError::UnsupportedPointLayout(
            "zero-width clouds must have zero row_step",
        ));
    }
    if width != 0 && point_step == 0 {
        return Err(DimosMessageError::UnsupportedPointLayout(
            "non-empty rows require a non-zero point_step",
        ));
    }

    let packed_row_step =
        width
            .checked_mul(point_step)
            .ok_or(DimosMessageError::UnsupportedPointLayout(
                "width * point_step overflows",
            ))?;
    if row_step < packed_row_step {
        return Err(DimosMessageError::UnsupportedPointLayout(
            "row_step is smaller than width * point_step",
        ));
    }

    for field in fields {
        if field.count == 0 {
            return Err(DimosMessageError::UnsupportedPointLayout(
                "point fields must have a non-zero count",
            ));
        }
        let scalar_size = point_field_size(field.datatype).ok_or(
            DimosMessageError::UnsupportedPointLayout("unknown point field datatype"),
        )?;
        let field_size = field.count.checked_mul(scalar_size).ok_or(
            DimosMessageError::UnsupportedPointLayout("point field size overflows"),
        )?;
        let field_end = field.offset.checked_add(field_size).ok_or(
            DimosMessageError::UnsupportedPointLayout("point field offset overflows"),
        )?;
        if field_end > point_step {
            return Err(DimosMessageError::UnsupportedPointLayout(
                "point field extends beyond point_step",
            ));
        }
    }

    let expected = (row_step as usize).checked_mul(height as usize).ok_or(
        DimosMessageError::UnsupportedPointLayout("row_step * height overflows"),
    )?;
    if data_len != expected {
        return Err(DimosMessageError::LengthMismatch {
            field: "data",
            expected,
            actual: data_len,
        });
    }
    Ok(())
}

impl TryFrom<&MessageHeader> for schema::Header {
    type Error = DimosMessageError;

    fn try_from(value: &MessageHeader) -> Result<Self, Self::Error> {
        let (stamp_sec, stamp_nsec) = split_timestamp(value.stamp)?;
        Ok(Self {
            seq: value.seq,
            stamp_sec,
            stamp_nsec,
            frame_id: value.frame_id.as_str().to_owned(),
        })
    }
}

impl TryFrom<schema::Header> for MessageHeader {
    type Error = DimosMessageError;

    fn try_from(value: schema::Header) -> Result<Self, Self::Error> {
        Ok(Self {
            seq: value.seq,
            stamp: join_timestamp(value.stamp_sec, value.stamp_nsec)?,
            frame_id: NavFrameId::from_static(&value.frame_id),
        })
    }
}

fn pose_stamped_to_wire(value: &PoseStamped) -> Result<schema::PoseStamped, DimosMessageError> {
    Ok(schema::PoseStamped {
        header: schema::Header::try_from(&MessageHeader {
            seq: 0,
            stamp: value.stamp,
            frame_id: value.frame_id.clone(),
        })?,
        pose: pose_to_wire(value.pose),
    })
}

fn pose_stamped_from_wire(value: schema::PoseStamped) -> Result<PoseStamped, DimosMessageError> {
    let header = MessageHeader::try_from(value.header)?;
    Ok(PoseStamped {
        frame_id: header.frame_id,
        stamp: header.stamp,
        pose: pose_from_wire(value.pose),
    })
}

impl TryFrom<&PathMessage> for schema::Path {
    type Error = DimosMessageError;

    fn try_from(value: &PathMessage) -> Result<Self, Self::Error> {
        Ok(Self {
            header: schema::Header::try_from(&value.header)?,
            poses: value
                .poses
                .iter()
                .map(pose_stamped_to_wire)
                .collect::<Result<_, _>>()?,
        })
    }
}

impl TryFrom<schema::Path> for PathMessage {
    type Error = DimosMessageError;

    fn try_from(value: schema::Path) -> Result<Self, Self::Error> {
        Ok(Self {
            header: MessageHeader::try_from(value.header)?,
            poses: value
                .poses
                .into_iter()
                .map(pose_stamped_from_wire)
                .collect::<Result<_, _>>()?,
        })
    }
}

impl From<&Path2D> for PathMessage {
    fn from(value: &Path2D) -> Self {
        let header = MessageHeader {
            seq: 0,
            stamp: value.stamp,
            frame_id: value.frame_id.clone(),
        };
        let poses = value
            .points
            .iter()
            .map(|point| PoseStamped {
                frame_id: value.frame_id.clone(),
                stamp: value.stamp,
                pose: Pose3 {
                    position: Point3::new(point.x, point.y, 0.0),
                    orientation: Quaternion::IDENTITY,
                },
            })
            .collect();
        Self { header, poses }
    }
}

impl From<&PathMessage> for Path2D {
    fn from(value: &PathMessage) -> Self {
        Self::stamped(
            value.header.frame_id.clone(),
            value.header.stamp,
            value
                .poses
                .iter()
                .map(|pose| Point2::new(pose.pose.position.x, pose.pose.position.y))
                .collect(),
        )
    }
}

impl TryFrom<&OccupancyGridMessage> for schema::OccupancyGrid {
    type Error = DimosMessageError;

    fn try_from(value: &OccupancyGridMessage) -> Result<Self, Self::Error> {
        let expected = value.info.width as usize * value.info.height as usize;
        if value.data.len() != expected {
            return Err(DimosMessageError::LengthMismatch {
                field: "data",
                expected,
                actual: value.data.len(),
            });
        }
        validate_occupancy_values(&value.data)?;
        let (map_load_time_sec, map_load_time_nsec) = split_timestamp(value.info.map_load_time)?;
        Ok(Self {
            header: schema::Header::try_from(&value.header)?,
            info: schema::MapMetaData {
                map_load_time_sec,
                map_load_time_nsec,
                resolution: value.info.resolution,
                width: i32::try_from(value.info.width)
                    .map_err(|_| DimosMessageError::NumericRange { field: "width" })?,
                height: i32::try_from(value.info.height)
                    .map_err(|_| DimosMessageError::NumericRange { field: "height" })?,
                origin: pose_to_wire(value.info.origin),
            },
            data: value.data.clone(),
        })
    }
}

impl TryFrom<schema::OccupancyGrid> for OccupancyGridMessage {
    type Error = DimosMessageError;

    fn try_from(value: schema::OccupancyGrid) -> Result<Self, Self::Error> {
        let width = u32::try_from(value.info.width)
            .map_err(|_| DimosMessageError::NumericRange { field: "width" })?;
        let height = u32::try_from(value.info.height)
            .map_err(|_| DimosMessageError::NumericRange { field: "height" })?;
        let expected = width as usize * height as usize;
        if value.data.len() != expected {
            return Err(DimosMessageError::LengthMismatch {
                field: "data",
                expected,
                actual: value.data.len(),
            });
        }
        validate_occupancy_values(&value.data)?;
        Ok(Self {
            header: MessageHeader::try_from(value.header)?,
            info: MapMetadata {
                map_load_time: join_timestamp(
                    value.info.map_load_time_sec,
                    value.info.map_load_time_nsec,
                )?,
                resolution: value.info.resolution,
                width,
                height,
                origin: pose_from_wire(value.info.origin),
            },
            data: value.data,
        })
    }
}

impl TryFrom<&OccupancyGrid2D> for OccupancyGridMessage {
    type Error = DimosMessageError;

    fn try_from(value: &OccupancyGrid2D) -> Result<Self, Self::Error> {
        let expected = value.width as usize * value.height as usize;
        if value.cells.len() != expected {
            return Err(DimosMessageError::LengthMismatch {
                field: "cells",
                expected,
                actual: value.cells.len(),
            });
        }
        let data = value
            .cells
            .iter()
            .map(|cell| match *cell {
                OccupancyCell::Unknown => Ok(-1),
                OccupancyCell::Free => Ok(0),
                OccupancyCell::Occupied => Ok(100),
                OccupancyCell::Inflated => Ok(99),
                OccupancyCell::Cost(cost) if cost <= 100 => Ok(cost as i8),
                OccupancyCell::Cost(_) => Err(DimosMessageError::NumericRange { field: "cells" }),
            })
            .collect::<Result<_, _>>()?;
        Ok(Self {
            header: MessageHeader::default(),
            info: MapMetadata {
                map_load_time: Timestamp::default(),
                resolution: value.resolution as f32,
                width: value.width,
                height: value.height,
                origin: Pose3 {
                    position: Point3::new(value.origin.x, value.origin.y, 0.0),
                    orientation: Quaternion::IDENTITY,
                },
            },
            data,
        })
    }
}

impl TryFrom<&OccupancyGridMessage> for OccupancyGrid2D {
    type Error = DimosMessageError;

    fn try_from(value: &OccupancyGridMessage) -> Result<Self, Self::Error> {
        let expected = value.info.width as usize * value.info.height as usize;
        if value.data.len() != expected {
            return Err(DimosMessageError::LengthMismatch {
                field: "data",
                expected,
                actual: value.data.len(),
            });
        }
        validate_occupancy_values(&value.data)?;
        let cells = value
            .data
            .iter()
            .map(|value| match *value {
                -1 => OccupancyCell::Unknown,
                0 => OccupancyCell::Free,
                100 => OccupancyCell::Occupied,
                value => OccupancyCell::Cost(value as u8),
            })
            .collect();
        Ok(Self::new(
            Point2::new(value.info.origin.position.x, value.info.origin.position.y),
            value.info.resolution.into(),
            value.info.width,
            value.info.height,
            cells,
        ))
    }
}

impl TryFrom<&TfMessage> for schema::TfMessage {
    type Error = DimosMessageError;

    fn try_from(value: &TfMessage) -> Result<Self, Self::Error> {
        Ok(Self {
            transforms: value
                .transforms
                .iter()
                .map(|item| {
                    Ok(schema::TransformStamped {
                        header: schema::Header::try_from(&item.header)?,
                        child_frame_id: item.child_frame_id.as_str().to_owned(),
                        transform: schema::Transform {
                            translation: [
                                item.transform.translation.x,
                                item.transform.translation.y,
                                item.transform.translation.z,
                            ],
                            rotation_xyzw: [
                                item.transform.rotation.x,
                                item.transform.rotation.y,
                                item.transform.rotation.z,
                                item.transform.rotation.w,
                            ],
                        },
                    })
                })
                .collect::<Result<_, DimosMessageError>>()?,
        })
    }
}

impl TryFrom<schema::TfMessage> for TfMessage {
    type Error = DimosMessageError;

    fn try_from(value: schema::TfMessage) -> Result<Self, Self::Error> {
        Ok(Self {
            transforms: value
                .transforms
                .into_iter()
                .map(|item| {
                    Ok(TransformStampedMessage {
                        header: MessageHeader::try_from(item.header)?,
                        child_frame_id: NavFrameId::from_static(&item.child_frame_id),
                        transform: Transform3 {
                            translation: Point3::new(
                                item.transform.translation[0],
                                item.transform.translation[1],
                                item.transform.translation[2],
                            ),
                            rotation: Quaternion {
                                x: item.transform.rotation_xyzw[0],
                                y: item.transform.rotation_xyzw[1],
                                z: item.transform.rotation_xyzw[2],
                                w: item.transform.rotation_xyzw[3],
                            },
                        },
                    })
                })
                .collect::<Result<_, DimosMessageError>>()?,
        })
    }
}

impl TryFrom<&OdometryMessage> for schema::Odometry {
    type Error = DimosMessageError;

    fn try_from(value: &OdometryMessage) -> Result<Self, Self::Error> {
        Ok(Self {
            header: schema::Header::try_from(&value.header)?,
            child_frame_id: value.child_frame_id.as_str().to_owned(),
            pose: pose_to_wire(value.pose),
            pose_covariance: covariance_to_wire(&value.pose_covariance),
            twist_linear: [value.linear.x, value.linear.y, value.linear.z],
            twist_angular: [value.angular.x, value.angular.y, value.angular.z],
            twist_covariance: covariance_to_wire(&value.twist_covariance),
        })
    }
}

impl TryFrom<schema::Odometry> for OdometryMessage {
    type Error = DimosMessageError;

    fn try_from(value: schema::Odometry) -> Result<Self, Self::Error> {
        Ok(Self {
            header: MessageHeader::try_from(value.header)?,
            child_frame_id: NavFrameId::from_static(&value.child_frame_id),
            pose: pose_from_wire(value.pose),
            pose_covariance: covariance_from_wire(value.pose_covariance),
            linear: Point3::new(
                value.twist_linear[0],
                value.twist_linear[1],
                value.twist_linear[2],
            ),
            angular: Point3::new(
                value.twist_angular[0],
                value.twist_angular[1],
                value.twist_angular[2],
            ),
            twist_covariance: covariance_from_wire(value.twist_covariance),
        })
    }
}

impl From<&Odometry> for OdometryMessage {
    fn from(value: &Odometry) -> Self {
        Self {
            header: MessageHeader {
                seq: 0,
                stamp: value.stamp,
                frame_id: value.frame_id.clone(),
            },
            child_frame_id: value.child_frame_id.clone(),
            pose: value.pose,
            pose_covariance: Covariance6::default(),
            linear: value.linear,
            angular: value.angular,
            twist_covariance: Covariance6::default(),
        }
    }
}

impl From<&OdometryMessage> for Odometry {
    fn from(value: &OdometryMessage) -> Self {
        Self {
            frame_id: value.header.frame_id.clone(),
            stamp: value.header.stamp,
            pose: value.pose,
            child_frame_id: value.child_frame_id.clone(),
            linear: value.linear,
            angular: value.angular,
        }
    }
}

impl TryFrom<&PointCloud2Message> for schema::PointCloud2 {
    type Error = DimosMessageError;

    fn try_from(value: &PointCloud2Message) -> Result<Self, Self::Error> {
        validate_point_cloud_layout(
            value.height,
            value.width,
            &value.fields,
            value.point_step,
            value.row_step,
            value.data.len(),
        )?;
        Ok(Self {
            header: schema::Header::try_from(&value.header)?,
            height: i32::try_from(value.height)
                .map_err(|_| DimosMessageError::NumericRange { field: "height" })?,
            width: i32::try_from(value.width)
                .map_err(|_| DimosMessageError::NumericRange { field: "width" })?,
            fields: value
                .fields
                .iter()
                .map(|field| {
                    Ok(schema::PointField {
                        name: field.name.clone(),
                        offset: i32::try_from(field.offset).map_err(|_| {
                            DimosMessageError::NumericRange {
                                field: "fields.offset",
                            }
                        })?,
                        datatype: field.datatype,
                        count: i32::try_from(field.count).map_err(|_| {
                            DimosMessageError::NumericRange {
                                field: "fields.count",
                            }
                        })?,
                    })
                })
                .collect::<Result<_, DimosMessageError>>()?,
            is_bigendian: value.is_bigendian,
            point_step: i32::try_from(value.point_step).map_err(|_| {
                DimosMessageError::NumericRange {
                    field: "point_step",
                }
            })?,
            row_step: i32::try_from(value.row_step)
                .map_err(|_| DimosMessageError::NumericRange { field: "row_step" })?,
            data: value.data.clone(),
            is_dense: value.is_dense,
        })
    }
}

impl TryFrom<schema::PointCloud2> for PointCloud2Message {
    type Error = DimosMessageError;

    fn try_from(value: schema::PointCloud2) -> Result<Self, Self::Error> {
        let height = u32::try_from(value.height)
            .map_err(|_| DimosMessageError::NumericRange { field: "height" })?;
        let width = u32::try_from(value.width)
            .map_err(|_| DimosMessageError::NumericRange { field: "width" })?;
        let point_step =
            u32::try_from(value.point_step).map_err(|_| DimosMessageError::NumericRange {
                field: "point_step",
            })?;
        let row_step = u32::try_from(value.row_step)
            .map_err(|_| DimosMessageError::NumericRange { field: "row_step" })?;
        let fields = value
            .fields
            .into_iter()
            .map(|field| {
                Ok(PointField {
                    name: field.name,
                    offset: u32::try_from(field.offset).map_err(|_| {
                        DimosMessageError::NumericRange {
                            field: "fields.offset",
                        }
                    })?,
                    datatype: field.datatype,
                    count: u32::try_from(field.count).map_err(|_| {
                        DimosMessageError::NumericRange {
                            field: "fields.count",
                        }
                    })?,
                })
            })
            .collect::<Result<Vec<_>, DimosMessageError>>()?;
        validate_point_cloud_layout(
            height,
            width,
            &fields,
            point_step,
            row_step,
            value.data.len(),
        )?;
        Ok(Self {
            header: MessageHeader::try_from(value.header)?,
            height,
            width,
            fields,
            is_bigendian: value.is_bigendian,
            point_step,
            row_step,
            data: value.data,
            is_dense: value.is_dense,
        })
    }
}
