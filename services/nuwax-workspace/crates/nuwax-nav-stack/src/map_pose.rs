#![cfg(any(feature = "guard-runtime", test))]

use std::time::{Duration, Instant};

use nuwax_codec::lcm::schema;
use thiserror::Error;

const DEFAULT_TF_TTL: Duration = Duration::from_millis(250);
const DEFAULT_TF_ALIGNMENT_TOLERANCE: Duration = Duration::from_millis(200);
const BASE_FRAME_ALIASES: &[&str] = &[
    "base",
    "base_link",
    "base_footprint",
    "body",
    "go2_base_link",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameRole {
    Map,
    Odom,
    World,
    Base,
    Other,
}

fn validated_frame_tokens(frame_id: &str) -> Option<Vec<&str>> {
    let frame_id = frame_id.trim();
    if frame_id.is_empty() {
        return None;
    }
    let tokens: Vec<_> = frame_id.split('/').collect();
    tokens
        .iter()
        .all(|token| {
            let mut chars = token.chars();
            chars
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && chars.all(|character| character.is_ascii_alphanumeric() || character == '_')
        })
        .then_some(tokens)
}

fn is_base_frame_leaf(leaf: &str) -> bool {
    BASE_FRAME_ALIASES.contains(&leaf)
        || leaf.ends_with("_base_link")
        || leaf.ends_with("_base_footprint")
}

pub(crate) fn valid_body_reference(frame: &str) -> bool {
    validated_frame_tokens(frame).is_some()
}

fn infer_frame_role(leaf: &str) -> FrameRole {
    match leaf {
        "map" => FrameRole::Map,
        "odom" | "odom_combined" => FrameRole::Odom,
        "world" => FrameRole::World,
        _ if is_base_frame_leaf(leaf) => FrameRole::Base,
        _ => FrameRole::Other,
    }
}

fn frame_ids_semantically_equal(left: &str, right: &str) -> bool {
    let (Some(left_tokens), Some(right_tokens)) =
        (validated_frame_tokens(left), validated_frame_tokens(right))
    else {
        return false;
    };
    if left_tokens == right_tokens {
        return true;
    }
    let (Some(left_leaf), Some(right_leaf)) = (left_tokens.last(), right_tokens.last()) else {
        return false;
    };
    let left_role = infer_frame_role(left_leaf);
    let right_role = infer_frame_role(right_leaf);
    if left_role != right_role {
        return false;
    }
    if left_tokens.len() > 1
        && right_tokens.len() > 1
        && left_tokens[..left_tokens.len() - 1] != right_tokens[..right_tokens.len() - 1]
    {
        return false;
    }
    matches!(
        left_role,
        FrameRole::Map | FrameRole::Odom | FrameRole::World | FrameRole::Base
    )
}

/// One real map-pose observation produced by composing map->odom with odometry.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MapPoseObservation {
    pub(crate) pose: schema::PoseStamped,
    pub(crate) generation: u64,
    pub(crate) map_frame: String,
    pub(crate) pose_frame: String,
}

/// Result of consuming one TF message. A coherent canonical chain can carry a
/// map pose directly; a map->odom-only update remains available to raw odometry.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MapPoseTfObservation {
    pub(crate) accepted: bool,
    pub(crate) map_pose: Option<MapPoseObservation>,
}

/// Fail-closed map-pose adapter errors.
#[derive(Debug, Error, Clone, PartialEq)]
pub(crate) enum MapPoseError {
    #[error("invalid map-pose adapter configuration: {0}")]
    InvalidConfig(String),
    #[error("frame mismatch for {field}: expected {expected}, got {actual}")]
    FrameMismatch {
        field: &'static str,
        expected: String,
        actual: String,
    },
    #[error("non-finite or zero-norm quaternion in {0}")]
    InvalidQuaternion(&'static str),
    #[error("non-finite translation in {0}")]
    InvalidTranslation(&'static str),
    #[error("invalid or missing source timestamp in {0}")]
    InvalidTimestamp(&'static str),
    #[error("map->odom transform receipt age {age_ms}ms exceeds TTL {ttl_ms}ms")]
    StaleTransform { age_ms: u128, ttl_ms: u128 },
    #[error("map->odom and odometry source timestamps differ by {skew_ms}ms (max {max_ms}ms)")]
    MisalignedTransform { skew_ms: u128, max_ms: u128 },
}

#[derive(Debug, Clone, Copy)]
struct RigidTransform {
    translation: [f64; 3],
    rotation_xyzw: [f64; 4],
    source_timestamp_ns: i64,
    received_at: Instant,
}

/// Stateful adapter that never relabels raw odometry as map pose.
pub(crate) struct MapPoseAdapter {
    map_frame: String,
    odom_frame: String,
    base_frame: String,
    strict_body_reference: bool,
    map_to_odom: Option<RigidTransform>,
    tf_ttl: Duration,
    alignment_tolerance: Duration,
    generation: u64,
}

impl MapPoseAdapter {
    pub(crate) fn new(
        map_frame: impl Into<String>,
        odom_frame: impl Into<String>,
        base_frame: impl Into<String>,
    ) -> Result<Self, MapPoseError> {
        Self::with_timing(
            map_frame,
            odom_frame,
            base_frame,
            DEFAULT_TF_TTL,
            DEFAULT_TF_ALIGNMENT_TOLERANCE,
        )
    }

    pub(crate) fn with_timing(
        map_frame: impl Into<String>,
        odom_frame: impl Into<String>,
        base_frame: impl Into<String>,
        tf_ttl: Duration,
        alignment_tolerance: Duration,
    ) -> Result<Self, MapPoseError> {
        let map_frame = map_frame.into().trim().to_string();
        let odom_frame = odom_frame.into().trim().to_string();
        let base_frame = base_frame.into().trim().to_string();
        if validated_frame_tokens(&map_frame).is_none()
            || validated_frame_tokens(&odom_frame).is_none()
            || validated_frame_tokens(&base_frame).is_none()
        {
            return Err(MapPoseError::InvalidConfig(
                "map, odom, and base frames must be valid frame IDs".into(),
            ));
        }
        if frame_ids_semantically_equal(&map_frame, &odom_frame)
            || frame_ids_semantically_equal(&odom_frame, &base_frame)
            || frame_ids_semantically_equal(&map_frame, &base_frame)
        {
            return Err(MapPoseError::InvalidConfig(
                "map, odom, and base frames must be distinct".into(),
            ));
        }
        if tf_ttl.is_zero() || alignment_tolerance.is_zero() {
            return Err(MapPoseError::InvalidConfig(
                "TF TTL and source alignment tolerance must be positive".into(),
            ));
        }
        Ok(Self {
            map_frame,
            odom_frame,
            base_frame,
            strict_body_reference: false,
            map_to_odom: None,
            tf_ttl,
            alignment_tolerance,
            generation: 0,
        })
    }

    /// Goal-bound execution requires the actual captured body reference, not role aliases.
    pub(crate) fn require_body_reference(mut self) -> Self {
        self.strict_body_reference = true;
        self
    }

    fn matches_base_frame(&self, actual: &str) -> bool {
        if self.strict_body_reference {
            actual.trim() == self.base_frame
        } else {
            frame_ids_semantically_equal(actual, &self.base_frame)
        }
    }

    /// Observe the latest real map->odom transform. Returns whether one was accepted.
    #[cfg(test)]
    pub(crate) fn observe_tf(&mut self, message: &schema::TfMessage) -> Result<bool, MapPoseError> {
        self.observe_tf_at(message, Instant::now())
    }

    /// Observe canonical TF and return an atomic map pose when the same message
    /// contains a source-aligned map->odom->base chain.
    pub(crate) fn observe_tf_message(
        &mut self,
        message: &schema::TfMessage,
    ) -> Result<MapPoseTfObservation, MapPoseError> {
        self.observe_tf_message_at(message, Instant::now())
    }

    #[cfg(test)]
    fn observe_tf_at(
        &mut self,
        message: &schema::TfMessage,
        received_at: Instant,
    ) -> Result<bool, MapPoseError> {
        self.observe_tf_message_at(message, received_at)
            .map(|observed| observed.accepted)
    }

    fn observe_tf_message_at(
        &mut self,
        message: &schema::TfMessage,
        received_at: Instant,
    ) -> Result<MapPoseTfObservation, MapPoseError> {
        let Some(transform) = message.transforms.iter().rev().find(|transform| {
            frame_ids_semantically_equal(&transform.header.frame_id, &self.map_frame)
                && frame_ids_semantically_equal(&transform.child_frame_id, &self.odom_frame)
        }) else {
            return Ok(MapPoseTfObservation {
                accepted: false,
                map_pose: None,
            });
        };
        validate_translation(transform.transform.translation, "map_to_odom")?;
        let rotation_xyzw =
            normalize_quaternion(transform.transform.rotation_xyzw, "map_to_odom.rotation")?;
        let source_timestamp_ns = header_timestamp_ns(&transform.header, "map_to_odom.header")?;
        if self
            .map_to_odom
            .is_some_and(|cached| source_timestamp_ns <= cached.source_timestamp_ns)
        {
            return Ok(MapPoseTfObservation {
                accepted: false,
                map_pose: None,
            });
        }
        let map_to_odom = RigidTransform {
            translation: transform.transform.translation,
            rotation_xyzw,
            source_timestamp_ns,
            received_at,
        };
        let generation = self.generation.saturating_add(1).max(1);
        let map_pose = message
            .transforms
            .iter()
            .rev()
            .find(|transform| {
                frame_ids_semantically_equal(&transform.header.frame_id, &self.odom_frame)
                    && self.matches_base_frame(&transform.child_frame_id)
            })
            .map(|odom_to_base| {
                validate_translation(odom_to_base.transform.translation, "odom_to_base")?;
                let odom_rotation = normalize_quaternion(
                    odom_to_base.transform.rotation_xyzw,
                    "odom_to_base.rotation",
                )?;
                let odom_timestamp_ns =
                    header_timestamp_ns(&odom_to_base.header, "odom_to_base.header")?;
                if odom_timestamp_ns != source_timestamp_ns {
                    let skew_ns = i128::from(odom_timestamp_ns)
                        .saturating_sub(i128::from(source_timestamp_ns))
                        .unsigned_abs();
                    return Err(MapPoseError::MisalignedTransform {
                        skew_ms: skew_ns / 1_000_000,
                        max_ms: 0,
                    });
                }
                let rotated = rotate_vector(
                    map_to_odom.rotation_xyzw,
                    odom_to_base.transform.translation,
                );
                let position = [
                    map_to_odom.translation[0] + rotated[0],
                    map_to_odom.translation[1] + rotated[1],
                    map_to_odom.translation[2] + rotated[2],
                ];
                let orientation_xyzw = normalize_quaternion(
                    multiply_quaternion(map_to_odom.rotation_xyzw, odom_rotation),
                    "map_pose.rotation",
                )?;
                let mut header = odom_to_base.header.clone();
                header.frame_id = self.map_frame.clone();
                header.seq = i32::try_from(generation).unwrap_or(i32::MAX);
                Ok(MapPoseObservation {
                    pose: schema::PoseStamped {
                        header,
                        pose: schema::Pose {
                            position,
                            orientation_xyzw,
                        },
                    },
                    generation,
                    map_frame: self.map_frame.clone(),
                    pose_frame: self.map_frame.clone(),
                })
            })
            .transpose()?;
        self.map_to_odom = Some(map_to_odom);
        self.generation = generation;
        Ok(MapPoseTfObservation {
            accepted: true,
            map_pose,
        })
    }

    /// Transform odom->base into map->base. Missing TF is a normal waiting state.
    pub(crate) fn transform_odometry(
        &self,
        odometry: &schema::Odometry,
    ) -> Result<Option<MapPoseObservation>, MapPoseError> {
        self.transform_odometry_at(odometry, Instant::now())
    }

    fn transform_odometry_at(
        &self,
        odometry: &schema::Odometry,
        now: Instant,
    ) -> Result<Option<MapPoseObservation>, MapPoseError> {
        if !self.matches_base_frame(&odometry.child_frame_id) {
            return Err(MapPoseError::FrameMismatch {
                field: "odometry.child_frame_id",
                expected: self.base_frame.clone(),
                actual: odometry.child_frame_id.clone(),
            });
        }
        validate_translation(odometry.pose.position, "odometry.pose")?;
        let odom_rotation =
            normalize_quaternion(odometry.pose.orientation_xyzw, "odometry.pose.rotation")?;
        let odometry_timestamp_ns = header_timestamp_ns(&odometry.header, "odometry.header")?;

        if frame_ids_semantically_equal(&odometry.header.frame_id, &self.map_frame) {
            let mut header = odometry.header.clone();
            header.seq = i32::try_from(self.generation.max(1)).unwrap_or(i32::MAX);
            return Ok(Some(MapPoseObservation {
                pose: schema::PoseStamped {
                    header,
                    pose: schema::Pose {
                        position: odometry.pose.position,
                        orientation_xyzw: odom_rotation,
                    },
                },
                generation: self.generation.max(1),
                map_frame: self.map_frame.clone(),
                pose_frame: self.map_frame.clone(),
            }));
        }
        if !frame_ids_semantically_equal(&odometry.header.frame_id, &self.odom_frame) {
            return Err(MapPoseError::FrameMismatch {
                field: "odometry.header.frame_id",
                expected: self.odom_frame.clone(),
                actual: odometry.header.frame_id.clone(),
            });
        }
        let Some(map_to_odom) = self.map_to_odom else {
            return Ok(None);
        };
        let receipt_age = now.duration_since(map_to_odom.received_at);
        if receipt_age > self.tf_ttl {
            return Err(MapPoseError::StaleTransform {
                age_ms: receipt_age.as_millis(),
                ttl_ms: self.tf_ttl.as_millis(),
            });
        }
        let source_skew_ns = i128::from(odometry_timestamp_ns)
            .saturating_sub(i128::from(map_to_odom.source_timestamp_ns))
            .unsigned_abs();
        if source_skew_ns > self.alignment_tolerance.as_nanos() {
            return Err(MapPoseError::MisalignedTransform {
                skew_ms: source_skew_ns / 1_000_000,
                max_ms: self.alignment_tolerance.as_millis(),
            });
        }
        let rotated = rotate_vector(map_to_odom.rotation_xyzw, odometry.pose.position);
        let position = [
            map_to_odom.translation[0] + rotated[0],
            map_to_odom.translation[1] + rotated[1],
            map_to_odom.translation[2] + rotated[2],
        ];
        let orientation_xyzw = normalize_quaternion(
            multiply_quaternion(map_to_odom.rotation_xyzw, odom_rotation),
            "map_pose.rotation",
        )?;
        let mut header = odometry.header.clone();
        header.frame_id = self.map_frame.clone();
        header.seq = i32::try_from(self.generation).unwrap_or(i32::MAX);
        Ok(Some(MapPoseObservation {
            pose: schema::PoseStamped {
                header,
                pose: schema::Pose {
                    position,
                    orientation_xyzw,
                },
            },
            generation: self.generation,
            map_frame: self.map_frame.clone(),
            pose_frame: self.map_frame.clone(),
        }))
    }
}

fn header_timestamp_ns(header: &schema::Header, label: &'static str) -> Result<i64, MapPoseError> {
    if header.stamp_sec < 0 || !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err(MapPoseError::InvalidTimestamp(label));
    }
    let timestamp = i64::from(header.stamp_sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(header.stamp_nsec)))
        .filter(|timestamp| *timestamp > 0)
        .ok_or(MapPoseError::InvalidTimestamp(label))?;
    Ok(timestamp)
}

fn validate_translation(value: [f64; 3], label: &'static str) -> Result<(), MapPoseError> {
    if value.into_iter().all(f64::is_finite) {
        Ok(())
    } else {
        Err(MapPoseError::InvalidTranslation(label))
    }
}

fn normalize_quaternion(value: [f64; 4], label: &'static str) -> Result<[f64; 4], MapPoseError> {
    if !value.into_iter().all(f64::is_finite) {
        return Err(MapPoseError::InvalidQuaternion(label));
    }
    let norm = value
        .into_iter()
        .map(|part| part * part)
        .sum::<f64>()
        .sqrt();
    if norm <= f64::EPSILON {
        return Err(MapPoseError::InvalidQuaternion(label));
    }
    Ok(value.map(|part| part / norm))
}

fn multiply_quaternion(left: [f64; 4], right: [f64; 4]) -> [f64; 4] {
    let [lx, ly, lz, lw] = left;
    let [rx, ry, rz, rw] = right;
    [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]
}

fn rotate_vector(rotation: [f64; 4], vector: [f64; 3]) -> [f64; 3] {
    let [x, y, z, w] = rotation;
    let [vx, vy, vz] = vector;
    let tx = 2.0 * (y * vz - z * vy);
    let ty = 2.0 * (z * vx - x * vz);
    let tz = 2.0 * (x * vy - y * vx);
    [
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ]
}

#[cfg(test)]
mod tests {
    use nuwax_codec::lcm::schema;

    use super::*;

    fn odometry(frame: &str, child: &str, x: f64, y: f64) -> schema::Odometry {
        schema::Odometry {
            header: schema::Header {
                stamp_sec: 1,
                frame_id: frame.into(),
                ..schema::Header::default()
            },
            child_frame_id: child.into(),
            pose: schema::Pose {
                position: [x, y, 0.0],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            ..schema::Odometry::default()
        }
    }

    fn map_to_odom(x: f64, y: f64) -> schema::TfMessage {
        schema::TfMessage {
            transforms: vec![schema::TransformStamped {
                header: schema::Header {
                    stamp_sec: 1,
                    frame_id: "map".into(),
                    ..schema::Header::default()
                },
                child_frame_id: "odom".into(),
                transform: schema::Transform {
                    translation: [x, y, 0.0],
                    rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
            }],
        }
    }

    fn coherent_tf_chain(map_x: f64, map_y: f64, odom_x: f64, odom_y: f64) -> schema::TfMessage {
        let mut message = map_to_odom(map_x, map_y);
        message.transforms.push(schema::TransformStamped {
            header: schema::Header {
                stamp_sec: 1,
                frame_id: "odom".into(),
                ..schema::Header::default()
            },
            child_frame_id: "base_link".into(),
            transform: schema::Transform {
                translation: [odom_x, odom_y, 0.0],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
        });
        message
    }

    #[test]
    fn raw_odom_waits_for_real_transform_without_crashing_or_relabelling() {
        let adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let result = adapter
            .transform_odometry(&odometry("odom", "base_link", 1.0, 2.0))
            .expect("raw odom is a valid waiting input");
        assert!(result.is_none());
    }

    #[test]
    fn map_to_odom_composes_with_real_odom_pose() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        assert!(adapter.observe_tf(&map_to_odom(10.0, -2.0)).expect("TF"));
        let observed = adapter
            .transform_odometry(&odometry("odom", "base_link", 1.5, 3.0))
            .expect("transform")
            .expect("map pose");
        assert_eq!(observed.pose.header.frame_id, "map");
        assert_eq!(observed.pose.pose.position, [11.5, 1.0, 0.0]);
        assert_eq!(observed.generation, 1);
        assert_eq!(observed.map_frame, "map");
        assert_eq!(observed.pose_frame, "map");
    }

    #[test]
    fn mismatched_source_frame_is_rejected_instead_of_renamed() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        adapter.observe_tf(&map_to_odom(0.0, 0.0)).expect("TF");
        let error = adapter
            .transform_odometry(&odometry("camera", "base_link", 1.0, 2.0))
            .expect_err("foreign frame must fail closed");
        assert!(matches!(error, MapPoseError::FrameMismatch { .. }));
    }

    #[test]
    fn base_frame_aliases_are_accepted_but_foreign_namespaces_are_rejected() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "robot/base_link").expect("adapter");
        adapter.observe_tf(&map_to_odom(0.0, 0.0)).expect("TF");

        assert!(adapter
            .transform_odometry(&odometry("odom", "base_footprint", 1.0, 2.0))
            .expect("unnamespaced base alias")
            .is_some());
        assert!(adapter
            .transform_odometry(&odometry("odom", "robot/body", 1.0, 2.0))
            .expect("same-namespace base alias")
            .is_some());

        let error = adapter
            .transform_odometry(&odometry("odom", "other/base_footprint", 1.0, 2.0))
            .expect_err("foreign namespace must fail closed");
        assert!(matches!(error, MapPoseError::FrameMismatch { .. }));
        let error = adapter
            .transform_odometry(&odometry("odom", "camera", 1.0, 2.0))
            .expect_err("foreign frame role must fail closed");
        assert!(matches!(error, MapPoseError::FrameMismatch { .. }));
    }

    #[test]
    fn surface_reference_does_not_alias_footprint_height_to_body() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link")
            .unwrap().require_body_reference();
        adapter.observe_tf(&map_to_odom(0.0, 0.0)).unwrap();
        assert!(adapter.transform_odometry(&odometry("odom", "base_footprint", 1.0, 2.0)).is_err());
        let mut body = odometry("odom", "base_link", 1.0, 2.0);
        body.pose.position[2] = 0.4;
        let observed = adapter.transform_odometry(&body).unwrap().unwrap();
        assert_eq!(observed.pose.pose.position[2], 0.4);
    }

    #[test]
    fn strict_body_binding_selects_the_actual_child_even_when_role_alias_is_last() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "robot/body_a")
            .unwrap().require_body_reference();
        let mut chain = coherent_tf_chain(0.0, 0.0, 1.0, 2.0);
        chain.transforms[1].child_frame_id = "robot/body_a".into();
        let mut foreign = chain.transforms[1].clone();
        foreign.child_frame_id = "robot/base_link".into();
        foreign.transform.translation[0] = 9.0;
        chain.transforms.push(foreign.clone());
        let observed = adapter.observe_tf_message(&chain).unwrap().map_pose.unwrap();
        assert_eq!(observed.pose.pose.position[0], 1.0);
        let mut foreign_only = map_to_odom(0.0, 0.0);
        foreign_only.transforms.push(foreign);
        assert!(adapter.observe_tf_message(&foreign_only).unwrap().map_pose.is_none());
        assert!(adapter.transform_odometry(&odometry("odom", "robot/base_link", 9.0, 2.0)).is_err());
        assert!(adapter.transform_odometry(&odometry("odom", "robot/body_a", 1.0, 2.0)).unwrap().is_some());
    }

    #[test]
    fn strict_body_binding_does_not_treat_body_and_base_link_as_physical_synonyms() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "body")
            .unwrap().require_body_reference();
        adapter.observe_tf(&map_to_odom(0.0, 0.0)).unwrap();
        assert!(adapter.transform_odometry(&odometry("odom", "base_link", 1.0, 2.0)).is_err());
        assert!(adapter.transform_odometry(&odometry("odom", "body", 1.0, 2.0)).unwrap().is_some());
    }

    #[test]
    fn coherent_tf_chain_accepts_semantic_base_alias() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_footprint").expect("adapter");
        let observed = adapter
            .observe_tf_message(&coherent_tf_chain(10.0, -2.0, 1.5, 3.0))
            .expect("base_link is a base_footprint alias");

        assert!(observed.map_pose.is_some());
    }

    #[test]
    fn frame_semantics_match_python_contract_parity_matrix() {
        let cases = [
            ("map", "world/go2/map", true),
            ("world/go2/map", "world/other_robot/map", false),
            ("odom", "odom_combined", true),
            ("robot/odom", "robot/odom_combined", true),
            ("robot/odom", "other/odom_combined", false),
            ("world", "robot/world", true),
            ("body", "world/go2/body", true),
            ("robot/chassis_base_link", "robot/base_footprint", true),
            ("robot/chassis_base_link", "other/base_link", false),
            ("map", "odom", false),
            ("camera", "robot/camera", false),
            ("camera", "camera", true),
        ];

        for (left, right, expected) in cases {
            assert_eq!(
                frame_ids_semantically_equal(left, right),
                expected,
                "semantic comparison mismatch for {left:?} and {right:?}"
            );
            assert_eq!(
                frame_ids_semantically_equal(right, left),
                expected,
                "semantic comparison must be symmetric for {left:?} and {right:?}"
            );
        }
    }

    #[test]
    fn frame_semantics_fail_closed_for_invalid_ids() {
        assert!(!frame_ids_semantically_equal("/base_link", "base_link"));
        assert!(!frame_ids_semantically_equal("base-link", "base_link"));
    }

    #[test]
    fn scoped_map_and_odom_aliases_are_accepted_across_tf_and_odometry() {
        let mut adapter =
            MapPoseAdapter::new("robot/map", "robot/odom_combined", "base_link").expect("adapter");

        let observed = adapter
            .observe_tf_message(&coherent_tf_chain(10.0, -2.0, 1.5, 3.0))
            .expect("unscoped TF chain is semantically compatible");
        let map_pose = observed.map_pose.expect("atomic map pose");
        assert_eq!(map_pose.pose.header.frame_id, "robot/map");

        let fallback = adapter
            .transform_odometry(&odometry("odom", "base_footprint", 1.5, 3.0))
            .expect("odom alias is semantically compatible")
            .expect("fallback map pose");
        assert_eq!(fallback.pose.header.frame_id, "robot/map");
    }

    #[test]
    fn semantically_duplicate_configured_frames_are_rejected() {
        let error = match MapPoseAdapter::new("map", "robot/map", "base_link") {
            Ok(_) => panic!("semantic aliases cannot describe distinct chain roles"),
            Err(error) => error,
        };
        assert!(matches!(error, MapPoseError::InvalidConfig(_)));
    }

    #[test]
    fn quaternion_rotation_is_applied_to_translation() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let mut tf = map_to_odom(1.0, 2.0);
        tf.transforms[0].transform.rotation_xyzw = [0.0, 0.0, half, half];
        adapter.observe_tf(&tf).expect("TF");
        let observed = adapter
            .transform_odometry(&odometry("odom", "base_link", 1.0, 0.0))
            .expect("transform")
            .expect("map pose");
        assert!((observed.pose.pose.position[0] - 1.0).abs() < 1e-9);
        assert!((observed.pose.pose.position[1] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn stale_tf_edge_is_rejected_instead_of_reused_forever() {
        let ttl = Duration::from_millis(20);
        let mut adapter = MapPoseAdapter::with_timing(
            "map",
            "odom",
            "base_link",
            ttl,
            Duration::from_millis(200),
        )
        .expect("adapter");
        let observed_at = Instant::now();
        adapter
            .observe_tf_at(&map_to_odom(0.0, 0.0), observed_at)
            .expect("TF");
        let error = adapter
            .transform_odometry_at(
                &odometry("odom", "base_link", 1.0, 2.0),
                observed_at + ttl + Duration::from_millis(1),
            )
            .expect_err("expired TF must fail closed");
        assert!(matches!(error, MapPoseError::StaleTransform { .. }));
    }

    #[test]
    fn tf_and_odometry_must_be_source_time_aligned() {
        let mut adapter = MapPoseAdapter::with_timing(
            "map",
            "odom",
            "base_link",
            Duration::from_secs(1),
            Duration::from_millis(100),
        )
        .expect("adapter");
        let now = Instant::now();
        adapter
            .observe_tf_at(&map_to_odom(0.0, 0.0), now)
            .expect("TF");
        let mut delayed_odom = odometry("odom", "base_link", 1.0, 2.0);
        delayed_odom.header.stamp_nsec = 101_000_000;
        let error = adapter
            .transform_odometry_at(&delayed_odom, now)
            .expect_err("source-time skew must fail closed");
        assert!(matches!(error, MapPoseError::MisalignedTransform { .. }));
    }

    #[test]
    fn coherent_tf_message_produces_map_pose_without_cross_topic_join() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let observed = adapter
            .observe_tf_message(&coherent_tf_chain(10.0, -2.0, 1.5, 3.0))
            .expect("coherent TF chain");

        assert!(observed.accepted);
        let map_pose = observed.map_pose.expect("atomic map pose");
        assert_eq!(map_pose.pose.header.frame_id, "map");
        assert_eq!(map_pose.pose.pose.position, [11.5, 1.0, 0.0]);
        assert_eq!(map_pose.generation, 1);
    }

    #[test]
    fn atomic_tf_chain_requires_identical_source_timestamps() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let mut message = coherent_tf_chain(10.0, -2.0, 1.5, 3.0);
        message.transforms[1].header.stamp_nsec = 1;

        let error = adapter
            .observe_tf_message(&message)
            .expect_err("mixed-source TF chain must fail closed");
        assert!(matches!(error, MapPoseError::MisalignedTransform { .. }));
        assert!(adapter
            .transform_odometry(&odometry("odom", "base_link", 1.5, 3.0))
            .expect("raw odometry remains a waiting input")
            .is_none());
    }

    #[test]
    fn incomplete_tf_chain_keeps_raw_odometry_fallback() {
        let mut adapter = MapPoseAdapter::new("map", "odom", "base_link").expect("adapter");
        let observed = adapter
            .observe_tf_message(&map_to_odom(10.0, -2.0))
            .expect("map-to-odom TF");

        assert!(observed.accepted);
        assert!(observed.map_pose.is_none());
        let fallback = adapter
            .transform_odometry(&odometry("odom", "base_link", 1.5, 3.0))
            .expect("aligned raw odometry")
            .expect("fallback map pose");
        assert_eq!(fallback.pose.pose.position, [11.5, 1.0, 0.0]);
    }
}
