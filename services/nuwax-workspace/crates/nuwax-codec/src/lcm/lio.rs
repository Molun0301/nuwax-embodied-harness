//! Versioned LIO ingress envelopes generated from the repository LCM schemas.

use super::schema::{
    decode_odometry, decode_point_cloud2, encode_odometry_body, encode_point_cloud2_body, Odometry,
    PointCloud2, ODOMETRY_FINGERPRINT, POINT_CLOUD2_FINGERPRINT,
};
use crate::{CodecError, CodecResult};

/// Locked fingerprint for `nuwax_msgs.LioOdometryEnvelope` schema V1.
pub const LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1: [u8; 8] = 0x742d_9b06_c0c7_a223_u64.to_be_bytes();
/// Locked fingerprint for `nuwax_msgs.LioCloudEnvelope` schema V1.
pub const LIO_CLOUD_ENVELOPE_FINGERPRINT_V1: [u8; 8] = 0xdcd4_ae0f_83a9_a81b_u64.to_be_bytes();
/// Locked fingerprint for `nuwax_msgs.LioStatus` schema V1.
pub const LIO_STATUS_FINGERPRINT_V1: [u8; 8] = 0x6d7a_e636_d8ba_41b2_u64.to_be_bytes();

/// First public LIO schema version.
pub const LIO_SCHEMA_VERSION_V1: i32 = 1;
/// Device-owned passthrough backend.
pub const LIO_BACKEND_DEVICE: i32 = 1;
/// Native FastLIO2 backend.
pub const LIO_BACKEND_FASTLIO2: i32 = 2;
/// Native Point-LIO backend.
pub const LIO_BACKEND_POINTLIO: i32 = 3;

/// A raw sensor cloud. Public ingress rejects this state.
pub const LIO_REGISTRATION_RAW_SENSOR: i32 = 1;
/// A cloud registered into the map frame by its LIO owner.
pub const LIO_REGISTRATION_REGISTERED_MAP: i32 = 2;

/// LIO owner is starting.
pub const LIO_LIFECYCLE_STARTING: i32 = 1;
/// LIO owner is ready.
pub const LIO_LIFECYCLE_READY: i32 = 2;
/// LIO owner is stopping.
pub const LIO_LIFECYCLE_STOPPING: i32 = 3;
/// LIO owner stopped.
pub const LIO_LIFECYCLE_STOPPED: i32 = 4;
/// LIO owner failed.
pub const LIO_LIFECYCLE_FAILED: i32 = 5;

/// No failure or shutdown reason.
pub const LIO_REASON_NONE: i32 = 0;
/// Owner startup timed out.
pub const LIO_REASON_STARTUP_TIMEOUT: i32 = 1;
/// Native child exited normally.
pub const LIO_REASON_CHILD_EXITED: i32 = 2;
/// Native child crashed.
pub const LIO_REASON_CHILD_CRASHED: i32 = 3;
/// Shutdown was explicitly requested.
pub const LIO_REASON_SHUTDOWN_REQUESTED: i32 = 4;
/// Startup preflight rejected the configuration.
pub const LIO_REASON_PREFLIGHT_FAILED: i32 = 5;
/// The private LCM transport failed.
pub const LIO_REASON_TRANSPORT_ERROR: i32 = 6;

/// Maximum point count accepted from a native LIO source.
pub const MAX_LIO_SOURCE_POINT_COUNT: i64 = 250_000;
/// Maximum registered point count admitted into Geometry.
pub const MAX_LIO_EMITTED_POINT_COUNT: i64 = 100_000;

const MAX_FRAME_ID_BYTES: usize = 255;
const MAX_REASON_BYTES: usize = 1_024;
const NANOS_PER_SECOND: i64 = 1_000_000_000;
const ODOMETRY_QUATERNION_NORM_TOLERANCE: f64 = 1.0e-3;

/// Versioned odometry sample emitted by exactly one LIO owner generation.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LioOdometryEnvelope {
    /// Wire schema version.
    pub schema_version: i32,
    /// One of the `LIO_BACKEND_*` constants.
    pub backend: i32,
    /// Unique owner-process generation.
    pub runtime_generation: i64,
    /// Monotonic source sequence within the generation.
    pub source_sequence: i64,
    /// Source timestamp in nanoseconds; zero is a valid present timestamp.
    pub source_timestamp_ns: i64,
    /// Authoritative odometry frame.
    pub odom_frame_id: String,
    /// Authoritative robot body frame.
    pub body_frame_id: String,
    /// Map metadata frame for the owner contract.
    pub map_frame_id: String,
    /// Nested standard odometry body.
    pub odometry: Odometry,
}

/// Versioned registered cloud correlated to an odometry source sample.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct LioCloudEnvelope {
    /// Wire schema version.
    pub schema_version: i32,
    /// One of the `LIO_BACKEND_*` constants.
    pub backend: i32,
    /// Unique owner-process generation.
    pub runtime_generation: i64,
    /// Monotonic cloud source sequence.
    pub source_sequence: i64,
    /// Cloud source timestamp in nanoseconds; zero is a valid present timestamp.
    pub source_timestamp_ns: i64,
    /// Correlated odometry source sequence.
    pub correlated_odometry_source_sequence: i64,
    /// Correlated odometry timestamp; zero is valid with a positive sequence.
    pub correlated_odometry_source_timestamp_ns: i64,
    /// Original sensor frame metadata.
    pub sensor_frame_id: String,
    /// Robot body frame metadata.
    pub body_frame_id: String,
    /// Odometry frame metadata.
    pub odom_frame_id: String,
    /// Authoritative registered-cloud frame.
    pub map_frame_id: String,
    /// One of the `LIO_REGISTRATION_*` constants.
    pub registration_state: i32,
    /// Point count before deterministic downsampling.
    pub source_point_count: i64,
    /// Point count after deterministic downsampling.
    pub emitted_point_count: i64,
    /// Nested standard point-cloud body.
    pub cloud: PointCloud2,
}

/// Versioned lifecycle update for one LIO owner generation.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LioStatus {
    /// Wire schema version.
    pub schema_version: i32,
    /// One of the `LIO_BACKEND_*` constants.
    pub backend: i32,
    /// Unique owner-process generation.
    pub runtime_generation: i64,
    /// Monotonic status sequence.
    pub status_sequence: i64,
    /// Status timestamp in nanoseconds; zero is a valid present timestamp.
    pub status_timestamp_ns: i64,
    /// Last emitted odometry sequence, or zero when absent.
    pub last_odometry_source_sequence: i64,
    /// Last odometry timestamp; absence is determined by the sequence pair.
    pub last_odometry_source_timestamp_ns: i64,
    /// Last emitted cloud sequence, or zero when absent.
    pub last_cloud_source_sequence: i64,
    /// Last cloud timestamp; absence is determined by the sequence pair.
    pub last_cloud_source_timestamp_ns: i64,
    /// Sensor frame metadata.
    pub sensor_frame_id: String,
    /// Robot body frame metadata.
    pub body_frame_id: String,
    /// Odometry frame metadata.
    pub odom_frame_id: String,
    /// Map frame metadata.
    pub map_frame_id: String,
    /// One of the `LIO_LIFECYCLE_*` constants.
    pub lifecycle_state: i32,
    /// One of the `LIO_REASON_*` constants.
    pub reason_code: i32,
    /// Bounded human-readable lifecycle reason.
    pub reason: String,
}

/// Encode a complete generated `LioOdometryEnvelope` message.
pub fn encode_lio_odometry_envelope(message: &LioOdometryEnvelope) -> CodecResult<Vec<u8>> {
    validate_odometry(message).map_err(encode_error)?;
    let mut out = Vec::new();
    out.extend_from_slice(&LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1);
    encode_common_identity(
        message.schema_version,
        message.backend,
        message.runtime_generation,
        message.source_sequence,
        message.source_timestamp_ns,
        &mut out,
    );
    encode_string(&message.odom_frame_id, &mut out)?;
    encode_string(&message.body_frame_id, &mut out)?;
    encode_string(&message.map_frame_id, &mut out)?;
    out.extend_from_slice(&encode_odometry_body(&message.odometry)?);
    Ok(out)
}

/// Decode and validate a complete generated `LioOdometryEnvelope` message.
pub fn decode_lio_odometry_envelope(encoded: &[u8]) -> CodecResult<LioOdometryEnvelope> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(
        LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1,
        "nuwax_msgs.LioOdometryEnvelope",
    )?;
    let (schema_version, backend, runtime_generation, source_sequence, source_timestamp_ns) =
        decode_common_identity(&mut cursor)?;
    let odom_frame_id = cursor.read_string()?;
    let body_frame_id = cursor.read_string()?;
    let map_frame_id = cursor.read_string()?;
    let odometry = decode_nested_odometry(cursor.remaining_slice())?;
    let message = LioOdometryEnvelope {
        schema_version,
        backend,
        runtime_generation,
        source_sequence,
        source_timestamp_ns,
        odom_frame_id,
        body_frame_id,
        map_frame_id,
        odometry,
    };
    validate_odometry(&message).map_err(decode_error)?;
    Ok(message)
}

/// Encode a complete generated `LioCloudEnvelope` message.
pub fn encode_lio_cloud_envelope(message: &LioCloudEnvelope) -> CodecResult<Vec<u8>> {
    validate_cloud(message).map_err(encode_error)?;
    let mut out = Vec::new();
    out.extend_from_slice(&LIO_CLOUD_ENVELOPE_FINGERPRINT_V1);
    encode_common_identity(
        message.schema_version,
        message.backend,
        message.runtime_generation,
        message.source_sequence,
        message.source_timestamp_ns,
        &mut out,
    );
    out.extend_from_slice(&message.correlated_odometry_source_sequence.to_be_bytes());
    out.extend_from_slice(
        &message
            .correlated_odometry_source_timestamp_ns
            .to_be_bytes(),
    );
    encode_string(&message.sensor_frame_id, &mut out)?;
    encode_string(&message.body_frame_id, &mut out)?;
    encode_string(&message.odom_frame_id, &mut out)?;
    encode_string(&message.map_frame_id, &mut out)?;
    out.extend_from_slice(&message.registration_state.to_be_bytes());
    out.extend_from_slice(&message.source_point_count.to_be_bytes());
    out.extend_from_slice(&message.emitted_point_count.to_be_bytes());
    out.extend_from_slice(&encode_point_cloud2_body(&message.cloud)?);
    Ok(out)
}

/// Decode and validate a complete generated `LioCloudEnvelope` message.
pub fn decode_lio_cloud_envelope(encoded: &[u8]) -> CodecResult<LioCloudEnvelope> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(
        LIO_CLOUD_ENVELOPE_FINGERPRINT_V1,
        "nuwax_msgs.LioCloudEnvelope",
    )?;
    let (schema_version, backend, runtime_generation, source_sequence, source_timestamp_ns) =
        decode_common_identity(&mut cursor)?;
    let correlated_odometry_source_sequence = cursor.read_i64()?;
    let correlated_odometry_source_timestamp_ns = cursor.read_i64()?;
    let sensor_frame_id = cursor.read_string()?;
    let body_frame_id = cursor.read_string()?;
    let odom_frame_id = cursor.read_string()?;
    let map_frame_id = cursor.read_string()?;
    let registration_state = cursor.read_i32()?;
    let source_point_count = cursor.read_i64()?;
    let emitted_point_count = cursor.read_i64()?;
    let cloud = decode_nested_cloud(cursor.remaining_slice())?;
    let message = LioCloudEnvelope {
        schema_version,
        backend,
        runtime_generation,
        source_sequence,
        source_timestamp_ns,
        correlated_odometry_source_sequence,
        correlated_odometry_source_timestamp_ns,
        sensor_frame_id,
        body_frame_id,
        odom_frame_id,
        map_frame_id,
        registration_state,
        source_point_count,
        emitted_point_count,
        cloud,
    };
    validate_cloud(&message).map_err(decode_error)?;
    Ok(message)
}

/// Encode a complete generated `LioStatus` message.
pub fn encode_lio_status(message: &LioStatus) -> CodecResult<Vec<u8>> {
    validate_status(message).map_err(encode_error)?;
    let mut out = Vec::new();
    out.extend_from_slice(&LIO_STATUS_FINGERPRINT_V1);
    out.extend_from_slice(&message.schema_version.to_be_bytes());
    out.extend_from_slice(&message.backend.to_be_bytes());
    for value in [
        message.runtime_generation,
        message.status_sequence,
        message.status_timestamp_ns,
        message.last_odometry_source_sequence,
        message.last_odometry_source_timestamp_ns,
        message.last_cloud_source_sequence,
        message.last_cloud_source_timestamp_ns,
    ] {
        out.extend_from_slice(&value.to_be_bytes());
    }
    encode_string(&message.sensor_frame_id, &mut out)?;
    encode_string(&message.body_frame_id, &mut out)?;
    encode_string(&message.odom_frame_id, &mut out)?;
    encode_string(&message.map_frame_id, &mut out)?;
    out.extend_from_slice(&message.lifecycle_state.to_be_bytes());
    out.extend_from_slice(&message.reason_code.to_be_bytes());
    encode_string(&message.reason, &mut out)?;
    Ok(out)
}

/// Decode and validate a complete generated `LioStatus` message.
pub fn decode_lio_status(encoded: &[u8]) -> CodecResult<LioStatus> {
    let mut cursor = Cursor::new(encoded);
    cursor.expect_fingerprint(LIO_STATUS_FINGERPRINT_V1, "nuwax_msgs.LioStatus")?;
    let message = LioStatus {
        schema_version: cursor.read_i32()?,
        backend: cursor.read_i32()?,
        runtime_generation: cursor.read_i64()?,
        status_sequence: cursor.read_i64()?,
        status_timestamp_ns: cursor.read_i64()?,
        last_odometry_source_sequence: cursor.read_i64()?,
        last_odometry_source_timestamp_ns: cursor.read_i64()?,
        last_cloud_source_sequence: cursor.read_i64()?,
        last_cloud_source_timestamp_ns: cursor.read_i64()?,
        sensor_frame_id: cursor.read_string()?,
        body_frame_id: cursor.read_string()?,
        odom_frame_id: cursor.read_string()?,
        map_frame_id: cursor.read_string()?,
        lifecycle_state: cursor.read_i32()?,
        reason_code: cursor.read_i32()?,
        reason: cursor.read_string()?,
    };
    cursor.finish("nuwax_msgs.LioStatus")?;
    validate_status(&message).map_err(decode_error)?;
    Ok(message)
}

fn validate_odometry(message: &LioOdometryEnvelope) -> Result<(), String> {
    validate_common_identity(
        message.schema_version,
        message.backend,
        message.runtime_generation,
        message.source_sequence,
        message.source_timestamp_ns,
    )?;
    validate_frames([
        ("odom_frame_id", &message.odom_frame_id),
        ("body_frame_id", &message.body_frame_id),
        ("map_frame_id", &message.map_frame_id),
    ])?;
    validate_header_identity(
        message.odometry.header.seq,
        message.odometry.header.stamp_sec,
        message.odometry.header.stamp_nsec,
        &message.odometry.header.frame_id,
        message.source_sequence,
        message.source_timestamp_ns,
        &message.odom_frame_id,
    )?;
    if message.odometry.child_frame_id != message.body_frame_id {
        return Err("odometry child_frame_id must equal body_frame_id".into());
    }
    if message
        .odometry
        .pose
        .position
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err("odometry pose position must contain only finite values".into());
    }
    let orientation = &message.odometry.pose.orientation_xyzw;
    if orientation.iter().any(|value| !value.is_finite()) {
        return Err("odometry quaternion must contain only finite values".into());
    }
    let quaternion_norm = orientation
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if !quaternion_norm.is_finite()
        || quaternion_norm <= f64::EPSILON
        || (quaternion_norm - 1.0).abs() > ODOMETRY_QUATERNION_NORM_TOLERANCE
    {
        return Err(
            "odometry quaternion must be nonzero and unit length within tolerance 1e-3".into(),
        );
    }
    if message
        .odometry
        .twist_linear
        .iter()
        .chain(message.odometry.twist_angular.iter())
        .any(|value| !value.is_finite())
    {
        return Err("odometry twist must contain only finite values".into());
    }
    if message
        .odometry
        .pose_covariance
        .iter()
        .chain(message.odometry.twist_covariance.iter())
        .any(|value| !value.is_finite())
    {
        return Err("odometry covariance must contain only finite values".into());
    }
    Ok(())
}

fn validate_cloud(message: &LioCloudEnvelope) -> Result<(), String> {
    validate_common_identity(
        message.schema_version,
        message.backend,
        message.runtime_generation,
        message.source_sequence,
        message.source_timestamp_ns,
    )?;
    if message.correlated_odometry_source_sequence <= 0 {
        return Err("correlated odometry source sequence must be positive".into());
    }
    if message.correlated_odometry_source_timestamp_ns < 0 {
        return Err("correlated odometry source timestamp must be non-negative".into());
    }
    validate_frames([
        ("sensor_frame_id", &message.sensor_frame_id),
        ("body_frame_id", &message.body_frame_id),
        ("odom_frame_id", &message.odom_frame_id),
        ("map_frame_id", &message.map_frame_id),
    ])?;
    if message.registration_state != LIO_REGISTRATION_REGISTERED_MAP {
        return Err(format!(
            "public LIO cloud registration_state must be REGISTERED_MAP, got {}",
            message.registration_state
        ));
    }
    if message.emitted_point_count <= 0 || message.emitted_point_count > MAX_LIO_EMITTED_POINT_COUNT
    {
        return Err(format!(
            "emitted_point_count must be in 1..={MAX_LIO_EMITTED_POINT_COUNT}"
        ));
    }
    if message.source_point_count < message.emitted_point_count
        || message.source_point_count > MAX_LIO_SOURCE_POINT_COUNT
    {
        return Err(format!(
            "source_point_count must be emitted_point_count..={MAX_LIO_SOURCE_POINT_COUNT}"
        ));
    }
    validate_header_identity(
        message.cloud.header.seq,
        message.cloud.header.stamp_sec,
        message.cloud.header.stamp_nsec,
        &message.cloud.header.frame_id,
        message.source_sequence,
        message.source_timestamp_ns,
        &message.map_frame_id,
    )?;
    validate_cloud_layout(&message.cloud, message.emitted_point_count)
}

fn validate_status(message: &LioStatus) -> Result<(), String> {
    if message.schema_version != LIO_SCHEMA_VERSION_V1 {
        return Err(format!(
            "unsupported LIO schema_version={}",
            message.schema_version
        ));
    }
    validate_backend(message.backend)?;
    if message.runtime_generation <= 0 {
        return Err("runtime_generation must be positive".into());
    }
    if message.status_sequence <= 0 {
        return Err("status_sequence must be positive".into());
    }
    if message.status_timestamp_ns < 0 {
        return Err("status_timestamp_ns must be non-negative".into());
    }
    validate_optional_stream_pair(
        "last_odometry",
        message.last_odometry_source_sequence,
        message.last_odometry_source_timestamp_ns,
    )?;
    validate_optional_stream_pair(
        "last_cloud",
        message.last_cloud_source_sequence,
        message.last_cloud_source_timestamp_ns,
    )?;
    validate_frames([
        ("sensor_frame_id", &message.sensor_frame_id),
        ("body_frame_id", &message.body_frame_id),
        ("odom_frame_id", &message.odom_frame_id),
        ("map_frame_id", &message.map_frame_id),
    ])?;
    if !matches!(
        message.lifecycle_state,
        LIO_LIFECYCLE_STARTING
            | LIO_LIFECYCLE_READY
            | LIO_LIFECYCLE_STOPPING
            | LIO_LIFECYCLE_STOPPED
            | LIO_LIFECYCLE_FAILED
    ) {
        return Err(format!(
            "unknown LIO lifecycle_state={}",
            message.lifecycle_state
        ));
    }
    if !matches!(
        message.reason_code,
        LIO_REASON_NONE
            | LIO_REASON_STARTUP_TIMEOUT
            | LIO_REASON_CHILD_EXITED
            | LIO_REASON_CHILD_CRASHED
            | LIO_REASON_SHUTDOWN_REQUESTED
            | LIO_REASON_PREFLIGHT_FAILED
            | LIO_REASON_TRANSPORT_ERROR
    ) {
        return Err(format!("unknown LIO reason_code={}", message.reason_code));
    }
    if message.reason.len() > MAX_REASON_BYTES || message.reason.contains('\0') {
        return Err(format!(
            "reason must not contain NUL or exceed {MAX_REASON_BYTES} bytes"
        ));
    }
    let reason_present = !message.reason.trim().is_empty();
    let valid_lifecycle_reason = match message.lifecycle_state {
        LIO_LIFECYCLE_STARTING | LIO_LIFECYCLE_READY => {
            message.reason_code == LIO_REASON_NONE && message.reason.is_empty()
        }
        LIO_LIFECYCLE_STOPPING => {
            message.reason_code == LIO_REASON_SHUTDOWN_REQUESTED && reason_present
        }
        LIO_LIFECYCLE_STOPPED => {
            matches!(
                message.reason_code,
                LIO_REASON_SHUTDOWN_REQUESTED | LIO_REASON_CHILD_EXITED
            ) && reason_present
        }
        LIO_LIFECYCLE_FAILED => {
            matches!(
                message.reason_code,
                LIO_REASON_STARTUP_TIMEOUT
                    | LIO_REASON_CHILD_EXITED
                    | LIO_REASON_CHILD_CRASHED
                    | LIO_REASON_PREFLIGHT_FAILED
                    | LIO_REASON_TRANSPORT_ERROR
            ) && reason_present
        }
        _ => false,
    };
    if !valid_lifecycle_reason {
        return Err(format!(
            "invalid LIO lifecycle/reason combination: lifecycle_state={}, reason_code={}, reason_present={reason_present}",
            message.lifecycle_state, message.reason_code
        ));
    }
    Ok(())
}

fn validate_common_identity(
    schema_version: i32,
    backend: i32,
    runtime_generation: i64,
    source_sequence: i64,
    source_timestamp_ns: i64,
) -> Result<(), String> {
    if schema_version != LIO_SCHEMA_VERSION_V1 {
        return Err(format!("unsupported LIO schema_version={schema_version}"));
    }
    validate_backend(backend)?;
    if runtime_generation <= 0 {
        return Err("runtime_generation must be positive".into());
    }
    if source_sequence <= 0 {
        return Err("source_sequence must be positive".into());
    }
    if source_timestamp_ns < 0 {
        return Err("source_timestamp_ns must be non-negative".into());
    }
    Ok(())
}

fn validate_backend(backend: i32) -> Result<(), String> {
    if matches!(
        backend,
        LIO_BACKEND_DEVICE | LIO_BACKEND_FASTLIO2 | LIO_BACKEND_POINTLIO
    ) {
        Ok(())
    } else {
        Err(format!("unknown LIO backend={backend}"))
    }
}

fn validate_optional_stream_pair(
    name: &str,
    sequence: i64,
    timestamp_ns: i64,
) -> Result<(), String> {
    if sequence == 0 && timestamp_ns == 0 {
        return Ok(());
    }
    if sequence <= 0 || timestamp_ns < 0 {
        return Err(format!(
            "{name} stream identity must be absent as (0,0) or present with sequence>0 and timestamp>=0"
        ));
    }
    Ok(())
}

fn validate_frames<'a, const N: usize>(frames: [(&'a str, &'a str); N]) -> Result<(), String> {
    for (name, value) in frames {
        if !valid_frame_id(value) {
            return Err(format!(
                "{name} must use slash-separated ASCII frame tokens and be at most {MAX_FRAME_ID_BYTES} bytes"
            ));
        }
    }
    Ok(())
}

fn valid_frame_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_FRAME_ID_BYTES
        && value.trim() == value
        && value.split('/').all(|segment| {
            let mut characters = segment.chars();
            characters
                .next()
                .is_some_and(|first| first.is_ascii_alphabetic())
                && characters.all(|character| character.is_ascii_alphanumeric() || character == '_')
        })
}

fn validate_header_identity(
    header_sequence: i32,
    stamp_sec: i32,
    stamp_nsec: i32,
    header_frame_id: &str,
    source_sequence: i64,
    source_timestamp_ns: i64,
    envelope_frame_id: &str,
) -> Result<(), String> {
    if i64::from(header_sequence) != source_sequence {
        return Err("nested header sequence must equal envelope source_sequence".into());
    }
    let nested_timestamp_ns = header_timestamp_ns(stamp_sec, stamp_nsec)?;
    if nested_timestamp_ns != source_timestamp_ns {
        return Err("nested header timestamp must equal envelope source_timestamp_ns".into());
    }
    if header_frame_id != envelope_frame_id {
        return Err("nested header frame_id must equal the authoritative envelope frame".into());
    }
    Ok(())
}

fn header_timestamp_ns(stamp_sec: i32, stamp_nsec: i32) -> Result<i64, String> {
    if stamp_sec < 0 || !(0..NANOS_PER_SECOND as i32).contains(&stamp_nsec) {
        return Err("nested header timestamp must be normalized and non-negative".into());
    }
    i64::from(stamp_sec)
        .checked_mul(NANOS_PER_SECOND)
        .and_then(|value| value.checked_add(i64::from(stamp_nsec)))
        .ok_or_else(|| "nested header timestamp overflow".into())
}

fn validate_cloud_layout(cloud: &PointCloud2, emitted_point_count: i64) -> Result<(), String> {
    if cloud.height <= 0 || cloud.width <= 0 || cloud.point_step <= 0 || cloud.row_step <= 0 {
        return Err("PointCloud2 height, width, point_step, and row_step must be positive".into());
    }
    let layout_points = i64::from(cloud.height)
        .checked_mul(i64::from(cloud.width))
        .ok_or_else(|| "PointCloud2 height*width overflow".to_string())?;
    if layout_points != emitted_point_count {
        return Err("emitted_point_count must equal PointCloud2 height*width".into());
    }
    let expected_row_step = cloud
        .width
        .checked_mul(cloud.point_step)
        .ok_or_else(|| "PointCloud2 row_step overflow".to_string())?;
    if cloud.row_step != expected_row_step {
        return Err("PointCloud2 row_step must equal width*point_step".into());
    }
    let expected_data_len = usize::try_from(cloud.height)
        .ok()
        .and_then(|height| {
            usize::try_from(cloud.row_step)
                .ok()
                .and_then(|row_step| height.checked_mul(row_step))
        })
        .ok_or_else(|| "PointCloud2 data length overflow".to_string())?;
    if cloud.data.len() != expected_data_len {
        return Err("PointCloud2 data length must equal height*row_step".into());
    }
    if cloud.fields.is_empty() {
        return Err("PointCloud2 must declare at least one field".into());
    }
    for field in &cloud.fields {
        let scalar_bytes = match field.datatype {
            1 | 2 => 1_i32,
            3 | 4 => 2,
            5..=7 => 4,
            8 => 8,
            other => return Err(format!("unknown PointField datatype={other}")),
        };
        if field.name.trim().is_empty() || field.name.contains('\0') {
            return Err("PointField name must be nonempty and NUL-free".into());
        }
        if field.offset < 0 || field.count <= 0 {
            return Err("PointField offset must be non-negative and count positive".into());
        }
        let end = field
            .count
            .checked_mul(scalar_bytes)
            .and_then(|length| field.offset.checked_add(length))
            .ok_or_else(|| "PointField layout overflow".to_string())?;
        if end > cloud.point_step {
            return Err("PointField layout exceeds PointCloud2 point_step".into());
        }
    }
    Ok(())
}

fn encode_common_identity(
    schema_version: i32,
    backend: i32,
    runtime_generation: i64,
    source_sequence: i64,
    source_timestamp_ns: i64,
    out: &mut Vec<u8>,
) {
    out.extend_from_slice(&schema_version.to_be_bytes());
    out.extend_from_slice(&backend.to_be_bytes());
    out.extend_from_slice(&runtime_generation.to_be_bytes());
    out.extend_from_slice(&source_sequence.to_be_bytes());
    out.extend_from_slice(&source_timestamp_ns.to_be_bytes());
}

fn decode_common_identity(cursor: &mut Cursor<'_>) -> CodecResult<(i32, i32, i64, i64, i64)> {
    Ok((
        cursor.read_i32()?,
        cursor.read_i32()?,
        cursor.read_i64()?,
        cursor.read_i64()?,
        cursor.read_i64()?,
    ))
}

fn decode_nested_odometry(body: &[u8]) -> CodecResult<Odometry> {
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&ODOMETRY_FINGERPRINT);
    encoded.extend_from_slice(body);
    decode_odometry(&encoded)
}

fn decode_nested_cloud(body: &[u8]) -> CodecResult<PointCloud2> {
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&POINT_CLOUD2_FINGERPRINT);
    encoded.extend_from_slice(body);
    decode_point_cloud2(&encoded)
}

fn encode_string(value: &str, out: &mut Vec<u8>) -> CodecResult<()> {
    let encoded_len = value
        .len()
        .checked_add(1)
        .and_then(|length| u32::try_from(length).ok())
        .ok_or_else(|| CodecError::LcmEncode("LCM string exceeds uint32 length".into()))?;
    out.extend_from_slice(&encoded_len.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    out.push(0);
    Ok(())
}

fn encode_error(message: String) -> CodecError {
    CodecError::LcmEncode(message)
}

fn decode_error(message: String) -> CodecError {
    CodecError::LcmDecode(message)
}

struct Cursor<'a> {
    input: &'a [u8],
    offset: usize,
}

impl<'a> Cursor<'a> {
    fn new(input: &'a [u8]) -> Self {
        Self { input, offset: 0 }
    }

    fn read_exact(&mut self, length: usize) -> CodecResult<&'a [u8]> {
        let end = self
            .offset
            .checked_add(length)
            .ok_or(CodecError::Incomplete {
                need: usize::MAX,
                got: self.input.len(),
            })?;
        if end > self.input.len() {
            return Err(CodecError::Incomplete {
                need: end,
                got: self.input.len(),
            });
        }
        let bytes = &self.input[self.offset..end];
        self.offset = end;
        Ok(bytes)
    }

    fn read_i32(&mut self) -> CodecResult<i32> {
        Ok(i32::from_be_bytes(
            self.read_exact(4)?.try_into().expect("four bytes"),
        ))
    }

    fn read_i64(&mut self) -> CodecResult<i64> {
        Ok(i64::from_be_bytes(
            self.read_exact(8)?.try_into().expect("eight bytes"),
        ))
    }

    fn read_string(&mut self) -> CodecResult<String> {
        let length = usize::try_from(u32::from_be_bytes(
            self.read_exact(4)?.try_into().expect("four bytes"),
        ))
        .expect("u32 fits usize");
        if length == 0 {
            return Err(CodecError::LcmDecode(
                "LCM string length must include a trailing NUL".into(),
            ));
        }
        let bytes = self.read_exact(length)?;
        if bytes.last() != Some(&0) {
            return Err(CodecError::LcmDecode(
                "LCM string is missing its trailing NUL".into(),
            ));
        }
        String::from_utf8(bytes[..length - 1].to_vec())
            .map_err(|_| CodecError::LcmDecode("LCM string is not valid UTF-8".into()))
    }

    fn expect_fingerprint(&mut self, expected: [u8; 8], message_type: &str) -> CodecResult<()> {
        if self.read_exact(8)? != expected {
            return Err(CodecError::LcmDecode(format!(
                "fingerprint mismatch for {message_type}"
            )));
        }
        Ok(())
    }

    fn remaining_slice(&self) -> &'a [u8] {
        &self.input[self.offset..]
    }

    fn finish(&self, message_type: &str) -> CodecResult<()> {
        if self.offset == self.input.len() {
            Ok(())
        } else {
            Err(CodecError::LcmDecode(format!(
                "trailing bytes after {message_type}: {}",
                self.input.len() - self.offset
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::schema::{Header, PointField, Pose};
    use super::*;

    fn header(seq: i32, timestamp_ns: i64, frame_id: &str) -> Header {
        Header {
            seq,
            stamp_sec: i32::try_from(timestamp_ns / NANOS_PER_SECOND).unwrap(),
            stamp_nsec: i32::try_from(timestamp_ns % NANOS_PER_SECOND).unwrap(),
            frame_id: frame_id.into(),
        }
    }

    fn odometry() -> LioOdometryEnvelope {
        LioOdometryEnvelope {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: LIO_BACKEND_FASTLIO2,
            runtime_generation: 7,
            source_sequence: 11,
            source_timestamp_ns: 2_000_000_003,
            odom_frame_id: "odom".into(),
            body_frame_id: "base_link".into(),
            map_frame_id: "map".into(),
            odometry: Odometry {
                header: header(11, 2_000_000_003, "odom"),
                child_frame_id: "base_link".into(),
                pose: Pose {
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    ..Pose::default()
                },
                ..Odometry::default()
            },
        }
    }

    fn cloud() -> LioCloudEnvelope {
        LioCloudEnvelope {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: LIO_BACKEND_POINTLIO,
            runtime_generation: 8,
            source_sequence: 12,
            source_timestamp_ns: 4_000_000_005,
            correlated_odometry_source_sequence: 11,
            correlated_odometry_source_timestamp_ns: 3_000_000_004,
            sensor_frame_id: "lidar".into(),
            body_frame_id: "base_link".into(),
            odom_frame_id: "odom".into(),
            map_frame_id: "map".into(),
            registration_state: LIO_REGISTRATION_REGISTERED_MAP,
            source_point_count: 3,
            emitted_point_count: 2,
            cloud: PointCloud2 {
                header: header(12, 4_000_000_005, "map"),
                height: 1,
                width: 2,
                fields: vec![PointField {
                    name: "x".into(),
                    offset: 0,
                    datatype: 7,
                    count: 1,
                }],
                point_step: 4,
                row_step: 8,
                data: vec![0; 8],
                is_dense: true,
                ..PointCloud2::default()
            },
        }
    }

    fn status() -> LioStatus {
        LioStatus {
            schema_version: LIO_SCHEMA_VERSION_V1,
            backend: LIO_BACKEND_DEVICE,
            runtime_generation: 9,
            status_sequence: 1,
            status_timestamp_ns: 0,
            last_odometry_source_sequence: 1,
            last_odometry_source_timestamp_ns: 0,
            last_cloud_source_sequence: 0,
            last_cloud_source_timestamp_ns: 0,
            sensor_frame_id: "lidar".into(),
            body_frame_id: "base_link".into(),
            odom_frame_id: "odom".into(),
            map_frame_id: "map".into(),
            lifecycle_state: LIO_LIFECYCLE_READY,
            reason_code: LIO_REASON_NONE,
            reason: String::new(),
        }
    }

    #[test]
    fn valid_messages_round_trip() {
        let odometry = odometry();
        assert_eq!(
            decode_lio_odometry_envelope(&encode_lio_odometry_envelope(&odometry).unwrap())
                .unwrap(),
            odometry
        );
        let cloud = cloud();
        assert_eq!(
            decode_lio_cloud_envelope(&encode_lio_cloud_envelope(&cloud).unwrap()).unwrap(),
            cloud
        );
        let status = status();
        assert_eq!(
            decode_lio_status(&encode_lio_status(&status).unwrap()).unwrap(),
            status
        );
    }

    #[test]
    fn encoders_match_generated_c_and_python_byte_goldens() {
        let odometry_bytes = encode_lio_odometry_envelope(&odometry()).unwrap();
        let odometry_golden = format!(
            "{}{}{}{}",
            concat!(
                "742d9b06c0c7a22300000001000000020000000000000007000000000000000b0000000077359403",
                "000000056f646f6d000000000a626173655f6c696e6b00000000046d6170000000000b0000000200000003",
                "000000056f646f6d000000000a626173655f6c696e6b00"
            ),
            "00".repeat(48),
            "3ff0000000000000",
            "00".repeat(624)
        );
        assert_eq!(odometry_bytes, decode_hex(&odometry_golden));

        assert_eq!(
            encode_lio_cloud_envelope(&cloud()).unwrap(),
            decode_hex(concat!(
                "dcd4ae0f83a9a81b00000001000000030000000000000008000000000000000c00000000ee6b2805",
                "000000000000000b00000000b2d05e04000000066c69646172000000000a626173655f6c696e6b00",
                "000000056f646f6d00000000046d617000000000020000000000000003000000000000000200000001",
                "000000080000000c0000000400000005000000046d6170000000000100000002000000027800000000",
                "000700000001000000000400000008000000000000000001"
            ))
        );

        assert_eq!(
            encode_lio_status(&status()).unwrap(),
            decode_hex(concat!(
                "6d7ae636d8ba41b20000000100000001000000000000000900000000000000010000000000000000",
                "0000000000000001000000000000000000000000000000000000000000000000000000066c696461",
                "72000000000a626173655f6c696e6b00000000056f646f6d00000000046d6170000000000200000000",
                "0000000100"
            ))
        );
    }

    #[test]
    fn decoders_reject_unknown_identity_and_raw_public_cloud() {
        let mut odometry_bytes = encode_lio_odometry_envelope(&odometry()).unwrap();
        overwrite_i32(&mut odometry_bytes, 8, 2);
        assert!(decode_lio_odometry_envelope(&odometry_bytes).is_err());
        odometry_bytes = encode_lio_odometry_envelope(&odometry()).unwrap();
        overwrite_i32(&mut odometry_bytes, 12, 99);
        assert!(decode_lio_odometry_envelope(&odometry_bytes).is_err());

        let cloud = cloud();
        let mut cloud_bytes = encode_lio_cloud_envelope(&cloud).unwrap();
        overwrite_i32(&mut cloud_bytes, 8, 2);
        assert!(decode_lio_cloud_envelope(&cloud_bytes).is_err());
        cloud_bytes = encode_lio_cloud_envelope(&cloud).unwrap();
        overwrite_i32(&mut cloud_bytes, 12, 99);
        assert!(decode_lio_cloud_envelope(&cloud_bytes).is_err());
        cloud_bytes = encode_lio_cloud_envelope(&cloud).unwrap();
        let registration_offset = 8
            + 4
            + 4
            + 8
            + 8
            + 8
            + 8
            + 8
            + [
                &cloud.sensor_frame_id,
                &cloud.body_frame_id,
                &cloud.odom_frame_id,
                &cloud.map_frame_id,
            ]
            .into_iter()
            .map(|value| 4 + value.len() + 1)
            .sum::<usize>();
        overwrite_i32(
            &mut cloud_bytes,
            registration_offset,
            LIO_REGISTRATION_RAW_SENSOR,
        );
        assert!(decode_lio_cloud_envelope(&cloud_bytes).is_err());

        let mut status_bytes = encode_lio_status(&status()).unwrap();
        overwrite_i32(&mut status_bytes, 8, 2);
        assert!(decode_lio_status(&status_bytes).is_err());
        status_bytes = encode_lio_status(&status()).unwrap();
        overwrite_i32(&mut status_bytes, 12, 99);
        assert!(decode_lio_status(&status_bytes).is_err());
    }

    fn overwrite_i32(bytes: &mut [u8], offset: usize, value: i32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn decode_hex(value: &str) -> Vec<u8> {
        value
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                u8::from_str_radix(std::str::from_utf8(pair).expect("ASCII hex"), 16)
                    .expect("valid hex")
            })
            .collect()
    }

    #[test]
    fn identity_and_nested_authority_are_fail_closed() {
        let mut message = odometry();
        message.schema_version = 2;
        assert!(encode_lio_odometry_envelope(&message).is_err());
        message = odometry();
        message.backend = 99;
        assert!(encode_lio_odometry_envelope(&message).is_err());
        message = odometry();
        message.odometry.header.seq += 1;
        assert!(encode_lio_odometry_envelope(&message).is_err());
        message = odometry();
        message.odometry.child_frame_id = "other".into();
        assert!(encode_lio_odometry_envelope(&message).is_err());
        for invalid_frame in [" map ", "/map", "map/", "map//robot", "9map", "map-drift"] {
            message = odometry();
            message.map_frame_id = invalid_frame.into();
            assert!(encode_lio_odometry_envelope(&message).is_err());
        }
    }

    fn encode_unchecked_odometry_for_decode_test(message: &LioOdometryEnvelope) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&LIO_ODOMETRY_ENVELOPE_FINGERPRINT_V1);
        encode_common_identity(
            message.schema_version,
            message.backend,
            message.runtime_generation,
            message.source_sequence,
            message.source_timestamp_ns,
            &mut out,
        );
        encode_string(&message.odom_frame_id, &mut out).expect("test odom frame");
        encode_string(&message.body_frame_id, &mut out).expect("test body frame");
        encode_string(&message.map_frame_id, &mut out).expect("test map frame");
        out.extend_from_slice(
            &encode_odometry_body(&message.odometry).expect("unchecked test odometry body"),
        );
        out
    }

    fn assert_odometry_validation_error(message: LioOdometryEnvelope, expected: &str) {
        match encode_lio_odometry_envelope(&message).expect_err("invalid odometry must not encode")
        {
            CodecError::LcmEncode(actual) => assert_eq!(actual, expected),
            other => panic!("unexpected encode error: {other}"),
        }
        let unchecked = encode_unchecked_odometry_for_decode_test(&message);
        match decode_lio_odometry_envelope(&unchecked)
            .expect_err("invalid odometry wire body must not decode")
        {
            CodecError::LcmDecode(actual) => assert_eq!(actual, expected),
            other => panic!("unexpected decode error: {other}"),
        }
    }

    #[test]
    fn odometry_numeric_contract_is_fail_closed_on_encode_and_decode() {
        let mut message = odometry();
        message.odometry.pose.position[0] = f64::NAN;
        assert_odometry_validation_error(
            message,
            "odometry pose position must contain only finite values",
        );

        let mut message = odometry();
        message.odometry.pose.orientation_xyzw[1] = f64::INFINITY;
        assert_odometry_validation_error(
            message,
            "odometry quaternion must contain only finite values",
        );

        for orientation in [[0.0; 4], [0.0, 0.0, 0.0, 0.5]] {
            let mut message = odometry();
            message.odometry.pose.orientation_xyzw = orientation;
            assert_odometry_validation_error(
                message,
                "odometry quaternion must be nonzero and unit length within tolerance 1e-3",
            );
        }

        let mut message = odometry();
        message.odometry.twist_linear[2] = f64::NEG_INFINITY;
        assert_odometry_validation_error(message, "odometry twist must contain only finite values");

        let mut message = odometry();
        message.odometry.twist_angular[0] = f64::NAN;
        assert_odometry_validation_error(message, "odometry twist must contain only finite values");

        let mut message = odometry();
        message.odometry.pose_covariance[7] = f64::NAN;
        assert_odometry_validation_error(
            message,
            "odometry covariance must contain only finite values",
        );

        let mut message = odometry();
        message.odometry.twist_covariance[11] = f64::INFINITY;
        assert_odometry_validation_error(
            message,
            "odometry covariance must contain only finite values",
        );
    }

    #[test]
    fn odometry_quaternion_accepts_the_native_contract_tolerance() {
        for norm in [1.0 - 5.0e-4, 1.0, 1.0 + 5.0e-4] {
            let mut message = odometry();
            message.odometry.pose.orientation_xyzw = [0.0, 0.0, 0.0, norm];
            assert!(encode_lio_odometry_envelope(&message).is_ok());
        }
    }

    #[test]
    fn source_timestamp_zero_is_valid_and_present() {
        let mut odometry = odometry();
        odometry.source_timestamp_ns = 0;
        odometry.odometry.header = header(11, 0, "odom");
        assert!(encode_lio_odometry_envelope(&odometry).is_ok());

        let mut cloud = cloud();
        cloud.source_timestamp_ns = 0;
        cloud.cloud.header = header(12, 0, "map");
        cloud.correlated_odometry_source_timestamp_ns = 0;
        assert!(encode_lio_cloud_envelope(&cloud).is_ok());
    }

    #[test]
    fn cloud_rejects_raw_registration_counts_and_layout_drift() {
        let mut message = cloud();
        message.registration_state = LIO_REGISTRATION_RAW_SENSOR;
        assert!(encode_lio_cloud_envelope(&message).is_err());
        message = cloud();
        message.emitted_point_count = MAX_LIO_EMITTED_POINT_COUNT + 1;
        assert!(encode_lio_cloud_envelope(&message).is_err());
        message = cloud();
        message.source_point_count = MAX_LIO_SOURCE_POINT_COUNT + 1;
        assert!(encode_lio_cloud_envelope(&message).is_err());
        message = cloud();
        message.cloud.row_step += 1;
        assert!(encode_lio_cloud_envelope(&message).is_err());
        message = cloud();
        message.cloud.header.frame_id = "odom".into();
        assert!(encode_lio_cloud_envelope(&message).is_err());
    }

    #[test]
    fn status_accepts_present_zero_timestamp_and_rejects_invalid_pairs() {
        assert!(encode_lio_status(&status()).is_ok());
        let mut message = status();
        message.last_cloud_source_timestamp_ns = 1;
        assert!(encode_lio_status(&message).is_err());
        message = status();
        message.lifecycle_state = 99;
        assert!(encode_lio_status(&message).is_err());
        message = status();
        message.reason_code = 99;
        assert!(encode_lio_status(&message).is_err());
        message = status();
        message.lifecycle_state = LIO_LIFECYCLE_FAILED;
        assert!(encode_lio_status(&message).is_err());
        message = status();
        message.reason_code = LIO_REASON_CHILD_CRASHED;
        message.reason = "crashed".into();
        assert!(encode_lio_status(&message).is_err());
        message.lifecycle_state = LIO_LIFECYCLE_FAILED;
        assert!(encode_lio_status(&message).is_ok());
    }

    #[test]
    fn lifecycle_reason_matrix_is_exhaustive() {
        for lifecycle in 0..=6 {
            for reason_code in 0..=7 {
                for reason in ["", "detail"] {
                    let mut message = status();
                    message.lifecycle_state = lifecycle;
                    message.reason_code = reason_code;
                    message.reason = reason.into();
                    let expected = match lifecycle {
                        LIO_LIFECYCLE_STARTING | LIO_LIFECYCLE_READY => {
                            reason_code == LIO_REASON_NONE && reason.is_empty()
                        }
                        LIO_LIFECYCLE_STOPPING => {
                            reason_code == LIO_REASON_SHUTDOWN_REQUESTED && !reason.is_empty()
                        }
                        LIO_LIFECYCLE_STOPPED => {
                            matches!(
                                reason_code,
                                LIO_REASON_SHUTDOWN_REQUESTED | LIO_REASON_CHILD_EXITED
                            ) && !reason.is_empty()
                        }
                        LIO_LIFECYCLE_FAILED => {
                            matches!(
                                reason_code,
                                LIO_REASON_STARTUP_TIMEOUT
                                    | LIO_REASON_CHILD_EXITED
                                    | LIO_REASON_CHILD_CRASHED
                                    | LIO_REASON_PREFLIGHT_FAILED
                                    | LIO_REASON_TRANSPORT_ERROR
                            ) && !reason.is_empty()
                        }
                        _ => false,
                    };
                    assert_eq!(
                        encode_lio_status(&message).is_ok(),
                        expected,
                        "lifecycle={lifecycle} reason_code={reason_code} reason={reason:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn decoders_reject_wrong_fingerprint_truncation_and_trailing_bytes() {
        let mut encoded = encode_lio_status(&status()).unwrap();
        encoded[0] ^= 1;
        assert!(decode_lio_status(&encoded).is_err());

        let mut encoded = encode_lio_cloud_envelope(&cloud()).unwrap();
        encoded.pop();
        assert!(decode_lio_cloud_envelope(&encoded).is_err());

        let mut encoded = encode_lio_odometry_envelope(&odometry()).unwrap();
        encoded.push(0);
        assert!(decode_lio_odometry_envelope(&encoded).is_err());
    }
}
