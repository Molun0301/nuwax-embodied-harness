//! 规范传感器输入的独立 v1 小端线协议。
//!
//! 本模块不改变既有 LIO envelope，也不加载 SHM 或制品载荷；它只负责把
//! Python facade 的 canonical cloud/IMU 身份和载荷描述编码为可验证的字节帧。

use crate::{CodecError, CodecResult};

pub const SENSOR_WIRE_MAGIC: [u8; 4] = *b"NWS1";
pub const SENSOR_WIRE_VERSION: u8 = 1;
pub const SENSOR_WIRE_HEADER_BYTES: usize = 12;
pub const SENSOR_WIRE_MAX_FRAME_BYTES: usize = 8 * 1024 * 1024;
pub const SENSOR_WIRE_MAX_PAYLOAD_BYTES: usize = 4 * 1024 * 1024;
pub const SENSOR_WIRE_MAX_STRING_BYTES: usize = 255;
pub const SENSOR_WIRE_MAX_LAYOUT_FIELDS: usize = 8;

/// 仅用于文档/配置引用；不表示已经实现 Livox 原始帧解码。
pub const LIVOX_REFERENCE_ONLY_MANIFEST: &str = "livox-custom-msg-reference-only-v1";

const KIND_CLOUD: u8 = 1;
const KIND_IMU: u8 = 2;
const OWNERSHIP_INLINE: u8 = 0;
const OWNERSHIP_SHM: u8 = 1;
const OWNERSHIP_ARTIFACT: u8 = 2;
const CALIBRATION_ABSENT: u8 = 0;
const CALIBRATION_PRESENT: u8 = 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PayloadOwnership {
    Inline,
    Shm,
    Artifact,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SensorIdentity {
    pub schema_version: u16,
    pub sensor_id: String,
    pub source_sequence: u64,
    pub timestamp_ns: u64,
    pub clock_domain: String,
    pub frame_id: String,
    pub registration_state: String,
    pub registration_frame_id: String,
    pub calibration_ref: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CloudPayload {
    Inline {
        xyz_f32_le: Vec<u8>,
        intensity_f32_le: Option<Vec<u8>>,
        ring_u16_le: Option<Vec<u8>>,
        time_offset_ns_le: Option<Vec<u8>>,
    },
    External {
        ownership: PayloadOwnership,
        payload_ref: String,
        payload_layout: Vec<String>,
        payload_size_bytes: u64,
        lease_epoch: u64,
        expiry_ns: u64,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanonicalCloudFrame {
    pub identity: SensorIdentity,
    pub point_count: u32,
    pub scan_duration_ns: u64,
    pub payload: CloudPayload,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanonicalImuFrame {
    pub identity: SensorIdentity,
    pub accel_mps2: [f64; 3],
    pub gyro_rps: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub enum SensorFrame {
    Cloud(CanonicalCloudFrame),
    Imu(CanonicalImuFrame),
}

#[derive(Debug, Clone, Default)]
pub struct SensorSequenceValidator {
    last_sequence: Option<u64>,
    last_timestamp_ns: Option<u64>,
}

impl SensorSequenceValidator {
    pub fn validate(&mut self, frame: &SensorFrame) -> CodecResult<()> {
        let identity = match frame {
            SensorFrame::Cloud(frame) => &frame.identity,
            SensorFrame::Imu(frame) => &frame.identity,
        };
        if identity.source_sequence == 0 {
            return decode_error("source_sequence must be positive");
        }
        if let Some(previous) = self.last_sequence {
            if identity.source_sequence <= previous {
                return decode_error("source_sequence regressed");
            }
        }
        if let Some(previous) = self.last_timestamp_ns {
            if identity.timestamp_ns < previous {
                return decode_error("timestamp_ns regressed");
            }
        }
        self.last_sequence = Some(identity.source_sequence);
        self.last_timestamp_ns = Some(identity.timestamp_ns);
        Ok(())
    }
}

pub fn encode(frame: &SensorFrame) -> CodecResult<Vec<u8>> {
    let mut body = Writer::default();
    match frame {
        SensorFrame::Cloud(cloud) => {
            validate_identity(&cloud.identity)?;
            validate_cloud(cloud)?;
            encode_identity(&mut body, &cloud.identity)?;
            body.u32(cloud.point_count);
            body.u64(cloud.scan_duration_ns);
            encode_cloud_payload(&mut body, cloud)?;
        }
        SensorFrame::Imu(imu) => {
            validate_identity(&imu.identity)?;
            validate_imu(imu)?;
            encode_identity(&mut body, &imu.identity)?;
            for value in imu.accel_mps2.iter().chain(imu.gyro_rps.iter()) {
                body.f64(*value);
            }
        }
    }
    if body.bytes.len() > SENSOR_WIRE_MAX_FRAME_BYTES - SENSOR_WIRE_HEADER_BYTES {
        return encode_error("sensor wire body exceeds maximum frame size");
    }
    let kind = match frame {
        SensorFrame::Cloud(_) => KIND_CLOUD,
        SensorFrame::Imu(_) => KIND_IMU,
    };
    let body_len = u32::try_from(body.bytes.len())
        .map_err(|_| CodecError::LcmEncode("sensor wire body length overflow".into()))?;
    let mut out = Vec::with_capacity(SENSOR_WIRE_HEADER_BYTES + body.bytes.len());
    out.extend_from_slice(&SENSOR_WIRE_MAGIC);
    out.push(SENSOR_WIRE_VERSION);
    out.push(kind);
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&body_len.to_le_bytes());
    out.extend_from_slice(&body.bytes);
    Ok(out)
}

pub fn decode(bytes: &[u8]) -> CodecResult<SensorFrame> {
    if bytes.len() < SENSOR_WIRE_HEADER_BYTES {
        return decode_error("sensor wire frame is truncated");
    }
    if bytes.len() > SENSOR_WIRE_MAX_FRAME_BYTES {
        return decode_error("sensor wire frame exceeds maximum size");
    }
    if bytes[0..4] != SENSOR_WIRE_MAGIC {
        return decode_error("sensor wire magic mismatch");
    }
    if bytes[4] != SENSOR_WIRE_VERSION {
        return decode_error("unsupported sensor wire version");
    }
    if u16::from_le_bytes([bytes[6], bytes[7]]) != 0 {
        return decode_error("sensor wire flags are not zero");
    }
    let body_len = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize;
    if body_len != bytes.len() - SENSOR_WIRE_HEADER_BYTES {
        return decode_error("sensor wire body length mismatch or trailing bytes");
    }
    let mut reader = Reader::new(&bytes[SENSOR_WIRE_HEADER_BYTES..]);
    let frame = match bytes[5] {
        KIND_CLOUD => {
            let identity = decode_identity(&mut reader)?;
            let point_count = reader.u32()?;
            let scan_duration_ns = reader.u64()?;
            let payload = decode_cloud_payload(&mut reader, point_count, scan_duration_ns)?;
            SensorFrame::Cloud(CanonicalCloudFrame {
                identity,
                point_count,
                scan_duration_ns,
                payload,
            })
        }
        KIND_IMU => {
            let identity = decode_identity(&mut reader)?;
            let mut values = [0.0_f64; 6];
            for value in &mut values {
                *value = reader.f64()?;
                if !value.is_finite() {
                    return decode_error("IMU vector contains NaN or infinity");
                }
            }
            SensorFrame::Imu(CanonicalImuFrame {
                identity,
                accel_mps2: [values[0], values[1], values[2]],
                gyro_rps: [values[3], values[4], values[5]],
            })
        }
        _ => return decode_error("unknown sensor wire frame kind"),
    };
    if !reader.done() {
        return decode_error("sensor wire contains trailing body bytes");
    }
    Ok(frame)
}

fn encode_identity(writer: &mut Writer, identity: &SensorIdentity) -> CodecResult<()> {
    writer.u16(identity.schema_version);
    writer.string(&identity.sensor_id)?;
    writer.u64(identity.source_sequence);
    writer.u64(identity.timestamp_ns);
    writer.string(&identity.clock_domain)?;
    writer.string(&identity.frame_id)?;
    writer.string(&identity.registration_state)?;
    writer.string(&identity.registration_frame_id)?;
    match &identity.calibration_ref {
        Some(value) => {
            writer.u8(CALIBRATION_PRESENT);
            writer.string(value)?;
        }
        None => writer.u8(CALIBRATION_ABSENT),
    }
    Ok(())
}

fn decode_identity(reader: &mut Reader<'_>) -> CodecResult<SensorIdentity> {
    let identity = SensorIdentity {
        schema_version: reader.u16()?,
        sensor_id: reader.string()?,
        source_sequence: reader.u64()?,
        timestamp_ns: reader.u64()?,
        clock_domain: reader.string()?,
        frame_id: reader.string()?,
        registration_state: reader.string()?,
        registration_frame_id: reader.string()?,
        calibration_ref: match reader.u8()? {
            CALIBRATION_ABSENT => None,
            CALIBRATION_PRESENT => Some(reader.string()?),
            _ => return decode_error("invalid calibration presence flag"),
        },
    };
    validate_identity(&identity)?;
    Ok(identity)
}

fn encode_cloud_payload(writer: &mut Writer, cloud: &CanonicalCloudFrame) -> CodecResult<()> {
    match &cloud.payload {
        CloudPayload::Inline {
            xyz_f32_le,
            intensity_f32_le,
            ring_u16_le,
            time_offset_ns_le,
        } => {
            writer.u8(OWNERSHIP_INLINE);
            encode_bytes(writer, xyz_f32_le)?;
            encode_optional_bytes(writer, intensity_f32_le.as_deref())?;
            encode_optional_bytes(writer, ring_u16_le.as_deref())?;
            encode_optional_bytes(writer, time_offset_ns_le.as_deref())?;
        }
        CloudPayload::External {
            ownership,
            payload_ref,
            payload_layout,
            payload_size_bytes,
            lease_epoch,
            expiry_ns,
        } => {
            writer.u8(ownership_code(*ownership));
            writer.string(payload_ref)?;
            if payload_layout.is_empty() || payload_layout.len() > SENSOR_WIRE_MAX_LAYOUT_FIELDS {
                return encode_error("external payload layout is empty or too large");
            }
            writer.u8(payload_layout.len() as u8);
            for field in payload_layout {
                writer.string(field)?;
            }
            writer.u64(*payload_size_bytes);
            writer.u64(*lease_epoch);
            writer.u64(*expiry_ns);
        }
    }
    Ok(())
}

fn decode_cloud_payload(
    reader: &mut Reader<'_>,
    point_count: u32,
    scan_duration_ns: u64,
) -> CodecResult<CloudPayload> {
    let ownership_code = reader.u8()?;
    match ownership_code {
        OWNERSHIP_INLINE => {
            let xyz = reader.bytes()?;
            let intensity = reader.optional_bytes()?;
            let ring = reader.optional_bytes()?;
            let time = reader.optional_bytes()?;
            let payload = CloudPayload::Inline {
                xyz_f32_le: xyz,
                intensity_f32_le: intensity,
                ring_u16_le: ring,
                time_offset_ns_le: time,
            };
            validate_inline_payload(&payload, point_count, scan_duration_ns)?;
            Ok(payload)
        }
        OWNERSHIP_SHM | OWNERSHIP_ARTIFACT => {
            let ownership = ownership_from_code(ownership_code)?;
            let payload_ref = reader.string()?;
            let layout_count = reader.u8()? as usize;
            if layout_count == 0 || layout_count > SENSOR_WIRE_MAX_LAYOUT_FIELDS {
                return decode_error("invalid external payload layout count");
            }
            let mut payload_layout = Vec::with_capacity(layout_count);
            for _ in 0..layout_count {
                payload_layout.push(reader.string()?);
            }
            let payload_size_bytes = reader.u64()?;
            let lease_epoch = reader.u64()?;
            let expiry_ns = reader.u64()?;
            let payload = CloudPayload::External {
                ownership,
                payload_ref,
                payload_layout,
                payload_size_bytes,
                lease_epoch,
                expiry_ns,
            };
            validate_external_payload(&payload, point_count, scan_duration_ns)?;
            Ok(payload)
        }
        _ => decode_error("invalid cloud payload ownership"),
    }
}

fn encode_bytes(writer: &mut Writer, bytes: &[u8]) -> CodecResult<()> {
    if bytes.len() > SENSOR_WIRE_MAX_PAYLOAD_BYTES {
        return encode_error("payload exceeds sensor wire limit");
    }
    let length = u32::try_from(bytes.len())
        .map_err(|_| CodecError::LcmEncode("payload length overflow".into()))?;
    writer.u32(length);
    writer.bytes.extend_from_slice(bytes);
    Ok(())
}

fn encode_optional_bytes(writer: &mut Writer, bytes: Option<&[u8]>) -> CodecResult<()> {
    match bytes {
        Some(bytes) => {
            writer.u8(1);
            encode_bytes(writer, bytes)?;
        }
        None => writer.u8(0),
    }
    Ok(())
}

fn validate_identity(identity: &SensorIdentity) -> CodecResult<()> {
    if identity.schema_version != SENSOR_WIRE_VERSION as u16
        || identity.source_sequence == 0
        || identity.sensor_id.is_empty()
        || identity.clock_domain.is_empty()
        || identity.frame_id.is_empty()
        || identity.registration_state.is_empty()
        || identity.registration_frame_id.is_empty()
    {
        return encode_error("sensor identity contains a missing or zero field");
    }
    for value in [
        &identity.sensor_id,
        &identity.clock_domain,
        &identity.frame_id,
        &identity.registration_state,
        &identity.registration_frame_id,
    ] {
        validate_string(value)?;
    }
    if let Some(value) = &identity.calibration_ref {
        validate_string(value)?;
    }
    Ok(())
}

fn validate_cloud(cloud: &CanonicalCloudFrame) -> CodecResult<()> {
    if cloud.point_count == 0 {
        return encode_error("cloud point_count must be positive");
    }
    match &cloud.payload {
        CloudPayload::Inline { .. } => {
            validate_inline_payload(&cloud.payload, cloud.point_count, cloud.scan_duration_ns)
        }
        CloudPayload::External { .. } => {
            validate_external_payload(&cloud.payload, cloud.point_count, cloud.scan_duration_ns)
        }
    }
}

fn validate_imu(imu: &CanonicalImuFrame) -> CodecResult<()> {
    if imu
        .accel_mps2
        .iter()
        .chain(imu.gyro_rps.iter())
        .any(|v| !v.is_finite())
    {
        return encode_error("IMU vector contains NaN or infinity");
    }
    Ok(())
}

fn validate_inline_payload(
    payload: &CloudPayload,
    point_count: u32,
    scan_duration_ns: u64,
) -> CodecResult<()> {
    let CloudPayload::Inline {
        xyz_f32_le,
        intensity_f32_le,
        ring_u16_le,
        time_offset_ns_le,
    } = payload
    else {
        return encode_error("expected inline payload");
    };
    exact_len(xyz_f32_le, point_count as usize * 12, "xyz")?;
    validate_f32_bytes(xyz_f32_le, "xyz")?;
    validate_optional_len(
        intensity_f32_le.as_deref(),
        point_count as usize * 4,
        "intensity",
    )?;
    if let Some(bytes) = intensity_f32_le {
        validate_f32_bytes(bytes, "intensity")?;
    }
    validate_optional_len(ring_u16_le.as_deref(), point_count as usize * 2, "ring")?;
    validate_optional_len(
        time_offset_ns_le.as_deref(),
        point_count as usize * 8,
        "time_offset_ns",
    )?;
    if let Some(bytes) = time_offset_ns_le {
        let mut max_offset = 0_u64;
        for chunk in bytes.chunks_exact(8) {
            max_offset = max_offset.max(u64::from_le_bytes(chunk.try_into().unwrap()));
        }
        if max_offset > scan_duration_ns {
            return encode_error("point time offset exceeds scan duration");
        }
    }
    Ok(())
}

fn validate_external_payload(
    payload: &CloudPayload,
    point_count: u32,
    scan_duration_ns: u64,
) -> CodecResult<()> {
    let CloudPayload::External {
        ownership,
        payload_ref,
        payload_layout,
        payload_size_bytes,
        lease_epoch,
        expiry_ns,
    } = payload
    else {
        return encode_error("expected external payload");
    };
    let prefix = match ownership {
        PayloadOwnership::Shm => "shm://",
        PayloadOwnership::Artifact => "artifact://",
        PayloadOwnership::Inline => return encode_error("inline ownership cannot be external"),
    };
    if !payload_ref.starts_with(prefix)
        || payload_ref.len() == prefix.len()
        || payload_ref.chars().any(char::is_whitespace)
    {
        return encode_error("external payload reference does not match ownership");
    }
    if payload_layout.first().map(String::as_str) != Some("xyz_f32_le") {
        return encode_error("external payload layout must start with xyz_f32_le");
    }
    if payload_layout.len() > SENSOR_WIRE_MAX_LAYOUT_FIELDS
        || payload_layout.iter().any(|v| v.is_empty())
    {
        return encode_error("invalid external payload layout");
    }
    let mut bytes_per_point = 0_u64;
    let mut seen = std::collections::BTreeSet::new();
    for field in payload_layout {
        if !seen.insert(field.as_str()) {
            return encode_error("external payload layout contains duplicate fields");
        }
        bytes_per_point = bytes_per_point
            .checked_add(match field.as_str() {
                "xyz_f32_le" => 12,
                "intensity_f32_le" => 4,
                "ring_u16_le" => 2,
                "time_offset_ns_le" => 8,
                _ => return encode_error("unknown external payload layout field"),
            })
            .ok_or_else(|| CodecError::LcmEncode("external payload size overflow".into()))?;
    }
    let expected = bytes_per_point
        .checked_mul(point_count as u64)
        .ok_or_else(|| CodecError::LcmEncode("external payload size overflow".into()))?;
    if *payload_size_bytes != expected
        || *payload_size_bytes as usize > SENSOR_WIRE_MAX_PAYLOAD_BYTES
    {
        return encode_error("external payload size mismatch or exceeds limit");
    }
    if *lease_epoch == 0 || *expiry_ns == 0 {
        return encode_error("external payload lease epoch and expiry must be positive");
    }
    if payload_layout.iter().any(|v| v == "time_offset_ns_le") && scan_duration_ns == 0 {
        return encode_error("external time payload requires scan duration");
    }
    Ok(())
}

fn validate_string(value: &str) -> CodecResult<()> {
    if value.is_empty() || value.len() > SENSOR_WIRE_MAX_STRING_BYTES || value.contains('\0') {
        return encode_error("invalid sensor wire string");
    }
    Ok(())
}
fn exact_len(bytes: &[u8], expected: usize, name: &str) -> CodecResult<()> {
    if bytes.len() != expected {
        return encode_error(&format!("{name} payload length mismatch"));
    }
    if bytes.len() > SENSOR_WIRE_MAX_PAYLOAD_BYTES {
        return encode_error("payload exceeds limit");
    }
    Ok(())
}
fn validate_optional_len(bytes: Option<&[u8]>, expected: usize, name: &str) -> CodecResult<()> {
    if let Some(bytes) = bytes {
        exact_len(bytes, expected, name)?;
    }
    Ok(())
}
fn validate_f32_bytes(bytes: &[u8], name: &str) -> CodecResult<()> {
    for chunk in bytes.chunks_exact(4) {
        if !f32::from_le_bytes(chunk.try_into().unwrap()).is_finite() {
            return encode_error(&format!("{name} contains NaN or infinity"));
        }
    }
    Ok(())
}
fn ownership_code(value: PayloadOwnership) -> u8 {
    match value {
        PayloadOwnership::Inline => OWNERSHIP_INLINE,
        PayloadOwnership::Shm => OWNERSHIP_SHM,
        PayloadOwnership::Artifact => OWNERSHIP_ARTIFACT,
    }
}
fn ownership_from_code(value: u8) -> CodecResult<PayloadOwnership> {
    match value {
        OWNERSHIP_SHM => Ok(PayloadOwnership::Shm),
        OWNERSHIP_ARTIFACT => Ok(PayloadOwnership::Artifact),
        _ => decode_error("invalid external ownership"),
    }
}
fn encode_error<T>(message: &str) -> CodecResult<T> {
    Err(CodecError::LcmEncode(message.into()))
}
fn decode_error<T>(message: &str) -> CodecResult<T> {
    Err(CodecError::LcmDecode(message.into()))
}

#[derive(Default)]
struct Writer {
    bytes: Vec<u8>,
}
impl Writer {
    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }
    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    fn f64(&mut self, value: f64) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }
    fn string(&mut self, value: &str) -> CodecResult<()> {
        validate_string(value)?;
        self.u16(value.len() as u16);
        self.bytes.extend_from_slice(value.as_bytes());
        Ok(())
    }
}

struct Reader<'a> {
    bytes: &'a [u8],
    offset: usize,
}
impl<'a> Reader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }
    fn take(&mut self, count: usize) -> CodecResult<&'a [u8]> {
        let end = self
            .offset
            .checked_add(count)
            .ok_or_else(|| CodecError::LcmDecode("sensor wire offset overflow".into()))?;
        if end > self.bytes.len() {
            return decode_error("sensor wire payload is truncated");
        }
        let value = &self.bytes[self.offset..end];
        self.offset = end;
        Ok(value)
    }
    fn u8(&mut self) -> CodecResult<u8> {
        Ok(self.take(1)?[0])
    }
    fn u16(&mut self) -> CodecResult<u16> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }
    fn u32(&mut self) -> CodecResult<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn u64(&mut self) -> CodecResult<u64> {
        Ok(u64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f64(&mut self) -> CodecResult<f64> {
        Ok(f64::from_le_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn string(&mut self) -> CodecResult<String> {
        let len = self.u16()? as usize;
        if len == 0 || len > SENSOR_WIRE_MAX_STRING_BYTES {
            return decode_error("invalid sensor wire string length");
        }
        String::from_utf8(self.take(len)?.to_vec())
            .map_err(|_| CodecError::LcmDecode("sensor wire string is not UTF-8".into()))
    }
    fn bytes(&mut self) -> CodecResult<Vec<u8>> {
        let len = self.u32()? as usize;
        if len > SENSOR_WIRE_MAX_PAYLOAD_BYTES {
            return decode_error("sensor wire payload exceeds limit");
        }
        Ok(self.take(len)?.to_vec())
    }
    fn optional_bytes(&mut self) -> CodecResult<Option<Vec<u8>>> {
        match self.u8()? {
            0 => Ok(None),
            1 => Ok(Some(self.bytes()?)),
            _ => decode_error("invalid optional payload flag"),
        }
    }
    fn done(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> SensorIdentity {
        SensorIdentity {
            schema_version: 1,
            sensor_id: "go2_l2".into(),
            source_sequence: 1,
            timestamp_ns: 1_000,
            clock_domain: "unix_ns".into(),
            frame_id: "go2_base_link".into(),
            registration_state: "body".into(),
            registration_frame_id: "go2_base_link".into(),
            calibration_ref: Some("go2-urdf-base-v1".into()),
        }
    }
    fn cloud() -> SensorFrame {
        SensorFrame::Cloud(CanonicalCloudFrame {
            identity: identity(),
            point_count: 1,
            scan_duration_ns: 10,
            payload: CloudPayload::Inline {
                xyz_f32_le: vec![0, 0, 128, 63, 0, 0, 0, 64, 0, 0, 64, 64],
                intensity_f32_le: None,
                ring_u16_le: None,
                time_offset_ns_le: Some(5_u64.to_le_bytes().to_vec()),
            },
        })
    }
    #[test]
    fn cloud_roundtrip_is_little_endian() {
        let frame = cloud();
        let bytes = encode(&frame).unwrap();
        assert_eq!(&bytes[..4], b"NWS1");
        assert_eq!(decode(&bytes).unwrap(), frame);
    }
    #[test]
    fn imu_roundtrip_and_nan_rejection() {
        let frame = SensorFrame::Imu(CanonicalImuFrame {
            identity: identity(),
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rps: [4.0, 5.0, 6.0],
        });
        assert_eq!(decode(&encode(&frame).unwrap()).unwrap(), frame);
        let bad = SensorFrame::Imu(CanonicalImuFrame {
            identity: identity(),
            accel_mps2: [f64::NAN, 0.0, 0.0],
            gyro_rps: [0.0; 3],
        });
        assert!(encode(&bad).is_err());
    }
    #[test]
    fn malformed_and_trailing_frames_are_rejected() {
        let bytes = encode(&cloud()).unwrap();
        assert!(decode(&bytes[..bytes.len() - 1]).is_err());
        let mut trailing = bytes.clone();
        trailing.push(0);
        assert!(decode(&trailing).is_err());
    }
    #[test]
    fn external_reference_must_match_ownership_and_lease() {
        let bad = SensorFrame::Cloud(CanonicalCloudFrame {
            identity: identity(),
            point_count: 1,
            scan_duration_ns: 1,
            payload: CloudPayload::External {
                ownership: PayloadOwnership::Shm,
                payload_ref: "artifact://x".into(),
                payload_layout: vec!["xyz_f32_le".into()],
                payload_size_bytes: 12,
                lease_epoch: 0,
                expiry_ns: 1,
            },
        });
        assert!(encode(&bad).is_err());
    }
    #[test]
    fn sequence_regression_is_rejected() {
        let mut validator = SensorSequenceValidator::default();
        let first = cloud();
        let mut second = cloud();
        if let SensorFrame::Cloud(ref mut frame) = second {
            frame.identity.source_sequence = 2;
            frame.identity.timestamp_ns = 900;
        }
        validator.validate(&first).unwrap();
        assert!(validator.validate(&second).is_err());
    }
}
