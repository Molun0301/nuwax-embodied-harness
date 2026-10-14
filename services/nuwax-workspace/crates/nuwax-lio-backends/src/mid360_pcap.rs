//! Livox Mid-360 classic PCAP 到 LIO 输入帧的严格流式转换。

use crate::{ImuSample, LioInputFrame, TimedPoint, TimestampedCloud};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;
use std::io::{self, Read};

pub const RULES_VERSION: &str = "mid360-cartesian-high-v1";
pub const WINDOW_NS: u64 = 100_000_000;
pub const STANDARD_GRAVITY_MPS2: f64 = 9.80665;
pub const MAX_RECORD_BYTES: usize = 65_535;
pub const MAX_POINTS_PER_WINDOW: usize = 250_000;
pub const MAX_IMU_PER_WINDOW: usize = 10_000;
pub const MAX_STREAM_SKEW_NS: u64 = WINDOW_NS;

const MAX_BUFFERED_WINDOWS: usize = 3;
const MAX_BUFFERED_ELEMENTS: usize =
    MAX_BUFFERED_WINDOWS * (MAX_POINTS_PER_WINDOW + MAX_IMU_PER_WINDOW);
const MAX_LEADING_DATA_PACKETS: usize = 1_024;
const MAX_LEADING_DATA_BYTES: usize = 16 * 1024 * 1024;
const MAX_LEADING_TRANSACTION_ACCOUNTED_HEAP_BYTES: usize = 128 * 1024 * 1024;
const LIVox_DATA_HEADER_BYTES: usize = 36;
const SDK_WRAPPER_BYTES: usize = 24;
const POINT_BYTES: usize = 14;
const IMU_BYTES: usize = 24;
const MULTICAST_IP: [u8; 4] = [224, 1, 1, 5];
const BROADCAST_IP: [u8; 4] = [255, 255, 255, 255];
const MULTICAST_MAC: [u8; 6] = [0x01, 0x00, 0x5e, 0x01, 0x01, 0x05];
const BROADCAST_MAC: [u8; 6] = [0xff; 6];

#[derive(Debug)]
pub enum ConversionError {
    Io(io::Error),
    Invalid(String),
    Limit(String),
}

impl fmt::Display for ConversionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "I/O error: {error}"),
            Self::Invalid(message) => f.write_str(message),
            Self::Limit(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for ConversionError {}

impl From<io::Error> for ConversionError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

type Result<T> = std::result::Result<T, ConversionError>;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConversionRules {
    pub expected_mac: [u8; 6],
    pub expected_ip: [u8; 4],
    pub expected_serial: String,
    pub frame_id: String,
}

impl ConversionRules {
    pub fn validate(&self) -> Result<()> {
        if self.expected_serial.is_empty()
            || self.expected_serial.len() >= 16
            || !self
                .expected_serial
                .bytes()
                .all(|byte| byte.is_ascii_graphic())
        {
            return invalid("expected serial must contain 1..15 printable ASCII bytes");
        }
        if self.frame_id.trim().is_empty()
            || self.frame_id.len() > 255
            || self.frame_id.bytes().any(|byte| byte.is_ascii_control())
        {
            return invalid(
                "frame_id must contain 1..255 UTF-8 bytes without ASCII control characters",
            );
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PcapStatistics {
    pub magic: String,
    pub version_major: u16,
    pub version_minor: u16,
    pub snaplen: u32,
    pub linktype: u32,
    pub records: u64,
    pub truncated_records: u64,
    pub first_capture_us: Option<u64>,
    pub last_capture_us: Option<u64>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChecksumStatistics {
    pub ipv4_verified: u64,
    pub udp_verified: u64,
    pub udp_absent: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct SequenceStatistics {
    pub packets: u64,
    pub dots: u64,
    pub first_udp_cnt: Option<u16>,
    pub last_udp_cnt: Option<u16>,
    pub wraps: u64,
    pub missing: u64,
    pub repeated: u64,
    pub reversed: u64,
    pub first_timestamp_ns: Option<u64>,
    pub last_timestamp_ns: Option<u64>,
    pub first_frame_cnt: Option<u8>,
    pub last_frame_cnt: Option<u8>,
    pub frame_cnt_zero: u64,
    pub time_type: Option<u8>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FilterStatistics {
    pub input_points: u64,
    pub output_points: u64,
    pub zero_placeholder: u64,
    pub blind_lt_0_5m: u64,
    pub other_rejected: u64,
    pub blind_boundary: String,
    pub point_filter_num: u8,
    pub tags: BTreeMap<u8, u64>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct WindowStatistics {
    pub emitted: u64,
    pub dropped_first_incomplete: u64,
    pub dropped_first_reason: Option<String>,
    pub dropped_tail_incomplete: u64,
    pub dropped_tail_reason: Option<String>,
    pub total_points: u64,
    pub total_imu: u64,
    pub min_points: Option<u64>,
    pub max_points: Option<u64>,
    pub min_imu: Option<u64>,
    pub max_imu: Option<u64>,
    pub first_timestamp_ns: Option<i64>,
    pub last_timestamp_ns: Option<i64>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EndpointStatistics {
    pub point_56300_to_56301: u64,
    pub imu_56400_to_56401: u64,
    pub discovery_56000_to_56000: u64,
    pub status_56200_to_56201: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ToolProvenance {
    pub package_version: Option<String>,
    pub binary_sha256: Option<String>,
    pub rust_version: Option<String>,
    pub target_triple: Option<String>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileIdentityProvenance {
    pub basename: Option<String>,
    pub size_bytes: Option<u64>,
    pub sha256: Option<String>,
    pub fd_device: Option<u64>,
    pub fd_inode: Option<u64>,
    pub fd_link_count: Option<u64>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct OutputProvenance {
    pub size_bytes: Option<u64>,
    pub sha256: Option<String>,
    pub max_output_bytes: Option<u64>,
    pub deserialized_frames: Option<u64>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationProvenance {
    pub fastlio2_frames_processed: Option<u64>,
    pub fastlio2_contract_passed: Option<bool>,
    pub fastlio2_healthy_frames: Option<u64>,
    pub fastlio2_unhealthy_frames: Option<u64>,
    pub pointlio_frames_processed: Option<u64>,
    pub pointlio_contract_passed: Option<bool>,
    pub pointlio_healthy_frames: Option<u64>,
    pub pointlio_unhealthy_frames: Option<u64>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalProvenance {
    pub tool: ToolProvenance,
    pub input: FileIdentityProvenance,
    pub output: OutputProvenance,
    pub validation: ValidationProvenance,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConversionStatistics {
    pub pcap: PcapStatistics,
    pub checksums: ChecksumStatistics,
    pub endpoints: EndpointStatistics,
    pub point_stream: SequenceStatistics,
    pub imu_stream: SequenceStatistics,
    pub filter: FilterStatistics,
    pub windows: WindowStatistics,
    pub discovery_packets: u64,
    pub status_packets: u64,
    pub data_crc_zero_packets: u64,
    pub command_port: Option<u16>,
    pub device_type: Option<u8>,
    pub raw_anchor_ns: Option<u64>,
    pub raw_first_timestamp_ns: Option<u64>,
    pub raw_last_timestamp_ns: Option<u64>,
    pub time_type: Option<u8>,
    pub max_buffered_windows: usize,
    pub max_buffered_elements: usize,
    pub peak_buffered_windows: usize,
    pub peak_buffered_elements: usize,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConversionManifest {
    pub schema_version: String,
    pub rules_version: String,
    pub time_domain: String,
    pub window_semantics: String,
    pub point_time_formula: String,
    pub data_crc_status: String,
    pub canonical_json_profile: String,
    pub timestamp_normalization: String,
    pub millimeters_to_meters: String,
    pub acceleration_conversion: String,
    pub external_provenance: ExternalProvenance,
    pub fail_closed_reject_categories: Vec<String>,
    pub expected_mac: [u8; 6],
    pub expected_ip: [u8; 4],
    pub expected_serial: String,
    pub frame_id: String,
    pub window_ns: u64,
    pub max_stream_skew_ns: u64,
    pub blind_radius_mm: u32,
    pub standard_gravity_mps2: String,
    pub statistics: ConversionStatistics,
}

impl ConversionManifest {
    pub fn from_result(rules: &ConversionRules, statistics: ConversionStatistics) -> Self {
        Self {
            schema_version: "nuwax.lio-input-provenance.v1".into(),
            rules_version: RULES_VERSION.into(),
            time_domain: "device_relative_unsynchronized".into(),
            window_semantics: "100ms half-open; cloud timestamp is window_end_ns-1".into(),
            point_time_formula: "timestamp_ns + index*(time_interval*100/dot_num)".into(),
            data_crc_status: "zero_field_is_unprotected; nonzero_field_is_crc32_ieee_over_payload_from_offset_28_and_must_verify".into(),
            canonical_json_profile: "nuwax.lio-input-json.v1".into(),
            timestamp_normalization: "raw_timestamp_ns - raw_anchor_ns".into(),
            millimeters_to_meters: "f64(integer_mm) / 1000.0".into(),
            acceleration_conversion: "f64(acceleration_g) * 9.80665".into(),
            external_provenance: ExternalProvenance::default(),
            fail_closed_reject_categories: vec![
                "pcap_container".into(),
                "ethernet_ipv4_udp".into(),
                "endpoint_allowlist".into(),
                "command_wrapper_crc".into(),
                "discovery_identity".into(),
                "data_header".into(),
                "sequence".into(),
                "timestamp".into(),
                "checksum".into(),
                "numeric_contract".into(),
                "resource_limit".into(),
                "window_contract".into(),
            ],
            expected_mac: rules.expected_mac,
            expected_ip: rules.expected_ip,
            expected_serial: rules.expected_serial.clone(),
            frame_id: rules.frame_id.clone(),
            window_ns: WINDOW_NS,
            max_stream_skew_ns: MAX_STREAM_SKEW_NS,
            blind_radius_mm: 500,
            standard_gravity_mps2: "9.80665".into(),
            statistics,
        }
    }
}

#[derive(Clone, Default)]
struct WindowBuffer {
    points: Vec<TimedPoint>,
    imu: Vec<ImuSample>,
}

struct WindowSlots {
    entries: Vec<(u64, WindowBuffer)>,
}

impl WindowSlots {
    fn try_new() -> Result<Self> {
        let mut entries = Vec::new();
        entries
            .try_reserve_exact(MAX_BUFFERED_WINDOWS)
            .map_err(|_| ConversionError::Limit("window slot allocation failed".into()))?;
        Ok(Self { entries })
    }
    fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
    fn len(&self) -> usize {
        self.entries.len()
    }
    fn contains_key(&self, key: &u64) -> bool {
        self.entries.iter().any(|(value, _)| value == key)
    }
    fn values(&self) -> impl Iterator<Item = &WindowBuffer> {
        self.entries.iter().map(|(_, value)| value)
    }
    fn keys(&self) -> impl Iterator<Item = &u64> {
        self.entries.iter().map(|(key, _)| key)
    }
    fn get_or_insert(&mut self, key: u64) -> Result<&mut WindowBuffer> {
        match self.entries.binary_search_by_key(&key, |(value, _)| *value) {
            Ok(index) => Ok(&mut self.entries[index].1),
            Err(index) => {
                if self.entries.len() >= MAX_BUFFERED_WINDOWS {
                    return limit("window slot count exceeds the frozen maximum");
                }
                self.entries.insert(index, (key, WindowBuffer::default()));
                Ok(&mut self.entries[index].1)
            }
        }
    }
    fn remove(&mut self, key: &u64) -> Option<WindowBuffer> {
        self.entries
            .binary_search_by_key(key, |(value, _)| *value)
            .ok()
            .map(|index| self.entries.remove(index).1)
    }
}

#[derive(Clone, Default)]
struct StreamState {
    previous_udp_cnt: Option<u16>,
    previous_timestamp_ns: Option<u64>,
}

#[derive(Clone)]
struct LeadingDataPacket {
    payload: Vec<u8>,
    expected_type: u8,
}

#[derive(Clone, Copy)]
struct DiscoveryEvidence {
    command_port: u16,
    device_type: u8,
}

struct Converter<'a, F> {
    rules: &'a ConversionRules,
    sink: F,
    statistics: ConversionStatistics,
    point_state: StreamState,
    imu_state: StreamState,
    windows: WindowSlots,
    anchor_ns: Option<u64>,
    latest_point_ns: Option<u64>,
    latest_imu_ns: Option<u64>,
    previous_effective_imu_ns: Option<u64>,
    buffered_elements: usize,
    leading_data: Vec<LeadingDataPacket>,
    leading_data_bytes: usize,
    leading_buffered_packets: u64,
    leading_buffered_bytes: u64,
    leading_replayed_packets: u64,
    data_crc_verified_packets: u64,
    leading_replayed_bytes: u64,
    tag_counts: [u64; 256],
    next_window: u64,
    first_point_ns: Option<u64>,
    first_imu_ns: Option<u64>,
    last_emitted_window: Option<u64>,
}

/// 严格转换 PCAP，并把已完成帧依序交给 `sink`。
///
/// `sink` 可能在后续帧返回错误前观察到成功前缀；此时 Converter 内部状态不提交。
/// 需要外部原子可见性的调用方必须把 sink 指向事务 staging，并在 Err 时 abort。
/// 发布 CLI 使用 staging/no-final 合同，不把通用 `FnMut` 误声明为可回滚事务。
pub fn convert_pcap<R, F>(
    mut reader: R,
    rules: &ConversionRules,
    sink: F,
) -> Result<ConversionManifest>
where
    R: Read,
    F: FnMut(LioInputFrame) -> Result<()>,
{
    rules.validate()?;
    let mut global = [0_u8; 24];
    read_exact_required(&mut reader, &mut global, "truncated PCAP global header")?;
    if global[0..4] != [0xd4, 0xc3, 0xb2, 0xa1]
        || le_u16(&global, 4)? != 2
        || le_u16(&global, 6)? != 4
        || le_u32(&global, 20)? != 1
    {
        return invalid("only little-endian microsecond classic PCAP 2.4 EN10MB is accepted");
    }
    let snaplen = le_u32(&global, 16)?;
    if !(64..=MAX_RECORD_BYTES as u32).contains(&snaplen) {
        return invalid("PCAP snaplen is outside 64..65535");
    }

    let windows = WindowSlots::try_new()?;
    let mut converter = Converter {
        rules,
        sink,
        statistics: ConversionStatistics::default(),
        point_state: StreamState::default(),
        imu_state: StreamState::default(),
        windows,
        anchor_ns: None,
        latest_point_ns: None,
        latest_imu_ns: None,
        previous_effective_imu_ns: None,
        buffered_elements: 0,
        leading_data: Vec::new(),
        leading_data_bytes: 0,
        leading_buffered_packets: 0,
        leading_buffered_bytes: 0,
        leading_replayed_packets: 0,
        data_crc_verified_packets: 0,
        leading_replayed_bytes: 0,
        tag_counts: [0; 256],
        next_window: 0,
        first_point_ns: None,
        first_imu_ns: None,
        last_emitted_window: None,
    };
    converter.statistics.pcap.magic = "d4c3b2a1".into();
    converter.statistics.pcap.version_major = 2;
    converter.statistics.pcap.version_minor = 4;
    converter.statistics.pcap.snaplen = snaplen;
    converter.statistics.pcap.linktype = 1;
    converter.statistics.filter.blind_boundary = "drop radius < 0.5m; retain radius == 0.5m".into();
    converter.statistics.filter.point_filter_num = 1;
    converter.statistics.max_buffered_windows = MAX_BUFFERED_WINDOWS;
    converter.statistics.max_buffered_elements = MAX_BUFFERED_ELEMENTS;

    loop {
        let mut header = [0_u8; 16];
        let count = read_record_header(&mut reader, &mut header)?;
        if count == 0 {
            break;
        }
        let seconds = le_u32(&header, 0)? as u64;
        let micros = le_u32(&header, 4)? as u64;
        let included = le_u32(&header, 8)?;
        let original = le_u32(&header, 12)?;
        if micros >= 1_000_000 || included != original || included > snaplen {
            return invalid("invalid PCAP record timestamp or length");
        }
        let capture_us = seconds
            .checked_mul(1_000_000)
            .and_then(|value| value.checked_add(micros))
            .ok_or_else(|| ConversionError::Invalid("PCAP capture timestamp overflow".into()))?;
        if converter
            .statistics
            .pcap
            .last_capture_us
            .is_some_and(|previous| capture_us < previous)
        {
            return invalid("PCAP capture timestamps are not monotonic");
        }
        let record_count = checked_inc(converter.statistics.pcap.records, "record count")?;
        let size = usize::try_from(included).map_err(|_| {
            ConversionError::Invalid("PCAP record length does not fit usize".into())
        })?;
        let mut record = vec![0_u8; size];
        read_exact_required(&mut reader, &mut record, "truncated PCAP record body")?;
        converter.parse_record(&record)?;
        converter
            .statistics
            .pcap
            .first_capture_us
            .get_or_insert(capture_us);
        converter.statistics.pcap.last_capture_us = Some(capture_us);
        converter.statistics.pcap.records = record_count;
    }
    converter.finish()
}

impl<F> Converter<'_, F>
where
    F: FnMut(LioInputFrame) -> Result<()>,
{
    fn parse_record(&mut self, frame: &[u8]) -> Result<()> {
        if frame.len() < 14 {
            return invalid("truncated Ethernet frame");
        }
        if frame[12..14] != [0x08, 0x00] {
            return invalid("only untagged Ethernet IPv4 frames are accepted");
        }
        let destination_mac = slice_array::<6>(frame, 0)?;
        let source_mac = slice_array::<6>(frame, 6)?;
        if source_mac != self.rules.expected_mac {
            return invalid("Ethernet source MAC does not match the pinned device");
        }
        let ip = &frame[14..];
        if ip.len() < 20 || ip[0] >> 4 != 4 {
            return invalid("truncated or non-IPv4 packet");
        }
        let ihl = usize::from(ip[0] & 0x0f)
            .checked_mul(4)
            .ok_or_else(|| ConversionError::Invalid("IPv4 IHL overflow".into()))?;
        if ihl < 20 || ihl > ip.len() {
            return invalid("invalid IPv4 IHL");
        }
        let total_length = usize::from(be_u16(ip, 2)?);
        let minimum_udp_length = ihl
            .checked_add(8)
            .ok_or_else(|| ConversionError::Invalid("IPv4 payload offset overflow".into()))?;
        if total_length != ip.len() || total_length < minimum_udp_length {
            return invalid("IPv4 total length does not close the Ethernet payload");
        }
        if ip[9] != 17 || be_u16(ip, 6)? & 0x3fff != 0 {
            return invalid("only unfragmented IPv4 UDP packets are accepted");
        }
        if ones_complement_sum(&ip[..ihl]) != 0xffff {
            return invalid("invalid IPv4 header checksum");
        }

        let source_ip = slice_array::<4>(ip, 12)?;
        let destination_ip = slice_array::<4>(ip, 16)?;
        if source_ip != self.rules.expected_ip {
            return invalid("IPv4 source does not match the pinned device");
        }
        match destination_ip {
            BROADCAST_IP if destination_mac == BROADCAST_MAC => {}
            MULTICAST_IP if destination_mac == MULTICAST_MAC => {}
            _ => {
                return invalid("IPv4 destination and Ethernet destination are not an allowed pair")
            }
        }

        let udp = &ip[ihl..];
        let udp_length = usize::from(be_u16(udp, 4)?);
        if udp_length != udp.len() || udp_length < 8 {
            return invalid("UDP length does not close the IPv4 payload");
        }
        let checksum = be_u16(udp, 6)?;
        let udp_checksum_absent = checksum == 0;
        if !udp_checksum_absent && udp_checksum(source_ip, destination_ip, udp)? != 0xffff {
            return invalid("invalid UDP checksum");
        }
        let source_port = be_u16(udp, 0)?;
        let destination_port = be_u16(udp, 2)?;
        let payload = &udp[8..];
        let ipv4_count = checked_inc(
            self.statistics.checksums.ipv4_verified,
            "IPv4 checksum count",
        )?;
        let udp_count = if udp_checksum_absent {
            checked_inc(
                self.statistics.checksums.udp_absent,
                "absent UDP checksum count",
            )?
        } else {
            checked_inc(
                self.statistics.checksums.udp_verified,
                "verified UDP checksum count",
            )?
        };
        match (source_port, destination_port, destination_ip) {
            (56300, 56301, MULTICAST_IP) => {
                let endpoint_count = checked_inc(
                    self.statistics.endpoints.point_56300_to_56301,
                    "point endpoint packet count",
                )?;
                self.parse_data(payload, 1)?;
                self.statistics.endpoints.point_56300_to_56301 = endpoint_count;
            }
            (56400, 56401, MULTICAST_IP) => {
                let endpoint_count = checked_inc(
                    self.statistics.endpoints.imu_56400_to_56401,
                    "IMU endpoint packet count",
                )?;
                self.parse_data(payload, 0)?;
                self.statistics.endpoints.imu_56400_to_56401 = endpoint_count;
            }
            (56000, 56000, BROADCAST_IP) => {
                let endpoint_count = checked_inc(
                    self.statistics.endpoints.discovery_56000_to_56000,
                    "discovery endpoint packet count",
                )?;
                self.parse_command(payload, true)?;
                self.statistics.endpoints.discovery_56000_to_56000 = endpoint_count;
            }
            (56200, 56201, MULTICAST_IP) => {
                let endpoint_count = checked_inc(
                    self.statistics.endpoints.status_56200_to_56201,
                    "status endpoint packet count",
                )?;
                self.parse_command(payload, false)?;
                self.statistics.endpoints.status_56200_to_56201 = endpoint_count;
            }
            _ => return invalid("UDP endpoint is not one of the four frozen Livox streams"),
        }
        self.statistics.checksums.ipv4_verified = ipv4_count;
        if udp_checksum_absent {
            self.statistics.checksums.udp_absent = udp_count;
        } else {
            self.statistics.checksums.udp_verified = udp_count;
        }
        Ok(())
    }

    fn parse_command(&mut self, payload: &[u8], discovery: bool) -> Result<()> {
        let evidence = validate_command(payload, discovery, self.rules)?;
        if discovery {
            if self.statistics.discovery_packets == 0 {
                self.replay_leading_data(evidence)
            } else {
                self.statistics.discovery_packets =
                    checked_inc(self.statistics.discovery_packets, "discovery packet count")?;
                Ok(())
            }
        } else {
            self.statistics.status_packets =
                checked_inc(self.statistics.status_packets, "status packet count")?;
            Ok(())
        }
    }

    fn parse_data(&mut self, payload: &[u8], expected_type: u8) -> Result<()> {
        validate_data_payload(payload, expected_type)?;
        if self.statistics.discovery_packets == 0 {
            return self.buffer_leading_data(payload, expected_type);
        }
        self.process_validated_data(payload, expected_type)
    }

    fn buffer_leading_data(&mut self, payload: &[u8], expected_type: u8) -> Result<()> {
        let packet_count =
            self.leading_data.len().checked_add(1).ok_or_else(|| {
                ConversionError::Limit("leading data packet count overflow".into())
            })?;
        let byte_count = self
            .leading_data_bytes
            .checked_add(payload.len())
            .ok_or_else(|| ConversionError::Limit("leading data byte count overflow".into()))?;
        if packet_count > MAX_LEADING_DATA_PACKETS || byte_count > MAX_LEADING_DATA_BYTES {
            return limit("leading data before discovery exceeds the bounded cache");
        }
        let current_heap = leading_cache_heap_bytes(&self.leading_data)?;
        let candidate_outer =
            checked_heap_mul(packet_count, std::mem::size_of::<LeadingDataPacket>())?;
        let candidate_payload = checked_heap_add(self.leading_data_bytes, payload.len())?;
        let candidate_heap = checked_heap_add(candidate_outer, candidate_payload)?;
        enforce_heap_limit(checked_heap_add(
            checked_heap_add(current_heap, candidate_heap)?,
            statistics_heap_bytes(&self.statistics)?,
        )?)?;
        let mut owned = Vec::new();
        owned
            .try_reserve_exact(payload.len())
            .map_err(|_| ConversionError::Limit("leading payload allocation failed".into()))?;
        owned.extend_from_slice(payload);
        let mut candidate_packets = Vec::new();
        candidate_packets
            .try_reserve_exact(packet_count)
            .map_err(|_| ConversionError::Limit("leading packet allocation failed".into()))?;
        for packet in &self.leading_data {
            candidate_packets.push(LeadingDataPacket {
                payload: fallible_copy_bytes(&packet.payload)?,
                expected_type: packet.expected_type,
            });
        }
        candidate_packets.push(LeadingDataPacket {
            payload: owned,
            expected_type,
        });
        let actual = leading_cache_heap_bytes(&candidate_packets)?;
        enforce_heap_limit(checked_heap_add(
            checked_heap_add(current_heap, actual)?,
            statistics_heap_bytes(&self.statistics)?,
        )?)?;
        self.leading_data = candidate_packets;
        self.leading_data_bytes = byte_count;
        self.leading_buffered_packets = u64::try_from(packet_count)
            .map_err(|_| ConversionError::Limit("leading packet statistic overflow".into()))?;
        self.leading_buffered_bytes = u64::try_from(byte_count)
            .map_err(|_| ConversionError::Limit("leading byte statistic overflow".into()))?;
        Ok(())
    }

    fn replay_leading_data(&mut self, evidence: DiscoveryEvidence) -> Result<()> {
        if self.anchor_ns.is_some()
            || !self.windows.is_empty()
            || self.buffered_elements != 0
            || self.statistics.point_stream.packets != 0
            || self.statistics.imu_stream.packets != 0
        {
            return invalid("leading replay did not start from a pristine stream state");
        }
        self.preflight_replay_heap()?;
        {
            let mut validation = self.new_replay_shadow(evidence, |_| Ok(()))?;
            replay_packets(&mut validation, &self.leading_data)?;
            validation.enforce_stream_skew()?;
        }
        self.preflight_replay_heap()?;
        let statistics = fallible_clone_statistics(&self.statistics)?;
        let rules = self.rules;
        let packets = &self.leading_data;
        let sink = &mut self.sink;
        let mut shadow = Converter {
            rules,
            sink: |frame| sink(frame),
            statistics,
            point_state: StreamState::default(),
            imu_state: StreamState::default(),
            windows: WindowSlots::try_new()?,
            anchor_ns: None,
            latest_point_ns: None,
            latest_imu_ns: None,
            previous_effective_imu_ns: None,
            buffered_elements: 0,
            leading_data: Vec::new(),
            leading_data_bytes: 0,
            leading_buffered_packets: self.leading_buffered_packets,
            leading_buffered_bytes: self.leading_buffered_bytes,
            leading_replayed_packets: self.leading_replayed_packets,
            data_crc_verified_packets: self.data_crc_verified_packets,
            leading_replayed_bytes: self.leading_replayed_bytes,
            tag_counts: self.tag_counts,
            next_window: 0,
            first_point_ns: None,
            first_imu_ns: None,
            last_emitted_window: None,
        };
        apply_discovery_evidence(&mut shadow, evidence)?;
        replay_packets(&mut shadow, packets)?;
        shadow.enforce_stream_skew()?;
        let Converter {
            statistics,
            point_state,
            imu_state,
            windows,
            anchor_ns,
            latest_point_ns,
            latest_imu_ns,
            previous_effective_imu_ns,
            buffered_elements,
            leading_buffered_packets,
            leading_buffered_bytes,
            leading_replayed_packets,
            data_crc_verified_packets,
            leading_replayed_bytes,
            tag_counts,
            next_window,
            first_point_ns,
            first_imu_ns,
            last_emitted_window,
            ..
        } = shadow;
        self.statistics = statistics;
        self.point_state = point_state;
        self.imu_state = imu_state;
        self.windows = windows;
        self.anchor_ns = anchor_ns;
        self.latest_point_ns = latest_point_ns;
        self.latest_imu_ns = latest_imu_ns;
        self.previous_effective_imu_ns = previous_effective_imu_ns;
        self.buffered_elements = buffered_elements;
        self.leading_buffered_packets = leading_buffered_packets;
        self.leading_buffered_bytes = leading_buffered_bytes;
        self.leading_replayed_packets = leading_replayed_packets;
        self.data_crc_verified_packets = data_crc_verified_packets;
        self.leading_replayed_bytes = leading_replayed_bytes;
        self.tag_counts = tag_counts;
        self.next_window = next_window;
        self.first_point_ns = first_point_ns;
        self.first_imu_ns = first_imu_ns;
        self.last_emitted_window = last_emitted_window;
        self.leading_data.clear();
        self.leading_data_bytes = 0;
        Ok(())
    }

    fn new_replay_shadow<G>(&self, evidence: DiscoveryEvidence, sink: G) -> Result<Converter<'_, G>>
    where
        G: FnMut(LioInputFrame) -> Result<()>,
    {
        let mut shadow = Converter {
            rules: self.rules,
            sink,
            statistics: fallible_clone_statistics(&self.statistics)?,
            point_state: StreamState::default(),
            imu_state: StreamState::default(),
            windows: WindowSlots::try_new()?,
            anchor_ns: None,
            latest_point_ns: None,
            latest_imu_ns: None,
            previous_effective_imu_ns: None,
            buffered_elements: 0,
            leading_data: Vec::new(),
            leading_data_bytes: 0,
            leading_buffered_packets: self.leading_buffered_packets,
            leading_buffered_bytes: self.leading_buffered_bytes,
            leading_replayed_packets: self.leading_replayed_packets,
            data_crc_verified_packets: self.data_crc_verified_packets,
            leading_replayed_bytes: self.leading_replayed_bytes,
            tag_counts: self.tag_counts,
            next_window: 0,
            first_point_ns: None,
            first_imu_ns: None,
            last_emitted_window: None,
        };
        apply_discovery_evidence(&mut shadow, evidence)?;
        Ok(shadow)
    }

    fn preflight_replay_heap(&self) -> Result<()> {
        let raw = leading_cache_heap_bytes(&self.leading_data)?;
        let original = statistics_heap_bytes(&self.statistics)?;
        let shadow_statistics = original;
        let decoded = checked_heap_mul(self.leading_data_bytes, 8)?;
        let window_nodes = checked_heap_mul(MAX_BUFFERED_WINDOWS, 64 * 1024)?;
        let tag_slots = std::mem::size_of::<[u64; 256]>();
        let frame_ids = checked_heap_mul(MAX_LEADING_DATA_PACKETS, self.rules.frame_id.len())?;
        let total = checked_heap_add(raw, original)?;
        let total = checked_heap_add(total, shadow_statistics)?;
        let total = checked_heap_add(total, decoded)?;
        let total = checked_heap_add(total, window_nodes)?;
        let total = checked_heap_add(total, tag_slots)?;
        let total = checked_heap_add(total, frame_ids)?;
        if total > MAX_LEADING_TRANSACTION_ACCOUNTED_HEAP_BYTES {
            return limit("leading replay candidate exceeds accounted heap budget");
        }
        Ok(())
    }

    fn process_validated_data(&mut self, payload: &[u8], expected_type: u8) -> Result<()> {
        let stored_crc = le_u32(payload, 24)?;
        if stored_crc == 0 {
            self.statistics.data_crc_zero_packets =
                checked_inc(self.statistics.data_crc_zero_packets, "zero data CRC count")?;
        } else {
            self.data_crc_verified_packets =
                checked_inc(self.data_crc_verified_packets, "verified data CRC count")?;
        }
        let time_interval = u64::from(le_u16(payload, 3)?);
        let dots = le_u16(payload, 5)?;
        let frame_cnt = payload[9];
        let time_type = payload[11];
        let udp_cnt = le_u16(payload, 7)?;
        let timestamp_ns = le_u64(payload, 28)?;
        let interval_ns = time_interval
            .checked_mul(100)
            .ok_or_else(|| ConversionError::Invalid("Livox time interval overflow".into()))?
            / u64::from(dots);
        self.statistics
            .raw_first_timestamp_ns
            .get_or_insert(timestamp_ns);
        self.statistics.raw_last_timestamp_ns = Some(
            self.statistics
                .raw_last_timestamp_ns
                .map_or(timestamp_ns, |previous| previous.max(timestamp_ns)),
        );
        self.statistics.time_type.get_or_insert(time_type);
        if self.statistics.time_type != Some(time_type) || time_type != 0 {
            return invalid("Livox time_type is not the frozen device-relative value");
        }
        if expected_type == 1 {
            update_stream_state(
                &mut self.point_state,
                &mut self.statistics.point_stream,
                udp_cnt,
                timestamp_ns,
                dots,
                frame_cnt,
                time_type,
                "point",
            )?;
            self.parse_points(
                &payload[LIVox_DATA_HEADER_BYTES..],
                timestamp_ns,
                interval_ns,
            )?;
        } else {
            update_stream_state(
                &mut self.imu_state,
                &mut self.statistics.imu_stream,
                udp_cnt,
                timestamp_ns,
                dots,
                frame_cnt,
                time_type,
                "IMU",
            )?;
            self.parse_imu(
                &payload[LIVox_DATA_HEADER_BYTES..],
                timestamp_ns,
                interval_ns,
            )?;
        }
        self.enforce_stream_skew()?;
        self.finalize_ready_windows()
    }

    fn parse_points(&mut self, data: &[u8], packet_ns: u64, interval_ns: u64) -> Result<()> {
        let anchor = *self.anchor_ns.get_or_insert(packet_ns);
        self.statistics.raw_anchor_ns = Some(anchor);
        self.first_point_ns.get_or_insert(packet_ns);
        for (index, raw) in data.chunks_exact(POINT_BYTES).enumerate() {
            let effective_raw_ns = effective_time(packet_ns, interval_ns, index)?;
            let relative_ns = effective_raw_ns.checked_sub(anchor).ok_or_else(|| {
                ConversionError::Invalid("point timestamp predates anchor".into())
            })?;
            self.latest_point_ns = Some(effective_raw_ns);
            self.statistics.filter.input_points =
                checked_inc(self.statistics.filter.input_points, "input point count")?;
            let x_mm = le_i32(raw, 0)?;
            let y_mm = le_i32(raw, 4)?;
            let z_mm = le_i32(raw, 8)?;
            let tag = raw[13];
            self.tag_counts[usize::from(tag)] =
                checked_inc(self.tag_counts[usize::from(tag)], "point tag count")?;
            if x_mm == 0 && y_mm == 0 && z_mm == 0 {
                self.statistics.filter.zero_placeholder =
                    checked_inc(self.statistics.filter.zero_placeholder, "zero point count")?;
                continue;
            }
            let radius_sq = i128::from(x_mm) * i128::from(x_mm)
                + i128::from(y_mm) * i128::from(y_mm)
                + i128::from(z_mm) * i128::from(z_mm);
            if radius_sq < 250_000 {
                self.statistics.filter.blind_lt_0_5m =
                    checked_inc(self.statistics.filter.blind_lt_0_5m, "blind point count")?;
                continue;
            }
            let window = relative_ns / WINDOW_NS;
            let start = window
                .checked_mul(WINDOW_NS)
                .ok_or_else(|| ConversionError::Invalid("window start overflow".into()))?;
            let candidate_window_count = if self.windows.contains_key(&window) {
                self.windows.len()
            } else {
                self.windows
                    .len()
                    .checked_add(1)
                    .ok_or_else(|| ConversionError::Limit("window count overflow".into()))?
            };
            let candidate_bytes = checked_heap_add(
                std::mem::size_of::<TimedPoint>(),
                windows_candidate_heap_bytes(&self.windows, candidate_window_count)?,
            )?;
            let buffer = self.windows.get_or_insert(window)?;
            if buffer.points.len() >= MAX_POINTS_PER_WINDOW {
                return limit("point count exceeds the per-window limit");
            }
            if candidate_bytes > MAX_LEADING_TRANSACTION_ACCOUNTED_HEAP_BYTES {
                return limit("point candidate exceeds accounted heap budget");
            }
            buffer
                .points
                .try_reserve_exact(1)
                .map_err(|_| ConversionError::Limit("point window allocation failed".into()))?;
            buffer.points.push(TimedPoint {
                offset_ns: relative_ns.checked_sub(start).ok_or_else(|| {
                    ConversionError::Invalid("point window offset underflow".into())
                })?,
                x: normalize_zero(f64::from(x_mm) / 1000.0),
                y: normalize_zero(f64::from(y_mm) / 1000.0),
                z: normalize_zero(f64::from(z_mm) / 1000.0),
                intensity: f64::from(raw[12]),
            });
            self.note_buffered_element()?;
            self.statistics.filter.output_points =
                checked_inc(self.statistics.filter.output_points, "output point count")?;
        }
        Ok(())
    }

    fn parse_imu(&mut self, data: &[u8], packet_ns: u64, interval_ns: u64) -> Result<()> {
        let anchor = self.anchor_ns.ok_or_else(|| {
            ConversionError::Invalid("IMU packet arrived before the point anchor".into())
        })?;
        self.first_imu_ns.get_or_insert(packet_ns);
        for (index, raw) in data.chunks_exact(IMU_BYTES).enumerate() {
            let effective_raw_ns = effective_time(packet_ns, interval_ns, index)?;
            if self
                .previous_effective_imu_ns
                .is_some_and(|previous| effective_raw_ns <= previous)
            {
                return invalid("effective IMU sample timestamps are not strictly increasing");
            }
            self.previous_effective_imu_ns = Some(effective_raw_ns);
            let relative_ns = effective_raw_ns.checked_sub(anchor).ok_or_else(|| {
                ConversionError::Invalid("IMU timestamp predates point anchor".into())
            })?;
            self.latest_imu_ns = Some(effective_raw_ns);
            let values = [
                le_f32(raw, 0)?,
                le_f32(raw, 4)?,
                le_f32(raw, 8)?,
                le_f32(raw, 12)?,
                le_f32(raw, 16)?,
                le_f32(raw, 20)?,
            ];
            if values.iter().any(|value| !value.is_finite()) {
                return invalid("IMU contains a non-finite value");
            }
            let gyro = [
                f64::from(values[0]),
                f64::from(values[1]),
                f64::from(values[2]),
            ];
            let accel = [
                f64::from(values[3]) * STANDARD_GRAVITY_MPS2,
                f64::from(values[4]) * STANDARD_GRAVITY_MPS2,
                f64::from(values[5]) * STANDARD_GRAVITY_MPS2,
            ];
            if gyro.iter().any(|value| value.abs() > 1_000.0)
                || accel
                    .iter()
                    .any(|value| !value.is_finite() || value.abs() > 1_000.0)
            {
                return invalid("IMU conversion exceeds the LIO contract bounds");
            }
            let window = relative_ns / WINDOW_NS;
            let candidate_window_count = if self.windows.contains_key(&window) {
                self.windows.len()
            } else {
                self.windows
                    .len()
                    .checked_add(1)
                    .ok_or_else(|| ConversionError::Limit("window count overflow".into()))?
            };
            let candidate_bytes = checked_heap_add(
                std::mem::size_of::<ImuSample>(),
                windows_candidate_heap_bytes(&self.windows, candidate_window_count)?,
            )?;
            let buffer = self.windows.get_or_insert(window)?;
            if buffer.imu.len() >= MAX_IMU_PER_WINDOW {
                return limit("IMU count exceeds the per-window limit");
            }
            let timestamp_ns = i64::try_from(relative_ns).map_err(|_| {
                ConversionError::Invalid("relative IMU timestamp exceeds i64".into())
            })?;
            if candidate_bytes > MAX_LEADING_TRANSACTION_ACCOUNTED_HEAP_BYTES {
                return limit("IMU candidate exceeds accounted heap budget");
            }
            buffer
                .imu
                .try_reserve_exact(1)
                .map_err(|_| ConversionError::Limit("IMU window allocation failed".into()))?;
            buffer.imu.push(ImuSample {
                timestamp_ns,
                accel_mps2: accel.map(normalize_zero),
                gyro_rps: gyro.map(normalize_zero),
            });
            self.note_buffered_element()?;
        }
        Ok(())
    }

    fn note_buffered_element(&mut self) -> Result<()> {
        self.buffered_elements = self
            .buffered_elements
            .checked_add(1)
            .ok_or_else(|| ConversionError::Limit("buffered element count overflow".into()))?;
        if self.buffered_elements > MAX_BUFFERED_ELEMENTS {
            return limit("buffered element count exceeds the global limit");
        }
        if self.windows.len() > MAX_BUFFERED_WINDOWS {
            return limit("buffered window count exceeds the global limit");
        }
        self.statistics.peak_buffered_elements = self
            .statistics
            .peak_buffered_elements
            .max(self.buffered_elements);
        self.statistics.peak_buffered_windows = self
            .statistics
            .peak_buffered_windows
            .max(self.windows.len());
        Ok(())
    }

    fn enforce_stream_skew(&self) -> Result<()> {
        match (self.anchor_ns, self.latest_point_ns, self.latest_imu_ns) {
            (_, Some(point), Some(imu)) if point.abs_diff(imu) > MAX_STREAM_SKEW_NS => {
                invalid("point and IMU streams exceed the maximum permitted skew")
            }
            (Some(anchor), Some(point), None)
                if point.checked_sub(anchor).ok_or_else(|| {
                    ConversionError::Invalid("point timestamp predates anchor".into())
                })? > MAX_STREAM_SKEW_NS =>
            {
                invalid("IMU stream did not start within the maximum permitted skew")
            }
            (Some(anchor), None, Some(imu))
                if imu.checked_sub(anchor).ok_or_else(|| {
                    ConversionError::Invalid("IMU timestamp predates anchor".into())
                })? > MAX_STREAM_SKEW_NS =>
            {
                invalid("point stream did not start within the maximum permitted skew")
            }
            _ => Ok(()),
        }
    }

    fn finalize_ready_windows(&mut self) -> Result<()> {
        let (Some(anchor), Some(latest_point), Some(latest_imu)) =
            (self.anchor_ns, self.latest_point_ns, self.latest_imu_ns)
        else {
            return Ok(());
        };
        loop {
            let end_relative = self
                .next_window
                .checked_add(1)
                .and_then(|value| value.checked_mul(WINDOW_NS))
                .ok_or_else(|| ConversionError::Invalid("window end overflow".into()))?;
            let end_raw = anchor
                .checked_add(end_relative)
                .ok_or_else(|| ConversionError::Invalid("raw window end overflow".into()))?;
            if latest_point < end_raw || latest_imu < end_raw {
                break;
            }
            self.finalize_window(self.next_window, false)?;
            self.next_window = self
                .next_window
                .checked_add(1)
                .ok_or_else(|| ConversionError::Invalid("window index overflow".into()))?;
        }
        if self.windows.len() > MAX_BUFFERED_WINDOWS {
            return limit("stream skew retained more than the bounded window limit");
        }
        Ok(())
    }

    fn finalize_window(&mut self, index: u64, tail: bool) -> Result<()> {
        let buffer = self.windows.remove(&index).unwrap_or_default();
        let removed_elements = buffer
            .points
            .len()
            .checked_add(buffer.imu.len())
            .ok_or_else(|| ConversionError::Limit("removed buffer size overflow".into()))?;
        self.buffered_elements = self
            .buffered_elements
            .checked_sub(removed_elements)
            .ok_or_else(|| ConversionError::Invalid("buffer accounting underflow".into()))?;
        let start = index
            .checked_mul(WINDOW_NS)
            .ok_or_else(|| ConversionError::Invalid("window start overflow".into()))?;
        let raw_start = self
            .anchor_ns
            .ok_or_else(|| ConversionError::Invalid("missing raw timestamp anchor".into()))?
            .checked_add(start)
            .ok_or_else(|| ConversionError::Invalid("raw window start overflow".into()))?;
        let left_complete = self.first_point_ns.is_some_and(|first| first <= raw_start)
            && self.first_imu_ns.is_some_and(|first| first <= raw_start);
        if !left_complete && index == 0 {
            self.statistics.windows.dropped_first_incomplete = checked_inc(
                self.statistics.windows.dropped_first_incomplete,
                "first incomplete window count",
            )?;
            self.statistics.windows.dropped_first_reason =
                Some(fallible_copy_string("left_boundary_incomplete")?);
            return Ok(());
        }
        if tail {
            if self.statistics.windows.dropped_tail_incomplete != 0 {
                return invalid("more than one EOF tail window was dropped");
            }
            self.statistics.windows.dropped_tail_incomplete = checked_inc(
                self.statistics.windows.dropped_tail_incomplete,
                "tail incomplete window count",
            )?;
            self.statistics.windows.dropped_tail_reason =
                Some(fallible_copy_string("right_boundary_incomplete")?);
            return Ok(());
        }
        if !left_complete || buffer.points.is_empty() || buffer.imu.is_empty() {
            return invalid("an interior window is incomplete or has an empty stream");
        }
        if let Some(previous) = self.last_emitted_window {
            let expected = previous
                .checked_add(1)
                .ok_or_else(|| ConversionError::Invalid("emitted window index overflow".into()))?;
            if index != expected {
                return invalid("emitted windows are not contiguous");
            }
        }
        let timestamp_u64 = start
            .checked_add(WINDOW_NS - 1)
            .ok_or_else(|| ConversionError::Invalid("cloud timestamp overflow".into()))?;
        let timestamp_ns = i64::try_from(timestamp_u64)
            .map_err(|_| ConversionError::Invalid("cloud timestamp exceeds i64".into()))?;
        let point_count = u64::try_from(buffer.points.len())
            .map_err(|_| ConversionError::Limit("point count does not fit u64".into()))?;
        let imu_count = u64::try_from(buffer.imu.len())
            .map_err(|_| ConversionError::Limit("IMU count does not fit u64".into()))?;
        let frame = LioInputFrame {
            cloud: TimestampedCloud {
                timestamp_ns,
                frame_id: fallible_copy_string(&self.rules.frame_id)?,
                points: buffer.points,
            },
            imu: buffer.imu,
        };
        (self.sink)(frame)?;
        update_window_statistics(
            &mut self.statistics.windows,
            timestamp_ns,
            point_count,
            imu_count,
        )?;
        self.last_emitted_window = Some(index);
        Ok(())
    }

    fn finish(mut self) -> Result<ConversionManifest> {
        if self.statistics.discovery_packets == 0 {
            return invalid("no valid pinned Mid-360 discovery packet was observed");
        }
        if !self.leading_data.is_empty()
            || self.leading_replayed_packets != self.leading_buffered_packets
            || self.leading_replayed_bytes != self.leading_buffered_bytes
        {
            return invalid("leading data replay did not close after pinned discovery");
        }
        if self.statistics.point_stream.packets == 0 || self.statistics.imu_stream.packets == 0 {
            return invalid("both point and IMU streams are required");
        }
        self.enforce_stream_skew()?;
        self.finalize_ready_windows()?;
        let tail_indices: Vec<u64> = self.windows.keys().copied().collect();
        if tail_indices.iter().any(|index| *index < self.next_window) {
            return invalid("an unfinalized interior window remained at EOF");
        }
        if let Some(first) = tail_indices.first().copied() {
            let count = u64::try_from(tail_indices.len())
                .map_err(|_| ConversionError::Limit("tail window count does not fit u64".into()))?;
            let last =
                first
                    .checked_add(count.checked_sub(1).ok_or_else(|| {
                        ConversionError::Invalid("empty tail window range".into())
                    })?)
                    .ok_or_else(|| ConversionError::Invalid("tail window range overflow".into()))?;
            if tail_indices.iter().copied().ne(first..=last) {
                return invalid("tail windows are not contiguous");
            }
            for index in tail_indices.iter().copied().take(tail_indices.len() - 1) {
                self.finalize_window(index, false)?;
            }
            self.finalize_window(last, true)?;
        }
        if self.statistics.windows.dropped_tail_incomplete > 1 {
            return invalid("conversion dropped more than one EOF tail window");
        }
        if self.statistics.windows.emitted == 0 {
            return invalid("conversion produced no complete LIO windows");
        }
        for (tag, count) in self.tag_counts.iter().copied().enumerate() {
            if count != 0 {
                let tag = u8::try_from(tag)
                    .map_err(|_| ConversionError::Invalid("tag index exceeds u8".into()))?;
                self.statistics.filter.tags.insert(tag, count);
            }
        }
        Ok(ConversionManifest::from_result(self.rules, self.statistics))
    }
}

fn apply_discovery_evidence<F>(
    converter: &mut Converter<'_, F>,
    evidence: DiscoveryEvidence,
) -> Result<()> {
    converter.statistics.discovery_packets = checked_inc(
        converter.statistics.discovery_packets,
        "discovery packet count",
    )?;
    converter.statistics.command_port = Some(evidence.command_port);
    converter.statistics.device_type = Some(evidence.device_type);
    Ok(())
}

fn replay_packets<F>(converter: &mut Converter<'_, F>, packets: &[LeadingDataPacket]) -> Result<()>
where
    F: FnMut(LioInputFrame) -> Result<()>,
{
    for packet in packets {
        converter.process_validated_data(&packet.payload, packet.expected_type)?;
        converter.leading_replayed_packets = checked_inc(
            converter.leading_replayed_packets,
            "replayed leading packet count",
        )?;
        converter.leading_replayed_bytes = converter
            .leading_replayed_bytes
            .checked_add(u64::try_from(packet.payload.len()).map_err(|_| {
                ConversionError::Limit("replayed byte count does not fit u64".into())
            })?)
            .ok_or_else(|| ConversionError::Limit("replayed byte count overflow".into()))?;
    }
    Ok(())
}

fn validate_command(
    payload: &[u8],
    discovery: bool,
    rules: &ConversionRules,
) -> Result<DiscoveryEvidence> {
    if payload.len() < SDK_WRAPPER_BYTES
        || payload[0] != 0xaa
        || payload[1] != 0
        || usize::from(le_u16(payload, 2)?) != payload.len()
        || payload[12..18].iter().any(|byte| *byte != 0)
    {
        return invalid("invalid Livox SDK2 command wrapper");
    }
    if crc16_ccitt_false(&payload[..18]) != le_u16(payload, 18)? {
        return invalid("invalid Livox SDK2 command header CRC16");
    }
    let body = &payload[SDK_WRAPPER_BYTES..];
    if crc32_ieee(body) != le_u32(payload, 20)? {
        return invalid("invalid Livox SDK2 command data CRC32");
    }
    let command = le_u16(payload, 8)?;
    let expected_type = if discovery { 1 } else { 0 };
    if payload[10] != expected_type || payload[11] != 1 {
        return invalid("Livox command has an invalid type or sender");
    }
    if !discovery {
        if command != 0x0102 {
            return invalid("unexpected Livox status command identifier");
        }
        return Ok(DiscoveryEvidence {
            command_port: 56100,
            device_type: 9,
        });
    }
    if command != 0 || body.len() != 24 || body[0] != 0 || body[1] != 9 {
        return invalid("invalid Mid-360 discovery command or body");
    }
    let serial_bytes = &body[2..18];
    let nul = serial_bytes
        .iter()
        .position(|byte| *byte == 0)
        .ok_or_else(|| ConversionError::Invalid("discovery serial lacks NUL terminator".into()))?;
    if serial_bytes[nul..].iter().any(|byte| *byte != 0) {
        return invalid("discovery serial has non-zero bytes after NUL");
    }
    let serial = std::str::from_utf8(&serial_bytes[..nul])
        .map_err(|_| ConversionError::Invalid("discovery serial is not ASCII".into()))?;
    if !serial.is_ascii()
        || serial != rules.expected_serial
        || body[18..22] != rules.expected_ip
        || le_u16(body, 22)? != 56100
    {
        return invalid("discovery identity does not match the pinned Mid-360");
    }
    Ok(DiscoveryEvidence {
        command_port: 56100,
        device_type: 9,
    })
}

fn checked_heap_add(left: usize, right: usize) -> Result<usize> {
    left.checked_add(right)
        .ok_or_else(|| ConversionError::Limit("accounted heap arithmetic overflow".into()))
}
fn enforce_heap_limit(bytes: usize) -> Result<()> {
    if bytes > MAX_LEADING_TRANSACTION_ACCOUNTED_HEAP_BYTES {
        return limit("leading transaction accounted heap exceeds the frozen budget");
    }
    Ok(())
}
fn leading_cache_heap_bytes(packets: &Vec<LeadingDataPacket>) -> Result<usize> {
    let mut bytes = packets
        .capacity()
        .checked_mul(std::mem::size_of::<LeadingDataPacket>())
        .ok_or_else(|| ConversionError::Limit("leading cache accounting overflow".into()))?;
    for packet in packets {
        bytes = checked_heap_add(bytes, packet.payload.capacity())?;
    }
    Ok(bytes)
}
fn windows_candidate_heap_bytes(windows: &WindowSlots, candidate_len: usize) -> Result<usize> {
    let mut bytes = checked_heap_mul(candidate_len, 64 * 1024)?;
    for window in windows.values() {
        bytes = checked_heap_add(
            bytes,
            checked_heap_mul(window.points.capacity(), std::mem::size_of::<TimedPoint>())?,
        )?;
        bytes = checked_heap_add(
            bytes,
            checked_heap_mul(window.imu.capacity(), std::mem::size_of::<ImuSample>())?,
        )?;
    }
    Ok(bytes)
}
fn string_heap_bytes(value: &String) -> usize {
    value.capacity()
}
fn option_string_heap_bytes(value: &Option<String>) -> usize {
    value.as_ref().map_or(0, String::capacity)
}
fn statistics_heap_bytes(statistics: &ConversionStatistics) -> Result<usize> {
    let mut bytes = std::mem::size_of::<ConversionStatistics>();
    bytes = checked_heap_add(bytes, string_heap_bytes(&statistics.pcap.magic))?;
    bytes = checked_heap_add(bytes, string_heap_bytes(&statistics.filter.blind_boundary))?;
    bytes = checked_heap_add(
        bytes,
        option_string_heap_bytes(&statistics.windows.dropped_first_reason),
    )?;
    bytes = checked_heap_add(
        bytes,
        option_string_heap_bytes(&statistics.windows.dropped_tail_reason),
    )?;
    // u8 键域最多 256 项；按每项 4 KiB 保守核算完整 BTree 节点/边/header。
    bytes = checked_heap_add(bytes, checked_heap_mul(statistics.filter.tags.len(), 4096)?)?;
    Ok(bytes)
}
fn fallible_copy_bytes(value: &[u8]) -> Result<Vec<u8>> {
    let mut copy = Vec::new();
    copy.try_reserve_exact(value.len())
        .map_err(|_| ConversionError::Limit("byte copy allocation failed".into()))?;
    copy.extend_from_slice(value);
    Ok(copy)
}
fn fallible_copy_string(value: &str) -> Result<String> {
    String::from_utf8(fallible_copy_bytes(value.as_bytes())?)
        .map_err(|_| ConversionError::Invalid("internal UTF-8 copy failed".into()))
}
fn fallible_copy_option_string(value: &Option<String>) -> Result<Option<String>> {
    match value {
        Some(text) => Ok(Some(fallible_copy_string(text)?)),
        None => Ok(None),
    }
}
fn fallible_clone_statistics(value: &ConversionStatistics) -> Result<ConversionStatistics> {
    if !value.filter.tags.is_empty() {
        return invalid("pre-discovery filter tag state is not pristine");
    }
    Ok(ConversionStatistics {
        pcap: PcapStatistics {
            magic: fallible_copy_string(&value.pcap.magic)?,
            version_major: value.pcap.version_major,
            version_minor: value.pcap.version_minor,
            snaplen: value.pcap.snaplen,
            linktype: value.pcap.linktype,
            records: value.pcap.records,
            truncated_records: value.pcap.truncated_records,
            first_capture_us: value.pcap.first_capture_us,
            last_capture_us: value.pcap.last_capture_us,
        },
        checksums: value.checksums.clone(),
        endpoints: value.endpoints.clone(),
        point_stream: value.point_stream.clone(),
        imu_stream: value.imu_stream.clone(),
        filter: FilterStatistics {
            input_points: value.filter.input_points,
            output_points: value.filter.output_points,
            zero_placeholder: value.filter.zero_placeholder,
            blind_lt_0_5m: value.filter.blind_lt_0_5m,
            other_rejected: value.filter.other_rejected,
            blind_boundary: fallible_copy_string(&value.filter.blind_boundary)?,
            point_filter_num: value.filter.point_filter_num,
            tags: BTreeMap::new(),
        },
        windows: WindowStatistics {
            emitted: value.windows.emitted,
            dropped_first_incomplete: value.windows.dropped_first_incomplete,
            dropped_first_reason: fallible_copy_option_string(&value.windows.dropped_first_reason)?,
            dropped_tail_incomplete: value.windows.dropped_tail_incomplete,
            dropped_tail_reason: fallible_copy_option_string(&value.windows.dropped_tail_reason)?,
            total_points: value.windows.total_points,
            total_imu: value.windows.total_imu,
            min_points: value.windows.min_points,
            max_points: value.windows.max_points,
            min_imu: value.windows.min_imu,
            max_imu: value.windows.max_imu,
            first_timestamp_ns: value.windows.first_timestamp_ns,
            last_timestamp_ns: value.windows.last_timestamp_ns,
        },
        discovery_packets: value.discovery_packets,
        status_packets: value.status_packets,
        data_crc_zero_packets: value.data_crc_zero_packets,
        command_port: value.command_port,
        device_type: value.device_type,
        raw_anchor_ns: value.raw_anchor_ns,
        raw_first_timestamp_ns: value.raw_first_timestamp_ns,
        raw_last_timestamp_ns: value.raw_last_timestamp_ns,
        time_type: value.time_type,
        max_buffered_windows: value.max_buffered_windows,
        max_buffered_elements: value.max_buffered_elements,
        peak_buffered_windows: value.peak_buffered_windows,
        peak_buffered_elements: value.peak_buffered_elements,
    })
}
fn checked_heap_mul(left: usize, right: usize) -> Result<usize> {
    left.checked_mul(right)
        .ok_or_else(|| ConversionError::Limit("accounted heap multiplication overflow".into()))
}

fn validate_data_payload(payload: &[u8], expected_type: u8) -> Result<()> {
    if payload.len() < LIVox_DATA_HEADER_BYTES
        || payload[0] != 0
        || usize::from(le_u16(payload, 1)?) != payload.len()
        || payload[10] != expected_type
        || payload[11] != 0
        || payload[12..24].iter().any(|byte| *byte != 0)
    {
        return invalid("invalid Livox SDK2 data header");
    }
    let stored_crc = le_u32(payload, 24)?;
    if stored_crc != 0 && crc32_ieee(&payload[28..]) != stored_crc {
        return invalid("invalid non-zero Livox data CRC32/IEEE");
    }
    let time_interval = u64::from(le_u16(payload, 3)?);
    let dots = le_u16(payload, 5)?;
    if dots == 0 {
        return invalid("Livox data packet has zero dots");
    }
    let element_bytes = if expected_type == 1 {
        POINT_BYTES
    } else if expected_type == 0 {
        IMU_BYTES
    } else {
        return invalid("unsupported Livox data type");
    };
    let expected_length = LIVox_DATA_HEADER_BYTES
        .checked_add(
            usize::from(dots)
                .checked_mul(element_bytes)
                .ok_or_else(|| ConversionError::Invalid("Livox element length overflow".into()))?,
        )
        .ok_or_else(|| ConversionError::Invalid("Livox packet length overflow".into()))?;
    if expected_length != payload.len() {
        return invalid("Livox dot count and element size do not close the payload");
    }
    let timestamp_ns = le_u64(payload, 28)?;
    let interval_ns = time_interval
        .checked_mul(100)
        .ok_or_else(|| ConversionError::Invalid("Livox time interval overflow".into()))?
        / u64::from(dots);
    if dots > 1 && interval_ns == 0 {
        return invalid("multi-dot Livox packet has a zero effective timestamp step");
    }
    let data = &payload[LIVox_DATA_HEADER_BYTES..];
    if expected_type == 1 {
        validate_point_elements(data, timestamp_ns, interval_ns)?;
    } else {
        validate_imu_elements(data, timestamp_ns, interval_ns)?;
    }
    Ok(())
}

fn validate_point_elements(data: &[u8], packet_ns: u64, interval_ns: u64) -> Result<()> {
    for (index, raw) in data.chunks_exact(POINT_BYTES).enumerate() {
        effective_time(packet_ns, interval_ns, index)?;
        let x_mm = le_i32(raw, 0)?;
        let y_mm = le_i32(raw, 4)?;
        let z_mm = le_i32(raw, 8)?;
        let _reflectivity = raw[12];
        let _tag = raw[13];
        let _radius_sq = i128::from(x_mm) * i128::from(x_mm)
            + i128::from(y_mm) * i128::from(y_mm)
            + i128::from(z_mm) * i128::from(z_mm);
    }
    Ok(())
}

fn validate_imu_elements(data: &[u8], packet_ns: u64, interval_ns: u64) -> Result<()> {
    for (index, raw) in data.chunks_exact(IMU_BYTES).enumerate() {
        effective_time(packet_ns, interval_ns, index)?;
        let values = [
            le_f32(raw, 0)?,
            le_f32(raw, 4)?,
            le_f32(raw, 8)?,
            le_f32(raw, 12)?,
            le_f32(raw, 16)?,
            le_f32(raw, 20)?,
        ];
        if values.iter().any(|value| !value.is_finite()) {
            return invalid("IMU contains a non-finite value");
        }
        let gyro = &values[..3];
        let accel = &values[3..];
        if gyro.iter().any(|value| f64::from(*value).abs() > 1_000.0)
            || accel.iter().any(|value| {
                let converted = f64::from(*value) * STANDARD_GRAVITY_MPS2;
                !converted.is_finite() || converted.abs() > 1_000.0
            })
        {
            return invalid("IMU conversion exceeds the LIO contract bounds");
        }
    }
    Ok(())
}

fn update_stream_state(
    state: &mut StreamState,
    statistics: &mut SequenceStatistics,
    udp_cnt: u16,
    timestamp_ns: u64,
    dots: u16,
    frame_cnt: u8,
    time_type: u8,
    name: &str,
) -> Result<()> {
    if let Some(previous) = state.previous_udp_cnt {
        let expected = previous.wrapping_add(1);
        if udp_cnt != expected {
            return invalid(format!("{name} udp_cnt is not exactly consecutive"));
        }
        if expected == 0 {
            statistics.wraps = checked_inc(statistics.wraps, "udp_cnt wrap count")?;
        }
    }
    if state
        .previous_timestamp_ns
        .is_some_and(|previous| timestamp_ns <= previous)
    {
        return invalid(format!(
            "{name} packet timestamps are not strictly increasing"
        ));
    }
    statistics.first_udp_cnt.get_or_insert(udp_cnt);
    statistics.last_udp_cnt = Some(udp_cnt);
    statistics.first_timestamp_ns.get_or_insert(timestamp_ns);
    statistics.last_timestamp_ns = Some(timestamp_ns);
    statistics.first_frame_cnt.get_or_insert(frame_cnt);
    statistics.last_frame_cnt = Some(frame_cnt);
    if frame_cnt == 0 {
        statistics.frame_cnt_zero = checked_inc(statistics.frame_cnt_zero, "zero frame_cnt count")?;
    }
    statistics.time_type.get_or_insert(time_type);
    if statistics.time_type != Some(time_type) {
        return invalid(format!("{name} time_type changed within the stream"));
    }
    statistics.packets = checked_inc(statistics.packets, "stream packet count")?;
    statistics.dots = statistics
        .dots
        .checked_add(u64::from(dots))
        .ok_or_else(|| ConversionError::Limit("stream dot count overflow".into()))?;
    state.previous_udp_cnt = Some(udp_cnt);
    state.previous_timestamp_ns = Some(timestamp_ns);
    Ok(())
}

fn update_window_statistics(
    statistics: &mut WindowStatistics,
    timestamp_ns: i64,
    points: u64,
    imu: u64,
) -> Result<()> {
    statistics.emitted = checked_inc(statistics.emitted, "emitted window count")?;
    statistics.total_points = statistics
        .total_points
        .checked_add(points)
        .ok_or_else(|| ConversionError::Limit("window point total overflow".into()))?;
    statistics.total_imu = statistics
        .total_imu
        .checked_add(imu)
        .ok_or_else(|| ConversionError::Limit("window IMU total overflow".into()))?;
    statistics.min_points = Some(statistics.min_points.map_or(points, |old| old.min(points)));
    statistics.max_points = Some(statistics.max_points.map_or(points, |old| old.max(points)));
    statistics.min_imu = Some(statistics.min_imu.map_or(imu, |old| old.min(imu)));
    statistics.max_imu = Some(statistics.max_imu.map_or(imu, |old| old.max(imu)));
    statistics.first_timestamp_ns.get_or_insert(timestamp_ns);
    statistics.last_timestamp_ns = Some(timestamp_ns);
    Ok(())
}

fn read_record_header<R: Read>(reader: &mut R, buffer: &mut [u8; 16]) -> Result<usize> {
    let mut read = 0;
    while read < buffer.len() {
        match reader.read(&mut buffer[read..])? {
            0 if read == 0 => return Ok(0),
            0 => return invalid("truncated PCAP record header"),
            count => read += count,
        }
    }
    Ok(read)
}

fn read_exact_required<R: Read>(reader: &mut R, buffer: &mut [u8], message: &str) -> Result<()> {
    reader.read_exact(buffer).map_err(|error| {
        if error.kind() == io::ErrorKind::UnexpectedEof {
            ConversionError::Invalid(message.into())
        } else {
            ConversionError::Io(error)
        }
    })
}

fn effective_time(packet_ns: u64, interval_ns: u64, index: usize) -> Result<u64> {
    let index = u64::try_from(index)
        .map_err(|_| ConversionError::Invalid("element index does not fit u64".into()))?;
    packet_ns
        .checked_add(index.checked_mul(interval_ns).ok_or_else(|| {
            ConversionError::Invalid("intra-packet timestamp offset overflow".into())
        })?)
        .ok_or_else(|| ConversionError::Invalid("effective element timestamp overflow".into()))
}

fn udp_checksum(source: [u8; 4], destination: [u8; 4], udp: &[u8]) -> Result<u16> {
    let length = u16::try_from(udp.len())
        .map_err(|_| ConversionError::Invalid("UDP datagram exceeds u16 length".into()))?;
    let mut sum = 0_u32;
    checksum_add(&mut sum, &source);
    checksum_add(&mut sum, &destination);
    checksum_add(&mut sum, &[0, 17]);
    checksum_add(&mut sum, &length.to_be_bytes());
    checksum_add(&mut sum, udp);
    Ok(checksum_finish(sum))
}

fn checksum_add(sum: &mut u32, bytes: &[u8]) {
    for pair in bytes.chunks(2) {
        let word = u16::from_be_bytes([pair[0], pair.get(1).copied().unwrap_or(0)]);
        *sum += u32::from(word);
        *sum = (*sum & 0xffff) + (*sum >> 16);
    }
}

fn checksum_finish(mut sum: u32) -> u16 {
    while sum >> 16 != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    sum as u16
}

fn ones_complement_sum(bytes: &[u8]) -> u16 {
    let mut sum = 0_u32;
    checksum_add(&mut sum, bytes);
    checksum_finish(sum)
}

fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;
    for byte in bytes {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn crc32_ieee(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

fn checked_inc(value: u64, label: &str) -> Result<u64> {
    value
        .checked_add(1)
        .ok_or_else(|| ConversionError::Limit(format!("{label} overflow")))
}

fn normalize_zero(value: f64) -> f64 {
    if value == 0.0 {
        0.0
    } else {
        value
    }
}

fn slice_array<const N: usize>(bytes: &[u8], offset: usize) -> Result<[u8; N]> {
    let end = offset
        .checked_add(N)
        .ok_or_else(|| ConversionError::Invalid("byte offset overflow".into()))?;
    bytes
        .get(offset..end)
        .ok_or_else(|| ConversionError::Invalid("truncated byte field".into()))?
        .try_into()
        .map_err(|_| ConversionError::Invalid("invalid fixed-size byte field".into()))
}

fn le_u16(bytes: &[u8], offset: usize) -> Result<u16> {
    Ok(u16::from_le_bytes(slice_array(bytes, offset)?))
}

fn be_u16(bytes: &[u8], offset: usize) -> Result<u16> {
    Ok(u16::from_be_bytes(slice_array(bytes, offset)?))
}

fn le_u32(bytes: &[u8], offset: usize) -> Result<u32> {
    Ok(u32::from_le_bytes(slice_array(bytes, offset)?))
}

fn le_i32(bytes: &[u8], offset: usize) -> Result<i32> {
    Ok(i32::from_le_bytes(slice_array(bytes, offset)?))
}

fn le_u64(bytes: &[u8], offset: usize) -> Result<u64> {
    Ok(u64::from_le_bytes(slice_array(bytes, offset)?))
}

fn le_f32(bytes: &[u8], offset: usize) -> Result<f32> {
    Ok(f32::from_le_bytes(slice_array(bytes, offset)?))
}

fn invalid<T>(message: impl Into<String>) -> Result<T> {
    Err(ConversionError::Invalid(message.into()))
}

fn limit<T>(message: impl Into<String>) -> Result<T> {
    Err(ConversionError::Limit(message.into()))
}
