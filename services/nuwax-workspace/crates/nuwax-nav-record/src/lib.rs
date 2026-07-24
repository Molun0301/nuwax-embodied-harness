//! Navigation topic recording profile and an offline persistence/replay core.
//!
//! The log format is versioned, length-prefixed, and checksummed. Readers validate the
//! complete file and its embedded topic/type schema before exposing any records.

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::collections::{BTreeMap, BTreeSet};
use std::fs::{File, OpenOptions};
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use thiserror::Error;

mod runtime;

pub use runtime::{
    NavRecordInput, NavRecorderConfig, NavRecorderDiagnostics, NavRecorderRuntime, OnExisting,
    RecorderRuntimeError, RecorderState,
};

pub use nuwax_nav_base::NavError as FoundationNavError;
pub use nuwax_nav_types_messages::{
    PoseStamped as FoundationPoseStamped, Twist2 as FoundationTwist2,
};

const MAGIC: &[u8; 8] = b"NUWAXNR\0";
const FORMAT_VERSION: u16 = 2;
const MAX_RECORD_BYTES: usize = 64 * 1024 * 1024;
const MAX_RETAINED_RECORD_BYTES: usize = 64 * 1024 * 1024;
const MAX_RETAINED_RECORDS: usize = 250_000;
const MAX_VALIDATED_RECORDS: usize = 10_000_000;
const MAX_METADATA_BYTES: usize = 255;
const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-6;

/// One topic entry in a navigation recording profile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NavRecordTopic {
    /// Topic name.
    pub topic: String,
    /// Opaque payload type name.
    pub msg_type: String,
}

/// Navigation topic/type schema used to validate recording and replay.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavRecordProfile {
    /// Configuration name used for diagnostics.
    pub name: String,
    /// Compatible output-path field used by callers.
    pub db_path: String,
    /// Allowed topic/type pairs.
    pub topics: Vec<NavRecordTopic>,
}

impl NavRecordProfile {
    /// Default profile matching the original Dimos `NavRecord` inputs.
    #[must_use]
    pub fn default_topics() -> Vec<NavRecordTopic> {
        [
            ("cmd_vel", "geometry_msgs/Twist"),
            ("corrected_odometry", "nav_msgs/Odometry"),
            ("path", "nav_msgs/Path"),
            ("goal_path", "nav_msgs/Path"),
            ("way_point", "geometry_msgs/PointStamped"),
            ("goal", "geometry_msgs/PointStamped"),
            ("stop_movement", "std_msgs/Bool"),
            ("effective_cmd_vel", "geometry_msgs/Twist"),
            ("slow_down", "std_msgs/Int8"),
            ("goal_reached", "std_msgs/Bool"),
            ("terrain_map", "sensor_msgs/PointCloud2"),
            ("global_map", "sensor_msgs/PointCloud2"),
            ("odometry", "nav_msgs/Odometry"),
            ("registered_scan", "sensor_msgs/PointCloud2"),
        ]
        .into_iter()
        .map(|(topic, msg_type)| NavRecordTopic {
            topic: topic.into(),
            msg_type: msg_type.into(),
        })
        .collect()
    }

    /// Constructs the default profile.
    #[must_use]
    pub fn new(name: impl Into<String>, db_path: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            db_path: db_path.into(),
            topics: Self::default_topics(),
        }
    }

    /// Validates that the profile has unique, non-empty topic/type entries.
    pub fn validate(&self) -> Result<(), NavRecordError> {
        if self.topics.is_empty() {
            return Err(NavRecordError::Empty);
        }
        let mut seen = BTreeSet::new();
        for entry in &self.topics {
            if entry.topic.is_empty() {
                return Err(NavRecordError::EmptyTopic);
            }
            if entry.msg_type.is_empty() {
                return Err(NavRecordError::EmptyMsgType(entry.topic.clone()));
            }
            if !seen.insert(entry.topic.clone()) {
                return Err(NavRecordError::DuplicateTopic(entry.topic.clone()));
            }
        }
        Ok(())
    }

    /// Returns all configured topic names.
    #[must_use]
    pub fn topic_names(&self) -> Vec<&str> {
        self.topics
            .iter()
            .map(|entry| entry.topic.as_str())
            .collect()
    }

    /// Returns whether the profile contains `topic`.
    #[must_use]
    pub fn contains(&self, topic: &str) -> bool {
        self.topics.iter().any(|entry| entry.topic == topic)
    }

    /// Returns the number of configured topics.
    #[must_use]
    pub fn len(&self) -> usize {
        self.topics.len()
    }

    /// Returns whether the profile has no topics.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.topics.is_empty()
    }
}

/// A single validated navigation record with an opaque typed payload.
#[derive(Debug, Clone, PartialEq)]
pub struct NavRecord {
    /// Source timestamp in nanoseconds.
    pub timestamp_ns: u64,
    /// Recorded topic.
    pub topic: String,
    /// Declared payload type.
    pub msg_type: String,
    /// Optional source frame metadata.
    pub frame_id: Option<String>,
    /// Optional world/reference-frame pose metadata.
    pub pose: Option<NavRecordPose>,
    /// Opaque encoded message bytes.
    pub payload: Vec<u8>,
}

/// Pose metadata anchored to a named reference frame.
#[derive(Debug, Clone, PartialEq)]
pub struct NavRecordPose {
    /// Reference frame containing this pose, such as `world` or `map`.
    pub reference_frame: String,
    /// Translation `(x, y, z)` in metres.
    pub position_xyz: [f64; 3],
    /// Quaternion `(x, y, z, w)`.
    pub orientation_xyzw: [f64; 4],
}

impl NavRecord {
    /// Constructs a record without frame metadata.
    #[must_use]
    pub fn new(
        timestamp_ns: u64,
        topic: impl Into<String>,
        msg_type: impl Into<String>,
        payload: impl Into<Vec<u8>>,
    ) -> Self {
        Self {
            timestamp_ns,
            topic: topic.into(),
            msg_type: msg_type.into(),
            frame_id: None,
            pose: None,
            payload: payload.into(),
        }
    }
}

/// Optional replay predicates. All populated predicates must match.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct NavRecordFilter {
    /// Exact topic match.
    pub topic: Option<String>,
    /// Exact message-type match.
    pub msg_type: Option<String>,
    /// Inclusive minimum timestamp.
    pub from_timestamp_ns: Option<u64>,
    /// Inclusive maximum timestamp.
    pub to_timestamp_ns: Option<u64>,
}

impl NavRecordFilter {
    fn matches(&self, record: &NavRecord) -> bool {
        self.topic
            .as_deref()
            .is_none_or(|value| value == record.topic)
            && self
                .msg_type
                .as_deref()
                .is_none_or(|value| value == record.msg_type)
            && self
                .from_timestamp_ns
                .is_none_or(|value| record.timestamp_ns >= value)
            && self
                .to_timestamp_ns
                .is_none_or(|value| record.timestamp_ns <= value)
    }
}

/// Metadata returned after a complete validation pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavRecordSummary {
    /// On-disk format version.
    pub format_version: u16,
    /// Number of valid records.
    pub record_count: usize,
}

/// Buffered writer for the versioned navigation record log.
pub struct NavRecordWriter {
    writer: BufWriter<File>,
    schema: BTreeMap<String, String>,
    _path_lock: SidecarLock,
}

impl NavRecordWriter {
    /// Creates or truncates a log and writes its embedded profile schema.
    pub fn create(
        path: impl AsRef<Path>,
        profile: &NavRecordProfile,
    ) -> Result<Self, NavRecordError> {
        profile.validate()?;
        let path = path.as_ref();
        let path_lock = SidecarLock::acquire(path)?;
        Self::create_locked(path, profile, path_lock)
    }

    pub(crate) fn create_locked(
        path: &Path,
        profile: &NavRecordProfile,
        path_lock: SidecarLock,
    ) -> Result<Self, NavRecordError> {
        profile.validate()?;
        let file = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(path)?;
        lock_file_exclusive(&file)?;
        file.set_len(0)?;
        let mut writer = BufWriter::new(file);
        write_header(&mut writer, profile)?;
        Ok(Self {
            writer,
            schema: owned_schema(profile),
            _path_lock: path_lock,
        })
    }

    /// Opens a fully validated existing log for appending.
    pub fn open_append(
        path: impl AsRef<Path>,
        profile: &NavRecordProfile,
    ) -> Result<Self, NavRecordError> {
        profile.validate()?;
        let path = path.as_ref();
        let path_lock = SidecarLock::acquire(path)?;
        Self::open_append_locked(path, profile, path_lock)
    }

    pub(crate) fn open_append_locked(
        path: &Path,
        profile: &NavRecordProfile,
        path_lock: SidecarLock,
    ) -> Result<Self, NavRecordError> {
        profile.validate()?;
        let file = OpenOptions::new().read(true).append(true).open(path)?;
        lock_file_exclusive(&file)?;
        NavRecordReader::from_file(file.try_clone()?, profile)?;
        Ok(Self {
            writer: BufWriter::new(file),
            schema: owned_schema(profile),
            _path_lock: path_lock,
        })
    }

    /// Appends one record after checking its topic/type against the profile.
    pub fn append(&mut self, record: &NavRecord) -> Result<(), NavRecordError> {
        validate_record(&self.schema, record)?;
        let body = encode_record(record)?;
        let record_len = body
            .len()
            .checked_add(4)
            .ok_or(NavRecordError::RecordTooLarge(body.len()))?;
        if record_len > MAX_RECORD_BYTES {
            return Err(NavRecordError::RecordTooLarge(record_len));
        }
        let mut frame = Vec::with_capacity(record_len + 4);
        write_u32(&mut frame, to_u32(record_len)?)?;
        frame.extend_from_slice(&body);
        write_u32(&mut frame, checksum(&body))?;
        self.writer.write_all(&frame)?;
        Ok(())
    }

    /// Flushes buffered bytes to the underlying file.
    pub fn flush(&mut self) -> Result<(), NavRecordError> {
        self.writer.flush()?;
        Ok(())
    }
}

pub(crate) struct SidecarLock {
    _file: File,
}

impl SidecarLock {
    pub(crate) fn acquire(recording_path: &Path) -> std::io::Result<Self> {
        let file = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(false)
            .open(sidecar_lock_path(recording_path))?;
        lock_file_exclusive(&file)?;
        Ok(Self { _file: file })
    }
}

fn sidecar_lock_path(recording_path: &Path) -> PathBuf {
    let mut lock_path = recording_path.as_os_str().to_os_string();
    lock_path.push(".lock");
    PathBuf::from(lock_path)
}

/// Fully validated in-memory view of a navigation record log.
pub struct NavRecordReader {
    records: Vec<NavRecord>,
}

impl NavRecordReader {
    /// Opens and validates the complete file before returning any data.
    pub fn open(
        path: impl AsRef<Path>,
        profile: &NavRecordProfile,
    ) -> Result<Self, NavRecordError> {
        profile.validate()?;
        Self::from_file(File::open(path)?, profile)
    }

    fn from_file(file: File, profile: &NavRecordProfile) -> Result<Self, NavRecordError> {
        let mut reader = BufReader::new(file);
        read_and_validate_header(&mut reader, profile)?;
        let schema = owned_schema(profile);
        let (_, records) = scan_records(&mut reader, &schema, ScanLimits::retained_reader(), true)?;
        Ok(Self { records })
    }

    /// Validates a complete file without retaining its reader.
    pub fn validate(
        path: impl AsRef<Path>,
        profile: &NavRecordProfile,
    ) -> Result<NavRecordSummary, NavRecordError> {
        profile.validate()?;
        let mut reader = BufReader::new(File::open(path)?);
        read_and_validate_header(&mut reader, profile)?;
        let schema = owned_schema(profile);
        let (record_count, records) =
            scan_records(&mut reader, &schema, ScanLimits::validation(), false)?;
        debug_assert!(records.is_empty());
        Ok(NavRecordSummary {
            format_version: FORMAT_VERSION,
            record_count,
        })
    }

    /// Returns all records in their original append order.
    #[must_use]
    pub fn records(&self) -> &[NavRecord] {
        &self.records
    }

    /// Replays matching records in original append order and returns the emitted count.
    pub fn replay(&self, filter: &NavRecordFilter, mut emit: impl FnMut(&NavRecord)) -> usize {
        let mut count = 0;
        for record in self.filter(filter) {
            emit(record);
            count += 1;
        }
        count
    }

    /// Iterates over records matching `filter`, preserving append order.
    pub fn filter<'a>(
        &'a self,
        filter: &'a NavRecordFilter,
    ) -> impl Iterator<Item = &'a NavRecord> + 'a {
        self.records
            .iter()
            .filter(move |record| filter.matches(record))
    }
}

#[cfg(unix)]
#[allow(unsafe_code)]
fn lock_file_exclusive(file: &File) -> std::io::Result<()> {
    use std::os::fd::AsRawFd;
    use std::os::raw::c_int;

    const LOCK_EX: c_int = 2;

    unsafe extern "C" {
        fn flock(fd: c_int, operation: c_int) -> c_int;
    }

    loop {
        // SAFETY: `file` owns a live descriptor for the duration of this call. `flock`
        // neither retains the pointer nor accesses Rust-managed memory.
        if unsafe { flock(file.as_raw_fd(), LOCK_EX) } == 0 {
            return Ok(());
        }
        let error = std::io::Error::last_os_error();
        if error.kind() != std::io::ErrorKind::Interrupted {
            return Err(error);
        }
    }
}

#[cfg(not(unix))]
fn lock_file_exclusive(_file: &File) -> std::io::Result<()> {
    Err(std::io::Error::new(
        std::io::ErrorKind::Unsupported,
        "exclusive navigation-record writer locks require a Unix target",
    ))
}

/// Errors produced by profile validation, persistence, and replay.
#[derive(Debug, Error)]
pub enum NavRecordError {
    /// Topic list is empty.
    #[error("NavRecord profile has no topics")]
    Empty,
    /// Topic name is empty.
    #[error("topic name cannot be empty")]
    EmptyTopic,
    /// Message type is empty.
    #[error("msg_type cannot be empty for topic '{0}'")]
    EmptyMsgType(String),
    /// Topic is duplicated.
    #[error("duplicate topic '{0}' in NavRecord profile")]
    DuplicateTopic(String),
    /// Underlying file operation failed.
    #[error("navigation record I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// File magic is not recognized.
    #[error("invalid navigation record file magic")]
    InvalidMagic,
    /// File format version is unsupported.
    #[error("unsupported navigation record format version {0}")]
    UnsupportedVersion(u16),
    /// File ended before a complete header or record was available.
    #[error("truncated navigation record file")]
    Truncated,
    /// Embedded profile does not match the requested profile.
    #[error("navigation record profile schema mismatch")]
    ProfileMismatch,
    /// Record topic is not present in the profile.
    #[error("unknown navigation record topic '{0}'")]
    UnknownTopic(String),
    /// Record type differs from the profile type.
    #[error("message type mismatch for topic '{topic}': expected '{expected}', got '{actual}'")]
    MessageTypeMismatch {
        /// Topic whose type did not match.
        topic: String,
        /// Profile type.
        expected: String,
        /// Record type.
        actual: String,
    },
    /// Length exceeds the bounded log format.
    #[error("navigation record length {0} exceeds the supported bound")]
    RecordTooLarge(usize),
    /// Retained reader record count exceeds its memory-safety bound.
    #[error("navigation recording contains more than {0} records")]
    TooManyRecords(usize),
    /// Retained reader aggregate framed bytes exceed its memory-safety bound.
    #[error("navigation recording retained bytes exceed {0}")]
    RecordingTooLarge(usize),
    /// A string cannot fit in the on-disk length field.
    #[error("navigation record string is too long")]
    StringTooLong,
    /// Record checksum did not match its content.
    #[error("navigation record checksum mismatch")]
    CorruptChecksum,
    /// Record body is structurally invalid.
    #[error("invalid navigation record body")]
    InvalidRecord,
}

fn owned_schema(profile: &NavRecordProfile) -> BTreeMap<String, String> {
    profile
        .topics
        .iter()
        .map(|entry| (entry.topic.clone(), entry.msg_type.clone()))
        .collect()
}

fn validate_record(
    schema: &BTreeMap<String, String>,
    record: &NavRecord,
) -> Result<(), NavRecordError> {
    let Some(expected) = schema.get(&record.topic) else {
        return Err(NavRecordError::UnknownTopic(record.topic.clone()));
    };
    if expected != &record.msg_type {
        return Err(NavRecordError::MessageTypeMismatch {
            topic: record.topic.clone(),
            expected: expected.clone(),
            actual: record.msg_type.clone(),
        });
    }
    if let Some(frame_id) = &record.frame_id {
        validate_frame_metadata(frame_id)?;
    }
    if let Some(pose) = &record.pose {
        validate_frame_metadata(&pose.reference_frame)?;
        if pose
            .position_xyz
            .iter()
            .chain(pose.orientation_xyzw.iter())
            .any(|value| !value.is_finite())
        {
            return Err(NavRecordError::InvalidRecord);
        }
        let norm_squared = pose
            .orientation_xyzw
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        if (norm_squared - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
            return Err(NavRecordError::InvalidRecord);
        }
    }
    Ok(())
}

fn validate_frame_metadata(frame: &str) -> Result<(), NavRecordError> {
    if frame.is_empty()
        || frame.len() > MAX_METADATA_BYTES
        || frame.starts_with('/')
        || frame.ends_with('/')
        || frame.contains("//")
        || frame
            .chars()
            .any(|character| character.is_control() || character.is_whitespace())
    {
        return Err(NavRecordError::InvalidRecord);
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct ScanLimits {
    max_records: usize,
    max_retained_bytes: usize,
}

impl ScanLimits {
    const fn retained_reader() -> Self {
        Self {
            max_records: MAX_RETAINED_RECORDS,
            max_retained_bytes: MAX_RETAINED_RECORD_BYTES,
        }
    }

    const fn validation() -> Self {
        Self {
            max_records: MAX_VALIDATED_RECORDS,
            max_retained_bytes: usize::MAX,
        }
    }
}

fn scan_records(
    reader: &mut impl Read,
    schema: &BTreeMap<String, String>,
    limits: ScanLimits,
    retain: bool,
) -> Result<(usize, Vec<NavRecord>), NavRecordError> {
    let mut count = 0usize;
    let mut retained_bytes = 0usize;
    let mut records = Vec::new();
    while let Some(record_len) = read_optional_u32(reader)? {
        let record_len = record_len as usize;
        if !(4..=MAX_RECORD_BYTES).contains(&record_len) {
            return Err(NavRecordError::RecordTooLarge(record_len));
        }
        count = count
            .checked_add(1)
            .ok_or(NavRecordError::TooManyRecords(limits.max_records))?;
        if count > limits.max_records {
            return Err(NavRecordError::TooManyRecords(limits.max_records));
        }
        if retain {
            retained_bytes = retained_bytes
                .checked_add(record_len)
                .ok_or(NavRecordError::RecordingTooLarge(limits.max_retained_bytes))?;
            if retained_bytes > limits.max_retained_bytes {
                return Err(NavRecordError::RecordingTooLarge(limits.max_retained_bytes));
            }
        }

        let body_len = record_len - 4;
        let mut body = vec![0; body_len];
        read_exact_log(reader, &mut body)?;
        let expected = read_u32_log(reader)?;
        if checksum(&body) != expected {
            return Err(NavRecordError::CorruptChecksum);
        }
        let record = decode_record(&body)?;
        validate_record(schema, &record)?;
        if retain {
            records.push(record);
        }
    }
    Ok((count, records))
}

fn write_header(writer: &mut impl Write, profile: &NavRecordProfile) -> Result<(), NavRecordError> {
    let mut body = Vec::new();
    write_string(&mut body, &profile.name)?;
    let topic_count =
        u16::try_from(profile.topics.len()).map_err(|_| NavRecordError::StringTooLong)?;
    write_u16(&mut body, topic_count)?;
    for entry in &profile.topics {
        write_string(&mut body, &entry.topic)?;
        write_string(&mut body, &entry.msg_type)?;
    }
    writer.write_all(MAGIC)?;
    write_u16(writer, FORMAT_VERSION)?;
    write_u32(writer, to_u32(body.len())?)?;
    writer.write_all(&body)?;
    write_u32(writer, checksum(&body))?;
    Ok(())
}

fn read_and_validate_header(
    reader: &mut impl Read,
    profile: &NavRecordProfile,
) -> Result<(), NavRecordError> {
    let mut magic = [0; 8];
    read_exact_log(reader, &mut magic)?;
    if &magic != MAGIC {
        return Err(NavRecordError::InvalidMagic);
    }
    let version = read_u16_log(reader)?;
    if version != FORMAT_VERSION {
        return Err(NavRecordError::UnsupportedVersion(version));
    }
    let body_len = read_u32_log(reader)? as usize;
    if body_len > MAX_RECORD_BYTES {
        return Err(NavRecordError::RecordTooLarge(body_len));
    }
    let mut body = vec![0; body_len];
    read_exact_log(reader, &mut body)?;
    let expected_checksum = read_u32_log(reader)?;
    if checksum(&body) != expected_checksum {
        return Err(NavRecordError::CorruptChecksum);
    }
    let mut cursor = SliceReader::new(&body);
    let _name = cursor.string()?;
    let topic_count = cursor.u16()? as usize;
    let mut embedded = BTreeMap::new();
    for _ in 0..topic_count {
        let topic = cursor.string()?;
        let msg_type = cursor.string()?;
        if embedded.insert(topic, msg_type).is_some() {
            return Err(NavRecordError::InvalidRecord);
        }
    }
    if !cursor.is_empty() || embedded != owned_schema(profile) {
        return Err(NavRecordError::ProfileMismatch);
    }
    Ok(())
}

fn encode_record(record: &NavRecord) -> Result<Vec<u8>, NavRecordError> {
    let mut body = Vec::new();
    body.extend_from_slice(&record.timestamp_ns.to_le_bytes());
    write_string(&mut body, &record.topic)?;
    write_string(&mut body, &record.msg_type)?;
    match &record.frame_id {
        Some(frame_id) => {
            body.push(1);
            write_string(&mut body, frame_id)?;
        }
        None => body.push(0),
    }
    write_u32(&mut body, to_u32(record.payload.len())?)?;
    body.extend_from_slice(&record.payload);
    match &record.pose {
        Some(pose) => {
            body.push(1);
            write_string(&mut body, &pose.reference_frame)?;
            for value in pose.position_xyz.into_iter().chain(pose.orientation_xyzw) {
                body.extend_from_slice(&value.to_le_bytes());
            }
        }
        None => body.push(0),
    }
    Ok(body)
}

fn decode_record(body: &[u8]) -> Result<NavRecord, NavRecordError> {
    let mut cursor = SliceReader::new(body);
    let timestamp_ns = cursor.u64()?;
    let topic = cursor.string()?;
    let msg_type = cursor.string()?;
    let frame_id = match cursor.u8()? {
        0 => None,
        1 => Some(cursor.string()?),
        _ => return Err(NavRecordError::InvalidRecord),
    };
    let payload_len = cursor.u32()? as usize;
    let payload = cursor.take(payload_len)?.to_vec();
    let pose = match cursor.u8()? {
        0 => None,
        1 => {
            let reference_frame = cursor.string()?;
            let mut position_xyz = [0.0; 3];
            for value in &mut position_xyz {
                *value = cursor.f64()?;
            }
            let mut orientation_xyzw = [0.0; 4];
            for value in &mut orientation_xyzw {
                *value = cursor.f64()?;
            }
            Some(NavRecordPose {
                reference_frame,
                position_xyz,
                orientation_xyzw,
            })
        }
        _ => return Err(NavRecordError::InvalidRecord),
    };
    if !cursor.is_empty() {
        return Err(NavRecordError::InvalidRecord);
    }
    Ok(NavRecord {
        timestamp_ns,
        topic,
        msg_type,
        frame_id,
        pose,
        payload,
    })
}

struct SliceReader<'a> {
    remaining: &'a [u8],
}

impl<'a> SliceReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { remaining: bytes }
    }

    fn take(&mut self, len: usize) -> Result<&'a [u8], NavRecordError> {
        if len > self.remaining.len() {
            return Err(NavRecordError::InvalidRecord);
        }
        let (value, remaining) = self.remaining.split_at(len);
        self.remaining = remaining;
        Ok(value)
    }

    fn u8(&mut self) -> Result<u8, NavRecordError> {
        Ok(self.take(1)?[0])
    }

    fn u16(&mut self) -> Result<u16, NavRecordError> {
        let bytes = self.take(2)?;
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    fn u32(&mut self) -> Result<u32, NavRecordError> {
        let bytes = self.take(4)?;
        Ok(u32::from_le_bytes(
            bytes
                .try_into()
                .map_err(|_| NavRecordError::InvalidRecord)?,
        ))
    }

    fn u64(&mut self) -> Result<u64, NavRecordError> {
        let bytes = self.take(8)?;
        Ok(u64::from_le_bytes(
            bytes
                .try_into()
                .map_err(|_| NavRecordError::InvalidRecord)?,
        ))
    }

    fn f64(&mut self) -> Result<f64, NavRecordError> {
        let bytes = self.take(8)?;
        Ok(f64::from_le_bytes(
            bytes
                .try_into()
                .map_err(|_| NavRecordError::InvalidRecord)?,
        ))
    }

    fn string(&mut self) -> Result<String, NavRecordError> {
        let len = self.u16()? as usize;
        String::from_utf8(self.take(len)?.to_vec()).map_err(|_| NavRecordError::InvalidRecord)
    }

    fn is_empty(&self) -> bool {
        self.remaining.is_empty()
    }
}

fn write_string(writer: &mut impl Write, value: &str) -> Result<(), NavRecordError> {
    let len = u16::try_from(value.len()).map_err(|_| NavRecordError::StringTooLong)?;
    write_u16(writer, len)?;
    writer.write_all(value.as_bytes())?;
    Ok(())
}

fn read_optional_u32(reader: &mut impl Read) -> Result<Option<u32>, NavRecordError> {
    let mut first = [0; 1];
    match reader.read(&mut first) {
        Ok(0) => return Ok(None),
        Ok(1) => {}
        Ok(_) => unreachable!(),
        Err(error) => return Err(NavRecordError::Io(error)),
    }
    let mut rest = [0; 3];
    read_exact_log(reader, &mut rest)?;
    Ok(Some(u32::from_le_bytes([
        first[0], rest[0], rest[1], rest[2],
    ])))
}

fn read_exact_log(reader: &mut impl Read, bytes: &mut [u8]) -> Result<(), NavRecordError> {
    reader.read_exact(bytes).map_err(|error| {
        if error.kind() == std::io::ErrorKind::UnexpectedEof {
            NavRecordError::Truncated
        } else {
            NavRecordError::Io(error)
        }
    })
}

fn read_u16_log(reader: &mut impl Read) -> Result<u16, NavRecordError> {
    let mut bytes = [0; 2];
    read_exact_log(reader, &mut bytes)?;
    Ok(u16::from_le_bytes(bytes))
}

fn read_u32_log(reader: &mut impl Read) -> Result<u32, NavRecordError> {
    let mut bytes = [0; 4];
    read_exact_log(reader, &mut bytes)?;
    Ok(u32::from_le_bytes(bytes))
}

fn write_u16(writer: &mut impl Write, value: u16) -> Result<(), NavRecordError> {
    writer.write_all(&value.to_le_bytes())?;
    Ok(())
}

fn write_u32(writer: &mut impl Write, value: u32) -> Result<(), NavRecordError> {
    writer.write_all(&value.to_le_bytes())?;
    Ok(())
}

fn to_u32(value: usize) -> Result<u32, NavRecordError> {
    u32::try_from(value).map_err(|_| NavRecordError::RecordTooLarge(value))
}

fn checksum(bytes: &[u8]) -> u32 {
    let mut hash = 0x811c_9dc5_u32;
    for byte in bytes {
        hash ^= u32::from(*byte);
        hash = hash.wrapping_mul(0x0100_0193);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::mpsc::{self, RecvTimeoutError};
    use std::time::Duration;

    static NEXT_PATH: AtomicU64 = AtomicU64::new(0);

    struct TempLog(std::path::PathBuf);

    impl TempLog {
        fn new() -> Self {
            let id = NEXT_PATH.fetch_add(1, Ordering::Relaxed);
            Self(
                std::env::temp_dir()
                    .join(format!("nuwax-nav-record-{}-{id}.bin", std::process::id())),
            )
        }
    }

    impl Drop for TempLog {
        fn drop(&mut self) {
            let _ = fs::remove_file(&self.0);
            let _ = fs::remove_file(sidecar_lock_path(&self.0));
        }
    }

    fn profile() -> NavRecordProfile {
        NavRecordProfile {
            name: "test".into(),
            db_path: "unused.bin".into(),
            topics: vec![
                NavRecordTopic {
                    topic: "cmd_vel".into(),
                    msg_type: "geometry_msgs/Twist".into(),
                },
                NavRecordTopic {
                    topic: "path".into(),
                    msg_type: "nav_msgs/Path".into(),
                },
            ],
        }
    }

    fn record(timestamp_ns: u64, topic: &str, msg_type: &str, payload: &[u8]) -> NavRecord {
        NavRecord::new(timestamp_ns, topic, msg_type, payload)
    }

    fn write_unchecked_log(path: &Path, profile: &NavRecordProfile, record: &NavRecord) {
        let mut file = BufWriter::new(File::create(path).unwrap());
        write_header(&mut file, profile).unwrap();
        let body = encode_record(record).unwrap();
        write_u32(&mut file, u32::try_from(body.len() + 4).unwrap()).unwrap();
        file.write_all(&body).unwrap();
        write_u32(&mut file, checksum(&body)).unwrap();
        file.flush().unwrap();
    }

    #[test]
    fn default_topics_match_python_parity() {
        let topics = NavRecordProfile::default_topics();
        let names: Vec<&str> = topics.iter().map(|entry| entry.topic.as_str()).collect();
        for required in [
            "cmd_vel",
            "corrected_odometry",
            "path",
            "goal_path",
            "way_point",
            "goal",
            "stop_movement",
            "effective_cmd_vel",
            "slow_down",
            "goal_reached",
            "terrain_map",
            "global_map",
            "odometry",
            "registered_scan",
        ] {
            assert!(names.contains(&required), "missing topic {required}");
        }
    }

    #[test]
    fn profile_validation_rejects_duplicates_and_empty_entries() {
        let mut duplicate = profile();
        duplicate.topics.push(duplicate.topics[0].clone());
        assert!(matches!(
            duplicate.validate(),
            Err(NavRecordError::DuplicateTopic(_))
        ));

        let mut empty_type = profile();
        empty_type.topics[0].msg_type.clear();
        assert!(matches!(
            empty_type.validate(),
            Err(NavRecordError::EmptyMsgType(_))
        ));
    }

    #[test]
    fn real_file_roundtrip_append_order_and_filter() {
        let temp = TempLog::new();
        let profile = profile();
        {
            let mut writer = NavRecordWriter::create(&temp.0, &profile).unwrap();
            let mut first = record(10, "cmd_vel", "geometry_msgs/Twist", &[1, 2]);
            first.frame_id = Some("base_link".into());
            writer.append(&first).unwrap();
            writer
                .append(&record(20, "path", "nav_msgs/Path", &[3]))
                .unwrap();
            writer.flush().unwrap();
        }
        {
            let mut writer = NavRecordWriter::open_append(&temp.0, &profile).unwrap();
            writer
                .append(&record(30, "cmd_vel", "geometry_msgs/Twist", &[4, 5]))
                .unwrap();
            writer.flush().unwrap();
        }

        let summary = NavRecordReader::validate(&temp.0, &profile).unwrap();
        assert_eq!(summary.format_version, 2);
        assert_eq!(summary.record_count, 3);
        let reader = NavRecordReader::open(&temp.0, &profile).unwrap();
        assert_eq!(
            reader
                .records()
                .iter()
                .map(|record| record.timestamp_ns)
                .collect::<Vec<_>>(),
            vec![10, 20, 30]
        );
        assert_eq!(reader.records()[0].frame_id.as_deref(), Some("base_link"));

        let filter = NavRecordFilter {
            topic: Some("cmd_vel".into()),
            from_timestamp_ns: Some(11),
            ..NavRecordFilter::default()
        };
        let mut replayed = Vec::new();
        assert_eq!(
            reader.replay(&filter, |entry| replayed.push(entry.payload.clone())),
            1
        );
        assert_eq!(replayed, vec![vec![4, 5]]);
    }

    #[test]
    fn competing_create_waits_before_truncating_and_lock_releases_on_drop() {
        let temp = TempLog::new();
        let profile = profile();
        let mut first = NavRecordWriter::create(&temp.0, &profile).unwrap();
        first
            .append(&record(1, "path", "nav_msgs/Path", &[1, 2, 3]))
            .unwrap();
        first.flush().unwrap();
        let bytes_before_competitor = fs::read(&temp.0).unwrap();

        let path = temp.0.clone();
        let competitor_profile = profile.clone();
        let (sender, receiver) = mpsc::sync_channel(1);
        let competitor = std::thread::spawn(move || {
            sender
                .send(NavRecordWriter::create(path, &competitor_profile))
                .unwrap();
        });

        assert!(matches!(
            receiver.recv_timeout(Duration::from_millis(100)),
            Err(RecvTimeoutError::Timeout)
        ));
        assert_eq!(fs::read(&temp.0).unwrap(), bytes_before_competitor);

        drop(first);
        let second = receiver
            .recv_timeout(Duration::from_secs(2))
            .expect("competing create should acquire the released lock")
            .unwrap();
        drop(second);
        competitor.join().unwrap();

        assert_eq!(
            NavRecordReader::validate(&temp.0, &profile)
                .unwrap()
                .record_count,
            0
        );
    }

    #[test]
    fn competing_append_waits_then_validates_latest_locked_contents() {
        let temp = TempLog::new();
        let profile = profile();
        let mut first = NavRecordWriter::create(&temp.0, &profile).unwrap();

        let path = temp.0.clone();
        let competitor_profile = profile.clone();
        let (sender, receiver) = mpsc::sync_channel(1);
        let competitor = std::thread::spawn(move || {
            sender
                .send(NavRecordWriter::open_append(path, &competitor_profile))
                .unwrap();
        });

        assert!(matches!(
            receiver.recv_timeout(Duration::from_millis(100)),
            Err(RecvTimeoutError::Timeout)
        ));
        first
            .append(&record(1, "path", "nav_msgs/Path", &[1]))
            .unwrap();
        first.flush().unwrap();
        drop(first);

        let mut second = receiver
            .recv_timeout(Duration::from_secs(2))
            .expect("competing append should acquire the released lock")
            .unwrap();
        second
            .append(&record(2, "path", "nav_msgs/Path", &[2]))
            .unwrap();
        second.flush().unwrap();
        drop(second);
        competitor.join().unwrap();

        let reader = NavRecordReader::open(&temp.0, &profile).unwrap();
        assert_eq!(
            reader
                .records()
                .iter()
                .map(|record| record.timestamp_ns)
                .collect::<Vec<_>>(),
            vec![1, 2]
        );
    }

    #[test]
    fn writer_rejects_unknown_topic_and_type() {
        let temp = TempLog::new();
        let profile = profile();
        let mut writer = NavRecordWriter::create(&temp.0, &profile).unwrap();
        assert!(matches!(
            writer.append(&record(1, "unknown", "nav_msgs/Path", &[])),
            Err(NavRecordError::UnknownTopic(_))
        ));
        assert!(matches!(
            writer.append(&record(1, "path", "wrong/Type", &[])),
            Err(NavRecordError::MessageTypeMismatch { .. })
        ));
    }

    #[test]
    fn writer_and_reader_reject_invalid_frame_and_pose_metadata() {
        let temp = TempLog::new();
        let profile = profile();
        let mut writer = NavRecordWriter::create(&temp.0, &profile).unwrap();

        let mut empty_frame = record(1, "path", "nav_msgs/Path", &[]);
        empty_frame.frame_id = Some(String::new());
        assert!(matches!(
            writer.append(&empty_frame),
            Err(NavRecordError::InvalidRecord)
        ));

        let mut non_finite_pose = record(2, "path", "nav_msgs/Path", &[]);
        non_finite_pose.pose = Some(NavRecordPose {
            reference_frame: "map".into(),
            position_xyz: [f64::NAN, 0.0, 0.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        });
        assert!(matches!(
            writer.append(&non_finite_pose),
            Err(NavRecordError::InvalidRecord)
        ));

        let mut invalid_quaternion = record(3, "path", "nav_msgs/Path", &[]);
        invalid_quaternion.pose = Some(NavRecordPose {
            reference_frame: "map".into(),
            position_xyz: [0.0, 0.0, 0.0],
            orientation_xyzw: [0.0; 4],
        });
        assert!(matches!(
            writer.append(&invalid_quaternion),
            Err(NavRecordError::InvalidRecord)
        ));

        write_unchecked_log(&temp.0, &profile, &invalid_quaternion);
        assert!(matches!(
            NavRecordReader::open(&temp.0, &profile),
            Err(NavRecordError::InvalidRecord)
        ));
    }

    #[test]
    fn validation_streams_records_while_retained_reader_is_bounded() {
        let temp = TempLog::new();
        let profile = profile();
        {
            let mut writer = NavRecordWriter::create(&temp.0, &profile).unwrap();
            for timestamp in 1..=3 {
                writer
                    .append(&record(timestamp, "path", "nav_msgs/Path", &[]))
                    .unwrap();
            }
            writer.flush().unwrap();
        }

        let schema = owned_schema(&profile);
        let mut retained = BufReader::new(File::open(&temp.0).unwrap());
        read_and_validate_header(&mut retained, &profile).unwrap();
        assert!(matches!(
            scan_records(
                &mut retained,
                &schema,
                ScanLimits {
                    max_records: 2,
                    max_retained_bytes: usize::MAX,
                },
                true,
            ),
            Err(NavRecordError::TooManyRecords(2))
        ));

        let mut streaming = BufReader::new(File::open(&temp.0).unwrap());
        read_and_validate_header(&mut streaming, &profile).unwrap();
        let (count, records) = scan_records(
            &mut streaming,
            &schema,
            ScanLimits {
                max_records: 3,
                max_retained_bytes: 0,
            },
            false,
        )
        .unwrap();
        assert_eq!(count, 3);
        assert!(records.is_empty());
        assert_eq!(
            NavRecordReader::validate(&temp.0, &profile)
                .unwrap()
                .record_count,
            3
        );
    }

    #[test]
    fn reader_rejects_checksummed_unknown_topic_and_type() {
        let profile = profile();
        let unknown_topic = TempLog::new();
        write_unchecked_log(
            &unknown_topic.0,
            &profile,
            &record(1, "nope", "nav_msgs/Path", &[]),
        );
        assert!(matches!(
            NavRecordReader::open(&unknown_topic.0, &profile),
            Err(NavRecordError::UnknownTopic(_))
        ));

        let wrong_type = TempLog::new();
        write_unchecked_log(
            &wrong_type.0,
            &profile,
            &record(1, "path", "wrong/Type", &[]),
        );
        assert!(matches!(
            NavRecordReader::open(&wrong_type.0, &profile),
            Err(NavRecordError::MessageTypeMismatch { .. })
        ));
    }

    #[test]
    fn reader_rejects_profile_mismatch_corruption_and_truncation() {
        let valid = TempLog::new();
        let profile = profile();
        {
            let mut writer = NavRecordWriter::create(&valid.0, &profile).unwrap();
            writer
                .append(&record(1, "path", "nav_msgs/Path", &[9, 8, 7]))
                .unwrap();
            writer.flush().unwrap();
        }

        let mut mismatched = profile.clone();
        mismatched.topics[1].msg_type = "wrong/Type".into();
        assert!(matches!(
            NavRecordReader::open(&valid.0, &mismatched),
            Err(NavRecordError::ProfileMismatch)
        ));

        let bytes = fs::read(&valid.0).unwrap();
        let corrupt = TempLog::new();
        let mut corrupt_bytes = bytes.clone();
        let index = corrupt_bytes.len() - 5;
        corrupt_bytes[index] ^= 0x80;
        fs::write(&corrupt.0, corrupt_bytes).unwrap();
        assert!(matches!(
            NavRecordReader::open(&corrupt.0, &profile),
            Err(NavRecordError::CorruptChecksum)
        ));

        let truncated = TempLog::new();
        fs::write(&truncated.0, &bytes[..bytes.len() - 2]).unwrap();
        assert!(matches!(
            NavRecordReader::open(&truncated.0, &profile),
            Err(NavRecordError::Truncated)
        ));
    }
}
