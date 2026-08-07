//! Portable input format shared by the mapping CLI commands.
//!
//! The supported format is UTF-8 newline-delimited JSON with extension
//! `.jsonl` or `.ndjson`. The first non-comment line is:
//!
//! ```json
//! {"format":"nuwax-mapping-jsonl","version":1}
//! ```
//!
//! Each following line is one timestamped record. Supported `type` values are
//! `occupancy_grid`, `point_cloud`, `pose`, and `marker`. This format is the
//! deterministic interchange boundary until a native rosbag2/MCAP/SQLite
//! decoder is added; those file types are rejected rather than guessed.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

use anyhow::{bail, Context};
use serde::{Deserialize, Serialize};

use crate::artifact::{publish_new_file, ArtifactFormat};

/// Current dataset format identifier.
pub const DATASET_FORMAT: &str = "nuwax-mapping-jsonl";
/// Current dataset format version.
pub const DATASET_VERSION: u32 = 1;

/// File header.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DatasetHeader {
    /// Must equal [`DATASET_FORMAT`].
    pub format: String,
    /// Must equal [`DATASET_VERSION`].
    pub version: u32,
}

impl Default for DatasetHeader {
    fn default() -> Self {
        Self {
            format: DATASET_FORMAT.to_string(),
            version: DATASET_VERSION,
        }
    }
}

/// A complete parsed dataset.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Dataset {
    /// Format header.
    pub header: DatasetHeader,
    /// Timestamped records in file order.
    pub records: Vec<DatasetRecord>,
}

impl Dataset {
    /// Construct an in-memory version-1 dataset.
    #[must_use]
    pub fn new(records: Vec<DatasetRecord>) -> Self {
        Self {
            header: DatasetHeader::default(),
            records,
        }
    }
}

/// Position and orientation in `[x,y,z]` and `[qx,qy,qz,qw]` order.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PoseData {
    /// Translation in meters.
    pub position: [f64; 3],
    /// Unit quaternion in XYZW order.
    pub orientation: [f64; 4],
}

/// One timestamped stream record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatasetRecord {
    /// Seconds on the dataset timeline.
    pub timestamp: f64,
    /// Logical stream name, such as `lidar`, `odom`, or `map`.
    pub stream: String,
    /// Typed payload.
    #[serde(flatten)]
    pub payload: RecordPayload,
}

/// Supported record payloads.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum RecordPayload {
    /// ROS-style occupancy grid with row-major cells (`-1`, `0..=100`).
    OccupancyGrid {
        /// Grid width in cells.
        width: u32,
        /// Grid height in cells.
        height: u32,
        /// Cell resolution in meters.
        resolution: f64,
        /// World-space origin translation.
        #[serde(default)]
        origin: [f64; 3],
        /// World-space origin quaternion in XYZW order.
        #[serde(default = "identity_quaternion")]
        origin_orientation: [f64; 4],
        /// Coordinate frame.
        frame_id: String,
        /// Row-major occupancy values.
        cells: Vec<i8>,
    },
    /// Point cloud whose coordinates are already expressed in `frame_id`.
    PointCloud {
        /// Coordinate frame.
        frame_id: String,
        /// XYZ points in meters.
        points: Vec<[f64; 3]>,
        /// Optional pose metadata populated by `pose-fill`.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        pose: Option<PoseData>,
    },
    /// Pose observation used to fill another stream's pose metadata.
    Pose {
        /// Pose value.
        pose: PoseData,
    },
    /// Already-detected fiducial marker observation.
    Marker {
        /// Detector family, for example `tag36h11`.
        family: String,
        /// Marker ID within the family.
        id: u32,
        /// Marker pose.
        pose: PoseData,
    },
}

impl RecordPayload {
    /// Stable payload type name used by summaries and replay manifests.
    #[must_use]
    pub fn kind(&self) -> &'static str {
        match self {
            Self::OccupancyGrid { .. } => "occupancy_grid",
            Self::PointCloud { .. } => "point_cloud",
            Self::Pose { .. } => "pose",
            Self::Marker { .. } => "marker",
        }
    }
}

/// Exact per-stream summary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StreamSummary {
    /// Record count.
    pub records: usize,
    /// Earliest timestamp.
    pub start_timestamp: f64,
    /// Latest timestamp.
    pub end_timestamp: f64,
}

/// Exact dataset summary.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DatasetSummary {
    /// Total records.
    pub records: usize,
    /// Counts keyed by record type.
    pub record_types: BTreeMap<String, usize>,
    /// Counts and time ranges keyed by stream.
    pub streams: BTreeMap<String, StreamSummary>,
    /// Total point count across point-cloud records.
    pub points: usize,
    /// Total marker observations.
    pub markers: usize,
    /// Earliest dataset timestamp, if any.
    pub start_timestamp: Option<f64>,
    /// Latest dataset timestamp, if any.
    pub end_timestamp: Option<f64>,
}

/// Read and validate a supported JSONL dataset.
///
/// # Errors
/// Returns a contextual error for unsupported extensions, missing or invalid
/// headers, malformed JSON, invalid record data, directories, or I/O errors.
pub fn read_dataset(path: &Path) -> anyhow::Result<Dataset> {
    require_supported_extension(path)?;
    if !path.is_file() {
        bail!("dataset is not a regular file: {}", path.display());
    }
    let file = File::open(path).with_context(|| format!("open dataset {}", path.display()))?;
    let mut meaningful_lines = Vec::new();
    for (index, line) in BufReader::new(file).lines().enumerate() {
        let line = line.with_context(|| format!("read {} line {}", path.display(), index + 1))?;
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        meaningful_lines.push((index + 1, trimmed.to_string()));
    }
    let (header_line, header_text) = meaningful_lines
        .first()
        .ok_or_else(|| anyhow::anyhow!("dataset {} is empty", path.display()))?;
    let header: DatasetHeader = serde_json::from_str(header_text).with_context(|| {
        format!(
            "parse {} line {} as dataset header",
            path.display(),
            header_line
        )
    })?;
    if header.format != DATASET_FORMAT || header.version != DATASET_VERSION {
        bail!(
            "unsupported dataset header format={:?} version={}; expected {:?} version {}",
            header.format,
            header.version,
            DATASET_FORMAT,
            DATASET_VERSION
        );
    }
    let mut records = Vec::with_capacity(meaningful_lines.len().saturating_sub(1));
    for (line_number, text) in meaningful_lines.into_iter().skip(1) {
        let record: DatasetRecord = serde_json::from_str(&text).with_context(|| {
            format!(
                "parse {} line {} as mapping record",
                path.display(),
                line_number
            )
        })?;
        validate_record(&record)
            .with_context(|| format!("validate {} line {}", path.display(), line_number))?;
        records.push(record);
    }
    Ok(Dataset { header, records })
}

/// Persist a dataset atomically to a new `.jsonl` or `.ndjson` file.
///
/// # Errors
/// Refuses to overwrite an existing output and returns validation or I/O
/// errors. A temporary sibling is removed on failure.
pub fn write_dataset(path: &Path, dataset: &Dataset) -> anyhow::Result<()> {
    require_supported_extension(path)?;
    validate_dataset(dataset)?;
    publish_new_file(
        path,
        ArtifactFormat::DatasetJsonLines,
        |writer| {
            serde_json::to_writer(&mut *writer, &dataset.header)?;
            writer.write_all(b"\n")?;
            for record in &dataset.records {
                serde_json::to_writer(&mut *writer, record)?;
                writer.write_all(b"\n")?;
            }
            Ok(())
        },
        |staging| {
            let persisted = read_dataset(staging)?;
            if persisted != *dataset {
                bail!("persisted dataset does not match the validated input");
            }
            Ok(())
        },
    )
}

/// Validate every record and the versioned header.
///
/// # Errors
/// Returns the first schema or numeric validation error.
pub fn validate_dataset(dataset: &Dataset) -> anyhow::Result<()> {
    if dataset.header.format != DATASET_FORMAT || dataset.header.version != DATASET_VERSION {
        bail!("dataset header does not match the supported format/version");
    }
    for record in &dataset.records {
        validate_record(record)?;
    }
    Ok(())
}

/// Compute exact counts and time ranges from parsed records.
#[must_use]
pub fn summarize(dataset: &Dataset) -> DatasetSummary {
    let mut record_types = BTreeMap::new();
    let mut streams: BTreeMap<String, StreamSummary> = BTreeMap::new();
    let mut points = 0usize;
    let mut markers = 0usize;
    let mut start_timestamp: Option<f64> = None;
    let mut end_timestamp: Option<f64> = None;

    for record in &dataset.records {
        *record_types
            .entry(record.payload.kind().to_string())
            .or_insert(0) += 1;
        let stream = streams
            .entry(record.stream.clone())
            .or_insert(StreamSummary {
                records: 0,
                start_timestamp: record.timestamp,
                end_timestamp: record.timestamp,
            });
        stream.records += 1;
        stream.start_timestamp = stream.start_timestamp.min(record.timestamp);
        stream.end_timestamp = stream.end_timestamp.max(record.timestamp);
        start_timestamp =
            Some(start_timestamp.map_or(record.timestamp, |value| value.min(record.timestamp)));
        end_timestamp =
            Some(end_timestamp.map_or(record.timestamp, |value| value.max(record.timestamp)));
        match &record.payload {
            RecordPayload::PointCloud { points: cloud, .. } => points += cloud.len(),
            RecordPayload::Marker { .. } => markers += 1,
            RecordPayload::OccupancyGrid { .. } | RecordPayload::Pose { .. } => {}
        }
    }

    DatasetSummary {
        records: dataset.records.len(),
        record_types,
        streams,
        points,
        markers,
        start_timestamp,
        end_timestamp,
    }
}

fn require_supported_extension(path: &Path) -> anyhow::Result<()> {
    let extension = path
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    if extension != "jsonl" && extension != "ndjson" {
        bail!(
            "unsupported dataset format for {}: expected .jsonl or .ndjson using {}; native rosbag2 (.db3), memory2 SQLite (.db), MCAP, and ROS1 bag decoding are not available in this build",
            path.display(),
            DATASET_FORMAT
        );
    }
    Ok(())
}

fn validate_record(record: &DatasetRecord) -> anyhow::Result<()> {
    if !record.timestamp.is_finite() {
        bail!("timestamp must be finite");
    }
    if record.stream.trim().is_empty() {
        bail!("stream name must not be empty");
    }
    match &record.payload {
        RecordPayload::OccupancyGrid {
            width,
            height,
            resolution,
            origin,
            origin_orientation,
            frame_id,
            cells,
        } => {
            if *width == 0 || *height == 0 {
                bail!("occupancy grid dimensions must be non-zero");
            }
            if !resolution.is_finite() || *resolution <= 0.0 {
                bail!("occupancy grid resolution must be finite and positive");
            }
            if frame_id.trim().is_empty() {
                bail!("occupancy grid frame_id must not be empty");
            }
            validate_finite(origin, "occupancy grid origin")?;
            validate_quaternion(*origin_orientation, "occupancy grid origin orientation")?;
            let expected = usize::try_from(u64::from(*width) * u64::from(*height))
                .context("occupancy grid dimensions overflow usize")?;
            if cells.len() != expected {
                bail!(
                    "occupancy grid has {} cells, expected {} for {}x{}",
                    cells.len(),
                    expected,
                    width,
                    height
                );
            }
            if let Some(value) = cells.iter().find(|&&value| !(-1..=100).contains(&value)) {
                bail!("occupancy value {value} is outside -1..=100");
            }
        }
        RecordPayload::PointCloud {
            frame_id,
            points,
            pose,
        } => {
            if frame_id.trim().is_empty() {
                bail!("point cloud frame_id must not be empty");
            }
            if points.is_empty() {
                bail!("point cloud must contain at least one point");
            }
            for point in points {
                validate_finite(point, "point cloud point")?;
            }
            if let Some(pose) = pose {
                validate_pose(*pose)?;
            }
        }
        RecordPayload::Pose { pose } => validate_pose(*pose)?,
        RecordPayload::Marker { family, pose, .. } => {
            if family.trim().is_empty() {
                bail!("marker family must not be empty");
            }
            validate_pose(*pose)?;
        }
    }
    Ok(())
}

fn validate_pose(pose: PoseData) -> anyhow::Result<()> {
    validate_finite(&pose.position, "pose position")?;
    validate_quaternion(pose.orientation, "pose orientation")
}

fn validate_quaternion(orientation: [f64; 4], label: &str) -> anyhow::Result<()> {
    validate_finite(&orientation, label)?;
    let norm_squared = orientation.iter().map(|value| value * value).sum::<f64>();
    if norm_squared <= 1e-12 {
        bail!("{label} quaternion must be non-zero");
    }
    Ok(())
}

fn identity_quaternion() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

fn validate_finite(values: &[f64], label: &str) -> anyhow::Result<()> {
    if values.iter().any(|value| !value.is_finite()) {
        bail!("{label} must contain only finite values");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summary_counts_records_points_and_markers() {
        let pose = PoseData {
            position: [0.0, 0.0, 0.0],
            orientation: [0.0, 0.0, 0.0, 1.0],
        };
        let dataset = Dataset::new(vec![
            DatasetRecord {
                timestamp: 2.0,
                stream: "lidar".into(),
                payload: RecordPayload::PointCloud {
                    frame_id: "map".into(),
                    points: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                    pose: None,
                },
            },
            DatasetRecord {
                timestamp: 3.0,
                stream: "markers".into(),
                payload: RecordPayload::Marker {
                    family: "tag36h11".into(),
                    id: 7,
                    pose,
                },
            },
        ]);
        let summary = summarize(&dataset);
        assert_eq!(summary.records, 2);
        assert_eq!(summary.points, 2);
        assert_eq!(summary.markers, 1);
        assert_eq!(summary.streams["lidar"].records, 1);
        assert_eq!(summary.start_timestamp, Some(2.0));
        assert_eq!(summary.end_timestamp, Some(3.0));
    }
}
