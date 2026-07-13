//! Native persisted-dataset adapters used by mapping CLI commands.
//!
//! DimOS mapping recordings use the memory2 SQLite schema: `_streams` stores
//! the payload module and codec, each logical stream has a metadata table, and
//! non-scalar payload bytes live in a sibling `<stream>_blob` table. This
//! module reads and rewrites that schema directly through the system SQLite C
//! API, so the CLI does not need a new Rust package or a Python subprocess.

use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;
use std::path::Path;

use anyhow::{bail, Context};
use nuwax_protocol::lcm::schema::{
    decode_occupancy_grid, decode_odometry, decode_point_cloud2, decode_pose, decode_pose_stamped,
    decode_tf_message, PointCloud2,
};

use crate::dataset::{
    read_dataset, validate_dataset, Dataset, DatasetRecord, DatasetSummary, PoseData,
    RecordPayload, StreamSummary,
};

/// Dataset container identified from its extension and native header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputFormat {
    /// Versioned `nuwax-mapping-jsonl` interchange data.
    JsonLines,
    /// DimOS memory2 SQLite database.
    Memory2Sqlite,
    /// ROS 2 rosbag2 SQLite database.
    Rosbag2Sqlite,
    /// Foxglove MCAP container.
    Mcap,
    /// ROS 1 bag container.
    Ros1Bag,
}

/// Result of decoding records usable by the Rust mapping commands.
#[derive(Debug, Clone, PartialEq)]
pub struct DecodedDataset {
    /// Typed records decoded from the source.
    pub dataset: Dataset,
    /// Streams intentionally skipped because the command has no decoder for
    /// their registered payload type.
    pub skipped_streams: Vec<String>,
}

/// Persisted memory2 stream registration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Memory2StreamConfig {
    /// Fully qualified Python payload class.
    pub payload_module: String,
    /// memory2 codec chain, for example `lcm` or `lz4+lcm`.
    pub codec_id: String,
    /// Complete registry JSON, preserved byte-for-byte by copy operations.
    pub raw_json: String,
}

/// Raw memory2 observation used by preservation tests and diagnostics.
#[derive(Debug, Clone, PartialEq)]
pub struct Memory2Row {
    /// SQLite observation identifier.
    pub id: i64,
    /// Observation timestamp.
    pub timestamp: f64,
    /// Inline scalar value, when present.
    pub value: Option<f64>,
    /// Seven-field observation pose metadata, when present.
    pub pose: Option<PoseData>,
    /// Raw JSON/JSONB tags bytes.
    pub tags: Vec<u8>,
    /// Raw encoded payload bytes, when the stream uses blob storage.
    pub blob: Option<Vec<u8>>,
}

/// One inspected memory2 stream.
#[derive(Debug, Clone, PartialEq)]
pub struct Memory2StreamInspection {
    /// Registry configuration.
    pub config: Memory2StreamConfig,
    /// Metadata and blob rows ordered by observation id.
    pub rows: Vec<Memory2Row>,
}

/// Complete raw inspection of a memory2 database.
#[derive(Debug, Clone, PartialEq)]
pub struct Memory2Inspection {
    /// Streams keyed by logical name.
    pub streams: BTreeMap<String, Memory2StreamInspection>,
}

/// Options for a native memory2 pose-fill copy.
#[derive(Debug, Clone, Copy)]
pub struct Memory2PoseFillOptions<'a> {
    /// Target stream whose observation pose metadata is replaced.
    pub target: &'a str,
    /// Stream whose decoded payload supplies poses.
    pub pose_source: &'a str,
    /// Maximum absolute timestamp difference in seconds.
    pub tolerance: f64,
    /// Optional subset of streams copied to the output.
    pub streams: Option<&'a [String]>,
    /// Relative seconds skipped per selected stream.
    pub seek: f64,
    /// Optional relative duration per selected stream.
    pub duration: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PayloadKind {
    PointCloud,
    OccupancyGrid,
    Pose,
    Unsupported,
}

/// Identify a supported or explicitly unsupported native container.
///
/// # Errors
/// Returns an error for a missing/non-file input, an extension/header
/// mismatch, or a container with no supported contract.
pub fn input_format(path: &Path) -> anyhow::Result<InputFormat> {
    if !path.is_file() {
        bail!("dataset is not a regular file: {}", path.display());
    }
    let extension = path
        .extension()
        .and_then(|value| value.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    match extension.as_str() {
        "jsonl" | "ndjson" => Ok(InputFormat::JsonLines),
        "db" => {
            let connection = sqlite::Connection::open_read_only(path)?;
            require_memory2_schema(&connection, path)?;
            Ok(InputFormat::Memory2Sqlite)
        }
        "db3" => {
            require_magic(path, b"SQLite format 3\0", "rosbag2 SQLite")?;
            Ok(InputFormat::Rosbag2Sqlite)
        }
        "mcap" => {
            require_magic(path, b"\x89MCAP0\r\n", "MCAP")?;
            Ok(InputFormat::Mcap)
        }
        "bag" => {
            require_magic(path, b"#ROSBAG V2.0\n", "ROS1 bag")?;
            Ok(InputFormat::Ros1Bag)
        }
        _ => bail!(
            "unsupported dataset extension for {}: expected .jsonl/.ndjson or DimOS memory2 .db",
            path.display()
        ),
    }
}

fn require_magic(path: &Path, expected: &[u8], label: &str) -> anyhow::Result<()> {
    let mut file = std::fs::File::open(path)?;
    let mut actual = vec![0_u8; expected.len()];
    file.read_exact(&mut actual)
        .with_context(|| format!("read {label} header from {}", path.display()))?;
    if actual != expected {
        bail!("{} does not have a valid {label} header", path.display());
    }
    Ok(())
}

/// Decode records supported by the mapping/replay commands.
///
/// Unknown memory2 stream types are reported in `skipped_streams`; a known
/// mapping payload with an unsupported codec is an error rather than a silent
/// skip.
///
/// # Errors
/// Returns schema, codec, payload, native-format, validation, or I/O errors.
pub fn read_command_dataset(path: &Path) -> anyhow::Result<DecodedDataset> {
    match input_format(path)? {
        InputFormat::JsonLines => Ok(DecodedDataset {
            dataset: read_dataset(path)?,
            skipped_streams: Vec::new(),
        }),
        InputFormat::Memory2Sqlite => decode_memory2_dataset(path),
        InputFormat::Rosbag2Sqlite => bail!(
            "rosbag2 SQLite (.db3) input {} requires topic metadata plus ROS 2 type-support CDR decoding; this dependency-free build refuses to treat CDR payloads as memory2 rows",
            path.display()
        ),
        InputFormat::Mcap => bail!(
            "MCAP (.mcap) input {} requires schema/channel records, chunk decompression, and per-schema message decoding; no dependency-free decoder is available",
            path.display()
        ),
        InputFormat::Ros1Bag => bail!(
            "ROS1 bag (.bag) input {} requires connection header parsing, chunk decompression, and ROS1 message definitions; no dependency-free decoder is available",
            path.display()
        ),
    }
}

/// Inspect every memory2 metadata/blob row without decoding payloads.
///
/// # Errors
/// Returns SQLite/schema errors or malformed row/config errors.
pub fn inspect_memory2(path: &Path) -> anyhow::Result<Memory2Inspection> {
    let connection = open_memory2(path)?;
    let configs = load_stream_configs(&connection)?;
    let mut streams = BTreeMap::new();
    for (name, config) in configs {
        let rows = load_raw_rows(&connection, &name)?;
        streams.insert(name, Memory2StreamInspection { config, rows });
    }
    Ok(Memory2Inspection { streams })
}

/// Compute exact memory2 stream counts, timestamp ranges, and point count.
///
/// # Errors
/// Returns SQLite/schema/config errors or a supported point-cloud codec error.
pub fn summarize_memory2(path: &Path) -> anyhow::Result<DatasetSummary> {
    let connection = open_memory2(path)?;
    let configs = load_stream_configs(&connection)?;
    let mut summary = DatasetSummary {
        records: 0,
        record_types: BTreeMap::new(),
        streams: BTreeMap::new(),
        points: 0,
        markers: 0,
        start_timestamp: None,
        end_timestamp: None,
    };

    for (name, config) in configs {
        let table = quote_identifier(&name)?;
        let rows = connection.query(
            &format!("SELECT COUNT(*), MIN(ts), MAX(ts) FROM {table}"),
            &[],
        )?;
        let row = rows
            .first()
            .ok_or_else(|| anyhow::anyhow!("missing aggregate row for stream {name:?}"))?;
        let count = usize::try_from(row_integer(row, 0, "stream count")?)
            .context("stream count does not fit usize")?;
        summary.records = summary
            .records
            .checked_add(count)
            .context("dataset record count overflow")?;
        let kind_name = payload_summary_name(&config.payload_module);
        *summary.record_types.entry(kind_name).or_insert(0) += count;

        if count > 0 {
            let start = row_real(row, 1, "minimum timestamp")?;
            let end = row_real(row, 2, "maximum timestamp")?;
            summary.streams.insert(
                name.clone(),
                StreamSummary {
                    records: count,
                    start_timestamp: start,
                    end_timestamp: end,
                },
            );
            summary.start_timestamp = Some(
                summary
                    .start_timestamp
                    .map_or(start, |value| value.min(start)),
            );
            summary.end_timestamp = Some(summary.end_timestamp.map_or(end, |value| value.max(end)));
        }

        if payload_kind(&config.payload_module) == PayloadKind::PointCloud {
            require_lcm_codec(&name, &config)?;
            let blob_table = quote_identifier(&format!("{name}_blob"))?;
            if !connection.table_exists(&format!("{name}_blob"))? {
                bail!("point-cloud stream {name:?} has no blob table");
            }
            for blob_row in connection.query(&format!("SELECT data FROM {blob_table}"), &[])? {
                let encoded = row_blob(&blob_row, 0, "point-cloud blob")?;
                let cloud = decode_point_cloud2(encoded)
                    .with_context(|| format!("decode point-cloud stream {name:?}"))?;
                let point_count = point_cloud_xyz(&cloud)?.len();
                summary.points = summary
                    .points
                    .checked_add(point_count)
                    .context("dataset point count overflow")?;
            }
        }
    }
    Ok(summary)
}

/// Copy a memory2 database while renaming/dropping logical streams.
///
/// Metadata, JSONB tags, LCM/pickle/JPEG blobs, ids, and registry JSON are
/// copied without decoding. Optional `seek`/`duration` windows match the
/// original per-stream relative-time behavior.
///
/// # Errors
/// Returns validation, collision, unsupported sqlite-vec, SQLite, or I/O
/// errors. Existing outputs are never overwritten.
pub fn rename_memory2(
    source_path: &Path,
    output_path: &Path,
    renames: &BTreeMap<String, String>,
    drops: &BTreeSet<String>,
    seek: f64,
    duration: Option<f64>,
    dry_run: bool,
) -> anyhow::Result<()> {
    validate_window(seek, duration)?;
    let source = open_memory2(source_path)?;
    let configs = load_stream_configs(&source)?;
    for name in renames.keys().chain(drops.iter()) {
        if !configs.contains_key(name) {
            bail!(
                "stream {name:?} does not exist in {}; available={:?}",
                source_path.display(),
                configs.keys().collect::<Vec<_>>()
            );
        }
    }
    let overlap: Vec<_> = renames
        .keys()
        .filter(|name| drops.contains(*name))
        .collect();
    if !overlap.is_empty() {
        bail!("streams cannot be both renamed and dropped: {overlap:?}");
    }
    let mut destinations = BTreeMap::<String, String>::new();
    for source_name in configs.keys().filter(|name| !drops.contains(*name)) {
        let destination = renames.get(source_name).unwrap_or(source_name);
        validate_identifier(destination)?;
        if let Some(previous) = destinations.insert(destination.clone(), source_name.clone()) {
            bail!(
                "renaming would merge streams {previous:?} and {source_name:?} into {destination:?}"
            );
        }
    }
    reject_vector_tables(&source, configs.keys())?;
    validate_known_lcm_blobs(&source, &configs)?;
    if dry_run {
        return Ok(());
    }

    with_memory2_copy(&source, output_path, |destination| {
        transaction(destination, |connection| {
            for name in configs.keys() {
                if drops.contains(name) {
                    drop_stream(connection, name)?;
                } else {
                    retain_time_window(connection, name, seek, duration)?;
                }
            }
            rename_streams(connection, renames)?;
            Ok(())
        })
    })
}

/// Copy a memory2 database and attach nearest pose-source payloads to a target.
///
/// Target payload blobs are copied verbatim. Only metadata pose columns and
/// the target R*Tree are rewritten; target observations without an in-window
/// match are removed.
///
/// # Errors
/// Returns validation, stream, codec, payload, SQLite, or I/O errors. Existing
/// outputs are never overwritten.
pub fn pose_fill_memory2(
    source_path: &Path,
    output_path: &Path,
    options: Memory2PoseFillOptions<'_>,
) -> anyhow::Result<(usize, usize)> {
    let Memory2PoseFillOptions {
        target,
        pose_source,
        tolerance,
        streams,
        seek,
        duration,
    } = options;
    validate_identifier(target)?;
    validate_identifier(pose_source)?;
    validate_window(seek, duration)?;
    if !tolerance.is_finite() || tolerance < 0.0 {
        bail!("tolerance must be finite and non-negative");
    }
    let source = open_memory2(source_path)?;
    let configs = load_stream_configs(&source)?;
    if !configs.contains_key(target) {
        bail!("target stream {target:?} does not exist");
    }
    let pose_config = configs
        .get(pose_source)
        .ok_or_else(|| anyhow::anyhow!("pose source stream {pose_source:?} does not exist"))?;
    if payload_kind(&pose_config.payload_module) != PayloadKind::Pose {
        bail!(
            "pose source stream {pose_source:?} payload {:?} is not a supported pose message",
            pose_config.payload_module
        );
    }
    require_lcm_codec(pose_source, pose_config)?;
    reject_vector_tables(&source, configs.keys())?;

    let selected: Option<BTreeSet<String>> = streams
        .map(|names| {
            names
                .iter()
                .map(|name| {
                    validate_identifier(name)?;
                    if !configs.contains_key(name) {
                        bail!("selected stream {name:?} does not exist");
                    }
                    Ok(name.clone())
                })
                .collect()
        })
        .transpose()?;
    if selected
        .as_ref()
        .is_some_and(|names| !names.contains(target))
    {
        bail!("selected streams must include target {target:?}");
    }
    let pose_samples = load_pose_samples(&source, pose_source, pose_config, seek, duration)?;
    if pose_samples.is_empty() {
        bail!("pose source stream {pose_source:?} has no observations in the selected window");
    }

    let mut result_counts = None;
    with_memory2_copy(&source, output_path, |destination| {
        transaction(destination, |connection| {
            if let Some(selected) = &selected {
                for name in configs.keys().filter(|name| !selected.contains(*name)) {
                    drop_stream(connection, name)?;
                }
            }
            for name in configs.keys() {
                if selected.as_ref().is_none_or(|names| names.contains(name)) {
                    retain_time_window(connection, name, seek, duration)?;
                }
            }
            result_counts = Some(fill_target_poses(
                connection,
                target,
                &pose_samples,
                tolerance,
            )?);
            Ok(())
        })
    })?;
    result_counts.ok_or_else(|| anyhow::anyhow!("pose-fill completed without result counts"))
}

fn with_memory2_copy(
    source: &sqlite::Connection,
    output_path: &Path,
    mutate: impl FnOnce(&sqlite::Connection) -> anyhow::Result<()>,
) -> anyhow::Result<()> {
    if output_path.exists() {
        bail!(
            "refusing to overwrite existing dataset: {}",
            output_path.display()
        );
    }
    if output_path.extension().and_then(|value| value.to_str()) != Some("db") {
        bail!(
            "memory2 output must use .db extension: {}",
            output_path.display()
        );
    }
    let parent = output_path.parent().unwrap_or_else(|| Path::new("."));
    if !parent.is_dir() {
        bail!("output directory does not exist: {}", parent.display());
    }
    let result = (|| {
        let destination = sqlite::Connection::open_read_write_create(output_path)?;
        destination.backup_from(source)?;
        mutate(&destination)?;
        destination.query("PRAGMA wal_checkpoint(TRUNCATE)", &[])?;
        Ok(())
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(output_path);
        let _ = std::fs::remove_file(output_path.with_extension("db-wal"));
        let _ = std::fs::remove_file(output_path.with_extension("db-shm"));
    }
    result
}

fn transaction(
    connection: &sqlite::Connection,
    action: impl FnOnce(&sqlite::Connection) -> anyhow::Result<()>,
) -> anyhow::Result<()> {
    connection.execute("BEGIN IMMEDIATE", &[])?;
    match action(connection) {
        Ok(()) => connection.execute("COMMIT", &[]),
        Err(error) => {
            let _ = connection.execute("ROLLBACK", &[]);
            Err(error)
        }
    }
}

fn reject_vector_tables<'a>(
    connection: &sqlite::Connection,
    names: impl Iterator<Item = &'a String>,
) -> anyhow::Result<()> {
    for name in names {
        if connection.table_exists(&format!("{name}_vec"))? {
            bail!(
                "memory2 stream {name:?} contains a sqlite-vec table; dependency-free rewriting cannot safely load or rebuild vec0, so the database is left unchanged"
            );
        }
    }
    Ok(())
}

fn validate_known_lcm_blobs(
    connection: &sqlite::Connection,
    configs: &BTreeMap<String, Memory2StreamConfig>,
) -> anyhow::Result<()> {
    for (name, config) in configs {
        let kind = payload_kind(&config.payload_module);
        if kind == PayloadKind::Unsupported {
            continue;
        }
        require_lcm_codec(name, config)?;
        let blob_name = format!("{name}_blob");
        if !connection.table_exists(&blob_name)? {
            bail!("LCM stream {name:?} has no blob table");
        }
        let blob_table = quote_identifier(&blob_name)?;
        for row in connection.query(&format!("SELECT id, data FROM {blob_table}"), &[])? {
            let id = row_integer(&row, 0, "blob id")?;
            let encoded = row_blob(&row, 1, "blob data")?;
            match kind {
                PayloadKind::PointCloud => {
                    let cloud = decode_point_cloud2(encoded).with_context(|| {
                        format!("decode memory2 stream {name:?} observation id={id}")
                    })?;
                    point_cloud_xyz(&cloud)?;
                }
                PayloadKind::OccupancyGrid => {
                    decode_occupancy_grid(encoded).with_context(|| {
                        format!("decode memory2 stream {name:?} observation id={id}")
                    })?;
                }
                PayloadKind::Pose => {
                    decode_pose_payload(&config.payload_module, encoded).with_context(|| {
                        format!("decode memory2 stream {name:?} observation id={id}")
                    })?;
                }
                PayloadKind::Unsupported => unreachable!("unsupported payloads are skipped"),
            }
        }
    }
    Ok(())
}

fn validate_window(seek: f64, duration: Option<f64>) -> anyhow::Result<()> {
    if !seek.is_finite() || seek < 0.0 {
        bail!("seek must be finite and non-negative");
    }
    if duration.is_some_and(|value| !value.is_finite() || value < 0.0) {
        bail!("duration must be finite and non-negative");
    }
    Ok(())
}

fn retain_time_window(
    connection: &sqlite::Connection,
    name: &str,
    seek: f64,
    duration: Option<f64>,
) -> anyhow::Result<()> {
    if seek == 0.0 && duration.is_none() {
        return Ok(());
    }
    let table = quote_identifier(name)?;
    let initial = connection.query(&format!("SELECT ts FROM {table} ORDER BY id LIMIT 1"), &[])?;
    let Some(first_row) = initial.first() else {
        return Ok(());
    };
    let mut predicates = Vec::new();
    let mut parameters = Vec::new();
    if seek > 0.0 {
        let cutoff = row_real(first_row, 0, "first stream timestamp")? + seek;
        predicates.push("ts > ?");
        parameters.push(sqlite::Bind::Real(cutoff));
    }
    if let Some(duration) = duration {
        let first_selected = if predicates.is_empty() {
            connection.query(&format!("SELECT ts FROM {table} ORDER BY id LIMIT 1"), &[])?
        } else {
            connection.query(
                &format!(
                    "SELECT ts FROM {table} WHERE {} ORDER BY id LIMIT 1",
                    predicates[0]
                ),
                &parameters,
            )?
        };
        if let Some(row) = first_selected.first() {
            let cutoff = row_real(row, 0, "window start timestamp")? + duration;
            predicates.push("ts < ?");
            parameters.push(sqlite::Bind::Real(cutoff));
        } else {
            connection.execute(&format!("DELETE FROM {table}"), &[])?;
            cleanup_stream_children(connection, name)?;
            return Ok(());
        }
    }
    if !predicates.is_empty() {
        connection.execute(
            &format!(
                "DELETE FROM {table} WHERE NOT ({})",
                predicates.join(" AND ")
            ),
            &parameters,
        )?;
        cleanup_stream_children(connection, name)?;
    }
    Ok(())
}

fn cleanup_stream_children(connection: &sqlite::Connection, name: &str) -> anyhow::Result<()> {
    let table = quote_identifier(name)?;
    for suffix in ["_blob", "_rtree"] {
        let child_name = format!("{name}{suffix}");
        if connection.table_exists(&child_name)? {
            let child = quote_identifier(&child_name)?;
            connection.execute(
                &format!("DELETE FROM {child} WHERE id NOT IN (SELECT id FROM {table})"),
                &[],
            )?;
        }
    }
    Ok(())
}

fn drop_stream(connection: &sqlite::Connection, name: &str) -> anyhow::Result<()> {
    for table_name in [
        format!("{name}_rtree"),
        format!("{name}_blob"),
        name.to_string(),
    ] {
        if connection.table_exists(&table_name)? {
            connection.execute(
                &format!("DROP TABLE {}", quote_identifier(&table_name)?),
                &[],
            )?;
        }
    }
    connection.execute(
        "DELETE FROM _streams WHERE name = ?",
        &[sqlite::Bind::Text(name)],
    )?;
    Ok(())
}

fn rename_streams(
    connection: &sqlite::Connection,
    renames: &BTreeMap<String, String>,
) -> anyhow::Result<()> {
    let changed: Vec<_> = renames
        .iter()
        .filter(|(source, destination)| source != destination)
        .collect();
    let mut staged = Vec::with_capacity(changed.len());
    for (index, (source, destination)) in changed.into_iter().enumerate() {
        let temporary = format!("_nuwax_rename_{index}");
        if connection.table_exists(&temporary)? {
            bail!("temporary rename table {temporary:?} already exists");
        }
        rename_stream_tables(connection, source, &temporary)?;
        connection.execute(
            "UPDATE _streams SET name = ? WHERE name = ?",
            &[sqlite::Bind::Text(&temporary), sqlite::Bind::Text(source)],
        )?;
        staged.push((temporary, destination.clone()));
    }
    for (temporary, destination) in staged {
        rename_stream_tables(connection, &temporary, &destination)?;
        connection.execute(
            "UPDATE _streams SET name = ? WHERE name = ?",
            &[
                sqlite::Bind::Text(&destination),
                sqlite::Bind::Text(&temporary),
            ],
        )?;
    }
    Ok(())
}

fn rename_stream_tables(
    connection: &sqlite::Connection,
    source: &str,
    destination: &str,
) -> anyhow::Result<()> {
    validate_identifier(source)?;
    validate_identifier(destination)?;
    let source_table = quote_identifier(source)?;
    let destination_table = quote_identifier(destination)?;
    connection.execute(
        &format!("ALTER TABLE {source_table} RENAME TO {destination_table}"),
        &[],
    )?;

    let source_blob_name = format!("{source}_blob");
    if connection.table_exists(&source_blob_name)? {
        let destination_blob_name = format!("{destination}_blob");
        connection.execute(
            &format!(
                "ALTER TABLE {} RENAME TO {}",
                quote_identifier(&source_blob_name)?,
                quote_identifier(&destination_blob_name)?
            ),
            &[],
        )?;
    }

    let source_rtree_name = format!("{source}_rtree");
    if connection.table_exists(&source_rtree_name)? {
        let destination_rtree_name = format!("{destination}_rtree");
        let source_rtree = quote_identifier(&source_rtree_name)?;
        let destination_rtree = quote_identifier(&destination_rtree_name)?;
        connection.execute(
            &format!(
                "CREATE VIRTUAL TABLE {destination_rtree} USING rtree(id, x_min, x_max, y_min, y_max, z_min, z_max)"
            ),
            &[],
        )?;
        connection.execute(
            &format!(
                "INSERT INTO {destination_rtree} SELECT id, x_min, x_max, y_min, y_max, z_min, z_max FROM {source_rtree}"
            ),
            &[],
        )?;
        connection.execute(&format!("DROP TABLE {source_rtree}"), &[])?;
    }
    Ok(())
}

fn load_pose_samples(
    connection: &sqlite::Connection,
    name: &str,
    config: &Memory2StreamConfig,
    seek: f64,
    duration: Option<f64>,
) -> anyhow::Result<Vec<(f64, PoseData)>> {
    let table = quote_identifier(name)?;
    let blob_name = format!("{name}_blob");
    if !connection.table_exists(&blob_name)? {
        bail!("pose source stream {name:?} has no blob table");
    }
    let blob_table = quote_identifier(&blob_name)?;
    let bounds = time_window_bounds(connection, name, seek, duration)?;
    let mut sql = format!(
        "SELECT meta.ts, blob.data FROM {table} AS meta JOIN {blob_table} AS blob ON blob.id = meta.id"
    );
    let mut parameters = Vec::new();
    if let Some((lower, upper)) = bounds {
        let mut predicates = Vec::new();
        if let Some(lower) = lower {
            predicates.push("meta.ts > ?");
            parameters.push(sqlite::Bind::Real(lower));
        }
        if let Some(upper) = upper {
            predicates.push("meta.ts < ?");
            parameters.push(sqlite::Bind::Real(upper));
        }
        if !predicates.is_empty() {
            sql.push_str(" WHERE ");
            sql.push_str(&predicates.join(" AND "));
        }
    } else {
        return Ok(Vec::new());
    }
    sql.push_str(" ORDER BY meta.ts, meta.id");
    connection
        .query(&sql, &parameters)?
        .into_iter()
        .map(|row| {
            let timestamp = row_real(&row, 0, "pose timestamp")?;
            let blob = row_blob(&row, 1, "pose blob")?;
            Ok((
                timestamp,
                decode_pose_payload(&config.payload_module, blob)?,
            ))
        })
        .collect()
}

fn time_window_bounds(
    connection: &sqlite::Connection,
    name: &str,
    seek: f64,
    duration: Option<f64>,
) -> anyhow::Result<Option<(Option<f64>, Option<f64>)>> {
    let table = quote_identifier(name)?;
    let first = connection.query(&format!("SELECT ts FROM {table} ORDER BY id LIMIT 1"), &[])?;
    let Some(first) = first.first() else {
        return Ok(None);
    };
    let lower = (seek > 0.0)
        .then(|| row_real(first, 0, "first timestamp").map(|ts| ts + seek))
        .transpose()?;
    let first_selected = if let Some(lower) = lower {
        connection.query(
            &format!("SELECT ts FROM {table} WHERE ts > ? ORDER BY id LIMIT 1"),
            &[sqlite::Bind::Real(lower)],
        )?
    } else {
        vec![first.clone()]
    };
    let Some(first_selected) = first_selected.first() else {
        return Ok(None);
    };
    let upper = duration
        .map(|value| row_real(first_selected, 0, "window start timestamp").map(|ts| ts + value))
        .transpose()?;
    Ok(Some((lower, upper)))
}

fn fill_target_poses(
    connection: &sqlite::Connection,
    target: &str,
    poses: &[(f64, PoseData)],
    tolerance: f64,
) -> anyhow::Result<(usize, usize)> {
    let table = quote_identifier(target)?;
    let rows = connection.query(&format!("SELECT id, ts FROM {table} ORDER BY ts, id"), &[])?;
    let total = rows.len();
    let mut matched = 0usize;
    for row in rows {
        let id = row_integer(&row, 0, "target id")?;
        let timestamp = row_real(&row, 1, "target timestamp")?;
        if let Some(pose) = nearest_pose_sample(poses, timestamp, tolerance) {
            connection.execute(
                &format!(
                    "UPDATE {table} SET pose_x=?, pose_y=?, pose_z=?, pose_qx=?, pose_qy=?, pose_qz=?, pose_qw=? WHERE id=?"
                ),
                &[
                    sqlite::Bind::Real(pose.position[0]),
                    sqlite::Bind::Real(pose.position[1]),
                    sqlite::Bind::Real(pose.position[2]),
                    sqlite::Bind::Real(pose.orientation[0]),
                    sqlite::Bind::Real(pose.orientation[1]),
                    sqlite::Bind::Real(pose.orientation[2]),
                    sqlite::Bind::Real(pose.orientation[3]),
                    sqlite::Bind::Integer(id),
                ],
            )?;
            matched += 1;
        } else {
            connection.execute(
                &format!("DELETE FROM {table} WHERE id = ?"),
                &[sqlite::Bind::Integer(id)],
            )?;
        }
    }
    cleanup_stream_children(connection, target)?;
    rebuild_target_rtree(connection, target)?;
    Ok((matched, total))
}

fn nearest_pose_sample(
    poses: &[(f64, PoseData)],
    timestamp: f64,
    tolerance: f64,
) -> Option<PoseData> {
    let insertion = poses.partition_point(|(candidate, _)| *candidate < timestamp);
    let before = insertion.checked_sub(1).and_then(|index| poses.get(index));
    let after = poses.get(insertion);
    let best = match (before, after) {
        (Some(before), Some(after)) => {
            if timestamp - before.0 <= after.0 - timestamp {
                before
            } else {
                after
            }
        }
        (Some(before), None) => before,
        (None, Some(after)) => after,
        (None, None) => return None,
    };
    ((best.0 - timestamp).abs() <= tolerance).then_some(best.1)
}

fn rebuild_target_rtree(connection: &sqlite::Connection, target: &str) -> anyhow::Result<()> {
    let table = quote_identifier(target)?;
    let rtree_name = format!("{target}_rtree");
    if !connection.table_exists(&rtree_name)? {
        return Ok(());
    }
    let rtree = quote_identifier(&rtree_name)?;
    connection.execute(&format!("DELETE FROM {rtree}"), &[])?;
    connection.execute(
        &format!(
            "INSERT INTO {rtree} (id, x_min, x_max, y_min, y_max, z_min, z_max) SELECT id, pose_x, pose_x, pose_y, pose_y, pose_z, pose_z FROM {table} WHERE pose_x IS NOT NULL"
        ),
        &[],
    )?;
    Ok(())
}

fn decode_memory2_dataset(path: &Path) -> anyhow::Result<DecodedDataset> {
    let connection = open_memory2(path)?;
    let configs = load_stream_configs(&connection)?;
    let mut records = Vec::new();
    let mut skipped_streams = Vec::new();

    for (name, config) in configs {
        let kind = payload_kind(&config.payload_module);
        if kind == PayloadKind::Unsupported {
            skipped_streams.push(name);
            continue;
        }
        require_lcm_codec(&name, &config)?;
        let table = quote_identifier(&name)?;
        let blob_name = format!("{name}_blob");
        if !connection.table_exists(&blob_name)? {
            bail!("LCM stream {name:?} has no blob table");
        }
        let blob_table = quote_identifier(&blob_name)?;
        let sql = format!(
            "SELECT meta.ts, meta.pose_x, meta.pose_y, meta.pose_z, meta.pose_qx, meta.pose_qy, meta.pose_qz, meta.pose_qw, blob.data FROM {table} AS meta JOIN {blob_table} AS blob ON blob.id = meta.id ORDER BY meta.id"
        );
        for row in connection.query(&sql, &[])? {
            let timestamp = row_real(&row, 0, "observation timestamp")?;
            let observation_pose = row_pose(&row, 1)?;
            let encoded = row_blob(&row, 8, "observation blob")?;
            let payload = match kind {
                PayloadKind::PointCloud => {
                    let cloud = decode_point_cloud2(encoded)
                        .with_context(|| format!("decode point-cloud stream {name:?}"))?;
                    let points = point_cloud_xyz(&cloud)?;
                    RecordPayload::PointCloud {
                        frame_id: cloud.header.frame_id,
                        points,
                        pose: observation_pose,
                    }
                }
                PayloadKind::OccupancyGrid => {
                    let grid = decode_occupancy_grid(encoded)
                        .with_context(|| format!("decode occupancy stream {name:?}"))?;
                    let width =
                        u32::try_from(grid.info.width).context("occupancy width is negative")?;
                    let height =
                        u32::try_from(grid.info.height).context("occupancy height is negative")?;
                    RecordPayload::OccupancyGrid {
                        width,
                        height,
                        resolution: f64::from(grid.info.resolution),
                        origin: grid.info.origin.position,
                        origin_orientation: grid.info.origin.orientation_xyzw,
                        frame_id: grid.header.frame_id,
                        cells: grid.data,
                    }
                }
                PayloadKind::Pose => RecordPayload::Pose {
                    pose: decode_pose_payload(&config.payload_module, encoded)
                        .with_context(|| format!("decode pose stream {name:?}"))?,
                },
                PayloadKind::Unsupported => unreachable!("unsupported streams are skipped"),
            };
            records.push(DatasetRecord {
                timestamp,
                stream: name.clone(),
                payload,
            });
        }
    }

    records.sort_by(|left, right| {
        left.timestamp
            .total_cmp(&right.timestamp)
            .then_with(|| left.stream.cmp(&right.stream))
    });
    skipped_streams.sort();
    let dataset = Dataset::new(records);
    validate_dataset(&dataset)?;
    Ok(DecodedDataset {
        dataset,
        skipped_streams,
    })
}

fn decode_pose_payload(module: &str, encoded: &[u8]) -> anyhow::Result<PoseData> {
    if module.ends_with(".Odometry.Odometry") {
        let odometry = decode_odometry(encoded)?;
        return Ok(PoseData {
            position: odometry.pose.position,
            orientation: odometry.pose.orientation_xyzw,
        });
    }
    if module.ends_with(".PoseStamped.PoseStamped") {
        let stamped = decode_pose_stamped(encoded)?;
        return Ok(PoseData {
            position: stamped.pose.position,
            orientation: stamped.pose.orientation_xyzw,
        });
    }
    if module.ends_with(".Transform.Transform") {
        let transform = decode_tf_message(encoded)?
            .transforms
            .into_iter()
            .next()
            .ok_or_else(|| anyhow::anyhow!("TFMessage contains no transform"))?;
        return Ok(PoseData {
            position: transform.transform.translation,
            orientation: transform.transform.rotation_xyzw,
        });
    }
    if module.ends_with(".Pose.Pose") {
        let pose = decode_pose(encoded)?;
        return Ok(PoseData {
            position: pose.position,
            orientation: pose.orientation_xyzw,
        });
    }
    bail!("unsupported pose payload module {module:?}")
}

fn point_cloud_count(cloud: &PointCloud2) -> anyhow::Result<usize> {
    let width = usize::try_from(cloud.width).context("point-cloud width is negative")?;
    let height = usize::try_from(cloud.height).context("point-cloud height is negative")?;
    width
        .checked_mul(height)
        .context("point-cloud dimensions overflow usize")
}

fn point_cloud_xyz(cloud: &PointCloud2) -> anyhow::Result<Vec<[f64; 3]>> {
    let count = point_cloud_count(cloud)?;
    if count == 0 {
        bail!("point-cloud payload contains no points");
    }
    let point_step =
        usize::try_from(cloud.point_step).context("point-cloud point_step is negative")?;
    let row_step = usize::try_from(cloud.row_step).context("point-cloud row_step is negative")?;
    let width = usize::try_from(cloud.width).context("point-cloud width is negative")?;
    let height = usize::try_from(cloud.height).context("point-cloud height is negative")?;
    if point_step == 0 || row_step < point_step.saturating_mul(width) {
        bail!("point-cloud point_step/row_step is inconsistent with dimensions");
    }
    let x = coordinate_field(cloud, "x")?;
    let y = coordinate_field(cloud, "y")?;
    let z = coordinate_field(cloud, "z")?;
    let mut points = Vec::with_capacity(count);
    for row in 0..height {
        for column in 0..width {
            let base = row
                .checked_mul(row_step)
                .and_then(|value| value.checked_add(column.saturating_mul(point_step)))
                .context("point-cloud byte offset overflow")?;
            let point = [
                read_coordinate(cloud, base, x)?,
                read_coordinate(cloud, base, y)?,
                read_coordinate(cloud, base, z)?,
            ];
            if point.iter().any(|value| !value.is_finite()) {
                bail!("point-cloud contains a non-finite XYZ coordinate");
            }
            points.push(point);
        }
    }
    Ok(points)
}

#[derive(Debug, Clone, Copy)]
struct CoordinateField {
    offset: usize,
    datatype: u8,
}

fn coordinate_field(cloud: &PointCloud2, name: &str) -> anyhow::Result<CoordinateField> {
    let field = cloud
        .fields
        .iter()
        .find(|field| field.name == name)
        .ok_or_else(|| anyhow::anyhow!("point-cloud is missing {name:?} field"))?;
    if field.count != 1 || (field.datatype != 7 && field.datatype != 8) {
        bail!(
            "point-cloud field {name:?} must be scalar FLOAT32(7) or FLOAT64(8), got datatype={} count={}",
            field.datatype,
            field.count
        );
    }
    Ok(CoordinateField {
        offset: usize::try_from(field.offset).context("point field offset is negative")?,
        datatype: field.datatype,
    })
}

fn read_coordinate(
    cloud: &PointCloud2,
    point_base: usize,
    field: CoordinateField,
) -> anyhow::Result<f64> {
    let size = if field.datatype == 7 { 4 } else { 8 };
    let start = point_base
        .checked_add(field.offset)
        .context("point field offset overflow")?;
    let end = start
        .checked_add(size)
        .context("point field end overflow")?;
    let bytes = cloud
        .data
        .get(start..end)
        .ok_or_else(|| anyhow::anyhow!("point field extends past PointCloud2 data"))?;
    if field.datatype == 7 {
        let bytes: [u8; 4] = bytes.try_into().expect("checked four-byte slice");
        Ok(f64::from(if cloud.is_bigendian {
            f32::from_be_bytes(bytes)
        } else {
            f32::from_le_bytes(bytes)
        }))
    } else {
        let bytes: [u8; 8] = bytes.try_into().expect("checked eight-byte slice");
        Ok(if cloud.is_bigendian {
            f64::from_be_bytes(bytes)
        } else {
            f64::from_le_bytes(bytes)
        })
    }
}

fn payload_kind(module: &str) -> PayloadKind {
    if module.ends_with(".PointCloud2.PointCloud2") {
        PayloadKind::PointCloud
    } else if module.ends_with(".OccupancyGrid.OccupancyGrid") {
        PayloadKind::OccupancyGrid
    } else if module.ends_with(".Odometry.Odometry")
        || module.ends_with(".PoseStamped.PoseStamped")
        || module.ends_with(".Transform.Transform")
        || module.ends_with(".Pose.Pose")
    {
        PayloadKind::Pose
    } else {
        PayloadKind::Unsupported
    }
}

fn payload_summary_name(module: &str) -> String {
    match payload_kind(module) {
        PayloadKind::PointCloud => "point_cloud".to_string(),
        PayloadKind::OccupancyGrid => "occupancy_grid".to_string(),
        PayloadKind::Pose => "pose".to_string(),
        PayloadKind::Unsupported => format!("native:{module}"),
    }
}

fn require_lcm_codec(name: &str, config: &Memory2StreamConfig) -> anyhow::Result<()> {
    if config.codec_id != "lcm" {
        bail!(
            "memory2 stream {name:?} uses codec {:?}; this build can decode exact generated LCM blobs only (Python lz4.frame/pickle/jpeg codecs are not guessed)",
            config.codec_id
        );
    }
    Ok(())
}

fn open_memory2(path: &Path) -> anyhow::Result<sqlite::Connection> {
    let connection = sqlite::Connection::open_read_only(path)?;
    require_memory2_schema(&connection, path)?;
    Ok(connection)
}

fn require_memory2_schema(connection: &sqlite::Connection, path: &Path) -> anyhow::Result<()> {
    if !connection.table_exists("_streams")? {
        bail!(
            "SQLite .db {} is not a DimOS memory2 dataset: missing _streams registry",
            path.display()
        );
    }
    let columns = connection.query("PRAGMA table_info(\"_streams\")", &[])?;
    let names: BTreeSet<String> = columns
        .iter()
        .filter_map(|row| {
            row.get(1)
                .and_then(sqlite::Value::as_text)
                .map(str::to_owned)
        })
        .collect();
    if !names.contains("name") || !names.contains("config") {
        bail!(
            "SQLite .db {} has an incompatible _streams registry",
            path.display()
        );
    }
    Ok(())
}

fn load_stream_configs(
    connection: &sqlite::Connection,
) -> anyhow::Result<BTreeMap<String, Memory2StreamConfig>> {
    let mut configs = BTreeMap::new();
    for row in connection.query("SELECT name, config FROM _streams ORDER BY name", &[])? {
        let name = row_text(&row, 0, "stream name")?.to_string();
        validate_identifier(&name)?;
        let raw_json = row_text(&row, 1, "stream config")?.to_string();
        let parsed: serde_json::Value = serde_json::from_str(&raw_json)
            .with_context(|| format!("parse memory2 registry config for {name:?}"))?;
        let payload_module = parsed
            .get("payload_module")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("stream {name:?} config has no payload_module"))?
            .to_string();
        let codec_id = parsed
            .get("codec_id")
            .and_then(serde_json::Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("stream {name:?} config has no codec_id"))?
            .to_string();
        if !connection.table_exists(&name)? {
            bail!("memory2 stream {name:?} has no metadata table");
        }
        configs.insert(
            name,
            Memory2StreamConfig {
                payload_module,
                codec_id,
                raw_json,
            },
        );
    }
    Ok(configs)
}

fn load_raw_rows(connection: &sqlite::Connection, name: &str) -> anyhow::Result<Vec<Memory2Row>> {
    let table = quote_identifier(name)?;
    let sql = format!(
        "SELECT id, ts, value, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, tags FROM {table} ORDER BY id"
    );
    let blob_name = format!("{name}_blob");
    let blobs = if connection.table_exists(&blob_name)? {
        let blob_table = quote_identifier(&blob_name)?;
        connection
            .query(
                &format!("SELECT id, data FROM {blob_table} ORDER BY id"),
                &[],
            )?
            .into_iter()
            .map(|row| {
                Ok((
                    row_integer(&row, 0, "blob id")?,
                    row_blob(&row, 1, "blob data")?.to_vec(),
                ))
            })
            .collect::<anyhow::Result<BTreeMap<_, _>>>()?
    } else {
        BTreeMap::new()
    };
    connection
        .query(&sql, &[])?
        .into_iter()
        .map(|row| {
            let id = row_integer(&row, 0, "observation id")?;
            let value = match row.get(2) {
                Some(sqlite::Value::Null) | None => None,
                Some(sqlite::Value::Integer(value)) => Some(*value as f64),
                Some(sqlite::Value::Real(value)) => Some(*value),
                Some(other) => bail!("inline value has unexpected SQLite type {other:?}"),
            };
            let tags = match row.get(10) {
                Some(sqlite::Value::Blob(value)) => value.clone(),
                Some(sqlite::Value::Text(value)) => value.as_bytes().to_vec(),
                Some(sqlite::Value::Null) | None => Vec::new(),
                Some(other) => bail!("tags have unexpected SQLite type {other:?}"),
            };
            Ok(Memory2Row {
                id,
                timestamp: row_real(&row, 1, "observation timestamp")?,
                value,
                pose: row_pose(&row, 3)?,
                tags,
                blob: blobs.get(&id).cloned(),
            })
        })
        .collect()
}

fn row_pose(row: &[sqlite::Value], start: usize) -> anyhow::Result<Option<PoseData>> {
    if matches!(row.get(start), Some(sqlite::Value::Null) | None) {
        return Ok(None);
    }
    Ok(Some(PoseData {
        position: [
            row_real(row, start, "pose_x")?,
            row_real(row, start + 1, "pose_y")?,
            row_real(row, start + 2, "pose_z")?,
        ],
        orientation: [
            row_real(row, start + 3, "pose_qx")?,
            row_real(row, start + 4, "pose_qy")?,
            row_real(row, start + 5, "pose_qz")?,
            row_real(row, start + 6, "pose_qw")?,
        ],
    }))
}

fn row_integer(row: &[sqlite::Value], index: usize, label: &str) -> anyhow::Result<i64> {
    match row.get(index) {
        Some(sqlite::Value::Integer(value)) => Ok(*value),
        other => bail!("{label} is not an SQLite integer: {other:?}"),
    }
}

fn row_real(row: &[sqlite::Value], index: usize, label: &str) -> anyhow::Result<f64> {
    match row.get(index) {
        Some(sqlite::Value::Real(value)) => Ok(*value),
        Some(sqlite::Value::Integer(value)) => Ok(*value as f64),
        other => bail!("{label} is not an SQLite number: {other:?}"),
    }
}

fn row_text<'a>(row: &'a [sqlite::Value], index: usize, label: &str) -> anyhow::Result<&'a str> {
    match row.get(index) {
        Some(sqlite::Value::Text(value)) => Ok(value),
        other => bail!("{label} is not SQLite text: {other:?}"),
    }
}

fn row_blob<'a>(row: &'a [sqlite::Value], index: usize, label: &str) -> anyhow::Result<&'a [u8]> {
    match row.get(index) {
        Some(sqlite::Value::Blob(value)) => Ok(value),
        other => bail!("{label} is not an SQLite blob: {other:?}"),
    }
}

fn validate_identifier(name: &str) -> anyhow::Result<()> {
    let mut characters = name.chars();
    let valid_first = characters
        .next()
        .is_some_and(|value| value == '_' || value.is_ascii_alphabetic());
    if !valid_first || !characters.all(|value| value == '_' || value.is_ascii_alphanumeric()) {
        bail!("invalid memory2 stream identifier {name:?}");
    }
    Ok(())
}

fn quote_identifier(name: &str) -> anyhow::Result<String> {
    validate_identifier(name)?;
    Ok(format!("\"{name}\""))
}

mod sqlite {
    use std::ffi::{c_char, c_int, c_uchar, c_void, CStr, CString};
    use std::path::Path;
    use std::ptr;

    use anyhow::{bail, Context};

    const SQLITE_OK: c_int = 0;
    const SQLITE_ROW: c_int = 100;
    const SQLITE_DONE: c_int = 101;
    const SQLITE_INTEGER: c_int = 1;
    const SQLITE_FLOAT: c_int = 2;
    const SQLITE_TEXT: c_int = 3;
    const SQLITE_BLOB: c_int = 4;
    const SQLITE_NULL: c_int = 5;
    const SQLITE_OPEN_READONLY: c_int = 0x0000_0001;
    const SQLITE_OPEN_READWRITE: c_int = 0x0000_0002;
    const SQLITE_OPEN_CREATE: c_int = 0x0000_0004;

    #[allow(non_camel_case_types)]
    #[repr(C)]
    struct sqlite3 {
        _private: [u8; 0],
    }

    #[allow(non_camel_case_types)]
    #[repr(C)]
    struct sqlite3_stmt {
        _private: [u8; 0],
    }

    #[allow(non_camel_case_types)]
    #[repr(C)]
    struct sqlite3_backup {
        _private: [u8; 0],
    }

    #[link(name = "sqlite3")]
    extern "C" {
        fn sqlite3_open_v2(
            filename: *const c_char,
            database: *mut *mut sqlite3,
            flags: c_int,
            vfs: *const c_char,
        ) -> c_int;
        fn sqlite3_close_v2(database: *mut sqlite3) -> c_int;
        fn sqlite3_errmsg(database: *mut sqlite3) -> *const c_char;
        fn sqlite3_prepare_v2(
            database: *mut sqlite3,
            sql: *const c_char,
            bytes: c_int,
            statement: *mut *mut sqlite3_stmt,
            tail: *mut *const c_char,
        ) -> c_int;
        fn sqlite3_step(statement: *mut sqlite3_stmt) -> c_int;
        fn sqlite3_finalize(statement: *mut sqlite3_stmt) -> c_int;
        fn sqlite3_column_count(statement: *mut sqlite3_stmt) -> c_int;
        fn sqlite3_column_type(statement: *mut sqlite3_stmt, column: c_int) -> c_int;
        fn sqlite3_column_int64(statement: *mut sqlite3_stmt, column: c_int) -> i64;
        fn sqlite3_column_double(statement: *mut sqlite3_stmt, column: c_int) -> f64;
        fn sqlite3_column_text(statement: *mut sqlite3_stmt, column: c_int) -> *const c_uchar;
        fn sqlite3_column_blob(statement: *mut sqlite3_stmt, column: c_int) -> *const c_void;
        fn sqlite3_column_bytes(statement: *mut sqlite3_stmt, column: c_int) -> c_int;
        fn sqlite3_bind_int64(statement: *mut sqlite3_stmt, index: c_int, value: i64) -> c_int;
        fn sqlite3_bind_double(statement: *mut sqlite3_stmt, index: c_int, value: f64) -> c_int;
        fn sqlite3_bind_text(
            statement: *mut sqlite3_stmt,
            index: c_int,
            value: *const c_char,
            bytes: c_int,
            destructor: Option<unsafe extern "C" fn(*mut c_void)>,
        ) -> c_int;
        fn sqlite3_backup_init(
            destination: *mut sqlite3,
            destination_name: *const c_char,
            source: *mut sqlite3,
            source_name: *const c_char,
        ) -> *mut sqlite3_backup;
        fn sqlite3_backup_step(backup: *mut sqlite3_backup, pages: c_int) -> c_int;
        fn sqlite3_backup_finish(backup: *mut sqlite3_backup) -> c_int;
    }

    /// SQLite value copied out of a prepared statement row.
    #[derive(Debug, Clone, PartialEq)]
    pub enum Value {
        Null,
        Integer(i64),
        Real(f64),
        Text(String),
        Blob(Vec<u8>),
    }

    impl Value {
        pub fn as_text(&self) -> Option<&str> {
            match self {
                Self::Text(value) => Some(value),
                Self::Null | Self::Integer(_) | Self::Real(_) | Self::Blob(_) => None,
            }
        }
    }

    /// Borrowed SQLite bind parameter.
    pub enum Bind<'a> {
        Integer(i64),
        Real(f64),
        Text(&'a str),
    }

    pub struct Connection {
        handle: *mut sqlite3,
    }

    impl Connection {
        pub fn open_read_only(path: &Path) -> anyhow::Result<Self> {
            Self::open(path, SQLITE_OPEN_READONLY)
        }

        pub fn open_read_write_create(path: &Path) -> anyhow::Result<Self> {
            Self::open(path, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE)
        }

        fn open(path: &Path, flags: c_int) -> anyhow::Result<Self> {
            let path_text = path.to_str().ok_or_else(|| {
                anyhow::anyhow!("SQLite path is not valid UTF-8: {}", path.display())
            })?;
            let path_c = CString::new(path_text).context("SQLite path contains NUL")?;
            let mut handle = ptr::null_mut();
            // SAFETY: `path_c` is NUL-terminated, `handle` is writable, and the
            // returned database pointer is exclusively owned by `Connection`.
            let code = unsafe { sqlite3_open_v2(path_c.as_ptr(), &mut handle, flags, ptr::null()) };
            if code != SQLITE_OK || handle.is_null() {
                let message = if handle.is_null() {
                    format!("SQLite open failed with code {code}")
                } else {
                    // SAFETY: SQLite owns a valid error string for an open handle.
                    unsafe { CStr::from_ptr(sqlite3_errmsg(handle)) }
                        .to_string_lossy()
                        .into_owned()
                };
                if !handle.is_null() {
                    // SAFETY: `handle` came from sqlite3_open_v2 and is not used again.
                    unsafe { sqlite3_close_v2(handle) };
                }
                bail!("open SQLite {} failed: {message}", path.display());
            }
            Ok(Self { handle })
        }

        pub fn query(&self, sql: &str, parameters: &[Bind<'_>]) -> anyhow::Result<Vec<Vec<Value>>> {
            let mut statement = self.prepare(sql)?;
            statement.bind(parameters)?;
            let mut rows = Vec::new();
            loop {
                // SAFETY: the prepared statement is live and exclusively stepped here.
                match unsafe { sqlite3_step(statement.handle) } {
                    SQLITE_ROW => rows.push(statement.copy_row()?),
                    SQLITE_DONE => break,
                    _ => return Err(self.error(sql)),
                }
            }
            Ok(rows)
        }

        pub fn execute(&self, sql: &str, parameters: &[Bind<'_>]) -> anyhow::Result<()> {
            let mut statement = self.prepare(sql)?;
            statement.bind(parameters)?;
            // SAFETY: the prepared statement is live and exclusively stepped here.
            let code = unsafe { sqlite3_step(statement.handle) };
            if code != SQLITE_DONE {
                return Err(self.error(sql));
            }
            Ok(())
        }

        pub fn table_exists(&self, name: &str) -> anyhow::Result<bool> {
            Ok(!self
                .query(
                    "SELECT 1 FROM sqlite_master WHERE type IN ('table','view') AND name = ? LIMIT 1",
                    &[Bind::Text(name)],
                )?
                .is_empty())
        }

        pub fn backup_from(&self, source: &Self) -> anyhow::Result<()> {
            let main = c"main";
            // SAFETY: both connections are open and `main` is NUL-terminated.
            let backup = unsafe {
                sqlite3_backup_init(self.handle, main.as_ptr(), source.handle, main.as_ptr())
            };
            if backup.is_null() {
                return Err(self.error("sqlite3_backup_init"));
            }
            // SAFETY: `backup` is live until the matching finish call.
            let step_code = unsafe { sqlite3_backup_step(backup, -1) };
            // SAFETY: every successful backup_init is finished exactly once.
            let finish_code = unsafe { sqlite3_backup_finish(backup) };
            if step_code != SQLITE_DONE || finish_code != SQLITE_OK {
                return Err(self.error("sqlite3_backup"));
            }
            Ok(())
        }

        fn prepare(&self, sql: &str) -> anyhow::Result<Statement<'_>> {
            let sql_c = CString::new(sql).context("SQLite statement contains NUL")?;
            let mut handle = ptr::null_mut();
            // SAFETY: the connection and SQL string are valid for this call;
            // SQLite returns an owned statement finalized by `Statement`.
            let code = unsafe {
                sqlite3_prepare_v2(
                    self.handle,
                    sql_c.as_ptr(),
                    -1,
                    &mut handle,
                    ptr::null_mut(),
                )
            };
            if code != SQLITE_OK || handle.is_null() {
                return Err(self.error(sql));
            }
            Ok(Statement {
                connection: self,
                handle,
            })
        }

        fn error(&self, sql: &str) -> anyhow::Error {
            // SAFETY: an open SQLite connection always exposes a stable error string.
            let message = unsafe { CStr::from_ptr(sqlite3_errmsg(self.handle)) }.to_string_lossy();
            anyhow::anyhow!("SQLite statement failed: {message}; sql={sql}")
        }
    }

    impl Drop for Connection {
        fn drop(&mut self) {
            // SAFETY: `Connection` owns the handle and drops it exactly once.
            unsafe { sqlite3_close_v2(self.handle) };
        }
    }

    struct Statement<'a> {
        connection: &'a Connection,
        handle: *mut sqlite3_stmt,
    }

    impl Statement<'_> {
        fn bind(&mut self, parameters: &[Bind<'_>]) -> anyhow::Result<()> {
            for (offset, parameter) in parameters.iter().enumerate() {
                let index = c_int::try_from(offset + 1).context("too many SQLite parameters")?;
                // All borrowed parameter bytes outlive statement execution. A
                // null destructor therefore requests SQLITE_STATIC semantics.
                let code = match parameter {
                    Bind::Integer(value) => {
                        // SAFETY: statement and one-based bind index are valid.
                        unsafe { sqlite3_bind_int64(self.handle, index, *value) }
                    }
                    Bind::Real(value) => {
                        // SAFETY: statement and one-based bind index are valid.
                        unsafe { sqlite3_bind_double(self.handle, index, *value) }
                    }
                    Bind::Text(value) => {
                        let bytes = c_int::try_from(value.len())
                            .context("SQLite text parameter exceeds c_int")?;
                        // SAFETY: `value` remains alive until the query/execute
                        // call completes and SQLite receives its explicit length.
                        unsafe {
                            sqlite3_bind_text(
                                self.handle,
                                index,
                                value.as_ptr().cast(),
                                bytes,
                                None,
                            )
                        }
                    }
                };
                if code != SQLITE_OK {
                    return Err(self.connection.error("bind parameter"));
                }
            }
            Ok(())
        }

        fn copy_row(&self) -> anyhow::Result<Vec<Value>> {
            // SAFETY: the statement currently points at SQLITE_ROW.
            let count = unsafe { sqlite3_column_count(self.handle) };
            let mut row = Vec::with_capacity(usize::try_from(count).unwrap_or(0));
            for column in 0..count {
                // SAFETY: `column` is within sqlite3_column_count.
                let value_type = unsafe { sqlite3_column_type(self.handle, column) };
                let value = match value_type {
                    SQLITE_NULL => Value::Null,
                    SQLITE_INTEGER => {
                        // SAFETY: conversion is valid for an INTEGER column.
                        Value::Integer(unsafe { sqlite3_column_int64(self.handle, column) })
                    }
                    SQLITE_FLOAT => {
                        // SAFETY: conversion is valid for a FLOAT column.
                        Value::Real(unsafe { sqlite3_column_double(self.handle, column) })
                    }
                    SQLITE_TEXT => {
                        // SAFETY: SQLite owns a buffer valid until the next step/finalize.
                        let pointer = unsafe { sqlite3_column_text(self.handle, column) };
                        // SAFETY: byte length corresponds to this column buffer.
                        let length = unsafe { sqlite3_column_bytes(self.handle, column) };
                        let length =
                            usize::try_from(length).context("negative SQLite text length")?;
                        if pointer.is_null() && length != 0 {
                            bail!("SQLite returned a null text pointer with non-zero length");
                        }
                        let bytes = if length == 0 {
                            &[][..]
                        } else {
                            // SAFETY: non-null pointer/length are supplied by SQLite.
                            unsafe { std::slice::from_raw_parts(pointer, length) }
                        };
                        Value::Text(
                            std::str::from_utf8(bytes)
                                .context("SQLite text is not UTF-8")?
                                .to_string(),
                        )
                    }
                    SQLITE_BLOB => {
                        // SAFETY: SQLite owns a buffer valid until the next step/finalize.
                        let pointer = unsafe { sqlite3_column_blob(self.handle, column) };
                        // SAFETY: byte length corresponds to this column buffer.
                        let length = unsafe { sqlite3_column_bytes(self.handle, column) };
                        let length =
                            usize::try_from(length).context("negative SQLite blob length")?;
                        if pointer.is_null() && length != 0 {
                            bail!("SQLite returned a null blob pointer with non-zero length");
                        }
                        let bytes = if length == 0 {
                            &[][..]
                        } else {
                            // SAFETY: non-null pointer/length are supplied by SQLite.
                            unsafe { std::slice::from_raw_parts(pointer.cast::<u8>(), length) }
                        };
                        Value::Blob(bytes.to_vec())
                    }
                    other => bail!("unsupported SQLite column type {other}"),
                };
                row.push(value);
            }
            Ok(row)
        }
    }

    impl Drop for Statement<'_> {
        fn drop(&mut self) {
            // SAFETY: `Statement` owns the prepared handle and finalizes once.
            unsafe { sqlite3_finalize(self.handle) };
        }
    }
}
