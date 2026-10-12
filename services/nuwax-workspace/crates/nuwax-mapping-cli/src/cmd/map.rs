//! Build and durably publish an immutable offline map asset bundle.

use clap::Args;
use nuwax_mapping::cost_mapper::{CostMapper, CostMapperConfig};
use nuwax_mapping::occupancy::types::OccupancyGrid;
use serde_json::{json, Map, Value};
use std::collections::HashSet;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};

use crate::artifact::publish_map_bundle;
use crate::dataset::RecordPayload;
use crate::native::read_command_dataset;

const ASSET_SCHEMA: &str = "nuwax.mapping.offline-map-asset.v2";
const MANIFEST_SCHEMA: &str = "nuwax.mapping.offline-map-manifest.v2";
const PGM_ENCODING: &str = "pgm-p5-cost-inverted-v1";

/// Arguments for the `map` command.
#[derive(Debug, Args)]
pub struct MapArgs {
    /// Input JSONL, NDJSON, or DimOS memory2 `.db` dataset.
    #[arg(long)]
    pub bag: PathBuf,
    /// Existing parent directory for the canonical map bundle.
    #[arg(long)]
    pub output_parent: PathBuf,
    /// Stable business identity of the map.
    #[arg(long)]
    pub map_name: String,
    /// Canonical frame shared by the dataset and bundle.
    #[arg(long)]
    pub frame_id: String,
    /// Optional original owner request identity; omitted for standalone export.
    #[arg(long)]
    pub request_id: Option<String>,
    /// 可选发布版本身份；源码离线建图无需提供。
    #[arg(long)]
    pub release_generation: Option<String>,
    /// 可选发布清单摘要，提供时记录到地图来源。
    #[arg(long)]
    pub release_manifest_sha256: Option<String>,
    /// 可选建图程序摘要，提供时记录到地图来源。
    #[arg(long)]
    pub artifact_sha256: Option<String>,
    /// Display theme label retained for CLI compatibility.
    #[arg(long, default_value = "default")]
    pub theme: String,
    /// Raster resolution used when the input contains point clouds only.
    #[arg(long, default_value_t = 0.1)]
    pub resolution: f64,
}

/// Execute the `map` command.
///
/// # Errors
/// Returns input schema, identity, publication, or output validation errors.
pub async fn run(args: MapArgs) -> anyhow::Result<()> {
    if !args.resolution.is_finite() || args.resolution <= 0.0 {
        anyhow::bail!("resolution must be finite and positive");
    }
    let map_name = require_identity(&args.map_name, "map-name")?;
    let frame_id = require_identity(&args.frame_id, "frame-id")?;
    if let Some(request_id) = args.request_id.as_deref() {
        require_request_id(request_id)?;
    }
    producer_metadata(
        args.release_generation.as_deref(),
        args.release_manifest_sha256.as_deref(),
        args.artifact_sha256.as_deref(),
    )?;
    if !args.output_parent.is_dir() {
        anyhow::bail!(
            "output parent must be an existing directory: {}",
            args.output_parent.display()
        );
    }
    let output = args.output_parent.join(safe_map_segment(map_name));

    let input_format = input_format(&args.bag)?;
    let input = File::open(&args.bag)?;
    let observation = InputObservation::from_metadata(&input.metadata()?)?;
    // The existing decoder reopens the path (JSONL File or SQLite connection).
    // Bind its surrounding observations to the held input without rereading bytes.
    let decoded = read_command_dataset(&args.bag)?;
    observation.verify_metadata(&input.metadata()?)?;
    observation.verify_path(&args.bag)?;
    report_skipped_streams(&decoded.skipped_streams);
    let dataset = decoded.dataset;
    let costmap = if let Some(record) = dataset
        .records
        .iter()
        .filter(|record| matches!(record.payload, RecordPayload::OccupancyGrid { .. }))
        .max_by(|left, right| left.timestamp.total_cmp(&right.timestamp))
    {
        costmap_from_occupancy_record(record, frame_id)?
    } else {
        costmap_from_point_records(&dataset.records, args.resolution, frame_id)?
    };

    let asset = canonical_asset(
        map_name,
        frame_id,
        &observation,
        input_format,
        args.request_id.as_deref(),
        &costmap,
        args.release_generation.as_deref(),
        args.release_manifest_sha256.as_deref(),
        args.artifact_sha256.as_deref(),
    )?;
    publish_map_bundle(
        &output,
        |staging| write_bundle(staging, &costmap, &asset, map_name, frame_id, args.request_id.as_deref()),
        |staging| {
            validate_bundle(
                staging,
                map_name,
                frame_id,
                &observation,
                input_format,
                args.request_id.as_deref(),
                args.release_generation.as_deref(),
                args.release_manifest_sha256.as_deref(),
                args.artifact_sha256.as_deref(),
            )
        },
    )?;
    let request_label = args.request_id.as_deref()
        .map(|request_id| format!(" request_id={request_id}"))
        .unwrap_or_default();
    println!(
        "[map] dataset={} theme={} records={} costmap={}x{} output={}{}",
        args.bag.display(),
        args.theme,
        dataset.records.len(),
        costmap.width(),
        costmap.height(),
        output.display(),
        request_label
    );
    Ok(())
}

fn require_request_id(value: &str) -> anyhow::Result<()> {
    let bytes = value.as_bytes();
    if bytes.is_empty() || bytes.len() > 128 || !bytes[0].is_ascii_alphanumeric()
        || !bytes.iter().all(|byte| byte.is_ascii_alphanumeric() || b"_.:-".contains(byte))
    {
        anyhow::bail!("request-id must match [A-Za-z0-9][A-Za-z0-9_.:-]{{0,127}}");
    }
    Ok(())
}

#[derive(Debug, PartialEq, Eq)]
struct InputObservation {
    size: u64,
    device: u64,
    inode: u64,
    mtime_ns: i64,
    ctime_ns: i64,
}

fn checked_timestamp_ns(seconds: i64, nanoseconds: i64) -> anyhow::Result<i64> {
    Ok(i64::try_from(i128::from(seconds) * 1_000_000_000 + i128::from(nanoseconds))?)
}

impl InputObservation {
    #[cfg(target_os = "linux")]
    fn from_metadata(metadata: &fs::Metadata) -> anyhow::Result<Self> {
        use std::os::linux::fs::MetadataExt;
        if !metadata.is_file() || metadata.len() == 0 {
            anyhow::bail!("map input must be a nonempty regular file");
        }
        Ok(Self {
            size: metadata.len(),
            device: metadata.st_dev(),
            inode: metadata.st_ino(),
            mtime_ns: checked_timestamp_ns(metadata.st_mtime(), metadata.st_mtime_nsec())?,
            ctime_ns: checked_timestamp_ns(metadata.st_ctime(), metadata.st_ctime_nsec())?,
        })
    }

    #[cfg(not(target_os = "linux"))]
    fn from_metadata(_metadata: &fs::Metadata) -> anyhow::Result<Self> {
        anyhow::bail!("map input observations require Linux file metadata");
    }

    fn verify_metadata(&self, metadata: &fs::Metadata) -> anyhow::Result<()> {
        if *self != Self::from_metadata(metadata)? {
            anyhow::bail!("map input identity or metadata changed during decode");
        }
        Ok(())
    }

    fn verify_path(&self, path: &Path) -> anyhow::Result<()> {
        let fresh = File::open(path)?;
        self.verify_metadata(&fresh.metadata()?)
    }

    fn source(&self, input_format: &str) -> Value {
        json!({
            "dataset_size": self.size,
            "input_format": input_format,
            "dataset_device": self.device,
            "dataset_inode": self.inode,
            "dataset_mtime_ns": self.mtime_ns,
            "dataset_ctime_ns": self.ctime_ns
        })
    }
}

fn require_identity<'a>(value: &'a str, label: &str) -> anyhow::Result<&'a str> {
    if value.is_empty()
        || value.trim() != value
        || value
            .bytes()
            .any(|byte| byte == 0 || byte == b'\n' || byte == b'\r')
    {
        anyhow::bail!("{label} must be a non-empty canonical UTF-8 identity");
    }
    Ok(value)
}

fn require_digest(value: &str, label: &str) -> anyhow::Result<()> {
    if value.len() != 64
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        anyhow::bail!("{label} must be a 64-character lowercase SHA-256 digest");
    }
    Ok(())
}

fn safe_map_segment(map_name: &str) -> String {
    let mut segment = String::from("map-");
    for byte in map_name.as_bytes() {
        segment.push_str(&format!("{byte:02x}"));
    }
    segment
}

fn input_format(path: &Path) -> anyhow::Result<&'static str> {
    match path.extension().and_then(|value| value.to_str()) {
        Some("jsonl") => Ok("jsonl"),
        Some("ndjson") => Ok("ndjson"),
        Some("db") => Ok("memory2_sqlite"),
        _ => anyhow::bail!("map dataset must use .jsonl, .ndjson, or .db extension"),
    }
}

fn report_skipped_streams(streams: &[String]) {
    if !streams.is_empty() {
        eprintln!(
            "[map] skipped unsupported memory2 streams: {}",
            streams.join(",")
        );
    }
}

fn costmap_from_occupancy_record(
    record: &crate::dataset::DatasetRecord,
    required_frame: &str,
) -> anyhow::Result<nuwax_mapping::Costmap> {
    let RecordPayload::OccupancyGrid {
        width,
        height,
        resolution,
        origin,
        origin_orientation,
        frame_id,
        cells,
    } = &record.payload
    else {
        anyhow::bail!("selected record is not an occupancy grid");
    };
    if frame_id != required_frame {
        anyhow::bail!(
            "occupancy-grid frame {frame_id:?} does not match --frame-id {required_frame:?}"
        );
    }
    let grid = OccupancyGrid {
        data: cells.clone(),
        width: *width,
        height: *height,
        resolution: *resolution,
        origin: *origin,
        origin_orientation: *origin_orientation,
        frame_id: frame_id.clone(),
        ts: record.timestamp,
    };
    Ok(CostMapper::new(CostMapperConfig::dimos_default()).convert(&grid))
}

fn costmap_from_point_records(
    _records: &[crate::dataset::DatasetRecord],
    _resolution: f64,
    _required_frame: &str,
) -> anyhow::Result<nuwax_mapping::Costmap> {
    anyhow::bail!("点云记录缺少逐帧配对雷达原点；请使用几何离线重放生成占据图后再导出地图")
}

fn producer_metadata(
    release_generation: Option<&str>,
    release_manifest_sha256: Option<&str>,
    artifact_sha256: Option<&str>,
) -> anyhow::Result<Value> {
    let mut producer = json!({"role": "offline_mapping_cli"});
    // 缺失的发布身份保持缺失，不以地图内容或源码摘要冒充发布来源。
    for (name, value) in [
        ("release_generation", release_generation),
        ("release_manifest_sha256", release_manifest_sha256),
        ("artifact_sha256", artifact_sha256),
    ] {
        if let Some(value) = value {
            require_digest(value, name)?;
            producer[name] = Value::String(value.to_string());
        }
    }
    Ok(producer)
}

fn canonical_asset(
    map_name: &str,
    frame_id: &str,
    observation: &InputObservation,
    input_format: &str,
    request_id: Option<&str>,
    costmap: &nuwax_mapping::Costmap,
    release_generation: Option<&str>,
    release_manifest_sha256: Option<&str>,
    artifact_sha256: Option<&str>,
) -> anyhow::Result<Vec<u8>> {
    let translation = costmap.metadata.origin.translation.vector;
    let (_, _, yaw) = costmap.metadata.origin.rotation.euler_angles();
    let mut asset = json!({
        "schema_version": ASSET_SCHEMA,
        "map_name": map_name,
        "frame_id": frame_id,
        "source": observation.source(input_format),
        "grid": {
            "width": costmap.width(),
            "height": costmap.height(),
            "resolution_m": costmap.metadata.resolution,
            "origin_xy_yaw": [translation.x, translation.y, yaw],
            "encoding": PGM_ENCODING,
            "path": "map.pgm"
        },
        "producer": producer_metadata(release_generation, release_manifest_sha256, artifact_sha256)?
    });
    if let Some(request_id) = request_id {
        asset["request_id"] = Value::String(request_id.to_string());
    }
    canonical_json(&asset)
}

fn canonical_json(value: &Value) -> anyhow::Result<Vec<u8>> {
    fn sorted(value: &Value) -> Value {
        match value {
            Value::Object(values) => {
                let mut keys: Vec<&String> = values.keys().collect();
                keys.sort();
                let mut result = Map::new();
                for key in keys {
                    result.insert(key.clone(), sorted(&values[key]));
                }
                Value::Object(result)
            }
            Value::Array(values) => Value::Array(values.iter().map(sorted).collect()),
            _ => value.clone(),
        }
    }
    let mut encoded = serde_json::to_vec(&sorted(value))?;
    encoded.push(b'\n');
    Ok(encoded)
}

fn write_bundle(
    staging: &Path,
    costmap: &nuwax_mapping::Costmap,
    asset: &[u8],
    map_name: &str,
    frame_id: &str,
    request_id: Option<&str>,
) -> anyhow::Result<()> {
    let pgm_path = staging.join("map.pgm");
    write_pgm(&pgm_path, costmap)?;
    let asset_path = staging.join("asset.json");
    write_synced_file(&asset_path, asset)?;
    validate_pgm(&pgm_path, costmap)?;
    if fs::read(&asset_path)? != asset {
        anyhow::bail!("asset.json changed after durable write");
    }
    let files = [
        file_entry(&asset_path, "asset.json")?,
        file_entry(&pgm_path, "map.pgm")?,
    ];
    let mut manifest = json!({
        "schema_version": MANIFEST_SCHEMA,
        "map_name": map_name,
        "frame_id": frame_id,
        "asset_path": "asset.json",
        "files": files
    });
    if let Some(request_id) = request_id {
        manifest["request_id"] = Value::String(request_id.to_string());
    }
    write_synced_file(&staging.join("manifest.json"), &canonical_json(&manifest)?)?;
    Ok(())
}

fn file_entry(path: &Path, relative: &str) -> anyhow::Result<Value> {
    let metadata = fs::metadata(path)?;
    Ok(json!({
        "path": relative,
        "size": metadata.len()
    }))
}

fn write_synced_file(path: &Path, content: &[u8]) -> anyhow::Result<()> {
    let mut file = File::create(path)?;
    file.write_all(content)?;
    file.flush()?;
    file.sync_all()?;
    drop(file);
    Ok(())
}

fn write_pgm(path: &Path, costmap: &nuwax_mapping::Costmap) -> anyhow::Result<()> {
    let file = File::create(path)?;
    let mut writer = BufWriter::with_capacity(8 * 1024, file);
    write_pgm_to(&mut writer, costmap)?;
    writer.flush()?;
    writer.get_ref().sync_all()?;
    drop(writer);
    Ok(())
}

fn write_pgm_to(writer: &mut impl Write, costmap: &nuwax_mapping::Costmap) -> std::io::Result<()> {
    writeln!(writer, "P5")?;
    writeln!(writer, "{} {}", costmap.width(), costmap.height())?;
    writeln!(writer, "255")?;
    for row in 0..costmap.height() {
        for column in 0..costmap.width() {
            let cost = costmap.get(row, column).unwrap_or(0);
            writer.write_all(&[255_u8.saturating_sub(cost)])?;
        }
    }
    Ok(())
}

fn validate_pgm(path: &Path, costmap: &nuwax_mapping::Costmap) -> anyhow::Result<()> {
    let mut reader = BufReader::new(File::open(path)?);
    let expected_headers = [
        "P5\n".to_string(),
        format!("{} {}\n", costmap.width(), costmap.height()),
        "255\n".to_string(),
    ];
    let mut line = String::new();
    for expected in expected_headers {
        line.clear();
        reader.read_line(&mut line)?;
        if line != expected {
            anyhow::bail!("invalid PGM header in {}", path.display());
        }
    }
    let width = usize::try_from(costmap.width())?;
    let mut persisted_row = vec![0_u8; width];
    for row in 0..costmap.height() {
        reader.read_exact(&mut persisted_row)?;
        for (column, persisted) in persisted_row.iter().copied().enumerate() {
            let column = u32::try_from(column)?;
            let expected = 255_u8.saturating_sub(costmap.get(row, column).unwrap_or(0));
            if persisted != expected {
                anyhow::bail!("PGM payload mismatch at row {row}, column {column}");
            }
        }
    }
    let mut trailing = [0_u8; 1];
    if reader.read(&mut trailing)? != 0 {
        anyhow::bail!("PGM contains trailing payload bytes: {}", path.display());
    }
    Ok(())
}

fn exact_object<'a>(
    value: &'a Value,
    expected_fields: &[&str],
    label: &str,
) -> anyhow::Result<&'a Map<String, Value>> {
    let object = value
        .as_object()
        .ok_or_else(|| anyhow::anyhow!("{label} is not an object"))?;
    if object.len() != expected_fields.len()
        || !expected_fields
            .iter()
            .all(|field| object.contains_key(*field))
    {
        anyhow::bail!("{label} field set is not exact");
    }
    Ok(object)
}

fn validate_bundle(
    staging: &Path,
    map_name: &str,
    frame_id: &str,
    observation: &InputObservation,
    input_format: &str,
    request_id: Option<&str>,
    release_generation: Option<&str>,
    release_manifest_sha256: Option<&str>,
    artifact_sha256: Option<&str>,
) -> anyhow::Result<()> {
    let expected: HashSet<&str> = ["asset.json", "manifest.json", "map.pgm"]
        .into_iter()
        .collect();
    let actual: HashSet<String> = fs::read_dir(staging)?
        .map(|entry| entry.map(|value| value.file_name().to_string_lossy().into_owned()))
        .collect::<Result<_, _>>()?;
    if actual != expected.iter().map(|value| (*value).to_string()).collect() {
        anyhow::bail!("map bundle member set is not exact");
    }
    let manifest_bytes = fs::read(staging.join("manifest.json"))?;
    let manifest: Value = serde_json::from_slice(&manifest_bytes)?;
    if canonical_json(&manifest)? != manifest_bytes
        || manifest.get("schema_version") != Some(&Value::String(MANIFEST_SCHEMA.to_string()))
        || manifest.get("map_name") != Some(&Value::String(map_name.to_string()))
        || manifest.get("frame_id") != Some(&Value::String(frame_id.to_string()))
    {
        anyhow::bail!("map manifest identity or canonical encoding is invalid");
    }
    let mut manifest_fields = vec!["schema_version", "map_name", "frame_id", "asset_path", "files"];
    if request_id.is_some() { manifest_fields.push("request_id"); }
    exact_object(&manifest, &manifest_fields, "map manifest")?;
    if manifest.get("request_id") != request_id.map(|id| Value::String(id.to_string())).as_ref()
        || manifest.get("asset_path").and_then(Value::as_str) != Some("asset.json")
    {
        anyhow::bail!("map manifest request or asset path is inconsistent");
    }
    let files = manifest
        .get("files")
        .and_then(Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("manifest files is not an array"))?;
    if files.len() != 2 {
        anyhow::bail!("manifest files must be exact and byte-sorted");
    }
    for (entry, path) in files.iter().zip(["asset.json", "map.pgm"]) {
        exact_object(entry, &["path", "size"], "manifest file entry")?;
        if entry.get("path").and_then(Value::as_str) != Some(path)
            || entry.get("size").and_then(Value::as_u64) != Some(fs::metadata(staging.join(path))?.len())
        {
            anyhow::bail!("manifest member path or size is inconsistent for {path}");
        }
    }
    let asset_bytes = fs::read(staging.join("asset.json"))?;
    let asset: Value = serde_json::from_slice(&asset_bytes)?;
    if canonical_json(&asset)? != asset_bytes {
        anyhow::bail!("map asset is not canonical JSON");
    }
    let mut asset_fields = vec!["schema_version", "map_name", "frame_id", "source", "grid", "producer"];
    if request_id.is_some() { asset_fields.push("request_id"); }
    let asset = exact_object(&asset, &asset_fields, "map asset")?;
    if asset.get("request_id") != request_id.map(|id| Value::String(id.to_string())).as_ref() {
        anyhow::bail!("map asset request identity is inconsistent");
    }
    if asset.get("schema_version") != Some(&Value::String(ASSET_SCHEMA.to_string()))
        || asset.get("map_name") != Some(&Value::String(map_name.to_string()))
        || asset.get("frame_id") != Some(&Value::String(frame_id.to_string()))
    {
        anyhow::bail!("map asset identity is inconsistent");
    }

    if asset["source"] != observation.source(input_format) {
        anyhow::bail!("map asset input observation is inconsistent");
    }

    let grid = exact_object(
        &asset["grid"],
        &[
            "width",
            "height",
            "resolution_m",
            "origin_xy_yaw",
            "encoding",
            "path",
        ],
        "map asset grid",
    )?;
    let width = grid.get("width").and_then(Value::as_u64);
    let height = grid.get("height").and_then(Value::as_u64);
    let resolution = grid.get("resolution_m").and_then(Value::as_f64);
    let origin = grid.get("origin_xy_yaw").and_then(Value::as_array);
    if !width.is_some_and(|value| value > 0 && value <= u64::from(u32::MAX))
        || !height.is_some_and(|value| value > 0 && value <= u64::from(u32::MAX))
        || !resolution.is_some_and(|value| value.is_finite() && value > 0.0)
        || !origin.is_some_and(|values| {
            values.len() == 3
                && values
                    .iter()
                    .all(|value| value.as_f64().is_some_and(f64::is_finite))
        })
        || grid.get("encoding") != Some(&Value::String(PGM_ENCODING.to_string()))
        || grid.get("path") != Some(&Value::String("map.pgm".to_string()))
    {
        anyhow::bail!("map asset grid is invalid");
    }

    if asset["producer"]
        != producer_metadata(release_generation, release_manifest_sha256, artifact_sha256)?
    {
        anyhow::bail!("map asset producer proof is inconsistent");
    }
    Ok(())
}


#[cfg(all(test, target_os = "linux"))]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static NEXT_DIR: AtomicU64 = AtomicU64::new(0);
    const DATASET: &[u8] = b"{\"format\":\"nuwax-mapping-jsonl\",\"version\":1}\n{\"timestamp\":1.0,\"stream\":\"map\",\"type\":\"occupancy_grid\",\"width\":2,\"height\":2,\"resolution\":0.1,\"origin\":[0,0,0],\"origin_orientation\":[0,0,0,1],\"frame_id\":\"map\",\"cells\":[-1,0,50,100]}\n";

    struct TestDir(PathBuf);
    impl TestDir {
        fn new() -> Self {
            for _ in 0..32 {
                let path = std::env::temp_dir().join(format!("nuwax-map-schema2-{}-{}",
                    std::process::id(), NEXT_DIR.fetch_add(1, Ordering::Relaxed)));
                match fs::create_dir(&path) {
                    Ok(()) => return Self(path),
                    Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                    Err(error) => panic!("create map test directory: {error}"),
                }
            }
            panic!("could not allocate map test directory");
        }
        fn dataset(&self) -> PathBuf {
            let path = self.0.join("input.jsonl");
            fs::write(&path, DATASET).unwrap();
            path
        }
    }
    impl Drop for TestDir {
        fn drop(&mut self) { let _ = fs::remove_dir_all(&self.0); }
    }

    fn args(bag: &Path, output_parent: &Path, request_id: Option<&str>) -> MapArgs {
        MapArgs {
            bag: bag.to_path_buf(), output_parent: output_parent.to_path_buf(),
            map_name: "真实-map".to_string(), frame_id: "map".to_string(),
            request_id: request_id.map(str::to_string), release_generation: None,
            release_manifest_sha256: None, artifact_sha256: None,
            theme: "default".to_string(), resolution: 0.1,
        }
    }

    #[tokio::test]
    async fn map_schema2_standalone_and_original_request_ids_roundtrip() {
        let directory = TestDir::new();
        let bag = directory.dataset();
        let observation = InputObservation::from_metadata(&File::open(&bag).unwrap().metadata().unwrap()).unwrap();
        let longest = format!("R{}", ":".repeat(127));
        for (index, request_id) in [None, Some("Job:A_1.-"), Some(longest.as_str())].into_iter().enumerate() {
            let output_parent = directory.0.join(format!("out-{index}"));
            fs::create_dir(&output_parent).unwrap();
            run(args(&bag, &output_parent, request_id)).await.unwrap();
            let output = output_parent.join(safe_map_segment("真实-map"));
            let asset: Value = serde_json::from_slice(&fs::read(output.join("asset.json")).unwrap()).unwrap();
            let manifest: Value = serde_json::from_slice(&fs::read(output.join("manifest.json")).unwrap()).unwrap();
            assert_eq!(asset["schema_version"], ASSET_SCHEMA);
            assert_eq!(manifest["schema_version"], MANIFEST_SCHEMA);
            assert_eq!(asset.get("request_id").and_then(Value::as_str), request_id);
            assert_eq!(manifest.get("request_id").and_then(Value::as_str), request_id);
            if request_id.is_none() {
                assert!(!asset.as_object().unwrap().contains_key("request_id"));
                assert!(!manifest.as_object().unwrap().contains_key("request_id"));
            }
            assert_eq!(asset["source"], observation.source("jsonl"));
            assert_eq!(asset["producer"], json!({"role":"offline_mapping_cli"}));
            for (entry, path) in manifest["files"].as_array().unwrap().iter().zip(["asset.json", "map.pgm"]) {
                assert_eq!(entry.as_object().unwrap().len(), 2);
                assert_eq!(entry["path"], path);
                assert_eq!(entry["size"].as_u64(), Some(fs::metadata(output.join(path)).unwrap().len()));
            }
            assert!(!asset.as_object().unwrap().contains_key("asset_id"));
            assert!(!manifest.as_object().unwrap().contains_key("manifest_sha256"));
            assert_eq!(fs::read_dir(&output_parent).unwrap().count(), 1);
        }
    }

    #[test]
    fn request_id_matches_owner_ascii_contract_and_cli_flag() {
        use clap::Parser;
        for id in ["A", "a:B_c.9-", &"a".repeat(128)] {
            require_request_id(id).unwrap();
            let parsed = crate::Cli::try_parse_from(["nuwax-mapping-cli", "map", "--bag", "input.jsonl",
                "--output-parent", ".", "--map-name", "m", "--frame-id", "map", "--request-id", id]).unwrap();
            let crate::Command::Map(parsed) = parsed.command else { panic!("map command expected") };
            assert_eq!(parsed.request_id.as_deref(), Some(id));
        }
        for id in ["", ":a", " a", "a ", "a/b", "a\\b", "a\n", "a\0", "地图", &"a".repeat(129)] {
            assert!(require_request_id(id).is_err(), "unexpected accepted ID: {id:?}");
        }
    }

    #[test]
    fn timestamp_observations_preserve_signed_i64_bounds() {
        assert_eq!(checked_timestamp_ns(-9_223_372_037, 145_224_192).unwrap(), i64::MIN);
        assert_eq!(checked_timestamp_ns(9_223_372_036, 854_775_807).unwrap(), i64::MAX);
        assert_eq!(checked_timestamp_ns(-1, 500_000_000).unwrap(), -500_000_000);
        assert_eq!(checked_timestamp_ns(0, 0).unwrap(), 0);
        let zero_identity = InputObservation { size: 1, device: 0, inode: 0, mtime_ns: -1, ctime_ns: 0 };
        assert_eq!(zero_identity.source("jsonl"), json!({"dataset_size":1, "input_format":"jsonl",
            "dataset_device":0, "dataset_inode":0, "dataset_mtime_ns":-1, "dataset_ctime_ns":0}));
        assert!(checked_timestamp_ns(-9_223_372_037, 145_224_191).is_err());
        assert!(checked_timestamp_ns(9_223_372_036, 854_775_808).is_err());
    }

    #[test]
    fn input_observation_accepts_owner_fd_alias_and_rejects_persistent_changes() {
        use std::os::fd::AsRawFd;
        use std::os::unix::fs::symlink;
        let directory = TestDir::new();
        let bag = directory.dataset();
        let held = File::open(&bag).unwrap();
        let observation = InputObservation::from_metadata(&held.metadata().unwrap()).unwrap();
        let alias = directory.0.join("owner-input.jsonl");
        symlink(format!("/proc/self/fd/{}", held.as_raw_fd()), &alias).unwrap();
        let alias_parent = File::open(&directory.0).unwrap();
        let alias = PathBuf::from(format!("/proc/self/fd/{}/owner-input.jsonl", alias_parent.as_raw_fd()));
        observation.verify_path(&alias).unwrap();
        assert_eq!(read_command_dataset(&alias).unwrap().dataset.records.len(), 1);
        observation.verify_metadata(&held.metadata().unwrap()).unwrap();
        observation.verify_path(&alias).unwrap();

        fs::rename(&bag, directory.0.join("original.jsonl")).unwrap();
        fs::write(&bag, DATASET).unwrap();
        assert!(observation.verify_path(&bag).is_err());
        let current = File::open(&bag).unwrap();
        let current_observation = InputObservation::from_metadata(&current.metadata().unwrap()).unwrap();
        std::fs::OpenOptions::new().append(true).open(&bag).unwrap().write_all(b"\n").unwrap();
        assert!(current_observation.verify_metadata(&current.metadata().unwrap()).is_err());
        let before_time_change = InputObservation::from_metadata(&current.metadata().unwrap()).unwrap();
        current.set_times(std::fs::FileTimes::new().set_modified(std::time::UNIX_EPOCH
            + std::time::Duration::from_secs(1))).unwrap();
        assert!(before_time_change.verify_metadata(&current.metadata().unwrap()).is_err());
    }

    #[tokio::test]
    async fn map_schema2_keeps_release_proof_fields_and_no_clobber() {
        let directory = TestDir::new();
        let bag = directory.dataset();
        let mut command = args(&bag, &directory.0, Some("release:job"));
        command.release_generation = Some("a".repeat(64));
        command.release_manifest_sha256 = Some("b".repeat(64));
        command.artifact_sha256 = Some("c".repeat(64));
        run(command).await.unwrap();
        let output = directory.0.join(safe_map_segment("真实-map"));
        let before = fs::read(output.join("asset.json")).unwrap();
        let asset: Value = serde_json::from_slice(&before).unwrap();
        assert_eq!(asset["producer"], json!({"role":"offline_mapping_cli", "release_generation":"a".repeat(64),
            "release_manifest_sha256":"b".repeat(64), "artifact_sha256":"c".repeat(64)}));
        assert!(producer_metadata(Some("invalid"), None, None).is_err());
        let error = run(args(&bag, &directory.0, Some("other:job"))).await.unwrap_err();
        assert!(matches!(error.downcast_ref::<crate::artifact::MapBundlePublicationError>(),
            Some(crate::artifact::MapBundlePublicationError::Conflict { .. })));
        assert_eq!(fs::read(output.join("asset.json")).unwrap(), before);
    }

    #[tokio::test]
    async fn map_schema2_reader_rejects_old_or_mismatched_protocol_fields() {
        let directory = TestDir::new();
        let bag = directory.dataset();
        run(args(&bag, &directory.0, Some("Job:one"))).await.unwrap();
        let output = directory.0.join(safe_map_segment("真实-map"));
        let observation = InputObservation::from_metadata(&File::open(&bag).unwrap().metadata().unwrap()).unwrap();
        let manifest_bytes = fs::read(output.join("manifest.json")).unwrap();
        let asset_bytes = fs::read(output.join("asset.json")).unwrap();
        let valid_manifest: Value = serde_json::from_slice(&manifest_bytes).unwrap();
        let valid_asset: Value = serde_json::from_slice(&asset_bytes).unwrap();
        let validate = || validate_bundle(&output, "真实-map", "map", &observation, "jsonl", Some("Job:one"), None, None, None);
        validate().unwrap();
        for variant in 0..5 {
            let mut value = valid_manifest.clone();
            match variant {
                0 => value["schema_version"] = json!("nuwax.mapping.offline-map-manifest.v1"),
                1 => value["request_id"] = json!("Job:two"),
                2 => value["files"][0]["sha256"] = json!("opaque-legacy-value"),
                3 => value["files"][0]["path"] = json!("../outside"),
                _ => value["files"].as_array_mut().unwrap().reverse(),
            }
            fs::write(output.join("manifest.json"), canonical_json(&value).unwrap()).unwrap();
            assert!(validate().is_err(), "accepted manifest variant {variant}");
        }
        for variant in 0..5 {
            let mut value = valid_asset.clone();
            match variant {
                0 => value["schema_version"] = json!("nuwax.mapping.offline-map-asset.v1"),
                1 => value["request_id"] = json!("Job:two"),
                2 => value["source"]["dataset_sha256"] = json!("opaque-legacy-value"),
                3 => value["source"]["dataset_inode"] = json!(observation.inode ^ 1),
                _ => value["source"]["dataset_size"] = json!(true),
            }
            let bytes = canonical_json(&value).unwrap();
            fs::write(output.join("asset.json"), &bytes).unwrap();
            let mut manifest = valid_manifest.clone();
            manifest["files"][0]["size"] = json!(bytes.len());
            fs::write(output.join("manifest.json"), canonical_json(&manifest).unwrap()).unwrap();
            assert!(validate().is_err(), "accepted asset variant {variant}");
        }
    }
    fn representative_costmap(width: u32, height: u32) -> nuwax_mapping::Costmap {
        let grid = OccupancyGrid {
            data: (0..u64::from(width) * u64::from(height))
                .map(|index| [-1, 0, 25, 75, 100][index as usize % 5]).collect(),
            width, height, resolution: 0.1, origin: [1.25, -2.0, 0.0],
            origin_orientation: [0.0, 0.0, 0.0, 1.0], frame_id: "map".to_string(), ts: 1.0,
        };
        CostMapper::new(CostMapperConfig::dimos_default()).convert(&grid)
    }

    // Frozen phase1 output algorithm; generic Write replaces only its File receiver for counting.
    fn original_pgm_to(writer: &mut impl Write, costmap: &nuwax_mapping::Costmap) -> std::io::Result<()> {
        writeln!(writer, "P5")?;
        writeln!(writer, "{} {}", costmap.width(), costmap.height())?;
        writeln!(writer, "255")?;
        for row in 0..costmap.height() {
            for column in 0..costmap.width() {
                let cost = costmap.get(row, column).unwrap_or(0);
                writer.write_all(&[255_u8.saturating_sub(cost)])?;
            }
        }
        Ok(())
    }

    struct CountWrites<W> { inner: W, writes: usize, bytes: usize }
    impl<W: Write> Write for CountWrites<W> {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            self.writes += 1;
            let count = self.inner.write(bytes)?;
            self.bytes += count;
            Ok(count)
        }
        fn flush(&mut self) -> std::io::Result<()> { self.inner.flush() }
    }

    #[test]
    fn pgm_buffer_preserves_original_bytes_with_bounded_write_batches() {
        for (width, height) in [(1, 1), (17, 13), (320, 240), (1024, 1024)] {
            let costmap = representative_costmap(width, height);
            let mut original = CountWrites { inner: Vec::new(), writes: 0, bytes: 0 };
            original_pgm_to(&mut original, &costmap).unwrap();
            original.flush().unwrap();
            let mut buffered = BufWriter::with_capacity(8 * 1024,
                CountWrites { inner: Vec::new(), writes: 0, bytes: 0 });
            write_pgm_to(&mut buffered, &costmap).unwrap();
            buffered.flush().unwrap();
            let actual = buffered.get_ref();
            assert_eq!(actual.inner, original.inner);
            assert_eq!(actual.bytes, original.bytes);
            assert_eq!(buffered.capacity(), 8 * 1024);
            assert_eq!(actual.writes, (actual.bytes + 8191) / 8192);
            assert!(actual.writes < original.writes);
            println!("{}", json!({"width":width,"height":height,"bytes":actual.bytes,
                "original_write_calls":original.writes,"buffered_write_calls":actual.writes,"capacity":buffered.capacity()}));
        }
    }

    #[test]
    fn pgm_buffer_propagates_write_failures_without_changing_written_prefix() {
        struct FailAfter { bytes: Vec<u8>, limit: usize }
        impl Write for FailAfter {
            fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
                let remaining = self.limit - self.bytes.len();
                if remaining == 0 { return Err(std::io::Error::new(std::io::ErrorKind::Other, "injected write failure")); }
                let count = remaining.min(bytes.len());
                self.bytes.extend_from_slice(&bytes[..count]);
                Ok(count)
            }
            fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
        }
        let costmap = representative_costmap(128, 128);
        for limit in [0, 5, 8191, 8192, 16000] {
            let mut original = FailAfter { bytes: Vec::new(), limit };
            assert!(original_pgm_to(&mut original, &costmap).is_err());
            let mut buffered = BufWriter::with_capacity(8192, FailAfter { bytes: Vec::new(), limit });
            let result = write_pgm_to(&mut buffered, &costmap).and_then(|()| buffered.flush());
            assert!(result.is_err());
            assert_eq!(buffered.get_ref().bytes, original.bytes);
        }
    }

    #[test]
    #[ignore = "bounded paired filesystem measurement, run explicitly"]
    fn pgm_buffer_paired_ab_ba_real_file_measurement() {
        use std::time::Instant;
        let directory = TestDir::new();
        for (width, height) in [(320, 240), (1024, 1024)] {
            let costmap = representative_costmap(width, height);
            for (pair, order) in [[false, true], [true, false]].into_iter().enumerate() {
                let mut paths = Vec::new();
                for buffered in order {
                    let path = directory.0.join(format!("{width}-{height}-{pair}-{buffered}.pgm"));
                    let start = Instant::now();
                    if buffered {
                        write_pgm(&path, &costmap).unwrap();
                    } else {
                        let mut file = File::create(&path).unwrap();
                        original_pgm_to(&mut file, &costmap).unwrap();
                        file.flush().unwrap();
                        file.sync_all().unwrap();
                        drop(file);
                    }
                    let elapsed = start.elapsed();
                    println!("{}", json!({"width":width,"height":height,"pair":pair,
                        "buffered":buffered,"elapsed_ns":elapsed.as_nanos(),"bytes":fs::metadata(&path).unwrap().len()}));
                    paths.push(path);
                }
                assert_eq!(fs::read(&paths[0]).unwrap(), fs::read(&paths[1]).unwrap());
                validate_pgm(&paths[0], &costmap).unwrap();
                validate_pgm(&paths[1], &costmap).unwrap();
            }
        }
    }

}
