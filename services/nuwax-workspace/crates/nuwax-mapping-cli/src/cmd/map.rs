//! Build and durably publish an immutable offline map asset bundle.

use clap::Args;
use nalgebra::{point, Point3};
use nuwax_mapping::cost_mapper::{
    CostAlgoSelector, CostMapper, CostMapperConfig, OccupancyConfig, SimplePointCloud2,
};
use nuwax_mapping::occupancy::types::OccupancyGrid;
use serde_json::{json, Map, Value};
use std::collections::{BTreeSet, HashSet};
use std::fs::{self, File};
use std::io::{BufRead, BufReader, Read, Write};
use std::path::{Path, PathBuf};

use crate::artifact::publish_map_bundle;
use crate::dataset::RecordPayload;
use crate::native::read_command_dataset;

const ASSET_SCHEMA: &str = "nuwax.mapping.offline-map-asset.v1";
const MANIFEST_SCHEMA: &str = "nuwax.mapping.offline-map-manifest.v1";
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

    let dataset_bytes = fs::read(&args.bag)?;
    let dataset_size = u64::try_from(dataset_bytes.len())?;
    let dataset_sha256 = sha256_hex(&dataset_bytes);
    let input_format = input_format(&args.bag)?;
    let decoded = read_command_dataset(&args.bag)?;
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
        &dataset_sha256,
        dataset_size,
        input_format,
        &costmap,
        args.release_generation.as_deref(),
        args.release_manifest_sha256.as_deref(),
        args.artifact_sha256.as_deref(),
    )?;
    publish_map_bundle(
        &output,
        |staging| write_bundle(staging, &costmap, &asset, map_name, frame_id),
        |staging| {
            validate_bundle(
                staging,
                map_name,
                frame_id,
                args.release_generation.as_deref(),
                args.release_manifest_sha256.as_deref(),
                args.artifact_sha256.as_deref(),
            )
        },
    )?;
    let manifest_sha256 = sha256_file(&output.join("manifest.json"))?;
    println!(
        "[map] dataset={} theme={} records={} costmap={}x{} output={} manifest_sha256={}",
        args.bag.display(),
        args.theme,
        dataset.records.len(),
        costmap.width(),
        costmap.height(),
        output.display(),
        manifest_sha256
    );
    Ok(())
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
    records: &[crate::dataset::DatasetRecord],
    resolution: f64,
    required_frame: &str,
) -> anyhow::Result<nuwax_mapping::Costmap> {
    let frames: BTreeSet<&str> = records
        .iter()
        .filter_map(|record| match &record.payload {
            RecordPayload::PointCloud { frame_id, .. } => Some(frame_id.as_str()),
            _ => None,
        })
        .collect();
    if frames.len() != 1 || frames.iter().next().copied() != Some(required_frame) {
        anyhow::bail!(
            "point-cloud frame set {frames:?} does not exactly match --frame-id {required_frame:?}"
        );
    }
    let points: Vec<Point3<f64>> = records
        .iter()
        .filter_map(|record| match &record.payload {
            RecordPayload::PointCloud { points, .. } => Some(points.as_slice()),
            _ => None,
        })
        .flatten()
        .map(|value| point![value[0], value[1], value[2]])
        .collect();
    if points.is_empty() {
        anyhow::bail!("dataset contains neither occupancy-grid nor point-cloud records");
    }
    let timestamp = records
        .iter()
        .filter(|record| matches!(record.payload, RecordPayload::PointCloud { .. }))
        .map(|record| record.timestamp)
        .max_by(f64::total_cmp)
        .unwrap_or(0.0);
    let cloud =
        SimplePointCloud2::with_metadata(points, point![0.0, 0.0, 0.0], required_frame, timestamp);
    let config = OccupancyConfig {
        algo: CostAlgoSelector::Simple,
        ..OccupancyConfig::default()
    };
    let (_, _, costmap) = CostMapper::from_pointcloud_auto(&cloud, &config, resolution)?;
    Ok(costmap)
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
    dataset_sha256: &str,
    dataset_size: u64,
    input_format: &str,
    costmap: &nuwax_mapping::Costmap,
    release_generation: Option<&str>,
    release_manifest_sha256: Option<&str>,
    artifact_sha256: Option<&str>,
) -> anyhow::Result<Vec<u8>> {
    let translation = costmap.metadata.origin.translation.vector;
    let (_, _, yaw) = costmap.metadata.origin.rotation.euler_angles();
    canonical_json(&json!({
        "schema_version": ASSET_SCHEMA,
        "map_name": map_name,
        "frame_id": frame_id,
        "source": {
            "dataset_sha256": dataset_sha256,
            "dataset_size": dataset_size,
            "input_format": input_format
        },
        "grid": {
            "width": costmap.width(),
            "height": costmap.height(),
            "resolution_m": costmap.metadata.resolution,
            "origin_xy_yaw": [translation.x, translation.y, yaw],
            "encoding": PGM_ENCODING,
            "path": "map.pgm"
        },
        "producer": producer_metadata(release_generation, release_manifest_sha256, artifact_sha256)?
    }))
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
    let manifest = canonical_json(&json!({
        "schema_version": MANIFEST_SCHEMA,
        "map_name": map_name,
        "frame_id": frame_id,
        "asset_path": "asset.json",
        "files": files
    }))?;
    write_synced_file(&staging.join("manifest.json"), &manifest)?;
    Ok(())
}

fn file_entry(path: &Path, relative: &str) -> anyhow::Result<Value> {
    let metadata = fs::metadata(path)?;
    Ok(json!({
        "path": relative,
        "size": metadata.len(),
        "sha256": sha256_file(path)?
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
    let mut file = File::create(path)?;
    writeln!(file, "P5")?;
    writeln!(file, "{} {}", costmap.width(), costmap.height())?;
    writeln!(file, "255")?;
    for row in 0..costmap.height() {
        for column in 0..costmap.width() {
            let cost = costmap.get(row, column).unwrap_or(0);
            file.write_all(&[255_u8.saturating_sub(cost)])?;
        }
    }
    file.flush()?;
    file.sync_all()?;
    drop(file);
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
    let files = manifest
        .get("files")
        .and_then(Value::as_array)
        .ok_or_else(|| anyhow::anyhow!("manifest files is not an array"))?;
    let mut paths = Vec::new();
    for entry in files {
        let path = entry
            .get("path")
            .and_then(Value::as_str)
            .ok_or_else(|| anyhow::anyhow!("manifest file path is invalid"))?;
        paths.push(path);
        let member = staging.join(path);
        let size = entry.get("size").and_then(Value::as_u64);
        let digest = entry.get("sha256").and_then(Value::as_str);
        if size != Some(fs::metadata(&member)?.len())
            || digest != Some(sha256_file(&member)?.as_str())
        {
            anyhow::bail!("manifest proof failed for {path}");
        }
    }
    if paths != ["asset.json", "map.pgm"] {
        anyhow::bail!("manifest files must be exact and byte-sorted");
    }
    let asset_bytes = fs::read(staging.join("asset.json"))?;
    let asset: Value = serde_json::from_slice(&asset_bytes)?;
    if canonical_json(&asset)? != asset_bytes {
        anyhow::bail!("map asset is not canonical JSON");
    }
    let asset = exact_object(
        &asset,
        &[
            "schema_version",
            "map_name",
            "frame_id",
            "source",
            "grid",
            "producer",
        ],
        "map asset",
    )?;
    if asset.get("schema_version") != Some(&Value::String(ASSET_SCHEMA.to_string()))
        || asset.get("map_name") != Some(&Value::String(map_name.to_string()))
        || asset.get("frame_id") != Some(&Value::String(frame_id.to_string()))
    {
        anyhow::bail!("map asset identity is inconsistent");
    }

    let source = exact_object(
        &asset["source"],
        &["dataset_sha256", "dataset_size", "input_format"],
        "map asset source",
    )?;
    let dataset_sha256 = source.get("dataset_sha256").and_then(Value::as_str);
    if !dataset_sha256.is_some_and(|value| require_digest(value, "dataset-sha256").is_ok())
        || source.get("dataset_size").and_then(Value::as_u64).is_none()
        || !matches!(
            source.get("input_format").and_then(Value::as_str),
            Some("jsonl" | "ndjson" | "memory2_sqlite")
        )
    {
        anyhow::bail!("map asset source proof is invalid");
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

fn sha256_file(path: &Path) -> anyhow::Result<String> {
    Ok(sha256_hex(&fs::read(path)?))
}

fn sha256_hex(input: &[u8]) -> String {
    const INITIAL: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let bit_len = (input.len() as u64).wrapping_mul(8);
    let mut bytes = input.to_vec();
    bytes.push(0x80);
    while bytes.len() % 64 != 56 {
        bytes.push(0);
    }
    bytes.extend_from_slice(&bit_len.to_be_bytes());
    let mut state = INITIAL;
    for block in bytes.chunks_exact(64) {
        let mut w = [0_u32; 64];
        for (index, word) in block.chunks_exact(4).enumerate() {
            w[index] = u32::from_be_bytes([word[0], word[1], word[2], word[3]]);
        }
        for index in 16..64 {
            let s0 = w[index - 15].rotate_right(7)
                ^ w[index - 15].rotate_right(18)
                ^ (w[index - 15] >> 3);
            let s1 = w[index - 2].rotate_right(17)
                ^ w[index - 2].rotate_right(19)
                ^ (w[index - 2] >> 10);
            w[index] = w[index - 16]
                .wrapping_add(s0)
                .wrapping_add(w[index - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = state;
        for index in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choice = (e & f) ^ ((!e) & g);
            let temp1 = h
                .wrapping_add(s1)
                .wrapping_add(choice)
                .wrapping_add(K[index])
                .wrapping_add(w[index]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let temp2 = s0.wrapping_add(majority);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(temp1);
            d = c;
            c = b;
            b = a;
            a = temp1.wrapping_add(temp2);
        }
        state[0] = state[0].wrapping_add(a);
        state[1] = state[1].wrapping_add(b);
        state[2] = state[2].wrapping_add(c);
        state[3] = state[3].wrapping_add(d);
        state[4] = state[4].wrapping_add(e);
        state[5] = state[5].wrapping_add(f);
        state[6] = state[6].wrapping_add(g);
        state[7] = state[7].wrapping_add(h);
    }
    state.iter().map(|word| format!("{word:08x}")).collect()
}
