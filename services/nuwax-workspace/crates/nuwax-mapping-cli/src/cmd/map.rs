//! Build a PGM costmap from persisted occupancy-grid or point-cloud records.

use clap::Args;
use nalgebra::{point, Point3};
use nuwax_mapping::cost_mapper::{
    CostAlgoSelector, CostMapper, CostMapperConfig, OccupancyConfig, SimplePointCloud2,
};
use nuwax_mapping::occupancy::types::OccupancyGrid;
use std::collections::BTreeSet;
use std::path::PathBuf;

use crate::dataset::RecordPayload;
use crate::native::read_command_dataset;

/// Arguments for the `map` command.
#[derive(Debug, Args)]
pub struct MapArgs {
    /// Input JSONL or DimOS memory2 `.db` dataset.
    #[arg(long)]
    pub bag: PathBuf,
    /// Output PGM path. Defaults to the input path with `.pgm` extension.
    #[arg(long)]
    pub output: Option<PathBuf>,
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
/// Returns input schema, map construction, or output I/O errors.
pub async fn run(args: MapArgs) -> anyhow::Result<()> {
    if !args.resolution.is_finite() || args.resolution <= 0.0 {
        anyhow::bail!("resolution must be finite and positive");
    }
    let decoded = read_command_dataset(&args.bag)?;
    report_skipped_streams(&decoded.skipped_streams);
    let dataset = decoded.dataset;
    let costmap = if let Some(record) = dataset
        .records
        .iter()
        .filter(|record| matches!(record.payload, RecordPayload::OccupancyGrid { .. }))
        .max_by(|left, right| left.timestamp.total_cmp(&right.timestamp))
    {
        costmap_from_occupancy_record(record)?
    } else {
        costmap_from_point_records(&dataset.records, args.resolution)?
    };

    let output = args.output.clone().unwrap_or_else(|| {
        let mut path = args.bag.clone();
        path.set_extension("pgm");
        path
    });
    write_pgm(&output, &costmap)?;
    println!(
        "[map] dataset={} theme={} records={} costmap={}x{} output={}",
        args.bag.display(),
        args.theme,
        dataset.records.len(),
        costmap.width(),
        costmap.height(),
        output.display()
    );
    Ok(())
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
) -> anyhow::Result<nuwax_mapping::Costmap> {
    let frames: BTreeSet<&str> = records
        .iter()
        .filter_map(|record| match &record.payload {
            RecordPayload::PointCloud { frame_id, .. } => Some(frame_id.as_str()),
            _ => None,
        })
        .collect();
    if frames.len() > 1 {
        anyhow::bail!(
            "point-cloud records span multiple frames {frames:?}; transform them to one frame before mapping"
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
    let frame_id = frames.iter().next().copied().unwrap_or("world");
    let cloud =
        SimplePointCloud2::with_metadata(points, point![0.0, 0.0, 0.0], frame_id, timestamp);
    let config = OccupancyConfig {
        algo: CostAlgoSelector::Simple,
        ..OccupancyConfig::default()
    };
    let (_, _, costmap) = CostMapper::from_pointcloud_auto(&cloud, &config, resolution)?;
    Ok(costmap)
}

fn write_pgm(path: &PathBuf, costmap: &nuwax_mapping::Costmap) -> anyhow::Result<()> {
    use std::io::Write;

    let mut file = std::fs::File::create(path)?;
    writeln!(file, "P5")?;
    writeln!(file, "{} {}", costmap.width(), costmap.height())?;
    writeln!(file, "255")?;
    for row in 0..costmap.height() {
        for column in 0..costmap.width() {
            let cost = costmap.get(row, column).unwrap_or(0);
            file.write_all(&[255_u8.saturating_sub(cost)])?;
        }
    }
    Ok(())
}
