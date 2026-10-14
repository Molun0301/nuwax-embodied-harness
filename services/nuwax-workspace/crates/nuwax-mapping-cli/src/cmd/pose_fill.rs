//! Attach nearest-in-time pose observations to point-cloud records.

use clap::Args;
use std::path::PathBuf;

use crate::dataset::{read_dataset, write_dataset, Dataset, PoseData, RecordPayload};
use crate::native::{
    input_format, pose_fill_memory2, read_command_dataset, InputFormat, Memory2PoseFillOptions,
};

/// Arguments for `pose-fill`.
#[derive(Debug, Args)]
pub struct PoseFillArgs {
    /// Input JSONL or DimOS memory2 `.db` dataset.
    #[arg(long)]
    pub input: PathBuf,
    /// New output dataset. Existing files are never overwritten.
    #[arg(long)]
    pub output: PathBuf,
    /// Point-cloud stream to receive pose metadata.
    #[arg(long, default_value = "lidar")]
    pub target: String,
    /// Pose record stream used as the nearest-time source.
    #[arg(long, default_value = "odom")]
    pub pose_source: String,
    /// Maximum timestamp difference in seconds.
    #[arg(long, default_value_t = 0.1)]
    pub tolerance: f64,
    /// Comma-separated memory2 streams to copy. Defaults to all streams.
    #[arg(long)]
    pub streams: Option<String>,
    /// Skip the first N seconds of every selected memory2 stream.
    #[arg(long, default_value_t = 0.0)]
    pub seek: f64,
    /// Keep this many seconds after each selected memory2 stream starts.
    #[arg(long)]
    pub duration: Option<f64>,
}

/// Execute the `pose-fill` command.
///
/// Target records without a pose inside the tolerance are dropped, matching
/// the source DimOS stream-alignment behavior. All non-target records are
/// copied verbatim.
///
/// # Errors
/// Returns input schema, missing-stream, unsupported-target, output collision,
/// validation, or I/O errors.
pub async fn run(args: PoseFillArgs) -> anyhow::Result<()> {
    if !args.tolerance.is_finite() || args.tolerance < 0.0 {
        anyhow::bail!("tolerance must be finite and non-negative");
    }
    let streams = parse_streams(args.streams.as_deref())?;
    match input_format(&args.input)? {
        InputFormat::Memory2Sqlite => {
            let (filled_records, target_records) = pose_fill_memory2(
                &args.input,
                &args.output,
                Memory2PoseFillOptions {
                    target: &args.target,
                    pose_source: &args.pose_source,
                    tolerance: args.tolerance,
                    streams: streams.as_deref(),
                    seek: args.seek,
                    duration: args.duration,
                },
            )?;
            println!(
                "[pose-fill] input={} target={} pose_source={} matched={}/{} output={}",
                args.input.display(),
                args.target,
                args.pose_source,
                filled_records,
                target_records,
                args.output.display()
            );
            return Ok(());
        }
        InputFormat::JsonLines => {
            if streams.is_some() || args.seek != 0.0 || args.duration.is_some() {
                anyhow::bail!("--streams/--seek/--duration are native memory2 options; JSONL pose-fill copies its complete typed dataset");
            }
        }
        InputFormat::Rosbag2Sqlite | InputFormat::Mcap | InputFormat::Ros1Bag => {
            return read_command_dataset(&args.input).map(|_| ());
        }
    }
    let source = read_dataset(&args.input)?;
    let mut poses: Vec<(f64, PoseData)> = source
        .records
        .iter()
        .filter_map(|record| {
            if record.stream != args.pose_source {
                return None;
            }
            match record.payload {
                RecordPayload::Pose { pose } => Some((record.timestamp, pose)),
                _ => None,
            }
        })
        .collect();
    if poses.is_empty() {
        anyhow::bail!(
            "pose source stream {:?} has no pose records",
            args.pose_source
        );
    }
    poses.sort_by(|left, right| left.0.total_cmp(&right.0));

    let mut target_records = 0usize;
    let mut filled_records = 0usize;
    let mut records = Vec::with_capacity(source.records.len());
    for mut record in source.records {
        if record.stream != args.target {
            records.push(record);
            continue;
        }
        target_records += 1;
        let RecordPayload::PointCloud { pose, .. } = &mut record.payload else {
            anyhow::bail!(
                "target stream {:?} contains unsupported {} records; pose-fill currently supports point_cloud targets",
                args.target,
                record.payload.kind()
            );
        };
        if let Some(value) = nearest_pose(&poses, record.timestamp, args.tolerance) {
            *pose = Some(value);
            records.push(record);
            filled_records += 1;
        }
    }
    if target_records == 0 {
        anyhow::bail!("target stream {:?} was not found", args.target);
    }
    if filled_records == 0 {
        anyhow::bail!(
            "no {:?} records matched {:?} within {} seconds",
            args.target,
            args.pose_source,
            args.tolerance
        );
    }

    let output = Dataset {
        header: source.header,
        records,
    };
    write_dataset(&args.output, &output)?;
    println!(
        "[pose-fill] input={} target={} pose_source={} matched={}/{} output={}",
        args.input.display(),
        args.target,
        args.pose_source,
        filled_records,
        target_records,
        args.output.display()
    );
    Ok(())
}

fn parse_streams(value: Option<&str>) -> anyhow::Result<Option<Vec<String>>> {
    let Some(value) = value else {
        return Ok(None);
    };
    let streams: Vec<String> = value
        .split(',')
        .map(str::trim)
        .map(str::to_string)
        .collect();
    if streams.is_empty() || streams.iter().any(String::is_empty) {
        anyhow::bail!("--streams must contain non-empty comma-separated stream names");
    }
    Ok(Some(streams))
}

fn nearest_pose(poses: &[(f64, PoseData)], timestamp: f64, tolerance: f64) -> Option<PoseData> {
    let insertion = poses.partition_point(|(candidate, _)| *candidate < timestamp);
    let before = insertion.checked_sub(1).and_then(|index| poses.get(index));
    let after = poses.get(insertion);
    [before, after]
        .into_iter()
        .flatten()
        .min_by(|left, right| {
            (left.0 - timestamp)
                .abs()
                .total_cmp(&(right.0 - timestamp).abs())
        })
        .filter(|(candidate, _)| (*candidate - timestamp).abs() <= tolerance)
        .map(|(_, pose)| *pose)
}
