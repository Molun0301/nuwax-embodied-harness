//! Export persisted marker detections as a deterministic replay artifact.

use std::collections::BTreeSet;
use std::io::Write;
use std::path::PathBuf;

use clap::Args;
use serde::{Deserialize, Serialize};

use crate::artifact::{publish_new_file, ArtifactFormat};
use crate::dataset::{read_dataset, PoseData, RecordPayload};
use crate::native::{input_format, read_command_dataset, InputFormat};

/// Arguments for `replay-marker`.
#[derive(Debug, Args)]
pub struct ReplayMarkerArgs {
    /// Input JSONL dataset containing pre-decoded marker records.
    #[arg(long)]
    pub bag: PathBuf,
    /// Output JSON marker replay artifact.
    #[arg(long)]
    pub output: PathBuf,
    /// Marker family to select.
    #[arg(long, default_value = "tag36h11")]
    pub family: String,
}

/// Marker replay artifact.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarkerReplay {
    /// Format identifier.
    pub format: String,
    /// Format version.
    pub version: u32,
    /// Source dataset path.
    pub source: String,
    /// Selected marker family.
    pub family: String,
    /// Sorted distinct marker IDs.
    pub marker_ids: Vec<u32>,
    /// Timestamped marker observations.
    pub markers: Vec<MarkerEvent>,
}

/// One marker observation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarkerEvent {
    /// Source timestamp.
    pub timestamp: f64,
    /// Source stream name.
    pub stream: String,
    /// Marker ID.
    pub id: u32,
    /// Marker pose.
    pub pose: PoseData,
}

/// Execute the `replay-marker` command.
///
/// # Errors
/// Returns errors for empty family names, unsupported input, no matching
/// marker data, output collisions, serialization, or I/O.
pub async fn run(args: ReplayMarkerArgs) -> anyhow::Result<()> {
    if args.family.trim().is_empty() {
        anyhow::bail!("marker family must not be empty");
    }
    let dataset = match input_format(&args.bag)? {
        InputFormat::JsonLines => read_dataset(&args.bag)?,
        InputFormat::Memory2Sqlite => anyhow::bail!(
            "memory2 replay-marker requires the original image/JPEG, camera calibration, and AprilTag detector pipeline; this dependency-free CLI does not claim detector parity from opaque image blobs"
        ),
        InputFormat::Rosbag2Sqlite | InputFormat::Mcap | InputFormat::Ros1Bag => {
            return read_command_dataset(&args.bag).map(|_| ());
        }
    };
    let mut markers: Vec<MarkerEvent> = dataset
        .records
        .into_iter()
        .filter_map(|record| match record.payload {
            RecordPayload::Marker { family, id, pose } if family == args.family => {
                Some(MarkerEvent {
                    timestamp: record.timestamp,
                    stream: record.stream,
                    id,
                    pose,
                })
            }
            _ => None,
        })
        .collect();
    markers.sort_by(|left, right| {
        left.timestamp
            .total_cmp(&right.timestamp)
            .then_with(|| left.id.cmp(&right.id))
    });
    if markers.is_empty() {
        anyhow::bail!(
            "dataset {} contains no marker records for family {:?}",
            args.bag.display(),
            args.family
        );
    }
    let marker_ids = markers
        .iter()
        .map(|marker| marker.id)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let replay = MarkerReplay {
        format: "nuwax-marker-replay".to_string(),
        version: 1,
        source: args.bag.display().to_string(),
        family: args.family.clone(),
        marker_ids,
        markers,
    };

    publish_new_file(
        &args.output,
        ArtifactFormat::MarkerReplayJson,
        |writer| {
            serde_json::to_writer_pretty(&mut *writer, &replay)?;
            writer.write_all(b"\n")?;
            Ok(())
        },
        |staging| {
            let persisted: MarkerReplay = serde_json::from_reader(std::fs::File::open(staging)?)?;
            if persisted != replay {
                anyhow::bail!("persisted marker replay does not match generated replay");
            }
            Ok(())
        },
    )?;
    println!(
        "[replay-marker] dataset={} family={} markers={} unique_ids={} output={}",
        args.bag.display(),
        args.family,
        replay.markers.len(),
        replay.marker_ids.len(),
        args.output.display()
    );
    Ok(())
}
