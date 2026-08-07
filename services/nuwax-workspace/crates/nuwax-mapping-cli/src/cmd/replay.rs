//! Build a deterministic replay schedule from persisted records.

use std::io::Write;
use std::path::PathBuf;

use clap::Args;
use serde::{Deserialize, Serialize};

use crate::artifact::{publish_new_file, ArtifactFormat};
use crate::dataset::DatasetRecord;
use crate::native::read_command_dataset;

/// Arguments for `replay`.
#[derive(Debug, Args)]
pub struct ReplayArgs {
    /// Input JSONL or DimOS memory2 `.db` dataset.
    #[arg(long)]
    pub bag: PathBuf,
    /// Output replay manifest. Defaults to `<input>.replay.json`.
    #[arg(long)]
    pub output: Option<PathBuf>,
    /// Skip this many seconds from the first dataset timestamp.
    #[arg(long, default_value_t = 0.0)]
    pub start_offset: f64,
    /// Playback speed multiplier.
    #[arg(long, default_value_t = 1.0)]
    pub rate: f64,
    /// Parse and schedule records without writing the manifest.
    #[arg(long)]
    pub dry_run: bool,
}

/// Versioned replay manifest.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReplayManifest {
    /// Format identifier.
    pub format: String,
    /// Format version.
    pub version: u32,
    /// Source dataset path.
    pub source: String,
    /// Playback rate.
    pub rate: f64,
    /// Source timestamp used as replay time zero.
    pub source_start_timestamp: f64,
    /// Scheduled records.
    pub events: Vec<ReplayEvent>,
}

/// One replay event.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReplayEvent {
    /// Seconds on the rate-adjusted replay timeline.
    pub replay_time: f64,
    /// Original persisted record.
    pub record: DatasetRecord,
}

/// Execute the `replay` command.
///
/// # Errors
/// Returns errors for invalid rates/offsets, unsupported or empty input,
/// filters that select no records, output collisions, serialization, or I/O.
pub async fn run(args: ReplayArgs) -> anyhow::Result<()> {
    if !args.start_offset.is_finite() || args.start_offset < 0.0 {
        anyhow::bail!("start_offset must be finite and non-negative");
    }
    if !args.rate.is_finite() || args.rate <= 0.0 {
        anyhow::bail!("rate must be finite and positive");
    }
    let decoded = read_command_dataset(&args.bag)?;
    if !decoded.skipped_streams.is_empty() {
        eprintln!(
            "[replay] skipped unsupported memory2 streams: {}",
            decoded.skipped_streams.join(",")
        );
    }
    let dataset = decoded.dataset;
    let first_timestamp = dataset
        .records
        .iter()
        .map(|record| record.timestamp)
        .min_by(f64::total_cmp)
        .ok_or_else(|| anyhow::anyhow!("dataset contains no records"))?;
    let source_start_timestamp = first_timestamp + args.start_offset;
    let mut selected: Vec<DatasetRecord> = dataset
        .records
        .into_iter()
        .filter(|record| record.timestamp >= source_start_timestamp)
        .collect();
    selected.sort_by(|left, right| left.timestamp.total_cmp(&right.timestamp));
    if selected.is_empty() {
        anyhow::bail!(
            "start_offset {} selects no records from {}",
            args.start_offset,
            args.bag.display()
        );
    }
    let manifest = ReplayManifest {
        format: "nuwax-mapping-replay".to_string(),
        version: 1,
        source: args.bag.display().to_string(),
        rate: args.rate,
        source_start_timestamp,
        events: selected
            .into_iter()
            .map(|record| ReplayEvent {
                replay_time: (record.timestamp - source_start_timestamp) / args.rate,
                record,
            })
            .collect(),
    };

    let output = args.output.clone().unwrap_or_else(|| {
        let file_name = args
            .bag
            .file_name()
            .and_then(|value| value.to_str())
            .unwrap_or("dataset");
        args.bag.with_file_name(format!("{file_name}.replay.json"))
    });
    if !args.dry_run {
        publish_new_file(
            &output,
            ArtifactFormat::ReplayJson,
            |writer| {
                serde_json::to_writer_pretty(&mut *writer, &manifest)?;
                writer.write_all(b"\n")?;
                Ok(())
            },
            |staging| {
                let persisted: ReplayManifest =
                    serde_json::from_reader(std::fs::File::open(staging)?)?;
                if persisted != manifest {
                    anyhow::bail!("persisted replay manifest does not match generated manifest");
                }
                Ok(())
            },
        )?;
    }
    println!(
        "[replay] dataset={} start_offset={} rate={} events={} output={} dry_run={}",
        args.bag.display(),
        args.start_offset,
        args.rate,
        manifest.events.len(),
        output.display(),
        args.dry_run
    );
    Ok(())
}
