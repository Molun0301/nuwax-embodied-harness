//! Rename or drop logical streams in a persisted dataset.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use clap::Args;

use crate::dataset::{read_dataset, write_dataset, Dataset};
use crate::native::{input_format, read_command_dataset, rename_memory2, InputFormat};

/// Arguments for `rename`.
#[derive(Debug, Args)]
pub struct RenameArgs {
    /// Input JSONL or DimOS memory2 `.db` dataset.
    #[arg(long)]
    pub bag: PathBuf,
    /// New output dataset. Existing files are never overwritten.
    #[arg(long)]
    pub output: PathBuf,
    /// Stream rename in `OLD=NEW` form. Repeat for multiple streams.
    #[arg(long = "rename", value_name = "OLD=NEW")]
    pub renames: Vec<String>,
    /// Stream to omit from the output. Repeat for multiple streams.
    #[arg(long = "drop", value_name = "STREAM")]
    pub drops: Vec<String>,
    /// Skip the first N seconds of every native memory2 stream.
    #[arg(long, default_value_t = 0.0)]
    pub seek: f64,
    /// Keep this many seconds after each native stream's selected start.
    #[arg(long)]
    pub duration: Option<f64>,
    /// Parse and validate the operation without writing the output.
    #[arg(long)]
    pub dry_run: bool,
}

/// Execute the `rename` command.
///
/// # Errors
/// Returns errors for malformed rename pairs, missing streams, destination
/// collisions, unsupported input, output collisions, validation, or I/O.
pub async fn run(args: RenameArgs) -> anyhow::Result<()> {
    if args.renames.is_empty() && args.drops.is_empty() {
        anyhow::bail!("at least one --rename OLD=NEW or --drop STREAM is required");
    }
    let renames = parse_renames(&args.renames)?;
    let drops: BTreeSet<String> = args.drops.iter().cloned().collect();
    match input_format(&args.bag)? {
        InputFormat::Memory2Sqlite => {
            rename_memory2(
                &args.bag,
                &args.output,
                &renames,
                &drops,
                args.seek,
                args.duration,
                args.dry_run,
            )?;
            println!(
                "[rename] input={} output={} renames={} drops={} seek={} duration={:?} dry_run={}",
                args.bag.display(),
                args.output.display(),
                renames.len(),
                drops.len(),
                args.seek,
                args.duration,
                args.dry_run
            );
            return Ok(());
        }
        InputFormat::JsonLines => {
            if args.seek != 0.0 || args.duration.is_some() {
                anyhow::bail!("--seek/--duration are native memory2 options; JSONL rename has no implicit per-stream timeline window");
            }
        }
        InputFormat::Rosbag2Sqlite | InputFormat::Mcap | InputFormat::Ros1Bag => {
            return read_command_dataset(&args.bag).map(|_| ());
        }
    }
    let mut dataset = read_dataset(&args.bag)?;
    let streams: BTreeSet<String> = dataset
        .records
        .iter()
        .map(|record| record.stream.clone())
        .collect();

    for source in renames.keys().chain(drops.iter()) {
        if !streams.contains(source) {
            anyhow::bail!(
                "stream {:?} does not exist in {}",
                source,
                args.bag.display()
            );
        }
    }
    let mut destinations = BTreeMap::<String, String>::new();
    for source in &streams {
        if drops.contains(source) {
            continue;
        }
        let destination = renames.get(source).unwrap_or(source);
        if let Some(previous) = destinations.insert(destination.clone(), source.clone()) {
            anyhow::bail!(
                "renaming would merge streams {previous:?} and {source:?} into {destination:?}"
            );
        }
    }

    let before = dataset.records.len();
    dataset.records.retain_mut(|record| {
        if drops.contains(&record.stream) {
            return false;
        }
        if let Some(destination) = renames.get(&record.stream) {
            record.stream.clone_from(destination);
        }
        true
    });
    let dropped = before - dataset.records.len();
    if !args.dry_run {
        write_dataset(
            &args.output,
            &Dataset {
                header: dataset.header,
                records: dataset.records,
            },
        )?;
    }
    println!(
        "[rename] input={} output={} renames={} dropped_records={} dry_run={}",
        args.bag.display(),
        args.output.display(),
        renames.len(),
        dropped,
        args.dry_run
    );
    Ok(())
}

fn parse_renames(values: &[String]) -> anyhow::Result<BTreeMap<String, String>> {
    let mut output = BTreeMap::new();
    for value in values {
        let (source, destination) = value
            .split_once('=')
            .ok_or_else(|| anyhow::anyhow!("invalid rename {value:?}; expected OLD=NEW"))?;
        let source = source.trim();
        let destination = destination.trim();
        if source.is_empty() || destination.is_empty() {
            anyhow::bail!("invalid rename {value:?}; stream names must not be empty");
        }
        if output
            .insert(source.to_string(), destination.to_string())
            .is_some()
        {
            anyhow::bail!("source stream {source:?} is renamed more than once");
        }
    }
    Ok(output)
}
