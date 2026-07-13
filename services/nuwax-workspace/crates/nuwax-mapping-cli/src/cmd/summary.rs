//! Print exact per-stream and per-payload counts for a persisted dataset.

use std::io::Write;
use std::path::PathBuf;

use clap::Args;

use crate::dataset::summarize;
use crate::native::{input_format, read_command_dataset, summarize_memory2, InputFormat};

/// Arguments for `summary`.
#[derive(Debug, Args)]
pub struct SummaryArgs {
    /// Input JSONL or DimOS memory2 `.db` dataset.
    #[arg(long)]
    pub bag: PathBuf,
    /// Optional JSON output path. Existing files are never overwritten.
    #[arg(long)]
    pub output: Option<PathBuf>,
}

/// Execute the `summary` command.
///
/// # Errors
/// Returns input schema, output collision, serialization, or I/O errors.
pub async fn run(args: SummaryArgs) -> anyhow::Result<()> {
    let summary = match input_format(&args.bag)? {
        InputFormat::Memory2Sqlite => summarize_memory2(&args.bag)?,
        InputFormat::JsonLines => summarize(&read_command_dataset(&args.bag)?.dataset),
        InputFormat::Rosbag2Sqlite | InputFormat::Mcap | InputFormat::Ros1Bag => {
            return read_command_dataset(&args.bag).map(|_| ());
        }
    };
    let json = serde_json::to_string_pretty(&summary)?;
    println!("{json}");
    if let Some(output) = args.output {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&output)
            .map_err(|error| {
                anyhow::anyhow!("create summary output {} failed: {error}", output.display())
            })?;
        file.write_all(json.as_bytes())?;
        file.write_all(b"\n")?;
        file.sync_all()?;
    }
    Ok(())
}
