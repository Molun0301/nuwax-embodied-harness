//! nuwax-mapping-cli binary entrypoint
//! - `rename.py` → `nuwax-mapping-cli rename`
//! - `pose_fill.py` → `nuwax-mapping-cli pose-fill`
//! - `summary.py` → `nuwax-mapping-cli summary`
//! - `map.py` → `nuwax-mapping-cli map`
//! - `replay.py` → `nuwax-mapping-cli replay`
//! - `replay_marker.py` → `nuwax-mapping-cli replay-marker`

use clap::Parser;
use nuwax_mapping_cli::{run, Cli};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    run(cli).await
}