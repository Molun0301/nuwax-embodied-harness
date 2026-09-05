//! nuwax-mapping-cli binary entrypoint
//! - `rename.py` → `nuwax-mapping-cli rename`
//! - `pose_fill.py` → `nuwax-mapping-cli pose-fill`
//! - `summary.py` → `nuwax-mapping-cli summary`
//! - `map.py` → `nuwax-mapping-cli map`

use clap::Parser;
use nuwax_mapping_cli::{run, Cli};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    nuwax_core::logging::init_component_logging("nuwax-mapping-cli", "info");

    let result = run(cli).await;
    if let Err(error) = &result {
        nuwax_core::logging::process_error("nuwax-mapping-cli", error);
    }
    result
}
