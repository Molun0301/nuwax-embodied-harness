//! 离线地图处理命令行入口：先解析参数，再执行所选地图操作。
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
