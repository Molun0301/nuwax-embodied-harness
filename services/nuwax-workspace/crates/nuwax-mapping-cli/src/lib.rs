//! nuwax-mapping-cli 库入口
//!
//! binary `nuwax-mapping-cli` 在 `main.rs` 调本 crate 的 `run()`。
//! 集成测试（`tests/*.rs`）可直接用 `nuwax_mapping_cli::cmd::*`。
//!
//! The supported persisted input schema is documented in the crate-local
//! `FORMAT.md` and implemented by [`dataset`].

pub mod cmd;
pub mod dataset;
pub mod native;

use clap::Parser;

#[derive(Debug, Parser)]
#[command(name = "nuwax-mapping-cli", version, about = "nuwax mapping utilities")]
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Debug, clap::Subcommand)]
pub enum Command {
    /// Build a PGM map from persisted grid or point-cloud records
    Map(cmd::map::MapArgs),
    /// Attach nearest persisted poses to a target stream
    PoseFill(cmd::pose_fill::PoseFillArgs),
    /// Persist logical stream renames and drops
    Rename(cmd::rename::RenameArgs),
    /// Build a deterministic replay schedule
    Replay(cmd::replay::ReplayArgs),
    /// Export persisted marker observations
    ReplayMarker(cmd::replay_marker::ReplayMarkerArgs),
    /// Print exact dataset counts and time ranges
    Summary(cmd::summary::SummaryArgs),
}

pub async fn run(cli: Cli) -> anyhow::Result<()> {
    match cli.command {
        Command::Map(args) => cmd::map::run(args).await,
        Command::PoseFill(args) => cmd::pose_fill::run(args).await,
        Command::Rename(args) => cmd::rename::run(args).await,
        Command::Replay(args) => cmd::replay::run(args).await,
        Command::ReplayMarker(args) => cmd::replay_marker::run(args).await,
        Command::Summary(args) => cmd::summary::run(args).await,
    }
}
