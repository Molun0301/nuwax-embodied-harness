//! nuwax-mapping-cli 库入口
//!
//! binary `nuwax-mapping-cli` 在 `main.rs` 调本 crate 的 `run()`。
//! 集成测试（`tests/*.rs`）可直接用 `nuwax_mapping_cli::cmd::*`。

pub mod cmd;

use clap::Parser;

#[derive(Debug, Parser)]
#[command(name = "nuwax-mapping-cli", version, about = "nuwax mapping utilities")]
pub struct Cli {
    #[command(subcommand)]
    pub command: Command,
}

#[derive(Debug, clap::Subcommand)]
pub enum Command {
    /// 生成地图（PGM + ASCII art）
    Map(cmd::map::MapArgs),
    /// pose 插值
    PoseFill(cmd::pose_fill::PoseFillArgs),
    /// 重命名 bag 文件
    Rename(cmd::rename::RenameArgs),
    /// 回放 PointCloud2 → costmap
    Replay(cmd::replay::ReplayArgs),
    /// 回放 marker → loop closure
    ReplayMarker(cmd::replay_marker::ReplayMarkerArgs),
    /// bag 目录摘要
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