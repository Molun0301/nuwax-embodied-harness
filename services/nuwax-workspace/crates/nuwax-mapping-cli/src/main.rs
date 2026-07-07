//! nuwax-mapping-cli
//! - `rename.py` → `nuwax-mapping-cli rename`
//! - `pose_fill.py` → `nuwax-mapping-cli pose-fill`
//! - `summary.py` → `nuwax-mapping-cli summary`

use clap::{Parser, Subcommand};
use std::path::PathBuf;

mod cmd;

#[derive(Debug, Parser)]
#[command(name = "nuwax-mapping-cli")]
#[command(about = "nuwax 建图板块 CLI")]
struct Cli {
    /// 全局配置文件（YAML）
    #[arg(long, global = true)]
    config: Option<PathBuf>,

    /// 详细日志
    #[arg(long, global = true)]
    verbose: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// Rerun 地图可视化
    Map(cmd::map::MapArgs),
    /// rosbag 回放
    Replay(cmd::replay::ReplayArgs),
    /// AprilTag 回放占位
    ReplayMarker(cmd::replay_marker::ReplayMarkerArgs),
    /// bag 重命名
    Rename(cmd::rename::RenameArgs),
    /// 位姿插值
    PoseFill(cmd::pose_fill::PoseFillArgs),
    /// bag 摘要
    Summary(cmd::summary::SummaryArgs),
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    if cli.verbose {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::DEBUG)
            .init();
    } else {
        tracing_subscriber::fmt()
            .with_max_level(tracing::Level::INFO)
            .init();
    }

    match cli.command {
        Commands::Map(args) => cmd::map::run(args).await,
        Commands::Replay(args) => cmd::replay::run(args).await,
        Commands::ReplayMarker(args) => cmd::replay_marker::run(args).await,
        Commands::Rename(args) => cmd::rename::run(args).await,
        Commands::PoseFill(args) => cmd::pose_fill::run(args).await,
        Commands::Summary(args) => cmd::summary::run(args).await,
    }
}
