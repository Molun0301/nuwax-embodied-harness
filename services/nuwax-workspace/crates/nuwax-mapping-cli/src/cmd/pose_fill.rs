//! `pose-fill` 子命令

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct PoseFillArgs {
    /// 输入轨迹 CSV 路径
    #[arg(long)]
    pub input: PathBuf,
    /// 输出路径
    #[arg(long)]
    pub output: PathBuf,
    /// 采样率（Hz）
    #[arg(long, default_value_t = 100.0)]
    pub rate: f64,
}

pub async fn run(args: PoseFillArgs) -> anyhow::Result<()> {
    tracing::info!(
        "pose-fill: input={:?} output={:?} rate={}",
        args.input,
        args.output,
        args.rate
    );
    println!(
        "[pose-fill] (placeholder) input={} output={} rate={}",
        args.input.display(),
        args.output.display(),
        args.rate
    );
    Ok(())
}
