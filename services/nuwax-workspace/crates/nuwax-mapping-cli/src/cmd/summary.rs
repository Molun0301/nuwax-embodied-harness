//! `summary` 子命令

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct SummaryArgs {
    /// 输入 bag 路径
    #[arg(long)]
    pub bag: PathBuf,
}

pub async fn run(args: SummaryArgs) -> anyhow::Result<()> {
    tracing::info!("summary: bag={:?}", args.bag);
    println!("[summary] (placeholder) bag={}", args.bag.display());
    Ok(())
}
