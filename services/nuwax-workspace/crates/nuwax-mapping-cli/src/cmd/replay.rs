//! `replay` 子命令

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct ReplayArgs {
    /// 输入 bag 路径
    #[arg(long)]
    pub bag: PathBuf,
    /// 起始时间偏移（秒）
    #[arg(long, default_value_t = 0.0)]
    pub start_offset: f64,
    /// 倍速
    #[arg(long, default_value_t = 1.0)]
    pub rate: f64,
    /// dry-run（仅打印参数）
    #[arg(long)]
    pub dry_run: bool,
}

pub async fn run(args: ReplayArgs) -> anyhow::Result<()> {
    if args.dry_run {
        println!(
            "[replay] (dry-run) bag={} start_offset={} rate={}",
            args.bag.display(),
            args.start_offset,
            args.rate
        );
        return Ok(());
    }
    tracing::info!(
        "replay: bag={:?} start_offset={} rate={}",
        args.bag,
        args.start_offset,
        args.rate
    );
    println!(
        "[replay] (placeholder) bag={} start_offset={} rate={}",
        args.bag.display(),
        args.start_offset,
        args.rate
    );
    Ok(())
}
