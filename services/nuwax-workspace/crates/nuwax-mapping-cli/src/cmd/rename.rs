//! `rename` 子命令

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct RenameArgs {
    /// 输入 bag 路径
    #[arg(long)]
    pub bag: PathBuf,
    /// 新名称前缀
    #[arg(long)]
    pub prefix: String,
    /// dry-run
    #[arg(long)]
    pub dry_run: bool,
}

pub async fn run(args: RenameArgs) -> anyhow::Result<()> {
    if args.dry_run {
        println!(
            "[rename] (dry-run) bag={} prefix={}",
            args.bag.display(),
            args.prefix
        );
        return Ok(());
    }
    tracing::info!("rename: bag={:?} prefix={}", args.bag, args.prefix);
    println!(
        "[rename] (placeholder) bag={} prefix={}",
        args.bag.display(),
        args.prefix
    );
    Ok(())
}
