//! `map` 子命令
//! **占位实装**：完整实装需与 `nuwax-visualization-rerun` 集成，当前仅打印参数。

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct MapArgs {
    /// 输入 bag 路径
    #[arg(long)]
    pub bag: PathBuf,
    /// 输出地图路径
    #[arg(long)]
    pub output: Option<PathBuf>,
    /// 主题（占位）
    #[arg(long, default_value = "default")]
    pub theme: String,
}

pub async fn run(args: MapArgs) -> anyhow::Result<()> {
    tracing::info!(
        "map: bag={:?} output={:?} theme={}",
        args.bag,
        args.output,
        args.theme
    );
    // 占位：实装需集成 nuwax-visualization-rerun
    println!(
        "[map] (placeholder) bag={} output={:?} theme={}",
        args.bag.display(),
        args.output,
        args.theme
    );
    Ok(())
}
