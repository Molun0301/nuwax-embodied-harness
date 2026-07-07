//! `replay-marker` 子命令
//! **AprilTag 检测占位**——实装需引用 `nuwax-vision-fiducial::AprilTagDetector`。

use clap::Args;
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct ReplayMarkerArgs {
    /// 输入 bag 路径
    #[arg(long)]
    pub bag: PathBuf,
    /// 输出 .rrd 路径
    #[arg(long)]
    pub output: PathBuf,
    /// AprilTag 族
    #[arg(long, default_value = "tag36h11")]
    pub family: String,
}

pub async fn run(args: ReplayMarkerArgs) -> anyhow::Result<()> {
    tracing::info!(
        "replay-marker: bag={:?} output={:?} family={}",
        args.bag,
        args.output,
        args.family
    );
    println!(
        "[replay-marker] (placeholder) bag={} output={} family={}",
        args.bag.display(),
        args.output.display(),
        args.family
    );
    // TODO: 实装时调用 nuwax_vision_fiducial::AprilTagDetector::detect(image)
    Ok(())
}
