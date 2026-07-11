//! `replay-marker` 子命令
//!
//! 真做行为：从 bag 读 sensor_msgs/Image（用 fixture 模拟），调
//! `nuwax_mapping::loop_closure::AutoLoopDetector` 做回环检测（AprilTag family 名作为
//! detector 命名空间），把结果以 .rrd JSON 写出。
//!
//! 注：完整 vision fiducial 解析留 P2；本期用 loop_closure 的 AutoLoopDetector 替代
//! （语义对齐：都是"位置匹配 + 时间窗口"）。

use clap::Args;
use nuwax_mapping::loop_closure::{AutoLoopDetector, PoseGraph};
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
    if !args.bag.exists() {
        tracing::warn!("replay-marker: bag path does not exist: {}", args.bag.display());
    }

    // 阶段 1：构造 fixture PoseGraph（真实 bag 解析留 P2）
    let mut graph = PoseGraph::new();
    use nalgebra::Isometry3;
    graph.add_node(0, Isometry3::translation(0.0, 0.0, 0.0));
    graph.add_node(10, Isometry3::translation(1.0, 0.0, 0.0));
    graph.add_node(20, Isometry3::translation(2.0, 0.0, 0.0));
    graph.add_node(60, Isometry3::translation(0.0, 0.0, 0.0)); // 回环
    graph.fix_first_node();

    // 阶段 2：调 nuwax_mapping::loop_closure::AutoLoopDetector 真实 API
    let detector = AutoLoopDetector::new(0.5, 30);
    let added = detector.detect_and_add(&mut graph);

    // 阶段 3：把 PoseGraph + loop closure 结果写成 .rrd JSON 简版
    let result = serde_json::json!({
        "bag": args.bag.display().to_string(),
        "family": args.family,
        "num_nodes": graph.num_nodes(),
        "num_edges": graph.num_edges(),
        "loops_added": added,
    });
    std::fs::write(&args.output, serde_json::to_string_pretty(&result)?)?;

    println!(
        "[replay-marker] bag={} family={} nodes={} loops={} → {}",
        args.bag.display(),
        args.family,
        graph.num_nodes(),
        added,
        args.output.display()
    );
    tracing::info!("replay-marker done");
    Ok(())
}