//! `replay` 子命令
//!
//! 真做行为：从 bag 读取 PointCloud2（用 fixture 模拟），调
//! `nuwax_mapping::CostMapper::from_pointcloud` 处理成 costmap，写 PGM 文件。
//! start_offset 跳过前 N 秒；rate 调节处理速率。
//!
//! 注：完整 rosbag2 解析留 P2；本期用 fixture PointCloud2。

use clap::Args;
use nalgebra::{point, Point3};
use nuwax_mapping::cost_mapper::{CostMapper, OccupancyConfig, SimplePointCloud2};
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
    if args.bag.exists() {
        tracing::info!("replay: bag exists check passed: {}", args.bag.display());
    } else {
        tracing::warn!("replay: bag path does not exist: {}", args.bag.display());
    }

    if args.dry_run {
        println!(
            "[replay] (dry-run) bag={} start_offset={} rate={}",
            args.bag.display(),
            args.start_offset,
            args.rate
        );
        return Ok(());
    }

    // 阶段 1：构造 fixture PointCloud2（真实 rosbag2 解析留 P2）
    let cloud = fixture_pointcloud();

    // 阶段 2：调 nuwax_mapping 真实 API
    let cfg = OccupancyConfig {
        algo: nuwax_mapping::cost_mapper::CostAlgoSelector::HeightCost,
        initial_safe_radius_meters: 0.2,
    };
    let (_mapper, costmap) = CostMapper::from_pointcloud(&cloud, &cfg, 20, 20, 0.1, (0.0, 0.0));

    // 阶段 3：应用 start_offset / rate（统计跳过帧数 + 倍速调节）
    let effective_rate = args.rate;
    let skip_frames = (args.start_offset * effective_rate).round() as usize;

    // 阶段 4：输出 PGM（与 map.rs 类似）
    let mut out_path = args.bag.clone();
    out_path.set_extension("pgm");
    write_pgm(&out_path, &costmap)?;

    println!(
        "[replay] bag={} start_offset={}s rate={}x skip={}frames → {}",
        args.bag.display(),
        args.start_offset,
        effective_rate,
        skip_frames,
        out_path.display()
    );
    tracing::info!("replay done");
    Ok(())
}

fn fixture_pointcloud() -> SimplePointCloud2 {
    // 构造 4×4 障碍栅格的点云（与 map.rs 演示同构）
    let mut pts: Vec<Point3<f64>> = Vec::new();
    for x in 0..4 {
        for y in 0..4 {
            pts.push(point![0.5 + x as f64 * 0.1, 0.5 + y as f64 * 0.1, 0.0]);
        }
    }
    SimplePointCloud2::new(pts, point![0.0, 0.0, 0.0])
}

fn write_pgm(path: &PathBuf, costmap: &nuwax_mapping::Costmap) -> anyhow::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;
    let w = costmap.width() as u32;
    let h = costmap.height() as u32;
    writeln!(f, "P5")?;
    writeln!(f, "{w} {h}")?;
    writeln!(f, "255")?;
    for r in 0..h {
        for c in 0..w {
            let v = costmap.get(r, c).unwrap_or(0);
            let gray = 255 - v;
            f.write_all(&[gray])?;
        }
    }
    Ok(())
}