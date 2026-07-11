//! `map` 子命令
//!
//! 真做行为：从 bag 读取 OccupancyGrid（简化为内存 fixture 演示），
//! 调 `nuwax_mapping::CostMapper::convert` 转 Costmap，写 .pgm 文本格式输出。
//!
//! 注：完整 Rerun 集成留 P2；本期先用文件输出 + ASCII art 兜底（保证可测可运行）。

use clap::Args;
use nuwax_mapping::cost_mapper::{CostMapper, CostMapperConfig};
use nuwax_mapping::occupancy::types::{OccupancyGrid, OccupancyValue};
use std::path::PathBuf;

#[derive(Debug, Args)]
pub struct MapArgs {
    /// 输入 bag 路径（本期先用 dummy fixture）
    #[arg(long)]
    pub bag: PathBuf,
    /// 输出地图路径（.pgm 灰度图或 .txt ASCII art）
    #[arg(long)]
    pub output: Option<PathBuf>,
    /// 主题（"default" / "dark" / "light"）
    #[arg(long, default_value = "default")]
    pub theme: String,
}

pub async fn run(args: MapArgs) -> anyhow::Result<()> {
    // 阶段 1：构造一个 fixture OccupancyGrid（演示用；真实 bag 解析留 P2）
    let mut grid = OccupancyGrid::new(20, 20, 0.1);
    // 画一个十字障碍
    for c in 5..15 {
        grid.set(10, c, OccupancyValue::Occupied);
    }
    for r in 5..15 {
        grid.set(r, 10, OccupancyValue::Occupied);
    }

    // 阶段 2：调 nuwax_mapping 真实 API: CostMapper::convert
    let mapper = CostMapper::new(CostMapperConfig::dimos_default());
    let costmap = mapper.convert(&grid);

    // 阶段 3：写到输出文件（.pgm 灰度图）或 stdout
    let output = args.output.clone().unwrap_or_else(|| {
        let mut p = args.bag.clone();
        p.set_extension("pgm");
        p
    });
    write_pgm(&output, &costmap)?;

    println!(
        "[map] bag={} theme={} costmap={}x{} → {}",
        args.bag.display(),
        args.theme,
        costmap.width(),
        costmap.height(),
        output.display()
    );

    // 阶段 4：ASCII art 兜底（terminal 直接可视化）
    if args.output.is_none() {
        println!("ASCII art (cost 0=., 1-50=:, 51-100=#, 255=X):");
        for r in 0..costmap.height() {
            for c in 0..costmap.width() {
                let v = costmap.get(r, c).unwrap_or(0);
                let ch = match v {
                    0 => '.',
                    1..=50 => ':',
                    51..=100 => '#',
                    _ => 'X',
                };
                print!("{ch}");
            }
            println!();
        }
    }

    tracing::info!("map done: wrote {}", output.display());
    Ok(())
}

/// 写 .pgm 灰度图（P5 二进制格式，简单且通用）
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
            // 翻转：cost=255 (obstacle) → 0 (黑), cost=0 (free) → 255 (白)
            let gray = 255 - v;
            f.write_all(&[gray])?;
        }
    }
    Ok(())
}