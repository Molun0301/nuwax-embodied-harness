//! 验证完整原生图在另一架构恢复后的物理栅格，与保存源逐字节对账。
use nuwax_slam_toolbox::{Config, Pose, Slam};
use std::{fs, path::PathBuf};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let directory = PathBuf::from(std::env::args_os().nth(1).ok_or("需要跨架构图验证目录")?);
    let config: Config = serde_json::from_slice(&fs::read(directory.join("config.json"))?)?;
    let expected: serde_json::Value =
        serde_json::from_slice(&fs::read(directory.join("grid.json"))?)?;
    let mut slam = Slam::new(config)?;
    slam.load(&directory.join("graph"), true, Pose::default())?;
    let grid = slam.occupancy_grid()?;
    assert_eq!(
        grid.metadata.width as u64,
        expected["width"].as_u64().unwrap()
    );
    assert_eq!(
        grid.metadata.height as u64,
        expected["height"].as_u64().unwrap()
    );
    assert_eq!(
        grid.metadata.resolution,
        expected["resolution"].as_f64().unwrap()
    );
    assert_eq!(
        [grid.metadata.origin_x, grid.metadata.origin_y],
        [
            expected["origin"][0].as_f64().unwrap(),
            expected["origin"][1].as_f64().unwrap()
        ]
    );
    assert_eq!(
        grid.cells.iter().map(|v| *v as u8).collect::<Vec<_>>(),
        fs::read(directory.join("grid.i8"))?
    );
    println!(
        "跨架构原生图加载与物理栅格逐字节验证通过：{}×{}",
        grid.metadata.width, grid.metadata.height
    );
    Ok(())
}
