use nuwax_explore_terrain_bridge::core::{analyze, Point3, TerrainClass, TerrainCoreConfig};
use std::io::{self, BufRead};
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
fn run() -> Result<(), String> {
    let points = io::stdin()
        .lock()
        .lines()
        .map(|line| {
            let values = parse(&line.map_err(|e| e.to_string())?)?;
            if values.len() != 3 {
                return Err("point needs x y z".into());
            }
            Ok(Point3 {
                x: values[0],
                y: values[1],
                z: values[2],
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let cells = analyze(
        &points,
        Point3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        TerrainCoreConfig::default(),
    )
    .ok_or("invalid config")?;
    for cell in cells {
        let class = if cell.class == TerrainClass::Ground {
            "ground"
        } else {
            "obstacle"
        };
        println!(
            "{:.6} {:.6} {:.6} {class} {}",
            cell.x, cell.y, cell.z, cell.samples
        );
    }
    Ok(())
}
fn parse(line: &str) -> Result<Vec<f64>, String> {
    line.split_whitespace()
        .map(|v| v.parse::<f64>().map_err(|e| e.to_string()))
        .collect()
}
