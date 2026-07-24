use nuwax_explore_tare_bridge::core::{select_waypoint, FrontierPoint, RobotPose, TareCoreConfig};
use std::io::{self, BufRead};
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
fn run() -> Result<(), String> {
    let mut lines = io::stdin().lock().lines();
    let robot = parse(
        &lines
            .next()
            .ok_or("missing robot")?
            .map_err(|e| e.to_string())?,
    )?;
    if robot.len() != 2 {
        return Err("robot needs x y".into());
    }
    let frontiers = lines
        .map(|line| {
            let values = parse(&line.map_err(|e| e.to_string())?)?;
            if values.len() != 3 {
                return Err("frontier needs x y gain".into());
            }
            Ok(FrontierPoint {
                x: values[0],
                y: values[1],
                gain: values[2],
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let waypoint = select_waypoint(
        RobotPose {
            x: robot[0],
            y: robot[1],
        },
        &frontiers,
        TareCoreConfig::default(),
    )
    .ok_or("no waypoint")?;
    println!(
        "{:.6} {:.6} {:.6} {}",
        waypoint.x, waypoint.y, waypoint.score, waypoint.support
    );
    Ok(())
}
fn parse(line: &str) -> Result<Vec<f64>, String> {
    line.split_whitespace()
        .map(|v| v.parse::<f64>().map_err(|e| e.to_string()))
        .collect()
}
