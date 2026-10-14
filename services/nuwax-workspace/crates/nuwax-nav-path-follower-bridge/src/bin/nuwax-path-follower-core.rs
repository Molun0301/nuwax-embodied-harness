use nuwax_nav_path_follower_bridge::core::{follow, FollowerConfig, Point2, Pose2};
use std::io::{self, Read};
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
fn run() -> Result<(), String> {
    let mut input = String::new();
    io::stdin()
        .read_to_string(&mut input)
        .map_err(|e| e.to_string())?;
    let mut lines = input.lines();
    let pose = parse(lines.next().ok_or("missing pose")?)?;
    if pose.len() != 3 {
        return Err("pose needs x y yaw".into());
    }
    let path = lines
        .map(parse)
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .map(|v| {
            if v.len() == 2 {
                Ok(Point2 { x: v[0], y: v[1] })
            } else {
                Err("path point needs x y".into())
            }
        })
        .collect::<Result<Vec<_>, String>>()?;
    let command = follow(
        Pose2 {
            x: pose[0],
            y: pose[1],
            yaw: pose[2],
        },
        &path,
        FollowerConfig::default(),
    )
    .ok_or("invalid input")?;
    println!(
        "{:.6} {:.6} {}",
        command.linear, command.angular, command.reached
    );
    Ok(())
}
fn parse(line: &str) -> Result<Vec<f64>, String> {
    line.split_whitespace()
        .map(|v| v.parse::<f64>().map_err(|e| e.to_string()))
        .collect()
}
