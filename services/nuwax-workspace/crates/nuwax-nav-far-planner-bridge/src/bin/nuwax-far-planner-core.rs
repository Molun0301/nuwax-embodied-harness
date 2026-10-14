use std::io::{self, BufRead};

use nuwax_nav_far_planner_bridge::core::{plan, CircleObstacle, FarCoreConfig, Point2};

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut lines = io::stdin().lock().lines();
    let header_line = lines
        .next()
        .ok_or("missing header")?
        .map_err(|error| error.to_string())?;
    let header = numbers(&header_line)?;
    if header.len() != 5 {
        return Err("header must be: start_x start_y goal_x goal_y clearance".into());
    }
    let obstacles = lines
        .map(|line| {
            let values = numbers(&line.map_err(|error| error.to_string())?)?;
            if values.len() != 3 {
                return Err("obstacle must be: x y radius".into());
            }
            Ok(CircleObstacle {
                center: Point2 {
                    x: values[0],
                    y: values[1],
                },
                radius: values[2],
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let path = plan(
        Point2 {
            x: header[0],
            y: header[1],
        },
        Point2 {
            x: header[2],
            y: header[3],
        },
        &obstacles,
        FarCoreConfig {
            clearance: header[4],
            ..FarCoreConfig::default()
        },
    )
    .ok_or("no path")?;
    for point in path {
        println!("{:.6} {:.6}", point.x, point.y);
    }
    Ok(())
}

fn numbers(line: &str) -> Result<Vec<f64>, String> {
    line.split_whitespace()
        .map(|value| value.parse::<f64>().map_err(|error| error.to_string()))
        .collect()
}
