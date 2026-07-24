use nuwax_nav_local_planner_bridge::core::{
    select_trajectory, LocalCoreConfig, Obstacle, Point2, Trajectory,
};
use std::io::{self, BufRead};

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut goal = None;
    let mut obstacles = Vec::new();
    let mut candidates = Vec::new();
    for line in io::stdin().lock().lines() {
        let line = line.map_err(|error| error.to_string())?;
        let mut fields = line.split_whitespace();
        match fields.next() {
            Some("goal") => {
                if goal.is_some() {
                    return Err("duplicate goal record".into());
                }
                let parsed = Point2 {
                    x: number(fields.next())?,
                    y: number(fields.next())?,
                };
                if fields.next().is_some() {
                    return Err("goal needs exactly x y".into());
                }
                goal = Some(parsed);
            }
            Some("obstacle") => {
                let obstacle = Obstacle {
                    center: Point2 {
                        x: number(fields.next())?,
                        y: number(fields.next())?,
                    },
                    radius: number(fields.next())?,
                };
                if fields.next().is_some() {
                    return Err("obstacle needs exactly x y radius".into());
                }
                obstacles.push(obstacle);
            }
            Some("candidate") => {
                let values = fields
                    .map(str::parse::<f64>)
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|error| error.to_string())?;
                if values.len() < 4 || values.len() % 2 != 0 {
                    return Err("candidate needs x y pairs".into());
                }
                candidates.push(Trajectory {
                    points: values
                        .chunks_exact(2)
                        .map(|v| Point2 { x: v[0], y: v[1] })
                        .collect(),
                });
            }
            Some(other) => return Err(format!("unknown record {other}")),
            None => {}
        }
    }
    let (index, cost) = select_trajectory(
        goal.ok_or("missing goal")?,
        &candidates,
        &obstacles,
        LocalCoreConfig::default(),
    )
    .ok_or("no safe trajectory")?;
    println!("{index} {cost:.6}");
    Ok(())
}

fn number(value: Option<&str>) -> Result<f64, String> {
    value
        .ok_or("missing number")?
        .parse::<f64>()
        .map_err(|error| error.to_string())
}
