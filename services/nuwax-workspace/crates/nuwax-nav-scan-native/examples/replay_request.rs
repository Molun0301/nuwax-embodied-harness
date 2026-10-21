//! 复放真实调用输入，保留所有感知、姿态、包络和运动能力。
use nuwax_nav_scan_native::{plan, Config, PlanRequest};
use serde_json::{json, Value};
use std::{fs, time::Instant};
fn main() {
    let args: Vec<_> = std::env::args().collect();
    let value: Value = serde_json::from_slice(&fs::read(&args[1]).unwrap()).unwrap();
    let config = &value["config"];
    let n = |key: &str| config[key].as_f64().unwrap();
    let cfg = Config {
        resolution: n("resolution"),
        body_length: n("body_length"),
        body_width: n("body_width"),
        body_height: n("body_height"),
        body_to_ground: n("body_to_ground"),
        max_velocity: n("max_velocity"),
        max_acceleration: n("max_acceleration"),
        max_yaw_rate: n("max_yaw_rate"),
        planning_horizon: n("planning_horizon"),
        sample_dt: n("sample_dt"),
        body_center_x: n("body_center_x"),
        body_center_y: n("body_center_y"),
        body_z_min: n("body_z_min"),
        body_z_max: n("body_z_max"),
        max_step_up: n("max_step_up"),
        max_step_down: n("max_step_down"),
        max_slope: n("max_slope"),
        max_yaw_acceleration: n("max_yaw_acceleration"),
    };
    let points = |key: &str| serde_json::from_value::<Vec<[f64; 3]>>(value[key].clone()).unwrap();
    let obstacles = points("obstacles");
    let ground = points("ground");
    let reference = points("reference");
    let boundary = serde_json::from_value::<Vec<[f64; 2]>>(value["boundary"].clone()).unwrap();
    let forbidden =
        serde_json::from_value::<Vec<[i32; 2]>>(value["forbidden_columns"].clone()).unwrap();
    let p = |key: &str| serde_json::from_value::<[f64; 3]>(value[key].clone()).unwrap();
    let req = PlanRequest {
        obstacles: &obstacles,
        ground: &ground,
        reference: &reference,
        boundary: &boundary,
        forbidden_columns: &forbidden,
        start_position: p("start_position"),
        start_velocity: p("start_velocity"),
        start_acceleration: p("start_acceleration"),
        start_yaw: value["start_yaw"].as_f64().unwrap(),
        start_yaw_rate: value["start_yaw_rate"].as_f64().unwrap(),
        odometry_timestamp_ns: value
            .get("odometry_timestamp_ns")
            .and_then(Value::as_i64)
            .unwrap_or(0),
        goal_yaw: value["goal_yaw"].as_f64(),
        config: cfg,
    };
    let began = Instant::now();
    let result = plan(&req);
    let ms = began.elapsed().as_secs_f64() * 1000.0;
    let output = match result {
        Ok(p) => {
            json!({"success":true,"wall_ms":ms,"obstacles":obstacles.len(),"ground":ground.len(),"reference":reference.len(),
        "command":[p.command_forward,p.command_yaw_rate],"duration":p.duration,"reference_progress":p.reference_progress,
        "local_target":p.local_target,"samples":p.samples.iter().map(|s|json!({"t":s.time,"position":s.position,"velocity":s.velocity})).collect::<Vec<_>>()})
        }
        Err(e) => {
            json!({"success":false,"wall_ms":ms,"error":e,"obstacles":obstacles.len(),"ground":ground.len(),"reference":reference.len()})
        }
    };
    if let Some(path) = args.get(2) {
        fs::write(path, serde_json::to_vec_pretty(&output).unwrap()).unwrap();
    }
    println!(
        "{}",
        json!({"success":output["success"],"wall_ms":ms,"error":output.get("error"),"command":output.get("command"),"obstacles":obstacles.len(),"ground":ground.len()})
    );
}
