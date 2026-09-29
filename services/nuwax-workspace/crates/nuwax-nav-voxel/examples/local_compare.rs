//! 原始点云局部规划对照进程；依赖树不包含MLS或生产跟随器。
use nuwax_nav_voxel::{BodyModel, BodyPose, VoxelMap};
use serde_json::{json, Value};
use std::{fs, time::Instant};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config: Value =
        serde_json::from_slice(&fs::read(std::env::args().nth(1).ok_or("缺少输入配置")?)?)?;
    let model: BodyModel = serde_json::from_value(config["model"].clone())?;
    let bytes = fs::read(config["cloud"].as_str().ok_or("缺少点云")?)?;
    let stride = config["stride"].as_u64().unwrap_or(16) as usize;
    if ![12, 16].contains(&stride) || bytes.len() % stride != 0 {
        return Err("点云格式无效".into());
    }
    let points: Vec<_> = bytes
        .chunks_exact(stride)
        .map(|p| {
            let read = |i| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
            (read(0), read(4), read(8))
        })
        .collect();
    let resolution = config["resolution"].as_f64().unwrap_or(0.05) as f32;
    let began = Instant::now();
    let map = VoxelMap::from_points(&points, resolution, model)?;
    let map = if config["surface_measurements"].as_bool().unwrap_or(false) {
        map.with_surface_measurements()
    } else {
        map
    };
    let build_ms = began.elapsed().as_secs_f64() * 1000.0;
    let mut results = Vec::new();
    for query in config["queries"].as_array().ok_or("缺少查询")? {
        let start: BodyPose = serde_json::from_value(query["start"].clone())?;
        let reference: Vec<[f64; 3]> = serde_json::from_value(query["reference"].clone())?;
        let began = Instant::now();
        let arrival = if query["arrival"].is_null() {
            None
        } else {
            Some(serde_json::from_value(query["arrival"].clone())?)
        };
        let result = map.plan_reference_with_goal(start, &reference, 0.81, arrival);
        results.push(json!({"id":query["id"],"elapsed_ms":began.elapsed().as_secs_f64()*1000.0,"result":result}));
    }
    println!(
        "{}",
        json!({"build_ms":build_ms,"queries":results,"backend":"independent_raw_voxel_local_geometry"})
    );
    Ok(())
}
