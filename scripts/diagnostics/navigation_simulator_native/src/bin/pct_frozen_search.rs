//! 复跑原502秒样本的实测尺寸、细体素、墙体和全局起终点。
use nuwax_nav_pct_native::{Config, Planner};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let q = f64::from(0.01_f32);
    let mut points = Vec::new();
    for x in -130..=130 {
        for y in -70..100 {
            points.push([(f64::from(x)+0.5)*q,(f64::from(y)+0.5)*q,-0.5*q]);
        }
    }
    for x in 0..4 {
        for y in -70..100 {
            for z in 0..65 {
                points.push([(f64::from(x)+0.5)*q,(f64::from(y)+0.5)*q,(f64::from(z)+0.5)*q]);
            }
        }
    }
    let config = Config {resolution:q,point_resolution:q,robot_height:0.55,body_to_ground:0.4,
        body_z_min:-0.15,body_z_max:0.15,max_step:0.15,max_step_down:0.15,
        max_slope:18.0_f64.to_radians(),footprint_radius:0.34,
        footprint:vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
        ..Config::default()};
    let begin = std::time::Instant::now();
    let mut planner = Planner::new(config,&points)?;
    let prepared_ms = begin.elapsed().as_secs_f64()*1000.0;
    let search_begin = std::time::Instant::now();
    let route = planner.plan([-1.0,0.2,0.4],[1.0,0.2,0.4]);
    let search_ms = search_begin.elapsed().as_secs_f64()*1000.0;
    let result = match route {
        Ok(route) => serde_json::json!({"status":"planned","search_ms":route.elapsed_ms,
            "expanded":route.expanded,"path":route.path}),
        Err(error) => serde_json::json!({"status":"rejected","search_ms":search_ms,
            "error":error.to_string()}),
    };
    println!("{}",serde_json::json!({"case":"原502秒全局绕墙","source_resolution_m":q,
        "ground_columns":261*170,"input_points":points.len(),"prepared_ms":prepared_ms,
        "result":result,"global_boundary":[],
        "scope":"相同细体素与实际机体/墙/起终点。墙横断全部已知地面，恢复原PCT未知语义后不再补造墙外地面，预期明确拒绝。原502秒及补造未知地面的绕行耗时仅作历史记录，不计算同语义加速倍率。"}));
    Ok(())
}
