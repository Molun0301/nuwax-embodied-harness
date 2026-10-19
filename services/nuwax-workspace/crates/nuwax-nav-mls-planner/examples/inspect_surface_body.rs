//! 从二进制点云复核完整机体前沿候选；不修改地图或创建运动任务。
use nuwax_nav_mls_planner::{mls_planner::Config,SurfaceSnapshot,SurfaceBodyEnvelope};
fn main(){
    let path=std::env::args().nth(1).expect("点云路径");let data=std::fs::read(path).unwrap();let marker=b"DATA binary\n";let start=data.windows(marker.len()).position(|w|w==marker).unwrap()+marker.len();
    let points:Vec<_>=data[start..].chunks_exact(12).map(|b|(f32::from_le_bytes(b[..4].try_into().unwrap()),f32::from_le_bytes(b[4..8].try_into().unwrap()),f32::from_le_bytes(b[8..12].try_into().unwrap()))).collect();
    let cfg=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.5,surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.15,wall_buffer_m:0.1,wall_buffer_weight:2.0,step_threshold_m:0.15,max_slope_deg:18.0,min_step_tread_depth_m:0.2,goal_tolerance:0.2,step_penalty_weight:2.0,viz_publish_hz:0.0};
    let model=SurfaceBodyEnvelope{fixed_collision:None,footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.35,max_step_up_m:0.15,max_step_down_m:0.15,max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model).unwrap();let candidates=map.reachable_frontiers([0.0,0.0,-0.35],&[]).unwrap();let mut errors=std::collections::BTreeMap::new();let mut valid=0;let mut examples=vec![];
    for candidate in &candidates {let p=candidate.support_xyz;let yaw=p[1].atan2(p[0]);let q=map.body_orientation_at(p,yaw).unwrap();let body=map.body_position_from_support(p,q,0.35).unwrap();match map.validate_body_pose(body,q){Ok(())=>{valid+=1;},Err(e)=>{*errors.entry(e.to_string()).or_insert(0)+=1;if examples.len()<3{examples.push((p,body,e.to_string()));}}}}
    println!("单元={} 前沿={} 完整机体有效={} 错误={errors:?} 示例={examples:?}",map.cell_keys().count(),candidates.len(),valid);
}
