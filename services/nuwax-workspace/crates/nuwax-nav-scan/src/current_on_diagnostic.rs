use super::*;
use nuwax_nav_mls_planner::{SurfaceBodyEnvelope,SurfaceMapBuilder,SurfaceObservations,SurfaceObservation,mls_planner::Config,surface::GroundEvidence};
use std::{collections::BTreeMap,sync::Arc};
#[test]
#[ignore="local captured current ON map and two scan inputs"]
fn reproduce_current_on() {
 let _subscriber=tracing::subscriber::set_default(ClearanceSubscriber);
 let root=std::env::var("NEH_ON_FIXTURE").unwrap();
 let evidence:serde_json::Value=serde_json::from_slice(&std::fs::read(std::env::var("NEH_ON_INPUT").unwrap_or_else(|_|format!("{root}/current-on-scan-evidence.json"))).unwrap()).unwrap();
 let catalog:serde_json::Value=serde_json::from_slice(&std::fs::read(format!("{root}/current-catalog.json")).unwrap()).unwrap();
 let scene=catalog["scenes"].as_array().unwrap().iter().find(|s|s["id"]=="recorded").unwrap();
 let bytes=std::fs::read(catalog["service_version"]["source_cloud"].as_str().unwrap()).unwrap();
 let points:Vec<_>=bytes.chunks_exact(16).map(|b|(f32::from_le_bytes(b[0..4].try_into().unwrap()),f32::from_le_bytes(b[4..8].try_into().unwrap()),f32::from_le_bytes(b[8..12].try_into().unwrap()))).collect();
 let model=SurfaceBodyEnvelope{fixed_collision:None,footprint_xy_m:serde_json::from_value(scene["robot"]["footprint_xy"].clone()).unwrap(),z_bounds_m:serde_json::from_value(scene["robot"]["z_bounds"].clone()).unwrap(),body_to_support_z_m:0.4,max_step_up_m:f64::from(0.15_f32),max_step_down_m:f64::from(0.15_f32),max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
 let cfg=Config{world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:(model.radius()+f64::from(0.1_f32))as f32,wall_buffer_weight:2.0,step_threshold_m:0.15,max_slope_deg:18.0,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,goal_tolerance:0.2,viz_publish_hz:0.0};
 let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
 for &(x,y,z) in &points {let k=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor()as i32);blocks.entry(k.map(|v|v.div_euclid(32))).or_default().insert(k,SurfaceObservation{observed_ns:0,hits:0});}
 let mut obs=SurfaceObservations::new(0.05,32).unwrap();for(k,v)in blocks {obs.replace_block(k,v).unwrap();}obs.mark_complete_geometry_source();
 let source=SurfaceMapBuilder::configured(cfg,Some(model)).build(&points,0.05).unwrap().snapshot.with_observations(Some(Arc::new(obs))).with_ground_evidence(GroundEvidence::MeasuredSurface);
 for event in evidence.as_array().unwrap() {
  let input=&event["scan_input"];
  let start:BodyPose=serde_json::from_value(input["start"].clone()).unwrap();
  let reference:Vec<[f64;3]>=serde_json::from_value(input["reference"].clone()).unwrap();
  let map=source.with_current_body_support(start.0,start.1).unwrap().fork_for_planning();
  let tomo=Tomogram::local_from_surface(&map,[start.0[0],start.0[1]],6.5).unwrap();
  let motion=MotionKinematics{model:nuwax_nav_types_messages::MotionModel::Differential,max_forward_mps:input["motion"]["forward"].as_f64().unwrap(),max_reverse_mps:0.0,max_lateral_mps:0.0,max_angular_rps:input["motion"]["angular"].as_f64().unwrap(),minimum_turning_radius_m:input["motion"]["radius"].as_f64().unwrap(),can_rotate_in_place:input["motion"]["can_rotate"].as_bool().unwrap()};
  let arrival:Option<([f64;3],f64)>=serde_json::from_value(input["arrival"].clone()).unwrap();
  let final_yaw:Option<f64>=serde_json::from_value(input["final_yaw"].clone()).unwrap();
  let began=std::time::Instant::now();
  let result=plan(&map,&tomo,start,&reference,&motion,&ScanConfig::default(),final_yaw,arrival,&[],input["follow_reference"].as_bool().unwrap(),false);
  if let Ok(route)=&result {
   assert_eq!(route.poses[0],start);
   let mut checked=0;
   for pair in route.poses.windows(2) {
    map.validate_body_motion(pair[0],pair[1],&[]).unwrap(); checked+=1;
    let dx=pair[1].0[0]-pair[0].0[0]; let dy=pair[1].0[1]-pair[0].0[1];
    let heading=yaw(pair[0].1);
    assert!(dx*heading.cos()+dy*heading.sin()>=-1e-9,"reverse motion");
   }
   let mode=input["follow_reference"].as_bool().unwrap();
   if let Ok(output)=std::env::var("NEH_ON_OUTPUT") {
    use std::io::Write;
    let mut file=std::fs::OpenOptions::new().write(true).create_new(true).open(format!("{output}/path-{mode}.json")).unwrap();
    file.write_all(&serde_json::to_vec_pretty(&route.poses).unwrap()).unwrap();
   }
   eprintln!("ON_VALIDATION segments={checked}");
  }
  eprintln!("ON_RESULT follow={} ms={} result={:?}",input["follow_reference"],began.elapsed().as_secs_f64()*1000.0,result.as_ref().map(|r|r.poses.len()));
 }
}

struct ClearanceSubscriber;
impl tracing::Subscriber for ClearanceSubscriber {
 fn enabled(&self,_:&tracing::Metadata<'_>)->bool {true}
 fn new_span(&self,_:&tracing::span::Attributes<'_>)->tracing::span::Id {tracing::span::Id::from_u64(1)}
 fn record(&self,_:&tracing::span::Id,_:&tracing::span::Record<'_>) {}
 fn record_follows_from(&self,_:&tracing::span::Id,_:&tracing::span::Id) {}
 fn enter(&self,_:&tracing::span::Id) {}
 fn exit(&self,_:&tracing::span::Id) {}
 fn event(&self,e:&tracing::Event<'_>) {
  struct Fields(String);
  impl tracing::field::Visit for Fields {fn record_debug(&mut self,f:&tracing::field::Field,v:&dyn std::fmt::Debug){use std::fmt::Write;let _=write!(self.0," {}={:?}",f.name(),v);}}
  let mut fields=Fields(String::new());e.record(&mut fields);
  if fields.0.contains("机体与地面高度不兼容") || fields.0.contains("机体与占据单元相交") || fields.0.contains("fine_fringe") {eprintln!("ON_CLEARANCE{}",fields.0);}
 }
}
