//! 生产PCT与MLS密集起伏同源回归；名义参考与实际机体许可分别检查。
use super::*;
use std::sync::Arc;
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceMapBuilder,
    SurfaceError, SurfaceObservation, SurfaceObservations, SurfaceSnapshot};
use nuwax_nav_types_messages::GoalOptions;
use std::collections::{BTreeMap, BTreeSet};

type Body = ([f64; 3], [f64; 4]);
const FINE: f32 = 0.05;

#[derive(Clone, Copy, Debug)]
enum Terrain { Flat, Rough, RoughGentle, Ramp, Steps, Gap }

fn ground(kind: Terrain) -> BTreeSet<[i32; 3]> {
    let mut keys = BTreeSet::new();
    for x in -32_i32..32 { for y in -20_i32..20 {
        if matches!(kind, Terrain::Gap) && (-2..=1).contains(&x) && (-2..=1).contains(&y) { continue; }
        let top = match kind {
            Terrain::Flat | Terrain::Gap => -1,
            Terrain::Rough => (x.div_euclid(4) + y.div_euclid(4)).rem_euclid(3) - 1,
            Terrain::RoughGentle => (x.div_euclid(8) + y.div_euclid(8)).rem_euclid(2) - 1,
            Terrain::Ramp => x.div_euclid(5) - 1,
            Terrain::Steps => x.div_euclid(10) - 1,
        };
        keys.insert([x, y, top]);
    }}
    keys
}

fn envelope(whole_body: bool) -> SurfaceBodyEnvelope {
    SurfaceBodyEnvelope {
        footprint_xy_m: vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]],
        z_bounds_m: if whole_body {[-0.448,0.057]} else {[-0.15,0.15]},
        body_to_support_z_m: 0.4, max_step_up_m: 0.15, max_step_down_m: 0.15,
        max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
    }
}

fn build_map(keys: &BTreeSet<[i32; 3]>, whole_body: bool) -> SurfaceSnapshot {
    build_map_config(keys,whole_body,0.1,0.2)
}
fn build_map_config(keys: &BTreeSet<[i32; 3]>, whole_body: bool, planner:f32, tread:f32) -> SurfaceSnapshot {
    let fine = f64::from(FINE);
    let points: Vec<_> = keys.iter().map(|k| (((f64::from(k[0])+0.5)*fine) as f32,
        ((f64::from(k[1])+0.5)*fine) as f32, ((f64::from(k[2])+0.5)*fine) as f32)).collect();
    let mut blocks = BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for &key in keys { blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
        .insert(key, SurfaceObservation { observed_ns: 0, hits: 0 }); }
    let mut observations = SurfaceObservations::new(FINE,32).unwrap();
    for (key, values) in blocks { observations.replace_block(key,values).unwrap(); }
    observations.mark_complete_geometry_source();
    let config = Config { world_frame:"map".into(),voxel_size:planner,robot_height:0.1,
        surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
        wall_buffer_m:0.0,wall_buffer_weight:0.0,max_slope_deg:18.0,step_threshold_m:0.15,
        min_step_tread_depth_m:tread,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0 };
    SurfaceSnapshot::from_points(&points,config,FINE).unwrap().with_body_envelope(envelope(whole_body)).unwrap()
        .with_observations(Some(Arc::new(observations)))
}
fn body(x:f64,y:f64,z:f64) -> Body { ([x,y,z],[0.0,0.0,0.0,1.0]) }
fn advance(map:&SurfaceSnapshot, mut from:Body, end:f64) -> Result<Body,SurfaceError> {
    let start=from.0[0];
    map.validate_body_pose(from.0,from.1)?;
    for step in 1..=20 {
        let xy=[start+(end-start)*f64::from(step)/20.0,from.0[1]];
        let next=map.body_pose_after_motion(from,xy,0.0).map_err(|e|{eprintln!("ADVANCE failure step={step} from={from:?} xy={xy:?} error={e:?}");e})?;
        map.validate_body_motion(from,next,&[])?;from=next;
    }
    Ok(from)
}

#[test]
fn dense_treads_production_pct_reference_and_actual_continuation_are_checked_separately() {
    for resolution in [0.05,0.1] {
        let map=build_map_config(&ground(Terrain::Rough),true,resolution,0.2);
        let start=body(0.025,0.025,0.4);let goal=body(0.425,0.025,0.5);
        let mut options=GoalOptions::default();options.position_tolerance_m=0.005;
        let reference=PctPlanner::default().body_route(&map,start,goal,options,&[],None)
            .expect("生产PCT应能在完整20厘米踏面上给出名义参考");
        assert!(reference.len()>1,"不能由近目标提前返回冒充路线");
        assert_eq!(advance(&map,start,goal.0[0]).unwrap().0[0],goal.0[0]);
        // 不把名义参考高度当作拟合后的机身高度或已经进入目标三维球。
    }
    let keys:BTreeSet<_>=(-32_i32..32).flat_map(|x|(-20_i32..20).map(move|y|
        [x,y,(x.div_euclid(3)+y.div_euclid(3)).rem_euclid(3)-1])).collect();
    assert!(advance(&build_map(&keys,true),body(0.025,0.025,0.4),0.425).is_err(),
        "全局参考不能授予15厘米不足踏面的运动许可");
}

#[test]
fn dense_treads_fine_column_deletion_retires_pct_connectivity_even_when_coarse_graph_is_reused() {
    let complete=ground(Terrain::Rough);let mut missing=complete.clone();missing.retain(|key|key[0]!=4);
    let config=build_map(&complete,true).config().clone();
    let mut builder=SurfaceMapBuilder::configured(config,Some(envelope(true)));
    let planner=PctPlanner::default();
    let mut previous_source:Option<SurfaceSnapshot>=None;
    let mut previous_map:Option<Arc<Tomogram>>=None;
    let mut options=GoalOptions::default();options.position_tolerance_m=0.005;
    for (index,input) in [&complete,&missing,&complete].into_iter().enumerate() {
        let cold_source=build_map(input,true);
        let points:Vec<_>=input.iter().map(|k| {
            let p=k.map(|v|((f64::from(v)+0.5)*f64::from(FINE)) as f32);(p[0],p[1],p[2])
        }).collect();
        let built=builder.build(&points,FINE).unwrap();
        let source=built.snapshot.with_observations(cold_source.observations().cloned());
        let map=planner.map(&source).unwrap();
        if let Some(previous)=&previous_source {
            assert!(source.same_geometry(previous),"删除x=4细列不应伪造粗格高度变化");
            assert!(!Arc::ptr_eq(previous_map.as_ref().unwrap(),&map),"细源连通变化必须撤销旧PCT图");
        }
        let start=body(0.025,0.025,0.4);let goal=body(0.425,0.025,0.5);
        let route=planner.body_route(&source,start,goal,options,&[],None);
        assert_eq!(route.is_ok(),index!=1,"完整、缺列、恢复的路线必须跟随细源");
        let cold=PctPlanner::default().body_route(&cold_source,start,goal,options,&[],None);
        assert_eq!(route.is_ok(),cold.is_ok(),"已有拥有方和冷构建须保持相同可达性");
        previous_source=Some(source);previous_map=Some(map);
    }
}
