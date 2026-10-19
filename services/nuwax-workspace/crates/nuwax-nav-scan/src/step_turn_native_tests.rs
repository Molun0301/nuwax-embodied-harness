//! 真实Writer/Receiver准入的原生台阶与固定实体，验证引导不再借PCT圆盘作机体许可。
use super::*;
use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
use nuwax_map_chunks::{ColumnGroundRecords, Domain, HeightfieldSurface, Receiver, Repository, Transfer, Writer};
use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceObservation, SurfaceObservations};
use nuwax_nav_types_messages::RobotGeometry;
use std::{collections::{BTreeMap,BTreeSet},sync::Arc};

const Q:f32=0.05;

fn scene()->SurfaceSnapshot {
    let mut writer=Writer::new(Repository::temporary().unwrap(),Q,4*1024*1024).unwrap();
    let mut points=Vec::new();
    for x in -20..21 {for y in -20..21 {
        let z=if x>=0 {3}else{0};
        let counts=BTreeMap::from([(z,[1000,0,0,0,0,0,0,0])]);
        writer.observe([[x,y,z]],20).unwrap();
        points.push(((x as f32+0.5)*Q,(y as f32+0.5)*Q,(z as f32+0.5)*Q));
        writer.replace_ground_column([x,y],&ColumnGroundRecords {classified_empty: false, legacy:vec![],heightfields:vec![
            HeightfieldSurface {column:HeightfieldColumn::from_counts([x,y],Q,counts).unwrap().unwrap(),
                samples:vec![]}]}).unwrap();
    }}
    let root=writer.snapshot(Domain {session:"scan-native-step-turn".into(),generation:1,
        frame_generation:0,producer_generation:1},1,20).unwrap();
    let mut receiver=Receiver::new(Repository::temporary().unwrap());
    receiver.receive(&Transfer {root:root.encode().unwrap(),included:BTreeMap::new()},&root.domain,
        |_,r|writer.repository().read_object(r.object,nuwax_map_chunks::MAX_BLOCK_BYTES)).unwrap();
    let mut observations=SurfaceObservations::new(Q,32).unwrap();
    let mut parts=Vec::new();
    for r in &root.blocks {
        let (block,records)=receiver.ground_parts(r).unwrap();
        observations.replace_block(r.key,block.iter().map(|(&local,e)|
            (nuwax_map_chunks::global_key(r.key,local).unwrap(),SurfaceObservation {
                observed_ns:e.observed_ns,hits:e.hits})).collect()).unwrap();
        parts.push((r.key,records));
    }
    for xy in parts.iter().map(|(key,_)|[key[0],key[1]]).collect::<BTreeSet<_>>() {
        let columns=parts.iter().filter(|(key,_)|key[..2]==xy).cloned().collect::<Vec<_>>();
        observations.replace_heightfield_columns(xy,&columns);
    }
    observations.mark_complete_geometry_source();
    // 现有Go2固定collision尺寸；全身外包仍含低处腿部范围，不从凸包猜躯干。
    let body=RobotGeometry::from_runtime_json(r#"{
        "footprint_xy_m":[[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]],
        "z_min_m":-0.448,"z_max_m":0.057,"source":"robot_default_urdf_collision",
        "self_collision":{"model_base_link":"base","excluded_moving_collision_count":24,
            "unsupported_collision_count":0,"primitives":[
            {"link_name":"base","shape":"box","translation_m":[0,0,0],
                "rotation_row_major":[1,0,0,0,1,0,0,0,1],"size_m":[0.3762,0.0935,0.114]},
            {"link_name":"Head_upper","shape":"cylinder","translation_m":[0.285,0,0.01],
                "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.05,"length_m":0.09},
            {"link_name":"Head_lower","shape":"sphere","translation_m":[0.293,0,-0.06],
                "rotation_row_major":[1,0,0,0,1,0,0,0,1],"radius_m":0.047}]}}
    "#).unwrap();
    let model=SurfaceBodyEnvelope {fixed_collision:body.self_collision,footprint_xy_m:body.footprint_xy_m,
        z_bounds_m:[body.z_min_m,body.z_max_m],body_to_support_z_m:0.4,max_step_up_m:0.21,max_step_down_m:0.21,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.2,surface_closing_radius:0.0,
        node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.2,wall_buffer_weight:2.0,
        max_slope_deg:18.0,step_threshold_m:0.21,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    SurfaceSnapshot::from_points(&points,config,Q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

fn heights(map:&SurfaceSnapshot)->[f64;2] {
    [-0.075,0.025].map(|x|map.query_ground([x,0.025],0.0).height.unwrap())
}

#[test]
fn native_step_turn_uses_bound_finite_body_and_still_requires_full_sweep() {
    let map=scene();let [low,high]=heights(&map);
    assert!((high-low-3.0*f64::from(Q)).abs()<1e-12);
    let reference=[[-0.075,0.025,low],[0.025,0.025,high],[0.025,0.475,high]];
    let start=([reference[0][0],reference[0][1],low+0.43],[0.0,0.0,0.0,1.0]);
    let query=map.body_query();let ground=query.start_ground_state(start).unwrap();
    let before=ground.clone();
    query.validate_motion_from_ground(start,start,&ground,&[],&mut None).unwrap();
    let radius=map.body_envelope().unwrap().radius();
    let tomo=crate::Tomogram::from_surface(&map).unwrap();
    let old=tomo.project_reference_motion(reference[0],[-0.075,0.025-radius],&[]).unwrap_err();
    assert!(old.contains("NativeContactGeometryUnavailable"),"固定旧圆盘路径的原生接触缺口: {old}");

    let guide=guide(&map,&query,&ground,start,&reference,&ScanConfig::default(),false)
        .unwrap().expect("同一原生源已有固定机体几何，应保留台阶转角候选");
    assert_eq!(ground,before,"引导分支不消费实际起点关联");
    assert_eq!(guide.len(),4);
    assert_eq!(guide[1],[reference[0][0],reference[0][1]-radius,low]);
    assert_eq!(guide[2],[reference[1][0]+radius,reference[1][1]-radius,high]);
    assert_eq!(guide[3][2],high);
    let samples=crate::spline::resample(&guide,f64::from(Q)*0.5).unwrap();
    let path=crate::project_bound(&map,&query,&ground,start,&samples,&crate::tests::motion(),false,
        None,None,&[],&std::cell::Cell::new(None)).unwrap();
    let mut checked=ground.clone();
    for edge in path.windows(2) {
        checked=query.validate_motion_from_ground(edge[0],edge[1],&checked,&[],&mut None).unwrap();
    }
    assert!((path.last().unwrap().0[2]-start.0[2]-(high-low)).abs()<1e-10,
        "实际高度偏差保留，只响应真实台阶h差");
    let boundary=[[-0.5,-0.5],[0.38,-0.5],[0.38,0.8],[-0.5,0.8]];
    assert!(crate::project_bound(&map,&query,&ground,start,&samples,&crate::tests::motion(),false,
        None,None,&boundary,&std::cell::Cell::new(None)).is_err(),"候选不能绕过后续完整边界扫掠");
}

#[test]
fn native_step_turn_height_uses_actual_tilted_support_column() {
    let map=scene();let [low,high]=heights(&map);
    let pitch=0.1_f64;
    let q=[0.0,(pitch*0.5).sin(),0.0,(pitch*0.5).cos()];
    let start=([0.025,0.025,low+0.4*pitch.cos()+0.03],q);
    let support=map.support_position_from_body(start.0,start.1,0.4).unwrap();
    assert!(support[0]<0.0 && start.0[0]>0.0);
    let query=map.body_query();let ground=query.start_ground_state(start).unwrap();
    assert_eq!(ground.height(),low);
    assert_eq!(map.query_ground([start.0[0],start.0[1]],low).height,Some(high));
    let reference=[[0.025,0.025,low],[0.125,0.025,high],[0.125,0.475,high]];
    let candidate=guide(&map,&query,&ground,start,&reference,&ScanConfig::default(),false)
        .unwrap().expect("真实支撑列和中心列不同的固定姿态");
    assert_eq!(candidate[0][2],low,"同XY起点必须保留真实倾斜姿态的低列关联");
    // 向入口移动同时改变航向，缺省normal不会固定实际俯仰偏差的世界朝向。
    // 此时支撑点的X位移消失，物理支撑转入高列，不能锁死初始低列。
    assert_eq!(candidate[1][2],high);
    assert_eq!(candidate[2][2],high);
}

#[test]
fn step_turn_unknown_height_carries_actual_start_instead_of_reference_z() {
    let map=scene();let pitch=0.1_f64;
    let start=([100.0,100.0,0.55],[0.0,(pitch*0.5).sin(),0.0,(pitch*0.5).cos()]);
    let query=map.body_query();let ground=query.start_ground_state(start).unwrap();
    assert_eq!(ground.query_state(),nuwax_nav_mls_planner::surface::GroundQueryState::Unknown);
    let reference=[[100.0,100.0,9.0],[100.1,100.0,9.15],[100.1,100.4,9.15]];
    let candidate=guide(&map,&query,&ground,start,&reference,&ScanConfig::default(),false)
        .unwrap().expect("未知空间不新增高度准入门");
    assert!(candidate.iter().all(|point|point[2]==ground.height()),
        "参考Z只触发候选形状，不能作为未知地面的高度或目标坡面");
    assert!((ground.height()-(start.0[2]-0.4*pitch.cos())).abs()<1e-12);
}
