//! 对照原稀疏枚举与局部稠密枚举，比较真实输入、权重及拟合结果而非采样密度。
use super::*;
use std::collections::BTreeMap;

fn make(points:&[(f32,f32,f32)],footprint:Vec<[f64;2]>,bounds:[f64;2])->SurfaceSnapshot {
    let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,surface_closing_radius:0.0,
        node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.44,wall_buffer_weight:2.0,
        step_threshold_m:0.15,max_slope_deg:18.0,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
        goal_tolerance:0.2,viz_publish_hz:0.0};
    let model=SurfaceBodyEnvelope {footprint_xy_m:footprint,z_bounds_m:bounds,body_to_support_z_m:0.4,
        max_step_up_m:f64::from(0.15_f32),max_step_down_m:f64::from(0.15_f32),max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for &(x,y,z) in points {let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
        blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});}
    let mut observations=SurfaceObservations::new(0.05,32).unwrap();
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}observations.mark_complete_geometry_source();
    SurfaceSnapshot::from_points(points,config,0.05).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}
fn sort(points:&mut Vec<([f64;3],f64,f64)>) {points.sort_by(|a,b|a.0[0].total_cmp(&b.0[0]).then_with(||a.0[1].total_cmp(&b.0[1])));}
fn bits(points:&[([f64;3],f64,f64)])->Vec<[u64;5]> {
    points.iter().map(|&(p,low,w)|[p[0],p[1],p[2],low,w].map(f64::to_bits)).collect()
}

#[test]
fn dense_contact_domain_preserves_sparse_holes_layers_seed_and_directional_connection() {
    let mut points=vec![];
    for x in -14_i32..14 {for y in -14_i32..14 {
        if (x+y).rem_euclid(5)==0 || (x==1&&y<5) {continue;}
        let top=if x>5 {5}else{if y>4 {1}else{-1}};
        for z in -4..=top {points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));}
        if x< -4 {points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,0.625));}
    }}
    let map=make(&points,vec![[-0.2,-0.12],[0.3,-0.12],[0.3,0.12],[-0.2,0.12]],[-0.4,0.1]);
    for height in [-0.12,0.0,0.04,0.3,0.65,2.0] {for step in -10..=10 {
        let origin=[step as f64*0.019,step as f64*-0.013,height];let radius=0.39;
        let old=map.fine_contact_domain_sparse(origin,radius);let new=map.fine_contact_domain(origin,radius);
        // 私有输出不再保证 BFS 遍历顺序；实际消费者原有排序后的逐位输入契约不变。
        match (old,new) {
            (Ok(mut old),Ok(mut new))=> {
                sort(&mut old);sort(&mut new);
                assert_eq!(bits(&old),bits(&new),"种子、连通域、全部真实点及权重须相同：{origin:?}");
                assert_eq!(fit_contact_plane(&old,origin),fit_contact_plane(&new,origin));
            },
            (Err(old),Err(new))=>assert_eq!(old,new),
            (old,new)=>panic!("接触域结果不同：{origin:?}: {old:?} / {new:?}"),
        }
    }}
}

#[test]
#[ignore = "需要指定原始实录点云和具名接触域样本"]
fn recorded_contact_domain_and_fit_exact_differential_benchmark() {
    let input=std::env::var("NUWAX_CONTACT_BENCH_INPUT").expect("具名实录输入");
    let record:serde_json::Value=serde_json::from_slice(&std::fs::read(input).unwrap()).unwrap();
    let points:Vec<_>=std::fs::read(record["cloud"].as_str().unwrap()).unwrap().chunks_exact(16).map(|b|
        (f32::from_le_bytes(b[0..4].try_into().unwrap()),f32::from_le_bytes(b[4..8].try_into().unwrap()),f32::from_le_bytes(b[8..12].try_into().unwrap()))).collect();
    let footprint=serde_json::from_value(record["model"]["footprint_xy_m"].clone()).unwrap();
    let model_bounds=[record["model"]["z_min_m"].as_f64().unwrap(),record["model"]["z_max_m"].as_f64().unwrap()];
    let base=make(&points,footprint,model_bounds);let radius=base.body_envelope().unwrap().radius()+f64::from(0.05_f32);
    let origins:Vec<[f64;3]>=serde_json::from_value(record["origins"].clone()).unwrap();
    let old=base.with_observations(base.observations().cloned());let new=base.with_observations(base.observations().cloned());
    let started=std::time::Instant::now();let reference:Vec<_>=origins.iter().map(|&origin|old.fine_contact_domain_sparse(origin,radius)).collect();
    let old_ms=started.elapsed().as_secs_f64()*1000.;
    let started=std::time::Instant::now();let actual:Vec<_>=origins.iter().map(|&origin|new.fine_contact_domain(origin,radius)).collect();
    let new_ms=started.elapsed().as_secs_f64()*1000.;
    let mut expected_planes=Vec::new();let mut actual_planes=Vec::new();let mut failures=0;
    // 不再要求私有 BFS 发点顺序一致；只比较真实消费者排序后的完整输入。
    // 单独克隆排序，保留域构建计时和下方原有排序+拟合计时的含义。
    for (expected,actual) in reference.iter().zip(&actual) {
        match (expected,actual) {
            (Ok(expected),Ok(actual))=> {
                let mut expected=expected.clone();let mut actual=actual.clone();
                sort(&mut expected);sort(&mut actual);
                assert_eq!(bits(&expected),bits(&actual),"真实样本及权重必须逐位相同");
            },
            (Err(expected),Err(actual))=>assert_eq!(expected,actual),
            (expected,actual)=>panic!("接触域结果不同：{expected:?} / {actual:?}"),
        }
    }
    let started=std::time::Instant::now();
    for (origin,points) in origins.iter().zip(reference) {
        let plane=points.ok().and_then(|mut points| {sort(&mut points);fit_contact_plane(&points,*origin)});
        failures+=usize::from(plane.is_none());expected_planes.push(plane);
    }
    let fit_ms=started.elapsed().as_secs_f64()*1000.;
    for (origin,points) in origins.iter().zip(actual) {actual_planes.push(points.ok().and_then(|mut points| {sort(&mut points);fit_contact_plane(&points,*origin)}));}
    assert_eq!(expected_planes,actual_planes,"拟合迭代精度和结果不能改变");
    let result=serde_json::json!({"samples":origins.len(),"old_domain_ms":old_ms,"new_domain_ms":new_ms,"unchanged_fit_ms":fit_ms,
        "input_mismatches":0,"plane_mismatches":0,"missing_or_degenerate":failures,"radius_m":radius});
    if let Ok(path)=std::env::var("NUWAX_CONTACT_BENCH_OUTPUT") {std::fs::write(path,serde_json::to_vec_pretty(&result).unwrap()).unwrap();}
    eprintln!("{result}");
}
