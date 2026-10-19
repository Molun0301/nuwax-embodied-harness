//! 通用连续支撑契约的区分场景；合成输入不是九月实录或用户当前瓷砖现场。
//! 已知自体身份仅在夹具生成器中标注，不伪造生产观测的足端/材质字段。
use super::*;
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
        fixed_collision: None,
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
    build_map_source(keys,whole_body,planner,tread,FINE)
}
fn build_map_source(keys: &BTreeSet<[i32; 3]>, whole_body: bool, planner:f32, tread:f32, source:f32) -> SurfaceSnapshot {
    let fine = f64::from(source);
    let points: Vec<_> = keys.iter().map(|k| (((f64::from(k[0])+0.5)*fine) as f32,
        ((f64::from(k[1])+0.5)*fine) as f32, ((f64::from(k[2])+0.5)*fine) as f32)).collect();
    let mut blocks = BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for &key in keys { blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
        .insert(key, SurfaceObservation { observed_ns: 0, hits: 0 }); }
    let mut observations = SurfaceObservations::new(source,32).unwrap();
    for (key, values) in blocks { observations.replace_block(key,values).unwrap(); }
    observations.mark_complete_geometry_source();
    let config = Config { world_frame:"map".into(),voxel_size:planner,robot_height:0.1,
        surface_closing_radius:0.0,node_spacing_m:0.3,wall_clearance_m:0.0,
        wall_buffer_m:0.0,wall_buffer_weight:0.0,max_slope_deg:18.0,step_threshold_m:0.15,
        min_step_tread_depth_m:tread,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0 };
    SurfaceSnapshot::from_points(&points,config,source).unwrap().with_body_envelope(envelope(whole_body)).unwrap()
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
#[ignore = "阶段诊断：先固定场景和原始结果，不能把诊断返回成功当作全部场景可通行"]
fn terrain_contract_matrix_diagnostic() {
    let _logging=tracing::subscriber::set_default(DiagnosticSubscriber);
    for whole in [false,true] {
        for terrain in [Terrain::Flat,Terrain::Rough,Terrain::RoughGentle,Terrain::Ramp,Terrain::Steps,Terrain::Gap] {
            let map=build_map(&ground(terrain),whole);
            let from=body(0.025,0.025,0.4);
            eprintln!("CONTRACT terrain={terrain:?} whole={whole} pose={:?} advance={:?}",
                map.validate_body_pose(from.0,from.1),advance(&map,from,0.425));
            let seed_z = match terrain { Terrain::Ramp => -0.15, Terrain::Steps => -0.1, _ => 0.0 };
            let projected=map.body_pose_on_surface([-0.525,0.025],0.0,[-0.525,0.025,seed_z]);
            eprintln!("CONTRACT projected={projected:?} route={:?}",projected.clone().and_then(|p|advance(&map,p,0.525)));
        }
        for echo_layer in [0,1,2,4,6] {
            let mut keys=ground(Terrain::Flat);
            for x in [-5,4] {for y in [-3,2] {keys.insert([x,y,echo_layer]);}}
            let map=build_map(&keys,whole);let from=body(0.025,0.025,0.4);
            eprintln!("CONTRACT echo_layer={echo_layer} whole={whole} raw={:?} current={:?}",
                advance(&map,from,0.425),map.with_current_body_support(from.0,from.1).and_then(|m|advance(&m,from,0.425)));
        }
        let mut keys=ground(Terrain::Flat);
        for x in 8..10 {for y in -1..1 {for z in 5..9 {keys.insert([x,y,z]);}}}
        let map=build_map(&keys,whole);let from=body(0.025,0.025,0.4);let to=body(0.225,0.025,0.4);
        eprintln!("CONTRACT hard whole={whole} start={:?} end={:?} sweep={:?} current_sweep={:?}",
            map.validate_body_pose(from.0,from.1),map.validate_body_pose(to.0,to.1),map.validate_body_motion(from,to,&[]),
            map.with_current_body_support(from.0,from.1).and_then(|m|m.validate_body_motion(from,to,&[])));
        for upper in [8,19] {
            let mut keys=ground(Terrain::Flat);keys.extend((-32..32).flat_map(|x|(-20..20).map(move|y|[x,y,upper])));
            let map=build_map(&keys,whole);let lower=body(0.025,0.025,0.4);let higher=body(0.025,0.025,(upper+1)as f64*f64::from(FINE)+0.4);
            eprintln!("CONTRACT layer={upper} whole={whole} lower={:?} higher={:?} change_layer={:?}",
                map.validate_body_pose(lower.0,lower.1),map.validate_body_pose(higher.0,higher.1),map.validate_body_motion(lower,higher,&[]));
        }
    }
}

// 只用于本场景诊断，不增加生产日志、不引入新的日志依赖。
struct DiagnosticSubscriber;
impl tracing::Subscriber for DiagnosticSubscriber {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool { true }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id { tracing::span::Id::from_u64(1) }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct Fields(String);
        impl tracing::field::Visit for Fields {
            fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
                use std::fmt::Write;
                let _=write!(self.0," {}={:?}",field.name(),value);
            }
        }
        let mut fields=Fields(String::new());event.record(&mut fields);
        eprintln!("GEOMETRY{}",fields.0);
    }
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
}

#[test]
fn terrain_contract_flat_tile_and_tagged_echo_keep_the_original_source() {
    for planner in [0.05,0.1] {
    let clean=ground(Terrain::Flat);
    let echo_keys=[[-5,-3,2],[-5,2,2],[4,-3,2],[4,2,2]];
    let mut contaminated=clean.clone();contaminated.extend(echo_keys);
    let from=body(0.025,0.025,0.4);
    let raw=build_map_config(&contaminated,true,planner,0.2);
    // 有意修正旧 oracle：raw 必拒曾被用作“源未擦除”的代理，
    // 但此完整高带在既有名义接触范围内；来源保留必须直接检查。
    // echo_keys 仅是夹具标签，不进入生产接口，也不授予自体识别能力。
    raw.validate_body_pose(from.0,from.1).unwrap();
    let view=raw.with_current_body_support(from.0,from.1).unwrap();
    let clean_map=build_map_config(&clean,true,planner,0.2);
    let expected_end=advance(&clean_map,from,0.425).unwrap();
    assert_eq!(expected_end,body(0.425,0.025,0.4));
    for map in [&raw,&view] {
        let centre=map.locate_near_support([from.0[0],from.0[1],0.0],0.3).unwrap();
        assert_eq!(map.body_contact_frame(centre,0.0,0.0).unwrap(),(0.0,from.1),
            "高带不能替代原平地支撑参考或制造倾斜");
        assert_eq!(advance(map,from,0.425).unwrap(),expected_end);
        let observations=map.observations.as_ref().unwrap();
        // 包括四个具名点在内的每条原始观察均相同：生产数据没有 tag 字段。
        for &key in &contaminated {
            assert_eq!(observations.get(key),Some(SurfaceObservation{observed_ns:0,hits:0}),
                "原始观察和元数据不能被私有视图或运动查询擦除：{key:?}");
        }
        for key in echo_keys {
            assert_eq!(observations.column_levels(key[0],key[1]),Some(&[-1,2][..]),
                "保留原始两带及中间缺层，不把高带删除或填成连续占据");
            assert_eq!(&map.fine_contact_bands(key[0],key[1]).iter().map(|band|band.fit).collect::<Vec<_>>(),
                &vec![[0.0,0.0],[3.0*f64::from(FINE),3.0*f64::from(FINE)]],
                "接触许可不改写完整原始带的顶面");
        }
        assert_eq!(map.validate_body_pose(from.0,from.1),Ok(()),
            "派生与运动查询前后原始许可一致，不能通过持久化清理改变结果");
    }
    }
}

#[test]
fn terrain_contract_out_of_contact_band_preserves_source_through_private_overlay() {
    for planner in [0.05,0.1] {
    let mut keys=ground(Terrain::Flat);
    // 与低接触案例相同 XY；完整顶面约 0.30m，明确超过原 0.20m 接触范围，
    // 且仍在当前整体凸柱内，以固定输入实际覆盖原私有 overlay 路径。
    let upper_keys=[[-5,-3,5],[-5,2,5],[4,-3,5],[4,2,5]];
    keys.extend(upper_keys);
    let raw=build_map_config(&keys,true,planner,0.2);let from=body(0.025,0.025,0.4);
    assert_eq!(raw.validate_body_pose(from.0,from.1),Err(SurfaceError::InsufficientClearance));
    assert!(raw.with_current_body_support(from.0,from.1).is_err(),
        "current-pose queries must retain out-of-contact occupied volumes");
    let view=raw.clone();
    assert!(Arc::ptr_eq(&raw.body_query_cache,&view.body_query_cache));
    for key in upper_keys {assert_eq!(view.fine_contact_bands(key[0],key[1]).iter().map(|b|b.run).collect::<Vec<_>>(),vec![[-1,-1],[5,5]]);}
    assert_eq!(view.validate_body_pose(from.0,from.1),Err(SurfaceError::InsufficientClearance));
    for map in [&raw,&view] {
        let observations=map.observations.as_ref().unwrap();
        for &key in &keys {assert_eq!(observations.get(key),Some(SurfaceObservation{observed_ns:0,hits:0}));}
        for key in upper_keys {
            assert_eq!(observations.column_levels(key[0],key[1]),Some(&[-1,5][..]));
            assert_eq!(&map.fine_contact_bands(key[0],key[1]).iter().map(|band|band.fit).collect::<Vec<_>>(),
                &vec![[0.0,0.0],[6.0*f64::from(FINE),6.0*f64::from(FINE)]],
                "私有清除不能切割原始高带或把原始源改写为自由");
        }
    }
    assert_eq!(raw.validate_body_pose(from.0,from.1),Err(SurfaceError::InsufficientClearance),
        "真实执行私有 overlay 后，原图的超接触范围拒绝仍须保留");
    }
}

#[test]
fn terrain_contract_real_uneven_ground_is_not_forced_to_a_horizontal_plane() {
    for whole in [false,true] {
        let map=build_map(&ground(Terrain::RoughGentle),whole);
        let from=map.body_pose_on_surface([-0.525,0.025],0.0,[-0.525,0.025,0.0]).unwrap();
        let end=advance(&map,from,0.525).unwrap();
        assert!(from.1[0].abs()>0.01 && end.1[0].abs()>0.01,"真实起伏应保留地形姿态响应");
        assert_eq!(end.0[0],0.525);
    }
}

#[test]
fn terrain_contract_bounded_blind_gap_is_distinct_from_observed_pit_and_disconnected_strip() {
    for observed_pit in [false,true] {
        let mut keys=ground(Terrain::Gap);
        if observed_pit {keys.extend((-2..2).flat_map(|x|(-2..2).map(move|y|[x,y,-8])));}
        let map=build_map(&keys,true);
        let from=body(-0.525,0.025,0.4);map.validate_body_pose(from.0,from.1).unwrap();
        let above_gap=body(0.025,0.025,0.4);
        assert_eq!(map.validate_body_pose(above_gap.0,above_gap.1),
            if observed_pit {Err(SurfaceError::UntraversableSegment)} else {Ok(())},
            "known drop and missing ground remain different states");
        let projected=map.body_pose_after_motion(from,[above_gap.0[0],above_gap.0[1]],0.0);
        if observed_pit {
            assert!(advance(&map,from,0.525).is_err(),"已观测深坑不能按同层无回波盲区补平");
            assert!(projected.is_err(),"运动投影必须保留已知落差");
            assert!(map.with_current_body_support([0.025,0.025,0.4],from.1).is_err(),
                "已观测下方落差不能借当前足迹合成支撑填平");
        } else {
            let estimated=map.query_ground([above_gap.0[0],above_gap.0[1]],0.0);
            assert_eq!(estimated.state,GroundQueryState::Estimated,
                "四周同层连通的20厘米无回波洞可作有界地形估计，不能冒充实测");
            assert_eq!(estimated.height,Some(0.0));
            assert!(!estimated.sources.is_empty());
            assert_eq!(projected.unwrap(),above_gap);
            assert_eq!(advance(&map,from,0.525).unwrap(),body(0.525,0.025,0.4));
        }
    }
    let mut keys=ground(Terrain::Flat);
    keys.retain(|key|!(-2..=1).contains(&key[0]));
    let strip=build_map(&keys,true);
    let from=body(-0.525,0.025,0.4);let above_strip=body(0.025,0.025,0.4);
    assert_eq!(strip.validate_body_pose(above_strip.0,above_strip.1),Ok(()));
    let unknown=strip.query_ground([above_strip.0[0],above_strip.0[1]],0.0);
    assert_eq!(unknown.state,GroundQueryState::Unknown,"贯穿地图的缺测带隔开两侧地面，没有连通插值依据");
    assert_eq!(unknown.height,None);assert_eq!(unknown.normal,None);
    assert_eq!(strip.body_pose_after_motion(from,[above_strip.0[0],above_strip.0[1]],0.0).unwrap(),above_strip);
    advance(&strip,from,0.525).unwrap();
    assert_eq!(strip.query_ground([above_strip.0[0],above_strip.0[1]],0.0).state,GroundQueryState::Unknown);
}

#[test]
fn terrain_contract_external_trunk_obstacle_survives_current_body_view() {
    let mut keys=ground(Terrain::Flat);
    for x in 8..10 {for y in -1..1 {for z in 5..9 {keys.insert([x,y,z]);}}}
    for whole in [false,true] {
        let map=build_map(&keys,whole);let from=body(0.025,0.025,0.4);let to=body(0.225,0.025,0.4);
        map.validate_body_pose(from.0,from.1).unwrap();
        let view=map.with_current_body_support(from.0,from.1).unwrap();
        assert_eq!(view.validate_body_pose(to.0,to.1),Err(SurfaceError::InsufficientClearance));
        assert!(view.validate_body_motion(from,to,&[]).is_err());
    }
}

#[test]
fn terrain_contract_declared_ramp_and_stairs_keep_terrain_relative_motion() {
    for (kind,reference) in [(Terrain::Ramp,-0.15),(Terrain::Steps,-0.1)] {
        let map=build_map(&ground(kind),true);
        let from=map.body_pose_on_surface([-0.525,0.025],0.0,[-0.525,0.025,reference]).unwrap();
        let to=advance(&map,from,0.525).unwrap();
        assert!(to.0[2]>from.0[2]+0.05,"坡阶不是只能在平砖上工作的特例");
    }
}

#[test]
fn terrain_contract_layers_and_low_roof_remain_separate_checks() {
    for upper in [8,19] {
        let mut keys=ground(Terrain::Flat);keys.extend((-32..32).flat_map(|x|(-20..20).map(move|y|[x,y,upper])));
        let map=build_map(&keys,true);let lower=body(0.025,0.025,0.4);
        let higher=body(0.025,0.025,(upper+1)as f64*f64::from(FINE)+0.4);
        map.validate_body_pose(higher.0,higher.1).unwrap();
        if upper==19 {
            map.validate_body_pose(lower.0,lower.1).unwrap();
            advance(&map,lower,0.425).unwrap();advance(&map,higher,0.425).unwrap();
        } else {assert_eq!(map.validate_body_pose(lower.0,lower.1),Err(SurfaceError::InsufficientClearance));}
        assert!(map.validate_body_motion(lower,higher,&[]).is_err(),"分别可站立不等于层间存在接续");
    }
}

#[test]
fn terrain_contract_explicit_support_calibration_is_not_a_five_centimetre_posture_rule() {
    for height in [0.30,0.40,0.50] {
        // 只使用已知躯干外形和明确给定的支撑距离，不把静态伸腿凸柱冒充弯腿几何。
        let original=build_map(&ground(Terrain::Flat),false);
        let mut model=original.body_envelope().unwrap().clone();model.body_to_support_z_m=height;
        let map=original.with_body_envelope(model).unwrap();let from=body(0.025,0.025,height);
        let end=advance(&map,from,0.425).unwrap();
        assert_eq!(end.0[2],height,"显式标定的真实支撑距离可以不同，不需要重置为0.4米");
        assert_eq!(end.1,from.1);
    }
}

#[test]
#[ignore = "固定密集起伏的拒绝归因；表格保留失败，不授予新能力"]
fn dense_rough_rejection_attribution_diagnostic() {
    let map=build_map(&ground(Terrain::Rough),true);
    let from=([0.18499999999999997,0.025,0.39532616967339146],
        [0.016001586899234994,-0.011486490867544068,0.00018383775197955735,0.9998059691499831]);
    let to=([0.205,0.025,0.3950324446076362],
        [0.0208207767429179,-0.018617811932384785,0.00038778862668550933,0.9996097848433922]);
    let a=map.locate_support_from_body(from.0,from.1,0.4).unwrap();
    let b=map.locate_support_from_body(to.0,to.1,0.4).unwrap();
    let before=super::super::side_profile(&map.data.physical_graph.surface_lookup,a.cell,(-1,0),
        map.data.profile_cells,map.data.candidate_step_cells,&map.data.height_bounds,map.config().voxel_size);
    let after=super::super::side_profile(&map.data.physical_graph.surface_lookup,b.cell,(1,0),
        map.data.profile_cells,map.data.candidate_step_cells,&map.data.height_bounds,map.config().voxel_size);
    let dense_poses=(0..=200).try_for_each(|i| {let p=interpolated_body_pose(from,to,f64::from(i)/200.0);map.validate_body_pose(p.0,p.1)});
    let contact_events=map.validate_body_contact_events(from,to,|t|{let p=interpolated_body_pose(from,to,t);map.validate_body_pose(p.0,p.1)});
    eprintln!("ATTRIBUTION from={:?} to={:?} before_cells={before:?} after_cells={after:?} source_m={} planner_m={} tread_m={} profiles={}",
        a.cell,b.cell,map.source_voxel_size_m(),map.config().voxel_size,map.config().min_step_tread_depth_m,map.data.profile_cells);
    eprintln!("ATTRIBUTION endpoints={:?}/{:?} dense_poses={dense_poses:?} contact_events={contact_events:?} transition={:?} segment={:?} sweep={:?}",
        map.validate_body_pose(from.0,from.1),map.validate_body_pose(to.0,to.1),map.transition(a.cell,b.cell),
        map.validate_segment(a.support_xyz,b.support_xyz),map.validate_body_motion(from,to,&[]));
    eprintln!("ATTRIBUTION unchanged_coarse_global_plan={:?}",map.plan(a.support_xyz,b.support_xyz));
    for planner in [0.05,0.1] {for width in 3..=8 {for phase in [0,1] {
        let keys:BTreeSet<_>=(-32_i32..32).flat_map(|x|(-20_i32..20).map(move|y|
            [x,y,((x-phase).div_euclid(width)+y.div_euclid(width)).rem_euclid(3)-1])).collect();
        let map=build_map_config(&keys,true,planner,0.2);
        let origin=(f64::from(phase)+0.5)*f64::from(FINE);let from=body(origin,0.025,0.4);
        eprintln!("ATTRIBUTION planner_m={planner} tile_m={} phase_cells={phase} route={:?}",
            f64::from(width)*f64::from(FINE),advance(&map,from,origin+0.4));
    }}}
}

#[test]
fn dense_rough_observed_treads_connect_an_independently_valid_body_motion() {
    let map=build_map(&ground(Terrain::Rough),true);
    let from=([0.18499999999999997,0.025,0.39532616967339146],
        [0.016001586899234994,-0.011486490867544068,0.00018383775197955735,0.9998059691499831]);
    let to=([0.205,0.025,0.3950324446076362],
        [0.0208207767429179,-0.018617811932384785,0.00038778862668550933,0.9996097848433922]);
    map.validate_body_pose(from.0,from.1).unwrap();map.validate_body_pose(to.0,to.1).unwrap();
    map.validate_body_contact_events(from,to,|t|{let p=interpolated_body_pose(from,to,t);map.validate_body_pose(p.0,p.1)}).unwrap();
    for i in 0..=200 {let p=interpolated_body_pose(from,to,f64::from(i)/200.0);map.validate_body_pose(p.0,p.1).unwrap();}
    map.validate_body_motion(from,to,&[]).expect("真实20厘米踏面不能被多取一格的粗剖面拒绝");
    let next=map.body_pose_after_motion(from,[to.0[0],to.0[1]],0.0).unwrap();
    map.validate_body_motion(from,next,&[]).unwrap();
}

#[test]
fn dense_rough_tread_proof_keeps_depth_step_and_complete_source_limits() {
    let keys=ground(Terrain::Rough);let map=build_map(&keys,true);
    let from=([0.18499999999999997,0.025,0.39532616967339146],
        [0.016001586899234994,-0.011486490867544068,0.00018383775197955735,0.9998059691499831]);
    let to=([0.205,0.025,0.3950324446076362],
        [0.0208207767429179,-0.018617811932384785,0.00038778862668550933,0.9996097848433922]);
    assert!(map.known_tread_geometry_samples(from,to).is_some());
    let deeper=build_map_config(&keys,true,0.1,0.25);
    assert!(deeper.known_tread_geometry_samples(from,to).is_none(),"20厘米实测踏面不满足25厘米明确要求");
    assert_eq!(deeper.validate_body_motion(from,to,&[]),Err(SurfaceError::UntraversableSegment),
        "known 20cm terrain cannot satisfy the configured 25cm tread requirement");
    let mut model=map.body_envelope().unwrap().clone();model.max_step_up_m=0.025;
    let limited=map.with_body_envelope(model).unwrap();
    assert!(limited.known_tread_geometry_samples(from,to).is_none(),"5厘米上阶不满足2.5厘米能力");
    let mut model=map.body_envelope().unwrap().clone();model.max_step_down_m=0.025;
    let limited=map.with_body_envelope(model).unwrap();
    assert!(limited.known_tread_geometry_samples(to,from).is_none(),"下阶独立检查，不能沿上阶许可反用");
    let mut incomplete=map.observations.as_ref().unwrap().as_ref().clone();
    incomplete.replace_block([100,100,100],BTreeMap::new()).unwrap();
    assert!(map.with_observations(Some(Arc::new(incomplete))).known_tread_geometry_samples(from,to).is_none(),
        "部分更新不能继承完整来源事实");
    let mut missing=keys;missing.remove(&[4,0,0]);
    assert!(build_map(&missing,true).known_tread_geometry_samples(from,to).is_none(),"踏面实际缺测不能用邻列填充");
    let boundary=[[-1.0,-1.0],[0.1,-1.0],[0.1,1.0],[-1.0,1.0]];
    assert_eq!(map.validate_body_motion(from,to,&boundary),Err(SurfaceError::InvalidBoundary));
}

#[test]
fn dense_rough_continuation_uses_observed_width_across_grid_resolution_and_phase() {
    for planner in [0.05,0.1] {for width in [3,4] {for phase in [0,1] {
        let keys:BTreeSet<_>=(-32_i32..32).flat_map(|x|(-20_i32..20).map(move|y|
            [x,y,((x-phase).div_euclid(width)+y.div_euclid(width)).rem_euclid(3)-1])).collect();
        let map=build_map_config(&keys,true,planner,0.2);
        let origin=(f64::from(phase)+0.5)*f64::from(FINE);
        let result=advance(&map,body(origin,0.025,0.4),origin+0.4);
        if width==4 {
            assert_eq!(result.unwrap().0[0],origin+0.4,"满足20厘米真实踏面时不依赖粗格对齐");
        } else {
            assert!(matches!(result,Err(SurfaceError::UntraversableSegment|SurfaceError::MissingSupport)),
                "15厘米踏面不能膨胀成20厘米：{result:?}");
            let edge=f64::from(width+phase)*f64::from(FINE);
            let before=body(edge-0.5*f64::from(FINE),0.025,0.4);
            let after=body(edge+0.5*f64::from(FINE),0.025,0.4+f64::from(FINE));
            assert!(map.known_tread_geometry_samples(before,after).is_none(),
                "显式细踏面证明仍须拒绝15厘米宽度，不能仅靠前进错误类作代理");
        }
    }}}
}

#[test]
fn dense_rough_diagonal_cannot_split_a_step_through_a_third_height() {
    let map=build_map(&ground(Terrain::Rough),true);
    let from=body(0.185,0.185,0.4);let to=body(0.205,0.205,0.5);
    let mut model=map.body_envelope().unwrap().clone();model.max_step_up_m=0.075;
    let map=map.with_body_envelope(model).unwrap();
    assert!(map.known_tread_geometry_samples(from,to).is_none(),
        "10厘米对角阶差不能借两侧5厘米中间层绕过7.5厘米上阶能力");
}


// 以真实f32源格构造恰好满足深度的两侧踏面；不使用十进制f64替代输入。
fn exact_tread_source(source:f32,width:i32,tread:f32)->(SurfaceSnapshot,Body,Body) {
    // 一格高差并在粗格内部换阶，隔离踏深取整，不让悬空插值或两种网格边界抢先拒绝。
    let phase=(0.05_f32/source).round() as i32;
    let keys=(-width+phase..3*width+phase).flat_map(|x|(-2..3).map(move|y|
        [x,y,(x-phase).div_euclid(width).rem_euclid(2)-1])).collect();
    let map=build_map_source(&keys,false,0.1,tread,source);
    let size=f64::from(source);
    let from=body((f64::from(width+phase)-0.5)*size,0.5*size,0.4);
    let to=body((f64::from(width+phase)+0.5)*size,0.5*size,0.4+size);
    for t in map.body_support_grid_fractions(from,to,size).unwrap() {
        let p=interpolated_body_pose(from,to,t);
        map.locate_support_from_body(p.0,p.1,0.4).expect("计数夹具的每个实际参考须先具有既有支撑");
    }
    (map,from,to)
}

#[test]
fn dense_tread_review_exact_f32_depth_does_not_require_an_extra_cell() {
    for (source,width) in [(0.01_f32,20),(0.02,10),(0.05,4)] {
        let (map,from,to)=exact_tread_source(source,width,0.2);
        eprintln!("TREAD_BOUNDARY source={source:?} requested={:?} promoted_ratio={} f32_ratio={}",
            0.2_f32,f64::from(0.2_f32)/f64::from(source),0.2_f32/source);
        assert!(map.known_tread_geometry_samples(from,to).is_some(),
            "恰好20厘米踏面不能因f32提升后ceil被多算一格：source={source}");
        let (short,from,to)=exact_tread_source(source,width-1,0.2);
        assert!(short.known_tread_geometry_samples(from,to).is_none(),"真实少一格仍不满足踏深");
        let longer=f32::from_bits(0.2_f32.to_bits()+1);
        let (longer,from,to)=exact_tread_source(source,width,longer);
        assert!(longer.known_tread_geometry_samples(from,to).is_none(),"明确增加到下一个f32值仍要求更多支撑");
    }
}

#[test]
fn dense_tread_review_source_resolution_does_not_reinterpret_the_coarse_profile_limit() {
    for (source,width,tread) in [(0.001,200,0.2),(0.01,130,1.3)] {
        let (map,from,to)=exact_tread_source(source,width,tread);
        assert!(map.known_tread_geometry_samples(from,to).is_some(),
            "合法粗格踏深不能因细分超过128格而变为不许可：source={source} tread={tread}");
    }
}

#[test]
fn dense_tread_review_long_reference_does_not_multiply_all_samples_by_tread_depth() {
    let source=0.01_f32;
    let keys=(0..700).flat_map(|x|(-2..3).map(move|y| {
        let z=if x<305 {-1}else if x<325 {0}else if x<345 {-1}else{0};[x,y,z]
    })).collect();
    let map=build_map_source(&keys,false,0.1,0.2,source);
    let from=body(0.505,0.005,0.4);let to=body(6.505,0.005,0.41);
    let samples=map.body_support_grid_fractions(from,to,f64::from(source)).unwrap();
    assert!(samples.len()*20>16_384,"夹具必须覆盖旧乘积门槛");
    for &t in &samples {let p=interpolated_body_pose(from,to,t);
        map.locate_support_from_body(p.0,p.1,0.4).expect("长段夹具参考本身必须有支撑");}
    // 此处只验证支撑参考连接；完整足迹、碰撞与动作许可仍由外层核验。
    assert!(map.known_tread_geometry_samples(from,to).is_some(),
        "长段只有三个阶差，不应按每个采样点扫描踏深的假定拒绝");
}


#[test]
fn dense_tread_review_selects_the_actual_fine_top_inside_the_chosen_support_band() {
    let keys:BTreeSet<_>=(-40_i32..60).flat_map(|x|(-30_i32..30).flat_map(move|y| {
        let height=29+if y.rem_euclid(10)>=5 {1}else{0};
        [[x,y,height],[x,y,129]]
    })).collect();
    let map=build_map_source(&keys,false,0.1,0.2,0.01);
    let from=body(0.025,0.005,0.7);let to=body(0.035,0.005,0.7);
    let support=map.locate_support_from_body(from.0,from.1,0.4).unwrap();
    assert!((support.surface_xyz[2]-0.31).abs()<1e-6,
        "粗支撑顶必须来自旁边31厘米的细列，当前细列实际为30厘米");
    map.validate_body_pose(from.0,from.1).unwrap();map.validate_body_pose(to.0,to.1).unwrap();
    assert!(map.known_tread_geometry_samples(from,to).is_some(),
        "不能在当前XY查询旁边细列的31厘米顶而漏掉同一支撑带内的30厘米真顶");
    map.validate_body_motion(from,to,&[]).unwrap();
    let mut missing=keys.clone();missing.remove(&[2,0,29]);
    let missing=build_map_source(&missing,false,0.1,0.2,0.01);
    assert!(missing.known_tread_geometry_samples(from,to).is_none(),
        "选定低层缺测后不能改选该细列上方独立楼层");
    let upper=body(from.0[0],from.0[1],1.7);
    assert!(map.known_tread_geometry_samples(from,upper).is_none(),"独立楼层不能原地跨层连接");
}



#[test]
fn planar_known_terrain_is_lock_free_while_shared_body_cache_is_held() {
    let with_body=build_map(&ground(Terrain::Flat),false);
    let points:Vec<_>=ground(Terrain::Flat).iter().map(|k|
        ((k[0] as f32+0.5)*FINE,(k[1] as f32+0.5)*FINE,(k[2] as f32+0.5)*FINE)).collect();
    let coarse=SurfaceSnapshot::from_points(&points,with_body.config().clone(),FINE).unwrap();
    for source in [coarse.clone(),coarse.with_observations(with_body.observations().cloned())] {
        assert!(source.body_envelope().is_none());
        let from=[-0.5,0.025,0.0];let to=[0.5,0.025,0.0];
        let expected=source.validate_known_reference_motion(from,to);
        let reader=source.clone();
        assert!(source.same_geometry(&reader));
        assert!(source.same_body_query_view(&reader));
        assert!(Arc::ptr_eq(&source.body_query_cache,&reader.body_query_cache));
        let held=source.body_query_cache.lock().unwrap();
        let (send,receive)=std::sync::mpsc::channel();
        let worker=std::thread::spawn(move||send.send(reader.validate_known_reference_motion(from,to)).unwrap());
        let completed=receive.recv_timeout(std::time::Duration::from_secs(2));
        drop(held);worker.join().unwrap();
        assert_eq!(completed.unwrap(),expected,"known terrain must not wait on a planner's body cache");
    }
}

// APPEND-ONLY DRAFT for body_terrain_contract_tests.rs. Not compiled or run.
// Four new tests only. Existing fixtures/tests must remain byte-for-byte unchanged.
// These are internal bound-query tests, not a PCT/Scan/owner execution proof.
// `query_ground_with_native` is the production producer companion agreed for C0-a.
// Repetition below checks result/witness stability after source-only caches may warm;
// it does not claim C0-a association-sensitive result-cache hits.
use crate::surface::body_query::{
    BodyQueryReference, BoundBodyQueryView, GroundSample, LocalGroundReference,
    NativeGroundSource,
};

// This helper consumes the actual producer's typed output; it never guesses a
// source domain from the public GroundQuery.sources array or from actual body.z.
fn c0_terrain_sample(
    source: &SurfaceSnapshot, xy: [f64; 2], reference: f64, state: GroundQueryState,
) -> GroundSample {
    let (query, native) = source.query_ground_with_native(xy, reference);
    eprintln!("C0_TERRAIN_SOURCE xy={xy:?} reference={reference:?} query={query:?} native={native:?}");
    assert_eq!(query.state, state, "fixed source prerequisite failed; do not move input points");
    assert!(!native.is_empty(), "known/estimated source needs its actual typed producer provenance");
    assert_eq!(query.normal, Some([0.0, 0.0, 1.0]), "all four fixed fixtures have horizontal reference planes");
    let observations = source.observations.as_ref().unwrap();
    for item in &native {
        match item {
            NativeGroundSource::LegacyFineRun { xy, first, last } => {
                assert!(source.fine_contact_bands(xy[0], xy[1]).iter()
                    .any(|band| band.run == [*first, *last]));
                let layer = GroundLayer::from_voxel_run([*first, *last], f64::from(FINE),
                    source.ground_evidence).unwrap();
                assert_eq!(Some(layer.representative_z), query.height,
                    "horizontal fixture contributors must match the returned reference");
                for z in *first..=*last {
                    assert!(observations.get([xy[0], xy[1], z]).is_some());
                }
            }
            NativeGroundSource::ObservedFine { anchor } => {
                let record = observations.ground_layers([anchor[0], anchor[1]])
                    .find(|record| record.anchor == *anchor).expect("native anchor is in this source");
                assert!(!record.members.is_empty());
                assert_eq!(record.height_at(xy), query.height);
                for &z in &record.members {
                    assert!(observations.get([anchor[0], anchor[1], z]).is_some());
                }
            }
            NativeGroundSource::HeightfieldFine { key } => {
                panic!("legacy fixture unexpectedly selected a heightfield: {key:?}");
            }
            NativeGroundSource::CoarseCell(cell) => {
                panic!("complete fine fixtures must not silently substitute coarse provenance: {cell:?}");
            }
        }
    }
    GroundSample {mode:crate::surface::body_query::GroundSampleMode::Compatibility,
        reference_origin: [xy[0], xy[1], query.height.expect("known source height")],
        reference_normal: query.normal,
        query_state: query.state,
        reference_sources: native,
    }
}

fn c0_terrain_bound(source: &SurfaceSnapshot, sample: GroundSample) -> BoundBodyQueryView {
    source.bind_body_query(BodyQueryReference::Ground(Arc::new(LocalGroundReference {
        first: sample, changes: vec![],
    })))
}

#[derive(Debug, PartialEq)]
struct C0TerrainProbe {
    poses: [Result<(), SurfaceError>; 2],
    pose_witnesses: [Option<BodyFailureWitness>; 2],
    projection: Result<Body, SurfaceError>,
    projection_witness: Option<BodyFailureWitness>,
    motion: Result<(), SurfaceError>,
    motion_witness: Option<BodyFailureWitness>,
}

fn c0_terrain_probe(
    label: &str, view: &BoundBodyQueryView, from: Body, to: Body,
    yaw: f64, boundary: &[[f64; 2]],
) -> C0TerrainProbe {
    let mut pose_witnesses = [None, None];
    let poses = [view.pose_at(from, 0.0, &mut pose_witnesses[0]),
        view.pose_at(to, 1.0, &mut pose_witnesses[1])];
    let mut projection_witness = None;
    let projection = view.project_from_to_samples(from, [to.0[0], to.0[1]], yaw,
        [0.0, 1.0], &mut projection_witness);
    let mut motion_witness = None;
    let motion = view.motion_on_segment(from, to, [0.0, 1.0], boundary, &mut motion_witness);
    let result = C0TerrainProbe { poses, pose_witnesses, projection,
        projection_witness, motion, motion_witness };
    eprintln!("C0_TERRAIN_PROBE {label} from={from:?} to={to:?} result={result:?}");
    result
}

fn c0_terrain_assert_clear(probe: &C0TerrainProbe, expected: Body) {
    assert_eq!(probe.poses, [Ok(()), Ok(())]);
    assert_eq!(probe.pose_witnesses, [None, None]);
    assert_eq!(probe.projection, Ok(expected));
    assert_eq!(probe.projection_witness, None);
    assert_eq!(probe.motion, Ok(()));
    assert_eq!(probe.motion_witness, None);
}

fn c0_terrain_assert_fine_witness(
    source: &SurfaceSnapshot, witness: Option<BodyFailureWitness>, occupied: &BTreeSet<[i32; 3]>,
) {
    let witness = witness.expect("actual fine occupancy rejection must retain its witness");
    let BodyRejection::Occupancy { cell, source: kind, .. } = witness.rejection;
    assert!(matches!(kind, OccupancySource::Fine | OccupancySource::FineFringe), "{witness:?}");
    assert!(occupied.contains(&cell), "unexpected source member: {witness:?}");
    assert!(source.observations.as_ref().unwrap().get(cell).is_some());
    let size = f64::from(FINE);
    let prism = PrismQuery::new(source.body_envelope().unwrap(),
        rotation(witness.pose.1).unwrap(), witness.pose.0);
    assert!(prism.intersects(cell.map(|v| f64::from(v) * size),
        cell.map(|v| f64::from(v + 1) * size)), "witness must intersect this actual prism");
}

#[test]
fn ground_context_whole_body_floor_and_noncontact_occupancy_pair() {
    let from = body(0.025, 0.025, 0.4);
    let to = body(0.425, 0.025, 0.4);
    let high_keys = [[-5, -3, 5], [-5, 2, 5], [4, -3, 5], [4, 2, 5]];
    let low_keys = [[-5, -3, 2], [-5, 2, 2], [4, -3, 2], [4, 2, 2]];
    let size = f64::from(FINE);
    assert!(from.0[2] + envelope(true).z_bounds_m[0] < 0.0);
    assert!(from.0[2] + envelope(true).z_bounds_m[0] > -size);
    for planner in [0.05, 0.1] {
        for obstructed in [false, true] {
            let mut keys = ground(Terrain::Flat);
            if obstructed { keys.extend(high_keys); }
            let source = build_map_config(&keys, true, planner, 0.2);
            let sample = c0_terrain_sample(&source, [0.025, 0.025], 0.0,
                GroundQueryState::LegacyGeometry);
            assert_eq!(sample.reference_origin[2], 0.0);
            let grounded = c0_terrain_bound(&source, sample);
            let unanchored = source.bind_body_query(BodyQueryReference::Unanchored);
            // Both direction orders keep the same actual endpoints and source.
            let mut expected_ground = None;
            let mut expected_unanchored = None;
            for (mode, view, expected) in [
                ("Ground", &grounded, &mut expected_ground),
                ("Unanchored", &unanchored, &mut expected_unanchored),
            ] {
                let probe = c0_terrain_probe(mode, view, from, to, 0.0, &[]);
                if !obstructed { c0_terrain_assert_clear(&probe, to); }
                else {
                    assert_eq!(probe.poses, [Err(SurfaceError::InsufficientClearance),
                        Err(SurfaceError::InsufficientClearance)]);
                    assert_eq!(probe.projection, Err(SurfaceError::InsufficientClearance));
                    assert_eq!(probe.motion, Err(SurfaceError::InsufficientClearance));
                    let occupied = high_keys.into_iter().collect();
                    for witness in probe.pose_witnesses.into_iter()
                        .chain([probe.projection_witness, probe.motion_witness]) {
                        c0_terrain_assert_fine_witness(&source, witness, &occupied);
                    }
                }
                *expected = Some(probe);
            }
            for (mode, view, expected) in [
                ("Unanchored repeat", &unanchored, expected_unanchored.as_ref().unwrap()),
                ("Ground repeat", &grounded, expected_ground.as_ref().unwrap()),
                ("Unanchored alternate", &unanchored, expected_unanchored.as_ref().unwrap()),
            ] {
                assert_eq!(&c0_terrain_probe(mode, view, from, to, 0.0, &[]), expected);
            }
            for &key in &keys {
                assert_eq!(source.observations.as_ref().unwrap().get(key),
                    Some(SurfaceObservation { observed_ns: 0, hits: 0 }));
            }
            if obstructed {
                for key in high_keys {
                    assert_eq!(source.observations.as_ref().unwrap().column_levels(key[0], key[1]),
                        Some(&[-1, 5][..]));
                    assert_eq!(source.fine_contact_bands(key[0], key[1]).iter()
                        .map(|band| (band.run, band.fit)).collect::<Vec<_>>(),
                        vec![([-1, -1], [0.0, 0.0]), ([5, 5], [6.0 * size, 6.0 * size])]);
                }
            }
        }
        // C01-c explicitly keeps the old LegacyNominal compatibility oracle.
        // These fixture-only labels do not assign production self identity.
        let mut keys = ground(Terrain::Flat); keys.extend(low_keys);
        let source = build_map_config(&keys, true, planner, 0.2);
        assert_eq!(advance(&source, from, to.0[0]).unwrap(), to);
        for key in low_keys {
            assert_eq!(source.observations.as_ref().unwrap().column_levels(key[0], key[1]),
                Some(&[-1, 2][..]));
            assert_eq!(source.fine_contact_bands(key[0], key[1]).iter()
                .map(|band| band.fit).collect::<Vec<_>>(),
                vec![[0.0, 0.0], [3.0 * size, 3.0 * size]]);
        }
    }
}

// Exact F1 lower-only data/settings from layered_map(&[[0,0]]) plus
// with_measured_floor(0.5 * f64::from(FINE)); copied test-local because those
// existing helpers are private in the separate body::tests module. No new physics.
fn c0_terrain_measured_lower_only() -> SurfaceSnapshot {
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, block_key, local_key};
    let keys: Vec<_> = (-12..12).flat_map(|x| (-12..12).map(move |y| [x, y, 0])).collect();
    let points: Vec<_> = keys.iter().map(|k| ((k[0] as f32 + 0.5) * FINE,
        (k[1] as f32 + 0.5) * FINE, (k[2] as f32 + 0.5) * FINE)).collect();
    let config = Config { voxel_size: 0.1, robot_height: 0.25, surface_closing_radius: 0.0,
        wall_clearance_m: 0.0, step_threshold_m: 0.21, min_step_tread_depth_m: 0.2,
        world_frame: "map".into(), node_spacing_m: 0.3, wall_buffer_m: 0.0,
        wall_buffer_weight: 0.0, max_slope_deg: 25.0, step_penalty_weight: 1.0,
        goal_tolerance: 0.1, viz_publish_hz: 0.0 };
    let model = SurfaceBodyEnvelope {
        fixed_collision: None,
        footprint_xy_m: vec![[-0.55, -0.14], [0.55, -0.14], [0.55, 0.14], [-0.55, 0.14]],
        z_bounds_m: [-0.15, 0.15], body_to_support_z_m: 0.4,
        max_step_up_m: 0.2, max_step_down_m: 0.1, max_cross_slope_deg: 10.0,
        support_uncertainty_m: 0.0,
    };
    let mut occupancy = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for &key in &keys { occupancy.entry(block_key(key)).or_default().insert(key,
        SurfaceObservation { observed_ns: 1000, hits: 1 }); }
    let mut observations = SurfaceObservations::new(FINE, 32).unwrap();
    for (block, values) in occupancy { observations.replace_block(block, values).unwrap(); }
    let size = f64::from(FINE); let height = 0.5 * size;
    let mut blocks = BTreeMap::<[i32; 3], LayerObservations>::new();
    for [x, y, z] in keys {
        let key = [x, y, z]; let local = local_key(key);
        let origin = [(f64::from(x) + 0.5) * size, (f64::from(y) + 0.5) * size, height];
        blocks.entry(block_key(key)).or_default().insert(local, LayerObservation {
            members: vec![local], normal: [0.0, 0.0, 1.0], plane_origin: origin,
            samples: (0..3).map(|i| LayerSample { source: Default::default(), point: origin,
                source_sequence: i + 1, source_timestamp_ns: 100 + i, point_index: u32::from(local) }).collect(),
        });
    }
    for (block, layers) in blocks { observations.replace_ground_layers(block, Arc::new(layers)).unwrap(); }
    observations.mark_complete_geometry_source();
    SurfaceSnapshot::from_points(&points, config, FINE).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

#[test]
fn ground_context_fixed_ground_varies_body_height_and_roof_collision() {
    let size = f64::from(FINE);
    for planner in [0.05, 0.1] {
        for roof in [false, true] {
            let mut keys = ground(Terrain::Flat);
            let roof_keys: BTreeSet<_> = (-8..16).flat_map(|x| (-8..8).map(move |y| [x, y, 13])).collect();
            assert_eq!(roof_keys.len(), 24 * 16);
            if roof { keys.extend(roof_keys.iter().copied()); }
            let source = build_map_config(&keys, false, planner, 0.2);
            assert_eq!(source.body_envelope().unwrap().body_to_support_z_m, 0.4);
            let sample = c0_terrain_sample(&source, [0.025, 0.025], 0.0,
                GroundQueryState::LegacyGeometry);
            assert_eq!(sample.reference_origin[2], 0.0);
            let grounded = c0_terrain_bound(&source, sample);
            let unanchored = source.bind_body_query(BodyQueryReference::Unanchored);
            for z in [0.25, 0.4, 0.65] {
                let from = body(0.025, 0.025, z); let to = body(0.425, 0.025, z);
                assert!(z - 0.15 > 0.0, "all three fixed torso envelopes clear the floor");
                let collides = roof && z == 0.65;
                assert_eq!(roof && z + 0.15 >= 13.0 * size && z - 0.15 <= 14.0 * size, collides);
                for (mode, view) in [("Ground", &grounded), ("Unanchored", &unanchored)] {
                    let probe = c0_terrain_probe(mode, view, from, to, 0.0, &[]);
                    if collides {
                        assert_eq!(probe.poses, [Err(SurfaceError::InsufficientClearance),
                            Err(SurfaceError::InsufficientClearance)]);
                        assert_eq!(probe.projection, Err(SurfaceError::InsufficientClearance));
                        assert_eq!(probe.motion, Err(SurfaceError::InsufficientClearance));
                        for witness in probe.pose_witnesses.into_iter()
                            .chain([probe.projection_witness, probe.motion_witness]) {
                            c0_terrain_assert_fine_witness(&source, witness, &roof_keys);
                        }
                    } else { c0_terrain_assert_clear(&probe, to); }
                    assert_eq!(c0_terrain_probe("repeat fixed height", view, from, to, 0.0, &[]), probe);
                }
                let fixed = source.query_ground([0.025, 0.025], 0.0);
                assert_eq!(fixed.height, Some(0.0));
                assert_eq!(fixed.state, GroundQueryState::LegacyGeometry);
                assert_eq!(source.observations.as_ref().unwrap().column_levels(0, 0),
                    if roof { Some(&[-1, 13][..]) } else { Some(&[-1][..]) });
            }
        }
    }
    // C03-c records the fixed whole-body model's mechanical interpretation.
    // It cannot be presented as known retractable-leg or torso geometry.
    let whole = build_map(&ground(Terrain::Flat), true);
    let grounded = c0_terrain_bound(&whole, c0_terrain_sample(&whole, [0.025, 0.025], 0.0,
        GroundQueryState::LegacyGeometry));
    let floor = ground(Terrain::Flat);
    for z in [0.25, 0.4, 0.65] {
        let from = body(0.025, 0.025, z); let to = body(0.425, 0.025, z);
        let probe = c0_terrain_probe("fixed whole envelope diagnostic", &grounded, from, to, 0.0, &[]);
        for (result, witness) in probe.poses.iter().zip(probe.pose_witnesses)
            .chain(std::iter::once((&probe.motion, probe.motion_witness))) {
            match result {
                Ok(()) => assert_eq!(witness, None),
                Err(SurfaceError::InsufficientClearance) => c0_terrain_assert_fine_witness(&whole, witness, &floor),
                other => panic!("fixed whole envelope must not reintroduce nominal navigation veto: {other:?}"),
            }
        }
        match &probe.projection {
            Ok(projected) => { assert_eq!(*projected, to); assert_eq!(probe.projection_witness, None); }
            Err(SurfaceError::InsufficientClearance) => c0_terrain_assert_fine_witness(&whole, probe.projection_witness, &floor),
            other => panic!("mechanical diagnostic has an unexpected projection result: {other:?}"),
        }
    }
    // C03-d: the same measured lower members remain ground, but membership alone
    // must not waive an actual submerged torso through those members.
    let source = c0_terrain_measured_lower_only();
    let sample = c0_terrain_sample(&source, [0.025, 0.025], 0.5 * size, GroundQueryState::Observed);
    assert_eq!(sample.reference_origin[2], 0.5 * size);
    let observed = source.observations.as_ref().unwrap();
    let record = observed.ground_layers([0, 0]).next().unwrap();
    assert_eq!(record.members, vec![0]); assert_eq!(record.plane_origin[2], 0.5 * size);
    let grounded = c0_terrain_bound(&source, sample);
    let normal = body(0.025, 0.025, 0.4); let submerged = body(0.025, 0.025, -0.1);
    let floor: BTreeSet<_> = (-12..12).flat_map(|x| (-12..12).map(move |y| [x, y, 0])).collect();
    assert!(submerged.0[2] - 0.15 < 0.0 && submerged.0[2] + 0.15 > 0.0);
    assert!(normal.0[2] - 0.15 > size);
    for pose in [normal, submerged, normal, submerged] {
        let probe = c0_terrain_probe("measured torso penetration pair", &grounded, pose, pose, 0.0, &[]);
        if pose == normal { c0_terrain_assert_clear(&probe, normal); }
        else {
            assert_eq!(probe.poses, [Err(SurfaceError::InsufficientClearance), Err(SurfaceError::InsufficientClearance)]);
            assert_eq!(probe.projection, Err(SurfaceError::InsufficientClearance));
            assert_eq!(probe.motion, Err(SurfaceError::InsufficientClearance));
            for witness in probe.pose_witnesses.into_iter().chain([probe.projection_witness, probe.motion_witness]) {
                c0_terrain_assert_fine_witness(&source, witness, &floor);
            }
        }
        assert_eq!(source.query_ground([0.025, 0.025], 0.5 * size).height, Some(0.5 * size));
        assert_eq!(observed.ground_layers([0, 0]).next().unwrap().members, vec![0]);
    }
}

#[test]
fn ground_context_observed_drop_missing_and_unanchored_pair() {
    let size = f64::from(FINE);
    let from = body(-0.525, 0.025, 0.4); let above = body(0.025, 0.025, 0.4);
    let end = body(0.525, 0.025, 0.4); let short = body(0.075, 0.025, 0.4);
    let low_keys: BTreeSet<_> = (-2..2).flat_map(|x| (-2..2).map(move |y| [x, y, -9])).collect();
    let mut pit_keys = ground(Terrain::Gap); pit_keys.extend(low_keys.iter().copied());
    let pit = build_map(&pit_keys, true);
    let lower = pit.query_ground([above.0[0], above.0[1]], 0.0);
    assert_eq!(lower.state, GroundQueryState::LegacyGeometry);
    assert_eq!(lower.height, Some(-8.0 * size));
    assert!(8.0 * size > envelope(true).max_step_down_m + size + 1e-6);
    // The independent floor0 reference is produced on THIS pit source at the
    // real left endpoint; it is not fabricated on the isolated low-only source.
    let sample = c0_terrain_sample(&pit, [from.0[0], from.0[1]], 0.0, GroundQueryState::LegacyGeometry);
    assert_eq!(sample.reference_origin[2], 0.0);
    let grounded = c0_terrain_bound(&pit, sample);
    let unanchored = pit.bind_body_query(BodyQueryReference::Unanchored);
    let across = c0_terrain_probe("known drop across endpoints", &grounded, from, end, 0.0, &[]);
    assert_eq!(across.poses, [Ok(()), Ok(())]);
    assert_eq!(across.pose_witnesses, [None, None]);
    assert_eq!(across.projection, Err(SurfaceError::UntraversableSegment));
    assert_eq!(across.motion, Err(SurfaceError::UntraversableSegment));
    assert_eq!(across.projection_witness, None); assert_eq!(across.motion_witness, None);
    let entering = c0_terrain_probe("known drop at center", &grounded, from, above, 0.0, &[]);
    assert_eq!(entering.poses, [Ok(()), Err(SurfaceError::UntraversableSegment)]);
    assert_eq!(entering.projection, Err(SurfaceError::UntraversableSegment));
    assert_eq!(entering.motion, Err(SurfaceError::UntraversableSegment));
    assert_eq!(entering.pose_witnesses, [None, None]);
    assert_eq!(entering.projection_witness, None); assert_eq!(entering.motion_witness, None);
    // C05-b: identical actual endpoints and source, opposite navigation contexts.
    let grounded_pit = c0_terrain_probe("anchored pit center", &grounded, above, short, 0.0, &[]);
    assert_eq!(grounded_pit.poses, [Err(SurfaceError::UntraversableSegment), Err(SurfaceError::UntraversableSegment)]);
    assert_eq!(grounded_pit.projection, Err(SurfaceError::UntraversableSegment));
    assert_eq!(grounded_pit.motion, Err(SurfaceError::UntraversableSegment));
    assert_eq!(grounded_pit.pose_witnesses, [None, None]);
    assert_eq!(grounded_pit.projection_witness, None); assert_eq!(grounded_pit.motion_witness, None);
    let free_pit = c0_terrain_probe("unanchored pit center", &unanchored, above, short, 0.0, &[]);
    c0_terrain_assert_clear(&free_pit, short);
    for (label, view, expected) in [
        ("Ground repeat", &grounded, &grounded_pit), ("Unanchored repeat", &unanchored, &free_pit),
        ("Unanchored alternate", &unanchored, &free_pit), ("Ground alternate", &grounded, &grounded_pit),
    ] { assert_eq!(&c0_terrain_probe(label, view, above, short, 0.0, &[]), expected); }
    for &key in &pit_keys { assert_eq!(pit.observations.as_ref().unwrap().get(key),
        Some(SurfaceObservation { observed_ns: 0, hits: 0 })); }

    // C04-b: delete only the pit returns. Keep the original bounded estimate,
    // including its production-native contributor domains, rather than Observed.
    let gap = build_map(&ground(Terrain::Gap), true);
    let sample = c0_terrain_sample(&gap, [above.0[0], above.0[1]], 0.0, GroundQueryState::Estimated);
    assert_eq!(sample.reference_origin[2], 0.0);
    let gap_view = c0_terrain_bound(&gap, sample);
    c0_terrain_assert_clear(&c0_terrain_probe("bounded missing gap", &gap_view, from, above, 0.0, &[]), above);
    c0_terrain_assert_clear(&c0_terrain_probe("bounded missing crossing", &gap_view, from, end, 0.0, &[]), end);
    assert_eq!(gap.query_ground([above.0[0], above.0[1]], 0.0).state, GroundQueryState::Estimated);
    assert!(gap.observations.as_ref().unwrap().column_levels(0, 0).is_none());

    // C04-c: Unknown current strip query, with a genuinely carried reference from
    // the same source's known left side. Unknown itself is not a new obstacle.
    let mut keys = ground(Terrain::Flat); keys.retain(|key| !(-2..=1).contains(&key[0]));
    let strip = build_map(&keys, true);
    let (unknown, native) = strip.query_ground_with_native([above.0[0], above.0[1]], 0.0);
    assert_eq!(unknown.state, GroundQueryState::Unknown); assert_eq!(unknown.height, None);
    assert_eq!(unknown.normal, None); assert!(unknown.sources.is_empty()); assert!(native.is_empty());
    let mut carried = c0_terrain_sample(&strip, [from.0[0], from.0[1]], 0.0, GroundQueryState::LegacyGeometry);
    assert_eq!(carried.reference_origin[2], 0.0);
    carried.query_state = GroundQueryState::Unknown;
    let strip_view = c0_terrain_bound(&strip, carried);
    c0_terrain_assert_clear(&c0_terrain_probe("Unknown carried strip", &strip_view, from, above, 0.0, &[]), above);
    c0_terrain_assert_clear(&c0_terrain_probe("Unknown carried crossing", &strip_view, from, end, 0.0, &[]), end);
    let after = strip.query_ground([above.0[0], above.0[1]], 0.0);
    assert_eq!(after.state, GroundQueryState::Unknown); assert_eq!(after.height, None); assert_eq!(after.normal, None);

    // C05-a/c: the actual low occupancy is below either unchanged envelope.
    // No Ground(0) is ever constructed from this isolated source or empty pair.
    for whole in [false, true] {
        let isolated = build_map(&low_keys, whole);
        let low = isolated.query_ground([above.0[0], above.0[1]], 0.0);
        assert_eq!(low.height, Some(-8.0 * size));
        assert_eq!(low.state, GroundQueryState::LegacyGeometry);
        assert!(above.0[2] + isolated.body_envelope().unwrap().z_bounds_m[0] > -8.0 * size);
        let view = isolated.bind_body_query(BodyQueryReference::Unanchored);
        let first = c0_terrain_probe("unanchored isolated low face", &view, above, short, 0.0, &[]);
        c0_terrain_assert_clear(&first, short);
        assert_eq!(c0_terrain_probe("unanchored isolated repeat", &view, above, short, 0.0, &[]), first);
        // from_points([]) is InvalidPointCloud; use the existing complete-empty
        // root builder, which expressly represents an empty unknown map.
        let mut builder = crate::surface::SurfaceMapBuilder::configured(isolated.config().clone(),
            Some(isolated.body_envelope().unwrap().clone()));
        let empty = builder.build_column_updates(&[], FINE, true, 0).unwrap().snapshot;
        let mut empty_observations = SurfaceObservations::new(FINE, 32).unwrap();
        empty_observations.mark_complete_geometry_source();
        let empty = empty.with_observations(Some(Arc::new(empty_observations)));
        assert!(empty.data.occupied_columns.is_empty());
        let (unknown, native) = empty.query_ground_with_native([above.0[0], above.0[1]], 0.0);
        assert_eq!(unknown.state, GroundQueryState::Unknown); assert_eq!(unknown.height, None);
        assert_eq!(unknown.normal, None); assert!(unknown.sources.is_empty()); assert!(native.is_empty());
        let view = empty.bind_body_query(BodyQueryReference::Unanchored);
        c0_terrain_assert_clear(&c0_terrain_probe("unanchored empty source", &view, above, short, 0.0, &[]), short);
    }
    // The bound API returns physical outcomes only. These checks prove no
    // nominal drop/delta is synthesized; they do not claim an owner-level
    // navigation-ground=None output field that this C0-a interface does not have.
}

#[test]
fn ground_context_origin_boundary_and_physical_forbidden_pair() {
    let mut model = envelope(false);
    model.footprint_xy_m = vec![[-0.02, -0.02], [0.02, -0.02], [0.02, 0.02], [-0.02, 0.02]];
    model.z_bounds_m = [-0.05, 0.05];
    let source = build_map(&ground(Terrain::Flat), false).with_body_envelope(model).unwrap();
    let pitch = 0.3_f64;
    let q0 = [0.0, (pitch * 0.5).sin(), 0.0, (pitch * 0.5).cos()];
    let yaw = std::f64::consts::FRAC_PI_2;
    let yaw_rotation = rotation([0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]).unwrap();
    let q1 = quaternion_from_columns(rotation(q0).unwrap().map(|axis| world(yaw_rotation, [0.0; 3], axis)));
    let from = ([0.025, 0.025, 0.4], q0); let to = (from.0, q1);
    let boundary = [[-0.025, -0.025], [0.075, -0.025], [0.075, 0.075], [-0.025, 0.075]];
    assert!(segment_inside_boundary([from.0[0], from.0[1]], [to.0[0], to.0[1]], &boundary));
    for pose in [from, to] {
        let support = source.support_position_from_body(pose.0, pose.1, 0.4).unwrap();
        assert!(!inside_boundary([support[0], support[1]], &boundary),
            "fixed rotated nominal point must actually be outside: {support:?}");
    }
    for i in 0..=36 {
        let pose = interpolated_body_pose(from, to, f64::from(i) / 36.0);
        let r = rotation(pose.1).unwrap();
        for point in &source.body_envelope().unwrap().footprint_xy_m {
            for z in [-0.05, 0.05] {
                let corner = world(r, pose.0, [point[0], point[1], z]);
                assert!(inside_boundary([corner[0], corner[1]], &boundary));
                assert!(corner[2] > 0.0, "tiny torso must clear source floor before interpreting the result");
            }
        }
    }
    let mut blocked = vec![false; 20 * 20]; blocked[10 * 20 + 10] = true;
    // The actual body origin lies in forbidden XY [0,.1]², so the paired
    // failure is a real physical exclusion, not a shifted nominal boundary.
    assert!(from.0[0] >= 0.0 && from.0[0] < 0.1 && from.0[1] >= 0.0 && from.0[1] < 0.1);
    let forbidden = source.with_virtual_obstacles([-1.0, -1.0], 0.1, 20, 20, &blocked).unwrap();
    for (blocked, map) in [(false, &source), (true, &forbidden)] {
        let grounded = c0_terrain_bound(map, c0_terrain_sample(map, [0.025, 0.025], 0.0,
            GroundQueryState::LegacyGeometry));
        let unanchored = map.bind_body_query(BodyQueryReference::Unanchored);
        for (mode, view) in [("Ground origin boundary", &grounded), ("Unanchored origin boundary", &unanchored)] {
            let probe = c0_terrain_probe(mode, view, from, to, yaw, &boundary);
            if blocked {
                assert_eq!(probe.poses, [Err(SurfaceError::VirtualObstacle), Err(SurfaceError::VirtualObstacle)]);
                assert_eq!(probe.projection, Err(SurfaceError::VirtualObstacle));
                assert_eq!(probe.motion, Err(SurfaceError::VirtualObstacle));
                assert_eq!(probe.pose_witnesses, [None, None]);
                assert_eq!(probe.projection_witness, None); assert_eq!(probe.motion_witness, None);
            } else {
                assert_eq!(probe.poses, [Ok(()), Ok(())]); assert_eq!(probe.pose_witnesses, [None, None]);
                assert_eq!(probe.motion, Ok(())); assert_eq!(probe.motion_witness, None);
                let projected = probe.projection.as_ref().expect("same-origin yaw projection must clear");
                assert_eq!(projected.0, from.0); assert_eq!(probe.projection_witness, None);
                let actual = rotation(projected.1).unwrap();
                assert!((actual[0][1].atan2(actual[0][0]) - yaw).abs() < 1e-12);
                assert!((actual[2][2] - pitch.cos()).abs() < 1e-12,
                    "projection cannot erase actual tilt to pass the boundary");
                // The fixed q0->q1 pair above is the full-motion oracle. The
                // projector retains the original mode's attitude/yaw response;
                // it need not use the same world-normal transport convention.
                let mut witness = None;
                view.motion_on_segment(from, *projected, [0.0, 1.0], &boundary, &mut witness).unwrap();
                assert_eq!(witness, None);
                let mut checker = view.geometry_checker(&boundary);
                checker(from, to, [0.0, 1.0], &mut witness).unwrap();
                assert_eq!(witness, None);
            }
            assert_eq!(c0_terrain_probe("origin boundary repeat", view, from, to, yaw, &boundary), probe);
        }
    }
}
