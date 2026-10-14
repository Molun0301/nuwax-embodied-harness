//! 用真实细体素门框验证完整机体许可，存储分块不应改变完整源的物理含义。
use super::*;
use std::collections::BTreeMap;

fn door(gap:i32,block_edge:i32)->(SurfaceSnapshot,SurfaceObservations) {
    door_transformed(gap,block_edge,[0,0],false)
}
fn door_transformed(gap:i32,block_edge:i32,offset:[i32;2],quarter_turn:bool)->(SurfaceSnapshot,SurfaceObservations) {
    let fine=f64::from(0.01_f32);let width=32.0*fine;
    let mut keys:Vec<[i32;3]>=(-100..100).flat_map(|x|(-50..100).map(move|y|[x,y,-1])).collect();
    keys.extend((0..4).flat_map(|x|(-50..100).filter(move|&y|y<0||y>=gap)
        .flat_map(move|y|(0..80).map(move|z|[x,y,z]))));
    for key in &mut keys {
        if quarter_turn {*key=[-key[1]-1,key[0],key[2]];}
        key[0]+=offset[0];key[1]+=offset[1];
    }
    let points:Vec<_>=keys.iter().map(|k|(((f64::from(k[0])+0.5)*fine)as f32,
        ((f64::from(k[1])+0.5)*fine)as f32,((f64::from(k[2])+0.5)*fine)as f32)).collect();
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    for key in keys {blocks.entry(key.map(|v|v.div_euclid(block_edge))).or_default()
        .insert(key,SurfaceObservation{observed_ns:0,hits:0});}
    let mut observations=SurfaceObservations::new(0.01,block_edge).unwrap();
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
    let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,surface_closing_radius:0.0,
        node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.4,wall_buffer_weight:2.0,max_slope_deg:18.0,
        step_threshold_m:0.0,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0};
    let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.3,-width/2.0],[0.3,-width/2.0],[0.3,width/2.0],[-0.3,width/2.0]],
        z_bounds_m:[-0.15,0.15],body_to_support_z_m:0.4,max_step_up_m:0.0,max_step_down_m:0.0,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    (SurfaceSnapshot::from_points(&points,config,0.01).unwrap().with_body_envelope(envelope).unwrap(),observations)
}
fn pose(x:f64,y:f64,yaw:f64)->([f64;3],[f64;4]) {
    ([x,y,0.4],[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()])
}

#[test]
fn complete_fine_door_geometry_is_independent_of_sparse_block_packing() {
    let fine=f64::from(0.01_f32);let centre=33.0*fine/2.0;
    for block_edge in [16,32,64] {
        let (base,mut observations)=door(33,block_edge);
        let middle=pose(2.0*fine,centre,0.0);
        assert!(base.validate_body_pose(middle.0,middle.1).is_err(),"粗半列被门框占据覆盖");
        if block_edge==32 {
            let partial=base.with_observations(Some(Arc::new(observations.clone())));
            assert!(partial.validate_body_pose(middle.0,middle.1).is_err(),"未确认完整源时保留缺块拒绝");
        }
        observations.mark_complete_geometry_source();
        let map=base.with_observations(Some(Arc::new(observations)));
        map.validate_body_pose(middle.0,middle.1).unwrap();
        let start=pose(-0.5,centre,0.0);let end=pose(0.5,centre,0.0);
        map.validate_body_motion(start,end,&[]).unwrap();
        let mut current=start;
        for step in 1..=100 {
            let xy=[-0.5+step as f64*0.01,centre];
            let next=map.body_pose_after_motion(current,xy,0.0).unwrap_or_else(|error|panic!(
                "block={block_edge}, step={step}, from={current:?}, to={xy:?}, pose={:?}, support={:?}, frame={:?}, error={error}",
                map.validate_body_pose([xy[0],xy[1],0.4],start.1),
                map.sample_segment_support([current.0[0],current.0[1],0.0],[xy[0],xy[1],0.0],1.0),
                map.body_orientation_at([xy[0],xy[1],0.0],0.0)));

            map.validate_body_motion(current,next,&[]).unwrap();current=next;
        }
        assert!((current.0[0]-end.0[0]).abs()<1e-12);
    }
}

#[test]
fn narrow_door_keeps_equal_width_collision_alignment_and_wall_start_checks() {
    let fine=f64::from(0.01_f32);let width=32.0*fine;
    for gap in [30,32,33,34] {
        let (base,mut observations)=door(gap,32);observations.mark_complete_geometry_source();
        let map=base.with_observations(Some(Arc::new(observations)));let centre=gap as f64*fine/2.0;
        for angle in [0.0_f64,0.5,5.0,10.0] {
            let yaw=angle.to_radians();let position=|x:f64|pose(x,centre+(x-2.0*fine)*yaw.tan(),yaw);
            // 门框厚度也参与斜入几何；不能把整个长机体的外接圆/投影宽当门洞截面。
            let required=width/yaw.cos()+4.0*fine*yaw.tan().abs();
            let legal=gap as f64*fine>required+1e-10;
            let middle=position(2.0*fine);
            assert_eq!(map.validate_body_pose(middle.0,middle.1).is_ok(),legal,"门宽{gap}、偏航{angle}");
            assert_eq!(map.validate_body_motion(position(-0.5),position(0.5),&[]).is_ok(),legal,
                "完整扫掠必须与真实门截面一致，门宽{gap}、偏航{angle}");
        }
        if gap==33 {
            let start=pose(-0.5,centre,0.2);let aligned=pose(-0.5,centre,0.0);
            map.validate_body_motion(start,aligned,&[]).unwrap();
            map.validate_body_motion(aligned,pose(0.5,centre,0.0),&[]).unwrap();
            assert!(map.validate_body_motion(pose(2.0*fine,centre,0.0),pose(2.0*fine,centre,0.7),&[]).is_err());
            let near=centre+(gap as f64*fine-width)*0.5-1e-5;
            let a=pose(2.0*fine,near,0.0);map.validate_body_pose(a.0,a.1).unwrap();
            let view=map.with_current_body_support(a.0,a.1).unwrap();
            view.validate_body_motion(a,pose(0.5,near,0.0),&[]).unwrap();
            let hit=pose(2.0*fine,near+0.001,0.0);
            assert!(view.validate_body_pose(hit.0,hit.1).is_err(),"真实入墙不能借当前足迹或完整源获得许可");
        }
    }
}

#[test]
fn complete_source_fact_cannot_erase_coarse_conflicts_or_independent_boundaries() {
    let fine=f64::from(0.01_f32);let centre=33.0*fine/2.0;
    let (base,mut observations)=door(33,32);observations.mark_complete_geometry_source();
    let complete=base.with_observations(Some(Arc::new(observations.clone())));
    let middle=pose(2.0*fine,centre,0.0);complete.validate_body_pose(middle.0,middle.1).unwrap();
    observations.replace_block([0,1,0],BTreeMap::new()).unwrap();
    let partial=complete.with_observations(Some(Arc::new(observations.clone())));
    assert!(partial.validate_body_pose(middle.0,middle.1).is_err());
    observations.mark_complete_geometry_source();
    let conflicting=complete.with_observations(Some(Arc::new(observations)));
    assert!(conflicting.validate_body_pose(middle.0,middle.1).is_err(),"重新声明完整不能补出粗图中仍存在的墙占据来源");
    complete.validate_body_pose(middle.0,middle.1).unwrap();
    let mut wrong=complete.observations.as_ref().unwrap().as_ref().clone();wrong.size_m=0.02;
    let wrong_source=complete.with_observations(Some(Arc::new(wrong)));
    assert!(matches!(wrong_source.validate_body_pose(middle.0,middle.1),Err(SurfaceError::InvalidConfig(_))),
        "完整源标记不能改变几何实际来源的分辨率");
    let mut blocked=vec![false;200*150];
    for y in 0..150 {blocked[y*200+100]=true;}
    let masked=complete.with_virtual_obstacles([-1.0,-0.5],0.01,200,150,&blocked).unwrap();
    assert!(masked.validate_body_pose(middle.0,middle.1).is_err(),"完整精细源不能覆盖独立虚拟禁区");
    let boundary=[[-0.8,-0.4],[-0.01,-0.4],[-0.01,0.8],[-0.8,0.8]];
    assert!(complete.validate_body_motion(pose(-0.5,centre,0.0),pose(0.5,centre,0.0),&boundary).is_err());
}

#[test]
fn fine_wall_contacts_are_not_cut_off_by_coarse_float_boundaries() {
    let fine=f64::from(0.01_f32);
    for gap in [40,41] {
        let (base,mut observations)=door(gap,32);
        let mut envelope=base.body_envelope().unwrap().clone();
        envelope.footprint_xy_m=vec![[-0.3,-20.0*fine],[0.3,-20.0*fine],[0.3,20.0*fine],[-0.3,20.0*fine]];
        let y=gap as f64*fine*0.5;let middle=pose(2.0*fine,y,0.0);
        let base=base.with_body_envelope(envelope).unwrap();
        if gap==40 {
            let partial=base.with_observations(Some(Arc::new(observations.clone())));
            assert!(partial.validate_body_pose(middle.0,middle.1).is_err(),
                "已知细墙接触属于拒绝证据，不依赖完整源标记");
        }
        observations.mark_complete_geometry_source();
        let map=base.with_observations(Some(Arc::new(observations)));
        assert_eq!(map.validate_body_pose(middle.0,middle.1).is_ok(),gap>40,
            "真实细墙接触不能被粗列包围框的单精度边界截掉，门宽={gap}");
        assert_eq!(map.validate_body_motion(pose(-0.5,y,0.0),pose(0.5,y,0.0),&[]).is_ok(),gap>40);
    }
}


#[test]
fn local_passage_candidates_resolve_subcell_windows_without_granting_motion() {
    let fine=f64::from(0.01_f32);
    for gap in [32,33,34] {for (offset,rotated) in [([0,0],false),([17,-23],false),([17,-23],true)] {
        let (base,mut observations)=door_transformed(gap,32,offset,rotated);
        observations.mark_complete_geometry_source();let map=base.with_observations(Some(Arc::new(observations)));
        let heading=if rotated {std::f64::consts::FRAC_PI_2}else{0.0};
        let transform=|x:f64,y:f64|if rotated {[offset[0] as f64*fine-y,offset[1] as f64*fine+x,0.4]}
            else {[offset[0] as f64*fine+x,offset[1] as f64*fine+y,0.4]};
        let candidates=map.body_passage_candidates(transform(0.02,0.30),heading,[0.15,0.6]).unwrap();
        if gap==32 {assert!(candidates.is_empty(),"等宽接触不能生成严格自由窗口");continue;}
        assert_eq!(candidates.len(),1);let candidate=candidates[0];
        let expected=transform(2.0*fine,gap as f64*fine/2.0);
        assert!((candidate.center_xyz[0]-expected[0]).abs()<1e-8);
        assert!((candidate.center_xyz[1]-expected[1]).abs()<1e-8);
        assert!((candidate.clear_width_m-gap as f64*fine).abs()<1e-8);
        let q=[0.0,0.0,(heading*0.5).sin(),(heading*0.5).cos()];
        map.validate_body_motion((candidate.entry_xyz,q),(candidate.exit_xyz,q),&[]).unwrap();
        // 接口不检查独立边界；消费者必须复核才能获得运动许可。
        let boundary=[[-2.0,-2.0],[-1.5,-2.0],[-1.5,-1.5],[-2.0,-1.5]];
        assert!(map.validate_body_motion((candidate.entry_xyz,q),(candidate.exit_xyz,q),&boundary).is_err());
    }}
    let (base,mut observations)=door(33,32);observations.mark_complete_geometry_source();
    let map=base.with_observations(Some(Arc::new(observations)));
    assert!(map.body_passage_candidates([0.02,0.3,0.4],0.0,[100.0,100.0]).is_err());
    assert!(map.body_passage_candidates([0.02,0.3,0.4],f64::NAN,[0.1,0.1]).is_err());
    assert!(base.body_passage_candidates([0.02,0.3,0.4],0.0,[0.1,0.1]).unwrap().is_empty());
    // 斜入候选仍须通过真实扫掠：5度可过，10度截面超宽。
    let heading=5.0_f64.to_radians();
    let candidates=map.body_passage_candidates([0.02,0.3,0.4],heading,[0.15,0.6]).unwrap();
    assert_eq!(candidates.len(),1);let candidate=candidates[0];
    let q=[0.0,0.0,(heading*0.5).sin(),(heading*0.5).cos()];
    map.validate_body_motion((candidate.entry_xyz,q),(candidate.exit_xyz,q),&[]).unwrap();
    assert!(map.body_passage_candidates([0.02,0.3,0.4],10.0_f64.to_radians(),[0.15,0.6]).unwrap().is_empty());
    let began=std::time::Instant::now();
    for _ in 0..20 {assert_eq!(map.body_passage_candidates([0.02,0.3,0.4],0.0,[0.15,0.6]).unwrap().len(),1);}
    eprintln!("局部通道查询：20次共{:?}，每次约3800细列，不扫描全图",began.elapsed());
}

#[test]
fn offset_start_and_goal_have_forward_s_arcs_through_fine_door_window() {
    let (base,mut observations)=door(33,32);observations.mark_complete_geometry_source();
    let map=base.with_observations(Some(Arc::new(observations)));
    let candidate=map.body_passage_candidates([0.02,0.3,0.4],0.0,[0.15,0.6]).unwrap()[0];
    let curve=|mut current:([f64;3],[f64;4]),end:[f64;3]| {
        let start=current.0;let dx=end[0]-start[0];let dy=end[1]-start[1];
        let sign=dy.signum();let theta=2.0*(dy.abs()/dx).atan();
        let radius=(dx*dx+dy*dy)/(4.0*dy.abs());
        for part in 0..2 {for step in 1..=60 {
            let a=theta*step as f64/60.0;
            let (x,y,yaw)=if part==0 {(radius*a.sin(),sign*radius*(1.0-a.cos()),sign*a)}
                else {(radius*(2.0*theta.sin()-(theta-a).sin()),
                    sign*radius*(1.0+(theta-a).cos()-2.0*theta.cos()),sign*(theta-a))};
            let next=map.body_pose_after_motion(current,[start[0]+x,start[1]+y],yaw).unwrap();
            map.validate_body_motion(current,next,&[]).unwrap_or_else(|error|panic!("S段{part}/{step}: {current:?} → {next:?}: {error}"));
            // 两段圆弧的位移方向始终与中点机体朝向一致，机体前向且零侧移。
            let previous_yaw=2.0*current.1[2].atan2(current.1[3]);let mid_yaw=(previous_yaw+yaw)*0.5;
            let delta=[next.0[0]-current.0[0],next.0[1]-current.0[1]];
            assert!(delta[0]*mid_yaw.cos()+delta[1]*mid_yaw.sin()>0.0);
            assert!((-delta[0]*mid_yaw.sin()+delta[1]*mid_yaw.cos()).abs()<1e-10);
            current=next;
        }}
        assert!((current.0[0]-end[0]).hypot(current.0[1]-end[1])<1e-9);current
    };
    let entry=curve(pose(-0.65,0.3,0.0),candidate.entry_xyz);
    let exit=map.body_pose_after_motion(entry,[candidate.exit_xyz[0],candidate.exit_xyz[1]],0.0).unwrap();
    map.validate_body_motion(entry,exit,&[]).unwrap();
    curve(exit,[0.65,0.3,0.4]);
}
