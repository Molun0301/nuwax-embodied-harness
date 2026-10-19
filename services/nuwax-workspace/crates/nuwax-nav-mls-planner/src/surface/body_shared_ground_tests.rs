use super::*;

fn layered_map(runs:&[[i32;2]])->SurfaceSnapshot {
    let quantum=0.05_f32;
    let keys:Vec<_>=(-12..12).flat_map(|x|(-12..12).flat_map(move|y|
        runs.iter().filter(move|run|**run!=[2,2] || (x==0 && y==0))
            .flat_map(move|run|(run[0]..=run[1]).map(move|z|[x,y,z])))).collect();
    let points:Vec<_>=keys.iter().map(|k|((k[0] as f32+0.5)*quantum,
        (k[1] as f32+0.5)*quantum,(k[2] as f32+0.5)*quantum)).collect();
    let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.1;cfg.wall_clearance_m=0.0;
    let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
    for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
        .insert(key,SurfaceObservation{observed_ns:0,hits:0});}
    let mut observations=SurfaceObservations::new(quantum,32).unwrap();
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
    observations.mark_complete_geometry_source();
    SurfaceSnapshot::from_points(&points,cfg,quantum).unwrap().with_body_envelope(model()).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

#[test]
fn shared_legacy_bands_keep_exact_membership_contact_quantization_and_provenance() {
    for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
        let map=layered_map(&[[-3,-1],[17,19]]).with_ground_evidence(evidence);
        let quantum=f64::from(map.source_voxel_size_m());
        let bands=map.fine_contact_bands(0,0);
        assert_eq!(bands.len(),2);
        for (band,run) in bands.iter().zip([[-3,-1],[17,19]]) {
            let shared=GroundLayer::from_voxel_run(run,quantum,evidence).unwrap();
            assert_eq!(band.run,run);
            assert_eq!(band.fit,shared.contact_interval);
            assert_eq!(shared.approximation,if evidence==GroundEvidence::MeasuredSurface {
                GroundApproximation::LegacyContactInterval
            } else {GroundApproximation::ExposedVolumeTop});
            for z in run[0]..=run[1] {assert!(map.observations.as_ref().unwrap().get([0,0,z]).is_some());}
        }
        assert!(bands[0].fit[1]<bands[1].fit[0]);
    }
}

fn with_measured_floor(map:SurfaceSnapshot,plane_z:f64)->SurfaceSnapshot {
    with_measured_floor_heights(map,|_,_|plane_z)
}

fn with_measured_floor_heights(map:SurfaceSnapshot,height:impl Fn(i32,i32)->f64)->SurfaceSnapshot {
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
    let mut observations=map.observations.as_ref().unwrap().as_ref().clone();
    let q=f64::from(observations.size_m);
    let mut occupancy=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
    for (column,levels) in observations.navigation_columns() {for &z in levels {
        let key=[column[0],column[1],z];
        occupancy.entry(block_key(key)).or_default().insert(key,SurfaceObservation{observed_ns:1000,hits:1});
    }}
    for (key,values) in occupancy {observations.replace_block(key,values).unwrap();}
    let mut blocks=std::collections::BTreeMap::<[i32;3],LayerObservations>::new();
    for x in -12..12 {for y in -12..12 {
        let plane_z=height(x,y);
        let key=[x,y,(plane_z/q).floor() as i32];let local=local_key(key);
        let origin=[(f64::from(x)+0.5)*q,(f64::from(y)+0.5)*q,plane_z];
        blocks.entry(block_key(key)).or_default().insert(local,LayerObservation {
            members:vec![local],normal:[0.0,0.0,1.0],plane_origin:origin,
            samples:(0..3).map(|i|LayerSample{source: Default::default(), point:origin,source_sequence:i+1,
                source_timestamp_ns:100+i,point_index:u32::from(local)}).collect(),
        });
    }}
    for (block,layers) in blocks {observations.replace_ground_layers(block,Arc::new(layers)).unwrap();}
    observations.mark_complete_geometry_source();
    map.with_observations(Some(Arc::new(observations)))
}

#[test]
fn measured_layer_change_recomputes_contact_neighbors_without_rewriting_occupancy() {
    let mut envelope=model();envelope.max_step_up_m=0.0;envelope.max_step_down_m=0.0;
    let map=with_measured_floor(layered_map(&[[-2,-1]]),-0.075)
        .with_body_envelope(envelope).unwrap();
    let centre=map.locate_support([0.05,0.05,-0.075]).unwrap();
    let old_patch=map.ground_patch(centre,0.4);
    assert!(old_patch.contains_key(&(2,0)));
    let changed=with_measured_floor_heights(map.clone(),|x,_|if x>=4 {-0.005}else{-0.075});
    assert!(map.same_geometry(&changed),"层面更新不能重建未变化的物理占据图");
    let next=changed.locate_support([0.05,0.05,-0.075]).unwrap();
    assert_eq!(centre,next,"中心未变化，变化发生在接触邻域边缘");
    assert!(!changed.ground_patch(next,0.4).contains_key(&(2,0)),
        "零越阶机体不能复用旧平面跨过新测得的7厘米高度差");
    assert!(map.ground_patch(centre,0.4).contains_key(&(2,0)),"旧快照仍须返回自己的地面层");
}

#[test]
fn observed_ground_height_does_not_absorb_an_upper_return_in_the_same_coarse_cell() {
    let mut envelope=model();envelope.z_bounds_m=[-0.4,0.2];
    let map=with_measured_floor(layered_map(&[[0,0],[1,1]]),0.025)
        .with_body_envelope(envelope).unwrap();
    let pose=([0.025,0.025,0.425],[0.,0.,0.,1.]);
    let location=map.locate_support_from_body(pose.0,pose.1,0.4).unwrap();
    assert!((location.surface_xyz[2]-0.025).abs()<1e-12);
    assert_eq!(map.query_ground([0.025,0.025],0.025).height,Some(0.025));
    assert!(map.observations.as_ref().unwrap().get([0,0,1]).is_some());
    assert_eq!(map.validate_body_pose(pose.0,pose.1),Err(SurfaceError::InsufficientClearance),
        "地面参考下降不能把同粗格内未分类的真实障碍变成接触面");
}

#[test]
fn support_anchor_uses_the_observed_layer_plane_at_both_grid_resolutions() {
    for coarse in [0.05,0.1] {
        let original=layered_map(&[[-1,-1],[17,19]]);
        let mut config=original.config().clone();config.voxel_size=coarse;
        let points:Vec<_>=original.observations.as_ref().unwrap().navigation_columns()
            .flat_map(|(xy,zs)|zs.iter().map(move|&z|((xy[0] as f32+0.5)*0.05,
                (xy[1] as f32+0.5)*0.05,(z as f32+0.5)*0.05))).collect();
        let source=SurfaceSnapshot::from_points(&points,config,0.05).unwrap()
            .with_body_envelope(model()).unwrap().with_observations(original.observations.clone());
        let map=with_measured_floor(source,-0.025);
        for body_z in [0.375,0.39,0.415] {
            let location=map.locate_support_from_body([0.025,0.025,body_z],[0.,0.,0.,1.],0.4).unwrap();
            eprintln!("GROUND_ANCHOR coarse={coarse} body_z={body_z} height={}",location.surface_xyz[2]);
            assert!((location.surface_xyz[2]+0.025).abs()<1e-12,
                "粗支撑定位与精细地面查询必须使用同一个真实观测平面");
            assert!((map.query_ground([0.025,0.025],body_z-0.4).height.unwrap()+0.025).abs()<1e-12);
        }
        assert!(map.observations.as_ref().unwrap().get([0,0,19]).is_some(),"上层占据不能被地面估计清除");
    }
}

#[test]
fn measured_plane_response_uses_source_height_instead_of_legacy_voxel_top() {
    let map=with_measured_floor(layered_map(&[[-1,-1],[17,19]]),-0.025);
    for reference in [-0.04,-0.025,0.0] {
        let samples=map.fine_contact_domain([0.025,0.025,reference],0.3).unwrap();
        assert!(!samples.is_empty());
        for (point,low,_) in &samples {assert_eq!(point[2],-0.025);assert_eq!(*low,-0.025);}
        let plane=fit_contact_plane(&samples,[0.025,0.025,reference]).unwrap();
        assert!((plane.2+0.025).abs()<1e-12);
    }
    let (resolution,_)=map.navigation_source_columns(None);
    let column=map.navigation_measured_ground_layers([0,0],resolution,&[[-0.05,0.0],[0.85,1.0]]).unwrap();
    assert_eq!(column[0].0.approximation,GroundApproximation::ObservedLayer);
    assert_eq!(column[0].0.representative_z,-0.025);
    assert!(column[0].1>0.8,"upper floor remains distinct ceiling");
}

#[test]
fn fixed_motion_reference_keeps_observed_layer_priority_over_legacy_midpoint() {
    let map=with_measured_floor(layered_map(&[[-3,-1],[37,39]]),-0.025);
    for reference in [-0.04,-0.025,0.0] {
        let center=map.locate_near_support([0.025,0.025,reference],0.3).unwrap();
        let dynamic=map.body_motion_contact_frame(center,reference,0.4).unwrap();
        let static_frame=map.body_contact_frame(center,reference,0.4).unwrap();
        assert_eq!(dynamic,static_frame,"实测平面应在两个模式下保持同一真实高度与法向");
        assert!((dynamic.0+0.025).abs()<1e-12,"实测高度不能换成旧占据带中点");
    }
    assert!(map.body_query_cache.lock().unwrap().planes.is_empty(),"真实层不读写旧式拟合缓存");
}

#[test]
fn measured_members_stay_fixed_as_body_rises_and_preserve_the_low_obstacle() {
    let map=with_measured_floor(layered_map(&[[-1,-1],[2,2]]),-0.025);
    let ground=(0,0,-1); // the explicit, fixed lower physical layer
    let polygon=PolygonQuery::new(&[[0.005,0.005],[0.045,0.005],[0.045,0.045],[0.005,0.045]]);
    let normal=[0.0,0.0,1.0];
    for z in [0.375,0.55,0.8] {
        let body=[0.025,0.025,z];
        let prism=PrismQuery::new(map.body_envelope().unwrap(),rotation([0.0,0.0,0.0,1.0]).unwrap(),body);
        let rows=map.fine_ground_columns(Some(ground),&polygon,&prism,body,normal,0,0).unwrap();
        let row=rows.iter().find(|r|r.xy==[0,0]).unwrap();
        assert_eq!(row.height,Some(-0.025));
        assert_eq!(row.contacts.as_slice(),&[[-1,-1]]);
        assert!(!row.permits_contact(2));
        assert!(map.observations.as_ref().unwrap().get([0,0,2]).is_some());
    }
    let q=f64::from(map.source_voxel_size_m());
    let column=map.navigation_measured_ground_layers([0,0],q,&[[-q,0.0],[2.0*q,3.0*q]]).unwrap();
    assert_eq!(column[0].1,2.0*q,"real low obstacle is still the ceiling");
}

#[test]
fn unique_upper_observed_layer_is_not_contact_without_same_physical_ground_anchor() {
    use nuwax_map_chunks::{LayerObservation,LayerSample,local_key};
    let map=layered_map(&[[-1,-1],[9,9]]);
    let mut observed=map.observations.as_ref().unwrap().as_ref().clone();
    let key=local_key([0,0,9]);let origin=[0.025,0.025,0.475];
    let layer=LayerObservation{members:vec![key],normal:[0.,0.,1.],plane_origin:origin,
        samples:vec![LayerSample{source: Default::default(), point:origin,source_sequence:1,source_timestamp_ns:1,point_index:0}]};
    observed.replace_ground_layers([0,0,0],Arc::new(std::collections::BTreeMap::from([(key,layer)]))).unwrap();
    observed.mark_complete_geometry_source();let map=map.with_observations(Some(Arc::new(observed)));
    let body=[0.025,0.025,0.4];let r=rotation([0.,0.,0.,1.]).unwrap();
    let polygon=PolygonQuery::new(&[[0.005,0.005],[0.045,0.005],[0.045,0.045],[0.005,0.045]]);
    let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
    for ground in [None,Some((0,0,-1))] {
        let rows=map.fine_ground_columns(ground,&polygon,&prism,body,r[2],0,0).unwrap();
        assert!(!rows.iter().find(|row|row.xy==[0,0]).unwrap().permits_contact(9));
    }
    assert_eq!(map.validate_body_pose(body,[0.,0.,0.,1.]),Err(SurfaceError::InsufficientClearance));
}

#[test]
fn partial_fine_cell_does_not_clear_unassigned_coarse_obstacle_height() {
    let map=scene(&[(0.025,0.025,0.575)],false)
        .with_observations(Some(witnessed_observations(&[(0.025,0.025,0.525)],0.05)));
    let body=[0.025,0.025,0.66];let r=rotation([0.,0.,0.,1.]).unwrap();
    let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
    let polygon=PolygonQuery::new(&[[-0.525,-0.115],[0.575,-0.115],[0.575,0.165],[-0.525,0.165]]);
    let mut witness=None;
    assert_eq!(map.check_unassigned_body_column(body,r,&polygon,&prism,0,0,&mut witness),Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness,Some(BodyRejection::Occupancy{source:OccupancySource::CoarseFallback,..})));
}

#[test]
fn measured_contact_members_do_not_require_a_connected_ground_anchor() {
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, block_key, local_key};
    let floor=[5,0,-1];
    let point=[0.275,0.025,-0.025];
    let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.4;
    let mut envelope=model();envelope.z_bounds_m=[-0.45,0.25];
    let mut observations=SurfaceObservations::new(0.05,32).unwrap();
    observations.replace_block(block_key(floor),std::collections::BTreeMap::from([
        (floor,SurfaceObservation{observed_ns:100,hits:1})])).unwrap();
    let local=local_key(floor);
    let layers=LayerObservations::from([(local,LayerObservation {
        members:vec![local],normal:[0.0,0.0,1.0],plane_origin:point,
        samples:vec![LayerSample{source: Default::default(), point,source_sequence:1,source_timestamp_ns:100,point_index:0}],
    })]);
    observations.replace_ground_layers(block_key(floor),Arc::new(layers)).unwrap();
    observations.mark_complete_geometry_source();
    let points=[(point[0] as f32,point[1] as f32,point[2] as f32)];
    let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap()
        .with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)));
    let pose=([0.025,0.025,0.4],[0.0,0.0,0.0,1.0]);
    assert_eq!(map.locate_support_from_body(pose.0,pose.1,0.4),Err(SurfaceError::MissingSupport));
    map.validate_body_pose(pose.0,pose.1).unwrap();
    let view=map.with_current_body_support(pose.0,pose.1).unwrap();
    assert!(view.same_body_query_view(&map));
    assert_eq!(view.query_ground([pose.0[0],pose.0[1]],0.0).state,GroundQueryState::Unknown);
    assert!(view.observations.as_ref().unwrap().get(floor).is_some());
    // The accepted floor does not grant contact to a new recent object in its column.
    let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[point[0],point[1],0.35])]).unwrap();
    assert_eq!(blocked.validate_body_pose(pose.0,pose.1),Err(SurfaceError::InsufficientClearance));
}

#[test]
fn unknown_motion_keeps_current_reference_and_factual_unknown_state() {
    let map=scene(&[],false);
    let pose=([4.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
    assert_eq!(map.query_ground([4.05,0.05],0.1).state,GroundQueryState::Unknown);
    let next=map.body_pose_after_motion(pose,[4.35,0.05],0.25).unwrap();
    assert_eq!(next.0,[4.35,0.05,0.5]);
    assert!((rotation(next.1).unwrap()[0][1].atan2(rotation(next.1).unwrap()[0][0])-0.25).abs()<1e-10);
    map.validate_body_motion(pose,next,&[]).unwrap();
    assert_eq!(map.query_ground([4.35,0.05],0.1).state,GroundQueryState::Unknown);
    assert!(map.locate_support([4.35,0.05,0.1]).is_err());
    let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[4.2,0.05,0.55])]).unwrap();
    assert_eq!(blocked.validate_body_motion(pose,next,&[]),Err(SurfaceError::InsufficientClearance));
}

#[test]
fn reported_g1_pose_keeps_unclassified_ankle_height_voxel_as_obstacle() {
    // 仅复用报告的姿态和包络；平地与单格障碍为合成输入，不冒充遗失的现场地图。
    let body=[0.025503599830406017,-0.01896920386050657,0.026239193644733066];
    let q=[-0.020289298542609635,0.003576779120836624,0.2868225967805427,0.9577621567966841];
    let envelope=SurfaceBodyEnvelope {
        footprint_xy_m:vec![[-0.029020616290581558,-0.21585853486343462],
            [0.4539265102797319,-0.21585853486343462],[0.4539265102797319,0.220775355547328],
            [-0.029020616290581558,0.220775355547328]],
        z_bounds_m:[-0.9432042695110971,0.3386525932121276],
        body_to_support_z_m:0.9432044618093445,max_step_up_m:0.0,max_step_down_m:0.0,
        max_cross_slope_deg:10.0,support_uncertainty_m:0.02,
    };
    let make_map=|obstacle:bool| {
        let fine=0.05_f32;
        let mut keys:Vec<[i32;3]>=(-16..16).flat_map(|x|(-16..16).map(move|y|[x,y,-19])).collect();
        if obstacle {keys.push([5,4,-18]);}
        let points:Vec<_>=keys.iter().map(|k|((k[0] as f32+0.5)*fine,
            (k[1] as f32+0.5)*fine,(k[2] as f32+0.5)*fine)).collect();
        let mut blocks=std::collections::BTreeMap::<_,std::collections::BTreeMap<_,_>>::new();
        for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
            .insert(key,SurfaceObservation{observed_ns:100,hits:1});}
        let mut observations=SurfaceObservations::new(fine,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        observations.mark_complete_geometry_source();
        let mut config=scene(&[],false).config().clone();config.wall_clearance_m=0.0;
        SurfaceSnapshot::from_points(&points,config,fine).unwrap()
            .with_body_envelope(envelope.clone()).unwrap().with_observations(Some(Arc::new(observations)))
    };
    let floor=make_map(false);
    assert_eq!(floor.validate_body_pose(body,q),Ok(()),"精细地面应已有接触豁免");
    let blocked=make_map(true);
    let mut witness=None;
    assert_eq!(blocked.check_body_pose(body,q,&mut witness),Err(SurfaceError::InsufficientClearance));
    assert!(matches!(witness,Some(BodyRejection::Occupancy{cell:[5,4,-18],source:OccupancySource::Fine,..})),"{witness:?}");
    eprintln!("报告姿态合成对照：平地通过，未分类低障碍保持拒绝：{witness:?}");
}

#[test]
fn recent_guide_and_body_share_measured_contact_when_center_ground_is_unknown() {
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, block_key, local_key};
    let key=[5,0,-1];let point=[0.275,0.025,-0.025];
    let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.4;
    let mut envelope=model();envelope.z_bounds_m=[-0.45,0.25];
    let mut observations=SurfaceObservations::new(0.05,32).unwrap();
    observations.replace_block(block_key(key),std::collections::BTreeMap::from([
        (key,SurfaceObservation{observed_ns:100,hits:1})])).unwrap();
    let local=local_key(key);
    let layers=LayerObservations::from([(local,LayerObservation {
        members:vec![local],normal:[0.0,0.0,1.0],plane_origin:point,
        samples:vec![LayerSample{source:Default::default(),point,source_sequence:1,
            source_timestamp_ns:100,point_index:0}],
    })]);
    observations.replace_ground_layers(block_key(key),Arc::new(layers)).unwrap();
    observations.mark_complete_geometry_source();
    let map=SurfaceSnapshot::from_points(&[(point[0] as f32,point[1] as f32,point[2] as f32)],cfg,0.05)
        .unwrap().with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)));
    let pose=([0.025,0.025,0.4],[0.0,0.0,0.0,1.0]);
    assert_eq!(map.locate_support_from_body(pose.0,pose.1,0.4),Err(SurfaceError::MissingSupport));
    let floor=map.with_recent_obstacle_points(1,100,0,&[0],[(0,point)]).unwrap();
    floor.validate_body_pose(pose.0,pose.1).unwrap();
    let bounds=[[-0.5,-0.5],[0.5,0.5]];
    assert!(floor.recent_obstacle_hints(bounds,pose).unwrap().is_empty(),
        "an accepted measured floor must not become a detour obstacle because the center lacks ground");
    assert_eq!(floor.query_ground([pose.0[0],pose.0[1]],0.0).state,GroundQueryState::Unknown);
    for indices in [&[][..],&[0][..]] {
        let obstacle=[point[0],point[1],0.35];
        let blocked=map.with_recent_obstacle_points(2,200,0,indices,[(0,obstacle)]).unwrap();
        assert_eq!(blocked.validate_body_pose(pose.0,pose.1),Err(SurfaceError::InsufficientClearance));
        assert_eq!(blocked.recent_obstacle_hints(bounds,pose).unwrap().len(),1,
            "an independent volume or elevated plane still supplies a real detour obstacle");
    }
}

// Frozen pre-fix guide only, for the opt-in same-source cost comparison below.
impl SurfaceSnapshot {
    fn legacy_recent_obstacle_hints(&self,bounds_xy:[[f64;2];2],reference_body:([f64;3],[f64;4]))
        ->Result<Vec<([f64;3],[f64;3])>,SurfaceError> {
        let Some(recent)=&self.recent_obstacles else {return Ok(vec![]);};
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        if !bounds_xy.into_iter().flatten().all(f64::is_finite)
            || (0..2).any(|i|bounds_xy[0][i]>bounds_xy[1][i])
            || bounds_xy.into_iter().flatten().any(|v|(v/recent.size).abs()>=f64::from(i32::MAX)*0.5) {
            return Err(SurfaceError::InvalidConfig("invalid recent-obstacle hint bounds".into()));
        }
        if !reference_body.0.into_iter().all(f64::is_finite) {return Err(SurfaceError::InvalidPose);}
        let r=rotation(reference_body.1)?;
        if r[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        let fine=recent.size;let coarse=f64::from(self.config().voxel_size);
        let low=bounds_xy[0].map(|v|(v/fine).floor() as i32);
        let high=bounds_xy[1].map(|v|(v/fine).floor() as i32);
        let centre=self.locate_support_from_body(reference_body.0,reference_body.1,model.body_to_support_z_m).ok();
        let radius=bounds_xy.into_iter().map(|p|(p[0]-reference_body.0[0]).hypot(p[1]-reference_body.0[1]))
            .fold(0.0,f64::max)+coarse*2.0;
        let patch=centre.map(|c|self.ground_patch(c,radius));
        let mut result=Vec::new();
        for frame in &recent.frames {for (&[x,y],zs) in frame.columns.range(low..=high) {
            if y<low[1] || y>high[1] {continue;}
            let xy=[x,y].map(|v|(f64::from(v)+0.5)*fine);
            let column=xy.map(|v|(v/coarse).floor() as i32);
            let ground=patch.as_ref().and_then(|p|p.get(&(column[0],column[1]))).copied();
            let height=ground.and_then(|ground| {
                let base=self.cell_position(ground)[2];
                if let Some(observations)=&self.observations {
                    let records:Vec<_>=observations.ground_layers([x,y]).collect();
                    if !records.is_empty() {return records.into_iter().filter(|record|record.members.iter().any(|&z|
                        (((f64::from(z)+0.5)*fine)/coarse).floor() as i32==ground.2))
                        .filter_map(|record|record.height_at(xy)).min_by(|a,b|(a-base).abs().total_cmp(&(b-base).abs()));}
                }
                Some(base)
            });
            let plane=reference_body.0[2]-((xy[0]-reference_body.0[0])*r[2][0]+(xy[1]-reference_body.0[1])*r[2][1])/r[2][2];
            let z_low=plane+model.z_bounds_m[0]/r[2][2];let z_high=plane+model.z_bounds_m[1]/r[2][2];
            for &z in zs {
                if let Some(points)=frame.surface_points.get(&[x,y,z]) {
                    for &point in points {
                        if point[2]>=z_low && point[2]<=z_high
                            && !self.recent_surface_contact(point,height,reference_body.0,r) {result.push((point,point));}
                    }
                } else {
                    let a=[x,y,z].map(|v|f64::from(v)*fine);let b=[x,y,z].map(|v|f64::from(v+1)*fine);
                    if b[2]>=z_low && a[2]<=z_high {result.push((a,b));}
                }
            }
        }}
        Ok(result)
    }

}

#[test]
#[ignore="manual serial release A/B timing of the same recent-hint source"]
fn recent_guide_shared_contact_cost_comparison() {
    use std::{hint::black_box,time::Instant};
    for evidence in [false,true] {for stride in [1,4] {for frames in [1,8] {
        let base=layered_map(&[[-1,-1]]);
        let base=if evidence {with_measured_floor(base,-0.025)}else{base};
        let mut model=base.body_envelope().unwrap().clone();model.z_bounds_m=[-0.45,0.25];
        let mut map=base.with_body_envelope(model).unwrap();
        let points:Vec<_>=(-12_i32..12).flat_map(|x|(-12_i32..12).filter(move|y|
            x.rem_euclid(stride)==0 && y.rem_euclid(stride)==0).map(move|y|
            [(f64::from(x)+0.5)*0.05,(f64::from(y)+0.5)*0.05,-0.025])).collect();
        let ids:Vec<_>=(0..points.len() as u32).collect();
        for seq in 1..=frames {map=map.with_recent_obstacle_points(seq,seq*100,10000,&ids,
            points.iter().copied().enumerate().map(|(i,p)|(i as u32,p))).unwrap();}
        let pose=([0.025,0.025,0.4],[0.,0.,0.,1.]);let bounds=[[-0.6,-0.6],[0.6,0.6]];
        let old=map.legacy_recent_obstacle_hints(bounds,pose).unwrap();
        let new=map.recent_obstacle_hints(bounds,pose).unwrap();
        assert_eq!(new,old,"timing fixture must preserve the same interpretation");
        for cold in [false,true] {for pair in 0..5 {
            let mut ns=[0;2];
            for arm in if pair%2==0 {[0,1]}else{[1,0]} {
                for _ in 0..20 {
                    // Fork only cache owners; immutable maps/observations are shared.
                    // Exclude fork allocation from the timed hint query itself.
                    let source=if cold {map.fork_for_planning()}else{map.clone()};
                    let start=Instant::now();
                    let result=if arm==0 {source.legacy_recent_obstacle_hints(black_box(bounds),black_box(pose))}
                        else {source.recent_obstacle_hints(black_box(bounds),black_box(pose))};
                    ns[arm]+=start.elapsed().as_nanos();
                    assert_eq!(result.unwrap(),old);
                }
            }
            eprintln!("RECENT_HINT_AB measured={evidence} stride={stride} frames={frames} cold={cold} pair={pair} columns={} calls=20 old_ns={} new_ns={}",points.len()*frames as usize,ns[0],ns[1]);
        }}
    }}}
}

fn same_coarse_measured_map(measured_levels: &[i32]) -> SurfaceSnapshot {
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
    let base=layered_map(&[[0,0],[1,1]]);
    let mut observations=base.observations.as_ref().unwrap().as_ref().clone();
    let q=f64::from(observations.size_m);
    let mut blocks=std::collections::BTreeMap::<[i32;3],LayerObservations>::new();
    for x in -12..12 {for y in -12..12 {for &z in measured_levels {
        let key=[x,y,z];let local=local_key(key);
        let origin=[(f64::from(x)+0.5)*q,(f64::from(y)+0.5)*q,(f64::from(z)+0.5)*q];
        blocks.entry(block_key(key)).or_default().insert(local,LayerObservation {
            members:vec![local],normal:[0.,0.,1.],plane_origin:origin,
            samples:vec![LayerSample{source:Default::default(),point:origin,
                source_sequence:1,source_timestamp_ns:1,point_index:u32::from(local)}],
        });
    }}}
    for (block,layers) in blocks {observations.replace_ground_layers(block,Arc::new(layers)).unwrap();}
    observations.mark_complete_geometry_source();
    let mut envelope=model();envelope.z_bounds_m=[-0.4,0.2];
    base.with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)))
}

#[test]
fn upper_measured_layer_sharing_coarse_cell_keeps_query_and_contact_consistent() {
    let map=same_coarse_measured_map(&[0,1]);
    let q=f64::from(map.source_voxel_size_m());
    let upper=1.5*q;
    let body=[0.025,0.025,upper+0.4];let attitude=[0.,0.,0.,1.];
    let direct=map.query_ground([body[0],body[1]],upper);
    let located=map.locate_support_from_body(body,attitude,0.4).unwrap();
    let validation=map.validate_body_pose(body,attitude);
    eprintln!("UPPER_MEASURED_SAME_COARSE direct={:?} located={:?} validation={:?}",direct.height,located.surface_xyz,validation);
    assert_eq!(direct.height,Some(upper));
    assert_eq!(located.surface_xyz[2],upper,"coarse support must retain the selected upper measured layer");
    assert_eq!(validation,Ok(()),"upper measured surface is a valid contact, while lower layer stays below the body");
    let lower=map.locate_support_from_body([body[0],body[1],0.5*q+0.4],attitude,0.4).unwrap();
    assert_eq!(lower.cell,located.cell);
    assert_eq!(lower.surface_xyz[2],0.5*q);
    assert!(!Arc::ptr_eq(&map.ground_patch(lower,0.4),&map.ground_patch(located,0.4)),
        "same physical cell still needs separate cached contact references");
}

#[test]
fn same_coarse_measured_members_do_not_exempt_the_other_layer_or_live_obstacles() {
    let map=same_coarse_measured_map(&[0,1]);
    let q=f64::from(map.source_voxel_size_m());
    let polygon=PolygonQuery::new(&[[0.005,0.005],[0.045,0.005],[0.045,0.045],[0.005,0.045]]);
    let attitude=[0.,0.,0.,1.];let r=rotation(attitude).unwrap();
    for selected in [0,1] {
        let height=(f64::from(selected)+0.5)*q;
        let body=[0.025,0.025,height+0.4];
        let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
        let rows=map.fine_ground_columns(Some((0,0,0)),&polygon,&prism,body,r[2],0,0).unwrap();
        let row=rows.iter().find(|r|r.xy==[0,0]).unwrap();
        assert_eq!(row.height,Some(height));
        assert_eq!(row.contacts.as_slice(),&[[selected,selected]]);
        assert!(!row.permits_contact(1-selected));
        assert_eq!(map.validate_body_pose(body,attitude),if selected==1 {Ok(())}
            else {Err(SurfaceError::InsufficientClearance)},
            "the upper measured plane remains real collision for a body on the lower plane");
    }
    let body=[0.025,0.025,1.5*q+0.4];
    let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[0.025,0.025,0.35])]).unwrap();
    assert_eq!(blocked.validate_body_pose(body,attitude),Err(SurfaceError::InsufficientClearance));
    let unclassified=same_coarse_measured_map(&[0]);
    assert_eq!(unclassified.validate_body_pose([body[0],body[1],0.5*q+0.4],attitude),
        Err(SurfaceError::InsufficientClearance),"unclassified upper occupancy must still collide");
}

#[test]
fn off_center_parallel_measured_slopes_select_at_the_actual_query_xy() {
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,local_key};
    let base=same_coarse_measured_map(&[]);
    let mut observations=base.observations.as_ref().unwrap().as_ref().clone();
    let q=f64::from(observations.size_m);
    let normal=normalize([0.45,0.0,1.0]);
    let mut layers=LayerObservations::new();
    for x in 0..2 {for y in 0..2 {for z in [0,1] {
        let local=local_key([x,y,z]);
        let xy=[x,y].map(|v|(f64::from(v)+0.5)*q);
        let height=0.045+f64::from(z)*0.04-0.45*(xy[0]-0.45*q);
        let origin=[xy[0],xy[1],height];
        layers.insert(local,LayerObservation{members:vec![local],normal,plane_origin:origin,
            samples:vec![LayerSample{source:Default::default(),point:origin,
                source_sequence:1,source_timestamp_ns:1,point_index:u32::from(local)}]});
    }}}
    observations.replace_ground_layers([0,0,0],Arc::new(layers)).unwrap();
    observations.mark_complete_geometry_source();
    let map=base.with_observations(Some(Arc::new(observations)));
    assert!(normal[2]>=f64::from(map.config().max_slope_deg).to_radians().cos());
    let xy=[0.1*q,0.5*q];
    for z in [0,1] {
        let height=0.045+f64::from(z)*0.04-0.45*(xy[0]-0.45*q);
        let query=map.query_ground(xy,height);
        assert_eq!(query.height,Some(height));
        let expected_center=0.045+f64::from(z)*0.04-0.45*(q-0.45*q);
        for tolerance in [0.2,1e-8] {
            let located=map.locate_near_support([xy[0],xy[1],height],tolerance).unwrap();
            assert!((located.surface_xyz[2]-expected_center).abs()<1e-15,
                "compare at actual XY before returning the same plane at the coarse center");
        }
        let contact=map.measured_contact_frame(xy,height,0.0).unwrap();
        assert_eq!(contact.0,height);
    }
}
fn parallel_measured_slope_map(base_height: f64, zero_step: bool, missing_upper: Option<[i32;2]>) -> SurfaceSnapshot {
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
    let q=f64::from(0.05_f32);
    let normal=normalize([0.4,0.0,1.0]);
    let mut points=Vec::new();
    let mut occupancy=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
    let mut blocks=std::collections::BTreeMap::<[i32;3],LayerObservations>::new();
    for x in -12..12 {for y in -12..12 {for floor in [0,1] {
        let xy=[x,y].map(|v|(f64::from(v)+0.5)*q);
        let height=base_height-0.4*xy[0]+f64::from(floor)*0.06;
        let key=[x,y,(height/q).floor() as i32];
        let local=local_key(key);let origin=[xy[0],xy[1],height];
        points.push((xy[0] as f32,xy[1] as f32,height as f32));
        occupancy.entry(block_key(key)).or_default().insert(key,SurfaceObservation{observed_ns:100,hits:1});
        if floor==1 && missing_upper==Some([x,y]) {continue;}
        blocks.entry(block_key(key)).or_default().insert(local,LayerObservation{members:vec![local],normal,
            plane_origin:origin,samples:vec![LayerSample{source:Default::default(),point:origin,
                source_sequence:1,source_timestamp_ns:100,point_index:u32::from(local)}]});
    }}}
    let mut observations=SurfaceObservations::new(0.05,32).unwrap();
    for (block,values) in occupancy {observations.replace_block(block,values).unwrap();}
    for (block,layers) in blocks {observations.replace_ground_layers(block,Arc::new(layers)).unwrap();}
    observations.mark_complete_geometry_source();
    let mut config=scene(&[],false).config().clone();config.wall_clearance_m=0.0;
    let mut envelope=model();
    if zero_step {envelope.max_step_up_m=0.0;envelope.max_step_down_m=0.0;}
    SurfaceSnapshot::from_points(&points,config,0.05).unwrap().with_body_envelope(envelope).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

#[test]
fn parallel_measured_slopes_keep_the_selected_floor_across_coarse_cells() {
    let map=parallel_measured_slope_map(0.045,false,None);
    let q=f64::from(map.source_voxel_size_m());
    let upper=0.045-0.4*q+0.06;
    let centre=map.locate_support([q,q,upper]).unwrap();
    assert_eq!(centre.cell,(0,0,0));
    let patch=map.ground_patch(centre,0.35);
    assert_eq!(patch.get(&(-1,0)),Some(&(-1,0,1)),
        "the upper measured slope continues left into coarse Z=1; lower Z=0 is a different floor");
}

#[test]
fn parallel_measured_slope_does_not_gain_a_false_step_from_another_layer() {
    let map=parallel_measured_slope_map(0.005,true,None);
    let q=f64::from(map.source_voxel_size_m());
    let upper=0.005-0.4*q+0.06;
    let centre=map.locate_support([q,q,upper]).unwrap();
    assert_eq!(centre.cell,(0,0,0));
    assert!(map.data.physical_graph.surface_lookup[&(-1,0)].contains(&0));
    assert!(map.data.physical_graph.surface_lookup[&(-2,0)].contains(&1));
    assert_eq!(map.ground_patch(centre,0.35).get(&(-2,0)),Some(&(-2,0,1)),
        "each real slope increment is 0.04m; selecting the middle lower plane creates a false 0.10m step");
    let lower_middle=0.005+0.4*q;
    let upper_middle=lower_middle+0.06;
    for (height,reaches_upper) in [(lower_middle,false),(upper_middle,true),
        (lower_middle,false),(upper_middle,true),(upper_middle,true),(lower_middle,false)] {
        let centre=map.locate_support([-q,q,height]).unwrap();
        assert_eq!(centre.cell,(-1,0,0));
        assert_eq!(map.ground_patch(centre,0.35).contains_key(&(-2,0)),reaches_upper,
            "height-specific cached patches must not borrow contact connectivity from the other layer");
    }
}

#[test]
fn projected_patch_reference_does_not_create_missing_measured_members() {
    // Remove one exact fine XY, preserving measured support at the coarse center.
    let map=parallel_measured_slope_map(0.005,true,Some([-4,0]));
    let q=f64::from(map.source_voxel_size_m());
    let upper=0.005-0.4*q+0.06;
    let centre=map.locate_support([q,q,upper]).unwrap();
    let patch=map.ground_patch(centre,0.35);
    let ground=*patch.get(&(-2,0)).expect("the real upper graph cell remains reachable");
    let r=rotation(map.measured_contact_frame([q,q],upper,0.0).unwrap().1).unwrap();
    let xy=[-3.5*q,0.5*q];let height=0.005-0.4*xy[0]+0.06;
    let body=[xy[0],xy[1],height+0.4/r[2][2]];
    let vertices=[[-3.9*q,0.1*q],[-3.1*q,0.1*q],[-3.1*q,0.9*q],[-3.9*q,0.9*q]];
    let polygon=PolygonQuery::new(&vertices);
    let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
    let rows=map.fine_ground_columns(Some(ground),&polygon,&prism,body,r[2],-2,0).unwrap();
    let row=rows.iter().find(|row|row.xy==[-4,0]).unwrap();
    assert_eq!(row.height,None);
    assert!(row.contacts.as_slice().is_empty(),"an extrapolated association reference supplies no observed contact members");
    let z=(height/q).floor() as i32;
    assert!(map.observations.as_ref().unwrap().get([-4,0,z]).is_some(),
        "unclassified real occupancy is still available to the collision checker");
}

mod hotpath_proposal_tests { include!("body_hotpath_proposal_tests.rs"); }

// Append-only draft for body_shared_ground_tests.rs. Not built or executed.
// Existing fixtures and tests above this append remain byte-for-byte unchanged.
use crate::surface::body_query::{
    BodyQueryReference, BoundBodyQueryView, GroundSample, LocalGroundReference,
    NativeGroundSource, QuerySample,
};

type C0SharedPose = ([f64; 3], [f64; 4]);
type C0SharedResults = [(Result<(), SurfaceError>, Option<BodyFailureWitness>); 3];

fn c0_shared_measured_sample(map: &SurfaceSnapshot, xy: [f64; 2], height: f64,
    anchor: [i32; 3]) -> GroundSample {
    let (query, sources) = map.query_ground_with_native(xy, height);
    eprintln!("C0_SHARED_SOURCE xy={xy:?} requested={height:?} query={query:?} native={sources:?}");
    assert_eq!(query.state, GroundQueryState::Observed, "fixed measured-source premise");
    assert_eq!(query.height, Some(height));
    assert_eq!(query.normal, Some([0.0, 0.0, 1.0]));
    assert_eq!(sources.len(), 1);
    assert!(matches!(sources.as_slice(), [NativeGroundSource::ObservedFine { anchor: a }] if *a == anchor));
    let observation = map.observations.as_ref().unwrap();
    let record = observation.ground_layers([anchor[0], anchor[1]])
        .find(|r| r.anchor == anchor).expect("actual source-owned measured record");
    assert_eq!(record.members.as_slice(), &[anchor[2]]);
    GroundSample {
        reference_origin: [xy[0], xy[1], height],
        reference_normal: query.normal,
        query_state: query.state,
        reference_sources: sources,
    }
}

fn c0_shared_constant(sample: GroundSample) -> BodyQueryReference {
    BodyQueryReference::Ground(Arc::new(LocalGroundReference { first: sample, changes: vec![] }))
}

// Every arm uses the real bound pose, relative projection and full-motion owners.
// A second call may warm source-only leaves; C0-a bypasses association-sensitive
// result caches, so no assertion below claims a new-mode result-cache hit.
fn c0_shared_check(view: &BoundBodyQueryView, from: C0SharedPose, to: C0SharedPose,
    t: f64, blocked: bool, source: Option<OccupancySource>, fine_z: Option<i32>) -> C0SharedResults {
    let mut witness = None;
    let pose = view.pose_at(from, t, &mut witness);
    let pose_witness = witness;
    let projected = view.project_from_to_samples(from, [to.0[0], to.0[1]], 0.0,
        [t, t], &mut witness);
    let projected_pose = projected.as_ref().ok().copied();
    let projection = projected.map(|_| ());
    let projection_witness = witness;
    let motion = view.motion_on_segment(from, to, [t, t], &[], &mut witness);
    let results = [(pose, pose_witness), (projection, projection_witness), (motion, witness)];
    eprintln!("C0_SHARED_CHECK from={from:?} to={to:?} sample={t} blocked={blocked} projected={projected_pose:?} results={results:?}");
    if !blocked { assert_eq!(projected_pose, Some(to)); }
    for (result, witness) in &results {
        if blocked {
            assert_eq!(*result, Err(SurfaceError::InsufficientClearance));
            let BodyRejection::Occupancy { source: actual_source, cell, .. } =
                witness.expect("actual occupancy must retain its original witness").rejection;
            if let Some(source) = source { assert_eq!(actual_source, source); }
            if let Some(z) = fine_z {
                assert!(matches!(actual_source, OccupancySource::Fine | OccupancySource::FineFringe));
                assert_eq!(cell[2], z, "the other fine member, not the selected floor, collides");
            }
        } else {
            assert_eq!(*result, Ok(()));
            assert!(witness.is_none(), "success must clear the prior rejection slot");
        }
    }
    results
}

// Read the actual context-aware contact leaf over every coarse tile touched by
// the unchanged actual footprint. This is an oracle call into the original
// kernel, not a replacement selector built in the test.
fn c0_shared_footprint_members(map: &SurfaceSnapshot, actual: C0SharedPose,
    sample: &GroundSample, selected: i32) {
    eprintln!("C0_SHARED_FOOTPRINT_INPUT pose={actual:?} selected={selected} reference={:?} state={:?} sources={:?}",
        sample.reference_origin, sample.query_state, sample.reference_sources);
    assert_eq!(actual.1, [0.0, 0.0, 0.0, 1.0]);
    let envelope = map.body_envelope().unwrap();
    let footprint: Vec<_> = envelope.footprint_xy_m.iter()
        .map(|p| [actual.0[0] + p[0], actual.0[1] + p[1]]).collect();
    let polygon = PolygonQuery::new(&footprint);
    let r = rotation(actual.1).unwrap();
    let prism = PrismQuery::new(envelope, r, actual.0);
    let coarse = f64::from(map.config().voxel_size);
    let fine = f64::from(map.source_voxel_size_m());
    let low: [i32; 2] = std::array::from_fn(|axis| (footprint.iter()
        .map(|p| p[axis]).fold(f64::INFINITY, f64::min) / coarse).floor() as i32);
    let high: [i32; 2] = std::array::from_fn(|axis| (footprint.iter()
        .map(|p| p[axis]).fold(f64::NEG_INFINITY, f64::max) / coarse).floor() as i32);
    let observations = map.observations.as_ref().unwrap();
    let mut seen = std::collections::BTreeSet::new();
    let mut evidence = Vec::new();
    for x in low[0]..=high[0] { for y in low[1]..=high[1] {
        let rows = map.fine_ground_columns_in_context(Some((x, y, 0)), &polygon,
            &prism, actual.0, r[2], x, y, QuerySample::Ground(sample)).unwrap();
        for row in rows {
            assert!(polygon.intersects(row.xy[0], row.xy[1], fine));
            let records: Vec<_> = observations.ground_layers(row.xy).collect();
            if records.is_empty() {
                assert!(row.contacts.as_slice().is_empty(), "outside fixed source has no new members");
                continue;
            }
            // Adjacent coarse tiles can enumerate a shared fine edge because
            // quantized coarse/fine sizes need not divide exactly. Every copy
            // must still preserve the same exact membership.
            seen.insert(row.xy);
            assert_eq!(row.height, Some(sample.reference_origin[2]), "actual fine column {:?}", row.xy);
            assert_eq!(row.contacts.as_slice(), &[[selected, selected]], "actual fine column {:?}", row.xy);
            assert!(!row.permits_contact(1 - selected));
            let record = records.iter().find(|record| record.anchor[2] == selected)
                .expect("selected measured floor exists in this actual fine column");
            assert_eq!(record.members.as_slice(), &[selected]);
            evidence.push((row.xy, row.height, row.contacts.as_slice().to_vec()));
        }
    }}
    assert!(seen.len() > 1 && seen.iter().any(|xy| *xy != [0, 0]),
        "center anchor alone is not the footprint oracle");
    // Every source-owned fine column intersecting the footprint was visited.
    for (xy, _) in observations.navigation_columns() {
        if polygon.intersects(xy[0], xy[1], fine) { assert!(seen.contains(&xy)); }
    }
    eprintln!("C0_SHARED_FOOTPRINT pose={actual:?} selected={selected} rows={evidence:?}");
}

#[test]
fn ground_context_exact_members_keep_same_coarse_obstacle() {
    let mut envelope = model();
    envelope.z_bounds_m = [-0.4, 0.2];
    let floor = with_measured_floor(layered_map(&[[0, 0]]), 0.025)
        .with_body_envelope(envelope.clone()).unwrap();
    let upper = with_measured_floor(layered_map(&[[0, 0], [1, 1]]), 0.025)
        .with_body_envelope(envelope).unwrap();
    let a = ([0.025, 0.025, 0.425], [0.0, 0.0, 0.0, 1.0]);
    let b = ([0.125, 0.025, 0.425], a.1);
    let q = f64::from(upper.source_voxel_size_m());
    let coarse = f64::from(upper.config().voxel_size);
    assert_eq!(((0.5 * q) / coarse).floor() as i32, 0);
    assert_eq!(((1.5 * q) / coarse).floor() as i32, 0);
    assert!(upper.observations.as_ref().unwrap().get([0, 0, 1]).is_some());
    assert!(!floor.observations.as_ref().unwrap().get([0, 0, 1]).is_some());
    for (label, map, blocked) in [("floor", &floor, false), ("same-coarse upper", &upper, true)] {
        for anchored in [true, false] {
            let reference = if anchored {
                c0_shared_constant(c0_shared_measured_sample(map, [0.025, 0.025], 0.025, [0, 0, 0]))
            } else { BodyQueryReference::Unanchored };
            let view = map.bind_body_query(reference);
            eprintln!("C02 arm={label} Ground={anchored} A={a:?} B={b:?}");
            let first = c0_shared_check(&view, a, b, 0.0, blocked, None, blocked.then_some(1));
            assert_eq!(c0_shared_check(&view, a, b, 0.0, blocked, None, blocked.then_some(1)), first);
        }
    }

    // Keep the original coarse geometry, but admit only the fixed high fine
    // return. The vanished lower anchor is a carried reference, never current
    // Observed evidence and never permission to exempt an absent source member.
    let mut partial_observations = SurfaceObservations::new(0.05, 32).unwrap();
    partial_observations.replace_block([0, 0, 0], std::collections::BTreeMap::from([
        ([0, 0, 1], SurfaceObservation { observed_ns: 1000, hits: 1 }),
    ])).unwrap();
    let partial = upper.with_observations(Some(Arc::new(partial_observations)));
    assert!(Arc::ptr_eq(&upper.data, &partial.data));
    assert!(!partial.observations.as_ref().unwrap().is_complete_geometry_source());
    assert!(partial.observations.as_ref().unwrap().get([0, 0, 0]).is_none());
    assert!(partial.observations.as_ref().unwrap().ground_layers([0, 0]).next().is_none());
    let (current, current_native) = partial.query_ground_with_native([0.025, 0.025], 0.025);
    eprintln!("C02 partial current={current:?} current_native={current_native:?}");
    assert_eq!(current.state, GroundQueryState::LegacyGeometry);
    assert!(current.height.is_some());
    assert_ne!(current.height, Some(0.025));
    assert!(matches!(current_native.as_slice(),
        [NativeGroundSource::LegacyFineRun { xy: [0, 0], first: 1, last: 1 }]));
    // Cold Legacy controls use equal geometry/facts but distinct private
    // owner/cache identities. This partial fixture replaces all measured layers
    // with one high run; the original full entry can classify its coarse cells
    // as mechanical contact. The older unassigned-leaf test uses a different
    // body/height and bypasses that assigned-patch path, so it is not an oracle
    // requiring this complete entry to reject. Preserve both original failures
    // and the cold Legacy values in the C0-a execution record.
    for method in ["pose", "projection", "motion"] {
        let cold = partial.fork_for_planning();
        assert_eq!(cold.data.occupied_columns, partial.data.occupied_columns);
        assert_eq!(cold.data.height_bounds, partial.data.height_bounds);
        assert_eq!(cold.observations.as_ref().unwrap().navigation_columns().collect::<Vec<_>>(),
            partial.observations.as_ref().unwrap().navigation_columns().collect::<Vec<_>>());
        assert_eq!(cold.ground_evidence, partial.ground_evidence);
        eprintln!("C02_COLD_ID method={method} data_same_arc={} observations_same_arc={} coarse_columns={} fine_columns={:?} complete={} measured={}",
            Arc::ptr_eq(&cold.data, &partial.data),
            Arc::ptr_eq(cold.observations.as_ref().unwrap(), partial.observations.as_ref().unwrap()),
            cold.data.occupied_columns.len(), cold.observations.as_ref().unwrap().navigation_columns().collect::<Vec<_>>(),
            cold.observations.as_ref().unwrap().is_complete_geometry_source(),
            cold.observations.as_ref().unwrap().has_measured_ground_layers());
        let mut witness = None;
        match method {
            "pose" => {
                let result = cold.validate_body_pose_with_witness(a.0, a.1, &mut witness);
                eprintln!("C02_COLD_LEGACY pose={result:?} witness={witness:?}");
                assert_eq!(result, Ok(()));
            }
            "projection" => {
                let result = cold.body_pose_after_motion_with_witness(a, [b.0[0], b.0[1]], 0.0, &mut witness);
                eprintln!("C02_COLD_LEGACY projection={result:?} witness={witness:?}");
                assert!(result.is_ok());
            }
            _ => {
                let result = cold.validate_body_motion_with_witness(a, b, &[], &mut witness);
                eprintln!("C02_COLD_LEGACY motion={result:?} witness={witness:?}");
                assert_eq!(result, Ok(()));
            }
        }
        assert!(witness.is_none());
    }
    let cold = partial.fork_for_planning();
    let model = cold.body_envelope().unwrap(); let r = rotation(a.1).unwrap();
    let vertices: Vec<_> = model.footprint_xy_m.iter().flat_map(|p|
        model.z_bounds_m.map(|z| world(r, a.0, [p[0], p[1], z]))).collect();
    let polygon_points = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
    let polygon = PolygonQuery::new(&polygon_points);
    let prism = PrismQuery::new(model, r, a.0);
    let centre = cold.locate_support_from_body(a.0, a.1, model.body_to_support_z_m);
    eprintln!("C02_COLD_LEAF centre={centre:?} model={model:?} actual={a:?}");
    if let Ok(centre) = centre {
        let size = f64::from(cold.config().voxel_size);
        let radius = ((vertices.iter().map(|p| (p[0]-centre.surface_xyz[0]).hypot(p[1]-centre.surface_xyz[1]))
            .fold(0.0, f64::max)+size*2.0)/size).ceil()*size;
        let patch = cold.ground_patch(centre, radius);
        for (x,y) in [(-6,-2),(-5,-2),(0,0)] {
            let ground = patch.get(&(x,y)).copied();
            let rows = cold.fine_ground_columns(ground, &polygon, &prism, a.0, r[2], x, y);
            let row_values = rows.as_ref().map(|rows| rows.iter()
                .map(|row| (row.xy, row.height, row.contacts.as_slice().to_vec())).collect::<Vec<_>>());
            eprintln!("C02_COLD_LEAF column={:?} patch_ground={ground:?} coarse_levels={:?} rows={row_values:?}",
                [x,y], cold.data.occupied_columns.get(&(x,y)));
            if let Some(ground) = ground {
                let uncertainty = f64::from(cold.source_voxel_size_m())+model.support_uncertainty_m;
                let contact = model.max_step_up_m.max(model.max_step_down_m)+uncertainty;
                let plane = polygon.contact_plane(a.0, r[2], x, y, size, cold.ground_cell_position(ground)[2],
                    (-model.body_to_support_z_m/r[2][2]+contact+1e-6).min(uncertainty));
                let support = plane-model.body_to_support_z_m/r[2][2];
                let height = cold.footprint_ground_height(ground,&polygon,&prism,support,contact,rows.as_deref());
                eprintln!("C02_COLD_CONTACT column={:?} plane={plane:?} support={support:?} contact={contact:?} height={height:?} predicate={:?} bounds={:?} actual_coarse_intersection={:?}",
                    [x,y], height.as_ref().ok().map(|&h| height_rejection(plane+model.z_bounds_m[0]/r[2][2],support,plane,contact,h,uncertainty)),
                    cold.data.height_bounds.get(&ground), cold.data.height_bounds.get(&ground).map(|b|
                        prism.intersects([f64::from(x)*size,f64::from(y)*size,f64::from(b[0])],
                            [f64::from(x+1)*size,f64::from(y+1)*size,f64::from(b[1])])));
            }
        }
    }
    for anchored in [true, false] {
        let reference = if anchored {
            let mut carried = c0_shared_measured_sample(&upper, [0.025, 0.025], 0.025, [0, 0, 0]);
            carried.query_state = GroundQueryState::Unknown;
            c0_shared_constant(carried)
        } else { BodyQueryReference::Unanchored };
        let view = partial.bind_body_query(reference);
        // Ground carried has no current lower member and still rejects via the
        // original coarse fallback. Unanchored retains the old mechanical
        // contact acceptance, but its projection keeps actual Z=.425 rather
        // than borrowing Legacy's navigation-height adjustment to about .4.
        let source = anchored.then_some(OccupancySource::CoarseFallback);
        let first = c0_shared_check(&view, a, b, 0.0, anchored, source, None);
        assert_eq!(c0_shared_check(&view, a, b, 0.0, anchored, source, None), first);
    }

    // Independent live geometry must survive even when all persistent fine
    // occupancy is precisely the allowed lower measured floor.
    let recent = floor.with_recent_obstacle_points(1, 100, 0, &[],
        [(0, [0.025, 0.025, 0.35])]).unwrap();
    for anchored in [true, false] {
        let reference = if anchored {
            c0_shared_constant(c0_shared_measured_sample(&recent, [0.025, 0.025], 0.025, [0, 0, 0]))
        } else { BodyQueryReference::Unanchored };
        let view = recent.bind_body_query(reference);
        let first = c0_shared_check(&view, a, b, 0.0, true, Some(OccupancySource::Recent), None);
        assert_eq!(c0_shared_check(&view, a, b, 0.0, true, Some(OccupancySource::Recent), None), first);
    }
}

#[test]
fn ground_context_recent_floor_box_and_refresh_pair() {
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, block_key, local_key};
    // Same isolated-floor fixture as the pre-existing guide/body test above.
    let key = [5, 0, -1];
    let point = [0.275, 0.025, -0.025];
    let mut cfg = scene(&[], false).config().clone();
    cfg.wall_clearance_m = 0.4;
    let mut envelope = model();
    envelope.z_bounds_m = [-0.45, 0.25];
    let mut observations = SurfaceObservations::new(0.05, 32).unwrap();
    observations.replace_block(block_key(key), std::collections::BTreeMap::from([
        (key, SurfaceObservation { observed_ns: 100, hits: 1 }),
    ])).unwrap();
    let local = local_key(key);
    observations.replace_ground_layers(block_key(key), Arc::new(LayerObservations::from([
        (local, LayerObservation {
            members: vec![local], normal: [0.0, 0.0, 1.0], plane_origin: point,
            samples: vec![LayerSample { source: Default::default(), point,
                source_sequence: 1, source_timestamp_ns: 100, point_index: 0 }],
        }),
    ]))).unwrap();
    observations.mark_complete_geometry_source();
    let map = SurfaceSnapshot::from_points(
        &[(point[0] as f32, point[1] as f32, point[2] as f32)], cfg, 0.05)
        .unwrap().with_body_envelope(envelope).unwrap()
        .with_observations(Some(Arc::new(observations)));
    let a = ([0.025, 0.025, 0.4], [0.0, 0.0, 0.0, 1.0]);
    // Stationary projection/motion are deliberate here: C07's fixed actual pose
    // stays identical while the recent owner and independent association change.
    let bounds = [[-0.5, -0.5], [0.5, 0.5]];
    assert_eq!(map.locate_support_from_body(a.0, a.1, 0.4), Err(SurfaceError::MissingSupport));
    let (center, center_native) = map.query_ground_with_native([a.0[0], a.0[1]], -0.025);
    assert_eq!(center.state, GroundQueryState::Unknown);
    assert_eq!(center.height, None);
    assert_eq!(center.normal, None);
    assert!(center_native.is_empty());
    let mut carried = c0_shared_measured_sample(&map, [point[0], point[1]], -0.025, key);
    carried.query_state = GroundQueryState::Unknown;
    let reference = Arc::new(LocalGroundReference { first: carried, changes: vec![] });
    assert_eq!(reference.first.reference_origin[2], -0.025, "this source has no ground zero");
    assert_eq!(reference.first.query_state, GroundQueryState::Unknown);
    let floor = map.with_recent_obstacle_points(1, 100, 0, &[0], [(0, point)]).unwrap();
    assert!(floor.recent_obstacle_hints(bounds, a).unwrap().is_empty());
    for indices in [&[][..], &[0][..]] {
        let blocked = floor.with_recent_obstacle_points(2, 200, 0, indices,
            [(0, [point[0], point[1], 0.35])]).unwrap();
        let restored = blocked.with_recent_obstacle_points(3, 300, 0, &[0], [(0, point)]).unwrap();
        let blocked_again = restored.with_recent_obstacle_points(4, 400, 0, indices,
            [(0, [point[0], point[1], 0.35])]).unwrap();
        let cleared = blocked_again.clear_recent_obstacles();
        assert_eq!(blocked.recent_obstacle_hints(bounds, a).unwrap().len(), 1);
        assert!(restored.recent_obstacle_hints(bounds, a).unwrap().is_empty());
        assert_eq!(blocked_again.recent_obstacle_hints(bounds, a).unwrap().len(), 1);
        assert!(cleared.recent_obstacle_hints(bounds, a).unwrap().is_empty());
        for source in [&floor, &blocked, &restored, &blocked_again, &cleared] {
            assert!(Arc::ptr_eq(&map.data, &source.data));
            assert!(Arc::ptr_eq(map.observations.as_ref().unwrap(), source.observations.as_ref().unwrap()));
            let query = source.query_ground([a.0[0], a.0[1]], -0.025);
            assert_eq!(query.state, GroundQueryState::Unknown);
            assert_eq!(query.height, None);
            assert_eq!(query.normal, None);
        }
        for order in [[false, true], [true, false]] {
            for (label, source, is_blocked) in [
                ("a", &floor, false), ("b", &blocked, true),
                ("a refreshed", &restored, false), ("b refreshed", &blocked_again, true),
                ("recent cleared", &cleared, false),
            ] {
                for anchored in order {
                    let binding = if anchored { BodyQueryReference::Ground(reference.clone()) }
                        else { BodyQueryReference::Unanchored };
                    let view = source.bind_body_query(binding);
                    eprintln!("C07 arm={label} indices={indices:?} Ground-carried={anchored} pose={a:?}");
                    let expected_source = is_blocked.then_some(OccupancySource::Recent);
                    let first = c0_shared_check(&view, a, a, 0.0, is_blocked, expected_source, None);
                    assert_eq!(c0_shared_check(&view, a, a, 0.0, is_blocked, expected_source, None), first);
                }
            }
        }
    }
}

#[test]
fn ground_context_same_pose_distinct_measured_layer_cache_pair() {
    for order in [[1, 0, 1], [0, 1, 0]] {
        // Each order starts with a fresh source owner. Repeated calls share only
        // that owner's caches; no private association cache is being enabled.
        let map = same_coarse_measured_map(&[0, 1]);
        let q = f64::from(map.source_voxel_size_m());
        let heights = [0.5 * q, 1.5 * q];
        let a = ([0.025, 0.025, heights[1] + 0.4], [0.0, 0.0, 0.0, 1.0]);
        let b = ([0.125, 0.025, heights[1] + 0.4], a.1);
        let low = map.locate_support([a.0[0], a.0[1], heights[0]]).unwrap();
        let high = map.locate_support([a.0[0], a.0[1], heights[1]]).unwrap();
        assert_eq!(low.cell, high.cell);
        assert_eq!(low.cell, (0, 0, 0));
        assert_ne!(low.surface_xyz[2], high.surface_xyz[2]);
        let views = [0, 1].map(|level| {
            let sample = c0_shared_measured_sample(&map, [a.0[0], a.0[1]],
                heights[level], [0, 0, level as i32]);
            c0_shared_footprint_members(&map, a, &sample, level as i32);
            c0_shared_footprint_members(&map, b, &sample, level as i32);
            map.bind_body_query(c0_shared_constant(sample))
        });
        let mut baseline: [Option<C0SharedResults>; 2] = [None, None];
        for pass in 0..2 {
            for selected in order {
                let blocked = selected == 0;
                eprintln!("C08 pass={pass} selected={selected} A={a:?} B={b:?} association-cache=bypassed");
                let result = c0_shared_check(&views[selected], a, b, 0.0, blocked, None,
                    blocked.then_some(1));
                if let Some(expected) = &baseline[selected] { assert_eq!(&result, expected); }
                else { baseline[selected] = Some(result); }
            }
        }
        // Explicit stale-slot test across views, with unchanged actual geometry.
        let mut witness = None;
        assert_eq!(views[0].pose_at(a, 0.0, &mut witness), Err(SurfaceError::InsufficientClearance));
        assert!(witness.is_some());
        assert_eq!(views[1].pose_at(a, 0.0, &mut witness), Ok(()));
        assert!(witness.is_none());

        // Local contact is allowed to choose its existing mechanical explanation;
        // map ground facts do not promote this explicitly Unanchored request.
        let unanchored = map.bind_body_query(BodyQueryReference::Unanchored);
        let mut first = None;
        for pass in 0..2 {
            let mut witness = None;
            let pose = unanchored.pose_at(a, 0.0, &mut witness);
            let pose_witness = witness;
            let projected = unanchored.project_from_to_samples(a, [b.0[0], b.0[1]], 0.0,
                [0.0, 0.0], &mut witness);
            if let Ok(actual) = &projected { assert_eq!(*actual, b); }
            let projection_witness = witness;
            let motion = unanchored.motion_on_segment(a, b, [0.0, 0.0], &[], &mut witness);
            let results = [(pose, pose_witness), (projected.map(|_| ()), projection_witness), (motion, witness)];
            eprintln!("C08 Unanchored pass={pass} results={results:?}; association input=None");
            for (result, witness) in &results {
                match result {
                    Ok(()) => assert!(witness.is_none()),
                    Err(SurfaceError::InsufficientClearance) => assert!(witness.is_some()),
                    other => panic!("Unanchored must not invent known terrain incompatibility: {other:?}"),
                }
            }
            if let Some(expected) = &first { assert_eq!(&results, expected); }
            else { first = Some(results); }
        }
    }
}

#[test]
fn ground_context_same_view_explicit_sample_cold_warm_pair() {
    for order in [[0.0, 0.5, 1.0], [0.5, 0.0, 0.5]] {
        let map = same_coarse_measured_map(&[0, 1]);
        let q = f64::from(map.source_voxel_size_m());
        let heights = [0.5 * q, 1.5 * q];
        let a = ([0.025, 0.025, heights[1] + 0.4], [0.0, 0.0, 0.0, 1.0]);
        let b = ([0.125, 0.025, heights[1] + 0.4], a.1);
        let sample = |level: usize| {
            let sample = c0_shared_measured_sample(&map,
                [a.0[0], a.0[1]], heights[level], [0, 0, level as i32]);
            c0_shared_footprint_members(&map, a, &sample, level as i32);
            sample
        };
        // A diagnostic local connection: right-continuous H / L / H. This is
        // input/cursor validation, not a claim of a robot changing floors in place.
        let view = map.bind_body_query(BodyQueryReference::Ground(Arc::new(LocalGroundReference {
            first: sample(1), changes: vec![(0.25, sample(0)), (0.75, sample(1))],
        })));
        let mut baseline: [Option<C0SharedResults>; 2] = [None, None];
        for pass in 0..2 {
            for t in order {
                let low = t >= 0.25 && t < 0.75;
                let result = c0_shared_check(&view, a, b, t, low, None, low.then_some(1));
                eprintln!("C11 pass={pass} t={t} low={low} association-cache=bypassed");
                let index = usize::from(low);
                if let Some(expected) = &baseline[index] { assert_eq!(&result, expected); }
                else { baseline[index] = Some(result); }
            }
        }
        // Exact changes are right-continuous, including stale witness clearing.
        let mut witness = None;
        assert_eq!(view.pose_at(a, 0.25, &mut witness), Err(SurfaceError::InsufficientClearance));
        assert!(witness.is_some());
        assert_eq!(view.pose_at(a, 0.75, &mut witness), Ok(()));
        assert!(witness.is_none());
        assert_eq!(view.project_from_to_samples(a, [a.0[0], a.0[1]], 0.0,
            [0.0, 0.0], &mut witness), Ok(a));
        assert!(witness.is_none());
        assert_eq!(view.motion_on_segment(a, a, [0.0, 0.0], &[], &mut witness), Ok(()));
        assert!(witness.is_none());
        // Different target sample cannot take the old zero-XY/yaw early return.
        let changed = view.project_from_to_samples(a, [a.0[0], a.0[1]], 0.0,
            [0.0, 0.5], &mut witness);
        eprintln!("C11 zero-XY changed-target result={changed:?} witness={witness:?}");
        // The unchanged known-terrain edge rejects a nonzero H->L change in
        // the same fine XY before target occupancy; no occupancy witness exists.
        assert_eq!(changed, Err(SurfaceError::UntraversableSegment));
        assert!(witness.is_none());
        // Both endpoints resolve to H. The interior L change must still be
        // consumed. Original terrain rejection precedes regular/event occupancy
        // in these fixed full-motion inputs; it does not prove event execution.
        for to in [a, b] {
            let mut first = None;
            for _ in 0..2 {
                let result = view.motion_on_segment(a, to, [0.0, 1.0], &[], &mut witness);
                eprintln!("C11 H/L/H segment from={a:?} to={to:?} result={result:?} witness={witness:?}");
                assert_eq!(result, Err(SurfaceError::UntraversableSegment));
                assert!(witness.is_none());
                let actual = (result, witness);
                if let Some(expected) = &first { assert_eq!(&actual, expected); }
                else { first = Some(actual); }
            }
        }
        let mut checker = view.geometry_checker(&[]);
        assert_eq!(checker(a, a, [0.0, 1.0], &mut witness), Err(SurfaceError::UntraversableSegment));
        assert!(witness.is_none());
        assert_eq!(checker(a, a, [0.0, 0.0], &mut witness), Ok(()));
        assert!(witness.is_none());

        // Separate original terrain-leaf replay using the real bound segment
        // and original fraction/height helpers. This is not a main-call trace.
        for (label, to, interval) in [("projection", a, [0.0, 0.5]),
            ("stationary", a, [0.0, 1.0]), ("translating", b, [0.0, 1.0])] {
            let context = view.segment_for_diagnostic(interval);
            let size = f64::from(map.source_voxel_size_m());
            let mut fractions = translation::reference_grid_fractions(
                [a.0[0], a.0[1], 0.0], [to.0[0], to.0[1], 0.0], size).unwrap();
            context.append_changes(&mut fractions);
            fractions.sort_by(f64::total_cmp); fractions.dedup();
            let samples: Vec<_> = fractions.iter().map(|&s| {
                let xy = [a.0[0] + (to.0[0]-a.0[0])*s, a.0[1] + (to.0[1]-a.0[1])*s];
                let sample = context.at(s).ground().unwrap();
                let height = sample.height_at(xy);
                eprintln!("C11_TERRAIN_LEAF label={label} s={s:?} t={:?} xy={xy:?} reference={height:?} actual_height={:?}",
                    interval[0]+s*(interval[1]-interval[0]), map.known_terrain_transition_top(xy, height));
                [xy[0], xy[1], height]
            }).collect();
            let model = map.body_envelope().unwrap();
            let terrain = map.validate_known_reference_samples(samples, size,
                model.max_step_up_m, model.max_step_down_m, model.support_uncertainty_m);
            eprintln!("C11_TERRAIN_LEAF_RESULT label={label} result={terrain:?}");
            assert_eq!(terrain, Err(SurfaceError::UntraversableSegment));
        }
        // Independent contact-event qualification: the complete motion may be
        // rejected by the earlier terrain pass. Invoke the unchanged real event
        // kernel with this same bound segment and its real pose checker.
        for to in [a, b] {
            let context = view.segment_for_diagnostic([0.0, 1.0]);
            let mut event_witness = None;
            let mut calls = Vec::new();
            let event = map.validate_body_contact_events_in_context(a, to, context, |s| {
                let actual = interpolated_body_pose(a, to, s);
                let result = view.pose_at(actual, s, &mut event_witness);
                eprintln!("C11_EVENT_LEAF s={s:?} t={s:?} actual={actual:?} result={result:?} witness={event_witness:?}");
                calls.push((s, actual, result.clone(), event_witness));
                result
            });
            eprintln!("C11_EVENT_LEAF_RESULT to={to:?} calls={calls:?} result={event:?} witness={event_witness:?}");
            assert_eq!(event, Err(SurfaceError::InsufficientClearance));
            let (s, actual_pose, result, last_witness) = calls.last().unwrap();
            assert_eq!(*s, 0.25, "right-continuous L association reaches the event checker");
            assert_eq!(*actual_pose, interpolated_body_pose(a, to, 0.25));
            assert_eq!(*result, Err(SurfaceError::InsufficientClearance));
            assert!(calls[..calls.len()-1].iter().all(|(s, _, result, witness)|
                *s < 0.25 && *result == Ok(()) && witness.is_none()));
            let actual = event_witness.expect("independent event leaf has a real occupancy witness");
            assert_eq!(Some(actual), *last_witness);
            assert_eq!(actual.pose, *actual_pose);
            assert_eq!(actual.rejection, BodyRejection::Occupancy {
                cell: if to == a { [-11, -3, 1] } else { [-10, -3, 1] },
                source: OccupancySource::Fine, z_bounds: [q, 2.0*q],
                ground_height: Some(heights[0]),
            });
        }
    }
}

// Fixed C0-a diagnostic: continuous height changes inside one fine cell are
// motion on one real measured plane, not a stationary change of selected floor.
#[test]
fn ground_context_same_observed_plane_subcell_motion_diagnostic() {
    let map = parallel_measured_slope_map(0.045, false, None);
    let fine = f64::from(map.source_voxel_size_m());
    let from_xy = [0.01, 0.025];
    let to_xy = [0.02, 0.025];
    let plane_height = |xy: [f64; 2]| 0.045 - 0.4 * xy[0] + 0.06;
    let observations = map.observations.as_ref().unwrap();
    let fractions = translation::reference_grid_fractions(
        [from_xy[0], from_xy[1], 0.0], [to_xy[0], to_xy[1], 0.0], fine).unwrap();
    let mut previous: Option<([f64; 2], f64, &ObservedGroundLayer)> = None;
    for &s in &fractions {
        let xy = std::array::from_fn(|i| from_xy[i] + (to_xy[i] - from_xy[i]) * s);
        let reference = plane_height(xy);
        let column = xy.map(|v| (v / fine).floor() as i32);
        let (query, native) = map.query_ground_with_native(xy, reference);
        let selected = observations.ground_layers(column).min_by(|a, b|
            (a.height_at(xy).unwrap_or(f64::INFINITY) - reference).abs().total_cmp(
                &(b.height_at(xy).unwrap_or(f64::INFINITY) - reference).abs())).unwrap();
        let height = selected.height_at(xy).unwrap();
        let top = map.known_terrain_transition_top(xy, reference);
        eprintln!("C12_SOURCE s={s:?} xy={xy:?} fine={fine:?} column={column:?} reference={reference:?} query={query:?} native={native:?} anchor={:?} members={:?} normal={:?} origin={:?} top={top:?}",
            selected.anchor, selected.members, selected.normal, selected.plane_origin);
        assert_eq!(column, [0, 0]);
        assert_eq!(query.state, GroundQueryState::Observed);
        assert_eq!(native, vec![NativeGroundSource::ObservedFine { anchor: [0, 0, 1] }]);
        assert_eq!(selected.anchor, [0, 0, 1]);
        assert_eq!(selected.members.as_slice(), &[1]);
        assert_eq!(query.height, Some(height));
        assert_eq!(top, Some(height));
        assert!(selected.normal[2] >= f64::from(map.config().max_slope_deg).to_radians().cos());
        if let Some((old_xy, old_height, old_layer)) = previous {
            let actual_run = (xy[0] - old_xy[0]).hypot(xy[1] - old_xy[1]);
            let rise = height - old_height;
            let same_record = std::ptr::eq(old_layer, selected);
            eprintln!("C12_EDGE from={old_xy:?} to={xy:?} same_record={same_record} actual_run={actual_run:?} cell_run=0 rise={rise:?} max_slope_deg={:?}", map.config().max_slope_deg);
            assert!(same_record);
            assert!(actual_run > 0.0 && rise.abs() > 1e-6);
            assert!(rise.abs() <= actual_run * f64::from(map.config().max_slope_deg).to_radians().tan() + 1e-6);
        }
        previous = Some((xy, height, selected));
    }
    assert_eq!(fractions, vec![0.0, 0.5, 1.0]);
    let (height, attitude) = map.measured_contact_frame(from_xy, plane_height(from_xy), 0.0).unwrap();
    let r = rotation(attitude).unwrap();
    let offset = map.body_envelope().unwrap().body_to_support_z_m / r[2][2];
    let from = ([from_xy[0], from_xy[1], height + offset], attitude);
    let to = ([to_xy[0], to_xy[1], plane_height(to_xy) + offset], attitude);
    let (query, native) = map.query_ground_with_native(from_xy, height);
    let reference = c0_shared_constant(GroundSample {
        reference_origin: [from_xy[0], from_xy[1], height],
        reference_normal: query.normal,
        query_state: query.state,
        reference_sources: native,
    });
    // Private disposable caches start cold. Forks retain identical geometry and
    // source records, but are deliberately not the same snapshot-owner identity.
    let legacy_source = map.fork_for_planning();
    let ground_source = map.fork_for_planning();
    let legacy = legacy_source.body_query();
    let ground = ground_source.bind_body_query(reference);
    let mut witness = None;
    let legacy_from = (legacy_source.validate_body_pose_with_witness(from.0, from.1, &mut witness), witness);
    let legacy_to = (legacy_source.validate_body_pose_with_witness(to.0, to.1, &mut witness), witness);
    let legacy_projection = (legacy.body_pose_after_motion_with_witness(from, to_xy, 0.0, &mut witness), witness);
    let legacy_motion = (legacy.validate_body_motion_with_witness(from, to, &[], &mut witness), witness);
    let ground_from = (ground.pose_at(from, 0.0, &mut witness), witness);
    let ground_to = (ground.pose_at(to, 1.0, &mut witness), witness);
    let ground_projection = (ground.project_from_to_samples(from, to_xy, 0.0, [0.0, 1.0], &mut witness), witness);
    let ground_motion = (ground.motion_on_segment(from, to, [0.0, 1.0], &[], &mut witness), witness);
    eprintln!("C12_POSES from={from:?} to={to:?} rotation={r:?} envelope={:?}", map.body_envelope());
    eprintln!("C12_LEGACY from={legacy_from:?} to={legacy_to:?} projection={legacy_projection:?} motion={legacy_motion:?}");
    eprintln!("C12_GROUND from={ground_from:?} to={ground_to:?} projection={ground_projection:?} motion={ground_motion:?}");
    // Check all fixed physical premises only after recording every full entry.
    for result in [legacy_from, legacy_to, ground_from, ground_to] {
        assert_eq!(result, (Ok(()), None), "fixed tilted endpoint body premise");
    }
    assert!(legacy_projection.0.is_ok(), "original Legacy projection control");
    assert_eq!(legacy_projection.1, None);
    assert_eq!(legacy_motion, (Ok(()), None), "original Legacy motion control");
    assert!(ground_projection.0.is_ok(), "one real legal measured plane must permit subcell movement");
    assert_eq!(ground_projection.1, None);
    assert_eq!(ground_motion, (Ok(()), None));
}

#[test]
fn ground_context_carried_unknown_keeps_actual_height_on_observed_subcell_slope() {
    let map = parallel_measured_slope_map(0.045, false, None);
    let from_xy = [0.01, 0.025];
    let to_xy = [0.02, 0.025];
    let (height, attitude) = map.measured_contact_frame(from_xy, 0.101, 0.0).unwrap();
    let (query, sources) = map.query_ground_with_native(from_xy, height);
    assert_eq!(query.state, GroundQueryState::Observed);
    assert_eq!(sources, vec![NativeGroundSource::ObservedFine { anchor: [0, 0, 1] }]);
    let sample = GroundSample { reference_origin: [from_xy[0], from_xy[1], height],
        reference_normal: query.normal, query_state: GroundQueryState::Unknown,
        reference_sources: sources };
    let reference = Arc::new(LocalGroundReference { first: sample.clone(), changes: vec![] });
    let view = map.bind_body_query(BodyQueryReference::Ground(reference.clone()));
    let r = rotation(attitude).unwrap();
    let from = ([from_xy[0], from_xy[1], height + map.body_envelope().unwrap().body_to_support_z_m / r[2][2]], attitude);
    let to = ([to_xy[0], to_xy[1], from.0[2]], attitude);
    let mut witness = None;
    let from_result = (view.pose_at(from, 0.0, &mut witness), witness);
    let to_result = (view.pose_at(to, 1.0, &mut witness), witness);
    let projection = (view.project_from_to_samples(from, to_xy, 0.0, [0.0, 1.0], &mut witness), witness);
    let motion = (view.motion_on_segment(from, to, [0.0, 1.0], &[], &mut witness), witness);
    let selected_a = map.known_terrain_transition_sample(from_xy, sample.height_at(from_xy)).unwrap();
    let selected_b = map.known_terrain_transition_sample(to_xy, sample.height_at(to_xy)).unwrap();
    let same_record = std::ptr::eq(selected_a.1.unwrap(), selected_b.1.unwrap());
    eprintln!("C12_CARRIED reference={sample:?} selected_heights={:?} same_record={same_record} from={from:?} to={to:?} from_result={from_result:?} to_result={to_result:?} projection={projection:?} motion={motion:?}", [selected_a.0, selected_b.0]);
    assert!(same_record);
    assert!(selected_b.0 < selected_a.0, "current source really has a descending plane");
    assert_eq!(from_result, (Ok(()), None));
    assert_eq!(to_result, (Ok(()), None));
    assert_eq!(projection, (Ok(to), None));
    assert_eq!(motion, (Ok(()), None));
    assert_eq!(to.0[2].to_bits(), from.0[2].to_bits(), "Unknown reference carries actual height without a new terrain delta");
    assert_eq!(reference.first, sample, "factual checking must not promote the carried reference to Observed");
}

#[test]
fn ground_context_subcell_distinct_observed_anchors_keep_original_rejection() {
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, block_key, local_key};
    // Fixed two source-native layers straddle the actual fine-cell boundary in Z.
    // Their 2 mm separation produces a legal apparent slope over 5 mm of XY;
    // only their distinct actual selected records can exclude continuity.
    let base = layered_map(&[[0, 0], [1, 1]]);
    let mut observations = base.observations.as_ref().unwrap().as_ref().clone();
    let fine = f64::from(observations.size_m);
    let mut blocks = std::collections::BTreeMap::<[i32; 3], LayerObservations>::new();
    for x in -12..12 { for y in -12..12 { for (z, height) in [(0, 0.049), (1, 0.051)] {
        let key = [x, y, z];
        let local = local_key(key);
        let origin = [(f64::from(x) + 0.5) * fine, (f64::from(y) + 0.5) * fine, height];
        assert_eq!((height / fine).floor() as i32, z);
        blocks.entry(block_key(key)).or_default().insert(local, LayerObservation {
            members: vec![local], normal: [0.0, 0.0, 1.0], plane_origin: origin,
            samples: vec![LayerSample { source: Default::default(), point: origin,
                source_sequence: 1, source_timestamp_ns: 100, point_index: u32::from(local) }],
        });
    } } }
    for (block, layers) in blocks { observations.replace_ground_layers(block, Arc::new(layers)).unwrap(); }
    observations.mark_complete_geometry_source();
    let map = base.with_observations(Some(Arc::new(observations)));
    let make_sample = |xy: [f64; 2], height: f64, anchor: [i32; 3]| {
        let (query, native) = map.query_ground_with_native(xy, height);
        assert_eq!(query.state, GroundQueryState::Observed);
        assert_eq!(query.height, Some(height));
        assert_eq!(native, vec![NativeGroundSource::ObservedFine { anchor }]);
        GroundSample { reference_origin: [xy[0], xy[1], height], reference_normal: query.normal,
            query_state: query.state, reference_sources: native }
    };
    let lower = make_sample([0.01, 0.025], 0.049, [0, 0, 0]);
    let upper = make_sample([0.02, 0.025], 0.051, [0, 0, 1]);
    let reference = BodyQueryReference::Ground(Arc::new(LocalGroundReference {
        first: lower, changes: vec![(1.0, upper)] }));
    let view = map.bind_body_query(reference);
    let samples = [[0.01, 0.025, 0.049], [0.015, 0.025, 0.049], [0.02, 0.025, 0.051]];
    let selected: Vec<_> = samples.iter().map(|p| map.known_terrain_transition_sample([p[0], p[1]], p[2]).unwrap()).collect();
    for (point, (height, layer)) in samples.iter().zip(&selected) {
        let layer = layer.unwrap();
        eprintln!("C12_DISTINCT source_point={point:?} selected_height={height:?} anchor={:?} members={:?}", layer.anchor, layer.members);
        assert_eq!(point[..2].iter().map(|v| (v / fine).floor() as i32).collect::<Vec<_>>(), vec![0, 0]);
        assert_eq!(*height, point[2]);
    }
    assert!(std::ptr::eq(selected[0].1.unwrap(), selected[1].1.unwrap()));
    assert!(!std::ptr::eq(selected[1].1.unwrap(), selected[2].1.unwrap()));
    assert_eq!(selected[1].1.unwrap().anchor, [0, 0, 0]);
    assert_eq!(selected[2].1.unwrap().anchor, [0, 0, 1]);
    let run = samples[2][0] - samples[1][0];
    let rise = selected[2].0 - selected[1].0;
    assert!(run > 0.0 && rise > 1e-6);
    assert!(rise <= run * f64::from(map.config().max_slope_deg).to_radians().tan() + 1e-6);
    let envelope = map.body_envelope().unwrap();
    let from = ([0.01, 0.025, 0.049 + envelope.body_to_support_z_m], [0.0, 0.0, 0.0, 1.0]);
    let to = ([0.02, 0.025, 0.051 + envelope.body_to_support_z_m], from.1);
    let mut witness = None;
    let from_result = (view.pose_at(from, 0.0, &mut witness), witness);
    let to_result = (view.pose_at(to, 1.0, &mut witness), witness);
    let projection = (view.project_from_to_samples(from, [0.02, 0.025], 0.0, [0.0, 1.0], &mut witness), witness);
    let motion = (view.motion_on_segment(from, to, [0.0, 1.0], &[], &mut witness), witness);
    let old_edge = map.validate_known_reference_samples(samples, fine,
        envelope.max_step_up_m, envelope.max_step_down_m, envelope.support_uncertainty_m);
    let ground_terrain = map.validate_known_terrain_motion_in_context(from, to,
        view.segment_for_diagnostic([0.0, 1.0]));
    eprintln!("C12_DISTINCT run={run:?} rise={rise:?} from={from:?} to={to:?} from_result={from_result:?} to_result={to_result:?} projection={projection:?} motion={motion:?} old_edge={old_edge:?} ground_terrain={ground_terrain:?}");
    assert_eq!(from_result, (Ok(()), None));
    assert_eq!(to_result, (Ok(()), None));
    assert_eq!(old_edge, Err(SurfaceError::UntraversableSegment));
    assert_eq!(ground_terrain, Err(SurfaceError::UntraversableSegment), "direct real Ground terrain kernel, independent of body-entry rejection order");
    assert_eq!(projection, (Err(SurfaceError::UntraversableSegment), None));
    assert_eq!(motion, (Err(SurfaceError::UntraversableSegment), None));
}
