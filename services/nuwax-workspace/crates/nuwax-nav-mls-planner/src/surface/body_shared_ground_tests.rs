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
