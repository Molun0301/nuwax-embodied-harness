// Small synthetic fine-source counterexamples shared by MLS 5 cm and 10 cm.
use super::*;

fn map_at(points:&[(f32,f32,f32)],observed:&Arc<SurfaceObservations>,envelope:&SurfaceBodyEnvelope,mls:f32)->SurfaceSnapshot {
    let mut cfg=scene(&[],false).config().clone();
    cfg.voxel_size=mls;cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
    SurfaceSnapshot::from_points(points,cfg,0.05).unwrap()
        .with_body_envelope(envelope.clone()).unwrap().with_observations(Some(observed.clone()))
}

#[test]
fn unassigned_complete_fine_coverage_is_not_clipped_to_the_body_polygon() {
    let mut envelope=model();
    envelope.footprint_xy_m=vec![[-0.025,-0.025],[0.025,-0.025],[0.025,0.025],[-0.025,0.025]];
    envelope.z_bounds_m=[-0.05,0.05];
    let body=[0.1,0.1,0.25];
    let r=rotation([0.0,0.0,0.0,1.0]).unwrap();
    let boundary=[[0.075,0.075],[0.125,0.075],[0.125,0.125],[0.075,0.125]];
    let polygon=PolygonQuery::new(&boundary);
    let check=|keys:Vec<[i32;3]>,complete:bool| {
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
            .insert(key,SurfaceObservation{observed_ns:1,hits:1});}
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        if complete {observations.mark_complete_geometry_source();}
        let map=map_at(&[(0.025,0.025,0.225)],&Arc::new(observations),&envelope,0.1);
        let prism=PrismQuery::new(map.body_envelope().unwrap(),r,body);
        let mut witness=None;
        let result=map.check_unassigned_body_column(body,r,&polygon,&prism,0,0,&mut witness);
        (result,witness)
    };
    let outside=check(vec![[0,0,4]],true);
    assert_eq!(outside.0,Ok(()),"粗格虽相交，其完整细占据都在机体外，不得再次膨胀粗格：{:?}",outside.1);
    assert!(outside.1.is_none());
    let inside=check(vec![[1,1,4]],true);
    assert_eq!(inside.0,Err(SurfaceError::InsufficientClearance));
    assert!(matches!(inside.1,Some(BodyRejection::Occupancy{source:OccupancySource::Fine,..})));
    for (keys,complete) in [(vec![[0,0,4]],false),(vec![[0,0,2]],true),(vec![],true)] {
        let missing=check(keys,complete);
        assert_eq!(missing.0,Err(SurfaceError::InsufficientClearance),"部分细源或缺对应高度不能当自由空间");
        assert!(matches!(missing.1,Some(BodyRejection::Occupancy{source:OccupancySource::CoarseFallback,..})));
    }
}

#[test]
fn partial_fine_source_cannot_alias_a_high_echo_to_coarse_ground() {
    let mut points:Vec<_>=(-30..30).flat_map(|x|(-20..20).map(move|y|
        ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,0.025))).collect();
    points.push((0.575,0.025,0.175));
    let observed=witnessed_observations(&[(0.575,0.025,0.175)],0.05);
    let mut envelope=model();envelope.z_bounds_m=[-0.448,0.057];
    envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;envelope.max_cross_slope_deg=18.0;
    for mls in [0.05,0.1] {
        let map=map_at(&points,&observed,&envelope,mls);
        assert_eq!(map.validate_body_pose([0.05,0.05,0.375],[0.0,0.0,0.0,1.0]),
            Err(SurfaceError::InsufficientClearance),"MLS={mls}");
    }
}

#[test]
fn fine_fringe_witness_survives_resolution_and_query_cache_changes() {
    let mut points:Vec<_>=(-30..30).flat_map(|x|(-30..30).map(move|y|
        ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,0.075))).collect();
    points.push((0.075,0.025,0.525));
    let observed=witnessed_observations(&[(0.075,0.025,0.525)],0.05);
    for mls in [0.05,0.1] {
        let map=map_at(&points,&observed,&model(),mls);
        let pose=([f64::from(0.1_f32)+0.55,0.05,0.5],[0.0,0.0,0.0,1.0]);let mut witness=None;
        assert_eq!(map.validate_body_pose_with_witness(pose.0,pose.1,&mut witness),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(witness.unwrap().rejection,
            BodyRejection::Occupancy{source:OccupancySource::FineFringe,cell:[1,0,10],..}),"MLS={mls}");
        assert_witness_cold_and_warm(&map,pose,witness);
    }
}

#[test]
fn same_fine_source_rigid_obstacle_blocks_pose_and_full_sweep() {
    let mut points:Vec<_>=(-30..30).flat_map(|x|(-20..20).flat_map(move|y|
        [-0.025,0.025].map(move|z|((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,z)))).collect();
    points.push((0.075,0.075,0.275));let observed=witnessed_observations(&points,0.05);
    let mut envelope=model();envelope.footprint_xy_m=vec![[-0.1,-0.08],[0.1,-0.08],[0.1,0.08],[-0.1,0.08]];
    let q=[0.0,0.0,0.0,1.0];let from=([-0.65,0.05,0.375],q);let to=([0.65,0.05,0.375],q);
    for mls in [0.05,0.1] {
        let map=map_at(&points,&observed,&envelope,mls);
        assert_eq!(map.validate_body_pose([0.05,0.05,0.375],q),Err(SurfaceError::InsufficientClearance));
        map.validate_body_pose(from.0,q).unwrap();map.validate_body_pose(to.0,q).unwrap();
        let mut witness=None;
        assert_eq!(map.validate_body_motion_with_witness(from,to,&[],&mut witness),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(witness.unwrap().rejection,
            BodyRejection::Occupancy{cell:[1,1,5],source:OccupancySource::Fine|OccupancySource::FineFringe,..}),"MLS={mls}");
    }
}

#[test]
fn solid_volume_keeps_its_exposed_top_at_both_resolutions() {
    let points:Vec<_>=(-20..20).flat_map(|x|(-10..10).flat_map(move|y|
        (-4..0).map(move|z|((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05)))).collect();
    let observed=witnessed_observations(&points,0.05);
    let mut envelope=model();envelope.footprint_xy_m=vec![[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]];
    envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;envelope.max_cross_slope_deg=18.0;
    for mls in [0.05,0.1] {
        let map=map_at(&points,&observed,&envelope,mls).with_ground_evidence(GroundEvidence::OccupiedVolume);
        map.validate_body_pose([0.025,0.025,0.4],[0.0,0.0,0.0,1.0]).unwrap();
        let bands=map.fine_contact_bands(0,0);
        assert_eq!(bands.len(),1);assert_eq!(bands[0].fit,[0.0,0.0]);assert_eq!(bands[0].run,[-4,-1]);
        for z in -4..0 {assert!(map.observations.as_ref().unwrap().get([0,0,z]).is_some());}
    }
}

#[test]
    fn partial_footprint_contact_uses_actual_overlap_not_outside_grid_centre_mls5() {
        let mut points:Vec<_>=(-20..20).flat_map(|x|(-20..20).flat_map(move |y|
            [0.025,0.075].map(|z|((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,z)))).collect();
        points.push((0.025,-0.025,0.175));
        let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;cfg.voxel_size=0.05;
        let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.2,-0.1],[0.2,-0.1],[0.2,0.1],[-0.2,0.1]],
            z_bounds_m:[-0.4,0.1],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation {observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let map=map.with_observations(Some(Arc::new(observations)));
        let tilt=0.1_f64;let q=[(tilt*0.5).sin(),0.0,0.0,(tilt*0.5).cos()];
        let height=0.2-(0.15+f64::from(0.05_f32))-0.0005+0.4/tilt.cos()-tilt.tan()*(-0.05-0.075);
        let body=[0.025,0.075,height];
        map.validate_body_pose(body,q).expect("粗列中心在机体投影外，实际相交区域仍在原接触行程内");
        assert_eq!(map.validate_body_pose([body[0],body[1],body[2]-0.002],q),Err(SurfaceError::InsufficientClearance),
            "真实交叠区域超过同一行程仍须拒绝");
    }

#[test]
    fn fine_ground_height_and_contact_plane_use_the_same_spatial_column_mls5() {
        let mut points=Vec::new();
        for x in -30..30 {for y in -20..20 {for z in [0.025,0.075] {
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,z));
        }}}
        // 较高地面只在支撑平面较高的半列，不能与另一半列的最低支撑值配对。
        for x in [0,1] {for z in [0.125,0.175] {points.push(((x as f32+0.5)*0.05,0.075,z));}}
        let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.05;cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        envelope.max_cross_slope_deg=18.0;
        let base=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let make=|missing:bool| {
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                if missing && key[0]==0 && key[1]==0 {continue;}
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
            let mut observations=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
            base.with_observations(Some(Arc::new(observations)))
        };
        let angle=0.1_f64;let q=[(angle*0.5).sin(),0.0,0.0,(angle*0.5).cos()];
        let body=[0.05,-0.06,0.4/angle.cos()-0.002-0.06*angle.tan()];
        let map=make(false);
        map.validate_body_pose(body,q).expect("每个真实细列的地面都满足该列原接触范围");
        assert_eq!(map.validate_body_pose([body[0],body[1],body[2]-0.004],q),Err(SurfaceError::InsufficientClearance),
            "同一真实高细列超过接触范围仍应拒绝");
        let mut slot=None;let rejected=[body[0],body[1],body[2]-0.004];
        assert!(map.validate_body_pose_with_witness(rejected,q,&mut slot).is_err());
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{..}));
    }
