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
        let key=[x,y,-1];let local=local_key(key);
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
