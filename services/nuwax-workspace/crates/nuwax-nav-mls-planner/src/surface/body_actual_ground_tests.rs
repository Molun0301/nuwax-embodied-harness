//! Actual observations may change the associated floor; predictions may not.
//! Unknown actual columns retain the last real h instead of reseeding from body Z.
use super::*;
use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample,block_key,local_key};
use std::collections::BTreeMap;

type Pose=([f64;3],[f64;4]);
const Q:f32=0.125;
const IDENTITY:[f64;4]=[0.0,0.0,0.0,1.0];

fn scene(levels:impl Fn(i32)->Vec<i32>,measured:bool)->SurfaceSnapshot {
    let mut points=Vec::new();
    let mut blocks=BTreeMap::<[i32;3],BTreeMap<[i32;3],SurfaceObservation>>::new();
    let mut layers=BTreeMap::<[i32;3],LayerObservations>::new();
    for x in 0..16 {for y in 0..8 {for z in levels(x) {
        let key=[x,y,z];let local=local_key(key);
        let origin=key.map(|v|(f64::from(v)+0.5)*f64::from(Q));
        points.push((origin[0] as f32,origin[1] as f32,origin[2] as f32));
        blocks.entry(block_key(key)).or_default().insert(key,SurfaceObservation{observed_ns:100,hits:3});
        if measured {
            layers.entry(block_key(key)).or_default().insert(local,LayerObservation {
                members:vec![local],normal:[0.0,0.0,1.0],plane_origin:origin,
                samples:(0..3).map(|i|LayerSample {source:Default::default(),point:origin,
                    source_sequence:i+1,source_timestamp_ns:100+i,point_index:u32::from(local)}).collect(),
            });
        }
    }}}
    let config=Config {world_frame:"map".into(),voxel_size:0.25,robot_height:0.2,
        surface_closing_radius:0.0,node_spacing_m:0.5,wall_clearance_m:0.0,
        wall_buffer_m:0.0,wall_buffer_weight:0.0,max_slope_deg:18.0,
        step_threshold_m:0.2,min_step_tread_depth_m:0.5,step_penalty_weight:1.0,
        goal_tolerance:0.1,viz_publish_hz:0.0};
    let model=SurfaceBodyEnvelope {fixed_collision:None,
        footprint_xy_m:vec![[-0.08,-0.06],[0.08,-0.06],[0.08,0.06],[-0.08,0.06]],
        z_bounds_m:[-0.1,0.1],body_to_support_z_m:0.4,max_step_up_m:0.2,max_step_down_m:0.2,
        max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
    let mut observations=SurfaceObservations::new(Q,32).unwrap();
    for (block,values) in blocks {observations.replace_block(block,values).unwrap();}
    for (block,values) in layers {observations.replace_ground_layers(block,Arc::new(values)).unwrap();}
    observations.mark_complete_geometry_source();
    SurfaceSnapshot::from_points(&points,config,Q).unwrap().with_body_envelope(model).unwrap()
        .with_observations(Some(Arc::new(observations)))
}

#[test]
fn actual_known_floor_reassociation_does_not_grant_the_predicted_floor_transfer() {
    for measured in [false,true] {
        let map=scene(|_|vec![0,12],measured);let owner=map.body_query();
        let lower=([0.5625,0.4375,0.5],IDENTITY);
        let upper=([0.8125,0.4375,2.0],IDENTITY);
        let initial=owner.start_ground_state(lower).unwrap();let saved=initial.clone();
        let actual=owner.observe_actual_ground(lower,&initial,upper).unwrap();
        let fresh=owner.start_ground_state(upper).unwrap();
        assert_eq!(actual,fresh);
        assert!((actual.height()-initial.height()-1.5).abs()<1e-12);
        assert!((actual.height()-(upper.0[2]-0.4)).abs()>0.01,
            "actual support Z selects a floor but must not replace its source h");
        assert_eq!(actual.query_state(),if measured {GroundQueryState::Observed}else{GroundQueryState::LegacyGeometry});
        if measured {assert_eq!(actual.ground_key(),Some(GroundKey::LegacyPlane([6,3,12])));}
        owner.validate_motion_from_ground(upper,upper,&actual,&[],&mut None).unwrap();
        let returned=owner.observe_actual_ground(upper,&actual,lower).unwrap();
        assert_eq!(returned,initial);
        assert_eq!(initial,saved,"actual reassociation cannot mutate the caller's old state");
        let predicted=owner.project_motion(lower,&initial,[upper.0[0],upper.0[1]],0.0,&mut None).unwrap();
        assert!((predicted.ground().height()-initial.height()).abs()<1e-12);
        assert!((predicted.pose().0[2]-lower.0[2]).abs()<1e-12);
        assert!(owner.validate_motion_from_ground(lower,upper,&initial,&[],&mut None).is_err(),
            "reassociation does not authorize a predicted 1.5 m transfer through floors");
    }
}

#[test]
fn actual_unknown_bias_keeps_last_source_height_and_preserves_real_tilt() {
    for measured in [false,true] {
        let map=scene(|_|vec![0],measured);let owner=map.body_query();
        let known=([1.8125,0.4375,0.5],IDENTITY);
        let initial=owner.start_ground_state(known).unwrap();
        let angle=0.12_f64;
        let unknown:Pose=([2.4375,0.4375,0.77],[(angle*0.5).sin(),0.0,0.0,(angle*0.5).cos()]);
        let fresh=owner.start_ground_state(unknown).unwrap();
        assert_eq!(fresh.query_state(),GroundQueryState::Unknown);
        assert!((fresh.height()-initial.height()).abs()>0.1,"fixture distinguishes a body-Z seed");
        let carried=owner.observe_actual_ground(known,&initial,unknown).unwrap();
        assert_eq!(carried.query_state(),GroundQueryState::Unknown);
        assert_eq!(carried.height().to_bits(),initial.height().to_bits());
        if measured {assert!(carried.ground_key().is_some(),"last measured source identity remains carried");}
        let next=owner.project_motion(unknown,&carried,[known.0[0],known.0[1]],0.0,&mut None).unwrap();
        assert!((next.pose().0[2]-unknown.0[2]).abs()<1e-12);
        let before=rotation(unknown.1).unwrap();let after=rotation(next.pose().1).unwrap();
        assert!(before.iter().flatten().zip(after.iter().flatten()).all(|(a,b)|(a-b).abs()<1e-12));
        assert_eq!(carried.query_state(),GroundQueryState::Unknown,"prediction does not consume actual state");
    }
}

#[test]
fn actual_unknown_after_crossing_known_step_carries_the_last_crossed_height() {
    for measured in [false,true] {
        let map=scene(|x|vec![if x<8 {0}else{1}],measured);let owner=map.body_query();
        let from=([0.5625,0.4375,0.5],IDENTITY);
        let actual=([2.4375,0.4375,0.75],IDENTITY);
        let initial=owner.start_ground_state(from).unwrap();
        let crossed=owner.start_ground_state(([1.5625,0.4375,0.65],IDENTITY)).unwrap();
        let fresh=owner.start_ground_state(actual).unwrap();
        assert_eq!(fresh.query_state(),GroundQueryState::Unknown);
        assert_ne!(fresh.height().to_bits(),crossed.height().to_bits());
        let observed=owner.observe_actual_ground(from,&initial,actual).unwrap();
        assert_eq!(observed.query_state(),GroundQueryState::Unknown);
        assert_eq!(observed.height().to_bits(),crossed.height().to_bits());
        assert_ne!(observed.height().to_bits(),initial.height().to_bits(),
            "the real trajectory crossed a known step before its final unknown column");
    }
}

#[test]
fn actual_known_reassociation_does_not_make_an_incompatible_high_body_valid() {
    for measured in [false,true] {
        let map=scene(|_|vec![0],measured);let owner=map.body_query();
        let from=([0.5625,0.4375,0.5],IDENTITY);
        let high=([0.8125,0.4375,2.0],IDENTITY);
        let initial=owner.start_ground_state(from).unwrap();
        let actual=owner.observe_actual_ground(from,&initial,high).unwrap();
        assert_eq!(actual.height().to_bits(),initial.height().to_bits());
        assert_ne!(actual.query_state(),GroundQueryState::Unknown);
        assert_eq!(owner.validate_motion_from_ground(high,high,&actual,&[],&mut None),
            Err(SurfaceError::UntraversableSegment));
    }
}
